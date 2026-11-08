// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK3026 EBC: 32-bit RGB and R8 to packed 4bpp grey with NEON.
 *
 * Built with CC_FLAGS_FPU, so the compiler may use NEON anywhere in this
 * unit: nothing here may run outside a kernel-mode NEON section.
 * rk3026_ebc_blit_convert() in rk3026_ebc_blit.c, a unit without FPU flags,
 * calls it in bands of rows inside scoped_ksimd(). <arm_neon.h> brings the
 * <stdint.h> types, which clash with <linux/types.h> on ARM, so this unit
 * includes no kernel header (as lib/raid6/recov_neon_inner.c).
 *
 * rk3026_ebc_blit_rows_neon() takes the DRM format of the source, as
 * rk3026_ebc_blit_rows() does, and writes its bytes. For XRGB8888 the
 * nibble is the top four bits of the BT.601 luma, (77 R + 150 G + 29 B) /
 * 256 rounded down. Per 16 pixels: vld4.8 splits B, G, R and X into
 * planes, vmull.u8/vmlal.u8 sum the luma into u16 lanes, vshrn #8 narrows
 * it to a byte, vuzp.8 separates the even and the odd pixels, and vsri.8
 * #4 packs each pair into one byte with the even pixel in the low nibble.
 * The source is read once and nothing is tested, so every page costs about
 * the same; on the RK3026 (Cortex-A9 at 600 MHz, with the L2 cache
 * controller's prefetch and line-fill options set as the vendor firmware
 * sets them) that is the memory floor. ARGB8888 takes the same rows;
 * XBGR8888 and ABGR8888 the same with the planes of R and B exchanged at
 * the multiplies, which costs nothing.
 *
 * For R8 each grey is rounded to the nearest level as (g * 15 + 135) >> 8
 * (rk3026_ebc_grey_level()). Per 32 pixels: vld2.8 splits the even and the
 * odd pixels at the load, vmull.u8 by 15 and vaddhn.u16 with 135 give the
 * levels of 8 pixels in two instructions, and vsli.8 #4 packs each pair
 * into one byte with the even pixel in the low nibble; one 16-byte store.
 *
 * rk3026_ebc_blit_rows_dither_neon() writes the bytes of
 * rk3026_ebc_blit_rows_dither() the same way: the grey of 16 pixels in two
 * u8 vectors, the XRGB8888 luma as above or the R8 bytes as loaded, and
 * the dither's few operations per pixel on u16 lanes.
 */

#include <arm_neon.h>
#include <stddef.h>

#include "rk3026_ebc_blit_neon.h"

/*
 * The grey nibble of the 32-bit pixel at @s whose byte @r is R and byte
 * 2 - @r is B: @r = 2 for XRGB8888 (B, G, R, X in memory), 0 for XBGR8888
 * (R, G, B, X).
 */
static __always_inline unsigned char ebc_neon_nibble(const unsigned char *s,
						     int r)
{
	return (77 * s[r] + 150 * s[1] + 29 * s[2 - r]) >> 12;
}

/*
 * Sixteen 32-bit pixels at @s into eight bytes, pixel 0 in the low nibble;
 * @r as for ebc_neon_nibble(), the plane of R after vld4.8.
 */
static __always_inline uint8x8_t ebc_neon_pack16(const unsigned char *s,
						 int r)
{
	const uint8x8_t k77 = vdup_n_u8(77);
	const uint8x8_t k150 = vdup_n_u8(150);
	const uint8x8_t k29 = vdup_n_u8(29);
	uint8x16x4_t v = vld4q_u8(s);	/* the bytes of a pixel, G in val[1] */
	uint16x8_t lo, hi;
	uint8x8x2_t eo;

	lo = vmull_u8(vget_low_u8(v.val[r]), k77);
	hi = vmull_u8(vget_high_u8(v.val[r]), k77);
	lo = vmlal_u8(lo, vget_low_u8(v.val[1]), k150);
	hi = vmlal_u8(hi, vget_high_u8(v.val[1]), k150);
	lo = vmlal_u8(lo, vget_low_u8(v.val[2 - r]), k29);
	hi = vmlal_u8(hi, vget_high_u8(v.val[2 - r]), k29);
	eo = vuzp_u8(vshrn_n_u16(lo, 8), vshrn_n_u16(hi, 8));	/* even, odd */
	return vsri_n_u8(eo.val[1], eo.val[0], 4);
}

/*
 * The 32-bit rows, @r as for ebc_neon_nibble(): sixteen pixels at a time,
 * with the source read 256 bytes ahead (pld, as prefetch()); a row's last
 * 2 to 14 pixels take the luma two at a time.
 */
static __always_inline void ebc_neon_rgb32_rows(unsigned char *dst,
						const void *src,
						unsigned int src_pitch, int x1,
						int x2, int y1, int y2,
						unsigned int width, int r)
{
	int y;

	for (y = y1; y < y2; y++) {
		const unsigned char *s = (const unsigned char *)src +
				   (size_t)y * src_pitch + x1 * 4;
		unsigned char *d = dst + ((size_t)y * width + x1) / 2;
		int n = x2 - x1;

		for (; n >= 16; n -= 16, s += 64, d += 8) {
			__builtin_prefetch(s + 256);
			__builtin_prefetch(s + 256 + 32);
			vst1_u8(d, ebc_neon_pack16(s, r));
		}
		for (; n > 0; n -= 2, s += 8, d++)
			*d = ebc_neon_nibble(s, r) |
			     ebc_neon_nibble(s + 4, r) << 4;
	}
}

/* The level of the 8-bit grey @g, as rk3026_ebc_grey_level(). */
static __always_inline unsigned char ebc_neon_grey(unsigned char g)
{
	return (g * 15 + 135) >> 8;
}

/* The levels of eight 8-bit greys, (g * 15 + 135) >> 8 in each lane. */
static __always_inline uint8x8_t ebc_neon_grey8(uint8x8_t g)
{
	return vaddhn_u16(vmull_u8(g, vdup_n_u8(15)), vdupq_n_u16(135));
}

/*
 * The R8 rows: thirty-two pixels at a time, with the source read 256 bytes
 * ahead, one prefetch per 32-byte line; then 16 if as many are left, and a
 * row's last 2 to 14 pixels two at a time.
 */
static __always_inline void ebc_neon_r8_rows(unsigned char *dst,
					     const void *src,
					     unsigned int src_pitch, int x1,
					     int x2, int y1, int y2,
					     unsigned int width)
{
	int y;

	for (y = y1; y < y2; y++) {
		const unsigned char *s = (const unsigned char *)src +
				   (size_t)y * src_pitch + x1;
		unsigned char *d = dst + ((size_t)y * width + x1) / 2;
		int n = x2 - x1;

		for (; n >= 32; n -= 32, s += 32, d += 16) {
			/* val[0] the even pixels, val[1] the odd */
			uint8x16x2_t g = vld2q_u8(s);
			uint8x8_t e0 = ebc_neon_grey8(vget_low_u8(g.val[0]));
			uint8x8_t e1 = ebc_neon_grey8(vget_high_u8(g.val[0]));
			uint8x8_t o0 = ebc_neon_grey8(vget_low_u8(g.val[1]));
			uint8x8_t o1 = ebc_neon_grey8(vget_high_u8(g.val[1]));

			__builtin_prefetch(s + 256);
			vst1q_u8(d, vsliq_n_u8(vcombine_u8(e0, e1),
					       vcombine_u8(o0, o1), 4));
		}
		if (n >= 16) {
			uint8x8x2_t g = vld2_u8(s);

			vst1_u8(d, vsli_n_u8(ebc_neon_grey8(g.val[0]),
					     ebc_neon_grey8(g.val[1]), 4));
			n -= 16;
			s += 16;
			d += 8;
		}
		for (; n > 0; n -= 2, s += 2, d++)
			*d = ebc_neon_grey(s[0]) | ebc_neon_grey(s[1]) << 4;
	}
}

/**
 * rk3026_ebc_blit_rows_neon - rk3026_ebc_blit_rows() with NEON
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: source, top-left of the whole framebuffer
 * @src_pitch: source stride in bytes
 * @format: source format: EBC_NEON_XRGB8888, EBC_NEON_ARGB8888,
 *	EBC_NEON_XBGR8888, EBC_NEON_ABGR8888 or EBC_NEON_R8
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 *
 * Writes the same bytes as rk3026_ebc_blit_rows() for @format, and only
 * those; any other format writes nothing. No alignment is needed.
 *
 * Call only between kernel_neon_begin() and kernel_neon_end().
 */
void rk3026_ebc_blit_rows_neon(unsigned char *dst, const void *src,
			       unsigned int src_pitch, unsigned int format,
			       int x1, int x2, int y1, int y2,
			       unsigned int width)
{
	switch (format) {
	case EBC_NEON_XRGB8888:
	case EBC_NEON_ARGB8888:
		ebc_neon_rgb32_rows(dst, src, src_pitch, x1, x2, y1, y2, width,
				    2);
		break;
	case EBC_NEON_XBGR8888:
	case EBC_NEON_ABGR8888:
		ebc_neon_rgb32_rows(dst, src, src_pitch, x1, x2, y1, y2, width,
				    0);
		break;
	case EBC_NEON_R8:
		ebc_neon_r8_rows(dst, src, src_pitch, x1, x2, y1, y2, width);
		break;
	}
}

/* The bytes per pixel of a dither source @format: XRGB8888 or R8. */
static __always_inline unsigned int ebc_neon_dither_cpp(unsigned int format)
{
	return format == EBC_NEON_XRGB8888 ? 4 : 1;
}

/*
 * The 8-bit grey of the pixel at @s, as rk3026_ebc_blit_rows_dither()
 * takes it: the luma of an XRGB8888 pixel, the byte of an R8 one.
 */
static __always_inline unsigned int ebc_neon_grey_of(const unsigned char *s,
						     unsigned int format)
{
	return format == EBC_NEON_XRGB8888 ?
	       (77 * s[2] + 150 * s[1] + 29 * s[0]) >> 8 : s[0];
}

/* One dithered nibble: (@l * @steps + @t) / 255, times 15 / @steps. */
static __always_inline unsigned char ebc_neon_dither1(unsigned int l,
						      unsigned int t,
						      unsigned int steps)
{
	unsigned int v = l * steps + t;

	return ((v + (v >> 8) + 1) >> 8) * (15 / steps);
}

/*
 * Sixteen pixels at @s into eight dithered bytes, pixel 0 in the low
 * nibble; @t holds the thresholds of the sixteen columns. The greys L in
 * two u8 vectors (the luma as in ebc_neon_pack16(), narrowed to its byte,
 * or the R8 bytes); v = L * @steps + t in u16 lanes; the level
 * (v + (v >> 8) + 1) >> 8, narrowed again; times 15 for one step; vuzp.8
 * and vsli.8 #4 pack the pairs.
 */
static __always_inline uint8x8_t ebc_neon_dither16(const unsigned char *s,
						   uint8x16_t t,
						   unsigned int format,
						   unsigned int steps)
{
	const uint16x8_t one = vdupq_n_u16(1);
	uint16x8_t lo, hi;
	uint8x8_t llo, lhi;
	uint8x8x2_t eo;

	if (format == EBC_NEON_XRGB8888) {
		uint8x16x4_t v = vld4q_u8(s);	/* B, G, R, X */

		lo = vmull_u8(vget_low_u8(v.val[2]), vdup_n_u8(77));
		hi = vmull_u8(vget_high_u8(v.val[2]), vdup_n_u8(77));
		lo = vmlal_u8(lo, vget_low_u8(v.val[1]), vdup_n_u8(150));
		hi = vmlal_u8(hi, vget_high_u8(v.val[1]), vdup_n_u8(150));
		lo = vmlal_u8(lo, vget_low_u8(v.val[0]), vdup_n_u8(29));
		hi = vmlal_u8(hi, vget_high_u8(v.val[0]), vdup_n_u8(29));
		llo = vshrn_n_u16(lo, 8);
		lhi = vshrn_n_u16(hi, 8);
	} else {
		uint8x16_t g = vld1q_u8(s);

		llo = vget_low_u8(g);
		lhi = vget_high_u8(g);
	}
	if (steps == 1) {
		lo = vaddl_u8(llo, vget_low_u8(t));
		hi = vaddl_u8(lhi, vget_high_u8(t));
	} else {
		lo = vmlal_u8(vmovl_u8(vget_low_u8(t)), llo, vdup_n_u8(15));
		hi = vmlal_u8(vmovl_u8(vget_high_u8(t)), lhi, vdup_n_u8(15));
	}
	lo = vsraq_n_u16(vaddq_u16(lo, one), lo, 8);
	hi = vsraq_n_u16(vaddq_u16(hi, one), hi, 8);
	llo = vshrn_n_u16(lo, 8);
	lhi = vshrn_n_u16(hi, 8);
	if (steps == 1) {
		llo = vmul_u8(llo, vdup_n_u8(15));
		lhi = vmul_u8(lhi, vdup_n_u8(15));
	}
	eo = vuzp_u8(llo, lhi);	/* even, odd */
	return vsli_n_u8(eo.val[0], eo.val[1], 4);
}

/* rk3026_ebc_blit_rows_dither_neon() for a constant @format and @steps. */
static __always_inline void ebc_neon_dither_rows(unsigned char *dst,
						 const void *src,
						 unsigned int src_pitch,
						 unsigned int format,
						 int x1, int x2, int y1,
						 int y2, unsigned int width,
						 const unsigned char *thr,
						 unsigned int steps)
{
	const unsigned int cpp = ebc_neon_dither_cpp(format);
	const uint8x8_t lanes = vcreate_u8(0x0706050403020100ULL);
	/* Lane j takes column (x1 + j) mod 8 of a threshold row. */
	const uint8x8_t idx = vand_u8(vadd_u8(vdup_n_u8(x1 & 7), lanes),
				      vdup_n_u8(7));
	int x, y;

	for (y = y1; y < y2; y++) {
		const unsigned char *s = (const unsigned char *)src +
				   (size_t)y * src_pitch + x1 * cpp;
		const unsigned char *t = thr + (y & 7) * 8;
		unsigned char *d = dst + ((size_t)y * width + x1) / 2;
		uint8x8_t t8 = vtbl1_u8(vld1_u8(t), idx);
		uint8x16_t t16 = vcombine_u8(t8, t8);

		/* Blocks of 16 keep the phase of the 8-px threshold row. */
		for (x = x1; x2 - x >= 16; x += 16, s += 16 * cpp, d += 8) {
			__builtin_prefetch(s + 256);
			if (cpp == 4)
				__builtin_prefetch(s + 256 + 32);
			vst1_u8(d, ebc_neon_dither16(s, t16, format, steps));
		}
		for (; x < x2; x += 2, s += 2 * cpp, d++)
			*d = ebc_neon_dither1(ebc_neon_grey_of(s, format),
					      t[x & 7], steps) |
			     ebc_neon_dither1(ebc_neon_grey_of(s + cpp, format),
					      t[(x + 1) & 7], steps) << 4;
	}
}

/**
 * rk3026_ebc_blit_rows_dither_neon - rk3026_ebc_blit_rows_dither() with NEON
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: source, top-left of the whole framebuffer
 * @src_pitch: source stride in bytes
 * @format: source format, EBC_NEON_XRGB8888 or EBC_NEON_R8
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 * @thr: the 8x8 threshold matrix, row-major
 * @steps: 15 to dither to the 16 levels, 1 to black and white
 *
 * Writes the same bytes as rk3026_ebc_blit_rows_dither() with the same
 * thresholds, and only those; any other format writes nothing. Sixteen
 * pixels at a time, with the source read 256 bytes ahead; a row's last 2
 * to 14 pixels two at a time. No alignment is needed.
 *
 * Call only between kernel_neon_begin() and kernel_neon_end().
 */
void rk3026_ebc_blit_rows_dither_neon(unsigned char *dst, const void *src,
				      unsigned int src_pitch,
				      unsigned int format, int x1, int x2,
				      int y1, int y2, unsigned int width,
				      const unsigned char *thr,
				      unsigned int steps)
{
	switch (format) {
	case EBC_NEON_XRGB8888:
		if (steps == 1)
			ebc_neon_dither_rows(dst, src, src_pitch,
					     EBC_NEON_XRGB8888, x1, x2, y1, y2,
					     width, thr, 1);
		else
			ebc_neon_dither_rows(dst, src, src_pitch,
					     EBC_NEON_XRGB8888, x1, x2, y1, y2,
					     width, thr, 15);
		break;
	case EBC_NEON_R8:
		if (steps == 1)
			ebc_neon_dither_rows(dst, src, src_pitch, EBC_NEON_R8,
					     x1, x2, y1, y2, width, thr, 1);
		else
			ebc_neon_dither_rows(dst, src, src_pitch, EBC_NEON_R8,
					     x1, x2, y1, y2, width, thr, 15);
		break;
	}
}
