// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK3026 EBC: 32-bit RGB and 8-bit grey to the panel's packed
 * 4bpp grey.
 *
 * Every plane and mxcfb update converts its damage here, under @image_lock,
 * so this loop is most of the CPU time of a refresh. It is a pure function
 * of the source rows; rk3026_ebc_blit() in rk3026_ebc.c widens the
 * clip and passes the target image.
 *
 * rk3026_ebc_blit_rows() takes the DRM format of the source and converts
 * its rows with a loop of that format. For XRGB8888 the grey is the BT.601
 * luma, (77 R + 150 G + 29 B) / 256 rounded down, and the nibble is its
 * top four bits: the same few operations for every pixel, with no test of
 * the content, so every page costs the same. ARGB8888 takes the same rows,
 * XBGR8888 and ABGR8888 the same with the weights of R and B exchanged:
 * a colour gives the same nibble in all four, at the same cost. The alpha
 * byte is ignored: the plane has nothing below it, and a premultiplied
 * colour without its alpha is that colour over black.
 *
 * With CONFIG_DRM_RK3026_EBC_NEON, rk3026_ebc_blit_convert() runs the NEON
 * rows of rk3026_ebc_blit_neon.c instead where it can, and these rows
 * everywhere else.
 *
 * For R8 sources (one grey byte per pixel) each grey is rounded to the
 * nearest of the panel's 16 levels (rk3026_ebc_grey_level()), so the greys
 * k * 17 that a 16-level dither emits stay exactly level k. The rows
 * compute the level, four pixels per 32-bit word or 32 per NEON step, with
 * no table and no division.
 *
 * C8 sources (one palette index per pixel) take rk3026_ebc_blit_c8_rows():
 * each index goes through a 256-entry table of levels, one
 * rk3026_ebc_palette_level() an entry of the palette.
 *
 * An update of XRGB8888 or R8 may ask for an ordered dither instead
 * (rk3026_ebc_blit_dither(), enum rk3026_ebc_dither): the same 8-bit grey,
 * the luma or the R8 byte, spread over the 16 levels or over black and
 * white by an 8x8 threshold pattern. It costs a few operations more per
 * pixel, again the same for every page, and takes NEON rows where the
 * plain conversion does.
 */

#include <linux/build_bug.h>
#include <linux/compiler.h>
#include <linux/export.h>
#include <linux/minmax.h>
#include <linux/prefetch.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include <drm/drm_fourcc.h>

#include <kunit/visibility.h>

#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_NEON)
#include <asm/neon.h>
#include <asm/simd.h>
#endif

#include "rk3026_ebc.h"
#include "rk3026_ebc_blit_neon.h"

static_assert(EBC_NEON_XRGB8888 == DRM_FORMAT_XRGB8888);
static_assert(EBC_NEON_ARGB8888 == DRM_FORMAT_ARGB8888);
static_assert(EBC_NEON_XBGR8888 == DRM_FORMAT_XBGR8888);
static_assert(EBC_NEON_ABGR8888 == DRM_FORMAT_ABGR8888);
static_assert(EBC_NEON_R8 == DRM_FORMAT_R8);

/*
 * The BT.601 weights of the two outer bytes of a 32-bit pixel, for
 * ebc_luma_k(): EBC_LUMA_XRGB for R in bits 23:16 (XRGB8888, ARGB8888),
 * EBC_LUMA_XBGR for R in bits 7:0 (XBGR8888, ABGR8888).
 */
#define EBC_LUMA_XRGB	(0x001d004d - 0x9600)
#define EBC_LUMA_XBGR	(0x004d001d - 0x9600)

/*
 * The luma of one XRGB8888 pixel @p in bits 31:24 with @k = EBC_LUMA_XRGB,
 * so its nibble is in bits 31:28. With a = @p & 0x00ff00ff (R << 16 | B),
 * a * 0x001d004d holds 77 R + 29 B in the high half (29 R leaves at bit
 * 32) and 77 B < 2^16 in the low half, and (@p & 0xff00) * 0x9600 adds
 * 150 G to the high half: the high half is the sum, and the low half
 * cannot carry into it. a * (0x001d004d - 0x9600) + @p * 0x9600 is the
 * same mod 2^32 with one mask less: @p * 0x9600 adds the 150 R << 24 and
 * 150 B << 8 that a * -0x9600 takes away, and the X byte leaves at bit 32.
 *
 * In an XBGR8888 pixel a is B << 16 | R, and @k = EBC_LUMA_XBGR swaps the
 * two weights: 77 R + 29 B again in the high half, 29 R < 2^16 in the low
 * one. The sum, and so the luma, is that of the same colour in XRGB8888,
 * at the same cost.
 */
static __always_inline u32 ebc_luma_k(u32 p, u32 k)
{
	return (p & 0x00ff00ff) * k + p * 0x9600;
}

/* Two pixels into one byte, @p0 in the low nibble; @k as for ebc_luma_k(). */
static __always_inline u32 ebc_pack2(u32 p0, u32 p1, u32 k)
{
	return (ebc_luma_k(p0, k) >> 28) | ((ebc_luma_k(p1, k) >> 24) & 0xf0);
}

/*
 * The 32-bit rows, @k as for ebc_luma_k(): four pixels per 16-bit store,
 * with the source prefetched 128 bytes ahead (twice per 32-byte cache
 * line); a row's last two pixels, if any, go into one byte. Four pixels,
 * not eight: a body of eight pixels spills registers at -Os on 32-bit ARM.
 */
static __always_inline void ebc_rgb32_rows(u8 *dst, const void *src,
					   unsigned int src_pitch, int x1,
					   int x2, int y1, int y2,
					   unsigned int width, u32 k)
{
	int y;

	for (y = y1; y < y2; y++) {
		const u8 *s = (const u8 *)src + (size_t)y * src_pitch + x1 * 4;
		u8 *d = dst + ((size_t)y * width + x1) / 2;
		int n = x2 - x1;

		for (; n >= 4; n -= 4, s += 16, d += 2) {
			prefetch(s + 128);
			put_unaligned_le16(ebc_pack2(get_unaligned_le32(s),
						     get_unaligned_le32(s + 4),
						     k) |
					   ebc_pack2(get_unaligned_le32(s + 8),
						     get_unaligned_le32(s + 12),
						     k) << 8,
					   d);
		}
		for (; n > 0; n -= 2, s += 8, d++)
			*d = ebc_pack2(get_unaligned_le32(s),
				       get_unaligned_le32(s + 4), k);
	}
}

/* The luma of an XRGB8888 pixel, as ebc_luma_k() computes it. */
static __always_inline u32 ebc_luma(u32 p)
{
	return ebc_luma_k(p, EBC_LUMA_XRGB);
}

/*
 * The levels of the four 8-bit greys in @w, pixel 0 in the low byte, as
 * two packed bytes in bits 0 to 15, pixel 0 in the low nibble.
 * rk3026_ebc_grey_level() of two pixels at once, one in each 16-bit half:
 * g * 15 + 135 stays below 2^12, so the halves never carry into each other.
 */
static __always_inline u32 ebc_r8_pack4(u32 w)
{
	u32 even = w & 0x00ff00ff, odd = w >> 8 & 0x00ff00ff;

	even = even * 15 + 0x00870087;	/* levels in bits 8-11 and 24-27 */
	odd = odd * 15 + 0x00870087;
	even = (even >> 8 & 0x000f000f) | (odd >> 4 & 0x00f000f0);
	return (even | even >> 8) & 0xffff;
}

/*
 * The R8 rows, rk3026_ebc_grey_level() of each pixel: eight pixels per
 * 32-bit store from two 32-bit loads, with the source prefetched 128 bytes
 * ahead; a row's last 2 to 6 pixels two at a time.
 */
static __always_inline void ebc_r8_rows(u8 *dst, const void *src,
					unsigned int src_pitch, int x1, int x2,
					int y1, int y2, unsigned int width)
{
	int y;

	for (y = y1; y < y2; y++) {
		const u8 *s = (const u8 *)src + (size_t)y * src_pitch + x1;
		u8 *d = dst + ((size_t)y * width + x1) / 2;
		int n = x2 - x1;

		for (; n >= 8; n -= 8, s += 8, d += 4) {
			u32 lo = ebc_r8_pack4(get_unaligned_le32(s));
			u32 hi = ebc_r8_pack4(get_unaligned_le32(s + 4));

			prefetch(s + 128);
			put_unaligned_le32(lo | hi << 16, d);
		}
		for (; n > 0; n -= 2, s += 2, d++)
			*d = rk3026_ebc_grey_level(s[0]) |
			     rk3026_ebc_grey_level(s[1]) << 4;
	}
}

/**
 * rk3026_ebc_blit_rows - convert rows to packed 4bpp grey
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: source, top-left of the whole framebuffer
 * @src_pitch: source stride in bytes
 * @format: source format: DRM_FORMAT_XRGB8888, DRM_FORMAT_ARGB8888,
 *	DRM_FORMAT_XBGR8888, DRM_FORMAT_ABGR8888 or DRM_FORMAT_R8
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 *
 * Writes bytes @x1 / 2 to @x2 / 2 of rows @y1 to @y2 of @dst, nothing else:
 * the top four bits of the luma of a 32-bit pixel, alpha ignored, the
 * rk3026_ebc_grey_level() of an R8 one. Any other format writes nothing.
 */
void rk3026_ebc_blit_rows(u8 *dst, const void *src, unsigned int src_pitch,
			  u32 format, int x1, int x2, int y1, int y2,
			  unsigned int width)
{
	switch (format) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		ebc_rgb32_rows(dst, src, src_pitch, x1, x2, y1, y2, width,
			       EBC_LUMA_XRGB);
		break;
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		ebc_rgb32_rows(dst, src, src_pitch, x1, x2, y1, y2, width,
			       EBC_LUMA_XBGR);
		break;
	case DRM_FORMAT_R8:
		ebc_r8_rows(dst, src, src_pitch, x1, x2, y1, y2, width);
		break;
	}
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_rows);

#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_NEON)
/*
 * Rows per kernel-mode NEON section. A section disables preemption and
 * softirqs; 64 full rows of a 1448x1072 panel take about 0.5 ms on a
 * Cortex-A9 at 600 MHz.
 */
#define EBC_NEON_BAND_ROWS	64

/* Exported here: rk3026_ebc_blit_neon.c includes no kernel header. */
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_rows_neon);
/* And the dither rows. */
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_rows_dither_neon);

/* The NEON rows are built in and this CPU can run them. */
static bool ebc_neon_cpu(void)
{
	return cpu_has_neon();
}

/*
 * The NEON rows of @format on the clip, one band of rows per section, if
 * NEON may run here and the clip holds at least one 16-pixel block; else
 * false and nothing written.
 */
static bool ebc_neon_bands(u8 *dst, const void *src, unsigned int src_pitch,
			   u32 format, int x1, int x2, int y1, int y2,
			   unsigned int width)
{
	int y;

	if (x2 - x1 < 16 || !ebc_neon_cpu() || !may_use_simd())
		return false;

	for (y = y1; y < y2; y += EBC_NEON_BAND_ROWS) {
		int end = min(y + EBC_NEON_BAND_ROWS, y2);

		scoped_ksimd()
			rk3026_ebc_blit_rows_neon(dst, src, src_pitch, format,
						  x1, x2, y, end, width);
	}
	return true;
}
#else
static bool ebc_neon_cpu(void)
{
	return false;
}

static bool ebc_neon_bands(u8 *dst, const void *src, unsigned int src_pitch,
			   u32 format, int x1, int x2, int y1, int y2,
			   unsigned int width)
{
	return false;
}
#endif

/**
 * rk3026_ebc_blit_convert - convert rows, with NEON if it can
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: source, top-left of the whole framebuffer
 * @src_pitch: source stride in bytes
 * @format: source format, as for rk3026_ebc_blit_rows()
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 *
 * rk3026_ebc_blit_rows_neon() when the driver has it, the CPU has NEON,
 * may_use_simd() allows it and the clip is at least 16 pixels wide;
 * rk3026_ebc_blit_rows() otherwise. Both write the same bytes, so the
 * choice only changes the time.
 *
 * Returns true if the NEON rows ran. For KUnit, which checks the choice;
 * a caller needs no answer. A format without rows (only KUnit's) writes
 * nothing but may return true.
 */
bool rk3026_ebc_blit_convert(u8 *dst, const void *src, unsigned int src_pitch,
			     u32 format, int x1, int x2, int y1, int y2,
			     unsigned int width)
{
	if (ebc_neon_bands(dst, src, src_pitch, format, x1, x2, y1, y2, width))
		return true;
	rk3026_ebc_blit_rows(dst, src, src_pitch, format, x1, x2, y1, y2,
			     width);
	return false;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_convert);

/* Two C8 pixels at @s into one byte through @levels, @s[0] low. */
static __always_inline u32 ebc_c8_pack2(const u8 *levels, const u8 *s)
{
	return levels[s[0]] | levels[s[1]] << 4;
}

/**
 * rk3026_ebc_blit_c8_rows - convert C8 rows to packed 4bpp grey
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: C8 source, one palette index per pixel, top-left of the framebuffer
 * @src_pitch: source stride in bytes
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 * @levels: EBC_PALETTE_SIZE levels of 0 to 15, one per index
 *
 * Writes bytes @x1 / 2 to @x2 / 2 of rows @y1 to @y2 of @dst, nothing else.
 *
 * Eight pixels per 32-bit store, with the source prefetched 128 bytes
 * ahead; a row's last 2 to 6 pixels two at a time.
 */
void rk3026_ebc_blit_c8_rows(u8 *dst, const void *src, unsigned int src_pitch,
			     int x1, int x2, int y1, int y2,
			     unsigned int width, const u8 *levels)
{
	int y;

	for (y = y1; y < y2; y++) {
		const u8 *s = (const u8 *)src + (size_t)y * src_pitch + x1;
		u8 *d = dst + ((size_t)y * width + x1) / 2;
		int n = x2 - x1;

		for (; n >= 8; n -= 8, s += 8, d += 4) {
			prefetch(s + 128);
			put_unaligned_le32(ebc_c8_pack2(levels, s) |
					   ebc_c8_pack2(levels, s + 2) << 8 |
					   ebc_c8_pack2(levels, s + 4) << 16 |
					   ebc_c8_pack2(levels, s + 6) << 24,
					   d);
		}
		for (; n > 0; n -= 2, s += 2, d++)
			*d = ebc_c8_pack2(levels, s);
	}
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_c8_rows);

/**
 * rk3026_ebc_palette_level - the level of a C8 palette entry
 * @red: red of the entry, 16 bits
 * @green: green of the entry, 16 bits
 * @blue: blue of the entry, 16 bits
 *
 * The entry is taken at 8 bits a component, the top byte of each 16-bit
 * value, its grey is the luma of the XRGB8888 conversion, and
 * rk3026_ebc_grey_level() rounds that grey to the nearest level. An index
 * thus shows as its colour would in XRGB8888, but rounded rather than cut
 * to the top four bits, and a grey ramp, entry i = i * 0x101 as
 * drm_crtc_fill_palette_8() fills it or i << 8 as the legacy gamma table
 * starts, gives the levels of R8.
 *
 * Returns the level, 0 to 15.
 */
u8 rk3026_ebc_palette_level(u16 red, u16 green, u16 blue)
{
	u32 p = (red >> 8) << 16 | (green >> 8) << 8 | blue >> 8;

	return rk3026_ebc_grey_level(ebc_luma(p) >> 24);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_palette_level);

/*
 * The ordered dither (EBC_DITHER_Y4, EBC_DITHER_Y1).
 *
 * The 8-bit grey L of a pixel, the luma of rk3026_ebc_blit_rows() or the
 * byte of an R8 source, is spread over @steps steps, 15 for the 16 levels
 * and 1 for black and white: its level is (L * @steps + t) / 255, where t
 * is the threshold of the pixel's cell in an 8x8 Bayer matrix, 4 b + 1 for
 * the cell's index b (0 to 63). The thresholds lie evenly between 1 and
 * 253, so with f = L * @steps mod 255 a flat area of L shows the upper of
 * its two levels on f * 64 / 255 cells of every 8x8 tile, to within half
 * a cell: the area keeps its mean grey. The grey of a level itself
 * (L = 17 n for the 16 levels; 0 and 255 for black and white) is drawn
 * flat, as the same nibble as the plain conversion's. The nibble is the
 * level times 15 / @steps. Between the levels the conversions differ on
 * purpose: the plain XRGB8888 one draws L / 16 rounded down, the dither
 * keeps the mean L / 17, so a dithered grey can differ by up to about one
 * level from the same grey drawn plainly: lighter at the dark end, darker
 * near white, where the plain one draws 240 to 254 as white. R8 rounds to
 * the nearest level, which the dither's mean matches.
 *
 * The matrix is anchored to the image: the cell of pixel (x, y) is
 * (x mod 8, y mod 8) whatever the clip, so adjacent updates with the same
 * conversion tile without a seam, and a pixel converted twice from the
 * same source gets the same nibble. A clip with an odd edge is widened to
 * whole bytes (rk3026_ebc_widen_clip()), so the pixel next to that edge
 * takes the conversion of the update that widened over it.
 */
#define EBC_DT(b)	(4 * (b) + 1)
#define EBC_DT_ROW(a, b, c, d, e, f, g, h)				\
	{ EBC_DT(a), EBC_DT(b), EBC_DT(c), EBC_DT(d),			\
	  EBC_DT(e), EBC_DT(f), EBC_DT(g), EBC_DT(h) }

static const u8 ebc_dither_t[8][8] = {
	EBC_DT_ROW(0, 32, 8, 40, 2, 34, 10, 42),
	EBC_DT_ROW(48, 16, 56, 24, 50, 18, 58, 26),
	EBC_DT_ROW(12, 44, 4, 36, 14, 46, 6, 38),
	EBC_DT_ROW(60, 28, 52, 20, 62, 30, 54, 22),
	EBC_DT_ROW(3, 35, 11, 43, 1, 33, 9, 41),
	EBC_DT_ROW(51, 19, 59, 27, 49, 17, 57, 25),
	EBC_DT_ROW(15, 47, 7, 39, 13, 45, 5, 37),
	EBC_DT_ROW(63, 31, 55, 23, 61, 29, 53, 21),
};

/* The bytes per pixel of a dither source @format: XRGB8888 or R8. */
static __always_inline unsigned int ebc_dither_cpp(u32 format)
{
	return format == DRM_FORMAT_XRGB8888 ? 4 : 1;
}

/*
 * The 8-bit grey of the pixel at @s: the luma of an XRGB8888 pixel, the
 * byte of an R8 one.
 */
static __always_inline u32 ebc_dither_grey(const u8 *s, u32 format)
{
	return format == DRM_FORMAT_XRGB8888 ?
	       ebc_luma(get_unaligned_le32(s)) >> 24 : *s;
}

/*
 * The dithered nibble of grey @l with threshold @t on @steps steps. The
 * sum v = @l * @steps + @t is below 2^16, where v / 255 is
 * (v + (v >> 8) + 1) >> 8.
 */
static __always_inline u32 ebc_dither_nibble(u32 l, u32 t, unsigned int steps)
{
	u32 v = l * steps + t;

	return ((v + (v >> 8) + 1) >> 8) * (15 / steps);
}

/* Two pixels at @s, columns @x and @x + 1 of row thresholds @t, one byte. */
static __always_inline u32 ebc_dither2(const u8 *s, const u8 *t, int x,
				       u32 format, unsigned int steps)
{
	unsigned int cpp = ebc_dither_cpp(format);

	return ebc_dither_nibble(ebc_dither_grey(s, format), t[x & 7],
				 steps) |
	       ebc_dither_nibble(ebc_dither_grey(s + cpp, format),
				 t[(x + 1) & 7], steps) << 4;
}

/* rk3026_ebc_blit_rows_dither() for a constant @format and @steps. */
static __always_inline void ebc_dither_rows(u8 *dst, const void *src,
					    unsigned int src_pitch, u32 format,
					    int x1, int x2, int y1, int y2,
					    unsigned int width,
					    unsigned int steps)
{
	const unsigned int cpp = ebc_dither_cpp(format);
	int x, y;

	for (y = y1; y < y2; y++) {
		const u8 *s = (const u8 *)src + (size_t)y * src_pitch +
			      x1 * cpp;
		const u8 *t = ebc_dither_t[y & 7];
		u8 *d = dst + ((size_t)y * width + x1) / 2;

		for (x = x1; x2 - x >= 4; x += 4, s += 4 * cpp, d += 2) {
			prefetch(s + 128);
			put_unaligned_le16(ebc_dither2(s, t, x, format, steps) |
					   ebc_dither2(s + 2 * cpp, t, x + 2,
						       format, steps) << 8, d);
		}
		if (x < x2)
			*d = ebc_dither2(s, t, x, format, steps);
	}
}

/**
 * rk3026_ebc_blit_rows_dither - convert rows with an ordered dither
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: source, top-left of the whole framebuffer
 * @src_pitch: source stride in bytes
 * @format: source format, DRM_FORMAT_XRGB8888 or DRM_FORMAT_R8
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 * @steps: 15 to dither to the 16 levels, 1 to black and white
 *
 * Writes bytes @x1 / 2 to @x2 / 2 of rows @y1 to @y2 of @dst, nothing else,
 * with the grey of each pixel dithered as described above. Four pixels
 * per 16-bit store, as the XRGB8888 rows of rk3026_ebc_blit_rows(). Any
 * other format writes nothing.
 */
void rk3026_ebc_blit_rows_dither(u8 *dst, const void *src,
				 unsigned int src_pitch, u32 format, int x1,
				 int x2, int y1, int y2, unsigned int width,
				 unsigned int steps)
{
	switch (format) {
	case DRM_FORMAT_XRGB8888:
		if (steps == 1)
			ebc_dither_rows(dst, src, src_pitch,
					DRM_FORMAT_XRGB8888, x1, x2, y1, y2,
					width, 1);
		else
			ebc_dither_rows(dst, src, src_pitch,
					DRM_FORMAT_XRGB8888, x1, x2, y1, y2,
					width, 15);
		break;
	case DRM_FORMAT_R8:
		if (steps == 1)
			ebc_dither_rows(dst, src, src_pitch, DRM_FORMAT_R8, x1,
					x2, y1, y2, width, 1);
		else
			ebc_dither_rows(dst, src, src_pitch, DRM_FORMAT_R8, x1,
					x2, y1, y2, width, 15);
		break;
	}
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_rows_dither);

#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_NEON)
/* ebc_neon_bands() for the dither rows of @format on @steps steps. */
static bool ebc_dither_neon(u8 *dst, const void *src, unsigned int src_pitch,
			    u32 format, int x1, int x2, int y1, int y2,
			    unsigned int width, unsigned int steps)
{
	int y;

	if (x2 - x1 < 16 || !ebc_neon_cpu() || !may_use_simd())
		return false;

	for (y = y1; y < y2; y += EBC_NEON_BAND_ROWS) {
		int end = min(y + EBC_NEON_BAND_ROWS, y2);

		scoped_ksimd()
			rk3026_ebc_blit_rows_dither_neon(dst, src, src_pitch,
							 format, x1, x2, y, end,
							 width,
							 &ebc_dither_t[0][0],
							 steps);
	}
	return true;
}
#else
static bool ebc_dither_neon(u8 *dst, const void *src, unsigned int src_pitch,
			    u32 format, int x1, int x2, int y1, int y2,
			    unsigned int width, unsigned int steps)
{
	return false;
}
#endif

/**
 * rk3026_ebc_blit_dither - convert XRGB8888 or R8 rows with a dither
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: source, top-left of the whole framebuffer
 * @src_pitch: source stride in bytes
 * @format: source format, DRM_FORMAT_XRGB8888 or DRM_FORMAT_R8
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 * @dither: EBC_DITHER_Y4 or EBC_DITHER_Y1
 *
 * The dither on 15 steps for EBC_DITHER_Y4 and on one step for
 * EBC_DITHER_Y1: rk3026_ebc_blit_rows_dither_neon() under the conditions
 * of rk3026_ebc_blit_convert(), rk3026_ebc_blit_rows_dither() otherwise.
 * Both write the same bytes, so the choice only changes the time.
 * EBC_DITHER_NONE is the caller's: it takes the plain conversion of its
 * format.
 *
 * Returns true if the NEON rows ran. For KUnit, which checks the choice;
 * a caller needs no answer. A format without rows (only KUnit's) writes
 * nothing but may return true.
 */
bool rk3026_ebc_blit_dither(u8 *dst, const void *src, unsigned int src_pitch,
			    u32 format, int x1, int x2, int y1, int y2,
			    unsigned int width, enum rk3026_ebc_dither dither)
{
	unsigned int steps = dither == EBC_DITHER_Y1 ? 1 : 15;

	if (ebc_dither_neon(dst, src, src_pitch, format, x1, x2, y1, y2, width,
			    steps))
		return true;
	rk3026_ebc_blit_rows_dither(dst, src, src_pitch, format, x1, x2, y1,
				    y2, width, steps);
	return false;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_dither);

/**
 * rk3026_ebc_blit_impl - the XRGB8888 conversion rk3026_ebc_blit() runs
 *
 * For the probe line and debugfs "timing", so a measured blit time says
 * which loop it is from. The other 32-bit formats, R8 and the dither take
 * NEON under the same conditions; C8 always takes the scalar table rows
 * (rk3026_ebc_blit_c8_rows()), whatever this says.
 *
 * Returns "neon" if rk3026_ebc_blit_convert() takes the NEON rows for a
 * clip of 16 pixels or more in process context, else "scalar".
 */
const char *rk3026_ebc_blit_impl(void)
{
	return ebc_neon_cpu() ? "neon" : "scalar";
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_impl);
