// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK3026 EBC: XRGB8888 to packed 4bpp grey with NEON.
 *
 * Built with CC_FLAGS_FPU, so the compiler may use NEON anywhere in this
 * unit: nothing here may run outside a kernel-mode NEON section.
 * rk3026_ebc_blit_convert() in rk3026_ebc_blit.c, a unit without FPU flags,
 * calls it in bands of rows inside scoped_ksimd(). <arm_neon.h> brings the
 * <stdint.h> types, which clash with <linux/types.h> on ARM, so this unit
 * includes no kernel header (as lib/raid6/recov_neon_inner.c).
 *
 * The bytes are those of rk3026_ebc_blit_rows(): the nibble is the top four
 * bits of the BT.601 luma, (77 R + 150 G + 29 B) / 256 rounded down. Per 16
 * pixels: vld4.8 splits B, G, R and X into planes, vmull.u8/vmlal.u8 sum the
 * luma into u16 lanes, vshrn #8 narrows it to a byte, vuzp.8 separates the
 * even and the odd pixels, and vsri.8 #4 packs each pair into one byte with
 * the even pixel in the low nibble. The source is read once and nothing is
 * tested, so every page costs about the same; on the RK3026 (Cortex-A9 at
 * 600 MHz, with the L2 cache controller's prefetch and line-fill options
 * set as the vendor firmware sets them) that is the memory floor.
 */

#include <arm_neon.h>
#include <stddef.h>

#include "rk3026_ebc_blit_neon.h"

/* The grey nibble of the XRGB8888 pixel at @s (B, G, R, X in memory). */
static __always_inline unsigned char ebc_neon_nibble(const unsigned char *s)
{
	return (77 * s[2] + 150 * s[1] + 29 * s[0]) >> 12;
}

/* Sixteen pixels at @s into eight bytes, pixel 0 in the low nibble. */
static __always_inline uint8x8_t ebc_neon_pack16(const unsigned char *s)
{
	const uint8x8_t k77 = vdup_n_u8(77);
	const uint8x8_t k150 = vdup_n_u8(150);
	const uint8x8_t k29 = vdup_n_u8(29);
	uint8x16x4_t v = vld4q_u8(s);	/* val[0] B, [1] G, [2] R, [3] X */
	uint16x8_t lo, hi;
	uint8x8x2_t eo;

	lo = vmull_u8(vget_low_u8(v.val[2]), k77);
	hi = vmull_u8(vget_high_u8(v.val[2]), k77);
	lo = vmlal_u8(lo, vget_low_u8(v.val[1]), k150);
	hi = vmlal_u8(hi, vget_high_u8(v.val[1]), k150);
	lo = vmlal_u8(lo, vget_low_u8(v.val[0]), k29);
	hi = vmlal_u8(hi, vget_high_u8(v.val[0]), k29);
	eo = vuzp_u8(vshrn_n_u16(lo, 8), vshrn_n_u16(hi, 8));	/* even, odd */
	return vsri_n_u8(eo.val[1], eo.val[0], 4);
}

/**
 * rk3026_ebc_blit_rows_neon - rk3026_ebc_blit_rows() with NEON
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: XRGB8888 source, top-left of the whole framebuffer
 * @src_pitch: source stride in bytes
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 *
 * Writes the same bytes as rk3026_ebc_blit_rows(), and only those. Sixteen
 * pixels at a time, with the source read 256 bytes ahead (pld, as
 * prefetch()); a row's last 2 to 14 pixels take the luma two at a time. No
 * alignment is needed.
 *
 * Call only between kernel_neon_begin() and kernel_neon_end().
 */
void rk3026_ebc_blit_rows_neon(unsigned char *dst, const void *src,
			       unsigned int src_pitch, int x1, int x2,
			       int y1, int y2, unsigned int width)
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
			vst1_u8(d, ebc_neon_pack16(s));
		}
		for (; n > 0; n -= 2, s += 8, d++)
			*d = ebc_neon_nibble(s) | ebc_neon_nibble(s + 4) << 4;
	}
}
