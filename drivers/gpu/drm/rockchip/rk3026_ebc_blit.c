// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK3026 EBC: XRGB8888 to the panel's packed 4bpp grey.
 *
 * Every plane and mxcfb update converts its damage here, under @image_lock,
 * so this loop is most of the CPU time of a refresh. It is a pure function
 * of the source rows; rk3026_ebc_blit_xrgb8888() in rk3026_ebc.c widens the
 * clip and passes the target image.
 *
 * The grey is the BT.601 luma, (77 R + 150 G + 29 B) / 256 rounded down,
 * and the nibble is its top four bits. Every pixel takes the same few
 * operations, with no test of the content, so every page costs the same.
 */

#include <linux/compiler.h>
#include <linux/export.h>
#include <linux/prefetch.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include <kunit/visibility.h>

#include "rk3026_ebc.h"

/*
 * The luma of one XRGB8888 pixel @p in bits 31:24, so its nibble is in bits
 * 31:28. With a = @p & 0x00ff00ff (R << 16 | B), a * 0x001d004d holds
 * 77 R + 29 B in the high half (29 R leaves at bit 32) and 77 B < 2^16 in
 * the low half, and (@p & 0xff00) * 0x9600 adds 150 G to the high half:
 * the high half is the sum, and the low half cannot carry into it.
 * a * (0x001d004d - 0x9600) + @p * 0x9600 is the same mod 2^32 with one
 * mask less: @p * 0x9600 adds the 150 R << 24 and 150 B << 8 that
 * a * -0x9600 takes away, and the X byte leaves at bit 32.
 */
static __always_inline u32 ebc_luma(u32 p)
{
	return (p & 0x00ff00ff) * 0x001c6a4d + p * 0x9600;
}

/* Two pixels into one byte, @p0 in the low nibble. */
static __always_inline u32 ebc_pack2(u32 p0, u32 p1)
{
	return (ebc_luma(p0) >> 28) | ((ebc_luma(p1) >> 24) & 0xf0);
}

/**
 * rk3026_ebc_blit_rows - convert XRGB8888 rows to packed 4bpp grey
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: XRGB8888 source, top-left of the whole framebuffer
 * @src_pitch: source stride in bytes
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 *
 * Writes bytes @x1 / 2 to @x2 / 2 of rows @y1 to @y2 of @dst, nothing else.
 *
 * Four pixels per 16-bit store, with the source prefetched 128 bytes ahead
 * (twice per 32-byte cache line); a row's last two pixels, if any, go into
 * one byte. Four pixels, not eight: a body of eight pixels spills
 * registers at -Os on 32-bit ARM.
 */
void rk3026_ebc_blit_rows(u8 *dst, const void *src, unsigned int src_pitch,
			  int x1, int x2, int y1, int y2, unsigned int width)
{
	int y;

	for (y = y1; y < y2; y++) {
		const u8 *s = (const u8 *)src + (size_t)y * src_pitch + x1 * 4;
		u8 *d = dst + ((size_t)y * width + x1) / 2;
		int n = x2 - x1;

		for (; n >= 4; n -= 4, s += 16, d += 2) {
			prefetch(s + 128);
			put_unaligned_le16(ebc_pack2(get_unaligned_le32(s),
						     get_unaligned_le32(s + 4)) |
					   ebc_pack2(get_unaligned_le32(s + 8),
						     get_unaligned_le32(s + 12)) << 8,
					   d);
		}
		for (; n > 0; n -= 2, s += 8, d++)
			*d = ebc_pack2(get_unaligned_le32(s),
				       get_unaligned_le32(s + 4));
	}
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_rows);
