// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK3026 EBC: XRGB8888 to the panel's packed 4bpp grey.
 *
 * Every plane and mxcfb update converts its damage here, under @image_lock,
 * so this loop is most of the CPU time of a refresh. It is a pure function
 * of the source rows; rk3026_ebc_blit() in rk3026_ebc.c widens the
 * clip and passes the target image.
 *
 * The grey is the BT.601 luma, (77 R + 150 G + 29 B) / 256 rounded down,
 * and the nibble is its top four bits. rk3026_ebc_blit_rows() takes the
 * same few operations for every pixel, with no test of the content, so
 * every page costs the same.
 *
 * With CONFIG_DRM_RK3026_EBC_NEON, rk3026_ebc_blit_convert() runs the NEON
 * rows of rk3026_ebc_blit_neon.c instead where it can, and these rows
 * everywhere else.
 */

#include <linux/compiler.h>
#include <linux/export.h>
#include <linux/minmax.h>
#include <linux/prefetch.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include <kunit/visibility.h>

#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_NEON)
#include <asm/neon.h>
#include <asm/simd.h>
#endif

#include "rk3026_ebc.h"
#include "rk3026_ebc_blit_neon.h"

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

#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_NEON)
/*
 * Rows per kernel-mode NEON section. A section disables preemption and
 * softirqs; 64 full rows of a 1448x1072 panel take about 0.5 ms on a
 * Cortex-A9 at 600 MHz.
 */
#define EBC_NEON_BAND_ROWS	64

/* Exported here: rk3026_ebc_blit_neon.c includes no kernel header. */
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_rows_neon);

/* The NEON rows are built in and this CPU can run them. */
static bool ebc_neon_cpu(void)
{
	return cpu_has_neon();
}

/*
 * The NEON rows on the clip, one band of rows per section, if NEON may run
 * here and the clip holds at least one 16-pixel block; else false and
 * nothing written.
 */
static bool ebc_blit_neon(u8 *dst, const void *src, unsigned int src_pitch,
			  int x1, int x2, int y1, int y2, unsigned int width)
{
	int y;

	if (x2 - x1 < 16 || !ebc_neon_cpu() || !may_use_simd())
		return false;

	for (y = y1; y < y2; y += EBC_NEON_BAND_ROWS) {
		int end = min(y + EBC_NEON_BAND_ROWS, y2);

		scoped_ksimd()
			rk3026_ebc_blit_rows_neon(dst, src, src_pitch, x1, x2,
						  y, end, width);
	}
	return true;
}
#else
static bool ebc_neon_cpu(void)
{
	return false;
}

static bool ebc_blit_neon(u8 *dst, const void *src, unsigned int src_pitch,
			  int x1, int x2, int y1, int y2, unsigned int width)
{
	return false;
}
#endif

/**
 * rk3026_ebc_blit_convert - convert XRGB8888 rows, with NEON if it can
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: XRGB8888 source, top-left of the whole framebuffer
 * @src_pitch: source stride in bytes
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
 */
void rk3026_ebc_blit_convert(u8 *dst, const void *src, unsigned int src_pitch,
			     int x1, int x2, int y1, int y2,
			     unsigned int width)
{
	if (!ebc_blit_neon(dst, src, src_pitch, x1, x2, y1, y2, width))
		rk3026_ebc_blit_rows(dst, src, src_pitch, x1, x2, y1, y2,
				     width);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_convert);

/**
 * rk3026_ebc_blit_impl - the conversion rk3026_ebc_blit() runs
 *
 * For the probe line and debugfs "timing", so a measured blit time says
 * which loop it is from.
 *
 * Returns "neon" if rk3026_ebc_blit_convert() takes the NEON rows for a
 * clip of 16 pixels or more in process context, else "scalar".
 */
const char *rk3026_ebc_blit_impl(void)
{
	return ebc_neon_cpu() ? "neon" : "scalar";
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_impl);
