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
 * and the nibble is its top four bits. rk3026_ebc_blit_rows() takes the
 * same few operations for every pixel, with no test of the content, so
 * every page costs the same.
 *
 * With CONFIG_DRM_RK3026_EBC_NEON, rk3026_ebc_blit_convert() runs the NEON
 * rows of rk3026_ebc_blit_neon.c instead where it can, and these rows
 * everywhere else.
 *
 * An update may ask for an ordered dither instead (rk3026_ebc_blit_dither(),
 * enum rk3026_ebc_dither): the same luma, spread over the 16 levels or over
 * black and white by an 8x8 threshold pattern. It costs a few operations
 * more per pixel, again the same for every page.
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

/*
 * The ordered dither (EBC_DITHER_Y4, EBC_DITHER_Y1).
 *
 * The 8-bit luma L of a pixel is spread over @steps steps, 15 for the 16
 * levels and 1 for black and white: its level is (L * @steps + t) / 255,
 * where t is the threshold of the pixel's cell in an 8x8 Bayer matrix,
 * 4 b + 1 for the cell's index b (0 to 63). The thresholds lie evenly
 * between 1 and 253, so with f = L * @steps mod 255 a flat area of L shows
 * the upper of its two levels on f * 64 / 255 cells of every 8x8 tile, to
 * within half a cell: the area keeps its mean grey. The grey of a level
 * itself (L = 17 n for the 16 levels; 0 and 255 for black and white) is
 * drawn flat, as the same nibble as the plain conversion's. The nibble is
 * the level times 15 / @steps. Between the levels the two conversions
 * differ on purpose: the plain one draws L / 16 rounded down, the dither
 * keeps the mean L / 17, so a dithered grey can differ by up to about one
 * level from the same grey drawn plainly: lighter at the dark end, darker
 * near white, where the plain one draws 240 to 254 as white.
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

/*
 * The dithered nibble of pixel @p with threshold @t on @steps steps. The
 * sum v = L * @steps + @t is below 2^16, where v / 255 is
 * (v + (v >> 8) + 1) >> 8.
 */
static __always_inline u32 ebc_dither_nibble(u32 p, u32 t, unsigned int steps)
{
	u32 v = (ebc_luma(p) >> 24) * steps + t;

	return ((v + (v >> 8) + 1) >> 8) * (15 / steps);
}

/* Two pixels at @s, columns @x and @x + 1 of row thresholds @t, one byte. */
static __always_inline u32 ebc_dither2(const u8 *s, const u8 *t, int x,
				       unsigned int steps)
{
	return ebc_dither_nibble(get_unaligned_le32(s), t[x & 7], steps) |
	       ebc_dither_nibble(get_unaligned_le32(s + 4), t[(x + 1) & 7],
				 steps) << 4;
}

/* rk3026_ebc_blit_rows_dither() for a constant @steps. */
static __always_inline void ebc_dither_rows(u8 *dst, const void *src,
					    unsigned int src_pitch, int x1,
					    int x2, int y1, int y2,
					    unsigned int width,
					    unsigned int steps)
{
	int x, y;

	for (y = y1; y < y2; y++) {
		const u8 *s = (const u8 *)src + (size_t)y * src_pitch + x1 * 4;
		const u8 *t = ebc_dither_t[y & 7];
		u8 *d = dst + ((size_t)y * width + x1) / 2;

		for (x = x1; x2 - x >= 4; x += 4, s += 16, d += 2) {
			prefetch(s + 128);
			put_unaligned_le16(ebc_dither2(s, t, x, steps) |
					   ebc_dither2(s + 8, t, x + 2,
						       steps) << 8, d);
		}
		if (x < x2)
			*d = ebc_dither2(s, t, x, steps);
	}
}

/**
 * rk3026_ebc_blit_rows_dither - rk3026_ebc_blit_rows() with an ordered dither
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: XRGB8888 source, top-left of the whole framebuffer
 * @src_pitch: source stride in bytes
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 * @steps: 15 to dither to the 16 levels, 1 to black and white
 *
 * Writes bytes @x1 / 2 to @x2 / 2 of rows @y1 to @y2 of @dst, nothing else,
 * with the luma of rk3026_ebc_blit_rows() dithered as described above.
 * Four pixels per 16-bit store, as rk3026_ebc_blit_rows().
 */
void rk3026_ebc_blit_rows_dither(u8 *dst, const void *src,
				 unsigned int src_pitch, int x1, int x2,
				 int y1, int y2, unsigned int width,
				 unsigned int steps)
{
	if (steps == 1)
		ebc_dither_rows(dst, src, src_pitch, x1, x2, y1, y2, width, 1);
	else
		ebc_dither_rows(dst, src, src_pitch, x1, x2, y1, y2, width, 15);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_rows_dither);

/**
 * rk3026_ebc_blit_dither - convert XRGB8888 rows as an update asks
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: XRGB8888 source, top-left of the whole framebuffer
 * @src_pitch: source stride in bytes
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 * @dither: the conversion
 *
 * rk3026_ebc_blit_convert() for EBC_DITHER_NONE, so an update that asks
 * for no dither gets the bytes it got before the dither existed;
 * rk3026_ebc_blit_rows_dither() on 15 steps for EBC_DITHER_Y4 and on one
 * step for EBC_DITHER_Y1.
 */
void rk3026_ebc_blit_dither(u8 *dst, const void *src, unsigned int src_pitch,
			    int x1, int x2, int y1, int y2, unsigned int width,
			    enum rk3026_ebc_dither dither)
{
	if (dither == EBC_DITHER_NONE)
		rk3026_ebc_blit_convert(dst, src, src_pitch, x1, x2, y1, y2,
					width);
	else
		rk3026_ebc_blit_rows_dither(dst, src, src_pitch, x1, x2, y1,
					    y2, width,
					    dither == EBC_DITHER_Y1 ? 1 : 15);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_dither);

/**
 * rk3026_ebc_blit_impl - the conversion rk3026_ebc_blit_xrgb8888() runs
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
