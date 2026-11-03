// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK3026 EBC: XRGB8888 and 8-bit grey to the panel's packed 4bpp
 * grey.
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
 *
 * R8 sources (one grey byte per pixel) take rk3026_ebc_blit_r8_convert():
 * each grey is rounded to the nearest of the panel's 16 levels
 * (rk3026_ebc_grey_level()), so the greys k * 17 that a 16-level dither
 * emits stay exactly level k. The rows compute the level, four pixels per
 * 32-bit word or 32 per NEON step, with no table and no division.
 *
 * C8 sources (one palette index per pixel) take rk3026_ebc_blit_c8_rows():
 * each index goes through a 256-entry table of levels, one
 * rk3026_ebc_palette_level() an entry of the palette.
 *
 * rk3026_ebc_invert() complements a region of the converted image in place,
 * for the mxcfb inversion flag: an image operation of the core, not of the
 * shim that asks for it.
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

/**
 * rk3026_ebc_blit_r8_rows - convert R8 rows to packed 4bpp grey
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: R8 source, one grey byte per pixel, top-left of the framebuffer
 * @src_pitch: source stride in bytes
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 *
 * Writes bytes @x1 / 2 to @x2 / 2 of rows @y1 to @y2 of @dst, nothing else:
 * rk3026_ebc_grey_level() of each pixel.
 *
 * Eight pixels per 32-bit store from two 32-bit loads, with the source
 * prefetched 128 bytes ahead; a row's last 2 to 6 pixels two at a time.
 */
void rk3026_ebc_blit_r8_rows(u8 *dst, const void *src, unsigned int src_pitch,
			     int x1, int x2, int y1, int y2,
			     unsigned int width)
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
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_r8_rows);

/* A NEON row function of rk3026_ebc_blit_neon.c. */
typedef void ebc_neon_rows_fn(unsigned char *dst, const void *src,
			      unsigned int src_pitch, int x1, int x2,
			      int y1, int y2, unsigned int width);

#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_NEON)
/*
 * Rows per kernel-mode NEON section. A section disables preemption and
 * softirqs; 64 full rows of a 1448x1072 panel take about 0.5 ms on a
 * Cortex-A9 at 600 MHz.
 */
#define EBC_NEON_BAND_ROWS	64

/* Exported here: rk3026_ebc_blit_neon.c includes no kernel header. */
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_rows_neon);
/* And the R8 rows of that unit. */
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_r8_rows_neon);

/* The NEON rows are built in and this CPU can run them. */
static bool ebc_neon_cpu(void)
{
	return cpu_has_neon();
}

/*
 * @rows on the clip, one band of rows per section, if NEON may run here
 * and the clip holds at least one 16-pixel block; else false and nothing
 * written.
 */
static __always_inline bool ebc_neon_bands(ebc_neon_rows_fn *rows, u8 *dst,
					   const void *src,
					   unsigned int src_pitch, int x1,
					   int x2, int y1, int y2,
					   unsigned int width)
{
	int y;

	if (x2 - x1 < 16 || !ebc_neon_cpu() || !may_use_simd())
		return false;

	for (y = y1; y < y2; y += EBC_NEON_BAND_ROWS) {
		int end = min(y + EBC_NEON_BAND_ROWS, y2);

		scoped_ksimd()
			rows(dst, src, src_pitch, x1, x2, y, end, width);
	}
	return true;
}
#else
static bool ebc_neon_cpu(void)
{
	return false;
}

/* The NEON rows are not built: their names are NULL, never called. */
#define rk3026_ebc_blit_rows_neon	NULL
#define rk3026_ebc_blit_r8_rows_neon	NULL

static bool ebc_neon_bands(ebc_neon_rows_fn *rows, u8 *dst, const void *src,
			   unsigned int src_pitch, int x1, int x2, int y1,
			   int y2, unsigned int width)
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
	if (!ebc_neon_bands(rk3026_ebc_blit_rows_neon, dst, src, src_pitch, x1,
			    x2, y1, y2, width))
		rk3026_ebc_blit_rows(dst, src, src_pitch, x1, x2, y1, y2,
				     width);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_convert);

/**
 * rk3026_ebc_blit_r8_convert - convert R8 rows, with NEON if it can
 * @dst: target image, @width / 2 bytes per row, low nibble first
 * @src: R8 source, one grey byte per pixel, top-left of the framebuffer
 * @src_pitch: source stride in bytes
 * @x1: first column, even
 * @x2: end column, even, at most @width, above @x1
 * @y1: first row
 * @y2: end row, above @y1
 * @width: image width in pixels
 *
 * rk3026_ebc_blit_r8_rows_neon() under the conditions of
 * rk3026_ebc_blit_convert(), rk3026_ebc_blit_r8_rows() otherwise. Both
 * write the same bytes, so the choice only changes the time.
 *
 * Returns true if the NEON rows ran. For KUnit, which checks the choice;
 * a caller needs no answer.
 */
bool rk3026_ebc_blit_r8_convert(u8 *dst, const void *src,
				unsigned int src_pitch, int x1, int x2, int y1,
				int y2, unsigned int width)
{
	if (ebc_neon_bands(rk3026_ebc_blit_r8_rows_neon, dst, src, src_pitch,
			   x1, x2, y1, y2, width))
		return true;
	rk3026_ebc_blit_r8_rows(dst, src, src_pitch, x1, x2, y1, y2, width);
	return false;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_r8_convert);

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

/**
 * rk3026_ebc_blit_impl - the XRGB8888 conversion rk3026_ebc_blit() runs
 *
 * For the probe line and debugfs "timing", so a measured blit time says
 * which loop it is from. R8 takes NEON under the same conditions; C8
 * always takes the scalar table rows (rk3026_ebc_blit_c8_rows()),
 * whatever this says.
 *
 * Returns "neon" if rk3026_ebc_blit_convert() takes the NEON rows for a
 * clip of 16 pixels or more in process context, else "scalar".
 */
const char *rk3026_ebc_blit_impl(void)
{
	return ebc_neon_cpu() ? "neon" : "scalar";
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_impl);

/**
 * rk3026_ebc_invert - turn every level of a region into 15 - level
 * @img: the packed 4bpp image, @width x @height pixels
 * @clip: the region, clamped to the image and widened to whole bytes
 * @width: the image's width in pixels, even
 * @height: its height in lines
 *
 * The region a blit converted is complemented in place, the other pixel of
 * an edge byte with it, so the image then holds what the glass is to show
 * rather than what the framebuffer holds. Flipping every bit of a byte does
 * that for both of its pixels, so the loop is one byte at a time. No image
 * format reaches here: the caller blits first.
 *
 * The mxcfb shim's EPDC_FLAG_ENABLE_INVERSION is the only caller today; the
 * plane path never inverts.
 */
VISIBLE_IF_KUNIT void rk3026_ebc_invert(u8 *img,
					const struct drm_rect *clip,
					unsigned int width,
					unsigned int height)
{
	struct drm_rect r = *clip;
	int y;

	if (!rk3026_ebc_widen_clip(&r, width, height))
		return;

	for (y = r.y1; y < r.y2; y++) {
		u8 *d = img + ((size_t)y * width + r.x1) / 2;
		int i, n = (r.x2 - r.x1) / 2;

		for (i = 0; i < n; i++)
			d[i] = ~d[i];
	}
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_invert);
