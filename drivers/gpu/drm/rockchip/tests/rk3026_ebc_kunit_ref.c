// SPDX-License-Identifier: GPL-2.0
/*
 * Reference implementations for the RK3026 EBC KUnit tests.
 *
 * The snapshot and the 32-bit blit worked out pixel by pixel, in the
 * byte loops rk3026_ebc_core.c and rk3026_ebc.c had before the word-wide
 * versions (the blit in both byte orders of R and B), and the 16-bit
 * blit, the 8-bit blit and the dither one pixel at a time as well. The tests
 * require the driver's functions to match these bit for bit. They have
 * their own copies of the helpers and share no code with the driver, so a
 * change to the driver cannot change them too.
 */

#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "../rk3026_ebc.h"
#include "rk3026_ebc_kunit_ref.h"

static bool ref_mode_two_level(u32 mode)
{
	return mode == EBC_MODE_A2 || mode == EBC_MODE_BLACK_WHITE;
}

static u8 ref_level(u8 grey, bool two_level)
{
	if (!two_level)
		return grey;
	return grey >= 8 ? 0xf : 0;
}

static bool ref_grey(u8 level)
{
	return level != 0 && level != 0xf;
}

/* rk3026_ebc_build_snapshot() as a byte loop, two pixels per byte. */
bool rk3026_ebc_ref_snapshot(const struct rk3026_ebc_shadows *sh,
			     const u8 *next, const struct drm_rect *box,
			     u32 *mode)
{
	const u8 *prev = sh->prev;
	u32 pitch = sh->pitch, height = sh->height;
	u8 *dst = sh->scan;
	bool two_level = ref_mode_two_level(*mode);
	bool over_grey = false;
	int x1 = max(box->x1, 0) & ~1;
	int x2 = min_t(int, round_up(box->x2, 2), (int)(pitch * 2));
	int y1 = max(box->y1, 0);
	int y2 = min_t(int, box->y2, (int)height);
	bool moves = false;
	int x, y;

	memcpy(dst, prev, (size_t)pitch * height);
	if (x1 >= x2 || y1 >= y2)
		return false;

	for (y = y1; y < y2; y++) {
		size_t off = (size_t)y * pitch + x1 / 2;

		for (x = x1; x < x2; x += 2, off++) {
			u8 p = prev[off], n = next[off], out = p;

			if ((p ^ n) & 0x0f) {
				out = (out & 0xf0) | ref_level(n & 0x0f, two_level);
				over_grey |= ref_grey(p & 0x0f);
			}
			if ((p ^ n) & 0xf0) {
				out = (out & 0x0f) | ref_level(n >> 4, two_level) << 4;
				over_grey |= ref_grey(p >> 4);
			}
			dst[off] = out;
			moves |= out != p;
		}
	}
	if (over_grey && *mode == EBC_MODE_A2)
		*mode = EBC_MODE_BLACK_WHITE;
	return moves;
}

/*
 * The BT.601 luma (77 R + 150 G + 29 B) / 256 of the 32-bit pixel @p, R in
 * bits 23:16 and B in bits 7:0, or the other way round with @bgr.
 */
static u32 ref_luma32(u32 p, bool bgr)
{
	u32 r = bgr ? p & 0xff : (p >> 16) & 0xff;
	u32 b = bgr ? (p >> 16) & 0xff : p & 0xff;

	return (77 * r + 150 * ((p >> 8) & 0xff) + 29 * b) / 256;
}

/*
 * The 32-bit blit as a per-pixel loop: R, G and B taken apart, the BT.601
 * luma, two nibbles per byte. On a box the caller has already widened and
 * clamped (even @x1 and @x2, at most @width; @y1 and @y2 within the image),
 * into @dst of @width / 2 bytes per line.
 */
static void ref_blit_rows32(u8 *dst, const void *src, unsigned int src_pitch,
			    int x1, int x2, int y1, int y2,
			    unsigned int width, bool bgr)
{
	int x, y;

	for (y = y1; y < y2; y++) {
		const u8 *s = (const u8 *)src + (size_t)y * src_pitch + x1 * 4;
		u8 *d = dst + ((size_t)y * width + x1) / 2;

		for (x = x1; x < x2; x += 2, s += 8, d++) {
			u32 g0 = ref_luma32(get_unaligned_le32(s), bgr);
			u32 g1 = ref_luma32(get_unaligned_le32(s + 4), bgr);

			*d = (g0 >> 4) | (g1 & 0xf0);
		}
	}
}

/* The XRGB8888 (and ARGB8888) blit as a per-pixel loop. */
void rk3026_ebc_ref_blit_rows(u8 *dst, const void *src,
			      unsigned int src_pitch, int x1, int x2,
			      int y1, int y2, unsigned int width)
{
	ref_blit_rows32(dst, src, src_pitch, x1, x2, y1, y2, width, false);
}

/* The XBGR8888 (and ABGR8888) blit, R in the low byte, as a per-pixel loop. */
void rk3026_ebc_ref_blit_rows_bgr(u8 *dst, const void *src,
				  unsigned int src_pitch, int x1, int x2,
				  int y1, int y2, unsigned int width)
{
	ref_blit_rows32(dst, src, src_pitch, x1, x2, y1, y2, width, true);
}

/*
 * The fields of the 16-bit pixel @v: @t in bits 15:11, @m in 10:5, @l in
 * 4:0. R is @t and B is @l in RGB565, the other way round in BGR565.
 */
static void ref_565_fields(u16 v, bool bgr, u32 *r, u32 *g, u32 *b)
{
	u32 t = v >> 11, m = (v >> 5) & 0x3f, l = v & 0x1f;

	*r = bgr ? l : t;
	*g = m;
	*b = bgr ? t : l;
}

/*
 * The nibble of the 16-bit pixel @v, RGB565 or with @bgr BGR565, from its
 * fields: (159 R + 152 G + 59 B) / 64, the BT.601 weights scaled by
 * 255 / 31, 255 / 63 and 255 / 31, then its top four bits.
 */
u8 rk3026_ebc_ref_565_nibble(u16 v, bool bgr)
{
	u32 r, g, b;

	ref_565_fields(v, bgr, &r, &g, &b);
	return (159 * r + 152 * g + 59 * b) / 64 / 16;
}

/*
 * The nibble of the same colour expanded to 8 bits a component by bit
 * replication (r5 r5[4:2], g6 g6[5:4], b5 b5[4:2]) and converted as
 * XRGB8888: (77 R + 150 G + 29 B) / 256, its top four bits.
 */
u8 rk3026_ebc_ref_565_rep_nibble(u16 v, bool bgr)
{
	u32 r, g, b;

	ref_565_fields(v, bgr, &r, &g, &b);
	r = r << 3 | r >> 2;
	g = g << 2 | g >> 4;
	b = b << 3 | b >> 2;
	return (77 * r + 150 * g + 29 * b) / 256 / 16;
}

/*
 * The 16-bit blit as a per-pixel loop, each pixel through
 * rk3026_ebc_ref_565_nibble(). Same box rules as
 * rk3026_ebc_ref_blit_rows().
 */
static void ref_blit_rows16(u8 *dst, const void *src, unsigned int src_pitch,
			    int x1, int x2, int y1, int y2,
			    unsigned int width, bool bgr)
{
	int x, y;

	for (y = y1; y < y2; y++) {
		const u8 *s = (const u8 *)src + (size_t)y * src_pitch + x1 * 2;
		u8 *d = dst + ((size_t)y * width + x1) / 2;

		for (x = x1; x < x2; x += 2, s += 4, d++)
			*d = rk3026_ebc_ref_565_nibble(get_unaligned_le16(s),
						       bgr) |
			     rk3026_ebc_ref_565_nibble(get_unaligned_le16(s + 2),
						       bgr) << 4;
	}
}

/* The RGB565 blit, R in bits 15:11, as a per-pixel loop. */
void rk3026_ebc_ref_blit_rows_rgb565(u8 *dst, const void *src,
				     unsigned int src_pitch, int x1, int x2,
				     int y1, int y2, unsigned int width)
{
	ref_blit_rows16(dst, src, src_pitch, x1, x2, y1, y2, width, false);
}

/* The BGR565 blit, B in bits 15:11, as a per-pixel loop. */
void rk3026_ebc_ref_blit_rows_bgr565(u8 *dst, const void *src,
				     unsigned int src_pitch, int x1, int x2,
				     int y1, int y2, unsigned int width)
{
	ref_blit_rows16(dst, src, src_pitch, x1, x2, y1, y2, width, true);
}

/*
 * The 8-bit blit as a per-pixel loop, on a box the caller has widened and
 * clamped as for rk3026_ebc_ref_blit_rows(): each byte through @levels,
 * or with @levels NULL to the nearest of the 16 levels k * 17, which is
 * (g + 8) / 17 since 17 is odd.
 */
void rk3026_ebc_ref_grey_rows(u8 *dst, const void *src,
			      unsigned int src_pitch, int x1, int x2,
			      int y1, int y2, unsigned int width,
			      const u8 *levels)
{
	int x, y;

	for (y = y1; y < y2; y++) {
		const u8 *s = (const u8 *)src + (size_t)y * src_pitch + x1;
		u8 *d = dst + ((size_t)y * width + x1) / 2;

		for (x = x1; x < x2; x += 2, s += 2, d++) {
			u8 l0 = levels ? levels[s[0]] : (s[0] + 8) / 17;
			u8 l1 = levels ? levels[s[1]] : (s[1] + 8) / 17;

			*d = l0 | l1 << 4;
		}
	}
}

/*
 * The 8x8 Bayer index of cell (@x mod 8, @y mod 8): the bits of x ^ y and
 * of y interleaved, lowest first, which is the recursive matrix
 * M(2n) = [4 M(n), 4 M(n) + 2; 4 M(n) + 3, 4 M(n) + 1] from M(1) = [0].
 */
static u32 ref_bayer8(int x, int y)
{
	u32 a = (x ^ y) & 7, b = y & 7;

	return (a & 1) << 5 | (b & 1) << 4 | (a & 2) << 2 | (b & 2) << 1 |
	       (a & 4) >> 1 | (b & 4) >> 2;
}

/* The 8x8 thresholds 4 b + 1 of the Bayer indices b, row-major. */
void rk3026_ebc_ref_dither_thr(u8 *thr)
{
	int x, y;

	for (y = 0; y < 8; y++)
		for (x = 0; x < 8; x++)
			thr[y * 8 + x] = 4 * ref_bayer8(x, y) + 1;
}

/*
 * The ordered-dither blit as a per-pixel loop: the grey, the luma as in
 * rk3026_ebc_ref_blit_rows() for @cpp 4 or the byte for @cpp 1, the
 * threshold 4 b + 1 of the pixel's Bayer index b, the level
 * (grey * @steps + threshold) / 255 by a division, and the nibble
 * 15 / @steps times the level. Same box rules as
 * rk3026_ebc_ref_blit_rows().
 */
void rk3026_ebc_ref_blit_rows_dither(u8 *dst, const void *src,
				     unsigned int src_pitch, int x1, int x2,
				     int y1, int y2, unsigned int width,
				     unsigned int cpp, unsigned int steps)
{
	int x, y, i;

	for (y = y1; y < y2; y++) {
		const u8 *s = (const u8 *)src + (size_t)y * src_pitch +
			      x1 * cpp;
		u8 *d = dst + ((size_t)y * width + x1) / 2;

		for (x = x1; x < x2; x += 2, d++) {
			u8 out = 0;

			for (i = 0; i < 2; i++, s += cpp) {
				u32 p = cpp == 4 ? get_unaligned_le32(s) : *s;
				u32 g = cpp == 4 ? (77 * ((p >> 16) & 0xff) +
						    150 * ((p >> 8) & 0xff) +
						    29 * (p & 0xff)) / 256 : p;
				u32 t = 4 * ref_bayer8(x + i, y) + 1;
				u32 n = (g * steps + t) / 255 * (15 / steps);

				out |= n << (4 * i);
			}
			*d = out;
		}
	}
}
