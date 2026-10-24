// SPDX-License-Identifier: GPL-2.0
/*
 * Reference implementations for the RK3026 EBC KUnit tests.
 *
 * The snapshot and the XRGB8888 blit worked out pixel by pixel, in the
 * byte loops rk3026_ebc_core.c and rk3026_ebc.c had before the word-wide
 * versions. The tests require the driver's functions to match these bit
 * for bit. They have their own copies of the helpers and share no code
 * with the driver, so a change to the driver cannot change them too.
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
 * The XRGB8888 blit as a per-pixel loop: R, G and B taken apart, the
 * BT.601 luma (77 R + 150 G + 29 B) / 256, two nibbles per byte. On a box
 * the caller has already widened and clamped (even @x1 and @x2, at most
 * @width; @y1 and @y2 within the image), into @dst of @width / 2 bytes per
 * line.
 */
void rk3026_ebc_ref_blit_rows(u8 *dst, const void *src,
			      unsigned int src_pitch, int x1, int x2,
			      int y1, int y2, unsigned int width)
{
	int x, y;

	for (y = y1; y < y2; y++) {
		const u8 *s = (const u8 *)src + (size_t)y * src_pitch + x1 * 4;
		u8 *d = dst + ((size_t)y * width + x1) / 2;

		for (x = x1; x < x2; x += 2, s += 8, d++) {
			u32 p0 = get_unaligned_le32(s);
			u32 p1 = get_unaligned_le32(s + 4);
			u32 g0 = (77 * ((p0 >> 16) & 0xff) +
				  150 * ((p0 >> 8) & 0xff) +
				  29 * (p0 & 0xff)) / 256;
			u32 g1 = (77 * ((p1 >> 16) & 0xff) +
				  150 * ((p1 >> 8) & 0xff) +
				  29 * (p1 & 0xff)) / 256;

			*d = (g0 >> 4) | (g1 & 0xf0);
		}
	}
}
