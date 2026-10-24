// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the RK3026 EBC refresh bookkeeping (rk3026_ebc_core.c),
 * the waveform container bounds (rk3026_ebc_waveform.c), the mxcfb waveform
 * map (rk3026_ebc_mxcfb.c), the refresh thread's glue (rk3026_ebc.c), and
 * the snapshot and the blit against the per-pixel references of
 * rk3026_ebc_kunit_ref.c.
 */

#include <kunit/device.h>
#include <kunit/test.h>

#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/prandom.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "../rk3026_ebc.h"
#include "../rk3026_ebc_mxcfb.h"
#include "rk3026_ebc_kunit_ref.h"

/* A 16x6 panel: 8 bytes per line, two pixels per byte, low nibble first. */
#define T_W		16
#define T_H		6
#define T_PITCH		(T_W / 2)
#define T_SIZE		(T_PITCH * T_H)
#define T_GUARD		16

static const struct drm_rect t_panel = {
	.x1 = 0, .y1 = 0, .x2 = T_W, .y2 = T_H,
};

static u8 t_px(const u8 *img, int x, int y)
{
	u8 b = img[y * T_PITCH + x / 2];

	return x & 1 ? b >> 4 : b & 0xf;
}

static void t_px_set(u8 *img, int x, int y, u8 grey)
{
	u8 *b = &img[y * T_PITCH + x / 2];

	if (x & 1)
		*b = (*b & 0x0f) | grey << 4;
	else
		*b = (*b & 0xf0) | grey;
}

static void t_img_fill(u8 *img, u8 grey)
{
	memset(img, grey | grey << 4, T_SIZE);
}

/* An image with guard bytes behind it, to catch a write past its end. */
static u8 *t_img_alloc(struct kunit *test, u8 grey)
{
	u8 *img = kunit_kzalloc(test, T_SIZE + T_GUARD, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, img);
	t_img_fill(img, grey);
	memset(img + T_SIZE, 0xa5, T_GUARD);
	return img;
}

static void t_expect_guard(struct kunit *test, const u8 *img)
{
	int i;

	for (i = 0; i < T_GUARD; i++)
		KUNIT_EXPECT_EQ(test, img[T_SIZE + i], 0xa5);
}

static void t_expect_white(struct kunit *test, const u8 *img)
{
	int i;

	for (i = 0; i < T_SIZE; i++)
		KUNIT_EXPECT_EQ(test, img[i], 0xff);
}

static bool t_in_rect(const struct drm_rect *r, int x, int y)
{
	return x >= r->x1 && x < r->x2 && y >= r->y1 && y < r->y2;
}

static struct rk3026_ebc_update t_upd(const struct drm_rect *clip, u32 mode,
				      u32 marker, enum rk3026_ebc_source source)
{
	return (struct rk3026_ebc_update) {
		.clip = *clip, .mode = mode, .marker = marker, .source = source,
	};
}

/* One update into @p; returns its seq. */
static u32 t_add(struct rk3026_ebc_pending *p, const struct drm_rect *clip,
		 u32 mode, enum rk3026_ebc_source source)
{
	struct rk3026_ebc_update upd = t_upd(clip, mode, 0, source);
	u32 seq;

	rk3026_ebc_pending_add(p, &upd, &seq);
	return seq;
}

/* A send as the driver makes it: the update, then its marker's seq. */
static u32 t_add_marked(struct rk3026_ebc_pending *p,
			struct rk3026_ebc_marker_ring *ring,
			const struct drm_rect *clip, u32 mode, u32 marker)
{
	struct rk3026_ebc_update upd = t_upd(clip, mode, marker,
					     EBC_SOURCE_MXCFB);
	u32 seq;

	rk3026_ebc_pending_add(p, &upd, &seq);
	rk3026_ebc_marker_seq_record(ring, marker, seq);
	return seq;
}

/* --- rank and mode class --- */

static void ebc_test_dsp_ctrl_mode(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(EBC_MODE_FULL),
			DSP_DISPLAY_MODE);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(EBC_MODE_OED_PART),
			DSP_DISPLAY_MODE);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(EBC_MODE_RESET),
			DSP_DISPLAY_MODE);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(EBC_MODE_PART),
			DSP_UPDATE_MODE | DSP_DISPLAY_MODE);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(EBC_MODE_A2),
			DSP_UPDATE_MODE | DSP_DISPLAY_MODE);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(EBC_MODE_AUTO), 0);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(EBC_MODE_MAX + 1),
			DSP_DISPLAY_MODE);
}

static void ebc_test_rank_order(struct kunit *test)
{
	unsigned int a2 = rk3026_ebc_mode_rank(EBC_MODE_A2);
	unsigned int du = rk3026_ebc_mode_rank(EBC_MODE_BLACK_WHITE);
	unsigned int part = rk3026_ebc_mode_rank(EBC_MODE_PART);
	unsigned int oed = rk3026_ebc_mode_rank(EBC_MODE_OED_PART);
	unsigned int full = rk3026_ebc_mode_rank(EBC_MODE_FULL);

	KUNIT_EXPECT_LT(test, a2, du);
	KUNIT_EXPECT_LT(test, du, part);
	KUNIT_EXPECT_LT(test, part, oed);
	KUNIT_EXPECT_LT(test, oed, full);
}

static void ebc_test_rank_unknown(struct kunit *test)
{
	unsigned int full = rk3026_ebc_mode_rank(EBC_MODE_FULL);

	KUNIT_EXPECT_EQ(test, rk3026_ebc_mode_rank(EBC_MODE_TEXT), full);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_mode_rank(EBC_MODE_MAX + 1), full);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_mode_rank(U32_MAX), full);
}

static u32 t_merged_mode(u32 first, u32 second)
{
	struct rk3026_ebc_pending p = {};
	struct rk3026_ebc_set part;

	t_add(&p, &t_panel, first, EBC_SOURCE_PLANE);
	t_add(&p, &t_panel, second, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&p, &part);
	return part.mode;
}

static void ebc_test_rank_merge(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, t_merged_mode(EBC_MODE_A2, EBC_MODE_PART),
			EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, t_merged_mode(EBC_MODE_BLACK_WHITE, EBC_MODE_A2),
			EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, t_merged_mode(EBC_MODE_PART, EBC_MODE_FULL),
			EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, t_merged_mode(EBC_MODE_FULL, EBC_MODE_PART),
			EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, t_merged_mode(EBC_MODE_PART, EBC_MODE_OED_PART),
			EBC_MODE_OED_PART);
	KUNIT_EXPECT_EQ(test, t_merged_mode(EBC_MODE_OED_PART, EBC_MODE_PART),
			EBC_MODE_OED_PART);
	/* An unknown mode ranks as FULL and wins over everything below it... */
	KUNIT_EXPECT_EQ(test, t_merged_mode(EBC_MODE_OED_PART, EBC_MODE_TEXT),
			EBC_MODE_TEXT);
	/* ...and FULL itself wins the tie, whichever came first. */
	KUNIT_EXPECT_EQ(test, t_merged_mode(EBC_MODE_TEXT, EBC_MODE_FULL),
			EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, t_merged_mode(EBC_MODE_FULL, EBC_MODE_TEXT),
			EBC_MODE_FULL);
}

/* --- the pending set --- */

static void ebc_test_pending_empty(struct kunit *test)
{
	struct rk3026_ebc_pending p = {};
	struct rk3026_ebc_set part;

	memset(&part, 0x5a, sizeof(part));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_pending_empty(&p));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&p));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_pending_take(&p, &part));
	KUNIT_EXPECT_EQ(test, part.merged, 0);
	KUNIT_EXPECT_EQ(test, part.seq, 0);
	KUNIT_EXPECT_FALSE(test, drm_rect_visible(&part.box));
}

static void ebc_test_pending_add(struct kunit *test)
{
	struct rk3026_ebc_update a = {
		.clip = DRM_RECT_INIT(2, 1, 4, 2), .mode = EBC_MODE_A2,
		.marker = 7, .temp = 20, .temp_valid = true,
		.source = EBC_SOURCE_MXCFB,
	};
	struct rk3026_ebc_update b = {
		.clip = DRM_RECT_INIT(9, 3, 3, 3), .mode = EBC_MODE_PART,
		.source = EBC_SOURCE_PLANE,
	};
	struct rk3026_ebc_pending p = {};
	struct rk3026_ebc_set part;
	u32 seq;

	KUNIT_EXPECT_FALSE(test, rk3026_ebc_pending_add(&p, &a, &seq));
	KUNIT_EXPECT_EQ(test, seq, 1);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_pending_add(&p, &b, &seq));
	KUNIT_EXPECT_EQ(test, seq, 2);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_pending_empty(&p));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_work_pending(&p));
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&p), 1);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&p, &part));
	KUNIT_EXPECT_EQ(test, part.box.x1, 2);
	KUNIT_EXPECT_EQ(test, part.box.y1, 1);
	KUNIT_EXPECT_EQ(test, part.box.x2, 12);
	KUNIT_EXPECT_EQ(test, part.box.y2, 6);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, part.seq, 2);
	KUNIT_EXPECT_EQ(test, part.merged, 2);
	/* The newest valid temperature: the plane send had none. */
	KUNIT_EXPECT_TRUE(test, part.temp_valid);
	KUNIT_EXPECT_EQ(test, part.temp, 20);
	KUNIT_EXPECT_EQ(test, part.source, EBC_SOURCE_PLANE);
	/* The newest update's marker, even when it had none. */
	KUNIT_EXPECT_EQ(test, part.marker, 0);
	KUNIT_EXPECT_FALSE(test, part.has_mxcfb_full);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_pending_empty(&p));
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&p), 0);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_pending_take(&p, &part));
}

static void ebc_test_pending_temp_newest(struct kunit *test)
{
	struct rk3026_ebc_update upd = t_upd(&t_panel, EBC_MODE_PART, 0,
					     EBC_SOURCE_MXCFB);
	struct rk3026_ebc_pending p = {};
	struct rk3026_ebc_set part;
	u32 seq;

	upd.temp_valid = true;
	upd.temp = 10;
	rk3026_ebc_pending_add(&p, &upd, &seq);
	upd.temp = 30;
	rk3026_ebc_pending_add(&p, &upd, &seq);
	upd.temp = 0;
	upd.temp_valid = false;
	rk3026_ebc_pending_add(&p, &upd, &seq);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_TRUE(test, part.temp_valid);
	KUNIT_EXPECT_EQ(test, part.temp, 30);
}

static bool t_full_flag(u32 mode, enum rk3026_ebc_source source)
{
	struct rk3026_ebc_pending p = {};
	struct rk3026_ebc_set part;

	t_add(&p, &t_panel, EBC_MODE_A2, EBC_SOURCE_PLANE);
	t_add(&p, &t_panel, mode, source);
	rk3026_ebc_pending_take(&p, &part);
	return part.has_mxcfb_full;
}

static void ebc_test_pending_mxcfb_full(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, t_full_flag(EBC_MODE_FULL, EBC_SOURCE_MXCFB));
	KUNIT_EXPECT_TRUE(test,
			  t_full_flag(EBC_MODE_OED_PART, EBC_SOURCE_MXCFB));
	KUNIT_EXPECT_FALSE(test, t_full_flag(EBC_MODE_PART, EBC_SOURCE_MXCFB));
	KUNIT_EXPECT_FALSE(test, t_full_flag(EBC_MODE_A2, EBC_SOURCE_MXCFB));
	KUNIT_EXPECT_FALSE(test, t_full_flag(EBC_MODE_FULL, EBC_SOURCE_PLANE));
}

/* --- the snapshot --- */

/*
 * rk3026_ebc_build_snapshot() on the test images; the mode it returns is
 * checked by ebc_test_snapshot_a2_over_grey().
 */
static bool t_snapshot(u8 *dst, u8 *prev, const u8 *next,
		       const struct drm_rect *box, u32 mode)
{
	const struct rk3026_ebc_shadows sh = {
		.prev = prev, .scan = dst, .pitch = T_PITCH, .height = T_H,
	};

	return rk3026_ebc_build_snapshot(&sh, next, box, &mode);
}

static void ebc_test_snapshot_outside_box(struct kunit *test)
{
	struct drm_rect box = DRM_RECT_INIT(4, 2, 6, 2);
	u8 *prev = t_img_alloc(test, 5);
	u8 *next = t_img_alloc(test, 12);
	u8 *dst = t_img_alloc(test, 0);
	int x, y;

	KUNIT_EXPECT_TRUE(test,
			  t_snapshot(dst, prev, next, &box, EBC_MODE_PART));
	for (y = 0; y < T_H; y++)
		for (x = 0; x < T_W; x++)
			KUNIT_EXPECT_EQ(test, t_px(dst, x, y),
					t_in_rect(&box, x, y) ? 12 : 5);
	t_expect_guard(test, dst);
}

static void ebc_test_snapshot_keeps_equal(struct kunit *test)
{
	struct drm_rect box = DRM_RECT_INIT(0, 0, 8, 2);
	u8 *prev = t_img_alloc(test, 6);
	u8 *next = t_img_alloc(test, 6);
	u8 *dst = t_img_alloc(test, 0);

	t_px_set(next, 1, 0, 3);
	t_px_set(next, 6, 1, 13);
	t_snapshot(dst, prev, next, &box, EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, t_px(dst, 1, 0), 3);
	KUNIT_EXPECT_EQ(test, t_px(dst, 6, 1), 13);
	KUNIT_EXPECT_EQ(test, t_px(dst, 0, 0), 6);
	KUNIT_EXPECT_EQ(test, t_px(dst, 7, 1), 6);
}

/* DU and A2 threshold the pixels they lay over; nothing else does. */
static void ebc_test_snapshot_two_level(struct kunit *test)
{
	static const u32 two_level[] = { EBC_MODE_A2, EBC_MODE_BLACK_WHITE };
	static const u32 sixteen[] = {
		EBC_MODE_PART, EBC_MODE_OED_PART, EBC_MODE_FULL, EBC_MODE_RESET,
	};
	/* Odd edges: x 3..4, widened to the bytes of pixels 2..5. */
	struct drm_rect box = DRM_RECT_INIT(3, 1, 2, 2);
	u8 *prev = t_img_alloc(test, 6);
	u8 *next = t_img_alloc(test, 6);
	u8 *dst = t_img_alloc(test, 0);
	unsigned int i;
	int y;

	for (y = 1; y < 3; y++) {
		t_px_set(next, 3, y, 9);
		t_px_set(next, 4, y, 7);
	}
	for (i = 0; i < ARRAY_SIZE(two_level); i++) {
		t_snapshot(dst, prev, next, &box, two_level[i]);
		for (y = 1; y < 3; y++) {
			KUNIT_EXPECT_EQ(test, t_px(dst, 3, y), 15);
			KUNIT_EXPECT_EQ(test, t_px(dst, 4, y), 0);
			/* The untouched greys of the widened bytes survive. */
			KUNIT_EXPECT_EQ(test, t_px(dst, 2, y), 6);
			KUNIT_EXPECT_EQ(test, t_px(dst, 5, y), 6);
		}
	}
	for (i = 0; i < ARRAY_SIZE(sixteen); i++) {
		t_snapshot(dst, prev, next, &box, sixteen[i]);
		KUNIT_EXPECT_EQ(test, t_px(dst, 3, 1), 9);
		KUNIT_EXPECT_EQ(test, t_px(dst, 4, 1), 7);
	}
	/* So does an untouched grey well inside a box. */
	box = t_panel;
	t_px_set(next, 10, 4, 1);
	t_snapshot(dst, prev, next, &box, EBC_MODE_A2);
	KUNIT_EXPECT_EQ(test, t_px(dst, 10, 4), 0);
	KUNIT_EXPECT_EQ(test, t_px(dst, 11, 4), 6);
	KUNIT_EXPECT_EQ(test, t_px(dst, 0, 0), 6);
}

static void ebc_test_snapshot_restores_greys(struct kunit *test)
{
	struct drm_rect box = DRM_RECT_INIT(0, 0, 8, 3);
	u8 *prev = t_img_alloc(test, 15);
	u8 *next = t_img_alloc(test, 15);
	u8 *dst = t_img_alloc(test, 0);
	int x, y;

	for (y = 0; y < 3; y++)
		for (x = 0; x < 8; x++)
			t_px_set(next, x, y, (x + y) & 0xf);

	/* A DU scan draws the greys as 0/15... */
	t_snapshot(dst, prev, next, &box, EBC_MODE_BLACK_WHITE);
	for (y = 0; y < 3; y++)
		for (x = 0; x < 8; x++) {
			u8 want = (x + y) >= 8 ? 15 : 0;

			KUNIT_EXPECT_EQ(test, t_px(dst, x, y), want);
		}

	/* ...it is now on the glass, and a PART scan of the rect restores them. */
	memcpy(prev, dst, T_SIZE);
	KUNIT_EXPECT_TRUE(test,
			  t_snapshot(dst, prev, next, &box, EBC_MODE_PART));
	KUNIT_EXPECT_EQ(test, memcmp(dst, next, T_SIZE), 0);
}

static void ebc_test_snapshot_edges(struct kunit *test)
{
	/* Its odd left edge widens to the byte: pixel T_W - 4 differs too. */
	struct drm_rect corner = DRM_RECT_INIT(T_W - 3, T_H - 1, 3, 1);
	struct drm_rect wide = DRM_RECT_INIT(T_W - 4, T_H - 1, 4, 1);
	struct drm_rect over = { .x1 = -5, .y1 = -2, .x2 = T_W + 7,
				 .y2 = T_H + 3 };
	struct drm_rect empty = DRM_RECT_INIT(4, 4, 0, 2);
	struct drm_rect off = DRM_RECT_INIT(T_W, 0, 4, 2);
	u8 *prev = t_img_alloc(test, 2);
	u8 *next = t_img_alloc(test, 11);
	u8 *dst = t_img_alloc(test, 0);
	int x, y;

	t_snapshot(dst, prev, next, &corner, EBC_MODE_PART);
	for (y = 0; y < T_H; y++)
		for (x = 0; x < T_W; x++)
			KUNIT_EXPECT_EQ(test, t_px(dst, x, y),
					t_in_rect(&wide, x, y) ? 11 : 2);
	t_expect_guard(test, dst);

	/* A box past every edge is clipped to the panel. */
	t_snapshot(dst, prev, next, &over, EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, memcmp(dst, next, T_SIZE), 0);
	t_expect_guard(test, dst);

	/* An empty box or one outside the panel copies prev. */
	KUNIT_EXPECT_FALSE(test, t_snapshot(dst, prev, next, &empty,
					    EBC_MODE_PART));
	KUNIT_EXPECT_EQ(test, memcmp(dst, prev, T_SIZE), 0);
	KUNIT_EXPECT_FALSE(test, t_snapshot(dst, prev, next, &off,
					    EBC_MODE_PART));
	KUNIT_EXPECT_EQ(test, memcmp(dst, prev, T_SIZE), 0);
	t_expect_guard(test, dst);
}

/* What the snapshot returns: whether the scan moves anything. */
static void ebc_test_snapshot_moves(struct kunit *test)
{
	struct drm_rect box = DRM_RECT_INIT(2, 1, 5, 3);
	struct drm_rect over = { .x1 = -5, .y1 = -2, .x2 = T_W + 7,
				 .y2 = T_H + 3 };
	u8 *prev = t_img_alloc(test, 6);
	u8 *next = t_img_alloc(test, 6);
	u8 *dst = t_img_alloc(test, 0);

	/* next == prev in the box: nothing moves; outside it does not count */
	t_px_set(next, 0, 0, 1);
	KUNIT_EXPECT_FALSE(test, t_snapshot(dst, prev, next, &box,
					    EBC_MODE_PART));

	/* one pixel in the box differs, on its last line and column */
	t_px_set(next, 6, 3, 9);
	KUNIT_EXPECT_TRUE(test,
			  t_snapshot(dst, prev, next, &box, EBC_MODE_PART));

	/* a two-level threshold that lands on prev moves nothing either */
	t_img_fill(prev, 15);
	t_img_fill(next, 15);
	t_px_set(next, 3, 2, 12);
	KUNIT_EXPECT_FALSE(test,
			   t_snapshot(dst, prev, next, &box, EBC_MODE_A2));
	KUNIT_EXPECT_TRUE(test,
			  t_snapshot(dst, prev, next, &box, EBC_MODE_PART));

	/* a box past every edge is clipped */
	t_img_fill(next, 15);
	t_px_set(next, T_W - 1, T_H - 1, 0);
	KUNIT_EXPECT_TRUE(test, t_snapshot(dst, prev, next, &over,
					   EBC_MODE_PART));
	t_expect_guard(test, dst);
}

/*
 * A2 does not move a pixel off a grey, DU does: an A2 scan that lays a pixel
 * over a grey of prev runs DU, with the same thresholded snapshot.
 */
static void ebc_test_snapshot_a2_over_grey(struct kunit *test)
{
	static const u32 kept[] = {
		EBC_MODE_BLACK_WHITE, EBC_MODE_PART, EBC_MODE_OED_PART,
		EBC_MODE_FULL, EBC_MODE_RESET,
	};
	struct drm_rect box = DRM_RECT_INIT(2, 1, 4, 2);
	u8 *prev = t_img_alloc(test, 15);
	u8 *next = t_img_alloc(test, 15);
	u8 *dst = t_img_alloc(test, 0);
	u8 *du = t_img_alloc(test, 0);
	const struct rk3026_ebc_shadows sh = {
		.prev = prev, .scan = dst, .pitch = T_PITCH, .height = T_H,
	};
	unsigned int i;
	u32 mode;

	/* 0 and 15 under every laid-over pixel: A2 stays. */
	t_px_set(next, 3, 1, 0);
	t_px_set(prev, 4, 2, 0);
	t_px_set(next, 4, 2, 9);
	/* A grey outside the box, or one the scan keeps, does not count. */
	t_px_set(prev, 0, 0, 7);
	t_px_set(next, 0, 0, 0);
	t_px_set(prev, 5, 1, 7);
	t_px_set(next, 5, 1, 7);
	mode = EBC_MODE_A2;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_build_snapshot(&sh, next, &box,
							  &mode));
	KUNIT_EXPECT_EQ(test, mode, EBC_MODE_A2);
	KUNIT_EXPECT_EQ(test, t_px(dst, 3, 1), 0);
	KUNIT_EXPECT_EQ(test, t_px(dst, 4, 2), 15);
	KUNIT_EXPECT_EQ(test, t_px(dst, 5, 1), 7);

	/* One laid-over pixel on a grey, a grey target too: DU. */
	t_px_set(prev, 2, 2, 6);
	t_px_set(next, 2, 2, 12);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_build_snapshot(&sh, next, &box,
							  &mode));
	KUNIT_EXPECT_EQ(test, mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, t_px(dst, 2, 2), 15);
	KUNIT_EXPECT_EQ(test, t_px(dst, 5, 1), 7);
	t_snapshot(du, prev, next, &box, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, memcmp(dst, du, T_SIZE), 0);
	t_expect_guard(test, dst);

	/* No other mode is raised. */
	for (i = 0; i < ARRAY_SIZE(kept); i++) {
		mode = kept[i];
		rk3026_ebc_build_snapshot(&sh, next, &box, &mode);
		KUNIT_EXPECT_EQ(test, mode, kept[i]);
	}
}

/* --- skip and scan mode --- */

static void ebc_test_skip(struct kunit *test)
{
	struct rk3026_ebc_pending p = {};
	struct rk3026_ebc_set part;

	/* Plane and fbcon updates in a full-class mode still skip. */
	t_add(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_should_skip(&part, false, true));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_should_skip(&part, false, false));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_should_skip(&part, true, true));

	/* A partial mxcfb send skips, a full one does not. */
	t_add(&p, &t_panel, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_should_skip(&part, false, true));
	t_add(&p, &t_panel, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	t_add(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_should_skip(&part, false, true));

	/* RESET is never skipped. */
	t_add(&p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_ENABLE);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_RESET);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_should_skip(&part, false, true));
}

static void ebc_test_scan_mode(struct kunit *test)
{
	static const u32 modes[] = {
		EBC_MODE_A2, EBC_MODE_BLACK_WHITE, EBC_MODE_PART,
		EBC_MODE_OED_PART, EBC_MODE_FULL, EBC_MODE_RESET,
	};
	static const u32 repaired[] = {
		EBC_MODE_PART, EBC_MODE_PART, EBC_MODE_PART,
		EBC_MODE_OED_PART, EBC_MODE_FULL, EBC_MODE_RESET,
	};
	struct rk3026_ebc_set part = {};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		part.mode = modes[i];
		KUNIT_EXPECT_EQ(test, rk3026_ebc_scan_mode(&part, false),
				modes[i]);
		KUNIT_EXPECT_EQ(test, rk3026_ebc_scan_mode(&part, true),
				repaired[i]);
	}
}

/* --- sequences of add, take and finish --- */

struct t_ctx {
	struct rk3026_ebc_pending	p;
	struct rk3026_ebc_marker_ring	ring;
	struct rk3026_ebc_shadows	sh;
	u8				*prev;
	u8				*scan;
};

static struct t_ctx *t_ctx_alloc(struct kunit *test)
{
	struct t_ctx *c = kunit_kzalloc(test, sizeof(*c), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, c);
	c->prev = t_img_alloc(test, 7);
	c->scan = t_img_alloc(test, 3);
	c->sh.prev = c->prev;
	c->sh.scan = c->scan;
	c->sh.pitch = T_PITCH;
	c->sh.height = T_H;
	return c;
}

/* The update queued with @seq has finished: what the driver's wait tests. */
static bool t_done(const struct t_ctx *c, u32 seq)
{
	return rk3026_ebc_seq_finished(&c->p, seq);
}

static void t_finish(struct t_ctx *c, const struct rk3026_ebc_set *part,
		     enum rk3026_ebc_outcome outcome)
{
	rk3026_ebc_scan_finish(&c->sh, &c->p, part, outcome, EBC_REPAIR_NONE);
}

static void ebc_test_seq_barrier(struct kunit *test)
{
	struct t_ctx *c = t_ctx_alloc(test);
	struct drm_rect r = DRM_RECT_INIT(0, 0, 4, 4);
	struct rk3026_ebc_set part;
	u32 s1, s2, s3, s5, seq;

	s1 = t_add_marked(&c->p, &c->ring, &r, EBC_MODE_PART, 1);
	s2 = t_add_marked(&c->p, &c->ring, &t_panel, EBC_MODE_RESET, 2);
	s3 = t_add_marked(&c->p, &c->ring, &r, EBC_MODE_PART, 3);
	t_add_marked(&c->p, &c->ring, &r, EBC_MODE_FULL, 4);
	KUNIT_EXPECT_EQ(test, c->p.post.merged, 2);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&c->p), 3);
	/* The ring maps each marker to its update's seq. */
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_marker_seq_lookup(&c->ring, 2, &seq));
	KUNIT_EXPECT_EQ(test, seq, s2);

	/* pre first, finished below the RESET's seq */
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, part.merged, 1);
	KUNIT_EXPECT_EQ(test, part.marker, 1);
	t_finish(c, &part, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_TRUE(test, t_done(c, s1));
	KUNIT_EXPECT_FALSE(test, t_done(c, s2));
	KUNIT_EXPECT_FALSE(test, t_done(c, s3));

	/* then the RESET alone */
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, part.seq, s2);
	KUNIT_EXPECT_EQ(test, part.merged, 1);
	/* a send now joins what was post: the RESET is no longer pending */
	s5 = t_add_marked(&c->p, &c->ring, &r, EBC_MODE_A2, 5);
	KUNIT_EXPECT_EQ(test, c->p.pre.merged, 3);
	KUNIT_EXPECT_EQ(test, c->p.post.merged, 0);
	t_finish(c, &part, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_TRUE(test, t_done(c, s2));
	KUNIT_EXPECT_FALSE(test, t_done(c, s3));

	/* then post, in one scan, in its strongest mode */
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, part.merged, 3);
	KUNIT_EXPECT_EQ(test, part.seq, c->p.queued_seq);
	KUNIT_EXPECT_EQ(test, part.marker, 5);
	t_finish(c, &part, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_TRUE(test, t_done(c, s3));
	KUNIT_EXPECT_TRUE(test, t_done(c, s5));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_pending_take(&c->p, &part));
}

static void ebc_test_seq_reset_pending_twice(struct kunit *test)
{
	struct t_ctx *c = t_ctx_alloc(test);
	struct rk3026_ebc_update reset = t_upd(&t_panel, EBC_MODE_RESET, 3,
					       EBC_SOURCE_MXCFB);
	struct rk3026_ebc_set part;
	u32 s2, seq;

	t_add_marked(&c->p, &c->ring, &t_panel, EBC_MODE_RESET, 1);
	s2 = t_add_marked(&c->p, &c->ring, &t_panel, EBC_MODE_PART, 2);
	/* A second RESET absorbs the post set it would whiten anyway. */
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_pending_add(&c->p, &reset, &seq));
	KUNIT_EXPECT_EQ(test, c->p.post.merged, 0);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, part.seq, seq);
	KUNIT_EXPECT_EQ(test, part.merged, 3);
	t_finish(c, &part, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_TRUE(test, t_done(c, s2));
	KUNIT_EXPECT_TRUE(test, t_done(c, seq));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_pending_empty(&c->p));
}

static void ebc_test_seq_merged_markers(struct kunit *test)
{
	struct t_ctx *c = t_ctx_alloc(test);
	struct rk3026_ebc_set part;
	u32 seq[4], m;

	for (m = 0; m < 3; m++)
		seq[m] = t_add_marked(&c->p, &c->ring, &t_panel, EBC_MODE_PART,
				      10 + m);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
	KUNIT_EXPECT_EQ(test, part.merged, 3);
	/* Sent while the scan runs: drawn by the next one. */
	seq[3] = t_add_marked(&c->p, &c->ring, &t_panel, EBC_MODE_PART, 13);
	for (m = 0; m < 4; m++)
		KUNIT_EXPECT_FALSE(test, t_done(c, seq[m]));

	t_finish(c, &part, EBC_OUTCOME_DRIVEN);
	for (m = 0; m < 3; m++)
		KUNIT_EXPECT_TRUE(test, t_done(c, seq[m]));
	KUNIT_EXPECT_FALSE(test, t_done(c, seq[3]));
}

/*
 * What the driver asks locklessly between parts: work pending
 * (rk3026_ebc_work_pending(), the thread's wake) must agree with
 * rk3026_ebc_pending_empty() on every outcome, a carry included, or the
 * thread would sleep on work or spin on a carry.
 */
static void t_expect_between_parts(struct kunit *test,
				   const struct rk3026_ebc_pending *p)
{
	KUNIT_EXPECT_EQ(test, rk3026_ebc_work_pending(p),
			!rk3026_ebc_pending_empty(p));
}

static void ebc_test_seq_between_parts(struct kunit *test)
{
	static const enum rk3026_ebc_outcome outcomes[] = {
		EBC_OUTCOME_DRIVEN, EBC_OUTCOME_SKIPPED, EBC_OUTCOME_CRTC_OFF,
		EBC_OUTCOME_NO_RAILS, EBC_OUTCOME_NO_LUT,
		EBC_OUTCOME_NO_WAVEFORM, EBC_OUTCOME_TIMEOUT,
		EBC_OUTCOME_ABANDONED,
	};
	struct drm_rect r = DRM_RECT_INIT(2, 2, 4, 2);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(outcomes); i++) {
		struct t_ctx *c = t_ctx_alloc(test);
		struct rk3026_ebc_set part;
		unsigned int parts = 0;
		u32 before, last;

		t_expect_between_parts(test, &c->p);
		t_add_marked(&c->p, &c->ring, &r, EBC_MODE_A2, 1);
		t_add_marked(&c->p, &c->ring, &t_panel, EBC_MODE_RESET, 2);
		last = t_add_marked(&c->p, &c->ring, &r, EBC_MODE_PART, 3);
		t_expect_between_parts(test, &c->p);

		while (rk3026_ebc_pending_take(&c->p, &part)) {
			before = c->p.done_seq;
			/* in flight: still work, so not idle */
			KUNIT_EXPECT_TRUE(test, rk3026_ebc_work_pending(&c->p));
			rk3026_ebc_scan_finish(&c->sh, &c->p, &part,
					       outcomes[i], EBC_REPAIR_NONE);
			/* done_seq advanced, whatever the outcome */
			KUNIT_EXPECT_EQ(test, c->p.done_seq, part.seq);
			KUNIT_EXPECT_GT(test, (s32)(c->p.done_seq - before), 0);
			t_expect_between_parts(test, &c->p);
			parts++;
		}
		KUNIT_EXPECT_EQ(test, parts, 3);
		KUNIT_EXPECT_EQ(test, c->p.done_seq, c->p.queued_seq);
		KUNIT_EXPECT_TRUE(test, t_done(c, last));
	}
}

/* A RESET sent while a part is in flight waits behind it, then post. */
static void ebc_test_seq_reset_in_flight(struct kunit *test)
{
	struct t_ctx *c = t_ctx_alloc(test);
	struct drm_rect r = DRM_RECT_INIT(2, 2, 4, 2);
	struct rk3026_ebc_set part;
	u32 s1, s2, s3;

	s1 = t_add(&c->p, &r, EBC_MODE_A2, EBC_SOURCE_MXCFB);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
	s2 = t_add(&c->p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_MXCFB);
	s3 = t_add(&c->p, &r, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_EQ(test, c->p.pre.merged, 0);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&c->p), 2);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_work_pending(&c->p));

	t_finish(c, &part, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, c->p.done_seq, s1);
	t_expect_between_parts(test, &c->p);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_RESET);
	t_finish(c, &part, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, c->p.done_seq, s2);
	KUNIT_EXPECT_FALSE(test, t_done(c, s3));

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_PART);
	t_finish(c, &part, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_TRUE(test, t_done(c, s3));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&c->p));
}

static void ebc_test_finish_driven(struct kunit *test)
{
	struct t_ctx *c = t_ctx_alloc(test);
	struct rk3026_ebc_set part;

	c->sh.prev_unknown = true;
	t_add(&c->p, &t_panel, EBC_MODE_PART, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_scan_finish(&c->sh, &c->p, &part,
						       EBC_OUTCOME_DRIVEN,
						       EBC_REPAIR_NONE));
	KUNIT_EXPECT_PTR_EQ(test, c->sh.prev, c->scan);
	KUNIT_EXPECT_PTR_EQ(test, c->sh.scan, c->prev);
	KUNIT_EXPECT_FALSE(test, c->sh.prev_unknown);
	KUNIT_EXPECT_EQ(test, c->p.done_seq, part.seq);
	KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
}

/* The part's own waveform ran, a repair or not: the snapshot is prev. */
static void ebc_test_finish_entered(struct kunit *test)
{
	static const enum rk3026_ebc_outcome outcomes[] = {
		EBC_OUTCOME_DRIVEN, EBC_OUTCOME_TIMEOUT, EBC_OUTCOME_ABANDONED,
	};
	static const enum rk3026_ebc_repair repairs[] = {
		EBC_REPAIR_NONE, EBC_REPAIR_DONE,
	};
	unsigned int i, j;

	for (i = 0; i < ARRAY_SIZE(outcomes); i++)
		for (j = 0; j < ARRAY_SIZE(repairs); j++) {
			struct t_ctx *c = t_ctx_alloc(test);
			struct rk3026_ebc_set part;

			c->sh.prev_unknown = repairs[j] == EBC_REPAIR_DONE;
			t_add(&c->p, &t_panel, EBC_MODE_PART, EBC_SOURCE_PLANE);
			rk3026_ebc_pending_take(&c->p, &part);
			/* Partly driven: the snapshot is the best prev there is. */
			KUNIT_EXPECT_TRUE(test,
					  rk3026_ebc_scan_finish(&c->sh, &c->p,
								 &part,
								 outcomes[i],
								 repairs[j]));
			KUNIT_EXPECT_PTR_EQ(test, c->sh.prev, c->scan);
			KUNIT_EXPECT_EQ(test, c->sh.prev_unknown,
					outcomes[i] != EBC_OUTCOME_DRIVEN);
			KUNIT_EXPECT_EQ(test, c->p.done_seq, part.seq);
			KUNIT_EXPECT_FALSE(test,
					   c->p.carry.merged);
		}
}

/*
 * The repair RESET was cut short: the part's waveform never ran and the
 * glass is not known. prev stays, marked, and the part is carried into the
 * scan that repairs the glass.
 */
static void ebc_test_finish_repair_failed(struct kunit *test)
{
	static const enum rk3026_ebc_outcome outcomes[] = {
		EBC_OUTCOME_TIMEOUT, EBC_OUTCOME_ABANDONED,
	};
	struct drm_rect r = DRM_RECT_INIT(4, 1, 4, 2);
	struct drm_rect s = DRM_RECT_INIT(12, 4, 2, 2);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(outcomes); i++) {
		struct t_ctx *c = t_ctx_alloc(test);
		struct rk3026_ebc_set part;
		u32 seq;

		c->sh.prev_unknown = true;
		seq = t_add_marked(&c->p, &c->ring, &r, EBC_MODE_FULL, 1);
		rk3026_ebc_pending_take(&c->p, &part);
		KUNIT_EXPECT_FALSE(test,
				   rk3026_ebc_scan_finish(&c->sh, &c->p, &part,
							  outcomes[i],
							  EBC_REPAIR_FAILED));
		KUNIT_EXPECT_PTR_EQ(test, c->sh.prev, c->prev);
		KUNIT_EXPECT_EQ(test, c->prev[0], 0x77);
		KUNIT_EXPECT_TRUE(test, c->sh.prev_unknown);
		KUNIT_EXPECT_TRUE(test, t_done(c, seq));
		KUNIT_EXPECT_TRUE(test, drm_rect_equals(&c->p.carry.box, &r));
		KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&c->p));

		/* The next part draws it, in its mode, still a full send. */
		t_add(&c->p, &s, EBC_MODE_A2, EBC_SOURCE_MXCFB);
		KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
		KUNIT_EXPECT_EQ(test, part.box.x1, 4);
		KUNIT_EXPECT_EQ(test, part.box.y1, 1);
		KUNIT_EXPECT_EQ(test, part.box.x2, 14);
		KUNIT_EXPECT_EQ(test, part.box.y2, 6);
		KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_FULL);
		KUNIT_EXPECT_TRUE(test, part.has_mxcfb_full);
		/* ...after the repair, which the mark asks for. */
		KUNIT_EXPECT_EQ(test, rk3026_ebc_scan_mode(&part, true),
				EBC_MODE_FULL);
	}
}

static void ebc_test_finish_carry(struct kunit *test)
{
	static const enum rk3026_ebc_outcome outcomes[] = {
		EBC_OUTCOME_NO_RAILS, EBC_OUTCOME_NO_LUT,
		EBC_OUTCOME_NO_WAVEFORM,
	};
	struct drm_rect a = DRM_RECT_INIT(0, 0, 4, 2);
	struct drm_rect b = DRM_RECT_INIT(8, 4, 2, 2);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(outcomes); i++) {
		struct t_ctx *c = t_ctx_alloc(test);
		struct rk3026_ebc_set part;
		u32 seq;

		seq = t_add_marked(&c->p, &c->ring, &a, EBC_MODE_FULL, 1);
		rk3026_ebc_pending_take(&c->p, &part);
		KUNIT_EXPECT_FALSE(test, rk3026_ebc_scan_finish(&c->sh, &c->p,
								&part,
								outcomes[i],
								EBC_REPAIR_NONE));
		/* Nothing driven: prev stays, the wait ends, nothing wakes. */
		KUNIT_EXPECT_PTR_EQ(test, c->sh.prev, c->prev);
		KUNIT_EXPECT_TRUE(test, t_done(c, seq));
		KUNIT_EXPECT_TRUE(test, rk3026_ebc_pending_empty(&c->p));
		KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&c->p));
		KUNIT_EXPECT_FALSE(test, rk3026_ebc_pending_take(&c->p, &part));

		/* The next update's scan draws the carried box, as strongly. */
		t_add(&c->p, &b, EBC_MODE_A2, EBC_SOURCE_PLANE);
		KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
		KUNIT_EXPECT_EQ(test, part.box.x1, 0);
		KUNIT_EXPECT_EQ(test, part.box.y1, 0);
		KUNIT_EXPECT_EQ(test, part.box.x2, 10);
		KUNIT_EXPECT_EQ(test, part.box.y2, 6);
		KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_FULL);
		KUNIT_EXPECT_TRUE(test, part.has_mxcfb_full);
		KUNIT_EXPECT_EQ(test, part.merged, 1);
		KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
	}
}

/* Two parts not driven in a row: one carry, their union, the stronger mode. */
static void ebc_test_finish_carry_union(struct kunit *test)
{
	struct t_ctx *c = t_ctx_alloc(test);
	struct drm_rect a = DRM_RECT_INIT(0, 0, 2, 2);
	struct drm_rect b = DRM_RECT_INIT(10, 3, 2, 1);
	struct drm_rect s = DRM_RECT_INIT(4, 5, 2, 1);
	struct rk3026_ebc_set part;

	t_add(&c->p, &a, EBC_MODE_OED_PART, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&c->p, &part);
	t_finish(c, &part, EBC_OUTCOME_NO_RAILS);
	t_add(&c->p, &b, EBC_MODE_A2, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&c->p, &part);
	/* The first carry rode in that part: carried again with it. */
	t_finish(c, &part, EBC_OUTCOME_NO_RAILS);
	t_add(&c->p, &s, EBC_MODE_A2, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_EQ(test, part.box.x1, 0);
	KUNIT_EXPECT_EQ(test, part.box.y1, 0);
	KUNIT_EXPECT_EQ(test, part.box.x2, 12);
	KUNIT_EXPECT_EQ(test, part.box.y2, 6);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_OED_PART);
	KUNIT_EXPECT_FALSE(test, part.has_mxcfb_full);

	/* A carry made while one is pending: the union branch. */
	c->p.carry = (struct rk3026_ebc_set) {
		.box = a, .mode = EBC_MODE_A2, .merged = 1,
	};
	part.box = b;
	part.mode = EBC_MODE_PART;
	part.has_mxcfb_full = true;
	t_finish(c, &part, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_EQ(test, c->p.carry.box.x1, 0);
	KUNIT_EXPECT_EQ(test, c->p.carry.box.y1, 0);
	KUNIT_EXPECT_EQ(test, c->p.carry.box.x2, 12);
	KUNIT_EXPECT_EQ(test, c->p.carry.box.y2, 4);
	KUNIT_EXPECT_EQ(test, c->p.carry.mode, EBC_MODE_PART);
	KUNIT_EXPECT_TRUE(test, c->p.carry.has_mxcfb_full);
}

/* A RESET sent drops the carry: it whitens what the carry would draw. */
static void ebc_test_finish_carry_reset(struct kunit *test)
{
	struct t_ctx *c = t_ctx_alloc(test);
	struct drm_rect a = DRM_RECT_INIT(0, 0, 4, 2);
	struct drm_rect s = DRM_RECT_INIT(8, 4, 2, 2);
	struct rk3026_ebc_set part;

	t_add(&c->p, &a, EBC_MODE_FULL, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&c->p, &part);
	t_finish(c, &part, EBC_OUTCOME_NO_RAILS);
	KUNIT_EXPECT_TRUE(test, c->p.carry.merged);

	t_add(&c->p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_RESET);
	t_finish(c, &part, EBC_OUTCOME_DRIVEN);

	t_add(&c->p, &s, EBC_MODE_A2, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&part.box, &s));
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_A2);
	KUNIT_EXPECT_FALSE(test, part.has_mxcfb_full);
}

static void ebc_test_finish_skipped_crtc_off(struct kunit *test)
{
	static const enum rk3026_ebc_outcome outcomes[] = {
		EBC_OUTCOME_SKIPPED, EBC_OUTCOME_CRTC_OFF,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(outcomes); i++) {
		struct t_ctx *c = t_ctx_alloc(test);
		struct rk3026_ebc_set part;
		u32 seq;

		seq = t_add_marked(&c->p, &c->ring, &t_panel, EBC_MODE_PART, 1);
		rk3026_ebc_pending_take(&c->p, &part);
		KUNIT_EXPECT_FALSE(test, rk3026_ebc_scan_finish(&c->sh, &c->p,
								&part,
								outcomes[i],
								EBC_REPAIR_NONE));
		KUNIT_EXPECT_PTR_EQ(test, c->sh.prev, c->prev);
		KUNIT_EXPECT_TRUE(test, t_done(c, seq));
		/* Dropped, not carried. */
		KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
	}
}

static void ebc_test_finish_reset(struct kunit *test)
{
	struct t_ctx *c = t_ctx_alloc(test);
	struct rk3026_ebc_set part;

	c->sh.prev_unknown = true;
	t_add(&c->p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_ENABLE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_scan_finish(&c->sh, &c->p, &part,
						       EBC_OUTCOME_DRIVEN,
						       EBC_REPAIR_NONE));
	t_expect_white(test, c->sh.prev);
	KUNIT_EXPECT_FALSE(test, c->sh.prev_unknown);
	KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
	t_expect_guard(test, c->sh.prev);
}

static void ebc_test_finish_reset_failed(struct kunit *test)
{
	static const enum rk3026_ebc_outcome outcomes[] = {
		EBC_OUTCOME_NO_RAILS, EBC_OUTCOME_NO_LUT,
		EBC_OUTCOME_NO_WAVEFORM,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(outcomes); i++) {
		struct t_ctx *c = t_ctx_alloc(test);
		struct rk3026_ebc_set part;

		t_add(&c->p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_ENABLE);
		rk3026_ebc_pending_take(&c->p, &part);
		KUNIT_EXPECT_FALSE(test, rk3026_ebc_scan_finish(&c->sh, &c->p,
								&part,
								outcomes[i],
								EBC_REPAIR_NONE));
		/* The glass is not known: the next draw repairs it first. */
		KUNIT_EXPECT_TRUE(test, c->sh.prev_unknown);
		KUNIT_EXPECT_PTR_EQ(test, c->sh.prev, c->prev);
		KUNIT_EXPECT_EQ(test, c->prev[0], 0x77);
		KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
		KUNIT_EXPECT_EQ(test, c->p.done_seq, part.seq);
	}
}

/*
 * A RESET cut short leaves the white it drove towards, marked unknown: the
 * next draw repairs the glass and lays its box over white, as the target
 * has it, not over what the glass showed before the RESET.
 */
static void ebc_test_finish_reset_cut_short(struct kunit *test)
{
	static const enum rk3026_ebc_outcome outcomes[] = {
		EBC_OUTCOME_TIMEOUT, EBC_OUTCOME_ABANDONED,
	};
	struct drm_rect r = DRM_RECT_INIT(2, 2, 2, 2);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(outcomes); i++) {
		struct t_ctx *c = t_ctx_alloc(test);
		u8 *next = t_img_alloc(test, 15);
		struct rk3026_ebc_set part;

		t_add(&c->p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_ENABLE);
		rk3026_ebc_pending_take(&c->p, &part);
		KUNIT_EXPECT_TRUE(test,
				  rk3026_ebc_scan_finish(&c->sh, &c->p, &part,
							 outcomes[i],
							 EBC_REPAIR_NONE));
		KUNIT_EXPECT_TRUE(test, c->sh.prev_unknown);
		KUNIT_EXPECT_PTR_EQ(test, c->sh.prev, c->prev);
		t_expect_white(test, c->sh.prev);
		t_expect_guard(test, c->sh.prev);
		KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
		KUNIT_EXPECT_EQ(test, c->p.done_seq, part.seq);

		/* The next draw: its box over white. */
		t_px_set(next, 2, 2, 0);
		KUNIT_EXPECT_TRUE(test, t_snapshot(c->scan, c->prev, next, &r,
						   EBC_MODE_PART));
		KUNIT_EXPECT_EQ(test, t_px(c->scan, 2, 2), 0);
		KUNIT_EXPECT_EQ(test, t_px(c->scan, 0, 0), 15);
		KUNIT_EXPECT_EQ(test, t_px(c->scan, 5, 4), 15);
	}
}

static void ebc_test_finish_repaired_not_driven(struct kunit *test)
{
	struct t_ctx *c = t_ctx_alloc(test);
	struct drm_rect r = DRM_RECT_INIT(2, 2, 2, 2);
	struct rk3026_ebc_set part;

	c->sh.prev_unknown = true;
	t_add(&c->p, &r, EBC_MODE_A2, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_scan_mode(&part, true), EBC_MODE_PART);
	/* The repair whitened the glass, then the LUT was missing. */
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_scan_finish(&c->sh, &c->p, &part,
						       EBC_OUTCOME_NO_LUT,
						       EBC_REPAIR_DONE));
	t_expect_white(test, c->sh.prev);
	KUNIT_EXPECT_FALSE(test, c->sh.prev_unknown);

	t_add(&c->p, &r, EBC_MODE_A2, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&part.box, &t_panel));
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_PART);
}

/* --- seq and markers --- */

static void ebc_test_seq_wrap(struct kunit *test)
{
	struct t_ctx *c = t_ctx_alloc(test);
	struct rk3026_ebc_set part;
	u32 s1, s2, s3;

	c->p.queued_seq = U32_MAX - 1;
	c->p.done_seq = U32_MAX - 1;
	s1 = t_add(&c->p, &t_panel, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	s2 = t_add(&c->p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_MXCFB);
	s3 = t_add(&c->p, &t_panel, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_EQ(test, s1, U32_MAX);
	KUNIT_EXPECT_EQ(test, s2, 0);
	KUNIT_EXPECT_EQ(test, s3, 1);

	KUNIT_EXPECT_FALSE(test, t_done(c, s1));
	rk3026_ebc_pending_take(&c->p, &part);
	t_finish(c, &part, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, c->p.done_seq, U32_MAX);
	KUNIT_EXPECT_TRUE(test, t_done(c, s1));
	KUNIT_EXPECT_FALSE(test, t_done(c, s2));
	KUNIT_EXPECT_FALSE(test, t_done(c, s3));

	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_RESET);
	t_finish(c, &part, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, c->p.done_seq, 0);
	KUNIT_EXPECT_TRUE(test, t_done(c, s2));
	KUNIT_EXPECT_FALSE(test, t_done(c, s3));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_work_pending(&c->p));

	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_done(0, U32_MAX));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_done(U32_MAX, 0));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_done(5, 5));
}

static void ebc_test_marker_zero(struct kunit *test)
{
	struct rk3026_ebc_marker_ring ring = {};
	u32 seq = 1234;

	rk3026_ebc_marker_seq_record(&ring, 0, 7);
	KUNIT_EXPECT_EQ(test, ring.next, 0);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_marker_seq_lookup(&ring, 0, &seq));
	KUNIT_EXPECT_EQ(test, seq, 1234);
	/* Never sent: nothing to wait for. */
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_marker_seq_lookup(&ring, 42, &seq));
}

static void ebc_test_marker_newest(struct kunit *test)
{
	struct rk3026_ebc_marker_ring ring = {};
	u32 seq = 0;

	rk3026_ebc_marker_seq_record(&ring, 7, 100);
	rk3026_ebc_marker_seq_record(&ring, 8, 150);
	rk3026_ebc_marker_seq_record(&ring, 7, 200);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_marker_seq_lookup(&ring, 7, &seq));
	KUNIT_EXPECT_EQ(test, seq, 200);
	/* Done for the older send is not done for the newest. */
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_done(150, seq));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_done(200, seq));
}

static void ebc_test_marker_eviction(struct kunit *test)
{
	struct rk3026_ebc_marker_ring ring = {};
	u32 seq = 0, m;

	/* One past the ring, and the counter past its wrap as well. */
	ring.next = UINT_MAX - 3;
	for (m = 1; m <= EBC_MARKER_RING_LEN + 1; m++)
		rk3026_ebc_marker_seq_record(&ring, m, 1000 + m);

	/* Overwritten: a wait on it returns at once. */
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_marker_seq_lookup(&ring, 1, &seq));
	for (m = 2; m <= EBC_MARKER_RING_LEN + 1; m++) {
		KUNIT_EXPECT_TRUE(test, rk3026_ebc_marker_seq_lookup(&ring, m,
								     &seq));
		KUNIT_EXPECT_EQ(test, seq, 1000 + m);
	}
}

static struct kunit_case rk3026_ebc_core_test_cases[] = {
	KUNIT_CASE(ebc_test_dsp_ctrl_mode),
	KUNIT_CASE(ebc_test_rank_order),
	KUNIT_CASE(ebc_test_rank_unknown),
	KUNIT_CASE(ebc_test_rank_merge),
	KUNIT_CASE(ebc_test_pending_empty),
	KUNIT_CASE(ebc_test_pending_add),
	KUNIT_CASE(ebc_test_pending_temp_newest),
	KUNIT_CASE(ebc_test_pending_mxcfb_full),
	KUNIT_CASE(ebc_test_snapshot_outside_box),
	KUNIT_CASE(ebc_test_snapshot_keeps_equal),
	KUNIT_CASE(ebc_test_snapshot_two_level),
	KUNIT_CASE(ebc_test_snapshot_restores_greys),
	KUNIT_CASE(ebc_test_snapshot_edges),
	KUNIT_CASE(ebc_test_snapshot_moves),
	KUNIT_CASE(ebc_test_snapshot_a2_over_grey),
	KUNIT_CASE(ebc_test_skip),
	KUNIT_CASE(ebc_test_scan_mode),
	KUNIT_CASE(ebc_test_seq_barrier),
	KUNIT_CASE(ebc_test_seq_reset_pending_twice),
	KUNIT_CASE(ebc_test_seq_merged_markers),
	KUNIT_CASE(ebc_test_seq_between_parts),
	KUNIT_CASE(ebc_test_seq_reset_in_flight),
	KUNIT_CASE(ebc_test_finish_driven),
	KUNIT_CASE(ebc_test_finish_entered),
	KUNIT_CASE(ebc_test_finish_repair_failed),
	KUNIT_CASE(ebc_test_finish_carry),
	KUNIT_CASE(ebc_test_finish_carry_union),
	KUNIT_CASE(ebc_test_finish_carry_reset),
	KUNIT_CASE(ebc_test_finish_skipped_crtc_off),
	KUNIT_CASE(ebc_test_finish_reset),
	KUNIT_CASE(ebc_test_finish_reset_failed),
	KUNIT_CASE(ebc_test_finish_reset_cut_short),
	KUNIT_CASE(ebc_test_finish_repaired_not_driven),
	KUNIT_CASE(ebc_test_seq_wrap),
	KUNIT_CASE(ebc_test_marker_zero),
	KUNIT_CASE(ebc_test_marker_newest),
	KUNIT_CASE(ebc_test_marker_eviction),
	{}
};

static struct kunit_suite rk3026_ebc_core_test_suite = {
	.name = "rk3026_ebc_core",
	.test_cases = rk3026_ebc_core_test_cases,
};

kunit_test_suite(rk3026_ebc_core_test_suite);

#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_MXCFB)

/* --- the mxcfb waveform map --- */

struct t_mxcfb {
	const char	*desc;
	u32		waveform;
	u32		update_mode;
	u32		mode;
};

#define T_PARTIAL	MXCFB_UPDATE_MODE_PARTIAL
#define T_FULL		MXCFB_UPDATE_MODE_FULL

static const struct t_mxcfb t_mxcfb_cases[] = {
	{ "INIT partial", MXCFB_WAVEFORM_MODE_INIT, T_PARTIAL, EBC_MODE_RESET },
	{ "INIT full", MXCFB_WAVEFORM_MODE_INIT, T_FULL, EBC_MODE_RESET },
	{ "DU partial", MXCFB_WAVEFORM_MODE_DU, T_PARTIAL,
	  EBC_MODE_BLACK_WHITE },
	{ "DU full", MXCFB_WAVEFORM_MODE_DU, T_FULL, EBC_MODE_BLACK_WHITE },
	{ "GC16 partial", MXCFB_WAVEFORM_MODE_GC16, T_PARTIAL, EBC_MODE_PART },
	{ "GC16 full", MXCFB_WAVEFORM_MODE_GC16, T_FULL, EBC_MODE_FULL },
	{ "GC4 partial", MXCFB_WAVEFORM_MODE_GC4, T_PARTIAL, EBC_MODE_PART },
	{ "GC4 full", MXCFB_WAVEFORM_MODE_GC4, T_FULL, EBC_MODE_PART },
	{ "A2 partial", MXCFB_WAVEFORM_MODE_A2, T_PARTIAL, EBC_MODE_A2 },
	{ "A2 full", MXCFB_WAVEFORM_MODE_A2, T_FULL, EBC_MODE_A2 },
	{ "GL16 partial", MXCFB_WAVEFORM_MODE_GL16, T_PARTIAL, EBC_MODE_PART },
	{ "GL16 full", MXCFB_WAVEFORM_MODE_GL16, T_FULL, EBC_MODE_OED_PART },
	{ "REAGL partial", MXCFB_WAVEFORM_MODE_REAGL, T_PARTIAL,
	  EBC_MODE_PART },
	{ "REAGL full", MXCFB_WAVEFORM_MODE_REAGL, T_FULL, EBC_MODE_FULL },
	{ "REAGLD partial", MXCFB_WAVEFORM_MODE_REAGLD, T_PARTIAL,
	  EBC_MODE_FULL },
	{ "REAGLD full", MXCFB_WAVEFORM_MODE_REAGLD, T_FULL, EBC_MODE_FULL },
	{ "AUTO partial", MXCFB_WAVEFORM_MODE_AUTO, T_PARTIAL, EBC_MODE_PART },
	{ "AUTO full", MXCFB_WAVEFORM_MODE_AUTO, T_FULL, EBC_MODE_FULL },
	{ "unknown partial", 99, T_PARTIAL, EBC_MODE_PART },
	{ "unknown full", 99, T_FULL, EBC_MODE_FULL },
	/* Only UPDATE_MODE_FULL is full. */
	{ "GL16 update_mode 2", MXCFB_WAVEFORM_MODE_GL16, 2, EBC_MODE_PART },
	{ "REAGL update_mode 2", MXCFB_WAVEFORM_MODE_REAGL, 2, EBC_MODE_PART },
	{ "GC16 update_mode 2", MXCFB_WAVEFORM_MODE_GC16, 2, EBC_MODE_PART },
};

KUNIT_ARRAY_PARAM_DESC(t_mxcfb, t_mxcfb_cases, desc);

static void ebc_test_mxcfb_map(struct kunit *test)
{
	const struct t_mxcfb *c = test->param_value;

	KUNIT_EXPECT_EQ(test,
			rk3026_ebc_mxcfb_to_ebc_mode(c->waveform,
						     c->update_mode),
			c->mode);
}

/*
 * The 16-level waveforms' full updates are full class, so they are exempt
 * from the identical-image skip; their partial updates are not.
 */
static void ebc_test_mxcfb_full_class(struct kunit *test)
{
	static const u32 waveforms[] = {
		MXCFB_WAVEFORM_MODE_GC16, MXCFB_WAVEFORM_MODE_GL16,
		MXCFB_WAVEFORM_MODE_REAGL, MXCFB_WAVEFORM_MODE_AUTO,
	};
	unsigned int i;
	u32 mode;

	for (i = 0; i < ARRAY_SIZE(waveforms); i++) {
		mode = rk3026_ebc_mxcfb_to_ebc_mode(waveforms[i], T_FULL);
		KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(mode),
				DSP_DISPLAY_MODE);
		KUNIT_EXPECT_TRUE(test, t_full_flag(mode, EBC_SOURCE_MXCFB));
		mode = rk3026_ebc_mxcfb_to_ebc_mode(waveforms[i], T_PARTIAL);
		KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(mode),
				DSP_UPDATE_MODE | DSP_DISPLAY_MODE);
		KUNIT_EXPECT_FALSE(test, t_full_flag(mode, EBC_SOURCE_MXCFB));
	}
}

static struct kunit_case rk3026_ebc_mxcfb_test_cases[] = {
	KUNIT_CASE_PARAM(ebc_test_mxcfb_map, t_mxcfb_gen_params),
	KUNIT_CASE(ebc_test_mxcfb_full_class),
	{}
};

static struct kunit_suite rk3026_ebc_mxcfb_test_suite = {
	.name = "rk3026_ebc_mxcfb",
	.test_cases = rk3026_ebc_mxcfb_test_cases,
};

kunit_test_suite(rk3026_ebc_mxcfb_test_suite);

#endif /* CONFIG_DRM_RK3026_EBC_MXCFB */

/* --- waveform container bounds --- */

/*
 * A five-node tree: "0" is symbol 0, "10" symbol 1, "11" symbol 2; nodes 3
 * and 4 are inner, 4 the root.
 */
static const s16 t_left[] = { -1, -1, -1, 1, 0 };
static const s16 t_right[] = { -1, -1, -1, 2, 3 };
#define T_ROOT		4

static int t_decode(u8 *dst, size_t dst_len, size_t n, const u8 *src,
		    size_t len)
{
	return rk3026_ebc_rkf_decode(t_left, t_right, T_ROOT, dst, dst_len, n,
				     src, len);
}

static void ebc_test_rkf_decode(struct kunit *test)
{
	/* 0 10 11 0 11: five symbols in exactly one byte. */
	static const u8 src[] = { 0x5b };
	static const u8 want[] = { 0, 1, 2, 0, 2 };
	u8 dst[8];

	memset(dst, 0xee, sizeof(dst));
	KUNIT_EXPECT_EQ(test, t_decode(dst, sizeof(dst), 5, src, 1), 0);
	KUNIT_EXPECT_MEMEQ(test, dst, want, sizeof(want));
	KUNIT_EXPECT_EQ(test, dst[5], 0xee);

	/* Nothing asked, nothing read. */
	KUNIT_EXPECT_EQ(test, t_decode(dst, sizeof(dst), 0, src, 0), 0);
}

/* The stream ends before the last symbol: at a code boundary or inside one. */
static void ebc_test_rkf_decode_truncated(struct kunit *test)
{
	/* 11 11 11 0 1|0: the second byte completes symbol 1. */
	static const u8 src[] = { 0xfd, 0x00 };
	static const u8 one[] = { 0x5b };
	u8 dst[8];

	KUNIT_EXPECT_EQ(test, t_decode(dst, sizeof(dst), 6, one, 1), -EINVAL);
	KUNIT_EXPECT_EQ(test, t_decode(dst, sizeof(dst), 4, src, 1), 0);
	KUNIT_EXPECT_EQ(test, t_decode(dst, sizeof(dst), 5, src, 1), -EINVAL);
	/* The bound is @len, not the bytes that happen to follow. */
	KUNIT_EXPECT_EQ(test, t_decode(dst, sizeof(dst), 5, src, 2), 0);
	KUNIT_EXPECT_EQ(test, dst[4], 1);
}

static void ebc_test_rkf_decode_zero_length(struct kunit *test)
{
	static const u8 src[] = { 0x00 };
	u8 dst[4];

	KUNIT_EXPECT_EQ(test, t_decode(dst, sizeof(dst), 1, src, 0), -EINVAL);
	KUNIT_EXPECT_EQ(test, t_decode(dst, 0, 1, src, 1), -EINVAL);
}

/* More symbols than the output holds: refused before anything is written. */
static void ebc_test_rkf_decode_oversized(struct kunit *test)
{
	static const u8 src[] = { 0x00, 0x00 };
	u8 dst[8];

	memset(dst, 0xee, sizeof(dst));
	KUNIT_EXPECT_EQ(test, t_decode(dst, 4, 5, src, sizeof(src)), -EINVAL);
	KUNIT_EXPECT_EQ(test, dst[0], 0xee);
	KUNIT_EXPECT_EQ(test, t_decode(dst, 5, 5, src, sizeof(src)), 0);
	KUNIT_EXPECT_EQ(test, dst[0], 0);
}

/*
 * Minimal containers: 256 equal Huffman weights, which make every code eight
 * bits long, so one stream byte is one symbol, and all-zero temperature
 * tables, so every temperature takes slot 0 of a mode's offset table. A
 * segment is a frame-count byte, three pad bytes and its stream: 0x40 bytes
 * a frame for GRAY16, MODE6 and GRAYA2, 8 for GRAY2, one per four RESET
 * frames. The offsets mirror RKF_PAYLOAD_OFF and rkf_modes[] in
 * rk3026_ebc_waveform.c, the 4 of T_RKF_SEG its RKF_SEG_HDR.
 */
#define T_RKF_PAYLOAD	0xad0
#define T_RKF_RESET	0x1d0
#define T_RKF_GRAY16	0x2d0
#define T_RKF_GRAYA2	0x3d0
#define T_RKF_MODE6	0x4d0
#define T_RKF_GRAY2	0x5d0
#define T_RKF_SEG	(4 + EBC_LUT_FRAME_BYTES)

/* The vendor's CRC32: MSB-first, polynomial 0x04c10db7, init 0. */
static u32 t_rkf_crc(const u8 *p, size_t n)
{
	u32 crc = 0;
	int k;

	while (n--) {
		crc ^= (u32)*p++ << 24;
		for (k = 0; k < 8; k++)
			crc = (crc << 1) ^ (crc & 0x80000000u ? 0x04c10db7u : 0);
	}
	return crc;
}

/* A container of @size bytes plus its CRC word, in a buffer of @room. */
static u8 *t_rkf_alloc(struct kunit *test, size_t size, size_t room)
{
	static const u32 key[2] = { 0xaa55aa55, 0x55aa55aa };
	u8 *buf = kunit_kzalloc(test, room, GFP_KERNEL);
	int i;

	KUNIT_ASSERT_NOT_NULL(test, buf);
	put_unaligned_le32(size, buf);
	put_unaligned_le32(T_RKF_PAYLOAD, buf + 4);
	memcpy(buf + 8, "rkf:v", 5);
	memcpy(buf + 0x1c, "Einks", 5);
	/* Stored obfuscated, as in a pristine file. */
	for (i = 0; i < 256; i++)
		put_unaligned_le32(1 ^ key[i & 1], buf + 0x6d0 + i * 4);
	return buf;
}

static void t_rkf_seal(u8 *buf, size_t size)
{
	put_unaligned_le32(t_rkf_crc(buf, size), buf + size);
}

/* One GRAY16 segment at @off, with one frame if it fits. */
static u8 *t_rkf_container(struct kunit *test, size_t size, u32 off)
{
	u8 *buf = t_rkf_alloc(test, size, size + 4);

	put_unaligned_le32(off, buf + T_RKF_GRAY16);
	if (off >= T_RKF_PAYLOAD && off < size)
		buf[off] = 1;
	t_rkf_seal(buf, size);
	return buf;
}

static struct device *t_rkf_dev(struct kunit *test)
{
	struct device *dev = kunit_device_register(test, "rk3026-ebc-test");

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	return dev;
}

struct t_rkf_seg {
	u32	table;		/* the mode's offset table */
	u8	frames;
};

static size_t t_rkf_stream(const struct t_rkf_seg *seg)
{
	switch (seg->table) {
	case T_RKF_RESET:
		return (seg->frames + 3) >> 2;
	case T_RKF_GRAY2:
		return seg->frames * 8;
	default:
		return seg->frames * EBC_LUT_FRAME_BYTES;
	}
}

/*
 * Segments laid out one after another from the payload start, the last one
 * @cut bytes short of its stream: the container ends there.
 */
static struct ebc_waveform *t_rkf_wf(struct kunit *test, struct device *dev,
				     const struct t_rkf_seg *segs,
				     unsigned int n, size_t cut)
{
	struct ebc_waveform *wf;
	size_t size = T_RKF_PAYLOAD;
	unsigned int i;
	u8 *buf;

	for (i = 0; i < n; i++)
		size += 4 + t_rkf_stream(&segs[i]);
	size -= cut;
	buf = t_rkf_alloc(test, size, size + 4);

	size = T_RKF_PAYLOAD;
	for (i = 0; i < n; i++) {
		put_unaligned_le32(size, buf + segs[i].table);
		buf[size] = segs[i].frames;
		size += 4 + t_rkf_stream(&segs[i]);
	}
	size -= cut;
	t_rkf_seal(buf, size);

	wf = rk3026_ebc_waveform_parse(dev, buf, size + 4);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, wf);
	return wf;
}

/* The frames of a mode's LUT, or -1 if there is none. */
static int t_rkf_frames(struct ebc_waveform *wf, u32 mode)
{
	u32 frames = 0;

	if (!rk3026_ebc_waveform_lut(wf, mode, 25, &frames))
		return -1;
	return frames;
}

static const u8 *t_rkf_lut(struct kunit *test, struct device *dev,
			   size_t size, u32 off, u32 *frames)
{
	struct ebc_waveform *wf;

	wf = rk3026_ebc_waveform_parse(dev, t_rkf_container(test, size, off),
				       size + 4);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, wf);
	*frames = 0;
	return rk3026_ebc_waveform_lut(wf, EBC_MODE_PART, 25, frames);
}

/* A segment whose stream ends exactly at the container end is whole. */
static void ebc_test_rkf_segment_fits(struct kunit *test)
{
	struct device *dev = t_rkf_dev(test);
	size_t size = T_RKF_PAYLOAD + T_RKF_SEG;
	u32 frames;

	KUNIT_EXPECT_NOT_NULL(test, t_rkf_lut(test, dev, size, T_RKF_PAYLOAD,
					      &frames));
	KUNIT_EXPECT_EQ(test, frames, 1);
}

static void ebc_test_rkf_segment_truncated(struct kunit *test)
{
	struct device *dev = t_rkf_dev(test);
	size_t size = T_RKF_PAYLOAD + T_RKF_SEG;
	u32 frames;

	/* One stream byte short; the CRC word follows and must not count. */
	KUNIT_EXPECT_NULL(test, t_rkf_lut(test, dev, size - 1, T_RKF_PAYLOAD,
					  &frames));
	/* A frame count with no stream at all. */
	KUNIT_EXPECT_NULL(test, t_rkf_lut(test, dev, size, size - 4, &frames));
	/* A segment header cut by the end, and offsets past it. */
	KUNIT_EXPECT_NULL(test, t_rkf_lut(test, dev, size, size - 2, &frames));
	KUNIT_EXPECT_NULL(test, t_rkf_lut(test, dev, size, size, &frames));
	KUNIT_EXPECT_NULL(test, t_rkf_lut(test, dev, size, U32_MAX, &frames));
	/* An empty slot is an absent mode. */
	KUNIT_EXPECT_NULL(test, t_rkf_lut(test, dev, size, 0, &frames));
}

/* Every LUT family decodes its own stream length. */
static void ebc_test_rkf_modes(struct kunit *test)
{
	struct device *dev = t_rkf_dev(test);
	static const struct t_rkf_seg segs[] = {
		{ T_RKF_RESET, 5 }, { T_RKF_GRAY16, 1 }, { T_RKF_MODE6, 2 },
		{ T_RKF_GRAY2, 3 }, { T_RKF_GRAYA2, 4 },
	};
	struct ebc_waveform *wf = t_rkf_wf(test, dev, segs, ARRAY_SIZE(segs), 0);

	KUNIT_EXPECT_EQ(test, t_rkf_frames(wf, EBC_MODE_RESET), 5);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(wf, EBC_MODE_PART), 1);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(wf, EBC_MODE_FULL), 1);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(wf, EBC_MODE_OED_PART), 2);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(wf, EBC_MODE_BLACK_WHITE), 3);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(wf, EBC_MODE_A2), 4);
	/* AUTO: GRAY16, the GRAY2 overlay skipped when longer than it */
	KUNIT_EXPECT_EQ(test, t_rkf_frames(wf, EBC_MODE_AUTO), 1);
}

/* Each family cut one byte short, as the last segment, has no LUT. */
static void ebc_test_rkf_truncated_modes(struct kunit *test)
{
	struct device *dev = t_rkf_dev(test);
	static const struct {
		struct t_rkf_seg	seg;
		u32			mode;
	} cases[] = {
		{ { T_RKF_RESET, 5 }, EBC_MODE_RESET },
		{ { T_RKF_GRAY16, 1 }, EBC_MODE_PART },
		{ { T_RKF_GRAY2, 3 }, EBC_MODE_BLACK_WHITE },
		/* GRAYA2 falls back to an absent GRAY2 */
		{ { T_RKF_GRAYA2, 2 }, EBC_MODE_A2 },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		KUNIT_EXPECT_EQ(test,
				t_rkf_frames(t_rkf_wf(test, dev, &cases[i].seg, 1, 0),
					     cases[i].mode),
				cases[i].seg.frames);
		KUNIT_EXPECT_EQ(test,
				t_rkf_frames(t_rkf_wf(test, dev, &cases[i].seg, 1, 1),
					     cases[i].mode), -1);
	}
}

/* MODE6 and GRAYA2 fall back on any error, absent or damaged. */
static void ebc_test_rkf_fallbacks(struct kunit *test)
{
	struct device *dev = t_rkf_dev(test);
	static const struct t_rkf_seg mode6_absent[] = {
		{ T_RKF_GRAY16, 3 },
	};
	static const struct t_rkf_seg mode6_cut[] = {
		{ T_RKF_GRAY16, 3 }, { T_RKF_MODE6, 2 },
	};
	static const struct t_rkf_seg a2_absent[] = {
		{ T_RKF_GRAY2, 3 },
	};
	static const struct t_rkf_seg a2_cut[] = {
		{ T_RKF_GRAY2, 3 }, { T_RKF_GRAYA2, 2 },
	};

	KUNIT_EXPECT_EQ(test, t_rkf_frames(t_rkf_wf(test, dev, mode6_absent, 1, 0),
					   EBC_MODE_OED_PART), 3);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(t_rkf_wf(test, dev, mode6_cut, 2, 0),
					   EBC_MODE_OED_PART), 2);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(t_rkf_wf(test, dev, mode6_cut, 2, 1),
					   EBC_MODE_OED_PART), 3);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(t_rkf_wf(test, dev, a2_absent, 1, 0),
					   EBC_MODE_A2), 3);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(t_rkf_wf(test, dev, a2_cut, 2, 0),
					   EBC_MODE_A2), 2);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(t_rkf_wf(test, dev, a2_cut, 2, 1),
					   EBC_MODE_A2), 3);
}

/*
 * AUTO overlays GRAY2 onto GRAY16: an absent overlay or a longer one is
 * skipped, a damaged one fails the LUT.
 */
static void ebc_test_rkf_auto_overlay(struct kunit *test)
{
	struct device *dev = t_rkf_dev(test);
	static const struct t_rkf_seg absent[] = {
		{ T_RKF_GRAY16, 3 },
	};
	static const struct t_rkf_seg longer[] = {
		{ T_RKF_GRAY16, 1 }, { T_RKF_GRAY2, 2 },
	};
	static const struct t_rkf_seg overlay[] = {
		{ T_RKF_GRAY16, 3 }, { T_RKF_GRAY2, 2 },
	};
	struct ebc_waveform *wf;

	KUNIT_EXPECT_EQ(test, t_rkf_frames(t_rkf_wf(test, dev, absent, 1, 0),
					   EBC_MODE_AUTO), 3);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(t_rkf_wf(test, dev, longer, 2, 0),
					   EBC_MODE_AUTO), 1);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(t_rkf_wf(test, dev, overlay, 2, 0),
					   EBC_MODE_AUTO), 3);
	/* The overlay one byte short: no LUT, though GRAY16 is whole. */
	wf = t_rkf_wf(test, dev, overlay, 2, 1);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(wf, EBC_MODE_AUTO), -1);
	KUNIT_EXPECT_EQ(test, t_rkf_frames(wf, EBC_MODE_PART), 3);
}

/* Containers whose size field does not match the buffer are refused. */
static void ebc_test_rkf_parse_bounds(struct kunit *test)
{
	struct device *dev = t_rkf_dev(test);
	size_t size = T_RKF_PAYLOAD + T_RKF_SEG;
	size_t big = 0x40001;
	u8 *buf;

	buf = t_rkf_container(test, size, T_RKF_PAYLOAD);

	KUNIT_EXPECT_NOT_ERR_OR_NULL(test,
				     rk3026_ebc_waveform_parse(dev, buf,
							       size + 4));
	/* Zero-length and truncated buffers, the CRC word cut. */
	KUNIT_EXPECT_EQ(test,
			PTR_ERR(rk3026_ebc_waveform_parse(dev, buf, 0)),
			-EINVAL);
	KUNIT_EXPECT_EQ(test,
			PTR_ERR(rk3026_ebc_waveform_parse(dev, buf, 16)),
			-EINVAL);
	KUNIT_EXPECT_EQ(test,
			PTR_ERR(rk3026_ebc_waveform_parse(dev, buf, size + 3)),
			-EINVAL);

	/*
	 * A size field one past the buffer, with a CRC that would close: only
	 * the length check refuses it, and nothing past @len is read.
	 */
	buf = t_rkf_alloc(test, size + 1, size + 5);
	t_rkf_seal(buf, size + 1);
	KUNIT_EXPECT_NOT_ERR_OR_NULL(test,
				     rk3026_ebc_waveform_parse(dev, buf,
							       size + 5));
	KUNIT_EXPECT_EQ(test,
			PTR_ERR(rk3026_ebc_waveform_parse(dev, buf, size + 4)),
			-EINVAL);

	/* Larger than any container, though whole and sealed. */
	buf = t_rkf_alloc(test, big, big + 4);
	t_rkf_seal(buf, big);
	KUNIT_EXPECT_EQ(test,
			PTR_ERR(rk3026_ebc_waveform_parse(dev, buf, big + 4)),
			-EINVAL);
}

static struct kunit_case rk3026_ebc_waveform_test_cases[] = {
	KUNIT_CASE(ebc_test_rkf_decode),
	KUNIT_CASE(ebc_test_rkf_decode_truncated),
	KUNIT_CASE(ebc_test_rkf_decode_zero_length),
	KUNIT_CASE(ebc_test_rkf_decode_oversized),
	KUNIT_CASE(ebc_test_rkf_segment_fits),
	KUNIT_CASE(ebc_test_rkf_segment_truncated),
	KUNIT_CASE(ebc_test_rkf_modes),
	KUNIT_CASE(ebc_test_rkf_truncated_modes),
	KUNIT_CASE(ebc_test_rkf_fallbacks),
	KUNIT_CASE(ebc_test_rkf_auto_overlay),
	KUNIT_CASE(ebc_test_rkf_parse_bounds),
	{}
};

static struct kunit_suite rk3026_ebc_waveform_test_suite = {
	.name = "rk3026_ebc_waveform",
	.test_cases = rk3026_ebc_waveform_test_cases,
};

kunit_test_suite(rk3026_ebc_waveform_test_suite);

/* --- the refresh thread's glue (rk3026_ebc.c) --- */

/*
 * A bare struct rk3026_ebc with what the glue needs and no hardware: with
 * the CRTC off, without a waveform, for a part whose snapshot moves
 * nothing, or with the rails counted as up and no LUT for the part, a part
 * finishes without touching a register or a regulator.
 */
static struct rk3026_ebc *t_ebc_alloc(struct kunit *test)
{
	struct rk3026_ebc *ebc = kunit_kzalloc(test, sizeof(*ebc), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, ebc);
	ebc->dev = kunit_device_register(test, "rk3026-ebc-glue");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ebc->dev);
	mutex_init(&ebc->image_lock);
	mutex_init(&ebc->rails_lock);
	init_waitqueue_head(&ebc->update_wq);
	init_waitqueue_head(&ebc->marker_wq);
	ebc->width = T_W;
	ebc->height = T_H;
	ebc->img_size = T_SIZE;
	ebc->next_shadow = t_img_alloc(test, 15);
	ebc->pre_shadow = t_img_alloc(test, 15);
	ebc->shadows.prev = t_img_alloc(test, 15);
	ebc->shadows.scan = t_img_alloc(test, 15);
	ebc->shadows.pitch = T_PITCH;
	ebc->shadows.height = T_H;
	ebc->prev_virt = t_img_alloc(test, 15);
	ebc->crtc_on = true;
	return ebc;
}

static u32 t_send(struct rk3026_ebc *ebc, const struct drm_rect *clip,
		  u32 mode, u32 marker, enum rk3026_ebc_source source)
{
	struct rk3026_ebc_update upd = t_upd(clip, mode, marker, source);
	u32 seq;

	mutex_lock(&ebc->image_lock);
	seq = rk3026_ebc_add_update(ebc, &upd);
	mutex_unlock(&ebc->image_lock);
	return seq;
}

static void ebc_test_glue_add_update(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	struct drm_rect r = DRM_RECT_INIT(2, 1, 4, 2);
	u32 s1, s2, s3, seq;

	s1 = t_send(ebc, &r, EBC_MODE_PART, 5, EBC_SOURCE_MXCFB);
	s2 = t_send(ebc, &r, EBC_MODE_A2, 6, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_EQ(test, s2, s1 + 1);
	KUNIT_EXPECT_EQ(test, ebc->stats.updates, 2);
	KUNIT_EXPECT_EQ(test, ebc->stats.coalesced, 1);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&ebc->pending), 1);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_marker_seq_lookup(&ebc->markers, 5,
							     &seq));
	KUNIT_EXPECT_EQ(test, seq, s1);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_work_pending(&ebc->pending));

	/*
	 * A RESET whitens the target and drops a carry; the part pending in
	 * front of it keeps its image.
	 */
	t_img_fill(ebc->next_shadow, 3);
	ebc->pending.carry = (struct rk3026_ebc_set) {
		.box = r, .mode = EBC_MODE_FULL, .merged = 1,
	};
	s3 = t_send(ebc, &t_panel, EBC_MODE_RESET, 0, EBC_SOURCE_ENABLE);
	KUNIT_EXPECT_EQ(test, s3, s2 + 1);
	t_expect_white(test, ebc->next_shadow);
	t_expect_guard(test, ebc->next_shadow);
	KUNIT_EXPECT_TRUE(test, ebc->pre_split);
	KUNIT_EXPECT_EQ(test, ebc->pre_shadow[0], 0x33);
	KUNIT_EXPECT_EQ(test, ebc->pre_shadow[T_SIZE - 1], 0x33);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&ebc->pending), 2);
	KUNIT_EXPECT_EQ(test, ebc->stats.coalesced, 1);
}

/* With the CRTC off a part is dropped; its waits end all the same. */
static void ebc_test_glue_crtc_off(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	struct rk3026_ebc_last last;
	u8 *prev = ebc->shadows.prev;
	u32 seq;

	ebc->crtc_on = false;
	seq = t_send(ebc, &t_panel, EBC_MODE_FULL, 1, EBC_SOURCE_MXCFB);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_CRTC_OFF);
	KUNIT_EXPECT_EQ(test, last.marker, 1);
	KUNIT_EXPECT_EQ(test, ebc->last.outcome, EBC_OUTCOME_CRTC_OFF);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_PTR_EQ(test, ebc->shadows.prev, prev);
	KUNIT_EXPECT_EQ(test, ebc->stats.done, 1);
	KUNIT_EXPECT_EQ(test, ebc->stats.dropped, 1);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_marker(ebc, 1), 0);

	/* Nothing pending: nothing taken, the record stays. */
	last.outcome = EBC_OUTCOME_NONE;
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NONE);
	KUNIT_EXPECT_EQ(test, ebc->stats.done, 1);
}

/* Without a waveform a part is carried, even with the glass unknown. */
static void ebc_test_glue_no_waveform(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	struct drm_rect a = DRM_RECT_INIT(0, 0, 4, 2);
	struct drm_rect b = DRM_RECT_INIT(8, 4, 2, 2);
	struct rk3026_ebc_last last;
	u32 seq;

	ebc->shadows.prev_unknown = true;
	seq = t_send(ebc, &a, EBC_MODE_FULL, 2, EBC_SOURCE_MXCFB);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_WAVEFORM);
	KUNIT_EXPECT_FALSE(test, last.repaired);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&ebc->pending.carry.box, &a));
	KUNIT_EXPECT_TRUE(test, ebc->shadows.prev_unknown);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_marker(ebc, 2), 0);

	t_send(ebc, &b, EBC_MODE_A2, 0, EBC_SOURCE_PLANE);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.clip.x1, 0);
	KUNIT_EXPECT_EQ(test, last.clip.x2, 10);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, ebc->stats.dropped, 2);
}

/* A snapshot that moves nothing is skipped before any hardware. */
static void ebc_test_glue_skipped(struct kunit *test)
{
	static const struct t_rkf_seg seg = { T_RKF_GRAY16, 1 };
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	struct rk3026_ebc_last last;
	u8 *prev = ebc->shadows.prev;
	u32 seq;

	/* Never read before the skip; a real one all the same. */
	ebc->waveform = t_rkf_wf(test, ebc->dev, &seg, 1, 0);
	seq = t_send(ebc, &t_panel, EBC_MODE_FULL, 0, EBC_SOURCE_PLANE);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_SKIPPED);
	KUNIT_EXPECT_EQ(test, last.scans, 0);
	KUNIT_EXPECT_FALSE(test, last.temp_read);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_PTR_EQ(test, ebc->shadows.prev, prev);
	KUNIT_EXPECT_EQ(test, ebc->stats.skips, 1);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
}

/*
 * A RESET sent behind a pending part leaves the part its own pixels: on a
 * white glass its black box is drawn, not skipped as the RESET's white.
 * Sends after the RESET and a second RESET leave that image alone. With the
 * rails counted as up and a waveform that has only a RESET, the part stops
 * at its missing LUT, before any register.
 */
static void ebc_test_glue_reset_keeps_pre(struct kunit *test)
{
	static const struct t_rkf_seg seg = { T_RKF_RESET, 5 };
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	struct drm_rect r = DRM_RECT_INIT(2, 1, 4, 2);
	struct drm_rect after = DRM_RECT_INIT(8, 3, 2, 1);
	struct rk3026_ebc_last last;
	u8 *pre_image = ebc->next_shadow;
	int x, y;

	ebc->waveform = t_rkf_wf(test, ebc->dev, &seg, 1, 0);
	ebc->next_virt = t_img_alloc(test, 15);
	ebc->epd_on = true;

	for (y = r.y1; y < r.y2; y++)
		for (x = r.x1; x < r.x2; x++)
			t_px_set(ebc->next_shadow, x, y, 0);
	t_send(ebc, &r, EBC_MODE_PART, 1, EBC_SOURCE_MXCFB);
	t_send(ebc, &t_panel, EBC_MODE_RESET, 2, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, ebc->pre_split);
	KUNIT_EXPECT_PTR_EQ(test, ebc->pre_shadow, pre_image);
	KUNIT_EXPECT_PTR_NE(test, ebc->next_shadow, pre_image);
	t_expect_white(test, ebc->next_shadow);

	/* Sent after the RESET: blitted on white, into the other image. */
	t_px_set(ebc->next_shadow, 8, 3, 0);
	t_send(ebc, &after, EBC_MODE_PART, 3, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_EQ(test, t_px(pre_image, 8, 3), 15);
	/* A second RESET absorbs it and whitens its image again. */
	t_send(ebc, &t_panel, EBC_MODE_RESET, 4, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, ebc->pre_split);
	KUNIT_EXPECT_PTR_EQ(test, ebc->pre_shadow, pre_image);
	t_expect_white(test, ebc->next_shadow);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&ebc->pending), 2);

	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_EQ(test, last.marker, 1);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&last.clip, &r));
	KUNIT_EXPECT_FALSE(test, ebc->pre_split);
	for (y = 0; y < T_H; y++)
		for (x = 0; x < T_W; x++)
			KUNIT_EXPECT_EQ(test, t_px(ebc->shadows.scan, x, y),
					t_in_rect(&r, x, y) ? 0 : 15);
	t_expect_guard(test, ebc->shadows.scan);
	t_expect_white(test, ebc->next_shadow);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&ebc->pending.carry.box, &r));
	KUNIT_EXPECT_EQ(test, ebc->stats.dropped, 1);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&ebc->pending), 1);
}

struct t_glue_work {
	struct delayed_work	work;
	struct rk3026_ebc	*ebc;
	struct rk3026_ebc_last	last;
	unsigned int		parts;
};

/* The refresh thread, later: take and finish @parts parts. */
static void t_glue_refresh(struct work_struct *work)
{
	struct t_glue_work *w = container_of(to_delayed_work(work),
					     struct t_glue_work, work);

	while (w->parts--)
		rk3026_ebc_do_refresh(w->ebc, &w->last);
}

static void ebc_test_glue_wait_marker(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	struct t_glue_work *w = kunit_kzalloc(test, sizeof(*w), GFP_KERNEL);
	u32 seq;

	KUNIT_ASSERT_NOT_NULL(test, w);
	ebc->crtc_on = false;

	/* Marker 0 and a marker never sent: nothing to wait for. */
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_marker(ebc, 0), 0);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_marker(ebc, 42), 0);

	/* The thread does not come: the wait times out. */
	seq = t_send(ebc, &t_panel, EBC_MODE_PART, 7, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_seq(ebc, seq, 20, true),
			-ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_seq(ebc, seq, 20, false),
			-ETIMEDOUT);

	/* It comes later: the finish wakes the wait. */
	w->ebc = ebc;
	w->parts = 1;
	INIT_DELAYED_WORK(&w->work, t_glue_refresh);
	schedule_delayed_work(&w->work, msecs_to_jiffies(50));
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_marker(ebc, 7), 0);
	flush_delayed_work(&w->work);
	KUNIT_EXPECT_EQ(test, w->last.outcome, EBC_OUTCOME_CRTC_OFF);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	/* Finished: a wait returns at once. */
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_seq(ebc, seq, 0, false), 0);
}

/* A wait on the RESET ends at the RESET, not at what was sent after it. */
static void ebc_test_glue_wait_barrier(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	struct t_glue_work *w = kunit_kzalloc(test, sizeof(*w), GFP_KERNEL);
	struct drm_rect r = DRM_RECT_INIT(2, 2, 4, 2);
	u32 s1, s2, s3;

	KUNIT_ASSERT_NOT_NULL(test, w);
	ebc->crtc_on = false;
	s1 = t_send(ebc, &r, EBC_MODE_PART, 1, EBC_SOURCE_MXCFB);
	s2 = t_send(ebc, &t_panel, EBC_MODE_RESET, 2, EBC_SOURCE_MXCFB);
	s3 = t_send(ebc, &r, EBC_MODE_PART, 3, EBC_SOURCE_MXCFB);

	w->ebc = ebc;
	w->parts = 2;
	INIT_DELAYED_WORK(&w->work, t_glue_refresh);
	schedule_delayed_work(&w->work, msecs_to_jiffies(20));
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_seq(ebc, s2, 10000, false), 0);
	flush_delayed_work(&w->work);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, s1));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending, s3));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_work_pending(&ebc->pending));
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_seq(ebc, s3, 20, false),
			-ETIMEDOUT);

	w->parts = 1;
	schedule_delayed_work(&w->work, 0);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_marker(ebc, 3), 0);
	flush_delayed_work(&w->work);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));
}

/* Four parts, each a setup and eight chunks at their budget. */
static void ebc_test_glue_wait_timeout(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_ebc_alloc(test);

	/* The fallback dot clock on a 16x6 panel: every budget at its floor. */
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_timeout_ms(ebc),
			4 * (1000 + 8 * 5000));

	/*
	 * 1 kHz, 4 x 6 clocks a frame: 24 ms a frame, a 64-frame budget of
	 * 64 x 24 x 4 = 6144 ms, above the floor.
	 */
	ebc->timing.pixelclock.typ = 1000;
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_timeout_ms(ebc),
			4 * (1000 + 8 * 6144));
}

/* A plane update leaves the panel to an mxcfb client. */
static void ebc_test_glue_plane_update(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	u32 *fb = kunit_kcalloc(test, T_W * T_H, sizeof(*fb), GFP_KERNEL);
	int i;

	KUNIT_ASSERT_NOT_NULL(test, fb);

	/* An mxcfb client owns the panel: no blit, no update. */
	ebc->mxcfb_active = true;
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4));
	t_expect_white(test, ebc->next_shadow);
	KUNIT_EXPECT_EQ(test, ebc->stats.updates, 0);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));

	/* None: the whole framebuffer (black), the whole panel, FULL. */
	ebc->mxcfb_active = false;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4));
	for (i = 0; i < T_SIZE; i++)
		KUNIT_EXPECT_EQ(test, ebc->next_shadow[i], 0);
	t_expect_guard(test, ebc->next_shadow);
	KUNIT_EXPECT_EQ(test, ebc->stats.updates, 1);
	KUNIT_EXPECT_EQ(test, ebc->pending.pre.merged, 1);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&ebc->pending.pre.box, &t_panel));
	KUNIT_EXPECT_EQ(test, ebc->pending.pre.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, ebc->pending.pre.source, EBC_SOURCE_PLANE);
}

static struct kunit_case rk3026_ebc_glue_test_cases[] = {
	KUNIT_CASE(ebc_test_glue_add_update),
	KUNIT_CASE(ebc_test_glue_crtc_off),
	KUNIT_CASE(ebc_test_glue_no_waveform),
	KUNIT_CASE(ebc_test_glue_skipped),
	KUNIT_CASE(ebc_test_glue_reset_keeps_pre),
	KUNIT_CASE(ebc_test_glue_wait_marker),
	KUNIT_CASE(ebc_test_glue_wait_barrier),
	KUNIT_CASE(ebc_test_glue_wait_timeout),
	KUNIT_CASE(ebc_test_glue_plane_update),
	{}
};

static struct kunit_suite rk3026_ebc_glue_test_suite = {
	.name = "rk3026_ebc_glue",
	.test_cases = rk3026_ebc_glue_test_cases,
};

kunit_test_suite(rk3026_ebc_glue_test_suite);

/* --- the conversion: fixtures and generators --- */

/*
 * The snapshot and the blit on wider images than the 16x6 panel: 256 px x
 * 16 rows (the 4096 set fills it once) and the line width of a 1448x1072
 * panel, 1448 px, over 4 rows. An image has T_GUARD bytes of 0xa5 in front
 * and behind.
 */
struct t_fix {
	const char *name;
	int w;
	int h;
};

static const struct t_fix t_fixes[] = {
	{ "16x6", T_W, T_H },
	{ "256x16", 256, 16 },
	{ "1448x4", 1448, 4 },
};

static u32 t_fix_pitch(const struct t_fix *f)
{
	return f->w / 2;
}

static size_t t_fix_size(const struct t_fix *f)
{
	return (size_t)t_fix_pitch(f) * f->h;
}

/*
 * @r widened to whole bytes, an even x on both edges, and clamped to @f, as
 * the driver widens a clip, into @w; false if nothing of it is left. The
 * tests' own copy, so the references do not share the driver's code.
 */
static bool t_widen(const struct t_fix *f, const struct drm_rect *r,
		    struct drm_rect *w)
{
	w->x1 = max(r->x1, 0) & ~1;
	w->x2 = min_t(int, round_up(r->x2, 2), f->w);
	w->y1 = max(r->y1, 0);
	w->y2 = min_t(int, r->y2, f->h);
	return w->x1 < w->x2 && w->y1 < w->y2;
}

static u8 *t_fix_img(struct kunit *test, const struct t_fix *f)
{
	u8 *buf = kunit_kmalloc(test, t_fix_size(f) + 2 * T_GUARD, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, buf);
	memset(buf, 0xa5, t_fix_size(f) + 2 * T_GUARD);
	return buf + T_GUARD;
}

/*
 * The first byte, guards included, where @a and @b differ, as an offset
 * from the image start; INT_MAX if they are the same.
 */
static int t_fix_diff(const struct t_fix *f, const u8 *a, const u8 *b)
{
	int i, end = t_fix_size(f) + T_GUARD;

	for (i = -T_GUARD; i < end; i++)
		if (a[i] != b[i])
			return i;
	return INT_MAX;
}

static bool t_fix_guard_ok(const struct t_fix *f, const u8 *img)
{
	int i;

	for (i = 1; i <= T_GUARD; i++)
		if (img[-i] != 0xa5 || img[t_fix_size(f) + i - 1] != 0xa5)
			return false;
	return true;
}

/*
 * Clips and boxes for @f: whole, odd edges, one pixel, word-misaligned
 * starts, tails of 2, 4 and 6 pixels, out of range, empty, inverted, and
 * T_CLIPS_RANDOM random ones (empty ones among them). Returns the count.
 */
#define T_CLIPS_RANDOM	8
#define T_CLIPS_MAX	(16 + T_CLIPS_RANDOM)

static int t_fix_clips(const struct t_fix *f, struct rnd_state *rs,
		       struct drm_rect *c)
{
	int w = f->w, h = f->h, n = 0, i;

	c[n++] = DRM_RECT_INIT(0, 0, w, h);
	c[n++] = DRM_RECT_INIT(1, 0, w - 2, h);
	c[n++] = DRM_RECT_INIT(3, 1, w - 8, h - 1);
	c[n++] = DRM_RECT_INIT(5, 0, 1, 1);
	c[n++] = DRM_RECT_INIT(w - 1, h - 1, 1, 1);
	c[n++] = DRM_RECT_INIT(0, 0, 1, h);
	c[n++] = DRM_RECT_INIT(2, 0, w - 2, h);
	c[n++] = DRM_RECT_INIT(4, 0, w - 6, h);
	c[n++] = DRM_RECT_INIT(6, 1, w - 10, h - 1);
	c[n++] = DRM_RECT_INIT(9, 0, 17, h);
	c[n++] = DRM_RECT_INIT(0, 0, 10, h);
	c[n++] = DRM_RECT_INIT(-5, -3, w + 12, h + 5);
	c[n++] = DRM_RECT_INIT(4, 2, 0, h);
	c[n++] = (struct drm_rect){ .x1 = 10, .y1 = 3, .x2 = 5, .y2 = 4 };
	c[n++] = DRM_RECT_INIT(0, h, w, 3);
	c[n++] = DRM_RECT_INIT(w, 0, 4, h);
	for (i = 0; i < T_CLIPS_RANDOM; i++) {
		int x1 = prandom_u32_state(rs) % (w + 8) - 4;
		int x2 = prandom_u32_state(rs) % (w + 8) - 4;
		int y1 = prandom_u32_state(rs) % (h + 4) - 2;
		int y2 = prandom_u32_state(rs) % (h + 4) - 2;

		c[n++] = (struct drm_rect){
			.x1 = min(x1, x2), .y1 = min(y1, y2),
			.x2 = max(x1, x2), .y2 = max(y1, y2),
		};
	}
	return n;
}

/* XRGB8888 sources; the X byte is random in all of them. */
enum t_src_kind {
	T_SRC_4096,	/* the 16^3 colours with the low 4 bits of R, G, B zero */
	T_SRC_GREY256,	/* R = G = B = v, v = 0..255 */
	T_SRC_RANDOM,	/* random colours */
	T_SRC_BLOCKS,	/* runs of 8-px blocks of mixed content */
	T_SRC_COUNT,
};

/*
 * The blocks of T_SRC_BLOCKS, page content in small: white and black, greys
 * that pack to 0 or 15 and ones that do not, colour, white or black blocks
 * with one pixel off (black, or one bit short of white or black), and
 * pixels whose R, G and B share only the high nibble.
 */
enum t_block {
	T_BLK_WHITE,
	T_BLK_BLACK,
	T_BLK_GREY_LO,		/* 0x08: packs to 0 */
	T_BLK_GREY_HI,		/* 0xf7: packs to 15 */
	T_BLK_GREY_MID,		/* 0x80 */
	T_BLK_GREYS,		/* a random grey per pixel */
	T_BLK_NIBBLE,		/* one high nibble, random low nibbles */
	T_BLK_COLOUR,
	T_BLK_DOT,		/* white with one black pixel */
	T_BLK_NEAR_WHITE,	/* white with one 0xfffeff pixel */
	T_BLK_NEAR_BLACK,	/* black with one 0x000100 pixel */
	T_BLK_COUNT,
};

/*
 * Row 0 of a T_SRC_BLOCKS image: a fixed sequence that puts every kind next
 * to several others; the other rows take runs of random kinds.
 */
static const u8 t_block_seq[] = {
	T_BLK_WHITE, T_BLK_WHITE, T_BLK_GREY_MID, T_BLK_GREY_MID,
	T_BLK_GREY_LO, T_BLK_WHITE, T_BLK_BLACK, T_BLK_COLOUR, T_BLK_COLOUR,
	T_BLK_GREY_HI, T_BLK_WHITE, T_BLK_DOT, T_BLK_NEAR_WHITE, T_BLK_BLACK,
	T_BLK_NEAR_BLACK, T_BLK_NIBBLE, T_BLK_NIBBLE, T_BLK_GREYS,
	T_BLK_GREY_LO, T_BLK_BLACK, T_BLK_GREY_HI, T_BLK_WHITE, T_BLK_COLOUR,
	T_BLK_BLACK,
};

/* Pixel @i of a block of @kind; @pos is the odd pixel of DOT and NEAR_*. */
static u32 t_block_px(enum t_block kind, int i, int pos, struct rnd_state *rs)
{
	u32 r = prandom_u32_state(rs), hi = r & 0xf0;

	switch (kind) {
	case T_BLK_WHITE:
		return 0xffffff;
	case T_BLK_BLACK:
		return 0;
	case T_BLK_GREY_LO:
		return 0x080808;
	case T_BLK_GREY_HI:
		return 0xf7f7f7;
	case T_BLK_GREY_MID:
		return 0x808080;
	case T_BLK_GREYS:
		return (r & 0xff) * 0x010101;
	case T_BLK_NIBBLE:
		return (hi | (r >> 8 & 0xf)) << 16 | (hi | (r >> 12 & 0xf)) << 8 |
		       hi | (r >> 16 & 0xf);
	case T_BLK_DOT:
		return i == pos ? 0 : 0xffffff;
	case T_BLK_NEAR_WHITE:
		return i == pos ? 0xfffeff : 0xffffff;
	case T_BLK_NEAR_BLACK:
		return i == pos ? 0x000100 : 0;
	default:
		return r & 0xffffff;
	}
}

static u32 t_col_4096(u32 i)
{
	return ((i >> 8) & 15) << 20 | ((i >> 4) & 15) << 12 | (i & 15) << 4;
}

/*
 * Fill the @f->w x @f->h pixels of @src, @src_pitch bytes per row (at least
 * 4 * @f->w; the callers pass it dense and with 4 bytes of padding), with
 * @kind from @seed.
 */
static void t_src_fill(u8 *src, unsigned int src_pitch, const struct t_fix *f,
		       enum t_src_kind kind, u32 seed)
{
	struct rnd_state rs;
	int x, y, k = 0;

	prandom_seed_state(&rs, seed);
	for (y = 0; y < f->h; y++) {
		enum t_block blk = T_BLK_WHITE;
		int run = 0, pos = 0;

		for (x = 0; x < f->w; x++, k++) {
			u32 px;

			if (kind == T_SRC_BLOCKS && !(x & 7)) {
				if (!y) {
					blk = t_block_seq[(x / 8) %
							  ARRAY_SIZE(t_block_seq)];
				} else if (!run) {
					blk = prandom_u32_state(&rs) % T_BLK_COUNT;
					run = 1 + prandom_u32_state(&rs) % 3;
				}
				run = max(run - 1, 0);
				pos = 1 + prandom_u32_state(&rs) % 7;
			}
			switch (kind) {
			case T_SRC_4096:
				px = t_col_4096((k + seed * 977) % 4096);
				break;
			case T_SRC_GREY256:
				px = ((k + seed) & 0xff) * 0x010101;
				break;
			case T_SRC_RANDOM:
				px = prandom_u32_state(&rs) & 0xffffff;
				break;
			default:
				px = t_block_px(blk, x & 7, pos, &rs);
				break;
			}
			px |= prandom_u32_state(&rs) & 0xff000000;
			put_unaligned_le32(px, src + (size_t)y * src_pitch + x * 4);
		}
	}
}

/* prev and next images for the snapshot */
enum t_img_kind {
	T_IMG_RANDOM,		/* random greys */
	T_IMG_TWO_LEVEL,	/* random 0 and 15 */
	T_IMG_SOME,		/* @base with about a quarter of it random */
	T_IMG_SOME_TWO_LEVEL,	/* @base with about a quarter of it 0 or 15 */
	T_IMG_SAME,		/* @base */
};

static void t_img_gen(u8 *img, const struct t_fix *f, const u8 *base,
		      enum t_img_kind kind, struct rnd_state *rs)
{
	size_t i, size = t_fix_size(f);

	for (i = 0; i < size; i++) {
		u32 r = prandom_u32_state(rs);
		u8 lo = r & 0xf, hi = r >> 4 & 0xf;

		switch (kind) {
		case T_IMG_RANDOM:
			img[i] = r;
			break;
		case T_IMG_TWO_LEVEL:
			img[i] = (r & 1 ? 0x0f : 0) | (r & 2 ? 0xf0 : 0);
			break;
		case T_IMG_SAME:
			img[i] = base[i];
			break;
		default:
			if (kind == T_IMG_SOME_TWO_LEVEL) {
				lo = lo & 1 ? 0xf : 0;
				hi = hi & 1 ? 0xf : 0;
			}
			img[i] = base[i];
			if (!(r >> 8 & 3))
				img[i] = (img[i] & 0xf0) | lo;
			if (!(r >> 10 & 3))
				img[i] = (img[i] & 0x0f) | hi << 4;
			break;
		}
	}
}

static const u32 t_snap_modes[] = {
	EBC_MODE_PART, EBC_MODE_FULL, EBC_MODE_BLACK_WHITE, EBC_MODE_A2,
};

/* prev and next kinds of the snapshot cases */
static const u8 t_snap_imgs[][2] = {
	{ T_IMG_RANDOM, T_IMG_RANDOM },
	{ T_IMG_RANDOM, T_IMG_SOME },
	{ T_IMG_RANDOM, T_IMG_SOME_TWO_LEVEL },
	{ T_IMG_RANDOM, T_IMG_SAME },
	{ T_IMG_TWO_LEVEL, T_IMG_RANDOM },
	{ T_IMG_TWO_LEVEL, T_IMG_SOME },
	{ T_IMG_TWO_LEVEL, T_IMG_SOME_TWO_LEVEL },
	{ T_IMG_TWO_LEVEL, T_IMG_SAME },
};

/* --- the conversion: the per-pixel references against the driver --- */

struct t_snap_count {
	int raised;
	int moved;
	int still;
};

/*
 * Every mode and box on one prev/next pair: the reference and the driver
 * write the same scan image over the same junk, guards included, return
 * the same and give the same mode. False after the first difference.
 */
static bool t_ref_snapshot_pair(struct kunit *test, const struct t_fix *f,
				u8 *prev, const u8 *next, u8 *scan_ref,
				u8 *scan, const struct drm_rect *clips, int n,
				struct rnd_state *rs, struct t_snap_count *cnt)
{
	struct rk3026_ebc_shadows sh = {
		.prev = prev, .pitch = t_fix_pitch(f), .height = f->h,
	};
	unsigned int mi;
	int c;

	for (mi = 0; mi < ARRAY_SIZE(t_snap_modes); mi++) {
		for (c = 0; c < n; c++) {
			u32 mode_ref = t_snap_modes[mi], mode = mode_ref;
			bool ret_ref, ret;
			int diff;

			prandom_bytes_state(rs, scan_ref, t_fix_size(f));
			memcpy(scan, scan_ref, t_fix_size(f));
			sh.scan = scan_ref;
			ret_ref = rk3026_ebc_ref_snapshot(&sh, next, &clips[c],
							  &mode_ref);
			sh.scan = scan;
			ret = rk3026_ebc_build_snapshot(&sh, next, &clips[c],
							&mode);
			diff = t_fix_diff(f, scan_ref, scan);
			if (ret != ret_ref || mode != mode_ref ||
			    diff != INT_MAX) {
				KUNIT_FAIL(test,
					   "%s mode %u box " DRM_RECT_FMT
					   ": ret %d/%d mode %u/%u byte %d",
					   f->name, t_snap_modes[mi],
					   DRM_RECT_ARG(&clips[c]), ret, ret_ref,
					   mode, mode_ref, diff);
				return false;
			}
			cnt->raised += mode_ref != t_snap_modes[mi];
			cnt->moved += ret_ref;
			cnt->still += !ret_ref;
		}
	}
	return true;
}

/*
 * rk3026_ebc_build_snapshot() equals rk3026_ebc_ref_snapshot() on every
 * fixture, image pair, mode and box. The cases reach a move, no move and
 * the A2-over-grey raise.
 */
static void ebc_test_ref_snapshot(struct kunit *test)
{
	struct t_snap_count cnt = { 0 };
	unsigned int fi, ii;
	u32 seed;

	for (fi = 0; fi < ARRAY_SIZE(t_fixes); fi++) {
		const struct t_fix *f = &t_fixes[fi];
		u8 *prev = t_fix_img(test, f), *next = t_fix_img(test, f);
		u8 *scan_ref = t_fix_img(test, f), *scan = t_fix_img(test, f);
		struct drm_rect clips[T_CLIPS_MAX];
		struct rnd_state rs;
		int n;

		for (ii = 0; ii < ARRAY_SIZE(t_snap_imgs); ii++) {
			for (seed = 1; seed <= 2; seed++) {
				prandom_seed_state(&rs, seed * 131 + ii);
				t_img_gen(prev, f, NULL, t_snap_imgs[ii][0], &rs);
				t_img_gen(next, f, prev, t_snap_imgs[ii][1], &rs);
				n = t_fix_clips(f, &rs, clips);
				if (!t_ref_snapshot_pair(test, f, prev, next,
							 scan_ref, scan, clips,
							 n, &rs, &cnt)) {
					kunit_info(test, "images %u seed %u\n",
						   ii, seed);
					return;
				}
			}
		}
		KUNIT_EXPECT_TRUE(test, t_fix_guard_ok(f, scan));
	}
	KUNIT_EXPECT_GT(test, cnt.raised, 0);
	KUNIT_EXPECT_GT(test, cnt.moved, 0);
	KUNIT_EXPECT_GT(test, cnt.still, 0);
}

/* Pixel (@x, @y) of an image of fixture @f. */
static u8 t_px_w(const u8 *img, const struct t_fix *f, int x, int y)
{
	u8 b = img[(size_t)y * t_fix_pitch(f) + x / 2];

	return x & 1 ? b >> 4 : b & 0xf;
}

static void t_px_set_w(u8 *img, const struct t_fix *f, int x, int y, u8 grey)
{
	u8 *b = &img[(size_t)y * t_fix_pitch(f) + x / 2];

	if (x & 1)
		*b = (*b & 0x0f) | grey << 4;
	else
		*b = (*b & 0xf0) | grey;
}

/*
 * One snapshot of @box in @mode by the reference and the driver over the
 * same junk; *@mode_out is the driver's mode. False, with a failure, if
 * they differ.
 */
static bool t_snap_vs_ref(struct kunit *test, const struct t_fix *f,
			  u8 *prev, const u8 *next, u8 *scan_ref,
			  u8 *scan, const struct drm_rect *box, u32 mode,
			  u32 *mode_out, struct rnd_state *rs)
{
	struct rk3026_ebc_shadows sh = {
		.prev = prev, .pitch = t_fix_pitch(f), .height = f->h,
	};
	u32 mode_ref = mode;
	bool ret_ref, ret;
	int diff;

	prandom_bytes_state(rs, scan_ref, t_fix_size(f));
	memcpy(scan, scan_ref, t_fix_size(f));
	sh.scan = scan_ref;
	ret_ref = rk3026_ebc_ref_snapshot(&sh, next, box, &mode_ref);
	sh.scan = scan;
	*mode_out = mode;
	ret = rk3026_ebc_build_snapshot(&sh, next, box, mode_out);
	diff = t_fix_diff(f, scan_ref, scan);
	if (ret == ret_ref && *mode_out == mode_ref && diff == INT_MAX)
		return true;
	KUNIT_FAIL(test, "%s mode %u box " DRM_RECT_FMT
		   ": ret %d/%d mode %u/%u byte %d", f->name, mode,
		   DRM_RECT_ARG(box), ret, ret_ref, *mode_out, mode_ref, diff);
	return false;
}

/*
 * Every prev/next pair of levels (256) at every pixel of a word (8): row y
 * of the 256 x 16 fixture holds pair (x + y) % 256 at pixel x, prev's level
 * in the high four bits of the pair. Every mode, the whole image and boxes
 * with odd edges, against the reference.
 */
static void ebc_test_snapshot_nibble_pairs(struct kunit *test)
{
	const struct t_fix *f = &t_fixes[1];
	u8 *prev = t_fix_img(test, f), *next = t_fix_img(test, f);
	u8 *scan_ref = t_fix_img(test, f), *scan = t_fix_img(test, f);
	const struct drm_rect boxes[] = {
		DRM_RECT_INIT(0, 0, f->w, f->h),
		DRM_RECT_INIT(1, 0, f->w - 2, 8),
		DRM_RECT_INIT(3, 2, 77, 9),
		DRM_RECT_INIT(7, 8, 1, 8),
	};
	struct rnd_state rs;
	unsigned int mi, bi;
	int x, y;
	u32 mode;

	for (y = 0; y < f->h; y++) {
		for (x = 0; x < f->w; x++) {
			u8 q = x + y;

			t_px_set_w(prev, f, x, y, q >> 4);
			t_px_set_w(next, f, x, y, q & 0xf);
		}
	}
	prandom_seed_state(&rs, 29);
	for (mi = 0; mi < ARRAY_SIZE(t_snap_modes); mi++)
		for (bi = 0; bi < ARRAY_SIZE(boxes); bi++)
			if (!t_snap_vs_ref(test, f, prev, next, scan_ref, scan,
					   &boxes[bi], t_snap_modes[mi], &mode,
					   &rs))
				return;
	KUNIT_EXPECT_TRUE(test, t_fix_guard_ok(f, scan));
}

/*
 * The A2-over-grey raise at every pixel of three words: on a black and
 * white prev, one grey at pixel @j of a row. An A2 scan that changes pixel
 * @j runs DU; one that changes only the pixel next to it, or changes @j on
 * a prev without the grey, stays A2.
 */
static void ebc_test_snapshot_a2_guard_positions(struct kunit *test)
{
	const struct t_fix *f = &t_fixes[1];
	u8 *prev = t_fix_img(test, f), *next = t_fix_img(test, f);
	u8 *scan_ref = t_fix_img(test, f), *scan = t_fix_img(test, f);
	const struct drm_rect box = DRM_RECT_INIT(0, 0, f->w, f->h);
	struct rnd_state rs;
	int j, g, nb;
	u32 mode;

	prandom_seed_state(&rs, 31);
	for (j = 0; j < 24; j++) {
		for (g = 0; g < 16; g++) {
			for (nb = -1; nb <= 1; nb++) {
				int x = j + nb, y = 3;
				bool grey = g != 0 && g != 15;
				u8 lvl;

				if (x < 0)
					continue;
				t_img_gen(prev, f, NULL, T_IMG_TWO_LEVEL, &rs);
				t_px_set_w(prev, f, j, y, g);
				memcpy(next, prev, t_fix_size(f));
				lvl = t_px_w(prev, f, x, y);
				t_px_set_w(next, f, x, y, lvl >= 8 ? 0 : 15);
				if (!t_snap_vs_ref(test, f, prev, next,
						   scan_ref, scan, &box,
						   EBC_MODE_A2, &mode, &rs))
					return;
				KUNIT_EXPECT_EQ_MSG(test, mode,
						    !nb && grey ?
						    EBC_MODE_BLACK_WHITE :
						    EBC_MODE_A2,
						    "pixel %d grey %d at %d",
						    j, g, x);
			}
		}
	}
}

/* A bare struct rk3026_ebc for rk3026_ebc_blit_xrgb8888() on @f. */
static struct rk3026_ebc *t_blit_ebc(struct kunit *test, const struct t_fix *f)
{
	struct rk3026_ebc *ebc = kunit_kzalloc(test, sizeof(*ebc), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, ebc);
	mutex_init(&ebc->image_lock);
	ebc->width = f->w;
	ebc->height = f->h;
	ebc->next_shadow = t_fix_img(test, f);
	return ebc;
}

/*
 * Every clip on one source: the reference rows on the clip widened as the
 * driver widens it and rk3026_ebc_blit_xrgb8888() write the same target
 * over the same junk, guards included. False after the first difference.
 */
static bool t_ref_blit_src(struct kunit *test, const struct t_fix *f,
			   struct rk3026_ebc *ebc, u8 *ref, const u8 *src,
			   unsigned int pitch, const struct drm_rect *clips,
			   int n, struct rnd_state *rs)
{
	int c;

	for (c = 0; c < n; c++) {
		const struct drm_rect *r = &clips[c];
		struct drm_rect w;
		int diff;

		prandom_bytes_state(rs, ref, t_fix_size(f));
		memcpy(ebc->next_shadow, ref, t_fix_size(f));
		if (t_widen(f, r, &w))
			rk3026_ebc_ref_blit_rows(ref, src, pitch, w.x1, w.x2,
						 w.y1, w.y2, f->w);
		mutex_lock(&ebc->image_lock);
		rk3026_ebc_blit_xrgb8888(ebc, src, pitch, r);
		mutex_unlock(&ebc->image_lock);
		diff = t_fix_diff(f, ref, ebc->next_shadow);
		if (diff != INT_MAX) {
			KUNIT_FAIL(test, "%s pitch %u clip " DRM_RECT_FMT
				   ": byte %d", f->name, pitch,
				   DRM_RECT_ARG(r), diff);
			return false;
		}
	}
	return true;
}

/*
 * rk3026_ebc_blit_xrgb8888() equals rk3026_ebc_ref_blit_rows() on every
 * fixture, source set, dense and padded source pitch, and clip.
 */
static void ebc_test_ref_blit(struct kunit *test)
{
	unsigned int fi, pad, kind;
	u32 seed;

	for (fi = 0; fi < ARRAY_SIZE(t_fixes); fi++) {
		const struct t_fix *f = &t_fixes[fi];
		struct rk3026_ebc *ebc = t_blit_ebc(test, f);
		u8 *ref = t_fix_img(test, f);
		struct drm_rect clips[T_CLIPS_MAX];
		struct rnd_state rs;
		u8 *src;
		int n;

		src = kunit_kmalloc(test, (size_t)(f->w * 4 + 4) * f->h,
				    GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, src);

		for (pad = 0; pad <= 4; pad += 4) {
			unsigned int pitch = f->w * 4 + pad;

			for (kind = 0; kind < T_SRC_COUNT; kind++) {
				for (seed = 1; seed <= 2; seed++) {
					t_src_fill(src, pitch, f, kind, seed);
					prandom_seed_state(&rs, seed * 7 + kind);
					n = t_fix_clips(f, &rs, clips);
					if (!t_ref_blit_src(test, f, ebc, ref,
							    src, pitch, clips,
							    n, &rs)) {
						kunit_info(test,
							   "source %u seed %u\n",
							   kind, seed);
						return;
					}
				}
			}
		}
		KUNIT_EXPECT_TRUE(test, t_fix_guard_ok(f, ebc->next_shadow));
	}
}

static struct kunit_case rk3026_ebc_conv_test_cases[] = {
	KUNIT_CASE(ebc_test_ref_snapshot),
	KUNIT_CASE(ebc_test_snapshot_nibble_pairs),
	KUNIT_CASE(ebc_test_snapshot_a2_guard_positions),
	KUNIT_CASE(ebc_test_ref_blit),
	{}
};

static struct kunit_suite rk3026_ebc_conv_test_suite = {
	.name = "rk3026_ebc_conv",
	.test_cases = rk3026_ebc_conv_test_cases,
};

kunit_test_suite(rk3026_ebc_conv_test_suite);

MODULE_DESCRIPTION("KUnit tests for the RK3026 EBC driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("EXPORTED_FOR_KUNIT_TESTING");
