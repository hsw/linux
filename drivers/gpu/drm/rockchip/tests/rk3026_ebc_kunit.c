// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the RK3026 EBC refresh bookkeeping (rk3026_ebc_core.c)
 * and the mxcfb waveform map (rk3026_ebc_mxcfb.c).
 */

#include <kunit/test.h>

#include <linux/limits.h>
#include <linux/module.h>
#include <linux/string.h>

#include "../rk3026_ebc.h"
#include "../rk3026_ebc_mxcfb.h"

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

MODULE_DESCRIPTION("KUnit tests for the RK3026 EBC driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("EXPORTED_FOR_KUNIT_TESTING");
