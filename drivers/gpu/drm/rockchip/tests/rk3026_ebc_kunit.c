// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the RK3026 EBC refresh bookkeeping (rk3026_ebc_core.c),
 * the waveform container bounds (rk3026_ebc_waveform.c), the mxcfb waveform
 * map (rk3026_ebc_mxcfb.c), the refresh thread's glue and the fbdev damage
 * commit (rk3026_ebc.c), and the snapshot and the blit against the
 * per-pixel references of rk3026_ebc_kunit_ref.c, and the NEON blit against
 * the scalar one.
 */

#include <kunit/device.h>
#include <kunit/static_stub.h>
#include <kunit/test.h>

#include <linux/console.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
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

#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_plane.h>

#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_NEON)
#include <asm/neon.h>
#include <asm/simd.h>
#endif

#include "../rk3026_ebc.h"
#include "../rk3026_ebc_blit_neon.h"
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

/* An auto update, as a plane update or an mxcfb AUTO send makes it. */
static u32 t_add_auto(struct rk3026_ebc_pending *p, const struct drm_rect *clip,
		      u32 mode, enum rk3026_ebc_source source)
{
	struct rk3026_ebc_update upd = t_upd(clip, mode, 0, source);
	u32 seq;

	upd.auto_mode = true;
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

/*
 * A record's time in us: rounded up, at least 1 once measured (0 means
 * none), 1 for a clock seen going backwards, saturated at U32_MAX.
 */
static void ebc_test_us_between(struct kunit *test)
{
	ktime_t t0 = ns_to_ktime(5000);

	KUNIT_EXPECT_EQ(test, rk3026_ebc_us_between(t0, t0), 1);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_us_between(t0, t0 + 1), 1);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_us_between(t0, t0 + 1000), 1);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_us_between(t0, t0 + 1001), 2);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_us_between(t0, t0 + 42 * NSEC_PER_MSEC),
			42000);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_us_between(t0, t0 - 1), 1);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_us_between(t0, KTIME_MAX), U32_MAX);
	KUNIT_EXPECT_EQ(test,
			rk3026_ebc_us_between(0, (ktime_t)U32_MAX * NSEC_PER_USEC),
			U32_MAX);
	KUNIT_EXPECT_EQ(test,
			rk3026_ebc_us_between(0, ((ktime_t)U32_MAX - 1) * NSEC_PER_USEC),
			U32_MAX - 1);
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
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_pending(&p));
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
		.source = EBC_SOURCE_MXCFB, .queued = 1000,
	};
	struct rk3026_ebc_update b = {
		.clip = DRM_RECT_INIT(9, 3, 3, 3), .mode = EBC_MODE_PART,
		.source = EBC_SOURCE_PLANE, .queued = 5000,
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
	/* An mxcfb send is in it; queued from the first to the newest. */
	KUNIT_EXPECT_TRUE(test, part.has_mxcfb);
	KUNIT_EXPECT_EQ(test, part.first, 1000);
	KUNIT_EXPECT_EQ(test, part.last, 5000);
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

/*
 * A peek gives the part the take then gives and leaves the pending state
 * as it was: @pre with the carry merged in (no longer auto), then the RESET
 * alone, then what was sent after it. Nothing pending gives nothing.
 */
static void ebc_test_pending_peek(struct kunit *test)
{
	struct drm_rect a = DRM_RECT_INIT(0, 0, 4, 2);
	struct drm_rect b = DRM_RECT_INIT(8, 2, 4, 2);
	struct rk3026_ebc_pending p = {};
	struct rk3026_ebc_set peek, part;
	u32 s2, s3, s4;

	KUNIT_EXPECT_FALSE(test, rk3026_ebc_pending_peek(&p, &peek));
	KUNIT_EXPECT_EQ(test, peek.merged, 0);

	t_add_auto(&p, &a, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	s2 = t_add_auto(&p, &a, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	s3 = t_add(&p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_ENABLE);
	s4 = t_add(&p, &b, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	p.carry = (struct rk3026_ebc_set) {
		.box = b, .mode = EBC_MODE_PART, .merged = 1,
	};

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_peek(&p, &peek));
	KUNIT_EXPECT_EQ(test, peek.seq, s2);
	KUNIT_EXPECT_EQ(test, peek.merged, 2);
	KUNIT_EXPECT_FALSE(test, peek.auto_ok);
	KUNIT_EXPECT_EQ(test, peek.box.x2, 12);
	KUNIT_EXPECT_EQ(test, peek.box.y2, 4);
	/* Left as it was. */
	KUNIT_EXPECT_EQ(test, p.pre.merged, 2);
	KUNIT_EXPECT_TRUE(test, p.pre.auto_ok);
	KUNIT_EXPECT_EQ(test, p.pre.box.x2, 4);
	KUNIT_EXPECT_EQ(test, p.carry.merged, 1);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&p), 3);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&p, &part));
	KUNIT_EXPECT_EQ(test, part.seq, peek.seq);
	KUNIT_EXPECT_EQ(test, part.merged, peek.merged);
	KUNIT_EXPECT_EQ(test, part.auto_ok, peek.auto_ok);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&part.box, &peek.box));
	KUNIT_EXPECT_FALSE(test, p.carry.merged);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_peek(&p, &peek));
	KUNIT_EXPECT_EQ(test, peek.mode, EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, peek.seq, s3);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&p), 2);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&p, &part));
	KUNIT_EXPECT_EQ(test, part.seq, s3);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_peek(&p, &peek));
	KUNIT_EXPECT_EQ(test, peek.seq, s4);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&peek.box, &b));
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&p, &part));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_pending_peek(&p, &peek));
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
	bool two_level, over_grey;
	u32 changed;

	return rk3026_ebc_build_snapshot(&sh, next, box, &mode, &two_level,
					 &over_grey, &changed);
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
 * over a grey of prev runs DU, with the same thresholded snapshot. The
 * over-grey result says so in every mode, and only A2 is raised for it.
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
	bool two_level, over_grey;
	u32 mode, changed;

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
							  &mode, &two_level,
							  &over_grey,
							  &changed));
	KUNIT_EXPECT_EQ(test, mode, EBC_MODE_A2);
	KUNIT_EXPECT_FALSE(test, over_grey);
	KUNIT_EXPECT_EQ(test, t_px(dst, 3, 1), 0);
	KUNIT_EXPECT_EQ(test, t_px(dst, 4, 2), 15);
	KUNIT_EXPECT_EQ(test, t_px(dst, 5, 1), 7);

	/* One laid-over pixel on a grey, a grey target too: DU. */
	t_px_set(prev, 2, 2, 6);
	t_px_set(next, 2, 2, 12);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_build_snapshot(&sh, next, &box,
							  &mode, &two_level,
							  &over_grey,
							  &changed));
	KUNIT_EXPECT_EQ(test, mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_TRUE(test, over_grey);
	KUNIT_EXPECT_EQ(test, t_px(dst, 2, 2), 15);
	KUNIT_EXPECT_EQ(test, t_px(dst, 5, 1), 7);
	t_snapshot(du, prev, next, &box, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, memcmp(dst, du, T_SIZE), 0);
	t_expect_guard(test, dst);

	/* No other mode is raised; each reports the grey all the same. */
	for (i = 0; i < ARRAY_SIZE(kept); i++) {
		mode = kept[i];
		over_grey = false;
		rk3026_ebc_build_snapshot(&sh, next, &box, &mode, &two_level,
					  &over_grey, &changed);
		KUNIT_EXPECT_EQ(test, mode, kept[i]);
		KUNIT_EXPECT_TRUE(test, over_grey);
	}

	/* The grey left as it is: no mode finds a change over a grey. */
	t_px_set(next, 2, 2, 6);
	for (i = 0; i < ARRAY_SIZE(kept); i++) {
		mode = kept[i];
		over_grey = true;
		rk3026_ebc_build_snapshot(&sh, next, &box, &mode, &two_level,
					  &over_grey, &changed);
		KUNIT_EXPECT_FALSE(test, over_grey);
	}
}

/*
 * The target compared with the glass over the box, widened to whole bytes
 * as the snapshot widens it: a change outside the box is not seen, one on
 * the widened edge is; where it finds none the snapshot moves nothing.
 */
static void ebc_test_target_differs(struct kunit *test)
{
	u8 *prev = t_img_alloc(test, 15);
	u8 *next = t_img_alloc(test, 15);
	u8 *dst = t_img_alloc(test, 15);
	const struct rk3026_ebc_shadows sh = {
		.prev = prev, .pitch = T_PITCH, .height = T_H,
	};
	/* x 5..9, widened to 4..10. */
	struct drm_rect box = DRM_RECT_INIT(5, 2, 4, 2);
	struct drm_rect off = DRM_RECT_INIT(T_W, 0, 4, 2);

	KUNIT_EXPECT_FALSE(test, rk3026_ebc_target_differs(&sh, next, &box));

	t_px_set(next, 12, 3, 0);
	t_px_set(next, 6, 1, 0);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_target_differs(&sh, next, &box));
	KUNIT_EXPECT_FALSE(test, t_snapshot(dst, prev, next, &box,
					    EBC_MODE_FULL));

	t_px_set(next, 4, 2, 0);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_target_differs(&sh, next, &box));

	t_img_fill(next, 15);
	t_px_set(next, 8, 3, 7);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_target_differs(&sh, next, &box));
	t_px_set(next, 8, 3, 15);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_target_differs(&sh, next, &box));

	t_img_fill(next, 0);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_target_differs(&sh, next, &off));
	t_expect_guard(test, prev);
	t_expect_guard(test, next);
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

	/*
	 * Nor is the idle cleanup: it redraws the glass as it is, so its
	 * snapshot is identical as a rule. A plane update merged into it does
	 * not make it skip either.
	 */
	t_add(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_IDLE);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_TRUE(test, part.cleanup_idle);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_should_skip(&part, false, true));
	t_add(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_IDLE);
	t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&p, &part);
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

/*
 * Only a RESET or a FULL, in the full DSP_CTRL class, makes a glass that is
 * not known known: a PART, DU or A2 scan moves only the pixels whose prev
 * differs, and prev is what is not known. force_mode can drive a LUT in
 * another class, and the class decides too.
 */
static void ebc_test_drive_knows_glass(struct kunit *test)
{
	static const struct {
		u32	mode;
		u32	lut_mode;
		bool	knows;
	} rows[] = {
		{ EBC_MODE_RESET, EBC_MODE_RESET, true },
		{ EBC_MODE_FULL, EBC_MODE_FULL, true },
		{ EBC_MODE_PART, EBC_MODE_PART, false },
		{ EBC_MODE_BLACK_WHITE, EBC_MODE_BLACK_WHITE, false },
		{ EBC_MODE_A2, EBC_MODE_A2, false },
		{ EBC_MODE_OED_PART, EBC_MODE_OED_PART, false },
		{ EBC_MODE_FULL_DITHER, EBC_MODE_FULL_DITHER, false },
		/* force_mode: a partial class over a FULL or a RESET LUT, */
		{ EBC_MODE_PART, EBC_MODE_FULL, false },
		{ EBC_MODE_PART, EBC_MODE_RESET, false },
		/* a direct mode over the FULL LUT it falls back to, */
		{ EBC_MODE_DIRECT_PART, EBC_MODE_FULL, false },
		/* a full class over it. */
		{ EBC_MODE_BLOCK, EBC_MODE_FULL, true },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rows); i++)
		KUNIT_EXPECT_EQ_MSG(test,
				    rk3026_ebc_drive_knows_glass(rows[i].mode,
								 rows[i].lut_mode),
				    rows[i].knows, "row %u", i);
}

/* --- sequences of add, take and finish --- */

struct t_ctx {
	struct rk3026_ebc_pending	p;
	struct rk3026_ebc_marker_ring	ring;
	struct rk3026_ebc_shadows	sh;
	u8				*prev;
	u8				*scan;
	enum rk3026_ebc_gave_up		gave_up;	/* of the last finish */
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
	rk3026_ebc_scan_finish(&c->sh, &c->p, part, outcome, EBC_REPAIR_NONE,
			       0, &c->gave_up);
}

/* t_finish() at @ms on the time line of the carry's retries. */
static void t_finish_at(struct t_ctx *c, const struct rk3026_ebc_set *part,
			enum rk3026_ebc_outcome outcome, s64 ms)
{
	rk3026_ebc_scan_finish(&c->sh, &c->p, part, outcome, EBC_REPAIR_NONE,
			       ms_to_ktime(ms), &c->gave_up);
}

/*
 * The refresh thread's look at @ms with the CRTC on, then its take: the
 * carry alone when its retry is due, else whatever is pending.
 */
static bool t_retry_take(struct t_ctx *c, s64 ms, struct rk3026_ebc_set *part)
{
	rk3026_ebc_retry_ready(&c->p, ms_to_ktime(ms), true);
	return rk3026_ebc_pending_take(&c->p, part);
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
 * What the driver asks locklessly between parts: a part to take
 * (rk3026_ebc_take_pending(), the thread's wake) must agree with
 * rk3026_ebc_pending_empty() on every outcome, a carry included, or the
 * thread would sleep on work or spin on a carry; an update not finished
 * (rk3026_ebc_work_pending()) is one pending or held by a carry's retry.
 */
static void t_expect_between_parts(struct kunit *test,
				   const struct rk3026_ebc_pending *p)
{
	KUNIT_EXPECT_EQ(test, rk3026_ebc_take_pending(p),
			!rk3026_ebc_pending_empty(p));
	KUNIT_EXPECT_EQ(test, rk3026_ebc_work_pending(p),
			!rk3026_ebc_pending_empty(p) || p->retry.armed);
}

/* The outcomes after which a part is carried and tried again. */
static bool t_retried(enum rk3026_ebc_outcome outcome)
{
	return outcome == EBC_OUTCOME_NO_RAILS || outcome == EBC_OUTCOME_NO_LUT;
}

/*
 * done_seq moves over a part when it is driven, skipped, dropped or given
 * up; a part carried for a retry holds it below, and the RESET behind it,
 * which drops the carry, moves it on.
 */
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
		bool retried = t_retried(outcomes[i]);
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
			bool held = retried && part.mode != EBC_MODE_RESET;

			before = c->p.done_seq;
			/* in flight: still work, nothing more to take */
			KUNIT_EXPECT_TRUE(test, rk3026_ebc_work_pending(&c->p));
			KUNIT_EXPECT_EQ(test, c->p.taken_seq, part.seq);
			rk3026_ebc_scan_finish(&c->sh, &c->p, &part,
					       outcomes[i], EBC_REPAIR_NONE, 0, &c->gave_up);
			if (held) {
				KUNIT_EXPECT_EQ(test, c->p.done_seq, before);
				KUNIT_EXPECT_TRUE(test, c->p.retry.armed);
			} else {
				KUNIT_EXPECT_EQ(test, c->p.done_seq, part.seq);
				KUNIT_EXPECT_GT(test,
						(s32)(c->p.done_seq - before), 0);
			}
			t_expect_between_parts(test, &c->p);
			parts++;
		}
		KUNIT_EXPECT_EQ(test, parts, 3);
		KUNIT_EXPECT_EQ(test, t_done(c, last), !retried);
		KUNIT_EXPECT_EQ(test, c->p.done_seq == c->p.queued_seq,
				!retried);
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

/*
 * A part driven to its end: the snapshot is prev, and the mark is what
 * rk3026_ebc_drive() left: clear after a drive that knew the glass or made
 * it known, still set after a PART over a glass that is not known.
 */
static void ebc_test_finish_driven(struct kunit *test)
{
	unsigned int i;

	for (i = 0; i < 2; i++) {
		struct t_ctx *c = t_ctx_alloc(test);
		struct rk3026_ebc_set part;

		c->sh.prev_unknown = i;
		t_add(&c->p, &t_panel, EBC_MODE_PART, EBC_SOURCE_PLANE);
		rk3026_ebc_pending_take(&c->p, &part);
		KUNIT_EXPECT_TRUE(test,
				  rk3026_ebc_scan_finish(&c->sh, &c->p, &part,
							 EBC_OUTCOME_DRIVEN,
							 EBC_REPAIR_NONE, 0, &c->gave_up));
		KUNIT_EXPECT_PTR_EQ(test, c->sh.prev, c->scan);
		KUNIT_EXPECT_PTR_EQ(test, c->sh.scan, c->prev);
		KUNIT_EXPECT_EQ(test, c->sh.prev_unknown, i);
		KUNIT_EXPECT_EQ(test, c->p.done_seq, part.seq);
		KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
	}
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

			/* As the drive of the part, or of its repair, left it. */
			c->sh.prev_unknown = false;
			t_add(&c->p, &t_panel, EBC_MODE_PART, EBC_SOURCE_PLANE);
			rk3026_ebc_pending_take(&c->p, &part);
			/* Partly driven: the snapshot is the best prev there is. */
			KUNIT_EXPECT_TRUE(test,
					  rk3026_ebc_scan_finish(&c->sh, &c->p,
								 &part,
								 outcomes[i],
								 repairs[j], 0, &c->gave_up));
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
							  EBC_REPAIR_FAILED, 0, &c->gave_up));
		KUNIT_EXPECT_PTR_EQ(test, c->sh.prev, c->prev);
		KUNIT_EXPECT_EQ(test, c->prev[0], 0x77);
		KUNIT_EXPECT_TRUE(test, c->sh.prev_unknown);
		/* Tried again, like a part without rails: its wait holds. */
		KUNIT_EXPECT_FALSE(test, t_done(c, seq));
		KUNIT_EXPECT_TRUE(test, c->p.retry.armed);
		KUNIT_EXPECT_TRUE(test, drm_rect_equals(&c->p.carry.box, &r));
		KUNIT_EXPECT_TRUE(test, rk3026_ebc_work_pending(&c->p));
		KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_pending(&c->p));

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
		bool retried = t_retried(outcomes[i]);

		seq = t_add_marked(&c->p, &c->ring, &a, EBC_MODE_FULL, 1);
		rk3026_ebc_pending_take(&c->p, &part);
		KUNIT_EXPECT_FALSE(test, rk3026_ebc_scan_finish(&c->sh, &c->p,
								&part,
								outcomes[i],
								EBC_REPAIR_NONE, 0, &c->gave_up));
		/*
		 * Nothing driven: prev stays, nothing to take. The wait
		 * holds for the retries; without a waveform the part is
		 * given up at once and its wait ends.
		 */
		KUNIT_EXPECT_PTR_EQ(test, c->sh.prev, c->prev);
		KUNIT_EXPECT_EQ(test, t_done(c, seq), !retried);
		KUNIT_EXPECT_EQ(test, c->p.retry.armed, retried);
		KUNIT_EXPECT_EQ(test, c->gave_up != EBC_GAVE_UP_NONE, !retried);
		KUNIT_EXPECT_TRUE(test, rk3026_ebc_pending_empty(&c->p));
		KUNIT_EXPECT_EQ(test, rk3026_ebc_work_pending(&c->p), retried);
		KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_pending(&c->p));
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
								EBC_REPAIR_NONE, 0, &c->gave_up));
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

	/* Its drive, a RESET, cleared the mark (rk3026_ebc_drive()). */
	c->sh.prev_unknown = false;
	t_add(&c->p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_ENABLE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_scan_finish(&c->sh, &c->p, &part,
						       EBC_OUTCOME_DRIVEN,
						       EBC_REPAIR_NONE, 0, &c->gave_up));
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
								EBC_REPAIR_NONE, 0, &c->gave_up));
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
							 EBC_REPAIR_NONE, 0, &c->gave_up));
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

	/* The repair's drive, a RESET, cleared the mark (rk3026_ebc_drive()). */
	c->sh.prev_unknown = false;
	t_add(&c->p, &r, EBC_MODE_A2, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_scan_mode(&part, true), EBC_MODE_PART);
	/* The repair whitened the glass, then the LUT was missing. */
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_scan_finish(&c->sh, &c->p, &part,
						       EBC_OUTCOME_NO_LUT,
						       EBC_REPAIR_DONE, 0, &c->gave_up));
	t_expect_white(test, c->sh.prev);
	KUNIT_EXPECT_FALSE(test, c->sh.prev_unknown);

	t_add(&c->p, &r, EBC_MODE_A2, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&part.box, &t_panel));
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_PART);
}

/*
 * The retry schedule (rk3026_ebc_retry_next()), in ms from the first carry:
 * the next try is the first of 100, 500 and 2100 after the one that failed;
 * after 2100, the span the waits allow for, none: the part is given up.
 */
static void ebc_test_retry_next(struct kunit *test)
{
	static const struct {
		s64	now;
		bool	more;
		s64	at;
	} rows[] = {
		{ 0, true, 100 }, { 99, true, 100 }, { 100, true, 500 },
		{ 250, true, 500 }, { 500, true, EBC_RETRY_SPAN_MS },
		{ EBC_RETRY_SPAN_MS - 1, true, EBC_RETRY_SPAN_MS },
		{ EBC_RETRY_SPAN_MS, false }, { 60000, false },
	};
	ktime_t first = ms_to_ktime(1000);
	unsigned int i;

	KUNIT_EXPECT_EQ(test, EBC_RETRY_SPAN_MS, 100 + 400 + 1600);
	for (i = 0; i < ARRAY_SIZE(rows); i++) {
		ktime_t at = 12345;

		KUNIT_EXPECT_EQ_MSG(test,
				    rk3026_ebc_retry_next(first,
							  ktime_add_ms(first,
								       rows[i].now),
							  &at),
				    rows[i].more, "failed at %lld ms",
				    rows[i].now);
		KUNIT_EXPECT_EQ(test, at, rows[i].more ?
				ktime_add_ms(first, rows[i].at) : 12345);
	}
}

/*
 * A carry keeps its part's marker, source and temperature: taken alone at
 * its retry, the part has them. Merged into the next part, that part's
 * own win, and a temperature it lacks comes from the carry.
 */
static void ebc_test_carry_record(struct kunit *test)
{
	struct t_ctx *c = t_ctx_alloc(test);
	struct drm_rect a = DRM_RECT_INIT(0, 0, 4, 2);
	struct rk3026_ebc_update upd = t_upd(&a, EBC_MODE_PART, 7,
					     EBC_SOURCE_MXCFB);
	struct rk3026_ebc_set part;
	u32 seq;

	upd.temp = 12;
	upd.temp_valid = true;
	rk3026_ebc_pending_add(&c->p, &upd, &seq);
	rk3026_ebc_pending_take(&c->p, &part);
	t_finish_at(c, &part, EBC_OUTCOME_NO_RAILS, 1000);
	KUNIT_ASSERT_TRUE(test, t_retry_take(c, 1100, &part));
	KUNIT_EXPECT_EQ(test, part.seq, seq);
	KUNIT_EXPECT_EQ(test, part.marker, 7);
	KUNIT_EXPECT_EQ(test, part.source, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, part.temp_valid);
	KUNIT_EXPECT_EQ(test, part.temp, 12);

	/* Carried again; a plane update with no temperature joins it. */
	t_finish_at(c, &part, EBC_OUTCOME_NO_RAILS, 1100);
	t_add(&c->p, &a, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_EQ(test, part.marker, 0);
	KUNIT_EXPECT_EQ(test, part.source, EBC_SOURCE_PLANE);
	KUNIT_EXPECT_TRUE(test, part.temp_valid);
	KUNIT_EXPECT_EQ(test, part.temp, 12);

	/* One that brings its own temperature keeps it. */
	t_finish_at(c, &part, EBC_OUTCOME_NO_RAILS, 1200);
	upd = t_upd(&a, EBC_MODE_PART, 8, EBC_SOURCE_MXCFB);
	upd.temp = 30;
	upd.temp_valid = true;
	rk3026_ebc_pending_add(&c->p, &upd, &seq);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_EQ(test, part.marker, 8);
	KUNIT_EXPECT_EQ(test, part.temp, 30);
}

/*
 * A RESET that was not driven (no rails, no LUT, no waveform) is not
 * carried: the glass is marked not known, for the next draw to repair,
 * its waits end, and it is given up, so it is counted. It does not make
 * the next carry give up at once. Driven, it is not given up.
 */
static void ebc_test_finish_reset_lost(struct kunit *test)
{
	static const enum rk3026_ebc_outcome outcomes[] = {
		EBC_OUTCOME_NO_RAILS, EBC_OUTCOME_NO_LUT,
		EBC_OUTCOME_NO_WAVEFORM, EBC_OUTCOME_DRIVEN,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(outcomes); i++) {
		bool lost = outcomes[i] != EBC_OUTCOME_DRIVEN;
		struct t_ctx *c = t_ctx_alloc(test);
		struct rk3026_ebc_set part;
		u32 seq;

		seq = t_add(&c->p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_ENABLE);
		rk3026_ebc_pending_take(&c->p, &part);
		t_finish_at(c, &part, outcomes[i], 1000);
		KUNIT_EXPECT_EQ(test, c->gave_up,
				lost ? EBC_GAVE_UP : EBC_GAVE_UP_NONE);
		KUNIT_EXPECT_EQ(test, c->sh.prev_unknown, lost);
		KUNIT_EXPECT_TRUE(test, t_done(c, seq));
		KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
		KUNIT_EXPECT_FALSE(test, c->p.retry.armed);
		KUNIT_EXPECT_FALSE(test, c->p.retry.failing);
	}
}

/*
 * A carried part on the time line of its retries: carried at 1000 ms, its
 * wait held; nothing to take before 1100, then the carry alone, its seq,
 * nothing new taken; an update that joins it at 1200 is tried with it and
 * moves neither the next try (1500) nor the last (3100); the try at 3100
 * gives both up, their waits end, the carry stays for the next part. That
 * part gives up at once (failing), until a part is driven; the next carry
 * is tried again, on a schedule of its own.
 */
static void ebc_test_finish_retry(struct kunit *test)
{
	struct t_ctx *c = t_ctx_alloc(test);
	struct rk3026_ebc_retry *r = &c->p.retry;
	struct drm_rect a = DRM_RECT_INIT(0, 0, 4, 2);
	struct drm_rect b = DRM_RECT_INIT(8, 4, 2, 2);
	struct rk3026_ebc_set part;
	u32 s1, s2, s3, s4;

	s1 = t_add_marked(&c->p, &c->ring, &a, EBC_MODE_FULL, 1);
	rk3026_ebc_pending_take(&c->p, &part);
	t_finish_at(c, &part, EBC_OUTCOME_NO_RAILS, 1000);
	KUNIT_EXPECT_TRUE(test, r->armed);
	KUNIT_EXPECT_EQ(test, r->at, ms_to_ktime(1100));
	KUNIT_EXPECT_EQ(test, r->tries, 0);
	KUNIT_EXPECT_EQ(test, c->gave_up, EBC_GAVE_UP_NONE);
	KUNIT_EXPECT_FALSE(test, t_done(c, s1));
	KUNIT_EXPECT_EQ(test, c->p.carry.seq, s1);
	/* Not due: the next take would not have the carry in it. */
	KUNIT_EXPECT_EQ(test, rk3026_ebc_retry_try(&c->p), 0);

	KUNIT_EXPECT_FALSE(test, t_retry_take(c, 1099, &part));
	KUNIT_EXPECT_TRUE(test, c->p.carry.merged);
	rk3026_ebc_retry_ready(&c->p, ms_to_ktime(1100), true);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_retry_try(&c->p), 1);
	KUNIT_ASSERT_TRUE(test, t_retry_take(c, 1100, &part));
	KUNIT_EXPECT_EQ(test, part.seq, s1);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&part.box, &a));
	KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
	KUNIT_EXPECT_EQ(test, c->p.taken_seq, s1);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_pending(&c->p));
	t_finish_at(c, &part, EBC_OUTCOME_NO_RAILS, 1100);
	KUNIT_EXPECT_EQ(test, r->at, ms_to_ktime(1500));
	KUNIT_EXPECT_EQ(test, r->tries, 1);

	s2 = t_add_marked(&c->p, &c->ring, &b, EBC_MODE_PART, 2);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_retry_try(&c->p), 2);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
	KUNIT_EXPECT_EQ(test, part.seq, s2);
	KUNIT_EXPECT_EQ(test, part.box.x2, 10);
	t_finish_at(c, &part, EBC_OUTCOME_NO_LUT, 1200);
	KUNIT_EXPECT_EQ(test, r->at, ms_to_ktime(1500));
	KUNIT_EXPECT_EQ(test, r->first, ms_to_ktime(1000));
	KUNIT_EXPECT_EQ(test, r->tries, 2);
	KUNIT_EXPECT_EQ(test, c->p.carry.seq, s2);
	KUNIT_EXPECT_FALSE(test, t_done(c, s1));

	KUNIT_ASSERT_TRUE(test, t_retry_take(c, 1500, &part));
	t_finish_at(c, &part, EBC_OUTCOME_NO_RAILS, 1500);
	KUNIT_EXPECT_EQ(test, r->at, ms_to_ktime(3100));
	KUNIT_EXPECT_FALSE(test, t_retry_take(c, 3099, &part));
	KUNIT_EXPECT_EQ(test, rk3026_ebc_retry_try(&c->p), 0);
	rk3026_ebc_retry_ready(&c->p, ms_to_ktime(3100), true);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_retry_try(&c->p), 4);
	KUNIT_ASSERT_TRUE(test, t_retry_take(c, 3100, &part));
	t_finish_at(c, &part, EBC_OUTCOME_NO_RAILS, 3100);
	KUNIT_EXPECT_EQ(test, c->gave_up, EBC_GAVE_UP);
	KUNIT_EXPECT_TRUE(test, r->failing);
	KUNIT_EXPECT_FALSE(test, r->armed);
	KUNIT_EXPECT_TRUE(test, t_done(c, s1));
	KUNIT_EXPECT_TRUE(test, t_done(c, s2));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&c->p));
	KUNIT_EXPECT_TRUE(test, c->p.carry.merged);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_retry_try(&c->p), 0);
	KUNIT_EXPECT_FALSE(test, t_retry_take(c, 60000, &part));

	s3 = t_add_marked(&c->p, &c->ring, &a, EBC_MODE_PART, 3);
	rk3026_ebc_pending_take(&c->p, &part);
	t_finish_at(c, &part, EBC_OUTCOME_NO_RAILS, 4000);
	KUNIT_EXPECT_EQ(test, c->gave_up, EBC_GAVE_UP_FAILING);
	KUNIT_EXPECT_FALSE(test, r->armed);
	KUNIT_EXPECT_TRUE(test, t_done(c, s3));

	t_add_marked(&c->p, &c->ring, &a, EBC_MODE_PART, 4);
	rk3026_ebc_pending_take(&c->p, &part);
	t_finish_at(c, &part, EBC_OUTCOME_DRIVEN, 5000);
	KUNIT_EXPECT_FALSE(test, r->failing);
	KUNIT_EXPECT_EQ(test, c->gave_up, EBC_GAVE_UP_NONE);
	KUNIT_EXPECT_FALSE(test, c->p.carry.merged);

	s4 = t_add_marked(&c->p, &c->ring, &b, EBC_MODE_PART, 5);
	rk3026_ebc_pending_take(&c->p, &part);
	t_finish_at(c, &part, EBC_OUTCOME_NO_LUT, 6000);
	KUNIT_EXPECT_TRUE(test, r->armed);
	KUNIT_EXPECT_EQ(test, r->first, ms_to_ktime(6000));
	KUNIT_EXPECT_EQ(test, r->at, ms_to_ktime(6100));
	KUNIT_EXPECT_FALSE(test, t_done(c, s4));
}

/*
 * What ends a retry besides a give-up: the try drawn, skipped, dropped with
 * the CRTC off, or cut short in its waveform; the CRTC off makes it due at
 * once. A repair cut short is retried like missing rails. A RESET sent
 * drops the carry: the next look disarms the retry, and the RESET's finish
 * ends the waits; a carry made while a RESET was pending (its part was in
 * flight) is dropped by the RESET's finish.
 */
static void ebc_test_finish_retry_ends(struct kunit *test)
{
	static const enum rk3026_ebc_outcome outcomes[] = {
		EBC_OUTCOME_DRIVEN, EBC_OUTCOME_SKIPPED, EBC_OUTCOME_CRTC_OFF,
		EBC_OUTCOME_TIMEOUT, EBC_OUTCOME_ABANDONED,
	};
	struct drm_rect a = DRM_RECT_INIT(0, 0, 4, 2);
	struct rk3026_ebc_set part;
	struct t_ctx *c;
	unsigned int i;
	u32 seq, s2;

	for (i = 0; i < ARRAY_SIZE(outcomes); i++) {
		c = t_ctx_alloc(test);
		seq = t_add(&c->p, &a, EBC_MODE_PART, EBC_SOURCE_MXCFB);
		rk3026_ebc_pending_take(&c->p, &part);
		t_finish_at(c, &part, EBC_OUTCOME_NO_RAILS, 1000);
		KUNIT_ASSERT_TRUE(test, t_retry_take(c, 1100, &part));
		t_finish_at(c, &part, outcomes[i], 1100);
		KUNIT_EXPECT_TRUE(test, t_done(c, seq));
		KUNIT_EXPECT_FALSE(test, c->p.retry.armed);
		KUNIT_EXPECT_EQ(test, c->gave_up, EBC_GAVE_UP_NONE);
		KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
		KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&c->p));
	}

	c = t_ctx_alloc(test);
	seq = t_add(&c->p, &a, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&c->p, &part);
	t_finish_at(c, &part, EBC_OUTCOME_NO_LUT, 1000);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_retry_due(&c->p.retry,
						      ms_to_ktime(1001), true));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_retry_due(&c->p.retry,
						     ms_to_ktime(1001), false));
	rk3026_ebc_retry_ready(&c->p, ms_to_ktime(1001), false);
	KUNIT_EXPECT_TRUE(test, c->p.retry.ready);

	c = t_ctx_alloc(test);
	c->sh.prev_unknown = true;
	seq = t_add(&c->p, &a, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&c->p, &part);
	rk3026_ebc_scan_finish(&c->sh, &c->p, &part, EBC_OUTCOME_TIMEOUT,
			       EBC_REPAIR_FAILED, ms_to_ktime(1000), &c->gave_up);
	KUNIT_EXPECT_TRUE(test, c->p.retry.armed);
	KUNIT_EXPECT_FALSE(test, t_done(c, seq));

	s2 = t_add(&c->p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_ENABLE);
	KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
	rk3026_ebc_retry_ready(&c->p, ms_to_ktime(5000), true);
	KUNIT_EXPECT_FALSE(test, c->p.retry.ready);
	KUNIT_EXPECT_FALSE(test, c->p.retry.armed);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_RESET);
	t_finish_at(c, &part, EBC_OUTCOME_DRIVEN, 5000);
	KUNIT_EXPECT_TRUE(test, t_done(c, seq));
	KUNIT_EXPECT_TRUE(test, t_done(c, s2));

	c = t_ctx_alloc(test);
	seq = t_add(&c->p, &a, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&c->p, &part);
	s2 = t_add(&c->p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_ENABLE);
	t_finish_at(c, &part, EBC_OUTCOME_NO_RAILS, 1000);
	KUNIT_EXPECT_TRUE(test, c->p.carry.merged);
	KUNIT_EXPECT_TRUE(test, c->p.retry.armed);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_take(&c->p, &part));
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_RESET);
	t_finish_at(c, &part, EBC_OUTCOME_DRIVEN, 1050);
	KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
	KUNIT_EXPECT_FALSE(test, c->p.retry.armed);
	KUNIT_EXPECT_TRUE(test, t_done(c, seq));
	KUNIT_EXPECT_TRUE(test, t_done(c, s2));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&c->p));
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

/* --- the mode policy --- */

/* The parts of the resolve table, as their updates make them. */
enum t_kind {
	T_PLANE,		/* plane updates only */
	T_AUTO_PART,		/* mxcfb AUTO + PARTIAL only */
	T_PLANE_AUTO_PART,	/* both, merged */
	T_PLANE_DU,		/* a plane update and an explicit DU */
	T_AUTO_PART_DU,		/* AUTO + PARTIAL and an explicit DU */
	T_MXCFB_FULL,		/* AUTO + FULL: FULL, no auto */
	T_A2,			/* an explicit A2 */
	T_DU,			/* an explicit DU */
	T_RESET,
	T_IDLE,			/* the idle cleanup alone */
	T_IDLE_PLANE,		/* the idle cleanup, a plane update joined */
};

static void t_kind_part(enum t_kind kind, struct rk3026_ebc_set *part)
{
	struct rk3026_ebc_pending p = {};

	switch (kind) {
	case T_PLANE:
		t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
		t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
		break;
	case T_AUTO_PART:
		t_add_auto(&p, &t_panel, EBC_MODE_PART, EBC_SOURCE_MXCFB);
		break;
	case T_PLANE_AUTO_PART:
		t_add_auto(&p, &t_panel, EBC_MODE_PART, EBC_SOURCE_MXCFB);
		t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
		break;
	case T_PLANE_DU:
		t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
		t_add(&p, &t_panel, EBC_MODE_BLACK_WHITE, EBC_SOURCE_MXCFB);
		break;
	case T_AUTO_PART_DU:
		t_add_auto(&p, &t_panel, EBC_MODE_PART, EBC_SOURCE_MXCFB);
		t_add(&p, &t_panel, EBC_MODE_BLACK_WHITE, EBC_SOURCE_MXCFB);
		break;
	case T_MXCFB_FULL:
		t_add(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_MXCFB);
		break;
	case T_A2:
		t_add(&p, &t_panel, EBC_MODE_A2, EBC_SOURCE_MXCFB);
		break;
	case T_DU:
		t_add(&p, &t_panel, EBC_MODE_BLACK_WHITE, EBC_SOURCE_MXCFB);
		break;
	case T_RESET:
		t_add(&p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_ENABLE);
		break;
	case T_IDLE:
		t_add(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_IDLE);
		break;
	case T_IDLE_PLANE:
		t_add(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_IDLE);
		t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
		break;
	}
	rk3026_ebc_pending_take(&p, part);
}

/*
 * The DU budget a resolve row runs with, the pixels its part changes and
 * its box: a limit of 100, a big part of 50, a ceiling of 400; a small
 * part changes 10 pixels in a box of 10, a big one 50 in the whole panel
 * (96), and a big box holds 10 changed pixels in the whole panel, which
 * is small: big counts changed pixels, not the box.
 */
enum t_budget {
	T_FRESH,	/* nothing spent, a small part */
	T_FRESH_BIG,	/* nothing spent, a big part */
	T_FRESH_BOX,	/* nothing spent, a big box, a small part */
	T_NEAR_SMALL,	/* 95 spent, a small part */
	T_NEAR_BIG,	/* 60 spent, a big part */
	T_NEAR_BOX,	/* 95 spent, a big box, a small part */
	T_SPENT,	/* spent, a small part */
	T_SPENT_BIG,	/* spent, a big part */
	T_SPENT_BOX,	/* spent, a big box, a small part */
	T_CEIL,		/* at the ceiling, a small part */
	T_LOW_CEIL,	/* spent, a ceiling below the limit, a small part */
	T_NO_BUDGET,	/* a limit of 0, a ceiling of 1, a big part */
};

static void t_budget_of(enum t_budget kind, struct rk3026_ebc_du_budget *b,
			u32 *changed_px, struct drm_rect *box)
{
	*b = (struct rk3026_ebc_du_budget){ .area = 0, .limit = 100,
					    .big_area = 50, .ceiling = 400 };
	*changed_px = 10;
	*box = DRM_RECT_INIT(0, 0, 5, 2);
	switch (kind) {
	case T_FRESH:
		break;
	case T_FRESH_BIG:
		*changed_px = 50;
		*box = t_panel;
		break;
	case T_FRESH_BOX:
		*box = t_panel;
		break;
	case T_NEAR_SMALL:
		b->area = 95;
		break;
	case T_NEAR_BIG:
		b->area = 60;
		*changed_px = 50;
		*box = t_panel;
		break;
	case T_NEAR_BOX:
		b->area = 95;
		*box = t_panel;
		break;
	case T_SPENT:
		b->area = 100;
		break;
	case T_SPENT_BIG:
		b->area = 100;
		*changed_px = 50;
		*box = t_panel;
		break;
	case T_SPENT_BOX:
		b->area = 100;
		*box = t_panel;
		break;
	case T_CEIL:
		b->area = 400;
		break;
	case T_LOW_CEIL:
		b->area = 100;
		b->ceiling = 10;
		break;
	case T_NO_BUDGET:
		b->area = U32_MAX;
		b->limit = 0;
		b->ceiling = 1;
		*changed_px = U32_MAX;
		*box = t_panel;
		break;
	}
}

/*
 * The A2 inputs a resolve row runs with (struct rk3026_ebc_a2), for its
 * part of 10 changed pixels (50 if big). T_A2_OFF, the default, is the
 * probe's max_area of 0 and no grey: A2 off whatever else.
 */
enum t_a2 {
	T_A2_OFF,	/* max_area 0 */
	T_A2_ON,	/* max_area 10: the part is at the edge */
	T_A2_ONE_SHORT,	/* max_area 9: the part is one pixel over */
	T_A2_GREY,	/* max_area 10, a changed pixel over a grey */
	T_A2_ALL,	/* max_area U32_MAX */
};

static void t_a2_of(enum t_a2 kind, struct rk3026_ebc_a2 *a2)
{
	*a2 = (struct rk3026_ebc_a2){ .max_area = 10 };
	switch (kind) {
	case T_A2_OFF:
		a2->max_area = 0;
		break;
	case T_A2_ON:
		break;
	case T_A2_ONE_SHORT:
		a2->max_area = 9;
		break;
	case T_A2_GREY:
		a2->over_grey = true;
		break;
	case T_A2_ALL:
		a2->max_area = U32_MAX;
		break;
	}
}

struct t_resolve {
	const char		*desc;
	enum t_kind		kind;
	bool			repair;
	bool			two_level;
	u32			mode;
	enum rk3026_ebc_reason	reason;
	enum t_budget		budget;
	enum t_a2		a2;
};

#define T_DU_MODE	EBC_MODE_BLACK_WHITE
#define T_EXPL		EBC_REASON_EXPLICIT
#define T_AUTO_DU	EBC_REASON_AUTO_DU
#define T_FLOOR		EBC_REASON_REPAIR_FLOOR
#define T_BIG		EBC_REASON_CLEANUP_BIG
#define T_CEILING	EBC_REASON_CLEANUP_CEILING
#define T_AUTO_GREY	EBC_REASON_AUTO_PART
#define T_IDLE_CLEAN	EBC_REASON_CLEANUP_IDLE
#define T_AUTO_A2	EBC_REASON_AUTO_A2
/* Not a reason: what a test starts its result with. */
#define T_NO_REASON	((enum rk3026_ebc_reason)-1)

static const struct t_resolve t_resolve_cases[] = {
	{ "plane two-level", T_PLANE, false, true, T_DU_MODE, T_AUTO_DU },
	/* A grey plane part: PART, only the changed pixels flash. */
	{ "plane grey", T_PLANE, false, false, EBC_MODE_PART, T_AUTO_GREY },
	/* A repair keeps FULL: the floor raises a mode, never lowers one. */
	{ "plane repair two-level", T_PLANE, true, true, EBC_MODE_FULL,
	  T_EXPL },
	{ "plane repair grey", T_PLANE, true, false, EBC_MODE_FULL, T_EXPL },
	{ "auto part two-level", T_AUTO_PART, false, true, T_DU_MODE,
	  T_AUTO_DU },
	{ "auto part grey", T_AUTO_PART, false, false, EBC_MODE_PART, T_EXPL },
	{ "auto part repair two-level", T_AUTO_PART, true, true,
	  EBC_MODE_PART, T_EXPL },
	{ "auto part repair grey", T_AUTO_PART, true, false, EBC_MODE_PART,
	  T_EXPL },
	{ "plane and auto part two-level", T_PLANE_AUTO_PART, false, true,
	  T_DU_MODE, T_AUTO_DU },
	{ "plane and auto part grey", T_PLANE_AUTO_PART, false, false,
	  EBC_MODE_PART, T_AUTO_GREY },
	{ "plane and auto part repair grey", T_PLANE_AUTO_PART, true, false,
	  EBC_MODE_FULL, T_EXPL },
	/* An explicit mode merged in: the rank's mode, as before. */
	{ "plane and DU two-level", T_PLANE_DU, false, true, EBC_MODE_FULL,
	  T_EXPL },
	{ "plane and DU grey", T_PLANE_DU, false, false, EBC_MODE_FULL,
	  T_EXPL },
	{ "auto part and DU two-level", T_AUTO_PART_DU, false, true,
	  EBC_MODE_PART, T_EXPL },
	{ "auto part and DU grey", T_AUTO_PART_DU, false, false,
	  EBC_MODE_PART, T_EXPL },
	{ "mxcfb full two-level", T_MXCFB_FULL, false, true, EBC_MODE_FULL,
	  T_EXPL },
	{ "mxcfb full repair", T_MXCFB_FULL, true, true, EBC_MODE_FULL,
	  T_EXPL },
	/* The policy never picks A2, and never touches an explicit mode. */
	{ "A2 two-level", T_A2, false, true, EBC_MODE_A2, T_EXPL },
	{ "A2 repair", T_A2, true, true, EBC_MODE_PART, T_FLOOR },
	{ "DU two-level", T_DU, false, true, T_DU_MODE, T_EXPL },
	{ "DU repair", T_DU, true, true, EBC_MODE_PART, T_FLOOR },
	{ "RESET", T_RESET, false, true, EBC_MODE_RESET, T_EXPL },
	/* A big part on a budget it does not spend is no cleanup. */
	{ "plane two-level big", T_PLANE, false, true, T_DU_MODE, T_AUTO_DU,
	  T_FRESH_BIG },
	{ "plane grey big", T_PLANE, false, false, EBC_MODE_PART,
	  T_AUTO_GREY, T_FRESH_BIG },
	{ "plane two-level big box", T_PLANE, false, true, T_DU_MODE,
	  T_AUTO_DU, T_FRESH_BOX },
	{ "plane two-level near small", T_PLANE, false, true, T_DU_MODE,
	  T_AUTO_DU, T_NEAR_SMALL },
	/*
	 * A big two-level part that would spend the budget takes the
	 * cleanup now, rather than DU and the cleanup after it. A grey one
	 * is PART, which adds nothing to the budget.
	 */
	{ "plane two-level near big", T_PLANE, false, true, EBC_MODE_FULL,
	  T_BIG, T_NEAR_BIG },
	{ "plane and auto part two-level near big", T_PLANE_AUTO_PART, false,
	  true, EBC_MODE_FULL, T_BIG, T_NEAR_BIG },
	{ "plane grey near big", T_PLANE, false, false, EBC_MODE_PART,
	  T_AUTO_GREY, T_NEAR_BIG },
	{ "plane repair two-level near big", T_PLANE, true, true,
	  EBC_MODE_FULL, T_EXPL, T_NEAR_BIG },
	/*
	 * The DU budget is spent: a small auto part with a plane update
	 * stays DU or PART, so the flash does not land on a pointer move.
	 */
	{ "plane two-level spent", T_PLANE, false, true, T_DU_MODE,
	  T_AUTO_DU, T_SPENT },
	{ "plane grey spent", T_PLANE, false, false, EBC_MODE_PART,
	  T_AUTO_GREY, T_SPENT },
	{ "plane and auto part two-level spent", T_PLANE_AUTO_PART, false,
	  true, T_DU_MODE, T_AUTO_DU, T_SPENT },
	{ "plane and auto part grey spent", T_PLANE_AUTO_PART, false, false,
	  EBC_MODE_PART, T_AUTO_GREY, T_SPENT },
	/* A big one keeps FULL, two-level or grey. */
	{ "plane two-level spent big", T_PLANE, false, true, EBC_MODE_FULL,
	  T_BIG, T_SPENT_BIG },
	{ "plane grey spent big", T_PLANE, false, false, EBC_MODE_FULL,
	  T_BIG, T_SPENT_BIG },
	{ "plane repair two-level spent big", T_PLANE, true, true,
	  EBC_MODE_FULL, T_EXPL, T_SPENT_BIG },
	{ "plane and auto part two-level spent big", T_PLANE_AUTO_PART, false,
	  true, EBC_MODE_FULL, T_BIG, T_SPENT_BIG },
	{ "plane and auto part grey spent big", T_PLANE_AUTO_PART, false,
	  false, EBC_MODE_FULL, T_BIG, T_SPENT_BIG },
	{ "plane and DU two-level spent big", T_PLANE_DU, false, true,
	  EBC_MODE_FULL, T_EXPL, T_SPENT_BIG },
	{ "plane and DU grey spent big", T_PLANE_DU, false, false,
	  EBC_MODE_FULL, T_EXPL, T_SPENT_BIG },
	/*
	 * A big box of few changed pixels is small: a pointer that jumps
	 * far, two small changes far apart, a window over white. The box
	 * bounds the damage; it is no cleanup on a spent budget, nor when
	 * its pixels would spend it.
	 */
	{ "plane two-level spent big box", T_PLANE, false, true, T_DU_MODE,
	  T_AUTO_DU, T_SPENT_BOX },
	{ "plane grey spent big box", T_PLANE, false, false, EBC_MODE_PART,
	  T_AUTO_GREY, T_SPENT_BOX },
	{ "plane two-level near big box", T_PLANE, false, true, T_DU_MODE,
	  T_AUTO_DU, T_NEAR_BOX },
	{ "plane grey near big box", T_PLANE, false, false, EBC_MODE_PART,
	  T_AUTO_GREY, T_NEAR_BOX },
	/* At the ceiling any auto plane part keeps FULL, a small one too. */
	{ "plane two-level ceiling", T_PLANE, false, true, EBC_MODE_FULL,
	  T_CEILING, T_CEIL },
	{ "plane grey ceiling", T_PLANE, false, false, EBC_MODE_FULL,
	  T_CEILING, T_CEIL },
	{ "plane and auto part two-level ceiling", T_PLANE_AUTO_PART, false,
	  true, EBC_MODE_FULL, T_CEILING, T_CEIL },
	{ "plane and auto part grey ceiling", T_PLANE_AUTO_PART, false, false,
	  EBC_MODE_FULL, T_CEILING, T_CEIL },
	{ "plane repair grey ceiling", T_PLANE, true, false, EBC_MODE_FULL,
	  T_EXPL, T_CEIL },
	{ "plane and DU two-level ceiling", T_PLANE_DU, false, true,
	  EBC_MODE_FULL, T_EXPL, T_CEIL },
	/* A ceiling below the limit acts as the limit. */
	{ "plane two-level low ceiling", T_PLANE, false, true, EBC_MODE_FULL,
	  T_CEILING, T_LOW_CEIL },
	/* No budget (a limit of 0): no cleanup, the ceiling ignored. */
	{ "plane two-level no budget", T_PLANE, false, true, T_DU_MODE,
	  T_AUTO_DU, T_NO_BUDGET },
	{ "plane grey no budget", T_PLANE, false, false, EBC_MODE_PART,
	  T_AUTO_GREY, T_NO_BUDGET },
	/* An mxcfb client runs its own fulls: its auto parts stay DU. */
	{ "auto part two-level spent big", T_AUTO_PART, false, true,
	  T_DU_MODE, T_AUTO_DU, T_SPENT_BIG },
	{ "auto part grey spent big", T_AUTO_PART, false, false,
	  EBC_MODE_PART, T_EXPL, T_SPENT_BIG },
	{ "auto part two-level near big", T_AUTO_PART, false, true,
	  T_DU_MODE, T_AUTO_DU, T_NEAR_BIG },
	{ "auto part two-level spent big box", T_AUTO_PART, false, true,
	  T_DU_MODE, T_AUTO_DU, T_SPENT_BOX },
	{ "auto part two-level ceiling", T_AUTO_PART, false, true, T_DU_MODE,
	  T_AUTO_DU, T_CEIL },
	{ "auto part grey ceiling", T_AUTO_PART, false, false, EBC_MODE_PART,
	  T_EXPL, T_CEIL },
	{ "DU two-level spent big", T_DU, false, true, T_DU_MODE, T_EXPL,
	  T_SPENT_BIG },
	{ "DU two-level ceiling", T_DU, false, true, T_DU_MODE, T_EXPL,
	  T_CEIL },
	{ "mxcfb full ceiling", T_MXCFB_FULL, false, true, EBC_MODE_FULL,
	  T_EXPL, T_CEIL },
	{ "RESET ceiling", T_RESET, false, true, EBC_MODE_RESET, T_EXPL,
	  T_CEIL },
	/*
	 * The idle cleanup is FULL whatever its content, the budget or a
	 * repair: its own reason, also with a plane update joined.
	 */
	{ "idle cleanup", T_IDLE, false, false, EBC_MODE_FULL, T_IDLE_CLEAN },
	{ "idle cleanup two-level", T_IDLE, false, true, EBC_MODE_FULL,
	  T_IDLE_CLEAN },
	{ "idle cleanup repair", T_IDLE, true, false, EBC_MODE_FULL,
	  T_IDLE_CLEAN },
	{ "idle cleanup spent big", T_IDLE, false, true, EBC_MODE_FULL,
	  T_IDLE_CLEAN, T_SPENT_BIG },
	{ "idle cleanup ceiling", T_IDLE, false, true, EBC_MODE_FULL,
	  T_IDLE_CLEAN, T_CEIL },
	{ "idle cleanup and plane two-level", T_IDLE_PLANE, false, true,
	  EBC_MODE_FULL, T_IDLE_CLEAN },
	{ "idle cleanup and plane grey", T_IDLE_PLANE, false, false,
	  EBC_MODE_FULL, T_IDLE_CLEAN },
	/*
	 * A2 on: a small two-level plane part, at most max_area changed
	 * pixels, none over a grey, goes A2.
	 */
	{ "A2 on plane two-level", T_PLANE, false, true, EBC_MODE_A2,
	  T_AUTO_A2, T_FRESH, T_A2_ON },
	{ "A2 on plane and auto part two-level", T_PLANE_AUTO_PART, false,
	  true, EBC_MODE_A2, T_AUTO_A2, T_FRESH, T_A2_ON },
	/* One pixel over max_area, a grey under a change: DU. */
	{ "A2 on plane two-level one over", T_PLANE, false, true, T_DU_MODE,
	  T_AUTO_DU, T_FRESH, T_A2_ONE_SHORT },
	{ "A2 on plane two-level over grey", T_PLANE, false, true, T_DU_MODE,
	  T_AUTO_DU, T_FRESH, T_A2_GREY },
	/* A big part on a budget not spent: A2 if max_area allows it. */
	{ "A2 on plane two-level big", T_PLANE, false, true, EBC_MODE_A2,
	  T_AUTO_A2, T_FRESH_BIG, T_A2_ALL },
	{ "A2 on plane two-level big over max", T_PLANE, false, true,
	  T_DU_MODE, T_AUTO_DU, T_FRESH_BIG, T_A2_ON },
	/* A grey part, a repair, an explicit mode merged in: as before. */
	{ "A2 on plane grey", T_PLANE, false, false, EBC_MODE_PART,
	  T_AUTO_GREY, T_FRESH, T_A2_ALL },
	{ "A2 on plane repair two-level", T_PLANE, true, true, EBC_MODE_FULL,
	  T_EXPL, T_FRESH, T_A2_ALL },
	{ "A2 on plane and DU two-level", T_PLANE_DU, false, true,
	  EBC_MODE_FULL, T_EXPL, T_FRESH, T_A2_ALL },
	/* An mxcfb client names A2 itself: its auto parts stay DU. */
	{ "A2 on auto part two-level", T_AUTO_PART, false, true, T_DU_MODE,
	  T_AUTO_DU, T_FRESH, T_A2_ALL },
	{ "A2 on A2 two-level", T_A2, false, true, EBC_MODE_A2, T_EXPL,
	  T_FRESH, T_A2_ALL },
	{ "A2 on DU two-level", T_DU, false, true, T_DU_MODE, T_EXPL,
	  T_FRESH, T_A2_ALL },
	/* A spent budget: a small part stays A2, a big one cleans up. */
	{ "A2 on plane two-level spent", T_PLANE, false, true, EBC_MODE_A2,
	  T_AUTO_A2, T_SPENT, T_A2_ON },
	{ "A2 on plane two-level spent big", T_PLANE, false, true,
	  EBC_MODE_FULL, T_BIG, T_SPENT_BIG, T_A2_ALL },
	{ "A2 on plane two-level near big", T_PLANE, false, true,
	  EBC_MODE_FULL, T_BIG, T_NEAR_BIG, T_A2_ALL },
	{ "A2 on plane two-level spent big box", T_PLANE, false, true,
	  EBC_MODE_A2, T_AUTO_A2, T_SPENT_BOX, T_A2_ON },
	{ "A2 on plane two-level ceiling", T_PLANE, false, true,
	  EBC_MODE_FULL, T_CEILING, T_CEIL, T_A2_ON },
	{ "A2 on plane two-level no budget", T_PLANE, false, true,
	  EBC_MODE_A2, T_AUTO_A2, T_NO_BUDGET, T_A2_ALL },
	/* The idle cleanup stays FULL. */
	{ "A2 on idle cleanup two-level", T_IDLE, false, true, EBC_MODE_FULL,
	  T_IDLE_CLEAN, T_FRESH, T_A2_ALL },
	{ "A2 on idle cleanup and plane two-level", T_IDLE_PLANE, false, true,
	  EBC_MODE_FULL, T_IDLE_CLEAN, T_FRESH, T_A2_ALL },
};

KUNIT_ARRAY_PARAM_DESC(t_resolve, t_resolve_cases, desc);

/*
 * Each row as it is. A row with A2 off (max_area 0) is run with every
 * other A2 input as well and gives the same: with the probe's default
 * the policy is the one before A2, whatever the waveform and the glass.
 */
static void ebc_test_resolve_mode(struct kunit *test)
{
	const struct t_resolve *c = test->param_value;
	enum rk3026_ebc_reason reason = T_NO_REASON;
	enum rk3026_ebc_repair_due repair;
	struct rk3026_ebc_du_budget budget;
	struct rk3026_ebc_set part;
	struct rk3026_ebc_a2 a2;
	u32 scan_mode, changed_px;
	struct drm_rect box;
	unsigned int i;

	t_kind_part(c->kind, &part);
	t_budget_of(c->budget, &budget, &changed_px, &box);
	part.box = box;
	t_a2_of(c->a2, &a2);
	repair = c->repair ? EBC_REPAIR_BY_RESET : EBC_REPAIR_NOT_DUE;
	scan_mode = rk3026_ebc_scan_mode(&part, c->repair);
	KUNIT_EXPECT_EQ(test,
			rk3026_ebc_resolve_mode(&part, scan_mode, c->two_level,
						changed_px, repair, &budget,
						&a2, &reason),
			c->mode);
	KUNIT_EXPECT_EQ(test, reason, c->reason);

	if (c->a2 != T_A2_OFF)
		return;
	for (i = 0; i < 2; i++) {
		a2 = (struct rk3026_ebc_a2){ .max_area = 0, .over_grey = i };
		reason = T_NO_REASON;
		KUNIT_EXPECT_EQ(test,
				rk3026_ebc_resolve_mode(&part, scan_mode,
							c->two_level,
							changed_px, repair,
							&budget, &a2, &reason),
				c->mode);
		KUNIT_EXPECT_EQ(test, reason, c->reason);
	}
}

/*
 * A repair due without a RESET at the temperature (EBC_REPAIR_BY_FULL):
 * the part is the repair and runs FULL, whatever it asked, its content, the
 * budget or A2, for the repair's reason when that raised it. The idle
 * cleanup keeps its own reason (FULL anyway).
 */
static void ebc_test_resolve_repair_full(struct kunit *test)
{
	static const enum t_kind kinds[] = {
		T_PLANE, T_AUTO_PART, T_PLANE_AUTO_PART, T_PLANE_DU,
		T_AUTO_PART_DU, T_MXCFB_FULL, T_A2, T_DU, T_IDLE, T_IDLE_PLANE,
	};
	static const enum t_budget budgets[] = {
		T_FRESH, T_SPENT_BIG, T_CEIL,
	};
	static const enum t_a2 a2s[] = { T_A2_OFF, T_A2_ALL };
	const unsigned int per_kind = ARRAY_SIZE(budgets) * ARRAY_SIZE(a2s) * 2;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(kinds) * per_kind; i++) {
		enum t_budget bu = budgets[i / (ARRAY_SIZE(a2s) * 2) %
					   ARRAY_SIZE(budgets)];
		enum rk3026_ebc_reason want, reason = T_NO_REASON;
		enum t_a2 a2k = a2s[i / 2 % ARRAY_SIZE(a2s)];
		struct rk3026_ebc_du_budget budget;
		bool two_level = i % 2;
		struct rk3026_ebc_set part;
		struct rk3026_ebc_a2 a2;
		u32 changed_px, scan_mode;
		struct drm_rect box;

		t_kind_part(kinds[i / per_kind], &part);
		t_budget_of(bu, &budget, &changed_px, &box);
		part.box = box;
		t_a2_of(a2k, &a2);
		scan_mode = rk3026_ebc_scan_mode(&part, true);
		if (part.cleanup_idle)
			want = T_IDLE_CLEAN;
		else
			want = part.mode == EBC_MODE_FULL ? T_EXPL : T_FLOOR;
		KUNIT_EXPECT_EQ_MSG(test,
				    rk3026_ebc_resolve_mode(&part, scan_mode,
							    two_level,
							    changed_px,
							    EBC_REPAIR_BY_FULL,
							    &budget, &a2,
							    &reason),
				    EBC_MODE_FULL, "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, reason, want, "case %u", i);
	}
}

/*
 * The cleanup rule alone, at the edges of its knobs: rk3026_ebc_du_cleanup()
 * with @area spent of @limit, a big part of @big_area pixels, a @ceiling,
 * and a part that changes @changed pixels and would add @added to the
 * budget.
 */
struct t_du_cleanup {
	const char		*desc;
	u32			area;
	u32			limit;
	u32			big_area;
	u32			ceiling;
	u32			changed;
	u32			added;
	enum rk3026_ebc_reason	reason;		/* T_NO_REASON: not due */
};

static const struct t_du_cleanup t_du_cleanup_cases[] = {
	{ "nothing spent, big, adds nothing", 0, 100, 50, 400, U32_MAX, 0,
	  T_NO_REASON },
	{ "nothing spent, big, one short of spending it", 0, 100, 50, 400, 99,
	  99, T_NO_REASON },
	{ "nothing spent, big, spends it", 0, 100, 50, 400, 100, 100, T_BIG },
	{ "one short of the limit, big, adds nothing", 99, 100, 50, 400, 50,
	  0, T_NO_REASON },
	{ "one short of the limit, big, spends it", 99, 100, 50, 400, 50, 50,
	  T_BIG },
	/* A small part that would spend the budget is no cleanup. */
	{ "one short of the limit, small, spends it", 99, 100, 50, 400, 49,
	  49, T_NO_REASON },
	/* The projection saturates: no wrap back under the limit. */
	{ "one short of the limit, big, adds U32_MAX", 99, 100, 50, 400,
	  U32_MAX, U32_MAX, T_BIG },
	{ "spent, one short of big", 100, 100, 50, 400, 49, 49, T_NO_REASON },
	{ "spent, big", 100, 100, 50, 400, 50, 50, T_BIG },
	{ "spent, big, adds nothing", 100, 100, 50, 400, 50, 0, T_BIG },
	{ "over the limit, big", 399, 100, 50, 400, 50, 0, T_BIG },
	{ "over the limit, big, adds U32_MAX", 399, 100, 50, 400, U32_MAX,
	  U32_MAX, T_BIG },
	{ "one short of the ceiling", 399, 100, 50, 400, 49, 49, T_NO_REASON },
	{ "at the ceiling", 400, 100, 50, 400, 0, 0, T_CEILING },
	{ "past the ceiling", U32_MAX, 100, 50, 400, 49, 49, T_CEILING },
	/* A big part at the ceiling is reported big. */
	{ "at the ceiling, big", 400, 100, 50, 400, 50, 0, T_BIG },
	/* A ceiling of 0: none, only big parts clean up. */
	{ "no ceiling", U32_MAX, 100, 50, 0, 49, 0, T_NO_REASON },
	{ "no ceiling, big", U32_MAX, 100, 50, 0, 50, 0, T_BIG },
	/* A ceiling below the limit acts as the limit. */
	{ "ceiling below the limit", 100, 100, 50, 10, 0, 0, T_CEILING },
	{ "ceiling below the limit, not spent", 99, 100, 50, 10, 0, 0,
	  T_NO_REASON },
	{ "ceiling at the limit", 100, 100, 50, 100, 0, 0, T_CEILING },
	/*
	 * A big part of 0 pixels: every part is big once spent, or when it
	 * spends the budget (the rule before the big-part threshold).
	 */
	{ "big of 0", 100, 100, 0, 0, 0, 0, T_BIG },
	{ "big of 0, not spent", 99, 100, 0, 0, U32_MAX, 0, T_NO_REASON },
	{ "big of 0, spends it", 99, 100, 0, 0, 1, 1, T_BIG },
	/* A limit of 0: no budget, no cleanup of any kind. */
	{ "no budget", U32_MAX, 0, 0, 1, U32_MAX, U32_MAX, T_NO_REASON },
	{ "no budget, nothing spent", 0, 0, 0, 0, 0, 0, T_NO_REASON },
	/* The probe defaults for an area of 480000: area / 2, 4 * area. */
	{ "probe defaults, spent, small", 480000, 480000, 240000, 1920000,
	  239999, 239999, T_NO_REASON },
	{ "probe defaults, spent, big", 480000, 480000, 240000, 1920000,
	  240000, 240000, T_BIG },
	{ "probe defaults, ceiling", 1920000, 480000, 240000, 1920000, 1, 1,
	  T_CEILING },
	/* A whole frame drawn in DU on a fresh budget would spend it. */
	{ "probe defaults, fresh, whole frame", 0, 480000, 240000, 1920000,
	  480000, 480000, T_BIG },
	/* A window opened over white changes few pixels: small. */
	{ "probe defaults, spent, window over white", 480000, 480000, 240000,
	  1920000, 20000, 20000, T_NO_REASON },
};

KUNIT_ARRAY_PARAM_DESC(t_du_cleanup, t_du_cleanup_cases, desc);

static void ebc_test_du_cleanup(struct kunit *test)
{
	const struct t_du_cleanup *c = test->param_value;
	const struct rk3026_ebc_du_budget budget = {
		.area = c->area, .limit = c->limit, .big_area = c->big_area,
		.ceiling = c->ceiling,
	};
	enum rk3026_ebc_reason reason = T_NO_REASON;

	KUNIT_EXPECT_EQ(test,
			rk3026_ebc_du_cleanup(&budget, c->changed, c->added,
					      &reason),
			c->reason != T_NO_REASON);
	/* Left alone when no cleanup is due. */
	KUNIT_EXPECT_EQ(test, reason, c->reason);
}

/*
 * Spent and at the ceiling, as debugfs du_state reports them
 * (cleanup_due, ceiling_due): rk3026_ebc_du_spent() and
 * rk3026_ebc_du_at_ceiling() with @area spent of @limit and a @ceiling;
 * the big part enters neither.
 */
struct t_du_state {
	const char	*desc;
	u32		area;
	u32		limit;
	u32		big_area;
	u32		ceiling;
	bool		spent;
	bool		at_ceiling;
};

static const struct t_du_state t_du_state_cases[] = {
	{ "nothing spent", 0, 100, 50, 400, false, false },
	{ "one short of the limit", 99, 100, 50, 400, false, false },
	{ "spent", 100, 100, 50, 400, true, false },
	{ "spent, big of 0", 100, 100, 0, 400, true, false },
	{ "one short of the ceiling", 399, 100, 50, 400, true, false },
	{ "at the ceiling", 400, 100, 50, 400, true, true },
	{ "past the ceiling", U32_MAX, 100, 50, 400, true, true },
	{ "no ceiling", U32_MAX, 100, 50, 0, true, false },
	{ "ceiling below the limit", 100, 100, 50, 10, true, true },
	{ "ceiling below the limit, not spent", 99, 100, 50, 10, false,
	  false },
	{ "no budget", U32_MAX, 0, 0, 1, false, false },
	{ "no budget, nothing spent", 0, 0, 0, 0, false, false },
};

KUNIT_ARRAY_PARAM_DESC(t_du_state, t_du_state_cases, desc);

static void ebc_test_du_state(struct kunit *test)
{
	const struct t_du_state *c = test->param_value;
	const struct rk3026_ebc_du_budget budget = {
		.area = c->area, .limit = c->limit, .big_area = c->big_area,
		.ceiling = c->ceiling,
	};

	KUNIT_EXPECT_EQ(test, rk3026_ebc_du_spent(&budget), c->spent);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_du_at_ceiling(&budget),
			c->at_ceiling);
}

/* --- the idle cleanup --- */

/*
 * One look of the refresh thread at the idle cleanup
 * (rk3026_ebc_idle_cleanup_due()): armed by a plane update at @last_plane,
 * a @delay in jiffies, the budget @spent, something else holding the glass
 * (@busy); @due says whether it queues the cleanup, @expired whether the
 * look disarms it. What makes the budget spent and the glass busy is
 * rk3026_ebc_idle_tick()'s, tested by the glue cases.
 */
struct t_idle {
	const char	*desc;
	bool		armed;
	unsigned long	last_plane;
	unsigned long	delay;
	unsigned long	now;
	bool		spent;
	bool		busy;
	bool		due;
	bool		expired;
};

#define T_LONG_AGO	(ULONG_MAX - 99)

static const struct t_idle t_idle_cases[] = {
	{ "due", true, 1000, 200, 1200, true, .due = true, .expired = true },
	{ "one jiffy short", true, 1000, 200, 1199, true },
	{ "long after", true, 1000, 200, 900000, true, .due = true,
	  .expired = true },
	{ "not armed", false, 1000, 200, 1200, true },
	/* Only a spent budget cleans up: a keystroke and a pause do not. */
	{ "not spent", true, 1000, 200, 1200, false, .expired = true },
	{ "busy", true, 1000, 200, 1200, true, true, .expired = true },
	{ "busy, not spent", true, 1000, 200, 1200, false, true,
	  .expired = true },
	/* A delay of 0: no idle cleanup, and the arming is used up at once. */
	{ "off", true, 1000, 0, 1000, true, .expired = true },
	{ "off, later", true, 1000, 0, 5000, true, .expired = true },
	{ "off, not armed", false, 1000, 0, 5000, true },
	/* jiffies wrap between the plane update and the deadline. */
	{ "wrap, before it", true, T_LONG_AGO, 200, ULONG_MAX, true },
	{ "wrap, one jiffy short", true, T_LONG_AGO, 200, 99, true },
	{ "wrap, due", true, T_LONG_AGO, 200, 100, true, .due = true,
	  .expired = true },
	{ "wrap, not armed", false, T_LONG_AGO, 200, 100, true },
};

KUNIT_ARRAY_PARAM_DESC(t_idle, t_idle_cases, desc);

static void ebc_test_idle_cleanup_due(struct kunit *test)
{
	const struct t_idle *c = test->param_value;
	const struct rk3026_ebc_idle idle = {
		.armed = c->armed, .last_plane = c->last_plane,
		.delay = c->delay, .spent = c->spent, .busy = c->busy,
	};
	bool expired = !c->expired;

	KUNIT_EXPECT_EQ(test,
			rk3026_ebc_idle_cleanup_due(&idle, c->now, &expired),
			c->due);
	KUNIT_EXPECT_EQ(test, expired, c->expired);
}

/*
 * The deadline of the refresh thread's wait
 * (rk3026_ebc_thread_deadline()): the rails-off one while the rails are
 * on, the idle one while the cleanup is armed, the retry of a carry while
 * one is armed, the earliest of them, or none (@has false).
 */
struct t_deadline {
	const char	*desc;
	bool		rails_on;
	unsigned long	rails_off_at;
	bool		idle_armed;
	unsigned long	idle_at;
	bool		has;
	unsigned long	deadline;
	bool		retry_armed;
	unsigned long	retry_at;
};

static const struct t_deadline t_deadline_cases[] = {
	{ "neither", false, 1000, false, 2000, false },
	{ "rails", true, 1000, false, 500, true, 1000 },
	{ "idle", false, 500, true, 2000, true, 2000 },
	{ "rails first", true, 1000, true, 2000, true, 1000 },
	{ "idle first", true, 3000, true, 2000, true, 2000 },
	{ "same", true, 2000, true, 2000, true, 2000 },
	/* time_after(), not a plain compare: the later one wrapped. */
	{ "rails wrapped", true, 5, true, ULONG_MAX - 5, true, ULONG_MAX - 5 },
	{ "idle wrapped", true, ULONG_MAX - 5, true, 5, true, ULONG_MAX - 5 },
	/* ..., has, deadline, retry_armed, retry_at */
	{ "retry", false, 100, false, 200, true, 700, true, 700 },
	{ "retry first", true, 1000, true, 2000, true, 700, true, 700 },
	{ "rails before retry", true, 500, false, 0, true, 500, true, 700 },
	{ "idle before retry", false, 0, true, 600, true, 600, true, 700 },
	{ "retry not armed", true, 1000, false, 0, true, 1000, false, 700 },
	{ "retry wrapped", true, ULONG_MAX - 5, false, 0, true, ULONG_MAX - 5,
	  true, 5 },
};

KUNIT_ARRAY_PARAM_DESC(t_deadline, t_deadline_cases, desc);

static void ebc_test_thread_deadline(struct kunit *test)
{
	const struct t_deadline *c = test->param_value;
	const struct rk3026_ebc_pass s = {
		.rails_on	= c->rails_on,
		.rails_off_at	= c->rails_off_at,
		.idle_armed	= c->idle_armed,
		.idle_at	= c->idle_at,
		.retry_armed	= c->retry_armed,
	};
	unsigned long deadline = 12345;

	KUNIT_EXPECT_EQ(test,
			rk3026_ebc_thread_deadline(&s, c->retry_at, &deadline),
			c->has);
	if (c->has)
		KUNIT_EXPECT_EQ(test, deadline, c->deadline);
	else
		KUNIT_EXPECT_EQ(test, deadline, 12345UL);
}

/*
 * The take's quiet gap (rk3026_ebc_take_wait()), times in us: a lone small
 * update with the rails up is taken at once; a burst, a big box, a FULL
 * part or any part with the rails down waits until @quiet_us after the
 * newest update, at most the cap (250 ms) from the later of the first
 * update and the previous finish; an open gap runs to its end; an exempt
 * part (mxcfb, RESET) and a quiet gap or a cap of 0 never wait. A big box
 * is 1000 pixels here. A cap above EBC_TAKE_CAP_MAX_MS (@cap_ms, with
 * @own_cap) is clamped to it, and a quiet gap above the cap to the cap.
 * A carry's retry armed for @retry_us (@retry) ends the gap there, if that
 * comes first: its look finds the retry due and the part exempt.
 */
struct t_take {
	const char	*desc;
	u32		count;
	u32		area;
	bool		full;
	bool		exempt;
	bool		rails_up;
	bool		open;
	u32		quiet_us;
	s64		first_us;
	s64		prev_us;
	s64		last_us;
	s64		now_us;
	bool		wait;
	s64		end_us;
	bool		own_cap;
	u32		cap_ms;
	bool		retry;
	s64		retry_us;
};

static const struct t_take t_take_cases[] = {
	/* count, area, full, exempt, rails_up, open, quiet, first, prev, last, now */
	{ "lone small, rails up: now", 1, 100, false, false, true, false,
	  10000, 0, 0, 0, 0, false },
	{ "burst: the gap after the newest", 2, 100, false, false, true, false,
	  10000, 0, 0, 5000, 6000, true, 15000 },
	{ "burst, quiet reached: now", 2, 100, false, false, true, false,
	  10000, 0, 0, 5000, 15000, false },
	{ "big box", 1, 1000, false, false, true, false,
	  10000, 0, 0, 0, 0, true, 10000 },
	{ "box just under big: now", 1, 999, false, false, true, false,
	  10000, 0, 0, 0, 0, false },
	{ "FULL part", 1, 100, true, false, true, false,
	  10000, 0, 0, 0, 2000, true, 10000 },
	{ "rails down, lone small", 1, 100, false, false, false, false,
	  10000, 0, 0, 0, 0, true, 10000 },
	{ "rails down, quiet reached: now", 1, 100, false, false, false, false,
	  10000, 0, 0, 0, 10000, false },
	{ "open, lone small, rails up", 1, 100, false, false, true, true,
	  10000, 0, 0, 3000, 5000, true, 13000 },
	{ "exempt: now", 3, 100000, true, true, false, false,
	  10000, 0, 0, 0, 0, false },
	{ "exempt while open: now", 3, 100, false, true, true, true,
	  10000, 0, 0, 0, 0, false },
	{ "quiet 0: now", 3, 100000, true, false, false, true,
	  0, 0, 0, 0, 0, false },
	{ "stream: the cap", 5, 100, false, false, true, true,
	  10000, 0, 0, 245000, 249000, true, 250000 },
	{ "stream, cap reached: now", 5, 100, false, false, true, true,
	  10000, 0, 0, 245000, 250000, false },
	{ "cap from the previous finish", 5, 100, false, false, true, true,
	  10000, 0, 100000, 345000, 349000, true, 350000 },
	{ "cap from the first update", 5, 100, false, false, true, true,
	  10000, 100000, 0, 345000, 349000, true, 350000 },
	/* ..., wait, end, own_cap, cap_ms */
	{ "cap 0: now", 2, 100, false, false, false, false,
	  10000, 0, 0, 0, 0, false, 0, true, 0 },
	{ "cap above the max: clamped", 5, 100, false, false, true, true,
	  10000, 0, 0, 995000, 999000, true, 1000000, true, 60000 },
	{ "quiet above the cap: the cap", 2, 100, false, false, true, false,
	  300000, 0, 0, 0, 0, true, 250000 },
	{ "quiet above the cap, cap from the previous finish", 2, 100, false,
	  false, true, false, 300000, 0, 200000, 100000, 300000, true, 350000 },
	{ "previous finish after the quiet gap: now", 2, 100, false, false,
	  true, false, 10000, 0, 50000, 5000, 50000, false },
	/* ..., wait, end, own_cap, cap_ms, retry, retry_us */
	{ "retry before the gap's end: the retry", 2, 100, false, false, true,
	  true, 10000, 0, 0, 5000, 6000, true, 8000, false, 0, true, 8000 },
	{ "retry after the gap's end: the end", 2, 100, false, false, true,
	  true, 10000, 0, 0, 5000, 6000, true, 15000, false, 0, true, 20000 },
	{ "retry before the cap: the retry", 5, 100, false, false, true, true,
	  10000, 0, 0, 245000, 249000, true, 249500, false, 0, true, 249500 },
	{ "retry at the look: now", 2, 100, false, false, true, true,
	  10000, 0, 0, 5000, 6000, false, 0, false, 0, true, 6000 },
};

KUNIT_ARRAY_PARAM_DESC(t_take, t_take_cases, desc);

static void ebc_test_take_wait(struct kunit *test)
{
	const struct t_take *c = test->param_value;
	const struct rk3026_ebc_take t = {
		.first		= c->first_us * NSEC_PER_USEC,
		.last		= c->last_us * NSEC_PER_USEC,
		.prev_done	= c->prev_us * NSEC_PER_USEC,
		.count		= c->count,
		.area		= c->area,
		.big_area	= 1000,
		.quiet_us	= c->quiet_us,
		.cap_ms		= c->own_cap ? c->cap_ms : 250,
		.full		= c->full,
		.exempt		= c->exempt,
		.rails_up	= c->rails_up,
		.open		= c->open,
		.retry_armed	= c->retry,
		.retry_at	= c->retry_us * NSEC_PER_USEC,
	};
	ktime_t end = 12345;

	KUNIT_EXPECT_EQ(test,
			rk3026_ebc_take_wait(&t, c->now_us * NSEC_PER_USEC,
					     &end),
			c->wait);
	if (c->wait)
		KUNIT_EXPECT_EQ(test, end, c->end_us * NSEC_PER_USEC);
	else
		KUNIT_EXPECT_EQ(test, end, 12345);
}

/*
 * The thread's sleep in the gap (rk3026_ebc_gap_timeout()), the gap's end
 * in ns after the wait's start: none once the end has come, as when the
 * rails' ramp outlasted the gap; else the time to it, which the hrtimer
 * adds to the clock (ktime_add_safe()) to expire at the gap's end. A
 * negative timeout would saturate there and never expire.
 */
struct t_gap_timeout {
	const char	*desc;
	s64		at_ns;
	bool		sleeps;
};

static const struct t_gap_timeout t_gap_timeout_cases[] = {
	{ "27 ms past (37 ms ramp, 10 ms gap): no wait", -27000000, false },
	{ "1 ns past: no wait", -1, false },
	{ "at it: no wait", 0, false },
	{ "1 ns ahead: 1 ns", 1, true },
	{ "10 ms ahead", 10000000, true },
	{ "the 1 s cap maximum ahead", 1000000000, true },
	{ "far ahead", S64_MAX / 4, true },
};

KUNIT_ARRAY_PARAM_DESC(t_gap_timeout, t_gap_timeout_cases, desc);

static void ebc_test_gap_timeout(struct kunit *test)
{
	const struct t_gap_timeout *c = test->param_value;
	ktime_t now = ms_to_ktime(5000);
	ktime_t at = ktime_add_ns(now, c->at_ns);
	ktime_t left = 12345;

	KUNIT_EXPECT_EQ(test, rk3026_ebc_gap_timeout(at, now, &left),
			c->sleeps);
	if (!c->sleeps) {
		KUNIT_EXPECT_EQ(test, left, 12345);
		return;
	}
	KUNIT_EXPECT_EQ(test, left, c->at_ns);
	KUNIT_EXPECT_GT(test, left, 0);
	KUNIT_EXPECT_EQ(test, ktime_add_safe(left, now), at);
}

/*
 * The refresh thread's wait condition: between parts it wakes on a part to
 * take, or a carry's retry due; while a gap is open, on an update newer
 * than the one it saw, not on the part pending all along, or on the CRTC
 * going off. While the system suspends, on nothing.
 */
static void ebc_test_thread_woken(struct kunit *test)
{
	struct rk3026_ebc_pending p = {};
	struct rk3026_ebc_set part;
	struct t_ctx *c;
	u32 seen;

	KUNIT_EXPECT_FALSE(test, rk3026_ebc_thread_woken(&p, 0, true, false));
	t_add(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_thread_woken(&p, 0, true, false));

	seen = p.queued_seq;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_work_pending(&p));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_gap_woken(&p, seen, true));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_gap_woken(&p, seen, false));
	t_add(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_gap_woken(&p, seen, true));
	seen = p.queued_seq;
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_gap_woken(&p, seen, true));

	/*
	 * A carry armed for its retry and nothing new: work not finished,
	 * nothing to take, asleep until its time; with the CRTC off at once.
	 */
	c = t_ctx_alloc(test);
	t_add(&c->p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&c->p, &part);
	t_finish_at(c, &part, EBC_OUTCOME_NO_RAILS, 1000);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_work_pending(&c->p));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_pending(&c->p));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_thread_woken(&c->p,
							 ms_to_ktime(1000),
							 true, false));
	KUNIT_EXPECT_FALSE(test,
			   rk3026_ebc_thread_woken(&c->p,
						   ktime_sub_ns(ms_to_ktime(1100),
								1), true, false));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_thread_woken(&c->p,
							ms_to_ktime(1100),
							true, false));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_thread_woken(&c->p,
							ms_to_ktime(1000),
							false, false));

	/*
	 * While the system suspends nothing wakes it: not a part to take, not
	 * a carry's retry that is due, not one the CRTC off makes due.
	 */
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_thread_woken(&p, 0, true, true));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_thread_woken(&c->p,
							 ms_to_ktime(1100),
							 true, true));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_thread_woken(&c->p,
							 ms_to_ktime(1000),
							 false, true));
}

/*
 * One pass of the refresh thread (rk3026_ebc_thread_step()), times in
 * jiffies, the retry in ns after the pass's clock: the rails go off at
 * their deadline (time_after_eq(), across a wrap too), on every pass, but
 * not while a gap is open; the idle cleanup is looked at, at its deadline,
 * but not while a gap is open; the next part is looked at while a gap is
 * open or the wait's condition holds; else the wait times out at the
 * earliest deadline, at least a jiffy ahead: the retry's converted from the
 * clock and rounded up (@timeout_us, usecs_to_jiffies() of it), or none.
 * While the system suspends (@suspended) the rails still go off at their
 * deadline, an open gap does not hold them, and nothing else is looked at:
 * the wait has the rails' deadline alone.
 */
struct t_step {
	const char		*desc;
	unsigned long		now;
	bool			rails_on;
	unsigned long		rails_off_at;
	bool			idle_armed;
	unsigned long		idle_at;
	bool			retry_armed;
	s64			retry_ns;
	bool			gap_open;
	bool			woken;
	enum rk3026_ebc_step	step;
	long			timeout;
	u32			timeout_us;
	bool			suspended;
};

static const struct t_step t_step_cases[] = {
	/* now, rails, at, idle, at, retry, ns, gap, woken, step, timeout, us */
	{ "nothing: no timeout", 1000, false, 0, false, 0, false, 0,
	  false, false, EBC_STEP_WAIT, MAX_SCHEDULE_TIMEOUT },
	{ "rails at their deadline: off", 1000, true, 1000, false, 0, false, 0,
	  false, false, EBC_STEP_RAILS_OFF },
	{ "rails before it: wait to it", 999, true, 1000, false, 0, false, 0,
	  false, false, EBC_STEP_WAIT, 1 },
	{ "rails due, gap open: held, look", 1000, true, 1000, false, 0, false,
	  0, true, false, EBC_STEP_LOOK },
	{ "rails due, part pending: off first", 1000, true, 1000, false, 0,
	  false, 0, false, true, EBC_STEP_RAILS_OFF },
	{ "rails down: none", 1000, false, 900, false, 0, false, 0,
	  false, false, EBC_STEP_WAIT, MAX_SCHEDULE_TIMEOUT },
	{ "rails due across a wrap: off", 5, true, ULONG_MAX - 5, false, 0,
	  false, 0, false, false, EBC_STEP_RAILS_OFF },
	{ "rails ahead across a wrap: wait", ULONG_MAX - 5, true, 5, false, 0,
	  false, 0, false, false, EBC_STEP_WAIT, 11 },
	{ "idle at its deadline: its look", 2000, false, 0, true, 2000, false,
	  0, false, false, EBC_STEP_IDLE },
	{ "idle due, part pending: idle first", 2000, false, 0, true, 2000,
	  false, 0, false, true, EBC_STEP_IDLE },
	{ "idle due, gap open: the part's look", 2000, false, 0, true, 2000,
	  false, 0, true, false, EBC_STEP_LOOK },
	{ "idle before it: wait to it", 1990, false, 0, true, 2000, false, 0,
	  false, false, EBC_STEP_WAIT, 10 },
	{ "rails and idle due: rails first", 2000, true, 2000, true, 1500,
	  false, 0, false, false, EBC_STEP_RAILS_OFF },
	{ "part pending: look", 1000, true, 2000, true, 3000, false, 0,
	  false, true, EBC_STEP_LOOK },
	{ "gap open: look", 1000, false, 0, false, 0, false, 0,
	  true, false, EBC_STEP_LOOK },
	{ "retry due: look", 1000, false, 0, false, 0, true, -1,
	  false, true, EBC_STEP_LOOK },
	{ "retry 100 ms ahead", 1000, false, 0, false, 0, true, 100000000,
	  false, false, EBC_STEP_WAIT, 0, 100000 },
	{ "retry 1 ns ahead: a jiffy", 1000, false, 0, false, 0, true, 1,
	  false, false, EBC_STEP_WAIT, 1 },
	{ "retry 1.5 ms ahead: rounded up", 1000, false, 0, false, 0, true,
	  1500000, false, false, EBC_STEP_WAIT, 0, 1500 },
	{ "retry past, not woken: a jiffy", 1000, false, 0, false, 0, true,
	  -5000000, false, false, EBC_STEP_WAIT, 1 },
	{ "retry 3 s ahead", 1000, false, 0, false, 0, true, 3000000000LL,
	  false, false, EBC_STEP_WAIT, 0, 3000000 },
	{ "retry across a jiffies wrap", ULONG_MAX - 2, false, 0, false, 0,
	  true, 100000000, false, false, EBC_STEP_WAIT, 0, 100000 },
	{ "rails before the retry", 1000, true, 1001, false, 0, true,
	  3000000000LL, false, false, EBC_STEP_WAIT, 1 },
	{ "retry before rails and idle", 1000, true, 1000000, true, 1000000,
	  true, 100000000, false, false, EBC_STEP_WAIT, 0, 100000 },
	{ .desc = "suspended, rails due: off", .now = 1000, .rails_on = true,
	  .rails_off_at = 1000, .woken = true, .suspended = true,
	  .step = EBC_STEP_RAILS_OFF },
	{ .desc = "suspended, rails due, gap open: off", .now = 1000,
	  .rails_on = true, .rails_off_at = 1000, .gap_open = true,
	  .suspended = true, .step = EBC_STEP_RAILS_OFF },
	{ .desc = "suspended, part pending: wait for the rails", .now = 1000,
	  .rails_on = true, .rails_off_at = 1150, .woken = true,
	  .suspended = true, .step = EBC_STEP_WAIT, .timeout = 150 },
	{ .desc = "suspended, gap open: no look", .now = 1000,
	  .gap_open = true, .suspended = true, .step = EBC_STEP_WAIT,
	  .timeout = MAX_SCHEDULE_TIMEOUT },
	{ .desc = "suspended, idle due: no look", .now = 2000,
	  .idle_armed = true, .idle_at = 2000, .suspended = true,
	  .step = EBC_STEP_WAIT, .timeout = MAX_SCHEDULE_TIMEOUT },
	{ .desc = "suspended, retry due: no look", .now = 1000,
	  .retry_armed = true, .retry_ns = -1, .suspended = true,
	  .step = EBC_STEP_WAIT, .timeout = MAX_SCHEDULE_TIMEOUT },
	{ .desc = "suspended, idle and retry ahead: the rails only",
	  .now = 1000, .rails_on = true, .rails_off_at = 1200,
	  .idle_armed = true, .idle_at = 1010, .retry_armed = true,
	  .retry_ns = 1000000, .suspended = true, .step = EBC_STEP_WAIT,
	  .timeout = 200 },
};

KUNIT_ARRAY_PARAM_DESC(t_step, t_step_cases, desc);

static void ebc_test_thread_step(struct kunit *test)
{
	const struct t_step *c = test->param_value;
	ktime_t clock = ms_to_ktime(5000);
	const struct rk3026_ebc_pass pass = {
		.now		= c->now,
		.rails_off_at	= c->rails_off_at,
		.idle_at	= c->idle_at,
		.clock		= clock,
		.retry_at	= ktime_add_ns(clock, c->retry_ns),
		.rails_on	= c->rails_on,
		.idle_armed	= c->idle_armed,
		.retry_armed	= c->retry_armed,
		.gap_open	= c->gap_open,
		.suspended	= c->suspended,
		.woken		= c->woken,
	};
	long timeout = 12345;

	KUNIT_EXPECT_EQ(test, rk3026_ebc_thread_step(&pass, &timeout), c->step);
	if (c->step != EBC_STEP_WAIT)
		return;
	KUNIT_EXPECT_EQ(test, timeout,
			c->timeout_us ? (long)usecs_to_jiffies(c->timeout_us) :
					c->timeout);
	KUNIT_EXPECT_GE(test, timeout, 1);
}

/*
 * The idle cleanup's flag survives what may join the part before it is
 * taken: updates on either side of it, and a carry merged into it. Not
 * driven (no rails, no LUT), the cleanup alone is not carried, and one
 * with an update joined is carried without the flag; after a repair that
 * ran, the whole white panel is carried all the same.
 */
static void ebc_test_pending_cleanup_idle(struct kunit *test)
{
	struct drm_rect r = DRM_RECT_INIT(2, 1, 4, 2);
	struct t_ctx *c = t_ctx_alloc(test);
	struct rk3026_ebc_set part;

	t_add(&c->p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_IDLE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_TRUE(test, part.cleanup_idle);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_FALSE(test, part.auto_ok);
	KUNIT_EXPECT_FALSE(test, part.has_plane);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&part.box, &t_panel));

	/* A plane update after it, and one before it. */
	t_add(&c->p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_IDLE);
	t_add_auto(&c->p, &r, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_TRUE(test, part.cleanup_idle);
	KUNIT_EXPECT_FALSE(test, part.auto_ok);
	KUNIT_EXPECT_TRUE(test, part.has_plane);
	t_add_auto(&c->p, &r, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	t_add(&c->p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_IDLE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_TRUE(test, part.cleanup_idle);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&part.box, &t_panel));

	/* Other parts do not have it. */
	t_add_auto(&c->p, &r, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_FALSE(test, part.cleanup_idle);

	/* Not driven alone: not carried, the next part is its own. */
	t_add(&c->p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_IDLE);
	rk3026_ebc_pending_take(&c->p, &part);
	rk3026_ebc_scan_finish(&c->sh, &c->p, &part, EBC_OUTCOME_NO_RAILS,
			       EBC_REPAIR_NONE, 0, &c->gave_up);
	KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
	t_add_auto(&c->p, &r, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_FALSE(test, part.cleanup_idle);
	KUNIT_EXPECT_TRUE(test, part.auto_ok);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&part.box, &r));

	/* With a plane update joined: carried, no longer the cleanup. */
	t_add(&c->p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_IDLE);
	t_add_auto(&c->p, &r, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&c->p, &part);
	rk3026_ebc_scan_finish(&c->sh, &c->p, &part, EBC_OUTCOME_NO_LUT,
			       EBC_REPAIR_NONE, 0, &c->gave_up);
	KUNIT_EXPECT_TRUE(test, c->p.carry.merged);
	KUNIT_EXPECT_FALSE(test, c->p.carry.cleanup_idle);
	KUNIT_EXPECT_TRUE(test, c->p.carry.has_plane);
	KUNIT_EXPECT_EQ(test, c->p.carry.mode, EBC_MODE_FULL);
	t_add_auto(&c->p, &r, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_FALSE(test, part.cleanup_idle);
	KUNIT_EXPECT_FALSE(test, part.auto_ok);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&part.box, &t_panel));
	KUNIT_EXPECT_FALSE(test, c->p.carry.merged);

	/* Alone after a repair that ran: the white panel is carried. */
	t_add(&c->p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_IDLE);
	rk3026_ebc_pending_take(&c->p, &part);
	rk3026_ebc_scan_finish(&c->sh, &c->p, &part, EBC_OUTCOME_NO_LUT,
			       EBC_REPAIR_DONE, 0, &c->gave_up);
	KUNIT_EXPECT_TRUE(test, c->p.carry.merged);
	KUNIT_EXPECT_FALSE(test, c->p.carry.cleanup_idle);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&c->p.carry.box, &t_panel));
	t_add_auto(&c->p, &r, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_FALSE(test, part.cleanup_idle);
	KUNIT_EXPECT_FALSE(test, c->p.carry.merged);

	/* A plane part carried into the cleanup. */
	t_add_auto(&c->p, &r, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&c->p, &part);
	rk3026_ebc_scan_finish(&c->sh, &c->p, &part, EBC_OUTCOME_NO_LUT,
			       EBC_REPAIR_NONE, 0, &c->gave_up);
	KUNIT_EXPECT_FALSE(test, c->p.carry.cleanup_idle);
	t_add(&c->p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_IDLE);
	rk3026_ebc_pending_take(&c->p, &part);
	KUNIT_EXPECT_TRUE(test, part.cleanup_idle);
	KUNIT_EXPECT_TRUE(test, part.has_plane);
}

static void ebc_test_pending_auto_ok(struct kunit *test)
{
	struct rk3026_ebc_pending p = {};
	struct rk3026_ebc_set part;

	/* A plane update alone; an mxcfb auto send alone. */
	t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_TRUE(test, part.auto_ok);
	KUNIT_EXPECT_TRUE(test, part.has_plane);
	t_add_auto(&p, &t_panel, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_TRUE(test, part.auto_ok);
	KUNIT_EXPECT_FALSE(test, part.has_plane);

	/* AUTO + PART with an explicit DU: PART by rank, no longer auto. */
	t_add_auto(&p, &t_panel, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	t_add(&p, &t_panel, EBC_MODE_BLACK_WHITE, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_PART);
	KUNIT_EXPECT_FALSE(test, part.auto_ok);

	/* Whichever comes first, and whatever follows. */
	t_add(&p, &t_panel, EBC_MODE_BLACK_WHITE, EBC_SOURCE_MXCFB);
	t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_FALSE(test, part.auto_ok);
	KUNIT_EXPECT_TRUE(test, part.has_plane);

	/* A taken set does not leak into the next: it starts auto again. */
	t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_TRUE(test, part.auto_ok);

	/* An identical AUTO + PART part is partial class: it still skips. */
	t_add_auto(&p, &t_panel, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_PART);
	KUNIT_EXPECT_FALSE(test, part.has_mxcfb_full);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_should_skip(&part, false, true));
}

/* The RESET barrier keeps each side's own flag; the RESET has none. */
static void ebc_test_pending_auto_ok_reset(struct kunit *test)
{
	struct rk3026_ebc_pending p = {};
	struct rk3026_ebc_set part;

	t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	t_add(&p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_ENABLE);
	t_add(&p, &t_panel, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	t_add(&p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_MXCFB);
	t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);

	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_TRUE(test, part.auto_ok);
	/* The second RESET absorbed the explicit post; the one after is auto. */
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_RESET);
	KUNIT_EXPECT_FALSE(test, part.auto_ok);
	KUNIT_EXPECT_FALSE(test, part.has_plane);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_TRUE(test, part.auto_ok);
	KUNIT_EXPECT_TRUE(test, part.has_plane);

	/* post -> pre keeps an explicit post's flag too. */
	t_add(&p, &t_panel, EBC_MODE_RESET, EBC_SOURCE_ENABLE);
	t_add(&p, &t_panel, EBC_MODE_PART, EBC_SOURCE_MXCFB);
	t_add_auto(&p, &t_panel, EBC_MODE_FULL, EBC_SOURCE_PLANE);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_RESET);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_FALSE(test, part.auto_ok);
	KUNIT_EXPECT_TRUE(test, part.has_plane);
}

/* A carried part is drawn in its plain mode: the carry clears the flag. */
static void ebc_test_pending_auto_ok_carry(struct kunit *test)
{
	static const enum rk3026_ebc_outcome outcomes[] = {
		EBC_OUTCOME_NO_RAILS, EBC_OUTCOME_NO_LUT,
		EBC_OUTCOME_NO_WAVEFORM,
	};
	static const enum rk3026_ebc_repair repairs[] = {
		EBC_REPAIR_NONE, EBC_REPAIR_DONE,
	};
	struct drm_rect r = DRM_RECT_INIT(2, 1, 4, 2);
	unsigned int i, j;

	for (i = 0; i < ARRAY_SIZE(outcomes); i++)
		for (j = 0; j < ARRAY_SIZE(repairs); j++) {
			struct t_ctx *c = t_ctx_alloc(test);
			struct rk3026_ebc_set part;

			t_add_auto(&c->p, &r, EBC_MODE_FULL, EBC_SOURCE_PLANE);
			rk3026_ebc_pending_take(&c->p, &part);
			KUNIT_EXPECT_TRUE(test, part.auto_ok);
			rk3026_ebc_scan_finish(&c->sh, &c->p, &part,
					       outcomes[i], repairs[j], 0, &c->gave_up);
			KUNIT_EXPECT_TRUE(test, c->p.carry.merged);
			KUNIT_EXPECT_EQ(test, c->p.carry.mode, EBC_MODE_FULL);
			KUNIT_EXPECT_FALSE(test, c->p.carry.auto_ok);
			KUNIT_EXPECT_TRUE(test, c->p.carry.has_plane);
			KUNIT_EXPECT_FALSE(test, c->p.carry.has_mxcfb_full);

			/* The next auto part takes it in, and is not auto. */
			t_add_auto(&c->p, &r, EBC_MODE_FULL, EBC_SOURCE_PLANE);
			rk3026_ebc_pending_take(&c->p, &part);
			KUNIT_EXPECT_FALSE(test, part.auto_ok);
			KUNIT_EXPECT_TRUE(test, part.has_plane);
			KUNIT_EXPECT_FALSE(test, c->p.carry.merged);
		}

	/* A repair cut short carries an mxcfb auto part the same way. */
	for (i = 0; i < 2; i++) {
		struct t_ctx *c = t_ctx_alloc(test);
		struct rk3026_ebc_set part;

		t_add_auto(&c->p, &r, EBC_MODE_PART, EBC_SOURCE_MXCFB);
		rk3026_ebc_pending_take(&c->p, &part);
		rk3026_ebc_scan_finish(&c->sh, &c->p, &part,
				       i ? EBC_OUTCOME_ABANDONED :
				       EBC_OUTCOME_TIMEOUT, EBC_REPAIR_FAILED, 0, &c->gave_up);
		KUNIT_EXPECT_EQ(test, c->p.carry.mode, EBC_MODE_PART);
		KUNIT_EXPECT_FALSE(test, c->p.carry.auto_ok);
		KUNIT_EXPECT_FALSE(test, c->p.carry.has_plane);
	}
}

/* --- the DU cleanup budget --- */

static const enum rk3026_ebc_outcome t_waveform_outcomes[] = {
	EBC_OUTCOME_DRIVEN, EBC_OUTCOME_TIMEOUT, EBC_OUTCOME_ABANDONED,
};

static const enum rk3026_ebc_outcome t_undriven_outcomes[] = {
	EBC_OUTCOME_NONE, EBC_OUTCOME_SKIPPED, EBC_OUTCOME_CRTC_OFF,
	EBC_OUTCOME_NO_WAVEFORM, EBC_OUTCOME_NO_RAILS, EBC_OUTCOME_NO_LUT,
};

static const enum rk3026_ebc_repair t_repairs[] = {
	EBC_REPAIR_NONE, EBC_REPAIR_DONE, EBC_REPAIR_FAILED,
};

static u32 t_du(u32 area, const struct rk3026_ebc_set *part, u32 mode,
		u32 changed_px, enum rk3026_ebc_outcome outcome,
		enum rk3026_ebc_repair repair)
{
	return rk3026_ebc_du_account(area, part, mode, changed_px, outcome,
				     repair);
}

/*
 * A driven plane DU or A2 part grows the budget, a FULL of anyone's clears
 * it.
 */
static void ebc_test_du_account_driven(struct kunit *test)
{
	struct rk3026_ebc_set plane, mxcfb;
	unsigned int i, j;

	t_kind_part(T_PLANE, &plane);
	t_kind_part(T_AUTO_PART, &mxcfb);
	KUNIT_ASSERT_TRUE(test, plane.has_plane);
	KUNIT_ASSERT_FALSE(test, mxcfb.has_plane);

	for (i = 0; i < ARRAY_SIZE(t_waveform_outcomes); i++) {
		enum rk3026_ebc_outcome o = t_waveform_outcomes[i];

		/*
		 * No repair, or one that ran to the end and so cleared the
		 * glass, and the budget with it, before the part.
		 */
		for (j = 0; j < 2; j++) {
			enum rk3026_ebc_repair r = t_repairs[j];
			u32 left = r == EBC_REPAIR_DONE ? 0 : 100;

			KUNIT_EXPECT_EQ(test,
					t_du(100, &plane, T_DU_MODE, 28, o, r),
					left + 28);
			KUNIT_EXPECT_EQ(test,
					t_du(0, &plane, T_DU_MODE, 0, o, r), 0);
			/* An mxcfb DU send never counts. */
			KUNIT_EXPECT_EQ(test,
					t_du(100, &mxcfb, T_DU_MODE, 28, o, r),
					left);
			KUNIT_EXPECT_EQ(test,
					t_du(100, &plane, EBC_MODE_FULL, 28, o,
					     r), 0);
			KUNIT_EXPECT_EQ(test,
					t_du(100, &mxcfb, EBC_MODE_FULL, 28, o,
					     r), 0);
			/*
			 * Other modes neither count nor clear; GL16 full
			 * (OED_PART) is full class, but leaves white over
			 * white alone, so it clears no ghost.
			 */
			KUNIT_EXPECT_EQ(test,
					t_du(100, &plane, EBC_MODE_PART, 28, o,
					     r), left);
			/* A2 leaves a ghost too: it counts as DU does. */
			KUNIT_EXPECT_EQ(test,
					t_du(100, &plane, EBC_MODE_A2, 28, o,
					     r), left + 28);
			KUNIT_EXPECT_EQ(test,
					t_du(100, &mxcfb, EBC_MODE_A2, 28, o,
					     r), left);
			KUNIT_EXPECT_EQ(test,
					t_du(100, &plane, EBC_MODE_OED_PART, 28,
					     o, r), left);
		}
	}

	/* Saturating, never wrapping round to a fresh budget. */
	KUNIT_EXPECT_EQ(test,
			t_du(U32_MAX - 5, &plane, T_DU_MODE, 10,
			     EBC_OUTCOME_DRIVEN, EBC_REPAIR_NONE), U32_MAX);
	KUNIT_EXPECT_EQ(test,
			t_du(U32_MAX - 10, &plane, T_DU_MODE, 10,
			     EBC_OUTCOME_DRIVEN, EBC_REPAIR_NONE), U32_MAX);
	KUNIT_EXPECT_EQ(test,
			t_du(U32_MAX, &plane, T_DU_MODE, 1,
			     EBC_OUTCOME_DRIVEN, EBC_REPAIR_NONE), U32_MAX);
	KUNIT_EXPECT_EQ(test,
			t_du(U32_MAX - 5, &plane, EBC_MODE_A2, 10,
			     EBC_OUTCOME_DRIVEN, EBC_REPAIR_NONE), U32_MAX);
}

/*
 * A grey plane part resolved PART (rk3026_ebc_resolve_mode()) drives only
 * its changed pixels, with a 16-level waveform: it adds no DU ghost and
 * clears none of the others, so the budget stays as it was, spent or not,
 * whatever its waveform's outcome.
 */
static void ebc_test_du_account_part(struct kunit *test)
{
	static const u32 areas[] = { 0, 100, T_W * T_H, U32_MAX };
	static const enum t_kind kinds[] = { T_PLANE, T_PLANE_AUTO_PART };
	struct rk3026_ebc_set part;
	unsigned int i, j, k;

	for (k = 0; k < ARRAY_SIZE(kinds); k++) {
		t_kind_part(kinds[k], &part);
		KUNIT_ASSERT_TRUE(test, part.has_plane);
		KUNIT_ASSERT_TRUE(test, part.auto_ok);
		for (i = 0; i < ARRAY_SIZE(t_waveform_outcomes); i++)
			for (j = 0; j < ARRAY_SIZE(areas); j++)
				KUNIT_EXPECT_EQ(test,
						t_du(areas[j], &part,
						     EBC_MODE_PART, 28,
						     t_waveform_outcomes[i],
						     EBC_REPAIR_NONE),
						areas[j]);
	}
}

/*
 * Nothing driven, nothing counted: a skip, a drop, a carry (no rails, LUT or
 * waveform) or a repair cut short leave the budget as it is, so a part that
 * is carried counts once, when it is drawn. A repair that ran to its end
 * whitened the glass before the part was carried: that clears it.
 */
static void ebc_test_du_account_undriven(struct kunit *test)
{
	static const u32 modes[] = { T_DU_MODE, EBC_MODE_A2, EBC_MODE_FULL };
	struct rk3026_ebc_set plane;
	unsigned int i, j, k;

	t_kind_part(T_PLANE, &plane);
	for (i = 0; i < ARRAY_SIZE(t_undriven_outcomes); i++)
		for (j = 0; j < ARRAY_SIZE(t_repairs); j++)
			for (k = 0; k < ARRAY_SIZE(modes); k++)
				KUNIT_EXPECT_EQ(test,
						t_du(100, &plane, modes[k], 28,
						     t_undriven_outcomes[i],
						     t_repairs[j]),
						t_repairs[j] == EBC_REPAIR_DONE ?
						0 : 100);

	for (i = 0; i < ARRAY_SIZE(t_waveform_outcomes); i++)
		for (k = 0; k < ARRAY_SIZE(modes); k++)
			KUNIT_EXPECT_EQ(test,
					t_du(100, &plane, modes[k], 28,
					     t_waveform_outcomes[i],
					     EBC_REPAIR_FAILED), 100);
}

/* A RESET taken clears the budget, whatever became of it. */
static void ebc_test_du_account_reset(struct kunit *test)
{
	struct rk3026_ebc_set reset;
	unsigned int i, j;

	t_kind_part(T_RESET, &reset);
	for (j = 0; j < ARRAY_SIZE(t_repairs); j++) {
		for (i = 0; i < ARRAY_SIZE(t_waveform_outcomes); i++)
			KUNIT_EXPECT_EQ(test,
					t_du(100, &reset, EBC_MODE_RESET, 0,
					     t_waveform_outcomes[i],
					     t_repairs[j]), 0);
		for (i = 0; i < ARRAY_SIZE(t_undriven_outcomes); i++)
			KUNIT_EXPECT_EQ(test,
					t_du(100, &reset, EBC_MODE_RESET, 0,
					     t_undriven_outcomes[i],
					     t_repairs[j]), 0);
	}
}

static struct kunit_case rk3026_ebc_core_test_cases[] = {
	KUNIT_CASE(ebc_test_dsp_ctrl_mode),
	KUNIT_CASE(ebc_test_us_between),
	KUNIT_CASE(ebc_test_rank_order),
	KUNIT_CASE(ebc_test_rank_unknown),
	KUNIT_CASE(ebc_test_rank_merge),
	KUNIT_CASE(ebc_test_pending_empty),
	KUNIT_CASE(ebc_test_pending_add),
	KUNIT_CASE(ebc_test_pending_temp_newest),
	KUNIT_CASE(ebc_test_pending_mxcfb_full),
	KUNIT_CASE(ebc_test_pending_peek),
	KUNIT_CASE(ebc_test_snapshot_outside_box),
	KUNIT_CASE(ebc_test_snapshot_keeps_equal),
	KUNIT_CASE(ebc_test_snapshot_two_level),
	KUNIT_CASE(ebc_test_snapshot_restores_greys),
	KUNIT_CASE(ebc_test_snapshot_edges),
	KUNIT_CASE(ebc_test_snapshot_moves),
	KUNIT_CASE(ebc_test_snapshot_a2_over_grey),
	KUNIT_CASE(ebc_test_target_differs),
	KUNIT_CASE(ebc_test_skip),
	KUNIT_CASE(ebc_test_scan_mode),
	KUNIT_CASE(ebc_test_drive_knows_glass),
	KUNIT_CASE_PARAM(ebc_test_resolve_mode, t_resolve_gen_params),
	KUNIT_CASE(ebc_test_resolve_repair_full),
	KUNIT_CASE_PARAM(ebc_test_du_cleanup, t_du_cleanup_gen_params),
	KUNIT_CASE_PARAM(ebc_test_du_state, t_du_state_gen_params),
	KUNIT_CASE_PARAM(ebc_test_idle_cleanup_due, t_idle_gen_params),
	KUNIT_CASE_PARAM(ebc_test_thread_deadline, t_deadline_gen_params),
	KUNIT_CASE_PARAM(ebc_test_take_wait, t_take_gen_params),
	KUNIT_CASE_PARAM(ebc_test_gap_timeout, t_gap_timeout_gen_params),
	KUNIT_CASE(ebc_test_thread_woken),
	KUNIT_CASE_PARAM(ebc_test_thread_step, t_step_gen_params),
	KUNIT_CASE(ebc_test_pending_cleanup_idle),
	KUNIT_CASE(ebc_test_pending_auto_ok),
	KUNIT_CASE(ebc_test_pending_auto_ok_reset),
	KUNIT_CASE(ebc_test_pending_auto_ok_carry),
	KUNIT_CASE(ebc_test_du_account_driven),
	KUNIT_CASE(ebc_test_du_account_part),
	KUNIT_CASE(ebc_test_du_account_undriven),
	KUNIT_CASE(ebc_test_du_account_reset),
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
	KUNIT_CASE(ebc_test_retry_next),
	KUNIT_CASE(ebc_test_finish_retry),
	KUNIT_CASE(ebc_test_finish_reset_lost),
	KUNIT_CASE(ebc_test_carry_record),
	KUNIT_CASE(ebc_test_finish_retry_ends),
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
	u32		flags;
	bool		auto_mode;
};

#define T_PARTIAL	MXCFB_UPDATE_MODE_PARTIAL
#define T_FULL		MXCFB_UPDATE_MODE_FULL
#define T_MONO		EPDC_FLAG_FORCE_MONOCHROME

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
	/* AUTO + PARTIAL leaves the mode to the content; AUTO + FULL does not. */
	{ "AUTO partial", MXCFB_WAVEFORM_MODE_AUTO, T_PARTIAL, EBC_MODE_PART,
	  0, true },
	{ "AUTO full", MXCFB_WAVEFORM_MODE_AUTO, T_FULL, EBC_MODE_FULL },
	{ "unknown partial", 99, T_PARTIAL, EBC_MODE_PART },
	{ "unknown full", 99, T_FULL, EBC_MODE_FULL },
	/* Only UPDATE_MODE_FULL is full. */
	{ "GL16 update_mode 2", MXCFB_WAVEFORM_MODE_GL16, 2, EBC_MODE_PART },
	{ "REAGL update_mode 2", MXCFB_WAVEFORM_MODE_REAGL, 2, EBC_MODE_PART },
	{ "GC16 update_mode 2", MXCFB_WAVEFORM_MODE_GC16, 2, EBC_MODE_PART },
	{ "AUTO update_mode 2", MXCFB_WAVEFORM_MODE_AUTO, 2, EBC_MODE_PART, 0,
	  true },
	/* AUTO + FORCE_MONOCHROME: monochrome first, so DU, as asked. */
	{ "AUTO partial mono", MXCFB_WAVEFORM_MODE_AUTO, T_PARTIAL,
	  EBC_MODE_BLACK_WHITE, T_MONO },
	{ "AUTO full mono", MXCFB_WAVEFORM_MODE_AUTO, T_FULL,
	  EBC_MODE_BLACK_WHITE, T_MONO },
	{ "AUTO partial all flags", MXCFB_WAVEFORM_MODE_AUTO, T_PARTIAL,
	  EBC_MODE_BLACK_WHITE, ~0u },
	/* Only the FORCE_MONOCHROME bit is read. */
	{ "AUTO partial other flags", MXCFB_WAVEFORM_MODE_AUTO, T_PARTIAL,
	  EBC_MODE_PART, ~T_MONO, true },
	{ "AUTO full other flags", MXCFB_WAVEFORM_MODE_AUTO, T_FULL,
	  EBC_MODE_FULL, ~T_MONO },
	/* With a named waveform the flag changes nothing. */
	{ "INIT mono", MXCFB_WAVEFORM_MODE_INIT, T_PARTIAL, EBC_MODE_RESET,
	  T_MONO },
	{ "DU partial mono", MXCFB_WAVEFORM_MODE_DU, T_PARTIAL,
	  EBC_MODE_BLACK_WHITE, T_MONO },
	{ "A2 partial mono", MXCFB_WAVEFORM_MODE_A2, T_PARTIAL, EBC_MODE_A2,
	  T_MONO },
	{ "GC16 partial mono", MXCFB_WAVEFORM_MODE_GC16, T_PARTIAL,
	  EBC_MODE_PART, T_MONO },
	{ "GC16 full mono", MXCFB_WAVEFORM_MODE_GC16, T_FULL, EBC_MODE_FULL,
	  T_MONO },
	{ "GL16 full mono", MXCFB_WAVEFORM_MODE_GL16, T_FULL,
	  EBC_MODE_OED_PART, T_MONO },
	{ "unknown partial mono", 99, T_PARTIAL, EBC_MODE_PART, T_MONO },
};

KUNIT_ARRAY_PARAM_DESC(t_mxcfb, t_mxcfb_cases, desc);

static void ebc_test_mxcfb_map(struct kunit *test)
{
	const struct t_mxcfb *c = test->param_value;
	bool auto_mode = !c->auto_mode;

	KUNIT_EXPECT_EQ(test,
			rk3026_ebc_mxcfb_to_ebc_mode(c->waveform,
						     c->update_mode, c->flags,
						     &auto_mode),
			c->mode);
	KUNIT_EXPECT_EQ(test, auto_mode, c->auto_mode);
}

/*
 * The 16-level waveforms' full updates are full class, so they are exempt
 * from the identical-image skip; their partial updates are not. AUTO +
 * PARTIAL is partial class, as before: it is PART, now with auto_mode, and
 * resolves to DU or PART, both partial. AUTO + FULL stays a FULL without
 * auto_mode, so the policy never takes its flash or its skip exemption.
 */
static void ebc_test_mxcfb_full_class(struct kunit *test)
{
	static const u32 waveforms[] = {
		MXCFB_WAVEFORM_MODE_GC16, MXCFB_WAVEFORM_MODE_GL16,
		MXCFB_WAVEFORM_MODE_REAGL, MXCFB_WAVEFORM_MODE_AUTO,
	};
	unsigned int i;
	bool auto_mode;
	u32 mode;

	for (i = 0; i < ARRAY_SIZE(waveforms); i++) {
		mode = rk3026_ebc_mxcfb_to_ebc_mode(waveforms[i], T_FULL, 0,
						    &auto_mode);
		KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(mode),
				DSP_DISPLAY_MODE);
		KUNIT_EXPECT_TRUE(test, t_full_flag(mode, EBC_SOURCE_MXCFB));
		KUNIT_EXPECT_FALSE(test, auto_mode);
		mode = rk3026_ebc_mxcfb_to_ebc_mode(waveforms[i], T_PARTIAL, 0,
						    &auto_mode);
		KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(mode),
				DSP_UPDATE_MODE | DSP_DISPLAY_MODE);
		KUNIT_EXPECT_FALSE(test, t_full_flag(mode, EBC_SOURCE_MXCFB));
		KUNIT_EXPECT_EQ(test, auto_mode,
				waveforms[i] == MXCFB_WAVEFORM_MODE_AUTO);
	}

	/*
	 * A monochrome AUTO is DU, partial class whatever its update_mode:
	 * an identical one is skipped, it has nothing to draw.
	 */
	mode = rk3026_ebc_mxcfb_to_ebc_mode(MXCFB_WAVEFORM_MODE_AUTO, T_FULL,
					    T_MONO, &auto_mode);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(mode),
			DSP_UPDATE_MODE | DSP_DISPLAY_MODE);
	KUNIT_EXPECT_FALSE(test, t_full_flag(mode, EBC_SOURCE_MXCFB));
}

/* A mapped mxcfb send through the pending set and the mode policy. */
static u32 t_mxcfb_resolve(struct kunit *test, u32 waveform, u32 update_mode,
			   u32 flags, bool two_level,
			   enum rk3026_ebc_reason *reason)
{
	/*
	 * Past the ceiling, and the part big: a plane part would clean up.
	 * A2 on for any part: a two-level plane part would go A2.
	 */
	static const struct rk3026_ebc_du_budget budget = {
		.area = U32_MAX, .limit = 1, .big_area = 0, .ceiling = 1,
	};
	static const struct rk3026_ebc_a2 a2 = { .max_area = U32_MAX };
	struct rk3026_ebc_pending p = {};
	struct rk3026_ebc_update upd;
	struct rk3026_ebc_set part;
	bool auto_mode;
	u32 seq;

	upd = t_upd(&t_panel,
		    rk3026_ebc_mxcfb_to_ebc_mode(waveform, update_mode, flags,
						 &auto_mode),
		    0, EBC_SOURCE_MXCFB);
	upd.auto_mode = auto_mode;
	rk3026_ebc_pending_add(&p, &upd, &seq);
	rk3026_ebc_pending_take(&p, &part);
	KUNIT_EXPECT_EQ(test, part.auto_ok, auto_mode);
	KUNIT_EXPECT_FALSE(test, part.has_plane);
	return rk3026_ebc_resolve_mode(&part,
				       rk3026_ebc_scan_mode(&part, false),
				       two_level, U32_MAX, EBC_REPAIR_NOT_DUE,
				       &budget, &a2, reason);
}

/*
 * AUTO + PARTIAL goes DU when the content is two-level, else PART, even
 * with the plane's DU budget past its ceiling and the part big, and with
 * A2 on for any part; AUTO + FULL and a named waveform keep their mode.
 */
static void ebc_test_mxcfb_auto_resolve(struct kunit *test)
{
	enum rk3026_ebc_reason reason;

	KUNIT_EXPECT_EQ(test,
			t_mxcfb_resolve(test, MXCFB_WAVEFORM_MODE_AUTO,
					T_PARTIAL, 0, true, &reason),
			EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_EQ(test,
			t_mxcfb_resolve(test, MXCFB_WAVEFORM_MODE_AUTO,
					T_PARTIAL, 0, false, &reason),
			EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, reason, EBC_REASON_EXPLICIT);
	KUNIT_EXPECT_EQ(test,
			t_mxcfb_resolve(test, MXCFB_WAVEFORM_MODE_AUTO,
					T_FULL, 0, true, &reason),
			EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, reason, EBC_REASON_EXPLICIT);
	KUNIT_EXPECT_EQ(test,
			t_mxcfb_resolve(test, MXCFB_WAVEFORM_MODE_AUTO,
					T_PARTIAL, T_MONO, false, &reason),
			EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, reason, EBC_REASON_EXPLICIT);
	KUNIT_EXPECT_EQ(test,
			t_mxcfb_resolve(test, MXCFB_WAVEFORM_MODE_GC16,
					T_PARTIAL, 0, true, &reason),
			EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, reason, EBC_REASON_EXPLICIT);
}

static struct kunit_case rk3026_ebc_mxcfb_test_cases[] = {
	KUNIT_CASE_PARAM(ebc_test_mxcfb_map, t_mxcfb_gen_params),
	KUNIT_CASE(ebc_test_mxcfb_full_class),
	KUNIT_CASE(ebc_test_mxcfb_auto_resolve),
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

/* The temperature boundary table of a mode's offset table (rkf_modes[]). */
static u32 t_rkf_temps(u32 table)
{
	switch (table) {
	case T_RKF_RESET:
		return 0x090;
	case T_RKF_GRAY16:
		return 0x0d0;
	case T_RKF_GRAYA2:
		return 0x110;
	case T_RKF_MODE6:
		return 0x150;
	default:
		return 0x190;
	}
}

/*
 * Segments laid out one after another from the payload start, the last one
 * @cut bytes short of its stream: the container ends there. With @hot, the
 * segment of that offset table is there only from @from degrees up: its
 * boundary table splits the temperatures at @from, and the slot below is
 * empty, as a waveform without that mode at those temperatures has it.
 */
static struct ebc_waveform *t_rkf_wf_hot(struct kunit *test,
					 struct device *dev,
					 const struct t_rkf_seg *segs,
					 unsigned int n, size_t cut, u32 hot,
					 u8 from)
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
	if (hot) {
		put_unaligned_le32(get_unaligned_le32(buf + hot), buf + hot + 4);
		put_unaligned_le32(0, buf + hot);
		buf[t_rkf_temps(hot) + 1] = from;
	}
	t_rkf_seal(buf, size);

	wf = rk3026_ebc_waveform_parse(dev, buf, size + 4);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, wf);
	return wf;
}

static struct ebc_waveform *t_rkf_wf(struct kunit *test, struct device *dev,
				     const struct t_rkf_seg *segs,
				     unsigned int n, size_t cut)
{
	return t_rkf_wf_hot(test, dev, segs, n, cut, 0, 0);
}

/*
 * A segment only from 20 degrees up: a LUT at 20 and above, none below;
 * the other modes at every temperature.
 */
static void ebc_test_rkf_hot(struct kunit *test)
{
	static const struct t_rkf_seg segs[] = {
		{ T_RKF_RESET, 5 }, { T_RKF_GRAY16, 1 },
	};
	struct device *dev = t_rkf_dev(test);
	struct ebc_waveform *wf = t_rkf_wf_hot(test, dev, segs,
					       ARRAY_SIZE(segs), 0,
					       T_RKF_RESET, 20);
	u32 frames = 0;

	KUNIT_EXPECT_NULL(test, rk3026_ebc_waveform_lut(wf, EBC_MODE_RESET, 5,
							&frames));
	KUNIT_EXPECT_NULL(test, rk3026_ebc_waveform_lut(wf, EBC_MODE_RESET, 19,
							&frames));
	KUNIT_EXPECT_NOT_NULL(test, rk3026_ebc_waveform_lut(wf, EBC_MODE_RESET,
							    20, &frames));
	KUNIT_EXPECT_EQ(test, frames, 5);
	KUNIT_EXPECT_NOT_NULL(test, rk3026_ebc_waveform_lut(wf, EBC_MODE_FULL, 5,
							    &frames));
	KUNIT_EXPECT_NOT_NULL(test, rk3026_ebc_waveform_lut(wf, EBC_MODE_FULL,
							    25, &frames));
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
	KUNIT_CASE(ebc_test_rkf_hot),
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
	/* As probe sets them: one panel's area, half of it, four panels. */
	ebc->du_cleanup_area = T_W * T_H;
	ebc->du_cleanup_big_area = T_W * T_H / 2;
	ebc->du_cleanup_ceiling = 4 * T_W * T_H;
	ebc->du_cleanup_idle_ms = EBC_DU_CLEANUP_IDLE_MS;
	/* A2 off for plane parts. */
	ebc->a2_max_area = 0;
	/*
	 * plane_resync stays false: after probe it is true (the probe RESET
	 * is a non-plane update), but here the target and the framebuffers
	 * start in step, so a boxed plane update is taken as it is.
	 */
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
	u32 seq, mseq;

	ebc->shadows.prev_unknown = true;
	seq = t_send(ebc, &a, EBC_MODE_FULL, 2, EBC_SOURCE_MXCFB);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_WAVEFORM);
	KUNIT_EXPECT_FALSE(test, last.repaired);
	/* Loaded once, at probe: given up at once, the marker done. */
	KUNIT_EXPECT_TRUE(test, last.gave_up);
	KUNIT_EXPECT_EQ(test, last.tries, 0);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_marker_seq_lookup(&ebc->markers, 2,
							     &mseq));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, mseq));
	KUNIT_EXPECT_FALSE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_EQ(test, ebc->stats.carry_failed, 1);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&ebc->pending.carry.box, &a));
	KUNIT_EXPECT_TRUE(test, ebc->shadows.prev_unknown);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));

	t_send(ebc, &b, EBC_MODE_A2, 0, EBC_SOURCE_PLANE);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.clip.x1, 0);
	KUNIT_EXPECT_EQ(test, last.clip.x2, 10);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, ebc->stats.dropped, 2);
	KUNIT_EXPECT_EQ(test, ebc->stats.carry_failed, 2);
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

/*
 * Four parts, each a setup and eight chunks at their budget, and the
 * retries of a carried one: a part more and the schedule's span.
 */
static void ebc_test_glue_wait_timeout(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_ebc_alloc(test);

	/* The fallback dot clock on a 16x6 panel: every budget at its floor. */
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_timeout_ms(ebc),
			5 * (1000 + 8 * 5000) + EBC_RETRY_SPAN_MS +
			EBC_TAKE_CAP_MAX_MS);

	/*
	 * 1 kHz, 4 x 6 clocks a frame: 24 ms a frame, a 64-frame budget of
	 * 64 x 24 x 4 = 6144 ms, above the floor.
	 */
	ebc->timing.pixelclock.typ = 1000;
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_timeout_ms(ebc),
			5 * (1000 + 8 * 6144) + EBC_RETRY_SPAN_MS +
			EBC_TAKE_CAP_MAX_MS);
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
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4,
							 &t_panel));
	t_expect_white(test, ebc->next_shadow);
	KUNIT_EXPECT_EQ(test, ebc->stats.updates, 0);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));

	/* None: the whole framebuffer (black), the whole panel, FULL. */
	ebc->mxcfb_active = false;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4,
							&t_panel));
	for (i = 0; i < T_SIZE; i++)
		KUNIT_EXPECT_EQ(test, ebc->next_shadow[i], 0);
	t_expect_guard(test, ebc->next_shadow);
	KUNIT_EXPECT_EQ(test, ebc->stats.updates, 1);
	KUNIT_EXPECT_EQ(test, ebc->pending.pre.merged, 1);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&ebc->pending.pre.box, &t_panel));
	KUNIT_EXPECT_EQ(test, ebc->pending.pre.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, ebc->pending.pre.source, EBC_SOURCE_PLANE);
	/* An auto update: the content may pick DU. */
	KUNIT_EXPECT_TRUE(test, ebc->pending.pre.auto_ok);
	KUNIT_EXPECT_TRUE(test, ebc->pending.pre.has_plane);
}

/*
 * The mode policy end to end, on a white glass: a plane update whose changes
 * are all black or white resolves DU, one with a grey PART, one that
 * changes nothing is skipped in its own mode. With the rails counted as up
 * and a waveform that has only a RESET, each stops at its missing LUT.
 */
static u32 *t_plane_fb(struct kunit *test, u32 xrgb)
{
	u32 *fb = kunit_kcalloc(test, T_W * T_H, sizeof(*fb), GFP_KERNEL);
	int i;

	KUNIT_ASSERT_NOT_NULL(test, fb);
	for (i = 0; i < T_W * T_H; i++)
		fb[i] = xrgb;
	return fb;
}

/* Fill the box @x1,@y1 - @x2,@y2 (exclusive) of a plane framebuffer. */
static void t_fb_box(u32 *fb, int x1, int y1, int x2, int y2, u32 xrgb)
{
	int x, y;

	for (y = y1; y < y2; y++)
		for (x = x1; x < x2; x++)
			fb[y * T_W + x] = xrgb;
}

static struct rk3026_ebc *t_policy_ebc(struct kunit *test)
{
	static const struct t_rkf_seg seg = { T_RKF_RESET, 5 };
	struct rk3026_ebc *ebc = t_ebc_alloc(test);

	ebc->waveform = t_rkf_wf(test, ebc->dev, &seg, 1, 0);
	ebc->next_virt = t_img_alloc(test, 15);
	ebc->epd_on = true;
	return ebc;
}

static void ebc_test_glue_auto_du(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_last last;
	struct rk3026_ebc_set *carry = &ebc->pending.carry;

	/*
	 * Black over white: two-level, DU. A budget of two panels, so the
	 * whole panel does not spend it and takes no cleanup.
	 */
	ebc->du_cleanup_area = 2 * T_W * T_H;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_EQ(test, ebc->last.reason, EBC_REASON_AUTO_DU);
	/* The snapshot as FULL built it: every pixel black, no threshold. */
	KUNIT_EXPECT_EQ(test, ebc->shadows.scan[0], 0);
	KUNIT_EXPECT_EQ(test, ebc->shadows.scan[T_SIZE - 1], 0);
	t_expect_guard(test, ebc->shadows.scan);

	/* Carried in its plain mode, the auto flag cleared. */
	KUNIT_EXPECT_TRUE(test, carry->merged);
	KUNIT_EXPECT_EQ(test, carry->mode, EBC_MODE_FULL);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&carry->box, &t_panel));
	KUNIT_EXPECT_FALSE(test, carry->auto_ok);
	KUNIT_EXPECT_TRUE(test, carry->has_plane);
	KUNIT_EXPECT_FALSE(test, carry->has_mxcfb_full);
	KUNIT_EXPECT_FALSE(test, ebc->shadows.prev_unknown);

	/* The next plane part takes the carry in: FULL, as it was asked. */
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_EXPLICIT);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_FALSE(test, carry->auto_ok);
	KUNIT_EXPECT_EQ(test, ebc->stats.dropped, 2);
}

static void ebc_test_glue_auto_grey(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	/* Luma 128: grey 8, not two-level. */
	u32 *grey = t_plane_fb(test, 0x808080);
	struct rk3026_ebc_last last;

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, grey, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_PART);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_EQ(test, ebc->shadows.scan[0], 0x88);
	/* No PART LUT: carried in its plain mode, drawn FULL with the next. */
	KUNIT_EXPECT_TRUE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_EQ(test, ebc->pending.carry.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.auto_ok);
}

/*
 * White over the white glass: skipped, in the part's own mode. A part that
 * changes nothing is never two-level, so the policy would keep its mode
 * too; the checks pin the record of a skip, not the order of skip and
 * policy.
 */
static void ebc_test_glue_auto_skipped(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	u32 *white = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_SKIPPED);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_EXPLICIT);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
}

/*
 * A send that names its mode is driven in it: an mxcfb FULL over the white
 * glass whose pixels are all black stays FULL.
 */
static void ebc_test_glue_explicit_full(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	struct rk3026_ebc_last last;

	t_img_fill(ebc->next_shadow, 0);
	t_send(ebc, &t_panel, EBC_MODE_FULL, 1, EBC_SOURCE_MXCFB);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_EXPLICIT);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.has_plane);
	KUNIT_EXPECT_TRUE(test, ebc->pending.carry.has_mxcfb_full);
}

/*
 * An explicit A2 over a grey glass: the snapshot raises it to DU, and the
 * policy leaves that as it is, reason explicit, though the content is
 * two-level. It stops at the missing DU LUT and is carried as the A2 it
 * was asked as.
 */
static void ebc_test_glue_a2_over_grey(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	struct rk3026_ebc_last last;

	t_img_fill(ebc->shadows.prev, 8);
	t_img_fill(ebc->next_shadow, 0);
	t_send(ebc, &t_panel, EBC_MODE_A2, 0, EBC_SOURCE_MXCFB);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_A2);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_EXPLICIT);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_EQ(test, ebc->shadows.scan[0], 0);
	KUNIT_EXPECT_TRUE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_EQ(test, ebc->pending.carry.mode, EBC_MODE_A2);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.auto_ok);
}

/*
 * The DU budget spent: the black plane update over the white panel, a big
 * part, that would be DU is FULL, reason cleanup_big. It stops at the
 * missing LUT, so the budget stays as it was and the part is carried.
 */
static void ebc_test_glue_du_cleanup(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_last last;

	ebc->du_area = ebc->du_cleanup_area;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_CLEANUP_BIG);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_EQ(test, ebc->du_area, ebc->du_cleanup_area);
	KUNIT_EXPECT_TRUE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_EQ(test, ebc->pending.carry.mode, EBC_MODE_FULL);
}

/*
 * A black box of 4 x 2 pixels over white, a small plane update, with @area
 * spent of @limit: DU, though it would spend the budget.
 */
static void t_du_budget_left(struct kunit *test, u32 limit, u32 area)
{
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	struct drm_rect r = DRM_RECT_INIT(4, 2, 4, 2);
	u32 *box = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;

	t_fb_box(box, r.x1, r.y1, r.x2, r.y2, 0);
	ebc->du_cleanup_area = limit;
	ebc->du_area = area;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, box, T_W * 4,
							&r));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	/* Not driven: not counted. */
	KUNIT_EXPECT_EQ(test, ebc->du_area, area);
}

/* One pixel short of the budget. */
static void ebc_test_glue_du_budget_left(struct kunit *test)
{
	t_du_budget_left(test, T_W * T_H, T_W * T_H - 1);
}

/* No budget at all (debugfs du_cleanup_area 0): never a cleanup. */
static void ebc_test_glue_du_budget_off(struct kunit *test)
{
	t_du_budget_left(test, 0, U32_MAX);
}

/*
 * The fbdev duplicate: identical, skipped in its own mode before a cleanup
 * is chosen, so the budget neither counts it nor clears.
 */
static void ebc_test_glue_du_skipped(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	u32 *white = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;

	ebc->du_area = ebc->du_cleanup_area;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_SKIPPED);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_EXPLICIT);
	KUNIT_EXPECT_EQ(test, ebc->du_area, ebc->du_cleanup_area);
}

/* A RESET taken clears the budget, here one dropped with the CRTC off. */
static void ebc_test_glue_du_reset(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	struct rk3026_ebc_last last;

	ebc->du_area = 50;
	t_send(ebc, &t_panel, EBC_MODE_FULL, 0, EBC_SOURCE_MXCFB);
	ebc->crtc_on = false;
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_CRTC_OFF);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 50);

	t_send(ebc, &t_panel, EBC_MODE_RESET, 0, EBC_SOURCE_ENABLE);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_CRTC_OFF);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 0);
}

/*
 * The waveforms the static stub of rk3026_ebc_drive() was asked for, and
 * what the stub of rk3026_ebc_raise_rails() (t_rails()) does: @rails counts
 * its calls, @rails_ret is what they return, with @rails_fb the first one
 * sends a plane update of @rails_box from it, as a sender would while the
 * rails come up, with @rails_reset a RESET, as a CRTC enable would, with
 * @rails_suspend it begins a system suspend (rk3026_ebc_sleep_begin()), and
 * rails that come up move the fake clock (t_clock(), @now) by @ramp; drive
 * k moves it by @drive_time[k]. @temp is what the sensor reads, with
 * t_temp_ebc().
 */
struct t_drives {
	unsigned int	n;
	u32		mode[8];
	u32		seq[8];
	u32		setup_us[8];
	unsigned int	rails;
	int		rails_ret;
	const u32	*rails_fb;
	struct drm_rect	rails_box;
	bool		rails_reset;
	bool		rails_suspend;
	ktime_t		now;
	ktime_t		ramp;
	ktime_t		drive_time[8];
	int		temp;
};

/*
 * rk3026_ebc_drive() without the controller: every frame driven, as the
 * real one records it, and the mode noted in the test's struct t_drives,
 * with the part's seq and the time from its take that the real one traces
 * at scan_start (rk3026_ebc_setup_us()); the waveform then takes its
 * @drive_time on the fake clock. The mark follows the real one's rule:
 * cleared by a drive that begins with the glass known or makes it known.
 */
static int t_drive_ok(struct rk3026_ebc *ebc, const u8 *lut, u32 frames,
		      u32 mode, u32 lut_mode, struct rk3026_ebc_last *last)
{
	bool unknown = READ_ONCE(ebc->shadows.prev_unknown);
	struct t_drives *d = kunit_get_current_test()->priv;

	if (d->n < ARRAY_SIZE(d->mode)) {
		d->mode[d->n] = mode;
		d->seq[d->n] = last->seq;
		d->setup_us[d->n] = rk3026_ebc_setup_us(ebc, last);
		d->now = ktime_add(d->now, d->drive_time[d->n]);
	}
	d->n++;
	last->scans++;
	last->lut_mode = lut_mode;
	last->frames = frames;
	last->driven = frames;
	last->outcome = EBC_OUTCOME_DRIVEN;
	if (!unknown || rk3026_ebc_drive_knows_glass(mode, lut_mode))
		WRITE_ONCE(ebc->shadows.prev_unknown, false);
	return 0;
}

/*
 * rk3026_ebc_drive() cut short in its first chunk, as the real one ends on
 * a frame timeout: nothing driven, the mark set, the mode noted.
 */
static int t_drive_timeout(struct rk3026_ebc *ebc, const u8 *lut,
			   u32 frames, u32 mode, u32 lut_mode,
			   struct rk3026_ebc_last *last)
{
	struct t_drives *d = kunit_get_current_test()->priv;

	if (d->n < ARRAY_SIZE(d->mode)) {
		d->mode[d->n] = mode;
		d->seq[d->n] = last->seq;
		d->setup_us[d->n] = rk3026_ebc_setup_us(ebc, last);
		d->now = ktime_add(d->now, d->drive_time[d->n]);
	}
	d->n++;
	WRITE_ONCE(ebc->shadows.prev_unknown, true);
	last->scans++;
	last->lut_mode = lut_mode;
	last->frames = frames;
	last->timeouts++;
	last->outcome = EBC_OUTCOME_TIMEOUT;
	return -ETIMEDOUT;
}

/*
 * The policy fixture with rk3026_ebc_drive() stubbed by t_drive_ok() and,
 * with @luts, a waveform that also has gray16 and gray2 LUTs: FULL, PART
 * and DU parts are driven.
 */
static struct rk3026_ebc *t_driven_ebc(struct kunit *test, struct t_drives *d,
				       bool luts)
{
	static const struct t_rkf_seg segs[] = {
		{ T_RKF_RESET, 5 }, { T_RKF_GRAY16, 1 }, { T_RKF_GRAY2, 1 },
	};
	struct rk3026_ebc *ebc = t_policy_ebc(test);

	if (luts)
		ebc->waveform = t_rkf_wf(test, ebc->dev, segs,
					 ARRAY_SIZE(segs), 0);
	memset(d, 0, sizeof(*d));
	test->priv = d;
	kunit_activate_static_stub(test, rk3026_ebc_drive, t_drive_ok);
	return ebc;
}

/*
 * The budget follows driven parts with the counts of their own snapshots.
 * With a budget of two panels, a black plane update over the white glass
 * is driven DU and adds every pixel of the panel; the white one after it,
 * two-level too and big, would spend the budget, so it is driven FULL for
 * the cleanup and clears it; then a black box of 4 x 2 pixels on white is
 * driven DU and adds its 8 changed pixels (not the mode, not the panel's
 * area).
 */
static void ebc_test_glue_du_driven(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	u32 *black = t_plane_fb(test, 0);
	u32 *white = t_plane_fb(test, 0xffffff);
	u32 *box = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;

	t_fb_box(box, 4, 2, 8, 4, 0);

	ebc->du_cleanup_area = 2 * T_W * T_H;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_EQ(test, ebc->du_area, T_W * T_H);
	KUNIT_EXPECT_EQ(test, ebc->shadows.prev[0], 0);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_CLEANUP_BIG);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 0);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, box, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 8);

	KUNIT_EXPECT_EQ(test, d.n, 3);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, d.mode[2], EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, ebc->stats.scans, 3);
}

/*
 * A grey box of 4 x 2 pixels on the white glass is driven PART and leaves
 * the budget as it was; with the budget spent, a grey plane update over the
 * whole panel, a big part, is driven FULL for the cleanup and clears it.
 */
static void ebc_test_glue_part_driven(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	u32 *box = t_plane_fb(test, 0xffffff);
	u32 *grey = t_plane_fb(test, 0x404040);
	struct rk3026_ebc_last last;

	t_fb_box(box, 4, 2, 8, 4, 0x808080);

	ebc->du_area = 50;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, box, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, last.lut_mode, EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_PART);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 50);
	KUNIT_EXPECT_EQ(test, t_px(ebc->shadows.prev, 4, 2), 8);
	KUNIT_EXPECT_EQ(test, t_px(ebc->shadows.prev, 3, 2), 15);

	ebc->du_area = ebc->du_cleanup_area;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, grey, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_CLEANUP_BIG);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 0);

	KUNIT_EXPECT_EQ(test, d.n, 2);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_FULL);
}

/*
 * The budget spent, the cleanup waits for a big part. A black box of
 * 4 x 2 pixels on the white glass is small: driven DU, it adds its 8
 * pixels. A grey box of 2 x 2 beside it is small too: driven PART, it
 * leaves the budget. A grey plane update over the whole panel changes
 * every pixel, at least half of it: driven FULL for the cleanup, reason
 * cleanup_big, and the budget is cleared. Spent again, a change of one
 * pixel in a damage box of the whole panel, a far pointer jump, is small:
 * big counts changed pixels, not the box, so it is driven DU and adds 1.
 */
static void ebc_test_glue_du_big(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	u32 *box = t_plane_fb(test, 0xffffff);
	u32 *boxes = t_plane_fb(test, 0xffffff);
	u32 *grey = t_plane_fb(test, 0x404040);
	u32 *dot = t_plane_fb(test, 0x404040);
	struct drm_rect r1 = DRM_RECT_INIT(4, 2, 4, 2);
	struct drm_rect r2 = DRM_RECT_INIT(4, 2, 8, 4);
	struct rk3026_ebc_last last;

	t_fb_box(box, 4, 2, 8, 4, 0);
	t_fb_box(boxes, 4, 2, 8, 4, 0);
	t_fb_box(boxes, 10, 4, 12, 6, 0x808080);
	t_fb_box(dot, 0, 0, 1, 1, 0);
	KUNIT_ASSERT_GE(test, T_W * T_H, ebc->du_cleanup_big_area);
	KUNIT_ASSERT_LT(test, 1, ebc->du_cleanup_big_area);

	ebc->du_area = ebc->du_cleanup_area;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, box, T_W * 4,
							&r1));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_EQ(test, ebc->du_area, ebc->du_cleanup_area + 8);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, boxes, T_W * 4,
							&r2));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_PART);
	KUNIT_EXPECT_EQ(test, ebc->du_area, ebc->du_cleanup_area + 8);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, grey, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_CLEANUP_BIG);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 0);

	ebc->du_area = ebc->du_cleanup_area;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, dot, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_EQ(test, ebc->du_area, ebc->du_cleanup_area + 1);

	KUNIT_EXPECT_EQ(test, d.n, 4);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, d.mode[2], EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, d.mode[3], EBC_MODE_BLACK_WHITE);
}

/*
 * One pixel short of the ceiling, a small black box is driven DU and
 * takes the budget past it; the next small box is driven FULL for the
 * cleanup, reason cleanup_ceiling, and the budget is cleared.
 */
static void ebc_test_glue_du_ceiling(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	u32 *box = t_plane_fb(test, 0xffffff);
	u32 *boxes = t_plane_fb(test, 0xffffff);
	struct drm_rect r1 = DRM_RECT_INIT(4, 2, 4, 2);
	struct drm_rect r2 = DRM_RECT_INIT(4, 2, 8, 4);
	struct rk3026_ebc_last last;

	t_fb_box(box, 4, 2, 8, 4, 0);
	t_fb_box(boxes, 4, 2, 8, 4, 0);
	t_fb_box(boxes, 10, 4, 12, 6, 0);
	KUNIT_ASSERT_GT(test, ebc->du_cleanup_ceiling, ebc->du_cleanup_area);

	ebc->du_area = ebc->du_cleanup_ceiling - 1;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, box, T_W * 4,
							&r1));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_EQ(test, ebc->du_area, ebc->du_cleanup_ceiling + 7);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, boxes, T_W * 4,
							&r2));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_CLEANUP_CEILING);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 0);
	KUNIT_EXPECT_EQ(test, t_px(ebc->shadows.prev, 10, 4), 0);

	KUNIT_EXPECT_EQ(test, d.n, 2);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_FULL);
}

/*
 * The policy fixture with rk3026_ebc_drive() stubbed (t_driven_ebc()) and a
 * waveform that has, with @a2_table, an A2 table of its own of 2 frames
 * next to the gray2 (DU) table of 1, so the frames driven tell the two
 * apart.
 */
static struct rk3026_ebc *t_a2_ebc(struct kunit *test, struct t_drives *d,
				   bool a2_table)
{
	static const struct t_rkf_seg segs[] = {
		{ T_RKF_RESET, 5 }, { T_RKF_GRAY16, 1 }, { T_RKF_GRAY2, 1 },
		{ T_RKF_GRAYA2, 2 },
	};
	struct rk3026_ebc *ebc = t_driven_ebc(test, d, true);

	if (a2_table)
		ebc->waveform = t_rkf_wf(test, ebc->dev, segs,
					 ARRAY_SIZE(segs), 0);
	return ebc;
}

/*
 * A black box of 4 x 2 pixels on the white glass, one plane update, with
 * A2 on for @max_area pixels; returns the record.
 */
static void t_a2_box(struct kunit *test, struct rk3026_ebc *ebc, u32 max_area,
		     struct rk3026_ebc_last *last)
{
	struct drm_rect r = DRM_RECT_INIT(4, 2, 4, 2);
	u32 *box = t_plane_fb(test, 0xffffff);

	t_fb_box(box, 4, 2, 8, 4, 0);
	ebc->a2_max_area = max_area;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, box, T_W * 4,
							&r));
	rk3026_ebc_do_refresh(ebc, last);
	KUNIT_EXPECT_EQ(test, last->outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 8);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
}

/*
 * A2 off, as probe leaves it: the small two-level plane part is driven DU,
 * as before A2, with an A2 table in the waveform.
 */
static void ebc_test_glue_a2_off(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_a2_ebc(test, &d, true);
	struct rk3026_ebc_last last;

	t_a2_box(test, ebc, 0, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.lut_mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.frames, 1);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_BLACK_WHITE);
}

/*
 * A2 on, and the part well within it, with a waveform that has no A2 table
 * of its own: the part is A2, the A2 fetch hands out the DU table (1
 * frame), and it is drawn in the DSP_CTRL class A2 and DU share, never
 * carried for it.
 */
static void ebc_test_glue_a2_no_table(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_a2_ebc(test, &d, false);
	struct rk3026_ebc_last last;

	t_a2_box(test, ebc, U32_MAX, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_A2);
	KUNIT_EXPECT_EQ(test, last.lut_mode, EBC_MODE_A2);
	KUNIT_EXPECT_EQ(test, last.frames, 1);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_A2);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_A2);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_dsp_ctrl_mode(EBC_MODE_A2),
			rk3026_ebc_dsp_ctrl_mode(EBC_MODE_BLACK_WHITE));
}

/*
 * Draw a box of @fb that t_fb_box() filled, @x1, @y1 to @x2, @y2, as one
 * plane update with that damage box; returns the record.
 */
static void t_a2_draw(struct kunit *test, struct rk3026_ebc *ebc, u32 *fb,
		      int x1, int y1, int x2, int y2,
		      struct rk3026_ebc_last *last)
{
	struct drm_rect r = { .x1 = x1, .y1 = y1, .x2 = x2, .y2 = y2 };

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4,
							&r));
	rk3026_ebc_do_refresh(ebc, last);
	KUNIT_EXPECT_EQ(test, last->outcome, EBC_OUTCOME_DRIVEN);
}

/*
 * A2 on for 8 pixels, each step one plane update on the glass the one
 * before left:
 *  - a black box of 8 pixels on white is driven A2, with the A2 table (2
 *    frames), reason auto_a2, and adds its 8 pixels to the budget;
 *  - one of 9 pixels is one over: DU;
 *  - a grey box of 4 pixels is driven PART, and leaves the budget;
 *  - the same 4 pixels made black are over a grey: DU, though small.
 */
static void ebc_test_glue_a2_driven(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_a2_ebc(test, &d, true);
	u32 *fb = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;

	ebc->a2_max_area = 8;

	t_fb_box(fb, 0, 0, 4, 2, 0);
	t_a2_draw(test, ebc, fb, 0, 0, 4, 2, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_A2);
	KUNIT_EXPECT_EQ(test, last.lut_mode, EBC_MODE_A2);
	KUNIT_EXPECT_EQ(test, last.frames, 2);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_A2);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 8);
	KUNIT_EXPECT_EQ(test, t_px(ebc->shadows.prev, 3, 1), 0);

	t_fb_box(fb, 8, 0, 11, 3, 0);
	t_a2_draw(test, ebc, fb, 8, 0, 11, 3, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 17);

	t_fb_box(fb, 12, 4, 14, 6, 0x808080);
	t_a2_draw(test, ebc, fb, 12, 4, 14, 6, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_PART);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 17);

	t_fb_box(fb, 12, 4, 14, 6, 0);
	t_a2_draw(test, ebc, fb, 12, 4, 14, 6, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 21);
	KUNIT_EXPECT_EQ(test, t_px(ebc->shadows.prev, 13, 5), 0);

	KUNIT_EXPECT_EQ(test, d.n, 4);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_A2);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, d.mode[2], EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, d.mode[3], EBC_MODE_BLACK_WHITE);
}

/*
 * A2 on, the budget spent: a small part is still A2, and a big one takes
 * the cleanup FULL however far A2 reaches.
 */
static void ebc_test_glue_a2_spent(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_a2_ebc(test, &d, true);
	u32 *fb = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;

	ebc->a2_max_area = U32_MAX;
	ebc->du_area = ebc->du_cleanup_area;

	t_fb_box(fb, 0, 0, 4, 2, 0);
	t_a2_draw(test, ebc, fb, 0, 0, 4, 2, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_A2);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_A2);
	KUNIT_EXPECT_EQ(test, ebc->du_area, ebc->du_cleanup_area + 8);

	t_fb_box(fb, 0, 0, T_W, T_H, 0);
	t_a2_draw(test, ebc, fb, 0, 0, T_W, T_H, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_CLEANUP_BIG);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 0);

	KUNIT_EXPECT_EQ(test, d.n, 2);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_A2);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_FULL);
}

/* The deadline of the idle cleanup the last plane update armed. */
static unsigned long t_idle_at(struct rk3026_ebc *ebc)
{
	return ebc->idle_last_plane + msecs_to_jiffies(ebc->du_cleanup_idle_ms);
}

/*
 * The idle cleanup end to end. A black plane update over the white glass
 * is driven DU, spends the budget and arms the cleanup. One jiffy short of
 * the delay the thread's look does nothing; at the delay it queues the
 * cleanup, a whole-panel FULL with a seq of its own, and disarms it. The
 * cleanup's snapshot is identical to the glass, and it is driven all the
 * same: FULL, reason cleanup_idle, the budget cleared. A later look finds
 * it disarmed: one arming, one cleanup.
 */
static void ebc_test_glue_idle_cleanup(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	struct rk3026_ebc_set *pre = &ebc->pending.pre;
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_last last;
	unsigned long at;
	u32 seq;

	/* No part is big: the black panel spends the budget in DU. */
	ebc->du_cleanup_big_area = U32_MAX;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	KUNIT_EXPECT_TRUE(test, ebc->idle_armed);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, ebc->du_area, T_W * T_H);

	at = t_idle_at(ebc);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_idle_tick(ebc, at - 1));
	KUNIT_EXPECT_TRUE(test, ebc->idle_armed);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));

	KUNIT_EXPECT_TRUE(test, rk3026_ebc_idle_tick(ebc, at));
	KUNIT_EXPECT_FALSE(test, ebc->idle_armed);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_work_pending(&ebc->pending));
	KUNIT_EXPECT_TRUE(test, pre->cleanup_idle);
	KUNIT_EXPECT_EQ(test, pre->mode, EBC_MODE_FULL);
	KUNIT_EXPECT_FALSE(test, pre->auto_ok);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&pre->box, &t_panel));
	seq = ebc->pending.queued_seq;
	KUNIT_EXPECT_EQ(test, pre->seq, seq);
	KUNIT_EXPECT_EQ(test, ebc->stats.updates, 2);
	/* It wrote no pixel: plane updates need no whole-panel resync. */
	KUNIT_EXPECT_FALSE(test, ebc->plane_resync);

	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.source, EBC_SOURCE_IDLE);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_CLEANUP_IDLE);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 0);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_EQ(test, ebc->shadows.prev[0], 0);
	KUNIT_EXPECT_EQ(test, ebc->stats.skips, 0);

	/* Disarmed: no second cleanup, however long the glass stays idle. */
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_idle_tick(ebc, at + 100000));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));
	KUNIT_EXPECT_EQ(test, d.n, 2);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_FULL);
}

/*
 * At the delay a look that finds no cleanup due disarms it all the same:
 * a budget not spent (a small box typed), an mxcfb client in front, the
 * CRTC off. Only the next plane update arms it again, and an update the
 * mxcfb client blocks does not.
 */
static void ebc_test_glue_idle_not_due(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	u32 *box = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;

	t_fb_box(box, 4, 2, 8, 4, 0);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, box, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 8);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_idle_tick(ebc, t_idle_at(ebc)));
	KUNIT_EXPECT_FALSE(test, ebc->idle_armed);

	/* Spent, then an mxcfb client takes the panel. */
	ebc->du_area = ebc->du_cleanup_area;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, box, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_SKIPPED);
	KUNIT_EXPECT_TRUE(test, ebc->idle_armed);
	ebc->mxcfb_active = true;
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_idle_tick(ebc, t_idle_at(ebc)));
	KUNIT_EXPECT_FALSE(test, ebc->idle_armed);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_plane_update(ebc, box, T_W * 4,
							 &t_panel));
	KUNIT_EXPECT_FALSE(test, ebc->idle_armed);

	/* The client gone, the CRTC off. */
	ebc->mxcfb_active = false;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, box, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	ebc->crtc_on = false;
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_idle_tick(ebc, t_idle_at(ebc)));
	KUNIT_EXPECT_FALSE(test, ebc->idle_armed);

	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));
	KUNIT_EXPECT_EQ(test, ebc->du_area, ebc->du_cleanup_area);
	KUNIT_EXPECT_EQ(test, d.n, 1);
}

/*
 * An idle cleanup that is not driven (here: no gray16 LUT) is not carried:
 * it would make the next part, a small box typed, a whole-panel FULL. The
 * budget stays spent, and the cleanup does not fire again on its own: the
 * look that queued it disarmed it. The next plane update, a small box, is
 * drawn as its own part, DU; the end of its scan arms the cleanup again,
 * and at its delay, the LUT back, the cleanup is driven: FULL, reason
 * cleanup_idle, the budget cleared.
 */
static void ebc_test_glue_idle_carried(struct kunit *test)
{
	static const struct t_rkf_seg reset = { T_RKF_RESET, 5 };
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	struct drm_rect r = DRM_RECT_INIT(4, 2, 4, 2);
	struct ebc_waveform *wf = ebc->waveform;
	u32 *black = t_plane_fb(test, 0);
	u32 *box = t_plane_fb(test, 0);
	struct rk3026_ebc_last last;
	unsigned long at;

	t_fb_box(box, 4, 2, 8, 4, 0xffffff);
	/* No part is big: the black panel spends the budget in DU. */
	ebc->du_cleanup_big_area = U32_MAX;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, ebc->du_area, T_W * T_H);

	at = t_idle_at(ebc);
	ebc->waveform = t_rkf_wf(test, ebc->dev, &reset, 1, 0);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_idle_tick(ebc, at));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_CLEANUP_IDLE);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_EQ(test, ebc->du_area, T_W * T_H);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_FALSE(test, ebc->idle_armed);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_idle_tick(ebc, at + 100000));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));

	ebc->waveform = wf;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, box, T_W * 4,
							&r));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&last.clip, &r));
	KUNIT_EXPECT_EQ(test, ebc->du_area, T_W * T_H + 8);
	KUNIT_EXPECT_TRUE(test, ebc->idle_armed);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_idle_tick(ebc, t_idle_at(ebc)));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_CLEANUP_IDLE);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 0);
	KUNIT_EXPECT_EQ(test, t_px(ebc->shadows.prev, 4, 2), 15);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));

	KUNIT_EXPECT_EQ(test, d.n, 3);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, d.mode[2], EBC_MODE_FULL);
}

/*
 * What holds the glass for the look at the idle cleanup, on a spent
 * budget: a repair due, a part pending, a carry waiting. Each look finds
 * the delay over and nothing due: it queues nothing and disarms. The end
 * of a plane part's scan arms the cleanup again and stamps its time, so
 * the part a look found pending is followed by a cleanup all the same.
 */
static void ebc_test_glue_idle_busy(struct kunit *test)
{
	static const struct t_rkf_seg reset = { T_RKF_RESET, 5 };
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	struct drm_rect r = DRM_RECT_INIT(4, 2, 4, 2);
	struct ebc_waveform *wf = ebc->waveform;
	u32 *black = t_plane_fb(test, 0);
	u32 *box = t_plane_fb(test, 0);
	u32 *box2 = t_plane_fb(test, 0);
	struct rk3026_ebc_last last;
	unsigned long stamp;

	t_fb_box(box, 4, 2, 8, 4, 0xffffff);
	t_fb_box(box2, 4, 2, 8, 4, 0xffffff);
	t_fb_box(box2, 4, 2, 6, 4, 0);
	ebc->du_cleanup_big_area = U32_MAX;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_ASSERT_EQ(test, ebc->du_area, T_W * T_H);
	KUNIT_ASSERT_TRUE(test, ebc->idle_armed);

	/* A repair due. */
	ebc->shadows.prev_unknown = true;
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_idle_tick(ebc, t_idle_at(ebc)));
	KUNIT_EXPECT_FALSE(test, ebc->idle_armed);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));
	ebc->shadows.prev_unknown = false;

	/* A part pending; its scan arms the cleanup again, stamped later. */
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, box, T_W * 4,
							&r));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_idle_tick(ebc, t_idle_at(ebc)));
	KUNIT_EXPECT_FALSE(test, ebc->idle_armed);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_work_pending(&ebc->pending));
	ebc->idle_last_plane -= 1000;
	stamp = ebc->idle_last_plane;
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_TRUE(test, ebc->idle_armed);
	KUNIT_EXPECT_TRUE(test, time_after(ebc->idle_last_plane, stamp));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_idle_tick(ebc, t_idle_at(ebc)));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_CLEANUP_IDLE);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 0);

	/* A carry waiting: a part not drawn for want of a LUT. */
	ebc->waveform = t_rkf_wf(test, ebc->dev, &reset, 1, 0);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, box2, T_W * 4,
							&r));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_ASSERT_TRUE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_TRUE(test, ebc->idle_armed);
	ebc->du_area = ebc->du_cleanup_area;
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_idle_tick(ebc, t_idle_at(ebc)));
	KUNIT_EXPECT_FALSE(test, ebc->idle_armed);
	/* Nothing to take; the carry's retry holds its update unfinished. */
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_pending(&ebc->pending));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_work_pending(&ebc->pending));
	ebc->waveform = wf;

	KUNIT_EXPECT_EQ(test, d.n, 3);
}

/*
 * A repair RESET that runs to its end whitens the glass: the budget is
 * cleared even though the part after it is not driven (no gray16 LUT).
 */
static void ebc_test_glue_du_repaired(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, false);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_last last;

	ebc->du_area = 50;
	ebc->shadows.prev_unknown = true;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_TRUE(test, last.repaired);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_FALSE(test, ebc->shadows.prev_unknown);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 0);
}

/*
 * A glass that is not known. With a RESET at the temperature the repair
 * runs first and an mxcfb DU send after it is drawn PART, the repair floor,
 * as before. Without one (gray16 and gray2 only) no repair runs: the send
 * is the repair and is driven FULL, which makes the glass known and clears
 * the budget; the plane part after it resolves as usual (DU), no FULL loop.
 * A FULL cut short leaves the mark, and the next part is FULL again.
 */
static void ebc_test_glue_repair_no_reset(struct kunit *test)
{
	static const struct t_rkf_seg segs[] = {
		{ T_RKF_GRAY16, 1 }, { T_RKF_GRAY2, 1 },
	};
	struct drm_rect r = DRM_RECT_INIT(2, 2, 4, 2);
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	u32 *black = t_plane_fb(test, 0);
	u32 *white = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;
	u32 seq;

	ebc->du_cleanup_area = 2 * T_W * T_H;
	ebc->shadows.prev_unknown = true;
	t_send(ebc, &r, EBC_MODE_BLACK_WHITE, 0, EBC_SOURCE_MXCFB);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_TRUE(test, last.repaired);
	KUNIT_ASSERT_EQ(test, d.n, 2);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_REPAIR_FLOOR);
	KUNIT_EXPECT_FALSE(test, ebc->shadows.prev_unknown);

	ebc->waveform = t_rkf_wf(test, ebc->dev, segs, ARRAY_SIZE(segs), 0);
	ebc->shadows.prev_unknown = true;
	ebc->du_area = 50;
	seq = t_send(ebc, &r, EBC_MODE_BLACK_WHITE, 0, EBC_SOURCE_MXCFB);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_FALSE(test, last.repaired);
	KUNIT_ASSERT_EQ(test, d.n, 3);
	KUNIT_EXPECT_EQ(test, d.mode[2], EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, d.seq[2], seq);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.lut_mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_REPAIR_FLOOR);
	KUNIT_EXPECT_FALSE(test, ebc->shadows.prev_unknown);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 0);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));

	/* Black over the glass the FULL left: two-level, DU, as usual. */
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_ASSERT_EQ(test, d.n, 4);
	KUNIT_EXPECT_EQ(test, d.mode[3], EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_FALSE(test, ebc->shadows.prev_unknown);

	/* The FULL cut short: the mark stays, and the next part repairs. */
	kunit_activate_static_stub(test, rk3026_ebc_drive, t_drive_timeout);
	ebc->shadows.prev_unknown = true;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_TIMEOUT);
	KUNIT_ASSERT_EQ(test, d.n, 5);
	KUNIT_EXPECT_EQ(test, d.mode[4], EBC_MODE_FULL);
	KUNIT_EXPECT_TRUE(test, ebc->shadows.prev_unknown);

	kunit_activate_static_stub(test, rk3026_ebc_drive, t_drive_ok);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_ASSERT_EQ(test, d.n, 6);
	KUNIT_EXPECT_EQ(test, d.mode[5], EBC_MODE_FULL);
	KUNIT_EXPECT_FALSE(test, ebc->shadows.prev_unknown);
}

/* rk3026_ebc_clock() on the fake clock of struct t_drives. */
static ktime_t t_clock(struct rk3026_ebc *ebc)
{
	struct t_drives *d = kunit_get_current_test()->priv;

	return d->now;
}

/*
 * The record the tracepoints read: the part's newest seq, its box, the
 * pixels its snapshot changes and the DU budget spent at its take, and
 * when it was taken, on the driver's clock (a fake one here). A black
 * plane update over the white glass is driven DU and changes every pixel,
 * its waveform at once after the take (setup 1 us, the floor); the same
 * update again is skipped, with the budget it spent; with the CRTC off
 * nothing is built (0 pixels). A repair RESET is driven for the part's own
 * seq, before the part, which starts the RESET's 7 ms after its take.
 */
static void ebc_test_glue_part_record(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	u32 *black = t_plane_fb(test, 0);
	u32 *white = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;
	ktime_t t0 = ms_to_ktime(1000);

	d.now = t0;
	kunit_activate_static_stub(test, rk3026_ebc_clock, t_clock);
	ebc->du_cleanup_area = 2 * T_W * T_H;
	ebc->du_area = 5;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.seq, ebc->pending.queued_seq);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&last.clip, &t_panel));
	KUNIT_EXPECT_EQ(test, last.changed_px, T_W * T_H);
	KUNIT_EXPECT_EQ(test, last.du_area, 5);
	KUNIT_EXPECT_EQ(test, ebc->du_area, 5 + T_W * T_H);
	KUNIT_EXPECT_EQ(test, last.taken, t0);
	KUNIT_EXPECT_EQ(test, last.gap_us, 0);
	KUNIT_EXPECT_EQ(test, last.tries, 0);
	KUNIT_EXPECT_FALSE(test, last.gave_up);
	KUNIT_ASSERT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.seq[0], last.seq);
	KUNIT_EXPECT_EQ(test, d.setup_us[0], 1);
	KUNIT_EXPECT_EQ(test, ebc->last.seq, last.seq);
	KUNIT_EXPECT_EQ(test, ebc->last.changed_px, T_W * T_H);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_SKIPPED);
	KUNIT_EXPECT_EQ(test, last.seq, ebc->pending.queued_seq);
	KUNIT_EXPECT_EQ(test, last.changed_px, 0);
	KUNIT_EXPECT_EQ(test, last.du_area, 5 + T_W * T_H);
	KUNIT_EXPECT_EQ(test, d.n, 1);

	ebc->crtc_on = false;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_CRTC_OFF);
	KUNIT_EXPECT_EQ(test, last.seq, ebc->pending.queued_seq);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&last.clip, &t_panel));
	KUNIT_EXPECT_EQ(test, last.changed_px, 0);
	KUNIT_EXPECT_EQ(test, d.n, 1);

	ebc->crtc_on = true;
	ebc->shadows.prev_unknown = true;
	d.now = ktime_add_ms(t0, 500);
	d.drive_time[1] = ms_to_ktime(7);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_TRUE(test, last.repaired);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.taken, ktime_add_ms(t0, 500));
	KUNIT_ASSERT_EQ(test, d.n, 3);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, d.seq[1], last.seq);
	KUNIT_EXPECT_EQ(test, d.seq[2], last.seq);
	KUNIT_EXPECT_EQ(test, d.setup_us[1], 1);
	KUNIT_EXPECT_EQ(test, d.setup_us[2], 7000);
}

/* rk3026_ebc_raise_rails() without a regulator, see struct t_drives. */
static int t_rails(struct rk3026_ebc *ebc)
{
	struct t_drives *d = kunit_get_current_test()->priv;

	if (!d->rails++ && d->rails_fb)
		rk3026_ebc_plane_update(ebc, d->rails_fb, T_W * 4,
					&d->rails_box);
	if (d->rails == 1 && d->rails_reset)
		t_send(ebc, &t_panel, EBC_MODE_RESET, 0, EBC_SOURCE_ENABLE);
	if (d->rails == 1 && d->rails_suspend)
		rk3026_ebc_sleep_begin(ebc);
	if (d->rails_ret)
		return d->rails_ret;
	if (!ebc->epd_on) {
		d->now = ktime_add(d->now, d->ramp);
		ebc->rails_off_at = jiffies + msecs_to_jiffies(200);
	}
	ebc->epd_on = true;
	ebc->rails_on = true;
	return 0;
}

/* t_driven_ebc() with the rails down and raised by t_rails(). */
static struct rk3026_ebc *t_rails_ebc(struct kunit *test, struct t_drives *d)
{
	struct rk3026_ebc *ebc = t_driven_ebc(test, d, true);

	ebc->epd_on = false;
	kunit_activate_static_stub(test, rk3026_ebc_raise_rails, t_rails);
	return ebc;
}

/*
 * A part that will not be driven leaves the rails down: skipped (the
 * target is what the glass shows), dropped with the CRTC off, carried
 * without a waveform. The next part that will be driven, the carry with
 * it, brings them up, once.
 */
static void ebc_test_glue_rails_not_raised(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_rails_ebc(test, &d);
	struct ebc_waveform *wf = ebc->waveform;
	u32 *white = t_plane_fb(test, 0xffffff);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_last last;

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_SKIPPED);

	ebc->crtc_on = false;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_CRTC_OFF);

	ebc->crtc_on = true;
	ebc->waveform = NULL;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_WAVEFORM);
	KUNIT_EXPECT_TRUE(test, ebc->pending.carry.merged);

	KUNIT_EXPECT_EQ(test, d.rails, 0);
	KUNIT_EXPECT_FALSE(test, ebc->epd_on);
	KUNIT_EXPECT_FALSE(test, ebc->rails_on);
	KUNIT_EXPECT_FALSE(test, last.temp_read);
	KUNIT_EXPECT_EQ(test, d.n, 0);

	ebc->waveform = wf;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_TRUE(test, ebc->epd_on);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
}

/*
 * The rails come up before the take: a plane update sent while they come
 * up joins the part, which is driven once, for the newer seq, over both
 * boxes. With the rails up the next part is asked for them again (the
 * real rk3026_ebc_raise_rails() leaves them as they are) and taken alone.
 */
static void ebc_test_glue_rails_before_take(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_rails_ebc(test, &d);
	struct drm_rect a = DRM_RECT_INIT(0, 0, 4, 2);
	struct drm_rect c = DRM_RECT_INIT(4, 4, 2, 2);
	u32 *fb = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;
	u32 first;

	t_fb_box(fb, 0, 0, 4, 2, 0);
	t_fb_box(fb, 8, 2, 12, 4, 0);
	d.rails_fb = fb;
	d.rails_box = DRM_RECT_INIT(8, 2, 4, 2);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &a));
	first = ebc->pending.queued_seq;
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.merged, 2);
	KUNIT_EXPECT_EQ(test, last.seq, first + 1);
	KUNIT_EXPECT_EQ(test, last.clip.x1, 0);
	KUNIT_EXPECT_EQ(test, last.clip.y1, 0);
	KUNIT_EXPECT_EQ(test, last.clip.x2, 12);
	KUNIT_EXPECT_EQ(test, last.clip.y2, 4);
	KUNIT_EXPECT_EQ(test, last.changed_px, 16);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_TRUE(test, last.temp_read);
	KUNIT_ASSERT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.seq[0], first + 1);
	KUNIT_EXPECT_EQ(test, ebc->stats.coalesced, 1);
	KUNIT_EXPECT_EQ(test, t_px(ebc->shadows.prev, 9, 3), 0);
	KUNIT_EXPECT_EQ(test, t_px(ebc->shadows.prev, 5, 1), 15);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));

	d.rails_fb = NULL;
	t_fb_box(fb, 4, 4, 6, 6, 0);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &c));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, d.rails, 2);
	KUNIT_EXPECT_EQ(test, last.merged, 1);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&last.clip, &c));
	KUNIT_EXPECT_EQ(test, d.n, 2);
}

/*
 * Rails that do not come up are asked once for the part: it is taken all
 * the same, with the sensor read before the take, and carried for a retry,
 * its update not finished yet.
 */
static void ebc_test_glue_rails_fail(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_rails_ebc(test, &d);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_last last;
	u32 seq;

	d.rails_ret = -EIO;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	seq = ebc->pending.queued_seq;
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_RAILS);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_EQ(test, d.n, 0);
	KUNIT_EXPECT_TRUE(test, last.temp_read);
	KUNIT_EXPECT_EQ(test, last.temp, EBC_TEMP_FALLBACK);
	KUNIT_EXPECT_TRUE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_TRUE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_FALSE(test, last.gave_up);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_EQ(test, ebc->stats.dropped, 1);
	KUNIT_EXPECT_FALSE(test, ebc->epd_on);
}

/*
 * The RESET of a CRTC enable that is not driven, for want of rails or of
 * a RESET LUT: its wait ends, the glass is marked not known, and it is
 * given up and counted (carry_failed), where it was silent before.
 */
static void ebc_test_glue_reset_lost(struct kunit *test)
{
	static const struct t_rkf_seg segs[] = {
		{ T_RKF_GRAY16, 1 }, { T_RKF_GRAY2, 1 },
	};
	struct t_drives d;
	struct rk3026_ebc *ebc = t_rails_ebc(test, &d);
	struct rk3026_ebc_last last;
	u32 seq;

	d.rails_ret = -EIO;
	seq = t_send(ebc, &t_panel, EBC_MODE_RESET, 0, EBC_SOURCE_ENABLE);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_RAILS);
	KUNIT_EXPECT_TRUE(test, last.gave_up);
	KUNIT_EXPECT_EQ(test, last.tries, 0);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_TRUE(test, ebc->shadows.prev_unknown);
	KUNIT_EXPECT_EQ(test, ebc->stats.carry_failed, 1);
	KUNIT_EXPECT_FALSE(test, ebc->pending.retry.failing);

	d.rails_ret = 0;
	ebc->waveform = t_rkf_wf(test, ebc->dev, segs, ARRAY_SIZE(segs), 0);
	seq = t_send(ebc, &t_panel, EBC_MODE_RESET, 0, EBC_SOURCE_ENABLE);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_TRUE(test, last.gave_up);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_EQ(test, ebc->stats.carry_failed, 2);
	KUNIT_EXPECT_EQ(test, d.n, 0);
}

/*
 * t_rails_ebc() on the fake clock, at 1 s, with the take's gap as probe
 * sets it; the last part finished long before.
 */
static struct rk3026_ebc *t_gap_ebc(struct kunit *test, struct t_drives *d)
{
	struct rk3026_ebc *ebc = t_rails_ebc(test, d);

	ebc->take_quiet_us = EBC_TAKE_QUIET_US;
	ebc->take_cap_ms = EBC_TAKE_CAP_MS;
	d->now = ms_to_ktime(1000);
	kunit_activate_static_stub(test, rk3026_ebc_clock, t_clock);
	return ebc;
}

/*
 * The refresh thread's loop on the fake clock while a part is pending, as
 * ebc_refresh_thread() runs it: a pass (rk3026_ebc_thread_pass(),
 * rk3026_ebc_thread_step()) that looks at the part (rk3026_ebc_take_ready()),
 * with the rails and the idle cleanup held while the gap is open, past
 * their deadlines too; and while the gap is open, ebc_gap_wait(): a wait
 * that sleeps for the time rk3026_ebc_gap_timeout() gives, ending at the
 * gap's end or at the next send, and none at all if the end has passed
 * (the rails' ramp outlasted the gap): the next look must take then.
 * Send k (k from *@sent up to @n) is a plane update of @boxes[k % @nboxes]
 * from @fb at @every_us * k after the start. Every wait must sleep at the
 * look (no newer update: no spin) and wake on the next send. Returns the
 * looks, the one that takes included.
 */
static unsigned int t_gap_run(struct kunit *test, struct rk3026_ebc *ebc,
			      struct t_drives *d, const u32 *fb,
			      const struct drm_rect *boxes, unsigned int nboxes,
			      unsigned int n, s64 every_us, unsigned int *sent)
{
	ktime_t t0 = d->now;
	unsigned int looks = 0;
	bool ended = false;

	while (looks++ < 1000) {
		struct rk3026_ebc_pass pass;
		ktime_t next = KTIME_MAX;
		ktime_t left, end;
		long timeout;

		rk3026_ebc_thread_pass(ebc, jiffies, &pass);
		KUNIT_EXPECT_EQ(test, rk3026_ebc_thread_step(&pass, &timeout),
				EBC_STEP_LOOK);
		if (ebc->gap.open) {
			pass.rails_off_at = pass.now;
			pass.idle_armed = true;
			pass.idle_at = pass.now;
			KUNIT_EXPECT_EQ(test,
					rk3026_ebc_thread_step(&pass, &timeout),
					EBC_STEP_LOOK);
		}
		if (rk3026_ebc_take_ready(ebc, d->now))
			return looks;
		KUNIT_EXPECT_FALSE(test, ended);
		KUNIT_EXPECT_TRUE(test, ebc->gap.open);
		KUNIT_EXPECT_TRUE(test, rk3026_ebc_work_pending(&ebc->pending));
		KUNIT_EXPECT_FALSE(test,
				   rk3026_ebc_gap_woken(&ebc->pending,
							ebc->gap.seen, true));
		if (!rk3026_ebc_gap_timeout(ebc->gap.at, d->now, &left)) {
			ended = true;
			continue;
		}
		/* The hrtimer expires at the gap's end. */
		end = ktime_add_safe(left, d->now);
		KUNIT_EXPECT_EQ(test, end, ebc->gap.at);
		if (*sent < n)
			next = ktime_add_us(t0, *sent * every_us);
		if (!ktime_before(next, end)) {
			d->now = end;
			continue;
		}
		if (ktime_after(next, d->now))
			d->now = next;
		KUNIT_EXPECT_TRUE(test,
				  rk3026_ebc_plane_update(ebc, fb, T_W * 4,
							  &boxes[*sent % nboxes]));
		(*sent)++;
		KUNIT_EXPECT_TRUE(test,
				  rk3026_ebc_gap_woken(&ebc->pending,
						       ebc->gap.seen, true));
	}
	KUNIT_FAIL(test, "no take after %u looks", looks);
	return looks;
}

/*
 * A burst of strips, as an fbdev fill sends them 2 ms apart, with the
 * rails down: the first strip opens the gap and raises the rails (4 ms
 * here), each later strip moves its end, and the take comes 10 ms after
 * the last strip. One part, one scan for the whole fill; one look per
 * strip and one at the gap's end, each followed by a wait that sleeps.
 */
static void ebc_test_glue_gap_burst(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	u32 *black = t_plane_fb(test, 0);
	struct drm_rect rows[T_H];
	struct rk3026_ebc_last last;
	unsigned int i, sent = 1, looks;
	ktime_t t0 = d.now;

	for (i = 0; i < T_H; i++)
		rows[i] = DRM_RECT_INIT(0, i, T_W, 1);
	d.ramp = ms_to_ktime(4);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&rows[0]));
	looks = t_gap_run(test, ebc, &d, black, rows, T_H, T_H, 2000, &sent);
	KUNIT_EXPECT_EQ(test, sent, T_H);
	KUNIT_EXPECT_EQ(test, looks, T_H + 1);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_EQ(test, ktime_us_delta(d.now, t0), 20000);
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);

	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.merged, T_H);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&last.clip, &t_panel));
	KUNIT_EXPECT_EQ(test, last.gap_us, 20000);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, ebc->take_done, d.now);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));
}

/*
 * A lone small update is not delayed. With the rails down the gap raises
 * them (42 ms here) and ends with them: the drive would have waited for
 * them anyway. With the rails up it is taken at once, no gap. A part with
 * an mxcfb send in it is taken at once with the rails down, and so is any
 * part with no quiet gap.
 */
static void ebc_test_glue_gap_lone(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	u32 *fb = t_plane_fb(test, 0xffffff);
	struct drm_rect a = DRM_RECT_INIT(4, 2, 2, 2);
	struct drm_rect b = DRM_RECT_INIT(8, 2, 2, 2);
	struct rk3026_ebc_last last;
	ktime_t t0 = d.now;

	d.ramp = ms_to_ktime(42);
	t_fb_box(fb, 4, 2, 6, 4, 0);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &a));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_TRUE(test, ebc->epd_on);
	KUNIT_EXPECT_EQ(test, ktime_us_delta(d.now, t0), 42000);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.gap_us, 42000);
	KUNIT_EXPECT_EQ(test, d.n, 1);

	t_fb_box(fb, 8, 2, 10, 4, 0);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &b));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.gap_us, 0);
	KUNIT_EXPECT_EQ(test, d.n, 2);

	ebc->epd_on = false;
	ebc->rails_on = false;
	d.rails = 0;
	t_fb_box(fb, 4, 2, 6, 4, 0xffffff);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &a));
	t_send(ebc, &b, EBC_MODE_FULL, 9, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	KUNIT_EXPECT_EQ(test, d.rails, 0);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.merged, 2);
	KUNIT_EXPECT_EQ(test, last.gap_us, 0);

	ebc->epd_on = false;
	ebc->rails_on = false;
	d.rails = 0;
	ebc->take_quiet_us = 0;
	t_fb_box(fb, 4, 2, 6, 4, 0);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &a));
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &a));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_EQ(test, d.rails, 0);
}

/*
 * A lone update with the rails down whose ramp (37 ms, as the book's)
 * outlasts the quiet gap (10 ms): the gap's end has passed when the
 * thread would wait, so it does not sleep, and the next look takes the
 * part, its gap the ramp. No later update is needed to end the wait: one
 * sent 200 ms later is a part of its own, taken at once with the rails up.
 */
static void ebc_test_glue_gap_ramp_past(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	u32 *fb = t_plane_fb(test, 0xffffff);
	struct drm_rect a = DRM_RECT_INIT(4, 2, 2, 2);
	struct drm_rect b = DRM_RECT_INIT(8, 2, 2, 2);
	struct rk3026_ebc_last last;
	unsigned int sent = 1;
	ktime_t t0 = d.now;

	d.ramp = ms_to_ktime(37);
	t_fb_box(fb, 4, 2, 6, 4, 0);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &a));
	KUNIT_EXPECT_EQ(test, t_gap_run(test, ebc, &d, fb, &a, 1, 1, 0, &sent),
			2);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_EQ(test, ktime_us_delta(d.now, t0), 37000);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.merged, 1);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&last.clip, &a));
	KUNIT_EXPECT_EQ(test, last.gap_us, 37000);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));

	d.now = ktime_add_ms(t0, 200);
	t_fb_box(fb, 8, 2, 10, 4, 0);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &b));
	sent = 1;
	KUNIT_EXPECT_EQ(test, t_gap_run(test, ebc, &d, fb, &b, 1, 1, 0, &sent),
			1);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.merged, 1);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&last.clip, &b));
	KUNIT_EXPECT_EQ(test, last.gap_us, 0);
	KUNIT_EXPECT_EQ(test, ktime_us_delta(d.now, t0), 200000);
	KUNIT_EXPECT_EQ(test, d.n, 2);
}

/*
 * A blinking cursor, as fbcon draws it: on, then off 210 ms later, with
 * the rails down at each toggle (37 ms ramp). Each toggle is a part of its
 * own, taken at the look after the ramp and driven: the on is not held
 * until the off joins it, which would make the pair what the glass shows
 * and skip it.
 */
static void ebc_test_glue_gap_cursor_blink(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	u32 *fb = t_plane_fb(test, 0xffffff);
	struct drm_rect c = DRM_RECT_INIT(4, 2, 2, 2);
	struct rk3026_ebc_last last;
	ktime_t t0 = d.now;
	unsigned int i, sent;

	d.ramp = ms_to_ktime(37);
	for (i = 0; i < 2; i++) {
		ebc->epd_on = false;
		ebc->rails_on = false;
		d.now = ktime_add_ms(t0, 210 * i);
		t_fb_box(fb, 4, 2, 6, 4, i ? 0xffffff : 0);
		KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb,
								T_W * 4, &c));
		sent = 1;
		KUNIT_EXPECT_EQ(test, t_gap_run(test, ebc, &d, fb, &c, 1, 1, 0,
						&sent), 2);
		/* One raise at the gap's open, one at each drive. */
		KUNIT_EXPECT_EQ(test, d.rails, 2 * i + 1);
		KUNIT_EXPECT_EQ(test, ktime_us_delta(d.now, t0),
				210000 * i + 37000);
		rk3026_ebc_do_refresh(ebc, &last);
		KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
		KUNIT_EXPECT_EQ(test, last.merged, 1);
		KUNIT_EXPECT_EQ(test, last.gap_us, 37000);
	}
	KUNIT_EXPECT_EQ(test, d.n, 2);
}

/* The refresh thread's wait queue, as a waiter on it sees a wake-up. */
struct t_waker {
	struct wait_queue_entry	wq;
	unsigned int		woken;
};

static int t_wake(struct wait_queue_entry *wq, unsigned int mode, int flags,
		  void *key)
{
	container_of(wq, struct t_waker, wq)->woken++;
	return 0;
}

/*
 * rk3026_ebc_crtc_off(), as the CRTC disable calls it: the CRTC goes off,
 * the newest seq is returned for the disable's wait, and the refresh
 * thread's wait is woken, once.
 */
static void t_crtc_off(struct kunit *test, struct rk3026_ebc *ebc)
{
	struct t_waker w = { .woken = 0 };

	init_waitqueue_func_entry(&w.wq, t_wake);
	add_wait_queue(&ebc->update_wq, &w.wq);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_crtc_off(ebc),
			ebc->pending.queued_seq);
	remove_wait_queue(&ebc->update_wq, &w.wq);
	KUNIT_EXPECT_FALSE(test, ebc->crtc_on);
	KUNIT_EXPECT_EQ(test, w.woken, 1);
}

/*
 * No gap for a part that will not be driven, so no rails: a burst that
 * redraws what the glass shows is taken at once and skipped. A gap open
 * when the CRTC goes off wakes the thread's wait (rk3026_ebc_crtc_off()),
 * ends at the next look, and the part is dropped.
 */
static void ebc_test_glue_gap_not_driven(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	u32 *white = t_plane_fb(test, 0xffffff);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_last last;

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&t_panel));
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&t_panel));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_SKIPPED);
	KUNIT_EXPECT_EQ(test, d.rails, 0);
	KUNIT_EXPECT_FALSE(test, ebc->epd_on);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_TRUE(test, ebc->gap.open);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_gap_woken(&ebc->pending,
						      ebc->gap.seen,
						      ebc->crtc_on));
	t_crtc_off(test, ebc);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_gap_woken(&ebc->pending,
						     ebc->gap.seen,
						     ebc->crtc_on));
	d.now = ktime_add_us(d.now, 1000);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_CRTC_OFF);
	KUNIT_EXPECT_EQ(test, last.gap_us, 1000);
	KUNIT_EXPECT_EQ(test, d.n, 0);
}

/*
 * A steady stream, an update every 4 ms, never 10 ms quiet: with the rails
 * up it opens a gap once two updates are pending, and is taken at the cap,
 * 250 ms after its first update; then, with the previous part finished
 * 100 ms after the first update of the next stream, 250 ms after that
 * finish. Still one look per update.
 */
static void ebc_test_glue_gap_cap(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	struct drm_rect box = DRM_RECT_INIT(4, 2, 2, 2);
	u32 *black = t_plane_fb(test, 0);
	u32 *white = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;
	unsigned int sent = 2, looks;
	ktime_t t0 = d.now;

	ebc->epd_on = true;
	ebc->rails_on = true;
	ebc->rails_off_at = jiffies + msecs_to_jiffies(200);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&box));
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&box));
	looks = t_gap_run(test, ebc, &d, black, &box, 1, 1000, 4000, &sent);
	KUNIT_EXPECT_EQ(test, ktime_us_delta(d.now, t0), 250000);
	/* Sends at 8, 12, ... 248 ms: 61 of them, and the two at the start. */
	KUNIT_EXPECT_EQ(test, sent, 63);
	KUNIT_EXPECT_EQ(test, looks, 63);
	KUNIT_EXPECT_EQ(test, d.rails, 0);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.merged, 63);
	KUNIT_EXPECT_EQ(test, last.gap_us, 250000);

	d.now = ktime_add_ms(d.now, 1000);
	t0 = d.now;
	sent = 2;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&box));
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&box));
	ebc->take_done = ktime_add_ms(t0, 100);
	t_gap_run(test, ebc, &d, white, &box, 1, 1000, 4000, &sent);
	KUNIT_EXPECT_EQ(test, ktime_us_delta(d.now, t0), 350000);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.gap_us, 350000);
	KUNIT_EXPECT_EQ(test, d.n, 2);
}

/*
 * Parts that never wait for the quiet gap, with the rails down too: the
 * idle cleanup, the whole panel already, with a plane update joined; and,
 * with a gap open for a plane update, a RESET sent ends it at the next
 * look: the part in front of the RESET is taken at once, and the RESET
 * after it.
 */
static void ebc_test_glue_gap_exempt(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	struct drm_rect a = DRM_RECT_INIT(4, 2, 2, 2);
	u32 *fb = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;
	u32 seq;

	t_fb_box(fb, 4, 2, 6, 4, 0);
	t_send(ebc, &t_panel, EBC_MODE_FULL, 0, EBC_SOURCE_IDLE);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &a));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	KUNIT_EXPECT_EQ(test, d.rails, 0);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_CLEANUP_IDLE);
	KUNIT_EXPECT_EQ(test, last.merged, 2);
	KUNIT_EXPECT_EQ(test, last.gap_us, 0);
	KUNIT_EXPECT_EQ(test, d.rails, 1);

	ebc->epd_on = false;
	ebc->rails_on = false;
	t_fb_box(fb, 4, 2, 6, 4, 0xffffff);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &a));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_TRUE(test, ebc->gap.open);
	KUNIT_EXPECT_EQ(test, d.rails, 2);
	d.now = ktime_add_us(d.now, 2000);
	seq = t_send(ebc, &t_panel, EBC_MODE_RESET, 0, EBC_SOURCE_ENABLE);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_gap_woken(&ebc->pending,
						     ebc->gap.seen, true));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.gap_us, 2000);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, last.seq, seq);
	KUNIT_EXPECT_EQ(test, last.gap_us, 0);
	KUNIT_EXPECT_EQ(test, d.n, 3);
}

/*
 * With the rails up a lone update waits for the quiet gap when it is big
 * (half the panel here), or when it will be driven FULL: an auto plane
 * part with the DU budget at its ceiling. A smaller box is taken at once.
 */
static void ebc_test_glue_gap_big_full(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	struct drm_rect big = DRM_RECT_INIT(0, 0, 8, 6);
	struct drm_rect small = DRM_RECT_INIT(8, 0, 8, 5);
	u32 *black = t_plane_fb(test, 0);
	u32 *white = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;
	ktime_t t0 = d.now;

	ebc->epd_on = true;
	ebc->rails_on = true;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&big));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_TRUE(test, ebc->gap.open);
	KUNIT_EXPECT_EQ(test, ebc->gap.at, ktime_add_us(t0, EBC_TAKE_QUIET_US));
	d.now = ebc->gap.at;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.gap_us, EBC_TAKE_QUIET_US);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&small));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.gap_us, 0);

	ebc->du_area = ebc->du_cleanup_ceiling;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&small));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_TRUE(test, ebc->gap.open);
	d.now = ebc->gap.at;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_CLEANUP_CEILING);
	KUNIT_EXPECT_EQ(test, last.gap_us, EBC_TAKE_QUIET_US);
	/* Asked by the drives alone: no gap raises rails that are up. */
	KUNIT_EXPECT_EQ(test, d.rails, 3);
}

/*
 * A carry armed for its retry does not exempt what is sent before it: a
 * plane update carried for want of rails (its gap's raise failed), then a
 * burst at 95 ms waits in a gap, which the retry falling due at 100 ms
 * ends: the gap's end is the retry's time, so the thread's wait wakes for
 * it with no update to wake it. The part is taken with the carry in it,
 * as the carry's first try.
 */
static void ebc_test_glue_gap_retry(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	struct drm_rect a = DRM_RECT_INIT(0, 0, 4, 2);
	struct drm_rect b = DRM_RECT_INIT(8, 2, 4, 2);
	u32 *fb = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;
	ktime_t t0 = d.now;
	u32 s1;

	d.rails_ret = -EIO;
	t_fb_box(fb, 0, 0, 4, 2, 0);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &a));
	s1 = ebc->pending.queued_seq;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_RAILS);
	KUNIT_EXPECT_TRUE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_EQ(test, ebc->pending.retry.at, ktime_add_ms(t0, 100));

	d.rails_ret = 0;
	d.now = ktime_add_ms(t0, 95);
	t_fb_box(fb, 8, 2, 12, 4, 0);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &b));
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &b));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_TRUE(test, ebc->gap.open);
	KUNIT_EXPECT_EQ(test, ebc->gap.at, ktime_add_ms(t0, 100));
	d.now = ktime_add_ms(t0, 99);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_TRUE(test, ebc->gap.open);
	d.now = ebc->gap.at;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.tries, 1);
	KUNIT_EXPECT_EQ(test, last.gap_us, 5000);
	KUNIT_EXPECT_EQ(test, last.clip.x1, 0);
	KUNIT_EXPECT_EQ(test, last.clip.x2, 12);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, s1));
	KUNIT_EXPECT_FALSE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_EQ(test, d.n, 1);
}

/*
 * A carry made while a RESET is pending (the RESET sent while the rails
 * of the part in flight came up) is not in the RESET's part: the RESET
 * records no try, and its finish drops the carry and ends its waits.
 */
static void ebc_test_glue_retry_reset_tries(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	struct drm_rect a = DRM_RECT_INIT(0, 0, 4, 2);
	struct rk3026_ebc_last last;
	u32 s1;

	d.rails_ret = -EIO;
	d.rails_reset = true;
	t_img_fill(ebc->next_shadow, 0);
	s1 = t_send(ebc, &a, EBC_MODE_PART, 5, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_RAILS);
	KUNIT_EXPECT_TRUE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_TRUE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_TRUE(test, ebc->pending.reset.merged);

	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, last.tries, 0);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, s1));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));
}

/* rk3026_ebc_read_temp() without a sensor: @temp of struct t_drives. */
static int t_temp(struct rk3026_ebc *ebc)
{
	struct t_drives *d = kunit_get_current_test()->priv;

	return d->temp;
}

/*
 * t_driven_ebc() with gray16 and gray2 LUTs at every temperature, @hot
 * only from 20 degrees up, and the sensor reading @temp.
 */
static struct rk3026_ebc *t_temp_ebc(struct kunit *test, struct t_drives *d,
				     u32 hot, int temp)
{
	static const struct t_rkf_seg segs[] = {
		{ T_RKF_RESET, 5 }, { T_RKF_GRAY16, 1 }, { T_RKF_GRAY2, 1 },
	};
	struct rk3026_ebc *ebc = t_driven_ebc(test, d, false);

	ebc->waveform = t_rkf_wf_hot(test, ebc->dev, segs, ARRAY_SIZE(segs),
				     0, hot, 20);
	d->temp = temp;
	kunit_activate_static_stub(test, rk3026_ebc_read_temp, t_temp);
	return ebc;
}

/*
 * The repair is chosen at the sensor's temperature, read before the
 * take: with a RESET only from 20 degrees up and the sensor at 5, an
 * mxcfb DU send on a glass not known is driven FULL, no RESET before it
 * (the fallback temperature, 25, has one); at 25 the RESET runs and the
 * send after it is drawn PART.
 */
static void ebc_test_glue_repair_sensor(struct kunit *test)
{
	struct drm_rect r = DRM_RECT_INIT(2, 2, 4, 2);
	struct t_drives d;
	struct rk3026_ebc *ebc = t_temp_ebc(test, &d, T_RKF_RESET, 5);
	struct rk3026_ebc_last last;

	ebc->shadows.prev_unknown = true;
	t_send(ebc, &r, EBC_MODE_BLACK_WHITE, 0, EBC_SOURCE_MXCFB);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.temp, 5);
	KUNIT_EXPECT_FALSE(test, last.repaired);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_ASSERT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_REPAIR_FLOOR);
	KUNIT_EXPECT_FALSE(test, ebc->shadows.prev_unknown);

	d.temp = 25;
	ebc->shadows.prev_unknown = true;
	t_send(ebc, &r, EBC_MODE_BLACK_WHITE, 0, EBC_SOURCE_MXCFB);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.temp, 25);
	KUNIT_EXPECT_TRUE(test, last.repaired);
	KUNIT_ASSERT_EQ(test, d.n, 3);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, d.mode[2], EBC_MODE_PART);
	KUNIT_EXPECT_FALSE(test, ebc->shadows.prev_unknown);
}

/*
 * Neither a RESET nor a FULL at the temperature (gray2 alone): nothing can
 * make the glass known, so no repair is due. An mxcfb DU send is drawn in
 * its own mode, no PART floor, and the mark stays; the same send again is
 * not skipped: the glass is still not known.
 */
static void ebc_test_glue_repair_no_full(struct kunit *test)
{
	static const struct t_rkf_seg segs[] = { { T_RKF_GRAY2, 1 } };
	struct drm_rect r = DRM_RECT_INIT(2, 2, 4, 2);
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, false);
	struct rk3026_ebc_last last;
	u32 seq;

	ebc->waveform = t_rkf_wf(test, ebc->dev, segs, ARRAY_SIZE(segs), 0);
	ebc->shadows.prev_unknown = true;
	t_img_fill(ebc->next_shadow, 0);
	seq = t_send(ebc, &r, EBC_MODE_BLACK_WHITE, 0, EBC_SOURCE_MXCFB);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_FALSE(test, last.repaired);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_EXPLICIT);
	KUNIT_ASSERT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_TRUE(test, ebc->shadows.prev_unknown);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));

	t_send(ebc, &r, EBC_MODE_BLACK_WHITE, 0, EBC_SOURCE_MXCFB);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, d.n, 2);
	KUNIT_EXPECT_TRUE(test, ebc->shadows.prev_unknown);
}

/*
 * A part carried for want of a LUT is retried at the temperature of its
 * try: with gray16 only from 20 degrees up and the sensor at 5, an mxcfb
 * PART send is carried; the sensor reads 25 by its retry, 100 ms later,
 * and the retry draws it.
 */
static void ebc_test_glue_retry_temp(struct kunit *test)
{
	struct drm_rect r = DRM_RECT_INIT(2, 2, 4, 2);
	struct t_drives d;
	struct rk3026_ebc *ebc = t_temp_ebc(test, &d, T_RKF_GRAY16, 5);
	struct rk3026_ebc_last last;
	ktime_t t0 = ms_to_ktime(1000);
	u32 seq;

	d.now = t0;
	kunit_activate_static_stub(test, rk3026_ebc_clock, t_clock);
	t_img_fill(ebc->next_shadow, 0);
	seq = t_send(ebc, &r, EBC_MODE_PART, 0, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_EQ(test, last.temp, 5);
	KUNIT_EXPECT_TRUE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));

	d.temp = 25;
	d.now = ktime_add_ms(t0, 100);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.temp, 25);
	KUNIT_EXPECT_EQ(test, last.tries, 1);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_PART);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
}

/*
 * A RESET sent while the rails come up for a part drops the carry
 * (rk3026_ebc_pending_add()). The look read no sensor: the carry brought a
 * temperature, 30 degrees, and gave it to the part. The part taken in its
 * place has none, so the sensor is read after all, and the part is driven
 * at the sensor's 5 degrees, not at the fallback's 25. With @pre, an mxcfb
 * send with no temperature is taken, the carry was merged into it at the
 * look; without, the carry's try alone, and the RESET is taken instead.
 */
static void t_reset_drops_carry_temp(struct kunit *test, bool pre)
{
	struct drm_rect r = DRM_RECT_INIT(2, 2, 4, 2);
	struct rk3026_ebc_update upd = t_upd(&r, EBC_MODE_PART, 0,
					     EBC_SOURCE_MXCFB);
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	struct rk3026_ebc_last last;
	ktime_t t0 = d.now;

	d.temp = 5;
	kunit_activate_static_stub(test, rk3026_ebc_read_temp, t_temp);
	d.rails_ret = -EIO;
	upd.temp = 30;
	upd.temp_valid = true;
	t_img_fill(ebc->next_shadow, 0);
	mutex_lock(&ebc->image_lock);
	rk3026_ebc_add_update(ebc, &upd);
	mutex_unlock(&ebc->image_lock);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_RAILS);
	KUNIT_ASSERT_TRUE(test, ebc->pending.carry.temp_valid);

	d.rails_ret = 0;
	d.rails = 0;
	d.rails_reset = true;
	d.now = ktime_add_ms(t0, 100);
	if (pre)
		t_send(ebc, &r, EBC_MODE_PART, 0, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_EQ(test, last.asked_mode,
			pre ? EBC_MODE_PART : EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_TRUE(test, last.temp_read);
	KUNIT_EXPECT_EQ(test, last.temp, 5);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
}

static void ebc_test_glue_reset_drops_carry_temp(struct kunit *test)
{
	t_reset_drops_carry_temp(test, false);
}

static void ebc_test_glue_reset_drops_carry_temp_pre(struct kunit *test)
{
	t_reset_drops_carry_temp(test, true);
}

/*
 * The wait bound (rk3026_ebc_wait_timeout_ms()) against the worst schedule
 * on the fake clock: the three parts ahead of an update take their longest
 * (rk3026_ebc_part_max_ms()), its own part
 * waits the longest gap (a cap set above EBC_TAKE_CAP_MAX_MS, a sender
 * busy until the previous part's finish), and every
 * try is a repair RESET cut short: the first and the last at a part's
 * longest, the one between ending just before the last retry is due. The
 * give-up ends the wait, at the bound and not after it.
 */
static void ebc_test_glue_wait_bound(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_last last;
	ktime_t t0 = d.now, first;
	unsigned int part_ms = rk3026_ebc_part_max_ms(ebc);
	u32 seq;

	kunit_activate_static_stub(test, rk3026_ebc_drive, t_drive_timeout);
	ebc->take_quiet_us = U32_MAX;
	ebc->take_cap_ms = 60000;
	ebc->shadows.prev_unknown = true;
	d.drive_time[0] = ms_to_ktime(part_ms);
	d.drive_time[1] = ms_to_ktime(EBC_RETRY_SPAN_MS - 100 - 1);
	d.drive_time[2] = ms_to_ktime(part_ms);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	seq = ebc->pending.queued_seq;
	/*
	 * In flight, before a RESET, the RESET: done at their longest; a
	 * sender still busy then keeps the gap open to its cap.
	 */
	d.now = ktime_add_ms(t0, 3 * part_ms);
	ebc->take_done = d.now;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_EQ(test, ebc->gap.at,
			ktime_add_ms(d.now, EBC_TAKE_CAP_MAX_MS));
	d.now = ebc->gap.at;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_TRUE(test, last.repaired);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_TIMEOUT);
	KUNIT_EXPECT_TRUE(test, ebc->pending.retry.armed);
	first = d.now;

	while (ebc->pending.retry.armed) {
		KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending,
								 seq));
		d.now = ebc->pending.retry.at;
		KUNIT_ASSERT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
		rk3026_ebc_do_refresh(ebc, &last);
	}
	KUNIT_EXPECT_EQ(test, d.n, 3);
	KUNIT_EXPECT_EQ(test, last.tries, 2);
	KUNIT_EXPECT_TRUE(test, last.gave_up);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_EQ(test, ktime_ms_delta(d.now, first),
			EBC_RETRY_SPAN_MS + part_ms);
	KUNIT_EXPECT_LE(test, ktime_ms_delta(d.now, t0),
			(s64)rk3026_ebc_wait_timeout_ms(ebc));
}

/*
 * A part whose target is what the glass shows is still driven when it is
 * exempt from the skip, so it gets the rails: a full-class mxcfb send (the
 * flash its client asked for), and, with the glass not known, a plane
 * update, whose look opens a gap and raises them there.
 */
static void ebc_test_glue_rails_exempt(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	u32 *white = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;

	t_send(ebc, &t_panel, EBC_MODE_FULL, 0, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.changed_px, 0);
	KUNIT_EXPECT_EQ(test, d.rails, 1);

	ebc->epd_on = false;
	ebc->rails_on = false;
	ebc->shadows.prev_unknown = true;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&t_panel));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_TRUE(test, ebc->gap.open);
	KUNIT_EXPECT_EQ(test, d.rails, 2);
	KUNIT_EXPECT_TRUE(test, ebc->epd_on);
	d.now = ebc->gap.at;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_TRUE(test, last.repaired);
	KUNIT_EXPECT_EQ(test, d.n, 3);
}

/*
 * A repair RESET cut short carries the part (its own waveform never ran),
 * the glass still not known, and its update waits; the retry repairs the
 * glass, draws the part, and the update finishes.
 */
static void ebc_test_glue_retry_repair(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	struct drm_rect a = DRM_RECT_INIT(0, 0, 4, 2);
	struct rk3026_ebc_last last;
	ktime_t t0 = d.now;
	u32 seq;

	kunit_activate_static_stub(test, rk3026_ebc_drive, t_drive_timeout);
	ebc->shadows.prev_unknown = true;
	t_img_fill(ebc->next_shadow, 0);
	seq = t_send(ebc, &a, EBC_MODE_PART, 3, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_TRUE(test, last.repaired);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_TIMEOUT);
	KUNIT_EXPECT_FALSE(test, last.gave_up);
	KUNIT_ASSERT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_RESET);
	KUNIT_EXPECT_TRUE(test, ebc->shadows.prev_unknown);
	KUNIT_EXPECT_TRUE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_TRUE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));

	kunit_activate_static_stub(test, rk3026_ebc_drive, t_drive_ok);
	d.now = ktime_add_ms(t0, 100);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_TRUE(test, last.repaired);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.tries, 1);
	KUNIT_EXPECT_EQ(test, last.marker, 3);
	KUNIT_ASSERT_EQ(test, d.n, 3);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, d.mode[2], EBC_MODE_PART);
	KUNIT_EXPECT_FALSE(test, ebc->shadows.prev_unknown);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_FALSE(test, ebc->pending.retry.armed);
}

/*
 * While a failure lasts (a carry was given up, none driven since), a plane
 * update with the rails down still opens a gap, whose raise fails: the
 * part is taken at once, not asked for the rails again, and given up at
 * once, without retries; its update finishes.
 */
static void ebc_test_glue_retry_failing(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_last last;
	u32 seq;

	d.rails_ret = -EIO;
	ebc->pending.retry.failing = true;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	seq = ebc->pending.queued_seq;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_RAILS);
	KUNIT_EXPECT_TRUE(test, last.gave_up);
	KUNIT_EXPECT_EQ(test, last.tries, 0);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_FALSE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_TRUE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_EQ(test, ebc->stats.carry_failed, 1);
}

/*
 * The refresh thread's look between parts on the fake clock, with no gap
 * open, as ebc_refresh_thread() makes it: whether its wait wakes.
 */
static bool t_woken(struct rk3026_ebc *ebc, const struct t_drives *d)
{
	return rk3026_ebc_thread_woken(&ebc->pending, d->now, ebc->crtc_on,
				       ebc->suspended);
}

/*
 * Rails that fail once. A plane update with the rails down opens the take's
 * gap, whose raise fails: the part is taken at once and carried without
 * asking for the rails again, its update not finished. Nothing wakes the
 * thread until the retry, 100 ms later, which opens no gap, brings the
 * rails up and draws the part; its update finishes then.
 */
static void ebc_test_glue_retry_once(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_pass pass;
	struct rk3026_ebc_last last;
	ktime_t t0 = d.now;
	long timeout;
	u32 seq;

	d.rails_ret = -EIO;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	seq = ebc->pending.queued_seq;
	KUNIT_EXPECT_TRUE(test, t_woken(ebc, &d));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_RAILS);
	KUNIT_EXPECT_EQ(test, last.tries, 0);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_TRUE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_EQ(test, ebc->pending.retry.at, ktime_add_ms(t0, 100));
	rk3026_ebc_thread_pass(ebc, jiffies, &pass);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_thread_step(&pass, &timeout),
			EBC_STEP_WAIT);
	KUNIT_EXPECT_EQ(test, timeout, (long)msecs_to_jiffies(100));

	d.now = ktime_add_ms(t0, 99);
	KUNIT_EXPECT_FALSE(test, t_woken(ebc, &d));
	rk3026_ebc_thread_pass(ebc, jiffies, &pass);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_thread_step(&pass, &timeout),
			EBC_STEP_WAIT);
	KUNIT_EXPECT_EQ(test, timeout, (long)msecs_to_jiffies(1));
	d.now = ktime_add_ms(t0, 100);
	KUNIT_EXPECT_TRUE(test, t_woken(ebc, &d));
	rk3026_ebc_thread_pass(ebc, jiffies, &pass);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_thread_step(&pass, &timeout),
			EBC_STEP_LOOK);

	d.rails_ret = 0;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.seq, seq);
	KUNIT_EXPECT_EQ(test, last.tries, 1);
	KUNIT_EXPECT_EQ(test, last.gap_us, 0);
	KUNIT_EXPECT_FALSE(test, last.gave_up);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&last.clip, &t_panel));
	KUNIT_EXPECT_EQ(test, d.rails, 2);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_FALSE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_FALSE(test, t_woken(ebc, &d));
	KUNIT_EXPECT_EQ(test, ebc->stats.carry_failed, 0);
}

/*
 * Rails that stay down. An mxcfb send (no gap) is carried at 1 s and tried
 * at 1.1, 1.5 and 3.1 s, the thread asleep between; a send at 1.2 s joins
 * it and is tried at once, moving neither. The try at 3.1 s gives both up:
 * their waits end (a marker wait returns 0), carry_failed counts it, and
 * the carry is kept for the next part, which gives up at once. Once the
 * rails are back, that part draws it and the failure ends.
 */
static void ebc_test_glue_retry_give_up(struct kunit *test)
{
	static const s64 tries_ms[] = { 100, 200, 500, EBC_RETRY_SPAN_MS };
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	struct drm_rect a = DRM_RECT_INIT(0, 0, 4, 2);
	struct drm_rect b = DRM_RECT_INIT(8, 4, 2, 2);
	struct rk3026_ebc_last last;
	ktime_t t0 = d.now;
	u32 s1, s2 = 0, s3;
	unsigned int i;

	d.rails_ret = -EIO;
	t_img_fill(ebc->next_shadow, 0);
	s1 = t_send(ebc, &a, EBC_MODE_PART, 5, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_RAILS);
	KUNIT_EXPECT_FALSE(test, last.gave_up);

	for (i = 0; i < ARRAY_SIZE(tries_ms); i++) {
		ktime_t at = ktime_add_ms(t0, tries_ms[i]);

		if (i == 1) {
			d.now = at;
			s2 = t_send(ebc, &b, EBC_MODE_PART, 6,
				    EBC_SOURCE_MXCFB);
		} else {
			KUNIT_EXPECT_EQ(test, ebc->pending.retry.at, at);
			d.now = ktime_sub_ns(at, 1);
			KUNIT_EXPECT_FALSE(test, t_woken(ebc, &d));
			d.now = at;
		}
		KUNIT_EXPECT_TRUE(test, t_woken(ebc, &d));
		KUNIT_EXPECT_FALSE(test,
				   rk3026_ebc_seq_finished(&ebc->pending, s1));
		KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
		KUNIT_EXPECT_FALSE(test, ebc->gap.open);
		rk3026_ebc_do_refresh(ebc, &last);
		KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_RAILS);
		KUNIT_EXPECT_EQ(test, last.tries, i + 1);
		/* The carry's own record: the newest send's marker. */
		KUNIT_EXPECT_EQ(test, last.marker, i ? 6 : 5);
		KUNIT_EXPECT_EQ(test, last.source, EBC_SOURCE_MXCFB);
	}
	KUNIT_EXPECT_TRUE(test, last.gave_up);
	KUNIT_EXPECT_EQ(test, last.seq, s2);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, s1));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, s2));
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_marker(ebc, 5), 0);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_marker(ebc, 6), 0);
	KUNIT_EXPECT_EQ(test, ebc->stats.carry_failed, 1);
	KUNIT_EXPECT_EQ(test, d.rails, 5);
	KUNIT_EXPECT_EQ(test, d.n, 0);
	KUNIT_EXPECT_TRUE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_FALSE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_FALSE(test, t_woken(ebc, &d));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_work_pending(&ebc->pending));

	d.now = ktime_add_ms(t0, 4000);
	s3 = t_send(ebc, &a, EBC_MODE_PART, 7, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_RAILS);
	KUNIT_EXPECT_EQ(test, last.tries, 0);
	KUNIT_EXPECT_TRUE(test, last.gave_up);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, s3));
	KUNIT_EXPECT_FALSE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_EQ(test, ebc->stats.carry_failed, 2);
	KUNIT_EXPECT_EQ(test, d.rails, 6);

	d.rails_ret = 0;
	t_send(ebc, &a, EBC_MODE_PART, 8, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.clip.x2, 10);
	KUNIT_EXPECT_FALSE(test, last.gave_up);
	KUNIT_EXPECT_FALSE(test, ebc->pending.retry.failing);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_EQ(test, d.n, 1);
}

/*
 * A CRTC disable while a carry waits for its retry: the waiting thread
 * wakes at once (ebc_crtc_atomic_disable() wakes it), takes the carry and
 * drops it, so the disable's wait on the newest seq ends without the
 * retry's delay, and without a give-up.
 */
static void ebc_test_glue_retry_crtc_off(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	struct rk3026_ebc_last last;
	u32 seq;

	d.rails_ret = -EIO;
	t_img_fill(ebc->next_shadow, 0);
	seq = t_send(ebc, &t_panel, EBC_MODE_FULL, 0, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_RAILS);
	KUNIT_EXPECT_FALSE(test, t_woken(ebc, &d));
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_seq(ebc, seq, 0, false),
			-ETIMEDOUT);

	t_crtc_off(test, ebc);
	KUNIT_EXPECT_TRUE(test, t_woken(ebc, &d));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_CRTC_OFF);
	KUNIT_EXPECT_EQ(test, last.seq, seq);
	KUNIT_EXPECT_FALSE(test, last.gave_up);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_wait_seq(ebc, seq, 0, false), 0);
	KUNIT_EXPECT_FALSE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_FALSE(test, t_woken(ebc, &d));
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_EQ(test, ebc->stats.carry_failed, 0);
}

/*
 * A system suspend holds the pending set for the resume (ebc_suspend()).
 * A plane update over the whole panel with the rails down: while suspended
 * the thread's wait does not wake for it and its pass only waits; a look
 * ends at once, with no gap and no rails, and takes nothing, so the update
 * is not finished and the glass is as it was. An update sent meanwhile
 * joins the part. After the resume the part is taken with both and driven
 * from the kept glass, with no RESET in front of it.
 */
static void ebc_test_glue_sleep_holds(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	u32 *black = t_plane_fb(test, 0);
	struct drm_rect r = DRM_RECT_INIT(2, 1, 4, 2);
	struct rk3026_ebc_last last = { .outcome = EBC_OUTCOME_NONE };
	struct rk3026_ebc_pass pass;
	long timeout;
	u32 seq;

	ebc->du_cleanup_area = 2 * T_W * T_H;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	seq = ebc->pending.queued_seq;
	rk3026_ebc_sleep_begin(ebc);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_sleep_wait(ebc, 0), 0);
	KUNIT_EXPECT_FALSE(test, t_woken(ebc, &d));
	rk3026_ebc_thread_pass(ebc, jiffies, &pass);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_thread_step(&pass, &timeout),
			EBC_STEP_WAIT);
	KUNIT_EXPECT_EQ(test, timeout, MAX_SCHEDULE_TIMEOUT);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NONE);
	KUNIT_EXPECT_EQ(test, d.rails, 0);
	KUNIT_EXPECT_EQ(test, d.n, 0);
	KUNIT_EXPECT_FALSE(test, ebc->in_flight);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&ebc->pending), 1);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	t_expect_white(test, ebc->shadows.prev);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&r));
	seq = ebc->pending.queued_seq;
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&ebc->pending), 1);

	rk3026_ebc_sleep_end(ebc);
	KUNIT_EXPECT_TRUE(test, t_woken(ebc, &d));
	/* Past the gap's cap: the look takes the part. */
	d.now = ktime_add_ms(d.now, 1000);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.seq, seq);
	KUNIT_EXPECT_EQ(test, last.merged, 2);
	KUNIT_ASSERT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.mode[0], EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_EQ(test, ebc->shadows.prev[0], 0);
}

/*
 * A suspend that begins while the rails come up for a part (t_rails() with
 * @rails_suspend): the part is not taken. It stays pending, its update not
 * finished, nothing is driven, and the rails stay up for the suspend to
 * take down. After the resume it is taken and drawn, a first try still.
 */
static void ebc_test_glue_sleep_rails_ramp(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_rails_ebc(test, &d);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_last last = { .outcome = EBC_OUTCOME_NONE };
	u32 seq;

	ebc->du_cleanup_area = 2 * T_W * T_H;
	d.rails_suspend = true;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	seq = ebc->pending.queued_seq;
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_TRUE(test, ebc->suspended);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NONE);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_TRUE(test, ebc->epd_on);
	KUNIT_EXPECT_EQ(test, d.n, 0);
	KUNIT_EXPECT_FALSE(test, ebc->in_flight);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_sleep_wait(ebc, 0), 0);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&ebc->pending), 1);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	t_expect_white(test, ebc->shadows.prev);

	rk3026_ebc_sleep_end(ebc);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.seq, seq);
	KUNIT_EXPECT_EQ(test, last.tries, 0);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.rails, 2);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_EQ(test, ebc->shadows.prev[0], 0);
}

/*
 * rk3026_ebc_drive() with a suspend that begins in the middle of the
 * waveform: the part is in flight, so the suspend's wait does not end, at
 * once or after a while; the waveform then runs to its end (t_drive_ok()).
 */
static int t_drive_suspend(struct rk3026_ebc *ebc, const u8 *lut, u32 frames,
			   u32 mode, u32 lut_mode,
			   struct rk3026_ebc_last *last)
{
	struct kunit *test = kunit_get_current_test();

	KUNIT_EXPECT_TRUE(test, READ_ONCE(ebc->in_flight));
	rk3026_ebc_sleep_begin(ebc);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_sleep_wait(ebc, 0), -EBUSY);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_sleep_wait(ebc, 10), -EBUSY);
	return t_drive_ok(ebc, lut, frames, mode, lut_mode, last);
}

/*
 * A suspend that begins while a part is driven: the waveform is not cut,
 * the part finishes as driven, the glass follows it, and the suspend's
 * wait ends then. An update sent after it stays pending until the resume,
 * which draws it.
 */
static void ebc_test_glue_sleep_mid_waveform(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	u32 *black = t_plane_fb(test, 0);
	u32 *white = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last;
	u32 seq;

	ebc->du_cleanup_area = 2 * T_W * T_H;
	kunit_activate_static_stub(test, rk3026_ebc_drive, t_drive_suspend);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	seq = ebc->pending.queued_seq;
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_TRUE(test, ebc->suspended);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_FALSE(test, ebc->in_flight);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_sleep_wait(ebc, 0), 0);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_EQ(test, ebc->shadows.prev[0], 0);

	kunit_activate_static_stub(test, rk3026_ebc_drive, t_drive_ok);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&t_panel));
	seq = ebc->pending.queued_seq;
	last.outcome = EBC_OUTCOME_NONE;
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NONE);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_EQ(test, ebc->shadows.prev[0], 0);

	rk3026_ebc_sleep_end(ebc);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.seq, seq);
	KUNIT_EXPECT_EQ(test, d.n, 2);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_EQ(test, ebc->shadows.prev[0], 0xff);
}

/*
 * A carry and its retry across a suspend: rails that fail carry a part,
 * its retry 100 ms on. A suspend begins; at the retry's time the thread
 * does not wake and its pass only waits, and a look takes nothing, so the
 * carry, its retry and its update's wait stay as they were. After the
 * resume the retry is due: the part is taken and drawn, its second try,
 * at the temperature the sensor reads then: a carry keeps only one an
 * update brought (an mxcfb send's), and the sensor is read for every try.
 */
static void ebc_test_glue_sleep_carry(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_pass pass;
	struct rk3026_ebc_last last;
	ktime_t t0 = d.now;
	long timeout;
	u32 seq;

	kunit_activate_static_stub(test, rk3026_ebc_read_temp, t_temp);
	d.temp = 10;
	d.rails_ret = -EIO;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	seq = ebc->pending.queued_seq;
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_RAILS);
	KUNIT_ASSERT_TRUE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.temp_valid);

	rk3026_ebc_sleep_begin(ebc);
	d.now = ktime_add_ms(t0, 100);
	KUNIT_EXPECT_FALSE(test, t_woken(ebc, &d));
	rk3026_ebc_thread_pass(ebc, jiffies, &pass);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_thread_step(&pass, &timeout),
			EBC_STEP_WAIT);
	KUNIT_EXPECT_EQ(test, timeout, MAX_SCHEDULE_TIMEOUT);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	last.outcome = EBC_OUTCOME_NONE;
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NONE);
	KUNIT_EXPECT_TRUE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_TRUE(test, ebc->pending.retry.armed);
	KUNIT_EXPECT_EQ(test, ebc->pending.retry.at, ktime_add_ms(t0, 100));
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_EQ(test, d.rails, 1);

	d.rails_ret = 0;
	d.temp = 30;
	rk3026_ebc_sleep_end(ebc);
	KUNIT_EXPECT_TRUE(test, t_woken(ebc, &d));
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.seq, seq);
	KUNIT_EXPECT_EQ(test, last.tries, 1);
	KUNIT_EXPECT_TRUE(test, last.temp_read);
	KUNIT_EXPECT_EQ(test, last.temp, 30);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.rails, 2);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_FALSE(test, ebc->pending.carry.merged);
}

/*
 * A suspend that begins while the take's gap is open and its rails come
 * up (rk3026_ebc_take_ready(), t_rails() with @rails_suspend): the look
 * keeps the gap open, as it would. While suspended the thread's pass
 * looks at the rails alone, the open gap holding them no longer: it waits
 * for their deadline, then switches them off. The next look ends the gap
 * and the take takes nothing; no part is in flight, so the suspend's wait
 * ends at once, and the update stays pending. After the resume, with the
 * rails down as the suspend left them and the quiet gap over, the part is
 * taken at once and drawn, the rails raised for it.
 */
static void ebc_test_glue_sleep_gap_ramp(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_gap_ebc(test, &d);
	u32 *fb = t_plane_fb(test, 0xffffff);
	struct drm_rect a = DRM_RECT_INIT(4, 2, 2, 2);
	struct rk3026_ebc_last last = { .outcome = EBC_OUTCOME_NONE };
	struct rk3026_ebc_pass pass;
	long timeout;
	u32 seq;

	d.ramp = ms_to_ktime(42);
	d.rails_suspend = true;
	t_fb_box(fb, 4, 2, 6, 4, 0);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, fb, T_W * 4, &a));
	seq = ebc->pending.queued_seq;
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_TRUE(test, ebc->suspended);
	KUNIT_EXPECT_TRUE(test, ebc->gap.open);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_TRUE(test, ebc->epd_on);

	KUNIT_EXPECT_FALSE(test, t_woken(ebc, &d));
	rk3026_ebc_thread_pass(ebc, jiffies, &pass);
	KUNIT_EXPECT_TRUE(test, pass.gap_open);
	KUNIT_EXPECT_TRUE(test, pass.rails_on);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_thread_step(&pass, &timeout),
			EBC_STEP_WAIT);
	KUNIT_EXPECT_GT(test, timeout, 0);
	KUNIT_EXPECT_LE(test, timeout, (long)msecs_to_jiffies(200));
	pass.now = pass.rails_off_at;
	KUNIT_EXPECT_EQ(test, rk3026_ebc_thread_step(&pass, &timeout),
			EBC_STEP_RAILS_OFF);

	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NONE);
	KUNIT_EXPECT_EQ(test, d.n, 0);
	KUNIT_EXPECT_EQ(test, d.rails, 1);
	KUNIT_EXPECT_FALSE(test, ebc->in_flight);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_sleep_wait(ebc, 0), 0);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&ebc->pending), 1);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));

	/* The suspend switched the rails off (ebc_suspend()). */
	ebc->epd_on = false;
	ebc->rails_on = false;
	rk3026_ebc_sleep_end(ebc);
	KUNIT_EXPECT_TRUE(test, t_woken(ebc, &d));
	/* The quiet gap after the update is over: taken at once. */
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_take_ready(ebc, d.now));
	KUNIT_EXPECT_FALSE(test, ebc->gap.open);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.seq, seq);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_EQ(test, d.rails, 2);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
}

/*
 * A hibernation restore (rk3026_ebc_sleep_restore()): the glass may have
 * been drawn on since the image was saved. A black panel driven and
 * known; the restore marks the glass not known and queues the whole
 * panel, FULL, which a take while suspended does not take. After the
 * resume the part repairs first, a RESET, then draws the target as it
 * was, FULL; the glass is known again and the target unchanged, and no
 * plane update has to redraw the panel. With the CRTC off the restore
 * marks the glass and queues nothing.
 */
static void ebc_test_glue_sleep_restore(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_last last = { .outcome = EBC_OUTCOME_NONE };
	struct rk3026_ebc_set part;
	u32 seq;

	ebc->du_cleanup_area = 2 * T_W * T_H;
	KUNIT_ASSERT_TRUE(test, ebc->crtc_on);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_ASSERT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_ASSERT_EQ(test, d.n, 1);
	KUNIT_EXPECT_FALSE(test, ebc->shadows.prev_unknown);
	ebc->plane_resync = false;

	rk3026_ebc_sleep_begin(ebc);
	rk3026_ebc_sleep_restore(ebc);
	seq = ebc->pending.queued_seq;
	KUNIT_EXPECT_TRUE(test, ebc->shadows.prev_unknown);
	KUNIT_EXPECT_FALSE(test, ebc->plane_resync);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_pending_peek(&ebc->pending, &part));
	KUNIT_EXPECT_EQ(test, part.source, EBC_SOURCE_RESTORE);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&part.box, &t_panel));
	KUNIT_EXPECT_FALSE(test, part.auto_ok);
	last.outcome = EBC_OUTCOME_NONE;
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NONE);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));

	rk3026_ebc_sleep_end(ebc);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, last.seq, seq);
	KUNIT_EXPECT_TRUE(test, last.repaired);
	KUNIT_ASSERT_EQ(test, d.n, 3);
	KUNIT_EXPECT_EQ(test, d.mode[1], EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, d.mode[2], EBC_MODE_FULL);
	KUNIT_EXPECT_FALSE(test, ebc->shadows.prev_unknown);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, seq));
	KUNIT_EXPECT_EQ(test, ebc->shadows.prev[0], 0);
	KUNIT_EXPECT_EQ(test, ebc->shadows.prev[T_SIZE - 1], 0);
	KUNIT_EXPECT_FALSE(test, ebc->plane_resync);

	rk3026_ebc_crtc_off(ebc);
	rk3026_ebc_sleep_begin(ebc);
	rk3026_ebc_sleep_restore(ebc);
	KUNIT_EXPECT_TRUE(test, ebc->shadows.prev_unknown);
	KUNIT_EXPECT_EQ(test, ebc->pending.queued_seq, seq);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&ebc->pending), 0);
	rk3026_ebc_sleep_end(ebc);
}

/*
 * rk3026_ebc_drive() for ebc_test_glue_sleep_crtc_wait(): a suspend begins
 * in the middle of the waveform, and a CRTC wait on the part's own seq
 * does not end while the part is in flight; the waveform then runs to its
 * end (t_drive_ok()).
 */
static int t_drive_crtc_wait(struct rk3026_ebc *ebc, const u8 *lut,
			     u32 frames, u32 mode, u32 lut_mode,
			     struct rk3026_ebc_last *last)
{
	struct kunit *test = kunit_get_current_test();

	rk3026_ebc_sleep_begin(ebc);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_crtc_wait(ebc, last->seq, 0),
			-ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_crtc_wait(ebc, last->seq, 10),
			-ETIMEDOUT);
	return t_drive_ok(ebc, lut, frames, mode, lut_mode, last);
}

/*
 * The wait of a CRTC enable or disable (rk3026_ebc_crtc_wait()). Awake it
 * waits for its update. Asleep it waits for the part in flight, if one
 * is, and no longer: not for updates that stay pending until the resume,
 * nor for the seq of a CRTC disable, which would block the commit for the
 * whole bound after a resume that found no clocks. After the resume the
 * thread drops what the CRTC off left pending, and the wait ends with
 * its update.
 */
static void ebc_test_glue_sleep_crtc_wait(struct kunit *test)
{
	struct t_drives d;
	struct rk3026_ebc *ebc = t_driven_ebc(test, &d, true);
	u32 *black = t_plane_fb(test, 0);
	u32 *white = t_plane_fb(test, 0xffffff);
	struct rk3026_ebc_last last = { .outcome = EBC_OUTCOME_NONE };
	u32 seq, seq2, off;

	ebc->du_cleanup_area = 2 * T_W * T_H;
	kunit_activate_static_stub(test, rk3026_ebc_drive, t_drive_crtc_wait);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	seq = ebc->pending.queued_seq;
	KUNIT_EXPECT_EQ(test, rk3026_ebc_crtc_wait(ebc, seq, 0), -ETIMEDOUT);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_DRIVEN);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_TRUE(test, ebc->suspended);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_crtc_wait(ebc, seq, 0), 0);

	kunit_activate_static_stub(test, rk3026_ebc_drive, t_drive_ok);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, white, T_W * 4,
							&t_panel));
	seq2 = ebc->pending.queued_seq;
	KUNIT_EXPECT_EQ(test, rk3026_ebc_crtc_wait(ebc, seq2, 0), 0);
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_seq_finished(&ebc->pending, seq2));
	off = rk3026_ebc_crtc_off(ebc);
	KUNIT_EXPECT_EQ(test, off, seq2);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_crtc_wait(ebc, off, 0), 0);

	rk3026_ebc_sleep_end(ebc);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_crtc_wait(ebc, off, 0), -ETIMEDOUT);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, d.n, 1);
	KUNIT_EXPECT_TRUE(test, rk3026_ebc_seq_finished(&ebc->pending, off));
	KUNIT_EXPECT_EQ(test, rk3026_ebc_crtc_wait(ebc, off, 0), 0);
}

#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_MXCFB)
/* The part one mxcfb send queued, taken as the refresh thread takes it. */
static int t_mxcfb_part(struct rk3026_ebc *ebc, const u32 *src,
			const struct mxcfb_update_data *req,
			struct rk3026_ebc_set *part)
{
	int ret = rk3026_ebc_mxcfb_send(ebc, src, T_W * 4, req);

	memset(part, 0, sizeof(*part));
	mutex_lock(&ebc->image_lock);
	rk3026_ebc_pending_take(&ebc->pending, part);
	mutex_unlock(&ebc->image_lock);
	return ret;
}

/*
 * A send from the request to the pending set: the region clamped to the
 * panel, the waveform and the flags mapped, auto_mode, the marker and the
 * temperature passed on, the client's pixels converted, with the time
 * that took, and the panel claimed from the plane. A RESET converts
 * nothing; a region off the panel and a send with the CRTC off are
 * refused, and queue nothing.
 */
static void ebc_test_glue_mxcfb_send(struct kunit *test)
{
	static const struct drm_rect box = DRM_RECT_INIT(4, 2, 6, 2);
	static const struct drm_rect clamped = DRM_RECT_INIT(12, 4, 4, 2);
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	u32 *black = t_plane_fb(test, 0);
	struct mxcfb_update_data req = {
		.update_region = { .left = 4, .top = 2, .width = 6,
				   .height = 2 },
		.waveform_mode = MXCFB_WAVEFORM_MODE_AUTO,
		.update_mode = MXCFB_UPDATE_MODE_PARTIAL,
		.update_marker = 7,
		.temp = MXCFB_TEMP_USE_AMBIENT,
	};
	struct rk3026_ebc_set part;

	/* AUTO + PARTIAL: PART with auto_ok, the box converted. */
	KUNIT_EXPECT_EQ(test, t_mxcfb_part(ebc, black, &req, &part), 0);
	KUNIT_EXPECT_TRUE(test, ebc->mxcfb_active);
	KUNIT_EXPECT_EQ(test, part.merged, 1);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&part.box, &box));
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_PART);
	KUNIT_EXPECT_TRUE(test, part.auto_ok);
	KUNIT_EXPECT_FALSE(test, part.has_plane);
	KUNIT_EXPECT_FALSE(test, part.has_mxcfb_full);
	KUNIT_EXPECT_EQ(test, part.marker, 7);
	KUNIT_EXPECT_EQ(test, part.source, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_FALSE(test, part.temp_valid);
	KUNIT_EXPECT_EQ(test, t_px(ebc->next_shadow, 4, 2), 0);
	KUNIT_EXPECT_EQ(test, t_px(ebc->next_shadow, 9, 3), 0);
	KUNIT_EXPECT_EQ(test, t_px(ebc->next_shadow, 3, 2), 15);
	KUNIT_EXPECT_EQ(test, t_px(ebc->next_shadow, 10, 2), 15);
	KUNIT_EXPECT_EQ(test, t_px(ebc->next_shadow, 4, 1), 15);
	KUNIT_EXPECT_GE(test, part.blit_us, 1);

	/* FORCE_MONOCHROME: DU, no auto; a temperature of the client's. */
	req.flags = EPDC_FLAG_FORCE_MONOCHROME;
	req.temp = 30;
	KUNIT_EXPECT_EQ(test, t_mxcfb_part(ebc, black, &req, &part), 0);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_FALSE(test, part.auto_ok);
	KUNIT_EXPECT_TRUE(test, part.temp_valid);
	KUNIT_EXPECT_EQ(test, part.temp, 30);

	/* AUTO + FULL: FULL, full class, no auto; clamped to the panel. */
	req.flags = 0;
	req.update_mode = MXCFB_UPDATE_MODE_FULL;
	req.update_region.left = 12;
	req.update_region.top = 4;
	KUNIT_EXPECT_EQ(test, t_mxcfb_part(ebc, black, &req, &part), 0);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_FULL);
	KUNIT_EXPECT_FALSE(test, part.auto_ok);
	KUNIT_EXPECT_TRUE(test, part.has_mxcfb_full);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&part.box, &clamped));

	/* INIT: a RESET, the target white, nothing converted. */
	req.waveform_mode = MXCFB_WAVEFORM_MODE_INIT;
	KUNIT_EXPECT_EQ(test, t_mxcfb_part(ebc, black, &req, &part), 0);
	KUNIT_EXPECT_EQ(test, part.mode, EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, part.blit_us, 0);
	t_expect_white(test, ebc->next_shadow);

	/* Off the panel, then with the CRTC off: refused, nothing queued. */
	req.waveform_mode = MXCFB_WAVEFORM_MODE_GC16;
	req.update_region.left = T_W;
	KUNIT_EXPECT_EQ(test, t_mxcfb_part(ebc, black, &req, &part), -EINVAL);
	KUNIT_EXPECT_EQ(test, part.merged, 0);
	req.update_region.left = 0;
	ebc->crtc_on = false;
	ebc->mxcfb_active = false;
	KUNIT_EXPECT_EQ(test, t_mxcfb_part(ebc, black, &req, &part), -EBUSY);
	KUNIT_EXPECT_EQ(test, part.merged, 0);
	KUNIT_EXPECT_FALSE(test, ebc->mxcfb_active);
	t_expect_white(test, ebc->next_shadow);
	KUNIT_EXPECT_EQ(test, ebc->stats.updates, 4);
}
#endif
/*
 * The CPU time of a part in its record: the blit of its newest update, as
 * the sender measured it, and the snapshot the thread built for it. The
 * fixture cannot drive a waveform, so the run that builds a snapshot and
 * would drive it stops at the missing LUT.
 */
static u32 t_send_blit(struct rk3026_ebc *ebc, u32 mode, u32 blit_us)
{
	struct rk3026_ebc_update upd = t_upd(&t_panel, mode, 0,
					     EBC_SOURCE_PLANE);
	u32 seq;

	upd.blit_us = blit_us;
	mutex_lock(&ebc->image_lock);
	seq = rk3026_ebc_add_update(ebc, &upd);
	mutex_unlock(&ebc->image_lock);
	return seq;
}

static void ebc_test_glue_timing(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	struct rk3026_ebc_last last;

	/* Two updates in one part: the newest one's blit. */
	t_img_fill(ebc->next_shadow, 0);
	t_send_blit(ebc, EBC_MODE_FULL, 7);
	t_send_blit(ebc, EBC_MODE_FULL, 9);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_EQ(test, last.merged, 2);
	KUNIT_EXPECT_EQ(test, last.blit_us, 9);
	KUNIT_EXPECT_GE(test, last.snap_us, 1);
	KUNIT_EXPECT_EQ(test, ebc->last.blit_us, 9);
	KUNIT_EXPECT_EQ(test, ebc->last.snap_us, last.snap_us);
	/* Nothing was driven: no scan record. */
	KUNIT_EXPECT_EQ(test, ebc->last_driven.snap_us, 0);

	/* A carry blitted nothing of its own: the new update's blit. */
	KUNIT_ASSERT_TRUE(test, ebc->pending.carry.merged);
	KUNIT_EXPECT_EQ(test, ebc->pending.carry.blit_us, 0);
	t_send_blit(ebc, EBC_MODE_FULL, 5);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_EQ(test, last.blit_us, 5);
	KUNIT_EXPECT_GE(test, last.snap_us, 1);
}

/* A skipped part had its snapshot built. */
static void ebc_test_glue_timing_skipped(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	struct rk3026_ebc_last last;

	t_send_blit(ebc, EBC_MODE_FULL, 3);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_SKIPPED);
	KUNIT_EXPECT_EQ(test, last.blit_us, 3);
	KUNIT_EXPECT_GE(test, last.snap_us, 1);
}

/* No snapshot with the CRTC off or without a waveform; a RESET blits none. */
static void ebc_test_glue_timing_dropped(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	struct rk3026_ebc_last last;

	t_send_blit(ebc, EBC_MODE_FULL, 4);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_WAVEFORM);
	KUNIT_EXPECT_EQ(test, last.blit_us, 4);
	KUNIT_EXPECT_EQ(test, last.snap_us, 0);

	ebc->crtc_on = false;
	t_send_blit(ebc, EBC_MODE_FULL, 6);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_CRTC_OFF);
	KUNIT_EXPECT_EQ(test, last.blit_us, 6);
	KUNIT_EXPECT_EQ(test, last.snap_us, 0);

	t_send(ebc, &t_panel, EBC_MODE_RESET, 0, EBC_SOURCE_ENABLE);
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_RESET);
	KUNIT_EXPECT_EQ(test, last.blit_us, 0);
	KUNIT_EXPECT_EQ(test, last.snap_us, 0);
}

/* The conversion rk3026_ebc_blit_impl() should name on this kernel and CPU. */
static const char *t_blit_impl(void)
{
#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_NEON)
	if (cpu_has_neon())
		return "neon";
#endif
	return "scalar";
}

/*
 * The blit reports its own time: none for a clip off the panel, at least
 * 1 us for one it converted, carried by a plane update into its part.
 */
static void ebc_test_glue_blit_time(struct kunit *test)
{
	static const struct drm_rect off = DRM_RECT_INIT(T_W, 0, 4, 2);
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	u32 *black = t_plane_fb(test, 0);
	u32 us;

	KUNIT_EXPECT_STREQ(test, rk3026_ebc_blit_impl(), t_blit_impl());

	mutex_lock(&ebc->image_lock);
	us = rk3026_ebc_blit_xrgb8888(ebc, black, T_W * 4, &off);
	mutex_unlock(&ebc->image_lock);
	KUNIT_EXPECT_EQ(test, us, 0);
	t_expect_white(test, ebc->next_shadow);

	mutex_lock(&ebc->image_lock);
	us = rk3026_ebc_blit_xrgb8888(ebc, black, T_W * 4, &t_panel);
	mutex_unlock(&ebc->image_lock);
	KUNIT_EXPECT_GE(test, us, 1);

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&t_panel));
	KUNIT_EXPECT_GE(test, ebc->pending.pre.blit_us, 1);
}

/*
 * The plane box. A plane state just complete enough for the damage helpers,
 * as their own tests build it: a plane with the damage clips property, a
 * framebuffer of the panel's size, the source on all of it.
 */
struct t_damage {
	struct drm_driver driver;
	struct drm_device device;
	struct drm_object_properties props;
	struct drm_plane plane;
	struct drm_property prop;
	struct drm_framebuffer fb;
	struct drm_plane_state old_state;
	struct drm_plane_state state;
	struct drm_property_blob blob;
	struct drm_mode_rect rects[2];
};

static void t_plane_src(struct drm_plane_state *state, int w, int h)
{
	state->src_x = 0;
	state->src_y = 0;
	state->src_w = w << 16;
	state->src_h = h << 16;
	state->src = DRM_RECT_INIT(0, 0, w << 16, h << 16);
}

static struct t_damage *t_damage_alloc(struct kunit *test)
{
	struct t_damage *d = kunit_kzalloc(test, sizeof(*d), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, d);
	d->device.driver = &d->driver;
	d->device.mode_config.prop_fb_damage_clips = &d->prop;
	d->prop.base.id = 1;
	d->prop.dev = &d->device;
	d->plane.dev = &d->device;
	d->plane.base.properties = &d->props;
	drm_plane_enable_fb_damage_clips(&d->plane);

	d->fb.width = T_W;
	d->fb.height = T_H;
	d->old_state.plane = &d->plane;
	d->state.plane = &d->plane;
	d->state.crtc = ZERO_SIZE_PTR;
	d->state.fb = &d->fb;
	d->state.visible = true;
	t_plane_src(&d->old_state, T_W, T_H);
	t_plane_src(&d->state, T_W, T_H);
	return d;
}

/* Damage clips @rects[0..n) on the commit; n == 0 sends an empty list. */
static void t_damage_set(struct t_damage *d, unsigned int n)
{
	d->blob.data = d->rects;
	d->blob.length = n * sizeof(d->rects[0]);
	d->state.fb_damage_clips = &d->blob;
}

static void t_expect_rect(struct kunit *test, const struct drm_rect *r,
			  int x1, int y1, int x2, int y2)
{
	KUNIT_EXPECT_EQ(test, r->x1, x1);
	KUNIT_EXPECT_EQ(test, r->y1, y1);
	KUNIT_EXPECT_EQ(test, r->x2, x2);
	KUNIT_EXPECT_EQ(test, r->y2, y2);
}

static bool t_plane_box(struct t_damage *d, unsigned int width,
			unsigned int height, struct drm_rect *box)
{
	return rk3026_ebc_plane_box(&d->old_state, &d->state, width, height,
				    box);
}

static void ebc_test_plane_box(struct kunit *test)
{
	struct t_damage *d = t_damage_alloc(test);
	struct drm_mode_rect *r = d->rects;
	struct drm_rect box;

	/* No damage clips: the whole plane, the panel. */
	KUNIT_EXPECT_TRUE(test, t_plane_box(d, T_W, T_H, &box));
	t_expect_rect(test, &box, 0, 0, T_W, T_H);

	/* An empty list, or clips outside the plane: nothing to draw. */
	t_damage_set(d, 0);
	KUNIT_EXPECT_FALSE(test, t_plane_box(d, T_W, T_H, &box));
	r[0] = (struct drm_mode_rect) { T_W, 0, T_W + 4, 2 };
	r[1] = (struct drm_mode_rect) { 0, T_H, 4, T_H + 1 };
	t_damage_set(d, 2);
	KUNIT_EXPECT_FALSE(test, t_plane_box(d, T_W, T_H, &box));

	/* One rectangle on even edges: as it is. */
	r[0] = (struct drm_mode_rect) { 2, 1, 6, 3 };
	t_damage_set(d, 1);
	KUNIT_EXPECT_TRUE(test, t_plane_box(d, T_W, T_H, &box));
	t_expect_rect(test, &box, 2, 1, 6, 3);

	/* Odd edges widen to whole bytes, a 1-px clip to its byte. */
	r[0] = (struct drm_mode_rect) { 3, 1, 5, 2 };
	KUNIT_EXPECT_TRUE(test, t_plane_box(d, T_W, T_H, &box));
	t_expect_rect(test, &box, 2, 1, 6, 2);
	r[0] = (struct drm_mode_rect) { T_W - 1, T_H - 1, T_W, T_H };
	KUNIT_EXPECT_TRUE(test, t_plane_box(d, T_W, T_H, &box));
	t_expect_rect(test, &box, T_W - 2, T_H - 1, T_W, T_H);

	/* Several rectangles merge into their bounding box. */
	r[0] = (struct drm_mode_rect) { 1, 0, 3, 1 };
	r[1] = (struct drm_mode_rect) { 10, 4, 13, 6 };
	t_damage_set(d, 2);
	KUNIT_EXPECT_TRUE(test, t_plane_box(d, T_W, T_H, &box));
	t_expect_rect(test, &box, 0, 0, 14, 6);

	/*
	 * Out of range: the plane clips the damage to itself, and the box is
	 * clamped to the panel as well.
	 */
	r[0] = (struct drm_mode_rect) { -4, -1, T_W + 20, T_H + 20 };
	t_damage_set(d, 1);
	KUNIT_EXPECT_TRUE(test, t_plane_box(d, T_W, T_H, &box));
	t_expect_rect(test, &box, 0, 0, T_W, T_H);
	KUNIT_EXPECT_TRUE(test, t_plane_box(d, T_W - 4, T_H - 2, &box));
	t_expect_rect(test, &box, 0, 0, T_W - 4, T_H - 2);
	r[0] = (struct drm_mode_rect) { T_W - 3, 1, T_W, 2 };
	KUNIT_EXPECT_FALSE(test, t_plane_box(d, T_W - 4, T_H, &box));

	/* A new source rectangle (a modeset): the whole plane, clips or not. */
	r[0] = (struct drm_mode_rect) { 2, 1, 6, 3 };
	t_plane_src(&d->old_state, T_W - 2, T_H);
	KUNIT_EXPECT_TRUE(test, t_plane_box(d, T_W, T_H, &box));
	t_expect_rect(test, &box, 0, 0, T_W, T_H);

	/* Not visible: nothing. */
	t_plane_src(&d->old_state, T_W, T_H);
	d->state.visible = false;
	KUNIT_EXPECT_FALSE(test, t_plane_box(d, T_W, T_H, &box));
}

/* Every pixel of @img is @in inside @box and @out outside it. */
static void t_expect_box_img(struct kunit *test, const u8 *img,
			     const struct drm_rect *box, u8 in, u8 out)
{
	int x, y;

	for (y = 0; y < T_H; y++)
		for (x = 0; x < T_W; x++)
			KUNIT_EXPECT_EQ_MSG(test, t_px(img, x, y),
					    t_in_rect(box, x, y) ? in : out,
					    "pixel %d,%d", x, y);
	t_expect_guard(test, img);
}

/* A plane update blits and queues its box alone. */
static void ebc_test_glue_plane_box_update(struct kunit *test)
{
	static const struct drm_rect box = DRM_RECT_INIT(2, 1, 4, 2);
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_set *pre = &ebc->pending.pre;

	t_img_fill(ebc->next_shadow, 3);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&box));
	t_expect_box_img(test, ebc->next_shadow, &box, 0, 3);
	KUNIT_EXPECT_EQ(test, ebc->stats.updates, 1);
	KUNIT_EXPECT_EQ(test, pre->merged, 1);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&pre->box, &box));
	KUNIT_EXPECT_EQ(test, pre->mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, pre->source, EBC_SOURCE_PLANE);
	KUNIT_EXPECT_TRUE(test, pre->auto_ok);
	KUNIT_EXPECT_TRUE(test, pre->has_plane);
	KUNIT_EXPECT_FALSE(test, ebc->plane_resync);
}

/*
 * A plane box merges with an mxcfb part into their bounding box; the part
 * has a plane update in it and is no longer an auto part.
 */
static void ebc_test_glue_plane_box_merge(struct kunit *test)
{
	static const struct drm_rect box = DRM_RECT_INIT(2, 1, 4, 2);
	static const struct drm_rect clip = DRM_RECT_INIT(8, 3, 4, 2);
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_set *pre = &ebc->pending.pre;

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&box));
	t_send(ebc, &clip, EBC_MODE_PART, 0, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_EQ(test, rk3026_ebc_pending_parts(&ebc->pending), 1);
	KUNIT_EXPECT_EQ(test, pre->merged, 2);
	t_expect_rect(test, &pre->box, 2, 1, 12, 5);
	KUNIT_EXPECT_EQ(test, pre->mode, EBC_MODE_FULL);
	KUNIT_EXPECT_FALSE(test, pre->auto_ok);
	KUNIT_EXPECT_TRUE(test, pre->has_plane);
}

/*
 * After another source wrote the target, the next plane update redraws the
 * whole panel, whatever its damage; the one after that its box again. An
 * mxcfb client blocks plane updates with a box too, and the resync stays
 * due.
 */
static void ebc_test_glue_plane_resync(struct kunit *test)
{
	static const struct drm_rect box = DRM_RECT_INIT(2, 1, 4, 2);
	static const struct drm_rect clip = DRM_RECT_INIT(8, 3, 4, 2);
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_set *pre = &ebc->pending.pre;

	KUNIT_EXPECT_FALSE(test, ebc->plane_resync);
	t_send(ebc, &clip, EBC_MODE_PART, 0, EBC_SOURCE_MXCFB);
	KUNIT_EXPECT_TRUE(test, ebc->plane_resync);

	ebc->mxcfb_active = true;
	KUNIT_EXPECT_FALSE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							 &box));
	t_expect_white(test, ebc->next_shadow);
	KUNIT_EXPECT_EQ(test, ebc->stats.updates, 1);
	KUNIT_EXPECT_TRUE(test, ebc->plane_resync);

	ebc->mxcfb_active = false;
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&box));
	t_expect_box_img(test, ebc->next_shadow, &t_panel, 0, 0);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&pre->box, &t_panel));
	KUNIT_EXPECT_FALSE(test, ebc->plane_resync);

	t_img_fill(ebc->next_shadow, 3);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&box));
	t_expect_box_img(test, ebc->next_shadow, &box, 0, 3);

	/* A RESET's white fill is another writer. */
	t_send(ebc, &t_panel, EBC_MODE_RESET, 0, EBC_SOURCE_ENABLE);
	KUNIT_EXPECT_TRUE(test, ebc->plane_resync);
	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&box));
	t_expect_box_img(test, ebc->next_shadow, &t_panel, 0, 0);
	KUNIT_EXPECT_FALSE(test, ebc->plane_resync);
}

/*
 * The policy does not change with the box: a damaged plane part whose
 * changes are all black or white goes DU, one with a grey PART. The
 * snapshot lays the box alone over the white glass.
 */
static void ebc_test_glue_plane_box_du(struct kunit *test)
{
	static const struct drm_rect box = DRM_RECT_INIT(2, 1, 4, 2);
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	u32 *black = t_plane_fb(test, 0);
	struct rk3026_ebc_last last;

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, black, T_W * 4,
							&box));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.asked_mode, EBC_MODE_FULL);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_BLACK_WHITE);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_DU);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	KUNIT_EXPECT_TRUE(test, drm_rect_equals(&last.clip, &box));
	t_expect_box_img(test, ebc->shadows.scan, &box, 0, 15);
}

static void ebc_test_glue_plane_box_grey(struct kunit *test)
{
	static const struct drm_rect box = DRM_RECT_INIT(2, 1, 4, 2);
	struct rk3026_ebc *ebc = t_policy_ebc(test);
	u32 *grey = t_plane_fb(test, 0x808080);
	struct rk3026_ebc_last last;

	KUNIT_ASSERT_TRUE(test, rk3026_ebc_plane_update(ebc, grey, T_W * 4,
							&box));
	rk3026_ebc_do_refresh(ebc, &last);
	KUNIT_EXPECT_EQ(test, last.mode, EBC_MODE_PART);
	KUNIT_EXPECT_EQ(test, last.reason, EBC_REASON_AUTO_PART);
	KUNIT_EXPECT_EQ(test, last.outcome, EBC_OUTCOME_NO_LUT);
	t_expect_box_img(test, ebc->shadows.scan, &box, 8, 15);
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
	KUNIT_CASE(ebc_test_glue_auto_du),
	KUNIT_CASE(ebc_test_glue_auto_grey),
	KUNIT_CASE(ebc_test_glue_auto_skipped),
	KUNIT_CASE(ebc_test_glue_explicit_full),
	KUNIT_CASE(ebc_test_glue_a2_over_grey),
	KUNIT_CASE(ebc_test_glue_du_cleanup),
	KUNIT_CASE(ebc_test_glue_du_budget_left),
	KUNIT_CASE(ebc_test_glue_du_budget_off),
	KUNIT_CASE(ebc_test_glue_du_skipped),
	KUNIT_CASE(ebc_test_glue_du_reset),
	KUNIT_CASE(ebc_test_glue_du_driven),
	KUNIT_CASE(ebc_test_glue_part_driven),
	KUNIT_CASE(ebc_test_glue_du_big),
	KUNIT_CASE(ebc_test_glue_du_ceiling),
	KUNIT_CASE(ebc_test_glue_a2_off),
	KUNIT_CASE(ebc_test_glue_a2_no_table),
	KUNIT_CASE(ebc_test_glue_a2_driven),
	KUNIT_CASE(ebc_test_glue_a2_spent),
	KUNIT_CASE(ebc_test_glue_idle_cleanup),
	KUNIT_CASE(ebc_test_glue_idle_not_due),
	KUNIT_CASE(ebc_test_glue_idle_carried),
	KUNIT_CASE(ebc_test_glue_idle_busy),
	KUNIT_CASE(ebc_test_glue_du_repaired),
	KUNIT_CASE(ebc_test_glue_repair_no_reset),
	KUNIT_CASE(ebc_test_glue_part_record),
	KUNIT_CASE(ebc_test_glue_rails_not_raised),
	KUNIT_CASE(ebc_test_glue_rails_before_take),
	KUNIT_CASE(ebc_test_glue_rails_fail),
	KUNIT_CASE(ebc_test_glue_gap_burst),
	KUNIT_CASE(ebc_test_glue_gap_lone),
	KUNIT_CASE(ebc_test_glue_gap_ramp_past),
	KUNIT_CASE(ebc_test_glue_gap_cursor_blink),
	KUNIT_CASE(ebc_test_glue_gap_not_driven),
	KUNIT_CASE(ebc_test_glue_gap_cap),
	KUNIT_CASE(ebc_test_glue_gap_exempt),
	KUNIT_CASE(ebc_test_glue_gap_big_full),
	KUNIT_CASE(ebc_test_glue_gap_retry),
	KUNIT_CASE(ebc_test_glue_retry_reset_tries),
	KUNIT_CASE(ebc_test_glue_reset_lost),
	KUNIT_CASE(ebc_test_glue_repair_sensor),
	KUNIT_CASE(ebc_test_glue_repair_no_full),
	KUNIT_CASE(ebc_test_glue_retry_temp),
	KUNIT_CASE(ebc_test_glue_reset_drops_carry_temp),
	KUNIT_CASE(ebc_test_glue_reset_drops_carry_temp_pre),
	KUNIT_CASE(ebc_test_glue_wait_bound),
	KUNIT_CASE(ebc_test_glue_rails_exempt),
	KUNIT_CASE(ebc_test_glue_retry_repair),
	KUNIT_CASE(ebc_test_glue_retry_failing),
	KUNIT_CASE(ebc_test_glue_retry_once),
	KUNIT_CASE(ebc_test_glue_retry_give_up),
	KUNIT_CASE(ebc_test_glue_retry_crtc_off),
	KUNIT_CASE(ebc_test_glue_sleep_holds),
	KUNIT_CASE(ebc_test_glue_sleep_rails_ramp),
	KUNIT_CASE(ebc_test_glue_sleep_mid_waveform),
	KUNIT_CASE(ebc_test_glue_sleep_carry),
	KUNIT_CASE(ebc_test_glue_sleep_gap_ramp),
	KUNIT_CASE(ebc_test_glue_sleep_restore),
	KUNIT_CASE(ebc_test_glue_sleep_crtc_wait),
#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_MXCFB)
	KUNIT_CASE(ebc_test_glue_mxcfb_send),
#endif
	KUNIT_CASE(ebc_test_glue_timing),
	KUNIT_CASE(ebc_test_glue_timing_skipped),
	KUNIT_CASE(ebc_test_glue_timing_dropped),
	KUNIT_CASE(ebc_test_glue_blit_time),
	KUNIT_CASE(ebc_test_plane_box),
	KUNIT_CASE(ebc_test_glue_plane_box_update),
	KUNIT_CASE(ebc_test_glue_plane_box_merge),
	KUNIT_CASE(ebc_test_glue_plane_resync),
	KUNIT_CASE(ebc_test_glue_plane_box_du),
	KUNIT_CASE(ebc_test_glue_plane_box_grey),
	{}
};

static struct kunit_suite rk3026_ebc_glue_test_suite = {
	.name = "rk3026_ebc_glue",
	.test_cases = rk3026_ebc_glue_test_cases,
};

kunit_test_suite(rk3026_ebc_glue_test_suite);

#if IS_ENABLED(CONFIG_DRM_FBDEV_EMULATION)

/* --- fbdev damage commits (rk3026_ebc_fbdev_fb_dirty()) --- */

/* What the wrapped fb_dirty saw: its calls, the clip, the console lock. */
static struct {
	unsigned int calls;
	struct drm_clip_rect clip;
	bool console_locked;
	int ret;
} t_dirty;

static int t_fb_dirty(struct drm_fb_helper *helper, struct drm_clip_rect *clip)
{
	t_dirty.calls++;
	t_dirty.clip = *clip;
	t_dirty.console_locked = is_console_locked();
	return t_dirty.ret;
}

static const struct drm_fb_helper_funcs t_fbdev_funcs = {
	.fb_dirty = t_fb_dirty,
};

/* A helper on a bare controller, with @pending as its damage not yet taken. */
static struct drm_fb_helper *t_fbdev(struct kunit *test,
				     struct rk3026_ebc *ebc,
				     struct drm_clip_rect pending)
{
	struct drm_fb_helper *helper = kunit_kzalloc(test, sizeof(*helper),
						     GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, helper);
	helper->dev = &ebc->drm;
	spin_lock_init(&helper->damage_lock);
	helper->damage_clip = pending;
	ebc->fbdev_funcs = &t_fbdev_funcs;
	memset(&t_dirty, 0, sizeof(t_dirty));
	return helper;
}

static const struct drm_clip_rect t_no_damage = {
	.x1 = USHRT_MAX, .y1 = USHRT_MAX, .x2 = 0, .y2 = 0,
};

static void t_expect_clip(struct kunit *test, const struct drm_clip_rect *c,
			  unsigned short x1, unsigned short y1,
			  unsigned short x2, unsigned short y2)
{
	KUNIT_EXPECT_EQ(test, c->x1, x1);
	KUNIT_EXPECT_EQ(test, c->y1, y1);
	KUNIT_EXPECT_EQ(test, c->x2, x2);
	KUNIT_EXPECT_EQ(test, c->y2, y2);
}

/* Off (the default): the clip goes through as it is, nothing else moves. */
static void ebc_test_fbdev_dirty_off(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	struct drm_clip_rect pending = { 0, 900, 32, 1072 };
	struct drm_fb_helper *helper = t_fbdev(test, ebc, pending);
	struct drm_clip_rect clip = { 0, 48, 32, 1072 };

	KUNIT_EXPECT_EQ(test, rk3026_ebc_fbdev_fb_dirty(helper, &clip), 0);
	KUNIT_EXPECT_EQ(test, t_dirty.calls, 1);
	t_expect_clip(test, &t_dirty.clip, 0, 48, 32, 1072);
	t_expect_clip(test, &helper->damage_clip, 0, 900, 32, 1072);
}

/*
 * On: the commit runs under the console lock, with the damage reported
 * since the worker took its clip folded in and taken off the helper; the
 * wrapped call's error comes back, with the folded clip for the worker
 * to put back.
 */
static void ebc_test_fbdev_dirty_locked(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	struct drm_clip_rect pending = { 64, 16, 96, 1072 };
	struct drm_fb_helper *helper = t_fbdev(test, ebc, pending);
	struct drm_clip_rect clip = { 0, 48, 32, 1000 };

	ebc->fbdev_console_lock = true;
	t_dirty.ret = -EIO;
	KUNIT_EXPECT_EQ(test, rk3026_ebc_fbdev_fb_dirty(helper, &clip), -EIO);
	KUNIT_EXPECT_EQ(test, t_dirty.calls, 1);
	KUNIT_EXPECT_TRUE(test, t_dirty.console_locked);
	t_expect_clip(test, &t_dirty.clip, 0, 16, 96, 1072);
	t_expect_clip(test, &clip, 0, 16, 96, 1072);
	t_expect_clip(test, &helper->damage_clip, USHRT_MAX, USHRT_MAX, 0, 0);
}

/* On, with no damage pending: the clip stays as the worker took it. */
static void ebc_test_fbdev_dirty_locked_quiet(struct kunit *test)
{
	struct rk3026_ebc *ebc = t_ebc_alloc(test);
	struct drm_fb_helper *helper = t_fbdev(test, ebc, t_no_damage);
	struct drm_clip_rect clip = { 0, 48, 32, 1072 };

	ebc->fbdev_console_lock = true;
	KUNIT_EXPECT_EQ(test, rk3026_ebc_fbdev_fb_dirty(helper, &clip), 0);
	KUNIT_EXPECT_TRUE(test, t_dirty.console_locked);
	t_expect_clip(test, &t_dirty.clip, 0, 48, 32, 1072);
	t_expect_clip(test, &helper->damage_clip, USHRT_MAX, USHRT_MAX, 0, 0);
}

static struct kunit_case rk3026_ebc_fbdev_test_cases[] = {
	KUNIT_CASE(ebc_test_fbdev_dirty_off),
	KUNIT_CASE(ebc_test_fbdev_dirty_locked),
	KUNIT_CASE(ebc_test_fbdev_dirty_locked_quiet),
	{}
};

static struct kunit_suite rk3026_ebc_fbdev_test_suite = {
	.name = "rk3026_ebc_fbdev",
	.test_cases = rk3026_ebc_fbdev_test_cases,
};

kunit_test_suite(rk3026_ebc_fbdev_test_suite);

#endif /* CONFIG_DRM_FBDEV_EMULATION */

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
	int two_level;
	int over_grey;
};

/*
 * The two-level result, the over-grey result and the changed count pixel
 * by pixel: on @box clipped and widened to whole bytes as the driver does,
 * the pixels where @next differs from @prev, whether every one of them is 0
 * or 15 in @next, and whether one of them is grey in @prev.
 */
static bool t_ref_two_level(const struct t_fix *f, const u8 *prev,
			    const u8 *next, const struct drm_rect *box,
			    bool *over_grey, u32 *changed_px)
{
	struct drm_rect w;
	bool grey = false;
	u32 n = 0;
	int x, y;

	*over_grey = false;
	t_widen(f, box, &w);
	for (y = w.y1; y < w.y2; y++) {
		for (x = w.x1; x < w.x2; x++) {
			u8 lvl = t_px_w(next, f, x, y);
			u8 old = t_px_w(prev, f, x, y);

			if (lvl == old)
				continue;
			n++;
			grey |= lvl != 0 && lvl != 15;
			*over_grey |= old != 0 && old != 15;
		}
	}
	*changed_px = n;
	return n && !grey;
}

/*
 * Every mode and box on one prev/next pair: the reference and the driver
 * write the same scan image over the same junk, guards included, return
 * the same and give the same mode; the two-level and over-grey results and
 * the changed count are t_ref_two_level()'s. False after the first
 * difference.
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
			bool ret_ref, ret, tl_ref, tl, og_ref, og;
			u32 chg_ref, chg;
			int diff;

			prandom_bytes_state(rs, scan_ref, t_fix_size(f));
			memcpy(scan, scan_ref, t_fix_size(f));
			sh.scan = scan_ref;
			ret_ref = rk3026_ebc_ref_snapshot(&sh, next, &clips[c],
							  &mode_ref);
			tl_ref = t_ref_two_level(f, prev, next, &clips[c],
						 &og_ref, &chg_ref);
			sh.scan = scan;
			ret = rk3026_ebc_build_snapshot(&sh, next, &clips[c],
							&mode, &tl, &og, &chg);
			diff = t_fix_diff(f, scan_ref, scan);
			if (ret != ret_ref || mode != mode_ref ||
			    diff != INT_MAX || tl != tl_ref || og != og_ref ||
			    chg != chg_ref) {
				KUNIT_FAIL(test,
					   "%s mode %u box " DRM_RECT_FMT
					   ": ret %d/%d mode %u/%u byte %d two-level %d/%d over-grey %d/%d changed %u/%u",
					   f->name, t_snap_modes[mi],
					   DRM_RECT_ARG(&clips[c]), ret, ret_ref,
					   mode, mode_ref, diff, tl, tl_ref,
					   og, og_ref, chg, chg_ref);
				return false;
			}
			cnt->raised += mode_ref != t_snap_modes[mi];
			cnt->moved += ret_ref;
			cnt->still += !ret_ref;
			cnt->two_level += tl_ref;
			cnt->over_grey += og_ref;
		}
	}
	return true;
}

/*
 * rk3026_ebc_build_snapshot() equals rk3026_ebc_ref_snapshot() on every
 * fixture, image pair, mode and box. The cases reach a move, no move, the
 * A2-over-grey raise, a two-level update and a change over a grey.
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
	KUNIT_EXPECT_GT(test, cnt.two_level, 0);
	KUNIT_EXPECT_GT(test, cnt.over_grey, 0);
}

/*
 * One snapshot of @box in @mode by the reference and the driver over the
 * same junk; *@mode_out is the driver's mode, *@og_out its over-grey
 * result. The two-level and over-grey results and the changed count are
 * checked against t_ref_two_level(). False, with a failure, if anything
 * differs.
 */
static bool t_snap_vs_ref(struct kunit *test, const struct t_fix *f,
			  u8 *prev, const u8 *next, u8 *scan_ref,
			  u8 *scan, const struct drm_rect *box, u32 mode,
			  u32 *mode_out, bool *og_out, struct rnd_state *rs)
{
	struct rk3026_ebc_shadows sh = {
		.prev = prev, .pitch = t_fix_pitch(f), .height = f->h,
	};
	u32 mode_ref = mode, chg_ref, chg;
	bool ret_ref, ret, tl_ref, tl, og_ref;
	int diff;

	prandom_bytes_state(rs, scan_ref, t_fix_size(f));
	memcpy(scan, scan_ref, t_fix_size(f));
	sh.scan = scan_ref;
	ret_ref = rk3026_ebc_ref_snapshot(&sh, next, box, &mode_ref);
	tl_ref = t_ref_two_level(f, prev, next, box, &og_ref, &chg_ref);
	sh.scan = scan;
	*mode_out = mode;
	ret = rk3026_ebc_build_snapshot(&sh, next, box, mode_out, &tl, og_out,
					&chg);
	diff = t_fix_diff(f, scan_ref, scan);
	if (ret == ret_ref && *mode_out == mode_ref && diff == INT_MAX &&
	    tl == tl_ref && *og_out == og_ref && chg == chg_ref)
		return true;
	KUNIT_FAIL(test, "%s mode %u box " DRM_RECT_FMT
		   ": ret %d/%d mode %u/%u byte %d two-level %d/%d over-grey %d/%d changed %u/%u",
		   f->name, mode, DRM_RECT_ARG(box), ret, ret_ref, *mode_out,
		   mode_ref, diff, tl, tl_ref, *og_out, og_ref, chg, chg_ref);
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
	bool og;
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
					   &og, &rs))
				return;
	KUNIT_EXPECT_TRUE(test, t_fix_guard_ok(f, scan));
}

/*
 * The A2-over-grey raise at every pixel of three words: on a black and
 * white prev, one grey at pixel @j of a row. An A2 scan that changes pixel
 * @j runs DU; one that changes only the pixel next to it, or changes @j on
 * a prev without the grey, stays A2. The over-grey result follows the same
 * pixels, in A2 and in FULL (a plane part's scan mode), which it does not
 * raise.
 */
static void ebc_test_snapshot_a2_guard_positions(struct kunit *test)
{
	const struct t_fix *f = &t_fixes[1];
	u8 *prev = t_fix_img(test, f), *next = t_fix_img(test, f);
	u8 *scan_ref = t_fix_img(test, f), *scan = t_fix_img(test, f);
	const struct drm_rect box = DRM_RECT_INIT(0, 0, f->w, f->h);
	struct rnd_state rs;
	int j, g, nb;
	bool og;
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
						   EBC_MODE_A2, &mode, &og,
						   &rs))
					return;
				KUNIT_EXPECT_EQ_MSG(test, mode,
						    !nb && grey ?
						    EBC_MODE_BLACK_WHITE :
						    EBC_MODE_A2,
						    "pixel %d grey %d at %d",
						    j, g, x);
				KUNIT_EXPECT_EQ(test, og, !nb && grey);
				if (!t_snap_vs_ref(test, f, prev, next,
						   scan_ref, scan, &box,
						   EBC_MODE_FULL, &mode, &og,
						   &rs))
					return;
				KUNIT_EXPECT_EQ(test, mode, EBC_MODE_FULL);
				KUNIT_EXPECT_EQ(test, og, !nb && grey);
			}
		}
	}
}

/* rk3026_ebc_build_snapshot() of @box on fixture @f, mode kept. */
static bool t_snap_tl(const struct t_fix *f, u8 *prev, const u8 *next,
		      u8 *scan, const struct drm_rect *box, u32 mode,
		      bool *two_level, u32 *changed_px)
{
	const struct rk3026_ebc_shadows sh = {
		.prev = prev, .scan = scan, .pitch = t_fix_pitch(f),
		.height = f->h,
	};
	bool over_grey;

	return rk3026_ebc_build_snapshot(&sh, next, box, &mode, two_level,
					 &over_grey, changed_px);
}

/*
 * The two-level result: true when every changed pixel goes to 0 or 15,
 * whatever the greys that stay; false on one changed grey and when nothing
 * changes, in every mode. The changed count counts what changes, also
 * where the scan moves nothing, and on the widened byte of an odd box
 * edge, but not past it.
 */
static void ebc_test_snapshot_two_level_result(struct kunit *test)
{
	const struct t_fix *f = &t_fixes[1];
	u8 *prev = t_fix_img(test, f), *next = t_fix_img(test, f);
	u8 *scan = t_fix_img(test, f);
	const struct drm_rect all = DRM_RECT_INIT(0, 0, f->w, f->h);
	/* x 5..24, widened to the bytes of pixels 4..25 */
	const struct drm_rect odd = DRM_RECT_INIT(5, 2, 20, 3);
	const struct drm_rect empty = DRM_RECT_INIT(8, 2, 0, 3);
	struct rnd_state rs;
	unsigned int mi;
	bool tl;
	u32 chg;
	int x;

	prandom_seed_state(&rs, 37);
	t_img_gen(prev, f, NULL, T_IMG_RANDOM, &rs);
	memcpy(next, prev, t_fix_size(f));
	for (mi = 0; mi < ARRAY_SIZE(t_snap_modes); mi++) {
		KUNIT_EXPECT_FALSE(test, t_snap_tl(f, prev, next, scan, &all,
						   t_snap_modes[mi], &tl,
						   &chg));
		KUNIT_EXPECT_FALSE(test, tl);
		KUNIT_EXPECT_EQ(test, chg, 0);
	}

	/* Black or white at every third pixel of a line, over greys. */
	for (x = 0; x < f->w; x += 3)
		t_px_set_w(next, f, x, 5, t_px_w(prev, f, x, 5) >= 8 ? 0 : 15);
	for (mi = 0; mi < ARRAY_SIZE(t_snap_modes); mi++) {
		KUNIT_EXPECT_TRUE(test, t_snap_tl(f, prev, next, scan, &all,
						  t_snap_modes[mi], &tl, &chg));
		KUNIT_EXPECT_TRUE(test, tl);
		KUNIT_EXPECT_EQ(test, chg, (f->w + 2) / 3);
	}
	KUNIT_EXPECT_FALSE(test, t_snap_tl(f, prev, next, scan, &empty,
					   EBC_MODE_PART, &tl, &chg));
	KUNIT_EXPECT_FALSE(test, tl);
	KUNIT_EXPECT_EQ(test, chg, 0);

	/* One changed grey among them. */
	t_px_set_w(next, f, 1, 5, t_px_w(prev, f, 1, 5) == 7 ? 8 : 7);
	for (mi = 0; mi < ARRAY_SIZE(t_snap_modes); mi++) {
		t_snap_tl(f, prev, next, scan, &all, t_snap_modes[mi], &tl,
			  &chg);
		KUNIT_EXPECT_FALSE(test, tl);
		KUNIT_EXPECT_EQ(test, chg, (f->w + 2) / 3 + 1);
	}

	/* A 12 over a 15 in A2 moves nothing, but it changed, and is grey. */
	memset(prev, 0xff, t_fix_size(f));
	memset(next, 0xff, t_fix_size(f));
	t_px_set_w(next, f, 9, 3, 12);
	KUNIT_EXPECT_FALSE(test, t_snap_tl(f, prev, next, scan, &all,
					   EBC_MODE_A2, &tl, &chg));
	KUNIT_EXPECT_FALSE(test, tl);
	KUNIT_EXPECT_EQ(test, chg, 1);
	KUNIT_EXPECT_TRUE(test, t_snap_tl(f, prev, next, scan, &all,
					  EBC_MODE_PART, &tl, &chg));
	KUNIT_EXPECT_FALSE(test, tl);
	KUNIT_EXPECT_EQ(test, chg, 1);
	t_px_set_w(next, f, 9, 3, 0);
	KUNIT_EXPECT_TRUE(test, t_snap_tl(f, prev, next, scan, &all,
					  EBC_MODE_A2, &tl, &chg));
	KUNIT_EXPECT_TRUE(test, tl);
	KUNIT_EXPECT_EQ(test, chg, 1);

	/* The widened edges count, the pixels past them do not. */
	memcpy(next, prev, t_fix_size(f));
	t_px_set_w(next, f, 10, 3, 0);
	t_px_set_w(next, f, 3, 2, 6);
	t_px_set_w(next, f, 26, 4, 6);
	t_px_set_w(next, f, 12, 1, 6);
	t_snap_tl(f, prev, next, scan, &odd, EBC_MODE_PART, &tl, &chg);
	KUNIT_EXPECT_TRUE(test, tl);
	KUNIT_EXPECT_EQ(test, chg, 1);
	t_px_set_w(next, f, 4, 2, 6);
	t_snap_tl(f, prev, next, scan, &odd, EBC_MODE_PART, &tl, &chg);
	KUNIT_EXPECT_FALSE(test, tl);
	KUNIT_EXPECT_EQ(test, chg, 2);
	t_px_set_w(next, f, 4, 2, 15);
	t_px_set_w(next, f, 25, 4, 9);
	t_snap_tl(f, prev, next, scan, &odd, EBC_MODE_BLACK_WHITE, &tl, &chg);
	KUNIT_EXPECT_FALSE(test, tl);
	KUNIT_EXPECT_EQ(test, chg, 2);
	t_px_set_w(next, f, 25, 4, 0);
	t_snap_tl(f, prev, next, scan, &odd, EBC_MODE_BLACK_WHITE, &tl, &chg);
	KUNIT_EXPECT_TRUE(test, tl);
	KUNIT_EXPECT_EQ(test, chg, 2);
	KUNIT_EXPECT_TRUE(test, t_fix_guard_ok(f, scan));
}

/*
 * The two-level result at every pixel of three words, and of a word and a
 * three-byte tail: on a black and white prev, pixel @j changed to each
 * other level. Two-level iff the new level is 0 or 15, one pixel changed.
 */
static void ebc_test_snapshot_two_level_positions(struct kunit *test)
{
	const struct t_fix *f = &t_fixes[1];
	u8 *prev = t_fix_img(test, f), *next = t_fix_img(test, f);
	u8 *scan_ref = t_fix_img(test, f), *scan = t_fix_img(test, f);
	const struct drm_rect boxes[] = {
		DRM_RECT_INIT(0, 0, f->w, f->h),
		DRM_RECT_INIT(0, 3, 14, 1),
	};
	struct rnd_state rs;
	unsigned int bi;
	int j, g, w;
	bool tl, og;
	u32 mode, chg;

	prandom_seed_state(&rs, 41);
	for (bi = 0; bi < ARRAY_SIZE(boxes); bi++) {
		w = bi ? 14 : 24;
		for (j = 0; j < w; j++) {
			for (g = 0; g < 16; g++) {
				t_img_gen(prev, f, NULL, T_IMG_TWO_LEVEL, &rs);
				memcpy(next, prev, t_fix_size(f));
				if (t_px_w(prev, f, j, 3) == g)
					continue;
				t_px_set_w(next, f, j, 3, g);
				if (!t_snap_vs_ref(test, f, prev, next,
						   scan_ref, scan, &boxes[bi],
						   EBC_MODE_PART, &mode, &og,
						   &rs))
					return;
				KUNIT_EXPECT_FALSE(test, og);
				t_snap_tl(f, prev, next, scan, &boxes[bi],
					  EBC_MODE_BLACK_WHITE, &tl, &chg);
				KUNIT_EXPECT_EQ_MSG(test, tl, g == 0 || g == 15,
						    "box %u pixel %d level %d",
						    bi, j, g);
				KUNIT_EXPECT_EQ(test, chg, 1);
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
 * rk3026_ebc_blit_xrgb8888() widens and clamps the clip to whole bytes and
 * writes the bytes of the per-pixel reference loop, on every fixture,
 * source set, dense and padded source pitch, and clip.
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

/* A row conversion in the shape of rk3026_ebc_blit_rows(). */
typedef void t_rows_fn(u8 *dst, const void *src, unsigned int src_pitch,
		       int x1, int x2, int y1, int y2, unsigned int width);

/*
 * @rows and the reference rows @ref_rows on @x1..@x2 (even), @y1..@y2 of
 * @f write the same bytes over the same junk, guards included. False after
 * a difference.
 */
static bool t_rows_vs_ref(struct kunit *test, const struct t_fix *f,
			  t_rows_fn *rows, t_rows_fn *ref_rows, u8 *out,
			  u8 *ref, const u8 *src, unsigned int pitch, int x1,
			  int x2, int y1, int y2, struct rnd_state *rs)
{
	int diff;

	prandom_bytes_state(rs, ref, t_fix_size(f));
	memcpy(out, ref, t_fix_size(f));
	ref_rows(ref, src, pitch, x1, x2, y1, y2, f->w);
	rows(out, src, pitch, x1, x2, y1, y2, f->w);
	diff = t_fix_diff(f, ref, out);
	if (diff == INT_MAX)
		return true;
	KUNIT_FAIL(test, "%s pitch %u x %d..%d y %d..%d: byte %d (%02x, ref %02x)",
		   f->name, pitch, x1, x2, y1, y2, diff, out[diff], ref[diff]);
	return false;
}

/*
 * @rows against @ref_rows on one source and every non-empty clip of
 * @clips, widened and clamped as rk3026_ebc_blit_xrgb8888() does.
 */
static bool t_rows_clips(struct kunit *test, const struct t_fix *f,
			 t_rows_fn *rows, t_rows_fn *ref_rows, u8 *out,
			 u8 *ref, const u8 *src, unsigned int pitch,
			 const struct drm_rect *clips, int n,
			 struct rnd_state *rs)
{
	int c;

	for (c = 0; c < n; c++) {
		struct drm_rect w;

		if (!t_widen(f, &clips[c], &w))
			continue;
		if (!t_rows_vs_ref(test, f, rows, ref_rows, out, ref, src,
				   pitch, w.x1, w.x2, w.y1, w.y2, rs))
			return false;
	}
	return true;
}

/*
 * @rows against @ref_rows on every fixture, source set, dense and padded
 * source pitch, and clip.
 */
static void t_rows_sets(struct kunit *test, t_rows_fn *rows,
			t_rows_fn *ref_rows)
{
	unsigned int fi, pad, kind;
	u32 seed;

	for (fi = 0; fi < ARRAY_SIZE(t_fixes); fi++) {
		const struct t_fix *f = &t_fixes[fi];
		u8 *out = t_fix_img(test, f);
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
					prandom_seed_state(&rs, seed * 11 + kind);
					n = t_fix_clips(f, &rs, clips);
					if (t_rows_clips(test, f, rows, ref_rows,
							 out, ref, src, pitch,
							 clips, n, &rs))
						continue;
					kunit_info(test, "source %u seed %u\n",
						   kind, seed);
					return;
				}
			}
		}
	}
}

/*
 * Every alignment of a row: every even start 0..30 and every even length
 * 2..80, so every way a row splits into blocks and a tail for blocks of
 * up to 16 pixels, on random colours, white and black, each with a random
 * X byte; @rows against @ref_rows.
 */
static void t_rows_tails(struct kunit *test, t_rows_fn *rows,
			 t_rows_fn *ref_rows)
{
	static const struct t_fix f = { "112x3", 112, 3 };
	unsigned int pitch = f.w * 4;
	u8 *out = t_fix_img(test, &f);
	u8 *ref = t_fix_img(test, &f);
	u8 *src = kunit_kmalloc(test, (size_t)pitch * f.h, GFP_KERNEL);
	struct rnd_state rs;
	int x, x1, len;

	KUNIT_ASSERT_NOT_NULL(test, src);
	prandom_seed_state(&rs, 1616);
	for (x = 0; x < f.w; x++) {
		u32 x_byte = prandom_u32_state(&rs) & 0xff000000;

		put_unaligned_le32(prandom_u32_state(&rs), src + x * 4);
		put_unaligned_le32(x_byte | 0xffffff, src + pitch + x * 4);
		put_unaligned_le32(x_byte, src + 2 * pitch + x * 4);
	}
	for (x1 = 0; x1 <= 30; x1 += 2) {
		for (len = 2; len <= 80; len += 2) {
			if (t_rows_vs_ref(test, &f, rows, ref_rows, out, ref,
					  src, pitch, x1, x1 + len, 0, f.h,
					  &rs))
				continue;
			kunit_info(test, "start %d length %d\n", x1, len);
			return;
		}
	}
}

/* rk3026_ebc_blit_rows() writes the bytes of the per-pixel reference. */
static void ebc_test_blit_rows(struct kunit *test)
{
	t_rows_sets(test, rk3026_ebc_blit_rows, rk3026_ebc_ref_blit_rows);
}

/* The 4-px stores and the 2-px tail from every start. */
static void ebc_test_blit_rows_tails(struct kunit *test)
{
	t_rows_tails(test, rk3026_ebc_blit_rows, rk3026_ebc_ref_blit_rows);
}

/*
 * Whether the NEON rows can be compared here: built in
 * (DRM_RK3026_EBC_NEON) and a CPU with NEON, in a thread where
 * may_use_simd() holds. Else the case is skipped, so the run with the NEON
 * options of .kunitconfig must show no skip.
 */
static bool t_neon_ready(struct kunit *test)
{
#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_NEON)
	if (!cpu_has_neon())
		kunit_skip(test, "the CPU has no NEON");
	KUNIT_ASSERT_TRUE_MSG(test, may_use_simd(),
			      "may_use_simd() false in a test thread");
	return true;
#else
	kunit_skip(test, "built without CONFIG_DRM_RK3026_EBC_NEON");
	return false;
#endif
}

#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_NEON)
/* rk3026_ebc_blit_rows_neon() in the NEON section it needs. */
static void t_rows_neon(u8 *dst, const void *src, unsigned int src_pitch,
			int x1, int x2, int y1, int y2, unsigned int width)
{
	scoped_ksimd()
		rk3026_ebc_blit_rows_neon(dst, src, src_pitch, x1, x2, y1, y2,
					  width);
}

/*
 * rk3026_ebc_blit_rows_neon() writes the bytes of rk3026_ebc_blit_rows():
 * every fixture (1448 px leave an 8-px tail), source set, pitch and clip.
 */
static void ebc_test_blit_neon_rows(struct kunit *test)
{
	if (!t_neon_ready(test))
		return;
	t_rows_sets(test, t_rows_neon, rk3026_ebc_blit_rows);
}

/* The 16-px blocks and every tail of 0 to 14 px after 0 to 5 of them. */
static void ebc_test_blit_neon_tails(struct kunit *test)
{
	if (!t_neon_ready(test))
		return;
	t_rows_tails(test, t_rows_neon, rk3026_ebc_blit_rows);
}
#else
/* No NEON rows: a skip. */
static void ebc_test_blit_neon_rows(struct kunit *test)
{
	t_neon_ready(test);
}

static void ebc_test_blit_neon_tails(struct kunit *test)
{
	t_neon_ready(test);
}
#endif

/*
 * rk3026_ebc_blit_convert() writes the bytes of rk3026_ebc_blit_rows() on
 * any kernel: with NEON in bands of 64 rows, so on row ranges that start
 * and end inside a band and on its edges, and with the scalar rows for a
 * clip narrower than 16 px. With the NEON rows built in, the NEON path
 * must be able to run here, as for the NEON cases; without them the case
 * only shows that the dispatch reaches the scalar rows.
 */
static void ebc_test_blit_convert(struct kunit *test)
{
	static const struct t_fix f = { "48x200", 48, 200 };
	static const struct {
		int x1, x2, y1, y2;
	} r[] = {
		{ 0, 48, 0, 200 }, { 2, 46, 1, 130 }, { 0, 48, 63, 65 },
		{ 4, 38, 5, 69 }, { 0, 48, 0, 64 }, { 0, 48, 0, 65 },
		{ 0, 48, 64, 128 }, { 16, 32, 199, 200 }, { 0, 16, 0, 200 },
		{ 2, 16, 0, 200 }, { 6, 8, 60, 70 },
	};
	unsigned int pitch = f.w * 4;
	u8 *out = t_fix_img(test, &f);
	u8 *ref = t_fix_img(test, &f);
	u8 *src = kunit_kmalloc(test, (size_t)pitch * f.h, GFP_KERNEL);
	struct rnd_state rs;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, src);
	if (IS_ENABLED(CONFIG_DRM_RK3026_EBC_NEON) && !t_neon_ready(test))
		return;
	t_src_fill(src, pitch, &f, T_SRC_RANDOM, 77);
	prandom_seed_state(&rs, 7700);
	for (i = 0; i < ARRAY_SIZE(r); i++)
		if (!t_rows_vs_ref(test, &f, rk3026_ebc_blit_convert,
				   rk3026_ebc_blit_rows, out, ref, src, pitch,
				   r[i].x1, r[i].x2, r[i].y1, r[i].y2, &rs))
			return;
}

static struct kunit_case rk3026_ebc_conv_test_cases[] = {
	KUNIT_CASE(ebc_test_ref_snapshot),
	KUNIT_CASE(ebc_test_snapshot_nibble_pairs),
	KUNIT_CASE(ebc_test_snapshot_a2_guard_positions),
	KUNIT_CASE(ebc_test_snapshot_two_level_result),
	KUNIT_CASE(ebc_test_snapshot_two_level_positions),
	KUNIT_CASE(ebc_test_ref_blit),
	KUNIT_CASE(ebc_test_blit_rows),
	KUNIT_CASE(ebc_test_blit_rows_tails),
	KUNIT_CASE(ebc_test_blit_neon_rows),
	KUNIT_CASE(ebc_test_blit_neon_tails),
	KUNIT_CASE(ebc_test_blit_convert),
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
