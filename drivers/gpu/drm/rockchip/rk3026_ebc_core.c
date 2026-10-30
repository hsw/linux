// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK3026 EBC: the refresh bookkeeping as pure functions.
 *
 * Which updates one scan draws, in which mode, from which image, and what the
 * driver believes is on the glass afterwards. Nothing here touches the
 * device or takes a lock: the callers hold the one that guards the plain
 * structs below, and rk3026_ebc_kunit.c tests them as they are.
 */

#include <linux/compiler.h>
#include <linux/export.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/overflow.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include <kunit/visibility.h>

#include "rk3026_ebc.h"

/*
 * DSP_CTRL mode class. Modes split into three groups by the mask tests in the
 * controller's mode dispatch: LUT-driven full refreshes assert DISPLAY_MODE,
 * partial refreshes assert UPDATE_MODE as well, and the direct (non-LUT)
 * modes assert neither.
 */
u32 rk3026_ebc_dsp_ctrl_mode(u32 mode)
{
	u32 bit;

	if (mode > EBC_MODE_MAX)
		return DSP_DISPLAY_MODE;

	bit = BIT(mode);
	if (bit & 0x0522)
		return DSP_DISPLAY_MODE;
	if (bit & 0x1a01)
		return 0;
	if (bit & 0x00cc)
		return DSP_UPDATE_MODE | DSP_DISPLAY_MODE;
	return DSP_DISPLAY_MODE;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_dsp_ctrl_mode);

/**
 * rk3026_ebc_us_between - microseconds from @start to @end, for a record
 * @start: when it began
 * @end: when it ended
 *
 * Return: the time rounded up and at least 1, so a time that was measured
 * is never 0 (0 means none in a record), even on a clock too coarse to see
 * it; an @end before @start counts as 1 too. Saturates at U32_MAX.
 */
u32 rk3026_ebc_us_between(ktime_t start, ktime_t end)
{
	s64 ns = ktime_to_ns(ktime_sub(end, start));

	if (ns <= 0)
		return 1;
	return min_t(u64, DIV_ROUND_UP_ULL(ns, NSEC_PER_USEC), U32_MAX);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_us_between);

/*
 * How strongly a mode drives, for merging updates into one scan: the merged
 * scan runs the strongest of their modes, so none of them is drawn weaker
 * than it asked. A2 < DU < PART (16-level partial) < OED_PART (the 16-level
 * no-flash family) < FULL. A mode without a place in that order ranks as
 * FULL: it is never the one a merge gives up. RESET is never merged.
 */
VISIBLE_IF_KUNIT unsigned int rk3026_ebc_mode_rank(u32 mode)
{
	switch (mode) {
	case EBC_MODE_A2:
		return 0;
	case EBC_MODE_BLACK_WHITE:
		return 1;
	case EBC_MODE_PART:
		return 2;
	case EBC_MODE_OED_PART:
		return 3;
	default:
		return 4;
	}
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_mode_rank);

/* The mode drives black and white only (DU, A2): greys are not targets. */
static bool ebc_mode_two_level(u32 mode)
{
	return mode == EBC_MODE_A2 || mode == EBC_MODE_BLACK_WHITE;
}

/* The stronger of two modes; on a tie FULL, the mode the rank stands for. */
static u32 ebc_mode_max(u32 a, u32 b)
{
	unsigned int ra = rk3026_ebc_mode_rank(a);
	unsigned int rb = rk3026_ebc_mode_rank(b);

	if (rb > ra || (rb == ra && b == EBC_MODE_FULL))
		return b;
	return a;
}

static void ebc_box_union(struct drm_rect *dst, const struct drm_rect *src)
{
	if (!drm_rect_visible(src))
		return;
	if (!drm_rect_visible(dst)) {
		*dst = *src;
		return;
	}
	dst->x1 = min(dst->x1, src->x1);
	dst->y1 = min(dst->y1, src->y1);
	dst->x2 = max(dst->x2, src->x2);
	dst->y2 = max(dst->y2, src->y2);
}

/*
 * Merge the box, the mode and the flags of @src into @set: the bounding box,
 * the stronger mode, either full-class, plane or idle cleanup flag, and
 * @auto_ok only if both have it. An @empty set takes the box, the mode and
 * @auto_ok of @src as they are. A temperature @set lacks is taken from
 * @src, and a valid one of @set stays, as rk3026_ebc_pending_peek() needs
 * it: there @src is the carry, the older. Where @src is the newer, the
 * caller puts its temperature over @set's after the merge (ebc_set_add(),
 * ebc_carry()).
 */
static void ebc_set_merge(struct rk3026_ebc_set *set, bool empty,
			  const struct rk3026_ebc_set *src)
{
	if (empty) {
		set->box = src->box;
		set->mode = src->mode;
		set->auto_ok = src->auto_ok;
	} else {
		ebc_box_union(&set->box, &src->box);
		set->mode = ebc_mode_max(set->mode, src->mode);
		set->auto_ok &= src->auto_ok;
	}
	set->has_mxcfb_full |= src->has_mxcfb_full;
	set->has_mxcfb |= src->has_mxcfb;
	set->has_plane |= src->has_plane;
	set->cleanup_idle |= src->cleanup_idle;
	if (!set->temp_valid && src->temp_valid) {
		set->temp = src->temp;
		set->temp_valid = true;
	}
}

static void ebc_set_add(struct rk3026_ebc_set *set,
			const struct rk3026_ebc_update *upd, u32 seq)
{
	/*
	 * Full class is what rk3026_ebc_dsp_ctrl_mode() gives DISPLAY_MODE
	 * alone. A client asked for that flash once, so the identical-image
	 * skip must not take it away; fbdev and plane commits arrive twice and
	 * still skip.
	 */
	const struct rk3026_ebc_set one = {
		.box		= upd->clip,
		.mode		= upd->mode,
		.has_mxcfb_full	= upd->source == EBC_SOURCE_MXCFB &&
				  rk3026_ebc_dsp_ctrl_mode(upd->mode) ==
				  DSP_DISPLAY_MODE,
		.has_mxcfb	= upd->source == EBC_SOURCE_MXCFB,
		.auto_ok	= upd->auto_mode,
		.has_plane	= upd->source == EBC_SOURCE_PLANE,
		.cleanup_idle	= upd->source == EBC_SOURCE_IDLE,
	};

	if (!set->merged)
		set->first = upd->queued;
	set->last = upd->queued;
	ebc_set_merge(set, !set->merged, &one);
	if (upd->temp_valid) {
		set->temp = upd->temp;
		set->temp_valid = true;
	}
	set->seq = seq;
	set->marker = upd->marker;
	set->blit_us = upd->blit_us;
	set->source = upd->source;
	set->merged++;
}

/**
 * rk3026_ebc_pending_add - merge one update into the pending state
 * @p: pending state
 * @upd: the update
 * @seq: returns the seq the update was queued with
 *
 * A RESET goes to the barrier; anything else joins @post while a RESET is
 * pending and @pre otherwise. A RESET sent while one is pending absorbs
 * @post: that RESET whitens the glass after whatever @post would draw, so
 * the two collapse into one barrier carrying the newest seq. The one state
 * cannot hold two barriers, and nothing ever overflows. A RESET drops the
 * carry for the same reason; a retry armed for it ends at the refresh
 * thread's next look (rk3026_ebc_retry_ready()), and the RESET's finish
 * moves @done_seq over the carry's updates.
 *
 * Returns true if the update was merged into an update already pending.
 */
bool rk3026_ebc_pending_add(struct rk3026_ebc_pending *p,
			    const struct rk3026_ebc_update *upd, u32 *seq)
{
	struct rk3026_ebc_set *set;
	bool merged;

	/* Read without the lock by the refresh thread's wait. */
	*seq = p->queued_seq + 1;
	WRITE_ONCE(p->queued_seq, *seq);

	if (upd->mode == EBC_MODE_RESET) {
		set = &p->reset;
		merged = set->merged || p->post.merged;
		if (set->merged) {
			set->merged += p->post.merged;
			memset(&p->post, 0, sizeof(p->post));
		}
		memset(&p->carry, 0, sizeof(p->carry));
	} else {
		set = p->reset.merged ? &p->post : &p->pre;
		merged = set->merged;
	}

	ebc_set_add(set, upd, *seq);
	return merged;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_pending_add);

/**
 * rk3026_ebc_pending_peek - the part the next take would give
 * @p: pending state
 * @part: returns the part, as rk3026_ebc_pending_take() would give it
 *
 * @p is left as it is, so the refresh thread can look at the next part
 * (whether it will be driven, rk3026_ebc_do_refresh()) before it takes it.
 * With nothing else pending, the carry is the part while its retry is due
 * (@p->retry.ready, rk3026_ebc_retry_ready()).
 *
 * Returns false, with @part cleared, if nothing is pending.
 */
bool rk3026_ebc_pending_peek(const struct rk3026_ebc_pending *p,
			     struct rk3026_ebc_set *part)
{
	if (p->pre.merged) {
		*part = p->pre;
		if (p->carry.merged)
			ebc_set_merge(part, false, &p->carry);
		return true;
	}
	if (p->reset.merged) {
		*part = p->reset;
		return true;
	}
	if (p->carry.merged && p->retry.ready) {
		*part = p->carry;
		return true;
	}

	memset(part, 0, sizeof(*part));
	return false;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_pending_peek);

/**
 * rk3026_ebc_pending_take - take the next part for one scan, in seq order
 * @p: pending state
 * @part: returns the part
 *
 * @pre comes first, with the carry merged in; a carry clears @auto_ok, so
 * what it carries is drawn in its plain mode. Then the RESET alone, after
 * which @post becomes @pre: everything left was queued after the RESET.
 * Then the carry alone, at its retry. @taken_seq follows the part; the seq
 * of a carry alone is the newest taken already, so it stays.
 *
 * Returns false, with @part cleared, if nothing is pending.
 */
bool rk3026_ebc_pending_take(struct rk3026_ebc_pending *p,
			     struct rk3026_ebc_set *part)
{
	if (!rk3026_ebc_pending_peek(p, part))
		return false;

	/* Read without the lock by the refresh thread's wait. */
	WRITE_ONCE(p->taken_seq, part->seq);
	p->retry.ready = false;
	if (p->pre.merged) {
		memset(&p->pre, 0, sizeof(p->pre));
		memset(&p->carry, 0, sizeof(p->carry));
		return true;
	}
	if (p->reset.merged) {
		p->pre = p->post;
		memset(&p->reset, 0, sizeof(p->reset));
		memset(&p->post, 0, sizeof(p->post));
		return true;
	}

	memset(&p->carry, 0, sizeof(p->carry));
	return true;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_pending_take);

/* What ebc_snap_word() collects over the box, see there. */
struct ebc_snap_acc {
	u32 over_grey;
	u32 grey_new;
	u32 moves;
	u32 count;
};

/*
 * Eight pixels of the snapshot at once: @p and @n are one little-endian
 * word of prev and next, two pixels per byte, low nibble first.
 *  - @changed has all four bits set in every nibble where @n differs from
 *    @p: the OR of the nibble's bits lands in its bit 0, and the multiply
 *    by 15 fills the nibble without a carry between nibbles;
 *  - @thr is the two-level threshold of @n, 15 where bit 3 is set (8..15),
 *    else 0; @tl (all ones for a two-level mode, else 0) picks it;
 *  - a nibble of @p is grey when its four bits are not all equal; bit 3
 *    would be compared with the next nibble up, so bits 0..2 are tested.
 * The accumulators of one snapshot:
 *  - @over_grey collects a grey of prev under a changed nibble;
 *  - @grey_new a grey of next in a changed nibble, tested as for prev;
 *  - @moves the bits that change;
 *  - @count the changed nibbles: with one bit per changed nibble, the
 *    multiply by 0x11111111 sums the eight bits into the top nibble (no
 *    partial sum reaches 16, so none carries).
 * A short tail goes through zero-extended: its upper nibbles are equal,
 * so they neither change nor count.
 */
static __always_inline u32 ebc_snap_word(u32 p, u32 n, u32 tl,
					 struct ebc_snap_acc *acc)
{
	u32 d = p ^ n;
	u32 bits = (d | d >> 1 | d >> 2 | d >> 3) & 0x11111111;
	u32 changed = bits * 0xf;
	u32 thr = ((n >> 3) & 0x11111111) * 0xf;
	u32 v = (thr & tl) | (n & ~tl);
	u32 out = (p & ~changed) | (v & changed);

	acc->over_grey |= (p ^ (p >> 1)) & 0x77777777 & changed;
	acc->grey_new |= (n ^ (n >> 1)) & 0x77777777 & changed;
	acc->moves |= out ^ p;
	acc->count += (bits * 0x11111111) >> 28;
	return out;
}

/* ebc_snap_word() over the box rows: words of 8 pixels, then bytes. */
static __always_inline void ebc_snap_rows(u8 *dst, const u8 *prev,
					  const u8 *next, u32 pitch,
					  const struct drm_rect *r, u32 tl,
					  struct ebc_snap_acc *acc)
{
	int y;

	for (y = r->y1; y < r->y2; y++) {
		size_t off = (size_t)y * pitch + r->x1 / 2;
		const u8 *p = prev + off, *n = next + off;
		u8 *d = dst + off;
		int nb = (r->x2 - r->x1) / 2;
		u32 out;

		for (; nb >= 4; nb -= 4, p += 4, n += 4, d += 4) {
			out = ebc_snap_word(get_unaligned_le32(p),
					    get_unaligned_le32(n), tl, acc);
			put_unaligned_le32(out, d);
		}
		for (; nb > 0; nb--, p++, n++, d++)
			*d = ebc_snap_word(*p, *n, tl, acc);
	}
}

/**
 * rk3026_ebc_build_snapshot - the image one scan drives
 * @sh: the shadow images; the snapshot is built into @sh->scan
 * @next: the target image the senders write, laid out like the shadows
 * @box: the part's bounding box, in pixels
 * @mode: the scan's mode (rk3026_ebc_scan_mode()); returns the mode that
 *	draws the snapshot
 * @two_level: returns true if some pixel changes and every changed pixel
 *	of @next is 0 or 15
 * @over_grey: returns true if some pixel that changes is grey in @prev
 * @changed_px: returns the number of pixels where @next differs from @prev
 *
 * @prev (@sh->prev, the image on the glass) with @next laid over it inside
 * @box, pixel by pixel, where @next differs from @prev. Outside @box
 * nothing moves, so a scan draws only what its updates covered. For a
 * two-level mode (DU, A2) the laid-over pixels are thresholded to 0 or 15,
 * which is what the glass will show and so what @prev must hold afterwards.
 * Pixels where @next equals @prev keep @prev, so a grey the scan does not
 * change is never thresholded: not inside @box, and not on the even-pixel
 * widening of an odd @box edge (the images are packed two pixels per byte,
 * low nibble first, and blits are widened the same way).
 *
 * The box rows are worked eight pixels per 32-bit word (ebc_snap_word()),
 * without a branch per pixel, and every byte of the box is written once;
 * only what lies outside the box is copied from @prev.
 *
 * The A2 waveform drives only black <-> white: from a grey it does not
 * move a pixel at all (the A2 table of the panel's waveform), so the glass
 * would keep the grey while @prev said 0 or 15. An A2 scan that lays a pixel
 * over a grey of @prev is therefore raised to DU in @mode. DU drives every
 * level to black or white and thresholds the same way, so the snapshot
 * stands.
 *
 * @two_level, @over_grey and @changed_px describe the update, not the
 * scan: they are taken on @next before the threshold, on @prev, and on the
 * pixels where the two differ, the widened edges included; greys that do
 * not change do not count. A two-level update can be driven in DU with the
 * same snapshot, as no changed pixel needs the threshold, and in A2 when
 * @over_grey is false too: @over_grey is the test that raises an A2 scan,
 * returned for every mode, so the caller can tell whether A2 would stand.
 *
 * Returns true if the snapshot differs from @prev, false if the scan would
 * move nothing.
 */
bool rk3026_ebc_build_snapshot(const struct rk3026_ebc_shadows *sh,
			       const u8 *next, const struct drm_rect *box,
			       u32 *mode, bool *two_level, bool *over_grey,
			       u32 *changed_px)
{
	const u8 *prev = sh->prev;
	u32 pitch = sh->pitch, height = sh->height;
	u8 *dst = sh->scan;
	u32 tl = ebc_mode_two_level(*mode) ? ~0u : 0;
	struct ebc_snap_acc acc = { 0 };
	struct drm_rect r = *box;
	int y;

	*two_level = false;
	*over_grey = false;
	*changed_px = 0;
	if (!rk3026_ebc_widen_clip(&r, pitch * 2, height)) {
		memcpy(dst, prev, (size_t)pitch * height);
		return false;
	}

	memcpy(dst, prev, (size_t)pitch * r.y1);
	memcpy(dst + (size_t)pitch * r.y2, prev + (size_t)pitch * r.y2,
	       (size_t)pitch * (height - r.y2));
	if (r.x1 > 0 || r.x2 < (int)(pitch * 2)) {
		for (y = r.y1; y < r.y2; y++) {
			size_t row = (size_t)y * pitch;

			memcpy(dst + row, prev + row, r.x1 / 2);
			memcpy(dst + row + r.x2 / 2, prev + row + r.x2 / 2,
			       pitch - r.x2 / 2);
		}
	}

	ebc_snap_rows(dst, prev, next, pitch, &r, tl, &acc);
	if (acc.over_grey && *mode == EBC_MODE_A2)
		*mode = EBC_MODE_BLACK_WHITE;
	*two_level = acc.count && !acc.grey_new;
	*over_grey = acc.over_grey != 0;
	*changed_px = acc.count;
	return acc.moves != 0;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_build_snapshot);

/**
 * rk3026_ebc_target_differs - whether a part's target differs from the glass
 * @sh: the shadow images; only @sh->prev is read
 * @next: the target image the senders write, laid out like the shadows
 * @box: the part's bounding box, in pixels
 *
 * The bytes of @box, widened as rk3026_ebc_build_snapshot() widens it,
 * compared between @next and @sh->prev, without building a snapshot. Where
 * they are equal the snapshot keeps @prev, so false here means the snapshot
 * would be identical. True does not promise the reverse: a two-level mode
 * may threshold a change back to what @prev holds.
 *
 * Return: true if some byte of the widened box differs.
 */
bool rk3026_ebc_target_differs(const struct rk3026_ebc_shadows *sh,
			       const u8 *next, const struct drm_rect *box)
{
	struct drm_rect r = *box;
	int y;

	if (!rk3026_ebc_widen_clip(&r, sh->pitch * 2, sh->height))
		return false;

	for (y = r.y1; y < r.y2; y++) {
		size_t off = (size_t)y * sh->pitch + r.x1 / 2;

		if (memcmp(next + off, sh->prev + off, (r.x2 - r.x1) / 2))
			return true;
	}
	return false;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_target_differs);

/**
 * rk3026_ebc_should_skip - whether a part's scan would drive nothing
 * @part: the part
 * @prev_unknown: the prev image is not evidence
 * @identical: the snapshot equals the prev image (rk3026_ebc_build_snapshot())
 *
 * A waveform with identical old and new greys still runs its flash: it is
 * wear, and fbdev commits every update twice. But a RESET is never skipped
 * (it must make the glass white, not assume it), an unknown prev proves
 * nothing, and an mxcfb full-class send is the flash its client asked for.
 * Nor is the idle cleanup (@part->cleanup_idle): that flash is the point
 * of it, and it redraws the glass as it is, so its snapshot is identical
 * as a rule.
 */
bool rk3026_ebc_should_skip(const struct rk3026_ebc_set *part,
			    bool prev_unknown, bool identical)
{
	return identical && !prev_unknown && part->mode != EBC_MODE_RESET &&
	       !part->has_mxcfb_full && !part->cleanup_idle;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_should_skip);

/**
 * rk3026_ebc_scan_mode - the mode a part is driven in
 * @part: the part
 * @repair_due: a repair is due before it
 *
 * After a repair the glass is white outside the box too, while the snapshot
 * there still holds the old greys, and a two-level LUT does not drive white
 * to grey: such a scan runs at least PART. Without a RESET at the
 * temperature the part is the repair and runs FULL
 * (rk3026_ebc_resolve_mode()): its snapshot is not thresholded either.
 */
u32 rk3026_ebc_scan_mode(const struct rk3026_ebc_set *part, bool repair_due)
{
	if (repair_due && part->mode != EBC_MODE_RESET &&
	    rk3026_ebc_mode_rank(part->mode) <
	    rk3026_ebc_mode_rank(EBC_MODE_PART))
		return EBC_MODE_PART;
	return part->mode;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_scan_mode);

/**
 * rk3026_ebc_drive_knows_glass - whether a waveform run to its end makes the
 *	glass known whatever it showed before
 * @mode: the EBC work mode it was driven in, for the DSP_CTRL class
 * @lut_mode: the mode its LUT belongs to
 *
 * A RESET reads neither image: every pixel goes through one fixed phase
 * sequence and ends white. A FULL drives every pixel through its flash,
 * unchanged ones too, the way the cleanup FULL clears the DU ghosts. Both
 * need the full DSP_CTRL class (rk3026_ebc_dsp_ctrl_mode() gives
 * DISPLAY_MODE alone), which drives the whole frame. Any other waveform
 * (PART, DU, A2, OED_PART) moves only the pixels whose old grey, read from
 * @shadows.prev, differs from the new one, so it cannot make known a glass
 * that @shadows.prev_unknown says @shadows.prev does not describe.
 *
 * Return: true for a RESET or a FULL LUT driven in the full class.
 */
bool rk3026_ebc_drive_knows_glass(u32 mode, u32 lut_mode)
{
	if (rk3026_ebc_dsp_ctrl_mode(mode) != DSP_DISPLAY_MODE)
		return false;
	return lut_mode == EBC_MODE_RESET || lut_mode == EBC_MODE_FULL;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_drive_knows_glass);

/**
 * rk3026_ebc_du_spent - whether the DU budget is spent
 * @budget: the DU budget (struct rk3026_ebc_du_budget)
 *
 * Return: true once @budget->area reaches @budget->limit; never with a
 * limit of 0, which is no budget.
 */
bool rk3026_ebc_du_spent(const struct rk3026_ebc_du_budget *budget)
{
	return budget->limit && budget->area >= budget->limit;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_du_spent);

/**
 * rk3026_ebc_du_at_ceiling - whether the DU budget reached its ceiling
 * @budget: the DU budget (struct rk3026_ebc_du_budget)
 *
 * Return: true when the budget is spent (rk3026_ebc_du_spent()) and
 * @budget->area reached @budget->ceiling, a ceiling of 0 being none. A
 * ceiling below the limit so acts as the limit.
 */
bool rk3026_ebc_du_at_ceiling(const struct rk3026_ebc_du_budget *budget)
{
	return rk3026_ebc_du_spent(budget) && budget->ceiling &&
	       budget->area >= budget->ceiling;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_du_at_ceiling);

/**
 * rk3026_ebc_du_cleanup - whether an auto plane part takes the cleanup FULL
 * @budget: the DU budget at the take (struct rk3026_ebc_du_budget)
 * @changed_px: the pixels the part's snapshot changes
 *	(rk3026_ebc_build_snapshot())
 * @added_px: the pixels the part adds to the budget when it is driven as
 *	its content asks: @changed_px if two-level (DU or A2), else 0 (PART
 *	adds nothing; rk3026_ebc_du_account())
 * @reason: returns which cleanup, if one is due; left alone if not
 *
 * The budget is spent once @budget->area reaches @budget->limit. A spent
 * budget alone does not make the next part a FULL: that part is usually
 * small (a pointer move, a key typed), and a whole-panel flash on it is
 * the one the user notices most. It falls on a big part, one that changes
 * at least @budget->big_area pixels (EBC_REASON_CLEANUP_BIG): a window
 * opened or closed, a page scrolled, where the user expects the panel to
 * change anyway. A big_area of 0 makes every part big.
 *
 * Big is counted in changed pixels, not by the part's box: the box bounds
 * every damage clip of a commit and every update merged into the part, so
 * a pointer that jumps far, or two small changes far apart, would make a
 * big box of a few changed pixels. Changed pixels are what leaves ghosts:
 * a window opened over white that changes few of them adds few, and needs
 * no cleanup of its own; the idle cleanup clears them once the budget is
 * spent.
 *
 * A big part takes the FULL when the budget is spent before it, and also
 * when the part itself would spend it (@budget->area + @added_px, which
 * saturates, reaching the limit): driven DU, it would leave the budget
 * spent, and the cleanup would follow it as a second refresh for one
 * change.
 *
 * A session that only ever draws small parts would never clean up, so
 * the ceiling stops that: once the budget reaches it
 * (rk3026_ebc_du_at_ceiling()), any part takes the FULL
 * (EBC_REASON_CLEANUP_CEILING). A big part at the ceiling is reported
 * big: it would have taken the FULL anyway.
 *
 * A limit of 0 means no budget, and so no cleanup of any kind, the
 * ceiling included.
 *
 * Return: true when the part is driven FULL for a cleanup.
 */
bool rk3026_ebc_du_cleanup(const struct rk3026_ebc_du_budget *budget,
			   u32 changed_px, u32 added_px,
			   enum rk3026_ebc_reason *reason)
{
	u32 after;

	if (!budget->limit)
		return false;
	if (changed_px >= budget->big_area &&
	    (check_add_overflow(budget->area, added_px, &after) ||
	     after >= budget->limit)) {
		*reason = EBC_REASON_CLEANUP_BIG;
		return true;
	}
	if (rk3026_ebc_du_at_ceiling(budget)) {
		*reason = EBC_REASON_CLEANUP_CEILING;
		return true;
	}
	return false;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_du_cleanup);

/**
 * rk3026_ebc_resolve_mode - the mode the content picks for a part
 * @part: the part
 * @scan_mode: its scan mode (rk3026_ebc_scan_mode()), as the snapshot
 *	returned it
 * @two_level: the snapshot's two-level result (rk3026_ebc_build_snapshot())
 * @changed_px: the pixels the snapshot changes
 * @repair: the repair due before it (enum rk3026_ebc_repair_due)
 * @budget: the plane parts' DU cleanup budget (rk3026_ebc_du_account())
 * @a2: what a two-level plane part needs for A2 (struct rk3026_ebc_a2)
 * @reason: returns why the part is driven in the returned mode
 *
 * Only a part made of auto updates alone (@part->auto_ok) is the content's
 * to decide. If some pixel changes and every changed pixel becomes black or
 * white, it is driven in DU, which moves exactly those pixels and leaves the
 * rest of the glass alone. The snapshot built for @scan_mode stands as it
 * is: no changed pixel needs the two-level threshold.
 *
 * Such a part with a plane update in it, a pointer move or a key typed,
 * is driven in A2 instead when it is small, at most @a2->max_area changed
 * pixels: A2 is the shorter waveform, so the next move comes sooner, and
 * leaves more of a ghost. It needs no changed pixel over a grey of the
 * glass (@a2->over_grey): A2 drives only black and white and does not move
 * a grey at all, so such a part stays DU, which drives every level. A
 * waveform without an A2 table at the temperature hands out the DU table
 * for A2 (rk3026_ebc_waveform_lut()): the part is then drawn as DU, in the
 * DSP_CTRL class the two share, and never carried for it. A max_area of 0
 * is no A2: the default, until its ghosting is measured on the panel. A
 * part of mxcfb sends alone stays DU: such a client names A2 itself when
 * it wants it.
 *
 * Otherwise a part with a plane update in it is driven in PART. Like DU, it
 * moves only the pixels that change, so the box flashes and the rest of the
 * panel does not; unlike DU, it drives them to all 16 levels, a full
 * black/white transition included (why a plane part may go PART: see
 * EBC_MODE_DEFAULT in rk3026_ebc.c). The snapshot stands here too: a plane
 * update's own mode is FULL, so @scan_mode is a 16-level one and nothing
 * was thresholded. A part of mxcfb sends alone keeps @scan_mode, its
 * client's default.
 *
 * DU leaves the ghosts of what it draws over, and @budget counts them. An
 * auto part with a plane update in it, two-level or grey, keeps
 * @scan_mode, FULL, which clears the glass, when rk3026_ebc_du_cleanup()
 * says a cleanup is due: a big part (by its changed pixels) once the
 * budget is spent or when it would spend it, any part at its ceiling. A
 * small part on a spent budget stays DU or PART, so the flash does not
 * land on a pointer move. A part of mxcfb sends alone is not cleaned up:
 * such a client runs its own fulls.
 *
 * A part that has a mode of its own keeps @scan_mode, and so does one
 * behind a repair RESET: the glass is white then outside the box, and only
 * a 16-level scan draws the old greys back. Without a RESET at the
 * temperature (EBC_REPAIR_BY_FULL) the part itself makes the glass known:
 * it is driven FULL, whatever it asked (EBC_REASON_REPAIR_FLOOR if that
 * raised it), since a PART or DU scan moves only the pixels whose prev,
 * which is not evidence, differs (rk3026_ebc_drive_knows_glass()). The
 * FULL that ends clears the mark, so the part after it resolves as usual.
 *
 * The idle cleanup (@part->cleanup_idle, queued by the refresh thread when
 * the budget is spent and the plane has been idle) keeps @scan_mode, FULL,
 * whatever its content, its budget or a repair before it, for the reason
 * EBC_REASON_CLEANUP_IDLE; updates sent before it was taken join it and
 * are drawn in the same FULL.
 */
u32 rk3026_ebc_resolve_mode(const struct rk3026_ebc_set *part, u32 scan_mode,
			    bool two_level, u32 changed_px,
			    enum rk3026_ebc_repair_due repair,
			    const struct rk3026_ebc_du_budget *budget,
			    const struct rk3026_ebc_a2 *a2,
			    enum rk3026_ebc_reason *reason)
{
	if (part->cleanup_idle) {
		*reason = EBC_REASON_CLEANUP_IDLE;
		return scan_mode;
	}
	*reason = EBC_REASON_EXPLICIT;
	if (repair == EBC_REPAIR_BY_FULL) {
		if (part->mode != EBC_MODE_FULL)
			*reason = EBC_REASON_REPAIR_FLOOR;
		return EBC_MODE_FULL;
	}
	if (repair == EBC_REPAIR_BY_RESET) {
		if (scan_mode != part->mode)
			*reason = EBC_REASON_REPAIR_FLOOR;
		return scan_mode;
	}
	if (!part->auto_ok)
		return scan_mode;
	if (part->has_plane &&
	    rk3026_ebc_du_cleanup(budget, changed_px,
				  two_level ? changed_px : 0, reason))
		return scan_mode;
	if (two_level) {
		if (part->has_plane && a2->max_area &&
		    changed_px <= a2->max_area && !a2->over_grey) {
			*reason = EBC_REASON_AUTO_A2;
			return EBC_MODE_A2;
		}
		*reason = EBC_REASON_AUTO_DU;
		return EBC_MODE_BLACK_WHITE;
	}
	if (part->has_plane) {
		*reason = EBC_REASON_AUTO_PART;
		return EBC_MODE_PART;
	}
	return scan_mode;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_resolve_mode);

/**
 * rk3026_ebc_idle_cleanup_due - whether the refresh thread queues the idle
 *	cleanup
 * @idle: the state at @now (struct rk3026_ebc_idle)
 * @now: the time of the look, in jiffies
 * @expired: returns true when this look uses the arming up: armed, and the
 *	delay over since the last plane update (at once with no delay),
 *	whether the cleanup is due or not
 *
 * A small part on a spent budget stays DU or PART (rk3026_ebc_du_cleanup()),
 * so the glass keeps its DU ghosts until a big part or the ceiling comes.
 * When the plane goes idle first, the cleanup FULL is free to land: nobody
 * is drawing, and the next update starts on a clean glass. It is due once
 * the plane has been idle for @idle->delay since the update that armed it,
 * and only on a spent budget: a budget merely above zero (a keystroke and
 * a pause) would flash more often than the budget alone does.
 *
 * Not while anything else holds the glass (@idle->busy): a part pending
 * (it is drawn first; a RESET clears the glass anyway) or a carry waiting
 * for one, the CRTC off, an mxcfb client in front (it runs its own fulls),
 * or a repair due (the RESET of the next draw clears the glass). A delay
 * of 0 is no idle cleanup. A look that finds the delay over disarms the
 * cleanup whether it is due or not, so a cleanup that was not driven, or
 * did not clear the budget, does not fire again until the next plane
 * update arms it; and the thread, whose wait ends at the idle deadline,
 * never wakes for it twice.
 *
 * Return: true when the cleanup is due.
 */
bool rk3026_ebc_idle_cleanup_due(const struct rk3026_ebc_idle *idle,
				 unsigned long now, bool *expired)
{
	*expired = idle->armed &&
		   time_after_eq(now, idle->last_plane + idle->delay);
	return *expired && idle->delay && idle->spent && !idle->busy;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_idle_cleanup_due);

/* @at, or @deadline if @has and it comes first (time_after(), wrap-safe). */
static unsigned long ebc_earlier(bool has, unsigned long deadline,
				 unsigned long at)
{
	return has && time_after(at, deadline) ? deadline : at;
}

/**
 * rk3026_ebc_thread_deadline - when the refresh thread's wait ends
 * @s: what the pass read: the rails' off deadline while they are up
 *	(@rails_on, @rails_off_at), the idle cleanup's while it is armed
 *	(@idle_armed, @idle_at), and whether a carry waits for its retry
 *	(@retry_armed)
 * @retry_at: the retry's time, in jiffies (ebc_retry_jiffies()); only
 *	meaningful with @s->retry_armed
 * @deadline: returns the earliest of those that apply
 *
 * The thread waits for work until the next of its deadlines: the rails go
 * off at one, it looks at the idle cleanup at another, and it takes the
 * carry at its retry. All are jiffies, so they are compared with
 * time_after(), across a wrap too.
 *
 * Return: false, @deadline left alone, when none applies: the wait has no
 * timeout.
 */
bool rk3026_ebc_thread_deadline(const struct rk3026_ebc_pass *s,
				unsigned long retry_at, unsigned long *deadline)
{
	unsigned long d = 0;
	bool has = false;

	if (s->rails_on) {
		d = s->rails_off_at;
		has = true;
	}
	if (s->idle_armed) {
		d = ebc_earlier(has, d, s->idle_at);
		has = true;
	}
	if (s->retry_armed) {
		d = ebc_earlier(has, d, retry_at);
		has = true;
	}
	if (has)
		*deadline = d;
	return has;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_thread_deadline);

/**
 * ebc_retry_jiffies - the carry's retry time in jiffies
 * @now: the pass's time, in jiffies
 * @clock: rk3026_ebc_clock() of the same pass
 * @at: the retry's time, on rk3026_ebc_clock()
 *
 * For the deadline of the refresh thread's wait, whose other deadlines are
 * jiffies. At least a jiffy after @now, and rounded up: a wait that ends
 * before the retry finds it not due and waits again, a jiffy at least, so
 * the thread never spins on it.
 *
 * Return: the retry's time, in jiffies.
 */
static unsigned long ebc_retry_jiffies(unsigned long now, ktime_t clock,
				       ktime_t at)
{
	s64 us = ktime_us_delta(at, clock);

	return now + usecs_to_jiffies(clamp_t(s64, us, 1, U32_MAX));
}

/**
 * rk3026_ebc_thread_step - what one pass of the refresh thread does
 * @s: what the pass read (rk3026_ebc_thread_pass())
 * @timeout: returns the wait's timeout in jiffies for EBC_STEP_WAIT,
 *	MAX_SCHEDULE_TIMEOUT without a deadline
 *
 * In this order:
 *
 * - the rails go off at their deadline, checked on every pass, not only
 *   when the wait times out: a steady stream of skipped updates keeps the
 *   thread busy and must not keep the rails up. Not while the take's gap is
 *   open: the part taken after it will be driven (a gap opens only for
 *   such a part), and the gap's cap bounds the hold; but while the system
 *   suspends (@s->suspended) nothing is taken, so the gap holds nothing;
 * - while the system suspends, nothing else: no look at the idle cleanup,
 *   the next part or the carry's retry, which all wait for the resume, and
 *   the wait has the rails' deadline alone;
 * - the idle cleanup is looked at, at its deadline (rk3026_ebc_idle_tick()).
 *   Not while a gap is open: the part pending holds the glass, and its end
 *   arms the cleanup again;
 * - the next part is looked at (rk3026_ebc_take_ready()) while a gap is
 *   open, a part is pending or the carry's retry is due (@s->woken);
 * - else the thread waits until the earliest of the rails-off, idle and
 *   retry deadlines (rk3026_ebc_thread_deadline(),
 *   ebc_retry_jiffies()). Each is at least a jiffy ahead then (the
 *   two that are due were acted on above, and a gap is closed), so no
 *   pass spins.
 *
 * Return: the step (enum rk3026_ebc_step).
 */
enum rk3026_ebc_step rk3026_ebc_thread_step(const struct rk3026_ebc_pass *s,
					    long *timeout)
{
	unsigned long deadline, retry_at = 0;

	*timeout = MAX_SCHEDULE_TIMEOUT;
	if (s->rails_on && (!s->gap_open || s->suspended) &&
	    time_after_eq(s->now, s->rails_off_at))
		return EBC_STEP_RAILS_OFF;
	if (s->suspended) {
		if (s->rails_on)
			*timeout = (long)(s->rails_off_at - s->now);
		return EBC_STEP_WAIT;
	}
	if (s->idle_armed && !s->gap_open && time_after_eq(s->now, s->idle_at))
		return EBC_STEP_IDLE;
	if (s->gap_open || s->woken)
		return EBC_STEP_LOOK;

	if (s->retry_armed)
		retry_at = ebc_retry_jiffies(s->now, s->clock, s->retry_at);
	if (rk3026_ebc_thread_deadline(s, retry_at, &deadline))
		*timeout = (long)(deadline - s->now);
	return EBC_STEP_WAIT;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_thread_step);

/**
 * rk3026_ebc_take_wait - whether the take of the next part waits for a gap
 * @t: the next part (struct rk3026_ebc_take)
 * @now: the time of the look
 * @deadline: returns the gap's end when the take waits; left alone if not
 *
 * The thread wakes on the first update of a burst: an fbdev client fills
 * the panel in strips some ms apart, and a take on the first strip makes a
 * part of it alone, then one of the rest, two scans for one fill. A take
 * that waits until the senders have been quiet for @t->quiet_us after the
 * newest update gets the whole burst into one part.
 *
 * A lone small update must not wait: a pointer move or a key typed is the
 * latency the user feels. So a gap opens only for what is gathered anyway:
 * a burst (two updates or more), a big box (@t->big_area, as the DU
 * cleanup sizes a big part), a part that will be driven FULL, or any part
 * while the rails are down: the rails take tens of ms to come up, and the
 * gap opened then raises them at once (rk3026_ebc_take_ready()), so it ends
 * no earlier than a drive could start anyway, and a lone update gets no
 * latency the rails did not add already. With the rails up, a lone small
 * update is taken at once.
 *
 * Once open (@t->open), a gap runs to its end however small the part
 * stays, unless the part becomes exempt (an mxcfb send or a RESET is sent,
 * the carry's retry falls due): then it ends at once. It ends
 * @t->quiet_us after the newest update, or @t->cap_ms after the later of
 * the first update and the previous part's finish, whichever is earlier:
 * a steady stream is taken at the cap, and updates queued while a scan ran
 * do not use up the cap of the part they make. An armed retry
 * (@t->retry_at) ends it no later than its time: nothing else wakes the
 * thread's wait then, and the look at it finds the part exempt. The cap is at most
 * EBC_TAKE_CAP_MAX_MS and the quiet gap at most the cap: debugfs may set
 * either higher, and the rails would be held all that time.
 *
 * Never for an exempt part (@t->exempt: an mxcfb send, the idle cleanup, a
 * RESET pending, the carry's retry due), and never with a quiet gap or a
 * cap of 0.
 *
 * Return: true while the take waits, with @deadline the gap's end.
 */
bool rk3026_ebc_take_wait(const struct rk3026_ebc_take *t, ktime_t now,
			  ktime_t *deadline)
{
	ktime_t from = ktime_after(t->prev_done, t->first) ? t->prev_done :
							     t->first;
	u32 cap_ms = min_t(u32, t->cap_ms, EBC_TAKE_CAP_MAX_MS);
	u32 quiet_us = min_t(u32, t->quiet_us, cap_ms * USEC_PER_MSEC);
	ktime_t end, cap;

	if (!quiet_us || t->exempt)
		return false;
	if (!t->open && t->rails_up && t->count < 2 &&
	    t->area < t->big_area && !t->full)
		return false;

	end = ktime_add_us(t->last, quiet_us);
	cap = ktime_add_ms(from, cap_ms);
	if (ktime_before(cap, end))
		end = cap;
	if (t->retry_armed && ktime_before(t->retry_at, end))
		end = t->retry_at;
	if (!ktime_before(now, end))
		return false;
	*deadline = end;
	return true;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_take_wait);

/**
 * rk3026_ebc_gap_timeout - how long the refresh thread sleeps in the gap
 * @at: the gap's end (rk3026_ebc_take_wait()'s @deadline)
 * @now: the time the wait starts
 * @left: returns the time to @at when the wait sleeps; left alone if not
 *
 * The gap's end is set at the look, and the look that opens the gap raises
 * the rails before the wait (rk3026_ebc_take_ready()): their ramp, tens of
 * ms, may outlast the gap, so its end may be past when the wait starts.
 * The wait must not sleep then: a relative hrtimer with a negative expiry
 * saturates to KTIME_SEC_MAX (ktime_add_safe()), and the thread would sleep
 * until the next update or the CRTC going off, the part pending all along.
 * The next look takes it.
 *
 * Return: true to sleep for @left, which is positive; false when @at is
 * @now or past: look again at once.
 */
bool rk3026_ebc_gap_timeout(ktime_t at, ktime_t now, ktime_t *left)
{
	if (!ktime_before(now, at))
		return false;
	*left = ktime_sub(at, now);
	return true;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_gap_timeout);

/**
 * rk3026_ebc_du_account - the DU cleanup budget after one part
 * @du_area: the pixels plane parts changed in DU or A2 since the glass was
 *	cleared
 * @part: the part, as rk3026_ebc_pending_take() gave it
 * @mode: the mode it was driven in (force_mode included)
 * @changed_px: the pixels its snapshot changed (rk3026_ebc_build_snapshot())
 * @outcome: what became of it
 * @repair: the repair RESET run before it, and how it ended
 *
 * The budget counts what the glass went through, so it changes only for a
 * part whose waveform was entered (DRIVEN, TIMEOUT, ABANDONED), and for a
 * RESET:
 *
 * - a DU or A2 part with a plane update in it adds @changed_px
 *   (saturating): both leave a ghost of what they draw over, A2 the
 *   stronger one, counted the same for now;
 * - a FULL part clears it, whoever sent it, the idle cleanup too: the
 *   controller scans the whole frame, and GC16 drives every pixel,
 *   unchanged ones too;
 * - a PART part leaves it, a grey plane part too: it drives only the pixels
 *   that change, to all 16 levels, so it adds no DU ghost and clears none
 *   of the others;
 * - a RESET part clears it whatever its outcome: one that was not driven
 *   leaves the glass unknown, and the next draw repairs it first;
 * - a repair RESET that ran to its end clears it too, whatever became of
 *   the part after it: the glass is white.
 *
 * A part that was skipped, dropped or carried, or whose repair RESET was cut
 * short, leaves it as it is, so a carried part counts once, when it is
 * drawn. So does an mxcfb DU send: such a client runs its own fulls.
 *
 * Returns the new @du_area.
 */
u32 rk3026_ebc_du_account(u32 du_area, const struct rk3026_ebc_set *part,
			  u32 mode, u32 changed_px,
			  enum rk3026_ebc_outcome outcome,
			  enum rk3026_ebc_repair repair)
{
	if (part->mode == EBC_MODE_RESET)
		return 0;
	if (repair == EBC_REPAIR_FAILED)
		return du_area;
	if (repair == EBC_REPAIR_DONE)
		du_area = 0;

	switch (outcome) {
	case EBC_OUTCOME_DRIVEN:
	case EBC_OUTCOME_TIMEOUT:
	case EBC_OUTCOME_ABANDONED:
		break;
	default:
		return du_area;
	}

	if (mode == EBC_MODE_FULL)
		return 0;
	if (!ebc_mode_two_level(mode) || !part->has_plane)
		return du_area;
	if (check_add_overflow(du_area, changed_px, &du_area))
		return U32_MAX;
	return du_area;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_du_account);

/*
 * A part not driven is carried in its plain mode: the carry's @auto_ok is
 * cleared, which is conservative (rk3026_ebc_resolve_mode() keeps the mode
 * as it is).
 *
 * The idle cleanup alone (@cleanup_idle, one update) is not carried: it
 * draws nothing of its own, and carried it would turn the next part, a
 * pointer move as a rule, into a whole-panel FULL; with no FULL table at
 * the temperature every later part would merge into it and none be drawn.
 * The budget stays spent, so the next plane update arms the cleanup again.
 * Updates that joined it are carried, in the whole panel's box and FULL as
 * they were merged, but no longer as the cleanup.
 *
 * The carry keeps the part's marker, source and temperature, its newest
 * update's: a try alone is traced as the part was, and driven at the
 * temperature an update brought rather than the sensor's.
 */
static bool ebc_carry(struct rk3026_ebc_pending *p,
		      const struct rk3026_ebc_set *part)
{
	if (part->cleanup_idle && part->merged == 1)
		return false;
	ebc_set_merge(&p->carry, !p->carry.merged, part);
	p->carry.auto_ok = false;
	p->carry.cleanup_idle = false;
	p->carry.seq = part->seq;
	p->carry.marker = part->marker;
	p->carry.source = part->source;
	if (part->temp_valid) {
		p->carry.temp = part->temp;
		p->carry.temp_valid = true;
	}
	p->carry.merged = 1;
	return true;
}

/**
 * rk3026_ebc_retry_next - when a carried part is tried again
 * @first: when it was first carried
 * @now: when its last try failed
 * @at: returns the time of the next try
 *
 * The tries are fixed from @first: at 100, 500 and 2100 ms
 * (EBC_RETRY_FIRST_MS, EBC_RETRY_SECOND_MS, EBC_RETRY_SPAN_MS), each wait
 * four times the one before, so a rail that comes back soon is caught
 * soon, and one that stays dead costs three tries. The next is the first
 * of them after @now: a try made in between, with an update that joined
 * the carry, does not move them.
 *
 * Return: true with @at the next try; false once the last one is past:
 * the part is given up.
 */
bool rk3026_ebc_retry_next(ktime_t first, ktime_t now, ktime_t *at)
{
	static const u16 ms[EBC_RETRY_TRIES] = {
		EBC_RETRY_FIRST_MS, EBC_RETRY_SECOND_MS, EBC_RETRY_SPAN_MS,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ms); i++) {
		ktime_t t = ktime_add_ms(first, ms[i]);

		if (ktime_after(t, now)) {
			*at = t;
			return true;
		}
	}
	return false;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_retry_next);

/**
 * rk3026_ebc_retry_ready - the refresh thread's look at the carry's retry
 * @p: pending state
 * @now: the time of the look
 * @crtc_on: the CRTC is on
 *
 * Sets @p->retry.ready when the retry is due (rk3026_ebc_retry_due()), so
 * that the carry alone is the next part (rk3026_ebc_pending_peek()); with
 * the CRTC off at once, and the take drops it. A retry armed for a carry
 * that a RESET sent has dropped (rk3026_ebc_pending_add()) ends here: the
 * RESET's finish moves @done_seq over its updates.
 */
void rk3026_ebc_retry_ready(struct rk3026_ebc_pending *p, ktime_t now,
			    bool crtc_on)
{
	struct rk3026_ebc_retry *r = &p->retry;

	if (!p->carry.merged) {
		WRITE_ONCE(r->armed, false);
		r->tries = 0;
	}
	r->ready = rk3026_ebc_retry_due(r, now, crtc_on);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_retry_ready);

/*
 * What becomes of a part's waits (rk3026_ebc_scan_finish()), @carried: the
 * part was carried.
 *
 * A part not carried finishes: driven, skipped, dropped. A carry is tried
 * again on the schedule of rk3026_ebc_retry_next(), from its first carry,
 * and @done_seq stays behind it meanwhile; past the last try, or at once
 * without a waveform (it is loaded once, at probe), it is given up and its
 * updates finish. Given up, it stays carried for the next part, as before
 * the retries. After a give-up, a carry gives up at once until a part is
 * driven: a dead rail costs one schedule, not one per keystroke.
 *
 * Returns whether the part was given up.
 */
static enum rk3026_ebc_gave_up ebc_retry_settle(struct rk3026_ebc_pending *p,
						const struct rk3026_ebc_set *part,
						bool carried,
						enum rk3026_ebc_outcome outcome,
						ktime_t now)
{
	struct rk3026_ebc_retry *r = &p->retry;
	enum rk3026_ebc_gave_up gave_up = EBC_GAVE_UP_NONE;
	ktime_t at;

	if (outcome == EBC_OUTCOME_DRIVEN)
		r->failing = false;
	if (carried && outcome != EBC_OUTCOME_NO_WAVEFORM && !r->failing) {
		if (r->armed) {
			r->tries++;
		} else {
			r->first = now;
			r->tries = 0;
		}
		/* Read without the lock by the refresh thread's wait. */
		if (rk3026_ebc_retry_next(r->first, now, &at)) {
			WRITE_ONCE(r->at, at);
			WRITE_ONCE(r->armed, true);
			return EBC_GAVE_UP_NONE;
		}
	}
	if (carried) {
		gave_up = r->failing ? EBC_GAVE_UP_FAILING : EBC_GAVE_UP;
		r->failing = true;
	}
	WRITE_ONCE(r->armed, false);
	r->tries = 0;
	/* Read without the lock by the waits on it. */
	WRITE_ONCE(p->done_seq, part->seq);
	return gave_up;
}

/**
 * rk3026_ebc_scan_finish - account for one part after its scan
 * @sh: the shadow images
 * @p: pending state
 * @part: the part, as rk3026_ebc_pending_take() gave it
 * @outcome: what became of it
 * @repair: the repair RESET run before it, and how it ended
 * @now: the time of the finish (rk3026_ebc_clock()), for a carry's retry
 * @gave_up: returns whether the part was given up (enum
 *	rk3026_ebc_gave_up)
 *
 * @done_seq moves to the part's seq, so every wait on an update in it
 * ends, when the part was driven (to its end or not), skipped or dropped.
 * A part carried for want of rails or a LUT, or behind a repair cut short,
 * is tried again by the refresh thread first (@p->retry), and its waits end
 * when a try draws it or when it is given up; one carried without a
 * waveform is given up at once (ebc_retry_settle()). Then:
 *
 * - the repair RESET was cut short (EBC_REPAIR_FAILED; @outcome is its
 *   TIMEOUT or ABANDONED): the part's own waveform never ran and the glass
 *   is not known. @prev stays, @prev_unknown is set, and the part is
 *   carried, so the scan that repairs the glass draws it as well.
 * - the part's waveform was entered (DRIVEN, TIMEOUT, ABANDONED): the
 *   snapshot is what was driven in, so @scan and @prev swap; a RESET instead
 *   leaves @prev white, its target, also when it was cut short: the next
 *   draw repairs the glass and lays its box over that white, as the target
 *   image has it. A waveform cut short sets @prev_unknown; one that ended
 *   leaves it as rk3026_ebc_drive() left it, which alone clears it, and
 *   only for a drive that makes the glass known
 *   (rk3026_ebc_drive_knows_glass()): a PART or DU scan over a glass that
 *   is not known leaves it set.
 * - SKIPPED: nothing differs.
 * - NO_RAILS, NO_LUT, NO_WAVEFORM: nothing was driven, so the part is
 *   carried into the next one, except the idle cleanup alone (see
 *   ebc_carry()). A RESET is not carried: @prev_unknown makes the next draw
 *   repair the glass, and the RESET is given up (EBC_GAVE_UP), so a dead
 *   rail at a CRTC enable is counted. If a repair did run
 *   (EBC_REPAIR_DONE), the glass is white now: @prev is too, and the whole
 *   panel is carried, at least as PART, the idle cleanup alone too: the
 *   white glass must be drawn. The repair's own drive cleared
 *   @prev_unknown.
 * - CRTC_OFF: dropped; the RESET of the next CRTC enable clears the glass.
 *
 * A RESET part drops a carry made while it was pending (the part in flight
 * when the RESET was sent), as a RESET sent drops one: it whitens the glass
 * the carry would draw, and its finish ends the carry's waits.
 *
 * Returns true if @prev changed, so the scanned prev image must follow it.
 */
bool rk3026_ebc_scan_finish(struct rk3026_ebc_shadows *sh,
			    struct rk3026_ebc_pending *p,
			    const struct rk3026_ebc_set *part,
			    enum rk3026_ebc_outcome outcome,
			    enum rk3026_ebc_repair repair, ktime_t now,
			    enum rk3026_ebc_gave_up *gave_up)
{
	size_t size = (size_t)sh->pitch * sh->height;
	bool reset = part->mode == EBC_MODE_RESET;
	bool carried = false, changed = false, lost = false;
	enum rk3026_ebc_gave_up end;
	struct rk3026_ebc_set whole;

	if (reset)
		memset(&p->carry, 0, sizeof(p->carry));

	if (repair == EBC_REPAIR_FAILED && !reset) {
		sh->prev_unknown = true;
		carried = ebc_carry(p, part);
		goto settle;
	}

	switch (outcome) {
	case EBC_OUTCOME_DRIVEN:
	case EBC_OUTCOME_TIMEOUT:
	case EBC_OUTCOME_ABANDONED:
		if (outcome != EBC_OUTCOME_DRIVEN)
			sh->prev_unknown = true;
		if (!reset)
			swap(sh->prev, sh->scan);
		else
			memset(sh->prev, 0xff, size);
		changed = true;
		break;

	case EBC_OUTCOME_NO_RAILS:
	case EBC_OUTCOME_NO_LUT:
	case EBC_OUTCOME_NO_WAVEFORM:
		if (reset) {
			sh->prev_unknown = true;
			lost = true;
			break;
		}
		if (repair != EBC_REPAIR_DONE) {
			carried = ebc_carry(p, part);
			break;
		}
		whole = *part;
		whole.box = DRM_RECT_INIT(0, 0, sh->pitch * 2, sh->height);
		whole.mode = ebc_mode_max(part->mode, EBC_MODE_PART);
		whole.cleanup_idle = false;
		carried = ebc_carry(p, &whole);
		memset(sh->prev, 0xff, size);
		changed = true;
		break;

	default:
		break;
	}

settle:
	end = ebc_retry_settle(p, part, carried, outcome, now);
	*gave_up = lost ? EBC_GAVE_UP : end;
	return changed;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_scan_finish);

/**
 * rk3026_ebc_marker_seq_record - remember which seq a marker was queued with
 * @ring: marker ring
 * @marker: the client's marker; 0 is never recorded
 * @seq: the update's seq
 *
 * The oldest record is overwritten, and a wait on it then returns at once.
 */
void rk3026_ebc_marker_seq_record(struct rk3026_ebc_marker_ring *ring,
				  u32 marker, u32 seq)
{
	unsigned int slot;

	if (!marker)
		return;
	slot = ring->next++ % EBC_MARKER_RING_LEN;
	ring->marker[slot] = marker;
	ring->seq[slot] = seq;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_marker_seq_record);

/**
 * rk3026_ebc_marker_seq_lookup - the seq of the newest update with a marker
 * @ring: marker ring
 * @marker: the client's marker
 * @seq: returns the seq
 *
 * Returns false for marker 0 and for a marker not in the ring (never sent,
 * or overwritten since).
 */
bool rk3026_ebc_marker_seq_lookup(const struct rk3026_ebc_marker_ring *ring,
				  u32 marker, u32 *seq)
{
	unsigned int i;

	if (!marker)
		return false;
	for (i = 1; i <= EBC_MARKER_RING_LEN; i++) {
		unsigned int slot = (ring->next - i) % EBC_MARKER_RING_LEN;

		if (ring->marker[slot] == marker) {
			*seq = ring->seq[slot];
			return true;
		}
	}
	return false;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_marker_seq_lookup);

/**
 * rk3026_ebc_vblank_mode_clock - the mode clock that reports a tick rate
 * @hz: the tick rate, clamped here to EBC_VBLANK_HZ_MIN..EBC_VBLANK_HZ_MAX:
 *	0 is the slowest tick, not none
 * @htotal: the mode's htotal
 * @vtotal: the mode's vtotal
 *
 * The connector's one mode describes the tick, not the controller's timing:
 * the dot clock comes from the DT's clock-frequency, and nothing reads the
 * mode's clock for it, while drm_mode_vrefresh() and the vblank core's
 * frame duration are derived from it. The clock is rounded to the nearest
 * kHz, which is off from @hz * @htotal * @vtotal / 1000 by at most 0.5 kHz,
 * so drm_mode_vrefresh() gives @hz back whenever @htotal * @vtotal is above
 * 1000. This is the one clamp of the rate: debugfs stores what is written.
 *
 * Return: the clock in kHz, at least 1 (a mode with clock 0 is invalid);
 * at most 65535 * 65535 * EBC_VBLANK_HZ_MAX / 1000, well within an int.
 */
int rk3026_ebc_vblank_mode_clock(u32 hz, u16 htotal, u16 vtotal)
{
	u64 px = mul_u32_u32(htotal, vtotal) *
		 clamp(hz, EBC_VBLANK_HZ_MIN, EBC_VBLANK_HZ_MAX);

	return max_t(u64, DIV_ROUND_CLOSEST_ULL(px, 1000), 1);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_vblank_mode_clock);
