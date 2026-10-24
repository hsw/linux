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
#include <linux/minmax.h>
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
 * Merge a box, a mode and the full-class flag into @set: the bounding box,
 * the stronger mode, either flag. An @empty set takes the box and the mode
 * as they are.
 */
static void ebc_set_merge(struct rk3026_ebc_set *set, bool empty,
			  const struct drm_rect *box, u32 mode,
			  bool has_mxcfb_full)
{
	if (empty) {
		set->box = *box;
		set->mode = mode;
	} else {
		ebc_box_union(&set->box, box);
		set->mode = ebc_mode_max(set->mode, mode);
	}
	set->has_mxcfb_full |= has_mxcfb_full;
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
	bool full = upd->source == EBC_SOURCE_MXCFB &&
		    rk3026_ebc_dsp_ctrl_mode(upd->mode) == DSP_DISPLAY_MODE;

	ebc_set_merge(set, !set->merged, &upd->clip, upd->mode, full);
	if (upd->temp_valid) {
		set->temp = upd->temp;
		set->temp_valid = true;
	}
	set->seq = seq;
	set->marker = upd->marker;
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
 * carry for the same reason.
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
 * rk3026_ebc_pending_take - take the next part for one scan, in seq order
 * @p: pending state
 * @part: returns the part
 *
 * @pre comes first, with the carry merged in. Then the RESET alone, after
 * which @post becomes @pre: everything left was queued after the RESET.
 *
 * Returns false, with @part cleared, if nothing is pending.
 */
bool rk3026_ebc_pending_take(struct rk3026_ebc_pending *p,
			     struct rk3026_ebc_set *part)
{
	if (rk3026_ebc_pending_empty(p)) {
		memset(part, 0, sizeof(*part));
		return false;
	}

	if (p->pre.merged) {
		*part = p->pre;
		memset(&p->pre, 0, sizeof(p->pre));
		if (p->carry.merged) {
			ebc_set_merge(part, false, &p->carry.box, p->carry.mode,
				      p->carry.has_mxcfb_full);
			memset(&p->carry, 0, sizeof(p->carry));
		}
		return true;
	}

	*part = p->reset;
	p->pre = p->post;
	memset(&p->reset, 0, sizeof(p->reset));
	memset(&p->post, 0, sizeof(p->post));
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
 * @two_level and @changed_px describe the update, not the scan: they are
 * taken on @next before the threshold and on the pixels where it differs
 * from @prev, the widened edges included; greys that do not change do not
 * count. A two-level update can be driven in DU with the same snapshot,
 * as no changed pixel needs the threshold.
 *
 * Returns true if the snapshot differs from @prev, false if the scan would
 * move nothing.
 */
bool rk3026_ebc_build_snapshot(const struct rk3026_ebc_shadows *sh,
			       const u8 *next, const struct drm_rect *box,
			       u32 *mode, bool *two_level, u32 *changed_px)
{
	const u8 *prev = sh->prev;
	u32 pitch = sh->pitch, height = sh->height;
	u8 *dst = sh->scan;
	u32 tl = ebc_mode_two_level(*mode) ? ~0u : 0;
	struct ebc_snap_acc acc = { 0 };
	struct drm_rect r = *box;
	int y;

	*two_level = false;
	*changed_px = 0;
	r.x1 = max(r.x1, 0) & ~1;
	r.x2 = min_t(int, round_up(r.x2, 2), (int)(pitch * 2));
	r.y1 = max(r.y1, 0);
	r.y2 = min_t(int, r.y2, (int)height);
	if (r.x1 >= r.x2 || r.y1 >= r.y2) {
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
	*changed_px = acc.count;
	return acc.moves != 0;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_build_snapshot);

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
 */
bool rk3026_ebc_should_skip(const struct rk3026_ebc_set *part,
			    bool prev_unknown, bool identical)
{
	return identical && !prev_unknown && part->mode != EBC_MODE_RESET &&
	       !part->has_mxcfb_full;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_should_skip);

/**
 * rk3026_ebc_scan_mode - the mode a part is driven in
 * @part: the part
 * @repaired: a repair RESET runs before it
 *
 * After a repair the glass is white outside the box too, while the snapshot
 * there still holds the old greys, and a two-level LUT does not drive white
 * to grey: such a scan runs at least PART.
 */
u32 rk3026_ebc_scan_mode(const struct rk3026_ebc_set *part, bool repaired)
{
	if (repaired && part->mode != EBC_MODE_RESET &&
	    rk3026_ebc_mode_rank(part->mode) <
	    rk3026_ebc_mode_rank(EBC_MODE_PART))
		return EBC_MODE_PART;
	return part->mode;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_scan_mode);

static void ebc_carry(struct rk3026_ebc_pending *p, const struct drm_rect *box,
		      u32 mode, bool has_mxcfb_full)
{
	ebc_set_merge(&p->carry, !p->carry.merged, box, mode, has_mxcfb_full);
	p->carry.merged = 1;
}

/**
 * rk3026_ebc_scan_finish - account for one part after its scan
 * @sh: the shadow images
 * @p: pending state
 * @part: the part, as rk3026_ebc_pending_take() gave it
 * @outcome: what became of it
 * @repair: the repair RESET run before it, and how it ended
 *
 * @done_seq moves to the part's seq whatever the outcome, so every wait on
 * an update in it ends. Then:
 *
 * - the repair RESET was cut short (EBC_REPAIR_FAILED; @outcome is its
 *   TIMEOUT or ABANDONED): the part's own waveform never ran and the glass
 *   is not known. @prev stays, @prev_unknown is set, and the part is
 *   carried, so the scan that repairs the glass draws it as well.
 * - the part's waveform was entered (DRIVEN, TIMEOUT, ABANDONED): the
 *   snapshot is what was driven in, so @scan and @prev swap; a RESET instead
 *   leaves @prev white, its target, also when it was cut short: the next
 *   draw repairs the glass and lays its box over that white, as the target
 *   image has it. @prev_unknown says whether every frame ran.
 * - SKIPPED: nothing differs.
 * - NO_RAILS, NO_LUT, NO_WAVEFORM: nothing was driven, so the part is
 *   carried into the next one. A RESET is not carried: @prev_unknown makes
 *   the next draw repair the glass. If a repair did run (EBC_REPAIR_DONE),
 *   the glass is white now: @prev is too, and the whole panel is carried,
 *   at least as PART.
 * - CRTC_OFF: dropped; the RESET of the next CRTC enable clears the glass.
 *
 * Returns true if @prev changed, so the scanned prev image must follow it.
 */
bool rk3026_ebc_scan_finish(struct rk3026_ebc_shadows *sh,
			    struct rk3026_ebc_pending *p,
			    const struct rk3026_ebc_set *part,
			    enum rk3026_ebc_outcome outcome,
			    enum rk3026_ebc_repair repair)
{
	size_t size = (size_t)sh->pitch * sh->height;
	bool reset = part->mode == EBC_MODE_RESET;
	struct drm_rect panel;

	/* Read without the lock by the waits on it. */
	WRITE_ONCE(p->done_seq, part->seq);

	if (repair == EBC_REPAIR_FAILED && !reset) {
		sh->prev_unknown = true;
		ebc_carry(p, &part->box, part->mode, part->has_mxcfb_full);
		return false;
	}

	switch (outcome) {
	case EBC_OUTCOME_DRIVEN:
	case EBC_OUTCOME_TIMEOUT:
	case EBC_OUTCOME_ABANDONED:
		sh->prev_unknown = outcome != EBC_OUTCOME_DRIVEN;
		if (!reset) {
			swap(sh->prev, sh->scan);
			return true;
		}
		memset(sh->prev, 0xff, size);
		return true;

	case EBC_OUTCOME_NO_RAILS:
	case EBC_OUTCOME_NO_LUT:
	case EBC_OUTCOME_NO_WAVEFORM:
		if (reset) {
			sh->prev_unknown = true;
			return false;
		}
		if (repair != EBC_REPAIR_DONE) {
			ebc_carry(p, &part->box, part->mode,
				  part->has_mxcfb_full);
			return false;
		}
		panel = DRM_RECT_INIT(0, 0, sh->pitch * 2, sh->height);
		ebc_carry(p, &panel, ebc_mode_max(part->mode, EBC_MODE_PART),
			  part->has_mxcfb_full);
		memset(sh->prev, 0xff, size);
		sh->prev_unknown = false;
		return true;

	default:
		return false;
	}
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
