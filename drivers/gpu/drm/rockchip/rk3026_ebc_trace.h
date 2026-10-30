/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Tracepoints of the RK3026 EBC driver: what was queued, what was driven, what
 * was skipped, how each part ended, when the panel rails switched and when a
 * held flip event was sent. Mode numbers are the EBC work modes of
 * rk3026_ebc.h (EBC_MODE_*). A part's events carry the seq of its newest
 * update; new keys go at the end of a line.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM rk3026_ebc

#if !defined(_RK3026_EBC_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _RK3026_EBC_TRACE_H

#include <linux/tracepoint.h>

#include "rk3026_ebc.h"

TRACE_DEFINE_ENUM(EBC_SOURCE_PLANE);
TRACE_DEFINE_ENUM(EBC_SOURCE_MXCFB);
TRACE_DEFINE_ENUM(EBC_SOURCE_ENABLE);
TRACE_DEFINE_ENUM(EBC_SOURCE_IDLE);
TRACE_DEFINE_ENUM(EBC_SOURCE_RESTORE);
TRACE_DEFINE_ENUM(EBC_OUTCOME_NONE);
TRACE_DEFINE_ENUM(EBC_OUTCOME_DRIVEN);
TRACE_DEFINE_ENUM(EBC_OUTCOME_SKIPPED);
TRACE_DEFINE_ENUM(EBC_OUTCOME_CRTC_OFF);
TRACE_DEFINE_ENUM(EBC_OUTCOME_NO_WAVEFORM);
TRACE_DEFINE_ENUM(EBC_OUTCOME_NO_RAILS);
TRACE_DEFINE_ENUM(EBC_OUTCOME_NO_LUT);
TRACE_DEFINE_ENUM(EBC_OUTCOME_TIMEOUT);
TRACE_DEFINE_ENUM(EBC_OUTCOME_ABANDONED);
TRACE_DEFINE_ENUM(EBC_FLIP_REASON_TAKE);
TRACE_DEFINE_ENUM(EBC_FLIP_REASON_WATCHDOG);
TRACE_DEFINE_ENUM(EBC_FLIP_REASON_OFF);
TRACE_DEFINE_ENUM(EBC_FLIP_REASON_SLEEP);
TRACE_DEFINE_ENUM(EBC_FLIP_REASON_REPLACED);
TRACE_DEFINE_ENUM(EBC_REASON_EXPLICIT);
TRACE_DEFINE_ENUM(EBC_REASON_AUTO_DU);
TRACE_DEFINE_ENUM(EBC_REASON_REPAIR_FLOOR);
TRACE_DEFINE_ENUM(EBC_REASON_CLEANUP_BIG);
TRACE_DEFINE_ENUM(EBC_REASON_AUTO_PART);
TRACE_DEFINE_ENUM(EBC_REASON_CLEANUP_CEILING);
TRACE_DEFINE_ENUM(EBC_REASON_CLEANUP_IDLE);
TRACE_DEFINE_ENUM(EBC_REASON_AUTO_A2);

#define ebc_trace_mode(m)						\
	__print_symbolic(m,						\
			 { EBC_MODE_AUTO,		"auto" },	\
			 { EBC_MODE_FULL,		"full" },	\
			 { EBC_MODE_A2,			"a2" },		\
			 { EBC_MODE_PART,		"part" },	\
			 { EBC_MODE_FULL_DITHER,	"full_dither" },\
			 { EBC_MODE_RESET,		"reset" },	\
			 { EBC_MODE_BLACK_WHITE,	"black_white" },\
			 { EBC_MODE_TEXT,		"text" },	\
			 { EBC_MODE_BLOCK,		"block" },	\
			 { EBC_MODE_FULL_WIN,		"full_win" },	\
			 { EBC_MODE_OED_PART,		"oed_part" },	\
			 { EBC_MODE_DIRECT_PART,	"direct_part" },\
			 { EBC_MODE_DIRECT_A2,		"direct_a2" })

#define ebc_trace_source(s)						\
	__print_symbolic(s,						\
			 { EBC_SOURCE_PLANE,	"plane" },		\
			 { EBC_SOURCE_MXCFB,	"mxcfb" },		\
			 { EBC_SOURCE_ENABLE,	"enable" },		\
			 { EBC_SOURCE_IDLE,	"idle" },		\
			 { EBC_SOURCE_RESTORE,	"restore" })

#define ebc_trace_outcome(o)						\
	__print_symbolic(o,						\
			 { EBC_OUTCOME_NONE,		"none" },	\
			 { EBC_OUTCOME_DRIVEN,		"driven" },	\
			 { EBC_OUTCOME_SKIPPED,		"identical" },	\
			 { EBC_OUTCOME_CRTC_OFF,	"crtc_off" },	\
			 { EBC_OUTCOME_NO_WAVEFORM,	"no_waveform" },\
			 { EBC_OUTCOME_NO_RAILS,	"no_rails" },	\
			 { EBC_OUTCOME_NO_LUT,		"no_lut" },	\
			 { EBC_OUTCOME_TIMEOUT,		"timeout" },	\
			 { EBC_OUTCOME_ABANDONED,	"abandoned" })

#define ebc_trace_flip_reason(r)					\
	__print_symbolic(r,						\
			 { EBC_FLIP_REASON_TAKE,	"take" },	\
			 { EBC_FLIP_REASON_WATCHDOG,	"watchdog" },	\
			 { EBC_FLIP_REASON_OFF,		"off" },	\
			 { EBC_FLIP_REASON_SLEEP,	"sleep" },	\
			 { EBC_FLIP_REASON_REPLACED,	"replaced" })

TRACE_EVENT(rk3026_ebc_update_queue,
	TP_PROTO(u32 source, u32 mode, const struct drm_rect *clip, u32 marker,
		 int temp, bool temp_valid, bool coalesced, unsigned int queued),
	TP_ARGS(source, mode, clip, marker, temp, temp_valid, coalesced, queued),
	TP_STRUCT__entry(
		__field(u32, source)
		__field(u32, mode)
		__field(int, x1)
		__field(int, y1)
		__field(int, x2)
		__field(int, y2)
		__field(u32, marker)
		__field(int, temp)
		__field(bool, temp_valid)
		__field(bool, coalesced)
		__field(unsigned int, queued)
	),
	TP_fast_assign(
		__entry->source = source;
		__entry->mode = mode;
		__entry->x1 = clip->x1;
		__entry->y1 = clip->y1;
		__entry->x2 = clip->x2;
		__entry->y2 = clip->y2;
		__entry->marker = marker;
		__entry->temp = temp;
		__entry->temp_valid = temp_valid;
		__entry->coalesced = coalesced;
		__entry->queued = queued;
	),
	TP_printk("source=%s mode=%u(%s) clip=%d,%d-%d,%d marker=%u temp=%d temp_valid=%d coalesced=%d queued=%u",
		  ebc_trace_source(__entry->source), __entry->mode,
		  ebc_trace_mode(__entry->mode), __entry->x1, __entry->y1,
		  __entry->x2, __entry->y2, __entry->marker, __entry->temp,
		  __entry->temp_valid, __entry->coalesced, __entry->queued)
);

#define ebc_trace_reason(r)						\
	__print_symbolic(r,						\
			 { EBC_REASON_EXPLICIT,		"explicit" },	\
			 { EBC_REASON_AUTO_DU,		"auto_du" },	\
			 { EBC_REASON_REPAIR_FLOOR,	"repair_floor" },\
			 { EBC_REASON_CLEANUP_BIG,	"cleanup_big" },\
			 { EBC_REASON_AUTO_PART,	"auto_part" },	\
			 { EBC_REASON_CLEANUP_CEILING,	"cleanup_ceiling" },\
			 { EBC_REASON_CLEANUP_IDLE,	"cleanup_idle" },\
			 { EBC_REASON_AUTO_A2,		"auto_a2" })

/*
 * One waveform starts. @reason is why the part is driven in its mode
 * (rk3026_ebc_resolve_mode()); the repair RESET in front of a part (@repair)
 * carries the part's reason, seq, box and counts. The part's fields are
 * appended to the line: its newest update's seq, its box in panel
 * coordinates, the pixels its snapshot changes, the DU budget spent at its
 * take, the time from the take to this waveform (@setup_us,
 * rk3026_ebc_setup_us()), the time the take waited for a quiet gap, the
 * retries of a carried part and the take's own time (taken_us, the
 * monotonic clock in microseconds, as the trace_clock "mono" stamps the
 * events: a flip event sent at the take is not before it).
 */
TRACE_EVENT(rk3026_ebc_scan_start,
	TP_PROTO(const struct rk3026_ebc_last *last, u32 mode, u32 lut_mode,
		 u32 frames, bool repair, u32 setup_us),
	TP_ARGS(last, mode, lut_mode, frames, repair, setup_us),
	TP_STRUCT__entry(
		__field(u32, mode)
		__field(u32, lut_mode)
		__field(int, temp)
		__field(u32, frames)
		__field(u32, marker)
		__field(u32, merged)
		__field(bool, repair)
		__field(u32, reason)
		__field(u32, seq)
		__field(int, x1)
		__field(int, y1)
		__field(int, x2)
		__field(int, y2)
		__field(u32, changed_px)
		__field(u32, du_area)
		__field(u32, setup_us)
		__field(u32, gap_us)
		__field(u32, tries)
		__field(s64, taken_us)
	),
	TP_fast_assign(
		__entry->mode = mode;
		__entry->lut_mode = lut_mode;
		__entry->temp = last->temp;
		__entry->frames = frames;
		__entry->marker = last->marker;
		__entry->merged = last->merged;
		__entry->repair = repair;
		__entry->reason = last->reason;
		__entry->seq = last->seq;
		__entry->x1 = last->clip.x1;
		__entry->y1 = last->clip.y1;
		__entry->x2 = last->clip.x2;
		__entry->y2 = last->clip.y2;
		__entry->changed_px = last->changed_px;
		__entry->du_area = last->du_area;
		__entry->setup_us = setup_us;
		__entry->gap_us = last->gap_us;
		__entry->tries = last->tries;
		__entry->taken_us = ktime_to_us(last->taken);
	),
	TP_printk("mode=%u(%s) lut_mode=%u(%s) temp=%d frames=%u marker=%u merged=%u repair=%d reason=%s seq=%u box=%d,%d-%d,%d changed_px=%u du_area=%u setup_us=%u gap_us=%u tries=%u taken_us=%lld",
		  __entry->mode, ebc_trace_mode(__entry->mode),
		  __entry->lut_mode, ebc_trace_mode(__entry->lut_mode),
		  __entry->temp, __entry->frames, __entry->marker,
		  __entry->merged, __entry->repair,
		  ebc_trace_reason(__entry->reason), __entry->seq,
		  __entry->x1, __entry->y1, __entry->x2, __entry->y2,
		  __entry->changed_px, __entry->du_area, __entry->setup_us,
		  __entry->gap_us, __entry->tries, __entry->taken_us)
);

/* A waveform ends; @seq is its part's (a repair RESET's too). */
TRACE_EVENT(rk3026_ebc_scan_end,
	TP_PROTO(u32 mode, u32 frames, u32 driven, s64 ms, int ret, u32 seq),
	TP_ARGS(mode, frames, driven, ms, ret, seq),
	TP_STRUCT__entry(
		__field(u32, mode)
		__field(u32, frames)
		__field(u32, driven)
		__field(s64, ms)
		__field(int, ret)
		__field(u32, seq)
	),
	TP_fast_assign(
		__entry->mode = mode;
		__entry->frames = frames;
		__entry->driven = driven;
		__entry->ms = ms;
		__entry->ret = ret;
		__entry->seq = seq;
	),
	TP_printk("mode=%u(%s) driven=%u/%u ms=%lld ret=%d seq=%u",
		  __entry->mode, ebc_trace_mode(__entry->mode),
		  __entry->driven, __entry->frames, __entry->ms, __entry->ret,
		  __entry->seq)
);

/*
 * A part was taken and nothing was driven for it: skipped as identical,
 * dropped or carried (the reason). Its seq, box, the pixels its snapshot
 * changes (0 if none was built) and the take's time (taken_us, as in
 * rk3026_ebc_scan_start) are appended.
 */
TRACE_EVENT(rk3026_ebc_skip,
	TP_PROTO(const struct rk3026_ebc_last *last),
	TP_ARGS(last),
	TP_STRUCT__entry(
		__field(u32, mode)
		__field(u32, marker)
		__field(u32, reason)
		__field(u32, seq)
		__field(int, x1)
		__field(int, y1)
		__field(int, x2)
		__field(int, y2)
		__field(u32, changed_px)
		__field(s64, taken_us)
	),
	TP_fast_assign(
		__entry->mode = last->asked_mode;
		__entry->marker = last->marker;
		__entry->reason = last->outcome;
		__entry->seq = last->seq;
		__entry->x1 = last->clip.x1;
		__entry->y1 = last->clip.y1;
		__entry->x2 = last->clip.x2;
		__entry->y2 = last->clip.y2;
		__entry->changed_px = last->changed_px;
		__entry->taken_us = ktime_to_us(last->taken);
	),
	TP_printk("mode=%u(%s) marker=%u reason=%s seq=%u box=%d,%d-%d,%d changed_px=%u taken_us=%lld",
		  __entry->mode, ebc_trace_mode(__entry->mode),
		  __entry->marker, ebc_trace_outcome(__entry->reason),
		  __entry->seq, __entry->x1, __entry->y1, __entry->x2,
		  __entry->y2, __entry->changed_px, __entry->taken_us)
);

/*
 * A part is finished, whatever became of it: after its last scan_end or its
 * skip, with the DU budget spent after it (@du_area) and the retries of a
 * carried part (@last's tries, gave_up).
 */
TRACE_EVENT(rk3026_ebc_part_done,
	TP_PROTO(const struct rk3026_ebc_last *last, u32 du_area),
	TP_ARGS(last, du_area),
	TP_STRUCT__entry(
		__field(u32, seq)
		__field(u32, outcome)
		__field(u32, du_area)
		__field(u32, tries)
		__field(bool, gave_up)
	),
	TP_fast_assign(
		__entry->seq = last->seq;
		__entry->outcome = last->outcome;
		__entry->du_area = du_area;
		__entry->tries = last->tries;
		__entry->gave_up = last->gave_up;
	),
	TP_printk("seq=%u outcome=%s du_area=%u tries=%u gave_up=%d",
		  __entry->seq, ebc_trace_outcome(__entry->outcome),
		  __entry->du_area, __entry->tries, __entry->gave_up)
);

TRACE_EVENT(rk3026_ebc_rails,
	TP_PROTO(bool on, bool epd_on, bool vcom_on, int ret),
	TP_ARGS(on, epd_on, vcom_on, ret),
	TP_STRUCT__entry(
		__field(bool, on)
		__field(bool, epd_on)
		__field(bool, vcom_on)
		__field(int, ret)
	),
	TP_fast_assign(
		__entry->on = on;
		__entry->epd_on = epd_on;
		__entry->vcom_on = vcom_on;
		__entry->ret = ret;
	),
	TP_printk("%s epd=%d vcom=%d ret=%d", __entry->on ? "on" : "off",
		  __entry->epd_on, __entry->vcom_on, __entry->ret)
);

/*
 * A flip event held for its part (rk3026_ebc_flip_hold()) was sent: the seq
 * of the update its commit queued, how long it was held, and why now (enum
 * rk3026_ebc_flip_reason): its part was taken (take), the watchdog sent it
 * late, the CRTC went off, a system sleep began, or a second hold found the
 * slot full (replaced: the core gave up waiting for a commit).
 */
TRACE_EVENT(rk3026_ebc_flip_event,
	TP_PROTO(u32 seq, u32 held_us, u32 why),
	TP_ARGS(seq, held_us, why),
	TP_STRUCT__entry(
		__field(u32, seq)
		__field(u32, held_us)
		__field(u32, why)
	),
	TP_fast_assign(
		__entry->seq = seq;
		__entry->held_us = held_us;
		__entry->why = why;
	),
	TP_printk("seq=%u held_us=%u why=%s", __entry->seq, __entry->held_us,
		  ebc_trace_flip_reason(__entry->why))
);

#endif /* _RK3026_EBC_TRACE_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE rk3026_ebc_trace
#include <trace/define_trace.h>
