/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Tracepoints of the RK3026 EBC driver: what was queued, what was driven, what
 * was skipped and when the panel rails switched. Mode numbers are the EBC work
 * modes of rk3026_ebc.h (EBC_MODE_*).
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
TRACE_DEFINE_ENUM(EBC_OUTCOME_SKIPPED);
TRACE_DEFINE_ENUM(EBC_OUTCOME_CRTC_OFF);
TRACE_DEFINE_ENUM(EBC_OUTCOME_NO_WAVEFORM);
TRACE_DEFINE_ENUM(EBC_OUTCOME_NO_RAILS);
TRACE_DEFINE_ENUM(EBC_OUTCOME_NO_LUT);
TRACE_DEFINE_ENUM(EBC_REASON_EXPLICIT);
TRACE_DEFINE_ENUM(EBC_REASON_AUTO_DU);
TRACE_DEFINE_ENUM(EBC_REASON_REPAIR_FLOOR);
TRACE_DEFINE_ENUM(EBC_REASON_CLEANUP);
TRACE_DEFINE_ENUM(EBC_REASON_AUTO_PART);

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
			 { EBC_SOURCE_ENABLE,	"enable" })

#define ebc_trace_outcome(o)						\
	__print_symbolic(o,						\
			 { EBC_OUTCOME_SKIPPED,		"identical" },	\
			 { EBC_OUTCOME_CRTC_OFF,	"crtc_off" },	\
			 { EBC_OUTCOME_NO_WAVEFORM,	"no_waveform" },\
			 { EBC_OUTCOME_NO_RAILS,	"no_rails" },	\
			 { EBC_OUTCOME_NO_LUT,		"no_lut" })

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
			 { EBC_REASON_CLEANUP,		"cleanup" },	\
			 { EBC_REASON_AUTO_PART,	"auto_part" })

/*
 * One waveform starts. @reason is why the part is driven in its mode
 * (rk3026_ebc_resolve_mode()); the repair RESET in front of a part (@repair)
 * carries the part's reason.
 */
TRACE_EVENT(rk3026_ebc_scan_start,
	TP_PROTO(u32 mode, u32 lut_mode, int temp, u32 frames, u32 marker,
		 u32 merged, bool repair, u32 reason),
	TP_ARGS(mode, lut_mode, temp, frames, marker, merged, repair, reason),
	TP_STRUCT__entry(
		__field(u32, mode)
		__field(u32, lut_mode)
		__field(int, temp)
		__field(u32, frames)
		__field(u32, marker)
		__field(u32, merged)
		__field(bool, repair)
		__field(u32, reason)
	),
	TP_fast_assign(
		__entry->mode = mode;
		__entry->lut_mode = lut_mode;
		__entry->temp = temp;
		__entry->frames = frames;
		__entry->marker = marker;
		__entry->merged = merged;
		__entry->repair = repair;
		__entry->reason = reason;
	),
	TP_printk("mode=%u(%s) lut_mode=%u(%s) temp=%d frames=%u marker=%u merged=%u repair=%d reason=%s",
		  __entry->mode, ebc_trace_mode(__entry->mode),
		  __entry->lut_mode, ebc_trace_mode(__entry->lut_mode),
		  __entry->temp, __entry->frames, __entry->marker,
		  __entry->merged, __entry->repair,
		  ebc_trace_reason(__entry->reason))
);

TRACE_EVENT(rk3026_ebc_scan_end,
	TP_PROTO(u32 mode, u32 frames, u32 driven, s64 ms, int ret),
	TP_ARGS(mode, frames, driven, ms, ret),
	TP_STRUCT__entry(
		__field(u32, mode)
		__field(u32, frames)
		__field(u32, driven)
		__field(s64, ms)
		__field(int, ret)
	),
	TP_fast_assign(
		__entry->mode = mode;
		__entry->frames = frames;
		__entry->driven = driven;
		__entry->ms = ms;
		__entry->ret = ret;
	),
	TP_printk("mode=%u(%s) driven=%u/%u ms=%lld ret=%d",
		  __entry->mode, ebc_trace_mode(__entry->mode),
		  __entry->driven, __entry->frames, __entry->ms, __entry->ret)
);

TRACE_EVENT(rk3026_ebc_skip,
	TP_PROTO(u32 mode, u32 marker, u32 reason),
	TP_ARGS(mode, marker, reason),
	TP_STRUCT__entry(
		__field(u32, mode)
		__field(u32, marker)
		__field(u32, reason)
	),
	TP_fast_assign(
		__entry->mode = mode;
		__entry->marker = marker;
		__entry->reason = reason;
	),
	TP_printk("mode=%u(%s) marker=%u reason=%s",
		  __entry->mode, ebc_trace_mode(__entry->mode),
		  __entry->marker, ebc_trace_outcome(__entry->reason))
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

#endif /* _RK3026_EBC_TRACE_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE rk3026_ebc_trace
#include <trace/define_trace.h>
