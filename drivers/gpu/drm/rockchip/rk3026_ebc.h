/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Rockchip RK3026 EBC (E-Ink Broadcaster) register map and driver-internal API.
 *
 * Base 0x10114000, size 16 KiB, GIC SPI 49. The shared display pads are routed
 * to EBC (instead of LCDC) by setting bit 0 of GRF register 0x150.
 */
#ifndef _RK3026_EBC_H
#define _RK3026_EBC_H

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/ktime.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder.h>
#include <drm/drm_plane.h>
#include <drm/drm_rect.h>

#include <video/display_timing.h>

/* Register offsets within the EBC MMIO window. */
#define EBC_DSP_START		0x00
#define EBC_EPD_CTRL		0x04
#define EBC_DSP_CTRL		0x08
#define EBC_DSP_HTIMING0	0x0c
#define EBC_DSP_HTIMING1	0x10
#define EBC_DSP_VTIMING0	0x14
#define EBC_DSP_VTIMING1	0x18
#define EBC_DSP_ACT_INFO	0x1c
#define EBC_WIN_CTRL		0x20
#define EBC_WIN_MST0		0x24
#define EBC_WIN_MST1		0x28
#define EBC_WIN_VIR		0x2c
#define EBC_WIN_ACT		0x30
#define EBC_WIN_DSP		0x34
#define EBC_WIN_DSP_ST		0x38
#define EBC_INT			0x3c
#define EBC_VCOM0		0x40
#define EBC_VCOM1		0x44
#define EBC_VCOM2		0x48
#define EBC_VCOM3		0x4c
#define EBC_CONFIG_DONE		0x50

/*
 * Waveform LUT SRAM, mapped inside the same window at +0x1000: 0x40 frames of
 * 0x40 bytes. Each frame is 16 words indexed by the old grey level, 2 bits per
 * new grey level. A waveform longer than 0x40 frames is sent in chunks.
 */
#define EBC_LUT_BASE		0x1000
#define EBC_LUT_FRAME_BYTES	0x40
#define EBC_LUT_MAX_FRAMES	0x40
/* A waveform's frame count is one byte of its container segment. */
#define EBC_WAVEFORM_MAX_FRAMES	255

/* EBC_DSP_START. */
#define DSP_FRAME_START		BIT(0)
#define DSP_FRAME_NUM_MASK	(0x3fu << 2)
#define DSP_FRAME_NUM(x)	(((x) & 0x3f) << 2)

/*
 * EBC_EPD_CTRL. SDCK counts source-driver clocks per line (width/4,
 * saturated), LGONL and GDCK_STA are the E Ink gate timings offset by
 * hsync_len.
 */
#define EPD_SOURCE_SCAN_DIR	BIT(0)
#define EPD_GATE_SCAN_DIR	BIT(1)
#define EPD_SDCK(x)		(((x) & 0xff) << 8)
#define EPD_LGONL(x)		(((x) & 0x7ff) << 16)
#define EPD_GDCK_STA(x)		(((x) & 0x1f) << 27)

/* EBC_WIN_CTRL: win enable, AHB burst, 4bpp format. */
#define WIN_CTRL_FIXED		0x20fc0u

/* EBC_INT. Status bit 1 is frame end. */
#define EBC_INT_FRAME_END	BIT(1)
#define EBC_INT_DISPLAYEND_MASK	BIT(4)
#define EBC_INT_FRAMEEND_CLEAR	BIT(6)
#define EBC_INT_DISPLAYEND_CLEAR BIT(7)
#define EBC_INT_FRAMEFLAG_CLEAR	BIT(8)
#define EBC_INT_FRAME_NUM(x)	(((x) & 0x7f) << 9)
/* Bits 3+5, set by every frame start. */
#define EBC_INT_ENABLE		0x28u

/* EBC_DSP_CTRL fixed top bits. */
#define DSP_CTRL_FIXED		0xc0030000u
#define DSP_VCOM_MODE		BIT(27)
#define DSP_DISPLAY_MODE	BIT(28)
#define DSP_UPDATE_MODE		BIT(29)

/* Latch value written to EBC_CONFIG_DONE after each register group. */
#define EBC_CONFIG_DONE_VAL	0xffff

/*
 * GRF 0x150 -- /proc/iomem calls this region "rk3026-lvds", and it carries
 * three bits this driver must ESTABLISH rather than inherit.
 *
 * bit 0  EBC vs LCDC on the shared pads.
 * bit 8  LVDS PLL power-down.
 * bit 9  LVDS-vs-LVTTL output select.
 *
 * 🔴 Bits 8 and 9 are REQUIRED, measured 2026-08-06. The driver used to write
 * bit 0 alone, and the panel rendered anyway on earlier boots, where this
 * register read 0x301. On a boot from a freshly written card it read 0x101 and
 * nothing drew; a single `devmem 0x20008150 32 0x03010301` and the same circle
 * appeared. One variable, same boot. Proven to be established rather than found
 * by forcing the register to 0x001, hot-swapping this driver, and reading 0x301
 * back after probe.
 *
 * ⚠️ WHERE THE 0x300 CAME FROM ON THOSE EARLIER BOOTS IS NOT ESTABLISHED, and a
 * previous revision of this comment asserted it was Android. That was never
 * checked and the maintainer had deliberately not been booting Android. What IS
 * ruled out: this driver (it wrote bit 0 only) and U-Boot (it never references
 * 0x150 -- grepped). What remains open: persistence from some earlier state
 * (INDEX records GRF surviving a warm SND reset and being cleared by U-Boot's
 * FST reset), or a reset default that differs by boot path. Do not repeat the
 * Android story as if it were the mechanism.
 *
 * ⚠️ This is still the class of defect docs/PENDING.md P7 names -- a driver that
 * works by inheriting state from whatever ran before is not working -- and the
 * fix is the same whatever the source turns out to be: write the bits.
 *
 * ⚠️ And do not trust an earlier note of mine calling these bits a no-op. They
 * read back already-set, which is why they looked inert; the test could not
 * distinguish "does nothing" from "already set by something else".
 */
#define GRF_EBC_SEL_REG		0x150
#define GRF_EBC_SEL_BIT		BIT(0)
#define GRF_LVDS_PLL_PD_BIT	BIT(8)
#define GRF_LVDS_OUT_SEL_BIT	BIT(9)
#define GRF_EBC_SEL_MASK	(GRF_EBC_SEL_BIT | GRF_LVDS_PLL_PD_BIT | \
				 GRF_LVDS_OUT_SEL_BIT)

/*
 * GRF pad-mux registers for the eight EBC pads, read back only for reporting.
 * Two bits per pin at 0xa8 + 4 * group; bank 2 groups B, C and D.
 */
#define GRF_IOMUX_GPIO2B	0xcc
#define GRF_IOMUX_GPIO2C	0xd0
#define GRF_IOMUX_GPIO2D	0xd4

/*
 * Source-driver clock: 306 x 769 x 85 Hz = 20 MHz, which is the figure the
 * panel table carries and what DRM's mode->clock describes.
 *
 * dclk_ebc runs at four times that. The vendor stores 20 MHz in its panel
 * info and then shifts it left by two before clk_set_rate (rk29_ebc_probe:
 * "ldr r1,[r4,#144]; lsl r1,r1,#2"), because DSP_CTRL's fixed
 * SCLK_DIVIDE_RATE field divides the dot clock by four. Programming 20 MHz
 * here would run every waveform at a quarter of its intended frame rate.
 */
/*
 * Fallback only. The real source-driver clock comes from the DEVICE TREE --
 * `panel-timing { clock-frequency }`, which of_get_display_timing() parses into
 * timing.pixelclock -- because it is a property of the panel and the board, not
 * of this driver. This constant is used only if the DT omits it, and 20 MHz is
 * the figure the vendor's panel table carries.
 */
#define EBC_PIXEL_RATE_FALLBACK	20000000

/*
 * There is deliberately NO EBC_DCLK_RATE here.
 *
 * It existed briefly on 2026-08-06 as a compiled-in 12000000 and was wrong in
 * principle before it was wrong in practice: the rate is a property of this
 * panel on this board, so it belongs in the device tree. It now comes from
 * `panel-timing { clock-frequency }` via ebc_dclk_rate(), which is x4 the source
 * clock. The measurement that set the value, and why 12 MHz is a WORKAROUND
 * rather than the answer, is recorded beside the property in
 * dts/rk3026-onyx-c67ml-magellan.dts.
 *
 * ⚠️ Do not reintroduce a constant here to "override" the DT. Two sources for
 * one number is how the driver came to ask for 12 MHz while the hardware ran at
 * 66, and how the dts said 20 MHz while nothing read it.
 */

/*
 * EBC work modes. These index the controller's mode dispatch and select both
 * the DSP_CTRL class and the waveform table; the numbering is the vendor
 * driver's and is baked into the mode-mask constants below.
 */
#define EBC_MODE_AUTO		0
#define EBC_MODE_FULL		1
#define EBC_MODE_A2		2
#define EBC_MODE_PART		3
#define EBC_MODE_FULL_DITHER	4
#define EBC_MODE_RESET		5
#define EBC_MODE_BLACK_WHITE	6
#define EBC_MODE_TEXT		7
#define EBC_MODE_BLOCK		8
#define EBC_MODE_FULL_WIN	9
#define EBC_MODE_OED_PART	10
#define EBC_MODE_DIRECT_PART	11
#define EBC_MODE_DIRECT_A2	12
#define EBC_MODE_MAX		12

/* Temperatures above this are replaced by 25 degrees before bucket lookup. */
#define EBC_TEMP_MAX		50
#define EBC_TEMP_FALLBACK	25

struct clk;
struct dentry;
struct fb_info;
struct fb_ops;
struct firmware;
struct regmap;
struct thermal_zone_device;

/* Who sent an update: debugfs "state" and the update_queue tracepoint. */
enum rk3026_ebc_source {
	EBC_SOURCE_PLANE,	/* a plane update: fbcon or a KMS client */
	EBC_SOURCE_MXCFB,	/* MXCFB_SEND_UPDATE */
	EBC_SOURCE_ENABLE,	/* the RESET of a CRTC enable */
	EBC_SOURCE_IDLE,	/* the idle cleanup the refresh thread queues */
	EBC_SOURCE_RESTORE,	/* the redraw after a hibernation restore */
};

/* What became of one part (the updates one scan takes). */
enum rk3026_ebc_outcome {
	EBC_OUTCOME_NONE,	/* nothing taken yet */
	EBC_OUTCOME_DRIVEN,	/* every frame of its waveform driven */
	EBC_OUTCOME_SKIPPED,	/* image identical to prev: nothing driven */
	EBC_OUTCOME_CRTC_OFF,	/* dropped: the CRTC is off */
	EBC_OUTCOME_NO_WAVEFORM, /* dropped: no waveform loaded */
	EBC_OUTCOME_NO_RAILS,	/* dropped: the rails did not come up */
	EBC_OUTCOME_NO_LUT,	/* dropped: no LUT for the mode and temperature */
	EBC_OUTCOME_TIMEOUT,	/* a chunk timed out (repair or update) */
	EBC_OUTCOME_ABANDONED,	/* the rails were down before a chunk */
};

/* The repair RESET a scan runs before its part when the glass is not known. */
enum rk3026_ebc_repair {
	EBC_REPAIR_NONE,	/* none ran: not due, or no RESET waveform */
	EBC_REPAIR_DONE,	/* every frame driven: the glass is white */
	EBC_REPAIR_FAILED,	/* cut short: the part was not driven */
};

/*
 * The repair due before a part when the glass is not known, as the mode
 * policy sees it (rk3026_ebc_resolve_mode()): what the waveform has at the
 * panel's temperature decides it.
 */
enum rk3026_ebc_repair_due {
	EBC_REPAIR_NOT_DUE,	/* glass known, a RESET part, or no RESET nor FULL */
	EBC_REPAIR_BY_RESET,	/* a RESET runs before the part */
	EBC_REPAIR_BY_FULL,	/* no RESET at the temperature: the part runs FULL */
};

/*
 * Why a part is driven in the mode it is (rk3026_ebc_resolve_mode()), for the
 * scan_start tracepoint.
 */
enum rk3026_ebc_reason {
	EBC_REASON_EXPLICIT,	/* the part's own mode, as asked or by default */
	EBC_REASON_AUTO_DU,	/* an auto part whose changes are all 0 or 15 */
	EBC_REASON_REPAIR_FLOOR, /* raised by a repair: PART after its RESET, FULL without one */
	EBC_REASON_CLEANUP_BIG,	/* a big auto plane part kept FULL: spent, or spent by it */
	EBC_REASON_AUTO_PART,	/* an auto plane part with a grey change */
	EBC_REASON_CLEANUP_CEILING, /* ceiling: any auto plane part FULL */
	EBC_REASON_CLEANUP_IDLE, /* the idle cleanup's own FULL */
	EBC_REASON_AUTO_A2,	/* a small two-level auto plane part, in A2 */
};

/*
 * The plane parts' DU cleanup budget as rk3026_ebc_du_cleanup() reads it:
 * @area spent of @limit (0: no cleanup at all), the changed pixels that
 * make a part big (@big_area) and the @ceiling at which any part cleans up
 * (0: none). The refresh thread fills it from @du_area and the
 * du_cleanup_* fields of struct rk3026_ebc.
 */
struct rk3026_ebc_du_budget {
	u32	area;
	u32	limit;
	u32	big_area;
	u32	ceiling;
};

/*
 * The idle delay before the cleanup FULL of a spent DU budget
 * (rk3026_ebc_idle_tick()): long enough that a pause between keystrokes or
 * pointer moves does not flash the panel, short enough that the glass is
 * clean when the user looks at it. debugfs du_cleanup_idle_ms changes it.
 */
#define EBC_DU_CLEANUP_IDLE_MS	2000

/*
 * What rk3026_ebc_resolve_mode() needs to drive a two-level auto plane part
 * in A2 rather than DU:
 *
 * @max_area: the most pixels such a part may change (0: never A2);
 * @over_grey: some pixel the part changes is grey on the glass
 *	(rk3026_ebc_build_snapshot()).
 */
struct rk3026_ebc_a2 {
	u32	max_area;
	bool	over_grey;
};

/*
 * The idle cleanup as rk3026_ebc_idle_cleanup_due() reads it, at one look
 * of the refresh thread under @image_lock (rk3026_ebc_idle_tick()):
 *
 * @armed: a plane update was queued or drawn since the last look that
 *	found the delay over (struct rk3026_ebc, @idle_armed);
 * @last_plane: when the newest plane update was queued or drawn, in
 *	jiffies;
 * @delay: the idle delay in jiffies, 0: no idle cleanup;
 * @spent: the DU budget is spent (rk3026_ebc_du_spent());
 * @busy: something else holds the glass: a part pending (a RESET
 *	included) or a carry waiting, the CRTC off, an mxcfb client in
 *	front, or a repair due (@shadows.prev_unknown).
 */
struct rk3026_ebc_idle {
	unsigned long	last_plane;
	unsigned long	delay;
	bool		armed;
	bool		spent;
	bool		busy;
};

/*
 * The take's quiet gap and its cap as probe sets them; the rules are
 * rk3026_ebc_take_wait()'s. debugfs take_quiet_us (0: no gap) and
 * take_cap_ms change them, used up to EBC_TAKE_CAP_MAX_MS for the cap and
 * the cap for the quiet gap: the rails are held while a gap is open, and
 * a wait on an update counts one gap (rk3026_ebc_wait_timeout_ms()).
 */
#define EBC_TAKE_QUIET_US	10000
#define EBC_TAKE_CAP_MS		250
#define EBC_TAKE_CAP_MAX_MS	1000

/*
 * The next part as rk3026_ebc_take_wait() reads it, under @image_lock:
 *
 * @first, @last: when its first and newest update were queued;
 * @prev_done: when the previous part finished; the cap counts from the
 *	later of it and @first, so updates queued during a scan do not use
 *	up the cap of the part they make;
 * @count: the updates in it; @area: the pixels of its box;
 * @big_area: the box that makes it big (@du_cleanup_big_area);
 * @quiet_us: the quiet gap, 0: none; @cap_ms: the longest wait; both as
 *	debugfs set them, clamped where they are used;
 * @full: it will be driven FULL, as far as is known before its snapshot:
 *	force_mode, an auto plane part at the DU ceiling;
 * @exempt: it never waits: an mxcfb send is in it (its client batches its
 *	own updates and waits on the marker), it is the idle cleanup (the
 *	whole panel already), a RESET is pending (the part is the RESET, or
 *	the one in front of it, which nothing can join), or the carry's
 *	retry is due (@retry.ready): the part is that try, late already;
 * @rails_up: the panel rails are up;
 * @open: a gap is open for it already;
 * @retry_armed, @retry_at: the carry's retry is armed, for @retry_at. A gap
 *	opens only for a part with @pre in it, which holds the carry, and
 *	the retry falling due makes it exempt: the gap ends there at the
 *	latest, so the thread's wait wakes for it.
 */
struct rk3026_ebc_take {
	ktime_t		first;
	ktime_t		last;
	ktime_t		prev_done;
	u32		count;
	u32		area;
	u32		big_area;
	u32		quiet_us;
	u32		cap_ms;
	bool		full;
	bool		exempt;
	bool		rails_up;
	bool		open;
	ktime_t		retry_at;
	bool		retry_armed;
};

/*
 * The take's quiet gap, the refresh thread's alone (rk3026_ebc_take_ready()):
 * open since @start, until @at unless a new update moves it; @seen is the
 * newest seq at the last look, so the thread's wait wakes on a newer one
 * (rk3026_ebc_gap_woken()); @us is the length of the gap that ended at
 * the last take, for that part's record. @rails_err is the error of the
 * rails raised at the gap's open, if they did not come up: the take does
 * not ask for them again, the carry's retry does.
 */
struct rk3026_ebc_gap {
	ktime_t		start;
	ktime_t		at;
	u32		seen;
	u32		us;
	int		rails_err;
	bool		open;
};

/*
 * One pass of the refresh thread, as it read it without the lock
 * (rk3026_ebc_thread_pass()): @now in jiffies and @clock
 * (rk3026_ebc_clock()) of the same pass; the rails up (@rails_on) and their
 * off deadline; the idle cleanup armed and its deadline; the carry's retry
 * armed and its time, on @clock; the take's gap open; the system
 * suspending (@suspended); and the wait's condition (@woken:
 * rk3026_ebc_thread_woken()).
 */
struct rk3026_ebc_pass {
	unsigned long	now;
	unsigned long	rails_off_at;
	unsigned long	idle_at;
	ktime_t		clock;
	ktime_t		retry_at;
	bool		rails_on;
	bool		idle_armed;
	bool		retry_armed;
	bool		gap_open;
	bool		suspended;
	bool		woken;
};

/* What a pass of the refresh thread does (rk3026_ebc_thread_step()). */
enum rk3026_ebc_step {
	EBC_STEP_RAILS_OFF,	/* the rails go off, at their deadline */
	EBC_STEP_IDLE,		/* the idle cleanup's look, at its deadline */
	EBC_STEP_LOOK,		/* the next part's look (rk3026_ebc_take_ready()) */
	EBC_STEP_WAIT,		/* a wait for work, until the next deadline */
};

/*
 * The retries of a carried part: EBC_RETRY_TRIES, at these times after the
 * first carry, the last at EBC_RETRY_SPAN_MS (rk3026_ebc_retry_next()).
 */
#define EBC_RETRY_TRIES		3
#define EBC_RETRY_FIRST_MS	100
#define EBC_RETRY_SECOND_MS	500
#define EBC_RETRY_SPAN_MS	2100

/*
 * The retry of the carry (@carry of struct rk3026_ebc_pending), under
 * @image_lock and written by the refresh thread alone, which also reads
 * @armed and @at without the lock, for its wait:
 *
 * @armed: the carry is tried again at @at, and @done_seq stays below its
 *	updates until it is driven, skipped, dropped or given up;
 * @first: when it was first carried: the schedule counts from it, and
 *	updates that join the carry later do not move it;
 * @tries: the tries since then;
 * @ready: the retry is due at the thread's look (rk3026_ebc_retry_ready()):
 *	the carry alone is the next part (rk3026_ebc_pending_peek());
 * @failing: a carry was given up and no part was driven since, so a carry
 *	gives up at once, without retries.
 */
struct rk3026_ebc_retry {
	ktime_t		first;
	ktime_t		at;
	u32		tries;
	bool		armed;
	bool		ready;
	bool		failing;
};

/*
 * Whether a part was given up (rk3026_ebc_scan_finish()): not drawn and not
 * tried again, its waits ended. A carry past its last retry, or at once
 * without a waveform or while a failure lasts (@retry.failing); and a
 * RESET that was not driven, whose glass the next draw repairs.
 */
enum rk3026_ebc_gave_up {
	EBC_GAVE_UP_NONE,	/* driven, skipped, dropped, or its retry armed */
	EBC_GAVE_UP,		/* given up, the first since a part was driven */
	EBC_GAVE_UP_FAILING,	/* given up at once: a carry was given up already */
};

/*
 * One update as a sender hands it over (rk3026_ebc_add_update()): its region,
 * EBC work mode, MXCFB_WAIT_FOR_UPDATE_COMPLETE marker (0: none), the panel
 * temperature it brings (@temp, if @temp_valid) and who sent it.
 *
 * @auto_mode: the sender asked for no particular waveform, so @mode is only
 * the default the content may override (rk3026_ebc_resolve_mode()). A plane
 * update and an mxcfb AUTO + PARTIAL send set it; an update that names its
 * mode leaves it false.
 *
 * @blit_us: what its blit took (rk3026_ebc_blit_xrgb8888()), 0 without one.
 *
 * @queued: when it was queued (rk3026_ebc_clock()), set by
 * rk3026_ebc_add_update(), for the take's quiet gap (rk3026_ebc_take_wait()).
 */
struct rk3026_ebc_update {
	struct drm_rect	clip;
	ktime_t		queued;
	u32		mode;
	u32		marker;
	u32		blit_us;
	int		temp;
	bool		temp_valid;
	bool		auto_mode;
	u8		source;		/* enum rk3026_ebc_source */
};

/*
 * The record of one part, for debugfs and the tracepoints. Filled by the
 * refresh thread and published under @image_lock. The _us times (debugfs
 * "timing") are rounded up and at least 1 when measured; 0 means none.
 *
 * @taken is when the part was taken (rk3026_ebc_clock()); a scan_start
 * event reports the time from it to the waveform (the snapshot, a repair
 * RESET before the part; the sensor and the rails come before the take,
 * rk3026_ebc_do_refresh()).
 * @du_area is the plane parts' DU budget spent at the take, before
 * the part is accounted (rk3026_ebc_du_account()). @gap_us and @tries are
 * the time the take waited for the senders to go quiet and the retries of
 * a carried part; @gave_up says the part was given up without a drive
 * (enum rk3026_ebc_gave_up).
 */
struct rk3026_ebc_last {
	struct drm_rect	clip;
	ktime_t		taken;
	u32		seq;		/* the newest update's */
	u32		asked_mode;	/* the part's own mode */
	u32		mode;		/* the DSP_CTRL class driven (force_mode) */
	u32		lut_mode;	/* the mode whose LUT was driven */
	u32		marker;		/* the newest update's */
	u32		merged;		/* updates in the part */
	u32		changed_px;	/* by its snapshot, 0: none built */
	u32		du_area;	/* the DU budget spent at the take */
	u32		gap_us;		/* the take's wait for a quiet gap */
	u32		tries;		/* a carried part's retries */
	int		temp;		/* clamped; valid if @temp_read */
	bool		temp_read;
	bool		repaired;	/* a RESET repair ran before it */
	bool		gave_up;	/* given up without a drive */
	u8		source;		/* enum rk3026_ebc_source */
	u8		outcome;	/* enum rk3026_ebc_outcome */
	u8		reason;		/* enum rk3026_ebc_reason, of @mode */
	u32		frames;		/* frames in its (last) waveform */
	u32		driven;		/* of those, driven */
	u32		scans;		/* waveforms started, repair included */
	u32		timeouts;
	u32		ms;		/* take to done */
	u32		scan_ms;	/* the last waveform */
	u32		blit_us;	/* the newest update's blit */
	u32		snap_us;	/* its snapshot, 0: none built */
};

/*
 * Counters since probe, under @image_lock. @skips, @dropped and @done count
 * parts; @updates less @coalesced is the parts made, @done plus those
 * pending, except where a RESET absorbed a pending set behind another RESET
 * (rk3026_ebc_pending_add()): that set's first update made a part that is
 * gone; and a carry taken alone at its retry is a part of its own.
 * @carry_failed counts the parts given up (enum rk3026_ebc_gave_up): carries
 * past their retries, and RESETs not driven.
 */
struct rk3026_ebc_stats {
	u64		updates;	/* rk3026_ebc_add_update() calls */
	u64		coalesced;	/* of those, merged into a pending set */
	u64		done;		/* parts taken and finished */
	u64		scans;		/* waveforms started, repairs included */
	u64		skips;		/* identical image, nothing driven */
	u64		dropped;	/* CRTC off, no waveform, rails or LUT */
	u64		timeouts;	/* chunk timeouts */
	u64		abandoned;	/* rails found down before a chunk */
	u64		carry_failed;	/* carries, RESETs given up */
};

/*
 * Refresh bookkeeping as plain data, for the pure helpers in rk3026_ebc_core.c.
 * None of it knows about the device; the driver embeds it and holds the lock.
 */

/*
 * Updates merged into one scan: the bounding box of their clips, the
 * strongest of their modes (rk3026_ebc_mode_rank()), the newest seq. Empty
 * while @merged is 0. Taken out of a struct rk3026_ebc_pending it is the part
 * one scan drives; a part whose @mode is EBC_MODE_RESET is a RESET alone.
 *
 * @auto_ok: every update in the set has @auto_mode, so the content may pick
 * the mode (rk3026_ebc_resolve_mode()); one that names its mode clears it,
 * and so does a carry. @mode and its rank stay the plain modes' either way.
 *
 * @cleanup_idle: the idle cleanup (EBC_SOURCE_IDLE) is in the set. Updates
 * that join it keep it, so the part is still the cleanup FULL: never
 * skipped (rk3026_ebc_should_skip()), and driven in its scan mode for the
 * reason EBC_REASON_CLEANUP_IDLE. A carry never has it
 * (rk3026_ebc_scan_finish()), and the cleanup is not queued while a carry
 * waits (rk3026_ebc_idle_tick()), so a part that has it and one update is
 * the cleanup alone.
 *
 * @first and @last: when its first and its newest update were queued; a
 * carry merged in keeps those of the set it joins. @has_mxcfb: an
 * MXCFB_SEND_UPDATE is in the set, so its take does not wait for a gap.
 */
struct rk3026_ebc_set {
	struct drm_rect	box;
	ktime_t		first;
	ktime_t		last;
	u32		mode;
	u32		seq;		/* the newest update's */
	u32		marker;		/* the newest update's, or 0 */
	u32		blit_us;	/* the newest update's */
	u32		merged;		/* updates in the set */
	int		temp;		/* the newest valid one */
	bool		temp_valid;
	/* an MXCFB_SEND_UPDATE in a full-class mode: never skipped */
	bool		has_mxcfb_full;
	bool		has_mxcfb;
	bool		auto_ok;
	bool		has_plane;	/* a plane update is in the set */
	bool		cleanup_idle;
	u8		source;		/* the newest update's */
};

/*
 * Everything queued and not yet taken, in seq order: @pre, then the RESET
 * barrier @reset, then @post. An update joins @post while a RESET is pending
 * and @pre otherwise, so the RESET's scan comes after every update queued
 * before it and before every one queued after it. The target image is split
 * the same way: a RESET sent behind @pre leaves @pre the image its updates
 * were sent into, and what is sent after the RESET is blitted on white
 * (rk3026_ebc_add_update()).
 *
 * @carry is what a scan took but could not drive (no rails, LUT or waveform,
 * or a failed repair). It is merged into the next @pre that is taken, and
 * without one the refresh thread takes it alone at its retry (@retry,
 * rk3026_ebc_retry_next()), a bounded number of times; it is not work
 * before that, so a failing rail cannot spin the thread. A RESET sent drops
 * it: that RESET whitens the glass it would draw. Present while its
 * @merged is 1 (a carry counts no updates); its @seq is that of the newest
 * update in it.
 *
 * @queued_seq is the seq of the newest update, @taken_seq that of the
 * newest taken one and @done_seq that of the newest finished one; all wrap
 * and compare with rk3026_ebc_seq_done(). Parts are taken in seq order and
 * finished one at a time; a part's @done_seq moves when it is driven,
 * skipped or dropped, and for a carry when it is given up, so it stays
 * behind while the carry waits for a retry. Once the part taken last has
 * finished and no retry is armed, @done_seq equals @queued_seq exactly when
 * nothing is pending.
 */
struct rk3026_ebc_pending {
	struct rk3026_ebc_set	pre;
	struct rk3026_ebc_set	reset;
	struct rk3026_ebc_set	post;
	struct rk3026_ebc_set	carry;
	struct rk3026_ebc_retry	retry;
	u32			queued_seq;
	u32			taken_seq;
	u32			done_seq;
};

/*
 * The CPU-side images, packed 4bpp like the scanned ones (@pitch bytes per
 * line, @height lines): what the glass shows, and the snapshot a scan drives.
 * @prev_unknown says @prev is not evidence: a waveform did not end, and no
 * RESET or FULL has run to its end since (rk3026_ebc_drive_knows_glass()).
 */
struct rk3026_ebc_shadows {
	u8		*prev;
	u8		*scan;
	u32		pitch;
	u32		height;
	bool		prev_unknown;
};

/*
 * MXCFB_WAIT_FOR_UPDATE_COMPLETE markers of the newest updates, and the seq
 * each was queued with. Marker 0 is never recorded, so an empty slot never
 * matches.
 */
#define EBC_MARKER_RING_LEN	32

struct rk3026_ebc_marker_ring {
	u32		marker[EBC_MARKER_RING_LEN];
	u32		seq[EBC_MARKER_RING_LEN];
	unsigned int	next;		/* the slot written next, unwrapped */
};

/*
 * Widens @r to whole bytes, an even x on both edges (two pixels a byte), and
 * clamps it to an image of @width x @height pixels. Returns false if nothing
 * of it is left.
 */
static inline bool rk3026_ebc_widen_clip(struct drm_rect *r,
					 unsigned int width,
					 unsigned int height)
{
	r->x1 = max(r->x1, 0) & ~1;
	r->x2 = min_t(int, round_up(r->x2, 2), (int)width);
	r->y1 = max(r->y1, 0);
	r->y2 = min_t(int, r->y2, (int)height);
	return r->x1 < r->x2 && r->y1 < r->y2;
}

/* The update queued with @seq has finished once @done_seq reached it. */
static inline bool rk3026_ebc_seq_done(u32 done_seq, u32 seq)
{
	return (s32)(done_seq - seq) >= 0;
}

/*
 * Nothing pending. A carry alone is not pending: the refresh thread takes
 * it at its retry (rk3026_ebc_retry_ready()) or with the next part.
 */
static inline bool rk3026_ebc_pending_empty(const struct rk3026_ebc_pending *p)
{
	return !p->pre.merged && !p->reset.merged;
}

/*
 * A part is pending or in flight, read without the lock. Parts are taken in
 * seq order and @done_seq moves only when one finishes, so from the take of
 * a part to its finish @done_seq is behind @queued_seq, and so it stays while
 * a carry waits for its retry (@retry.armed); otherwise, between parts, the
 * two differ exactly when a part is pending (!rk3026_ebc_pending_empty()).
 * An update not finished, for debugfs "busy"; the thread's wake is
 * rk3026_ebc_take_pending().
 */
static inline bool rk3026_ebc_work_pending(const struct rk3026_ebc_pending *p)
{
	return READ_ONCE(p->queued_seq) != READ_ONCE(p->done_seq);
}

/*
 * A part to take, read without the lock: an update sent and not taken.
 * Parts are taken in seq order, so @taken_seq differs from @queued_seq
 * exactly when one is pending (!rk3026_ebc_pending_empty()). A carry is not
 * one: it is taken at its retry (rk3026_ebc_retry_due()) or with the next
 * part, so a held @done_seq does not spin the thread.
 */
static inline bool rk3026_ebc_take_pending(const struct rk3026_ebc_pending *p)
{
	return READ_ONCE(p->queued_seq) != READ_ONCE(p->taken_seq);
}

/*
 * The carry's retry is due at @now (struct rk3026_ebc_retry): at its time,
 * or at once with the CRTC off, which drops it and ends its waits.
 */
static inline bool rk3026_ebc_retry_due(const struct rk3026_ebc_retry *r,
					ktime_t now, bool crtc_on)
{
	return READ_ONCE(r->armed) &&
	       (!crtc_on || !ktime_before(now, READ_ONCE(r->at)));
}

/*
 * The refresh thread's wait condition between parts, read without the
 * lock: a part to take (rk3026_ebc_take_pending()) or the carry's retry due
 * at @now (rk3026_ebc_retry_due(), with @crtc_on). A carry that waits for
 * its retry is not work before it; its time is a deadline of the wait
 * (rk3026_ebc_thread_deadline()). While the system suspends (@suspended)
 * nothing is work: the parts stay pending, and the carry keeps its retry,
 * until the resume.
 */
static inline bool rk3026_ebc_thread_woken(const struct rk3026_ebc_pending *p,
					   ktime_t now, bool crtc_on,
					   bool suspended)
{
	return !suspended &&
	       (rk3026_ebc_take_pending(p) ||
		rk3026_ebc_retry_due(&p->retry, now, crtc_on));
}

/*
 * The refresh thread's wait condition while the take's gap is open, read
 * without the lock. A part is pending all along, so rk3026_ebc_thread_woken()
 * would be true and the thread would spin: it wakes on an update newer than
 * @seen, the newest seq at its last look, or on the CRTC going off
 * (@crtc_on false), whose look ends the gap; else at the gap's end, the
 * wait's timeout.
 */
static inline bool rk3026_ebc_gap_woken(const struct rk3026_ebc_pending *p,
					u32 seen, bool crtc_on)
{
	return READ_ONCE(p->queued_seq) != seen || !crtc_on;
}

/*
 * The try of the carry the next take makes (struct rk3026_ebc_retry), for
 * its record: 0 when the carry is not armed for a retry, or not in the part
 * taken next (rk3026_ebc_pending_peek(): a RESET goes first, and the carry
 * alone only at its retry). Under @image_lock.
 */
static inline u32 rk3026_ebc_retry_try(const struct rk3026_ebc_pending *p)
{
	if (!p->carry.merged || !p->retry.armed)
		return 0;
	if (!p->pre.merged && (p->reset.merged || !p->retry.ready))
		return 0;
	return p->retry.tries + 1;
}

/* The update queued with @seq has finished. Read without the lock. */
static inline bool rk3026_ebc_seq_finished(const struct rk3026_ebc_pending *p,
					   u32 seq)
{
	return rk3026_ebc_seq_done(READ_ONCE(p->done_seq), seq);
}

/* The parts still to take: @pre, the RESET, @post. */
static inline unsigned int
rk3026_ebc_pending_parts(const struct rk3026_ebc_pending *p)
{
	return !!p->pre.merged + !!p->reset.merged + !!p->post.merged;
}

struct rk3026_ebc {
	struct drm_device	drm;
	struct drm_plane	plane;
	struct drm_crtc		crtc;
	struct drm_encoder	encoder;
	struct drm_connector	connector;

	struct device		*dev;
	void __iomem		*regs;
	struct clk		*hclk;
	struct clk		*dclk;
	struct clk		*aclk;
	/* The EBC's own AXI master port; "aclk" is the video NIU, not this. */
	struct clk		*mclk;
	/* Asserted as dclk's parent after set_rate — see ebc_hw_init(). */
	struct clk		*dclk_parent;
	/* SRST_EBC_AXI/AHB, optional: ebc_stop_scan() after a chunk timeout. */
	struct reset_control	*rst;
	struct regulator	*epd_supply;
	/*
	 * VCOM (the panel's common electrode) and V3P3 (its logic supply). Both
	 * are separate rails on the TPS65185 and both were unconsumed until
	 * 2026-08-03, so nothing ever enabled them: the vendor writes
	 * ENABLE = 0xAF, i.e. V3P3_EN set, and drives VCOM through its own
	 * VCOM_CTRL pin, which upstream models as the vcom regulator's ena_gpiod.
	 * Left unconsumed, that pin stays low -- and low does not merely disable
	 * the amplifier, it actively DISCHARGES VCOM (SLVSAQ8G pin 12).
	 */
	struct regulator	*vcom_supply;
	struct regulator	*v3p3_supply;
	struct regmap		*grf;
	struct thermal_zone_device *tzd;
	int			irq;

	struct display_timing	timing;
	u32			width;
	u32			height;
	u32			gdck_sta;
	u32			lgonl;
	unsigned int		rotation;

	/* The two 4bpp images the EBC scans: on-panel and target. */
	void			*prev_virt;
	dma_addr_t		prev_phys;
	void			*next_virt;
	dma_addr_t		next_phys;
	size_t			img_size;

	/*
	 * The CPU-side images, all cached, the pending updates and the
	 * refresh bookkeeping: what senders, the refresh thread and debugfs
	 * share, under @image_lock.
	 *
	 * @next_shadow is the target every sender blits into, white again when
	 * a RESET is sent (the RESET ends white everywhere). While @pre_split,
	 * @pre_shadow is the target as a RESET found it with @pending.pre still
	 * pending, and that part is drawn from it; otherwise it is a spare
	 * (rk3026_ebc_add_update()). @shadows.prev is
	 * what the glass shows and @shadows.scan the snapshot a scan drives
	 * (rk3026_ebc_build_snapshot()). The refresh thread alone writes those
	 * two, under @image_lock, and swaps them after a scan
	 * (rk3026_ebc_scan_finish()); being their only writer, it reads them
	 * without the lock.
	 *
	 * @plane_resync: an update from another source wrote @next_shadow (an
	 * mxcfb send, after which plane updates are blocked while its client
	 * runs, or a RESET's white fill), so outside a later damage box it no
	 * longer follows the plane's framebuffer: the next plane update redraws
	 * the whole panel (rk3026_ebc_plane_update()).
	 *
	 * @shadows.prev_unknown: @shadows.prev does not describe the glass.
	 * Set when a waveform starts, cleared ONLY when every frame has been
	 * driven, so a timeout, an error path or a path nobody has written yet
	 * all leave it set by construction. ⚠️ Do not turn this into a list of
	 * clears on the known abort paths: the one that matters is the path not
	 * yet written. Read at the next draw, which then clears the panel first.
	 * Written by the refresh thread alone: rk3026_ebc_drive() owns it during
	 * a waveform, and rk3026_ebc_scan_finish() records the part's outcome
	 * under @image_lock, never clearing what rk3026_ebc_drive() left set.
	 * One more writer, while that thread is parked: a hibernation restore
	 * sets it (rk3026_ebc_sleep_restore()), never clears it.
	 *
	 * @pending holds every update sent and not yet taken (a sender blits
	 * and adds its update in one section, so no snapshot sees pixels
	 * without their update), and the carry of a scan that could not
	 * drive its part. @markers maps the senders' markers to the seq of
	 * their updates for MXCFB_WAIT_FOR_UPDATE_COMPLETE. @pending.queued_seq
	 * and @pending.done_seq are also read without the lock, by the waits
	 * on them.
	 *
	 * @prev_virt and @next_virt, the write-combined images the controller
	 * scans, are written by the thread alone, outside @image_lock, and
	 * never read by the CPU.
	 *
	 * @image_lock is never held across a waveform, the sensor read or a
	 * write into @prev_virt or @next_virt, so a sender waits at most for
	 * one snapshot to be built; @rails_lock is never taken under it.
	 */
	struct mutex		image_lock;
	u8			*next_shadow;
	u8			*pre_shadow;
	bool			pre_split;
	bool			plane_resync;
	struct rk3026_ebc_shadows shadows;
	struct rk3026_ebc_pending pending;
	struct rk3026_ebc_marker_ring markers;

	/*
	 * The DU cleanup budget (rk3026_ebc_du_account()). @du_area counts the
	 * pixels plane parts changed in DU or A2 since the glass was last
	 * cleared; the refresh thread alone writes it, under @image_lock;
	 * PART parts leave it. Once it reaches @du_cleanup_area (spent), or
	 * would reach it with the part, an auto plane part, two-level or
	 * grey, that changes at least @du_cleanup_big_area pixels is driven
	 * FULL instead of DU, A2 or PART; once it reaches
	 * @du_cleanup_ceiling, any auto plane part is
	 * (rk3026_ebc_resolve_mode()). Probe sets the limit to one panel's
	 * area, a big part to half of it and the ceiling to four panels;
	 * debugfs may change each (a limit of 0: no cleanup; a ceiling of 0:
	 * none), and the thread reads them with READ_ONCE().
	 */
	u32			du_area;
	u32			du_cleanup_area;
	u32			du_cleanup_big_area;
	u32			du_cleanup_ceiling;

	/*
	 * The idle cleanup (rk3026_ebc_idle_tick()): with the budget spent and
	 * no plane update for @du_cleanup_idle_ms, the refresh thread queues a
	 * whole-panel FULL. A queued plane update arms it and stamps
	 * @idle_last_plane (jiffies), and so does the end of the scan of a
	 * part with a plane update in it, so the delay runs from the later of
	 * the two; the thread's first look after the delay disarms it, and
	 * queues the cleanup if it is due then, so one arming makes at most
	 * one cleanup. Both under @image_lock, written with WRITE_ONCE(): the
	 * thread also reads them without the lock, for the deadline of its
	 * wait alone, and decides under the lock. @du_cleanup_idle_ms is the
	 * delay (0: none); probe sets EBC_DU_CLEANUP_IDLE_MS, debugfs may
	 * change it, and the thread reads it with READ_ONCE().
	 */
	unsigned long		idle_last_plane;
	bool			idle_armed;
	u32			du_cleanup_idle_ms;

	/*
	 * A two-level auto plane part that changes at most @a2_max_area
	 * pixels, none of them over a grey of the glass, is driven in A2
	 * (rk3026_ebc_resolve_mode()). Probe sets 0, which is never: A2 stays
	 * off until its ghosting is measured on the panel. debugfs may change
	 * it, and the thread reads it with READ_ONCE().
	 */
	u32			a2_max_area;

	/*
	 * The take's quiet gap (rk3026_ebc_take_ready()): @gap is the refresh
	 * thread's alone; @take_done, when the last part finished
	 * (rk3026_ebc_clock()), is written under @image_lock by the thread.
	 * @take_quiet_us and @take_cap_ms: probe sets EBC_TAKE_QUIET_US and
	 * EBC_TAKE_CAP_MS, debugfs may change them, and the thread reads them
	 * with READ_ONCE(); rk3026_ebc_take_wait() clamps them.
	 */
	struct rk3026_ebc_gap	gap;
	ktime_t			take_done;
	u32			take_quiet_us;
	u32			take_cap_ms;

	/* Write-only registers the controller does not read back. */
	u32			dsp_start;
	u32			dsp_ctrl;
	u32			int_reg;

	struct completion	frame_done;

	/*
	 * Refresh thread. A send and a CRTC disable wake @update_wq; every
	 * finished part wakes @marker_wq, where the waits on
	 * @pending.done_seq look whether it moved over their update.
	 */
	struct task_struct	*refresh_thread;
	wait_queue_head_t	update_wq;
	wait_queue_head_t	marker_wq;

	/*
	 * Panel rails off when idle. @epd_supply and @vcom_supply are switched
	 * by the refresh thread alone: on before a waveform is driven, off at
	 * @rails_off_at (rails_off_ms after the last waveform ended, or after
	 * they came up), and off at CRTC disable.
	 * @rails_lock serialises the thread against ebc_crtc_atomic_disable().
	 *
	 * @epd_on and @vcom_on say which of the two this driver holds enabled,
	 * so every regulator_enable() is matched by exactly one
	 * regulator_disable(): a disable that fails leaves its flag set, and the
	 * next attempt (idle tick, CRTC disable, shutdown) retries only what is
	 * still on. @rails_on is "either is on", for the thread's lockless
	 * timeout. All three under @rails_lock.
	 */
	struct mutex		rails_lock;
	bool			rails_on;
	bool			epd_on;
	bool			vcom_on;
	/* @v3p3_supply enabled by ebc_crtc_atomic_enable(). */
	bool			v3p3_on;
	/* Rails-off deadline, in jiffies; only meaningful while @rails_on. */
	unsigned long		rails_off_at;
	/*
	 * The CRTC is enabled, so a part may drive the panel. Under
	 * @image_lock: the thread samples it in the section that takes a
	 * part, and a sender checks it in the section that adds its update.
	 */
	bool			crtc_on;
	/*
	 * System sleep (ebc_suspend()): while @suspended no part is taken and
	 * updates stay pending; @in_flight is a part taken and not yet
	 * finished, which the suspend waits for. Both under @image_lock; the
	 * refresh thread, and the CRTC enable's and disable's wait
	 * (rk3026_ebc_crtc_wait()), read them without it.
	 */
	bool			suspended;
	bool			in_flight;
	/*
	 * The clocks are enabled: by ebc_hw_init(), and again by a resume
	 * that got them (ebc_clks_on()). ebc_hw_fini() disables only these,
	 * so an unbind after a resume that found no clocks does not disable
	 * them twice. Probe, remove and the PM callbacks only.
	 */
	bool			clks_on;

	const struct firmware	*fw;
	struct ebc_waveform	*waveform;

	/*
	 * mxcfb shim state; unused when the shim is not built. @fbops is
	 * drm-managed, see rk3026_ebc_mxcfb_install().
	 */
	struct fb_ops		*fbops;
	const struct fb_ops	*orig_fbops;
	atomic_t		fb_users;
	/* Written under @image_lock; debugfs reads it without. */
	bool			mxcfb_active;

	/*
	 * The commit being applied: rk3026_ebc_commit_begin() clears
	 * @commit_queued, and its plane update sets it with @commit_seq, the
	 * seq of the update it queued (rk3026_ebc_plane_update(), under
	 * @image_lock). Written and read only in the commit tail: the panel
	 * has one CRTC, and each commit's planes are applied after the
	 * previous commit's are done
	 * (drm_atomic_helper_wait_for_dependencies()).
	 */
	u32			commit_seq;
	bool			commit_queued;

	/*
	 * debugfs: the last part, the last driven one, the counters. Under
	 * @image_lock.
	 */
	struct rk3026_ebc_last	last;
	struct rk3026_ebc_last	last_driven;
	struct rk3026_ebc_stats	stats;
	struct dentry		*debugfs;
};

static inline struct rk3026_ebc *drm_to_ebc(struct drm_device *drm)
{
	return container_of(drm, struct rk3026_ebc, drm);
}

/* rk3026_ebc.c */
u32 rk3026_ebc_blit_xrgb8888(struct rk3026_ebc *ebc, const void *src,
			     unsigned int src_pitch, const struct drm_rect *clip);
u32 rk3026_ebc_add_update(struct rk3026_ebc *ebc,
			  const struct rk3026_ebc_update *upd);
int rk3026_ebc_wait_marker(struct rk3026_ebc *ebc, u32 marker);
#if IS_ENABLED(CONFIG_KUNIT)
void rk3026_ebc_do_refresh(struct rk3026_ebc *ebc, struct rk3026_ebc_last *last);
int rk3026_ebc_drive(struct rk3026_ebc *ebc, const u8 *lut, u32 frames,
		     u32 mode, u32 lut_mode, struct rk3026_ebc_last *last);
int rk3026_ebc_wait_seq(struct rk3026_ebc *ebc, u32 seq,
			unsigned int timeout_ms, bool interruptible);
unsigned int rk3026_ebc_part_max_ms(struct rk3026_ebc *ebc);
unsigned int rk3026_ebc_wait_timeout_ms(struct rk3026_ebc *ebc);
bool rk3026_ebc_plane_box(const struct drm_plane_state *old_state,
			  const struct drm_plane_state *new_state,
			  unsigned int width, unsigned int height,
			  struct drm_rect *box);
void rk3026_ebc_commit_begin(struct rk3026_ebc *ebc);
bool rk3026_ebc_plane_update(struct rk3026_ebc *ebc, const void *src,
			     unsigned int src_pitch, const struct drm_rect *box);
bool rk3026_ebc_idle_tick(struct rk3026_ebc *ebc, unsigned long now);
int rk3026_ebc_raise_rails(struct rk3026_ebc *ebc);
ktime_t rk3026_ebc_clock(struct rk3026_ebc *ebc);
bool rk3026_ebc_take_ready(struct rk3026_ebc *ebc, ktime_t now);
u32 rk3026_ebc_crtc_off(struct rk3026_ebc *ebc);
int rk3026_ebc_read_temp(struct rk3026_ebc *ebc);
void rk3026_ebc_thread_pass(struct rk3026_ebc *ebc, unsigned long now,
			    struct rk3026_ebc_pass *s);
u32 rk3026_ebc_setup_us(struct rk3026_ebc *ebc,
			const struct rk3026_ebc_last *last);
void rk3026_ebc_sleep_begin(struct rk3026_ebc *ebc);
int rk3026_ebc_sleep_wait(struct rk3026_ebc *ebc, unsigned int timeout_ms);
void rk3026_ebc_sleep_end(struct rk3026_ebc *ebc);
void rk3026_ebc_sleep_restore(struct rk3026_ebc *ebc);
int rk3026_ebc_crtc_wait(struct rk3026_ebc *ebc, u32 seq,
			 unsigned int timeout_ms);
#endif

/* rk3026_ebc_blit.c */
void rk3026_ebc_blit_rows(u8 *dst, const void *src, unsigned int src_pitch,
			  int x1, int x2, int y1, int y2, unsigned int width);
void rk3026_ebc_blit_convert(u8 *dst, const void *src, unsigned int src_pitch,
			     int x1, int x2, int y1, int y2,
			     unsigned int width);
const char *rk3026_ebc_blit_impl(void);

/* rk3026_ebc_core.c */
u32 rk3026_ebc_dsp_ctrl_mode(u32 mode);
u32 rk3026_ebc_us_between(ktime_t start, ktime_t end);
bool rk3026_ebc_pending_add(struct rk3026_ebc_pending *p,
			    const struct rk3026_ebc_update *upd, u32 *seq);
bool rk3026_ebc_pending_peek(const struct rk3026_ebc_pending *p,
			     struct rk3026_ebc_set *part);
bool rk3026_ebc_pending_take(struct rk3026_ebc_pending *p,
			     struct rk3026_ebc_set *part);
bool rk3026_ebc_build_snapshot(const struct rk3026_ebc_shadows *sh,
			       const u8 *next, const struct drm_rect *box,
			       u32 *mode, bool *two_level, bool *over_grey,
			       u32 *changed_px);
bool rk3026_ebc_target_differs(const struct rk3026_ebc_shadows *sh,
			       const u8 *next, const struct drm_rect *box);
bool rk3026_ebc_should_skip(const struct rk3026_ebc_set *part,
			    bool prev_unknown, bool identical);
u32 rk3026_ebc_scan_mode(const struct rk3026_ebc_set *part, bool repair_due);
bool rk3026_ebc_drive_knows_glass(u32 mode, u32 lut_mode);
bool rk3026_ebc_du_spent(const struct rk3026_ebc_du_budget *budget);
bool rk3026_ebc_du_at_ceiling(const struct rk3026_ebc_du_budget *budget);
bool rk3026_ebc_du_cleanup(const struct rk3026_ebc_du_budget *budget,
			   u32 changed_px, u32 added_px,
			   enum rk3026_ebc_reason *reason);
u32 rk3026_ebc_resolve_mode(const struct rk3026_ebc_set *part, u32 scan_mode,
			    bool two_level, u32 changed_px,
			    enum rk3026_ebc_repair_due repair,
			    const struct rk3026_ebc_du_budget *budget,
			    const struct rk3026_ebc_a2 *a2,
			    enum rk3026_ebc_reason *reason);
bool rk3026_ebc_idle_cleanup_due(const struct rk3026_ebc_idle *idle,
				 unsigned long now, bool *expired);
bool rk3026_ebc_thread_deadline(const struct rk3026_ebc_pass *s,
				unsigned long retry_at, unsigned long *deadline);
enum rk3026_ebc_step rk3026_ebc_thread_step(const struct rk3026_ebc_pass *s,
					    long *timeout);
bool rk3026_ebc_take_wait(const struct rk3026_ebc_take *t, ktime_t now,
			  ktime_t *deadline);
bool rk3026_ebc_gap_timeout(ktime_t at, ktime_t now, ktime_t *left);
u32 rk3026_ebc_du_account(u32 du_area, const struct rk3026_ebc_set *part,
			  u32 mode, u32 changed_px,
			  enum rk3026_ebc_outcome outcome,
			  enum rk3026_ebc_repair repair);
bool rk3026_ebc_retry_next(ktime_t first, ktime_t now, ktime_t *at);
void rk3026_ebc_retry_ready(struct rk3026_ebc_pending *p, ktime_t now,
			    bool crtc_on);
bool rk3026_ebc_scan_finish(struct rk3026_ebc_shadows *sh,
			    struct rk3026_ebc_pending *p,
			    const struct rk3026_ebc_set *part,
			    enum rk3026_ebc_outcome outcome,
			    enum rk3026_ebc_repair repair, ktime_t now,
			    enum rk3026_ebc_gave_up *gave_up);
void rk3026_ebc_marker_seq_record(struct rk3026_ebc_marker_ring *ring,
				  u32 marker, u32 seq);
bool rk3026_ebc_marker_seq_lookup(const struct rk3026_ebc_marker_ring *ring,
				  u32 marker, u32 *seq);
#if IS_ENABLED(CONFIG_KUNIT)
unsigned int rk3026_ebc_mode_rank(u32 mode);
#endif

/* rk3026_ebc_waveform.c */
struct ebc_waveform *rk3026_ebc_waveform_parse(struct device *dev,
					       const u8 *buf, size_t len);
const u8 *rk3026_ebc_waveform_lut(struct ebc_waveform *wf, u32 mode, int temp,
				  u32 *frames);
#if IS_ENABLED(CONFIG_KUNIT)
int rk3026_ebc_rkf_decode(const s16 *left, const s16 *right, int root,
			  u8 *dst, size_t dst_len, size_t n, const u8 *src,
			  size_t len);
#endif

/* rk3026_ebc_debugfs.c */
#if IS_ENABLED(CONFIG_DEBUG_FS)
void rk3026_ebc_debugfs_init(struct rk3026_ebc *ebc);
void rk3026_ebc_debugfs_fini(struct rk3026_ebc *ebc);
#else
static inline void rk3026_ebc_debugfs_init(struct rk3026_ebc *ebc) { }
static inline void rk3026_ebc_debugfs_fini(struct rk3026_ebc *ebc) { }
#endif

/* rk3026_ebc_mxcfb.c */
#if IS_ENABLED(CONFIG_DRM_RK3026_EBC_MXCFB)
void rk3026_ebc_mxcfb_install(struct rk3026_ebc *ebc, struct fb_info *info);
#if IS_ENABLED(CONFIG_KUNIT)
struct mxcfb_update_data;
u32 rk3026_ebc_mxcfb_to_ebc_mode(u32 waveform_mode, u32 update_mode,
				 u32 flags, bool *auto_mode);
int rk3026_ebc_mxcfb_send(struct rk3026_ebc *ebc, const void *src,
			  unsigned int pitch,
			  const struct mxcfb_update_data *req);
#endif
#else
static inline void rk3026_ebc_mxcfb_install(struct rk3026_ebc *ebc,
					    struct fb_info *info) { }
#endif

#endif /* _RK3026_EBC_H */
