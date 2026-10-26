// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK3026 EBC (E-Ink Broadcaster) DRM driver.
 *
 * Targets the ONYX BOOX C67ML and its rebrands: a 6.0" 1024x758 E Ink panel
 * (ED060XC3T1) driven by the RK3026 EBC block. Register offsets, timing
 * packing and the frame sequence were reversed from the vendor binary-only
 * driver; see rk3026_ebc.h for the map.
 *
 * The EBC scans a pair of packed 4bpp images - what is on the panel and what
 * is being driven towards - through a waveform LUT indexed by (old grey, new
 * grey). A refresh is tens to hundreds of frames at 85 Hz, so all of it runs
 * on a dedicated kthread: atomic commits and the frame-end interrupt only
 * hand work to it.
 *
 * Copyright (C) 2026
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/thermal.h>
#include <linux/unaligned.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>

#include <video/of_display_timing.h>

#include <kunit/static_stub.h>
#include <kunit/visibility.h>

#include "rk3026_ebc.h"

#define CREATE_TRACE_POINTS
#include "rk3026_ebc_trace.h"

#define DRIVER_NAME		"rk3026-ebc"
#define EBC_WAVEFORM_FW		"rockchip/rk3026-c67ml.rkf"
/*
 * Per-CHUNK frame budget, and a chunk is at most EBC_LUT_MAX_FRAMES (64) frames.
 *
 * Measured on hardware 2026-08-03 with dclk_ebc at 66 MHz: 54 frames take
 * 790-800 ms, i.e. ~14.8 ms/frame, so a full 64-frame chunk is ~950 ms. 5 s is
 * a little over 5x that.
 *
 * This replaces a deliberately absurd 60000 that was set while the dot clock was
 * still stuck at 12 MHz, so it could not read as a considered value.
 *
 * ⚠️ Do NOT tighten this toward the measured time. The timeout path abandons
 * the waveform partway (it stops the controller, see ebc_stop_scan(), and drops
 * the rails), and a panel left mid-transition accumulates real ghosting and
 * darkening that persists into Android -- that is exactly what a 3000 ms
 * budget did against a 3.1 s waveform on 2026-08-02. A budget that is merely
 * tight is not a benign mis-tuning on this hardware, it is cumulative damage.
 * The timeout exists for a controller that has stopped answering, nothing
 * else.
 */
#define EBC_FRAME_TIMEOUT_MARGIN	4
#define EBC_FRAME_TIMEOUT_MIN_MS	5000
/*
 * A part's work besides its waveforms: the sensor read (~0.4 s at worst),
 * the rails coming up (~0.2 s), the snapshot copy.
 */
#define EBC_PART_SETUP_MS	1000
/*
 * The parts a wait on one update can sit behind, its own included: the one
 * in flight, then what was sent before a pending RESET, the RESET, and what
 * was sent after it (rk3026_ebc_pending_take()). A later send adds no part
 * ahead of these: a RESET sent behind a pending one absorbs the set between
 * them.
 */
#define EBC_WAIT_PARTS		4

/*
 * The panel's high-voltage rails and VCOM are on only around refreshes: the
 * refresh thread switches them on before it drives a waveform and off this
 * long after the last waveform ended (or after they came up, if none was
 * driven). A short delay keeps a burst of updates (typing, a scrolling
 * console) from cycling the PMIC per update. Only a driven waveform moves
 * the deadline: an update that is skipped (identical image) or has no LUT
 * does not, so a stream of no-op commits cannot keep the rails up. Stock
 * powers off right after each update; leaving the rails and VCOM up between
 * updates holds a DC bias on the panel.
 */
static unsigned int rails_off_ms = 200;
module_param(rails_off_ms, uint, 0644);
MODULE_PARM_DESC(rails_off_ms, "switch the panel rails off this long after the last refresh, in ms (default 200)");

/* Rails-off deadline: rails_off_ms from now. Under @rails_lock or on the thread. */
static void ebc_rails_rearm(struct rk3026_ebc *ebc)
{
	WRITE_ONCE(ebc->rails_off_at,
		   jiffies + msecs_to_jiffies(READ_ONCE(rails_off_ms)));
}

/*
 * Refresh mode used when nothing annotates the update: FULL.
 *
 * Such an update is sent with @auto_mode, and the content picks its waveform
 * (rk3026_ebc_resolve_mode()): when every pixel it changes becomes black or
 * white, the part is driven in DU, which drives those pixels fully and does
 * not flash (in A2 when it is small and a2_max_area allows it); with a grey
 * among them, in PART, which moves only the changed pixels, to all 16
 * levels. FULL is what a part keeps when the content does
 * not decide: behind a repair, merged with a send that names its mode,
 * carried, or as a cleanup of the DU budget (a big part once it is spent
 * or when it would spend it, any part once it reaches its ceiling). The
 * idle cleanup the refresh thread queues on a spent budget is a FULL of
 * its own (rk3026_ebc_idle_tick()).
 *
 * An early version defaulted to PART and rendered washed grey where
 * force_mode=1 rendered solid black (docs/PENDING.md P18), and plane parts
 * were kept off PART since. A later camera run on this panel did not
 * reproduce that: PART drew a grey ramp from white as FULL drew it, solid
 * black included, so a grey plane part goes PART now.
 */
#define EBC_MODE_DEFAULT	EBC_MODE_FULL

/*
 * The idle delay before the cleanup FULL of a spent DU budget
 * (rk3026_ebc_idle_tick()): long enough that a pause between keystrokes or
 * pointer moves does not flash the panel, short enough that the glass is
 * clean when the user looks at it. debugfs du_cleanup_idle_ms changes it.
 */
#define EBC_DU_CLEANUP_IDLE_MS	2000

static inline void ebc_write(struct rk3026_ebc *ebc, u32 reg, u32 val)
{
	writel_relaxed(val, ebc->regs + reg);
}

static inline u32 ebc_read(struct rk3026_ebc *ebc, u32 reg)
{
	return readl_relaxed(ebc->regs + reg);
}

static void ebc_config_done(struct rk3026_ebc *ebc)
{
	ebc_write(ebc, EBC_CONFIG_DONE, EBC_CONFIG_DONE_VAL);
}

/*
 * Source-driver clock, from the device tree's panel-timing, and the dot clock
 * derived from it.
 *
 * These are board and panel properties, so they belong in the DT and not in a
 * header. of_get_display_timing() already parses `clock-frequency` into
 * timing.pixelclock; the driver simply ignored it and used a constant, which is
 * how the two came to disagree.
 *
 * dclk is four times the source clock: the 8-bit source bus carries four 2-bit
 * pixels per clock, which is the same x4 the vendor applies before its own
 * clk_set_rate.
 */
static unsigned long ebc_pixel_rate(struct rk3026_ebc *ebc)
{
	unsigned long hz = ebc->timing.pixelclock.typ;

	return hz ? hz : EBC_PIXEL_RATE_FALLBACK;
}

static unsigned long ebc_dclk_rate(struct rk3026_ebc *ebc)
{
	return ebc_pixel_rate(ebc) * 4;
}

/**
 * ebc_chunk_timeout_ms - frame budget for one chunk, DERIVED from the timings
 * @ebc: controller
 * @frames: frames in this chunk
 *
 * A fixed budget is coupled to the dot clock, and that coupling broke silently:
 * EBC_FRAME_TIMEOUT_MS was 5000, sized when 64 frames took ~950 ms at dclk
 * 66 MHz. Lowering the clock to 12 MHz made a 64-frame chunk take 5050 ms, so
 * the FIRST scan of every boot -- the RESET that clears the panel -- timed out
 * 40 ms over, broke out of the frame loop and drove 0 of 101 frames. The panel
 * came up holding whatever was on it, and nothing in the log said "too slow".
 *
 * So compute it. One frame is htotal source clocks x vtotal lines, at the source
 * clock; the x4 between dclk and the source clock cancels out because both come
 * from the same DT property.
 *
 * The margin is deliberately large. This timeout exists only so a wedged
 * controller cannot hang the thread forever -- it is not a tuning knob, and
 * expiring it is destructive: the loop `break`s mid-waveform, which leaves the
 * panel part-driven and accumulates ghosting that survives into the next boot.
 * Erring generous costs a few seconds once; erring tight corrupts the display.
 */
static unsigned int ebc_chunk_timeout_ms(struct rk3026_ebc *ebc, u32 frames)
{
	const struct display_timing *t = &ebc->timing;
	u32 hclks = t->hsync_len.typ + ebc->width / 4 +
		    t->hback_porch.typ + t->hfront_porch.typ;
	u32 vlines = t->vsync_len.typ + ebc->height +
		     t->vback_porch.typ + t->vfront_porch.typ;
	u64 us = div_u64((u64)hclks * vlines * USEC_PER_SEC,
			 ebc_pixel_rate(ebc));
	u64 ms = div_u64(us * frames * EBC_FRAME_TIMEOUT_MARGIN, USEC_PER_MSEC);

	return max_t(u64, ms, EBC_FRAME_TIMEOUT_MIN_MS);
}

/*
 * The longest one part takes while the refresh thread works: the setup
 * (EBC_PART_SETUP_MS), then at most two waveforms, a repair RESET and the
 * part's own, of at most EBC_WAVEFORM_MAX_FRAMES frames each. A chunk ends
 * within its budget (ebc_chunk_timeout_ms()) or ends the part, so every
 * chunk counts at its budget. Derived from the timings like the budget: at
 * a lower dot clock a part takes longer.
 */
static unsigned int ebc_part_max_ms(struct rk3026_ebc *ebc)
{
	u32 chunks = 2 * DIV_ROUND_UP(EBC_WAVEFORM_MAX_FRAMES,
				      EBC_LUT_MAX_FRAMES);

	return EBC_PART_SETUP_MS +
	       chunks * ebc_chunk_timeout_ms(ebc, EBC_LUT_MAX_FRAMES);
}

/*
 * Longest a wait on one update blocks (rk3026_ebc_wait_seq():
 * MXCFB_WAIT_FOR_UPDATE_COMPLETE, CRTC enable and disable): EBC_WAIT_PARTS
 * parts at their longest, 164 s with every chunk budget at its 5 s floor.
 * Past this the thread is stuck outside the EBC (an I2C transfer, say), and
 * the waiter goes on without it.
 */
VISIBLE_IF_KUNIT unsigned int rk3026_ebc_wait_timeout_ms(struct rk3026_ebc *ebc)
{
	return EBC_WAIT_PARTS * ebc_part_max_ms(ebc);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_wait_timeout_ms);

/*
 * DIAGNOSTIC, REVERT ME. The scan completes -- frame-end interrupts arrive and
 * no chunk times out -- while the glass does not move, so every stage between
 * the source image and the panel is reported as a positive marker rather than
 * inferred from the absence of an error. Reports the next N scans; defaults to
 * the first few so the fbcon updates at boot are covered without a shell, and
 * re-arms with
 *
 *	echo 4 > /sys/module/rk3026_ebc/parameters/debug_scan
 */
static unsigned int debug_scan = 6;
module_param(debug_scan, uint, 0644);
MODULE_PARM_DESC(debug_scan, "report every stage of the next N waveform scans (0 = off)");

static atomic_t dbg_irqs = ATOMIC_INIT(0);
static u32 dbg_last_status;

/*
 * DIAGNOSTIC, REVERT ME. Runtime knobs so one card can answer several questions
 * without a rebuild, because judging a weak panel response by eye is expensive
 * and ambiguous.
 *
 * force_mode: drive every update in this EBC_MODE_* instead of the one the
 * update carries. Modes 11 and 12 are the DIRECT modes -- they assert neither
 * DISPLAY_MODE nor UPDATE_MODE, so the controller drives from the image without
 * the waveform LUT at all. A hard black/white there proves the pad, clock, rail
 * and image-fetch chain and isolates the fault to the LUT; mush there exonerates
 * the LUT and points at the data path.
 *
 * swap_mst: put NEXT in WIN_MST0 -- the order this driver shipped between
 * 2026-08-03 and 2026-08-05, which drives one-directionally: the panel only ever
 * bleaches toward white and never darkens. It exists to reproduce that failure,
 * not as an equal alternative. The default is now the vendor's order, prev in
 * MST0 (docs/re/EBC-REGISTERS.md §5).
 *
 * ⚠️ Takes effect on the SECOND scan, not the first: WIN_MST0/1 are shadowed.
 * See ebc_set_image_addr().
 */
static int force_mode = -1;
module_param(force_mode, int, 0644);
MODULE_PARM_DESC(force_mode, "override the refresh mode (0-12; -1 = use the update's own)");

static bool swap_mst;
module_param(swap_mst, bool, 0644);
MODULE_PARM_DESC(swap_mst, "put next in WIN_MST0 (the transposed order that only drives toward white)");

/*
 * Knobs for the ONE open panel defect (docs/PENDING.md P15): a uniform
 * full-screen black renders as strong black crossed by regular, slightly
 * slanted white vertical stripes.
 *
 * ⚠️ DO NOT TOUCH EITHER KNOB YET. The axis the artifact lives on is NOT
 * established, and an earlier revision of this comment asserted that it was --
 * claiming the stripes must vary along the source-driver axis. That was
 * inference, not measurement, and the framebuffer is 1024x758 on a physically
 * portrait device, so which fb axis maps to which physical one is exactly the
 * thing in question. Retracted rather than reworded.
 *
 * What IS measured on hardware (2026-08-03/04), all with writes confirmed to
 * have landed via the `differ from prev` count:
 *
 *   - the stripes appear for EVERY pattern -- black, white, hsplit, vsplit,
 *     geometry and ybands alike -- so they are largely content-INDEPENDENT
 *   - ⚠️ "`fbpoke ybands` produced NO bands, so content that varies line-to-line
 *     is not reaching the glass" is RETRACTED (2026-08-05). Re-run on the card
 *     that carries the AXI-clock fix, from a forced black baseline, ybands
 *     produced clear bands. The source address DOES advance per line, and the
 *     stride hypothesis is dead. The original reading was taken on a card whose
 *     aclk was still 24 MHz -- check `scan: aclk` before trusting a panel result
 *   - `fbpoke xbands` and `ramp` show structure only in a strip near the top
 *     rather than across the panel. Both were drawn while the MST order was
 *     still transposed, so every greyscale reading from them is suspect; re-run
 *     now that direction is correct before theorising
 *
 * ⚠️ And sdck's suspicious value is NOT obviously wrong: Android drives this
 * panel with EPD_CTRL bit-identical to ours (0x5110ff03, same 0xff = 255), so
 * 255 is evidently correct and something else differs.
 *
 * ⚠️ Uniform images cannot reveal a mapping fault -- uniform in, uniform out.
 * Every apparent success before these probes (black, white, the RESET refresh)
 * was uniform, which is the same blind spot that hid the transposed MST order
 * for a whole session.
 *
 * sdck: source-driver clocks per line. Computed as width/4, but the field is
 * 8 bits and 1024/4 = 256 does not fit -- the driver's `width < 0x400` test
 * silently yields 0xff instead, which is either exactly right (if the field
 * encodes count-1) or one short (if it does not). Nothing establishes which.
 *
 * win_vir: the window's virtual width. ✅ **The unit is settled: PIXELS.** Android
 * drives this panel with WIN_VIR = 0x00000400 live (docs/VENDOR-BLOBS.md §6,
 * tier 0) = 1024 = width exactly, high half zero. So the knob is now only for
 * deliberately trying a *wrong* stride, not for resolving an ambiguity -- and
 * WIN_VIR is ELIMINATED as the cause of `ybands` showing no bands, because our
 * low half already matched.
 */
static int sdck = -1;
module_param(sdck, int, 0644);
MODULE_PARM_DESC(sdck, "override EPD_CTRL SDCK (-1 = width/4 saturated to 0xff)");

static uint win_vir;
module_param(win_vir, uint, 0644);
MODULE_PARM_DESC(win_vir, "override WIN_VIR's low half in pixels (0 = panel width)");

/*
 * DIAGNOSTIC, REVERT ME. Provoke the abort that docs/PENDING.md P62 repairs.
 *
 * P62's recovery is unreachable by every other means: the frame budget is
 * DERIVED from the panel timings so there is no way to make it too small, and
 * the interruptible wait in rk3026_ebc_wait_marker() aborts a caller's wait
 * rather than the drive. Without this the repair path can only be reasoned
 * about, and a repair nobody has seen run is a repair nobody has tested.
 *
 * Cuts the budget of the next N chunks to 1 ms, so wait_for_completion_timeout()
 * genuinely expires: same code path, same controller stop and rails-off, same
 * prev_unknown. Nothing is faked.
 *
 * 🔴 An abort leaves the panel MID-WAVEFORM, which is the cumulative ghosting
 * this driver otherwise works hard to avoid. Set it once, to a small number, to
 * take a verdict -- never leave it armed.
 *
 * ⚠️ Reading the result: without a reset in the device tree the controller is
 * only un-armed (ebc_stop_scan()), and the aborted scan's frame-end interrupt
 * may still arrive while the NEXT waveform is being set up, completing that
 * waveform's first chunk early. A chunk that reports "ended in 0 ms" is that
 * stale interrupt, not a scan. Judge the recovery by its chunk durations, not
 * by the word "ended".
 */
static unsigned int debug_timeout;
module_param(debug_timeout, uint, 0644);
MODULE_PARM_DESC(debug_timeout, "force the next N chunks to time out (0 = off); provokes P62's abort");

/*
 * DIAGNOSTIC: the snapshot a scan drives against the image the panel is
 * believed to show, from the cached shadows (debug_scan only). The skip
 * decision does not depend on it: rk3026_ebc_do_refresh() makes that one on
 * every refresh, from what rk3026_ebc_build_snapshot() returns.
 */
static void ebc_report_image(struct rk3026_ebc *ebc)
{
	const u8 *p = ebc->shadows.prev, *n = ebc->shadows.scan;
	size_t i, diff = 0, black = 0, white = 0;
	u32 hn = 2166136261u, hp = 2166136261u;

	for (i = 0; i < ebc->img_size; i++) {
		if (p[i] != n[i])
			diff++;
		if (n[i] == 0x00)
			black++;
		else if (n[i] == 0xff)
			white++;
		/*
		 * Position-sensitive fingerprints, because the counts are NOT.
		 * A half-black image has the same totals whether the split is
		 * horizontal or vertical, so "194048 x00 194048 xff" matched both
		 * fbpoke hsplit and vsplit -- and a dropped update therefore read
		 * as a plausible new one on 2026-08-03. FNV-1a; the value means
		 * nothing on its own, only whether it CHANGED.
		 */
		hn = (hn ^ n[i]) * 16777619u;
		hp = (hp ^ p[i]) * 16777619u;
	}

	dev_info(ebc->dev,
		 "scan: image %zu bytes, %zu differ from prev, scan %zu x00 %zu xff, fp scan %08x prev %08x\n",
		 ebc->img_size, diff, black, white, hn, hp);
}

/*
 * What the clocks and the EPD rail actually are at the moment of the scan.
 *
 * The GRF and CRU reads are here because a good boot and a bad boot have so
 * far produced dmesg that is identical line for line -- same registers, same
 * LUT, same frame counts, same 4260 ms -- while the panel images on one and
 * not the other. Whatever differs is therefore something the driver sets up
 * but never reads back, and pad mux, the EBC/LCDC pad select and the dclk mux
 * are exactly that: written once at probe, believed forever after.
 */
static void ebc_report_env(struct rk3026_ebc *ebc)
{
	u32 iomux2b = 0, iomux2c = 0, iomux2d = 0, ebc_sel = 0;

	dev_info(ebc->dev, "scan: aclk %lu dclk %lu hclk %lu epd_supply %d\n",
		 clk_get_rate(ebc->aclk), clk_get_rate(ebc->dclk),
		 clk_get_rate(ebc->hclk),
		 ebc->epd_supply ? regulator_is_enabled(ebc->epd_supply) : -1);

	if (ebc->grf) {
		regmap_read(ebc->grf, GRF_IOMUX_GPIO2B, &iomux2b);
		regmap_read(ebc->grf, GRF_IOMUX_GPIO2C, &iomux2c);
		regmap_read(ebc->grf, GRF_IOMUX_GPIO2D, &iomux2d);
		regmap_read(ebc->grf, GRF_EBC_SEL_REG, &ebc_sel);
	}
	dev_info(ebc->dev,
		 "scan: grf iomux2b %08x 2c %08x 2d %08x ebc_sel %08x (want aa/ff28/0a/0x301)\n",
		 iomux2b, iomux2c, iomux2d, ebc_sel);
}

/*
 * Registers the controller reads back, after the group has been latched by
 * CONFIG_DONE. DSP_START, DSP_CTRL and EBC_INT are write-only, so the values
 * printed for those are the driver's own shadow state, not the hardware's.
 */
static void ebc_report_regs(struct rk3026_ebc *ebc, const char *when)
{
	dev_info(ebc->dev,
		 "scan: %s win_ctrl %08x mst0 %08x mst1 %08x vir %08x act %08x dsp %08x dsp_st %08x\n",
		 when, ebc_read(ebc, EBC_WIN_CTRL), ebc_read(ebc, EBC_WIN_MST0),
		 ebc_read(ebc, EBC_WIN_MST1), ebc_read(ebc, EBC_WIN_VIR),
		 ebc_read(ebc, EBC_WIN_ACT), ebc_read(ebc, EBC_WIN_DSP),
		 ebc_read(ebc, EBC_WIN_DSP_ST));
	dev_info(ebc->dev,
		 "scan: %s epd_ctrl %08x act_info %08x ht %08x %08x vt %08x %08x\n",
		 when, ebc_read(ebc, EBC_EPD_CTRL),
		 ebc_read(ebc, EBC_DSP_ACT_INFO),
		 ebc_read(ebc, EBC_DSP_HTIMING0), ebc_read(ebc, EBC_DSP_HTIMING1),
		 ebc_read(ebc, EBC_DSP_VTIMING0), ebc_read(ebc, EBC_DSP_VTIMING1));
	dev_info(ebc->dev,
		 "scan: %s int(hw) %08x; shadow dsp_start %08x dsp_ctrl %08x int %08x\n",
		 when, ebc_read(ebc, EBC_INT), ebc->dsp_start, ebc->dsp_ctrl,
		 ebc->int_reg);
}

/*
 * Read the LUT SRAM window back. A scan against a zero LUT drives no pigment
 * and still ends normally, which is indistinguishable from success by every
 * signal we have today.
 *
 * A readback that is uniformly zero is ambiguous: either the writes did not
 * land, or the window is write-only. Compare against @nonzero_src -- if the
 * source LUT itself is mostly zeros, the waveform selection is the suspect,
 * not the SRAM.
 */
static void ebc_report_lut(struct rk3026_ebc *ebc, const u8 *src, u32 words,
			   u32 chunk)
{
	u32 i, want, got, bad = 0, nonzero_hw = 0, nonzero_src = 0;
	u32 first = 0, first_want = 0, first_got = 0;

	for (i = 0; i < words; i++) {
		want = get_unaligned_le32(src + i * 4);
		got = ebc_read(ebc, EBC_LUT_BASE + i * 4);
		if (want)
			nonzero_src++;
		if (got)
			nonzero_hw++;
		if (got != want && !bad++) {
			first = i;
			first_want = want;
			first_got = got;
		}
	}

	dev_info(ebc->dev,
		 "scan: chunk %u lut %u words, nonzero src %u hw %u, %u mismatched\n",
		 chunk, words, nonzero_src, nonzero_hw, bad);
	if (bad)
		dev_info(ebc->dev, "scan: chunk %u first mismatch word %u: wrote %08x read %08x\n",
			 chunk, first, first_want, first_got);
}

/*
 * H/V timing and active window. The H registers count source-driver clocks,
 * four pixels each; the V registers count lines. E Ink spec names in comments.
 */
static void ebc_set_timing(struct rk3026_ebc *ebc)
{
	const struct display_timing *t = &ebc->timing;
	u32 lsl = t->hsync_len.typ;
	u32 lbl = t->hback_porch.typ;
	u32 lel = t->hfront_porch.typ;
	u32 fsl = t->vsync_len.typ;
	u32 fbl = t->vback_porch.typ;
	u32 fel = t->vfront_porch.typ;
	u32 line = ebc->width / 4;

	ebc_write(ebc, EBC_DSP_HTIMING0,
		  lsl | ((lsl + line + lbl + lel) << 16));
	ebc_write(ebc, EBC_DSP_HTIMING1,
		  (lsl + lbl) | ((lsl + line + lbl) << 16));
	ebc_write(ebc, EBC_DSP_VTIMING0,
		  fsl | ((fsl + ebc->height + fel + fbl) << 16));
	ebc_write(ebc, EBC_DSP_VTIMING1,
		  (fsl + fbl) | ((fsl + fbl + ebc->height) << 16));
	ebc_write(ebc, EBC_DSP_ACT_INFO,
		  ebc->width | (ebc->height << 16));
	ebc_config_done(ebc);
}

/* Full-panel window, 4bpp, origin at the first active pixel. */
static void ebc_set_window(struct rk3026_ebc *ebc)
{
	const struct display_timing *t = &ebc->timing;
	u32 size = ebc->width | (ebc->height << 16);

	ebc_write(ebc, EBC_WIN_CTRL, WIN_CTRL_FIXED);
	/*
	 * WIN_VIR is the window's virtual width, in PIXELS, and the high half
	 * stays ZERO -- Android drives this panel with 0x00000400 live
	 * (docs/VENDOR-BLOBS.md §6, tier 0), i.e. 1024 with nothing above it,
	 * while every other register in the block matches us bit-for-bit.
	 *
	 * We used to write `width | height << 16` here. That made WIN_VIR the
	 * ONLY divergence from a known-good configuration, so it is removed:
	 * matching the vendor costs nothing and the alternative is carrying an
	 * unexplained delta into every panel experiment. It also settles the
	 * unit question the P15 notes raised -- pixels, not bytes and not
	 * 4-pixel groups, since 0x400 == width exactly.
	 */
	ebc_write(ebc, EBC_WIN_VIR, win_vir ? win_vir : ebc->width);
	ebc_write(ebc, EBC_WIN_ACT, size);
	ebc_write(ebc, EBC_WIN_DSP, size);
	ebc_write(ebc, EBC_WIN_DSP_ST,
		  (t->hsync_len.typ + t->hback_porch.typ + 1) |
		  ((t->vsync_len.typ + 4) << 16));
	ebc_config_done(ebc);
}

/* VCOM0..3 stay zero on this panel. */
static void ebc_set_border(struct rk3026_ebc *ebc)
{
	ebc_write(ebc, EBC_VCOM0, 0);
	ebc_write(ebc, EBC_VCOM1, 0);
	ebc_write(ebc, EBC_VCOM2, 0);
	ebc_write(ebc, EBC_VCOM3, 0);
}

static void ebc_set_epd_ctrl(struct rk3026_ebc *ebc)
{
	u32 gdck = ebc->timing.hsync_len.typ + ebc->gdck_sta;
	u32 n = ebc->width < 0x400 ? ebc->width / 4 : 0xff;

	if (sdck >= 0)
		n = sdck;

	ebc_write(ebc, EBC_EPD_CTRL,
		  EPD_GDCK_STA(gdck) | EPD_LGONL(gdck + ebc->lgonl) |
		  EPD_SDCK(n) | EPD_GATE_SCAN_DIR | EPD_SOURCE_SCAN_DIR);
}

static void ebc_set_dsp_ctrl(struct rk3026_ebc *ebc, u32 mode)
{
	ebc->dsp_ctrl = DSP_CTRL_FIXED | DSP_VCOM_MODE |
			rk3026_ebc_dsp_ctrl_mode(mode);
	ebc_write(ebc, EBC_DSP_CTRL, ebc->dsp_ctrl);
	ebc_config_done(ebc);
}

/*
 * MST0 holds PREV -- the image on the panel -- and MST1 holds NEXT, the one
 * being driven towards.
 *
 * Two independent lines of evidence, which finally agree:
 *
 * - The audited vendor kernel (docs/re/EBC-REGISTERS.md §5). ebc_register_update
 *   at 0xc0607414 loads [dev+0x18c] into arg1 -> MST0 and [dev+0x190] into
 *   arg2 -> MST1, dispatched through ops+0x14. ebc_thread proves which is which:
 *   it stores the freshly dequeued buffer to 0x190 (0xc0609728), drives, then
 *   ebc_buf_release()s 0x18c (0xc06098f8) and rotates 0x18c = 0x190. The buffer
 *   released after the scan is the one that was on the glass.
 *
 * - Hardware, 2026-08-05: with next in MST0 a black frame renders LIGHT and a
 *   white frame renders darker -- the transposed lookup, driving one-directionally
 *   toward white. With prev in MST0, white brightens and black darkens.
 *   Reproduced twice, same card, same boot, identical transitions, every draw
 *   with a nonzero `differ from prev`.
 *
 * ⚠️ This REVERSES what this driver did between 2026-08-03 and 2026-08-05
 * (f2227388e331), which put next in MST0 on the strength of docs/VENDOR-BLOBS.md
 * §5 -- a decompile of a linux/.refs/rychly object with no evidentiary standing.
 * The comment that stood here called that "PROVEN ON HARDWARE" and credited it
 * with making the panel image; the session it cites changed more than one thing
 * at once, so it never isolated MST order at all.
 *
 * ⚠️ An even earlier comment claimed the order this code now uses was
 * "established ON HARDWARE", in a session where the panel rendered nothing. It
 * happened to be right, and it was still a belief recorded as a measurement.
 * Being accidentally correct is not evidence.
 *
 * ⚠️ MST order and LUT index order are a PAIR: inverting one alone inverts the
 * transition lookup. Do not change one without re-testing direction on glass.
 *
 * ⚠️ WIN_MST0/1 are SHADOWED: a write does not affect the scan that follows it,
 * only the one after, because the shadow goes active on a latch the scan itself
 * performs. So flipping this (or swap_mst) and running ONE scan tests nothing —
 * the first scan still uses the previous pair. Verify against the `armed`
 * readback, not against what was just written.
 *
 * Both hold absolute physical addresses, biased by the window origin at two
 * pixels per byte. The row stride is the panel width: the images are packed
 * at width / 2 bytes per line (rk3026_ebc_blit_xrgb8888()), and WIN_VIR, from
 * which the controller takes its line step, is the width as well.
 */
static void ebc_set_image_addr(struct rk3026_ebc *ebc, u32 x, u32 y)
{
	u32 off = (x + y * ebc->width) / 2;
	dma_addr_t m0 = swap_mst ? ebc->next_phys : ebc->prev_phys;
	dma_addr_t m1 = swap_mst ? ebc->prev_phys : ebc->next_phys;

	ebc_write(ebc, EBC_WIN_MST0, lower_32_bits(m0) + off);
	ebc_write(ebc, EBC_WIN_MST1, lower_32_bits(m1) + off);
	ebc_config_done(ebc);
}

/*
 * Kick one waveform chunk: at most EBC_LUT_MAX_FRAMES frames of LUT into the
 * SRAM window, then the window geometry, the per-chunk frame count and the
 * scan itself.
 *
 * Everything up to CONFIG_DONE is a register group latched by that write, and
 * FRAME_START is only asserted afterwards. VCOM_MODE stays set while further
 * chunks follow so the rails are not dropped mid-waveform.
 */
static void ebc_frame_kick(struct rk3026_ebc *ebc, const u8 *lut,
			   u32 chunk, u32 frames, bool last)
{
	const u8 *src = lut + chunk * EBC_LUT_MAX_FRAMES * EBC_LUT_FRAME_BYTES;
	u32 size = ebc->width | (ebc->height << 16);
	const struct display_timing *t = &ebc->timing;
	u32 words = frames * (EBC_LUT_FRAME_BYTES / 4);
	u32 i;

	for (i = 0; i < words; i++)
		ebc_write(ebc, EBC_LUT_BASE + i * 4, get_unaligned_le32(src + i * 4));

	/*
	 * Drain everything the CPU has written before the controller is told to
	 * fetch it. Two distinct things need this and neither had it:
	 *
	 *  - the IMAGE, in prev_virt/next_virt, which are DMA_ATTR_WRITE_COMBINE
	 *    (ebc_setup_buffers). Write-combining means the thread's copies into
	 *    them sit in a merge buffer with no ordering against the EBC's AXI
	 *    master reading the same memory.
	 *  - the LUT SRAM and the register group just above, written with
	 *    writel_relaxed() throughout, which by definition orders nothing.
	 *
	 * Why this is the prime suspect for docs/PENDING.md P15 rather than a
	 * theoretical tidy-up: it predicts exactly the observed asymmetry. Uniform
	 * black/white always rendered, because when every byte is identical it does
	 * not matter which stores landed or in what order. `ybands` renders, because
	 * 64-line bands are long contiguous runs and whole bursts survive. `xbands`
	 * does not, because variation WITHIN a line is the finest granularity and
	 * exactly what write-combining coalesces away. And the vendor path draws the
	 * same patterns correctly from the same 4bpp packing (proven on hardware
	 * 2026-08-05), so the format is not the problem.
	 *
	 * The vendor does the equivalent explicitly: flush_cache_all() + outer_sync()
	 * after every generated frame, on a pool that is additionally mapped uncached
	 * (docs/re/EBC-DATAPATH.md).
	 *
	 * ⚠️ ebc_report_lut() below is a READ of the LUT window, and a read drains
	 * posted writes. So while debug_scan was armed this ordering happened by
	 * accident, and it stopped happening when the budget ran out -- which makes
	 * every "608/608 LUT words match" measurement one taken under a barrier that
	 * normal operation did not have. The barrier must not depend on logging.
	 */
	wmb();

	if (debug_scan)
		ebc_report_lut(ebc, src, words, chunk);

	ebc_write(ebc, EBC_WIN_DSP_ST,
		  (t->hsync_len.typ + t->hback_porch.typ + 1) |
		  ((t->vsync_len.typ + 4) << 16));
	ebc_write(ebc, EBC_WIN_DSP, size);
	ebc_write(ebc, EBC_WIN_ACT, size);

	ebc->int_reg = EBC_INT_FRAME_NUM(frames) | EBC_INT_ENABLE;
	ebc_write(ebc, EBC_INT, ebc->int_reg);

	ebc->dsp_ctrl &= ~DSP_VCOM_MODE;
	if (!last)
		ebc->dsp_ctrl |= DSP_VCOM_MODE;
	ebc_write(ebc, EBC_DSP_CTRL, ebc->dsp_ctrl);

	ebc->dsp_start &= ~DSP_FRAME_NUM_MASK;
	ebc->dsp_start |= DSP_FRAME_NUM(frames - 1);
	ebc_write(ebc, EBC_DSP_START, ebc->dsp_start);

	ebc_config_done(ebc);

	/*
	 * Second drain, and the placement is the point: everything from WIN_DSP_ST
	 * down to CONFIG_DONE above is writel_relaxed(), so the barrier after the
	 * LUT does NOT order this register group against the FRAME_START below.
	 * Without this the controller can be started before its own window and
	 * frame count have landed.
	 */
	wmb();

	ebc->dsp_start |= DSP_FRAME_START;
	ebc_write(ebc, EBC_DSP_START, ebc->dsp_start);
}

static irqreturn_t ebc_irq(int irq, void *data)
{
	struct rk3026_ebc *ebc = data;
	u32 status = ebc_read(ebc, EBC_INT);

	dbg_last_status = status;
	atomic_inc(&dbg_irqs);

	if (!(status & EBC_INT_FRAME_END))
		return IRQ_NONE;

	/*
	 * Acknowledge order: the three write-one-to-clear bits are asserted one
	 * at a time, then display-end is masked and unmasked again.
	 */
	ebc->int_reg |= EBC_INT_FRAMEEND_CLEAR;
	ebc_write(ebc, EBC_INT, ebc->int_reg);
	ebc->int_reg |= EBC_INT_DISPLAYEND_CLEAR;
	ebc_write(ebc, EBC_INT, ebc->int_reg);
	ebc->int_reg |= EBC_INT_FRAMEFLAG_CLEAR;
	ebc_write(ebc, EBC_INT, ebc->int_reg);
	ebc->int_reg |= EBC_INT_DISPLAYEND_MASK;
	ebc_write(ebc, EBC_INT, ebc->int_reg);

	ebc->int_reg &= ~EBC_INT_DISPLAYEND_MASK;
	ebc_write(ebc, EBC_INT, ebc->int_reg);
	ebc_config_done(ebc);

	complete(&ebc->frame_done);
	return IRQ_HANDLED;
}

/* Panel temperature, in degrees Celsius. */
static int ebc_read_temp(struct rk3026_ebc *ebc)
{
	int temp;

	if (!ebc->tzd || thermal_zone_get_temp(ebc->tzd, &temp))
		return EBC_TEMP_FALLBACK;

	temp /= 1000;
	/* A reading above 50 degrees is not trusted; fall back to 25. */
	if (temp > EBC_TEMP_MAX) {
		dev_warn_ratelimited(ebc->dev,
				     "panel temperature %d out of range, using %d\n",
				     temp, EBC_TEMP_FALLBACK);
		temp = EBC_TEMP_FALLBACK;
	}
	return temp;
}

/*
 * Microseconds since @start, rounded up and at least 1, so a time that was
 * measured is never 0 (0 means none), even on a clock too coarse to see it.
 */
static u32 ebc_us_since(ktime_t start)
{
	u64 ns = ktime_to_ns(ktime_sub(ktime_get(), start));

	return clamp_t(u64, DIV_ROUND_UP_ULL(ns, NSEC_PER_USEC), 1, U32_MAX);
}

/**
 * rk3026_ebc_blit_xrgb8888 - pack a damaged region into the target image
 * @ebc: controller
 * @src: XRGB8888 source, top-left of the whole framebuffer
 * @src_pitch: source stride in bytes
 * @clip: damaged region, in pixels
 *
 * The panel is 4bpp with two pixels per byte, low nibble first. The clip is
 * widened to an even pixel boundary so byte writes stay whole, and clamped
 * to the panel; rk3026_ebc_blit_convert() converts it.
 *
 * Writes @next_shadow. Called under @image_lock, in the same section that
 * queues the update, so no snapshot sees the pixels without their update.
 *
 * Returns the time the conversion took in us, rounded up, or 0 if the clip
 * is off the panel; the sender passes it on with the update
 * (&rk3026_ebc_update.blit_us).
 */
u32 rk3026_ebc_blit_xrgb8888(struct rk3026_ebc *ebc, const void *src,
			     unsigned int src_pitch, const struct drm_rect *clip)
{
	struct drm_rect r = *clip;
	ktime_t start;

	lockdep_assert_held(&ebc->image_lock);
	if (!rk3026_ebc_widen_clip(&r, ebc->width, ebc->height))
		return 0;

	start = ktime_get();
	rk3026_ebc_blit_convert(ebc->next_shadow, src, src_pitch, r.x1, r.x2,
				r.y1, r.y2, ebc->width);
	return ebc_us_since(start);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_blit_xrgb8888);

static void ebc_program_regs(struct rk3026_ebc *ebc);
static int ebc_rails_off(struct rk3026_ebc *ebc);

/*
 * Both rails are held, so a chunk may be kicked. Lockless: the rails only go
 * down under the thread's feet when a CRTC disable gave up waiting for it.
 */
static bool ebc_rails_up(struct rk3026_ebc *ebc)
{
	return READ_ONCE(ebc->epd_on) &&
	       (!ebc->vcom_supply || READ_ONCE(ebc->vcom_on));
}

/*
 * A chunk did not end within its budget: stop the controller before the
 * rails go down and before anything else is kicked.
 *
 * With the block's resets described (SRST_EBC_AXI/AHB) it is reset and
 * reprogrammed: that stops a scan in progress, drops a pending interrupt and
 * clears the LUT SRAM, which the next kick writes again. Without them the
 * best the documented registers offer is to un-arm it (docs: DSP_START b0 is
 * the start bit, nothing is known to stop a running scan): start bit and
 * VCOM_MODE cleared, the display-end interrupt masked and acknowledged. The
 * interrupt handler is kept out meanwhile, since it shares @int_reg.
 */
static void ebc_stop_scan(struct rk3026_ebc *ebc)
{
	disable_irq(ebc->irq);

	if (ebc->rst) {
		reset_control_assert(ebc->rst);
		usleep_range(10, 20);
		reset_control_deassert(ebc->rst);
		ebc->dsp_start = 0;
		ebc->int_reg = 0;
		ebc_program_regs(ebc);
	} else {
		ebc->dsp_start &= ~DSP_FRAME_START;
		ebc_write(ebc, EBC_DSP_START, ebc->dsp_start);
		ebc->dsp_ctrl &= ~DSP_VCOM_MODE;
		ebc_write(ebc, EBC_DSP_CTRL, ebc->dsp_ctrl);
		ebc->int_reg |= EBC_INT_DISPLAYEND_MASK | EBC_INT_FRAMEEND_CLEAR |
				EBC_INT_DISPLAYEND_CLEAR | EBC_INT_FRAMEFLAG_CLEAR;
		ebc_write(ebc, EBC_INT, ebc->int_reg);
		ebc_config_done(ebc);
	}

	enable_irq(ebc->irq);
	dev_warn(ebc->dev, "controller stopped (%s), rails off\n",
		 ebc->rst ? "reset" : "un-armed");
}

/**
 * rk3026_ebc_drive - run one whole waveform, and own the "is prev true?" flag
 * @ebc: controller
 * @lut: packed LUT for @mode, @frames frames of EBC_LUT_FRAME_BYTES
 * @frames: frames in the waveform
 * @mode: EBC work mode, for the DSP_CTRL class
 * @lut_mode: the mode @lut belongs to, for the record
 * @last: the update's record: waveform, frames driven, time, outcome
 *
 * Returns 0 only if every frame was driven, -ETIMEDOUT if a chunk did not
 * end in its budget: the controller is then stopped and the rails are down,
 * so the caller must not drive anything else for this update. -EIO if the
 * rails were found down before a chunk.
 *
 * @shadows.prev_unknown is set here and cleared in exactly ONE place of this
 * function -- the successful exit below. rk3026_ebc_scan_finish() records the
 * part's outcome afterwards and never clears what this left set.
 * That asymmetry is the whole design: a refresh that ends any other way,
 * including one added later by someone who never read this comment, leaves
 * the flag set without having to be listed anywhere. Do not "improve" this
 * into a set of clears on the known failure paths; the failure that matters
 * is the path not yet written.
 *
 * KUnit replaces it with a static stub, so that the refresh thread's glue
 * can be run to a driven part without the controller.
 */
VISIBLE_IF_KUNIT int rk3026_ebc_drive(struct rk3026_ebc *ebc, const u8 *lut,
				      u32 frames, u32 mode, u32 lut_mode,
				      struct rk3026_ebc_last *last)
{
	u32 done = 0, chunk = 0;
	ktime_t start = ktime_get();
	int ret = 0;

	KUNIT_STATIC_STUB_REDIRECT(rk3026_ebc_drive, ebc, lut, frames, mode,
				   lut_mode, last);

	/* A repair is the first waveform of an update that has repaired set. */
	trace_rk3026_ebc_scan_start(mode, lut_mode, last->temp, frames,
				    last->marker, last->merged,
				    last->repaired && !last->scans,
				    last->reason);

	WRITE_ONCE(ebc->shadows.prev_unknown, true);
	last->scans++;
	last->lut_mode = lut_mode;
	last->frames = frames;

	ebc_set_dsp_ctrl(ebc, mode);
	ebc_set_image_addr(ebc, 0, 0);

	if (debug_scan)
		ebc_report_regs(ebc, "armed");

	while (done < frames) {
		u32 n = min_t(u32, frames - done, EBC_LUT_MAX_FRAMES);
		int irqs = atomic_read(&dbg_irqs);
		ktime_t t0 = ktime_get();
		bool ok;

		unsigned int budget = ebc_chunk_timeout_ms(ebc, n);

		if (!ebc_rails_up(ebc)) {
			dev_warn_ratelimited(ebc->dev,
					     "rails down before chunk %u, waveform abandoned\n",
					     chunk);
			last->outcome = EBC_OUTCOME_ABANDONED;
			ret = -EIO;
			goto out;
		}

		/* DIAGNOSTIC: make the wait expire for real. See debug_timeout. */
		if (debug_timeout) {
			debug_timeout--;
			budget = 1;
		}

		reinit_completion(&ebc->frame_done);
		ebc_frame_kick(ebc, lut, chunk, n, done + n >= frames);
		ok = wait_for_completion_timeout(&ebc->frame_done,
						msecs_to_jiffies(budget));
		if (debug_scan)
			dev_info(ebc->dev,
				 "scan: chunk %u %u frames %s in %lld ms (budget %u), %d irqs, status %08x\n",
				 chunk, n, ok ? "ended" : "TIMED OUT",
				 ktime_ms_delta(ktime_get(), t0), budget,
				 atomic_read(&dbg_irqs) - irqs, dbg_last_status);
		if (!ok) {
			dev_warn_ratelimited(ebc->dev, "frame timeout\n");
			ebc_stop_scan(ebc);
			mutex_lock(&ebc->rails_lock);
			ebc_rails_off(ebc);
			mutex_unlock(&ebc->rails_lock);
			last->timeouts++;
			last->outcome = EBC_OUTCOME_TIMEOUT;
			ret = -ETIMEDOUT;
			goto out;
		}
		done += n;
		chunk++;
	}

	if (debug_scan) {
		ebc_report_regs(ebc, "done");
		dev_info(ebc->dev, "scan: %u of %u frames driven\n", done, frames);
	}

	/* The idle deadline runs from the end of the last waveform. */
	ebc_rails_rearm(ebc);

	WRITE_ONCE(ebc->shadows.prev_unknown, false);
	last->outcome = EBC_OUTCOME_DRIVEN;
out:
	last->driven = done;
	last->scan_ms = ktime_ms_delta(ktime_get(), start);
	trace_rk3026_ebc_scan_end(mode, frames, done, last->scan_ms, ret);
	return ret;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_drive);

/**
 * ebc_recover_panel - drive the glass back to a state @prev_virt can describe
 * @ebc: controller
 * @temp: panel temperature, already clamped
 * @last: the update's record
 *
 * A RESET waveform reads NEITHER image. rkf_expand_reset() fills every one of a
 * frame's 16 words -- the index is the new grey -- with the same phase *
 * 0x55555555, so all four old-grey fields carry that phase too: the whole panel
 * is driven through one fixed phase sequence with the pixel data irrelevant.
 *
 * That is what makes this repair cheap and content-preserving. It needs no third
 * buffer and no MST reprogramming (WIN_MST0/1 are shadowed, so a per-pass swap
 * would land on the wrong pass anyway); @next_virt keeps the snapshot
 * untouched through the RESET and is drawn immediately afterwards from a @prev
 * that is now true.
 *
 * White is the state a RESET lands on -- the same assumption
 * rk3026_ebc_add_update() makes when it fills @next_shadow with 0xff for a
 * RESET. @prev_virt follows here, for the part's scan; @shadows.prev follows
 * in rk3026_ebc_scan_finish() (EBC_REPAIR_DONE), whatever the part's outcome.
 *
 * Returns 0 if the RESET ran to completion (@last->repaired set) or there is
 * no RESET waveform at @temp (@last->repaired clear: nothing was driven), or
 * rk3026_ebc_drive()'s error: the rails are down and the part must not be
 * driven.
 */
static int ebc_recover_panel(struct rk3026_ebc *ebc, int temp,
			     struct rk3026_ebc_last *last)
{
	const u8 *lut;
	u32 frames;
	int ret;

	lut = rk3026_ebc_waveform_lut(ebc->waveform, EBC_MODE_RESET, temp,
				      &frames);
	if (!lut) {
		dev_warn_ratelimited(ebc->dev,
				     "panel state unknown and no RESET waveform at %d C\n",
				     temp);
		return 0;
	}

	dev_warn(ebc->dev, "panel state unknown, clearing with %u RESET frames\n",
		 frames);

	last->repaired = true;
	ret = rk3026_ebc_drive(ebc, lut, frames, EBC_MODE_RESET, EBC_MODE_RESET,
			       last);
	if (ret)
		return ret;

	memset(ebc->prev_virt, 0xff, ebc->img_size);
	return 0;
}

/*
 * Panel rails, refresh thread and CRTC disable only, under @rails_lock. The
 * TPS65185 sequences its rails internally once "epd" is enabled; VCOM goes
 * last because its amplifier needs the positive boost in regulation, and
 * first on the way down.
 */
static int ebc_rails_on(struct rk3026_ebc *ebc)
{
	bool changed = false;
	int ret;

	lockdep_assert_held(&ebc->rails_lock);
	if (!ebc->epd_on) {
		ret = regulator_enable(ebc->epd_supply);
		if (ret) {
			dev_err_ratelimited(ebc->dev, "failed to enable EPD rails: %d\n", ret);
			trace_rk3026_ebc_rails(true, ebc->epd_on, ebc->vcom_on, ret);
			return ret;
		}
		changed = true;
		WRITE_ONCE(ebc->epd_on, true);
		WRITE_ONCE(ebc->rails_on, true);
		/* Off again after the delay even if nothing is driven. */
		ebc_rails_rearm(ebc);
	}
	if (ebc->vcom_supply && !ebc->vcom_on) {
		ret = regulator_enable(ebc->vcom_supply);
		if (ret) {
			dev_err_ratelimited(ebc->dev, "failed to enable VCOM: %d\n", ret);
			trace_rk3026_ebc_rails(true, ebc->epd_on, ebc->vcom_on, ret);
			/*
			 * No waveform without VCOM: take the high-voltage rails
			 * down again. If that fails too, @epd_on stays set and
			 * the idle tick retries it.
			 */
			ebc_rails_off(ebc);
			return ret;
		}
		WRITE_ONCE(ebc->vcom_on, true);
		changed = true;
	}
	/* dev_dbg only: a console line would itself queue a refresh. */
	dev_dbg(ebc->dev, "rails on\n");
	if (changed)
		trace_rk3026_ebc_rails(true, ebc->epd_on, ebc->vcom_on, 0);
	return 0;
}

/*
 * VCOM first, then the high-voltage rails. A regulator_disable() that fails
 * has not dropped the consumer's enable count (the core returns before it
 * does), so the supply is still ours: its flag stays set, @rails_on stays
 * true, and the caller's next attempt disables exactly what is still on.
 * The epd rail is not dropped while VCOM is still held: VCOM's own supply is
 * that rail, so it would stay up regardless.
 */
static int ebc_rails_off(struct rk3026_ebc *ebc)
{
	int ret;

	lockdep_assert_held(&ebc->rails_lock);
	if (ebc->vcom_on) {
		ret = regulator_disable(ebc->vcom_supply);
		if (ret)
			goto err;
		WRITE_ONCE(ebc->vcom_on, false);
	}
	if (ebc->epd_on) {
		ret = regulator_disable(ebc->epd_supply);
		if (ret)
			goto err;
		WRITE_ONCE(ebc->epd_on, false);
	}
	if (ebc->rails_on) {
		dev_dbg(ebc->dev, "rails off\n");
		trace_rk3026_ebc_rails(false, ebc->epd_on, ebc->vcom_on, 0);
	}
	WRITE_ONCE(ebc->rails_on, false);
	return 0;

err:
	trace_rk3026_ebc_rails(false, ebc->epd_on, ebc->vcom_on, ret);
	dev_err_ratelimited(ebc->dev,
			    "failed to switch the panel rails off (%d), will retry\n",
			    ret);
	/* Next attempt one delay from now (at least 100 ms), not in a tight loop. */
	WRITE_ONCE(ebc->rails_off_at,
		   jiffies + msecs_to_jiffies(max(READ_ONCE(rails_off_ms), 100U)));
	return ret;
}

/*
 * Publish the record of a finished part, and count it. Called under
 * @image_lock, in the section of rk3026_ebc_scan_finish(), so a debugfs
 * "panel" snapshot (taken under @image_lock) never shows an image the
 * counters do not cover.
 */
static void ebc_publish(struct rk3026_ebc *ebc, struct rk3026_ebc_last *last,
			ktime_t start)
{
	struct rk3026_ebc_stats *st = &ebc->stats;

	lockdep_assert_held(&ebc->image_lock);
	last->ms = ktime_ms_delta(ktime_get(), start);

	switch (last->outcome) {
	case EBC_OUTCOME_SKIPPED:
	case EBC_OUTCOME_CRTC_OFF:
	case EBC_OUTCOME_NO_WAVEFORM:
	case EBC_OUTCOME_NO_RAILS:
	case EBC_OUTCOME_NO_LUT:
		trace_rk3026_ebc_skip(last->asked_mode, last->marker,
				      last->outcome);
		break;
	default:
		break;
	}

	ebc->last = *last;
	if (last->scans)
		ebc->last_driven = *last;
	st->done++;
	st->scans += last->scans;
	st->timeouts += last->timeouts;
	switch (last->outcome) {
	case EBC_OUTCOME_SKIPPED:
		st->skips++;
		break;
	case EBC_OUTCOME_CRTC_OFF:
	case EBC_OUTCOME_NO_WAVEFORM:
	case EBC_OUTCOME_NO_RAILS:
	case EBC_OUTCOME_NO_LUT:
		st->dropped++;
		break;
	case EBC_OUTCOME_ABANDONED:
		st->abandoned++;
		break;
	default:
		break;
	}
}

/*
 * Whether the waveform has an A2 table of its own at the temperature @part
 * is likely drawn at (rk3026_ebc_resolve_mode()), under @image_lock. The
 * sensor is read only after the lock is dropped, so the temperature is the
 * part's own if it brings one, else the one the last driven part was drawn
 * at; with neither, no A2. Should the sensor then say otherwise, the A2
 * fetch falls back to the DU table (rk3026_ebc_waveform_lut()): the part is
 * drawn as DU, in the class A2 and DU share, and is never carried for it.
 */
static bool ebc_a2_lut(struct rk3026_ebc *ebc,
		       const struct rk3026_ebc_set *part)
{
	int temp;

	if (part->temp_valid)
		temp = part->temp > EBC_TEMP_MAX ? EBC_TEMP_FALLBACK :
						   part->temp;
	else if (ebc->last_driven.temp_read)
		temp = ebc->last_driven.temp;
	else
		return false;
	return rk3026_ebc_waveform_has_a2(ebc->waveform, temp);
}

/*
 * Take the next part and drive it onto the panel, if anything is pending.
 * Runs only on the refresh thread.
 *
 * Three steps, so that no sender waits for a waveform:
 *
 *  1. under @image_lock: the part is taken (rk3026_ebc_pending_take(): what
 *     was sent before a RESET, then the RESET alone, then what was sent
 *     after it), and its snapshot is built into @shadows.scan from
 *     @shadows.prev and the target (@next_shadow, or @pre_shadow for what
 *     was sent before a pending RESET). Senders may blit into @next_shadow
 *     and add to @pending as soon as the lock is dropped: the snapshot is
 *     what this scan drives, and their updates go to the next part.
 *  2. without it: the sensor, the rails, the snapshot into @next_virt, a
 *     repair if the glass is not known, the waveform;
 *  3. under @image_lock again: rk3026_ebc_scan_finish() says what the glass
 *     shows now and moves @pending.done_seq over the part, the DU cleanup
 *     budget follows it (rk3026_ebc_du_account()), and the record is
 *     published with them. The waiters on the part's updates are woken then,
 *     whatever the outcome, and @prev_virt follows.
 */
VISIBLE_IF_KUNIT void rk3026_ebc_do_refresh(struct rk3026_ebc *ebc,
					    struct rk3026_ebc_last *last)
{
	enum rk3026_ebc_repair repair_result = EBC_REPAIR_NONE;
	struct rk3026_ebc_shadows *sh = &ebc->shadows;
	ktime_t start = ktime_get(), snap_start;
	enum rk3026_ebc_reason reason;
	struct rk3026_ebc_du_budget budget;
	struct rk3026_ebc_a2 a2 = { 0 };
	struct rk3026_ebc_set part;
	bool repair_due, identical, prev_changed, two_level;
	u32 frames, mode, scan_mode, resolved, lut_mode;
	u32 changed_px = 0;
	const u8 *lut, *target;
	int temp;
	int ret;

	mutex_lock(&ebc->image_lock);
	if (!rk3026_ebc_pending_take(&ebc->pending, &part)) {
		mutex_unlock(&ebc->image_lock);
		return;
	}

	/*
	 * Taken while @pre_split, the part is the @pending.pre a RESET was sent
	 * behind (it goes first): it is drawn from the image its updates were
	 * sent into, and that image is the spare again.
	 */
	target = ebc->next_shadow;
	if (ebc->pre_split) {
		target = ebc->pre_shadow;
		ebc->pre_split = false;
	}

	memset(last, 0, sizeof(*last));
	last->clip = part.box;
	last->asked_mode = part.mode;
	last->mode = part.mode;
	last->lut_mode = part.mode;
	last->marker = part.marker;
	last->merged = part.merged;
	last->source = part.source;
	last->blit_us = part.blit_us;

	/*
	 * Nothing reaches the panel while the CRTC is off: the rails stay
	 * down and the prev images are untouched. The part is dropped, not
	 * carried (the RESET of the next CRTC enable clears the glass), and
	 * the waits on its updates end like any other part's.
	 */
	if (!ebc->crtc_on) {
		dev_dbg(ebc->dev, "CRTC off, part (mode %u) dropped\n",
			part.mode);
		last->outcome = EBC_OUTCOME_CRTC_OFF;
		goto finish;
	}
	if (!ebc->waveform) {
		last->outcome = EBC_OUTCOME_NO_WAVEFORM;
		goto finish;
	}

	/*
	 * A glass that is not known is repaired with a RESET before the draw
	 * (below), except before a RESET, which clears it anyway. After a
	 * repair the glass is white outside the box too, while the snapshot
	 * there holds the old greys, so the scan runs at least PART; the
	 * snapshot is then not thresholded either.
	 */
	repair_due = sh->prev_unknown && part.mode != EBC_MODE_RESET;
	scan_mode = rk3026_ebc_scan_mode(&part, repair_due);

	/* An A2 scan over a grey of the glass comes back as DU. */
	snap_start = ktime_get();
	identical = !rk3026_ebc_build_snapshot(sh, target, &part.box,
					       &scan_mode, &two_level,
					       &a2.over_grey, &changed_px);
	last->snap_us = ebc_us_since(snap_start);
	if (debug_scan)
		ebc_report_image(ebc);

	/*
	 * Nothing changed? Then do not drive the panel.
	 *
	 * This is not an optimisation, it is a correctness and wear fix. Every
	 * update arrives here TWICE: FBIOPAN_DISPLAY alone produces two atomic
	 * commits through drm_fb_helper's fbdev emulation (measured 2026-08-06 --
	 * skipping the client's msync changes nothing), and ebc_plane_atomic_update()
	 * queues the damage of each without asking whether the pixels moved. So
	 * the second scan runs a complete waveform with prev == next.
	 *
	 * A GC16 waveform with identical prev and next is NOT a no-op: it still
	 * runs its whole flash sequence, so it re-shows the previous image and
	 * settles back. At 12 MHz that is a visible three-second replay after
	 * every single draw, which is what "the previous image comes back" was.
	 * Left alone it is also continuous wear -- fbcon's cursor blink drove a
	 * 38-frame full waveform every ~1.1 s indefinitely (docs/PENDING.md P29).
	 *
	 * The test is on the cached snapshot against the cached prev, never on
	 * the write-combined images. rk3026_ebc_should_skip() holds the
	 * exemptions:
	 *
	 * ⚠️ EBC_MODE_RESET is exempt, and the test is on the PART's own mode,
	 * not the possibly-forced one. The boot-time clear legitimately has
	 * prev == next == white: the controller indexes its LUT by (prev, next),
	 * so the driver must MAKE the panel white rather than assume it is, and
	 * skipping that would leave whatever the previous OS left on the glass.
	 * Keying off `mode` instead would let force_mode disable the boot clear.
	 *
	 * ⚠️ DO NOT DROP the @prev_unknown exemption. The skip is a decision made
	 * from @shadows.prev, and the flag says it is not evidence; without the
	 * exemption an aborted waveform makes the draw that would repair the
	 * glass the one thrown away, and the panel is unreachable from userspace.
	 * It does not weaken the skip -- a successful refresh clears the flag, so
	 * P29's case (fbcon redrawing one image forever) still skips.
	 *
	 * ⚠️ An MXCFB_SEND_UPDATE in a full-class mode is exempt too
	 * (@has_mxcfb_full; full class = what rk3026_ebc_dsp_ctrl_mode() gives
	 * DISPLAY_MODE alone: FULL, i.e. GC16, REAGL or AUTO with
	 * UPDATE_MODE_FULL and REAGLD, OED_PART for GL16 with UPDATE_MODE_FULL,
	 * RESET, BLOCK, FULL_DITHER). Neither reason for the skip applies to
	 * it: it is queued once, by a client, not by every fbdev commit or
	 * cursor blink. A GC16 full on identical content
	 * is the ghost-clearing flash the client asks for, and a full sent
	 * after a scan that already drew the same pixels would otherwise be
	 * skipped as identical. Plane updates and partial-class mxcfb sends
	 * still skip.
	 *
	 * ⚠️ The idle cleanup is exempt as well (@cleanup_idle): it redraws the
	 * glass as it is, so its snapshot is identical as a rule, and the
	 * flash is what it was queued for (rk3026_ebc_idle_tick()).
	 */
	if (rk3026_ebc_should_skip(&part, sh->prev_unknown, identical)) {
		if (debug_scan)
			dev_info(ebc->dev, "scan: skipped, image identical to prev\n");
		last->outcome = EBC_OUTCOME_SKIPPED;
		goto finish;
	}

	/*
	 * The content's choice for a part of auto updates: DU when every
	 * changed pixel becomes black or white (A2 for a small plane part, if
	 * a2_max_area allows it), else PART for a plane part, unless the plane
	 * parts' DU budget makes it the cleanup FULL (a big part once it is
	 * spent or when it would spend it, any part at its ceiling). The
	 * idle cleanup keeps its FULL. It comes after the skip, which stays
	 * on the part's own mode, and before force_mode.
	 */
	budget.area = ebc->du_area;
	budget.limit = READ_ONCE(ebc->du_cleanup_area);
	budget.big_area = READ_ONCE(ebc->du_cleanup_big_area);
	budget.ceiling = READ_ONCE(ebc->du_cleanup_ceiling);
	a2.max_area = READ_ONCE(ebc->a2_max_area);
	if (a2.max_area)
		a2.lut = ebc_a2_lut(ebc, &part);
	resolved = rk3026_ebc_resolve_mode(&part, scan_mode, two_level,
					   changed_px, repair_due, &budget,
					   &a2, &reason);
	last->reason = reason;

	/* DIAGNOSTIC: drive a different mode class with the same data and scan. */
	mode = resolved;
	if (force_mode >= 0 && force_mode <= EBC_MODE_MAX)
		mode = force_mode;
	last->mode = mode;
	mutex_unlock(&ebc->image_lock);

	/*
	 * The sensor read is only needed when a waveform follows, so it comes
	 * after the skip, and it is done without @image_lock so that no sender
	 * waits for it.
	 */
	if (part.temp_valid)
		temp = part.temp;
	else
		temp = ebc_read_temp(ebc);
	if (temp > EBC_TEMP_MAX)
		temp = EBC_TEMP_FALLBACK;
	last->temp = temp;
	last->temp_read = true;

	/*
	 * Rails up before anything is driven, the repair below included. If they
	 * do not come up nothing is driven and the glass is as it was, so the
	 * prev image stays true and the part is carried into the next scan --
	 * except for a RESET, which is only queued when the glass is not known
	 * (the boot clear) and so leaves the mark (rk3026_ebc_scan_finish()).
	 */
	mutex_lock(&ebc->rails_lock);
	ret = ebc_rails_on(ebc);
	mutex_unlock(&ebc->rails_lock);
	if (ret) {
		last->outcome = EBC_OUTCOME_NO_RAILS;
		goto relock;
	}

	/* The controller reads the snapshot; nothing on the CPU reads it back. */
	memcpy(ebc->next_virt, sh->scan, ebc->img_size);

	/*
	 * Repair before drawing, never instead of it: this returns with the prev
	 * images describing the glass again, and @next_virt still holding the
	 * snapshot, so the part below is driven from a true prev.
	 *
	 * ⚠️ It must run BEFORE the LUT is fetched. rk3026_ebc_waveform_lut()
	 * decodes into one buffer inside the container and hands out a pointer to
	 * it, so loading the RESET waveform overwrites whatever was fetched
	 * earlier -- fetching first would silently drive the requested mode's
	 * frame count through the RESET LUT.
	 */
	if (repair_due) {
		/*
		 * A RESET that failed took the rails down: nothing more this
		 * time, and the part is carried (rk3026_ebc_scan_finish()).
		 */
		if (ebc_recover_panel(ebc, temp, last)) {
			repair_result = EBC_REPAIR_FAILED;
			goto relock;
		}
		if (last->repaired)
			repair_result = EBC_REPAIR_DONE;
	}

	/*
	 * A direct mode has no LUT of its own; fall back to the resolved mode
	 * for the scan data so the DSP_CTRL class is the only thing that changed.
	 * A resolved mode without a LUT is not driven in another one: the part
	 * is carried in its plain mode (rk3026_ebc_scan_finish()). A carry
	 * alone is no work for the thread, so such a part waits for the next
	 * update and is drawn with it, in the stronger of the two plain modes:
	 * an auto plane part without a DU or PART LUT ends up FULL, as before
	 * the policy.
	 */
	lut_mode = mode;
	lut = rk3026_ebc_waveform_lut(ebc->waveform, mode, temp, &frames);
	if (!lut && mode != resolved) {
		lut_mode = resolved;
		lut = rk3026_ebc_waveform_lut(ebc->waveform, resolved, temp,
					      &frames);
	}
	if (!lut) {
		if (debug_scan)
			dev_info(ebc->dev, "scan: no LUT for mode %u at %d C\n",
				 mode, temp);
		last->outcome = EBC_OUTCOME_NO_LUT;
		goto relock;
	}

	if (debug_scan) {
		dev_info(ebc->dev,
			 "scan: mode %u (part asked %u) temp %d C, %u frames, clip %d,%d-%d,%d\n",
			 mode, part.mode, temp, frames, part.box.x1, part.box.y1,
			 part.box.x2, part.box.y2);
		ebc_report_env(ebc);
	}

	rk3026_ebc_drive(ebc, lut, frames, mode, lut_mode, last);

	if (debug_scan)
		debug_scan--;

relock:
	mutex_lock(&ebc->image_lock);
finish:
	/*
	 * What was driven in is now what the panel shows: once rk3026_ebc_drive()
	 * was entered for the part, the snapshot becomes prev whatever the
	 * outcome.
	 * A repair RESET that failed never got that far: the part is carried
	 * and prev stays, marked unknown.
	 *
	 * ⚠️ The swap STAYS UNCONDITIONAL once the waveform was entered. Keeping
	 * the old prev after a timeout looks like the fix and is not: prev would
	 * then hold a still older image, equally false and carrying no mark
	 * saying so. @prev_unknown is the mark, and the next draw acts on it.
	 */
	prev_changed = rk3026_ebc_scan_finish(sh, &ebc->pending, &part,
					      last->outcome, repair_result);
	ebc->du_area = rk3026_ebc_du_account(ebc->du_area, &part, last->mode,
					     changed_px, last->outcome,
					     repair_result);
	/*
	 * A part with a plane update in it arms the idle cleanup again at its
	 * end, so the idle delay runs from the end of the scan, not from the
	 * queue; and a look that disarmed it while the part was pending does
	 * not leave it unarmed.
	 */
	if (part.has_plane) {
		WRITE_ONCE(ebc->idle_last_plane, jiffies);
		WRITE_ONCE(ebc->idle_armed, true);
	}
	ebc_publish(ebc, last, start);
	mutex_unlock(&ebc->image_lock);

	/*
	 * @pending.done_seq moved over the part on every outcome -- driven,
	 * skipped, dropped with the CRTC off, carried without rails -- so
	 * every wait on one of its updates ends here.
	 */
	wake_up_all(&ebc->marker_wq);

	/* Only this thread writes either prev image, so this needs no lock. */
	if (prev_changed)
		memcpy(ebc->prev_virt, sh->prev, ebc->img_size);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_do_refresh);

/*
 * Idle power-off: rails_off_ms passed since the last waveform. Only this
 * thread drives the panel and it is here, so no waveform is in flight. Queued
 * work does not hold the rails up: an update that needs them brings them up
 * again, and one that is skipped never needed them.
 */
static void ebc_idle_rails_off(struct rk3026_ebc *ebc)
{
	mutex_lock(&ebc->rails_lock);
	ebc_rails_off(ebc);
	mutex_unlock(&ebc->rails_lock);
}

/*
 * The idle cleanup's deadline, read without @image_lock: for the wait of
 * the refresh thread alone, which decides under the lock
 * (rk3026_ebc_idle_tick()). Returns whether the cleanup is armed and has
 * a delay: with du_cleanup_idle_ms 0 there is nothing to look at, and the
 * thread does not wake for it. A write to that knob does not wake the
 * thread either: a new delay counts from the next wake.
 */
static bool ebc_idle_at(struct rk3026_ebc *ebc, unsigned long *at)
{
	u32 delay_ms = READ_ONCE(ebc->du_cleanup_idle_ms);

	*at = READ_ONCE(ebc->idle_last_plane) + msecs_to_jiffies(delay_ms);
	return READ_ONCE(ebc->idle_armed) && delay_ms;
}

/**
 * rk3026_ebc_idle_tick - the refresh thread's look at the idle cleanup
 * @ebc: controller
 * @now: the time of the look, in jiffies
 *
 * Called by the refresh thread between parts, once the deadline of the
 * armed cleanup has passed by its lockless read (ebc_idle_at()). Under
 * @image_lock it reads the state again and asks
 * rk3026_ebc_idle_cleanup_due(): a look that finds the delay over disarms
 * the cleanup, and if it is due, queues it as an update of its own
 * (EBC_SOURCE_IDLE): the whole panel, FULL, not auto. It goes through
 * rk3026_ebc_add_update() like a send, so it has a real seq, which
 * rk3026_ebc_scan_finish() moves @pending.done_seq to (a part made by hand
 * with no seq would move it backwards), and it is counted and traced like
 * any update. It writes no pixel: the scan draws the target as it is,
 * which is the glass, plus whatever joins the part before it is taken.
 *
 * A plane update that arrived after the lockless read moved the deadline:
 * the look under the lock sees it, and leaves the cleanup armed.
 *
 * Between the queue here and the take in rk3026_ebc_do_refresh() the
 * thread drops @image_lock for a moment, and an update sent in that
 * window joins the cleanup and is drawn in its whole-panel FULL. A first
 * mxcfb send gets a full flash it did not ask for then, once; such a
 * client runs its own fulls, and the window is the thread's own unlock
 * and lock.
 *
 * Return: true if the cleanup was queued.
 */
VISIBLE_IF_KUNIT bool rk3026_ebc_idle_tick(struct rk3026_ebc *ebc,
					   unsigned long now)
{
	struct rk3026_ebc_update upd = {
		.clip	= DRM_RECT_INIT(0, 0, ebc->width, ebc->height),
		.mode	= EBC_MODE_FULL,
		.source	= EBC_SOURCE_IDLE,
	};
	u32 delay_ms = READ_ONCE(ebc->du_cleanup_idle_ms);
	struct rk3026_ebc_du_budget budget = { 0 };
	struct rk3026_ebc_idle idle;
	bool due, expired;

	mutex_lock(&ebc->image_lock);
	budget.area = ebc->du_area;
	budget.limit = READ_ONCE(ebc->du_cleanup_area);
	idle = (struct rk3026_ebc_idle) {
		.armed		= ebc->idle_armed,
		.last_plane	= ebc->idle_last_plane,
		.delay		= msecs_to_jiffies(delay_ms),
		.spent		= rk3026_ebc_du_spent(&budget),
		.busy		= !rk3026_ebc_pending_empty(&ebc->pending) ||
				  ebc->pending.carry.merged || !ebc->crtc_on ||
				  ebc->mxcfb_active ||
				  ebc->shadows.prev_unknown,
	};
	due = rk3026_ebc_idle_cleanup_due(&idle, now, &expired);
	if (expired)
		WRITE_ONCE(ebc->idle_armed, false);
	if (due)
		rk3026_ebc_add_update(ebc, &upd);
	mutex_unlock(&ebc->image_lock);
	return due;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_idle_tick);

static int ebc_refresh_thread(void *data)
{
	struct rk3026_ebc *ebc = data;

	while (!kthread_should_stop()) {
		struct rk3026_ebc_last last;
		long timeout = MAX_SCHEDULE_TIMEOUT;
		unsigned long now = jiffies, deadline, rails_at, idle_at;
		bool rails_on, idle_armed;

		/*
		 * With the rails up, drop them at the deadline. Checked on every
		 * pass, not only when the wait times out: a steady stream of
		 * skipped updates keeps the thread busy and must not keep the
		 * rails up.
		 */
		rails_on = READ_ONCE(ebc->rails_on);
		rails_at = READ_ONCE(ebc->rails_off_at);
		if (rails_on && time_after_eq(now, rails_at)) {
			ebc_idle_rails_off(ebc);
			continue;
		}
		/*
		 * The idle cleanup, at its deadline: the look disarms it, and
		 * queues it if it is due, which the wait below then takes.
		 */
		idle_armed = ebc_idle_at(ebc, &idle_at);
		if (idle_armed && time_after_eq(now, idle_at)) {
			rk3026_ebc_idle_tick(ebc, now);
			continue;
		}
		if (rk3026_ebc_thread_deadline(rails_on, rails_at, idle_armed,
					       idle_at, &deadline))
			timeout = (long)(deadline - now);
		/*
		 * The wait cannot take @image_lock; the thread asks between
		 * parts, where rk3026_ebc_work_pending() means a part to take.
		 * Both deadlines are ahead here, so @timeout is positive; a
		 * plane update that arms or moves the idle one wakes the wait
		 * (rk3026_ebc_add_update()), and the next pass takes it.
		 */
		wait_event_interruptible_timeout(ebc->update_wq,
						 rk3026_ebc_work_pending(&ebc->pending) ||
						 kthread_should_stop(),
						 timeout);
		if (kthread_should_stop())
			break;

		rk3026_ebc_do_refresh(ebc, &last);
	}

	mutex_lock(&ebc->rails_lock);
	ebc_rails_off(ebc);
	mutex_unlock(&ebc->rails_lock);
	return 0;
}

/**
 * rk3026_ebc_wait_seq - block until a sent update has finished
 * @ebc: controller
 * @seq: the seq rk3026_ebc_add_update() gave the update
 * @timeout_ms: the longest wait
 * @interruptible: a signal ends the wait
 *
 * The update has finished once @pending.done_seq reached @seq, whatever
 * became of its part: driven, skipped, dropped with the CRTC off, or carried
 * into a later scan. Every part taken before it has finished too, and the
 * seqs cover each part from its take to rk3026_ebc_scan_finish(), waveform
 * included: @done_seq moves only in the @image_lock section after the scan.
 *
 * ⚠️ Nothing pending is NOT finished: the thread takes a part out of
 * @pending before it drives it, and in that window nothing is pending while a
 * full waveform is about to run -- so a wait on that returned at once and the
 * first blit of a boot raced the boot RESET. Nor is @image_lock a
 * substitute: the thread does not hold it across a waveform.
 *
 * Returns 0, -ETIMEDOUT after @timeout_ms, or -ERESTARTSYS if @interruptible
 * and a signal arrived.
 */
VISIBLE_IF_KUNIT int rk3026_ebc_wait_seq(struct rk3026_ebc *ebc, u32 seq,
					 unsigned int timeout_ms,
					 bool interruptible)
{
	const struct rk3026_ebc_pending *p = &ebc->pending;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	if (interruptible)
		ret = wait_event_interruptible_timeout(ebc->marker_wq,
						       rk3026_ebc_seq_finished(p, seq),
						       timeout);
	else
		ret = wait_event_timeout(ebc->marker_wq,
					 rk3026_ebc_seq_finished(p, seq),
					 timeout);
	if (ret < 0)
		return ret;
	return ret ? 0 : -ETIMEDOUT;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_wait_seq);

/*
 * For CRTC enable and disable: bounded, so a thread stuck elsewhere cannot
 * hang blank or shutdown.
 */
static void ebc_wait_finished(struct rk3026_ebc *ebc, u32 seq)
{
	unsigned int ms = rk3026_ebc_wait_timeout_ms(ebc);

	if (rk3026_ebc_wait_seq(ebc, seq, ms, false))
		dev_warn(ebc->dev, "refresh thread busy for %u ms, not waiting\n",
			 ms);
}

/**
 * rk3026_ebc_add_update - hand a panel update to the refresh thread
 * @ebc: controller
 * @upd: the update. Its clip is the damaged region: the scan drives the
 *	 whole panel, but moves only the pixels inside it
 *	 (rk3026_ebc_build_snapshot()). Its marker, if not 0, is recorded with
 *	 the update's seq for rk3026_ebc_wait_marker().
 *
 * Called under @image_lock, in the section that wrote the update's pixels
 * into @next_shadow, so no snapshot sees the pixels without their update.
 *
 * The update joins the pending set the next scan takes
 * (rk3026_ebc_pending_add()): one bounding box and the strongest mode of
 * everything sent since the last take, behind a pending RESET if there is
 * one. Nothing is overwritten and nothing overflows.
 *
 * A RESET ends white everywhere whatever the images hold, so it fills
 * @next_shadow white: the target follows the glass, and pixels sent before
 * it are not drawn again by a later box that covers them. The updates
 * still pending in front of the RESET keep their pixels all the same:
 * @next_shadow and @pre_shadow swap before the fill, and @pending.pre is
 * drawn from @pre_shadow (rk3026_ebc_do_refresh()). Otherwise its part
 * would lay white over the glass in its box, or be skipped as identical
 * on a white glass, and its updates would finish without being drawn. A
 * second RESET finds the split made: @pending.pre is still the part
 * before the first.
 *
 * An mxcfb send and a RESET set @plane_resync: plane updates blit only
 * their damage, and the target now holds pixels outside it that the
 * plane's framebuffer may not. The idle cleanup writes no pixel, so it
 * does not.
 *
 * Returns the update's seq, for rk3026_ebc_wait_seq().
 */
u32 rk3026_ebc_add_update(struct rk3026_ebc *ebc,
			  const struct rk3026_ebc_update *upd)
{
	bool merged;
	u32 seq;

	lockdep_assert_held(&ebc->image_lock);

	if (upd->mode == EBC_MODE_RESET) {
		if (ebc->pending.pre.merged && !ebc->pre_split) {
			swap(ebc->next_shadow, ebc->pre_shadow);
			ebc->pre_split = true;
		}
		memset(ebc->next_shadow, 0xff, ebc->img_size);
	}
	if (upd->source != EBC_SOURCE_PLANE && upd->source != EBC_SOURCE_IDLE)
		ebc->plane_resync = true;
	merged = rk3026_ebc_pending_add(&ebc->pending, upd, &seq);
	rk3026_ebc_marker_seq_record(&ebc->markers, upd->marker, seq);
	ebc->stats.updates++;
	if (merged)
		ebc->stats.coalesced++;
	trace_rk3026_ebc_update_queue(upd->source, upd->mode, &upd->clip,
				      upd->marker, upd->temp, upd->temp_valid,
				      merged,
				      rk3026_ebc_pending_parts(&ebc->pending));

	wake_up(&ebc->update_wq);
	return seq;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_add_update);

/**
 * rk3026_ebc_wait_marker - block until a sent update has been driven out
 * @ebc: controller
 * @marker: the marker the update was sent with
 *
 * Waits for the newest update sent with @marker, until the part that took
 * it has finished, whatever became of it: driven, skipped, dropped with the
 * CRTC off, or carried into a later scan because the rails did not come up.
 * The marker is resolved to its seq once, here. Marker 0, a marker never
 * sent, and one overwritten by EBC_MARKER_RING_LEN newer markers return at
 * once.
 */
int rk3026_ebc_wait_marker(struct rk3026_ebc *ebc, u32 marker)
{
	bool known;
	u32 seq;

	mutex_lock(&ebc->image_lock);
	known = rk3026_ebc_marker_seq_lookup(&ebc->markers, marker, &seq);
	mutex_unlock(&ebc->image_lock);
	if (!known)
		return 0;

	return rk3026_ebc_wait_seq(ebc, seq, rk3026_ebc_wait_timeout_ms(ebc),
				   true);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_wait_marker);

/* --- DRM --- */

static const u32 ebc_plane_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const u64 ebc_plane_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static int ebc_plane_atomic_check(struct drm_plane *plane,
				  struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;

	if (!new_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, new_state->crtc);
	return drm_atomic_helper_check_plane_state(new_state, crtc_state,
						   DRM_PLANE_NO_SCALING,
						   DRM_PLANE_NO_SCALING,
						   false, false);
}

/**
 * rk3026_ebc_plane_box - the panel box a plane commit redraws
 * @old_state: the plane state before the commit
 * @new_state: the plane state of the commit
 * @width: panel width
 * @height: panel height
 * @box: returns the box
 *
 * The commit's damage merged into one rectangle
 * (drm_atomic_helper_damage_merged()): what fbcon drew, the rows an fb0 mmap
 * writer dirtied, a KMS client's FB_DAMAGE_CLIPS. With no damage clips, after
 * a modeset or with a new source rectangle, that is the whole plane. The
 * framebuffer has the panel's size (mode_config min = max) and the plane
 * covers the CRTC unscaled (ebc_plane_atomic_check()), so the source is the
 * whole framebuffer at 0,0 and the damage is in panel coordinates. The box
 * is widened to whole bytes and clamped to the panel, as the blit widens it,
 * so the queued box is the region blitted.
 *
 * Returns false if nothing is damaged: the commit changes no pixel.
 */
VISIBLE_IF_KUNIT bool rk3026_ebc_plane_box(const struct drm_plane_state *old_state,
					   const struct drm_plane_state *new_state,
					   unsigned int width, unsigned int height,
					   struct drm_rect *box)
{
	if (!drm_atomic_helper_damage_merged(old_state, new_state, box))
		return false;
	return rk3026_ebc_widen_clip(box, width, height);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_plane_box);

/*
 * A plane update: the box of the framebuffer into @next_shadow and to the
 * thread in the default mode, as an auto update (DU if every changed pixel
 * becomes black or white, else PART; see EBC_MODE_DEFAULT), unless an mxcfb
 * client drives the panel. Such a client annotates its own updates with a
 * waveform mode, so its pixels are left alone rather than refreshed twice.
 *
 * @box is the damage (rk3026_ebc_plane_box()), inside the panel. After
 * another source wrote @next_shadow (@plane_resync) the whole panel is
 * blitted and queued instead: the framebuffer may differ from the target
 * anywhere.
 *
 * @mxcfb_active is checked in the @image_lock section of the blit, where a
 * send sets it. Checked before the lock, a plane update could take the lock
 * after a send that claimed the panel, overwrite the send's pixels in
 * @next_shadow and queue a refresh with it.
 *
 * A queued update arms the idle cleanup and stamps its time, in the same
 * section (rk3026_ebc_idle_tick()); the wake of rk3026_ebc_add_update()
 * makes the thread take the new deadline. The end of its scan stamps it
 * again (rk3026_ebc_do_refresh()).
 *
 * Returns true if the update was queued.
 */
VISIBLE_IF_KUNIT bool rk3026_ebc_plane_update(struct rk3026_ebc *ebc,
					      const void *src,
					      unsigned int src_pitch,
					      const struct drm_rect *box)
{
	struct rk3026_ebc_update upd = {
		.clip		= *box,
		.mode		= EBC_MODE_DEFAULT,
		.auto_mode	= true,
		.source		= EBC_SOURCE_PLANE,
	};
	bool queue;

	mutex_lock(&ebc->image_lock);
	queue = !ebc->mxcfb_active;
	if (queue) {
		if (ebc->plane_resync)
			upd.clip = DRM_RECT_INIT(0, 0, ebc->width, ebc->height);
		ebc->plane_resync = false;
		upd.blit_us = rk3026_ebc_blit_xrgb8888(ebc, src, src_pitch,
						       &upd.clip);
		WRITE_ONCE(ebc->idle_last_plane, jiffies);
		WRITE_ONCE(ebc->idle_armed, true);
		rk3026_ebc_add_update(ebc, &upd);
	}
	mutex_unlock(&ebc->image_lock);
	return queue;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_plane_update);

static void ebc_plane_atomic_update(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old_state =
		drm_atomic_get_old_plane_state(state, plane);
	struct drm_shadow_plane_state *shadow =
		to_drm_shadow_plane_state(new_state);
	struct rk3026_ebc *ebc = drm_to_ebc(plane->dev);
	struct drm_rect box;
	int idx;

	if (!new_state->fb || !new_state->visible)
		return;

	/*
	 * Only the damage is converted and queued. The box bounds the CPU
	 * work and nothing else: the EBC still scans the whole panel, and the
	 * snapshot lays the box over the prev image, which holds what the
	 * glass shows (rk3026_ebc_scan_finish()), so the pixels outside it are
	 * driven from their old level to the same level in FULL, and left
	 * alone in DU and PART. The waveform is chosen as for a whole-panel
	 * update: DU when every changed pixel ends black or white, else PART,
	 * and FULL for a cleanup of the DU budget (rk3026_ebc_resolve_mode()).
	 */
	if (!rk3026_ebc_plane_box(old_state, new_state, ebc->width,
				  ebc->height, &box))
		return;
	if (!drm_dev_enter(plane->dev, &idx))
		return;

	rk3026_ebc_plane_update(ebc, shadow->data[0].vaddr,
				new_state->fb->pitches[0], &box);

	drm_dev_exit(idx);
}

static const struct drm_plane_funcs ebc_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

static const struct drm_plane_helper_funcs ebc_plane_helper_funcs = {
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
	.atomic_check		= ebc_plane_atomic_check,
	.atomic_update		= ebc_plane_atomic_update,
};

static void ebc_crtc_atomic_enable(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	struct rk3026_ebc *ebc = drm_to_ebc(crtc->dev);
	struct rk3026_ebc_update upd = {
		.clip	= DRM_RECT_INIT(0, 0, ebc->width, ebc->height),
		.mode	= EBC_MODE_RESET,
		.source	= EBC_SOURCE_ENABLE,
	};
	u32 seq;
	int ret;

	/*
	 * Order matters and it is the datasheet's, not a preference. V3P3 is the
	 * panel's logic rail. The high-voltage rails come next; the PMIC sequences
	 * them internally per UPSEQ once ENABLE is written. VCOM goes LAST because
	 * its enable is internally gated on the positive boost (DCDC1) already
	 * being in regulation (SLVSAQ8G: "Positive boost (DCDC1) must be in
	 * regulation before VCOM can be enabled").
	 */
	if (ebc->v3p3_supply && !ebc->v3p3_on) {
		ret = regulator_enable(ebc->v3p3_supply);
		if (ret) {
			dev_err(ebc->dev, "failed to enable v3p3: %d\n", ret);
			return;
		}
		ebc->v3p3_on = true;
	}

	/*
	 * The high-voltage rails and VCOM are not switched here: the refresh
	 * thread brings them up for each waveform, the boot RESET below
	 * included, and drops them once nothing is pending (rails_off_ms).
	 * V3P3, the logic rail, stays with the CRTC.
	 *
	 * Drive the panel to a known white before anything is composed on it.
	 *
	 * ⚠️ The RESET's white fill of @next_shadow (rk3026_ebc_add_update())
	 * MUST NOT race a client blit into the same buffer: the enabling commit
	 * also carries the first plane update, and the default commit_tail runs
	 * the planes FIRST, so the blit landed and was then overwritten white
	 * here. The first draw of every boot was silently lost.
	 *
	 * Three parts, and all are needed:
	 *
	 *  - the commit order is reversed (ebc_mode_config_helper_funcs, the _rpm
	 *    tail), so the CRTC enables before the planes commit;
	 *  - @crtc_on, the white fill and the RESET are one @image_lock section,
	 *    so a sender that sees the CRTC on blits after the fill and queues
	 *    behind the RESET; the RESET also drops a carry left from before
	 *    the blank;
	 *  - the RESET is then driven to completion here, so by the time the plane
	 *    blits, prev is white and the client's own update is the next thing
	 *    the thread sees. The wait is for the RESET's own seq, so sends made
	 *    meanwhile do not extend it.
	 *
	 * Waiting costs the enabling commit one full RESET (~8 s at 12 MHz), which
	 * is time the RESET takes regardless; it only moves who blocks for it.
	 */
	mutex_lock(&ebc->image_lock);
	ebc->crtc_on = true;
	seq = rk3026_ebc_add_update(ebc, &upd);
	mutex_unlock(&ebc->image_lock);
	ebc_wait_finished(ebc, seq);
}

static void ebc_crtc_atomic_disable(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct rk3026_ebc *ebc = drm_to_ebc(crtc->dev);
	u32 seq;

	/*
	 * No new waveform from here on: the thread drops what it takes while
	 * the CRTC is off, and mxcfb sends are refused. Let the part in flight
	 * and those pending finish, then drop the rails now rather than after
	 * the idle delay: VCOM first, then the high-voltage rails, then logic.
	 */
	mutex_lock(&ebc->image_lock);
	ebc->crtc_on = false;
	seq = ebc->pending.queued_seq;
	mutex_unlock(&ebc->image_lock);
	/*
	 * If the thread does not come back, drop the rails under it anyway:
	 * rk3026_ebc_drive() checks them before every chunk and abandons the
	 * waveform, leaving prev_unknown set.
	 */
	ebc_wait_finished(ebc, seq);

	/* A failure here is retried by the thread's idle tick. */
	mutex_lock(&ebc->rails_lock);
	ebc_rails_off(ebc);
	mutex_unlock(&ebc->rails_lock);
	if (ebc->v3p3_on) {
		if (regulator_disable(ebc->v3p3_supply))
			dev_err(ebc->dev, "failed to disable v3p3\n");
		else
			ebc->v3p3_on = false;
	}
}

static const struct drm_crtc_funcs ebc_crtc_funcs = {
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= drm_atomic_helper_crtc_reset,
	.destroy		= drm_crtc_cleanup,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_crtc_helper_funcs ebc_crtc_helper_funcs = {
	.atomic_enable		= ebc_crtc_atomic_enable,
	.atomic_disable		= ebc_crtc_atomic_disable,
};

static const struct drm_encoder_funcs ebc_encoder_funcs = {
	.destroy		= drm_encoder_cleanup,
};

static int ebc_connector_get_modes(struct drm_connector *connector)
{
	struct rk3026_ebc *ebc = drm_to_ebc(connector->dev);
	struct drm_display_mode *mode;

	mode = drm_mode_create(connector->dev);
	if (!mode)
		return 0;

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	mode->clock = ebc_pixel_rate(ebc) / 1000;
	mode->hdisplay = ebc->width;
	mode->hsync_start = ebc->width;
	mode->hsync_end = ebc->width;
	mode->htotal = ebc->width;
	mode->vdisplay = ebc->height;
	mode->vsync_start = ebc->height;
	mode->vsync_end = ebc->height;
	mode->vtotal = ebc->height;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_connector_funcs ebc_connector_funcs = {
	.reset			= drm_atomic_helper_connector_reset,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= drm_connector_cleanup,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs ebc_connector_helper_funcs = {
	.get_modes		= ebc_connector_get_modes,
};

static const struct drm_mode_config_funcs ebc_mode_config_funcs = {
	.fb_create		= drm_gem_fb_create_with_dirty,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

/*
 * CRTC BEFORE PLANES. The default tail commits planes first, which on the
 * enabling commit means the client's blit lands in next_shadow and is then
 * filled white by the RESET of ebc_crtc_atomic_enable().
 *
 * The _rpm tail differs from the default only in that order, and this driver
 * has no runtime-PM stake in it -- it is being used for the ordering alone.
 */
static const struct drm_mode_config_helper_funcs ebc_mode_config_helper_funcs = {
	.atomic_commit_tail	= drm_atomic_helper_commit_tail_rpm,
};

DEFINE_DRM_GEM_FOPS(ebc_fops);

#ifdef CONFIG_DRM_FBDEV_EMULATION
static int ebc_fbdev_probe(struct drm_fb_helper *helper,
			   struct drm_fb_helper_surface_size *sizes)
{
	int ret = drm_fbdev_shmem_driver_fbdev_probe(helper, sizes);

	if (!ret)
		rk3026_ebc_mxcfb_install(drm_to_ebc(helper->dev), helper->info);
	return ret;
}
#endif

static const struct drm_driver ebc_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &ebc_fops,
	.name			= DRIVER_NAME,
	.desc			= "Rockchip RK3026 EBC",
	.major			= 1,
	.minor			= 0,
	DRM_GEM_SHMEM_DRIVER_OPS,
#ifdef CONFIG_DRM_FBDEV_EMULATION
	.fbdev_probe		= ebc_fbdev_probe,
#endif
};

/* --- bring-up --- */

static void ebc_kvfree(struct drm_device *drm, void *ptr)
{
	kvfree(ptr);
}

/*
 * One cached image, white like the panel after power-up. kvmalloc: an image
 * is more than kmalloc should be asked for (758 KiB, an order-8 allocation,
 * on a 1448x1072 panel). Released with the drm device: debugfs and the
 * mxcfb shim read the images through it.
 */
static int ebc_alloc_shadow(struct rk3026_ebc *ebc, u8 **img)
{
	*img = kvmalloc(ebc->img_size, GFP_KERNEL);
	if (!*img)
		return -ENOMEM;
	memset(*img, 0xff, ebc->img_size);
	return drmm_add_action_or_reset(&ebc->drm, ebc_kvfree, *img);
}

/*
 * ONE allocation, the two frames adjacent -- prev first, next at +img_size.
 *
 * This mirrors the vendor exactly, and the difference was measured rather than
 * guessed. Live vendor registers during a working refresh (2026-08-05):
 *
 *	MST0 = 0x7c800000   MST1 = 0x7c85ec00   delta 0x5ec00 == img_size
 *
 * Ours, from two separate dmam_alloc_attrs() calls:
 *
 *	MST0 = 0x7f600000   MST1 = 0x7f680000   delta 0x80000, with a 0x14400 gap
 *
 * By the time this was written that was the LAST known difference between the
 * vendor's setup and ours. Everything else had been checked and matched: all
 * 21 registers including EPD_CTRL/DSP_CTRL/DSP_START/INT, the LUT SRAM word
 * count for the mode, the rails (ENABLE 0x3f, PG 0xfa, VCOM1 0xb4, vcom_ctl
 * high, LDO4 on), the clocks, the MST order, the 4bpp packing, the geometry, and
 * -- verified with devmem at the physical addresses -- the image bytes
 * themselves, correct across the whole frame with the right 32-byte xbands
 * period on lines 0, 1 and 100.
 *
 * And the failure shape fits: uniform content drove the whole panel while
 * structured content appeared only in a strip at one edge, i.e. the panel was
 * inert wherever the controller saw prev == next. That is what a second fetch
 * landing somewhere other than where we think would produce, while leaving every
 * CPU-side check passing.
 *
 * ⚠️ HYPOTHESIS, not established: nothing in the audited vendor code says the
 * controller requires contiguity or derives the second address itself. What is
 * established is only that the vendor's two buffers ARE contiguous and ours were
 * not. If this does not fix it, the contiguity idea is dead and should be
 * recorded as such rather than left hanging -- but the layout is worth keeping
 * regardless, because matching a known-good configuration costs nothing.
 */
static int ebc_setup_buffers(struct rk3026_ebc *ebc)
{
	size_t pool;
	int ret;

	/* One size for every image, the scanned ones and the shadows alike. */
	ebc->shadows.pitch = ebc->width / 2;
	ebc->shadows.height = ebc->height;
	ebc->img_size = (size_t)ebc->shadows.pitch * ebc->shadows.height;
	pool = ebc->img_size * 2;

	ebc->prev_virt = dmam_alloc_attrs(ebc->dev, pool, &ebc->prev_phys,
					  GFP_KERNEL, DMA_ATTR_WRITE_COMBINE);
	if (!ebc->prev_virt)
		return -ENOMEM;

	ebc->next_virt = ebc->prev_virt + ebc->img_size;
	ebc->next_phys = ebc->prev_phys + ebc->img_size;

	dev_info(ebc->dev, "images: prev %pad next %pad (delta %#zx, one pool of %#zx)\n",
		 &ebc->prev_phys, &ebc->next_phys, (size_t)ebc->img_size, pool);

	/* The panel is assumed white after power-up. */
	memset(ebc->prev_virt, 0xff, pool);

	ret = ebc_alloc_shadow(ebc, &ebc->next_shadow);
	if (ret)
		return ret;
	ret = ebc_alloc_shadow(ebc, &ebc->pre_shadow);
	if (ret)
		return ret;
	ret = ebc_alloc_shadow(ebc, &ebc->shadows.prev);
	if (ret)
		return ret;
	return ebc_alloc_shadow(ebc, &ebc->shadows.scan);
}

/*
 * The controller's register state outside a scan: timings, window, border,
 * EPD_CTRL and the idle DSP_CTRL. At probe, and again after a block reset.
 */
static void ebc_program_regs(struct rk3026_ebc *ebc)
{
	ebc_set_timing(ebc);
	ebc_set_window(ebc);
	ebc_set_border(ebc);
	ebc_set_epd_ctrl(ebc);

	/* Idle DSP_CTRL: the full-refresh class value. */
	ebc->dsp_ctrl = DSP_CTRL_FIXED | DSP_DISPLAY_MODE;
	ebc_write(ebc, EBC_DSP_CTRL, ebc->dsp_ctrl);
	ebc_config_done(ebc);
}

static int ebc_hw_init(struct rk3026_ebc *ebc)
{
	int ret;

	ret = clk_prepare_enable(ebc->hclk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(ebc->aclk);
	if (ret)
		goto err_hclk;

	/*
	 * mclk is ACLK_EBC, the block's own AXI master port -- the clock the
	 * controller fetches the image over. Without it the timing generator
	 * still runs off dclk, so frames complete and the frame-end IRQ arrives
	 * on schedule while no pixel data is ever read: exactly the signature of
	 * a scan that reports success and leaves the glass untouched.
	 *
	 * ⚠️ This was `if (0 && ebc->mclk)` from 2026-08-03 to 2026-08-05. It was
	 * disabled because the two cards that enabled it were also the two whose
	 * U-Boot console hung at misc_init_r on the FOLLOWING boot -- but that
	 * was never shown to be caused by this, and the same session's dead
	 * console was later traced to a flat battery and a hub (docs/INDEX.md).
	 * Three separate reviews since concluded the disable was not addressing a
	 * real cause; none of them re-enabled it. Being probably-not-the-hang
	 * does not make skipping the AXI clock correct.
	 *
	 * The real hazard the old comment named IS real and is fixed in the same
	 * change: clk-rk3026.c modelled aclk_vio1_niu and aclk_ebc as two GATE
	 * objects on the SAME bit (CLKGATE_CON9 b10), so a disable from either
	 * closes the bit under the other -- and ebc_hw_fini() disables mclk. It
	 * is now one clock: ACLK_EBC is a FACTOR of aclk_vio1_niu, no second gate.
	 */
	if (ebc->mclk) {
		ret = clk_prepare_enable(ebc->mclk);
		if (ret)
			goto err_aclk;
	}

	/*
	 * dclk boots muxed to CPLL, which sits in slow mode at 24 MHz -- 12 MHz
	 * after the divider, against the panel's 20 MHz source-clock spec, and a
	 * 54-frame scan then takes 4260 ms instead of 790.
	 *
	 * ⚠️ Order matters and cost a session. clk_set_rate() alone returns
	 * SUCCESS and leaves the hardware at 12 MHz: CCF reparents, writes the
	 * mux, then writes the divider -- and CLKSEL_CON23 reads back 0xff000100,
	 * which is exactly a hiword-style divider write applied as a plain 32-bit
	 * store, clearing the mux bit the same call had just set. clk_get_rate()
	 * then reports the rate CCF believes, not the one the panel gets. So the
	 * parent is asserted LAST, and the register is logged rather than trusted.
	 */
	/*
	 * PARENT FIRST, THEN RATE. This order is load-bearing and it is the
	 * REVERSE of what this function did until 2026-08-06.
	 *
	 * clk_set_rate() is free to satisfy a request from either parent, so
	 * asking for 12 MHz can be answered with CPLL (slow mode, 24 MHz) and a
	 * divider of 1. Reparenting afterwards to GPLL keeps that divider and
	 * 132/2 lands on 66 MHz -- the rate is destroyed by the very call that
	 * follows it, silently, with both calls returning success. That is why a
	 * cold boot still measured 940 ms for 64 frames when the driver had asked
	 * for 12 MHz.
	 *
	 * Asserting the parent last was CORRECT while CLKSEL_CON23 was declared
	 * with the HIWORD_MASK flags, because the divider write then landed as
	 * 0xff000100 and cleared the mux bit. Those flags are gone from
	 * clk-rk3026.c, so the workaround outlived the bug it was working around.
	 *
	 * ⚠️ Verify against the FRAME TIME in the scan log, never clk_get_rate():
	 * 38 frames take ~3000 ms at 12 MHz and ~560 ms at 66.
	 */
	if (ebc->dclk_parent) {
		ret = clk_set_parent(ebc->dclk, ebc->dclk_parent);
		if (ret)
			goto err_mclk;
	}

	/*
	 * The rate comes from the DEVICE TREE, not a compiled-in constant: it is
	 * a property of this panel and this board. of_get_display_timing() has
	 * already parsed `panel-timing { clock-frequency }` into timing.pixelclock,
	 * and the driver used to ignore it in favour of EBC_PIXEL_RATE.
	 *
	 * dclk runs at four times the source clock because the 8-bit source bus
	 * carries four 2-bit pixels per clock.
	 */
	ret = clk_set_rate(ebc->dclk, ebc_dclk_rate(ebc));
	if (ret)
		goto err_mclk;
	ret = clk_prepare_enable(ebc->dclk);
	if (ret)
		goto err_mclk;

	dev_info(ebc->dev, "dclk %lu Hz (CCF's view; check a scan's frame time)\n",
		 clk_get_rate(ebc->dclk));

	/*
	 * Route the shared display pads to EBC, and put the LVDS block in the
	 * state the panel needs. All three bits are written, not just the pad
	 * select -- see the GRF_EBC_SEL_* comment in rk3026_ebc.h.
	 *
	 * This driver wrote bit 0 alone until 2026-08-06 and appeared to work,
	 * because on those boots the register already held 0x301. On a boot from
	 * a freshly written card it read 0x101 and drew nothing. ⚠️ What put the
	 * 0x300 there is NOT established -- not this driver, not U-Boot.
	 */
	if (ebc->grf)
		regmap_write(ebc->grf, GRF_EBC_SEL_REG,
			     (GRF_EBC_SEL_MASK << 16) | GRF_EBC_SEL_MASK);

	ebc_program_regs(ebc);
	return 0;

err_mclk:
	clk_disable_unprepare(ebc->mclk);
err_aclk:
	clk_disable_unprepare(ebc->aclk);
err_hclk:
	clk_disable_unprepare(ebc->hclk);
	return ret;
}

static void ebc_hw_fini(void *data)
{
	struct rk3026_ebc *ebc = data;

	clk_disable_unprepare(ebc->dclk);
	clk_disable_unprepare(ebc->mclk);
	clk_disable_unprepare(ebc->aclk);
	clk_disable_unprepare(ebc->hclk);
}

static void ebc_stop_thread(void *data)
{
	struct rk3026_ebc *ebc = data;

	kthread_stop(ebc->refresh_thread);
}

static void ebc_debugfs_fini(void *data)
{
	rk3026_ebc_debugfs_fini(data);
}

static void ebc_release_fw(void *data)
{
	release_firmware(data);
}

static int ebc_load_waveform(struct rk3026_ebc *ebc)
{
	struct ebc_waveform *wf;
	int ret;

	ret = request_firmware(&ebc->fw, EBC_WAVEFORM_FW, ebc->dev);
	if (ret) {
		dev_warn(ebc->dev, "waveform %s not found (%d)\n",
			 EBC_WAVEFORM_FW, ret);
		return 0;
	}
	ret = devm_add_action_or_reset(ebc->dev, ebc_release_fw,
				       (void *)ebc->fw);
	if (ret)
		return ret;

	wf = rk3026_ebc_waveform_parse(ebc->dev, ebc->fw->data, ebc->fw->size);
	if (IS_ERR(wf)) {
		dev_warn(ebc->dev, "waveform %s rejected (%ld)\n",
			 EBC_WAVEFORM_FW, PTR_ERR(wf));
		return 0;
	}
	ebc->waveform = wf;
	return 0;
}

static int ebc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rk3026_ebc *ebc;
	struct drm_device *drm;
	u32 rotation;
	int ret;

	ebc = devm_drm_dev_alloc(dev, &ebc_drm_driver, struct rk3026_ebc, drm);
	if (IS_ERR(ebc))
		return PTR_ERR(ebc);

	drm = &ebc->drm;
	ebc->dev = dev;
	platform_set_drvdata(pdev, ebc);

	ebc->epd_supply = devm_regulator_get(dev, "epd");
	if (IS_ERR(ebc->epd_supply))
		return dev_err_probe(dev, PTR_ERR(ebc->epd_supply),
				     "failed to get EPD rail supply\n");

	/*
	 * Optional so a device tree predating these properties still probes, but a
	 * board that omits them cannot image -- see the struct comment. V3P3 is
	 * enabled with the CRTC, VCOM by the refresh thread around each
	 * waveform (ebc_rails_on()), never here.
	 */
	ebc->v3p3_supply = devm_regulator_get_optional(dev, "v3p3");
	if (IS_ERR(ebc->v3p3_supply)) {
		if (PTR_ERR(ebc->v3p3_supply) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ebc->v3p3_supply),
					     "failed to get v3p3 supply\n");
		ebc->v3p3_supply = NULL;
		dev_warn(dev, "no v3p3-supply; the panel's logic rail is unmanaged\n");
	}
	ebc->vcom_supply = devm_regulator_get_optional(dev, "vcom");
	if (IS_ERR(ebc->vcom_supply)) {
		if (PTR_ERR(ebc->vcom_supply) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ebc->vcom_supply),
					     "failed to get vcom supply\n");
		ebc->vcom_supply = NULL;
		dev_warn(dev, "no vcom-supply; VCOM stays discharged and nothing will image\n");
	}

	init_completion(&ebc->frame_done);
	init_waitqueue_head(&ebc->update_wq);
	init_waitqueue_head(&ebc->marker_wq);
	ret = drmm_mutex_init(drm, &ebc->image_lock);
	if (ret)
		return ret;
	ret = drmm_mutex_init(drm, &ebc->rails_lock);
	if (ret)
		return ret;

	ebc->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ebc->regs))
		return PTR_ERR(ebc->regs);

	ebc->hclk = devm_clk_get(dev, "hclk");
	if (IS_ERR(ebc->hclk))
		return dev_err_probe(dev, PTR_ERR(ebc->hclk), "no hclk\n");
	ebc->dclk = devm_clk_get(dev, "dclk");
	if (IS_ERR(ebc->dclk))
		return dev_err_probe(dev, PTR_ERR(ebc->dclk), "no dclk\n");
	ebc->aclk = devm_clk_get(dev, "aclk");
	if (IS_ERR(ebc->aclk))
		return dev_err_probe(dev, PTR_ERR(ebc->aclk), "no aclk\n");
	/* Both optional so an older device tree still probes. */
	ebc->mclk = devm_clk_get_optional(dev, "mclk");
	if (IS_ERR(ebc->mclk))
		return dev_err_probe(dev, PTR_ERR(ebc->mclk), "bad mclk\n");
	ebc->dclk_parent = devm_clk_get_optional(dev, "dclk-parent");
	if (IS_ERR(ebc->dclk_parent))
		return dev_err_probe(dev, PTR_ERR(ebc->dclk_parent), "bad dclk-parent\n");

	/* Used only to stop a controller that stopped answering. */
	ebc->rst = devm_reset_control_array_get_optional_exclusive(dev);
	if (IS_ERR(ebc->rst))
		return dev_err_probe(dev, PTR_ERR(ebc->rst), "bad resets\n");

	ebc->grf = syscon_regmap_lookup_by_phandle(dev->of_node, "rockchip,grf");
	if (IS_ERR(ebc->grf)) {
		dev_warn(dev, "no rockchip,grf; skipping pad mux\n");
		ebc->grf = NULL;
	}

	ret = of_get_display_timing(dev->of_node, "panel-timing", &ebc->timing);
	if (ret)
		return dev_err_probe(dev, ret, "missing panel-timing\n");
	ebc->width = ebc->timing.hactive.typ;
	ebc->height = ebc->timing.vactive.typ;
	if (ebc->width % 8 || ebc->height % 2)
		return dev_err_probe(dev, -EINVAL, "unsupported panel size\n");
	/*
	 * One panel's worth of DU-changed pixels spends the budget; a part
	 * that changes half the panel takes the cleanup FULL then, and any
	 * part does at four panels' worth (rk3026_ebc_du_cleanup()).
	 */
	ebc->du_cleanup_area = ebc->width * ebc->height;
	ebc->du_cleanup_big_area = ebc->du_cleanup_area / 2;
	ebc->du_cleanup_ceiling = 4 * ebc->du_cleanup_area;
	/* And it is cleaned up once the plane is idle this long. */
	ebc->du_cleanup_idle_ms = EBC_DU_CLEANUP_IDLE_MS;
	/* No A2 for plane parts until its ghosting is measured. */
	ebc->a2_max_area = 0;

	/* E Ink GDCK_STA / LGONL; no display-timing binding covers them. */
	if (of_property_read_u32(dev->of_node, "rockchip,gdck-sta",
				 &ebc->gdck_sta))
		return dev_err_probe(dev, -EINVAL, "missing rockchip,gdck-sta\n");
	if (of_property_read_u32(dev->of_node, "rockchip,lgonl", &ebc->lgonl))
		return dev_err_probe(dev, -EINVAL, "missing rockchip,lgonl\n");

	/*
	 * Panel mounting angle, counter-clockwise, as the common "rotation"
	 * property. The panel table in the vendor firmware carries 270.
	 */
	ebc->rotation = DRM_MODE_PANEL_ORIENTATION_UNKNOWN;
	if (!of_property_read_u32(dev->of_node, "rotation", &rotation)) {
		switch (rotation) {
		case 0:
			ebc->rotation = DRM_MODE_PANEL_ORIENTATION_NORMAL;
			break;
		case 90:
			ebc->rotation = DRM_MODE_PANEL_ORIENTATION_RIGHT_UP;
			break;
		case 180:
			ebc->rotation = DRM_MODE_PANEL_ORIENTATION_BOTTOM_UP;
			break;
		case 270:
			ebc->rotation = DRM_MODE_PANEL_ORIENTATION_LEFT_UP;
			break;
		default:
			dev_warn(dev, "ignoring rotation %u\n", rotation);
			break;
		}
	}

	/*
	 * Panel thermometer, in the EPD PMIC. The waveform LUT is selected by
	 * temperature, so probing before the zone exists does not degrade this
	 * driver, it drives every update from the wrong bucket for the life of
	 * the boot -- silently, behind one dev_warn. That was the old
	 * behaviour and it is why a wrong LUT could never be seen in a log.
	 *
	 * Ordering against the sensor is DECLARED, not hoped for, and it is
	 * declared in two places because one is not expressible:
	 *
	 * - Against the tps65185 device: `epd-supply` points at a regulator
	 *   whose parent is the PMIC, and fw_devlink turns that phandle into a
	 *   real probe-order edge. So by the time we are here the PMIC has
	 *   probed. That link is what devm_regulator_get() above establishes.
	 *
	 * - Against the ZONE: not expressible, and NOT expressible as a device
	 *   link either. A device link models a driver-bound supplier, and a
	 *   thermal zone device has no driver -- so the link never reaches
	 *   DL_STATE_CONSUMER_PROBE and device_links_driver_bound() splats
	 *   WARN_ON(link->status != DL_STATE_CONSUMER_PROBE) (core.c:1383) the
	 *   moment this driver binds. That warning was added here on
	 *   2026-08-03 and removed the same day; do not reintroduce it.
	 *
	 * So EPROBE_DEFER is not a lesser substitute for a declaration here,
	 * it is the mechanism: the ordering that CAN be declared already is,
	 * through the supply, and the zone is waited for.
	 */
	ebc->tzd = thermal_zone_get_zone_by_name("epd-thermal");
	if (IS_ERR(ebc->tzd))
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "epd-thermal zone not ready (%ld)\n",
				     PTR_ERR(ebc->tzd));

	ret = of_reserved_mem_device_init(dev);
	if (ret)
		dev_warn(dev, "no reserved memory region (%d)\n", ret);

	ret = ebc_setup_buffers(ebc);
	if (ret)
		return ret;

	ret = ebc_load_waveform(ebc);
	if (ret)
		return ret;

	ebc->irq = platform_get_irq(pdev, 0);
	if (ebc->irq < 0)
		return ebc->irq;

	ret = ebc_hw_init(ebc);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, ebc_hw_fini, ebc);
	if (ret)
		return ret;

	/*
	 * After the clocks and @int_reg are set up: the handler reads EBC_INT
	 * and acknowledges through @int_reg. Requested after ebc_hw_fini() is
	 * registered, so devres frees it before the clocks go down.
	 */
	ret = devm_request_irq(dev, ebc->irq, ebc_irq, 0, DRIVER_NAME, ebc);
	if (ret)
		return dev_err_probe(dev, ret, "request_irq\n");

	ebc->refresh_thread = kthread_run(ebc_refresh_thread, ebc, "ebc-refresh");
	if (IS_ERR(ebc->refresh_thread))
		return PTR_ERR(ebc->refresh_thread);
	ret = devm_add_action_or_reset(dev, ebc_stop_thread, ebc);
	if (ret)
		return ret;

	rk3026_ebc_debugfs_init(ebc);
	ret = devm_add_action_or_reset(dev, ebc_debugfs_fini, ebc);
	if (ret)
		return ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.funcs = &ebc_mode_config_funcs;
	drm->mode_config.helper_private = &ebc_mode_config_helper_funcs;
	drm->mode_config.min_width = ebc->width;
	drm->mode_config.max_width = ebc->width;
	drm->mode_config.min_height = ebc->height;
	drm->mode_config.max_height = ebc->height;

	ret = drm_universal_plane_init(drm, &ebc->plane, 1, &ebc_plane_funcs,
				       ebc_plane_formats,
				       ARRAY_SIZE(ebc_plane_formats),
				       ebc_plane_modifiers,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&ebc->plane, &ebc_plane_helper_funcs);
	/*
	 * ebc_plane_atomic_update() blits and queues only the damage. The fbdev
	 * emulation attaches its damage either way; the property lets KMS
	 * clients send theirs.
	 */
	drm_plane_enable_fb_damage_clips(&ebc->plane);

	ret = drm_crtc_init_with_planes(drm, &ebc->crtc, &ebc->plane, NULL,
					&ebc_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&ebc->crtc, &ebc_crtc_helper_funcs);

	ret = drm_encoder_init(drm, &ebc->encoder, &ebc_encoder_funcs,
			       DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ret;
	ebc->encoder.possible_crtcs = drm_crtc_mask(&ebc->crtc);

	ret = drm_connector_init(drm, &ebc->connector, &ebc_connector_funcs,
				 DRM_MODE_CONNECTOR_Unknown);
	if (ret)
		return ret;
	drm_connector_helper_add(&ebc->connector, &ebc_connector_helper_funcs);
	drm_connector_set_panel_orientation(&ebc->connector, ebc->rotation);
	drm_connector_attach_encoder(&ebc->connector, &ebc->encoder);

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_client_setup(drm, NULL);

	dev_info(dev, "blit: %s\n", rk3026_ebc_blit_impl());
	dev_info(dev, "RK3026 EBC %ux%u\n", ebc->width, ebc->height);
	return 0;
}

static void ebc_remove(struct platform_device *pdev)
{
	struct rk3026_ebc *ebc = platform_get_drvdata(pdev);

	drm_dev_unplug(&ebc->drm);
	drm_atomic_helper_shutdown(&ebc->drm);
}

static void ebc_shutdown(struct platform_device *pdev)
{
	struct rk3026_ebc *ebc = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&ebc->drm);

	/* Last chance for a rails-off that failed at CRTC disable. */
	mutex_lock(&ebc->rails_lock);
	ebc_rails_off(ebc);
	mutex_unlock(&ebc->rails_lock);
}

static const struct of_device_id ebc_of_match[] = {
	{ .compatible = "rockchip,rk3026-ebc" },
	{ }
};
MODULE_DEVICE_TABLE(of, ebc_of_match);

static struct platform_driver ebc_driver = {
	.probe	= ebc_probe,
	.remove	= ebc_remove,
	.shutdown = ebc_shutdown,
	.driver	= {
		.name		= DRIVER_NAME,
		.of_match_table	= ebc_of_match,
	},
};
module_platform_driver(ebc_driver);

MODULE_DESCRIPTION("Rockchip RK3026 EBC E-Ink DRM driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(EBC_WAVEFORM_FW);
