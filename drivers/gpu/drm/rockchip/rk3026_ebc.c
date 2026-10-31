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
#include <linux/dma-fence.h>
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
#include <linux/workqueue.h>

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
#include <drm/drm_vblank.h>

#include <video/of_display_timing.h>

#include <kunit/static_stub.h>
#include <kunit/visibility.h>

#include "rk3026_ebc.h"

#define CREATE_TRACE_POINTS
#include "rk3026_ebc_trace.h"

#define DRIVER_NAME		"rk3026-ebc"
#define EBC_WAVEFORM_FW		"rockchip/rk3026-c67ml.rkf"
/*
 * Per-chunk frame budget; a chunk is at most EBC_LUT_MAX_FRAMES (64) frames.
 *
 * Measured with dclk_ebc at 66 MHz: 54 frames take 790-800 ms, ~14.8 ms a
 * frame, so a full 64-frame chunk takes ~950 ms; 5 s is a little over five
 * times that.
 *
 * Do not tighten this toward the measured time. The timeout path abandons
 * the waveform partway (it stops the controller, see ebc_stop_scan(), and
 * drops the rails), and a panel left mid-transition keeps ghosting and
 * darkening that later updates do not remove: a 3000 ms budget against a
 * 3.1 s waveform did exactly that. A tight budget is not a harmless
 * mis-tuning here, it is cumulative damage. The timeout exists for a
 * controller that has stopped answering, nothing else.
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
 * refresh thread switches them on for a part it will drive, before its take
 * (rk3026_ebc_do_refresh()) or when the take's quiet gap opens for it
 * (rk3026_ebc_take_ready()), and off @rails_off_ms after the last
 * waveform ended (or after they came up, if none was driven; probe sets
 * EBC_RAILS_OFF_MS, debugfs rails_off_ms overrides it); while a gap is open
 * they are held past the deadline (rk3026_ebc_thread_step()), at most the
 * gap's cap. A short delay keeps a burst of updates (typing, a scrolling
 * console) from cycling the PMIC per update. Only a driven waveform moves
 * the deadline: an update that is skipped (identical image) or has no LUT
 * does not, so a stream of no-op commits cannot keep the rails up; nor do
 * the rails come up for a part the look finds will not be driven. Stock
 * powers off right after each update; leaving the rails and VCOM up between
 * updates holds a DC bias on the panel.
 *
 * This sets the deadline @rails_off_ms from now. Under @rails_lock or on
 * the thread.
 */
static void ebc_rails_rearm(struct rk3026_ebc *ebc)
{
	WRITE_ONCE(ebc->rails_off_at,
		   jiffies + msecs_to_jiffies(READ_ONCE(ebc->rails_off_ms)));
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
 * A grey plane part goes PART: on this panel PART draws a grey ramp from
 * white as FULL draws it, solid black included (camera run).
 */
#define EBC_MODE_DEFAULT	EBC_MODE_FULL

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
VISIBLE_IF_KUNIT unsigned int rk3026_ebc_part_max_ms(struct rk3026_ebc *ebc)
{
	u32 chunks = 2 * DIV_ROUND_UP(EBC_WAVEFORM_MAX_FRAMES,
				      EBC_LUT_MAX_FRAMES);

	return EBC_PART_SETUP_MS +
	       chunks * ebc_chunk_timeout_ms(ebc, EBC_LUT_MAX_FRAMES);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_part_max_ms);

/*
 * Longest a wait on one update blocks (rk3026_ebc_wait_seq():
 * MXCFB_WAIT_FOR_UPDATE_COMPLETE, CRTC enable and disable): EBC_WAIT_PARTS
 * parts at their longest, and the retries of its own part if that is
 * carried: tries until EBC_RETRY_SPAN_MS after its first carry, and the
 * one that gives it up may start just before then, so one part more and
 * the span (rk3026_ebc_retry_next()); and the quiet gap its own part may
 * wait in before the take, at most EBC_TAKE_CAP_MAX_MS: the parts ahead of
 * it are in flight already or exempt (in front of a RESET, or the RESET),
 * and a retry is exempt. 208 s with every chunk budget at its 5 s floor.
 * Past this the thread is stuck outside the EBC (an I2C transfer, say), and
 * the waiter goes on without it.
 */
VISIBLE_IF_KUNIT unsigned int rk3026_ebc_wait_timeout_ms(struct rk3026_ebc *ebc)
{
	return (EBC_WAIT_PARTS + 1) * rk3026_ebc_part_max_ms(ebc) +
	       EBC_RETRY_SPAN_MS +
	       EBC_TAKE_CAP_MAX_MS;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_wait_timeout_ms);

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
	 * WIN_VIR is the window's virtual width in pixels, the high half zero:
	 * the controller's line step. The vendor kernel programs 0x00000400 on
	 * a 1024 pixel panel, the width exactly.
	 */
	ebc_write(ebc, EBC_WIN_VIR, ebc->width);
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

	ebc_write(ebc, EBC_EPD_CTRL,
		  EPD_GDCK_STA(gdck) | EPD_LGONL(gdck + ebc->lgonl) |
		  EPD_SDCK(rk3026_ebc_sdck(ebc->width)) |
		  EPD_GATE_SCAN_DIR | EPD_SOURCE_SCAN_DIR);
}

static void ebc_set_dsp_ctrl(struct rk3026_ebc *ebc, u32 mode)
{
	ebc->dsp_ctrl = DSP_CTRL_FIXED | DSP_VCOM_MODE |
			rk3026_ebc_dsp_ctrl_mode(mode);
	ebc_write(ebc, EBC_DSP_CTRL, ebc->dsp_ctrl);
	ebc_config_done(ebc);
}

/*
 * MST0 holds prev, the image on the glass, and MST1 holds next, the one
 * being driven towards. The controller looks its LUT up by (prev, next):
 * with the two swapped a black frame renders light and a white one darker,
 * the transposed lookup that only ever drives toward white; with prev in
 * MST0 white brightens and black darkens, as measured on the panel. The
 * vendor kernel uses the same order: the buffer it releases after a scan,
 * the one that was on the glass, is the one it put in MST0.
 *
 * The MST order and the LUT index order are a pair: inverting one alone
 * inverts the transition lookup. Do not change one without testing the
 * direction on the glass.
 *
 * WIN_MST0/1 are shadowed: a write takes effect not on the scan that
 * follows it but on the one after, as the shadow becomes active on a latch
 * the scan itself performs. A change of the order tested with one scan
 * tests nothing: that scan still uses the previous pair.
 *
 * Both hold absolute physical addresses, biased by the window origin at two
 * pixels per byte. The row stride is the panel width: the images are packed
 * at width / 2 bytes per line (rk3026_ebc_blit_xrgb8888()), and WIN_VIR, from
 * which the controller takes its line step, is the width as well.
 */
static void ebc_set_image_addr(struct rk3026_ebc *ebc, u32 x, u32 y)
{
	u32 off = (x + y * ebc->width) / 2;

	ebc_write(ebc, EBC_WIN_MST0, lower_32_bits(ebc->prev_phys) + off);
	ebc_write(ebc, EBC_WIN_MST1, lower_32_bits(ebc->next_phys) + off);
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
	 * fetch it. Two distinct things need this:
	 *
	 *  - the IMAGE, in prev_virt/next_virt, which are DMA_ATTR_WRITE_COMBINE
	 *    (ebc_setup_buffers). Write-combining means the thread's copies into
	 *    them sit in a merge buffer with no ordering against the EBC's AXI
	 *    master reading the same memory.
	 *  - the LUT SRAM and the register group just above, written with
	 *    writel_relaxed() throughout, which by definition orders nothing.
	 *
	 * A missing drain would show first in content that varies within a line:
	 * write-combining coalesces stores, so uniform images and long runs of
	 * identical lines still draw correctly whichever stores landed late.
	 * The vendor kernel flushes the caches after every generated frame, on
	 * a pool it maps uncached.
	 */
	wmb();

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

/*
 * Panel temperature, in degrees Celsius. KUnit replaces it with a static
 * stub, so that the glue can read a temperature of its own.
 */
VISIBLE_IF_KUNIT int rk3026_ebc_read_temp(struct rk3026_ebc *ebc)
{
	int temp;

	KUNIT_STATIC_STUB_REDIRECT(rk3026_ebc_read_temp, ebc);

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
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_read_temp);

/*
 * The clock of the take's quiet gap: when an update was queued, when a part
 * finished, the refresh thread's looks (rk3026_ebc_take_ready()). KUnit
 * replaces it with a static stub, so that the gap can be run on a fake
 * clock.
 */
VISIBLE_IF_KUNIT ktime_t rk3026_ebc_clock(struct rk3026_ebc *ebc)
{
	KUNIT_STATIC_STUB_REDIRECT(rk3026_ebc_clock, ebc);
	return ktime_get();
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_clock);

/* Microseconds since @start, as a record keeps them (rk3026_ebc_us_between()). */
static u32 ebc_us_since(ktime_t start)
{
	return rk3026_ebc_us_between(start, ktime_get());
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

/*
 * The time from the take of @last's part to now (rk3026_ebc_clock()), for
 * the scan_start event of each of its waveforms: the snapshot, and a repair
 * RESET in front of the part.
 */
VISIBLE_IF_KUNIT u32 rk3026_ebc_setup_us(struct rk3026_ebc *ebc,
					 const struct rk3026_ebc_last *last)
{
	return rk3026_ebc_us_between(last->taken, rk3026_ebc_clock(ebc));
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_setup_us);

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
 * And only a drive that makes the glass known clears a flag that was set
 * when it began (rk3026_ebc_drive_knows_glass(): a RESET, or a FULL): a PART
 * or DU scan moves only the pixels whose prev, not evidence then, differs,
 * so the glass is no better known after it than before.
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
	bool unknown = READ_ONCE(ebc->shadows.prev_unknown);
	u32 done = 0, chunk = 0;
	ktime_t start = ktime_get();
	int ret = 0;

	KUNIT_STATIC_STUB_REDIRECT(rk3026_ebc_drive, ebc, lut, frames, mode,
				   lut_mode, last);

	/* A repair is the first waveform of an update that has repaired set. */
	trace_rk3026_ebc_scan_start(last, mode, lut_mode, frames,
				    last->repaired && !last->scans,
				    rk3026_ebc_setup_us(ebc, last));

	WRITE_ONCE(ebc->shadows.prev_unknown, true);
	last->scans++;
	last->lut_mode = lut_mode;
	last->frames = frames;

	ebc_set_dsp_ctrl(ebc, mode);
	ebc_set_image_addr(ebc, 0, 0);

	while (done < frames) {
		u32 n = min_t(u32, frames - done, EBC_LUT_MAX_FRAMES);
		unsigned int budget = ebc_chunk_timeout_ms(ebc, n);
		bool ok;

		if (!ebc_rails_up(ebc)) {
			dev_warn_ratelimited(ebc->dev,
					     "rails down before chunk %u, waveform abandoned\n",
					     chunk);
			last->outcome = EBC_OUTCOME_ABANDONED;
			ret = -EIO;
			goto out;
		}

		/* debugfs debug_timeout: this chunk's wait expires for real. */
		if (atomic_dec_if_positive(&ebc->debug_timeout) >= 0)
			budget = 1;

		reinit_completion(&ebc->frame_done);
		ebc_frame_kick(ebc, lut, chunk, n, done + n >= frames);
		ok = wait_for_completion_timeout(&ebc->frame_done,
						msecs_to_jiffies(budget));
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

	/* The idle deadline runs from the end of the last waveform. */
	ebc_rails_rearm(ebc);

	if (!unknown || rk3026_ebc_drive_knows_glass(mode, lut_mode))
		WRITE_ONCE(ebc->shadows.prev_unknown, false);
	last->outcome = EBC_OUTCOME_DRIVEN;
out:
	last->driven = done;
	last->scan_ms = ktime_ms_delta(ktime_get(), start);
	trace_rk3026_ebc_scan_end(mode, frames, done, last->scan_ms, ret,
				  last->seq);
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
 * Called for EBC_REPAIR_BY_RESET alone (ebc_repair_by() found the RESET
 * at @temp).
 *
 * Return: EBC_REPAIR_DONE if the RESET ran to completion (@last->repaired
 * set); EBC_REPAIR_FAILED on rk3026_ebc_drive()'s error: the rails are
 * down and the part must not be driven.
 */
static enum rk3026_ebc_repair ebc_recover_panel(struct rk3026_ebc *ebc,
						int temp,
						struct rk3026_ebc_last *last)
{
	const u8 *lut;
	u32 frames;

	lut = rk3026_ebc_waveform_lut(ebc->waveform, EBC_MODE_RESET, temp,
				      &frames);
	if (WARN_ON_ONCE(!lut))
		return EBC_REPAIR_NONE;

	dev_warn(ebc->dev, "panel state unknown, clearing with %u RESET frames\n",
		 frames);

	last->repaired = true;
	if (rk3026_ebc_drive(ebc, lut, frames, EBC_MODE_RESET, EBC_MODE_RESET,
			     last))
		return EBC_REPAIR_FAILED;

	memset(ebc->prev_virt, 0xff, ebc->img_size);
	return EBC_REPAIR_DONE;
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
		   jiffies + msecs_to_jiffies(max(READ_ONCE(ebc->rails_off_ms), 100U)));
	return ret;
}

/*
 * The refresh thread brings the rails up for a part it will drive, before
 * it takes the part (rk3026_ebc_do_refresh()), or when the take's gap opens
 * for it (rk3026_ebc_take_ready()). Rails already up are left as they are,
 * their off deadline too: only a driven waveform moves it (@rails_off_ms),
 * and the thread looks at it between parts alone, and not while a gap is
 * open, so the rails cannot drop between this and the drive.
 *
 * KUnit replaces it with a static stub, so that the glue can be run with
 * the rails down, coming up or failing, without a regulator.
 *
 * Returns 0 with both rails up, or ebc_rails_on()'s error.
 */
VISIBLE_IF_KUNIT int rk3026_ebc_raise_rails(struct rk3026_ebc *ebc)
{
	int ret;

	KUNIT_STATIC_STUB_REDIRECT(rk3026_ebc_raise_rails, ebc);

	mutex_lock(&ebc->rails_lock);
	ret = ebc_rails_on(ebc);
	mutex_unlock(&ebc->rails_lock);
	return ret;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_raise_rails);

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
		trace_rk3026_ebc_skip(last);
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
 * The image the next part is drawn from: @pre_shadow while @pre_split (the
 * part is the @pending.pre a RESET was sent behind), else @next_shadow.
 */
static const u8 *ebc_take_target(struct rk3026_ebc *ebc)
{
	return ebc->pre_split ? ebc->pre_shadow : ebc->next_shadow;
}

/*
 * Whether @part, the next one (rk3026_ebc_pending_peek()), will be driven:
 * the CRTC is on, a waveform is loaded, and its scan would move something
 * or is exempt from the skip (rk3026_ebc_should_skip(), on the target
 * compared with the glass: rk3026_ebc_target_differs()). Under @image_lock.
 * A part that will not be driven never raises the rails (@rails_off_ms).
 */
static bool ebc_part_drives(struct rk3026_ebc *ebc,
			    const struct rk3026_ebc_set *part)
{
	lockdep_assert_held(&ebc->image_lock);
	if (!ebc->crtc_on || !ebc->waveform)
		return false;
	return !rk3026_ebc_should_skip(part, ebc->shadows.prev_unknown,
				       !rk3026_ebc_target_differs(&ebc->shadows,
								  ebc_take_target(ebc),
								  &part->box));
}

/*
 * What a part needs before its waveform, without @image_lock: the sensor,
 * unless an update brought a temperature (@temp_valid), and the rails.
 * Returns rk3026_ebc_raise_rails()'s result, or @failed, the error of the
 * rails raised for the part already, at the take's gap: they are not asked
 * twice for one try, the carry's retry asks next (rk3026_ebc_retry_next()).
 */
static int ebc_prepare(struct rk3026_ebc *ebc, bool temp_valid, int *temp,
		       int failed)
{
	if (!temp_valid)
		*temp = rk3026_ebc_read_temp(ebc);
	return failed ?: rk3026_ebc_raise_rails(ebc);
}

/*
 * The temperature @part is driven at: the newest one an update brought wins
 * over the sensor's (@temp, from ebc_prepare()); one above the tables is
 * replaced by the fallback.
 */
static int ebc_part_temp(const struct rk3026_ebc_set *part, int temp)
{
	if (part->temp_valid)
		temp = part->temp;
	if (temp > EBC_TEMP_MAX)
		temp = EBC_TEMP_FALLBACK;
	return temp;
}

/*
 * The plane parts' DU budget as it stands: what is spent, and the limits
 * (debugfs du_cleanup_*). Under @image_lock, which guards @du_area.
 */
static struct rk3026_ebc_du_budget ebc_du_budget(struct rk3026_ebc *ebc)
{
	lockdep_assert_held(&ebc->image_lock);
	return (struct rk3026_ebc_du_budget) {
		.area		= ebc->du_area,
		.limit		= READ_ONCE(ebc->du_cleanup_area),
		.big_area	= READ_ONCE(ebc->du_cleanup_big_area),
		.ceiling	= READ_ONCE(ebc->du_cleanup_ceiling),
	};
}

/*
 * The repair a part gets on a glass that is not known: a RESET before it;
 * without a RESET waveform at @temp, the part itself driven FULL; without
 * a FULL one either, none (EBC_REPAIR_NOT_DUE): the part is driven as if
 * the glass were known, and its drive leaves the mark
 * (rk3026_ebc_drive_knows_glass()), so every part is drawn so, none
 * skipped, until the temperature has a RESET or a FULL. The RESET LUT this
 * looks up is the one ebc_recover_panel() drives (the waveform keeps the
 * last LUT it decoded), and nothing else is fetched before it.
 */
static enum rk3026_ebc_repair_due ebc_repair_by(struct rk3026_ebc *ebc,
						int temp)
{
	u32 frames;

	if (rk3026_ebc_waveform_lut(ebc->waveform, EBC_MODE_RESET, temp,
				    &frames))
		return EBC_REPAIR_BY_RESET;
	dev_warn_ratelimited(ebc->dev,
			     "panel state unknown and no RESET waveform at %d C\n",
			     temp);
	if (rk3026_ebc_waveform_lut(ebc->waveform, EBC_MODE_FULL, temp,
				    &frames))
		return EBC_REPAIR_BY_FULL;
	return EBC_REPAIR_NOT_DUE;
}

/*
 * The flip_event tracepoint of a commit event, held for @held_us and sent
 * as @why with @err on its out-fence (ebc_flip_send()), or not held, its
 * part already failed (rk3026_ebc_flip_hold()). KUnit replaces it with a
 * static stub to read what is traced.
 */
VISIBLE_IF_KUNIT void rk3026_ebc_flip_trace(struct rk3026_ebc *ebc, u32 seq,
					    u32 held_us,
					    enum rk3026_ebc_flip_why why,
					    int err)
{
	KUNIT_STATIC_STUB_REDIRECT(rk3026_ebc_flip_trace, ebc, seq, held_us,
				   why, err);
	trace_rk3026_ebc_flip_event(seq, held_us, why, err);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_flip_trace);

/*
 * Send the held commit event, traced with @why, and empty the slot, under
 * @drm.event_lock. The EBC has no vblank, so drm_crtc_send_vblank_event()
 * stamps it frame 0 at the current time; the core completes the commit's
 * flip_done and signals its out-fence with it, with the error @err if not
 * 0. An event held to the take (@flip_rel EBC_FLIP_REL_TAKE) gets no
 * error, whoever sends it: it says its part was taken, and never that the
 * frame is on the glass. The watchdog is stopped, without waiting for a
 * run already started: that run finds the slot empty, or holding a later
 * event not yet due (rk3026_ebc_flip_watchdog_run()). A watchdog send
 * counts late.
 */
static void ebc_flip_send(struct rk3026_ebc *ebc, enum rk3026_ebc_flip_why why,
			  int err)
{
	struct dma_fence *fence = ebc->flip_event->base.fence;

	lockdep_assert_held(&ebc->drm.event_lock);

	if (ebc->flip_rel == EBC_FLIP_REL_TAKE)
		err = 0;
	rk3026_ebc_flip_trace(ebc, ebc->flip_seq,
			      rk3026_ebc_us_between(ebc->flip_held_at,
						    rk3026_ebc_clock(ebc)),
			      why, err);
	if (err && fence)
		dma_fence_set_error(fence, err);
	cancel_delayed_work(&ebc->flip_wd);
	if (why == EBC_FLIP_WATCHDOG)
		WRITE_ONCE(ebc->flip_late, ebc->flip_late + 1);
	drm_crtc_send_vblank_event(&ebc->crtc, ebc->flip_event);
	ebc->flip_event = NULL;
}

/*
 * Why a held event whose update has finished goes out: @part, the part
 * that finished it, left it off the glass (@err, rk3026_ebc_part_err()),
 * was dropped with the CRTC off, as the CRTC disable's send says, or put
 * it on the glass.
 */
static enum rk3026_ebc_flip_why
ebc_flip_finished_why(const struct rk3026_ebc_last *part, int err)
{
	if (err)
		return EBC_FLIP_FAILED;
	if (part->outcome == EBC_OUTCOME_CRTC_OFF)
		return EBC_FLIP_OFF;
	return EBC_FLIP_DONE;
}

/* The move of a done seq from @from to @to passed the update @seq. */
static bool ebc_seq_passed(u32 from, u32 to, u32 seq)
{
	return !rk3026_ebc_seq_done(from, seq) && rk3026_ebc_seq_done(to, seq);
}

/*
 * Send the held commit event (rk3026_ebc_flip_hold()) once its part has
 * reached the release point the hold recorded, under @image_lock, which
 * @pending.taken_seq and @pending.done_seq move under, and
 * @drm.event_lock. Only records and sends: the event never waits for the
 * panel.
 *
 * An event held to the take (@flip_rel EBC_FLIP_REL_TAKE) goes out, why
 * take, once @pending.taken_seq has reached its update: the refresh thread
 * checks right after rk3026_ebc_pending_take(), for every part it takes,
 * driven or not. Parts are taken in seq order, so the take of a part ahead
 * of it does not send it; a part taken but not driven (identical, the CRTC
 * off, no waveform) has done with the update as far as the panel ever
 * will.
 *
 * An event held to the finish is never sent by the take: the refresh
 * thread checks after rk3026_ebc_scan_finish(), the one place
 * @pending.done_seq moves, and after ebc_publish() recorded the part that
 * moved it in @last. If it moved since the last check (@flip_done_seq),
 * the result of that part (rk3026_ebc_part_err()) is recorded in
 * @commit_err if the part has the update of the commit between its plane
 * update and its flush (@commit_seq), and the held event goes out if the
 * part has the update of the event's commit (@flip_seq,
 * ebc_flip_finished_why()), its out-fence with that error if not 0; then
 * @flip_done_seq follows.
 */
static void ebc_flip_check_locked(struct rk3026_ebc *ebc)
{
	u32 done = READ_ONCE(ebc->pending.done_seq);
	int err;

	lockdep_assert_held(&ebc->image_lock);
	lockdep_assert_held(&ebc->drm.event_lock);
	/* First: a take hold's update is taken before it is done. */
	if (ebc->flip_event && ebc->flip_rel == EBC_FLIP_REL_TAKE &&
	    rk3026_ebc_seq_done(ebc->pending.taken_seq, ebc->flip_seq))
		ebc_flip_send(ebc, EBC_FLIP_TAKE, 0);
	if (done != ebc->flip_done_seq) {
		err = rk3026_ebc_part_err(&ebc->last);
		if (ebc_seq_passed(ebc->flip_done_seq, done, ebc->commit_seq))
			ebc->commit_err = err;
		if (ebc->flip_event &&
		    ebc_seq_passed(ebc->flip_done_seq, done, ebc->flip_seq))
			ebc_flip_send(ebc, ebc_flip_finished_why(&ebc->last, err),
				      err);
		ebc->flip_done_seq = done;
	}
}

/*
 * The refresh thread's check, under @image_lock: at the take (step 2 of
 * rk3026_ebc_do_refresh()) and at the finish (step 4) of every part
 * (ebc_flip_check_locked()).
 */
static void ebc_flip_check(struct rk3026_ebc *ebc)
{
	unsigned long flags;

	spin_lock_irqsave(&ebc->drm.event_lock, flags);
	ebc_flip_check_locked(ebc);
	spin_unlock_irqrestore(&ebc->drm.event_lock, flags);
}

/*
 * The check (ebc_flip_check_locked()), then a held event still there goes
 * out now as @why, whatever its part does: the CRTC went off, a system
 * sleep is under way, or the driver goes away, and no part is taken before
 * the next enable or the resume. Under @image_lock.
 */
static void ebc_flip_send_held(struct rk3026_ebc *ebc,
			       enum rk3026_ebc_flip_why why)
{
	unsigned long flags;

	spin_lock_irqsave(&ebc->drm.event_lock, flags);
	ebc_flip_check_locked(ebc);
	if (ebc->flip_event)
		ebc_flip_send(ebc, why, 0);
	spin_unlock_irqrestore(&ebc->drm.event_lock, flags);
}

/**
 * rk3026_ebc_flip_watchdog_run - send a held commit event that is overdue
 * @ebc: the device
 * @now: jiffies
 *
 * The watchdog's run (@flip_wd, armed by rk3026_ebc_flip_hold() for
 * @flip_due). If the part of the held event has not finished by then (a
 * carry waiting for its rails, a refresh thread stuck, a long repair), or
 * not been taken for an event held to the take, the event is sent late
 * rather than let the core's commit wait time out, counted in @flip_late,
 * its out-fence with -ETIMEDOUT (none when held to the take,
 * ebc_flip_send()): the frame is not confirmed on the glass, and its part
 * may still put it there. A run that lost the race with a send finds the
 * slot empty, or holding a later event that is not due yet: that one is
 * left held, and the watchdog armed again for its time. Takes only
 * @drm.event_lock, never @image_lock.
 */
VISIBLE_IF_KUNIT void rk3026_ebc_flip_watchdog_run(struct rk3026_ebc *ebc,
						   unsigned long now)
{
	unsigned long flags;

	spin_lock_irqsave(&ebc->drm.event_lock, flags);
	if (ebc->flip_event) {
		/*
		 * An event not due yet is armed for again. Defensive only:
		 * the hold of a later event armed the watchdog for it
		 * already, and a timer never runs early.
		 */
		if (time_after_eq(now, ebc->flip_due))
			ebc_flip_send(ebc, EBC_FLIP_WATCHDOG, -ETIMEDOUT);
		else
			mod_delayed_work(system_dfl_wq, &ebc->flip_wd,
					 ebc->flip_due - now);
	}
	spin_unlock_irqrestore(&ebc->drm.event_lock, flags);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_flip_watchdog_run);

static void ebc_flip_watchdog(struct work_struct *work)
{
	struct rk3026_ebc *ebc = container_of(to_delayed_work(work),
					      struct rk3026_ebc, flip_wd);

	rk3026_ebc_flip_watchdog_run(ebc, jiffies);
}

/**
 * rk3026_ebc_flip_init - set up the commit event holder
 * @ebc: the device
 *
 * At probe, before the CRTC can take a commit: the watchdog and its default
 * bound (EBC_FLIP_TIMEOUT_MS), and the release at the finish
 * (EBC_FLIP_REL_DONE). The remove and the shutdown stop it all
 * (rk3026_ebc_flip_fini()).
 */
VISIBLE_IF_KUNIT void rk3026_ebc_flip_init(struct rk3026_ebc *ebc)
{
	INIT_DELAYED_WORK(&ebc->flip_wd, ebc_flip_watchdog);
	ebc->flip_timeout_ms = EBC_FLIP_TIMEOUT_MS;
	ebc->flip_release = EBC_FLIP_REL_DONE;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_flip_init);

/**
 * rk3026_ebc_flip_fini - stop the commit event holder for good
 * @ebc: the device
 *
 * At remove and at shutdown, after the CRTC disable. A commit tail that
 * the disable's commit did not wait for (drm_atomic_helper_shutdown()
 * failed before it) can still reach .atomic_flush; the tail keeps the
 * DRM device, not the memory the watchdog runs on. From here
 * rk3026_ebc_flip_hold() holds nothing (@flip_closed): a commit after the
 * shutdown gets its event at once. Then the watchdog is stopped and what
 * is still held is sent (why off), with its out-fence. Only the hold arms
 * the watchdog for a new event, under @drm.event_lock after it looked at
 * @flip_closed: a hold before this function's section of that lock armed
 * it before the cancel, one after arms nothing, and the watchdog's own run
 * arms it again only for an event still held, which the cancel waits out.
 * So no watchdog runs past the remove or the shutdown. Running it twice
 * does no harm: the second finds the slot empty and the watchdog idle.
 */
VISIBLE_IF_KUNIT void rk3026_ebc_flip_fini(struct rk3026_ebc *ebc)
{
	unsigned long flags;

	spin_lock_irqsave(&ebc->drm.event_lock, flags);
	ebc->flip_closed = true;
	spin_unlock_irqrestore(&ebc->drm.event_lock, flags);
	cancel_delayed_work_sync(&ebc->flip_wd);
	mutex_lock(&ebc->image_lock);
	ebc_flip_send_held(ebc, EBC_FLIP_OFF);
	mutex_unlock(&ebc->image_lock);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_flip_fini);

/*
 * Take the next part and drive it onto the panel, if anything is pending.
 * Runs only on the refresh thread.
 *
 * Four steps, so that no sender waits for a waveform:
 *
 *  1. under @image_lock: a look at the next part (rk3026_ebc_pending_peek()).
 *     One that will be driven (ebc_part_drives()) gets the sensor and the
 *     rails first, without the lock: what is sent while the rails come up
 *     joins it, rather than make a part of its own behind it. One that will
 *     not (CRTC off, no waveform, nothing to move) is taken at once, and
 *     the rails stay as they are;
 *  2. under @image_lock: the part is taken (rk3026_ebc_pending_take(): what
 *     was sent before a RESET, then the RESET alone, then what was sent
 *     after it), a commit event held to the take for an update in it is
 *     sent (ebc_flip_check()), and its snapshot is built into
 *     @shadows.scan from @shadows.prev and the target (@next_shadow, or
 *     @pre_shadow for what was sent before a pending RESET). Senders may
 *     blit into @next_shadow and add to @pending as soon as the lock is
 *     dropped: the snapshot is what this scan drives, and their updates go
 *     to the next part.
 *  3. without it: the snapshot into @next_virt, a repair if the glass is
 *     not known, the waveform;
 *  4. under @image_lock again: rk3026_ebc_scan_finish() says what the glass
 *     shows now and moves @pending.done_seq over the part, or arms the
 *     retry of a carry, the DU cleanup budget follows it
 *     (rk3026_ebc_du_account()), and the record is published and traced
 *     (part_done) with them. A commit event held to the finish for an
 *     update in the part is sent (ebc_flip_check()), the waiters on the
 *     part's updates are woken then, and @prev_virt follows.
 *
 * The part is the carry alone at its retry (rk3026_ebc_retry_ready()).
 *
 * While the system suspends (@suspended) nothing is taken: at step 1, or
 * at step 2 when the suspend began while the rails came up. From the take
 * to the finish of step 4 the part is @in_flight, which the suspend waits
 * for, so a waveform is never cut short by it.
 */
VISIBLE_IF_KUNIT void rk3026_ebc_do_refresh(struct rk3026_ebc *ebc,
					    struct rk3026_ebc_last *last)
{
	enum rk3026_ebc_repair_due repair = EBC_REPAIR_NOT_DUE;
	enum rk3026_ebc_repair repair_result = EBC_REPAIR_NONE;
	struct rk3026_ebc_shadows *sh = &ebc->shadows;
	ktime_t start = ktime_get(), snap_start, taken;
	enum rk3026_ebc_reason reason;
	struct rk3026_ebc_du_budget budget;
	struct rk3026_ebc_a2 a2 = { 0 };
	enum rk3026_ebc_gave_up gave_up;
	struct rk3026_ebc_set part;
	bool identical, prev_changed, two_level, prepared = false;
	u32 frames, mode, scan_mode, resolved, lut_mode, tries;
	u32 changed_px = 0;
	const u8 *lut, *target;
	int temp = EBC_TEMP_FALLBACK;
	int rails = 0, rails_err, forced;

	mutex_lock(&ebc->image_lock);
	if (ebc->suspended || !rk3026_ebc_pending_peek(&ebc->pending, &part)) {
		mutex_unlock(&ebc->image_lock);
		return;
	}
	/* The rails failed at the take's gap: this try does not ask again. */
	rails_err = ebc->gap.rails_err;
	ebc->gap.rails_err = 0;

	/*
	 * Only this thread takes, so the part is still pending after the
	 * unlock; senders may have joined it, which is the point.
	 */
	if (ebc_part_drives(ebc, &part)) {
		bool sensed = !part.temp_valid;

		mutex_unlock(&ebc->image_lock);
		rails = ebc_prepare(ebc, part.temp_valid, &temp, rails_err);
		prepared = true;
		mutex_lock(&ebc->image_lock);
		/*
		 * A RESET sent meanwhile drops the carry, and with it the
		 * temperature the part may have had from it alone: the sensor
		 * is read after all. Updates only add temperatures, and only
		 * this thread makes a carry, so the next look holds.
		 */
		rk3026_ebc_pending_peek(&ebc->pending, &part);
		if (!sensed && !part.temp_valid) {
			mutex_unlock(&ebc->image_lock);
			temp = rk3026_ebc_read_temp(ebc);
			mutex_lock(&ebc->image_lock);
		}
		/*
		 * A suspend that began while the rails came up: the part stays
		 * pending, with its carry and retry as they were, and is taken
		 * after the resume. The suspend takes the rails down.
		 */
		if (ebc->suspended) {
			mutex_unlock(&ebc->image_lock);
			return;
		}
	}
	tries = rk3026_ebc_retry_try(&ebc->pending);
	rk3026_ebc_pending_take(&ebc->pending, &part);
	/* Until the finish below; a suspend waits for it (ebc_suspend()). */
	WRITE_ONCE(ebc->in_flight, true);
	/*
	 * A commit event held to the take of this part goes now, after the
	 * take's time is read: the setup_us of the part's waveforms counts
	 * from that time, and so never puts the take after its event.
	 */
	taken = rk3026_ebc_clock(ebc);
	ebc_flip_check(ebc);

	/*
	 * Taken while @pre_split, the part is the @pending.pre a RESET was sent
	 * behind (it goes first): it is drawn from the image its updates were
	 * sent into, and that image is the spare again.
	 */
	target = ebc_take_target(ebc);
	ebc->pre_split = false;

	memset(last, 0, sizeof(*last));
	last->taken = taken;
	last->seq = part.seq;
	last->du_area = ebc->du_area;
	last->clip = part.box;
	last->asked_mode = part.mode;
	last->mode = part.mode;
	last->lut_mode = part.mode;
	last->marker = part.marker;
	last->merged = part.merged;
	last->source = part.source;
	last->blit_us = part.blit_us;
	/* The gap the take waited in, if one was open (rk3026_ebc_take_ready()). */
	last->gap_us = ebc->gap.us;
	ebc->gap.us = 0;
	last->tries = tries;

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
	 * The sensor was read before the take (step 1) for a part the look
	 * found it would drive, so the temperature, and with it what the
	 * waveform has, is known here.
	 */
	if (prepared)
		temp = ebc_part_temp(&part, temp);

	/*
	 * A glass that is not known is repaired with a RESET before the draw
	 * (below), except before a RESET, which clears it anyway. After a
	 * repair the glass is white outside the box too, while the snapshot
	 * there holds the old greys, so the scan runs at least PART; the
	 * snapshot is then not thresholded either. Without a RESET at the
	 * temperature the part is driven FULL instead
	 * (rk3026_ebc_resolve_mode()): a PART drive would not clear the mark
	 * (rk3026_ebc_drive()); without a FULL either, as it is
	 * (ebc_repair_by()). Such a part was prepared: the look drives every
	 * part while @prev_unknown is set, and while this thread runs only it
	 * changes it (a hibernation restore sets it with the thread parked).
	 */
	if (sh->prev_unknown && part.mode != EBC_MODE_RESET)
		repair = ebc_repair_by(ebc, temp);
	scan_mode = rk3026_ebc_scan_mode(&part, repair != EBC_REPAIR_NOT_DUE);

	/* An A2 scan over a grey of the glass comes back as DU. */
	snap_start = ktime_get();
	identical = !rk3026_ebc_build_snapshot(sh, target, &part.box,
					       &scan_mode, &two_level,
					       &a2.over_grey, &changed_px);
	last->snap_us = ebc_us_since(snap_start);
	last->changed_px = changed_px;

	/*
	 * Nothing changed? Then do not drive the panel.
	 *
	 * This is not an optimisation, it is a correctness and wear fix. Every
	 * update arrives here TWICE: FBIOPAN_DISPLAY alone produces two atomic
	 * commits through drm_fb_helper's fbdev emulation (skipping the
	 * client's msync changes nothing), and ebc_plane_atomic_update()
	 * queues the damage of each without asking whether the pixels moved. So
	 * the second scan runs a complete waveform with prev == next.
	 *
	 * A GC16 waveform with identical prev and next is NOT a no-op: it still
	 * runs its whole flash sequence, so it re-shows the previous image and
	 * settles back. At 12 MHz that is a visible three-second replay after
	 * every single draw. Left alone it is also continuous wear: fbcon's
	 * cursor blink drove a 38-frame full waveform every ~1.1 s for ever.
	 *
	 * The test is on the cached snapshot against the cached prev, never on
	 * the write-combined images. rk3026_ebc_should_skip() holds the
	 * exemptions:
	 *
	 * EBC_MODE_RESET is exempt, and the test is on the PART's own mode,
	 * not the possibly-forced one. The boot-time clear legitimately has
	 * prev == next == white: the controller indexes its LUT by (prev, next),
	 * so the driver must MAKE the panel white rather than assume it is, and
	 * skipping that would leave whatever the previous OS left on the glass.
	 * Keying off `mode` instead would let force_mode disable the boot clear.
	 *
	 * DO NOT DROP the @prev_unknown exemption. The skip is a decision made
	 * from @shadows.prev, and the flag says it is not evidence; without the
	 * exemption an aborted waveform makes the draw that would repair the
	 * glass the one thrown away, and the panel is unreachable from userspace.
	 * It does not weaken the skip: a successful refresh clears the flag, so
	 * fbcon redrawing one image for ever still skips.
	 *
	 * An MXCFB_SEND_UPDATE in a full-class mode is exempt too
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
	 * The idle cleanup is exempt as well (@cleanup_idle): it redraws the
	 * glass as it is, so its snapshot is identical as a rule, and the
	 * flash is what it was queued for (rk3026_ebc_idle_tick()).
	 */
	if (rk3026_ebc_should_skip(&part, sh->prev_unknown, identical)) {
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
	budget = ebc_du_budget(ebc);
	a2.max_area = READ_ONCE(ebc->a2_max_area);
	resolved = rk3026_ebc_resolve_mode(&part, scan_mode, two_level,
					   changed_px, repair, &budget, &a2,
					   &reason);
	last->reason = reason;

	/* debugfs force_mode: another mode class, same data and scan. */
	forced = READ_ONCE(ebc->force_mode);
	mode = forced >= 0 && forced <= EBC_MODE_MAX ? forced : resolved;
	last->mode = mode;
	mutex_unlock(&ebc->image_lock);

	/*
	 * The sensor and the rails came before the take (step 1) for a part
	 * the look found it would drive. The look and the take see the same
	 * part under one hold of the lock when it found otherwise, and such a
	 * part ends above (CRTC off, no waveform, skipped: a target equal to
	 * the glass makes an identical snapshot); one that got here all the
	 * same gets them now, as before the look.
	 */
	if (WARN_ON_ONCE(!prepared)) {
		rails = ebc_prepare(ebc, part.temp_valid, &temp, rails_err);
		temp = ebc_part_temp(&part, temp);
	}
	last->temp = temp;
	last->temp_read = true;

	/*
	 * Rails up before anything is driven, the repair below included. If they
	 * did not come up nothing is driven and the glass is as it was, so the
	 * prev image stays true and the part is carried and tried again
	 * (rk3026_ebc_retry_next()) -- except for a RESET, which is only queued
	 * when the glass is not known (the boot clear) and so leaves the mark
	 * (rk3026_ebc_scan_finish()).
	 */
	if (rails) {
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
	 * It must run BEFORE the LUT is fetched. rk3026_ebc_waveform_lut()
	 * decodes into one buffer inside the container and hands out a pointer to
	 * it, so loading the RESET waveform overwrites whatever was fetched
	 * earlier -- fetching first would silently drive the requested mode's
	 * frame count through the RESET LUT.
	 */
	if (repair == EBC_REPAIR_BY_RESET) {
		/*
		 * A RESET that failed took the rails down: nothing more this
		 * time, and the part is carried (rk3026_ebc_scan_finish()).
		 */
		repair_result = ebc_recover_panel(ebc, temp, last);
		if (repair_result == EBC_REPAIR_FAILED)
			goto relock;
	}

	/*
	 * A direct mode has no LUT of its own; fall back to the resolved mode
	 * for the scan data so the DSP_CTRL class is the only thing that changed.
	 * A resolved mode without a LUT is not driven in another one: the part
	 * is carried in its plain mode (rk3026_ebc_scan_finish()) and tried
	 * again, alone at its retry (the temperature, and with it the table,
	 * may change) or with the next update, in the stronger of the two
	 * plain modes: an auto plane part without a DU or PART LUT ends up
	 * FULL, as before the policy. Given up, it waits for the next update.
	 */
	lut_mode = mode;
	lut = rk3026_ebc_waveform_lut(ebc->waveform, mode, temp, &frames);
	if (!lut && mode != resolved) {
		lut_mode = resolved;
		lut = rk3026_ebc_waveform_lut(ebc->waveform, resolved, temp,
					      &frames);
	}
	if (!lut) {
		last->outcome = EBC_OUTCOME_NO_LUT;
		goto relock;
	}

	rk3026_ebc_drive(ebc, lut, frames, mode, lut_mode, last);

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
	 * The swap STAYS UNCONDITIONAL once the waveform was entered. Keeping
	 * the old prev after a timeout looks like the fix and is not: prev would
	 * then hold a still older image, equally false and carrying no mark
	 * saying so. @prev_unknown is the mark, and the next draw acts on it.
	 */
	prev_changed = rk3026_ebc_scan_finish(sh, &ebc->pending, &part,
					      last->outcome, repair_result,
					      rk3026_ebc_clock(ebc), &gave_up);
	last->gave_up = gave_up != EBC_GAVE_UP_NONE;
	if (last->gave_up) {
		ebc->stats.carry_failed++;
		/* Once a failure: later carries give up at once until a drive. */
		if (gave_up == EBC_GAVE_UP)
			dev_warn_ratelimited(ebc->dev,
					     "part (seq %u) not drawn after %u tries (outcome %u), given up\n",
					     part.seq, tries + 1, last->outcome);
	}
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
	/* The cap of the next take's gap counts from here at the earliest. */
	ebc->take_done = rk3026_ebc_clock(ebc);
	WRITE_ONCE(ebc->in_flight, false);
	ebc_publish(ebc, last, start);
	trace_rk3026_ebc_part_done(last, ebc->du_area);
	/* After the move of @pending.done_seq: see rk3026_ebc_flip_hold(). */
	ebc_flip_check(ebc);
	mutex_unlock(&ebc->image_lock);

	/*
	 * @pending.done_seq moved over the part -- driven, skipped, dropped
	 * with the CRTC off, a carry given up -- so every wait on one of its
	 * updates ends here; a carry armed for a retry holds it (a wake-up
	 * then finds the wait's condition false and sleeps again). A suspend
	 * waiting for the part in flight is woken here too.
	 */
	wake_up_all(&ebc->marker_wq);

	/* Only this thread writes either prev image, so this needs no lock. */
	if (prev_changed)
		memcpy(ebc->prev_virt, sh->prev, ebc->img_size);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_do_refresh);

/*
 * Idle power-off: @rails_off_ms passed since the last waveform. Only this
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
 * The cleanup does not wait for the take's quiet gap (exempt,
 * rk3026_ebc_take_wait()): it is the whole panel already, and nothing
 * sent after it would make it bigger. An update sent between the queue
 * here and the take in rk3026_ebc_do_refresh() (the rails coming up, if
 * they are down) joins it and is drawn in its whole-panel FULL. A first
 * mxcfb send gets a full flash it did not ask for then, once; such a
 * client runs its own fulls.
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
	struct rk3026_ebc_du_budget budget;
	struct rk3026_ebc_idle idle;
	bool due, expired;

	mutex_lock(&ebc->image_lock);
	budget = ebc_du_budget(ebc);
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

/*
 * The next part as rk3026_ebc_take_wait() reads it (struct
 * rk3026_ebc_take), from @part, as rk3026_ebc_pending_peek() gave it.
 * Under @image_lock.
 */
static void ebc_take_state(struct rk3026_ebc *ebc,
			   const struct rk3026_ebc_set *part,
			   struct rk3026_ebc_take *t)
{
	struct rk3026_ebc_du_budget budget = ebc_du_budget(ebc);
	u64 area = 0;

	if (drm_rect_visible(&part->box))
		area = (u64)drm_rect_width(&part->box) *
		       drm_rect_height(&part->box);

	*t = (struct rk3026_ebc_take) {
		.first		= part->first,
		.last		= part->last,
		.prev_done	= ebc->take_done,
		.count		= part->merged,
		.area		= min_t(u64, area, U32_MAX),
		.big_area	= budget.big_area,
		.quiet_us	= READ_ONCE(ebc->take_quiet_us),
		.cap_ms		= READ_ONCE(ebc->take_cap_ms),
		.full		= READ_ONCE(ebc->force_mode) == EBC_MODE_FULL ||
				  (part->auto_ok && part->has_plane &&
				   rk3026_ebc_du_at_ceiling(&budget)),
		.exempt		= part->has_mxcfb || part->cleanup_idle ||
				  part->mode == EBC_MODE_RESET ||
				  ebc->pending.reset.merged ||
				  ebc->pending.retry.ready,
		.rails_up	= ebc_rails_up(ebc),
		.open		= ebc->gap.open,
		.retry_at	= ebc->pending.retry.at,
		.retry_armed	= ebc->pending.retry.armed,
	};
}

/* The gap ends at @now, if one is open: its length goes to the next part. */
static void ebc_gap_close(struct rk3026_ebc *ebc, ktime_t now)
{
	if (!ebc->gap.open)
		return;
	ebc->gap.us = rk3026_ebc_us_between(ebc->gap.start, now);
	ebc->gap.open = false;
}

/**
 * rk3026_ebc_take_ready - whether the refresh thread takes the next part now
 * @ebc: controller
 * @now: the time of the look (rk3026_ebc_clock())
 *
 * Called by the refresh thread when a part is pending, a gap is open, or
 * the carry's retry is due (rk3026_ebc_retry_ready()). Under @image_lock
 * it looks at the next part and asks rk3026_ebc_take_wait(), which holds
 * the rules, whether to wait for the senders to go quiet.
 *
 * A gap opens only for a part that will be driven (ebc_part_drives()); the
 * look cannot tell every part that will be skipped (a two-level change
 * that thresholds back to the glass, updates that join and restore it),
 * and such a part raised the rails for nothing, once. Opening the gap
 * raises the rails at once, without the lock: they take tens of ms, and
 * updates sent meanwhile join the part. Rails that do not come up end the
 * gap: the part is taken and carried without asking for them again
 * (@gap.rails_err), and its retries ask next.
 *
 * Every look records the newest seq in @gap.seen, for the thread's wait
 * (rk3026_ebc_gap_woken()). The gap ends at its deadline, when the part
 * becomes exempt or the gap is switched off, when the CRTC goes off
 * (rk3026_ebc_crtc_off()), or when the system suspends; its length is the
 * next part's @gap_us.
 *
 * Return: true to take the next part now; false while the gap is open,
 * which ends at @gap.at unless a newer update moves it.
 */
VISIBLE_IF_KUNIT bool rk3026_ebc_take_ready(struct rk3026_ebc *ebc,
					    ktime_t now)
{
	struct rk3026_ebc_gap *gap = &ebc->gap;
	struct rk3026_ebc_set part;
	struct rk3026_ebc_take t;
	bool raise = false;

	mutex_lock(&ebc->image_lock);
	/* Suspending: the gap ends, and rk3026_ebc_do_refresh() takes nothing. */
	if (ebc->suspended)
		goto take;
	rk3026_ebc_retry_ready(&ebc->pending, now, ebc->crtc_on);
	gap->seen = ebc->pending.queued_seq;
	if (!rk3026_ebc_pending_peek(&ebc->pending, &part) || !ebc->crtc_on)
		goto take;
	ebc_take_state(ebc, &part, &t);
	if (!rk3026_ebc_take_wait(&t, now, &gap->at))
		goto take;
	if (!gap->open) {
		if (!ebc_part_drives(ebc, &part))
			goto take;
		gap->open = true;
		gap->start = now;
		raise = !t.rails_up;
	}
	mutex_unlock(&ebc->image_lock);

	if (raise) {
		int ret = rk3026_ebc_raise_rails(ebc);

		if (ret) {
			gap->rails_err = ret;
			ebc_gap_close(ebc, rk3026_ebc_clock(ebc));
			return true;
		}
	}
	return false;

take:
	ebc_gap_close(ebc, now);
	mutex_unlock(&ebc->image_lock);
	return true;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_take_ready);

/*
 * The refresh thread's wait while the take's gap is open: for an update
 * newer than its last look, the CRTC going off, or the gap's end, which is
 * the carry's retry at the latest (rk3026_ebc_take_wait()). The gap is a
 * few ms, so the wait runs on an hrtimer, not on jiffies. A gap whose end
 * passed while the rails came up is not waited for
 * (rk3026_ebc_gap_timeout()): the next look takes the part. A park ends
 * the wait (ebc_suspend()); the gap stays open across it.
 */
static void ebc_gap_wait(struct rk3026_ebc *ebc)
{
	const struct rk3026_ebc_gap *gap = &ebc->gap;
	ktime_t left;

	if (!rk3026_ebc_gap_timeout(gap->at, rk3026_ebc_clock(ebc), &left))
		return;
	wait_event_interruptible_hrtimeout(ebc->update_wq,
					   rk3026_ebc_gap_woken(&ebc->pending,
								gap->seen,
								READ_ONCE(ebc->crtc_on)) ||
					   kthread_should_stop() ||
					   kthread_should_park(),
					   left);
}

/**
 * rk3026_ebc_thread_pass - what one pass of the refresh thread reads
 * @ebc: controller
 * @now: the pass's time, in jiffies
 * @s: returns it, for rk3026_ebc_thread_step()
 *
 * Without @image_lock: the rails and their deadline, the idle cleanup's
 * (ebc_idle_at()), the carry's retry, the take's gap, whether the system
 * suspends and the wait's condition (rk3026_ebc_thread_woken()), at one
 * rk3026_ebc_clock() read.
 * The retry and the gap are this thread's own, so they hold still while
 * the pass runs.
 */
VISIBLE_IF_KUNIT void rk3026_ebc_thread_pass(struct rk3026_ebc *ebc,
					     unsigned long now,
					     struct rk3026_ebc_pass *s)
{
	const struct rk3026_ebc_retry *r = &ebc->pending.retry;
	bool suspended = READ_ONCE(ebc->suspended);
	ktime_t clock = rk3026_ebc_clock(ebc);

	*s = (struct rk3026_ebc_pass) {
		.now		= now,
		.rails_off_at	= READ_ONCE(ebc->rails_off_at),
		.clock		= clock,
		.retry_at	= READ_ONCE(r->at),
		.rails_on	= READ_ONCE(ebc->rails_on),
		.retry_armed	= READ_ONCE(r->armed),
		.gap_open	= ebc->gap.open,
		.suspended	= suspended,
		.woken		= rk3026_ebc_thread_woken(&ebc->pending, clock,
							  READ_ONCE(ebc->crtc_on),
							  suspended),
	};
	s->idle_armed = ebc_idle_at(ebc, &s->idle_at);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_thread_pass);

/*
 * The refresh thread's wait between parts, up to @timeout jiffies. It
 * cannot take @image_lock; it asks rk3026_ebc_thread_woken(). A plane
 * update that arms or moves the idle deadline wakes it
 * (rk3026_ebc_add_update()), and so does a CRTC disable, for a carry
 * waiting for its retry, and the end of a suspend (rk3026_ebc_sleep_end()).
 * A park ends it too (ebc_suspend()).
 */
static void ebc_work_wait(struct rk3026_ebc *ebc, long timeout)
{
	wait_event_interruptible_timeout(ebc->update_wq,
					 rk3026_ebc_thread_woken(&ebc->pending,
								 rk3026_ebc_clock(ebc),
								 READ_ONCE(ebc->crtc_on),
								 READ_ONCE(ebc->suspended)) ||
					 kthread_should_stop() ||
					 kthread_should_park(),
					 timeout);
}

static int ebc_refresh_thread(void *data)
{
	struct rk3026_ebc *ebc = data;

	while (!kthread_should_stop()) {
		struct rk3026_ebc_last last;
		struct rk3026_ebc_pass s;
		long timeout;

		/* Between passes only: no part is in flight here. */
		if (kthread_should_park()) {
			kthread_parkme();
			continue;
		}

		rk3026_ebc_thread_pass(ebc, jiffies, &s);
		switch (rk3026_ebc_thread_step(&s, &timeout)) {
		case EBC_STEP_RAILS_OFF:
			ebc_idle_rails_off(ebc);
			break;
		case EBC_STEP_IDLE:
			/* Disarms the cleanup, and queues it if it is due. */
			rk3026_ebc_idle_tick(ebc, s.now);
			break;
		case EBC_STEP_LOOK:
			/* Take the part, or wait for the senders to go quiet. */
			if (rk3026_ebc_take_ready(ebc, rk3026_ebc_clock(ebc)))
				rk3026_ebc_do_refresh(ebc, &last);
			else
				ebc_gap_wait(ebc);
			break;
		case EBC_STEP_WAIT:
			ebc_work_wait(ebc, timeout);
			break;
		}
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
 * The update has finished once @pending.done_seq reached @seq: its part was
 * driven, skipped, dropped with the CRTC off, or given up (a RESET not
 * driven, a carry without a waveform); or, carried for want of rails or a
 * LUT or behind a repair RESET cut short, it was drawn by a later try or
 * given up after its retries (rk3026_ebc_retry_next()), the wait on it
 * held meanwhile. Every part taken before it has finished too, and the
 * seqs cover each part from its take to rk3026_ebc_scan_finish(),
 * waveform included: @done_seq moves only in the @image_lock section
 * after the scan.
 *
 * Nothing pending is NOT finished: the thread takes a part out of
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
 * The wait of a CRTC enable or disable (ebc_wait_finished()): for the
 * update of @seq to finish, as rk3026_ebc_wait_seq() without a signal, or
 * for the system to be asleep with no part in flight
 * (rk3026_ebc_sleep_begin()), whichever comes first. Asleep, no part is
 * taken until the resume, and after a resume that found no clocks none is
 * taken at all, so a wait for the seq would block for @timeout_ms, a
 * suspend's freezer included; what is pending then is drawn after the
 * resume, in the order it was sent. A part in flight is still waited for:
 * the disable takes the rails down after the wait.
 *
 * Returns 0, or -ETIMEDOUT after @timeout_ms.
 */
VISIBLE_IF_KUNIT int rk3026_ebc_crtc_wait(struct rk3026_ebc *ebc, u32 seq,
					  unsigned int timeout_ms)
{
	if (!wait_event_timeout(ebc->marker_wq,
				rk3026_ebc_seq_finished(&ebc->pending, seq) ||
				(READ_ONCE(ebc->suspended) &&
				 !READ_ONCE(ebc->in_flight)),
				msecs_to_jiffies(timeout_ms)))
		return -ETIMEDOUT;
	return 0;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_crtc_wait);

/*
 * For CRTC enable and disable: bounded, so a thread stuck elsewhere cannot
 * hang blank or shutdown; and over at once while the system is asleep
 * (rk3026_ebc_crtc_wait()).
 */
static void ebc_wait_finished(struct rk3026_ebc *ebc, u32 seq)
{
	unsigned int ms = rk3026_ebc_wait_timeout_ms(ebc);

	if (rk3026_ebc_crtc_wait(ebc, seq, ms))
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
 * plane's framebuffer may not. The idle cleanup and the redraw after a
 * hibernation restore write no pixel, so they do not.
 *
 * The update is stamped with the time it was queued (rk3026_ebc_clock()),
 * for the take's quiet gap (rk3026_ebc_take_ready()).
 *
 * Returns the update's seq, for rk3026_ebc_wait_seq().
 */
u32 rk3026_ebc_add_update(struct rk3026_ebc *ebc,
			  const struct rk3026_ebc_update *upd)
{
	struct rk3026_ebc_update stamped = *upd;
	bool merged;
	u32 seq;

	lockdep_assert_held(&ebc->image_lock);
	stamped.queued = rk3026_ebc_clock(ebc);

	if (upd->mode == EBC_MODE_RESET) {
		if (ebc->pending.pre.merged && !ebc->pre_split) {
			swap(ebc->next_shadow, ebc->pre_shadow);
			ebc->pre_split = true;
		}
		memset(ebc->next_shadow, 0xff, ebc->img_size);
	}
	if (upd->source != EBC_SOURCE_PLANE && upd->source != EBC_SOURCE_IDLE &&
	    upd->source != EBC_SOURCE_RESTORE)
		ebc->plane_resync = true;
	merged = rk3026_ebc_pending_add(&ebc->pending, &stamped, &seq);
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
 * it has finished: driven, skipped, dropped with the CRTC off, or, carried
 * because the rails or a LUT were missing, drawn by a retry or given up
 * (rk3026_ebc_wait_seq()). A give-up returns 0 like a drive: the client's
 * wait means "done with it", and the content stays carried for the next
 * part. The marker is resolved to its seq once, here. Marker 0, a marker never
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
 * The queued update is recorded as the commit's (@commit_seq,
 * @commit_queued, @commit_err cleared), for its event; nothing is recorded
 * when nothing was queued.
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
		ebc->commit_seq = rk3026_ebc_add_update(ebc, &upd);
		ebc->commit_queued = true;
		ebc->commit_err = 0;
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
	 * alone in DU, A2 and PART. The waveform is chosen as for a
	 * whole-panel update: DU when every changed pixel ends black or white
	 * (A2 for a small part when a2_max_area allows it), else PART, and
	 * FULL for a cleanup of the DU budget (rk3026_ebc_resolve_mode()).
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
	 * included, and drops them once nothing is pending (@rails_off_ms).
	 * V3P3, the logic rail, stays with the CRTC.
	 *
	 * Drive the panel to a known white before anything is composed on it.
	 *
	 * The RESET's white fill of @next_shadow (rk3026_ebc_add_update())
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
	 * While the system is asleep the RESET is driven after the resume, and
	 * the commit does not wait for it (rk3026_ebc_crtc_wait()): the plane
	 * updates queue behind it all the same.
	 */
	mutex_lock(&ebc->image_lock);
	WRITE_ONCE(ebc->crtc_on, true);
	seq = rk3026_ebc_add_update(ebc, &upd);
	mutex_unlock(&ebc->image_lock);
	ebc_wait_finished(ebc, seq);
}

/*
 * The CRTC goes off: no new waveform from here on, the refresh thread drops
 * what it takes, and mxcfb sends are refused. The thread is woken, so a
 * carry waiting for its retry is dropped and an open gap ends now, not at
 * their time (rk3026_ebc_gap_woken()). A commit event still held for its
 * part is sent now (why off), still under @image_lock, the lock of the
 * take: the part is dropped, or waits for the resume of a sleep, and the
 * refresh thread cannot take it between the two. The send never waits.
 * The disable's commit waited for that event first
 * (drm_atomic_helper_wait_for_dependencies()), so this is a fallback
 * (rk3026_ebc_flip_hold()). Returns the newest seq, for the disable's
 * wait.
 */
VISIBLE_IF_KUNIT u32 rk3026_ebc_crtc_off(struct rk3026_ebc *ebc)
{
	u32 seq;

	mutex_lock(&ebc->image_lock);
	WRITE_ONCE(ebc->crtc_on, false);
	seq = ebc->pending.queued_seq;
	ebc_flip_send_held(ebc, EBC_FLIP_OFF);
	mutex_unlock(&ebc->image_lock);
	wake_up(&ebc->update_wq);
	return seq;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_crtc_off);

/*
 * A system suspend begins (ebc_suspend()): from here no part is taken
 * (rk3026_ebc_do_refresh()), and the refresh thread looks at nothing but
 * the rails (rk3026_ebc_thread_step()). Updates sent meanwhile join the
 * pending set, which keeps its parts, its carry and the carry's retry for
 * the resume. The flag goes up under @image_lock, the lock of the take, so
 * a part is either taken before it, and @in_flight, or not taken at all.
 * A CRTC enable or disable waiting meanwhile is woken: it waits no longer
 * than for the part in flight (rk3026_ebc_crtc_wait()). From here no
 * commit event is held (rk3026_ebc_flip_hold()); one held already is sent
 * by rk3026_ebc_sleep_wait().
 */
VISIBLE_IF_KUNIT void rk3026_ebc_sleep_begin(struct rk3026_ebc *ebc)
{
	mutex_lock(&ebc->image_lock);
	WRITE_ONCE(ebc->suspended, true);
	mutex_unlock(&ebc->image_lock);
	wake_up_all(&ebc->marker_wq);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_sleep_begin);

/*
 * After rk3026_ebc_sleep_begin(): wait for the part taken before it, if one
 * is in flight, to finish, its waveform included. No part is taken after
 * it, so once none is in flight none will be. Bounded by @timeout_ms, the
 * longest a part takes (rk3026_ebc_part_max_ms()): past it the thread is
 * stuck outside the EBC.
 *
 * The part in flight sent a commit event held for it at its finish, under
 * @image_lock after it cleared @in_flight (at its take, one held to the
 * take); one still held, for a part that waits for the resume, is sent now
 * (why sleep) rather than after it. With a part still in flight the
 * suspend fails, and the event stays held for that part.
 *
 * Returns 0, or -EBUSY with a part still in flight.
 */
VISIBLE_IF_KUNIT int rk3026_ebc_sleep_wait(struct rk3026_ebc *ebc,
					   unsigned int timeout_ms)
{
	if (!wait_event_timeout(ebc->marker_wq, !READ_ONCE(ebc->in_flight),
				msecs_to_jiffies(timeout_ms)))
		return -EBUSY;
	mutex_lock(&ebc->image_lock);
	ebc_flip_send_held(ebc, EBC_FLIP_SLEEP);
	mutex_unlock(&ebc->image_lock);
	return 0;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_sleep_wait);

/*
 * The suspend ends, or fails: parts are taken again, the ones pending
 * first, from @shadows.prev as the glass kept it.
 */
VISIBLE_IF_KUNIT void rk3026_ebc_sleep_end(struct rk3026_ebc *ebc)
{
	mutex_lock(&ebc->image_lock);
	WRITE_ONCE(ebc->suspended, false);
	mutex_unlock(&ebc->image_lock);
	wake_up(&ebc->update_wq);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_sleep_end);

/*
 * A hibernation restore, before rk3026_ebc_sleep_end(): the images are the
 * ones saved, but the glass may not be. The kernel that loaded the image
 * may have bound this driver and drawn with it (the RESET of its CRTC
 * enable, its console), so @shadows.prev is not evidence: the mark makes
 * the next part repair the glass first (rk3026_ebc_do_refresh()). With
 * the CRTC on, a redraw of the whole panel is queued as that part
 * (EBC_SOURCE_RESTORE): FULL, not auto, of the target as it is, so it
 * writes no pixel. Updates pending join it, as they would join any part
 * (rk3026_ebc_pending_add()); a RESET pending is drawn before it, and a
 * carry keeps its retry. With the CRTC off the next enable RESETs the
 * panel.
 *
 * The thread is parked, so it does not read the mark while it changes.
 */
VISIBLE_IF_KUNIT void rk3026_ebc_sleep_restore(struct rk3026_ebc *ebc)
{
	struct rk3026_ebc_update upd = {
		.clip	= DRM_RECT_INIT(0, 0, ebc->width, ebc->height),
		.mode	= EBC_MODE_FULL,
		.source	= EBC_SOURCE_RESTORE,
	};

	mutex_lock(&ebc->image_lock);
	WRITE_ONCE(ebc->shadows.prev_unknown, true);
	if (ebc->crtc_on)
		rk3026_ebc_add_update(ebc, &upd);
	mutex_unlock(&ebc->image_lock);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_sleep_restore);

static void ebc_crtc_atomic_disable(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct rk3026_ebc *ebc = drm_to_ebc(crtc->dev);
	u32 seq;

	/*
	 * Let the part in flight and those pending finish, then drop the
	 * rails now rather than after the idle delay: VCOM first, then the
	 * high-voltage rails, then logic. While the system is asleep only the
	 * part in flight is waited for; the thread drops the pending ones
	 * after the resume (rk3026_ebc_crtc_wait()).
	 */
	seq = rk3026_ebc_crtc_off(ebc);
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

/**
 * rk3026_ebc_commit_begin - start the record of a commit's update
 * @ebc: the device
 *
 * A commit starts with no update queued: its plane update records the one
 * it queues (rk3026_ebc_plane_update(): @commit_queued, @commit_seq). The
 * core calls .atomic_begin before the plane updates of every commit that
 * holds the CRTC while it is active, CRTC-only commits too
 * (drm_atomic_helper_commit_planes(), DRM_PLANE_COMMIT_ACTIVE_ONLY in
 * drm_atomic_helper_commit_tail_rpm()). A plane update holds the CRTC of
 * its plane (drm_atomic_get_plane_state()), and a commit that does not
 * hold the CRTC carries no CRTC event either.
 */
VISIBLE_IF_KUNIT void rk3026_ebc_commit_begin(struct rk3026_ebc *ebc)
{
	ebc->commit_queued = false;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_commit_begin);

static void ebc_crtc_atomic_begin(struct drm_crtc *crtc,
				  struct drm_atomic_state *state)
{
	rk3026_ebc_commit_begin(drm_to_ebc(crtc->dev));
}

/**
 * rk3026_ebc_flip_hold - hold a commit's event until its part has finished
 * @ebc: the device
 * @crtc_state: the commit's new CRTC state, with its event or none
 *
 * The CRTC's .atomic_flush, at the end of a commit's plane updates. The
 * event userspace asked for, as a flip event (DRM_MODE_PAGE_FLIP_EVENT:
 * @event->base.file_priv) or an out-fence (@event->base.fence), of a
 * commit that queued an update (@commit_queued) not finished yet is held
 * (rk3026_ebc_flip_holds()): taken from @crtc_state, so the core neither
 * sends it nor arms its fake vblank for it. Any other event goes out at
 * once, as before. The refresh thread sends a held event when the part
 * with that update has finished (ebc_flip_check()): driven to its end,
 * skipped for want of a change, dropped with the CRTC off, or a carry
 * drawn by a retry or given up. The event and the out-fence then say the
 * frame is on the glass, where the immediate event said only that it was
 * queued; a frame merged into a part with later updates is on the glass
 * with that part. A part that finished without putting it there sets its
 * error on the out-fence first (rk3026_ebc_part_err(): -ECANCELED for an
 * update a RESET whitened, -EIO otherwise), also when it finished before
 * this flush (@commit_err).
 *
 * The core lets the next nonblocking commit through only after this
 * flip_done, so a client that waits for its events draws one frame per
 * waveform; and every later commit's tail waits for it
 * (drm_atomic_helper_wait_for_dependencies()), so a blocking internal
 * commit (fbcon, fbdev writes, DirtyFB) waits behind a held event for up
 * to one part.
 *
 * A held event waits for its part no longer than @flip_timeout_ms, clamped
 * to EBC_FLIP_TIMEOUT_MAX_MS: the watchdog then sends it late
 * (@flip_late), within the 10 s the core waits for a commit's flip_done,
 * its out-fence with -ETIMEDOUT (not confirmed on the glass). A system
 * sleep sends it once no part is in flight (rk3026_ebc_sleep_wait()); a
 * CRTC disable, whose commit waited for it first, sends it as a fallback
 * (rk3026_ebc_crtc_off()), and so do the remove and the shutdown
 * (rk3026_ebc_flip_fini()); all without an error, as the core sends the
 * events of a CRTC that goes off. So a clean out-fence says the frame is
 * on the glass only when its part sent it: one sent by a sleep, the CRTC
 * off or the remove says no more than the core's own event would, and
 * its frame is drawn after the resume or the next enable, if at all.
 * @flip_held counts the events held.
 *
 * With debugfs flip_release "take" (@flip_release EBC_FLIP_REL_TAKE, read
 * once here into @flip_rel) the event is released at the take of its part
 * instead (ebc_flip_check()), when the panel starts its waveform: it says
 * the frame is the panel's now, not that it is on the glass. Its
 * out-fence then gets no error, also from the watchdog or a full slot;
 * only a part that finished off the glass before this flush (@commit_err)
 * still sets its error. A part taken before this flush, not finished yet,
 * is not held: the core sends its event at once; one finished too, its
 * result not yet recorded by the refresh thread, is held to the finish,
 * so that its error still reaches the out-fence. A change of the knob
 * reaches the next hold, never an event already held.
 *
 * Returns true if the event is held, taken from @crtc_state; false leaves it
 * there for the core.
 */
VISIBLE_IF_KUNIT bool rk3026_ebc_flip_hold(struct rk3026_ebc *ebc,
					   struct drm_crtc_state *crtc_state)
{
	struct drm_pending_vblank_event *event = crtc_state->event;
	enum rk3026_ebc_flip_rel rel = READ_ONCE(ebc->flip_release);
	unsigned long flags, timeout;
	bool held = false;

	if (!event)
		return false;

	/*
	 * Not held, and sent by the core's fake vblank at once:
	 * - internal events (no file and no fence): the core's own for
	 *   commits that asked for none (fbcon, fbdev writes, DirtyFB).
	 *   Holding them would make each of those wait for the panel.
	 * - a commit that queued no update: no damage, or an mxcfb client
	 *   owns the panel.
	 * - an update already finished: the refresh thread may finish a part
	 *   (skipped, dropped, or failed) before the commit gets here. The
	 *   part's error (@commit_err) goes on the out-fence the core
	 *   signals.
	 * - released at the take (EBC_FLIP_REL_TAKE), an update already
	 *   taken: the take moved @pending.taken_seq before it looked at the
	 *   slot under the lock, so a take after this look finds the event
	 *   held. No error: the part's end is not known yet. If it is
	 *   (@pending.done_seq moved over it, @flip_done_seq not yet), the
	 *   event is held to the finish instead: the thread's check right
	 *   after sends it with the part's error.
	 * - a commit during a system sleep (@suspended): no part is taken
	 *   until the resume. A nonblocking commit's tail runs on an unfrozen
	 *   worker and can get here after rk3026_ebc_sleep_begin(); so does
	 *   every commit while a resume that found no clocks left the device
	 *   suspended.
	 * - after the remove or the shutdown began (@flip_closed,
	 *   rk3026_ebc_flip_fini()): the watchdog must not be armed by a
	 *   commit tail that drm_atomic_helper_shutdown() did not wait for.
	 *
	 * Under @drm.event_lock: the close of the DRM file clears @file_priv
	 * there (drm_events_release()). The thread records a finished part
	 * (@flip_done_seq, @commit_err) under the lock, where it looks at the
	 * slot, so an event is either seen finished here, its part's result
	 * known, or seen held there; so is @suspended, set before the sleep's
	 * send takes the lock; @flip_closed is set under it.
	 */
	spin_lock_irqsave(&ebc->drm.event_lock, flags);
	if (!rk3026_ebc_flip_holds(event, ebc->commit_queued))
		goto out;
	if (rel == EBC_FLIP_REL_TAKE &&
	    rk3026_ebc_seq_done(READ_ONCE(ebc->pending.taken_seq),
				ebc->commit_seq)) {
		if (!rk3026_ebc_seq_done(READ_ONCE(ebc->pending.done_seq),
					 ebc->commit_seq))
			goto out;
		rel = EBC_FLIP_REL_DONE;
	}
	if (rk3026_ebc_seq_done(ebc->flip_done_seq, ebc->commit_seq)) {
		if (ebc->commit_err) {
			rk3026_ebc_flip_trace(ebc, ebc->commit_seq, 0,
					      EBC_FLIP_FAILED,
					      ebc->commit_err);
			if (event->base.fence)
				dma_fence_set_error(event->base.fence,
						    ebc->commit_err);
		}
	} else if (!ebc->flip_closed && !READ_ONCE(ebc->suspended)) {
		/*
		 * The core lets no second commit with an event through before
		 * the first one's flip_done, or not before its 10 s wait for it
		 * ends (unreachable with the watchdog); send the old one, not
		 * confirmed on the glass, rather than lose it.
		 */
		if (ebc->flip_event) {
			dev_warn_ratelimited(ebc->dev,
					     "flip event (seq %u) still held\n",
					     ebc->flip_seq);
			ebc_flip_send(ebc, EBC_FLIP_REPLACED, -ETIMEDOUT);
		}
		ebc->flip_event = event;
		ebc->flip_seq = ebc->commit_seq;
		ebc->flip_rel = rel;
		ebc->flip_held_at = rk3026_ebc_clock(ebc);
		WRITE_ONCE(ebc->flip_held, ebc->flip_held + 1);
		timeout = msecs_to_jiffies(min(READ_ONCE(ebc->flip_timeout_ms),
					       EBC_FLIP_TIMEOUT_MAX_MS));
		ebc->flip_due = jiffies + timeout;
		mod_delayed_work(system_dfl_wq, &ebc->flip_wd, timeout);
		held = true;
	}
out:
	spin_unlock_irqrestore(&ebc->drm.event_lock, flags);
	if (held)
		crtc_state->event = NULL;
	return held;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_flip_hold);

/* The commit's planes are applied: hold its event (rk3026_ebc_flip_hold()). */
static void ebc_crtc_atomic_flush(struct drm_crtc *crtc,
				  struct drm_atomic_state *state)
{
	rk3026_ebc_flip_hold(drm_to_ebc(crtc->dev),
			     drm_atomic_get_new_crtc_state(state, crtc));
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
	.atomic_begin		= ebc_crtc_atomic_begin,
	.atomic_flush		= ebc_crtc_atomic_flush,
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
 * One allocation, the two images adjacent: prev first, next at +img_size,
 * as the vendor kernel lays them out (its MST1 is MST0 + img_size on a
 * working refresh). Nothing shows that the controller needs them
 * contiguous; the layout matches a known-good configuration at no cost.
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

/*
 * The pads and the registers, with the clocks running: at probe, and at
 * resume in case the block lost them in the sleep.
 */
static void ebc_hw_setup(struct rk3026_ebc *ebc)
{
	/*
	 * Route the shared display pads to EBC, and put the LVDS block in the
	 * state the panel needs. All three bits are written, not just the pad
	 * select: see the GRF_EBC_SEL_* comment in rk3026_ebc.h.
	 */
	if (ebc->grf)
		regmap_write(ebc->grf, GRF_EBC_SEL_REG,
			     (GRF_EBC_SEL_MASK << 16) | GRF_EBC_SEL_MASK);

	ebc_program_regs(ebc);
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
	 * ACLK_EBC is a factor of aclk_vio1_niu in the clock driver, not a
	 * second gate on the same bit, so disabling it in ebc_hw_fini()
	 * does not close the bit under the NIU.
	 */
	if (ebc->mclk) {
		ret = clk_prepare_enable(ebc->mclk);
		if (ret)
			goto err_aclk;
	}

	/*
	 * dclk boots muxed to CPLL, which sits in slow mode at 24 MHz: 12 MHz
	 * after the divider, against the panel's 20 MHz source clock, and a
	 * 54-frame scan then takes 4260 ms instead of 790.
	 *
	 * Parent first, then rate. clk_set_rate() may satisfy a request from
	 * either parent, so a rate can be answered with CPLL and a divider;
	 * reparenting to GPLL afterwards keeps that divider and lands on
	 * another rate, both calls returning success.
	 *
	 * Check the rate against the frame time (the scan_end tracepoint's
	 * ms) rather than clk_get_rate(), which reports the rate the clock
	 * framework believes, not a mux the hardware left elsewhere: 38
	 * frames take ~3000 ms at 12 MHz and ~560 ms at 66.
	 */
	if (ebc->dclk_parent) {
		ret = clk_set_parent(ebc->dclk, ebc->dclk_parent);
		if (ret)
			goto err_mclk;
	}

	/*
	 * The rate comes from the DEVICE TREE, not a compiled-in constant: it is
	 * a property of this panel and this board. of_get_display_timing() has
	 * already parsed `panel-timing { clock-frequency }` into timing.pixelclock;
	 * EBC_PIXEL_RATE_FALLBACK is only for a DT without it.
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

	ebc->clks_on = true;
	ebc_hw_setup(ebc);
	return 0;

err_mclk:
	clk_disable_unprepare(ebc->mclk);
err_aclk:
	clk_disable_unprepare(ebc->aclk);
err_hclk:
	clk_disable_unprepare(ebc->hclk);
	return ret;
}

static void ebc_clks_off(struct rk3026_ebc *ebc)
{
	clk_disable_unprepare(ebc->dclk);
	clk_disable_unprepare(ebc->mclk);
	clk_disable_unprepare(ebc->aclk);
	clk_disable_unprepare(ebc->hclk);
	ebc->clks_on = false;
}

/*
 * The clocks again after ebc_clks_off(), in ebc_hw_init()'s order; dclk
 * keeps the parent and rate it was given there.
 */
static int ebc_clks_on(struct rk3026_ebc *ebc)
{
	int ret;

	ret = clk_prepare_enable(ebc->hclk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(ebc->aclk);
	if (ret)
		goto err_hclk;
	ret = clk_prepare_enable(ebc->mclk);
	if (ret)
		goto err_aclk;
	ret = clk_prepare_enable(ebc->dclk);
	if (ret)
		goto err_mclk;
	ebc->clks_on = true;
	return 0;

err_mclk:
	clk_disable_unprepare(ebc->mclk);
err_aclk:
	clk_disable_unprepare(ebc->aclk);
err_hclk:
	clk_disable_unprepare(ebc->hclk);
	return ret;
}

/* Off by now if the device was unbound asleep, after a resume without them. */
static void ebc_hw_fini(void *data)
{
	struct rk3026_ebc *ebc = data;

	if (ebc->clks_on)
		ebc_clks_off(ebc);
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
	rk3026_ebc_flip_init(ebc);
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
	/* A burst of updates is taken once it has been quiet this long. */
	ebc->take_quiet_us = EBC_TAKE_QUIET_US;
	ebc->take_cap_ms = EBC_TAKE_CAP_MS;
	/* Every part in its own mode. */
	ebc->force_mode = -1;
	/* The rails stay up this long after a waveform. */
	ebc->rails_off_ms = EBC_RAILS_OFF_MS;

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
	 * the boot -- silently, behind one dev_warn, so a wrong LUT would never
	 * be seen in a log.
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
	 *   WARN_ON(link->status != DL_STATE_CONSUMER_PROBE) the moment this
	 *   driver binds. Do not add such a link.
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
	/*
	 * The CRTC disable's commit waited for a held event, or sent it. A
	 * shutdown that failed before the disable left a held event, and a
	 * commit tail it did not wait for may still offer one.
	 */
	rk3026_ebc_flip_fini(ebc);
}

static void ebc_shutdown(struct platform_device *pdev)
{
	struct rk3026_ebc *ebc = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&ebc->drm);
	/* As at remove: no event held, no watchdog left armed. */
	rk3026_ebc_flip_fini(ebc);

	/* Last chance for a rails-off that failed at CRTC disable. */
	mutex_lock(&ebc->rails_lock);
	ebc_rails_off(ebc);
	mutex_unlock(&ebc->rails_lock);
}

/*
 * System sleep keeps the CRTC, the plane and the images as they are: the
 * glass holds its picture without power, and the first part after the
 * resume is driven from @shadows.prev like any other. Hence no
 * drm_mode_config_helper_suspend(): its CRTC disable would drop the parts
 * pending, and the enable after the resume would RESET the panel to white.
 *
 * The suspend stops the takes (rk3026_ebc_sleep_begin()), waits for the
 * part in flight, so a waveform always runs to its end, parks the refresh
 * thread between two passes, and switches off the panel rails, then the
 * interrupt and the clocks. V3P3, the panel's logic rail, stays with the
 * CRTC, as when the rails go off between parts. Updates sent while
 * suspended stay pending and are drawn after the resume. A part that does
 * not end in time, or rails that do not go off, fail the suspend: neither a
 * cut waveform nor a sleep with the rails up.
 */
static int ebc_suspend(struct device *dev)
{
	struct rk3026_ebc *ebc = dev_get_drvdata(dev);
	unsigned int ms = rk3026_ebc_part_max_ms(ebc);
	int ret;

	/* Still asleep: the last resume found no clocks. */
	if (ebc->suspended)
		return 0;

	rk3026_ebc_sleep_begin(ebc);
	ret = rk3026_ebc_sleep_wait(ebc, ms);
	if (ret) {
		dev_err(dev, "a part still in flight after %u ms\n", ms);
		goto err_end;
	}
	/*
	 * No part in flight and none to be taken: the thread parks at the
	 * end of its pass, at most a rails ramp and a sensor read away.
	 */
	kthread_park(ebc->refresh_thread);

	mutex_lock(&ebc->rails_lock);
	ret = ebc_rails_off(ebc);
	mutex_unlock(&ebc->rails_lock);
	if (ret)
		goto err_unpark;

	disable_irq(ebc->irq);
	ebc_clks_off(ebc);
	return 0;

err_unpark:
	kthread_unpark(ebc->refresh_thread);
err_end:
	rk3026_ebc_sleep_end(ebc);
	return ret;
}

/*
 * Undo ebc_suspend(). The registers are written again in case the block
 * lost them; the rails come up for the next part that is driven. Without
 * the clocks nothing is touched: the driver stays suspended, and the next
 * resume tries again. Meanwhile a CRTC enable or disable does not wait for
 * the thread (rk3026_ebc_crtc_wait()), and an unbind leaves the clocks
 * alone (ebc_hw_fini()).
 *
 * Also the thaw of a hibernation, after the image is written: the same
 * kernel, the glass as it was left.
 */
static int ebc_resume(struct device *dev)
{
	struct rk3026_ebc *ebc = dev_get_drvdata(dev);
	int ret;

	ret = ebc_clks_on(ebc);
	if (ret) {
		dev_err(dev, "failed to enable the clocks: %d\n", ret);
		return ret;
	}
	ebc_hw_setup(ebc);
	enable_irq(ebc->irq);
	rk3026_ebc_sleep_end(ebc);
	kthread_unpark(ebc->refresh_thread);
	return 0;
}

/*
 * A hibernation restore: a resume whose glass may have been drawn on by the
 * kernel that loaded the image, so the first part repairs it and the
 * whole panel is redrawn (rk3026_ebc_sleep_restore()).
 */
static int ebc_restore(struct device *dev)
{
	rk3026_ebc_sleep_restore(dev_get_drvdata(dev));
	return ebc_resume(dev);
}

/*
 * Hibernation: freeze and poweroff are a suspend, thaw a resume; only the
 * restore does not trust the glass.
 */
static const struct dev_pm_ops ebc_pm_ops = {
	.suspend	= pm_sleep_ptr(ebc_suspend),
	.resume		= pm_sleep_ptr(ebc_resume),
	.freeze		= pm_sleep_ptr(ebc_suspend),
	.thaw		= pm_sleep_ptr(ebc_resume),
	.poweroff	= pm_sleep_ptr(ebc_suspend),
	.restore	= pm_sleep_ptr(ebc_restore),
};

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
		.pm		= pm_sleep_ptr(&ebc_pm_ops),
	},
};
module_platform_driver(ebc_driver);

MODULE_DESCRIPTION("Rockchip RK3026 EBC E-Ink DRM driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(EBC_WAVEFORM_FW);
