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
 * One update as a sender hands it over (rk3026_ebc_add_update()): its region,
 * EBC work mode, MXCFB_WAIT_FOR_UPDATE_COMPLETE marker (0: none), the panel
 * temperature it brings (@temp, if @temp_valid) and who sent it.
 */
struct rk3026_ebc_update {
	struct drm_rect	clip;
	u32		mode;
	u32		marker;
	int		temp;
	bool		temp_valid;
	u8		source;		/* enum rk3026_ebc_source */
};

/*
 * The record of one part, for debugfs. Filled by the refresh thread and
 * published under @image_lock.
 */
struct rk3026_ebc_last {
	struct drm_rect	clip;
	u32		asked_mode;	/* the part's own mode */
	u32		mode;		/* the DSP_CTRL class driven (force_mode) */
	u32		lut_mode;	/* the mode whose LUT was driven */
	u32		marker;		/* the newest update's */
	u32		merged;		/* updates in the part */
	int		temp;		/* clamped; valid if @temp_read */
	bool		temp_read;
	bool		repaired;	/* a RESET repair ran before it */
	u8		source;		/* enum rk3026_ebc_source */
	u8		outcome;	/* enum rk3026_ebc_outcome */
	u32		frames;		/* frames in its (last) waveform */
	u32		driven;		/* of those, driven */
	u32		scans;		/* waveforms started, repair included */
	u32		timeouts;
	u32		ms;		/* take to done */
	u32		scan_ms;	/* the last waveform */
};

/*
 * Counters since probe, under @image_lock. @skips, @dropped and @done count
 * parts; @updates less @coalesced is the parts made, @done plus those
 * pending, except where a RESET absorbed a pending set behind another RESET
 * (rk3026_ebc_pending_add()): that set's first update made a part that is
 * gone.
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
 */
struct rk3026_ebc_set {
	struct drm_rect	box;
	u32		mode;
	u32		seq;		/* the newest update's */
	u32		marker;		/* the newest update's, or 0 */
	u32		merged;		/* updates in the set */
	int		temp;		/* the newest valid one */
	bool		temp_valid;
	/* an MXCFB_SEND_UPDATE in a full-class mode: never skipped */
	bool		has_mxcfb_full;
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
 * or a failed repair). It is merged into the next @pre that is taken and is
 * not work on its own, so a failing rail cannot spin the thread. A RESET
 * sent drops it: that RESET whitens the glass it would draw. Present while
 * its @merged is 1 (a carry counts no updates).
 *
 * @queued_seq is the seq of the newest update and @done_seq that of the newest
 * finished one; both wrap and compare with rk3026_ebc_seq_done(). Parts are
 * taken in seq order and finished one at a time, so once the part taken
 * last has finished, @done_seq equals @queued_seq exactly when nothing is
 * pending. A carry has no seq of its own.
 */
struct rk3026_ebc_pending {
	struct rk3026_ebc_set	pre;
	struct rk3026_ebc_set	reset;
	struct rk3026_ebc_set	post;
	struct rk3026_ebc_set	carry;
	u32			queued_seq;
	u32			done_seq;
};

/*
 * The CPU-side images, packed 4bpp like the scanned ones (@pitch bytes per
 * line, @height lines): what the glass shows, and the snapshot a scan drives.
 * @prev_unknown says @prev is not evidence (a waveform did not end).
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

/* The update queued with @seq has finished once @done_seq reached it. */
static inline bool rk3026_ebc_seq_done(u32 done_seq, u32 seq)
{
	return (s32)(done_seq - seq) >= 0;
}

/* Nothing for the refresh thread to take; a carry alone is not work. */
static inline bool rk3026_ebc_pending_empty(const struct rk3026_ebc_pending *p)
{
	return !p->pre.merged && !p->reset.merged;
}

/*
 * A part is pending or in flight, read without the lock. Parts are taken in
 * seq order and @done_seq moves only when one finishes, so from the take of
 * a part to its finish @done_seq is behind @queued_seq; between parts the two
 * differ exactly when a part is pending (!rk3026_ebc_pending_empty()). A
 * carry has no seq: on its own it wakes nothing.
 */
static inline bool rk3026_ebc_work_pending(const struct rk3026_ebc_pending *p)
{
	return READ_ONCE(p->queued_seq) != READ_ONCE(p->done_seq);
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
	 * @shadows.prev_unknown: @shadows.prev does not describe the glass.
	 * Set when a waveform starts, cleared ONLY when every frame has been
	 * driven, so a timeout, an error path or a path nobody has written yet
	 * all leave it set by construction. ⚠️ Do not turn this into a list of
	 * clears on the known abort paths: the one that matters is the path not
	 * yet written. Read at the next draw, which then clears the panel first.
	 * Written by the refresh thread alone: ebc_drive() owns it during a
	 * waveform, and rk3026_ebc_scan_finish() records the part's outcome
	 * under @image_lock, never clearing what ebc_drive() left set.
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
	struct rk3026_ebc_shadows shadows;
	struct rk3026_ebc_pending pending;
	struct rk3026_ebc_marker_ring markers;

	/* Write-only registers the controller does not read back. */
	u32			dsp_start;
	u32			dsp_ctrl;
	u32			int_reg;

	struct completion	frame_done;

	/*
	 * Refresh thread. A send wakes @update_wq; every finished part
	 * (@pending.done_seq moved, whatever the outcome) wakes @marker_wq.
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

	const struct firmware	*fw;
	struct ebc_waveform	*waveform;

	/* mxcfb shim state; unused when the shim is not built. */
	struct fb_ops		*fbops;
	const struct fb_ops	*orig_fbops;
	atomic_t		fb_users;
	/* Written under @image_lock; debugfs reads it without. */
	bool			mxcfb_active;

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
void rk3026_ebc_blit_xrgb8888(struct rk3026_ebc *ebc, const void *src,
			      unsigned int src_pitch, const struct drm_rect *clip);
u32 rk3026_ebc_add_update(struct rk3026_ebc *ebc,
			  const struct rk3026_ebc_update *upd);
int rk3026_ebc_wait_marker(struct rk3026_ebc *ebc, u32 marker);

/* rk3026_ebc_core.c */
u32 rk3026_ebc_dsp_ctrl_mode(u32 mode);
bool rk3026_ebc_pending_add(struct rk3026_ebc_pending *p,
			    const struct rk3026_ebc_update *upd, u32 *seq);
bool rk3026_ebc_pending_take(struct rk3026_ebc_pending *p,
			     struct rk3026_ebc_set *part);
bool rk3026_ebc_build_snapshot(const struct rk3026_ebc_shadows *sh,
			       const u8 *next, const struct drm_rect *box,
			       u32 *mode);
bool rk3026_ebc_should_skip(const struct rk3026_ebc_set *part,
			    bool prev_unknown, bool identical);
u32 rk3026_ebc_scan_mode(const struct rk3026_ebc_set *part, bool repaired);
bool rk3026_ebc_scan_finish(struct rk3026_ebc_shadows *sh,
			    struct rk3026_ebc_pending *p,
			    const struct rk3026_ebc_set *part,
			    enum rk3026_ebc_outcome outcome,
			    enum rk3026_ebc_repair repair);
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
u32 rk3026_ebc_mxcfb_to_ebc_mode(u32 waveform_mode, u32 update_mode);
#endif
#else
static inline void rk3026_ebc_mxcfb_install(struct rk3026_ebc *ebc,
					    struct fb_info *info) { }
#endif

#endif /* _RK3026_EBC_H */
