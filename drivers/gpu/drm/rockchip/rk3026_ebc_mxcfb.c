// SPDX-License-Identifier: GPL-2.0
/*
 * mxcfb ioctl shim over the RK3026 EBC's fbdev emulation.
 *
 * DRM fbdev emulation delivers damage rectangles but has no way to express a
 * waveform mode, a temperature, an update marker or a completion wait. Every
 * shipping e-reader userspace - KOReader included - talks the i.MX EPDC ioctl
 * dialect instead, so this file bolts that dialect onto the emulated fbdev and
 * translates it into the driver's internal update API.
 *
 * It is deliberately a leaf: nothing in the DRM driver depends on it, and
 * building without it leaves a working single-mode partial-refresh display.
 *
 * Copyright (C) 2026
 */

#include <linux/fb.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <drm/drm_fb_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_print.h>
#include <drm/drm_rect.h>

#include <kunit/visibility.h>

#include "rk3026_ebc.h"
#include "rk3026_ebc_mxcfb.h"

/*
 * Waveform mode mapping.
 *
 * The rkf container this panel ships carries five LUT families: reset,
 * gray16, "mode 6", gray2 and grayA2. The EBC work modes below select them
 * through the controller's own dispatch, which also decides whether the
 * refresh is full-class or partial-class. The work mode (EBC_MODE_*) by
 * update_mode:
 *
 *   waveform   PARTIAL       FULL
 *   INIT       RESET         RESET         reset LUT, full class
 *   DU         BLACK_WHITE   BLACK_WHITE   gray2 LUT, partial class
 *   A2         A2            A2            grayA2 LUT, partial class
 *   GC16       PART          FULL          gray16 LUT
 *   GL16       PART          OED_PART      gray16 LUT / "mode 6" LUT
 *   REAGL      PART          FULL          gray16 LUT
 *   REAGLD     FULL          FULL          gray16 LUT, full class
 *   GC4        PART          PART          gray16 LUT; no 4-level LUT
 *   AUTO       PART, auto    FULL          gray16 LUT, or gray2 (below)
 *   (other)    PART          FULL          gray16 LUT
 *
 * The "mode 6" family is the container's 16-level no-flash table: it drives
 * like gray16 except white to white, which is what i.MX calls GL16, so a
 * GL16 full update gets it. EBC_MODE_OED_PART is a full-class work mode
 * (every pixel driven, unchanged ones too), so a GL16 partial update stays
 * on the gray16 partial path, which drives only the pixels that change.
 * REAGL is i.MX's ghost-reducing 16-level mode; this panel has no table for
 * it, so it takes the gray16 family by update_mode, like GC16.
 *
 * GC4 and REAGLD have no distinct waveform on this panel and are folded into
 * the gray16 family.
 *
 * AUTO is deliberately not routed to EBC_MODE_AUTO: that work mode is one of
 * the controller's direct modes, driven line by line with no LUT upload and
 * with DISPLAY_MODE clear, which is a different data path entirely. AUTO
 * stays on the LUT path. A partial AUTO update is PART with @auto_mode set:
 * the core drives it in DU when every pixel it changes becomes black or
 * white, else in PART (rk3026_ebc_resolve_mode()). This file only sets the
 * flag; the content test is the snapshot's. A full AUTO update is a plain
 * FULL: the client asked for the flash.
 *
 * EPDC_FLAG_FORCE_MONOCHROME: i.MX thresholds the update to black and white
 * before it picks the waveform, so AUTO with the flag is DU, treated as an
 * explicit mode (no @auto_mode), for a full update as for a partial one: DU
 * drives only the pixels that change either way. Such a full update is not
 * full class, so an identical one is skipped; an identical DU moves nothing
 * on the glass, so the skip is exact. With DU and A2 the flag needs no
 * code: the snapshot lays every changed pixel of a two-level mode over the
 * old image as black or white already (grey >= 8 is white), and unchanged
 * pixels stay, which DU and A2 would not drive anyway. With a grey waveform
 * it is not honoured. The client's image in the shadow is never
 * thresholded, so a later grey update of the same pixels restores them.
 */
VISIBLE_IF_KUNIT u32 rk3026_ebc_mxcfb_to_ebc_mode(u32 waveform_mode,
						  u32 update_mode, u32 flags,
						  bool *auto_mode)
{
	bool full = update_mode == MXCFB_UPDATE_MODE_FULL;

	*auto_mode = false;

	switch (waveform_mode) {
	case MXCFB_WAVEFORM_MODE_AUTO:
		if (flags & EPDC_FLAG_FORCE_MONOCHROME)
			return EBC_MODE_BLACK_WHITE;
		*auto_mode = !full;
		return full ? EBC_MODE_FULL : EBC_MODE_PART;
	case MXCFB_WAVEFORM_MODE_INIT:
		return EBC_MODE_RESET;
	case MXCFB_WAVEFORM_MODE_DU:
		return EBC_MODE_BLACK_WHITE;
	case MXCFB_WAVEFORM_MODE_A2:
		return EBC_MODE_A2;
	case MXCFB_WAVEFORM_MODE_GL16:
		return full ? EBC_MODE_OED_PART : EBC_MODE_PART;
	case MXCFB_WAVEFORM_MODE_REAGLD:
		return EBC_MODE_FULL;
	case MXCFB_WAVEFORM_MODE_GC4:
		return EBC_MODE_PART;
	case MXCFB_WAVEFORM_MODE_GC16:
	case MXCFB_WAVEFORM_MODE_REAGL:
	default:
		return full ? EBC_MODE_FULL : EBC_MODE_PART;
	}
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_mxcfb_to_ebc_mode);

/*
 * Dithering: EPDC_FLAG_USE_DITHERING_Y4, _Y1 and _NTX_D8.
 *
 * An i.MX EPDC client sets these to have 8-bit content dithered to what
 * the panel can show: Y4 to the 16 grey levels, Y1 to black and white.
 * NTX_D8, a dither request of the Netronix-based readers' kernels (Kobo,
 * PocketBook, Cervantes), is taken as Y4: whatever number of levels it
 * asks for, the panel's 16 are the finest dither it can show. The blit
 * honours them on the update region with an ordered dither of the luma, in
 * an 8x8 pattern anchored to the panel (rk3026_ebc_blit_dither()), on the
 * region widened to whole bytes (a pixel next to an odd edge takes this
 * update's conversion): Y4 draws a grey with the two levels around it, Y1
 * with black and white, in the proportion that keeps its mean. The grey of
 * a level itself (17 n for Y4; 0 and 255 for Y1) comes out flat, as
 * without the flag, so black and white text and the 16 levels convert as
 * before. Set together, Y1 wins over Y4 and NTX_D8, and Y4 with NTX_D8 is
 * Y4.
 *
 * The flags choose the conversion only, never the waveform: the mode
 * comes from waveform_mode, update_mode and FORCE_MONOCHROME as above. Y1
 * is meant for the two-level waveforms, DU, A2 and AUTO with
 * FORCE_MONOCHROME: every pixel it writes is black or white, so the
 * snapshot's two-level step (grey >= 8 is white) keeps the pattern as it
 * is, and a partial AUTO update of it may resolve to DU. Y1 with
 * FORCE_MONOCHROME is thus a dithered instead of a thresholded black and
 * white, in the mode the flag maps to. With a 16-level waveform Y1 draws
 * the same black and white pattern. Y4 (or NTX_D8) with a two-level
 * waveform is thresholded at the snapshot like any grey; a client that
 * wants a dithered two-level update asks for Y1.
 *
 * Without any of the three the conversion is the plain one, byte for
 * byte. A RESET converts nothing, so its flags have no effect. The plane
 * path (fbcon, a KMS client) has no flags and never dithers.
 */
VISIBLE_IF_KUNIT enum rk3026_ebc_dither rk3026_ebc_mxcfb_dither(u32 flags)
{
	if (flags & EPDC_FLAG_USE_DITHERING_Y1)
		return EBC_DITHER_Y1;
	if (flags & (EPDC_FLAG_USE_DITHERING_Y4 |
		     EPDC_FLAG_USE_DITHERING_NTX_D8))
		return EBC_DITHER_Y4;
	return EBC_DITHER_NONE;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_mxcfb_dither);

/*
 * One MXCFB_SEND_UPDATE request, already copied in: the region clamped to
 * the panel, the waveform and the flags mapped
 * (rk3026_ebc_mxcfb_to_ebc_mode(), rk3026_ebc_mxcfb_dither()), and, in one
 * @image_lock section, the client's pixels converted from @src, its
 * framebuffer with @pitch bytes per row, and the update queued.
 *
 * Returns 0, -EINVAL if nothing of the region is on the panel, or -EBUSY
 * with the CRTC off.
 */
VISIBLE_IF_KUNIT int rk3026_ebc_mxcfb_send(struct rk3026_ebc *ebc,
					   const void *src, unsigned int pitch,
					   const struct mxcfb_update_data *req)
{
	enum rk3026_ebc_dither dither;
	struct rk3026_ebc_update upd;
	struct drm_rect clip;
	bool auto_mode;
	u32 mode;

	clip.x1 = min_t(u32, req->update_region.left, ebc->width);
	clip.y1 = min_t(u32, req->update_region.top, ebc->height);
	clip.x2 = min_t(u32, clip.x1 + req->update_region.width, ebc->width);
	clip.y2 = min_t(u32, clip.y1 + req->update_region.height, ebc->height);
	if (clip.x1 >= clip.x2 || clip.y1 >= clip.y2)
		return -EINVAL;

	mode = rk3026_ebc_mxcfb_to_ebc_mode(req->waveform_mode,
					    req->update_mode, req->flags,
					    &auto_mode);
	dither = rk3026_ebc_mxcfb_dither(req->flags);
	upd = (struct rk3026_ebc_update) {
		.clip		= clip,
		.mode		= mode,
		.auto_mode	= auto_mode,
		.marker		= req->update_marker,
		.temp		= req->temp,
		.temp_valid	= req->temp != MXCFB_TEMP_USE_AMBIENT &&
				  req->temp > -128 && req->temp < 128,
		.source		= EBC_SOURCE_MXCFB,
	};

	/* One section: no snapshot sees these pixels without their update. */
	mutex_lock(&ebc->image_lock);

	/*
	 * With the CRTC off the panel is not driven (blanked fbdev, shutdown):
	 * refuse rather than queue a waveform that would not run. The refresh
	 * thread drops anything it takes while the CRTC is off as well; this
	 * is the early answer to the client. Checked in the section that adds
	 * the update, so a send either sees the CRTC on and goes behind the
	 * RESET of its enable, or is refused.
	 */
	if (!ebc->crtc_on) {
		mutex_unlock(&ebc->image_lock);
		return -EBUSY;
	}

	/*
	 * From here on this client owns the pixels: it writes them through the
	 * mmap and tells us when they are ready, so the deferred-IO path must
	 * not refresh the same region again with the default waveform. Set in
	 * this section, where the plane update checks it
	 * (rk3026_ebc_plane_update()).
	 */
	WRITE_ONCE(ebc->mxcfb_active, true);

	/* A RESET discards the pixels: rk3026_ebc_add_update() whitens them. */
	if (upd.mode != EBC_MODE_RESET)
		upd.blit_us = rk3026_ebc_blit_xrgb8888(ebc, src, pitch, &clip,
						       dither);
	rk3026_ebc_add_update(ebc, &upd);
	mutex_unlock(&ebc->image_lock);
	return 0;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_mxcfb_send);

static int mxcfb_send_update(struct rk3026_ebc *ebc, struct fb_info *info,
			     unsigned long arg)
{
	struct mxcfb_update_data req;

	if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
		return -EFAULT;

	return rk3026_ebc_mxcfb_send(ebc, info->screen_buffer,
				     info->fix.line_length, &req);
}

/*
 * The fbdev core calls fb_ioctl under lock_fb_info(), so a client blocked
 * here (up to rk3026_ebc_wait_timeout_ms() when the refresh thread is stuck)
 * holds the fb_info lock: a send, FBIOPAN_DISPLAY or FBIOBLANK from another
 * thread of the same or another process waits for it. So a send waits for
 * no waveform only as long as a single thread both sends and waits, as
 * e-reader clients do.
 */
static int mxcfb_wait_for_update(struct rk3026_ebc *ebc, unsigned long arg)
{
	u32 marker;

	if (copy_from_user(&marker, (void __user *)arg, sizeof(marker)))
		return -EFAULT;

	return rk3026_ebc_wait_marker(ebc, marker);
}

static int ebc_fb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg)
{
	struct drm_fb_helper *helper = info->par;
	struct rk3026_ebc *ebc = drm_to_ebc(helper->dev);

	switch (cmd) {
	case MXCFB_SEND_UPDATE:
		return mxcfb_send_update(ebc, info, arg);
	case MXCFB_WAIT_FOR_UPDATE_COMPLETE:
		return mxcfb_wait_for_update(ebc, arg);
	}

	if (ebc->orig_fbops->fb_ioctl)
		return ebc->orig_fbops->fb_ioctl(info, cmd, arg);
	return -ENOTTY;
}

static int ebc_fb_open(struct fb_info *info, int user)
{
	struct drm_fb_helper *helper = info->par;
	struct rk3026_ebc *ebc = drm_to_ebc(helper->dev);
	int ret = 0;

	if (ebc->orig_fbops->fb_open)
		ret = ebc->orig_fbops->fb_open(info, user);
	if (!ret && user)
		atomic_inc(&ebc->fb_users);
	return ret;
}

static int ebc_fb_release(struct fb_info *info, int user)
{
	struct drm_fb_helper *helper = info->par;
	struct rk3026_ebc *ebc = drm_to_ebc(helper->dev);

	/* Hand the panel back to fbcon once the last mxcfb client is gone. */
	if (user && atomic_dec_and_test(&ebc->fb_users)) {
		mutex_lock(&ebc->image_lock);
		WRITE_ONCE(ebc->mxcfb_active, false);
		mutex_unlock(&ebc->image_lock);
	}

	if (ebc->orig_fbops->fb_release)
		return ebc->orig_fbops->fb_release(info, user);
	return 0;
}

/**
 * rk3026_ebc_mxcfb_install - graft the mxcfb ioctls onto an emulated fbdev
 * @ebc: controller
 * @info: fb_info the DRM fbdev client has just set up
 *
 * The generic fb_ops is const and shared, so it is copied once and the three
 * entry points this shim needs are replaced in the copy.
 *
 * The copy is drm-managed, not devres-managed: an fb fd open across an
 * unbind still reaches it from fb_release() and from the fb_destroy() of the
 * last put_fb_info(). The fbdev client holds a reference on the drm_device
 * until that fb_destroy() releases it, so the copy outlives every call made
 * through it.
 */
void rk3026_ebc_mxcfb_install(struct rk3026_ebc *ebc, struct fb_info *info)
{
	struct fb_ops *ops;

	if (!info || !info->fbops)
		return;

	ops = drmm_kmalloc(&ebc->drm, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return;
	*ops = *info->fbops;

	ebc->orig_fbops = info->fbops;
	ops->fb_ioctl = ebc_fb_ioctl;
	ops->fb_open = ebc_fb_open;
	ops->fb_release = ebc_fb_release;
	ebc->fbops = ops;
	info->fbops = ops;

	drm_info(&ebc->drm, "mxcfb V1 ioctl shim on fb%d\n", info->node);
}
