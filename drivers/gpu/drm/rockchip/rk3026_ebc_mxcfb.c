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
 *   AUTO       PART          FULL          gray16 LUT
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
 * with DISPLAY_MODE clear, which is a different data path entirely. Treating
 * AUTO as GC16 keeps every update on the LUT path.
 */
VISIBLE_IF_KUNIT u32 rk3026_ebc_mxcfb_to_ebc_mode(u32 waveform_mode,
						  u32 update_mode)
{
	bool full = update_mode == MXCFB_UPDATE_MODE_FULL;

	switch (waveform_mode) {
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
	case MXCFB_WAVEFORM_MODE_AUTO:
	default:
		return full ? EBC_MODE_FULL : EBC_MODE_PART;
	}
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_mxcfb_to_ebc_mode);

static int mxcfb_send_update(struct rk3026_ebc *ebc, struct fb_info *info,
			     unsigned long arg)
{
	struct rk3026_ebc_update upd;
	struct mxcfb_update_data req;
	struct drm_rect clip;

	if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
		return -EFAULT;

	clip.x1 = min_t(u32, req.update_region.left, ebc->width);
	clip.y1 = min_t(u32, req.update_region.top, ebc->height);
	clip.x2 = min_t(u32, clip.x1 + req.update_region.width, ebc->width);
	clip.y2 = min_t(u32, clip.y1 + req.update_region.height, ebc->height);
	if (clip.x1 >= clip.x2 || clip.y1 >= clip.y2)
		return -EINVAL;

	upd = (struct rk3026_ebc_update) {
		.clip		= clip,
		.mode		= rk3026_ebc_mxcfb_to_ebc_mode(req.waveform_mode,
							       req.update_mode),
		.marker		= req.update_marker,
		.temp		= req.temp,
		.temp_valid	= req.temp != MXCFB_TEMP_USE_AMBIENT &&
				  req.temp > -128 && req.temp < 128,
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
		rk3026_ebc_blit_xrgb8888(ebc, info->screen_buffer,
					 info->fix.line_length, &clip);
	rk3026_ebc_add_update(ebc, &upd);
	mutex_unlock(&ebc->image_lock);
	return 0;
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
 */
void rk3026_ebc_mxcfb_install(struct rk3026_ebc *ebc, struct fb_info *info)
{
	struct fb_ops *ops;

	if (!info || !info->fbops)
		return;

	ops = devm_kmemdup(ebc->dev, info->fbops, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return;

	ebc->orig_fbops = info->fbops;
	ops->fb_ioctl = ebc_fb_ioctl;
	ops->fb_open = ebc_fb_open;
	ops->fb_release = ebc_fb_release;
	ebc->fbops = ops;
	info->fbops = ops;

	drm_info(&ebc->drm, "mxcfb V1 ioctl shim on fb%d\n", info->node);
}
