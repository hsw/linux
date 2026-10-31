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
 * building without it leaves a working display whose updates all take the
 * plane path, with the waveform picked from the content.
 *
 * Copyright (C) 2026
 */

#include <linux/fb.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
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
 * EPDC_FLAG_ENABLE_INVERSION on @img, a packed 4bpp image of @width x
 * @height pixels: every level of @clip, widened to whole bytes and clamped
 * as rk3026_ebc_blit() widens it, becomes 15 - level. Flipping every bit
 * of a byte does that for both of its pixels.
 */
VISIBLE_IF_KUNIT void rk3026_ebc_mxcfb_invert(u8 *img,
					      const struct drm_rect *clip,
					      unsigned int width,
					      unsigned int height)
{
	struct drm_rect r = *clip;
	int y;

	if (!rk3026_ebc_widen_clip(&r, width, height))
		return;

	for (y = r.y1; y < r.y2; y++) {
		u8 *d = img + ((size_t)y * width + r.x1) / 2;
		int i, n = (r.x2 - r.x1) / 2;

		for (i = 0; i < n; i++)
			d[i] = ~d[i];
	}
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_mxcfb_invert);

/**
 * rk3026_ebc_mxcfb_send - one MXCFB_SEND_UPDATE request
 * @ebc: controller
 * @src: the fbdev's framebuffer, top-left
 * @pitch: its bytes per row
 * @format: its format, XRGB8888 or C8
 * @req: the request, already copied in
 *
 * The arguments are checked, the region clamped to the panel, the waveform
 * and the flags mapped (rk3026_ebc_mxcfb_to_ebc_mode()), and, in one
 * @image_lock section, the client's pixels converted from @src and the
 * update queued.
 *
 * The checks are i.MX's: an update_mode other than PARTIAL or FULL and a
 * waveform number above MXCFB_WAVEFORM_MODE_MAX other than AUTO are
 * invalid. Two flags change which image is meant and cannot be honoured,
 * so the update is refused rather than drawn without them:
 * EPDC_FLAG_USE_ALT_BUFFER names a physical address, and the shmem fbdev
 * has none a client could name; EPDC_FLAG_TEST_COLLISION asks for a dry
 * run. Other flags this shim does not read are ignored, as on i.MX.
 *
 * A C8 byte has two meanings on this driver. Through the fbdev and the
 * plane (the console, a KMS client) it is an index into the CRTC's
 * GAMMA_LUT, the palette the console loads with FBIOPUTCMAP. Here it is a
 * luminance, 0 black to 255 white, as on an i.MX EPDC 8-bit fbdev, which
 * is what an mxcfb client writes: it converts as R8, rounded to the
 * nearest of the 16 levels, whatever palette is set, so a client that
 * dithers to the greys 17 k gets level k exactly and needs no palette.
 *
 * The plane still reads the same buffer as indices whenever it blits it:
 * the fbdev's damage of an mmap write before the client's first send,
 * and the whole-panel redraw (@plane_resync) of the first plane update
 * after the last client has closed the fbdev, the console's next write
 * for one; the close itself redraws nothing. With the console's palette
 * set, the greys 1 to 15 a client leaves there then show as the
 * console's colours, not as dark greys, until the console redraws them.
 *
 * EPDC_FLAG_ENABLE_INVERSION draws the region inverted, every level as
 * 15 - level: the converted region is inverted after the blit, in the same
 * section (rk3026_ebc_mxcfb_invert()), the levels of a C8 byte taken as
 * grey as well as those of an XRGB8888 pixel. The region is the blit's,
 * widened to whole bytes, so the other pixel of an edge byte, which the
 * blit converts from the framebuffer as well, is inverted with it. The
 * target image then holds what the glass is to show, not what the
 * framebuffer holds: the snapshot's change test, the DU test of AUTO and
 * the threshold of a two-level mode see the image as drawn, and a later
 * send of the same pixels without the flag draws them as the framebuffer
 * has them. The threshold (level >= 8 white) of an inverted level is the
 * inverse of the threshold of the level, so FORCE_MONOCHROME and the
 * inversion may come in either order. INIT converts nothing and whitens
 * the region, with the flag as without it. The plane path never inverts:
 * once the panel is the plane's again, its next update redraws the whole
 * panel from the framebuffer (@plane_resync).
 *
 * Returns 0, -EINVAL if the format is neither, for an argument above or if
 * nothing of the region is on the panel, or -EBUSY with the CRTC off. A
 * refused send queues nothing and leaves the panel to whoever had it.
 */
VISIBLE_IF_KUNIT int rk3026_ebc_mxcfb_send(struct rk3026_ebc *ebc,
					   const void *src, unsigned int pitch,
					   u32 format,
					   const struct mxcfb_update_data *req)
{
	const struct mxcfb_rect *r = &req->update_region;
	const struct drm_rect panel = DRM_RECT_INIT(0, 0, ebc->width,
						    ebc->height);
	struct rk3026_ebc_update upd;
	struct drm_rect clip;
	u32 blit_format;
	bool auto_mode;
	u32 mode;

	switch (format) {
	case DRM_FORMAT_XRGB8888:
		blit_format = format;
		break;
	case DRM_FORMAT_C8:
		blit_format = DRM_FORMAT_R8;
		break;
	default:
		return -EINVAL;
	}

	if (req->update_mode > MXCFB_UPDATE_MODE_FULL)
		return -EINVAL;
	if (req->waveform_mode > MXCFB_WAVEFORM_MODE_MAX &&
	    req->waveform_mode != MXCFB_WAVEFORM_MODE_AUTO)
		return -EINVAL;
	if (req->flags & (EPDC_FLAG_USE_ALT_BUFFER | EPDC_FLAG_TEST_COLLISION))
		return -EINVAL;

	/* Each term at most the panel size first, so the sums cannot wrap. */
	clip = DRM_RECT_INIT(min(r->left, ebc->width), min(r->top, ebc->height),
			     min(r->width, ebc->width),
			     min(r->height, ebc->height));
	if (!drm_rect_intersect(&clip, &panel))
		return -EINVAL;

	mode = rk3026_ebc_mxcfb_to_ebc_mode(req->waveform_mode,
					    req->update_mode, req->flags,
					    &auto_mode);
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
	if (upd.mode != EBC_MODE_RESET) {
		upd.blit_us = rk3026_ebc_blit(ebc, src, pitch, blit_format,
					      &clip);
		if (req->flags & EPDC_FLAG_ENABLE_INVERSION)
			rk3026_ebc_mxcfb_invert(ebc->next_shadow, &clip,
						ebc->width, ebc->height);
	}
	rk3026_ebc_add_update(ebc, &upd);
	mutex_unlock(&ebc->image_lock);
	return 0;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_mxcfb_send);

static int mxcfb_send_update(struct rk3026_ebc *ebc, struct fb_info *info,
			     unsigned long arg)
{
	struct drm_fb_helper *helper = info->par;
	struct mxcfb_update_data req;

	if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
		return -EFAULT;

	/*
	 * XRGB8888 by default, C8 with "video=<connector>:<width>x<height>-8",
	 * the mode before the "-8" required.
	 */
	return rk3026_ebc_mxcfb_send(ebc, info->screen_buffer,
				     info->fix.line_length,
				     helper->fb->format->format, &req);
}

/*
 * A wait on a marker: MXCFB_WAIT_FOR_UPDATE_COMPLETE (EBC_WAIT_DONE)
 * until the update's part has finished, MXCFB_WAIT_FOR_UPDATE_SUBMISSION
 * (EBC_WAIT_TAKE) until the refresh thread has taken the update into a
 * part, before the part's waveform: a client that waits for the
 * submission may prepare its next update while the panel draws this one,
 * and what the part then does with the update (skipped, dropped with the
 * CRTC off, carried for a retry) is not waited for.
 *
 * The fbdev core calls fb_ioctl under lock_fb_info(), so a client blocked
 * here (up to rk3026_ebc_wait_timeout_ms() when the refresh thread is stuck)
 * holds the fb_info lock: a send, FBIOPAN_DISPLAY or FBIOBLANK from another
 * thread of the same or another process waits for it. So a send waits for
 * no waveform only as long as a single thread both sends and waits, as
 * e-reader clients do.
 */
static int mxcfb_wait_for_update(struct rk3026_ebc *ebc, unsigned long arg,
				 enum rk3026_ebc_wait_for until)
{
	u32 marker;

	if (copy_from_user(&marker, (void __user *)arg, sizeof(marker)))
		return -EFAULT;

	return rk3026_ebc_wait_marker(ebc, marker, until);
}

/*
 * The fb_ioctl of the emulated fbdev. The i.MX EPDC dialect and its Kobo,
 * Kindle, PocketBook and reMarkable variants use 'F' 0x21..0x55 (with the
 * i.MX IPU commands that shared the fb's number space): their sends
 * (0x2e) and waits (0x2f, 0x35, 0x37) in every struct size, the
 * temperature, power-down delay, auto-update, update scheme, waveform-set
 * and pause setters and getters, the work-buffer and EPDC-access test
 * hooks, and PocketBook's EPDC_GET_UPDATE_STATE (0x55). The two V1
 * commands and the submission wait are implemented (0x37 _IOW with a u32
 * marker is i.MX's MXCFB_WAIT_FOR_UPDATE_SUBMISSION, not Kobo's
 * SET_TEMP_AUTO_UPDATE_PERIOD); every other number there is refused with
 * -ENOTTY here, before the fbdev helper, so a client that probes for one
 * (PocketBook's B288 test takes 0 as "present") gets a defined failure
 * whatever a later helper does with the number. Every other number,
 * FBIO_WAITFORVSYNC (= MXCFB_WAIT_FOR_VSYNC) among them, is the helper's.
 */
VISIBLE_IF_KUNIT int rk3026_ebc_mxcfb_ioctl(struct fb_info *info,
					    unsigned int cmd,
					    unsigned long arg)
{
	struct drm_fb_helper *helper = info->par;
	struct rk3026_ebc *ebc = drm_to_ebc(helper->dev);

	switch (cmd) {
	case MXCFB_SEND_UPDATE:
		return mxcfb_send_update(ebc, info, arg);
	case MXCFB_WAIT_FOR_UPDATE_COMPLETE:
		return mxcfb_wait_for_update(ebc, arg, EBC_WAIT_DONE);
	case MXCFB_WAIT_FOR_UPDATE_SUBMISSION:
		return mxcfb_wait_for_update(ebc, arg, EBC_WAIT_TAKE);
	}

	if (_IOC_TYPE(cmd) == 'F' && _IOC_NR(cmd) >= MXCFB_IOC_NR_FIRST &&
	    _IOC_NR(cmd) <= MXCFB_IOC_NR_LAST)
		return -ENOTTY;

	if (ebc->orig_fbops->fb_ioctl)
		return ebc->orig_fbops->fb_ioctl(info, cmd, arg);
	return -ENOTTY;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_mxcfb_ioctl);

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
	/* Not const: a copy whose entry points this shim replaces. */
	struct fb_ops *ops;

	if (!info || !info->fbops)
		return;

	ops = drmm_kmalloc(&ebc->drm, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return;
	*ops = *info->fbops;

	ebc->orig_fbops = info->fbops;
	ops->fb_ioctl = rk3026_ebc_mxcfb_ioctl;
	ops->fb_open = ebc_fb_open;
	ops->fb_release = ebc_fb_release;
	ebc->fbops = ops;
	info->fbops = ops;

	drm_info(&ebc->drm, "mxcfb V1 ioctl shim on fb%d\n", info->node);
}
