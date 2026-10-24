// SPDX-License-Identifier: GPL-2.0
/*
 * debugfs of the RK3026 EBC: what the driver believes is on the glass.
 *
 * <debugfs>/rk3026-ebc/
 *   panel   the on-panel image (@shadows.prev), read-only binary
 *   target  the target image the senders write (@next_shadow); a scan draws
 *           the part of it its updates cover, read-only binary
 *   state   text, one "key value" line per item (rk3026_ebc_state_show())
 *   du_state         text, the DU cleanup budget (rk3026_ebc_du_state_show())
 *   du_cleanup_area  the budget's limit in pixels, read-write; 0: no
 *                    cleanup FULL. A diagnostic: probe sets one panel's area.
 *
 * Both images are @img_size = width * height / 2 bytes: 4 bpp, two pixels per
 * byte with the LEFT pixel in the LOW nibble, rows of width / 2 bytes, top row
 * first, in the controller's scan order (the DT "rotation" is a property of
 * the connector and does not apply here), 0 black and 15 white. That is the
 * layout rk3026_ebc_blit_xrgb8888() writes.
 *
 * Both are the cached shadows, copied at open() under @image_lock. Every
 * write to them takes that lock: a sender blits into next and queues its
 * update in one section, the white fill of a RESET is one section with the
 * RESET, and the refresh thread changes prev only at the end of a scan,
 * in the section that publishes the "state" record. So an image is never
 * half of an update. Copying at open() instead of locking per read() matters
 * because a reader takes the file in chunks: a lock per read() would let an
 * update land between two of them. The cost is one image-sized buffer per
 * open file; the lock is not held across a waveform, so an open() waits at
 * most for one snapshot to be built.
 */

#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/overflow.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include "rk3026_ebc.h"

struct ebc_snapshot {
	size_t	len;
	u8	data[];
};

static int ebc_image_open(struct inode *inode, struct file *file, bool target)
{
	struct rk3026_ebc *ebc = inode->i_private;
	struct ebc_snapshot *snap;
	int ret;

	snap = kvmalloc(struct_size(snap, data, ebc->img_size), GFP_KERNEL);
	if (!snap)
		return -ENOMEM;

	ret = mutex_lock_interruptible(&ebc->image_lock);
	if (ret) {
		kvfree(snap);
		return ret;
	}
	memcpy(snap->data, target ? ebc->next_shadow : ebc->shadows.prev,
	       ebc->img_size);
	mutex_unlock(&ebc->image_lock);

	snap->len = ebc->img_size;
	file->private_data = snap;
	return 0;
}

static int ebc_panel_open(struct inode *inode, struct file *file)
{
	return ebc_image_open(inode, file, false);
}

static int ebc_target_open(struct inode *inode, struct file *file)
{
	return ebc_image_open(inode, file, true);
}

static ssize_t ebc_image_read(struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	const struct ebc_snapshot *snap = file->private_data;

	return simple_read_from_buffer(buf, count, ppos, snap->data, snap->len);
}

static int ebc_image_release(struct inode *inode, struct file *file)
{
	kvfree(file->private_data);
	return 0;
}

static const struct file_operations ebc_panel_fops = {
	.owner		= THIS_MODULE,
	.open		= ebc_panel_open,
	.read		= ebc_image_read,
	.release	= ebc_image_release,
	.llseek		= default_llseek,
};

static const struct file_operations ebc_target_fops = {
	.owner		= THIS_MODULE,
	.open		= ebc_target_open,
	.read		= ebc_image_read,
	.release	= ebc_image_release,
	.llseek		= default_llseek,
};

static const char *const ebc_source_names[] = {
	[EBC_SOURCE_PLANE]	= "plane",
	[EBC_SOURCE_MXCFB]	= "mxcfb",
	[EBC_SOURCE_ENABLE]	= "enable",
};

static const char *const ebc_outcome_names[] = {
	[EBC_OUTCOME_NONE]	  = "none",
	[EBC_OUTCOME_DRIVEN]	  = "driven",
	[EBC_OUTCOME_SKIPPED]	  = "skipped",
	[EBC_OUTCOME_CRTC_OFF]	  = "crtc_off",
	[EBC_OUTCOME_NO_WAVEFORM] = "no_waveform",
	[EBC_OUTCOME_NO_RAILS]	  = "no_rails",
	[EBC_OUTCOME_NO_LUT]	  = "no_lut",
	[EBC_OUTCOME_TIMEOUT]	  = "timeout",
	[EBC_OUTCOME_ABANDONED]	  = "abandoned",
};

static const char *ebc_name(const char *const *names, size_t n, unsigned int i)
{
	return i < n && names[i] ? names[i] : "?";
}

/* One update record, every key prefixed: "last_" or "scan_". */
static void ebc_show_last(struct seq_file *m, const char *p,
			  const struct rk3026_ebc_last *l)
{
	seq_printf(m, "%ssource %s\n", p,
		   ebc_name(ebc_source_names, ARRAY_SIZE(ebc_source_names),
			    l->source));
	seq_printf(m, "%soutcome %s\n", p,
		   ebc_name(ebc_outcome_names, ARRAY_SIZE(ebc_outcome_names),
			    l->outcome));
	seq_printf(m, "%sskipped %d\n", p, l->outcome == EBC_OUTCOME_SKIPPED);
	seq_printf(m, "%sasked_mode %u\n", p, l->asked_mode);
	seq_printf(m, "%smode %u\n", p, l->mode);
	seq_printf(m, "%slut_mode %u\n", p, l->lut_mode);
	seq_printf(m, "%sclip %d %d %d %d\n", p, l->clip.x1, l->clip.y1,
		   l->clip.x2, l->clip.y2);
	seq_printf(m, "%smarker %u\n", p, l->marker);
	if (l->temp_read)
		seq_printf(m, "%stemp %d\n", p, l->temp);
	else
		seq_printf(m, "%stemp none\n", p);
	seq_printf(m, "%srepaired %d\n", p, l->repaired);
	seq_printf(m, "%sscans %u\n", p, l->scans);
	seq_printf(m, "%sframes %u\n", p, l->frames);
	seq_printf(m, "%sdriven %u\n", p, l->driven);
	seq_printf(m, "%stimeouts %u\n", p, l->timeouts);
	seq_printf(m, "%sms %u\n", p, l->ms);
	seq_printf(m, "%swave_ms %u\n", p, l->scan_ms);
}

/*
 * "state": one "key value" line per item, values decimal or a word. The
 * record and the counters are copied under @image_lock, where the refresh
 * thread publishes them, so they agree with each other; this waits at most
 * for one snapshot to be built, never for a waveform.
 *
 * last_* is the last part the refresh thread finished, whatever became of
 * it; scan_* the last one that started a waveform. A part is the updates
 * one scan took: its clip is their bounding box, its marker and source the
 * newest update's. queued counts the parts still to take (at most three:
 * before a RESET, the RESET, after it); busy is 1 while a part is pending or
 * in flight (a send not finished yet). Clips are x1 y1 x2 y2, x2/y2
 * exclusive. Modes are the EBC work modes (EBC_MODE_*). Times in ms.
 */
static int rk3026_ebc_state_show(struct seq_file *m, void *unused)
{
	struct rk3026_ebc *ebc = m->private;
	struct rk3026_ebc_last last, scan;
	struct rk3026_ebc_stats st;
	unsigned int queued;
	bool busy, crtc_on;

	mutex_lock(&ebc->image_lock);
	last = ebc->last;
	scan = ebc->last_driven;
	st = ebc->stats;
	queued = rk3026_ebc_pending_parts(&ebc->pending);
	busy = rk3026_ebc_work_pending(&ebc->pending);
	crtc_on = ebc->crtc_on;
	mutex_unlock(&ebc->image_lock);

	seq_puts(m, "version 1\n");
	seq_printf(m, "width %u\n", ebc->width);
	seq_printf(m, "height %u\n", ebc->height);
	seq_printf(m, "stride %u\n", ebc->width / 2);
	seq_printf(m, "img_size %zu\n", ebc->img_size);
	seq_printf(m, "crtc_on %d\n", crtc_on);
	seq_printf(m, "mxcfb_active %d\n", READ_ONCE(ebc->mxcfb_active));
	seq_printf(m, "rails_on %d\n", READ_ONCE(ebc->rails_on));
	seq_printf(m, "epd_on %d\n", READ_ONCE(ebc->epd_on));
	seq_printf(m, "vcom_on %d\n", READ_ONCE(ebc->vcom_on));
	seq_printf(m, "v3p3_on %d\n", READ_ONCE(ebc->v3p3_on));
	seq_printf(m, "prev_unknown %d\n", READ_ONCE(ebc->shadows.prev_unknown));
	seq_printf(m, "busy %d\n", busy);
	seq_printf(m, "queued %u\n", queued);
	seq_printf(m, "updates %llu\n", st.updates);
	seq_printf(m, "coalesced %llu\n", st.coalesced);
	seq_printf(m, "done %llu\n", st.done);
	seq_printf(m, "scans %llu\n", st.scans);
	seq_printf(m, "skips %llu\n", st.skips);
	seq_printf(m, "dropped %llu\n", st.dropped);
	seq_printf(m, "timeouts %llu\n", st.timeouts);
	seq_printf(m, "abandoned %llu\n", st.abandoned);
	ebc_show_last(m, "last_", &last);
	ebc_show_last(m, "scan_", &scan);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(rk3026_ebc_state);

/*
 * "du_state", apart from "state" so that file keeps its version: du_area is
 * the pixels plane parts changed in DU since the glass was last cleared
 * (rk3026_ebc_du_account()), du_cleanup_area the limit (0: none), and
 * cleanup_due 1 when the next two-level plane part will be driven FULL
 * (rk3026_ebc_resolve_mode()).
 */
static int rk3026_ebc_du_state_show(struct seq_file *m, void *unused)
{
	struct rk3026_ebc *ebc = m->private;
	u32 area, limit;

	mutex_lock(&ebc->image_lock);
	area = ebc->du_area;
	mutex_unlock(&ebc->image_lock);
	limit = READ_ONCE(ebc->du_cleanup_area);

	seq_printf(m, "du_area %u\n", area);
	seq_printf(m, "du_cleanup_area %u\n", limit);
	seq_printf(m, "cleanup_due %d\n", limit && area >= limit);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(rk3026_ebc_du_state);

void rk3026_ebc_debugfs_init(struct rk3026_ebc *ebc)
{
	struct dentry *dir = debugfs_create_dir("rk3026-ebc", NULL);

	ebc->debugfs = dir;
	debugfs_create_file_size("panel", 0400, dir, ebc, &ebc_panel_fops,
				 ebc->img_size);
	debugfs_create_file_size("target", 0400, dir, ebc, &ebc_target_fops,
				 ebc->img_size);
	debugfs_create_file("state", 0444, dir, ebc, &rk3026_ebc_state_fops);
	debugfs_create_file("du_state", 0444, dir, ebc,
			    &rk3026_ebc_du_state_fops);
	debugfs_create_u32("du_cleanup_area", 0644, dir, &ebc->du_cleanup_area);
}

void rk3026_ebc_debugfs_fini(struct rk3026_ebc *ebc)
{
	debugfs_remove(ebc->debugfs);
	ebc->debugfs = NULL;
}
