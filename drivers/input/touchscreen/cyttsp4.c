// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cypress TrueTouch(TM) Standard Product V4 (Gen4) I2C touchscreen driver.
 * For use with Cypress Txx4xx parts.
 * Supported parts include:
 * TMA4XX
 * TMA1036
 *
 * Copyright (C) 2009, 2010, 2011 Cypress Semiconductor, Inc.
 * Copyright (C) 2012 Javier Martinez Canillas <javier@dowhile0.org>
 * Copyright (C) 2012, 2013 Cypress Semiconductor
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 * Based on the cyttsp4 core and I2C drivers that were removed in commit
 * 25162a4f64f8 ("Input: cyttsp4 - remove driver"): I2C only, configured
 * from the device tree instead of platform data, no loader or debug
 * interfaces.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define CYTTSP4_NAME			"cyttsp4_mt"

#define CY_REG_BASE			0x00

/* host mode register (register 0) */
#define CY_HST_TOGGLE			BIT(7)
#define CY_HST_MODE			GENMASK(6, 4)
#define CY_HST_OPERATE			FIELD_PREP_CONST(CY_HST_MODE, 0)
#define CY_HST_SYSINFO			FIELD_PREP_CONST(CY_HST_MODE, 1)
#define CY_HST_CAT			FIELD_PREP_CONST(CY_HST_MODE, 2)
#define CY_HST_MODE_CHANGE		BIT(3)
#define CY_HST_SLEEP			BIT(1)
#define CY_HST_RESET			BIT(0)

/* the bootloader runs when bit 0 of register 0 or the reset detect byte is set */
#define IS_BOOTLOADER(hst_mode, reset_detect) \
	(((hst_mode) & BIT(0)) || (reset_detect) != 0)

/* POST code, low byte */
#define CY_POST_CODEL_WDG_RST		BIT(0)
#define CY_POST_CODEL_SCANNING		BIT(3)

/* touch record field location: byte offset and bit offset */
#define CY_BYTE_OFS_MASK		GENMASK(4, 0)
#define CY_BOFS_MASK			GENMASK(7, 5)

/* panel resolution: 15 bits, the top bit of the high byte is the origin */
#define CY_PCFG_RES_MASK		GENMASK(14, 0)

#define CY_TT_STAT_NUM_TOUCHES		GENMASK(4, 0)
#define CY_REP_STAT_BAD_PKT		BIT(5)

#define CY_TMA1036_TCH_REC_SIZE		6
#define CY_MAX_TCHS			31	/* 5-bit field */
#define CY_MAX_TCH_REC_SIZE		31	/* 5-bit field */
#define CY_MAX_SLOTS			64
#define CY_MAX_WRITE			8	/* longest command: ldr_exit */

#define CY_EV_LIFTOFF			3

#define CY_MODE_CHANGE_TIMEOUT_MS	1000
#define CY_HEARTBEAT_TIMEOUT_MS		500
#define CY_WAKEUP_TIMEOUT_MS		500
#define CY_WATCHDOG_MS			1000
#define CY_STARTUP_RETRIES		3
#define CY_SLEEP_SETTLE_MS		50
#define CY_POWER_ON_DELAY_MS		20
#define CY_LEVEL_IRQ_UDELAY		5

/* bootloader command: exit to the touch application (sysinfo mode) */
static const u8 ldr_exit[] = {
	0xff, 0x01, 0x3b, 0x00, 0x00, 0x4f, 0x6d, 0x17
};

/* bootloader status after a failed launch of the touch application */
static const u8 ldr_err_app[] = {
	0x01, 0x02, 0x00, 0x00, 0x55, 0xdd, 0x17
};

enum cyttsp4_tch_abs {
	CY_TCH_X,	/* X */
	CY_TCH_Y,	/* Y */
	CY_TCH_P,	/* pressure */
	CY_TCH_T,	/* touch (track) id */
	CY_TCH_E,	/* event id */
	CY_TCH_O,	/* object id */
	CY_TCH_W,	/* size */
	CY_TCH_MAJ,	/* touch major */
	CY_TCH_MIN,	/* touch minor */
	CY_TCH_OR,	/* orientation */
	CY_TCH_NUM_ABS
};

#define CY_NUM_TCH_FIELDS		7	/* X .. W */
#define CY_NUM_EXT_TCH_FIELDS		3	/* MAJ .. OR, TMA4xx only */

enum cyttsp4_int_state {
	CY_INT_IGNORE		= BIT(0),
	CY_INT_MODE_CHANGE	= BIT(1),
	CY_INT_AWAKE		= BIT(2),
};

enum cyttsp4_mode {
	CY_MODE_UNKNOWN,
	CY_MODE_BOOTLOADER,
	CY_MODE_OPERATIONAL,
	CY_MODE_SYSINFO,
	CY_MODE_CAT,
};

enum cyttsp4_sleep_state {
	SS_SLEEP_OFF,
	SS_SLEEP_ON,
	SS_SLEEPING,
	SS_WAKING,
};

enum cyttsp4_startup_state {
	STARTUP_NONE,
	STARTUP_QUEUED,
	STARTUP_RUNNING,
};

/* register map, as read from the controller */
struct cyttsp4_sysinfo_data {
	u8 hst_mode;
	u8 reserved;
	__be16 map_sz;
	__be16 cydata_ofs;
	__be16 test_ofs;
	__be16 pcfg_ofs;
	__be16 opcfg_ofs;
	__be16 ddata_ofs;
	__be16 mdata_ofs;
} __packed;

/* start of the cydata block; mfg_id[mfgid_sz] and the cyito ids follow */
struct cyttsp4_cydata {
	__be16 ttpid;
	u8 fw_ver_major;
	u8 fw_ver_minor;
	u8 revctrl[8];
	u8 blver_major;
	u8 blver_minor;
	u8 jtag_si_id[4];
	u8 mfgid_sz;
} __packed;

/* cyito_idh .. device_info, after mfg_id */
#define CY_CYDATA_TAIL_SIZE		7

struct cyttsp4_test {
	u8 post_codeh;
	u8 post_codel;
} __packed;

struct cyttsp4_pcfg {
	u8 electrodes_x;
	u8 electrodes_y;
	__be16 len_x;
	__be16 len_y;
	__be16 res_x;
	__be16 res_y;
	__be16 max_z;
	u8 panel_info0;
} __packed;

struct cyttsp4_tch_rec_params {
	u8 loc;
	u8 size;	/* bits */
} __packed;

struct cyttsp4_opcfg {
	u8 cmd_ofs;
	u8 rep_ofs;
	__be16 rep_sz;
	u8 num_btns;
	u8 tt_stat_ofs;
	u8 obj_cfg0;
	u8 max_tchs;
	u8 tch_rec_size;
	struct cyttsp4_tch_rec_params tch_rec_old[CY_NUM_TCH_FIELDS];
	u8 btn_rec_size;
	u8 btn_diff_ofs;
	u8 btn_diff_size;
	struct cyttsp4_tch_rec_params tch_rec_new[CY_NUM_EXT_TCH_FIELDS];
} __packed;

struct cyttsp4_tch_field {
	unsigned int ofs;	/* byte offset in the touch record */
	unsigned int size;	/* bytes */
	unsigned int bofs;	/* bit offset */
	unsigned int max;	/* 1 << bits; 0 = field not present */
};

/* what the driver keeps of the register map */
struct cyttsp4_sysinfo {
	unsigned int rep_ofs;
	unsigned int tt_stat_ofs;
	unsigned int mode_size;		/* status registers: 0 .. tt_stat_ofs */
	unsigned int max_tchs;
	unsigned int tch_rec_size;
	unsigned int res_x;
	unsigned int res_y;
	unsigned int max_p;
	struct cyttsp4_tch_field tch[CY_TCH_NUM_ABS];
};

struct cyttsp4 {
	struct i2c_client *client;
	struct device *dev;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct gpio_desc *reset_gpio;
	int irq;
	bool level_irq;

	/* serializes startup, sleep and wake */
	struct mutex ctrl_lock;
	/* protects the fields below and the bus traffic of the IRQ thread */
	struct mutex system_lock;
	wait_queue_head_t wait_q;
	enum cyttsp4_mode mode;
	enum cyttsp4_sleep_state sleep_state;
	enum cyttsp4_startup_state startup_state;
	unsigned int int_status;
	bool input_ready;
	bool suspended;
	bool stopped;

	struct work_struct startup_work;
	struct delayed_work watchdog_work;

	struct cyttsp4_sysinfo si;
	unsigned int num_slots;		/* registered with the input device */
	bool ext_fields;		/* major, minor, orientation reported */

	u8 xy_mode[U8_MAX + 1];
	u8 xy_data[CY_MAX_TCHS * CY_MAX_TCH_REC_SIZE];
};

/*
 * Register addresses are 9 bits wide: bit 8 selects the next I2C address
 * (0x24 and 0x25 for a controller at 0x24).
 */
static int cyttsp4_read(struct cyttsp4 *ts, u16 addr, void *buf, size_t len)
{
	struct i2c_client *client = ts->client;
	u16 client_addr = client->addr | ((addr >> 8) & 0x1);
	u8 reg = addr & 0xff;
	struct i2c_msg msgs[] = {
		{
			.addr = client_addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client_addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;

	return ret == ARRAY_SIZE(msgs) ? 0 : -EIO;
}

static int cyttsp4_write(struct cyttsp4 *ts, u16 addr, const void *buf,
			 size_t len)
{
	struct i2c_client *client = ts->client;
	u8 data[CY_MAX_WRITE + 1];
	struct i2c_msg msg = {
		.addr = client->addr | ((addr >> 8) & 0x1),
		.flags = 0,
		.len = len + 1,
		.buf = data,
	};
	int ret;

	if (len > CY_MAX_WRITE)
		return -EINVAL;

	data[0] = addr & 0xff;
	memcpy(&data[1], buf, len);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return ret == 1 ? 0 : -EIO;
}

/* acknowledge an interrupt: flip the toggle bit of the host mode register */
static int cyttsp4_handshake(struct cyttsp4 *ts, u8 mode)
{
	u8 cmd = mode ^ CY_HST_TOGGLE;
	int error;

	/*
	 * A mode change is in progress: a handshake now would request another
	 * one.
	 */
	if (mode & CY_HST_MODE_CHANGE)
		return 0;

	error = cyttsp4_write(ts, CY_REG_BASE, &cmd, sizeof(cmd));
	if (error)
		dev_err(ts->dev, "handshake failed: %d\n", error);

	return error;
}

static int cyttsp4_hw_reset(struct cyttsp4 *ts)
{
	u8 cmd = CY_HST_RESET;

	if (!ts->reset_gpio)
		return cyttsp4_write(ts, CY_REG_BASE, &cmd, sizeof(cmd));

	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(20);
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(40);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(20);

	return 0;
}

/* called with system_lock held */
static void cyttsp4_queue_startup(struct cyttsp4 *ts)
{
	if (ts->stopped || ts->suspended ||
	    ts->startup_state != STARTUP_NONE)
		return;

	ts->startup_state = STARTUP_QUEUED;
	schedule_work(&ts->startup_work);
}

static void cyttsp4_start_wd(struct cyttsp4 *ts)
{
	mutex_lock(&ts->system_lock);
	if (!ts->stopped && !ts->suspended)
		schedule_delayed_work(&ts->watchdog_work,
				      msecs_to_jiffies(CY_WATCHDOG_MS));
	mutex_unlock(&ts->system_lock);
}

static void cyttsp4_stop_wd(struct cyttsp4 *ts)
{
	cancel_delayed_work_sync(&ts->watchdog_work);
}

static int cyttsp4_si_field(struct cyttsp4 *ts, enum cyttsp4_tch_abs abs,
			    const struct cyttsp4_tch_rec_params *rec)
{
	struct cyttsp4_tch_field *f = &ts->si.tch[abs];
	unsigned int bits = rec->size;

	f->ofs = rec->loc & CY_BYTE_OFS_MASK;
	f->bofs = FIELD_GET(CY_BOFS_MASK, rec->loc);
	f->size = DIV_ROUND_UP(bits, 8);
	f->max = bits ? 1U << bits : 0;

	if (bits > 16 || (bits && f->ofs + f->size > ts->si.tch_rec_size)) {
		dev_err(ts->dev, "touch field %d: %u bits at byte %u, record %u bytes\n",
			abs, bits, f->ofs, ts->si.tch_rec_size);
		return -EINVAL;
	}

	return 0;
}

/* informational: identity of the part and its firmware */
static void cyttsp4_si_log_cydata(struct cyttsp4 *ts, u16 ofs, size_t size)
{
	struct cyttsp4_cydata *cy;
	u8 *tail;
	u8 *buf;
	int error;

	if (size < sizeof(*cy) + CY_CYDATA_TAIL_SIZE || size > U8_MAX) {
		dev_warn(ts->dev, "cydata: unexpected size %zu\n", size);
		return;
	}

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
		return;

	error = cyttsp4_read(ts, ofs, buf, size);
	if (error) {
		dev_warn(ts->dev, "cydata: read failed: %d\n", error);
		goto out;
	}

	cy = (struct cyttsp4_cydata *)buf;
	if (cy->mfgid_sz != size - sizeof(*cy) - CY_CYDATA_TAIL_SIZE) {
		dev_warn(ts->dev, "cydata: mfgid size %u, block %zu bytes\n",
			 cy->mfgid_sz, size);
		goto out;
	}

	tail = buf + sizeof(*cy) + cy->mfgid_sz;
	dev_info(ts->dev,
		 "TTPID 0x%04x, firmware %u.%u, revctrl %*phN, bootloader %u.%u, silicon id %*phN, TTSP %u.%u, cyito 0x%02x%02x v%u.%u\n",
		 be16_to_cpu(cy->ttpid), cy->fw_ver_major, cy->fw_ver_minor,
		 (int)sizeof(cy->revctrl), cy->revctrl,
		 cy->blver_major, cy->blver_minor,
		 (int)sizeof(cy->jtag_si_id), cy->jtag_si_id,
		 tail[4], tail[5], tail[0], tail[1], tail[2], tail[3]);
out:
	kfree(buf);
}

static int cyttsp4_si_read_test(struct cyttsp4 *ts, u16 ofs, size_t size)
{
	struct cyttsp4_test test;
	int error;

	if (size < sizeof(test))
		return 0;

	error = cyttsp4_read(ts, ofs, &test, sizeof(test));
	if (error)
		return error;

	dev_dbg(ts->dev, "POST code 0x%02x%02x\n", test.post_codeh,
		test.post_codel);
	if (test.post_codel & CY_POST_CODEL_WDG_RST)
		dev_info(ts->dev, "the last reset was a watchdog reset\n");
	if (!(test.post_codel & CY_POST_CODEL_SCANNING))
		dev_warn(ts->dev, "scanning disabled (POST code 0x%02x%02x)\n",
			 test.post_codeh, test.post_codel);

	return 0;
}

static int cyttsp4_si_read_pcfg(struct cyttsp4 *ts, u16 ofs, size_t size)
{
	struct cyttsp4_pcfg pcfg;
	int error;

	if (size < sizeof(pcfg)) {
		dev_err(ts->dev, "pcfg: %zu bytes, need %zu\n", size,
			sizeof(pcfg));
		return -EINVAL;
	}

	error = cyttsp4_read(ts, ofs, &pcfg, sizeof(pcfg));
	if (error)
		return error;

	ts->si.res_x = be16_to_cpu(pcfg.res_x) & CY_PCFG_RES_MASK;
	ts->si.res_y = be16_to_cpu(pcfg.res_y) & CY_PCFG_RES_MASK;
	ts->si.max_p = be16_to_cpu(pcfg.max_z);

	dev_info(ts->dev,
		 "panel: %u x %u electrodes, resolution %u x %u (origin x %s, y %s), max z %u\n",
		 pcfg.electrodes_x, pcfg.electrodes_y,
		 ts->si.res_x, ts->si.res_y,
		 be16_to_cpu(pcfg.res_x) & BIT(15) ? "right" : "left",
		 be16_to_cpu(pcfg.res_y) & BIT(15) ? "bottom" : "top",
		 ts->si.max_p);

	if (!ts->si.res_x || !ts->si.res_y) {
		dev_err(ts->dev, "pcfg: no resolution\n");
		return -EINVAL;
	}

	return 0;
}

static int cyttsp4_si_read_opcfg(struct cyttsp4 *ts, u16 ofs, size_t size)
{
	struct cyttsp4_sysinfo *si = &ts->si;
	struct cyttsp4_opcfg opcfg = { };
	size_t need = offsetof(struct cyttsp4_opcfg, tch_rec_new);
	enum cyttsp4_tch_abs abs;
	int error;
	int i;

	if (size < need) {
		dev_err(ts->dev, "opcfg: %zu bytes, need %zu\n", size, need);
		return -EINVAL;
	}

	error = cyttsp4_read(ts, ofs, &opcfg, min(size, sizeof(opcfg)));
	if (error)
		return error;

	si->rep_ofs = opcfg.rep_ofs;
	si->tt_stat_ofs = opcfg.tt_stat_ofs;
	si->mode_size = si->tt_stat_ofs + 1;
	si->max_tchs = opcfg.max_tchs & CY_BYTE_OFS_MASK;
	si->tch_rec_size = opcfg.tch_rec_size & CY_BYTE_OFS_MASK;

	if (si->rep_ofs + 1 >= si->mode_size || !si->max_tchs ||
	    !si->tch_rec_size) {
		dev_err(ts->dev, "opcfg: rep_ofs %u, tt_stat_ofs %u, max_tchs %u, record %u bytes\n",
			si->rep_ofs, si->tt_stat_ofs, si->max_tchs,
			si->tch_rec_size);
		return -EINVAL;
	}

	memset(si->tch, 0, sizeof(si->tch));
	for (abs = CY_TCH_X; abs < CY_NUM_TCH_FIELDS; abs++) {
		error = cyttsp4_si_field(ts, abs, &opcfg.tch_rec_old[abs]);
		if (error)
			return error;
	}

	if (si->tch_rec_size > CY_TMA1036_TCH_REC_SIZE) {
		if (size < sizeof(opcfg)) {
			dev_err(ts->dev, "opcfg: %zu bytes, need %zu\n", size,
				sizeof(opcfg));
			return -EINVAL;
		}
		for (i = 0; i < CY_NUM_EXT_TCH_FIELDS; i++) {
			error = cyttsp4_si_field(ts, CY_TCH_MAJ + i,
						 &opcfg.tch_rec_new[i]);
			if (error)
				return error;
		}
	}

	if (!si->tch[CY_TCH_X].max || !si->tch[CY_TCH_Y].max ||
	    !si->tch[CY_TCH_T].max || !si->tch[CY_TCH_E].max ||
	    si->tch[CY_TCH_T].max > CY_MAX_SLOTS) {
		dev_err(ts->dev, "opcfg: unusable touch record layout\n");
		return -EINVAL;
	}

	dev_dbg(ts->dev, "status regs %u bytes, report at %u, %u touches of %u bytes, %u track ids\n",
		si->mode_size, si->rep_ofs, si->max_tchs, si->tch_rec_size,
		si->tch[CY_TCH_T].max);

	return 0;
}

static int cyttsp4_get_sysinfo_regs(struct cyttsp4 *ts)
{
	struct cyttsp4_sysinfo_data sd;
	u16 cydata, test, pcfg, opcfg, ddata;
	int error;

	error = cyttsp4_read(ts, CY_REG_BASE, &sd, sizeof(sd));
	if (error)
		return error;

	cydata = be16_to_cpu(sd.cydata_ofs);
	test = be16_to_cpu(sd.test_ofs);
	pcfg = be16_to_cpu(sd.pcfg_ofs);
	opcfg = be16_to_cpu(sd.opcfg_ofs);
	ddata = be16_to_cpu(sd.ddata_ofs);

	if (test <= cydata || pcfg <= test || opcfg <= pcfg || ddata <= opcfg) {
		dev_err(ts->dev, "sysinfo: bad offsets cydata %u test %u pcfg %u opcfg %u ddata %u\n",
			cydata, test, pcfg, opcfg, ddata);
		return -EINVAL;
	}

	cyttsp4_si_log_cydata(ts, cydata, test - cydata);

	error = cyttsp4_si_read_test(ts, test, pcfg - test);
	if (error)
		return error;

	error = cyttsp4_si_read_pcfg(ts, pcfg, opcfg - pcfg);
	if (error)
		return error;

	error = cyttsp4_si_read_opcfg(ts, opcfg, ddata - opcfg);
	if (error)
		return error;

	/* flow control: acknowledge the sysinfo data */
	return cyttsp4_handshake(ts, sd.hst_mode);
}

static int cyttsp4_wait_mode(struct cyttsp4 *ts, enum cyttsp4_mode mode,
			     unsigned int timeout_ms)
{
	if (!wait_event_timeout(ts->wait_q, ts->mode == mode,
				msecs_to_jiffies(timeout_ms)))
		return -ETIMEDOUT;

	return 0;
}

static int cyttsp4_reset_and_wait(struct cyttsp4 *ts)
{
	int error;

	mutex_lock(&ts->system_lock);
	error = cyttsp4_hw_reset(ts);
	ts->mode = CY_MODE_UNKNOWN;
	mutex_unlock(&ts->system_lock);
	if (error) {
		dev_err(ts->dev, "reset failed: %d\n", error);
		return error;
	}

	/* the bootloader signals itself by a heartbeat interrupt */
	error = cyttsp4_wait_mode(ts, CY_MODE_BOOTLOADER,
				  CY_HEARTBEAT_TIMEOUT_MS);
	if (error)
		dev_err(ts->dev, "no bootloader heartbeat after reset\n");

	return error;
}

/* request a mode and wait for the mode change interrupt */
static int cyttsp4_set_mode(struct cyttsp4 *ts, u8 new_dev_mode)
{
	u8 mode;
	int error;

	mutex_lock(&ts->system_lock);
	error = cyttsp4_read(ts, CY_REG_BASE, &mode, sizeof(mode));
	if (!error) {
		mode &= ~CY_HST_MODE;
		mode |= new_dev_mode | CY_HST_MODE_CHANGE;
		ts->int_status |= CY_INT_MODE_CHANGE;
		error = cyttsp4_write(ts, CY_REG_BASE, &mode, sizeof(mode));
		if (error)
			ts->int_status &= ~CY_INT_MODE_CHANGE;
	}
	mutex_unlock(&ts->system_lock);
	if (error)
		return error;

	if (!wait_event_timeout(ts->wait_q,
				!(ts->int_status & CY_INT_MODE_CHANGE),
				msecs_to_jiffies(CY_MODE_CHANGE_TIMEOUT_MS))) {
		mutex_lock(&ts->system_lock);
		ts->int_status &= ~CY_INT_MODE_CHANGE;
		mutex_unlock(&ts->system_lock);
		return -ETIMEDOUT;
	}

	return 0;
}

static void cyttsp4_lift_all(struct cyttsp4 *ts)
{
	mutex_lock(&ts->system_lock);
	if (ts->input_ready) {
		input_mt_sync_frame(ts->input);
		input_sync(ts->input);
	}
	mutex_unlock(&ts->system_lock);
}

static int cyttsp4_sleep_locked(struct cyttsp4 *ts);

/* called with ctrl_lock held */
static int cyttsp4_startup_locked(struct cyttsp4 *ts)
{
	int retry = CY_STARTUP_RETRIES;
	u8 buf[sizeof(ldr_err_app)];
	int error;

	cyttsp4_stop_wd(ts);

	do {
		error = cyttsp4_reset_and_wait(ts);
		if (error)
			continue;

		/* leave the bootloader into sysinfo mode */
		mutex_lock(&ts->system_lock);
		ts->int_status &= ~CY_INT_IGNORE;
		ts->int_status |= CY_INT_MODE_CHANGE;
		error = cyttsp4_write(ts, CY_REG_BASE, ldr_exit,
				      sizeof(ldr_exit));
		mutex_unlock(&ts->system_lock);
		if (error) {
			dev_err(ts->dev, "bootloader exit failed: %d\n", error);
			continue;
		}

		error = cyttsp4_wait_mode(ts, CY_MODE_SYSINFO,
					  CY_MODE_CHANGE_TIMEOUT_MS);
		if (error) {
			mutex_lock(&ts->system_lock);
			ts->int_status &= ~CY_INT_MODE_CHANGE;
			mutex_unlock(&ts->system_lock);

			if (!cyttsp4_read(ts, CY_REG_BASE, buf, sizeof(buf)) &&
			    !memcmp(buf, ldr_err_app, sizeof(buf))) {
				dev_err(ts->dev, "the touch application does not start\n");
				return -ENODEV;
			}
			dev_err(ts->dev, "no sysinfo mode after bootloader exit\n");
			continue;
		}

		error = cyttsp4_get_sysinfo_regs(ts);
		if (error) {
			dev_err(ts->dev, "sysinfo read failed: %d\n", error);
			continue;
		}

		error = cyttsp4_set_mode(ts, CY_HST_OPERATE);
		if (error) {
			dev_err(ts->dev, "no operational mode: %d\n", error);
			continue;
		}

		break;
	} while (retry-- > 0);

	if (error)
		goto out;

	cyttsp4_lift_all(ts);

	/* a controller reset while suspended: put it back to sleep */
	mutex_lock(&ts->system_lock);
	if (ts->sleep_state == SS_SLEEP_ON) {
		ts->sleep_state = SS_SLEEP_OFF;
		mutex_unlock(&ts->system_lock);
		return cyttsp4_sleep_locked(ts);
	}
	mutex_unlock(&ts->system_lock);

out:
	cyttsp4_start_wd(ts);
	return error;
}

static int cyttsp4_startup(struct cyttsp4 *ts)
{
	int error;

	mutex_lock(&ts->system_lock);
	ts->startup_state = STARTUP_RUNNING;
	mutex_unlock(&ts->system_lock);

	mutex_lock(&ts->ctrl_lock);
	error = cyttsp4_startup_locked(ts);
	mutex_unlock(&ts->ctrl_lock);

	mutex_lock(&ts->system_lock);
	ts->startup_state = STARTUP_NONE;
	mutex_unlock(&ts->system_lock);

	return error;
}

static void cyttsp4_startup_work(struct work_struct *work)
{
	struct cyttsp4 *ts = container_of(work, struct cyttsp4, startup_work);
	int error;

	error = cyttsp4_startup(ts);
	if (error)
		dev_err(ts->dev, "restart failed: %d\n", error);
}

static void cyttsp4_watchdog_work(struct work_struct *work)
{
	struct cyttsp4 *ts = container_of(to_delayed_work(work),
					  struct cyttsp4, watchdog_work);
	u8 mode[2];
	int error;

	mutex_lock(&ts->system_lock);
	error = cyttsp4_read(ts, CY_REG_BASE, mode, sizeof(mode));
	if (error || IS_BOOTLOADER(mode[0], mode[1])) {
		dev_err_ratelimited(ts->dev, "watchdog: %s, restarting\n",
				    error ? "no answer" :
				    "controller in the bootloader");
		cyttsp4_queue_startup(ts);
	} else if (!ts->stopped && !ts->suspended) {
		schedule_delayed_work(&ts->watchdog_work,
				      msecs_to_jiffies(CY_WATCHDOG_MS));
	}
	mutex_unlock(&ts->system_lock);
}

static unsigned int cyttsp4_get_field(const struct cyttsp4_tch_field *f,
				      const u8 *rec)
{
	unsigned int val = 0;
	unsigned int i;

	if (!f->max)
		return 0;

	for (i = 0; i < f->size; i++)
		val = (val << 8) | (rec[f->ofs + i] >> f->bofs);

	return val & (f->max - 1);
}

static void cyttsp4_report_touch(struct cyttsp4 *ts, const u8 *rec)
{
	const struct cyttsp4_sysinfo *si = &ts->si;
	struct input_dev *input = ts->input;
	unsigned int t, x, y, p, major, minor;

	t = cyttsp4_get_field(&si->tch[CY_TCH_T], rec);
	if (t >= ts->num_slots) {
		dev_err_ratelimited(ts->dev, "bad track id %u\n", t);
		return;
	}

	/* a lift-off record repeats the last position: release the slot */
	if (cyttsp4_get_field(&si->tch[CY_TCH_E], rec) == CY_EV_LIFTOFF)
		return;

	x = min(cyttsp4_get_field(&si->tch[CY_TCH_X], rec), ts->prop.max_x);
	y = min(cyttsp4_get_field(&si->tch[CY_TCH_Y], rec), ts->prop.max_y);
	p = cyttsp4_get_field(&si->tch[CY_TCH_P], rec);

	input_mt_slot(input, t);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
	touchscreen_report_pos(input, &ts->prop, x, y, true);
	input_report_abs(input, ABS_MT_PRESSURE, p);

	if (ts->ext_fields) {
		major = cyttsp4_get_field(&si->tch[CY_TCH_MAJ], rec);
		minor = cyttsp4_get_field(&si->tch[CY_TCH_MIN], rec);
		/* a contact with pressure but no size: the smallest size */
		if (p && !major) {
			major = 1;
			minor = 1;
		}
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, major);
		input_report_abs(input, ABS_MT_TOUCH_MINOR, minor);
		input_report_abs(input, ABS_MT_ORIENTATION,
				 (s8)cyttsp4_get_field(&si->tch[CY_TCH_OR], rec));
	}
}

/* called with system_lock held, in operational mode */
static void cyttsp4_xy_worker(struct cyttsp4 *ts)
{
	const struct cyttsp4_sysinfo *si = &ts->si;
	unsigned int num, i;
	u8 rep_len, rep_stat, tt_stat;
	int error;

	error = cyttsp4_read(ts, CY_REG_BASE, ts->xy_mode, si->mode_size);
	if (error) {
		dev_err_ratelimited(ts->dev, "status read failed: %d\n", error);
		return;
	}

	if (!ts->input_ready)
		return;

	rep_len = ts->xy_mode[si->rep_ofs];
	rep_stat = ts->xy_mode[si->rep_ofs + 1];
	tt_stat = ts->xy_mode[si->tt_stat_ofs];
	num = FIELD_GET(CY_TT_STAT_NUM_TOUCHES, tt_stat);

	if (rep_len == 0 && num > 0) {
		dev_err_ratelimited(ts->dev, "report length 0 with %u touches\n",
				    num);
		return;
	}

	if (rep_stat & CY_REP_STAT_BAD_PKT)
		return;

	if (num > si->max_tchs) {
		dev_err_ratelimited(ts->dev, "%u touches, at most %u\n", num,
				    si->max_tchs);
		num = si->max_tchs;
	}

	if (num) {
		error = cyttsp4_read(ts, si->tt_stat_ofs + 1, ts->xy_data,
				     num * si->tch_rec_size);
		if (error) {
			dev_err_ratelimited(ts->dev, "touch read failed: %d\n",
					    error);
			return;
		}
	}

	for (i = 0; i < num; i++)
		cyttsp4_report_touch(ts, ts->xy_data + i * si->tch_rec_size);

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static irqreturn_t cyttsp4_irq(int irq, void *handle)
{
	struct cyttsp4 *ts = handle;
	enum cyttsp4_mode cur_mode;
	u8 mode[2];
	int error;

	mutex_lock(&ts->system_lock);

	error = cyttsp4_read(ts, CY_REG_BASE, mode, sizeof(mode));
	if (error) {
		dev_err_ratelimited(ts->dev, "mode read failed: %d\n", error);
		goto out;
	}

	if (IS_BOOTLOADER(mode[0], mode[1])) {
		if (ts->mode == CY_MODE_BOOTLOADER) {
			/* bootloader heartbeat */
			wake_up(&ts->wait_q);
		} else if (ts->mode != CY_MODE_UNKNOWN) {
			/* the controller restarted on its own */
			ts->mode = CY_MODE_UNKNOWN;
			cyttsp4_queue_startup(ts);
		} else {
			/*
			 * Do not wake the waiter yet: an early heartbeat may
			 * come before the reset.
			 */
			ts->mode = CY_MODE_BOOTLOADER;
		}
		goto out;
	}

	switch (mode[0] & CY_HST_MODE) {
	case CY_HST_OPERATE:
		cur_mode = CY_MODE_OPERATIONAL;
		break;
	case CY_HST_CAT:
		cur_mode = CY_MODE_CAT;
		break;
	case CY_HST_SYSINFO:
		cur_mode = CY_MODE_SYSINFO;
		break;
	default:
		dev_err_ratelimited(ts->dev, "unknown host mode 0x%02x\n",
				    mode[0]);
		cur_mode = CY_MODE_UNKNOWN;
		break;
	}

	if (ts->int_status & CY_INT_IGNORE)
		goto out;

	if (ts->int_status & CY_INT_AWAKE) {
		ts->int_status &= ~CY_INT_AWAKE;
		wake_up(&ts->wait_q);
		goto handshake;
	}

	if ((ts->int_status & CY_INT_MODE_CHANGE) &&
	    !(mode[0] & CY_HST_MODE_CHANGE)) {
		ts->int_status &= ~CY_INT_MODE_CHANGE;
		ts->mode = cur_mode;
		wake_up(&ts->wait_q);
		goto handshake;
	}

	if (!(mode[0] & CY_HST_MODE_CHANGE) && ts->mode != cur_mode) {
		dev_err(ts->dev, "unexpected mode change %d -> %d, restarting\n",
			ts->mode, cur_mode);
		cyttsp4_queue_startup(ts);
		goto out;
	}

	if (ts->mode == CY_MODE_OPERATIONAL)
		cyttsp4_xy_worker(ts);

handshake:
	cyttsp4_handshake(ts, mode[0]);

	/* a level interrupt needs the controller to release the line first */
	if (ts->level_irq)
		udelay(CY_LEVEL_IRQ_UDELAY);

out:
	mutex_unlock(&ts->system_lock);
	return IRQ_HANDLED;
}

/* called with ctrl_lock held */
static int cyttsp4_sleep_locked(struct cyttsp4 *ts)
{
	u8 mode[2];
	int error;

	mutex_lock(&ts->system_lock);
	if (ts->sleep_state == SS_SLEEP_ON) {
		mutex_unlock(&ts->system_lock);
		return 0;
	}
	ts->sleep_state = SS_SLEEPING;
	mutex_unlock(&ts->system_lock);

	cyttsp4_stop_wd(ts);

	/* wait for a running IRQ thread to finish */
	disable_irq(ts->irq);

	mutex_lock(&ts->system_lock);
	error = cyttsp4_read(ts, CY_REG_BASE, mode, sizeof(mode));
	if (!error && IS_BOOTLOADER(mode[0], mode[1]))
		error = -EBUSY;
	if (!error) {
		mode[0] |= CY_HST_SLEEP;
		error = cyttsp4_write(ts, CY_REG_BASE, &mode[0],
				      sizeof(mode[0]));
	}
	mutex_unlock(&ts->system_lock);

	if (!error)
		msleep(CY_SLEEP_SETTLE_MS);

	mutex_lock(&ts->system_lock);
	if (error) {
		ts->sleep_state = SS_SLEEP_OFF;
	} else {
		ts->sleep_state = SS_SLEEP_ON;
		ts->int_status |= CY_INT_IGNORE;
	}
	mutex_unlock(&ts->system_lock);

	enable_irq(ts->irq);

	if (error)
		dev_err(ts->dev, "deep sleep failed: %d\n", error);

	return error;
}

/* called with ctrl_lock held */
static int cyttsp4_wake_locked(struct cyttsp4 *ts)
{
	u8 mode;

	mutex_lock(&ts->system_lock);
	if (ts->sleep_state == SS_SLEEP_OFF) {
		mutex_unlock(&ts->system_lock);
		cyttsp4_start_wd(ts);
		return 0;
	}
	ts->int_status &= ~CY_INT_IGNORE;
	ts->int_status |= CY_INT_AWAKE;
	ts->sleep_state = SS_WAKING;
	/* any bus transfer wakes the controller; it may not answer this one */
	cyttsp4_read(ts, CY_REG_BASE, &mode, sizeof(mode));
	mutex_unlock(&ts->system_lock);

	if (!wait_event_timeout(ts->wait_q, !(ts->int_status & CY_INT_AWAKE),
				msecs_to_jiffies(CY_WAKEUP_TIMEOUT_MS))) {
		dev_err(ts->dev, "no wake-up interrupt, restarting\n");
		mutex_lock(&ts->system_lock);
		ts->int_status &= ~CY_INT_AWAKE;
		cyttsp4_queue_startup(ts);
		mutex_unlock(&ts->system_lock);
	}

	mutex_lock(&ts->system_lock);
	ts->sleep_state = SS_SLEEP_OFF;
	mutex_unlock(&ts->system_lock);

	cyttsp4_start_wd(ts);

	return 0;
}

/*
 * No restart and no watchdog while the system sleeps: both talk to the bus.
 * A controller that cannot enter deep sleep stays awake, and the system
 * suspend goes on.
 */
static int cyttsp4_suspend(struct device *dev)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->system_lock);
	ts->suspended = true;
	mutex_unlock(&ts->system_lock);

	cancel_work_sync(&ts->startup_work);

	mutex_lock(&ts->ctrl_lock);
	cyttsp4_stop_wd(ts);
	/* a wake-up source keeps scanning; the PM core arms its IRQ */
	if (!device_may_wakeup(dev))
		cyttsp4_sleep_locked(ts);
	mutex_unlock(&ts->ctrl_lock);

	return 0;
}

static int cyttsp4_resume(struct device *dev)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	int error;

	mutex_lock(&ts->system_lock);
	ts->suspended = false;
	mutex_unlock(&ts->system_lock);

	mutex_lock(&ts->ctrl_lock);
	error = cyttsp4_wake_locked(ts);
	mutex_unlock(&ts->ctrl_lock);

	return error;
}

static DEFINE_SIMPLE_DEV_PM_OPS(cyttsp4_pm_ops, cyttsp4_suspend,
				cyttsp4_resume);

static int cyttsp4_setup_input(struct cyttsp4 *ts)
{
	const struct cyttsp4_sysinfo *si = &ts->si;
	struct input_dev *input = ts->input;
	unsigned int max_p;
	int error;

	/* controller axes, before any swap or inversion */
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, si->res_x - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, si->res_y - 1, 0, 0);
	max_p = si->max_p;
	if (!max_p)
		max_p = si->tch[CY_TCH_P].max ? si->tch[CY_TCH_P].max - 1 : U8_MAX;
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, max_p, 0, 0);

	ts->ext_fields = si->tch_rec_size > CY_TMA1036_TCH_REC_SIZE &&
			 si->tch[CY_TCH_MAJ].max;
	if (ts->ext_fields) {
		input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0,
				     si->tch[CY_TCH_MAJ].max - 1, 0, 0);
		input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0,
				     si->tch[CY_TCH_MIN].max ?
				     si->tch[CY_TCH_MIN].max - 1 : 0, 0, 0);
		input_set_abs_params(input, ABS_MT_ORIENTATION, -128, 127, 0, 0);
	}

	touchscreen_parse_properties(input, true, &ts->prop);

	/*
	 * Multitouch only: the single-touch emulation (BTN_TOUCH, ABS_X/Y)
	 * of INPUT_MT_DIRECT would follow the slot events of every frame.
	 */
	ts->num_slots = si->tch[CY_TCH_T].max;
	error = input_mt_init_slots(input, ts->num_slots, INPUT_MT_DROP_UNUSED);
	if (error)
		return error;
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	return input_register_device(input);
}

static void cyttsp4_stop(void *data)
{
	struct cyttsp4 *ts = data;

	mutex_lock(&ts->system_lock);
	ts->stopped = true;
	mutex_unlock(&ts->system_lock);

	cancel_work_sync(&ts->startup_work);
	cancel_delayed_work_sync(&ts->watchdog_work);
}

static int cyttsp4_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cyttsp4 *ts;
	int error;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -ENXIO, "I2C functionality not supported\n");

	if (!client->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	ts->dev = dev;
	ts->irq = client->irq;
	i2c_set_clientdata(client, ts);

	mutex_init(&ts->ctrl_lock);
	mutex_init(&ts->system_lock);
	init_waitqueue_head(&ts->wait_q);
	INIT_WORK(&ts->startup_work, cyttsp4_startup_work);
	INIT_DELAYED_WORK(&ts->watchdog_work, cyttsp4_watchdog_work);

	error = devm_regulator_get_enable_optional(dev, "vdd");
	if (error && error != -ENODEV)
		return dev_err_probe(dev, error, "vdd supply\n");
	if (!error)
		msleep(CY_POWER_ON_DELAY_MS);

	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "reset GPIO\n");

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

	ts->input->name = CYTTSP4_NAME;
	ts->input->phys = devm_kasprintf(dev, GFP_KERNEL, "%s/input0",
					 dev_name(dev));
	if (!ts->input->phys)
		return -ENOMEM;
	ts->input->id.bustype = BUS_I2C;

	ts->level_irq = irq_get_trigger_type(ts->irq) & IRQ_TYPE_LEVEL_MASK;

	error = devm_request_threaded_irq(dev, ts->irq, NULL, cyttsp4_irq,
					  IRQF_ONESHOT, client->name, ts);
	if (error)
		return dev_err_probe(dev, error, "request IRQ %d\n", ts->irq);

	error = devm_add_action_or_reset(dev, cyttsp4_stop, ts);
	if (error)
		return error;

	error = cyttsp4_startup(ts);
	if (error)
		return dev_err_probe(dev, error, "controller start failed\n");

	error = cyttsp4_setup_input(ts);
	if (error)
		return dev_err_probe(dev, error, "input device\n");

	mutex_lock(&ts->system_lock);
	ts->input_ready = true;
	mutex_unlock(&ts->system_lock);

	return 0;
}

static const struct i2c_device_id cyttsp4_i2c_id[] = {
	{ "cyttsp4" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cyttsp4_i2c_id);

static const struct of_device_id cyttsp4_of_match[] = {
	{ .compatible = "cypress,cyttsp4" },
	{ }
};
MODULE_DEVICE_TABLE(of, cyttsp4_of_match);

static struct i2c_driver cyttsp4_i2c_driver = {
	.driver = {
		.name = "cyttsp4",
		.of_match_table = cyttsp4_of_match,
		.pm = pm_sleep_ptr(&cyttsp4_pm_ops),
		/* the start takes up to a few seconds when the controller is silent */
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = cyttsp4_probe,
	.id_table = cyttsp4_i2c_id,
};
module_i2c_driver(cyttsp4_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product Gen4 I2C touchscreen driver");
MODULE_AUTHOR("Cypress");
