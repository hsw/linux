// SPDX-License-Identifier: GPL-2.0-only
/*
 * Active-semi ACT8931 PMU: regulators and charger state.
 *
 * Every register write passes one compiled-in rule per register before it
 * reaches the bus, whatever asked for it (a regulator op, the regulator
 * core, debugfs). The rules only keep the PMU from damaging what it feeds;
 * they do not keep a board running:
 *  - only the registers of the datasheet map are written; the undocumented
 *    and reserved addresses (0x02, 0x03, 0x0b, 0x70, the suspend set-points
 *    of the ACT8865 family) and the read-only APCH state are refused;
 *  - bits the datasheet calls reserved must keep the value read back from
 *    the PMU just before the write;
 *  - no output voltage selector, VSET1 or VSET2, may go above the value
 *    the board's vendor firmware runs it at (the caps below); any lower
 *    value is allowed;
 *  - the battery protections may not be relaxed: the system voltage
 *    threshold stays at or above 3.0 V, the charge safety timers at or
 *    below 80 min and 5 h (never off), the input over-voltage threshold
 *    at 6.6 V.
 * The caps are not taken from the device tree, so a wrong tree cannot
 * raise them.
 *
 * The ON bit of an output is edge-triggered: the output follows the last
 * 0->1 or 1->0 edge written to it, so ON alone is not the output state.
 * An output counts as enabled when ON and OK (power good) are both set.
 *
 * APCH 0x78 bits 7:4 are interrupt status bits that clear on read. The
 * register is marked precious, so regmap debugfs leaves it alone, and the
 * write rules never read it. act8931_charger_read() does, at probe, every
 * 5 s and on every read of a charger property, and each of these reads
 * clears the bits.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#define ACT8931_SYS0		0x00
#define ACT8931_SYS1		0x01
#define ACT8931_DCDC1_VSET1	0x20
#define ACT8931_DCDC1_VSET2	0x21
#define ACT8931_DCDC1_CTRL	0x22
#define ACT8931_DCDC2_VSET1	0x30
#define ACT8931_DCDC2_VSET2	0x31
#define ACT8931_DCDC2_CTRL	0x32
#define ACT8931_DCDC3_VSET1	0x40
#define ACT8931_DCDC3_VSET2	0x41
#define ACT8931_DCDC3_CTRL	0x42
#define ACT8931_LDO1_VSET	0x50
#define ACT8931_LDO1_CTRL	0x51
#define ACT8931_LDO2_VSET	0x54
#define ACT8931_LDO2_CTRL	0x55
#define ACT8931_LDO3_VSET	0x60
#define ACT8931_LDO3_CTRL	0x61
#define ACT8931_LDO4_VSET	0x64
#define ACT8931_LDO4_CTRL	0x65
#define ACT8931_APCH_CFG	0x71
#define ACT8931_APCH_STATUS	0x78
#define ACT8931_APCH_INTR	0x79
#define ACT8931_APCH_STATE	0x7a

#define ACT8931_VSEL_MASK	GENMASK(5, 0)
#define ACT8931_CTRL_ON		BIT(7)
#define ACT8931_CTRL_OK		BIT(0)	/* power good */
#define ACT8931_N_VOLTAGES	64

#define ACT8931_STATUS_CHGDAT	BIT(0)	/* end of charge */
#define ACT8931_STATUS_INDAT	BIT(1)	/* valid input at CHGIN */
#define ACT8931_STATUS_TEMPDAT	BIT(2)	/* battery temperature in range */
#define ACT8931_STATUS_TIMRDAT	BIT(3)	/* charge timer expired */
#define ACT8931_STATE_CSTATE	GENMASK(5, 4)
#define ACT8931_STATE_ACINSTAT	BIT(1)

#define ACT8931_CSTATE_OFF	0	/* suspended, disabled or fault */
#define ACT8931_CSTATE_EOC	1
#define ACT8931_CSTATE_FAST	2	/* fast charge or top-off */
#define ACT8931_CSTATE_PRE	3

#define ACT8931_POLL_MS		5000

/*
 * Soft-start: 400 us for the DC/DC converters, 140 us typical for an LDO
 * at 2.9 V (datasheet); no figure is given for a VSET step. The longer
 * one bounds both an enable and a voltage raise.
 */
#define ACT8931_SOFTSTART_US	400

/*
 * Voltage caps: the selectors the board's vendor kernel runs the outputs
 * at (ONYX BOOX Robinson Crusoe 2). DCDC1 0x3b is the vendor's raised
 * VCCIO while its Wi-Fi/BT module is powered; 0x3a otherwise.
 */
#define ACT8931_CAP_DCDC1	0x3b
#define ACT8931_CAP_DCDC2	0x1b
#define ACT8931_CAP_DCDC3	0x1c
#define ACT8931_CAP_LDO		0x37

enum {
	ACT8931_ID_DCDC1,
	ACT8931_ID_DCDC2,
	ACT8931_ID_DCDC3,
	ACT8931_ID_LDO1,
	ACT8931_ID_LDO2,
	ACT8931_ID_LDO3,
	ACT8931_ID_LDO4,
	ACT8931_NUM_RAILS,
};

struct act8931_rail {
	const char *name;
	const char *of_name;
	unsigned int vset1;	/* DCDC only: the VSEL-low register */
	unsigned int vset;	/* LDO: the one VSET; DCDC: VSET2 */
	unsigned int ctrl;
	unsigned int cap;	/* highest selector a write may set */
	bool switchable;	/* may get enable/disable ops */
	bool settable;		/* may get set_voltage ops (DCDC: VSET2) */
};

static const struct act8931_rail act8931_rails[ACT8931_NUM_RAILS] = {
	[ACT8931_ID_DCDC1] = { "act8931-dcdc1", "DCDC_REG1", ACT8931_DCDC1_VSET1,
			       ACT8931_DCDC1_VSET2, ACT8931_DCDC1_CTRL,
			       ACT8931_CAP_DCDC1 },
	[ACT8931_ID_DCDC2] = { "act8931-dcdc2", "DCDC_REG2", ACT8931_DCDC2_VSET1,
			       ACT8931_DCDC2_VSET2, ACT8931_DCDC2_CTRL,
			       ACT8931_CAP_DCDC2 },
	[ACT8931_ID_DCDC3] = { "act8931-dcdc3", "DCDC_REG3", ACT8931_DCDC3_VSET1,
			       ACT8931_DCDC3_VSET2, ACT8931_DCDC3_CTRL,
			       ACT8931_CAP_DCDC3, false, true },
	[ACT8931_ID_LDO1] = { "act8931-ldo1", "LDO_REG1", 0,
			      ACT8931_LDO1_VSET, ACT8931_LDO1_CTRL,
			      ACT8931_CAP_LDO },
	[ACT8931_ID_LDO2] = { "act8931-ldo2", "LDO_REG2", 0,
			      ACT8931_LDO2_VSET, ACT8931_LDO2_CTRL,
			      ACT8931_CAP_LDO },
	[ACT8931_ID_LDO3] = { "act8931-ldo3", "LDO_REG3", 0,
			      ACT8931_LDO3_VSET, ACT8931_LDO3_CTRL,
			      ACT8931_CAP_LDO },
	[ACT8931_ID_LDO4] = { "act8931-ldo4", "LDO_REG4", 0,
			      ACT8931_LDO4_VSET, ACT8931_LDO4_CTRL,
			      ACT8931_CAP_LDO, true },
};

struct act8931 {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	bool vsel_high;
	struct regulator_desc desc[ACT8931_NUM_RAILS];
	struct power_supply *charger;
	struct delayed_work poll;
	struct mutex dbg_lock;	/* dbg_result */
	char dbg_result[48];
	unsigned int last_status;
	unsigned int last_state;
};

/* ACT8931 output voltage table: three linear ranges of the 6-bit selector */
static const struct linear_range act8931_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(600000, 0, 23, 25000),
	REGULATOR_LINEAR_RANGE(1200000, 24, 47, 50000),
	REGULATOR_LINEAR_RANGE(2400000, 48, 63, 100000),
};

/*
 * The ON bit is edge-triggered: the output follows the last 0->1 or 1->0
 * edge written to it, so ON alone is not the output state. An output
 * counts as enabled when ON and OK (power good) are both set. OK alone
 * says nothing: it reads 1 on LDOs that are off.
 */
static bool act8931_ctrl_enabled(unsigned int ctrl)
{
	return (ctrl & ACT8931_CTRL_ON) && (ctrl & ACT8931_CTRL_OK);
}

static int act8931_is_enabled(struct regulator_dev *rdev)
{
	unsigned int ctrl;
	int ret;

	ret = regmap_read(rdev_get_regmap(rdev), rdev->desc->enable_reg, &ctrl);
	if (ret)
		return ret;
	return act8931_ctrl_enabled(ctrl);
}

/*
 * Enable: a rising edge of ON, written as ON 0 then ON 1 from the value
 * read back. Nothing is written when the output is on already: the
 * regulator core enables an always-on output at registration without
 * asking first, and a 0 written to a live ON bit would switch it off.
 *
 * An output whose ON reads 1 without power good is never toggled: it may
 * still be ramping up, so it gets the soft-start time; without power good
 * after that it is overloaded or failing, and the falling edge of a
 * toggle would cut it while its load may still run from it. That is an
 * error, and the output is left as it is.
 */
static int act8931_enable(struct regulator_dev *rdev)
{
	struct act8931 *chip = rdev_get_drvdata(rdev);
	struct regmap *map = rdev_get_regmap(rdev);
	unsigned int reg = rdev->desc->enable_reg;
	unsigned int ctrl;
	int ret;

	ret = regmap_read(map, reg, &ctrl);
	if (ret)
		return ret;
	if (act8931_ctrl_enabled(ctrl))
		return 0;
	if (ctrl & ACT8931_CTRL_ON) {
		fsleep(ACT8931_SOFTSTART_US);
		ret = regmap_read(map, reg, &ctrl);
		if (ret)
			return ret;
		if (act8931_ctrl_enabled(ctrl))
			return 0;
		if (ctrl & ACT8931_CTRL_ON) {
			dev_err(chip->dev,
				"%s: ON without power good (CTRL 0x%02x): not toggled\n",
				rdev->desc->name, ctrl);
			return -EIO;
		}
	}

	ret = regmap_write(map, reg, ctrl & ~ACT8931_CTRL_ON);
	if (ret)
		return ret;
	return regmap_write(map, reg, ctrl | ACT8931_CTRL_ON);
}

/*
 * Disable: a falling edge, ON 1 then ON 0. An output whose ON reads 0
 * had a falling edge last and is off; nothing is written then.
 */
static int act8931_disable(struct regulator_dev *rdev)
{
	struct regmap *map = rdev_get_regmap(rdev);
	unsigned int reg = rdev->desc->enable_reg;
	unsigned int ctrl;
	int ret;

	ret = regmap_read(map, reg, &ctrl);
	if (ret)
		return ret;
	if (!(ctrl & ACT8931_CTRL_ON))
		return 0;

	ret = regmap_write(map, reg, ctrl | ACT8931_CTRL_ON);
	if (ret)
		return ret;
	return regmap_write(map, reg, ctrl & ~ACT8931_CTRL_ON);
}

/*
 * The selector is checked against the cap here and again in the write
 * rules; regulator_set_voltage_sel_regmap() keeps the reserved bits.
 */
static int act8931_set_voltage_sel(struct regulator_dev *rdev,
				   unsigned int sel)
{
	if (sel > act8931_rails[rdev_get_id(rdev)].cap)
		return -EPERM;
	return regulator_set_voltage_sel_regmap(rdev, sel);
}

/*
 * The datasheet gives no slew rate for a VSET change; a raise is given
 * the soft-start time, a drop needs no wait before the load may rely on
 * it.
 */
static int act8931_set_voltage_time_sel(struct regulator_dev *rdev,
					unsigned int old_sel,
					unsigned int new_sel)
{
	return new_sel > old_sel ? ACT8931_SOFTSTART_US : 0;
}

static const struct regulator_ops act8931_ro_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.is_enabled		= act8931_is_enabled,
};

static const struct regulator_ops act8931_set_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= act8931_set_voltage_sel,
	.set_voltage_time_sel	= act8931_set_voltage_time_sel,
	.is_enabled		= act8931_is_enabled,
};

static const struct regulator_ops act8931_switch_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.is_enabled		= act8931_is_enabled,
	.enable			= act8931_enable,
	.disable		= act8931_disable,
};

static bool act8931_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ACT8931_SYS0:
	case ACT8931_SYS1:
	case ACT8931_DCDC1_VSET1 ... ACT8931_DCDC1_CTRL:
	case ACT8931_DCDC2_VSET1 ... ACT8931_DCDC2_CTRL:
	case ACT8931_DCDC3_VSET1 ... ACT8931_DCDC3_CTRL:
	case ACT8931_LDO1_VSET:
	case ACT8931_LDO1_CTRL:
	case ACT8931_LDO2_VSET:
	case ACT8931_LDO2_CTRL:
	case ACT8931_LDO3_VSET:
	case ACT8931_LDO3_CTRL:
	case ACT8931_LDO4_VSET:
	case ACT8931_LDO4_CTRL:
	case ACT8931_APCH_CFG:
	case ACT8931_APCH_STATUS:
	case ACT8931_APCH_INTR:
	case ACT8931_APCH_STATE:
		return true;
	default:
		return false;
	}
}

/*
 * Write rules. @keep: bits that must equal the register's current value
 * (reserved bits). @lim: fields whose masked value must lie in
 * [min, max]; an entry with a zero mask is unused. Bits the datasheet
 * calls read-only (OK, nSYSSTAT, the APCH status data) are ignored by
 * the PMU on a write and are not checked.
 */
struct act8931_limit {
	u8 mask;
	u8 min;
	u8 max;
};

struct act8931_wrule {
	u8 reg;
	u8 keep;
	struct act8931_limit lim[3];
};

#define ACT8931_VSET_RESERVED	GENMASK(7, 6)
#define ACT8931_SYS0_SYSLEV	GENMASK(3, 0)
#define ACT8931_SYS1_RESERVED	(GENMASK(7, 6) | BIT(4))
#define ACT8931_DCDC3_CTRL_RESERVED BIT(6)
#define ACT8931_APCH_CFG_RESERVED BIT(6)
#define ACT8931_APCH_CFG_TOTTIMO GENMASK(5, 4)
#define ACT8931_APCH_CFG_PRETIMO GENMASK(3, 2)
#define ACT8931_APCH_CFG_OVPSET	GENMASK(1, 0)

#define ACT8931_VSET_RULE(_reg, _cap)					\
	{ .reg = (_reg), .keep = ACT8931_VSET_RESERVED,			\
	  .lim = { { ACT8931_VSEL_MASK, 0, (_cap) } } }
#define ACT8931_FREE_RULE(_reg)	{ .reg = (_reg) }

static const struct act8931_wrule act8931_wrules[] = {
	/* SYSLEV: 2.3 V + 100 mV per step; not below 3.0 V */
	{ .reg = ACT8931_SYS0,
	  .lim = { { ACT8931_SYS0_SYSLEV, 0x7, 0xf } } },
	{ .reg = ACT8931_SYS1, .keep = ACT8931_SYS1_RESERVED },
	ACT8931_VSET_RULE(ACT8931_DCDC1_VSET1, ACT8931_CAP_DCDC1),
	ACT8931_VSET_RULE(ACT8931_DCDC1_VSET2, ACT8931_CAP_DCDC1),
	ACT8931_FREE_RULE(ACT8931_DCDC1_CTRL),
	ACT8931_VSET_RULE(ACT8931_DCDC2_VSET1, ACT8931_CAP_DCDC2),
	ACT8931_VSET_RULE(ACT8931_DCDC2_VSET2, ACT8931_CAP_DCDC2),
	ACT8931_FREE_RULE(ACT8931_DCDC2_CTRL),
	ACT8931_VSET_RULE(ACT8931_DCDC3_VSET1, ACT8931_CAP_DCDC3),
	ACT8931_VSET_RULE(ACT8931_DCDC3_VSET2, ACT8931_CAP_DCDC3),
	{ .reg = ACT8931_DCDC3_CTRL, .keep = ACT8931_DCDC3_CTRL_RESERVED },
	ACT8931_VSET_RULE(ACT8931_LDO1_VSET, ACT8931_CAP_LDO),
	ACT8931_FREE_RULE(ACT8931_LDO1_CTRL),
	ACT8931_VSET_RULE(ACT8931_LDO2_VSET, ACT8931_CAP_LDO),
	ACT8931_FREE_RULE(ACT8931_LDO2_CTRL),
	ACT8931_VSET_RULE(ACT8931_LDO3_VSET, ACT8931_CAP_LDO),
	ACT8931_FREE_RULE(ACT8931_LDO3_CTRL),
	ACT8931_VSET_RULE(ACT8931_LDO4_VSET, ACT8931_CAP_LDO),
	ACT8931_FREE_RULE(ACT8931_LDO4_CTRL),
	/*
	 * Charge safety timers no longer than 80 min precondition and 5 h
	 * total (code 3 = no time-out), input OVP at 6.6 V; SUSCHG is free.
	 */
	{ .reg = ACT8931_APCH_CFG, .keep = ACT8931_APCH_CFG_RESERVED,
	  .lim = { { ACT8931_APCH_CFG_TOTTIMO, 0, FIELD_PREP_CONST(ACT8931_APCH_CFG_TOTTIMO, 2) },
		   { ACT8931_APCH_CFG_PRETIMO, 0, FIELD_PREP_CONST(ACT8931_APCH_CFG_PRETIMO, 2) },
		   { ACT8931_APCH_CFG_OVPSET, 0, 0 } } },
	ACT8931_FREE_RULE(ACT8931_APCH_STATUS),
	ACT8931_FREE_RULE(ACT8931_APCH_INTR),
};

static const struct act8931_wrule *act8931_find_wrule(unsigned int reg)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(act8931_wrules); i++)
		if (act8931_wrules[i].reg == reg)
			return &act8931_wrules[i];
	return NULL;
}

static bool act8931_writeable_reg(struct device *dev, unsigned int reg)
{
	return act8931_find_wrule(reg);
}

static bool act8931_precious_reg(struct device *dev, unsigned int reg)
{
	return reg == ACT8931_APCH_STATUS;
}

static bool act8931_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static int act8931_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct act8931 *chip = context;
	int ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0)
		return ret;

	*val = ret;
	return 0;
}

/* The one path to the bus for a write: the rule of the register first */
static int act8931_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct act8931 *chip = context;
	const struct act8931_wrule *rule = act8931_find_wrule(reg);
	const char *why = NULL;
	int i, cur;

	if (!rule) {
		why = "not a writable register";
	} else if (val > 0xff) {
		why = "not a byte";
	} else {
		for (i = 0; i < ARRAY_SIZE(rule->lim); i++) {
			const struct act8931_limit *lim = &rule->lim[i];

			if (lim->mask && ((val & lim->mask) < lim->min ||
					  (val & lim->mask) > lim->max)) {
				why = "outside the limit";
				break;
			}
		}
	}

	if (!why && rule->keep) {
		cur = i2c_smbus_read_byte_data(chip->client, reg);
		if (cur < 0)
			return cur;
		if ((cur ^ val) & rule->keep)
			why = "changes reserved bits";
	}

	if (why) {
		dev_err_ratelimited(chip->dev,
				    "write of 0x%02x to 0x%02x refused: %s\n",
				    val, reg, why);
		return -EPERM;
	}

	return i2c_smbus_write_byte_data(chip->client, reg, val);
}

static const struct regmap_config act8931_regmap_config = {
	.name		= "act8931",
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= ACT8931_APCH_STATE,
	.readable_reg	= act8931_readable_reg,
	.writeable_reg	= act8931_writeable_reg,
	.precious_reg	= act8931_precious_reg,
	.volatile_reg	= act8931_volatile_reg,
	.reg_read	= act8931_reg_read,
	.reg_write	= act8931_reg_write,
	.cache_type	= REGCACHE_NONE,
};

static unsigned int act8931_sel_to_uv(unsigned int sel)
{
	if (sel < 24)
		return 600000 + sel * 25000;
	if (sel < 48)
		return 1200000 + (sel - 24) * 50000;
	return 2400000 + (sel - 48) * 100000;
}

/*
 * The voltage the board states for an output (regulators/<name> with
 * regulator-min-microvolt == regulator-max-microvolt), as a selector.
 * Only compared with the hardware; never handed to the regulator core.
 */
static int act8931_board_sel(struct act8931 *chip, struct device_node *regs,
			     const struct act8931_rail *rail)
{
	struct device_node *np;
	u32 min_uv, max_uv;
	unsigned int sel;
	int ret = -ENOENT;

	np = of_get_child_by_name(regs, rail->of_name);
	if (!np)
		return -ENOENT;

	if (of_property_read_u32(np, "regulator-min-microvolt", &min_uv) ||
	    of_property_read_u32(np, "regulator-max-microvolt", &max_uv) ||
	    min_uv != max_uv)
		goto out;

	for (sel = 0; sel < ACT8931_N_VOLTAGES; sel++) {
		if (act8931_sel_to_uv(sel) == min_uv) {
			ret = sel;
			goto out;
		}
	}
	dev_warn(chip->dev, "%s: %u uV is not an ACT8931 voltage\n",
		 rail->of_name, min_uv);
out:
	of_node_put(np);
	return ret;
}

static int act8931_read_rail(struct act8931 *chip,
			     const struct act8931_rail *rail,
			     unsigned int *vset1, unsigned int *vset,
			     unsigned int *ctrl)
{
	int ret;

	*vset1 = 0;
	if (rail->vset1) {
		ret = regmap_read(chip->regmap, rail->vset1, vset1);
		if (ret)
			return ret;
	}
	ret = regmap_read(chip->regmap, rail->vset, vset);
	if (ret)
		return ret;
	return regmap_read(chip->regmap, rail->ctrl, ctrl);
}

/* The DCDC selector in effect: VSET2 with VSEL high, else VSET1 */
static unsigned int act8931_live_sel(struct act8931 *chip,
				     const struct act8931_rail *rail,
				     unsigned int vset1, unsigned int vset)
{
	if (rail->vset1 && !chip->vsel_high)
		return vset1 & ACT8931_VSEL_MASK;
	return vset & ACT8931_VSEL_MASK;
}

/*
 * Logs an output's registers and compares its selector with the board's.
 * Returns the selector in effect, or a negative error; *board is the
 * board's selector or a negative error.
 */
static int act8931_report_rail(struct act8931 *chip, struct device_node *regs,
			       const struct act8931_rail *rail, int *board)
{
	unsigned int vset1, vset, ctrl, sel;
	int ret;

	*board = regs ? act8931_board_sel(chip, regs, rail) : -ENOENT;
	ret = act8931_read_rail(chip, rail, &vset1, &vset, &ctrl);
	if (ret) {
		dev_warn(chip->dev, "%s: read failed: %d\n", rail->of_name, ret);
		return ret;
	}
	sel = act8931_live_sel(chip, rail, vset1, vset);

	if (rail->vset1)
		dev_info(chip->dev,
			 "%s: VSET1 0x%02x VSET2 0x%02x, VSEL %s: selector 0x%02x = %u uV; CTRL 0x%02x (ON %d OK %d): %s\n",
			 rail->of_name, vset1, vset,
			 chip->vsel_high ? "high" : "low", sel,
			 act8931_sel_to_uv(sel), ctrl,
			 !!(ctrl & ACT8931_CTRL_ON), !!(ctrl & ACT8931_CTRL_OK),
			 act8931_ctrl_enabled(ctrl) ? "on" : "off");
	else
		dev_info(chip->dev,
			 "%s: VSET 0x%02x: selector 0x%02x = %u uV; CTRL 0x%02x (ON %d OK %d): %s\n",
			 rail->of_name, vset, sel, act8931_sel_to_uv(sel), ctrl,
			 !!(ctrl & ACT8931_CTRL_ON), !!(ctrl & ACT8931_CTRL_OK),
			 act8931_ctrl_enabled(ctrl) ? "on" : "off");

	if (*board >= 0 && *board != sel)
		dev_warn(chip->dev,
			 "%s: selector 0x%02x (%u uV) differs from the board's 0x%02x (%u uV); left as it is\n",
			 rail->of_name, sel, act8931_sel_to_uv(sel), *board,
			 act8931_sel_to_uv(*board));
	return sel;
}

/*
 * An output gets enable/disable ops only when the board describes it as
 * always-on: until a consumer holds it, the regulator core would switch
 * an output with a disable op off as unused 30 s after boot. And only
 * when it runs at the board's voltage, within the cap: an enable switches
 * on whatever voltage is set.
 */
static bool act8931_may_switch(struct act8931 *chip, struct device_node *regs,
			       const struct act8931_rail *rail, int sel,
			       int board)
{
	struct device_node *np;
	bool always_on;

	if (!rail->switchable || !regs)
		return false;

	np = of_get_child_by_name(regs, rail->of_name);
	always_on = of_property_read_bool(np, "regulator-always-on");
	of_node_put(np);
	if (!always_on) {
		dev_info(chip->dev, "%s: not always-on on this board: no switching\n",
			 rail->of_name);
		return false;
	}
	if (sel < 0 || board < 0) {
		dev_warn(chip->dev, "%s: no selector or no board voltage: no switching\n",
			 rail->of_name);
		return false;
	}
	if (sel != board || sel > rail->cap) {
		dev_warn(chip->dev, "%s: selector 0x%02x is not the board's 0x%02x (cap 0x%02x): no switching\n",
			 rail->of_name, sel, board, rail->cap);
		return false;
	}
	return true;
}

/*
 * An output gets set_voltage ops only when the voltage it runs at lies in
 * its node's range, within the cap: with minimum = maximum the regulator
 * core would otherwise write the node's voltage at registration. Only
 * VSET2 is set, so only with VSEL high.
 */
static bool act8931_may_set(struct act8931 *chip, struct device_node *regs,
			    const struct act8931_rail *rail, int sel)
{
	struct device_node *np;
	u32 min_uv = 0, max_uv = 0;
	unsigned int uv;
	bool range;

	if (!rail->settable || !chip->vsel_high || !regs || sel < 0)
		return false;

	np = of_get_child_by_name(regs, rail->of_name);
	if (!np)
		return false;
	range = !of_property_read_u32(np, "regulator-min-microvolt", &min_uv) &&
		!of_property_read_u32(np, "regulator-max-microvolt", &max_uv);
	of_node_put(np);

	uv = act8931_sel_to_uv(sel);
	if (!range || uv < min_uv || uv > max_uv || sel > rail->cap) {
		dev_warn(chip->dev, "%s: selector 0x%02x (%u uV) is outside the board's range or the cap 0x%02x: no voltage setting\n",
			 rail->of_name, sel, uv, rail->cap);
		return false;
	}
	return true;
}

static int act8931_register_rail(struct act8931 *chip, int id,
				 struct device_node *regs, int sel, int board)
{
	const struct act8931_rail *rail = &act8931_rails[id];
	struct regulator_desc *desc = &chip->desc[id];
	struct regulator_config config = { };
	struct regulator_dev *rdev;

	desc->name = rail->name;
	desc->id = id;
	desc->type = REGULATOR_VOLTAGE;
	desc->owner = THIS_MODULE;
	desc->ops = &act8931_ro_ops;
	desc->n_voltages = ACT8931_N_VOLTAGES;
	desc->linear_ranges = act8931_voltage_ranges;
	desc->n_linear_ranges = ARRAY_SIZE(act8931_voltage_ranges);
	desc->vsel_reg = (rail->vset1 && !chip->vsel_high) ?
			 rail->vset1 : rail->vset;
	desc->vsel_mask = ACT8931_VSEL_MASK;
	desc->enable_reg = rail->ctrl;

	/*
	 * A read-only output is registered without of_match, init_data or
	 * of_node: the core gets empty constraints and, with no enable op,
	 * treats it as always on. An output with ops takes its constraints
	 * from its device tree node.
	 */
	if (act8931_may_switch(chip, regs, rail, sel, board)) {
		desc->ops = &act8931_switch_ops;
		desc->enable_time = ACT8931_SOFTSTART_US;
	} else if (act8931_may_set(chip, regs, rail, sel)) {
		desc->ops = &act8931_set_ops;
	}
	if (desc->ops != &act8931_ro_ops) {
		desc->of_match = rail->of_name;
		desc->regulators_node = "regulators";
	}

	config.dev = chip->dev;
	config.regmap = chip->regmap;
	config.driver_data = chip;

	rdev = devm_regulator_register(chip->dev, desc, &config);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);
	return 0;
}

/* The one reader of APCH 0x78 (bits 7:4 clear on read) */
static int act8931_charger_read(struct act8931 *chip, unsigned int *status,
				unsigned int *state)
{
	int ret;

	ret = regmap_read(chip->regmap, ACT8931_APCH_STATUS, status);
	if (ret)
		return ret;
	return regmap_read(chip->regmap, ACT8931_APCH_STATE, state);
}

static int act8931_charger_status(unsigned int status, unsigned int state)
{
	switch (FIELD_GET(ACT8931_STATE_CSTATE, state)) {
	case ACT8931_CSTATE_PRE:
	case ACT8931_CSTATE_FAST:
		return POWER_SUPPLY_STATUS_CHARGING;
	case ACT8931_CSTATE_EOC:
		if (status & ACT8931_STATUS_CHGDAT)
			return POWER_SUPPLY_STATUS_FULL;
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	default:
		if (status & ACT8931_STATUS_INDAT)
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}
}

static int act8931_charger_type(unsigned int state)
{
	switch (FIELD_GET(ACT8931_STATE_CSTATE, state)) {
	case ACT8931_CSTATE_PRE:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case ACT8931_CSTATE_FAST:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}
}

static int act8931_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct act8931 *chip = power_supply_get_drvdata(psy);
	unsigned int status, state;
	int ret;

	ret = act8931_charger_read(chip, &status, &state);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !!(status & ACT8931_STATUS_INDAT);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = act8931_charger_status(status, state);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = act8931_charger_type(state);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const enum power_supply_property act8931_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
};

static const struct power_supply_desc act8931_charger_desc = {
	.name		= "act8931-charger",
	.type		= POWER_SUPPLY_TYPE_USB,
	.properties	= act8931_charger_props,
	.num_properties	= ARRAY_SIZE(act8931_charger_props),
	.get_property	= act8931_charger_get_property,
};

/* Charger state bits a change of which is worth a uevent */
#define ACT8931_STATUS_WATCH	(ACT8931_STATUS_CHGDAT | ACT8931_STATUS_INDAT | \
				 ACT8931_STATUS_TEMPDAT | ACT8931_STATUS_TIMRDAT)

/*
 * No interrupt: the PMU's charger interrupts stay disabled (APCH 0x79 is
 * never written), so the state is polled for the uevent.
 */
static void act8931_charger_poll(struct work_struct *work)
{
	struct act8931 *chip = container_of(work, struct act8931, poll.work);
	unsigned int status, state;

	if (!act8931_charger_read(chip, &status, &state)) {
		status &= ACT8931_STATUS_WATCH;
		state &= ACT8931_STATE_CSTATE | ACT8931_STATE_ACINSTAT;
		if (status != chip->last_status || state != chip->last_state) {
			chip->last_status = status;
			chip->last_state = state;
			power_supply_changed(chip->charger);
		}
	}
	queue_delayed_work(system_freezable_power_efficient_wq, &chip->poll,
			   msecs_to_jiffies(ACT8931_POLL_MS));
}

static int act8931_rails_show(struct seq_file *s, void *unused)
{
	struct act8931 *chip = s->private;
	unsigned int vset1, vset, ctrl, sel, status, state;
	int i;

	seq_printf(s, "VSEL %s\n", chip->vsel_high ? "high" : "low");
	seq_puts(s, "rail       vset1 vset  sel   uV       ctrl ON OK enabled\n");
	for (i = 0; i < ACT8931_NUM_RAILS; i++) {
		const struct act8931_rail *rail = &act8931_rails[i];

		if (act8931_read_rail(chip, rail, &vset1, &vset, &ctrl)) {
			seq_printf(s, "%-10s read error\n", rail->of_name);
			continue;
		}
		sel = act8931_live_sel(chip, rail, vset1, vset);
		if (rail->vset1)
			seq_printf(s, "%-10s 0x%02x  ", rail->of_name, vset1);
		else
			seq_printf(s, "%-10s -     ", rail->of_name);
		seq_printf(s, "0x%02x  0x%02x  %-8u 0x%02x %d  %d  %d\n", vset, sel,
			   act8931_sel_to_uv(sel), ctrl,
			   !!(ctrl & ACT8931_CTRL_ON), !!(ctrl & ACT8931_CTRL_OK),
			   act8931_ctrl_enabled(ctrl));
	}
	if (!act8931_charger_read(chip, &status, &state))
		seq_printf(s, "charger: status 0x%02x (INDAT %d CHGDAT %d TEMPDAT %d TIMRDAT %d), state 0x%02x (CSTATE %lu ACINSTAT %d)\n",
			   status, !!(status & ACT8931_STATUS_INDAT),
			   !!(status & ACT8931_STATUS_CHGDAT),
			   !!(status & ACT8931_STATUS_TEMPDAT),
			   !!(status & ACT8931_STATUS_TIMRDAT), state,
			   FIELD_GET(ACT8931_STATE_CSTATE, state),
			   !!(state & ACT8931_STATE_ACINSTAT));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(act8931_rails);

/*
 * debugfs act8931-reg, for bring-up: write "REG" to read a register or
 * "REG VAL" (hex) to write one through the write rules; reading the file
 * returns the result of the last command. A write is logged.
 */
static ssize_t act8931_reg_dbg_write(struct file *file,
				     const char __user *ubuf, size_t len,
				     loff_t *ppos)
{
	struct act8931 *chip = file->private_data;
	unsigned int reg, val;
	char buf[32];
	int n, ret;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';

	n = sscanf(buf, "%x %x", &reg, &val);
	if (n < 1)
		return -EINVAL;

	mutex_lock(&chip->dbg_lock);
	if (n == 1) {
		ret = regmap_read(chip->regmap, reg, &val);
		if (ret)
			snprintf(chip->dbg_result, sizeof(chip->dbg_result),
				 "0x%02x: read error %d\n", reg, ret);
		else
			snprintf(chip->dbg_result, sizeof(chip->dbg_result),
				 "0x%02x = 0x%02x\n", reg, val);
	} else {
		ret = regmap_write(chip->regmap, reg, val);
		snprintf(chip->dbg_result, sizeof(chip->dbg_result),
			 "0x%02x <- 0x%02x: %s %d\n", reg, val,
			 ret ? "error" : "ok", ret);
		dev_info(chip->dev, "debugfs write 0x%02x <- 0x%02x: %d\n",
			 reg, val, ret);
	}
	mutex_unlock(&chip->dbg_lock);

	return ret ? ret : len;
}

static ssize_t act8931_reg_dbg_read(struct file *file, char __user *ubuf,
				    size_t len, loff_t *ppos)
{
	struct act8931 *chip = file->private_data;
	ssize_t ret;

	mutex_lock(&chip->dbg_lock);
	ret = simple_read_from_buffer(ubuf, len, ppos, chip->dbg_result,
				      strlen(chip->dbg_result));
	mutex_unlock(&chip->dbg_lock);
	return ret;
}

static const struct file_operations act8931_reg_dbg_fops = {
	.owner	= THIS_MODULE,
	.open	= simple_open,
	.read	= act8931_reg_dbg_read,
	.write	= act8931_reg_dbg_write,
	.llseek	= default_llseek,
};

static void act8931_debugfs_remove(void *dentry)
{
	debugfs_remove(dentry);
}

static int act8931_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = { };
	struct device_node *regs;
	struct act8931 *chip;
	unsigned int sys0, sys1, cfg, intr, status, state;
	int i, ret, sel, board;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BYTE_DATA))
		return -EOPNOTSUPP;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->dev = dev;
	chip->client = client;
	chip->vsel_high = of_property_read_bool(dev->of_node,
						"active-semi,vsel-high");
	i2c_set_clientdata(client, chip);

	chip->regmap = devm_regmap_init(dev, NULL, chip,
					&act8931_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "regmap init failed\n");

	ret = regmap_read(chip->regmap, ACT8931_SYS0, &sys0);
	if (!ret)
		ret = regmap_read(chip->regmap, ACT8931_SYS1, &sys1);
	if (!ret)
		ret = regmap_read(chip->regmap, ACT8931_APCH_CFG, &cfg);
	if (!ret)
		ret = regmap_read(chip->regmap, ACT8931_APCH_INTR, &intr);
	if (ret)
		return dev_err_probe(dev, ret, "no answer at 0x%02x\n",
				     client->addr);
	dev_info(dev, "SYS 0x%02x 0x%02x, APCH_CFG 0x%02x, APCH_INTR 0x%02x\n",
		 sys0, sys1, cfg, intr);

	regs = of_get_child_by_name(dev->of_node, "regulators");
	for (i = 0; i < ACT8931_NUM_RAILS; i++) {
		sel = act8931_report_rail(chip, regs, &act8931_rails[i], &board);
		ret = act8931_register_rail(chip, i, regs, sel, board);
		if (ret)
			dev_warn(dev, "%s: not registered: %d\n",
				 act8931_rails[i].of_name, ret);
	}
	of_node_put(regs);

	ret = act8931_charger_read(chip, &status, &state);
	if (ret)
		return dev_err_probe(dev, ret, "charger status read failed\n");
	dev_info(dev, "charger: APCH status 0x%02x, state 0x%02x: input %s, CSTATE %lu\n",
		 status, state,
		 status & ACT8931_STATUS_INDAT ? "valid" : "absent",
		 FIELD_GET(ACT8931_STATE_CSTATE, state));
	chip->last_status = status & ACT8931_STATUS_WATCH;
	chip->last_state = state & (ACT8931_STATE_CSTATE |
				    ACT8931_STATE_ACINSTAT);

	/* no fwnode: the charger stays out of the board's supply graph */
	psy_cfg.drv_data = chip;
	chip->charger = devm_power_supply_register(dev, &act8931_charger_desc,
						   &psy_cfg);
	if (IS_ERR(chip->charger))
		return dev_err_probe(dev, PTR_ERR(chip->charger),
				     "charger registration failed\n");

	ret = devm_delayed_work_autocancel(dev, &chip->poll,
					   act8931_charger_poll);
	if (ret)
		return ret;
	queue_delayed_work(system_freezable_power_efficient_wq, &chip->poll,
			   msecs_to_jiffies(ACT8931_POLL_MS));

	ret = devm_add_action_or_reset(dev, act8931_debugfs_remove,
				       debugfs_create_file("act8931", 0444, NULL, chip,
							   &act8931_rails_fops));
	if (ret)
		return ret;

	mutex_init(&chip->dbg_lock);
	return devm_add_action_or_reset(dev, act8931_debugfs_remove,
					debugfs_create_file("act8931-reg", 0600, NULL, chip,
							    &act8931_reg_dbg_fops));
}

static const struct of_device_id act8931_of_match[] = {
	{ .compatible = "active-semi,act8931" },
	{ }
};
MODULE_DEVICE_TABLE(of, act8931_of_match);

static const struct i2c_device_id act8931_ids[] = {
	{ "act8931" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, act8931_ids);

static struct i2c_driver act8931_driver = {
	.driver = {
		.name		= "act8931",
		.of_match_table	= act8931_of_match,
	},
	.probe		= act8931_probe,
	.id_table	= act8931_ids,
};
module_i2c_driver(act8931_driver);

MODULE_DESCRIPTION("Active-semi ACT8931 regulators and charger state");
MODULE_AUTHOR("Sergei Khomenkov <shomenkow@gmail.com>");
MODULE_LICENSE("GPL");
