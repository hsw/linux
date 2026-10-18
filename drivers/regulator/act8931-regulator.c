// SPDX-License-Identifier: GPL-2.0-only
/*
 * Active-semi ACT8931 PMU: regulators and charger state.
 *
 * Every register write passes a check in the regmap write callback, the
 * one path from the driver to the bus, whatever asked for it:
 *  - only the output registers (VSET1, VSET2, VSET, CTRL) are writable;
 *    the system, charger, undocumented and reserved registers, and the
 *    suspend set-points of the ACT8865 family, are not;
 *  - bits the datasheet calls reserved must keep the value read back from
 *    the PMU just before the write;
 *  - no VSET write goes above the output's regulator-max-microvolt: not
 *    to VSET1 or VSET2 of a DC/DC, whichever VSEL picks, nor to the VSET
 *    of an LDO. The VSET registers of an output without a maximum are not
 *    written. A value the boot firmware left in the DC/DC register that
 *    VSEL does not pick is not checked.
 *
 * The ON bit of an output is edge-triggered: the output follows the last
 * 0->1 or 1->0 edge written to it, so ON is the last edge, not a
 * measurement. OK (power good) is set while the output is in regulation.
 *
 * The charger is read only: APCH 0x78 for the input and end-of-charge
 * status, 0x7a for the charge state. Bits 7:4 of 0x78 arm the charger
 * interrupts together with 0x79 and read back their status, which a read
 * clears. The driver enables no interrupt (0x79 is not written) and polls
 * the state for the uevent.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/linear_range.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include "act8931-rc2-guard.h"	/* LOCAL: RC2 stock limits */

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
#define ACT8931_VSET_RESERVED	GENMASK(7, 6)
#define ACT8931_CTRL_ON		BIT(7)
#define ACT8931_DCDC3_CTRL_RESERVED BIT(6)
#define ACT8931_CTRL_DELAY	GENMASK(4, 2)	/* turn-on delay */
#define ACT8931_CTRL_OK		BIT(0)	/* power good */
#define ACT8931_N_VOLTAGES	64

#define ACT8931_STATUS_CHGDAT	BIT(0)	/* end of charge */
#define ACT8931_STATUS_INDAT	BIT(1)	/* valid input at CHGIN */
#define ACT8931_STATE_CSTATE	GENMASK(5, 4)
/* the 0x78 bits the properties are made of: a change raises a uevent */
#define ACT8931_STATUS_WATCH	(ACT8931_STATUS_CHGDAT | ACT8931_STATUS_INDAT)

#define ACT8931_CSTATE_OFF	0	/* suspended, disabled or fault */
#define ACT8931_CSTATE_EOC	1
#define ACT8931_CSTATE_FAST	2	/* fast charge or top-off */
#define ACT8931_CSTATE_PRE	3

#define ACT8931_POLL_MS		5000

/*
 * Soft-start: 400 us for the DC/DC converters, 140 us for an LDO at
 * 2.9 V, both typical figures (datasheet); no figure is given for a VSET
 * step. The longer one is used for an enable and a voltage raise.
 */
#define ACT8931_SOFTSTART_US	400

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
	const char *supply;	/* the input pin, as the binding names it */
	unsigned int vset1;	/* DCDC only: the VSEL-low register */
	unsigned int vset;	/* LDO: the one VSET; DCDC: VSET2 */
	unsigned int ctrl;
	bool switchable;	/* may get enable/disable ops */
	bool settable;		/* may get set_voltage ops */
};

static const struct act8931_rail act8931_rails[ACT8931_NUM_RAILS] = {
	[ACT8931_ID_DCDC1] = {
		.name = "act8931-dcdc1", .of_name = "DCDC_REG1", .supply = "vp1",
		.vset1 = ACT8931_DCDC1_VSET1, .vset = ACT8931_DCDC1_VSET2,
		.ctrl = ACT8931_DCDC1_CTRL,
	},
	[ACT8931_ID_DCDC2] = {
		.name = "act8931-dcdc2", .of_name = "DCDC_REG2", .supply = "vp2",
		.vset1 = ACT8931_DCDC2_VSET1, .vset = ACT8931_DCDC2_VSET2,
		.ctrl = ACT8931_DCDC2_CTRL,
	},
	[ACT8931_ID_DCDC3] = {
		.name = "act8931-dcdc3", .of_name = "DCDC_REG3", .supply = "vp3",
		.vset1 = ACT8931_DCDC3_VSET1, .vset = ACT8931_DCDC3_VSET2,
		.ctrl = ACT8931_DCDC3_CTRL, .settable = true,
	},
	[ACT8931_ID_LDO1] = {
		.name = "act8931-ldo1", .of_name = "LDO_REG1", .supply = "inl",
		.vset = ACT8931_LDO1_VSET, .ctrl = ACT8931_LDO1_CTRL,
	},
	[ACT8931_ID_LDO2] = {
		.name = "act8931-ldo2", .of_name = "LDO_REG2", .supply = "inl",
		.vset = ACT8931_LDO2_VSET, .ctrl = ACT8931_LDO2_CTRL,
	},
	[ACT8931_ID_LDO3] = {
		.name = "act8931-ldo3", .of_name = "LDO_REG3", .supply = "inl",
		.vset = ACT8931_LDO3_VSET, .ctrl = ACT8931_LDO3_CTRL,
	},
	[ACT8931_ID_LDO4] = {
		.name = "act8931-ldo4", .of_name = "LDO_REG4", .supply = "inl",
		.vset = ACT8931_LDO4_VSET, .ctrl = ACT8931_LDO4_CTRL,
		.switchable = true,
	},
};

struct act8931 {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	bool vsel_high;
	/* highest selector a VSET write may set, for the outputs in capped */
	unsigned int cap[ACT8931_NUM_RAILS];
	DECLARE_BITMAP(capped, ACT8931_NUM_RAILS);
	struct of_regulator_match match[ACT8931_NUM_RAILS];
	struct regulator_desc desc[ACT8931_NUM_RAILS];
	struct power_supply *charger;
	struct delayed_work poll;
	unsigned int last_status;
	unsigned int last_state;
	/* LOCAL (RC2 stack): debugfs act8931-reg */
	struct mutex dbg_lock;
	char dbg_result[40];
};

/* ACT8931 output voltage table: three linear ranges of the 6-bit selector */
static const struct linear_range act8931_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(600000, 0, 23, 25000),
	REGULATOR_LINEAR_RANGE(1200000, 24, 47, 50000),
	REGULATOR_LINEAR_RANGE(2400000, 48, 63, 100000),
};

static bool act8931_ctrl_enabled(unsigned int ctrl)
{
	return (ctrl & ACT8931_CTRL_ON) && (ctrl & ACT8931_CTRL_OK);
}

/* The turn-on delay set in CTRL (0, 2, 4 ... 128 ms) plus the soft-start */
static unsigned int act8931_turn_on_us(unsigned int ctrl)
{
	unsigned int delay = FIELD_GET(ACT8931_CTRL_DELAY, ctrl);

	return ACT8931_SOFTSTART_US + (delay ? 1000U << delay : 0);
}

/*
 * An output whose CTRL reads ON without power good may still be ramping
 * up: wait its turn-on time and read CTRL again.
 */
static int act8931_settle(struct regmap *map, unsigned int reg,
			  unsigned int *ctrl)
{
	if (!(*ctrl & ACT8931_CTRL_ON) || (*ctrl & ACT8931_CTRL_OK))
		return 0;
	fsleep(act8931_turn_on_us(*ctrl));
	return regmap_read(map, reg, ctrl);
}

/*
 * ON with OK is on, ON without OK is out of regulation, and ON reads 0
 * after a falling edge.
 */
static int act8931_get_status(struct regulator_dev *rdev)
{
	unsigned int ctrl;
	int ret;

	ret = regmap_read(rdev_get_regmap(rdev), rdev->desc->enable_reg, &ctrl);
	if (ret)
		return ret;
	if (!(ctrl & ACT8931_CTRL_ON))
		return REGULATOR_STATUS_OFF;
	if (!(ctrl & ACT8931_CTRL_OK))
		return REGULATOR_STATUS_ERROR;
	return REGULATOR_STATUS_ON;
}

static int act8931_enable_time(struct regulator_dev *rdev)
{
	unsigned int ctrl;
	int ret;

	ret = regmap_read(rdev_get_regmap(rdev), rdev->desc->enable_reg, &ctrl);
	if (ret)
		return ret;
	return act8931_turn_on_us(ctrl);
}

/*
 * Enable: a rising edge of ON, written as ON 0 then ON 1 from the value
 * read back, as the datasheet gives the sequence. Nothing is written when
 * the output is on already: the regulator core enables an always-on or
 * boot-on output at registration without asking first, and a 0 written
 * to a live ON bit would switch it off.
 *
 * An output whose ON reads 1 without power good is never toggled: it may
 * still be ramping up, so it gets its turn-on time; without power good
 * after that it is out of regulation, and the falling edge of a toggle
 * would cut it while its load may still run from it. That is an error,
 * and the output is left as it is.
 */
static int act8931_enable(struct regulator_dev *rdev)
{
	struct act8931 *chip = rdev_get_drvdata(rdev);
	struct regmap *map = rdev_get_regmap(rdev);
	unsigned int reg = rdev->desc->enable_reg;
	unsigned int ctrl;
	int ret;

	ret = regmap_read(map, reg, &ctrl);
	if (!ret)
		ret = act8931_settle(map, reg, &ctrl);
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

	ret = regmap_write(map, reg, ctrl & ~ACT8931_CTRL_ON);
	if (ret)
		return ret;
	return regmap_write(map, reg, ctrl | ACT8931_CTRL_ON);
}

/*
 * Disable: a falling edge, ON 1 then ON 0. Nothing is written when ON
 * reads 0: the last edge was falling and the output is off, or ON is at
 * its default of 0. An output that the power-up sequence keeps on with
 * ON at 0 would need the full edge, but nothing here tells it from one
 * that is off (an LDO that is off may read OK 1), so it is left alone.
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

/*
 * An output without enable/disable ops counts as enabled by ON alone, so
 * that a short drop of OK under load does not make the regulator core
 * see it as off; get_status reports OK.
 */
static const struct regulator_ops act8931_ro_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
	.get_status		= act8931_get_status,
};

/* The write callback refuses a selector above the output's maximum */
static const struct regulator_ops act8931_set_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= act8931_set_voltage_time_sel,
	.is_enabled		= regulator_is_enabled_regmap,
	.get_status		= act8931_get_status,
};

/*
 * A switched output counts as enabled with ON and OK: enable has to see
 * an output that is not in regulation, and waits out a short drop of OK
 * itself before it reports an error.
 */
static const struct regulator_ops act8931_switch_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= act8931_enable,
	.disable		= act8931_disable,
	.enable_time		= act8931_enable_time,
	.get_status		= act8931_get_status,
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
 * The output an output register belongs to, or -1 for any other
 * register; *vset is true for a VSET register.
 */
static int act8931_reg_rail(unsigned int reg, bool *vset)
{
	int i;

	for (i = 0; i < ACT8931_NUM_RAILS; i++) {
		const struct act8931_rail *rail = &act8931_rails[i];

		*vset = reg == rail->vset || (rail->vset1 && reg == rail->vset1);
		if (*vset || reg == rail->ctrl)
			return i;
	}
	return -1;
}

static bool act8931_writeable_reg(struct device *dev, unsigned int reg)
{
	bool vset;

	return act8931_reg_rail(reg, &vset) >= 0;
}

/*
 * Reserved bits of an output register, which a write must keep. OK is a
 * status bit; its written value is not checked.
 */
static unsigned int act8931_reserved(unsigned int reg, bool vset)
{
	if (vset)
		return ACT8931_VSET_RESERVED;
	return reg == ACT8931_DCDC3_CTRL ? ACT8931_DCDC3_CTRL_RESERVED : 0;
}

/*
 * APCH 0x78 bits 7:4 read back the charger interrupt status, which a
 * read clears
 */
static bool act8931_precious_reg(struct device *dev, unsigned int reg)
{
	return reg == ACT8931_APCH_STATUS;
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
	const char *why = NULL;
	unsigned int keep = 0, guard_keep, cur;
	bool vset;
	int id, ret;

	/* regmap refuses a register that is not writable before this */
	id = act8931_reg_rail(reg, &vset);
	if (id < 0)
		why = "not a writable register";
	else if (val > 0xff)
		why = "not a byte";
	else if (vset && (!test_bit(id, chip->capped) ||
			  (val & ACT8931_VSEL_MASK) > chip->cap[id]))
		why = "above the output's maximum";
	else
		keep = act8931_reserved(reg, vset);

	/* LOCAL (RC2 stack): the compiled-in stock limits on top */
	if (!why) {
		why = act8931_rc2_guard(reg, val, &guard_keep);
		keep |= guard_keep;
	}

	if (keep) {
		ret = act8931_reg_read(chip, reg, &cur);
		if (ret)
			return ret;
		if ((cur ^ val) & keep)
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
	.reg_read	= act8931_reg_read,
	.reg_write	= act8931_reg_write,
};

/*
 * The output's regulator-max-microvolt as the highest selector at or
 * below it in *sel; false if the node or the property is missing, the
 * node has a regulator-microvolt-offset (the output is then registered
 * without it) or the value is below the lowest step. Taken before the
 * regmap exists, as the bound of every VSET write.
 */
static bool act8931_max_sel(struct act8931 *chip, int id, unsigned int *sel)
{
	const struct regulator_init_data *init = chip->match[id].init_data;
	bool found;

	if (!init || init->constraints.max_uV <= 0 ||
	    init->constraints.uV_offset)
		return false;
	if (linear_range_get_selector_low_array(act8931_voltage_ranges,
						ARRAY_SIZE(act8931_voltage_ranges),
						init->constraints.max_uV,
						sel, &found)) {
		dev_warn(chip->dev, "%s: maximum %d uV is below the lowest step\n",
			 act8931_rails[id].of_name, init->constraints.max_uV);
		return false;
	}
	return true;
}

/*
 * Registers one output. It is bound to its device tree node unless the
 * regulator core would write it or refuse the node at registration: a
 * node with only one end of the voltage range, a voltage outside it, or
 * a voltage offset, which the core takes off the voltage it reads.
 * Ops are given so that binding the driver moves no output, except that
 * the core switches on an always-on or boot-on LDO4 that is off:
 *  - DCDC3 gets set_voltage ops when its node states a range; the core
 *    keeps the voltage inside it;
 *  - LDO4 gets enable/disable ops when its node states one voltage
 *    (minimum = maximum), which it then runs at; whether it stays on is
 *    the board's choice (regulator-always-on);
 *  - the other outputs are only read.
 */
static int act8931_register_rail(struct act8931 *chip, int id)
{
	const struct act8931_rail *rail = &act8931_rails[id];
	const struct regulator_init_data *init = chip->match[id].init_data;
	const struct regulation_constraints *c = init ? &init->constraints : NULL;
	struct regulator_desc *desc = &chip->desc[id];
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	unsigned int sel, ctrl, uv;
	bool range;
	int ret;

	desc->name = rail->name;
	desc->supply_name = rail->supply;
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
	desc->enable_mask = ACT8931_CTRL_ON;
	desc->enable_val = ACT8931_CTRL_ON;

	ret = regmap_read(chip->regmap, desc->vsel_reg, &sel);
	if (!ret)
		ret = regmap_read(chip->regmap, rail->ctrl, &ctrl);
	if (ret)
		return dev_err_probe(chip->dev, ret, "%s: read failed\n",
				     rail->of_name);
	sel &= ACT8931_VSEL_MASK;
	linear_range_get_value_array(act8931_voltage_ranges,
				     ARRAY_SIZE(act8931_voltage_ranges), sel, &uv);
	dev_dbg(chip->dev, "%s: selector 0x%02x = %u uV, CTRL 0x%02x\n",
		rail->of_name, sel, uv, ctrl);

	if (c && c->uV_offset) {
		dev_warn(chip->dev, "%s: regulator-microvolt-offset is not supported: registered without the node\n",
			 rail->of_name);
		c = NULL;
	}
	range = c && c->min_uV > 0 && c->max_uV >= c->min_uV;
	if (c && (c->min_uV || c->max_uV) && !range) {
		dev_warn(chip->dev, "%s: no usable voltage range in the node: registered without it\n",
			 rail->of_name);
		c = NULL;
	} else if (range && ((int)uv < c->min_uV || (int)uv > c->max_uV)) {
		dev_warn(chip->dev, "%s: selector 0x%02x (%u uV) is outside the board's %d-%d uV; left as it is, registered without the node\n",
			 rail->of_name, sel, uv, c->min_uV, c->max_uV);
		c = NULL;
	}

	if (c) {
		if (rail->settable && range) {
			desc->ops = &act8931_set_ops;
		} else if (rail->switchable && range &&
			   c->min_uV == c->max_uV) {
			desc->ops = &act8931_switch_ops;
			desc->enable_mask = ACT8931_CTRL_ON | ACT8931_CTRL_OK;
			desc->enable_val = desc->enable_mask;
		} else if (rail->settable || rail->switchable) {
			dev_info(chip->dev, "%s: the node states no %s: read only\n",
				 rail->of_name, rail->settable ?
				 "voltage range" : "single voltage");
		}
		config.init_data = init;
		config.of_node = chip->match[id].of_node;

		/*
		 * The core enables an always-on or boot-on output at
		 * registration, and a failed enable fails the registration.
		 * An LDO4 that stays ON without power good is left alone
		 * here and not registered.
		 */
		if (desc->ops == &act8931_switch_ops &&
		    (c->always_on || c->boot_on)) {
			ret = act8931_settle(chip->regmap, rail->ctrl, &ctrl);
			if (ret)
				return dev_err_probe(chip->dev, ret,
						     "%s: read failed\n",
						     rail->of_name);
			if ((ctrl & ACT8931_CTRL_ON) &&
			    !(ctrl & ACT8931_CTRL_OK)) {
				dev_warn(chip->dev,
					 "%s: ON without power good (CTRL 0x%02x): not registered\n",
					 rail->of_name, ctrl);
				return 0;
			}
		}
	}
	config.dev = chip->dev;
	config.regmap = chip->regmap;
	config.driver_data = chip;

	rdev = devm_regulator_register(chip->dev, desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(chip->dev, PTR_ERR(rdev),
				     "%s: not registered\n", rail->of_name);
	return 0;
}

/* The one reader of APCH 0x78, whose bits 7:4 clear on read */
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
	case ACT8931_CSTATE_OFF:
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

static void act8931_charger_poll(struct work_struct *work)
{
	struct act8931 *chip = container_of(work, struct act8931, poll.work);
	unsigned int status, state;

	if (!act8931_charger_read(chip, &status, &state)) {
		status &= ACT8931_STATUS_WATCH;
		state &= ACT8931_STATE_CSTATE;
		if (status != chip->last_status || state != chip->last_state) {
			chip->last_status = status;
			chip->last_state = state;
			power_supply_changed(chip->charger);
		}
	}
	queue_delayed_work(system_freezable_power_efficient_wq, &chip->poll,
			   msecs_to_jiffies(ACT8931_POLL_MS));
}

/*
 * The charger comes first: a failure there leaves no output registered
 * that a consumer could already hold.
 */
static int act8931_charger_init(struct act8931 *chip)
{
	struct power_supply_config psy_cfg = { };
	unsigned int status, state;
	int ret;

	ret = act8931_charger_read(chip, &status, &state);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "charger state read failed\n");
	chip->last_status = status & ACT8931_STATUS_WATCH;
	chip->last_state = state & ACT8931_STATE_CSTATE;

	/* no fwnode: the charger stays out of the board's supply graph */
	psy_cfg.drv_data = chip;
	chip->charger = devm_power_supply_register(chip->dev,
						   &act8931_charger_desc,
						   &psy_cfg);
	if (IS_ERR(chip->charger))
		return dev_err_probe(chip->dev, PTR_ERR(chip->charger),
				     "charger registration failed\n");

	ret = devm_delayed_work_autocancel(chip->dev, &chip->poll,
					   act8931_charger_poll);
	if (ret)
		return ret;
	queue_delayed_work(system_freezable_power_efficient_wq, &chip->poll,
			   msecs_to_jiffies(ACT8931_POLL_MS));
	return 0;
}

/*
 * LOCAL (RC2 stack), bring-up debugfs files, the interface of the RC2
 * book tools (rc2-dvfs-sweep, rc2-cpufreq-test.sh, the checklists):
 *  - act8931 (0444): every output's VSET registers, live selector and
 *    voltage, CTRL with ON, OK and enabled (ON && OK), then the charger
 *    status;
 *  - act8931-reg (0600): write "REG" (hex) to read a register, "REG VAL"
 *    to write one through the regmap, so through every write rule of the
 *    driver and the RC2 stock limits; reading the file returns the result
 *    of the last command. A write is logged.
 */
static int act8931_rails_show(struct seq_file *s, void *unused)
{
	struct act8931 *chip = s->private;
	unsigned int vset1, vset, ctrl, sel, uv, status, state;
	int i;

	seq_printf(s, "VSEL %s\n", chip->vsel_high ? "high" : "low");
	seq_puts(s, "rail       vset1 vset  sel   uV       ctrl ON OK enabled\n");
	for (i = 0; i < ACT8931_NUM_RAILS; i++) {
		const struct act8931_rail *rail = &act8931_rails[i];

		vset1 = 0;
		if ((rail->vset1 &&
		     regmap_read(chip->regmap, rail->vset1, &vset1)) ||
		    regmap_read(chip->regmap, rail->vset, &vset) ||
		    regmap_read(chip->regmap, rail->ctrl, &ctrl)) {
			seq_printf(s, "%-10s read error\n", rail->of_name);
			continue;
		}
		sel = (rail->vset1 && !chip->vsel_high ? vset1 : vset) &
		      ACT8931_VSEL_MASK;
		linear_range_get_value_array(act8931_voltage_ranges,
					     ARRAY_SIZE(act8931_voltage_ranges),
					     sel, &uv);
		if (rail->vset1)
			seq_printf(s, "%-10s 0x%02x  ", rail->of_name, vset1);
		else
			seq_printf(s, "%-10s -     ", rail->of_name);
		seq_printf(s, "0x%02x  0x%02x  %-8u 0x%02x %d  %d  %d\n", vset,
			   sel, uv, ctrl, !!(ctrl & ACT8931_CTRL_ON),
			   !!(ctrl & ACT8931_CTRL_OK),
			   act8931_ctrl_enabled(ctrl));
	}
	if (!act8931_charger_read(chip, &status, &state))
		seq_printf(s, "charger: status 0x%02x (INDAT %d CHGDAT %d), state 0x%02x (CSTATE %lu)\n",
			   status, !!(status & ACT8931_STATUS_INDAT),
			   !!(status & ACT8931_STATUS_CHGDAT), state,
			   FIELD_GET(ACT8931_STATE_CSTATE, state));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(act8931_rails);

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

static int act8931_debugfs_init(struct act8931 *chip)
{
	int ret;

	ret = devm_mutex_init(chip->dev, &chip->dbg_lock);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(chip->dev, act8931_debugfs_remove,
				       debugfs_create_file("act8931", 0444,
							   NULL, chip,
							   &act8931_rails_fops));
	if (ret)
		return ret;
	return devm_add_action_or_reset(chip->dev, act8931_debugfs_remove,
					debugfs_create_file("act8931-reg",
							    0600, NULL, chip,
							    &act8931_reg_dbg_fops));
}

static int act8931_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *regs;
	struct act8931 *chip;
	unsigned int val;
	int i, n, ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA))
		return -EOPNOTSUPP;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->dev = dev;
	chip->client = client;
	chip->vsel_high = of_property_read_bool(dev->of_node,
						"active-semi,vsel-high");

	/* the bounds of the VSET writes, before anything can write */
	regs = of_get_child_by_name(dev->of_node, "regulators");
	if (regs) {
		for (i = 0; i < ACT8931_NUM_RAILS; i++)
			chip->match[i].name = act8931_rails[i].of_name;
		n = of_get_child_count(regs);
		ret = of_regulator_match(dev, regs, chip->match,
					 ACT8931_NUM_RAILS);
		of_node_put(regs);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "failed to parse regulators node\n");
		if (ret < n)
			dev_warn(dev, "regulators: %d of %d child nodes name no output (DCDC_REG1-3, LDO_REG1-4)\n",
				 n - ret, n);
	}
	for (i = 0; i < ACT8931_NUM_RAILS; i++)
		if (act8931_max_sel(chip, i, &chip->cap[i]))
			__set_bit(i, chip->capped);

	chip->regmap = devm_regmap_init(dev, NULL, chip,
					&act8931_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "regmap init failed\n");

	ret = regmap_read(chip->regmap, ACT8931_SYS0, &val);
	if (ret)
		return dev_err_probe(dev, ret, "no answer at 0x%02x\n",
				     client->addr);

	ret = act8931_charger_init(chip);
	if (ret)
		return ret;

	for (i = 0; i < ACT8931_NUM_RAILS; i++) {
		ret = act8931_register_rail(chip, i);
		if (ret)
			return ret;
	}
	return act8931_debugfs_init(chip);	/* LOCAL */
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
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= act8931_probe,
	.id_table	= act8931_ids,
};
module_i2c_driver(act8931_driver);

MODULE_DESCRIPTION("Active-semi ACT8931 regulators and charger state");
MODULE_AUTHOR("Sergei Khomenkov <shomenkow@gmail.com>");
MODULE_LICENSE("GPL");
