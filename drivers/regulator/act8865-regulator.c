// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * act8865-regulator.c - Voltage regulation for active-semi ACT88xx PMUs
 *
 * http://www.active-semi.com/products/power-management-units/act88xx/
 *
 * Copyright (C) 2013 Atmel Corporation
 */

#include <linux/bitfield.h>
#include <linux/devm-helpers.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/act8865.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>
#include <dt-bindings/regulator/active-semi,8865-regulator.h>

/*
 * ACT8600 Global Register Map.
 */
#define ACT8600_SYS_MODE	0x00
#define ACT8600_SYS_CTRL	0x01
#define ACT8600_DCDC1_VSET	0x10
#define ACT8600_DCDC1_CTRL	0x12
#define ACT8600_DCDC2_VSET	0x20
#define ACT8600_DCDC2_CTRL	0x22
#define ACT8600_DCDC3_VSET	0x30
#define ACT8600_DCDC3_CTRL	0x32
#define ACT8600_SUDCDC4_VSET	0x40
#define ACT8600_SUDCDC4_CTRL	0x41
#define ACT8600_LDO5_VSET	0x50
#define ACT8600_LDO5_CTRL	0x51
#define ACT8600_LDO6_VSET	0x60
#define ACT8600_LDO6_CTRL	0x61
#define ACT8600_LDO7_VSET	0x70
#define ACT8600_LDO7_CTRL	0x71
#define ACT8600_LDO8_VSET	0x80
#define ACT8600_LDO8_CTRL	0x81
#define ACT8600_LDO910_CTRL	0x91
#define ACT8600_APCH0		0xA1
#define ACT8600_APCH1		0xA8
#define ACT8600_APCH2		0xA9
#define ACT8600_APCH_STAT	0xAA
#define ACT8600_OTG0		0xB0
#define ACT8600_OTG1		0xB2

/*
 * ACT8846 Global Register Map.
 */
#define	ACT8846_SYS0		0x00
#define	ACT8846_SYS1		0x01
#define	ACT8846_REG1_VSET	0x10
#define	ACT8846_REG1_CTRL	0x12
#define	ACT8846_REG2_VSET0	0x20
#define	ACT8846_REG2_VSET1	0x21
#define	ACT8846_REG2_CTRL	0x22
#define	ACT8846_REG3_VSET0	0x30
#define	ACT8846_REG3_VSET1	0x31
#define	ACT8846_REG3_CTRL	0x32
#define	ACT8846_REG4_VSET0	0x40
#define	ACT8846_REG4_VSET1	0x41
#define	ACT8846_REG4_CTRL	0x42
#define	ACT8846_REG5_VSET	0x50
#define	ACT8846_REG5_CTRL	0x51
#define	ACT8846_REG6_VSET	0x58
#define	ACT8846_REG6_CTRL	0x59
#define	ACT8846_REG7_VSET	0x60
#define	ACT8846_REG7_CTRL	0x61
#define	ACT8846_REG8_VSET	0x68
#define	ACT8846_REG8_CTRL	0x69
#define	ACT8846_REG9_VSET	0x70
#define	ACT8846_REG9_CTRL	0x71
#define	ACT8846_REG10_VSET	0x80
#define	ACT8846_REG10_CTRL	0x81
#define	ACT8846_REG11_VSET	0x90
#define	ACT8846_REG11_CTRL	0x91
#define	ACT8846_REG12_VSET	0xa0
#define	ACT8846_REG12_CTRL	0xa1
#define	ACT8846_REG13_CTRL	0xb1
#define	ACT8846_GLB_OFF_CTRL	0xc3
#define	ACT8846_OFF_SYSMASK	0x18

/*
 * ACT8865 Global Register Map.
 */
#define	ACT8865_SYS_MODE	0x00
#define	ACT8865_SYS_CTRL	0x01
#define	ACT8865_SYS_UNLK_REGS	0x0b
#define	ACT8865_DCDC1_VSET1	0x20
#define	ACT8865_DCDC1_VSET2	0x21
#define	ACT8865_DCDC1_CTRL	0x22
#define	ACT8865_DCDC1_SUS	0x24
#define	ACT8865_DCDC2_VSET1	0x30
#define	ACT8865_DCDC2_VSET2	0x31
#define	ACT8865_DCDC2_CTRL	0x32
#define	ACT8865_DCDC2_SUS	0x34
#define	ACT8865_DCDC3_VSET1	0x40
#define	ACT8865_DCDC3_VSET2	0x41
#define	ACT8865_DCDC3_CTRL	0x42
#define	ACT8865_DCDC3_SUS	0x44
#define	ACT8865_LDO1_VSET	0x50
#define	ACT8865_LDO1_CTRL	0x51
#define	ACT8865_LDO1_SUS	0x52
#define	ACT8865_LDO2_VSET	0x54
#define	ACT8865_LDO2_CTRL	0x55
#define	ACT8865_LDO2_SUS	0x56
#define	ACT8865_LDO3_VSET	0x60
#define	ACT8865_LDO3_CTRL	0x61
#define	ACT8865_LDO3_SUS	0x62
#define	ACT8865_LDO4_VSET	0x64
#define	ACT8865_LDO4_CTRL	0x65
#define	ACT8865_LDO4_SUS	0x66
#define	ACT8865_MSTROFF		0x20

/*
 * ACT8931 Global Register Map: the ACT8865 output registers (without
 * 0x0b and the *_SUS registers) and the charger registers.
 */
#define	ACT8931_APCH_STATUS	0x78
#define	ACT8931_APCH_STATE	0x7a

#define	ACT8931_APCH_CHGDAT	BIT(0)		/* 0x78: end of charge */
#define	ACT8931_APCH_INDAT	BIT(1)		/* 0x78: valid input */
#define	ACT8931_CSTATE_MASK	GENMASK(5, 4)	/* 0x7a: charge state */
#define	ACT8931_CSTATE_EOC	1	/* end of charge */
#define	ACT8931_CSTATE_FAST	2	/* fast charge or top-off */
#define	ACT8931_CSTATE_PRE	3	/* precondition */

#define	ACT8931_CHARGER_POLL_MS	5000

/*
 * Field Definitions.
 */
#define	ACT8865_ENA		0x80	/* ON - [7] */
#define	ACT8865_DIS		0x40	/* DIS - [6] */

#define	ACT8865_VSEL_MASK	0x3F	/* VSET - [5:0] */

#define	ACT8931_CTRL_DELAY	GENMASK(4, 2)	/* turn-on delay */
#define	ACT8931_CTRL_OK		BIT(0)		/* power good */

/*
 * ACT8931 soft-start, typical: 400 us for a DC/DC converter, 140 us for
 * an LDO at 2.9 V. The longer one is used for an enable and a voltage
 * raise; no slew rate is given for a VSET change.
 */
#define	ACT8931_SOFTSTART_US	400


#define ACT8600_LDO10_ENA		0x40	/* ON - [6] */
#define ACT8600_SUDCDC_VSEL_MASK	0xFF	/* SUDCDC VSET - [7:0] */

#define ACT8600_APCH_CHG_ACIN		BIT(7)
#define ACT8600_APCH_CHG_USB		BIT(6)
#define ACT8600_APCH_CSTATE0		BIT(5)
#define ACT8600_APCH_CSTATE1		BIT(4)

/*
 * ACT8865 voltage number
 */
#define	ACT8865_VOLTAGE_NUM	64
#define ACT8600_SUDCDC_VOLTAGE_NUM	256

struct act8865 {
	struct regmap *regmap;
	int off_reg;
	int off_mask;
};

static const struct regmap_range act8600_reg_ranges[] = {
	regmap_reg_range(0x00, 0x01),
	regmap_reg_range(0x10, 0x10),
	regmap_reg_range(0x12, 0x12),
	regmap_reg_range(0x20, 0x20),
	regmap_reg_range(0x22, 0x22),
	regmap_reg_range(0x30, 0x30),
	regmap_reg_range(0x32, 0x32),
	regmap_reg_range(0x40, 0x41),
	regmap_reg_range(0x50, 0x51),
	regmap_reg_range(0x60, 0x61),
	regmap_reg_range(0x70, 0x71),
	regmap_reg_range(0x80, 0x81),
	regmap_reg_range(0x91, 0x91),
	regmap_reg_range(0xA1, 0xA1),
	regmap_reg_range(0xA8, 0xAA),
	regmap_reg_range(0xB0, 0xB0),
	regmap_reg_range(0xB2, 0xB2),
	regmap_reg_range(0xC1, 0xC1),
};

static const struct regmap_range act8600_reg_ro_ranges[] = {
	regmap_reg_range(0xAA, 0xAA),
	regmap_reg_range(0xC1, 0xC1),
};

static const struct regmap_range act8600_reg_volatile_ranges[] = {
	regmap_reg_range(0x00, 0x01),
	regmap_reg_range(0x12, 0x12),
	regmap_reg_range(0x22, 0x22),
	regmap_reg_range(0x32, 0x32),
	regmap_reg_range(0x41, 0x41),
	regmap_reg_range(0x51, 0x51),
	regmap_reg_range(0x61, 0x61),
	regmap_reg_range(0x71, 0x71),
	regmap_reg_range(0x81, 0x81),
	regmap_reg_range(0xA8, 0xA8),
	regmap_reg_range(0xAA, 0xAA),
	regmap_reg_range(0xB0, 0xB0),
	regmap_reg_range(0xC1, 0xC1),
};

static const struct regmap_access_table act8600_write_ranges_table = {
	.yes_ranges	= act8600_reg_ranges,
	.n_yes_ranges	= ARRAY_SIZE(act8600_reg_ranges),
	.no_ranges	= act8600_reg_ro_ranges,
	.n_no_ranges	= ARRAY_SIZE(act8600_reg_ro_ranges),
};

static const struct regmap_access_table act8600_read_ranges_table = {
	.yes_ranges	= act8600_reg_ranges,
	.n_yes_ranges	= ARRAY_SIZE(act8600_reg_ranges),
};

static const struct regmap_access_table act8600_volatile_ranges_table = {
	.yes_ranges	= act8600_reg_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(act8600_reg_volatile_ranges),
};

static const struct regmap_config act8600_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
	.wr_table = &act8600_write_ranges_table,
	.rd_table = &act8600_read_ranges_table,
	.volatile_table = &act8600_volatile_ranges_table,
};

static const struct regmap_range act8931_reg_ranges[] = {
	regmap_reg_range(0x00, 0x01),
	regmap_reg_range(0x20, 0x22),
	regmap_reg_range(0x30, 0x32),
	regmap_reg_range(0x40, 0x42),
	regmap_reg_range(0x50, 0x51),
	regmap_reg_range(0x54, 0x55),
	regmap_reg_range(0x60, 0x61),
	regmap_reg_range(0x64, 0x65),
	regmap_reg_range(0x70, 0x71),
	regmap_reg_range(0x78, 0x7A),
};

/* Only the output registers: VSET1, VSET2, VSET and CTRL */
static const struct regmap_range act8931_reg_wr_ranges[] = {
	regmap_reg_range(0x20, 0x22),
	regmap_reg_range(0x30, 0x32),
	regmap_reg_range(0x40, 0x42),
	regmap_reg_range(0x50, 0x51),
	regmap_reg_range(0x54, 0x55),
	regmap_reg_range(0x60, 0x61),
	regmap_reg_range(0x64, 0x65),
};

/* Bits 7:4 of APCH 0x78 report the charger interrupts and clear on read */
static const struct regmap_range act8931_reg_precious_ranges[] = {
	regmap_reg_range(ACT8931_APCH_STATUS, ACT8931_APCH_STATUS),
};

static const struct regmap_access_table act8931_read_ranges_table = {
	.yes_ranges	= act8931_reg_ranges,
	.n_yes_ranges	= ARRAY_SIZE(act8931_reg_ranges),
};

static const struct regmap_access_table act8931_write_ranges_table = {
	.yes_ranges	= act8931_reg_wr_ranges,
	.n_yes_ranges	= ARRAY_SIZE(act8931_reg_wr_ranges),
};

static const struct regmap_access_table act8931_precious_ranges_table = {
	.yes_ranges	= act8931_reg_precious_ranges,
	.n_yes_ranges	= ARRAY_SIZE(act8931_reg_precious_ranges),
};

static const struct regmap_config act8931_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ACT8931_APCH_STATE,
	.wr_table = &act8931_write_ranges_table,
	.rd_table = &act8931_read_ranges_table,
	.precious_table = &act8931_precious_ranges_table,
};

static const struct regmap_config act8865_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct linear_range act8865_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(600000, 0, 23, 25000),
	REGULATOR_LINEAR_RANGE(1200000, 24, 47, 50000),
	REGULATOR_LINEAR_RANGE(2400000, 48, 63, 100000),
};

static const struct linear_range act8600_sudcdc_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(3000000, 0, 63, 0),
	REGULATOR_LINEAR_RANGE(3000000, 64, 159, 100000),
	REGULATOR_LINEAR_RANGE(12600000, 160, 191, 200000),
	REGULATOR_LINEAR_RANGE(19000000, 192, 247, 400000),
	REGULATOR_LINEAR_RANGE(41400000, 248, 255, 0),
};

static int act8865_set_suspend_state(struct regulator_dev *rdev, bool enable)
{
	struct regmap *regmap = rdev->regmap;
	int id = rdev->desc->id, reg, val;

	switch (id) {
	case ACT8865_ID_DCDC1:
		reg = ACT8865_DCDC1_SUS;
		val = 0xa8;
		break;
	case ACT8865_ID_DCDC2:
		reg = ACT8865_DCDC2_SUS;
		val = 0xa8;
		break;
	case ACT8865_ID_DCDC3:
		reg = ACT8865_DCDC3_SUS;
		val = 0xa8;
		break;
	case ACT8865_ID_LDO1:
		reg = ACT8865_LDO1_SUS;
		val = 0xe8;
		break;
	case ACT8865_ID_LDO2:
		reg = ACT8865_LDO2_SUS;
		val = 0xe8;
		break;
	case ACT8865_ID_LDO3:
		reg = ACT8865_LDO3_SUS;
		val = 0xe8;
		break;
	case ACT8865_ID_LDO4:
		reg = ACT8865_LDO4_SUS;
		val = 0xe8;
		break;
	default:
		return -EINVAL;
	}

	if (enable)
		val |= BIT(4);

	/*
	 * Ask the PMIC to enable/disable this output when entering hibernate
	 * mode.
	 */
	return regmap_write(regmap, reg, val);
}

static int act8865_set_suspend_enable(struct regulator_dev *rdev)
{
	return act8865_set_suspend_state(rdev, true);
}

static int act8865_set_suspend_disable(struct regulator_dev *rdev)
{
	return act8865_set_suspend_state(rdev, false);
}

static unsigned int act8865_of_map_mode(unsigned int mode)
{
	switch (mode) {
	case ACT8865_REGULATOR_MODE_FIXED:
		return REGULATOR_MODE_FAST;
	case ACT8865_REGULATOR_MODE_NORMAL:
		return REGULATOR_MODE_NORMAL;
	case ACT8865_REGULATOR_MODE_LOWPOWER:
		return REGULATOR_MODE_STANDBY;
	default:
		return REGULATOR_MODE_INVALID;
	}
}

static int act8865_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct regmap *regmap = rdev->regmap;
	int id = rdev_get_id(rdev);
	int reg, val = 0;

	switch (id) {
	case ACT8865_ID_DCDC1:
		reg = ACT8865_DCDC1_CTRL;
		break;
	case ACT8865_ID_DCDC2:
		reg = ACT8865_DCDC2_CTRL;
		break;
	case ACT8865_ID_DCDC3:
		reg = ACT8865_DCDC3_CTRL;
		break;
	case ACT8865_ID_LDO1:
		reg = ACT8865_LDO1_CTRL;
		break;
	case ACT8865_ID_LDO2:
		reg = ACT8865_LDO2_CTRL;
		break;
	case ACT8865_ID_LDO3:
		reg = ACT8865_LDO3_CTRL;
		break;
	case ACT8865_ID_LDO4:
		reg = ACT8865_LDO4_CTRL;
		break;
	default:
		return -EINVAL;
	}

	switch (mode) {
	case REGULATOR_MODE_FAST:
	case REGULATOR_MODE_NORMAL:
		if (id <= ACT8865_ID_DCDC3)
			val = BIT(5);
		break;
	case REGULATOR_MODE_STANDBY:
		if (id > ACT8865_ID_DCDC3)
			val = BIT(5);
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(regmap, reg, BIT(5), val);
}

static unsigned int act8865_get_mode(struct regulator_dev *rdev)
{
	struct regmap *regmap = rdev->regmap;
	int id = rdev_get_id(rdev);
	int reg, ret, val = 0;

	switch (id) {
	case ACT8865_ID_DCDC1:
		reg = ACT8865_DCDC1_CTRL;
		break;
	case ACT8865_ID_DCDC2:
		reg = ACT8865_DCDC2_CTRL;
		break;
	case ACT8865_ID_DCDC3:
		reg = ACT8865_DCDC3_CTRL;
		break;
	case ACT8865_ID_LDO1:
		reg = ACT8865_LDO1_CTRL;
		break;
	case ACT8865_ID_LDO2:
		reg = ACT8865_LDO2_CTRL;
		break;
	case ACT8865_ID_LDO3:
		reg = ACT8865_LDO3_CTRL;
		break;
	case ACT8865_ID_LDO4:
		reg = ACT8865_LDO4_CTRL;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_read(regmap, reg, &val);
	if (ret)
		return ret;

	if (id <= ACT8865_ID_DCDC3 && (val & BIT(5)))
		return REGULATOR_MODE_FAST;
	else if	(id > ACT8865_ID_DCDC3 && !(val & BIT(5)))
		return REGULATOR_MODE_NORMAL;
	else
		return REGULATOR_MODE_STANDBY;
}

static const struct regulator_ops act8865_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.set_mode		= act8865_set_mode,
	.get_mode		= act8865_get_mode,
	.is_enabled		= regulator_is_enabled_regmap,
	.set_suspend_enable	= act8865_set_suspend_enable,
	.set_suspend_disable	= act8865_set_suspend_disable,
};

static const struct regulator_ops act8865_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.set_mode		= act8865_set_mode,
	.get_mode		= act8865_get_mode,
	.is_enabled		= regulator_is_enabled_regmap,
	.set_suspend_enable	= act8865_set_suspend_enable,
	.set_suspend_disable	= act8865_set_suspend_disable,
	.set_pull_down		= regulator_set_pull_down_regmap,
};

static const struct regulator_ops act8865_fixed_ldo_ops = {
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
};

/*
 * The ACT8931 ON bit is edge-triggered: an output follows the last rising
 * or falling edge written to ON, and ON reads back that edge, not the
 * output state. OK (power good) is set while the output is in regulation;
 * an LDO that is off may read OK set. An output is on with ON and OK;
 * is_enabled of a DC/DC reads ON alone (act8931_ops).
 */
static bool act8931_ctrl_enabled(unsigned int ctrl)
{
	return (ctrl & ACT8865_ENA) && (ctrl & ACT8931_CTRL_OK);
}

/*
 * The turn-on delay set in CTRL (DELAY n > 0: 2^n ms, up to 12% longer)
 * plus the soft-start
 */
static unsigned int act8931_turn_on_us(unsigned int ctrl)
{
	unsigned int delay = FIELD_GET(ACT8931_CTRL_DELAY, ctrl);

	return ACT8931_SOFTSTART_US + (delay ? (1000U << delay) * 112 / 100 : 0);
}

/*
 * LDOs: ON without OK counts as off. Such an LDO is out of regulation, or
 * ramps inside an enable (which holds the regulator lock); while it reads
 * so, a voltage change skips the ramp wait, the max-step stepping and the
 * coupling balance.
 */
static int act8931_is_enabled(struct regulator_dev *rdev)
{
	unsigned int ctrl;
	int ret;

	ret = regmap_read(rdev->regmap, rdev->desc->enable_reg, &ctrl);
	if (ret)
		return ret;
	return act8931_ctrl_enabled(ctrl);
}

/* ON with OK is on, ON without OK is out of regulation */
static int act8931_get_status(struct regulator_dev *rdev)
{
	unsigned int ctrl;
	int ret;

	ret = regmap_read(rdev->regmap, rdev->desc->enable_reg, &ctrl);
	if (ret)
		return ret;
	if (!(ctrl & ACT8865_ENA))
		return REGULATOR_STATUS_OFF;
	if (!(ctrl & ACT8931_CTRL_OK))
		return REGULATOR_STATUS_ERROR;
	return REGULATOR_STATUS_ON;
}

/*
 * The core enables without a wait when this fails, and the enable reads
 * CTRL again: a failed read gives the longest turn-on time instead.
 */
static int act8931_enable_time(struct regulator_dev *rdev)
{
	unsigned int ctrl;

	if (regmap_read(rdev->regmap, rdev->desc->enable_reg, &ctrl))
		ctrl = ACT8931_CTRL_DELAY;
	return act8931_turn_on_us(ctrl);
}

/*
 * Enable: ON 0 then ON 1 from the value read back, the datasheet's
 * sequence; nothing is written when the output is on already. An output
 * that is ON without power good after its turn-on time is never toggled:
 * the falling edge would cut a load that may still run from it.
 */
static int act8931_enable(struct regulator_dev *rdev)
{
	struct regmap *regmap = rdev->regmap;
	unsigned int reg = rdev->desc->enable_reg;
	unsigned int ctrl;
	int ret;

	ret = regmap_read(regmap, reg, &ctrl);
	if (ret)
		return ret;
	/* ON without power good may still be ramping up: wait, read again */
	if ((ctrl & ACT8865_ENA) && !(ctrl & ACT8931_CTRL_OK)) {
		fsleep(act8931_turn_on_us(ctrl));
		ret = regmap_read(regmap, reg, &ctrl);
		if (ret)
			return ret;
	}
	if (act8931_ctrl_enabled(ctrl))
		return 0;
	if (ctrl & ACT8865_ENA) {
		dev_err(rdev_get_dev(rdev)->parent,
			"%s: ON without power good (CTRL 0x%02x): not toggled\n",
			rdev->desc->name, ctrl);
		return -EIO;
	}

	ret = regmap_write(regmap, reg, ctrl & ~ACT8865_ENA);
	if (ret)
		return ret;
	return regmap_write(regmap, reg, ctrl | ACT8865_ENA);
}

/*
 * Disable: ON 1 then ON 0. Nothing is written when ON reads 0: an output
 * kept on by the power-up sequence with ON at 0 cannot be told from one
 * that is off.
 */
static int act8931_disable(struct regulator_dev *rdev)
{
	struct regmap *regmap = rdev->regmap;
	unsigned int reg = rdev->desc->enable_reg;
	unsigned int ctrl;
	int ret;

	ret = regmap_read(regmap, reg, &ctrl);
	if (ret)
		return ret;
	if (!(ctrl & ACT8865_ENA))
		return 0;

	ret = regmap_write(regmap, reg, ctrl | ACT8865_ENA);
	if (ret)
		return ret;
	return regmap_write(regmap, reg, ctrl & ~ACT8865_ENA);
}

/*
 * The core calls this instead of its own delay: return that delay, and at
 * least the soft-start time for a raise.
 */
static int act8931_set_voltage_time_sel(struct regulator_dev *rdev,
					unsigned int old_sel,
					unsigned int new_sel)
{
	int us = regulator_set_voltage_time_sel(rdev, old_sel, new_sel);

	if (us < 0 || new_sel <= old_sel)
		return us;
	return max(us, ACT8931_SOFTSTART_US);
}

/*
 * The ACT8931 has no suspend registers. The DC/DC converters get no
 * disable op: they supply the SoC, and a falling edge on their ON bit
 * would cut the system it runs on. They are on with ON alone, OK goes
 * to get_status: the core must not count a live CPU or memory rail off
 * when OK drops for a moment, or it skips the voltage-change wait, the
 * max-step stepping and the coupling balance. A DC/DC left on by firmware
 * with ON at 0 counts off, with the same skips, until its first enable;
 * with regulator-always-on the core makes that enable at registration.
 *
 * No shutdown hook puts DCDC3, the CPU rail of the RK3026 reference
 * design, back to its boot voltage. A SoC reset leaves the chip as it
 * is: after a warm reboot DCDC3 is at the last DVFS level, and the boot
 * loader must set the voltage it needs itself.
 */
static const struct regulator_ops act8931_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= act8931_set_voltage_time_sel,
	.enable			= act8931_enable,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable_time		= act8931_enable_time,
	.get_status		= act8931_get_status,
	.set_mode		= act8865_set_mode,
	.get_mode		= act8865_get_mode,
};

static const struct regulator_ops act8931_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= act8931_set_voltage_time_sel,
	.enable			= act8931_enable,
	.disable		= act8931_disable,
	.is_enabled		= act8931_is_enabled,
	.enable_time		= act8931_enable_time,
	.get_status		= act8931_get_status,
	.set_mode		= act8865_set_mode,
	.get_mode		= act8865_get_mode,
	.set_pull_down		= regulator_set_pull_down_regmap,
};

#define ACT88xx_REG_(_name, _family, _id, _vsel_reg, _supply, _ops)	\
	[_family##_ID_##_id] = {					\
		.name			= _name,			\
		.of_match		= of_match_ptr(_name),		\
		.of_map_mode		= act8865_of_map_mode,		\
		.regulators_node	= of_match_ptr("regulators"),	\
		.supply_name		= _supply,			\
		.id			= _family##_ID_##_id,		\
		.type			= REGULATOR_VOLTAGE,		\
		.ops			= _ops,				\
		.n_voltages		= ACT8865_VOLTAGE_NUM,		\
		.linear_ranges		= act8865_voltage_ranges,	\
		.n_linear_ranges	= ARRAY_SIZE(act8865_voltage_ranges), \
		.vsel_reg		= _family##_##_id##_##_vsel_reg, \
		.vsel_mask		= ACT8865_VSEL_MASK,		\
		.enable_reg		= _family##_##_id##_CTRL,	\
		.enable_mask		= ACT8865_ENA,			\
		.pull_down_reg		= _family##_##_id##_CTRL,	\
		.pull_down_mask		= ACT8865_DIS,			\
		.owner			= THIS_MODULE,			\
	}

#define ACT88xx_REG(_name, _family, _id, _vsel_reg, _supply) \
	ACT88xx_REG_(_name, _family, _id, _vsel_reg, _supply, &act8865_ops)

#define ACT88xx_LDO(_name, _family, _id, _vsel_reg, _supply) \
	ACT88xx_REG_(_name, _family, _id, _vsel_reg, _supply, &act8865_ldo_ops)

static const struct regulator_desc act8600_regulators[] = {
	ACT88xx_REG("DCDC1", ACT8600, DCDC1, VSET, "vp1"),
	ACT88xx_REG("DCDC2", ACT8600, DCDC2, VSET, "vp2"),
	ACT88xx_REG("DCDC3", ACT8600, DCDC3, VSET, "vp3"),
	{
		.name = "SUDCDC_REG4",
		.of_match = of_match_ptr("SUDCDC_REG4"),
		.regulators_node = of_match_ptr("regulators"),
		.id = ACT8600_ID_SUDCDC4,
		.ops = &act8865_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = ACT8600_SUDCDC_VOLTAGE_NUM,
		.linear_ranges = act8600_sudcdc_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(act8600_sudcdc_voltage_ranges),
		.vsel_reg = ACT8600_SUDCDC4_VSET,
		.vsel_mask = ACT8600_SUDCDC_VSEL_MASK,
		.enable_reg = ACT8600_SUDCDC4_CTRL,
		.enable_mask = ACT8865_ENA,
		.owner = THIS_MODULE,
	},
	ACT88xx_REG("LDO5", ACT8600, LDO5, VSET, "inl"),
	ACT88xx_REG("LDO6", ACT8600, LDO6, VSET, "inl"),
	ACT88xx_REG("LDO7", ACT8600, LDO7, VSET, "inl"),
	ACT88xx_REG("LDO8", ACT8600, LDO8, VSET, "inl"),
	{
		.name = "LDO_REG9",
		.of_match = of_match_ptr("LDO_REG9"),
		.regulators_node = of_match_ptr("regulators"),
		.id = ACT8600_ID_LDO9,
		.ops = &act8865_fixed_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 1,
		.fixed_uV = 3300000,
		.enable_reg = ACT8600_LDO910_CTRL,
		.enable_mask = ACT8865_ENA,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO_REG10",
		.of_match = of_match_ptr("LDO_REG10"),
		.regulators_node = of_match_ptr("regulators"),
		.id = ACT8600_ID_LDO10,
		.ops = &act8865_fixed_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 1,
		.fixed_uV = 1200000,
		.enable_reg = ACT8600_LDO910_CTRL,
		.enable_mask = ACT8600_LDO10_ENA,
		.owner = THIS_MODULE,
	},
};

static const struct regulator_desc act8846_regulators[] = {
	ACT88xx_REG("REG1", ACT8846, REG1, VSET, "vp1"),
	ACT88xx_REG("REG2", ACT8846, REG2, VSET0, "vp2"),
	ACT88xx_REG("REG3", ACT8846, REG3, VSET0, "vp3"),
	ACT88xx_REG("REG4", ACT8846, REG4, VSET0, "vp4"),
	ACT88xx_REG("REG5", ACT8846, REG5, VSET, "inl1"),
	ACT88xx_REG("REG6", ACT8846, REG6, VSET, "inl1"),
	ACT88xx_REG("REG7", ACT8846, REG7, VSET, "inl1"),
	ACT88xx_REG("REG8", ACT8846, REG8, VSET, "inl2"),
	ACT88xx_REG("REG9", ACT8846, REG9, VSET, "inl2"),
	ACT88xx_REG("REG10", ACT8846, REG10, VSET, "inl3"),
	ACT88xx_REG("REG11", ACT8846, REG11, VSET, "inl3"),
	ACT88xx_REG("REG12", ACT8846, REG12, VSET, "inl3"),
};

static const struct regulator_desc act8865_regulators[] = {
	ACT88xx_REG("DCDC_REG1", ACT8865, DCDC1, VSET1, "vp1"),
	ACT88xx_REG("DCDC_REG2", ACT8865, DCDC2, VSET1, "vp2"),
	ACT88xx_REG("DCDC_REG3", ACT8865, DCDC3, VSET1, "vp3"),
	ACT88xx_LDO("LDO_REG1", ACT8865, LDO1, VSET, "inl45"),
	ACT88xx_LDO("LDO_REG2", ACT8865, LDO2, VSET, "inl45"),
	ACT88xx_LDO("LDO_REG3", ACT8865, LDO3, VSET, "inl67"),
	ACT88xx_LDO("LDO_REG4", ACT8865, LDO4, VSET, "inl67"),
};

static const struct regulator_desc act8865_alt_regulators[] = {
	ACT88xx_REG("DCDC_REG1", ACT8865, DCDC1, VSET2, "vp1"),
	ACT88xx_REG("DCDC_REG2", ACT8865, DCDC2, VSET2, "vp2"),
	ACT88xx_REG("DCDC_REG3", ACT8865, DCDC3, VSET2, "vp3"),
	ACT88xx_LDO("LDO_REG1", ACT8865, LDO1, VSET, "inl45"),
	ACT88xx_LDO("LDO_REG2", ACT8865, LDO2, VSET, "inl45"),
	ACT88xx_LDO("LDO_REG3", ACT8865, LDO3, VSET, "inl67"),
	ACT88xx_LDO("LDO_REG4", ACT8865, LDO4, VSET, "inl67"),
};

static const struct regulator_desc act8931_regulators[] = {
	ACT88xx_REG_("DCDC_REG1", ACT8865, DCDC1, VSET1, "vp1", &act8931_ops),
	ACT88xx_REG_("DCDC_REG2", ACT8865, DCDC2, VSET1, "vp2", &act8931_ops),
	ACT88xx_REG_("DCDC_REG3", ACT8865, DCDC3, VSET1, "vp3", &act8931_ops),
	ACT88xx_REG_("LDO_REG1", ACT8865, LDO1, VSET, "inl", &act8931_ldo_ops),
	ACT88xx_REG_("LDO_REG2", ACT8865, LDO2, VSET, "inl", &act8931_ldo_ops),
	ACT88xx_REG_("LDO_REG3", ACT8865, LDO3, VSET, "inl", &act8931_ldo_ops),
	ACT88xx_REG_("LDO_REG4", ACT8865, LDO4, VSET, "inl", &act8931_ldo_ops),
};

static const struct regulator_desc act8931_alt_regulators[] = {
	ACT88xx_REG_("DCDC_REG1", ACT8865, DCDC1, VSET2, "vp1", &act8931_ops),
	ACT88xx_REG_("DCDC_REG2", ACT8865, DCDC2, VSET2, "vp2", &act8931_ops),
	ACT88xx_REG_("DCDC_REG3", ACT8865, DCDC3, VSET2, "vp3", &act8931_ops),
	ACT88xx_REG_("LDO_REG1", ACT8865, LDO1, VSET, "inl", &act8931_ldo_ops),
	ACT88xx_REG_("LDO_REG2", ACT8865, LDO2, VSET, "inl", &act8931_ldo_ops),
	ACT88xx_REG_("LDO_REG3", ACT8865, LDO3, VSET, "inl", &act8931_ldo_ops),
	ACT88xx_REG_("LDO_REG4", ACT8865, LDO4, VSET, "inl", &act8931_ldo_ops),
};

#ifdef CONFIG_OF
static const struct of_device_id act8865_dt_ids[] = {
	{ .compatible = "active-semi,act8600", .data = (void *)ACT8600 },
	{ .compatible = "active-semi,act8846", .data = (void *)ACT8846 },
	{ .compatible = "active-semi,act8865", .data = (void *)ACT8865 },
	{ .compatible = "active-semi,act8931", .data = (void *)ACT8931 },
	{ }
};
MODULE_DEVICE_TABLE(of, act8865_dt_ids);
#endif

static struct act8865_regulator_data *act8865_get_regulator_data(
		int id, struct act8865_platform_data *pdata)
{
	int i;

	for (i = 0; i < pdata->num_regulators; i++) {
		if (pdata->regulators[i].id == id)
			return &pdata->regulators[i];
	}

	return NULL;
}

static struct i2c_client *act8865_i2c_client;
static void act8865_power_off(void)
{
	struct act8865 *act8865;

	act8865 = i2c_get_clientdata(act8865_i2c_client);
	regmap_write(act8865->regmap, act8865->off_reg, act8865->off_mask);
	while (1);
}

static int act8600_charger_get_status(struct regmap *map)
{
	unsigned int val;
	int ret;
	u8 state0, state1;

	ret = regmap_read(map, ACT8600_APCH_STAT, &val);
	if (ret < 0)
		return ret;

	state0 = val & ACT8600_APCH_CSTATE0;
	state1 = val & ACT8600_APCH_CSTATE1;

	if (state0 && !state1)
		return POWER_SUPPLY_STATUS_CHARGING;
	if (!state0 && state1)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	if (!state0 && !state1)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static int act8600_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct regmap *map = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = act8600_charger_get_status(map);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property act8600_charger_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
};

static const struct power_supply_desc act8600_charger_desc = {
	.name = "act8600-charger",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = act8600_charger_properties,
	.num_properties = ARRAY_SIZE(act8600_charger_properties),
	.get_property = act8600_charger_get_property,
};

static int act8600_charger_probe(struct device *dev, struct regmap *regmap)
{
	struct power_supply *charger;
	struct power_supply_config cfg = {
		.drv_data = regmap,
		.fwnode = dev_fwnode(dev),
	};

	charger = devm_power_supply_register(dev, &act8600_charger_desc, &cfg);

	return PTR_ERR_OR_ZERO(charger);
}

/*
 * ACT8931 charger, read only: APCH 0x78 holds the input (INDAT) and
 * end-of-charge (CHGDAT) status, 0x7a the charge state (CSTATE). Bits 7:4
 * of 0x78 arm the charger interrupts together with 0x79 and read back
 * their status, which a read clears. Neither is written, so no interrupt
 * is enabled and the state is polled for the change event. The chip does
 * not tell a USB port from an adapter at its input (ACINSTAT is the level
 * of the ACIN current-mode pin), so the supply type is fixed.
 */
struct act8931_charger {
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work poll;
	unsigned int status;	/* INDAT and CHGDAT of the last poll */
	unsigned int state;	/* CSTATE of the last poll */
};

static int act8931_charger_read(struct act8931_charger *chg,
				unsigned int *status, unsigned int *state)
{
	int ret;

	ret = regmap_read(chg->regmap, ACT8931_APCH_STATUS, status);
	if (ret)
		return ret;
	ret = regmap_read(chg->regmap, ACT8931_APCH_STATE, state);
	if (ret)
		return ret;

	*status &= ACT8931_APCH_INDAT | ACT8931_APCH_CHGDAT;
	*state &= ACT8931_CSTATE_MASK;
	return 0;
}

static int act8931_charger_status(unsigned int status, unsigned int state)
{
	switch (FIELD_GET(ACT8931_CSTATE_MASK, state)) {
	case ACT8931_CSTATE_PRE:
	case ACT8931_CSTATE_FAST:
		return POWER_SUPPLY_STATUS_CHARGING;
	case ACT8931_CSTATE_EOC:
		if (status & ACT8931_APCH_CHGDAT)
			return POWER_SUPPLY_STATUS_FULL;
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	default:	/* suspended, disabled or fault */
		if (status & ACT8931_APCH_INDAT)
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}
}

static int act8931_charger_type(unsigned int state)
{
	switch (FIELD_GET(ACT8931_CSTATE_MASK, state)) {
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
	struct act8931_charger *chg = power_supply_get_drvdata(psy);
	unsigned int status, state;
	int ret;

	ret = act8931_charger_read(chg, &status, &state);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !!(status & ACT8931_APCH_INDAT);
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

static const enum power_supply_property act8931_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
};

static const struct power_supply_desc act8931_charger_desc = {
	.name = "act8931-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = act8931_charger_properties,
	.num_properties = ARRAY_SIZE(act8931_charger_properties),
	.get_property = act8931_charger_get_property,
};

static void act8931_charger_poll(struct work_struct *work)
{
	struct act8931_charger *chg = container_of(to_delayed_work(work),
						   struct act8931_charger, poll);
	unsigned int status, state;

	if (!act8931_charger_read(chg, &status, &state) &&
	    (status != chg->status || state != chg->state)) {
		chg->status = status;
		chg->state = state;
		power_supply_changed(chg->psy);
	}

	queue_delayed_work(system_freezable_power_efficient_wq, &chg->poll,
			   msecs_to_jiffies(ACT8931_CHARGER_POLL_MS));
}

/*
 * A failure releases what the charger got so far (devres group); the
 * caller keeps the outputs.
 */
static int act8931_charger_probe(struct device *dev, struct regmap *regmap)
{
	struct power_supply_config cfg = { .fwnode = dev_fwnode(dev) };
	struct act8931_charger *chg;
	int ret;

	if (!devres_open_group(dev, act8931_charger_probe, GFP_KERNEL))
		return -ENOMEM;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg) {
		ret = -ENOMEM;
		goto err;
	}

	chg->regmap = regmap;
	ret = act8931_charger_read(chg, &chg->status, &chg->state);
	if (ret)
		goto err;

	cfg.drv_data = chg;
	chg->psy = devm_power_supply_register(dev, &act8931_charger_desc, &cfg);
	if (IS_ERR(chg->psy)) {
		ret = PTR_ERR(chg->psy);
		goto err;
	}

	/* after the power supply: the work is cancelled before it goes */
	ret = devm_delayed_work_autocancel(dev, &chg->poll,
					   act8931_charger_poll);
	if (ret)
		goto err;

	devres_close_group(dev, act8931_charger_probe);
	queue_delayed_work(system_freezable_power_efficient_wq, &chg->poll,
			   msecs_to_jiffies(ACT8931_CHARGER_POLL_MS));
	return 0;

err:
	devres_release_group(dev, act8931_charger_probe);
	return ret;
}

static int act8865_pmic_probe(struct i2c_client *client)
{
	const struct i2c_device_id *i2c_id = i2c_client_get_device_id(client);
	const struct regulator_desc *regulators;
	struct act8865_platform_data *pdata = NULL;
	struct device *dev = &client->dev;
	int i, ret, num_regulators;
	struct act8865 *act8865;
	const struct regmap_config *regmap_config;
	unsigned long type;
	int off_reg, off_mask;
	int voltage_select = 0;

	if (dev->of_node) {
		const struct of_device_id *id;

		id = of_match_device(of_match_ptr(act8865_dt_ids), dev);
		if (!id)
			return -ENODEV;

		type = (unsigned long) id->data;

		voltage_select = of_property_read_bool(dev->of_node, "active-semi,vsel-high");
	} else {
		type = i2c_id->driver_data;
		pdata = dev_get_platdata(dev);
	}

	switch (type) {
	case ACT8600:
		regulators = act8600_regulators;
		num_regulators = ARRAY_SIZE(act8600_regulators);
		regmap_config = &act8600_regmap_config;
		off_reg = -1;
		off_mask = -1;
		break;
	case ACT8846:
		regulators = act8846_regulators;
		num_regulators = ARRAY_SIZE(act8846_regulators);
		regmap_config = &act8865_regmap_config;
		off_reg = ACT8846_GLB_OFF_CTRL;
		off_mask = ACT8846_OFF_SYSMASK;
		break;
	case ACT8865:
		if (voltage_select) {
			regulators = act8865_alt_regulators;
			num_regulators = ARRAY_SIZE(act8865_alt_regulators);
		} else {
			regulators = act8865_regulators;
			num_regulators = ARRAY_SIZE(act8865_regulators);
		}
		regmap_config = &act8865_regmap_config;
		off_reg = ACT8865_SYS_CTRL;
		off_mask = ACT8865_MSTROFF;
		break;
	case ACT8931:
		if (voltage_select) {
			regulators = act8931_alt_regulators;
			num_regulators = ARRAY_SIZE(act8931_alt_regulators);
		} else {
			regulators = act8931_regulators;
			num_regulators = ARRAY_SIZE(act8931_regulators);
		}
		regmap_config = &act8931_regmap_config;
		/* MSTROFF is in SYS 0x01, which the regmap does not write */
		off_reg = -1;
		off_mask = -1;
		break;
	default:
		dev_err(dev, "invalid device id %lu\n", type);
		return -EINVAL;
	}

	act8865 = devm_kzalloc(dev, sizeof(struct act8865), GFP_KERNEL);
	if (!act8865)
		return -ENOMEM;

	act8865->regmap = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR(act8865->regmap)) {
		ret = PTR_ERR(act8865->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	if (of_device_is_system_power_controller(dev->of_node)) {
		if (!pm_power_off && (off_reg > 0)) {
			act8865_i2c_client = client;
			act8865->off_reg = off_reg;
			act8865->off_mask = off_mask;
			pm_power_off = act8865_power_off;
		} else {
			dev_err(dev, "Failed to set poweroff capability, already defined\n");
		}
	}

	/* Finally register devices */
	for (i = 0; i < num_regulators; i++) {
		const struct regulator_desc *desc = &regulators[i];
		struct regulator_config config = { };
		struct regulator_dev *rdev;

		config.dev = dev;
		config.driver_data = act8865;
		config.regmap = act8865->regmap;

		if (pdata) {
			struct act8865_regulator_data *rdata;

			rdata = act8865_get_regulator_data(desc->id, pdata);
			if (rdata) {
				config.init_data = rdata->init_data;
				config.of_node = rdata->of_node;
			}
		}

		rdev = devm_regulator_register(dev, desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(dev, "failed to register %s\n", desc->name);
			return PTR_ERR(rdev);
		}
	}

	if (type == ACT8600) {
		ret = act8600_charger_probe(dev, act8865->regmap);
		if (ret < 0) {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to probe charger");
			return ret;
		}
	}

	/*
	 * The outputs are registered and may have consumers already, so a
	 * charger that fails is reported and left out; the outputs stay.
	 */
	if (type == ACT8931) {
		ret = act8931_charger_probe(dev, act8865->regmap);
		if (ret)
			dev_warn(dev, "Failed to probe charger: %d\n", ret);
	}

	i2c_set_clientdata(client, act8865);

	/* Unlock expert registers for ACT8865. */
	return type != ACT8865 ? 0 : regmap_write(act8865->regmap,
						  ACT8865_SYS_UNLK_REGS, 0xef);
}

static const struct i2c_device_id act8865_ids[] = {
	{ .name = "act8600", .driver_data = ACT8600 },
	{ .name = "act8846", .driver_data = ACT8846 },
	{ .name = "act8865", .driver_data = ACT8865 },
	{ .name = "act8931", .driver_data = ACT8931 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, act8865_ids);

static struct i2c_driver act8865_pmic_driver = {
	.driver	= {
		.name	= "act8865",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= act8865_pmic_probe,
	.id_table	= act8865_ids,
};

module_i2c_driver(act8865_pmic_driver);

MODULE_DESCRIPTION("active-semi act88xx voltage regulator driver");
MODULE_AUTHOR("Wenyou Yang <wenyou.yang@atmel.com>");
MODULE_LICENSE("GPL v2");
