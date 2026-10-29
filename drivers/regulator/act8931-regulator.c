// SPDX-License-Identifier: GPL-2.0-only
/*
 * Active-semi ACT8931 PMU: regulators and charger state.
 *
 * Only the VSET and CTRL registers of the seven outputs are writable (the
 * regmap's access table): the driver never writes the system, charger or
 * undocumented registers. Every write keeps the bits it does not change.
 *
 * The ON bit is edge-triggered, so ON is the last edge written, not a
 * measurement. OK (power good) is set while the output is in regulation.
 *
 * The charger is only read (APCH 0x78 and 0x7a), and polled: the driver
 * enables none of its interrupts. It is registered after the outputs, and
 * a charger failure leaves them registered.
 *
 * There is no shutdown callback: the chip keeps its registers across a
 * warm reset, so the boot firmware starts with the voltages the consumers
 * left (for a CPU supply, cpufreq's suspend operating point if the board
 * has one).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/linear_range.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/workqueue.h>

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
#define ACT8931_APCH_STATUS	0x78
#define ACT8931_APCH_STATE	0x7a

#define ACT8931_VSEL_MASK	GENMASK(5, 0)
#define ACT8931_CTRL_ON		BIT(7)
#define ACT8931_CTRL_DELAY	GENMASK(4, 2)	/* turn-on delay */
#define ACT8931_CTRL_OK		BIT(0)	/* power good */
#define ACT8931_N_VOLTAGES	64

#define ACT8931_STATUS_CHGDAT	BIT(0)	/* end of charge */
#define ACT8931_STATUS_INDAT	BIT(1)	/* valid input at CHGIN */
#define ACT8931_STATE_CSTATE	GENMASK(5, 4)
/* the bits the properties are made of (0x78, 0x7a; they do not overlap) */
#define ACT8931_STATUS_WATCH	(ACT8931_STATUS_CHGDAT | ACT8931_STATUS_INDAT)
#define ACT8931_STATE_WATCH	ACT8931_STATE_CSTATE

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
/* DELAY n > 0 is 2^n ms, up to 12% longer: 2^n units of 1 ms + 12% */
#define ACT8931_DELAY_UNIT_US	1120

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

struct act8931 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *charger;
	struct delayed_work poll;
	unsigned int last;	/* watched charger bits at the last read */
};

/* ACT8931 output voltage table: three linear ranges of the 6-bit selector */
static const struct linear_range act8931_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(600000, 0, 23, 25000),
	REGULATOR_LINEAR_RANGE(1200000, 24, 47, 50000),
	REGULATOR_LINEAR_RANGE(2400000, 48, 63, 100000),
};

/* The turn-on delay set in CTRL plus the soft-start */
static unsigned int act8931_turn_on_us(unsigned int ctrl)
{
	unsigned int delay = FIELD_GET(ACT8931_CTRL_DELAY, ctrl);

	return ACT8931_SOFTSTART_US + (delay ? ACT8931_DELAY_UNIT_US << delay : 0);
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

/*
 * The core enables without a wait when this fails, and the enable reads
 * CTRL again: a failed read gives the longest turn-on time instead.
 */
static int act8931_enable_time(struct regulator_dev *rdev)
{
	unsigned int ctrl;

	if (regmap_read(rdev_get_regmap(rdev), rdev->desc->enable_reg, &ctrl))
		ctrl = ACT8931_CTRL_DELAY;
	return act8931_turn_on_us(ctrl);
}

/*
 * Enable: ON 0 then ON 1 from the value read back, the datasheet's
 * sequence; nothing is written when the output is on already (the core
 * enables an always-on or boot-on output at registration). An output that
 * reads ON without power good is never toggled, since the falling edge
 * would cut a load that may still run from it. A DC/DC in that state is
 * on as far as the core is concerned (it reads enabled by ON alone), and
 * its status reports the error. An LDO may still be ramping up: it gets
 * its turn-on time, and the enable fails if OK is still low then; the core
 * then fails the registration of an always-on or boot-on LDO in that
 * state, and with it the probe.
 */
static int act8931_enable(struct regulator_dev *rdev)
{
	struct regmap *map = rdev_get_regmap(rdev);
	unsigned int reg = rdev->desc->enable_reg;
	unsigned int ctrl;
	int ret;

	ret = regmap_read(map, reg, &ctrl);
	if (ret)
		return ret;
	if ((ctrl & ACT8931_CTRL_ON) &&
	    !(rdev->desc->enable_mask & ACT8931_CTRL_OK)) {
		if (!(ctrl & ACT8931_CTRL_OK))
			dev_warn(rdev->dev.parent,
				 "%s: ON without power good (CTRL 0x%02x)\n",
				 rdev->desc->name, ctrl);
		return 0;
	}
	if ((ctrl & ACT8931_CTRL_ON) && !(ctrl & ACT8931_CTRL_OK)) {
		fsleep(act8931_turn_on_us(ctrl));
		ret = regmap_read(map, reg, &ctrl);
		if (ret)
			return ret;
		if (!(ctrl & ACT8931_CTRL_OK)) {
			dev_err(rdev->dev.parent,
				"%s: ON without power good (CTRL 0x%02x): not toggled\n",
				rdev->desc->name, ctrl);
			return -EIO;
		}
	}
	if (ctrl & ACT8931_CTRL_ON)
		return 0;

	ret = regmap_write(map, reg, ctrl & ~ACT8931_CTRL_ON);
	if (ret)
		return ret;
	return regmap_write(map, reg, ctrl | ACT8931_CTRL_ON);
}

/*
 * Disable: ON 1 then ON 0. Nothing is written when ON reads 0: an output
 * kept on by the power-up sequence with ON at 0 cannot be told from one
 * that is off (an LDO that is off may read OK 1). An output that reads ON
 * without power good gets the falling edge too: switching it off is what
 * was asked, and no edge switches it back on.
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
 * The wait after a voltage change: the node's ramp delay or settling time
 * by the core's rules, and at least the soft-start time for a raise (no
 * slew rate is specified). It is timed from the voltages of the old and
 * the new selector (regulator_set_voltage_time_sel()), so a
 * regulator-microvolt-offset does not shift them. After a set of a DC/DC
 * the core calls that as the set_voltage_time_sel op, for an output that
 * reads enabled; an LDO waits in its own set op.
 */
static int act8931_set_voltage_time(struct regulator_dev *rdev,
				    int old_uV, int new_uV)
{
	const struct regulation_constraints *c = rdev->constraints;
	int us;

	if (c->ramp_delay)
		us = DIV_ROUND_UP(abs(new_uV - old_uV), c->ramp_delay);
	else if (c->settling_time)
		us = c->settling_time;
	else if (new_uV > old_uV)
		us = c->settling_time_up;
	else if (new_uV < old_uV)
		us = c->settling_time_down;
	else
		us = 0;

	return new_uV > old_uV ? max(us, ACT8931_SOFTSTART_US) : us;
}

/*
 * An LDO reads enabled by ON and OK, and the core waits after a voltage
 * change only for an output that reads enabled; an LDO whose OK drops may
 * still be live, and ON 0 does not show that it is off (act8931_disable()).
 * So the LDO set op waits itself after every change of the selector,
 * whatever ON and OK read, and the LDO set_voltage_time_sel op gives the
 * core no wait of its own: the wait is neither skipped nor doubled.
 */
static int act8931_ldo_set_voltage_sel(struct regulator_dev *rdev,
				       unsigned int sel)
{
	unsigned int old_sel;
	int ret;

	ret = regulator_get_voltage_sel_regmap(rdev);
	if (ret < 0)
		return ret;
	old_sel = ret;

	ret = regulator_set_voltage_sel_regmap(rdev, sel);
	if (ret || old_sel == sel)
		return ret;

	ret = regulator_set_voltage_time_sel(rdev, old_sel, sel);
	if (ret > 0)
		fsleep(ret);
	return 0;
}

static int act8931_ldo_set_voltage_time_sel(struct regulator_dev *rdev,
					    unsigned int old_sel,
					    unsigned int new_sel)
{
	return 0;	/* act8931_ldo_set_voltage_sel() has waited */
}

/*
 * The DC/DC converters have no disable op by driver policy. They supply the
 * SoC (I/O, DDR, CPU on the boards seen so far), and with no op neither a
 * consumer nor the late cleanup can switch them off; the chip itself can
 * switch them through their ON edges, and its power-on sequence starts them
 * (their DELAY bits are its OTP sequencing).
 * They have the enable op, which writes nothing to an output that is on:
 * with it the core keeps a node's always-on and the supply that holds. A
 * disable (a consumer's last one, the unused-regulator pass for a node
 * without always-on) reaches no op and leaves the output on.
 *
 * A DC/DC reads enabled by ON alone, and OK goes to the status: OK low
 * does not switch it off, and the core keeps the voltage-change wait and
 * regulator-max-step-microvolt stepping for the CPU or memory supply
 * while its OK drops. An LDO reads enabled by ON and OK
 * (act8931_ldo_set_voltage_sel() for its wait).
 */
static const struct regulator_ops act8931_dcdc_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
	.set_voltage_time	= act8931_set_voltage_time,
	.enable			= act8931_enable,
	.enable_time		= act8931_enable_time,
	.is_enabled		= regulator_is_enabled_regmap,
	.get_status		= act8931_get_status,
};

static const struct regulator_ops act8931_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= act8931_ldo_set_voltage_sel,
	.set_voltage_time_sel	= act8931_ldo_set_voltage_time_sel,
	.set_voltage_time	= act8931_set_voltage_time,
	.enable			= act8931_enable,
	.disable		= act8931_disable,
	.enable_time		= act8931_enable_time,
	.is_enabled		= regulator_is_enabled_regmap,
	.get_status		= act8931_get_status,
};

/* A DC/DC reads enabled by ON alone, an LDO by ON and OK (act8931_dcdc_ops) */
#define ACT8931_EN_BITS(_id)						\
	((_id) <= ACT8931_ID_DCDC3 ? ACT8931_CTRL_ON :			\
	 ACT8931_CTRL_ON | ACT8931_CTRL_OK)

#define ACT8931_REG(_id, _name, _match, _supply, _vset, _ctrl, _ops)	\
	[ACT8931_ID_##_id] = {						\
		.name			= _name,			\
		.of_match		= _match,			\
		.regulators_node	= "regulators",			\
		.supply_name		= _supply,			\
		.id			= ACT8931_ID_##_id,		\
		.type			= REGULATOR_VOLTAGE,		\
		.owner			= THIS_MODULE,			\
		.ops			= &(_ops),			\
		.n_voltages		= ACT8931_N_VOLTAGES,		\
		.linear_ranges		= act8931_voltage_ranges,	\
		.n_linear_ranges	= ARRAY_SIZE(act8931_voltage_ranges), \
		.vsel_reg		= _vset,			\
		.vsel_mask		= ACT8931_VSEL_MASK,		\
		.enable_reg		= _ctrl,			\
		.enable_mask		= ACT8931_EN_BITS(ACT8931_ID_##_id), \
		.enable_val		= ACT8931_EN_BITS(ACT8931_ID_##_id), \
	}

/* _vset: VSET1 or VSET2, the bank the VSEL pin selects */
#define ACT8931_DCDC(_n, _vset)						\
	ACT8931_REG(DCDC##_n, "act8931-dcdc" #_n, "DCDC_REG" #_n,	\
		    "vp" #_n, ACT8931_DCDC##_n##_##_vset,		\
		    ACT8931_DCDC##_n##_CTRL, act8931_dcdc_ops)

#define ACT8931_LDO(_n)							\
	ACT8931_REG(LDO##_n, "act8931-ldo" #_n, "LDO_REG" #_n, "inl",	\
		    ACT8931_LDO##_n##_VSET, ACT8931_LDO##_n##_CTRL,	\
		    act8931_ldo_ops)

/* VSEL high: the DC/DC outputs run from VSET2 */
static const struct regulator_desc act8931_descs_vset2[ACT8931_NUM_RAILS] = {
	ACT8931_DCDC(1, VSET2),
	ACT8931_DCDC(2, VSET2),
	ACT8931_DCDC(3, VSET2),
	ACT8931_LDO(1),
	ACT8931_LDO(2),
	ACT8931_LDO(3),
	ACT8931_LDO(4),
};

/* VSEL low: the DC/DC outputs run from VSET1 */
static const struct regulator_desc act8931_descs_vset1[ACT8931_NUM_RAILS] = {
	ACT8931_DCDC(1, VSET1),
	ACT8931_DCDC(2, VSET1),
	ACT8931_DCDC(3, VSET1),
	ACT8931_LDO(1),
	ACT8931_LDO(2),
	ACT8931_LDO(3),
	ACT8931_LDO(4),
};

/*
 * The VSET and CTRL registers of the seven outputs, in the first
 * ACT8931_NUM_RAILS entries, are readable and writable; the charger state
 * after them (APCH 0x78, 0x7a) is only readable.
 */
static const struct regmap_range act8931_ranges[] = {
	regmap_reg_range(ACT8931_DCDC1_VSET1, ACT8931_DCDC1_CTRL),
	regmap_reg_range(ACT8931_DCDC2_VSET1, ACT8931_DCDC2_CTRL),
	regmap_reg_range(ACT8931_DCDC3_VSET1, ACT8931_DCDC3_CTRL),
	regmap_reg_range(ACT8931_LDO1_VSET, ACT8931_LDO1_CTRL),
	regmap_reg_range(ACT8931_LDO2_VSET, ACT8931_LDO2_CTRL),
	regmap_reg_range(ACT8931_LDO3_VSET, ACT8931_LDO3_CTRL),
	regmap_reg_range(ACT8931_LDO4_VSET, ACT8931_LDO4_CTRL),
	regmap_reg_range(ACT8931_APCH_STATUS, ACT8931_APCH_STATUS),
	regmap_reg_range(ACT8931_APCH_STATE, ACT8931_APCH_STATE),
};

static_assert(ARRAY_SIZE(act8931_ranges) == ACT8931_NUM_RAILS + 2);

static const struct regmap_access_table act8931_writable_table = {
	.yes_ranges	= act8931_ranges,
	.n_yes_ranges	= ACT8931_NUM_RAILS,
};

static const struct regmap_access_table act8931_readable_table = {
	.yes_ranges	= act8931_ranges,
	.n_yes_ranges	= ARRAY_SIZE(act8931_ranges),
};

/*
 * APCH 0x78 bits 7:4 read back the charger interrupt status, which a
 * read clears
 */
static const struct regmap_range act8931_precious_ranges[] = {
	regmap_reg_range(ACT8931_APCH_STATUS, ACT8931_APCH_STATUS),
};

static const struct regmap_access_table act8931_precious_table = {
	.yes_ranges	= act8931_precious_ranges,
	.n_yes_ranges	= ARRAY_SIZE(act8931_precious_ranges),
};

/* No cache: ON and OK must be read from the chip */
static const struct regmap_config act8931_regmap_config = {
	.name		= "act8931",
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= ACT8931_APCH_STATE,
	.wr_table	= &act8931_writable_table,
	.rd_table	= &act8931_readable_table,
	.precious_table	= &act8931_precious_table,
};

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

/*
 * The chip cannot tell what feeds CHGIN, a USB port or an adapter (ACIN
 * only selects the current mode, which is board wiring): the type is
 * fixed and unknown.
 */
static const struct power_supply_desc act8931_charger_desc = {
	.name		= "act8931-charger",
	.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.properties	= act8931_charger_props,
	.num_properties	= ARRAY_SIZE(act8931_charger_props),
	.get_property	= act8931_charger_get_property,
};

static void act8931_poll_arm(struct act8931 *chip)
{
	queue_delayed_work(system_freezable_power_efficient_wq, &chip->poll,
			   msecs_to_jiffies(ACT8931_POLL_MS));
}

/* True when the watched bits changed; a failed read is no change */
static bool act8931_charger_changed(struct act8931 *chip)
{
	unsigned int status, state, cur;

	if (act8931_charger_read(chip, &status, &state))
		return false;
	cur = (status & ACT8931_STATUS_WATCH) | (state & ACT8931_STATE_WATCH);
	if (cur == chip->last)
		return false;
	chip->last = cur;
	return true;
}

static void act8931_charger_poll(struct work_struct *work)
{
	struct act8931 *chip = container_of(work, struct act8931, poll.work);

	if (act8931_charger_changed(chip))
		power_supply_changed(chip->charger);
	act8931_poll_arm(chip);
}

/*
 * The charger only reports state: it comes after the outputs, and a
 * failure here is logged and leaves them registered. Its node is the
 * chip's, so a fuel gauge can name it in power-supplies.
 */
static void act8931_charger_init(struct act8931 *chip)
{
	struct power_supply_config psy_cfg = {
		.drv_data = chip,
		.fwnode = dev_fwnode(chip->dev),
	};
	int ret;

	if (!IS_ENABLED(CONFIG_POWER_SUPPLY))
		return;

	act8931_charger_changed(chip);	/* the state at registration */
	chip->charger = devm_power_supply_register(chip->dev,
						   &act8931_charger_desc,
						   &psy_cfg);
	if (IS_ERR(chip->charger)) {
		dev_warn(chip->dev, "failed to register the charger: %pe\n",
			 chip->charger);
		return;
	}

	ret = devm_delayed_work_autocancel(chip->dev, &chip->poll,
					   act8931_charger_poll);
	if (ret) {
		dev_warn(chip->dev, "failed to poll the charger: %pe\n",
			 ERR_PTR(ret));
		return;
	}
	act8931_poll_arm(chip);
}

static int act8931_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	const struct regulator_desc *desc;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct act8931 *chip;
	unsigned int i;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->dev = dev;

	chip->regmap = devm_regmap_init_i2c(client, &act8931_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "regmap init failed\n");

	if (of_property_read_bool(dev->of_node, "active-semi,vsel-high"))
		desc = act8931_descs_vset2;
	else
		desc = act8931_descs_vset1;

	config.dev = dev;
	config.regmap = chip->regmap;
	for (i = 0; i < ACT8931_NUM_RAILS; i++) {
		rdev = devm_regulator_register(dev, &desc[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     desc[i].name);
	}

	act8931_charger_init(chip);
	return 0;
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
