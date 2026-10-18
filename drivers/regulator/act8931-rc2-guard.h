/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * LOCAL, RC2 stack only, not for upstream: compiled-in stock limits for
 * every ACT8931 register write, on top of the driver's own rules.
 *
 * The driver bounds a VSET write by the output's regulator-max-microvolt.
 * On the ONYX BOOX Robinson Crusoe 2 a wrong device tree must not be able
 * to raise an output above what the stock kernel ever sets, so the write
 * callback also asks this table (owner rule: a hang or a power-off is
 * acceptable, a burnt part is not):
 *  - VSET selectors (VSET1 and VSET2 of a DC/DC, VSET of an LDO) at most
 *    the stock maximum: DCDC1 0x3b (3.5 V, stock VCCIO with Wi-Fi/BT on),
 *    DCDC2 0x1b (1.35 V, DDR), DCDC3 0x1c (1.40 V, the RK3026's absolute
 *    maximum), LDO1-4 0x37 (3.1 V); no lower limit;
 *  - reserved bits (VSET 7:6, DCDC3 CTRL 6) equal to the read-back;
 *  - the CTRL registers otherwise free;
 *  - every other register refused: SYS 0x00/0x01 (SYSLEV, MSTROFF),
 *    APCH 0x71 (safety timers, input OVP), 0x78/0x79 (charger interrupt
 *    arming), and anything outside the output map (0x02, 0x03, 0x0b,
 *    0x70, the ACT8865 *_SUS addresses, 0x7a, ...). The battery limits of
 *    the bring-up driver (SYSLEV >= 3.0 V, timers no longer than stock,
 *    OVP 6.6 V) hold because none of these registers can be written.
 *
 * Self-contained (no kernel header) so that a host test can include it.
 */
#ifndef ACT8931_RC2_GUARD_H
#define ACT8931_RC2_GUARD_H

#define ACT8931_RC2_VSET_RESERVED	0xc0	/* VSET bits 7:6 */
#define ACT8931_RC2_SEL_MASK		0x3f	/* VSET bits 5:0 */
#define ACT8931_RC2_DCDC3_CTRL_RESERVED	0x40	/* DCDC3 CTRL bit 6 */

#define ACT8931_RC2_CAP_DCDC1	0x3b	/* 3.5 V */
#define ACT8931_RC2_CAP_DCDC2	0x1b	/* 1.35 V */
#define ACT8931_RC2_CAP_DCDC3	0x1c	/* 1.40 V: never raise */
#define ACT8931_RC2_CAP_LDO	0x37	/* 3.1 V */

struct act8931_rc2_rule {
	unsigned char reg;
	unsigned char vset;	/* 1: the selector is capped at cap */
	unsigned char cap;
	unsigned char keep;	/* bits that must equal the read-back */
};

#define ACT8931_RC2_VSET(_reg, _cap) \
	{ .reg = (_reg), .vset = 1, .cap = (_cap), \
	  .keep = ACT8931_RC2_VSET_RESERVED }
#define ACT8931_RC2_CTRL(_reg, _keep) { .reg = (_reg), .keep = (_keep) }

static const struct act8931_rc2_rule act8931_rc2_rules[] = {
	ACT8931_RC2_VSET(0x20, ACT8931_RC2_CAP_DCDC1),
	ACT8931_RC2_VSET(0x21, ACT8931_RC2_CAP_DCDC1),
	ACT8931_RC2_CTRL(0x22, 0),
	ACT8931_RC2_VSET(0x30, ACT8931_RC2_CAP_DCDC2),
	ACT8931_RC2_VSET(0x31, ACT8931_RC2_CAP_DCDC2),
	ACT8931_RC2_CTRL(0x32, 0),
	ACT8931_RC2_VSET(0x40, ACT8931_RC2_CAP_DCDC3),
	ACT8931_RC2_VSET(0x41, ACT8931_RC2_CAP_DCDC3),
	ACT8931_RC2_CTRL(0x42, ACT8931_RC2_DCDC3_CTRL_RESERVED),
	ACT8931_RC2_VSET(0x50, ACT8931_RC2_CAP_LDO),
	ACT8931_RC2_CTRL(0x51, 0),
	ACT8931_RC2_VSET(0x54, ACT8931_RC2_CAP_LDO),
	ACT8931_RC2_CTRL(0x55, 0),
	ACT8931_RC2_VSET(0x60, ACT8931_RC2_CAP_LDO),
	ACT8931_RC2_CTRL(0x61, 0),
	ACT8931_RC2_VSET(0x64, ACT8931_RC2_CAP_LDO),
	ACT8931_RC2_CTRL(0x65, 0),
};

/*
 * NULL when the stock limits let the write of @val to @reg through, with
 * the bits that must keep their read-back value in *@keep; otherwise the
 * reason it is refused.
 */
static inline const char *act8931_rc2_guard(unsigned int reg,
					    unsigned int val,
					    unsigned int *keep)
{
	unsigned int i;

	*keep = 0;
	for (i = 0; i < sizeof(act8931_rc2_rules) /
			sizeof(act8931_rc2_rules[0]); i++) {
		const struct act8931_rc2_rule *r = &act8931_rc2_rules[i];

		if (r->reg != reg)
			continue;
		if (val > 0xff)
			return "not a byte";
		if (r->vset && (val & ACT8931_RC2_SEL_MASK) > r->cap)
			return "above the RC2 stock cap";
		*keep = r->keep;
		return NULL;
	}
	return "not in the RC2 output map";
}

#endif /* ACT8931_RC2_GUARD_H */
