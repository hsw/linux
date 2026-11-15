/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * RK3026 suspend to RAM: what pm.c and the SRAM routine in rk3026-suspend.S
 * share.
 */
#ifndef __MACH_ROCKCHIP_RK3026_SUSPEND_H
#define __MACH_ROCKCHIP_RK3026_SUSPEND_H

/*
 * The routine's table, in words. pm.c fills it in the SRAM copy at boot.
 * The first RK3026_SUSPEND_BASES words are the virtual bases of every
 * region the routine reads or writes: the routine touches each of them
 * before it changes anything, so no access from the SRAM needs a page
 * table walk while the DRAM is in self-refresh.
 */
#define RK3026_SUSPEND_SRAM		0	/* the SRAM section */
#define RK3026_SUSPEND_CRU		1
#define RK3026_SUSPEND_GRF		2
#define RK3026_SUSPEND_PCTL		3	/* the DDR controller */
#define RK3026_SUSPEND_PHY		4	/* the DDR PHY */
#define RK3026_SUSPEND_BASES		5
#define RK3026_SUSPEND_STACK_TOP	5	/* the section's end */
/* the routine saves these here and leaves them as it found them */
#define RK3026_SUSPEND_MCFG		6
#define RK3026_SUSPEND_MCFG1		7
#define RK3026_SUSPEND_SOC_CON2		8
#define RK3026_SUSPEND_PHY_REG1		9
#define RK3026_SUSPEND_PHY_REG264	10
/* the CRU's clock setup the routine changes while it sleeps */
#define RK3026_SUSPEND_MODE_CON		11
#define RK3026_SUSPEND_CLKSEL0		12
#define RK3026_SUSPEND_CLKSEL1		13
#define RK3026_SUSPEND_CLKSEL10		14
/* the PLLs it put into slow mode and powered down: BIT(PLL index) */
#define RK3026_SUSPEND_PLLS		15
#define RK3026_SUSPEND_TABLE_WORDS	16

/* the stack, at the top of the section */
#define RK3026_SUSPEND_STACK_SIZE	256

/*
 * Step codes the routine returns; 0 means it never ran. An E_* code: an
 * entry step failed and the DRAM was brought back, so the sleep fails.
 * The R_* codes are never returned: the DRAM may be unusable, and the
 * routine resets the SoC instead.
 */
#define RK3026_SUSPEND_DONE		0x01
#define RK3026_SUSPEND_E_STATE		0x11	/* the PCTL was not in Access */
#define RK3026_SUSPEND_E_CONFIG		0x12	/* no Config */
#define RK3026_SUSPEND_E_ACCESS		0x13	/* no Access from Config */
#define RK3026_SUSPEND_E_SLEEP		0x14	/* no Low_power (self-refresh) */
#define RK3026_SUSPEND_R_WAKEUP		0x21	/* no Access after the wake-up */
#define RK3026_SUSPEND_R_BACK_OUT	0x22	/* a back-out found no Access */
#define RK3026_SUSPEND_R_CONFIG		0x23	/* no Config to restore MCFG */
#define RK3026_SUSPEND_R_ACCESS		0x24	/* no Access after that */
#define RK3026_SUSPEND_R_DPLL		0x25	/* the DDR PLL did not lock */
#define RK3026_SUSPEND_R_TRAIN_CFG	0x26	/* no Config for the training */
#define RK3026_SUSPEND_R_TRAIN		0x27	/* the training did not end */
#define RK3026_SUSPEND_R_REFRESH	0x28	/* a refresh was not taken */
#define RK3026_SUSPEND_R_TRAIN_GO	0x29	/* no Access after the training */
#define RK3026_SUSPEND_R_APLL		0x2a	/* a PLL did not lock: 0x2a + */
#define RK3026_SUSPEND_R_CPLL		0x2c	/* its index (APLL 0, DPLL 1, */
#define RK3026_SUSPEND_R_GPLL		0x2d	/* CPLL 2, GPLL 3) */

#ifndef __ASSEMBLY__
u32 rk3026_suspend_sram(void);
extern const u32 rk3026_suspend_sram_sz;
extern const u32 rk3026_suspend_sram_table_off;
#endif

#endif /* __MACH_ROCKCHIP_RK3026_SUSPEND_H */
