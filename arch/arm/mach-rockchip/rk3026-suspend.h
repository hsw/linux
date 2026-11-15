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
#define RK3026_SUSPEND_WDT		5	/* debug layer */
#define RK3026_SUSPEND_GICC		6	/* debug: the GIC CPU interface */
#define RK3026_SUSPEND_BASES		7
#define RK3026_SUSPEND_STACK_TOP	7	/* the section's end */
/* the routine saves these here and leaves them as it found them */
#define RK3026_SUSPEND_MCFG		8
#define RK3026_SUSPEND_MCFG1		9
#define RK3026_SUSPEND_SOC_CON2		10
#define RK3026_SUSPEND_PHY_REG1		11
#define RK3026_SUSPEND_PHY_REG264	12
/* the CRU's clock setup the routine changes while it sleeps */
#define RK3026_SUSPEND_MODE_CON		13
#define RK3026_SUSPEND_CLKSEL0		14
#define RK3026_SUSPEND_CLKSEL1		15
#define RK3026_SUSPEND_CLKSEL10		16
/* the PLLs it put into slow mode and powered down: BIT(PLL index) */
#define RK3026_SUSPEND_PLLS		17
#define RK3026_SUSPEND_FAIL_AT		18	/* debug: fail at this point/step */
#define RK3026_SUSPEND_WDT_TORR		19	/* debug: guard TORR, 0 = off */
#define RK3026_SUSPEND_DEPTH		20	/* debug: 0 = no self-refresh */
/* debug: global timer ticks with the auto refresh off, of one 1 us wait */
#define RK3026_SUSPEND_GT_REFOFF	21
#define RK3026_SUSPEND_GT_DELAY		22
#define RK3026_SUSPEND_REG62_PRE	23	/* debug: PHY REG62 at the start */
/* debug: guard TORR while GPLL is in slow mode (pclk_peri /32) */
#define RK3026_SUSPEND_WDT_TORR_SLOW	24
/* debug: the PLLs (BIT(index)) the routine may power down after slow mode */
#define RK3026_SUSPEND_PD_MASK		25
#define RK3026_SUSPEND_TABLE_WORDS	26

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

/*
 * Debug layer (rk3026-pm-debug.c, not for upstream): the routine's points
 * in GRF_OS_REG6, .enter's in GRF_OS_REG7, and the code the routine
 * returns when it backs out at a point (fail_at). From RK3026_PM_SRAM_WOKE
 * on, OS_REG6 bits 15:8 hold the low byte of GICC_HPPIR's interrupt ID:
 * the interrupt that ended the wfi (0xff: none pending).
 *
 * depth: 0 = wfi with the DRAM active, 1 = self-refresh only, 2 = also
 * the PHY reset and clock gate and the DPLL down, 3 = also APLL, CPLL and
 * GPLL slow and down, the CPU on the oscillator (the full routine).
 * fail_at: 1 or 2 back out at that point (code RK3026_SUSPEND_ABORT |
 * point; point 2 comes before the PHY reset); a step code (E_* or R_*)
 * makes that step fail once it succeeded, as if its poll never ended: an
 * E_* step backs out and returns its code, an R_* step resets the SoC.
 * OS_REG6 low byte: the point; RK3026_SUSPEND_ABORT | code after a back
 * out; RK3026_PM_SRAM_RESET | code right before the core reset.
 */
#define RK3026_SUSPEND_ABORT		0x80	/* | point */
#define RK3026_PM_SRAM_ENTRY		1	/* on the SRAM stack */
#define RK3026_PM_SRAM_WFI		2	/* in self-refresh, PHY on */
#define RK3026_PM_SRAM_WOKE		3	/* after wfi */
#define RK3026_PM_SRAM_SR		4	/* in self-refresh (depth >= 1) */
#define RK3026_PM_SRAM_BACK		5	/* the DRAM back in Access */
#define RK3026_PM_SRAM_PHY_OFF		6	/* depth 2: right before wfi */
#define RK3026_PM_SRAM_PHY_ON		7	/* DPLL locked, PHY out of reset */
#define RK3026_PM_SRAM_TRAINED		8	/* trained, refreshes sent */
#define RK3026_PM_SRAM_PLL_SLOW		9	/* depth 3: PLLs slow, CPU on 24M */
#define RK3026_PM_SRAM_PLL_OFF		10	/* depth 3: PLLs down, before wfi */
#define RK3026_PM_SRAM_PLL_ON		11	/* the PLLs locked, back in mode */
#define RK3026_PM_SRAM_RESET		0x40	/* | code: core reset */
#define RK3026_PM_DEPTH_MAX		3
#define RK3026_PM_C_ENTER		1
#define RK3026_PM_C_RETURNED		2	/* the routine returned */
#define RK3026_PM_C_DONE		3	/* .enter returns 0 */
#define RK3026_PM_C_FAILED		4	/* .enter returns -EIO */
#define RK3026_PM_C_REFUSED		5	/* refused before the routine */
#define RK3026_PM_CRUMB_SRAM		0x50340000
#define RK3026_PM_CRUMB_C		0x50430000
#define RK3026_GRF_OS_REG6		0x1e0
#define RK3026_GICC_HPPIR		0x18
/* the A9 global timer's counter (low word), on the GICC's page */
#define RK3026_GICC_GT_COUNT		0x100
#define DW_WDT_CRR			0x0c
#define DW_WDT_CRR_KICK			0x76
#define DW_WDT_TORR			0x04

#ifndef __ASSEMBLY__
u32 rk3026_suspend_sram(void);
extern const u32 rk3026_suspend_sram_sz;
extern const u32 rk3026_suspend_sram_table_off;

struct platform_suspend_ops;
void rk3026_pm_debug_init(u32 *table, const struct platform_suspend_ops *ops);
int rk3026_pm_debug_begin(void);
void rk3026_pm_debug_woke(u32 code);
void rk3026_pm_debug_disarm(void);
void rk3026_pm_debug_end(int ret);
void rk3026_pm_debug_again(bool again);
#endif

#endif /* __MACH_ROCKCHIP_RK3026_SUSPEND_H */
