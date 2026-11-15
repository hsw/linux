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
#define RK3026_SUSPEND_WDT		4	/* debug layer */
#define RK3026_SUSPEND_GICC		5	/* debug: the GIC CPU interface */
#define RK3026_SUSPEND_BASES		6
#define RK3026_SUSPEND_STACK_TOP	6	/* the section's end */
/* the routine saves these three here and leaves them as it found them */
#define RK3026_SUSPEND_MCFG		7
#define RK3026_SUSPEND_MCFG1		8
#define RK3026_SUSPEND_SOC_CON2		9
#define RK3026_SUSPEND_FAIL_AT		10	/* debug: fail at this point/step */
#define RK3026_SUSPEND_WDT_TORR		11	/* debug: guard TORR, 0 = off */
#define RK3026_SUSPEND_DEPTH		12	/* debug: 0 = no self-refresh */
#define RK3026_SUSPEND_TABLE_WORDS	13

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

/*
 * Debug layer (rk3026-pm-debug.c, not for upstream): the routine's points
 * in GRF_OS_REG6, .enter's in GRF_OS_REG7, and the code the routine
 * returns when it backs out at a point (fail_at). From RK3026_PM_SRAM_WOKE
 * on, OS_REG6 bits 15:8 hold the low byte of GICC_HPPIR's interrupt ID:
 * the interrupt that ended the wfi (0xff: none pending).
 *
 * fail_at: 1 or 2 back out at that point (code RK3026_SUSPEND_ABORT |
 * point); a step code (RK3026_SUSPEND_E_* or _R_*) makes that step's
 * STAT poll fail once the state is reached, as if it never came: an E_*
 * step backs out and returns its code, an R_* step resets the SoC.
 * OS_REG6 low byte: the point; RK3026_SUSPEND_ABORT | code after a back
 * out; RK3026_PM_SRAM_RESET | code right before the core reset.
 */
#define RK3026_SUSPEND_ABORT		0x80	/* | point */
#define RK3026_PM_SRAM_ENTRY		1	/* on the SRAM stack */
#define RK3026_PM_SRAM_WFI		2	/* right before wfi */
#define RK3026_PM_SRAM_WOKE		3	/* after wfi */
#define RK3026_PM_SRAM_SR		4	/* in self-refresh (depth 1) */
#define RK3026_PM_SRAM_BACK		5	/* the DRAM back in Access */
#define RK3026_PM_SRAM_RESET		0x40	/* | code: core reset */
#define RK3026_PM_DEPTH_MAX		1
#define RK3026_PM_C_ENTER		1
#define RK3026_PM_C_RETURNED		2	/* the routine returned */
#define RK3026_PM_C_DONE		3	/* .enter returns 0 */
#define RK3026_PM_C_FAILED		4	/* .enter returns -EIO */
#define RK3026_PM_C_REFUSED		5	/* refused before the routine */
#define RK3026_PM_CRUMB_SRAM		0x50340000
#define RK3026_PM_CRUMB_C		0x50430000
#define RK3026_GRF_OS_REG6		0x1e0
#define RK3026_GICC_HPPIR		0x18
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
