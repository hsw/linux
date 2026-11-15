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
 * region the routine reads or writes: the finisher touches each of them
 * before the call, so no access from the SRAM needs a page table walk.
 */
#define RK3026_SUSPEND_SRAM		0	/* the SRAM section */
#define RK3026_SUSPEND_CRU		1
#define RK3026_SUSPEND_GRF		2
#define RK3026_SUSPEND_WDT		3	/* debug layer */
#define RK3026_SUSPEND_GICC		4	/* debug: the GIC CPU interface */
#define RK3026_SUSPEND_BASES		5
#define RK3026_SUSPEND_STACK_TOP	5	/* the section's end */
#define RK3026_SUSPEND_FAIL_AT		6	/* debug: back out at this point */
#define RK3026_SUSPEND_WDT_TORR		7	/* debug: guard TORR, 0 = off */
#define RK3026_SUSPEND_TABLE_WORDS	8

/* the stack, at the top of the section */
#define RK3026_SUSPEND_STACK_SIZE	256

/* step codes the routine returns; 0 means it never ran */
#define RK3026_SUSPEND_DONE		1

/*
 * Debug layer (rk3026-pm-debug.c, not for upstream): the routine's points
 * in GRF_OS_REG6, .enter's in GRF_OS_REG7, and the code the routine
 * returns when it backs out at a point (fail_at). At RK3026_PM_SRAM_WOKE,
 * OS_REG6 bits 15:8 hold the low byte of GICC_HPPIR's interrupt ID: the
 * interrupt that ended the wfi (0xff: none pending).
 */
#define RK3026_SUSPEND_ABORT		0x80	/* | point */
#define RK3026_PM_SRAM_ENTRY		1	/* on the SRAM stack */
#define RK3026_PM_SRAM_WFI		2	/* right before wfi */
#define RK3026_PM_SRAM_WOKE		3	/* after wfi */
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
