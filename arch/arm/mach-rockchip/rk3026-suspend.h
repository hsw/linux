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
#define RK3026_SUSPEND_BASES		3
#define RK3026_SUSPEND_STACK_TOP	3	/* the section's end */
#define RK3026_SUSPEND_TABLE_WORDS	4

/* the stack, at the top of the section */
#define RK3026_SUSPEND_STACK_SIZE	256

/* step codes the routine returns; 0 means it never ran */
#define RK3026_SUSPEND_DONE		1

#ifndef __ASSEMBLY__
u32 rk3026_suspend_sram(void);
extern const u32 rk3026_suspend_sram_sz;
extern const u32 rk3026_suspend_sram_table_off;
#endif

#endif /* __MACH_ROCKCHIP_RK3026_SUSPEND_H */
