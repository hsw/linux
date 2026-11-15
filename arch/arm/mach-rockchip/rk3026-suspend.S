/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * RK3026 suspend to RAM: the steps that run from the internal SRAM.
 *
 * pm.c copies this routine with its table to the
 * "rockchip,rk3026-suspend-sram" section once at boot, through a
 * non-cacheable executable mapping, and fills the table with the virtual
 * bases the routine uses. It is entered from the cpu_suspend() finisher
 * with the MMU on, the L2 cache off and interrupts masked. It moves the
 * stack into the SRAM and waits for a wake-up interrupt. Position
 * independent: it reaches its table PC-relative.
 *
 * Returns a step code, RK3026_SUSPEND_DONE after a normal wake-up.
 */
#include <linux/linkage.h>
#include "rk3026-suspend.h"

	.arch	armv7-a
	.arm
	.text
	.align	3
ENTRY(rk3026_suspend_sram)
	adr	r2, rk3026_suspend_sram_table
	mov	r1, sp
	ldr	sp, [r2, #RK3026_SUSPEND_STACK_TOP * 4]
	push	{r1, lr}

	dsb
	wfi

	mov	r0, #RK3026_SUSPEND_DONE
	pop	{r1, lr}
	mov	sp, r1
	bx	lr
ENDPROC(rk3026_suspend_sram)

	.align	2
rk3026_suspend_sram_table:
	.space	RK3026_SUSPEND_TABLE_WORDS * 4

	/* the bytes to copy, and where the table starts in them */
	.globl	rk3026_suspend_sram_sz
rk3026_suspend_sram_sz:
	.word	. - rk3026_suspend_sram
	.globl	rk3026_suspend_sram_table_off
rk3026_suspend_sram_table_off:
	.word	rk3026_suspend_sram_table - rk3026_suspend_sram
