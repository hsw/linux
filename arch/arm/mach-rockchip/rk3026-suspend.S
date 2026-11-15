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

	/* debug layer: the point reached, to GRF_OS_REG6 (r2 = the table) */
	.macro	crumb, point
	ldr	r3, [r2, #RK3026_SUSPEND_GRF * 4]
	movw	r0, #\point
	movt	r0, #(RK3026_PM_CRUMB_SRAM >> 16)
	str	r0, [r3, #RK3026_GRF_OS_REG6]
	dsb
	.endm

	/* debug layer: back out at this point when fail_at names it */
	.macro	fail_at, point
	ldr	r0, [r2, #RK3026_SUSPEND_FAIL_AT * 4]
	cmp	r0, #\point
	moveq	r0, #(RK3026_SUSPEND_ABORT | \point)
	beq	.Lback_out
	.endm

	.arch	armv7-a
	.arm
	.text
	.align	3
ENTRY(rk3026_suspend_sram)
	adr	r2, rk3026_suspend_sram_table
	mov	r1, sp
	ldr	sp, [r2, #RK3026_SUSPEND_STACK_TOP * 4]
	push	{r1, lr}
	crumb	RK3026_PM_SRAM_ENTRY
	fail_at	RK3026_PM_SRAM_ENTRY

	/* debug guard: restart the watchdog with its sleep timeout */
	ldr	r0, [r2, #RK3026_SUSPEND_WDT_TORR * 4]
	cmp	r0, #0
	ldrne	r3, [r2, #RK3026_SUSPEND_WDT * 4]
	strne	r0, [r3, #DW_WDT_TORR]
	movne	r0, #DW_WDT_CRR_KICK
	strne	r0, [r3, #DW_WDT_CRR]
	crumb	RK3026_PM_SRAM_WFI
	fail_at	RK3026_PM_SRAM_WFI

	dsb
	wfi

	/*
	 * debug: the point, with the interrupt that ended the wfi in bits
	 * 15:8 (the low byte of GICC_HPPIR's ID; 0xff when none is pending)
	 */
	ldr	r3, [r2, #RK3026_SUSPEND_GICC * 4]
	ldr	r1, [r3, #RK3026_GICC_HPPIR]
	and	r1, r1, #0xff
	ldr	r3, [r2, #RK3026_SUSPEND_GRF * 4]
	movw	r0, #RK3026_PM_SRAM_WOKE
	movt	r0, #(RK3026_PM_CRUMB_SRAM >> 16)
	orr	r0, r0, r1, lsl #8
	str	r0, [r3, #RK3026_GRF_OS_REG6]
	dsb
	mov	r0, #RK3026_SUSPEND_DONE
.Lreturn:
	pop	{r1, lr}
	mov	sp, r1
	bx	lr

	/* debug: r0 = the code; the point with the abort flag to OS_REG6 */
.Lback_out:
	ldr	r3, [r2, #RK3026_SUSPEND_GRF * 4]
	movw	r12, #0
	movt	r12, #(RK3026_PM_CRUMB_SRAM >> 16)
	orr	r12, r12, r0
	str	r12, [r3, #RK3026_GRF_OS_REG6]
	dsb
	b	.Lreturn
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
