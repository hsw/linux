/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * RK3026 suspend to RAM: the steps that run from the internal SRAM.
 *
 * pm.c copies this routine with its table to the
 * "rockchip,rk3026-suspend-sram" section once at boot, through a
 * non-cacheable executable mapping, and fills the table with the virtual
 * bases the routine uses. It is entered from the cpu_suspend() finisher
 * with the MMU on, the L2 cache off and interrupts masked. It moves the
 * stack into the SRAM, touches every page it reaches, puts the DRAM into
 * self-refresh, waits for a wake-up interrupt and brings the DRAM back.
 * Position independent: it reaches its table PC-relative.
 *
 * The DDR controller sequence follows Rockchip's RK2928/RK3026 vendor
 * kernel (arch/arm/mach-rk2928/ddr.c, GPL-2.0, Copyright (C) 2012 Fuzhou
 * Rockchip Electronics Co., Ltd), with every poll bounded and the
 * controller's configuration given back exactly as it was found.
 *
 * Returns a step code: RK3026_SUSPEND_DONE after a normal wake-up, or the
 * RK3026_SUSPEND_E_* code of the entry step that failed, after the DRAM
 * was brought back. Once the DRAM may be unusable (a wake-up that does
 * not come back to Access), it never returns into DDR: it resets the SoC
 * from the SRAM.
 */
#include <linux/linkage.h>
#include "rk3026-core-reset.S"
#include "rk3026-suspend.h"

#define PCTL_SCTL		0x04
#define PCTL_STAT		0x08
#define PCTL_MCFG1		0x7c
#define PCTL_MCFG		0x80
#define PCTL_STAT_MASK		0x7
/* STAT bits 6:4, what put the controller into Low_power; 1: MCFG1 sr_idle */
#define PCTL_STAT_LP_TRIG	0x70
#define PCTL_LP_TRIG_SR_IDLE	0x10

/* PCTL_SCTL commands */
#define PCTL_CFG_STATE		1
#define PCTL_GO_STATE		2
#define PCTL_SLEEP_STATE	3
#define PCTL_WAKEUP_STATE	4

/* PCTL_STAT states; 2, 4, 6 and 7 are transitions */
#define PCTL_INIT_MEM		0
#define PCTL_CONFIG		1
#define PCTL_ACCESS		3
#define PCTL_LOW_POWER		5

/*
 * GRF_SOC_CON2 bit 0: the vendor kernel sets it ("hw_wakeup: disable auto
 * sr") on the way to Config and clears it in Access; here it is set for
 * the transitions and given its old value back. Bit 16 is its write
 * enable.
 */
#define GRF_DDR_HW_WAKEUP	0x00010001

/*
 * Reads of PCTL_STAT per transition. A read is a bus access to the
 * controller, tens of nanoseconds at the fastest CPU clock, while a
 * transition takes microseconds (self-refresh entry and exit are a few
 * hundred DRAM clocks), so a transition that never ends gives up after a
 * few milliseconds.
 */
#define PCTL_POLLS		0x10000

	.arch	armv7-a
	.arm
	.text
	.align	3
ENTRY(rk3026_suspend_sram)
	/* adrl: the table may lie further than one adr immediate reaches */
	adrl	r2, rk3026_suspend_sram_table
	mov	r1, sp
	ldr	sp, [r2, #RK3026_SUSPEND_STACK_TOP * 4]
	push	{r1, r4-r6, lr}

	/*
	 * Load a TLB entry for every page the routine reaches, with one read
	 * at each base in the table: once the DRAM is in self-refresh, a
	 * page table walk would read DDR, and with the L2 cache off nothing
	 * holds the tables. From here on the routine reaches nothing but its
	 * own section and these bases, so no other access can evict one of
	 * the entries before the DRAM is back. The finisher cannot do this:
	 * its own loads from DDR come after the touches.
	 */
	mov	r1, #RK3026_SUSPEND_BASES
1:	subs	r1, r1, #1
	ldr	r0, [r2, r1, lsl #2]
	ldr	r0, [r0]
	bne	1b

	ldr	r4, [r2, #RK3026_SUSPEND_PCTL * 4]
	ldr	r5, [r2, #RK3026_SUSPEND_GRF * 4]

	/* what the boot loader set up, to be found the same way after */
	ldr	r0, [r4, #PCTL_MCFG]
	str	r0, [r2, #RK3026_SUSPEND_MCFG * 4]
	ldr	r0, [r4, #PCTL_MCFG1]
	str	r0, [r2, #RK3026_SUSPEND_MCFG1 * 4]
	ldr	r0, [r5, #RK3026_GRF_SOC_CON2]
	str	r0, [r2, #RK3026_SUSPEND_SOC_CON2 * 4]

	/*
	 * Only a controller in Access goes to sleep; nothing changed yet. As
	 * in the vendor code, Low_power that the controller entered on its
	 * own (MCFG1 sr_idle, lp_trig 1) counts as Access.
	 */
	ldr	r0, [r4, #PCTL_STAT]
	and	r1, r0, #PCTL_STAT_MASK
	cmp	r1, #PCTL_ACCESS
	beq	1f
	and	r0, r0, #PCTL_STAT_LP_TRIG
	cmp	r1, #PCTL_LOW_POWER
	cmpeq	r0, #PCTL_LP_TRIG_SR_IDLE
	movne	r0, #RK3026_SUSPEND_E_STATE
	bne	.Lreturn
1:
	/*
	 * Access -> Config -> Access -> Low_power: self-refresh. With
	 * GRF_SOC_CON2 bit 0 set, a self-refresh the controller entered on
	 * its own is ended first, as the vendor's move to Config does.
	 */
	movw	r0, #(GRF_DDR_HW_WAKEUP & 0xffff)
	movt	r0, #(GRF_DDR_HW_WAKEUP >> 16)
	str	r0, [r5, #RK3026_GRF_SOC_CON2]
	dsb
	mov	r3, #RK3026_SUSPEND_E_CONFIG
	bl	.Laccess
	cmp	r0, #0
	bne	.Lback_out
	mov	r0, #PCTL_CFG_STATE
	mov	r1, #PCTL_CONFIG
	mov	r3, #RK3026_SUSPEND_E_CONFIG
	bl	.Lmove
	cmp	r0, #0
	bne	.Lback_out
	mov	r0, #PCTL_GO_STATE
	mov	r1, #PCTL_ACCESS
	mov	r3, #RK3026_SUSPEND_E_ACCESS
	bl	.Lmove
	cmp	r0, #0
	bne	.Lback_out
	mov	r0, #PCTL_SLEEP_STATE
	mov	r1, #PCTL_LOW_POWER
	mov	r3, #RK3026_SUSPEND_E_SLEEP
	bl	.Lmove
	cmp	r0, #0
	bne	.Lback_out

	dsb
	wfi

	/*
	 * Low_power -> Access. If that fails the DRAM state is unknown and
	 * nothing in DDR may run any more.
	 */
	mov	r0, #PCTL_WAKEUP_STATE
	mov	r1, #PCTL_ACCESS
	mov	r3, #RK3026_SUSPEND_R_WAKEUP
	bl	.Lmove
	cmp	r0, #0
	bne	.Lreset
	mov	r6, #RK3026_SUSPEND_DONE

	/*
	 * In Access, with the code to return in r6. Nothing above writes
	 * MCFG or MCFG1. Should the controller have changed either, write the
	 * saved values back in Config, the only state that takes them.
	 */
.Lrestore:
	ldr	r0, [r4, #PCTL_MCFG]
	ldr	r1, [r2, #RK3026_SUSPEND_MCFG * 4]
	cmp	r0, r1
	ldreq	r0, [r4, #PCTL_MCFG1]
	ldreq	r1, [r2, #RK3026_SUSPEND_MCFG1 * 4]
	cmpeq	r0, r1
	beq	.Lrestored
	mov	r0, #PCTL_CFG_STATE
	mov	r1, #PCTL_CONFIG
	mov	r3, #RK3026_SUSPEND_R_CONFIG
	bl	.Lmove
	cmp	r0, #0
	bne	.Lreset
	ldr	r0, [r2, #RK3026_SUSPEND_MCFG * 4]
	str	r0, [r4, #PCTL_MCFG]
	ldr	r0, [r2, #RK3026_SUSPEND_MCFG1 * 4]
	str	r0, [r4, #PCTL_MCFG1]
	mov	r0, #PCTL_GO_STATE
	mov	r1, #PCTL_ACCESS
	mov	r3, #RK3026_SUSPEND_R_ACCESS
	bl	.Lmove
	cmp	r0, #0
	bne	.Lreset
.Lrestored:
	bl	.Lsoc_con2
	mov	r0, r6
.Lreturn:
	pop	{r1, r4-r6, lr}
	mov	sp, r1
	bx	lr

	/*
	 * An entry step failed with the code in r0: bring the controller
	 * back to Access from wherever it stopped, check MCFG and MCFG1 as
	 * after a wake-up, and return the code. A controller that does not
	 * get back to Access resets the SoC.
	 */
.Lback_out:
	mov	r6, r0
	mov	r3, #RK3026_SUSPEND_R_BACK_OUT
	bl	.Laccess
	cmp	r0, #0
	beq	.Lrestore
	b	.Lreset

	/*
	 * Bring the controller to Access from wherever it is: Low_power
	 * takes WAKEUP, Config takes GO, a transition is waited out. Returns
	 * r0 = 0, or the failure code r3 when it never gets there; Init_mem
	 * cannot be left without the boot loader's setup. Uses r0, r1, r12
	 * and the flags.
	 */
.Laccess:
	mov	r12, #PCTL_POLLS
1:	ldr	r0, [r4, #PCTL_STAT]
	and	r0, r0, #PCTL_STAT_MASK
	cmp	r0, #PCTL_ACCESS
	moveq	r0, #0
	bxeq	lr
	mov	r1, #PCTL_ACCESS
	cmp	r0, #PCTL_LOW_POWER
	moveq	r0, #PCTL_WAKEUP_STATE
	beq	.Lmove
	cmp	r0, #PCTL_CONFIG
	moveq	r0, #PCTL_GO_STATE
	beq	.Lmove
	cmp	r0, #PCTL_INIT_MEM
	beq	2f
	/* a transition: wait for its end */
	subs	r12, r12, #1
	bne	1b
2:	mov	r0, r3
	bx	lr

	/*
	 * Write the command r0 to SCTL and wait until STAT reads the state
	 * r1. Returns r0 = 0, or the failure code r3 when it never does.
	 * Uses r0, r12 and the flags.
	 */
.Lmove:
	str	r0, [r4, #PCTL_SCTL]
	dsb
	mov	r12, #PCTL_POLLS
1:	ldr	r0, [r4, #PCTL_STAT]
	and	r0, r0, #PCTL_STAT_MASK
	cmp	r0, r1
	beq	2f
	subs	r12, r12, #1
	bne	1b
	mov	r0, r3
	bx	lr
2:	mov	r0, #0
	bx	lr

	/* GRF_SOC_CON2 bit 0 back to its saved value; uses r0 */
.Lsoc_con2:
	ldr	r0, [r2, #RK3026_SUSPEND_SOC_CON2 * 4]
	and	r0, r0, #1
	orr	r0, r0, #(GRF_DDR_HW_WAKEUP & 0xffff0000)
	str	r0, [r5, #RK3026_GRF_SOC_CON2]
	dsb
	bx	lr

	/*
	 * The DRAM did not come back (the code in r0): reset the SoC from
	 * here, with no further access to DDR. The GRF and the GPIO banks,
	 * and so a GPIO power hold, keep their state through the core reset,
	 * so the system boots again.
	 */
.Lreset:
	ldr	r1, [r2, #RK3026_SUSPEND_CRU * 4]
	ldr	r4, [r2, #RK3026_SUSPEND_GRF * 4]
	rk3026_core_reset 0
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
