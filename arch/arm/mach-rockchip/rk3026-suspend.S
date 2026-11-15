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
 * self-refresh, puts the DDR PHY into reset and stops its clock and the
 * DDR PLL, waits for a wake-up interrupt and brings it all back, with a
 * new data training of the PHY. Position independent: it reaches its
 * table PC-relative.
 *
 * The DDR controller and PHY sequences follow Rockchip's RK2928/RK3026
 * vendor kernel (arch/arm/mach-rk2928/ddr.c, GPL-2.0, Copyright (C) 2012
 * Fuzhou Rockchip Electronics Co., Ltd), with every poll bounded and the
 * controller's and the PHY's configuration given back exactly as they
 * were found.
 *
 * Returns a step code: RK3026_SUSPEND_DONE after a normal wake-up, or the
 * RK3026_SUSPEND_E_* code of the entry step that failed, after the DRAM
 * was brought back. Once the DRAM may be unusable, from the PHY reset on
 * or after a wake-up that does not come back to Access, it never returns
 * into DDR: it resets the SoC from the SRAM.
 */
#include <linux/linkage.h>
#include "rk3026-core-reset.S"
#include "rk3026-suspend.h"

#define PCTL_SCTL		0x04
#define PCTL_STAT		0x08
#define PCTL_MCMD		0x40
#define PCTL_MCFG1		0x7c
#define PCTL_MCFG		0x80
#define PCTL_TREFI		0xd0
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

/* PCTL_MCMD: an auto refresh to the ranks in bits 23:20 */
#define PCTL_MCMD_START		0x80000000
#define PCTL_MCMD_RANK0		0x00100000
#define PCTL_MCMD_RANK1		0x00200000
#define PCTL_MCMD_REFRESH	0x2

/* DDR PHY */
#define PHY_REG1		0x000
#define PHY_REG1_SOFT_RESET_N	0x0c	/* both held low: the PHY in reset */
#define PHY_REG2		0x008
#define PHY_REG2_CAL_MASK	0x3	/* bit 1 clear: normal mode */
#define PHY_REG2_CAL_START	0x1	/* DQS squelch calibration */
#define PHY_REG62		0x3e8
#define PHY_REG62_CAL_DONE	0x3	/* both byte lanes calibrated */
#define PHY_REG264		0x264
#define PHY_REG264_ON		0x2	/* cleared with the soft reset */

/* CRU: the DDR PLL (DPLL) and the PHY's clock */
#define CRU_DPLL_CON1		RK3026_CRU_PLL_CON(5)
#define CRU_PLL_LOCK		0x0400
#define CRU_PLL_PWRDOWN		0x2000
#define CRU_DPLL_MODE_NORMAL	0x0010	/* MODE_CON; clear: slow (24 MHz) */
#define CRU_DDR_SRC_GPLL	0x0100	/* CLKSEL_CON26: the DDR runs from GPLL */
#define CRU_DDRPHY_GATE		0x0004	/* CLKGATE_CON0: dpll_ddr */

/* GRF_OS_REG1, the boot loader's DRAM description: bit 11 set, two ranks */
#define RK3026_GRF_OS_REG1	0x1cc
#define GRF_DDR_TWO_RANKS	0x0800

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

/*
 * The waits count loop iterations: one takes at least a cycle, so
 * DELAY_1US iterations last at least 1 us at the fastest CPU clock
 * (1008 MHz) and stay bounded at the slowest. Fetched uncached from the
 * SRAM, an iteration takes many cycles, so a wait lasts longer than it
 * says: fine with the DRAM refreshed, not while the training has the
 * auto refresh off. There the routine polls without waits where it can
 * (TRAIN_START_US only when the done bits still read set from the last
 * training) and sends the most refreshes DDR3 lets a controller postpone.
 * The DPLL gets as long to lock as the vendor code gives it.
 */
#define DELAY_1US		1024
#define DPLL_LOCK_US		1000
#define PHY_SETTLE_US		10
#define TRAIN_START_US		6
#define TRAIN_POLLS		0x10000
#define REFRESHES		8

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

	/* debug layer: reset after this exit step when fail_at names it */
	.macro	fail_step, code
	ldr	r0, [r2, #RK3026_SUSPEND_FAIL_AT * 4]
	cmp	r0, #\code
	moveq	r0, #\code
	beq	.Lreset
	.endm

	/* debug layer: r0 | the tag to GRF_OS_REG6; uses r3 */
	.macro	crumb_r0, flag
	ldr	r3, [r2, #RK3026_SUSPEND_GRF * 4]
	orr	r0, r0, #\flag
	orr	r0, r0, #(RK3026_PM_CRUMB_SRAM & 0xff000000)
	orr	r0, r0, #(RK3026_PM_CRUMB_SRAM & 0x00ff0000)
	str	r0, [r3, #RK3026_GRF_OS_REG6]
	dsb
	.endm

	.arch	armv7-a
	.arm
	.text
	.align	3
ENTRY(rk3026_suspend_sram)
	/* adrl: the table may lie further than one adr immediate reaches */
	adrl	r2, rk3026_suspend_sram_table
	mov	r1, sp
	ldr	sp, [r2, #RK3026_SUSPEND_STACK_TOP * 4]
	push	{r1, r4-r10, lr}

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
	ldr	r8, [r2, #RK3026_SUSPEND_PHY * 4]
	ldr	r9, [r2, #RK3026_SUSPEND_CRU * 4]

	/* what the boot loader set up, to be found the same way after */
	ldr	r0, [r4, #PCTL_MCFG]
	str	r0, [r2, #RK3026_SUSPEND_MCFG * 4]
	ldr	r0, [r4, #PCTL_MCFG1]
	str	r0, [r2, #RK3026_SUSPEND_MCFG1 * 4]
	ldr	r0, [r5, #RK3026_GRF_SOC_CON2]
	str	r0, [r2, #RK3026_SUSPEND_SOC_CON2 * 4]
	ldr	r0, [r8, #PHY_REG1]
	str	r0, [r2, #RK3026_SUSPEND_PHY_REG1 * 4]
	ldr	r0, [r8, #PHY_REG264]
	str	r0, [r2, #RK3026_SUSPEND_PHY_REG264 * 4]

	/*
	 * r10 = 1: the DDR clock comes from the DPLL in normal mode, as the
	 * boot loaders set it up, so the PHY and the DPLL can stop in the
	 * sleep. Otherwise the DRAM only goes into self-refresh.
	 */
	mov	r10, #0
	ldr	r0, [r9, #RK3026_CRU_CLKSEL_CON(26)]
	tst	r0, #CRU_DDR_SRC_GPLL
	bne	1f
	ldr	r0, [r9, #RK3026_CRU_MODE_CON]
	tst	r0, #CRU_DPLL_MODE_NORMAL
	movne	r10, #1
1:
	crumb	RK3026_PM_SRAM_ENTRY
	fail_at	RK3026_PM_SRAM_ENTRY

	/*
	 * debug: depth 1 leaves the PHY and the DPLL running, depth 0 also
	 * the DRAM active
	 */
	ldr	r0, [r2, #RK3026_SUSPEND_DEPTH * 4]
	cmp	r0, #2
	movlo	r10, #0
	cmp	r0, #0
	beq	.Lsleep

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
	crumb	RK3026_PM_SRAM_SR

.Lsleep:
	/* debug guard: restart the watchdog with its sleep timeout */
	ldr	r0, [r2, #RK3026_SUSPEND_WDT_TORR * 4]
	cmp	r0, #0
	ldrne	r3, [r2, #RK3026_SUSPEND_WDT * 4]
	strne	r0, [r3, #DW_WDT_TORR]
	movne	r0, #DW_WDT_CRR_KICK
	strne	r0, [r3, #DW_WDT_CRR]
	crumb	RK3026_PM_SRAM_WFI
	fail_at	RK3026_PM_SRAM_WFI

	cmp	r10, #0
	beq	.Lwfi
	/*
	 * The point of no return: from the PHY reset on, the DRAM cannot be
	 * used before the PHY is trained again, so every failure after this
	 * resets the SoC. The vendor order: the PHY into reset, its clock
	 * gated, the DPLL to slow mode and powered down.
	 */
	ldr	r0, [r8, #PHY_REG264]
	bic	r0, r0, #PHY_REG264_ON
	str	r0, [r8, #PHY_REG264]
	ldr	r0, [r8, #PHY_REG1]
	bic	r0, r0, #PHY_REG1_SOFT_RESET_N
	str	r0, [r8, #PHY_REG1]
	dsb
	movw	r0, #CRU_DDRPHY_GATE
	movt	r0, #CRU_DDRPHY_GATE
	str	r0, [r9, #RK3026_CRU_CLKGATE_CON(0)]
	dsb
	mov	r0, #1
	bl	.Ldelay
	mov	r0, #(CRU_DPLL_MODE_NORMAL << 16)
	str	r0, [r9, #RK3026_CRU_MODE_CON]
	dsb
	mov	r0, #1
	bl	.Ldelay
	movw	r0, #CRU_PLL_PWRDOWN
	movt	r0, #CRU_PLL_PWRDOWN
	str	r0, [r9, #CRU_DPLL_CON1]
	dsb
	mov	r0, #1
	bl	.Ldelay
	crumb	RK3026_PM_SRAM_PHY_OFF

.Lwfi:
	dsb
	wfi

	/*
	 * debug: the point, with the interrupt that ended the wfi in bits
	 * 15:8 (the low byte of GICC_HPPIR's ID; 0xff when none is pending);
	 * r7 keeps those bits for the points after this one
	 */
	ldr	r3, [r2, #RK3026_SUSPEND_GICC * 4]
	ldr	r1, [r3, #RK3026_GICC_HPPIR]
	and	r1, r1, #0xff
	lsl	r7, r1, #8
	mov	r0, r7
	crumb_r0 RK3026_PM_SRAM_WOKE
	ldr	r0, [r2, #RK3026_SUSPEND_DEPTH * 4]
	cmp	r0, #0
	moveq	r0, #RK3026_SUSPEND_DONE
	beq	.Lreturn

	cmp	r10, #0
	beq	.Lwakeup
	/* the DPLL powered up, locked (bounded as in the vendor code), normal */
	mov	r0, #(CRU_PLL_PWRDOWN << 16)
	str	r0, [r9, #CRU_DPLL_CON1]
	dsb
	mov	r6, #DPLL_LOCK_US
1:	mov	r0, #1
	bl	.Ldelay
	ldr	r0, [r9, #CRU_DPLL_CON1]
	tst	r0, #CRU_PLL_LOCK
	bne	2f
	subs	r6, r6, #1
	bne	1b
	mov	r0, #RK3026_SUSPEND_R_DPLL
	b	.Lreset
2:	fail_step RK3026_SUSPEND_R_DPLL
	movw	r0, #CRU_DPLL_MODE_NORMAL
	movt	r0, #CRU_DPLL_MODE_NORMAL
	str	r0, [r9, #RK3026_CRU_MODE_CON]
	dsb
	/* the PHY's clock on, then the PHY out of reset as it was */
	mov	r0, #(CRU_DDRPHY_GATE << 16)
	str	r0, [r9, #RK3026_CRU_CLKGATE_CON(0)]
	dsb
	mov	r0, #1
	bl	.Ldelay
	ldr	r0, [r2, #RK3026_SUSPEND_PHY_REG1 * 4]
	str	r0, [r8, #PHY_REG1]
	ldr	r0, [r2, #RK3026_SUSPEND_PHY_REG264 * 4]
	str	r0, [r8, #PHY_REG264]
	dsb
	mov	r0, #PHY_SETTLE_US
	bl	.Ldelay
	orr	r0, r7, #RK3026_PM_SRAM_PHY_ON
	crumb_r0 0

	/*
	 * Low_power -> Access. If that fails the DRAM state is unknown and
	 * nothing in DDR may run any more.
	 */
.Lwakeup:
	mov	r0, #PCTL_WAKEUP_STATE
	mov	r1, #PCTL_ACCESS
	mov	r3, #RK3026_SUSPEND_R_WAKEUP
	bl	.Lmove
	cmp	r0, #0
	bne	.Lreset
	cmp	r10, #0
	beq	.Ldone

	/*
	 * The PHY came out of reset untrained: Access -> Config, the data
	 * training (DQS squelch calibration) with the auto refresh off,
	 * the refreshes it held back, Config -> Access.
	 */
	mov	r0, #PCTL_CFG_STATE
	mov	r1, #PCTL_CONFIG
	mov	r3, #RK3026_SUSPEND_R_TRAIN_CFG
	bl	.Lmove
	cmp	r0, #0
	bne	.Lreset
	ldr	r6, [r4, #PCTL_TREFI]
	mov	r0, #0
	str	r0, [r4, #PCTL_TREFI]
	/* debug: the global timer at the auto refresh's stop */
	ldr	r3, [r2, #RK3026_SUSPEND_GICC * 4]
	ldr	r0, [r3, #RK3026_GICC_GT_COUNT]
	str	r0, [r2, #RK3026_SUSPEND_GT_REFOFF * 4]
	/*
	 * Done bits that read set before the start may be the last
	 * training's until the calibration clears them: then wait as the
	 * vendor does before the first poll. Clear ones can only be set by
	 * this calibration, so poll at once.
	 */
	ldr	r1, [r8, #PHY_REG62]
	str	r1, [r2, #RK3026_SUSPEND_REG62_PRE * 4]
	ldr	r0, [r8, #PHY_REG2]
	bic	r0, r0, #PHY_REG2_CAL_MASK
	orr	r0, r0, #PHY_REG2_CAL_START
	str	r0, [r8, #PHY_REG2]
	dsb
	tst	r1, #PHY_REG62_CAL_DONE
	movne	r0, #TRAIN_START_US
	blne	.Ldelay
	mov	r1, #TRAIN_POLLS
1:	ldr	r0, [r8, #PHY_REG62]
	and	r0, r0, #PHY_REG62_CAL_DONE
	cmp	r0, #PHY_REG62_CAL_DONE
	beq	2f
	subs	r1, r1, #1
	bne	1b
	mov	r0, #RK3026_SUSPEND_R_TRAIN
	b	.Lreset
2:	fail_step RK3026_SUSPEND_R_TRAIN
	ldr	r0, [r8, #PHY_REG2]
	bic	r0, r0, #PHY_REG2_CAL_MASK
	str	r0, [r8, #PHY_REG2]
	dsb
	/* to the first rank, or to both when the boot loader says two */
	movw	r3, #PCTL_MCMD_REFRESH
	movt	r3, #((PCTL_MCMD_START | PCTL_MCMD_RANK0) >> 16)
	ldr	r0, [r5, #RK3026_GRF_OS_REG1]
	tst	r0, #GRF_DDR_TWO_RANKS
	orrne	r3, r3, #PCTL_MCMD_RANK1
	mov	r1, #REFRESHES
3:	str	r3, [r4, #PCTL_MCMD]
	dsb
	mov	r12, #PCTL_POLLS
4:	ldr	r0, [r4, #PCTL_MCMD]
	tst	r0, #PCTL_MCMD_START
	beq	5f
	subs	r12, r12, #1
	bne	4b
	mov	r0, #RK3026_SUSPEND_R_REFRESH
	b	.Lreset
5:	subs	r1, r1, #1
	bne	3b
	fail_step RK3026_SUSPEND_R_REFRESH
	str	r6, [r4, #PCTL_TREFI]
	/*
	 * debug: the global timer ticks the auto refresh was off, and of one
	 * .Ldelay(1)
	 */
	ldr	r3, [r2, #RK3026_SUSPEND_GICC * 4]
	ldr	r0, [r3, #RK3026_GICC_GT_COUNT]
	ldr	r1, [r2, #RK3026_SUSPEND_GT_REFOFF * 4]
	sub	r0, r0, r1
	str	r0, [r2, #RK3026_SUSPEND_GT_REFOFF * 4]
	ldr	r1, [r3, #RK3026_GICC_GT_COUNT]
	mov	r0, #1
	bl	.Ldelay
	ldr	r0, [r3, #RK3026_GICC_GT_COUNT]
	sub	r0, r0, r1
	str	r0, [r2, #RK3026_SUSPEND_GT_DELAY * 4]
	orr	r0, r7, #RK3026_PM_SRAM_TRAINED
	crumb_r0 0
	mov	r0, #PCTL_GO_STATE
	mov	r1, #PCTL_ACCESS
	mov	r3, #RK3026_SUSPEND_R_TRAIN_GO
	bl	.Lmove
	cmp	r0, #0
	bne	.Lreset
.Ldone:
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
	/*
	 * debug: the point with the interrupt after a wake-up, the code with
	 * the back-out flag after a back-out, to OS_REG6
	 */
	cmp	r6, #RK3026_SUSPEND_DONE
	orreq	r0, r7, #RK3026_PM_SRAM_BACK
	orrne	r0, r6, #RK3026_SUSPEND_ABORT
	crumb_r0 0
	mov	r0, r6
.Lreturn:
	pop	{r1, r4-r10, lr}
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
	/* debug: fail_at names this step: fail as if STAT never got there */
2:	ldr	r0, [r2, #RK3026_SUSPEND_FAIL_AT * 4]
	cmp	r0, r3
	moveq	r0, r3
	bxeq	lr
	mov	r0, #0
	bx	lr

	/* wait at least r0 microseconds; uses r0, r12 and the flags */
.Ldelay:
1:	mov	r12, #DELAY_1US
2:	subs	r12, r12, #1
	bne	2b
	subs	r0, r0, #1
	bne	1b
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
	 * so the system boots again; GRF_SOC_CON2 bit 0 goes back to what the
	 * boot loader left there first.
	 */
.Lreset:
	/* debug: the code with the reset flag to OS_REG6 */
	and	r0, r0, #0xff
	crumb_r0 RK3026_PM_SRAM_RESET
	bl	.Lsoc_con2
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
