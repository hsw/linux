/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * RK3026 core reset, copied to the internal SRAM by rk3026-restart.c and
 * entered from cpu_reset() with the MMU and the caches off. Position
 * independent: the CRU base is a PC-relative literal that the C side
 * fills in after the copy. No stack and no DDR access: once the PLLs are
 * in slow mode the DDR controller runs from 24 MHz and cannot be used.
 */
#include <linux/linkage.h>

#define RK3026_CRU_MODE_CON		0x40
#define RK3026_CRU_SOFTRST_CON0		0x110

	.arch	armv7-a
	.arm
	.section .rodata
	.align	2
	.globl	rk3026_restart_sram
rk3026_restart_sram:
	ldr	r1, rk3026_restart_sram_cru
	/* every PLL to slow mode, as the BootROM expects after a cold start */
	movw	r2, #0x0000
	movt	r2, #0xffff
	str	r2, [r1, #RK3026_CRU_MODE_CON]
	dsb
	/* a few milliseconds at 24 MHz */
	mov	r3, #0x4000
1:	subs	r3, r3, #1
	bne	1b
	/*
	 * Assert the MCORE, CORE0, CORE1 and L2C resets. Everything else
	 * (GPIO banks, GRF, SRAM, DDR controller) keeps its state, and CPU0
	 * restarts at address 0 in the BootROM.
	 */
	movw	r2, #0xffff
	movt	r2, #0x801c
	str	r2, [r1, #RK3026_CRU_SOFTRST_CON0]
	dsb
2:	wfi
	b	2b

	.align	2
	.globl	rk3026_restart_sram_cru
rk3026_restart_sram_cru:
	.word	0
	.globl	rk3026_restart_sram_end
rk3026_restart_sram_end:
