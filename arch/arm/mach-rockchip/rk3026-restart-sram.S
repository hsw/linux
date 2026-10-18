/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * RK3026 core reset, copied to the internal SRAM by rk3026-restart.c and
 * entered from cpu_reset() with the MMU and the caches off. Position
 * independent: the CRU and GRF bases are PC-relative literals that the C
 * side fills in after the copy, and the tables below travel with the
 * code. No stack and no DDR access: once the PLLs are in slow mode the
 * DDR controller runs from 24 MHz and cannot be used.
 */
#include <linux/linkage.h>

#define RK3026_CRU_MODE_CON		0x40
#define RK3026_CRU_CLKSEL_CON(x)	(0x44 + (x) * 4)
#define RK3026_CRU_CLKGATE_CON(x)	(0xd0 + (x) * 4)
#define RK3026_CRU_SOFTRST_CON(x)	(0x110 + (x) * 4)

#define RK3026_GRF_SOC_CON2		0x148
#define RK3026_GRF_SOC_STATUS0		0x14c
#define RK3026_GRF_OS_REG6		0x1e0
#define RK3026_GRF_OS_REG7		0x1e4

/* idle requests of the PERI, GPU, VIDEO and VIO buses: SOC_CON2 bits 9-12 */
#define RK3026_IDLE_REQ			0x1e00
/* their acknowledges (bits 17-20) and idle states (bits 23-26) */
#define RK3026_IDLE_ACK_HI		0x079e
/*
 * Polls of SOC_STATUS0 per wait. Each poll is an APB read of the GRF with
 * pclk_cpu at 1-2 MHz and uncached fetches from the SRAM, several
 * microseconds, so a wait that never sees its acknowledge ends after
 * tens of milliseconds, with the DDR still refreshed from 24 MHz.
 */
#define RK3026_IDLE_POLLS		0x1000
/*
 * The polls left when each wait ended go to the low half of OS_REG6
 * (acknowledge) and OS_REG7 (release), 0 meaning it timed out, under this
 * tag in the high half. Nothing else uses these two scratch registers and
 * the GRF keeps them through the core reset, so the next boot can read
 * how the buses took the idle request.
 */
#define RK3026_IDLE_TRACE		0x1d1e
/* a few milliseconds at 24 MHz */
#define RK3026_DELAY			0x4000

	.arch	armv7-a
	.arm
	.section .rodata
	.align	2
	.globl	rk3026_restart_sram
rk3026_restart_sram:
	ldr	r1, rk3026_restart_sram_cru
	ldr	r4, rk3026_restart_sram_grf

	/* every PLL to slow mode, as the BootROM expects after a cold start */
	movw	r2, #0x0000
	movt	r2, #0xffff
	str	r2, [r1, #RK3026_CRU_MODE_CON]
	dsb
	mov	r3, #RK3026_DELAY
1:	subs	r3, r3, #1
	bne	1b

	/*
	 * The clock selects of the CPU and peripheral buses and of the SD,
	 * SDIO and eMMC clocks back to their reset values, then every clock
	 * gate open: what a cold start hands the BootROM, and the clocks the
	 * module resets below need to take effect.
	 */
	adr	r5, .Lwrites
	adr	r6, .Lwrites_end
2:	ldmia	r5!, {r2, r3}
	str	r3, [r1, r2]
	cmp	r5, r6
	bne	2b
	dsb

	/* let the masters' buses finish their transfers and refuse new ones */
	movw	r2, #RK3026_IDLE_REQ
	movt	r2, #RK3026_IDLE_REQ
	str	r2, [r4, #RK3026_GRF_SOC_CON2]
	dsb
	movw	r7, #0
	movt	r7, #RK3026_IDLE_ACK_HI
	movw	r8, #0
	movt	r8, #RK3026_IDLE_TRACE
	mov	r3, #RK3026_IDLE_POLLS
3:	ldr	r2, [r4, #RK3026_GRF_SOC_STATUS0]
	and	r2, r2, r7
	cmp	r2, r7
	beq	4f
	subs	r3, r3, #1
	bne	3b
	/* no acknowledge in time: reset the masters all the same */
4:	orr	r2, r3, r8
	str	r2, [r4, #RK3026_GRF_OS_REG6]
	adr	r5, .Lresets
	adr	r6, .Lresets_end
5:	ldmia	r5!, {r2, r3}
	orr	r3, r3, r3, lsl #16
	str	r3, [r1, r2]
	cmp	r5, r6
	bne	5b
	dsb
	mov	r3, #RK3026_DELAY
6:	subs	r3, r3, #1
	bne	6b

	/*
	 * Release them again: the BootROM and the loader do not reset these
	 * blocks themselves, the SD controller least of all.
	 */
	adr	r5, .Lresets
7:	ldmia	r5!, {r2, r3}
	lsl	r3, r3, #16
	str	r3, [r1, r2]
	cmp	r5, r6
	bne	7b
	dsb

	/* drop the idle requests and wait until the buses are back */
	movw	r2, #0
	movt	r2, #RK3026_IDLE_REQ
	str	r2, [r4, #RK3026_GRF_SOC_CON2]
	dsb
	mov	r3, #RK3026_IDLE_POLLS
8:	ldr	r2, [r4, #RK3026_GRF_SOC_STATUS0]
	tst	r2, r7
	beq	9f
	subs	r3, r3, #1
	bne	8b

	/*
	 * Assert the MCORE, CORE0, CORE1 and L2C resets. The GPIO banks, the
	 * GRF, the SRAM and the DDR controller keep their state, and CPU0
	 * restarts at address 0 in the BootROM.
	 */
9:	orr	r2, r3, r8
	str	r2, [r4, #RK3026_GRF_OS_REG7]
	movw	r2, #0xffff
	movt	r2, #0x801c
	str	r2, [r1, #RK3026_CRU_SOFTRST_CON(0)]
	dsb
10:	wfi
	b	10b

	.align	2
	/* { CRU offset, value } */
.Lwrites:
	.word	RK3026_CRU_CLKSEL_CON(0), 0xffff0200	/* core, aclk_cpu */
	.word	RK3026_CRU_CLKSEL_CON(1), 0xffff3113	/* hclk/pclk_cpu */
	.word	RK3026_CRU_CLKSEL_CON(10), 0xffff2100	/* peripheral bus */
	.word	RK3026_CRU_CLKSEL_CON(11), 0x007f0017	/* sclk_sdmmc */
	.word	RK3026_CRU_CLKSEL_CON(12), 0xffff1717	/* sclk_sdio, sclk_emmc */
	.word	RK3026_CRU_CLKGATE_CON(0), 0xffff0000
	.word	RK3026_CRU_CLKGATE_CON(1), 0xffff0000
	.word	RK3026_CRU_CLKGATE_CON(2), 0xffff0000
	.word	RK3026_CRU_CLKGATE_CON(3), 0xffff0000
	.word	RK3026_CRU_CLKGATE_CON(4), 0xffff0000
	.word	RK3026_CRU_CLKGATE_CON(5), 0xffff0000
	.word	RK3026_CRU_CLKGATE_CON(6), 0xffff0000
	.word	RK3026_CRU_CLKGATE_CON(7), 0xffff0000
	.word	RK3026_CRU_CLKGATE_CON(8), 0xffff0000
	.word	RK3026_CRU_CLKGATE_CON(9), 0xffff0000
.Lwrites_end:

	/*
	 * { CRU offset, reset bits }: the blocks that master the AXI or AHB
	 * bus, and so may still be transferring, or be stuck halfway, when
	 * the cores go down. Never the GPIO banks (a board's PMIC power hold
	 * may be a GPIO), the GRF, the internal SRAM, the DDR controller and
	 * PHY (DDR keeps its content), nor the bus fabric.
	 */
.Lresets:
	.word	RK3026_CRU_SOFTRST_CON(3), 0x0020	/* CRYPTO */
	.word	RK3026_CRU_SOFTRST_CON(4), 0x05b1	/* DMA2 NANDC USBOTG0 OTGC0 USBOTG1 OTGC1 */
	.word	RK3026_CRU_SOFTRST_CON(5), 0x000e	/* MMC0 SDIO EMMC */
	.word	RK3026_CRU_SOFTRST_CON(6), 0x7c70	/* LCDC0, IEP, RGA, CIF0 */
	.word	RK3026_CRU_SOFTRST_CON(7), 0x19e3	/* VCODEC, LCDC1, GPU, EBC */
.Lresets_end:

	.globl	rk3026_restart_sram_cru
rk3026_restart_sram_cru:
	.word	0
	.globl	rk3026_restart_sram_grf
rk3026_restart_sram_grf:
	.word	0
	.globl	rk3026_restart_sram_end
rk3026_restart_sram_end:
