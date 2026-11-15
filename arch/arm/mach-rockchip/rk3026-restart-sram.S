/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * RK3026 core reset, copied to the internal SRAM by rk3026-restart.c and
 * entered from cpu_reset() with the MMU and the caches off. Position
 * independent: the CRU and GRF bases are PC-relative literals that the C
 * side fills in after the copy, and the core reset's tables travel with the
 * code. No stack and no DDR access: once the PLLs are in slow mode the
 * DDR controller runs from 24 MHz and cannot be used.
 */
#include <linux/linkage.h>
#include "rk3026-core-reset.S"

	.arch	armv7-a
	.arm
	.section .rodata
	.align	2
	.globl	rk3026_restart_sram
rk3026_restart_sram:
	ldr	r1, rk3026_restart_sram_cru
	ldr	r4, rk3026_restart_sram_grf

	rk3026_core_reset 1

	.globl	rk3026_restart_sram_cru
rk3026_restart_sram_cru:
	.word	0
	.globl	rk3026_restart_sram_grf
rk3026_restart_sram_grf:
	.word	0
	.globl	rk3026_restart_sram_end
rk3026_restart_sram_end:
