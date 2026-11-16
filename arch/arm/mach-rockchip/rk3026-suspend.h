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
#define RK3026_SUSPEND_PHY		4	/* the DDR PHY */
#define RK3026_SUSPEND_BASES		5
#define RK3026_SUSPEND_STACK_TOP	5	/* the section's end */
/* the routine saves these here and leaves them as it found them */
#define RK3026_SUSPEND_MCFG		6
#define RK3026_SUSPEND_MCFG1		7
#define RK3026_SUSPEND_SOC_CON2		8
#define RK3026_SUSPEND_PHY_REG1		9
#define RK3026_SUSPEND_PHY_REG264	10
/* the CRU's clock setup the routine changes while it sleeps */
#define RK3026_SUSPEND_MODE_CON		11
#define RK3026_SUSPEND_CLKSEL0		12
#define RK3026_SUSPEND_CLKSEL1		13
#define RK3026_SUSPEND_CLKSEL10		14
/* the PLLs it put into slow mode and powered down: BIT(PLL index) */
#define RK3026_SUSPEND_PLLS		15
/* CLKGATE_CON0 before the routine closed the GPLL inputs of the CPU's muxes */
#define RK3026_SUSPEND_GATE0		16
#define RK3026_SUSPEND_TABLE_WORDS	17

/* the stack, at the top of the section */
#define RK3026_SUSPEND_STACK_SIZE	256

/*
 * The clock gates the sleep leaves as it finds them, by CLKGATE_CON
 * register (bit n: gate n; the names are clk-rk3026.c's). pm.c closes
 * every other gate of CLKGATE_CON0..9 before the routine runs and writes
 * all ten back as they were once it returns. Held: what the CPU, the
 * routine and the DRAM run on, and what wakes the system:
 * 0: core_peri, gpll_cpu, dpll_ddr, aclk_cpu, hclk_cpu, pclk_cpu,
 *    gpll_armclk, aclk_core_pre, gpll_ddr; the routine closes the three
 *    GPLL inputs itself once the CPU and the DDR run from other PLLs;
 * 1: sclk_otgphy0/1, and the DDR, GPU and core DLL gates (bits 7, 14,
 *    15, which the clock driver does not describe);
 * 2: aclk_peri_src, aclk_peri, hclk_peri, pclk_peri: the GPIO banks'
 *    clock and the USB controllers' bus;
 * 4: the peripheral AXI matrix and the CPU's bridge to it (bits 0-3, not
 *    in the clock driver), aclk_strc_sys, aclk_intmem (the SRAM);
 * 5: pclk_grf, pclk_ddrupctl, hclk_otg;
 * 7: hclk_host, pclk_pwm (a PWM may set a supply voltage);
 * 8: pclk_gpio0-3;
 * 9: clk_l2c, hclk_usb_peri, hclk_peri_arbi, aclk_peri_niu.
 */
#define RK3026_SUSPEND_GATE_REGS	10
#define RK3026_SUSPEND_HOLD0		0x01ff
#define RK3026_SUSPEND_HOLD1		0xc0e0
#define RK3026_SUSPEND_HOLD2		0x000f
#define RK3026_SUSPEND_HOLD3		0x0000
#define RK3026_SUSPEND_HOLD4		0x140f
#define RK3026_SUSPEND_HOLD5		0x2090
#define RK3026_SUSPEND_HOLD6		0x0000
#define RK3026_SUSPEND_HOLD7		0x0408
#define RK3026_SUSPEND_HOLD8		0x1e00
#define RK3026_SUSPEND_HOLD9		0xe010

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
#define RK3026_SUSPEND_R_DPLL		0x25	/* the DDR PLL did not lock */
#define RK3026_SUSPEND_R_TRAIN_CFG	0x26	/* no Config for the training */
#define RK3026_SUSPEND_R_TRAIN		0x27	/* the training did not end */
#define RK3026_SUSPEND_R_REFRESH	0x28	/* a refresh was not taken */
#define RK3026_SUSPEND_R_TRAIN_GO	0x29	/* no Access after the training */
#define RK3026_SUSPEND_R_APLL		0x2a	/* a PLL did not lock: 0x2a + */
#define RK3026_SUSPEND_R_CPLL		0x2c	/* its index (APLL 0, DPLL 1, */
#define RK3026_SUSPEND_R_GPLL		0x2d	/* CPLL 2, GPLL 3) */

#ifndef __ASSEMBLY__
u32 rk3026_suspend_sram(void);
extern const u32 rk3026_suspend_sram_sz;
extern const u32 rk3026_suspend_sram_table_off;
#endif

#endif /* __MACH_ROCKCHIP_RK3026_SUSPEND_H */
