// SPDX-License-Identifier: GPL-2.0-only
/*
 * RK3026 suspend to RAM: debug layer for book tests. Not for upstream.
 *
 * debugfs rk3026_pm/:
 *   enable   write 1: register the suspend ops (mem_sleep offers "deep"
 *            only after this; at boot it stays "[s2idle]"). Registering
 *            makes "deep" the current mem_sleep at once (no
 *            mem_sleep_default= on the cmdline), so writing enable is the
 *            opt-in. It also keeps the consoles running through the sleep,
 *            so the ramoops console records the suspend path
 *            (no_console_suspend). One way.
 *   depth    how far the sleep goes: 0 = wfi from SRAM with the DRAM
 *            active, 1 = the DRAM in self-refresh (PCTL Low_power), 2 =
 *            also the DDR PHY in reset with its clock gated and the DPLL
 *            slow and powered down, a new data training on the way back,
 *            3 = also APLL, CPLL and GPLL in slow mode and powered down,
 *            the CPU and its buses on the 24 MHz oscillator, 4 = also the
 *            clock gates closed (pm.c's first stage before the routine,
 *            then the GPLL inputs of the CPU's muxes) and the core at /32
 *            (the full routine); below 4 pm.c closes no gate
 *   fail_at  0 = never; 1 or 2: back out of the SRAM routine at that point
 *            (1 = on the SRAM stack, 2 = in self-refresh before the PHY
 *            reset; at depth >= 1 the DRAM is woken first), so .enter
 *            returns -EIO; a step code (rk3026-suspend.h
 *            RK3026_SUSPEND_E_* or _R_*, e.g. 0x14 or 0x21): that step
 *            fails once it succeeded, as if its poll never ended, so an
 *            E_* step backs out (-EIO) and an R_* step resets the SoC from
 *            the SRAM, leaving 0x503400(0x40 | code) in GRF_OS_REG6; the
 *            PCTL steps need depth 1, the PHY/DPLL steps (0x25-0x29) depth
 *            2, the PLL locks (0x2a APLL, 0x2c CPLL, 0x2d GPLL) depth 3;
 *            0x2e (RK3026_PM_FAIL_GATED, depth 4) resets the SoC right
 *            after the wfi with the gates closed and the core at /32
 *            (refused at a lower depth, and a lower depth is refused
 *            while one is set)
 *   pd       the PLLs depth 3 powers down after their slow mode, BIT(n)
 *            for APLL 0, CPLL 2, GPLL 3 (default 0xd: all three); a PLL
 *            left out only goes to slow mode
 *   guard    1 = keep the DesignWare watchdog counting through the sleep:
 *            pclk_wdt opened in .enter, the counter restarted with TOP 15
 *            (2^31 cycles, 44.7 s at the 48 MHz pclk_peri) right before wfi
 *            and again once the L2 is back on, the gate closed again; at
 *            depth 3 the routine restarts it with TOP 10 once GPLL is in
 *            slow mode (2^26 cycles, 44.7 s at the 1.5 MHz pclk_peri), and
 *            with TOP 15 once GPLL is back in its mode; the watchdog must
 *            be running already (a feeder has /dev/watchdog open); at
 *            depth 4 pm.c's first stage keeps pclk_wdt open
 *   mark     write anything: test markers into GRF_OS_REG6/7, the top two
 *            words of the suspend SRAM section and the kernel log (those
 *            two words are the routine's stack: a sleep overwrites them)
 *   crumbs   read: OS_REG6/7 and the SRAM words now and at boot, the
 *            knobs, the cycle counters, the last wake report and the
 *            report of the last wake that reached no wake-up source
 *            (after which .suspend_again slept again); PCTL MCFG/MCFG1,
 *            GRF_SOC_CON2 and PHY REG1/REG264 now, at boot and as the
 *            routine last saved them; PCTL TREFI, PHY REG2, the DPLL's
 *            CON0/CON1 and CLKSEL_CON26 now and at boot; from the last
 *            depth-2 sleep: how long the training kept the auto refresh
 *            off and how long one of the routine's 1 us waits took, in
 *            global timer ticks and in ns (the timer's rate tracked by a
 *            clock notifier), and PHY REG62 before the calibration start;
 *            the PLLs the last sleep parked, the CRU's PLL and clock
 *            select words and CLKGATE_CON0..9 before and after the last
 *            sleep, and how many sleeps changed any of them (pm.c and the
 *            routine must give them back), CLKGATE_CON0 as the routine
 *            saved it; CPU0's rate (policy, hardware), the policy's max
 *            and the CPU supply's voltage at the PM notifier's suspend
 *            and resume steps and in .prepare (after cpufreq_suspend(),
 *            deep only), each also logged
 *
 * Breadcrumbs: the SRAM routine writes its point to GRF_OS_REG6
 * (RK3026_PM_CRUMB_SRAM | point; 0x80 | code when it backed out, 0x40 |
 * code right before a core reset), .enter its
 * own point and the routine's return code to GRF_OS_REG7
 * (RK3026_PM_CRUMB_C | point << 8 | code). The GRF keeps both through a
 * core reset; the boot values are read here at init and printed. After
 * the wfi the routine adds the interrupt that ended it (GICC_HPPIR) to
 * OS_REG6, and the wake report holds the GIC's highest pending
 * interrupt, its pending, active and enable bits and the GPIO banks'
 * interrupt status, read before interrupts are back on.
 */

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <linux/suspend.h>

#include "rk3026-suspend.h"

#define RK3026_GRF_OS_REG7		0x1e4
#define RK3026_GRF_SOC_CON2		0x148
#define RK3026_PCTL_MCFG1		0x7c
#define RK3026_PCTL_MCFG		0x80
#define RK3026_PCTL_TREFI		0xd0
#define RK3026_PHY_REG1			0x000
#define RK3026_PHY_REG2			0x008
#define RK3026_PHY_REG264		0x264
#define RK3026_CRU_DPLL_CON0		0x10
#define RK3026_CRU_DPLL_CON1		0x14
#define RK3026_CRU_CLKSEL_CON26		0xac
/* the global timer's control register (bit 0: the counter runs), GICC page */
#define RK3026_GICC_GT_CONTROL		(RK3026_GICC_GT_COUNT + 0x8)

/* Cortex-A9 main TLB: 2-way; 32 sets in its smallest configuration */
#define RK3026_TLB_SETS			32

/* pclk_wdt: CLKGATE_CON7 bit 15 (clk-rk3026.c) */
#define RK3026_CRU_CLKGATE_CON7		0xec
#define RK3026_CRU_PCLK_WDT		BIT(15)

#define DW_WDT_CR			0x00
#define DW_WDT_CR_EN			BIT(0)
/* TOP 15 and TOP_INIT 15 */
#define DW_WDT_TORR_44S			0xff
/* TOP 10 and TOP_INIT 15: 44.7 s at 1/32 of the rate */
#define DW_WDT_TORR_44S_SLOW		0xfa

/* the PLLs depth 3 parks: APLL, CPLL, GPLL */
#define RK3026_PM_PLLS			(BIT(0) | BIT(2) | BIT(3))

#define GIC_DIST_ENABLE_SET		0x100
#define GIC_DIST_PENDING_SET		0x200
#define GIC_DIST_ACTIVE_SET		0x300
#define RK3026_GIC_PENDING_WORDS	4
#define RK_GPIO_INT_STATUS		0x40
#define RK_GPIO_INT_RAWSTATUS		0x44
#define RK3026_GPIO_BANKS		4

#define RK3026_PM_MARK_OS_REG6		0xa0a00006
#define RK3026_PM_MARK_OS_REG7		0xa0a00007
#define RK3026_PM_MARK_SRAM0		0xa0a05e00
#define RK3026_PM_MARK_SRAM1		0xa0a05e01

static u32 *rk3026_pm_table;
static void __iomem *rk3026_pm_grf;
static void __iomem *rk3026_pm_cru;
static void __iomem *rk3026_pm_wdt;
static void __iomem *rk3026_pm_gic;
static void __iomem *rk3026_pm_gicc;
static void __iomem *rk3026_pm_gpio[RK3026_GPIO_BANKS];
static const struct platform_suspend_ops *rk3026_pm_ops;
static bool rk3026_pm_enabled;

static u32 rk3026_pm_depth;
static u32 rk3026_pm_fail_at;
static u32 rk3026_pm_guard;
static u32 rk3026_pm_pd = RK3026_PM_PLLS;

static u32 rk3026_pm_boot_reg6, rk3026_pm_boot_reg7;
static u32 rk3026_pm_boot_sram0, rk3026_pm_boot_sram1;
static u32 rk3026_pm_boot_mcfg, rk3026_pm_boot_mcfg1, rk3026_pm_boot_soc_con2;
static void __iomem *rk3026_pm_pctl;
static void __iomem *rk3026_pm_phy;

/* the DDR registers the crumbs show now and at boot */
struct rk3026_pm_ddr_reg {
	const char *name;
	int base;	/* the table index */
	u32 off;
	int saved;	/* the table index of the routine's copy, or -1 */
	u32 boot;
};

static struct rk3026_pm_ddr_reg rk3026_pm_ddr_regs[] = {
	{ "phy_reg1", RK3026_SUSPEND_PHY, RK3026_PHY_REG1,
	  RK3026_SUSPEND_PHY_REG1 },
	{ "phy_reg264", RK3026_SUSPEND_PHY, RK3026_PHY_REG264,
	  RK3026_SUSPEND_PHY_REG264 },
	{ "phy_reg2", RK3026_SUSPEND_PHY, RK3026_PHY_REG2, -1 },
	{ "trefi", RK3026_SUSPEND_PCTL, RK3026_PCTL_TREFI, -1 },
	{ "dpll_con0", RK3026_SUSPEND_CRU, RK3026_CRU_DPLL_CON0, -1 },
	{ "dpll_con1", RK3026_SUSPEND_CRU, RK3026_CRU_DPLL_CON1, -1 },
	{ "clksel26", RK3026_SUSPEND_CRU, RK3026_CRU_CLKSEL_CON26, -1 },
};

/* the CRU words depth 3 changes and gives back, compared around a sleep */
static const struct {
	const char *name;
	u32 off;
} rk3026_pm_cru_regs[] = {
	{ "mode_con", 0x40 }, { "clksel0", 0x44 }, { "clksel1", 0x48 },
	{ "clksel10", 0x6c }, { "apll_con0", 0x00 }, { "apll_con1", 0x04 },
	{ "cpll_con0", 0x20 }, { "cpll_con1", 0x24 }, { "gpll_con0", 0x30 },
	{ "gpll_con1", 0x34 }, { "clkgate0", 0xd0 }, { "clkgate1", 0xd4 },
	{ "clkgate2", 0xd8 }, { "clkgate3", 0xdc }, { "clkgate4", 0xe0 },
	{ "clkgate5", 0xe4 }, { "clkgate6", 0xe8 }, { "clkgate7", 0xec },
	{ "clkgate8", 0xf0 }, { "clkgate9", 0xf4 },
};

#define RK3026_PM_CRU_REGS		ARRAY_SIZE(rk3026_pm_cru_regs)

static u32 rk3026_pm_cru_before[RK3026_PM_CRU_REGS];
static u32 rk3026_pm_cru_after[RK3026_PM_CRU_REGS];
static u32 rk3026_pm_cru_diffs;		/* sleeps that changed one */
static u32 rk3026_pm_cru_diff_last;	/* BIT(i): changed in the last */

static u32 rk3026_pm_ddr_reg_now(const struct rk3026_pm_ddr_reg *r)
{
	return readl_relaxed((void __iomem *)rk3026_pm_table[r->base] + r->off);
}

static u32 rk3026_pm_cycles, rk3026_pm_done, rk3026_pm_failed;
static u32 rk3026_pm_refused;
static u32 rk3026_pm_again;
static bool rk3026_pm_guarded;
static u32 rk3026_pm_gate7;

/* the A9 global timer's rate, for the routine's tick counts */
static struct clk *rk3026_pm_gt_clk;
static unsigned long rk3026_pm_gt_rate;

static int rk3026_pm_gt_rate_cb(struct notifier_block *nb, unsigned long event,
				void *data)
{
	struct clk_notifier_data *nd = data;

	if (event == POST_RATE_CHANGE)
		WRITE_ONCE(rk3026_pm_gt_rate, nd->new_rate);
	return NOTIFY_OK;
}

static struct notifier_block rk3026_pm_gt_nb = {
	.notifier_call = rk3026_pm_gt_rate_cb,
};

struct rk3026_pm_wake_report {
	u32 cycle;
	u32 code;
	u32 os_reg6;
	u32 hppir;
	u32 gic[RK3026_GIC_PENDING_WORDS];
	u32 gic_active[RK3026_GIC_PENDING_WORDS];
	u32 gic_enable[RK3026_GIC_PENDING_WORDS];
	u32 gpio[RK3026_GPIO_BANKS];
	u32 gpio_raw[RK3026_GPIO_BANKS];
	u32 gt_refoff;		/* ticks with the auto refresh off */
	u32 gt_delay;		/* ticks of one .Ldelay(1) */
	u32 reg62_pre;		/* PHY REG62 before the calibration */
	u32 plls;		/* the PLLs the routine parked */
	unsigned long gt_rate;
};

static struct rk3026_pm_wake_report rk3026_pm_wake;
/* the last wake that reached no wake-up source */
static struct rk3026_pm_wake_report rk3026_pm_again_wake;

static u32 *rk3026_pm_sram_top(int word)
{
	return (u32 *)rk3026_pm_table[RK3026_SUSPEND_STACK_TOP] - 1 - word;
}

static void rk3026_pm_debug_crumb(u32 point, u32 code)
{
	writel_relaxed(RK3026_PM_CRUMB_C | point << 8 | (code & 0xff),
		       rk3026_pm_grf + RK3026_GRF_OS_REG7);
	dsb();
}

/*
 * pm.c's first stage of gating: below depth 4 it closes nothing, and
 * under the guard pclk_wdt stays open (its parents are in the hold set).
 */
u16 rk3026_pm_debug_hold(int reg)
{
	if (rk3026_pm_depth < 4)
		return 0xffff;
	if (reg == 7 && rk3026_pm_guarded)
		return RK3026_CRU_PCLK_WDT;
	return 0;
}

void rk3026_pm_debug_gated(void)
{
	rk3026_pm_debug_crumb(RK3026_PM_C_GATED, 0);
}

/* CPU0's rate and voltage at the sleep's start (S4a), by point */
struct rk3026_pm_cpu_line {
	u32 cycle;
	u32 khz;	/* the policy's current rate */
	u32 hw_khz;	/* the clock's */
	u32 max_khz;	/* the policy's max */
	int uv;		/* the CPU supply */
};

static struct rk3026_pm_cpu_line rk3026_pm_cpu[RK3026_PM_CPU_POINTS];
static const char *const rk3026_pm_cpu_names[RK3026_PM_CPU_POINTS] = {
	"notify", "prepare", "post",
};

static struct regulator *rk3026_pm_vdd;

/* sleepable: the PM notifier and .prepare, with I2C still running */
void rk3026_pm_debug_cpu(int point)
{
	struct rk3026_pm_cpu_line *l = &rk3026_pm_cpu[point];
	struct cpufreq_policy *policy;
	struct regulator *vdd;
	struct device *cpu;

	if (!rk3026_pm_vdd) {
		cpu = get_cpu_device(0);
		vdd = cpu ? regulator_get_optional(cpu, "cpu") : NULL;
		if (!IS_ERR_OR_NULL(vdd))
			rk3026_pm_vdd = vdd;
	}
	l->cycle = rk3026_pm_cycles;
	l->uv = rk3026_pm_vdd ? regulator_get_voltage(rk3026_pm_vdd) : -ENODEV;
	l->khz = 0;
	l->max_khz = 0;
	policy = cpufreq_cpu_get(0);
	if (policy) {
		l->khz = policy->cur;
		l->max_khz = policy->max;
		cpufreq_cpu_put(policy);
	}
	l->hw_khz = cpufreq_get(0);
	pr_info("rk3026-pm: cpu %s: %u kHz (hw %u kHz), max %u kHz, vdd %d uV\n",
		rk3026_pm_cpu_names[point], l->khz, l->hw_khz, l->max_khz,
		l->uv);
}

/* .prepare: after cpufreq_suspend(), before the late suspend (deep) */
int rk3026_pm_debug_prepare(void)
{
	rk3026_pm_debug_cpu(RK3026_PM_CPU_PREPARE);
	return 0;
}

int rk3026_pm_debug_begin(void)
{
	int i;

	rk3026_pm_cycles++;
	rk3026_pm_debug_crumb(RK3026_PM_C_ENTER, 0);
	writel_relaxed(RK3026_PM_CRUMB_SRAM, rk3026_pm_grf + RK3026_GRF_OS_REG6);

	rk3026_pm_table[RK3026_SUSPEND_FAIL_AT] = rk3026_pm_fail_at;
	rk3026_pm_table[RK3026_SUSPEND_DEPTH] = rk3026_pm_depth;
	rk3026_pm_table[RK3026_SUSPEND_WDT_TORR] = 0;
	rk3026_pm_table[RK3026_SUSPEND_GT_REFOFF] = 0;
	rk3026_pm_table[RK3026_SUSPEND_GT_DELAY] = 0;
	rk3026_pm_table[RK3026_SUSPEND_REG62_PRE] = ~0;
	rk3026_pm_table[RK3026_SUSPEND_PLLS] = 0;
	rk3026_pm_table[RK3026_SUSPEND_PD_MASK] = rk3026_pm_pd;
	rk3026_pm_table[RK3026_SUSPEND_WDT_TORR_SLOW] = 0;
	/*
	 * The routine touches every base in the table, and a read of the
	 * WDT with pclk_wdt gated hangs the bus: the WDT's page goes in only
	 * while the guard has opened the gate, the GRF stands in otherwise.
	 */
	rk3026_pm_table[RK3026_SUSPEND_WDT] = (u32)rk3026_pm_grf;
	rk3026_pm_guarded = false;

	if (rk3026_pm_depth > RK3026_PM_DEPTH_MAX) {
		rk3026_pm_refused++;
		rk3026_pm_debug_crumb(RK3026_PM_C_REFUSED, 0);
		return -EINVAL;
	}

	if (!rk3026_pm_guard)
		goto snapshot;

	/* dw_wdt_suspend() has gated pclk_wdt: open it, then check the WDT */
	rk3026_pm_gate7 = readl_relaxed(rk3026_pm_cru + RK3026_CRU_CLKGATE_CON7);
	writel_relaxed(RK3026_CRU_PCLK_WDT << 16,
		       rk3026_pm_cru + RK3026_CRU_CLKGATE_CON7);
	readl_relaxed(rk3026_pm_cru + RK3026_CRU_CLKGATE_CON7);
	dsb();
	if (!(readl_relaxed(rk3026_pm_wdt + DW_WDT_CR) & DW_WDT_CR_EN)) {
		writel_relaxed(RK3026_CRU_PCLK_WDT << 16 |
			       (rk3026_pm_gate7 & RK3026_CRU_PCLK_WDT),
			       rk3026_pm_cru + RK3026_CRU_CLKGATE_CON7);
		rk3026_pm_refused++;
		rk3026_pm_debug_crumb(RK3026_PM_C_REFUSED, 1);
		pr_err("rk3026-pm: guard on, but the watchdog is not running\n");
		return -ENODEV;
	}
	writel_relaxed(DW_WDT_CRR_KICK, rk3026_pm_wdt + DW_WDT_CRR);
	rk3026_pm_table[RK3026_SUSPEND_WDT] = (u32)rk3026_pm_wdt;
	rk3026_pm_table[RK3026_SUSPEND_WDT_TORR] = DW_WDT_TORR_44S;
	rk3026_pm_table[RK3026_SUSPEND_WDT_TORR_SLOW] = DW_WDT_TORR_44S_SLOW;
	rk3026_pm_guarded = true;

snapshot:
	/*
	 * The CRU words as the sleep starts, after the guard opened pclk_wdt:
	 * rk3026_pm_debug_woke() reads them again before the disarm closes
	 * that gate, so both sides see the guard's gate open.
	 */
	for (i = 0; i < RK3026_PM_CRU_REGS; i++)
		rk3026_pm_cru_before[i] =
			readl_relaxed(rk3026_pm_cru + rk3026_pm_cru_regs[i].off);

	return 0;
}

/* right after the SRAM routine, still with interrupts masked */
void rk3026_pm_debug_woke(u32 code)
{
	u32 diff = 0;
	int i;

	for (i = 0; i < RK3026_PM_CRU_REGS; i++) {
		rk3026_pm_cru_after[i] =
			readl_relaxed(rk3026_pm_cru + rk3026_pm_cru_regs[i].off);
		if (rk3026_pm_cru_after[i] != rk3026_pm_cru_before[i])
			diff |= BIT(i);
	}
	rk3026_pm_cru_diff_last = diff;
	if (diff)
		rk3026_pm_cru_diffs++;
	rk3026_pm_wake.plls = rk3026_pm_table[RK3026_SUSPEND_PLLS];

	rk3026_pm_wake.hppir = rk3026_pm_gicc ?
		readl_relaxed(rk3026_pm_gicc + RK3026_GICC_HPPIR) : 0x3ff;
	rk3026_pm_wake.os_reg6 = readl_relaxed(rk3026_pm_grf + RK3026_GRF_OS_REG6);
	rk3026_pm_debug_crumb(RK3026_PM_C_RETURNED, code);

	rk3026_pm_wake.cycle = rk3026_pm_cycles;
	rk3026_pm_wake.code = code;
	rk3026_pm_wake.gt_refoff = rk3026_pm_table[RK3026_SUSPEND_GT_REFOFF];
	rk3026_pm_wake.gt_delay = rk3026_pm_table[RK3026_SUSPEND_GT_DELAY];
	rk3026_pm_wake.reg62_pre = rk3026_pm_table[RK3026_SUSPEND_REG62_PRE];
	rk3026_pm_wake.gt_rate = READ_ONCE(rk3026_pm_gt_rate);
	for (i = 0; i < RK3026_GIC_PENDING_WORDS; i++) {
		if (!rk3026_pm_gic)
			break;
		rk3026_pm_wake.gic[i] =
			readl_relaxed(rk3026_pm_gic + GIC_DIST_PENDING_SET + 4 * i);
		rk3026_pm_wake.gic_active[i] =
			readl_relaxed(rk3026_pm_gic + GIC_DIST_ACTIVE_SET + 4 * i);
		rk3026_pm_wake.gic_enable[i] =
			readl_relaxed(rk3026_pm_gic + GIC_DIST_ENABLE_SET + 4 * i);
	}
	for (i = 0; i < RK3026_GPIO_BANKS; i++) {
		if (!rk3026_pm_gpio[i])
			continue;
		rk3026_pm_wake.gpio[i] =
			readl_relaxed(rk3026_pm_gpio[i] + RK_GPIO_INT_STATUS);
		rk3026_pm_wake.gpio_raw[i] =
			readl_relaxed(rk3026_pm_gpio[i] + RK_GPIO_INT_RAWSTATUS);
	}
}

/*
 * After outer_resume(), so the guard also covers the L2 coming back on:
 * restart the counter for dw_wdt_resume() and close the gate again. The
 * WDT (peripheral APB) and the CRU (CPU APB) sit behind different
 * bridges, so the last WDT write is read back before the gate closes: a
 * write still in flight to a gated WDT stalls the bus.
 */
void rk3026_pm_debug_disarm(void)
{
	if (!rk3026_pm_guarded)
		return;

	writel_relaxed(DW_WDT_TORR_44S, rk3026_pm_wdt + DW_WDT_TORR);
	writel_relaxed(DW_WDT_CRR_KICK, rk3026_pm_wdt + DW_WDT_CRR);
	readl_relaxed(rk3026_pm_wdt + DW_WDT_TORR);
	dsb();
	writel_relaxed(RK3026_CRU_PCLK_WDT << 16 |
		       (rk3026_pm_gate7 & RK3026_CRU_PCLK_WDT),
		       rk3026_pm_cru + RK3026_CRU_CLKGATE_CON7);
	readl_relaxed(rk3026_pm_cru + RK3026_CRU_CLKGATE_CON7);
	dsb();
	rk3026_pm_guarded = false;
}

static u32 rk3026_pm_gt_ns(u32 ticks, unsigned long rate)
{
	return rate ? (u32)div_u64((u64)ticks * NSEC_PER_SEC, rate) : 0;
}

static void rk3026_pm_debug_report(const char *what,
				   const struct rk3026_pm_wake_report *w)
{
	pr_info("rk3026-pm: cycle %u %s: hppir %#x, gic pending %08x %08x %08x %08x active %08x %08x %08x %08x enable %08x %08x %08x %08x, gpio int %08x %08x %08x %08x raw %08x %08x %08x %08x\n",
		w->cycle, what, w->hppir,
		w->gic[0], w->gic[1], w->gic[2], w->gic[3],
		w->gic_active[0], w->gic_active[1],
		w->gic_active[2], w->gic_active[3],
		w->gic_enable[0], w->gic_enable[1],
		w->gic_enable[2], w->gic_enable[3],
		w->gpio[0], w->gpio[1], w->gpio[2], w->gpio[3],
		w->gpio_raw[0], w->gpio_raw[1], w->gpio_raw[2], w->gpio_raw[3]);
}

void rk3026_pm_debug_end(int ret)
{
	if (ret) {
		rk3026_pm_failed++;
		rk3026_pm_debug_crumb(RK3026_PM_C_FAILED, rk3026_pm_wake.code);
	} else {
		rk3026_pm_done++;
		rk3026_pm_debug_crumb(RK3026_PM_C_DONE, rk3026_pm_wake.code);
	}

	pr_info("rk3026-pm: cycle %u %s, depth %u sram code %#x os_reg6 %#x os_reg7 %#x mcfg %#x mcfg1 %#x soc_con2 %#x phy_reg1 %#x phy_reg264 %#x trefi %#x dpll_con1 %#x\n",
		rk3026_pm_cycles, ret ? "failed" : "done", rk3026_pm_depth,
		rk3026_pm_wake.code, rk3026_pm_wake.os_reg6,
		readl_relaxed(rk3026_pm_grf + RK3026_GRF_OS_REG7),
		readl_relaxed(rk3026_pm_pctl + RK3026_PCTL_MCFG),
		readl_relaxed(rk3026_pm_pctl + RK3026_PCTL_MCFG1),
		readl_relaxed(rk3026_pm_grf + RK3026_GRF_SOC_CON2),
		readl_relaxed(rk3026_pm_phy + RK3026_PHY_REG1),
		readl_relaxed(rk3026_pm_phy + RK3026_PHY_REG264),
		readl_relaxed(rk3026_pm_pctl + RK3026_PCTL_TREFI),
		readl_relaxed(rk3026_pm_cru + RK3026_CRU_DPLL_CON1));
	if (rk3026_pm_wake.reg62_pre != ~0U)
		pr_info("rk3026-pm: cycle %u training: auto refresh off %u ns (%u ticks), 1 us wait %u ns (%u ticks), reg62 before %#x, timer %lu Hz\n",
			rk3026_pm_cycles,
			rk3026_pm_gt_ns(rk3026_pm_wake.gt_refoff,
					rk3026_pm_wake.gt_rate),
			rk3026_pm_wake.gt_refoff,
			rk3026_pm_gt_ns(rk3026_pm_wake.gt_delay,
					rk3026_pm_wake.gt_rate),
			rk3026_pm_wake.gt_delay, rk3026_pm_wake.reg62_pre,
			rk3026_pm_wake.gt_rate);
	if (rk3026_pm_depth >= 3)
		pr_info("rk3026-pm: cycle %u clocks: parked %#x pd %#x cru diff %#x, mode_con %#x clksel0 %#x clksel1 %#x clksel10 %#x apll_con1 %#x cpll_con1 %#x gpll_con1 %#x\n",
			rk3026_pm_cycles, rk3026_pm_wake.plls, rk3026_pm_pd,
			rk3026_pm_cru_diff_last, rk3026_pm_cru_after[0],
			rk3026_pm_cru_after[1], rk3026_pm_cru_after[2],
			rk3026_pm_cru_after[3], rk3026_pm_cru_after[5],
			rk3026_pm_cru_after[7], rk3026_pm_cru_after[9]);
	if (rk3026_pm_depth >= 4)
		pr_info("rk3026-pm: cycle %u gates: %#x %#x %#x %#x %#x %#x %#x %#x %#x %#x, saved gate0 %#x\n",
			rk3026_pm_cycles, rk3026_pm_cru_after[10],
			rk3026_pm_cru_after[11], rk3026_pm_cru_after[12],
			rk3026_pm_cru_after[13], rk3026_pm_cru_after[14],
			rk3026_pm_cru_after[15], rk3026_pm_cru_after[16],
			rk3026_pm_cru_after[17], rk3026_pm_cru_after[18],
			rk3026_pm_cru_after[19],
			rk3026_pm_table[RK3026_SUSPEND_GATE0]);
	rk3026_pm_debug_report("woke", &rk3026_pm_wake);
}

/*
 * After each round: again = the wake-up reached no wake-up source and
 * .suspend_again sleeps once more; keep that round's report.
 */
void rk3026_pm_debug_again(bool again)
{
	if (!again)
		return;

	rk3026_pm_again++;
	rk3026_pm_again_wake = rk3026_pm_wake;
	pr_info("rk3026-pm: cycle %u: no wake-up source (hppir %u), sleeping again (%u so far)\n",
		rk3026_pm_wake.cycle, rk3026_pm_wake.hppir & 0x3ff,
		rk3026_pm_again);
}

static int rk3026_pm_enable_get(void *data, u64 *val)
{
	*val = rk3026_pm_enabled;
	return 0;
}

static int rk3026_pm_enable_set(void *data, u64 val)
{
	if (val != 1)
		return -EINVAL;
	if (rk3026_pm_enabled)
		return 0;

	console_suspend_enabled = false;
	suspend_set_ops(rk3026_pm_ops);
	rk3026_pm_enabled = true;
	pr_info("rk3026-pm: suspend ops registered, consoles kept on in suspend\n");

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(rk3026_pm_enable_fops, rk3026_pm_enable_get,
			 rk3026_pm_enable_set, "%llu\n");

static int rk3026_pm_u32_get(void *data, u64 *val)
{
	*val = *(u32 *)data;
	return 0;
}

/*
 * The depth a fail_at needs to fire: the step codes name PCTL transitions,
 * which only depth 1 makes, PHY/DPLL steps, which only depth 2 makes, PLL
 * locks, which only depth 3 makes, and the reset with the gates closed,
 * which only depth 4 has; at a lower depth such a fail_at would never
 * fire, so the pair is refused either way round (fail_at 0, 1 or 2 work
 * at every depth).
 */
static u32 rk3026_pm_fail_at_depth(u32 fail_at)
{
	if (fail_at <= RK3026_PM_SRAM_WFI)
		return 0;
	if (fail_at == RK3026_PM_FAIL_GATED)
		return 4;
	if (fail_at >= RK3026_SUSPEND_R_APLL)
		return 3;
	if (fail_at >= RK3026_SUSPEND_R_DPLL)
		return 2;
	return 1;
}

static int rk3026_pm_depth_set(void *data, u64 val)
{
	if (val > RK3026_PM_DEPTH_MAX)
		return -EINVAL;
	if (val < rk3026_pm_fail_at_depth(rk3026_pm_fail_at))
		return -EINVAL;
	rk3026_pm_depth = val;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(rk3026_pm_depth_fops, rk3026_pm_u32_get,
			 rk3026_pm_depth_set, "%llu\n");

static int rk3026_pm_fail_at_set(void *data, u64 val)
{
	switch (val) {
	case 0:
	case RK3026_PM_SRAM_ENTRY:
	case RK3026_PM_SRAM_WFI:
	case RK3026_SUSPEND_E_CONFIG:
	case RK3026_SUSPEND_E_ACCESS:
	case RK3026_SUSPEND_E_SLEEP:
	case RK3026_SUSPEND_R_WAKEUP:
	case RK3026_SUSPEND_R_DPLL:
	case RK3026_SUSPEND_R_TRAIN_CFG:
	case RK3026_SUSPEND_R_TRAIN:
	case RK3026_SUSPEND_R_REFRESH:
	case RK3026_SUSPEND_R_TRAIN_GO:
	case RK3026_SUSPEND_R_APLL:
	case RK3026_SUSPEND_R_CPLL:
	case RK3026_SUSPEND_R_GPLL:
	case RK3026_PM_FAIL_GATED:
		break;
	default:
		return -EINVAL;
	}
	if (rk3026_pm_depth < rk3026_pm_fail_at_depth(val))
		return -EINVAL;
	rk3026_pm_fail_at = val;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(rk3026_pm_fail_at_fops, rk3026_pm_u32_get,
			 rk3026_pm_fail_at_set, "%llu\n");

static int rk3026_pm_guard_set(void *data, u64 val)
{
	if (val > 1)
		return -EINVAL;
	if (val && !rk3026_pm_wdt)
		return -ENODEV;
	rk3026_pm_guard = val;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(rk3026_pm_guard_fops, rk3026_pm_u32_get,
			 rk3026_pm_guard_set, "%llu\n");

static int rk3026_pm_pd_set(void *data, u64 val)
{
	if (val & ~(u64)RK3026_PM_PLLS)
		return -EINVAL;
	rk3026_pm_pd = val;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(rk3026_pm_pd_fops, rk3026_pm_u32_get,
			 rk3026_pm_pd_set, "%llu\n");

static int rk3026_pm_mark_set(void *data, u64 val)
{
	writel_relaxed(RK3026_PM_MARK_OS_REG6, rk3026_pm_grf + RK3026_GRF_OS_REG6);
	writel_relaxed(RK3026_PM_MARK_OS_REG7, rk3026_pm_grf + RK3026_GRF_OS_REG7);
	WRITE_ONCE(*rk3026_pm_sram_top(0), RK3026_PM_MARK_SRAM0);
	WRITE_ONCE(*rk3026_pm_sram_top(1), RK3026_PM_MARK_SRAM1);
	dsb();
	pr_info("rk3026-pm: marker written: os_reg6 %#x os_reg7 %#x sram top %#x %#x\n",
		readl_relaxed(rk3026_pm_grf + RK3026_GRF_OS_REG6),
		readl_relaxed(rk3026_pm_grf + RK3026_GRF_OS_REG7),
		READ_ONCE(*rk3026_pm_sram_top(0)),
		READ_ONCE(*rk3026_pm_sram_top(1)));
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(rk3026_pm_mark_fops, NULL, rk3026_pm_mark_set,
			 "%llu\n");

static void rk3026_pm_crumbs_words(struct seq_file *s, const char *pfx,
				   const char *name, const u32 *v, int n)
{
	int i;

	seq_printf(s, "%s_%s", pfx, name);
	for (i = 0; i < n; i++)
		seq_printf(s, " %08x", v[i]);
	seq_putc(s, '\n');
}

static void rk3026_pm_crumbs_wake(struct seq_file *s, const char *pfx,
				  const struct rk3026_pm_wake_report *w)
{
	seq_printf(s, "%s_cycle %u\n%s_code %#x\n%s_os_reg6 %#x\n%s_hppir %u\n",
		   pfx, w->cycle, pfx, w->code, pfx, w->os_reg6, pfx,
		   w->hppir & 0x3ff);
	rk3026_pm_crumbs_words(s, pfx, "gic", w->gic, RK3026_GIC_PENDING_WORDS);
	rk3026_pm_crumbs_words(s, pfx, "gic_active", w->gic_active,
			       RK3026_GIC_PENDING_WORDS);
	rk3026_pm_crumbs_words(s, pfx, "gic_enable", w->gic_enable,
			       RK3026_GIC_PENDING_WORDS);
	rk3026_pm_crumbs_words(s, pfx, "gpio", w->gpio, RK3026_GPIO_BANKS);
	rk3026_pm_crumbs_words(s, pfx, "gpio_raw", w->gpio_raw,
			       RK3026_GPIO_BANKS);
	seq_printf(s, "%s_refoff_ns %u\n%s_delay_1us_ns %u\n%s_reg62_pre %#x\n%s_gt_rate %lu\n%s_plls %#x\n",
		   pfx, rk3026_pm_gt_ns(w->gt_refoff, w->gt_rate),
		   pfx, rk3026_pm_gt_ns(w->gt_delay, w->gt_rate),
		   pfx, w->reg62_pre, pfx, w->gt_rate, pfx, w->plls);
}

static int rk3026_pm_crumbs_show(struct seq_file *s, void *data)
{
	int i;

	seq_printf(s, "os_reg6 %#x\nos_reg7 %#x\n",
		   readl_relaxed(rk3026_pm_grf + RK3026_GRF_OS_REG6),
		   readl_relaxed(rk3026_pm_grf + RK3026_GRF_OS_REG7));
	seq_printf(s, "sram_top %#x %#x\n", READ_ONCE(*rk3026_pm_sram_top(0)),
		   READ_ONCE(*rk3026_pm_sram_top(1)));
	seq_printf(s, "boot_os_reg6 %#x\nboot_os_reg7 %#x\nboot_sram_top %#x %#x\n",
		   rk3026_pm_boot_reg6, rk3026_pm_boot_reg7,
		   rk3026_pm_boot_sram0, rk3026_pm_boot_sram1);
	seq_printf(s, "enabled %d\ndepth %u\nfail_at %u\nguard %u\npd %#x\n",
		   rk3026_pm_enabled, rk3026_pm_depth, rk3026_pm_fail_at,
		   rk3026_pm_guard, rk3026_pm_pd);
	seq_printf(s, "cycles %u\ndone %u\nfailed %u\nrefused %u\nagain %u\n",
		   rk3026_pm_cycles, rk3026_pm_done, rk3026_pm_failed,
		   rk3026_pm_refused, rk3026_pm_again);
	seq_printf(s, "mcfg %#x\nmcfg1 %#x\nsoc_con2 %#x\n",
		   readl_relaxed(rk3026_pm_pctl + RK3026_PCTL_MCFG),
		   readl_relaxed(rk3026_pm_pctl + RK3026_PCTL_MCFG1),
		   readl_relaxed(rk3026_pm_grf + RK3026_GRF_SOC_CON2));
	seq_printf(s, "boot_mcfg %#x\nboot_mcfg1 %#x\nboot_soc_con2 %#x\n",
		   rk3026_pm_boot_mcfg, rk3026_pm_boot_mcfg1,
		   rk3026_pm_boot_soc_con2);
	seq_printf(s, "saved_mcfg %#x\nsaved_mcfg1 %#x\nsaved_soc_con2 %#x\n",
		   rk3026_pm_table[RK3026_SUSPEND_MCFG],
		   rk3026_pm_table[RK3026_SUSPEND_MCFG1],
		   rk3026_pm_table[RK3026_SUSPEND_SOC_CON2]);
	for (i = 0; i < ARRAY_SIZE(rk3026_pm_ddr_regs); i++) {
		const struct rk3026_pm_ddr_reg *r = &rk3026_pm_ddr_regs[i];

		seq_printf(s, "%s %#x\nboot_%s %#x\n", r->name,
			   rk3026_pm_ddr_reg_now(r), r->name, r->boot);
		if (r->saved >= 0)
			seq_printf(s, "saved_%s %#x\n", r->name,
				   rk3026_pm_table[r->saved]);
	}
	seq_printf(s, "cru_diffs %u\ncru_diff_last %#x\n", rk3026_pm_cru_diffs,
		   rk3026_pm_cru_diff_last);
	for (i = 0; i < RK3026_PM_CRU_REGS; i++)
		seq_printf(s, "%s %#x\nlast_%s %#x %#x\n",
			   rk3026_pm_cru_regs[i].name,
			   readl_relaxed(rk3026_pm_cru + rk3026_pm_cru_regs[i].off),
			   rk3026_pm_cru_regs[i].name, rk3026_pm_cru_before[i],
			   rk3026_pm_cru_after[i]);
	seq_printf(s, "saved_gate0 %#x\n", rk3026_pm_table[RK3026_SUSPEND_GATE0]);
	for (i = 0; i < RK3026_PM_CPU_POINTS; i++)
		seq_printf(s, "cpu_%s %u %u %u %u %d\n", rk3026_pm_cpu_names[i],
			   rk3026_pm_cpu[i].cycle, rk3026_pm_cpu[i].khz,
			   rk3026_pm_cpu[i].hw_khz, rk3026_pm_cpu[i].max_khz,
			   rk3026_pm_cpu[i].uv);
	rk3026_pm_crumbs_wake(s, "wake", &rk3026_pm_wake);
	rk3026_pm_crumbs_wake(s, "again", &rk3026_pm_again_wake);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(rk3026_pm_crumbs);

/*
 * The pages the routine reaches, by their main-TLB set: the routine's
 * touches only keep them all in the TLB if no set holds more than its two
 * ways. With the guard on, the WDT page replaces the GRF stand-in. The
 * set is taken as the page number modulo 32, the smallest main TLB; that
 * flags every clash a larger modulo-indexed TLB has, but an A9 that
 * hashes the index could clash where this sees none.
 */
static void __init rk3026_pm_debug_tlb(const u32 *table)
{
	u32 pages[RK3026_SUSPEND_BASES + 1];
	int sets[RK3026_TLB_SETS] = { };
	int i, n = 0, worst = 0;

	for (i = 0; i < RK3026_SUSPEND_BASES; i++)
		pages[n++] = table[i] >> PAGE_SHIFT;
	if (rk3026_pm_wdt)
		pages[n++] = (u32)rk3026_pm_wdt >> PAGE_SHIFT;

	for (i = 0; i < n; i++) {
		int j, dup = 0;

		for (j = 0; j < i; j++)
			dup |= pages[j] == pages[i];
		if (dup)
			continue;
		sets[pages[i] % RK3026_TLB_SETS]++;
		worst = max(worst, sets[pages[i] % RK3026_TLB_SETS]);
	}
	for (i = 0; i < n; i++)
		pr_info("rk3026-pm: table page %d va %#x tlb set %u\n", i,
			pages[i] << PAGE_SHIFT, pages[i] % RK3026_TLB_SETS);
	pr_info("rk3026-pm: tlb worst set holds %d pages (%d sets, 2 ways)%s\n",
		worst, RK3026_TLB_SETS, worst > 2 ? ": CONFLICT" : "");
}

static void __iomem * __init rk3026_pm_debug_iomap(const char *compatible)
{
	struct device_node *np;
	void __iomem *base;

	np = of_find_compatible_node(NULL, NULL, compatible);
	if (!np)
		return NULL;
	base = of_iomap(np, 0);
	of_node_put(np);

	return base;
}

void __init rk3026_pm_debug_init(u32 *table,
				 const struct platform_suspend_ops *ops)
{
	struct device_node *np;
	struct dentry *dir;
	int i = 0, j;

	rk3026_pm_table = table;
	rk3026_pm_ops = ops;
	rk3026_pm_grf = (void __iomem *)table[RK3026_SUSPEND_GRF];
	rk3026_pm_cru = (void __iomem *)table[RK3026_SUSPEND_CRU];

	rk3026_pm_boot_reg6 = readl_relaxed(rk3026_pm_grf + RK3026_GRF_OS_REG6);
	rk3026_pm_boot_reg7 = readl_relaxed(rk3026_pm_grf + RK3026_GRF_OS_REG7);
	rk3026_pm_boot_sram0 = READ_ONCE(*rk3026_pm_sram_top(0));
	rk3026_pm_boot_sram1 = READ_ONCE(*rk3026_pm_sram_top(1));
	pr_info("rk3026-pm: at boot os_reg6 %#x os_reg7 %#x sram top %#x %#x\n",
		rk3026_pm_boot_reg6, rk3026_pm_boot_reg7,
		rk3026_pm_boot_sram0, rk3026_pm_boot_sram1);

	rk3026_pm_pctl = (void __iomem *)table[RK3026_SUSPEND_PCTL];
	rk3026_pm_boot_mcfg = readl_relaxed(rk3026_pm_pctl + RK3026_PCTL_MCFG);
	rk3026_pm_boot_mcfg1 = readl_relaxed(rk3026_pm_pctl + RK3026_PCTL_MCFG1);
	rk3026_pm_boot_soc_con2 = readl_relaxed(rk3026_pm_grf + RK3026_GRF_SOC_CON2);
	pr_info("rk3026-pm: at boot mcfg %#x mcfg1 %#x soc_con2 %#x\n",
		rk3026_pm_boot_mcfg, rk3026_pm_boot_mcfg1,
		rk3026_pm_boot_soc_con2);
	rk3026_pm_phy = (void __iomem *)table[RK3026_SUSPEND_PHY];
	for (j = 0; j < ARRAY_SIZE(rk3026_pm_ddr_regs); j++) {
		struct rk3026_pm_ddr_reg *r = &rk3026_pm_ddr_regs[j];

		r->boot = rk3026_pm_ddr_reg_now(r);
		pr_info("rk3026-pm: at boot %s %#x\n", r->name, r->boot);
	}

	/* the guard puts the watchdog's base into the table (begin) */
	rk3026_pm_wdt = rk3026_pm_debug_iomap("snps,dw-wdt");
	table[RK3026_SUSPEND_WDT] = (u32)rk3026_pm_grf;

	np = of_find_compatible_node(NULL, NULL, "arm,cortex-a9-global-timer");
	if (np) {
		rk3026_pm_gt_clk = of_clk_get(np, 0);
		of_node_put(np);
		if (IS_ERR(rk3026_pm_gt_clk)) {
			rk3026_pm_gt_clk = NULL;
		} else {
			rk3026_pm_gt_rate = clk_get_rate(rk3026_pm_gt_clk);
			clk_notifier_register(rk3026_pm_gt_clk, &rk3026_pm_gt_nb);
		}
	}

	np = of_find_compatible_node(NULL, NULL, "arm,cortex-a9-gic");
	if (np) {
		rk3026_pm_gic = of_iomap(np, 0);
		rk3026_pm_gicc = of_iomap(np, 1);
		of_node_put(np);
	}
	/*
	 * The global timer's node is disabled, so no driver starts it: the
	 * routine's tick counts mean something only if the boot loader left
	 * it running (bit 0); a stopped one reads 0 ticks.
	 */
	if (rk3026_pm_gicc) {
		u32 ctrl = readl_relaxed(rk3026_pm_gicc + RK3026_GICC_GT_CONTROL);

		pr_info("rk3026-pm: global timer control %#x rate %lu%s\n",
			ctrl, rk3026_pm_gt_rate,
			ctrl & 1 ? "" : ": STOPPED, the training window is not measured");
	} else {
		pr_info("rk3026-pm: global timer control: no GICC, not measured\n");
	}
	/* the routine reads GICC_HPPIR after the wfi; the GRF stands in */
	table[RK3026_SUSPEND_GICC] = rk3026_pm_gicc ? (u32)rk3026_pm_gicc :
						      (u32)rk3026_pm_grf;
	for_each_compatible_node(np, NULL, "rockchip,gpio-bank") {
		if (i == RK3026_GPIO_BANKS) {
			of_node_put(np);
			break;
		}
		rk3026_pm_gpio[i++] = of_iomap(np, 0);
	}

	rk3026_pm_debug_tlb(table);

	dir = debugfs_create_dir("rk3026_pm", NULL);
	debugfs_create_file_unsafe("enable", 0600, dir, NULL, &rk3026_pm_enable_fops);
	debugfs_create_file_unsafe("depth", 0600, dir, &rk3026_pm_depth,
				   &rk3026_pm_depth_fops);
	debugfs_create_file_unsafe("fail_at", 0600, dir, &rk3026_pm_fail_at,
				   &rk3026_pm_fail_at_fops);
	debugfs_create_file_unsafe("pd", 0600, dir, &rk3026_pm_pd,
				   &rk3026_pm_pd_fops);
	debugfs_create_file_unsafe("guard", 0600, dir, &rk3026_pm_guard,
				   &rk3026_pm_guard_fops);
	debugfs_create_file_unsafe("mark", 0200, dir, NULL, &rk3026_pm_mark_fops);
	debugfs_create_file("crumbs", 0400, dir, NULL, &rk3026_pm_crumbs_fops);
}
