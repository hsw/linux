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
 *            active, 1 = the DRAM in self-refresh (PCTL Low_power)
 *   fail_at  0 = never; 1 or 2: back out of the SRAM routine at that point
 *            (1 = on the SRAM stack, 2 = right before wfi; at depth 1 the
 *            DRAM is woken first), so .enter returns -EIO; a step code
 *            (rk3026-suspend.h RK3026_SUSPEND_E_* or _R_*, e.g. 0x14 or
 *            0x21): that step's PCTL_STAT poll fails as if the state never
 *            came, so an E_* step backs out (-EIO) and an R_* step resets
 *            the SoC from the SRAM, leaving 0x503400(0x40 | code) in
 *            GRF_OS_REG6; a step code needs depth 1 (refused at depth
 *            0, and depth 0 is refused while one is set)
 *   guard    1 = keep the DesignWare watchdog counting through the sleep:
 *            pclk_wdt opened in .enter, the counter restarted with TOP 15
 *            (2^31 cycles, 44.7 s at the 48 MHz pclk_peri) right before wfi
 *            and again once the L2 is back on, the gate closed again; the
 *            watchdog must be running already (a feeder has /dev/watchdog
 *            open)
 *   mark     write anything: test markers into GRF_OS_REG6/7, the top two
 *            words of the suspend SRAM section and the kernel log (those
 *            two words are the routine's stack: a sleep overwrites them)
 *   crumbs   read: OS_REG6/7 and the SRAM words now and at boot, the
 *            knobs, the cycle counters, the last wake report and the
 *            report of the last wake that reached no wake-up source
 *            (after which .suspend_again slept again); PCTL MCFG/MCFG1 and
 *            GRF_SOC_CON2 now, at boot and as the routine last saved them
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

#include <linux/console.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/seq_file.h>
#include <linux/suspend.h>

#include "rk3026-suspend.h"

#define RK3026_GRF_OS_REG7		0x1e4
#define RK3026_GRF_SOC_CON2		0x148
#define RK3026_PCTL_MCFG1		0x7c
#define RK3026_PCTL_MCFG		0x80

/* Cortex-A9 main TLB: 2-way; 32 sets in its smallest configuration */
#define RK3026_TLB_SETS			32

/* pclk_wdt: CLKGATE_CON7 bit 15 (clk-rk3026.c) */
#define RK3026_CRU_CLKGATE_CON7		0xec
#define RK3026_CRU_PCLK_WDT		BIT(15)

#define DW_WDT_CR			0x00
#define DW_WDT_CR_EN			BIT(0)
/* TOP 15 and TOP_INIT 15 */
#define DW_WDT_TORR_44S			0xff

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

static u32 rk3026_pm_boot_reg6, rk3026_pm_boot_reg7;
static u32 rk3026_pm_boot_sram0, rk3026_pm_boot_sram1;
static u32 rk3026_pm_boot_mcfg, rk3026_pm_boot_mcfg1, rk3026_pm_boot_soc_con2;
static void __iomem *rk3026_pm_pctl;

static u32 rk3026_pm_cycles, rk3026_pm_done, rk3026_pm_failed;
static u32 rk3026_pm_refused;
static u32 rk3026_pm_again;
static bool rk3026_pm_guarded;
static u32 rk3026_pm_gate7;

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

int rk3026_pm_debug_begin(void)
{
	rk3026_pm_cycles++;
	rk3026_pm_debug_crumb(RK3026_PM_C_ENTER, 0);
	writel_relaxed(RK3026_PM_CRUMB_SRAM, rk3026_pm_grf + RK3026_GRF_OS_REG6);

	rk3026_pm_table[RK3026_SUSPEND_FAIL_AT] = rk3026_pm_fail_at;
	rk3026_pm_table[RK3026_SUSPEND_DEPTH] = rk3026_pm_depth;
	rk3026_pm_table[RK3026_SUSPEND_WDT_TORR] = 0;
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
		return 0;

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
	rk3026_pm_guarded = true;

	return 0;
}

/* right after the SRAM routine, still with interrupts masked */
void rk3026_pm_debug_woke(u32 code)
{
	int i;

	rk3026_pm_wake.hppir = rk3026_pm_gicc ?
		readl_relaxed(rk3026_pm_gicc + RK3026_GICC_HPPIR) : 0x3ff;
	rk3026_pm_wake.os_reg6 = readl_relaxed(rk3026_pm_grf + RK3026_GRF_OS_REG6);
	rk3026_pm_debug_crumb(RK3026_PM_C_RETURNED, code);

	rk3026_pm_wake.cycle = rk3026_pm_cycles;
	rk3026_pm_wake.code = code;
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

	pr_info("rk3026-pm: cycle %u %s, depth %u sram code %#x os_reg6 %#x os_reg7 %#x mcfg %#x mcfg1 %#x soc_con2 %#x\n",
		rk3026_pm_cycles, ret ? "failed" : "done", rk3026_pm_depth,
		rk3026_pm_wake.code, rk3026_pm_wake.os_reg6,
		readl_relaxed(rk3026_pm_grf + RK3026_GRF_OS_REG7),
		readl_relaxed(rk3026_pm_pctl + RK3026_PCTL_MCFG),
		readl_relaxed(rk3026_pm_pctl + RK3026_PCTL_MCFG1),
		readl_relaxed(rk3026_pm_grf + RK3026_GRF_SOC_CON2));
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
 * The step codes name PCTL transitions, which only depth 1 makes: at
 * depth 0 such a fail_at would never fire, so the pair is refused either
 * way round (fail_at 0, 1 or 2 work at both depths).
 */
static bool rk3026_pm_fail_at_step(u32 fail_at)
{
	return fail_at > RK3026_PM_SRAM_WFI;
}

static int rk3026_pm_depth_set(void *data, u64 val)
{
	if (val > RK3026_PM_DEPTH_MAX)
		return -EINVAL;
	if (!val && rk3026_pm_fail_at_step(rk3026_pm_fail_at))
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
		break;
	default:
		return -EINVAL;
	}
	if (!rk3026_pm_depth && rk3026_pm_fail_at_step(val))
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
}

static int rk3026_pm_crumbs_show(struct seq_file *s, void *data)
{
	seq_printf(s, "os_reg6 %#x\nos_reg7 %#x\n",
		   readl_relaxed(rk3026_pm_grf + RK3026_GRF_OS_REG6),
		   readl_relaxed(rk3026_pm_grf + RK3026_GRF_OS_REG7));
	seq_printf(s, "sram_top %#x %#x\n", READ_ONCE(*rk3026_pm_sram_top(0)),
		   READ_ONCE(*rk3026_pm_sram_top(1)));
	seq_printf(s, "boot_os_reg6 %#x\nboot_os_reg7 %#x\nboot_sram_top %#x %#x\n",
		   rk3026_pm_boot_reg6, rk3026_pm_boot_reg7,
		   rk3026_pm_boot_sram0, rk3026_pm_boot_sram1);
	seq_printf(s, "enabled %d\ndepth %u\nfail_at %u\nguard %u\n",
		   rk3026_pm_enabled, rk3026_pm_depth, rk3026_pm_fail_at,
		   rk3026_pm_guard);
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
	rk3026_pm_crumbs_wake(s, "wake", &rk3026_pm_wake);
	rk3026_pm_crumbs_wake(s, "again", &rk3026_pm_again_wake);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(rk3026_pm_crumbs);

/*
 * The pages the routine reaches, by their main-TLB set: the routine's
 * touches only keep them all in the TLB if no set holds more than its two
 * ways. With the guard on, the WDT page replaces the GRF stand-in.
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
	int i = 0;

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

	/* the guard puts the watchdog's base into the table (begin) */
	rk3026_pm_wdt = rk3026_pm_debug_iomap("snps,dw-wdt");
	table[RK3026_SUSPEND_WDT] = (u32)rk3026_pm_grf;

	np = of_find_compatible_node(NULL, NULL, "arm,cortex-a9-gic");
	if (np) {
		rk3026_pm_gic = of_iomap(np, 0);
		rk3026_pm_gicc = of_iomap(np, 1);
		of_node_put(np);
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
	debugfs_create_file_unsafe("guard", 0600, dir, &rk3026_pm_guard,
				   &rk3026_pm_guard_fops);
	debugfs_create_file_unsafe("mark", 0200, dir, NULL, &rk3026_pm_mark_fops);
	debugfs_create_file("crumbs", 0400, dir, NULL, &rk3026_pm_crumbs_fops);
}
