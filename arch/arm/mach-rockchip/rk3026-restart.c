// SPDX-License-Identifier: GPL-2.0-only
/*
 * RK3026 restart through a core reset
 *
 * The CRU global resets (GLB_SRST_FST, GLB_SRST_SND) also reset the GPIO
 * banks. Where the PMIC's power hold is a SoC GPIO, as on the ONYX BOOX
 * Robinson Crusoe 2 (ACT8931 PWRHLD on GPIO1_A2), the hold drops with
 * them, and with no external power the PMIC switches every rail off: a
 * reboot on battery turns the board off.
 *
 * The vendor kernel resets the cores instead (rk30_arch_reset,
 * soft_reset): every PLL to slow mode, the clock selects back to their
 * reset values and every clock gate open, then a pulse of the module
 * resets with the buses idle, and last the MCORE, CORE0, CORE1 and L2C
 * resets in SOFTRST_CON0. The GPIO banks, the GRF, the internal SRAM and
 * the DDR controller keep their state, and CPU0 restarts in the BootROM,
 * which loads the loader as on a cold start. Do the same, with the
 * module resets limited to the bus masters: the SD controller in
 * particular has no reset of its own in the BootROM or the loader, and
 * one left in the middle of a transfer, or with a clock tree the next
 * stage does not expect, keeps the next kernel from finding the card.
 *
 * With the PLLs in slow mode DDR cannot be used, so the last steps run
 * from the internal SRAM. They are entered through soft_restart(), which
 * cleans and turns off the caches and the MMU: the Cortex-A9 keeps its L1
 * contents across this reset, and dirty lines written back later would
 * corrupt whatever the next boot has loaded by then.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <asm/system_misc.h>
#include <dt-bindings/clock/rk3026-cru.h>

#include "core.h"

#define RK3026_GRF_SOC_CON0		0x140
#define RK3026_GRF_IMEM_REMAP_CLR	BIT(28)

#define DWC2_DCTL			0x804
#define DWC2_DCTL_SFTDISCON		BIT(1)

#define RK3026_CRU_SOFTRST_CON(id)	(0x110 + (id) / 16 * 4)
#define RK3026_CRU_SOFTRST_BIT(id)	BIT((id) % 16)

static void __iomem *rk3026_restart_code;
static void __iomem *rk3026_restart_cru;
static phys_addr_t rk3026_restart_code_phys;
static u32 rk3026_restart_cru_phys;
static u32 rk3026_restart_grf_phys;
static struct regmap *rk3026_restart_grf;
static void __iomem *rk3026_restart_otg;
static struct clk *rk3026_restart_otg_clk;

/*
 * The core reset leaves the OTG controller as it is, D+ pull-up included,
 * so a USB host would keep talking to a device that no longer answers.
 * Disconnect first and give the host time to see it, so the next boot's
 * gadget enumerates afresh. After dwc2's shutdown (a normal reboot) the
 * PHY is off and the controller clock gated: then there is nothing to do,
 * and the controller must not be touched.
 */
static void rk3026_restart_usb_disconnect(void)
{
	u32 dctl;

	if (!rk3026_restart_otg || !__clk_is_enabled(rk3026_restart_otg_clk))
		return;

	dctl = readl(rk3026_restart_otg + DWC2_DCTL);
	writel(dctl | DWC2_DCTL_SFTDISCON, rk3026_restart_otg + DWC2_DCTL);
	mdelay(100);
}

/*
 * The core reset leaves the DesignWare watchdog running, and nothing in the
 * BootROM or the loader feeds it before the loader arms it again for the
 * next kernel: a watchdog that expires in between resets the whole chip,
 * the power hold with it. Stop it with a pulse of its soft reset, as the
 * vendor's module sweep (and any global reset) did.
 */
static void rk3026_restart_stop_wdt(void)
{
	u32 bit = RK3026_CRU_SOFTRST_BIT(SRST_WDT);
	void __iomem *reg = rk3026_restart_cru + RK3026_CRU_SOFTRST_CON(SRST_WDT);

	writel(bit << 16 | bit, reg);
	udelay(10);
	writel(bit << 16, reg);
}

static int rk3026_restart(struct sys_off_data *data)
{
	size_t size = rk3026_restart_sram_end - rk3026_restart_sram;
	size_t cru = rk3026_restart_sram_cru - rk3026_restart_sram;
	size_t grf = rk3026_restart_sram_grf - rk3026_restart_sram;

	rk3026_restart_usb_disconnect();
	rk3026_restart_stop_wdt();

	/* CPU0 must restart in the BootROM, not in the SMP trampoline */
	regmap_write(rk3026_restart_grf, RK3026_GRF_SOC_CON0,
		     RK3026_GRF_IMEM_REMAP_CLR);

	memcpy_toio(rk3026_restart_code, rk3026_restart_sram, size);
	writel(rk3026_restart_cru_phys, rk3026_restart_code + cru);
	writel(rk3026_restart_grf_phys, rk3026_restart_code + grf);
	/* the routine is fetched with the MMU off: the copy must be in SRAM */
	wmb();

	soft_restart(rk3026_restart_code_phys);

	return NOTIFY_DONE;
}

static void __init rk3026_restart_otg_init(void)
{
	struct device_node *np;
	struct resource res;
	const char *mode;

	for_each_compatible_node(np, NULL, "snps,dwc2") {
		if (!of_device_is_available(np))
			continue;
		if (!of_property_read_string(np, "dr_mode", &mode) &&
		    !strcmp(mode, "host"))
			continue;
		if (of_address_to_resource(np, 0, &res))
			continue;

		rk3026_restart_otg_clk = of_clk_get(np, 0);
		if (IS_ERR(rk3026_restart_otg_clk)) {
			rk3026_restart_otg_clk = NULL;
			continue;
		}

		rk3026_restart_otg = ioremap(res.start, SZ_4K);
		if (!rk3026_restart_otg) {
			clk_put(rk3026_restart_otg_clk);
			rk3026_restart_otg_clk = NULL;
			continue;
		}

		of_node_put(np);
		return;
	}
}

static int __init rk3026_restart_init(void)
{
	size_t size = rk3026_restart_sram_end - rk3026_restart_sram;
	struct device_node *np;
	struct resource res;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "rockchip,rk3026-restart-sram");
	if (!np)
		return 0;

	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret)
		return ret;

	if (resource_size(&res) < size) {
		pr_err("%s: SRAM section of 0x%llx bytes is too small for 0x%zx\n",
		       __func__, (u64)resource_size(&res), size);
		return -EINVAL;
	}
	rk3026_restart_code_phys = res.start;

	np = of_find_compatible_node(NULL, NULL, "rockchip,rk3026-cru");
	if (!np)
		return -ENODEV;
	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret)
		return ret;
	rk3026_restart_cru_phys = res.start;

	rk3026_restart_cru = ioremap(res.start, resource_size(&res));
	if (!rk3026_restart_cru)
		return -ENOMEM;

	np = of_find_compatible_node(NULL, NULL, "rockchip,rk3026-grf");
	if (!np)
		return -ENODEV;
	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret)
		return ret;
	rk3026_restart_grf_phys = res.start;

	rk3026_restart_grf = syscon_regmap_lookup_by_compatible("rockchip,rk3026-grf");
	if (IS_ERR(rk3026_restart_grf))
		return PTR_ERR(rk3026_restart_grf);

	rk3026_restart_code = ioremap(rk3026_restart_code_phys, size);
	if (!rk3026_restart_code)
		return -ENOMEM;

	rk3026_restart_otg_init();

	/* ahead of the CRU driver's global reset, which stays as the fallback */
	return PTR_ERR_OR_ZERO(register_sys_off_handler(SYS_OFF_MODE_RESTART,
							SYS_OFF_PRIO_HIGH,
							rk3026_restart, NULL));
}
arch_initcall(rk3026_restart_init);
