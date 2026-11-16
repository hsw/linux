// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK3026 EBC: the NoC QoS priorities of the GPU and EBC ports.
 *
 * A scan fetches its images over the EBC's own master port while the GPU, the
 * CPUs and the peripherals fetch theirs over the same NoC. With the QoS
 * generators at their reset values a GPU rendering into the EBC's buffer while
 * another GPU job loads DRAM starves that fetch: the scan fetches lines that
 * are already stale, and the glass shows column bands of garbage. The
 * controller has no under-run flag, so the scan itself looks normal -- its
 * frames all run, its time is the usual one, the driver's image is right and
 * dmesg is clean. Measured on the RC2: lima rendering under a second lima
 * load left garbage on 23 to 32 of 192 scans at the resets, none with the GPU
 * port at 0x9 and the EBC port at 0xf.
 *
 * PRIORITY packs two 2-bit levels, (high << 2) | low: in the reset mode
 * (regulator, MODE 3) "high" is the level while the port is under its
 * bandwidth budget, "low" the one while it is over. 0x9 is (2, 1), the GPU's
 * value in the vendor kernel's cpu_axi_init() (arch/arm/mach-rk3026/common.c;
 * the GPU port resets to 0xd, (3, 1)). 0xf is (3, 3), the top level in both
 * states: the vendor leaves the EBC's port at its reset 0x8, (2, 0), the
 * lowest level once a line burst takes it over budget. Only PRIORITY is
 * written; MODE, BANDWIDTH and SATURATION keep their resets.
 *
 * Written before every scan (rk3026_ebc_drive()), not once at probe: two
 * clock enables and two register writes per scan, and the values are right
 * whatever reset, re-probe or loader got to the window since.
 *
 * A generator answers only while its port is clocked; an access to one that
 * is not hangs the bus and the board resets. The GPU port needs ACLK_GPU. The
 * ports at +0x7000, the EBC's among them, need both video NIUs: VIO1, the
 * EBC's own "aclk" (ACLK_LCDC1), and VIO0, which only ACLK_LCDC0 holds open
 * and which clk_disable_unused() gates on a board without an LCDC0 user. A
 * read with VIO1 alone held hung the RC2. So each access holds "qos-gpu" and
 * "qos-vio0" (prepared at probe) and "aclk" around it, and without the window
 * or either clock nothing here touches the window at all.
 */

#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <kunit/visibility.h>

#include "rk3026_ebc.h"

/*
 * Port bases inside the window, from the vendor's
 * mach-rk3026/include/mach/cpu_axi.h; PRIORITY at +0x08 of each.
 */
#define NOC_QOS_PRIORITY		0x08
#define NOC_QOS_GPU			0x5000
#define NOC_QOS_EBC			0x7080

#define NOC_QOS_LEVEL(high, low)	((((high) & 3) << 2) | ((low) & 3))

const struct rk3026_ebc_noc_qos_port
rk3026_ebc_noc_qos_ports[RK3026_EBC_NOC_QOS_PORTS] = {
	{ "gpu", NOC_QOS_GPU + NOC_QOS_PRIORITY, NOC_QOS_LEVEL(2, 1) },
	{ "ebc", NOC_QOS_EBC + NOC_QOS_PRIORITY, NOC_QOS_LEVEL(3, 3) },
};
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_noc_qos_ports);

/**
 * rk3026_ebc_noc_qos_init - find the QoS window and the clocks behind it
 * @ebc: controller, with @dev set
 *
 * Optional: a device tree without the "rockchip,rk3026-qos" node or without
 * the "qos-gpu" and "qos-vio0" clocks on the EBC node leaves @noc_qos NULL,
 * and the priorities are never touched.
 *
 * Returns 0, or a probe error (-EPROBE_DEFER included) of a clock that the
 * device tree names but that cannot be had.
 */
int rk3026_ebc_noc_qos_init(struct rk3026_ebc *ebc)
{
	struct device *dev = ebc->dev;
	struct regmap *qos;
	struct clk *gpu, *vio0;

	/*
	 * Found by compatible, not through a phandle: the EBC node would need
	 * one more property, and the clocks below already tie it to the window.
	 */
	qos = syscon_regmap_lookup_by_compatible("rockchip,rk3026-qos");
	if (IS_ERR(qos)) {
		if (PTR_ERR(qos) == -EPROBE_DEFER)
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "NoC QoS window not ready\n");
		qos = NULL;
	}

	gpu = devm_clk_get_optional_prepared(dev, "qos-gpu");
	if (IS_ERR(gpu))
		return dev_err_probe(dev, PTR_ERR(gpu), "bad qos-gpu clock\n");
	vio0 = devm_clk_get_optional_prepared(dev, "qos-vio0");
	if (IS_ERR(vio0))
		return dev_err_probe(dev, PTR_ERR(vio0),
				     "bad qos-vio0 clock\n");

	if (!qos || !gpu || !vio0) {
		dev_info(dev, "no NoC QoS window or clocks: GPU and EBC priorities left as found\n");
		return 0;
	}

	ebc->noc_qos_gpu_clk = gpu;
	ebc->noc_qos_vio0_clk = vio0;
	ebc->noc_qos = qos;
	dev_info(dev, "NoC QoS: GPU priority %#x, EBC %#x before every scan\n",
		 rk3026_ebc_noc_qos_ports[0].val,
		 rk3026_ebc_noc_qos_ports[1].val);
	return 0;
}

/* Hold every clock a port of the table sits behind, in process context. */
static int noc_qos_hold(struct rk3026_ebc *ebc)
{
	int ret;

	ret = clk_enable(ebc->noc_qos_gpu_clk);
	if (ret)
		return ret;
	ret = clk_enable(ebc->noc_qos_vio0_clk);
	if (ret)
		goto err_gpu;
	ret = clk_prepare_enable(ebc->aclk);
	if (ret)
		goto err_vio0;
	return 0;

err_vio0:
	clk_disable(ebc->noc_qos_vio0_clk);
err_gpu:
	clk_disable(ebc->noc_qos_gpu_clk);
	return ret;
}

static void noc_qos_release(struct rk3026_ebc *ebc)
{
	clk_disable_unprepare(ebc->aclk);
	clk_disable(ebc->noc_qos_vio0_clk);
	clk_disable(ebc->noc_qos_gpu_clk);
}

/**
 * rk3026_ebc_noc_qos_apply - write the table's priorities
 * @ebc: controller
 *
 * Called before every scan, in process context. A no-op without the window
 * and its clocks; @noc_qos_writes counts the times the whole table was
 * written.
 */
void rk3026_ebc_noc_qos_apply(struct rk3026_ebc *ebc)
{
	unsigned int i;
	int ret = 0;

	if (!ebc->noc_qos)
		return;

	if (noc_qos_hold(ebc)) {
		dev_warn_ratelimited(ebc->dev,
				     "NoC QoS clocks not enabled, priorities not written\n");
		return;
	}
	for (i = 0; i < RK3026_EBC_NOC_QOS_PORTS && !ret; i++)
		ret = regmap_write(ebc->noc_qos,
				   rk3026_ebc_noc_qos_ports[i].off,
				   rk3026_ebc_noc_qos_ports[i].val);
	noc_qos_release(ebc);

	if (!ret)
		atomic_inc(&ebc->noc_qos_writes);
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_noc_qos_apply);

/**
 * rk3026_ebc_noc_qos_read - read back the table's priorities
 * @ebc: controller
 * @prio: one value per row of rk3026_ebc_noc_qos_ports[]
 *
 * Process context; the same clocks are held as for the writes. Returns 0,
 * -ENODEV without the window and its clocks, or a clock or regmap error.
 */
int rk3026_ebc_noc_qos_read(struct rk3026_ebc *ebc,
			    u32 prio[RK3026_EBC_NOC_QOS_PORTS])
{
	unsigned int i;
	int ret;

	if (!ebc->noc_qos)
		return -ENODEV;

	ret = noc_qos_hold(ebc);
	if (ret)
		return ret;
	for (i = 0; i < RK3026_EBC_NOC_QOS_PORTS && !ret; i++)
		ret = regmap_read(ebc->noc_qos,
				  rk3026_ebc_noc_qos_ports[i].off, &prio[i]);
	noc_qos_release(ebc);
	return ret;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_noc_qos_read);
