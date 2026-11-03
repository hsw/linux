// SPDX-License-Identifier: GPL-2.0
/*
 * NoC QoS priorities for the RK3026 EBC's DRAM fetch.
 *
 * A scan fetches both 4-bpp images over the EBC's AHB master port while the
 * CPUs, the GPU and the peripherals fetch their own over the same NoC. The
 * generators that arbitrate those ports reset to values that put CPU DRAM
 * traffic ABOVE the display: CPU1W and CPU1R reset to 0xf, the top level in
 * both regulator states, so nothing can demote them; the EBC resets to 0x8,
 * the second level below its bandwidth budget and the lowest above it, and a
 * scan's line bursts are above it.
 *
 * A CPU copy loop then makes a scan fetch lines that are already stale. The
 * controller has no under-run flag, so the scan still reports its 38 frames
 * and its 454 ms, the driver's own image is correct and dmesg is clean -- only
 * the glass shows it (streaks, or an undriven region).
 *
 * The vendor kernel writes the CPU-side ports to 0 in cpu_axi_init()
 * (arch/arm/mach-rk3026/common.c), i.e. in SoC code at boot. A bootloader that
 * does the same (U-Boot arch_cpu_init()) cures it, but a kernel booted from a
 * loader that does not, or from a loader whose warm reset kept the values,
 * runs with the resets.
 *
 * So do what cpu_axi_init() does, from here: the EBC is the port that gets
 * starved, and its probe is the first place that runs with the VIO1 NIU clock
 * (ACLK_LCDC1) enabled, which the EBC's own QoS port sits behind.
 *
 * PRIORITY uses the vendor's 4-bit (high << 2 | low) packing (arch/arm/plat-rk
 * include/plat/cpu_axi.h). In the reset mode (regulator, MODE 3) "high" is the
 * level while the port is under its bandwidth budget and "low" the level while
 * it is over it, so (0, 0) is the lowest level in both states and (3, 3) the
 * highest. The values below are the vendor's, for the CPU-side ports, both
 * LCDCs and the GPU. The one value added here is the EBC's own port, at the
 * top level: the vendor leaves it at reset, and at the top a CPU port that
 * something raises again cannot outrank the fetch.
 *
 * Only PRIORITY is written: MODE, BANDWIDTH and SATURATION keep their resets,
 * as they do under the vendor kernel.
 */

#include <linux/kernel.h>
#include <linux/types.h>

#include "rk3026_ebc_qos.h"

struct rk3026_ebc_qos_port {
	u32	off;
	u32	val;
};

/*
 * Port base offsets from the vendor's mach-rk3026/include/mach/cpu_axi.h,
 * PRIORITY at +0x08 of each port (plat-rk/include/plat/cpu_axi.h). The window
 * covers all of them, CPU-side ports and VIO ports alike.
 */
#define RK3026_QOS_PRIORITY		0x08
#define RK3026_QOS_PORT(base)		((base) + RK3026_QOS_PRIORITY)

#define RK3026_QOS_CPU0			RK3026_QOS_PORT(0x2000)
#define RK3026_QOS_CPU1W		RK3026_QOS_PORT(0x2080)
#define RK3026_QOS_CPU1R		RK3026_QOS_PORT(0x2100)
#define RK3026_QOS_PERI			RK3026_QOS_PORT(0x4000)
#define RK3026_QOS_GPU			RK3026_QOS_PORT(0x5000)
#define RK3026_QOS_LCDC1		RK3026_QOS_PORT(0x7000)
#define RK3026_QOS_EBC			RK3026_QOS_PORT(0x7080)
#define RK3026_QOS_LCDC0		RK3026_QOS_PORT(0x7180)

#define RK3026_QOS_LEVEL(high, low)	((((high) & 3) << 2) | ((low) & 3))

/* cpu_axi_init()'s values, then the EBC's own at the top level. */
static const struct rk3026_ebc_qos_port rk3026_ebc_qos_ports[] = {
	{ RK3026_QOS_CPU0,	RK3026_QOS_LEVEL(0, 0) },
	{ RK3026_QOS_CPU1W,	RK3026_QOS_LEVEL(0, 0) },
	{ RK3026_QOS_CPU1R,	RK3026_QOS_LEVEL(0, 0) },
	{ RK3026_QOS_PERI,	RK3026_QOS_LEVEL(0, 0) },
	{ RK3026_QOS_GPU,	RK3026_QOS_LEVEL(2, 1) },
	{ RK3026_QOS_LCDC1,	RK3026_QOS_LEVEL(3, 3) },
	{ RK3026_QOS_EBC,	RK3026_QOS_LEVEL(3, 3) },
	{ RK3026_QOS_LCDC0,	RK3026_QOS_LEVEL(3, 3) },
};

void rk3026_ebc_qos_program(void *ctx,
			    void (*write)(void *ctx, u32 off, u32 val))
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rk3026_ebc_qos_ports); i++)
		write(ctx, rk3026_ebc_qos_ports[i].off,
		      rk3026_ebc_qos_ports[i].val);
}
