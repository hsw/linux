/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NoC QoS priorities for the RK3026 EBC's DRAM fetch.
 */
#ifndef __RK3026_EBC_QOS_H__
#define __RK3026_EBC_QOS_H__

#include <linux/types.h>

/* The SoC's NoC QoS window: one generator per master port. */
#define RK3026_QOS_BASE		0x10128000
#define RK3026_QOS_SIZE		0x8000

/*
 * Write every priority the table names through @write(), which gets @ctx as
 * it was passed in: the driver hands over the window's regmap, a test a
 * recorder of its own.
 */
void rk3026_ebc_qos_program(void *ctx,
			    void (*write)(void *ctx, u32 off, u32 val));

#endif /* __RK3026_EBC_QOS_H__ */
