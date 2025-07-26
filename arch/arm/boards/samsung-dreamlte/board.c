// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 Pengutronix e.K.
 *
 */
#include <common.h>
#include <init.h>
#include <of.h>
#include <deep-probe.h>
#include <asm/memory.h>
#include <linux/sizes.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <asm/system.h>
#include <asm/system_info.h>
#include <mach/layerscape/layerscape.h>
#include <of_address.h>

#define MACHINE "dreamlte"

static int dreamlte_mem_init(void)
{
	if (!of_machine_is_compatible("samsung,dreamlte"))
		return 0;

	//arm_add_mem_device("ram0", 0x80000000, 0x3c800000);
	//arm_add_mem_device("ram1", 0xc0000000, 0x40000000);
	//arm_add_mem_device("ram2", 0x880000000, 0x80000000);

	return 0;
}
mem_initcall(dreamlte_mem_init);

static int dreamlte_postcore_init(void)
{
	if (!of_machine_is_compatible("samsung,dreamlte"))
		return 0;

	/* unlock decon trigger control */
	writel(0x1281, 0x12860000 + 0x70);

	return 0;
}
coredevice_initcall(dreamlte_postcore_init);

static inline int dreamlte_init(struct device *dev)
{
	barebox_set_model("ARM SAMSUNG " MACHINE);
	barebox_set_hostname(MACHINE);

	return 0;
}

static const struct of_device_id dreamlte_of_match[] = {
	{ .compatible = "samsung,dreamlte" },
	{ /* Sentinel */},
};

MODULE_DEVICE_TABLE(of, dreamlte_of_match);

static struct driver dreamlte_board_driver = {
	.name = "board-dreamlte",
	.probe = dreamlte_init,
	.of_compatible = dreamlte_of_match,
};

postcore_platform_driver(dreamlte_board_driver);
