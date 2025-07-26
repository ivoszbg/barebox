// SPDX-License-Identifier: GPL-2.0-only
#include <common.h>
#include <pbl.h>
#include <linux/sizes.h>
#include <asm/barebox-arm-head.h>
#include <asm/barebox-arm.h>
#include <asm/sections.h>
#include <asm/cache.h>
#include <asm/mmu.h>

static noinline void dreamlte_continue(void)
{
	void *fdt;
	unsigned long membase, memsize;
	extern char __dtb_exynos8895_dreamlte_start[];

	fdt = __dtb_exynos8895_dreamlte_start + get_runtime_offset();

	fdt_find_mem(fdt, &membase, &memsize);

	barebox_arm_entry(membase, memsize, fdt);
}

ENTRY_FUNCTION(start_dreamlte, r0, r1, r2)
{
	arm_cpu_lowlevel_init();

	relocate_to_current_adr();

	setup_c();

	dreamlte_continue();
}
