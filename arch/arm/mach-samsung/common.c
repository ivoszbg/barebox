// SPDX-License-Identifier: GPL-2.0-only
#include <of.h>
#include <deep-probe.h>
#include <init.h>
#include <pm_domain.h>

static const struct of_device_id samsung_of_match[] = {
	{
		.compatible = "samsung,exynos8895",
	},
	{ /* sentinel */ },
};
BAREBOX_DEEP_PROBE_ENABLE(samsung_of_match);

static int samsung_init(void)
{
	if (!of_machine_is_compatible("samsung,exynos8895"))
		return 0;

	genpd_activate();

	return 0;
}
postcore_initcall(samsung_init);
