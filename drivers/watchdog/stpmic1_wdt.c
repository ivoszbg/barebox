// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Copyright (C) 2018, STMicroelectronics - All Rights Reserved
 */

#include <common.h>
#include <init.h>
#include <watchdog.h>
#include <asm/io.h>
#include <of_device.h>
#include <linux/iopoll.h>
#include <poweroff.h>
#include <mfd/syscon.h>
#include <restart.h>
#include <reset_source.h>

#define RESTART_SR		0x05
#define MAIN_CR			0x10
#define WCHDG_CR		0x1B
#define WCHDG_TIMER_CR		0x1C

/* Restart Status Register (RESTART_SR) */
#define R_RST			BIT(0)
#define R_SWOFF			BIT(1)
#define R_WDG			BIT(2)
#define R_PKEYLKP		BIT(3)
#define R_VINOK_FA		BIT(4)

/* Main PMIC Control Register (MAIN_CR) */
#define SWOFF                   BIT(0)
#define RREQ_EN                 BIT(1)

/* Watchdog Control Register (WCHDG_CR) */
#define WDT_START		BIT(0)
#define WDT_PING		BIT(1)
#define WDT_START_MASK		BIT(0)
#define WDT_PING_MASK		BIT(1)
#define WDT_STOP		0

#define PMIC_WDT_MIN_TIMEOUT 1
#define PMIC_WDT_MAX_TIMEOUT 256
#define PMIC_WDT_DEFAULT_TIMEOUT 30


struct stpmic1_reset_reason {
	uint32_t mask;
	enum reset_src_type type;
	int instance;
};

struct stpmic1_wdt {
	struct watchdog wdd;
	struct restart_handler restart;
	struct poweroff_handler poweroff;
	struct regmap *regmap;
	unsigned int timeout;
};

static inline struct stpmic1_wdt *to_stpmic1_wdt(struct watchdog *wdd)
{
	return container_of(wdd, struct stpmic1_wdt, wdd);
}

static int stpmic1_wdt_ping(struct regmap *regmap)
{
	return regmap_update_bits(regmap, WCHDG_CR, WDT_PING_MASK, WDT_PING);
}

static int stpmic1_wdt_start(struct regmap *regmap, unsigned int timeout)
{
	int ret = regmap_write(regmap, WCHDG_TIMER_CR, timeout - 1);
	if (ret)
		return ret;

	return regmap_update_bits(regmap, WCHDG_CR, WDT_START_MASK, WDT_START);
}

static int stpmic1_wdt_stop(struct regmap *regmap)
{
	return regmap_update_bits(regmap, WCHDG_CR, WDT_START_MASK, WDT_STOP);
}

static int stpmic1_wdt_set_timeout(struct watchdog *wdd, unsigned int timeout)
{
	struct stpmic1_wdt *wdt = to_stpmic1_wdt(wdd);
	int ret;

	if (!timeout)
		return stpmic1_wdt_stop(wdt->regmap);

	if (timeout < PMIC_WDT_MIN_TIMEOUT || timeout > wdd->timeout_max)
		return -EINVAL;

	if (wdt->timeout == timeout)
		return stpmic1_wdt_ping(wdt->regmap);

	ret = stpmic1_wdt_start(wdt->regmap, timeout);
	if (ret)
		return ret;

	wdt->timeout = timeout;
	return 0;
}

static void __noreturn stpmic1_restart_handler(struct restart_handler *rst)
{
	struct stpmic1_wdt *wdt = container_of(rst, struct stpmic1_wdt, restart);

	regmap_write_bits(wdt->regmap, MAIN_CR,
			  SWOFF | RREQ_EN, SWOFF | RREQ_EN);

	mdelay(1000);
	hang();
}

static void __noreturn stpmic1_poweroff(struct poweroff_handler *handler)
{
	struct stpmic1_wdt *wdt = container_of(handler, struct stpmic1_wdt, poweroff);

	shutdown_barebox();

	regmap_write_bits(wdt->regmap, MAIN_CR,
			  SWOFF | RREQ_EN, SWOFF);

	mdelay(1000);
	hang();
}

static const struct stpmic1_reset_reason stpmic1_reset_reasons[] = {
	{ R_VINOK_FA,	RESET_BROWNOUT, 0 },
	{ R_PKEYLKP,	RESET_EXT, 0 },
	{ R_WDG,	RESET_WDG, 2 },
	{ R_SWOFF,	RESET_RST, 0 },
	{ R_RST,	RESET_EXT, 0 },
	{ /* sentinel */ }
};

static int stpmic1_set_reset_reason(struct regmap *map)
{
	enum reset_src_type type = RESET_POR;
	u32 reg;
	int ret;
	int i, instance = 0;

	ret = regmap_read(map, RESTART_SR, &reg);
	if (ret)
		return ret;

	for (i = 0; stpmic1_reset_reasons[i].mask; i++) {
		if (reg & stpmic1_reset_reasons[i].mask) {
			type     = stpmic1_reset_reasons[i].type;
			instance = stpmic1_reset_reasons[i].instance;
			break;
		}
	}

	reset_source_set_prinst(type, 400, instance);

	pr_info("STPMIC1 reset reason %s (RESTART_SR: 0x%08x)\n",
		reset_source_name(), reg);

	return 0;
}

static int stpmic1_wdt_probe(struct device_d *dev)
{
	struct stpmic1_wdt *wdt;
	struct watchdog *wdd;
	int ret;

	wdt = xzalloc(sizeof(*wdt));
	wdt->regmap = dev->parent->priv;

	wdd = &wdt->wdd;
	wdd->hwdev = dev;
	wdd->set_timeout = stpmic1_wdt_set_timeout;
	wdd->timeout_max = PMIC_WDT_MAX_TIMEOUT;
	wdd->timeout_cur = PMIC_WDT_DEFAULT_TIMEOUT;

	/* have the watchdog reset, not power-off the system */
	regmap_write_bits(wdt->regmap, MAIN_CR, RREQ_EN, RREQ_EN);

	ret = watchdog_register(wdd);
	if (ret) {
		dev_err(dev, "Failed to register watchdog device\n");
		return ret;
	}

	wdt->restart.name = "stpmic1-reset";
	wdt->restart.restart = stpmic1_restart_handler;
	wdt->restart.priority = 300;

	ret = restart_handler_register(&wdt->restart);
	if (ret)
		dev_warn(dev, "Cannot register restart handler\n");

	wdt->poweroff.name = "stpmic1-poweroff";
	wdt->poweroff.poweroff = stpmic1_poweroff;
	wdt->poweroff.priority = 200;

	ret = poweroff_handler_register(&wdt->poweroff);
	if (ret)
		dev_warn(dev, "Cannot register poweroff handler\n");

	stpmic1_set_reset_reason(wdt->regmap);
	if (ret)
		dev_warn(dev, "Cannot query reset reason\n");

	dev_info(dev, "probed\n");
	return 0;
}

static __maybe_unused const struct of_device_id stpmic1_wdt_of_match[] = {
	{ .compatible = "st,stpmic1-wdt" },
	{ /* sentinel */ }
};

static struct driver_d stpmic1_wdt_driver = {
	.name  = "stpmic1-wdt",
	.probe = stpmic1_wdt_probe,
	.of_compatible = DRV_OF_COMPAT(stpmic1_wdt_of_match),
};
device_platform_driver(stpmic1_wdt_driver);