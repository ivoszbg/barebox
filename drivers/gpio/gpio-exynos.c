// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2009 Samsung Electronics
 * Minkyu Kang <mk7.kang@samsung.com>
 */

#include <common.h>
#include <init.h>
#include <driver.h>
#include <io.h>
#include <of.h>
#include <gpio.h>

#define S5P_GPIO_GET_PIN(x)		(x % GPIO_PER_BANK)

#define CON_MASK(val)			  (0xf << ((val) << 2))
#define CON_SFR(gpio, cfg)		 ((cfg) << ((gpio) << 2))
#define CON_SFR_UNSHIFT(val, gpio) ((val) >> ((gpio) << 2))

#define DAT_MASK(gpio)			 (0x1 << (gpio))
#define DAT_SET(gpio)			  (0x1 << (gpio))

#define PULL_MASK(gpio)			(0x3 << ((gpio) << 1))
#define PULL_MODE(gpio, pull)	  ((pull) << ((gpio) << 1))

#define DRV_MASK(gpio)			 (0x3 << ((gpio) << 1))
#define DRV_SET(gpio, mode)		((mode) << ((gpio) << 1))
#define RATE_MASK(gpio)			(0x1 << (gpio + 16))
#define RATE_SET(gpio)			 (0x1 << (gpio + 16))

/* Pin configurations */
#define S5P_GPIO_INPUT	0x0
#define S5P_GPIO_OUTPUT	0x1
#define S5P_GPIO_IRQ	0xf
#define S5P_GPIO_FUNC(x)	(x)

/* Pull mode */
#define S5P_GPIO_PULL_NONE	0x0
#define S5P_GPIO_PULL_DOWN	0x1
#define S5P_GPIO_PULL_UP	0x3

/* Drive Strength level */
#define S5P_GPIO_DRV_1X	0x0
#define S5P_GPIO_DRV_3X	0x1
#define S5P_GPIO_DRV_2X	0x2
#define S5P_GPIO_DRV_4X	0x3
#define S5P_GPIO_DRV_FAST	0x0
#define S5P_GPIO_DRV_SLOW	0x1

/* Platform data for each bank */
struct exynos_gpio_plat {
	void __iomem *bank_base;
	const char *bank_name; /* Name of port, e.g. 'gpa0' */
};

/* Information about each bank at runtime */
struct exynos_bank_info {
	void __iomem *bank_base;
	struct gpio_chip chip;
};

static void exynos_gpio_cfg_pin(void __iomem *base, unsigned gpio, int cfg)
{
	unsigned int value;

	value = readl(base + 0x00); /* Offset for configuration register */
	value &= ~CON_MASK(gpio);
	value |= CON_SFR(gpio, cfg);
	writel(value, base + 0x00);
}

static void exynos_gpio_cfg_set_value(void __iomem *base, unsigned gpio, int en)
{
	unsigned int value;

	value = readl(base + 0x04); /* Offset for data register */
	value &= ~DAT_MASK(gpio);
	if (en)
		value |= DAT_SET(gpio);
	writel(value, base + 0x04);
}

static int exynos_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	struct exynos_bank_info *info = container_of(chip, struct exynos_bank_info, chip);

	exynos_gpio_cfg_pin(info->bank_base, gpio, S5P_GPIO_INPUT);

	return 0;
}

static int exynos_gpio_direction_output(struct gpio_chip *chip, unsigned gpio, int value)
{
	struct exynos_bank_info *info = container_of(chip, struct exynos_bank_info, chip);

	exynos_gpio_cfg_set_value(info->bank_base, gpio, value);
	exynos_gpio_cfg_pin(info->bank_base, gpio, S5P_GPIO_OUTPUT);

	return 0;
}

static int exynos_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	struct exynos_bank_info *info = container_of(chip, struct exynos_bank_info, chip);
	unsigned int value;

	value = readl(info->bank_base + 0x04);
	return !!(value & DAT_MASK(gpio));
}

static int exynos_gpio_set_value(struct gpio_chip *chip, unsigned gpio, int value)
{
	struct exynos_bank_info *info = container_of(chip, struct exynos_bank_info, chip);

	exynos_gpio_cfg_set_value(info->bank_base, gpio, value);

	return 0;
}

static struct gpio_ops exynos_gpio_ops = {
	.direction_input = exynos_gpio_direction_input,
	.direction_output = exynos_gpio_direction_output,
	.get = exynos_gpio_get_value,
	.set = exynos_gpio_set_value,
};

static int exynos_gpio_probe(struct device *dev)
{
	struct exynos_gpio_plat *plat = dev->platform_data;
	struct exynos_bank_info *info;
	struct gpio_chip *chip;
	int ret, id;

	info = xzalloc(sizeof(*info));
	info->bank_base = plat->bank_base;

	info->chip.ops = &exynos_gpio_ops;
	if (dev->id < 0) {
		id = of_alias_get_id(dev->of_node, "gpio");
		if (id < 0)
			return id;
		info->base = dev_get_mem_region(dev->parent, 0);
		info->chip.base = id * 32;
	} else {
		id = dev->id;
		info->base = dev_get_mem_region(dev, 0);
		info->chip.base = dev->id * 32;
	}

	info->chip.ngpio = 32;
	info->chip.dev = dev;

	gpiochip_add(&info->chip);

	return 0;
}

static struct of_device_id exynos_gpio_dt_ids[] = {
	{ .compatible = "samsung,exynos-gpio" },
	{ /* sentinel */ }
};

static struct driver exynos_gpio_driver = {
	.name = "exynos_gpio",
	.of_compatible = exynos_gpio_dt_ids,
	.probe = exynos_gpio_probe,
};
device_platform_driver(exynos_gpio_driver);

