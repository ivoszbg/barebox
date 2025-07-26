// SPDX-License-Identifier: GPL-2.0+
/*
 * Exynos pinctrl driver common code.
 * Copyright (C) 2016 Samsung Electronics
 * Thomas Abraham <thomas.ab@samsung.com>
 */

#include <common.h>
#include <gpio.h>
#include <init.h>
#include <malloc.h>
#include <mfd/syscon.h>
#include <linux/regmap.h>
#include <of.h>
#include <of_address.h>
#include <pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>

#include <linux/basic_mmio_gpio.h>
#include <linux/clk.h>
#include <linux/err.h>

#include "pinctrl-exynos.h"

/*
 * Bank type for alive type. Bit fields:
 * CON: 4, DAT: 1, PUD: 2, DRV: 2
 */
const struct samsung_pin_bank_type bank_type_alive = {
	.fld_width = { 4, 1, 2, 2, },
	.reg_offset = { 0x00, 0x04, 0x08, 0x0c, },
};

/*
 * Bank type for non-alive type. Bit fields:
 * CON: 4, DAT: 1, PUD: 2, DRV: 3
 */
const struct samsung_pin_bank_type exynos8895_bank_type_off  = {
	.fld_width = { 4, 1, 2, 3, },
	.reg_offset = { 0x00, 0x04, 0x08, 0x0c, },
};

static const char * const exynos_pinctrl_props[PINCFG_TYPE_NUM] = {
	[PINCFG_TYPE_FUNC]	= "samsung,pin-function",
	[PINCFG_TYPE_DAT]	= "samsung,pin-val",
	[PINCFG_TYPE_PUD]	= "samsung,pin-pud",
	[PINCFG_TYPE_DRV]	= "samsung,pin-drv",
};

static void parse_pin(const char *pin_name, u32 *pin, char *bank_name)
{
	u32 idx = 0;

	while (pin_name[idx] != '-') {
		bank_name[idx] = pin_name[idx];
		idx++;
	}
	bank_name[idx] = '\0';
	*pin = pin_name[++idx] - '0';
}

static const struct samsung_pin_bank_data *get_bank(struct device *dev,
						    const char *bank_name)
{
	struct exynos_pinctrl_priv *priv = dev->priv;
	const struct samsung_pin_ctrl *pin_ctrl_array = priv->pin_ctrl;
	const struct samsung_pin_bank_data *bank_data;
	u32 nr_banks, pin_ctrl_idx = 0, idx = 0;

	while (true) {
		const struct samsung_pin_ctrl *pin_ctrl =
			&pin_ctrl_array[pin_ctrl_idx];

		nr_banks = pin_ctrl->nr_banks;
		if (!nr_banks)
			break;

		bank_data = pin_ctrl->pin_banks;
		for (idx = 0; idx < nr_banks; idx++) {
			dev_info(dev, "pinctrl[%d] bank_data[%d] name is: %s\n",
					pin_ctrl_idx, idx, bank_data[idx].name);
			if (!strcmp(bank_name, bank_data[idx].name))
				return &bank_data[idx];
		}
		pin_ctrl_idx++;
	}

	return NULL;
}

static void exynos_pinctrl_set_pincfg(void __iomem *reg_base, u32 pin_num,
				      u32 val, enum pincfg_type pincfg,
				      const struct samsung_pin_bank_type *type)
{
	u32 width = type->fld_width[pincfg];
	u32 reg_offset = type->reg_offset[pincfg];
	u32 mask = (1 << width) - 1;
	u32 shift = pin_num * width;
	u32 data;

	data = readl(reg_base + reg_offset);
	data &= ~(mask << shift);
	data |= val << shift;
	writel(data, reg_base + reg_offset);
}

static int exynos_pinctrl_set_state(struct pinctrl_device *pdev, struct device_node *np)
{
	struct exynos_pinctrl_priv *priv = to_exynos_pinctrl(pdev);
	unsigned int count, idx;
	unsigned int pinvals[PINCFG_TYPE_NUM];

	count = of_property_count_strings(np, "samsung,pins");
	if (count <= 0)
		return -EINVAL;

	for (idx = 0; idx < PINCFG_TYPE_NUM; ++idx) {
		int ret = of_property_read_u32(np, exynos_pinctrl_props[idx], &pinvals[idx]);
		pinvals[idx] = (ret < 0) ? -1 : pinvals[idx];
	}
	pinvals[PINCFG_TYPE_DAT] = -1;

	for (idx = 0; idx < count; idx++) {
		const struct samsung_pin_bank_data *bank;
		unsigned int pin_num;
		char bank_name[10];
		void __iomem *reg;
		const char *name = NULL;
		int pincfg, err;

		err = of_property_read_string_index(np, "samsung,pins", idx, &name);
		if (err || !name)
			continue;

		parse_pin(name, &pin_num, bank_name);
		bank = get_bank(pdev->dev, bank_name);
		reg = IOMEM(priv->base + bank->offset);

		for (pincfg = 0; pincfg < PINCFG_TYPE_NUM; ++pincfg) {
			unsigned int val = pinvals[pincfg];

			if (val != -1)
				exynos_pinctrl_set_pincfg(reg, pin_num, val,
							  pincfg, bank->type);
		}
	}

	return 0;
}

#define EXYNOS_PIN_BANK(pins, reg, id)			\
	{						\
		.type		= &bank_type_alive,	\
		.offset		= reg,			\
		.nr_pins	= pins,			\
		.name		= id			\
	}

#define EXYNOS8895_PIN_BANK(pins, reg, id)			\
	{							\
		.type		= &exynos8895_bank_type_off,	\
		.offset		= reg,				\
		.nr_pins	= pins,				\
		.name		= id				\
	}

/* pin banks of exynos8895 pin-controller 0 (ALIVE) */
static const struct samsung_pin_bank_data exynos8895_pin_banks0[] __initconst = {
	EXYNOS_PIN_BANK(8, 0x020, "gpa0"),
	EXYNOS_PIN_BANK(8, 0x040, "gpa1"),
	EXYNOS_PIN_BANK(8, 0x060, "gpa2"),
	EXYNOS_PIN_BANK(8, 0x080, "gpa3"),
	EXYNOS_PIN_BANK(7, 0x0a0, "gpa4"),
};

/* pin banks of exynos8895 pin-controller 1 (ABOX) */
static const struct samsung_pin_bank_data exynos8895_pin_banks1[] __initconst = {
	EXYNOS_PIN_BANK(8, 0x000, "gph0"),
	EXYNOS_PIN_BANK(7, 0x020, "gph1"),
	EXYNOS_PIN_BANK(4, 0x040, "gph3"),
};

/* pin banks of exynos8895 pin-controller 2 (VTS) */
static const struct samsung_pin_bank_data exynos8895_pin_banks2[] __initconst = {
	EXYNOS_PIN_BANK(3, 0x000, "gph2"),
};

/* pin banks of exynos8895 pin-controller 3 (FSYS0) */
static const struct samsung_pin_bank_data exynos8895_pin_banks3[] __initconst = {
	EXYNOS8895_PIN_BANK(3, 0x000, "gpi0"),
	EXYNOS8895_PIN_BANK(8, 0x020, "gpi1"),
};

/* pin banks of exynos8895 pin-controller 4 (FSYS1) */
static const struct samsung_pin_bank_data exynos8895_pin_banks4[] __initconst = {
	EXYNOS_PIN_BANK(8, 0x000, "gpj1"),
	EXYNOS_PIN_BANK(7, 0x020, "gpj0"),
};

/* pin banks of exynos8895 pin-controller 5 (BUSC) */
static const struct samsung_pin_bank_data exynos8895_pin_banks5[] __initconst = {
	EXYNOS_PIN_BANK(2, 0x000, "gpb2"),
};

/* pin banks of exynos8895 pin-controller 6 (PERIC0) */
static const struct samsung_pin_bank_data exynos8895_pin_banks6[] __initconst = {
	EXYNOS_PIN_BANK(8, 0x000, "gpd0"),
	EXYNOS_PIN_BANK(8, 0x020, "gpd1"),
	EXYNOS_PIN_BANK(4, 0x040, "gpd2"),
	EXYNOS_PIN_BANK(5, 0x060, "gpd3"),
	EXYNOS_PIN_BANK(4, 0x080, "gpb1"),
	EXYNOS_PIN_BANK(8, 0x0a0, "gpe7"),
	EXYNOS_PIN_BANK(8, 0x0c0, "gpf1"),
};

/* pin banks of exynos8895 pin-controller 7 (PERIC1) */
static const struct samsung_pin_bank_data exynos8895_pin_banks7[] __initconst = {
	EXYNOS_PIN_BANK(3, 0x000, "gpb0"),
	EXYNOS_PIN_BANK(5, 0x020, "gpc0"),
	EXYNOS_PIN_BANK(5, 0x040, "gpc1"),
	EXYNOS_PIN_BANK(8, 0x060, "gpc2"),
	EXYNOS_PIN_BANK(8, 0x080, "gpc3"),
	EXYNOS_PIN_BANK(4, 0x0a0, "gpk0"),
	EXYNOS_PIN_BANK(8, 0x0c0, "gpe5"),
	EXYNOS_PIN_BANK(8, 0x0e0, "gpe6"),
	EXYNOS_PIN_BANK(8, 0x100, "gpe2"),
	EXYNOS_PIN_BANK(8, 0x120, "gpe3"),
	EXYNOS_PIN_BANK(8, 0x140, "gpe4"),
	EXYNOS_PIN_BANK(4, 0x160, "gpf0"),
	EXYNOS_PIN_BANK(8, 0x180, "gpe1"),
	EXYNOS_PIN_BANK(2, 0x1a0, "gpg0"),
};

static const struct samsung_pin_ctrl exynos8895_pin_ctrl[] __initconst = {
	{
		/* pin-controller instance 0 ALIVE data */
		.pin_banks	= exynos8895_pin_banks0,
		.nr_banks	= ARRAY_SIZE(exynos8895_pin_banks0),
	}, {
		/* pin-controller instance 1 ABOX data */
		.pin_banks	= exynos8895_pin_banks1,
		.nr_banks	= ARRAY_SIZE(exynos8895_pin_banks1),
	}, {
		/* pin-controller instance 2 VTS data */
		.pin_banks	= exynos8895_pin_banks2,
		.nr_banks	= ARRAY_SIZE(exynos8895_pin_banks2),
	}, {
		/* pin-controller instance 3 FSYS0 data */
		.pin_banks	= exynos8895_pin_banks3,
		.nr_banks	= ARRAY_SIZE(exynos8895_pin_banks3),
	}, {
		/* pin-controller instance 4 FSYS1 data */
		.pin_banks	= exynos8895_pin_banks4,
		.nr_banks	= ARRAY_SIZE(exynos8895_pin_banks4),
	}, {
		/* pin-controller instance 5 BUSC data */
		.pin_banks	= exynos8895_pin_banks5,
		.nr_banks	= ARRAY_SIZE(exynos8895_pin_banks5),
	}, {
		/* pin-controller instance 6 PERIC0 data */
		.pin_banks	= exynos8895_pin_banks6,
		.nr_banks	= ARRAY_SIZE(exynos8895_pin_banks6),
	}, {
		/* pin-controller instance 7 PERIC1 data */
		.pin_banks	= exynos8895_pin_banks7,
		.nr_banks	= ARRAY_SIZE(exynos8895_pin_banks7),
	},
};

static struct pinctrl_ops exynos_pinctrl_ops = {
	.set_state = exynos_pinctrl_set_state
};

int exynos_pinctrl_probe(struct device *dev)
{
	struct exynos_pinctrl_priv *info;
	int ret, id;

	info = xzalloc(sizeof(*info));
	if (!info)
		return -ENOMEM;

	info->base = dev_request_mem_region(dev, 0);
	if (!info->base)
		return -EINVAL;

	id = of_alias_get_id(dev->of_node, "pinctrl");
	if (id < 0) {
		dev_err(dev, "failed to get alias id\n");
		return EINVAL;
	}

	info->pin_ctrl = (const struct samsung_pin_ctrl *)
			 device_get_match_data(dev) + id;
	info->dev = dev;

	pr_info("%s: id:%x nr_banks:%x\n", __func__, id, info->pin_ctrl->nr_banks);
	dev->priv = info;

	info->pctl_dev.dev = dev;
	info->pctl_dev.ops = &exynos_pinctrl_ops;

	ret = pinctrl_register(&info->pctl_dev);

	return ret;
}

static const struct of_device_id exynos_pinctrl_dt_ids[] = {
	{ .compatible = "samsung,exynos8895-pinctrl",
		.data = &exynos8895_pin_ctrl },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, exynos_pinctrl_dt_ids);

struct driver exynos_pinctrl_driver = {
	.name = "exynos-pinctrl",
	.probe = exynos_pinctrl_probe,
	.of_compatible = DRV_OF_COMPAT(exynos_pinctrl_dt_ids),
};

core_platform_driver(exynos_pinctrl_driver);
