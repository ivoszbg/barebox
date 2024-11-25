// SPDX-License-Identifier: GPL-2.0-only
/*
 * PWM driver for Rockchip SoCs
 *
 * Copyright (C) 2014 Beniamino Galvani <b.galvani@gmail.com>
 * Copyright (C) 2014 ROCKCHIP, Inc.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <module.h>
#include <stdio.h>
#include <of.h>
#include <linux/device.h>
#include <pwm.h>
#include <linux/time.h>

#define PWM_CTRL_TIMER_EN	(1 << 0)
#define PWM_CTRL_OUTPUT_EN	(1 << 3)

#define PWM_ENABLE		(1 << 0)
#define PWM_CONTINUOUS		(1 << 1)
#define PWM_DUTY_POSITIVE	(1 << 3)
#define PWM_DUTY_NEGATIVE	(0 << 3)
#define PWM_INACTIVE_NEGATIVE	(0 << 4)
#define PWM_INACTIVE_POSITIVE	(1 << 4)
#define PWM_POLARITY_MASK	(PWM_DUTY_POSITIVE | PWM_INACTIVE_POSITIVE)
#define PWM_OUTPUT_LEFT		(0 << 5)
#define PWM_LOCK_EN		(1 << 6)
#define PWM_LP_DISABLE		(0 << 8)

struct rockchip_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk;
	struct clk *pclk;
	const struct rockchip_pwm_data *data;
	void __iomem *base;
};

struct rockchip_pwm_regs {
	unsigned long duty;
	unsigned long period;
	unsigned long cntr;
	unsigned long ctrl;
};

struct rockchip_pwm_data {
	struct rockchip_pwm_regs regs;
	unsigned int prescaler;
	bool supports_polarity;
	bool supports_lock;
	u32 enable_conf;
};

static inline struct rockchip_pwm_chip *to_rockchip_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct rockchip_pwm_chip, chip);
}

static void rockchip_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			       const struct pwm_state *state)
{
	struct rockchip_pwm_chip *pc = to_rockchip_pwm_chip(chip);
	unsigned long period, duty;
	u64 clk_rate, div;
	u32 ctrl;

	clk_rate = clk_get_rate(pc->clk);

	/*
	 * Since period and duty cycle registers have a width of 32
	 * bits, every possible input period can be obtained using the
	 * default prescaler value for all practical clock rate values.
	 */
	div = clk_rate * state->period;
	period = DIV_ROUND_CLOSEST_ULL(div,
				       pc->data->prescaler * NSEC_PER_SEC);

	div = clk_rate * state->duty_cycle;
	duty = DIV_ROUND_CLOSEST_ULL(div, pc->data->prescaler * NSEC_PER_SEC);

	/*
	 * Lock the period and duty of previous configuration, then
	 * change the duty and period, that would not be effective.
	 */
	ctrl = readl_relaxed(pc->base + pc->data->regs.ctrl);
	if (pc->data->supports_lock) {
		ctrl |= PWM_LOCK_EN;
		writel_relaxed(ctrl, pc->base + pc->data->regs.ctrl);
	}

	writel(period, pc->base + pc->data->regs.period);
	writel(duty, pc->base + pc->data->regs.duty);

	if (pc->data->supports_polarity) {
		ctrl &= ~PWM_POLARITY_MASK;
		if (state->polarity == PWM_POLARITY_INVERSED)
			ctrl |= PWM_DUTY_NEGATIVE | PWM_INACTIVE_POSITIVE;
		else
			ctrl |= PWM_DUTY_POSITIVE | PWM_INACTIVE_NEGATIVE;
	}

	/*
	 * Unlock and set polarity at the same time,
	 * the configuration of duty, period and polarity
	 * would be effective together at next period.
	 */
	if (pc->data->supports_lock)
		ctrl &= ~PWM_LOCK_EN;

	writel(ctrl, pc->base + pc->data->regs.ctrl);
}

static int rockchip_pwm_enable(struct pwm_chip *chip,
			       struct pwm_device *pwm,
			       bool enable)
{
	struct rockchip_pwm_chip *pc = to_rockchip_pwm_chip(chip);
	u32 enable_conf = pc->data->enable_conf;
	int ret;
	u32 val;

	if (enable) {
		ret = clk_enable(pc->clk);
		if (ret)
			return ret;
	}

	val = readl_relaxed(pc->base + pc->data->regs.ctrl);

	if (enable)
		val |= enable_conf;
	else
		val &= ~enable_conf;

	writel_relaxed(val, pc->base + pc->data->regs.ctrl);

	if (!enable)
		clk_disable(pc->clk);

	return 0;
}

static int rockchip_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			      const struct pwm_state *state)
{
	struct rockchip_pwm_chip *pc = to_rockchip_pwm_chip(chip);
	struct pwm_state curstate;
	bool enabled;
	int ret = 0;

	ret = clk_enable(pc->pclk);
	if (ret)
		return ret;

	ret = clk_enable(pc->clk);
	if (ret)
		return ret;

	pwm_get_state(pwm, &curstate);
	enabled = curstate.enabled;

	if (state->polarity != curstate.polarity && enabled &&
	    !pc->data->supports_lock) {
		ret = rockchip_pwm_enable(chip, pwm, false);
		if (ret)
			goto out;
		enabled = false;
	}

	rockchip_pwm_config(chip, pwm, state);
	if (state->enabled != enabled) {
		ret = rockchip_pwm_enable(chip, pwm, state->enabled);
		if (ret)
			goto out;
	}

out:
	clk_disable(pc->clk);
	clk_disable(pc->pclk);

	return ret;
}

static const struct pwm_ops rockchip_pwm_ops = {
	.apply = rockchip_pwm_apply,
};

static const struct rockchip_pwm_data pwm_data_v1 = {
	.regs = {
		.duty = 0x04,
		.period = 0x08,
		.cntr = 0x00,
		.ctrl = 0x0c,
	},
	.prescaler = 2,
	.supports_polarity = false,
	.supports_lock = false,
	.enable_conf = PWM_CTRL_OUTPUT_EN | PWM_CTRL_TIMER_EN,
};

static const struct rockchip_pwm_data pwm_data_v2 = {
	.regs = {
		.duty = 0x08,
		.period = 0x04,
		.cntr = 0x00,
		.ctrl = 0x0c,
	},
	.prescaler = 1,
	.supports_polarity = true,
	.supports_lock = false,
	.enable_conf = PWM_OUTPUT_LEFT | PWM_LP_DISABLE | PWM_ENABLE |
		       PWM_CONTINUOUS,
};

static const struct rockchip_pwm_data pwm_data_vop = {
	.regs = {
		.duty = 0x08,
		.period = 0x04,
		.cntr = 0x0c,
		.ctrl = 0x00,
	},
	.prescaler = 1,
	.supports_polarity = true,
	.supports_lock = false,
	.enable_conf = PWM_OUTPUT_LEFT | PWM_LP_DISABLE | PWM_ENABLE |
		       PWM_CONTINUOUS,
};

static const struct rockchip_pwm_data pwm_data_v3 = {
	.regs = {
		.duty = 0x08,
		.period = 0x04,
		.cntr = 0x00,
		.ctrl = 0x0c,
	},
	.prescaler = 1,
	.supports_polarity = true,
	.supports_lock = true,
	.enable_conf = PWM_OUTPUT_LEFT | PWM_LP_DISABLE | PWM_ENABLE |
		       PWM_CONTINUOUS,
};

static const struct of_device_id rockchip_pwm_dt_ids[] = {
	{ .compatible = "rockchip,rk2928-pwm", .data = &pwm_data_v1},
	{ .compatible = "rockchip,rk3288-pwm", .data = &pwm_data_v2},
	{ .compatible = "rockchip,vop-pwm", .data = &pwm_data_vop},
	{ .compatible = "rockchip,rk3328-pwm", .data = &pwm_data_v3},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rockchip_pwm_dt_ids);

static int rockchip_pwm_probe(struct device *dev)
{
	struct rockchip_pwm_chip *pc;
	u32 enable_conf, ctrl;
	bool enabled;
	int ret, count;

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	pc->base = dev_platform_ioremap_resource(dev, 0);
	if (IS_ERR(pc->base))
		return PTR_ERR(pc->base);

	pc->clk = clk_get(dev, "pwm");
	if (IS_ERR(pc->clk)) {
		pc->clk = clk_get(dev, NULL);
		if (IS_ERR(pc->clk))
			return dev_err_probe(dev, PTR_ERR(pc->clk),
					     "Can't get PWM clk\n");
	}

	count = of_count_phandle_with_args(dev->of_node,
					   "clocks", "#clock-cells");
	if (count == 2)
		pc->pclk = clk_get(dev, "pclk");
	else
		pc->pclk = pc->clk;

	if (IS_ERR(pc->pclk))
		return dev_err_probe(dev, PTR_ERR(pc->pclk), "Can't get APB clk\n");

	ret = clk_prepare_enable(pc->clk);
	if (ret)
		return dev_err_probe(dev, ret, "Can't prepare enable PWM clk\n");

	ret = clk_prepare_enable(pc->pclk);
	if (ret) {
		dev_err_probe(dev, ret, "Can't prepare enable APB clk\n");
		goto err_clk;
	}

	dev->priv = pc;

	pc->data = device_get_match_data(dev);
	pc->chip.dev = dev;
	pc->chip.ops = &rockchip_pwm_ops;

	enable_conf = pc->data->enable_conf;
	ctrl = readl_relaxed(pc->base + pc->data->regs.ctrl);
	enabled = (ctrl & enable_conf) == enable_conf;

	pc->chip.devname = of_alias_get(dev->of_node);
	if (!pc->chip.devname)
		pc->chip.devname = basprintf("pwm_%p", pc->base);

	ret = pwmchip_add(&pc->chip);
	if (ret < 0) {
		dev_err_probe(dev, ret, "pwmchip_add() failed\n");
		goto err_pclk;
	}

	/* Keep the PWM clk enabled if the PWM appears to be up and running. */
	if (!enabled)
		clk_disable(pc->clk);

	clk_disable(pc->pclk);

	return 0;

err_pclk:
	clk_disable_unprepare(pc->pclk);
err_clk:
	clk_disable_unprepare(pc->clk);

	return ret;
}

static void rockchip_pwm_remove(struct device *dev)
{
	struct rockchip_pwm_chip *pc = dev->priv;

	pwmchip_remove(&pc->chip);
}

static struct driver rockchip_pwm_driver = {
	.name = "rockchip-pwm",
	.of_match_table = rockchip_pwm_dt_ids,
	.probe = rockchip_pwm_probe,
	.remove = rockchip_pwm_remove,
};
device_platform_driver(rockchip_pwm_driver);

MODULE_AUTHOR("Beniamino Galvani <b.galvani@gmail.com>");
MODULE_DESCRIPTION("Rockchip SoC PWM driver");
MODULE_LICENSE("GPL v2");
