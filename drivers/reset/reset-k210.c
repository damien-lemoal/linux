// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 */
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <soc/canaan/k210-sysctl.h>

#include <dt-bindings/reset/k210-rst.h>

#define K210_RST_MASK	0x27FFFFFF

struct k210_rst {
	struct regmap *map;
	struct reset_controller_dev rcdev;
};

static inline struct k210_rst *
to_k210_rst(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct k210_rst, rcdev);
}

static inline int k210_rst_assert(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	struct k210_rst *ksr = to_k210_rst(rcdev);
	u32 bit = BIT(id);

	if (!(bit & K210_RST_MASK))
		return -EINVAL;

	dev_dbg(rcdev->dev, "assert %lu\n", id);

	regmap_update_bits(ksr->map, K210_SYSCTL_PERI_RESET, bit, 1);

	return 0;
}

static inline int k210_rst_deassert(struct reset_controller_dev *rcdev,
				    unsigned long id)
{
	struct k210_rst *ksr = to_k210_rst(rcdev);
	u32 bit = BIT(id);

	if (!(bit & K210_RST_MASK))
		return -EINVAL;

	dev_dbg(rcdev->dev, "deassert %lu\n", id);

	regmap_update_bits(ksr->map, K210_SYSCTL_PERI_RESET, bit, 0);

	return 0;
}

static int k210_rst_reset(struct reset_controller_dev *rcdev,
			  unsigned long id)
{
	int ret;

	dev_dbg(rcdev->dev, "reset %lu\n", id);

	ret = k210_rst_assert(rcdev, id);
	if (ret == 0) {
		udelay(10);
		ret = k210_rst_deassert(rcdev, id);
	}

	return ret;
}

static int k210_rst_status(struct reset_controller_dev *rcdev,
			   unsigned long id)
{
	struct k210_rst *ksr = to_k210_rst(rcdev);
	u32 reg, bit = BIT(id);
	int ret;

	if (!(bit & K210_RST_MASK))
		return -EINVAL;

	ret = regmap_read(ksr->map, K210_SYSCTL_PERI_RESET, &reg);
	if (ret)
		return ret;

	return ret & bit;
}

static const struct reset_control_ops k210_rst_ops = {
	.assert		= k210_rst_assert,
	.deassert	= k210_rst_deassert,
	.reset		= k210_rst_reset,
	.status		= k210_rst_status,
};

static int __init k210_rst_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct k210_rst *ksr;

	dev_info(dev, "K210 reset controller\n");

	if (!dev->parent) {
		dev_err(&pdev->dev, "No parent for K210 reset controller\n");
		return -ENODEV;
	}

	ksr = devm_kzalloc(dev, sizeof(*ksr), GFP_KERNEL);
	if (!ksr)
		return -ENOMEM;

	ksr->map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(ksr->map))
		return PTR_ERR(ksr->map);

	ksr->rcdev.owner = THIS_MODULE;
	ksr->rcdev.dev = dev;
	ksr->rcdev.of_node = dev->of_node;
	ksr->rcdev.nr_resets = fls(K210_RST_MASK);
	ksr->rcdev.ops = &k210_rst_ops;

	return devm_reset_controller_register(dev, &ksr->rcdev);
}

static const struct of_device_id k210_rst_dt_ids[] = {
	{ .compatible = "canaan,k210-rst" },
};

static struct platform_driver k210_rst_driver = {
	.probe	= k210_rst_probe,
	.driver = {
		.name		= "k210-rst",
		.of_match_table	= k210_rst_dt_ids,
	},
};
builtin_platform_driver(k210_rst_driver);
