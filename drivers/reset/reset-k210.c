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

#include <dt-bindings/mfd/k210-sysctl.h>
#include <dt-bindings/reset/k210-rst.h>

struct k210_rst {
	struct regmap *map;
	u32 offset;
	u32 mask;
	u32 assert_high;
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

	if (!(bit & ksr->mask)) {
		dev_err(rcdev->dev, "Invalid assert id %lu\n", id);
		return -EINVAL;
	}

	dev_dbg(rcdev->dev, "assert %s %lu\n",
		ksr->assert_high ? "high" : "low", id);

	regmap_update_bits(ksr->map, ksr->offset, bit,
			   ksr->assert_high ? bit : 0);

	return 0;
}

static inline int k210_rst_deassert(struct reset_controller_dev *rcdev,
				    unsigned long id)
{
	struct k210_rst *ksr = to_k210_rst(rcdev);
	u32 bit = BIT(id);

	if (!(bit & ksr->mask)) {
		dev_err(rcdev->dev, "Invalid deassert id %lu\n", id);
		return -EINVAL;
	}

	dev_dbg(rcdev->dev, "deassert %s %lu\n",
		ksr->assert_high ? "high" : "low", id);

	regmap_update_bits(ksr->map, ksr->offset, bit,
			   ksr->assert_high ? 0 : bit);

	return 0;
}

static int k210_rst_reset(struct reset_controller_dev *rcdev,
			  unsigned long id)
{
	struct k210_rst *ksr = to_k210_rst(rcdev);
	int ret;

	dev_dbg(rcdev->dev, "reset %s %lu\n",
		ksr->assert_high ? "high" : "low", id);

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

	if (!(bit & ksr->mask)) {
		dev_err(rcdev->dev, "Invalid reset %lx\n", id);
		return -EINVAL;
	}

	ret = regmap_read(ksr->map, ksr->offset, &reg);
	if (ret)
		return ret;

	if (ksr->assert_high)
		return ret & bit;

	return !(ret & bit);
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
	int ret, nr_resets;

	dev_info(dev, "K210 reset controller\n");

	ksr = devm_kzalloc(dev, sizeof(*ksr), GFP_KERNEL);
	if (!ksr)
		return -ENOMEM;

	ksr->map = syscon_regmap_lookup_by_phandle(dev->of_node, "regmap");
	if (IS_ERR(ksr->map)) {
		ksr->map = syscon_node_to_regmap(dev->parent->of_node);
		if (IS_ERR(ksr->map)) {
			dev_err(dev, "get register map failed\n");
			return PTR_ERR(ksr->map);
		}
	}

	ret = of_property_read_u32(dev->of_node, "offset", &ksr->offset);
	ret = of_property_read_u32(dev->of_node, "assert-high",
				   &ksr->assert_high);
	if (ret) {
		dev_err(dev, "unable to read 'offset' and 'assert-high'\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(dev->of_node, "mask", &ksr->mask);
	if (ret) {
		/* Use default mask */
		ksr->mask = 0x27FFFFFF;
	}
	nr_resets = fls(ksr->mask);
	if (!nr_resets) {
		dev_err(dev, "Invalid mask 0x%08x\n", ksr->mask);
		return -EINVAL;
	}

	ksr->rcdev.owner = THIS_MODULE;
	ksr->rcdev.dev = dev;
	ksr->rcdev.of_node = dev->of_node;
	ksr->rcdev.nr_resets = nr_resets;
	ksr->rcdev.ops = &k210_rst_ops;

	return devm_reset_controller_register(dev, &ksr->rcdev);
}

static const struct of_device_id k210_rst_dt_ids[] = {
	{ .compatible = "kendryte,k210-rst" },
};

static struct platform_driver k210_rst_driver = {
	.probe	= k210_rst_probe,
	.driver = {
		.name		= "k210-rst",
		.of_match_table	= k210_rst_dt_ids,
	},
};

/*
 * Most devices on the K210 SoC need reset as part of their initialization.
 * So initialize this driver early as part of the post core initialization.
 */
static int __init k210_rst_init(void)
{
	return platform_driver_register(&k210_rst_driver);
}
postcore_initcall(k210_rst_init);
