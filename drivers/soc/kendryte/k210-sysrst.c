// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include <dt-bindings/mfd/k210-sysctl.h>
#include <dt-bindings/reset/k210-sysctl.h>
#include "k210-sysctl.h"

struct k210_sysrst {
	spinlock_t			lock;
	void __iomem			*regs;
	struct reset_controller_dev	rcdev;
};

static inline struct k210_sysrst *
to_k210_sysrst(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct k210_sysrst, rcdev);
}

static void k210_sysrst_update(struct reset_controller_dev *rcdev,
			       unsigned long id, bool assert)
{
	struct k210_sysrst *ksr = to_k210_sysrst(rcdev);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&ksr->lock, flags);

	reg = readl(ksr->regs);
	if (assert)
		reg |= BIT(id);
	else
		reg &= ~BIT(id);
	writel(reg, ksr->regs);

	spin_unlock_irqrestore(&ksr->lock, flags);
}

static int k210_sysrst_assert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	k210_sysrst_update(rcdev, id, true);

	return 0;
}

static int k210_sysrst_deassert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	k210_sysrst_update(rcdev, id, false);

	return 0;
}

static int k210_sysrst_reset(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	k210_sysrst_assert(rcdev, id);
	udelay(10);
	k210_sysrst_deassert(rcdev, id);

	return 0;
}

static int k210_sysrst_status(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct k210_sysrst *ksr = to_k210_sysrst(rcdev);

	if (!(BIT(id) & K210_RST_MASK))
		return -EINVAL;

	return readl(ksr->regs) & BIT(id);
}

static const struct reset_control_ops k210_sysrst_ops = {
	.assert		= k210_sysrst_assert,
	.deassert	= k210_sysrst_deassert,
	.reset		= k210_sysrst_reset,
	.status		= k210_sysrst_status,
};

static int k210_sysrst_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct k210_sysrst *ksr;

	dev_info(dev, "K210 reset controller\n");

	ksr = devm_kzalloc(dev, sizeof(*ksr), GFP_KERNEL);
	if (!ksr)
		return -ENOMEM;

	spin_lock_init(&ksr->lock);
	ksr->regs = ioremap(K210_SYSCTL_BASE_ADDR + K210_SYSCTL_PERI_RESET, 4);
	if (!ksr->regs) {
		dev_err(dev, "map sysctl resources failed\n");
		return -ENODEV;
	}

	ksr->rcdev.owner = THIS_MODULE;
	ksr->rcdev.of_node = dev->of_node;
	ksr->rcdev.nr_resets = K210_RST_RTC + 1;
	ksr->rcdev.ops = &k210_sysrst_ops;

	return devm_reset_controller_register(dev, &ksr->rcdev);
}

static const struct of_device_id k210_sysrst_dt_ids[] = {
	{ .compatible = "kendryte,k210-sysrst" },
};

static struct platform_driver k210_sysrst_driver = {
	.probe	= k210_sysrst_probe,
	.driver = {
		.name		= "k210-sysrst",
		.of_match_table	= k210_sysrst_dt_ids,
	},
};
builtin_platform_driver(k210_sysrst_driver);
