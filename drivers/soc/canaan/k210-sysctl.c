// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2019 Christoph Hellwig.
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 */
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <asm/soc.h>

#include <soc/canaan/k210-sysctl.h>

static int __init k210_sysctl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk *pclk;
	int ret;

	dev_info(dev, "K210 system controller\n");

	/* Get power bus clock */
	pclk = devm_clk_get(dev, NULL);
	if (IS_ERR(pclk)) {
		dev_err(dev, "Get bus clock failed\n");
		return PTR_ERR(pclk);
	}

	ret = clk_prepare_enable(pclk);
	if (ret) {
		dev_err(dev, "Enable bus clock failed\n");
		return ret;
	}

	/* Populate children */
	ret = devm_of_platform_populate(dev);
	if (ret)
		dev_err(dev, "Populate platform failed %d\n", ret);

	return ret;
}

static const struct of_device_id k210_sysctl_of_match[] = {
	{ .compatible = "canaan,k210-sysctl", },
	{}
};

static struct platform_driver k210_sysctl_driver = {
	.driver	= {
		.name		= "k210-sysctl",
		.of_match_table	= k210_sysctl_of_match,
	},
	.probe			= k210_sysctl_probe,
};

/*
 * Most devices on the K210 SoC depend on the early initialization of sysctl
 * fpioa and reset child nodes. So initialize this driver early as part of
 * the post core initialization.
 */
static int __init k210_sysctl_init(void)
{
	return platform_driver_register(&k210_sysctl_driver);
}
postcore_initcall(k210_sysctl_init);

/*
 * System controller registers base address and size.
 */
#define K210_SYSCTL_BASE_ADDR	0x50440000ULL
#define K210_SYSCTL_BASE_SIZE	0x1000

/*
 * This needs to be called very early during initialization, given that
 * PLL1 needs to be enabled to be able to use all SRAM.
 */
static void __init k210_soc_early_init(const void *fdt)
{
	void __iomem *sysctl_base;

	sysctl_base = ioremap(K210_SYSCTL_BASE_ADDR, K210_SYSCTL_BASE_SIZE);
	if (!sysctl_base)
		panic("k210-sysctl: ioremap failed");

	k210_clk_early_init(sysctl_base);

	iounmap(sysctl_base);
}
SOC_EARLY_INIT_DECLARE(k210_soc, "canaan,kendryte-k210", k210_soc_early_init);
