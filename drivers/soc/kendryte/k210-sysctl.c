// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2019 Christoph Hellwig.
 * Copyright (C) 2019-20 Sean Anderson <seanga2@gmail.com>
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 */
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <asm/soc.h>

#include "k210-sysctl.h"

static int k210_sysctl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	dev_info(dev, "K210 system controller driver\n");

	/* Populate children */
	ret = devm_of_platform_populate(dev);
	if (ret)
		dev_err(dev, "Populate platform failed %d\n", ret);

	return ret;
}

static const struct of_device_id k210_sysctl_of_match[] = {
	{ .compatible = "kendryte,k210-sysctl", },
	{}
};

static struct platform_driver k210_sysctl_driver = {
	.driver	= {
		.name		= "k210-sysctl",
		.of_match_table	= k210_sysctl_of_match,
	},
	.probe			= k210_sysctl_probe,
};
builtin_platform_driver(k210_sysctl_driver);

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
SOC_EARLY_INIT_DECLARE(generic_k210, "kendryte,k210", k210_soc_early_init);

#ifdef CONFIG_SOC_KENDRYTE_K210_DTB_BUILTIN
/*
 * Generic entry for the default k210.dtb embedded DTB for boards with:
 *   - Vendor ID: 0x4B5
 *   - Arch ID: 0xE59889E6A5A04149 (= "Canaan AI" in UTF-8 encoded Chinese)
 *   - Impl ID:	0x4D41495832303030 (= "MAIX2000")
 * These values are reported by the SiPEED MAXDUINO, SiPEED MAIX GO and
 * SiPEED Dan dock boards.
 */
SOC_BUILTIN_DTB_DECLARE(k210, 0x4B5, 0xE59889E6A5A04149, 0x4D41495832303030);
#endif
