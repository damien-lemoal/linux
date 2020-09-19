// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2019 Christoph Hellwig.
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 */
#include <linux/platform_device.h>
#include <linux/io.h>
#include <asm/soc.h>

#include "k210-sysctl.h"

static int k210_sysctl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_info(dev, "Kendryte K210 SoC system controller\n");

	return 0;
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

static int __init k210_sysctl_init(void)
{
	return platform_driver_register(&k210_sysctl_driver);
}
core_initcall(k210_sysctl_init);

/*
 * This needs to be called very early during initialization, given that
 * PLL1 needs to be enabled to be able to use all SRAM.
 */
static void __init k210_soc_early_init(const void *fdt)
{
	void __iomem *sysctl_base = NULL;

	sysctl_base =
		ioremap(K210_SYSCTL_BASE_ADDR, K210_SYSCTL_BASE_SIZE);
	if (!sysctl_base)
		panic("k210-sysctl: ioremap failed");

	/* Enable PLL1 to make the KPU SRAM useable */
	k210_enable_pll1(sysctl_base);

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
