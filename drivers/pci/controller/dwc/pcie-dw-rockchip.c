// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Rockchip SoCs.
 *
 * Copyright (C) 2021 Rockchip Electronics Co., Ltd.
 *		http://www.rock-chips.com
 *
 * Author: Simon Xue <xxm@rock-chips.com>
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "pcie-designware.h"

/*
 * The upper 16 bits of PCIE_CLIENT_CONFIG are a write
 * mask for the lower 16 bits.
 */
#define HIWORD_UPDATE(mask, val) (((mask) << 16) | (val))
#define HIWORD_UPDATE_BIT(val)	HIWORD_UPDATE(val, val)
#define HIWORD_DISABLE_BIT(val)	HIWORD_UPDATE(val, ~val)

#define to_rockchip_pcie(x) dev_get_drvdata((x)->dev)

#define PCIE_CLIENT_RC_MODE		HIWORD_UPDATE_BIT(0x40)
#define PCIE_CLIENT_EP_MODE		HIWORD_UPDATE(0xf0, 0x0)
#define PCIE_CLIENT_ENABLE_LTSSM	HIWORD_UPDATE_BIT(0xc)
#define PCIE_CLIENT_DISABLE_LTSSM	HIWORD_UPDATE(0x0c, 0x8)
#define PCIE_CLIENT_INTR_STATUS_MISC	0x10
#define PCIE_CLIENT_INTR_MASK_MISC	0x24
#define PCIE_SMLH_LINKUP		BIT(16)
#define PCIE_RDLH_LINKUP		BIT(17)
#define PCIE_LINKUP			(PCIE_SMLH_LINKUP | PCIE_RDLH_LINKUP)
#define PCIE_RDLH_LINK_UP_CHGED		BIT(1)
#define PCIE_L0S_ENTRY			0x11
#define PCIE_CLIENT_GENERAL_CONTROL	0x0
#define PCIE_CLIENT_INTR_STATUS_LEGACY	0x8
#define PCIE_CLIENT_INTR_MASK_LEGACY	0x1c
#define PCIE_CLIENT_GENERAL_DEBUG	0x104
#define PCIE_CLIENT_HOT_RESET_CTRL	0x180
#define PCIE_CLIENT_LTSSM_STATUS	0x300
#define PCIE_LTSSM_ENABLE_ENHANCE	BIT(4)
#define PCIE_LTSSM_STATUS_MASK		GENMASK(5, 0)

#define PCIE_EP_STATE_DISABLED		0
#define PCIE_EP_STATE_ENABLED		1

struct rockchip_pcie {
	struct dw_pcie			pci;
	void __iomem			*apb_base;
	struct phy			*phy;
	struct clk_bulk_data		*clks;
	unsigned int			clk_cnt;
	struct reset_control		*rst;
	struct gpio_desc		*rst_gpio;
	struct regulator                *vpcie3v3;
	struct irq_domain		*irq_domain;
	enum dw_pcie_device_mode	mode;
	unsigned int			perst_irq;
	int				ep_state;
};

struct rockchip_pcie_of_data {
	enum dw_pcie_device_mode mode;
};

static int rockchip_pcie_readl_apb(struct rockchip_pcie *rockchip,
					     u32 reg)
{
	return readl_relaxed(rockchip->apb_base + reg);
}

static void rockchip_pcie_writel_apb(struct rockchip_pcie *rockchip,
						u32 val, u32 reg)
{
	writel_relaxed(val, rockchip->apb_base + reg);
}

static void rockchip_pcie_legacy_int_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct rockchip_pcie *rockchip = irq_desc_get_handler_data(desc);
	unsigned long reg, hwirq;

	chained_irq_enter(chip, desc);

	reg = rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_INTR_STATUS_LEGACY);

	for_each_set_bit(hwirq, &reg, 4)
		generic_handle_domain_irq(rockchip->irq_domain, hwirq);

	chained_irq_exit(chip, desc);
}

static void rockchip_intx_mask(struct irq_data *data)
{
	rockchip_pcie_writel_apb(irq_data_get_irq_chip_data(data),
				 HIWORD_UPDATE_BIT(BIT(data->hwirq)),
				 PCIE_CLIENT_INTR_MASK_LEGACY);
};

static void rockchip_intx_unmask(struct irq_data *data)
{
	rockchip_pcie_writel_apb(irq_data_get_irq_chip_data(data),
				 HIWORD_DISABLE_BIT(BIT(data->hwirq)),
				 PCIE_CLIENT_INTR_MASK_LEGACY);
};

static struct irq_chip rockchip_intx_irq_chip = {
	.name			= "INTx",
	.irq_mask		= rockchip_intx_mask,
	.irq_unmask		= rockchip_intx_unmask,
	.flags			= IRQCHIP_SKIP_SET_WAKE | IRQCHIP_MASK_ON_SUSPEND,
};

static int rockchip_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
				  irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &rockchip_intx_irq_chip, handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops intx_domain_ops = {
	.map = rockchip_pcie_intx_map,
};

static int rockchip_pcie_init_irq_domain(struct rockchip_pcie *rockchip)
{
	struct device *dev = rockchip->pci.dev;
	struct device_node *intc;

	intc = of_get_child_by_name(dev->of_node, "legacy-interrupt-controller");
	if (!intc) {
		dev_err(dev, "missing child interrupt-controller node\n");
		return -EINVAL;
	}

	rockchip->irq_domain = irq_domain_add_linear(intc, PCI_NUM_INTX,
						    &intx_domain_ops, rockchip);
	of_node_put(intc);
	if (!rockchip->irq_domain) {
		dev_err(dev, "failed to get a INTx IRQ domain\n");
		return -EINVAL;
	}

	return 0;
}

static void rockchip_pcie_enable_ltssm(struct rockchip_pcie *rockchip)
{
	rockchip_pcie_writel_apb(rockchip, PCIE_CLIENT_ENABLE_LTSSM,
				 PCIE_CLIENT_GENERAL_CONTROL);
}

static int rockchip_pcie_link_up(struct dw_pcie *pci)
{
	struct rockchip_pcie *rockchip = to_rockchip_pcie(pci);
	u32 val = rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_LTSSM_STATUS);

	if ((val & PCIE_LINKUP) == PCIE_LINKUP &&
	    (val & PCIE_LTSSM_STATUS_MASK) == PCIE_L0S_ENTRY)
		return 1;

	return 0;
}

static void rockchip_pcie_stop_link(struct dw_pcie *pci)
{
	struct rockchip_pcie *rockchip = to_rockchip_pcie(pci);

	disable_irq(rockchip->perst_irq);
}

static int rockchip_pcie_start_link(struct dw_pcie *pci)
{
	struct rockchip_pcie *rockchip = to_rockchip_pcie(pci);

	if (rockchip->mode == DW_PCIE_EP_TYPE) {
		enable_irq(rockchip->perst_irq);
		return 0;
	}

	/* Reset device */
	gpiod_set_value_cansleep(rockchip->rst_gpio, 0);

	rockchip_pcie_enable_ltssm(rockchip);

	/*
	 * PCIe requires the refclk to be stable for 100µs prior to releasing
	 * PERST. See table 2-4 in section 2.6.2 AC Specifications of the PCI
	 * Express Card Electromechanical Specification, 1.1. However, we don't
	 * know if the refclk is coming from RC's PHY or external OSC. If it's
	 * from RC, so enabling LTSSM is the just right place to release #PERST.
	 * We need more extra time as before, rather than setting just
	 * 100us as we don't know how long should the device need to reset.
	 */
	msleep(100);
	gpiod_set_value_cansleep(rockchip->rst_gpio, 1);

	return 0;
}

static int rockchip_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct rockchip_pcie *rockchip = to_rockchip_pcie(pci);
	struct device *dev = rockchip->pci.dev;
	int irq, ret;

	irq = of_irq_get_byname(dev->of_node, "legacy");
	if (irq < 0)
		return irq;

	ret = rockchip_pcie_init_irq_domain(rockchip);
	if (ret < 0)
		dev_err(dev, "failed to init irq domain\n");

	irq_set_chained_handler_and_data(irq, rockchip_pcie_legacy_int_handler,
					 rockchip);

	return 0;
}

static const struct dw_pcie_host_ops rockchip_pcie_host_ops = {
	.host_init = rockchip_pcie_host_init,
};

static void rockchip_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	enum pci_barno bar;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++)
		dw_pcie_ep_reset_bar(pci, bar);
};

static int rockchip_pcie_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				   enum pci_epc_irq_type type,
				   u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return dw_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_EPC_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	case PCI_EPC_IRQ_MSIX:
		return dw_pcie_ep_raise_msix_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "UNKNOWN IRQ type\n");
	}

	return 0;
}

static const struct pci_epc_features rockchip_pcie_epc_features = {
	.linkup_notifier = true,
	.core_init_notifier = true,
	.msi_capable = true,
	.msix_capable = true,
	.align = SZ_64K,
	.reserved_bar = BIT(BAR_4),
	.bar_fixed_size[0] = SZ_1M,
	.bar_fixed_size[1] = SZ_1M,
	.bar_fixed_size[2] = SZ_1M,
	.bar_fixed_size[3] = SZ_1M,
	.bar_fixed_size[5] = SZ_1M,
};

static const struct pci_epc_features *
rockchip_pcie_get_features(struct dw_pcie_ep *ep)
{
	return &rockchip_pcie_epc_features;
}

static const struct dw_pcie_ep_ops rockchip_pcie_ep_ops = {
	.ep_init = rockchip_pcie_ep_init,
	.raise_irq = rockchip_pcie_raise_irq,
	.get_features = rockchip_pcie_get_features,
};

static int rockchip_pcie_clk_init(struct rockchip_pcie *rockchip)
{
	struct device *dev = rockchip->pci.dev;
	int ret;

	ret = devm_clk_bulk_get_all(dev, &rockchip->clks);
	if (ret < 0)
		return ret;

	rockchip->clk_cnt = ret;

	return clk_bulk_prepare_enable(rockchip->clk_cnt, rockchip->clks);
}

static int rockchip_pcie_phy_init(struct rockchip_pcie *rockchip)
{
	struct device *dev = rockchip->pci.dev;
	int ret;

	rockchip->phy = devm_phy_get(dev, "pcie-phy");
	if (IS_ERR(rockchip->phy))
		return dev_err_probe(dev, PTR_ERR(rockchip->phy),
				     "missing PHY\n");

	ret = phy_init(rockchip->phy);
	if (ret < 0)
		return ret;

	ret = phy_power_on(rockchip->phy);
	if (ret)
		phy_exit(rockchip->phy);

	return ret;
}

static void rockchip_pcie_phy_deinit(struct rockchip_pcie *rockchip)
{
	phy_exit(rockchip->phy);
	phy_power_off(rockchip->phy);
}

static int rockchip_perst_assert(struct rockchip_pcie *rockchip)
{
	int ret;

	if (rockchip->ep_state == PCIE_EP_STATE_DISABLED)
		return 0;

	rockchip_pcie_writel_apb(rockchip, PCIE_CLIENT_DISABLE_LTSSM,
				 PCIE_CLIENT_GENERAL_CONTROL);

	ret = reset_control_assert(rockchip->rst);
	if (ret)
		return ret;

	clk_bulk_disable_unprepare(rockchip->clk_cnt, rockchip->clks);

	rockchip_pcie_phy_deinit(rockchip);

	if (rockchip->vpcie3v3)
		regulator_disable(rockchip->vpcie3v3);

	rockchip->ep_state = PCIE_EP_STATE_DISABLED;

	return 0;
}

static int rockchip_perst_deassert(struct rockchip_pcie *rockchip)
{
	int ret;
	u32 val;

	if (rockchip->ep_state == PCIE_EP_STATE_ENABLED)
		return 0;

	//TODO: verify that all core registers are the same here as after probe

	//also things like unmasked IRQs, ENHANCED LTSSM, EP mode, etc...

	//probably need to do a core reset here... (enable_resources())
	if (rockchip->vpcie3v3) {
		ret = regulator_enable(rockchip->vpcie3v3);
		if (ret)
			return ret; //TODO: goto
	}

	ret = rockchip_pcie_phy_init(rockchip);
	if (ret)
		return ret; //TODO: goto

	ret = reset_control_deassert(rockchip->rst);
	if (ret)
		return ret; //TODO: goto

	ret = rockchip_pcie_clk_init(rockchip);
	if (ret)
		return ret;

	/* LTSSM enable control mode */
	val = HIWORD_UPDATE_BIT(PCIE_LTSSM_ENABLE_ENHANCE);
	rockchip_pcie_writel_apb(rockchip, val, PCIE_CLIENT_HOT_RESET_CTRL);

	/* unmask DLL up/down indicator and Hot reset/link-down reset */
	rockchip_pcie_writel_apb(rockchip, 0x60000, PCIE_CLIENT_INTR_MASK_MISC);

	rockchip_pcie_writel_apb(rockchip, PCIE_CLIENT_EP_MODE,
				 PCIE_CLIENT_GENERAL_CONTROL);

	dw_pcie_ep_reset_bar(&rockchip->pci, BAR_4);

	ret = dw_pcie_ep_init_complete(&rockchip->pci.ep);
	if (ret) {
		//dev_err(dev, "Failed to complete initialization: %d\n", ret);
		pr_err("Failed to complete initialization: %d\n", ret);
		//TODO: goto error
		return ret;
	}

	dw_pcie_ep_init_notify(&rockchip->pci.ep);

	rockchip_pcie_enable_ltssm(rockchip);

	rockchip->ep_state = PCIE_EP_STATE_ENABLED;

	return 0;
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.link_up = rockchip_pcie_link_up,
	.start_link = rockchip_pcie_start_link,
	.stop_link = rockchip_pcie_stop_link,
};

static irqreturn_t rockchip_pcie_sys_irq_handler(int irq, void *arg)
{
	struct rockchip_pcie *rockchip = arg;
	struct dw_pcie *pci = &rockchip->pci;
	u32 reg;
	u32 val;

	reg = rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_INTR_STATUS_MISC);
	pr_err("pci: PCIE_CLIENT_INTR_STATUS_MISC: %#x\n", reg);
	pr_err("LTSSM_STATUS: %#x\n",
	       rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_LTSSM_STATUS));

	if (reg & BIT(2)) {
		val = rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_LTSSM_STATUS);
		pr_err("pci: hot reset or link-down reset (LTSSM_STATUS: %#x)\n", val);
		pr_err("pci: if you see this, then reboot the box with the RC\n");
		pr_err("pci: all BAR addresses assigned by RC will be cleared\n");
		pr_err("pci: and there is no way that the EP can restore them\n");
	}

	if (reg & PCIE_RDLH_LINK_UP_CHGED) {
		val = rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_LTSSM_STATUS);
		if ((val & PCIE_LINKUP) == PCIE_LINKUP) {
			dw_pcie_ep_linkup(&pci->ep);
			pr_err("pci: link up! (LTSSM_STATUS: %#x)\n", val);
		}
	}

	rockchip_pcie_writel_apb(rockchip, reg, PCIE_CLIENT_INTR_STATUS_MISC);

	return IRQ_HANDLED;
}

static irqreturn_t rockchip_pcie_ep_perst_irq_thread(int irq, void *data)
{
	struct rockchip_pcie *rockchip = data;
	int perst;

	perst = gpiod_get_value(rockchip->rst_gpio);
	if (perst) {
		pr_err("pci: PERST asserted by host. Shutting down the PCIe link!\n");
		rockchip_perst_assert(rockchip);
	} else {
		pr_err("pci: PERST de-asserted by host. Starting link training!\n");
		rockchip_perst_deassert(rockchip);
	}

	return IRQ_HANDLED;
}

static int rockchip_pcie_resource_get(struct platform_device *pdev,
				      struct rockchip_pcie *rockchip)
{
	rockchip->apb_base = devm_platform_ioremap_resource_byname(pdev, "apb");
	if (IS_ERR(rockchip->apb_base))
		return PTR_ERR(rockchip->apb_base);

	if (rockchip->mode == DW_PCIE_RC_TYPE)
		rockchip->rst_gpio = devm_gpiod_get_optional(&pdev->dev, "reset",
							     GPIOD_OUT_HIGH);
	else if (rockchip->mode == DW_PCIE_EP_TYPE)
		rockchip->rst_gpio = devm_gpiod_get_optional(&pdev->dev, "reset",
							     GPIOD_IN);
	if (IS_ERR(rockchip->rst_gpio))
		return PTR_ERR(rockchip->rst_gpio);

	if (rockchip->mode == DW_PCIE_EP_TYPE) {
		int ret;

		ret = gpiod_to_irq(rockchip->rst_gpio);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to get IRQ for PERST GPIO: %d\n", ret);
			return ret;
		}
		rockchip->perst_irq = (unsigned int)ret;

		irq_set_status_flags(rockchip->perst_irq, IRQ_NOAUTOEN);

		rockchip->ep_state = PCIE_EP_STATE_DISABLED;

		ret = devm_request_threaded_irq(&pdev->dev, rockchip->perst_irq, NULL,
						rockchip_pcie_ep_perst_irq_thread,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"perst_irq", rockchip);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to request IRQ for PERST: %d\n", ret);
			return ret;
		}
	}

	rockchip->rst = devm_reset_control_array_get_exclusive(&pdev->dev);
	if (IS_ERR(rockchip->rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(rockchip->rst),
				     "failed to get reset lines\n");

	return 0;
}

static int rockchip_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rockchip_pcie *rockchip;
	struct dw_pcie_rp *pp;
	const struct rockchip_pcie_of_data *data;
	enum dw_pcie_device_mode mode;
	u32 val;
	int ret;
	int irq;

	data = of_device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	mode = (enum dw_pcie_device_mode)data->mode;

	rockchip = devm_kzalloc(dev, sizeof(*rockchip), GFP_KERNEL);
	if (!rockchip)
		return -ENOMEM;

	platform_set_drvdata(pdev, rockchip);

	rockchip->pci.dev = dev;
	rockchip->pci.ops = &dw_pcie_ops;
	rockchip->mode = mode;

	ret = rockchip_pcie_resource_get(pdev, rockchip);
	if (ret)
		return ret;

	ret = reset_control_assert(rockchip->rst);
	if (ret)
		return ret;

	/* DON'T MOVE ME: must be enable before PHY init */
	rockchip->vpcie3v3 = devm_regulator_get_optional(dev, "vpcie3v3");
	if (IS_ERR(rockchip->vpcie3v3)) {
		if (PTR_ERR(rockchip->vpcie3v3) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(rockchip->vpcie3v3),
					"failed to get vpcie3v3 regulator\n");
		rockchip->vpcie3v3 = NULL;
	} else {
		ret = regulator_enable(rockchip->vpcie3v3);
		if (ret) {
			dev_err(dev, "failed to enable vpcie3v3 regulator\n");
			return ret;
		}
	}

	ret = rockchip_pcie_phy_init(rockchip);
	if (ret)
		goto disable_regulator;

	ret = reset_control_deassert(rockchip->rst);
	if (ret)
		goto deinit_phy;

	ret = rockchip_pcie_clk_init(rockchip);
	if (ret)
		goto deinit_phy;

	/* LTSSM enable control mode */
	val = HIWORD_UPDATE_BIT(PCIE_LTSSM_ENABLE_ENHANCE);
	rockchip_pcie_writel_apb(rockchip, val, PCIE_CLIENT_HOT_RESET_CTRL);

	// TODO: either move this code do a add_ep() function, or if mode == EP
	irq = platform_get_irq_byname(pdev, "sys");
	if (irq < 0) {
		dev_err(dev, "missing sys IRQ resource\n");
		return -EINVAL;
	}

	ret = devm_request_irq(dev, irq, rockchip_pcie_sys_irq_handler,
			       IRQF_SHARED, "pcie-sys", rockchip);
	if (ret) {
		dev_err(dev, "failed to request PCIe sys IRQ\n");
		return ret;
	}
	/* unmask DLL up/down indicator and Hot reset/link-down reset */
	rockchip_pcie_writel_apb(rockchip, 0x60000, PCIE_CLIENT_INTR_MASK_MISC);

	switch (rockchip->mode) {
	case DW_PCIE_RC_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_ROCKCHIP_DW_HOST)) {
			ret = -ENODEV;
			goto deinit_clk;
		}

		rockchip_pcie_writel_apb(rockchip, PCIE_CLIENT_RC_MODE,
					 PCIE_CLIENT_GENERAL_CONTROL);

		pp = &rockchip->pci.pp;
		pp->ops = &rockchip_pcie_host_ops;

		ret = dw_pcie_host_init(pp);
		if (!ret)
			return 0;
		break;
	case DW_PCIE_EP_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_ROCKCHIP_DW_EP)) {
			ret = -ENODEV;
			goto deinit_clk;
		}

		rockchip_pcie_writel_apb(rockchip, PCIE_CLIENT_EP_MODE,
					 PCIE_CLIENT_GENERAL_CONTROL);

		rockchip->pci.ep.ops = &rockchip_pcie_ep_ops;
		rockchip->pci.ep.page_size = SZ_64K;

		dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));

		return dw_pcie_ep_init(&rockchip->pci.ep);
	default:
		dev_err(dev, "INVALID device type %d\n", rockchip->mode);
		ret = -EINVAL;
		goto deinit_clk;
	}

deinit_clk:
	clk_bulk_disable_unprepare(rockchip->clk_cnt, rockchip->clks);
deinit_phy:
	rockchip_pcie_phy_deinit(rockchip);
disable_regulator:
	if (rockchip->vpcie3v3)
		regulator_disable(rockchip->vpcie3v3);

	return ret;
}

static const struct rockchip_pcie_of_data rk3568_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
};

static const struct rockchip_pcie_of_data rk3588_pcie_ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
};

static const struct of_device_id rockchip_pcie_of_match[] = {
	{
		.compatible = "rockchip,rk3568-pcie",
		.data = &rk3568_pcie_rc_of_data,
	},
	{
		.compatible = "rockchip,rk3588-pcie",
		.data = &rk3568_pcie_rc_of_data,
	},
	{
		.compatible = "rockchip,rk3588-pcie-ep",
		.data = &rk3588_pcie_ep_of_data,
	},
	{},
};

static struct platform_driver rockchip_pcie_driver = {
	.driver = {
		.name	= "rockchip-dw-pcie",
		.of_match_table = rockchip_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = rockchip_pcie_probe,
};
builtin_platform_driver(rockchip_pcie_driver);
