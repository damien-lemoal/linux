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
#include <linux/phy/pcie.h>
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
#define PCIE_EP_STATE_LINK_UP		2

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

static void rockchip_pcie_disable_ltssm(struct rockchip_pcie *rockchip)
{
	rockchip_pcie_writel_apb(rockchip, PCIE_CLIENT_DISABLE_LTSSM,
				 PCIE_CLIENT_GENERAL_CONTROL);
}

static int rockchip_pcie_link_up(struct dw_pcie *pci)
{
	struct rockchip_pcie *rockchip = to_rockchip_pcie(pci);
	u32 val;

	/* Add this 1ms delay to ensure the link is stable. */
	usleep_range(1000, 1100);

	val = rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_LTSSM_STATUS);
	if ((val & PCIE_LINKUP) == PCIE_LINKUP &&
	    (val & PCIE_LTSSM_STATUS_MASK) == PCIE_L0S_ENTRY)
		return 1;

	return 0;
}

static void rockchip_pcie_stop_link(struct dw_pcie *pci)
{
	struct rockchip_pcie *rockchip = to_rockchip_pcie(pci);

	rockchip_pcie_disable_ltssm(rockchip);

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
				   unsigned int type, u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_IRQ_LEGACY:
		return dw_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	case PCI_IRQ_MSIX:
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

	ret = clk_bulk_prepare_enable(rockchip->clk_cnt, rockchip->clks);
	if (ret) {
		dev_err(dev, "failed to prepare enable pcie bulk clks: %d\n",
			ret);
		return ret;
	}

	return 0;
}

static int rockchip_pcie_phy_init(struct rockchip_pcie *rockchip)
{
	struct device *dev = rockchip->pci.dev;
	int ret, phy_sub_mode;

	rockchip->phy = devm_phy_get(dev, "pcie-phy");
	if (IS_ERR(rockchip->phy))
		return dev_err_probe(dev, PTR_ERR(rockchip->phy),
				     "missing PHY\n");

	/* This must be done before PHY init */
	if (rockchip->vpcie3v3) {
		ret = regulator_enable(rockchip->vpcie3v3);
		if (ret) {
			dev_err(dev, "Enable regulator failed %d\n", ret);
			return ret;
		}
	}

	switch (rockchip->mode) {
	case DW_PCIE_RC_TYPE:
		phy_sub_mode = PHY_MODE_PCIE_RC;
		break;
	case DW_PCIE_EP_TYPE:
		phy_sub_mode = PHY_MODE_PCIE_EP;
		break;
	default:
		dev_err(dev, "Invalid device type %d\n", rockchip->mode);
		return -EINVAL;
	}

	ret = phy_init(rockchip->phy);
	if (ret < 0) {
		dev_err(dev, "phy init failed %d\n", ret);
		goto disable_regulator;
	}

	ret = phy_set_mode_ext(rockchip->phy, PHY_MODE_PCIE, phy_sub_mode);
	if (ret) {
		dev_err(dev, "fail to set phy to mode %s, err %d\n",
			(phy_sub_mode == PHY_MODE_PCIE_RC) ? "RC" : "EP",
			ret);
		goto phy_exit;
	}

	ret = phy_power_on(rockchip->phy);
	if (ret) {
		dev_err(dev, "phy power on failed %d\n", ret);
		goto phy_exit;
	}

	return 0;

phy_exit:
	phy_exit(rockchip->phy);

disable_regulator:
	if (rockchip->vpcie3v3)
		regulator_disable(rockchip->vpcie3v3);

	return ret;
}

static void rockchip_pcie_phy_deinit(struct rockchip_pcie *rockchip)
{
	phy_power_off(rockchip->phy);
	phy_exit(rockchip->phy);

	if (rockchip->vpcie3v3)
		regulator_disable(rockchip->vpcie3v3);
}

static int rockchip_pcie_enable_resources(struct rockchip_pcie *rockchip,
					  bool probe)
{
	struct device *dev = rockchip->pci.dev;
	int ret;

	if (probe) {
		ret = reset_control_assert(rockchip->rst);
		if (ret) {
			dev_err(dev, "reset control assert failed %d\n", ret);
			return ret;
		}
	}

	ret = rockchip_pcie_phy_init(rockchip);
	if (ret)
		return ret;

	usleep_range(1000, 1005);

	ret = reset_control_deassert(rockchip->rst);
	if (ret) {
		dev_err(dev, "reset control deassert failed %d\n", ret);
		goto phy_deinit;
	}

	ret = rockchip_pcie_clk_init(rockchip);
	if (ret) {
		dev_err(dev, "clk init failed %d\n", ret);
		goto phy_deinit;
	}

	return 0;

phy_deinit:
	rockchip_pcie_phy_deinit(rockchip);

	return ret;
}

static void rockchip_pcie_disable_resources(struct rockchip_pcie *rockchip)
{
	reset_control_assert(rockchip->rst);
	rockchip_pcie_phy_deinit(rockchip);
	clk_bulk_disable_unprepare(rockchip->clk_cnt, rockchip->clks);
}

static int rockchip_perst_assert(struct rockchip_pcie *rockchip)
{
	struct device *dev = rockchip->pci.dev;

	dev_info(dev, "PERST asserted\n");

	if (rockchip->ep_state == PCIE_EP_STATE_DISABLED)
		return 0;

	dev_info(dev, "Shutting down the link\n");

	rockchip_pcie_disable_ltssm(rockchip);
	rockchip_pcie_disable_resources(rockchip);

	rockchip->ep_state = PCIE_EP_STATE_DISABLED;

	return 0;
}

static void rockchip_pcie_reset_bars(struct rockchip_pcie *rockchip)
{
	struct dw_pcie *pci = &rockchip->pci;
	unsigned int offset, nbars;
	u32 val;
	int i;

	offset = dw_pcie_find_ext_capability(pci, PCI_EXT_CAP_ID_REBAR);
	val = dw_pcie_readl_dbi(pci, offset + PCI_REBAR_CTRL);
	nbars = (val & PCI_REBAR_CTRL_NBAR_MASK) >> PCI_REBAR_CTRL_NBAR_SHIFT;

	for (i = 0; i < nbars; i++, offset += PCI_REBAR_CTRL)
		dw_pcie_writel_dbi(pci, offset + PCI_REBAR_CTRL, 0x0);

	/* Setup command register */
	val = dw_pcie_readl_dbi(pci, PCI_COMMAND);
	val &= 0xffff0000;
	val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
	dw_pcie_writel_dbi(pci, PCI_COMMAND, val);
}

static int rockchip_perst_deassert(struct rockchip_pcie *rockchip)
{
	struct device *dev = rockchip->pci.dev;
	int ret;
	u32 val;

	dev_info(dev, "PERST de-asserted\n");

	if (rockchip->ep_state != PCIE_EP_STATE_DISABLED) {
		dev_info(dev, "Bad state %d\n", rockchip->ep_state);
		return 0;
	}

	dev_info(dev, "Starting link training\n");

	ret = rockchip_pcie_enable_resources(rockchip, false);
	if (ret)
		return ret;

	/* Assert WAKE# to RC to indicate device is ready */
	gpiod_set_value_cansleep(rockchip->rst_gpio, 1);
	usleep_range(2000, 2500);
	gpiod_set_value_cansleep(rockchip->rst_gpio, 0);

	/* LTSSM enable control mode */
	val = HIWORD_UPDATE_BIT(PCIE_LTSSM_ENABLE_ENHANCE);
	rockchip_pcie_writel_apb(rockchip, val, PCIE_CLIENT_HOT_RESET_CTRL);

	/* unmask DLL up/down indicator and Hot reset/link-down reset */
	rockchip_pcie_writel_apb(rockchip, 0x60000, PCIE_CLIENT_INTR_MASK_MISC);

	/* Configure PCIe to endpoint mode */
	rockchip_pcie_writel_apb(rockchip, PCIE_CLIENT_EP_MODE,
				 PCIE_CLIENT_GENERAL_CONTROL);

	dw_pcie_ep_reset_bar(&rockchip->pci, BAR_4);

	ret = dw_pcie_ep_init_complete(&rockchip->pci.ep);
	if (ret) {
		dev_err(dev, "Failed to complete initialization: %d\n", ret);
		goto disable_resources;
	}

	dw_pcie_ep_init_notify(&rockchip->pci.ep);

	rockchip->ep_state = PCIE_EP_STATE_ENABLED;

	rockchip_pcie_enable_ltssm(rockchip);

	return 0;

disable_resources:
	rockchip_pcie_disable_resources(rockchip);
	rockchip->ep_state = PCIE_EP_STATE_DISABLED;

	return ret;
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
	struct device *dev = pci->dev;
	u32 reg;
	u32 val;

	reg = rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_INTR_STATUS_MISC);

	dev_info(dev, "pci: PCIE_CLIENT_INTR_STATUS_MISC: %#x\n", reg);

	val = rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_LTSSM_STATUS);
	dev_info(dev, "    LTSSM_STATUS: %#x\n", val);
	dev_info(dev, "    PHY is %s\n",
		 (val & PCIE_SMLH_LINKUP) ? "UP" : "DOWN");
	dev_info(dev, "    DATA link is %s\n",
		 (val & PCIE_RDLH_LINKUP) ? "UP" : "DOWN");

	if (reg & BIT(2)) {
		dev_info(dev,
			 "Hot reset or link-down reset (LTSSM_STATUS: %#x)\n",
			 rockchip_pcie_readl_apb(rockchip,
						 PCIE_CLIENT_LTSSM_STATUS));

		rockchip_pcie_reset_bars(rockchip);
	}

	if (reg & PCIE_RDLH_LINK_UP_CHGED) {
		if (rockchip->ep_state != PCIE_EP_STATE_ENABLED) {
			dev_info(dev,
				 "Link changed (LTSSM_STATUS: %#x)\n", val);
			goto out;
		}

		val = rockchip_pcie_readl_apb(rockchip,
					      PCIE_CLIENT_LTSSM_STATUS);
		if ((val & PCIE_LINKUP) == PCIE_LINKUP) {
			dev_info(dev, "Link UP (LTSSM_STATUS: %#x)\n", val);
			dw_pcie_ep_linkup(&pci->ep);
			rockchip->ep_state = PCIE_EP_STATE_LINK_UP;
		}
	}

out:
	rockchip_pcie_writel_apb(rockchip, reg, PCIE_CLIENT_INTR_STATUS_MISC);

	return IRQ_HANDLED;
}

static irqreturn_t rockchip_pcie_ep_perst_irq_thread(int irq, void *data)
{
	struct rockchip_pcie *rockchip = data;

	if (gpiod_get_value(rockchip->rst_gpio))
		rockchip_perst_assert(rockchip);
	else
		rockchip_perst_deassert(rockchip);

	return IRQ_HANDLED;
}

static int rockchip_pcie_resource_get(struct platform_device *pdev,
				      struct rockchip_pcie *rockchip)
{
	struct device *dev = &pdev->dev;

	rockchip->apb_base = devm_platform_ioremap_resource_byname(pdev, "apb");
	if (IS_ERR(rockchip->apb_base))
		return PTR_ERR(rockchip->apb_base);

	if (rockchip->mode == DW_PCIE_RC_TYPE)
		rockchip->rst_gpio = devm_gpiod_get_optional(dev, "reset",
							     GPIOD_OUT_HIGH);
	else if (rockchip->mode == DW_PCIE_EP_TYPE)
		rockchip->rst_gpio = devm_gpiod_get_optional(dev, "reset",
							     GPIOD_IN);
	if (IS_ERR(rockchip->rst_gpio))
		return PTR_ERR(rockchip->rst_gpio);

	if (rockchip->mode == DW_PCIE_EP_TYPE) {
		int ret;

		ret = gpiod_to_irq(rockchip->rst_gpio);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to get IRQ for PERST GPIO: %d\n", ret);
			return ret;
		}
		rockchip->perst_irq = (unsigned int)ret;

		irq_set_status_flags(rockchip->perst_irq, IRQ_NOAUTOEN);

		rockchip->ep_state = PCIE_EP_STATE_DISABLED;

		ret = devm_request_threaded_irq(dev, rockchip->perst_irq, NULL,
						rockchip_pcie_ep_perst_irq_thread,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"perst_irq", rockchip);
		if (ret < 0) {
			dev_err(dev,
				"Failed to request IRQ for PERST: %d\n", ret);
			return ret;
		}
	}

	rockchip->rst = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(rockchip->rst))
		return dev_err_probe(dev, PTR_ERR(rockchip->rst),
				     "failed to get reset lines\n");

	rockchip->vpcie3v3 = devm_regulator_get_optional(dev, "vpcie3v3");
	if (IS_ERR(rockchip->vpcie3v3)) {
		if (PTR_ERR(rockchip->vpcie3v3) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(rockchip->vpcie3v3),
					"failed to get vpcie3v3 regulator\n");
		rockchip->vpcie3v3 = NULL;
		dev_info(dev, "No vpcie3v3 regulator\n");
	}

	return 0;
}

static int rockchip_pcie_probe_host(struct rockchip_pcie *rockchip)
{
	struct device *dev = rockchip->pci.dev;

	if (!IS_ENABLED(CONFIG_PCIE_ROCKCHIP_DW_HOST))
		return -ENODEV;

	rockchip->pci.pp.ops = &rockchip_pcie_host_ops;

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));

	rockchip_pcie_writel_apb(rockchip, PCIE_CLIENT_RC_MODE,
				 PCIE_CLIENT_GENERAL_CONTROL);

	return dw_pcie_host_init(&rockchip->pci.pp);
}

static int rockchip_pcie_probe_ep(struct rockchip_pcie *rockchip)
{
	struct device *dev = rockchip->pci.dev;

	if (!IS_ENABLED(CONFIG_PCIE_ROCKCHIP_DW_EP))
		return -ENODEV;

	rockchip->pci.ep.ops = &rockchip_pcie_ep_ops;
	rockchip->pci.ep.page_size = SZ_64K;

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));

	rockchip_pcie_writel_apb(rockchip, PCIE_CLIENT_EP_MODE,
				 PCIE_CLIENT_GENERAL_CONTROL);

	return dw_pcie_ep_init(&rockchip->pci.ep);
}

static int rockchip_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rockchip_pcie *rockchip;
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

	ret = rockchip_pcie_enable_resources(rockchip, true);
	if (ret)
		return ret;

	/* LTSSM enable control mode */
	val = HIWORD_UPDATE_BIT(PCIE_LTSSM_ENABLE_ENHANCE);
	rockchip_pcie_writel_apb(rockchip, val, PCIE_CLIENT_HOT_RESET_CTRL);

	if (rockchip->mode == DW_PCIE_EP_TYPE) {
		irq = platform_get_irq_byname(pdev, "sys");
		if (irq < 0) {
			dev_err(dev, "Missing sys IRQ resource\n");
			ret = -EINVAL;
			goto disable_resources;
		}
	}

	ret = devm_request_threaded_irq(dev, irq, rockchip_pcie_sys_irq_handler,
					NULL, IRQF_SHARED,
					"pcie-sys", rockchip);
	if (ret) {
		dev_err(dev, "Failed to request PCIe sys IRQ\n");
		goto disable_resources;
	}

	/* unmask DLL up/down indicator and Hot reset/link-down reset */
	rockchip_pcie_writel_apb(rockchip, 0x60000, PCIE_CLIENT_INTR_MASK_MISC);

	switch (rockchip->mode) {
	case DW_PCIE_RC_TYPE:
		ret = rockchip_pcie_probe_host(rockchip);
		break;
	case DW_PCIE_EP_TYPE:
		ret = rockchip_pcie_probe_ep(rockchip);
		break;
	default:
		dev_err(dev, "Invalid device type %d\n", rockchip->mode);
		ret = -EINVAL;
		goto disable_resources;
	}

	return 0;

disable_resources:
	rockchip_pcie_disable_resources(rockchip);

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
