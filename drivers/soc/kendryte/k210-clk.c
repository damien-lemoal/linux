// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2019 Christoph Hellwig.
 * Copyright (C) 2019-20 Sean Anderson <seanga2@gmail.com>
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 */
#define pr_fmt(fmt)     "k210-clk: " fmt

#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/bitfield.h>
#include <asm/soc.h>
#include <asm/delay.h>

#include <dt-bindings/clock/k210-sysctl.h>
#include <dt-bindings/mfd/k210-sysctl.h>

#include "k210-sysctl.h"

/*
 * in0: fixed-rate 26MHz oscillator base clock.
 */
#define K210_IN0_RATE		26000000UL

/*
 * Clocks parameters.
 */
struct k210_clk_cfg {
	u8 gate_reg;
	u8 gate_bit;
	u8 div_reg;
	u8 div_shift;
	u8 div_width;
	u8 div_type;
	u8 mux_reg;
	u8 mux_bit;
};

enum k210_clk_div_type {
	DIV_ONE_BASED,
	DIV_DOUBLE_ONE_BASED,
	DIV_POWER_OF_TWO,
};

#define GATE(_reg, _bit)	\
	.gate_reg = (_reg),	\
	.gate_bit = (_bit)
#define DIV(_reg, _shift, _width, _type)	\
	.div_reg = (_reg),			\
	.div_shift = (_shift),			\
	.div_width = (_width),			\
	.div_type = (_type)
#define MUX(_reg, _bit)		\
	.mux_reg = (_reg),	\
	.mux_bit = (_bit)

static const struct k210_clk_cfg k210_clks[K210_NUM_CLKS] = {

	/* Gated clocks, no mux, no divider */
	[K210_CLK_CPU] = { GATE(K210_SYSCTL_EN_CENT, 0) },
	[K210_CLK_DMA] = { GATE(K210_SYSCTL_EN_PERI, 1) },
	[K210_CLK_FFT] = { GATE(K210_SYSCTL_EN_PERI, 4) },
	[K210_CLK_GPIO] = { GATE(K210_SYSCTL_EN_PERI, 5) },
	[K210_CLK_UART1] = { GATE(K210_SYSCTL_EN_PERI, 16) },
	[K210_CLK_UART2] = { GATE(K210_SYSCTL_EN_PERI, 17) },
	[K210_CLK_UART3] = { GATE(K210_SYSCTL_EN_PERI, 18) },
	[K210_CLK_FPIOA] = { GATE(K210_SYSCTL_EN_PERI, 20) },
	[K210_CLK_SHA] = { GATE(K210_SYSCTL_EN_PERI, 26) },
	[K210_CLK_AES] = { GATE(K210_SYSCTL_EN_PERI, 19) },
	[K210_CLK_OTP] = { GATE(K210_SYSCTL_EN_PERI, 27) },
	[K210_CLK_RTC] = { GATE(K210_SYSCTL_EN_PERI, 29) },

	/* Gated divider clocks */
	[K210_CLK_SRAM0] = {
		GATE(K210_SYSCTL_EN_CENT, 1),
		DIV(K210_SYSCTL_THR0, 0, 4, DIV_ONE_BASED)
	},
	[K210_CLK_SRAM1] = {
		GATE(K210_SYSCTL_EN_CENT, 2),
		DIV(K210_SYSCTL_THR0, 4, 4, DIV_ONE_BASED)
	},
	[K210_CLK_ROM] = {
		GATE(K210_SYSCTL_EN_PERI, 0),
		DIV(K210_SYSCTL_THR0, 16, 4, DIV_ONE_BASED)
	},
	[K210_CLK_DVP] = {
		GATE(K210_SYSCTL_EN_PERI, 3),
		DIV(K210_SYSCTL_THR0, 12, 4, DIV_ONE_BASED)
	},
	[K210_CLK_APB0] = {
		GATE(K210_SYSCTL_EN_CENT, 3),
		DIV(K210_SYSCTL_SEL0, 3, 3, DIV_ONE_BASED)
	},
	[K210_CLK_APB1] = {
		GATE(K210_SYSCTL_EN_CENT, 4),
		DIV(K210_SYSCTL_SEL0, 6, 3, DIV_ONE_BASED)
	},
	[K210_CLK_APB2] = {
		GATE(K210_SYSCTL_EN_CENT, 5),
		DIV(K210_SYSCTL_SEL0, 9, 3, DIV_ONE_BASED)
	},
	[K210_CLK_AI] = {
		GATE(K210_SYSCTL_EN_PERI, 2),
		DIV(K210_SYSCTL_THR0, 8, 4, DIV_ONE_BASED)
	},
	[K210_CLK_SPI0] = {
		GATE(K210_SYSCTL_EN_PERI, 6),
		DIV(K210_SYSCTL_THR1, 0, 8, DIV_DOUBLE_ONE_BASED)
	},
	[K210_CLK_SPI1] = {
		GATE(K210_SYSCTL_EN_PERI, 7),
		DIV(K210_SYSCTL_THR1, 8, 8, DIV_DOUBLE_ONE_BASED)
	},
	[K210_CLK_SPI2] = {
		GATE(K210_SYSCTL_EN_PERI, 8),
		DIV(K210_SYSCTL_THR1, 16, 8, DIV_DOUBLE_ONE_BASED)
	},
	[K210_CLK_I2C0] = {
		GATE(K210_SYSCTL_EN_PERI, 13),
		DIV(K210_SYSCTL_THR5, 8, 8, DIV_DOUBLE_ONE_BASED)
	},
	[K210_CLK_I2C1] = {
		GATE(K210_SYSCTL_EN_PERI, 14),
		DIV(K210_SYSCTL_THR5, 16, 8, DIV_DOUBLE_ONE_BASED)
	},
	[K210_CLK_I2C2] = {
		GATE(K210_SYSCTL_EN_PERI, 15),
		DIV(K210_SYSCTL_THR5, 24, 8, DIV_DOUBLE_ONE_BASED)
	},
	[K210_CLK_WDT0] = {
		GATE(K210_SYSCTL_EN_PERI, 24),
		DIV(K210_SYSCTL_THR6, 0, 8, DIV_DOUBLE_ONE_BASED)
	},
	[K210_CLK_WDT1] = {
		GATE(K210_SYSCTL_EN_PERI, 25),
		DIV(K210_SYSCTL_THR6, 8, 8, DIV_DOUBLE_ONE_BASED)
	},
	[K210_CLK_I2S0] = {
		GATE(K210_SYSCTL_EN_PERI, 10),
		DIV(K210_SYSCTL_THR3, 0, 16, DIV_DOUBLE_ONE_BASED)
	},
	[K210_CLK_I2S1] = {
		GATE(K210_SYSCTL_EN_PERI, 11),
		DIV(K210_SYSCTL_THR3, 16, 16, DIV_DOUBLE_ONE_BASED)
	},
	[K210_CLK_I2S2] = {
		GATE(K210_SYSCTL_EN_PERI, 12),
		DIV(K210_SYSCTL_THR4, 0, 16, DIV_DOUBLE_ONE_BASED)
	},

	/* Divider clocks, no gate, no mux */
	[K210_CLK_I2S0_M] = {
		DIV(K210_SYSCTL_THR4, 16, 8, DIV_DOUBLE_ONE_BASED)
	},
	[K210_CLK_I2S1_M] = {
		DIV(K210_SYSCTL_THR4, 24, 8, DIV_DOUBLE_ONE_BASED)
	},
	[K210_CLK_I2S2_M] = {
		DIV(K210_SYSCTL_THR4, 0, 8, DIV_DOUBLE_ONE_BASED)
	},

	/* Muxed gated divider clocks */
	[K210_CLK_SPI3] = {
		GATE(K210_SYSCTL_EN_PERI, 9),
		DIV(K210_SYSCTL_THR1, 24, 8, DIV_DOUBLE_ONE_BASED),
		MUX(K210_SYSCTL_SEL0, 12)
	},
	[K210_CLK_TIMER0] = {
		GATE(K210_SYSCTL_EN_PERI, 21),
		DIV(K210_SYSCTL_THR2,  0, 8, DIV_DOUBLE_ONE_BASED),
		MUX(K210_SYSCTL_SEL0, 13)
	},
	[K210_CLK_TIMER1] = {
		GATE(K210_SYSCTL_EN_PERI, 22),
		DIV(K210_SYSCTL_THR2, 8, 8, DIV_DOUBLE_ONE_BASED),
		MUX(K210_SYSCTL_SEL0, 14)
	},
	[K210_CLK_TIMER2] = {
		GATE(K210_SYSCTL_EN_PERI, 23),
		DIV(K210_SYSCTL_THR2, 16, 8, DIV_DOUBLE_ONE_BASED),
		MUX(K210_SYSCTL_SEL0, 15)
	},
};

/*
 * PLL control register bits.
 */
#define K210_PLL_CLKR		GENMASK(3, 0)
#define K210_PLL_CLKF		GENMASK(9, 4)
#define K210_PLL_CLKOD		GENMASK(13, 10)
#define K210_PLL_BWADJ		GENMASK(19, 14)
#define K210_PLL_RESET		(1 << 20)
#define K210_PLL_PWRD		(1 << 21)
#define K210_PLL_INTFB		(1 << 22)
#define K210_PLL_BYPASS		(1 << 23)
#define K210_PLL_TEST		(1 << 24)
#define K210_PLL_EN		(1 << 25)
#define K210_PLL_SEL		GENMASK(27, 26) /* PLL2 only */

/*
 * PLL lock register bits.
 */
#define K210_PLL_LOCK		0
#define K210_PLL_CLEAR_SLIP	2
#define K210_PLL_TEST_OUT	3

/*
 * Clock selector register bits.
 */
#define K210_ACLK_SEL		BIT(0)
#define K210_ACLK_DIV		GENMASK(2, 1)

/*
 * PLLs.
 */
enum k210_pll_id {
	K210_PLL0, K210_PLL1, K210_PLL2, K210_PLL_NUM
};

struct k210_pll {
	enum k210_pll_id id;
	void __iomem *reg;
	void __iomem *lock;
	u8 shift; 		/* Offset of lock bits */
	u8 width; 		/* Width of lock bits */
	struct clk_hw hw;
};
#define to_k210_pll(hw)	container_of(hw, struct k210_pll, hw)

struct k210_pll_cfg {
	u32 reg;
	u8 shift; 		/* Offset of lock bits */
	u8 width; 		/* Width of lock bits */
	u32 r, f, od, bwadj;	/* Initial factors */
};

/*
 * PLL factors:
 * By default, PLL0 runs at 780 MHz and PLL1 at 299 MHz.
 * The first 2 sram banks depend on ACLK/CPU clock which is by default
 * PLL0 rate divided by 2. Set PLL1 to 390 MHz so that the third sram
 * bank has the same clock.
 */
static struct k210_pll_cfg k210_plls_cfg[] = {
	{ K210_SYSCTL_PLL0,  0, 2, 0, 59, 1, 59 }, /* 780 MHz */
	{ K210_SYSCTL_PLL1,  8, 1, 0, 59, 3, 59 }, /* 390 MHz */
	{ K210_SYSCTL_PLL2, 16, 1, 0, 22, 1, 22 }, /* 299 MHz */
};

/*
 * Clocks data.
 */
struct k210_clk {
	void __iomem			*regs;
	spinlock_t			clk_lock;
	struct k210_pll			plls[K210_PLL_NUM];
	struct clk_hw			aclk;
	struct clk_hw			clks[K210_NUM_CLKS];
	struct clk_hw_onecell_data	*clk_data;
};

static struct k210_clk *kcl;

/*
 * Set ACLK parent selector: 0 for IN0, 1 for PLL0.
 */
static void k210_aclk_set_selector(u8 sel)
{
	u32 reg = readl(kcl->regs + K210_SYSCTL_SEL0);

	if (sel)
		reg |= K210_ACLK_SEL;
	else
		reg &= K210_ACLK_SEL;
	writel(reg, kcl->regs + K210_SYSCTL_SEL0);
}

static void k210_init_pll(struct k210_pll *pll, enum k210_pll_id id,
			  void __iomem *base)
{
	pll->id = id;
	pll->lock = base + K210_SYSCTL_PLL_LOCK;
	pll->reg = base + k210_plls_cfg[id].reg;
	pll->shift = k210_plls_cfg[id].shift;
	pll->width = k210_plls_cfg[id].width;
}

static void k210_pll_wait_for_lock(struct k210_pll *pll)
{
	u32 reg, mask = GENMASK(pll->width - 1, 0) << pll->shift;

        while (true) {
                reg = readl(pll->lock);
                if ((reg & mask) == mask)
                        break;

                reg |= BIT(pll->shift + K210_PLL_CLEAR_SLIP);
                writel(reg, pll->lock);
        }
}

static bool k210_pll_hw_is_enabled(struct k210_pll *pll)
{
	u32 reg = readl(pll->reg);
	u32 mask = K210_PLL_PWRD | K210_PLL_EN;

	if (reg & K210_PLL_RESET)
		return false;

	return (reg & mask) == mask;
}

static void k210_pll_enable_hw(struct k210_pll *pll)
{
	struct k210_pll_cfg *pll_cfg = &k210_plls_cfg[pll->id];
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&kcl->clk_lock, flags);

	if (k210_pll_hw_is_enabled(pll))
		goto unlock;

	if (pll->id == K210_PLL0) {
		/* Re-parent aclk to IN0 to keep the CPUs running */
		k210_aclk_set_selector(0);
	}

	/* Set factors */
	reg = readl(pll->reg);
	reg &= ~GENMASK(19, 0);
	reg |= FIELD_PREP(K210_PLL_CLKR, pll_cfg->r);
	reg |= FIELD_PREP(K210_PLL_CLKF, pll_cfg->f);
	reg |= FIELD_PREP(K210_PLL_CLKOD, pll_cfg->od);
	reg |= FIELD_PREP(K210_PLL_BWADJ, pll_cfg->bwadj);
	reg |= K210_PLL_PWRD;
	writel(reg, pll->reg);

	/* Ensure reset is low before asserting it */
	reg &= ~K210_PLL_RESET;
	writel(reg, pll->reg);
	reg |= K210_PLL_RESET;
	writel(reg, pll->reg);
	nop();
        nop();
	reg &= ~K210_PLL_RESET;
	writel(reg, pll->reg);

	k210_pll_wait_for_lock(pll);

	reg &= ~K210_PLL_BYPASS;
        reg |= K210_PLL_EN;
	writel(reg, pll->reg);

	if (pll->id == K210_PLL0) {
		/* Re-parent aclk back to PLL0 */
		k210_aclk_set_selector(1);
	}
unlock:
	spin_unlock_irqrestore(&kcl->clk_lock, flags);
}

static void k210_pll_disable_hw(struct k210_pll *pll)
{
	unsigned long flags;
	u32 reg;

        /*
         * Bypassing before powering off is important so child clocks don't stop
         * working. This is especially important for pll0, the indirect parent
         * of the cpu clock.
         */
	spin_lock_irqsave(&kcl->clk_lock, flags);
	reg = readl(pll->reg);
        reg |= K210_PLL_BYPASS;
        writel(reg, pll->reg);

        reg &= ~K210_PLL_PWRD;
        reg &= ~K210_PLL_EN;
        writel(reg, pll->reg);
	spin_unlock_irqrestore(&kcl->clk_lock, flags);
}

static int k210_pll_enable(struct clk_hw *hw)
{
	k210_pll_enable_hw(to_k210_pll(hw));

	return 0;
}

static void k210_pll_disable(struct clk_hw *hw)
{
	k210_pll_disable_hw(to_k210_pll(hw));
}

static int k210_pll_is_enabled(struct clk_hw *hw)
{
	return k210_pll_hw_is_enabled(to_k210_pll(hw));
}

static int k210_pll_set_parent(struct clk_hw *hw, u8 index)
{
	struct k210_pll *pll = to_k210_pll(hw);
	unsigned long flags;
	int ret = 0;
	u32 reg;

	spin_lock_irqsave(&kcl->clk_lock, flags);

	switch (pll->id) {
	case K210_PLL0:
	case K210_PLL1:
		if (WARN_ON(index != 0))
			ret = -EINVAL;
		break;
	case K210_PLL2:
		if (WARN_ON(index > 2)) {
			ret = -EINVAL;
			break;
		}
		reg = readl(pll->reg);
		reg &= ~K210_PLL_SEL;
		reg |= FIELD_PREP(K210_PLL_SEL, index);
		writel(reg, pll->reg);
		break;
	default:
		pr_err("Invalid pll\n");
		ret = -EINVAL;
		break;
	}

	spin_unlock_irqrestore(&kcl->clk_lock, flags);

	return ret;
}

static u8 k210_pll_get_parent(struct clk_hw *hw)
{
	struct k210_pll *pll = to_k210_pll(hw);
	u32 reg;

	switch (pll->id) {
	case K210_PLL0:
	case K210_PLL1:
		return 0;
	case K210_PLL2:
		reg = readl(pll->reg);
		return FIELD_GET(K210_PLL_SEL, reg);
	default:
		pr_err("Invalid pll\n");
		return 0;
	}
}

static unsigned long k210_pll_get_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct k210_pll *pll = to_k210_pll(hw);
	u32 reg = readl(pll->reg);
	u32 r, f, od;

	if (reg & K210_PLL_BYPASS)
		return parent_rate;

	if (!(reg & K210_PLL_PWRD))
		return 0;

	r = FIELD_GET(K210_PLL_CLKR, reg) + 1;
	f = FIELD_GET(K210_PLL_CLKF, reg) + 1;
	od = FIELD_GET(K210_PLL_CLKOD, reg) + 1;

	return (u64)parent_rate * f / (r * od);
}

static const struct clk_ops k210_pll_ops = {
	.enable		= k210_pll_enable,
	.disable	= k210_pll_disable,
	.is_enabled	= k210_pll_is_enabled,
	.set_parent	= k210_pll_set_parent,
	.get_parent	= k210_pll_get_parent,
	.recalc_rate	= k210_pll_get_rate,
};

static const char *pll_parents[] = { NULL, "pll0", "pll1" };

static struct clk_hw *k210_register_pll(enum k210_pll_id id, const char *name,
				const char **parent_names, int num_parents,
				unsigned long flags)
{
        struct k210_pll *pll = &kcl->plls[id];
        struct clk_init_data init = {};
	int ret;

        init.name = name;
        init.parent_names = parent_names;
        init.num_parents = num_parents;
        init.flags = flags;
        init.ops = &k210_pll_ops;
        pll->hw.init = &init;

	ret = clk_hw_register(NULL, &pll->hw);
	if (ret)
		return ERR_PTR(ret);

	return &pll->hw;
}

static int k210_aclk_set_parent(struct clk_hw *hw, u8 index)
{
	if (WARN_ON(index > 1))
		return -EINVAL;

	k210_aclk_set_selector(index);

	return 0;
}

static u8 k210_aclk_get_parent(struct clk_hw *hw)
{
	u32 sel = readl(kcl->regs + K210_SYSCTL_SEL0);

	return (sel & K210_ACLK_SEL) ? 1 : 0;
}

static unsigned long k210_aclk_get_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	u32 reg = readl(kcl->regs + K210_SYSCTL_SEL0);
	unsigned int shift;

	if (!(reg & 0x1))
		return parent_rate;

	shift = FIELD_GET(K210_ACLK_DIV, reg);

	return parent_rate / (2UL << shift);
}

static const struct clk_ops k210_aclk_ops = {
	.set_parent	= k210_aclk_set_parent,
	.get_parent	= k210_aclk_get_parent,
	.recalc_rate	= k210_aclk_get_rate,
};

static const char *aclk_parents[] = { NULL, "pll0" };

static struct clk_hw *k210_register_aclk(void)
{
        struct clk_init_data init = {};
	int ret;

        init.name = "aclk";
        init.parent_names = aclk_parents;
        init.num_parents = 2;
        init.flags = CLK_IS_CRITICAL;
        init.ops = &k210_aclk_ops;
	kcl->aclk.init = &init;

	ret = clk_hw_register(NULL, &kcl->aclk);
	if (ret)
		return ERR_PTR(ret);

	return &kcl->aclk;
}

#define to_k210_clk_id(hw)	(int)((hw) - &kcl->clks[0])
#define to_k210_clk_cfg(hw)	&k210_clks[to_k210_clk_id(hw)]

static int k210_clk_is_prepared(struct clk_hw *hw)
{
	return 1;
}

static int k210_clk_enable(struct clk_hw *hw)
{
	const struct k210_clk_cfg *kclk = to_k210_clk_cfg(hw);
	unsigned long flags;
	u32 reg;

	if (!kclk->gate_reg)
		return 0;

	spin_lock_irqsave(&kcl->clk_lock, flags);
	reg = readl(kcl->regs + kclk->gate_reg);
	reg |= BIT(kclk->gate_bit);
	writel(reg, kcl->regs + kclk->gate_reg);
	spin_unlock_irqrestore(&kcl->clk_lock, flags);

	return 0;
}

static void k210_clk_disable(struct clk_hw *hw)
{
	const struct k210_clk_cfg *kclk = to_k210_clk_cfg(hw);
	unsigned long flags;
	u32 reg;

	if (!kclk->gate_reg)
		return;

	spin_lock_irqsave(&kcl->clk_lock, flags);
	reg = readl(kcl->regs + kclk->gate_reg);
	reg &= ~BIT(kclk->gate_bit);
	writel(reg, kcl->regs + kclk->gate_reg);
	spin_unlock_irqrestore(&kcl->clk_lock, flags);
}

static int k210_clk_is_enabled(struct clk_hw *hw)
{
	const struct k210_clk_cfg *kclk = to_k210_clk_cfg(hw);
	u32 reg = 0;

	if (kclk->gate_reg)
		reg = readl(kcl->regs + kclk->gate_reg);

	return !kclk->gate_reg || (reg & BIT(kclk->gate_bit));
}

static int k210_clk_set_parent(struct clk_hw *hw, u8 index)
{
	const struct k210_clk_cfg *kclk = to_k210_clk_cfg(hw);
	unsigned long flags;
	u32 reg;

	if (!kclk->mux_reg) {
		if (WARN_ON(index != 0))
			return -EINVAL;
		return 0;
	}

	spin_lock_irqsave(&kcl->clk_lock, flags);
	reg = readl(kcl->regs + kclk->mux_reg);
	if (index)
		reg |= BIT(kclk->mux_bit);
	else
		reg &= ~BIT(kclk->mux_bit);
	spin_unlock_irqrestore(&kcl->clk_lock, flags);

	return 0;
}

static u8 k210_clk_get_parent(struct clk_hw *hw)
{
	const struct k210_clk_cfg *kclk = to_k210_clk_cfg(hw);
	unsigned long flags;
	u32 reg, idx;

	if (!kclk->mux_reg)
		return 0;

	spin_lock_irqsave(&kcl->clk_lock, flags);
	reg = readl(kcl->regs + kclk->mux_reg);
	idx = (reg & BIT(kclk->mux_bit)) ? 1 : 0;
	spin_unlock_irqrestore(&kcl->clk_lock, flags);

	return idx;
}

static unsigned long k210_clk_get_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	const struct k210_clk_cfg *kclk = to_k210_clk_cfg(hw);
	u32 reg, div;

	if (!kclk->div_reg)
		return parent_rate;

	reg = readl(kcl->regs + kclk->div_reg);
	div = (reg >> kclk->div_shift) & GENMASK(kclk->div_width - 1, 0);

	switch (kclk->div_type) {
	case DIV_ONE_BASED:
		return parent_rate / (div + 1);
	case DIV_DOUBLE_ONE_BASED:
		return parent_rate / ((div + 1) * 2);
	case DIV_POWER_OF_TWO:
		return parent_rate / (2UL << div);
	default:
		pr_err("Invalid divider type\n");
		return 0;
	}
}

static const struct clk_ops k210_clk_ops = {
	.is_prepared	= k210_clk_is_prepared,
	.enable		= k210_clk_enable,
	.is_enabled	= k210_clk_is_enabled,
	.disable	= k210_clk_disable,
	.set_parent	= k210_clk_set_parent,
	.get_parent	= k210_clk_get_parent,
	.recalc_rate	= k210_clk_get_rate,
};

static const char *mux_parents[] = { NULL, "pll0" };

static struct clk_hw *k210_register_clk(int id, const char *name,
					const char *parent, unsigned long flags)
{
	struct clk_init_data init = {};
	int ret;

        init.name = name;
	if (parent) {
		init.parent_names = &parent;
		init.num_parents = 1;
	} else {
		init.parent_names = mux_parents;
		init.num_parents = 2;
	}
        init.flags = flags;
        init.ops = &k210_clk_ops;
	kcl->clks[id].init = &init;

	ret = clk_hw_register(NULL, &kcl->clks[id]);
	if (ret)
		return ERR_PTR(ret);

	return &kcl->clks[id];
}

static __init void k210_clk_init(struct device_node *np)
{
	struct clk *in0_clk;
	const char *in0;
	struct clk_hw **hws;
	int i, ret;

	pr_info("K210 system controller\n");

	kcl = kzalloc(sizeof(*kcl), GFP_KERNEL);
	if (!kcl) {
		pr_err("data allocation failed\n");
		return;
	}

	kcl->regs = ioremap(K210_SYSCTL_BASE_ADDR, K210_SYSCTL_BASE_SIZE);
	if (!kcl->regs) {
		pr_err("map sysctl resources failed\n");
		goto err;
	}

	kcl->clk_data = kzalloc(struct_size(kcl->clk_data, hws, K210_NUM_CLKS),
				GFP_KERNEL);
	if (!kcl->clk_data) {
		pr_err("clk data allocation failed\n");
		goto err;
	}

	for (i = 0; i < K210_PLL_NUM; i++)
		k210_init_pll(&kcl->plls[i], i, kcl->regs);
	spin_lock_init(&kcl->clk_lock);
	kcl->clk_data->num = K210_NUM_CLKS;
	hws = kcl->clk_data->hws;
	for (i = 1; i < K210_NUM_CLKS; i++)
		hws[i] = ERR_PTR(-EPROBE_DEFER);

	/*
	 * in0 is the system base fixed-rate 26MHz oscillator which
	 * should already be defined by the device tree.
	 */
	in0_clk = of_clk_get(np, 0);
	if (IS_ERR(in0_clk)) {
		pr_warn("in0 oscillator not found\n");
		hws[K210_CLK_IN0] =
			clk_hw_register_fixed_rate(NULL, "in0", NULL,
						   0, K210_IN0_RATE);
	} else {
		hws[K210_CLK_IN0] = __clk_get_hw(in0_clk);
	}
	if (IS_ERR(hws[K210_CLK_IN0])) {
		pr_err("failed to get base oscillator\n");
		goto err;
	}

	in0 = clk_hw_get_name(hws[K210_CLK_IN0]);
	aclk_parents[0] = in0;
	pll_parents[0] = in0;
	mux_parents[0] = in0;

	pr_info("fixed-rate %lu MHz %s base clock\n",
		clk_hw_get_rate(hws[K210_CLK_IN0]) / 1000000, in0);

	/* PLLs */
	hws[K210_CLK_PLL0] =
		k210_register_pll(K210_PLL0, "pll0",
				  pll_parents, 1, CLK_IS_CRITICAL);
	hws[K210_CLK_PLL1] =
		k210_register_pll(K210_PLL1, "pll1",
				  pll_parents, 1, CLK_IS_CRITICAL);
	hws[K210_CLK_PLL2] =
		k210_register_pll(K210_PLL2, "pll2", pll_parents, 3, 0);

	/* aclk: muxed of in0 and pll0_d, no gate */
	hws[K210_CLK_ACLK] = k210_register_aclk();

	/* Clocks with aclk as source */
	hws[K210_CLK_CPU] =
		k210_register_clk(K210_CLK_CPU,
				  "cpu", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_CLINT] =
		clk_hw_register_fixed_factor(NULL, "clint", "aclk",
					     CLK_IS_CRITICAL, 1, 50);
	hws[K210_CLK_DMA] =
		k210_register_clk(K210_CLK_DMA, "dma", "aclk", 0);
	hws[K210_CLK_FFT] =
		k210_register_clk(K210_CLK_FFT, "fft", "aclk", 0);
	hws[K210_CLK_SRAM0] =
		k210_register_clk(K210_CLK_SRAM0,
				  "sram0", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_SRAM1] =
		k210_register_clk(K210_CLK_SRAM1,
				  "sram1", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_ROM] =
		k210_register_clk(K210_CLK_ROM, "rom", "aclk", 0);
	hws[K210_CLK_DVP] =
		k210_register_clk(K210_CLK_DVP, "dvp", "aclk", 0);
	hws[K210_CLK_APB0] =
		k210_register_clk(K210_CLK_APB0,
				  "apb0", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_APB1] =
		k210_register_clk(K210_CLK_APB1,
				  "apb1", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_APB2] =
		k210_register_clk(K210_CLK_APB2,
				  "apb2", "aclk", CLK_IS_CRITICAL);

	/* Clocks with PLL0 as source */
	hws[K210_CLK_SPI0] = k210_register_clk(K210_CLK_SPI0,
					       "spi0", "pll0", 0);
	hws[K210_CLK_SPI1] = k210_register_clk(K210_CLK_SPI1,
					       "spi1", "pll0", 0);
	hws[K210_CLK_SPI2] = k210_register_clk(K210_CLK_SPI2,
					       "spi2", "pll0", 0);
	hws[K210_CLK_I2C0] = k210_register_clk(K210_CLK_I2C0,
					       "i2c0", "pll0", 0);
	hws[K210_CLK_I2C1] = k210_register_clk(K210_CLK_I2C1,
					       "i2c1", "pll0", 0);
	hws[K210_CLK_I2C2] = k210_register_clk(K210_CLK_I2C2,
					       "i2c2", "pll0", 0);

	/*
	 * Clocks with PLL1 as source: there is only the AI clock.
	 * Do not enable it as the KPU device uses the aisram bank
	 * which used as general memory.
	 * hws[K210_CLK_AI] =
	 *	k210_register_clk(K210_CLK_AI, "ai", "pll1", 0);
	 */
	hws[K210_CLK_AI] = NULL;

	/* Clocks with PLL2 as source */
	hws[K210_CLK_I2S0] = k210_register_clk(K210_CLK_I2S0,
					       "i2s0", "pll2", 0);
	hws[K210_CLK_I2S1] = k210_register_clk(K210_CLK_I2S1,
					       "i2s1", "pll2", 0);
	hws[K210_CLK_I2S2] = k210_register_clk(K210_CLK_I2S2,
					       "i2s2", "pll2", 0);
	hws[K210_CLK_I2S0_M] =
		k210_register_clk(K210_CLK_I2S0_M, "i2s0_m", "pll2", 0);
	hws[K210_CLK_I2S1_M] =
		k210_register_clk(K210_CLK_I2S1_M, "i2s1_m", "pll2", 0);
	hws[K210_CLK_I2S2_M] =
		k210_register_clk(K210_CLK_I2S2_M, "i2s2_m", "pll2", 0);

	/* Clocks with IN0 as source */
	hws[K210_CLK_WDT0] = k210_register_clk(K210_CLK_WDT0, "wdt0", in0, 0);
	hws[K210_CLK_WDT1] = k210_register_clk(K210_CLK_WDT1, "wdt1", in0, 0);
	hws[K210_CLK_RTC] = k210_register_clk(K210_CLK_RTC, "rtc", in0, 0);

	/* Clocks with APB0 as source */
	hws[K210_CLK_GPIO] =
		k210_register_clk(K210_CLK_GPIO, "gpio", "apb0", 0);
	hws[K210_CLK_UART1] =
		k210_register_clk(K210_CLK_UART1, "uart1", "apb0", 0);
	hws[K210_CLK_UART2] =
		k210_register_clk(K210_CLK_UART2, "uart2", "apb0", 0);
	hws[K210_CLK_UART3] =
		k210_register_clk(K210_CLK_UART3, "uart3", "apb0", 0);
	hws[K210_CLK_FPIOA] =
		k210_register_clk(K210_CLK_FPIOA, "fpioa", "apb0", 0);
	hws[K210_CLK_SHA] =
		k210_register_clk(K210_CLK_SHA, "sha", "apb0", 0);

	/* Clocks with APB1 as source */
	hws[K210_CLK_AES] = k210_register_clk(K210_CLK_AES, "aes", "apb1", 0);
	hws[K210_CLK_OTP] = k210_register_clk(K210_CLK_OTP, "otp", "apb1", 0);

	/* Muxed clocks with in0/pll0 as source */
	hws[K210_CLK_SPI3] =
		k210_register_clk(K210_CLK_SPI3, "spi3", NULL, 0);
	hws[K210_CLK_TIMER0] =
		k210_register_clk(K210_CLK_TIMER0, "timer0", NULL, 0);
	hws[K210_CLK_TIMER1] =
		k210_register_clk(K210_CLK_TIMER1, "timer1", NULL, 0);
	hws[K210_CLK_TIMER2] =
		k210_register_clk(K210_CLK_TIMER2, "timer2", NULL, 0);

	for (i = 0; i < K210_NUM_CLKS; i++) {
		if (IS_ERR(hws[i])) {
			pr_err("register clock %d failed %ld\n",
			       i, PTR_ERR(hws[i]));
			goto err;
		}
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, kcl->clk_data);
	if (ret)
		pr_err("add clock provider failed %d\n", ret);

	return;
err:
	pr_err("clock initialization failed\n");
	iounmap(kcl->regs);
	kfree(kcl->clk_data);
	kfree(kcl);
	kcl = NULL;
}

CLK_OF_DECLARE_DRIVER(k210_clk, "kendryte,k210-clk", k210_clk_init);

/*
 * Enable PLL1 to be able to use the AI SRAM.
 */
void k210_clk_early_init(void __iomem *regs)
{
	struct k210_pll pll1;

	/* Make sure aclk selector is set to PLL0 */
	k210_aclk_set_selector(1);

	/* Startup PLL1 to enable the third sram bank */
	k210_init_pll(&pll1, K210_PLL1, regs);
	k210_pll_enable_hw(&pll1);
}

#ifdef CONFIG_SOC_KENDRYTE_K210_CLK_DEBUG

static void k210_clk_show_clk(int id, int level, char *str, bool *shown)
{
	struct clk_hw *hw = kcl->clk_data->hws[id];

	if (level == 0)
		pr_info("(%d) %s : %lu Hz (%d)\n",
			id, clk_hw_get_name(hw), clk_hw_get_rate(hw),
			__clk_get_enable_count(hw->clk));
	else
		pr_info("%s+-- (%d) %s : %lu Hz (%d)\n",
			str, id, clk_hw_get_name(hw), clk_hw_get_rate(hw),
			__clk_get_enable_count(hw->clk));

	shown[id] = true;
}

static void k210_clk_dump_clk(struct clk_hw *parent, int level, char *str,
			      bool *shown)
{
	struct clk_hw *hw, *h;
	bool has_prev, has_next;
	int i, j;

	for (i = 0; i < kcl->clk_data->num; i++) {
		hw = kcl->clk_data->hws[i];

		if (!hw || parent == hw || shown[i])
			continue;

		if (clk_hw_get_parent(hw) != parent)
			continue;

		k210_clk_show_clk(i, level, str, shown);

		has_prev = false;
		for (j = i - 1; j >= 0; j--) {
			h = kcl->clk_data->hws[j];
			if (clk_hw_get_parent(h) == parent) {
				has_prev = true;
				break;
			}
		}

		has_next = false;
		for (j = i + 1; j < kcl->clk_data->num; j++) {
			h = kcl->clk_data->hws[j];
			if (clk_hw_get_parent(h) == parent) {
				has_next = true;
				break;
			}
		}

		if (level > 0) {
			if (has_next || has_prev)
				strcat(str, "|    ");
			else
				strcat(str, "     ");
		}

		k210_clk_dump_clk(hw, level + 1, str, shown);

		if (level > 0)
			str[strlen(str) - 5] = '\0';
	}
}

static void k210_clk_dump_info(void)
{
	static bool shown[K210_NUM_CLKS];
	char str[64];
	int i;

	pr_info("-------------------------------------\n");

	memset(shown, 0, sizeof(shown));
	memset(str, 0, sizeof(str));

	pr_info("PLLs factors:\n");
	for (i = 0; i < K210_PLL_NUM; i++)
		pr_info("    PLL%d: r = %u, f = %u, od = %u, bwadj = %u\n",
			i, k210_plls_cfg[i].r,
			k210_plls_cfg[i].f,
			k210_plls_cfg[i].od,
			k210_plls_cfg[i].bwadj);

	pr_info("K210 clock tree:\n");
	k210_clk_show_clk(K210_CLK_IN0, 0, str, shown);
	strcat(str, " ");
	k210_clk_dump_clk(kcl->clk_data->hws[K210_CLK_IN0], 1, str, shown);

	pr_info("-------------------------------------\n");
}

static int k210_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_info(dev, "K210 clock driver\n");

	if (of_property_read_bool(dev->of_node, "debug"))
		k210_clk_dump_info();

	return 0;
}

static const struct of_device_id k210_clk_of_match[] = {
	{ .compatible = "kendryte,k210-clk", },
	{}
};

static struct platform_driver k210_clk_driver = {
	.driver	= {
		.name		= "k210-clk",
		.of_match_table	= k210_clk_of_match,
	},
	.probe			= k210_clk_probe,
};
builtin_platform_driver(k210_clk_driver);

#endif
