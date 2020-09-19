// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2019-20 Sean Anderson <seanga2@gmail.com>
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 */
#define pr_fmt(fmt)     "k210-clk: " fmt

#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/clk-provider.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <asm/soc.h>
#include <soc/canaan/k210-sysctl.h>

#include <dt-bindings/clock/k210-clk.h>

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
	DIV_NONE,
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

static struct k210_clk_cfg k210_clks[K210_NUM_CLKS] = {

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
	u8 lock_shift;
	u8 lock_width;
	struct clk_hw hw;
};
#define to_k210_pll(hw)	container_of(hw, struct k210_pll, hw)

/*
 * PLLs configuration: by default PLL0 runs at 780 MHz and PLL1 at 299 MHz.
 * The first 2 SRAM banks depend on ACLK/CPU clock which is by default PLL0
 * rate divided by 2. Set PLL1 to 390 MHz so that the third SRAM bank has the
 * same clock as the first 2.
 */
struct k210_pll_cfg {
	u32 reg;
	u8 lock_shift;
	u8 lock_width;
	u32 r;
	u32 f;
	u32 od;
	u32 bwadj;
};

static struct k210_pll_cfg k210_plls_cfg[] = {
	{ K210_SYSCTL_PLL0,  0, 2, 0, 59, 1, 59 }, /* 780 MHz */
	{ K210_SYSCTL_PLL1,  8, 1, 0, 59, 3, 59 }, /* 390 MHz */
	{ K210_SYSCTL_PLL2, 16, 1, 0, 22, 1, 22 }, /* 299 MHz */
};

/**
 * struct k210_clk - Driver data
 * @regs: system controller registers start address
 * @clk_lock: clock setting spinlock
 * @plls: SoC PLLs descriptors
 * @aclk: ACLK clock
 * @clks: All other clocks
 * @clk_data: clock specifier translation for all clocks
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
	pll->lock_shift = k210_plls_cfg[id].lock_shift;
	pll->lock_width = k210_plls_cfg[id].lock_width;
}

static void k210_pll_wait_for_lock(struct k210_pll *pll)
{
	u32 reg, mask = GENMASK(pll->lock_shift + pll->lock_width - 1,
				pll->lock_shift);

	while (true) {
		reg = readl(pll->lock);
		if ((reg & mask) == mask)
			break;

		reg |= BIT(pll->lock_shift + K210_PLL_CLEAR_SLIP);
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

	/*
	 * For PLL0, we need to re-parent ACLK to IN0 to keep the CPU cores and
	 * SRAM running.
	 */
	if (pll->id == K210_PLL0)
		k210_aclk_set_selector(0);

	/* Set PLL factors */
	reg = readl(pll->reg);
	reg &= ~GENMASK(19, 0);
	reg |= FIELD_PREP(K210_PLL_CLKR, pll_cfg->r);
	reg |= FIELD_PREP(K210_PLL_CLKF, pll_cfg->f);
	reg |= FIELD_PREP(K210_PLL_CLKOD, pll_cfg->od);
	reg |= FIELD_PREP(K210_PLL_BWADJ, pll_cfg->bwadj);
	reg |= K210_PLL_PWRD;
	writel(reg, pll->reg);

	/*
	 * Reset the PLL: ensure reset is low before asserting it.
	 * The magic NOPs come from the Kendryte reference SDK.
	 */
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

	if (pll->id == K210_PLL0)
		k210_aclk_set_selector(1);

unlock:
	spin_unlock_irqrestore(&kcl->clk_lock, flags);
}

static void k210_pll_disable_hw(struct k210_pll *pll)
{
	unsigned long flags;
	u32 reg;

	/*
	 * Bypassing before powering off is important so child clocks do not
	 * stop working. This is especially important for pll0, the indirect
	 * parent of the cpu clock.
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
	.recalc_rate	= k210_pll_get_rate,
};

static int k210_pll2_set_parent(struct clk_hw *hw, u8 index)
{
	struct k210_pll *pll = to_k210_pll(hw);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&kcl->clk_lock, flags);

	reg = readl(pll->reg);
	reg &= ~K210_PLL_SEL;
	reg |= FIELD_PREP(K210_PLL_SEL, index);
	writel(reg, pll->reg);

	spin_unlock_irqrestore(&kcl->clk_lock, flags);

	return 0;
}

static u8 k210_pll2_get_parent(struct clk_hw *hw)
{
	struct k210_pll *pll = to_k210_pll(hw);
	u32 reg = readl(pll->reg);

	return FIELD_GET(K210_PLL_SEL, reg);
}

static const struct clk_ops k210_pll2_ops = {
	.enable		= k210_pll_enable,
	.disable	= k210_pll_disable,
	.is_enabled	= k210_pll_is_enabled,
	.recalc_rate	= k210_pll_get_rate,
	.set_parent	= k210_pll2_set_parent,
	.get_parent	= k210_pll2_get_parent,
};

static const char * const k210_pll_parents[] = { "in0", "pll0", "pll1" };

static int k210_register_pll(struct k210_pll *pll, const char *name,
			     int num_parents, const struct clk_ops *ops)
{
	struct clk_init_data init = {};

	init.name = name;
	init.parent_names = k210_pll_parents;
	init.num_parents = num_parents;
	init.ops = ops;
	pll->hw.init = &init;

	return clk_hw_register(NULL, &pll->hw);
}

static int k210_register_plls(struct device_node *np)
{
	int i, ret;

	for (i = 0; i < K210_PLL_NUM; i++)
		k210_init_pll(&kcl->plls[i], i, kcl->regs);

	ret = k210_register_pll(&kcl->plls[K210_PLL0], "pll0", 1,
				&k210_pll_ops);
	if (ret) {
		pr_err("%pOFP: register PLL0 failed\n", np);
		return ret;
	}

	ret = k210_register_pll(&kcl->plls[K210_PLL1], "pll1", 1,
				&k210_pll_ops);
	if (ret) {
		pr_err("%pOFP: register PLL1 failed\n", np);
		return ret;
	}

	ret = k210_register_pll(&kcl->plls[K210_PLL2], "pll2", 3,
				&k210_pll2_ops);
	if (ret) {
		pr_err("%pOFP: register PLL2 failed\n", np);
		return ret;
	}

	return 0;
}

static int k210_aclk_set_parent(struct clk_hw *hw, u8 index)
{
	k210_aclk_set_selector(index);

	return 0;
}

static u8 k210_aclk_get_parent(struct clk_hw *hw)
{
	u32 sel;

	sel = readl(kcl->regs + K210_SYSCTL_SEL0) & K210_ACLK_SEL;

	return sel ? 1 : 0;
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

static const char * const k210_aclk_parents[] = { "in0", "pll0" };

static int k210_register_aclk(struct device_node *np)
{
	struct clk_init_data init = {};
	int ret;

	init.name = "aclk";
	init.parent_names = k210_aclk_parents;
	init.num_parents = 2;
	init.ops = &k210_aclk_ops;
	kcl->aclk.init = &init;

	ret = clk_hw_register(NULL, &kcl->aclk);
	if (ret)
		pr_err("%pOFP: register aclk failed\n", np);

	return ret;
}

#define to_k210_clk_id(hw)	((unsigned int)((hw) - &kcl->clks[0]))
#define to_k210_clk_cfg(hw)	(&k210_clks[to_k210_clk_id(hw)])

static u32 k210_clk_get_div_val(struct k210_clk_cfg *kclk)
{
	u32 reg = readl(kcl->regs + kclk->div_reg);

	return (reg >> kclk->div_shift) & GENMASK(kclk->div_width - 1, 0);
}

static unsigned long k210_clk_divider(struct k210_clk_cfg *kclk,
				      u32 div_val)
{
	switch (kclk->div_type) {
	case DIV_ONE_BASED:
		return div_val + 1;
	case DIV_DOUBLE_ONE_BASED:
		return (div_val + 1) * 2;
	case DIV_POWER_OF_TWO:
		return 2UL << div_val;
	case DIV_NONE:
	default:
		return 0;
	}
}

static int k210_clk_enable(struct clk_hw *hw)
{
	struct k210_clk_cfg *kclk = to_k210_clk_cfg(hw);
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
	struct k210_clk_cfg *kclk = to_k210_clk_cfg(hw);
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

static int k210_clk_set_parent(struct clk_hw *hw, u8 index)
{
	struct k210_clk_cfg *kclk = to_k210_clk_cfg(hw);
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
	struct k210_clk_cfg *kclk = to_k210_clk_cfg(hw);
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
	struct k210_clk_cfg *kclk = to_k210_clk_cfg(hw);
	unsigned long divider;

	if (!kclk->div_reg)
		return parent_rate;

	divider = k210_clk_divider(kclk, k210_clk_get_div_val(kclk));
	if (WARN_ON(!divider))
		return 0;

	return parent_rate / divider;
}

static const struct clk_ops k210_clk_ops = {
	.enable		= k210_clk_enable,
	.disable	= k210_clk_disable,
	.set_parent	= k210_clk_set_parent,
	.get_parent	= k210_clk_get_parent,
	.recalc_rate	= k210_clk_get_rate,
};

static const char * const k210_mux_parents[] = { "in0", "pll0" };

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
		init.parent_names = k210_mux_parents;
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

static void __init k210_clk_init(struct device_node *np)
{
	struct device_node *sysctl_np;
	struct clk_hw **hws;
	int i, ret;

	kcl = kzalloc(sizeof(*kcl), GFP_KERNEL);
	if (!kcl)
		return;

	kcl->clk_data = kzalloc(struct_size(kcl->clk_data, hws, K210_NUM_CLKS),
				GFP_KERNEL);
	if (!kcl->clk_data)
		return;

	sysctl_np = of_get_parent(np);
	kcl->regs = of_iomap(sysctl_np, 0);
	of_node_put(sysctl_np);
	if (!kcl->regs) {
		pr_err("%pOFP: failed to map registers\n", np);
		return;
	}

	spin_lock_init(&kcl->clk_lock);
	kcl->clk_data->num = K210_NUM_CLKS;
	hws = kcl->clk_data->hws;
	for (i = 0; i < K210_NUM_CLKS; i++)
		hws[i] = ERR_PTR(-EPROBE_DEFER);

	/* PLLs */
	ret = k210_register_plls(np);
	if (ret)
		return;

	/* aclk: muxed of in0 and pll0, no gate */
	ret = k210_register_aclk(np);
	if (ret)
		return;

	/*
	 * Critical clocks: there are no consumers of the SRAM clocks,
	 * including the AI clock for the third SRAM bank. The CPU clock
	 * is only referenced by the uarths serial device and so would be
	 * disabled if the serial console is disabled. Mark all these clocks
	 * as critical so that they are never disabled by the core clock
	 * management.
	 */
	hws[K210_CLK_CPU] =
		k210_register_clk(K210_CLK_CPU,
				  "cpu", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_SRAM0] =
		k210_register_clk(K210_CLK_SRAM0,
				  "sram0", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_SRAM1] =
		k210_register_clk(K210_CLK_SRAM1,
				  "sram1", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_AI] =
		 k210_register_clk(K210_CLK_AI,
				   "ai", "pll1", CLK_IS_CRITICAL);

	/* Clocks with aclk as source */
	hws[K210_CLK_DMA] =
		k210_register_clk(K210_CLK_DMA, "dma", "aclk", 0);
	hws[K210_CLK_FFT] =
		k210_register_clk(K210_CLK_FFT, "fft", "aclk", 0);
	hws[K210_CLK_ROM] =
		k210_register_clk(K210_CLK_ROM, "rom", "aclk", 0);
	hws[K210_CLK_DVP] =
		k210_register_clk(K210_CLK_DVP, "dvp", "aclk", 0);
	hws[K210_CLK_APB0] =
		k210_register_clk(K210_CLK_APB0, "apb0", "aclk", 0);
	hws[K210_CLK_APB1] =
		k210_register_clk(K210_CLK_APB1, "apb1", "aclk", 0);
	hws[K210_CLK_APB2] =
		k210_register_clk(K210_CLK_APB2, "apb2", "aclk", 0);

	/* Clocks with PLL0 as source */
	hws[K210_CLK_SPI0] =
		k210_register_clk(K210_CLK_SPI0, "spi0", "pll0", 0);
	hws[K210_CLK_SPI1] =
		 k210_register_clk(K210_CLK_SPI1, "spi1", "pll0", 0);
	hws[K210_CLK_SPI2] =
		 k210_register_clk(K210_CLK_SPI2, "spi2", "pll0", 0);
	hws[K210_CLK_I2C0] =
		 k210_register_clk(K210_CLK_I2C0, "i2c0", "pll0", 0);
	hws[K210_CLK_I2C1] =
		 k210_register_clk(K210_CLK_I2C1, "i2c1", "pll0", 0);
	hws[K210_CLK_I2C2] =
		 k210_register_clk(K210_CLK_I2C2, "i2c2", "pll0", 0);

	/* Clocks with PLL2 as source */
	hws[K210_CLK_I2S0] =
		 k210_register_clk(K210_CLK_I2S0, "i2s0", "pll2", 0);
	hws[K210_CLK_I2S1] =
		 k210_register_clk(K210_CLK_I2S1, "i2s1", "pll2", 0);
	hws[K210_CLK_I2S2] =
		k210_register_clk(K210_CLK_I2S2, "i2s2", "pll2", 0);
	hws[K210_CLK_I2S0_M] =
		k210_register_clk(K210_CLK_I2S0_M, "i2s0_m", "pll2", 0);
	hws[K210_CLK_I2S1_M] =
		k210_register_clk(K210_CLK_I2S1_M, "i2s1_m", "pll2", 0);
	hws[K210_CLK_I2S2_M] =
		k210_register_clk(K210_CLK_I2S2_M, "i2s2_m", "pll2", 0);

	/* Clocks with IN0 as source */
	hws[K210_CLK_WDT0] =
		k210_register_clk(K210_CLK_WDT0, "wdt0", "in0", 0);
	hws[K210_CLK_WDT1] =
		 k210_register_clk(K210_CLK_WDT1, "wdt1", "in0", 0);
	hws[K210_CLK_RTC] =
		 k210_register_clk(K210_CLK_RTC, "rtc", "in0", 0);

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
	hws[K210_CLK_AES] =
		 k210_register_clk(K210_CLK_AES, "aes", "apb1", 0);
	hws[K210_CLK_OTP] =
		 k210_register_clk(K210_CLK_OTP, "otp", "apb1", 0);

	/* Muxed clocks with in0 or pll0 as source */
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
			pr_err("%pOFP: register clock %d failed %ld\n",
			       np, i, PTR_ERR(hws[i]));
			return;
		}
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, kcl->clk_data);
	if (ret) {
		pr_err("%pOFP: add clock provider failed %d\n", np, ret);
		return;
	}

	pr_info("%pOFP: CPU running at %lu MHz\n",
		np, clk_hw_get_rate(hws[K210_CLK_CPU]) / 1000000);
}

CLK_OF_DECLARE(k210_clk, "canaan,k210-clk", k210_clk_init);

/*
 * Enable PLL1 to be able to use the AI SRAM.
 */
void __init k210_clk_early_init(void __iomem *regs)
{
	struct k210_pll pll1;

	/* Make sure ACLK selector is set to PLL0 */
	k210_aclk_set_selector(1);

	/* Startup PLL1 to enable the aisram bank for general memory use */
	k210_init_pll(&pll1, K210_PLL1, regs);
	k210_pll_enable_hw(&pll1);
}
