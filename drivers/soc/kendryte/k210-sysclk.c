// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2019 Christoph Hellwig.
 * Copyright (C) 2019-20 Sean Anderson <seanga2@gmail.com>
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 */
#define pr_fmt(fmt)     "k210-sysclk: " fmt

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

static struct k210_pll_cfg k210_plls_cfg[] = {
	{ K210_SYSCTL_PLL0,  0, 2, 0, 59, 1, 59 },
	{ K210_SYSCTL_PLL1,  8, 1, 0, 59, 3, 59 },
	{ K210_SYSCTL_PLL2, 16, 1, 0, 22, 1, 22 },
};

/*
 * Clocks data.
 */
struct k210_sysclk {
	void __iomem			*regs;
	spinlock_t			lock;
	struct k210_pll			plls[K210_PLL_NUM];
	struct clk_hw			aclk;
	struct clk_hw_onecell_data	*clk_data;
};

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
	u32 reg = readl(pll->reg);

	if (k210_pll_hw_is_enabled(pll))
		return;

	/* Set factors */
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
}

static void k210_pll_disable_hw(struct k210_pll *pll)
{
	u32 reg = readl(pll->reg);

        /*
         * Bypassing before powering off is important so child clocks don't stop
         * working. This is especially important for pll0, the indirect parent
         * of the cpu clock.
         */
        reg |= K210_PLL_BYPASS;
        writel(reg, pll->reg);

        reg &= ~K210_PLL_PWRD;
        reg &= ~K210_PLL_EN;
        writel(reg, pll->reg);
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
	u32 reg;

	switch (pll->id) {
	case K210_PLL0:
	case K210_PLL1:
		if (WARN_ON(index != 0))
			return -EINVAL;
		return 0;
	case K210_PLL2:
		if (WARN_ON(index > 2))
			return -EINVAL;
		reg = readl(pll->reg);
		reg &= ~K210_PLL_SEL;
		reg |= FIELD_PREP(K210_PLL_SEL, index);
		writel(reg, pll->reg);
		return 0;
	default:
		pr_err("Invalid pll\n");
		return -EINVAL;
	}
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

static void k210_init_pll(struct k210_pll *pll, enum k210_pll_id id,
			  void __iomem *base)
{
	pll->id = id;
	pll->lock = base + K210_SYSCTL_PLL_LOCK;
	pll->reg = base + k210_plls_cfg[id].reg;
	pll->shift = k210_plls_cfg[id].shift;
	pll->width = k210_plls_cfg[id].width;
}

#define to_k210_sysclk(hw)	container_of(hw, struct k210_sysclk, aclk)

static int k210_aclk_set_parent(struct clk_hw *hw, u8 index)
{
	struct k210_sysclk *ksc = to_k210_sysclk(hw);
	u32 reg = readl(ksc->regs + K210_SYSCTL_SEL0);

	if (WARN_ON(index > 1))
		return -EINVAL;

	/* 0: IN0, 1: PLL0 */
	if (index)
		reg |= K210_ACLK_SEL;
	else
		reg &= K210_ACLK_SEL;
	writel(reg, ksc->regs + K210_SYSCTL_SEL0);

	return 0;
}

static u8 k210_aclk_get_parent(struct clk_hw *hw)
{
	struct k210_sysclk *ksc = to_k210_sysclk(hw);
	u32 sel = readl(ksc->regs + K210_SYSCTL_SEL0);

	return (sel & K210_ACLK_SEL) ? 1 : 0;
}

static unsigned long k210_aclk_get_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct k210_sysclk *ksc = to_k210_sysclk(hw);
	u32 reg = readl(ksc->regs + K210_SYSCTL_SEL0);
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

/*
 * Muxed clocks.
 */
#define _MUXID(id)	K210_CLK_MUX_##id
#define MUXID(id)	_MUXID(id)

enum k210_mux_ids {
	MUXID(SPI3), MUXID(TIMER0), MUXID(TIMER1), MUXID(TIMER2), MUXID(NONE),
};

struct k210_mux_params {
	const char *const *parent_names;
	u8 num_parents;
	u8 off;
	u8 shift;
	u8 width;
	u32 *table;
};

#define MUX(id, parents, _off, _shift, _width, _table)		 \
	[MUXID(id)] = {						 \
		.parent_names = (const char * const *)(parents), \
		.num_parents = ARRAY_SIZE(parents),		 \
		.off = (_off),					 \
		.shift = (_shift),				 \
		.width = (_width),				 \
		.table = (_table),				 \
	}

static const char *pll_parents[] = { NULL, "pll0", "pll1" };
static const char *aclk_parents[] = { NULL, "pll0" };
static const char *spt_parents[] = { "in0_half", "pll0_half" };
static u32 generic_table[] = { 0, 1 };

static const struct k210_mux_params k210_muxes[] = {
	MUX(SPI3,   spt_parents,  K210_SYSCTL_SEL0, 12, 1, generic_table),
	MUX(TIMER0, spt_parents,  K210_SYSCTL_SEL0, 13, 1, generic_table),
	MUX(TIMER1, spt_parents,  K210_SYSCTL_SEL0, 14, 1, generic_table),
	MUX(TIMER2, spt_parents,  K210_SYSCTL_SEL0, 15, 1, generic_table)
};

static struct clk_mux *k210_create_mux(struct k210_sysclk *ksc,
				       enum k210_mux_ids id)
{
	const struct k210_mux_params *mp = &k210_muxes[id];
	struct clk_mux *mux;

	mux = kzalloc(sizeof(*mux), GFP_KERNEL);
	if (mux) {
		mux->reg = ksc->regs + mp->off;
		mux->shift = mp->shift;
		mux->mask = BIT(mp->width) - 1;
		mux->table = mp->table;
		mux->lock = &ksc->lock;
	}

	return mux;
}

/*
 * Divider clocks.
 */
#define _DIVID(id)	K210_CLK_DIV_##id
#define DIVID(id)	_DIVID(id)

enum k210_div_ids {
	DIVID(SRAM0), DIVID(SRAM1), DIVID(ROM), DIVID(DVP),
	DIVID(APB0), DIVID(APB1), DIVID(APB2), DIVID(AI),
	DIVID(I2S0), DIVID(I2S1), DIVID(I2S2),
	DIVID(I2S0_M), DIVID(I2S1_M), DIVID(I2S2_M),
	DIVID(WDT0), DIVID(WDT1),
	DIVID(SPI0), DIVID(SPI1), DIVID(SPI2),
	DIVID(I2C0), DIVID(I2C1), DIVID(I2C2),
	DIVID(SPI3), DIVID(TIMER0), DIVID(TIMER1), DIVID(TIMER2),
};

struct k210_div_params {
	u8 off;
	u8 shift;
	u8 width;
	u8 flags;
};

#define DIV(id, _off, _shift, _width, _flags)	\
	[DIVID(id)] = {				\
		.off = (_off),			\
		.shift = (_shift),		\
		.width = (_width),		\
		.flags = (_flags),		\
	}

static const struct k210_div_params k210_divs[] = {
	DIV(SRAM0,  K210_SYSCTL_THR0,  0,  4, 0),
	DIV(SRAM1,  K210_SYSCTL_THR0,  4,  4, 0),
	DIV(ROM,    K210_SYSCTL_THR0, 16,  4, 0),
	DIV(DVP,    K210_SYSCTL_THR0, 12,  4, 0),
	DIV(APB0,   K210_SYSCTL_SEL0,  3,  3, 0),
	DIV(APB1,   K210_SYSCTL_SEL0,  6,  3, 0),
	DIV(APB2,   K210_SYSCTL_SEL0,  9,  3, 0),

	DIV(AI,     K210_SYSCTL_THR0,  8,  4, 0),

	DIV(I2S0,   K210_SYSCTL_THR3,  0, 16, 0),
	DIV(I2S1,   K210_SYSCTL_THR3, 16, 16, 0),
	DIV(I2S2,   K210_SYSCTL_THR4,  0, 16, 0),
	DIV(I2S0_M, K210_SYSCTL_THR4, 16,  8, 0),
	DIV(I2S1_M, K210_SYSCTL_THR4, 24,  8, 0),
	DIV(I2S2_M, K210_SYSCTL_THR4,  0,  8, 0),

	DIV(WDT0,   K210_SYSCTL_THR6,  0,  8, 0),
	DIV(WDT1,   K210_SYSCTL_THR6,  8,  8, 0),

	DIV(SPI0,   K210_SYSCTL_THR1,  0,  8, 0),
	DIV(SPI1,   K210_SYSCTL_THR1,  8,  8, 0),
	DIV(SPI2,   K210_SYSCTL_THR1, 16,  8, 0),
	DIV(I2C0,   K210_SYSCTL_THR5,  8,  8, 0),
	DIV(I2C1,   K210_SYSCTL_THR5, 16,  8, 0),
	DIV(I2C2,   K210_SYSCTL_THR5, 24,  8, 0),

	DIV(SPI3,   K210_SYSCTL_THR1, 24,  8, 0),
	DIV(TIMER0, K210_SYSCTL_THR2,  0,  8, 0),
	DIV(TIMER1, K210_SYSCTL_THR2,  8,  8, 0),
	DIV(TIMER2, K210_SYSCTL_THR2, 16,  8, 0),

};

static struct clk_divider *k210_create_divider(struct k210_sysclk *ksc,
					       enum k210_div_ids id)
{
	const struct k210_div_params *dp = &k210_divs[id];
	struct clk_divider *div;

	div = kzalloc(sizeof(*div), GFP_KERNEL);
	if (div) {
		div->reg = ksc->regs + dp->off;
		div->shift = dp->shift;
		div->width = dp->width;
		div->flags = dp->flags;
		div->lock = &ksc->lock;
	}

	return div;
}

/*
 * Gated clocks.
 */
#define _GATEID(id) K210_CLK_GATE_##id
#define GATEID(id) _GATEID(id)

enum k210_gate_ids {
	GATEID(CPU), GATEID(SRAM0), GATEID(SRAM1),
	GATEID(APB0), GATEID(APB1), GATEID(APB2),
	GATEID(ROM), GATEID(DMA), GATEID(AI), GATEID(DVP),
	GATEID(FFT), GATEID(GPIO),
	GATEID(SPI0), GATEID(SPI1), GATEID(SPI2), GATEID(SPI3),
	GATEID(I2S0), GATEID(I2S1), GATEID(I2S2),
	GATEID(I2C0), GATEID(I2C1), GATEID(I2C2),
	GATEID(UART1), GATEID(UART2), GATEID(UART3),
	GATEID(AES), GATEID(FPIOA),
	GATEID(TIMER0), GATEID(TIMER1), GATEID(TIMER2),
	GATEID(WDT0), GATEID(WDT1), GATEID(SHA), GATEID(OTP), GATEID(RTC),
	GATEID(NONE),
};

struct k210_gate_params {
	u8 off;
	u8 bit_idx;
	u32 flags;
};

#define GATE(id, _off, _idx, _flags)	\
	[GATEID(id)] = {		\
		.off = (_off),		\
		.bit_idx = (_idx),	\
		.flags = (_flags),	\
	}

static const struct k210_gate_params k210_gates[] = {
	GATE(CPU,    K210_SYSCTL_EN_CENT,  0, 0),
	GATE(SRAM0,  K210_SYSCTL_EN_CENT,  1, 0),
	GATE(SRAM1,  K210_SYSCTL_EN_CENT,  2, 0),
	GATE(APB0,   K210_SYSCTL_EN_CENT,  3, 0),
	GATE(APB1,   K210_SYSCTL_EN_CENT,  4, 0),
	GATE(APB2,   K210_SYSCTL_EN_CENT,  5, 0),
	GATE(ROM,    K210_SYSCTL_EN_PERI,  0, 0),
	GATE(DMA,    K210_SYSCTL_EN_PERI,  1, 0),
	GATE(AI,     K210_SYSCTL_EN_PERI,  2, 0),
	GATE(DVP,    K210_SYSCTL_EN_PERI,  3, 0),
	GATE(FFT,    K210_SYSCTL_EN_PERI,  4, 0),
	GATE(GPIO,   K210_SYSCTL_EN_PERI,  5, 0),
	GATE(SPI0,   K210_SYSCTL_EN_PERI,  6, 0),
	GATE(SPI1,   K210_SYSCTL_EN_PERI,  7, 0),
	GATE(SPI2,   K210_SYSCTL_EN_PERI,  8, 0),
	GATE(SPI3,   K210_SYSCTL_EN_PERI,  9, 0),
	GATE(I2S0,   K210_SYSCTL_EN_PERI, 10, 0),
	GATE(I2S1,   K210_SYSCTL_EN_PERI, 11, 0),
	GATE(I2S2,   K210_SYSCTL_EN_PERI, 12, 0),
	GATE(I2C0,   K210_SYSCTL_EN_PERI, 13, 0),
	GATE(I2C1,   K210_SYSCTL_EN_PERI, 14, 0),
	GATE(I2C2,   K210_SYSCTL_EN_PERI, 15, 0),
	GATE(UART1,  K210_SYSCTL_EN_PERI, 16, 0),
	GATE(UART2,  K210_SYSCTL_EN_PERI, 17, 0),
	GATE(UART3,  K210_SYSCTL_EN_PERI, 18, 0),
	GATE(AES,    K210_SYSCTL_EN_PERI, 19, 0),
	GATE(FPIOA,  K210_SYSCTL_EN_PERI, 20, 0),
	GATE(TIMER0, K210_SYSCTL_EN_PERI, 21, 0),
	GATE(TIMER1, K210_SYSCTL_EN_PERI, 22, 0),
	GATE(TIMER2, K210_SYSCTL_EN_PERI, 23, 0),
	GATE(WDT0,   K210_SYSCTL_EN_PERI, 24, 0),
	GATE(WDT1,   K210_SYSCTL_EN_PERI, 25, 0),
	GATE(SHA,    K210_SYSCTL_EN_PERI, 26, 0),
	GATE(OTP,    K210_SYSCTL_EN_PERI, 27, 0),
	GATE(RTC,    K210_SYSCTL_EN_PERI, 29, 0),
};

static struct clk_gate *k210_create_gate(struct k210_sysclk *ksc,
					 enum k210_gate_ids id)
{
	const struct k210_gate_params *gp = &k210_gates[id];
	struct clk_gate *gate;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (gate) {
		gate->reg = ksc->regs + gp->off;
		gate->bit_idx = gp->bit_idx;
		gate->flags = gp->flags;
		gate->lock = &ksc->lock;
	}

	return gate;
}

/*
 * Composite clocks.
 */
#define _COMPID(id)	K210_CLK_COMP_##id
#define COMPID(id)	_COMPID(id)

enum k210_comp_ids {
	COMPID(SPI3), COMPID(TIMER0), COMPID(TIMER1), COMPID(TIMER2),
	COMPID(SRAM0), COMPID(SRAM1), COMPID(ROM), COMPID(DVP),
	COMPID(APB0), COMPID(APB1), COMPID(APB2), COMPID(AI),
	COMPID(I2S0), COMPID(I2S1), COMPID(I2S2), COMPID(WDT0), COMPID(WDT1),
	COMPID(SPI0), COMPID(SPI1), COMPID(SPI2),
	COMPID(I2C0), COMPID(I2C1), COMPID(I2C2),
};

struct k210_comp_params {
	u8 mux;
	u8 div;
	u8 gate;
};

#define COMP(id, _mux, _div, _gate)		\
	[COMPID(id)] = {			\
		.mux = (_mux),			\
		.div = (_div),			\
		.gate = (_gate),		\
	}
#define COMP_MUX(id) COMP(id, MUXID(id), DIVID(id), GATEID(id))
#define COMP_NOGATE(id) COMP(id, MUXID(id), DIVID(id), GATEID(NONE))
#define COMP_NOMUX(id) COMP(id, MUXID(NONE), DIVID(id), GATEID(id))

static const struct k210_comp_params k210_comps[] = {
	COMP_MUX(SPI3),
	COMP_MUX(TIMER0),
	COMP_MUX(TIMER1),
	COMP_MUX(TIMER2),
	COMP_NOMUX(SRAM0),
	COMP_NOMUX(SRAM1),
	COMP_NOMUX(ROM),
	COMP_NOMUX(DVP),
	COMP_NOMUX(APB0),
	COMP_NOMUX(APB1),
	COMP_NOMUX(APB2),
	COMP_NOMUX(AI),
	COMP_NOMUX(I2S0),
	COMP_NOMUX(I2S1),
	COMP_NOMUX(I2S2),
	COMP_NOMUX(WDT0),
	COMP_NOMUX(WDT1),
	COMP_NOMUX(SPI0),
	COMP_NOMUX(SPI1),
	COMP_NOMUX(SPI2),
	COMP_NOMUX(I2C0),
	COMP_NOMUX(I2C1),
	COMP_NOMUX(I2C2)
};

static struct clk_hw *k210_register_pll(struct k210_sysclk *ksc,
				enum k210_pll_id id, const char *name,
				const char **parent_names, int num_parents,
				unsigned long flags)
{
        struct k210_pll *pll = &ksc->plls[id];
        struct clk_init_data init = {};
	int ret;

	k210_init_pll(pll, id, ksc->regs);

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

static struct clk_hw *k210_register_aclk(struct k210_sysclk *ksc)
{
        struct clk_init_data init = {};
	int ret;

        init.name = "aclk";
        init.parent_names = aclk_parents;
        init.num_parents = 2;
        init.flags = CLK_IS_CRITICAL;
        init.ops = &k210_aclk_ops;
	ksc->aclk.init = &init;

	ret = clk_hw_register(NULL, &ksc->aclk);
	if (ret)
		return ERR_PTR(ret);

	return &ksc->aclk;
}

static struct clk_hw *k210_register_divider(struct k210_sysclk *ksc,
					enum k210_div_ids id,
					const char *name, const char *parent,
					unsigned long flags)
{
	const struct k210_div_params *dp = &k210_divs[id];

	return clk_hw_register_divider(NULL, name, parent, flags,
				       ksc->regs + dp->off,
				       dp->shift, dp->width, dp->flags,
				       &ksc->lock);
}

static struct clk_hw *k210_register_gate(struct k210_sysclk *ksc,
					enum k210_gate_ids id,
					const char *name, const char *parent,
					unsigned long flags)
{
	const struct k210_gate_params *gp = &k210_gates[id];

	return clk_hw_register_gate(NULL, name, parent, flags,
				 ksc->regs + gp->off,
				 gp->bit_idx, gp->flags, &ksc->lock);
}

static struct clk_hw *k210_register_composite(struct k210_sysclk *ksc,
					enum k210_comp_ids id,
					const char *name, const char *parent,
					unsigned long flags)
{
	const struct k210_comp_params *cp = &k210_comps[id];
	const char * const *parent_names;
	int num_parents;
	struct clk_mux *mux = NULL;
	struct clk_hw *mux_hw = NULL;
	const struct clk_ops *mux_ops = NULL;
	struct clk_divider *div = NULL;
	struct clk_gate *gate = NULL;
	struct clk_hw *gate_hw = NULL;
	const struct clk_ops *gate_ops = NULL;
	struct clk_hw *hw;

	if (cp->mux != K210_CLK_MUX_NONE) {
		mux = k210_create_mux(ksc, cp->mux);
		if (!mux)
			return ERR_PTR(-ENOMEM);

		parent_names = k210_muxes[cp->mux].parent_names;
		num_parents = k210_muxes[cp->mux].num_parents;
		mux_hw = &mux->hw;
		mux_ops = &clk_mux_ops;
	} else {
		parent_names = &parent;
		num_parents = 1;
	}

	div = k210_create_divider(ksc, cp->div);
	if (!div) {
		hw = ERR_PTR(-ENOMEM);
		goto out;
	}

	if (cp->gate != K210_CLK_GATE_NONE) {
		gate = k210_create_gate(ksc, cp->gate);
		if (!gate) {
			hw = ERR_PTR(-ENOMEM);
			goto out;
		}
		gate_hw = &gate->hw;
		gate_ops = &clk_gate_ops;
	}

	hw = clk_hw_register_composite(NULL, name,
				       parent_names, num_parents,
				       mux_hw, mux_ops,
				       &div->hw, &clk_divider_ops,
				       gate_hw, &clk_gate_ops, flags);

out:
	if (IS_ERR(hw)) {
		kfree(gate);
		kfree(div);
		kfree(mux);
	}

	return hw;
}

static struct k210_sysclk *ksc;
static void k210_sysclk_dump_info(struct k210_sysclk *ksc);

static __init void k210_sysclk_init(struct device_node *np)
{
	struct clk *in0_clk;
	const char *in0;
	struct clk_hw **hws;
	int i, ret;

	pr_info("K210 system controller\n");

	ksc = kzalloc(sizeof(*ksc), GFP_KERNEL);
	if (!ksc) {
		pr_err("data allocation failed\n");
		return;
	}

	ksc->regs = ioremap(K210_SYSCTL_BASE_ADDR, K210_SYSCTL_BASE_SIZE);
	if (!ksc->regs) {
		pr_err("map sysctl resources failed\n");
		goto err;
	}

	spin_lock_init(&ksc->lock);
	ksc->clk_data = kzalloc(struct_size(ksc->clk_data, hws, K210_NUM_CLKS),
				GFP_KERNEL);
	if (!ksc->clk_data) {
		pr_err("clk data allocation failed\n");
		goto err;
	}

	ksc->clk_data->num = K210_NUM_CLKS;
	hws = ksc->clk_data->hws;
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
	pr_info("fixed-rate %lu MHz %s base clock\n",
		clk_hw_get_rate(hws[K210_CLK_IN0]) / 1000000, in0);
	aclk_parents[0] = in0;
	pll_parents[0] = in0;

	/* PLLs */
	hws[K210_CLK_PLL0] =
		k210_register_pll(ksc, K210_PLL0, "pll0",
				  pll_parents, 1, CLK_IS_CRITICAL);
	hws[K210_CLK_PLL1] =
		k210_register_pll(ksc, K210_PLL1, "pll1",
				  pll_parents, 1, CLK_IS_CRITICAL);
	hws[K210_CLK_PLL2] =
		k210_register_pll(ksc, K210_PLL2, "pll2", pll_parents, 3, 0);

	/* Half rate clocks */
	hws[K210_CLK_IN0_H] =
		clk_hw_register_fixed_factor(NULL, "in0_half", in0,
					     0, 1, 2);
	hws[K210_CLK_PLL0_H] =
		clk_hw_register_fixed_factor(NULL, "pll0_half", "pll0",
					     0, 1, 2);
	hws[K210_CLK_PLL2_H] =
		clk_hw_register_fixed_factor(NULL, "pll2_half", "pll2",
					     0, 1, 2);

	/* aclk: muxed of in0 and pll0_d, no gate */
	hws[K210_CLK_ACLK] = k210_register_aclk(ksc);

	/* Gated clocks with aclk as source */
	hws[K210_CLK_CPU] =
		k210_register_gate(ksc, GATEID(CPU),
				   "cpu", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_DMA] =
		k210_register_gate(ksc, GATEID(DMA), "dma", "aclk", 0);
	hws[K210_CLK_FFT] =
		k210_register_gate(ksc, GATEID(FFT), "fft", "aclk", 0);
	hws[K210_CLK_SRAM0] =
		k210_register_composite(ksc, COMPID(SRAM0),
					"sram0", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_SRAM1] =
		k210_register_composite(ksc, COMPID(SRAM1),
					"sram1", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_ROM] =
		k210_register_composite(ksc, COMPID(ROM), "rom", "aclk", 0);
	hws[K210_CLK_DVP] =
		k210_register_composite(ksc, COMPID(DVP), "dvp", "aclk", 0);
	hws[K210_CLK_APB0] =
		k210_register_composite(ksc, COMPID(APB0),
					"apb0", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_APB1] =
		k210_register_composite(ksc, COMPID(APB1),
					"apb1", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_APB2] =
		k210_register_composite(ksc, COMPID(APB2),
					"apb2", "aclk", CLK_IS_CRITICAL);

	/* Composite clocks, not muxed */
	/*
	 * The KPU is not supported, so do not create its AI clock.
	 * hws[K210_CLK_AI] =
	 *	k210_register_composite(COMPID(AI), "ai", "pll1", 0);
	 */
	hws[K210_CLK_AI] = NULL;

	/* Gated divider clocks with pll2 (half) as source */
	hws[K210_CLK_I2S0] =
		k210_register_composite(ksc, COMPID(I2S0),
					"i2s0", "pll2_half", 0);
	hws[K210_CLK_I2S1] =
		k210_register_composite(ksc, COMPID(I2S1),
					"i2s1", "pll2_half", 0);
	hws[K210_CLK_I2S2] =
		k210_register_composite(ksc, COMPID(I2S2),
					"i2s2", "pll2_half", 0);
	hws[K210_CLK_I2S0_M] =
		k210_register_divider(ksc, DIVID(I2S0_M),
				      "i2s0_m", "pll2_half", 0);
	hws[K210_CLK_I2S1_M] =
		k210_register_divider(ksc, DIVID(I2S1_M),
				      "i2s1_m", "pll2_half", 0);
	hws[K210_CLK_I2S2_M] =
		k210_register_divider(ksc, DIVID(I2S2_M),
				      "i2s2_m", "pll2_half", 0);

	/* Gated divider clocks with in0 (half) as source */
	hws[K210_CLK_WDT0] =
		k210_register_composite(ksc, COMPID(WDT0),
					"wdt0", "in0_half", 0);
	hws[K210_CLK_WDT1] =
		k210_register_composite(ksc, COMPID(WDT1),
					"wdt1", "in0_half", 0);

	/* Gated divider clocks with pll0 (half) as source */
	hws[K210_CLK_SPI0] =
		k210_register_composite(ksc, COMPID(SPI0),
					"spi0", "pll0_half", 0);
	hws[K210_CLK_SPI1] =
		k210_register_composite(ksc, COMPID(SPI1),
					"spi1", "pll0_half", 0);
	hws[K210_CLK_SPI2] =
		k210_register_composite(ksc, COMPID(SPI2),
					"spi2", "pll0_half", 0);
	hws[K210_CLK_I2C0] =
		k210_register_composite(ksc, COMPID(I2C0),
					"i2c0", "pll0_half", 0);
	hws[K210_CLK_I2C1] =
		k210_register_composite(ksc, COMPID(I2C1),
					"i2c1", "pll0_half", 0);
	hws[K210_CLK_I2C2] =
		k210_register_composite(ksc, COMPID(I2C2),
					"i2c2", "pll0_half", 0);

	/* Muxed gated divider clocks with in0/pll0 (half) as source */
	hws[K210_CLK_SPI3] =
		k210_register_composite(ksc, COMPID(SPI3), "spi3", NULL, 0);
	hws[K210_CLK_TIMER0] =
		k210_register_composite(ksc, COMPID(TIMER0), "timer0", NULL, 0);
	hws[K210_CLK_TIMER1] =
		k210_register_composite(ksc, COMPID(TIMER1), "timer1", NULL, 0);
	hws[K210_CLK_TIMER2] =
		k210_register_composite(ksc, COMPID(TIMER2), "timer2", NULL, 0);

	/* Gated clocks with apb0 as source */
	hws[K210_CLK_GPIO] =
		k210_register_gate(ksc, GATEID(GPIO), "gpio", "apb0", 0);
	hws[K210_CLK_UART1] =
		k210_register_gate(ksc, GATEID(UART1), "uart1", "apb0", 0);
	hws[K210_CLK_UART2] =
		k210_register_gate(ksc, GATEID(UART2), "uart2", "apb0", 0);
	hws[K210_CLK_UART3] =
		k210_register_gate(ksc, GATEID(UART3), "uart3", "apb0", 0);
	hws[K210_CLK_FPIOA] =
		k210_register_gate(ksc, GATEID(FPIOA), "fpioa", "apb0", 0);
	hws[K210_CLK_SHA] =
		k210_register_gate(ksc, GATEID(SHA), "sha", "apb0", 0);

	/* Gated clocks with apb1 as source */
	hws[K210_CLK_AES] =
		k210_register_gate(ksc, GATEID(AES), "aes", "apb1", 0);
	hws[K210_CLK_OTP] =
		k210_register_gate(ksc, GATEID(OTP), "otp", "apb1", 0);

	/* Gated clocks with in0 as source */
	hws[K210_CLK_RTC] =
		k210_register_gate(ksc, GATEID(RTC), "rtc", in0, 0);

	/* clint: 1/50 the CPU rate */
	hws[K210_CLK_CLINT] =
		clk_hw_register_fixed_factor(NULL, "clint", "cpu",
					     CLK_IS_CRITICAL, 1, 50);

	for (i = 0; i < K210_NUM_CLKS; i++) {
		if (IS_ERR(hws[i])) {
			pr_err("register clock %d failed %ld\n",
			       i, PTR_ERR(hws[i]));
			goto err;
		}
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, ksc->clk_data);
	if (ret)
		pr_err("add clock provider failed %d\n", ret);

	return;
err:
	pr_err("clock initialization failed\n");
	iounmap(ksc->regs);
	kfree(ksc->clk_data);
	kfree(ksc);
	ksc = NULL;
}

CLK_OF_DECLARE_DRIVER(k210_sysclk, "kendryte,k210-sysclk", k210_sysclk_init);

static u32 pll_regs[K210_PLL_NUM];
static u32 clk_sel;

static void k210_get_cfg(void __iomem *regs)
{
	int i;

	for (i = 0; i < K210_PLL_NUM; i++)
		pll_regs[i] = readl(regs + k210_plls_cfg[i].reg);

	clk_sel = readl(regs + K210_SYSCTL_SEL0);
}

/*
 * Enable PLL1 to be able to use the AI SRAM.
 */
void k210_sysclk_early_init(void __iomem *regs)
{
	struct k210_pll pll1;

	k210_get_cfg(regs);

	k210_init_pll(&pll1, K210_PLL1, regs);
	k210_pll_enable_hw(&pll1);
}

static void k210_sysclk_show_clk(struct k210_sysclk *ksc, int id,
				 int level, char *str, bool *shown)
{
	struct clk_hw *hw = ksc->clk_data->hws[id];

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

static void k210_sysclk_dump_clk(struct k210_sysclk *ksc, struct clk_hw *parent,
				 int level, char *str, bool *shown)
{
	struct clk_hw *hw, *h;
	bool has_prev, has_next;
	int i, j;

	for (i = 0; i < ksc->clk_data->num; i++) {
		hw = ksc->clk_data->hws[i];

		if (!hw || parent == hw || shown[i])
			continue;

		if (clk_hw_get_parent(hw) != parent)
			continue;

		k210_sysclk_show_clk(ksc, i, level, str, shown);

		has_prev = false;
		for (j = i - 1; j >= 0; j--) {
			h = ksc->clk_data->hws[j];
			if (clk_hw_get_parent(h) == parent) {
				has_prev = true;
				break;
			}
		}

		has_next = false;
		for (j = i + 1; j < ksc->clk_data->num; j++) {
			h = ksc->clk_data->hws[j];
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

		k210_sysclk_dump_clk(ksc, hw, level + 1, str, shown);

		if (level > 0)
			str[strlen(str) - 5] = '\0';
	}
}

static void k210_sysclk_dump_info(struct k210_sysclk *ksc)
{
	static bool shown[K210_NUM_CLKS];
	char str[64];
	int i;

	pr_info("-------------------------------------\n");

	memset(shown, 0, sizeof(shown));
	memset(str, 0, sizeof(str));

	pr_info("PLLs initial factors (from HW):\n");
	for (i = 0; i < K210_PLL_NUM; i++)
		pr_info("    PLL%d: r = %lu, f = %lu, od = %lu, bwadj = %lu\n",
			i, FIELD_GET(K210_PLL_CLKR, pll_regs[i]),
			FIELD_GET(K210_PLL_CLKF, pll_regs[i]),
			FIELD_GET(K210_PLL_CLKOD, pll_regs[i]),
			FIELD_GET(K210_PLL_BWADJ, pll_regs[i]));

	pr_info("Clock selectors:\n");
	pr_info("    ACLK: sel = %lu, div shift = %lu, div = %lu\n",
		clk_sel & K210_ACLK_SEL,
		FIELD_GET(K210_ACLK_DIV, clk_sel),
		2UL << FIELD_GET(K210_ACLK_DIV, clk_sel));
	pr_info("    PLL2: sel = %lu\n",
		FIELD_GET(K210_PLL_SEL, pll_regs[2]));

	pr_info("PLLs factors in use:\n");
	for (i = 0; i < K210_PLL_NUM; i++)
		pr_info("    PLL%d: r = %u, f = %u, od = %u, bwadj = %u\n",
			i, k210_plls_cfg[i].r,
			k210_plls_cfg[i].f,
			k210_plls_cfg[i].od,
			k210_plls_cfg[i].bwadj);

	pr_info("K210 clock tree:\n");
	k210_sysclk_show_clk(ksc, K210_CLK_IN0, 0, str, shown);
	strcat(str, " ");
	k210_sysclk_dump_clk(ksc, ksc->clk_data->hws[K210_CLK_IN0],
			     1, str, shown);

	pr_info("-------------------------------------\n");
}

static int k210_sysclk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_info(dev, "K210 clock driver\n");

	k210_sysclk_dump_info(ksc);

	return 0;
}

static const struct of_device_id k210_sysclk_of_match[] = {
	{ .compatible = "kendryte,k210-sysclk", },
	{}
};

static struct platform_driver k210_sysclk_driver = {
	.driver	= {
		.name		= "k210-sysclk",
		.of_match_table	= k210_sysclk_of_match,
	},
	.probe			= k210_sysclk_probe,
};
builtin_platform_driver(k210_sysclk_driver);
