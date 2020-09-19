// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2019 Christoph Hellwig.
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 */
#define pr_fmt(fmt)     "k210-sysclk: " fmt

#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/bitfield.h>
#include <asm/soc.h>

#include "k210-sysctl.h"

static inline u32 k210_read_reg(void __iomem *base, u8 reg)
{
	return readl(base + reg);
}

static inline void k210_write_reg(void __iomem *base, u8 reg, u32 val)
{
	return writel(val, base + reg);
}

static inline void k210_set_bits(void __iomem *base, u8 reg, u32 bits)
{
	k210_write_reg(base, reg, k210_read_reg(base, reg) | bits);
}

static inline void k210_clear_bits(void __iomem *base, u8 reg, u32 bits)
{
	k210_write_reg(base, reg, k210_read_reg(base, reg) & ~bits);
}

struct k210_sysclk {
	void __iomem *sysctl_base;
	struct clk_hw_onecell_data *clk_data;
	spinlock_t lock;
};

/*
 * in0: fixed-rate 26MHz oscillator base clock.
 */
#define K210_IN0_RATE		26000000UL

/*
 * PLL control register bits.
 */
#define   PLL_CLKR		GENMASK(3, 0)
#define   PLL_CLKF		GENMASK(9, 4)
#define   PLL_CLKOD		GENMASK(13, 10)
#define   PLL_BWADJ		GENMASK(19, 14)
#define   PLL_RESET		(1 << 20)
#define   PLL_PWR		(1 << 21)
#define   PLL_INTFB		(1 << 22)
#define   PLL_BYPASS		(1 << 23)
#define   PLL_TEST		(1 << 24)
#define   PLL_OUT_EN		(1 << 25)
#define   PLL_CKIN_SEL		GENMASK(27, 26) /* PLL2 only */

/*
 * PLL lock register bits.
 */
#define   PLL_LOCK		0
#define   PLL_CLEAR_SLIP	2
#define   PLL_TEST_OUT		3

/*
 * PLLs.
 */
struct k210_pll {
	u8 reg; /* Offset of the PLL register */
	u8 shift; /* Offset of lock bits in the PLL lock register */
	u8 width; /* Width of lock bits to test against */
	u32 clkr, clkf, clkod, bwadj; /* Initial factors */
	void __iomem *sysctl_base;
	struct clk_hw hw;
};
#define to_k210_pll(hw)	container_of(hw, struct k210_pll, hw)

enum k210_pll_ids {
	K210_PLL0, K210_PLL1, K210_PLL2,
};

static struct k210_pll k210_plls[] = {
	{ K210_SYSCTL_PLL0,  0, 2, 0, 59, 3, 59, NULL, {} },
	{ K210_SYSCTL_PLL1,  8, 1, 0, 59, 3, 59, NULL, {} },
	{ K210_SYSCTL_PLL2, 16, 1, 0, 59, 3, 59, NULL, {} },
};

static bool k210_pll_is_locked(void __iomem *base, struct k210_pll *pll)
{
	u32 mask = GENMASK(pll->width - 1, 0) << pll->shift;
	u32 val = k210_read_reg(base, K210_SYSCTL_PLL_LOCK);

	return (val & mask) == mask;
}

static void __k210_pll_enable(void __iomem *base, struct k210_pll *pll)
{
	u32 val = k210_read_reg(base, pll->reg);

	if ((val & PLL_PWR) && !(val & PLL_RESET))
		return;

	/* Set factors */
	val &= ~GENMASK(19, 0);
	val |= FIELD_PREP(PLL_CLKR, pll->clkr);
	val |= FIELD_PREP(PLL_CLKF, pll->clkf);
	val |= FIELD_PREP(PLL_CLKOD, pll->clkod);
	val |= FIELD_PREP(PLL_BWADJ, pll->bwadj);
	val |= PLL_PWR;
	k210_write_reg(base, pll->reg, val);

	/*
	 * Reset the pll. The magic NOPs come from the Kendryte reference SDK.
	 */
	k210_clear_bits(base, pll->reg, PLL_RESET);
	k210_set_bits(base, pll->reg, PLL_RESET);
        nop();
        nop();
	k210_clear_bits(base, pll->reg, PLL_RESET);

	/* Wait for PLL lock */
	while (k210_pll_is_locked(base, pll))
		k210_clear_bits(base, K210_SYSCTL_PLL_LOCK,
				BIT(pll->shift + PLL_CLEAR_SLIP));

	k210_clear_bits(base, pll->reg, PLL_BYPASS);
	k210_set_bits(base, pll->reg, PLL_OUT_EN);
}

void k210_enable_pll1(void __iomem *base)
{
	__k210_pll_enable(base, &k210_plls[K210_PLL1]);
}

static int k210_pll_enable(struct clk_hw *hw)
{
	struct k210_pll *pll = to_k210_pll(hw);

	__k210_pll_enable(pll->sysctl_base, pll);

	return 0;
}

static void k210_pll_disable(struct clk_hw *hw)
{
	struct k210_pll *pll = to_k210_pll(hw);

	/*
	 * Bypassing before powering off is important so child clocks don't stop
	 * working. This is especially important for pll0, the indirect parent
	 * of the cpu clock.
	 */
	k210_set_bits(pll->sysctl_base, pll->reg, PLL_BYPASS);
	k210_clear_bits(pll->sysctl_base, pll->reg, PLL_PWR);
}

static int k210_pll_is_enabled(struct clk_hw *hw)
{
	struct k210_pll *pll = to_k210_pll(hw);
	u32 val = k210_read_reg(pll->sysctl_base, pll->reg);

	return (val | PLL_PWR) && !(val | PLL_RESET);
}

static unsigned long k210_pll_get_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct k210_pll *pll = to_k210_pll(hw);
	u32 val = k210_read_reg(pll->sysctl_base, pll->reg);
	u32 clkr, clkf, clkod;

	if (val & PLL_BYPASS)
		return parent_rate;

	if (!(val & PLL_PWR))
		return 0;

	/* freq = base frequency * clkf0 / (clkr0 * clkod0) */
	clkr = FIELD_GET(PLL_CLKR, val) + 1;
	clkf = FIELD_GET(PLL_CLKF, val) + 1;
	clkod = FIELD_GET(PLL_CLKOD, val) + 1;

	return (u64)parent_rate * clkf / (clkr * clkod);
}

static const struct clk_ops k210_pll_ops = {
	.enable		= k210_pll_enable,
	.disable	= k210_pll_disable,
	.is_enabled	= k210_pll_is_enabled,
	.recalc_rate	= k210_pll_get_rate,
};

static struct clk_hw *k210_register_pll(struct k210_sysclk *ksc,
				enum k210_pll_ids id,
				const char *name, const char *parent_name,
				unsigned long flags)
{
        struct clk_init_data init = {};
        struct k210_pll *pll = &k210_plls[id];
	int ret;

        init.name = name;
        init.parent_names = &parent_name;
        init.num_parents = 1;
        init.flags = flags;
        init.ops = &k210_pll_ops;
        pll->hw.init = &init;

	ret = clk_hw_register(NULL, &pll->hw);
	if (ret) {
		pr_err("register PLL%d failed\n", id);
		return ERR_PTR(ret);
	}

	return &pll->hw;
}

/*
 * Muxed clocks.
 */
#define _MUXID(id)	K210_CLK_MUX_##id
#define MUXID(id)	_MUXID(id)

enum k210_mux_ids {
	MUXID(PLL2), MUXID(ACLK), MUXID(SPI3),
	MUXID(TIMER0), MUXID(TIMER1), MUXID(TIMER2),
	MUXID(NONE),
};

struct k210_mux_params {
	const char *const *parent_names;
	u8 num_parents;
	u8 off;
	u8 shift;
	u8 width;
};

#define MUX(id, parents, _off, _shift, _width) \
	[MUXID(id)] = {						 \
		.parent_names = (const char * const *)(parents), \
		.num_parents = ARRAY_SIZE(parents),		 \
		.off = (_off),					 \
		.shift = (_shift),				 \
		.width = (_width),				 \
	}

static const char * const generic_sels[] = { "in0_half", "pll0_half" };
static const char *aclk_sels[] = { NULL, "pll0_half" };
static const char *pll2_sels[] = { NULL, "pll0", "pll1" };

static const struct k210_mux_params k210_muxes[] = {
	MUX(PLL2,   pll2_sels,    K210_SYSCTL_PLL2, 26, 2),
	MUX(ACLK,   aclk_sels,    K210_SYSCTL_SEL0,  0, 1),
	MUX(SPI3,   generic_sels, K210_SYSCTL_SEL0, 12, 1),
	MUX(TIMER0, generic_sels, K210_SYSCTL_SEL0, 13, 1),
	MUX(TIMER1, generic_sels, K210_SYSCTL_SEL0, 14, 1),
	MUX(TIMER2, generic_sels, K210_SYSCTL_SEL0, 15, 1)
};

static struct clk_mux *k210_create_mux(struct k210_sysclk *ksc,
				       enum k210_mux_ids id)
{
	const struct k210_mux_params *mp = &k210_muxes[id];
	struct clk_mux *mux;

	mux = kzalloc(sizeof(*mux), GFP_KERNEL);
	if (mux) {
		mux->reg = ksc->sysctl_base + mp->off;
		mux->shift = mp->shift;
		mux->mask = BIT(mp->width) - 1;
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
	DIVID(ACLK), DIVID(APB0), DIVID(APB1), DIVID(APB2),
	DIVID(SRAM0), DIVID(SRAM1), DIVID(AI), DIVID(DVP), DIVID(ROM),
	DIVID(SPI0), DIVID(SPI1), DIVID(SPI2), DIVID(SPI3),
	DIVID(TIMER0), DIVID(TIMER1), DIVID(TIMER2),
	DIVID(I2S0), DIVID(I2S1), DIVID(I2S2),
	DIVID(I2S0_M), DIVID(I2S1_M), DIVID(I2S2_M),
	DIVID(I2C0), DIVID(I2C1), DIVID(I2C2),
	DIVID(WDT0), DIVID(WDT1),
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
	DIV(ACLK,   K210_SYSCTL_SEL0,  1,  2, CLK_DIVIDER_POWER_OF_TWO),
	DIV(APB0,   K210_SYSCTL_SEL0,  3,  3, 0),
	DIV(APB1,   K210_SYSCTL_SEL0,  6,  3, 0),
	DIV(APB2,   K210_SYSCTL_SEL0,  9,  3, 0),
	DIV(SRAM0,  K210_SYSCTL_THR0,  0,  4, 0),
	DIV(SRAM1,  K210_SYSCTL_THR0,  4,  4, 0),
	DIV(AI,     K210_SYSCTL_THR0,  8,  4, 0),
	DIV(DVP,    K210_SYSCTL_THR0, 12,  4, 0),
	DIV(ROM,    K210_SYSCTL_THR0, 16,  4, 0),
	DIV(SPI0,   K210_SYSCTL_THR1,  0,  8, 0),
	DIV(SPI1,   K210_SYSCTL_THR1,  8,  8, 0),
	DIV(SPI2,   K210_SYSCTL_THR1, 16,  8, 0),
	DIV(SPI3,   K210_SYSCTL_THR1, 24,  8, 0),
	DIV(TIMER0, K210_SYSCTL_THR2,  0,  8, 0),
	DIV(TIMER1, K210_SYSCTL_THR2,  8,  8, 0),
	DIV(TIMER2, K210_SYSCTL_THR2, 16,  8, 0),
	DIV(I2S0,   K210_SYSCTL_THR3,  0, 16, 0),
	DIV(I2S1,   K210_SYSCTL_THR3, 16, 16, 0),
	DIV(I2S2,   K210_SYSCTL_THR4,  0, 16, 0),
	DIV(I2S0_M, K210_SYSCTL_THR4, 16,  8, 0),
	DIV(I2S1_M, K210_SYSCTL_THR4, 24,  8, 0),
	DIV(I2S2_M, K210_SYSCTL_THR4,  0,  8, 0),
	DIV(I2C0,   K210_SYSCTL_THR5,  8,  8, 0),
	DIV(I2C1,   K210_SYSCTL_THR5, 16,  8, 0),
	DIV(I2C2,   K210_SYSCTL_THR5, 24,  8, 0),
	DIV(WDT0,   K210_SYSCTL_THR6,  0,  8, 0),
	DIV(WDT1,   K210_SYSCTL_THR6,  8,  8, 0),
};

static struct clk_divider *k210_create_divider(struct k210_sysclk *ksc,
					       enum k210_div_ids id)
{
	const struct k210_div_params *dp = &k210_divs[id];
	struct clk_divider *div;

	div = kzalloc(sizeof(*div), GFP_KERNEL);
	if (div) {
		div->reg = ksc->sysctl_base + dp->off;
		div->shift = dp->shift;
		div->width = dp->width;
		div->flags = dp->flags;
		div->lock = &ksc->lock;
	}

	return div;
}

static struct clk_hw *k210_register_divider(struct k210_sysclk *ksc,
					enum k210_div_ids id,
					const char *name, const char *parent,
					unsigned long flags)
{
	const struct k210_div_params *dp = &k210_divs[id];

	return clk_hw_register_divider(NULL, name, parent, flags,
				       ksc->sysctl_base + dp->off,
				       dp->shift, dp->width, dp->flags,
				       &ksc->lock);
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
		gate->reg = ksc->sysctl_base + gp->off;
		gate->bit_idx = gp->bit_idx;
		gate->flags = gp->flags;
		gate->lock = &ksc->lock;
	}

	return gate;
}

static struct clk_hw *k210_register_gate(struct k210_sysclk *ksc,
					enum k210_gate_ids id,
					const char *name, const char *parent,
					unsigned long flags)
{
	const struct k210_gate_params *gp = &k210_gates[id];

	return clk_hw_register_gate(NULL, name, parent, flags,
				 ksc->sysctl_base + gp->off,
				 gp->bit_idx, gp->flags, &ksc->lock);
}

/*
 * Composite clocks.
 */
#define _COMPID(id)	K210_CLK_COMP_##id
#define COMPID(id)	_COMPID(id)

enum k210_comp_ids {
	COMPID(ACLK), COMPID(SPI3),
	COMPID(TIMER0), COMPID(TIMER1), COMPID(TIMER2),
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
	COMP_NOGATE(ACLK),
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

static __init void k210_sysclk_setup(struct device_node *np)
{
	struct device_node *sysctl_np;
	const char *in0 = "in0";
	struct k210_sysclk *ksc;
	struct clk_hw **hws;
	struct clk_mux *mux;
	struct k210_pll *pll;
	int i, ret;

	pr_info("registering %d clocks\n",
		K210_NUM_CLKS);

	ksc = kzalloc(sizeof(*ksc), GFP_KERNEL);
	if (!ksc) {
		pr_err("sysclk allocation failed\n");
		return;
	}

	/* Get iomem resources from parent sysctl node */
	sysctl_np = of_find_compatible_node(NULL, NULL,
					    "kendryte,k210-sysctl");
	if (!sysctl_np) {
		pr_err("sysctl node not found\n");
		return;
	}
	ksc->sysctl_base = of_iomap(sysctl_np, 0);
	of_node_put(sysctl_np);
	if (!ksc->sysctl_base) {
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
	k210_plls[K210_PLL0].sysctl_base = ksc->sysctl_base;
	k210_plls[K210_PLL1].sysctl_base = ksc->sysctl_base;
	k210_plls[K210_PLL2].sysctl_base = ksc->sysctl_base;

	/*
	 * in0 is the system base fixed-rate 26MHz oscillator which
	 * should already be defined by the device tree.
	 */
	i = of_property_match_string(np, "clock-names", in0);
        if (i < 0) {
		pr_warn("in0 oscillator not found\n");
		hws[K210_CLK_IN0] =
			clk_hw_register_fixed_rate(NULL, in0, NULL,
						CLK_IS_CRITICAL, K210_IN0_RATE);
	} else {
		in0 = of_clk_get_parent_name(np, i);
		hws[K210_CLK_IN0] = NULL;
	}

	pr_info("fixed-rate %lu MHz %s\n",
		K210_IN0_RATE / 1000000, in0);
	aclk_sels[0] = in0;
	pll2_sels[0] = in0;

	/*
	 * Critical clocks first: mark these clocks as crtitical so that they
	 * are never disabled even if no driver is using them.
	 */

	/* pll0 */
	hws[K210_CLK_PLL0] =
		k210_register_pll(ksc, K210_PLL0,
				  "pll0", in0, CLK_IS_CRITICAL);

	/* pll1 */
	hws[K210_CLK_PLL1] =
		k210_register_pll(ksc, K210_PLL1,
				  "pll1", in0, CLK_IS_CRITICAL);

	/* aclk: muxed clock with divider, no gate */
	hws[K210_CLK_ACLK] =
		k210_register_composite(ksc, COMPID(ACLK),
					"aclk", NULL, CLK_IS_CRITICAL);

	hws[K210_CLK_CPU] =
		k210_register_gate(ksc, GATEID(CPU),
				   "cpu", "aclk", CLK_IS_CRITICAL);

	hws[K210_CLK_SRAM0] =
		k210_register_composite(ksc, COMPID(SRAM0),
					"sram0", "aclk", CLK_IS_CRITICAL);
	hws[K210_CLK_SRAM1] =
		k210_register_composite(ksc, COMPID(SRAM1),
					"sram1", "aclk", CLK_IS_CRITICAL);

	/*
	 * Power bus clocks: make these critical too as all peripherals,
	 * including sysctl/sysclk are attached to them.
	 */
	hws[K210_CLK_APB0] =
		k210_register_composite(ksc, COMPID(APB0),
					"apb0", "aclk", CLK_IGNORE_UNUSED);
	hws[K210_CLK_APB1] =
		k210_register_composite(ksc, COMPID(APB1),
					"apb1", "aclk", CLK_IGNORE_UNUSED);
	hws[K210_CLK_APB2] =
		k210_register_composite(ksc, COMPID(APB2),
					"apb2", "aclk", CLK_IGNORE_UNUSED);
	/* in0_h: in0 half rate */
	hws[K210_CLK_IN0_H] =
		clk_hw_register_fixed_factor(NULL, "in0_half", in0,
					     0, 1, 2);

	/* pll0_h: pll0 half rate */
	hws[K210_CLK_PLL0_H] =
		clk_hw_register_fixed_factor(NULL, "pll0_half", "pll0",
					     0, 1, 2);

	/* pll2: muxed, set up a composite clock */
        pll = &k210_plls[K210_PLL2];
	__k210_pll_enable(pll->sysctl_base, pll);
	mux = k210_create_mux(ksc, MUXID(PLL2));
	if (!mux) {
		pr_err("pll2 mux allocation failed\n");
		goto err;
	}
	hws[K210_CLK_PLL2] =
		clk_hw_register_composite(NULL, "pll2",
					  pll2_sels, ARRAY_SIZE(pll2_sels),
					  &mux->hw, &clk_mux_ops,
					  &pll->hw, &k210_pll_ops,
					  &pll->hw, &k210_pll_ops, 0);

	/* pll2_h: pll2 half rate */
	hws[K210_CLK_PLL2_H] =
		clk_hw_register_fixed_factor(NULL, "pll2_half", "pll2",
					     0, 1, 2);

	/* Composite clocksc, muxed */
	hws[K210_CLK_SPI3] =
		k210_register_composite(ksc, COMPID(SPI3),
					"spi3", NULL, 0);
	hws[K210_CLK_TIMER0] =
		k210_register_composite(ksc, COMPID(TIMER0),
					 "timer0", NULL, 0);
	hws[K210_CLK_TIMER1] =
		k210_register_composite(ksc, COMPID(TIMER1),
					 "timer1", NULL, 0);
	hws[K210_CLK_TIMER2] =
		k210_register_composite(ksc, COMPID(TIMER2),
					 "timer2", NULL, 0);

	/* Composite clocksc, not muxed */
	hws[K210_CLK_ROM] =
		k210_register_composite(ksc, COMPID(ROM),
					"rom", "aclk", 0);
	hws[K210_CLK_DVP] =
		k210_register_composite(ksc, COMPID(DVP),
					"dvp", "aclk", 0);
	/*
	 * The KPU is not supported, so do not create its AI clock.
	 * hws[K210_CLK_AI] =
	 *	k210_register_composite(ksc, COMPID(AI), "ai", "pll1", 0);
	 */
	hws[K210_CLK_AI] = NULL;

	hws[K210_CLK_I2S0] =
		k210_register_composite(ksc, COMPID(I2S0),
					"i2s0", "pll2_half", 0);
	hws[K210_CLK_I2S1] =
		k210_register_composite(ksc, COMPID(I2S1),
					"i2s1", "pll2_half", 0);
	hws[K210_CLK_I2S2] =
		k210_register_composite(ksc, COMPID(I2S2),
					"i2s2", "pll2_half", 0);
	hws[K210_CLK_WDT0] =
		k210_register_composite(ksc, COMPID(WDT0),
					"wdt0", "in0_half", 0);
	hws[K210_CLK_WDT1] =
		k210_register_composite(ksc, COMPID(WDT1),
					"wdt1", "in0_half", 0);
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

	/* Dividing clocks */
	hws[K210_CLK_I2S0_M] =
		k210_register_divider(ksc, DIVID(I2S0_M),
				      "i2s0_m", "pll2_half", 0);
	hws[K210_CLK_I2S1_M] =
		k210_register_divider(ksc, DIVID(I2S1_M),
				      "i2s1_m", "pll2_half", 0);
	hws[K210_CLK_I2S2_M] =
		k210_register_divider(ksc, DIVID(I2S2_M),
				      "i2s2_m", "pll2_half", 0);

	/* Gated clocks */
	hws[K210_CLK_DMA] =
		k210_register_gate(ksc, GATEID(DMA), "dma", "aclk", 0);
	hws[K210_CLK_FFT] =
		k210_register_gate(ksc, GATEID(FFT), "fft", "aclk", 0);
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
	hws[K210_CLK_AES] =
		k210_register_gate(ksc, GATEID(AES), "aes", "apb1", 0);
	hws[K210_CLK_OTP] =
		k210_register_gate(ksc, GATEID(OTP), "otp", "apb1", 0);
	hws[K210_CLK_RTC] =
		k210_register_gate(ksc, GATEID(RTC), "rtc", in0, 0);

	/* clint: 1/50 the CPU rate */
	hws[K210_CLK_CLINT] =
		clk_hw_register_fixed_factor(NULL, "clint", "cpu",
					     0, 1, 50);

	for (i = 0; i < K210_NUM_CLKS; i++) {
		if (IS_ERR(hws[i]))
			pr_err("register clock %d failed %ld\n",
			       i, PTR_ERR(hws[i]));
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, ksc->clk_data);
	if (ret)
		pr_err("add clock provider failed %d\n", ret);

	return;

err:
	pr_err("setup failed\n");
	iounmap(ksc->sysctl_base);
	kfree(ksc->clk_data);
	kfree(ksc);
}

CLK_OF_DECLARE_DRIVER(k210_sysclk, "kendryte,k210-sysclk", k210_sysclk_setup);
