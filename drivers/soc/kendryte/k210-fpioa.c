// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 Sean Anderson <seanga2@gmail.com>
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 */
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/bitfield.h>
#include <linux/regmap.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <asm/io.h>

#include <dt-bindings/pinctrl/k210-pinctrl.h>

#include "../../pinctrl/core.h"
#include "../../pinctrl/pinctrl-utils.h"

/*
 * The K210 only implements 8 drive levels, even though
 * there is register space for 16
 */
#define K210_PC_DRIVE_MASK	GENMASK(11, 8)
#define K210_PC_DRIVE_SHIFT	8
#define K210_PC_DRIVE_0		(0 << K210_PC_DRIVE_SHIFT)
#define K210_PC_DRIVE_1		(1 << K210_PC_DRIVE_SHIFT)
#define K210_PC_DRIVE_2		(2 << K210_PC_DRIVE_SHIFT)
#define K210_PC_DRIVE_3		(3 << K210_PC_DRIVE_SHIFT)
#define K210_PC_DRIVE_4		(4 << K210_PC_DRIVE_SHIFT)
#define K210_PC_DRIVE_5		(5 << K210_PC_DRIVE_SHIFT)
#define K210_PC_DRIVE_6		(6 << K210_PC_DRIVE_SHIFT)
#define K210_PC_DRIVE_7		(7 << K210_PC_DRIVE_SHIFT)
#define K210_PC_DRIVE_MAX	7
#define K210_PC_MODE_MASK	GENMASK(23, 12)

/*
 * output enabled == PC_OE & (PC_OE_INV ^ FUNCTION_OE)
 * where FUNCTION_OE is a physical signal from the function.
 */
#define K210_PC_OE		BIT(12) /* Output Enable */
#define K210_PC_OE_INV		BIT(13) /* INVert Output Enable */
#define K210_PC_DO_OE		BIT(14) /* set Data Out to Output Enable sig */
#define K210_PC_DO_INV		BIT(15) /* INVert final Data Output */
#define K210_PC_PU		BIT(16) /* Pull Up */
#define K210_PC_PD		BIT(17) /* Pull Down */
/* Strong pull up not implemented on K210 */
#define K210_PC_SL		BIT(19) /* reduce SLew rate */
/* Same semantics as OE above */
#define K210_PC_IE		BIT(20) /* Input Enable */
#define K210_PC_IE_INV		BIT(21) /* INVert Input Enable */
#define K210_PC_DI_INV		BIT(22) /* INVert Data Input */
#define K210_PC_ST		BIT(23) /* Schmitt Trigger */
#define K210_PC_DI		BIT(31) /* raw Data Input */

#define K210_PC_BIAS_MASK	(K210_PC_PU & K210_PC_PD)

#define K210_PC_MODE_IN		(K210_PC_IE | K210_PC_ST)
#define K210_PC_MODE_OUT	(K210_PC_DRIVE_7 | K210_PC_OE)
#define K210_PC_MODE_I2C	(K210_PC_MODE_IN | K210_PC_IE_INV | \
				 K210_PC_SL | K210_PC_OE | K210_PC_OE_INV | \
				 K210_PC_PU)
#define K210_PC_MODE_SPI	(K210_PC_MODE_IN | K210_PC_IE_INV | \
				 K210_PC_MODE_OUT | K210_PC_OE_INV)
#define K210_PC_MODE_GPIO	(K210_PC_MODE_IN | K210_PC_MODE_OUT)

#define K210_PF_FUNC		GENMASK(7, 0)
#define K210_PF_DO		BIT(8)
#define K210_PF_PIN		GENMASK(22, 16)

/*
 * struct k210_functions: Pin function group descriptor
 * @name: function name
 * @npins: numbers of pins for the group
 * @pins: array of pins number array
 *
 * The function name come from the device tree fpioa_xxx nodes.
 * E.g., the node:
 * fpioa_uarths: uarths {
 * 	pinmux = <K210_FPIOA(4, K210_PCF_UARTHS_RX)>,
 *		 <K210_FPIOA(5, K210_PCF_UARTHS_TX)>;
 *	};
 * is represented as the function "uarths" with npins = 2 and pins = {4, 5}.
 * @pfs will store each pin function as defined by the K210_FPIOA() macro.
 */
struct k210_function {
	const char *name;
	unsigned int npins;
	unsigned int *pins;
	u32 *pfs;
};

/*
 * struct k210_fpioa: Kendryte K210 FPIOA memory mapped registers
 * @pins: 48 32-bits IO pin registers
 * @tie_en: 256 (one per function) input tie enable bits
 * @tie_val: 256 (one per function) input tie value bits
 */
struct k210_fpioa {
	u32 pins[48];
	u32 tie_en[8];
	u32 tie_val[8];
};

struct k210_fpioa_data {

	struct device *dev;
	struct pinctrl_dev *pctl;

	struct k210_fpioa __iomem *fpioa;
	struct regmap *sysctl_map;
	u32 power_offset;
	struct clk *clk;

	struct k210_function *functions;
	unsigned int nfunctions;
};

/*
 * The K210 has 48 programmable pins.
 * Each pin belong to a power domain.
 * Each domain can be 3.3v or 1.8v.
 */
#define K210_POWER_NDOMAINS	8
#define K210_PINS_PER_DOMAIN	6

struct k210_power_domain {
	const char *name;
	const unsigned int pins[K210_PINS_PER_DOMAIN];
};

static const struct k210_power_domain
k210_power_domains[K210_POWER_NDOMAINS] = {
	[0] = { "A0", {  0,  1,  2,  3,  4,  5 } },
	[1] = { "A1", {  6,  7,  8,  9, 10, 11 } },
	[2] = { "A2", { 12, 13, 14, 15, 16, 17 } },
	[3] = { "B3", { 18, 19, 20, 21, 22, 23 } },
	[4] = { "B4", { 24, 25, 26, 27, 28, 29 } },
	[5] = { "B5", { 30, 31, 32, 33, 34, 35 } },
	[6] = { "C6", { 36, 37, 38, 39, 40, 41 } },
	[7] = { "C7", { 42, 43, 44, 45, 46, 47 } },
};

#define K210_PIN(i)	[i] = PINCTRL_PIN(i, "IO_" #i)

static const struct pinctrl_pin_desc k210_pins[] =
{
	/* Power group A0 */
	K210_PIN(0), K210_PIN(1), K210_PIN(2),
	K210_PIN(3), K210_PIN(4), K210_PIN(5),
	/* Power group A1 */
	K210_PIN(6), K210_PIN(7), K210_PIN(8),
	K210_PIN(9), K210_PIN(10), K210_PIN(11),
	/* Power group A2 */
	K210_PIN(12), K210_PIN(13), K210_PIN(14),
	K210_PIN(15), K210_PIN(16), K210_PIN(17),
	/* Power group B3 */
	K210_PIN(18), K210_PIN(19), K210_PIN(20),
	K210_PIN(21), K210_PIN(22), K210_PIN(23),
	/* Power group B4 */
	K210_PIN(24), K210_PIN(25), K210_PIN(26),
	K210_PIN(27), K210_PIN(28), K210_PIN(29),
	/* Power group B5 */
	K210_PIN(30), K210_PIN(31), K210_PIN(32),
	K210_PIN(33), K210_PIN(34), K210_PIN(35),
	/* Power group C6 */
	K210_PIN(36), K210_PIN(37), K210_PIN(38),
	K210_PIN(39), K210_PIN(40), K210_PIN(41),
	/* Power group C7 */
	K210_PIN(42), K210_PIN(43), K210_PIN(44),
	K210_PIN(45), K210_PIN(46), K210_PIN(47),
};

#define K210_NPINS	ARRAY_SIZE(k210_pins)

enum k210_pinctrl_mode_id {
	K210_PC_DEFAULT_DISABLED,
	K210_PC_DEFAULT_IN,
	K210_PC_DEFAULT_IN_TIE,
	K210_PC_DEFAULT_OUT,
	K210_PC_DEFAULT_I2C,
	K210_PC_DEFAULT_SPI,
	K210_PC_DEFAULT_GPIO,
	K210_PC_DEFAULT_INT13,
};

#define DEFAULT(mode) \
	[K210_PC_DEFAULT_##mode] = K210_PC_MODE_##mode

static const u32 k210_pinconf_mode_id_to_mode[] = {
	[K210_PC_DEFAULT_DISABLED] = 0,
	DEFAULT(IN),
	[K210_PC_DEFAULT_IN_TIE] = K210_PC_MODE_IN,
	DEFAULT(OUT),
	DEFAULT(I2C),
	DEFAULT(SPI),
	DEFAULT(GPIO),
	[K210_PC_DEFAULT_INT13] = K210_PC_MODE_IN | K210_PC_PU,
};

#undef DEFAULT

/*
 * Pin functions configuration information.
 */
struct k210_pcf_info {
	char name[15];
	u8 mode_id;
};

#define K210_FUNC(id, mode)				\
	[K210_PCF_##id] = {				\
		.name = #id,				\
		.mode_id = K210_PC_DEFAULT_##mode	\
	}

static const struct k210_pcf_info k210_pcf_infos[] = {
	K210_FUNC(JTAG_TCLK,		IN),
	K210_FUNC(JTAG_TDI,		IN),
	K210_FUNC(JTAG_TMS,		IN),
	K210_FUNC(JTAG_TDO,		OUT),
	K210_FUNC(SPI0_D0,		SPI),
	K210_FUNC(SPI0_D1,		SPI),
	K210_FUNC(SPI0_D2,		SPI),
	K210_FUNC(SPI0_D3,		SPI),
	K210_FUNC(SPI0_D4,		SPI),
	K210_FUNC(SPI0_D5,		SPI),
	K210_FUNC(SPI0_D6,		SPI),
	K210_FUNC(SPI0_D7,		SPI),
	K210_FUNC(SPI0_SS0,		OUT),
	K210_FUNC(SPI0_SS1,		OUT),
	K210_FUNC(SPI0_SS2,		OUT),
	K210_FUNC(SPI0_SS3,		OUT),
	K210_FUNC(SPI0_ARB,		IN_TIE),
	K210_FUNC(SPI0_SCLK,		OUT),
	K210_FUNC(UARTHS_RX,		IN),
	K210_FUNC(UARTHS_TX,		OUT),
	K210_FUNC(RESV6,		IN),
	K210_FUNC(RESV7,		IN),
	K210_FUNC(CLK_SPI1,		OUT),
	K210_FUNC(CLK_I2C1,		OUT),
	K210_FUNC(GPIOHS0,		GPIO),
	K210_FUNC(GPIOHS1,		GPIO),
	K210_FUNC(GPIOHS2,		GPIO),
	K210_FUNC(GPIOHS3,		GPIO),
	K210_FUNC(GPIOHS4,		GPIO),
	K210_FUNC(GPIOHS5,		GPIO),
	K210_FUNC(GPIOHS6,		GPIO),
	K210_FUNC(GPIOHS7,		GPIO),
	K210_FUNC(GPIOHS8,		GPIO),
	K210_FUNC(GPIOHS9,		GPIO),
	K210_FUNC(GPIOHS10,		GPIO),
	K210_FUNC(GPIOHS11,		GPIO),
	K210_FUNC(GPIOHS12,		GPIO),
	K210_FUNC(GPIOHS13,		GPIO),
	K210_FUNC(GPIOHS14,		GPIO),
	K210_FUNC(GPIOHS15,		GPIO),
	K210_FUNC(GPIOHS16,		GPIO),
	K210_FUNC(GPIOHS17,		GPIO),
	K210_FUNC(GPIOHS18,		GPIO),
	K210_FUNC(GPIOHS19,		GPIO),
	K210_FUNC(GPIOHS20,		GPIO),
	K210_FUNC(GPIOHS21,		GPIO),
	K210_FUNC(GPIOHS22,		GPIO),
	K210_FUNC(GPIOHS23,		GPIO),
	K210_FUNC(GPIOHS24,		GPIO),
	K210_FUNC(GPIOHS25,		GPIO),
	K210_FUNC(GPIOHS26,		GPIO),
	K210_FUNC(GPIOHS27,		GPIO),
	K210_FUNC(GPIOHS28,		GPIO),
	K210_FUNC(GPIOHS29,		GPIO),
	K210_FUNC(GPIOHS30,		GPIO),
	K210_FUNC(GPIOHS31,		GPIO),
	K210_FUNC(GPIO0,		GPIO),
	K210_FUNC(GPIO1,		GPIO),
	K210_FUNC(GPIO2,		GPIO),
	K210_FUNC(GPIO3,		GPIO),
	K210_FUNC(GPIO4,		GPIO),
	K210_FUNC(GPIO5,		GPIO),
	K210_FUNC(GPIO6,		GPIO),
	K210_FUNC(GPIO7,		GPIO),
	K210_FUNC(UART1_RX,		IN),
	K210_FUNC(UART1_TX,		OUT),
	K210_FUNC(UART2_RX,		IN),
	K210_FUNC(UART2_TX,		OUT),
	K210_FUNC(UART3_RX,		IN),
	K210_FUNC(UART3_TX,		OUT),
	K210_FUNC(SPI1_D0,		SPI),
	K210_FUNC(SPI1_D1,		SPI),
	K210_FUNC(SPI1_D2,		SPI),
	K210_FUNC(SPI1_D3,		SPI),
	K210_FUNC(SPI1_D4,		SPI),
	K210_FUNC(SPI1_D5,		SPI),
	K210_FUNC(SPI1_D6,		SPI),
	K210_FUNC(SPI1_D7,		SPI),
	K210_FUNC(SPI1_SS0,		OUT),
	K210_FUNC(SPI1_SS1,		OUT),
	K210_FUNC(SPI1_SS2,		OUT),
	K210_FUNC(SPI1_SS3,		OUT),
	K210_FUNC(SPI1_ARB,		IN_TIE),
	K210_FUNC(SPI1_SCLK,		OUT),
	K210_FUNC(SPI2_D0,		SPI),
	K210_FUNC(SPI2_SS,		IN),
	K210_FUNC(SPI2_SCLK,		IN),
	K210_FUNC(I2S0_MCLK,		OUT),
	K210_FUNC(I2S0_SCLK,		OUT),
	K210_FUNC(I2S0_WS,		OUT),
	K210_FUNC(I2S0_IN_D0,		IN),
	K210_FUNC(I2S0_IN_D1,		IN),
	K210_FUNC(I2S0_IN_D2,		IN),
	K210_FUNC(I2S0_IN_D3,		IN),
	K210_FUNC(I2S0_OUT_D0,		OUT),
	K210_FUNC(I2S0_OUT_D1,		OUT),
	K210_FUNC(I2S0_OUT_D2,		OUT),
	K210_FUNC(I2S0_OUT_D3,		OUT),
	K210_FUNC(I2S1_MCLK,		OUT),
	K210_FUNC(I2S1_SCLK,		OUT),
	K210_FUNC(I2S1_WS,		OUT),
	K210_FUNC(I2S1_IN_D0,		IN),
	K210_FUNC(I2S1_IN_D1,		IN),
	K210_FUNC(I2S1_IN_D2,		IN),
	K210_FUNC(I2S1_IN_D3,		IN),
	K210_FUNC(I2S1_OUT_D0,		OUT),
	K210_FUNC(I2S1_OUT_D1,		OUT),
	K210_FUNC(I2S1_OUT_D2,		OUT),
	K210_FUNC(I2S1_OUT_D3,		OUT),
	K210_FUNC(I2S2_MCLK,		OUT),
	K210_FUNC(I2S2_SCLK,		OUT),
	K210_FUNC(I2S2_WS,		OUT),
	K210_FUNC(I2S2_IN_D0,		IN),
	K210_FUNC(I2S2_IN_D1,		IN),
	K210_FUNC(I2S2_IN_D2,		IN),
	K210_FUNC(I2S2_IN_D3,		IN),
	K210_FUNC(I2S2_OUT_D0,		OUT),
	K210_FUNC(I2S2_OUT_D1,		OUT),
	K210_FUNC(I2S2_OUT_D2,		OUT),
	K210_FUNC(I2S2_OUT_D3,		OUT),
	K210_FUNC(RESV0,		DISABLED),
	K210_FUNC(RESV1,		DISABLED),
	K210_FUNC(RESV2,		DISABLED),
	K210_FUNC(RESV3,		DISABLED),
	K210_FUNC(RESV4,		DISABLED),
	K210_FUNC(RESV5,		DISABLED),
	K210_FUNC(I2C0_SCLK,		I2C),
	K210_FUNC(I2C0_SDA,		I2C),
	K210_FUNC(I2C1_SCLK,		I2C),
	K210_FUNC(I2C1_SDA,		I2C),
	K210_FUNC(I2C2_SCLK,		I2C),
	K210_FUNC(I2C2_SDA,		I2C),
	K210_FUNC(DVP_XCLK,		OUT),
	K210_FUNC(DVP_RST,		OUT),
	K210_FUNC(DVP_PWDN,		OUT),
	K210_FUNC(DVP_VSYNC,		IN),
	K210_FUNC(DVP_HSYNC,		IN),
	K210_FUNC(DVP_PCLK,		IN),
	K210_FUNC(DVP_D0,		IN),
	K210_FUNC(DVP_D1,		IN),
	K210_FUNC(DVP_D2,		IN),
	K210_FUNC(DVP_D3,		IN),
	K210_FUNC(DVP_D4,		IN),
	K210_FUNC(DVP_D5,		IN),
	K210_FUNC(DVP_D6,		IN),
	K210_FUNC(DVP_D7,		IN),
	K210_FUNC(SCCB_SCLK,		I2C),
	K210_FUNC(SCCB_SDA,		I2C),
	K210_FUNC(UART1_CTS,		IN),
	K210_FUNC(UART1_DSR,		IN),
	K210_FUNC(UART1_DCD,		IN),
	K210_FUNC(UART1_RI,		IN),
	K210_FUNC(UART1_SIR_IN,		IN),
	K210_FUNC(UART1_DTR,		OUT),
	K210_FUNC(UART1_RTS,		OUT),
	K210_FUNC(UART1_OUT2,		OUT),
	K210_FUNC(UART1_OUT1,		OUT),
	K210_FUNC(UART1_SIR_OUT,	OUT),
	K210_FUNC(UART1_BAUD,		OUT),
	K210_FUNC(UART1_RE,		OUT),
	K210_FUNC(UART1_DE,		OUT),
	K210_FUNC(UART1_RS485_EN,	OUT),
	K210_FUNC(UART2_CTS,		IN),
	K210_FUNC(UART2_DSR,		IN),
	K210_FUNC(UART2_DCD,		IN),
	K210_FUNC(UART2_RI,		IN),
	K210_FUNC(UART2_SIR_IN,		IN),
	K210_FUNC(UART2_DTR,		OUT),
	K210_FUNC(UART2_RTS,		OUT),
	K210_FUNC(UART2_OUT2,		OUT),
	K210_FUNC(UART2_OUT1,		OUT),
	K210_FUNC(UART2_SIR_OUT,	OUT),
	K210_FUNC(UART2_BAUD,		OUT),
	K210_FUNC(UART2_RE,		OUT),
	K210_FUNC(UART2_DE,		OUT),
	K210_FUNC(UART2_RS485_EN,	OUT),
	K210_FUNC(UART3_CTS,		IN),
	K210_FUNC(UART3_DSR,		IN),
	K210_FUNC(UART3_DCD,		IN),
	K210_FUNC(UART3_RI,		IN),
	K210_FUNC(UART3_SIR_IN,		IN),
	K210_FUNC(UART3_DTR,		OUT),
	K210_FUNC(UART3_RTS,		OUT),
	K210_FUNC(UART3_OUT2,		OUT),
	K210_FUNC(UART3_OUT1,		OUT),
	K210_FUNC(UART3_SIR_OUT,	OUT),
	K210_FUNC(UART3_BAUD,		OUT),
	K210_FUNC(UART3_RE,		OUT),
	K210_FUNC(UART3_DE,		OUT),
	K210_FUNC(UART3_RS485_EN,	OUT),
	K210_FUNC(TIMER0_TOGGLE1,	OUT),
	K210_FUNC(TIMER0_TOGGLE2,	OUT),
	K210_FUNC(TIMER0_TOGGLE3,	OUT),
	K210_FUNC(TIMER0_TOGGLE4,	OUT),
	K210_FUNC(TIMER1_TOGGLE1,	OUT),
	K210_FUNC(TIMER1_TOGGLE2,	OUT),
	K210_FUNC(TIMER1_TOGGLE3,	OUT),
	K210_FUNC(TIMER1_TOGGLE4,	OUT),
	K210_FUNC(TIMER2_TOGGLE1,	OUT),
	K210_FUNC(TIMER2_TOGGLE2,	OUT),
	K210_FUNC(TIMER2_TOGGLE3,	OUT),
	K210_FUNC(TIMER2_TOGGLE4,	OUT),
	K210_FUNC(CLK_SPI2,		OUT),
	K210_FUNC(CLK_I2C2,		OUT),
	K210_FUNC(INTERNAL0,		OUT),
	K210_FUNC(INTERNAL1,		OUT),
	K210_FUNC(INTERNAL2,		OUT),
	K210_FUNC(INTERNAL3,		OUT),
	K210_FUNC(INTERNAL4,		OUT),
	K210_FUNC(INTERNAL5,		OUT),
	K210_FUNC(INTERNAL6,		OUT),
	K210_FUNC(INTERNAL7,		OUT),
	K210_FUNC(INTERNAL8,		OUT),
	K210_FUNC(INTERNAL9,		IN),
	K210_FUNC(INTERNAL10,		IN),
	K210_FUNC(INTERNAL11,		IN),
	K210_FUNC(INTERNAL12,		IN),
	K210_FUNC(INTERNAL13,		INT13),
	K210_FUNC(INTERNAL14,		I2C),
	K210_FUNC(INTERNAL15,		IN),
	K210_FUNC(INTERNAL16,		IN),
	K210_FUNC(INTERNAL17,		IN),
	K210_FUNC(CONSTANT,		DISABLED),
	K210_FUNC(INTERNAL18,		IN),
	K210_FUNC(DEBUG0,		OUT),
	K210_FUNC(DEBUG1,		OUT),
	K210_FUNC(DEBUG2,		OUT),
	K210_FUNC(DEBUG3,		OUT),
	K210_FUNC(DEBUG4,		OUT),
	K210_FUNC(DEBUG5,		OUT),
	K210_FUNC(DEBUG6,		OUT),
	K210_FUNC(DEBUG7,		OUT),
	K210_FUNC(DEBUG8,		OUT),
	K210_FUNC(DEBUG9,		OUT),
	K210_FUNC(DEBUG10,		OUT),
	K210_FUNC(DEBUG11,		OUT),
	K210_FUNC(DEBUG12,		OUT),
	K210_FUNC(DEBUG13,		OUT),
	K210_FUNC(DEBUG14,		OUT),
	K210_FUNC(DEBUG15,		OUT),
	K210_FUNC(DEBUG16,		OUT),
	K210_FUNC(DEBUG17,		OUT),
	K210_FUNC(DEBUG18,		OUT),
	K210_FUNC(DEBUG19,		OUT),
	K210_FUNC(DEBUG20,		OUT),
	K210_FUNC(DEBUG21,		OUT),
	K210_FUNC(DEBUG22,		OUT),
	K210_FUNC(DEBUG23,		OUT),
	K210_FUNC(DEBUG24,		OUT),
	K210_FUNC(DEBUG25,		OUT),
	K210_FUNC(DEBUG26,		OUT),
	K210_FUNC(DEBUG27,		OUT),
	K210_FUNC(DEBUG28,		OUT),
	K210_FUNC(DEBUG29,		OUT),
	K210_FUNC(DEBUG30,		OUT),
	K210_FUNC(DEBUG31,		OUT),
};

#define PIN_CONFIG_OUTPUT_INVERT	(PIN_CONFIG_END + 1)
#define PIN_CONFIG_INPUT_INVERT		(PIN_CONFIG_END + 2)

static const struct pinconf_generic_params k210_pinconf_params[] = {
	{ "bias-disable",	    PIN_CONFIG_BIAS_DISABLE, 0 },
	{ "bias-pull-down",	    PIN_CONFIG_BIAS_PULL_DOWN, 1 },
	{ "bias-pull-up",	    PIN_CONFIG_BIAS_PULL_UP, 1 },
	{ "drive-strength",	    PIN_CONFIG_DRIVE_STRENGTH, U32_MAX },
	{ "drive-strength-ua",	    PIN_CONFIG_DRIVE_STRENGTH_UA, U32_MAX },
	{ "input-enable",	    PIN_CONFIG_INPUT_ENABLE, 1 },
	{ "input-disable",	    PIN_CONFIG_INPUT_ENABLE, 0 },
	{ "input-schmitt-enable",   PIN_CONFIG_INPUT_SCHMITT_ENABLE, 1 },
	{ "input-schmitt-disable",  PIN_CONFIG_INPUT_SCHMITT_ENABLE, 0 },
	{ "power-source",	    PIN_CONFIG_POWER_SOURCE, K210_PC_POWER_1V8 },
	{ "output-low",		    PIN_CONFIG_OUTPUT, 0 },
	{ "output-high",	    PIN_CONFIG_OUTPUT, 1 },
	{ "output-enable",	    PIN_CONFIG_OUTPUT_ENABLE, 1 },
	{ "output-disable",	    PIN_CONFIG_OUTPUT_ENABLE, 0 },
	{ "slew-rate",		    PIN_CONFIG_SLEW_RATE, 1 },
	{ "output-polarity-invert", PIN_CONFIG_OUTPUT_INVERT, 1 },
	{ "input-polarity-invert",  PIN_CONFIG_INPUT_INVERT, 1 },
};

#define K210_PINCONF_NPARAMS	ARRAY_SIZE(k210_pinconf_params)

/*
 * Max drive strength in uA.
 */
static const int k210_pinconf_drive_strength[] = {
	[0] = 11200,
	[1] = 16800,
	[2] = 22300,
	[3] = 27800,
	[4] = 33300,
	[5] = 38700,
	[6] = 44100,
	[7] = 49500,
};

static int k210_pinconf_get_drive(unsigned int max_strength_ua)
{
	int i;

	for (i = K210_PC_DRIVE_MAX; i; i--) {
		if (k210_pinconf_drive_strength[i] < max_strength_ua)
			return i;
	}

	return -EINVAL;
}

static void k210_pinmux_set_pin_function(struct pinctrl_dev *pctldev,
					 u32 pf)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);
	unsigned pin = FIELD_GET(K210_PF_PIN, pf);
	bool do_oe = FIELD_GET(K210_PF_DO, pf);
	unsigned func = FIELD_GET(K210_PF_FUNC, pf);
	const struct k210_pcf_info *info = &k210_pcf_infos[func];
	u32 mode = k210_pinconf_mode_id_to_mode[info->mode_id];
	u32 val = func | mode | (do_oe ? K210_PC_DO_OE : 0);

	dev_dbg(pdata->dev,
		"set mux (0x%08x): IO_%02u = %03u (%s), "
		"mode %d -> 0x%08x\n",
		pf, pin, func, info->name,
		info->mode_id, mode);

	writel(val, &pdata->fpioa->pins[pin]);
}

static int k210_pinconf_set_param(struct pinctrl_dev *pctldev,
				  unsigned int pin,
				  unsigned int param, unsigned int arg)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);
	u32 val = readl(&pdata->fpioa->pins[pin]);
	int drive;

	dev_dbg(pdata->dev, "set pin %u param %u, arg 0x%x\n",
		pin, param, arg);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		val &= ~K210_PC_BIAS_MASK;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (!arg)
			return -EINVAL;
		val |= K210_PC_PD;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (!arg)
			return -EINVAL;
		val |= K210_PC_PD;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		arg *= 1000;
		/* fallthrough */
	case PIN_CONFIG_DRIVE_STRENGTH_UA:
		drive = k210_pinconf_get_drive(arg);
		if (drive < 0)
			return drive;
		val &= ~K210_PC_DRIVE_MASK;
		val |= FIELD_PREP(K210_PC_DRIVE_MASK, drive);
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		if (arg)
			val |= K210_PC_IE;
		else
			val &= ~K210_PC_IE;
		break;
	case PIN_CONFIG_INPUT_SCHMITT:
		arg = 1;
		/* fallthrough */
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		if (arg)
			val |= K210_PC_ST;
		else
			val &= ~K210_PC_ST;
		break;
	case PIN_CONFIG_OUTPUT:
		k210_pinmux_set_pin_function(pctldev,
					K210_FPIOA(pin, K210_PCF_CONSTANT));
		val = readl(&pdata->fpioa->pins[pin]);
		val |= K210_PC_MODE_OUT;
		if (!arg)
			val |= K210_PC_DO_INV;
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		if (arg)
			val |= K210_PC_OE;
		else
			val &= ~K210_PC_OE;
		break;
	case PIN_CONFIG_SLEW_RATE:
		if (arg)
			val |= K210_PC_SL;
		else
			val &= ~K210_PC_SL;
		break;
	case PIN_CONFIG_OUTPUT_INVERT:
		if (arg)
			val |= K210_PC_DO_INV;
		else
			val &= ~K210_PC_DO_INV;
		break;
	case PIN_CONFIG_INPUT_INVERT:
		if (arg)
			val |= K210_PC_DI_INV;
		else
			val &= ~K210_PC_DI_INV;
		break;
	default:
		return -EINVAL;
	}

	writel(val, &pdata->fpioa->pins[pin]);

	return 0;
}

static int k210_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			    unsigned long *configs, unsigned int num_configs)
{
	unsigned int param, arg;
	int i, ret;

	if (WARN_ON(pin >= K210_NPINS))
		return -EINVAL;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);
		ret = k210_pinconf_set_param(pctldev, pin, param, arg);
		if (ret)
			return ret;
	}

	return 0;
}

static int k210_pinconf_get(struct pinctrl_dev *pctldev,
			    unsigned int pin, unsigned long *config)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	if (pin >= K210_NPINS)
		return -EINVAL;

	*config = readl(&pdata->fpioa->pins[pin]);

	return 0;
}

static void k210_pinconf_dbg_show(struct pinctrl_dev *pctldev,
				  struct seq_file *s, unsigned int pin)
{
	unsigned long config;
	int ret;

	ret = k210_pinconf_get(pctldev, pin, &config);
	if (ret)
		return;

	seq_printf(s, "0x%lx", config);
}

static int k210_pinconf_set_power_domain(struct pinctrl_dev *pctldev,
					 const struct k210_function *function,
					 unsigned int arg)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);
	int last_domain = -1;
	int i, domain;
	u32 bit;

	/*
	 * The function pins may be over several power domains.
	 * Set all of them.
	 */
	for (i = 0; i < function->npins; i++) {
		domain = function->pins[i] / K210_PINS_PER_DOMAIN;
		if (domain == last_domain)
			continue;
		last_domain = domain;

		bit = BIT(domain);
		regmap_update_bits(pdata->sysctl_map,
				   pdata->power_offset, bit,
				   arg ? bit : 0);
	}

	return 0;
}

static void k210_pinconf_get_power_domains(struct k210_fpioa_data *pdata)
{
	int ret, i;
	u32 val;

	ret = regmap_read(pdata->sysctl_map, pdata->power_offset, &val);
	if (ret) {
		dev_err(pdata->dev, "Failed to read power reg\n");
		return;
	}

	dev_dbg(pdata->dev, "Power domains:\n");
	for (i = 0; i < K210_POWER_NDOMAINS; i++)
		dev_dbg(pdata->dev, "    %d: %s V\n",
			i, val & BIT(i) ? "1.8" : "3.3");
}

static int k210_pinconf_group_set(struct pinctrl_dev *pctldev,
				  unsigned int selector, unsigned long *configs,
				  unsigned int num_configs)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);
	const struct k210_function *function;
	unsigned int param, arg;
	int ret, i, j;

	if (WARN_ON(selector >= pdata->nfunctions))
		return -EINVAL;

	function = &pdata->functions[selector];
	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		if (param == PIN_CONFIG_POWER_SOURCE) {
			ret = k210_pinconf_set_power_domain(pctldev,
							    function, arg);
			if (ret)
				return ret;
			continue;
		}

		for (j = 0; j < function->npins; j++) {
			ret = k210_pinconf_set_param(pctldev, function->pins[j],
						     param, arg);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static void k210_pinconf_group_dbg_show(struct pinctrl_dev *pctldev,
					struct seq_file *s,
					unsigned int selector)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);
	const struct k210_function *function;
	unsigned long config;
	unsigned int pin;
	int i, ret;

	if (selector >= pdata->nfunctions)
		return;

	seq_putc(s, '\n');

	function = &pdata->functions[selector];
	for (i = 0; i < function->npins; i++) {
		pin = function->pins[i];
		ret = k210_pinconf_get(pctldev, pin, &config);
		if (ret)
			return;

		seq_printf(s, "%s: 0x%08lx ", k210_pins[pin].name, config);
	}
}

static const struct pinconf_ops k210_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = k210_pinconf_get,
	.pin_config_set = k210_pinconf_set,
	.pin_config_group_set = k210_pinconf_group_set,
	.pin_config_dbg_show = k210_pinconf_dbg_show,
	.pin_config_group_dbg_show = k210_pinconf_group_dbg_show,
};

static int k210_pinmux_get_function_count(struct pinctrl_dev *pctldev)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	return pdata->nfunctions;
}

static const char *k210_pinmux_get_function_name(struct pinctrl_dev *pctldev,
						 unsigned int selector)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	if (WARN_ON(selector >= pdata->nfunctions))
		return NULL;

	return pdata->functions[selector].name;
}

static int k210_pinmux_get_function_groups(struct pinctrl_dev *pctldev,
					   unsigned int selector,
					   const char * const **groups,
					   unsigned int * const num_groups)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);
	struct k210_function *func;

	if (WARN_ON(selector >= pdata->nfunctions))
		return -EINVAL;

	func = &pdata->functions[selector];

	*groups = &pdata->functions[selector].name;
	*num_groups = 1;

	return 0;
}

static int k210_pinmux_set_mux(struct pinctrl_dev *pctldev,
			       unsigned int function,
			       unsigned int group)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);
	struct k210_function *f;
	int i;

	if (WARN_ON(function >= pdata->nfunctions))
		return -EINVAL;

	dev_dbg(pdata->dev, "Set function %s mux\n",
		pdata->functions[function].name);

	f = &pdata->functions[function];
	for (i = 0; i < f->npins; i++)
		k210_pinmux_set_pin_function(pctldev, f->pfs[i]);

	return 0;
}

static const struct pinmux_ops k210_pinmux_ops = {
	.get_functions_count = k210_pinmux_get_function_count,
	.get_function_name = k210_pinmux_get_function_name,
	.get_function_groups = k210_pinmux_get_function_groups,
	.set_mux = k210_pinmux_set_mux,
	.strict = true,
};

static int k210_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	return pdata->nfunctions;
}

static const char *k210_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
					       unsigned int group)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	if (WARN_ON(group >= pdata->nfunctions))
		return NULL;

	return pdata->functions[group].name;
}

static int k210_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
				       unsigned int group,
				       const unsigned int **pins,
				       unsigned int *npins)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	if (WARN_ON(group >= pdata->nfunctions))
		return -EINVAL;

	*pins = pdata->functions[group].pins;
	*npins = pdata->functions[group].npins;

	return 0;
}

static void k210_pinctrl_pin_dbg_show(struct pinctrl_dev *pctldev,
				      struct seq_file *s, unsigned int offset)
{
	seq_printf(s, "%s", dev_name(pctldev->dev));
}

static int k210_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
				       struct device_node *node,
				       struct pinctrl_map **map,
				       unsigned *num_maps)
{
	struct k210_fpioa_data *pdata = pinctrl_dev_get_drvdata(pctldev);
	struct k210_function *function = NULL;
	unsigned int reserved_maps = 0;
	int i, ret;

	*map = NULL;
	*num_maps = 0;

	/* Find the function using the node name */
	for (i = 0; i < pdata->nfunctions; i++) {
		if (!strcmp(node->name, pdata->functions[i].name)) {
			function = &pdata->functions[i];
			break;
		}
	}
	if (!function) {
		dev_err(pdata->dev, "function %s not defined\n", node->name);
		return -ENOTSUPP;
	}

	ret = pinctrl_utils_reserve_map(pctldev, map, &reserved_maps,
					num_maps, 1);
	if (ret) {
		dev_err(pdata->dev, "reserve map failed %d\n", ret);
		return ret;
	}

	ret = pinctrl_utils_add_map_mux(pctldev, map,
					&reserved_maps, num_maps,
					function->name, function->name);
	if (ret) {
		dev_err(pdata->dev, "add function map failed %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct pinctrl_ops k210_pinctrl_ops = {
	.get_groups_count = k210_pinctrl_get_groups_count,
	.get_group_name = k210_pinctrl_get_group_name,
	.get_group_pins = k210_pinctrl_get_group_pins,
	.pin_dbg_show = k210_pinctrl_pin_dbg_show,
	.dt_node_to_map = k210_pinctrl_dt_node_to_map,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static struct pinctrl_desc k210_pinctrl_desc = {
	.name = "k210-pinctrl",
	.pins = k210_pins,
	.npins = K210_NPINS,
	.pctlops = &k210_pinctrl_ops,
	.pmxops = &k210_pinmux_ops,
	.confops = &k210_pinconf_ops,
	.custom_params = k210_pinconf_params,
	.num_custom_params = K210_PINCONF_NPARAMS,
};

static int k210_fpioa_get_functions(struct k210_fpioa_data *pdata)
{
	struct device *dev = pdata->dev;
	struct device_node *child, *np = pdata->dev->of_node;
	struct k210_function *functions;
	struct property *prop;
	const __be32 *cur;
	int nfunctions;
	int ret, p, i = 0;
	u32 pf;

	/* The number of groups is the number of child nodes */
	nfunctions = of_get_child_count(np);
	if (!nfunctions)
		return 0;

	functions = devm_kcalloc(dev, nfunctions,
			      sizeof(struct k210_function), GFP_KERNEL);
	if (!functions)
		return -ENOMEM;

	/* Get the pins in each group */
	for_each_child_of_node(np, child) {
		ret = of_property_count_u32_elems(child, "pinmux");
		if (ret <= 0)
			continue;

		functions[i].name = child->name;
		functions[i].npins = ret;
		functions[i].pins = devm_kcalloc(dev, functions[i].npins,
					sizeof(unsigned int), GFP_KERNEL);
		if (!functions[i].pins)
			return -ENOMEM;

		functions[i].pfs = devm_kcalloc(dev, functions[i].npins,
					sizeof(u32), GFP_KERNEL);
		if (!functions[i].pfs)
			return -ENOMEM;

		dev_dbg(dev, "function %s: %d pins\n",
			functions[i].name, functions[i].npins);
		p = 0;
		of_property_for_each_u32(child, "pinmux", prop, cur, pf) {
			functions[i].pfs[p] = pf;
			functions[i].pins[p] = FIELD_GET(K210_PF_PIN, pf);
			if (functions[i].pins[p] >= K210_NPINS) {
				dev_err(dev,
					"invalid pin %u in function %s\n",
					functions[i].pins[p],
					functions[i].name);
				return -EINVAL;
			}
			dev_dbg(dev, "    pin %u\n", functions[i].pins[p]);
			p++;
		}

		i++;
	}

	pdata->functions = functions;
	pdata->nfunctions = i;

	return 0;
}

static void k210_fpioa_init_ties(struct k210_fpioa_data *pdata)
{
	struct k210_fpioa *fpioa = pdata->fpioa;
	u32 val;
	int i, j;

	dev_dbg(pdata->dev, "Init pin ties\n");

	/* Init pin functions input ties */
	for (i = 0; i < ARRAY_SIZE(fpioa->tie_en); i++) {
		val = 0;
		for (j = 0; j < 32; j++) {
			if (k210_pcf_infos[i * 32 + j].mode_id ==
			    K210_PC_DEFAULT_IN_TIE) {
				dev_dbg(pdata->dev,
					"tie_en function %d (%s)\n",
					i * 32 + j,
					k210_pcf_infos[i * 32 + j].name);
				val |= BIT(j);
			}
		}

		/* Set value before enable */
		writel(val, &fpioa->tie_val[i]);
		writel(val, &fpioa->tie_en[i]);
	}
}

static int k210_fpioa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct k210_fpioa_data *pdata;
	int ret;

	dev_info(dev, "K210 FPIOA controller\n");

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
        if (!pdata) {
		dev_err(dev, "Allocation failed\n");
                return -ENOMEM;
	}

        pdata->dev = dev;
        platform_set_drvdata(pdev, pdata);

	pdata->fpioa = devm_platform_ioremap_resource(pdev, 0);
        if (IS_ERR(pdata->fpioa)) {
		dev_err(dev, "resource mapping failed\n");
                return PTR_ERR(pdata->fpioa);
	}

	pdata->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(pdata->clk)) {
		dev_err(dev, "Get clock failed\n");
		return PTR_ERR(pdata->clk);
	}
	if (ret)
		return ret;

	clk_prepare(pdata->clk);
	ret = clk_enable(pdata->clk);
	if (ret) {
		dev_err(dev, "Enable clock failed\n");
		return ret;
	}

	pdata->sysctl_map =
		syscon_regmap_lookup_by_phandle(np, "kendryte,sysctl");
	if (IS_ERR(pdata->sysctl_map)) {
		dev_err(dev, "Get sysctl regmap failed %ld\n",
			PTR_ERR(pdata->sysctl_map));
		return PTR_ERR(pdata->sysctl_map);
	}

	ret = of_property_read_u32(np, "kendryte,power-offset",
				   &pdata->power_offset);
	if (ret) {
		dev_err(dev, "get power-offset property failed\n");
		return -EINVAL;
	}

	ret = k210_fpioa_get_functions(pdata);
	if (ret) {
		dev_err(dev, "get functions information failed\n");
		return ret;
	}

	k210_fpioa_init_ties(pdata);

	pdata->pctl = pinctrl_register(&k210_pinctrl_desc, dev, (void *)pdata);
	if (IS_ERR(pdata->pctl)) {
		dev_err(dev, "register pinctrl driver failed %ld\n",
			PTR_ERR(pdata->pctl));
		return PTR_ERR(pdata->pctl);
	}

	k210_pinconf_get_power_domains(pdata);

	return 0;
}

static const struct of_device_id k210_fpioa_dt_ids[] = {
	{ .compatible = "kendryte,k210-fpioa" },
};

static struct platform_driver k210_fpioa_driver = {
	.probe	= k210_fpioa_probe,
	.driver = {
		.name		= "k210-fpioa",
		.of_match_table	= k210_fpioa_dt_ids,
	},
};

builtin_platform_driver(k210_fpioa_driver);
