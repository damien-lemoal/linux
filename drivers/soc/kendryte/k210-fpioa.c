// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 */
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/bitfield.h>
#include <linux/spinlock.h>

#include "k210-fpioa.h"

/*
 * FPIOA memory:
 *   - 48 32-bits IO pin register
 *   - 256 bit input tie enable bits
 *   - 256 bit input tie value bits
 * All SPI arbitration signal will tie high by default.
 */
struct k210_fpioa {
	u32 __iomem *io;
	u32 __iomem *tie_en;
	u32 __iomem *tie_val;
};
static struct k210_fpioa fpioa;

/*
 * IO functions configuration: each function is a pin of the chip package.
 * Every IO has a 32bit width register that can independently implement
 * schmitt trigger, invert input, invert output, strong pull up, driving
 * selector, static input and static output.
 * It can implement any pin of any peripheral devices.
 *
 * FPIOA IO's register bits Layout:
 */
#define PAD_DI	BIT(31)		/* Read current IO's data input */
#define RESV2	GENMASK(30, 26)	/* | Reserved bits */
#define TIE_VAL	BIT(25)		/* Input tie value, 1 for high, 0 for low */
#define TIE_EN	BIT(24)		/* Input tie, 1 for enable, 0 for disable */
#define ST	BIT(23)		/* Schmitt trigger enable */
#define DI_INV	BIT(22)		/* Invert Data input */
#define IE_INV	BIT(21)		/* Invert input enable */
#define IE_EN	BIT(20)		/* Static input enable, will AND with IE_INV */
#define SL	BIT(19)		/* Slew rate control enable */
#define SPU	BIT(18)		/* Strong pull up */
#define PD	BIT(17)		/* Pull select: 1 for pull down */
#define PU	BIT(16)		/* Pull up enable. 1 for pull up */
#define DO_INV	BIT(15)		/* Invert data output select (DO_SEL) */
#define DO_SEL	BIT(14)		/* Data output select: 0 for DO, 1 for OE */
#define OE_INV	BIT(13)		/* Invert output enable */
#define OE_EN	BIT(12)		/* Output enable. will AND with OE_INV */
#define DS	GENMASK(11, 8)	/* Current driving selector */
#define CH_SEL	GENMASK(7, 0)	/* Channel select from 256 input */

/* Ignore bit 18 and bits 24-30 */
#define FPIOA_FUNC_CONFIG_MASK	0X80F7FFFF

static const u32 fpioa_func_config[K210_FPIOA_FUNC_NUM] = {
	0x00900000, 0x00900001, 0x00900002, 0x00001f03,
	0x00b03f04, 0x00b03f05, 0x00b03f06, 0x00b03f07,
	0x00b03f08, 0x00b03f09, 0x00b03f0a, 0x00b03f0b,
	0x00001f0c, 0x00001f0d, 0x00001f0e, 0x00001f0f,
	0x03900010, 0x00001f11, 0x00900012, 0x00001f13,
	0x00900014, 0x00900015, 0x00001f16, 0x00001f17,
	0x00901f18, 0x00901f19, 0x00901f1a, 0x00901f1b,
	0x00901f1c, 0x00901f1d, 0x00901f1e, 0x00901f1f,
	0x00901f20, 0x00901f21, 0x00901f22, 0x00901f23,
	0x00901f24, 0x00901f25, 0x00901f26, 0x00901f27,
	0x00901f28, 0x00901f29, 0x00901f2a, 0x00901f2b,
	0x00901f2c, 0x00901f2d, 0x00901f2e, 0x00901f2f,
	0x00901f30, 0x00901f31, 0x00901f32, 0x00901f33,
	0x00901f34, 0x00901f35, 0x00901f36, 0x00901f37,
	0x00901f38, 0x00901f39, 0x00901f3a, 0x00901f3b,
	0x00901f3c, 0x00901f3d, 0x00901f3e, 0x00901f3f,
	0x00900040, 0x00001f41, 0x00900042, 0x00001f43,
	0x00900044, 0x00001f45, 0x00b03f46, 0x00b03f47,
	0x00b03f48, 0x00b03f49, 0x00b03f4a, 0x00b03f4b,
	0x00b03f4c, 0x00b03f4d, 0x00001f4e, 0x00001f4f,
	0x00001f50, 0x00001f51, 0x03900052, 0x00001f53,
	0x00b03f54, 0x00900055, 0x00900056, 0x00001f57,
	0x00001f58, 0x00001f59, 0x0090005a, 0x0090005b,
	0x0090005c, 0x0090005d, 0x00001f5e, 0x00001f5f,
	0x00001f60, 0x00001f61, 0x00001f62, 0x00001f63,
	0x00001f64, 0x00900065, 0x00900066, 0x00900067,
	0x00900068, 0x00001f69, 0x00001f6a, 0x00001f6b,
	0x00001f6c, 0x00001f6d, 0x00001f6e, 0x00001f6f,
	0x00900070, 0x00900071, 0x00900072, 0x00900073,
	0x00001f74, 0x00001f75, 0x00001f76, 0x00001f77,
	0x00000078, 0x00000079, 0x0000007a, 0x0000007b,
	0x0000007c, 0x0000007d, 0x0099107e, 0x0099107f,
	0x00991080, 0x00991081, 0x00991082, 0x00991083,
	0x00001f84, 0x00001f85, 0x00001f86, 0x00900087,
	0x00900088, 0x00900089, 0x0090008a, 0x0090008b,
	0x0090008c, 0x0090008d, 0x0090008e, 0x0090008f,
	0x00900090, 0x00900091, 0x00993092, 0x00993093,
	0x00900094, 0x00900095, 0x00900096, 0x00900097,
	0x00900098, 0x00001f99, 0x00001f9a, 0x00001f9b,
	0x00001f9c, 0x00001f9d, 0x00001f9e, 0x00001f9f,
	0x00001fa0, 0x00001fa1, 0x009000a2, 0x009000a3,
	0x009000a4, 0x009000a5, 0x009000a6, 0x00001fa7,
	0x00001fa8, 0x00001fa9, 0x00001faa, 0x00001fab,
	0x00001fac, 0x00001fad, 0x00001fae, 0x00001faf,
	0x009000b0, 0x009000b1, 0x009000b2, 0x009000b3,
	0x009000b4, 0x00001fb5, 0x00001fb6, 0x00001fb7,
	0x00001fb8, 0x00001fb9, 0x00001fba, 0x00001fbb,
	0x00001fbc, 0x00001fbd, 0x00001fbe, 0x00001fbf,
	0x00001fc0, 0x00001fc1, 0x00001fc2, 0x00001fc3,
	0x00001fc4, 0x00001fc5, 0x00001fc6, 0x00001fc7,
	0x00001fc8, 0x00001fc9, 0x00001fca, 0x00001fcb,
	0x00001fcc, 0x00001fcd, 0x00001fce, 0x00001fcf,
	0x00001fd0, 0x00001fd1, 0x00001fd2, 0x00001fd3,
	0x00001fd4, 0x009000d5, 0x009000d6, 0x009000d7,
	0x009000d8, 0x009100d9, 0x00991fda, 0x009000db,
	0x009000dc, 0x009000dd, 0x000000de, 0x009000df,
	0x00001fe0, 0x00001fe1, 0x00001fe2, 0x00001fe3,
	0x00001fe4, 0x00001fe5, 0x00001fe6, 0x00001fe7,
	0x00001fe8, 0x00001fe9, 0x00001fea, 0x00001feb,
	0x00001fec, 0x00001fed, 0x00001fee, 0x00001fef,
	0x00001ff0, 0x00001ff1, 0x00001ff2, 0x00001ff3,
	0x00001ff4, 0x00001ff5, 0x00001ff6, 0x00001ff7,
	0x00001ff8, 0x00001ff9, 0x00001ffa, 0x00001ffb,
	0x00001ffc, 0x00001ffd, 0x00001ffe, 0x00001fff,
};

static inline u32 k210_fpioa_read_io_reg(int io)
{
	return readl(fpioa.io + io);
}

static inline void k210_fpioa_write_io_reg(int io, u32 val)
{
	writel(val, fpioa.io + io);
}

static inline void k210_fpioa_write_func(int io, int func)
{
	k210_fpioa_write_io_reg(io,
			fpioa_func_config[func] & FPIOA_FUNC_CONFIG_MASK);
}

static int k210_fpioa_get_io_by_function(int function)
{
	u32 ioreg;
	int i;

	for (i = 0; i < K210_FPIOA_IO_NUM; i++) {
		ioreg =  k210_fpioa_read_io_reg(i);
		if (FIELD_GET(CH_SEL, ioreg) == function)
			return i;
	}

	return -1;
}

static void k210_fpioa_set_function(int io, int func)
{
	u32 ioreg;
	int i;

	if (func == FUNC_RESV0) {
		k210_fpioa_write_func(io, FUNC_RESV0);
		return;
	}

	/* Compare all IOs */
	for (i = 0; i < K210_FPIOA_IO_NUM; i++) {
		ioreg = k210_fpioa_read_io_reg(i);
		if (FIELD_GET(CH_SEL, ioreg) == func && i != io)
			k210_fpioa_write_func(i, FUNC_RESV0);
	}

	k210_fpioa_write_func(io, func);
}

static void k210_fpioa_set_pull(int io, enum k210_fpioa_pull pull)
{
	u32 ioreg = k210_fpioa_read_io_reg(io);

	switch (pull) {
	case FPIOA_PULL_NONE:
		ioreg &= ~PU;
		ioreg &= ~PD;
		break;
	case FPIOA_PULL_DOWN:
		ioreg &= ~PU;
		ioreg |= PD;
		break;
	case FPIOA_PULL_UP:
		ioreg |= PU;
		ioreg &= ~PD;
		break;
	default:
		break;
	}

	k210_fpioa_write_io_reg(io, ioreg);
}

static void k210_fpioa_init_io_mux(void)
{
	int pin;

	k210_fpioa_set_function(29, FUNC_SPI0_SCLK);
	k210_fpioa_set_function(30, FUNC_SPI0_D0);
	k210_fpioa_set_function(31, FUNC_SPI0_D1);
	k210_fpioa_set_function(32, FUNC_GPIOHS7);

	k210_fpioa_set_function(24, FUNC_SPI0_SS3);

	k210_fpioa_set_function(33, FUNC_I2S0_OUT_D0);
	k210_fpioa_set_function(35, FUNC_I2S0_SCLK);
	k210_fpioa_set_function(34, FUNC_I2S0_WS);

	pin = k210_fpioa_get_io_by_function(FUNC_GPIOHS0 + 7);
	if (pin < 0)
		pr_warn("k210-fpioa: function FUNC_GPIOHS0 + 7 (%d) not found\n",
			FUNC_GPIOHS0 + 7);
	else
		k210_fpioa_set_pull(pin, FPIOA_PULL_DOWN);
}

static void k210_fpioa_setup(void)
{
	u32 tie_en, tie_val;
	int b, i, j;

	/* Set tie enable and tie value */
	for (i = 0; i < K210_FPIOA_TIE_NUM; i++) {
		tie_en = 0;
		tie_val = 0;

		for (j = 0; j < 32; j++) {
			b = FIELD_GET(GENMASK(24, 24),
				      fpioa_func_config[i * 32 + j]);
			if (b)
				tie_en |= b << j;
			b = FIELD_GET(GENMASK(25, 25),
				      fpioa_func_config[i * 32 + j]);
			if (b)
				tie_val |= b << j;
		}

		/* Set value before enable */
		writel(tie_val, fpioa.tie_val + i);
		writel(tie_en, fpioa.tie_en + i);
	}

	k210_fpioa_init_io_mux();
}

static int k210_fpioa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_info(dev, "Initializing IO multiplexer\n");

	fpioa.io = devm_ioremap_resource(dev,
				platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(fpioa.io)) {
		dev_err(dev, "Map register failed\n");
		return PTR_ERR(fpioa.io);
	}

	fpioa.tie_en = fpioa.io + K210_FPIOA_IO_NUM;
	fpioa.tie_val = fpioa.tie_en + K210_FPIOA_TIE_NUM;

	k210_fpioa_setup();

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
