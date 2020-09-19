/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019-20 Sean Anderson <seanga2@gmail.com>
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 */
#ifndef CLOCK_K210_SYSCLK_H
#define CLOCK_K210_SYSCLK_H

/*
 * Kendryte K210 SoC clock identifiers (arbitrary values).
 */
#define K210_CLK_IN0	0
#define K210_CLK_IN0_H	1
#define K210_CLK_PLL0	2
#define K210_CLK_PLL0_H	3
#define K210_CLK_PLL1	4
#define K210_CLK_PLL2	5
#define K210_CLK_PLL2_H	6

/* Source IN0 or PLL0 divided by (ACLK tho)^2 */
#define K210_CLK_ACLK	7

/* Source ACLK */
#define K210_CLK_CPU	8
#define K210_CLK_DMA	9
#define K210_CLK_FFT	10

/* Source ACLK divided by (tho + 1) */
#define K210_CLK_SRAM0	11
#define K210_CLK_SRAM1	12
#define K210_CLK_ROM	13
#define K210_CLK_DVP	14
#define K210_CLK_APB0	15
#define K210_CLK_APB1	16
#define K210_CLK_APB2	17

/* Source PLL1 divided by (tho + 1) */
#define K210_CLK_AI	18

/* Source PLL2 divided by (tho + 1) * 2 */
#define K210_CLK_I2S0	19
#define K210_CLK_I2S1	20
#define K210_CLK_I2S2	21
#define K210_CLK_I2S0_M	22
#define K210_CLK_I2S1_M	23
#define K210_CLK_I2S2_M	24

/* Source IN0 divided by (tho + 1) * 2 */
#define K210_CLK_WDT0	25
#define K210_CLK_WDT1	26

/* Source PLL0 divided by (tho + 1) * 2 */
#define K210_CLK_SPI0	27
#define K210_CLK_SPI1	28
#define K210_CLK_SPI2	29
#define K210_CLK_I2C0	30
#define K210_CLK_I2C1	31
#define K210_CLK_I2C2	32

/* Source (IN0 or PLL0) divided by (tho + 1) * 2 */
#define K210_CLK_SPI3	33
#define K210_CLK_TIMER0	34
#define K210_CLK_TIMER1	35
#define K210_CLK_TIMER2	36

/* Source APB0 */
#define K210_CLK_GPIO	37
#define K210_CLK_UART1	38
#define K210_CLK_UART2	39
#define K210_CLK_UART3	40
#define K210_CLK_FPIOA	41
#define K210_CLK_SHA	42

/* Source APB1 */
#define K210_CLK_AES	43
#define K210_CLK_OTP	44

/* Source IN0 */
#define K210_CLK_RTC	45

/* Source CPU divided by 50 */
#define K210_CLK_CLINT	46

#define K210_NUM_CLKS	47

#endif /* CLOCK_K210_SYSCLK_H */
