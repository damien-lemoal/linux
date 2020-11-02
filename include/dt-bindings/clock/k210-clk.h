/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019-20 Sean Anderson <seanga2@gmail.com>
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 */
#ifndef CLOCK_K210_CLK_H
#define CLOCK_K210_CLK_H

/*
 * Kendryte K210 SoC clock identifiers (arbitrary values).
 */
#define K210_CLK_IN0	0
#define K210_CLK_PLL0	1
#define K210_CLK_PLL1	2
#define K210_CLK_PLL2	3
#define K210_CLK_ACLK	4
#define K210_CLK_CPU	5
#define K210_CLK_CLINT	6
#define K210_CLK_DMA	7
#define K210_CLK_FFT	8
#define K210_CLK_SRAM0	9
#define K210_CLK_SRAM1	10
#define K210_CLK_ROM	11
#define K210_CLK_DVP	12
#define K210_CLK_APB0	13
#define K210_CLK_APB1	14
#define K210_CLK_APB2	15
#define K210_CLK_AI	16
#define K210_CLK_I2S0	17
#define K210_CLK_I2S1	18
#define K210_CLK_I2S2	19
#define K210_CLK_I2S0_M	20
#define K210_CLK_I2S1_M	21
#define K210_CLK_I2S2_M	22
#define K210_CLK_WDT0	23
#define K210_CLK_WDT1	24
#define K210_CLK_SPI0	25
#define K210_CLK_SPI1	26
#define K210_CLK_SPI2	27
#define K210_CLK_I2C0	28
#define K210_CLK_I2C1	29
#define K210_CLK_I2C2	30
#define K210_CLK_SPI3	31
#define K210_CLK_TIMER0	32
#define K210_CLK_TIMER1	33
#define K210_CLK_TIMER2	34
#define K210_CLK_GPIO	35
#define K210_CLK_UART1	36
#define K210_CLK_UART2	37
#define K210_CLK_UART3	38
#define K210_CLK_FPIOA	39
#define K210_CLK_SHA	40
#define K210_CLK_AES	41
#define K210_CLK_OTP	42
#define K210_CLK_RTC	43

#define K210_NUM_CLKS	44

#endif /* CLOCK_K210_CLK_H */
