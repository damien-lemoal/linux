/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019-20 Sean Anderson <seanga2@gmail.com>
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 */
#ifndef K210_SYSCTL_H
#define K210_SYSCTL_H

#include <linux/io.h>

/* System controller registers base address and size */
#define K210_SYSCTL_BASE_ADDR	0x50440000ULL
#define K210_SYSCTL_BASE_SIZE	0x1000

void k210_clk_early_init(void __iomem *regs);

#endif
