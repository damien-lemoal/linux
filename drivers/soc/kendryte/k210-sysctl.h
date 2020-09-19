/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2019 Christoph Hellwig.
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 */
#ifndef K210_SYSCTL_H
#define K210_SYSCTL_H

#include <dt-bindings/clock/k210-sysclk.h>
#include <dt-bindings/mfd/k210-sysctl.h>
#include <dt-bindings/reset/k210-sysctl.h>

/* System controller registers base address and size */
#define K210_SYSCTL_BASE_ADDR	0x50440000ULL
#define K210_SYSCTL_BASE_SIZE	0x1000

void k210_enable_pll1(void __iomem *base);

#endif /* K210_SYSCTL_H */
