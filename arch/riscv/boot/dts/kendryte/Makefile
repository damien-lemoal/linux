# SPDX-License-Identifier: GPL-2.0
dtb-$(CONFIG_SOC_KENDRYTE_K210_DTB) += $(strip $(shell echo $(CONFIG_SOC_KENDRYTE_K210_DTB_SOURCE))).dtb

obj-$(CONFIG_SOC_KENDRYTE_K210_DTB_BUILTIN) += $(addsuffix .o, $(dtb-y))
