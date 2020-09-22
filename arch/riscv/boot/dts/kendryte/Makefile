# SPDX-License-Identifier: GPL-2.0

k210_dtb_source := $(strip $(shell echo $(CONFIG_SOC_KENDRYTE_K210_DTB_SOURCE)))

$(src)/k210.dts: $(src)/$(k210_dtb_source).dts FORCE
	@ln -sf $(notdir $<) $@

$(obj)/k210.dtb: $(src)/k210.dts $(DTC) FORCE
	$(call if_changed_dep,dtc)

dtb-$(CONFIG_SOC_KENDRYTE_K210_DTB) += k210.dtb

obj-$(CONFIG_SOC_KENDRYTE_K210_DTB_BUILTIN) += $(addsuffix .o, k210.dtb)

clean-files := k210.dts
