// SPDX-License-Identifier: GPL-2.0
/*
 * Synopsys DesignWare PCIe controller debugfs driver
 *
 * Copyright (C) 2025 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Shradha Todi <shradha.t@samsung.com>
 */

#include <linux/debugfs.h>

#include "pcie-designware.h"

#define SD_STATUS_L1LANE_REG		0xb0
#define PIPE_RXVALID			BIT(18)
#define PIPE_DETECT_LANE		BIT(17)
#define LANE_SELECT			GENMASK(3, 0)

#define DWC_DEBUGFS_BUF_MAX		128

struct dwc_pcie_vendor_id {
	u16 vendor_id;
	u16 vsec_rasdes_cap_id;
};

static const struct dwc_pcie_vendor_id dwc_pcie_vendor_ids[] = {
	{PCI_VENDOR_ID_SAMSUNG,	0x2},
	{} /* terminator */
};

/**
 * struct dwc_pcie_rasdes_info - Stores controller common information
 * @ras_cap_offset: RAS DES vendor specific extended capability offset
 * @reg_lock: Mutex used for RASDES shadow event registers
 * @rasdes_dir: Top level debugfs directory entry
 *
 * Any parameter constant to all files of the debugfs hierarchy for a single controller
 * will be stored in this struct. It is allocated and assigned to controller specific
 * struct dw_pcie during initialization.
 */
struct dwc_pcie_rasdes_info {
	u32 ras_cap_offset;
	struct mutex reg_lock;
	struct dentry *rasdes_dir;
};

static ssize_t lane_detect_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct dw_pcie *pci = file->private_data;
	struct dwc_pcie_rasdes_info *rinfo = pci->rasdes_info;
	char debugfs_buf[DWC_DEBUGFS_BUF_MAX];
	ssize_t off = 0;
	u32 val;

	val = dw_pcie_readl_dbi(pci, rinfo->ras_cap_offset + SD_STATUS_L1LANE_REG);
	val = FIELD_GET(PIPE_DETECT_LANE, val);
	if (val)
		off += scnprintf(debugfs_buf, DWC_DEBUGFS_BUF_MAX - off, "Lane Detected\n");
	else
		off += scnprintf(debugfs_buf, DWC_DEBUGFS_BUF_MAX - off, "Lane Undetected\n");

	return simple_read_from_buffer(buf, count, ppos, debugfs_buf, off);
}

static ssize_t lane_detect_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct dw_pcie *pci = file->private_data;
	struct dwc_pcie_rasdes_info *rinfo = pci->rasdes_info;
	u32 lane, val;

	val = kstrtou32_from_user(buf, count, 0, &lane);
	if (val)
		return val;

	val = dw_pcie_readl_dbi(pci, rinfo->ras_cap_offset + SD_STATUS_L1LANE_REG);
	val &= ~(LANE_SELECT);
	val |= FIELD_PREP(LANE_SELECT, lane);
	dw_pcie_writel_dbi(pci, rinfo->ras_cap_offset + SD_STATUS_L1LANE_REG, val);

	return count;
}

static ssize_t rx_valid_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct dw_pcie *pci = file->private_data;
	struct dwc_pcie_rasdes_info *rinfo = pci->rasdes_info;
	char debugfs_buf[DWC_DEBUGFS_BUF_MAX];
	ssize_t off = 0;
	u32 val;

	val = dw_pcie_readl_dbi(pci, rinfo->ras_cap_offset + SD_STATUS_L1LANE_REG);
	val = FIELD_GET(PIPE_RXVALID, val);
	if (val)
		off += scnprintf(debugfs_buf, DWC_DEBUGFS_BUF_MAX - off, "RX Valid\n");
	else
		off += scnprintf(debugfs_buf, DWC_DEBUGFS_BUF_MAX - off, "RX Invalid\n");

	return simple_read_from_buffer(buf, count, ppos, debugfs_buf, off);
}

static ssize_t rx_valid_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return lane_detect_write(file, buf, count, ppos);
}

#define dwc_debugfs_create(name)			\
debugfs_create_file(#name, 0644, rasdes_debug, pci,	\
			&dbg_ ## name ## _fops)

#define DWC_DEBUGFS_FOPS(name)					\
static const struct file_operations dbg_ ## name ## _fops = {	\
	.open = simple_open,				\
	.read = name ## _read,				\
	.write = name ## _write				\
}

DWC_DEBUGFS_FOPS(lane_detect);
DWC_DEBUGFS_FOPS(rx_valid);

void dwc_pcie_rasdes_debugfs_deinit(struct dw_pcie *pci)
{
	struct dwc_pcie_rasdes_info *rinfo = pci->rasdes_info;

	debugfs_remove_recursive(rinfo->rasdes_dir);
	mutex_destroy(&rinfo->reg_lock);
}

int dwc_pcie_rasdes_debugfs_init(struct dw_pcie *pci)
{
	struct dentry *dir, *rasdes_debug;
	struct dwc_pcie_rasdes_info *rasdes_info;
	const struct dwc_pcie_vendor_id *vid;
	char dirname[DWC_DEBUGFS_BUF_MAX];
	struct device *dev = pci->dev;
	int ras_cap;

	for (vid = dwc_pcie_vendor_ids; vid->vendor_id; vid++) {
		ras_cap = dw_pcie_find_vsec_capability(pci, vid->vendor_id,
							vid->vsec_rasdes_cap_id);
		if (ras_cap)
			break;
	}
	if (!ras_cap) {
		dev_dbg(dev, "No RASDES capability available\n");
		return -ENODEV;
	}

	rasdes_info = devm_kzalloc(dev, sizeof(*rasdes_info), GFP_KERNEL);
	if (!rasdes_info)
		return -ENOMEM;

	/* Create main directory for each platform driver */
	snprintf(dirname, DWC_DEBUGFS_BUF_MAX, "dwc_pcie_%s", dev_name(dev));
	dir = debugfs_create_dir(dirname, NULL);
	if (IS_ERR(dir))
		return PTR_ERR(dir);

	/* Create subdirectories for Debug, Error injection, Statistics */
	rasdes_debug = debugfs_create_dir("rasdes_debug", dir);

	mutex_init(&rasdes_info->reg_lock);
	rasdes_info->ras_cap_offset = ras_cap;
	rasdes_info->rasdes_dir = dir;
	pci->rasdes_info = rasdes_info;

	/* Create debugfs files for Debug subdirectory */
	dwc_debugfs_create(lane_detect);
	dwc_debugfs_create(rx_valid);

	return 0;
}
