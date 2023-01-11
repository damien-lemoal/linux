// SPDX-License-Identifier: GPL-2.0
/*
 * NVMe function driver for PCI Endpoint Framework
 *
 * Copyright (C) 2019 SiFive
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/nvme.h>

#include "../../../nvme/host/nvme.h"
#include "../../../nvme/target/nvmet.h"

#define TIMER_RESOLUTION		1

/* Minimum and maximum page size (MPS): 4K */
#define PCI_EPF_NVME_MPSMIN		0ULL
#define PCI_EPF_NVME_MPSMAX		0ULL
/* Command sets supported: NVMe command set */
#define PCI_EPF_NVME_CSS		1ULL
/* CC.EN timeout in 500msec units */
#define PCI_EPF_NVME_TO			15ULL
/* Zero-based maximum queue entries */
#define PCI_EPF_NVME_MQES		(NVME_AQ_DEPTH - 1)
/* Maximum queue id */
#define PCI_EPF_NVME_QIDMAX		4
/* Keepalive ticks */
#define PCI_EPF_NVME_KA_TICKS		1000

/* Number of address maps */
#define PCI_EPF_NVME_PRP_MAPS		12
/* Size of address map: 1G = 2^30 */
#define PCI_EPF_NVME_PRP_MAP_SIZE	0x0000000040000000ULL
/* Flag bit to indicate when a prp is mapped */
#define PCI_EPF_NVME_PRP_MAP_FLAG	(1 << 0)
/* No prp marker */
#define PCI_EPF_NVME_PRP_NONE		0xffffffffffffffffULL
/* Size of prp */
#define PCI_EPF_NVME_PRP_SIZE		sizeof(__le64)
/* Maximum # of prps in a page per nvme */
#define PCI_EPF_NVME_PRP_PER_PAGE	\
	(NVME_CTRL_PAGE_SIZE / PCI_EPF_NVME_PRP_SIZE)

/* Number of prp lists supported */
#define PCI_EPF_NVME_PRP_LISTS		1
/* Number of prp list entries supported */
#define PCI_EPF_NVME_PRP_LIST_ENTRIES	\
	(PCI_EPF_NVME_PRP_LISTS * PCI_EPF_NVME_PRP_PER_PAGE)

static struct workqueue_struct *epf_nvme_workqueue;
static struct kmem_cache *epf_nvme_cmd_cache;

struct pci_epf_nvme_map {
	size_t size;
	size_t align;
	off_t offset;
	struct {
		u64 phys_addr;
		u64 phys_base;
		u64 phys_end;
	} host;
	struct {
		size_t size;
		void __iomem *virt_addr;
		void __iomem *virt_base;
		phys_addr_t phys_addr;
		phys_addr_t phys_base;
	} pci;
};

enum pci_epf_nvme_status {
	PCI_EPF_NVME_SYNC = -1,
	PCI_EPF_NVME_ASYNC = -2
};

struct pci_epf_nvme_queue {
	u16 qid;
	u16 size;
	u16 depth;
	u16 flags;
	u16 vector;
	u16 head;
	u16 tail;
	u16 phase;
	u32 db;
	struct pci_epf_nvme_map map;
};

struct pci_epf_nvme_cmb {
	size_t size;
	enum pci_barno bar;
	void *virt_dma_addr;
	u64 phys_dma_addr;
};

struct pci_epf_nvme_features {
	u32 aec;
};

struct pci_epf_nvme_prplist {
	unsigned int count;
	unsigned int index;
	size_t align;
	u64 min;
	u64 max;
	u64 prp[PCI_EPF_NVME_PRP_LIST_ENTRIES];
};

struct pci_epf_nvme_host {
	void __iomem *reg;
	int msi;
	u64 cap;
	u32 vs;
	u32 intms;
	u32 intmc;
	u32 cc;
	u32 csts;
	u32 cmbsz;
	u32 cmbloc;
	u32 aqa;
	u64 asq;
	u64 acq;
	struct pci_epf_nvme_queue sq[PCI_EPF_NVME_QIDMAX + 1];
	struct pci_epf_nvme_queue cq[PCI_EPF_NVME_QIDMAX + 1];
	struct pci_epf_nvme_cmb cmb;
	struct pci_epf_nvme_features features;
};

struct pci_epf_nvme;

struct pci_epf_nvme_cmd {
	struct pci_epf_nvme	*nvme;
	int			qid;
	int			head;
	size_t			transfer_len;
	struct scatterlist 	sgl;
	struct nvmet_req 	req;
	struct nvme_command 	cmd;
	struct nvme_completion	cqe;
	struct completion	done;
};

struct pci_epf_nvme_target {
	struct device *dev;
	struct nvmet_host host;
	struct nvmet_host_link host_link;
	struct nvmet_subsys subsys;
	struct nvmet_subsys_link subsys_link;
	struct nvmet_port port;
	enum nvme_ana_state port_ana_state[NVMET_MAX_ANAGRPS + 1];
	struct nvmet_ns ns;
	struct nvmet_ctrl *nvmet_ctrl;
	struct nvmet_sq sq[PCI_EPF_NVME_QIDMAX + 1];
	struct nvmet_cq cq[PCI_EPF_NVME_QIDMAX + 1];
	struct pci_epf_nvme_cmd keep_alive_epcmd;
	struct nvmet_req req;
	struct nvme_command cmd;
	struct nvme_completion cqe;
	struct completion done;
	struct sg_table sg_table;
	struct scatterlist sgl[PCI_EPF_NVME_PRP_LIST_ENTRIES + 2];
	struct pci_epf_nvme_map map[PCI_EPF_NVME_PRP_MAPS];
	struct pci_epf_nvme_prplist prplist;
	int keepalive;
};

struct pci_epf_nvme {
	void			*reg[PCI_STD_NUM_BARS];
	struct pci_epf		*epf;
	enum pci_barno		reg_bar;
	size_t			msix_table_offset;
	struct delayed_work	cmd_handler;
	const struct pci_epc_features *epc_features;
	struct pci_epf_nvme_host host;
	struct pci_epf_nvme_target target;
	unsigned int		tick;
	struct config_group	group;
	char			*hostnqn;
	char			*subsysnqn;
	char			*ns_device_path;
};

static void pci_epf_nvme_init_cmd(struct pci_epf_nvme *nvme,
				  int qid,
				  struct pci_epf_nvme_cmd *epcmd)
{
	memset(epcmd, 0, sizeof(*epcmd));
	epcmd->nvme = nvme;
	epcmd->qid = qid;
	epcmd->req.cmd = &epcmd->cmd;
	epcmd->req.cqe = &epcmd->cqe;
	epcmd->req.port = &nvme->target.port;
}

static struct pci_epf_nvme_cmd *pci_epf_nvme_alloc_cmd(struct pci_epf_nvme *nvme,
						       int qid)
{
	struct pci_epf_nvme_cmd *epcmd;

	epcmd = kmem_cache_alloc(epf_nvme_cmd_cache, GFP_KERNEL);
	if (epcmd)
		pci_epf_nvme_init_cmd(nvme, qid, epcmd);

	return epcmd;
}

static void pci_epf_nvme_free_cmd(struct pci_epf_nvme_cmd *epcmd)
{
	kmem_cache_free(epf_nvme_cmd_cache, epcmd);
}

static int pci_epf_nvme_map_alloc_region(struct pci_epf_nvme *nvme,
					 struct pci_epf_nvme_map *map)
{
	if (map->pci.phys_base)
		return -EALREADY;

	if (nvme->epc_features->align)
		map->align = nvme->epc_features->align;
	else
		map->align = PAGE_SIZE;

	if (map->size < map->align)
		map->pci.size = map->align << 1;
	else
		map->pci.size = map->size;

	map->pci.virt_base = pci_epc_mem_alloc_addr(nvme->epf->epc,
					&map->pci.phys_base, map->pci.size);
	if (!map->pci.virt_base)
		return -ENOMEM;

	return 0;
}

static void pci_epf_nvme_map_free_region(struct pci_epf_nvme *nvme,
					 struct pci_epf_nvme_map *map)
{
	if (!map->pci.phys_base)
		return;

	pci_epc_mem_free_addr(nvme->epf->epc, map->pci.phys_base,
			      map->pci.virt_base, map->pci.size);
	map->pci.phys_base = 0;
}

static int pci_epf_nvme_map_enable_region(struct pci_epf_nvme *nvme,
					  struct pci_epf_nvme_map *map)
{
	struct pci_epf *epf = nvme->epf;
	int ret;

	if (!map->pci.phys_base)
		return -ENOMEM;

	if (map->pci.phys_addr)
		return -EALREADY;

	map->host.phys_base = map->host.phys_addr;
	if (map->align > PAGE_SIZE)
		map->host.phys_base &= ~(map->align - 1);

	map->host.phys_end = map->host.phys_base + map->pci.size - 1;

	map->offset = map->host.phys_addr - map->host.phys_base;
	if (map->offset + map->size > map->pci.size)
		return -ERANGE;

	ret = pci_epc_map_addr(epf->epc, epf->func_no, epf->vfunc_no,
			       map->pci.phys_base, map->host.phys_base,
			       map->pci.size);
	if (ret)
		return ret;

	map->pci.virt_addr = map->pci.virt_base + map->offset;
	map->pci.phys_addr = map->pci.phys_base + map->offset;

	return 0;
}

static void pci_epf_nvme_map_disable_region(struct pci_epf_nvme *nvme,
					    struct pci_epf_nvme_map *map)
{
	struct pci_epf *epf = nvme->epf;

	if (!map->pci.phys_addr)
		return;

	pci_epc_unmap_addr(epf->epc, epf->func_no, epf->vfunc_no,
			   map->pci.phys_base);
	map->pci.phys_addr = 0;
}

static int pci_epf_nvme_map(struct pci_epf_nvme *nvme,
			    struct pci_epf_nvme_map *map)
{
	int ret;

	ret = pci_epf_nvme_map_alloc_region(nvme, map);
	if (ret) {
		dev_err(&nvme->epf->dev, "Failed to allocate region\n");
		return ret;
	}

	ret = pci_epf_nvme_map_enable_region(nvme, map);
	if (ret) {
		dev_err(&nvme->epf->dev, "Failed to enable region\n");
		pci_epf_nvme_map_free_region(nvme, map);
		return ret;
	}

	return 0;
}

static void pci_epf_nvme_unmap(struct pci_epf_nvme *nvme,
			       struct pci_epf_nvme_map *map)
{
	pci_epf_nvme_map_disable_region(nvme, map);
	pci_epf_nvme_map_free_region(nvme, map);

	memset(map, 0, sizeof(*map));
}

static int pci_epf_nvme_map_slot(struct pci_epf_nvme *nvme, int slot,
				 u64 addr, size_t size)
{
	struct pci_epf_nvme_map *map;

	if (!addr || !size || slot >= PCI_EPF_NVME_PRP_MAPS)
		return -EINVAL;

	map = &nvme->target.map[slot];
	map->size = size;
	map->host.phys_addr = addr;

	return pci_epf_nvme_map(nvme, map);
}

static inline void pci_epf_nvme_unmap_slot(struct pci_epf_nvme *nvme, int slot)
{
	struct pci_epf_nvme_map *map = &nvme->target.map[slot];

	if (map->pci.virt_addr)
		pci_epf_nvme_unmap(nvme, map);
}

static void __iomem *pci_epf_nvme_map_find(struct pci_epf_nvme *nvme,
					   u64 addr, size_t size)
{
	int slot;
	struct pci_epf_nvme_map *map;
	u64 end = addr + size - 1;

	for (slot = 0; slot < PCI_EPF_NVME_PRP_MAPS; slot++) {
		map = &nvme->target.map[slot];
		if (!map->pci.virt_addr)
			break;

		if (addr >= map->host.phys_base && end <= map->host.phys_end)
			return addr - map->host.phys_base + map->pci.virt_base;
	}

	return NULL;
}

static int pci_epf_nvme_map_sgl(struct pci_epf_nvme *nvme,
				struct nvme_sgl_desc *sgl)
{
	return pci_epf_nvme_map_slot(nvme, 0, le64_to_cpu(sgl->addr),
				     le32_to_cpu(sgl->length));
}

static int pci_epf_nvme_map_ksgl(struct pci_epf_nvme *nvme,
				 struct nvme_keyed_sgl_desc *ksgl)
{
	return pci_epf_nvme_map_slot(nvme, 0, le64_to_cpu(ksgl->addr),
				     NVME_CTRL_PAGE_SIZE);
}

static int pci_epf_nvme_map_prp(struct pci_epf_nvme *nvme, int slot, u64 prp)
{
	if (!prp)
		return 0;

	return pci_epf_nvme_map_slot(nvme, slot, prp, NVME_CTRL_PAGE_SIZE);
}

static inline int pci_epf_nvme_map_prp1(struct pci_epf_nvme *nvme,
					struct nvme_command *cmd)
{
	return pci_epf_nvme_map_slot(nvme, 0,
				     le64_to_cpu(cmd->common.dptr.prp1),
				     NVME_CTRL_PAGE_SIZE);
}

static inline void pci_epf_nvme_unmap_prp1(struct pci_epf_nvme *nvme)
{
	pci_epf_nvme_unmap_slot(nvme, 0);
}

static inline int pci_epf_nvme_map_prp2(struct pci_epf_nvme *nvme,
					struct nvme_command *cmd)
{
	return pci_epf_nvme_map_slot(nvme, 1,
				     le64_to_cpu(cmd->common.dptr.prp2),
				     NVME_CTRL_PAGE_SIZE);
}

static inline void pci_epf_nvme_unmap_dptr(struct pci_epf_nvme *nvme)
{
	int slot;

	for (slot = 0; slot < PCI_EPF_NVME_PRP_MAPS; slot++)
		pci_epf_nvme_unmap_slot(nvme, slot);
}

static int pci_epf_nvme_map_dptr(struct pci_epf_nvme *nvme,
				 struct nvme_command *cmd)
{
	u8 psdt = (cmd->common.flags & NVME_CMD_SGL_ALL);
	int ret;

	if (psdt == NVME_CMD_SGL_METABUF)
		return pci_epf_nvme_map_sgl(nvme, &cmd->common.dptr.sgl);

	if (psdt == NVME_CMD_SGL_METASEG)
		return pci_epf_nvme_map_ksgl(nvme, &cmd->common.dptr.ksgl);

	if (psdt != 0)
		return -EIO;

	ret = pci_epf_nvme_map_prp1(nvme, cmd);
	if (ret)
		return ret;

	if (cmd->common.dptr.prp2) {
		ret = pci_epf_nvme_map_prp2(nvme, cmd);
		if (ret) {
			pci_epf_nvme_unmap_prp1(nvme);
			return ret;
		}
	}

	return 0;
}

static void pci_epf_nvme_dump(struct pci_epf_nvme *nvme, const char *label,
			      void *data, size_t size)
{
	unsigned char *p = data;

	dev_dbg(&nvme->epf->dev, "%s:\n", label);

	while (size >= 8) {
		dev_dbg(&nvme->epf->dev,
			"%02x %02x %02x %02x %02x %02x %02x %02x\n",
			p[0], p[1], p[2], p[3],	p[4], p[5], p[6], p[7]);
		p += 8;
		size -= 8;
	}
}

static int pci_epf_nvme_expand_prplist(struct pci_epf_nvme *nvme,
				       unsigned int more)
{
	struct pci_epf_nvme_prplist *list = &nvme->target.prplist;
	u64 prp;

	list->count += more;

	while (more--) {
		prp = le64_to_cpu(list->prp[list->index]);
		if (!prp || (prp & (NVME_CTRL_PAGE_SIZE - 1)))
			return -EINVAL;

		if (prp > list->max)
			list->max = prp;
		if (prp < list->min)
			list->min = prp;

		list->prp[list->index++] = prp;
	}

	return 0;
}

static int pci_epf_nvme_transfer_prplist(struct pci_epf_nvme *nvme,
					 int slot, unsigned int count)
{
	struct pci_epf_nvme_prplist *list = &nvme->target.prplist;
	struct pci_epf_nvme_map *map = &nvme->target.map[slot];
	unsigned int more;
	size_t size;
	u64 nextlist;
	int ret;

	list->align = map->align;
	list->min = PCI_EPF_NVME_PRP_NONE;
	list->max = 0;
	list->index = 0;

	while (count) {
		if (count <= PCI_EPF_NVME_PRP_PER_PAGE) {
			more = count;
			size = more * PCI_EPF_NVME_PRP_SIZE;
		} else {
			more = PCI_EPF_NVME_PRP_PER_PAGE - 1;
			size = NVME_CTRL_PAGE_SIZE;
		}

		pci_epf_nvme_dump(nvme, "prplist", map->pci.virt_addr, size);
		memcpy_fromio(&list->prp[list->index],
			      map->pci.virt_addr, size);
		pci_epf_nvme_unmap(nvme, map);

		if (pci_epf_nvme_expand_prplist(nvme, more)) {
			dev_err(&nvme->epf->dev, "prplist invalid prp\n");
			return -EINVAL;
		}

		if (count <= PCI_EPF_NVME_PRP_PER_PAGE)
			break;

		count -= PCI_EPF_NVME_PRP_PER_PAGE - 1;

		nextlist = le64_to_cpu(list->prp[list->index]);
		if (!nextlist || (nextlist & (NVME_CTRL_PAGE_SIZE - 1))) {
			dev_err(&nvme->epf->dev, "Invalid next prplist\n");
			return -EINVAL;
		}

		ret = pci_epf_nvme_map_prp(nvme, slot, nextlist);
		if (ret) {
			dev_err(&nvme->epf->dev, "Next prplist map error\n");
			return ret;
		}
	}

	return 0;
}

static void pci_epf_nvme_review_prplist(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_prplist *list = &nvme->target.prplist;
	unsigned int index;
	u64 prp;

	list->min = PCI_EPF_NVME_PRP_NONE;
	for (index = 0; index < list->count; index++) {
		prp = list->prp[index];
		if (prp & PCI_EPF_NVME_PRP_MAP_FLAG)
			continue;

		if (pci_epf_nvme_map_find(nvme, prp, NVME_CTRL_PAGE_SIZE))
			list->prp[index] |= PCI_EPF_NVME_PRP_MAP_FLAG;
		else if (prp < list->min)
			list->min = prp;
	}
}

static int pci_epf_nvme_map_prplist(struct pci_epf_nvme *nvme, int slot)
{
	struct pci_epf_nvme_prplist *list = &nvme->target.prplist;
	size_t span;
	u64 base, mask;
	int ret;

	if (list->min == PCI_EPF_NVME_PRP_NONE) {
		dev_err(&nvme->epf->dev,
			"Unexpected empty prplist\n");
		return -ENOMEM;
	}

	if (pci_epf_nvme_map_find(nvme, list->min, NVME_CTRL_PAGE_SIZE)) {
		pci_epf_nvme_review_prplist(nvme);
		if (list->min == PCI_EPF_NVME_PRP_NONE)
			return 0;
	}

	mask = ~(list->align - 1);
	base = list->min & mask;
	span = list->max + NVME_CTRL_PAGE_SIZE - base;

	while (span) {
		ret = pci_epf_nvme_map_slot(nvme, slot, base,
					    PCI_EPF_NVME_PRP_MAP_SIZE);
		if (ret) {
			dev_err(&nvme->epf->dev,
				"prplist map region failed\n");
			return ret;
		}

		if (span <= PCI_EPF_NVME_PRP_MAP_SIZE)
			return 0;

		span -= PCI_EPF_NVME_PRP_MAP_SIZE;

		if (++slot == PCI_EPF_NVME_PRP_MAPS) {
			dev_err(&nvme->epf->dev,
				"prplist map out of resources\n");
			return -ENOMEM;
		}

		pci_epf_nvme_review_prplist(nvme);
		if (list->min == PCI_EPF_NVME_PRP_NONE)
			return 0;

		base = list->min & mask;
		span = list->max + NVME_CTRL_PAGE_SIZE - base;
	}

	return 0;
}

static int pci_epf_nvmet_write32(struct pci_epf_nvme_target *target,
				 u32 off, u32 val)
{
	struct nvmet_ctrl *nvmet_ctrl = target->nvmet_ctrl;

	if (!nvmet_ctrl)
		return -ENXIO;

	switch (off) {
	case NVME_REG_CC:
		nvmet_update_cc(nvmet_ctrl, val);
		return 0;
	default:
		return -EIO;
	}
}

static int pci_epf_nvmet_read32(struct pci_epf_nvme_target *target,
				u32 off, u32 *val)
{
	struct nvmet_ctrl *nvmet_ctrl = target->nvmet_ctrl;
	u32 reg;

	if (!nvmet_ctrl)
		return -ENXIO;

	switch (off) {
	case NVME_REG_VS:
		reg = nvmet_ctrl->subsys->ver;
		break;
	case NVME_REG_CC:
		reg = nvmet_ctrl->cc;
		break;
	case NVME_REG_CSTS:
		reg = nvmet_ctrl->csts;
		break;
	default:
		return -EIO;
	}

	if (val)
		*val = reg;
	return 0;
}

static int pci_epf_nvmet_read64(struct pci_epf_nvme_target *target,
				u32 off, u64 *val)
{
	struct nvmet_ctrl *nvmet_ctrl = target->nvmet_ctrl;
	u64 reg;

	if (!nvmet_ctrl)
		return -ENXIO;

	switch (off) {
	case NVME_REG_CAP:
		reg = nvmet_ctrl->cap;
		break;
	default:
		return -EIO;
	}

	if (val)
		*val = reg;
	return 0;
}

static int pci_epf_nvmet_add_port(struct nvmet_port *nvmet_port)
{
	pr_err("Unexpected call to add port\n");

	return -ENOTSUPP;
}

static void pci_epf_nvmet_remove_port(struct nvmet_port *nvmet_port)
{
	pr_err("Unexpected call to remove port\n");
}

static void pci_epf_nvmet_queue_response(struct nvmet_req *req)
{
	struct pci_epf_nvme_cmd *epcmd =
		container_of(req, struct pci_epf_nvme_cmd, req);
	struct pci_epf_nvme *nvme = epcmd->nvme;

	if (req->cqe->status)
		dev_err(&nvme->epf->dev, "queue_response status 0x%x\n",
			le16_to_cpu(req->cqe->status));

	complete(&epcmd->done);
	nvme->target.keepalive = 0;
}

static void pci_epf_nvmet_delete_ctrl(struct nvmet_ctrl *nvmet_ctrl)
{
	pr_err("Unexpected call to delete controller\n");
}

static struct nvmet_fabrics_ops pci_epf_nvmet_fabrics_ops = {
	.owner		= THIS_MODULE,
	.type		= NVMF_TRTYPE_LOOP,
	.add_port	= pci_epf_nvmet_add_port,
	.remove_port	= pci_epf_nvmet_remove_port,
	.queue_response = pci_epf_nvmet_queue_response,
	.delete_ctrl	= pci_epf_nvmet_delete_ctrl,
};

static int pci_epf_nvmet_execute(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmet_req *req = &target->req;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	req->execute(req);

	if (req->cmd->common.opcode != nvme_admin_async_event)
		wait_for_completion(&target->done);

	return req->cqe->status ? -EIO : NVME_SC_SUCCESS;
}

static int pci_epf_nvmet_execute_sg_table(struct pci_epf_nvme *nvme,
					  struct scatterlist *sg)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmet_req *req = &target->req;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	req->sg = target->sgl;
	req->sg_cnt = sg - target->sgl;
	sg_init_marker(req->sg, req->sg_cnt);

	target->sg_table.sgl = req->sg;
	target->sg_table.nents = req->sg_cnt;
	target->sg_table.orig_nents = req->sg_cnt;

	return pci_epf_nvmet_execute(nvme);
}

static void pci_epf_nvmet_set_sg(struct scatterlist *sg,
				 const void *buffer, size_t size)
{
	sg_set_page(sg, virt_to_page(buffer), size, offset_in_page(buffer));
}

static size_t pci_epf_nvmet_set_sg_page(struct scatterlist *sg,
					const void *page)
{
	pci_epf_nvmet_set_sg(sg, page, PAGE_SIZE);

	return PAGE_SIZE;
}

static size_t pci_epf_nvmet_set_sg_map(struct scatterlist *sg,
				       const struct pci_epf_nvme_map *map)
{
	pci_epf_nvmet_set_sg(sg, map->pci.virt_addr, map->size);

	return map->size;
}

static int pci_epf_nvmet_execute_sgl_ksgl(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmet_req *req = &target->req;
	struct scatterlist *sg = target->sgl;
	struct pci_epf_nvme_map *map = target->map;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	pci_epf_nvme_dump(nvme, "sgl", map->pci.virt_addr, 64);
	req->transfer_len = pci_epf_nvmet_set_sg_map(sg++, map);

	return pci_epf_nvmet_execute_sg_table(nvme, sg);
}

static int pci_epf_nvmet_execute_prp1_prp2(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmet_req *req = &target->req;
	struct scatterlist *sg = target->sgl;
	struct pci_epf_nvme_map *map = target->map;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	req->transfer_len = pci_epf_nvmet_set_sg_map(sg++, map++);
	req->transfer_len += pci_epf_nvmet_set_sg_map(sg++, map);

	return pci_epf_nvmet_execute_sg_table(nvme, sg);
}

static int pci_epf_nvmet_execute_prp1(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmet_req *req = &target->req;
	struct scatterlist *sg = target->sgl;
	struct pci_epf_nvme_map *map = target->map;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	req->transfer_len = pci_epf_nvmet_set_sg_map(sg++, map);

	return pci_epf_nvmet_execute_sg_table(nvme, sg);
}

static int pci_epf_nvmet_execute_prplist(struct pci_epf_nvme *nvme,
					 int slot, size_t list_data_len)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct pci_epf_nvme_prplist *list = &target->prplist;
	struct scatterlist *sg;
	struct nvmet_req *req;
	void __iomem *virt_addr;
	unsigned int index, count;
	int status;
	u64 prp;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	memset(list, 0, sizeof(*list));

	if (slot < 0 || slot > 1)
		return -EINVAL;

	count = list_data_len >> PAGE_SHIFT;
	if (count == 0 || count > PCI_EPF_NVME_PRP_LIST_ENTRIES) {
		dev_err(&nvme->epf->dev,
			"prplist invalid length 0x%x\n", count);
		return -EINVAL;
	}

	status = pci_epf_nvme_transfer_prplist(nvme, slot, count);
	if (status)
		return status;

	status = pci_epf_nvme_map_prplist(nvme, slot);
	if (status)
		return status;

	sg = &target->sgl[slot];
	req = &target->req;

	for (index = 0; index < list->count; index++) {
		prp = list->prp[index] & ~(PCI_EPF_NVME_PRP_MAP_FLAG);
		virt_addr = pci_epf_nvme_map_find(nvme, prp,
						  NVME_CTRL_PAGE_SIZE);
		if (unlikely(!virt_addr)) {
			dev_err(&nvme->epf->dev, "prplist map find error\n");
			return -ENOMEM;
		}

		req->transfer_len += pci_epf_nvmet_set_sg_page(sg++, virt_addr);
	}

	return pci_epf_nvmet_execute_sg_table(nvme, sg);
}

static int
pci_epf_nvmet_execute_prp1_prp2list(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmet_req *req = &target->req;
	size_t list_data_len = req->transfer_len - target->map[0].size;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	req->transfer_len = pci_epf_nvmet_set_sg_map(target->sgl, target->map);

	return pci_epf_nvmet_execute_prplist(nvme, 1, list_data_len);
}

static int pci_epf_nvmet_execute_prp1list(struct pci_epf_nvme *nvme)
{
	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	return pci_epf_nvmet_execute_prplist(nvme, 0,
					     nvme->target.req.transfer_len);
}

static int pci_epf_nvmet_execute_request(struct pci_epf_nvme *nvme, int qid)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct pci_epf_nvme_map *map = target->map;
	struct nvmet_req *req = &target->req;
	struct nvme_command *cmd = req->cmd;
	u8 psdt = (cmd->common.flags & NVME_CMD_SGL_ALL);
	size_t transfer_len = req->transfer_len;
	int status;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	cmd->common.flags &= ~(NVME_CMD_SGL_ALL);
	cmd->common.flags |= NVME_CMD_SGL_METABUF;

	if (!nvmet_req_init(req, &target->cq[qid], &target->sq[qid],
			    &pci_epf_nvmet_fabrics_ops)) {
		dev_err(&nvme->epf->dev, "nvmet_req_init failed\n");
		status = le16_to_cpu(req->cqe->status) >> 1;
		goto out;
	}

	req->transfer_len = transfer_len;

	memset(target->sgl, 0, sizeof(target->sgl));

	if (!req->transfer_len) {
		status = pci_epf_nvmet_execute(nvme);
		goto out;
	}

	if (!map[0].pci.virt_addr || !map[0].size) {
		status = -ENOMEM;
		goto out;
	}

	if (psdt) {
		status = pci_epf_nvmet_execute_sgl_ksgl(nvme);
		goto out;
	}

	if (req->transfer_len <= NVME_CTRL_PAGE_SIZE) {
		status = pci_epf_nvmet_execute_prp1(nvme);
		goto out;
	}

	if (!map[1].pci.virt_addr || !map[1].size) {
		status = pci_epf_nvmet_execute_prp1list(nvme);
		goto out;
	}

	if (req->transfer_len <= (2 * NVME_CTRL_PAGE_SIZE)) {
		status = pci_epf_nvmet_execute_prp1_prp2(nvme);
		goto out;
	}

	status = pci_epf_nvmet_execute_prp1_prp2list(nvme);

out:
	switch (status) {
	case NVME_SC_SUCCESS:
		return NVME_SC_SUCCESS;
	default:
		dev_dbg(&nvme->epf->dev, "req exec status %d\n", status);
		req->cqe->status = cpu_to_le16(status << 1);
		return status;
	}
}

/*
 * Execute a fabrics command using a local buffer.
 */
static int pci_epf_nvme_exec_fabrics_cmd(struct pci_epf_nvme *nvme,
					 struct pci_epf_nvme_cmd *epcmd,
					 void *buffer,
					 size_t buffer_size)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmet_req *req = &epcmd->req;
	struct nvme_command *cmd = req->cmd;
	struct scatterlist *sg = &epcmd->sgl;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	/* NVMe target mandates SGL for fabrics commands */
	cmd->common.flags &= ~(NVME_CMD_SGL_ALL);
	cmd->common.flags |= NVME_CMD_SGL_METABUF;

	if (!nvmet_req_init(req, &target->cq[epcmd->qid], &target->sq[epcmd->qid],
			    &pci_epf_nvmet_fabrics_ops)) {
		dev_err(&nvme->epf->dev, "nvmet_req_init failed\n");
		return -ENOMEM;
	}

	/* Map buffer to the request sgl and execute */
	sg_set_page(sg, virt_to_page(buffer), buffer_size,
		    offset_in_page(buffer));
	sg_init_marker(sg, 1);

	req->transfer_len = buffer_size;
	req->sg = sg;
	req->sg_cnt = 1;

	req->execute(req);

	wait_for_completion(&epcmd->done);

	if (epcmd->req.cqe->status)
		return -EIO;

	return 0;
}

static int pci_epf_nvmet_connect(struct pci_epf_nvme *nvme,
				 u16 qid, u16 qsize, u16 command_id)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmf_connect_data *data;
	struct pci_epf_nvme_cmd *epcmd;
	struct nvme_command *cmd;
	char *subsysnqn;
	char *hostnqn;
	u16 cntlid;
	int ret;

	/* Allocate a command and a buffer */
	epcmd = pci_epf_nvme_alloc_cmd(nvme, qid);
	if (!epcmd)
		return -ENOMEM;

	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		pci_epf_nvme_free_cmd(epcmd);
		return -ENOMEM;
	}

	subsysnqn = target->subsys.subsysnqn;
	hostnqn = nvmet_host_name(target->host_link.host);
	cntlid = qid ? target->nvmet_ctrl->cntlid : 0xffff;

	dev_dbg(&nvme->epf->dev,
		"Execute connect cmd: cmdid 0x%02x, qid %d, host %s, subsys %s\n",
		command_id, (int)qid, hostnqn, subsysnqn);

	strncpy(data->hostnqn, hostnqn, NVMF_NQN_SIZE);
	strncpy(data->subsysnqn, subsysnqn, NVMF_NQN_SIZE);
	uuid_gen(&data->hostid);
	data->cntlid = cpu_to_le16(cntlid);

	cmd = &epcmd->cmd;
	cmd->connect.command_id = command_id;
	cmd->connect.opcode = nvme_fabrics_command;
	cmd->connect.fctype = nvme_fabrics_type_connect;
	cmd->connect.qid = cpu_to_le16(qid);
	cmd->connect.sqsize = cpu_to_le16(qsize);
	cmd->connect.kato = cpu_to_le32(NVME_DEFAULT_KATO * 1000);

	ret = pci_epf_nvme_exec_fabrics_cmd(nvme, epcmd, data, sizeof(*data));
	if (ret)
		dev_err(&nvme->epf->dev, "Connect qid %d failed\n", qid);

	pci_epf_nvme_free_cmd(epcmd);
	kfree(data);

	return ret;
}

static void pci_epf_nvme_target_init(struct pci_epf_nvme *nvme)
{
	static u8 nguid[16] = {
		0xef, 0x90, 0x68, 0x9c, 0x6c, 0x46, 0xd4, 0x4c,
		0x89, 0xc1, 0x40, 0x67, 0x80, 0x13, 0x09, 0xa8
	};
	struct pci_epf *epf = nvme->epf;
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmet_host *host = &target->host;
	struct nvmet_host_link *host_link = &target->host_link;
	struct nvmet_subsys *subsys = &target->subsys;
	struct nvmet_subsys_link *subsys_link = &target->subsys_link;
	struct nvmet_port *port = &target->port;
	struct nvmet_ns *ns = &target->ns;
	int qid, gid;

	dev_dbg(&epf->dev, "%s\n", __FUNCTION__);

	target->dev = &epf->dev;
	target->keepalive = PCI_EPF_NVME_KA_TICKS - 1;
	init_completion(&target->done);

	host->group.cg_item.ci_name = nvme->hostnqn;

	subsys->subsysnqn = nvme->subsysnqn;
	subsys->type = NVME_NQN_NVME;
	subsys->max_qid = PCI_EPF_NVME_QIDMAX;
	kref_init(&subsys->ref);
	mutex_init(&subsys->lock);
	xa_init(&subsys->namespaces);
	INIT_LIST_HEAD(&subsys->ctrls);
	INIT_LIST_HEAD(&subsys->hosts);

	port->ana_state = target->port_ana_state;
	for (gid = 0; gid <= NVMET_MAX_ANAGRPS; gid++)
		port->ana_state[gid] = NVME_ANA_INACCESSIBLE;
	port->ana_state[NVMET_DEFAULT_ANA_GRPID] = NVME_ANA_OPTIMIZED;
	port->ana_default_group.port = port;
	port->ana_default_group.grpid = NVMET_DEFAULT_ANA_GRPID;

	port->disc_addr.trtype = NVMF_TRTYPE_LOOP;
	port->disc_addr.treq = NVMF_TREQ_DISABLE_SQFLOW;
	memset(&port->disc_addr.tsas, 0, NVMF_TSAS_SIZE);
	INIT_LIST_HEAD(&port->global_entry);
	INIT_LIST_HEAD(&port->entry);
	INIT_LIST_HEAD(&port->referrals);
	INIT_LIST_HEAD(&port->subsystems);
	port->inline_data_size = 0;

	INIT_LIST_HEAD(&host_link->entry);
	host_link->host = host;
	list_add_tail(&host_link->entry, &subsys->hosts);

	INIT_LIST_HEAD(&subsys_link->entry);
	subsys_link->subsys = subsys;
	list_add_tail(&subsys_link->entry, &port->subsystems);

	init_completion(&ns->disable_done);
	ns->nsid = 1;
	ns->subsys = subsys;
	ns->device_path = nvme->ns_device_path;
	ns->anagrpid = NVMET_DEFAULT_ANA_GRPID;
	ns->buffered_io = false;
	memcpy(&ns->nguid, nguid, sizeof(ns->nguid));
	uuid_gen(&ns->uuid);

	for (qid = 0; qid <= PCI_EPF_NVME_QIDMAX; qid++) {
		target->cq[qid].qid = 0;
		target->cq[qid].size = 0;

		target->sq[qid].sqhd = 0;
		target->sq[qid].qid = 0;
		target->sq[qid].size = 0;
		target->sq[qid].ctrl = NULL;
	}
}

static int pci_epf_nvme_target_start(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct pci_epf *epf = nvme->epf;
	struct pci_epf_nvme_host *host = &nvme->host;
	int ret, qid;

	dev_dbg(&epf->dev, "%s\n", __FUNCTION__);

	for (qid = 0; qid <= PCI_EPF_NVME_QIDMAX; qid++)
		nvmet_sq_init(&target->sq[qid]);

	ret = pci_epf_nvmet_connect(nvme, 0, NVME_AQ_DEPTH - 1, 0);
	if (ret)
		return ret;

	target->nvmet_ctrl = target->sq[0].ctrl;
	if (!target->nvmet_ctrl) {
		dev_err(&epf->dev, "No target controller\n");
		return -EIO;
	}

	dev_dbg(&epf->dev,
		"Connected to target controller %p ID %d\n",
		target->nvmet_ctrl, target->nvmet_ctrl->cntlid);

	ret = pci_epf_nvmet_write32(target, NVME_REG_CC, host->cc);
	if (ret) {
		dev_err(&epf->dev, "Failed to write target CC\n");
		return ret;
	}

	return 0;
}

static void pci_epf_nvme_target_stop(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	int qid;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	nvmet_ns_disable(&target->ns);

	for (qid = 0; qid <= PCI_EPF_NVME_QIDMAX; qid++)
		nvmet_sq_destroy(&target->sq[qid]);

	target->nvmet_ctrl = NULL;
}

static u64 pci_epf_nvme_cap(struct pci_epf_nvme *nvme)
{
	u64 cap;

	if (pci_epf_nvmet_read64(&nvme->target, NVME_REG_CAP, &cap)) {
		/* maximum queue entries supported (MQES) */
		cap = PCI_EPF_NVME_MQES;
		/* CC.EN timeout in 500msec units (TO) */
		cap |= (PCI_EPF_NVME_TO << 24);
		/* command sets supported (CSS) */
		cap |= (PCI_EPF_NVME_CSS << 37);
	}

	if (NVME_CAP_MPSMIN(cap) != PCI_EPF_NVME_MPSMIN) {
		/* minimum page size (MPSMIN) */
		cap &= ~(0x0fULL << 48);
		cap |= (PCI_EPF_NVME_MPSMIN << 48);
	}

	if (NVME_CAP_MPSMAX(cap) != PCI_EPF_NVME_MPSMAX) {
		/* maximum page size (MPSMAX) */
		cap &= ~(0x0fULL << 52);
		cap |= (PCI_EPF_NVME_MPSMAX << 52);
	}

	return cap;
}

static u32 pci_epf_nvme_vs(struct pci_epf_nvme *nvme)
{
	u32 vs;

	if (pci_epf_nvmet_read32(&nvme->target, NVME_REG_VS, &vs))
		return NVME_VS(1, 3, 0);

	/* CMB supported on NVMe versions 1.2+ */
	if (vs < NVME_VS(1, 2, 0))
		vs = NVME_VS(1, 2, 0);

	return vs;
}

static u32 pci_epf_nvme_csts(struct pci_epf_nvme *nvme)
{
	u32 csts;

	if (pci_epf_nvmet_read32(&nvme->target, NVME_REG_CSTS, &csts))
		csts = 0;

	return csts;
}

static u32 pci_epf_nvme_cmbloc(struct pci_epf_nvme *nvme)
{
	if (nvme->host.cmb.size)
		return nvme->host.cmb.bar;

	return 0;
}

static u32 pci_epf_nvme_cmbsz(struct pci_epf_nvme *nvme)
{
	u32 cmbsz = 0;

	if (nvme->host.cmb.size) {
		cmbsz =	NVME_CMBSZ_SQS |   /* Submission Queue Support (SQS) */
			NVME_CMBSZ_CQS;    /* Completion Queue Support (CQS) */

		/* Size (SZ) in Size Units (SZU) of 4KiB */
		cmbsz |= (nvme->host.cmb.size << NVME_CMBSZ_SZ_SHIFT);
	}

	return cmbsz;
}

static size_t bar_size[PCI_STD_NUM_BARS] =
	{ SZ_4K, SZ_4K, SZ_4K, SZ_4K, SZ_4K, SZ_4K };

static u32 pci_epf_nvme_host_read32(struct pci_epf_nvme_host *host, u32 reg)
{
	return readl(host->reg + reg);
}

static void pci_epf_nvme_host_write32(struct pci_epf_nvme_host *host,
				      u32 reg, u32 val)
{
	writel(val, host->reg + reg);
}

static u64 pci_epf_nvme_host_read64(struct pci_epf_nvme_host *host, u32 reg)
{
	return lo_hi_readq(host->reg + reg);
}

static void pci_epf_nvme_host_write64(struct pci_epf_nvme_host *host,
				      u32 reg, u64 val)
{
	lo_hi_writeq(val, host->reg + reg);
}

static void pci_epf_nvme_host_emit(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_host *host = &nvme->host;

	host->cap = pci_epf_nvme_cap(nvme);
	host->vs = pci_epf_nvme_vs(nvme);
	host->cmbloc = pci_epf_nvme_cmbloc(nvme);
	host->cmbsz = pci_epf_nvme_cmbsz(nvme);
	host->csts = pci_epf_nvme_csts(nvme);

	pci_epf_nvme_host_write64(host, NVME_REG_CAP, host->cap);
	pci_epf_nvme_host_write32(host, NVME_REG_VS, host->vs);
	pci_epf_nvme_host_write32(host, NVME_REG_CMBLOC, host->cmbloc);
	pci_epf_nvme_host_write32(host, NVME_REG_CMBSZ, host->cmbsz);
	pci_epf_nvme_host_write32(host, NVME_REG_CSTS, host->csts);
}

static int pci_epf_nvme_host_queue_pair(struct pci_epf_nvme *nvme,
					u16 sqid, u16 cqid)
{
	struct pci_epf_nvme_queue *sq = &nvme->host.sq[sqid];
	struct pci_epf_nvme_queue *cq = &nvme->host.cq[cqid];
	struct pci_epf *epf = nvme->epf;
	int ret;

	sq->qid = sqid;
	sq->depth = sq->size + 1;
	sq->vector = 0;
	sq->map.size = sq->depth * sizeof(struct nvme_command);
	sq->db = NVME_REG_DBS + (sqid * 2 * sizeof(u32));

	cq->qid = cqid;
	cq->depth = cq->size + 1;
	cq->vector = 0;
	cq->map.size = cq->depth * sizeof(struct nvme_completion);
	cq->db = NVME_REG_DBS + (((cqid * 2) + 1) * sizeof(u32));
	cq->phase = 1;

	if (!sq->map.pci.virt_addr) {
		ret = pci_epf_nvme_map(nvme, &sq->map);
		if (ret) {
			dev_err(&epf->dev, "Failed to map host SQ%d\n", sqid);
			return ret;
		}
	}

	if (!cq->map.pci.virt_addr) {
		ret = pci_epf_nvme_map(nvme, &cq->map);
		if (ret) {
			dev_err(&epf->dev, "Failed to map host CQ%d\n", cqid);
			pci_epf_nvme_unmap(nvme, &sq->map);
			memset(sq, 0, sizeof(*sq));
			return ret;
		}
	}

	return 0;
}

static void pci_epf_nvme_complete_host_command(struct pci_epf_nvme *nvme,
					struct pci_epf_nvme_cmd *epcmd,
					int status)
{
	struct pci_epf_nvme_host *host = &nvme->host;
	struct pci_epf_nvme_queue *cq = &host->cq[epcmd->qid];
	struct pci_epf *epf = nvme->epf;
	struct pci_epc *epc = epf->epc;
	struct nvme_completion *host_cqe;
	struct nvmet_req *req = &epcmd->req;
	struct nvme_command *cmd = req->cmd;
	struct nvme_completion *cqe = req->cqe;
	int tail;

	cqe->sq_id = epcmd->qid;
	cqe->sq_head = cpu_to_le16(epcmd->head);
	cqe->command_id = cmd->common.command_id;
	if (status != NVME_SC_SUCCESS) {
		cqe->status = cpu_to_le16(status << 1);
		cqe->status |= cpu_to_le16(cq->phase);
	}

	tail = cq->tail++;
	if (cq->tail == cq->depth) {
		cq->tail = 0;
		cq->phase = !cq->phase;
	}

	host_cqe = (struct nvme_completion *)cq->map.pci.virt_addr + tail;
	memcpy_toio(host_cqe, cqe, sizeof(struct nvme_completion));
	pci_epf_nvme_dump(nvme, "cqe", host_cqe,
			  sizeof(struct nvme_completion));

	pci_epf_nvme_free_cmd(epcmd);

	if (!(cq->flags & NVME_CQ_IRQ_ENABLED))
		return;

	if (host->msi) {
		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
				  PCI_EPC_IRQ_MSI, 1);
		return;
	}

	pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
			  PCI_EPC_IRQ_LEGACY, 0);
}

static struct pci_epf_nvme_cmd *
pci_epf_nvme_fetch_host_command(struct pci_epf_nvme *nvme, int qid)
{
	struct pci_epf_nvme_host *host = &nvme->host;
	struct pci_epf_nvme_queue *sq = &host->sq[qid];
	struct pci_epf_nvme_cmd *epcmd;
	struct nvme_command *host_cmd;
	int db;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	if (WARN_ON(!sq->size))
		return NULL;

	epcmd = pci_epf_nvme_alloc_cmd(nvme, qid);
	if (!epcmd)
		return NULL;

	db = pci_epf_nvme_host_read32(host, sq->db);
	if (db == sq->head) {
		pci_epf_nvme_free_cmd(epcmd);
		return NULL;
	}

	dev_dbg(&nvme->epf->dev, "sq[%d]: head %d/%d, dbs %d\n",
		qid, (int)sq->head, (int)sq->depth, db);

	epcmd->head = sq->head;

	sq->head++;
	if (sq->head == sq->depth)
		sq->head = 0;

	/* Get the NVMe command submitted by the host */
	host_cmd = (struct nvme_command *)sq->map.pci.virt_addr + epcmd->head;
	memcpy_fromio(&epcmd->cmd, host_cmd, sizeof(struct nvme_command));

	pci_epf_nvme_dump(nvme, "cmd", host_cmd, sizeof(struct nvme_command));

	return epcmd;
}

static int pci_epf_nvme_host_start(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_host *host = &nvme->host;
	struct pci_epf *epf = nvme->epf;
	struct pci_epc *epc = epf->epc;
	int ret;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	host->aqa = pci_epf_nvme_host_read32(host, NVME_REG_AQA);
	host->asq = pci_epf_nvme_host_read64(host, NVME_REG_ASQ);
	host->acq = pci_epf_nvme_host_read64(host, NVME_REG_ACQ);

	host->sq[0].size = (host->aqa & 0x0fff);
	host->sq[0].flags = NVME_QUEUE_PHYS_CONTIG;
	host->sq[0].map.host.phys_addr = host->asq;

	host->cq[0].size = ((host->aqa & 0x0fff0000) >> 16);
	host->cq[0].flags = NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED;
	host->cq[0].map.host.phys_addr = host->acq;

	ret = pci_epf_nvme_host_queue_pair(nvme, 0, 0);
	if (ret)
		return ret;

	host->msi = pci_epc_get_msi(epc, epf->func_no, epf->vfunc_no);

	pci_epf_nvme_host_emit(nvme);

	return 0;
}

static void pci_epf_nvme_host_stop(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_host *host = &nvme->host;
	int qid;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	for (qid = 0; qid <= PCI_EPF_NVME_QIDMAX; qid++) {
		pci_epf_nvme_unmap(nvme, &host->sq[qid].map);
		memset(&host->sq[qid], 0, sizeof(struct pci_epf_nvme_queue));
		pci_epf_nvme_unmap(nvme, &host->cq[qid].map);
		memset(&host->cq[qid], 0, sizeof(struct pci_epf_nvme_queue));
	}

	host->msi = 0;

	host->csts &= ~NVME_CSTS_RDY;
	pci_epf_nvme_host_write32(host, NVME_REG_CSTS, host->csts);
}

static int pci_epf_nvme_host_cmb_init(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_cmb *cmb = &nvme->host.cmb;
	struct pci_epf *epf = nvme->epf;
	enum pci_barno bar = BAR_2;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	if (!nvme->reg[bar]) {
		dev_err(&epf->dev, "Failed to initialize CMB\n");
		return -ENOMEM;
	}

	cmb->bar = bar;
	cmb->size = epf->bar[bar].size;
	cmb->virt_dma_addr = nvme->reg[bar];
	cmb->phys_dma_addr = epf->bar[bar].phys_addr;

	return 0;
}

static void pci_epf_nvme_host_init(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_host *host = &nvme->host;
	int qid;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	for (qid = 0; qid <= PCI_EPF_NVME_QIDMAX; qid++) {
		memset(&host->sq[qid], 0, sizeof(struct pci_epf_nvme_queue));
		memset(&host->cq[qid], 0, sizeof(struct pci_epf_nvme_queue));
	}

	host->reg = nvme->reg[nvme->reg_bar];

	host->msi = 0;
	host->intms = 0;
	host->intmc = 0;
	host->cc = 0;
	host->aqa = 0;

	pci_epf_nvme_host_cmb_init(nvme);
	pci_epf_nvme_host_emit(nvme);
}

static void pci_epf_nvme_exec_admin_cmd(struct pci_epf_nvme *nvme,
					struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmet_req *req = &epcmd->req;
	struct nvme_command *cmd = req->cmd;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	/* NVMe target mandates SGL for admin commands */
	cmd->common.flags &= ~(NVME_CMD_SGL_ALL);
	cmd->common.flags |= NVME_CMD_SGL_METABUF;

	if (!nvmet_req_init(req, &target->cq[0], &target->sq[0],
			    &pci_epf_nvmet_fabrics_ops)) {
		dev_err(&nvme->epf->dev, "nvmet_req_init failed\n");
		return;
	}

	/* If we have data, map host prp1 to an sgl */
	if (epcmd->transfer_len) {
		struct pci_epf_nvme_map *map = target->map;
		struct scatterlist *sg = &epcmd->sgl;

		memset(sg, 0, sizeof(*sg));
		sg_set_page(sg, virt_to_page(map->pci.virt_addr),
			    epcmd->transfer_len,
			    offset_in_page(map->pci.virt_addr));
		sg_init_marker(sg, 1);

		req->transfer_len = epcmd->transfer_len;
		req->sg = sg;
		req->sg_cnt = 1;
	}

	/* Execute the request */
	req->execute(req);

	if (cmd->common.opcode == nvme_admin_async_event) {
		struct nvmet_ns *ns = &target->ns;

		if (ns->enabled || nvmet_ns_enable(ns))
			/* Completion will be signaled once an event occurs */
			return;
	}

	wait_for_completion(&epcmd->done);
}

static void pci_epf_nvme_target_keep_alive(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct pci_epf_nvme_cmd *epcmd = &target->keep_alive_epcmd;
	struct nvme_command *cmd = &epcmd->cmd;

	nvme->target.keepalive++;
	if (nvme->target.keepalive < PCI_EPF_NVME_KA_TICKS)
		return;

	pci_epf_nvme_init_cmd(nvme, 0, epcmd);
	cmd->common.opcode = nvme_admin_keep_alive;

	pci_epf_nvme_exec_admin_cmd(nvme, epcmd);

	if (epcmd->cqe.status)
		dev_err(&nvme->epf->dev, "Execute keep alive failed\n");
}

static int pci_epf_nvme_admin_create_cq(struct pci_epf_nvme *nvme,
					struct nvme_command *cmd)
{
	struct pci_epf_nvme_queue *cq;
	u16 cqid, cq_flags, qsize;

	cqid = le16_to_cpu(cmd->create_cq.cqid);
	if (!cqid || cqid > PCI_EPF_NVME_QIDMAX)
		return NVME_SC_QID_INVALID | NVME_SC_DNR;

	cq_flags = le16_to_cpu(cmd->create_cq.cq_flags);
	if (!(cq_flags & NVME_QUEUE_PHYS_CONTIG))
		return NVME_SC_INVALID_QUEUE | NVME_SC_DNR;

	qsize = le16_to_cpu(cmd->create_cq.qsize);
	if (!qsize)
		return NVME_SC_QUEUE_SIZE | NVME_SC_DNR;

	if (cmd->create_cq.irq_vector)
		return NVME_SC_INVALID_VECTOR | NVME_SC_DNR;

	cq = &nvme->host.cq[cqid];
	if (cq->qid)
		return NVME_SC_QID_INVALID | NVME_SC_DNR;

	cq->qid = cqid;
	cq->size = qsize;
	cq->flags = cq_flags;
	cq->map.host.phys_addr = cmd->create_cq.prp1;

	return NVME_SC_SUCCESS;
}

static int pci_epf_nvme_admin_create_sq(struct pci_epf_nvme *nvme,
					struct nvme_command *cmd)
{
	struct pci_epf_nvme_host *host = &nvme->host;
	struct pci_epf_nvme_queue *sq;
	u16 sqid, cqid, sq_flags, qsize;

	sqid = le16_to_cpu(cmd->create_sq.sqid);
	if (!sqid || sqid > PCI_EPF_NVME_QIDMAX)
		return NVME_SC_QID_INVALID | NVME_SC_DNR;

	cqid = le16_to_cpu(cmd->create_sq.cqid);
	if (sqid != cqid)
		return NVME_SC_CQ_INVALID | NVME_SC_DNR;

	sq_flags = le16_to_cpu(cmd->create_sq.sq_flags);
	if (sq_flags != NVME_QUEUE_PHYS_CONTIG)
		return NVME_SC_INVALID_QUEUE | NVME_SC_DNR;

	qsize = le16_to_cpu(cmd->create_sq.qsize);
	if (!qsize)
		return NVME_SC_QUEUE_SIZE | NVME_SC_DNR;

	sq = &host->sq[sqid];
	if (sq->qid)
		return NVME_SC_QID_INVALID | NVME_SC_DNR;

	sq->qid = sqid;
	sq->size = qsize;
	sq->flags = sq_flags;
	sq->map.host.phys_addr = cmd->create_sq.prp1;

	if (host->cmb.size)
		sq->map.pci.virt_addr = host->cmb.virt_dma_addr;

	if (pci_epf_nvmet_connect(nvme, sqid, qsize, cmd->create_sq.command_id))
		return NVME_SC_INTERNAL | NVME_SC_DNR;

	if (pci_epf_nvme_host_queue_pair(nvme, sqid, cqid))
		return NVME_SC_INTERNAL | NVME_SC_DNR;

	return NVME_SC_SUCCESS;
}

static bool pci_epf_nvme_admin_cmd_handler(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_cmd *epcmd;
	struct nvmet_req *req;
	struct nvme_command *cmd;
	size_t transfer_len;
	int status = NVME_SC_SUCCESS;
	int ret;

	/* Fetch new command from admin submission queue */
	epcmd = pci_epf_nvme_fetch_host_command(nvme, 0);
	if (!epcmd)
		return false;

	req = &epcmd->req;
	cmd = &epcmd->cmd;

	/* Admin commands use prp1 only */
	if (cmd->common.flags & NVME_CMD_SGL_ALL) {
		status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
		goto complete;
	}

	switch (cmd->common.opcode) {
	case nvme_admin_identify:
		transfer_len = NVME_IDENTIFY_DATA_SIZE;
		break;

	case nvme_admin_get_log_page:
		transfer_len = nvmet_get_log_page_len(cmd);
		break;

	case nvme_admin_get_features:
		transfer_len = nvmet_feat_data_len(req,
					le32_to_cpu(cmd->common.cdw10));
		break;

	case nvme_admin_async_event:
	case nvme_admin_set_features:
	case nvme_admin_keep_alive:
	case nvme_admin_abort_cmd:
		transfer_len = 0;
		break;

	case nvme_admin_create_cq:
		status = pci_epf_nvme_admin_create_cq(nvme, cmd);
		goto complete;

	case nvme_admin_create_sq:
		status = pci_epf_nvme_admin_create_sq(nvme, cmd);
		goto complete;

	case nvme_admin_delete_cq:
	case nvme_admin_delete_sq:
	case nvme_admin_ns_attach:
		goto complete;

	default:
		status = NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
		goto complete;
	}

	if (transfer_len) {
		if (transfer_len > NVME_CTRL_PAGE_SIZE) {
			status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
			goto complete;
		}

		epcmd->transfer_len = transfer_len;

		ret = pci_epf_nvme_map_prp1(nvme, cmd);
		if (ret) {
			status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
			goto complete;
		}
	}

	pci_epf_nvme_exec_admin_cmd(nvme, epcmd);

	if (transfer_len)
		pci_epf_nvme_unmap_prp1(nvme);

complete:
	pci_epf_nvme_complete_host_command(nvme, epcmd, status);

	return true;
}

static bool pci_epf_nvme_io_cmd_handler(struct pci_epf_nvme *nvme, int qid)
{
	struct pci_epf_nvme_cmd *epcmd;
	struct nvmet_req *req;
	struct nvme_command *cmd;
	int ret, status;

	/* Fetch new command from IO submission queue */
	epcmd = pci_epf_nvme_fetch_host_command(nvme, qid);
	if (!epcmd)
		return false;

	req = &epcmd->req;
	cmd = &epcmd->cmd;

	switch (cmd->common.opcode) {
	case nvme_cmd_write:
	case nvme_cmd_read:
		ret = pci_epf_nvme_map_dptr(nvme, cmd);
		if (ret) {
			status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
			goto complete;
		}
		break;

	case nvme_cmd_dsm:
		ret = pci_epf_nvme_map_prp1(nvme, cmd);
		if (ret) {
			status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
			goto complete;
		}
		break;

	case nvme_cmd_flush:
	case nvme_cmd_write_zeroes:
		break;

	default:
		status = NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
		goto complete;
	}

	/* XXXXXX BROKEN !!! FIX XXXXXXXXX */
	status = pci_epf_nvmet_execute_request(nvme, qid);
	if (status)
		dev_err(&nvme->epf->dev,
			"Execute I/O command failed\n");

	pci_epf_nvme_unmap_dptr(nvme);

complete:
	pci_epf_nvme_complete_host_command(nvme, epcmd, status);

	return true;
}

static void pci_epf_nvme_cmd_handler(struct work_struct *work)
{
	struct pci_epf_nvme *nvme = container_of(work, struct pci_epf_nvme,
						 cmd_handler.work);
	struct pci_epf_nvme_host *host = &nvme->host;
	struct pci_epf *epf = nvme->epf;
	struct device *dev = &epf->dev;
	bool did_work;
	int qid;

	nvme->tick++;

	host->cc = pci_epf_nvme_host_read32(host, NVME_REG_CC);
	if (host->cc & NVME_CC_ENABLE) {
		if ((host->csts & NVME_CSTS_RDY) == 0) {
			dev_dbg(dev, "CC 0x%x NVME_CC_ENABLE set tick %d\n",
				host->cc, nvme->tick);
			pci_epf_nvme_target_start(nvme);
			pci_epf_nvme_host_start(nvme);
		}

		do {
			/* Process admin and IO commands */
			did_work = pci_epf_nvme_admin_cmd_handler(nvme);
			for (qid = 1; qid <= PCI_EPF_NVME_QIDMAX; qid++)
				did_work |=
					pci_epf_nvme_io_cmd_handler(nvme, qid);
		} while (did_work);

		pci_epf_nvme_target_keep_alive(nvme);

		goto out;
	}

	if (host->csts & NVME_CSTS_RDY) {
		dev_dbg(dev, "CC 0x%x NVME_CC_ENABLE clear tick %d\n",
			host->cc, nvme->tick);
		pci_epf_nvme_host_stop(nvme);
		pci_epf_nvme_target_stop(nvme);
	}

out:
	queue_delayed_work(epf_nvme_workqueue, &nvme->cmd_handler,
			   msecs_to_jiffies(1));
}

static void pci_epf_nvme_unbind(struct pci_epf *epf)
{
	struct pci_epf_nvme *nvme = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	struct pci_epf_bar *epf_bar;
	int bar;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	cancel_delayed_work(&nvme->cmd_handler);
	for (bar = BAR_0; bar <= BAR_5; bar++) {
		epf_bar = &epf->bar[bar];

		if (nvme->reg[bar]) {
			pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no,
					  epf_bar);
			pci_epf_free_space(epf, nvme->reg[bar], bar,
					   PRIMARY_INTERFACE);
		}
	}
}

static int pci_epf_nvme_set_bar(struct pci_epf_nvme *nvme)
{
	struct pci_epf *epf = nvme->epf;
	struct pci_epc *epc = epf->epc;
	const struct pci_epc_features *features = nvme->epc_features;
	enum pci_barno reg_bar = nvme->reg_bar;
	struct pci_epf_bar *epf_bar;
	int bar, add;
	int ret;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar += add) {
		/*
		 * pci_epc_set_bar() sets PCI_BASE_ADDRESS_MEM_TYPE_64
		 * if the specific implementation required a 64-bit BAR,
		 * even if we only requested a 32-bit BAR.
		 */
		epf_bar = &epf->bar[bar];
		if (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
			add = 2;
		else
			add = 1;

		if (features->reserved_bar & (1 << bar))
			continue;

		ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no,
				      epf_bar);
		if (ret) {
			dev_err(&epf->dev, "Failed to set BAR%d\n", bar);
			pci_epf_free_space(epf, nvme->reg[bar], bar,
					   PRIMARY_INTERFACE);
			if (bar == reg_bar)
				return ret;
		}
	}

	return 0;
}

static int pci_epf_nvme_alloc_space(struct pci_epf_nvme *nvme)
{
	const struct pci_epc_features *features = nvme->epc_features;
	enum pci_barno reg_bar = nvme->reg_bar;
	struct pci_epf *epf = nvme->epf;
	struct pci_epf_bar *epf_bar;
	size_t reg_bar_size;
	size_t msix_table_size = 0;
	size_t pba_size = 0;
	int bar, add;
	size_t align = PAGE_SIZE;
	void *base;

	dev_dbg(&epf->dev, "%s\n", __FUNCTION__);

	if (features->msix_capable) {
		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		nvme->msix_table_offset = bar_size[reg_bar];
		/* Align to QWORD or 8 Bytes */
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);
	}
	reg_bar_size = bar_size[reg_bar] + msix_table_size + pba_size;

	if (features->bar_fixed_size[reg_bar]) {
		if (reg_bar_size > bar_size[reg_bar])
			return -ENOMEM;
		reg_bar_size = bar_size[reg_bar];
	}

	if (features->align)
		align = features->align;
	base = pci_epf_alloc_space(epf, bar_size[reg_bar], reg_bar, align,
				   PRIMARY_INTERFACE);
	if (!base) {
		dev_err(&epf->dev, "Failed to allocated register space\n");
		return -ENOMEM;
	}
	nvme->reg[reg_bar] = base;

	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		if (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
			add = 2;
		else
			add = 1;

		if (bar == reg_bar)
			continue;

		if (features->reserved_bar & (1 << bar))
			continue;

		dev_dbg(&epf->dev, "Allocating BAR%d (%zu B)\n",
			bar, bar_size[bar]);

		base = pci_epf_alloc_space(epf, bar_size[bar], bar, align,
					   PRIMARY_INTERFACE);
		if (!base)
			dev_err(&epf->dev, "Allocate BAR%d space failed\n",
				bar);
		nvme->reg[bar] = base;
	}

	return 0;
}

static void pci_epf_nvme_configure_bar(struct pci_epf_nvme *nvme)
{
	struct pci_epf *epf = nvme->epf;
	struct pci_epf_bar *epf_bar;
	int i;

	for (i = BAR_0; i < PCI_STD_NUM_BARS; i++) {
		epf_bar = &epf->bar[i];
		if (nvme->epc_features->bar_fixed_64bit & (1 << i))
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
		if (nvme->epc_features->bar_fixed_size[i])
			bar_size[i] = nvme->epc_features->bar_fixed_size[i];
	}
}

static int pci_epf_nvme_core_init(struct pci_epf_nvme *nvme)
{
	struct pci_epf *epf = nvme->epf;
	struct pci_epc *epc = epf->epc;
	int ret;

	dev_dbg(&epf->dev, "%s\n", __FUNCTION__);

	if (epf->vfunc_no <= 1) {
		ret = pci_epc_write_header(epc, epf->func_no, epf->vfunc_no,
					   epf->header);
		if (ret) {
			dev_err(&epf->dev,
				"Configuration header write failed\n");
			return ret;
		}
	}

	ret = pci_epf_nvme_set_bar(nvme);
	if (ret)
		return ret;

	if (nvme->epc_features->msi_capable) {
		dev_dbg(&epf->dev, "MSI capable, %d interrupts\n",
			epf->msi_interrupts);
		ret = pci_epc_set_msi(epc, epf->func_no, epf->vfunc_no,
				      epf->msi_interrupts);
		if (ret) {
			dev_err(&epf->dev, "MSI configuration failed\n");
			return ret;
		}
	}

	if (nvme->epc_features->msix_capable) {
		dev_dbg(&epf->dev, "MSIX capable, %d interrupts\n",
			epf->msix_interrupts);
		ret = pci_epc_set_msix(epc, epf->func_no, epf->vfunc_no,
				       epf->msix_interrupts,
				       nvme->reg_bar,
				       nvme->msix_table_offset);
		if (ret) {
			dev_err(&epf->dev, "MSI-X configuration failed\n");
			return ret;
		}
	}

	pci_epf_nvme_target_init(nvme);
	ret = pci_epf_nvme_target_start(nvme);
	if (ret)
		return ret;

	pci_epf_nvme_host_init(nvme);
	pci_epf_nvme_target_stop(nvme);

	return 0;
}

static int pci_epf_nvme_notifier(struct notifier_block *nb, unsigned long val,
				 void *data)
{
	struct pci_epf *epf = container_of(nb, struct pci_epf, nb);
	struct pci_epf_nvme *nvme = epf_get_drvdata(epf);
	int ret;

	switch (val) {
	case CORE_INIT:
		ret = pci_epf_nvme_core_init(nvme);
		if (ret)
			return NOTIFY_BAD;
		break;

	case LINK_UP:
		queue_delayed_work(epf_nvme_workqueue, &nvme->cmd_handler,
				   msecs_to_jiffies(1));
		break;

	default:
		dev_err(&epf->dev,
			"Invalid EPF nvme notifier event 0x%lx\n", val);
		return NOTIFY_BAD;
	}

	return NOTIFY_OK;
}

static int pci_epf_nvme_bind(struct pci_epf *epf)
{
	struct pci_epf_nvme *nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc = epf->epc;
	enum pci_barno reg_bar;
	int ret;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	epc_features = pci_epc_get_features(epc, epf->func_no, epf->vfunc_no);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}

	if (!nvme->ns_device_path) {
		dev_err(&epf->dev, "NS device path is not set\n");
		return -EINVAL;
	}

	dev_dbg(&epf->dev, "Bind device %s\n", nvme->ns_device_path);

	reg_bar = pci_epc_get_first_free_bar(epc_features);
	if (reg_bar < 0)
		return -EINVAL;

	nvme->reg_bar = reg_bar;
	nvme->epc_features = epc_features;

	pci_epf_nvme_configure_bar(nvme);

	ret = pci_epf_nvme_alloc_space(nvme);
	if (ret)
		return ret;

	if (!epc_features->core_init_notifier) {
		ret = pci_epf_nvme_core_init(nvme);
		if (ret)
			return ret;
	}

	if (epc_features->linkup_notifier || epc_features->core_init_notifier) {
		epf->nb.notifier_call = pci_epf_nvme_notifier;
		pci_epc_register_notifier(epc, &epf->nb);
	} else {
		queue_work(epf_nvme_workqueue, &nvme->cmd_handler.work);
	}

	return 0;
}

static struct pci_epf_header epf_nvme_pci_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.progif_code	= 0x02, /* NVM Express */
	.subclass_code	= 0x08, /* Non-Volatile Memory controller */
	.baseclass_code = PCI_BASE_CLASS_STORAGE,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static int pci_epf_nvme_probe(struct pci_epf *epf)
{
	struct pci_epf_nvme *nvme;

	dev_dbg(&epf->dev, "Probe func %d, vfunc %d\n",
		epf->func_no, epf->vfunc_no);

	nvme = devm_kzalloc(&epf->dev, sizeof(*nvme), GFP_KERNEL);
	if (!nvme)
		return -ENOMEM;

	nvme->epf = epf;
	epf->header = &epf_nvme_pci_header;
	epf_set_drvdata(epf, nvme);

	nvme->hostnqn = devm_kstrdup(&epf->dev, "hostnqn", GFP_KERNEL);
	if (!nvme->hostnqn)
		return -ENOMEM;

	nvme->subsysnqn = devm_kstrdup(&epf->dev, "testnqn", GFP_KERNEL);
	if (!nvme->subsysnqn)
		return -ENOMEM;

	INIT_DELAYED_WORK(&nvme->cmd_handler, pci_epf_nvme_cmd_handler);

	return 0;
}

#define to_epf_nvme(epf_group) container_of((epf_group), struct pci_epf_nvme, group)

#define PCI_EPF_NVME_R(_name)						\
static ssize_t pci_epf_nvme_##_name##_show(struct config_item *item,	\
					   char *page)			\
{									\
	struct config_group *group = to_config_group(item);		\
	struct pci_epf_nvme *nvme = to_epf_nvme(group);			\
									\
	if (!nvme->_name)						\
		return 0;						\
									\
	return sysfs_emit(page, "%s\n", nvme->_name);			\
}

#define PCI_EPF_NVME_W(_name)						\
static ssize_t pci_epf_nvme_##_name##_store(struct config_item *item,	\
					const char *page, size_t len)	\
{									\
	struct config_group *group = to_config_group(item);		\
	struct pci_epf_nvme *nvme = to_epf_nvme(group);			\
									\
	if (!strlen(page))						\
		return -EINVAL;						\
									\
	if (nvme->_name)						\
		kfree(nvme->_name);					\
									\
	nvme->_name = devm_kstrdup(&nvme->epf->dev, page, GFP_KERNEL);	\
	if (!nvme->_name)						\
		return -ENOMEM;						\
									\
	return len;							\
}

PCI_EPF_NVME_R(hostnqn)
PCI_EPF_NVME_W(hostnqn)
CONFIGFS_ATTR(pci_epf_nvme_, hostnqn);

PCI_EPF_NVME_R(subsysnqn)
PCI_EPF_NVME_W(subsysnqn)
CONFIGFS_ATTR(pci_epf_nvme_, subsysnqn);

PCI_EPF_NVME_R(ns_device_path)
PCI_EPF_NVME_W(ns_device_path)
CONFIGFS_ATTR(pci_epf_nvme_, ns_device_path);

static struct configfs_attribute *pci_epf_nvme_attrs[] = {
	&pci_epf_nvme_attr_hostnqn,
	&pci_epf_nvme_attr_subsysnqn,
	&pci_epf_nvme_attr_ns_device_path,
	NULL,
};

static const struct config_item_type pci_epf_nvme_group_type = {
	.ct_attrs	= pci_epf_nvme_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_group *pci_epf_nvme_add_cfs(struct pci_epf *epf,
						 struct config_group *group)
{
	struct pci_epf_nvme *nvme = epf_get_drvdata(epf);

	/* Add the NVMe target attributes */
	config_group_init_type_name(&nvme->group, "nvmet",
				    &pci_epf_nvme_group_type);

	return &nvme->group;
}

static struct pci_epf_ops pci_epf_nvme_ops = {
	.unbind	= pci_epf_nvme_unbind,
	.bind	= pci_epf_nvme_bind,
	.add_cfs = pci_epf_nvme_add_cfs,
};

static const struct pci_epf_device_id pci_epf_nvme_ids[] = {
	{ .name = "pci_epf_nvme" },
	{},
};

static struct pci_epf_driver epf_nvme_driver = {
	.driver.name	= "pci_epf_nvme",
	.probe		= pci_epf_nvme_probe,
	.id_table	= pci_epf_nvme_ids,
	.ops		= &pci_epf_nvme_ops,
};

static int __init pci_epf_nvme_init(void)
{
	int ret;

	epf_nvme_workqueue = alloc_workqueue("epf_nvme",
					     WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!epf_nvme_workqueue)
		return -ENOMEM;

	epf_nvme_cmd_cache = kmem_cache_create("epf_nvme_host_cmd",
					       sizeof(struct pci_epf_nvme_cmd),
					       0, SLAB_HWCACHE_ALIGN, NULL);
	if (!epf_nvme_cmd_cache) {
		ret = -ENOMEM;
		goto out_wq;
	}

	ret = pci_epf_register_driver(&epf_nvme_driver);
	if (ret) {
		pr_err("Register driver failed %d\n",
		       ret);
		goto out_cache;
	}

	pr_info("Registered driver\n");

	return 0;

out_cache:
	kmem_cache_destroy(epf_nvme_cmd_cache);
out_wq:
	destroy_workqueue(epf_nvme_workqueue);

	return ret;
}
module_init(pci_epf_nvme_init);

static void __exit pci_epf_nvme_exit(void)
{
	if (epf_nvme_workqueue)
		destroy_workqueue(epf_nvme_workqueue);

	pci_epf_unregister_driver(&epf_nvme_driver);

	kmem_cache_destroy(epf_nvme_cmd_cache);

	pr_info("Unregistered driver\n");
}
module_exit(pci_epf_nvme_exit);

MODULE_DESCRIPTION("PCI EPF NVME FUNCTION DRIVER");
MODULE_AUTHOR("Alan Mikhak <alan.mikhak@sifive.com>");
MODULE_LICENSE("GPL v2");
