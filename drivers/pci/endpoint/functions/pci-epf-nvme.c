// SPDX-License-Identifier: GPL-2.0
/*
 * NVMe function driver for PCI Endpoint Framework
 *
 * Copyright (C) 2019 SiFive
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/nvme.h>
#include <generated/utsrelease.h>

#include "../../../nvme/host/nvme.h"
#include "../../../nvme/target/nvmet.h"

#define PCI_EPF_NVME_HOSTNQN		"hostnqn"
#define PCI_EPF_NVME_SUBSYSNQN		"testnqn"
#define PCI_EPF_NVME_MODEL_NUMBER	"Linux pci-epf-nvme"
#define PCI_EPF_NVME_SERIAL		"12345"
#define PCI_EPF_NVME_FW_REV		UTS_RELEASE

/* Command sets supported: NVMe command set */
#define PCI_EPF_NVME_CSS		1ULL

/* CC.EN timeout in 500msec units */
#define PCI_EPF_NVME_TO			15ULL

/* Zero-based maximum queue entries */
#define PCI_EPF_NVME_MQES		(NVME_AQ_DEPTH - 1)

/* Maximum queue id */
#define PCI_EPF_NVME_MAX_QID		1
#define PCI_EPF_NVME_MAX_NR_QUEUES	(PCI_EPF_NVME_MAX_QID + 1)

/* Keep alive interval: 60s (default: NVME_DEFAULT_KATO = 5s) */
#define PCI_EPF_NVME_KATO		60
#define PCI_EPF_NVME_KEEP_ALIVE_JIFFIES ((HZ * PCI_EPF_NVME_KATO) / 2)

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

/*
 * Define host (root complex side) PCI address space mapping to our local
 * EP PCI address space and local memory space.
 */
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

/*
 * Define host (root complex side) PCI physical address space mapping to our
 * local EP PCI address space and local memory space.
 */
enum pci_epf_nvme_map_direction {
	PCI_EPF_NVME_FROM_HOST,
	PCI_EPF_NVME_TO_HOST,
};

struct pci_epf_nvme_pci_map {
	size_t size;
	phys_addr_t host_phys_addr;
	phys_addr_t phys_addr;
	void __iomem *virt_addr;
	enum pci_epf_nvme_map_direction direction;
};

/*
 * Controller queue definition and mapping.
 */
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

	size_t qes;

	struct pci_epf_nvme_pci_map map;
};

struct pci_epf_nvme_prplist {
	unsigned int count;
	unsigned int index;
	size_t align;
	u64 min;
	u64 max;
	u64 prp[PCI_EPF_NVME_PRP_LIST_ENTRIES];
};

/*
 * Our local emulated controller.
 */
struct pci_epf_nvme_ctrl {
	volatile void __iomem *reg;

	u64 cap;
	u32 vs;
	u32 cc;
	u32 csts;
	u32 aqa;
	u64 asq;
	u64 acq;

	size_t adm_sqes;
	size_t adm_cqes;
	size_t io_sqes;
	size_t io_cqes;
	size_t mps;

	struct pci_epf_nvme_queue sq[PCI_EPF_NVME_MAX_NR_QUEUES];
	struct pci_epf_nvme_queue cq[PCI_EPF_NVME_MAX_NR_QUEUES];
};

struct pci_epf_nvme;

/*
 * Command flags.
 */
#define PCI_EPF_NVME_CMD_ASYNC		(1LU << 0)

/*
 * Descriptor for commands sent by the host. This is also used internally for
 * fabrics commands to control our fabrics target.
 */
struct pci_epf_nvme_cmd {
	struct pci_epf_nvme	*nvme;
	unsigned long		flags;
	int			qid;
	struct pci_epf_nvme_pci_map map;
	void			*buffer;
	struct scatterlist 	sgl;
	struct nvmet_req 	req;
	struct nvme_command 	cmd;
	struct nvme_completion	cqe;
	unsigned int		status;
	struct completion	done;
};

/*
 * The fabrics target we will use behind our emulated controller.
 */
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
	struct nvmet_sq sq[PCI_EPF_NVME_MAX_NR_QUEUES];
	struct nvmet_cq cq[PCI_EPF_NVME_MAX_NR_QUEUES];

	unsigned long keep_alive;

	struct pci_epf_nvme_cmd fabrics_epcmd;
	struct pci_epf_nvme_cmd keep_alive_epcmd;
	struct pci_epf_nvme_cmd *admin_epcmd;

	struct nvmet_req req;
	struct nvme_command cmd;
	struct nvme_completion cqe;
	struct completion done;
	struct sg_table sg_table;
	struct scatterlist sgl[PCI_EPF_NVME_PRP_LIST_ENTRIES + 2];
	struct pci_epf_nvme_map map[PCI_EPF_NVME_PRP_MAPS];
	struct pci_epf_nvme_prplist prplist;
};

/*
 * EPF function private data representing our NVMe subsystem.
 */
struct pci_epf_nvme {
	struct pci_epf		*epf;
	const struct pci_epc_features *epc_features;

	void			*reg[PCI_STD_NUM_BARS];
	enum pci_barno		reg_bar;
	size_t			msix_table_offset;

	unsigned int		nr_vectors;

        bool			dma_supported;
        bool			dma_private;
	struct dma_chan		*dma_chan_tx;
        struct dma_chan		*dma_chan_rx;
        struct dma_chan		*dma_chan;
        dma_cookie_t		dma_cookie;
        enum dma_status		dma_status;
        struct completion	dma_complete;

	struct delayed_work	reg_poll;

	struct pci_epf_nvme_ctrl ctrl;

	struct pci_epf_nvme_target target;

	spinlock_t		qlock;

	struct config_group	group;
	char			model_number[NVMET_MN_MAX_SIZE + 1];
	char			serial[NVMET_SN_MAX_SIZE + 1];
	char			firmware_rev[NVMET_FR_MAX_SIZE + 1];
	char			hostnqn[NVMF_NQN_FIELD_LEN + 1];
	char			subsysnqn[NVMF_NQN_FIELD_LEN + 1];
	char			ns_device_path[PATH_MAX + 1];
};

static inline u32 pci_epf_nvme_reg_read32(struct pci_epf_nvme_ctrl *ctrl,
					  u32 reg)
{
	return readl(ctrl->reg + reg);
}

static inline void pci_epf_nvme_reg_write32(struct pci_epf_nvme_ctrl *ctrl,
					    u32 reg, u32 val)
{
	writel(val, ctrl->reg + reg);
	/* Flush */
	val = readl(ctrl->reg + reg);
}

static inline u64 pci_epf_nvme_reg_read64(struct pci_epf_nvme_ctrl *ctrl,
					  u32 reg)
{
	return lo_hi_readq(ctrl->reg + reg);
}

static inline void pci_epf_nvme_reg_write64(struct pci_epf_nvme_ctrl *ctrl,
					    u32 reg, u64 val)
{
	lo_hi_writeq(val, ctrl->reg + reg);
	/* Flush */
	val = lo_hi_readq(ctrl->reg + reg);
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

static int pci_epf_nvme_pci_map(struct pci_epf_nvme *nvme,
				struct pci_epf_nvme_pci_map *map)
{
	struct pci_epf *epf = nvme->epf;
	struct pci_epc *epc = epf->epc;
	void __iomem *virt_addr;
	phys_addr_t phys_addr;
	int ret;

	if (!map->host_phys_addr || !map->size)
		return -EINVAL;

	virt_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, map->size);
	if (!virt_addr) {
                dev_err(&epf->dev, "Failed to allocate map address\n");
		return -ENOMEM;
	}

	ret = pci_epc_map_addr(epf->epc, epf->func_no, epf->vfunc_no,
			       phys_addr, map->host_phys_addr, map->size);
	if (ret) {
                dev_err(&epf->dev, "Failed to map address\n");
		pci_epc_mem_free_addr(epc, phys_addr, virt_addr, map->size);
		return ret;
	}

	map->phys_addr = phys_addr;
	map->virt_addr = virt_addr;

	return 0;
}

static void pci_epf_nvme_pci_unmap(struct pci_epf_nvme *nvme,
				   struct pci_epf_nvme_pci_map *map)
{
	struct pci_epf *epf = nvme->epf;
	struct pci_epc *epc = epf->epc;

	if (!map->virt_addr || !map->phys_addr || !map->size)
		return;

	pci_epc_unmap_addr(epc, epf->func_no, epf->vfunc_no, map->phys_addr);
	pci_epc_mem_free_addr(epc, map->phys_addr, map->virt_addr, map->size);
}

static int pci_epf_nvme_pci_read(struct pci_epf_nvme *nvme,    
				 struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme_pci_map *map = &epcmd->map;
	int ret;

	ret = pci_epf_nvme_pci_map(nvme, map);
	if (ret) {
		epcmd->status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
		return ret;
	}

	memcpy_fromio(epcmd->buffer, map->virt_addr, map->size);

	pci_epf_nvme_pci_unmap(nvme, map);

	return 0;
}

static int pci_epf_nvme_pci_write(struct pci_epf_nvme *nvme,    
				  struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme_pci_map *map = &epcmd->map;
	int ret;

	ret = pci_epf_nvme_pci_map(nvme, map);
	if (ret) {
		epcmd->status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
		return ret;
	}

	dev_dbg(&nvme->epf->dev, "Data %zu B to host\n", map->size);
	pci_epf_nvme_dump(nvme, "Data:", epcmd->buffer,
			  min_t(size_t, map->size, 128));

	memcpy_toio(map->virt_addr, epcmd->buffer, map->size);

	pci_epf_nvme_pci_unmap(nvme, map);

	return 0;
}

static struct nvme_command *pci_epf_nvme_init_cmd(struct pci_epf_nvme *nvme,
				struct pci_epf_nvme_cmd *epcmd, int qid)
{
	memset(epcmd, 0, sizeof(*epcmd));
	epcmd->nvme = nvme;
	epcmd->qid = qid;
	epcmd->req.cmd = &epcmd->cmd;
	epcmd->req.cqe = &epcmd->cqe;
	epcmd->req.port = &nvme->target.port;
	epcmd->status = NVME_SC_SUCCESS;
	init_completion(&epcmd->done);

	return &epcmd->cmd;
}

static struct pci_epf_nvme_cmd *pci_epf_nvme_alloc_cmd(struct pci_epf_nvme *nvme)
{
	return kmem_cache_alloc(epf_nvme_cmd_cache, GFP_KERNEL);
}

static void pci_epf_nvme_free_cmd(struct pci_epf_nvme_cmd *epcmd)
{
	if (WARN_ON_ONCE(epcmd == &epcmd->nvme->target.fabrics_epcmd))
		return;

	kmem_cache_free(epf_nvme_cmd_cache, epcmd);
}

static int pci_epf_nvme_cmd_map(struct pci_epf_nvme *nvme,
				struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme_pci_map *map = &epcmd->map;
	struct scatterlist *sg = &epcmd->sgl;
	int ret = 0;

	/* Get a local buffer and set an SGL for it */
	epcmd->buffer = kzalloc(map->size, GFP_NOIO);
	if (!epcmd->buffer)
		return -ENOMEM;

	sg_set_page(sg, virt_to_page(epcmd->buffer), map->size, 
		    offset_in_page(epcmd->buffer));
	sg_init_marker(sg, 1);

	epcmd->req.transfer_len = map->size;
	epcmd->req.sg = sg;
	epcmd->req.sg_cnt = 1;

	/* Get data from the host if needed */
	if (map->direction == PCI_EPF_NVME_FROM_HOST) {
		ret = pci_epf_nvme_pci_read(nvme, epcmd);
		if (ret) {
			kfree(epcmd->buffer);
			epcmd->buffer = NULL;
			return ret;
		}
	}

	return 0;
}

static void pci_epf_nvme_cmd_unmap(struct pci_epf_nvme *nvme,
				   struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme_pci_map *map = &epcmd->map;

	if (!map->size)
		return;

	/* Send data to the host if needed */
	if (epcmd->status == NVME_SC_SUCCESS &&
	    map->direction == PCI_EPF_NVME_TO_HOST)
		pci_epf_nvme_pci_write(nvme, epcmd);

	kfree(epcmd->buffer);
	epcmd->buffer = NULL;
}

static void pci_epf_nvme_raise_irq(struct pci_epf_nvme *nvme,
				   struct pci_epf_nvme_queue *cq)
{
	struct pci_epf *epf = nvme->epf;
	struct pci_epc *epc = epf->epc;
	int ret, irq = cq->vector + 1;

	if (!(cq->flags & NVME_CQ_IRQ_ENABLED))
		return;

	/* Try MSIX first */
	if (epf->msix_interrupts &&
            pci_epc_get_msix(epc, epf->func_no, epf->vfunc_no) > 0) {
		ret = pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
					PCI_EPC_IRQ_MSI, irq);
		if (ret)
			dev_err(&epf->dev, "Raise MSI IRQ %d failed %d\n",
				irq, ret);
		return;
	}

	/* Next try MSI */
	if (epf->msi_interrupts &&
	    pci_epc_get_msi(epc, epf->func_no, epf->vfunc_no) > 0) {
		ret = pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
					PCI_EPC_IRQ_MSI, irq);
		if (ret)
			dev_err(&epf->dev, "Raise MSI IRQ %d failed %d\n",
				irq, ret);
		return;
	}

	/* If both MSIX and MSI are not supported/enabled, fallback to INTX */
	ret = pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
				PCI_EPC_IRQ_LEGACY, 0);
	if (ret)
		dev_err(&epf->dev, "Raise legacy IRQ failed %d\n", ret);
}

static void pci_epf_nvme_cmd_complete(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *nvme = epcmd->nvme;
	struct pci_epf_nvme_ctrl *ctrl = &nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[epcmd->qid];
	struct pci_epf_nvme_queue *cq = &ctrl->cq[epcmd->qid];
	struct pci_epf *epf = nvme->epf;
	struct nvmet_req *req = &epcmd->req;
	struct nvme_command *cmd = req->cmd;
	struct nvme_completion *cqe = req->cqe;
	unsigned long flags;
	int tail = cq->tail;

	spin_lock_irqsave(&nvme->qlock, flags);

	/* XXX Check completion queue full state XXX */
	cq->head = pci_epf_nvme_reg_read32(ctrl, cq->db);

	/* Setup the completion entry */
	cqe->sq_id = cpu_to_le16(epcmd->qid);
	cqe->sq_head = cpu_to_le16(sq->head);
	cqe->command_id = cmd->common.command_id;
	cqe->status = cpu_to_le16((epcmd->status << 1) | cq->phase);

	/* Post the completion entry */
	dev_dbg(&epf->dev, "cq[%d]: status 0x%x, phase %d, tail %d -> %d/%d, head %d\n",
		epcmd->qid, epcmd->status, cq->phase, tail,
		(int)cq->tail, (int)cq->depth, (int)cq->head);

	memcpy_toio(cq->map.virt_addr + tail * cq->qes, cqe, cq->qes);

	pci_epf_nvme_dump(nvme, "cqe", cqe, cq->qes);

	/* Advance cq tail */
	cq->tail++;
	if (cq->tail >= cq->depth) {
		cq->tail = 0;
		cq->phase ^= 1;
	}

	spin_unlock_irqrestore(&nvme->qlock, flags);

	pci_epf_nvme_raise_irq(nvme, cq);

	if (epcmd->flags & PCI_EPF_NVME_CMD_ASYNC)
		pci_epf_nvme_free_cmd(epcmd);
}

static int pci_epf_nvme_map_alloc_region(struct pci_epf_nvme *nvme,
					 struct pci_epf_nvme_map *map)
{
	if (map->pci.phys_base)
		return -EALREADY;

	map->align = max_t(size_t, nvme->epc_features->align, PAGE_SIZE);
	if (map->size < map->align)
		map->pci.size = map->align * 2;
	else
		map->pci.size = map->size;

	map->pci.virt_base = pci_epc_mem_alloc_addr(nvme->epf->epc,
					&map->pci.phys_base, map->pci.size);
	if (!map->pci.virt_base)
		return -ENOMEM;

	dev_dbg(&nvme->epf->dev,
		"Allocated region: virt 0x%p, phys 0x%llx, %zu B\n",
		map->pci.virt_base, map->pci.phys_base, map->pci.size);

	return 0;
}

static void pci_epf_nvme_map_free_region(struct pci_epf_nvme *nvme,
					 struct pci_epf_nvme_map *map)
{
	if (!map->pci.phys_base)
		return;

	dev_dbg(&nvme->epf->dev,
		"Free region: virt 0x%p, phys 0x%llx, %zu B\n",
		map->pci.virt_base, map->pci.phys_base, map->pci.size);

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

	dev_dbg(&nvme->epf->dev,
		"Enabled region: virt 0x%p, phys 0x%llx, host phys 0x%llx, "
		"offset %lu B, sz %zu B\n",
		map->pci.virt_base, map->pci.phys_base,
		map->host.phys_base, map->offset, map->pci.size);

	return 0;
}

static void pci_epf_nvme_map_disable_region(struct pci_epf_nvme *nvme,
					    struct pci_epf_nvme_map *map)
{
	struct pci_epf *epf = nvme->epf;

	if (!map->pci.phys_addr)
		return;

	dev_dbg(&nvme->epf->dev,
                "Disable region: pci virt 0x%p, phys 0x%llx, sz %zu B\n",
                map->pci.virt_base, map->pci.phys_base,
                map->pci.size);

	pci_epc_unmap_addr(epf->epc, epf->func_no, epf->vfunc_no,
			   map->pci.phys_base);
	map->pci.phys_addr = 0;
}

static int pci_epf_nvme_map(struct pci_epf_nvme *nvme,
			    struct pci_epf_nvme_map *map)
{
	int ret;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

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
	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	pci_epf_nvme_map_disable_region(nvme, map);
	pci_epf_nvme_map_free_region(nvme, map);

	memset(map, 0, sizeof(*map));
}

static int pci_epf_nvme_map_slot(struct pci_epf_nvme *nvme, int slot,
				 u64 addr, size_t size)
{
	struct pci_epf_nvme_map *map;

	dev_dbg(&nvme->epf->dev,
		"%s: slot %d, addr 0x%llx, %zu B\n",
		__FUNCTION__, slot, addr, size);

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

static int pci_epf_nvme_expand_prplist(struct pci_epf_nvme *nvme,
				       unsigned int more)
{
	struct pci_epf_nvme_prplist *list = &nvme->target.prplist;
	u64 prp;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

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

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

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

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

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

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

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

static int pci_epf_nvmet_add_port(struct nvmet_port *nvmet_port)
{
	return -ENOTSUPP;
}

static void pci_epf_nvmet_remove_port(struct nvmet_port *nvmet_port)
{
}

static void pci_epf_nvmet_queue_response(struct nvmet_req *req)
{
	struct pci_epf_nvme_cmd *epcmd =
		container_of(req, struct pci_epf_nvme_cmd, req);

	/* Get completion status from the target */
	epcmd->status = le16_to_cpu(req->cqe->status) >> 1;

	if (epcmd->flags & PCI_EPF_NVME_CMD_ASYNC) {
		pci_epf_nvme_cmd_complete(epcmd);
		return;
	}

	complete(&epcmd->done);
}

static void pci_epf_nvmet_delete_ctrl(struct nvmet_ctrl *nvmet_ctrl)
{
	pr_warn("Unhandled call to delete controller\n");
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

static int pci_epf_nvmet_execute_io_request(struct pci_epf_nvme *nvme, int qid)
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
		req->cqe->status = cpu_to_le16(status << 1);
		return status;
	}
}

/*
 * Execute a fabrics command using a local buffer.
 */
static int pci_epf_nvme_exec_fabrics_cmd(struct pci_epf_nvme *nvme,
					 void *buffer, size_t buffer_size)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct pci_epf_nvme_cmd *epcmd = &target->fabrics_epcmd;
	struct nvmet_req *req = &epcmd->req;
	struct nvme_command *cmd = req->cmd;
	struct scatterlist *sg = &epcmd->sgl;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	/* NVMe target mandates SGL for fabrics commands */
	cmd->common.flags &= ~(NVME_CMD_SGL_ALL);
	cmd->common.flags |= NVME_CMD_SGL_METABUF;

	if (!nvmet_req_init(req, &target->cq[epcmd->qid],
			    &target->sq[epcmd->qid],
			    &pci_epf_nvmet_fabrics_ops)) {
		dev_err(&nvme->epf->dev, "nvmet_req_init failed\n");
		return -EIO;
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

	if (epcmd->status != NVME_SC_SUCCESS)
		return -EIO;

	return 0;
}

static int pci_epf_nvmet_connect(struct pci_epf_nvme *nvme,
				 u16 qid, u16 qsize)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmf_connect_data *data;
	struct nvme_command *cmd;
	int ret;

	dev_dbg(&nvme->epf->dev, "Execute connect cmd: qid %d\n", (int)qid);

	cmd = pci_epf_nvme_init_cmd(nvme, &target->fabrics_epcmd, qid);
	cmd->connect.opcode = nvme_fabrics_command;
	cmd->connect.fctype = nvme_fabrics_type_connect;
	cmd->connect.qid = cpu_to_le16(qid);
	cmd->connect.sqsize = cpu_to_le16(qsize);

	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	strncpy(data->hostnqn, nvmet_host_name(target->host_link.host),
		NVMF_NQN_SIZE);
	strncpy(data->subsysnqn, target->subsys.subsysnqn, NVMF_NQN_SIZE);
	if (!qid) {
		/* Connect admin queue */
		cmd->connect.kato = cpu_to_le32(PCI_EPF_NVME_KATO * 1000);
		uuid_gen(&data->hostid);
		data->cntlid = 0xffff;
	} else {
		/* Connect IO queue */
		uuid_copy(&data->hostid, &target->nvmet_ctrl->hostid);
		data->cntlid = cpu_to_le16(target->nvmet_ctrl->cntlid);
	}

	ret = pci_epf_nvme_exec_fabrics_cmd(nvme, data, sizeof(*data));
	if (ret)
		dev_err(&nvme->epf->dev, "Connect command failed\n");

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

	memset(target, 0, sizeof(*target));

	target->dev = &epf->dev;
	target->keep_alive = jiffies + PCI_EPF_NVME_KEEP_ALIVE_JIFFIES;
	init_completion(&target->done);

	subsys->type = NVME_NQN_NVME;
	subsys->max_qid = PCI_EPF_NVME_MAX_QID;
	kref_init(&subsys->ref);
	mutex_init(&subsys->lock);
	xa_init(&subsys->namespaces);
	INIT_LIST_HEAD(&subsys->hosts);
	INIT_LIST_HEAD(&subsys->ctrls);
	subsys->subsysnqn = nvme->subsysnqn;
	subsys->model_number = nvme->model_number;
	strncpy(subsys->serial, nvme->serial, sizeof(subsys->serial));
	subsys->firmware_rev = nvme->firmware_rev;

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

	host->group.cg_item.ci_name = nvme->hostnqn;
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
	ns->buffered_io = true;
	memcpy(&ns->nguid, nguid, sizeof(ns->nguid));
	uuid_gen(&ns->uuid);

	for (qid = 0; qid <= PCI_EPF_NVME_MAX_QID; qid++) {
		nvmet_sq_init(&target->sq[qid]);
		target->sq[qid].sqhd = 0;
		target->sq[qid].qid = 0;
		target->sq[qid].size = 0;
		target->sq[qid].ctrl = NULL;

		target->cq[qid].qid = 0;
		target->cq[qid].size = 0;
	}
}

static int pci_epf_nvme_target_connect(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct pci_epf *epf = nvme->epf;
	int ret;

	dev_dbg(&epf->dev, "Connect target\n");

	/* Initialize the target and connect it */
	pci_epf_nvme_target_init(nvme);
	ret = pci_epf_nvmet_connect(nvme, 0, PCI_EPF_NVME_MQES);
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

	return 0;
}

static void pci_epf_nvme_target_disconnect(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	int qid;

	dev_dbg(&nvme->epf->dev, "Disconnect target\n");

	for (qid = 0; qid <= PCI_EPF_NVME_MAX_QID; qid++)
		nvmet_sq_destroy(&target->sq[qid]);

	target->nvmet_ctrl = NULL;
}

static void pci_epf_nvme_ctrl_init(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_ctrl *ctrl = &nvme->ctrl;
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmet_ctrl *nvmet_ctrl = target->nvmet_ctrl;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	ctrl->reg = nvme->reg[nvme->reg_bar];

	/* Maximum queue entries supported (MQES) */
	ctrl->cap = PCI_EPF_NVME_MQES;

	/* Contiguous Queues Required (CQR) */
	ctrl->cap |= 0x1ULL << 16;

	/* CC.EN timeout in 500msec units (TO) */
	ctrl->cap |= PCI_EPF_NVME_TO << 24;

	/* Doorbell stride = 4B (DSTRB) */
	ctrl->cap &= ~GENMASK(35, 32);

	/* NVM Subsystem Reset Supported (NSSRS) */
	ctrl->cap &= ~(0x1ULL << 36);

	/* Command sets supported (CSS) */
	ctrl->cap |= PCI_EPF_NVME_CSS << 37;

	/* Boot Partition Support (BPS) */
	ctrl->cap &= ~(0x1ULL << 45);

	/* Memory Page Size minimum (MPSMIN) */
	ctrl->cap |= NVME_CAP_MPSMIN(nvmet_ctrl->cap);

	/* Memory Page Size maximum (MPSMAX) */
	ctrl->cap |= NVME_CAP_MPSMAX(nvmet_ctrl->cap);

	/* Persistent Memory Region Supported (PMRS) */
	ctrl->cap &= ~(0x1ULL << 56);

	/* Controller Memory Buffer Supported (CMBS) */
	ctrl->cap &= ~(0x1ULL << 57);

	/* Subsystem compliance spec version (1.3.0 by default) */
	ctrl->vs = NVMET_DEFAULT_VS;

	/* Controller configuration */
	ctrl->cc = NVME_CC_CSS_NVM;
	ctrl->cc |= NVME_CC_IOCQES;
	ctrl->cc |= NVME_CC_IOSQES;
	ctrl->cc |= (NVME_CTRL_PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;

	/* Controller Status (not ready) */
	ctrl->csts = 0;

	pci_epf_nvme_reg_write64(ctrl, NVME_REG_CAP, ctrl->cap);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_VS, ctrl->vs);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CSTS, ctrl->csts);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CC, ctrl->cc);
}

static int pci_epf_nvme_ctrl_setup_queue_pair(struct pci_epf_nvme *nvme,
					      int qid)
{
	struct pci_epf_nvme_ctrl *ctrl = &nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[qid];
	struct pci_epf_nvme_queue *cq = &ctrl->cq[qid];
	struct pci_epf *epf = nvme->epf;
	int ret;

	/* Setup the submission queue */
	sq->qid = qid;
	sq->depth = sq->size + 1;
	sq->head = 0;
	sq->tail = 0;
	sq->db = NVME_REG_DBS + (qid * 2 * sizeof(u32));
	pci_epf_nvme_reg_write32(ctrl, sq->db, 0);

	if (!qid)
		sq->qes = ctrl->adm_sqes;
	else
		sq->qes = ctrl->io_sqes;

	sq->map.direction = PCI_EPF_NVME_FROM_HOST;
	sq->map.size = sq->depth * sq->qes;

	ret = pci_epf_nvme_pci_map(nvme, &sq->map);
	if (ret) {
		dev_err(&epf->dev, "Failed to map ctrl SQ%d\n", qid);
		return ret;
	}

	dev_dbg(&epf->dev,
		"SQ %d: host phys addr 0x%llx, virt addr 0x%p, phys addr 0x%llx, size %zu B\n",
		qid, sq->map.host_phys_addr, sq->map.virt_addr,
		sq->map.phys_addr, sq->map.size);

	/* Setup the completion queue */
	cq->qid = qid;
	cq->depth = cq->size + 1;
	cq->head = 0;
	cq->tail = 0;
	cq->phase = 1;
	cq->db = NVME_REG_DBS + (((qid * 2) + 1) * sizeof(u32));
	pci_epf_nvme_reg_write32(ctrl, cq->db, 0);

	if (!qid)
		cq->qes = ctrl->adm_cqes;
	else
		cq->qes = ctrl->io_cqes;

	cq->map.direction = PCI_EPF_NVME_TO_HOST;
	cq->map.size = cq->depth * cq->qes;

	ret = pci_epf_nvme_pci_map(nvme, &cq->map);
	if (ret) {
		dev_err(&epf->dev, "Failed to map ctrl CQ%d\n", qid);
		pci_epf_nvme_pci_unmap(nvme, &sq->map);
		return ret;
	}

	dev_dbg(&epf->dev,
		"CQ %d: host phys addr 0x%llx, virt addr 0x%p, phys addr 0x%llx, size %zu B\n",
		qid, cq->map.host_phys_addr, cq->map.virt_addr,
		cq->map.phys_addr, cq->map.size);

	return 0;
}

static void pci_epf_nvme_ctrl_destroy_queue_pair(struct pci_epf_nvme *nvme,
						 int qid)
{
	struct pci_epf_nvme_queue *sq = &nvme->ctrl.sq[qid];
	struct pci_epf_nvme_queue *cq = &nvme->ctrl.cq[qid];

	if (!sq->map.size)
		return;

	dev_dbg(&nvme->epf->dev, "Destroy queue pair %d\n", qid);

	pci_epf_nvme_pci_unmap(nvme, &sq->map);
	memset(sq, 0, sizeof(struct pci_epf_nvme_queue));

	pci_epf_nvme_pci_unmap(nvme, &cq->map);
	memset(cq, 0, sizeof(struct pci_epf_nvme_queue));
}

static int pci_epf_nvme_ctrl_enable(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct pci_epf_nvme_ctrl *ctrl = &nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[0];
	struct pci_epf_nvme_queue *cq = &ctrl->cq[0];
	struct pci_epf *epf = nvme->epf;
	int ret;

	dev_dbg(&epf->dev, "++++ Enabling controller ++++\n");

	/* Connect the target */
	ret = pci_epf_nvme_target_connect(nvme);
	if (ret)
		return ret;

	ctrl->mps = 1LU << (nvmet_cc_mps(ctrl->cc) + 12);
	ctrl->adm_sqes = 1LU << NVME_ADM_SQES;
	ctrl->adm_cqes = sizeof(struct nvme_completion);
	ctrl->io_sqes = 1LU << nvmet_cc_iosqes(ctrl->cc);
	ctrl->io_cqes = 1LU << nvmet_cc_iocqes(ctrl->cc);

	dev_dbg(&epf->dev,
		"Admin sqes %zu B, admin cqes %zu B, IO sqes %zu B, IO cqes %zu B, mps %zu\n",
		ctrl->adm_sqes, ctrl->adm_cqes,
		ctrl->io_sqes, ctrl->io_cqes, ctrl->mps);

	ctrl->aqa = pci_epf_nvme_reg_read32(ctrl, NVME_REG_AQA);
	ctrl->asq = pci_epf_nvme_reg_read64(ctrl, NVME_REG_ASQ);
	ctrl->acq = pci_epf_nvme_reg_read64(ctrl, NVME_REG_ACQ);

	/*
	 * Setup the controller admin submission and completion queues. The
	 * target admin queues were already created when the target was
	 * connected.
	 */
	sq->flags = NVME_QUEUE_PHYS_CONTIG;
	sq->size = ctrl->aqa & 0x0fff;
	sq->map.host_phys_addr = ctrl->asq & GENMASK(63,12);

	cq->flags = NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED;
	cq->size = (ctrl->aqa & 0x0fff0000) >> 16;
	cq->map.host_phys_addr = ctrl->acq & GENMASK(63,12);

	ret = pci_epf_nvme_ctrl_setup_queue_pair(nvme, 0);
	if (ret)
		return ret;

	/* Enable the namespace and the target controller */
	ret = nvmet_ns_enable(&target->ns);
	if (ret) {
		dev_err(&epf->dev, "Enable namespace failed\n");
		return ret;
	}

	ctrl->cc |= NVME_CC_ENABLE;
	nvmet_update_cc(target->nvmet_ctrl, ctrl->cc);
	if (!(target->nvmet_ctrl->cc & NVME_CC_ENABLE)) {
		dev_err(&epf->dev, "Target controller not enabled!\n");
		/* XXX */
		target->nvmet_ctrl->cc |= NVME_CC_ENABLE;
	}

	/* Tell the host we are now ready */
	ctrl->csts = nvme->target.nvmet_ctrl->csts;
	if (!(ctrl->csts & NVME_CSTS_RDY)) {
		dev_err(&epf->dev, "Target not ready !\n");
		/* XXX */
		ctrl->csts |= NVME_CSTS_RDY;
	}
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CSTS, ctrl->csts);

	/*
	 * Hack: Let things settle down on the link. otherwise command fetching
	 * from the admin SQ generates an NMI sometimes...
	 */
	msleep(20);

	return 0;
}

static void pci_epf_nvme_ctrl_disable(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_ctrl *ctrl = &nvme->ctrl;
	struct pci_epf *epf = nvme->epf;
	int qid;

	dev_dbg(&epf->dev, "---- Disabling controller ----\n");

	/* Disable the target controller */
	ctrl->cc &= ~NVME_CC_ENABLE;
	nvmet_update_cc(nvme->target.nvmet_ctrl, ctrl->cc);

	/*
	 * Disable the target namespace and destroy the IO queues. Keep the
	 * target admin queues !
	 */
	nvmet_ns_disable(&nvme->target.ns);
	for (qid = PCI_EPF_NVME_MAX_QID; qid >= 0; qid--)
		pci_epf_nvme_ctrl_destroy_queue_pair(nvme, qid);

	ctrl->csts &= ~NVME_CSTS_RDY;
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CSTS, ctrl->csts);

	pci_epf_nvme_target_disconnect(nvme);
}

static void pci_epf_nvme_exec_admin_cmd(struct pci_epf_nvme *nvme,
					struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct nvmet_req *req = &epcmd->req;
	struct nvme_command *cmd = req->cmd;
	int ret;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	/* NVMe target mandates SGL for admin commands */
	cmd->common.flags &= ~(NVME_CMD_SGL_ALL);
	cmd->common.flags |= NVME_CMD_SGL_METABUF;

	if (!nvmet_req_init(req, &target->cq[0], &target->sq[0],
			    &pci_epf_nvmet_fabrics_ops)) {
		dev_err(&nvme->epf->dev, "nvmet_req_init failed\n");
		epcmd->status = le16_to_cpu(req->cqe->status) >> 1;
		return;
	}

	/* If we have a command buffer, map it */
	if (epcmd->map.size) {
		ret = pci_epf_nvme_cmd_map(nvme, epcmd);
		if (ret)
			return;
	}

	if (epcmd->flags & PCI_EPF_NVME_CMD_ASYNC)
		target->admin_epcmd = NULL;

	/* Execute the request */
	req->execute(req);

	if (epcmd->flags & PCI_EPF_NVME_CMD_ASYNC)
		return;

	wait_for_completion(&epcmd->done);

	if (epcmd->map.size)
		pci_epf_nvme_cmd_unmap(nvme, epcmd);
}

static void pci_epf_nvme_target_keep_alive(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct pci_epf_nvme_cmd *epcmd = &target->keep_alive_epcmd;
	struct nvme_command *cmd;

	if (!target->nvmet_ctrl)
		return;

	if (time_before(jiffies, target->keep_alive))
		return;

	cmd = pci_epf_nvme_init_cmd(nvme, epcmd, 0);
	cmd->common.opcode = nvme_admin_keep_alive;

	pci_epf_nvme_exec_admin_cmd(nvme, epcmd);

	if (epcmd->status != NVME_SC_SUCCESS)
		dev_err(&nvme->epf->dev, "Execute keep alive failed\n");

	target->keep_alive = jiffies + PCI_EPF_NVME_KEEP_ALIVE_JIFFIES;
}

static void pci_epf_nvme_admin_create_cq(struct pci_epf_nvme *nvme,
					 struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	struct pci_epf_nvme_queue *cq;
	u16 cqid, cq_flags, qsize, vector;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	cqid = le16_to_cpu(cmd->create_cq.cqid);
	if (!cqid || cqid > PCI_EPF_NVME_MAX_QID) {
		dev_err(&nvme->epf->dev, "Invalid cqid %d\n", (int)cqid);
		epcmd->status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		return;
	}

	cq_flags = le16_to_cpu(cmd->create_cq.cq_flags);
	if (!(cq_flags & NVME_QUEUE_PHYS_CONTIG)) {
		dev_err(&nvme->epf->dev, "CQ not phys contig\n");
		epcmd->status = NVME_SC_INVALID_QUEUE | NVME_SC_DNR;
		return;
	}

	qsize = le16_to_cpu(cmd->create_cq.qsize);
	if (!qsize) {
		dev_err(&nvme->epf->dev, "Invalid queue size\n");
		epcmd->status = NVME_SC_QUEUE_SIZE | NVME_SC_DNR;
		return;
	}

	vector = le16_to_cpu(cmd->create_cq.irq_vector);
	if (vector >= nvme->nr_vectors) {
		dev_err(&nvme->epf->dev, "Invalid vector %d / %u\n",
			(int)vector, nvme->nr_vectors);
		epcmd->status = NVME_SC_INVALID_VECTOR | NVME_SC_DNR;
		return;
	}

	cq = &nvme->ctrl.cq[cqid];
	if (cq->qid) {
		dev_err(&nvme->epf->dev, "CQ %d/%d already exist\n",
			(int)cqid, (int)cq->qid);
		epcmd->status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		return;
	}

	cq->qid = cqid;
	cq->size = qsize;
	cq->flags = cq_flags;
	cq->vector = vector;

	cq->map.host_phys_addr = le64_to_cpu(cmd->create_cq.prp1);

	dev_dbg(&nvme->epf->dev, "CQ %d: size %d, vector %d\n",
		(int)cqid, (int)cq->size, (int)cq->vector);
}

static void pci_epf_nvme_admin_create_sq(struct pci_epf_nvme *nvme,
					 struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	struct pci_epf_nvme_ctrl *ctrl = &nvme->ctrl;
	struct pci_epf_nvme_queue *sq;
	u16 sqid, cqid, sq_flags, qsize;
	int ret;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	sqid = le16_to_cpu(cmd->create_sq.sqid);
	if (!sqid || sqid > PCI_EPF_NVME_MAX_QID) {
		dev_err(&nvme->epf->dev, "Invalid sqid %d\n", (int)cqid);
		epcmd->status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		return;
	}

	cqid = le16_to_cpu(cmd->create_sq.cqid);
	if (cqid != sqid || nvme->ctrl.cq[cqid].qid != cqid) {
		dev_err(&nvme->epf->dev, "Invalid cqid %d\n", (int)cqid);
		epcmd->status = NVME_SC_CQ_INVALID | NVME_SC_DNR;
		return;
	}

	sq_flags = le16_to_cpu(cmd->create_sq.sq_flags);
	if (sq_flags != NVME_QUEUE_PHYS_CONTIG) {
		dev_err(&nvme->epf->dev, "SQ not phys contig\n");
		epcmd->status = NVME_SC_INVALID_QUEUE | NVME_SC_DNR;
		return;
	}

	qsize = le16_to_cpu(cmd->create_sq.qsize);
	if (!qsize) {
		dev_err(&nvme->epf->dev, "Invalid queue size\n");
		epcmd->status = NVME_SC_QUEUE_SIZE | NVME_SC_DNR;
		return;
	}

	sq = &ctrl->sq[sqid];
	if (sq->qid) {
		dev_err(&nvme->epf->dev, "SQ %d/%d already exist\n",
			(int)sqid, (int)sq->qid);
		epcmd->status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		return;
	}

	sq->qid = sqid;
	sq->size = qsize;
	sq->flags = sq_flags;
	sq->map.host_phys_addr = le64_to_cpu(cmd->create_sq.prp1);

	dev_dbg(&nvme->epf->dev, "SQ %d: size %d\n",
		(int)sqid, (int)sq->size);

	ret = pci_epf_nvmet_connect(nvme, sqid, qsize);
	if (ret) {
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
		return;
	}

	ret = pci_epf_nvme_ctrl_setup_queue_pair(nvme, sqid);
	if (ret)
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
}

static struct pci_epf_nvme_cmd *
pci_epf_nvme_fetch_command(struct pci_epf_nvme *nvme,
			   struct pci_epf_nvme_cmd *epcmd, int qid)
{
	struct pci_epf_nvme_ctrl *ctrl = &nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[qid];
	struct nvme_command *cmd;
	unsigned long flags;

	if (!sq->size)
		return NULL;

	dma_rmb();

	sq->tail = pci_epf_nvme_reg_read32(ctrl, sq->db);
	if (sq->tail == sq->head) {
		/* Queue empty */
		return NULL;
	}

	dev_dbg(&nvme->epf->dev,
		"++++ New command: sq[%d]: head %d/%d, tail %d\n",
		qid, (int)sq->head, (int)sq->depth, (int)sq->tail);

	if (!epcmd) {
		epcmd = pci_epf_nvme_alloc_cmd(nvme);
		if (!epcmd)
			return NULL;
	}

	/* Get the NVMe command submitted by the host */
	cmd = pci_epf_nvme_init_cmd(nvme, epcmd, qid);
	memcpy_fromio(cmd, sq->map.virt_addr + sq->head * sq->qes, sq->qes);

	pci_epf_nvme_dump(nvme, "cmd", cmd, sizeof(struct nvme_command));

	spin_lock_irqsave(&nvme->qlock, flags);
	sq->head++;
	if (sq->head == sq->depth)
		sq->head = 0;
	spin_unlock_irqrestore(&nvme->qlock, flags);

	return epcmd;
}

static bool pci_epf_nvme_process_admin_cmd(struct pci_epf_nvme *nvme)
{
	struct pci_epf_nvme_target *target = &nvme->target;
	struct pci_epf_nvme_cmd *epcmd;
	struct nvmet_req *req;
	struct nvme_command *cmd;
	size_t transfer_len = 0;

	/* Fetch new command from admin submission queue */
	epcmd = pci_epf_nvme_fetch_command(nvme, target->admin_epcmd, 0);
	if (!epcmd)
		return false;

	/*
	 * Unless we need to execute a nvme_admin_async_event command, we are
	 * going to reuse this command for the next admin command. So save it
	 * if we do not have one saved already.
	 */
	if (!target->admin_epcmd)
		target->admin_epcmd = epcmd;

	req = &epcmd->req;
	cmd = &epcmd->cmd;

	/* Admin commands use prp1 only */
	if (cmd->common.flags & NVME_CMD_SGL_ALL) {
		dev_err(&nvme->epf->dev,
			"Admin command has NVME_CMD_SGL_ALL\n");
		epcmd->status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
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
		epcmd->flags |= PCI_EPF_NVME_CMD_ASYNC;
		break;

	case nvme_admin_set_features:
	case nvme_admin_abort_cmd:
		break;

	case nvme_admin_create_cq:
		pci_epf_nvme_admin_create_cq(nvme, epcmd);
		goto complete;

	case nvme_admin_create_sq:
		pci_epf_nvme_admin_create_sq(nvme, epcmd);
		goto complete;

	case nvme_admin_delete_cq:
	case nvme_admin_delete_sq:
	case nvme_admin_ns_attach:
		goto complete;

	default:
		dev_err(&nvme->epf->dev,
			"Unhandled admin command 0x%02x\n",
			cmd->common.opcode);
		epcmd->status = NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
		goto complete;
	}

	if (transfer_len) {
		/*
		 * Set up the command buffer mapping: admin commands always use
		 * prp1 only.
		 */
		epcmd->map.size = transfer_len;
		epcmd->map.host_phys_addr = le64_to_cpu(cmd->common.dptr.prp1);
		epcmd->map.direction = PCI_EPF_NVME_TO_HOST;
	}

	pci_epf_nvme_exec_admin_cmd(nvme, epcmd);

complete:
	if (!(epcmd->flags & PCI_EPF_NVME_CMD_ASYNC))
		pci_epf_nvme_cmd_complete(epcmd);

	return true;
}

static bool pci_epf_nvme_process_io_cmd(struct pci_epf_nvme *nvme, int qid)
{
	struct pci_epf_nvme_cmd *epcmd;
	struct nvmet_req *req;
	struct nvme_command *cmd;
	int ret;

	/* Fetch new command from IO submission queue */
	epcmd = pci_epf_nvme_fetch_command(nvme, NULL, qid);
	if (!epcmd)
		return false;

	req = &epcmd->req;
	cmd = &epcmd->cmd;

	/* XXXX for now XXXX */
	epcmd->status = NVME_SC_SUCCESS;
	goto complete;

	switch (cmd->common.opcode) {
	case nvme_cmd_write:
	case nvme_cmd_read:
		ret = pci_epf_nvme_map_dptr(nvme, cmd);
		if (ret) {
			epcmd->status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
			goto complete;
		}
		break;

	case nvme_cmd_dsm:
		ret = pci_epf_nvme_map_prp1(nvme, cmd);
		if (ret) {
			epcmd->status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
			goto complete;
		}
		break;

	case nvme_cmd_flush:
	case nvme_cmd_write_zeroes:
		break;

	default:
		epcmd->status = NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
		goto complete;
	}

	/* XXXXXX BROKEN !!! FIX XXXXXXXXX */
	pci_epf_nvmet_execute_io_request(nvme, qid);
	if (epcmd->status)
		dev_err(&nvme->epf->dev,
			"Execute I/O command failed\n");

	pci_epf_nvme_unmap_dptr(nvme);

complete:
	pci_epf_nvme_cmd_complete(epcmd);

	return true;
}

static void pci_epf_nvme_reg_poll(struct work_struct *work)
{
	struct pci_epf_nvme *nvme = container_of(work, struct pci_epf_nvme,
						 reg_poll.work);
	struct pci_epf_nvme_ctrl *ctrl = &nvme->ctrl;
	bool did_work = true;
	int ret, qid;
	u32 old_cc;

	/* Check CC.EN to determine what we need to do */
	old_cc = ctrl->cc;
	ctrl->cc = pci_epf_nvme_reg_read32(ctrl, NVME_REG_CC);
	if (!(old_cc & NVME_CC_ENABLE) && !(ctrl->cc & NVME_CC_ENABLE)) {
		/* Not enabled yet: wait */
		goto again;
	}

	if (!(old_cc & NVME_CC_ENABLE) && (ctrl->cc & NVME_CC_ENABLE)) {
		/* CC.EN was set by the host: enbale the controller */
		ret = pci_epf_nvme_ctrl_enable(nvme);
		if (ret)
			pci_epf_nvme_ctrl_disable(nvme);
		goto again;
	}

	if ((old_cc & NVME_CC_ENABLE) && !(ctrl->cc & NVME_CC_ENABLE)) {
		/* CC.EN was cleared by the host: disable the controller */
		pci_epf_nvme_ctrl_disable(nvme);
		goto again;
	}

	/* Enabled: process pending commands */
	do {
		did_work = pci_epf_nvme_process_admin_cmd(nvme);
		for (qid = 1; qid <= PCI_EPF_NVME_MAX_QID; qid++)
			did_work |= pci_epf_nvme_process_io_cmd(nvme, qid);

	} while (did_work);

again:
	pci_epf_nvme_target_keep_alive(nvme);

	queue_delayed_work(epf_nvme_workqueue, &nvme->reg_poll,
			   msecs_to_jiffies(1));
}

static int pci_epf_nvme_set_bars(struct pci_epf_nvme *nvme)
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
		epf_bar = &epf->bar[bar];

		/*
		 * pci_epc_set_bar() sets PCI_BASE_ADDRESS_MEM_TYPE_64
		 * if the specific implementation requires a 64-bit BAR,
		 * even if we only requested a 32-bit BAR.
		 */
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

static int pci_epf_nvme_alloc_reg_bar(struct pci_epf_nvme *nvme)
{
	const struct pci_epc_features *features = nvme->epc_features;
	size_t align = max_t(size_t, features->align, 4096);
	enum pci_barno reg_bar = nvme->reg_bar;
	struct pci_epf *epf = nvme->epf;
	size_t reg_size, reg_bar_size;
	size_t msix_table_size = 0;

	/*
	 * Calculate the size of the register bar: registers first with enough
	 * space for the doorbells, followed by the MSIX table if supported.
	 */
	reg_size = NVME_REG_DBS + ((PCI_EPF_NVME_MAX_QID * 2 + 1) * sizeof(u32));
	reg_size = ALIGN(reg_size, 8);

	if (features->msix_capable) {
		size_t pba_size;

		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		nvme->msix_table_offset = reg_size;
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);

		reg_size += msix_table_size + pba_size;
	}

	reg_bar_size = ALIGN(reg_size, 4096);

	if (features->bar_fixed_size[reg_bar]) {
		if (reg_bar_size > features->bar_fixed_size[reg_bar]) {
			dev_err(&epf->dev,
				"Reg BAR %d size %llu B too small, need %zu B\n",
				reg_bar,
				features->bar_fixed_size[reg_bar],
				reg_bar_size);
			return -ENOMEM;
		}
		reg_bar_size = features->bar_fixed_size[reg_bar];
	}

	nvme->reg[reg_bar] = pci_epf_alloc_space(epf, reg_bar_size, reg_bar,
						 align, PRIMARY_INTERFACE);
	if (!nvme->reg[reg_bar]) {
		dev_err(&epf->dev, "Allocate register BAR failed\n");
		return -ENOMEM;
	}
	memset(nvme->reg[reg_bar], 0, reg_bar_size);

	dev_dbg(&epf->dev,
		"Allocated BAR %d, virt addr 0x%p, phys addr 0x%llx, %zu B\n",
		reg_bar, nvme->reg[reg_bar], epf->bar[reg_bar].phys_addr,
		epf->bar[reg_bar].size);

	return 0;
}

static int pci_epf_nvme_configure_bars(struct pci_epf_nvme *nvme)
{
	const struct pci_epc_features *features = nvme->epc_features;
	size_t bar_size, align = max_t(size_t, features->align, 4096);
	struct pci_epf *epf = nvme->epf;
	struct pci_epf_bar *epf_bar;
	int bar, add, ret;

	/* The first free BAR will be our register BAR */
	bar = pci_epc_get_first_free_bar(features);
	if (bar < 0) {
		dev_err(&epf->dev, "No free BAR\n");
		return -EINVAL;
	}
	nvme->reg_bar = bar;

	/* Initialize BAR flags */
	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar++) {
		epf_bar = &epf->bar[bar];
		if (features->bar_fixed_64bit & (1 << bar))
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
	}

	/* Allocate the register BAR */
	ret = pci_epf_nvme_alloc_reg_bar(nvme);
	if (ret)
		return ret;

	/* Allocate remaining BARs */
	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		if (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
			add = 2;
		else
			add = 1;

		/*
		 * Skip the register BAR (already allocated) and
		 * reserved BARs.
		 */
		if (nvme->reg[bar] || features->reserved_bar & (1 << bar))
			continue;

		bar_size = max_t(size_t, features->bar_fixed_size[bar], SZ_4K);
		nvme->reg[bar] = pci_epf_alloc_space(epf, bar_size, bar,
						     align, PRIMARY_INTERFACE);
		if (!nvme->reg[bar]) {
			dev_err(&epf->dev, "Allocate BAR%d failed\n", bar);
			return -ENOMEM;
		}

		memset(nvme->reg[bar], 0, bar_size);

		dev_dbg(&nvme->epf->dev,
			"Allocated BAR %d, virt addr 0x%p, phys addr 0x%llx, %zu B\n",
			bar, nvme->reg[bar], epf->bar[bar].phys_addr,
			epf->bar[bar].size);
	}

	return 0;
}

struct pci_epf_nvme_dma_filter {
        struct device *dev;
        u32 dma_mask;
};

static bool pci_epf_nvme_dma_filter_fn(struct dma_chan *chan, void *node)
{
        struct pci_epf_nvme_dma_filter *filter = node;
        struct dma_slave_caps caps;

        memset(&caps, 0, sizeof(caps));
        dma_get_slave_caps(chan, &caps);

        return chan->device->dev == filter->dev
                && (filter->dma_mask & caps.directions);
}

static bool pci_epf_nvme_init_dma(struct pci_epf_nvme *nvme)
{
	struct pci_epf *epf = nvme->epf;
	struct device *dev = &epf->dev;
	struct pci_epf_nvme_dma_filter filter;
	struct dma_chan *dma_chan;
	dma_cap_mask_t mask;
	int ret;

	filter.dev = epf->epc->dev.parent;
	filter.dma_mask = BIT(DMA_DEV_TO_MEM);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_chan = dma_request_channel(mask, pci_epf_nvme_dma_filter_fn,
				       &filter);
	if (!dma_chan) {
		dev_info(dev,
			 "Failed to get private DMA rx channel. Falling back to generic one\n");
		goto fail_back_tx;
	}

	nvme->dma_chan_rx = dma_chan;
	filter.dma_mask = BIT(DMA_MEM_TO_DEV);
	dma_chan = dma_request_channel(mask, pci_epf_nvme_dma_filter_fn,
				       &filter);
	if (!dma_chan) {
		dev_info(dev,
			 "Failed to get private DMA tx channel. Falling back to generic one\n");
		goto fail_back_rx;
	}

	nvme->dma_chan_tx = dma_chan;
	nvme->dma_private = true;

	init_completion(&nvme->dma_complete);

	return true;

fail_back_rx:
	dma_release_channel(nvme->dma_chan_rx);
	nvme->dma_chan_tx = NULL;

fail_back_tx:
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	dma_chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(dma_chan)) {
		ret = PTR_ERR(dma_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get DMA channel\n");
		return false;
	}
	init_completion(&nvme->dma_complete);

	nvme->dma_chan_tx = dma_chan;
	nvme->dma_chan_rx = dma_chan;

	return true;
}

static void pci_epf_nvme_clean_dma(struct pci_epf_nvme *nvme)
{
	if (!nvme->dma_supported)
		return;

	dma_release_channel(nvme->dma_chan_tx);
	if (nvme->dma_chan_rx != nvme->dma_chan_tx)
		dma_release_channel(nvme->dma_chan_rx);

	nvme->dma_chan_tx = NULL;
	nvme->dma_chan_rx = NULL;
	nvme->dma_chan = NULL;
	nvme->dma_supported = false;
}

static int pci_epf_nvme_enable_msi(struct pci_epf_nvme *nvme)
{
	struct pci_epf *epf = nvme->epf;
	struct pci_epc *epc = epf->epc;
	int ret;

	/* Enable MSIX if supported. If not, we must have MSI */
	if (nvme->epc_features->msix_capable) {
		dev_dbg(&epf->dev, "MSIX capable, %d vectors\n",
			epf->msix_interrupts);
		ret = pci_epc_set_msix(epc, epf->func_no, epf->vfunc_no,
				       epf->msix_interrupts,
				       nvme->reg_bar,
				       nvme->msix_table_offset);
		if (ret) {
			dev_err(&epf->dev,
				"MSI-X configuration failed, trying MSI\n");
			return ret;
		}

		nvme->nr_vectors = epf->msix_interrupts;

		return 0;
	}

	if (nvme->epc_features->msi_capable) {
		dev_dbg(&epf->dev, "MSI capable, %d vectors\n",
			epf->msi_interrupts);
		ret = pci_epc_set_msi(epc, epf->func_no, epf->vfunc_no,
			      	epf->msi_interrupts);
		if (ret) {
			dev_err(&epf->dev, "MSI configuration failed\n");
			return ret;
		}

		nvme->nr_vectors = epf->msi_interrupts;

		return 0;
	}

	/* Legacy INTX */
	nvme->nr_vectors = 1;

	return 0;
}

static int pci_epf_nvme_core_init(struct pci_epf_nvme *nvme)
{
	struct pci_epf *epf = nvme->epf;
	struct pci_epc *epc = epf->epc;
	int ret;

	dev_dbg(&epf->dev, "%s\n", __FUNCTION__);

	if (epf->vfunc_no <= 1) {
		/* Set device ID, class, etc */
		ret = pci_epc_write_header(epc, epf->func_no, epf->vfunc_no,
					   epf->header);
		if (ret) {
			dev_err(&epf->dev,
				"Write configuration header failed %d\n", ret);
			return ret;
		}
	}

	/* Allocate PCIe BARs and enable MSI[X] interrupts */
	ret = pci_epf_nvme_set_bars(nvme);
	if (ret)
		return ret;

	ret = pci_epf_nvme_enable_msi(nvme);
	if (ret)
		return ret;

	/* Now initialize our local controller registers for the host to see */
	pci_epf_nvme_ctrl_init(nvme);

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
		queue_delayed_work(epf_nvme_workqueue, &nvme->reg_poll,
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
	int ret;

	if (!epc) {
		dev_err(&epf->dev, "No endpoint controller\n");
		return -EINVAL;
	}

	epc_features = pci_epc_get_features(epc, epf->func_no, epf->vfunc_no);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}
	nvme->epc_features = epc_features;

	/* We must have a backend device specified for the target */
	if (!strlen(nvme->ns_device_path)) {
		dev_err(&epf->dev, "Namespace device path is not set\n");
		return -EINVAL;
	}

	dev_dbg(&epf->dev, "Bind: backend %s\n", nvme->ns_device_path);

	ret = pci_epf_nvme_configure_bars(nvme);
	if (ret)
		return ret;

	if (!epc_features->core_init_notifier) {
		ret = pci_epf_nvme_core_init(nvme);
		if (ret)
			return ret;
	}

	nvme->dma_supported = pci_epf_nvme_init_dma(nvme);
	dev_dbg(&epf->dev, "DMA %ssupported\n",
		nvme->dma_supported ? "" : "not ");

	if (epc_features->linkup_notifier || epc_features->core_init_notifier) {
		epf->nb.notifier_call = pci_epf_nvme_notifier;
		pci_epc_register_notifier(epc, &epf->nb);
	} else {
		queue_work(epf_nvme_workqueue, &nvme->reg_poll.work);
	}

	return 0;
}

static void pci_epf_nvme_unbind(struct pci_epf *epf)
{
	struct pci_epf_nvme *nvme = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	int bar;

	dev_dbg(&nvme->epf->dev, "%s\n", __FUNCTION__);

	cancel_delayed_work(&nvme->reg_poll);

	pci_epf_nvme_clean_dma(nvme);

	pci_epf_nvme_ctrl_disable(nvme);

	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar++) {
		if (!nvme->reg[bar])
			continue;
		pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no,
				  &epf->bar[bar]);
		pci_epf_free_space(epf, nvme->reg[bar], bar, PRIMARY_INTERFACE);
	}
}

static struct pci_epf_header epf_nvme_pci_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.progif_code	= 0x02, /* NVM Express */
	.baseclass_code = PCI_BASE_CLASS_STORAGE,
	.subclass_code	= 0x08, /* Non-Volatile Memory controller */
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
	spin_lock_init(&nvme->qlock);
	INIT_DELAYED_WORK(&nvme->reg_poll, pci_epf_nvme_reg_poll);

	/* Set default attribute values */
	strncpy(nvme->hostnqn, PCI_EPF_NVME_HOSTNQN,
		sizeof(nvme->hostnqn) - 1);
	strncpy(nvme->subsysnqn, PCI_EPF_NVME_SUBSYSNQN,
		sizeof(nvme->subsysnqn) - 1);
	strncpy(nvme->model_number, PCI_EPF_NVME_MODEL_NUMBER,
		sizeof(nvme->model_number) - 1);
	strncpy(nvme->firmware_rev, PCI_EPF_NVME_FW_REV,
		sizeof(nvme->firmware_rev) - 1);
	strncpy(nvme->serial, PCI_EPF_NVME_SERIAL,
		sizeof(nvme->serial) - 1);

	epf->header = &epf_nvme_pci_header;
	epf_set_drvdata(epf, nvme);

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
	return sysfs_emit(page, "%s\n", nvme->_name);			\
}

#define PCI_EPF_NVME_W(_name)						\
static ssize_t pci_epf_nvme_##_name##_store(struct config_item *item,	\
					const char *page, size_t len)	\
{									\
	struct config_group *group = to_config_group(item);		\
	struct pci_epf_nvme *nvme = to_epf_nvme(group);			\
	size_t slen = strlen(page);					\
									\
	if (!slen || slen > sizeof(nvme->_name) - 1)			\
		return -EINVAL;						\
									\
	strncpy( nvme->_name, page, slen);				\
									\
	return slen;							\
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

PCI_EPF_NVME_R(model_number)
PCI_EPF_NVME_W(model_number)
CONFIGFS_ATTR(pci_epf_nvme_, model_number);

PCI_EPF_NVME_R(serial)
PCI_EPF_NVME_W(serial)
CONFIGFS_ATTR(pci_epf_nvme_, serial);

PCI_EPF_NVME_R(firmware_rev)
PCI_EPF_NVME_W(firmware_rev)
CONFIGFS_ATTR(pci_epf_nvme_, firmware_rev);

static struct configfs_attribute *pci_epf_nvme_attrs[] = {
	&pci_epf_nvme_attr_hostnqn,
	&pci_epf_nvme_attr_subsysnqn,
	&pci_epf_nvme_attr_ns_device_path,
	&pci_epf_nvme_attr_model_number,
	&pci_epf_nvme_attr_serial,
	&pci_epf_nvme_attr_firmware_rev,
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

static const struct pci_epf_device_id pci_epf_nvme_ids[] = {
	{ .name = "pci_epf_nvme" },
	{},
};

static struct pci_epf_ops pci_epf_nvme_ops = {
	.bind	= pci_epf_nvme_bind,
	.unbind	= pci_epf_nvme_unbind,
	.add_cfs = pci_epf_nvme_add_cfs,
};

static struct pci_epf_driver epf_nvme_driver = {
	.driver.name	= "pci_epf_nvme",
	.probe		= pci_epf_nvme_probe,
	.id_table	= pci_epf_nvme_ids,
	.ops		= &pci_epf_nvme_ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_nvme_init(void)
{
	int ret;

	pr_info("Registering driver\n");

	epf_nvme_workqueue = alloc_workqueue("epf_nvme",
					     WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!epf_nvme_workqueue)
		return -ENOMEM;

	epf_nvme_cmd_cache = kmem_cache_create("epf_nvme_cmd",
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
