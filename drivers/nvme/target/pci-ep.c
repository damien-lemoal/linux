// SPDX-License-Identifier: GPL-2.0
/*
 * NVMe PCI endpoint device.
 * Copyright (c) 2024, Western Digital Corporation or its affiliates.
 * Copyright (c) 2024, Rick Wertenbroek <rick.wertenbroek@gmail.com>
 *                     REDS Institute, HEIG-VD, HES-SO, Switzerland
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/mempool.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvme.h>
#include <linux/pci_ids.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>
#include <linux/slab.h>

#include "nvmet.h"

static LIST_HEAD(nvmet_pciep_ports);
static DEFINE_MUTEX(nvmet_pciep_ports_mutex);

/*
 * Default and maximum allowed data transfer size. For the default,
 * allow up to 128 page-sized segments. For the maximum allowed,
 * use 4 times the default (which is completely arbitrary).
 */
#define NVMET_PCIEP_MAX_SEGS	128
#define NVMET_PCIEP_MDTS_KB	(NVMET_PCIEP_MAX_SEGS << (PAGE_SHIFT - 10))
#define NVMET_PCIEP_MAX_MDTS_KB	(NVMET_PCIEP_MDTS_KB * 4)

/*
 * IRQ vector coalescing threshold: by default, post 8 CQEs before raising an
 * interrupt vector to the host. This default 8 is completely arbitrary and can
 * be changed by the host with a nvme_set_features command.
 */
#define NVMET_PCIEP_IV_THRESHOLD	8

/*
 * BAR CC register and SQ polling intervals.
 */
#define NVMET_PCIEP_CC_POLL_INTERVAL	msecs_to_jiffies(5)
#define NVMET_PCIEP_SQ_POLL_INTERVAL	msecs_to_jiffies(5)
#define NVMET_PCIEP_SQ_POLL_IDLE	msecs_to_jiffies(5000)

/*
 * SQ arbitration burst default: fetch at most 8 commands at a time from an SQ.
 */
#define NVMET_PCIE_SQ_AB		8

/*
 * Handling of CQs is normally immediate, unless we fail to map a CQ or the CQ
 * is full, in which case we retry the CQ processing after this interval.
 */
#define NVMET_PCIEP_CQ_RETRY_INTERVAL	msecs_to_jiffies(1)

enum nvmet_pciep_queue_flags {
	/* The queue is a submission queue */
	NVMET_PCIEP_Q_IS_SQ	= 0,
	/* The queue is live */
	NVMET_PCIEP_Q_LIVE,
	/* IRQ is enabled for this queue */
	NVMET_PCIEP_Q_IRQ_ENABLED,
};

/*
 * IRQ vector descriptor.
 */
struct nvmet_pciep_irq_vector {
	unsigned int	vector;
	unsigned int	ref;
	bool		cd;
	int		nr_irqs;
};

struct nvmet_pciep_queue {
	union {
		struct nvmet_sq	nvme_sq;
		struct nvmet_cq	nvme_cq;
	};
	struct nvmet_pciep_ctrl	*ctrl;
	unsigned long		flags;

	u64			pci_addr;
	size_t			pci_size;
	struct pci_epc_map	pci_map;

	u16			qid;
	u16			depth;
	u16			vector;
	u16			head;
	u16			tail;
	u16			phase;
	u32			db;

	size_t			qes;

	struct nvmet_pciep_irq_vector *iv;
	struct workqueue_struct	*iod_wq;
	struct delayed_work	work;
	spinlock_t		lock;
	struct list_head	list;
	atomic_t		list_count;
};

/*
 * PCI memory segment for mapping an admin or IO command buffer to PCI space.
 */
struct nvmet_pciep_segment {
	void			*buf;
	u64			pci_addr;
	u32			length;
};

/*
 * Command descriptors.
 */
struct nvmet_pciep_iod {
	struct list_head		link;

	struct nvmet_req		req;
	struct nvme_command		cmd;
	struct nvme_completion		cqe;
	unsigned int			status;

	struct nvmet_pciep_ctrl		*ctrl;

	struct nvmet_pciep_queue	*sq;
	struct nvmet_pciep_queue	*cq;

	/* Data transfer size and direction for the command. */
	size_t				data_len;
	enum dma_data_direction		dma_dir;

	/*
	 * RC PCI address data segments: if nr_data_segs is 1, we use only
	 * @data_seg. Otherwise, the array of segments @data_segs is allocated
	 * to manage multiple PCI address data segments. @data_sgl and @data_sgt
	 * are used to setup the command request for execution by the target
	 * core.
	 */
	unsigned int			nr_data_segs;
	struct nvmet_pciep_segment	data_seg;
	struct nvmet_pciep_segment	*data_segs;
	struct scatterlist		data_sgl;
	struct sg_table			data_sgt;

	struct work_struct		work;
	struct completion		done;
};

/*
 * PCI target controller private data.
 */
struct nvmet_pciep_ctrl {
	struct nvmet_pciep_epf		*nvme_epf;
	struct nvmet_port		*port;
	struct nvmet_ctrl		*tctrl;
	struct device			*dev;

	unsigned int			nr_queues;
	struct nvmet_pciep_queue	*sq;
	struct nvmet_pciep_queue	*cq;
	unsigned int			sq_ab;

	mempool_t			iod_pool;
	void				*bar;
	u64				cap;
	u32				cc;
	u32				csts;

	size_t				io_sqes;
	size_t				io_cqes;

	size_t				mps_shift;
	size_t				mps;
	size_t				mps_mask;

	unsigned int			mdts;

	struct delayed_work		poll_cc;
	struct delayed_work		poll_sqs;

	struct mutex			irq_lock;
	struct nvmet_pciep_irq_vector	*irq_vectors;
	unsigned int			irq_vector_threshold;

	bool				link_up;
	bool				enabled;
};

/*
 * PCI EPF driver private data.
 */
struct nvmet_pciep_epf {
	struct pci_epf			*epf;

	const struct pci_epc_features	*epc_features;

	void				*reg_bar;
	size_t				msix_table_offset;

	unsigned int			irq_type;
	unsigned int			nr_vectors;

	struct nvmet_pciep_ctrl		ctrl;

	struct dma_chan			*dma_tx_chan;
	struct mutex			dma_tx_lock;
	struct dma_chan			*dma_rx_chan;
	struct mutex			dma_rx_lock;

	struct mutex			mmio_lock;

	/* PCI endpoint function configfs attributes */
	struct config_group		group;
	bool				dma_enable;
	__le16				portid;
	char				subsysnqn[NVMF_NQN_SIZE];
	unsigned int			mdts_kb;
};

static inline u32 nvmet_pciep_bar_read32(struct nvmet_pciep_ctrl *ctrl, u32 off)
{
	__le32 *bar_reg = ctrl->bar + off;

	return le32_to_cpu(READ_ONCE(*bar_reg));
}

static inline void nvmet_pciep_bar_write32(struct nvmet_pciep_ctrl *ctrl,
					   u32 off, u32 val)
{
	__le32 *bar_reg = ctrl->bar + off;

	WRITE_ONCE(*bar_reg, cpu_to_le32(val));
}

static inline u64 nvmet_pciep_bar_read64(struct nvmet_pciep_ctrl *ctrl, u32 off)
{
	return (u64)nvmet_pciep_bar_read32(ctrl, off ) |
		((u64)nvmet_pciep_bar_read32(ctrl, off + 4) << 32);
}

static inline void nvmet_pciep_bar_write64(struct nvmet_pciep_ctrl *ctrl,
					   u32 off, u64 val)
{
	nvmet_pciep_bar_write32(ctrl, off , val & 0xFFFFFFFF);
	nvmet_pciep_bar_write32(ctrl, off + 4, (val >> 32) & 0xFFFFFFFF);
}

static inline int nvmet_pciep_epf_mem_map(struct nvmet_pciep_epf *nvme_epf,
					  u64 pci_addr, size_t size,
					  struct pci_epc_map *map)
{
	struct pci_epf *epf = nvme_epf->epf;

	return pci_epc_mem_map(epf->epc, epf->func_no, epf->vfunc_no,
			       pci_addr, size, map);
}

static inline void nvmet_pciep_epf_mem_unmap(struct nvmet_pciep_epf *nvme_epf,
					     struct pci_epc_map *map)
{
	struct pci_epf *epf = nvme_epf->epf;

	pci_epc_mem_unmap(epf->epc, epf->func_no, epf->vfunc_no, map);
}

struct nvmet_pciep_epf_dma_filter {
	struct device *dev;
	u32 dma_mask;
};

static bool nvmet_pciep_epf_dma_filter(struct dma_chan *chan, void *arg)
{
	struct nvmet_pciep_epf_dma_filter *filter = arg;
	struct dma_slave_caps caps;

	memset(&caps, 0, sizeof(caps));
	dma_get_slave_caps(chan, &caps);

	return chan->device->dev == filter->dev &&
		(filter->dma_mask & caps.directions);
}

static bool nvmet_pciep_epf_init_dma(struct nvmet_pciep_epf *nvme_epf)
{
	struct pci_epf *epf = nvme_epf->epf;
	struct device *dev = &epf->dev;
	struct nvmet_pciep_epf_dma_filter filter;
	struct dma_chan *chan;
	dma_cap_mask_t mask;

	mutex_init(&nvme_epf->dma_rx_lock);
	mutex_init(&nvme_epf->dma_tx_lock);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	filter.dev = epf->epc->dev.parent;
	filter.dma_mask = BIT(DMA_DEV_TO_MEM);

	chan = dma_request_channel(mask, nvmet_pciep_epf_dma_filter, &filter);
	if (!chan)
		return false;

	nvme_epf->dma_rx_chan = chan;

	dev_dbg(dev, "Using DMA RX channel %s, maximum segment size %u B\n",
		dma_chan_name(chan),
		dma_get_max_seg_size(dmaengine_get_dma_device(chan)));

	filter.dma_mask = BIT(DMA_MEM_TO_DEV);
	chan = dma_request_channel(mask, nvmet_pciep_epf_dma_filter, &filter);
	if (!chan) {
		dma_release_channel(nvme_epf->dma_rx_chan);
		nvme_epf->dma_rx_chan = NULL;
		return false;
	}

	nvme_epf->dma_tx_chan = chan;

	dev_dbg(dev, "Using DMA TX channel %s, maximum segment size %u B\n",
		dma_chan_name(chan),
		dma_get_max_seg_size(dmaengine_get_dma_device(chan)));

	return true;
}

static void nvmet_pciep_epf_deinit_dma(struct nvmet_pciep_epf *nvme_epf)
{
	if (nvme_epf->dma_tx_chan) {
		dma_release_channel(nvme_epf->dma_tx_chan);
		nvme_epf->dma_tx_chan = NULL;
	}

	if (nvme_epf->dma_rx_chan) {
		dma_release_channel(nvme_epf->dma_rx_chan);
		nvme_epf->dma_rx_chan = NULL;
	}

	mutex_destroy(&nvme_epf->dma_rx_lock);
	mutex_destroy(&nvme_epf->dma_tx_lock);
}

static int nvmet_pciep_epf_dma_transfer(struct nvmet_pciep_epf *nvme_epf,
		struct nvmet_pciep_segment *seg, enum dma_data_direction dir)
{
	struct pci_epf *epf = nvme_epf->epf;
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config sconf = {};
	struct device *dev = &epf->dev;
	struct device *dma_dev;
	struct dma_chan *chan;
	dma_cookie_t cookie;
	dma_addr_t dma_addr;
	struct mutex *lock;
	int ret;

	switch (dir) {
	case DMA_FROM_DEVICE:
		lock = &nvme_epf->dma_rx_lock;
		chan = nvme_epf->dma_rx_chan;
		sconf.direction = DMA_DEV_TO_MEM;
		sconf.src_addr = seg->pci_addr;
		break;
	case DMA_TO_DEVICE:
		lock = &nvme_epf->dma_tx_lock;
		chan = nvme_epf->dma_tx_chan;
		sconf.direction = DMA_MEM_TO_DEV;
		sconf.dst_addr = seg->pci_addr;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(lock);

	dma_dev = dmaengine_get_dma_device(chan);
	dma_addr = dma_map_single(dma_dev, seg->buf, seg->length, dir);
	ret = dma_mapping_error(dma_dev, dma_addr);
	if (ret)
		goto unlock;

	ret = dmaengine_slave_config(chan, &sconf);
	if (ret) {
		dev_err(dev, "Failed to configure DMA channel\n");
		goto unmap;
	}

	desc = dmaengine_prep_slave_single(chan, dma_addr, seg->length,
					   sconf.direction, DMA_CTRL_ACK);
	if (!desc) {
		dev_err(dev, "Failed to prepare DMA\n");
		ret = -EIO;
		goto unmap;
	}

	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(dev, "DMA submit failed %d\n", ret);
		goto unmap;
	}

	if (dma_sync_wait(chan, cookie) != DMA_COMPLETE) {
		dev_err(dev, "DMA transfer failed\n");
		ret = -EIO;
	}

	dmaengine_terminate_sync(chan);

unmap:
	dma_unmap_single(dma_dev, dma_addr, seg->length, dir);

unlock:
	mutex_unlock(lock);

	return ret;
}

static int nvmet_pciep_epf_mmio_transfer(struct nvmet_pciep_epf *nvme_epf,
		struct nvmet_pciep_segment *seg, enum dma_data_direction dir)
{
	u64 pci_addr = seg->pci_addr;
	u32 length = seg->length;
	void *buf = seg->buf;
	struct pci_epc_map map;
	int ret = -EINVAL;

	/*
	 * Note: mmio transfers do not need serialization but this is a
	 * simple way to avoid using too many mapping windows.
	 */
	mutex_lock(&nvme_epf->mmio_lock);

	while (length) {
		ret = nvmet_pciep_epf_mem_map(nvme_epf, pci_addr, length, &map);
		if (ret)
			break;

		switch (dir) {
		case DMA_FROM_DEVICE:
			memcpy_fromio(buf, map.virt_addr, map.pci_size);
			break;
		case DMA_TO_DEVICE:
			memcpy_toio(map.virt_addr, buf, map.pci_size);
			break;
		default:
			ret = -EINVAL;
			goto unlock;
		}

		pci_addr += map.pci_size;
		buf += map.pci_size;
		length -= map.pci_size;

		nvmet_pciep_epf_mem_unmap(nvme_epf, &map);
	}

unlock:
	mutex_unlock(&nvme_epf->mmio_lock);

	return ret;
}

static inline int nvmet_pciep_epf_transfer(struct nvmet_pciep_epf *nvme_epf,
		struct nvmet_pciep_segment *seg, enum dma_data_direction dir)
{
	if (nvme_epf->dma_enable)
		return nvmet_pciep_epf_dma_transfer(nvme_epf, seg, dir);

	return nvmet_pciep_epf_mmio_transfer(nvme_epf, seg, dir);
}

static inline int nvmet_pciep_transfer(struct nvmet_pciep_ctrl *ctrl,
		void *buf, u64 pci_addr, u32 length, enum dma_data_direction dir)
{
	struct nvmet_pciep_segment seg = {
		.buf = buf,
		.pci_addr = pci_addr,
		.length = length,
	};

	return nvmet_pciep_epf_transfer(ctrl->nvme_epf, &seg, dir);
}

static int nvmet_pciep_alloc_irq_vectors(struct nvmet_pciep_ctrl *ctrl)
{
	ctrl->irq_vectors = kcalloc(ctrl->nr_queues,
				    sizeof(struct nvmet_pciep_irq_vector),
				    GFP_KERNEL);
	if (!ctrl->irq_vectors)
		return -ENOMEM;

	mutex_init(&ctrl->irq_lock);

	return 0;
}

static void nvmet_pciep_free_irq_vectors(struct nvmet_pciep_ctrl *ctrl)
{
	if (ctrl->irq_vectors) {
		kfree(ctrl->irq_vectors);
		ctrl->irq_vectors = NULL;
		mutex_destroy(&ctrl->irq_lock);
	}
}

static struct nvmet_pciep_irq_vector *
nvmet_pciep_find_irq_vector(struct nvmet_pciep_ctrl *ctrl, u16 vector)
{
	struct nvmet_pciep_irq_vector *iv;
	int i;

	lockdep_assert_held(&ctrl->irq_lock);

	for (i = 0; i < ctrl->nr_queues; i++) {
		iv = &ctrl->irq_vectors[i];
		if (iv->ref && iv->vector == vector)
			return iv;
	}

	return NULL;
}

static struct nvmet_pciep_irq_vector *
nvmet_pciep_add_irq_vector(struct nvmet_pciep_ctrl *ctrl, u16 vector)
{
	struct nvmet_pciep_irq_vector *iv;
	int i;

	mutex_lock(&ctrl->irq_lock);

	iv = nvmet_pciep_find_irq_vector(ctrl, vector);
	if (iv) {
		iv->ref++;
		goto unlock;
	}

	for (i = 0; i < ctrl->nr_queues; i++) {
		iv = &ctrl->irq_vectors[i];
		if (!iv->ref)
			break;
	}

	if (WARN_ON_ONCE(!iv))
		goto unlock;

	iv->ref = 1;
	iv->vector = vector;
	iv->nr_irqs = 0;

unlock:
	mutex_unlock(&ctrl->irq_lock);

	return iv;
}

static void nvmet_pciep_remove_irq_vector(struct nvmet_pciep_ctrl *ctrl,
					  u16 vector)
{
	struct nvmet_pciep_irq_vector *iv;

	mutex_lock(&ctrl->irq_lock);

	iv = nvmet_pciep_find_irq_vector(ctrl, vector);
	if (iv) {
		iv->ref--;
		if (!iv->ref) {
			iv->vector = 0;
			iv->nr_irqs = 0;
		}
	}

	mutex_unlock(&ctrl->irq_lock);
}

static bool nvmet_pciep_should_raise_irq(struct nvmet_pciep_ctrl *ctrl,
					 struct nvmet_pciep_queue *cq,
					 bool force)
{
	struct nvmet_pciep_irq_vector *iv = cq->iv;
	bool ret;

	if (!test_bit(NVMET_PCIEP_Q_IRQ_ENABLED, &cq->flags))
		return false;

	/* IRQ coalescing for the admin queue is not allowed. */
	if (!cq->qid)
		return true;

	if (iv->cd)
		return true;

	if (force) {
		ret = iv->nr_irqs > 0;
	} else {
		iv->nr_irqs++;
		ret = iv->nr_irqs >= ctrl->irq_vector_threshold;
	}
	if (ret)
		iv->nr_irqs = 0;

	return ret;
}

static void nvmet_pciep_raise_irq(struct nvmet_pciep_ctrl *ctrl,
				  struct nvmet_pciep_queue *cq, bool force)
{
	struct nvmet_pciep_epf *nvme_epf = ctrl->nvme_epf;
	struct pci_epf *epf = nvme_epf->epf;
	int ret = 0;

	if (!test_bit(NVMET_PCIEP_Q_LIVE, &cq->flags))
		return;

	mutex_lock(&ctrl->irq_lock);

	if (!nvmet_pciep_should_raise_irq(ctrl, cq, force))
		goto unlock;

	switch (nvme_epf->irq_type) {
	case PCI_IRQ_MSIX:
	case PCI_IRQ_MSI:
		ret = pci_epc_raise_irq(epf->epc, epf->func_no, epf->vfunc_no,
					nvme_epf->irq_type, cq->vector + 1);
		if (!ret)
			break;
		/*
		 * If we got an error, it is likely because the host is using
		 * legacy IRQs (e.g. BIOS, grub).
		 */
		fallthrough;
	case PCI_IRQ_INTX:
		ret = pci_epc_raise_irq(epf->epc, epf->func_no, epf->vfunc_no,
					PCI_IRQ_INTX, 0);
		break;
	default:
		WARN_ON_ONCE(1);
		ret = -EINVAL;
		break;
	}

	if (ret)
		dev_err(ctrl->dev, "Raise IRQ failed %d\n", ret);

unlock:
	mutex_unlock(&ctrl->irq_lock);
}

static inline const char *nvmet_pciep_iod_name(struct nvmet_pciep_iod *iod)
{
	return nvme_opcode_str(iod->sq->qid, iod->cmd.common.opcode);
}

static void nvmet_pciep_exec_iod_work(struct work_struct *work);

static struct nvmet_pciep_iod *
nvmet_pciep_alloc_iod(struct nvmet_pciep_queue *sq)
{
	struct nvmet_pciep_ctrl *ctrl = sq->ctrl;
	struct nvmet_pciep_iod *iod;

	iod = mempool_alloc(&ctrl->iod_pool, GFP_KERNEL);
	if (unlikely(!iod))
		return NULL;

	memset(iod, 0, sizeof(*iod));
	iod->req.cmd = &iod->cmd;
	iod->req.cqe = &iod->cqe;
	iod->req.port = ctrl->port;
	iod->ctrl = ctrl;
	iod->sq = sq;
	iod->cq = &ctrl->cq[sq->qid];
	INIT_LIST_HEAD(&iod->link);
	iod->dma_dir = DMA_NONE;
	INIT_WORK(&iod->work, nvmet_pciep_exec_iod_work);
	init_completion(&iod->done);

	return iod;
}

/*
 * Allocate or grow a command table of PCI segments.
 */
static int nvmet_pciep_alloc_iod_data_segs(struct nvmet_pciep_iod *iod,
					   int nsegs)
{
	struct nvmet_pciep_segment *segs;
	int nr_segs = iod->nr_data_segs + nsegs;

	segs = krealloc(iod->data_segs,
			nr_segs * sizeof(struct nvmet_pciep_segment),
			GFP_KERNEL | __GFP_ZERO);
	if (!segs)
		return -ENOMEM;

	iod->nr_data_segs = nr_segs;
	iod->data_segs = segs;

	return 0;
}

static void nvmet_pciep_free_iod(struct nvmet_pciep_iod *iod)
{
	int i;

	if (iod->data_segs) {
		for (i = 0; i < iod->nr_data_segs; i++)
			kfree(iod->data_segs[i].buf);
		if (iod->data_segs != &iod->data_seg)
			kfree(iod->data_segs);
	}
	if (iod->data_sgt.nents > 1)
		sg_free_table(&iod->data_sgt);
	mempool_free(iod, &iod->ctrl->iod_pool);
}

static int nvmet_pciep_transfer_iod_data(struct nvmet_pciep_iod *iod)
{
	struct nvmet_pciep_epf *nvme_epf = iod->ctrl->nvme_epf;
	struct nvmet_pciep_segment *seg = &iod->data_segs[0];
	int i, ret;

	/* Split the data transfer according to the PCI segments. */
	for (i = 0; i < iod->nr_data_segs; i++, seg++) {
		ret = nvmet_pciep_epf_transfer(nvme_epf, seg, iod->dma_dir);
		if (ret) {
			iod->status = NVME_SC_DATA_XFER_ERROR | NVME_STATUS_DNR;
			return ret;
		}
	}

	return 0;
}

static inline u64 nvmet_pciep_prp_addr(struct nvmet_pciep_ctrl *ctrl, u64 prp)
{
	return prp & ~ctrl->mps_mask;
}

static inline u32 nvmet_pciep_prp_ofst(struct nvmet_pciep_ctrl *ctrl, u64 prp)
{
	return prp & ctrl->mps_mask;
}

static inline size_t nvmet_pciep_prp_size(struct nvmet_pciep_ctrl *ctrl, u64 prp)
{
	return ctrl->mps - nvmet_pciep_prp_ofst(ctrl, prp);
}

/*
 * Transfer a prp list from the host and return the number of prps.
 */
static int nvmet_pciep_get_prp_list(struct nvmet_pciep_ctrl *ctrl, u64 prp,
				    size_t xfer_len, __le64 *prps)
{
	size_t nr_prps = (xfer_len + ctrl->mps_mask) >> ctrl->mps_shift;
	u32 length;
	int ret;

	/*
	 * Compute the number of PRPs required for the number of bytes to
	 * transfer (xfer_len). If this number overflows the memory page size
	 * with the PRP list pointer specified, only return the space available
	 * in the memory page, the last PRP in there will be a PRP list pointer
	 * to the remaining PRPs.
	 */
	length = min(nvmet_pciep_prp_size(ctrl, prp), nr_prps << 3);
	ret = nvmet_pciep_transfer(ctrl, prps, prp, length, DMA_FROM_DEVICE);
	if (ret)
		return ret;

	return length >> 3;
}

static int nvmet_pciep_iod_parse_prp_list(struct nvmet_pciep_ctrl *ctrl,
					  struct nvmet_pciep_iod *iod)
{
	struct nvme_command *cmd = &iod->cmd;
	struct nvmet_pciep_segment *seg;
	size_t size = 0, ofst, prp_size, xfer_len;
	size_t transfer_len = iod->data_len;
	int nr_segs, nr_prps = 0;
	u64 pci_addr, prp;
	int i = 0, ret;
	__le64 *prps;

	prps = kzalloc(ctrl->mps, GFP_KERNEL);
	if (!prps)
		goto internal;

	/*
	 * Allocate PCI segments for the command: this considers the worst case
	 * scenario where all prps are discontiguous, so get as many segments
	 * as we can have prps. In practice, most of the time, we will have
	 * far less PCI segments than prps.
	 */
	prp = le64_to_cpu(cmd->common.dptr.prp1);
	if (!prp)
		goto invalid_field;

	ofst = nvmet_pciep_prp_ofst(ctrl, prp);
	nr_segs = (transfer_len + ofst + ctrl->mps - 1) >> ctrl->mps_shift;

	ret = nvmet_pciep_alloc_iod_data_segs(iod, nr_segs);
	if (ret)
		goto internal;

	/* Set the first segment using prp1 */
	seg = &iod->data_segs[0];
	seg->pci_addr = prp;
	seg->length = nvmet_pciep_prp_size(ctrl, prp);

	size = seg->length;
	pci_addr = prp + size;
	nr_segs = 1;

	/*
	 * Now build the PCI address segments using the prp lists, starting
	 * from prp2.
	 */
	prp = le64_to_cpu(cmd->common.dptr.prp2);
	if (!prp)
		goto invalid_field;

	while (size < transfer_len) {
		xfer_len = transfer_len - size;

		if (!nr_prps) {
			/* Get the prp list */
			nr_prps = nvmet_pciep_get_prp_list(ctrl, prp,
							   xfer_len, prps);
			if (nr_prps < 0)
				goto internal;

			i = 0;
			ofst = 0;
		}

		/* Current entry */
		prp = le64_to_cpu(prps[i]);
		if (!prp)
			goto invalid_field;

		/* Did we reach the last prp entry of the list ? */
		if (xfer_len > ctrl->mps && i == nr_prps - 1) {
			/* We need more PRPs: prp is a list pointer */
			nr_prps = 0;
			continue;
		}

		/* Only the first prp is allowed to have an offset */
		if (nvmet_pciep_prp_ofst(ctrl, prp))
			goto invalid_offset;

		if (prp != pci_addr) {
			/* Discontiguous prp: new segment */
			nr_segs++;
			if (WARN_ON_ONCE(nr_segs > iod->nr_data_segs))
				goto internal;

			seg++;
			seg->pci_addr = prp;
			seg->length = 0;
			pci_addr = prp;
		}

		prp_size = min_t(size_t, ctrl->mps, xfer_len);
		seg->length += prp_size;
		pci_addr += prp_size;
		size += prp_size;

		i++;
	}

	iod->nr_data_segs = nr_segs;
	ret = 0;

	if (size != transfer_len) {
		dev_err(ctrl->dev, "PRPs transfer length mismatch %zu / %zu\n",
			size, transfer_len);
		goto internal;
	}

	kfree(prps);

	return 0;

invalid_offset:
	dev_err(ctrl->dev, "PRPs list invalid offset\n");
	kfree(prps);
	iod->status = NVME_SC_PRP_INVALID_OFFSET | NVME_STATUS_DNR;
	return -EINVAL;

invalid_field:
	dev_err(ctrl->dev, "PRPs list invalid field\n");
	kfree(prps);
	iod->status = NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	return -EINVAL;

internal:
	dev_err(ctrl->dev, "PRPs list internal error\n");
	kfree(prps);
	iod->status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
	return -EINVAL;
}

static int nvmet_pciep_iod_parse_prp_simple(struct nvmet_pciep_ctrl *ctrl,
					    struct nvmet_pciep_iod *iod)
{
	struct nvme_command *cmd = &iod->cmd;
	size_t transfer_len = iod->data_len;
	int ret, nr_segs = 1;
	u64 prp1, prp2 = 0;
	size_t prp1_size;

	/* prp1 */
	prp1 = le64_to_cpu(cmd->common.dptr.prp1);
	prp1_size = nvmet_pciep_prp_size(ctrl, prp1);

	/* For commands crossing a page boundary, we should have a valid prp2 */
	if (transfer_len > prp1_size) {
		prp2 = le64_to_cpu(cmd->common.dptr.prp2);
		if (!prp2) {
			iod->status = NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
			return -EINVAL;
		}
		if (nvmet_pciep_prp_ofst(ctrl, prp2)) {
			iod->status =
				NVME_SC_PRP_INVALID_OFFSET | NVME_STATUS_DNR;
			return -EINVAL;
		}
		if (prp2 != prp1 + prp1_size)
			nr_segs = 2;
	}

	if (nr_segs == 1) {
		iod->nr_data_segs = 1;
		iod->data_segs = &iod->data_seg;
		iod->data_segs[0].pci_addr = prp1;
		iod->data_segs[0].length = transfer_len;
		return 0;
	}

	ret = nvmet_pciep_alloc_iod_data_segs(iod, nr_segs);
	if (ret) {
		iod->status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
		return ret;
	}

	iod->data_segs[0].pci_addr = prp1;
	iod->data_segs[0].length = prp1_size;
	iod->data_segs[1].pci_addr = prp2;
	iod->data_segs[1].length = transfer_len - prp1_size;

	return 0;
}

static int nvmet_pciep_iod_parse_prps(struct nvmet_pciep_iod *iod)
{
	struct nvmet_pciep_ctrl *ctrl = iod->ctrl;
	u64 prp1 = le64_to_cpu(iod->cmd.common.dptr.prp1);
	size_t ofst;

	/* Get the PCI address segments for the command using its PRPs */
	ofst = nvmet_pciep_prp_ofst(ctrl, prp1);
	if (ofst & 0x3) {
		iod->status = NVME_SC_PRP_INVALID_OFFSET | NVME_STATUS_DNR;
		return -EINVAL;
	}

	if (iod->data_len + ofst <= ctrl->mps * 2)
		return nvmet_pciep_iod_parse_prp_simple(ctrl, iod);

	return nvmet_pciep_iod_parse_prp_list(ctrl, iod);
}

/*
 * Transfer an SGL segment from the host and return the number of data
 * descriptors and the next segment descriptor, if any.
 */
static struct nvme_sgl_desc *
nvmet_pciep_get_sgl_segment(struct nvmet_pciep_ctrl *ctrl,
			    struct nvme_sgl_desc *desc, unsigned int *nr_sgls)
{
	struct nvme_sgl_desc *sgls;
	u32 length = le32_to_cpu(desc->length);
	int nr_descs, ret;
	void *buf;

	buf = kmalloc(length, GFP_KERNEL);
	if (!buf)
		return NULL;

	ret = nvmet_pciep_transfer(ctrl, buf, le64_to_cpu(desc->addr), length,
				   DMA_FROM_DEVICE);
	if (ret) {
		kfree(buf);
		return NULL;
	}

	sgls = buf;
	nr_descs = length / sizeof(struct nvme_sgl_desc);
	if (sgls[nr_descs - 1].type == (NVME_SGL_FMT_SEG_DESC << 4) ||
	    sgls[nr_descs - 1].type == (NVME_SGL_FMT_LAST_SEG_DESC << 4)) {
		/*
		 * We have another SGL segment following this one: do not count
		 * it as a regular data SGL descriptor and return it to the
		 * caller.
		 */
		*desc = sgls[nr_descs - 1];
		nr_descs--;
	} else {
		/* We do not have another SGL segment after this one. */
		desc->length = 0;
	}

	*nr_sgls = nr_descs;

	return sgls;
}

static int nvmet_pciep_iod_parse_sgl_segments(struct nvmet_pciep_ctrl *ctrl,
					      struct nvmet_pciep_iod *iod)
{
	struct nvme_command *cmd = &iod->cmd;
	struct nvme_sgl_desc seg = cmd->common.dptr.sgl;
	struct nvme_sgl_desc *sgls = NULL;
	int n = 0, i, nr_sgls;
	int ret;

	/*
	 * We do not support inline data nor keyed SGLs, so we should be seeing
	 * only segment descriptors.
	 */
	if (seg.type != (NVME_SGL_FMT_SEG_DESC << 4) &&
	    seg.type != (NVME_SGL_FMT_LAST_SEG_DESC << 4)) {
		iod->status = NVME_SC_SGL_INVALID_TYPE | NVME_STATUS_DNR;
		return -EIO;
	}

	while (seg.length) {
		sgls = nvmet_pciep_get_sgl_segment(ctrl, &seg, &nr_sgls);
		if (!sgls) {
			iod->status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
			return -EIO;
		}

		/* Grow the PCI segment table as needed */
		ret = nvmet_pciep_alloc_iod_data_segs(iod, nr_sgls);
		if (ret) {
			iod->status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
			goto out;
		}

		/*
		 * Parse the SGL descriptors to build the PCI segment table,
		 * checking the descriptor type as we go.
		 */
		for(i = 0; i < nr_sgls; i++) {
			if (sgls[i].type != (NVME_SGL_FMT_DATA_DESC << 4)) {
				iod->status = NVME_SC_SGL_INVALID_TYPE |
					NVME_STATUS_DNR;
				goto out;
			}
			iod->data_segs[n].pci_addr = le64_to_cpu(sgls[i].addr);
			iod->data_segs[n].length = le32_to_cpu(sgls[i].length);
			n++;
		}

		kfree(sgls);
	}

 out:
	if (iod->status != NVME_SC_SUCCESS) {
		kfree(sgls);
		return -EIO;
	}

	return 0;
}

static int nvmet_pciep_iod_parse_sgls(struct nvmet_pciep_iod *iod)
{
	struct nvmet_pciep_ctrl *ctrl = iod->ctrl;
	struct nvme_sgl_desc *sgl = &iod->cmd.common.dptr.sgl;

	if (sgl->type == (NVME_SGL_FMT_DATA_DESC << 4)) {
		/* Single data descriptor case */
		iod->nr_data_segs = 1;
		iod->data_segs = &iod->data_seg;
		iod->data_seg.pci_addr = le64_to_cpu(sgl->addr);
		iod->data_seg.length = le32_to_cpu(sgl->length);
		return 0;
	}

	return nvmet_pciep_iod_parse_sgl_segments(ctrl, iod);
}

static int nvmet_pciep_alloc_iod_data_buf(struct nvmet_pciep_iod *iod)
{
	struct nvmet_pciep_ctrl *ctrl = iod->ctrl;
	struct nvmet_req *req = &iod->req;
	struct nvmet_pciep_segment *seg;
	struct scatterlist *sg;
	int ret, i;

	if (iod->data_len > ctrl->mdts) {
		iod->status = NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		return -EINVAL;
	}

	/*
	 * Get the PCI address segments for the command data buffer using either
	 * its SGLs or PRPs.
	 */
	if (iod->cmd.common.flags & NVME_CMD_SGL_ALL)
		ret = nvmet_pciep_iod_parse_sgls(iod);
	else
		ret = nvmet_pciep_iod_parse_prps(iod);
	if (ret)
		return ret;

	/* Get a command buffer using SGLs matching the PCI segments. */
	if (iod->nr_data_segs == 1) {
		sg_init_table(&iod->data_sgl, 1);
		iod->data_sgt.sgl = &iod->data_sgl;
		iod->data_sgt.nents = 1;
		iod->data_sgt.orig_nents = 1;
	} else {
		ret = sg_alloc_table(&iod->data_sgt, iod->nr_data_segs,
				     GFP_KERNEL);
		if (ret)
			goto err_nomem;
	}

	for_each_sgtable_sg(&iod->data_sgt, sg, i) {
		seg = &iod->data_segs[i];
		seg->buf = kmalloc(seg->length, GFP_KERNEL);
		if (!seg->buf)
			goto err_nomem;
		sg_set_buf(sg, seg->buf, seg->length);
	}

	req->transfer_len = iod->data_len;
	req->sg = iod->data_sgt.sgl;
	req->sg_cnt = iod->data_sgt.nents;

	return 0;

err_nomem:
	iod->status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
	return -ENOMEM;
}

static void nvmet_pciep_complete_iod(struct nvmet_pciep_iod *iod)
{
	struct nvmet_pciep_queue *cq = iod->cq;
	unsigned long flags;

	/* Do not print an error message for AENs */
	iod->status = le16_to_cpu(iod->cqe.status) >> 1;
	if (iod->status && iod->cmd.common.opcode != nvme_admin_async_event)
		dev_err(iod->ctrl->dev,
			"CQ[%d]: Command %s (0x%x) status 0x%0x\n",
			iod->sq->qid, nvmet_pciep_iod_name(iod),
			iod->cmd.common.opcode, iod->status);

	/*
	 * Add the command to the list of completed commands and schedule the
	 * CQ work.
	 */
	spin_lock_irqsave(&cq->lock, flags);
	list_add_tail(&iod->link, &cq->list);
	atomic_inc(&cq->list_count);
	queue_delayed_work(system_highpri_wq, &cq->work, 0);
	spin_unlock_irqrestore(&cq->lock, flags);
}

static void nvmet_pciep_drain_queue(struct nvmet_pciep_queue *queue)
{
	struct nvmet_pciep_iod *iod;
	unsigned long flags;
	LIST_HEAD(list);

	spin_lock_irqsave(&queue->lock, flags);
	if (!list_empty(&queue->list))
		list_splice_tail_init(&queue->list, &list);
	spin_unlock_irqrestore(&queue->lock, flags);

	while (!list_empty(&list)) {
		iod = list_first_entry(&list, struct nvmet_pciep_iod, link);
		list_del_init(&iod->link);
		if (test_bit(NVMET_PCIEP_Q_IS_SQ, &queue->flags)) {
			iod->status = NVME_SC_ABORT_QUEUE | NVME_STATUS_DNR;
			nvmet_pciep_complete_iod(iod);
		} else {
			atomic_dec(&queue->list_count);
			nvmet_pciep_free_iod(iod);
		}
	}
}

static int nvmet_pciep_add_port(struct nvmet_port *port)
{
	mutex_lock(&nvmet_pciep_ports_mutex);
	list_add_tail(&port->entry, &nvmet_pciep_ports);
	mutex_unlock(&nvmet_pciep_ports_mutex);
	return 0;
}

static void nvmet_pciep_remove_port(struct nvmet_port *port)
{
	mutex_lock(&nvmet_pciep_ports_mutex);
	list_del_init(&port->entry);
	mutex_unlock(&nvmet_pciep_ports_mutex);
}

static struct nvmet_port *nvmet_pciep_find_port(struct nvmet_pciep_ctrl *ctrl,
						__le16 portid)
{
	struct nvmet_port *p, *port = NULL;

	/* For now, always use the first port */
	mutex_lock(&nvmet_pciep_ports_mutex);
	list_for_each_entry(p, &nvmet_pciep_ports, entry) {
		if (p->disc_addr.portid == portid) {
			port = p;
			break;
		}
	}
	mutex_unlock(&nvmet_pciep_ports_mutex);

	return port;
}

static void nvmet_pciep_queue_response(struct nvmet_req *req)
{
	struct nvmet_pciep_iod *iod =
		container_of(req, struct nvmet_pciep_iod, req);

	iod->status = le16_to_cpu(req->cqe->status) >> 1;

	/* If we have no data to transfer, directly complete the command. */
	if (!iod->data_len || iod->dma_dir != DMA_TO_DEVICE) {
		nvmet_pciep_complete_iod(iod);
		return;
	}

	complete(&iod->done);
}

static u8 nvmet_pciep_get_mdts(const struct nvmet_ctrl *tctrl)
{
	struct nvmet_pciep_ctrl *ctrl = tctrl->drvdata;
	int page_shift = NVME_CAP_MPSMIN(tctrl->cap) + 12;

	return ilog2(ctrl->mdts) - page_shift;
}

static u16 nvmet_pciep_create_cq(struct nvmet_ctrl *tctrl, u16 cqid, u16 flags,
				 u16 qsize, u64 pci_addr, u16 vector)
{
	struct nvmet_pciep_ctrl *ctrl = tctrl->drvdata;
	struct nvmet_pciep_queue *cq = &ctrl->cq[cqid];
	u16 status;

	if (test_and_set_bit(NVMET_PCIEP_Q_LIVE, &cq->flags))
		return NVME_SC_QID_INVALID | NVME_STATUS_DNR;

	if (!(flags & NVME_QUEUE_PHYS_CONTIG))
		return NVME_SC_INVALID_QUEUE | NVME_STATUS_DNR;

	if (flags & NVME_CQ_IRQ_ENABLED)
		set_bit(NVMET_PCIEP_Q_IRQ_ENABLED, &cq->flags);

	cq->pci_addr = pci_addr;
	cq->qid = cqid;
	cq->depth = qsize + 1;
	cq->vector = vector;
	cq->head = 0;
	cq->tail = 0;
	cq->phase = 1;
	cq->db = NVME_REG_DBS + (((cqid * 2) + 1) * sizeof(u32));
	nvmet_pciep_bar_write32(ctrl, cq->db, 0);

	if (!cqid)
		cq->qes = sizeof(struct nvme_completion);
	else
		cq->qes = ctrl->io_cqes;
	cq->pci_size = cq->qes * cq->depth;

	cq->iv = nvmet_pciep_add_irq_vector(ctrl, vector);
	if (!cq->iv) {
		status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
		goto err;
	}

	status = nvmet_cq_create(tctrl, &cq->nvme_cq, cqid, cq->depth);
	if (status != NVME_SC_SUCCESS)
		goto err;

	dev_dbg(ctrl->dev, "CQ[%u]: %u entries of %zu B, IRQ vector %u\n",
		cqid, qsize, cq->qes, cq->vector);

	return NVME_SC_SUCCESS;

err:
	clear_bit(NVMET_PCIEP_Q_IRQ_ENABLED, &cq->flags);
	clear_bit(NVMET_PCIEP_Q_LIVE, &cq->flags);
	return status;
}

static u16 nvmet_pciep_delete_cq(struct nvmet_ctrl *tctrl, u16 cqid)
{
	struct nvmet_pciep_ctrl *ctrl = tctrl->drvdata;
	struct nvmet_pciep_queue *cq = &ctrl->cq[cqid];

	if (!test_and_clear_bit(NVMET_PCIEP_Q_LIVE, &cq->flags))
		return NVME_SC_QID_INVALID | NVME_STATUS_DNR;

	cancel_delayed_work_sync(&cq->work);
	nvmet_pciep_drain_queue(cq);
	nvmet_pciep_remove_irq_vector(ctrl, cq->vector);

	return NVME_SC_SUCCESS;
}

static u16 nvmet_pciep_create_sq(struct nvmet_ctrl *tctrl, u16 sqid, u16 flags,
				 u16 qsize, u64 pci_addr)
{
	struct nvmet_pciep_ctrl *ctrl = tctrl->drvdata;
	struct nvmet_pciep_queue *sq = &ctrl->sq[sqid];
	u16 status;

	if (test_and_set_bit(NVMET_PCIEP_Q_LIVE, &sq->flags))
		return NVME_SC_QID_INVALID | NVME_STATUS_DNR;

	if (!(flags & NVME_QUEUE_PHYS_CONTIG))
		return NVME_SC_INVALID_QUEUE | NVME_STATUS_DNR;

	sq->pci_addr = pci_addr;
	sq->qid = sqid;
	sq->depth = qsize + 1;
	sq->head = 0;
	sq->tail = 0;
	sq->phase = 0;
	sq->db = NVME_REG_DBS + (sqid * 2 * sizeof(u32));
	nvmet_pciep_bar_write32(ctrl, sq->db, 0);
	if (!sqid)
		sq->qes = 1UL << NVME_ADM_SQES;
	else
		sq->qes = ctrl->io_sqes;
	sq->pci_size = sq->qes * sq->depth;

	status = nvmet_sq_create(tctrl, &sq->nvme_sq, sqid, sq->depth);
	if (status != NVME_SC_SUCCESS)
		goto out_clear_bit;

	sq->iod_wq = alloc_workqueue("sq%d_wq", WQ_UNBOUND,
				min_t(int, sq->depth, WQ_MAX_ACTIVE), sqid);
	if (!sq->iod_wq) {
		dev_err(ctrl->dev, "Create SQ %d work queue failed\n", sqid);
		status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
		goto out_destroy_sq;
	}

	dev_dbg(ctrl->dev, "SQ[%u]: %u entries of %zu B\n",
		sqid, qsize, sq->qes);

	return NVME_SC_SUCCESS;

out_destroy_sq:
	nvmet_sq_destroy(&sq->nvme_sq);
out_clear_bit:
	clear_bit(NVMET_PCIEP_Q_LIVE, &sq->flags);
	return status;
}

static u16 nvmet_pciep_delete_sq(struct nvmet_ctrl *tctrl, u16 sqid)
{
	struct nvmet_pciep_ctrl *ctrl = tctrl->drvdata;
	struct nvmet_pciep_queue *sq = &ctrl->sq[sqid];

	if (!test_and_clear_bit(NVMET_PCIEP_Q_LIVE, &sq->flags))
		return NVME_SC_QID_INVALID | NVME_STATUS_DNR;

	flush_workqueue(sq->iod_wq);
	destroy_workqueue(sq->iod_wq);
	sq->iod_wq = NULL;

	nvmet_pciep_drain_queue(sq);

	if (sq->nvme_sq.ctrl)
		nvmet_sq_destroy(&sq->nvme_sq);

	return NVME_SC_SUCCESS;
}

static u16 nvmet_pciep_get_feat(const struct nvmet_ctrl *tctrl, u8 feat,
				void *data)
{
	struct nvmet_pciep_ctrl *ctrl = tctrl->drvdata;
	struct nvmet_feat_arbitration *arb;
	struct nvmet_feat_irq_coalesce *irqc;
	struct nvmet_feat_irq_config *irqcfg;
	struct nvmet_pciep_irq_vector *iv;
	u16 status;

	switch (feat) {
	case NVME_FEAT_ARBITRATION:
		arb = data;
		if (!ctrl->sq_ab)
			arb->ab = 0x7;
		else
			arb->ab = ilog2(ctrl->sq_ab);
		return NVME_SC_SUCCESS;

	case NVME_FEAT_IRQ_COALESCE:
		irqc = data;
		irqc->thr = ctrl->irq_vector_threshold;
		irqc->time = 0;
		return NVME_SC_SUCCESS;

	case NVME_FEAT_IRQ_CONFIG:
		irqcfg = data;
		mutex_lock(&ctrl->irq_lock);
		iv = nvmet_pciep_find_irq_vector(ctrl, irqcfg->iv);
		if (iv) {
			irqcfg->cd = iv->cd;
			status = NVME_SC_SUCCESS;
		} else {
			status = NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		}
		mutex_unlock(&ctrl->irq_lock);
		return status;

	default:
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
}

static u16 nvmet_pciep_set_feat(const struct nvmet_ctrl *tctrl, u8 feat,
				void *data)
{
	struct nvmet_pciep_ctrl *ctrl = tctrl->drvdata;
	struct nvmet_feat_arbitration *arb;
	struct nvmet_feat_irq_coalesce *irqc;
	struct nvmet_feat_irq_config *irqcfg;
	struct nvmet_pciep_irq_vector *iv;
	u16 status;

	switch (feat) {
	case NVME_FEAT_ARBITRATION:
		arb = data;
		if (arb->ab == 0x7)
			ctrl->sq_ab = 0;
		else
			ctrl->sq_ab = 1 << arb->ab;
		return NVME_SC_SUCCESS;

	case NVME_FEAT_IRQ_COALESCE:
		/*
		 * Note: since we do not implement precise IRQ coalescing timing,
		 * so ignore the time field.
		 */
		irqc = data;
		ctrl->irq_vector_threshold = irqc->thr + 1;
		return NVME_SC_SUCCESS;

	case NVME_FEAT_IRQ_CONFIG:
		irqcfg = data;
		mutex_lock(&ctrl->irq_lock);
		iv = nvmet_pciep_find_irq_vector(ctrl, irqcfg->iv);
		if (iv) {
			iv->cd = irqcfg->cd;
			status = NVME_SC_SUCCESS;
		} else {
			status = NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		}
		mutex_unlock(&ctrl->irq_lock);
		return status;

	default:
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
}

static const struct nvmet_fabrics_ops nvmet_pciep_fabrics_ops = {
	.owner		= THIS_MODULE,
	.type		= NVMF_TRTYPE_PCI,
	.add_port	= nvmet_pciep_add_port,
	.remove_port	= nvmet_pciep_remove_port,
	.queue_response = nvmet_pciep_queue_response,
	.get_mdts	= nvmet_pciep_get_mdts,
	.create_cq	= nvmet_pciep_create_cq,
	.delete_cq	= nvmet_pciep_delete_cq,
	.create_sq	= nvmet_pciep_create_sq,
	.delete_sq	= nvmet_pciep_delete_sq,
	.get_feature	= nvmet_pciep_get_feat,
	.set_feature	= nvmet_pciep_set_feat,
};

static void nvmet_pciep_cq_work(struct work_struct *work);

static void nvmet_pciep_init_queue(struct nvmet_pciep_ctrl *ctrl,
				   unsigned int qid, bool sq)
{
	struct nvmet_pciep_queue *queue;

	if (sq) {
		queue = &ctrl->sq[qid];
		set_bit(NVMET_PCIEP_Q_IS_SQ, &queue->flags);
	} else {
		queue = &ctrl->cq[qid];
		INIT_DELAYED_WORK(&queue->work, nvmet_pciep_cq_work);
	}
	queue->ctrl = ctrl;
	queue->qid = qid;
	spin_lock_init(&queue->lock);
	INIT_LIST_HEAD(&queue->list);
	atomic_set(&queue->list_count, 0);
}

static int nvmet_pciep_alloc_queues(struct nvmet_pciep_ctrl *ctrl)
{
	unsigned int qid;

	ctrl->sq = kcalloc(ctrl->nr_queues,
			   sizeof(struct nvmet_pciep_queue), GFP_KERNEL);
	if (!ctrl->sq)
		return -ENOMEM;

	ctrl->cq = kcalloc(ctrl->nr_queues,
			   sizeof(struct nvmet_pciep_queue), GFP_KERNEL);
	if (!ctrl->cq) {
		kfree(ctrl->sq);
		ctrl->sq = NULL;
		return -ENOMEM;
	}

	for (qid = 0; qid < ctrl->nr_queues; qid++) {
		nvmet_pciep_init_queue(ctrl, qid, true);
		nvmet_pciep_init_queue(ctrl, qid, false);
	}

	return 0;
}

static void nvmet_pciep_free_queues(struct nvmet_pciep_ctrl *ctrl)
{
	kfree(ctrl->sq);
	ctrl->sq = NULL;
	kfree(ctrl->cq);
	ctrl->cq = NULL;
}

static int nvmet_pciep_map_queue(struct nvmet_pciep_ctrl *ctrl,
				 struct nvmet_pciep_queue *queue)
{
	struct nvmet_pciep_epf *nvme_epf = ctrl->nvme_epf;
	int ret;

	ret = nvmet_pciep_epf_mem_map(nvme_epf, queue->pci_addr,
				      queue->pci_size, &queue->pci_map);
	if (ret) {
		dev_err(ctrl->dev, "Map %cQ %d failed %d\n",
			test_bit(NVMET_PCIEP_Q_IS_SQ, &queue->flags) ? 'S' : 'C',
			queue->qid, ret);
		return ret;
	}

	if (queue->pci_map.pci_size < queue->pci_size) {
		dev_err(ctrl->dev, "Partial %cQ %d mapping\n",
			test_bit(NVMET_PCIEP_Q_IS_SQ, &queue->flags) ? 'S' : 'C',
			queue->qid);
		nvmet_pciep_epf_mem_unmap(nvme_epf, &queue->pci_map);
		return -ENOMEM;
	}

	return 0;
}

static inline void nvmet_pciep_unmap_queue(struct nvmet_pciep_ctrl *ctrl,
					   struct nvmet_pciep_queue *queue)
{
	nvmet_pciep_epf_mem_unmap(ctrl->nvme_epf, &queue->pci_map);
}

static void nvmet_pciep_init_bar(struct nvmet_pciep_ctrl *ctrl)
{
	struct nvmet_ctrl *tctrl = ctrl->tctrl;

	ctrl->bar = ctrl->nvme_epf->reg_bar;

	/* Copy the target controller capabilities as a base */
	ctrl->cap = tctrl->cap;

	/* Contiguous Queues Required (CQR) */
	ctrl->cap |= 0x1ULL << 16;

	/* Set Doorbell stride to 4B (DSTRB) */
	ctrl->cap &= ~GENMASK(35, 32);

	/* Clear NVM Subsystem Reset Supported (NSSRS) */
	ctrl->cap &= ~(0x1ULL << 36);

	/* Clear Boot Partition Support (BPS) */
	ctrl->cap &= ~(0x1ULL << 45);

	/* Clear Persistent Memory Region Supported (PMRS) */
	ctrl->cap &= ~(0x1ULL << 56);

	/* Clear Controller Memory Buffer Supported (CMBS) */
	ctrl->cap &= ~(0x1ULL << 57);

	/* Controller configuration */
	ctrl->cc = tctrl->cc & (~NVME_CC_ENABLE);

	/* Controller status */
	ctrl->csts = ctrl->tctrl->csts;

	nvmet_pciep_bar_write64(ctrl, NVME_REG_CAP, ctrl->cap);
	nvmet_pciep_bar_write32(ctrl, NVME_REG_VS, tctrl->subsys->ver);
	nvmet_pciep_bar_write32(ctrl, NVME_REG_CSTS, ctrl->csts);
	nvmet_pciep_bar_write32(ctrl, NVME_REG_CC, ctrl->cc);
}

static void nvmet_pciep_poll_cc_work(struct work_struct *work);
static void nvmet_pciep_poll_sqs_work(struct work_struct *work);

static int nvmet_pciep_create_ctrl(struct nvmet_pciep_epf *nvme_epf,
				   unsigned int max_nr_queues)
{
	struct nvmet_pciep_ctrl *ctrl = &nvme_epf->ctrl;
	struct nvmet_alloc_ctrl_args args = {};
	char hostnqn[NVMF_NQN_SIZE];
	uuid_t id;
	int ret;

	memset(ctrl, 0, sizeof(*ctrl));
	ctrl->dev = &nvme_epf->epf->dev;
	mutex_init(&ctrl->irq_lock);
	ctrl->nvme_epf = nvme_epf;
	ctrl->mdts = nvme_epf->mdts_kb * SZ_1K;
	INIT_DELAYED_WORK(&ctrl->poll_cc, nvmet_pciep_poll_cc_work);
	INIT_DELAYED_WORK(&ctrl->poll_sqs, nvmet_pciep_poll_sqs_work);

	ret = mempool_init_kmalloc_pool(&ctrl->iod_pool,
					max_nr_queues * NVMET_MAX_QUEUE_SIZE,
					sizeof(struct nvmet_pciep_iod));
	if (ret) {
		dev_err(ctrl->dev, "Initialize iod mempool failed\n");
		return ret;
	}

	ctrl->port = nvmet_pciep_find_port(ctrl, nvme_epf->portid);
	if (!ctrl->port) {
		dev_err(ctrl->dev, "Port not found\n");
		ret = -EINVAL;
		goto out_mempool_exit;
	}

	/* Create the target controller */
	uuid_gen(&id);
        snprintf(hostnqn, NVMF_NQN_SIZE,
                "nqn.2014-08.org.nvmexpress:uuid:%pUb", &id);
	args.port = ctrl->port;
	args.subsysnqn = nvme_epf->subsysnqn;
	memset(&id, 0, sizeof(uuid_t));
	args.hostid = &id;
	args.hostnqn = hostnqn;
	args.ops = &nvmet_pciep_fabrics_ops;

	ctrl->tctrl = nvmet_alloc_ctrl(&args);
	if (!ctrl->tctrl) {
		dev_err(ctrl->dev, "Create target controller failed\n");
		ret = -ENOMEM;
		goto out_mempool_exit;
	}
	ctrl->tctrl->drvdata = ctrl;

	/* We do not support protection information for now. */
	if (ctrl->tctrl->pi_support) {
		dev_err(ctrl->dev, "PI support is not supported\n");
		ret = -ENOTSUPP;
		goto out_put_ctrl;
	}

	/* Allocate our queues, up to the maximum number */
	ctrl->nr_queues = min(ctrl->tctrl->subsys->max_qid + 1, max_nr_queues);
	ret = nvmet_pciep_alloc_queues(ctrl);
	if (ret)
		goto out_put_ctrl;

	/*
	 * Allocate the IRQ vectors descriptors. We cannot have more than the
	 * maximum number of queues.
	 */
	ret = nvmet_pciep_alloc_irq_vectors(ctrl);
	if (ret)
		goto out_free_queues;

	dev_info(ctrl->dev,
		 "New PCI ctrl \"%s\", %u I/O queues, mdts %u B\n",
		 ctrl->tctrl->subsys->subsysnqn, ctrl->nr_queues - 1,
		 ctrl->mdts);

	/* Initialize BAR 0 using the target controller CAP */
	nvmet_pciep_init_bar(ctrl);

	return 0;

out_free_queues:
	nvmet_pciep_free_queues(ctrl);
out_put_ctrl:
	nvmet_ctrl_put(ctrl->tctrl);
	ctrl->tctrl = NULL;
out_mempool_exit:
	mempool_exit(&ctrl->iod_pool);
	return ret;
}

static int nvmet_pciep_enable_ctrl(struct nvmet_pciep_ctrl *ctrl)
{
	u64 pci_addr, asq, acq;
	u32 aqa;
	u16 status, qsize;
	int ret;

	if (ctrl->enabled)
		return 0;

	dev_info(ctrl->dev, "Enabling controller\n");

	ctrl->mps_shift = nvmet_cc_mps(ctrl->cc) + 12;
	ctrl->mps = 1UL << ctrl->mps_shift;
	ctrl->mps_mask = ctrl->mps - 1;

	ctrl->io_sqes = 1UL << nvmet_cc_iosqes(ctrl->cc);
	ctrl->io_cqes = 1UL << nvmet_cc_iocqes(ctrl->cc);

	if (ctrl->io_sqes < sizeof(struct nvme_command)) {
		dev_err(ctrl->dev, "Unsupported IO SQES %zu (need %zu)\n",
			ctrl->io_sqes, sizeof(struct nvme_command));
		return -EINVAL;
	}

	if (ctrl->io_cqes < sizeof(struct nvme_completion)) {
		dev_err(ctrl->dev, "Unsupported IO CQES %zu (need %zu)\n",
			ctrl->io_sqes, sizeof(struct nvme_completion));
		return -EINVAL;
	}

	/* Create the admin queue. */
	aqa = nvmet_pciep_bar_read32(ctrl, NVME_REG_AQA);
	asq = nvmet_pciep_bar_read64(ctrl, NVME_REG_ASQ);
	acq = nvmet_pciep_bar_read64(ctrl, NVME_REG_ACQ);

	qsize = (aqa & 0x0fff0000) >> 16;
	pci_addr = acq & GENMASK(63, 12);
	status = nvmet_pciep_create_cq(ctrl->tctrl, 0,
				NVME_CQ_IRQ_ENABLED | NVME_QUEUE_PHYS_CONTIG,
				qsize, pci_addr, 0);
	if (status != NVME_SC_SUCCESS) {
		dev_err(ctrl->dev, "Create admin completion queue failed\n");
		return -ENODEV;
	}

	qsize = aqa & 0x00000fff;
	pci_addr = asq & GENMASK(63, 12);
	status = nvmet_pciep_create_sq(ctrl->tctrl, 0, NVME_QUEUE_PHYS_CONTIG,
				       qsize, pci_addr);
	if (status != NVME_SC_SUCCESS) {
		dev_err(ctrl->dev, "Create admin submission queue failed\n");
		nvmet_pciep_delete_cq(ctrl->tctrl, 0);
		return ret;
	}

	ctrl->sq_ab = NVMET_PCIE_SQ_AB;
	ctrl->irq_vector_threshold = NVMET_PCIEP_IV_THRESHOLD;
	ctrl->enabled = true;

	/* Start polling the controller SQs */
	schedule_delayed_work(&ctrl->poll_sqs, 0);

	return 0;
}

static void nvmet_pciep_disable_ctrl(struct nvmet_pciep_ctrl *ctrl)
{
	int qid;

	if (!ctrl->enabled)
		return;

	dev_info(ctrl->dev, "Disabling controller\n");

	ctrl->enabled = false;
	cancel_delayed_work_sync(&ctrl->poll_sqs);

	/* Delete all IO queues */
	for (qid = 1; qid < ctrl->nr_queues; qid++)
		nvmet_pciep_delete_sq(ctrl->tctrl, qid);

	for (qid = 1; qid < ctrl->nr_queues; qid++)
		nvmet_pciep_delete_cq(ctrl->tctrl, qid);

	/* Delete the admin queue last */
	nvmet_pciep_delete_sq(ctrl->tctrl, 0);
	nvmet_pciep_delete_cq(ctrl->tctrl, 0);
}

static void nvmet_pciep_start_ctrl(struct nvmet_pciep_ctrl *ctrl)
{
	schedule_delayed_work(&ctrl->poll_cc, NVMET_PCIEP_CC_POLL_INTERVAL);
}

static void nvmet_pciep_stop_ctrl(struct nvmet_pciep_ctrl *ctrl)
{
	cancel_delayed_work_sync(&ctrl->poll_cc);

	nvmet_pciep_disable_ctrl(ctrl);
}

static void nvmet_pciep_destroy_ctrl(struct nvmet_pciep_ctrl *ctrl)
{
	if (!ctrl->tctrl)
		return;

	dev_info(ctrl->dev, "Destroying PCI ctrl \"%s\"\n",
		 ctrl->tctrl->subsys->subsysnqn);

	nvmet_pciep_stop_ctrl(ctrl);

	nvmet_pciep_free_queues(ctrl);
	nvmet_pciep_free_irq_vectors(ctrl);

	nvmet_ctrl_put(ctrl->tctrl);
	ctrl->tctrl = NULL;

	mempool_exit(&ctrl->iod_pool);
}

static void nvmet_pciep_poll_cc_work(struct work_struct *work)
{
	struct nvmet_pciep_ctrl *ctrl =
		container_of(work, struct nvmet_pciep_ctrl, poll_cc.work);
	u32 old_cc, new_cc;
	int ret;

	if (!ctrl->tctrl)
		return;

	old_cc = ctrl->cc;
	new_cc = nvmet_pciep_bar_read32(ctrl, NVME_REG_CC);
	ctrl->cc = new_cc;

	if (nvmet_cc_en(new_cc) && !nvmet_cc_en(old_cc)) {
		/* Enable the controller */
		ret = nvmet_pciep_enable_ctrl(ctrl);
		if (ret)
			return;
		ctrl->csts |= NVME_CSTS_RDY;
	}

	if (!nvmet_cc_en(new_cc) && nvmet_cc_en(old_cc)) {
		nvmet_pciep_disable_ctrl(ctrl);
		ctrl->csts &= ~NVME_CSTS_RDY;
	}

	if (nvmet_cc_shn(new_cc) && !nvmet_cc_shn(old_cc)) {
		nvmet_pciep_disable_ctrl(ctrl);
		ctrl->csts |= NVME_CSTS_SHST_CMPLT;
	}

	if (!nvmet_cc_shn(new_cc) && nvmet_cc_shn(old_cc)) {
		ctrl->csts &= ~NVME_CSTS_SHST_CMPLT;
	}

	nvmet_update_cc(ctrl->tctrl, ctrl->cc);
	nvmet_pciep_bar_write32(ctrl, NVME_REG_CSTS, ctrl->csts);

	schedule_delayed_work(&ctrl->poll_cc, NVMET_PCIEP_CC_POLL_INTERVAL);
}

static void nvmet_pciep_exec_iod_work(struct work_struct *work)
{
	struct nvmet_pciep_iod *iod =
		container_of(work, struct nvmet_pciep_iod, work);
	struct nvmet_req *req = &iod->req;
	int ret;

	if (!iod->ctrl->link_up) {
		nvmet_pciep_free_iod(iod);
		return;
	}

	if (!test_bit(NVMET_PCIEP_Q_LIVE, &iod->sq->flags)) {
		iod->status = NVME_SC_QID_INVALID | NVME_STATUS_DNR;
		goto complete;
	}

	if (!nvmet_req_init(req, &iod->cq->nvme_cq, &iod->sq->nvme_sq,
			    &nvmet_pciep_fabrics_ops))
		goto complete;

	iod->data_len = nvmet_req_transfer_len(req);
	if (iod->data_len) {
		/*
		 * Get the data DMA transfer direction. Here "device" means the
		 * PCI root-complex host.
		 */
		if (nvme_is_write(&iod->cmd))
			iod->dma_dir = DMA_FROM_DEVICE;
		else
			iod->dma_dir = DMA_TO_DEVICE;

		/*
		 * Setup the command data buffer and get the command data from
		 * the host if needed.
		 */
		ret = nvmet_pciep_alloc_iod_data_buf(iod);
		if (!ret && iod->dma_dir == DMA_FROM_DEVICE)
			ret = nvmet_pciep_transfer_iod_data(iod);
		if (ret) {
			nvmet_req_uninit(req);
			goto complete;
		}
	}

	req->execute(req);

	/*
	 * If we do not have data to transfer after the command execution
	 * finishes, nvmet_pciep_queue_response() will complete the command
	 * directly. No need to wait for the completion in this case.
	 */
	if (!iod->data_len || iod->dma_dir != DMA_TO_DEVICE)
		return;

	wait_for_completion(&iod->done);

	if (iod->status == NVME_SC_SUCCESS) {
		WARN_ON_ONCE(!iod->data_len || iod->dma_dir != DMA_TO_DEVICE);
		nvmet_pciep_transfer_iod_data(iod);
	}

complete:
	nvmet_pciep_complete_iod(iod);
}

static int nvmet_pciep_process_sq(struct nvmet_pciep_ctrl *ctrl,
				  struct nvmet_pciep_queue *sq)
{
	struct nvmet_pciep_iod *iod;
	int n = 0;

	sq->tail = nvmet_pciep_bar_read32(ctrl, sq->db);
	while (sq->head != sq->tail && (!ctrl->sq_ab || n < ctrl->sq_ab)) {
		iod = nvmet_pciep_alloc_iod(sq);
		if (!iod)
			break;

		/* Get the NVMe command submitted by the host */
		nvmet_pciep_transfer(ctrl, &iod->cmd,
				     sq->pci_addr + sq->head * sq->qes,
				     sizeof(struct nvme_command),
				     DMA_FROM_DEVICE);

		dev_dbg(ctrl->dev, "SQ[%u]: head %u, tail %u, command %s\n",
			sq->qid, sq->head, sq->tail, nvmet_pciep_iod_name(iod));

		sq->head++;
		if (sq->head == sq->depth)
			sq->head = 0;
		n++;

		queue_work_on(WORK_CPU_UNBOUND, sq->iod_wq, &iod->work);

		sq->tail = nvmet_pciep_bar_read32(ctrl, sq->db);
	}

	return n;
}

static void nvmet_pciep_poll_sqs_work(struct work_struct *work)
{
	struct nvmet_pciep_ctrl *ctrl =
		container_of(work, struct nvmet_pciep_ctrl, poll_sqs.work);
	struct nvmet_pciep_queue *sq;
	unsigned long last = 0;
	int i, nr_sqs;

	while (ctrl->link_up && ctrl->enabled) {
		nr_sqs = 0;
		/* Do round-robin command arbitration */
		for (i = 0; i < ctrl->nr_queues; i++) {
			sq = &ctrl->sq[i];
			if (!test_bit(NVMET_PCIEP_Q_LIVE, &sq->flags))
				continue;
			if (nvmet_pciep_process_sq(ctrl, sq))
				nr_sqs++;
		}

		if (nr_sqs) {
			last = jiffies;
			continue;
		}

		/*
		 * If we have not received any command on any queue for more than
		 * NVMET_PCIEP_SQ_POLL_IDLE, assume we are idle and reschedule.
		 * This avoids "burning" a CPU when the controller is idle for a
		 * long time.
		 */
		if (time_is_before_jiffies(last + NVMET_PCIEP_SQ_POLL_IDLE))
			break;

		cpu_relax();
	}

	schedule_delayed_work(&ctrl->poll_sqs, NVMET_PCIEP_SQ_POLL_INTERVAL);
}

static void nvmet_pciep_cq_work(struct work_struct *work)
{
	struct nvmet_pciep_queue *cq =
		container_of(work, struct nvmet_pciep_queue, work.work);
	struct nvmet_pciep_ctrl *ctrl = cq->ctrl;
	struct nvme_completion *cqe;
	struct nvmet_pciep_iod *iod;
	unsigned long flags;
	int ret, n;

	ret = nvmet_pciep_map_queue(ctrl, cq);
	if (ret)
		goto again;

	spin_lock_irqsave(&cq->lock, flags);

	while (!list_empty(&cq->list)) {
		/* Check that the CQ is not full. */
		cq->head = nvmet_pciep_bar_read32(ctrl, cq->db);
		if (cq->head == cq->tail + 1) {
			ret = -EAGAIN;
			break;
		}

		iod = list_first_entry(&cq->list, struct nvmet_pciep_iod, link);
		list_del_init(&iod->link);

		if (!test_bit(NVMET_PCIEP_Q_LIVE, &cq->flags) ||
		    !ctrl->link_up) {
			nvmet_pciep_free_iod(iod);
			continue;
		}

		spin_unlock_irqrestore(&cq->lock, flags);

		/* Post the IOD completion entry. */
		cqe = &iod->cqe;
		cqe->status = cpu_to_le16((iod->status << 1) | cq->phase);

		dev_dbg(ctrl->dev,
			"CQ[%u]: %s status 0x%x, result 0x%llx, head %u, tail %u, phase %u\n",
			cq->qid, nvmet_pciep_iod_name(iod), iod->status,
			le64_to_cpu(cqe->result.u64), cq->head, cq->tail,
			cq->phase);

		memcpy_toio(cq->pci_map.virt_addr + cq->tail * cq->qes, cqe,
			    sizeof(struct nvme_completion));

		/* Advance the tail */
		cq->tail++;
		if (cq->tail >= cq->depth) {
			cq->tail = 0;
			cq->phase ^= 1;
		}

		nvmet_pciep_free_iod(iod);

		/* Signal the host. */
		nvmet_pciep_raise_irq(ctrl, cq, false);
		n++;

		spin_lock_irqsave(&cq->lock, flags);
	}

	spin_unlock_irqrestore(&cq->lock, flags);

	nvmet_pciep_unmap_queue(ctrl, cq);

	/*
	 * We do not support precise IRQ coalescing time (100ns units as per
	 * NVMe specifications). So if we have posted completion entries without
	 * reaching the interrupt coalescing threshold, raise an interrupt.
	 */
	if (n)
		nvmet_pciep_raise_irq(ctrl, cq, true);

again:
	if (ret < 0)
		queue_delayed_work(system_highpri_wq, &cq->work,
				   NVMET_PCIEP_CQ_RETRY_INTERVAL);
}

static int nvmet_pciep_epf_configure_bar(struct nvmet_pciep_epf *nvme_epf)
{
	struct pci_epf *epf = nvme_epf->epf;
	const struct pci_epc_features *epc_features = nvme_epf->epc_features;
	size_t reg_size, reg_bar_size;
	size_t msix_table_size = 0;

	/*
	 * The first free BAR will be our register BAR and per NVMe
	 * specifications, it must be BAR 0.
	 */
	if (pci_epc_get_first_free_bar(epc_features) != BAR_0) {
		dev_err(&epf->dev, "BAR 0 is not free\n");
		return -EINVAL;
	}

	/* Initialize BAR flags */
	if (epc_features->bar[BAR_0].only_64bit)
		epf->bar[BAR_0].flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;

	/*
	 * Calculate the size of the register bar: NVMe registers first with
	 * enough space for the doorbells, followed by the MSI-X table
	 * if supported.
	 */
	reg_size = NVME_REG_DBS + (NVMET_NR_QUEUES * 2 * sizeof(u32));
	reg_size = ALIGN(reg_size, 8);

	if (epc_features->msix_capable) {
		size_t pba_size;

		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		nvme_epf->msix_table_offset = reg_size;
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);

		reg_size += msix_table_size + pba_size;
	}

	reg_bar_size = ALIGN(reg_size, max(epc_features->align, 4096));

	if (epc_features->bar[BAR_0].type == BAR_FIXED) {
		if (reg_bar_size > epc_features->bar[BAR_0].fixed_size) {
			dev_err(&epf->dev,
				"Reg BAR 0 size %llu B too small, need %zu B\n",
				epc_features->bar[BAR_0].fixed_size,
				reg_bar_size);
			return -ENOMEM;
		}
		reg_bar_size = epc_features->bar[BAR_0].fixed_size;
	}

	nvme_epf->reg_bar = pci_epf_alloc_space(epf, reg_bar_size, BAR_0,
						epc_features, PRIMARY_INTERFACE);
	if (!nvme_epf->reg_bar) {
		dev_err(&epf->dev, "Allocate BAR 0 failed\n");
		return -ENOMEM;
	}
	memset(nvme_epf->reg_bar, 0, reg_bar_size);

	return 0;
}

static void nvmet_pciep_epf_clear_bar(struct nvmet_pciep_epf *nvme_epf)
{
	struct pci_epf *epf = nvme_epf->epf;

	pci_epc_clear_bar(epf->epc, epf->func_no, epf->vfunc_no,
			  &epf->bar[BAR_0]);
	pci_epf_free_space(epf, nvme_epf->reg_bar, BAR_0, PRIMARY_INTERFACE);
	nvme_epf->reg_bar = NULL;
}

static int nvmet_pciep_epf_init_irq(struct nvmet_pciep_epf *nvme_epf)
{
	const struct pci_epc_features *epc_features = nvme_epf->epc_features;
	struct pci_epf *epf = nvme_epf->epf;
	int ret;

	/* Enable MSI-X if supported, otherwise, use MSI */
	if (epc_features->msix_capable && epf->msix_interrupts) {
		ret = pci_epc_set_msix(epf->epc, epf->func_no, epf->vfunc_no,
				       epf->msix_interrupts, BAR_0,
				       nvme_epf->msix_table_offset);
		if (ret) {
			dev_err(&epf->dev, "MSI-X configuration failed\n");
			return ret;
		}

		nvme_epf->nr_vectors = epf->msix_interrupts;
		nvme_epf->irq_type = PCI_IRQ_MSIX;

		return 0;
	}

	if (epc_features->msi_capable && epf->msi_interrupts) {
		ret = pci_epc_set_msi(epf->epc, epf->func_no, epf->vfunc_no,
				      epf->msi_interrupts);
		if (ret) {
			dev_err(&epf->dev, "MSI configuration failed\n");
			return ret;
		}

		nvme_epf->nr_vectors = epf->msi_interrupts;
		nvme_epf->irq_type = PCI_IRQ_MSI;

		return 0;
	}

	/* MSI and MSI-X are not supported: fall back to INTX */
	nvme_epf->nr_vectors = 1;
	nvme_epf->irq_type = PCI_IRQ_INTX;

	return 0;
}

static int nvmet_pciep_epf_epc_init(struct pci_epf *epf)
{
	struct nvmet_pciep_epf *nvme_epf = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features = nvme_epf->epc_features;
	struct nvmet_pciep_ctrl *ctrl = &nvme_epf->ctrl;
	unsigned int max_nr_queues = NVMET_NR_QUEUES;
	int ret;

	/*
	 * Cap the maximum number of queues we can support on the controller
	 * with the number of IRQs we can use.
	 */
	if (epc_features->msix_capable && epf->msix_interrupts) {
		dev_info(&epf->dev,
			 "PCI endpoint controller supports MSI-X, %u vectors\n",
			 epf->msix_interrupts);
		max_nr_queues = min(max_nr_queues, epf->msix_interrupts);
	} else if (epc_features->msi_capable && epf->msi_interrupts) {
		dev_info(&epf->dev,
			 "PCI endpoint controller supports MSI, %u vectors\n",
			 epf->msi_interrupts);
		max_nr_queues = min(max_nr_queues, epf->msi_interrupts);
	}

	if (max_nr_queues < 2) {
		dev_err(&epf->dev, "Invalid maximum number of queues %u\n",
			max_nr_queues);
		return -EINVAL;
	}

	/* Create the target controller. */
	ret = nvmet_pciep_create_ctrl(nvme_epf, max_nr_queues);
	if (ret) {
		dev_err(&epf->dev,
			"Create NVMe PCI target controller failed\n");
		return ret;
	}

	if (epf->vfunc_no <= 1) {
		/* Set device ID, class, etc */
		epf->header->vendorid = ctrl->tctrl->subsys->vendor_id;
		epf->header->subsys_vendor_id =
			ctrl->tctrl->subsys->subsys_vendor_id;
		ret = pci_epc_write_header(epf->epc, epf->func_no, epf->vfunc_no,
					   epf->header);
		if (ret) {
			dev_err(&epf->dev,
				"Write configuration header failed %d\n", ret);
			goto out_destroy_ctrl;
		}
	}

	/* Setup the PCIe BAR and create the controller */
	ret = pci_epc_set_bar(epf->epc, epf->func_no, epf->vfunc_no,
			      &epf->bar[BAR_0]);
	if (ret) {
		dev_err(&epf->dev, "Set BAR 0 failed\n");
		goto out_destroy_ctrl;
	}

	/*
	 * Enable interrupts and start polling the controller BAR if we do not
	 * have any link up notifier.
	 */
	ret = nvmet_pciep_epf_init_irq(nvme_epf);
	if (ret)
		goto out_clear_bar;

	if (!epc_features->linkup_notifier) {
		ctrl->link_up = true;
		nvmet_pciep_start_ctrl(&nvme_epf->ctrl);
	}

	return 0;

out_clear_bar:
	nvmet_pciep_epf_clear_bar(nvme_epf);
out_destroy_ctrl:
	nvmet_pciep_destroy_ctrl(&nvme_epf->ctrl);
	return ret;
}

static void nvmet_pciep_epf_epc_deinit(struct pci_epf *epf)
{
	struct nvmet_pciep_epf *nvme_epf = epf_get_drvdata(epf);
	struct nvmet_pciep_ctrl *ctrl = &nvme_epf->ctrl;

	ctrl->link_up = false;
	nvmet_pciep_destroy_ctrl(ctrl);

	nvmet_pciep_epf_deinit_dma(nvme_epf);
	nvmet_pciep_epf_clear_bar(nvme_epf);

	mutex_destroy(&nvme_epf->mmio_lock);
}

static int nvmet_pciep_epf_link_up(struct pci_epf *epf)
{
	struct nvmet_pciep_epf *nvme_epf = epf_get_drvdata(epf);
	struct nvmet_pciep_ctrl *ctrl = &nvme_epf->ctrl;

	dev_info(nvme_epf->ctrl.dev, "PCI link up\n");

	ctrl->link_up = true;
	nvmet_pciep_start_ctrl(ctrl);

	return 0;
}

static int nvmet_pciep_epf_link_down(struct pci_epf *epf)
{
	struct nvmet_pciep_epf *nvme_epf = epf_get_drvdata(epf);
	struct nvmet_pciep_ctrl *ctrl = &nvme_epf->ctrl;

	dev_info(nvme_epf->ctrl.dev, "PCI link down\n");

	ctrl->link_up = false;
	nvmet_pciep_stop_ctrl(ctrl);

	return 0;
}

static const struct pci_epc_event_ops nvmet_pciep_epf_event_ops = {
	.epc_init = nvmet_pciep_epf_epc_init,
	.epc_deinit = nvmet_pciep_epf_epc_deinit,
	.link_up = nvmet_pciep_epf_link_up,
	.link_down = nvmet_pciep_epf_link_down,
};

static int nvmet_pciep_epf_bind(struct pci_epf *epf)
{
	struct nvmet_pciep_epf *nvme_epf = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc = epf->epc;
	bool dma_supported;
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
	nvme_epf->epc_features = epc_features;

	ret = nvmet_pciep_epf_configure_bar(nvme_epf);
	if (ret)
		return ret;

	if (nvme_epf->dma_enable) {
		dma_supported = nvmet_pciep_epf_init_dma(nvme_epf);
		if (!dma_supported) {
			dev_info(&epf->dev,
				 "DMA not supported, falling back to mmio\n");
			nvme_epf->dma_enable = false;
		}
	} else {
		dev_info(&epf->dev, "DMA disabled\n");
	}

	return 0;
}

static void nvmet_pciep_epf_unbind(struct pci_epf *epf)
{
	struct nvmet_pciep_epf *nvme_epf = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	nvmet_pciep_destroy_ctrl(&nvme_epf->ctrl);

	if (epc->init_complete) {
		nvmet_pciep_epf_deinit_dma(nvme_epf);
		nvmet_pciep_epf_clear_bar(nvme_epf);
	}
}

static struct pci_epf_header nvme_epf_pci_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.progif_code	= 0x02, /* NVM Express */
	.baseclass_code = PCI_BASE_CLASS_STORAGE,
	.subclass_code	= 0x08, /* Non-Volatile Memory controller */
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static int nvmet_pciep_epf_probe(struct pci_epf *epf,
			      const struct pci_epf_device_id *id)
{
	struct nvmet_pciep_epf *nvme_epf;

	nvme_epf = devm_kzalloc(&epf->dev, sizeof(*nvme_epf), GFP_KERNEL);
	if (!nvme_epf)
		return -ENOMEM;

	nvme_epf->epf = epf;
	mutex_init(&nvme_epf->mmio_lock);

	/* Set default attribute values */
	nvme_epf->dma_enable = true;
	nvme_epf->mdts_kb = NVMET_PCIEP_MDTS_KB;

	epf->event_ops = &nvmet_pciep_epf_event_ops;
	epf->header = &nvme_epf_pci_header;
	epf_set_drvdata(epf, nvme_epf);

	return 0;
}

#define to_nvme_epf(epf_group)	\
	container_of(epf_group, struct nvmet_pciep_epf, group)

static ssize_t nvmet_pciep_epf_dma_enable_show(struct config_item *item,
					    char *page)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pciep_epf *nvme_epf = to_nvme_epf(group);

	return sysfs_emit(page, "%d\n", nvme_epf->dma_enable);
}

static ssize_t nvmet_pciep_epf_dma_enable_store(struct config_item *item,
					     const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pciep_epf *nvme_epf = to_nvme_epf(group);
	int ret;

	if (nvme_epf->ctrl.tctrl)
		return -EBUSY;

	ret = kstrtobool(page, &nvme_epf->dma_enable);
	if (ret)
		return ret;

	return len;
}

CONFIGFS_ATTR(nvmet_pciep_epf_, dma_enable);

static ssize_t nvmet_pciep_epf_portid_show(struct config_item *item, char *page)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pciep_epf *nvme_epf = to_nvme_epf(group);

	return sysfs_emit(page, "%u\n", le16_to_cpu(nvme_epf->portid));
}

static ssize_t nvmet_pciep_epf_portid_store(struct config_item *item,
					  const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pciep_epf *nvme_epf = to_nvme_epf(group);
	u16 portid;

	/* Do not allow setting this when the function is already started */
	if (nvme_epf->ctrl.tctrl)
		return -EBUSY;

	if (!len)
		return -EINVAL;

	if (kstrtou16(page, 0, &portid))
		return -EINVAL;

	nvme_epf->portid = cpu_to_le16(portid);

	return len;
}

CONFIGFS_ATTR(nvmet_pciep_epf_, portid);

static ssize_t nvmet_pciep_epf_subsysnqn_show(struct config_item *item,
					      char *page)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pciep_epf *nvme_epf = to_nvme_epf(group);

	return sysfs_emit(page, "%s\n", nvme_epf->subsysnqn);
}

static ssize_t nvmet_pciep_epf_subsysnqn_store(struct config_item *item,
					       const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pciep_epf *nvme_epf = to_nvme_epf(group);

	/* Do not allow setting this when the function is already started */
	if (nvme_epf->ctrl.tctrl)
		return -EBUSY;

	if (!len)
		return -EINVAL;

	strscpy(nvme_epf->subsysnqn, page, len);

	return len;
}

CONFIGFS_ATTR(nvmet_pciep_epf_, subsysnqn);

static ssize_t nvmet_pciep_epf_mdts_kb_show(struct config_item *item,
					    char *page)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pciep_epf *nvme_epf = to_nvme_epf(group);

	return sysfs_emit(page, "%u\n", nvme_epf->mdts_kb);
}

static ssize_t nvmet_pciep_epf_mdts_kb_store(struct config_item *item,
					     const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pciep_epf *nvme_epf = to_nvme_epf(group);
	unsigned long mdts_kb;
	int ret;

	if (nvme_epf->ctrl.tctrl)
		return -EBUSY;

	ret = kstrtoul(page, 0, &mdts_kb);
	if (ret)
		return ret;
	if (!mdts_kb)
		mdts_kb = NVMET_PCIEP_MDTS_KB;
	else if (mdts_kb > NVMET_PCIEP_MAX_MDTS_KB)
		mdts_kb = NVMET_PCIEP_MAX_MDTS_KB;

	if (!is_power_of_2(mdts_kb))
		return -EINVAL;

	nvme_epf->mdts_kb = mdts_kb;

	return len;
}

CONFIGFS_ATTR(nvmet_pciep_epf_, mdts_kb);

static struct configfs_attribute *nvmet_pciep_epf_attrs[] = {
	&nvmet_pciep_epf_attr_dma_enable,
	&nvmet_pciep_epf_attr_portid,
	&nvmet_pciep_epf_attr_subsysnqn,
	&nvmet_pciep_epf_attr_mdts_kb,
	NULL,
};

static const struct config_item_type nvmet_pciep_epf_group_type = {
	.ct_attrs	= nvmet_pciep_epf_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_group *nvmet_pciep_epf_add_cfs(struct pci_epf *epf,
						 struct config_group *group)
{
	struct nvmet_pciep_epf *nvme_epf = epf_get_drvdata(epf);

	/* Add the NVMe target attributes */
	config_group_init_type_name(&nvme_epf->group, "nvme",
				    &nvmet_pciep_epf_group_type);

	return &nvme_epf->group;
}

static const struct pci_epf_device_id nvmet_pciep_epf_ids[] = {
	{ .name = "nvmet_pciep" },
	{},
};

static struct pci_epf_ops nvmet_pciep_epf_ops = {
	.bind	= nvmet_pciep_epf_bind,
	.unbind	= nvmet_pciep_epf_unbind,
	.add_cfs = nvmet_pciep_epf_add_cfs,
};

static struct pci_epf_driver nvmet_pciep_epf_driver = {
	.driver.name	= "nvmet_pciep",
	.probe		= nvmet_pciep_epf_probe,
	.id_table	= nvmet_pciep_epf_ids,
	.ops		= &nvmet_pciep_epf_ops,
	.owner		= THIS_MODULE,
};

static int __init nvmet_pciep_init_module(void)
{
	int ret;

	ret = pci_epf_register_driver(&nvmet_pciep_epf_driver);
	if (ret)
		return ret;

	ret = nvmet_register_transport(&nvmet_pciep_fabrics_ops);
	if (ret) {
		pci_epf_unregister_driver(&nvmet_pciep_epf_driver);
		return ret;
	}

	return 0;
}

static void __exit nvmet_pciep_cleanup_module(void)
{
	nvmet_unregister_transport(&nvmet_pciep_fabrics_ops);
	pci_epf_unregister_driver(&nvmet_pciep_epf_driver);
}

module_init(nvmet_pciep_init_module);
module_exit(nvmet_pciep_cleanup_module);

MODULE_DESCRIPTION("NVMe PCI endpoint function driver");
MODULE_AUTHOR("Damien Le Moal <dlemoal@kernel.org>");
MODULE_LICENSE("GPL v2");
