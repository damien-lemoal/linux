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
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvme.h>
#include <linux/pci_ids.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>
#include <linux/slab.h>

#include "nvmet.h"
#include "../host/nvme.h"
#include "../host/fabrics.h"

static LIST_HEAD(nvmet_pci_ports);
static DEFINE_MUTEX(nvmet_pci_ports_mutex);

static LIST_HEAD(nvmet_pci_ctrl_list);
static DEFINE_MUTEX(nvmet_pci_ctrl_mutex);

static struct kmem_cache *nvmet_pci_iod_cache;

/*
 * Default maximum data transfer size: limit to 128 KB to avoid
 * excessive local memory use for buffers.
 */
#define NVMET_PCI_EPF_MDTS_KB		128
#define NVMET_PCI_EPF_MAX_MDTS_KB	1024

enum nvmet_pci_queue_flags {
	/* The queue is a submission queue */
	NVMET_PCI_Q_IS_SQ	= 0,
	/* The queue is live (PCI side) */
	NVMET_PCI_Q_LIVE,
	/* IRQ are enabled for this queue (PCI side) */
	NVMET_PCI_Q_IRQ_ENABLED,
};

struct nvmet_pci_queue {
	union {
		struct nvmet_sq	nvme_sq;
		struct nvmet_cq	nvme_cq;
	};
	struct nvmet_pci_ctrl	*ctrl;
	unsigned long		flags;

	phys_addr_t		pci_addr;
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

	struct workqueue_struct	*cmd_wq;
	struct delayed_work	work;
	spinlock_t		lock;
	struct list_head	list;
};

/*
 * PCI memory segments for admin and IO commands.
 */
struct nvmet_pci_segment {
	phys_addr_t		pci_addr;
	size_t			size;
};

/*
 * Command descriptors.
 */
struct nvmet_pci_iod {
	struct nvmet_req	req;
	struct nvme_command	cmd;
	struct nvme_completion	cqe;
	unsigned int		status;

	struct nvmet_pci_ctrl	*ctrl;

	struct nvmet_pci_queue	*sq;
	struct nvmet_pci_queue	*cq;

	struct list_head	link;

	/* Internal buffer that we will transfer over PCI */
	size_t			buffer_size;
	void			*buffer;
	struct scatterlist 	buffer_sgl;
	enum dma_data_direction	dma_dir;

	/*
	 * Host PCI address segments: if nr_segs is 1, we use only @seg,
	 * otherwise, the array of segments @segs is allocated to represent
	 * multiple segments.
	 */
	unsigned int		nr_segs;
	struct nvmet_pci_segment seg;
	struct nvmet_pci_segment *segs;

	struct work_struct	work;

	struct completion	done;
};

struct nvmet_pci_ctrl {
	struct nvmet_pci_epf	*nvme_epf;
	struct device		*dev;

	struct list_head	list;
	struct nvmet_port	*port;
	struct nvmet_subsys	*subsys;
	uuid_t			hostid;
	struct nvmet_ctrl	*tctrl;

	unsigned int		nr_queues;
	struct nvmet_pci_queue	*sq;
	struct nvmet_pci_queue	*cq;

	void			*bar;
	u64			cap;
	u32			vs;
	u32			cc;
	u32			csts;

	size_t			io_sqes;
	size_t			io_cqes;

	size_t			mps_shift;
	size_t			mps;
	size_t			mps_mask;

	unsigned int		mdts;

	struct delayed_work	poll_cc;

	struct workqueue_struct	*wq;

	bool			enabled;
};

/*
 * PCI EPF driver private data.
 */
struct nvmet_pci_epf {
	struct pci_epf			*epf;

	const struct pci_epc_features	*epc_features;

	void				*reg_bar;
	size_t				msix_table_offset;

	unsigned int			irq_type;
	unsigned int			nr_vectors;

	struct nvmet_pci_ctrl		ctrl;

	struct dma_chan			*dma_chan_tx;
	struct dma_chan			*dma_chan_rx;
	struct mutex			xfer_lock;

	struct mutex			irq_lock;

	/* PCI endpoint function configfs attributes */
	struct config_group		group;
	bool				dma_enable;
	char				subsysnqn[NVMF_NQN_SIZE];
	unsigned int			mdts_kb;
};

static inline u32 nvmet_pci_bar_read32(struct nvmet_pci_ctrl *ctrl, u32 off)
{
	__le32 *bar_reg = ctrl->bar + off;

	return le32_to_cpu(READ_ONCE(*bar_reg));
}

static inline void nvmet_pci_bar_write32(struct nvmet_pci_ctrl *ctrl,
					u32 off, u32 val)
{
	__le32 *bar_reg = ctrl->bar + off;

	WRITE_ONCE(*bar_reg, cpu_to_le32(val));
}

static inline u64 nvmet_pci_bar_read64(struct nvmet_pci_ctrl *ctrl, u32 off)
{
	return (u64)nvmet_pci_bar_read32(ctrl, off ) |
		((u64)nvmet_pci_bar_read32(ctrl, off + 4) << 32);
}

static inline void nvmet_pci_bar_write64(struct nvmet_pci_ctrl *ctrl,
					u32 off, u64 val)
{
	nvmet_pci_bar_write32(ctrl, off , val & 0xFFFFFFFF);
	nvmet_pci_bar_write32(ctrl, off + 4, (val >> 32) & 0xFFFFFFFF);
}

static inline int nvmet_pci_epf_mem_map(struct nvmet_pci_epf *nvme_epf,
					phys_addr_t pci_addr, size_t size,
					struct pci_epc_map *map)
{
	struct pci_epf *epf = nvme_epf->epf;

	return pci_epc_mem_map(epf->epc, epf->func_no, epf->vfunc_no,
			       pci_addr, size, map);
}

static inline void nvmet_pci_epf_mem_unmap(struct nvmet_pci_epf *nvme_epf,
					   struct pci_epc_map *map)
{
	struct pci_epf *epf = nvme_epf->epf;

	pci_epc_mem_unmap(epf->epc, epf->func_no, epf->vfunc_no, map);
}

struct nvmet_pci_epf_dma_filter {
	struct device *dev;
	u32 dma_mask;
};

static bool nvmet_pci_epf_dma_filter(struct dma_chan *chan, void *arg)
{
	struct nvmet_pci_epf_dma_filter *filter = arg;
	struct dma_slave_caps caps;

	memset(&caps, 0, sizeof(caps));
	dma_get_slave_caps(chan, &caps);

	return chan->device->dev == filter->dev &&
		(filter->dma_mask & caps.directions);
}

static bool nvmet_pci_epf_init_dma(struct nvmet_pci_epf *nvme_epf)
{
	struct pci_epf *epf = nvme_epf->epf;
	struct device *dev = &epf->dev;
	struct nvmet_pci_epf_dma_filter filter;
	struct dma_chan *chan;
	dma_cap_mask_t mask;

	mutex_init(&nvme_epf->xfer_lock);
	mutex_init(&nvme_epf->irq_lock);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	filter.dev = epf->epc->dev.parent;
	filter.dma_mask = BIT(DMA_DEV_TO_MEM);

	chan = dma_request_channel(mask, nvmet_pci_epf_dma_filter, &filter);
	if (!chan)
		return false;

	nvme_epf->dma_chan_rx = chan;

	filter.dma_mask = BIT(DMA_MEM_TO_DEV);
	chan = dma_request_channel(mask, nvmet_pci_epf_dma_filter, &filter);
	if (!chan) {
		dma_release_channel(nvme_epf->dma_chan_rx);
		nvme_epf->dma_chan_rx = NULL;
		return false;
	}

	nvme_epf->dma_chan_tx = chan;

	dev_dbg(dev, "DMA RX channel %s, maximum segment size %u B\n",
		dma_chan_name(nvme_epf->dma_chan_rx),
		dma_get_max_seg_size(nvme_epf->dma_chan_rx->device->dev));
	dev_dbg(dev, "DMA TX channel %s, maximum segment size %u B\n",
		dma_chan_name(nvme_epf->dma_chan_tx),
		dma_get_max_seg_size(nvme_epf->dma_chan_tx->device->dev));

	return true;
}

static void nvmet_pci_epf_deinit_dma(struct nvmet_pci_epf *nvme_epf)
{
	if (nvme_epf->dma_chan_tx) {
		dma_release_channel(nvme_epf->dma_chan_tx);
		nvme_epf->dma_chan_tx = NULL;
	}

	if (nvme_epf->dma_chan_rx) {
		dma_release_channel(nvme_epf->dma_chan_rx);
		nvme_epf->dma_chan_rx = NULL;
	}
}

static void nvmet_pci_epf_dma_callback(void *param)
{
	complete(param);
}

static ssize_t nvmet_pci_epf_dma_transfer(struct nvmet_pci_epf *nvme_epf,
					  struct nvmet_pci_segment *seg,
					  enum dma_data_direction dir, void *buf)
{
	struct pci_epf *epf = nvme_epf->epf;
	struct device *dma_dev = epf->epc->dev.parent;
	struct dma_async_tx_descriptor *desc;
	DECLARE_COMPLETION_ONSTACK(complete);
	struct dma_slave_config sconf = {};
	struct device *dev = &epf->dev;
	struct dma_chan *chan;
	phys_addr_t dma_addr;
	dma_cookie_t cookie;
	ssize_t ret;

	switch (dir) {
	case DMA_FROM_DEVICE:
		chan = nvme_epf->dma_chan_rx;
		sconf.direction = DMA_DEV_TO_MEM;
		sconf.src_addr = seg->pci_addr;
		break;
	case DMA_TO_DEVICE:
		chan = nvme_epf->dma_chan_tx;
		sconf.direction = DMA_MEM_TO_DEV;
		sconf.dst_addr = seg->pci_addr;
		break;
	default:
		return -EINVAL;
	}

	dma_addr = dma_map_single(dma_dev, buf, seg->size, dir);
	ret = dma_mapping_error(dma_dev, dma_addr);
	if (ret)
		return ret;

	ret = dmaengine_slave_config(chan, &sconf);
	if (ret) {
		dev_err(dev, "Failed to configure DMA channel\n");
		goto unmap;
	}

	desc = dmaengine_prep_slave_single(chan, dma_addr,
					   seg->size, sconf.direction,
					   DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(dev, "Failed to prepare DMA\n");
		ret = -EIO;
		goto unmap;
	}

	desc->callback = nvmet_pci_epf_dma_callback;
	desc->callback_param = &complete;

	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(dev, "DMA submit failed %zd\n", ret);
		goto unmap;
	}

	dma_async_issue_pending(chan);
	ret = wait_for_completion_timeout(&complete, msecs_to_jiffies(1000));
	if (!ret) {
		dev_err(dev, "DMA transfer timeout\n");
		dmaengine_terminate_sync(chan);
		ret = -ETIMEDOUT;
		goto unmap;
	}

	ret = seg->size;

unmap:
	dma_unmap_single(dma_dev, dma_addr, seg->size, dir);

	return ret;
}

static ssize_t nvmet_pci_epf_mmio_transfer(struct nvmet_pci_epf *nvme_epf,
					  struct nvmet_pci_segment *seg,
					  enum dma_data_direction dir,
					  void *buf)
{
	struct pci_epc_map map;
	ssize_t ret;

	ret = nvmet_pci_epf_mem_map(nvme_epf, seg->pci_addr, seg->size, &map);
	if (ret)
		return ret;

	switch (dir) {
	case DMA_FROM_DEVICE:
		memcpy_fromio(buf, map.virt_addr, map.pci_size);
		ret = map.pci_size;
		break;
	case DMA_TO_DEVICE:
		memcpy_toio(map.virt_addr, buf, map.pci_size);
		ret = map.pci_size;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	nvmet_pci_epf_mem_unmap(nvme_epf, &map);

	return ret;
}

static int nvmet_pci_epf_transfer(struct nvmet_pci_epf *nvme_epf,
				  struct nvmet_pci_segment *seg,
				  enum dma_data_direction dir, void *buf,
				  bool allow_dma)
{
	size_t size = seg->size;
	ssize_t ret;

	while (size) {
		/*
		 * Note: mmio transfers do not need serialization but this is a
		 * nice way to avoid using too many mapping windows.
		 */
		mutex_lock(&nvme_epf->xfer_lock);
		if (allow_dma && nvme_epf->dma_enable)
			ret = nvmet_pci_epf_dma_transfer(nvme_epf, seg,
							 dir, buf);
		else
			ret = nvmet_pci_epf_mmio_transfer(nvme_epf, seg,
							  dir, buf);
		mutex_unlock(&nvme_epf->xfer_lock);
		if (!ret)
			return -EIO;
		if (ret < 0)
			return ret;

		size -= ret;
		buf += ret;
	}

	return 0;
}

static void nvmet_pci_epf_raise_irq(struct nvmet_pci_epf *nvme_epf,
				   struct nvmet_pci_queue *cq)
{
	struct pci_epf *epf = nvme_epf->epf;
	int ret;

	if (!test_bit(NVMET_PCI_Q_LIVE, &cq->flags) ||
	    !test_bit(NVMET_PCI_Q_IRQ_ENABLED, &cq->flags))
		return;

	mutex_lock(&nvme_epf->irq_lock);

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
		dev_err(&epf->dev, "Raise IRQ failed %d\n", ret);

	mutex_unlock(&nvme_epf->irq_lock);
}

static int nvmet_pci_epf_map_queue(struct nvmet_pci_epf *nvme_epf,
				  struct nvmet_pci_queue *queue)
{
	struct pci_epf *epf = nvme_epf->epf;
	int ret;

	ret = nvmet_pci_epf_mem_map(nvme_epf, queue->pci_addr,
				    queue->pci_size, &queue->pci_map);
	if (ret) {
		dev_err(&epf->dev, "Map %cQ %d failed %d\n",
			test_bit(NVMET_PCI_Q_IS_SQ, &queue->flags) ? 'S' : 'C',
			queue->qid, ret);
		return ret;
	}

	if (queue->pci_map.pci_size < queue->pci_size) {
		dev_err(&epf->dev, "Partial %cQ %d mapping\n",
			test_bit(NVMET_PCI_Q_IS_SQ, &queue->flags) ? 'S' : 'C',
			queue->qid);
		nvmet_pci_epf_mem_unmap(nvme_epf, &queue->pci_map);
		return -ENOMEM;
	}

	return 0;
}

static inline void nvmet_pci_epf_unmap_queue(struct nvmet_pci_epf *nvme_epf,
					    struct nvmet_pci_queue *q)
{
	nvmet_pci_epf_mem_unmap(nvme_epf, &q->pci_map);
}

static const char *nvmet_pci_iod_name(struct nvmet_pci_iod *iod)
{
	u8 opcode = iod->cmd.common.opcode;

	if (iod->sq->qid)
		return nvme_get_opcode_str(opcode);
	return nvme_get_admin_opcode_str(opcode);
}

static void nvmet_pci_exec_iod_work(struct work_struct *work);

static struct nvmet_pci_iod *nvmet_pci_alloc_iod(struct nvmet_pci_queue *sq)
{
	struct nvmet_pci_ctrl *ctrl = sq->ctrl;
	struct nvmet_pci_iod *iod;

	iod = kmem_cache_alloc(nvmet_pci_iod_cache, GFP_KERNEL);
	if (likely(iod)) {
		iod->req.cmd = &iod->cmd;
		iod->req.cqe = &iod->cqe;
		iod->req.port = ctrl->port;
		iod->status = NVME_SC_SUCCESS;
		iod->ctrl = ctrl;
		iod->sq = sq;
		iod->cq = &ctrl->cq[sq->qid];
		INIT_LIST_HEAD(&iod->link);
		iod->buffer_size = 0;
		iod->buffer = NULL;
		iod->dma_dir = DMA_NONE;
		iod->nr_segs = 0;
		iod->segs = NULL;
		INIT_WORK(&iod->work, nvmet_pci_exec_iod_work);
		init_completion(&iod->done);
	}

	return iod;
}

static int nvmet_pci_alloc_iod_buffer(struct nvmet_pci_iod *iod)
{
	void *buffer;

	buffer = kvmalloc(iod->buffer_size, GFP_KERNEL);
	if (!buffer) {
		iod->buffer_size = 0;
		return -ENOMEM;
	}

	if (!iod->sq->qid)
		memset(buffer, 0, iod->buffer_size);
	iod->buffer = buffer;

	return 0;
}

static int nvmet_pci_alloc_iod_segs(struct nvmet_pci_iod *iod, int nr_segs)
{
	struct nvmet_pci_segment *segs;

	/* Single segment case: use the command embedded structure */
	if (nr_segs == 1) {
		iod->segs = &iod->seg;
		iod->nr_segs = 1;
		return 0;
	}

	/* More than one segment needed: allocate an array */
	segs = kcalloc(nr_segs, sizeof(struct nvmet_pci_segment), GFP_KERNEL);
	if (!segs)
		return -ENOMEM;

	iod->nr_segs = nr_segs;
	iod->segs = segs;

	return 0;
}

static void nvmet_pci_free_iod(struct nvmet_pci_iod *iod)
{
	if (iod->req.ops)
		nvmet_req_uninit(&iod->req);

	kvfree(iod->buffer);
	if (iod->segs && iod->segs != &iod->seg)
		kfree(iod->segs);
	kmem_cache_free(nvmet_pci_iod_cache, iod);
}

static int nvmet_pci_transfer_iod_data(struct nvmet_pci_iod *iod)
{
	struct nvmet_pci_epf *nvme_epf = iod->ctrl->nvme_epf;
	struct nvmet_pci_segment *seg;
	void *buf = iod->buffer;
	size_t size = 0;
	int i, ret;

	WARN_ON(!iod->nr_segs);

	/* Transfer each segment of the command */
	for (i = 0; i < iod->nr_segs; i++) {
		seg = &iod->segs[i];

		if (size >= iod->buffer_size)
			goto xfer_err;

		ret = nvmet_pci_epf_transfer(nvme_epf, seg, iod->dma_dir,
					     buf, true);
		if (ret)
			goto xfer_err;

		buf += seg->size;
		size += seg->size;
	}

	return 0;

xfer_err:
	iod->status = NVME_SC_DATA_XFER_ERROR | NVME_STATUS_DNR;
	return -EIO;
}

static void nvmet_pci_complete_iod(struct nvmet_pci_iod *iod)
{
	struct nvmet_pci_ctrl *ctrl = iod->ctrl;
	struct nvmet_pci_queue *cq = iod->cq;
	unsigned long flags;

	if (!test_bit(NVMET_PCI_Q_LIVE, &cq->flags)) {
		nvmet_pci_free_iod(iod);
		return;
	}

	if (iod->status)
		dev_err(iod->ctrl->dev,
			"CQ[%d]: Command %s (0x%x) failed, status 0x%0x\n",
			iod->sq->qid, nvmet_pci_iod_name(iod),
			iod->cmd.common.opcode, iod->status);

	/*
	 * Add the command to the list of completed commands for the
	 * cq and schedule the list processing.
	 */
	spin_lock_irqsave(&cq->lock, flags);
	list_add_tail(&iod->link, &cq->list);
	queue_delayed_work(ctrl->wq, &cq->work, 0);
	spin_unlock_irqrestore(&cq->lock, flags);
}

#define nvmet_pci_prp_addr(ctrl, prp)	((prp) & ~(ctrl)->mps_mask)
#define nvmet_pci_prp_ofst(ctrl, prp)	((prp) & (ctrl)->mps_mask)
#define nvmet_pci_prp_size(ctrl, prp)	\
	((size_t)((ctrl)->mps - nvmet_pci_prp_ofst(ctrl, prp)))

/*
 * Transfer a prp list from the host and return the number of prps.
 */
static int nvmet_pci_get_prp_list(struct nvmet_pci_ctrl *ctrl, u64 prp,
				 size_t xfer_len, __le64 *prps)
{
	size_t nr_prps = (xfer_len + ctrl->mps_mask) >> ctrl->mps_shift;
	struct nvmet_pci_segment seg;
	int ret;

	/*
	 * Compute the number of PRPs required for the number of bytes to
	 * transfer (xfer_len). If this number overflows the memory page size
	 * with the PRP list pointer specified, only return the space available
	 * in the memory page, the last PRP in there will be a PRP list pointer
	 * to the remaining PRPs.
	 */
	seg.pci_addr = prp;
	seg.size = min(nvmet_pci_prp_size(ctrl, prp), nr_prps << 3);
	ret = nvmet_pci_epf_transfer(ctrl->nvme_epf, &seg, DMA_FROM_DEVICE,
				     prps, false);
	if (ret)
		return ret;

	return seg.size >> 3;
}

static int nvmet_pci_iod_parse_prp_list(struct nvmet_pci_ctrl *ctrl,
				       struct nvmet_pci_iod *iod)
{
	struct nvme_command *cmd = &iod->cmd;
	struct nvmet_pci_segment *seg;
	size_t size = 0, ofst, prp_size, xfer_len;
	size_t transfer_len = iod->buffer_size;
	int nr_segs, nr_prps = 0;
	phys_addr_t pci_addr;
	int i = 0, ret;
	__le64 *prps;
	u64 prp;

	prps = kzalloc(ctrl->mps, GFP_KERNEL);
	if (!prps)
		goto internal;

	/*
	 * Allocate segments for the command: this considers the worst case
	 * scenario where all prps are discontiguous, so get as many segments
	 * as we can have prps. In practice, most of the time, we will have
	 * far less segments than prps.
	 */
	prp = le64_to_cpu(cmd->common.dptr.prp1);
	if (!prp)
		goto invalid_field;

	ofst = nvmet_pci_prp_ofst(ctrl, prp);
	nr_segs = (transfer_len + ofst + ctrl->mps - 1) >> ctrl->mps_shift;

	ret = nvmet_pci_alloc_iod_segs(iod, nr_segs);
	if (ret)
		goto internal;

	/* Set the first segment using prp1 */
	seg = &iod->segs[0];
	seg->pci_addr = prp;
	seg->size = nvmet_pci_prp_size(ctrl, prp);

	size = seg->size;
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
			nr_prps = nvmet_pci_get_prp_list(ctrl, prp,
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
		if (nvmet_pci_prp_ofst(ctrl, prp))
			goto invalid_offset;

		if (prp != pci_addr) {
			/* Discontiguous prp: new segment */
			nr_segs++;
			if (WARN_ON_ONCE(nr_segs > iod->nr_segs))
				goto internal;

			seg++;
			seg->pci_addr = prp;
			seg->size = 0;
			pci_addr = prp;
		}

		prp_size = min_t(size_t, ctrl->mps, xfer_len);
		seg->size += prp_size;
		pci_addr += prp_size;
		size += prp_size;

		i++;
	}

	iod->nr_segs = nr_segs;
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

static int nvmet_pci_iod_parse_prp_simple(struct nvmet_pci_ctrl *ctrl,
					 struct nvmet_pci_iod *iod)
{
	struct nvme_command *cmd = &iod->cmd;
	size_t transfer_len = iod->buffer_size;
	int ret, nr_segs = 1;
	u64 prp1, prp2 = 0;
	size_t prp1_size;

	/* prp1 */
	prp1 = le64_to_cpu(cmd->common.dptr.prp1);
	prp1_size = nvmet_pci_prp_size(ctrl, prp1);

	/* For commands crossing a page boundary, we should have a valid prp2 */
	if (transfer_len > prp1_size) {
		prp2 = le64_to_cpu(cmd->common.dptr.prp2);
		if (!prp2)
			goto invalid_field;
		if (nvmet_pci_prp_ofst(ctrl, prp2))
			goto invalid_offset;
		if (prp2 != prp1 + prp1_size)
			nr_segs = 2;
	}

	/* Create segments using the prps */
	ret = nvmet_pci_alloc_iod_segs(iod, nr_segs);
	if (ret)
		goto internal;

	iod->segs[0].pci_addr = prp1;
	if (nr_segs == 1) {
		iod->segs[0].size = transfer_len;
	} else {
		iod->segs[0].size = prp1_size;
		iod->segs[1].pci_addr = prp2;
		iod->segs[1].size = transfer_len - prp1_size;
	}

	return 0;

invalid_offset:
	dev_err(ctrl->dev, "PRPs simple invalid offset\n");
	iod->status = NVME_SC_PRP_INVALID_OFFSET | NVME_STATUS_DNR;
	return -EINVAL;

invalid_field:
	dev_err(ctrl->dev, "PRPs simple invalid field\n");
	iod->status = NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	return -EINVAL;

internal:
	dev_err(ctrl->dev, "PRPs simple internal error\n");
	iod->status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
	return ret;
}

static int nvmet_pci_iod_parse_dptr(struct nvmet_pci_iod *iod)
{
	struct nvmet_pci_ctrl *ctrl = iod->ctrl;
	struct nvme_command *cmd = &iod->cmd;
	u64 prp1 = le64_to_cpu(cmd->common.dptr.prp1);
	size_t ofst;
	int ret;

	if (iod->buffer_size > ctrl->mdts)
		goto invalid_field;

	/* Get PCI address segments for the command using its PRPs */
	ofst = nvmet_pci_prp_ofst(ctrl, prp1);
	if (ofst & 0x3)
		goto invalid_offset;

	if (iod->buffer_size + ofst <= ctrl->mps * 2)
		ret = nvmet_pci_iod_parse_prp_simple(ctrl, iod);
	else
		ret = nvmet_pci_iod_parse_prp_list(ctrl, iod);
	if (ret)
		return ret;

	/* Get an internal buffer for the command */
	ret = nvmet_pci_alloc_iod_buffer(iod);
	if (ret) {
		iod->status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
		return ret;
	}

	return 0;

invalid_field:
	dev_err(ctrl->dev, "DPTR invalid field\n");
	iod->status = NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	return -EINVAL;

invalid_offset:
	dev_err(ctrl->dev, "DPTR invalid offset\n");
	iod->status = NVME_SC_PRP_INVALID_OFFSET | NVME_STATUS_DNR;
	return -EINVAL;
}

static void nvmet_pci_exec_iod(struct nvmet_pci_iod *iod)
{
	struct nvmet_req *req = &iod->req;
	int ret;

	if (iod->buffer_size) {
		struct scatterlist *sg = &iod->buffer_sgl;

		/* Setup the command buffer */
		ret = nvmet_pci_iod_parse_dptr(iod);
		if (ret)
			goto complete;

		/* Get data from the host if needed */
		if (iod->dma_dir == DMA_FROM_DEVICE) {
			ret = nvmet_pci_transfer_iod_data(iod);
			if (ret)
				goto complete;
		}

		/* Setup SGL for our command local buffer */
		sg_init_one(sg, iod->buffer, iod->buffer_size);
		iod->req.transfer_len = iod->buffer_size;
		iod->req.sg = sg;
		iod->req.sg_cnt = 1;
	}

	/* Execute the request */
	req->execute(req);

	/*
	 * If we have no data to transfer once the command completes,
	 * nvmet_pci_queue_response() will complete the command directly.
	 * No need to wait for the completion in this case.
	 */
	if (!iod->buffer_size || iod->dma_dir != DMA_TO_DEVICE)
		return;

	wait_for_completion(&iod->done);

	if (iod->status == NVME_SC_SUCCESS) {
		WARN_ON_ONCE(!iod->buffer_size || iod->dma_dir != DMA_TO_DEVICE);
		nvmet_pci_transfer_iod_data(iod);
	}

complete:
	nvmet_pci_complete_iod(iod);
}

static void nvmet_pci_exec_iod_work(struct work_struct *work)
{
	struct nvmet_pci_iod *iod =
		container_of(work, struct nvmet_pci_iod, work);

	nvmet_pci_exec_iod(iod);
}

static bool nvmet_pci_post_iod_cqe(struct nvmet_pci_iod *iod)
{
	struct nvmet_pci_ctrl *ctrl = iod->ctrl;
	struct nvmet_pci_queue *sq = iod->sq;
	struct nvmet_pci_queue *cq = iod->cq;
	struct nvme_completion *cqe = &iod->cqe;

	/*
	 * Do not try to complete commands if the controller is not ready
	 * anymore, e.g. after the host cleared CC.EN.
	 */
	if (!test_bit(NVMET_PCI_Q_LIVE, &cq->flags)) {
		nvmet_pci_free_iod(iod);
		return false;
	}

	/* Check completion queue full state */
	cq->head = nvmet_pci_bar_read32(ctrl, cq->db);
	if (cq->head == cq->tail + 1) {
		dev_warn(ctrl->dev, "CQ[%d]: queue full\n", cq->qid);
		return false;
	}

	/* Setup the completion entry */
	cqe->sq_id = cpu_to_le16(sq->qid);
	cqe->sq_head = cpu_to_le16(sq->head);
	cqe->command_id = iod->cmd.common.command_id;
	cqe->status = cpu_to_le16((iod->status << 1) | cq->phase);

	/* Post the completion entry */
	dev_dbg(ctrl->dev,
		"CQ[%d]: %s status 0x%x, result 0x%llx, head %d, tail %d, phase %d\n",
		cq->qid, nvmet_pci_iod_name(iod), iod->status,
		le64_to_cpu(cqe->result.u64), cq->head, cq->tail, cq->phase);

	memcpy_toio(cq->pci_map.virt_addr + cq->tail * cq->qes, cqe,
		    sizeof(struct nvme_completion));

	/* Advance the tail */
	cq->tail++;
	if (cq->tail >= cq->depth) {
		cq->tail = 0;
		cq->phase ^= 1;
	}

	nvmet_pci_free_iod(iod);

	return true;
}

static void nvmet_pci_drain_queue(struct nvmet_pci_queue *queue)
{
	struct nvmet_pci_iod *iod;
	unsigned long flags;

	spin_lock_irqsave(&queue->lock, flags);

	while (!list_empty(&queue->list)) {
		iod = list_first_entry(&queue->list, struct nvmet_pci_iod, link);
		list_del_init(&iod->link);
		nvmet_pci_free_iod(iod);
	}

	spin_unlock_irqrestore(&queue->lock, flags);
}

static void nvmet_pci_cq_work(struct work_struct *work);

static int nvmet_pci_create_cq(struct nvmet_pci_ctrl *ctrl, u16 qid,
		u16 qsize, phys_addr_t pci_addr, u16 flags, u16 vector)
{
	struct nvmet_pci_queue *cq = &ctrl->cq[qid];
	int ret;

	if (test_and_set_bit(NVMET_PCI_Q_LIVE, &cq->flags)) {
		dev_err(ctrl->dev, "CQ %u is already created\n", qid);
		return -EALREADY;
	}

	if (flags & NVME_CQ_IRQ_ENABLED)
		set_bit(NVMET_PCI_Q_IRQ_ENABLED, &cq->flags);
	cq->pci_addr = pci_addr;
	cq->qid = qid;
	cq->depth = qsize + 1;
	cq->vector = vector;
	cq->head = 0;
	cq->tail = 0;
	cq->phase = 1;
	cq->db = NVME_REG_DBS + (((qid * 2) + 1) * sizeof(u32));
	nvmet_pci_bar_write32(ctrl, cq->db, 0);

	if (!qid)
		cq->qes = sizeof(struct nvme_completion);
	else
		cq->qes = ctrl->io_cqes;
	cq->pci_size = cq->qes * cq->depth;

	ret = nvmet_cq_create(ctrl->tctrl, &cq->nvme_cq, qid, cq->depth);
	if (ret) {
		clear_bit(NVMET_PCI_Q_IRQ_ENABLED, &cq->flags);
		clear_bit(NVMET_PCI_Q_LIVE, &cq->flags);
		return ret;
	}

	dev_dbg(ctrl->dev,
		"CQ %u: %u entries of %zu B, vector IRQ %u\n",
		qid, qsize, cq->qes, cq->vector + 1);

	return 0;
}

static int nvmet_pci_delete_cq(struct nvmet_pci_ctrl *ctrl, u16 qid)
{
	struct nvmet_pci_queue *cq = &ctrl->cq[qid];

	if (!test_and_clear_bit(NVMET_PCI_Q_LIVE, &cq->flags)) {
		dev_err(ctrl->dev, "CQ %u was not created\n", qid);
		return -EINVAL;
	}

	flush_delayed_work(&cq->work);
	cancel_delayed_work_sync(&cq->work);

	nvmet_pci_drain_queue(cq);

	return 0;
}

static void nvmet_pci_sq_work(struct work_struct *work);

static int nvmet_pci_create_sq(struct nvmet_pci_ctrl *ctrl,
		u16 qid, u16 qsize, phys_addr_t pci_addr)
{
	struct nvmet_pci_queue *sq = &ctrl->sq[qid];
	int ret;

	if (test_and_set_bit(NVMET_PCI_Q_LIVE, &sq->flags)) {
		dev_err(ctrl->dev, "SQ %u is already created\n", qid);
		return -EALREADY;
	}

	ret = nvmet_sq_create(ctrl->tctrl, &sq->nvme_sq, qid, sq->depth);
	if (ret)
		goto out_clear_bit;

	sq->pci_addr = pci_addr;
	sq->qid = qid;
	sq->depth = qsize + 1;
	sq->head = 0;
	sq->tail = 0;
	sq->phase = 0;
	sq->db = NVME_REG_DBS + (qid * 2 * sizeof(u32));
	nvmet_pci_bar_write32(ctrl, sq->db, 0);
	if (!qid)
		sq->qes = 1UL << NVME_ADM_SQES;
	else
		sq->qes = ctrl->io_sqes;
	sq->pci_size = sq->qes * sq->depth;

	sq->cmd_wq = alloc_workqueue("sq%d_wq", WQ_HIGHPRI | WQ_UNBOUND,
				     min_t(int, sq->depth, WQ_MAX_ACTIVE), qid);
	if (!sq->cmd_wq) {
		dev_err(ctrl->dev, "Create SQ %d cmd_wq failed\n", qid);
		ret = -ENOMEM;
		goto out_destroy_sq;
	}

	dev_dbg(ctrl->dev, "SQ %u: %u entries of %zu B\n",
		qid, qsize, sq->qes);

	/* Start polling the submission queue */
	queue_delayed_work(ctrl->wq, &sq->work, 1);

	return 0;

out_destroy_sq:
	nvmet_sq_destroy(&sq->nvme_sq);
out_clear_bit:
	clear_bit(NVMET_PCI_Q_LIVE, &sq->flags);
	return ret;
}

static int nvmet_pci_delete_sq(struct nvmet_pci_ctrl *ctrl, u16 qid)
{
	struct nvmet_pci_queue *sq = &ctrl->sq[qid];

	if (!test_and_clear_bit(NVMET_PCI_Q_LIVE, &sq->flags))
		return -EINVAL;

	flush_workqueue(sq->cmd_wq);
	destroy_workqueue(sq->cmd_wq);
	sq->cmd_wq = NULL;

	flush_delayed_work(&sq->work);
	cancel_delayed_work_sync(&sq->work);

	nvmet_pci_drain_queue(sq);

	nvmet_sq_destroy(&sq->nvme_sq);

	return 0;
}

static void nvmet_pci_delete_queues(struct nvmet_pci_ctrl *ctrl)
{
	int qid;

	for (qid = 1; qid < ctrl->nr_queues; qid++)
		nvmet_pci_delete_sq(ctrl, qid);

	for (qid = 1; qid < ctrl->nr_queues; qid++)
		nvmet_pci_delete_cq(ctrl, qid);

	/* Destory the admin queue last */
	nvmet_pci_delete_sq(ctrl, 0);
	nvmet_pci_delete_cq(ctrl, 0);
}

static int nvmet_pci_add_port(struct nvmet_port *port)
{
	mutex_lock(&nvmet_pci_ports_mutex);
	list_add_tail(&port->entry, &nvmet_pci_ports);
	mutex_unlock(&nvmet_pci_ports_mutex);
	return 0;
}

static void nvmet_pci_remove_port(struct nvmet_port *port)
{
	mutex_lock(&nvmet_pci_ports_mutex);
	list_del_init(&port->entry);
	mutex_unlock(&nvmet_pci_ports_mutex);
}

static struct nvmet_port *nvmet_pci_find_port(struct nvmet_pci_ctrl *ctrl)
{
	struct nvmet_port *port;

	/* For now, always use the first port */
	mutex_lock(&nvmet_pci_ports_mutex);
	port = list_first_entry_or_null(&nvmet_pci_ports,
					struct nvmet_port, entry);
	mutex_unlock(&nvmet_pci_ports_mutex);

	return port;
}

static int nvmet_pci_enable_ctrl(struct nvmet_ctrl *tctrl)
{
	struct nvmet_pci_ctrl *ctrl = tctrl->drvdata;
	phys_addr_t pci_addr;
	u64 asq, acq;
	u32 aqa;
	u16 qsize;
	int ret;

	if (ctrl->enabled) {
		dev_err(ctrl->dev, "Controller already enabled!\n");
		return -EALREADY;
	}

	dev_info(ctrl->dev, "Enabling controller\n");

	ctrl->mps_shift = ((ctrl->cc >> NVME_CC_MPS_SHIFT) & 0xf) + 12;
	ctrl->mps = 1UL << ctrl->mps_shift;
	ctrl->mps_mask = ctrl->mps - 1;

	ctrl->io_sqes = 1UL << ((ctrl->cc >> NVME_CC_IOSQES_SHIFT) & 0xf);
	ctrl->io_cqes = 1UL << ((ctrl->cc >> NVME_CC_IOCQES_SHIFT) & 0xf);

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
	aqa = nvmet_pci_bar_read32(ctrl, NVME_REG_AQA);
	asq = nvmet_pci_bar_read64(ctrl, NVME_REG_ASQ);
	acq = nvmet_pci_bar_read64(ctrl, NVME_REG_ACQ);

	qsize = (aqa & 0x0fff0000) >> 16;
	pci_addr = acq & GENMASK(63, 12);
	ret = nvmet_pci_create_cq(ctrl, 0, qsize, pci_addr,
				  NVME_CQ_IRQ_ENABLED, 0);
	if (ret) {
		dev_err(ctrl->dev,
			"Create admin completion queue failed\n");
		return ret;
	}

	qsize = aqa & 0x00000fff;
	pci_addr = asq & GENMASK(63, 12);
	ret = nvmet_pci_create_sq(ctrl, 0, qsize, pci_addr);
	if (ret) {
		dev_err(ctrl->dev,
			"Create admin submission queue failed\n");
		return ret;
	}

	ctrl->enabled = true;

	return 0;
}

static void nvmet_pci_disable_ctrl(struct nvmet_ctrl *tctrl)
{
	struct nvmet_pci_ctrl *ctrl = tctrl->drvdata;

	if (!ctrl->enabled)
		return;

	dev_info(ctrl->dev, "Disabling controller\n");

	nvmet_pci_delete_queues(ctrl);

	/* Tell the host we are done */
	ctrl->cc = ctrl->tctrl->cc;
	nvmet_pci_bar_write32(ctrl, NVME_REG_CC, ctrl->cc);

	ctrl->csts = ctrl->tctrl->csts;
	nvmet_pci_bar_write32(ctrl, NVME_REG_CSTS, ctrl->csts);

	ctrl->enabled = false;
}

static void nvmet_pci_queue_response(struct nvmet_req *req)
{
	struct nvmet_pci_iod *iod=
		container_of(req, struct nvmet_pci_iod, req);

	iod->status = le16_to_cpu(req->cqe->status) >> 1;

	/*
	 * If we have no data to transfer, directly complete the command.
	 * This includes AENs.
	 */
	if (!iod->buffer_size || iod->dma_dir != DMA_TO_DEVICE) {
		nvmet_pci_complete_iod(iod);
		return;
	}

	complete(&iod->done);
}

static void nvmet_pci_free_queues(struct nvmet_pci_ctrl *ctrl);

static void nvmet_pci_delete_ctrl(struct nvmet_ctrl *tctrl)
{
	struct nvmet_pci_ctrl *ctrl = tctrl->drvdata;

	nvmet_pci_free_queues(ctrl);

	if (ctrl->subsys)
		nvmet_subsys_put(ctrl->subsys);

	mutex_lock(&nvmet_pci_ctrl_mutex);
	list_del_init(&ctrl->list);
	mutex_unlock(&nvmet_pci_ctrl_mutex);
}

static u8 nvmet_pci_get_mdts(const struct nvmet_ctrl *tctrl)
{
	struct nvmet_pci_ctrl *ctrl = tctrl->drvdata;
	int page_shift = NVME_CAP_MPSMIN(tctrl->cap) + 12;

	return ilog2(ctrl->mdts) - page_shift;
}

static __le16 nvmet_pci_get_vid(const struct nvmet_ctrl *tctrl, __le16 *ssvid)
{
	struct nvmet_pci_ctrl *ctrl = tctrl->drvdata;
	struct pci_epf *epf = ctrl->nvme_epf->epf;

	*ssvid = cpu_to_le16(epf->header->subsys_vendor_id);

	return cpu_to_le16(epf->header->vendorid);
}

static const struct nvmet_fabrics_ops nvmet_pci_fabrics_ops = {
	.owner		= THIS_MODULE,
	.type		= NVMF_TRTYPE_PCI,
	.add_port	= nvmet_pci_add_port,
	.remove_port	= nvmet_pci_remove_port,
	.queue_response = nvmet_pci_queue_response,
	.delete_ctrl	= nvmet_pci_delete_ctrl,
	.enable_ctrl	= nvmet_pci_enable_ctrl,
	.disable_ctrl	= nvmet_pci_disable_ctrl,
	.get_mdts	= nvmet_pci_get_mdts,
	.get_vid	= nvmet_pci_get_vid,
};

static int nvmet_pci_alloc_queues(struct nvmet_pci_ctrl *ctrl)
{
	struct nvmet_pci_queue *sq, *cq;
	int i;

	ctrl->sq = kcalloc(ctrl->nr_queues,
			   sizeof(struct nvmet_pci_queue), GFP_KERNEL);
	if (!ctrl->sq)
		return -ENOMEM;

	ctrl->cq = kcalloc(ctrl->nr_queues,
			   sizeof(struct nvmet_pci_queue), GFP_KERNEL);
	if (!ctrl->cq) {
		kfree(ctrl->sq);
		ctrl->sq = NULL;
		return -ENOMEM;
	}

	for (i = 0; i < ctrl->nr_queues; i++) {
		sq = &ctrl->sq[i];
		set_bit(NVMET_PCI_Q_IS_SQ, &sq->flags);
		sq->ctrl = ctrl;
		sq->qid = i;
		INIT_DELAYED_WORK(&sq->work, nvmet_pci_sq_work);
		spin_lock_init(&sq->lock);
		INIT_LIST_HEAD(&sq->list);

		cq = &ctrl->cq[i];
		cq->ctrl = ctrl;
		cq->qid = i;
		INIT_DELAYED_WORK(&cq->work, nvmet_pci_cq_work);
		spin_lock_init(&cq->lock);
		INIT_LIST_HEAD(&cq->list);
	}

	return 0;
}

static void nvmet_pci_free_queues(struct nvmet_pci_ctrl *ctrl)
{
	kfree(ctrl->sq);
	ctrl->sq = NULL;
	kfree(ctrl->cq);
	ctrl->cq = NULL;
}

static void nvmet_pci_init_bar(struct nvmet_pci_ctrl *ctrl)
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

	/* Memory Page Size minimum (MPSMIN) = 4K */
	ctrl->cap |= (NVME_CTRL_PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;

	/* Memory Page Size maximum (MPSMAX) = 4K */
	ctrl->cap |= (NVME_CTRL_PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;

	/* Clear Persistent Memory Region Supported (PMRS) */
	ctrl->cap &= ~(0x1ULL << 56);

	/* Clear Controller Memory Buffer Supported (CMBS) */
	ctrl->cap &= ~(0x1ULL << 57);

	/* NVMe version supported */
	ctrl->vs = tctrl->subsys->ver;

	/* Controller configuration */
	ctrl->cc = tctrl->cc & (~NVME_CC_ENABLE);

	/* Controller status */
	ctrl->csts = ctrl->tctrl->csts;

	nvmet_pci_bar_write64(ctrl, NVME_REG_CAP, ctrl->cap);
	nvmet_pci_bar_write32(ctrl, NVME_REG_VS, ctrl->vs);
	nvmet_pci_bar_write32(ctrl, NVME_REG_CSTS, ctrl->csts);
	nvmet_pci_bar_write32(ctrl, NVME_REG_CC, ctrl->cc);
}

static void nvmet_pci_poll_cc_work(struct work_struct *work)
{
	struct nvmet_pci_ctrl *ctrl =
		container_of(work, struct nvmet_pci_ctrl, poll_cc.work);

	if (!ctrl->tctrl)
		return;

	/*
	 * Get the current value and we will be notified about what to do
	 * with the enable_ctrl and disable_ctrl operations.
	 */
	ctrl->cc = nvmet_pci_bar_read32(ctrl, NVME_REG_CC);
	nvmet_update_cc(ctrl->tctrl, ctrl->cc);
	ctrl->csts = ctrl->tctrl->csts;
	nvmet_pci_bar_write32(ctrl, NVME_REG_CSTS, ctrl->csts);

	schedule_delayed_work(&ctrl->poll_cc, msecs_to_jiffies(5));
}

static int nvmet_pci_create_ctrl(struct nvmet_pci_epf *nvme_epf,
				unsigned int max_nr_queues)
{
	struct nvmet_pci_ctrl *ctrl = &nvme_epf->ctrl;
	int ret;

	nvme_epf->ctrl.nvme_epf = nvme_epf;
	ctrl->dev = &nvme_epf->epf->dev;
	ctrl->mdts = nvme_epf->mdts_kb * SZ_1K;
	uuid_gen(&ctrl->hostid);
	INIT_DELAYED_WORK(&ctrl->poll_cc, nvmet_pci_poll_cc_work);

	ctrl->port = nvmet_pci_find_port(ctrl);
	if (!ctrl->port) {
		dev_err(ctrl->dev, "Port not found\n");
		return -EINVAL;
	}

	ctrl->subsys = nvmet_find_get_subsys(ctrl->port, nvme_epf->subsysnqn);
	if (!ctrl->subsys) {
		dev_err(ctrl->dev, "Subsystem not found\n");
		return -EINVAL;
	}

	/* Allocate our queues, up to the maximum number */
	ctrl->nr_queues = min(ctrl->subsys->max_qid + 1, max_nr_queues);
	ret = nvmet_pci_alloc_queues(ctrl);
	if (ret)
		goto out_put_subsys;

	/* Create the workqueue for processing our SQs and CQs */
	ctrl->wq = alloc_workqueue("nvmet_pci_ctrl_wq", WQ_HIGHPRI | WQ_UNBOUND,
			min_t(int, ctrl->nr_queues * 2, WQ_MAX_ACTIVE));
	if (!ctrl->wq) {
		dev_err(ctrl->dev, "Create controller wq failed\n");
		goto out_free_queues;
	}

	/* Create the target controller */
	ctrl->tctrl = nvmet_ctrl_create(ctrl->port, nvme_epf->subsysnqn,
					&nvmet_pci_fabrics_ops);
	if (!ctrl->tctrl) {
		dev_err(ctrl->dev, "Create target controller failed\n");
		goto out_destroy_wq;
	}

	ctrl->tctrl->drvdata = ctrl;

	dev_info(ctrl->dev,
		 "New PCI ctrl: \"%s\", %u I/O queues (qid_max=%u), mdts %u B\n",
		 ctrl->tctrl->subsys->subsysnqn,
		 ctrl->nr_queues - 1, ctrl->tctrl->subsys->max_qid,
		 ctrl->mdts);

	mutex_lock(&nvmet_pci_ctrl_mutex);
	list_add_tail(&ctrl->list, &nvmet_pci_ctrl_list);
	mutex_unlock(&nvmet_pci_ctrl_mutex);

	/* Initialize our BAR */
	nvmet_pci_init_bar(ctrl);

	return 0;

out_destroy_wq:
	destroy_workqueue(ctrl->wq);
	ctrl->wq = NULL;
out_free_queues:
	nvmet_pci_free_queues(ctrl);
out_put_subsys:
	nvmet_subsys_put(ctrl->subsys);
	ctrl->subsys = NULL;
	return ret;
}

static void nvmet_pci_admin_create_cq(struct nvmet_pci_iod *iod)
{
	struct nvmet_pci_ctrl *ctrl = iod->ctrl;
	struct nvme_command *cmd = &iod->cmd;
	u16 cqid = le16_to_cpu(cmd->create_cq.cqid);
	u16 cq_flags = le16_to_cpu(cmd->create_cq.cq_flags);
	u16 qsize = le16_to_cpu(cmd->create_cq.qsize);
	u16 vector = le16_to_cpu(cmd->create_cq.irq_vector);
	phys_addr_t pci_addr = le64_to_cpu(cmd->create_cq.prp1);
	u16 mqes = NVME_CAP_MQES(ctrl->cap);
	int ret;

	if (!cqid || cqid >= ctrl->nr_queues ||
	    test_bit(NVMET_PCI_Q_LIVE, &ctrl->cq[cqid].flags)) {
		iod->status = NVME_SC_QID_INVALID | NVME_STATUS_DNR;
		return;
	}

	if (!(cq_flags & NVME_QUEUE_PHYS_CONTIG)) {
		iod->status = NVME_SC_INVALID_QUEUE | NVME_STATUS_DNR;
		return;
	}

	if (!qsize) {
		iod->status = NVME_SC_QUEUE_SIZE | NVME_STATUS_DNR;
		return;
	}

	if (qsize > mqes) {
		iod->status = NVME_SC_QUEUE_SIZE | NVME_STATUS_DNR;
		return;
	}

	if (vector >= ctrl->nvme_epf->nr_vectors) {
		iod->status = NVME_SC_INVALID_VECTOR | NVME_STATUS_DNR;
		return;
	}

	ret = nvmet_pci_create_cq(ctrl, cqid, qsize, pci_addr, cq_flags, vector);
	if (ret) {
		iod->status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
		return;
	}

	iod->status = NVME_SC_SUCCESS;
}

static void nvmet_pci_admin_delete_cq(struct nvmet_pci_iod *iod)
{
	struct nvmet_pci_ctrl *ctrl = iod->ctrl;
	u16 cqid = cqid = le16_to_cpu(iod->cmd.delete_queue.qid);
	int ret;

	if (!cqid || cqid >= ctrl->nr_queues ||
	    !test_bit(NVMET_PCI_Q_LIVE, &ctrl->cq[cqid].flags)) {
		iod->status = NVME_SC_QID_INVALID | NVME_STATUS_DNR;
		return;
	}

	ret = nvmet_pci_delete_cq(ctrl, cqid);
	if (ret)
		iod->status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
}

static void nvmet_pci_admin_create_sq(struct nvmet_pci_iod *iod)
{
	struct nvmet_pci_ctrl *ctrl = iod->ctrl;
	struct nvme_command *cmd = &iod->cmd;
	u16 sqid = le16_to_cpu(cmd->create_sq.sqid);
	u16 cqid = le16_to_cpu(cmd->create_sq.cqid);
	u16 sq_flags = le16_to_cpu(cmd->create_sq.sq_flags);
	u16 qsize = le16_to_cpu(cmd->create_sq.qsize);
	phys_addr_t pci_addr = le64_to_cpu(cmd->create_sq.prp1);
	u16 mqes = NVME_CAP_MQES(ctrl->cap);
	int ret;

	if (!sqid || sqid >= ctrl->nr_queues ||
	    test_bit(NVMET_PCI_Q_LIVE, &ctrl->sq[sqid].flags)) {
		iod->status = NVME_SC_QID_INVALID | NVME_STATUS_DNR;
		return;
	}

	if (!cqid || cqid >= ctrl->nr_queues ||
	    !test_bit(NVMET_PCI_Q_LIVE, &ctrl->cq[cqid].flags)) {
		iod->status = NVME_SC_CQ_INVALID | NVME_STATUS_DNR;
		return;
	}

	/*
	 * Note: The NVMe specification allows multiple SQs to specify the
	 * same CQ. However, the target code does not support that. So for now,
	 * prevent this and fail the command if sqid and cqid are different.
	 */
	if (cqid != sqid) {
		dev_err(ctrl->dev,
			"SQ %u: Invalid CQID %u\n", sqid, cqid);
		iod->status = NVME_SC_CQ_INVALID | NVME_STATUS_DNR;
		return;
	}

	if (!(sq_flags & NVME_QUEUE_PHYS_CONTIG)) {
		iod->status = NVME_SC_INVALID_QUEUE | NVME_STATUS_DNR;
		return;
	}

	if (!qsize) {
		iod->status = NVME_SC_QUEUE_SIZE | NVME_STATUS_DNR;
		return;
	}

	if (qsize > mqes) {
		iod->status = NVME_SC_QUEUE_SIZE | NVME_STATUS_DNR;
		return;
	}

	ret = nvmet_pci_create_sq(ctrl, sqid, qsize, pci_addr);
	if (ret) {
		iod->status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
		return;
	}

	iod->status = NVME_SC_SUCCESS;
}

static void nvmet_pci_admin_delete_sq(struct nvmet_pci_iod *iod)
{
	struct nvmet_pci_ctrl *ctrl = iod->ctrl;
	u16 sqid = le16_to_cpu(iod->cmd.delete_queue.qid);
	int ret;

	if (!sqid || sqid >= ctrl->nr_queues ||
	    !test_bit(NVMET_PCI_Q_LIVE, &ctrl->sq[sqid].flags)) {
		iod->status = NVME_SC_QID_INVALID | NVME_STATUS_DNR;
		return;
	}

	ret = nvmet_pci_delete_sq(ctrl, sqid);
	if (ret)
		iod->status = NVME_SC_INTERNAL | NVME_STATUS_DNR;
}

static void nvmet_pci_process_admin_iod(struct nvmet_pci_iod *iod)
{
	struct nvmet_pci_ctrl *ctrl = iod->ctrl;
	struct nvme_command *cmd = &iod->cmd;
	struct nvmet_req *req = &iod->req;

	dev_dbg(ctrl->dev, "SQ[%u]: Admin command %s (0x%02x)\n",
		iod->sq->qid, nvmet_pci_iod_name(iod), cmd->common.opcode);

	switch (cmd->common.opcode) {
	case nvme_admin_identify:
		iod->buffer_size = NVME_IDENTIFY_DATA_SIZE;
		iod->dma_dir = DMA_TO_DEVICE;
		break;

	case nvme_admin_get_log_page:
		iod->buffer_size = nvmet_get_log_page_len(cmd);
		iod->dma_dir = DMA_TO_DEVICE;
		break;

	case nvme_admin_async_event:
	case nvme_admin_set_features:
	case nvme_admin_get_features:
	case nvme_admin_abort_cmd:
		break;

	case nvme_admin_create_cq:
		nvmet_pci_admin_create_cq(iod);
		goto complete;

	case nvme_admin_create_sq:
		nvmet_pci_admin_create_sq(iod);
		goto complete;

	case nvme_admin_delete_cq:
		nvmet_pci_admin_delete_cq(iod);
		goto complete;

	case nvme_admin_delete_sq:
		nvmet_pci_admin_delete_sq(iod);
		goto complete;

	default:
		dev_err(ctrl->dev,
			"SQ[%u]: Unhandled admin command %s (0x%02x)\n",
			iod->sq->qid, nvmet_pci_iod_name(iod),
			cmd->common.opcode);
		iod->status = NVME_SC_INVALID_OPCODE | NVME_STATUS_DNR;
		goto complete;
	}

	if (!nvmet_req_init(req, &iod->cq->nvme_cq, &iod->sq->nvme_sq,
			    &nvmet_pci_fabrics_ops)) {
		iod->status = le16_to_cpu(req->cqe->status) >> 1;
		goto complete;
	}

	nvmet_pci_exec_iod(iod);

	return;

complete:
	nvmet_pci_complete_iod(iod);
}

static void nvmet_pci_process_io_iod(struct nvmet_pci_iod *iod)
{
	struct nvmet_req *req = &iod->req;

	dev_dbg(iod->ctrl->dev, "SQ[%u]: IO command %s (0x%02x)\n",
		iod->sq->qid, nvmet_pci_iod_name(iod),
		iod->cmd.common.opcode);

	if (!nvmet_req_init(req, &iod->cq->nvme_cq, &iod->sq->nvme_sq,
			    &nvmet_pci_fabrics_ops)) {
		iod->status = le16_to_cpu(req->cqe->status) >> 1;
		goto complete;
	}

	switch (iod->cmd.common.opcode) {
	case nvme_cmd_read:
		iod->buffer_size = nvmet_rw_data_len(&iod->req);
		iod->dma_dir = DMA_TO_DEVICE;
		break;

	case nvme_cmd_write:
	case nvme_cmd_zone_append:
		iod->buffer_size = nvmet_rw_data_len(&iod->req);
		iod->dma_dir = DMA_FROM_DEVICE;
		break;

	case nvme_cmd_dsm:
		iod->buffer_size = (le32_to_cpu(iod->cmd.dsm.nr) + 1) *
			sizeof(struct nvme_dsm_range);
		iod->dma_dir = DMA_FROM_DEVICE;
		break;

	case nvme_cmd_zone_mgmt_recv:
		iod->buffer_size = (le32_to_cpu(req->cmd->zmr.numd) + 1) << 2;
		iod->dma_dir = DMA_TO_DEVICE;
		break;

	case nvme_cmd_flush:
	case nvme_cmd_write_zeroes:
	case nvme_cmd_zone_mgmt_send:
		break;

	default:
		dev_err(iod->ctrl->dev,
			"SQ[%u]: Unhandled IO command %s (0x%02x)\n",
			iod->sq->qid, nvmet_pci_iod_name(iod),
			iod->cmd.common.opcode);
		iod->status = NVME_SC_INVALID_OPCODE | NVME_STATUS_DNR;
		goto complete;
	}

	queue_work(iod->sq->cmd_wq, &iod->work);

	return;

complete:
	nvmet_pci_complete_iod(iod);
}

static void nvmet_pci_process_iods(struct nvmet_pci_ctrl *ctrl,
				  struct nvmet_pci_queue *sq)
{
	struct nvmet_pci_iod *iod;
	struct nvme_command *cmd;

	while (!list_empty(&sq->list)) {

		iod = list_first_entry(&sq->list, struct nvmet_pci_iod, link);
		list_del_init(&iod->link);

		if (!test_bit(NVMET_PCI_Q_LIVE, &sq->flags)) {
			iod->status = NVME_SC_QID_INVALID | NVME_STATUS_DNR;
			nvmet_pci_complete_iod(iod);
			continue;
		}

		/* We do not support SGL for now... */
		cmd = &iod->cmd;
		if (iod->cmd.common.flags & NVME_CMD_SGL_ALL) {
			dev_err(ctrl->dev, "IOD invalid SGL\n");
			iod->status = NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
			nvmet_pci_complete_iod(iod);
			continue;
		}

		/* ...but we must use SGLs for the target commands */
		cmd->common.flags &= ~(NVME_CMD_SGL_ALL);
		cmd->common.flags |= NVME_CMD_SGL_METABUF;

		if (iod->sq->qid)
			nvmet_pci_process_io_iod(iod);
		else
			nvmet_pci_process_admin_iod(iod);
	}
}

static bool nvmet_pci_fetch_iods(struct nvmet_pci_ctrl *ctrl,
				struct nvmet_pci_queue *sq)
{
	struct nvmet_pci_epf *nvme_epf = ctrl->nvme_epf;
	struct nvmet_pci_iod *iod;
	int ret;

	if (!test_bit(NVMET_PCI_Q_LIVE, &sq->flags))
		return false;

	sq->tail = nvmet_pci_bar_read32(ctrl, sq->db);
	if (sq->head == sq->tail)
		return false;

	ret = nvmet_pci_epf_map_queue(nvme_epf, sq);
	if (ret)
		return false;

	while (sq->head != sq->tail) {
		iod = nvmet_pci_alloc_iod(sq);
		if (!iod)
			break;

		/* Get the NVMe command submitted by the host */
		memcpy_fromio(&iod->cmd,
			      sq->pci_map.virt_addr + sq->head * sq->qes,
			      sizeof(struct nvme_command));

		dev_dbg(ctrl->dev,
			"SQ[%u]: head %u/%u, tail %u, command %s\n",
			sq->qid, (int)sq->head, (int)sq->depth,
			(int)sq->tail, nvmet_pci_iod_name(iod));

		sq->head++;
		if (sq->head == sq->depth)
			sq->head = 0;

		list_add_tail(&iod->link, &sq->list);
	}

	nvmet_pci_epf_unmap_queue(nvme_epf, sq);

	return !list_empty(&sq->list);
}

static void nvmet_pci_sq_work(struct work_struct *work)
{
	struct nvmet_pci_queue *sq =
		container_of(work, struct nvmet_pci_queue, work.work);
	struct nvmet_pci_ctrl *ctrl = sq->ctrl;
	unsigned long poll_interval = 1;
	unsigned long j = jiffies;

	while (test_bit(NVMET_PCI_Q_LIVE, &sq->flags)) {
		/*
		 * Try to get commands from the host. If We do not yet have any
		 * command, aggressively keep polling the SQ of IO queues for at
		 * most one tick and fall back to rescheduling the SQ work if we
		 * have not received any command after that. This hybrid
		 * spin-polling method significantly increases the IOPS for
		 * shallow queue depth operation (e.g. QD=1).
		 */
		if (!nvmet_pci_fetch_iods(ctrl, sq)) {
			if (!sq->qid || jiffies > j)
				break;
			usleep_range(1, 2);
			continue;
		}

		nvmet_pci_process_iods(ctrl, sq);
	}

	if (!test_bit(NVMET_PCI_Q_LIVE, &sq->flags))
		return;

	/* No need to aggressively poll the admin queue. */
	if (!sq->qid)
		poll_interval = msecs_to_jiffies(5);
	queue_delayed_work(ctrl->wq, &sq->work, poll_interval);
}

static void nvmet_pci_cq_work(struct work_struct *work)
{
	struct nvmet_pci_queue *cq =
		container_of(work, struct nvmet_pci_queue, work.work);
	struct nvmet_pci_ctrl *ctrl = cq->ctrl;
	struct nvmet_pci_epf *nvme_epf = ctrl->nvme_epf;
	struct nvmet_pci_iod *iod;
	unsigned long flags;
	LIST_HEAD(list);
	int ret, n;

	spin_lock_irqsave(&cq->lock, flags);

	while (!list_empty(&cq->list)) {

		list_splice_tail_init(&cq->list, &list);
		spin_unlock_irqrestore(&cq->lock, flags);

		ret = nvmet_pci_epf_map_queue(nvme_epf, cq);
		if (ret) {
			/* Retry again later */
			queue_delayed_work(ctrl->wq, &cq->work, 1);
			return;
		}

		n = 0;
		while (!list_empty(&list)) {
			iod = list_first_entry(&list, struct nvmet_pci_iod, link);
			list_del_init(&iod->link);
			if (nvmet_pci_post_iod_cqe(iod))
				n++;
		}

		nvmet_pci_epf_unmap_queue(nvme_epf, cq);

		if (n)
			nvmet_pci_epf_raise_irq(nvme_epf, cq);

		spin_lock_irqsave(&cq->lock, flags);
	}

	/*
	 * Completions on the host may trigger issuing of new commands. Try to
	 * get these early to improve IOPS and reduce latency.
	 */
	if (cq->qid)
		queue_delayed_work(ctrl->wq, &cq->work, 0);

	spin_unlock_irqrestore(&cq->lock, flags);
}

static int nvmet_pci_epf_create_ctrl(struct nvmet_pci_epf *nvme_epf)
{
	struct pci_epf *epf = nvme_epf->epf;
	const struct pci_epc_features *epc_features = nvme_epf->epc_features;
	unsigned int max_nr_queues = NVMET_NR_QUEUES;
	int ret;

	/*
	 * Cap the maximum number of queues we can support depending on the
	 * number of IRQs we can use.
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

	/* Create the controller */
	ret = nvmet_pci_create_ctrl(nvme_epf, max_nr_queues);
	if (ret) {
		dev_err(&epf->dev,
			"Create NVMe PCI target controller failed\n");
		return ret;
	}

	return 0;
}

static void nvmet_pci_epf_destroy_ctrl(struct nvmet_pci_epf *nvme_epf)
{
	struct nvmet_pci_ctrl *ctrl = &nvme_epf->ctrl;

	dev_info(ctrl->dev, "Deleting controller\n");

	cancel_delayed_work_sync(&ctrl->poll_cc);

	if (ctrl->enabled) {
		ctrl->cc |= NVME_CC_SHN_NORMAL;
		nvmet_update_cc(ctrl->tctrl, ctrl->cc);
	}

	if (ctrl->wq) {
		flush_workqueue(ctrl->wq);
		destroy_workqueue(ctrl->wq);
		ctrl->wq = NULL;
	}

	if (ctrl->tctrl) {
		nvmet_ctrl_put(ctrl->tctrl);
		ctrl->tctrl = NULL;
	}
}

static void nvmet_pci_epf_start_ctrl(struct nvmet_pci_epf *nvme_epf)
{
	struct nvmet_pci_ctrl *ctrl = &nvme_epf->ctrl;

	schedule_delayed_work(&ctrl->poll_cc, msecs_to_jiffies(5));
}

static int nvmet_pci_epf_configure_bar(struct nvmet_pci_epf *nvme_epf)
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

	reg_bar_size = ALIGN(reg_size, 4096);

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

static void nvmet_pci_epf_clear_bar(struct nvmet_pci_epf *nvme_epf)
{
	struct pci_epf *epf = nvme_epf->epf;

	pci_epc_clear_bar(epf->epc, epf->func_no, epf->vfunc_no,
			  &epf->bar[BAR_0]);
	pci_epf_free_space(epf, nvme_epf->reg_bar, BAR_0, PRIMARY_INTERFACE);
	nvme_epf->reg_bar = NULL;
}

static int nvmet_pci_epf_init_irq(struct nvmet_pci_epf *nvme_epf)
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

static int nvmet_pci_epf_epc_init(struct pci_epf *epf)
{
	struct nvmet_pci_epf *nvme_epf = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features = nvme_epf->epc_features;
	int ret;

	if (epf->vfunc_no <= 1) {
		/* Set device ID, class, etc */
		ret = pci_epc_write_header(epf->epc, epf->func_no, epf->vfunc_no,
					   epf->header);
		if (ret) {
			dev_err(&epf->dev,
				"Write configuration header failed %d\n", ret);
			return ret;
		}
	}

	/* Setup the PCIe BAR and create the controller */
	ret = pci_epc_set_bar(epf->epc, epf->func_no, epf->vfunc_no,
			      &epf->bar[BAR_0]);
	if (ret) {
		dev_err(&epf->dev, "Set BAR 0 failed\n");
		goto clear_bar;
	}

	/* We are now ready to create our controller */
	ret = nvmet_pci_epf_create_ctrl(nvme_epf);
	if (ret)
		goto clear_bar;

	/*
	 * Enable interrupts and start polling the controller BAR if we do not
	 * have any link up notifier.
	 */
	ret = nvmet_pci_epf_init_irq(nvme_epf);
	if (ret)
		goto clear_bar;

	if (!epc_features->linkup_notifier)
		nvmet_pci_epf_start_ctrl(nvme_epf);

	return 0;

clear_bar:
	nvmet_pci_epf_clear_bar(nvme_epf);
	return ret;
}

static void nvmet_pci_epf_epc_deinit(struct pci_epf *epf)
{
	struct nvmet_pci_epf *nvme_epf = epf_get_drvdata(epf);

	nvmet_pci_epf_destroy_ctrl(nvme_epf);
	nvmet_pci_epf_deinit_dma(nvme_epf);
	nvmet_pci_epf_clear_bar(nvme_epf);
}

static int nvmet_pci_epf_link_up(struct pci_epf *epf)
{
	struct nvmet_pci_epf *nvme_epf = epf_get_drvdata(epf);

	dev_info(nvme_epf->ctrl.dev, "PCI link up\n");

	nvmet_pci_epf_start_ctrl(nvme_epf);

	return 0;
}

static int nvmet_pci_epf_link_down(struct pci_epf *epf)
{
	struct nvmet_pci_epf *nvme_epf = epf_get_drvdata(epf);

	dev_info(nvme_epf->ctrl.dev, "PCI link down\n");

	nvmet_pci_epf_destroy_ctrl(nvme_epf);

	return 0;
}

static const struct pci_epc_event_ops nvmet_pci_epf_event_ops = {
	.epc_init = nvmet_pci_epf_epc_init,
	.epc_deinit = nvmet_pci_epf_epc_deinit,
	.link_up = nvmet_pci_epf_link_up,
	.link_down = nvmet_pci_epf_link_down,
};

static int nvmet_pci_epf_bind(struct pci_epf *epf)
{
	struct nvmet_pci_epf *nvme_epf = epf_get_drvdata(epf);
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

	ret = nvmet_pci_epf_configure_bar(nvme_epf);
	if (ret)
		return ret;

	if (nvme_epf->dma_enable) {
		dma_supported = nvmet_pci_epf_init_dma(nvme_epf);
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

static void nvmet_pci_epf_unbind(struct pci_epf *epf)
{
	struct nvmet_pci_epf *nvme_epf = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	nvmet_pci_epf_destroy_ctrl(nvme_epf);

	if (epc->init_complete) {
		nvmet_pci_epf_deinit_dma(nvme_epf);
		nvmet_pci_epf_clear_bar(nvme_epf);
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

static int nvmet_pci_epf_probe(struct pci_epf *epf,
			      const struct pci_epf_device_id *id)
{
	struct nvmet_pci_epf *nvme_epf;

	nvme_epf = devm_kzalloc(&epf->dev, sizeof(*nvme_epf), GFP_KERNEL);
	if (!nvme_epf)
		return -ENOMEM;

	nvme_epf->epf = epf;

	/* Set default attribute values */
	nvme_epf->dma_enable = true;
	nvme_epf->mdts_kb = NVMET_PCI_EPF_MDTS_KB;

	epf->event_ops = &nvmet_pci_epf_event_ops;
	epf->header = &nvme_epf_pci_header;
	epf_set_drvdata(epf, nvme_epf);

	return 0;
}

#define to_nvme_epf(epf_group)	\
	container_of(epf_group, struct nvmet_pci_epf, group)

static ssize_t nvmet_pci_epf_dma_enable_show(struct config_item *item,
					    char *page)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pci_epf *nvme_epf = to_nvme_epf(group);

	return sysfs_emit(page, "%d\n", nvme_epf->dma_enable);
}

static ssize_t nvmet_pci_epf_dma_enable_store(struct config_item *item,
					     const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pci_epf *nvme_epf = to_nvme_epf(group);
	int ret;

	if (nvme_epf->ctrl.tctrl)
		return -EBUSY;

	ret = kstrtobool(page, &nvme_epf->dma_enable);
	if (ret)
		return ret;

	return len;
}

CONFIGFS_ATTR(nvmet_pci_epf_, dma_enable);

static ssize_t nvmet_pci_epf_subsysnqn_show(struct config_item *item, char *page)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pci_epf *nvme_epf = to_nvme_epf(group);

	return sysfs_emit(page, "%s\n", nvme_epf->subsysnqn);
}

static ssize_t nvmet_pci_epf_subsysnqn_store(struct config_item *item,
					    const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pci_epf *nvme_epf = to_nvme_epf(group);

	/* Do not allow setting this when the function is already started */
	if (nvme_epf->ctrl.tctrl)
		return -EBUSY;

	if (!len)
		return -EINVAL;

	strscpy(nvme_epf->subsysnqn, page, len);

	return len;
}

CONFIGFS_ATTR(nvmet_pci_epf_, subsysnqn);

static ssize_t nvmet_pci_epf_mdts_kb_show(struct config_item *item, char *page)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pci_epf *nvme_epf = to_nvme_epf(group);

	return sysfs_emit(page, "%u\n", nvme_epf->mdts_kb);
}

static ssize_t nvmet_pci_epf_mdts_kb_store(struct config_item *item,
					  const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct nvmet_pci_epf *nvme_epf = to_nvme_epf(group);
	unsigned long mdts_kb;
	int ret;

	if (nvme_epf->ctrl.tctrl)
		return -EBUSY;

	ret = kstrtoul(page, 0, &mdts_kb);
	if (ret)
		return ret;
	if (!mdts_kb)
		mdts_kb = NVMET_PCI_EPF_MDTS_KB;
	else if (mdts_kb > NVMET_PCI_EPF_MAX_MDTS_KB)
		mdts_kb = NVMET_PCI_EPF_MAX_MDTS_KB;

	if (!is_power_of_2(mdts_kb))
		return -EINVAL;

	nvme_epf->mdts_kb = mdts_kb;

	return len;
}

CONFIGFS_ATTR(nvmet_pci_epf_, mdts_kb);

static struct configfs_attribute *nvmet_pci_epf_attrs[] = {
	&nvmet_pci_epf_attr_dma_enable,
	&nvmet_pci_epf_attr_subsysnqn,
	&nvmet_pci_epf_attr_mdts_kb,
	NULL,
};

static const struct config_item_type nvmet_pci_epf_group_type = {
	.ct_attrs	= nvmet_pci_epf_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_group *nvmet_pci_epf_add_cfs(struct pci_epf *epf,
						 struct config_group *group)
{
	struct nvmet_pci_epf *nvme_epf = epf_get_drvdata(epf);

	/* Add the NVMe target attributes */
	config_group_init_type_name(&nvme_epf->group, "nvme",
				    &nvmet_pci_epf_group_type);

	return &nvme_epf->group;
}

static const struct pci_epf_device_id nvmet_pci_epf_ids[] = {
	{ .name = "nvmet_pci_epf" },
	{},
};

static struct pci_epf_ops nvmet_pci_epf_ops = {
	.bind	= nvmet_pci_epf_bind,
	.unbind	= nvmet_pci_epf_unbind,
	.add_cfs = nvmet_pci_epf_add_cfs,
};

static struct pci_epf_driver nvmet_pci_epf_driver = {
	.driver.name	= "nvmet_pci_epf",
	.probe		= nvmet_pci_epf_probe,
	.id_table	= nvmet_pci_epf_ids,
	.ops		= &nvmet_pci_epf_ops,
	.owner		= THIS_MODULE,
};

static struct nvme_ctrl *nvmet_pci_create_ctrl_noop(struct device *dev,
					struct nvmf_ctrl_options *opts)
{
	dev_err(dev,
		"PCI target must be initialized using a PCI endpoint function\n");
	return ERR_PTR(-ENOTSUPP);
}

static struct nvmf_transport_ops nvmet_pci_transport_ops = {
	.name		= "pci",
	.module		= THIS_MODULE,
	.create_ctrl	= nvmet_pci_create_ctrl_noop,
};

static int __init nvmet_pci_init_module(void)
{
	int ret;

	nvmet_pci_iod_cache = kmem_cache_create("nvmet_pci_iod",
					       sizeof(struct nvmet_pci_iod),
					       0, SLAB_HWCACHE_ALIGN, NULL);
	if (!nvmet_pci_iod_cache)
		return -ENOMEM;

	ret = pci_epf_register_driver(&nvmet_pci_epf_driver);
	if (ret)
		goto cache_destroy;

	ret = nvmet_register_transport(&nvmet_pci_fabrics_ops);
	if (ret)
		goto unregister_epf_driver;

	ret = nvmf_register_transport(&nvmet_pci_transport_ops);
	if (ret)
		goto unregister_transport;

	return 0;

unregister_transport:
	nvmet_unregister_transport(&nvmet_pci_fabrics_ops);
unregister_epf_driver:
	pci_epf_unregister_driver(&nvmet_pci_epf_driver);
cache_destroy:
	kmem_cache_destroy(nvmet_pci_iod_cache);

	return ret;
}

static void __exit nvmet_pci_cleanup_module(void)
{
	nvmf_unregister_transport(&nvmet_pci_transport_ops);
	nvmet_unregister_transport(&nvmet_pci_fabrics_ops);

	pci_epf_unregister_driver(&nvmet_pci_epf_driver);

	kmem_cache_destroy(nvmet_pci_iod_cache);
}

module_init(nvmet_pci_init_module);
module_exit(nvmet_pci_cleanup_module);

MODULE_DESCRIPTION("NVMe PCI endpoint target driver");
MODULE_AUTHOR("Damien Le Moal <dlemoal@kernel.org>");
MODULE_LICENSE("GPL v2");
