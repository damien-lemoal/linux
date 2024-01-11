// SPDX-License-Identifier: GPL-2.0
/*
 * Zoned block device handling
 *
 * Copyright (c) 2015, Hannes Reinecke
 * Copyright (c) 2015, SUSE Linux GmbH
 *
 * Copyright (c) 2016, Damien Le Moal
 * Copyright (c) 2016, Western Digital
 * Copyright (c) 2024, Western Digital Corporation or its affiliates.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/sched/mm.h>
#include <linux/spinlock.h>

#include "blk.h"
#include "blk-mq-sched.h"

#define ZONE_COND_NAME(name) [BLK_ZONE_COND_##name] = #name
static const char *const zone_cond_name[] = {
	ZONE_COND_NAME(NOT_WP),
	ZONE_COND_NAME(EMPTY),
	ZONE_COND_NAME(IMP_OPEN),
	ZONE_COND_NAME(EXP_OPEN),
	ZONE_COND_NAME(CLOSED),
	ZONE_COND_NAME(READONLY),
	ZONE_COND_NAME(FULL),
	ZONE_COND_NAME(OFFLINE),
};
#undef ZONE_COND_NAME

static struct kmem_cache *blk_zone_wplugs_cachep;

/*
 * xarray value entry for conventional zones.
 */
#define BLK_ZONE_WPLUG_CONV	(BLK_ZONE_TYPE_CONVENTIONAL)

/*
 * Per-zone write plug.
 */
struct blk_zone_wplug {
	spinlock_t		lock;
	unsigned int		flags;
	unsigned int		zone_no;
	unsigned int		wp_offset;
	struct bio_list		bio_list;
	union {
		struct work_struct	bio_work;
		struct {
			struct gendisk	*disk;
			struct rcu_head	rcu_head;
		};
	};
};

/*
 * Zone write plug flags bits:
 *  - BLK_ZONE_WPLUG_NEW: Indicate that the zone write plug was just allocated
 *    and added to the xarray. This is used to prevent freeing new write plugs
 *    by reset or finish operations received concurrently with a write
 *    operation.
 *  - BLK_ZONE_WPLUG_PLUGGED: Indicate that the zone write plug is plugged,
 *    that is, that write BIOs are being throttled due to a write BIO already
 *    being executed or the zone write plug bio list is not empty.
 *  - BLK_ZONE_WPLUG_ERROR: Indicate that a write error happened which will be
 *    recovered with a report zone to update the zone write pointer offset.
 *  - BLK_ZONE_WPLUG_FREEING: Indicates that the zone write plug is being freed
 *    and should not be used.
 */
#define BLK_ZONE_WPLUG_NEW	(1U << 0)
#define BLK_ZONE_WPLUG_PLUGGED	(1U << 1)
#define BLK_ZONE_WPLUG_ERROR	(1U << 2)
#define BLK_ZONE_WPLUG_FREEING	(1U << 3)

#define BLK_ZONE_WPLUG_BUSY	\
	(BLK_ZONE_WPLUG_NEW | BLK_ZONE_WPLUG_PLUGGED | BLK_ZONE_WPLUG_ERROR)

/**
 * blk_zone_cond_str - Return string XXX in BLK_ZONE_COND_XXX.
 * @zone_cond: BLK_ZONE_COND_XXX.
 *
 * Description: Centralize block layer function to convert BLK_ZONE_COND_XXX
 * into string format. Useful in the debugging and tracing zone conditions. For
 * invalid BLK_ZONE_COND_XXX it returns string "UNKNOWN".
 */
const char *blk_zone_cond_str(enum blk_zone_cond zone_cond)
{
	static const char *zone_cond_str = "UNKNOWN";

	if (zone_cond < ARRAY_SIZE(zone_cond_name) && zone_cond_name[zone_cond])
		zone_cond_str = zone_cond_name[zone_cond];

	return zone_cond_str;
}
EXPORT_SYMBOL_GPL(blk_zone_cond_str);

/*
 * Return true if a request is a write requests that needs zone write locking.
 */
bool blk_req_needs_zone_write_lock(struct request *rq)
{
	if (!rq->q->disk->seq_zones_wlock)
		return false;

	return blk_rq_is_seq_zoned_write(rq);
}
EXPORT_SYMBOL_GPL(blk_req_needs_zone_write_lock);

bool blk_req_zone_write_trylock(struct request *rq)
{
	unsigned int zno = blk_rq_zone_no(rq);

	if (test_and_set_bit(zno, rq->q->disk->seq_zones_wlock))
		return false;

	WARN_ON_ONCE(rq->rq_flags & RQF_ZONE_WRITE_LOCKED);
	rq->rq_flags |= RQF_ZONE_WRITE_LOCKED;

	return true;
}
EXPORT_SYMBOL_GPL(blk_req_zone_write_trylock);

void __blk_req_zone_write_lock(struct request *rq)
{
	if (WARN_ON_ONCE(test_and_set_bit(blk_rq_zone_no(rq),
					  rq->q->disk->seq_zones_wlock)))
		return;

	WARN_ON_ONCE(rq->rq_flags & RQF_ZONE_WRITE_LOCKED);
	rq->rq_flags |= RQF_ZONE_WRITE_LOCKED;
}
EXPORT_SYMBOL_GPL(__blk_req_zone_write_lock);

void __blk_req_zone_write_unlock(struct request *rq)
{
	rq->rq_flags &= ~RQF_ZONE_WRITE_LOCKED;
	if (rq->q->disk->seq_zones_wlock)
		WARN_ON_ONCE(!test_and_clear_bit(blk_rq_zone_no(rq),
						 rq->q->disk->seq_zones_wlock));
}
EXPORT_SYMBOL_GPL(__blk_req_zone_write_unlock);

/**
 * bdev_nr_zones - Get number of zones
 * @bdev:	Target device
 *
 * Return the total number of zones of a zoned block device.  For a block
 * device without zone capabilities, the number of zones is always 0.
 */
unsigned int bdev_nr_zones(struct block_device *bdev)
{
	sector_t zone_sectors = bdev_zone_sectors(bdev);

	if (!bdev_is_zoned(bdev))
		return 0;
	return (bdev_nr_sectors(bdev) + zone_sectors - 1) >>
		ilog2(zone_sectors);
}
EXPORT_SYMBOL_GPL(bdev_nr_zones);

/**
 * blkdev_report_zones - Get zones information
 * @bdev:	Target block device
 * @sector:	Sector from which to report zones
 * @nr_zones:	Maximum number of zones to report
 * @cb:		Callback function called for each reported zone
 * @data:	Private data for the callback
 *
 * Description:
 *    Get zone information starting from the zone containing @sector for at most
 *    @nr_zones, and call @cb for each zone reported by the device.
 *    To report all zones in a device starting from @sector, the BLK_ALL_ZONES
 *    constant can be passed to @nr_zones.
 *    Returns the number of zones reported by the device, or a negative errno
 *    value in case of failure.
 *
 *    Note: The caller must use memalloc_noXX_save/restore() calls to control
 *    memory allocations done within this function.
 */
int blkdev_report_zones(struct block_device *bdev, sector_t sector,
			unsigned int nr_zones, report_zones_cb cb, void *data)
{
	struct gendisk *disk = bdev->bd_disk;
	sector_t capacity = get_capacity(disk);

	if (!bdev_is_zoned(bdev) || WARN_ON_ONCE(!disk->fops->report_zones))
		return -EOPNOTSUPP;

	if (!nr_zones || sector >= capacity)
		return 0;

	return disk->fops->report_zones(disk, sector, nr_zones, cb, data);
}
EXPORT_SYMBOL_GPL(blkdev_report_zones);

static inline unsigned long *blk_alloc_zone_bitmap(int node,
						   unsigned int nr_zones)
{
	return kcalloc_node(BITS_TO_LONGS(nr_zones), sizeof(unsigned long),
			    GFP_NOIO, node);
}

static int blk_zone_need_reset_cb(struct blk_zone *zone, unsigned int idx,
				  void *data)
{
	/*
	 * For an all-zones reset, ignore conventional, empty, read-only
	 * and offline zones.
	 */
	switch (zone->cond) {
	case BLK_ZONE_COND_NOT_WP:
	case BLK_ZONE_COND_EMPTY:
	case BLK_ZONE_COND_READONLY:
	case BLK_ZONE_COND_OFFLINE:
		return 0;
	default:
		set_bit(idx, (unsigned long *)data);
		return 0;
	}
}

static int blkdev_zone_reset_all_emulated(struct block_device *bdev)
{
	struct gendisk *disk = bdev->bd_disk;
	sector_t capacity = bdev_nr_sectors(bdev);
	sector_t zone_sectors = bdev_zone_sectors(bdev);
	unsigned long *need_reset;
	struct bio *bio = NULL;
	sector_t sector = 0;
	int ret;

	need_reset = blk_alloc_zone_bitmap(disk->queue->node, disk->nr_zones);
	if (!need_reset)
		return -ENOMEM;

	ret = disk->fops->report_zones(disk, 0, disk->nr_zones,
				       blk_zone_need_reset_cb, need_reset);
	if (ret < 0)
		goto out_free_need_reset;

	ret = 0;
	while (sector < capacity) {
		if (!test_bit(disk_zone_no(disk, sector), need_reset)) {
			sector += zone_sectors;
			continue;
		}

		bio = blk_next_bio(bio, bdev, 0, REQ_OP_ZONE_RESET | REQ_SYNC,
				   GFP_KERNEL);
		bio->bi_iter.bi_sector = sector;
		sector += zone_sectors;

		/* This may take a while, so be nice to others */
		cond_resched();
	}

	if (bio) {
		ret = submit_bio_wait(bio);
		bio_put(bio);
	}

out_free_need_reset:
	kfree(need_reset);
	return ret;
}

static int blkdev_zone_reset_all(struct block_device *bdev)
{
	struct bio bio;

	bio_init(&bio, bdev, NULL, 0, REQ_OP_ZONE_RESET_ALL | REQ_SYNC);
	return submit_bio_wait(&bio);
}

/**
 * blkdev_zone_mgmt - Execute a zone management operation on a range of zones
 * @bdev:	Target block device
 * @op:		Operation to be performed on the zones
 * @sector:	Start sector of the first zone to operate on
 * @nr_sectors:	Number of sectors, should be at least the length of one zone and
 *		must be zone size aligned.
 *
 * Description:
 *    Perform the specified operation on the range of zones specified by
 *    @sector..@sector+@nr_sectors. Specifying the entire disk sector range
 *    is valid, but the specified range should not contain conventional zones.
 *    The operation to execute on each zone can be a zone reset, open, close
 *    or finish request.
 */
int blkdev_zone_mgmt(struct block_device *bdev, enum req_op op,
		     sector_t sector, sector_t nr_sectors)
{
	struct request_queue *q = bdev_get_queue(bdev);
	sector_t zone_sectors = bdev_zone_sectors(bdev);
	sector_t capacity = bdev_nr_sectors(bdev);
	sector_t end_sector = sector + nr_sectors;
	struct bio *bio = NULL;
	int ret = 0;

	if (!bdev_is_zoned(bdev))
		return -EOPNOTSUPP;

	if (bdev_read_only(bdev))
		return -EPERM;

	if (!op_is_zone_mgmt(op))
		return -EOPNOTSUPP;

	if (end_sector <= sector || end_sector > capacity)
		/* Out of range */
		return -EINVAL;

	/* Check alignment (handle eventual smaller last zone) */
	if (!bdev_is_zone_start(bdev, sector))
		return -EINVAL;

	if (!bdev_is_zone_start(bdev, nr_sectors) && end_sector != capacity)
		return -EINVAL;

	/*
	 * In the case of a zone reset operation over all zones,
	 * REQ_OP_ZONE_RESET_ALL can be used with devices supporting this
	 * command. For other devices, we emulate this command behavior by
	 * identifying the zones needing a reset.
	 */
	if (op == REQ_OP_ZONE_RESET && sector == 0 && nr_sectors == capacity) {
		if (!blk_queue_zone_resetall(q))
			return blkdev_zone_reset_all_emulated(bdev);
		return blkdev_zone_reset_all(bdev);
	}

	while (sector < end_sector) {
		bio = blk_next_bio(bio, bdev, 0, op | REQ_SYNC, GFP_KERNEL);
		bio->bi_iter.bi_sector = sector;
		sector += zone_sectors;

		/* This may take a while, so be nice to others */
		cond_resched();
	}

	ret = submit_bio_wait(bio);
	bio_put(bio);

	return ret;
}
EXPORT_SYMBOL_GPL(blkdev_zone_mgmt);

struct zone_report_args {
	struct blk_zone __user *zones;
};

static int blkdev_copy_zone_to_user(struct blk_zone *zone, unsigned int idx,
				    void *data)
{
	struct zone_report_args *args = data;

	if (copy_to_user(&args->zones[idx], zone, sizeof(struct blk_zone)))
		return -EFAULT;
	return 0;
}

/*
 * BLKREPORTZONE ioctl processing.
 * Called from blkdev_ioctl.
 */
int blkdev_report_zones_ioctl(struct block_device *bdev, unsigned int cmd,
		unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct zone_report_args args;
	struct blk_zone_report rep;
	int ret;

	if (!argp)
		return -EINVAL;

	if (!bdev_is_zoned(bdev))
		return -ENOTTY;

	if (copy_from_user(&rep, argp, sizeof(struct blk_zone_report)))
		return -EFAULT;

	if (!rep.nr_zones)
		return -EINVAL;

	args.zones = argp + sizeof(struct blk_zone_report);
	ret = blkdev_report_zones(bdev, rep.sector, rep.nr_zones,
				  blkdev_copy_zone_to_user, &args);
	if (ret < 0)
		return ret;

	rep.nr_zones = ret;
	rep.flags = BLK_ZONE_REP_CAPACITY;
	if (copy_to_user(argp, &rep, sizeof(struct blk_zone_report)))
		return -EFAULT;
	return 0;
}

static int blkdev_truncate_zone_range(struct block_device *bdev,
		blk_mode_t mode, const struct blk_zone_range *zrange)
{
	loff_t start, end;

	if (zrange->sector + zrange->nr_sectors <= zrange->sector ||
	    zrange->sector + zrange->nr_sectors > get_capacity(bdev->bd_disk))
		/* Out of range */
		return -EINVAL;

	start = zrange->sector << SECTOR_SHIFT;
	end = ((zrange->sector + zrange->nr_sectors) << SECTOR_SHIFT) - 1;

	return truncate_bdev_range(bdev, mode, start, end);
}

/*
 * BLKRESETZONE, BLKOPENZONE, BLKCLOSEZONE and BLKFINISHZONE ioctl processing.
 * Called from blkdev_ioctl.
 */
int blkdev_zone_mgmt_ioctl(struct block_device *bdev, blk_mode_t mode,
			   unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct blk_zone_range zrange;
	enum req_op op;
	int ret;

	if (!argp)
		return -EINVAL;

	if (!bdev_is_zoned(bdev))
		return -ENOTTY;

	if (!(mode & BLK_OPEN_WRITE))
		return -EBADF;

	if (copy_from_user(&zrange, argp, sizeof(struct blk_zone_range)))
		return -EFAULT;

	switch (cmd) {
	case BLKRESETZONE:
		op = REQ_OP_ZONE_RESET;

		/* Invalidate the page cache, including dirty pages. */
		filemap_invalidate_lock(bdev->bd_inode->i_mapping);
		ret = blkdev_truncate_zone_range(bdev, mode, &zrange);
		if (ret)
			goto fail;
		break;
	case BLKOPENZONE:
		op = REQ_OP_ZONE_OPEN;
		break;
	case BLKCLOSEZONE:
		op = REQ_OP_ZONE_CLOSE;
		break;
	case BLKFINISHZONE:
		op = REQ_OP_ZONE_FINISH;
		break;
	default:
		return -ENOTTY;
	}

	ret = blkdev_zone_mgmt(bdev, op, zrange.sector, zrange.nr_sectors);

fail:
	if (cmd == BLKRESETZONE)
		filemap_invalidate_unlock(bdev->bd_inode->i_mapping);

	return ret;
}

static inline void blk_zone_wplug_bio_io_error(struct bio *bio)
{
	struct request_queue *q = bio->bi_bdev->bd_disk->queue;

	bio_clear_flag(bio, BIO_ZONE_WRITE_PLUGGING);
	bio_io_error(bio);
	blk_queue_exit(q);
}

static int blk_zone_wplug_abort(struct gendisk *disk,
				struct blk_zone_wplug *zwplug)
{
	struct bio *bio;
	int nr_aborted = 0;

	while ((bio = bio_list_pop(&zwplug->bio_list))) {
		blk_zone_wplug_bio_io_error(bio);
		nr_aborted++;
	}

	return nr_aborted;
}

static void blk_zone_wplug_abort_unaligned(struct gendisk *disk,
					   struct blk_zone_wplug *zwplug)
{
	unsigned int wp_offset = zwplug->wp_offset;
	struct bio_list bl = BIO_EMPTY_LIST;
	struct bio *bio;

	while ((bio = bio_list_pop(&zwplug->bio_list))) {
		if (wp_offset >= disk->zone_capacity ||
		    (bio_op(bio) != REQ_OP_ZONE_APPEND &&
		     bio_offset_from_zone_start(bio) != wp_offset)) {
			blk_zone_wplug_bio_io_error(bio);
			continue;
		}

		wp_offset += bio_sectors(bio);
		bio_list_add(&bl, bio);
	}

	bio_list_merge(&zwplug->bio_list, &bl);
}

static void blk_zone_wplug_bio_work(struct work_struct *work);

static inline struct blk_zone_wplug *bio_alloc_zone_wplug(struct bio *bio)
{
	struct gendisk *disk = bio->bi_bdev->bd_disk;
	sector_t sector = bio->bi_iter.bi_sector;
	unsigned int zno = disk_zone_no(disk, sector);
	struct blk_zone_wplug *zwplug, *zwplg;
	gfp_t gfp_mask = GFP_NOIO;

	if (bio->bi_opf & REQ_NOWAIT)
		gfp_mask = GFP_NOWAIT;

	zwplug = mempool_alloc(disk->zone_wplugs_pool, gfp_mask);
	if (!zwplug)
		return NULL;

	/*
	 * Mark the zone write plug as being a new plug so that
	 * it is not removed until we process the write BIO that
	 * triggered the allocation.
	 */
	spin_lock_init(&zwplug->lock);
	zwplug->flags = BLK_ZONE_WPLUG_NEW;
	bio_list_init(&zwplug->bio_list);
	INIT_WORK(&zwplug->bio_work, blk_zone_wplug_bio_work);
	zwplug->wp_offset = sector & (disk->queue->limits.chunk_sectors - 1);
	zwplug->zone_no = zno;

	/*
	 * Add the new zone write plug to the xarray, but carefully as we are
	 * racing with other submission context, so we may already have a plug.
	 */
	zwplg = xa_cmpxchg(&disk->zone_wplugs, zno, NULL, zwplug, gfp_mask);
	if (zwplg) {
		kmem_cache_free(blk_zone_wplugs_cachep, zwplug);
		if (xa_is_err(zwplg))
			return NULL;
		return zwplg;
	}

	return zwplug;
}

static void disk_free_zone_wplug_rcu(struct rcu_head *head)
{
	struct blk_zone_wplug *zwplug =
		container_of(head, struct blk_zone_wplug, rcu_head);

	mempool_free(zwplug, zwplug->disk->zone_wplugs_pool);
}

static inline void disk_free_zone_wplug(struct gendisk *disk,
					struct blk_zone_wplug *zwplug)
{
	WARN_ON_ONCE(!bio_list_empty(&zwplug->bio_list));

	xa_erase(&disk->zone_wplugs, zwplug->zone_no);

	zwplug->flags |= BLK_ZONE_WPLUG_FREEING;
	zwplug->disk = disk;

	call_rcu(&zwplug->rcu_head, disk_free_zone_wplug_rcu);
}

static inline bool disk_zone_wplug_can_free(struct gendisk *disk,
					    struct blk_zone_wplug *zwplug)
{
	/*
	 * If the zone is still busy or if we already are freeing it,
	 * the plug cannot be freed.
	 */
	if ((zwplug->flags & BLK_ZONE_WPLUG_BUSY) ||
	    (zwplug->flags & BLK_ZONE_WPLUG_FREEING))
		return false;

	/* We can free zone write plugs for zones that are empty or full. */
	return !zwplug->wp_offset || zwplug->wp_offset >= disk->zone_capacity;
}

static inline struct blk_zone_wplug *
disk_lookup_zone_wplug(struct gendisk *disk, sector_t sector)
{
	return xa_load(&disk->zone_wplugs, disk_zone_no(disk, sector));
}

static inline struct blk_zone_wplug *bio_lookup_zone_wplug(struct bio *bio)
{
	return disk_lookup_zone_wplug(bio->bi_bdev->bd_disk,
				      bio->bi_iter.bi_sector);
}

static inline bool blk_zone_wplug_is_conv(struct blk_zone_wplug *zwplug)
{
	return xa_is_value(zwplug) &&
		xa_to_value(zwplug) == BLK_ZONE_WPLUG_CONV;
}

/*
 * Set a zone write plug write pointer offset to either 0 (zone reset case)
 * or to the zone size (zone finish case). This aborts all plugged BIOs, which
 * is fine to do as doing a zone reset or zone finish while writes are in-flight
 * is a mistake from the user which will most likely cause all plugged BIOs to
 * fail anyway.
 */
static void blk_zone_wplug_set_wp_offset(struct gendisk *disk,
					 struct blk_zone_wplug *zwplug,
					 unsigned int wp_offset)
{
	unsigned long flags;

	spin_lock_irqsave(&zwplug->lock, flags);

	if (zwplug->flags & BLK_ZONE_WPLUG_FREEING) {
		spin_unlock_irqrestore(&zwplug->lock, flags);
		return;
	}

	/*
	 * Updating the write pointer offset puts back the zone
	 * in a good state. So clear the error flag and decrement the
	 * error count if we were in error state.
	 */
	if (zwplug->flags & BLK_ZONE_WPLUG_ERROR) {
		zwplug->flags &= ~BLK_ZONE_WPLUG_ERROR;
		atomic_dec(&disk->zone_nr_wplugs_with_error);
	}

	/* Update the zone write pointer and abort all plugged BIOs. */
	zwplug->wp_offset = wp_offset;
	blk_zone_wplug_abort(disk, zwplug);

	/*
	 * The zone write plug now has no BIO plugged. If there is no
	 * on-going BIOs, remove and free the plug.
	 */
	if (disk_zone_wplug_can_free(disk, zwplug))
		disk_free_zone_wplug(disk, zwplug);

	spin_unlock_irqrestore(&zwplug->lock, flags);
}

static bool blk_zone_wplug_handle_reset_or_finish(struct bio *bio,
						  unsigned int wp_offset)
{
	struct blk_zone_wplug *zwplug;

	/*
	 * If we have a zone write plug, set its write pointer offset to 0
	 * (reset case) or to the zone size (finish case). This will abort all
	 * BIOs plugged for the target zone. It is fine as resetting or
	 * finishing zones while writes are still in-flight will result in the
	 * writes failing anyway.
	 */
	rcu_read_lock();

	zwplug = bio_lookup_zone_wplug(bio);
	if (blk_zone_wplug_is_conv(zwplug)) {
		/* Conventional zones cannot be reset nor finished. */
		rcu_read_unlock();
		bio_io_error(bio);
		return true;
	}

	if (zwplug)
		blk_zone_wplug_set_wp_offset(bio->bi_bdev->bd_disk, zwplug,
					     wp_offset);
	rcu_read_unlock();

	return false;
}

static bool blk_zone_wplug_handle_reset_all(struct bio *bio)
{
	struct gendisk *disk = bio->bi_bdev->bd_disk;
	XA_STATE(xas, &disk->zone_wplugs, 0);
	struct blk_zone_wplug *zwplug;

	/*
	 * Set the write pointer offset of all zone write plugs to 0. This will
	 * abort all plugged BIOs. It is fine as resetting zones while writes
	 * are still in-flight will result in the writes failing anyway.
	 */
	rcu_read_lock();
	xas_for_each(&xas, zwplug, disk->nr_zones - 1) {
		if (xas_retry(&xas, zwplug))
			continue;
		if (blk_zone_wplug_is_conv(zwplug))
			continue;
		blk_zone_wplug_set_wp_offset(disk, zwplug, 0);
	}
	rcu_read_unlock();

	return false;
}

static inline void blk_zone_wplug_add_bio(struct blk_zone_wplug *zwplug,
					  struct bio *bio, unsigned int nr_segs)
{
	/*
	 * Grab an extra reference on the BIO request queue usage counter.
	 * This reference will be reused to submit a request for the BIO for
	 * blk-mq devices and dropped when the BIO is failed and after
	 * it is issued in the case of BIO-based devices.
	 */
	percpu_ref_get(&bio->bi_bdev->bd_disk->queue->q_usage_counter);

	/*
	 * The BIO is being plugged and thus will have to wait for the on-going
	 * write and for all other writes already plugged. So polling makes
	 * no sense.
	 */
	bio_clear_polled(bio);

	/*
	 * Reuse the poll cookie field to store the number of segments when
	 * split to the hardware limits.
	 */
	bio->__bi_nr_segments = nr_segs;

	/*
	 * We always receive BIOs after they are split and ready to be issued.
	 * The block layer passes the parts of a split BIO in order, and the
	 * user must also issue write sequentially. So simply add the new BIO
	 * at the tail of the list to preserve the sequential write order.
	 */
	bio_list_add(&zwplug->bio_list, bio);
}

/*
 * Called from bio_attempt_back_merge() when a BIO was merged with a request.
 */
void blk_zone_write_plug_bio_merged(struct bio *bio)
{
	struct blk_zone_wplug *zwplug = bio_lookup_zone_wplug(bio);
	unsigned long flags;

	/*
	 * If the BIO was already plugged, then this we were called through
	 * blk_zone_write_plug_attempt_merge() -> blk_attempt_bio_merge().
	 * For this case, blk_zone_write_plug_attempt_merge() will handle the
	 * zone write pointer offset update.
	 */
	if (bio_flagged(bio, BIO_ZONE_WRITE_PLUGGING))
		return;

	spin_lock_irqsave(&zwplug->lock, flags);

	bio_set_flag(bio, BIO_ZONE_WRITE_PLUGGING);

	/* Advance the zone write pointer offset. */
	zwplug->wp_offset += bio_sectors(bio);

	spin_unlock_irqrestore(&zwplug->lock, flags);
}

/*
 * Attempt to merge plugged BIOs with a newly formed request of a BIO that went
 * through zone write plugging (either a new BIO or one that was unplugged).
 */
void blk_zone_write_plug_attempt_merge(struct request *req)
{
	struct blk_zone_wplug *zwplug = bio_lookup_zone_wplug(req->bio);
	sector_t req_back_sector = blk_rq_pos(req) + blk_rq_sectors(req);
	struct request_queue *q = req->q;
	unsigned int zone_capacity = q->disk->zone_capacity;
	unsigned long flags;
	struct bio *bio;

	/*
	 * Completion of this request needs to be handled with
	 * blk_zone_write_complete_request().
	 */
	req->rq_flags |= RQF_ZONE_WRITE_PLUGGING;

	if (blk_queue_nomerges(q))
		return;

	/*
	 * Walk through the list of plugged BIOs to check if they can be merged
	 * into the back of the request.
	 */
	spin_lock_irqsave(&zwplug->lock, flags);
	while (zwplug->wp_offset < zone_capacity &&
	       (bio = bio_list_peek(&zwplug->bio_list))) {
		if (bio->bi_iter.bi_sector != req_back_sector ||
		    !blk_rq_merge_ok(req, bio))
			break;

		WARN_ON_ONCE(bio_op(bio) != REQ_OP_WRITE_ZEROES &&
			     !bio->__bi_nr_segments);

		bio_list_pop(&zwplug->bio_list);
		if (bio_attempt_back_merge(req, bio, bio->__bi_nr_segments) !=
		    BIO_MERGE_OK) {
			bio_list_add_head(&zwplug->bio_list, bio);
			break;
		}

		/*
		 * Drop the extra reference on the queue usage we got when
		 * plugging the BIO and advance the write pointer offset.
		 */
		blk_queue_exit(q);
		zwplug->wp_offset += bio_sectors(bio);

		req_back_sector += bio_sectors(bio);
	}
	spin_unlock_irqrestore(&zwplug->lock, flags);
}

static inline void blk_zone_wplug_set_error(struct gendisk *disk,
					    struct blk_zone_wplug *zwplug)
{
	if (!(zwplug->flags & BLK_ZONE_WPLUG_ERROR)) {
		zwplug->flags |= BLK_ZONE_WPLUG_ERROR;
		atomic_inc(&disk->zone_nr_wplugs_with_error);
	}
}

/*
 * Check and prepare a BIO for submission by incrementing the write pointer
 * offset of its zone write plug and changing zone append operations into
 * regular write when zone append emulation is needed.
 */
static bool blk_zone_wplug_prepare_bio(struct blk_zone_wplug *zwplug,
				       struct bio *bio)
{
	/*
	 * Check that the user is not attempting to write to a full zone.
	 * We know such BIO will fail, and that would potentially overflow our
	 * write pointer offset beyond the end of the zone.
	 */
	if (zwplug->wp_offset >= bio->bi_bdev->bd_disk->zone_capacity)
		goto err;

	if (bio_op(bio) == REQ_OP_ZONE_APPEND) {
		/*
		 * Use a regular write starting at the current write pointer.
		 * Similarly to native zone append operations, do not allow
		 * merging.
		 */
		bio->bi_opf &= ~REQ_OP_MASK;
		bio->bi_opf |= REQ_OP_WRITE | REQ_NOMERGE;
		bio->bi_iter.bi_sector += zwplug->wp_offset;

		/*
		 * Remember that this BIO is in fact a zone append operation
		 * so that we can restore its operation code on completion.
		 */
		bio_set_flag(bio, BIO_EMULATES_ZONE_APPEND);
	} else {
		/*
		 * Check for non-sequential writes early because we avoid a
		 * whole lot of error handling trouble if we don't send it off
		 * to the driver.
		 */
		if (bio_offset_from_zone_start(bio) != zwplug->wp_offset)
			goto err;
	}

	/* Advance the zone write pointer offset. */
	zwplug->wp_offset += bio_sectors(bio);

	return true;

err:
	/* We detected an invalid write BIO: schedule error recovery. */
	blk_zone_wplug_set_error(bio->bi_bdev->bd_disk, zwplug);
	kblockd_mod_delayed_work_on(WORK_CPU_UNBOUND,
				&bio->bi_bdev->bd_disk->zone_wplugs_work, 0);
	return false;
}

static bool blk_zone_wplug_handle_write(struct bio *bio, unsigned int nr_segs)
{
	struct blk_zone_wplug *zwplug;
	unsigned long flags;

	/*
	 * BIOs must be fully contained within a zone so that we use the correct
	 * zone write plug for the entire BIO. For blk-mq devices, the block
	 * layer should already have done any splitting required to ensure this
	 * and this BIO should thus not be straddling zone boundaries. For
	 * BIO-based devices, it is the responsibility of the driver to split
	 * the bio before submitting it.
	 */
	if (WARN_ON_ONCE(bio_straddle_zones(bio))) {
		bio_io_error(bio);
		return true;
	}

lookup:
	rcu_read_lock();

	zwplug = bio_lookup_zone_wplug(bio);
	if (blk_zone_wplug_is_conv(zwplug)) {
		/* Conventional zones do not need write plugging. */
		rcu_read_unlock();
		if (bio_op(bio) == REQ_OP_ZONE_APPEND) {
			/* Zone append to conventional zones is not allowed. */
			bio_io_error(bio);
			return true;
		}
		return false;
	}

	if (!zwplug) {
		/* Allocate a new plug. */
		rcu_read_unlock();
		zwplug = bio_alloc_zone_wplug(bio);
		if (!zwplug) {
			bio_io_error(bio);
			return true;
		}
		rcu_read_lock();
	}

	spin_lock_irqsave(&zwplug->lock, flags);

	/*
	 * The zone write plug may have been freed before we locked it, if
	 * a BIO completed fully writing the zone, or if a zone reset or zone
	 * finish was executed. We cannot reuse the zone write plug in such
	 * case. So go back to get a new one.
	 */
	if (zwplug->flags & BLK_ZONE_WPLUG_FREEING) {
		spin_unlock_irqrestore(&zwplug->lock, flags);
		rcu_read_unlock();
		goto lookup;
	}

	rcu_read_unlock();

	/*
	 * If the zone write plug was just allocated, it is flagged with
	 * BLK_ZONE_WPLUG_NEW. With the plug now locked, we can safely clear
	 * this flag without risking that the plug is freed under us.
	 */
	zwplug->flags &= ~BLK_ZONE_WPLUG_NEW;

	/* Indicate that this BIO is being handled using zone write plugging. */
	bio_set_flag(bio, BIO_ZONE_WRITE_PLUGGING);

	/*
	 * If the zone is already plugged or has a pending error, add the BIO
	 * to the plug BIO list. Otherwise, plug and let the BIO execute.
	 */
	if (zwplug->flags & (BLK_ZONE_WPLUG_PLUGGED | BLK_ZONE_WPLUG_ERROR))
		goto plug;

	/*
	 * If an error is detected when preparing the BIO, add it to the BIO
	 * list so that error recovery can deal with it.
	 */
	if (!blk_zone_wplug_prepare_bio(zwplug, bio))
		goto plug;

	zwplug->flags |= BLK_ZONE_WPLUG_PLUGGED;

	spin_unlock_irqrestore(&zwplug->lock, flags);

	return false;

plug:
	zwplug->flags |= BLK_ZONE_WPLUG_PLUGGED;
	blk_zone_wplug_add_bio(zwplug, bio, nr_segs);

	spin_unlock_irqrestore(&zwplug->lock, flags);

	return true;
}

/**
 * blk_zone_write_plug_bio - Handle a zone write BIO with zone write plugging
 * @bio: The BIO being submitted
 * @nr_segs: The number of physical segments of @bio
 *
 * Handle write, write zeroes and zone append operations requiring emulation
 * using zone write plugging. Return true whenever @bio execution needs to be
 * delayed through the zone write plug. Otherwise, return false to let the
 * submission path process @bio normally.
 */
bool blk_zone_write_plug_bio(struct bio *bio, unsigned int nr_segs)
{
	struct block_device *bdev = bio->bi_bdev;

	/*
	 * If the BIO already has the plugging flag set, then it was already
	 * handled through this path and this is a submission from the zone
	 * plug bio submit work.
	 */
	if (bio_flagged(bio, BIO_ZONE_WRITE_PLUGGING))
		return false;

	/*
	 * We do not need to do anything special for empty flush BIOs, e.g
	 * BIOs such as issued by blkdev_issue_flush(). The is because it is
	 * the responsibility of the user to first wait for the completion of
	 * write operations for flush to have any effect on the persistence of
	 * the written data.
	 */
	if (op_is_flush(bio->bi_opf) && !bio_sectors(bio))
		return false;

	/*
	 * Regular writes and write zeroes need to be handled through the target
	 * zone write plug. This includes writes with REQ_FUA | REQ_PREFLUSH
	 * which may need to go through the flush machinery depending on the
	 * target device capabilities. Plugging such writes is fine as the flush
	 * machinery operates at the request level, below the plug, and
	 * completion of the flush sequence will go through the regular BIO
	 * completion, which will handle zone write plugging.
	 * Zone append operations for devices that requested emulation must
	 * also be plugged so that these BIOs can be changed into regular
	 * write BIOs.
	 * Zone reset, reset all and finish commands need special treatment
	 * to correctly track the write pointer offset of zones. These commands
	 * are not plugged as we do not need serialization with write
	 * operations. It is the responsibility of the user to not issue reset
	 * and finish commands when write operations are in flight.
	 */
	switch (bio_op(bio)) {
	case REQ_OP_ZONE_APPEND:
		if (!bdev_emulates_zone_append(bdev))
			return false;
		fallthrough;
	case REQ_OP_WRITE:
	case REQ_OP_WRITE_ZEROES:
		return blk_zone_wplug_handle_write(bio, nr_segs);
	case REQ_OP_ZONE_RESET:
		return blk_zone_wplug_handle_reset_or_finish(bio, 0);
	case REQ_OP_ZONE_FINISH:
		return blk_zone_wplug_handle_reset_or_finish(bio,
						bdev_zone_sectors(bdev));
	case REQ_OP_ZONE_RESET_ALL:
		return blk_zone_wplug_handle_reset_all(bio);
	default:
		return false;
	}

	return false;
}
EXPORT_SYMBOL_GPL(blk_zone_write_plug_bio);

static void blk_zone_write_plug_unplug_bio(struct gendisk *disk,
					   struct blk_zone_wplug *zwplug)
{
	unsigned long flags;

	spin_lock_irqsave(&zwplug->lock, flags);

	/*
	 * If we had an error, schedule error recovery. The recovery work
	 * will restart submission of plugged BIOs.
	 */
	if (zwplug->flags & BLK_ZONE_WPLUG_ERROR) {
		spin_unlock_irqrestore(&zwplug->lock, flags);
		kblockd_mod_delayed_work_on(WORK_CPU_UNBOUND,
					    &disk->zone_wplugs_work, 0);
		return;
	}

	/* Schedule submission of the next plugged BIO if we have one. */
	if (!bio_list_empty(&zwplug->bio_list)) {
		spin_unlock_irqrestore(&zwplug->lock, flags);
		kblockd_schedule_work(&zwplug->bio_work);
		return;
	}

	zwplug->flags &= ~BLK_ZONE_WPLUG_PLUGGED;

	/*
	 * If the zone is full (it was fully written or finished, or empty
	 * (it was reset), free its zone write plug.
	 */
	if (disk_zone_wplug_can_free(disk, zwplug))
		disk_free_zone_wplug(disk, zwplug);

	spin_unlock_irqrestore(&zwplug->lock, flags);
}

void blk_zone_write_plug_bio_endio(struct bio *bio)
{
	struct gendisk *disk = bio->bi_bdev->bd_disk;
	struct blk_zone_wplug *zwplug = bio_lookup_zone_wplug(bio);
	unsigned long flags;

	/* Make sure we do not see this BIO again by clearing the plug flag. */
	bio_clear_flag(bio, BIO_ZONE_WRITE_PLUGGING);

	/*
	 * If this is a regular write emulating a zone append operation,
	 * restore the original operation code.
	 */
	if (bio_flagged(bio, BIO_EMULATES_ZONE_APPEND)) {
		bio->bi_opf &= ~REQ_OP_MASK;
		bio->bi_opf |= REQ_OP_ZONE_APPEND;
	}

	/*
	 * If the BIO failed, mark the plug as having an error to trigger
	 * recovery.
	 */
	if (bio->bi_status != BLK_STS_OK) {
		spin_lock_irqsave(&zwplug->lock, flags);
		blk_zone_wplug_set_error(disk, zwplug);
		spin_unlock_irqrestore(&zwplug->lock, flags);
	}

	/*
	 * For BIO-based devices, blk_zone_write_plug_complete_request()
	 * is not called. So we need to schedule execution of the next
	 * plugged BIO here.
	 */
	if (bio->bi_bdev->bd_has_submit_bio)
		blk_zone_write_plug_unplug_bio(disk, zwplug);
}

void blk_zone_write_plug_complete_request(struct request *req)
{
	struct gendisk *disk = req->q->disk;
	struct blk_zone_wplug *zwplug =
		disk_lookup_zone_wplug(disk, req->__sector);

	req->rq_flags &= ~RQF_ZONE_WRITE_PLUGGING;

	blk_zone_write_plug_unplug_bio(disk, zwplug);
}

static void blk_zone_wplug_bio_work(struct work_struct *work)
{
	struct blk_zone_wplug *zwplug =
		container_of(work, struct blk_zone_wplug, bio_work);
	struct block_device *bdev;
	unsigned long flags;
	struct bio *bio;

	/*
	 * Unplug and submit the next plugged BIO. If we do not have any, clear
	 * the plugged flag.
	 */
	spin_lock_irqsave(&zwplug->lock, flags);

	bio = bio_list_pop(&zwplug->bio_list);
	if (!bio) {
		zwplug->flags &= ~BLK_ZONE_WPLUG_PLUGGED;
		spin_unlock_irqrestore(&zwplug->lock, flags);
		return;
	}

	if (!blk_zone_wplug_prepare_bio(zwplug, bio)) {
		/* Error recovery will decide what to do with the BIO. */
		bio_list_add_head(&zwplug->bio_list, bio);
		spin_unlock_irqrestore(&zwplug->lock, flags);
		return;
	}

	spin_unlock_irqrestore(&zwplug->lock, flags);

	bdev = bio->bi_bdev;
	submit_bio_noacct_nocheck(bio);

	/*
	 * blk-mq devices will reuse the extra reference on the request queue
	 * usage counter we took when the BIO was plugged, but the submission
	 * path for BIO-based devices will not do that. So drop this extra
	 * reference here.
	 */
	if (bdev->bd_has_submit_bio)
		blk_queue_exit(bdev->bd_disk->queue);
}

static unsigned int blk_zone_wp_offset(struct blk_zone *zone)
{
	switch (zone->cond) {
	case BLK_ZONE_COND_IMP_OPEN:
	case BLK_ZONE_COND_EXP_OPEN:
	case BLK_ZONE_COND_CLOSED:
		return zone->wp - zone->start;
	case BLK_ZONE_COND_FULL:
		return zone->len;
	case BLK_ZONE_COND_EMPTY:
		return 0;
	case BLK_ZONE_COND_NOT_WP:
	case BLK_ZONE_COND_OFFLINE:
	case BLK_ZONE_COND_READONLY:
	default:
		/*
		 * Conventional, offline and read-only zones do not have a valid
		 * write pointer.
		 */
		return UINT_MAX;
	}
}

static int blk_zone_wplug_get_zone_cb(struct blk_zone *zone,
				      unsigned int idx, void *data)
{
	struct blk_zone *zonep = data;

	*zonep = *zone;
	return 0;
}

static void blk_zone_wplug_handle_error(struct gendisk *disk,
					struct blk_zone_wplug *zwplug)
{
	sector_t zone_start_sector =
		bdev_zone_sectors(disk->part0) * zwplug->zone_no;
	unsigned int noio_flag;
	struct blk_zone zone;
	unsigned long flags;
	int ret;

	/*
	 * Check if we still have an error as a zone reset or zone finish
	 * may have cleared the error already.
	 */
	spin_lock_irqsave(&zwplug->lock, flags);
	if (!(zwplug->flags & BLK_ZONE_WPLUG_ERROR))
		goto unlock;
	spin_unlock_irqrestore(&zwplug->lock, flags);

	/* Get the current zone information from the device. */
	noio_flag = memalloc_noio_save();
	ret = disk->fops->report_zones(disk, zone_start_sector, 1,
				       blk_zone_wplug_get_zone_cb, &zone);
	memalloc_noio_restore(noio_flag);

	spin_lock_irqsave(&zwplug->lock, flags);

	/* A zone reset or finish may have cleared the error already. */
	if (!(zwplug->flags & BLK_ZONE_WPLUG_ERROR))
		goto unlock;

	/* Clear the error. */
	zwplug->flags &= ~BLK_ZONE_WPLUG_ERROR;
	atomic_dec(&disk->zone_nr_wplugs_with_error);

	if (ret != 1) {
		/*
		 * We failed to get the zone information, likely meaning that
		 * something is really wrong with the device. Abort all
		 * remaining plugged BIOs as otherwise we could endup waiting
		 * forever on plugged BIOs to complete if there is a revalidate
		 * or queue freeze on-going.
		 */
		blk_zone_wplug_abort(disk, zwplug);
		goto unplug;
	}

	/* Update the zone write pointer offset. */
	zwplug->wp_offset = blk_zone_wp_offset(&zone);

	blk_zone_wplug_abort_unaligned(disk, zwplug);

	/* Restart BIO submission if we still have any BIO left. */
	if (!bio_list_empty(&zwplug->bio_list)) {
		WARN_ON_ONCE(!(zwplug->flags & BLK_ZONE_WPLUG_PLUGGED));
		kblockd_schedule_work(&zwplug->bio_work);
		goto unlock;
	}

unplug:
	zwplug->flags &= ~BLK_ZONE_WPLUG_PLUGGED;
	if (disk_zone_wplug_can_free(disk, zwplug))
		disk_free_zone_wplug(disk, zwplug);

unlock:
	spin_unlock_irqrestore(&zwplug->lock, flags);
}

static void disk_zone_wplugs_work(struct work_struct *work)
{
	struct gendisk *disk =
		container_of(work, struct gendisk, zone_wplugs_work.work);
	struct blk_zone_wplug *zwplug;
	unsigned long idx;

	while (atomic_read(&disk->zone_nr_wplugs_with_error)) {
		rcu_read_lock();
		xa_for_each_range(&disk->zone_wplugs, idx, zwplug,
				  0, disk->nr_zones - 1) {
			rcu_read_unlock();
			blk_zone_wplug_handle_error(disk, zwplug);
			rcu_read_lock();
		}
		rcu_read_unlock();
	}
}

void disk_init_zone_resources(struct gendisk *disk)
{
	xa_init_flags(&disk->zone_wplugs, XA_FLAGS_LOCK_IRQ);
	atomic_set(&disk->zone_nr_wplugs_with_error, 0);
	INIT_DELAYED_WORK(&disk->zone_wplugs_work, disk_zone_wplugs_work);
}

void disk_free_zone_resources(struct gendisk *disk)
{
	struct blk_zone_wplug *zwplug;
	XA_STATE(xas, &disk->zone_wplugs, 0);

	cancel_delayed_work_sync(&disk->zone_wplugs_work);

	rcu_read_lock();
	xas_for_each(&xas, zwplug, disk->nr_zones - 1) {
		if (xas_retry(&xas, zwplug))
			continue;
		disk_free_zone_wplug(disk, zwplug);
	}
	rcu_read_unlock();

	rcu_barrier();

	xa_destroy(&disk->zone_wplugs);

	mempool_destroy(disk->zone_wplugs_pool);
	disk->zone_wplugs_pool = NULL;
	disk->zone_wplugs_pool_size = 0;

	kfree(disk->conv_zones_bitmap);
	disk->conv_zones_bitmap = NULL;
	kfree(disk->seq_zones_wlock);
	disk->seq_zones_wlock = NULL;
	disk->zone_capacity = 0;
	disk->nr_zones = 0;
}

#define BLK_ZONE_DEFAULT_WPLUG_POOL_SIZE	128

static int disk_init_zone_wplug_mempool(struct gendisk *disk,
					unsigned int nr_zones)
{
	struct queue_limits *lim = &disk->queue->limits;
	unsigned int pool_size;
	int ret;

	/*
	 * Size the disk mempool of zone write plugs with enough elements
	 * given the device open and active zones limits. When there are no
	 * device limits, we use BLK_ZONE_DEFAULT_WPLUG_POOL_SIZE.
	 */
	pool_size = max(lim->max_active_zones, lim->max_open_zones);
	if (!pool_size)
		pool_size = BLK_ZONE_DEFAULT_WPLUG_POOL_SIZE;
	pool_size = min(pool_size, nr_zones);

 	if (!disk->zone_wplugs_pool) {
		disk->zone_wplugs_pool_size = pool_size;
		disk->zone_wplugs_pool =
			mempool_create_slab_pool(disk->zone_wplugs_pool_size,
						 blk_zone_wplugs_cachep);
		if (!disk->zone_wplugs_pool)
			ret = -ENOMEM;
	} else if (disk->zone_wplugs_pool_size != pool_size) {
		ret = mempool_resize(disk->zone_wplugs_pool, pool_size);
	}

	return ret;
}

struct blk_revalidate_zone_args {
	struct gendisk	*disk;
	unsigned long	*conv_zones_bitmap;
	unsigned long	*seq_zones_wlock;
	unsigned int	zone_capacity;
	unsigned int	nr_zones;
	sector_t	sector;
};

/*
 * Helper function to check the validity of zones of a zoned block device.
 */
static int blk_revalidate_zone_cb(struct blk_zone *zone, unsigned int idx,
				  void *data)
{
	struct blk_revalidate_zone_args *args = data;
	struct gendisk *disk = args->disk;
	struct request_queue *q = disk->queue;
	sector_t capacity = get_capacity(disk);
	sector_t zone_sectors = q->limits.chunk_sectors;
	struct blk_zone_wplug *zwplug;
	unsigned long flags;

	/* Check for bad zones and holes in the zone report */
	if (zone->start != args->sector) {
		pr_warn("%s: Zone gap at sectors %llu..%llu\n",
			disk->disk_name, args->sector, zone->start);
		return -ENODEV;
	}

	if (zone->start >= capacity || !zone->len) {
		pr_warn("%s: Invalid zone start %llu, length %llu\n",
			disk->disk_name, zone->start, zone->len);
		return -ENODEV;
	}

	/*
	 * All zones must have the same size, with the exception on an eventual
	 * smaller last zone.
	 */
	if (zone->start + zone->len < capacity) {
		if (zone->len != zone_sectors) {
			pr_warn("%s: Invalid zoned device with non constant zone size\n",
				disk->disk_name);
			return -ENODEV;
		}
	} else if (zone->len > zone_sectors) {
		pr_warn("%s: Invalid zoned device with larger last zone size\n",
			disk->disk_name);
		return -ENODEV;
	}

	/* Check zone type */
	switch (zone->type) {
	case BLK_ZONE_TYPE_CONVENTIONAL:
		if (!args->conv_zones_bitmap) {
			args->conv_zones_bitmap =
				blk_alloc_zone_bitmap(q->node, args->nr_zones);
			if (!args->conv_zones_bitmap)
				return -ENOMEM;
		}
		set_bit(idx, args->conv_zones_bitmap);
		/*
		 * Store the conventional zone type as a value in the xarray.
		 * If the previous entry was a valid plug, free it.
		 */
		zwplug = xa_store(&disk->zone_wplugs, idx,
				  xa_mk_value(BLK_ZONE_WPLUG_CONV), GFP_KERNEL);
		if (xa_is_err(zwplug))
			return xa_err(zwplug);
		if (zwplug && !xa_is_value(zwplug)) {
			spin_lock_irqsave(&zwplug->lock, flags);
			disk_free_zone_wplug(disk, zwplug);
			spin_unlock_irqrestore(&zwplug->lock, flags);
		}
		break;
	case BLK_ZONE_TYPE_SEQWRITE_REQ:
		if (!args->seq_zones_wlock) {
			args->seq_zones_wlock =
				blk_alloc_zone_bitmap(q->node, args->nr_zones);
			if (!args->seq_zones_wlock)
				return -ENOMEM;
		}

		/*
		 * Remember the capacity of the first sequential zone and check
		 * if it is constant for all zones.
		 */
		if (!args->zone_capacity)
			args->zone_capacity = zone->capacity;
		if (zone->capacity != args->zone_capacity) {
			pr_warn("%s: Invalid variable zone capacity\n",
				disk->disk_name);
			return -ENODEV;
		}
		break;
	case BLK_ZONE_TYPE_SEQWRITE_PREF:
	default:
		pr_warn("%s: Invalid zone type 0x%x at sectors %llu\n",
			disk->disk_name, (int)zone->type, zone->start);
		return -ENODEV;
	}

	args->sector += zone->len;
	return 0;
}

/**
 * blk_revalidate_disk_zones - (re)allocate and initialize zone bitmaps
 * @disk:	Target disk
 * @update_driver_data:	Callback to update driver data on the frozen disk
 *
 * Helper function for low-level device drivers to check and (re) allocate and
 * initialize a disk request queue zone bitmaps. This functions should normally
 * be called within the disk ->revalidate method for blk-mq based drivers.
 * Before calling this function, the device driver must already have set the
 * device zone size (chunk_sector limit) and the max zone append limit.
 * For BIO based drivers, this function cannot be used. BIO based device drivers
 * only need to set disk->nr_zones so that the sysfs exposed value is correct.
 * If the @update_driver_data callback function is not NULL, the callback is
 * executed with the device request queue frozen after all zones have been
 * checked.
 */
int blk_revalidate_disk_zones(struct gendisk *disk,
			      void (*update_driver_data)(struct gendisk *disk))
{
	struct request_queue *q = disk->queue;
	sector_t zone_sectors = q->limits.chunk_sectors;
	sector_t capacity = get_capacity(disk);
	struct blk_revalidate_zone_args args = { };
	unsigned int noio_flag;
	int ret = -ENOMEM;

	if (WARN_ON_ONCE(!blk_queue_is_zoned(q)))
		return -EIO;
	if (WARN_ON_ONCE(!queue_is_mq(q)))
		return -EIO;

	if (!capacity)
		return -ENODEV;

	/*
	 * Checks that the device driver indicated a valid zone size and that
	 * the max zone append limit is set.
	 */
	if (!zone_sectors || !is_power_of_2(zone_sectors)) {
		pr_warn("%s: Invalid non power of two zone size (%llu)\n",
			disk->disk_name, zone_sectors);
		return -ENODEV;
	}

	if (!queue_max_zone_append_sectors(q)) {
		pr_warn("%s: Invalid 0 maximum zone append limit\n",
			disk->disk_name);
		return -ENODEV;
	}

	/*
	 * Ensure that all memory allocations in this context are done as if
	 * GFP_NOIO was specified.
	 */
	noio_flag = memalloc_noio_save();
	args.disk = disk;
	args.nr_zones = (capacity + zone_sectors - 1) >> ilog2(zone_sectors);

	ret = disk_init_zone_wplug_mempool(disk, args.nr_zones);
	if (ret) {
		memalloc_noio_restore(noio_flag);
		return -ENODEV;
	}

	ret = disk->fops->report_zones(disk, 0, UINT_MAX,
				       blk_revalidate_zone_cb, &args);
	if (!ret) {
		pr_warn("%s: No zones reported\n", disk->disk_name);
		ret = -ENODEV;
	}
	memalloc_noio_restore(noio_flag);

	/*
	 * If zones where reported, make sure that the entire disk capacity
	 * has been checked.
	 */
	if (ret > 0 && args.sector != capacity) {
		pr_warn("%s: Missing zones from sector %llu\n",
			disk->disk_name, args.sector);
		ret = -ENODEV;
	}

	/*
	 * Install the new bitmaps and update nr_zones only once the queue is
	 * stopped and all I/Os are completed (i.e. a scheduler is not
	 * referencing the bitmaps).
	 */
	blk_mq_freeze_queue(q);
	if (ret > 0) {
		disk->nr_zones = args.nr_zones;
		swap(disk->seq_zones_wlock, args.seq_zones_wlock);
		swap(disk->conv_zones_bitmap, args.conv_zones_bitmap);
		disk->zone_capacity = args.zone_capacity;
		if (update_driver_data)
			update_driver_data(disk);
		ret = 0;
	} else {
		pr_warn("%s: failed to revalidate zones\n", disk->disk_name);
		disk_free_zone_resources(disk);
	}
	blk_mq_unfreeze_queue(q);

	kfree(args.seq_zones_wlock);
	kfree(args.conv_zones_bitmap);

	return ret;
}
EXPORT_SYMBOL_GPL(blk_revalidate_disk_zones);

void blk_zone_dev_init(void)
{
	blk_zone_wplugs_cachep = KMEM_CACHE(blk_zone_wplug, SLAB_PANIC);
}
