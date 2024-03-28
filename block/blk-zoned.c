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
#include <linux/atomic.h>

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

/*
 * Per-zone write plug.
 * @node: hlist_node structure for managing the plug using a hash table.
 * @link: To list the plug in the zone write plug free list or error list of
 *        the disk.
 * @ref: Zone write plug reference counter. A zone write plug reference is
 *       always at least 1 when the plug is hashed in the disk plug hash table.
 *       The reference is incremented whenever a new BIO needing plugging is
 *       submitted and when a function needs to manipulate a plug. The
 *       reference count is decremented whenever a plugged BIO completes and
 *       when a function that referenced the plug returns. The initial
 *       reference is dropped whenever the zone of the zone write plug is reset,
 *       finished and when the zone becomes full (last write BIO to the zone
 *       completes).
 * @lock: Spinlock to atomically manipulate the plug.
 * @flags: Flags indicating the plug state.
 * @zone_no: The number of the zone the plug is managing.
 * @wp_offset: The zone write pointer location relative to the start of the zone
 *             as a number of 512B sectors.
 * @bio_list: The list of BIOs that are currently plugged.
 * @bio_work: Work struct to handle issuing of plugged BIOs
 * @rcu_head: RCU head to free zone write plugs with an RCU grace period.
 * @disk: The gendisk the plug belongs to.
 */
struct blk_zone_wplug {
	struct hlist_node	node;
	struct list_head	link;
	atomic_t		ref;
	spinlock_t		lock;
	unsigned int		flags;
	unsigned int		zone_no;
	unsigned int		wp_offset;
	struct bio_list		bio_list;
	struct work_struct	bio_work;
	struct rcu_head		rcu_head;
	struct gendisk		*disk;
};

/*
 * Zone write plug flags bits:
 *  - BLK_ZONE_WPLUG_PLUGGED: Indicate that the zone write plug is plugged,
 *    that is, that write BIOs are being throttled due to a write BIO already
 *    being executed or the zone write plug bio list is not empty.
 *  - BLK_ZONE_WPLUG_ERROR: Indicate that a write error happened which will be
 *    recovered with a report zone to update the zone write pointer offset.
 *  - BLK_ZONE_WPLUG_UNHASHED: Indicates that the zone write plug was removed
 *    from the disk hash table and that the initial reference to the zone
 *    write plug set when the plug was first added to the hash table has been
 *    dropped. This flag is set when a zone is reset, finished or become full,
 *    to prevent new references to the zone write plug to be taken for
 *    newly incoming BIOs. A zone write plug flagged with this flag will be
 *    freed once all remaining references from BIOs or functions are dropped.
 *  - BLK_ZONE_WPLUG_NEEDS_FREE: Indicates that the zone write plug was
 *    dynamically allocated and needs to be freed instead of returned to the
 *    free list of zone write plugs of the disk.
 */
#define BLK_ZONE_WPLUG_PLUGGED		(1U << 0)
#define BLK_ZONE_WPLUG_ERROR		(1U << 1)
#define BLK_ZONE_WPLUG_UNHASHED		(1U << 2)
#define BLK_ZONE_WPLUG_NEEDS_FREE	(1U << 3)

#define BLK_ZONE_WPLUG_BUSY	(BLK_ZONE_WPLUG_PLUGGED | BLK_ZONE_WPLUG_ERROR)

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

static inline bool disk_zone_is_conv(struct gendisk *disk, sector_t sector)
{
	if (!disk->conv_zones_bitmap)
		return false;
	return test_bit(disk_zone_no(disk, sector), disk->conv_zones_bitmap);
}

static inline bool bio_zone_is_conv(struct bio *bio)
{
	return disk_zone_is_conv(bio->bi_bdev->bd_disk, bio->bi_iter.bi_sector);
}

static void blk_zone_wplug_bio_work(struct work_struct *work);

static void disk_init_zone_wplug(struct gendisk *disk,
				 struct blk_zone_wplug *zwplug,
				 unsigned int flags, sector_t sector)
{
	unsigned int zno = disk_zone_no(disk, sector);

	/*
	 * Initialize the zoen write plug with an extra reference so that
	 * it is not freed when the zone write plug becomes idle without
	 * the zone being full.
	 */
	INIT_HLIST_NODE(&zwplug->node);
	INIT_LIST_HEAD(&zwplug->link);
	atomic_set(&zwplug->ref, 2);
	spin_lock_init(&zwplug->lock);
	zwplug->flags = flags;
	zwplug->zone_no = zno;
	zwplug->wp_offset = sector & (disk->queue->limits.chunk_sectors - 1);
	bio_list_init(&zwplug->bio_list);
	INIT_WORK(&zwplug->bio_work, blk_zone_wplug_bio_work);
	zwplug->disk = disk;
}

static struct blk_zone_wplug *disk_alloc_zone_wplug(struct gendisk *disk,
						sector_t sector, gfp_t gfp_mask)
{
	struct blk_zone_wplug *zwplug = NULL;
	unsigned int zwp_flags = 0;
	unsigned long flags;

	spin_lock_irqsave(&disk->zone_wplugs_lock, flags);
	zwplug = list_first_entry_or_null(&disk->zone_wplugs_free_list,
					  struct blk_zone_wplug, link);
	if (zwplug)
		list_del_init(&zwplug->link);
	spin_unlock_irqrestore(&disk->zone_wplugs_lock, flags);

	if (!zwplug) {
		/* Allocate a new zone write plug. */
		zwplug = kmalloc(sizeof(struct blk_zone_wplug), gfp_mask);
		if (!zwplug)
			return NULL;
		zwp_flags = BLK_ZONE_WPLUG_NEEDS_FREE;
	}

	disk_init_zone_wplug(disk, zwplug, zwp_flags, sector);

	return zwplug;
}

static void disk_free_zone_wplug(struct blk_zone_wplug *zwplug)
{
	struct gendisk *disk = zwplug->disk;
	unsigned long flags;

	if (zwplug->flags & BLK_ZONE_WPLUG_NEEDS_FREE) {
		kfree(zwplug);
		return;
	}

	spin_lock_irqsave(&disk->zone_wplugs_lock, flags);
	list_add_tail(&zwplug->link, &disk->zone_wplugs_free_list);
	spin_unlock_irqrestore(&disk->zone_wplugs_lock, flags);
}

static bool disk_insert_zone_wplug(struct gendisk *disk,
				   struct blk_zone_wplug *zwplug)
{
	struct blk_zone_wplug *zwplg;
	unsigned long flags;
	unsigned int idx =
		hash_32(zwplug->zone_no, disk->zone_wplugs_hash_bits);

	/*
	 * Add the new zone write plug to the hash table, but carefully as we
	 * are racing with other submission context, so we may already have a
	 * zone write plug for the same zone.
	 */
	spin_lock_irqsave(&disk->zone_wplugs_lock, flags);
	hlist_for_each_entry_rcu(zwplg, &disk->zone_wplugs_hash[idx], node) {
		if (zwplg->zone_no == zwplug->zone_no) {
			spin_unlock_irqrestore(&disk->zone_wplugs_lock, flags);
			return false;
		}
	}
	hlist_add_head_rcu(&zwplug->node, &disk->zone_wplugs_hash[idx]);
	spin_unlock_irqrestore(&disk->zone_wplugs_lock, flags);

	return true;
}

static void disk_remove_zone_wplug(struct gendisk *disk,
				   struct blk_zone_wplug *zwplug)
{
	unsigned long flags;

	spin_lock_irqsave(&disk->zone_wplugs_lock, flags);
	zwplug->flags |= BLK_ZONE_WPLUG_UNHASHED;
	atomic_dec(&zwplug->ref);
	hlist_del_init_rcu(&zwplug->node);
	spin_unlock_irqrestore(&disk->zone_wplugs_lock, flags);
}

static inline bool disk_should_remove_zone_wplug(struct gendisk *disk,
						 struct blk_zone_wplug *zwplug)
{
	/* If the zone is still busy, the plug cannot be removed. */
	if (zwplug->flags & BLK_ZONE_WPLUG_BUSY)
		return false;

	/* We can remove zone write plugs for zones that are empty or full. */
	return !zwplug->wp_offset ||
		zwplug->wp_offset >= disk->zone_capacity;
}

static inline struct blk_zone_wplug *
disk_lookup_zone_wplug(struct gendisk *disk, sector_t sector)
{
	unsigned int zno = disk_zone_no(disk, sector);
	unsigned int idx = hash_32(zno, disk->zone_wplugs_hash_bits);
	struct blk_zone_wplug *zwplug;

	rcu_read_lock();
	hlist_for_each_entry_rcu(zwplug, &disk->zone_wplugs_hash[idx], node) {
		if (zwplug->zone_no == zno)
			goto unlock;
	}
	zwplug = NULL;

unlock:
	rcu_read_unlock();
	return zwplug;
}

static inline struct blk_zone_wplug *bio_lookup_zone_wplug(struct bio *bio)
{
	return disk_lookup_zone_wplug(bio->bi_bdev->bd_disk,
				      bio->bi_iter.bi_sector);
}

static inline void blk_get_zone_wplug(struct blk_zone_wplug *zwplug)
{
	atomic_inc(&zwplug->ref);
}

static struct blk_zone_wplug *disk_get_zone_wplug(struct gendisk *disk,
						  sector_t sector)
{
	struct blk_zone_wplug *zwplug;

	rcu_read_lock();
	zwplug = disk_lookup_zone_wplug(disk, sector);
	if (zwplug && !atomic_inc_not_zero(&zwplug->ref))
		zwplug = NULL;
	rcu_read_unlock();

	return zwplug;
}

static void disk_free_zone_wplug_rcu(struct rcu_head *rcu_head)
{
	struct blk_zone_wplug *zwplug =
		container_of(rcu_head, struct blk_zone_wplug, rcu_head);

	disk_free_zone_wplug(zwplug);
}

static inline void disk_put_zone_wplug(struct blk_zone_wplug *zwplug)
{
	if (atomic_dec_and_test(&zwplug->ref)) {
		WARN_ON_ONCE(!bio_list_empty(&zwplug->bio_list));
		WARN_ON_ONCE(!list_empty(&zwplug->link));

		call_rcu(&zwplug->rcu_head, disk_free_zone_wplug_rcu);
	}
}

/*
 * Get a reference on the write plug for the zone containing @sector.
 * If the plug does not exist, it is allocated and hashed.
 * Return a pointer to the zone write plug with the plug spinlock held.
 */
static struct blk_zone_wplug *disk_get_and_lock_zone_wplug(struct gendisk *disk,
					sector_t sector, gfp_t gfp_mask,
					unsigned long *flags)
{
	struct blk_zone_wplug *zwplug;

again:
	zwplug = disk_get_zone_wplug(disk, sector);
	if (zwplug) {
		/*
		 * Check that a BIO completion or a zone reset or finish
		 * operation has not already removed the zone write plug from
		 * the hash table and dropped its reference count. In such case,
		 * we need to get a new plug so start over from the beginning.
		 */
		spin_lock_irqsave(&zwplug->lock, *flags);
		if (zwplug->flags & BLK_ZONE_WPLUG_UNHASHED) {
			spin_unlock_irqrestore(&zwplug->lock, *flags);
			disk_put_zone_wplug(zwplug);
			goto again;
		}
		return zwplug;
	}

	zwplug = disk_alloc_zone_wplug(disk, sector, gfp_mask);
	if (!zwplug)
		return NULL;

	spin_lock_irqsave(&zwplug->lock, *flags);

	/*
	 * Insert the new zone write plug in the hash table. This can fail only
	 * if another context already inserted a plug. Retry from the beginning
	 * in such case.
	 */
	if (!disk_insert_zone_wplug(disk, zwplug)) {
		spin_unlock_irqrestore(&zwplug->lock, *flags);
		disk_free_zone_wplug(zwplug);
		goto again;
	}

	return zwplug;
}

static struct blk_zone_wplug *bio_get_and_lock_zone_wplug(struct bio *bio,
							  unsigned long *flags)
{
	gfp_t gfp_mask;

	if (bio->bi_opf & REQ_NOWAIT)
		gfp_mask = GFP_NOWAIT;
	else
		gfp_mask = GFP_NOIO;

	return disk_get_and_lock_zone_wplug(bio->bi_bdev->bd_disk,
				bio->bi_iter.bi_sector, gfp_mask, flags);
}

static inline void blk_zone_wplug_bio_io_error(struct bio *bio)
{
	struct request_queue *q = bio->bi_bdev->bd_disk->queue;

	bio_clear_flag(bio, BIO_ZONE_WRITE_PLUGGING);
	bio_io_error(bio);
	blk_queue_exit(q);
}

/*
 * Abort (fail) all plugged BIOs of a zone write plug.
 */
static void disk_zone_wplug_abort(struct blk_zone_wplug *zwplug)
{
	struct bio *bio;

	while ((bio = bio_list_pop(&zwplug->bio_list))) {
		blk_zone_wplug_bio_io_error(bio);
		disk_put_zone_wplug(zwplug);
	}
}

/*
 * Abort (fail) all plugged BIOs of a zone write plug that are not aligned
 * with the assumed write pointer location of the zone when the BIO will
 * be unplugged.
 */
static void disk_zone_wplug_abort_unaligned(struct gendisk *disk,
					    struct blk_zone_wplug *zwplug)
{
	unsigned int zone_capacity = disk->zone_capacity;
	unsigned int wp_offset = zwplug->wp_offset;
	struct bio_list bl = BIO_EMPTY_LIST;
	struct bio *bio;

	while ((bio = bio_list_pop(&zwplug->bio_list))) {
		if (wp_offset >= zone_capacity ||
		     bio_offset_from_zone_start(bio) != wp_offset) {
			blk_zone_wplug_bio_io_error(bio);
			disk_put_zone_wplug(zwplug);
			continue;
		}

		wp_offset += bio_sectors(bio);
		bio_list_add(&bl, bio);
	}

	bio_list_merge(&zwplug->bio_list, &bl);
}

/*
 * Set a zone write plug write pointer offset to either 0 (zone reset case)
 * or to the zone size (zone finish case). This aborts all plugged BIOs, which
 * is fine to do as doing a zone reset or zone finish while writes are in-flight
 * is a mistake from the user which will most likely cause all plugged BIOs to
 * fail anyway.
 */
static void disk_zone_wplug_set_wp_offset(struct gendisk *disk,
					  struct blk_zone_wplug *zwplug,
					  unsigned int wp_offset)
{
	unsigned long flags;

	spin_lock_irqsave(&zwplug->lock, flags);

	/*
	 * Make sure that a BIO completion or another zone reset or finish
	 * operation has not already removed the plug from the hash table.
	 */
	if (zwplug->flags & BLK_ZONE_WPLUG_UNHASHED) {
		spin_unlock_irqrestore(&zwplug->lock, flags);
		return;
	}

	/* Update the zone write pointer and abort all plugged BIOs. */
	zwplug->wp_offset = wp_offset;
	disk_zone_wplug_abort(zwplug);

	/*
	 * Updating the write pointer offset puts back the zone
	 * in a good state. So clear the error flag and decrement the
	 * error count if we were in error state.
	 */
	if (zwplug->flags & BLK_ZONE_WPLUG_ERROR) {
		zwplug->flags &= ~BLK_ZONE_WPLUG_ERROR;
		spin_lock(&disk->zone_wplugs_lock);
		list_del_init(&zwplug->link);
		spin_unlock(&disk->zone_wplugs_lock);
	}

	/*
	 * The zone write plug now has no BIO plugged: remove it from the
	 * hash table so that it cannot be seen. The plug will be freed
	 * when the last reference is dropped.
	 */
	if (disk_should_remove_zone_wplug(disk, zwplug))
		disk_remove_zone_wplug(disk, zwplug);

	spin_unlock_irqrestore(&zwplug->lock, flags);
}

static bool blk_zone_wplug_handle_reset_or_finish(struct bio *bio,
						  unsigned int wp_offset)
{
	struct gendisk *disk = bio->bi_bdev->bd_disk;
	struct blk_zone_wplug *zwplug;

	/* Conventional zones cannot be reset nor finished. */
	if (bio_zone_is_conv(bio)) {
		bio_io_error(bio);
		return true;
	}

	/*
	 * If we have a zone write plug, set its write pointer offset to 0
	 * (reset case) or to the zone size (finish case). This will abort all
	 * BIOs plugged for the target zone. It is fine as resetting or
	 * finishing zones while writes are still in-flight will result in the
	 * writes failing anyway.
	 */
	zwplug = disk_get_zone_wplug(disk, bio->bi_iter.bi_sector);
	if (zwplug) {
		disk_zone_wplug_set_wp_offset(disk, zwplug, wp_offset);
		disk_put_zone_wplug(zwplug);
	}

	return false;
}

static bool blk_zone_wplug_handle_reset_all(struct bio *bio)
{
	struct gendisk *disk = bio->bi_bdev->bd_disk;
	struct blk_zone_wplug *zwplug;
	sector_t sector;

	/*
	 * Set the write pointer offset of all zone write plugs to 0. This will
	 * abort all plugged BIOs. It is fine as resetting zones while writes
	 * are still in-flight will result in the writes failing anyway.
	 */
	for (sector = 0; sector < get_capacity(disk);
	     sector += disk->queue->limits.chunk_sectors) {
		zwplug = disk_get_zone_wplug(disk, sector);
		if (zwplug) {
			disk_zone_wplug_set_wp_offset(disk, zwplug, 0);
			disk_put_zone_wplug(zwplug);
		}
	}

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
	struct blk_zone_wplug *zwplug;
	unsigned long flags;

	/*
	 * If the BIO was already plugged, then we were called through
	 * blk_zone_write_plug_attempt_merge() -> blk_attempt_bio_merge().
	 * For this case, blk_zone_write_plug_attempt_merge() will handle the
	 * zone write pointer offset update.
	 */
	if (bio_flagged(bio, BIO_ZONE_WRITE_PLUGGING))
		return;

	bio_set_flag(bio, BIO_ZONE_WRITE_PLUGGING);

	/*
	 * Increase the plug reference count and advance the zone write
	 * pointer offset.
	 */
	zwplug = bio_lookup_zone_wplug(bio);
	spin_lock_irqsave(&zwplug->lock, flags);
	blk_get_zone_wplug(zwplug);
	zwplug->wp_offset += bio_sectors(bio);
	spin_unlock_irqrestore(&zwplug->lock, flags);
}

/*
 * Attempt to merge plugged BIOs with a newly prepared request for a BIO that
 * already went through zone write plugging (either a new BIO or one that was
 * unplugged).
 */
void blk_zone_write_plug_attempt_merge(struct request *req)
{
	sector_t req_back_sector = blk_rq_pos(req) + blk_rq_sectors(req);
	struct request_queue *q = req->q;
	struct gendisk *disk = q->disk;
	unsigned int zone_capacity = disk->zone_capacity;
	struct blk_zone_wplug *zwplug =
		disk_lookup_zone_wplug(disk, blk_rq_pos(req));
	unsigned long flags;
	struct bio *bio;

	/*
	 * Completion of this request needs to be handled with
	 * blk_zone_write_complete_request().
	 */
	req->rq_flags |= RQF_ZONE_WRITE_PLUGGING;
	blk_get_zone_wplug(zwplug);

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

static inline void disk_zone_wplug_set_error(struct gendisk *disk,
					     struct blk_zone_wplug *zwplug)
{
	if (!(zwplug->flags & BLK_ZONE_WPLUG_ERROR)) {
		unsigned long flags;

		zwplug->flags |= BLK_ZONE_WPLUG_ERROR;

		spin_lock_irqsave(&disk->zone_wplugs_lock, flags);
		list_add_tail(&zwplug->link, &disk->zone_wplugs_err_list);
		spin_unlock_irqrestore(&disk->zone_wplugs_lock, flags);
	}
}

/*
 * Check and prepare a BIO for submission by incrementing the write pointer
 * offset of its zone write plug.
 */
static bool blk_zone_wplug_prepare_bio(struct blk_zone_wplug *zwplug,
				       struct bio *bio)
{
	struct gendisk *disk = bio->bi_bdev->bd_disk;

	/*
	 * Check that the user is not attempting to write to a full zone.
	 * We know such BIO will fail, and that would potentially overflow our
	 * write pointer offset beyond the end of the zone.
	 */
	if (zwplug->wp_offset >= disk->zone_capacity)
		goto err;

	/*
	 * Check for non-sequential writes early because we avoid a
	 * whole lot of error handling trouble if we don't send it off
	 * to the driver.
	 */
	if (bio_offset_from_zone_start(bio) != zwplug->wp_offset)
		goto err;

	/* Advance the zone write pointer offset. */
	zwplug->wp_offset += bio_sectors(bio);

	return true;

err:
	/* We detected an invalid write BIO: schedule error recovery. */
	disk_zone_wplug_set_error(disk, zwplug);
	kblockd_schedule_work(&disk->zone_wplugs_work);
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
	if (WARN_ON_ONCE(bio_straddles_zones(bio))) {
		bio_io_error(bio);
		return true;
	}

	/* Conventional zones do not need write plugging. */
	if (bio_zone_is_conv(bio))
		return false;

	zwplug = bio_get_and_lock_zone_wplug(bio, &flags);
	if (!zwplug) {
		bio_io_error(bio);
		return true;
	}

	/* Indicate that this BIO is being handled using zone write plugging. */
	bio_set_flag(bio, BIO_ZONE_WRITE_PLUGGING);

	/*
	 * If the zone is already plugged or has a pending error, add the BIO
	 * to the plug BIO list. Otherwise, plug and let the BIO execute.
	 */
	if (zwplug->flags & BLK_ZONE_WPLUG_BUSY)
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
 * Handle write and write zeroes operations using zone write plugging.
 * Return true whenever @bio execution needs to be delayed through the zone
 * write plug. Otherwise, return false to let the submission path process
 * @bio normally.
 */
bool blk_zone_write_plug_bio(struct bio *bio, unsigned int nr_segs)
{
	struct block_device *bdev = bio->bi_bdev;

	if (!bdev->bd_disk->zone_wplugs_hash)
		return false;

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
	 * Zone reset, reset all and finish commands need special treatment
	 * to correctly track the write pointer offset of zones. These commands
	 * are not plugged as we do not need serialization with write
	 * operations. It is the responsibility of the user to not issue reset
	 * and finish commands when write operations are in flight.
	 */
	switch (bio_op(bio)) {
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

static void disk_zone_wplug_unplug_bio(struct gendisk *disk,
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
		kblockd_schedule_work(&disk->zone_wplugs_work);
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
	 * (it was reset), remove its zone write plug from the hash table.
	 */
	if (disk_should_remove_zone_wplug(disk, zwplug))
		disk_remove_zone_wplug(disk, zwplug);

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
	 * If the BIO failed, mark the plug as having an error to trigger
	 * recovery.
	 */
	if (bio->bi_status != BLK_STS_OK) {
		spin_lock_irqsave(&zwplug->lock, flags);
		disk_zone_wplug_set_error(disk, zwplug);
		spin_unlock_irqrestore(&zwplug->lock, flags);
	}

	/*
	 * For BIO-based devices, blk_zone_write_plug_complete_request()
	 * is not called. So we need to schedule execution of the next
	 * plugged BIO here.
	 */
	if (bio->bi_bdev->bd_has_submit_bio)
		disk_zone_wplug_unplug_bio(disk, zwplug);

	disk_put_zone_wplug(zwplug);
}

void blk_zone_write_plug_complete_request(struct request *req)
{
	struct gendisk *disk = req->q->disk;
	struct blk_zone_wplug *zwplug =
		disk_lookup_zone_wplug(disk, req->__sector);

	req->rq_flags &= ~RQF_ZONE_WRITE_PLUGGING;

	disk_zone_wplug_unplug_bio(disk, zwplug);

	disk_put_zone_wplug(zwplug);
}

static void blk_zone_wplug_bio_work(struct work_struct *work)
{
	struct blk_zone_wplug *zwplug =
		container_of(work, struct blk_zone_wplug, bio_work);
	struct block_device *bdev;
	unsigned long flags;
	struct bio *bio;

	/*
	 * Submit the next plugged BIO. If we do not have any, clear
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

static int blk_zone_wplug_report_zone_cb(struct blk_zone *zone,
					 unsigned int idx, void *data)
{
	struct blk_zone *zonep = data;

	*zonep = *zone;
	return 0;
}

static void disk_zone_wplug_handle_error(struct gendisk *disk,
					 struct blk_zone_wplug *zwplug)
{
	sector_t zone_start_sector =
		bdev_zone_sectors(disk->part0) * zwplug->zone_no;
	unsigned int noio_flag;
	struct blk_zone zone;
	unsigned long flags;
	int ret;

	/* Get the current zone information from the device. */
	noio_flag = memalloc_noio_save();
	ret = disk->fops->report_zones(disk, zone_start_sector, 1,
				       blk_zone_wplug_report_zone_cb, &zone);
	memalloc_noio_restore(noio_flag);

	spin_lock_irqsave(&zwplug->lock, flags);

	/*
	 * A zone reset or finish may have cleared the error already. In such
	 * case, do nothing as the report zones may have seen the "old" write
	 * pointer value before the reset/finish operation completed.
	 */
	if (!(zwplug->flags & BLK_ZONE_WPLUG_ERROR))
		goto unlock;

	zwplug->flags &= ~BLK_ZONE_WPLUG_ERROR;

	if (ret != 1) {
		/*
		 * We failed to get the zone information, meaning that something
		 * is likely really wrong with the device. Abort all remaining
		 * plugged BIOs as otherwise we could endup waiting forever on
		 * plugged BIOs to complete if there is a queue freeze on-going.
		 */
		disk_zone_wplug_abort(zwplug);
		goto unplug;
	}

	/* Update the zone write pointer offset. */
	zwplug->wp_offset = blk_zone_wp_offset(&zone);
	disk_zone_wplug_abort_unaligned(disk, zwplug);

	/* Restart BIO submission if we still have any BIO left. */
	if (!bio_list_empty(&zwplug->bio_list)) {
		WARN_ON_ONCE(!(zwplug->flags & BLK_ZONE_WPLUG_PLUGGED));
		kblockd_schedule_work(&zwplug->bio_work);
		goto unlock;
	}

unplug:
	zwplug->flags &= ~BLK_ZONE_WPLUG_PLUGGED;
	if (disk_should_remove_zone_wplug(disk, zwplug))
		disk_remove_zone_wplug(disk, zwplug);

unlock:
	spin_unlock_irqrestore(&zwplug->lock, flags);
}

static void disk_zone_wplugs_work(struct work_struct *work)
{
	struct gendisk *disk =
		container_of(work, struct gendisk, zone_wplugs_work);
	struct blk_zone_wplug *zwplug;
	unsigned long flags;

	spin_lock_irqsave(&disk->zone_wplugs_lock, flags);

	while (!list_empty(&disk->zone_wplugs_err_list)) {
		zwplug = list_first_entry(&disk->zone_wplugs_err_list,
					  struct blk_zone_wplug, link);
		list_del_init(&zwplug->link);
		blk_get_zone_wplug(zwplug);
		spin_unlock_irqrestore(&disk->zone_wplugs_lock, flags);

		disk_zone_wplug_handle_error(disk, zwplug);
		disk_put_zone_wplug(zwplug);

		spin_lock_irqsave(&disk->zone_wplugs_lock, flags);
	}

	spin_unlock_irqrestore(&disk->zone_wplugs_lock, flags);
}

static inline unsigned int disk_zone_wplugs_hash_size(struct gendisk *disk)
{
	return 1U << disk->zone_wplugs_hash_bits;
}

static int disk_alloc_zone_wplugs(struct gendisk *disk,
				  unsigned int max_nr_zwplugs)
{
	struct blk_zone_wplug *zwplug;
	unsigned int i;

	if (!disk->zone_wplugs_hash)
		return 0;

	/* Pre-allocate zone write plugs */
	for (i = 0; i < max_nr_zwplugs; i++) {
		zwplug = kmalloc(sizeof(struct blk_zone_wplug), GFP_KERNEL);
		if (!zwplug)
			return -ENOMEM;
		disk_init_zone_wplug(disk, zwplug, 0, 0);

		list_add_tail(&zwplug->link, &disk->zone_wplugs_free_list);
	}

	disk->zone_wplugs_max_nr += max_nr_zwplugs;

	return 0;
}

static void disk_free_zone_wplugs(struct gendisk *disk)
{
	struct blk_zone_wplug *zwplug;
	unsigned int i;

	if (!disk->zone_wplugs_hash)
		return;

	/* Free all the zone write plugs we have. */
	for (i = 0; i < disk_zone_wplugs_hash_size(disk); i++) {
		while (!hlist_empty(&disk->zone_wplugs_hash[i])) {
			zwplug = hlist_entry(disk->zone_wplugs_hash[i].first,
					     struct blk_zone_wplug, node);
			blk_get_zone_wplug(zwplug);
			disk_remove_zone_wplug(disk, zwplug);
			disk_put_zone_wplug(zwplug);
		}
	}

	/* Wait for the zone write plugs to be RCU-freed. */
	rcu_barrier();

	while (!list_empty(&disk->zone_wplugs_free_list)) {
		zwplug = list_first_entry(&disk->zone_wplugs_free_list,
					  struct blk_zone_wplug, link);
		list_del_init(&zwplug->link);

		kfree(zwplug);
	}

	disk->zone_wplugs_max_nr = 0;
}

void disk_init_zone_resources(struct gendisk *disk)
{
	spin_lock_init(&disk->zone_wplugs_lock);
	INIT_LIST_HEAD(&disk->zone_wplugs_free_list);
	INIT_LIST_HEAD(&disk->zone_wplugs_err_list);
	INIT_WORK(&disk->zone_wplugs_work, disk_zone_wplugs_work);
}

/*
 * For the size of a disk zone write plug hash table, use the disk maximum
 * open zones and maximum active zones limits, but do not exceed 4KB (512 hlist
 * head entries), that is, 9 bits. For a disk that has no limits, default to
 * 128 zones for the number of zone write plugs to hash.
 */
#define BLK_ZONE_MAX_WPLUG_HASH_BITS		9
#define BLK_ZONE_DEFAULT_MAX_NR_WPLUGS		128

static int disk_alloc_zone_resources(struct gendisk *disk,
				     unsigned int max_nr_zwplugs)
{
	unsigned int i;
	int ret;

	disk->zone_wplugs_hash_bits =
		min(ilog2(max_nr_zwplugs) + 1, BLK_ZONE_MAX_WPLUG_HASH_BITS);

	disk->zone_wplugs_hash =
		kcalloc(disk_zone_wplugs_hash_size(disk),
			sizeof(struct hlist_head), GFP_KERNEL);
	if (!disk->zone_wplugs_hash)
		return -ENOMEM;

	for (i = 0; i < disk_zone_wplugs_hash_size(disk); i++)
		INIT_HLIST_HEAD(&disk->zone_wplugs_hash[i]);

	ret = disk_alloc_zone_wplugs(disk, max_nr_zwplugs);
	if (ret) {
		disk_free_zone_wplugs(disk);
		kfree(disk->zone_wplugs_hash);
		disk->zone_wplugs_hash = NULL;
		disk->zone_wplugs_hash_bits = 0;
		return ret;
	}

	return 0;
}

void disk_free_zone_resources(struct gendisk *disk)
{
	cancel_work_sync(&disk->zone_wplugs_work);

	disk_free_zone_wplugs(disk);

	kfree(disk->zone_wplugs_hash);
	disk->zone_wplugs_hash = NULL;
	disk->zone_wplugs_hash_bits = 0;

	kfree(disk->conv_zones_bitmap);
	disk->conv_zones_bitmap = NULL;
	kfree(disk->seq_zones_wlock);
	disk->seq_zones_wlock = NULL;

	disk->zone_capacity = 0;
	disk->nr_zones = 0;
}

static int disk_revalidate_zone_resources(struct gendisk *disk,
					  unsigned int nr_zones)
{
	struct queue_limits *lim = &disk->queue->limits;
	unsigned int max_nr_zwplugs;
	bool set_max_open = false;
	int ret;

	/*
	 * If the device has no limit on the maximum number of open and active
	 * zones, use BLK_ZONE_DEFAULT_MAX_NR_WPLUGS for the maximum number
	 * of zone write plugs to hash and set the max_open_zones queue limit
	 * of the device to indicate to the user the number of pre-allocated
	 * zone write plugsso that the user is aware of the potential
	 * performance penalty for simultaneously writing to more zones than
	 * this limit.
	 */
	max_nr_zwplugs = max(lim->max_open_zones, lim->max_active_zones);
	if (!max_nr_zwplugs) {
		max_nr_zwplugs =
			min(BLK_ZONE_DEFAULT_MAX_NR_WPLUGS, nr_zones);
		set_max_open = true;
	}

	if (!disk->zone_wplugs_hash) {
		ret = disk_alloc_zone_resources(disk, max_nr_zwplugs);
		if (ret)
			return ret;
	}

	/* Grow the free list of zone write plugs if needed. */
	if (disk->zone_wplugs_max_nr < max_nr_zwplugs) {
		ret = disk_alloc_zone_wplugs(disk,
				max_nr_zwplugs - disk->zone_wplugs_max_nr);
		if (ret)
			return ret;
	}

	if (set_max_open)
		disk_set_max_open_zones(disk, max_nr_zwplugs);

	return 0;
}

struct blk_revalidate_zone_args {
	struct gendisk	*disk;
	unsigned long	*conv_zones_bitmap;
	unsigned long	*seq_zones_wlock;
	unsigned int	nr_zones;
	unsigned int	zone_capacity;
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
	unsigned int wp_offset;

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

	if (!zone->capacity || zone->capacity > zone->len) {
		pr_warn("%s: Invalid zone capacity\n",
			disk->disk_name);
		return -ENODEV;
	}

	/* Check zone type */
	switch (zone->type) {
	case BLK_ZONE_TYPE_CONVENTIONAL:
		if (zone->capacity != zone->len) {
			pr_warn("%s: Invalid conventional zone capacity\n",
				disk->disk_name);
			return -ENODEV;
		}
		if (!args->conv_zones_bitmap) {
			args->conv_zones_bitmap =
				blk_alloc_zone_bitmap(q->node, args->nr_zones);
			if (!args->conv_zones_bitmap)
				return -ENOMEM;
		}
		set_bit(idx, args->conv_zones_bitmap);
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

		/*
		 * We need to track the write pointer of all zones that are not
		 * empty nor full. So make sure we have a zone write plug for
		 * such zone.
		 */
		wp_offset = blk_zone_wp_offset(zone);
		if (wp_offset && wp_offset < zone_sectors) {
			zwplug = disk_get_and_lock_zone_wplug(disk, zone->start,
							      GFP_NOIO, &flags);
			if (!zwplug)
				return -ENOMEM;
			spin_unlock_irqrestore(&zwplug->lock, flags);
			disk_put_zone_wplug(zwplug);
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
	args.disk = disk;
	args.nr_zones = (capacity + zone_sectors - 1) >> ilog2(zone_sectors);
	noio_flag = memalloc_noio_save();
	ret = disk_revalidate_zone_resources(disk, args.nr_zones);
	if (ret) {
		memalloc_noio_restore(noio_flag);
		return ret;
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
		disk->zone_capacity = args.zone_capacity;
		swap(disk->seq_zones_wlock, args.seq_zones_wlock);
		swap(disk->conv_zones_bitmap, args.conv_zones_bitmap);
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
