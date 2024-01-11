// SPDX-License-Identifier: GPL-2.0
/*
 * Zoned block device handling
 *
 * Copyright (c) 2015, Hannes Reinecke
 * Copyright (c) 2015, SUSE Linux GmbH
 *
 * Copyright (c) 2016, Damien Le Moal
 * Copyright (c) 2016, Western Digital
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rbtree.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/sched/mm.h>

#include "blk.h"

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
 */
struct blk_zone_wplug {
	unsigned long		flags;
	struct bio_list		bio_list;
	struct work_struct	bio_work;
	unsigned int		wp_offset;
	unsigned int		capacity;
};

/*
 * Zone write plug flags bits:
 *  - BLK_ZONE_WPLUG_CONV: Indicate that the zone is a conventional one. Writes
 *    to these zones are never plugged.
 *  - BLK_ZONE_WPLUG_LOCKED: Indicate that the zone is locked.
 *  - BLK_ZONE_WPLUG_PLUGGED: Indicate that the zone write plug is plugged,
 *    that is, that write BIOs are being throttled due to a write BIO already
 *    being executed or the zone write plug bio list is not empty.
 *  - BLK_ZONE_WPLUG_ERROR: Indicate that a write error happened which will be
 *    recovered with a report zone to update the zone write pointer offset.
 */
enum {
	BLK_ZONE_WPLUG_CONV = 0,
	BLK_ZONE_WPLUG_LOCKED,
	BLK_ZONE_WPLUG_PLUGGED,
	BLK_ZONE_WPLUG_ERROR,
};

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

static int blkdev_zone_reset_all_emulated(struct block_device *bdev,
					  gfp_t gfp_mask)
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
				   gfp_mask);
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

static int blkdev_zone_reset_all(struct block_device *bdev, gfp_t gfp_mask)
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
 * @gfp_mask:	Memory allocation flags (for bio_alloc)
 *
 * Description:
 *    Perform the specified operation on the range of zones specified by
 *    @sector..@sector+@nr_sectors. Specifying the entire disk sector range
 *    is valid, but the specified range should not contain conventional zones.
 *    The operation to execute on each zone can be a zone reset, open, close
 *    or finish request.
 */
int blkdev_zone_mgmt(struct block_device *bdev, enum req_op op,
		     sector_t sector, sector_t nr_sectors, gfp_t gfp_mask)
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
			return blkdev_zone_reset_all_emulated(bdev, gfp_mask);
		return blkdev_zone_reset_all(bdev, gfp_mask);
	}

	while (sector < end_sector) {
		bio = blk_next_bio(bio, bdev, 0, op | REQ_SYNC, gfp_mask);
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

	ret = blkdev_zone_mgmt(bdev, op, zrange.sector, zrange.nr_sectors,
			       GFP_KERNEL);

fail:
	if (cmd == BLKRESETZONE)
		filemap_invalidate_unlock(bdev->bd_inode->i_mapping);

	return ret;
}

static inline void blk_zone_wplug_lock(struct blk_zone_wplug *zwplug)
{
	wait_on_bit_lock_io(&zwplug->flags, BLK_ZONE_WPLUG_LOCKED,
			    TASK_UNINTERRUPTIBLE);
}

static inline void blk_zone_wplug_unlock(struct blk_zone_wplug *zwplug)
{
	clear_and_wake_up_bit(BLK_ZONE_WPLUG_LOCKED, &zwplug->flags);
}

static inline void blk_zone_bio_io_error(struct bio *bio)
{
	bio_clear_flag(bio, BIO_ZONE_WRITE_PLUGGING);
	bio_io_error(bio);

}

static void blk_zone_abort_wplug(struct blk_zone_wplug *zwplug)
{
	struct gendisk *disk = NULL;
	unsigned int zno = 0;
	struct bio *bio;
	int n = 0;

	while ((bio = bio_list_pop(&zwplug->bio_list))) {
		if (!disk) {
			disk = bio->bi_bdev->bd_disk;
			zno = bio_zone_no(bio);
		}
		blk_zone_bio_io_error(bio);
		n++;
	}

	if (n && disk)
		pr_warn("%s: zone %u, %u plugged BIOs aborted\n",
			disk->disk_name, zno, n);
}

/*
 * Return the zone write plug for a BIO targetting a sequential write
 * required zone. Given that conventional zones have no write ordering
 * constraints, NULL is returned for BIOs targetting conventional zones
 * to indicate that plugging the BIO is not needed.
 */
static inline struct blk_zone_wplug *blk_zone_lookup_wplug(struct bio *bio)
{
	struct blk_zone_wplug *zwplug =
		&bio->bi_bdev->bd_disk->zone_wplugs[bio_zone_no(bio)];

	if (test_bit(BLK_ZONE_WPLUG_CONV, &zwplug->flags))
		return NULL;

	return zwplug;
}

static void blk_zone_wplug_add_bio(struct blk_zone_wplug *zwplug,
				   struct bio *bio)
{
	sector_t sector = bio->bi_iter.bi_sector;
	struct bio_list *bl = &zwplug->bio_list;
	struct bio *prev, *pos = bl->tail;

	/*
	 * The BIO is being plugged and thus will have to wait for the on-going
	 * write and for all other writes already plugged. So polling makes
	 * no sense.
	 */
	bio_clear_polled(bio);

	/*
	 * While the regular submission path will split and issue a BIO from
	 * the same context, using current->bio_list to avoid recursion and
	 * preserve the BIO chain submission order, zone write plugging will
	 * use this path only for the first part of a split large BIO.
	 * Eventual further splitting of the BIO will happen from the context
	 * of bio_work for the zone write plug.  This can cause overall
	 * reordering of write BIOs in the BIO list of the zone write plug if
	 * there are other user submitters of BIOs. To avoid this problem,
	 * the BIO is inserted in increasing sector position in the BIO list of
	 * the zone write plug.
	 * Given that zone append operations have no ordering constraints
	 * and are never split, we do a simple add-at-tail for these.
	 */
	if (bio_op(bio) == REQ_OP_ZONE_APPEND ||
	    !pos || sector > pos->bi_iter.bi_sector) {
		bio_list_add(bl, bio);
		return;
	}

	prev = bl->head;
	bio_list_for_each(pos, bl) {
		if (sector < pos->bi_iter.bi_sector) {
			if (pos == bl->head)
				bio_list_add_head(bl, bio);
			else
				bio_list_add_after(bl, bio, prev);
			return;
		}
		prev = pos;
	}

	bio_list_add(bl, bio);
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
	case BLK_ZONE_COND_NOT_WP:
	case BLK_ZONE_COND_EMPTY:
	case BLK_ZONE_COND_OFFLINE:
	case BLK_ZONE_COND_READONLY:
	default:
		/*
		 * Conventional, offline and read-only zones do not have a valid
		 * write pointer.
		 */
		return 0;
	}
}

static void blk_zone_handle_reset(struct bio *bio)
{
	struct blk_zone_wplug *zwplug = blk_zone_lookup_wplug(bio);

	/* Resetting conventional zones is not allowed */
	if (!zwplug) {
		bio_io_error(bio);
		return;
	}

	blk_zone_wplug_lock(zwplug);
	zwplug->wp_offset = 0;
	blk_zone_wplug_unlock(zwplug);
}

static void blk_zone_handle_reset_all(struct bio *bio)
{
	struct gendisk *disk = bio->bi_bdev->bd_disk;
	struct blk_zone_wplug *zwplug = &disk->zone_wplugs[0];
	unsigned int i;

	for (i = 0; i < disk->nr_zones; i++, zwplug++) {
		/* Ignore conventional zones */
		if (test_bit(BLK_ZONE_WPLUG_CONV, &zwplug->flags))
			continue;
		blk_zone_wplug_lock(zwplug);
		zwplug->wp_offset = 0;
		blk_zone_wplug_unlock(zwplug);
	}
}

static void blk_zone_handle_finish(struct bio *bio)
{
	struct blk_zone_wplug *zwplug = blk_zone_lookup_wplug(bio);

	/* Finishing conventional zones is not allowed */
	if (!zwplug) {
		bio_io_error(bio);
		return;
	}

	blk_zone_wplug_lock(zwplug);
	zwplug->wp_offset = bdev_zone_sectors(bio->bi_bdev);
	blk_zone_wplug_unlock(zwplug);
}

static int blk_zone_handle_write_error_cb(struct blk_zone *zone,
					  unsigned int idx, void *data)
{
	struct gendisk *disk = data;
	unsigned int zno = disk_zone_no(disk, zone->start);
	struct blk_zone_wplug *zwplug = &disk->zone_wplugs[zno];

	zwplug->capacity = zone->capacity;
	zwplug->wp_offset = blk_zone_wp_offset(zone);

	return 0;
}

static bool blk_zone_handle_write_error(struct blk_zone_wplug *zwplug,
					struct bio *bio)
{
	struct block_device *bdev = bio->bi_bdev;
	struct gendisk *disk = bdev->bd_disk;
	unsigned int zno = bio_zone_no(bio);
	unsigned int noio_flag;
	int ret;

	/*
	 * We had a write error, meaning that the sequential write pattern
	 * is broken and that the plugged BIOs will fail. So abort them now.
	 */
	blk_zone_abort_wplug(zwplug);

	/*
	 * Issue a zone report to update the zone write pointer offset. If
	 * this fail, we keep BLK_ZONE_WPLUG_ERROR set so that this is retried
	 * again when a newly submitted BIO targets the same zone.
	 */
	noio_flag = memalloc_noio_save();
	ret = disk->fops->report_zones(disk, bdev_zone_sectors(bdev) << zno, 1,
				       blk_zone_handle_write_error_cb, disk);
	memalloc_noio_restore(noio_flag);
	if (ret != 1)
		return false;

	clear_bit(BLK_ZONE_WPLUG_ERROR, &zwplug->flags);

	return true;
}

static struct bio *blk_zone_wplug_unplug_bio(struct blk_zone_wplug *zwplug,
					     struct bio *bio)
{
	/*
	 * If we do not need to emulate zone append, zone write pointer offset
	 * tracking is not necessary and we have nothing to do.
	 */
	if (!bdev_emulates_zone_append(bio->bi_bdev))
		return bio;

	/* Handle write errors first. */
	if (test_bit(BLK_ZONE_WPLUG_ERROR, &zwplug->flags)) {
		if (!blk_zone_handle_write_error(zwplug, bio)) {
			blk_zone_bio_io_error(bio);
			return NULL;
		}
	}

	/*
	 * Check that the user is not attempting to write to a full zone.
	 * We know such BIO will fail, and that would potentially overflow our
	 * write pointer offset, causing zone append BIOs for one zone to be
	 * directed at the following zone.
         */
	if (zwplug->wp_offset >= zwplug->capacity) {
		blk_zone_bio_io_error(bio);
		return NULL;
	}

	if (bio_op(bio) == REQ_OP_ZONE_APPEND) {
		/*
		 * Use a regular write starting at the current write pointer.
		 * The zone write pointer offset is updated on completion. Note
		 * that we do not allow this write to be merged, similarly to
		 * native zone append operations.
		 */
		bio->bi_opf &= ~REQ_OP_MASK;
		bio->bi_opf |= REQ_OP_WRITE | REQ_NOMERGE;
		bio->bi_iter.bi_sector += zwplug->wp_offset;

		/*
		 * Remember that this BIO is in fact a zone append operation
		 * so that we can restore its operation code on completion.
		 */
		bio_set_flag(bio, BIO_EMULATES_ZONE_APPEND);
	}

	/* Advance the zone write pointer offset. */
	zwplug->wp_offset += bio_sectors(bio);

	return bio;
}

/**
 * blk_zone_write_plug_bio - Handle a zone write BIO with zone write plugging
 * @bio: The BIO being submitted
 *
 * Handle write, write zeroes and zone append operations requiring emulation
 * using zone write plugging. Return true whenever @bio execution needs to be
 * delayed through the zone write plug. Otherwise, return false to let the
 * submission path process @bio normally.
 */
bool blk_zone_write_plug_bio(struct bio *bio)
{
	struct block_device *bdev = bio->bi_bdev;
	struct gendisk *disk = bdev->bd_disk;
	struct blk_zone_wplug *zwplug;
	unsigned int nr_segs;

	if (!disk->zone_wplugs)
		return false;

	/*
	 * If the BIO already has the plugging flag set, then it was already
	 * handled through this path and this is a submission from the zone
	 * plug bio submit work. Do nothing for this case and let the BIO
	 * execute.
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
	 * Zone append operations that need emulation must also be plugged so
	 * that these operations can be changed into regular writes.
	 * Zone reset, reset all and finish commands need special treatment
	 * to correctly track the write pointer offset of zones when zone
	 * append emulation is needed. These commands are not plugged as we do
	 * not need serialization with write and append operations. It is the
	 * responsibility of the user to not issue reset and finish commands
	 * when write operations are in flight.
	 */
	switch (bio_op(bio)) {
	case REQ_OP_WRITE:
	case REQ_OP_WRITE_ZEROES:
		break;
	case REQ_OP_ZONE_APPEND:
		if (bdev_emulates_zone_append(bdev))
			break;
		return false;
	case REQ_OP_ZONE_RESET:
		if (bdev_emulates_zone_append(bdev))
			blk_zone_handle_reset(bio);
		return false;
	case REQ_OP_ZONE_RESET_ALL:
		if (bdev_emulates_zone_append(bdev))
			blk_zone_handle_reset_all(bio);
		return false;
	case REQ_OP_ZONE_FINISH:
		if (bdev_emulates_zone_append(bdev))
			blk_zone_handle_finish(bio);
		return false;
	default:
		return false;
	}

	/*
	 * For BIO-based block devices, the bio is not split by the block layer.
	 * So it may be large and straddle the zone boundary. If it is the
	 * case, split the BIO here so that we use the correct zone write plugs.
	 */
	if (bdev->bd_has_submit_bio &&
	    bio_zone_no(bio) != disk_zone_no(disk, bio_end_sector(bio) - 1)) {
		bio = __bio_split_to_limits(bio, &disk->queue->limits, &nr_segs);
		if (!bio)
			return true;
	}

	zwplug = blk_zone_lookup_wplug(bio);
	if (!zwplug)
		return false;

	blk_zone_wplug_lock(zwplug);

	/* Indicate that this BIO is being handled using zone write plugging */
	bio_set_flag(bio, BIO_ZONE_WRITE_PLUGGING);

	/*
	 * Add the bio to the plug bio list and if the plug is not already,
	 * plugged, let the BIO execute.
	 */
	if (test_and_set_bit(BLK_ZONE_WPLUG_PLUGGED, &zwplug->flags)) {
		blk_zone_wplug_add_bio(zwplug, bio);
		bio = NULL;
	} else {
		bio = blk_zone_wplug_unplug_bio(zwplug, bio);
		if (!bio)
			clear_bit(BLK_ZONE_WPLUG_PLUGGED, &zwplug->flags);
	}

	blk_zone_wplug_unlock(zwplug);

	return bio == NULL;
}
EXPORT_SYMBOL_GPL(blk_zone_write_plug_bio);

static void blk_zone_wplug_bio_work(struct work_struct *work)
{
	struct blk_zone_wplug *zwplug =
		container_of(work, struct blk_zone_wplug, bio_work);
	struct bio *bio;

	blk_zone_wplug_lock(zwplug);

	/*
	 * Unplug and submit the next plugged BIO. If we do not have any, clear
	 * the plugged flag.
	 */
	while ((bio = bio_list_pop(&zwplug->bio_list))) {
		bio = blk_zone_wplug_unplug_bio(zwplug, bio);
		if (bio)
			break;
	}

	if (!bio)
		clear_bit(BLK_ZONE_WPLUG_PLUGGED, &zwplug->flags);

	blk_zone_wplug_unlock(zwplug);

	if (bio)
		submit_bio_noacct_nocheck(bio);
}

void blk_zone_write_bio_endio(struct bio *bio)
{
	struct blk_zone_wplug *zwplug = blk_zone_lookup_wplug(bio);

	/* Make sure we do not see this BIO again by clearing the plug flag. */
	bio_clear_flag(bio, BIO_ZONE_WRITE_PLUGGING);

	if (WARN_ON_ONCE(!zwplug))
		return;

	if (bdev_emulates_zone_append(bio->bi_bdev)) {
		/*
		 * If this is a regular write emulating a zone append operation,
		 * restore the original operation code.
		 */
		if (bio_flagged(bio, BIO_EMULATES_ZONE_APPEND)) {
			bio->bi_opf &= ~REQ_OP_MASK;
			bio->bi_opf |= REQ_OP_ZONE_APPEND;
		}

		/*
		 * If bio failed, mark the zone write plug as having an error
		 * so that the endio work recovers the write pointer using
		 * report zones.
		 */
		if (bio->bi_status != BLK_STS_OK)
			set_bit(BLK_ZONE_WPLUG_ERROR, &zwplug->flags);
	}

	/*
	 * Schedule submission of the next plugged BIO if the zone is still
	 * plugged.
	 */
	if (test_bit(BLK_ZONE_WPLUG_PLUGGED, &zwplug->flags))
		kblockd_schedule_work(&zwplug->bio_work);
}

static struct blk_zone_wplug *blk_zoned_alloc_write_plugs(unsigned int nr_zones)
{
	struct blk_zone_wplug *zwplug;
	int i;

	zwplug = kvcalloc(nr_zones, sizeof(struct blk_zone_wplug), GFP_NOIO);
	if (!zwplug)
		return NULL;

	for (i = 0; i < nr_zones; i++) {
		bio_list_init(&zwplug[i].bio_list);
		INIT_WORK(&zwplug[i].bio_work, blk_zone_wplug_bio_work);
	}

	return zwplug;
}

static void blk_zoned_free_write_plugs(struct gendisk *disk,
				       struct blk_zone_wplug *zwplugs,
				       unsigned int nr_zones)
{
	struct blk_zone_wplug *zwp = zwplugs;
	int i;

	if (!zwp)
		return;

	/* Make sure we do not leak any plugged BIO */
	for (i = 0; i < nr_zones; i++, zwp++)
		blk_zone_abort_wplug(zwp);

	kvfree(zwplugs);
}

void disk_free_zone_resources(struct gendisk *disk)
{
	kfree(disk->conv_zones_bitmap);
	disk->conv_zones_bitmap = NULL;
	kfree(disk->seq_zones_wlock);
	disk->seq_zones_wlock = NULL;

	blk_zoned_free_write_plugs(disk, disk->zone_wplugs, disk->nr_zones);
	disk->zone_wplugs = NULL;
}

struct blk_revalidate_zone_args {
	struct gendisk	*disk;
	unsigned long	*conv_zones_bitmap;
	unsigned long	*seq_zones_wlock;
	unsigned int	nr_zones;
	struct blk_zone_wplug *zone_wplugs;
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

	if (!args->zone_wplugs) {
		args->zone_wplugs = blk_zoned_alloc_write_plugs(args->nr_zones);
		if (!args->zone_wplugs)
			return -ENOMEM;
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
		set_bit(BLK_ZONE_WPLUG_CONV, &args->zone_wplugs[idx].flags);
		break;
	case BLK_ZONE_TYPE_SEQWRITE_REQ:
		if (!args->seq_zones_wlock) {
			args->seq_zones_wlock =
				blk_alloc_zone_bitmap(q->node, args->nr_zones);
			if (!args->seq_zones_wlock)
				return -ENOMEM;
		}
		args->zone_wplugs[idx].capacity = zone->capacity;
		args->zone_wplugs[idx].wp_offset = blk_zone_wp_offset(zone);
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
 * BIO based drivers can also use this function as long as the device queue
 * can be safely frozen.
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
	int ret;

	if (WARN_ON_ONCE(!blk_queue_is_zoned(q)))
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
		swap(disk->zone_wplugs, args.zone_wplugs);
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
	blk_zoned_free_write_plugs(disk, args.zone_wplugs, args.nr_zones);

	return ret;
}
EXPORT_SYMBOL_GPL(blk_revalidate_disk_zones);
