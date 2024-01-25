// SPDX-License-Identifier: GPLi2.0
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
#include <linux/bit_spinlock.h>
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
 * Active zone write plug.
 */
struct blk_zone_active_wplug {
	struct blk_zone_wplug	*zwplug;
	struct bio_list		bio_list;
	struct work_struct	bio_work;
	unsigned int		wp_offset;
	unsigned int		capacity;
};

static struct kmem_cache *blk_zone_active_wplugs_cachep;

/*
 * Per-zone write plug.
 */
struct blk_zone_wplug {
	unsigned long		flags;
	union {
		struct {
			unsigned int	wp_offset;
			unsigned int	capacity;
		} info;
		struct blk_zone_active_wplug *zawplug;
	};
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
 *  - BLK_ZONE_WPLUG_ACTIVE: Indicate that the zone is active, meaning that
 *    a struct blk_zone_active_wplug was allocated for the zone.
 *  - BLK_ZONE_WPLUG_FULL: Indicate that an active zone has been fully written
 *    and that its active zone write plug can be freed.
 */
enum {
	BLK_ZONE_WPLUG_CONV = 0,
	BLK_ZONE_WPLUG_LOCKED,
	BLK_ZONE_WPLUG_PLUGGED,
	BLK_ZONE_WPLUG_ERROR,
	BLK_ZONE_WPLUG_ACTIVE,
	BLK_ZONE_WPLUG_FULL,
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
	bit_spin_lock(BLK_ZONE_WPLUG_LOCKED, &zwplug->flags);
}

static inline void blk_zone_wplug_unlock(struct blk_zone_wplug *zwplug)
{
	bit_spin_unlock(BLK_ZONE_WPLUG_LOCKED, &zwplug->flags);
}

bool blk_zone_wplug_plugged(struct gendisk *disk, unsigned int zno)
{
	struct blk_zone_wplug *zwplug = &disk->zone_wplugs[zno];

	return test_bit(BLK_ZONE_WPLUG_PLUGGED, &zwplug->flags);
}

bool blk_zone_wplug_active(struct gendisk *disk, unsigned int zno,
			   unsigned int *wp_offset)
{
	struct blk_zone_wplug *zwplug = &disk->zone_wplugs[zno];
	bool active;

	blk_zone_wplug_lock(zwplug);
	active = test_bit(BLK_ZONE_WPLUG_ACTIVE, &zwplug->flags);
	if (active)
		*wp_offset = zwplug->zawplug->wp_offset;
	else
		*wp_offset = zwplug->info.wp_offset;
	blk_zone_wplug_unlock(zwplug);

	return active;
}

static inline void blk_zone_bio_io_error(struct bio *bio)
{
	bio_clear_flag(bio, BIO_ZONE_WRITE_PLUGGING);
	bio_io_error(bio);
}

static void blk_zone_abort_wplug(struct gendisk *disk,
				 struct blk_zone_wplug *zwplug)
{
	unsigned int zno = zwplug - disk->zone_wplugs;
	struct request_queue *q;
	struct bio *bio;
	int n = 0;

	/* If the zone write plug is not active, we have nothing to do. */
	if (!test_bit(BLK_ZONE_WPLUG_ACTIVE, &zwplug->flags))
		return;

	while ((bio = bio_list_pop(&zwplug->zawplug->bio_list))) {
		q = disk->queue;
		blk_zone_bio_io_error(bio);
		blk_queue_exit(q);
		n++;
	}

	if (n)
		pr_warn_ratelimited("%s: zone %u, %u plugged BIOs aborted\n",
				    disk->disk_name, zno, n);
}

static void blk_zone_wplug_bio_work(struct work_struct *work);

/*
 * Activate an inactive zone by allocating its active write plug.
 */
static bool blk_zone_activate_wplug(struct gendisk *disk,
				    struct blk_zone_wplug *zwplug)
{
	struct blk_zone_active_wplug *zawplug;

	/* If we have an active write plug already, keep using it. */
	if (test_bit(BLK_ZONE_WPLUG_ACTIVE, &zwplug->flags))
		return true;

	/*
	 * Allocate an active write plug. This may fail if the mempool is fully
	 * used if the user partially writes too many zones, which is possible
	 * if the device has no active zone limit, if the user is not respecting
	 * the open zone limit or if the device has no limits at all.
	 */
	zawplug = mempool_alloc(disk->zone_awplugs_pool, GFP_NOWAIT);
	if (!zawplug)
		return false;

	zawplug->zwplug = zwplug;
	bio_list_init(&zawplug->bio_list);
	INIT_WORK(&zawplug->bio_work, blk_zone_wplug_bio_work);
	zawplug->capacity = zwplug->info.capacity;
	zawplug->wp_offset = zwplug->info.wp_offset;

	zwplug->zawplug = zawplug;
	set_bit(BLK_ZONE_WPLUG_ACTIVE, &zwplug->flags);

	return true;
}

static void blk_zone_free_active_wplug(struct gendisk *disk,
				       struct blk_zone_wplug *zwplug)
{
	struct blk_zone_active_wplug *zawplug = zwplug->zawplug;

	if (WARN_ON_ONCE(!test_bit(BLK_ZONE_WPLUG_ACTIVE, &zwplug->flags)))
		return;

	/* If the zone is full, it should still be plugged. */
	if (WARN_ON_ONCE(test_bit(BLK_ZONE_WPLUG_FULL, &zwplug->flags) &&
			 !test_bit(BLK_ZONE_WPLUG_PLUGGED, &zwplug->flags)))

	/* The zone BIO work should be inactive at this point. */
	if (WARN_ON_ONCE(work_busy(&zawplug->bio_work)))
		return;

	blk_zone_abort_wplug(disk, zwplug);

	clear_bit(BLK_ZONE_WPLUG_PLUGGED, &zwplug->flags);
	clear_bit(BLK_ZONE_WPLUG_ACTIVE, &zwplug->flags);
	clear_bit(BLK_ZONE_WPLUG_FULL, &zwplug->flags);

	zwplug->info.capacity = zawplug->capacity;
	zwplug->info.wp_offset = zawplug->wp_offset;

	mempool_free(zawplug, disk->zone_awplugs_pool);
}

/*
 * Return the zone write plug for a BIO targetting a sequential write
 * required zone. Given that conventional zones have no write ordering
 * constraints, NULL is returned for BIOs targetting conventional zones
 * to indicate that plugging the BIO is not needed.
 */
static inline struct blk_zone_wplug *blk_zone_lookup_wplug(struct bio *bio)
{
	struct gendisk *disk = bio->bi_bdev->bd_disk;
	struct blk_zone_wplug *zwplug = &disk->zone_wplugs[bio_zone_no(bio)];

	if (test_bit(BLK_ZONE_WPLUG_CONV, &zwplug->flags))
		return NULL;

	return zwplug;
}

bool bio_zone_is_seq(struct bio *bio)
{
	return blk_zone_lookup_wplug(bio) != NULL;
}

static void blk_zone_wplug_set_wp_offset(struct gendisk *disk,
					 struct blk_zone_wplug *zwplug,
					 unsigned int wp_offset)
{
	blk_zone_abort_wplug(disk, zwplug);
	if (test_and_clear_bit(BLK_ZONE_WPLUG_ERROR, &zwplug->flags))
		atomic_dec(&disk->zone_wplugs_nr_errors);

	if (test_bit(BLK_ZONE_WPLUG_ACTIVE, &zwplug->flags)) {
		struct blk_zone_active_wplug *zawplug = zwplug->zawplug;

		zawplug->wp_offset = wp_offset;
		if (wp_offset >= zawplug->capacity)
			set_bit(BLK_ZONE_WPLUG_FULL, &zwplug->flags);
		else
			clear_bit(BLK_ZONE_WPLUG_FULL, &zwplug->flags);

		if (!wp_offset || wp_offset >= zawplug->capacity)
			blk_zone_free_active_wplug(disk, zwplug);
	} else {
		zwplug->info.wp_offset = wp_offset;
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

	/*
	 * Set the zone write pointer offset to 0: this will abort
	 * all plugged BIOs but it is fine as resetting a zone while writes
	 * are in-flight will result in the writes failing anyway.
	 */
	blk_zone_wplug_lock(zwplug);
	blk_zone_wplug_set_wp_offset(bio->bi_bdev->bd_disk, zwplug, 0);
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
		blk_zone_wplug_set_wp_offset(disk, zwplug, 0);
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

	/*
	 * Set the zone write pointer offset to the zone size: this will abort
	 * all plugged BIOs but it is fine as finishing a zone while writes
	 * are in-flight will result in the writes failing anyway.
	 */
	blk_zone_wplug_lock(zwplug);
	blk_zone_wplug_set_wp_offset(bio->bi_bdev->bd_disk, zwplug,
				     bdev_zone_sectors(bio->bi_bdev));
	blk_zone_wplug_unlock(zwplug);
}

static inline void blk_zone_wplug_add_bio(struct blk_zone_wplug *zwplug,
					  struct bio *bio)
{
	sector_t sector = bio->bi_iter.bi_sector;
	struct bio_list *bl = &zwplug->zawplug->bio_list;
	struct bio *prev, *pos = bl->tail;

	/*
	 * Keep a reference on the BIO request queue usage. This reference will
	 * be dropped either if the BIO is failed or after it is issued and
	 * completes.
	 */
	percpu_ref_get(&bio->bi_bdev->bd_disk->queue->q_usage_counter);

	/*
	 * The BIO is being plugged and thus will have to wait for the on-going
	 * write and for all other writes already plugged. So polling makes
	 * no sense.
	 */
	bio_clear_polled(bio);

	/*
	 * We always receive BIOs after they are split and ready to be issued.
	 * The block layer guarantees that the parts of a split BIO are passed
	 * in order, so we normally only need to add a new BIO at the tail of
	 * the list. However, if the disk queue limits are modified while a
	 * BIO is waiting in the list, a BIO may be split again when it is
	 * unplugged and issued with submit_bio_noacct_nocheck() by the zone
	 * BIO work. For such case, doing a sort insert of the BIO in the list
	 * is required as the later parts of the split BIO have to be inserted
	 * at the head of the list. This should be a rare case, so we optimize
	 * for the regular add-at-tail case first.
	 */
	if (!pos || bio_op(bio) == REQ_OP_ZONE_APPEND ||
	    sector > pos->bi_iter.bi_sector) {
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

/*
 * Prepare a zone write bio for submission by incrementing the write pointer and
 * setting up the zone append emulation if needed.
 */
static bool blk_zone_wplug_prepare_bio(struct blk_zone_wplug *zwplug,
				       struct bio *bio)
{
	struct blk_zone_active_wplug *zawplug = zwplug->zawplug;

	/*
	 * Check that the user is not attempting to write to a full zone.
	 * We know such BIO will fail, and that would potentially overflow our
	 * write pointer offset, causing zone append BIOs for one zone to be
	 * directed at the following zone.
         */
	if (zawplug->wp_offset >= zawplug->capacity)
		return false;

	if (bio_op(bio) == REQ_OP_ZONE_APPEND) {
		/*
		 * Use a regular write starting at the current write pointer.
		 * Note that the BIO already has REQ_NOMERGE set, similarly to
		 * native zone append operations.
		 */
		bio->bi_opf &= ~REQ_OP_MASK;
		bio->bi_opf |= REQ_OP_WRITE;
		bio->bi_iter.bi_sector += zawplug->wp_offset;

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
		if (bio_offset_from_zone_start(bio) != zawplug->wp_offset)
			return false;
	}

	/* Advance the zone write pointer offset. */
	zawplug->wp_offset += bio_sectors(bio);
	if (zawplug->wp_offset >= zawplug->capacity)
		set_bit(BLK_ZONE_WPLUG_FULL, &zwplug->flags);

	return true;
}

static bool __blk_zone_write_plug_bio(struct bio *bio)
{
	struct blk_zone_wplug *zwplug;

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

	zwplug = blk_zone_lookup_wplug(bio);
	if (!zwplug)
		return false;

	blk_zone_wplug_lock(zwplug);

	if (!blk_zone_activate_wplug(bio->bi_bdev->bd_disk, zwplug)) {
		bio_io_error(bio);
		blk_zone_wplug_unlock(zwplug);
		return true;
	}

	/*
	 * Indicate that this BIO is being handled using zone write plugging.
	 * When it is the turn of this BIO to execute, it will be the only
	 * write BIO in-flight for the zone, so we will have absolutely no
	 * chances of merging it, so do not try.
	 */
	bio_set_flag(bio, BIO_ZONE_WRITE_PLUGGING);
	bio->bi_opf |= REQ_NOMERGE;

	/*
	 * If we have not already plugged the zone, let the BIO execute.
	 * Otherwise, add the BIO to the plug bio list.
	 */
	if (test_and_set_bit(BLK_ZONE_WPLUG_PLUGGED, &zwplug->flags)) {
		blk_zone_wplug_add_bio(zwplug, bio);
		blk_zone_wplug_unlock(zwplug);
		return true;
	}

	if (!blk_zone_wplug_prepare_bio(zwplug, bio)) {
		clear_bit(BLK_ZONE_WPLUG_PLUGGED, &zwplug->flags);
		blk_zone_bio_io_error(bio);
		blk_zone_wplug_unlock(zwplug);
		return true;
	}

	blk_zone_wplug_unlock(zwplug);

	return false;
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

	if (!bdev->bd_disk->zone_wplugs)
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
	 * Zone append operations that need emulation must also be plugged so
	 * that these operations can be changed into regular writes.
	 * Zone reset, reset all and finish commands need special treatment
	 * to correctly track the write pointer offset of zones when zone
	 * append emulation is needed. These commands are not plugged as we do
	 * not need serialization with write and append operations. It is the
	 * responsibility of the user to not issue reset and finish commands
	 * when write operations are in flight.
	 *
	 * Note: for native zone append operations, we do not do any tracking of
	 * the zone write pointer offset. This means that zones written only
	 * using zone append operations will never be activated, thus avoiding
	 * any overhead. If the user mixes regular writes and native zone append
	 * operations for the same zone, the zone write plug will be activated
	 * and have an incorrect write pointer offset. That is fine as mixing
	 * these operations will very likely fail anyway, in which case the
	 * zone error handling will recover a correct write pointer offset.
	 */
	switch (bio_op(bio)) {
	case REQ_OP_ZONE_APPEND:
		if (!bdev_emulates_zone_append(bdev))
			return false;
		fallthrough;
	case REQ_OP_WRITE:
	case REQ_OP_WRITE_ZEROES:
		return __blk_zone_write_plug_bio(bio);
	case REQ_OP_ZONE_RESET:
		blk_zone_handle_reset(bio);
		return false;
	case REQ_OP_ZONE_RESET_ALL:
		blk_zone_handle_reset_all(bio);
		return false;
	case REQ_OP_ZONE_FINISH:
		blk_zone_handle_finish(bio);
		return false;
	default:
		return false;
	}

	return false;
}
EXPORT_SYMBOL_GPL(blk_zone_write_plug_bio);

static inline void blk_zone_wplug_error(struct gendisk *disk,
					struct blk_zone_wplug *zwplug)
{
	if (!test_and_set_bit(BLK_ZONE_WPLUG_ERROR, &zwplug->flags))
		return;

	atomic_inc(&disk->zone_wplugs_nr_errors);
	kblockd_mod_delayed_work_on(WORK_CPU_UNBOUND,
				    &disk->zone_wplugs_work, 0);
}

void blk_zone_write_bio_endio(struct bio *bio)
{
	struct blk_zone_wplug *zwplug = blk_zone_lookup_wplug(bio);

	/* Make sure we do not see this BIO again by clearing the plug flag. */
	bio_clear_flag(bio, BIO_ZONE_WRITE_PLUGGING);

	/* The BIO zone write plug should be valid and plugged. */
	if (WARN_ON_ONCE(!zwplug ||
			 !test_bit(BLK_ZONE_WPLUG_PLUGGED, &zwplug->flags)))
		return;

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
	 * and schedule the disk work to recover. This will abort all
	 * plugged BIOs so we do not need to schedule the plug BIO work.
	 */
	if (bio->bi_status != BLK_STS_OK) {
		blk_zone_wplug_error(bio->bi_bdev->bd_disk, zwplug);
		return;
	}

	/*
	 * If the zone was fully written, free its active write plug. Otherwise,
	 * schedule the submission of the next plugged BIO.
	 */
	if (test_bit(BLK_ZONE_WPLUG_FULL, &zwplug->flags)) {
		blk_zone_wplug_lock(zwplug);
		blk_zone_free_active_wplug(bio->bi_bdev->bd_disk, zwplug);
		blk_zone_wplug_unlock(zwplug);
		return;
	}

	/* Schedule submission of the next plugged BIO. */
	kblockd_schedule_work(&zwplug->zawplug->bio_work);
}

static void blk_zone_wplug_bio_work(struct work_struct *work)
{
	struct blk_zone_active_wplug *zawplug =
		container_of(work, struct blk_zone_active_wplug, bio_work);
	struct blk_zone_wplug *zwplug = zawplug->zwplug;
	struct request_queue *q;
	struct bio *bio;

	/*
	 * Unplug and submit the next plugged BIO. If we do not have any, clear
	 * the plugged flag.
	 */
	blk_zone_wplug_lock(zwplug);
	while ((bio = bio_list_pop(&zawplug->bio_list))) {
		if (blk_zone_wplug_prepare_bio(zwplug, bio)) {
			blk_zone_wplug_unlock(zwplug);
			submit_bio_noacct_nocheck(bio);
			return;
		}

		q = bio->bi_bdev->bd_disk->queue;
		blk_zone_bio_io_error(bio);
		blk_queue_exit(q);
	}
	clear_bit(BLK_ZONE_WPLUG_PLUGGED, &zwplug->flags);
	blk_zone_wplug_unlock(zwplug);
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

static int blk_zone_wplug_get_zone_cb(struct blk_zone *zone,
				      unsigned int idx, void *data)
{
	struct blk_zone *zonep = data;

	*zonep = *zone;

	return 0;
}

static int blk_zone_wplug_handle_error(struct gendisk *disk,
				       struct blk_zone_wplug *zwplug)
{
	unsigned int zno = zwplug - disk->zone_wplugs;
	sector_t zone_start_sector = bdev_zone_sectors(disk->part0) << zno;
	unsigned int noio_flag;
	struct blk_zone zone;
	int ret;

	blk_zone_wplug_lock(zwplug);

	/* A zone reseet or finish may have already cleared the error. */
	if (!test_bit(BLK_ZONE_WPLUG_ERROR, &zwplug->flags))
		goto unlock;

        /*
	 * We had a write error, meaning that the sequential write pattern
	 * is broken and that the plugged BIOs will fail. So abort them now.
         */
	blk_zone_abort_wplug(disk, zwplug);

	noio_flag = memalloc_noio_save();
	ret = disk->fops->report_zones(disk, zone_start_sector, 1,
				       blk_zone_wplug_get_zone_cb, &zone);
	memalloc_noio_restore(noio_flag);

	/*
	 * If report zones failed, leave the error flag. Recovery will be
	 * attempted again later.
	 */
	if (ret == 1) {
		if (test_bit(BLK_ZONE_WPLUG_ACTIVE, &zwplug->flags))
			zwplug->info.capacity = zone.capacity;
		else
			zwplug->zawplug->capacity = zone.capacity;
		blk_zone_wplug_set_wp_offset(disk, zwplug,
					     blk_zone_wp_offset(&zone));
		ret = 0;
	} else {
		ret = -EIO;
	}

unlock:
	blk_zone_wplug_unlock(zwplug);

	return ret;
}

static void disk_zone_wplugs_work(struct work_struct *work)
{
	 struct gendisk *disk =
		 container_of(work, struct gendisk, zone_wplugs_work.work);
	 struct blk_zone_wplug *zwplug = disk->zone_wplugs;
	 unsigned int i, ret = 0;

	 while (!ret && atomic_read(&disk->zone_wplugs_nr_errors)) {

		 if (blk_queue_enter(disk->queue, 0))
			 break;

		 for (i = 0; i < disk->nr_zones; i++, zwplug++) {
			 if (!test_bit(BLK_ZONE_WPLUG_ERROR, &zwplug->flags))
				 continue;
			 ret = blk_zone_wplug_handle_error(disk, zwplug);
			 if (ret)
				 break;
		 }

		 blk_queue_exit(disk->queue);
	 }

	 /*
	  * If we still have pending errors, reschedule with a delay to
	  * not hit the device hard with report zones.
	  */
	 if (atomic_read(&disk->zone_wplugs_nr_errors))
		 kblockd_mod_delayed_work_on(WORK_CPU_UNBOUND,
					     &disk->zone_wplugs_work, HZ / 10);
}

static struct blk_zone_wplug *blk_zone_alloc_write_plugs(unsigned int nr_zones)
{
	return kvcalloc(nr_zones, sizeof(struct blk_zone_wplug), GFP_NOIO);
}

static void blk_zone_free_write_plugs(struct gendisk *disk,
				      struct blk_zone_wplug *zwplugs,
				      unsigned int nr_zones)
{
	struct blk_zone_wplug *zwp = zwplugs;
	int i;

	if (!zwp)
		return;

	/* Make sure we do not leak any plugged BIO */
	for (i = 0; i < nr_zones; i++, zwp++) {
		blk_zone_wplug_lock(zwp);
		blk_zone_abort_wplug(disk, zwp);
		clear_bit(BLK_ZONE_WPLUG_PLUGGED, &zwp->flags);
		if (test_bit(BLK_ZONE_WPLUG_ACTIVE, &zwp->flags))
			blk_zone_free_active_wplug(disk, zwp);
		blk_zone_wplug_unlock(zwp);
	}

	kvfree(zwplugs);
}

void disk_free_zone_resources(struct gendisk *disk)
{
	if (disk->zone_wplugs)
		cancel_delayed_work_sync(&disk->zone_wplugs_work);

	blk_zone_free_write_plugs(disk, disk->zone_wplugs, disk->nr_zones);
	disk->zone_wplugs = NULL;

	mempool_destroy(disk->zone_awplugs_pool);
	disk->zone_awplugs_pool = NULL;
}

struct blk_revalidate_zone_args {
	struct gendisk	*disk;
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

	/* Check zone type */
	switch (zone->type) {
	case BLK_ZONE_TYPE_CONVENTIONAL:
		set_bit(BLK_ZONE_WPLUG_CONV, &args->zone_wplugs[idx].flags);
		break;
	case BLK_ZONE_TYPE_SEQWRITE_REQ:
		args->zone_wplugs[idx].info.capacity = zone->capacity;
		args->zone_wplugs[idx].info.wp_offset = blk_zone_wp_offset(zone);
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

static int blk_zone_active_wplugs_pool_size(struct gendisk *disk,
					    unsigned int nr_zones)
{
	unsigned int pool_size;

	/*
	 * Size the disk pool of active zone write plugs with enough elements
	 * given the device open and active zones limits. There may be no
	 * device limit, in which case, we use a page worth of active write
	 * plugs (BLK_ZONE_ACTIVE_WPLUG_NR).
	 */
	pool_size = max(disk->max_active_zones,
			disk->max_open_zones);
	if (!pool_size) {
		/* Default to two pages worth of active plugs. */
		pool_size = 128;
	}

	return min(pool_size, nr_zones);
}

/**
 * blk_revalidate_disk_zones - (re)allocate and initialize zone write plugs
 * @disk:	Target disk
 *
 * Helper function for low-level device drivers to check, (re) allocate and
 * initialize resources used for managing zoned disks. This function should
 * normally be called by blk-mq based drivers when a zoned gendisk is probed
 * and when the zone configuration of the gendisk changes (e.g. after a format).
 * Before calling this function, the device driver must already have set the
 * device zone size (chunk_sector limit) and the max zone append limit.
 * BIO based drivers can also use this function as long as the device queue
 * can be safely frozen.
 */
int blk_revalidate_disk_zones(struct gendisk *disk)
{
	struct request_queue *q = disk->queue;
	sector_t zone_sectors = q->limits.chunk_sectors;
	sector_t capacity = get_capacity(disk);
	struct blk_revalidate_zone_args args = { };
	unsigned int nr_zones, noio_flag;
	unsigned int pool_size;
	int ret = -ENOMEM;

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

	nr_zones = (capacity + zone_sectors - 1) >> ilog2(zone_sectors);

	/*
	 * Ensure that all memory allocations in this context are done as if
	 * GFP_NOIO was specified.
	 */
	noio_flag = memalloc_noio_save();

	args.disk = disk;
	args.zone_wplugs = blk_zone_alloc_write_plugs(nr_zones);
	if (!args.zone_wplugs)
		goto out_restore_noio;

	pool_size = blk_zone_active_wplugs_pool_size(disk, nr_zones);
	if (!disk->zone_wplugs) {
		INIT_DELAYED_WORK(&disk->zone_wplugs_work,
				  disk_zone_wplugs_work);
		atomic_set(&disk->zone_wplugs_nr_errors, 0);
		disk->zone_awplugs_pool_size = pool_size;
		disk->zone_awplugs_pool =
			mempool_create_slab_pool(disk->zone_awplugs_pool_size,
						 blk_zone_active_wplugs_cachep);
		if (!disk->zone_awplugs_pool)
			goto out_restore_noio;
	} else if (disk->zone_awplugs_pool_size != pool_size) {
		ret = mempool_resize(disk->zone_awplugs_pool, pool_size);
		if (ret)
			goto out_restore_noio;
		disk->zone_awplugs_pool_size = pool_size;
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
	 * Install the new write plugs and update nr_zones only once the queue
	 * is frozen and all I/Os are completed.
	 */
	blk_mq_freeze_queue(q);
	if (ret > 0) {
		disk->nr_zones = nr_zones;
		swap(disk->zone_wplugs, args.zone_wplugs);
		blk_zone_free_write_plugs(disk, args.zone_wplugs, nr_zones);
		ret = 0;
	} else {
		pr_warn("%s: failed to revalidate zones\n", disk->disk_name);
		blk_zone_free_write_plugs(disk, args.zone_wplugs, nr_zones);
		disk_free_zone_resources(disk);
	}
	blk_mq_unfreeze_queue(q);

	return ret;

out_restore_noio:
	memalloc_noio_restore(noio_flag);
	blk_zone_free_write_plugs(disk, args.zone_wplugs, nr_zones);
	return ret;
}
EXPORT_SYMBOL_GPL(blk_revalidate_disk_zones);

void blk_zone_dev_init(void)
{
	blk_zone_active_wplugs_cachep =
		kmem_cache_create("blk_zone_active_wplug",
				  sizeof(struct blk_zone_active_wplug), 0,
				  SLAB_PANIC, NULL);
}
