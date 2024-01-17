// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Western Digital Corporation or its affiliates.
 */

#include "blk.h"
#include "blk-mq-debugfs.h"

int queue_zone_plugged_wplugs_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	unsigned int i;

	if (!q->disk->zone_wplugs)
		return 0;

	for (i = 0; i < q->disk->nr_zones; i++)
		if (blk_zone_wplug_plugged(q->disk, i))
			seq_printf(m, "%u\n", i);

	return 0;
}

int queue_zone_active_wplugs_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	unsigned int i, wp_offset;

	if (!q->disk->zone_wplugs)
		return 0;

	for (i = 0; i < q->disk->nr_zones; i++)
		if (blk_zone_wplug_active(q->disk, i, &wp_offset))
			seq_printf(m, "%u %u\n", i, wp_offset);

	return 0;
}
