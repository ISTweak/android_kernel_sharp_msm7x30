/*
 * Block driver for media (i.e., flash cards)
 *
 * Copyright 2002 Hewlett-Packard Company
 * Copyright 2005-2008 Pierre Ossman
 * Copyright (C) 2011 SHARP CORPORATION
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * HEWLETT-PACKARD COMPANY MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
 * AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
 * FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 * Many thanks to Alessandro Rubini and Jonathan Corbet!
 *
 * Author:  Andrew Christian
 *          28 May 2002
 */
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/hdreg.h>
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/string_helpers.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>

#include <asm/system.h>
#include <asm/uaccess.h>

#include "queue.h"

MODULE_ALIAS("mmc:block");

/*
 * max 32 partitions per card
 */
#define MMC_SHIFT	5
#define MMC_NUM_MINORS	(256 >> MMC_SHIFT)

static DECLARE_BITMAP(dev_use, MMC_NUM_MINORS);

/*
 * There is one mmc_blk_data per slot.
 */
struct mmc_blk_data {
	spinlock_t	lock;
	struct gendisk	*disk;
	struct mmc_queue queue;

	unsigned int	usage;
	unsigned int	read_only;
};

static DEFINE_MUTEX(open_lock);

static struct mmc_blk_data *mmc_blk_get(struct gendisk *disk)
{
	struct mmc_blk_data *md;

	mutex_lock(&open_lock);
	md = disk->private_data;
	if (md && md->usage == 0)
		md = NULL;
	if (md)
		md->usage++;
	mutex_unlock(&open_lock);

	return md;
}

static void mmc_blk_put(struct mmc_blk_data *md)
{
	mutex_lock(&open_lock);
	md->usage--;
	if (md->usage == 0) {
		int devidx = md->disk->first_minor >> MMC_SHIFT;

		blk_cleanup_queue(md->queue.queue);

		__clear_bit(devidx, dev_use);

		put_disk(md->disk);
		kfree(md);
	}
	mutex_unlock(&open_lock);
}

static int mmc_blk_open(struct block_device *bdev, fmode_t mode)
{
	struct mmc_blk_data *md = mmc_blk_get(bdev->bd_disk);
	int ret = -ENXIO;

	if (md) {
		if (md->usage == 2)
			check_disk_change(bdev);
		ret = 0;

		if ((mode & FMODE_WRITE) && md->read_only) {
			mmc_blk_put(md);
			ret = -EROFS;
		}
	}

	return ret;
}

static int mmc_blk_release(struct gendisk *disk, fmode_t mode)
{
	struct mmc_blk_data *md = disk->private_data;

	mmc_blk_put(md);
	return 0;
}

static int
mmc_blk_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	geo->cylinders = get_capacity(bdev->bd_disk) / (4 * 16);
	geo->heads = 4;
	geo->sectors = 16;
	return 0;
}

static const struct block_device_operations mmc_bdops = {
	.open			= mmc_blk_open,
	.release		= mmc_blk_release,
	.getgeo			= mmc_blk_getgeo,
	.owner			= THIS_MODULE,
};

struct mmc_blk_request {
	struct mmc_request	mrq;
	struct mmc_command	cmd;
	struct mmc_command	stop;
	struct mmc_data		data;
};

static u32 mmc_sd_num_wr_blocks(struct mmc_card *card)
{
	int err;
	u32 result;
	__be32 *blocks;

	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	unsigned int timeout_us;

	struct scatterlist sg;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_APP_CMD;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err)
		return (u32)-1;
	if (!mmc_host_is_spi(card->host) && !(cmd.resp[0] & R1_APP_CMD))
		return (u32)-1;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = SD_APP_SEND_NUM_WR_BLKS;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	memset(&data, 0, sizeof(struct mmc_data));

	data.timeout_ns = card->csd.tacc_ns * 100;
	data.timeout_clks = card->csd.tacc_clks * 100;

	timeout_us = data.timeout_ns / 1000;
	timeout_us += data.timeout_clks * 1000 /
		(card->host->ios.clock / 1000);

	if (timeout_us > 100000) {
		data.timeout_ns = 100000000;
		data.timeout_clks = 0;
	}

	data.blksz = 4;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	memset(&mrq, 0, sizeof(struct mmc_request));

	mrq.cmd = &cmd;
	mrq.data = &data;

	blocks = kmalloc(4, GFP_KERNEL);
	if (!blocks)
		return (u32)-1;

	sg_init_one(&sg, blocks, 4);

	mmc_wait_for_req(card->host, &mrq);

	result = ntohl(*blocks);
	kfree(blocks);

	if (cmd.error || data.error)
		result = (u32)-1;

	return result;
}

static u32 get_card_status(struct mmc_card *card, struct request *req)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_SEND_STATUS;
	if (!mmc_host_is_spi(card->host))
		cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err)
		printk(KERN_ERR "%s: error %d sending status comand",
		       req->rq_disk->disk_name, err);
	return cmd.resp[0];
}

static int
mmc_blk_set_blksize(struct mmc_blk_data *md, struct mmc_card *card)
{
	struct mmc_command cmd;
	int err;

	/* Block-addressed cards ignore MMC_SET_BLOCKLEN. */
	if (mmc_card_blockaddr(card))
		return 0;

	mmc_claim_host(card->host);
	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.arg = 512;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 5);
	mmc_release_host(card->host);

	if (err) {
		printk(KERN_ERR "%s: unable to set block size to %d: %d\n",
			md->disk->disk_name, cmd.arg, err);
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_RELIABLE_WRITE_EMMC_CUST_SH
/* SH cust S */
extern unsigned int ext_csd_en_rel_wr_val;
extern unsigned int ext_csd_rel_wr_sec_c_val;
extern unsigned int ext_csd_hpi_support_val;

struct mmc_blk_pos {
	u32 start;
	u32 end;
};

#define MMC_RELIABLE_W_AREA_NUM    0
static const struct mmc_blk_pos mmc_reliable_w_area[] =
	{{0xFFFFFFFF, 0xFFFFFFFF},
	{0xFFFFFFFF, 0xFFFFFFFF},
	{0xFFFFFFFF, 0xFFFFFFFF}};

static int mmc_check_reliable_w_area(u32 start, u32 end)
{
	u32 i;

	for (i = 0; i < MMC_RELIABLE_W_AREA_NUM; i++) {
		/* start block check */
		if ((mmc_reliable_w_area[i].start <= start)
		&& (start < mmc_reliable_w_area[i].end))
			return -1;

		/* end block check */
		if ((mmc_reliable_w_area[i].start < end)
		&& (end <= mmc_reliable_w_area[i].end))
			return -1;

		/* start & end block check */
		if ((start < mmc_reliable_w_area[i].start)
		&& (mmc_reliable_w_area[i].end < end))
			return -1;
	}

	return 0;
}
/* SH cust E */
#endif

/* SH cust S */
#ifdef CONFIG_RW_PROTECT_EMMC_CUST_SH

#define MMC_MBR_PARTITION	0
#define MMC_EBR_PARTITION	4

struct mmc_protect_inf {
	u32 partition;
	u32 protect;
};

#define MMC_PROTECT_READ	0x01
#define MMC_PROTECT_WRITE	0x02

#ifdef CONFIG_ANDROID_ENGINEERING
static const struct mmc_protect_inf mmc_protect_part[] = {
	};
#else
static const struct mmc_protect_inf mmc_protect_part[] = {
	{0,	MMC_PROTECT_WRITE			},
	{1,	MMC_PROTECT_READ | MMC_PROTECT_WRITE	},
	{2,	MMC_PROTECT_READ | MMC_PROTECT_WRITE	},
	{3,	MMC_PROTECT_READ | MMC_PROTECT_WRITE	},
	{4,	MMC_PROTECT_WRITE			},
	{6,	MMC_PROTECT_READ | MMC_PROTECT_WRITE	},
	{7,	MMC_PROTECT_READ | MMC_PROTECT_WRITE	},
	{8,	MMC_PROTECT_WRITE			},
	{9,	MMC_PROTECT_WRITE			},
	{10,	MMC_PROTECT_READ | MMC_PROTECT_WRITE	},
	{11,	MMC_PROTECT_WRITE			}};
#endif

#define MMC_PROTECT_AREA_NUM	\
		(sizeof(mmc_protect_part) / sizeof(struct mmc_protect_inf))

static int mmc_check_protect_part(struct request *req, u32 *blocks, int *error)
{
	u32 i;
	u32 j;
	u32 protect;
	u32 part_start;
	u32 part_end;

	struct gendisk *disk = req->rq_disk;
	u32 start = blk_rq_pos(req);
	u32 end = blk_rq_pos(req) + blk_rq_sectors(req);
	u32 flg = (u32)rq_data_dir(req);

	/*
	* Partition Information check
	*/
	if (disk->part_tbl->len <= 1) {
		*blocks = end - start;
		return 0;
	}

	/*
	* MBR/EBR check
	*/
	if (*error == 0)
		goto partial_check;

	for (i = 0; i < MMC_PROTECT_AREA_NUM; i++) {

		j = mmc_protect_part[i].partition;

		if ((j != MMC_MBR_PARTITION) && (j != MMC_EBR_PARTITION))
			continue;

		protect = mmc_protect_part[i].protect;

		if ((flg == (u32)READ) && ((protect & MMC_PROTECT_READ) != 0))
			continue;

		if ((flg == (u32)WRITE) && ((protect & MMC_PROTECT_WRITE) != 0))
			continue;

		if ((j + 1) < disk->part_tbl->len) {

			part_start = disk->part_tbl->part[j]->start_sect;
			part_end   = disk->part_tbl->part[j+1]->start_sect;

			/* start block check */
			if ((part_start <= start) && (start < part_end))
				goto partial_check;

			/* end block check */
			if ((part_start < end) && (end <= part_end))
				goto partial_check;

			/* start & end block check */
			if ((start < part_start) && (part_end < end))
				goto partial_check;
		}

	}

	/*
	* whole check
	*/
	*blocks = end - start;

	for (i = 0; i < MMC_PROTECT_AREA_NUM; i++) {

		protect = mmc_protect_part[i].protect;

		if ((flg == (u32)READ) && ((protect & MMC_PROTECT_READ) == 0))
			continue;

		if ((flg == (u32)WRITE) && ((protect & MMC_PROTECT_WRITE) == 0))
			continue;

		j = mmc_protect_part[i].partition;

		if ((j == MMC_MBR_PARTITION) || (j == MMC_EBR_PARTITION)) {
			if ((j + 1) >= disk->part_tbl->len)
				continue;
			part_start = disk->part_tbl->part[j]->start_sect;
			part_end = disk->part_tbl->part[j + 1]->start_sect;
		} else {
			if (j >= disk->part_tbl->len)
				continue;
			part_start = disk->part_tbl->part[j]->start_sect;
			part_end = part_start
			+ disk->part_tbl->part[j]->nr_sects;
		}

		/* start block check */
		if ((part_start <= start) && (start < part_end))
			return -EROFS;

		/* end block check */
		if ((part_start < end) && (end <= part_end))
			return -EROFS;

		/* start & end block check */
		if ((start < part_start) && (part_end < end))
			return -EROFS;

	}

	return 0;

 partial_check:

	*error = 0;

	/*
	* partial check
	*/
	for (i = 0; i < MMC_PROTECT_AREA_NUM; i++) {

		protect = mmc_protect_part[i].protect;

		if ((flg == (u32)READ) && ((protect & MMC_PROTECT_READ) == 0))
			continue;

		if ((flg == (u32)WRITE) && ((protect & MMC_PROTECT_WRITE) == 0))
			continue;

		j = mmc_protect_part[i].partition;

		if ((j == MMC_MBR_PARTITION) || (j == MMC_EBR_PARTITION)) {
			if ((j + 1) >= disk->part_tbl->len)
				continue;
			part_start = disk->part_tbl->part[j]->start_sect;
			part_end = disk->part_tbl->part[j + 1]->start_sect;
		} else {
			if (j >= disk->part_tbl->len)
				continue;
			part_start = disk->part_tbl->part[j]->start_sect;
			part_end = part_start
			+ disk->part_tbl->part[j]->nr_sects;
		}

		/* start block check */
		if ((part_start <= start) && (start < part_end)) {
			if (end < part_end)
				*blocks = end - start;
			else
				*blocks = part_end - start;
			return -EROFS;
		}
		/* end block check */
		if ((part_start < end) && (end <= part_end)) {
			*blocks = part_start - start;
			return 0;
		}
		/* start & end block check */
		if ((start < part_start) && (part_end < end)) {
			*blocks = part_start - start;
			return 0;
		}
	}

	*blocks = end - start;
	return 0;
}

#endif
/* SH cust E */

static int mmc_blk_issue_rq(struct mmc_queue *mq, struct request *req)
{
	struct mmc_blk_data *md = mq->data;
	struct mmc_card *card = md->queue.card;
	struct mmc_blk_request brq;
	int ret = 1, disable_multi = 0;

/* SH cust S */
#ifdef CONFIG_RW_PROTECT_EMMC_CUST_SH
	int blocks;
	int protect_err = -EROFS;
#endif
#ifdef CONFIG_RELIABLE_WRITE_EMMC_CUST_SH
	int rel_wr_on = 0;
	int en_rel_wr_val = ext_csd_en_rel_wr_val;
	int rel_wr_sec_c_val = ext_csd_rel_wr_sec_c_val;
	int hpi_support_val = ext_csd_hpi_support_val;
#endif
#ifdef CONFIG_ERR_RETRY_EMMC_CUST_SH
	int err_retry_max = 3;
	int err_retry = 0;
#endif
/* SH cust E */

#ifdef CONFIG_MMC_BLOCK_DEFERRED_RESUME
	if (mmc_bus_needs_resume(card->host)) {
		mmc_resume_bus(card->host);
		mmc_blk_set_blksize(md, card);
	}
#endif

	mmc_claim_host(card->host);

#ifdef CONFIG_ERR_RETRY_EMMC_CUST_SH
cmd_err_retry:
#endif

	do {
		struct mmc_command cmd;
		u32 readcmd, writecmd, status = 0;

		memset(&brq, 0, sizeof(struct mmc_blk_request));
		brq.mrq.cmd = &brq.cmd;
		brq.mrq.data = &brq.data;

		brq.cmd.arg = blk_rq_pos(req);
		if (!mmc_card_blockaddr(card))
			brq.cmd.arg <<= 9;
		brq.cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
		brq.data.blksz = 512;
		brq.stop.opcode = MMC_STOP_TRANSMISSION;
		brq.stop.arg = 0;
		brq.stop.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
		brq.data.blocks = blk_rq_sectors(req);

/* SH cust S */
#ifdef CONFIG_RW_PROTECT_EMMC_CUST_SH

		if (mmc_check_protect_part(req, &blocks, &protect_err) != 0) {

			printk(KERN_WARNING "%s:%u: protect status %d "
				"sector %u, nr %u, blocks %d\n",
				req->rq_disk->disk_name,
				(unsigned)(rq_data_dir(req)), protect_err,
				(unsigned)(blk_rq_pos(req)),
				(unsigned)(blk_rq_sectors(req)), blocks);

			if ((brq.data.blocks <= blocks) && (protect_err != 0)) {
				mmc_release_host(card->host);
				spin_lock_irq(&md->lock);
				while (ret)
					ret = __blk_end_request(req,
							protect_err,
							blk_rq_cur_bytes(req));
				spin_unlock_irq(&md->lock);
				return 0;
			}

			if ((brq.data.blocks <= blocks) && (protect_err == 0)) {
				spin_lock_irq(&md->lock);
				while (ret)
					ret = __blk_end_request(req,
							protect_err,
							blk_rq_cur_bytes(req));
				spin_unlock_irq(&md->lock);
				mmc_release_host(card->host);
				return 1;
			}

			spin_lock_irq(&md->lock);
			ret = __blk_end_request(req, protect_err, blocks * 512);
			spin_unlock_irq(&md->lock);
			continue;

		} else {

			brq.data.blocks = blocks;

		}

#endif
/* SH cust E */

#ifdef CONFIG_RELIABLE_WRITE_EMMC_CUST_SH
		/* SH cust S */
		rel_wr_on = 0;
		if ((rq_data_dir(req) == WRITE)
		&& (mmc_check_reliable_w_area(blk_rq_pos(req),
			blk_rq_pos(req) + blk_rq_sectors(req)) != 0))
			rel_wr_on = 1;

		if (rel_wr_on != 0) {

			int err;

			/* check EN_REL_WR, HPI_SUPPORT */
			if ((en_rel_wr_val == 0) && (hpi_support_val == 0)) {
				if (brq.data.blocks >= rel_wr_sec_c_val)
					brq.data.blocks = rel_wr_sec_c_val;
				else
					brq.data.blocks = 1;
			}

			if ((en_rel_wr_val == 0) && (hpi_support_val == 1))
				brq.data.blocks = 1;

			/* issue CMD23 */
			cmd.opcode = MMC_SET_BLOCK_COUNT;
			cmd.arg = brq.data.blocks;
			cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
			err = mmc_wait_for_cmd(card->host, &cmd, 5);
			if (err) {
				printk(KERN_ERR "%s: error %d "
					"set block count\n",
					req->rq_disk->disk_name, err);
				goto cmd_err;
			}

		}
		/* SH cust E */
#endif

		/*
		 * The block layer doesn't support all sector count
		 * restrictions, so we need to be prepared for too big
		 * requests.
		 */
		if (brq.data.blocks > card->host->max_blk_count)
			brq.data.blocks = card->host->max_blk_count;

		/*
		 * After a read error, we redo the request one sector at a time
		 * in order to accurately determine which sectors can be read
		 * successfully.
		 */
		if (disable_multi && brq.data.blocks > 1)
			brq.data.blocks = 1;

#ifdef CONFIG_ACCESS_OPTIMIZE_EMMC_CUST_SH
		/* SH cust S */
		if (card->cid.manfid == 0x11) {
			if ((rq_data_dir(req) == WRITE)
			&& (brq.data.blocks <= 32)) {
				u32 unit = ((mmc_card_blockaddr(card)) ?
				1 : brq.data.blksz);
				if ((brq.cmd.arg / unit / 16)
				< ((brq.cmd.arg / unit)
				+ brq.data.blocks - 1) / 16) {
					brq.data.blocks =
					16 - ((brq.cmd.arg / unit) % 16);
				}
			}
		}
		/* SH cust E */
#endif
		if (brq.data.blocks > 1) {
			/* SPI multiblock writes terminate using a special
			 * token, not a STOP_TRANSMISSION request.
			 */
			if (!mmc_host_is_spi(card->host)
					|| rq_data_dir(req) == READ)
				brq.mrq.stop = &brq.stop;
			readcmd = MMC_READ_MULTIPLE_BLOCK;
			writecmd = MMC_WRITE_MULTIPLE_BLOCK;
		} else {
			brq.mrq.stop = NULL;
			readcmd = MMC_READ_SINGLE_BLOCK;
			writecmd = MMC_WRITE_BLOCK;
		}

		if (rq_data_dir(req) == READ) {
			brq.cmd.opcode = readcmd;
			brq.data.flags |= MMC_DATA_READ;
		} else {
			brq.cmd.opcode = writecmd;
			brq.data.flags |= MMC_DATA_WRITE;
		}

#ifdef CONFIG_RELIABLE_WRITE_EMMC_CUST_SH
		/* SH cust S */
		if (rel_wr_on != 0) {
			brq.mrq.stop = NULL;
			brq.cmd.opcode = MMC_WRITE_MULTIPLE_BLOCK;
		}
		/* SH cust E */
#endif

		mmc_set_data_timeout(&brq.data, card);

		brq.data.sg = mq->sg;
		brq.data.sg_len = mmc_queue_map_sg(mq);

		/*
		 * Adjust the sg list so it is the same size as the
		 * request.
		 */
		if (brq.data.blocks != blk_rq_sectors(req)) {
			int i, data_size = brq.data.blocks << 9;
			struct scatterlist *sg;

			for_each_sg(brq.data.sg, sg, brq.data.sg_len, i) {
				data_size -= sg->length;
				if (data_size <= 0) {
					sg->length += data_size;
					i++;
					break;
				}
			}
			brq.data.sg_len = i;
		}

		mmc_queue_bounce_pre(mq);

		mmc_wait_for_req(card->host, &brq.mrq);

		mmc_queue_bounce_post(mq);

		/*
		 * Check for errors here, but don't jump to cmd_err
		 * until later as we need to wait for the card to leave
		 * programming mode even when things go wrong.
		 */
		if (brq.cmd.error || brq.data.error || brq.stop.error) {
			if (brq.data.blocks > 1 && rq_data_dir(req) == READ) {
				/* Redo read one sector at a time */
				printk(KERN_WARNING "%s: retrying using single "
				       "block read\n", req->rq_disk->disk_name);
				disable_multi = 1;
				continue;
			}
			status = get_card_status(card, req);
		} else if (disable_multi == 1) {
			disable_multi = 0;
		}

		if (brq.cmd.error) {
			printk(KERN_ERR "%s: error %d sending read/write "
			       "command, response %#x, card status %#x\n",
			       req->rq_disk->disk_name, brq.cmd.error,
			       brq.cmd.resp[0], status);
		}

		if (brq.data.error) {
			if (brq.data.error == -ETIMEDOUT && brq.mrq.stop)
				/* 'Stop' response contains card status */
				status = brq.mrq.stop->resp[0];
			printk(KERN_ERR "%s: error %d transferring data,"
			       " sector %u, nr %u, card status %#x\n",
			       req->rq_disk->disk_name, brq.data.error,
			       (unsigned)blk_rq_pos(req),
			       (unsigned)blk_rq_sectors(req), status);
		}

		if (brq.stop.error) {
			printk(KERN_ERR "%s: error %d sending stop command, "
			       "response %#x, card status %#x\n",
			       req->rq_disk->disk_name, brq.stop.error,
			       brq.stop.resp[0], status);
		}

		if (!mmc_host_is_spi(card->host) && rq_data_dir(req) != READ) {
			do {
				int err;

				cmd.opcode = MMC_SEND_STATUS;
				cmd.arg = card->rca << 16;
				cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
				err = mmc_wait_for_cmd(card->host, &cmd, 5);
				if (err) {
					printk(KERN_ERR "%s: error %d requesting status\n",
					       req->rq_disk->disk_name, err);
					goto cmd_err;
				}
				/*
				 * Some cards mishandle the status bits,
				 * so make sure to check both the busy
				 * indication and the card state.
				 */
			} while (!(cmd.resp[0] & R1_READY_FOR_DATA) ||
				(R1_CURRENT_STATE(cmd.resp[0]) == 7));

#if 0
			if (cmd.resp[0] & ~0x00000900)
				printk(KERN_ERR "%s: status = %08x\n",
				       req->rq_disk->disk_name, cmd.resp[0]);
			if (mmc_decode_status(cmd.resp))
				goto cmd_err;
#endif
		}

		if (brq.cmd.error || brq.stop.error || brq.data.error) {
			if (rq_data_dir(req) == READ) {
				/*
				 * After an error, we redo I/O one sector at a
				 * time, so we only reach here after trying to
				 * read a single sector.
				 */
				spin_lock_irq(&md->lock);
				ret = __blk_end_request(req, -EIO, brq.data.blksz);
				spin_unlock_irq(&md->lock);
				continue;
			}
			goto cmd_err;
		}

		/*
		 * A block was successfully transferred.
		 */
		spin_lock_irq(&md->lock);
		ret = __blk_end_request(req, 0, brq.data.bytes_xfered);
		spin_unlock_irq(&md->lock);
	} while (ret);

	mmc_release_host(card->host);

	return 1;

 cmd_err:

#ifdef CONFIG_ERR_RETRY_EMMC_CUST_SH
	if ((rq_data_dir(req) == WRITE)
	&& ((brq.cmd.error == -EILSEQ)
	|| (brq.stop.error == -EILSEQ)
	|| (brq.data.error == -EILSEQ))) {
		if (err_retry < err_retry_max) {
			err_retry++;
			printk(KERN_WARNING "%s: write retry(%d)\n",
				req->rq_disk->disk_name, err_retry);
			goto cmd_err_retry;
		}
	}
#endif

 	/*
 	 * If this is an SD card and we're writing, we can first
 	 * mark the known good sectors as ok.
 	 *
	 * If the card is not SD, we can still ok written sectors
	 * as reported by the controller (which might be less than
	 * the real number of written sectors, but never more).
	 */
	if (mmc_card_sd(card)) {
		u32 blocks;

		blocks = mmc_sd_num_wr_blocks(card);
		if (blocks != (u32)-1) {
			spin_lock_irq(&md->lock);
			ret = __blk_end_request(req, 0, blocks << 9);
			spin_unlock_irq(&md->lock);
		}
	} else {
		spin_lock_irq(&md->lock);
		ret = __blk_end_request(req, 0, brq.data.bytes_xfered);
		spin_unlock_irq(&md->lock);
	}

	mmc_release_host(card->host);

	spin_lock_irq(&md->lock);
	while (ret)
		ret = __blk_end_request(req, -EIO, blk_rq_cur_bytes(req));
	spin_unlock_irq(&md->lock);

	return 0;
}


static inline int mmc_blk_readonly(struct mmc_card *card)
{
	return mmc_card_readonly(card) ||
	       !(card->csd.cmdclass & CCC_BLOCK_WRITE);
}

static struct mmc_blk_data *mmc_blk_alloc(struct mmc_card *card)
{
	struct mmc_blk_data *md;
	int devidx, ret;

	devidx = find_first_zero_bit(dev_use, MMC_NUM_MINORS);
	if (devidx >= MMC_NUM_MINORS)
		return ERR_PTR(-ENOSPC);
	__set_bit(devidx, dev_use);

	md = kzalloc(sizeof(struct mmc_blk_data), GFP_KERNEL);
	if (!md) {
		ret = -ENOMEM;
		goto out;
	}


	/*
	 * Set the read-only status based on the supported commands
	 * and the write protect switch.
	 */
	md->read_only = mmc_blk_readonly(card);

	md->disk = alloc_disk(1 << MMC_SHIFT);
	if (md->disk == NULL) {
		ret = -ENOMEM;
		goto err_kfree;
	}

	spin_lock_init(&md->lock);
	md->usage = 1;

	ret = mmc_init_queue(&md->queue, card, &md->lock);
	if (ret)
		goto err_putdisk;

	md->queue.issue_fn = mmc_blk_issue_rq;
	md->queue.data = md;

	md->disk->major	= MMC_BLOCK_MAJOR;
	md->disk->first_minor = devidx << MMC_SHIFT;
	md->disk->fops = &mmc_bdops;
	md->disk->private_data = md;
	md->disk->queue = md->queue.queue;
	md->disk->driverfs_dev = &card->dev;
	md->disk->flags = GENHD_FL_EXT_DEVT;

	/*
	 * As discussed on lkml, GENHD_FL_REMOVABLE should:
	 *
	 * - be set for removable media with permanent block devices
	 * - be unset for removable block devices with permanent media
	 *
	 * Since MMC block devices clearly fall under the second
	 * case, we do not set GENHD_FL_REMOVABLE.  Userspace
	 * should use the block device creation/destruction hotplug
	 * messages to tell when the card is present.
	 */

	sprintf(md->disk->disk_name, "mmcblk%d", devidx);

	blk_queue_logical_block_size(md->queue.queue, 512);

	if (!mmc_card_sd(card) && mmc_card_blockaddr(card)) {
		/*
		 * The EXT_CSD sector count is in number or 512 byte
		 * sectors.
		 */
		set_capacity(md->disk, card->ext_csd.sectors);
	} else {
		/*
		 * The CSD capacity field is in units of read_blkbits.
		 * set_capacity takes units of 512 bytes.
		 */
		set_capacity(md->disk,
			card->csd.capacity << (card->csd.read_blkbits - 9));
	}
	return md;

 err_putdisk:
	put_disk(md->disk);
 err_kfree:
	kfree(md);
 out:
	return ERR_PTR(ret);
}

static int mmc_blk_probe(struct mmc_card *card)
{
	struct mmc_blk_data *md;
	int err;

	char cap_str[10];

	/*
	 * Check that the card supports the command class(es) we need.
	 */
	if (!(card->csd.cmdclass & CCC_BLOCK_READ))
		return -ENODEV;

	md = mmc_blk_alloc(card);
	if (IS_ERR(md))
		return PTR_ERR(md);

	err = mmc_blk_set_blksize(md, card);
	if (err)
		goto out;

	string_get_size((u64)get_capacity(md->disk) << 9, STRING_UNITS_2,
			cap_str, sizeof(cap_str));
	printk(KERN_INFO "%s: %s %s %s %s\n",
		md->disk->disk_name, mmc_card_id(card), mmc_card_name(card),
		cap_str, md->read_only ? "(ro)" : "");

	mmc_set_drvdata(card, md);
#ifdef CONFIG_MMC_BLOCK_DEFERRED_RESUME
	mmc_set_bus_resume_policy(card->host, 1);
#endif
	add_disk(md->disk);
	return 0;

 out:
	mmc_cleanup_queue(&md->queue);
	mmc_blk_put(md);

	return err;
}

static void mmc_blk_remove(struct mmc_card *card)
{
	struct mmc_blk_data *md = mmc_get_drvdata(card);

	if (md) {
		/* Stop new requests from getting into the queue */
		del_gendisk(md->disk);

		/* Then flush out any already in there */
		mmc_cleanup_queue(&md->queue);

		mmc_blk_put(md);
	}
	mmc_set_drvdata(card, NULL);
#ifdef CONFIG_MMC_BLOCK_DEFERRED_RESUME
	mmc_set_bus_resume_policy(card->host, 0);
#endif
}

#ifdef CONFIG_PM
static int mmc_blk_suspend(struct mmc_card *card, pm_message_t state)
{
	struct mmc_blk_data *md = mmc_get_drvdata(card);

	if (md) {
		mmc_queue_suspend(&md->queue);
	}
	return 0;
}

static int mmc_blk_resume(struct mmc_card *card)
{
	struct mmc_blk_data *md = mmc_get_drvdata(card);

	if (md) {
#ifndef CONFIG_MMC_BLOCK_DEFERRED_RESUME
		mmc_blk_set_blksize(md, card);
#endif
		mmc_queue_resume(&md->queue);
	}
	return 0;
}
#else
#define	mmc_blk_suspend	NULL
#define mmc_blk_resume	NULL
#endif

static struct mmc_driver mmc_driver = {
	.drv		= {
		.name	= "mmcblk",
	},
	.probe		= mmc_blk_probe,
	.remove		= mmc_blk_remove,
	.suspend	= mmc_blk_suspend,
	.resume		= mmc_blk_resume,
};

static int __init mmc_blk_init(void)
{
	int res;

	res = register_blkdev(MMC_BLOCK_MAJOR, "mmc");
	if (res)
		goto out;

	res = mmc_register_driver(&mmc_driver);
	if (res)
		goto out2;

	return 0;
 out2:
	unregister_blkdev(MMC_BLOCK_MAJOR, "mmc");
 out:
	return res;
}

static void __exit mmc_blk_exit(void)
{
	mmc_unregister_driver(&mmc_driver);
	unregister_blkdev(MMC_BLOCK_MAJOR, "mmc");
}

module_init(mmc_blk_init);
module_exit(mmc_blk_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Multimedia Card (MMC) block device driver");

