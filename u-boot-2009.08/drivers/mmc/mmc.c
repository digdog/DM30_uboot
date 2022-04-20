/*
 * (C) Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Terry Lv
 *
 * Copyright 2008, Freescale Semiconductor, Inc
 * Andy Fleming
 *
 * Based vaguely on the Linux code
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

//#define DEBUG

#include <config.h>
#include <common.h>
#include <command.h>
#include <mmc.h>
#include <part.h>
#include <malloc.h>
#include <linux/list.h>
#include <mmc.h>
#include <div64.h>
#include <fsl_esdhc.h>

/*
 * 每次最多传输0xffff(65535)
 * 32 * 512 = 16K = 0x4000
 * 64 * 512 = 32k = 0x8000
 */
#undef MAX_BLK_CNT
#define MAX_BLK_CNT 	64

static struct list_head mmc_devices;
static int cur_dev_num = -1;

int mmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd, struct mmc_data *data)
{
	return mmc->send_cmd(mmc, cmd, data);
}

int mmc_set_blocklen(struct mmc *mmc, int len)
{
	struct mmc_cmd cmd;

	cmd.cmdidx = MMC_CMD_SET_BLOCKLEN;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = len;
	cmd.flags = MMC_CMD_AC;

	return mmc_send_cmd(mmc, &cmd, NULL);
}

struct mmc *find_mmc_device(int dev_num)
{
	struct mmc *m;
	struct list_head *entry;

	list_for_each(entry, &mmc_devices) {
		m = list_entry(entry, struct mmc, link);

		if (m->block_dev.dev == dev_num)
			return m;
	}

	printf("MMC Device %d not found\n", dev_num);

	return NULL;
}

static ulong
mmc_bwrite(int dev_num, ulong start, lbaint_t blkcnt, const void*src)
{
	struct mmc_cmd cmd;
	struct mmc_data data;
	int err;
	int stoperr = 0;
	struct mmc *mmc = find_mmc_device(dev_num);
	int blklen;
	lbaint_t blk_offset = 0, blk_left = blkcnt;

	if (!mmc)
		return -1;

	blklen = mmc->write_bl_len;
#if 0
	err = mmc_set_blocklen(mmc, mmc->write_bl_len);
	if (err) {
		puts("set write bl len failed\n\r");
		return err;
	}
#endif
	do {
		cmd.cmdidx = (blk_left > 1) \
				? MMC_CMD_WRITE_MULTIPLE_BLOCK \
				: MMC_CMD_WRITE_SINGLE_BLOCK;

		cmd.cmdarg = (mmc->high_capacity) \
				? (start + blk_offset) \
				: ((start + blk_offset) * blklen);

		cmd.resp_type = MMC_RSP_R1;
		cmd.flags = MMC_CMD_ADTC;

		data.src = src + blk_offset * blklen;
		data.blocks = (blk_left > MAX_BLK_CNT) \
					  ? MAX_BLK_CNT : blk_left;
		data.blocksize = blklen;
		data.flags = MMC_DATA_WRITE;

		err = mmc_send_cmd(mmc, &cmd, &data);

		if (err) {
			puts("mmc write failed\n\r");
			return err;
		}

		if (blk_left > 1) {
			cmd.cmdidx = MMC_CMD_STOP_TRANSMISSION;
			cmd.cmdarg = 0;
			cmd.resp_type = MMC_RSP_R1b;
			cmd.flags = MMC_CMD_AC;
			stoperr = mmc_send_cmd(mmc, &cmd, NULL);
		}

		if (blk_left > MAX_BLK_CNT) {
			blk_left -= MAX_BLK_CNT;
			blk_offset += MAX_BLK_CNT;
		} else
			break;
	} while (blk_left > 0);

	return blkcnt;
}

int mmc_read_block(struct mmc *mmc, void *dst, uint blocknum)
{
	struct mmc_cmd cmd;
	struct mmc_data data;

	cmd.cmdidx = MMC_CMD_READ_SINGLE_BLOCK;

	if (mmc->high_capacity)
		cmd.cmdarg = blocknum;
	else
		cmd.cmdarg = blocknum * mmc->read_bl_len;

	cmd.resp_type = MMC_RSP_R1;
	cmd.flags = MMC_CMD_ADTC;

	data.dest = dst;
	data.blocks = 1;
	data.blocksize = mmc->read_bl_len;
	data.flags = MMC_DATA_READ;

	return mmc_send_cmd(mmc, &cmd, &data);
}

int mmc_read(struct mmc *mmc, u64 src, uchar *dst, int size)
{
	char *buffer;
	int i;
	int blklen = mmc->read_bl_len;
	int startblock = lldiv(src, mmc->read_bl_len);
	int endblock = lldiv(src + size - 1, mmc->read_bl_len);
	int err = 0;

	/* Make a buffer big enough to hold all the blocks we might read */
	buffer = malloc(blklen);

	if (!buffer) {
		printf("Could not allocate buffer for MMC read!\n");
		return -1;
	}

	/* We always do full block reads from the card */
	err = mmc_set_blocklen(mmc, mmc->read_bl_len);
	if (err)
		goto free_buffer;

	for (i = startblock; i <= endblock; i++) {
		int segment_size;
		int offset;

		err = mmc_read_block(mmc, buffer, i);

		if (err)
			goto free_buffer;

		/*
		 * The first block may not be aligned, so we
		 * copy from the desired point in the block
		 */
		offset = (src & (blklen - 1));
		segment_size = MIN(blklen - offset, size);

		memcpy(dst, buffer + offset, segment_size);

		dst += segment_size;
		src += segment_size;
		size -= segment_size;
	}

free_buffer:
	free(buffer);

	return err;
}

static ulong mmc_bread(int dev_num, ulong start, lbaint_t blkcnt, void *dst)
{
	struct mmc_cmd cmd;
	struct mmc_data data;
	int err;
	int stoperr = 0;
	struct mmc *mmc = find_mmc_device(dev_num);
	int blklen;
	lbaint_t blk_offset = 0, blk_left = blkcnt;

	if (!mmc)
		return -1;
	if (!blk_left)
		return 0;

	blklen = mmc->read_bl_len;

#if 0
	err = mmc_set_blocklen(mmc, blklen);
	if (err) {
		puts("set read bl len failed\n\r");
		return err;
	}
#endif
	do {
		cmd.cmdidx = (blk_left > 1) ? MMC_CMD_READ_MULTIPLE_BLOCK : MMC_CMD_READ_SINGLE_BLOCK;
		cmd.cmdarg = mmc->high_capacity ? (start + blk_offset) : ((start + blk_offset) * blklen);
		cmd.resp_type = MMC_RSP_R1;
		cmd.flags = MMC_CMD_ADTC;

		data.dest = dst + blk_offset * blklen;
		data.blocks = (blk_left > MAX_BLK_CNT) ? MAX_BLK_CNT : blk_left;
		data.blocksize = blklen;
		data.flags = MMC_DATA_READ;

		err = mmc_send_cmd(mmc, &cmd, &data);
		if (err) {
			puts("mmc read failed\n\r");
			return err;
		}

		if (blk_left > 1) {
			cmd.cmdidx = MMC_CMD_STOP_TRANSMISSION;
			cmd.cmdarg = 0;
			cmd.resp_type = MMC_RSP_R1b;
			cmd.flags = MMC_CMD_AC;
			stoperr = mmc_send_cmd(mmc, &cmd, NULL);
		}

		if (blk_left > MAX_BLK_CNT) {
			blk_left -= MAX_BLK_CNT;
			blk_offset += MAX_BLK_CNT;
		} else
			break;
	} while (blk_left > 0);

	return blkcnt;
}

int mmc_go_idle(struct mmc* mmc)
{
	struct mmc_cmd cmd;
	int err;

	udelay(1000);

	cmd.cmdidx = MMC_CMD_GO_IDLE_STATE;
	cmd.cmdarg = 0;
	cmd.resp_type = MMC_RSP_NONE;
	cmd.flags = MMC_CMD_BC;
	err = mmc_send_cmd(mmc, &cmd, NULL);
	if (err) {
		debug("Activating IDLE state failed: %d\n", err);
		return err;
	}

	udelay(1000);

	return 0;
}

static int sd_send_op_cond(struct mmc *mmc)
{
	int timeout = 1000;
	int err;
	struct mmc_cmd cmd;

	do {
		cmd.cmdidx = MMC_CMD_APP_CMD;
		cmd.resp_type = MMC_RSP_R1;
		cmd.cmdarg = 0;
		cmd.flags = MMC_CMD_AC;
		err = mmc_send_cmd(mmc, &cmd, NULL);
		if (err) {
			debug("Preparing SD for operating conditions failed: %d\n", err);
			return err;
		}

		cmd.cmdidx = SD_CMD_APP_SEND_OP_COND;
		cmd.resp_type = MMC_RSP_R3;

		/*
		 * Most cards do not answer if some reserved bits
		 * in the ocr are set. However, Some controller
		 * can set bit 7 (reserved for low voltages), but
		 * how to manage low voltages SD card is not yet
		 * specified.
		 */
		cmd.cmdarg = mmc->voltages | (mmc->version == SD_VERSION_2 ? OCR_HCS : 0);
		cmd.flags = MMC_CMD_BCR;
		err = mmc_send_cmd(mmc, &cmd, NULL);
		if (err) {
			debug("SD operation condition set failed: %d\n", err);
			return err;
		}

		udelay(1000);
	} while ((!(cmd.response[0] & OCR_BUSY)) && timeout--);

	if (timeout <= 0) {
		debug("SD operation condition set timed out\n");
		return -ENODEV;
	}

	if (mmc->version != SD_VERSION_2)
		mmc->version = SD_VERSION_1_0;

	mmc->ocr = cmd.response[0];

	mmc->high_capacity = ((mmc->ocr & OCR_HCS) == OCR_HCS);
	mmc->rca = 0;

	return 0;
}

int mmc_send_op_cond(struct mmc *mmc)
{
	int timeout = 1000;
	struct mmc_cmd cmd;
	int err;

	/* Some cards seem to need this */
	mmc_go_idle(mmc);

	do {
		cmd.cmdidx = MMC_CMD_SEND_OP_COND;
		cmd.resp_type = MMC_RSP_R3;
		cmd.cmdarg = OCR_HCS | mmc->voltages;
		cmd.flags = MMC_CMD_BCR;
		err = mmc_send_cmd(mmc, &cmd, NULL);
		if (err)
			return err;

		udelay(1000);
	} while (!(cmd.response[0] & OCR_BUSY) && timeout--);

	if (timeout <= 0)
		return UNUSABLE_ERR;

	mmc->version = MMC_VERSION_UNKNOWN;
	mmc->ocr = cmd.response[0];

	mmc->high_capacity = ((mmc->ocr & OCR_HCS) == OCR_HCS);
	mmc->rca = 0;

	return 0;
}

int mmc_send_ext_csd(struct mmc *mmc, char *ext_csd)
{
	struct mmc_cmd cmd;
	struct mmc_data data;
	int err;

	/* Get the Card Status Register */
	cmd.cmdidx = MMC_CMD_SEND_EXT_CSD;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = 0;
	cmd.flags = MMC_CMD_BCR;

	data.dest = ext_csd;
	data.blocks = 1;
	data.blocksize = 512;
	data.flags = MMC_DATA_READ;

	err = mmc_send_cmd(mmc, &cmd, &data);

	return err;
}


int mmc_switch(struct mmc *mmc, u8 set, u8 index, u8 value)
{
	struct mmc_cmd cmd;

	cmd.cmdidx = MMC_CMD_SWITCH;
	cmd.resp_type = MMC_RSP_R1b;
	cmd.cmdarg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
		(index << 16) |
		(value << 8);
	cmd.flags = MMC_CMD_ADTC;

	return mmc_send_cmd(mmc, &cmd, NULL);
}

int mmc_change_freq(struct mmc *mmc)
{
	char *ext_csd;
	char cardtype;
	int err;

	mmc->card_caps = 0;

	/* Only version 4 supports high-speed */
	if (mmc->version < MMC_VERSION_4)
		return 0;

	mmc->card_caps |= MMC_MODE_4BIT;

	ext_csd = (char *)malloc(512);

	if (!ext_csd) {
		puts("Could not allocate buffer for MMC ext csd!\n");
		return -1;
	}

	err = mmc_send_ext_csd(mmc, ext_csd);

	if (err)
		goto err_rtn;

/*
	if (ext_csd[EXT_CSD_SEC_CNT] ||
		ext_csd[EXT_CSD_SEC_CNT + 1] ||
		ext_csd[EXT_CSD_SEC_CNT + 2] ||
		ext_csd[EXT_CSD_SEC_CNT + 3])
		mmc->high_capacity = 1;
*/

	cardtype = ext_csd[EXT_CSD_CARD_TYPE] & 0xf;

	err = mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_HS_TIMING, 1);

	if (err)
		goto err_rtn;

	/* Now check to see that it worked */
	err = mmc_send_ext_csd(mmc, ext_csd);

	if (err)
		goto err_rtn;

	/* No high-speed support */
	if (!ext_csd[EXT_CSD_HS_TIMING])
		goto no_err_rtn;

	/* High Speed is set, there are two types: 52MHz and 26MHz */
	if (cardtype & MMC_HS_52MHZ)
		mmc->card_caps |= MMC_MODE_HS_52MHz | MMC_MODE_HS;
	else
		mmc->card_caps |= MMC_MODE_HS;

no_err_rtn:
	free(ext_csd);
	return 0;

err_rtn:
	free(ext_csd);
	return err;
}

int sd_switch(struct mmc *mmc, int mode, int group, u8 value, u8 *resp)
{
	struct mmc_cmd cmd;
	struct mmc_data data;

	/* Switch the frequency */
	cmd.cmdidx = SD_CMD_SWITCH_FUNC;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = (mode << 31) | 0xffffff;
	cmd.cmdarg &= ~(0xf << (group << 2));
	cmd.cmdarg |= value << (group << 2);
	cmd.flags = MMC_CMD_ADTC;

	data.dest = (char *)resp;
	data.blocksize = 64;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;

	return mmc_send_cmd(mmc, &cmd, &data);
}

/**
 * sd_change_freq - 切换SD卡的传输模式
 * 
 * 发送ACMD51来获得SD卡的SCR寄存器，根据其中的SD_SPEC位来判断其版本号。
 * SD-Spec1.1及以后的版本才支持High-Speed
 *
 * 如果主控制器不支持High-Speed，则不用往下检查了
 *
 * 发送CMD6检查SD卡是否支持High-Speed
 *
 * 若SD卡支持High-Speed, 则切换SD卡为High-Speed模式。切换之后，注意要将SCK时钟
 * 设置为50M
 */
static int sd_change_freq(struct mmc *mmc)
{
	int err;
	struct mmc_cmd cmd;
	uint scr[2] = {0, 0};
	uint switch_status[16];
	struct mmc_data data;
	int timeout;

	debug("Changing transfer frequency\n");
	mmc->card_caps = 0;

	/* Read the SCR to find out if this card supports higher speeds */
	cmd.cmdidx = MMC_CMD_APP_CMD;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = mmc->rca << 16;
	cmd.flags = MMC_CMD_AC;

	err = mmc_send_cmd(mmc, &cmd, NULL);
	if (err) {
		debug("Query SD card capabilities failed: %d\n", err);
		return err;
	}

	cmd.cmdidx = SD_CMD_APP_SEND_SCR;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = 0;
	cmd.flags = MMC_CMD_ADTC;

	timeout = 3;

retry_scr:
	debug("Trying to read the SCR (try %d of %d)\n", 4 - timeout, 3);
	data.dest = (char *)scr;
	data.blocksize = 8;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;

	err = mmc_send_cmd(mmc, &cmd, &data);
	if (err) {
		debug(" Catch error (%d)", err);
		if (timeout--) {
			debug("-- retrying\n");
			goto retry_scr;
		}
		debug("-- giving up\n");
		return err;
	}

	mmc->scr[0] = __be32_to_cpu(scr[0]);
	mmc->scr[1] = __be32_to_cpu(scr[1]);
	debug("%s: scr[0] %x, scr[1] %x\n", __func__, mmc->scr[0], mmc->scr[1]);

	switch ((mmc->scr[0] >> 24) & 0xf) {
		case 0:
			mmc->version = SD_VERSION_1_0;
			break;
		case 1:
			mmc->version = SD_VERSION_1_10;
			break;
		case 2:
			mmc->version = SD_VERSION_2;
			break;
		default:
			mmc->version = SD_VERSION_1_0;
			break;
	}

	/* Version 1.0 doesn't support switching */
	if (mmc->version == SD_VERSION_1_0) {
		debug("SD VERSION 1.0 do not support High-Speed!\n");
		return 0;
	}

	timeout = 4;
	while (timeout--) {
		err = sd_switch(mmc, SD_SWITCH_CHECK, 0, 1, (u8 *)&switch_status);
		if (err) {
			debug("Checking SD transfer switch frequency feature failed: %d\n", err);
			return err;
		}

		/* The high-speed function is busy.  Try again */
		if (!(__be32_to_cpu(switch_status[7]) & SD_HIGHSPEED_BUSY))
			break;
	}

	if (mmc->scr[0] & SD_DATA_4BIT)
		mmc->card_caps |= MMC_MODE_4BIT;

	/* If high-speed isn't supported, we return */
	if (!(__be32_to_cpu(switch_status[3]) & SD_HIGHSPEED_SUPPORTED)) {
		debug("SD Card don't support High-Speed mode\n");
		return 0;
	}

	err = sd_switch(mmc, SD_SWITCH_SWITCH, 0, 1, (u8 *)&switch_status);
	if (err) {
		pr_err("Switching SD transfer frequency failed: %d\n", err);
		return err;
	}

	if ((__be32_to_cpu(switch_status[4]) & 0x0f000000) == 0x01000000)
		mmc->card_caps |= MMC_MODE_HS;

	return 0;
}

/* frequency bases */
/* divided by 10 to be nice to platforms without floating point */
static const int fbase[] = {
	[0] = 10000,		/* 100 kbit/s */
	[1] = 100000,		/* 1 Mbit/s */
	[2] = 1000000,		/* 10 Mbit/s */
	[3] = 10000000,		/* 100 Mbit/s */
	/* [4]...[7] are reserved */
};

/* Multiplier values for TRAN_SPEED.  Multiplied by 10 to be nice
 * to platforms without floating point.
 */
static const int multipliers[] = {
	0,	/* reserved */
	10,	/* 1.0 ns */
	12,	/* 1.2 ns */
	13,
	15,
	20,
	25,
	30,
	35,
	40,
	45,
	50,
	55,
	60,
	70,	/* 7.0 ns */
	80,	/* 8.0 ns */
};

void mmc_set_ios(struct mmc *mmc)
{
	mmc->set_ios(mmc);
}

void mmc_set_clock(struct mmc *mmc, uint clock)
{
	if (clock > mmc->f_max)
		clock = mmc->f_max;

	if (clock < mmc->f_min)
		clock = mmc->f_min;

	mmc->clock = clock;

	mmc_set_ios(mmc);
}

void mmc_set_bus_width(struct mmc *mmc, uint width)
{
	mmc->bus_width = width;

	mmc_set_ios(mmc);
}

#ifdef CONFIG_BOOT_PARTITION_ACCESS
/* Return 0/1/2 for partition id before switch; Return -1 if fail to switch */
int mmc_switch_partition(struct mmc *mmc, uint part)
{
	char *ext_csd;
	int err;
	uint old_part, new_part;
	char boot_config;

	/* partition must be -
		0 - user area
		1 - boot partition 1
		2 - boot partition 2
	*/
	if (part > 2) {
		printf("\nWrong partition id - 0 (user area), 1 (boot1), 2 (boot2)\n");
		return 1;
	}

	/* Before calling this func, "mmc" struct must have been initialized */
	if (mmc->version < MMC_VERSION_4) {
		puts("Error: invalid mmc version! mmc version is below version 4!");
		return -1;
	}

	if (mmc->boot_size_mult <= 0) {
		/* it's a normal SD/MMC but user request to boot partition */
		printf("\nError: This is a normal SD/MMC card but you request to access boot partition\n");
		return -1;
	}

	/* part must be 0 (user area), 1 (boot partition1) or 2 (boot partition2) */
	if (part > 2) {
		puts("Error: partition id must be 0(user area), 1(boot partition1) or 2(boot partition2)\n");
		return -1;
	}

	if (IS_SD(mmc)) {
		/* eSD card hadn't been supported. Return directly without warning */
		return -1;
	}

	ext_csd = (char *)malloc(512);
	if (!ext_csd) {
		puts("Error: Could not allocate buffer for MMC ext csd!\n");
		return -1;
	}

	err = mmc_send_ext_csd(mmc, ext_csd);
	if (err) {
		puts("Warning: fail to get ext csd for MMC!\n");
		goto err_rtn;
    }

	old_part = ext_csd[EXT_CSD_BOOT_CONFIG] & EXT_CSD_BOOT_PARTITION_ACCESS_MASK;

	/* Send SWITCH command to change partition for access */
	boot_config = (ext_csd[EXT_CSD_BOOT_CONFIG] & ~EXT_CSD_BOOT_PARTITION_ACCESS_MASK) | (char)part;

	err = mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_BOOT_CONFIG, boot_config);
	if (err) {
		puts("Error: fail to send SWITCH command to card to swich partition for access!\n");
		goto err_rtn;
	}

	/* Now check whether it works */
	err = mmc_send_ext_csd(mmc, ext_csd);
	if (err) {
		puts("Warning: fail to get ext csd for MMC!\n");
		goto err_rtn;
	}

	new_part = ext_csd[EXT_CSD_BOOT_CONFIG] & EXT_CSD_BOOT_PARTITION_ACCESS_MASK;
	if ((char)part != new_part) {
		printf("Warning: after SWITCH, current part id %d is not same as requested partition %d!\n",
			new_part, part);
		goto err_rtn;
	}

	/* Seems everything is ok, return the partition id before switch */
	free(ext_csd);
	return old_part;

err_rtn:
	free(ext_csd);
	return -1;
}

int sd_switch_partition(struct mmc *mmc, uint part)
{
	struct mmc_cmd cmd;
	int err;

	if (part > 1) {
		printf("\nWrong partition id - 0 (user area), 1 (boot1)\n");
		return 1;
	}

	cmd.cmdidx = SD_CMD_SELECT_PARTITION;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = part << 24;
	cmd.flags = 0;

	err = mmc_send_cmd(mmc, &cmd, NULL);

	if (err) {
		printf("Failed to switch to partition %d\nPartition not exists or command execute fail!\n");
		return -1;
	}

	return 0;
}

int mmc_get_cur_boot_partition(struct mmc *mmc)
{
	char *ext_csd;
	int err;

	ext_csd = (char *)malloc(512);

	if (!ext_csd) {
		puts("Error! Could not allocate buffer for MMC ext csd!\n");
		return -1;
	}

	err = mmc_send_ext_csd(mmc, ext_csd);

	if (err) {
		mmc->boot_config = 0;
		mmc->boot_size_mult = 0;
		/* continue since it's not a fatal error */
	} else {
		mmc->boot_config = ext_csd[EXT_CSD_BOOT_CONFIG];
		mmc->boot_size_mult = ext_csd[EXT_CSD_BOOT_INFO];
	}

	free(ext_csd);

	return err;
}

#endif

int mmc_startup(struct mmc *mmc)
{
	int err;
	uint mult, freq;
	u64 cmult, csize;
	struct mmc_cmd cmd;

	debug("Put the Card in Identify Mode\n");

	/* Put the Card in Identify Mode */
	cmd.cmdidx = MMC_CMD_ALL_SEND_CID;
	cmd.resp_type = MMC_RSP_R2;
	cmd.cmdarg = 0;
	cmd.flags = MMC_CMD_BCR;
	err = mmc_send_cmd(mmc, &cmd, NULL);
	if (err)
		return err;

	memcpy(mmc->cid, cmd.response, 16);

	debug("Card's identification data is: %08X-%08X-%08X-%08X\n",
		mmc->cid[0], mmc->cid[1], mmc->cid[2], mmc->cid[3]);

	debug("Get/Set relative address\n");

	/*
	 * For MMC cards, set the Relative Address.
	 * For SD cards, get the Relatvie Address.
	 * This also puts the cards into Standby State
	 */
	cmd.cmdidx = SD_CMD_SEND_RELATIVE_ADDR;
	cmd.cmdarg = mmc->rca << 16;
	cmd.resp_type = MMC_RSP_R6;
	cmd.flags = MMC_CMD_AC;
	err = mmc_send_cmd(mmc, &cmd, NULL);
	if (err)
		return err;

	if (IS_SD(mmc))
		mmc->rca = (cmd.response[0] >> 16) & 0xffff;

	debug("Get card's specific data\n");

	/* Get the Card-Specific Data */
	cmd.cmdidx = MMC_CMD_SEND_CSD;
	cmd.resp_type = MMC_RSP_R2;
	cmd.cmdarg = mmc->rca << 16;
	cmd.flags = MMC_CMD_AC;
	err = mmc_send_cmd(mmc, &cmd, NULL);
	if (err)
		return err;

	memcpy(mmc->csd, cmd.response, 16);

	debug("Card's specific data is: %08X-%08X-%08X-%08X\n",
		mmc->csd[0], mmc->csd[1], mmc->csd[2], mmc->csd[3]);

	if (mmc->version == MMC_VERSION_UNKNOWN) {
		int version = (cmd.response[0] >> 26) & 0xf;

		switch (version) {
			case 0:
				mmc->version = MMC_VERSION_1_2;
				break;
			case 1:
				mmc->version = MMC_VERSION_1_4;
				break;
			case 2:
				mmc->version = MMC_VERSION_2_2;
				break;
			case 3:
				mmc->version = MMC_VERSION_3;
				break;
			case 4:
				mmc->version = MMC_VERSION_4;
				break;
			default:
				mmc->version = MMC_VERSION_1_2;
				break;
		}
	}

	/* divide frequency by 10, since the mults are 10x bigger */
	freq = fbase[(mmc->csd[0] & 0x7)];
	mult = multipliers[((mmc->csd[0] >> 3) & 0xf)];
	if ((freq == 0) || (mult == 0)) {
		pr_warning("Unsupported 'TRAN_SPEED' unit/time value."
				" Can't calculate card's max. transfer speed\n");
	}
	mmc->tran_speed = freq * mult;
	debug("Transfer speed: %u\n", mmc->tran_speed);

	/* block length */
	mmc->read_bl_len = 1 << ((mmc->csd[1] >> 16) & 0xf);
	if (IS_SD(mmc))
		mmc->write_bl_len = mmc->read_bl_len;
	else
		mmc->write_bl_len = 1 << ((mmc->csd[3] >> 22) & 0xf);

	debug("Max. block length are: Write=%u, Read=%u Bytes\n",
		mmc->read_bl_len, mmc->write_bl_len);

	/* extra capacitiy */
	if (mmc->high_capacity) {
		csize = (mmc->csd[1] & 0x3f) << 16 | (mmc->csd[2] & 0xffff0000) >> 16;
		cmult = 8;
	} else {
		csize = (mmc->csd[1] & 0x3ff) << 2 | (mmc->csd[2] & 0xc0000000) >> 30;
		cmult = (mmc->csd[2] & 0x00038000) >> 15;
	}
	mmc->capacity = (csize + 1) << (cmult + 2);
	mmc->capacity *= mmc->read_bl_len;
	debug("Capacity: %u MiB\n", (unsigned)mmc->capacity >> 20);

	if (mmc->read_bl_len > 512) {
		mmc->read_bl_len = 512;
		debug("Limiting max. read block size down to %u\n",
				mmc->read_bl_len);
	}

	if (mmc->write_bl_len > 512) {
		mmc->write_bl_len = 512;
		debug("Limiting max. write block size down to %u\n",
				mmc->read_bl_len);
	}

	debug("Read block length: %u, Write block length: %u\n",
		mmc->read_bl_len, mmc->write_bl_len);

	debug("Select the card, and put it into Transfer Mode\n");

	/* Select the card, and put it into Transfer Mode */
	cmd.cmdidx = MMC_CMD_SELECT_CARD;
	cmd.resp_type = MMC_RSP_R1b;
	cmd.cmdarg = mmc->rca << 16;
	cmd.flags = MMC_CMD_AC;
	err = mmc_send_cmd(mmc, &cmd, NULL);
	if (err) {
		debug("Putting in transfer mode failed: %d\n", err);
		return err;
	}

	if (IS_SD(mmc))
		err = sd_change_freq(mmc);
	else
		err = mmc_change_freq(mmc);
	if (err)
		return err;

	/* Restrict card's capabilities by what the host can do */
	mmc->card_caps &= mmc->host_caps;

	if (IS_SD(mmc)) {
		if (mmc->card_caps & MMC_MODE_4BIT) {
			cmd.cmdidx = MMC_CMD_APP_CMD;
			cmd.resp_type = MMC_RSP_R1;
			cmd.cmdarg = mmc->rca << 16;
			cmd.flags = MMC_CMD_AC;

			err = mmc_send_cmd(mmc, &cmd, NULL);
			if (err)
				return err;

			cmd.cmdidx = SD_CMD_APP_SET_BUS_WIDTH;
			cmd.resp_type = MMC_RSP_R1;
			cmd.cmdarg = 2;
			cmd.flags = MMC_CMD_AC;
			err = mmc_send_cmd(mmc, &cmd, NULL);
			if (err)
				return err;

			mmc_set_bus_width(mmc, 4);
		}
		if (mmc->card_caps & MMC_MODE_HS)
			mmc_set_clock(mmc, 50000000);
		else
			mmc_set_clock(mmc, 25000000);
	} else {
		if (mmc->card_caps & MMC_MODE_4BIT) {
			/* Set the card to use 4 bit*/
			err = mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL,
					EXT_CSD_BUS_WIDTH,
					EXT_CSD_BUS_WIDTH_4);

			if (err)
				return err;

			mmc_set_bus_width(mmc, 4);
		} else if (mmc->card_caps & MMC_MODE_8BIT) {
			/* Set the card to use 8 bit*/
			err = mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL,
					EXT_CSD_BUS_WIDTH,
					EXT_CSD_BUS_WIDTH_8);

			if (err)
				return err;

			mmc_set_bus_width(mmc, 8);
		}

		if (mmc->card_caps & MMC_MODE_HS) {
			if (mmc->card_caps & MMC_MODE_HS_52MHz)
				mmc_set_clock(mmc, 52000000);
			else
				mmc_set_clock(mmc, 26000000);
		} else
			mmc_set_clock(mmc, 20000000);

#ifdef CONFIG_BOOT_PARTITION_ACCESS
		mmc_get_cur_boot_partition(mmc);
#endif
	}

	/* fill in device description */
	mmc->block_dev.lun = 0;
	mmc->block_dev.type = 0;
	mmc->block_dev.blksz = mmc->read_bl_len;
	mmc->block_dev.lba = lldiv(mmc->capacity, mmc->read_bl_len);

	sprintf(mmc->block_dev.vendor, "Man %06x Snr %08x", mmc->cid[0] >> 8,
			(mmc->cid[2] << 8) | (mmc->cid[3] >> 24));

	sprintf(mmc->block_dev.product, "%c%c%c%c%c", mmc->cid[0] & 0xff,
			(mmc->cid[1] >> 24), (mmc->cid[1] >> 16) & 0xff,
			(mmc->cid[1] >> 8) & 0xff, mmc->cid[1] & 0xff);

	sprintf(mmc->block_dev.revision, "%d.%d", mmc->cid[2] >> 28, (mmc->cid[2] >> 24) & 0xf);

	err = mmc_set_blocklen(mmc, mmc->read_bl_len);

	init_part(&mmc->block_dev);
	return 0;
}

int mmc_send_if_cond(struct mmc *mmc)
{
	struct mmc_cmd cmd;
	int err;

	cmd.cmdidx = SD_CMD_SEND_IF_COND;
	/* We set the bit if the host supports voltages between 2.7 and 3.6 V */
	cmd.cmdarg = ((mmc->voltages & 0xff8000) != 0) << 8 | 0xaa;
	cmd.resp_type = MMC_RSP_R7;
	cmd.flags = MMC_CMD_BCR;
	err = mmc_send_cmd(mmc, &cmd, NULL);
	if (err) {
		debug("Query interface conditions failed: %d\n", err);
		return err;
	}

	if ((cmd.response[0] & 0xff) != 0xaa) {
		debug("Card cannot work with hosts supply voltages\n");
		return UNUSABLE_ERR;
	} else {
		debug("SD Card Rev. 2.00 or later detected\n");
		mmc->version = SD_VERSION_2;
	}

	return 0;
}

int mmc_register(struct mmc *mmc)
{
	/* Setup the universal parts of the block interface just once */
	mmc->block_dev.if_type = IF_TYPE_MMC;
	mmc->block_dev.dev = cur_dev_num++;
	mmc->block_dev.removable = 1;
	mmc->block_dev.block_read = mmc_bread;
	mmc->block_dev.block_write = mmc_bwrite;
#if defined(CONFIG_DOS_PARTITION)
	mmc->block_dev.part_type = PART_TYPE_DOS;
	mmc->block_dev.type = DEV_TYPE_HARDDISK;
#elif defined(CONFIG_MAC_PARTITION)
	mmc->block_dev.part_type = PART_TYPE_MAC;
	mmc->block_dev.type = DEV_TYPE_HARDDISK;
#elif defined(CONFIG_ISO_PARTITION)
	mmc->block_dev.part_type = PART_TYPE_ISO;
	mmc->block_dev.type = DEV_TYPE_HARDDISK;
#elif defined(CONFIG_AMIGA_PARTITION)
	mmc->block_dev.part_type = PART_TYPE_AMIGA;
	mmc->block_dev.type = DEV_TYPE_HARDDISK;
#elif defined(CONFIG_EFI_PARTITION)
	mmc->block_dev.part_type = PART_TYPE_EFI;
	mmc->block_dev.type = DEV_TYPE_HARDDISK;
#endif

	INIT_LIST_HEAD (&mmc->link);

	list_add_tail (&mmc->link, &mmc_devices);

	return 0;
}

block_dev_desc_t *mmc_get_dev(int dev)
{
	struct mmc *mmc = find_mmc_device(dev);

	return mmc ? &mmc->block_dev : NULL;
}

int mmc_init(struct mmc *mmc)
{
	int err;

	/* start with a host interface reset */
	err = mmc->init(mmc);
	if (err) {
		pr_err("Cannot reset the SD/MMC interface\n");
		return err;
	}

	/* the default bus width after power on or GO_IDLE(cmd0) is 1-bit bus
	 * width*/
	mmc_set_bus_width(mmc, 1);

	/* set the lowest available clock */
	mmc_set_clock(mmc, 1);

	/* Reset the Card */
	err = mmc_go_idle(mmc);
	if (err) {
		pr_warning("Cannot reset the SD/MMC card\n");
		return err;
	}

	/* Test for SD version 2 */
	err = mmc_send_if_cond(mmc);

	/* Now try to get the SD card's operating condition */
	err = sd_send_op_cond(mmc);
	if (err == TIMEOUT) {
		/* If the command timed out, we check for an MMC card */
		debug("Card seems to be a MultiMediaCard\n");
		err = mmc_send_op_cond(mmc);
	}

	if (err) {
		pr_err("Card did not respond to voltage select!\n");
		return err;
	}

	err = mmc_startup(mmc);
	if (err)
		pr_err("Card's startup fails with %d\n", err);

	return err;
}

/*
 * CPU and board-specific MMC initializations.  Aliased function
 * signals caller to move on
 */
static int __def_mmc_init(bd_t *bis)
{
	return -1;
}

extern int cpu_mmc_init(bd_t *bis);
//int cpu_mmc_init(bd_t *bis) __attribute__((weak, alias("__def_mmc_init")));
int board_mmc_init(bd_t *bis) __attribute__((weak, alias("__def_mmc_init")));

void print_mmc_devices(char separator)
{
	struct mmc *m;
	struct list_head *entry;

	list_for_each(entry, &mmc_devices) {
		m = list_entry(entry, struct mmc, link);

		printf("%s: %d", m->name, m->block_dev.dev);

		if (entry->next != &mmc_devices)
			printf("%c ", separator);
	}

	printf("\n");
}

int mmc_initialize(bd_t *bis)
{
	INIT_LIST_HEAD (&mmc_devices);
	cur_dev_num = 0;

#ifdef CONFIG_MMC_MX23_DMA
	extern int mxs_mmc_probe(void);
	mxs_mmc_probe();
#else
	if (board_mmc_init(bis) < 0)
		cpu_mmc_init(bis);
#endif

	print_mmc_devices(',');

	return 0;
}
