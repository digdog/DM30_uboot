/*
 * (C) Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Terry Lv
 *
 * Copyright 2007, Freescale Semiconductor, Inc
 * Andy Fleming
 *
 * Based vaguely on the pxa mmc code:
 * (C) Copyright 2003
 * Kyle Harris, Nexus Technologies, Inc. kharris@nexus-tech.net
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

/* define DEBUG */
#define DEBUG

#include <common.h>
#include <malloc.h>
#include <mmc.h>
#include <asm/arch/mx23.h>
#include <asm/arch/regs-ssp.h>
#include <asm/arch/regs-clkctrl.h>
#include <asm/io.h>
#include <asm/bitops.h>
#include <asm/errno.h>

#undef REG_RD
#undef REG_WR
#undef REG_SET
#undef REG_CLR
#undef REG_TOG

#undef REG_RD_ADDR
#undef REG_WR_ADDR
#undef REG_SET_ADDR
#undef REG_CLR_ADDR
#undef REG_TOG_ADDR

#define REG_RD(base, reg) (*(volatile unsigned int *)((base) + (reg)))
#define REG_WR(base, reg, value) ((*(volatile unsigned int *)((base) + (reg))) = (value))
#define REG_SET(base, reg, value) ((*(volatile unsigned int *)((base) + (reg ## _SET))) = (value))
#define REG_CLR(base, reg, value) ((*(volatile unsigned int *)((base) + (reg ## _CLR))) = (value))
#define REG_TOG(base, reg, value) ((*(volatile unsigned int *)((base) + (reg ## _TOG))) = (value))

#define REG_RD_ADDR(addr) (*(volatile unsigned int *)((addr)))
#define REG_WR_ADDR(addr, value) ((*(volatile unsigned int *)((addr))) = (value))
#define REG_SET_ADDR(addr, value) ((*(volatile unsigned int *)((addr) + 0x4)) = (value))
#define REG_CLR_ADDR(addr, value) ((*(volatile unsigned int *)((addr) + 0x8)) = (value))
#define REG_TOG_ADDR(addr, value) ((*(volatile unsigned int *)((addr) + 0xc)) = (value))

# define SSP_STATUS_ERROR \
	(BM_SSP_STATUS_FIFO_OVRFLW  | \
	 BM_SSP_STATUS_FIFO_UNDRFLW | \
	 BM_SSP_STATUS_RESP_CRC_ERR | \
	 BM_SSP_STATUS_RESP_ERR     | \
	 BM_SSP_STATUS_RESP_TIMEOUT | \
	 BM_SSP_STATUS_DATA_CRC_ERR | \
	 BM_SSP_STATUS_TIMEOUT)

# define SSP_CMD0_CMD(x) ((x) & 0xff)
# define SSP_TIMING_TIMEOUT_MASK (0xffff0000)
# define SSP_TIMING_TIMEOUT(x) ((x) << 16)

# define SSP_CTRL0_XFER_COUNT(x) ((x) & 0xffff)
# define SSP_CMD0_BLOCK_SIZE(x) (((x) & 0xf) << 16)
# define SSP_CMD0_BLOCK_COUNT(x) (((x) & 0xff) << 8)
# define SSP_CTRL0_BUS_WIDTH(x) (((x) & 0x3) << 22)

extern void ssp_mmc_board_init(void);
extern u32 ssp_mmc_is_wp(void);
extern u32 imx_get_sspclk(u32 index);

static inline void mdelay(unsigned long msec)
{
	unsigned long i;
	for (i = 0; i < msec; i++)
		udelay(1000);
}

static inline void sdelay(unsigned long sec)
{
	unsigned long i;
	for (i = 0; i < sec; i++)
		mdelay(1000);
}

static inline int __ssp_bus_width(int bus_width)
{
	switch (bus_width) {
	case 8:
		return 2;
	case 4:
		return 1;
	default:
		return 0;
	}
}

static int get_cards_response(struct mmc_cmd *cmd)
{
	switch (cmd->resp_type) {
	case MMC_RSP_NONE:
		return 0;

	case MMC_RSP_R1:
	case MMC_RSP_R1b:
	case MMC_RSP_R3:
		cmd->response[0] = readl(REGS_SSP1_BASE + HW_SSP_SDRESP0);
		return 1;

	case MMC_RSP_R2:
		cmd->response[3] = readl(REGS_SSP1_BASE + HW_SSP_SDRESP0);
		cmd->response[2] = readl(REGS_SSP1_BASE + HW_SSP_SDRESP1);
		cmd->response[1] = readl(REGS_SSP1_BASE + HW_SSP_SDRESP2);
		cmd->response[0] = readl(REGS_SSP1_BASE + HW_SSP_SDRESP3);
		return 4;
	}

	return -EINVAL;
}

static int get_cmd_error(unsigned status)
{
	if (status & SSP_STATUS_ERROR)
		pr_err("Status Reg reports %08X\n", status);

	if (status & BM_SSP_STATUS_TIMEOUT) {
		pr_err("CMD timeout\n");
		return -ETIMEDOUT;
	} else if (status & BM_SSP_STATUS_RESP_TIMEOUT) {
		pr_err("RESP timeout\n");
		return -ETIMEDOUT;
	} else if (status & BM_SSP_STATUS_RESP_CRC_ERR) {
		pr_err("CMD crc error\n");
		return -EILSEQ;
	} else if (status & BM_SSP_STATUS_RESP_ERR) {
		pr_err("RESP error\n");
		return -EIO;
	}

	return 0;
}

static uint32_t prepare_transfer_setup(unsigned cmd_flags, unsigned data_flags)
{
	uint32_t reg = 0;

	if (cmd_flags & MMC_RSP_PRESENT)
		reg |= BM_SSP_CTRL0_GET_RESP;
	if ((cmd_flags & MMC_RSP_CRC) == 0)
		reg |= BM_SSP_CTRL0_IGNORE_CRC;
	if (cmd_flags & MMC_RSP_136)
		reg |= BM_SSP_CTRL0_LONG_RESP;
	if (cmd_flags & MMC_RSP_BUSY)
		reg |= BM_SSP_CTRL0_WAIT_FOR_IRQ;	/* FIXME correct? */
#if 0
	if (cmd_flags & MMC_RSP_OPCODE)
		/* TODO */
#endif
	if (data_flags & MMC_DATA_READ)
		reg |= BM_SSP_CTRL0_READ;

	return reg;
}

static void stm_setup_timout(unsigned to)
{
	uint32_t reg;

	reg = REG_RD(REGS_SSP1_BASE, HW_SSP_TIMING) & ~SSP_TIMING_TIMEOUT_MASK;
	reg |= SSP_TIMING_TIMEOUT(to);
	REG_WR(REGS_SSP1_BASE, HW_SSP_TIMING, reg);
}

/**
 * Read data from the MCI card
 * @param hw_dev Host interface device instance
 * @param buffer To write data into
 * @param length Count of bytes to read (must be multiples of 4)
 * @return 0 on success, negative values else
 *
 * @note This routine uses PIO to read in the data bytes from the FIFO. This
 * may fail whith high clock speeds. If you receive -EIO errors you can try
 * again with reduced clock speeds.
 */
static int read_data(void *buffer, unsigned length)
{
	uint32_t *p = buffer;

	if (length & 0x3) {
		pr_err("Cannot read data sizes not multiple of 4 (request for %u detected)\n",
				length);
		return -EINVAL;
	}

	while ((length != 0) && ((readl(REGS_SSP1_BASE + HW_SSP_STATUS) & SSP_STATUS_ERROR) == 0)) {
		/* TODO sort out FIFO overflows and emit -EOI for this case */
		if ((readl(REGS_SSP1_BASE + HW_SSP_STATUS) & BM_SSP_STATUS_FIFO_EMPTY) == 0) {
			*p = readl(REGS_SSP1_BASE + HW_SSP_DATA);
			p++;
			length -= 4;
		}
	}

	return (length == 0) ? 0 : -EINVAL;
}


/**
 * Write data into the MCI card
 * @param hw_dev Host interface device instance
 * @param buffer To read the data from
 * @param length Count of bytes to write (must be multiples of 4)
 * @return 0 on success, negative values else
 *
 * @note This routine uses PIO to write the data bytes into the FIFO. This
 * may fail with high clock speeds. If you receive -EIO errors you can try
 * again with reduced clock speeds.
 */
static int write_data(const void *buffer, unsigned length)
{
	const uint32_t *p = buffer;

	if (length & 0x3) {
		pr_err("Cannot write data sizes not multiple of 4 (request for %u detected)\n",
				length);
		return -EINVAL;
	}

	while ((length != 0) &&
		((readl(REGS_SSP1_BASE + HW_SSP_STATUS) & SSP_STATUS_ERROR) == 0)) {
		/* TODO sort out FIFO overflows and emit -EOI for this case */
		if ((readl(REGS_SSP1_BASE + HW_SSP_STATUS) & BM_SSP_STATUS_FIFO_FULL) == 0) {
			writel(*p, REGS_SSP1_BASE + HW_SSP_DATA);
			p++;
			length -= 4;
		}
	}

	return (length == 0) ? 0 : -EINVAL;
}

/**
 * Start the transaction with or without data
 * @param hw_dev Host interface device instance
 * @param data Data transfer description (might be NULL)
 * @return 0 on success
 */
static int transfer_data(struct mmc_data *data)
{
	unsigned length;

	if (data != NULL) {
		length = data->blocks * data->blocksize;
#if 0
		/*
		 * For the records: When writing data with high clock speeds it
		 * could be a good idea to fill the FIFO prior starting the
		 * transaction.
		 * But last time I tried it, it failed badly. Don't know why yet
		 */
		if (data->flags & MMC_DATA_WRITE) {
			err = write_data(host, data->src, 16);
			data->src += 16;
			length -= 16;
		}
#endif
	}

	/*
	 * Everything is ready for the transaction now:
	 * - transfer configuration
	 * - command and its parameters
	 *
	 * Start the transaction right now
	 */
	writel(BM_SSP_CTRL0_RUN, REGS_SSP1_BASE + HW_SSP_CTRL0 + 4);

	if (data != NULL) {
		if (data->flags & MMC_DATA_READ)
			return read_data(data->dest, length);
		else if (ssp_mmc_is_wp()) {
			pr_err("MMC: Can not write a locked card!\n");
			return -EIO;
		} else 
			return write_data(data->src, length);
	}

	return 0;
}

static int stm_mci_adtc(struct mmc *mmc, struct mmc_cmd *cmd, struct mmc_data *data)
{
	uint32_t xfer_cnt, log2blocksize, block_cnt;
	int err;

	/* Note: 'data' can be NULL! */
	if (data != NULL) {
		xfer_cnt = data->blocks * data->blocksize;
		block_cnt = data->blocks - 1;	/* can be 0 */
		//log2blocksize = find_first_bit((const unsigned long*)&data->blocksize, 32);
		log2blocksize = ffs(data->blocksize) - 1;
	} else
		xfer_cnt = log2blocksize = block_cnt = 0;

	debug("%s: xfer_cnt = %d(0x%x), block_cnt = %d(0x%x), log2blocksize = %d(0x%x)\n", 
		__func__, xfer_cnt, xfer_cnt, block_cnt, block_cnt, log2blocksize, log2blocksize);

	/* setup command and transfer parameters */
	writel(prepare_transfer_setup(cmd->resp_type, data != NULL ? data->flags : 0) |
		SSP_CTRL0_BUS_WIDTH(__ssp_bus_width(mmc->bus_width)) |
		(xfer_cnt != 0 ? BM_SSP_CTRL0_DATA_XFER : 0) | /* command plus data */
		BM_SSP_CTRL0_ENABLE |
		SSP_CTRL0_XFER_COUNT(xfer_cnt), /* byte count to be transfered */
		REGS_SSP1_BASE + HW_SSP_CTRL0);

	/* prepare the command and the transfered data count */
	writel(SSP_CMD0_CMD(cmd->cmdidx) |
		SSP_CMD0_BLOCK_SIZE(log2blocksize) |
		SSP_CMD0_BLOCK_COUNT(block_cnt) |
		(cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION ? BM_SSP_CMD0_APPEND_8CYC : 0),
		REGS_SSP1_BASE + HW_SSP_CMD0);

	/* prepare command's arguments */
	writel(cmd->cmdarg, REGS_SSP1_BASE + HW_SSP_CMD1);

	stm_setup_timout(0xffff);

	err = transfer_data(data);
	if (err != 0) {
		pr_err(" Transfering data failed\n");
		return err;
	}

	/* wait until finished */
	while (readl(REGS_SSP1_BASE + HW_SSP_CTRL0) & BM_SSP_CTRL0_RUN)
		;

	get_cards_response(cmd);

	return 0;
}

static int stm_mci_std_cmds(struct mmc *mmc, struct mmc_cmd *cmd)
{
	/* setup command and transfer parameters */
	REG_WR(REGS_SSP1_BASE, HW_SSP_CTRL0,
		prepare_transfer_setup(cmd->resp_type, 0) | BM_SSP_CTRL0_ENABLE);

	/* prepare the command, when no response is expected add a few trailing clocks */
	REG_WR(REGS_SSP1_BASE, HW_SSP_CMD0, 
		SSP_CMD0_CMD(cmd->cmdidx) | (cmd->resp_type & MMC_RSP_PRESENT ? 0 : BM_SSP_CMD0_APPEND_8CYC));

	/* prepare command's arguments */
	REG_WR(REGS_SSP1_BASE, HW_SSP_CMD1, cmd->cmdarg);

	stm_setup_timout(0xffff);

	/* start the transfer */
	REG_SET(REGS_SSP1_BASE, HW_SSP_CTRL0, BM_SSP_CTRL0_RUN);

	/* wait until finished */
	while (readl(REGS_SSP1_BASE + HW_SSP_CTRL0) & BM_SSP_CTRL0_RUN)
		;

	if (cmd->resp_type & MMC_RSP_PRESENT)
		get_cards_response(cmd);

	return get_cmd_error(readl(REGS_SSP1_BASE + HW_SSP_STATUS));
}

static void finish_request(void)
{
	/* stop the engines (normaly already done) */
	writel(BM_SSP_CTRL0_RUN, REGS_SSP1_BASE + HW_SSP_CTRL0 + 8);
}

static int ssp_mmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
			struct mmc_data *data)
{
	int rc;

	debug("MMC: CMD%d\n", cmd->cmdidx);

	if ((cmd->resp_type == 0) || (data == NULL))
		rc = stm_mci_std_cmds(mmc, cmd); 	/* no response and no data transfer */
	else
		rc = stm_mci_adtc(mmc, cmd, data);	/* with response and data */

	finish_request();
	return rc;
}

static uint32_t set_bit_clock(uint32_t nc)
{
	uint32_t ssp, div, rate, reg;

	if (nc == 0U) {
		/* TODO stop the clock */
		return 0;
	}

	ssp = imx_get_sspclk(0);

	/*
	 * SSP bit rate = SSPCLK / (CLOCK_DIVIDE * (1 + CLOCK_RATE)),
	 * CLOCK_DIVIDE has to be an even value from 2 to 254, and
	 * CLOCK_RATE could be any integer from 0 to 255.
	 */
	for (div = 2; div < 255; div += 2) {
		rate = DIV_ROUND_CLOSEST(DIV_ROUND_CLOSEST(ssp, nc), div);
		if (rate <= 0x100)
			break;
	}

	if (div >= 255) {
		pr_warning("Cannot set clock to %d Hz\n", nc);
		return 0;
	}

	reg = readl(REGS_SSP1_BASE + HW_SSP_TIMING) & SSP_TIMING_TIMEOUT_MASK;
	reg |= BF_SSP_TIMING_CLOCK_DIVIDE(div) | BF_SSP_TIMING_CLOCK_RATE(rate - 1);
	writel(reg, REGS_SSP1_BASE + HW_SSP_TIMING);

	return ssp / div / rate;
}

static void ssp_mmc_set_ios(struct mmc *mmc)
{
	debug("IO settings: bus width=%d, frequency=%u Hz\n", 
		mmc->bus_width, mmc->clock);

	/* Set the clock speed */
	if (mmc->clock)
		mmc->clock = set_bit_clock(mmc->clock);

	/* set bus_width when transfering */
}

static int ssp_mmc_init(struct mmc *mmc)
{
	/* Board level init */
	ssp_mmc_board_init();

	/* Prepare for software reset */
	REG_CLR(REGS_SSP1_BASE, HW_SSP_CTRL0, BM_SSP_CTRL0_SFTRST);
	REG_CLR(REGS_SSP1_BASE, HW_SSP_CTRL0, BM_SSP_CTRL0_CLKGATE);

	/* Assert reset */
	REG_SET(REGS_SSP1_BASE, HW_SSP_CTRL0, BM_SSP_CTRL0_SFTRST);

	while (!(REG_RD(REGS_SSP1_BASE, HW_SSP_CTRL0) & BM_SSP_CTRL0_CLKGATE))
		/* Wait for confirmation */;

	/* Done */
	REG_CLR(REGS_SSP1_BASE, HW_SSP_CTRL0, BM_SSP_CTRL0_SFTRST);
	REG_CLR(REGS_SSP1_BASE, HW_SSP_CTRL0, BM_SSP_CTRL0_CLKGATE);

	/* 8 bits word length in MMC mode */
	REG_WR(REGS_SSP1_BASE, HW_SSP_CTRL1,
		BM_SSP_CTRL1_POLARITY | 
		(BV_SSP_CTRL1_SSP_MODE__SD_MMC << BP_SSP_CTRL1_SSP_MODE) |
		(BV_SSP_CTRL1_WORD_LENGTH__EIGHT_BITS << BP_SSP_CTRL1_WORD_LENGTH));

	writel(BM_SSP_CTRL0_IGNORE_CRC |
		SSP_CTRL0_BUS_WIDTH(__ssp_bus_width(mmc->bus_width)),
		REGS_SSP1_BASE + HW_SSP_CTRL0);

	/* Send initial 74 clock cycles (185 us @ 400 KHz)*/
	REG_SET(REGS_SSP1_BASE, HW_SSP_CMD0, BM_SSP_CMD0_CONT_CLKING_EN);
	udelay(200);
	REG_CLR(REGS_SSP1_BASE, HW_SSP_CMD0, BM_SSP_CMD0_CONT_CLKING_EN);

	return 0;
}

int imx_ssp_mmc_initialize(bd_t *bis)
{
	struct mmc *mmc;

	mmc = malloc(sizeof(struct mmc));

	sprintf(mmc->name, "IMX_SSP_MMC");
	mmc->send_cmd = ssp_mmc_send_cmd;
	mmc->set_ios = ssp_mmc_set_ios;
	mmc->init = ssp_mmc_init;

	mmc->voltages = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->host_caps = MMC_MODE_4BIT | MMC_MODE_HS_52MHz | MMC_MODE_HS;

#if 0
	mmc->f_min = 400000;
	mmc->f_max = 148000000;	/* 297.731 MHz / 2 */
#else
	mmc->f_min = imx_get_sspclk(1) / 254 / 256;
	mmc->f_max = imx_get_sspclk(1) / 2 / 1;
#endif

	debug("Min. frequency is %u Hz\n", mmc->f_min);
	debug("Max. frequency is %u Hz\n", mmc->f_max);

	mmc_register(mmc);

	return 0;
}
