/*
 * ==========================================================================
 *
 *       Filename:  mxs-mmc.c
 *
 *    Description:  ssp driver with DMA on i.MX233
 *
 *        Version:  0.01
 *        Created:  2011年07月14日 17时01分33秒
 *
 *         Author:  smmei (), 
 *        Company:  
 *
 * ==========================================================================
 */

//#define DEBUG

#include 	<common.h>
#include 	<malloc.h>
#include 	<asm/io.h>
#include 	<asm/errno.h>
#include 	<asm/arch/registers/regsssp.h>
#include 	<asm/arch/registers/regsapbh.h>
#include 	<asm/arch/registers/hw_dma.h>
#include 	<asm/arch/registers/hw_irq.h>
#include 	<asm/arch/mxs-dma.h>
#include 	<asm/arch/mxs-block.h>
#include 	<mmc.h>

#ifdef DEBUG
#define MMCDEBUG(args...) 		\
	do { 				\
		serial_puts("[MMC] ");	\
		printf(args);		\
	} while (0)
#else
#define MMCDEBUG(args...)
#endif

#ifndef BF
#define BF(value, field) (((value) << BP_##field) & BM_##field)
#endif

/*
 * These are the command types.
 */
#define mmc_cmd_type(cmd)	((cmd)->flags & MMC_CMD_MASK)

#define mmc_resp_type(cmd) 	((cmd)->resp_type & 	\
				 (MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC|MMC_RSP_BUSY|MMC_RSP_OPCODE))

extern u32 imx_get_sspclk(u32 index);
extern void ssp_mmc_board_init(void);

struct mxs_mmc_host {
	struct mmc	*mmc;
	struct mmc_cmd 	*cmd;
	struct mmc_data *data;

	/* data bus width 0:1bit, 1:4bit, 2:8bit */
	unsigned char 	bus_width;

	//unsigned int 	present:1;

	/* DMA channel used for this host */
	unsigned int 	dmach;

	/* IRQs */
	int 		dmairq;
	int 		errirq;

	/* DMA descriptor to transfer data over SSP interface */
	struct mxs_dma_desc *dma_desc;

	int 		sdio_irq_en;
};

#ifdef DEBUG
int memory_display(char *addr, ulong offs, ulong nbytes, int size)
{
	ulong linebytes, i;
	u_char	*cp;

	#define DISP_LINE_LEN	16
	/* Print the lines.
	 *
	 * We buffer all read data, so we can make sure data is read only
	 * once, and all accesses are with the specified bus width.
	 */
	do {
		char	linebuf[DISP_LINE_LEN];
		uint	*uip = (uint   *)linebuf;
		ushort	*usp = (ushort *)linebuf;
		u_char	*ucp = (u_char *)linebuf;
		uint	count = 52;

		printf("%08lx:", offs);
		linebytes = (nbytes > DISP_LINE_LEN) ? DISP_LINE_LEN : nbytes;

		for (i = 0; i < linebytes; i += size) {
			if (size == 4) {
				printf(" %08x", (*uip++ = *((uint *)addr)));
				count -= 9;
			} else if (size == 2) {
				printf(" %04x", (*usp++ = *((ushort *)addr)));
				count -= 5;
			} else {
				printf(" %02x", (*ucp++ = *((u_char *)addr)));
				count -= 3;
			}
			addr += size;
			offs += size;
		}

		while(count--)
			serial_putc(' ');

		cp = (u_char *)linebuf;
		for (i=0; i<linebytes; i++) {
			if ((*cp < 0x20) || (*cp > 0x7e))
				serial_putc('.');
			else
				printf("%c", *cp);
			cp++;
		}
		serial_putc('\n');
		nbytes -= linebytes;
	} while (nbytes > 0);

	return 0;
}

static void dump_dma_context(struct mxs_dma_desc *d)
{
	int j;

	pr_info("    --\n");
	pr_info("    Physical Address: %p\n" , &(d->cmd));
	pr_info("    Next            : 0x%08lx\n", d->cmd.next);
	pr_info("    Command         : 0x%08lx\n", d->cmd.cmd.data);
	pr_info("    Buffer          : 0x%08x\n" , d->cmd.address);
	for (j = 0; j < 3; j++) {
		pr_info("    PIO[%u]          : 0x%08lx\n",
			j, d->cmd.pio_words[j]);
	}
}
#else
#define memory_display(a, b, c, d)
#define dump_dma_context(x)
#endif

/*
 * Check for MMC command errors
 * Returns error code or zerro if no errors
 */
static inline int mxs_mmc_cmd_error(u32 status)
{
	int err = 0;

	if (status & BM_SSP_STATUS_TIMEOUT)
		err = -ETIMEDOUT;
	else if (status & BM_SSP_STATUS_RESP_TIMEOUT)
		err = -ETIMEDOUT;
	else if (status & BM_SSP_STATUS_RESP_CRC_ERR)
		err = -EILSEQ;
	else if (status & BM_SSP_STATUS_RESP_ERR)
		err = -EIO;

	return err;
}

/* Detect if card is plugged */
int mxs_mmc_is_plugged(void)
{
	u32 status = __raw_readl(HW_SSP_STATUS_ADDR);
	return !(status & BM_SSP_STATUS_CARD_DETECT);
}

static int mxs_mmc_dma_go(void *pchain, unsigned int chan)
{
	uint32_t timout = 0xffffff;

	mxs_dma_apbh_reset(chan);
	mxs_dma_apbh_enable(pchain, chan);

	memory_display((void *)0x80010000, 0x80010000, 0x100, 4);
	memory_display((void *)0x80004000, 0x80004000, 0x100, 4);
	do {
		if (mxs_dma_apbh_irq_is_pending(chan)) {
			mxs_dma_apbh_ack_irq(chan);
			return 0;
		}
	} while (--timout);

	return -ETIMEDOUT;
}

/* Send the BC command to the device */
static int mxs_mmc_bc(struct mxs_mmc_host *host)
{
	struct mmc_cmd *cmd = host->cmd;
	struct mxs_dma_desc *dma_desc = host->dma_desc;
	uint32_t error, status;

	MMCDEBUG("%s\n", __func__);
	dma_desc->cmd.cmd.bits.command 	 = NO_DMA_XFER;
	dma_desc->cmd.cmd.bits.chain 	 = 0;
	dma_desc->cmd.cmd.bits.irq 	 = 1;
	dma_desc->cmd.cmd.bits.dec_sem 	 = 1;
	dma_desc->cmd.cmd.bits.wait4end  = 1;
	dma_desc->cmd.cmd.bits.pio_words = 3;
	dma_desc->cmd.cmd.bits.bytes 	 = 0;

	dma_desc->cmd.pio_words[0] = BM_SSP_CTRL0_ENABLE |
				     BM_SSP_CTRL0_IGNORE_CRC;
	dma_desc->cmd.pio_words[1] = BF(cmd->cmdidx, SSP_CMD0_CMD) |
				     BM_SSP_CMD0_APPEND_8CYC;
	dma_desc->cmd.pio_words[2] = BF(cmd->cmdarg, SSP_CMD1_CMD_ARG);

	if (host->sdio_irq_en) {
		dma_desc->cmd.pio_words[0] |= BM_SSP_CTRL0_SDIO_IRQ_CHECK;
		dma_desc->cmd.pio_words[1] |= BM_SSP_CMD0_CONT_CLKING_EN \
					      | BM_SSP_CMD0_SLOW_CLKING_EN;
	}

	dma_desc->cmd.next = (unsigned long)dma_desc;

	dump_dma_context(dma_desc);
	error = mxs_mmc_dma_go(&dma_desc->cmd, host->dmach);
	if (error)
		pr_err("[MMC] DMA timeout\n");

	status = __raw_readl(HW_SSP_STATUS_ADDR);
	error = mxs_mmc_cmd_error(status);

	if (error) {
		pr_err("[MMC] Command error 0x%x\n", error);
		mxs_dma_apbh_reset(host->dmach);
	}
	mxs_dma_apbh_disable(host->dmach);

	return error;
#if 0
	init_completion(&host->dma_done);
	mxs_dma_reset(host->dmach);
	if (mxs_dma_desc_append(host->dmach, host->dma_desc) < 0)
		dev_err(host->dev, "mmc_dma_desc_append failed\n");
	dev_dbg(host->dev, "%s start DMA.\n", __func__);
	if (mxs_dma_enable(host->dmach) < 0)
		dev_err(host->dev, "mmc_dma_enable failed\n");

	wait_for_completion(&host->dma_done);

	cmd->error = mxs_mmc_cmd_error(host->status);

	if (cmd->error) {
		dev_dbg(host->dev, "Command error 0x%x\n", cmd->error);
		mxs_dma_reset(host->dmach);
	}
	mxs_dma_disable(host->dmach);
#endif
}

static int mxs_mmc_ac(struct mxs_mmc_host *host)
{
	struct mmc_cmd *cmd = host->cmd;
	struct mxs_dma_desc *dma_desc = host->dma_desc;
	u32 ignore_crc, resp, long_resp;
	u32 ssp_ctrl0;
	u32 ssp_cmd0;
	u32 ssp_cmd1;
	uint32_t status, error;

	MMCDEBUG("%s\n", __func__);
	ignore_crc = (mmc_resp_type(cmd) & MMC_RSP_CRC) ?  0 : BM_SSP_CTRL0_IGNORE_CRC;
	resp 	   = (mmc_resp_type(cmd) & MMC_RSP_PRESENT) ?  BM_SSP_CTRL0_GET_RESP : 0;
	long_resp  = (mmc_resp_type(cmd) & MMC_RSP_136) ?  BM_SSP_CTRL0_LONG_RESP : 0;

	dma_desc->cmd.cmd.bits.command   = NO_DMA_XFER;
	dma_desc->cmd.cmd.bits.chain 	 = 0;
	dma_desc->cmd.cmd.bits.irq 	 = 1;
	dma_desc->cmd.cmd.bits.dec_sem   = 1;
	dma_desc->cmd.cmd.bits.wait4end  = 1;
	dma_desc->cmd.cmd.bits.pio_words = 3;
	dma_desc->cmd.cmd.bits.bytes     = 0;

	ssp_ctrl0 = BM_SSP_CTRL0_ENABLE | ignore_crc | long_resp | resp;
	ssp_cmd0 = BF(cmd->cmdidx, SSP_CMD0_CMD);
	ssp_cmd1 = BF(cmd->cmdarg, SSP_CMD1_CMD_ARG);

	if (host->sdio_irq_en) {
		ssp_ctrl0 |= BM_SSP_CTRL0_SDIO_IRQ_CHECK;
		ssp_cmd0 |= BM_SSP_CMD0_CONT_CLKING_EN | BM_SSP_CMD0_SLOW_CLKING_EN;
	}

	dma_desc->cmd.pio_words[0] = ssp_ctrl0;
	dma_desc->cmd.pio_words[1] = ssp_cmd0;
	dma_desc->cmd.pio_words[2] = ssp_cmd1;

	dma_desc->cmd.next = (unsigned long)dma_desc;

	dump_dma_context(dma_desc);
	error = mxs_mmc_dma_go(&dma_desc->cmd, host->dmach);
	if (error)
		pr_err("[MMC] DMA timeout\n");
#if 0
	mxs_dma_reset(host->dmach);
	init_completion(&host->dma_done);
	if (mxs_dma_desc_append(host->dmach, host->dma_desc) < 0)
		dev_err(host->dev, "mmc_dma_desc_append failed\n");
	dev_dbg(host->dev, "%s start DMA.\n", __func__);
	if (mxs_dma_enable(host->dmach) < 0)
		dev_err(host->dev, "mmc_dma_enable failed\n");
	wait_for_completion(&host->dma_done);
#endif

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		while (__raw_readl(HW_SSP_CTRL0_ADDR) & BM_SSP_CTRL0_RUN)
			continue;
		break;
	case MMC_RSP_R1:
	case MMC_RSP_R1b:
	case MMC_RSP_R3:
		cmd->response[0] = __raw_readl(HW_SSP_SDRESP0_ADDR);
		break;
	case MMC_RSP_R2:
		cmd->response[3] = __raw_readl(HW_SSP_SDRESP0_ADDR);
		cmd->response[2] = __raw_readl(HW_SSP_SDRESP1_ADDR);
		cmd->response[1] = __raw_readl(HW_SSP_SDRESP2_ADDR);
		cmd->response[0] = __raw_readl(HW_SSP_SDRESP3_ADDR);
		break;
	default:
		pr_err("[MMC] Unsupported response type 0x%x\n",
			 mmc_resp_type(cmd));
		break;
	}

	status = __raw_readl(HW_SSP_STATUS_ADDR);
	error = mxs_mmc_cmd_error(status);

	if (error) {
		pr_err("[MMC] Command error 0x%x\n", error);
		mxs_dma_apbh_reset(host->dmach);
	}
	mxs_dma_apbh_disable(host->dmach);

	return error;
}

static int mxs_mmc_adtc(struct mxs_mmc_host *host)
{
	struct mmc_cmd *cmd = host->cmd;
	struct mmc_data *data = host->data;
	struct mxs_dma_desc *dma_desc = host->dma_desc;
	int ignore_crc, resp, long_resp;
	int is_reading = 0;
	unsigned int ssp_ver_major;
	uint32_t status, error;

	u32 ssp_ctrl0 = 0;
	u32 ssp_cmd0 = 0;
	u32 ssp_cmd1 = 0;
	u32 timeout;
	u32 val;

	u32 data_size = data->blocksize * data->blocks;
	u32 log2_block_size;

	ignore_crc = mmc_resp_type(cmd) & MMC_RSP_CRC ? 0 : 1;
	resp = mmc_resp_type(cmd) & MMC_RSP_PRESENT ? 1 : 0;
	long_resp = mmc_resp_type(cmd) & MMC_RSP_136 ? 1 : 0;

	MMCDEBUG("ADTC command:\n"
		 "response: %d, ignore crc: %d\n"
		 "blocksz: %u, blocks: %u, flags: 0x%x\n",
		 resp, ignore_crc,
		 data->blocksize, data->blocks, data->flags);

	if (data->flags & MMC_DATA_WRITE) {
		MMCDEBUG("Data Write\n");
		is_reading = 0;
	} else if (data->flags & MMC_DATA_READ) {
		MMCDEBUG("Data Read\n");
		is_reading = 1;
	} else {
		pr_err("[MMC] Unsuspported data mode, 0x%x\n", data->flags);
		return -EINVAL;
	}

	/* when is_reading is set, DMA controller performs WRITE operation. */
	dma_desc->cmd.cmd.bits.command   = is_reading ? DMA_WRITE : DMA_READ;
	dma_desc->cmd.cmd.bits.chain 	 = 0;
	dma_desc->cmd.cmd.bits.irq 	 = 1;
	dma_desc->cmd.cmd.bits.dec_sem   = 1;
	dma_desc->cmd.cmd.bits.wait4end  = 1;
	dma_desc->cmd.cmd.bits.pio_words = 3;
	dma_desc->cmd.cmd.bits.bytes     = data_size;

	dma_desc->cmd.address = (dma_addr_t)data->dest;

	ssp_ver_major = __raw_readl(HW_SSP_VERSION_ADDR) >> 24;
	MMCDEBUG("ssp ver major is 0x%x\n", ssp_ver_major);
	if (ssp_ver_major > 3) {
		MMCDEBUG("ssp version not support\n");
	} else {
		ssp_ctrl0 = (ignore_crc ? BM_SSP_CTRL0_IGNORE_CRC : 0) |
			(resp ? BM_SSP_CTRL0_GET_RESP : 0) |
			(long_resp ? BM_SSP_CTRL0_LONG_RESP : 0) |
			(is_reading ? BM_SSP_CTRL0_READ : 0) |
			BM_SSP_CTRL0_DATA_XFER | BM_SSP_CTRL0_WAIT_FOR_IRQ |
			BM_SSP_CTRL0_ENABLE |
			BF(data_size, SSP_CTRL0_XFER_COUNT) |
			BF(host->bus_width ? BV_SSP_CTRL0_BUS_WIDTH__FOUR_BIT :
					     BV_SSP_CTRL0_BUS_WIDTH__ONE_BIT,
					     SSP_CTRL0_BUS_WIDTH);
	}

	/*
	 * We need to set the hardware register to the logarithm to base 2 of
	 * the block size.
	 */
	//log2_block_size = ilog2(data->blocksize);
	log2_block_size = ffs(data->blocksize) - 1;
	MMCDEBUG("%s blocksize is 0x%x.\n", __func__, log2_block_size);

	if (ssp_ver_major > 3) {
		MMCDEBUG("ssp version not support\n");
	} else {
		if ((1<<log2_block_size) != data->blocksize) {
			//BUG_ON(data->blocks > 1);
			ssp_cmd0 =
				BF(0, SSP_CMD0_BLOCK_SIZE) |
				BF(cmd->cmdidx, SSP_CMD0_CMD) |
				BF(0, SSP_CMD0_BLOCK_COUNT);
		} else
			ssp_cmd0 = 
				BF(log2_block_size, SSP_CMD0_BLOCK_SIZE) |
				BF(cmd->cmdidx, SSP_CMD0_CMD) |
				BF(data->blocks - 1, SSP_CMD0_BLOCK_COUNT);
	}

	if (host->sdio_irq_en) {
		ssp_ctrl0 |= BM_SSP_CTRL0_SDIO_IRQ_CHECK;
		ssp_cmd0  |= BM_SSP_CMD0_CONT_CLKING_EN |
			     BM_SSP_CMD0_SLOW_CLKING_EN;
	}

	if ((cmd->cmdidx == 12) || (cmd->cmdidx == 53))
		ssp_cmd0 |= BM_SSP_CMD0_APPEND_8CYC;

	ssp_cmd1 = BF(cmd->cmdarg, SSP_CMD1_CMD_ARG);

	dma_desc->cmd.pio_words[0] = ssp_ctrl0;
	dma_desc->cmd.pio_words[1] = ssp_cmd0;
	dma_desc->cmd.pio_words[2] = ssp_cmd1;

	dma_desc->cmd.next = (unsigned long)dma_desc;

	/* Set the timeout count */
	//timeout = mxs_ns_to_ssp_ticks(host->clkrt, data->timeout_ns);
	timeout = 0xffff;
	val = __raw_readl(HW_SSP_TIMING_ADDR);
	val &= ~(BM_SSP_TIMING_TIMEOUT);
	val |= BF(timeout, SSP_TIMING_TIMEOUT);
	__raw_writel(val, HW_SSP_TIMING_ADDR);

	dump_dma_context(dma_desc);
	error = mxs_mmc_dma_go(&dma_desc->cmd, host->dmach);
	if (error)
		pr_err("[MMC] DMA timeout\n");
#if 0
	init_completion(&host->dma_done);
	mxs_dma_reset(host->dmach);
	if (mxs_dma_desc_append(host->dmach, host->dma_desc) < 0)
		dev_err(host->dev, "mmc_dma_desc_append failed\n");
	dev_dbg(host->dev, "%s start DMA.\n", __func__);
	if (mxs_dma_enable(host->dmach) < 0)
		dev_err(host->dev, "mmc_dma_enable failed\n");
	wait_for_completion(&host->dma_done);
	if (host->regulator)
		regulator_set_current_limit(host->regulator, 0, 0);
#endif

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		break;
	case MMC_RSP_R1:
	case MMC_RSP_R3:
		cmd->response[0] = __raw_readl(HW_SSP_SDRESP0_ADDR);
		break;
	case MMC_RSP_R2:
		cmd->response[3] = __raw_readl(HW_SSP_SDRESP0_ADDR);
		cmd->response[2] = __raw_readl(HW_SSP_SDRESP1_ADDR);
		cmd->response[1] = __raw_readl(HW_SSP_SDRESP2_ADDR);
		cmd->response[0] = __raw_readl(HW_SSP_SDRESP3_ADDR);
		break;
	default:
		pr_err("Unsupported response type 0x%x\n",
		       mmc_resp_type(cmd));
		break;
	}

	status = __raw_readl(HW_SSP_STATUS_ADDR);
	error = mxs_mmc_cmd_error(status);

	if (error) {
		MMCDEBUG("Command error 0x%x\n", error);
		mxs_dma_apbh_reset(host->dmach);
	} else
		MMCDEBUG("Transferred %u bytes\n", data_size);

	mxs_dma_apbh_disable(host->dmach);

	return error;
}

static int mxs_mmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
			    struct mmc_data *data)
{
	struct mxs_mmc_host *host = mmc->priv;
	int ret = 0;

	MMCDEBUG("%s command: type: 0x%x cmdidx: %u, arg: %u, flags 0x%x\n",
		 __func__, mmc_cmd_type(cmd), cmd->cmdidx, cmd->cmdarg, cmd->flags);

	host->cmd = cmd;
	host->data = data;

	switch (mmc_cmd_type(cmd)) {
	case MMC_CMD_BC:
		ret = mxs_mmc_bc(host);
		break;
	case MMC_CMD_BCR:
		ret = mxs_mmc_ac(host);
		break;
	case MMC_CMD_AC:
		ret = mxs_mmc_ac(host);
		break;
	case MMC_CMD_ADTC:
		ret = mxs_mmc_adtc(host);
		break;
	default:
		pr_err("Unknown MMC command\n");
		ret = -EINVAL;
		break;
	}

	MMCDEBUG("%s, response: %x %x %x %x\n",
		 __func__, cmd->response[0], cmd->response[1],
		 cmd->response[2], cmd->response[3]);

	return ret;
}

/**
 * @param hw_dev Host interface device instance
 * @param nc New Clock in [Hz] (may be 0 to disable the clock)
 * @return The real clock frequency
 *
 * The SSP unit clock can base on the external 24 MHz or the internal 480 MHz
 * Its unit clock value is derived from the io clock, from the SSP divider
 * and at least the SSP bus clock itself is derived from the SSP unit's divider
 *
 * @note Up to "SSP unit DIV" the outer world must care. This routine only
 * handles the "SSP DIV".
 */
static unsigned mxs_set_sclk_speed(struct mxs_mmc_host *host, unsigned nc)
{
	unsigned ssp, div, rate, reg;

	if (nc == 0U)
		return 0;

	ssp = imx_get_sspclk(0);

	for (div = 2; div < 255; div += 2) {
		rate = DIV_ROUND_CLOSEST(DIV_ROUND_CLOSEST(ssp, nc), div);
		if (rate <= 0x100)
			break;
	}
	if (div >= 255) {
		pr_warning("Cannot set clock to %d Hz\n", nc);
		return 0;
	}

	reg = readl(HW_SSP_TIMING_ADDR) & BM_SSP_TIMING_TIMEOUT;
	reg |= BF_SSP_TIMING_CLOCK_DIVIDE(div) | BF_SSP_TIMING_CLOCK_RATE(rate - 1);
	writel(reg, HW_SSP_TIMING_ADDR);

	return ssp / div / rate;
}

/* Configure card */
static void mxs_mmc_set_ios(struct mmc *mmc)
{
	struct mxs_mmc_host *host = mmc->priv;

	MMCDEBUG("%s, bus_width %u, clock %u\n", 
		 __func__, mmc->bus_width, mmc->clock);

	if (mmc->bus_width == 8)
		host->bus_width = 2;
	else if (mmc->bus_width == 4)
		host->bus_width = 1;
	else
		host->bus_width = 0;

	if (mmc->clock > 0)
		mxs_set_sclk_speed(host, mmc->clock);
}

/* Set up MMC pins */
static void mxs_mmc_hw_init_mmc(void)
{
	ssp_mmc_board_init();
}

/* Reset ssp peripheral to default values */
static void mxs_mmc_reset(struct mxs_mmc_host *host)
{
	u32 ssp_ctrl0;
	u32 ssp_ctrl1;

	mxs_reset_block((void *)HW_SSP_CTRL0_ADDR, 0);

	/* Configure SSP Control Register 0 */
	ssp_ctrl0 =
		BM_SSP_CTRL0_IGNORE_CRC |
		BF(BV_SSP_CTRL0_BUS_WIDTH__ONE_BIT, SSP_CTRL0_BUS_WIDTH);

	/* Configure SSP Control Register 1 */
	ssp_ctrl1 =
		BM_SSP_CTRL1_DMA_ENABLE 		|
		BM_SSP_CTRL1_POLARITY 			|
		BM_SSP_CTRL1_RECV_TIMEOUT_IRQ_EN 	|
		BM_SSP_CTRL1_DATA_CRC_IRQ_EN 		|
		BM_SSP_CTRL1_DATA_TIMEOUT_IRQ_EN 	|
		BM_SSP_CTRL1_RESP_TIMEOUT_IRQ_EN 	|
		BM_SSP_CTRL1_RESP_ERR_IRQ_EN 		|
		BF(BV_SSP_CTRL1_WORD_LENGTH__EIGHT_BITS, SSP_CTRL1_WORD_LENGTH) |
		BF(BV_SSP_CTRL1_SSP_MODE__SD_MMC, SSP_CTRL1_SSP_MODE);

	__raw_writel(BF(0xFFFF, SSP_TIMING_TIMEOUT) |
		     BF(2, SSP_TIMING_CLOCK_DIVIDE) |
		     BF(0, SSP_TIMING_CLOCK_RATE),
		     HW_SSP_TIMING_ADDR);

	if (host->sdio_irq_en) {
		ssp_ctrl1 |= BM_SSP_CTRL1_SDIO_IRQ_EN;
		ssp_ctrl0 |= BM_SSP_CTRL0_SDIO_IRQ_CHECK;
	}

	/* Write the SSP Control Register 0 and 1 values out to the interface */
	__raw_writel(ssp_ctrl0, HW_SSP_CTRL0_ADDR);
	__raw_writel(ssp_ctrl1, HW_SSP_CTRL1_ADDR);

	/*
	 * This delay must be at least 74 clock sizes, or 1 ms, or the
	 * time required to reach a stable voltage.
	 */
	__raw_writel(BM_SSP_CMD0_CONT_CLKING_EN, HW_SSP_CMD0_SET_ADDR);
	udelay(200);
	__raw_writel(BM_SSP_CMD0_CONT_CLKING_EN, HW_SSP_CMD0_CLR_ADDR);
}

static int mxs_mmc_init(struct mmc *mmc)
{
	struct mxs_mmc_host *host = mmc->priv;

	MMCDEBUG("%s\n", __func__);

	/* Set up MMC pins */
	mxs_mmc_hw_init_mmc();

	//mxs_mmc_clk_enable();

	//mxs_set_sclk_speed(host, mmc->min_clk);

	/* Reset MMC block, Reset SSP peripheral to default values */
	mxs_mmc_reset(host);

	return 0;
}

static int mxs_mmc_dma_init(struct mxs_mmc_host *host, int reset)
{
	int ret = 0;

	if (!reset) {
		host->dma_desc = mxs_dma_alloc_desc();
		if (host->dma_desc == NULL) {
			pr_err("[MMC] Unable to allocate DMA descriptor\n");
			ret = -ENOMEM;
			goto out;
		}
	}

	/* Reset DMA channel */
	mxs_dma_apbh_reset(host->dmach);

	/* Enable DMA interrupt */
	mxs_dma_apbh_ack_irq(host->dmach);
	mxs_dma_apbh_enable_irq(host->dmach, 1);

out:
	return ret;
}

/**
 * mxs_mmc_probe - SD/MMC 初始化
 *	1. 分配资源
 *	2. 设备初始化(PINs, SSP controller)
 */
int mxs_mmc_probe(void)
{
	struct mxs_mmc_host *host;
	struct mmc *mmc;
	int err = 0;

	mmc = malloc(sizeof(*mmc) + sizeof(*host));
	if (!mmc) {
		pr_err("Unable to allocate MMC\n");
		err = -ENOMEM;
		goto out;
	}
	memset(mmc, 0, sizeof(*mmc) + sizeof(*host));

	host = (struct mxs_mmc_host *)&mmc[1];
	mmc->priv = host;

	host->dmach = HW_APBH_DMA_SSP1_CHANNEL;
	host->dmairq = HW_IRQ_SSP1_DMA;
	host->errirq = HW_IRQ_SSP_ERROR;

	host->mmc = mmc;
	host->sdio_irq_en = 0;

	err = mxs_mmc_dma_init(host, 0);
	if (err) {
		pr_err("%s: DMA init failed\n", __func__);
		goto out_dma;
	}

	//host->present = mxs_mmc_is_plugged(host);

	/* mmc */
	sprintf(mmc->name, "mxs-mmc");
	mmc->send_cmd = mxs_mmc_send_cmd;
	mmc->set_ios = mxs_mmc_set_ios;
	mmc->init = mxs_mmc_init;

	mmc->voltages = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->host_caps = MMC_MODE_4BIT | MMC_MODE_HS_52MHz | MMC_MODE_HS;

	mmc->f_min = 400000;
	mmc->f_max = 50000000;	/* 50 MHz */

	MMCDEBUG("Min. frequency is %u Hz\n", mmc->f_min);
	MMCDEBUG("Max. frequency is %u Hz\n", mmc->f_max);

	mmc_register(mmc);

	return err;

out_dma:
	free(mmc);
out:
	return err;
}
