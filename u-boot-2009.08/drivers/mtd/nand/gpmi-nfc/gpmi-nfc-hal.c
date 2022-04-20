/*
 * Freescale GPMI NFC NAND Flash Driver
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 * Copyright (C) 2008 Embedded Alley Solutions, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "gpmi-nfc.h"

/**
 * gpmi_nfc_dma_go - Run a DMA channel.
 *
 * @this:         Per-device data structure.
 * @dma_channel:  The DMA channel we're going to use.
 */
static int gpmi_nfc_dma_go(struct gpmi_nfc_data *this, int chan)
{
	struct nfc_hal       *nfc =  this->nfc;
	struct mxs_dma_desc  **d  = nfc->dma_descriptors;

	uint32_t timout = 0xffffff;

	mxs_dma_apbh_reset(chan);
	mxs_dma_apbh_enable(d[0], chan);

	do {
		if (mxs_dma_apbh_irq_is_pending(chan)) {
			mxs_dma_apbh_ack_irq(chan);
			return 0;
		}
	} while (--timout);

	return -ETIMEDOUT;
}

/**
 * init() - Initializes the NFC hardware.
 *
 * @this:  Per-device data.
 */
static int init(struct gpmi_nfc_data *this)
{
	int               error;

	/* Initialize DMA. */
	error = gpmi_nfc_dma_init(this);
	if (error)
		return error;

	/* Enable the clock. */
	clk_gpmi_enable();

	/* Reset the GPMI block. */
	mxs_reset_block((void *)HW_GPMI_CTRL0_ADDR, true);

	/* Choose NAND mode. */
	__raw_writel(BM_GPMI_CTRL1_GPMI_MODE, HW_GPMI_CTRL1_CLR_ADDR);

	/* Set the IRQ polarity. */
	//__raw_writel(BM_GPMI_CTRL1_ATA_IRQRDY_POLARITY, HW_GPMI_CTRL1_SET_ADDR);

	/* Disable write protection. */
	__raw_writel(BM_GPMI_CTRL1_DEV_RESET, HW_GPMI_CTRL1_SET_ADDR);

	/* Select BCH ECC. */
	__raw_writel(BM_GPMI_CTRL1_BCH_MODE, HW_GPMI_CTRL1_SET_ADDR);

	/* DMA2ECC - just for test */
	//__raw_writel(BM_GPMI_CTRL1_DMA2ECC_MODE, HW_GPMI_CTRL1_SET_ADDR);

	/* Disable the clock. */
	clk_gpmi_disable();

	/* If control arrives here, all is well. */
	return 0;
}

/**
 * set_geometry() - Configures the NFC geometry.
 *
 * @this:  Per-device data.
 */
static int set_geometry(struct gpmi_nfc_data *this)
{
	struct resources     *resources = &this->resources;
	struct nfc_geometry  *nfc       = &this->nfc_geometry;
	unsigned int         block_count;
	unsigned int         block_size;
	unsigned int         metadata_size;
	unsigned int         ecc_strength;
	unsigned int         page_size;

	/* We make the abstract choices in a common function. */
	if (gpmi_nfc_set_geometry(this))
		return !0;

	/* Translate the abstract choices into register fields. */
	block_count   = nfc->ecc_chunk_count - 1;
	block_size    = nfc->ecc_chunk_size_in_bytes;
	metadata_size = nfc->metadata_size_in_bytes;
	ecc_strength  = nfc->ecc_strength >> 1;
	page_size     = nfc->page_size_in_bytes;

	/* Enable the clock. */
	clk_gpmi_enable();

	/*
	 * Reset the BCH block. Notice that we pass in true for the just_enable
	 * flag. This is because the soft reset for the version 0 BCH block
	 * doesn't work and the version 1 BCH block is similar enough that we
	 * suspect the same (though this has not been officially tested). If you
	 * try to soft reset a version 0 BCH block, it becomes unusable until
	 * the next hard reset.
	 */

	mxs_reset_block(resources->bch_regs, true);

	/* Configure layout 0. */
	__raw_writel(
		BF_BCH_FLASH0LAYOUT0_NBLOCKS(block_count)     |
		BF_BCH_FLASH0LAYOUT0_META_SIZE(metadata_size) |
		BF_BCH_FLASH0LAYOUT0_ECC0(ecc_strength)       |
		BF_BCH_FLASH0LAYOUT0_DATA0_SIZE(block_size)   ,
		HW_BCH_FLASH0LAYOUT0_ADDR);
		//resources->bch_regs + HW_BCH_FLASH0LAYOUT0);

	__raw_writel(
		BF_BCH_FLASH0LAYOUT1_PAGE_SIZE(page_size)   |
		BF_BCH_FLASH0LAYOUT1_ECCN(ecc_strength)     |
		BF_BCH_FLASH0LAYOUT1_DATAN_SIZE(block_size) ,
		HW_BCH_FLASH0LAYOUT1_ADDR);
		//resources->bch_regs + HW_BCH_FLASH0LAYOUT1);

	/* Set *all* chip selects to use layout 0. */
	__raw_writel(0, HW_BCH_LAYOUTSELECT_ADDR);
	//resources->bch_regs + HW_BCH_LAYOUTSELECT);

	/* Enable interrupts. */
	//__raw_writel(BM_BCH_CTRL_COMPLETE_IRQ_EN, HW_BCH_CTRL_SET_ADDR);
	__raw_writel(BM_BCH_CTRL_COMPLETE_IRQ_EN, HW_BCH_CTRL_CLR_ADDR);
				//resources->bch_regs + HW_BCH_CTRL_SET);

	/* Disable the clock. */
	clk_gpmi_disable();

	/* Return success. */
	return 0;
}

static int gpmi_find_cycles(uint32_t ns, uint32_t period, uint32_t cn)
{
	int i, t = period;

	for (i = 1; i < cn; i++) {
		if (t >= ns)
			break;
		t += period;
	}

	return i;
}

extern unsigned imx_get_gpmiclk(void);

/**
 * set_set_timing() - Configures the NFC timing.
 *
 * @this:    Per-device data.
 * @timing:  The timing of interest.
 */
static int set_timing(struct gpmi_nfc_data *this,
		      const struct gpmi_nfc_timing *timing)
{
	struct nfc_hal *nfc = this->nfc;
	uint32_t clk, period;
	int t_ds, t_dh, t_as, t_data_sample;
	uint32_t val;

	clk_gpmi_enable();

	/* Accept the new timing. */
	nfc->timing = *timing;

	clk = imx_get_gpmiclk() / 1000;
	debug("%s: clk %d kHz\n", __func__, clk);
	period = DIV_ROUND_UP(1000000, clk);
	debug("%s: period %d ns\n", __func__, period);

	t_ds = gpmi_find_cycles(nfc->timing.data_setup_in_ns, period, 10);
	t_dh = gpmi_find_cycles(nfc->timing.data_hold_in_ns, period, 10);
	t_as = gpmi_find_cycles(nfc->timing.address_setup_in_ns, period, 10);
	
	debug("%s: t_ds %x, t_dh %x, t_as %x\n", __func__, t_ds, t_dh, t_as);

	HW_GPMI_TIMING0_WR(
		BF_GPMI_TIMING0_ADDRESS_SETUP(t_as) |
		BF_GPMI_TIMING0_DATA_HOLD(t_dh) |
		BF_GPMI_TIMING0_DATA_SETUP(t_ds)
	);

	/* data sample time */
	t_data_sample = gpmi_find_cycles(
		nfc->timing.gpmi_sample_delay_in_ns + (period >> 2), 
		period, 10) - 1;

	debug("%s: t_data_sample %x\n", __func__, t_data_sample);

	val = readl(HW_GPMI_CTRL1_ADDR);
	val &= ~BM_GPMI_CTRL1_RDN_DELAY;
	val |= BF_GPMI_CTRL1_RDN_DELAY(t_data_sample);
	writel(val, HW_GPMI_CTRL1_ADDR);

	/* Device busy timout */
	writel(BM_GPMI_TIMING1_DEVICE_BUSY_TIMEOUT, HW_GPMI_TIMING1_ADDR);

	clk_gpmi_disable();

	debug("GPMI TIME0: 0x%x\n", readl(HW_GPMI_TIMING0_ADDR));
	debug("GPMI TIME1: 0x%x\n", readl(HW_GPMI_TIMING1_ADDR));

	return 0;
}

/**
 * exit() - Shuts down the NFC hardware.
 *
 * @this:  Per-device data.
 */
static void exit(struct gpmi_nfc_data *this)
{
	gpmi_nfc_dma_exit(this);
}

/**
 * begin() - Begin NFC I/O.
 *
 * @this:  Per-device data.
 */
static void begin(struct gpmi_nfc_data *this)
{
	/* Enable the clock. */
	clk_gpmi_enable();
}

/**
 * end() - End NFC I/O.
 *
 * @this:  Per-device data.
 */
static void end(struct gpmi_nfc_data *this)
{
	/* Disable the clock. */
	clk_gpmi_disable();
}

/**
 * clear_bch() - Clears a BCH interrupt.
 *
 * @this:  Per-device data.
 */
static void clear_bch(struct gpmi_nfc_data *this)
{
	__raw_writel(BM_BCH_CTRL_COMPLETE_IRQ, HW_BCH_CTRL_CLR_ADDR);
}

/**
 * is_ready() - Returns the ready/busy status of the given chip.
 *
 * @this:  Per-device data.
 * @chip:  The chip of interest.
 */
static int is_ready(struct gpmi_nfc_data *this, unsigned chip)
{
	uint32_t          mask;
	uint32_t          register_image;

	/* Extract and return the status. */
	mask = BM_GPMI_DEBUG_READY0 << chip;

	register_image = __raw_readl(HW_GPMI_DEBUG_ADDR);

	return !!(register_image & mask);
}

/**
 * send_command() - Sends a command and associated addresses.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that contains the command bytes.
 * @length:  The number of bytes in the buffer.
 */
static int send_command(struct gpmi_nfc_data *this, unsigned chip,
					dma_addr_t buffer, unsigned int length)
{
	struct resources     *resources = &this->resources;
	struct nfc_hal       *nfc       =  this->nfc;
	struct mxs_dma_desc  **d        = nfc->dma_descriptors;
	int                  dma_channel;
	int                  error;
	uint32_t             command_mode;
	uint32_t             address;

	/* Compute the DMA channel. */

	dma_channel = resources->dma_low_channel + chip;

	/* A DMA descriptor that sends out the command. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_CLE;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = DMA_READ;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 0;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 3;
	(*d)->cmd.cmd.bits.bytes             = length;

	(*d)->cmd.address = buffer;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BM_GPMI_CTRL0_LOCK_CS			 | /* lock cs */
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BM_GPMI_CTRL0_ADDRESS_INCREMENT          |
		BF_GPMI_CTRL0_XFER_COUNT(length)         ;
		//BM_GPMI_CTRL0_UDMA			 | /* use dma */
	
	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;

//	mxs_dma_desc_append(dma_channel, (*d));
	(*d)->cmd.next = 0;
	d++;

	/* Go! */
	error = gpmi_nfc_dma_go(this, dma_channel);
	if (error)
		printf("[%s] DMA error\n", __func__);

	/* Return success. */
	return error;
}

/**
 * send_data() - Sends data to the given chip.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that contains the data.
 * @length:  The number of bytes in the buffer.
 */
static int send_data(struct gpmi_nfc_data *this, unsigned chip,
					dma_addr_t buffer, unsigned int length)
{
	struct resources     *resources = &this->resources;
	struct nfc_hal       *nfc       =  this->nfc;
	struct mxs_dma_desc  **d        = nfc->dma_descriptors;
	int                  dma_channel;
	int                  error = 0;
	uint32_t             command_mode;
	uint32_t             address;

	/* Compute the DMA channel. */

	dma_channel = resources->dma_low_channel + chip;

	/* A DMA descriptor that writes a buffer out. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = DMA_READ;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 4;
	(*d)->cmd.cmd.bits.bytes             = length;

	(*d)->cmd.address = buffer;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(length)         ;
	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;
	(*d)->cmd.pio_words[3] = 0;

//	mxs_dma_desc_append(dma_channel, (*d));
	(*d)->cmd.next = 0;
	d++;

	/* Go! */
	error = gpmi_nfc_dma_go(this, dma_channel);
	if (error)
		printf("[%s] DMA error\n", __func__);

	/* Return success. */
	return error;
}

/**
 * read_data() - Receives data from the given chip.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that will receive the data.
 * @length:  The number of bytes to read.
 */
static int read_data(struct gpmi_nfc_data *this, unsigned chip,
					dma_addr_t buffer, unsigned int length)
{
	struct resources     *resources = &this->resources;
	struct nfc_hal       *nfc       =  this->nfc;
	struct mxs_dma_desc  **d        = nfc->dma_descriptors;
	int                  dma_channel;
	int                  error = 0;
	uint32_t             command_mode;
	uint32_t             address;

	/* Compute the DMA channel. */
	dma_channel = resources->dma_low_channel + chip;

	/* A DMA descriptor that reads the data. */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__READ;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = DMA_WRITE;
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 0;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 0;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 1;
	(*d)->cmd.cmd.bits.bytes             = length;

	(*d)->cmd.address = buffer;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(length)         ;

//	mxs_dma_desc_append(dma_channel, (*d));
	(*d)->cmd.next = (uint32_t)*(d + 1);
	d++;

	/*
	 * A DMA descriptor that waits for the command to end and the chip to
	 * become ready.
	 *
	 * I think we actually should *not* be waiting for the chip to become
	 * ready because, after all, we don't care. I think the original code
	 * did that and no one has re-thought it yet.
	 */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 1;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 4;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(0)              ;
	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;
	(*d)->cmd.pio_words[3] = 0;

//	mxs_dma_desc_append(dma_channel, (*d));
	(*d)->cmd.next = 0;
	d++;

	/* Go! */
	error = gpmi_nfc_dma_go(this, dma_channel);
	if (error)
		printf("[%s] DMA error\n", __func__);

	/* Return success. */
	return error;
}

/**
 * send_page() - Sends a page, using ECC.
 *
 * @this:       Per-device data.
 * @chip:       The chip of interest.
 * @payload:    The physical address of the payload buffer.
 * @auxiliary:  The physical address of the auxiliary buffer.
 */
static int send_page(struct gpmi_nfc_data *this, unsigned chip,
				dma_addr_t payload, dma_addr_t auxiliary)
{
	struct resources     *resources = &this->resources;
	struct nfc_hal       *nfc       =  this->nfc;
	struct nfc_geometry  *nfc_geo   = &this->nfc_geometry;
	struct mxs_dma_desc  **d        = nfc->dma_descriptors;
	int                  dma_channel;
	int                  error = 0;
	uint32_t             command_mode;
	uint32_t             address;
	uint32_t             ecc_command;
	uint32_t             buffer_mask;

	/* Compute the DMA channel. */
	dma_channel = resources->dma_low_channel + chip;

	/* A DMA descriptor that does an ECC page read. */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
	ecc_command  = BV_GPMI_ECCCTRL_ECC_CMD__ENCODE_8_BIT;
	buffer_mask  = BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
		       BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 6;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(0)              ;

	(*d)->cmd.pio_words[1] = 0;

	(*d)->cmd.pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC               |
		BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)     |
		BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask) ;

	(*d)->cmd.pio_words[3] = nfc_geo->page_size_in_bytes;
	(*d)->cmd.pio_words[4] = payload;
	(*d)->cmd.pio_words[5] = auxiliary;

	//mxs_dma_desc_append(dma_channel, (*d));
	(*d)->cmd.next = (uint32_t)*(d + 1);
	d++;

	error = gpmi_nfc_dma_go(this, dma_channel);
	if (error)
		printf("[%s] DMA error\n", __func__);

	return error;
}

/**
 * read_page() - Reads a page, using ECC.
 *
 * @this:       Per-device data.
 * @chip:       The chip of interest.
 * @payload:    The physical address of the payload buffer.
 * @auxiliary:  The physical address of the auxiliary buffer.
 */
static int read_page(struct gpmi_nfc_data *this, unsigned chip,
				dma_addr_t payload, dma_addr_t auxiliary)
{
	struct resources     *resources = &this->resources;
	struct nfc_hal       *nfc       = this->nfc;
	struct nfc_geometry  *nfc_geo   = &this->nfc_geometry;
	struct mxs_dma_desc  **d        = nfc->dma_descriptors;
	int                  dma_channel;
	int                  error = 0;
	uint32_t             command_mode;
	uint32_t             address;
	uint32_t             ecc_command;
	uint32_t             buffer_mask;

	/* Compute the DMA channel. */
	dma_channel = resources->dma_low_channel + chip;

	/* Wait for the chip to report ready. */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 0;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 1;
	(*d)->cmd.cmd.bits.dec_sem           = 0;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 1;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(0)              ;

	(*d)->cmd.next = (uint32_t)d[1];
	d++;

	/* Enable the BCH block and read. */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__READ;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
	ecc_command  = BV_GPMI_ECCCTRL_ECC_CMD__DECODE_8_BIT;
	buffer_mask  = BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
		       BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 0;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 0;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 6;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode)              |
		BM_GPMI_CTRL0_WORD_LENGTH                             |
		BF_GPMI_CTRL0_CS(chip)                                |
		BF_GPMI_CTRL0_ADDRESS(address)                        |
		BF_GPMI_CTRL0_XFER_COUNT(nfc_geo->page_size_in_bytes) ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC 	         |
		BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)     |
		BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask) ;
	(*d)->cmd.pio_words[3] = nfc_geo->page_size_in_bytes;
	(*d)->cmd.pio_words[4] = payload;
	(*d)->cmd.pio_words[5] = auxiliary;

	(*d)->cmd.next = (uint32_t)d[1];
	d++;

	/* Disable the BCH block */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 0;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 1;
	(*d)->cmd.cmd.bits.dec_sem           = 0;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 3;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode)              |
		BM_GPMI_CTRL0_WORD_LENGTH                             |
		BF_GPMI_CTRL0_CS(chip)                                |
		BF_GPMI_CTRL0_ADDRESS(address)                        |
		BF_GPMI_CTRL0_XFER_COUNT(nfc_geo->page_size_in_bytes) ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;

	(*d)->cmd.next = (uint32_t)d[1];
	d++;

	/* Deassert the NAND lock and interrupt. */
	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 0;
	(*d)->cmd.cmd.bits.wait4end          = 0;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 0;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.next = 0;
	d++;

	//dump_dma_context(this, __func__);

	/* Go! */
	error = gpmi_nfc_dma_go(this, dma_channel);
	if (error)
		printf("[%s] DMA error\n", __func__);

	return error;
}

/* This structure represents the NFC HAL for this version of the hardware. */
struct nfc_hal  gpmi_nfc_hal_v0 = {
	.version                      = 0,
	.description                  = "4-chip GPMI and BCH",
	.max_chip_count               = 4,
	.max_data_setup_cycles        = (BM_GPMI_TIMING0_DATA_SETUP >>
						BP_GPMI_TIMING0_DATA_SETUP),
	.max_data_sample_delay_cycles = (BM_GPMI_CTRL1_RDN_DELAY >>
						BP_GPMI_CTRL1_RDN_DELAY),
	.max_dll_clock_period_in_ns   = 32,
	.init                         = init,
	.set_geometry                 = set_geometry,
	.set_timing                   = set_timing,
	.exit                         = exit,
	.begin                        = begin,
	.end                          = end,
	.clear_bch                    = clear_bch,
	.is_ready                     = is_ready,
	.send_command                 = send_command,
	.send_data                    = send_data,
	.read_data                    = read_data,
	.send_page                    = send_page,
	.read_page                    = read_page,
};
