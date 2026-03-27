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
#undef DETAILED_INFO

/**
 * mil_cmd_ctrl - MTD Interface cmd_ctrl()
 *
 * This is the function that we install in the cmd_ctrl function pointer of the
 * owning struct nand_chip. The only functions in the reference implementation
 * that use these functions pointers are cmdfunc and select_chip.
 *
 * In this driver, we implement our own select_chip, so this function will only
 * be called by the reference implementation's cmdfunc. For this reason, we can
 * ignore the chip enable bit and concentrate only on sending bytes to the
 * NAND Flash.
 *
 * @mtd:   The owning MTD.
 * @data:  The value to push onto the data signals.
 * @ctrl:  The values to push onto the control signals.
 */
static void mil_cmd_ctrl(struct mtd_info *mtd, int data, unsigned int ctrl)
{
	struct nand_chip      *nand = mtd->priv;
	struct gpmi_nfc_data  *this = nand->priv;
	struct mil            *mil  = &this->mil;
	struct nfc_hal        *nfc  =  this->nfc;
	int                   error;
#if defined(CONFIG_MTD_DEBUG)
	unsigned int          i;
	char                  display[MIL_COMMAND_BUFFER_SIZE * 5];
#endif

	/*
	 * Every operation begins with a command byte and a series of zero or
	 * more address bytes. These are distinguished by either the Address
	 * Latch Enable (ALE) or Command Latch Enable (CLE) signals being
	 * asserted. When MTD is ready to execute the command, it will deassert
	 * both latch enables.
	 *
	 * Rather than run a separate DMA operation for every single byte, we
	 * queue them up and run a single DMA operation for the entire series
	 * of command and data bytes.
	 */

	if ((ctrl & (NAND_ALE | NAND_CLE))) {
		if (data != NAND_CMD_NONE)
			mil->cmd_virt[mil->command_length++] = data;
		return;
	}

	/*
	 * If control arrives here, MTD has deasserted both the ALE and CLE,
	 * which means it's ready to run an operation. Check if we have any
	 * bytes to send.
	 */

	if (!mil->command_length)
		return;

	/* Hand the command over to the NFC. */

#if defined(CONFIG_MTD_DEBUG)
	display[0] = 0;
	for (i = 0; i < mil->command_length; i++)
		sprintf(display + strlen(display), " 0x%02x",
						mil->cmd_virt[i] & 0xff);
	MTDDEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc cmd_ctrl] command: %s\n", display);
#endif

	error = nfc->send_command(this,
			mil->current_chip, mil->cmd_phys, mil->command_length);

	if (error) {
		printf("[%s] Chip: %u, Error %d\n",
					__func__, mil->current_chip, error);
	}

	/* Reset. */
	mil->command_length = 0;
}

/**
 * mil_dev_ready() - MTD Interface dev_ready()
 *
 * @mtd:   A pointer to the owning MTD.
 */
static int mil_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip      *nand = mtd->priv;
	struct gpmi_nfc_data  *this = nand->priv;
	struct nfc_hal        *nfc  = this->nfc;
	struct mil            *mil  = &this->mil;

	MTDDEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc dev_ready]\n");

	return nfc->is_ready(this, mil->current_chip);
}

/**
 * mil_select_chip() - MTD Interface select_chip()
 *
 * @mtd:   A pointer to the owning MTD.
 * @chip:  The chip number to select, or -1 to select no chip.
 */
static void mil_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip      *nand  = mtd->priv;
	struct gpmi_nfc_data  *this  = nand->priv;
	struct mil            *mil   = &this->mil;
	struct nfc_hal        *nfc   =  this->nfc;

	MTDDEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc select_chip] chip: %d\n", chip);

	/* Figure out what kind of transition this is. */
	if ((mil->current_chip < 0) && (chip >= 0)) {
		clk_gpmi_enable();
		nfc->begin(this);
	} else if ((mil->current_chip >= 0) && (chip < 0)) {
		clk_gpmi_disable();
		nfc->end(this);
	}

	mil->current_chip = chip;
}

/**
 * mil_read_buf() - MTD Interface read_buf().
 *
 * @mtd:  A pointer to the owning MTD.
 * @buf:  The destination buffer.
 * @len:  The number of bytes to read.
 */
static void mil_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip      *nand     = mtd->priv;
	struct gpmi_nfc_data  *this     = nand->priv;
	struct nfc_hal        *nfc      =  this->nfc;
	struct mil            *mil      = &this->mil;
	dma_addr_t            use_phys  = ~0;

	MTDDEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc readbuf] len: %d\n", len);

	use_phys = (dma_addr_t)buf;

	/* Ask the NFC. */
	nfc->read_data(this, mil->current_chip, use_phys, len);
}

/**
 * mil_write_buf() - MTD Interface write_buf().
 *
 * @mtd:  A pointer to the owning MTD.
 * @buf:  The source buffer.
 * @len:  The number of bytes to read.
 */
static void mil_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip      *nand     = mtd->priv;
	struct gpmi_nfc_data  *this     = nand->priv;
	struct nfc_hal        *nfc      =  this->nfc;
	struct mil            *mil      = &this->mil;
	dma_addr_t            use_phys  = ~0;

	MTDDEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc writebuf] len: %d\n", len);

	use_phys = (dma_addr_t)buf;

	/* Ask the NFC. */
	nfc->send_data(this, mil->current_chip, use_phys, len);
}

/**
 * mil_read_byte() - MTD Interface read_byte().
 *
 * @mtd:  A pointer to the owning MTD.
 */
static uint8_t mil_read_byte(struct mtd_info *mtd)
{
	uint8_t  byte;

	MTDDEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc read_byte]\n");

	mil_read_buf(mtd, (uint8_t *) &byte, 1);

	MTDDEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc read_byte]: 0x%02x\n", byte);

	return byte;
}

/**
 * mil_ecc_read_page() - MTD Interface ecc.read_page().
 *
 * @mtd:   A pointer to the owning MTD.
 * @nand:  A pointer to the owning NAND Flash MTD.
 * @buf:   A pointer to the destination buffer.
 */
static int mil_ecc_read_page(struct mtd_info *mtd, struct nand_chip *nand, uint8_t *buf)
{
	struct gpmi_nfc_data    *this    = nand->priv;
	struct nfc_hal          *nfc     =  this->nfc;
	struct nfc_geometry     *nfc_geo = &this->nfc_geometry;
	struct mil              *mil     = &this->mil;
	void                    *payload_virt   =  0;
	dma_addr_t              payload_phys    = ~0;
	void                    *auxiliary_virt =  0;
	dma_addr_t              auxiliary_phys  = ~0;
	unsigned int            i;
	unsigned char           *status;
	unsigned int            failed;
	unsigned int            corrected;
	int                     error = 0;

	MTDDEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc ecc_read_page]\n");

	auxiliary_virt = mil->auxiliary_virt;
	auxiliary_phys = mil->auxiliary_phys;
	payload_virt = (void *)buf;
	payload_phys = (dma_addr_t)buf;

	nfc->clear_bch(this);

	/* Ask the NFC. */
	error = nfc->read_page(this, mil->current_chip, payload_phys, auxiliary_phys);
	if (error) {
		printf("[%s] Error in ECC-based read: %d\n", __func__, error);
		goto exit_nfc;
	}
#if 0
	while (1) {
		if (HW_BCH_CTRL_RD() & BM_BCH_CTRL_COMPLETE_IRQ) {
			//pr_info("BCH completed interrupt\n");
			break;
		}
		if (HW_BCH_CTRL_RD() & BM_BCH_CTRL_BM_ERROR_IRQ) {
			//pr_info("BCH: AHB Bus interface Error\n");
			break;
		}
	}
#endif
	/* Loop over status bytes, accumulating ECC status. */
	failed    = 0;
	corrected = 0;
	status = ((unsigned char *) auxiliary_virt) + nfc_geo->auxiliary_status_offset;

	for (i = 0; i < nfc_geo->ecc_chunk_count; i++, status++) {
		//pr_info(" xxx-%d 0x%02x\n", i, *status);
		if ((*status == 0x00) || (*status == 0xff))
			continue;
		if (*status == 0xfe) {
			failed++;
			continue;
		}
		corrected += *status;
	}

	/* Propagate ECC status to the owning MTD. */
	mtd->ecc_stats.failed    += failed;
	mtd->ecc_stats.corrected += corrected;

	debug("[%s] HW_BCH_STATUS0_RD() %08x\n", __func__, HW_BCH_STATUS0_RD());

	nfc->clear_bch(this);

	/*
	 * It's time to deliver the OOB bytes. See mil_ecc_read_oob() for
	 * details about our policy for delivering the OOB.
	 *
	 * We fill the caller's buffer with set bits, and then copy the block
	 * mark to th caller's buffer. Note that, if block mark swapping was
	 * necessary, it has already been done, so we can rely on the first
	 * byte of the auxiliary buffer to contain the block mark.
	 */
	memset(nand->oob_poi, ~0, mtd->oobsize);
	nand->oob_poi[0] = ((uint8_t *) auxiliary_virt)[0];

exit_nfc:
	return error;
}

/**
 * mil_ecc_write_page() - MTD Interface ecc.write_page().
 *
 * @mtd:   A pointer to the owning MTD.
 * @nand:  A pointer to the owning NAND Flash MTD.
 * @buf:   A pointer to the source buffer.
 */
static void mil_ecc_write_page(struct mtd_info *mtd, struct nand_chip *nand, const uint8_t *buf)
{
	struct gpmi_nfc_data    *this    = nand->priv;
	struct nfc_hal          *nfc     =  this->nfc;
	struct mil              *mil     = &this->mil;
	const void              *payload_virt   =  0;
	dma_addr_t              payload_phys    = ~0;
	const void              *auxiliary_virt =  0;
	dma_addr_t              auxiliary_phys  = ~0;
	int                     error;

	MTDDEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc ecc_write_page]\n");

	payload_phys = (dma_addr_t)buf;
	payload_virt = (void *)buf;
	auxiliary_phys = (dma_addr_t)nand->oob_poi;
	auxiliary_virt = (void *)nand->oob_poi;

	/* Ask the NFC. */
	error = nfc->send_page(this, mil->current_chip, payload_phys, auxiliary_phys);
	if (error)
		printf("[%s] Error in ECC-based write: %d\n", __func__, error);
}

/**
 * mil_ecc_read_oob() - MTD Interface ecc.read_oob().
 *
 * There are several places in this driver where we have to handle the OOB and
 * block marks. This is the function where things are the most complicated, so
 * this is where we try to explain it all. All the other places refer back to
 * here.
 *
 * These are the rules, in order of decreasing importance:
 *
 * 1) Nothing the caller does can be allowed to imperil the block mark, so all
 *    write operations take measures to protect it.
 *
 * 2) In read operations, the first byte of the OOB we return must reflect the
 *    true state of the block mark, no matter where that block mark appears in
 *    the physical page.
 *
 * 3) ECC-based read operations return an OOB full of set bits (since we never
 *    allow ECC-based writes to the OOB, it doesn't matter what ECC-based reads
 *    return).
 *
 * 4) "Raw" read operations return a direct view of the physical bytes in the
 *    page, using the conventional definition of which bytes are data and which
 *    are OOB. This gives the caller a way to see the actual, physical bytes
 *    in the page, without the distortions applied by our ECC engine.
 *
 *
 * What we do for this specific read operation depends on two questions:
 *
 * 1) Are we doing a "raw" read, or an ECC-based read?
 *
 * 2) Are we using block mark swapping or transcription?
 *
 * There are four cases, illustrated by the following Karnaugh map:
 *
 *                    |           Raw           |         ECC-based       |
 *       -------------+-------------------------+-------------------------+
 *                    | Read the conventional   |                         |
 *                    | OOB at the end of the   |                         |
 *       Swapping     | page and return it. It  |                         |
 *                    | contains exactly what   |                         |
 *                    | we want.                | Read the block mark and |
 *       -------------+-------------------------+ return it in a buffer   |
 *                    | Read the conventional   | full of set bits.       |
 *                    | OOB at the end of the   |                         |
 *                    | page and also the block |                         |
 *       Transcribing | mark in the metadata.   |                         |
 *                    | Copy the block mark     |                         |
 *                    | into the first byte of  |                         |
 *                    | the OOB.                |                         |
 *       -------------+-------------------------+-------------------------+
 *
 * Note that we break rule #4 in the Transcribing/Raw case because we're not
 * giving an accurate view of the actual, physical bytes in the page (we're
 * overwriting the block mark). That's OK because it's more important to follow
 * rule #2.
 *
 * It turns out that knowing whether we want an "ECC-based" or "raw" read is not
 * easy. When reading a page, for example, the NAND Flash MTD code calls our
 * ecc.read_page or ecc.read_page_raw function. Thus, the fact that MTD wants an
 * ECC-based or raw view of the page is implicit in which function it calls
 * (there is a similar pair of ECC-based/raw functions for writing).
 *
 * Since MTD assumes the OOB is not covered by ECC, there is no pair of
 * ECC-based/raw functions for reading or or writing the OOB. The fact that the
 * caller wants an ECC-based or raw view of the page is not propagated down to
 * this driver.
 *
 * Since our OOB *is* covered by ECC, we need this information. So, we hook the
 * ecc.read_oob and ecc.write_oob function pointers in the owning
 * struct mtd_info with our own functions. These hook functions set the
 * raw_oob_mode field so that, when control finally arrives here, we'll know
 * what to do.
 *
 * @mtd:     A pointer to the owning MTD.
 * @nand:    A pointer to the owning NAND Flash MTD.
 * @page:    The page number to read.
 * @sndcmd:  Indicates this function should send a command to the chip before
 *           reading the out-of-band bytes. This is only false for small page
 *           chips that support auto-increment.
 */
static int mil_ecc_read_oob(struct mtd_info *mtd, struct nand_chip *nand,
							int page, int sndcmd)
{
	struct gpmi_nfc_data      *this     = nand->priv;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct mil                *mil      = &this->mil;
	struct boot_rom_helper    *rom      =  this->rom;
	int                       block_mark_column;

	MTDDEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc ecc_read_oob] "
		"page: 0x%06x, sndcmd: %s\n", page, sndcmd ? "Yes" : "No");

	/*
	 * First, fill in the OOB buffer. If we're doing a raw read, we need to
	 * get the bytes from the physical page. If we're not doing a raw read,
	 * we need to fill the buffer with set bits.
	 */
	if (mil->raw_oob_mode) {
		/*
		 * If control arrives here, we're doing a "raw" read. Send the
		 * command to read the conventional OOB.
		 */
		nand->cmdfunc(mtd, NAND_CMD_READ0, physical->page_data_size_in_bytes, page);

		/* Read out the conventional OOB. */
		nand->read_buf(mtd, nand->oob_poi, mtd->oobsize);
	} else {
		/*
		 * If control arrives here, we're not doing a "raw" read. Fill
		 * the OOB buffer with set bits.
		 */
		memset(nand->oob_poi, ~0, mtd->oobsize);
	}

	/*
	 * Now, we want to make sure the block mark is correct. In the
	 * Swapping/Raw case, we already have it. Otherwise, we need to
	 * explicitly read it.
	 */

	if (!(rom->swap_block_mark && mil->raw_oob_mode)) {
		/* First, figure out where the block mark is. */
		if (rom->swap_block_mark)
			block_mark_column = physical->page_data_size_in_bytes;
		else
			block_mark_column = 0;

		/* Send the command to read the block mark. */
		nand->cmdfunc(mtd, NAND_CMD_READ0, block_mark_column, page);

		/* Read the block mark into the first byte of the OOB buffer. */
		nand->oob_poi[0] = nand->read_byte(mtd);
	}

	/*
	 * Return true, indicating that the next call to this function must send
	 * a command.
	 */
	return true;
}

/**
 * mil_ecc_write_oob() - MTD Interface ecc.write_oob().
 *
 * @mtd:   A pointer to the owning MTD.
 * @nand:  A pointer to the owning NAND Flash MTD.
 * @page:  The page number to write.
 */
static int mil_ecc_write_oob(struct mtd_info *mtd,
					struct nand_chip *nand, int page)
{
	struct gpmi_nfc_data      *this     = nand->priv;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct mil                *mil      = &this->mil;
	struct boot_rom_helper    *rom      =  this->rom;
	uint8_t                   block_mark = 0;
	int                       block_mark_column;
	int                       status;
	int                       error = 0;

	MTDDEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc ecc_write_oob] page: 0x%06x\n", page);

	/*
	 * There are fundamental incompatibilities between the i.MX GPMI NFC and
	 * the NAND Flash MTD model that make it essentially impossible to write
	 * the out-of-band bytes.
	 *
	 * We permit *ONE* exception. If the *intent* of writing the OOB is to
	 * mark a block bad, we can do that.
	 */
	if (!mil->marking_a_bad_block) {
		printf("This driver doesn't support writing the OOB\n");
		error = -EIO;
		goto exit;
	}

	/*
	 * If control arrives here, we're marking a block bad. First, figure out
	 * where the block mark is.
	 *
	 * If we're using swapping, the block mark is in the conventional
	 * location. Otherwise, we're using transcription, and the block mark
	 * appears in the first byte of the page.
	 */
	if (rom->swap_block_mark)
		block_mark_column = physical->page_data_size_in_bytes;
	else
		block_mark_column = 0;

	/* Write the block mark. */
	nand->cmdfunc(mtd, NAND_CMD_SEQIN, block_mark_column, page);
	nand->write_buf(mtd, &block_mark, 1);
	nand->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	status = nand->waitfunc(mtd, nand);

	/* Check if it worked. */
	if (status & NAND_STATUS_FAIL)
		error = -EIO;

	/* Return. */
exit:
	return error;
}

/**
 * mil_block_bad - Claims all blocks are good.
 *
 * In principle, this function is *only* called when the NAND Flash MTD system
 * isn't allowed to keep an in-memory bad block table, so it is forced to ask
 * the driver for bad block information.
 *
 * In fact, we permit the NAND Flash MTD system to have an in-memory BBT, so
 * this function is *only* called when we take it away.
 *
 * We take away the in-memory BBT when the user sets the "ignorebad" parameter,
 * which indicates that all blocks should be reported good.
 *
 * Thus, this function is only called when we want *all* blocks to look good,
 * so it *always* return success.
 *
 * @mtd:      Ignored.
 * @ofs:      Ignored.
 * @getchip:  Ignored.
 */
static int mil_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	return 0;
}

static int mil_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip         *nand = mtd->priv;
	struct gpmi_nfc_data     *this = nand->priv;
	struct mil               *mil  = &this->mil;
	int 			 error;

	mil->marking_a_bad_block = true;
	error = nand->block_markbad(mtd, ofs);
	mil->marking_a_bad_block = false;
	if (error)
		printf("Failed to mark block bad with " "error %d\n", error);

	return error;
}

/**
 * mil_set_physical_geometry() - Set up the physical medium geometry.
 *
 * This function retrieves the physical geometry information discovered by
 * nand_scan(), corrects it, and records it in the per-device data structure.
 *
 * @this:  Per-device data.
 */
static int mil_set_physical_geometry(struct gpmi_nfc_data  *this)
{
	struct mil                *mil      = &this->mil;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct nand_chip          *nand     = mil->nand;
	struct nand_device_info   *info     = &this->device_info;
	unsigned int              block_size_in_pages;
	unsigned int              chip_size_in_blocks;
	unsigned int              chip_size_in_pages;
	uint64_t                  medium_size_in_bytes;

	/*
	 * Record the number of physical chips that MTD found.
	 */
	physical->chip_count = nand->numchips;

	/*
	 * We know the total size of a page. We need to break that down into the
	 * data size and OOB size. The data size is the largest power of two
	 * that will fit in the given page size. The OOB size is what's left
	 * over.
	 */
	physical->page_data_size_in_bytes = 1 << (fls(info->page_total_size_in_bytes) - 1);
	physical->page_oob_size_in_bytes = info->page_total_size_in_bytes - physical->page_data_size_in_bytes;

	/*
	 * Now that we know the page data size, we can multiply this by the
	 * number of pages in a block to compute the block size.
	 */
	physical->block_size_in_bytes = physical->page_data_size_in_bytes * info->block_size_in_pages;

	/* Get the chip size. */
	physical->chip_size_in_bytes = info->chip_size_in_bytes;

	/* Compute some interesting facts. */
	block_size_in_pages  = physical->block_size_in_bytes >> (fls(physical->page_data_size_in_bytes) - 1);
	chip_size_in_pages   = physical->chip_size_in_bytes >> (fls(physical->page_data_size_in_bytes) - 1);
	chip_size_in_blocks  = physical->chip_size_in_bytes >> (fls(physical->block_size_in_bytes) - 1);
	medium_size_in_bytes = physical->chip_size_in_bytes * physical->chip_count;

	/* Report. */
#if defined(DETAILED_INFO)
	pr_info("-----------------\n");
	pr_info("Physical Geometry\n");
	pr_info("-----------------\n");
	pr_info("Chip Count             : %d\n", physical->chip_count);
	pr_info("Page Data Size in Bytes: %u (0x%x)\n",
			physical->page_data_size_in_bytes,
			physical->page_data_size_in_bytes);
	pr_info("Page OOB Size in Bytes : %u\n",
			physical->page_oob_size_in_bytes);
	pr_info("Block Size in Bytes    : %u (0x%x)\n",
			physical->block_size_in_bytes,
			physical->block_size_in_bytes);
	pr_info("Block Size in Pages    : %u (0x%x)\n",
			block_size_in_pages,
			block_size_in_pages);
	pr_info("Chip Size in Bytes     : %llu (0x%llx)\n",
			physical->chip_size_in_bytes,
			physical->chip_size_in_bytes);
	pr_info("Chip Size in Pages     : %u (0x%x)\n",
			chip_size_in_pages, chip_size_in_pages);
	pr_info("Chip Size in Blocks    : %u (0x%x)\n",
			chip_size_in_blocks, chip_size_in_blocks);
	pr_info("Medium Size in Bytes   : %llu (0x%llx)\n",
			medium_size_in_bytes, medium_size_in_bytes);
#endif

	return 0;
}

/**
 * mil_set_nfc_geometry() - Set up the NFC geometry.
 *
 * This function calls the NFC HAL to select an NFC geometry that is compatible
 * with the medium's physical geometry.
 *
 * @this:  Per-device data.
 */
static int mil_set_nfc_geometry(struct gpmi_nfc_data  *this)
{
	struct nfc_hal       *nfc =  this->nfc;
#if defined(DETAILED_INFO)
	struct nfc_geometry  *geo = &this->nfc_geometry;
#endif
	/* Set the NFC geometry. */
	if (nfc->set_geometry(this))
		return !0;

	/* Report. */
#if defined(DETAILED_INFO)
	pr_info("------------\n");
	pr_info("NFC Geometry\n");
	pr_info("------------\n");
	pr_info("ECC Algorithm          : %s\n", geo->ecc_algorithm);
	pr_info("ECC Strength           : %u\n", geo->ecc_strength);
	pr_info("Page Size in Bytes     : %u\n", geo->page_size_in_bytes);
	pr_info("Metadata Size in Bytes : %u\n", geo->metadata_size_in_bytes);
	pr_info("ECC Chunk Size in Bytes: %u\n", geo->ecc_chunk_size_in_bytes);
	pr_info("ECC Chunk Count        : %u\n", geo->ecc_chunk_count);
	pr_info("Payload Size in Bytes  : %u\n", geo->payload_size_in_bytes);
	pr_info("Auxiliary Size in Bytes: %u\n", geo->auxiliary_size_in_bytes);
	pr_info("Auxiliary Status Offset: %u\n", geo->auxiliary_status_offset);
	pr_info("Block Mark Byte Offset : %u\n", geo->block_mark_byte_offset);
	pr_info("Block Mark Bit Offset  : %u\n", geo->block_mark_bit_offset);
#endif

	return 0;
}

/**
 * mil_set_boot_rom_helper_geometry() - Set up the Boot ROM Helper geometry.
 *
 * @this:  Per-device data.
 */
static int mil_set_boot_rom_helper_geometry(struct gpmi_nfc_data *this)
{
	struct boot_rom_helper    *rom =  this->rom;
#if defined(DETAILED_INFO)
	struct boot_rom_geometry  *geo = &this->rom_geometry;
#endif

	/* Set the Boot ROM Helper geometry. */
	if (rom->set_geometry(this))
		return !0;

#if defined(DETAILED_INFO)
	pr_info("-----------------\n");
	pr_info("Boot ROM Geometry\n");
	pr_info("-----------------\n");
	pr_info("Boot Area Count            : %u\n", geo->boot_area_count);
	pr_info("Boot Area Size in Bytes    : %u (0x%x)\n",
		geo->boot_area_size_in_bytes, geo->boot_area_size_in_bytes);
	pr_info("Stride Size in Pages       : %u\n", geo->stride_size_in_pages);
	pr_info("Search Area Stride Exponent: %u\n",
					geo->search_area_stride_exponent);
#endif

	return 0;
}

/**
 * mil_set_mtd_geometry() - Set up the MTD geometry.
 *
 * This function adjusts the owning MTD data structures to match the logical
 * geometry we've chosen.
 *
 * @this:  Per-device data.
 */
static int mil_set_mtd_geometry(struct gpmi_nfc_data *this)
{
	struct physical_geometry  *physical = &this->physical_geometry;
	struct mil                *mil      = &this->mil;
	struct nand_ecclayout     *layout   = &mil->oob_layout;
	struct nand_chip          *nand     = mil->nand;
	struct mtd_info           *mtd      = mil->mtd;

	/* Configure the struct nand_ecclayout. */

	layout->eccbytes          = 0;
	layout->oobavail          = physical->page_oob_size_in_bytes;
	layout->oobfree[0].offset = 0;
	layout->oobfree[0].length = physical->page_oob_size_in_bytes;

	/* Configure the struct mtd_info. */

	mtd->size        = nand->numchips * physical->chip_size_in_bytes;
	mtd->erasesize   = physical->block_size_in_bytes;
	mtd->writesize   = physical->page_data_size_in_bytes;
	mtd->ecclayout   = layout;
	mtd->oobavail    = mtd->ecclayout->oobavail;
	mtd->oobsize     = mtd->ecclayout->oobavail + mtd->ecclayout->eccbytes;
	mtd->subpage_sft = 0; /* We don't support sub-page writing. */

	/* Configure the struct nand_chip. */
	nand->chipsize         = physical->chip_size_in_bytes;
	nand->page_shift       = ffs(mtd->writesize) - 1;
	nand->pagemask         = (nand->chipsize >> nand->page_shift) - 1;
	nand->subpagesize      = mtd->writesize >> mtd->subpage_sft;
	nand->phys_erase_shift = ffs(mtd->erasesize) - 1;
	nand->bbt_erase_shift  = nand->phys_erase_shift;
	nand->oob_poi          = nand->buffers->databuf + mtd->writesize;
	nand->ecc.layout       = layout;

	if (nand->chipsize & 0xffffffff)
		nand->chip_shift = ffs((unsigned) nand->chipsize) - 1;
	else {
		pr_err("%s, error chip shift\n", __func__);
		//nand->chip_shift = ffs((unsigned) (nand->chipsize >> 32)) + 32 - 1;
	}

	return 0;
}

/**
 * mil_set_geometry() - Set up the medium geometry.
 *
 * @this:  Per-device data.
 */
static int mil_set_geometry(struct gpmi_nfc_data  *this)
{
	struct nfc_geometry  *nfc_geo  = &this->nfc_geometry;
	struct mil           *mil      = &this->mil;

	/* Set up the various layers of geometry, in this specific order. */

	if (mil_set_physical_geometry(this))
		return -ENXIO;

	if (mil_set_nfc_geometry(this))
		return -ENXIO;

	if (mil_set_boot_rom_helper_geometry(this))
		return -ENXIO;

	if (mil_set_mtd_geometry(this))
		return -ENXIO;

	/*
	 * Allocate the page buffer.
	 *
	 * Both the payload buffer and the auxiliary buffer must appear on
	 * 32-bit boundaries. We presume the size of the payload buffer is a
	 * power of two and is much larger than four, which guarantees the
	 * auxiliary buffer will appear on a 32-bit boundary.
	 */

	mil->page_buffer_size = nfc_geo->payload_size_in_bytes + nfc_geo->auxiliary_size_in_bytes;
	mil->page_buffer_virt = kmalloc(mil->page_buffer_size, GFP_KERNEL);
	mil->page_buffer_phys = (uint32_t)mil->page_buffer_virt;
	if (!mil->page_buffer_virt)
		return -ENOMEM;

	/* Slice up the page buffer. */
	mil->payload_virt = mil->page_buffer_virt;
	mil->payload_phys = mil->page_buffer_phys;

	mil->auxiliary_virt = ((char *) mil->payload_virt) + nfc_geo->payload_size_in_bytes;
	mil->auxiliary_phys = mil->payload_phys + nfc_geo->payload_size_in_bytes;

	/* Return success. */
	return 0;
}

/**
 * mil_pre_bbt_scan() - Prepare for the BBT scan.
 *
 * @this:  Per-device data.
 */
static int __attribute__((unused)) mil_pre_bbt_scan(struct gpmi_nfc_data  *this)
{
	struct physical_geometry  *physical = &this->physical_geometry;
	struct boot_rom_helper    *rom      =  this->rom;
	struct mil                *mil      = &this->mil;
	struct nand_chip          *nand     = mil->nand;
	struct mtd_info           *mtd      = mil->mtd;
	unsigned int              block_count;
	unsigned int              block;
	int                       chip;
	int                       page;
	loff_t                    byte;
	uint8_t                   block_mark;
	int                       error;

	/*
	 * Check if we can use block mark swapping, which enables us to leave
	 * the block marks where they are. If so, we don't need to do anything
	 * at all.
	 */
	if (rom->swap_block_mark)
		return 0;

	/*
	 * If control arrives here, we can't use block mark swapping, which
	 * means we're forced to use transcription. First, scan for the
	 * transcription stamp. If we find it, then we don't have to do
	 * anything -- the block marks are already transcribed.
	 */
	if (rom->check_transcription_stamp(this))
		return 0;

	/*
	 * If control arrives here, we couldn't find a transcription stamp, so
	 * so we presume the block marks are in the conventional location.
	 */
	pr_info("Transcribing bad block marks...\n");

	/* Compute the number of blocks in the entire medium. */
	block_count = physical->chip_size_in_bytes >> nand->phys_erase_shift;

	/*
	 * Loop over all the blocks in the medium, transcribing block marks as
	 * we go.
	 */
	for (block = 0; block < block_count; block++) {
		/*
		 * Compute the chip, page and byte addresses for this block's
		 * conventional mark.
		 */
		chip = block >> (nand->chip_shift - nand->phys_erase_shift);
		page = block << (nand->phys_erase_shift - nand->page_shift);
		byte = block <<  nand->phys_erase_shift;

		/* Select the chip. */
		nand->select_chip(mtd, chip);

		/* Send the command to read the conventional block mark. */
		nand->cmdfunc(mtd, NAND_CMD_READ0, physical->page_data_size_in_bytes, page);

		/* Read the conventional block mark. */
		block_mark = nand->read_byte(mtd);

		/*
		 * Check if the block is marked bad. If so, we need to mark it
		 * again, but this time the result will be a mark in the
		 * location where we transcribe block marks.
		 *
		 * Notice that we have to explicitly set the marking_a_bad_block
		 * member before we call through the block_markbad function
		 * pointer in the owning struct nand_chip. If we could call
		 * though the block_markbad function pointer in the owning
		 * struct mtd_info, which we have hooked, then this would be
		 * taken care of for us. Unfortunately, we can't because that
		 * higher-level code path will do things like consulting the
		 * in-memory bad block table -- which doesn't even exist yet!
		 * So, we have to call at a lower level and handle some details
		 * ourselves.
		 */

		if (block_mark != 0xff) {
			pr_info("Transcribing mark in block %u\n", block);
			mil->marking_a_bad_block = true;
			error = nand->block_markbad(mtd, byte);
			mil->marking_a_bad_block = false;
			if (error)
				printf("Failed to mark block bad with " "error %d\n", error);
		}

		/* Deselect the chip. */
		nand->select_chip(mtd, -1);
	}

	/* Write the stamp that indicates we've transcribed the block marks. */
	//rom->write_transcription_stamp(this);

	/* Return success. */
	return 0;
}

/**
 * mil_scan_bbt() - MTD Interface scan_bbt().
 *
 * The HIL calls this function once, when it initializes the NAND Flash MTD.
 *
 * Nominally, the purpose of this function is to look for or create the bad
 * block table. In fact, since the HIL calls this function at the very end of
 * the initialization process started by nand_scan(), and the HIL doesn't have a
 * more formal mechanism, everyone "hooks" this function to continue the
 * initialization process.
 *
 * At this point, the physical NAND Flash chips have been identified and
 * counted, so we know the physical geometry. This enables us to make some
 * important configuration decisions.
 *
 * The return value of this function propogates directly back to this driver's
 * call to nand_scan(). Anything other than zero will cause this driver to
 * tear everything down and declare failure.
 *
 * @mtd:  A pointer to the owning MTD.
 */
static int mil_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip         *nand = mtd->priv;
	struct gpmi_nfc_data     *this = nand->priv;
	struct nfc_hal           *nfc  =  this->nfc;
	struct mil               *mil  = &this->mil;
	int                      saved_chip_number;
	uint8_t                  id_bytes[NAND_DEVICE_ID_BYTE_COUNT];
	struct nand_device_info  *info;
	struct gpmi_nfc_timing   timing;
	int                      error;

	MTDDEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc scan_bbt] \n");

	/*
	 * Tell MTD users that the out-of-band area can't be written.
	 *
	 * This flag is not part of the standard kernel source tree. It comes
	 * from a patch that touches both MTD and JFFS2.
	 *
	 * The problem is that, without this patch, JFFS2 believes it can write
	 * the data area and the out-of-band area separately. This is wrong for
	 * two reasons:
	 *
	 *     1)  Our NFC distributes out-of-band bytes throughout the page,
	 *         intermingled with the data, and covered by the same ECC.
	 *         Thus, it's not possible to write the out-of-band bytes and
	 *         data bytes separately.
	 *
	 *     2)  Large page (MLC) Flash chips don't support partial page
	 *         writes. You must write the entire page at a time. Thus, even
	 *         if our NFC didn't force you to write out-of-band and data
	 *         bytes together, it would *still* be a bad idea to do
	 *         otherwise.
	 */

//	mtd->flags &= ~MTD_OOB_WRITEABLE;

	/*
	 * MTD identified the attached NAND Flash devices, but we have a much
	 * better database that we want to consult. First, we need to gather all
	 * the ID bytes from the first chip (MTD only read the first two).
	 */

	saved_chip_number = mil->current_chip;
	nand->select_chip(mtd, 0);

	nand->cmdfunc(mtd, NAND_CMD_READID, 0, -1);
	nand->read_buf(mtd, id_bytes, NAND_DEVICE_ID_BYTE_COUNT);

	nand->select_chip(mtd, saved_chip_number);

	/* Look up this device in our database. */
	info = nand_device_get_info(id_bytes);

	/* Check if we understand this device. */
	if (!info) {
		pr_err("Unrecognized NAND Flash device.\n");
		return !0;
	}

	/* Display the information we discovered. */

	#if defined(DETAILED_INFO)
	pr_info("-----------------------------\n");
	pr_info("NAND Flash Device Information\n");
	pr_info("-----------------------------\n");
	nand_device_print_info(info);
	#endif

	/*
	 * Copy the device info into the per-device data. We can't just keep
	 * the pointer because that storage is reclaimed after initialization.
	 * Notice that we null out the pointer to the device description, which
	 * will also be reclaimed.
	 */
	this->device_info = *info;
	this->device_info.description = 0;

	/* Set up geometry. */
	error = mil_set_geometry(this);
	if (error)
		return error;

	/* Set up timing. */
	timing.data_setup_in_ns        = info->data_setup_in_ns;
	timing.data_hold_in_ns         = info->data_hold_in_ns;
	timing.address_setup_in_ns     = info->address_setup_in_ns;
	timing.gpmi_sample_delay_in_ns = info->gpmi_sample_delay_in_ns;
	timing.tREA_in_ns              = info->tREA_in_ns;
	timing.tRLOH_in_ns             = info->tRLOH_in_ns;
	timing.tRHOH_in_ns             = info->tRHOH_in_ns;

	error = nfc->set_timing(this, &timing);
	if (error)
		return error;

	/* Prepare for the BBT scan. */
#if 0
	error = mil_pre_bbt_scan(this);
	if (error)
		return error;
#endif

	/* We use the reference implementation for bad block management. */
	error = nand_default_bbt(mtd);
	if (error)
		return error;

	return 0;
}

/**
 * gpmi_nfc_mil_init() - Initializes the MTD Interface Layer.
 *
 * @this:  Per-device data.
 */
int gpmi_nfc_mil_init(struct gpmi_nfc_data *this)
{
	struct mil                     *mil   = &this->mil;
	struct nand_chip               *nand  = mil->nand;
	struct mtd_info 	       *mtd   = mil->mtd;
	static struct nand_ecclayout   fake_ecc_layout;
	int                            error = 0;

	/* Initialize MIL data. */
	mil->current_chip   = -1;
	mil->command_length =  0;

	mil->page_buffer_virt =  0;
	mil->page_buffer_phys = ~0;
	mil->page_buffer_size =  0;

	/* Initialize the MTD data structures. */
	nand->priv = this;

	/*
	 * Signal Control
	 */
	nand->cmd_ctrl = mil_cmd_ctrl;

	/*
	 * Chip Control
	 *
	 * We rely on the reference implementations of:
	 *     - cmdfunc
	 *     - waitfunc
	 */
	nand->dev_ready   = mil_dev_ready;
	nand->select_chip = mil_select_chip;

	/*
	 * Low-level I/O
	 *
	 * We don't support a 16-bit NAND Flash bus, so we don't implement
	 * read_word.
	 *
	 * We rely on the reference implentation of verify_buf.
	 */
	nand->read_byte = mil_read_byte;
	nand->read_buf  = mil_read_buf;
	nand->write_buf = mil_write_buf;

	/*
	 * ECC Control
	 *
	 * None of these functions are necessary for us:
	 *     - ecc.hwctl
	 *     - ecc.calculate
	 *     - ecc.correct
	 */

	/*
	 * ECC-aware I/O
	 *
	 * We rely on the reference implementations of:
	 *     - ecc.read_page_raw
	 *     - ecc.write_page_raw
	 */
	nand->ecc.read_page  = mil_ecc_read_page;
	nand->ecc.write_page = mil_ecc_write_page;

	/*
	 * High-level I/O
	 *
	 * We rely on the reference implementations of:
	 *     - write_page
	 *     - erase_cmd
	 */
	nand->ecc.read_oob  = mil_ecc_read_oob;
	nand->ecc.write_oob = mil_ecc_write_oob;

	/*
	 * Bad Block Management
	 *
	 * We rely on the reference implementations of:
	 *     - block_bad
	 *     - block_markbad
	 */
	nand->block_bad = mil_block_bad;
	nand->scan_bbt  = mil_scan_bbt;

	mtd->block_markbad = mil_block_markbad;

	/*
	 * Error Recovery Functions
	 *
	 * We don't fill in the errstat function pointer because it's optional
	 * and we don't have a need for it.
	 */

	/*
	 * Set up NAND Flash options. Specifically:
	 *
	 *     - Disallow partial page writes.
	 */
	nand->options |= NAND_NO_SUBPAGE_WRITE;

	/*
	 * Tell the NAND Flash MTD system that we'll be handling ECC with our
	 * own hardware. It turns out that we still have to fill in the ECC size
	 * because the MTD code will divide by it -- even though it doesn't
	 * actually care.
	 */
	nand->ecc.mode = NAND_ECC_HW;
	nand->ecc.size = 1;

	/*
	 * Install a "fake" ECC layout.
	 *
	 * We'll be calling nand_scan() to do the final MTD setup. If we haven't
	 * already chosen an ECC layout, then nand_scan() will choose one based
	 * on the part geometry it discovers. Unfortunately, it doesn't make
	 * good choices. It would be best if we could install the correct ECC
	 * layout now, before we call nand_scan(). We can't do that because we
	 * don't know the medium geometry yet. Here, we install a "fake" ECC
	 * layout just to stop nand_scan() from trying to pick one for itself.
	 * Later, when we know the medium geometry, we'll install the correct
	 * one.
	 *
	 * Of course, this tactic depends critically on the MTD code not doing
	 * an I/O operation that depends on the ECC layout being sensible. This
	 * is in fact the case.
	 */
	memset(&fake_ecc_layout, 0, sizeof(fake_ecc_layout));
	nand->ecc.layout = &fake_ecc_layout;

	/* Allocate a command buffer. */
	mil->cmd_virt = kmalloc(MIL_COMMAND_BUFFER_SIZE, GFP_KERNEL);
	mil->cmd_phys = (uint32_t)mil->cmd_virt;
	if (!mil->cmd_virt)
		goto exit_cmd_allocation;

	/* Return success. */
	return 0;

	/* Control arrives here if something went wrong. */
exit_cmd_allocation:
	return error;
}
