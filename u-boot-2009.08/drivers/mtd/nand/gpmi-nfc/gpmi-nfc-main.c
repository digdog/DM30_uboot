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

extern struct mtd_info nand_info[];

/*
 * This structure contains the "safe" GPMI timing that should succeed with any
 * NAND Flash device (although, with less-than-optimal performance).
 */

static struct gpmi_nfc_timing  safe_timing = {
	.data_setup_in_ns        = 40,
	.data_hold_in_ns         = 40,
	.address_setup_in_ns     =  5,
	.gpmi_sample_delay_in_ns =  6,
	.tREA_in_ns              = -1,
	.tRLOH_in_ns             = -1,
	.tRHOH_in_ns             = -1,
};

static void gpmi_set_pinmux(void)
{
	uint32_t val;

	// By default, all NAND pins are GPIO's after reset
	// Connect NAND signals by configuring 2-bit pinmux value for each pin
	// Most NAND pins are the default peripheral = 0b00.
	// Startup already checked PINCTRL present bits
	val = readl(HW_PINCTRL_CTRL_ADDR); // out of reset and gate
	val &= ~(0x3<<30);
	writel(val, HW_PINCTRL_CTRL_ADDR);

	// -----------------------------------------------------------------
	// Always power up lower 8 bits of NAND Data bus.
	val = readl(HW_PINCTRL_MUXSEL0_ADDR);
	val &= 0xffff0000; // gpmi data0~7
	writel(val, HW_PINCTRL_MUXSEL0_ADDR);

	// Select the ALE, CLE, WRN, and RDN pins of NAND.
	val = readl(HW_PINCTRL_MUXSEL1_ADDR);
	val &= ~(0x3<<18); // gpmi_rdn
	val &= ~(0x3<<16);  //gpmi wrn
	val &= ~(0x3<<2);  //gpmi ale
	val &= ~(0x3<<0);  //gpmi cle
	writel(val, HW_PINCTRL_MUXSEL1_ADDR);

	// Set the pin drive for the RDN, WRN pin to 12mA.
	val = readl(HW_PINCTRL_DRIVE3_ADDR);
	val &= ~(0x3<<4); // gpmi_rdn
	val |= (0x2<<4);
	val &= ~(0x3<<0); // gpmi_wrn
	val |= (0x2<<0);
	writel(val, HW_PINCTRL_DRIVE3_ADDR);

	// Power up Ready Busy for NAND0
	val = readl(HW_PINCTRL_MUXSEL1_ADDR);
	val &= ~(0x3<<6); // gpmi_rdy0
	writel(val, HW_PINCTRL_MUXSEL1_ADDR);

	// Power up CE0 by setting bit field to b00.
	val = readl(HW_PINCTRL_MUXSEL5_ADDR);
	val &= ~(0x3<<24); // gpmi ce0
	writel(val, HW_PINCTRL_MUXSEL5_ADDR);

	// this is called gpmi_wpn in data spec!
	val = readl(HW_PINCTRL_MUXSEL1_ADDR);
	val &= ~(0x3<<14); // gpmi_wpn
	writel(val, HW_PINCTRL_MUXSEL1_ADDR);
}

extern unsigned imx_set_gpmiclk(unsigned nc, int high);

static int gpmi_nfc_platform_init(unsigned int max_chip_count)
{
	gpmi_set_pinmux();
	return 0;
}

static void gpmi_nfc_platform_exit(unsigned int max_chip_count)
{
}

static const char *gpmi_nfc_partition_source_types[] = { "cmdlinepart", 0 };

static struct gpmi_nfc_platform_data  gpmi_nfc_platform_data = {
	.nfc_version             = 0,
	.boot_rom_version        = 0,
	.clock_name              = "gpmi",
	.platform_init           = gpmi_nfc_platform_init,
	.platform_exit           = gpmi_nfc_platform_exit,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 2,
	.boot_area_size_in_bytes = 20 * SZ_1M,
	.partition_source_types  = gpmi_nfc_partition_source_types,
	.partitions              = 0,
	.partition_count         = 0,
};

/*
 * This array has a pointer to every NFC HAL structure. The probing process will
 * find and install the one that matches the version given by the platform.
 */

static struct nfc_hal  *(nfc_hals[]) = {
	&gpmi_nfc_hal_v0,
	//&gpmi_nfc_hal_v1,
};

/*
 * This array has a pointer to every Boot ROM Helper structure. The probing
 * process will find and install the one that matches the version given by the
 * platform.
 */

static struct boot_rom_helper  *(boot_rom_helpers[]) = {
	&gpmi_nfc_boot_rom_helper_v0,
	//&gpmi_nfc_boot_rom_helper_v1,
};

/**
 * acquire_resources() - Tries to acquire resources.
 *
 * @this:  Per-device data.
 */
static int acquire_resources(struct gpmi_nfc_data *this)
{
	struct resources *resources = &this->resources;

	resources->gpmi_regs = (void *)REGS_GPMI_BASE;
	resources->bch_regs = (void *)REGS_BCH_BASE;
	resources->bch_interrupt = HW_IRQ_BCH;
	resources->dma_low_channel = HW_APBH_DMA_NAND_CS0_CHANNEL;
	resources->dma_high_channel = HW_APBH_DMA_NAND_CS3_CHANNEL;
	resources->dma_interrupt = HW_IRQ_GPMI_DMA;

	return 0;
}

/**
 * set_up_nfc_hal() - Sets up the NFC HAL.
 *
 * @this:  Per-device data.
 */
static int set_up_nfc_hal(struct gpmi_nfc_data *this)
{
	struct nfc_hal *nfc;
	int            error = 0;

	/* Attempt to find an NFC HAL that matches the given version. */
	nfc = nfc_hals[0];
	this->nfc = nfc;
	printf("NFC: Version %u, %s\n", nfc->version, nfc->description);

	/* Initialize the NFC HAL. */
	error = nfc->init(this);
	if (error)
		return error;

	/* Set up safe timing. */
	nfc->set_timing(this, &safe_timing);

	return 0;
}

/**
 * set_up_boot_rom_helper() - Sets up the Boot ROM Helper.
 *
 * @this:  Per-device data.
 */
static int set_up_boot_rom_helper(struct gpmi_nfc_data *this)
{
	struct gpmi_nfc_platform_data  *pdata = this->pdata;
	unsigned int                   i;
	struct boot_rom_helper         *rom;

	/* Attempt to find a Boot ROM Helper that matches the given version. */
	for (i = 0; i < ARRAY_SIZE(boot_rom_helpers); i++) {
		rom = boot_rom_helpers[i];
		if (rom->version == pdata->boot_rom_version) {
			this->rom = rom;
			break;
		}
	}

	/* Check if we found a Boot ROM Helper. */
	if (i >= ARRAY_SIZE(boot_rom_helpers)) {
		pr_err("Unkown Boot ROM version %u\n", pdata->boot_rom_version);
		return -ENXIO;
	}

	//pr_info("Boot ROM: Version %u, %s\n", rom->version, rom->description);
	return 0;
}
/**
 * gpmi_nfc_probe() - Probes for a device and, if possible, takes ownership.
 *
 * @pdev:  A pointer to the platform device data structure.
 */
static int gpmi_nfc_probe(struct nand_chip *nand)
{
	int                            error  = 0;
	struct gpmi_nfc_data           *this  = 0;
	struct gpmi_nfc_platform_data  *pdata = &gpmi_nfc_platform_data;

	/* Allocate memory for the per-device data. */
	this = kzalloc(sizeof(*this), GFP_KERNEL);
	if (!this) {
		printf("Failed to allocate per-device memory\n");
		error = -ENOMEM;
		goto exit_allocate_this;
	}

	/* mil 中的nan_chip结构体改为指针 */
	this->mil.nand = nand;
	this->mil.mtd = &nand_info[0];
	this->pdata = pdata;

	acquire_resources(this);

	if (pdata->platform_init)
		error = pdata->platform_init(pdata->max_chip_count);
	if (error)
		goto exit_platform_init;

	/* Set up the NFC. */
	error = set_up_nfc_hal(this);
	if (error)
		goto exit_nfc_init;

	error = set_up_boot_rom_helper(this);
	if (error)
		goto exit_boot_rom_helper_init;

	/* Initialize the MTD Interface Layer. */
	error = gpmi_nfc_mil_init(this);
	if (error)
		goto exit_mil_init;

	/* Return success. */
	return 0;

	/* Error return paths begin here. */
exit_mil_init:
exit_boot_rom_helper_init:
	if (pdata->platform_exit)
		pdata->platform_exit(pdata->max_chip_count);
exit_platform_init:
	this->nfc->exit(this);
exit_nfc_init:
	kfree(this);
exit_allocate_this:
	return error;
}

/*!
 * This function is called during the driver binding process.
 *
 * @param   pdev  the device structure used to store device specific
 *                information that is used by the suspend, resume and
 *                remove functions
 *
 * @return  The function always returns 0.
 */
int board_nand_init(struct nand_chip *nand)
{
	return gpmi_nfc_probe(nand);
}
