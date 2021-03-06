/*
 * Copyright (C) 2009 David Brownell
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <common.h>
#include <nand.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/emif_defs.h>
#include <asm/arch/nand_defs.h>
#include "../common/misc.h"
#include <net.h>
#include <netdev.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * With the DM355 EVM, u-boot is *always* a third stage loader,
 * unless a JTAG debugger handles the first two stages:
 *
 *   - 1st stage is ROM Boot Loader (RBL), which searches for a
 *     second stage loader in one of three places based on SW7:
 *     NAND (with MMC/SD fallback), MMC/SD, or UART.
 *
 *   - 2nd stage is User Boot Loader (UBL), using at most 30KB
 *     of on-chip SRAM, responsible for lowlevel init, and for
 *     loading the third stage loader into DRAM.
 *
 *   - 3rd stage, that's us!
 */

int board_init(void)
{
	gd->bd->bi_arch_number = MACH_TYPE_DAVINCI_DM355_EVM;
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	/* We expect the UBL to have handled "lowlevel init", which
	 * involves setting up at least:
	 *  - clocks
	 *      + PLL1 (for ARM and peripherals) and PLL2 (for DDR)
	 *      + clock divisors for those PLLs
	 *      + LPSC_DDR module enabled
	 *      + LPSC_TIMER0 module (still) enabled
	 *  - EMIF
	 *      + DDR init and timings
	 *      + AEMIF timings (for NAND and DM9000)
	 *  - pinmux
	 *
	 * Some of that is repeated here, mostly as a precaution.
	 */

	/* AEMIF:  Some "address" lines are available as GPIOs.  A3..A13
	 * could be too if we used A12 as a GPIO during NAND chipselect
	 * (and Linux did too), letting us control the LED on A7/GPIO61.
	 */
	REG(PINMUX2) = 0x0c08;

	/* UART0 may still be in SyncReset if we didn't boot from UART */
	davinci_enable_uart0();

	/* EDMA may be in SyncReset too; turn it on, Linux won't (yet) */
	lpsc_on(DAVINCI_LPSC_TPCC);
	lpsc_on(DAVINCI_LPSC_TPTC0);
	lpsc_on(DAVINCI_LPSC_TPTC1);

	return 0;
}

#ifdef CONFIG_DRIVER_DM9000
int board_eth_init(bd_t *bis)
{
	return dm9000_initialize(bis);
}
#endif

#ifdef CONFIG_NAND_DAVINCI

static void nand_dm355evm_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip	*this = mtd->priv;
	u32			wbase = (u32) this->IO_ADDR_W;
	u32			rbase = (u32) this->IO_ADDR_R;

	if (chip == 1) {
		__set_bit(14, &wbase);
		__set_bit(14, &rbase);
	} else {
		__clear_bit(14, &wbase);
		__clear_bit(14, &rbase);
	}
	this->IO_ADDR_W = (void *)wbase;
	this->IO_ADDR_R = (void *)rbase;
}

int board_nand_init(struct nand_chip *nand)
{
	davinci_nand_init(nand);
	nand->select_chip = nand_dm355evm_select_chip;
	return 0;
}

#endif
