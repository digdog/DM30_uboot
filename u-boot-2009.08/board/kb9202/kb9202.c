/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
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

/*
 * Adapted for KwikByte KB920x board from at91rm9200dk.c: 22APR2005
 */

#include <common.h>
#include <asm/arch/AT91RM9200.h>
#include <at91rm9200_net.h>
#include <lxt971a.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * Miscelaneous platform dependent initialisations
 */

void lowlevel_init(void) {
	/* Required by assembly functions - do nothing	*/
}

int board_init (void)
{
	/* Enable Ctrlc */
	console_init_f ();

	/* memory and cpu-speed are setup before relocation */
	/* so we do _nothing_ here */

	gd->bd->bi_arch_number = MACH_TYPE_KB9200;

	/* adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	return 0;
}

int dram_init (void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_SIZE;
	return 0;
}

#ifdef CONFIG_DRIVER_ETHER
#if defined(CONFIG_CMD_NET)

unsigned int lxt972_IsPhyConnected (AT91PS_EMAC p_mac);
UCHAR lxt972_GetLinkSpeed (AT91PS_EMAC p_mac);
UCHAR lxt972_InitPhy (AT91PS_EMAC p_mac);
UCHAR lxt972_AutoNegotiate (AT91PS_EMAC p_mac, int *status);

/*
 * Name:
 *	at91rm9200_GetPhyInterface
 * Description:
 *	Initialise the interface functions to the PHY
 * Arguments:
 *	None
 * Return value:
 *	None
 */
void at91rm9200_GetPhyInterface(AT91PS_PhyOps p_phyops)
{
	p_phyops->Init = lxt972_InitPhy;
	p_phyops->IsPhyConnected = lxt972_IsPhyConnected;
	p_phyops->GetLinkSpeed = lxt972_GetLinkSpeed;
	p_phyops->AutoNegotiate = lxt972_AutoNegotiate;
}

#endif
#endif	/* CONFIG_DRIVER_ETHER */
