/*
 * (C) Copyright 2001
 * Kyle Harris, Nexus Technologies, Inc. kharris@nexus-tech.net
 *
 * (C) Copyright 2001
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * Modified for the MP2USB by (C) Copyright 2005 Eric Benard
 * ebenard@eukrea.com
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

#include <common.h>
#include <linux/byteorder/swab.h>

#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define PHYS_FLASH_SECT_SIZE	0x00020000 /* 128 KB sectors (x1) */

flash_info_t flash_info[CONFIG_SYS_MAX_FLASH_BANKS];	/* info for FLASH chips */

#define FLASH_PORT_WIDTH	ushort
#define FLASH_PORT_WIDTHV	vu_short
#define SWAP(x)			__swab16(x)

#define FPW			FLASH_PORT_WIDTH
#define FPWV			FLASH_PORT_WIDTHV

#define mb() __asm__ __volatile__ ("" : : : "memory")

/* Intel-compatible flash commands */
#define INTEL_PROGRAM	0x00100010
#define INTEL_ERASE	0x00200020
#define INTEL_PROG	0x00400040
#define INTEL_CLEAR	0x00500050
#define INTEL_LOCKBIT	0x00600060
#define INTEL_PROTECT	0x00010001
#define INTEL_STATUS	0x00700070
#define INTEL_READID	0x00900090
#define INTEL_SUSPEND	0x00B000B0
#define INTEL_CONFIRM	0x00D000D0
#define INTEL_RESET	0xFFFFFFFF

/* Intel-compatible flash status bits */
#define INTEL_FINISHED	0x00800080
#define INTEL_OK	0x00800080

/*-----------------------------------------------------------------------
 * Functions
 */
static ulong flash_get_size (FPW *addr, flash_info_t *info);
static int write_data (flash_info_t *info, ulong dest, FPW data);
static void flash_get_offsets (ulong base, flash_info_t *info);
void inline spin_wheel (void);

/*-----------------------------------------------------------------------
 */

unsigned long flash_init (void)
{
	int i;
	ulong size = 0;

	for (i = 0; i < CONFIG_SYS_MAX_FLASH_BANKS; i++) {
		switch (i) {
		case 0:
			flash_get_size ((FPW *) PHYS_FLASH_1, &flash_info[i]);
			flash_get_offsets (PHYS_FLASH_1, &flash_info[i]);
			break;
		default:
			panic ("configured too many flash banks!\n");
			break;
		}
		size += flash_info[i].size;
	}

	/* Protect monitor and environment sectors
	 */
	flash_protect ( FLAG_PROTECT_SET,
			CONFIG_SYS_FLASH_BASE,
			CONFIG_SYS_FLASH_BASE + monitor_flash_len - 1,
			&flash_info[0] );

	flash_protect ( FLAG_PROTECT_SET,
			CONFIG_ENV_ADDR,
			CONFIG_ENV_ADDR + CONFIG_ENV_SIZE - 1, &flash_info[0] );

	return size;
}

/*-----------------------------------------------------------------------
 */
static void flash_get_offsets (ulong base, flash_info_t *info)
{
	int i;

	if (info->flash_id == FLASH_UNKNOWN) {
		return;
	}

	if ((info->flash_id & FLASH_VENDMASK) == FLASH_MAN_INTEL) {
		for (i = 0; i < info->sector_count; i++) {
			info->start[i] = base + (i * PHYS_FLASH_SECT_SIZE);
			info->protect[i] = 0;
		}
	}
}

/*-----------------------------------------------------------------------
 */
void flash_print_info (flash_info_t *info)
{
	int i;

	if (info->flash_id == FLASH_UNKNOWN) {
		printf ("missing or unknown FLASH type\n");
		return;
	}

	switch (info->flash_id & FLASH_VENDMASK) {
	case FLASH_MAN_INTEL:
		printf ("INTEL ");
		break;
	default:
		printf ("Unknown Vendor ");
		break;
	}

	switch (info->flash_id & FLASH_TYPEMASK) {
	case FLASH_28F640J3A:
		printf ("28F640J3A\n");
		break;
	case FLASH_28F128J3A:
		printf ("28F128J3A\n");
		break;
	default:
		printf ("Unknown Chip Type\n");
		break;
	}

	printf ("  Size: %ld MB in %d Sectors\n",
			info->size >> 20, info->sector_count);

	printf ("  Sector Start Addresses:");
	for (i = 0; i < info->sector_count; ++i) {
		if ((i % 5) == 0)
			printf ("\n   ");
		printf (" %08lX%s",
			info->start[i],
			info->protect[i] ? " (RO)" : "     ");
	}
	printf ("\n");
	return;
}

/*
 * The following code cannot be run from FLASH!
 */
static ulong flash_get_size (FPW *addr, flash_info_t *info)
{
	volatile FPW value;

	/* Write auto select command: read Manufacturer ID */
	addr[0x5555] = (FPW) 0x00AA00AA;
	addr[0x2AAA] = (FPW) 0x00550055;
	addr[0x5555] = (FPW) 0x00900090;

	mb ();
	value = addr[0];

	switch (value) {

	case (FPW) INTEL_MANUFACT:
		info->flash_id = FLASH_MAN_INTEL;
		break;

	default:
		info->flash_id = FLASH_UNKNOWN;
		info->sector_count = 0;
		info->size = 0;
		addr[0] = (FPW) INTEL_RESET;	/* restore read mode */
		return (0);			/* no or unknown flash  */
	}

	mb ();
	value = addr[1];			/* device ID        */

	switch (value) {

	case (FPW) INTEL_ID_28F640J3A:
		info->flash_id += FLASH_28F640J3A;
		info->sector_count = 64;
		info->size = 0x00800000;
		break;				/* => 8 MB     */

	case (FPW) INTEL_ID_28F128J3A:
		info->flash_id += FLASH_28F128J3A;
		info->sector_count = 128;
		info->size = 0x01000000;
		break;				/* => 16 MB     */

	default:
		info->flash_id = FLASH_UNKNOWN;
		break;
	}

	if (info->sector_count > CONFIG_SYS_MAX_FLASH_SECT) {
		printf ("** ERROR: sector count %d > max (%d) **\n",
			info->sector_count, CONFIG_SYS_MAX_FLASH_SECT);
		info->sector_count = CONFIG_SYS_MAX_FLASH_SECT;
	}

	addr[0] = (FPW) INTEL_RESET;		/* restore read mode */

	return (info->size);
}


/*-----------------------------------------------------------------------
 */

int flash_erase (flash_info_t *info, int s_first, int s_last)
{
	int prot, sect;
	ulong type, start, last;
	int rcode = 0;
	int cflag, iflag;

	if ((s_first < 0) || (s_first > s_last)) {
		if (info->flash_id == FLASH_UNKNOWN) {
			printf ("- missing\n");
		} else {
			printf ("- no sectors to erase\n");
		}
		return 1;
	}

	type = (info->flash_id & FLASH_VENDMASK);
	if ((type != FLASH_MAN_INTEL)) {
		printf ("Can't erase unknown flash type %08lx - aborted\n",
			info->flash_id);
		return 1;
	}

	prot = 0;
	for (sect = s_first; sect <= s_last; ++sect) {
		if (info->protect[sect]) {
			prot++;
		}
	}

	if (prot) {
		printf ("- Warning: %d protected sectors will not be erased!\n",
			prot);
	} else {
		printf ("\n");
	}

	start = get_timer (0);
	last = start;

	/*
	 * Disable interrupts which might cause a timeout
	 * here. Remember that our exception vectors are
	 * at address 0 in the flash, and we don't want a
	 * (ticker) exception to happen while the flash
	 * chip is in programming mode.
	 */
	cflag = icache_status ();
	icache_disable ();
	/* Disable interrupts which might cause a timeout here */
	iflag = disable_interrupts ();

	/* Start erase on unprotected sectors */
	for (sect = s_first; sect <= s_last; sect++) {
		if (info->protect[sect] == 0) {	/* not protected */
			FPWV *addr = (FPWV *) (info->start[sect]);
			FPW status;

			printf ("Erasing sector %2d ... ", sect);

			/* arm simple, non interrupt dependent timer */
			reset_timer_masked ();

			*addr = (FPW) INTEL_CLEAR;	/* clear status register */
			*addr = (FPW) INTEL_ERASE;	/* erase setup */
			*addr = (FPW) INTEL_CONFIRM;	/* erase confirm */

			while (((status = *addr) & (FPW) INTEL_FINISHED) != (FPW) INTEL_FINISHED) {
				if (get_timer_masked () > CONFIG_SYS_FLASH_ERASE_TOUT) {
					printf ("Timeout\n");
					*addr = (FPW) INTEL_SUSPEND;	/* suspend erase     */
					*addr = (FPW) INTEL_RESET;	/* reset to read mode */
					rcode = 1;
					break;
				}
			}

			*addr = (FPWV)INTEL_CLEAR;	/* clear status register cmd.   */
			*addr = (FPWV)INTEL_RESET;	/* resest to read mode          */

			printf (" done\n");
		}
	}

	if (iflag)
		enable_interrupts ();

	if (cflag)
		icache_enable ();

	return rcode;
}

/*-----------------------------------------------------------------------
 * Copy memory to flash, returns:
 * 0 - OK
 * 1 - write timeout
 * 2 - Flash not erased
 * 4 - Flash not identified
 */

int write_buff (flash_info_t *info, uchar *src, ulong addr, ulong cnt)
{
	ulong cp, wp;
	FPW data;
	int count, i, l, rc, port_width;

	if (info->flash_id == FLASH_UNKNOWN) {
		return 4;
	}

	/* get lower word aligned address */
	wp = (addr & ~1);
	port_width = 2;

	/*
	 * handle unaligned start bytes
	 */
	if ((l = addr - wp) != 0) {
		data = 0;
		for (i = 0, cp = wp; i < l; ++i, ++cp) {
			data = (data << 8) | (*(uchar *) cp);
		}
		for (; i < port_width && cnt > 0; ++i) {
			data = (data << 8) | *src++;
			--cnt;
			++cp;
		}
		for (; cnt == 0 && i < port_width; ++i, ++cp) {
			data = (data << 8) | (*(uchar *) cp);
		}

		if ((rc = write_data (info, wp, SWAP (data))) != 0) {
			return (rc);
		}
		wp += port_width;
	}

	/*
	 * handle word aligned part
	 */
	count = 0;
	while (cnt >= port_width) {
		data = 0;
		for (i = 0; i < port_width; ++i) {
			data = (data << 8) | *src++;
		}
		if ((rc = write_data (info, wp, SWAP (data))) != 0) {
			return (rc);
		}
		wp += port_width;
		cnt -= port_width;
		if (count++ > 0x800) {
			spin_wheel ();
			count = 0;
		}
	}

	if (cnt == 0) {
		return (0);
	}

	/*
	 * handle unaligned tail bytes
	 */
	data = 0;
	for (i = 0, cp = wp; i < port_width && cnt > 0; ++i, ++cp) {
		data = (data << 8) | *src++;
		--cnt;
	}
	for (; i < port_width; ++i, ++cp) {
		data = (data << 8) | (*(uchar *) cp);
	}

	return (write_data (info, wp, SWAP (data)));
}

/*-----------------------------------------------------------------------
 * Write a word or halfword to Flash, returns:
 * 0 - OK
 * 1 - write timeout
 * 2 - Flash not erased
 */
static int write_data (flash_info_t *info, ulong dest, FPW data)
{
	FPWV *addr = (FPWV *) dest;
	ulong status;
	int cflag, iflag;

	/* Check if Flash is (sufficiently) erased */
	if ((*addr & data) != data) {
		printf ("not erased at %08lx (%lx)\n", (ulong) addr, (ulong) *addr);
		return (2);
	}
	/*
	 * Disable interrupts which might cause a timeout
	 * here. Remember that our exception vectors are
	 * at address 0 in the flash, and we don't want a
	 * (ticker) exception to happen while the flash
	 * chip is in programming mode.
	 */
	cflag = icache_status ();
	icache_disable ();
	/* Disable interrupts which might cause a timeout here */
	iflag = disable_interrupts ();

	*addr = (FPW) INTEL_PROG;	/* write setup */
	*addr = data;

	/* arm simple, non interrupt dependent timer */
	reset_timer_masked ();

	/* wait while polling the status register */
	while (((status = *addr) & (FPW) INTEL_FINISHED) != (FPW) INTEL_FINISHED) {
		if (get_timer_masked () > CONFIG_SYS_FLASH_WRITE_TOUT) {
			*addr = (FPW) INTEL_RESET;	/* restore read mode */
			return (1);
		}
	}

	*addr = (FPW) INTEL_RESET;	/* restore read mode */

	if (iflag)
		enable_interrupts ();

	if (cflag)
		icache_enable ();

	return (0);
}

void inline spin_wheel (void)
{
	static int p = 0;
	static char w[] = "\\/-";

	printf ("\010%c", w[p]);
	(++p == 3) ? (p = 0) : 0;
}

/*-----------------------------------------------------------------------
 * Set/Clear sector's lock bit, returns:
 * 0 - OK
 * 1 - Error (timeout, voltage problems, etc.)
 */
int flash_real_protect(flash_info_t *info, long sector, int prot)
{
	int i;
	int rc = 0;
	FPWV *addr = (FPWV *)(info->start[sector]);
	int flag = disable_interrupts();

	*addr = (FPW) INTEL_CLEAR;	/* Clear status register */
	if (prot) {			/* Set sector lock bit */
		*addr = (FPW) INTEL_LOCKBIT;	/* Sector lock bit */
		*addr = (FPW) INTEL_PROTECT;	/* set */
	}
	else {				/* Clear sector lock bit */
		*addr = (FPW) INTEL_LOCKBIT;	/* All sectors lock bits */
		*addr = (FPW) INTEL_CONFIRM;	/* clear */
	}

	reset_timer_masked ();

	while ((*addr & (FPW) INTEL_FINISHED) != (FPW) INTEL_FINISHED) {
		if (get_timer_masked () > CONFIG_SYS_FLASH_UNLOCK_TOUT) {
			printf("Flash lock bit operation timed out\n");
			rc = 1;
			break;
		}
	}

	if (*addr != (FPW) INTEL_OK) {
		printf("Flash lock bit operation failed at %08X, CSR=%08X\n",
		       (uint)addr, (uint)*addr);
		rc = 1;
	}

	if (!rc)
		info->protect[sector] = prot;

	/*
	 * Clear lock bit command clears all sectors lock bits, so
	 * we have to restore lock bits of protected sectors.
	 */
	if (!prot)
	{
		for (i = 0; i < info->sector_count; i++)
		{
			if (info->protect[i])
			{
				reset_timer_masked ();
				addr = (FPWV *) (info->start[i]);
				*addr = (FPW) INTEL_LOCKBIT;	/* Sector lock bit */
				*addr = (FPW) INTEL_PROTECT;	/* set */
				while ((*addr & (FPW) INTEL_FINISHED) != (FPW) INTEL_FINISHED)
				{
					if (get_timer_masked () > CONFIG_SYS_FLASH_UNLOCK_TOUT)
					{
						printf("Flash lock bit operation timed out\n");
						rc = 1;
						break;
					}
				}
			}
		}
	}

	if (flag)
		enable_interrupts();

	*addr = (FPW) INTEL_RESET;		/* Reset to read array mode */

	return rc;
}
