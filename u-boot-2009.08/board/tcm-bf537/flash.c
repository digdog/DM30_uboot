/*
 * flash.c - helper commands for working with GPIO-assisted flash
 *
 * Copyright (c) 2005-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <common.h>
#include <command.h>
#include <asm/blackfin.h>
#include "gpio_cfi_flash.h"

int do_pf(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong faddr = CONFIG_SYS_FLASH_BASE;
	ushort data;
	ulong dflg;

	if (argc == 3) {
		dflg = simple_strtoul(argv[1], NULL, 16);
		faddr |= (dflg << 21);
		dflg = simple_strtoul(argv[2], NULL, 16);
		faddr |= (dflg << 22);
		gpio_cfi_flash_swizzle((void *)faddr);
	} else {
		data = bfin_read_PORTFIO();
		printf("Port F data %04x (PF4:%i PF5:%i)\n", data,
			!!(data & PF4), !!(data & PF5));
	}

	return 0;
}

U_BOOT_CMD(pf, 3, 0, do_pf,
	"set/clear PF4/PF5 GPIO flash bank switch\n",
	"<pf4> <pf5> - set PF4/PF5 GPIO pin state\n");
