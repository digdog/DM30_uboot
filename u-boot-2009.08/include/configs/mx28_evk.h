/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef __MX28_EVK_H
#define __MX28_EVK_H

#include <asm/arch/mx28.h>

/*
 * SoC configurations
 */
#define CONFIG_MX28				/* i.MX28 SoC */
#define CONFIG_SYS_HZ		1000		/* Ticks per second */
/* ROM loads UBOOT into DRAM */
#define CONFIG_SKIP_RELOCATE_UBOOT

/*
 * Memory configurations
 */
#define CONFIG_NR_DRAM_BANKS	1		/* 1 bank of DRAM */
#define PHYS_SDRAM_1		0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE	0x08000000	/* 128 MB */
#define CONFIG_STACKSIZE	0x00020000	/* 128 KB stack */
#define CONFIG_SYS_MALLOC_LEN	0x00400000	/* 4 MB for malloc */
#define CONFIG_SYS_GBL_DATA_SIZE 128		/* Reserved for initial data */
#define CONFIG_SYS_MEMTEST_START 0x40000000	/* Memtest start address */
#define CONFIG_SYS_MEMTEST_END	 0x40400000	/* 4 MB RAM test */

/*
 * U-Boot general configurations
 */
#define CONFIG_SYS_PROMPT	"MX28 U-Boot > "
#define CONFIG_SYS_CBSIZE	1024		/* Console I/O buffer size */
#define CONFIG_SYS_PBSIZE \
	(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
						/* Print buffer size */
#define CONFIG_SYS_MAXARGS	16		/* Max number of command args */
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE
						/* Boot argument buffer size */
#define CONFIG_VERSION_VARIABLE			/* U-BOOT version */
#define CONFIG_AUTO_COMPLETE			/* Command auto complete */
#define CONFIG_CMDLINE_EDITING			/* Command history etc */

#define CONFIG_SYS_64BIT_VSPRINTF

/*
 * Boot Linux
 */
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_BOOTDELAY	3
#define CONFIG_BOOTFILE		"uImage"
#define CONFIG_BOOTARGS		"console=ttyAM0,115200n8 "
#define CONFIG_BOOTCOMMAND	"run bootcmd_net"
#define CONFIG_LOADADDR		0x42000000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR

/*
 * Extra Environments
 */
#define	CONFIG_EXTRA_ENV_SETTINGS \
	"nfsroot=/home/notroot/nfs/rootfs\0" \
	"bootargs_nfs=setenv bootargs ${bootargs} root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp " \
		"fec_mac=${ethaddr}\0" \
	"bootcmd_net=run bootargs_nfs; dhcp; bootm\0" \
	"bootargs_mmc=setenv bootargs ${bootargs} root=/dev/mmcblk0p3 " \
		"rw rootwait ip=dhcp fec_mac=${ethaddr}\0" \
	"bootcmd_mmc=run bootargs_mmc; " \
		"mmc read 0 ${loadaddr} 100 3000; bootm\0" \

/*
 * U-Boot Commands
 */
#define CONFIG_SYS_NO_FLASH
#include <config_cmd_default.h>
#define CONFIG_ARCH_CPU_INIT
#define CONFIG_DISPLAY_CPUINFO

/*
 * Serial Driver
 */
#define CONFIG_UARTDBG_CLK		24000000
#define CONFIG_BAUDRATE			115200		/* Default baud rate */
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }

/*
 * FEC Driver
 */
#define CONFIG_MXC_FEC
#define CONFIG_FEC0_IOBASE		REGS_ENET_BASE
#define CONFIG_FEC0_PHY_ADDR		0
#define CONFIG_NET_MULTI
#define CONFIG_ETH_PRIME
#define CONFIG_RMII
#define CONFIG_CMD_MII
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_PING
#define CONFIG_IPADDR			192.168.1.103
#define CONFIG_SERVERIP			192.168.1.101
#define CONFIG_NETMASK			255.255.255.0
/* Add for working with "strict" DHCP server */
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_DNS


/*
 * MMC Driver
 */
#define CONFIG_CMD_MMC
#define CONFIG_MMC
#define CONFIG_IMX_SSP_MMC		/* MMC driver based on SSP */
#define CONFIG_GENERIC_MMC
#define CONFIG_SYS_MMC_ENV_DEV	0
#define CONFIG_DOS_PARTITION
#define CONFIG_CMD_FAT

/*
 * Environments on MMC
 */
#define CONFIG_CMD_ENV
#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_IS_IN_MMC
/* Assoiated with the MMC layout defined in mmcops.c */
#define CONFIG_ENV_OFFSET		(0x400) /* 1 KB */
#define CONFIG_ENV_SIZE			(0x20000 - 0x400) /* 127 KB */

#endif /* __MX28_EVK_H */
