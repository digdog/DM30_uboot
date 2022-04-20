/*
 * Copyright (C) 2008 Embedded Alley Solutions, Inc.
 *
 * (C) Copyright 2009-2010 Freescale Semiconductor, Inc.
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

#ifndef __CONFIG_H
#define __CONFIG_H
#include <asm/sizes.h>

#include <asm/arch/mx23.h>
/*
 * Define this to make U-Boot skip low level initialization when loaded
 * by initial bootloader. Not required by NAND U-Boot version but IS
 * required for a NOR version used to burn the real NOR U-Boot into
 * NOR Flash. NAND and NOR support for DaVinci chips is mutually exclusive
 * so it is NOT possible to build a U-Boot with both NAND and NOR routines.
 * NOR U-Boot is loaded directly from Flash so it must perform all the
 * low level initialization itself. NAND version is loaded by an initial
 * bootloader (UBL in TI-ese) that performs such an initialization so it's
 * skipped in NAND version. The third DaVinci boot mode loads a bootloader
 * via UART0 and that bootloader in turn loads and runs U-Boot (or whatever)
 * performing low level init prior to loading. All that means we can NOT use
 * NAND version to put U-Boot into NOR because it doesn't have NOR support and
 * we can NOT use NOR version because it performs low level initialization
 * effectively destroying itself in DDR memory. That's why a separate NOR
 * version with this define is needed. It is loaded via UART, then one uses
 * it to somehow download a proper NOR version built WITHOUT this define to
 * RAM (tftp?) and burn it to NOR Flash. I would be probably able to squeeze
 * NOR support into the initial bootloader so it won't be needed but DaVinci
 * static RAM might be too small for this (I have something like 2Kbytes left
 * as of now, without NOR support) so this might've not happened...
 *
 */

/*===================*/
/* SoC Configuration */
/*===================*/
#define CONFIG_ARM926EJS			/* arm926ejs CPU core */
#define CONFIG_MX23				/* MX23 SoC */
#define CONFIG_MX23_EVK				/* MX23 EVK board */
#define CONFIG_SYS_CLK_FREQ	120000000	/* Arm Clock frequency */
#define CONFIG_USE_TIMER0			/* use timer 0 */
#define CONFIG_SYS_HZ		1000		/* Ticks per second */

/*=============*/
/* Memory Info */
/*=============*/
//#define CONFIG_SYS_MALLOC_LEN	(0x10000 + 128*1024)	/* malloc() len */
#define CONFIG_SYS_MALLOC_LEN	(0x500000) 	/* malloc() len 5M */
#define CONFIG_SYS_GBL_DATA_SIZE 128		/* reserved for initial data */
#define CONFIG_SYS_MEMTEST_START 0x40000000	/* memtest start address */
#define CONFIG_SYS_MEMTEST_END	 0x41000000	/* 16MB RAM test */
#define CONFIG_NR_DRAM_BANKS	2		/* we have 1 bank of DRAM */
//#define CONFIG_STACKSIZE	(256*1024)	/* regular stack */
#define CONFIG_STACKSIZE	(512*1024)	/* regular stack */
#define PHYS_SDRAM_1		0x40000000	/* mDDR Start */
#define PHYS_SDRAM_1_SIZE	0x04000000	/* mDDR size 64MB */
#define PHYS_SDRAM_2		0x44000000	/* mDDR Start */
#define PHYS_SDRAM_2_SIZE	0x04000000	/* mDDR size 64MB */

/*====================*/
/* Serial Driver info */
/*====================*/
#define CONFIG_STMP3XXX_DBGUART			/* 378x debug UART */
#define CONFIG_DBGUART_CLK	24000000
#define CONFIG_BAUDRATE		115200		/* Default baud rate */
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }

/*====================*/
/* SPI Driver info */
/*====================*/
#define CONFIG_SSP_CLK		48000000
#define CONFIG_SPI_CLK		3000000
#define CONFIG_SPI_SSP1
#undef CONFIG_SPI_SSP2
#define CONFIG_CMD_CLOCK

/*==============================*/
/* U-Boot general configuration */
/*==============================*/
#undef	CONFIG_USE_IRQ				/* No IRQ/FIQ in U-Boot */
#define CONFIG_MISC_INIT_R
#if 0
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_IPADDR		192.167.10.2
#define CONFIG_SERVERIP		192.167.10.1
#endif
#define CONFIG_BOOTDELAY	2
#define CONFIG_BOOTFILE		"uImage"	/* Boot file name */
#define CONFIG_SYS_PROMPT	"MX23 U-Boot > "
						/* Monitor Command Prompt */
#define CONFIG_SYS_CBSIZE	1024		/* Console I/O Buffer Size  */
#define CONFIG_SYS_PBSIZE	(CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16)
						/* Print buffer sz */
#define CONFIG_SYS_MAXARGS	16		/* max number of command args */
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE
						/* Boot Argument Buffer Size */
#define CONFIG_SYS_LOAD_ADDR	0x40400000 	/* default Linux kernel load address */
#define CONFIG_VERSION_VARIABLE
#define CONFIG_AUTO_COMPLETE	/* Won't work with hush so far, may be later */
#define CFG_HUSH_PARSER
#define CFG_PROMPT_HUSH_PS2	"> "
#define CONFIG_CMDLINE_EDITING
//#define CFG_LONGHELP
#define CONFIG_CRC32_VERIFY
#define CONFIG_MX_CYCLIC

/*===================*/
/* Linux Information */
/*===================*/
#define LINUX_BOOT_PARAM_ADDR	0x40000100
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_BOOTARGS		"console=ttyAM0,115200n8 root=/dev/mtdblock1 rootfstype=jffs2 lcd_panel=lms350"
#define CONFIG_BOOTCOMMAND	"tftpboot ; bootm"

/*=================*/
/* U-Boot commands */
/*=================*/
#include <config_cmd_default.h>
#if 0
#define CONFIG_CMD_ASKENV
#define CONFIG_CMD_DHCP
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_DNS
#undef CONFIG_CMD_DIAG
#define CONFIG_CMD_MII
#define CONFIG_CMD_PING
#define CONFIG_CMD_NET
#define CONFIG_CMD_SAVES
#else
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_NET
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_ITEST
#undef CONFIG_CMD_NFS
#undef CONFIG_CMD_SETGETDCR
#undef CONFIG_CMD_SOURCE
#undef CONFIG_CMD_XIMG
#undef CONFIG_CMD_LOADB
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_MISC
#endif

/* Ethernet chip - select an alternative driver */
#if 0
#define CONFIG_ENC28J60_ETH
#define CONFIG_ENC28J60_ETH_SPI_BUS	0
#define CONFIG_ENC28J60_ETH_SPI_CS	0
#endif

/*=====================*/
/* Flash & Environment */
/*=====================*/
#define CONFIG_SYS_NO_FLASH		/* Flash is not supported */
#define CONFIG_ENV_IS_NOWHERE		/* Store ENV in memory only */
#define CONFIG_ENV_SECT_SIZE	(32 * 1024)
#define CONFIG_ENV_SIZE         CONFIG_ENV_SECT_SIZE

/*=============================*/
/* mx23 GPMI NandFlash support */
/*=============================*/
#define CONFIG_CMD_NAND
#define CONFIG_ENV_IS_IN_NAND 		1
#define CONFIG_ENV_OFFSET		0x80000
#define CONFIG_NAND_GPMI
//#define CONFIG_MX23_NAND
//#define CONFIG_MTD_DEBUG
//#define CONFIG_MTD_DEBUG_VERBOSE 	100
//#define CONFIG_MX23_NAND_DMA

#define CONFIG_SYS_NO_FLASH
#define NAND_MAX_CHIPS			1
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		0x40000000

#define CONFIG_SYS_64BIT_VSPRINTF       1
#define CONFIG_SYS_64BIT_STRTOUL        1

/*
 * MMC Driver
 */
#if 0
#define CONFIG_MMC_MX23
#else
#define CONFIG_MMC_MX23_DMA
#endif
#define CONFIG_CMD_MMC
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_SYS_MMC_ENV_DEV	0
#define CONFIG_DOS_PARTITION
#define CONFIG_CMD_FAT

//#define CONFIG_DISPLAY_CPUINFO
//#define CONFIG_DISPLAY_BOARDINFO

/*
 * Memory test
 */
#define CONFIG_SYS_ALT_MEMTEST
#define CONFIG_SYS_MEMTEST_SCRATCH 0x40000000

/*
 * LCD
 */
#define CONFIG_VIDEO_MX23
#define CONFIG_LCD_WIDTH	(801)
#define CONFIG_LCD_HEIGH	(600)
#define CONFIG_LCD
#define LCD_BPP			LCD_COLOR8
#define CONFIG_SYS_WHITE_ON_BLACK
#define CONFIG_VIDEO_MX23_DEMO
#define CONFIG_SYS_CONSOLE_INFO_QUIET

/*
 * I2C
 */
#if 0
#define CONFIG_I2C_MXS
#define CONFIG_HARD_I2C		1	/* I2C with hardware support */
#define CONFIG_SYS_I2C_MODULE	2	/* Select I2C module #1 or #2 */
#define CONFIG_SYS_I2C_SPEED	100000	/* 100 kHz */
#define CONFIG_SYS_I2C_SLAVE	0x8a
#define CONFIG_KEYBOARD
#define CONFIG_MXS_KPD
#endif

/*
 * GPIO
 */
#define DM100_KEY_LSHIFT 	PINID_AUART1_RX
#define DM100_KEY_CTRL		PINID_AUART1_TX
#define DM100_KEY_ALT		PINID_AUART1_CTS
#define DM100_KEY_RSHIFT	PINID_AUART1_RTS
#define DM100_KEY_BATDOOR	PINID_PWM2

/*
 * For debug
 */
#define CONFIG_BOOT_SD_VFAT

/*
 * Memory Map, Layout (RTFM: http://192.168.40.50/redmine/documents/124)
 */

#define DM100_DDR_OFFSET_USER_PROGRAM   (0x40000000)
#define DM100_DDR_OFFSET_DATABASE       (0x40800000)
#define DM100_DDR_OFFSET_HEAP           (0x43000000)

#define DM100_DDR_SIZE_USER_PROGRAM     (DM100_DDR_OFFSET_DATABASE - DM100_DDR_OFFSET_USER_PROGRAM)
#define DM100_DRR_SIZE_DATABASE         (DM100_DDR_OFFSET_HEAP - DM100_DDR_OFFSET_DATABASE)
#define DM100_DDR_SIZE_HEAP             (0x48000000 - DM100_DDR_OFFSET_HEAP)

/* size */
#define DM100_NF_SIZE_BOOTLOADER        SZ_4M
#define DM100_NF_SIZE_USER_PROGRAM      SZ_2M
#define DM100_NF_SIZE_DATABASE          (51 * SZ_1M) //51M
#define DM100_NF_SIZE_RESERVE           (55 * SZ_1M)
#define DM100_NF_SIZE_FILESYSTEM1       SZ_16M
#define DM100_NF_SIZE_FILESYSTEM2       SZ_128M

/* offset */
#define DM100_NF_OFFSET_BOOTLOADER        0
#define DM100_NF_OFFSET_USER_PROGRAM      (DM100_NF_OFFSET_BOOTLOADER   + DM100_NF_SIZE_BOOTLOADER)
#define DM100_NF_OFFSET_DATABASE          (DM100_NF_OFFSET_USER_PROGRAM + DM100_NF_SIZE_USER_PROGRAM)
#define DM100_NF_OFFSET_RESERVE           (DM100_NF_OFFSET_DATABASE     + DM100_NF_SIZE_DATABASE)
#define DM100_NF_OFFSET_FILESYSTEM1       (DM100_NF_OFFSET_RESERVE      + DM100_NF_SIZE_RESERVE)
#define DM100_NF_OFFSET_FILESYSTEM2       (DM100_NF_OFFSET_FILESYSTEM1  + DM100_NF_SIZE_FILESYSTEM1) 

#define CONFIG_DM100_POWER

#endif /* __CONFIG_H */
