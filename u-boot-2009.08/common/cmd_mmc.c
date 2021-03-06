/*
 * (C) Copyright 2003
 * Kyle Harris, kharris@nexus-tech.net
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

#define DEBUG

#include <common.h>
#include <command.h>
#include <mmc.h>

#ifdef DEBUG
#include <asm/arch/timrot.h>

#define TIMCTRL		TIMCTRL0
#define TIMCOUNT	TIMCOUNT0

#define READ_TIMER ((REG_RD(TIMROT_BASE + TIMCOUNT) & 0xffff0000) >> 16)

#define time_test_start() 				\
	{						\
		uint32_t __t1, __t2, __t;		\
		__t1 = READ_TIMER;


#define time_test_end()						\
		do {						\
			__t2 = READ_TIMER;			\
		       if (__t1 > __t2)				\
			       __t = __t1 - __t2;		\
		       else					\
			       __t = __t1 + 0xffff - __t2;	\
		       pr_info("%u ms\n", __t);			\
		} while (0);					\
	}

#else
#define time_test_end()
#define time_test_start()
#endif

#ifndef CONFIG_GENERIC_MMC
static int curr_device = -1;

int do_mmc (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int dev;

	if (argc < 2) {
		cmd_usage(cmdtp);
		return 1;
	}

	if (strcmp(argv[1], "init") == 0) {
		if (argc == 2) {
			if (curr_device < 0)
				dev = 1;
			else
				dev = curr_device;
		} else if (argc == 3) {
			dev = (int)simple_strtoul(argv[2], NULL, 10);
		} else {
			cmd_usage(cmdtp);
			return 1;
		}

		if (mmc_legacy_init(dev) != 0) {
			puts("No MMC card found\n");
			return 1;
		}

		curr_device = dev;
		printf("mmc%d is available\n", curr_device);
	} else if (strcmp(argv[1], "device") == 0) {
		if (argc == 2) {
			if (curr_device < 0) {
				puts("No MMC device available\n");
				return 1;
			}
		} else if (argc == 3) {
			dev = (int)simple_strtoul(argv[2], NULL, 10);

#ifdef CONFIG_SYS_MMC_SET_DEV
			if (mmc_set_dev(dev) != 0)
				return 1;
#endif
			curr_device = dev;
		} else {
			cmd_usage(cmdtp);
			return 1;
		}

		printf("mmc%d is current device\n", curr_device);
	} else {
		cmd_usage(cmdtp);
		return 1;
	}

	return 0;
}

U_BOOT_CMD(
	mmc, 3, 1, do_mmc,
	"MMC sub-system",
	"init [dev] - init MMC sub system\n"
	"mmc device [dev] - show or set current device"
);
#else /* !CONFIG_GENERIC_MMC */

/**
 * Extract the Manufacturer ID from the CID
 * @param mmc Instance data
 *
 * The 'MID' is encoded in bit 127:120 in the CID
 */
static unsigned extract_mid(struct mmc *mmc)
{
	return mmc->cid[0] >> 24;
}

/**
 * Extract the OEM/Application ID from the CID
 * @param mmc Instance data
 *
 * The 'OID' is encoded in bit 119:104 in the CID
 */
static unsigned extract_oid(struct mmc *mmc)
{
	return (mmc->cid[0] >> 8) & 0xffff;
}

/**
 * Extract the product revision from the CID
 * @param mmc Instance data
 *
 * The 'PRV' is encoded in bit 63:56 in the CID
 */
static unsigned extract_prv(struct mmc *mmc)
{
	return mmc->cid[2] >> 24;
}

/**
 * Extract the product serial number from the CID
 * @param mmc Instance data
 *
 * The 'PSN' is encoded in bit 55:24 in the CID
 */
static unsigned extract_psn(struct mmc *mmc)
{
	return (mmc->cid[2] << 8) | (mmc->cid[3] >> 24);
}

/**
 * Extract the month of the manufacturing date from the CID
 * @param mmc Instance data
 *
 * The 'MTD' is encoded in bit 19:8 in the CID, month in 11:8
 */
static unsigned extract_mtd_month(struct mmc *mmc)
{
	return (mmc->cid[3] >> 8) & 0xf;
}

/**
 * Extract the year of the manufacturing date from the CID
 * @param mmc Instance data
 *
 * The 'MTD' is encoded in bit 19:8 in the CID, year in 19:12
 * An encoded 0 means the year 2000
 */
static unsigned extract_mtd_year(struct mmc *mmc)
{
	return ((mmc->cid[3] >> 12) & 0xff) + 2000U;
}

static void print_mmcinfo(struct mmc *mmc)
{
	printf("Device: %s\n", mmc->name);
	printf(" Card:\n");
	if (mmc->version < SD_VERSION_SD) {
		printf("  Attached is a MultiMediaCard (Version: %u.%u)\n",
			(mmc->version >> 4) & 0xf, mmc->version & 0xf);
	} else {
		printf("  Attached is an SD Card (Version: %u.%u)\n",
			(mmc->version >> 4) & 0xf, mmc->version & 0xf);
	}
	printf("  Capacity: %u MiB\n", (unsigned)(mmc->capacity >> 20));

	if (mmc->high_capacity)
		printf("  High capacity card\n");
	printf("     CID: %08X-%08X-%08X-%08X\n", mmc->cid[0], mmc->cid[1],
		mmc->cid[2], mmc->cid[3]);
	printf("     CSD: %08X-%08X-%08X-%08X\n", mmc->csd[0], mmc->csd[1],
		mmc->csd[2], mmc->csd[3]);
	printf("  Max. transfer speed: %u Hz\n", mmc->tran_speed);
	printf("  Manufacturer ID: %02X\n", extract_mid(mmc));
	printf("  OEM/Application ID: %04X\n", extract_oid(mmc));
	printf("  Product name: '%c%c%c%c%c'\n", mmc->cid[0] & 0xff,
		(mmc->cid[1] >> 24), (mmc->cid[1] >> 16) & 0xff,
		(mmc->cid[1] >> 8) & 0xff, mmc->cid[1] & 0xff);
	printf("  Product revision: %u.%u\n", extract_prv(mmc) >> 4,
		extract_prv(mmc) & 0xf);
	printf("  Serial no: %0u\n", extract_psn(mmc));
	printf("  Manufacturing date: %u.%u\n", extract_mtd_month(mmc),
		extract_mtd_year(mmc));

	printf("  Read Block Len: %d\n", mmc->read_bl_len);
	printf("  Bus Width: %d-bit\n", mmc->bus_width);

#ifdef CONFIG_BOOT_PARTITION_ACCESS
	if (mmc->boot_size_mult == 0) {
		printf("Boot Partition Size: %s\n", "No boot partition available");
	} else {
		printf("Boot Partition Size: %5dKB\n", mmc->boot_size_mult * 128);

		printf("Current Partition for boot: ");
		switch (mmc->boot_config & EXT_CSD_BOOT_PARTITION_ENABLE_MASK) {
		case EXT_CSD_BOOT_PARTITION_DISABLE:
			printf("Not bootable\n");
			break;
		case EXT_CSD_BOOT_PARTITION_PART1:
			printf("Boot partition 1\n");
			break;
		case EXT_CSD_BOOT_PARTITION_PART2:
			printf("Boot partition 2\n");
			break;
		case EXT_CSD_BOOT_PARTITION_USER:
			printf("User area\n");
			break;
		default:
			printf("Unknown\n");
			break;
		}

		printf("Current Partition for read/write: ");
		switch (mmc->boot_config & EXT_CSD_BOOT_PARTITION_ACCESS_MASK) {
		case EXT_CSD_BOOT_PARTITION_ACCESS_DISABLE:
			printf("User Area\n");
			break;
		case EXT_CSD_BOOT_PARTITION_ACCESS_PART1:
			printf("Boot partition 1\n");
			break;
		case EXT_CSD_BOOT_PARTITION_ACCESS_PART2:
			printf("Boot partition 2\n");
			break;
		default:
			printf("Unknown\n");
			break;
		}
	}
#endif
}

int do_mmcinfo (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	struct mmc *mmc;
	int dev_num;

	if (argc < 2)
		dev_num = 0;
	else
		dev_num = simple_strtoul(argv[1], NULL, 0);

	mmc = find_mmc_device(dev_num);

	if (mmc) {
		if (mmc_init(mmc))
			puts("MMC card init failed!\n");
		else
			print_mmcinfo(mmc);
	}

	return 0;
}

U_BOOT_CMD(mmcinfo, 2, 0, do_mmcinfo,
	"mmcinfo <dev num>-- display MMC info",
	""
);

int do_mmcops(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int rc = 0;
#ifdef CONFIG_BOOT_PARTITION_ACCESS
	u32 part = 0;
#endif

	switch (argc) {
	case 3:
		if (!strcmp(argv[1], "rescan") || !strcmp(argv[1], "init")) {
			int dev = simple_strtoul(argv[2], NULL, 10);
			struct mmc *mmc = find_mmc_device(dev);

			if (!mmc)
				return 1;

			mmc_init(mmc);

			return 0;
		}

	case 0:
	case 1:
	case 4:
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;

	case 2:
		if (!strcmp(argv[1], "list")) {
			print_mmc_devices('\n');
			return 0;
		}
		return 1;
#ifdef CONFIG_BOOT_PARTITION_ACCESS
	case 6: /* Fall through */
		part = simple_strtoul(argv[6], NULL, 10);
#endif
	default: /* at least 5 args */
		if (strcmp(argv[1], "read") == 0) {
			int dev = simple_strtoul(argv[2], NULL, 10);
			void *addr = (void *)simple_strtoul(argv[3], NULL, 16);
			u32 cnt = simple_strtoul(argv[5], NULL, 16);
			u32 n;
			u32 blk = simple_strtoul(argv[4], NULL, 16);

			struct mmc *mmc = find_mmc_device(dev);

			if (!mmc)
				return 1;

			printf("\nMMC read: dev # %d, block # %d, count %d ... ",
				dev, blk, cnt);

			//mmc_init(mmc);

#ifdef CONFIG_BOOT_PARTITION_ACCESS
			/* After mmc_init, we now know whether this is a eSD/eMMC which support boot partition */
			if (part > 0) {
				if (IS_SD(mmc)) {
					if (sd_switch_partition(mmc, 1) < 0) {
						printf("\nError: Fail to switch partition to %d before reading\n", 1);
						return 1;
					}
				} else {
					if (mmc_switch_partition(mmc, part) < 0) {
						printf("\nError: Fail to switch partition to %d before reading\n", part);
						return 1;
					}
				}
			}
#endif
			time_test_start();
			n = mmc->block_dev.block_read(dev, blk, cnt, addr);
			time_test_end();

			/* flush cache after read */
			flush_cache((ulong)addr, cnt * 512); /* FIXME */

#ifdef CONFIG_BOOT_PARTITION_ACCESS
			/* Switch back */
			if (part > 0) {
				if (IS_SD(mmc)) {
					if (sd_switch_partition(mmc, 0) < 0) {
						printf("\nError: Fail to switch partition back!\n");
						return 1;
					}
				} else {
					if (mmc_switch_partition(mmc, 0) < 0) {
						printf("\nError: Fail to switch partition back!\n");
						return 1;
					}
				}
			}
#endif
			printf("%d blocks read: %s\n",
				n, (n==cnt) ? "OK" : "ERROR");
			return (n == cnt) ? 0 : 1;
		} else if (strcmp(argv[1], "write") == 0) {
			int dev = simple_strtoul(argv[2], NULL, 10);
			void *addr = (void *)simple_strtoul(argv[3], NULL, 16);
			u32 cnt = simple_strtoul(argv[5], NULL, 16);
			u32 n;

			struct mmc *mmc = find_mmc_device(dev);

			int blk = simple_strtoul(argv[4], NULL, 16);

			if (!mmc)
				return 1;

			printf("\nMMC write: dev # %d, block # %d, count %d ... ",
				dev, blk, cnt);

			//mmc_init(mmc);

#ifdef CONFIG_BOOT_PARTITION_ACCESS
			/* After mmc_init, we now know whether this is a eSD/eMMC which support boot partition */
			 if (part > 0) {
				if (IS_SD(mmc)) {
					if (sd_switch_partition(mmc, 1) < 0) {
						printf("\nError: Fail to switch partition to %d before writing\n", 1);
						return 1;
					}
				} else {
					if (mmc_switch_partition(mmc, part) < 0) {
						printf("\nError: Fail to switch partition to %d before writing\n", part);
						return 1;
					}
				}
			}
#endif

			time_test_start();
			n = mmc->block_dev.block_write(dev, blk, cnt, addr);
			time_test_end();

#ifdef CONFIG_BOOT_PARTITION_ACCESS
			/* Switch back */
			if (part > 0) {
				if (IS_SD(mmc)) {
					if (sd_switch_partition(mmc, 0) < 0) {
						printf("\nError: Fail to switch partition back!\n");
						return 1;
					}
				} else {
					if (mmc_switch_partition(mmc, 0) < 0) {
						printf("\nError: Fail to switch partition back!\n");
						return 1;
					}
				}
			}
#endif

			printf("%d blocks written: %s\n",
				n, (n == cnt) ? "OK" : "ERROR");
			return (n == cnt) ? 0 : 1;
		} else {
			printf("Usage:\n%s\n", cmdtp->usage);
			rc = 1;
		}

		return rc;
	}
}

#ifndef CONFIG_BOOT_PARTITION_ACCESS
U_BOOT_CMD(
	mmc, 6, 0, do_mmcops,
	"MMC sub system",
	"mmc read <device num> addr blk# cnt\n"
	"mmc write <device num> addr blk# cnt\n"
	"mmc rescan <device num>\n"
	"mmc list - lists available devices");
#else
U_BOOT_CMD(
	mmc, 7, 0, do_mmcops,
	"MMC sub system",
	"mmc read <device num> addr blk# cnt [partition]\n"
	"mmc write <device num> addr blk# cnt [partition]\n"
	"mmc rescan <device num>\n"
	"mmc list - lists available devices");
#endif
#endif

