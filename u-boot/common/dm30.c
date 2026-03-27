/*
 * ==========================================================================
 *
 *       Filename:  dm30.c
 *
 *    Description:  
 *
 *        Version:  0.01
 *        Created:  2011年06月12日 00时47分35秒
 *
 *         Author:  smmei (), 
 *        Company:  
 *
 * ==========================================================================
 */
#include <common.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <asm/errno.h>
#include <timestamp.h>
#include <asm/arch/pinctrl.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/mxs-block.h>
#include <asm/arch/registers/regsrtc.h>
#include <asm/arch/registers/regslradc.h>
#include <asm/arch/registers/regspower.h>
#include <nand.h>
#include <mmc.h>
#include <lcd.h>
#include <asm/arch/registers/regsdigctl.h>

DECLARE_GLOBAL_DATA_PTR;

#define PATCH_KEY_LSHIFT  	PINID_AUART1_RX
#define POWER_DETECT_PIN	PINID_LCD_D17

#define SECTION_SIZE		512
#define APP_HEAD_SIZE	0x800 // 2k
#define APP_DB_OFFSET		0x200000 // 2M

extern int fat_register_device(block_dev_desc_t *dev_desc, int part_no);
extern long file_fat_read(const char *filename, void *buffer, unsigned long maxsize);
extern int file_fat_ls(const char *dir);
extern void lcd_enable (void);
extern int mxs_mmc_is_plugged(void);
extern int dm30_patching_key(void);

static inline void mdelay(unsigned long msec)
{
	unsigned long i;
	for (i = 0; i < msec; i++)
		udelay(1000);
}

struct patch_rom {
	char name[64];
	char vers[20];
	uint32_t version;
	size_t size;
	uint32_t checksum;
	uint32_t addr_ddr; 
	uint32_t addr_nand;
};

struct user_prog_hl {
	uint32_t size;
	uint32_t checksum;
	char series[10];
	char vers[20];
	uint16_t refvol;   /* reference voltage */
};

struct database_hl {
	uint32_t size;
	uint32_t checksum;
};

struct logo_hl {
	uint16_t width;
	uint16_t heigh;
};

static struct patch_rom g_rom = {
	.addr_ddr = DM30_DDR_OFFSET_USER_PROGRAM, 
	.addr_nand = DM30_FLASH_OFFSET_USER_PROGRAM, 
};

static const char gromname[]= "Pomera_Update_Dm30_V";
static const char g_uboot_name[] = "dm30-uboot.raw";
static int g_first_partition_section = 0;

static int list_file_already=0;

static void __attribute__((unused)) poweroff(void)
{
	BW_RTC_PERSISTENT0_AUTO_RESTART(0);
	// 设置POWER_RESET寄存器必须先解锁
	BW_POWER_RESET_UNLOCK(BV_POWER_RESET_UNLOCK__KEY);
	mdelay(50);
	BW_POWER_RESET_PWD(1);	
}

static int mxs_usb5v_plugged(void)
{
	return (HW_POWER_STS.B.VDD5V_GT_VDDIO == 1 &&
		HW_POWER_STS.B.VDD5V_DROOP == 0);
}

void power_detect(void)
{
	if (mxs_usb5v_plugged())
		return;

	/* 检测干电池 */
	pin_set_type(POWER_DETECT_PIN, PIN_GPIO);
	pin_gpio_direction(POWER_DETECT_PIN, 0);

	while (pin_gpio_get(POWER_DETECT_PIN))
		printf("Power LowLevel, Poweroff\n");
}

static int strcasecmp(const char *s1, const char *s2)
{
	int r = 0;

	while (((s1 == s2) ||
		!(r = ((int)( tolower(*((unsigned char *)s1)))) - tolower(*((unsigned char *)s2))))
	       && (++s2, *s1++));

	return r;
}

static int strncasecmp(const char *s1, const char *s2, size_t n)
{
	int r = 0;

	while ( n &&
		((s1 == s2) ||
		 !(r = ((int)( tolower(*((unsigned char *)s1)))) - tolower(*((unsigned char *)s2)))) &&
		(--n, ++s2, *s1++));
	return r;
}

static uint32_t calc_checksum(void *addr, size_t sz)
{
	uint32_t sum = 0; 
	uint8_t *c = addr;

	while (sz--)
		sum += *c++;

	return sum;
}

void change_console_to_serial(void)
{
	char command_line[128];

	memset(command_line,0,sizeof(command_line));
	sprintf(command_line, "%s", "setenv stdout serial");
	run_command(command_line,0);
	memset(command_line,0,sizeof(command_line));
	sprintf(command_line, "%s", "setenv stderr serial");
	run_command(command_line,0);
}

void dm30_registe_file(const char *fname)
{
	struct patch_rom *rom = &g_rom;
	const char *ch1, *ch2;

	if (strncasecmp(fname, gromname, sizeof(gromname) - 1))
		return;

	ch1 = strchr(fname, 'V');
	if (!ch1)
		ch1 = strchr(fname, 'v');
	ch2 = strchr(fname, '.');
	if (!ch1 || !ch2 || ch1 > ch2)
		return;
	if (strcasecmp(ch2, ".bin"))
		return;

	++ch1;

	if (ch2 - ch1 > sizeof(rom->vers))
		return;

	if (strncasecmp(ch1, rom->vers, ch2 - ch1) > 0) {
		strncpy(rom->vers, ch1, ch2 - ch1);
		rom->vers[ch2 - ch1] = '\0';
		strncpy(rom->name, fname, sizeof(rom->name) - 1);
	}
}

static int dm30_check_sd(void)
{
	struct mmc *mmc;
	block_dev_desc_t *dev_desc=NULL;
	int part = 1, dev = 0, ret = 0;

	puts("Checking SD-Card ...\t");

	if (!mxs_mmc_is_plugged()) {
		ret = -ENODEV;
		goto out;
	}

	mmc = find_mmc_device(0);
	if (!mmc || mmc_init(mmc)) {
		ret = -EIO;
		goto out;
	}

	dev_desc = get_dev("mmc", dev);
	if (!dev_desc) {
		puts ("\n** Invalid boot device **\n");
		ret = -EIO;
		goto out;
	} 

	if (fat_register_device(dev_desc, part)) {
		printf ("\n** Unable to use mmc %d:%d for fatload **\n", dev, part);
		ret = -EIO;
		goto out;
	}

out:
	printf("%s\n", ret ? "ERROR" : "OK");

	if (ret == 0 && list_file_already == 0 ) {
		file_fat_ls("/");
		list_file_already = 1;
	}

	return ret;
}

static int verscmp(const char *vers1, const char *vers2)
{
	const char *ch;
	char vs1[64];
	char v[20];
	int i;

	memset(vs1, 0, sizeof(vs1));
	memset(v, 0, sizeof(v));
	strncpy(vs1, vers1, sizeof(vs1) -1);
	ch = vs1;

	while (!isdigit(*ch))
		++ch;

	for (i = 0; i < sizeof(v) && *ch; ++ch) {
		if (*ch == '.')
			continue;
		else if (isdigit(*ch))
			v[i++] = *ch;
		else
			break;
	}

	return strcasecmp(v, vers2);
}

static int dm30_check_rom(void)
{
	struct patch_rom *rom = &g_rom;
	uint8_t *addr = (uint8_t *)rom->addr_ddr;
	struct user_prog_hl *user;
	uint32_t checksum;
	long sz;

	puts("Checking Rom ...\t");
	lcd_refresh();

	if (!rom->name[0]) {
		printf("NO ROM found\n");
		return -ENOENT;
	}

	/* load patch-rom file */
	sz = file_fat_read(rom->name, (void *)addr, 0);
	if (sz <= 0) {
		printf("Unable to read \"%s\"\n", rom->name);
		return -ENOENT;
	}

	user = (struct user_prog_hl *)addr;
	/* check series-no */
	if (strncasecmp(user->series, "DM30", 6)) {
		printf("%s: series %.*s\n", __func__, 6, user->series);
		return -ENOENT;
	}

	/* check version */
	if (verscmp(user->vers, rom->vers)) {
		printf("Version ERROR: rom-version %s, file-verison %s\n",
		       user->vers, rom->vers);
		return -ENOENT;
	}

	
	checksum = calc_checksum(addr + APP_HEAD_SIZE, user->size);
	if (user->checksum != checksum) {
		printf("checksum error, 0x%08x, 0x%08x\n", user->checksum, checksum);
		return -EINVAL;
	}

	rom->size = sz;
	rom->checksum = calc_checksum(addr, rom->size);
	puts("OK\n");
	lcd_refresh();	
	return 0;
}

static int dm30_check_usb5v(void)
{
	printf("Checking USB5V ...\t");

	if (!mxs_usb5v_plugged()) {
		printf(" error, usb is not insert\n");
		return -ENODEV;
	}

	printf("OK\n");
	lcd_refresh();
	return 0;
}

static loff_t nand_logic2phys(nand_info_t *nand, loff_t ofs, loff_t logic)
{
	loff_t phys = 0;

	/* include the 'logic' block */
	while (ofs <= logic) {
		if (nand_block_isbad (nand, ofs & ~(nand->erasesize - 1)))
			phys += nand->erasesize;

		ofs += nand->erasesize;
	}

	phys += logic;
	if (phys != logic) {
		/* logic所在块已经检测，向后对齐一个block */
		logic = (logic + nand->erasesize) & (~(nand->erasesize - 1));
		phys = nand_logic2phys(nand, logic, phys);
	}

	return phys;
}

int dm30_upgrade_uboot(void)
{
	struct patch_rom *rom = &g_rom;
	uint8_t *addr = (uint8_t *)rom->addr_ddr;
	long sz;
	int ret=0;

	struct mmc *mmc;
	const int dev = 1;
	int blk;
	u32 n;
	u32 cnt;
#if 0
	ret = dm30_check_usb5v();
	if (ret) {
		printf("Error:Please insert USB and restart.\n");
		lcd_refresh();
		ret = -ENODEV;
		goto out;
	}
#endif
	ret = dm30_check_sd();
	if (ret) {
		printf("Error:No found SDCard\n");
		lcd_refresh();
		while(1);
		//ret = -ENODEV;
		//goto out;
	}

	/* load uboot file */
	sz = file_fat_read(g_uboot_name, (void *)addr, 0);
	if (sz <= 0) {
		//printf("Checking \"%s\" ...\tERROR\n", g_uboot_name);
		printf("Cannot found uboot %s !\n", g_uboot_name );
		lcd_refresh();
		ret = 1;
		goto out;
	}

	cnt = sz / SECTION_SIZE;
	if (sz % SECTION_SIZE) {
		cnt++;
	}

	mmc = find_mmc_device(dev);
	if (mmc) {
		if (mmc_init(mmc)) {
			printf("EMMC flash init ...\tERROR\n");
			lcd_refresh();
			//ret = -ENODEV;
			//goto out;
			while(1);  //stop!!!
			
		}
	}

	// Read MBR to get the start section of the first partition.
	n = mmc->block_dev.block_read(dev, 0, 1, addr + (cnt * SECTION_SIZE));
	if (n < 1) {
		printf("EMMC flash read MBR ...\tERROR\n");
		lcd_refresh();
		//ret = -ENODEV;
		//goto out;
		while(1); //stop!!!
	}
	blk = *((u16*)(addr + (cnt * SECTION_SIZE) + 0x1c6));
	g_first_partition_section = blk;
	blk += DM30_FLASH_OFFSET_BOOTLOADER / SECTION_SIZE;
	
	printf("Write UBoot: dev # %d, block # %d, count %d ... ",
		dev, blk, cnt);
	lcd_refresh();
	n = mmc->block_dev.block_write(dev, blk, cnt, addr);
	printf("%s\n", (n == cnt) ? "OK" : "ERROR");
	if (n != cnt) {
		//ret = -ENODEV;
		//goto out;
		while(1); //stop!!!
	}


out:
	//printf("Update UBoot ...\t%s\n", ret ? "ERROR" : "OK");
	if( ret == 0 ) {
		printf("upgrade uboot ok.\n");
		lcd_refresh();
	}

	return ret;
}

int dm30_upgrade_app(void)
{
	struct patch_rom *rom = &g_rom;
	uint8_t *addr = (uint8_t *)rom->addr_ddr;
	struct user_prog_hl *prog;
	struct database_hl *db;
	uint32_t checksum;
	int ret;

	struct mmc *mmc;
	const int dev = 1;
	int blk;
	u32 prog_size;
	u32 db_size;
	u32 n;
	u32 cnt=0;
#if 0
	ret = dm30_check_usb5v();
	if (ret) {
		printf("Error:Please insert USB and restart.\n");
		lcd_refresh();	
		goto out;
	}
#endif
	ret = dm30_check_sd();
	if (ret) {
		printf("Error:No found SDCard.\n");
		lcd_refresh();	
		//goto out;
		while(1);
	}


	ret = dm30_check_rom();
	if (ret) {
		lcd_refresh();
		//goto out;
		while(1);
	}
	prog = (struct user_prog_hl *)addr;
	prog_size = APP_HEAD_SIZE + prog->size;
	//printf("prog->size=0x%04X\t\n", prog->size);
	cnt = prog_size / SECTION_SIZE;
	if (prog_size % SECTION_SIZE) {
		cnt++;
	}


	mmc = find_mmc_device(dev);
	if (mmc) {
		if (mmc_init(mmc)) {
			printf("EMMC flash init ...\tERROR\n");
			lcd_refresh();
			ret = -ENOENT;
			//goto out;
			while(1);
		}
	}

	// Read MBR to get the start section of the first partition.
	//printf("Read firmware from SDCard...\t\n");
	//lcd_refresh();
	n = mmc->block_dev.block_read(dev, 0, 1, addr + (cnt * SECTION_SIZE));
	if (n < 1) {
		printf("EMMC flash read MBR ...\tERROR\n");
		lcd_refresh();
		ret = -ENOENT;
		//goto out;
		while(1);
	}	
	//printf("check read data[0~3]: %02X %02X %02X %02X!!\t\n", addr[0],addr[1],addr[2],addr[3]);
	blk = *((u16*)(addr + (cnt * SECTION_SIZE) + 0x1c6));
	g_first_partition_section = blk;
	blk += DM30_FLASH_OFFSET_USER_PROGRAM / SECTION_SIZE;		//start block number to write firmware.
	
	//ret = dm30_check_rom();
	//if (ret) {
	//	lcd_refresh();
		//goto out;
	//	while(1);
	//}
	
#if 0
	prog = (struct user_prog_hl *)addr;
	prog_size = APP_HEAD_SIZE + prog->size;
	//printf("prog->size=0x%04X\t\n", prog->size);
	cnt = prog_size / SECTION_SIZE;
	if (prog_size % SECTION_SIZE) {
		cnt++;
	}
#endif
	printf("ROM file name ...\t%s\n", rom->name);
	printf("ROM version ...\t\t%s\n", prog->vers);
	lcd_refresh();

	if (cnt >= (DM30_FLASH_SIZE_USER_PROGRAM / SECTION_SIZE)) {
		printf("Checking program size ...\tToo large\n");
		lcd_refresh();
		ret = -ENOENT;
		//goto out;
		while(1);
	}

	// Write.
	printf("Write program: dev # %d, block # %d, count %d ...\t",
		dev, blk, cnt);
	lcd_refresh();
	n = mmc->block_dev.block_write(dev, blk, cnt, addr);
	printf("%s\n", (n == cnt) ? "OK" : "ERROR");
	lcd_refresh();
	if (n != cnt) {
		ret = -ENOENT;
		//goto out;
		while(1);
	}

	// Read back and verify.
	memset((void *)addr, 0, cnt * SECTION_SIZE);

	printf("Read program: dev # %d, block # %d, count %d ...\t",
		dev, blk, cnt);
	lcd_refresh();
	n = mmc->block_dev.block_read(dev, blk, cnt, addr);
	printf("%s\n", (n == cnt) ? "OK" : "ERROR");
	lcd_refresh();
	if (n != cnt) {
		ret = -ENOENT;
		//goto out;
		while(1);
	}


	printf("Verify program ...\t");
	lcd_refresh();
	checksum = calc_checksum((void *)(addr+APP_HEAD_SIZE), /*rom->size*/prog->size);
	//ret = (checksum != rom->checksum);
	//printf("\nchecksum1=0x%08X\n", checksum);
	//printf("checksum2=0x%08X\n", prog->checksum);
	ret = (checksum != prog->checksum);
	printf("%s\n", ret ? "ERROR" : "OK");
	lcd_refresh();
	if (ret) {
		//goto out;
		while(1);
	}

	////////////////////////////////////////////////////////////////////////////
	// Write database.

	blk = g_first_partition_section;
	blk += DM30_FLASH_OFFSET_DATABASE / SECTION_SIZE;

	addr += APP_DB_OFFSET;
	db = (struct database_hl *)addr;
	db_size = APP_HEAD_SIZE + db->size;
	cnt = db_size / SECTION_SIZE;
	if (db_size % SECTION_SIZE) {
		cnt++;
	}

	if (cnt >= (DM30_FLASH_SIZE_DATABASE / SECTION_SIZE)) {
		printf("Checking database size ...\tToo large\n");
		lcd_refresh();
		ret = -ENOENT;
		//goto out;
		while(1);
	}

	printf("Write database: dev # %d, block # %d, count %d ...\t",
		dev, blk, cnt);
	lcd_refresh();
	n = mmc->block_dev.block_write(dev, blk, cnt, addr);
	printf("%s\n", (n == cnt) ? "OK" : "ERROR");
	lcd_refresh();
	if (n != cnt) {
		ret = -ENOENT;
		//goto out;
		while(1);
	}

	// Read back and verify.
	memset((void *)addr, 0, cnt * SECTION_SIZE);

	printf("Read database: dev # %d, block # %d, count %d ...\t",
		dev, blk, cnt);
	lcd_refresh();
	n = mmc->block_dev.block_read(dev, blk, cnt, addr);
	printf("%s\n", (n == cnt) ? "OK" : "ERROR");
	lcd_refresh();
	if (n != cnt) {
		ret = -ENOENT;
		//goto out;
		while(1);
	}

	printf("Verify database ...\t");
	lcd_refresh();
	checksum = calc_checksum((void *)(addr + APP_HEAD_SIZE), db->size);
	ret = (checksum != db->checksum);
	printf("%s, checksum=0x%08x, size=0x%08x\n", ret ? "ERROR" : "OK", db->checksum, db->size);
	lcd_refresh();
	if (ret) {
		//goto out;
		while(1);
	}

//out:
	//printf("Update ROM ...\t\t%s\n", ret ? "ERROR" : "OK");
	printf("Update firmware ok!\n");
	lcd_refresh();

	return 0;
}

int dm30_patching(void)
{
	int ret=1;

	//add by cyliang@20171109 
	if( !dm30_patching_key() ) {
		printf("patching key no be press!!\n");
		change_console_to_serial();
		return ret;
	}
	printf("%s\n\n",DM30_BOOT_VERSION);
	lcd_refresh();
	
	//check USB power supply & SDCard.
	ret = dm30_check_usb5v();
	if (ret) {
		printf("Error:Please insert USB and restart.\n");
		lcd_refresh();
		while(1);
	}

	ret = dm30_check_sd();
	if (ret) {
		printf("Error:No found SDCard\n");
		lcd_refresh();
		while(1);
	}
	
	
	ret = dm30_upgrade_uboot();
	ret = dm30_upgrade_app();
	
	//if press patch key , but update failed, then print error message , and stop here, let worker to checking.
	if( ret !=0 ) {
		printf("Patching failed, please check it...\n");
		lcd_refresh();
		while(1);
	} else {
		printf("Please restart!\n");
		lcd_refresh();
		while(1);
	}

	return ret;
}

#define VCOM_INFO_MAGIC	0x1234
typedef struct _vcom_info_{
	uint16_t magic;
	uint16_t data1;
	uint16_t data2;
} VCOMInfo;

uint16_t load_vcom()
{
	return 0;
}

int load_logo(uint32_t address)
{
	struct mmc *mmc;
	const int dev = 1;
	int first_partion_blk;
	int blk;
	u32 n;
	u32 cnt;
	u32 logo_size;
	u32 read_size;
	uint8_t buf[SECTION_SIZE];
	uint8_t *pDst;

	mmc = find_mmc_device(dev);
	if (mmc) {
		if (mmc_init(mmc)) {
			printf("EMMC flash init failed!\n");
			return -ENOENT;
		}
	}

	// Read MBR to get the start section of the first partition.
	n = mmc->block_dev.block_read(dev, 0, 1, (uint8_t*)address);
	if (n < 1) {
		printf("EMMC flash read MBR failed!\n");
		return -ENOENT;
	};
	first_partion_blk = *((u16*)((uint8_t*)address + 0x1c6));

	// Read logo to address.
	blk = first_partion_blk + ((DM30_FLASH_OFFSET_DATABASE + APP_HEAD_SIZE) / SECTION_SIZE);
	cnt = 937+1;	//937 sector is 937*512=479744, so remain extra add 1 sector to read out.

	printf("Read logo data ... \t");
	mmc->block_dev.block_read(dev, blk, cnt, (uint8_t*)address);
	printf("OK\n");

	return 0;
}

int load_program(uint32_t address)
{
	struct user_prog_hl *prog;
	int ret;

	struct mmc *mmc;
	const int dev = 1;
	int first_partion_blk;
	int blk;
	u32 n;
	u32 cnt;

	mmc = find_mmc_device(dev);
	if (mmc) {
		if (mmc_init(mmc)) {
			printf("EMMC flash init failed!\n");
			return -ENOENT;
		}
	}

	// Read MBR to get the start section of the first partition.
	n = mmc->block_dev.block_read(dev, 0, 1, (uint8_t*)address);
	if (n < 1) {
		printf("EMMC flash read MBR failed!\n");
		return -ENOENT;
	};
	first_partion_blk = *((u16*)((uint8_t*)address + 0x1c6));

	// Read rom head to get rom size.
	blk = first_partion_blk + (DM30_FLASH_OFFSET_USER_PROGRAM / SECTION_SIZE);
	cnt = (APP_HEAD_SIZE / SECTION_SIZE) + 1;
	n = mmc->block_dev.block_read(dev, blk, cnt, (uint8_t*)address);
	if (n != cnt) {
		printf("Read rom head faile.\n");
		return -ENOENT;
	}

	// Do some checking.
	prog = (struct user_prog_hl *)address;
	ret = strcasecmp(prog->series, "DM30");
	if (ret) {
		printf("User-Program Series ERROR\n");
		return -ENOENT;
	}

	// Read rom to address.
	blk = first_partion_blk + (DM30_FLASH_OFFSET_USER_PROGRAM + APP_HEAD_SIZE) / SECTION_SIZE;
	cnt = prog->size / SECTION_SIZE;
	if (prog->size % SECTION_SIZE) {
		cnt++;
	}

	if ((blk + cnt) >= DM30_FLASH_OFFSET_DATABASE / SECTION_SIZE) {
		printf("Rom is too large.\n");
		return -ENOENT;
	}

	printf("Read rom: dev # %d, block # %d, count %d ...\t",
		dev, blk, cnt);
	n = mmc->block_dev.block_read(dev, blk, cnt, (uint8_t*)address);	
	printf("%d blocks read: %s\n",
		n, (n == cnt) ? "OK" : "ERROR");
	if (n != cnt) {
		printf("Read rom faile.\n");
		return -ENOENT;
	}

	return 0;
}

void dm30_boot(void)
{
	uint32_t addr = DM30_DDR_OFFSET_USER_PROGRAM;
	void (*entry)(uint32_t version) = (void *)addr;
	uint32_t version = 0xdead << 16 | (DM30_UBOOT_VERSION << 8) | DM30_UBOOT_PATCHLEVEL;
	uint32_t starttime;

	change_console_to_serial();
	printf("start load program...\r\n");
	starttime = HW_DIGCTL_MICROSECONDS_RD();
	if (!load_program(addr)) {
		printf("load program: %d us\n", (HW_DIGCTL_MICROSECONDS_RD()-starttime));
		printf("u-boot version: %d.%d(0x%x)\n",
		       DM30_UBOOT_VERSION, DM30_UBOOT_PATCHLEVEL, version);
		entry(version);
	}
}

int dm30_patching_key(void)
{
	return !pin_gpio_get(DM30_KEY_RCTRL) &&
	       !pin_gpio_get(DM30_KEY_RSHIFT);
}

/* 下面是开机电压检测代码 */

static int BATTlastBattery = 0;

#define BATDET_CTRL 	PINID_PWM4
static struct pin_desc power_pins_desc[] = {
	{ BATDET_CTRL,  PIN_GPIO, PAD_8MA, PAD_3V3, 1 },
};

static struct pin_group power_pins = {
	.pins 	 = power_pins_desc, 
	.nr_pins = ARRAY_SIZE(power_pins_desc),
};

static void BATDET_CTRL_init(void)
{
	pin_set_group(&power_pins);
	pin_gpio_set(BATDET_CTRL, 1);
	pin_gpio_set(BATDET_CTRL, 0);
	pin_gpio_direction(BATDET_CTRL, 1);
}

void battery_detect_init(void)
{
	BATDET_CTRL_init();
#if 1
	mxs_reset_block((void *)HW_LRADC_CTRL0_ADDR, 0);
#endif

	BW_LRADC_CTRL0_TOUCH_DETECT_ENABLE(BV_LRADC_CTRL0_TOUCH_DETECT_ENABLE__OFF);
	//设置使能XPLUS状态位,并对应设置虚拟通道
	BW_LRADC_CTRL0_XPLUS_ENABLE(BV_LRADC_CTRL0_XPLUS_ENABLE__OFF);
	//BW_LRADC_CTRL0_SCHEDULE(0x04);
	BW_LRADC_CTRL0_YPLUS_ENABLE(BV_LRADC_CTRL0_YPLUS_ENABLE__OFF);
	//使能LRADC2中断标志位清零、中断允许
	BW_LRADC_CTRL1_LRADC2_IRQ(BV_LRADC_CTRL1_LRADC2_IRQ__CLEAR);
	BW_LRADC_CTRL1_LRADC2_IRQ_EN(BV_LRADC_CTRL1_LRADC2_IRQ_EN__ENABLE);
	//使能LRADC2中断标志位清零、中断允许
	BW_LRADC_CTRL1_LRADC3_IRQ(BV_LRADC_CTRL1_LRADC3_IRQ__CLEAR);
	BW_LRADC_CTRL1_LRADC3_IRQ_EN(BV_LRADC_CTRL1_LRADC3_IRQ_EN__ENABLE);
	//BW_LRADC_CTRL2_DIVIDE_BY_TWO(0x04);

	//去掉开始的2次//设置hightime为83NS//设置时钟正常工作
	HW_LRADC_CTRL3_WR(BF_LRADC_CTRL3_DISCARD(2)|BF_LRADC_CTRL3_HIGH_TIME(1)|
			  BW_LRADC_CTRL3_CYCLE_TIME(0)|BF_LRADC_CTRL3_INVERT_CLOCK(0));

	HW_LRADC_CTRL4_WR(BF_LRADC_CTRL4_LRADC2SELECT(2)|BF_LRADC_CTRL4_LRADC3SELECT(3)) ;
}

int get_battery_voltage(void)
{
	unsigned int status,battery_value;

#if 1
	pin_gpio_set(BATDET_CTRL, 1);
#endif
	udelay(10);

	BW_LRADC_CTRL0_SCHEDULE(0x04);
	BW_LRADC_CTRL2_DIVIDE_BY_TWO(0x04);
	HW_LRADC_DELAYn_WR(0, (BF_LRADC_DELAYn_TRIGGER_LRADCS(0x04) |
			       BF_LRADC_DELAYn_KICK(1) |
			       BF_LRADC_DELAYn_TRIGGER_DELAYS(0x1) |
			       BF_LRADC_DELAYn_DELAY(6) ) ); 
	BW_LRADC_CTRL1_LRADC2_IRQ(BV_LRADC_CTRL1_LRADC2_IRQ__CLEAR);
	HW_LRADC_CHn_WR(2,( BF_LRADC_CHn_ACCUMULATE(1)|BF_LRADC_CHn_NUM_SAMPLES(5)|BF_LRADC_CHn_VALUE(0) ));

	do{
		udelay(2);
		status = HW_LRADC_CTRL1_RD() ;
	}while((status&0x04) == 0);
	BW_LRADC_CTRL1_LRADC2_IRQ(BV_LRADC_CTRL1_LRADC2_IRQ__CLEAR);

	//获取电压AD值
	battery_value = HW_LRADC_CHn_RD(2) ;	
	battery_value = battery_value&0x3ffff;

	REG_CLR(PINCTRL_BASE+PINCTRL_DOUT(1),(1<<30) );	
	/*	|t - g_nlastBattery| < 20 */
	/* avoid the ad value unstable */
	if(BATTlastBattery == 0)
		BATTlastBattery = battery_value;
	else if(battery_value<=(BATTlastBattery - 150))
		BATTlastBattery = battery_value;
	else if(battery_value>=(BATTlastBattery + 150))
		BATTlastBattery = battery_value;
	else
		battery_value = BATTlastBattery;
	return (int)(battery_value);	
}

enum batt_voltage_level {
	VOL_3_0, 
	VOL_2_8, 
	VOL_2_6, 
	VOL_2_4, 
	VOL_2_2, 
	VOL_2_1, 
	VOL_2_0, 
	VOL_1_8, 
	VOL_NONE, 
};

static const int battery_voltage_ref[] = {
	[VOL_3_0] = 19800, 	/* 3.0V */
	[VOL_2_8] = 18634, 	/* 2.8V */
	[VOL_2_6] = 17168, 	/* 2.6V */
	[VOL_2_4] = 15890, 	/* 2.4V */
	[VOL_2_2] = 14510, 	/* 2.2V */
	[VOL_2_1] = 13983, 	/* 2.1V */
	[VOL_2_0] = 13317, 	/* 2.0V */
	[VOL_1_8] = 12052, 	/* 1.8V */
};

static int get_battery_refvoltage_from_nand(void)
{
	int ret = 0;
	struct user_prog_hl user;
	size_t size = sizeof(user);
	nand_info_t *nand = &nand_info[nand_curr_device];
	loff_t off = nand_logic2phys(nand, 0, g_rom.addr_nand);

	ret = nand_read_skip_bad(nand, off, &size, (u_char *)&user);
	if (ret) {
		printf("Read User-Header ERROR\n");
		goto err;
	}

	ret = strcasecmp(user.series, "DM30");
	if (ret) {
		printf("User-Program Series ERROR\n");
		goto err;
	}

	printf("Battery ref voltage(store in NandFlash): %d\n", user.refvol);
	return user.refvol;

err:
	return -1;
}

static int get_battery_refvoltage_from_sd(void)
{
	int ret = 0;
	uint32_t addr = 0x40000000;
	const char *fn = "battery_voltage.cfg";

	if (dm30_check_sd()) {
		printf("No SDcard found, please insert SDcard");
		goto err;
	}

	if (file_fat_read(fn, (void *)addr, 0) < 0) {
		printf("Unable to read \"%s\"\n", fn);
		goto err;
	}

	ret = (int)simple_strtoul((const char *)addr, NULL, 10);
err:
	return ret;
}

static int get_battery_refvoltage(void)
{
	if (1)
		return get_battery_refvoltage_from_nand();
	else
		return get_battery_refvoltage_from_sd();
}

static int is_voltage_inrange(int vol)
{
	return (vol >= battery_voltage_ref[VOL_1_8] &&
		vol <= battery_voltage_ref[VOL_3_0]);
}


static int get_preset_poweron_voltage(void)
{
	struct mmc *mmc;
	const int dev = 1;
	int blk;
	u32 n;
	struct patch_rom *rom = &g_rom;
	uint8_t *addr = (uint8_t *)rom->addr_ddr;	
	struct user_prog_hl* pRomheader=(struct user_prog_hl*)(rom->addr_ddr);
	
	mmc = find_mmc_device(dev);
	if (mmc) {
		if (mmc_init(mmc)) {
			printf("EMMC memory init ...\tERROR\n");
			return -1;
		}
	}


	// Read MBR to get the start section of the first partition.
	n = mmc->block_dev.block_read(dev, 0, 1, addr );
	if (n < 1) {
		printf("EMMC memory read MBR ...\tERROR\n");
		return -1;
	}
	
	blk = *((u16*)(addr + 0x1c6));	//get first sector of 1'st partition.
	printf("Get first sector number=%d\n",blk);
	blk += DM30_FLASH_OFFSET_USER_PROGRAM / SECTION_SIZE;	//get sector number of romcode.
	
	n = mmc->block_dev.block_read(dev, blk, 1, addr);
	if( n<1 ) {
		printf("EMMC memory read rom ...\tERROR\n");
		return -1;
	}
	printf("ROM preset power on voltage ad = %d\n", pRomheader->refvol);
	return (pRomheader->refvol);
	
}

#define CONFIG_DM30_DEFAULT_BOOT_VOLTAGE	(15250)

void dm30_power_detect(void)
{
	int vol, refvol;

	/* if usb insert, nothing to do */
	if (mxs_usb5v_plugged())
		return;
		
	battery_detect_init();
	vol = get_battery_voltage();
	printf("Battery voltage: %d\n", vol);

	//refvol = get_battery_refvoltage();
	refvol = get_preset_poweron_voltage();
	if( refvol == -1 )
	{
		printf("ROM preset power on voltage is invalid.\n");
		printf("set to default value : %d.\n",CONFIG_DM30_DEFAULT_BOOT_VOLTAGE);
		refvol = CONFIG_DM30_DEFAULT_BOOT_VOLTAGE;
	}

	if (!is_voltage_inrange(refvol)) {
		refvol = CONFIG_DM30_DEFAULT_BOOT_VOLTAGE;
		printf("reference voltage not in range, use default voltage(%d)\n", refvol);
	}

	if (vol < refvol) {
		printf("lcoal voltage is too low , system power off!\n");
		poweroff();
	}
}
