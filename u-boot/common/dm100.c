/*
 * ==========================================================================
 *
 *       Filename:  dm100.c
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

extern int fat_register_device(block_dev_desc_t *dev_desc, int part_no);
extern long file_fat_read(const char *filename, void *buffer, unsigned long maxsize);
extern int file_fat_ls(const char *dir);
extern void lcd_enable (void);
extern int mxs_mmc_is_plugged(void);
extern int dm100_patching_key(void);

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
	.addr_ddr = DM100_DDR_OFFSET_USER_PROGRAM, 
	.addr_nand = DM100_NF_OFFSET_USER_PROGRAM, 
};

static const char gromname[]= "Pomera_Update_Dm100_V";

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

/*
 * 1）首先检查是否插入USB，没有插入USB，提示插入USB。当插入USB继续，否则等待。或按ESC退出。
 *   （为了防止电池供电在patching的过程断电）
 * 2）检查是否有插入SDCard，没有插入SDCard或不能识别SDCard，提示没有发现SDCard信息。
 *    等待插入SDCard卡后继续，否则等待，或按ESC退出。
 * 3）检索SDCard内所有Pomera_Update_Dm100开始的规则的文件名，并找成版本号（例子的版本号为：02040）
 *    最大的文件作为候选patching文件。
 * 4）如果没有文件存在，提示不存在patching文件。
 * 5）如果找到该文件后，从该文件名得到该系列号（DM100）和版本号（02040），
 *    与该Patching文件内部保存有的系列号和版本号
 *   （参见老大的映射文档）。比较是否一致。
 *    a)如果序列号不一致，提示不是系列号错误。按ESC键退出。
 *    b)如果版本号不一致，提示不是版本号错误。按ESC键退出。
 * 6）如果都OK，提示当前需要升级的版本号，为用户是否进行升级？ 如果Y，升级开始。如果N，退出。
 */
static void patching_err(const char *str)
{
	printf(" *** PATCHING ERR: %s\n", str);
}

void dm100_registe_file(const char *fname)
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

static int dm100_check_sd(void)
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

	file_fat_ls("/");

out:
	printf("%s\n", ret ? "ERROR" : "OK");
	return ret;
}

/**
 * verscmp - 版本号比较
 * @vers1 - 这是从文件中读出的版本号，需要先进行过滤
 * @vers2 - 这是从文件名中取出的版本号
 *
 * 相同返回0, 不同返回非0
 */
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

static int dm100_check_rom(void)
{
	struct patch_rom *rom = &g_rom;
	uint8_t *addr = (uint8_t *)rom->addr_ddr;
	nand_info_t *nand = &nand_info[nand_curr_device];
	struct user_prog_hl *user;
	long sz;

	puts("[PATCHING] Checking Rom ...\n");

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
	if (strncasecmp(user->series, "DM100", 6)) {
		printf("%s: series %.*s\n", __func__, 6, user->series);
		return -ENOENT;
	}

	/* check version */
	if (verscmp(user->vers, rom->vers)) {
		printf("Version ERROR: rom-version %s, file-verison %s\n",
		       user->vers, rom->vers);
		return -ENOENT;
	}
	if (user->checksum != calc_checksum(addr + nand->writesize, user->size)) {
		printf("checksum error, %u\n", user->checksum);
		return -EINVAL;
	}

	rom->size = sz;
	rom->checksum = calc_checksum(addr, rom->size);
	puts("OK\n");
	return 0;
}

static int dm100_check_usb5v(void)
{
	printf("Checking USB5V ...\t");

	if (!mxs_usb5v_plugged()) {
		printf(" error, usb is not insert\n");
		return -ENODEV;
	}

	printf("OK\n");
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

/**
 * do_patching - 将数据从DDR的addr地址开始，写入到nand的to地址开始
 */
static int do_patching(uint32_t addr, loff_t addr_nand)
{
	nand_info_t *nand = &nand_info[nand_curr_device];
	loff_t off = nand_logic2phys(nand, 0, addr_nand);
	struct patch_rom *rom = &g_rom;
	size_t size;
	uint32_t checksum;
	int ret;

	printf("program address %08x, erase-length %08x\n",
	       (uint32_t)off,(uint32_t)(DM100_NF_OFFSET_FILESYSTEM1 - off));

	/* erase, 将Reserve区域也擦除掉，以防前面坏块太多超出database区 */
	puts("Erase ...\t");
	nand_erase_options_t opts;
	memset(&opts, 0, sizeof(opts));
	opts.offset = off;
	opts.length = DM100_NF_OFFSET_FILESYSTEM1 - off;
	opts.quiet  = 1;
	ret = nand_erase_opts(nand, &opts);
	printf("%s\n", ret ? "ERROR" : "OK");
	if (ret)
		goto out;

	/* write */
	puts("Write ...\t");
	size = (rom->size + nand->writesize - 1) & (~(nand->writesize - 1));
	ret = nand_write_skip_bad(nand, off, &size, (u_char *)addr);
	printf("%s\n", ret ? "ERROR" : "OK");
	if (ret)
		goto out;

	/* Verify */
	puts("Verify ...\t");
	size = (rom->size + nand->writesize - 1) & (~(nand->writesize - 1));
	memset((void *)addr, 0, size);
	ret = nand_read_skip_bad(nand, off, &size, (u_char *)addr);
	if (ret) {
		printf("Read ERROR\n");
		goto out;
	}

	checksum = calc_checksum((void *)addr, rom->size);
	ret = (checksum != rom->checksum);
	printf("%s\n", ret ? "ERROR" : "OK");

out:
	return ret;
}

static int dm100_patching(void)
{
	int ret;
	struct patch_rom *rom = &g_rom;
	struct user_prog_hl *user = (struct user_prog_hl *)rom->addr_ddr;

	ret = dm100_check_usb5v();
	if (ret) {
		patching_err("Please insert USB and restart");
		goto out;
	}

	ret = dm100_check_sd();
	if (ret) {
		patching_err("No SDcard found, please insert SDcard");
		goto out;
	}

	ret = dm100_check_rom();
	if (ret) {
		patching_err("No ROM found");
		goto out;
	}

	printf("Updating %s, Version: \"%s\"...[Y/N]\n", rom->name, user->vers);

	ret = do_patching(rom->addr_ddr, rom->addr_nand);

out:
	if (!ret) {
		printf("Patching success, please restart device\n");
		while (1)
			/* wait here for restart */;
	} else
		printf("Patching fail\n");

	return ret;
}

int load_logo(uint32_t address)
{
	nand_info_t *nand = &nand_info[nand_curr_device];
	struct logo_hl logo;
	size_t size = sizeof(logo);
	loff_t off;
	int ret;

	off = nand_logic2phys(nand, 0, DM100_NF_OFFSET_DATABASE + nand->writesize);

	ret = nand_read_skip_bad(nand, off, &size, (u_char *)&logo);
	if (ret) {
		printf("Read logo-Header ERROR\n");
		goto out;
	}

	if (logo.width > 801 || logo.heigh > 600) {
		printf("logo error(%d x %d)\n", logo.width, logo.heigh);
		ret = -1;
		goto out;
	}

	off += sizeof(logo);
	size = logo.width * logo.heigh;
	printf("load logo(%u bytes) from nand %08lx\n", size, (long)off);
	ret = nand_read_skip_bad(nand, off, &size, (u_char *)address);
	if (ret) {
		printf("Read logo ERROR\n");
		goto out;
	}

out:
	return ret;
}

int load_program(uint32_t address)
{
	struct patch_rom *rom = &g_rom;
	nand_info_t *nand = &nand_info[nand_curr_device];
	loff_t off = nand_logic2phys(nand, 0, rom->addr_nand);
	struct user_prog_hl user;
	size_t size = sizeof(user);
	int ret = 0;
	static int loaded = 0;

	if (loaded)
		goto out;

	printf("load program address %08lx\n", (unsigned long)off);
	ret = nand_read_skip_bad(nand, off, &size, (u_char *)&user);
	if (ret) {
		printf("Read User-Header ERROR\n");
		goto out;
	}

	ret = strcasecmp(user.series, "DM100");
	if (ret) {
		printf("User-Program Series ERROR\n");
		goto out;
	}

	off += nand->writesize;
	size = user.size;
	ret = nand_read_skip_bad(nand, off, &size, (u_char *)address);
	if (ret) {
		printf("Read User-Program ERROR\n");
		goto out;
	}

	++loaded;

out:
	return ret;
}

void dm100_boot(void)
{
	uint32_t addr = DM100_DDR_OFFSET_USER_PROGRAM;
	void (*entry)(uint32_t version) = (void *)addr;
	uint32_t version = 0xdead << 16 | (DM100_UBOOT_VERSION << 8) | DM100_UBOOT_PATCHLEVEL;

	if (dm100_patching_key()) {
		printf("%s\n", DM100_BOOT_VERSION);
		if (dm100_patching())
			return;
	}

	change_console_to_serial();

	if (!load_program(addr)) {
		printf("load program: %d us\n", HW_DIGCTL_MICROSECONDS_RD());
		printf("u-boot version: %d.%d(0x%x)\n",
		       DM100_UBOOT_VERSION, DM100_UBOOT_PATCHLEVEL, version);
		entry(version);
	}
}

int dm100_patching_key(void)
{
	return !pin_gpio_get(DM100_KEY_BATDOOR) && 
	       !pin_gpio_get(DM100_KEY_CTRL) &&
	       !pin_gpio_get(DM100_KEY_RSHIFT);
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
#if 1
	pin_set_group(&power_pins);
	pin_gpio_set(BATDET_CTRL, 1);
	pin_gpio_set(BATDET_CTRL, 0);
	pin_gpio_direction(BATDET_CTRL, 1);
#else
	//pwm4 -> gpio
	REG_SET(PINCTRL_BASE+PINCTRL_MUXSEL(3),((1<<28)|(1<<29)) );
		//HW_PINCTRL_DRIVE(1)-->8mA
	//REG_SET(PINCTRL_BASE+PINCTRL_DRIVE(7), (1<<25) |(1<<24));
	REG_SET(PINCTRL_BASE+PINCTRL_DRIVE(7), (1<<25));	
	//HW_PINCTRL_DOUT(0)-->High
	REG_SET(PINCTRL_BASE+PINCTRL_DOUT(1),(1<<30) );
	//HW_PINCTRL_DOUT(0)-->LOW
	REG_CLR(PINCTRL_BASE+PINCTRL_DOUT(1),(1<<30) );
	//HW_PINCTRL_DOE(0)-->enable
	REG_SET(PINCTRL_BASE+PINCTRL_DOE(1), (1<<30) );
#endif
}

void battery_detect_init(void)
{
	BATDET_CTRL_init();
#if 1
	mxs_reset_block((void *)HW_LRADC_CTRL0_ADDR, 0);
#else
	/* Clear SFTRST */
	REG_CLR(HW_LRADC_CTRL0_ADDR, 1 << 31);
	while (REG_RD(HW_LRADC_CTRL0_ADDR) & (1 << 31))
		;

	/* Clear CLKGATE */
	REG_CLR(HW_LRADC_CTRL0_ADDR, 1 << 30);

	/* Set SFTRST and wait until CLKGATE is set */
	REG_SET(HW_LRADC_CTRL0_ADDR, 1 << 31);
	while (!(REG_RD(HW_LRADC_CTRL0_ADDR) & (1 << 30)))
		;

	/* Clear SFTRST and CLKGATE */
	REG_CLR(HW_LRADC_CTRL0_ADDR, 1 << 31);
	REG_CLR(HW_LRADC_CTRL0_ADDR, 1 << 30);
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
	//设置延时参数
#if 0
	//HW_LRADC_DELAYn_WR(0, (BF_LRADC_DELAYn_TRIGGER_LRADCS(0x04) |
				 BF_LRADC_DELAYn_KICK(1) |
				 BF_LRADC_DELAYn_TRIGGER_DELAYS(0x1) |
				 BF_LRADC_DELAYn_DELAY(1) ) ); 
#endif
}

int get_battery_voltage(void)
{
	unsigned int status,battery_value;

#if 1
	pin_gpio_set(BATDET_CTRL, 1);
#else
	//HW_PINCTRL_DOUT(0)-->High
	REG_SET(PINCTRL_BASE+PINCTRL_DOUT(1),(1<<30) );	
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

	//等待中断挂起,挂起后中断清零
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

#if 0
static int get_battery_level(void)
{
	int vol;
	int level;

	vol = get_battery_voltage();
	printf("Battery voltage: %d\n", vol);

	for (level = 0; level < ARRAY_SIZE(battery_voltage_ref), ++level) {
		if (vol > battery_voltage_ref[level])
			return level;
	}

	return VOL_NONE;
}
#endif

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

	ret = strcasecmp(user.series, "DM100");
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

	if (dm100_check_sd()) {
		patching_err("No SDcard found, please insert SDcard");
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

/*
 * 如果从nand中的rom中读到的AD值无效(不在指定范围)，
 * 则使用此值作为开机参考电压
 */
#define CONFIG_DM100_DEFAULT_BOOT_VOLTAGE	(15250)

void dm100_power_detect(void)
{
	int vol, refvol;

	/* if usb insert, nothing to do */
	if (mxs_usb5v_plugged())
		return;

	battery_detect_init();
	vol = get_battery_voltage();
	printf("Battery voltage: %d\n", vol);

	refvol = get_battery_refvoltage();
	printf("Battery refernce voltage(store in NandFlash): %d\n", refvol);

	if (!is_voltage_inrange(refvol)) {
		refvol = CONFIG_DM100_DEFAULT_BOOT_VOLTAGE;
		printf("reference voltage not in range, use default voltage(%d)\n", refvol);
	}

	if (vol < refvol)
		poweroff();
}
