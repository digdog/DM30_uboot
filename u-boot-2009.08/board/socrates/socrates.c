/*
 * (C) Copyright 2008
 * Sergei Poselenov, Emcraft Systems, sposelenov@emcraft.com.
 *
 * Copyright 2004 Freescale Semiconductor.
 * (C) Copyright 2002,2003, Motorola Inc.
 * Xianghua Xiao, (X.Xiao@motorola.com)
 *
 * (C) Copyright 2002 Scott McNutt <smcnutt@artesyncp.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.         See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <pci.h>
#include <asm/processor.h>
#include <asm/immap_85xx.h>
#include <ioports.h>
#include <flash.h>
#include <libfdt.h>
#include <fdt_support.h>
#include <asm/io.h>
#include <i2c.h>
#include <mb862xx.h>
#include <video_fb.h>
#include "upm_table.h"

DECLARE_GLOBAL_DATA_PTR;

extern flash_info_t flash_info[];	/* FLASH chips info */
extern GraphicDevice mb862xx;

void local_bus_init (void);
ulong flash_get_size (ulong base, int banknum);

int checkboard (void)
{
	volatile ccsr_gur_t *gur = (void *)(CONFIG_SYS_MPC85xx_GUTS_ADDR);

	char *src;
	int f;
	char *s = getenv("serial#");

	puts("Board: Socrates");
	if (s != NULL) {
		puts(", serial# ");
		puts(s);
	}
	putc('\n');

#ifdef CONFIG_PCI
	/* Check the PCI_clk sel bit */
	if (in_be32(&gur->porpllsr) & (1<<15)) {
		src = "SYSCLK";
		f = CONFIG_SYS_CLK_FREQ;
	} else {
		src = "PCI_CLK";
		f = CONFIG_PCI_CLK_FREQ;
	}
	printf ("PCI1:  32 bit, %d MHz (%s)\n",	f/1000000, src);
#else
	printf ("PCI1:  disabled\n");
#endif

	/*
	 * Initialize local bus.
	 */
	local_bus_init ();
	return 0;
}

int misc_init_r (void)
{
	volatile ccsr_lbc_t *memctl = (void *)(CONFIG_SYS_MPC85xx_LBC_ADDR);

	/*
	 * Adjust flash start and offset to detected values
	 */
	gd->bd->bi_flashstart = 0 - gd->bd->bi_flashsize;
	gd->bd->bi_flashoffset = 0;

	/*
	 * Check if boot FLASH isn't max size
	 */
	if (gd->bd->bi_flashsize < (0 - CONFIG_SYS_FLASH0)) {
		memctl->or0 = gd->bd->bi_flashstart | (CONFIG_SYS_OR0_PRELIM & 0x00007fff);
		memctl->br0 = gd->bd->bi_flashstart | (CONFIG_SYS_BR0_PRELIM & 0x00007fff);

		/*
		 * Re-check to get correct base address
		 */
		flash_get_size(gd->bd->bi_flashstart, CONFIG_SYS_MAX_FLASH_BANKS - 1);
	}

	/*
	 * Check if only one FLASH bank is available
	 */
	if (gd->bd->bi_flashsize != CONFIG_SYS_MAX_FLASH_BANKS * (0 - CONFIG_SYS_FLASH0)) {
		memctl->or1 = 0;
		memctl->br1 = 0;

		/*
		 * Re-do flash protection upon new addresses
		 */
		flash_protect (FLAG_PROTECT_CLEAR,
			       gd->bd->bi_flashstart, 0xffffffff,
			       &flash_info[CONFIG_SYS_MAX_FLASH_BANKS - 1]);

		/* Monitor protection ON by default */
		flash_protect (FLAG_PROTECT_SET,
			       CONFIG_SYS_MONITOR_BASE, CONFIG_SYS_MONITOR_BASE + monitor_flash_len - 1,
			       &flash_info[CONFIG_SYS_MAX_FLASH_BANKS - 1]);

		/* Environment protection ON by default */
		flash_protect (FLAG_PROTECT_SET,
			       CONFIG_ENV_ADDR,
			       CONFIG_ENV_ADDR + CONFIG_ENV_SECT_SIZE - 1,
			       &flash_info[CONFIG_SYS_MAX_FLASH_BANKS - 1]);

		/* Redundant environment protection ON by default */
		flash_protect (FLAG_PROTECT_SET,
			       CONFIG_ENV_ADDR_REDUND,
			       CONFIG_ENV_ADDR_REDUND + CONFIG_ENV_SECT_SIZE - 1,
			       &flash_info[CONFIG_SYS_MAX_FLASH_BANKS - 1]);
	}

	return 0;
}

/*
 * Initialize Local Bus
 */
void local_bus_init (void)
{
	volatile ccsr_lbc_t *lbc = (void *)(CONFIG_SYS_MPC85xx_LBC_ADDR);
	volatile ccsr_local_ecm_t *ecm = (void *)(CONFIG_SYS_MPC85xx_ECM_ADDR);
	sys_info_t sysinfo;
	uint clkdiv;
	uint lbc_mhz;
	uint lcrr = CONFIG_SYS_LBC_LCRR;

	get_sys_info (&sysinfo);
	clkdiv = lbc->lcrr & LCRR_CLKDIV;
	lbc_mhz = sysinfo.freqSystemBus / 1000000 / clkdiv;

	/* Disable PLL bypass for Local Bus Clock >= 66 MHz */
	if (lbc_mhz >= 66)
		lcrr &= ~LCRR_DBYP;	/* DLL Enabled */
	else
		lcrr |= LCRR_DBYP;	/* DLL Bypass */

	out_be32 (&lbc->lcrr, lcrr);
	asm ("sync;isync;msync");

	out_be32 (&lbc->ltesr, 0xffffffff);	/* Clear LBC error interrupts */
	out_be32 (&lbc->lteir, 0xffffffff);	/* Enable LBC error interrupts */
	out_be32 (&ecm->eedr, 0xffffffff);	/* Clear ecm errors */
	out_be32 (&ecm->eeer, 0xffffffff);	/* Enable ecm errors */

	/* Init UPMA for FPGA access */
	out_be32 (&lbc->mamr, 0x44440); /* Use a customer-supplied value */
	upmconfig (UPMA, (uint *)UPMTableA, sizeof(UPMTableA)/sizeof(int));

	/* Init UPMB for Lime controller access */
	out_be32 (&lbc->mbmr, 0x444440); /* Use a customer-supplied value */
	upmconfig (UPMB, (uint *)UPMTableB, sizeof(UPMTableB)/sizeof(int));
}

#if defined(CONFIG_PCI)
/*
 * Initialize PCI Devices, report devices found.
 */

#ifndef CONFIG_PCI_PNP
static struct pci_config_table pci_mpc85xxads_config_table[] = {
	{PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID,
	 PCI_IDSEL_NUMBER, PCI_ANY_ID,
	 pci_cfgfunc_config_device, {PCI_ENET0_IOADDR,
				     PCI_ENET0_MEMADDR,
				     PCI_COMMAND_MEMORY |
				     PCI_COMMAND_MASTER}},
	{}
};
#endif


static struct pci_controller hose = {
#ifndef CONFIG_PCI_PNP
	config_table:pci_mpc85xxads_config_table,
#endif
};

#endif /* CONFIG_PCI */


void pci_init_board (void)
{
#ifdef CONFIG_PCI
	pci_mpc85xx_init (&hose);
#endif /* CONFIG_PCI */
}

#ifdef CONFIG_BOARD_EARLY_INIT_R
int board_early_init_r (void)
{
	volatile ccsr_gur_t *gur = (void *)(CONFIG_SYS_MPC85xx_GUTS_ADDR);

	/* set and reset the GPIO pin 2 which will reset the W83782G chip */
	out_8((unsigned char*)&gur->gpoutdr, 0x3F );
	out_be32((unsigned int*)&gur->gpiocr, 0x200 );	/* enable GPOut */
	udelay(200);
	out_8( (unsigned char*)&gur->gpoutdr, 0x1F );

	return (0);
}
#endif /* CONFIG_BOARD_EARLY_INIT_R */

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
void
ft_board_setup(void *blob, bd_t *bd)
{
	u32 val[12];
	int rc, i = 0;

	ft_cpu_setup(blob, bd);

	/* Fixup NOR FLASH mapping */
	val[i++] = 0;				/* chip select number */
	val[i++] = 0;				/* always 0 */
	val[i++] = gd->bd->bi_flashstart;
	val[i++] = gd->bd->bi_flashsize;

	if (mb862xx.frameAdrs == CONFIG_SYS_LIME_BASE) {
		/* Fixup LIME mapping */
		val[i++] = 2;			/* chip select number */
		val[i++] = 0;			/* always 0 */
		val[i++] = CONFIG_SYS_LIME_BASE;
		val[i++] = CONFIG_SYS_LIME_SIZE;
	}

	/* Fixup FPGA mapping */
	val[i++] = 3;				/* chip select number */
	val[i++] = 0;				/* always 0 */
	val[i++] = CONFIG_SYS_FPGA_BASE;
	val[i++] = CONFIG_SYS_FPGA_SIZE;

	rc = fdt_find_and_setprop(blob, "/localbus", "ranges",
				  val, i * sizeof(u32), 1);
	if (rc)
		printf("Unable to update localbus ranges, err=%s\n",
		       fdt_strerror(rc));
}
#endif /* defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP) */

#define CONFIG_SYS_LIME_SRST		((CONFIG_SYS_LIME_BASE) + 0x01FC002C)
#define CONFIG_SYS_LIME_CCF		((CONFIG_SYS_LIME_BASE) + 0x01FC0038)
#define CONFIG_SYS_LIME_MMR		((CONFIG_SYS_LIME_BASE) + 0x01FCFFFC)
/* Lime clock frequency */
#define CONFIG_SYS_LIME_CLK_100MHZ	0x00000
#define CONFIG_SYS_LIME_CLK_133MHZ	0x10000
/* SDRAM parameter */
#define CONFIG_SYS_LIME_MMR_VALUE	0x4157BA63

#define DISPLAY_WIDTH		800
#define DISPLAY_HEIGHT		480
#define DEFAULT_BRIGHTNESS	25
#define BACKLIGHT_ENABLE	(1 << 31)

static const gdc_regs init_regs [] =
{
	{0x0100, 0x00010f00},
	{0x0020, 0x801901df},
	{0x0024, 0x00000000},
	{0x0028, 0x00000000},
	{0x002c, 0x00000000},
	{0x0110, 0x00000000},
	{0x0114, 0x00000000},
	{0x0118, 0x01df0320},
	{0x0004, 0x041f0000},
	{0x0008, 0x031f031f},
	{0x000c, 0x017f0349},
	{0x0010, 0x020c0000},
	{0x0014, 0x01df01e9},
	{0x0018, 0x00000000},
	{0x001c, 0x01e00320},
	{0x0100, 0x80010f00},
	{0x0, 0x0}
};

const gdc_regs *board_get_regs (void)
{
	return init_regs;
}

#define CONFIG_SYS_LIME_CID		((CONFIG_SYS_LIME_BASE) + 0x01FC00F0)
#define CONFIG_SYS_LIME_REV		((CONFIG_SYS_LIME_BASE) + 0x01FF8084)
int lime_probe(void)
{
	volatile ccsr_lbc_t *memctl = (void *)(CONFIG_SYS_MPC85xx_LBC_ADDR);
	uint cfg_br2;
	uint cfg_or2;
	uint reg;

	cfg_br2 = memctl->br2;
	cfg_or2 = memctl->or2;

	/* Configure GPCM for CS2 */
	memctl->br2 = 0;
	memctl->or2 = 0xfc000410;
	memctl->br2 = (CONFIG_SYS_LIME_BASE) | 0x00001901;

	/* Try to access GDC ID/Revision registers */
	reg = in_be32((void *)CONFIG_SYS_LIME_CID);
	reg = in_be32((void *)CONFIG_SYS_LIME_CID);
	if (reg == 0x303) {
		reg = in_be32((void *)CONFIG_SYS_LIME_REV);
		reg = in_be32((void *)CONFIG_SYS_LIME_REV);
		reg = ((reg & ~0xff) == 0x20050100) ? 1 : 0;
	} else
		reg = 0;

	/* Restore previous CS2 configuration */
	memctl->br2 = 0;
	memctl->or2 = cfg_or2;
	memctl->br2 = cfg_br2;
	return reg;
}

/* Returns Lime base address */
unsigned int board_video_init (void)
{
	if (!lime_probe())
		return 0;

	/*
	 * Reset Lime controller
	 */
	out_be32((void *)CONFIG_SYS_LIME_SRST, 0x1);
	udelay(200);

	/* Set Lime clock to 133MHz */
	out_be32((void *)CONFIG_SYS_LIME_CCF, CONFIG_SYS_LIME_CLK_133MHZ);
	/* Delay required */
	udelay(300);
	/* Set memory parameters */
	out_be32((void *)CONFIG_SYS_LIME_MMR, CONFIG_SYS_LIME_MMR_VALUE);

	mb862xx.winSizeX = DISPLAY_WIDTH;
	mb862xx.winSizeY = DISPLAY_HEIGHT;
	mb862xx.gdfIndex = GDF_15BIT_555RGB;
	mb862xx.gdfBytesPP = 2;

	return CONFIG_SYS_LIME_BASE;
}

#define W83782D_REG_CFG		0x40
#define W83782D_REG_BANK_SEL	0x4e
#define W83782D_REG_ADCCLK	0x4b
#define W83782D_REG_BEEP_CTRL	0x4d
#define W83782D_REG_BEEP_CTRL2	0x57
#define W83782D_REG_PWMOUT1	0x5b
#define W83782D_REG_VBAT	0x5d

static int w83782d_hwmon_init(void)
{
	u8 buf;

	if (i2c_read(CONFIG_SYS_I2C_W83782G_ADDR, W83782D_REG_CFG, 1, &buf, 1))
		return -1;

	i2c_reg_write(CONFIG_SYS_I2C_W83782G_ADDR, W83782D_REG_CFG, 0x80);
	i2c_reg_write(CONFIG_SYS_I2C_W83782G_ADDR, W83782D_REG_BANK_SEL, 0);
	i2c_reg_write(CONFIG_SYS_I2C_W83782G_ADDR, W83782D_REG_ADCCLK, 0x40);

	buf = i2c_reg_read(CONFIG_SYS_I2C_W83782G_ADDR, W83782D_REG_BEEP_CTRL);
	i2c_reg_write(CONFIG_SYS_I2C_W83782G_ADDR, W83782D_REG_BEEP_CTRL,
		      buf | 0x80);
	i2c_reg_write(CONFIG_SYS_I2C_W83782G_ADDR, W83782D_REG_BEEP_CTRL2, 0);
	i2c_reg_write(CONFIG_SYS_I2C_W83782G_ADDR, W83782D_REG_PWMOUT1, 0x47);
	i2c_reg_write(CONFIG_SYS_I2C_W83782G_ADDR, W83782D_REG_VBAT, 0x01);

	buf = i2c_reg_read(CONFIG_SYS_I2C_W83782G_ADDR, W83782D_REG_CFG);
	i2c_reg_write(CONFIG_SYS_I2C_W83782G_ADDR, W83782D_REG_CFG,
		      (buf & 0xf4) | 0x01);
	return 0;
}

static void board_backlight_brightness(int br)
{
	u32 reg;
	u8 buf;
	u8 old_buf;

	/* Select bank 0 */
	if (i2c_read(CONFIG_SYS_I2C_W83782G_ADDR, 0x4e, 1, &old_buf, 1))
		goto err;
	else
		buf = old_buf & 0xf8;

	if (i2c_write(CONFIG_SYS_I2C_W83782G_ADDR, 0x4e, 1, &buf, 1))
		goto err;

	if (br > 0) {
		/* PWMOUT1 duty cycle ctrl */
		buf = 255 / (100 / br);
		if (i2c_write(CONFIG_SYS_I2C_W83782G_ADDR, 0x5b, 1, &buf, 1))
			goto err;

		/* LEDs on */
		reg = in_be32((void *)(CONFIG_SYS_FPGA_BASE + 0x0c));
		if (!(reg & BACKLIGHT_ENABLE));
			out_be32((void *)(CONFIG_SYS_FPGA_BASE + 0x0c),
				 reg | BACKLIGHT_ENABLE);
	} else {
		buf = 0;
		if (i2c_write(CONFIG_SYS_I2C_W83782G_ADDR, 0x5b, 1, &buf, 1))
			goto err;

		/* LEDs off */
		reg = in_be32((void *)(CONFIG_SYS_FPGA_BASE + 0x0c));
		reg &= ~BACKLIGHT_ENABLE;
		out_be32((void *)(CONFIG_SYS_FPGA_BASE + 0x0c), reg);
	}
	/* Restore previous bank setting */
	if (i2c_write(CONFIG_SYS_I2C_W83782G_ADDR, 0x4e, 1, &old_buf, 1))
		goto err;

	return;
err:
	printf("W83782G I2C access failed\n");
}

void board_backlight_switch (int flag)
{
	char * param;
	int rc;

	if (w83782d_hwmon_init())
		printf ("hwmon IC init failed\n");

	if (flag) {
		param = getenv("brightness");
		rc = param ? simple_strtol(param, NULL, 10) : -1;
		if (rc < 0)
			rc = DEFAULT_BRIGHTNESS;
	} else {
		rc = 0;
	}
	board_backlight_brightness(rc);
}

#if defined(CONFIG_CONSOLE_EXTRA_INFO)
/*
 * Return text to be printed besides the logo.
 */
void video_get_info_str (int line_number, char *info)
{
	if (line_number == 1) {
		strcpy (info, " Board: Socrates");
	} else {
		info [0] = '\0';
	}
}
#endif
