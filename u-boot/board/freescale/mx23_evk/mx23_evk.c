/*
 *
 * (c) 2008 Embedded Alley Solutions, Inc.
 *
 * (C) Copyright 2009-2010 Freescale Semiconductor, Inc.
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
#include <asm/arch/mx23.h>
#include <asm/arch/clkctrl.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/spi.h>

#include <asm/arch/regs-pinctrl.h>
#include <asm/arch/registers/regsdigctl.h>
#include <asm/arch/pinctrl.h>

DECLARE_GLOBAL_DATA_PTR;

#define KHz	1000
#define MHz	(1000 * KHz)

#define MMC0_POWER 	PINID_GPMI_D09
#define MMC0_WP 	PINID_GPMI_D10

#define mdelay(t)	udelay(t * 1000)

static struct pin_desc key_pins_desc[] = {
	{ DM100_KEY_LSHIFT,  PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
	{ DM100_KEY_CTRL,    PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
	{ DM100_KEY_ALT,     PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
	{ DM100_KEY_RSHIFT,  PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
	{ DM100_KEY_BATDOOR, PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
};

static struct pin_group key_pins = {
	.pins 	 = key_pins_desc, 
	.nr_pins = ARRAY_SIZE(key_pins_desc),
};

static void dm100_key_init(void)
{
	pin_set_group(&key_pins);
	pin_gpio_direction(DM100_KEY_LSHIFT, 0);
	pin_gpio_direction(DM100_KEY_CTRL, 0);
	pin_gpio_direction(DM100_KEY_ALT, 0);
	pin_gpio_direction(DM100_KEY_RSHIFT, 0);
	pin_gpio_direction(DM100_KEY_BATDOOR, 0);
}

/* MMC pins */
static struct pin_desc mmc_pins_desc[] = {
	{ PINID_SSP1_DATA0, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP1_DATA1, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP1_DATA2, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP1_DATA3, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP1_CMD,   PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP1_SCK,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 }, 
	{ PINID_SSP1_DETECT, PIN_FUN1, PAD_8MA, PAD_3V3, 1 }, 
	{ MMC0_POWER, 	PIN_GPIO, PAD_8MA, PAD_3V3, 1 }, 
};

static struct pin_group mmc_pins = {
	.pins		= mmc_pins_desc,
	.nr_pins	= ARRAY_SIZE(mmc_pins_desc)
};

u32 ssp_mmc_is_wp(void)
{
	return pin_gpio_get(MMC0_WP);
}

void ssp_mmc_board_init(void)
{
	/* Set up MMC pins */
	pin_set_group(&mmc_pins);

	/* Set up WP pin, input */
	pin_set_type(MMC0_WP, PIN_GPIO);
	pin_gpio_set(MMC0_WP, 0);
	pin_gpio_direction(MMC0_WP, 0);

	/* Power on the card slot, output */
	//pin_set_type(MMC0_POWER, PIN_GPIO);
	pin_gpio_direction(MMC0_POWER, 1);
	pin_gpio_set(MMC0_POWER, 1);

	/* Wait 10 ms for card ramping up */
	mdelay(50);
}

#if 0
static void set_pinmux(void)
{

#if defined(CONFIG_SPI_SSP1)

	/* Configure SSP1 pins for ENC28j60: 8maA */
	REG_CLR(PINCTRL_BASE + PINCTRL_MUXSEL(4), 0x00003fff);

	REG_CLR(PINCTRL_BASE + PINCTRL_DRIVE(8), 0X03333333);
	REG_SET(PINCTRL_BASE + PINCTRL_DRIVE(8), 0x01111111);

	REG_CLR(PINCTRL_BASE + PINCTRL_PULL(2), 0x0000003f);
#endif

#if defined(CONFIG_SPI_SSP2)

	/* Configure SSP2 pins for ENC28j60: 8maA */
	REG_CLR(PINCTRL_BASE + PINCTRL_MUXSEL(0), 0x00000fc3);
	REG_SET(PINCTRL_BASE + PINCTRL_MUXSEL(0), 0x00000a82);

	REG_CLR(PINCTRL_BASE + PINCTRL_MUXSEL(1), 0x00030300);
	REG_SET(PINCTRL_BASE + PINCTRL_MUXSEL(1), 0x00020200);

	REG_CLR(PINCTRL_BASE + PINCTRL_DRIVE(0), 0X00333003);
	REG_SET(PINCTRL_BASE + PINCTRL_DRIVE(0), 0x00111001);

	REG_CLR(PINCTRL_BASE + PINCTRL_DRIVE(2), 0x00030000);
	REG_SET(PINCTRL_BASE + PINCTRL_DRIVE(2), 0x00010000);

	REG_CLR(PINCTRL_BASE + PINCTRL_DRIVE(3), 0x00000003);
	REG_SET(PINCTRL_BASE + PINCTRL_DRIVE(3), 0x00000001);

	REG_CLR(PINCTRL_BASE + PINCTRL_PULL(0), 0x00100039);
#endif

}

#define IO_DIVIDER	18
static void set_clocks(void)
{
	u32 ssp_source_clk, ssp_clk;
	u32 ssp_div = 1;
	u32 val = 0;

	/*
	 * Configure 480Mhz IO clock
	 */

	/* Ungate IO_CLK and set divider */
	REG_CLR(CLKCTRL_BASE + CLKCTRL_FRAC, FRAC_CLKGATEIO);
	REG_CLR(CLKCTRL_BASE + CLKCTRL_FRAC, 0x3f << FRAC_IOFRAC);
	REG_SET(CLKCTRL_BASE + CLKCTRL_FRAC, IO_DIVIDER << FRAC_IOFRAC);

	/*
	 * Set SSP CLK to desired value
	 */

	/* Calculate SSP_CLK divider relatively to 480Mhz IO_CLK*/
	ssp_source_clk = 480 * MHz;
	ssp_clk = CONFIG_SSP_CLK;
	ssp_div = (ssp_source_clk + ssp_clk - 1) / ssp_clk;

	/* Enable SSP clock */
	val = REG_RD(CLKCTRL_BASE + CLKCTRL_SSP);
	val &= ~SSP_CLKGATE;
	REG_WR(CLKCTRL_BASE + CLKCTRL_SSP, val);

	/* Wait while clock is gated */
	while (REG_RD(CLKCTRL_BASE + CLKCTRL_SSP) & SSP_CLKGATE)
		;

	/* Set SSP clock divider */
	val &= ~(0x1ff << SSP_DIV);
	val |= ssp_div << SSP_DIV;
	REG_WR(CLKCTRL_BASE + CLKCTRL_SSP, val);

	/* Wait until new divider value is set */
	while (REG_RD(CLKCTRL_BASE + CLKCTRL_SSP) & SSP_BUSY)
		;

	/* Set SSP clock source to IO_CLK */
	REG_SET(CLKCTRL_BASE + CLKCTRL_CLKSEQ, CLKSEQ_BYPASS_SSP);
	REG_CLR(CLKCTRL_BASE + CLKCTRL_CLKSEQ, CLKSEQ_BYPASS_SSP);
}
#endif

int dram_init(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size  = PHYS_SDRAM_1_SIZE;
	gd->bd->bi_dram[1].start = PHYS_SDRAM_2;
	gd->bd->bi_dram[1].size  = PHYS_SDRAM_2_SIZE;

	return 0;
}

extern unsigned imx_set_ioclk(unsigned nc);
extern unsigned imx_set_sspclk(unsigned index, unsigned nc, int high);
extern unsigned imx_set_sspclk(unsigned index, unsigned nc, int high);
extern unsigned imx_set_gpmiclk(unsigned nc, int high);

#define LCD_PWR_CTRL	PINID_LCD_D16
extern struct pin_group lcd_ortus_pins;

int board_init(void)
{
	/* arch number of Freescale STMP 378x development board */
	gd->bd->bi_arch_number = MACH_TYPE_MX23EVK;

	/* adress of boot parameters */
	gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR;

#if 1
	dm100_key_init();

	/* enable IOCLK to run at the PLL frequency */
	imx_set_ioclk(480000000);

	/* run the SSP unit clock at 100,000 kHz */
	imx_set_sspclk(0, 100000000, 1);

	/* run the GPMI unit clock at 480,000,000 kHz */
	imx_set_gpmiclk(200000000, 1);

	pin_set_group(&lcd_ortus_pins);
	pin_gpio_direction(LCD_PWR_CTRL, 1); /* output */
	pin_gpio_set(LCD_PWR_CTRL, 1);
#else
	set_clocks();

	set_pinmux();

	/* Configure SPI on SSP1 or SSP2 */
	spi_init();
#endif
	return 0;
}

int misc_init_r(void)
{
	return 0;
}

int checkboard(void)
{
	printf("Board: MX23 EVK. \n");
	return 0;
}
