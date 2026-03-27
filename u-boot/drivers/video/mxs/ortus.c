/*
 * ==========================================================================
 *
 *       Filename:  ortus.c
 *
 *    Description:  
 *
 *        Version:  0.01
 *        Created:  2011年05月17日 17时06分55秒
 *
 *         Author:  smmei (), 
 *        Company:  
 *
 * ==========================================================================
 */

#include <common.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/arch/mx23_pins.h>
#include <asm/arch/pinctrl.h>
#include "lcdif.h"
#include <asm/arch/registers/regsdigctl.h>

#define PICOS2KHZ(a) (1000000000UL/(a))
#define KHZ2PICOS(a) (1000000000UL/(a))

#define CMD     0
#define DATA    1

#define lcdif_read(reg) 	__raw_readl(reg)
#define lcdif_write(reg,val) 	__raw_writel(val, reg)

#define LCD_PWR_CTRL	PINID_LCD_D16
#define LCD_FRAME	PINID_LCD_DOTCK
#define LEDPWM_CS	PINID_PWM3

#ifndef mdelay
#define mdelay(t)	udelay(t * 1000)
#endif

extern unsigned imx_get_lcdifclk(void);
extern unsigned imx_set_lcdifclk(unsigned nc);
extern void imx_lcdif_clk_enable(void);
extern void imx_lcdif_clk_disable(void);

static struct pin_desc lcd_ortus_desc[] = {
	{ PINID_LCD_D00,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
	{ PINID_LCD_D01,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
	{ PINID_LCD_D02,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
	{ PINID_LCD_D03,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
	{ PINID_LCD_D04,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
	{ PINID_LCD_D05,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 }, 
	{ PINID_LCD_D06,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 }, 
        { PINID_LCD_D07,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
        { PINID_LCD_D08,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
        { PINID_LCD_D09,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
        { PINID_LCD_D10,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
        { PINID_LCD_D11,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
        { PINID_LCD_D12,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
        { PINID_LCD_D13,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
        { PINID_LCD_D14,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
        { PINID_LCD_D15,   PIN_FUN1, PAD_8MA, PAD_3V3, 0 },

        { PINID_LCD_RESET, PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
        { PINID_LCD_RS,    PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
        { PINID_LCD_WR,    PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
        { PINID_LCD_CS,    PIN_FUN1, PAD_8MA, PAD_3V3, 0 },

	{ LCD_PWR_CTRL,    PIN_GPIO, PAD_12MA, PAD_3V3, 0 },
        { LCD_FRAME,       PIN_GPIO, PAD_4MA, PAD_3V3, 0 },
        { PINID_LCD_VSYNC, PIN_GPIO, PAD_4MA, PAD_3V3, 0 },
	{ LEDPWM_CS,       PIN_GPIO, PAD_4MA, PAD_3V3, 0 }, 
};

struct pin_group lcd_ortus_pins = {
	.pins		= lcd_ortus_desc,
	.nr_pins	= ARRAY_SIZE(lcd_ortus_desc)
};

static const uint8_t lcd_init_data[] = {
	//Manufacturer Command Access portect
	CMD, 0xB0, DATA, 0x00, 

	//Frame Memory Access and interface setting
	CMD, 0xB3, DATA, 0x80, DATA, 0x41, 

	//DSI Control
	CMD, 0xB6, DATA, 0x00, 

	//back light control(1)
	CMD, 0xB8, DATA, 0x00, DATA, 0x00, DATA, 0x00, DATA, 0x00, DATA, 0x00, 
	DATA, 0x00, DATA, 0x00, DATA, 0x01, DATA, 0x01, DATA, 0x00, DATA, 0x00, 
	DATA, 0x00, DATA, 0x00, DATA, 0x00, DATA, 0x00, 

	//back light control(2)
	CMD, 0xB9, DATA, 0x01, DATA, 0x00, DATA, 0x07, DATA, 0x18, 

	//Panel Driving Setting
	CMD, 0xC0, DATA, 0x1B, DATA, 0x95, DATA, 0x00, DATA, 0x00, DATA, 0x00, 
	DATA, 0x00, DATA, 0x08, 

	//Timing Setting for Normal Mode
	//CMD, 0xC1, DATA, 0x01, DATA, 0x00, DATA, 0x1B, DATA, 0x05, DATA, 0x0C, 
	//更改为下面这样的参数，电流减小12mA
	CMD, 0xC1, DATA, 0x00, DATA, 0x00, DATA, 0x13, DATA, 0x05, DATA, 0x0C, 

	//Display Timing Setting for Idle Mode
	CMD, 0xC3, DATA, 0x00, DATA, 0x00, DATA, 0x13, DATA, 0x05, DATA, 0x0C, 

	//Source/VCOM/Gate Driving Timing setting
	CMD, 0xC4, DATA, 0x31, DATA, 0x03, DATA, 0x43, DATA, 0x01, 

	//Gamma Set A
	CMD, 0xC8, DATA, 0x0A, DATA, 0x07, DATA, 0x56, DATA, 0xFF, DATA, 0x0F, 
	DATA, 0x56, DATA, 0x65, DATA, 0x07, DATA, 0x11, DATA, 0x0A, DATA, 0x07, 
	DATA, 0x56, DATA, 0x65, DATA, 0x0F, DATA, 0xFF, DATA, 0x65, DATA, 0x07, 
	DATA, 0x07, 

	//Gamma Set B
	CMD, 0xC9, DATA, 0x0A, DATA, 0x07, DATA, 0x56, DATA, 0xFF, DATA, 0x0F, 
	DATA, 0x56, DATA, 0x65, DATA, 0x07, DATA, 0x11, DATA, 0x0A, DATA, 0x07, 
	DATA, 0x56, DATA, 0x65, DATA, 0x0F, DATA, 0xFF, DATA, 0x65, DATA, 0x07, 
	DATA, 0x07, 

	//Gamma Set C
	CMD, 0xCA, DATA, 0x0A, DATA, 0x07, DATA, 0x56, DATA, 0xFF, DATA, 0x0F, 
	DATA, 0x56, DATA, 0x65, DATA, 0x07, DATA, 0x11, DATA, 0x0A, DATA, 0x07, 
	DATA, 0x56, DATA, 0x65, DATA, 0x0F, DATA, 0xFF, DATA, 0x65, DATA, 0x07, 
	DATA, 0x07, 

	//Power Setting Common
	CMD, 0xD0, DATA, 0x00, DATA, 0x55, DATA, 0xC0, DATA, 0x8F, 

	//Power Setting for Normal Mode
	CMD, 0xD2, DATA, 0x03, DATA, 0x00, DATA, 0x00, 

	//Power Setting for Idle Mode
	CMD, 0xD4, DATA, 0x03, DATA, 0x00, DATA, 0x00, 

	//set_address_mode
	CMD, 0x36, DATA, 0x00, 

	//set_column_address
	CMD, 0x2A, DATA, 0x00, DATA, 0x00, 
	DATA, 0x01, DATA, 0x0A,
	//DATA, 0x03, DATA, 0x1f,

	//set_page_address
	CMD, 0x2B, DATA, 0x00, DATA, 0x00, DATA, 0x02, DATA, 0x57, 

	//set_tear_on
	CMD, 0x35, DATA, 0x00, 

	//set_pixel_format
	CMD, 0x3A, DATA, 0x07, 

	//set_tear_scanline
	CMD, 0x44, DATA, 0x00, DATA, 0x00, 

	//exit_sleep_mode
	//CMD, 0x11, 

	//set_display_on
	//CMD, 0x29, 
	/* CMD, 0x2c, */
};


static void mpulcd_setup_pannel_register(uint32_t data, uint32_t val)
{
	lcdif_write(HW_LCDIF_CTRL_CLR_ADDR,
		    BM_LCDIF_CTRL_LCDIF_MASTER | BM_LCDIF_CTRL_RUN);

	lcdif_write(HW_LCDIF_TRANSFER_COUNT_ADDR,
		    BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) |
		    BF_LCDIF_TRANSFER_COUNT_H_COUNT(1));

	if (data)
		lcdif_write(HW_LCDIF_CTRL_SET_ADDR, BM_LCDIF_CTRL_DATA_SELECT);
	else
		lcdif_write(HW_LCDIF_CTRL_CLR_ADDR, BM_LCDIF_CTRL_DATA_SELECT);

	lcdif_write(HW_LCDIF_CTRL_SET_ADDR, BM_LCDIF_CTRL_RUN);

	while (lcdif_read(HW_LCDIF_STAT_ADDR) & BM_LCDIF_STAT_LFIFO_FULL)
		;

	lcdif_write(HW_LCDIF_DATA_ADDR, val << 8);

	while(lcdif_read(HW_LCDIF_CTRL_ADDR) & BM_LCDIF_CTRL_RUN)
		;

	//lcdif_write(HW_LCDIF_CTRL1_CLR_ADDR, BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ);
}

static void mpulcd_start_refresh(void)
{
	mpulcd_setup_pannel_register(CMD, 0x2c);

	lcdif_write(HW_LCDIF_CTRL_SET_ADDR, BM_LCDIF_CTRL_DATA_SELECT);
	lcdif_write(HW_LCDIF_TRANSFER_COUNT_ADDR,
		    BF_LCDIF_TRANSFER_COUNT_V_COUNT(600) |
		    BF_LCDIF_TRANSFER_COUNT_H_COUNT(402));

	mxs_lcdif_run();
}

static int mpulcd_set_backlight(uint32_t bl)
{
	int i;

	for(i = 0; i < sizeof(lcd_init_data);) {
		if (lcd_init_data[i] == CMD && lcd_init_data[i + 1] == 0xB9) {
			//back light control(2)
			mpulcd_setup_pannel_register(CMD, 0xB9);
			mpulcd_setup_pannel_register(DATA, 0x01);

			mpulcd_setup_pannel_register(DATA, bl & 0xff);

			mpulcd_setup_pannel_register(DATA, 0x07);
			mpulcd_setup_pannel_register(DATA, 0x18); 
			i += 10;
		} else {
			mpulcd_setup_pannel_register(lcd_init_data[i], lcd_init_data[i + 1]);
			i += 2;
		}
	}
	mpulcd_setup_pannel_register(CMD, 0x11);
	udelay(400);

	mpulcd_setup_pannel_register(CMD, 0x29);

	return 0;
}

static void mpulcd_display_on(void)
{
	mpulcd_setup_pannel_register(CMD, 0x29);
	//lcdif_write(HW_LCDIF_CTRL1_SET_ADDR, BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN);
	mpulcd_start_refresh();
}

static void mpulcd_display_off(void)
{
	//lcdif_write(HW_LCDIF_CTRL1_CLR_ADDR, BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN);
	mpulcd_setup_pannel_register(CMD, 0x28);
	mdelay(30);
	mpulcd_setup_pannel_register(CMD, 0x10);
	mdelay(120);
}

static void mpulcd_release_panel(struct mxs_platform_fb_entry *pentry)
{
	mpulcd_display_off();
	mxs_lcdif_dma_release();

	imx_lcdif_clk_disable();
	pin_gpio_set(LCD_PWR_CTRL, 0);
}

static int mpulcd_pan_display(dma_addr_t addr)
{
	lcdif_write(HW_LCDIF_CUR_BUF_ADDR, addr);
	return 0;
}

static void mpulcd_init_lcdif(void)
{
	lcdif_write(HW_LCDIF_CTRL_CLR_ADDR,
		    BM_LCDIF_CTRL_CLKGATE |
		    BM_LCDIF_CTRL_SFTRST);

	lcdif_write(HW_LCDIF_CTRL_ADDR,
		    BF_LCDIF_CTRL_LCD_DATABUS_WIDTH(BV_LCDIF_CTRL_LCD_DATABUS_WIDTH__16_BIT) |
		    BF_LCDIF_CTRL_WORD_LENGTH(BV_LCDIF_CTRL_WORD_LENGTH__16_BIT));

	lcdif_write(HW_LCDIF_CTRL1_CLR_ADDR, BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT);
	lcdif_write(HW_LCDIF_CTRL1_SET_ADDR, BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(0xf));

	lcdif_write(HW_LCDIF_TIMING_ADDR, 0x01010202);
}

static void mpulcd_init_panel_hw(void)
{
	int i;

	/*
	 * Make sure we do a high-to-low transition to reset the panel.
	 * First make it low for 100 msec, hi for 10 msec, low for 10 msec,
	 * then hi.
	 */
	writel(BM_LCDIF_CTRL1_RESET, HW_LCDIF_CTRL1_CLR_ADDR);	/* low */
	mdelay(1);
	writel(BM_LCDIF_CTRL1_RESET, HW_LCDIF_CTRL1_SET_ADDR);	/* high */
	mdelay(5);

	for(i = 0; i < sizeof(lcd_init_data); i += 2)
		mpulcd_setup_pannel_register(lcd_init_data[i], lcd_init_data[i + 1]);

	mpulcd_start_refresh();
	while(__raw_readl(HW_LCDIF_CTRL_ADDR) & BM_LCDIF_CTRL_RUN) ;

	mpulcd_setup_pannel_register(CMD, 0x11);
	//mdelay(120);
	{
		extern int load_program(uint32_t address);
		uint32_t addr = DM100_DDR_OFFSET_USER_PROGRAM;
		load_program(addr);
	}

	mpulcd_setup_pannel_register(CMD, 0x29);
	mdelay(30);

	mpulcd_set_backlight(70);
}

static int mpulcd_init_panel(dma_addr_t phys, int memsize,
			     struct mxs_platform_fb_entry *pentry)
{
	pin_set_group(&lcd_ortus_pins);

	pin_gpio_direction(LCD_PWR_CTRL, 1); /* output */
	pin_gpio_set(LCD_PWR_CTRL, 1);

	imx_set_lcdifclk(PICOS2KHZ(pentry->cycle_time_ns));

	mpulcd_init_lcdif();
	mxs_lcdif_dma_init(phys, memsize);
	mpulcd_init_panel_hw();

	return 0;
}

struct mxs_platform_fb_entry fb_entry = {
	.name           = "ortus",
	.x_res          = CONFIG_LCD_WIDTH,
	.y_res          = CONFIG_LCD_HEIGH,
	.bpp            = 8,
	.cycle_time_ns  = 33,
	.lcd_type       = MXS_LCD_PANEL_SYSTEM,
	.init_panel     = mpulcd_init_panel,
	.release_panel  = mpulcd_release_panel,
//	.blank_panel    = mpulcd_blank_panel,
	.run_panel      = mpulcd_display_on,
	.stop_panel     = mpulcd_display_off,
	.pan_display    = mpulcd_pan_display,
};
