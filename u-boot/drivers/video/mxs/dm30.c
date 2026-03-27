/*
 * ==========================================================================
 *
 *       Filename:  dm30.c
 *
 *    Description:  
 *
 *        Version:  0.01
 *        Created:  2017年08月18日 10时59分55秒
 *
 *         Author:  Zhong Yuan Huan 
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
#include "it8951_i80.h"

#define PICOS2KHZ(a) (1000000000UL/(a))
#define KHZ2PICOS(a) (1000000000UL/(a))

#define lcdif_read(reg) 	__raw_readl(reg)
#define lcdif_write(reg,val) 	__raw_writel(val, reg)

#define LCD_PWR_CTRL	PINID_LCD_D16
#define I80_HRST_L	PINID_LCD_RESET
#define I80_HD_C	PINID_LCD_RS
#define I80_HWE_L	PINID_LCD_WR
#define I80_HCS_L	PINID_LCD_CS
#define I80_HRDY	PINID_LCD_DOTCK
#define I80_HRD_L	PINID_LCD_VSYNC

#ifndef mdelay
#define mdelay(t)	udelay(t * 1000)
#endif

// Use gpio to write if define else use lcdif to write.
//#define I80_GPIO
#undef I80_GPIO

extern unsigned imx_get_lcdifclk(void);
extern unsigned imx_set_lcdifclk(unsigned nc);
extern void imx_lcdif_clk_enable(void);
extern void imx_lcdif_clk_disable(void);

// Host controller function.
void gpio_i80_pin_config(void);
#ifdef I80_GPIO
void gpio_i80_16b_cmd_out(TWord cmd);
void gpio_i80_16b_data_out(TWord data);
#endif
static TWord gpio_i80_16b_data_in(void);
inline void LCDWaitForReady(void);
void LCDWriteCmdCode(TWord cmd);
void LCDWriteData(TWord data);
void LCDWriteNData(TWord* pwBuf, TDWord wordCnt);
TWord LCDReadData(void);
void LCDSendCmdArg(TWord cmdCode, TWord* pArg, TWord numArg);

// GPIO config for I80 interface.
#ifdef I80_GPIO
static struct pin_desc lcd_ortus_desc[] = {
    // I80 data pins.
    { PINID_LCD_D00,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D01,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D02,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D03,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D04,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D05,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D06,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D07,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D08,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D09,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D10,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D11,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D12,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D13,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D14,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },
    { PINID_LCD_D15,   PIN_GPIO, PAD_8MA, PAD_3V3, 0 },

    // I80 control pins.
    { I80_HRST_L,      PIN_GPIO, PAD_8MA, PAD_3V3, 0 }, // HRST_L (LCD_RSTn)
    { I80_HD_C,        PIN_GPIO, PAD_8MA, PAD_3V3, 0 }, // HD/C (LCD_RS)
    { I80_HWE_L,       PIN_GPIO, PAD_8MA, PAD_3V3, 0 }, // HWE_L (LCD_WRn)
    { I80_HCS_L,       PIN_GPIO, PAD_8MA, PAD_3V3, 0 }, // HCS_L (LCD_CSn)

    // imx23 haven't function pin for HRDY and HRD_L,
    // So we used gpio to replace it. Means we used gpio to read.
    { I80_HRDY,        PIN_GPIO, PAD_4MA, PAD_3V3, 0 }, // HRDY (LCD_FRAME)
    { I80_HRD_L,       PIN_GPIO, PAD_4MA, PAD_3V3, 0 }, // HRD_L (LCD_RDn)

    // Control power 1.8v & 3.3v supply for IT8951E.
    { LCD_PWR_CTRL,   PIN_GPIO, PAD_12MA, PAD_3V3, 1 },
}; 
#else
static struct pin_desc lcd_ortus_desc[] = {
    // I80 data pins.
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

    // I80 control pins.
    { I80_HRST_L,      PIN_FUN1, PAD_8MA, PAD_3V3, 0 }, // HRST_L (LCD_RSTn)
    { I80_HD_C,        PIN_FUN1, PAD_8MA, PAD_3V3, 0 }, // HD/C (LCD_RS)
    { I80_HWE_L,       PIN_FUN1, PAD_8MA, PAD_3V3, 0 }, // HWE_L (LCD_WRn)
    { I80_HCS_L,       PIN_FUN1, PAD_8MA, PAD_3V3, 0 }, // HCS_L (LCD_CSn)

    // imx23 haven't function pin for HRDY and HRD_L,
    // So we used gpio to replace it. Means we used gpio to read.
    { I80_HRDY,        PIN_GPIO, PAD_4MA, PAD_3V3, 0 }, // HRDY (LCD_FRAME)
    { I80_HRD_L,       PIN_GPIO, PAD_4MA, PAD_3V3, 0 }, // HRD_L (LCD_RDn)

    // Control power 1.8v & 3.3v supply for IT8951E.
    { LCD_PWR_CTRL,   PIN_GPIO, PAD_12MA, PAD_3V3, 1 },
};
#endif

struct pin_group lcd_ortus_pins = {
    .pins = lcd_ortus_desc,
    .nr_pins = ARRAY_SIZE(lcd_ortus_desc)
}; 

static int g_it8951_init = 0;
I80IT8951DevInfo g_it8951_dev_info;
static u32 g_it8951_base_addr = 0;
static int g_gpio_is_input = 0;
uchar *g_lcd_frame_base = NULL;


void gpio_i80_pin_config()
{
	g_gpio_is_input = 0;

	pin_set_group(&lcd_ortus_pins);

#ifdef I80_GPIO
	// d0-d15 as output, cs as output.
	pin_gpio_direction(PINID_LCD_D00, 1);
	pin_gpio_direction(PINID_LCD_D01, 1);
	pin_gpio_direction(PINID_LCD_D02, 1);
	pin_gpio_direction(PINID_LCD_D03, 1);
	pin_gpio_direction(PINID_LCD_D04, 1);
	pin_gpio_direction(PINID_LCD_D05, 1);
	pin_gpio_direction(PINID_LCD_D06, 1);
	pin_gpio_direction(PINID_LCD_D07, 1);
	pin_gpio_direction(PINID_LCD_D08, 1);
	pin_gpio_direction(PINID_LCD_D09, 1);
	pin_gpio_direction(PINID_LCD_D10, 1);
	pin_gpio_direction(PINID_LCD_D11, 1);
	pin_gpio_direction(PINID_LCD_D12, 1);
	pin_gpio_direction(PINID_LCD_D13, 1);
	pin_gpio_direction(PINID_LCD_D14, 1);
	pin_gpio_direction(PINID_LCD_D15, 1);

	pin_gpio_direction(I80_HRST_L, 1);
	pin_gpio_direction(I80_HD_C, 1);
	pin_gpio_direction(I80_HWE_L, 1);
	pin_gpio_direction(I80_HCS_L, 1);
	pin_gpio_direction(I80_HRDY, 0);
	pin_gpio_direction(I80_HRD_L, 1);

	// Disable read or write.
	pin_gpio_set(I80_HD_C, 1);
	pin_gpio_set(I80_HWE_L, 1);
	pin_gpio_set(I80_HCS_L, 1);
	pin_gpio_set(I80_HRD_L, 1);
#else
	pin_gpio_direction(I80_HRDY, 0);
	pin_gpio_direction(I80_HRD_L, 1);
	pin_gpio_set(I80_HRD_L, 1);
#endif
}

#ifdef I80_GPIO

//-------------------------------------------------------------------
//Host controller Write command code for 16 bits using GPIO simulation
//-------------------------------------------------------------------
void gpio_i80_16b_cmd_out(uint16_t data)
{
	if (g_gpio_is_input) {
		gpio_i80_pin_config();
	}

	// Set 16 bits Bus Data
	pin_gpio_set(PINID_LCD_D00, (data & 0x0001) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D01, (data & 0x0002) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D02, (data & 0x0004) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D03, (data & 0x0008) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D04, (data & 0x0010) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D05, (data & 0x0020) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D06, (data & 0x0040) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D07, (data & 0x0080) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D08, (data & 0x0100) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D09, (data & 0x0200) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D10, (data & 0x0400) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D11, (data & 0x0800) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D12, (data & 0x1000) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D13, (data & 0x2000) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D14, (data & 0x4000) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D15, (data & 0x8000) ? 1 : 0);

	pin_gpio_set(I80_HD_C, 0); //Switch C/D to CMD => CMD - L
	udelay(1);
	pin_gpio_set(I80_HCS_L, 0); // CS-L
	udelay(1);
	pin_gpio_set(I80_HWE_L, 0); // WR Enable
	udelay(1);

	pin_gpio_set(I80_HWE_L, 1); //WR Enable - H
	udelay(1);
	pin_gpio_set(I80_HCS_L, 1); //CS-H
	udelay(1);
}

//-------------------------------------------------------------------
//Host controller Write Data for 16 bits using GPIO simulation
//-------------------------------------------------------------------
void gpio_i80_16b_data_out(uint16_t data)
{
	if (g_gpio_is_input) {
		gpio_i80_pin_config();
	}

	// Set 16 bits Bus Data
	pin_gpio_set(PINID_LCD_D00, (data & 0x0001) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D01, (data & 0x0002) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D02, (data & 0x0004) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D03, (data & 0x0008) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D04, (data & 0x0010) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D05, (data & 0x0020) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D06, (data & 0x0040) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D07, (data & 0x0080) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D08, (data & 0x0100) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D09, (data & 0x0200) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D10, (data & 0x0400) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D11, (data & 0x0800) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D12, (data & 0x1000) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D13, (data & 0x2000) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D14, (data & 0x4000) ? 1 : 0);
	pin_gpio_set(PINID_LCD_D15, (data & 0x8000) ? 1 : 0);

	pin_gpio_set(I80_HD_C, 1); // Switch C/D to Data => Data - H
	pin_gpio_set(I80_HCS_L, 0); // CS-L
	udelay(1);
	pin_gpio_set(I80_HWE_L, 0); // WR Enable
	udelay(1);

	pin_gpio_set(I80_HWE_L, 1); //WR Enable - H
	udelay(1);
	pin_gpio_set(I80_HCS_L, 1); //CS-H
	udelay(1);
}
#endif

//-------------------------------------------------------------------
//Host controller Read Data for 16 bits using GPIO simulation
//-------------------------------------------------------------------
static TWord gpio_i80_16b_data_in()
{
#ifdef I80_GPIO
	if (!g_gpio_is_input) {
		g_gpio_is_input = 1;

		// d0-d15 as input
		pin_set_type(PINID_LCD_D00, PIN_GPIO);
		pin_set_type(PINID_LCD_D01, PIN_GPIO);
		pin_set_type(PINID_LCD_D02, PIN_GPIO);
		pin_set_type(PINID_LCD_D03, PIN_GPIO);
		pin_set_type(PINID_LCD_D04, PIN_GPIO);
		pin_set_type(PINID_LCD_D05, PIN_GPIO);
		pin_set_type(PINID_LCD_D06, PIN_GPIO);
		pin_set_type(PINID_LCD_D07, PIN_GPIO);
		pin_set_type(PINID_LCD_D08, PIN_GPIO);
		pin_set_type(PINID_LCD_D09, PIN_GPIO);
		pin_set_type(PINID_LCD_D10, PIN_GPIO);
		pin_set_type(PINID_LCD_D11, PIN_GPIO);
		pin_set_type(PINID_LCD_D12, PIN_GPIO);
		pin_set_type(PINID_LCD_D13, PIN_GPIO);
		pin_set_type(PINID_LCD_D14, PIN_GPIO);
		pin_set_type(PINID_LCD_D15, PIN_GPIO);

		pin_gpio_direction(PINID_LCD_D00, 0);
		pin_gpio_direction(PINID_LCD_D01, 0);
		pin_gpio_direction(PINID_LCD_D02, 0);
		pin_gpio_direction(PINID_LCD_D03, 0);
		pin_gpio_direction(PINID_LCD_D04, 0);
		pin_gpio_direction(PINID_LCD_D05, 0);
		pin_gpio_direction(PINID_LCD_D06, 0);
		pin_gpio_direction(PINID_LCD_D07, 0);
		pin_gpio_direction(PINID_LCD_D08, 0);
		pin_gpio_direction(PINID_LCD_D09, 0);
		pin_gpio_direction(PINID_LCD_D10, 0);
		pin_gpio_direction(PINID_LCD_D11, 0);
		pin_gpio_direction(PINID_LCD_D12, 0);
		pin_gpio_direction(PINID_LCD_D13, 0);
		pin_gpio_direction(PINID_LCD_D14, 0);
		pin_gpio_direction(PINID_LCD_D15, 0);
	}
#else
	// d0-d15 as input
	pin_set_type(PINID_LCD_D00, PIN_GPIO);
	pin_set_type(PINID_LCD_D01, PIN_GPIO);
	pin_set_type(PINID_LCD_D02, PIN_GPIO);
	pin_set_type(PINID_LCD_D03, PIN_GPIO);
	pin_set_type(PINID_LCD_D04, PIN_GPIO);
	pin_set_type(PINID_LCD_D05, PIN_GPIO);
	pin_set_type(PINID_LCD_D06, PIN_GPIO);
	pin_set_type(PINID_LCD_D07, PIN_GPIO);
	pin_set_type(PINID_LCD_D08, PIN_GPIO);
	pin_set_type(PINID_LCD_D09, PIN_GPIO);
	pin_set_type(PINID_LCD_D10, PIN_GPIO);
	pin_set_type(PINID_LCD_D11, PIN_GPIO);
	pin_set_type(PINID_LCD_D12, PIN_GPIO);
	pin_set_type(PINID_LCD_D13, PIN_GPIO);
	pin_set_type(PINID_LCD_D14, PIN_GPIO);
	pin_set_type(PINID_LCD_D15, PIN_GPIO);

	pin_gpio_direction(PINID_LCD_D00, 0);
	pin_gpio_direction(PINID_LCD_D01, 0);
	pin_gpio_direction(PINID_LCD_D02, 0);
	pin_gpio_direction(PINID_LCD_D03, 0);
	pin_gpio_direction(PINID_LCD_D04, 0);
	pin_gpio_direction(PINID_LCD_D05, 0);
	pin_gpio_direction(PINID_LCD_D06, 0);
	pin_gpio_direction(PINID_LCD_D07, 0);
	pin_gpio_direction(PINID_LCD_D08, 0);
	pin_gpio_direction(PINID_LCD_D09, 0);
	pin_gpio_direction(PINID_LCD_D10, 0);
	pin_gpio_direction(PINID_LCD_D11, 0);
	pin_gpio_direction(PINID_LCD_D12, 0);
	pin_gpio_direction(PINID_LCD_D13, 0);
	pin_gpio_direction(PINID_LCD_D14, 0);
	pin_gpio_direction(PINID_LCD_D15, 0);

	// I80_HD_C, I80_HCS_L as output
	pin_set_type(I80_HD_C, PIN_GPIO);
	pin_set_type(I80_HCS_L, PIN_GPIO);
	pin_gpio_direction(I80_HD_C, 1);
	pin_gpio_direction(I80_HCS_L, 1);
	pin_gpio_set(I80_HRD_L, 1);
	pin_gpio_set(I80_HCS_L, 1);

#endif
	TWord data = 0;

	// I80_HD_C = 1 and Active I80_HCS_L & I80_HRD_L
	pin_gpio_set(I80_HD_C, 1); // Switch C/D to Data => Data - H
	udelay(1*20);
	pin_gpio_set(I80_HCS_L, 0); // CS-L
	udelay(1*20);
	pin_gpio_set(I80_HRD_L, 0); // RD Enable
	udelay(1*20);

	data |= pin_gpio_get(PINID_LCD_D00) << 0;
	data |= pin_gpio_get(PINID_LCD_D01) << 1;
	data |= pin_gpio_get(PINID_LCD_D02) << 2;
	data |= pin_gpio_get(PINID_LCD_D03) << 3;
	data |= pin_gpio_get(PINID_LCD_D04) << 4;
	data |= pin_gpio_get(PINID_LCD_D05) << 5;
	data |= pin_gpio_get(PINID_LCD_D06) << 6;
	data |= pin_gpio_get(PINID_LCD_D07) << 7;
	data |= pin_gpio_get(PINID_LCD_D08) << 8;
	data |= pin_gpio_get(PINID_LCD_D09) << 9;
	data |= pin_gpio_get(PINID_LCD_D10) << 10;
	data |= pin_gpio_get(PINID_LCD_D11) << 11;
	data |= pin_gpio_get(PINID_LCD_D12) << 12;
	data |= pin_gpio_get(PINID_LCD_D13) << 13;
	data |= pin_gpio_get(PINID_LCD_D14) << 14;
	data |= pin_gpio_get(PINID_LCD_D15) << 15;

	udelay(1*20);	//new add
	pin_gpio_set(I80_HRD_L, 1); // RD Enable - H
	udelay(1*20);
	pin_gpio_set(I80_HCS_L, 1); // CS-H
	udelay(1*20);

#ifdef I80_GPIO
#else
	// Restore d0-d15 to function mode
	pin_set_type(PINID_LCD_D00, PIN_FUN1);
	pin_set_type(PINID_LCD_D01, PIN_FUN1);
	pin_set_type(PINID_LCD_D02, PIN_FUN1);
	pin_set_type(PINID_LCD_D03, PIN_FUN1);
	pin_set_type(PINID_LCD_D04, PIN_FUN1);
	pin_set_type(PINID_LCD_D05, PIN_FUN1);
	pin_set_type(PINID_LCD_D06, PIN_FUN1);
	pin_set_type(PINID_LCD_D07, PIN_FUN1);
	pin_set_type(PINID_LCD_D08, PIN_FUN1);
	pin_set_type(PINID_LCD_D09, PIN_FUN1);
	pin_set_type(PINID_LCD_D10, PIN_FUN1);
	pin_set_type(PINID_LCD_D11, PIN_FUN1);
	pin_set_type(PINID_LCD_D12, PIN_FUN1);
	pin_set_type(PINID_LCD_D13, PIN_FUN1);
	pin_set_type(PINID_LCD_D14, PIN_FUN1);
	pin_set_type(PINID_LCD_D15, PIN_FUN1);

	// Restore I80_HD_C, I80_HCS_L to function mode.
	pin_set_type(I80_HD_C, PIN_FUN1);
	pin_set_type(I80_HCS_L, PIN_FUN1);
#endif

	return data;
}

//-----------------------------------------------------------
//Host controller function 1
//Wait for host data Bus Ready
//-----------------------------------------------------------
inline void LCDWaitForReady()
{
	while (pin_gpio_get(I80_HRDY) == 0);

/*
	int retry_count = 10000;

	//Regarding to HRDY
	//you may need to use a GPIO pin connected to HRDY of IT8951
	while (pin_gpio_get(I80_HRDY) == 0 && retry_count > 0) {
		udelay(1000*100);
	}

	if (retry_count <= 0) {
		serial_puts("IT8951 waite for read timeout.\n");
		g_it8951_init = 0;
	}
*/
}

//-----------------------------------------------------------------
//Host controller function 2
//Write command code to host data Bus
//-----------------------------------------------------------------
void LCDWriteCmdCode(TWord cmd)
{
	//wait for ready
	LCDWaitForReady();

#ifdef I80_GPIO
	gpio_i80_16b_cmd_out(cmd);
#else
	// Stop run.
	lcdif_write(HW_LCDIF_CTRL_CLR_ADDR,
		    BM_LCDIF_CTRL_LCDIF_MASTER | BM_LCDIF_CTRL_RUN);

	// Tells the LCDIF one word data will be sent.
	lcdif_write(HW_LCDIF_TRANSFER_COUNT_ADDR,
		    BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) |
		    BF_LCDIF_TRANSFER_COUNT_H_COUNT(1));

	// Tells the LCDIF the word is data (will clear I80_HD_C function pin for select cmd).
	lcdif_write(HW_LCDIF_CTRL_CLR_ADDR, BM_LCDIF_CTRL_DATA_SELECT);

	// Start LCDIF.
	lcdif_write(HW_LCDIF_CTRL_SET_ADDR, BM_LCDIF_CTRL_RUN);

	// Wait that LCD read datapath FIFO is full.
	while (lcdif_read(HW_LCDIF_STAT_ADDR) & BM_LCDIF_STAT_LFIFO_FULL) {
		;
	}

	// Output the data to D0-D15 function pin.
	lcdif_write(HW_LCDIF_DATA_ADDR, cmd);

	// Wait LCDIF done.
	while(lcdif_read(HW_LCDIF_CTRL_ADDR) & BM_LCDIF_CTRL_RUN) {
		;
	}

	//lcdif_write(HW_LCDIF_CTRL1_CLR_ADDR, BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ);
#endif
}

//-----------------------------------------------------------
//Host controller function 3
//Write Data to host data Bus
//-----------------------------------------------------------
void LCDWriteData(TWord data)
{

	//wait for ready
	LCDWaitForReady();

#ifdef I80_GPIO
	gpio_i80_16b_data_out(data);
#else
	// Stop run.
	lcdif_write(HW_LCDIF_CTRL_CLR_ADDR,
		    BM_LCDIF_CTRL_LCDIF_MASTER | BM_LCDIF_CTRL_RUN);

	// Tells the LCDIF one word data will be sent.
	lcdif_write(HW_LCDIF_TRANSFER_COUNT_ADDR,
		    BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) |
		    BF_LCDIF_TRANSFER_COUNT_H_COUNT(1));

	// Tells the LCDIF the word is data (will set I80_HD_C function pin for select cmd).
	lcdif_write(HW_LCDIF_CTRL_SET_ADDR, BM_LCDIF_CTRL_DATA_SELECT);

	// Start LCDIF.
	lcdif_write(HW_LCDIF_CTRL_SET_ADDR, BM_LCDIF_CTRL_RUN);

	// Wait that LCD read datapath FIFO is full.
	while (lcdif_read(HW_LCDIF_STAT_ADDR) & BM_LCDIF_STAT_LFIFO_FULL) {
		;
	}

	// Output the data to D0-D15 function pin.
	lcdif_write(HW_LCDIF_DATA_ADDR, data);

	// Wait LCDIF done.
	while(lcdif_read(HW_LCDIF_CTRL_ADDR) & BM_LCDIF_CTRL_RUN) {
		;
	}

	//lcdif_write(HW_LCDIF_CTRL1_CLR_ADDR, BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ);
#endif
}

//-----------------------------------------------------------
//Host controller function 4
//Read Data from host data Bus
//-----------------------------------------------------------
TWord LCDReadData()
{
	TWord data;
	//wait for ready
	LCDWaitForReady();
	//read data from host data bus
	data = gpio_i80_16b_data_in();
	return data;
}

//-----------------------------------------------------------
//Host controller function 5
// Write command to host data Bus with aruments
//-----------------------------------------------------------
void LCDSendCmdArg(TWord cmdCode, TWord* pArg, TWord numArg)
{
     uint16_t i;

     //Send Cmd code
     LCDWriteCmdCode(cmdCode);

     //Send Data
     for (i=0; i < numArg; i++) {
         LCDWriteData(pArg[i]);
     }
}


///////////////////////////////////////////////////////////////////////////
int  mpulcd_is_init()
{
	return g_it8951_init;
}

void mpulcd_update_rectangle(uchar* pBase, ushort x, ushort y, ushort width, ushort height)
{
	if (g_it8951_init == 0) {
		return;
	}

	IT8951LdImgInfo stLdImgInfo;
	IT8951AreaImgInfo stAreaImgInfo;

	ushort x0, y0;
	ushort x1, y1;
	TWord* pFrame;

	//Word align
	x0 = x;
	y0 = y;
	x1 = x0 + width;
	y1 = y0 + height;

	if ((x0 % 2) != 0) {
		x0--;
	}
	if ((x1 % 2) != 0) {
		x1++;
	}
	width = x1 - x0;

	//Load Image and Display
	//Setting Load image information
	stLdImgInfo.ulStartFBAddr    = (TDWord)NULL; // no used, framebuffer adress is set by mxs_lcdif_dma_init();
	stLdImgInfo.usEndianType     = IT8951_LDIMG_L_ENDIAN;
	stLdImgInfo.usPixelFormat    = IT8951_8BPP;
	stLdImgInfo.usRotate         = IT8951_ROTATE_0;
	stLdImgInfo.ulImgBufBaseAddr = g_it8951_base_addr;
	//Set Load Area
	stAreaImgInfo.usX      = x0;
	stAreaImgInfo.usY      = y0;
	stAreaImgInfo.usWidth  = width;
	stAreaImgInfo.usHeight = height;

	//Send Load Image start Cmd
	IT8951LoadImgAreaStart(&stLdImgInfo, &stAreaImgInfo);
#ifdef I80_GPIO
	for (y = y0; y < y1; y++) {
		pFrame = (TWord *)(pBase + y * CONFIG_LCD_WIDTH + x0);
		for (x = x0; x < x1; x++, x++) {
			LCDWriteData(*pFrame);
			pFrame++;
		}
	}
#else
	int lineSize = (width >> 1);
	for (y = y0; y < y1; y++) {
		pFrame = (TWord *)(pBase + y * CONFIG_LCD_WIDTH + x0);
		mxs_lcdif_dma_init((dma_addr_t)pFrame, width);

		lcdif_write(HW_LCDIF_CTRL_SET_ADDR, BM_LCDIF_CTRL_DATA_SELECT);
		lcdif_write(HW_LCDIF_TRANSFER_COUNT_ADDR,
			    BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) |
			    BF_LCDIF_TRANSFER_COUNT_H_COUNT(lineSize));
		lcdif_write(HW_LCDIF_CTRL_SET_ADDR, BM_LCDIF_CTRL_LCDIF_MASTER);
		lcdif_write(HW_LCDIF_CTRL_SET_ADDR, BM_LCDIF_CTRL_RUN);

		// Wait DMA done.
		while(lcdif_read(HW_LCDIF_CTRL_ADDR) & BM_LCDIF_CTRL_RUN);
	}
#endif

	//Send Load Img End Command
	IT8951LoadImgEnd();

	// Display Area.
	while(IT8951ReadReg(LUTAFSR) == 0xFFFF); //Wait if All of 16 Engines are busy
//serial_puts("+++.\n");
	IT8951DisplayAreaBuf(x0, y0, width, height, 4, g_it8951_base_addr);
//serial_puts("---.\n");
}

void mpulcd_update_screen(void)
{
	if (g_it8951_init == 0) {
		return;
	}

	IT8951WaitForDisplayReady();
	mpulcd_update_rectangle(g_lcd_frame_base, 0, 0, CONFIG_LCD_WIDTH, CONFIG_LCD_HEIGH);
	IT8951WaitForDisplayReady();
}

void mpulcd_clean_screen(void)
{
	if (g_it8951_init == 0) {
		return;
	}

	IT8951WaitForDisplayReady();
	mpulcd_refresh_screen(0);
	mpulcd_update_screen();
}

void mpulcd_clean_screenEx(void)
{
	if (g_it8951_init == 0) {
		return;
	}
	//IT8951WaitForDisplayReady();
	IT8951DisplayAreaEx(0, 0, CONFIG_LCD_WIDTH, CONFIG_LCD_HEIGH, 0x0000, 0x000F);
}


// Refresh all screen.
void mpulcd_refresh_screen(int mode)
{
	if (g_it8951_init == 0) {
		return;
	}

	IT8951LdImgInfo stLdImgInfo;
	IT8951AreaImgInfo stAreaImgInfo;

	IT8951WaitForDisplayReady();

	//Load Image and Display
	//Setting Load image information
	stLdImgInfo.ulStartFBAddr    = (TDWord)NULL; // no used, framebuffer adress is set by mxs_lcdif_dma_init();
	stLdImgInfo.usEndianType     = IT8951_LDIMG_L_ENDIAN;
	stLdImgInfo.usPixelFormat    = IT8951_8BPP;
	stLdImgInfo.usRotate         = IT8951_ROTATE_0;
	stLdImgInfo.ulImgBufBaseAddr = g_it8951_dev_info.usImgBufAddrL | (g_it8951_dev_info.usImgBufAddrH << 16);
	//Set Load Area
	stAreaImgInfo.usX      = 0;
	stAreaImgInfo.usY      = 0;
	stAreaImgInfo.usWidth  = CONFIG_LCD_WIDTH;
	stAreaImgInfo.usHeight = CONFIG_LCD_HEIGH;

	//Set Image buffer(IT8951) Base address.
	IT8951SetImgBufBaseAddr(g_it8951_base_addr); // get from GetIT8951SystemInfo()

	//Send Load Image start Cmd
	IT8951LoadImgAreaStart(&stLdImgInfo, &stAreaImgInfo);

#ifdef I80_GPIO
	int i;
	TWord *pFrame = g_lcd_frame_base;
	for (i = CONFIG_LCD_WIDTH/2 * CONFIG_LCD_HEIGH; i >= 0; i--) {
		LCDWriteData(*pFrame);
		pFrame++;
	}
#else
	//Write pixel data. the address is set by mxs_lcdif_dma_init()
	// or mpulcd_pan_display().
	lcdif_write(HW_LCDIF_CTRL_SET_ADDR, BM_LCDIF_CTRL_DATA_SELECT);
	lcdif_write(HW_LCDIF_TRANSFER_COUNT_ADDR,
		    BF_LCDIF_TRANSFER_COUNT_V_COUNT(CONFIG_LCD_HEIGH) |
		    BF_LCDIF_TRANSFER_COUNT_H_COUNT(CONFIG_LCD_WIDTH/2));
	lcdif_write(HW_LCDIF_CTRL_SET_ADDR, BM_LCDIF_CTRL_LCDIF_MASTER);
	lcdif_write(HW_LCDIF_CTRL_SET_ADDR, BM_LCDIF_CTRL_RUN);
	while(lcdif_read(HW_LCDIF_CTRL_ADDR) & BM_LCDIF_CTRL_RUN);
#endif

	//Send Load Img End Command
	IT8951LoadImgEnd();

	// Display Area ? (x,y,w,h) with mode 2 for fast gray clear mode - depends on current waveform
	IT8951DisplayArea(0, 0, CONFIG_LCD_WIDTH, CONFIG_LCD_HEIGH, mode);
}


static void mpulcd_start_refresh(void)
{
}

static void mpulcd_display_on(void)
{
	// 这里一定不要用printf打调试信息会造成递归死循环
	// printf(console.c)->putc(lcd.c)->lcd_enable->mpulcd_display_on.
//	IT8951SystemRun();
	//mpulcd_start_refresh();
}

static void mpulcd_display_off(void)
{
//	IT8951StandBy();
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

static void prv_init_lcdif(void)
{
	lcdif_write(HW_LCDIF_CTRL_CLR_ADDR,
		    BM_LCDIF_CTRL_CLKGATE |
		    BM_LCDIF_CTRL_SFTRST);

	lcdif_write(HW_LCDIF_CTRL_ADDR,
		    BF_LCDIF_CTRL_LCD_DATABUS_WIDTH(BV_LCDIF_CTRL_LCD_DATABUS_WIDTH__16_BIT) |
		    BF_LCDIF_CTRL_WORD_LENGTH(BV_LCDIF_CTRL_WORD_LENGTH__16_BIT));

	lcdif_write(HW_LCDIF_CTRL1_CLR_ADDR, BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT);
	lcdif_write(HW_LCDIF_CTRL1_SET_ADDR, BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(0xf));

	lcdif_write(HW_LCDIF_TIMING_ADDR, 0x01010101);
}

static void prv_init_panel_hw(void)
{
	printf("IT8951 reset.\n");
	
#ifdef I80_GPIO
	pin_gpio_set(I80_HRST_L, 0);
	mdelay(1);
	pin_gpio_set(I80_HRST_L, 1);
	mdelay(5);
#else
	/*
	 * Make sure we do a high-to-low transition to reset the panel.
	 * First make it low for 100 msec, hi for 10 msec, low for 10 msec,
	 * then hi.
         *
         * Means send reset signal on I80_HRST_L function pin to reset panel.
	 */	
	writel(BM_LCDIF_CTRL1_RESET, HW_LCDIF_CTRL1_CLR_ADDR);	/* low */
	mdelay(1);
	writel(BM_LCDIF_CTRL1_RESET, HW_LCDIF_CTRL1_SET_ADDR);	/* high */
	mdelay(5);
#endif

	g_it8951_init = 1;

	int max_retry = 10;
	//printf("read EPD information...\r\n");

	while (max_retry > 0) {
		// Get panel info.
		IT8951GetSystemInfo(&g_it8951_dev_info);

		if (g_it8951_dev_info.usPanelW != CONFIG_LCD_WIDTH ||
			g_it8951_dev_info.usPanelH != CONFIG_LCD_HEIGH) {
			max_retry--;
			printf("read retry count = %d\r\n", (10-max_retry));
			continue;
		}

		break;
	}

	if (max_retry <= 0) {
		printf ("IT8951 is some thing wrong. workaround it...\n");
		g_it8951_init = 0;
		return;
	}
	printf("read EPD information ok\r\n");

	g_it8951_base_addr = g_it8951_dev_info.usImgBufAddrL | (g_it8951_dev_info.usImgBufAddrH << 16);

	//Set to Enable I80 Packed mode
	IT8951WriteReg(I80CPCR, 0x0001);
#if 0
	// PP unit VCOM will be handled by ITE firmware .(20180305)
	//Set VCOM
	{
		extern uint16_t load_vcom();
		TWord old_vcom, new_vcom;
		old_vcom = IT8951GetVCOM();
		new_vcom = (TWord)load_vcom();
		
		printf ("IT8951 VCOM: %d.\n", (int)old_vcom);
		if (new_vcom != 0 && new_vcom != old_vcom) {
			IT8951SetVCOM(new_vcom);
			printf ("IT8951 VCOM: %d->%d.\n", (int)old_vcom, (int)new_vcom);
			printf ("IT8951 VCOM: %d.\n", (int)IT8951GetVCOM());
		}
	}
#endif	
}

static int mpulcd_init_panel(dma_addr_t phys, int memsize,
			     struct mxs_platform_fb_entry *pentry)
{
	g_lcd_frame_base = (uchar*)phys;

#ifdef I80_GPIO
	printf ("IT8951 gpio.\n");
#else
	printf ("IT8951 lcdif.\n");
#endif
	// Config gpio.
	gpio_i80_pin_config();

	// Open Power supply.
	pin_gpio_direction(LCD_PWR_CTRL, 1); /* output */
	pin_gpio_set(LCD_PWR_CTRL, 1);

	imx_set_lcdifclk(PICOS2KHZ(pentry->cycle_time_ns));

	prv_init_lcdif();
	mxs_lcdif_dma_init(phys, memsize);
	prv_init_panel_hw();

	//Clear screen to white.
	//memset(g_lcd_frame_base, 0xFF, memsize);
	//mpulcd_refresh_screen(0);



/*
	printf ("IT8951 PowerOn\n");

	IT8951PowerOn();
while(1) {
	udelay(1000*1000);
	printf ("IT8951 Temperature: %d..\n", (int)IT8951GetTemperature());

//	IT8951SetTemperature(0x14);

//	printf ("IT8951 Temperature: %d..\n", (int)IT8951GetTemperature());
}
//	IT8951PowerOff();
*/
	return 0;
}

struct mxs_platform_fb_entry fb_entry = {
	.name           = "dm30_it8951_i80",
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

