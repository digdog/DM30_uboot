/*
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
 *
 * Author: Vladislav Buzov <vbuzov@embeddedalley.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef __ASM_ARCH_PINS_H
#define __ASM_ARCH_PINS_H

//#include <mach/pinctrl.h>

/*
 * Define MX28 pins, the pin name corresponds to MX28 hardware
 * interface this pin belongs to.
 */

/* Bank 0 */
#define PINID_GPMI_D00		MXS_PIN_ENCODE(0, 0)
#define PINID_GPMI_D01		MXS_PIN_ENCODE(0, 1)
#define PINID_GPMI_D02		MXS_PIN_ENCODE(0, 2)
#define PINID_GPMI_D03		MXS_PIN_ENCODE(0, 3)
#define PINID_GPMI_D04		MXS_PIN_ENCODE(0, 4)
#define PINID_GPMI_D05		MXS_PIN_ENCODE(0, 5)
#define PINID_GPMI_D06		MXS_PIN_ENCODE(0, 6)
#define PINID_GPMI_D07		MXS_PIN_ENCODE(0, 7)
#define PINID_GPMI_D08		MXS_PIN_ENCODE(0, 8)
#define PINID_GPMI_D09		MXS_PIN_ENCODE(0, 9)
#define PINID_GPMI_D10		MXS_PIN_ENCODE(0, 10)
#define PINID_GPMI_D11		MXS_PIN_ENCODE(0, 11)
#define PINID_GPMI_D12		MXS_PIN_ENCODE(0, 12)
#define PINID_GPMI_D13		MXS_PIN_ENCODE(0, 13)
#define PINID_GPMI_D14		MXS_PIN_ENCODE(0, 14)
#define PINID_GPMI_D15		MXS_PIN_ENCODE(0, 15)
#define PINID_GPMI_CLE		MXS_PIN_ENCODE(0, 16)
#define PINID_GPMI_ALE		MXS_PIN_ENCODE(0, 17)
#define PINID_GPMI_CE2N		MXS_PIN_ENCODE(0, 18)
#define PINID_GPMI_RDY0		MXS_PIN_ENCODE(0, 19)
#define PINID_GPMI_RDY1		MXS_PIN_ENCODE(0, 20)
#define PINID_GPMI_RDY2		MXS_PIN_ENCODE(0, 21)
#define PINID_GPMI_RDY3		MXS_PIN_ENCODE(0, 22)
#define PINID_GPMI_WPN		MXS_PIN_ENCODE(0, 23)
#define PINID_GPMI_WRN		MXS_PIN_ENCODE(0, 24)
#define PINID_GPMI_RDN		MXS_PIN_ENCODE(0, 25)
#define PINID_AUART1_CTS	MXS_PIN_ENCODE(0, 26)
#define PINID_AUART1_RTS	MXS_PIN_ENCODE(0, 27)
#define PINID_AUART1_RX		MXS_PIN_ENCODE(0, 28)
#define PINID_AUART1_TX		MXS_PIN_ENCODE(0, 29)
#define PINID_I2C_SCL		MXS_PIN_ENCODE(0, 30)
#define PINID_I2C_SDA		MXS_PIN_ENCODE(0, 31)

/* Bank 1 */
#define PINID_LCD_D00		MXS_PIN_ENCODE(1, 0)
#define PINID_LCD_D01		MXS_PIN_ENCODE(1, 1)
#define PINID_LCD_D02		MXS_PIN_ENCODE(1, 2)
#define PINID_LCD_D03		MXS_PIN_ENCODE(1, 3)
#define PINID_LCD_D04		MXS_PIN_ENCODE(1, 4)
#define PINID_LCD_D05		MXS_PIN_ENCODE(1, 5)
#define PINID_LCD_D06		MXS_PIN_ENCODE(1, 6)
#define PINID_LCD_D07		MXS_PIN_ENCODE(1, 7)
#define PINID_LCD_D08		MXS_PIN_ENCODE(1, 8)
#define PINID_LCD_D09		MXS_PIN_ENCODE(1, 9)
#define PINID_LCD_D10		MXS_PIN_ENCODE(1, 10)
#define PINID_LCD_D11		MXS_PIN_ENCODE(1, 11)
#define PINID_LCD_D12		MXS_PIN_ENCODE(1, 12)
#define PINID_LCD_D13		MXS_PIN_ENCODE(1, 13)
#define PINID_LCD_D14		MXS_PIN_ENCODE(1, 14)
#define PINID_LCD_D15		MXS_PIN_ENCODE(1, 15)
#define PINID_LCD_D16		MXS_PIN_ENCODE(1, 16)
#define PINID_LCD_D17		MXS_PIN_ENCODE(1, 17)
#define PINID_LCD_RESET		MXS_PIN_ENCODE(1, 18)
#define PINID_LCD_RS		MXS_PIN_ENCODE(1, 19)
#define PINID_LCD_WR		MXS_PIN_ENCODE(1, 20)
#define PINID_LCD_CS		MXS_PIN_ENCODE(1, 21)
#define PINID_LCD_DOTCK		MXS_PIN_ENCODE(1, 22)
#define PINID_LCD_ENABLE	MXS_PIN_ENCODE(1, 23)
#define PINID_LCD_HSYNC		MXS_PIN_ENCODE(1, 24)
#define PINID_LCD_VSYNC		MXS_PIN_ENCODE(1, 25)
#define PINID_PWM0		MXS_PIN_ENCODE(1, 26)
#define PINID_PWM1		MXS_PIN_ENCODE(1, 27)
#define PINID_PWM2		MXS_PIN_ENCODE(1, 28)
#define PINID_PWM3		MXS_PIN_ENCODE(1, 29)
#define PINID_PWM4		MXS_PIN_ENCODE(1, 30)

/* Bank 2 */
#define PINID_SSP1_CMD		MXS_PIN_ENCODE(2, 0)
#define PINID_SSP1_DETECT	MXS_PIN_ENCODE(2, 1)
#define PINID_SSP1_DATA0	MXS_PIN_ENCODE(2, 2)
#define PINID_SSP1_DATA1	MXS_PIN_ENCODE(2, 3)
#define PINID_SSP1_DATA2	MXS_PIN_ENCODE(2, 4)
#define PINID_SSP1_DATA3	MXS_PIN_ENCODE(2, 5)
#define PINID_SSP1_SCK		MXS_PIN_ENCODE(2, 6)
#define PINID_ROTARYA		MXS_PIN_ENCODE(2, 7)
#define PINID_ROTARYB		MXS_PIN_ENCODE(2, 8)
#define PINID_EMI_A00		MXS_PIN_ENCODE(2, 9)
#define PINID_EMI_A01		MXS_PIN_ENCODE(2, 10)
#define PINID_EMI_A02		MXS_PIN_ENCODE(2, 11)
#define PINID_EMI_A03		MXS_PIN_ENCODE(2, 12)
#define PINID_EMI_A04		MXS_PIN_ENCODE(2, 13)
#define PINID_EMI_A05		MXS_PIN_ENCODE(2, 14)
#define PINID_EMI_A06		MXS_PIN_ENCODE(2, 15)
#define PINID_EMI_A07		MXS_PIN_ENCODE(2, 16)
#define PINID_EMI_A08		MXS_PIN_ENCODE(2, 17)
#define PINID_EMI_A09		MXS_PIN_ENCODE(2, 18)
#define PINID_EMI_A10		MXS_PIN_ENCODE(2, 19)
#define PINID_EMI_A11		MXS_PIN_ENCODE(2, 20)
#define PINID_EMI_A12		MXS_PIN_ENCODE(2, 21)
#define PINID_EMI_BA0		MXS_PIN_ENCODE(2, 22)
#define PINID_EMI_BA1		MXS_PIN_ENCODE(2, 23)
#define PINID_EMI_CASN		MXS_PIN_ENCODE(2, 24)
#define PINID_EMI_CE0N		MXS_PIN_ENCODE(2, 25)
#define PINID_EMI_CE1N		MXS_PIN_ENCODE(2, 26)
#define PINID_GPMI_CE1N		MXS_PIN_ENCODE(2, 27)
#define PINID_GPMI_CE0N		MXS_PIN_ENCODE(2, 28)
#define PINID_EMI_CKE		MXS_PIN_ENCODE(2, 29)
#define PINID_EMI_RASN		MXS_PIN_ENCODE(2, 30)
#define PINID_EMI_WEN		MXS_PIN_ENCODE(2, 31)

/* Bank 3 */
#define PINID_EMI_D00		MXS_PIN_ENCODE(3, 0)
#define PINID_EMI_D01		MXS_PIN_ENCODE(3, 1)
#define PINID_EMI_D02		MXS_PIN_ENCODE(3, 2)
#define PINID_EMI_D03		MXS_PIN_ENCODE(3, 3)
#define PINID_EMI_D04		MXS_PIN_ENCODE(3, 4)
#define PINID_EMI_D05		MXS_PIN_ENCODE(3, 5)
#define PINID_EMI_D06		MXS_PIN_ENCODE(3, 6)
#define PINID_EMI_D07		MXS_PIN_ENCODE(3, 7)
#define PINID_EMI_D08		MXS_PIN_ENCODE(3, 8)
#define PINID_EMI_D09		MXS_PIN_ENCODE(3, 9)
#define PINID_EMI_D10		MXS_PIN_ENCODE(3, 10)
#define PINID_EMI_D11		MXS_PIN_ENCODE(3, 11)
#define PINID_EMI_D12		MXS_PIN_ENCODE(3, 12)
#define PINID_EMI_D13		MXS_PIN_ENCODE(3, 13)
#define PINID_EMI_D14		MXS_PIN_ENCODE(3, 14)
#define PINID_EMI_D15		MXS_PIN_ENCODE(3, 15)
#define PINID_EMI_DQM0		MXS_PIN_ENCODE(3, 16)
#define PINID_EMI_DQM1		MXS_PIN_ENCODE(3, 17)
#define PINID_EMI_DQS0		MXS_PIN_ENCODE(3, 18)
#define PINID_EMI_DQS1		MXS_PIN_ENCODE(3, 19)
#define PINID_EMI_CLK		MXS_PIN_ENCODE(3, 20)
#define PINID_EMI_CLKN		MXS_PIN_ENCODE(3, 21)

#endif /* __ASM_ARCH_PINS_H */
