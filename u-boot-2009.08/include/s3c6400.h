/*
 * (C) Copyright 2007
 * Byungjae Lee, Samsung Erectronics, bjlee@samsung.com.
 *      - only support for S3C6400
 *
 * (C) Copyright 2008
 * Guennadi Liakhovetki, DENX Software Engineering, <lg@denx.de>
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

/************************************************
 * NAME	    : s3c6400.h
 *
 * Based on S3C6400 User's manual Rev 0.0
 ************************************************/

#ifndef __S3C6400_H__
#define __S3C6400_H__

#define S3C64XX_UART_CHANNELS	3
#define S3C64XX_SPI_CHANNELS	2

#include <asm/hardware.h>

#define ELFIN_CLOCK_POWER_BASE	0x7e00f000

/* Clock & Power Controller for mDirac3*/
#define APLL_LOCK_OFFSET	0x00
#define MPLL_LOCK_OFFSET	0x04
#define EPLL_LOCK_OFFSET	0x08
#define APLL_CON_OFFSET		0x0C
#define MPLL_CON_OFFSET		0x10
#define EPLL_CON0_OFFSET	0x14
#define EPLL_CON1_OFFSET	0x18
#define CLK_SRC_OFFSET		0x1C
#define CLK_DIV0_OFFSET		0x20
#define CLK_DIV1_OFFSET		0x24
#define CLK_DIV2_OFFSET		0x28
#define CLK_OUT_OFFSET		0x2C
#define HCLK_GATE_OFFSET	0x30
#define PCLK_GATE_OFFSET	0x34
#define SCLK_GATE_OFFSET	0x38
#define AHB_CON0_OFFSET		0x100
#define AHB_CON1_OFFSET		0x104
#define AHB_CON2_OFFSET		0x108
#define SELECT_DMA_OFFSET	0x110
#define SW_RST_OFFSET		0x114
#define SYS_ID_OFFSET		0x118
#define MEM_SYS_CFG_OFFSET	0x120
#define QOS_OVERRIDE0_OFFSET	0x124
#define QOS_OVERRIDE1_OFFSET	0x128
#define MEM_CFG_STAT_OFFSET	0x12C
#define PWR_CFG_OFFSET		0x804
#define EINT_MASK_OFFSET	0x808
#define NOR_CFG_OFFSET		0x810
#define STOP_CFG_OFFSET		0x814
#define SLEEP_CFG_OFFSET	0x818
#define OSC_FREQ_OFFSET		0x820
#define OSC_STABLE_OFFSET	0x824
#define PWR_STABLE_OFFSET	0x828
#define FPC_STABLE_OFFSET	0x82C
#define MTC_STABLE_OFFSET	0x830
#define OTHERS_OFFSET		0x900
#define RST_STAT_OFFSET		0x904
#define WAKEUP_STAT_OFFSET	0x908
#define BLK_PWR_STAT_OFFSET	0x90C
#define INF_REG0_OFFSET		0xA00
#define INF_REG1_OFFSET		0xA04
#define INF_REG2_OFFSET		0xA08
#define INF_REG3_OFFSET		0xA0C
#define INF_REG4_OFFSET		0xA10
#define INF_REG5_OFFSET		0xA14
#define INF_REG6_OFFSET		0xA18
#define INF_REG7_OFFSET		0xA1C

#define OSC_CNT_VAL_OFFSET	0x824
#define PWR_CNT_VAL_OFFSET	0x828
#define FPC_CNT_VAL_OFFSET	0x82C
#define MTC_CNT_VAL_OFFSET	0x830

#define APLL_LOCK_REG		__REG(ELFIN_CLOCK_POWER_BASE + APLL_LOCK_OFFSET)
#define MPLL_LOCK_REG		__REG(ELFIN_CLOCK_POWER_BASE + MPLL_LOCK_OFFSET)
#define EPLL_LOCK_REG		__REG(ELFIN_CLOCK_POWER_BASE + EPLL_LOCK_OFFSET)
#define APLL_CON_REG		__REG(ELFIN_CLOCK_POWER_BASE + APLL_CON_OFFSET)
#define MPLL_CON_REG		__REG(ELFIN_CLOCK_POWER_BASE + MPLL_CON_OFFSET)
#define EPLL_CON0_REG		__REG(ELFIN_CLOCK_POWER_BASE + EPLL_CON0_OFFSET)
#define EPLL_CON1_REG		__REG(ELFIN_CLOCK_POWER_BASE + EPLL_CON1_OFFSET)
#define CLK_SRC_REG		__REG(ELFIN_CLOCK_POWER_BASE + CLK_SRC_OFFSET)
#define CLK_DIV0_REG		__REG(ELFIN_CLOCK_POWER_BASE + CLK_DIV0_OFFSET)
#define CLK_DIV1_REG		__REG(ELFIN_CLOCK_POWER_BASE + CLK_DIV1_OFFSET)
#define CLK_DIV2_REG		__REG(ELFIN_CLOCK_POWER_BASE + CLK_DIV2_OFFSET)
#define CLK_OUT_REG		__REG(ELFIN_CLOCK_POWER_BASE + CLK_OUT_OFFSET)
#define HCLK_GATE_REG		__REG(ELFIN_CLOCK_POWER_BASE + HCLK_GATE_OFFSET)
#define PCLK_GATE_REG		__REG(ELFIN_CLOCK_POWER_BASE + PCLK_GATE_OFFSET)
#define SCLK_GATE_REG		__REG(ELFIN_CLOCK_POWER_BASE + SCLK_GATE_OFFSET)
#define AHB_CON0_REG		__REG(ELFIN_CLOCK_POWER_BASE + AHB_CON0_OFFSET)
#define AHB_CON1_REG		__REG(ELFIN_CLOCK_POWER_BASE + AHB_CON1_OFFSET)
#define AHB_CON2_REG		__REG(ELFIN_CLOCK_POWER_BASE + AHB_CON2_OFFSET)
#define SELECT_DMA_REG		__REG(ELFIN_CLOCK_POWER_BASE + \
				      SELECT_DMA_OFFSET)
#define SW_RST_REG		__REG(ELFIN_CLOCK_POWER_BASE + SW_RST_OFFSET)
#define SYS_ID_REG		__REG(ELFIN_CLOCK_POWER_BASE + SYS_ID_OFFSET)
#define MEM_SYS_CFG_REG		__REG(ELFIN_CLOCK_POWER_BASE + \
				      MEM_SYS_CFG_OFFSET)
#define QOS_OVERRIDE0_REG	__REG(ELFIN_CLOCK_POWER_BASE + \
				      QOS_OVERRIDE0_OFFSET)
#define QOS_OVERRIDE1_REG	__REG(ELFIN_CLOCK_POWER_BASE + \
				      QOS_OVERRIDE1_OFFSET)
#define MEM_CFG_STAT_REG	__REG(ELFIN_CLOCK_POWER_BASE + \
				      MEM_CFG_STAT_OFFSET)
#define PWR_CFG_REG		__REG(ELFIN_CLOCK_POWER_BASE + PWR_CFG_OFFSET)
#define EINT_MASK_REG		__REG(ELFIN_CLOCK_POWER_BASE + EINT_MASK_OFFSET)
#define NOR_CFG_REG		__REG(ELFIN_CLOCK_POWER_BASE + NOR_CFG_OFFSET)
#define STOP_CFG_REG		__REG(ELFIN_CLOCK_POWER_BASE + STOP_CFG_OFFSET)
#define SLEEP_CFG_REG		__REG(ELFIN_CLOCK_POWER_BASE + SLEEP_CFG_OFFSET)
#define OSC_FREQ_REG		__REG(ELFIN_CLOCK_POWER_BASE + OSC_FREQ_OFFSET)
#define OSC_CNT_VAL_REG		__REG(ELFIN_CLOCK_POWER_BASE + \
				      OSC_CNT_VAL_OFFSET)
#define PWR_CNT_VAL_REG		__REG(ELFIN_CLOCK_POWER_BASE + \
				      PWR_CNT_VAL_OFFSET)
#define FPC_CNT_VAL_REG		__REG(ELFIN_CLOCK_POWER_BASE + \
				      FPC_CNT_VAL_OFFSET)
#define MTC_CNT_VAL_REG		__REG(ELFIN_CLOCK_POWER_BASE + \
				      MTC_CNT_VAL_OFFSET)
#define OTHERS_REG		__REG(ELFIN_CLOCK_POWER_BASE + OTHERS_OFFSET)
#define RST_STAT_REG		__REG(ELFIN_CLOCK_POWER_BASE + RST_STAT_OFFSET)
#define WAKEUP_STAT_REG		__REG(ELFIN_CLOCK_POWER_BASE + \
				      WAKEUP_STAT_OFFSET)
#define BLK_PWR_STAT_REG	__REG(ELFIN_CLOCK_POWER_BASE + \
				      BLK_PWR_STAT_OFFSET)
#define INF_REG0_REG		__REG(ELFIN_CLOCK_POWER_BASE + INF_REG0_OFFSET)
#define INF_REG1_REG		__REG(ELFIN_CLOCK_POWER_BASE + INF_REG1_OFFSET)
#define INF_REG2_REG		__REG(ELFIN_CLOCK_POWER_BASE + INF_REG2_OFFSET)
#define INF_REG3_REG		__REG(ELFIN_CLOCK_POWER_BASE + INF_REG3_OFFSET)
#define INF_REG4_REG		__REG(ELFIN_CLOCK_POWER_BASE + INF_REG4_OFFSET)
#define INF_REG5_REG		__REG(ELFIN_CLOCK_POWER_BASE + INF_REG5_OFFSET)
#define INF_REG6_REG		__REG(ELFIN_CLOCK_POWER_BASE + INF_REG6_OFFSET)
#define INF_REG7_REG		__REG(ELFIN_CLOCK_POWER_BASE + INF_REG7_OFFSET)

#define APLL_LOCK	(ELFIN_CLOCK_POWER_BASE + APLL_LOCK_OFFSET)
#define MPLL_LOCK	(ELFIN_CLOCK_POWER_BASE + MPLL_LOCK_OFFSET)
#define EPLL_LOCK	(ELFIN_CLOCK_POWER_BASE + EPLL_LOCK_OFFSET)
#define APLL_CON	(ELFIN_CLOCK_POWER_BASE + APLL_CON_OFFSET)
#define MPLL_CON	(ELFIN_CLOCK_POWER_BASE + MPLL_CON_OFFSET)
#define EPLL_CON0	(ELFIN_CLOCK_POWER_BASE + EPLL_CON0_OFFSET)
#define EPLL_CON1	(ELFIN_CLOCK_POWER_BASE + EPLL_CON1_OFFSET)
#define CLK_SRC		(ELFIN_CLOCK_POWER_BASE + CLK_SRC_OFFSET)
#define CLK_DIV0	(ELFIN_CLOCK_POWER_BASE + CLK_DIV0_OFFSET)
#define CLK_DIV1	(ELFIN_CLOCK_POWER_BASE + CLK_DIV1_OFFSET)
#define CLK_DIV2	(ELFIN_CLOCK_POWER_BASE + CLK_DIV2_OFFSET)
#define CLK_OUT		(ELFIN_CLOCK_POWER_BASE + CLK_OUT_OFFSET)
#define HCLK_GATE	(ELFIN_CLOCK_POWER_BASE + HCLK_GATE_OFFSET)
#define PCLK_GATE	(ELFIN_CLOCK_POWER_BASE + PCLK_GATE_OFFSET)
#define SCLK_GATE	(ELFIN_CLOCK_POWER_BASE + SCLK_GATE_OFFSET)
#define AHB_CON0	(ELFIN_CLOCK_POWER_BASE + AHB_CON0_OFFSET)
#define AHB_CON1	(ELFIN_CLOCK_POWER_BASE + AHB_CON1_OFFSET)
#define AHB_CON2	(ELFIN_CLOCK_POWER_BASE + AHB_CON2_OFFSET)
#define SELECT_DMA	(ELFIN_CLOCK_POWER_BASE + SELECT_DMA_OFFSET)
#define SW_RST		(ELFIN_CLOCK_POWER_BASE + SW_RST_OFFSET)
#define SYS_ID		(ELFIN_CLOCK_POWER_BASE + SYS_ID_OFFSET)
#define MEM_SYS_CFG	(ELFIN_CLOCK_POWER_BASE + MEM_SYS_CFG_OFFSET)
#define QOS_OVERRIDE0	(ELFIN_CLOCK_POWER_BASE + QOS_OVERRIDE0_OFFSET)
#define QOS_OVERRIDE1	(ELFIN_CLOCK_POWER_BASE + QOS_OVERRIDE1_OFFSET)
#define MEM_CFG_STAT	(ELFIN_CLOCK_POWER_BASE + MEM_CFG_STAT_OFFSET)
#define PWR_CFG		(ELFIN_CLOCK_POWER_BASE + PWR_CFG_OFFSET)
#define EINT_MASK	(ELFIN_CLOCK_POWER_BASE + EINT_MASK_OFFSET)
#define NOR_CFG		(ELFIN_CLOCK_POWER_BASE + NOR_CFG_OFFSET)
#define STOP_CFG	(ELFIN_CLOCK_POWER_BASE + STOP_CFG_OFFSET)
#define SLEEP_CFG	(ELFIN_CLOCK_POWER_BASE + SLEEP_CFG_OFFSET)
#define OSC_FREQ	(ELFIN_CLOCK_POWER_BASE + OSC_FREQ_OFFSET)
#define OSC_CNT_VAL	(ELFIN_CLOCK_POWER_BASE + OSC_CNT_VAL_OFFSET)
#define PWR_CNT_VAL	(ELFIN_CLOCK_POWER_BASE + PWR_CNT_VAL_OFFSET)
#define FPC_CNT_VAL	(ELFIN_CLOCK_POWER_BASE + FPC_CNT_VAL_OFFSET)
#define MTC_CNT_VAL	(ELFIN_CLOCK_POWER_BASE + MTC_CNT_VAL_OFFSET)
#define OTHERS		(ELFIN_CLOCK_POWER_BASE + OTHERS_OFFSET)
#define RST_STAT	(ELFIN_CLOCK_POWER_BASE + RST_STAT_OFFSET)
#define WAKEUP_STAT	(ELFIN_CLOCK_POWER_BASE + WAKEUP_STAT_OFFSET)
#define BLK_PWR_STAT	(ELFIN_CLOCK_POWER_BASE + BLK_PWR_STAT_OFFSET)
#define INF_REG0	(ELFIN_CLOCK_POWER_BASE + INF_REG0_OFFSET)
#define INF_REG1	(ELFIN_CLOCK_POWER_BASE + INF_REG1_OFFSET)
#define INF_REG2	(ELFIN_CLOCK_POWER_BASE + INF_REG2_OFFSET)
#define INF_REG3	(ELFIN_CLOCK_POWER_BASE + INF_REG3_OFFSET)
#define INF_REG4	(ELFIN_CLOCK_POWER_BASE + INF_REG4_OFFSET)
#define INF_REG5	(ELFIN_CLOCK_POWER_BASE + INF_REG5_OFFSET)
#define INF_REG6	(ELFIN_CLOCK_POWER_BASE + INF_REG6_OFFSET)
#define INF_REG7	(ELFIN_CLOCK_POWER_BASE + INF_REG7_OFFSET)


/*
 * GPIO
 */
#define ELFIN_GPIO_BASE		0x7f008000

#define GPACON_OFFSET		0x00
#define GPADAT_OFFSET		0x04
#define GPAPUD_OFFSET		0x08
#define GPACONSLP_OFFSET	0x0C
#define GPAPUDSLP_OFFSET	0x10
#define GPBCON_OFFSET		0x20
#define GPBDAT_OFFSET		0x24
#define GPBPUD_OFFSET		0x28
#define GPBCONSLP_OFFSET	0x2C
#define GPBPUDSLP_OFFSET	0x30
#define GPCCON_OFFSET		0x40
#define GPCDAT_OFFSET		0x44
#define GPCPUD_OFFSET		0x48
#define GPCCONSLP_OFFSET	0x4C
#define GPCPUDSLP_OFFSET	0x50
#define GPDCON_OFFSET		0x60
#define GPDDAT_OFFSET		0x64
#define GPDPUD_OFFSET		0x68
#define GPDCONSLP_OFFSET	0x6C
#define GPDPUDSLP_OFFSET	0x70
#define GPECON_OFFSET		0x80
#define GPEDAT_OFFSET		0x84
#define GPEPUD_OFFSET		0x88
#define GPECONSLP_OFFSET	0x8C
#define GPEPUDSLP_OFFSET	0x90
#define GPFCON_OFFSET		0xA0
#define GPFDAT_OFFSET		0xA4
#define GPFPUD_OFFSET		0xA8
#define GPFCONSLP_OFFSET	0xAC
#define GPFPUDSLP_OFFSET	0xB0
#define GPGCON_OFFSET		0xC0
#define GPGDAT_OFFSET		0xC4
#define GPGPUD_OFFSET		0xC8
#define GPGCONSLP_OFFSET	0xCC
#define GPGPUDSLP_OFFSET	0xD0
#define GPHCON0_OFFSET		0xE0
#define GPHCON1_OFFSET		0xE4
#define GPHDAT_OFFSET		0xE8
#define GPHPUD_OFFSET		0xEC
#define GPHCONSLP_OFFSET	0xF0
#define GPHPUDSLP_OFFSET	0xF4
#define GPICON_OFFSET		0x100
#define GPIDAT_OFFSET		0x104
#define GPIPUD_OFFSET		0x108
#define GPICONSLP_OFFSET	0x10C
#define GPIPUDSLP_OFFSET	0x110
#define GPJCON_OFFSET		0x120
#define GPJDAT_OFFSET		0x124
#define GPJPUD_OFFSET		0x128
#define GPJCONSLP_OFFSET	0x12C
#define GPJPUDSLP_OFFSET	0x130
#define MEM0DRVCON_OFFSET	0x1D0
#define MEM1DRVCON_OFFSET	0x1D4
#define GPKCON0_OFFSET		0x800
#define GPKCON1_OFFSET		0x804
#define GPKDAT_OFFSET		0x808
#define GPKPUD_OFFSET		0x80C
#define GPLCON0_OFFSET		0x810
#define GPLCON1_OFFSET		0x814
#define GPLDAT_OFFSET		0x818
#define GPLPUD_OFFSET		0x81C
#define GPMCON_OFFSET		0x820
#define GPMDAT_OFFSET		0x824
#define GPMPUD_OFFSET		0x828
#define GPNCON_OFFSET		0x830
#define GPNDAT_OFFSET		0x834
#define GPNPUD_OFFSET		0x838
#define GPOCON_OFFSET		0x140
#define GPODAT_OFFSET		0x144
#define GPOPUD_OFFSET		0x148
#define GPOCONSLP_OFFSET	0x14C
#define GPOPUDSLP_OFFSET	0x150
#define GPPCON_OFFSET		0x160
#define GPPDAT_OFFSET		0x164
#define GPPPUD_OFFSET		0x168
#define GPPCONSLP_OFFSET	0x16C
#define GPPPUDSLP_OFFSET	0x170
#define GPQCON_OFFSET		0x180
#define GPQDAT_OFFSET		0x184
#define GPQPUD_OFFSET		0x188
#define GPQCONSLP_OFFSET	0x18C
#define GPQPUDSLP_OFFSET	0x190

#define EINTPEND_OFFSET		0x924

#define GPACON_REG		__REG(ELFIN_GPIO_BASE + GPACON_OFFSET)
#define GPADAT_REG		__REG(ELFIN_GPIO_BASE + GPADAT_OFFSET)
#define GPAPUD_REG		__REG(ELFIN_GPIO_BASE + GPAPUD_OFFSET)
#define GPACONSLP_REG		__REG(ELFIN_GPIO_BASE + GPACONSLP_OFFSET)
#define GPAPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPAPUDSLP_OFFSET)
#define GPBCON_REG		__REG(ELFIN_GPIO_BASE + GPBCON_OFFSET)
#define GPBDAT_REG		__REG(ELFIN_GPIO_BASE + GPBDAT_OFFSET)
#define GPBPUD_REG		__REG(ELFIN_GPIO_BASE + GPBPUD_OFFSET)
#define GPBCONSLP_REG		__REG(ELFIN_GPIO_BASE + GPBCONSLP_OFFSET)
#define GPBPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPBPUDSLP_OFFSET)
#define GPCCON_REG		__REG(ELFIN_GPIO_BASE + GPCCON_OFFSET)
#define GPCDAT_REG		__REG(ELFIN_GPIO_BASE + GPCDAT_OFFSET)
#define GPCPUD_REG		__REG(ELFIN_GPIO_BASE + GPCPUD_OFFSET)
#define GPCCONSLP_REG		__REG(ELFIN_GPIO_BASE + GPCCONSLP_OFFSET)
#define GPCPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPCPUDSLP_OFFSET)
#define GPDCON_REG		__REG(ELFIN_GPIO_BASE + GPDCON_OFFSET)
#define GPDDAT_REG		__REG(ELFIN_GPIO_BASE + GPDDAT_OFFSET)
#define GPDPUD_REG		__REG(ELFIN_GPIO_BASE + GPDPUD_OFFSET)
#define GPDCONSLP_REG		__REG(ELFIN_GPIO_BASE + GPDCONSLP_OFFSET)
#define GPDPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPDPUDSLP_OFFSET)
#define GPECON_REG		__REG(ELFIN_GPIO_BASE + GPECON_OFFSET)
#define GPEDAT_REG		__REG(ELFIN_GPIO_BASE + GPEDAT_OFFSET)
#define GPEPUD_REG		__REG(ELFIN_GPIO_BASE + GPEPUD_OFFSET)
#define GPECONSLP_REG		__REG(ELFIN_GPIO_BASE + GPECONSLP_OFFSET)
#define GPEPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPEPUDSLP_OFFSET)
#define GPFCON_REG		__REG(ELFIN_GPIO_BASE + GPFCON_OFFSET)
#define GPFDAT_REG		__REG(ELFIN_GPIO_BASE + GPFDAT_OFFSET)
#define GPFPUD_REG		__REG(ELFIN_GPIO_BASE + GPFPUD_OFFSET)
#define GPFCONSLP_REG		__REG(ELFIN_GPIO_BASE + GPFCONSLP_OFFSET)
#define GPFPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPFPUDSLP_OFFSET)
#define GPGCON_REG		__REG(ELFIN_GPIO_BASE + GPGCON_OFFSET)
#define GPGDAT_REG		__REG(ELFIN_GPIO_BASE + GPGDAT_OFFSET)
#define GPGPUD_REG		__REG(ELFIN_GPIO_BASE + GPGPUD_OFFSET)
#define GPGCONSLP_REG		__REG(ELFIN_GPIO_BASE + GPGCONSLP_OFFSET)
#define GPGPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPGPUDSLP_OFFSET)
#define GPHCON0_REG		__REG(ELFIN_GPIO_BASE + GPHCON0_OFFSET)
#define GPHCON1_REG		__REG(ELFIN_GPIO_BASE + GPHCON1_OFFSET)
#define GPHDAT_REG		__REG(ELFIN_GPIO_BASE + GPHDAT_OFFSET)
#define GPHPUD_REG		__REG(ELFIN_GPIO_BASE + GPHPUD_OFFSET)
#define GPHCONSLP_REG		__REG(ELFIN_GPIO_BASE + GPHCONSLP_OFFSET)
#define GPHPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPHPUDSLP_OFFSET)
#define GPICON_REG		__REG(ELFIN_GPIO_BASE + GPICON_OFFSET)
#define GPIDAT_REG		__REG(ELFIN_GPIO_BASE + GPIDAT_OFFSET)
#define GPIPUD_REG		__REG(ELFIN_GPIO_BASE + GPIPUD_OFFSET)
#define GPICONSLP_REG		__REG(ELFIN_GPIO_BASE + GPICONSLP_OFFSET)
#define GPIPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPIPUDSLP_OFFSET)
#define GPJCON_REG		__REG(ELFIN_GPIO_BASE + GPJCON_OFFSET)
#define GPJDAT_REG		__REG(ELFIN_GPIO_BASE + GPJDAT_OFFSET)
#define GPJPUD_REG		__REG(ELFIN_GPIO_BASE + GPJPUD_OFFSET)
#define GPJCONSLP_REG		__REG(ELFIN_GPIO_BASE + GPJCONSLP_OFFSET)
#define GPJPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPJPUDSLP_OFFSET)
#define GPKCON0_REG		__REG(ELFIN_GPIO_BASE + GPKCON0_OFFSET)
#define GPKCON1_REG		__REG(ELFIN_GPIO_BASE + GPKCON1_OFFSET)
#define GPKDAT_REG		__REG(ELFIN_GPIO_BASE + GPKDAT_OFFSET)
#define GPKPUD_REG		__REG(ELFIN_GPIO_BASE + GPKPUD_OFFSET)
#define GPLCON0_REG		__REG(ELFIN_GPIO_BASE + GPLCON0_OFFSET)
#define GPLCON1_REG		__REG(ELFIN_GPIO_BASE + GPLCON1_OFFSET)
#define GPLDAT_REG		__REG(ELFIN_GPIO_BASE + GPLDAT_OFFSET)
#define GPLPUD_REG		__REG(ELFIN_GPIO_BASE + GPLPUD_OFFSET)
#define GPMCON_REG		__REG(ELFIN_GPIO_BASE + GPMCON_OFFSET)
#define GPMDAT_REG		__REG(ELFIN_GPIO_BASE + GPMDAT_OFFSET)
#define GPMPUD_REG		__REG(ELFIN_GPIO_BASE + GPMPUD_OFFSET)
#define GPNCON_REG		__REG(ELFIN_GPIO_BASE + GPNCON_OFFSET)
#define GPNDAT_REG		__REG(ELFIN_GPIO_BASE + GPNDAT_OFFSET)
#define GPNPUD_REG		__REG(ELFIN_GPIO_BASE + GPNPUD_OFFSET)
#define GPOCON_REG		__REG(ELFIN_GPIO_BASE + GPOCON_OFFSET)
#define GPODAT_REG		__REG(ELFIN_GPIO_BASE + GPODAT_OFFSET)
#define GPOPUD_REG		__REG(ELFIN_GPIO_BASE + GPOPUD_OFFSET)
#define GPOCONSLP_REG		__REG(ELFIN_GPIO_BASE + GPOCONSLP_OFFSET)
#define GPOPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPOPUDSLP_OFFSET)
#define GPPCON_REG		__REG(ELFIN_GPIO_BASE + GPPCON_OFFSET)
#define GPPDAT_REG		__REG(ELFIN_GPIO_BASE + GPPDAT_OFFSET)
#define GPPPUD_REG		__REG(ELFIN_GPIO_BASE + GPPPUD_OFFSET)
#define GPPCONSLP_REG		__REG(ELFIN_GPIO_BASE + GPPCONSLP_OFFSET)
#define GPPPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPPPUDSLP_OFFSET)
#define GPQCON_REG		__REG(ELFIN_GPIO_BASE + GPQCON_OFFSET)
#define GPQDAT_REG		__REG(ELFIN_GPIO_BASE + GPQDAT_OFFSET)
#define GPQPUD_REG		__REG(ELFIN_GPIO_BASE + GPQPUD_OFFSET)
#define GPQCONSLP_REG		__REG(ELFIN_GPIO_BASE + GPQCONSLP_OFFSET)
#define GPQPUDSLP_REG		__REG(ELFIN_GPIO_BASE + GPQPUDSLP_OFFSET)

/*
 * Bus Matrix
 */
#define ELFIN_MEM_SYS_CFG	0x7e00f120

#define S3C64XX_MEM_SYS_CFG_16BIT	(1 << 12)

#define S3C64XX_MEM_SYS_CFG_NAND	0x0008
#define S3C64XX_MEM_SYS_CFG_ONENAND	S3C64XX_MEM_SYS_CFG_16BIT

#define GPACON		(ELFIN_GPIO_BASE + GPACON_OFFSET)
#define GPADAT		(ELFIN_GPIO_BASE + GPADAT_OFFSET)
#define GPAPUD		(ELFIN_GPIO_BASE + GPAPUD_OFFSET)
#define GPACONSLP	(ELFIN_GPIO_BASE + GPACONSLP_OFFSET)
#define GPAPUDSLP	(ELFIN_GPIO_BASE + GPAPUDSLP_OFFSET)
#define GPBCON		(ELFIN_GPIO_BASE + GPBCON_OFFSET)
#define GPBDAT		(ELFIN_GPIO_BASE + GPBDAT_OFFSET)
#define GPBPUD		(ELFIN_GPIO_BASE + GPBPUD_OFFSET)
#define GPBCONSLP	(ELFIN_GPIO_BASE + GPBCONSLP_OFFSET)
#define GPBPUDSLP	(ELFIN_GPIO_BASE + GPBPUDSLP_OFFSET)
#define GPCCON		(ELFIN_GPIO_BASE + GPCCON_OFFSET)
#define GPCDAT		(ELFIN_GPIO_BASE + GPCDAT_OFFSET)
#define GPCPUD		(ELFIN_GPIO_BASE + GPCPUD_OFFSET)
#define GPCCONSLP	(ELFIN_GPIO_BASE + GPCCONSLP_OFFSET)
#define GPCPUDSLP	(ELFIN_GPIO_BASE + GPCPUDSLP_OFFSET)
#define GPDCON		(ELFIN_GPIO_BASE + GPDCON_OFFSET)
#define GPDDAT		(ELFIN_GPIO_BASE + GPDDAT_OFFSET)
#define GPDPUD		(ELFIN_GPIO_BASE + GPDPUD_OFFSET)
#define GPDCONSLP	(ELFIN_GPIO_BASE + GPDCONSLP_OFFSET)
#define GPDPUDSLP	(ELFIN_GPIO_BASE + GPDPUDSLP_OFFSET)
#define GPECON		(ELFIN_GPIO_BASE + GPECON_OFFSET)
#define GPEDAT		(ELFIN_GPIO_BASE + GPEDAT_OFFSET)
#define GPEPUD		(ELFIN_GPIO_BASE + GPEPUD_OFFSET)
#define GPECONSLP	(ELFIN_GPIO_BASE + GPECONSLP_OFFSET)
#define GPEPUDSLP	(ELFIN_GPIO_BASE + GPEPUDSLP_OFFSET)
#define GPFCON		(ELFIN_GPIO_BASE + GPFCON_OFFSET)
#define GPFDAT		(ELFIN_GPIO_BASE + GPFDAT_OFFSET)
#define GPFPUD		(ELFIN_GPIO_BASE + GPFPUD_OFFSET)
#define GPFCONSLP	(ELFIN_GPIO_BASE + GPFCONSLP_OFFSET)
#define GPFPUDSLP	(ELFIN_GPIO_BASE + GPFPUDSLP_OFFSET)
#define GPGCON		(ELFIN_GPIO_BASE + GPGCON_OFFSET)
#define GPGDAT		(ELFIN_GPIO_BASE + GPGDAT_OFFSET)
#define GPGPUD		(ELFIN_GPIO_BASE + GPGPUD_OFFSET)
#define GPGCONSLP	(ELFIN_GPIO_BASE + GPGCONSLP_OFFSET)
#define GPGPUDSLP	(ELFIN_GPIO_BASE + GPGPUDSLP_OFFSET)
#define GPHCON0		(ELFIN_GPIO_BASE + GPHCON0_OFFSET)
#define GPHCON1		(ELFIN_GPIO_BASE + GPHCON1_OFFSET)
#define GPHDAT		(ELFIN_GPIO_BASE + GPHDAT_OFFSET)
#define GPHPUD		(ELFIN_GPIO_BASE + GPHPUD_OFFSET)
#define GPHCONSLP	(ELFIN_GPIO_BASE + GPHCONSLP_OFFSET)
#define GPHPUDSLP	(ELFIN_GPIO_BASE + GPHPUDSLP_OFFSET)
#define GPICON		(ELFIN_GPIO_BASE + GPICON_OFFSET)
#define GPIDAT		(ELFIN_GPIO_BASE + GPIDAT_OFFSET)
#define GPIPUD		(ELFIN_GPIO_BASE + GPIPUD_OFFSET)
#define GPICONSLP	(ELFIN_GPIO_BASE + GPICONSLP_OFFSET)
#define GPIPUDSLP	(ELFIN_GPIO_BASE + GPIPUDSLP_OFFSET)
#define GPJCON		(ELFIN_GPIO_BASE + GPJCON_OFFSET)
#define GPJDAT		(ELFIN_GPIO_BASE + GPJDAT_OFFSET)
#define GPJPUD		(ELFIN_GPIO_BASE + GPJPUD_OFFSET)
#define GPJCONSLP	(ELFIN_GPIO_BASE + GPJCONSLP_OFFSET)
#define GPJPUDSLP	(ELFIN_GPIO_BASE + GPJPUDSLP_OFFSET)
#define GPKCON0		(ELFIN_GPIO_BASE + GPKCON0_OFFSET)
#define GPKCON1		(ELFIN_GPIO_BASE + GPKCON1_OFFSET)
#define GPKDAT		(ELFIN_GPIO_BASE + GPKDAT_OFFSET)
#define GPKPUD		(ELFIN_GPIO_BASE + GPKPUD_OFFSET)
#define GPLCON0		(ELFIN_GPIO_BASE + GPLCON0_OFFSET)
#define GPLCON1		(ELFIN_GPIO_BASE + GPLCON1_OFFSET)
#define GPLDAT		(ELFIN_GPIO_BASE + GPLDAT_OFFSET)
#define GPLPUD		(ELFIN_GPIO_BASE + GPLPUD_OFFSET)
#define GPMCON		(ELFIN_GPIO_BASE + GPMCON_OFFSET)
#define GPMDAT		(ELFIN_GPIO_BASE + GPMDAT_OFFSET)
#define GPMPUD		(ELFIN_GPIO_BASE + GPMPUD_OFFSET)
#define GPNCON		(ELFIN_GPIO_BASE + GPNCON_OFFSET)
#define GPNDAT		(ELFIN_GPIO_BASE + GPNDAT_OFFSET)
#define GPNPUD		(ELFIN_GPIO_BASE + GPNPUD_OFFSET)
#define GPOCON		(ELFIN_GPIO_BASE + GPOCON_OFFSET)
#define GPODAT		(ELFIN_GPIO_BASE + GPODAT_OFFSET)
#define GPOPUD		(ELFIN_GPIO_BASE + GPOPUD_OFFSET)
#define GPOCONSLP	(ELFIN_GPIO_BASE + GPOCONSLP_OFFSET)
#define GPOPUDSLP	(ELFIN_GPIO_BASE + GPOPUDSLP_OFFSET)
#define GPPCON		(ELFIN_GPIO_BASE + GPPCON_OFFSET)
#define GPPDAT		(ELFIN_GPIO_BASE + GPPDAT_OFFSET)
#define GPPPUD		(ELFIN_GPIO_BASE + GPPPUD_OFFSET)
#define GPPCONSLP	(ELFIN_GPIO_BASE + GPPCONSLP_OFFSET)
#define GPPPUDSLP	(ELFIN_GPIO_BASE + GPPPUDSLP_OFFSET)
#define GPQCON		(ELFIN_GPIO_BASE + GPQCON_OFFSET)
#define GPQDAT		(ELFIN_GPIO_BASE + GPQDAT_OFFSET)
#define GPQPUD		(ELFIN_GPIO_BASE + GPQPUD_OFFSET)
#define GPQCONSLP	(ELFIN_GPIO_BASE + GPQCONSLP_OFFSET)
#define GPQPUDSLP	(ELFIN_GPIO_BASE + GPQPUDSLP_OFFSET)

/*
 * Memory controller
 */
#define ELFIN_SROM_BASE		0x70000000

#define SROM_BW_REG	__REG(ELFIN_SROM_BASE + 0x0)
#define SROM_BC0_REG	__REG(ELFIN_SROM_BASE + 0x4)
#define SROM_BC1_REG	__REG(ELFIN_SROM_BASE + 0x8)
#define SROM_BC2_REG	__REG(ELFIN_SROM_BASE + 0xC)
#define SROM_BC3_REG	__REG(ELFIN_SROM_BASE + 0x10)
#define SROM_BC4_REG	__REG(ELFIN_SROM_BASE + 0x14)
#define SROM_BC5_REG	__REG(ELFIN_SROM_BASE + 0x18)

/*
 * SDRAM Controller
 */
#define ELFIN_DMC0_BASE		0x7e000000
#define ELFIN_DMC1_BASE		0x7e001000

#define INDEX_DMC_MEMC_STATUS	0x00
#define INDEX_DMC_MEMC_CMD	0x04
#define INDEX_DMC_DIRECT_CMD	0x08
#define INDEX_DMC_MEMORY_CFG	0x0C
#define INDEX_DMC_REFRESH_PRD	0x10
#define INDEX_DMC_CAS_LATENCY	0x14
#define INDEX_DMC_T_DQSS	0x18
#define INDEX_DMC_T_MRD		0x1C
#define INDEX_DMC_T_RAS		0x20
#define INDEX_DMC_T_RC		0x24
#define INDEX_DMC_T_RCD		0x28
#define INDEX_DMC_T_RFC		0x2C
#define INDEX_DMC_T_RP		0x30
#define INDEX_DMC_T_RRD		0x34
#define INDEX_DMC_T_WR		0x38
#define INDEX_DMC_T_WTR		0x3C
#define INDEX_DMC_T_XP		0x40
#define INDEX_DMC_T_XSR		0x44
#define INDEX_DMC_T_ESR		0x48
#define INDEX_DMC_MEMORY_CFG2	0x4C
#define INDEX_DMC_CHIP_0_CFG	0x200
#define INDEX_DMC_CHIP_1_CFG	0x204
#define INDEX_DMC_CHIP_2_CFG	0x208
#define INDEX_DMC_CHIP_3_CFG	0x20C
#define INDEX_DMC_USER_STATUS	0x300
#define INDEX_DMC_USER_CONFIG	0x304

/*
 * Memory Chip direct command
 */
#define DMC_NOP0	0x0c0000
#define DMC_NOP1	0x1c0000
#define DMC_PA0		0x000000	/* Precharge all */
#define DMC_PA1		0x100000
#define DMC_AR0		0x040000	/* Autorefresh */
#define DMC_AR1		0x140000
#define DMC_SDR_MR0	0x080032	/* MRS, CAS 3,  Burst Length 4 */
#define DMC_SDR_MR1	0x180032
#define DMC_DDR_MR0	0x080162
#define DMC_DDR_MR1	0x180162
#define DMC_mDDR_MR0	0x080032	/* CAS 3, Burst Length 4 */
#define DMC_mDDR_MR1	0x180032
#define DMC_mSDR_EMR0	0x0a0000	/* EMRS, DS:Full, PASR:Full Array */
#define DMC_mSDR_EMR1	0x1a0000
#define DMC_DDR_EMR0	0x090000
#define DMC_DDR_EMR1	0x190000
#define DMC_mDDR_EMR0	0x0a0000	/*  DS:Full, PASR:Full Array */
#define DMC_mDDR_EMR1	0x1a0000

/*
 * Definitions for memory configuration
 * Set memory configuration
 *	active_chips	= 1'b0 (1 chip)
 *	qos_master_chip	= 3'b000(ARID[3:0])
 *	memory burst	= 3'b010(burst 4)
 *	stop_mem_clock	= 1'b0(disable dynamical stop)
 *	auto_power_down	= 1'b0(disable auto power-down mode)
 *	power_down_prd	= 6'b00_0000(0 cycle for auto power-down)
 *	ap_bit		= 1'b0 (bit position of auto-precharge is 10)
 *	row_bits	= 3'b010(# row address 13)
 *	column_bits	= 3'b010(# column address 10 )
 *
 * Set user configuration
 *	2'b10=SDRAM/mSDRAM, 2'b11=DDR, 2'b01=mDDR
 *
 * Set chip select for chip [n]
 *	 row bank control, bank address 0x3000_0000 ~ 0x37ff_ffff
 *	 CHIP_[n]_CFG=0x30F8,  30: ADDR[31:24], F8: Mask[31:24]
 */

/*
 * Nand flash controller
 */
#define ELFIN_NAND_BASE		0x70200000

#define NFCONF_OFFSET		0x00
#define NFCONT_OFFSET		0x04
#define NFCMMD_OFFSET		0x08
#define NFADDR_OFFSET		0x0c
#define NFDATA_OFFSET		0x10
#define NFMECCDATA0_OFFSET	0x14
#define NFMECCDATA1_OFFSET	0x18
#define NFSECCDATA0_OFFSET	0x1c
#define NFSBLK_OFFSET		0x20
#define NFEBLK_OFFSET		0x24
#define NFSTAT_OFFSET		0x28
#define NFESTAT0_OFFSET		0x2c
#define NFESTAT1_OFFSET		0x30
#define NFMECC0_OFFSET		0x34
#define NFMECC1_OFFSET		0x38
#define NFSECC_OFFSET		0x3c
#define NFMLCBITPT_OFFSET	0x40

#define NFCONF			(ELFIN_NAND_BASE + NFCONF_OFFSET)
#define NFCONT			(ELFIN_NAND_BASE + NFCONT_OFFSET)
#define NFCMMD			(ELFIN_NAND_BASE + NFCMMD_OFFSET)
#define NFADDR			(ELFIN_NAND_BASE + NFADDR_OFFSET)
#define NFDATA			(ELFIN_NAND_BASE + NFDATA_OFFSET)
#define NFMECCDATA0		(ELFIN_NAND_BASE + NFMECCDATA0_OFFSET)
#define NFMECCDATA1		(ELFIN_NAND_BASE + NFMECCDATA1_OFFSET)
#define NFSECCDATA0		(ELFIN_NAND_BASE + NFSECCDATA0_OFFSET)
#define NFSBLK			(ELFIN_NAND_BASE + NFSBLK_OFFSET)
#define NFEBLK			(ELFIN_NAND_BASE + NFEBLK_OFFSET)
#define NFSTAT			(ELFIN_NAND_BASE + NFSTAT_OFFSET)
#define NFESTAT0		(ELFIN_NAND_BASE + NFESTAT0_OFFSET)
#define NFESTAT1		(ELFIN_NAND_BASE + NFESTAT1_OFFSET)
#define NFMECC0			(ELFIN_NAND_BASE + NFMECC0_OFFSET)
#define NFMECC1			(ELFIN_NAND_BASE + NFMECC1_OFFSET)
#define NFSECC			(ELFIN_NAND_BASE + NFSECC_OFFSET)
#define NFMLCBITPT		(ELFIN_NAND_BASE + NFMLCBITPT_OFFSET)

#define NFCONF_REG		__REG(ELFIN_NAND_BASE + NFCONF_OFFSET)
#define NFCONT_REG		__REG(ELFIN_NAND_BASE + NFCONT_OFFSET)
#define NFCMD_REG		__REG(ELFIN_NAND_BASE + NFCMMD_OFFSET)
#define NFADDR_REG		__REG(ELFIN_NAND_BASE + NFADDR_OFFSET)
#define NFDATA_REG		__REG(ELFIN_NAND_BASE + NFDATA_OFFSET)
#define NFDATA8_REG		__REGb(ELFIN_NAND_BASE + NFDATA_OFFSET)
#define NFMECCDATA0_REG		__REG(ELFIN_NAND_BASE + NFMECCDATA0_OFFSET)
#define NFMECCDATA1_REG		__REG(ELFIN_NAND_BASE + NFMECCDATA1_OFFSET)
#define NFSECCDATA0_REG		__REG(ELFIN_NAND_BASE + NFSECCDATA0_OFFSET)
#define NFSBLK_REG		__REG(ELFIN_NAND_BASE + NFSBLK_OFFSET)
#define NFEBLK_REG		__REG(ELFIN_NAND_BASE + NFEBLK_OFFSET)
#define NFSTAT_REG		__REG(ELFIN_NAND_BASE + NFSTAT_OFFSET)
#define NFESTAT0_REG		__REG(ELFIN_NAND_BASE + NFESTAT0_OFFSET)
#define NFESTAT1_REG		__REG(ELFIN_NAND_BASE + NFESTAT1_OFFSET)
#define NFMECC0_REG		__REG(ELFIN_NAND_BASE + NFMECC0_OFFSET)
#define NFMECC1_REG		__REG(ELFIN_NAND_BASE + NFMECC1_OFFSET)
#define NFSECC_REG		__REG(ELFIN_NAND_BASE + NFSECC_OFFSET)
#define NFMLCBITPT_REG		__REG(ELFIN_NAND_BASE + NFMLCBITPT_OFFSET)

#define NFCONF_ECC_4BIT		(1<<24)

#define NFCONT_ECC_ENC		(1<<18)
#define NFCONT_WP		(1<<16)
#define NFCONT_MECCLOCK		(1<<7)
#define NFCONT_SECCLOCK		(1<<6)
#define NFCONT_INITMECC		(1<<5)
#define NFCONT_INITSECC		(1<<4)
#define NFCONT_INITECC		(NFCONT_INITMECC | NFCONT_INITSECC)
#define NFCONT_CS_ALT		(1<<2)
#define NFCONT_CS		(1<<1)
#define NFCONT_ENABLE		(1<<0)

#define NFSTAT_ECCENCDONE	(1<<7)
#define NFSTAT_ECCDECDONE	(1<<6)
#define NFSTAT_RnB		(1<<0)

#define NFESTAT0_ECCBUSY	(1<<31)

/*
 * Interrupt
 */
#define ELFIN_VIC0_BASE_ADDR	0x71200000
#define ELFIN_VIC1_BASE_ADDR	0x71300000
#define oINTMOD			0x0C	/* VIC INT SELECT (IRQ or FIQ) */
#define oINTUNMSK		0x10	/* VIC INT EN (write 1 to unmask) */
#define oINTMSK			0x14	/* VIC INT EN CLEAR (write 1 to mask) */
#define oINTSUBMSK		0x1C	/* VIC SOFT INT CLEAR */
#define oVECTADDR		0xF00 /* VIC ADDRESS */

/*
 * Watchdog timer
 */
#define ELFIN_WATCHDOG_BASE	0x7E004000

#define WTCON_REG		__REG(0x7E004004)
#define WTDAT_REG		__REG(0x7E004008)
#define WTCNT_REG		__REG(0x7E00400C)


/*
 * UART
 */
#define ELFIN_UART_BASE		0x7F005000

#define ELFIN_UART0_OFFSET	0x0000
#define ELFIN_UART1_OFFSET	0x0400
#define ELFIN_UART2_OFFSET	0x0800

#define ULCON_OFFSET		0x00
#define UCON_OFFSET		0x04
#define UFCON_OFFSET		0x08
#define UMCON_OFFSET		0x0C
#define UTRSTAT_OFFSET		0x10
#define UERSTAT_OFFSET		0x14
#define UFSTAT_OFFSET		0x18
#define UMSTAT_OFFSET		0x1C
#define UTXH_OFFSET		0x20
#define URXH_OFFSET		0x24
#define UBRDIV_OFFSET		0x28
#define UDIVSLOT_OFFSET		0x2C
#define UINTP_OFFSET		0x30
#define UINTSP_OFFSET		0x34
#define UINTM_OFFSET		0x38

#define ULCON0_REG		__REG(0x7F005000)
#define UCON0_REG		__REG(0x7F005004)
#define UFCON0_REG		__REG(0x7F005008)
#define UMCON0_REG		__REG(0x7F00500C)
#define UTRSTAT0_REG		__REG(0x7F005010)
#define UERSTAT0_REG		__REG(0x7F005014)
#define UFSTAT0_REG		__REG(0x7F005018)
#define UMSTAT0_REG		__REG(0x7F00501c)
#define UTXH0_REG		__REG(0x7F005020)
#define URXH0_REG		__REG(0x7F005024)
#define UBRDIV0_REG		__REG(0x7F005028)
#define UDIVSLOT0_REG		__REG(0x7F00502c)
#define UINTP0_REG		__REG(0x7F005030)
#define UINTSP0_REG		__REG(0x7F005034)
#define UINTM0_REG		__REG(0x7F005038)

#define ULCON1_REG		__REG(0x7F005400)
#define UCON1_REG		__REG(0x7F005404)
#define UFCON1_REG		__REG(0x7F005408)
#define UMCON1_REG		__REG(0x7F00540C)
#define UTRSTAT1_REG		__REG(0x7F005410)
#define UERSTAT1_REG		__REG(0x7F005414)
#define UFSTAT1_REG		__REG(0x7F005418)
#define UMSTAT1_REG		__REG(0x7F00541c)
#define UTXH1_REG		__REG(0x7F005420)
#define URXH1_REG		__REG(0x7F005424)
#define UBRDIV1_REG		__REG(0x7F005428)
#define UDIVSLOT1_REG		__REG(0x7F00542c)
#define UINTP1_REG		__REG(0x7F005430)
#define UINTSP1_REG		__REG(0x7F005434)
#define UINTM1_REG		__REG(0x7F005438)

#define UTRSTAT_TX_EMPTY	(1 << 2)
#define UTRSTAT_RX_READY	(1 << 0)
#define UART_ERR_MASK		0xF

/*
 * PWM timer
 */
#define ELFIN_TIMER_BASE	0x7F006000

#define TCFG0_REG	__REG(0x7F006000)
#define TCFG1_REG	__REG(0x7F006004)
#define TCON_REG	__REG(0x7F006008)
#define TCNTB0_REG	__REG(0x7F00600c)
#define TCMPB0_REG	__REG(0x7F006010)
#define TCNTO0_REG	__REG(0x7F006014)
#define TCNTB1_REG	__REG(0x7F006018)
#define TCMPB1_REG	__REG(0x7F00601c)
#define TCNTO1_REG	__REG(0x7F006020)
#define TCNTB2_REG	__REG(0x7F006024)
#define TCMPB2_REG	__REG(0x7F006028)
#define TCNTO2_REG	__REG(0x7F00602c)
#define TCNTB3_REG	__REG(0x7F006030)
#define TCMPB3_REG	__REG(0x7F006034)
#define TCNTO3_REG	__REG(0x7F006038)
#define TCNTB4_REG	__REG(0x7F00603c)
#define TCNTO4_REG	__REG(0x7F006040)

/* Fields */
#define fTCFG0_DZONE		Fld(8, 16) /* the dead zone length (=timer 0) */
#define fTCFG0_PRE1		Fld(8, 8)  /* prescaler value for time 2,3,4 */
#define fTCFG0_PRE0		Fld(8, 0)  /* prescaler value for time 0,1 */
#define fTCFG1_MUX4		Fld(4, 16)
/* bits */
#define TCFG0_DZONE(x)		FInsrt((x), fTCFG0_DZONE)
#define TCFG0_PRE1(x)		FInsrt((x), fTCFG0_PRE1)
#define TCFG0_PRE0(x)		FInsrt((x), fTCFG0_PRE0)
#define TCON_4_AUTO		(1 << 22)  /* auto reload on/off for Timer 4 */
#define TCON_4_UPDATE		(1 << 21)  /* manual Update TCNTB4 */
#define TCON_4_ONOFF		(1 << 20)  /* 0: Stop, 1: start Timer 4 */
#define COUNT_4_ON		(TCON_4_ONOFF * 1)
#define COUNT_4_OFF		(TCON_4_ONOFF * 0)
#define TCON_3_AUTO		(1 << 19)  /* auto reload on/off for Timer 3 */
#define TIMER3_ATLOAD_ON	(TCON_3_AUTO * 1)
#define TIMER3_ATLAOD_OFF	FClrBit(TCON, TCON_3_AUTO)
#define TCON_3_INVERT		(1 << 18)  /* 1: Inverter on for TOUT3 */
#define TIMER3_IVT_ON		(TCON_3_INVERT * 1)
#define TIMER3_IVT_OFF		(FClrBit(TCON, TCON_3_INVERT))
#define TCON_3_MAN		(1 << 17)  /* manual Update TCNTB3,TCMPB3 */
#define TIMER3_MANUP		(TCON_3_MAN*1)
#define TIMER3_NOP		(FClrBit(TCON, TCON_3_MAN))
#define TCON_3_ONOFF		(1 << 16)  /* 0: Stop, 1: start Timer 3 */
#define TIMER3_ON		(TCON_3_ONOFF * 1)
#define TIMER3_OFF		(FClrBit(TCON, TCON_3_ONOFF))

#if defined(CONFIG_CLK_400_100_50)
#define STARTUP_AMDIV		400
#define STARTUP_MDIV		400
#define STARTUP_PDIV		6
#define STARTUP_SDIV		1
#elif defined(CONFIG_CLK_400_133_66)
#define STARTUP_AMDIV		400
#define STARTUP_MDIV		533
#define STARTUP_PDIV		6
#define STARTUP_SDIV		1
#elif defined(CONFIG_CLK_533_133_66)
#define STARTUP_AMDIV		533
#define STARTUP_MDIV		533
#define STARTUP_PDIV		6
#define STARTUP_SDIV		1
#elif defined(CONFIG_CLK_667_133_66)
#define STARTUP_AMDIV		667
#define STARTUP_MDIV		533
#define STARTUP_PDIV		6
#define STARTUP_SDIV		1
#endif

#define	STARTUP_PCLKDIV		3
#define STARTUP_HCLKX2DIV	1
#define STARTUP_HCLKDIV		1
#define STARTUP_MPLLDIV		1
#define STARTUP_APLLDIV		0

#define CLK_DIV_VAL	((STARTUP_PCLKDIV << 12) | (STARTUP_HCLKX2DIV << 9) | \
	(STARTUP_HCLKDIV << 8) | (STARTUP_MPLLDIV<<4) | STARTUP_APLLDIV)
#define MPLL_VAL	((1 << 31) | (STARTUP_MDIV << 16) | \
	(STARTUP_PDIV << 8) | STARTUP_SDIV)
#define STARTUP_MPLL	(((CONFIG_SYS_CLK_FREQ >> STARTUP_SDIV) / \
	STARTUP_PDIV) * STARTUP_MDIV)

#if defined(CONFIG_SYNC_MODE)
#define APLL_VAL	((1 << 31) | (STARTUP_MDIV << 16) | \
	(STARTUP_PDIV << 8) | STARTUP_SDIV)
#define STARTUP_APLL	(((CONFIG_SYS_CLK_FREQ >> STARTUP_SDIV) / \
	STARTUP_PDIV) * STARTUP_MDIV)
#define STARTUP_HCLK	(STARTUP_MPLL / (STARTUP_HCLKX2DIV + 1) / \
	(STARTUP_HCLKDIV + 1))
#else
#define APLL_VAL	((1 << 31) | (STARTUP_AMDIV << 16) | \
	(STARTUP_PDIV << 8) | STARTUP_SDIV)
#define STARTUP_APLL	(((CONFIG_SYS_CLK_FREQ >> STARTUP_SDIV) / \
	STARTUP_PDIV) * STARTUP_AMDIV)
#define STARTUP_HCLK	(STARTUP_MPLL / (STARTUP_HCLKX2DIV + 1) / \
	(STARTUP_HCLKDIV + 1))
#endif


/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define DMC1_MEM_CFG	0x80010012	/* Chip1, Burst4, Row/Column bit */
#define DMC1_MEM_CFG2	0xB45
#define DMC1_CHIP0_CFG	0x150F8		/* 0x4000_0000 ~ 0x43ff_ffff (64MB) */
#define DMC_DDR_32_CFG	0x0 		/* 32bit, DDR */

/* Memory Parameters */
/* DDR Parameters */
#define DDR_tREFRESH		7800	/* ns */
#define DDR_tRAS		45	/* ns (min: 45ns)*/
#define DDR_tRC 		68	/* ns (min: 67.5ns)*/
#define DDR_tRCD		23	/* ns (min: 22.5ns)*/
#define DDR_tRFC		80	/* ns (min: 80ns)*/
#define DDR_tRP 		23	/* ns (min: 22.5ns)*/
#define DDR_tRRD		15	/* ns (min: 15ns)*/
#define DDR_tWR 		15	/* ns (min: 15ns)*/
#define DDR_tXSR		120	/* ns (min: 120ns)*/
#define DDR_CASL		3	/* CAS Latency 3 */

/*
 * mDDR memory configuration
 */

#define NS_TO_CLK(t)		((STARTUP_HCLK / 1000 * (t) - 1) / 1000000)

#define DMC_DDR_BA_EMRS 	2
#define DMC_DDR_MEM_CASLAT	3
/* 6   Set Cas Latency to 3 */
#define DMC_DDR_CAS_LATENCY	(DDR_CASL << 1)
/* Min 0.75 ~ 1.25 */
#define DMC_DDR_t_DQSS		1
/* Min 2 tck */
#define DMC_DDR_t_MRD		2
/* 7, Min 45ns */
#define DMC_DDR_t_RAS		(NS_TO_CLK(DDR_tRAS) + 1)
/* 10, Min 67.5ns */
#define DMC_DDR_t_RC		(NS_TO_CLK(DDR_tRC) + 1)
/* 4,5(TRM), Min 22.5ns */
#define DMC_DDR_t_RCD		(NS_TO_CLK(DDR_tRCD) + 1)
#define DMC_DDR_schedule_RCD	((DMC_DDR_t_RCD - 3) << 3)
/* 11,18(TRM) Min 80ns */
#define DMC_DDR_t_RFC		(NS_TO_CLK(DDR_tRFC) + 1)
#define DMC_DDR_schedule_RFC	((DMC_DDR_t_RFC - 3) << 5)
/* 4, 5(TRM) Min 22.5ns */
#define DMC_DDR_t_RP		(NS_TO_CLK(DDR_tRP) + 1)
#define DMC_DDR_schedule_RP	((DMC_DDR_t_RP - 3) << 3)
/* 3, Min 15ns */
#define DMC_DDR_t_RRD		(NS_TO_CLK(DDR_tRRD) + 1)
/* Min 15ns */
#define DMC_DDR_t_WR		(NS_TO_CLK(DDR_tWR) + 1)
#define DMC_DDR_t_WTR		2
/* 1tck + tIS(1.5ns) */
#define DMC_DDR_t_XP		2
/* 17, Min 120ns */
#define DMC_DDR_t_XSR		(NS_TO_CLK(DDR_tXSR) + 1)
#define DMC_DDR_t_ESR		DMC_DDR_t_XSR
/* TRM 2656 */
#define DMC_DDR_REFRESH_PRD	(NS_TO_CLK(DDR_tREFRESH))
/* 2b01 : mDDR */
#define DMC_DDR_USER_CONFIG	1

#ifndef __ASSEMBLY__
enum s3c64xx_uarts_nr {
	S3C64XX_UART0,
	S3C64XX_UART1,
	S3C64XX_UART2,
};

#include "s3c64x0.h"

static inline s3c64xx_uart *s3c64xx_get_base_uart(enum s3c64xx_uarts_nr nr)
{
	return (s3c64xx_uart *)(ELFIN_UART_BASE + (nr * 0x400));
}
#endif

#endif /*__S3C6400_H__*/
