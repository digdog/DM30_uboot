/*
 * Freescale CLKCTRL Register Definitions
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * This file is created by xml file. Don't Edit it.
 *
 * Xml Revision: 1.48
 * Template revision: 26195
 */

#ifndef __ARCH_ARM___CLKCTRL_H
#define __ARCH_ARM___CLKCTRL_H


#define HW_CLKCTRL_PLL0CTRL0	(0x00000000)
#define HW_CLKCTRL_PLL0CTRL0_SET	(0x00000004)
#define HW_CLKCTRL_PLL0CTRL0_CLR	(0x00000008)
#define HW_CLKCTRL_PLL0CTRL0_TOG	(0x0000000c)

#define BP_CLKCTRL_PLL0CTRL0_RSRVD6	30
#define BM_CLKCTRL_PLL0CTRL0_RSRVD6	0xC0000000
#define BF_CLKCTRL_PLL0CTRL0_RSRVD6(v) \
		(((v) << 30) & BM_CLKCTRL_PLL0CTRL0_RSRVD6)
#define BP_CLKCTRL_PLL0CTRL0_LFR_SEL	28
#define BM_CLKCTRL_PLL0CTRL0_LFR_SEL	0x30000000
#define BF_CLKCTRL_PLL0CTRL0_LFR_SEL(v)  \
		(((v) << 28) & BM_CLKCTRL_PLL0CTRL0_LFR_SEL)
#define BV_CLKCTRL_PLL0CTRL0_LFR_SEL__DEFAULT   0x0
#define BV_CLKCTRL_PLL0CTRL0_LFR_SEL__TIMES_2   0x1
#define BV_CLKCTRL_PLL0CTRL0_LFR_SEL__TIMES_05  0x2
#define BV_CLKCTRL_PLL0CTRL0_LFR_SEL__UNDEFINED 0x3
#define BP_CLKCTRL_PLL0CTRL0_RSRVD5	26
#define BM_CLKCTRL_PLL0CTRL0_RSRVD5	0x0C000000
#define BF_CLKCTRL_PLL0CTRL0_RSRVD5(v)  \
		(((v) << 26) & BM_CLKCTRL_PLL0CTRL0_RSRVD5)
#define BP_CLKCTRL_PLL0CTRL0_CP_SEL	24
#define BM_CLKCTRL_PLL0CTRL0_CP_SEL	0x03000000
#define BF_CLKCTRL_PLL0CTRL0_CP_SEL(v)  \
		(((v) << 24) & BM_CLKCTRL_PLL0CTRL0_CP_SEL)
#define BV_CLKCTRL_PLL0CTRL0_CP_SEL__DEFAULT   0x0
#define BV_CLKCTRL_PLL0CTRL0_CP_SEL__TIMES_2   0x1
#define BV_CLKCTRL_PLL0CTRL0_CP_SEL__TIMES_05  0x2
#define BV_CLKCTRL_PLL0CTRL0_CP_SEL__UNDEFINED 0x3
#define BP_CLKCTRL_PLL0CTRL0_RSRVD4	22
#define BM_CLKCTRL_PLL0CTRL0_RSRVD4	0x00C00000
#define BF_CLKCTRL_PLL0CTRL0_RSRVD4(v)  \
		(((v) << 22) & BM_CLKCTRL_PLL0CTRL0_RSRVD4)
#define BP_CLKCTRL_PLL0CTRL0_DIV_SEL	20
#define BM_CLKCTRL_PLL0CTRL0_DIV_SEL	0x00300000
#define BF_CLKCTRL_PLL0CTRL0_DIV_SEL(v)  \
		(((v) << 20) & BM_CLKCTRL_PLL0CTRL0_DIV_SEL)
#define BV_CLKCTRL_PLL0CTRL0_DIV_SEL__DEFAULT   0x0
#define BV_CLKCTRL_PLL0CTRL0_DIV_SEL__LOWER     0x1
#define BV_CLKCTRL_PLL0CTRL0_DIV_SEL__LOWEST    0x2
#define BV_CLKCTRL_PLL0CTRL0_DIV_SEL__UNDEFINED 0x3
#define BM_CLKCTRL_PLL0CTRL0_RSRVD3	0x00080000
#define BM_CLKCTRL_PLL0CTRL0_EN_USB_CLKS	0x00040000
#define BM_CLKCTRL_PLL0CTRL0_POWER	0x00020000
#define BP_CLKCTRL_PLL0CTRL0_RSRVD1	0
#define BM_CLKCTRL_PLL0CTRL0_RSRVD1	0x0001FFFF
#define BF_CLKCTRL_PLL0CTRL0_RSRVD1(v)  \
		(((v) << 0) & BM_CLKCTRL_PLL0CTRL0_RSRVD1)

#define HW_CLKCTRL_PLL0CTRL1	(0x00000010)

#define BM_CLKCTRL_PLL0CTRL1_LOCK	0x80000000
#define BM_CLKCTRL_PLL0CTRL1_FORCE_LOCK	0x40000000
#define BP_CLKCTRL_PLL0CTRL1_RSRVD1	16
#define BM_CLKCTRL_PLL0CTRL1_RSRVD1	0x3FFF0000
#define BF_CLKCTRL_PLL0CTRL1_RSRVD1(v)  \
		(((v) << 16) & BM_CLKCTRL_PLL0CTRL1_RSRVD1)
#define BP_CLKCTRL_PLL0CTRL1_LOCK_COUNT	0
#define BM_CLKCTRL_PLL0CTRL1_LOCK_COUNT	0x0000FFFF
#define BF_CLKCTRL_PLL0CTRL1_LOCK_COUNT(v)  \
		(((v) << 0) & BM_CLKCTRL_PLL0CTRL1_LOCK_COUNT)

#define HW_CLKCTRL_PLL1CTRL0	(0x00000020)
#define HW_CLKCTRL_PLL1CTRL0_SET	(0x00000024)
#define HW_CLKCTRL_PLL1CTRL0_CLR	(0x00000028)
#define HW_CLKCTRL_PLL1CTRL0_TOG	(0x0000002c)

#define BM_CLKCTRL_PLL1CTRL0_CLKGATEEMI	0x80000000
#define BM_CLKCTRL_PLL1CTRL0_RSRVD6	0x40000000
#define BP_CLKCTRL_PLL1CTRL0_LFR_SEL	28
#define BM_CLKCTRL_PLL1CTRL0_LFR_SEL	0x30000000
#define BF_CLKCTRL_PLL1CTRL0_LFR_SEL(v)  \
		(((v) << 28) & BM_CLKCTRL_PLL1CTRL0_LFR_SEL)
#define BV_CLKCTRL_PLL1CTRL0_LFR_SEL__DEFAULT   0x0
#define BV_CLKCTRL_PLL1CTRL0_LFR_SEL__TIMES_2   0x1
#define BV_CLKCTRL_PLL1CTRL0_LFR_SEL__TIMES_05  0x2
#define BV_CLKCTRL_PLL1CTRL0_LFR_SEL__UNDEFINED 0x3
#define BP_CLKCTRL_PLL1CTRL0_RSRVD5	26
#define BM_CLKCTRL_PLL1CTRL0_RSRVD5	0x0C000000
#define BF_CLKCTRL_PLL1CTRL0_RSRVD5(v)  \
		(((v) << 26) & BM_CLKCTRL_PLL1CTRL0_RSRVD5)
#define BP_CLKCTRL_PLL1CTRL0_CP_SEL	24
#define BM_CLKCTRL_PLL1CTRL0_CP_SEL	0x03000000
#define BF_CLKCTRL_PLL1CTRL0_CP_SEL(v)  \
		(((v) << 24) & BM_CLKCTRL_PLL1CTRL0_CP_SEL)
#define BV_CLKCTRL_PLL1CTRL0_CP_SEL__DEFAULT   0x0
#define BV_CLKCTRL_PLL1CTRL0_CP_SEL__TIMES_2   0x1
#define BV_CLKCTRL_PLL1CTRL0_CP_SEL__TIMES_05  0x2
#define BV_CLKCTRL_PLL1CTRL0_CP_SEL__UNDEFINED 0x3
#define BP_CLKCTRL_PLL1CTRL0_RSRVD4	22
#define BM_CLKCTRL_PLL1CTRL0_RSRVD4	0x00C00000
#define BF_CLKCTRL_PLL1CTRL0_RSRVD4(v)  \
		(((v) << 22) & BM_CLKCTRL_PLL1CTRL0_RSRVD4)
#define BP_CLKCTRL_PLL1CTRL0_DIV_SEL	20
#define BM_CLKCTRL_PLL1CTRL0_DIV_SEL	0x00300000
#define BF_CLKCTRL_PLL1CTRL0_DIV_SEL(v)  \
		(((v) << 20) & BM_CLKCTRL_PLL1CTRL0_DIV_SEL)
#define BV_CLKCTRL_PLL1CTRL0_DIV_SEL__DEFAULT   0x0
#define BV_CLKCTRL_PLL1CTRL0_DIV_SEL__LOWER     0x1
#define BV_CLKCTRL_PLL1CTRL0_DIV_SEL__LOWEST    0x2
#define BV_CLKCTRL_PLL1CTRL0_DIV_SEL__UNDEFINED 0x3
#define BM_CLKCTRL_PLL1CTRL0_RSRVD3	0x00080000
#define BM_CLKCTRL_PLL1CTRL0_EN_USB_CLKS	0x00040000
#define BM_CLKCTRL_PLL1CTRL0_POWER	0x00020000
#define BP_CLKCTRL_PLL1CTRL0_RSRVD1	0
#define BM_CLKCTRL_PLL1CTRL0_RSRVD1	0x0001FFFF
#define BF_CLKCTRL_PLL1CTRL0_RSRVD1(v)  \
		(((v) << 0) & BM_CLKCTRL_PLL1CTRL0_RSRVD1)

#define HW_CLKCTRL_PLL1CTRL1	(0x00000030)

#define BM_CLKCTRL_PLL1CTRL1_LOCK	0x80000000
#define BM_CLKCTRL_PLL1CTRL1_FORCE_LOCK	0x40000000
#define BP_CLKCTRL_PLL1CTRL1_RSRVD1	16
#define BM_CLKCTRL_PLL1CTRL1_RSRVD1	0x3FFF0000
#define BF_CLKCTRL_PLL1CTRL1_RSRVD1(v)  \
		(((v) << 16) & BM_CLKCTRL_PLL1CTRL1_RSRVD1)
#define BP_CLKCTRL_PLL1CTRL1_LOCK_COUNT	0
#define BM_CLKCTRL_PLL1CTRL1_LOCK_COUNT	0x0000FFFF
#define BF_CLKCTRL_PLL1CTRL1_LOCK_COUNT(v)  \
		(((v) << 0) & BM_CLKCTRL_PLL1CTRL1_LOCK_COUNT)

#define HW_CLKCTRL_PLL2CTRL0	(0x00000040)
#define HW_CLKCTRL_PLL2CTRL0_SET	(0x00000044)
#define HW_CLKCTRL_PLL2CTRL0_CLR	(0x00000048)
#define HW_CLKCTRL_PLL2CTRL0_TOG	(0x0000004c)

#define BM_CLKCTRL_PLL2CTRL0_CLKGATE	0x80000000
#define BM_CLKCTRL_PLL2CTRL0_RSRVD3	0x40000000
#define BP_CLKCTRL_PLL2CTRL0_LFR_SEL	28
#define BM_CLKCTRL_PLL2CTRL0_LFR_SEL	0x30000000
#define BF_CLKCTRL_PLL2CTRL0_LFR_SEL(v)  \
		(((v) << 28) & BM_CLKCTRL_PLL2CTRL0_LFR_SEL)
#define BM_CLKCTRL_PLL2CTRL0_RSRVD2	0x08000000
#define BM_CLKCTRL_PLL2CTRL0_HOLD_RING_OFF_B	0x04000000
#define BP_CLKCTRL_PLL2CTRL0_CP_SEL	24
#define BM_CLKCTRL_PLL2CTRL0_CP_SEL	0x03000000
#define BF_CLKCTRL_PLL2CTRL0_CP_SEL(v)  \
		(((v) << 24) & BM_CLKCTRL_PLL2CTRL0_CP_SEL)
#define BM_CLKCTRL_PLL2CTRL0_POWER	0x00800000
#define BP_CLKCTRL_PLL2CTRL0_RSRVD1	0
#define BM_CLKCTRL_PLL2CTRL0_RSRVD1	0x007FFFFF
#define BF_CLKCTRL_PLL2CTRL0_RSRVD1(v)  \
		(((v) << 0) & BM_CLKCTRL_PLL2CTRL0_RSRVD1)

#define HW_CLKCTRL_CPU	(0x00000050)
#define HW_CLKCTRL_CPU_SET	(0x00000054)
#define HW_CLKCTRL_CPU_CLR	(0x00000058)
#define HW_CLKCTRL_CPU_TOG	(0x0000005c)

#define BP_CLKCTRL_CPU_RSRVD5	30
#define BM_CLKCTRL_CPU_RSRVD5	0xC0000000
#define BF_CLKCTRL_CPU_RSRVD5(v) \
		(((v) << 30) & BM_CLKCTRL_CPU_RSRVD5)
#define BM_CLKCTRL_CPU_BUSY_REF_XTAL	0x20000000
#define BM_CLKCTRL_CPU_BUSY_REF_CPU	0x10000000
#define BM_CLKCTRL_CPU_RSRVD4	0x08000000
#define BM_CLKCTRL_CPU_DIV_XTAL_FRAC_EN	0x04000000
#define BP_CLKCTRL_CPU_DIV_XTAL	16
#define BM_CLKCTRL_CPU_DIV_XTAL	0x03FF0000
#define BF_CLKCTRL_CPU_DIV_XTAL(v)  \
		(((v) << 16) & BM_CLKCTRL_CPU_DIV_XTAL)
#define BP_CLKCTRL_CPU_RSRVD3	13
#define BM_CLKCTRL_CPU_RSRVD3	0x0000E000
#define BF_CLKCTRL_CPU_RSRVD3(v)  \
		(((v) << 13) & BM_CLKCTRL_CPU_RSRVD3)
#define BM_CLKCTRL_CPU_INTERRUPT_WAIT	0x00001000
#define BM_CLKCTRL_CPU_RSRVD2	0x00000800
#define BM_CLKCTRL_CPU_DIV_CPU_FRAC_EN	0x00000400
#define BP_CLKCTRL_CPU_RSRVD1	6
#define BM_CLKCTRL_CPU_RSRVD1	0x000003C0
#define BF_CLKCTRL_CPU_RSRVD1(v)  \
		(((v) << 6) & BM_CLKCTRL_CPU_RSRVD1)
#define BP_CLKCTRL_CPU_DIV_CPU	0
#define BM_CLKCTRL_CPU_DIV_CPU	0x0000003F
#define BF_CLKCTRL_CPU_DIV_CPU(v)  \
		(((v) << 0) & BM_CLKCTRL_CPU_DIV_CPU)

#define HW_CLKCTRL_HBUS	(0x00000060)
#define HW_CLKCTRL_HBUS_SET	(0x00000064)
#define HW_CLKCTRL_HBUS_CLR	(0x00000068)
#define HW_CLKCTRL_HBUS_TOG	(0x0000006c)

#define BM_CLKCTRL_HBUS_ASM_BUSY	0x80000000
#define BM_CLKCTRL_HBUS_DCP_AS_ENABLE	0x40000000
#define BM_CLKCTRL_HBUS_PXP_AS_ENABLE	0x20000000
#define BM_CLKCTRL_HBUS_RSRVD2	0x10000000
#define BM_CLKCTRL_HBUS_ASM_EMIPORT_AS_ENABLE	0x08000000
#define BM_CLKCTRL_HBUS_APBHDMA_AS_ENABLE	0x04000000
#define BM_CLKCTRL_HBUS_APBXDMA_AS_ENABLE	0x02000000
#define BM_CLKCTRL_HBUS_TRAFFIC_JAM_AS_ENABLE	0x01000000
#define BM_CLKCTRL_HBUS_TRAFFIC_AS_ENABLE	0x00800000
#define BM_CLKCTRL_HBUS_CPU_DATA_AS_ENABLE	0x00400000
#define BM_CLKCTRL_HBUS_CPU_INSTR_AS_ENABLE	0x00200000
#define BM_CLKCTRL_HBUS_ASM_ENABLE	0x00100000
#define BM_CLKCTRL_HBUS_AUTO_CLEAR_DIV_ENABLE	0x00080000
#define BP_CLKCTRL_HBUS_SLOW_DIV	16
#define BM_CLKCTRL_HBUS_SLOW_DIV	0x00070000
#define BF_CLKCTRL_HBUS_SLOW_DIV(v)  \
		(((v) << 16) & BM_CLKCTRL_HBUS_SLOW_DIV)
#define BV_CLKCTRL_HBUS_SLOW_DIV__BY1  0x0
#define BV_CLKCTRL_HBUS_SLOW_DIV__BY2  0x1
#define BV_CLKCTRL_HBUS_SLOW_DIV__BY4  0x2
#define BV_CLKCTRL_HBUS_SLOW_DIV__BY8  0x3
#define BV_CLKCTRL_HBUS_SLOW_DIV__BY16 0x4
#define BV_CLKCTRL_HBUS_SLOW_DIV__BY32 0x5
#define BP_CLKCTRL_HBUS_RSRVD1	6
#define BM_CLKCTRL_HBUS_RSRVD1	0x0000FFC0
#define BF_CLKCTRL_HBUS_RSRVD1(v)  \
		(((v) << 6) & BM_CLKCTRL_HBUS_RSRVD1)
#define BM_CLKCTRL_HBUS_DIV_FRAC_EN	0x00000020
#define BP_CLKCTRL_HBUS_DIV	0
#define BM_CLKCTRL_HBUS_DIV	0x0000001F
#define BF_CLKCTRL_HBUS_DIV(v)  \
		(((v) << 0) & BM_CLKCTRL_HBUS_DIV)

#define HW_CLKCTRL_XBUS	(0x00000070)

#define BM_CLKCTRL_XBUS_BUSY	0x80000000
#define BP_CLKCTRL_XBUS_RSRVD1	12
#define BM_CLKCTRL_XBUS_RSRVD1	0x7FFFF000
#define BF_CLKCTRL_XBUS_RSRVD1(v)  \
		(((v) << 12) & BM_CLKCTRL_XBUS_RSRVD1)
#define BM_CLKCTRL_XBUS_AUTO_CLEAR_DIV_ENABLE	0x00000800
#define BM_CLKCTRL_XBUS_DIV_FRAC_EN	0x00000400
#define BP_CLKCTRL_XBUS_DIV	0
#define BM_CLKCTRL_XBUS_DIV	0x000003FF
#define BF_CLKCTRL_XBUS_DIV(v)  \
		(((v) << 0) & BM_CLKCTRL_XBUS_DIV)

#define HW_CLKCTRL_XTAL	(0x00000080)
#define HW_CLKCTRL_XTAL_SET	(0x00000084)
#define HW_CLKCTRL_XTAL_CLR	(0x00000088)
#define HW_CLKCTRL_XTAL_TOG	(0x0000008c)

#define BM_CLKCTRL_XTAL_UART_CLK_GATE	0x80000000
#define BM_CLKCTRL_XTAL_RSRVD3	0x40000000
#define BM_CLKCTRL_XTAL_PWM_CLK24M_GATE	0x20000000
#define BP_CLKCTRL_XTAL_RSRVD2	27
#define BM_CLKCTRL_XTAL_RSRVD2	0x18000000
#define BF_CLKCTRL_XTAL_RSRVD2(v)  \
		(((v) << 27) & BM_CLKCTRL_XTAL_RSRVD2)
#define BM_CLKCTRL_XTAL_TIMROT_CLK32K_GATE	0x04000000
#define BP_CLKCTRL_XTAL_RSRVD1	2
#define BM_CLKCTRL_XTAL_RSRVD1	0x03FFFFFC
#define BF_CLKCTRL_XTAL_RSRVD1(v)  \
		(((v) << 2) & BM_CLKCTRL_XTAL_RSRVD1)
#define BP_CLKCTRL_XTAL_DIV_UART	0
#define BM_CLKCTRL_XTAL_DIV_UART	0x00000003
#define BF_CLKCTRL_XTAL_DIV_UART(v)  \
		(((v) << 0) & BM_CLKCTRL_XTAL_DIV_UART)

#define HW_CLKCTRL_SSP0	(0x00000090)

#define BM_CLKCTRL_SSP0_CLKGATE	0x80000000
#define BM_CLKCTRL_SSP0_RSRVD2	0x40000000
#define BM_CLKCTRL_SSP0_BUSY	0x20000000
#define BP_CLKCTRL_SSP0_RSRVD1	10
#define BM_CLKCTRL_SSP0_RSRVD1	0x1FFFFC00
#define BF_CLKCTRL_SSP0_RSRVD1(v)  \
		(((v) << 10) & BM_CLKCTRL_SSP0_RSRVD1)
#define BM_CLKCTRL_SSP0_DIV_FRAC_EN	0x00000200
#define BP_CLKCTRL_SSP0_DIV	0
#define BM_CLKCTRL_SSP0_DIV	0x000001FF
#define BF_CLKCTRL_SSP0_DIV(v)  \
		(((v) << 0) & BM_CLKCTRL_SSP0_DIV)

#define HW_CLKCTRL_SSP1	(0x000000a0)

#define BM_CLKCTRL_SSP1_CLKGATE	0x80000000
#define BM_CLKCTRL_SSP1_RSRVD2	0x40000000
#define BM_CLKCTRL_SSP1_BUSY	0x20000000
#define BP_CLKCTRL_SSP1_RSRVD1	10
#define BM_CLKCTRL_SSP1_RSRVD1	0x1FFFFC00
#define BF_CLKCTRL_SSP1_RSRVD1(v)  \
		(((v) << 10) & BM_CLKCTRL_SSP1_RSRVD1)
#define BM_CLKCTRL_SSP1_DIV_FRAC_EN	0x00000200
#define BP_CLKCTRL_SSP1_DIV	0
#define BM_CLKCTRL_SSP1_DIV	0x000001FF
#define BF_CLKCTRL_SSP1_DIV(v)  \
		(((v) << 0) & BM_CLKCTRL_SSP1_DIV)

#define HW_CLKCTRL_SSP2	(0x000000b0)

#define BM_CLKCTRL_SSP2_CLKGATE	0x80000000
#define BM_CLKCTRL_SSP2_RSRVD2	0x40000000
#define BM_CLKCTRL_SSP2_BUSY	0x20000000
#define BP_CLKCTRL_SSP2_RSRVD1	10
#define BM_CLKCTRL_SSP2_RSRVD1	0x1FFFFC00
#define BF_CLKCTRL_SSP2_RSRVD1(v)  \
		(((v) << 10) & BM_CLKCTRL_SSP2_RSRVD1)
#define BM_CLKCTRL_SSP2_DIV_FRAC_EN	0x00000200
#define BP_CLKCTRL_SSP2_DIV	0
#define BM_CLKCTRL_SSP2_DIV	0x000001FF
#define BF_CLKCTRL_SSP2_DIV(v)  \
		(((v) << 0) & BM_CLKCTRL_SSP2_DIV)

#define HW_CLKCTRL_SSP3	(0x000000c0)

#define BM_CLKCTRL_SSP3_CLKGATE	0x80000000
#define BM_CLKCTRL_SSP3_RSRVD2	0x40000000
#define BM_CLKCTRL_SSP3_BUSY	0x20000000
#define BP_CLKCTRL_SSP3_RSRVD1	10
#define BM_CLKCTRL_SSP3_RSRVD1	0x1FFFFC00
#define BF_CLKCTRL_SSP3_RSRVD1(v)  \
		(((v) << 10) & BM_CLKCTRL_SSP3_RSRVD1)
#define BM_CLKCTRL_SSP3_DIV_FRAC_EN	0x00000200
#define BP_CLKCTRL_SSP3_DIV	0
#define BM_CLKCTRL_SSP3_DIV	0x000001FF
#define BF_CLKCTRL_SSP3_DIV(v)  \
		(((v) << 0) & BM_CLKCTRL_SSP3_DIV)

#define HW_CLKCTRL_GPMI	(0x000000d0)

#define BM_CLKCTRL_GPMI_CLKGATE	0x80000000
#define BM_CLKCTRL_GPMI_RSRVD2	0x40000000
#define BM_CLKCTRL_GPMI_BUSY	0x20000000
#define BP_CLKCTRL_GPMI_RSRVD1	11
#define BM_CLKCTRL_GPMI_RSRVD1	0x1FFFF800
#define BF_CLKCTRL_GPMI_RSRVD1(v)  \
		(((v) << 11) & BM_CLKCTRL_GPMI_RSRVD1)
#define BM_CLKCTRL_GPMI_DIV_FRAC_EN	0x00000400
#define BP_CLKCTRL_GPMI_DIV	0
#define BM_CLKCTRL_GPMI_DIV	0x000003FF
#define BF_CLKCTRL_GPMI_DIV(v)  \
		(((v) << 0) & BM_CLKCTRL_GPMI_DIV)

#define HW_CLKCTRL_SPDIF	(0x000000e0)

#define BM_CLKCTRL_SPDIF_CLKGATE	0x80000000
#define BP_CLKCTRL_SPDIF_RSRVD	0
#define BM_CLKCTRL_SPDIF_RSRVD	0x7FFFFFFF
#define BF_CLKCTRL_SPDIF_RSRVD(v)  \
		(((v) << 0) & BM_CLKCTRL_SPDIF_RSRVD)

#define HW_CLKCTRL_EMI	(0x000000f0)

#define BM_CLKCTRL_EMI_CLKGATE	0x80000000
#define BM_CLKCTRL_EMI_SYNC_MODE_EN	0x40000000
#define BM_CLKCTRL_EMI_BUSY_REF_XTAL	0x20000000
#define BM_CLKCTRL_EMI_BUSY_REF_EMI	0x10000000
#define BM_CLKCTRL_EMI_BUSY_REF_CPU	0x08000000
#define BM_CLKCTRL_EMI_BUSY_SYNC_MODE	0x04000000
#define BP_CLKCTRL_EMI_RSRVD3	18
#define BM_CLKCTRL_EMI_RSRVD3	0x03FC0000
#define BF_CLKCTRL_EMI_RSRVD3(v)  \
		(((v) << 18) & BM_CLKCTRL_EMI_RSRVD3)
#define BM_CLKCTRL_EMI_BUSY_DCC_RESYNC	0x00020000
#define BM_CLKCTRL_EMI_DCC_RESYNC_ENABLE	0x00010000
#define BP_CLKCTRL_EMI_RSRVD2	12
#define BM_CLKCTRL_EMI_RSRVD2	0x0000F000
#define BF_CLKCTRL_EMI_RSRVD2(v)  \
		(((v) << 12) & BM_CLKCTRL_EMI_RSRVD2)
#define BP_CLKCTRL_EMI_DIV_XTAL	8
#define BM_CLKCTRL_EMI_DIV_XTAL	0x00000F00
#define BF_CLKCTRL_EMI_DIV_XTAL(v)  \
		(((v) << 8) & BM_CLKCTRL_EMI_DIV_XTAL)
#define BP_CLKCTRL_EMI_RSRVD1	6
#define BM_CLKCTRL_EMI_RSRVD1	0x000000C0
#define BF_CLKCTRL_EMI_RSRVD1(v)  \
		(((v) << 6) & BM_CLKCTRL_EMI_RSRVD1)
#define BP_CLKCTRL_EMI_DIV_EMI	0
#define BM_CLKCTRL_EMI_DIV_EMI	0x0000003F
#define BF_CLKCTRL_EMI_DIV_EMI(v)  \
		(((v) << 0) & BM_CLKCTRL_EMI_DIV_EMI)

#define HW_CLKCTRL_SAIF0	(0x00000100)

#define BM_CLKCTRL_SAIF0_CLKGATE	0x80000000
#define BM_CLKCTRL_SAIF0_RSRVD2	0x40000000
#define BM_CLKCTRL_SAIF0_BUSY	0x20000000
#define BP_CLKCTRL_SAIF0_RSRVD1	17
#define BM_CLKCTRL_SAIF0_RSRVD1	0x1FFE0000
#define BF_CLKCTRL_SAIF0_RSRVD1(v)  \
		(((v) << 17) & BM_CLKCTRL_SAIF0_RSRVD1)
#define BM_CLKCTRL_SAIF0_DIV_FRAC_EN	0x00010000
#define BP_CLKCTRL_SAIF0_DIV	0
#define BM_CLKCTRL_SAIF0_DIV	0x0000FFFF
#define BF_CLKCTRL_SAIF0_DIV(v)  \
		(((v) << 0) & BM_CLKCTRL_SAIF0_DIV)

#define HW_CLKCTRL_SAIF1	(0x00000110)

#define BM_CLKCTRL_SAIF1_CLKGATE	0x80000000
#define BM_CLKCTRL_SAIF1_RSRVD2	0x40000000
#define BM_CLKCTRL_SAIF1_BUSY	0x20000000
#define BP_CLKCTRL_SAIF1_RSRVD1	17
#define BM_CLKCTRL_SAIF1_RSRVD1	0x1FFE0000
#define BF_CLKCTRL_SAIF1_RSRVD1(v)  \
		(((v) << 17) & BM_CLKCTRL_SAIF1_RSRVD1)
#define BM_CLKCTRL_SAIF1_DIV_FRAC_EN	0x00010000
#define BP_CLKCTRL_SAIF1_DIV	0
#define BM_CLKCTRL_SAIF1_DIV	0x0000FFFF
#define BF_CLKCTRL_SAIF1_DIV(v)  \
		(((v) << 0) & BM_CLKCTRL_SAIF1_DIV)

#define HW_CLKCTRL_DIS_LCDIF	(0x00000120)

#define BM_CLKCTRL_DIS_LCDIF_CLKGATE	0x80000000
#define BM_CLKCTRL_DIS_LCDIF_RSRVD2	0x40000000
#define BM_CLKCTRL_DIS_LCDIF_BUSY	0x20000000
#define BP_CLKCTRL_DIS_LCDIF_RSRVD1	14
#define BM_CLKCTRL_DIS_LCDIF_RSRVD1	0x1FFFC000
#define BF_CLKCTRL_DIS_LCDIF_RSRVD1(v)  \
		(((v) << 14) & BM_CLKCTRL_DIS_LCDIF_RSRVD1)
#define BM_CLKCTRL_DIS_LCDIF_DIV_FRAC_EN	0x00002000
#define BP_CLKCTRL_DIS_LCDIF_DIV	0
#define BM_CLKCTRL_DIS_LCDIF_DIV	0x00001FFF
#define BF_CLKCTRL_DIS_LCDIF_DIV(v)  \
		(((v) << 0) & BM_CLKCTRL_DIS_LCDIF_DIV)

#define HW_CLKCTRL_ETM	(0x00000130)

#define BM_CLKCTRL_ETM_CLKGATE	0x80000000
#define BM_CLKCTRL_ETM_RSRVD2	0x40000000
#define BM_CLKCTRL_ETM_BUSY	0x20000000
#define BP_CLKCTRL_ETM_RSRVD1	8
#define BM_CLKCTRL_ETM_RSRVD1	0x1FFFFF00
#define BF_CLKCTRL_ETM_RSRVD1(v)  \
		(((v) << 8) & BM_CLKCTRL_ETM_RSRVD1)
#define BM_CLKCTRL_ETM_DIV_FRAC_EN	0x00000080
#define BP_CLKCTRL_ETM_DIV	0
#define BM_CLKCTRL_ETM_DIV	0x0000007F
#define BF_CLKCTRL_ETM_DIV(v)  \
		(((v) << 0) & BM_CLKCTRL_ETM_DIV)

#define HW_CLKCTRL_ENET	(0x00000140)

#define BM_CLKCTRL_ENET_SLEEP	0x80000000
#define BM_CLKCTRL_ENET_DISABLE	0x40000000
#define BM_CLKCTRL_ENET_STATUS	0x20000000
#define BM_CLKCTRL_ENET_RSRVD1	0x10000000
#define BM_CLKCTRL_ENET_BUSY_TIME	0x08000000
#define BP_CLKCTRL_ENET_DIV_TIME	21
#define BM_CLKCTRL_ENET_DIV_TIME	0x07E00000
#define BF_CLKCTRL_ENET_DIV_TIME(v)  \
		(((v) << 21) & BM_CLKCTRL_ENET_DIV_TIME)
#define BP_CLKCTRL_ENET_TIME_SEL	19
#define BM_CLKCTRL_ENET_TIME_SEL	0x00180000
#define BF_CLKCTRL_ENET_TIME_SEL(v)  \
		(((v) << 19) & BM_CLKCTRL_ENET_TIME_SEL)
#define BV_CLKCTRL_ENET_TIME_SEL__XTAL      0x0
#define BV_CLKCTRL_ENET_TIME_SEL__PLL       0x1
#define BV_CLKCTRL_ENET_TIME_SEL__RMII_CLK  0x2
#define BV_CLKCTRL_ENET_TIME_SEL__UNDEFINED 0x3
#define BM_CLKCTRL_ENET_CLK_OUT_EN	0x00040000
#define BM_CLKCTRL_ENET_RESET_BY_SW_CHIP	0x00020000
#define BM_CLKCTRL_ENET_RESET_BY_SW	0x00010000
#define BP_CLKCTRL_ENET_RSRVD0	0
#define BM_CLKCTRL_ENET_RSRVD0	0x0000FFFF
#define BF_CLKCTRL_ENET_RSRVD0(v)  \
		(((v) << 0) & BM_CLKCTRL_ENET_RSRVD0)

#define HW_CLKCTRL_HSADC	(0x00000150)

#define BM_CLKCTRL_HSADC_RSRVD2	0x80000000
#define BM_CLKCTRL_HSADC_RESETB	0x40000000
#define BP_CLKCTRL_HSADC_FREQDIV	28
#define BM_CLKCTRL_HSADC_FREQDIV	0x30000000
#define BF_CLKCTRL_HSADC_FREQDIV(v)  \
		(((v) << 28) & BM_CLKCTRL_HSADC_FREQDIV)
#define BP_CLKCTRL_HSADC_RSRVD1	0
#define BM_CLKCTRL_HSADC_RSRVD1	0x0FFFFFFF
#define BF_CLKCTRL_HSADC_RSRVD1(v)  \
		(((v) << 0) & BM_CLKCTRL_HSADC_RSRVD1)

#define HW_CLKCTRL_FLEXCAN	(0x00000160)

#define BM_CLKCTRL_FLEXCAN_RSRVD2	0x80000000
#define BM_CLKCTRL_FLEXCAN_STOP_CAN0	0x40000000
#define BM_CLKCTRL_FLEXCAN_CAN0_STATUS	0x20000000
#define BM_CLKCTRL_FLEXCAN_STOP_CAN1	0x10000000
#define BM_CLKCTRL_FLEXCAN_CAN1_STATUS	0x08000000
#define BP_CLKCTRL_FLEXCAN_RSRVD1	0
#define BM_CLKCTRL_FLEXCAN_RSRVD1	0x07FFFFFF
#define BF_CLKCTRL_FLEXCAN_RSRVD1(v)  \
		(((v) << 0) & BM_CLKCTRL_FLEXCAN_RSRVD1)

#define HW_CLKCTRL_FRAC0	(0x000001b0)
#define HW_CLKCTRL_FRAC0_SET	(0x000001b4)
#define HW_CLKCTRL_FRAC0_CLR	(0x000001b8)
#define HW_CLKCTRL_FRAC0_TOG	(0x000001bc)

#define BM_CLKCTRL_FRAC0_CLKGATEIO0	0x80000000
#define BM_CLKCTRL_FRAC0_IO0_STABLE	0x40000000
#define BP_CLKCTRL_FRAC0_IO0FRAC	24
#define BM_CLKCTRL_FRAC0_IO0FRAC	0x3F000000
#define BF_CLKCTRL_FRAC0_IO0FRAC(v)  \
		(((v) << 24) & BM_CLKCTRL_FRAC0_IO0FRAC)
#define BM_CLKCTRL_FRAC0_CLKGATEIO1	0x00800000
#define BM_CLKCTRL_FRAC0_IO1_STABLE	0x00400000
#define BP_CLKCTRL_FRAC0_IO1FRAC	16
#define BM_CLKCTRL_FRAC0_IO1FRAC	0x003F0000
#define BF_CLKCTRL_FRAC0_IO1FRAC(v)  \
		(((v) << 16) & BM_CLKCTRL_FRAC0_IO1FRAC)
#define BM_CLKCTRL_FRAC0_CLKGATEEMI	0x00008000
#define BM_CLKCTRL_FRAC0_EMI_STABLE	0x00004000
#define BP_CLKCTRL_FRAC0_EMIFRAC	8
#define BM_CLKCTRL_FRAC0_EMIFRAC	0x00003F00
#define BF_CLKCTRL_FRAC0_EMIFRAC(v)  \
		(((v) << 8) & BM_CLKCTRL_FRAC0_EMIFRAC)
#define BM_CLKCTRL_FRAC0_CLKGATECPU	0x00000080
#define BM_CLKCTRL_FRAC0_CPU_STABLE	0x00000040
#define BP_CLKCTRL_FRAC0_CPUFRAC	0
#define BM_CLKCTRL_FRAC0_CPUFRAC	0x0000003F
#define BF_CLKCTRL_FRAC0_CPUFRAC(v)  \
		(((v) << 0) & BM_CLKCTRL_FRAC0_CPUFRAC)

#define HW_CLKCTRL_FRAC1	(0x000001c0)
#define HW_CLKCTRL_FRAC1_SET	(0x000001c4)
#define HW_CLKCTRL_FRAC1_CLR	(0x000001c8)
#define HW_CLKCTRL_FRAC1_TOG	(0x000001cc)

#define BP_CLKCTRL_FRAC1_RSRVD2	24
#define BM_CLKCTRL_FRAC1_RSRVD2	0xFF000000
#define BF_CLKCTRL_FRAC1_RSRVD2(v) \
		(((v) << 24) & BM_CLKCTRL_FRAC1_RSRVD2)
#define BM_CLKCTRL_FRAC1_CLKGATEGPMI	0x00800000
#define BM_CLKCTRL_FRAC1_GPMI_STABLE	0x00400000
#define BP_CLKCTRL_FRAC1_GPMIFRAC	16
#define BM_CLKCTRL_FRAC1_GPMIFRAC	0x003F0000
#define BF_CLKCTRL_FRAC1_GPMIFRAC(v)  \
		(((v) << 16) & BM_CLKCTRL_FRAC1_GPMIFRAC)
#define BM_CLKCTRL_FRAC1_CLKGATEHSADC	0x00008000
#define BM_CLKCTRL_FRAC1_HSADC_STABLE	0x00004000
#define BP_CLKCTRL_FRAC1_HSADCFRAC	8
#define BM_CLKCTRL_FRAC1_HSADCFRAC	0x00003F00
#define BF_CLKCTRL_FRAC1_HSADCFRAC(v)  \
		(((v) << 8) & BM_CLKCTRL_FRAC1_HSADCFRAC)
#define BM_CLKCTRL_FRAC1_CLKGATEPIX	0x00000080
#define BM_CLKCTRL_FRAC1_PIX_STABLE	0x00000040
#define BP_CLKCTRL_FRAC1_PIXFRAC	0
#define BM_CLKCTRL_FRAC1_PIXFRAC	0x0000003F
#define BF_CLKCTRL_FRAC1_PIXFRAC(v)  \
		(((v) << 0) & BM_CLKCTRL_FRAC1_PIXFRAC)

#define HW_CLKCTRL_CLKSEQ	(0x000001d0)
#define HW_CLKCTRL_CLKSEQ_SET	(0x000001d4)
#define HW_CLKCTRL_CLKSEQ_CLR	(0x000001d8)
#define HW_CLKCTRL_CLKSEQ_TOG	(0x000001dc)

#define BP_CLKCTRL_CLKSEQ_RSRVD0	19
#define BM_CLKCTRL_CLKSEQ_RSRVD0	0xFFF80000
#define BF_CLKCTRL_CLKSEQ_RSRVD0(v) \
		(((v) << 19) & BM_CLKCTRL_CLKSEQ_RSRVD0)
#define BM_CLKCTRL_CLKSEQ_BYPASS_CPU	0x00040000
#define BP_CLKCTRL_CLKSEQ_RSRVD1	15
#define BM_CLKCTRL_CLKSEQ_RSRVD1	0x00038000
#define BF_CLKCTRL_CLKSEQ_RSRVD1(v)  \
		(((v) << 15) & BM_CLKCTRL_CLKSEQ_RSRVD1)
#define BM_CLKCTRL_CLKSEQ_BYPASS_DIS_LCDIF	0x00004000
#define BV_CLKCTRL_CLKSEQ_BYPASS_DIS_LCDIF__BYPASS 0x1
#define BV_CLKCTRL_CLKSEQ_BYPASS_DIS_LCDIF__PFD    0x0
#define BP_CLKCTRL_CLKSEQ_RSRVD2	9
#define BM_CLKCTRL_CLKSEQ_RSRVD2	0x00003E00
#define BF_CLKCTRL_CLKSEQ_RSRVD2(v)  \
		(((v) << 9) & BM_CLKCTRL_CLKSEQ_RSRVD2)
#define BM_CLKCTRL_CLKSEQ_BYPASS_ETM	0x00000100
#define BM_CLKCTRL_CLKSEQ_BYPASS_EMI	0x00000080
#define BM_CLKCTRL_CLKSEQ_BYPASS_SSP3	0x00000040
#define BM_CLKCTRL_CLKSEQ_BYPASS_SSP2	0x00000020
#define BM_CLKCTRL_CLKSEQ_BYPASS_SSP1	0x00000010
#define BM_CLKCTRL_CLKSEQ_BYPASS_SSP0	0x00000008
#define BM_CLKCTRL_CLKSEQ_BYPASS_GPMI	0x00000004
#define BM_CLKCTRL_CLKSEQ_BYPASS_SAIF1	0x00000002
#define BM_CLKCTRL_CLKSEQ_BYPASS_SAIF0	0x00000001

#define HW_CLKCTRL_RESET	(0x000001e0)

#define BP_CLKCTRL_RESET_RSRVD	6
#define BM_CLKCTRL_RESET_RSRVD	0xFFFFFFC0
#define BF_CLKCTRL_RESET_RSRVD(v) \
		(((v) << 6) & BM_CLKCTRL_RESET_RSRVD)
#define BM_CLKCTRL_RESET_WDOG_POR_DISABLE	0x00000020
#define BM_CLKCTRL_RESET_EXTERNAL_RESET_ENABLE	0x00000010
#define BM_CLKCTRL_RESET_THERMAL_RESET_ENABLE	0x00000008
#define BM_CLKCTRL_RESET_THERMAL_RESET_DEFAULT	0x00000004
#define BM_CLKCTRL_RESET_CHIP	0x00000002
#define BM_CLKCTRL_RESET_DIG	0x00000001

#define HW_CLKCTRL_STATUS	(0x000001f0)

#define BP_CLKCTRL_STATUS_CPU_LIMIT	30
#define BM_CLKCTRL_STATUS_CPU_LIMIT	0xC0000000
#define BF_CLKCTRL_STATUS_CPU_LIMIT(v) \
		(((v) << 30) & BM_CLKCTRL_STATUS_CPU_LIMIT)
#define BP_CLKCTRL_STATUS_RSRVD	0
#define BM_CLKCTRL_STATUS_RSRVD	0x3FFFFFFF
#define BF_CLKCTRL_STATUS_RSRVD(v)  \
		(((v) << 0) & BM_CLKCTRL_STATUS_RSRVD)

#define HW_CLKCTRL_VERSION	(0x00000200)

#define BP_CLKCTRL_VERSION_MAJOR	24
#define BM_CLKCTRL_VERSION_MAJOR	0xFF000000
#define BF_CLKCTRL_VERSION_MAJOR(v) \
		(((v) << 24) & BM_CLKCTRL_VERSION_MAJOR)
#define BP_CLKCTRL_VERSION_MINOR	16
#define BM_CLKCTRL_VERSION_MINOR	0x00FF0000
#define BF_CLKCTRL_VERSION_MINOR(v)  \
		(((v) << 16) & BM_CLKCTRL_VERSION_MINOR)
#define BP_CLKCTRL_VERSION_STEP	0
#define BM_CLKCTRL_VERSION_STEP	0x0000FFFF
#define BF_CLKCTRL_VERSION_STEP(v)  \
		(((v) << 0) & BM_CLKCTRL_VERSION_STEP)
#endif /* __ARCH_ARM___CLKCTRL_H */
