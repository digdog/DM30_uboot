/*
 * Copyright (C) 2008 Embedded Alley Solutions Inc.
 *
 * (C) Copyright 2009-2010 Freescale Semiconductor, Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef MX23_H
#define MX23_H

/*
 * Most of 378x SoC registers are associated with four addresses
 * used for different operations - read/write, set, clear and toggle bits.
 *
 * Some of registers do not implement such feature and, thus, should be
 * accessed/manipulated via single address in common way.
 */
#define REG_RD(x)	(*(volatile unsigned int *)(x))
#define REG_WR(x, v)	((*(volatile unsigned int *)(x)) = (v))
#define REG_SET(x, v)	((*(volatile unsigned int *)((x) + 0x04)) = (v))
#define REG_CLR(x, v)	((*(volatile unsigned int *)((x) + 0x08)) = (v))
#define REG_TOG(x, v)	((*(volatile unsigned int *)((x) + 0x0c)) = (v))

#define MX23_OCRAM_BASE	0x00000000
#define MX23_SDRAM_BASE	0x40000000
#define MX23_REGS_BASE	0x80000000

#define REGS_BASE	MX23_REGS_BASE

/*
 * Register base address
 */

#define REGS_ICOLL_BASE       (REGS_BASE + 0x00000000)
#define REGS_APBH_BASE        (REGS_BASE + 0x00004000)
#define REGS_ECC8_BASE        (REGS_BASE + 0x00008000)
#define REGS_GPMI_BASE        (REGS_BASE + 0x0000C000)
#define REGS_SSP1_BASE        (REGS_BASE + 0x00010000)
#define REGS_PINCTRL_BASE     (REGS_BASE + 0x00018000)
#define REGS_DIGCTL_BASE      (REGS_BASE + 0x0001C000)
#define REGS_EMI_BASE         (REGS_BASE + 0x00020000)
#define REGS_APBX_BASE        (REGS_BASE + 0x00024000)
#define REGS_DCP_BASE         (REGS_BASE + 0x00028000)
#define REGS_OCOTP_BASE       (REGS_BASE + 0x0002C000)
#define REGS_LCDIF_BASE       (REGS_BASE + 0x00030000)
#define REGS_SSP2_BASE        (REGS_BASE + 0x00034000)
#define REGS_CLKCTRL_BASE     (REGS_BASE + 0x00040000)
#define REGS_SAIF1_BASE       (REGS_BASE + 0x00042000)
#define REGS_POWER_BASE       (REGS_BASE + 0x00044000)
#define REGS_SAIF2_BASE       (REGS_BASE + 0x00046000)
#define REGS_AUDIOOUT_BASE    (REGS_BASE + 0x00048000)
#define REGS_AUDIOIN_BASE     (REGS_BASE + 0x0004C000)
#define REGS_LRADC_BASE       (REGS_BASE + 0x00050000)
#define REGS_SPDIF_BASE       (REGS_BASE + 0x00054000)
#define REGS_I2C_BASE         (REGS_BASE + 0x00058000)
#define REGS_RTC_BASE         (REGS_BASE + 0x0005C000)
#define REGS_PWM_BASE         (REGS_BASE + 0x00064000)
#define REGS_TIMROT_BASE      (REGS_BASE + 0x00068000)
#define REGS_UARTAPP_BASE     (REGS_BASE + 0x0006C000)
#define REGS_UARTDBG_BASE     (REGS_BASE + 0x00070000)
#define REGS_DRI_BASE         (REGS_BASE + 0x00074000)
#define REGS_IR_BASE          (REGS_BASE + 0x00078000)
#define REGS_USBPHY_BASE      (REGS_BASE + 0x0007C000)
#define REGS_USBCTRL_BASE     (REGS_BASE + 0x00080000)
#define REGS_DRAM_BASE        (REGS_BASE + 0x000E0000)

#endif /* STMP378X_H */
