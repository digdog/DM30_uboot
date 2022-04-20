/*
 * (C) Copyright 2006-2008
 * Texas Instruments, <www.ti.com>
 * Syed Mohammed Khasim <x0khasim@ti.com>
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
#ifndef _MUX_H_
#define _MUX_H_

/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 */

#define IEN	(1 << 8)

#define IDIS	(0 << 8)
#define PTU	(1 << 4)
#define PTD	(0 << 4)
#define EN	(1 << 3)
#define DIS	(0 << 3)

#define M0	0
#define M1	1
#define M2	2
#define M3	3
#define M4	4
#define M5	5
#define M6	6
#define M7	7

/*
 * To get the actual address the offset has to added
 * with OMAP34XX_CTRL_BASE to get the actual address
 */

/*SDRC*/
#define CONTROL_PADCONF_SDRC_D0		0x0030
#define CONTROL_PADCONF_SDRC_D1		0x0032
#define CONTROL_PADCONF_SDRC_D2		0x0034
#define CONTROL_PADCONF_SDRC_D3		0x0036
#define CONTROL_PADCONF_SDRC_D4		0x0038
#define CONTROL_PADCONF_SDRC_D5		0x003A
#define CONTROL_PADCONF_SDRC_D6		0x003C
#define CONTROL_PADCONF_SDRC_D7		0x003E
#define CONTROL_PADCONF_SDRC_D8		0x0040
#define CONTROL_PADCONF_SDRC_D9		0x0042
#define CONTROL_PADCONF_SDRC_D10	0x0044
#define CONTROL_PADCONF_SDRC_D11	0x0046
#define CONTROL_PADCONF_SDRC_D12	0x0048
#define CONTROL_PADCONF_SDRC_D13	0x004A
#define CONTROL_PADCONF_SDRC_D14	0x004C
#define CONTROL_PADCONF_SDRC_D15	0x004E
#define CONTROL_PADCONF_SDRC_D16	0x0050
#define CONTROL_PADCONF_SDRC_D17	0x0052
#define CONTROL_PADCONF_SDRC_D18	0x0054
#define CONTROL_PADCONF_SDRC_D19	0x0056
#define CONTROL_PADCONF_SDRC_D20	0x0058
#define CONTROL_PADCONF_SDRC_D21	0x005A
#define CONTROL_PADCONF_SDRC_D22	0x005C
#define CONTROL_PADCONF_SDRC_D23	0x005E
#define CONTROL_PADCONF_SDRC_D24	0x0060
#define CONTROL_PADCONF_SDRC_D25	0x0062
#define CONTROL_PADCONF_SDRC_D26	0x0064
#define CONTROL_PADCONF_SDRC_D27	0x0066
#define CONTROL_PADCONF_SDRC_D28	0x0068
#define CONTROL_PADCONF_SDRC_D29	0x006A
#define CONTROL_PADCONF_SDRC_D30	0x006C
#define CONTROL_PADCONF_SDRC_D31	0x006E
#define CONTROL_PADCONF_SDRC_CLK	0x0070
#define CONTROL_PADCONF_SDRC_DQS0	0x0072
#define CONTROL_PADCONF_SDRC_DQS1	0x0074
#define CONTROL_PADCONF_SDRC_DQS2	0x0076
#define CONTROL_PADCONF_SDRC_DQS3	0x0078
/*GPMC*/
#define CONTROL_PADCONF_GPMC_A1		0x007A
#define CONTROL_PADCONF_GPMC_A2		0x007C
#define CONTROL_PADCONF_GPMC_A3		0x007E
#define CONTROL_PADCONF_GPMC_A4		0x0080
#define CONTROL_PADCONF_GPMC_A5		0x0082
#define CONTROL_PADCONF_GPMC_A6		0x0084
#define CONTROL_PADCONF_GPMC_A7		0x0086
#define CONTROL_PADCONF_GPMC_A8		0x0088
#define CONTROL_PADCONF_GPMC_A9		0x008A
#define CONTROL_PADCONF_GPMC_A10	0x008C
#define CONTROL_PADCONF_GPMC_D0		0x008E
#define CONTROL_PADCONF_GPMC_D1		0x0090
#define CONTROL_PADCONF_GPMC_D2		0x0092
#define CONTROL_PADCONF_GPMC_D3		0x0094
#define CONTROL_PADCONF_GPMC_D4		0x0096
#define CONTROL_PADCONF_GPMC_D5		0x0098
#define CONTROL_PADCONF_GPMC_D6		0x009A
#define CONTROL_PADCONF_GPMC_D7		0x009C
#define CONTROL_PADCONF_GPMC_D8		0x009E
#define CONTROL_PADCONF_GPMC_D9		0x00A0
#define CONTROL_PADCONF_GPMC_D10	0x00A2
#define CONTROL_PADCONF_GPMC_D11	0x00A4
#define CONTROL_PADCONF_GPMC_D12	0x00A6
#define CONTROL_PADCONF_GPMC_D13	0x00A8
#define CONTROL_PADCONF_GPMC_D14	0x00AA
#define CONTROL_PADCONF_GPMC_D15	0x00AC
#define CONTROL_PADCONF_GPMC_NCS0	0x00AE
#define CONTROL_PADCONF_GPMC_NCS1	0x00B0
#define CONTROL_PADCONF_GPMC_NCS2	0x00B2
#define CONTROL_PADCONF_GPMC_NCS3	0x00B4
#define CONTROL_PADCONF_GPMC_NCS4	0x00B6
#define CONTROL_PADCONF_GPMC_NCS5	0x00B8
#define CONTROL_PADCONF_GPMC_NCS6	0x00BA
#define CONTROL_PADCONF_GPMC_NCS7	0x00BC
#define CONTROL_PADCONF_GPMC_CLK	0x00BE
#define CONTROL_PADCONF_GPMC_NADV_ALE	0x00C0
#define CONTROL_PADCONF_GPMC_NOE	0x00C2
#define CONTROL_PADCONF_GPMC_NWE	0x00C4
#define CONTROL_PADCONF_GPMC_NBE0_CLE	0x00C6
#define CONTROL_PADCONF_GPMC_NBE1	0x00C8
#define CONTROL_PADCONF_GPMC_NWP	0x00CA
#define CONTROL_PADCONF_GPMC_WAIT0	0x00CC
#define CONTROL_PADCONF_GPMC_WAIT1	0x00CE
#define CONTROL_PADCONF_GPMC_WAIT2	0x00D0
#define CONTROL_PADCONF_GPMC_WAIT3	0x00D2
/*DSS*/
#define CONTROL_PADCONF_DSS_PCLK	0x00D4
#define CONTROL_PADCONF_DSS_HSYNC	0x00D6
#define CONTROL_PADCONF_DSS_VSYNC	0x00D8
#define CONTROL_PADCONF_DSS_ACBIAS	0x00DA
#define CONTROL_PADCONF_DSS_DATA0	0x00DC
#define CONTROL_PADCONF_DSS_DATA1	0x00DE
#define CONTROL_PADCONF_DSS_DATA2	0x00E0
#define CONTROL_PADCONF_DSS_DATA3	0x00E2
#define CONTROL_PADCONF_DSS_DATA4	0x00E4
#define CONTROL_PADCONF_DSS_DATA5	0x00E6
#define CONTROL_PADCONF_DSS_DATA6	0x00E8
#define CONTROL_PADCONF_DSS_DATA7	0x00EA
#define CONTROL_PADCONF_DSS_DATA8	0x00EC
#define CONTROL_PADCONF_DSS_DATA9	0x00EE
#define CONTROL_PADCONF_DSS_DATA10	0x00F0
#define CONTROL_PADCONF_DSS_DATA11	0x00F2
#define CONTROL_PADCONF_DSS_DATA12	0x00F4
#define CONTROL_PADCONF_DSS_DATA13	0x00F6
#define CONTROL_PADCONF_DSS_DATA14	0x00F8
#define CONTROL_PADCONF_DSS_DATA15	0x00FA
#define CONTROL_PADCONF_DSS_DATA16	0x00FC
#define CONTROL_PADCONF_DSS_DATA17	0x00FE
#define CONTROL_PADCONF_DSS_DATA18	0x0100
#define CONTROL_PADCONF_DSS_DATA19	0x0102
#define CONTROL_PADCONF_DSS_DATA20	0x0104
#define CONTROL_PADCONF_DSS_DATA21	0x0106
#define CONTROL_PADCONF_DSS_DATA22	0x0108
#define CONTROL_PADCONF_DSS_DATA23	0x010A
/*CAMERA*/
#define CONTROL_PADCONF_CAM_HS		0x010C
#define CONTROL_PADCONF_CAM_VS		0x010E
#define CONTROL_PADCONF_CAM_XCLKA	0x0110
#define CONTROL_PADCONF_CAM_PCLK	0x0112
#define CONTROL_PADCONF_CAM_FLD		0x0114
#define CONTROL_PADCONF_CAM_D0		0x0116
#define CONTROL_PADCONF_CAM_D1		0x0118
#define CONTROL_PADCONF_CAM_D2		0x011A
#define CONTROL_PADCONF_CAM_D3		0x011C
#define CONTROL_PADCONF_CAM_D4		0x011E
#define CONTROL_PADCONF_CAM_D5		0x0120
#define CONTROL_PADCONF_CAM_D6		0x0122
#define CONTROL_PADCONF_CAM_D7		0x0124
#define CONTROL_PADCONF_CAM_D8		0x0126
#define CONTROL_PADCONF_CAM_D9		0x0128
#define CONTROL_PADCONF_CAM_D10		0x012A
#define CONTROL_PADCONF_CAM_D11		0x012C
#define CONTROL_PADCONF_CAM_XCLKB	0x012E
#define CONTROL_PADCONF_CAM_WEN		0x0130
#define CONTROL_PADCONF_CAM_STROBE	0x0132
#define CONTROL_PADCONF_CSI2_DX0	0x0134
#define CONTROL_PADCONF_CSI2_DY0	0x0136
#define CONTROL_PADCONF_CSI2_DX1	0x0138
#define CONTROL_PADCONF_CSI2_DY1	0x013A
/*Audio Interface */
#define CONTROL_PADCONF_MCBSP2_FSX	0x013C
#define CONTROL_PADCONF_MCBSP2_CLKX	0x013E
#define CONTROL_PADCONF_MCBSP2_DR	0x0140
#define CONTROL_PADCONF_MCBSP2_DX	0x0142
#define CONTROL_PADCONF_MMC1_CLK	0x0144
#define CONTROL_PADCONF_MMC1_CMD	0x0146
#define CONTROL_PADCONF_MMC1_DAT0	0x0148
#define CONTROL_PADCONF_MMC1_DAT1	0x014A
#define CONTROL_PADCONF_MMC1_DAT2	0x014C
#define CONTROL_PADCONF_MMC1_DAT3	0x014E
#define CONTROL_PADCONF_MMC1_DAT4	0x0150
#define CONTROL_PADCONF_MMC1_DAT5	0x0152
#define CONTROL_PADCONF_MMC1_DAT6	0x0154
#define CONTROL_PADCONF_MMC1_DAT7	0x0156
/*Wireless LAN */
#define CONTROL_PADCONF_MMC2_CLK	0x0158
#define CONTROL_PADCONF_MMC2_CMD	0x015A
#define CONTROL_PADCONF_MMC2_DAT0	0x015C
#define CONTROL_PADCONF_MMC2_DAT1	0x015E
#define CONTROL_PADCONF_MMC2_DAT2	0x0160
#define CONTROL_PADCONF_MMC2_DAT3	0x0162
#define CONTROL_PADCONF_MMC2_DAT4	0x0164
#define CONTROL_PADCONF_MMC2_DAT5	0x0166
#define CONTROL_PADCONF_MMC2_DAT6	0x0168
#define CONTROL_PADCONF_MMC2_DAT7	0x016A
/*Bluetooth*/
#define CONTROL_PADCONF_MCBSP3_DX	0x016C
#define CONTROL_PADCONF_MCBSP3_DR	0x016E
#define CONTROL_PADCONF_MCBSP3_CLKX	0x0170
#define CONTROL_PADCONF_MCBSP3_FSX	0x0172
#define CONTROL_PADCONF_UART2_CTS	0x0174
#define CONTROL_PADCONF_UART2_RTS	0x0176
#define CONTROL_PADCONF_UART2_TX	0x0178
#define CONTROL_PADCONF_UART2_RX	0x017A
/*Modem Interface */
#define CONTROL_PADCONF_UART1_TX	0x017C
#define CONTROL_PADCONF_UART1_RTS	0x017E
#define CONTROL_PADCONF_UART1_CTS	0x0180
#define CONTROL_PADCONF_UART1_RX	0x0182
#define CONTROL_PADCONF_MCBSP4_CLKX	0x0184
#define CONTROL_PADCONF_MCBSP4_DR	0x0186
#define CONTROL_PADCONF_MCBSP4_DX	0x0188
#define CONTROL_PADCONF_MCBSP4_FSX	0x018A
#define CONTROL_PADCONF_MCBSP1_CLKR	0x018C
#define CONTROL_PADCONF_MCBSP1_FSR	0x018E
#define CONTROL_PADCONF_MCBSP1_DX	0x0190
#define CONTROL_PADCONF_MCBSP1_DR	0x0192
#define CONTROL_PADCONF_MCBSP_CLKS	0x0194
#define CONTROL_PADCONF_MCBSP1_FSX	0x0196
#define CONTROL_PADCONF_MCBSP1_CLKX	0x0198
/*Serial Interface*/
#define CONTROL_PADCONF_UART3_CTS_RCTX	0x019A
#define CONTROL_PADCONF_UART3_RTS_SD	0x019C
#define CONTROL_PADCONF_UART3_RX_IRRX	0x019E
#define CONTROL_PADCONF_UART3_TX_IRTX	0x01A0
#define CONTROL_PADCONF_HSUSB0_CLK	0x01A2
#define CONTROL_PADCONF_HSUSB0_STP	0x01A4
#define CONTROL_PADCONF_HSUSB0_DIR	0x01A6
#define CONTROL_PADCONF_HSUSB0_NXT	0x01A8
#define CONTROL_PADCONF_HSUSB0_DATA0	0x01AA
#define CONTROL_PADCONF_HSUSB0_DATA1	0x01AC
#define CONTROL_PADCONF_HSUSB0_DATA2	0x01AE
#define CONTROL_PADCONF_HSUSB0_DATA3	0x01B0
#define CONTROL_PADCONF_HSUSB0_DATA4	0x01B2
#define CONTROL_PADCONF_HSUSB0_DATA5	0x01B4
#define CONTROL_PADCONF_HSUSB0_DATA6	0x01B6
#define CONTROL_PADCONF_HSUSB0_DATA7	0x01B8
#define CONTROL_PADCONF_I2C1_SCL	0x01BA
#define CONTROL_PADCONF_I2C1_SDA	0x01BC
#define CONTROL_PADCONF_I2C2_SCL	0x01BE
#define CONTROL_PADCONF_I2C2_SDA	0x01C0
#define CONTROL_PADCONF_I2C3_SCL	0x01C2
#define CONTROL_PADCONF_I2C3_SDA	0x01C4
#define CONTROL_PADCONF_I2C4_SCL	0x0A00
#define CONTROL_PADCONF_I2C4_SDA	0x0A02
#define CONTROL_PADCONF_HDQ_SIO		0x01C6
#define CONTROL_PADCONF_MCSPI1_CLK	0x01C8
#define CONTROL_PADCONF_MCSPI1_SIMO	0x01CA
#define CONTROL_PADCONF_MCSPI1_SOMI	0x01CC
#define CONTROL_PADCONF_MCSPI1_CS0	0x01CE
#define CONTROL_PADCONF_MCSPI1_CS1	0x01D0
#define CONTROL_PADCONF_MCSPI1_CS2	0x01D2
#define CONTROL_PADCONF_MCSPI1_CS3	0x01D4
#define CONTROL_PADCONF_MCSPI2_CLK	0x01D6
#define CONTROL_PADCONF_MCSPI2_SIMO	0x01D8
#define CONTROL_PADCONF_MCSPI2_SOMI	0x01DA
#define CONTROL_PADCONF_MCSPI2_CS0	0x01DC
#define CONTROL_PADCONF_MCSPI2_CS1	0x01DE
/*Control and debug */
#define CONTROL_PADCONF_SYS_32K		0x0A04
#define CONTROL_PADCONF_SYS_CLKREQ	0x0A06
#define CONTROL_PADCONF_SYS_NIRQ	0x01E0
#define CONTROL_PADCONF_SYS_BOOT0	0x0A0A
#define CONTROL_PADCONF_SYS_BOOT1	0x0A0C
#define CONTROL_PADCONF_SYS_BOOT2	0x0A0E
#define CONTROL_PADCONF_SYS_BOOT3	0x0A10
#define CONTROL_PADCONF_SYS_BOOT4	0x0A12
#define CONTROL_PADCONF_SYS_BOOT5	0x0A14
#define CONTROL_PADCONF_SYS_BOOT6	0x0A16
#define CONTROL_PADCONF_SYS_OFF_MODE	0x0A18
#define CONTROL_PADCONF_SYS_CLKOUT1	0x0A1A
#define CONTROL_PADCONF_SYS_CLKOUT2	0x01E2
#define CONTROL_PADCONF_JTAG_nTRST	0x0A1C
#define CONTROL_PADCONF_JTAG_TCK	0x0A1E
#define CONTROL_PADCONF_JTAG_TMS	0x0A20
#define CONTROL_PADCONF_JTAG_TDI	0x0A22
#define CONTROL_PADCONF_JTAG_EMU0	0x0A24
#define CONTROL_PADCONF_JTAG_EMU1	0x0A26
#define CONTROL_PADCONF_ETK_CLK		0x0A28
#define CONTROL_PADCONF_ETK_CTL		0x0A2A
#define CONTROL_PADCONF_ETK_D0		0x0A2C
#define CONTROL_PADCONF_ETK_D1		0x0A2E
#define CONTROL_PADCONF_ETK_D2		0x0A30
#define CONTROL_PADCONF_ETK_D3		0x0A32
#define CONTROL_PADCONF_ETK_D4		0x0A34
#define CONTROL_PADCONF_ETK_D5		0x0A36
#define CONTROL_PADCONF_ETK_D6		0x0A38
#define CONTROL_PADCONF_ETK_D7		0x0A3A
#define CONTROL_PADCONF_ETK_D8		0x0A3C
#define CONTROL_PADCONF_ETK_D9		0x0A3E
#define CONTROL_PADCONF_ETK_D10		0x0A40
#define CONTROL_PADCONF_ETK_D11		0x0A42
#define CONTROL_PADCONF_ETK_D12		0x0A44
#define CONTROL_PADCONF_ETK_D13		0x0A46
#define CONTROL_PADCONF_ETK_D14		0x0A48
#define CONTROL_PADCONF_ETK_D15		0x0A4A
#define CONTROL_PADCONF_ETK_CLK_ES2	0x05D8
#define CONTROL_PADCONF_ETK_CTL_ES2	0x05DA
#define CONTROL_PADCONF_ETK_D0_ES2	0x05DC
#define CONTROL_PADCONF_ETK_D1_ES2	0x05DE
#define CONTROL_PADCONF_ETK_D2_ES2	0x05E0
#define CONTROL_PADCONF_ETK_D3_ES2	0x05E2
#define CONTROL_PADCONF_ETK_D4_ES2	0x05E4
#define CONTROL_PADCONF_ETK_D5_ES2	0x05E6
#define CONTROL_PADCONF_ETK_D6_ES2	0x05E8
#define CONTROL_PADCONF_ETK_D7_ES2	0x05EA
#define CONTROL_PADCONF_ETK_D8_ES2	0x05EC
#define CONTROL_PADCONF_ETK_D9_ES2	0x05EE
#define CONTROL_PADCONF_ETK_D10_ES2	0x05F0
#define CONTROL_PADCONF_ETK_D11_ES2	0x05F2
#define CONTROL_PADCONF_ETK_D12_ES2	0x05F4
#define CONTROL_PADCONF_ETK_D13_ES2	0x05F6
#define CONTROL_PADCONF_ETK_D14_ES2	0x05F8
#define CONTROL_PADCONF_ETK_D15_ES2	0x05FA
/*Die to Die */
#define CONTROL_PADCONF_D2D_MCAD0	0x01E4
#define CONTROL_PADCONF_D2D_MCAD1	0x01E6
#define CONTROL_PADCONF_D2D_MCAD2	0x01E8
#define CONTROL_PADCONF_D2D_MCAD3	0x01EA
#define CONTROL_PADCONF_D2D_MCAD4	0x01EC
#define CONTROL_PADCONF_D2D_MCAD5	0x01EE
#define CONTROL_PADCONF_D2D_MCAD6	0x01F0
#define CONTROL_PADCONF_D2D_MCAD7	0x01F2
#define CONTROL_PADCONF_D2D_MCAD8	0x01F4
#define CONTROL_PADCONF_D2D_MCAD9	0x01F6
#define CONTROL_PADCONF_D2D_MCAD10	0x01F8
#define CONTROL_PADCONF_D2D_MCAD11	0x01FA
#define CONTROL_PADCONF_D2D_MCAD12	0x01FC
#define CONTROL_PADCONF_D2D_MCAD13	0x01FE
#define CONTROL_PADCONF_D2D_MCAD14	0x0200
#define CONTROL_PADCONF_D2D_MCAD15	0x0202
#define CONTROL_PADCONF_D2D_MCAD16	0x0204
#define CONTROL_PADCONF_D2D_MCAD17	0x0206
#define CONTROL_PADCONF_D2D_MCAD18	0x0208
#define CONTROL_PADCONF_D2D_MCAD19	0x020A
#define CONTROL_PADCONF_D2D_MCAD20	0x020C
#define CONTROL_PADCONF_D2D_MCAD21	0x020E
#define CONTROL_PADCONF_D2D_MCAD22	0x0210
#define CONTROL_PADCONF_D2D_MCAD23	0x0212
#define CONTROL_PADCONF_D2D_MCAD24	0x0214
#define CONTROL_PADCONF_D2D_MCAD25	0x0216
#define CONTROL_PADCONF_D2D_MCAD26	0x0218
#define CONTROL_PADCONF_D2D_MCAD27	0x021A
#define CONTROL_PADCONF_D2D_MCAD28	0x021C
#define CONTROL_PADCONF_D2D_MCAD29	0x021E
#define CONTROL_PADCONF_D2D_MCAD30	0x0220
#define CONTROL_PADCONF_D2D_MCAD31	0x0222
#define CONTROL_PADCONF_D2D_MCAD32	0x0224
#define CONTROL_PADCONF_D2D_MCAD33	0x0226
#define CONTROL_PADCONF_D2D_MCAD34	0x0228
#define CONTROL_PADCONF_D2D_MCAD35	0x022A
#define CONTROL_PADCONF_D2D_MCAD36	0x022C
#define CONTROL_PADCONF_D2D_CLK26MI	0x022E
#define CONTROL_PADCONF_D2D_NRESPWRON	0x0230
#define CONTROL_PADCONF_D2D_NRESWARM	0x0232
#define CONTROL_PADCONF_D2D_ARM9NIRQ	0x0234
#define CONTROL_PADCONF_D2D_UMA2P6FIQ	0x0236
#define CONTROL_PADCONF_D2D_SPINT	0x0238
#define CONTROL_PADCONF_D2D_FRINT	0x023A
#define CONTROL_PADCONF_D2D_DMAREQ0	0x023C
#define CONTROL_PADCONF_D2D_DMAREQ1	0x023E
#define CONTROL_PADCONF_D2D_DMAREQ2	0x0240
#define CONTROL_PADCONF_D2D_DMAREQ3	0x0242
#define CONTROL_PADCONF_D2D_N3GTRST	0x0244
#define CONTROL_PADCONF_D2D_N3GTDI	0x0246
#define CONTROL_PADCONF_D2D_N3GTDO	0x0248
#define CONTROL_PADCONF_D2D_N3GTMS	0x024A
#define CONTROL_PADCONF_D2D_N3GTCK	0x024C
#define CONTROL_PADCONF_D2D_N3GRTCK	0x024E
#define CONTROL_PADCONF_D2D_MSTDBY	0x0250
#define CONTROL_PADCONF_D2D_SWAKEUP	0x0A4C
#define CONTROL_PADCONF_D2D_IDLEREQ	0x0252
#define CONTROL_PADCONF_D2D_IDLEACK	0x0254
#define CONTROL_PADCONF_D2D_MWRITE	0x0256
#define CONTROL_PADCONF_D2D_SWRITE	0x0258
#define CONTROL_PADCONF_D2D_MREAD	0x025A
#define CONTROL_PADCONF_D2D_SREAD	0x025C
#define CONTROL_PADCONF_D2D_MBUSFLAG	0x025E
#define CONTROL_PADCONF_D2D_SBUSFLAG	0x0260
#define CONTROL_PADCONF_SDRC_CKE0	0x0262
#define CONTROL_PADCONF_SDRC_CKE1	0x0264

#define MUX_VAL(OFFSET,VALUE)\
	writew((VALUE), OMAP34XX_CTRL_BASE + (OFFSET));

#define	CP(x)	(CONTROL_PADCONF_##x)

#endif
