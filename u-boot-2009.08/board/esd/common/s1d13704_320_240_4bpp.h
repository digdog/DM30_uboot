/*
 *
 *  Generic Header information generated by 13704CFG.EXE (Build 10)
 *
 *  Copyright (c) 2000,2001 Epson Research and Development, Inc.
 *  All rights reserved.
 *
 *  Panel: 320x240x4bpp 78Hz Mono 4-Bit STN, Disabled (PCLK=6.666MHz)
 *
 *  This file defines the configuration environment and registers,
 *  which can be used by any software, such as display drivers.
 *
 *  PLEASE NOTE: If you FTP this file to a non-Windows platform, make
 *               sure you transfer this file using ASCII, not BINARY
 *               mode.
 *
 */

static S1D_REGS regs_13704_320_240_4bpp[] =
{
	{ 0x00,             0x00 },       /* Revision Code Register */
	{ 0x01,             0x04 }, /*00*/      /* Mode Register 0 Register */
	{ 0x02,             0xA4 }, /*a0*/      /* Mode Register 1 Register */
	{ 0x03,             0x83 }, /*03*/      /* Mode Register 2 Register - bit7 is LUT bypass */
	{ 0x04,             0x27 },       /* Horizontal Panel Size Register */
	{ 0x05,             0xEF },       /* Vertical Panel Size Register (LSB) */
	{ 0x06,             0x00 },       /* Vertical Panel Size Register (MSB) */
	{ 0x07,             0x00 },       /* FPLINE Start Position Register */
	{ 0x08,             0x00 },       /* Horizontal Non-Display Period Register */
	{ 0x09,             0x00 },       /* FPFRAME Start Position Register */
	{ 0x0A,             0x02 },       /* Vertical Non-Display Period Register */
	{ 0x0B,             0x00 },       /* MOD Rate Register */
	{ 0x0C,             0x00 },       /* Screen 1 Start Address Register (LSB) */
	{ 0x0D,             0x00 },       /* Screen 1 Start Address Register (MSB) */
	{ 0x0E,             0x00 },       /* Not Used */
	{ 0x0F,             0x00 },       /* Screen 2 Start Address Register (LSB) */
	{ 0x10,             0x00 },       /* Screen 2 Start Address Register (MSB) */
	{ 0x11,             0x00 },       /* Not Used */
	{ 0x12,             0x00 },       /* Memory Address Offset Register */
	{ 0x13,             0xFF },       /* Screen 1 Vertical Size Register (LSB) */
	{ 0x14,             0x03 },       /* Screen 1 Vertical Size Register (MSB) */
	{ 0x15,             0x00 },       /* Look-Up Table Address Register */
	{ 0x16,             0x00 },       /* Look-Up Table Bank Select Register */
	{ 0x17,             0x00 },       /* Look-Up Table Data Register */
	{ 0x18,             0x01 },       /* GPIO Configuration Control Register */
	{ 0x19,             0x01 },       /* GPIO Status/Control Register */
	{ 0x1A,             0x00 },       /* Scratch Pad Register */
	{ 0x1B,             0x00 },       /* SwivelView Mode Register */
	{ 0x1C,             0xA0 },       /* Line Byte Count Register */
	{ 0x1D,             0x00 },       /* Not Used */
	{ 0x1E,             0x00 },       /* Not Used */
	{ 0x1F,             0x00 },       /* Not Used */
};
