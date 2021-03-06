/*
 * (C) Copyright 2003-2004
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * This file is based on mpc4200fec.h
 * (C) Copyright Motorola, Inc., 2000
 *
 * odin ethernet header file
 */

#ifndef __MPC8220_FEC_H
#define __MPC8220_FEC_H

#include <common.h>
#include <mpc8220.h>
#include "dma.h"

typedef struct ethernet_register_set {

/* [10:2]addr = 00 */

/*  Control and status Registers (offset 000-1FF) */

	volatile u32 fec_id;		/* MBAR_ETH + 0x000 */
	volatile u32 ievent;		/* MBAR_ETH + 0x004 */
	volatile u32 imask;		/* MBAR_ETH + 0x008 */

	volatile u32 RES0[1];		/* MBAR_ETH + 0x00C */
	volatile u32 r_des_active;	/* MBAR_ETH + 0x010 */
	volatile u32 x_des_active;	/* MBAR_ETH + 0x014 */
	volatile u32 r_des_active_cl;	/* MBAR_ETH + 0x018 */
	volatile u32 x_des_active_cl;	/* MBAR_ETH + 0x01C */
	volatile u32 ivent_set;		/* MBAR_ETH + 0x020 */
	volatile u32 ecntrl;		/* MBAR_ETH + 0x024 */

	volatile u32 RES1[6];		/* MBAR_ETH + 0x028-03C */
	volatile u32 mii_data;		/* MBAR_ETH + 0x040 */
	volatile u32 mii_speed;		/* MBAR_ETH + 0x044 */
	volatile u32 mii_status;	/* MBAR_ETH + 0x048 */

	volatile u32 RES2[5];		/* MBAR_ETH + 0x04C-05C */
	volatile u32 mib_data;		/* MBAR_ETH + 0x060 */
	volatile u32 mib_control;	/* MBAR_ETH + 0x064 */

	volatile u32 RES3[6];		/* MBAR_ETH + 0x068-7C */
	volatile u32 r_activate;	/* MBAR_ETH + 0x080 */
	volatile u32 r_cntrl;		/* MBAR_ETH + 0x084 */
	volatile u32 r_hash;		/* MBAR_ETH + 0x088 */
	volatile u32 r_data;		/* MBAR_ETH + 0x08C */
	volatile u32 ar_done;		/* MBAR_ETH + 0x090 */
	volatile u32 r_test;		/* MBAR_ETH + 0x094 */
	volatile u32 r_mib;		/* MBAR_ETH + 0x098 */
	volatile u32 r_da_low;		/* MBAR_ETH + 0x09C */
	volatile u32 r_da_high;		/* MBAR_ETH + 0x0A0 */

	volatile u32 RES4[7];		/* MBAR_ETH + 0x0A4-0BC */
	volatile u32 x_activate;	/* MBAR_ETH + 0x0C0 */
	volatile u32 x_cntrl;		/* MBAR_ETH + 0x0C4 */
	volatile u32 backoff;		/* MBAR_ETH + 0x0C8 */
	volatile u32 x_data;		/* MBAR_ETH + 0x0CC */
	volatile u32 x_status;		/* MBAR_ETH + 0x0D0 */
	volatile u32 x_mib;		/* MBAR_ETH + 0x0D4 */
	volatile u32 x_test;		/* MBAR_ETH + 0x0D8 */
	volatile u32 fdxfc_da1;		/* MBAR_ETH + 0x0DC */
	volatile u32 fdxfc_da2;		/* MBAR_ETH + 0x0E0 */
	volatile u32 paddr1;		/* MBAR_ETH + 0x0E4 */
	volatile u32 paddr2;		/* MBAR_ETH + 0x0E8 */
	volatile u32 op_pause;		/* MBAR_ETH + 0x0EC */

	volatile u32 RES5[4];		/* MBAR_ETH + 0x0F0-0FC */
	volatile u32 instr_reg;		/* MBAR_ETH + 0x100 */
	volatile u32 context_reg;	/* MBAR_ETH + 0x104 */
	volatile u32 test_cntrl;	/* MBAR_ETH + 0x108 */
	volatile u32 acc_reg;		/* MBAR_ETH + 0x10C */
	volatile u32 ones;		/* MBAR_ETH + 0x110 */
	volatile u32 zeros;		/* MBAR_ETH + 0x114 */
	volatile u32 iaddr1;		/* MBAR_ETH + 0x118 */
	volatile u32 iaddr2;		/* MBAR_ETH + 0x11C */
	volatile u32 gaddr1;		/* MBAR_ETH + 0x120 */
	volatile u32 gaddr2;		/* MBAR_ETH + 0x124 */
	volatile u32 random;		/* MBAR_ETH + 0x128 */
	volatile u32 rand1;		/* MBAR_ETH + 0x12C */
	volatile u32 tmp;		/* MBAR_ETH + 0x130 */

	volatile u32 RES6[3];		/* MBAR_ETH + 0x134-13C */
	volatile u32 fifo_id;		/* MBAR_ETH + 0x140 */
	volatile u32 x_wmrk;		/* MBAR_ETH + 0x144 */
	volatile u32 fcntrl;		/* MBAR_ETH + 0x148 */
	volatile u32 r_bound;		/* MBAR_ETH + 0x14C */
	volatile u32 r_fstart;		/* MBAR_ETH + 0x150 */
	volatile u32 r_count;		/* MBAR_ETH + 0x154 */
	volatile u32 r_lag;		/* MBAR_ETH + 0x158 */
	volatile u32 r_read;		/* MBAR_ETH + 0x15C */
	volatile u32 r_write;		/* MBAR_ETH + 0x160 */
	volatile u32 x_count;		/* MBAR_ETH + 0x164 */
	volatile u32 x_lag;		/* MBAR_ETH + 0x168 */
	volatile u32 x_retry;		/* MBAR_ETH + 0x16C */
	volatile u32 x_write;		/* MBAR_ETH + 0x170 */
	volatile u32 x_read;		/* MBAR_ETH + 0x174 */

	volatile u32 RES7[2];		/* MBAR_ETH + 0x178-17C */
	volatile u32 fm_cntrl;		/* MBAR_ETH + 0x180 */
	volatile u32 rfifo_data;	/* MBAR_ETH + 0x184 */
	volatile u32 rfifo_status;	/* MBAR_ETH + 0x188 */
	volatile u32 rfifo_cntrl;	/* MBAR_ETH + 0x18C */
	volatile u32 rfifo_lrf_ptr;	/* MBAR_ETH + 0x190 */
	volatile u32 rfifo_lwf_ptr;	/* MBAR_ETH + 0x194 */
	volatile u32 rfifo_alarm;	/* MBAR_ETH + 0x198 */
	volatile u32 rfifo_rdptr;	/* MBAR_ETH + 0x19C */
	volatile u32 rfifo_wrptr;	/* MBAR_ETH + 0x1A0 */
	volatile u32 tfifo_data;	/* MBAR_ETH + 0x1A4 */
	volatile u32 tfifo_status;	/* MBAR_ETH + 0x1A8 */
	volatile u32 tfifo_cntrl;	/* MBAR_ETH + 0x1AC */
	volatile u32 tfifo_lrf_ptr;	/* MBAR_ETH + 0x1B0 */
	volatile u32 tfifo_lwf_ptr;	/* MBAR_ETH + 0x1B4 */
	volatile u32 tfifo_alarm;	/* MBAR_ETH + 0x1B8 */
	volatile u32 tfifo_rdptr;	/* MBAR_ETH + 0x1BC */
	volatile u32 tfifo_wrptr;	/* MBAR_ETH + 0x1C0 */

	volatile u32 reset_cntrl;	/* MBAR_ETH + 0x1C4 */
	volatile u32 xmit_fsm;		/* MBAR_ETH + 0x1C8 */

	volatile u32 RES8[3];		/* MBAR_ETH + 0x1CC-1D4 */
	volatile u32 rdes_data0;	/* MBAR_ETH + 0x1D8 */
	volatile u32 rdes_data1;	/* MBAR_ETH + 0x1DC */
	volatile u32 r_length;		/* MBAR_ETH + 0x1E0 */
	volatile u32 x_length;		/* MBAR_ETH + 0x1E4 */
	volatile u32 x_addr;		/* MBAR_ETH + 0x1E8 */
	volatile u32 cdes_data;		/* MBAR_ETH + 0x1EC */
	volatile u32 status;		/* MBAR_ETH + 0x1F0 */
	volatile u32 dma_control;	/* MBAR_ETH + 0x1F4 */
	volatile u32 des_cmnd;		/* MBAR_ETH + 0x1F8 */
	volatile u32 data;		/* MBAR_ETH + 0x1FC */

	/*  MIB COUNTERS (Offset 200-2FF) */

	volatile u32 rmon_t_drop;	/* MBAR_ETH + 0x200 */
	volatile u32 rmon_t_packets;	/* MBAR_ETH + 0x204 */
	volatile u32 rmon_t_bc_pkt;	/* MBAR_ETH + 0x208 */
	volatile u32 rmon_t_mc_pkt;	/* MBAR_ETH + 0x20C */
	volatile u32 rmon_t_crc_align;	/* MBAR_ETH + 0x210 */
	volatile u32 rmon_t_undersize;	/* MBAR_ETH + 0x214 */
	volatile u32 rmon_t_oversize;	/* MBAR_ETH + 0x218 */
	volatile u32 rmon_t_frag;	/* MBAR_ETH + 0x21C */
	volatile u32 rmon_t_jab;	/* MBAR_ETH + 0x220 */
	volatile u32 rmon_t_col;	/* MBAR_ETH + 0x224 */
	volatile u32 rmon_t_p64;	/* MBAR_ETH + 0x228 */
	volatile u32 rmon_t_p65to127;	/* MBAR_ETH + 0x22C */
	volatile u32 rmon_t_p128to255;	/* MBAR_ETH + 0x230 */
	volatile u32 rmon_t_p256to511;	/* MBAR_ETH + 0x234 */
	volatile u32 rmon_t_p512to1023;	/* MBAR_ETH + 0x238 */
	volatile u32 rmon_t_p1024to2047;/* MBAR_ETH + 0x23C */
	volatile u32 rmon_t_p_gte2048;	/* MBAR_ETH + 0x240 */
	volatile u32 rmon_t_octets;	/* MBAR_ETH + 0x244 */
	volatile u32 ieee_t_drop;	/* MBAR_ETH + 0x248 */
	volatile u32 ieee_t_frame_ok;	/* MBAR_ETH + 0x24C */
	volatile u32 ieee_t_1col;	/* MBAR_ETH + 0x250 */
	volatile u32 ieee_t_mcol;	/* MBAR_ETH + 0x254 */
	volatile u32 ieee_t_def;	/* MBAR_ETH + 0x258 */
	volatile u32 ieee_t_lcol;	/* MBAR_ETH + 0x25C */
	volatile u32 ieee_t_excol;	/* MBAR_ETH + 0x260 */
	volatile u32 ieee_t_macerr;	/* MBAR_ETH + 0x264 */
	volatile u32 ieee_t_cserr;	/* MBAR_ETH + 0x268 */
	volatile u32 ieee_t_sqe;	/* MBAR_ETH + 0x26C */
	volatile u32 t_fdxfc;		/* MBAR_ETH + 0x270 */
	volatile u32 ieee_t_octets_ok;	/* MBAR_ETH + 0x274 */

	volatile u32 RES9[2];		/* MBAR_ETH + 0x278-27C */
	volatile u32 rmon_r_drop;	/* MBAR_ETH + 0x280 */
	volatile u32 rmon_r_packets;	/* MBAR_ETH + 0x284 */
	volatile u32 rmon_r_bc_pkt;	/* MBAR_ETH + 0x288 */
	volatile u32 rmon_r_mc_pkt;	/* MBAR_ETH + 0x28C */
	volatile u32 rmon_r_crc_align;	/* MBAR_ETH + 0x290 */
	volatile u32 rmon_r_undersize;	/* MBAR_ETH + 0x294 */
	volatile u32 rmon_r_oversize;	/* MBAR_ETH + 0x298 */
	volatile u32 rmon_r_frag;	/* MBAR_ETH + 0x29C */
	volatile u32 rmon_r_jab;	/* MBAR_ETH + 0x2A0 */

	volatile u32 rmon_r_resvd_0;	/* MBAR_ETH + 0x2A4 */

	volatile u32 rmon_r_p64;	/* MBAR_ETH + 0x2A8 */
	volatile u32 rmon_r_p65to127;	/* MBAR_ETH + 0x2AC */
	volatile u32 rmon_r_p128to255;	/* MBAR_ETH + 0x2B0 */
	volatile u32 rmon_r_p256to511;	/* MBAR_ETH + 0x2B4 */
	volatile u32 rmon_r_p512to1023;	/* MBAR_ETH + 0x2B8 */
	volatile u32 rmon_r_p1024to2047;/* MBAR_ETH + 0x2BC */
	volatile u32 rmon_r_p_gte2048;	/* MBAR_ETH + 0x2C0 */
	volatile u32 rmon_r_octets;	/* MBAR_ETH + 0x2C4 */
	volatile u32 ieee_r_drop;	/* MBAR_ETH + 0x2C8 */
	volatile u32 ieee_r_frame_ok;	/* MBAR_ETH + 0x2CC */
	volatile u32 ieee_r_crc;	/* MBAR_ETH + 0x2D0 */
	volatile u32 ieee_r_align;	/* MBAR_ETH + 0x2D4 */
	volatile u32 r_macerr;		/* MBAR_ETH + 0x2D8 */
	volatile u32 r_fdxfc;		/* MBAR_ETH + 0x2DC */
	volatile u32 ieee_r_octets_ok;	/* MBAR_ETH + 0x2E0 */

	volatile u32 RES10[6];		/* MBAR_ETH + 0x2E4-2FC */

	volatile u32 RES11[64];		/* MBAR_ETH + 0x300-3FF */
} ethernet_regs;

/* Receive & Transmit Buffer Descriptor definitions */
typedef struct BufferDescriptor {
	u16 status;
	u16 dataLength;
	u32 dataPointer;
} FEC_RBD;

typedef struct {
	u16 status;
	u16 dataLength;
	u32 dataPointer;
} FEC_TBD;

/* private structure */
typedef enum {
	SEVENWIRE,		/* 7-wire       */
	MII10,			/* MII 10Mbps   */
	MII100			/* MII 100Mbps  */
} xceiver_type;

typedef struct {
	ethernet_regs *eth;
	xceiver_type xcv_type;	/* transceiver type */
	FEC_RBD *rbdBase;	/* RBD ring */
	FEC_TBD *tbdBase;	/* TBD ring */
	u16 rbdIndex;		/* next receive BD to read */
	u16 tbdIndex;		/* next transmit BD to send */
	u16 usedTbdIndex;	/* next transmit BD to clean */
	u16 cleanTbdNum;	/* the number of available transmit BDs */
} mpc8220_fec_priv;

/* Ethernet parameter area */
#define FEC_TBD_BASE	    (FEC_PARAM_BASE + 0x00)
#define FEC_TBD_NEXT	    (FEC_PARAM_BASE + 0x04)
#define FEC_RBD_BASE	    (FEC_PARAM_BASE + 0x08)
#define FEC_RBD_NEXT	    (FEC_PARAM_BASE + 0x0c)

/* BD Numer definitions */
#define FEC_TBD_NUM	   48	/* The user can adjust this value */
#define FEC_RBD_NUM	   32	/* The user can adjust this value */

/* packet size limit */
#define FEC_MAX_PKT_SIZE   1536

/* RBD bits definitions */
#define FEC_RBD_EMPTY	0x8000	/* Buffer is empty */
#define FEC_RBD_WRAP	0x2000	/* Last BD in ring */
#define FEC_RBD_INT	0x1000	/* Interrupt */
#define FEC_RBD_LAST	0x0800	/* Buffer is last in frame(useless) */
#define FEC_RBD_MISS	0x0100	/* Miss bit for prom mode */
#define FEC_RBD_BC	0x0080	/* The received frame is broadcast frame */
#define FEC_RBD_MC	0x0040	/* The received frame is multicast frame */
#define FEC_RBD_LG	0x0020	/* Frame length violation */
#define FEC_RBD_NO	0x0010	/* Nonoctet align frame */
#define FEC_RBD_SH	0x0008	/* Short frame */
#define FEC_RBD_CR	0x0004	/* CRC error */
#define FEC_RBD_OV	0x0002	/* Receive FIFO overrun */
#define FEC_RBD_TR	0x0001	/* Frame is truncated */
#define FEC_RBD_ERR	(FEC_RBD_LG | FEC_RBD_NO | FEC_RBD_CR | \
			 FEC_RBD_OV | FEC_RBD_TR)

/* TBD bits definitions */
#define FEC_TBD_READY	0x8000	/* Buffer is ready */
#define FEC_TBD_WRAP	0x2000	/* Last BD in ring */
#define FEC_TBD_INT	0x1000	/* Interrupt */
#define FEC_TBD_LAST	0x0800	/* Buffer is last in frame */
#define FEC_TBD_TC	0x0400	/* Transmit the CRC */
#define FEC_TBD_ABC	0x0200	/* Append bad CRC */

/* MII-related definitios */
#define FEC_MII_DATA_ST		0x40000000	/* Start of frame delimiter */
#define FEC_MII_DATA_OP_RD	0x20000000	/* Perform a read operation */
#define FEC_MII_DATA_OP_WR	0x10000000	/* Perform a write operation */
#define FEC_MII_DATA_PA_MSK	0x0f800000	/* PHY Address field mask */
#define FEC_MII_DATA_RA_MSK	0x007c0000	/* PHY Register field mask */
#define FEC_MII_DATA_TA		0x00020000	/* Turnaround */
#define FEC_MII_DATA_DATAMSK	0x0000ffff	/* PHY data field */

#define FEC_MII_DATA_RA_SHIFT	18	/* MII Register address bits */
#define FEC_MII_DATA_PA_SHIFT	23	/* MII PHY address bits */

#endif /* __MPC8220_FEC_H */
