#if 0
#include <pkgconf/hal.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/plf_mmap.h>
#include <redboot.h>
#include <stdlib.h>
#define  _FLASH_PRIVATE_
#include "gpmi/include/hal_io.h"
#include "gpmi/include/flash.h"
#endif

#include <common.h>

#include "gpmi/include/imx_nfc.h"
#include "gpmi/include/hal_soc.h"
#include "gpmi/include/nand_dma.h"
#include "gpmi/include/nand_dma_descriptor.h"
#include "gpmi/include/stmp_bch.h"

/* Search good / bad pattern on the first page only */
#define NAND_BBT_SCAN1STPAGE    0x00000001
/* Search good / bad pattern on the first and the second page */
#define NAND_BBT_SCAN2NDPAGE    0x00000002
/* Search good / bad pattern on the last page only */
#define NAND_BBT_SCANLSTPAGE    0x00000004

static const flash_dev_info_t *flash_dev_info;

static const flash_dev_info_t supported_devices[] = {
//#include "gpmi/include/mxc_nand_parts.inl"
    {
        device_id  : 0xd398, // TOSHIBA TH58NVG3D4xFT00 (2KB page 1G x 8 bit MLC nand)
        device_id2 : 0xFFFF, //  default for MX51
        device_id3 : 0xFFFF,
        device_id4 : 0xFFFF,
        col_cycle: 2,
        row_cycle: 3,
        page_size  : 2048,
        spare_size : 64,
        pages_per_block : 128,
        block_size : 128 * 2048,
        block_count: 4096,
        device_size: 0x40000000,   /* 1G */
        port_size  : MXC_NAND_8_BIT,
        type       : NAND_MLC,
        fis_start_addr: 0x80000,       // first 0.5MB reserved for Redboot
        bbt_blk_max_nr: 4,      // reserve 4 blocks for the bad block tables
            // BI is at 2048th byte out of factory (0-indexed)
            // our NFC read out data like this:
            // | 528 | 528 | 528 | 528 |
            //    P1      P2       P3         P4
            // 0-527|528-1055/1056-1583/1584-2111
            // So the last subpage starts: 1584th byte. 2048th byte is at offset 464.
        bi_off     : 3 * 512 + 464, // BUF3 offset + 464
        options    : NAND_BBT_SCANLSTPAGE,
        vendor_info: "TOSHIBA TH58NVG3D4xFT00 8-bit 2K page 1GB MLC",
    },
};

#define NUM_DEVICES (sizeof(supported_devices)/sizeof(flash_dev_info_t))

#define HAL_VIRT_TO_PHYS_ADDRESS( vaddr, paddr ) 		\
	do {							\
		/* no change */ ;                               \
		(paddr) = vaddr;                                  \
	} while (0)

#define COL_CYCLE           flash_dev_info->col_cycle
#define ROW_CYCLE           flash_dev_info->row_cycle
#define NF_PG_SZ            flash_dev_info->page_size
#define NF_PG_PER_BLK       flash_dev_info->pages_per_block
#define NF_DEV_SZ           flash_dev_info->device_size
#define NF_BLK_SZ           flash_dev_info->block_size
#define NF_BLK_CNT          flash_dev_info->block_count
#define NF_VEND_INFO        flash_dev_info->vendor_info
#define NF_OPTIONS          flash_dev_info->options
#define NF_BBT_MAX_NR       flash_dev_info->bbt_blk_max_nr
#define NF_OPTIONS          flash_dev_info->options
#define NF_BI_OFF           flash_dev_info->bi_off

#define BLOCK_TO_OFFSET(blk)            (blk * NF_PG_PER_BLK * NF_PG_SZ)
#define BLOCK_TO_PAGE(blk)              (blk * NF_PG_PER_BLK)
#define BLOCK_PAGE_TO_OFFSET(blk, pge)  ((blk * NF_PG_PER_BLK + pge) * NF_PG_SZ)
#define OFFSET_TO_BLOCK(offset)         ((offset / NF_PG_SZ) / NF_PG_PER_BLK)
#define OFFSET_TO_PAGE(offset)          ((offset / NF_PG_SZ) % NF_PG_PER_BLK)

static int nand_flash_index = -1;
static int g_nfc_debug_level = NFC_DEBUG_MIN;

#define nfc_printf(level, args...)          		\
	do {                                		\
			printf(args);          		\
	} while(0)

//if (g_nfc_debug_level >= level)     	\

#define SUCCESS 0
#define FAIL 1

#define RSVBUFFADDR  	0x44000000	// unbufferable, uncacheable
#define BUFFER_SIZE   	0x10000

static NAND_ECC_Params_t nand_ecc_params;

static void nand_ConfigurePinmux()
{
    unsigned int val;

    // By default, all NAND pins are GPIO's after reset
    // Connect NAND signals by configuring 2-bit pinmux value for each pin
    // Most NAND pins are the default peripheral = 0b00.
    // Startup already checked PINCTRL present bits
    val = readl(PINCTRL_ADDR); // out of reset and gate
    val &= ~(0x3<<30);
    writel(val, PINCTRL_ADDR);

    // -----------------------------------------------------------------
    // Always power up lower 8 bits of NAND Data bus.
    val = readl(PINCTRL_MUXSEL0_ADDR);
    val &= 0xffff0000; // gpmi data0~7
    writel(val, PINCTRL_MUXSEL0_ADDR);

    // Select the ALE, CLE, WRN, and RDN pins of NAND.
    val = readl(PINCTRL_MUXSEL1_ADDR);
    val &= ~(0x3<<18); // gpmi_rdn
    val &= ~(0x3<<16);  //gpmi wrn
    val &= ~(0x3<<2);  //gpmi ale
    val &= ~(0x3<<0);  //gpmi cle
    writel(val, PINCTRL_MUXSEL1_ADDR);

    // Set the pin drive for the RDN, WRN pin to 12mA.
    val = readl(PINCTRL_DRIVE3_ADDR);
    val &= ~(0x3<<4); // gpmi_rdn
    val |= (0x2<<4);
    val &= ~(0x3<<0); // gpmi_wrn
    val |= (0x2<<0);
    writel(val, PINCTRL_DRIVE3_ADDR);

    // Power up Ready Busy for NAND0
    val = readl(PINCTRL_MUXSEL1_ADDR);
    val &= ~(0x3<<6); // gpmi_rdy0
    writel(val, PINCTRL_MUXSEL1_ADDR);

    // Power up CE0 by setting bit field to b00.
    val = readl(PINCTRL_MUXSEL5_ADDR);
    val &= ~(0x3<<24); // gpmi ce0
    writel(val, PINCTRL_MUXSEL5_ADDR);

    // this is called gpmi_wpn in data spec!
    val = readl(PINCTRL_MUXSEL1_ADDR);
    val &= ~(0x3<<14); // gpmi_wpn
    writel(val, PINCTRL_MUXSEL1_ADDR);
}

void nandflash_query(void *data)
{
    return;
}

int nand_offset2pagenum(unsigned int offset, unsigned int *pagenum)
{
    if (offset % NF_PG_SZ) {
	printf("%s: nand read/write/erase address should be 2KB aligned\n",
	       __FUNCTION__);
	return 1;
    }

    *pagenum = (offset / NF_PG_SZ);

    return 0;
}

int nandflash_program_buf(void *addr, void *data, int len)
{
    int status;
    int i;
    unsigned char *p8PageBuf, *phy_8PageBuf;
    unsigned char *p8AuxillaryBuf, *phy_8AuxillaryBuf;
    int size;
    unsigned int offset, pageNum, BlockNum;

    nfc_printf(NFC_DEBUG_MED, "===========Write NAND==============\n");
    p8PageBuf = (unsigned char *) (RSVBUFFADDR + BUFFER_SIZE);
    memset(p8PageBuf, 0x00, 0x2000);
    p8AuxillaryBuf =
	(unsigned char *) (RSVBUFFADDR + BUFFER_SIZE + 0x2000);
    memset(p8AuxillaryBuf, 0x00, 0x2000);

    HAL_VIRT_TO_PHYS_ADDRESS(p8PageBuf, phy_8PageBuf);
    HAL_VIRT_TO_PHYS_ADDRESS(p8AuxillaryBuf, phy_8AuxillaryBuf);

    size = len;
    // convert addr to pageNum;
    nfc_printf(NFC_DEBUG_MAX, "len: 0x%x\n", len);
    nfc_printf(NFC_DEBUG_MAX, "addr: 0x%x\n", (unsigned int) addr);

    nand_offset2pagenum((unsigned int) addr, &pageNum);

    BlockNum = pageNum / NF_PG_PER_BLK;
    while (size > 0) {
	nand_Erase(0, BlockNum);
	BlockNum++;
	size -= NF_PG_SZ * NF_PG_PER_BLK;
    }

    i = 0;
    size = len;
    while (size > 0) {
	nfc_printf(NFC_DEBUG_MAX, "Page Num: 0x%x", pageNum);

	memcpy(p8PageBuf, (unsigned int) data + i * NF_PG_SZ, NF_PG_SZ);
	status = nand_Write(0, pageNum, phy_8PageBuf, phy_8AuxillaryBuf);
	if (status == SUCCESS) {
	    nfc_printf(NFC_DEBUG_MED, "Write Passed.\n");
	}

	size -= NF_PG_SZ;
	pageNum++;
	i++;
    }

    return SUCCESS;
}

int nandflash_erase_block(void *block, unsigned int size)
{
    unsigned int offset, BlockNum;

    return SUCCESS;

    printf("block: 0x%x size: 0x%x\n", (unsigned int) block, size);

    BlockNum = (unsigned int) block;

    while (size > 0) {
	nand_Erase(0, BlockNum);
	size -= NF_PG_SZ * 64;
	BlockNum++;
    }

    return 0;
}

#if 0
bool nandflash_code_overlaps(void *start, void *end)
{
    extern unsigned char _stext[], _etext[];

    return ((((unsigned long) &_stext >= (unsigned long) start) &&
	     ((unsigned long) &_stext < (unsigned long) end)) ||
	    (((unsigned long) &_etext >= (unsigned long) start) &&
	     ((unsigned long) &_etext < (unsigned long) end)));
}
#endif

int nandflash_hwr_map_error(int e)
{
    return e;
}

int nandflash_lock_block(void *block)
{
    // Not supported yet
    return 0;
}

int nandflash_unlock_block(void *block, int block_size, int blocks)
{
    // Not supported yet
    return 0;
}

int nand_FindGpmiCycles(unsigned int u32NandTime_ns,
			unsigned int u32GpmiPeriod_ns,
			unsigned int u32MaxSearchTimes)
{
    int i, iCycleTime = u32GpmiPeriod_ns;

    // Assume a maximum of 15 tests
    for (i = 1; i < u32MaxSearchTimes; i++) {
	if (iCycleTime > u32NandTime_ns)
	    break;
	else
	    iCycleTime += u32GpmiPeriod_ns;
    }
    return i;
}

////////////////////////////////////////////////////////////////////////////////
//! \brief Setup the NAND clocks
//!
//! This function sets the GPMI NAND timing based upon the NAND timings that
//! are passed in.  This module assumes a GPMI_CLK of 24MHz if the GpmiPeriod
//! parameter is zero (41nsec period).  If the GPMI clock period is non-zero
//! it is used in the calculation of the new register values.
//!
//! \param[in]  pNANDTiming Structure with Address Setup, Data Setup and Hold.
//! \param[in]  u32GpmiPeriod_ns GPMI Clock Period in nsec.
//!
//! \return void
////////////////////////////////////////////////////////////////////////////////
void nand_GpmiSetNandTiming(void *pNewNANDTiming,
			    unsigned int u32GpmiPeriod_ns)
{
    NAND_Timing_t *pNANDTiming = (NAND_Timing_t *) pNewNANDTiming;
    unsigned int val;


    // CLKGATE = 0 and DIV = 1 (we're assuming a 24MHz XTAL for this).
    // HW_CLKCTRL_GPMICLKCTRL_WR(0x01);
    // Clock dividers are now set globally for PLL bypass in startup / setup_default_clocks()
    // The divider may also be changed by drivers (like USB) that turn on the PLL
    // HW_CLKCTRL_GPMICLKCTRL_CLR(BM_CLKCTRL_GPMICLKCTRL_CLKGATE); // ungate

    // Ungate GPMICLK. Because the gate is upstream of the divider, special
    // care must be taken to make sure the divider is set correctly. Any
    // change to HW_CLKCTRL_GPMICLKCTRL.B.DIV while the clock is gated is
    // saved to the register, but *NOT* transferred to the actual divider.
    // Clearing HW_CLKCTRL_GPMICLKCTRL.B.WAIT_PLL_LOCK serves two purposes.
    // First, it forces the divider to update because it writes the control
    // register while the clock is not gated. Second, it makes sure the update
    // completes immediately by removing the PLL locked qualifier.
    //HW_CLKCTRL_GPMI.B.CLKGATE = 0;
    val = readl(CLKCTRL_GPMI_ADDR);	// clock control GPMI, clock on
    val &= ~(1 << 31);
    writel(val, CLKCTRL_GPMI_ADDR);


    // If u32GpmiPeriod is passed in as 0, we'll use the default 41nsec
    // for a 24MHz clock.
    if (u32GpmiPeriod_ns == 0)
	u32GpmiPeriod_ns = 42;

    // Set all NAND timing parameters
    // Setup pin timing parameters: ADRESS_SETUP, DATA_SETUP, and DATA_HOLD.
    // (Note that these are in units of GPMICLK cycles.)
    {
	unsigned int u32AddressSetup;
	unsigned int u32DataSetup;
	unsigned int u32DataHold;
	unsigned int u32DataSampleTime;
	unsigned int u32BusyTimeout;

	u32AddressSetup =
	    nand_FindGpmiCycles(pNANDTiming->m_u8AddressSetup,
				u32GpmiPeriod_ns, 20);
	u32DataSetup =
	    nand_FindGpmiCycles(pNANDTiming->m_u8DataSetup,
				u32GpmiPeriod_ns, 20);
	u32DataHold =
	    nand_FindGpmiCycles(pNANDTiming->m_u8DataHold,
				u32GpmiPeriod_ns, 20);

	// DSAMPLE is calculated in 1/2 GPMI clock units, so use shifts to compensate.
	// This one should not round up so I subtract the cycle back off.
	u32DataSampleTime =
	    nand_FindGpmiCycles((pNANDTiming->m_u8DSAMPLE_TIME +
				 (u32GpmiPeriod_ns >> 2)),
				(u32GpmiPeriod_ns >> 1), 20) - 1;

	HW_GPMI_TIMING0_WR(NAND_GPMI_TIMING0
			   (u32AddressSetup, u32DataSetup, u32DataHold));

	// set rdn_delay
	val = readl(GPMI_CTRL1_ADDR);
	val &= ~(0xf << 12);
	val |= ((u32DataSampleTime & 0xf) << 12);
	writel(val, GPMI_CTRL1_ADDR);

	// Set DSAMPLE_TIME value
	//BW_GPMI_CTRL1_DSAMPLE_TIME(ROUND_CLK(pNANDTiming->m_u8DSAMPLE_TIME, u32GpmiPeriod));
	//BW_GPMI_CTRL1_DSAMPLE_TIME(u32DataSampleTime);

	u32BusyTimeout = 0x132;
	//        u32BusyTimeout =  nand_FindGpmiCycles(((10000000 + 4095) / 4096),
	//                                                     u32GpmiPeriod_ns,
	//                                                     ((10000000 + 4095) / 4096)/5);

	// Number of cycles / 4096.
	//        HW_GPMI_TIMING1_WR( BF_GPMI_TIMING1_DEVICE_BUSY_TIMEOUT(u32BusyTimeout));
	writel(u32BusyTimeout << 16, GPMI_TIMEOUT1_ADDR);

	nfc_printf(NFC_DEBUG_MED, "GPMI TIME0: 0x%x\n",
		   readl(GPMI_TIMEOUT0_ADDR));
	nfc_printf(NFC_DEBUG_MED, "GPMI TIME1: 0x%x\n",
		   readl(GPMI_TIMEOUT1_ADDR));
    }
}

int nand_EnableGPMI(void)
{
    unsigned int val;

#if 1
    const NAND_Timing_t zFailsafeTimings = {
	100,			//!< Data Setup (ns)
	80,			//!< Data Hold (ns)
	120,			//!< Address Setup (ns)
	10			//!< DSAMPLE_TIME (ns)
    };

#else
    const NAND_Timing_t zFailsafeTimings = {
	10,			//!< Data Setup (ns)
	5,			//!< Data Hold (ns)
	10,			//!< Address Setup (ns)
	10			//!< DSAMPLE_TIME (ns)
    };
#endif

    /* enable GPMI clock */
    val = readl(CLKCTRL_GPMI_ADDR);	// clock control GPMI, clock on
    val &= ~(1 << 31);
    writel(val, CLKCTRL_GPMI_ADDR);

    // Bring GPMI out of soft reset and release clock gate.
    // SoftReset needs to be set before ClockGate - can't be the same
    // instruction.

    // preparing soft reset and clock gate.
    val = readl(GPMI_CTRL0_ADDR);
    val &= ~(0x3 << 30);
    writel(val, GPMI_CTRL0_ADDR);

    // Only soft reset if GPMI hasn't been enabled.
    val = readl(GPMI_CTRL0_ADDR);
    val |= (1 << 31);
    writel(val, GPMI_CTRL0_ADDR);

    // At 24Mhz, it takes no more than 4 clocks (160 ns) Maximum for
    // the part to reset, reading the register twice should
    // be sufficient to get 4 clks delay.
    // waiting for confirmation of soft reset
    while (!(readl(GPMI_CTRL0_ADDR) & (1 << 30))) {
	// busy wait
    }

    // Now bring out of reset and disable Clk gate.
    val = readl(GPMI_CTRL0_ADDR);
    val &= ~(0x3 << 30);
    writel(val, GPMI_CTRL0_ADDR);

    // Use the failsafe timings and default 24MHz clock
    nand_GpmiSetNandTiming((NAND_Timing_t *) & zFailsafeTimings, 0);

    // Configure all of the pads that will be used for GPMI.
    nand_ConfigurePinmux();

    // Put GPMI in NAND mode, keep DEVICE reset enabled, and make certain
    // polarity is active high
    HW_GPMI_CTRL1_WR(BF_GPMI_CTRL1_DEV_RESET
		     (BV_GPMI_CTRL1_DEV_RESET__DISABLED) |
		     BF_GPMI_CTRL1_ATA_IRQRDY_POLARITY
		     (BV_GPMI_CTRL1_ATA_IRQRDY_POLARITY__ACTIVEHIGH) |
		     BW_GPMI_CTRL1_GPMI_MODE
		     (BV_GPMI_CTRL1_GPMI_MODE__NAND));

    return SUCCESS;
}

// for BCH
void nand_SetFlashLayout(unsigned int u32NandDeviceNumber,
			 NAND_ECC_Params_t * pnandecc)
{
    unsigned int val;
    if (u32NandDeviceNumber == 0) {
	// for nand0

	// set flash0layout0 bch ecc register
#if 0
	BW_BCH_FLASH0LAYOUT0_NBLOCKS(pReadSeed->
				     zNANDEccParams.m_u32NumEccBlocksPerPage);
	BW_BCH_FLASH0LAYOUT0_META_SIZE(pReadSeed->
				       zNANDEccParams.m_u32MetadataBytes);
	BW_BCH_FLASH0LAYOUT0_ECC0(pReadSeed->
				  zNANDEccParams.m_u32EccBlock0EccLevel /
				  2);
	BW_BCH_FLASH0LAYOUT0_DATA0_SIZE(pReadSeed->
					zNANDEccParams.m_u32EccBlock0Size);
#else
	val = (pnandecc->m_u32EccBlock0Size & 0xfff) |
	    (((pnandecc->m_u32EccBlock0EccLevel / 2) & 0xf) << 12) |
	    ((pnandecc->m_u32MetadataBytes & 0xff) << 16) |
	    ((pnandecc->m_u32NumEccBlocksPerPage & 0xff) << 24);
	nfc_printf(NFC_DEBUG_MAX, "BCH Flash layout 0: 0x%x\n", val);
	writel(val, BCH_FLASH0_LAYOUT0_ADDR);
#endif

	// set flash0layout1 bch ecc register
#if 0
	BW_BCH_FLASH0LAYOUT1_PAGE_SIZE(pReadSeed->
				       zNANDEccParams.m_u32PageSize);
	BW_BCH_FLASH0LAYOUT1_ECCN(pReadSeed->zNANDEccParams.u32EccType /
				  2);
	BW_BCH_FLASH0LAYOUT1_DATAN_SIZE(pReadSeed->
					zNANDEccParams.m_u32EccBlockNSize);

#else
	val = (pnandecc->m_u32EccBlockNSize & 0xfff) |
	    (((pnandecc->u32EccType / 2) & 0xf) << 12) |
	    ((pnandecc->m_u32PageSize & 0xffff) << 16);

	nfc_printf(NFC_DEBUG_MAX, "BCH Flash layout 1: 0x%x\n", val);

	writel(val, BCH_FLASH0_LAYOUT1_ADDR);
#endif

    } else if (u32NandDeviceNumber == 1) {
	// for nand1
	/*
	   // set flash1layout0 bch ecc register
	   BW_BCH_FLASH1LAYOUT0_NBLOCKS(pReadSeed->zNANDEccParams.m_u32NumEccBlocksPerPage);
	   BW_BCH_FLASH1LAYOUT0_META_SIZE(pReadSeed->zNANDEccParams.m_u32MetadataBytes);
	   BW_BCH_FLASH1LAYOUT0_ECC0(pReadSeed->zNANDEccParams.m_u32EccBlock0EccLevel/2);
	   BW_BCH_FLASH1LAYOUT0_DATA0_SIZE(pReadSeed->zNANDEccParams.m_u32EccBlock0Size);

	   // set flash1layout1 bch ecc register
	   BW_BCH_FLASH1LAYOUT1_PAGE_SIZE(pReadSeed->zNANDEccParams.m_u32PageSize);
	   BW_BCH_FLASH1LAYOUT1_ECCN(pReadSeed->zNANDEccParams.u32EccType/2);
	   BW_BCH_FLASH1LAYOUT1_DATAN_SIZE(pReadSeed->zNANDEccParams.m_u32EccBlockNSize);

	 */
    }
    // Set flash layoutselect
    //writel(0x0, 0x8000a070);
}

void nand_ResetECC8(void)
{
    unsigned int val;
    // Bring out of reset and disable Clk gate.
    // Soft Reset the ECC8 block
    val = readl(ECC8_CTRL_ADDR);
    val |= (1 << 31);
    writel(val, ECC8_CTRL_ADDR);
    val &= ~(1 << 31);
    writel(val, ECC8_CTRL_ADDR);

    // Now bring out of reset and disable Clk gate.
    val = readl(ECC8_CTRL_ADDR);
    val |= (1 << 30);
    writel(val, ECC8_CTRL_ADDR);
    val &= ~(1 << 30);
    writel(val, ECC8_CTRL_ADDR);
    // Set the AHBM soft reset.
    val = readl(ECC8_CTRL_ADDR);
    val |= (1 << 29);
    writel(val, ECC8_CTRL_ADDR);
    val &= ~(1 << 29);
    writel(val, ECC8_CTRL_ADDR);

}

void nand_ResetBCH(void)
{
    unsigned int val;
    // Bring out of reset and disable Clk gate.
    // Soft Reset the BCH block
    val = readl(BCH_CTRL_ADDR);
    val |= (1 << 31);
    writel(val, BCH_CTRL_ADDR);
    val &= ~(1 << 31);
    writel(val, BCH_CTRL_ADDR);
    // Now bring out of reset and disable Clk gate.
    val = readl(BCH_CTRL_ADDR);
    val |= (1 << 30);
    writel(val, BCH_CTRL_ADDR);
    val &= ~(1 << 30);
    writel(val, BCH_CTRL_ADDR);
}

void nand_ResetDma(NAND_dma_reset_device_t * pChain,
		   unsigned int u32NandDeviceNumber)
{
    // First we want to wait for Ready.  The chip may be busy on power-up.
    // Wait for Ready.
    pChain->wait4rdy_dma.nxt =
	(apbh_dma_gpmi1_t *) & (pChain->sense_rdy_dma);
    pChain->wait4rdy_dma.cmd.U = NAND_DMA_WAIT4RDY_CMD;
    // BAR points to alternate branch if timeout occurs.
    pChain->wait4rdy_dma.bar =
	(apbh_dma_gpmi1_t *) & (pChain->timeout_dma);
    // Set GPMI wait for ready.
    pChain->wait4rdy_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.
	U = NAND_DMA_WAIT4RDY_PIO(u32NandDeviceNumber);

    // Now check for successful Ready.
    pChain->sense_rdy_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->tx_dma);
    pChain->sense_rdy_dma.cmd.U = NAND_DMA_SENSE_CMD(0);
    // BAR points to alternate branch if timeout occurs.
    pChain->sense_rdy_dma.bar =
	(apbh_dma_gpmi1_t *) & (pChain->timeout_dma);
    // Even though PIO is unused, set it to zero for comparison purposes.
    pChain->sense_rdy_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.
	U = 0;

    // Next command will be a wait.
    pChain->tx_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->wait_dma);
    // Configure APBH DMA for NAND Reset command.
    pChain->tx_dma.cmd.U =
	NAND_DMA_COMMAND_CMD(NAND_RESET_DEVICE_SIZE, 0, NAND_LOCK, 3);

    // Buffer Address Register being used to hold command.
    pChain->tx_dma.bar = pChain->tx_reset_command_buf;
    // Setup GPMI bus for Reset Command.  Need to set CLE high, then
    // low, then ALE toggles high and low.  (Actually, in this case
    // ALE toggling probably isn't necessary)
    pChain->tx_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_ctrl0.U =
	NAND_DMA_COMMAND_PIO(u32NandDeviceNumber,
			     NAND_RESET_DEVICE_SIZE, 0, ASSERT_CS);
    // Nothing needs to happen to the compare.
    pChain->tx_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_compare.U =
	(unsigned int) NULL;
    // Disable the ECC.
    pChain->tx_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_eccctrl.U =
	NAND_DMA_ECC_PIO(BV_GPMI_ECCCTRL_ENABLE_ECC__DISABLE);

    // Setup 2nd complete DMA sequence.
    // Wait for Ready.
    pChain->wait_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->sense_dma);
    pChain->wait_dma.cmd.U = NAND_DMA_WAIT4RDY_CMD;
    // BAR points to alternate branch if timeout occurs.
    pChain->wait_dma.bar = (apbh_dma_gpmi1_t *) 0x00;
    // Set GPMI wait for ready.
    pChain->wait_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U =
	NAND_DMA_WAIT4RDY_PIO(u32NandDeviceNumber);

    // Now check for success.
    pChain->sense_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->success_dma);
    // Decrement semaphore.
    pChain->sense_dma.cmd.U = NAND_DMA_SENSE_CMD(0);
    // BAR points to alternate branch if timeout occurs.
    pChain->sense_dma.bar = (apbh_dma_gpmi1_t *) & (pChain->timeout_dma);
    // Even though PIO is unused, set it to zero for comparison purposes.
    pChain->sense_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U =
	0;

    // Initialize the Terminator functions
    // Next function is null.
    pChain->success_dma.nxt = (apbh_dma_t *) 0x0;
    // Decrement semaphore, set IRQ, no DMA transfer.
    pChain->success_dma.cmd.U = ((unsigned int)
				 (BF_APBH_CHn_CMD_IRQONCMPLT(1) |
				  BF_APBH_CHn_CMD_SEMAPHORE(1) |
				  BV_FLD(APBH_CHn_CMD, COMMAND,
					 NO_DMA_XFER)));
    // BAR points to success termination code.
    pChain->success_dma.bar = (void *) SUCCESS;

    // Next function is null.
    pChain->timeout_dma.nxt = (apbh_dma_t *) 0x0;
    // Decrement semaphore, set IRQ, no DMA transfer.
    pChain->timeout_dma.cmd.U = ((unsigned int)
				 (BF_APBH_CHn_CMD_IRQONCMPLT(1) |
				  BF_APBH_CHn_CMD_SEMAPHORE(1) |
				  BV_FLD(APBH_CHn_CMD, COMMAND,
					 NO_DMA_XFER)));
    // BAR points to timeout termination code.
    pChain->timeout_dma.bar = (void *) 0x80508008;

}

void nand_StartDma(void *pDmaChain, unsigned int u32NandDeviceNumber)
{
    unsigned int val;
    unsigned int channel_mask =
	(0x1 << (NAND0_APBH_CH + u32NandDeviceNumber));

    // soft reset dma chan, load cmd pointer and inc semaphore
    val = readl(APBH_DMA_CTRL_ADDR);
    val &= (0xff << 16);
    val |= (channel_mask << 16);
    writel(val, APBH_DMA_CTRL_ADDR);

    // Clear IRQ
    writel(channel_mask, APBH_DMA_CTRL1_CLR);

#if 0
    // Enable IRQ
    val = readl(APBH_DMA_CTRL1_ADDR);
    val |= (channel_mask << 16);
    writel(val, APBH_DMA_CTRL1_ADDR);
#endif

    HW_APBH_CHn_NXTCMDAR_WR(NAND0_APBH_CH + u32NandDeviceNumber,
			    (unsigned int) pDmaChain);

    // Start DMA by incrementing the semaphore.
    BW_APBH_CHn_SEMA_INCREMENT_SEMA(NAND0_APBH_CH +
				    u32NandDeviceNumber, 1);

}

int nand_WaitDma(unsigned int u32uSecTimeout,
		 unsigned int u32NandDeviceNumber)
{
    unsigned int val;
    int iComplete;
    unsigned int channel_mask =
	(0x1 << (NAND0_APBH_CH + u32NandDeviceNumber));

    // end of DMA chain will set IRQ.
    do {
	iComplete = readl(APBH_DMA_CTRL1_ADDR) & (channel_mask);
    } while ((iComplete == 0));	// && ((readl(0x80004240) &(0xff<<16))!=0)

    // if timeout return error, else return NXTCMDAR field from last DMA command
    if (iComplete == 0) {
	// abort dma by resetting channel
	val = readl(APBH_DMA_CTRL_ADDR);
	val &= (0xff << 16);
	val |= (channel_mask << 16);
	writel(val, APBH_DMA_CTRL_ADDR);
	return 0x80508009;
    } else {
	val = readl(APBH_DMA_CH4_BAR);
	nfc_printf(NFC_DEBUG_MED, "APBH_DMA_CH4_BAR: 0x%x\n", val);
	return val;
    }
}


void nand_BuildReadStatusDma(NAND_dma_read_status_t * pChain,
			     unsigned int u32NandDeviceNumber,
			     void *pBuffer)
{
    pChain->tx_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->rx_dma);
    // Configure APBH DMA to push CheckStatus command (toggling CLE)
    // into GPMI_CTRL.
    // Transfer NAND_READ_STATUS_SIZE (1) bytes to GPMI when GPMI ready.
    // Wait for end command from GPMI before next part of chain.
    // Lock GPMI to this NAND during transfer.
    // DMA_READ - Perform PIO word transfers then transfer
    //            from memory to peripheral for specified # of bytes.
    pChain->tx_dma.cmd.U =
	NAND_DMA_COMMAND_CMD(NAND_READ_STATUS_SIZE, 0, NAND_LOCK, 3);
    // Point to byte where NAND Read Status Command is kept.
    pChain->tx_dma.bar = &(pChain->tx_cle1);
    // Setup GPMI bus for first part of Read Status Command.  Need to
    // set CLE high, then send Read Status command (0x70/71), then
    // clear CLE.
    pChain->tx_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_ctrl0.U =
	NAND_DMA_COMMAND_PIO(u32NandDeviceNumber,
			     NAND_READ_STATUS_SIZE,
			     BV_GPMI_CTRL0_ADDRESS_INCREMENT__DISABLED,
			     ASSERT_CS);

    // Set compare to NULL.
    pChain->tx_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_compare.U =
	(unsigned int) NULL;
    // Disable the ECC.
    pChain->tx_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_eccctrl.U =
	NAND_DMA_ECC_PIO(BV_GPMI_ECCCTRL_ENABLE_ECC__DISABLE);

    // Next dma chain is SUCCESS.
    //pChain->rx_dma.nxt = (apbh_dma_gpmi1_t*)&APBH_SUCCESS_DMA;
    pChain->rx_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->success_dma);
    // Read back 1 word.
    //pChain->rx_dma.cmd.U = NAND_DMA_RX_CMD(NAND_READ_STATUS_RESULT_SIZE,
    //                                           DECR_SEMAPHORE);
    pChain->rx_dma.cmd.U =
	NAND_DMA_RX_NO_ECC_CMD(NAND_READ_STATUS_RESULT_SIZE, 0);
    // Put result into pBuffer.
    pChain->rx_dma.bar = pBuffer;
    // Read NAND_STATUS_SIZE bytes from GPMI.
    pChain->rx_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U =
	NAND_DMA_RX_PIO(u32NandDeviceNumber,
			BV_GPMI_CTRL0_WORD_LENGTH__8_BIT,
			NAND_READ_STATUS_RESULT_SIZE);

    // Initialize the Terminator functions
    // Next function is null.
    pChain->success_dma.nxt = (apbh_dma_t *) 0x0;
    // Decrement semaphore, set IRQ, no DMA transfer.
    pChain->success_dma.cmd.U = ((unsigned int)
				 (BF_APBH_CHn_CMD_IRQONCMPLT(1) |
				  BF_APBH_CHn_CMD_WAIT4ENDCMD(1) |
				  BF_APBH_CHn_CMD_SEMAPHORE(1) |
				  BV_FLD(APBH_CHn_CMD, COMMAND,
					 NO_DMA_XFER)));
    // BAR points to success termination code.
    pChain->success_dma.bar = (void *) SUCCESS;
}


// send a reset command to nand
int nand_Reset(void)
{
    int retCode;
    unsigned int u32BusyTimeout;
    NAND_dma_reset_device_t *pdma_reset_device;
    NAND_dma_reset_device_t *phy_dma_reset_device;

    pdma_reset_device = (NAND_dma_reset_device_t *) RSVBUFFADDR;

    // Set GPMI DMA timeout for ~1msec in preparation for Reset.
    u32BusyTimeout = 6;
    writel(u32BusyTimeout << 16, GPMI_TIMEOUT1_ADDR);

    // convert from virtual address to physical address
    HAL_VIRT_TO_PHYS_ADDRESS(pdma_reset_device, phy_dma_reset_device);

    // Load the reset command
    // Most devices have the same reset command (0xFF)
    pdma_reset_device->tx_reset_command_buf[0] = 0xff;

    // Build the Reset Device DMA chain.
    nand_ResetDma(phy_dma_reset_device, 0);

    // Kick it off.
    nand_StartDma((dma_cmd_t *) phy_dma_reset_device, 0);

    retCode = nand_WaitDma(MAX_TRANSACTION_TIMEOUT, 0);

    if (retCode != SUCCESS) {
	printf("Reset Failed.\n");
	return 0x80508009;
    }

    u32BusyTimeout = 0x132;	//default
    writel(u32BusyTimeout << 16, GPMI_TIMEOUT1_ADDR);

    return (retCode);		// Success or Failure?
}


int nand_Init(ReadIDCode * pReadIDBuf)
{
    int status;
    unsigned int val;
    unsigned int id;
    unsigned int u32NandDeviceNumber = 0;
    ReadIDCode *phy_ReadIDBuf;
    unsigned int u32NumberOfECCbytes;

    nand_EnableGPMI();

    // Now reset the bch block.
    nand_ResetBCH();

    // Take the APBH out of reset.
    // APBH - disable reset, enable clock.
    val = readl(APBH_DMA_CTRL_ADDR);
    val &= ~(0x3 << 30);
    writel(val, APBH_DMA_CTRL_ADDR);

    // Reset the APBH NAND channels and clr IRQs
    val = readl(APBH_DMA_CTRL_ADDR);
    val &= ~(0xff << 16);
    val |= (0x10 << 16);	// reset nand0 channel
    writel(val, APBH_DMA_CTRL_ADDR);

    writel(0xf0, APBH_DMA_CTRL1_CLR);

    // Reset the NAND so we're in a known state.
    // Don't return a failure because we may still be able to boot from the
    // other nands.  The reset is not a good indicator of whether
    // a NAND is present.
    nfc_printf(NFC_DEBUG_MED, "============NAND Reset========\n");
    status = nand_Reset();
    if (status != SUCCESS)
	return FAIL;

    HAL_VIRT_TO_PHYS_ADDRESS(pReadIDBuf, phy_ReadIDBuf);

    nand_ReadID(0, phy_ReadIDBuf);
    printf("Manuf ID: 0x%X\n",
	   pReadIDBuf->DeviceID_Code.Device_Code.btManufacturerCode);
    printf("Device ID: 0x%X\n",
	   pReadIDBuf->DeviceID_Code.Device_Code.btDeviceCode);

    // Configure ECC parameters
    // BCH is used
    val = readl(GPMI_CTRL1_ADDR);
    val |= (1 << 18);
    writel(val, GPMI_CTRL1_ADDR);

    //set erase threshold, SLC as 0x0
    writel(0x0, BCH_MODE_ADDR);

    /* Configure BCH flash layout */
    nand_ecc_params.m_u32EccBlock0EccLevel = 8;
    nand_ecc_params.m_u32EccBlock0Size = 0;
    nand_ecc_params.m_u32EccBlockNSize = 512;
    nand_ecc_params.m_u32EraseThreshold = 0;
    nand_ecc_params.m_u32MetadataBytes = 8;
    nand_ecc_params.m_u32NumEccBlocksPerPage = 8;
    nand_ecc_params.u32ECCEngine = 1;
    nand_ecc_params.u32EccType = 8;

    // Calculate ecc bytes for 1 page from zNANDEccParams,
    // Block0 might have separate ecc level than other blocks.
    // Formula for calculating number of parity bits for each block is (ecc_level * 13)
    u32NumberOfECCbytes = (nand_ecc_params.m_u32EccBlock0EccLevel * 13) +	// block0
	(nand_ecc_params.m_u32NumEccBlocksPerPage *	// blockN
	 nand_ecc_params.u32EccType * 13);

    nand_ecc_params.m_u32PageSize = NF_PG_SZ +
	nand_ecc_params.m_u32MetadataBytes + u32NumberOfECCbytes;


    nand_SetFlashLayout(u32NandDeviceNumber, &nand_ecc_params);

    return SUCCESS;
}


void nand_BuildReadDma(NAND_dma_read_t * pChain,
		       unsigned int u32NandDeviceNumber,
		       NAND_read_seed_t * pReadSeed)
{
    // CLE1 chain size is # columns + # Rows + CLE command.
    unsigned int iCLE1_Size = pReadSeed->uiAddressSize + 1;

    // Send the 2nd CLE.
    pChain->tx_cle1_addr_dma.nxt =
	(apbh_dma_gpmi1_t *) & (pChain->tx_cle2_dma);

    // Configure APBH DMA to push Read command (toggling CLE)
    // into GPMI_CTRL.
    // Transfer CLE1_SIZE (5) bytes to GPMI when GPMI ready.
    // Transfer CLE1 and 4 ADDRESS bytes to GPMI_CTRL0 (see command below)
    // Wait for end command from GPMI before next part of chain.
    // Lock GPMI to this NAND during transfer.
    // DMA_READ - Perform PIO word transfers then transfer
    //            from memory to peripheral for specified # of bytes.
    pChain->tx_cle1_addr_dma.cmd.U =
	NAND_DMA_COMMAND_CMD(iCLE1_Size, 0, NAND_LOCK, 3);
    // Buffer Address Register holds Read Address command.
    //pChain->tx_cle1_addr_dma.bar = pReadSeed->tx_cle1_addr_buf;
    pChain->tx_cle1_addr_dma.bar =
	pReadSeed->tx_cle1_addr_dma_buffer.tx_cle1_addr_buf;
    // Setup GPMI bus for first part of Read Command.  Need to set CLE
    // high, then send Read command (0x00), then clear CLE, set ALE high
    // send # address bytes (Column then row) [Type1=2; Type2 = 4].
    pChain->tx_cle1_addr_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_ctrl0.U =
	NAND_DMA_COMMAND_PIO(u32NandDeviceNumber, iCLE1_Size,
			     BV_GPMI_CTRL0_ADDRESS_INCREMENT__ENABLED,
			     ASSERT_CS);

    pChain->tx_cle1_addr_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_compare.U = (unsigned int) NULL;

    pChain->tx_cle1_addr_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_eccctrl.U =
	NAND_DMA_ECC_PIO(BV_GPMI_ECCCTRL_ENABLE_ECC__DISABLE);

    // Setup next command - wait.
    pChain->tx_cle2_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->wait_dma);
    // Configure APBH DMA to push 2nd Read command (toggling CLE)
    // into GPMI_CTRL.
    // Transfer CLE2_SIZE (1) bytes to GPMI when GPMI ready.
    // Transfer CLE2 byte to GPMI_CTRL0 (see command below)
    // Wait for end command from GPMI before next part of chain.
    // Lock GPMI to this NAND during transfer.
    // DMA_READ - Perform PIO word transfers then transfer
    //            from memory to peripheral for specified # of bytes.

    pChain->tx_cle2_dma.cmd.U = NAND_DMA_COMMAND_CMD(1, 0, NAND_LOCK, 1);

    // Buffer Address Register holds tx_cle2 command
    pChain->tx_cle2_dma.bar =
	pReadSeed->tx_cle2_addr_dma_buffer.tx_cle2_buf;
    // Setup GPMI bus for second part of Read Command.  Need to set CLE
    // high, then send Read2 command (0x30), then clear CLE.
    pChain->tx_cle2_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U =
	NAND_DMA_COMMAND_PIO(u32NandDeviceNumber, 1,
			     BV_GPMI_CTRL0_ADDRESS_INCREMENT__DISABLED,
			     ASSERT_CS);
    // tt_todo - does CSLock = 1 cause problem here?

    // Once we've received ready, need to receive data.
    pChain->wait_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->sense_dma);
    // Wait for Ready (No transfer count)
    pChain->wait_dma.cmd.U = NAND_DMA_WAIT4RDY_CMD;
    // If there is an error, load Timeout DMA sequence.
    pChain->sense_dma.bar = (apbh_dma_gpmi1_t *) & (pChain->timeout_dma);
    // Send commands Wait for Ready to go high.
    pChain->wait_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U =
	NAND_DMA_WAIT4RDY_PIO(u32NandDeviceNumber);

    // Now psense to see if a timeout has occurred.
    pChain->sense_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->rx_data_dma);
    // Wait for Ready (No transfer count) - Do not decrement semaphore.
    pChain->sense_dma.cmd.U = NAND_DMA_SENSE_CMD(0);
    // If there is an error, load Timeout DMA sequence.
    pChain->sense_dma.bar = (apbh_dma_gpmi1_t *) & (pChain->timeout_dma);
    // Even though PIO is unused, set it to zero for comparison purposes.
    pChain->sense_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U =
	0;

    // Next step is to disable the ECC.
    pChain->rx_data_dma.nxt =
	(apbh_dma_gpmi1_t *) & pChain->rx_wait4done_dma;

    if (pReadSeed->bEnableHWECC) {
	// Configure APBH DMA to NOT read any bytes from the NAND into
	// memory using GPMI.  The ECC will become the Bus Master and
	// write the read data into memory.
	// Wait for end command from GPMI before next part of chain.
	// Lock GPMI to this NAND during transfer.
	// NO_DMA_XFER - No DMA transfer occurs on APBH - see above.
	// Decrement Semaphore to indicate finished.
	pChain->rx_data_dma.cmd.U = NAND_DMA_RX_CMD_ECC(0, 0);
	// Save Data into buffer.
	pChain->rx_data_dma.bar = 0x00;	// This field isn't used.
	pChain->rx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_compare.U = 0x00;	// This field isn't used.
	// Operate on 4 buffers (2K transfers)  Select which type of Decode - 4 bit or 8 bit.
	if (pReadSeed->zNANDEccParams.u32ECCEngine == 0)	//ECC8
	{
	    pChain->rx_data_dma.apbh_dma_gpmi6_u.
		apbh_dma_gpmi6_ctrl.gpmi_eccctrl.U =
		NAND_DMA_ECC_CTRL_PIO(0x0F,
				      pReadSeed->
				      zNANDEccParams.u32EccType);
	} else {
	    pChain->rx_data_dma.apbh_dma_gpmi6_u.
		apbh_dma_gpmi6_ctrl.gpmi_eccctrl.U =
		NAND_DMA_ECC_CTRL_PIO(pReadSeed->uiECCMask,
				      BV_GPMI_ECCCTRL_ECC_CMD__DECODE_8_BIT);
	}
	pChain->rx_data_dma.apbh_dma_gpmi6_u.
	    apbh_dma_gpmi6_ctrl.gpmi_ecccount.B.COUNT =
	    pReadSeed->uiReadSize;
    } else {
	// ECC is disabled. Configure DMA to write directly to memory.
	// Wait for end command from GPMI before next part of chain.
	// Lock GPMI to this NAND during transfer.
	pChain->rx_data_dma.cmd.U =
	    NAND_DMA_RX_NO_ECC_CMD(pReadSeed->uiReadSize, 0);
	// Save Data into buffer.
	pChain->rx_data_dma.bar = (void *) (((unsigned int) pReadSeed->pDataBuffer) & 0xFFFFFFFC);	// not sure if this is right...
	pChain->rx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_compare.U = 0x00;	// This field isn't used.
	pChain->rx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_eccctrl.U = 0;	// This field isn't used.
	pChain->rx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_ecccount.U = 0;	// This field isn't used.
    }
    // Setup the data buffer.
    pChain->rx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_payload.
	U = (((unsigned int) pReadSeed->pDataBuffer) & 0xFFFFFFFC);
    // And the Auxiliary buffer here.
    pChain->rx_data_dma.apbh_dma_gpmi6_u.
	apbh_dma_gpmi6_ctrl.gpmi_auxiliary.U =
	(((unsigned int) pReadSeed->pAuxBuffer) & 0xFFFFFFFC);
    // Setup GPMI bus for Read Sector Result.  GPMI Read.
    // Read ReadSize words (16 or 8 bit) data
    // Note - althought the GPMI knows more than one byte/word may be
    //        sent, the APBH assumes bytes only.

    pChain->rx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_ctrl0.U =
	NAND_DMA_RX_PIO(u32NandDeviceNumber,
			BV_GPMI_CTRL0_WORD_LENGTH__8_BIT,
			pReadSeed->uiReadSize);

    // Disable the ECC then load Success DMA sequence.
    pChain->rx_wait4done_dma.nxt =
	(apbh_dma_gpmi1_t *) & (pChain->success_dma);
    // Configure to send 3 GPMI PIO reads.
    pChain->rx_wait4done_dma.cmd.U = NAND_DMA_DISABLE_ECC_TRANSFER;
    // Nothing to be sent.
    pChain->rx_wait4done_dma.bar = NULL;
    // Disable the Chip Select and other outstanding GPMI things.
    pChain->rx_wait4done_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_ctrl0.U =
	NAND_DMA_DISABLE_ECC_PIO(u32NandDeviceNumber);
    // Ignore the compare - we need to skip over it.
    pChain->rx_wait4done_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_compare.U = 0x00;
    // Disable the ECC Block.
    pChain->rx_wait4done_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_eccctrl.U =
	BF_GPMI_ECCCTRL_ENABLE_ECC(BV_GPMI_ECCCTRL_ENABLE_ECC__DISABLE);
    // Disable bch mode
    //pChain->rx_wait4done_dma.gpmi_ctrl1.B.BCH_MODE = 0;

    // Initialize the Terminator functions
    // Next function is null.
    pChain->success_dma.nxt = (apbh_dma_t *) 0x0;
    // Decrement semaphore, set IRQ, no DMA transfer.
    pChain->success_dma.cmd.U = ((unsigned int)
				 (BF_APBH_CHn_CMD_IRQONCMPLT(1) |
				  BF_APBH_CHn_CMD_WAIT4ENDCMD(1) |
				  BF_APBH_CHn_CMD_SEMAPHORE(1) |
				  BV_FLD(APBH_CHn_CMD, COMMAND,
					 NO_DMA_XFER)));
    // BAR points to success termination code.
    pChain->success_dma.bar = (void *) SUCCESS;

    // Next function is null.
    pChain->timeout_dma.nxt = (apbh_dma_t *) 0x0;
    // Decrement semaphore, set IRQ, no DMA transfer.
    pChain->timeout_dma.cmd.U = ((unsigned int)
				 (BF_APBH_CHn_CMD_IRQONCMPLT(1) |
				  BF_APBH_CHn_CMD_WAIT4ENDCMD(1) |
				  BF_APBH_CHn_CMD_SEMAPHORE(1) |
				  BV_FLD(APBH_CHn_CMD, COMMAND,
					 NO_DMA_XFER)));
    // BAR points to timeout termination code.
    pChain->timeout_dma.bar = (void *) 0x80508008;

}


////////////////////////////////////////////////////////////////////////////////
//! \brief Build the abbreviated DMA to send a NAND Read Command to the device.
//!
//! This function builds the DMA descriptor for a NAND Read command to the
//! device.  This function assumes the DMA has already been setup once so
//! only the parameters that change need to be updated.
////////////////////////////////////////////////////////////////////////////////
void nand_BuildQuickReadDma(NAND_dma_read_t * pChain,
			    unsigned int u32NandDeviceNumber,
			    NAND_read_seed_t * pReadSeed)
{
    unsigned int uiGPMITransferSize;

    // Setup GPMI bus for first part of Read Command.  Need to set CLE
    // high, then send Read command (0x00), then clear CLE, set ALE high
    // send # address bytes (Column then row) [Type1=3; Type2 = 4].
    // Only thing that needs to be set here is which NAND to talk to.
    //pChain->tx_cle1_addr_dma.gpmi_ctrl0.B.CS = BF_GPMI_CTRL0_CS(u32NandDeviceNumber);
    pChain->tx_cle1_addr_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_ctrl0.B.CS = u32NandDeviceNumber;

    // Setup GPMI bus for second part of Read Command.  Need to set CLE
    // high, then send Read2 command (0x30), then clear CLE.
    // Only thing that needs to be set here is which NAND to talk to.
    //pChain->tx_cle2_dma.gpmi_ctrl0.B.CS = BF_GPMI_CTRL0_CS(u32NandDeviceNumber);
    pChain->tx_cle2_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.B.
	CS = u32NandDeviceNumber;

    // Only thing that needs to be set here is which NAND to talk to.
    //pChain->wait_dma.gpmi_ctrl0.B.CS = BF_GPMI_CTRL0_CS(u32NandDeviceNumber);
    pChain->wait_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.B.CS =
	u32NandDeviceNumber;

    if (pReadSeed->bEnableHWECC) {
	pChain->rx_data_dma.apbh_dma_gpmi6_u.
	    apbh_dma_gpmi6_ctrl.gpmi_eccctrl.B.BUFFER_MASK =
	    pReadSeed->uiECCMask;
	pChain->rx_data_dma.apbh_dma_gpmi6_u.
	    apbh_dma_gpmi6_ctrl.gpmi_ecccount.B.COUNT =
	    pReadSeed->uiReadSize;
    } else {
	// ECC is disabled.
	pChain->rx_data_dma.cmd.B.XFER_COUNT = pReadSeed->uiReadSize;
	pChain->rx_data_dma.bar = pReadSeed->pDataBuffer;
	pChain->rx_data_dma.apbh_dma_gpmi6_u.
	    apbh_dma_gpmi6_ctrl.gpmi_eccctrl.U = 0;
	pChain->rx_data_dma.apbh_dma_gpmi6_u.
	    apbh_dma_gpmi6_ctrl.gpmi_ecccount.U = 0;
    }

    // Setup the data buffer.
    pChain->rx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_payload.
	U = (((unsigned int) pReadSeed->pDataBuffer) & 0xFFFFFFFC);
    // And the Auxiliary buffer here.
    pChain->rx_data_dma.apbh_dma_gpmi6_u.
	apbh_dma_gpmi6_ctrl.gpmi_auxiliary.U =
	(((unsigned int) pReadSeed->pAuxBuffer) & 0xFFFFFFFC);
    // Setup GPMI bus for Read Sector Result.  GPMI Read.
    // Read ReadSize words (16 or 8 bit) data
    // Note - althought the GPMI knows more than one byte/word may be
    //        sent, the APBH assumes bytes only.
    uiGPMITransferSize =
	(pReadSeed->uiWordSize ==
	 BV_GPMI_CTRL0_WORD_LENGTH__8_BIT) ? pReadSeed->uiReadSize
	: (pReadSeed->uiReadSize >> 1);
    // Change only those values that need to be changed.
    //pChain->rx_data_dma.gpmi_ctrl0.B.CS = BF_GPMI_CTRL0_CS(u32NandDeviceNumber);
    //pChain->rx_data_dma.gpmi_ctrl0.B.XFER_COUNT = BF_GPMI_CTRL0_XFER_COUNT(uiGPMITransferSize);
    pChain->rx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_ctrl0.B.
	CS = u32NandDeviceNumber;
    pChain->rx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_ctrl0.B.
	XFER_COUNT = uiGPMITransferSize;

    // Disable the Chip Select and other outstanding GPMI things.
    pChain->rx_wait4done_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_ctrl0.B.CS = u32NandDeviceNumber;
}


void nand_InitReadDma(void *pReadDmaDescriptor,
		      unsigned int u32NumRowBytes,
		      unsigned int u32BusWidth,
		      unsigned int u32ECCSize,
		      unsigned int u32ReadCode1, unsigned int u32ReadCode2)
{
    unsigned int u32NumberOfECCbytes;
    //int32_t iSectorOffset=0;
    NAND_dma_read_t *pDmaReadDescriptor =
	(NAND_dma_read_t *) pReadDmaDescriptor;
    NAND_read_seed_t *pDmaReadSeed =
	(NAND_read_seed_t *) & (pDmaReadDescriptor->NAND_DMA_Read_Seed);
    NAND_read_seed_t *phy_DmaReadSeed;

    HAL_VIRT_TO_PHYS_ADDRESS(pDmaReadSeed, phy_DmaReadSeed);

    /* for BCH */
    // Initialize ReadSeed
    pDmaReadSeed->zNANDEccParams.u32EccType = u32ECCSize;
    // Initialize mask to read from the beginning of a page including metadata
    pDmaReadSeed->uiECCMask = BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE;

    // Calculate ecc bytes for 1 page from zNANDEccParams,
    // Block0 might have separate ecc level than other blocks.
    // Formula for calculating number of parity bits for each block is (ecc_level * 13)
    u32NumberOfECCbytes = (pDmaReadSeed->zNANDEccParams.m_u32EccBlock0EccLevel * 13) +	// block0
	(pDmaReadSeed->zNANDEccParams.m_u32NumEccBlocksPerPage *	// blockN
	 pDmaReadSeed->zNANDEccParams.u32EccType * 13);

    // Convert the result to bytes
    u32NumberOfECCbytes = (u32NumberOfECCbytes + (8 - 1)) / 8;

    nfc_printf(NFC_DEBUG_MAX, "u32NumberOfECCbytes: %d\n",
	       u32NumberOfECCbytes);

    // Alway 2 column bytes.
    pDmaReadSeed->uiAddressSize = MAX_COLUMNS + u32NumRowBytes;

    // Start off assuming we'll only read the data size.
    pDmaReadSeed->uiReadSize =
	(NF_PG_SZ + u32NumberOfECCbytes +
	 pDmaReadSeed->zNANDEccParams.m_u32MetadataBytes);

    // set the Word size
    // default to 8 bit data width
    if (u32BusWidth == 16) {
	pDmaReadSeed->uiWordSize = BV_GPMI_CTRL0_WORD_LENGTH__16_BIT;
    } else {
	pDmaReadSeed->uiWordSize = BV_GPMI_CTRL0_WORD_LENGTH__8_BIT;
    }

    // Setup the Read Data command words.
    pDmaReadSeed->tx_cle1_addr_dma_buffer.
	tx_cle1_addr_Columns_Rows.tx_cle1 = (unsigned char) u32ReadCode1;
    pDmaReadSeed->tx_cle2_addr_dma_buffer.tx_cle2_addr_dma.tx_cle2 =
	(unsigned char) u32ReadCode2;

    // Fill in the Column Address (Always 2 bytes)
    pDmaReadSeed->tx_cle1_addr_dma_buffer.
	tx_cle1_addr_Columns_Rows.tx_cle1_Columns_Rows.tx_cle1_Type2.
	bType2Columns[0] = (unsigned char) (0);
    pDmaReadSeed->tx_cle1_addr_dma_buffer.
	tx_cle1_addr_Columns_Rows.tx_cle1_Columns_Rows.tx_cle1_Type2.
	bType2Columns[1] = (unsigned char) (0);

    // Fill in the Row Address. (Can be 2 or 3 bytes)
    // Fill in the Column Address (Always 2 bytes)
    pDmaReadSeed->tx_cle1_addr_dma_buffer.
	tx_cle1_addr_Columns_Rows.tx_cle1_Columns_Rows.tx_cle1_Type2.
	bType2Rows[0] = (unsigned char) (0);
    pDmaReadSeed->tx_cle1_addr_dma_buffer.
	tx_cle1_addr_Columns_Rows.tx_cle1_Columns_Rows.tx_cle1_Type2.
	bType2Rows[1] = (unsigned char) (0);
    pDmaReadSeed->tx_cle1_addr_dma_buffer.
	tx_cle1_addr_Columns_Rows.tx_cle1_Columns_Rows.tx_cle1_Type2.
	bType2Rows[2] = (unsigned char) (0);

    // Buffer pointers used for DMA chain.
    pDmaReadSeed->pDataBuffer = NULL;
    pDmaReadSeed->pAuxBuffer = NULL;

    nand_BuildReadDma(pDmaReadDescriptor, 0, phy_DmaReadSeed);

}


void nand_BuildProgramDma(NAND_dma_program_t * pChain,
			  unsigned int u32NandDeviceNumber,
			  unsigned int u32AddressSize,
			  unsigned int u32DataSize,
			  unsigned int u32EccSize, void *pWriteBuffer,
			  void *pAuxBuffer)
{
    unsigned int u32EccDataSize = 0;

    // CLE1 chain size is # columns + # Rows + CLE command.
    unsigned int iCLE1_Size = u32AddressSize + 1;

    nfc_printf(NFC_DEBUG_MAX, "u32AddressSize: 0x%x\n", u32AddressSize);
    nfc_printf(NFC_DEBUG_MAX, "u32DataSize: 0x%x\n", u32DataSize);
    nfc_printf(NFC_DEBUG_MAX, "u32EccSize: 0x%x\n", u32EccSize);
    nfc_printf(NFC_DEBUG_MAX, "pAuxBuffer: 0x%x\n",
	       (unsigned int) pAuxBuffer);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Descriptor1: Issue NAND write setup command
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    pChain->tx_cle1_addr_dma.nxt =
	(apbh_dma_gpmi1_t *) (&pChain->tx_data_dma);
    pChain->tx_cle1_addr_dma.cmd.U =
	NAND_DMA_COMMAND_CMD(iCLE1_Size, 0, NAND_LOCK, 3) | (1 << 8);
    pChain->tx_cle1_addr_dma.bar = pChain->NandProgSeed.tx_cle1_addr_buf;
    pChain->tx_cle1_addr_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_ctrl0.U =
	NAND_DMA_COMMAND_PIO(u32NandDeviceNumber, iCLE1_Size,
			     BV_GPMI_CTRL0_ADDRESS_INCREMENT__ENABLED,
			     ASSERT_CS);
    pChain->tx_cle1_addr_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_compare.U = (unsigned int) NULL;
    // Disable the ECC.
    pChain->tx_cle1_addr_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_eccctrl.U =
	NAND_DMA_ECC_PIO(BV_GPMI_ECCCTRL_ENABLE_ECC__DISABLE);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Descriptor2: write the data payload
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    pChain->tx_data_dma.nxt = (apbh_dma_gpmi1_t *) (&pChain->tx_cle2_dma);
    // Calculate the ECC Mask for this transaction.
    // Auxilliary = 0x100 set to request transfer to/from the Auxiliary buffer.
    // Buffer7 = 0x080 set to request transfer to/from buffer7.
    // Buffer6 = 0x040 set to request transfer to/from buffer6.
    // Buffer5 = 0x020 set to request transfer to/from buffer5.
    // Buffer4 = 0x010 set to request transfer to/from buffer4.
    // Buffer3 = 0x008 set to request transfer to/from buffer3.
    // Buffer2 = 0x004 set to request transfer to/from buffer2.
    // Buffer1 = 0x002 set to request transfer to/from buffer1.
    // Buffer0 = 0x001 set to request transfer to/from buffer0.
    // First calculate how many 512 byte buffers fit in here.

    // Set DMA command.
    pChain->tx_data_dma.cmd.U = NAND_DMA_TXDATA_CMD(0, 0, 6, 1) | (1 << 8);

    // Set Buffer Address Register to WriteBuffer.
    pChain->tx_data_dma.bar = (void *) pWriteBuffer;

    pChain->tx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_ctrl0.U =
	NAND_DMA_TXDATA_PIO(u32NandDeviceNumber,
			    BV_GPMI_CTRL0_WORD_LENGTH__8_BIT, 0);
    // Compare isn't used.
    pChain->tx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_compare.
	U = 0;

    pChain->tx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_eccctrl.
	U =
	NAND_DMA_ECC_CTRL_PIO(0x1ff,
			      BV_GPMI_ECCCTRL_ECC_CMD__ENCODE_8_BIT);
    pChain->tx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_ecccount.
	U = u32DataSize;
    // Setup the data buffer.
    pChain->tx_data_dma.apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_payload.
	U = ((unsigned int) (pWriteBuffer) & 0xFFFFFFFC);
    // And the Auxiliary buffer here.
    pChain->tx_data_dma.apbh_dma_gpmi6_u.
	apbh_dma_gpmi6_ctrl.gpmi_auxiliary.U =
	((unsigned int) (pAuxBuffer) & 0xFFFFFFFC);



    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Descriptor3: issue NAND write execute command
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Current Action - Send CLE2 to the NAND.
    // Next Action - Wait for Write to complete.
    pChain->tx_cle2_dma.nxt = (apbh_dma_gpmi1_t *) (&pChain->wait_dma);
    pChain->tx_cle2_dma.cmd.U =
	NAND_DMA_COMMAND_CMD(1, 0, NAND_LOCK, 3) | (1 << 8);;
    pChain->tx_cle2_dma.bar = pChain->NandProgSeed.tx_cle2_buf;
    pChain->tx_cle2_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_ctrl0.U =
	NAND_DMA_COMMAND_PIO(u32NandDeviceNumber, 1,
			     BV_GPMI_CTRL0_ADDRESS_INCREMENT__DISABLED,
			     ASSERT_CS);

    // Set compare to NULL.
    pChain->tx_cle2_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_compare.
	U = (unsigned int) NULL;
    // Disable the ECC.
    pChain->tx_cle2_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_eccctrl.U = 0;	//NAND_DMA_ECC_PIO(BV_GPMI_ECCCTRL_ENABLE_ECC__DISABLE);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Descriptor4: wait for ready
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    pChain->wait_dma.nxt = (apbh_dma_gpmi1_t *) (&pChain->sense_dma);
    pChain->wait_dma.cmd.U = NAND_DMA_WAIT4RDY_CMD;
    pChain->wait_dma.bar = 0x00;
    pChain->wait_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U =
	NAND_DMA_WAIT4RDY_PIO(u32NandDeviceNumber);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Descriptor5: psense compare
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    pChain->sense_dma.nxt = (apbh_dma_t *) (&pChain->statustx_dma);
    pChain->sense_dma.cmd.U = NAND_DMA_SENSE_CMD(0);
    pChain->sense_dma.bar = (apbh_dma_t *) (&pChain->program_failed_dma);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Descriptor6: issue NAND status command
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    pChain->statustx_dma.nxt =
	(apbh_dma_gpmi1_t *) (&pChain->statusrx_dma);
    pChain->statustx_dma.cmd.U =
	NAND_DMA_COMMAND_CMD(NAND_READ_STATUS_SIZE, 0, NAND_LOCK,
			     3) | (1 << 8);;
    pChain->statustx_dma.bar = &pChain->NandProgSeed.u8StatusCmd;
    pChain->statustx_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_ctrl0.
	U =
	NAND_DMA_COMMAND_PIO(u32NandDeviceNumber, NAND_READ_STATUS_SIZE,
			     BV_GPMI_CTRL0_ADDRESS_INCREMENT__DISABLED,
			     ASSERT_CS);


    // Set compare to NULL.
    pChain->statustx_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_compare.
	U = (unsigned int) NULL;
    // Disable the ECC.
    pChain->statustx_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_eccctrl.U = 0;	//NAND_DMA_ECC_PIO(BV_GPMI_ECCCTRL_ENABLE_ECC__DISABLE);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Descriptor7: read status and compare
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    pChain->statusrx_dma.nxt =
	(apbh_dma_gpmi2_t *) (&pChain->statbranch_dma);
    pChain->statusrx_dma.cmd.U =
	NAND_DMA_RX_NO_ECC_CMD(NAND_READ_STATUS_RESULT_SIZE,
			       0) | (1 << 8);;
    pChain->statusrx_dma.bar = (&pChain->NandProgSeed.u16Status);
    pChain->statusrx_dma.gpmi_ctrl0.U =
	NAND_DMA_RX_PIO(u32NandDeviceNumber,
			BV_GPMI_CTRL0_WORD_LENGTH__8_BIT,
			NAND_READ_STATUS_RESULT_SIZE);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Descriptor8: psense compare
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    pChain->statbranch_dma.nxt = (apbh_dma_t *) (&pChain->success_dma);
    pChain->statbranch_dma.cmd.U = NAND_DMA_SENSE_CMD(0);
    pChain->statbranch_dma.bar =
	(apbh_dma_t *) (&pChain->program_failed_dma);


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Descriptor9: emit GPMI interrupt
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Initialize the Terminator functions
    // Next function is null.
    pChain->success_dma.nxt = (apbh_dma_t *) 0x0;
    // Decrement semaphore, set IRQ, no DMA transfer.
    pChain->success_dma.cmd.U = ((unsigned int)
				 (BF_APBH_CHn_CMD_IRQONCMPLT(1) |
				  BF_APBH_CHn_CMD_WAIT4ENDCMD(1) |
				  BF_APBH_CHn_CMD_SEMAPHORE(1) |
				  BV_FLD(APBH_CHn_CMD, COMMAND,
					 NO_DMA_XFER)));
    // BAR points to TRUE termination code.
    pChain->success_dma.bar = (void *) 0;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Next function is null.
    pChain->program_failed_dma.nxt = (apbh_dma_t *) 0x0;
    // Decrement semaphore, set IRQ, no DMA transfer.
    pChain->program_failed_dma.cmd.U = ((unsigned int)
					(BF_APBH_CHn_CMD_IRQONCMPLT(1)
					 | BF_APBH_CHn_CMD_WAIT4ENDCMD(1)
					 | BF_APBH_CHn_CMD_SEMAPHORE(1)
					 | BV_FLD(APBH_CHn_CMD,
						  COMMAND, NO_DMA_XFER)));
    // BAR points to timeout termination code.
    pChain->program_failed_dma.bar = (void *) 0x80508008;

#if 0
    printf("pChain->tx_cle1_addr_dma addr: 0x%x\n",
	   (unsigned int) &pChain->tx_cle1_addr_dma);
    printf("pChain->tx_cle1_addr_dma.nxt: 0x%x\n",
	   (unsigned int) pChain->tx_cle1_addr_dma.nxt);
    printf("pChain->tx_cle1_addr_dma.cmd: 0x%x\n",
	   (unsigned int) pChain->tx_cle1_addr_dma.cmd.U);
    printf("pChain->tx_cle1_addr_dma.bar: 0x%x\n",
	   (unsigned int) pChain->tx_cle1_addr_dma.bar);
    printf("bar data: 0x%x\n",
	   *(unsigned int *) pChain->tx_cle1_addr_dma.bar);
    printf("pChain->tx_cle1_addr_dma ctrl0: 0x%x\n",
	   (unsigned int) pChain->tx_cle1_addr_dma.apbh_dma_gpmi3_u.
	   apbh_dma_gpmi3_ctrl.gpmi_ctrl0.U);
    printf("pChain->tx_cle1_addr_dma eccctrl: 0x%x\n",
	   (unsigned int) pChain->tx_cle1_addr_dma.apbh_dma_gpmi3_u.
	   apbh_dma_gpmi3_ctrl.gpmi_eccctrl.U);


    printf("pChain->tx_data_dma addr: 0x%x\n",
	   (unsigned int) &pChain->tx_data_dma);
    printf("pChain->tx_data_dma.nxt: 0x%x\n",
	   (unsigned int) pChain->tx_data_dma.nxt);
    printf("pChain->tx_data_dma.cmd: 0x%x\n",
	   (unsigned int) pChain->tx_data_dma.cmd.U);
    printf("pChain->tx_data_dma.bar: 0x%x\n",
	   (unsigned int) pChain->tx_data_dma.bar);
    printf("pChain->tx_data_dma ctrl0: 0x%x\n",
	   (unsigned int) pChain->tx_data_dma.
	   apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_ctrl0.U);
    printf("pChain->tx_data_dma counter: 0x%x\n",
	   (unsigned int) pChain->tx_data_dma.
	   apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_ecccount.U);
    printf("pChain->tx_data_dma eccctrl: 0x%x\n",
	   (unsigned int) pChain->tx_data_dma.
	   apbh_dma_gpmi6_u.apbh_dma_gpmi6_ctrl.gpmi_eccctrl.U);

    printf("pChain->tx_cle2_dma eccctrl: 0x%x\n",
	   (unsigned int) pChain->tx_cle2_dma.
	   apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_eccctrl.U);

    printf("pChain->statbranch_dma addr: 0x%x\n",
	   (unsigned int) &pChain->statbranch_dma);
    printf("pChain->success_dma addr: 0x%x\n",
	   (unsigned int) &pChain->success_dma);
    printf("pChain->program_failed_dma addr: 0x%x\n",
	   (unsigned int) &pChain->program_failed_dma);

    printf("pChain->statusrx_dma.gpmi_ctrl0: 0x%x\n",
	   (unsigned int) pChain->statusrx_dma.gpmi_ctrl0.U);
    printf("pChain->sense_dma.bar: 0x%x\n",
	   (unsigned int) pChain->sense_dma.bar);

#endif


}

////////////////////////////////////////////////////////////////////////////////
//! \brief      Build the Erase Block DMA descriptor for the NAND.
//!
//! \fntype     Non-Reentrant
//!
//! Build descriptor to erase the block.  This is a synchronous call.
//!
//! \param[in]  pChain - pointer to the descriptor chain that gets filled.
//! \param[in]  u32BlockAddressBytes TBD
//! \param[in]  u32ChipSelect - Chip Select - NANDs 0-3.
//!
//! \note       branches to TRUE DMA upon completion.
//!
//! \todo [PUBS] Define TBD parameter(s)
////////////////////////////////////////////////////////////////////////////////
void nand_BuildEraseDma(NAND_dma_block_erase_t * pChain,
			unsigned int u32NandDeviceNumber,
			unsigned int u32BlockAddressBytes)
{
    // CLE1 chain size is # Blocks + CLE command.
    unsigned int iCLE1_Size = u32BlockAddressBytes + 1;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Point to next command.
    pChain->tx_cle1_row_dma.nxt =
	(apbh_dma_gpmi1_t *) & (pChain->tx_cle2_dma);
    // Configure APBH DMA to push Erase command (toggling CLE)
    // into GPMI_CTRL.
    // Transfer CLE1_SIZE (3) bytes to GPMI when GPMI ready.
    // Transfer CLE1 and the row address bytes to GPMI_CTRL0.
    // Wait for end command from GPMI before next part of chain.
    // Lock GPMI to this NAND during transfer.
    // DMA_READ - Perform PIO word transfers then transfer
    //            from memory to peripheral for specified # of bytes.
    pChain->tx_cle1_row_dma.cmd.U =
	NAND_DMA_COMMAND_CMD(iCLE1_Size, 0, NAND_LOCK, 3);
    // Buffer Address Register holds Erase Block command and address.
    pChain->tx_cle1_row_dma.bar =
	(pChain->NandEraseSeed.tx_cle1_block_buf);
    // Setup GPMI bus for first part of Write Command.  Need to set CLE
    // high, then send Write command (0x80), then clear CLE, set ALE high
    // send 4 address bytes.
    pChain->tx_cle1_row_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_ctrl0.U =
	NAND_DMA_COMMAND_PIO(u32NandDeviceNumber, iCLE1_Size,
			     BV_GPMI_CTRL0_ADDRESS_INCREMENT__ENABLED,
			     ASSERT_CS);

    // Set compare to NULL.
    pChain->tx_cle1_row_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_compare.U = (unsigned int) NULL;
    // Disable the ECC.
    pChain->tx_cle1_row_dma.apbh_dma_gpmi3_u.
	apbh_dma_gpmi3_ctrl.gpmi_eccctrl.U =
	NAND_DMA_ECC_PIO(BV_GPMI_ECCCTRL_ENABLE_ECC__DISABLE);

    //    printf("pChain->NandEraseSeed.tx_cle1_block_buf: 0x%x\n", *(unsigned int *)pChain->NandEraseSeed.tx_cle1_block_buf);
    //    printf("pChain->tx_cle1_row_dma.bar: 0x%x\n", pChain->tx_cle1_row_dma.bar);
    //    printf("pChain->tx_cle1_row_dma.bar content: 0x%x\n", *(unsigned int *)pChain->tx_cle1_row_dma.bar);


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Setup 2nd transfer.
    // Setup next command - wait.
    pChain->tx_cle2_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->wait_dma);
    // Configure APBH DMA to push 2nd Erase command (toggling CLE)
    // into GPMI_CTRL.
    // Transfer CLE2_SIZE (1) bytes to GPMI when GPMI ready.
    // Transfer CLE2 byte to GPMI_CTRL0 (see command below)
    // Wait for end command from GPMI before next part of chain.
    // Lock GPMI to this NAND during transfer.
    // DMA_READ - Perform PIO word transfers then transfer
    //            from memory to peripheral for specified # of bytes.
    pChain->tx_cle2_dma.cmd.U = NAND_DMA_COMMAND_CMD(1, 0, NAND_LOCK, 1);
    // Buffer Address Register holds tx_cle2 command
    pChain->tx_cle2_dma.bar = (pChain->NandEraseSeed.tx_cle2_buf);
    // Setup GPMI bus for second part of Erase Command.  Need to set CLE
    // high, then send Erase2 command (0xD0), then clear CLE.
    pChain->tx_cle2_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U =
	NAND_DMA_COMMAND_PIO(u32NandDeviceNumber, 1,
			     BV_GPMI_CTRL0_ADDRESS_INCREMENT__DISABLED,
			     ASSERT_CS);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Setup Wait for Ready descriptor.
    // Setup success DMA pointer.
    pChain->wait_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->sense_dma);
    // Setup Wait for Ready (No transfer count)
    pChain->wait_dma.cmd.U = NAND_DMA_WAIT4RDY_CMD;
    // If there is an error, load Timeout DMA sequence.
    pChain->wait_dma.bar = 0x0;
    // Wait for Ready to go high.
    pChain->wait_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U =
	NAND_DMA_WAIT4RDY_PIO(u32NandDeviceNumber);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Now use Sense sequence to wait for ready.
    pChain->sense_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->statustx_dma);
    // Wait for Ready (No transfer count)- Decrement semaphore.
    pChain->sense_dma.cmd.U = NAND_DMA_SENSE_CMD(0);
    // If failure occurs, branch to pTimeout DMA.
    //pChain->sense_dma.bar = (apbh_dma_gpmi1_t*)&APBH_PROGRAM_FAILED_DMA;
    pChain->sense_dma.bar = (apbh_dma_t *) & (pChain->program_failed_dma);
    // Even though PIO is unused, set it to zero for comparison purposes.
    pChain->sense_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U =
	0;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Next link will Read Status.
    pChain->statustx_dma.nxt =
	(apbh_dma_gpmi1_t *) & (pChain->statusrx_dma);
    // Configure APBH DMA to push CheckStatus command (toggling CLE)
    // into GPMI_CTRL.
    // Transfer NAND_READ_STATUS_SIZE (1) bytes to GPMI when GPMI ready.
    // Wait for end command from GPMI before next part of chain.
    // Lock GPMI to this NAND during transfer.
    // DMA_READ - Perform PIO word transfers then transfer
    //            from memory to peripheral for specified # of bytes.
    pChain->statustx_dma.cmd.U =
	NAND_DMA_COMMAND_CMD(NAND_READ_STATUS_SIZE, 0, NAND_LOCK, 1);
    // Point to structure where NAND Read Status Command is kept.
    pChain->statustx_dma.bar = &(pChain->NandEraseSeed.u8StatusCmd);
    // Setup GPMI bus for first part of Read Status Command.  Need to
    // set CLE high, then send Read Status command (0x70/71), then
    // clear CLE.
    pChain->statustx_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.
	U =
	NAND_DMA_COMMAND_PIO(u32NandDeviceNumber, NAND_READ_STATUS_SIZE,
			     BV_GPMI_CTRL0_ADDRESS_INCREMENT__DISABLED,
			     ASSERT_CS);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Next Link determines SUCCESS or FAILURE.
    pChain->statusrx_dma.nxt =
	(apbh_dma_gpmi1_t *) & (pChain->statbranch_dma);
    // Send a Read & Compare command to the NAND.
    pChain->statusrx_dma.cmd.U =
	NAND_DMA_RX_NO_ECC_CMD(NAND_READ_STATUS_RESULT_SIZE, 0);
    // No DMA Transfer.
    pChain->statusrx_dma.bar = &(pChain->NandEraseSeed.u16Status);
    // GPMI commands.
    pChain->statusrx_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.
	U =
	NAND_DMA_RX_PIO(u32NandDeviceNumber,
			BV_GPMI_CTRL0_WORD_LENGTH__8_BIT,
			NAND_READ_STATUS_RESULT_SIZE);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Branch to appropriate result DMA.
    //pChain->statbranch_dma.nxt = (apbh_dma_t*)&APBH_SUCCESS_DMA;
    pChain->statbranch_dma.nxt = (apbh_dma_t *) & (pChain->success_dma);
    // Based upon above Compare.
    pChain->statbranch_dma.cmd.U = NAND_DMA_SENSE_CMD(0);
    // Failure.
    //pChain->sense_dma.bar = (apbh_dma_gpmi1_t*)&APBH_PROGRAM_FAILED_DMA;
    pChain->sense_dma.bar = (apbh_dma_t *) & (pChain->program_failed_dma);
    // Even though PIO is unused, set it to zero for comparison purposes.
    pChain->sense_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U =
	0;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //TODO: future enhancement - use compare structure.  The mask will need to be
    // computed ahead of time.
    /*
       // Next link will Compare.
       //pChain->tx_dma.nxt = (apbh_dma_gpmi1_t*) &(pChain->statcmp_dma);
       pChain->statustx_dma.nxt = &pPhyDmaDescriptor[5];
       // Configure APBH DMA to push CheckStatus command (toggling CLE)
       // into GPMI_CTRL.
       // Transfer NAND_READ_STATUS_SIZE (1) bytes to GPMI when GPMI ready.
       // Wait for end command from GPMI before next part of chain.
       // Lock GPMI to this NAND during transfer.
       // DMA_READ - Perform PIO word transfers then transfer
       //            from memory to peripheral for specified # of bytes.
       pChain->statustx_dma.cmd.U = NAND_DMA_COMMAND_CMD(NAND_READ_STATUS_SIZE,0, NAND_LOCK);
       // Point to structure where NAND Read Status Command is kept.
       pChain->statustx_dma.bar = pPhyStatusCmd;
       // Setup GPMI bus for first part of Read Status Command.  Need to
       // set CLE high, then send Read Status command (0x70/71), then
       // clear CLE.
       pChain->statustx_dma.gpmi_ctrl0.U = NAND_DMA_COMMAND_PIO(u32NandDeviceNumber,
       NAND_READ_STATUS_SIZE, BV_GPMI_CTRL0_ADDRESS_INCREMENT__DISABLED, ASSERT_CS);

       // Next Link determines SUCCESS or FAILURE.
       pChain->statcmp_dma.nxt = (apbh_dma_gpmi2_t*) &(pChain->statbranch_dma);
       // Send a Read & Compare command to the NAND.
       pChain->statcmp_dma.cmd.U = NAND_DMA_COMPARE_CMD(2);
       // No DMA Transfer.
       pChain->statcmp_dma.bar = &(pChain->NandEraseSeed.u16Status);
       // GPMI commands.
       pChain->statcmp_dma.gpmi_ctrl0.U = NAND_DMA_COMPARE_PIO(u32NandDeviceNumber,
       NAND_READ_STATUS_SIZE);
       // Mask and then compare.
       pChain->statcmp_dma.gpmi_compare.U = pChain->NandEraseSeed.u32StatusMaskRef;

       // Branch to appropriate result DMA.
       pChain->statbranch_dma.nxt = (apbh_dma_t*)&APBH_SUCCESS_DMA;;
       // Based upon above Compare.
       //pChain->branch_dma.cmd.U = NAND_DMA_SENSE_CMD(DECR_SEMAPHORE);
       pChain->statbranch_dma.cmd.U = NAND_DMA_SENSE_CMD(0);
       // Failure.
       pChain->statbranch_dma.bar = (apbh_dma_t*)&APBH_PROGRAM_FAILED_DMA;
     */

    // Initialize the Terminator functions
    // Next function is null.
    pChain->success_dma.nxt = (apbh_dma_t *) 0x0;
    // Decrement semaphore, set IRQ, no DMA transfer.
    pChain->success_dma.cmd.U = ((unsigned int)
				 (BF_APBH_CHn_CMD_IRQONCMPLT(1) |
				  BF_APBH_CHn_CMD_WAIT4ENDCMD(1) |
				  BF_APBH_CHn_CMD_SEMAPHORE(1) |
				  BV_FLD(APBH_CHn_CMD, COMMAND,
					 NO_DMA_XFER)));
    // BAR points to success termination code.
    pChain->success_dma.bar = (void *) SUCCESS;


    // Next function is null.
    pChain->program_failed_dma.nxt = (apbh_dma_t *) 0x0;
    // Decrement semaphore, set IRQ, no DMA transfer.
    pChain->program_failed_dma.cmd.U = ((unsigned int)
					(BF_APBH_CHn_CMD_IRQONCMPLT(1)
					 | BF_APBH_CHn_CMD_WAIT4ENDCMD(1)
					 | BF_APBH_CHn_CMD_SEMAPHORE(1)
					 | BV_FLD(APBH_CHn_CMD,
						  COMMAND, NO_DMA_XFER)));
    // BAR points to timeout termination code.
    pChain->program_failed_dma.bar = 0x80508008;

}

int rom_nand_hal_FindEccErrors(unsigned int u32ECCEngineType,
			       unsigned int u32NumberOfCorrections)
{
    int i;
    unsigned int u32EccErrors = 0, u32Temp;
    unsigned int uEccStatus;

    if (u32ECCEngineType == 0) {
	// ECC8 not supported
    } else {
	// Spin until ECC Complete Interrupt triggers.
	nfc_printf(NFC_DEBUG_MED,
		   "Wait until ECC Complete Interrupt triggers.\n");
	nfc_printf(NFC_DEBUG_MAX, "BCH_CTRL_ADDR: 0x%x\n",
		   readl(BCH_CTRL_ADDR));
	while (1) {
	    if (HW_BCH_CTRL_RD() & BM_BCH_CTRL_COMPLETE_IRQ) {
		nfc_printf(NFC_DEBUG_MED, "BCH completed interrupt\n");
		break;
	    }
	    if (HW_BCH_CTRL_RD() & BM_BCH_CTRL_BM_ERROR_IRQ) {
		nfc_printf(NFC_DEBUG_MED,
			   "BCH: AHB Bus interface Error\n");
		break;
	    }
	}

	// Now read the ECC status.
	uEccStatus = HW_BCH_STATUS0_RD();
	nfc_printf(NFC_DEBUG_MAX, "BCH ECC Status0 = 0x%X\n", uEccStatus);

	if (uEccStatus != SUCCESS) {
	    //:dig later u32EccErrors = -1; //at least 1 error is corrected
	    u32EccErrors = uEccStatus;
	}

    }
    return u32EccErrors;
}

int nand_CheckECCStatus(unsigned int u32NandDeviceNumber,
			unsigned int u32Threshold,
			NAND_dma_read_t * pReadDmaDescriptor)
{
    int retStatus = SUCCESS;
    bool bThresholdReached = FAIL;
    NAND_dma_read_t *pDmaReadDescriptor =
	(NAND_dma_read_t *) pReadDmaDescriptor;
    NAND_read_seed_t *pDmaReadSeed =
	(NAND_read_seed_t *) & (pDmaReadDescriptor->NAND_DMA_Read_Seed);

    unsigned int u32MaxCorrections =
	rom_nand_hal_FindEccErrors(pDmaReadSeed->
				   zNANDEccParams.u32ECCEngine,
				   u32Threshold);

    // bch
    if (u32MaxCorrections) {
	// u32MaxCorrections will be Status0 for BCH.
	// get the errors from auxillary pointer at offset after metadata bytes
	unsigned char *p8AuxPointer =
	    (unsigned char *) pDmaReadSeed->pAuxBuffer;
	int i = 0;
	int indexToAuxBuffer = 0;
	unsigned int u32Temp;
	unsigned int uNumBlks =
	    pDmaReadSeed->zNANDEccParams.m_u32NumEccBlocksPerPage + 1;

	// get the status of Blocks. Each block's status is in a byte, starts at the beginning of a new word where metadata ends
	indexToAuxBuffer =
	    pDmaReadSeed->zNANDEccParams.m_u32MetadataBytes +
	    (pDmaReadSeed->zNANDEccParams.m_u32MetadataBytes % 4);
	// now get the max ecc corrections of data blocks including metadata ecc
	for (i = 0; i < uNumBlks; i++) {
	    unsigned int u32EccLevel;

	    if (i == 0)
		u32EccLevel =
		    pDmaReadSeed->zNANDEccParams.m_u32EccBlock0EccLevel;
	    else
		u32EccLevel = pDmaReadSeed->zNANDEccParams.u32EccType;

	    u32Temp = p8AuxPointer[indexToAuxBuffer + i];
	    if (u32Temp == BV_BCH_STATUS0_STATUS_BLK0__UNCORRECTABLE) {
		nfc_printf(NFC_DEBUG_MAX,
			   "BCH Status ECC Sub-Block Not Recoverable %d = 0x%X\n",
			   i, u32Temp);
	    } else if (u32Temp == BV_BCH_STATUS0_STATUS_BLK0__ERASED) {
		nfc_printf(NFC_DEBUG_MAX,
			   "BCH Status ECC Sub-Block Erased %d = 0x%X\n",
			   i, u32Temp);
	    } else {
		if (u32EccLevel == u32Temp) {
		    bThresholdReached = 1;
		}
		nfc_printf(NFC_DEBUG_MAX,
			   "BCH Status Sub-Block %d = 0x%X\n", i, u32Temp);
	    }
	}
    }

    if (u32MaxCorrections & BM_BCH_STATUS0_ALLONES) {
	nfc_printf(NFC_DEBUG_MAX, "ECC returned ALL ONES Error\n");
	retStatus = 0x8050800F;
    } else if (u32MaxCorrections & BM_BCH_STATUS0_UNCORRECTABLE)
	retStatus = 0x8050800F;
    else if (u32MaxCorrections & BM_BCH_STATUS0_CORRECTED) {
	if (bThresholdReached) {
	    retStatus = 0x80508017;
	}
    }
    // Clear the ECC Complete IRQ.
    BW_BCH_CTRL_COMPLETE_IRQ(0);
    return retStatus;
}



////////////////////////////////////////////////////////////////////////////////
//! \brief Read a given number of bytes from the NAND.
//!
//! This function will read a # of bytes from the NAND.  The structure holding
//! the address, the # of bytes to read, the commands to send, the buffer
//! pointer, the auxillary pointer, the ECCSize, etc are part of the Read
//! DMA descriptor.  This descriptor is passed in and overwritten with
//! the data that needs to change for this transaction.  The Read DMA
//! descriptor will have been filled in with the values that are constant
//! for all transactions using the /e rom_nand_hal_InitReadDma function.
//!
//! \param[in]  pReadDmaDescriptor Pointer to the Read Dma Descriptor.
//! \param[in]  u32NandDeviceNumber Physical NAND number to initialize.
//! \param[in]  u32ColumnOffset Offset in page to start at.
//! \param[in]  u32PageNum Physical page to read.
//! \param[in]  u32ReadSize Number of bytes to read from NAND.
//! \param[in]  p8PageBuf Buffer pointer where the data will be placed.
//! \param[in]  p8AuxillaryBuf Buffer pointer for auxillary buffer.
//!
//! \return Status of call or error.
//! \retval 0            If no error has occurred.
//!
//! \post If successful, the data is in pPageBuf.  The auxillary data (ECC)
//!       is appended at the end of the valid data.
//! \note p8PageBuf must be larger than 2112 because the ECC working area
//!       is appended after the valid data.
//! \note u32ReadSize is the total size which includes the ECC.
////////////////////////////////////////////////////////////////////////////////
int nand_Read(unsigned int u32NandDeviceNumber,
	      unsigned int u32ColumnOffset,
	      unsigned int u32PageNum,
	      unsigned int u32ReadSize,
	      unsigned char *p8PageBuf, unsigned char *p8AuxillaryBuf)
{
    unsigned int val;
    int status;

    NAND_dma_read_t *pDmaReadDescriptor = (NAND_dma_read_t *) RSVBUFFADDR;
    NAND_dma_read_t *phy_DmaReadDescriptor;

    NAND_read_seed_t *pDmaReadSeed =
	(NAND_read_seed_t *) & (pDmaReadDescriptor->NAND_DMA_Read_Seed);
    NAND_read_seed_t *phy_DmaReadSeed;
    unsigned int u32Temp;

    /* Init DMA Descriptor */
    unsigned int u32NumRowBytes = 3;

    memset(pDmaReadDescriptor, 0x0, sizeof(NAND_dma_read_t));

    HAL_VIRT_TO_PHYS_ADDRESS(pDmaReadDescriptor, phy_DmaReadDescriptor);

    /* Configuration parameters for BCH/ECC */
    pDmaReadSeed->zNANDEccParams.u32ECCEngine = nand_ecc_params.u32ECCEngine;	//BCH
    pDmaReadSeed->zNANDEccParams.m_u32NumEccBlocksPerPage =
	nand_ecc_params.m_u32NumEccBlocksPerPage;
    pDmaReadSeed->zNANDEccParams.m_u32MetadataBytes =
	nand_ecc_params.m_u32MetadataBytes;
    pDmaReadSeed->zNANDEccParams.m_u32EccBlock0Size =
	nand_ecc_params.m_u32EccBlock0Size;
    pDmaReadSeed->zNANDEccParams.m_u32EccBlockNSize =
	nand_ecc_params.m_u32EccBlockNSize;
    pDmaReadSeed->zNANDEccParams.m_u32EccBlock0EccLevel =
	nand_ecc_params.m_u32EccBlock0EccLevel;
    pDmaReadSeed->zNANDEccParams.m_u32EraseThreshold =
	nand_ecc_params.m_u32EraseThreshold;
    pDmaReadSeed->zNANDEccParams.u32EccType = nand_ecc_params.u32EccType;
    pDmaReadSeed->zNANDEccParams.m_u32PageSize =
	nand_ecc_params.m_u32PageSize;
    pDmaReadSeed->bEnableHWECC = 1;	//enable ecc

    // Reset the APBH NAND channels and clr IRQs
    val = readl(APBH_DMA_CTRL_ADDR);
    val &= ~(0xff << 16);
    val |= (0x10 << 16);	// reset nand0 channel
    writel(val, APBH_DMA_CTRL_ADDR);

    writel(0xf0, APBH_DMA_CTRL1_CLR);

    val &= ~(0xff << 16);
    writel(val, APBH_DMA_CTRL_ADDR);

    /* BCH complete IRQ enable */
    //HW_BCH_CTRL_SET(1<<8);

    nand_InitReadDma(phy_DmaReadDescriptor, u32NumRowBytes, 8, pDmaReadSeed->zNANDEccParams.u32EccType, 0x00, 0x30);	//ecc type: RS_Ecc_4bit

    nfc_printf(NFC_DEBUG_MED, "Read NAND - NAND Read size %d\n",
	       u32ReadSize);

    // Fill in the Column Address (Always 2 bytes)
    pDmaReadSeed->tx_cle1_addr_dma_buffer.
	tx_cle1_addr_Columns_Rows.tx_cle1_Columns_Rows.tx_cle1_Type2.
	bType2Columns[0] = (unsigned char) (u32ColumnOffset & 0xFF);
    pDmaReadSeed->tx_cle1_addr_dma_buffer.
	tx_cle1_addr_Columns_Rows.tx_cle1_Columns_Rows.tx_cle1_Type2.
	bType2Columns[1] = (unsigned char) ((u32ColumnOffset >> 8) & 0xFF);

    // Fill in the Row Address. (Can be 2 or 3 bytes)
    pDmaReadSeed->tx_cle1_addr_dma_buffer.
	tx_cle1_addr_Columns_Rows.tx_cle1_Columns_Rows.tx_cle1_Type2.
	bType2Rows[0] = (unsigned char) (u32PageNum & 0xFF);
    pDmaReadSeed->tx_cle1_addr_dma_buffer.
	tx_cle1_addr_Columns_Rows.tx_cle1_Columns_Rows.tx_cle1_Type2.
	bType2Rows[1] = (unsigned char) ((u32PageNum >> 8) & 0xFF);
    // This is always created, but the Address size determines whether
    // this data is actually sent.
    pDmaReadSeed->tx_cle1_addr_dma_buffer.
	tx_cle1_addr_Columns_Rows.tx_cle1_Columns_Rows.tx_cle1_Type2.
	bType2Rows[2] = (unsigned char) ((u32PageNum >> 16) & 0xFF);

    // Set how many bytes need to be read.
    pDmaReadSeed->uiReadSize = u32ReadSize;
    // Set the location where data will be read into.
    pDmaReadSeed->pDataBuffer = p8PageBuf;
    // Set the location where auxillary buffers will reside..
    pDmaReadSeed->pAuxBuffer = p8AuxillaryBuf;

    // Calculate the ECC Mask for this transaction.
    // BCH-AuxOnly = 0x100 set to request transfer to/from the Auxiliary buffer.
    // BCH Page = 0x1FF set to request transfer to/from the whole page buffer.
    // Auxilliary = 0x100 set to request transfer to/from the Auxiliary buffer.
    // Buffer7 = 0x080 set to request transfer to/from buffer7.
    // Buffer6 = 0x040 set to request transfer to/from buffer6.
    // Buffer5 = 0x020 set to request transfer to/from buffer5.
    // Buffer4 = 0x010 set to request transfer to/from buffer4.
    // Buffer3 = 0x008 set to request transfer to/from buffer3.
    // Buffer2 = 0x004 set to request transfer to/from buffer2.
    // Buffer1 = 0x002 set to request transfer to/from buffer1.
    // Buffer0 = 0x001 set to request transfer to/from buffer0.


    // BCH: initialize mask to read from the beginning of a page including metadata
    pDmaReadSeed->uiECCMask = BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE;


    nand_BuildQuickReadDma(pDmaReadDescriptor, u32NandDeviceNumber,
			   pDmaReadSeed);

    // Clear the ECC Complete flag.
    writel(0x1, BCH_CTRL_CLR);

    // Kick off the DMA.
    nand_StartDma((dma_cmd_t *) phy_DmaReadDescriptor,
		  u32NandDeviceNumber);

    nfc_printf(NFC_DEBUG_MED, "Wait for NAND read complete\n");
    status = nand_WaitDma(MAX_TRANSACTION_TIMEOUT, 0);
    if (status != SUCCESS) {
	nfc_printf(NFC_DEBUG_MAX, "Nand Read Failed.\n");
	nfc_printf(NFC_DEBUG_MAX, "DGB1: 0x%x, DGB2: 0x%x\n",
		   readl(APBH_DMA_CH4_DBG), readl(APBH_DMA_CH4_DBG2));
	nfc_printf(NFC_DEBUG_MAX, "GPMI Data: 0x%x\n", readl(GPMI_DATA));
	nfc_printf(NFC_DEBUG_MAX, "GPMI Status: 0x%x\n",
		   readl(GPMI_STATUS));
	nfc_printf(NFC_DEBUG_MAX, "GPMI Debug info: 0x%x\n",
		   readl(GPMI_DEBUG));
	nfc_printf(NFC_DEBUG_MAX, "GPMI TIME1: 0x%x\n",
		   readl(GPMI_TIMEOUT1_ADDR));
	return FAIL;
    } else {
	nfc_printf(NFC_DEBUG_MED,
		   "NAND Read Complete, Check ECC Status.\n");
	nand_CheckECCStatus(u32NandDeviceNumber,
			    pDmaReadSeed->zNANDEccParams.u32EccType,
			    phy_DmaReadDescriptor);
    }

    return (SUCCESS);
}

// Read form addr to data
int nandflash_read_buf(void *addr, void *data, int len)
{
    int status;
    int i;
    unsigned char *p8PageBuf, *phy_8PageBuf;
    unsigned char *p8AuxillaryBuf, *phy_8AuxillaryBuf;
    int size;
    unsigned int pageNum;


    nfc_printf(NFC_DEBUG_MED, "%s()\n", __FUNCTION__);

    p8PageBuf = (unsigned char *) (RSVBUFFADDR + BUFFER_SIZE);
    memset(p8PageBuf, 0x0, 0x2000);
    p8AuxillaryBuf =
	(unsigned char *) (RSVBUFFADDR + BUFFER_SIZE + 0x2000);
    memset(p8AuxillaryBuf, 0x0, 0x2000);

    HAL_VIRT_TO_PHYS_ADDRESS(p8PageBuf, phy_8PageBuf);
    HAL_VIRT_TO_PHYS_ADDRESS(p8AuxillaryBuf, phy_8AuxillaryBuf);

    size = len;
    // convert addr to pageNum;
    nfc_printf(NFC_DEBUG_MAX, "len: 0x%x\n", len);
    nfc_printf(NFC_DEBUG_MAX, "addr: 0x%x\n", (unsigned int) addr);

    nand_offset2pagenum((unsigned int) addr, &pageNum);

    i = 0;

    while (size > 0) {
	nfc_printf(NFC_DEBUG_MAX, "Page Num: 0x%x", pageNum);
	status =
	    nand_Read(0, 0, pageNum, NF_PG_SZ + flash_dev_info->spare_size,
		      phy_8PageBuf, phy_8AuxillaryBuf);
	if (status == SUCCESS) {
	    nfc_printf(NFC_DEBUG_MAX, "0x%x 0x%x 0x%x \n", *p8PageBuf,
		       *(p8PageBuf + 1), *(p8PageBuf + 2));
	    nfc_printf(NFC_DEBUG_MED, "Read Passed.\n");
	}

	if (size >= NF_PG_SZ) {
	    memcpy((unsigned int) data + i * NF_PG_SZ, p8PageBuf,
		   NF_PG_SZ);
	} else {
	    memcpy((unsigned int) data + i * NF_PG_SZ, p8PageBuf, size);
	}

	i++;
	pageNum++;
	size -= NF_PG_SZ;
    }

    return SUCCESS;
    //return nfc_read_region((u32)addr, (u32)data, (u32)len);
}


int nand_Erase(unsigned int u32NandDeviceNumber, unsigned int BlockNum)
{
    int status;
    unsigned int wRowAddr;
    //    unsigned int wChipNum = pNANDDescriptor->wChipNumber;
    unsigned int u32BlockAddressBytes;
    unsigned int u32NumRowBytes = 3;

    NAND_dma_block_erase_t *pNandDmaDescriptor =
	(NAND_dma_block_erase_t *) RSVBUFFADDR;

    NAND_dma_block_erase_t *phy_DmaEraseDescriptor;

    memset(pNandDmaDescriptor, 0xaa, sizeof(NAND_dma_block_erase_t));

    HAL_VIRT_TO_PHYS_ADDRESS(pNandDmaDescriptor, phy_DmaEraseDescriptor);

    // The seed can be local because the DMA portion will complete before
    // leaving this function.

    // Use the 1st page of the block to calculate the Row address.
    wRowAddr = BlockNum * NF_PG_PER_BLK;

    u32BlockAddressBytes = u32NumRowBytes;	//pNANDDescriptor->pNANDParams->wNumRowBytes;

    // Fill in the Row Address. (Can be 2 or 3 bytes)
    // The Address size will determine how many bytes are sent.
    pNandDmaDescriptor->NandEraseSeed.tx_block[0] =
	(unsigned char) (wRowAddr & 0xFF);
    pNandDmaDescriptor->NandEraseSeed.tx_block[1] =
	(unsigned char) ((wRowAddr >> 8) & 0xFF);
    pNandDmaDescriptor->NandEraseSeed.tx_block[2] =
	(unsigned char) ((wRowAddr >> 16) & 0xFF);

    // Load Command Code for Serial Data Input
    pNandDmaDescriptor->NandEraseSeed.tx_cle1 = 0x60;

    // Load Command Code for Page Program
    pNandDmaDescriptor->NandEraseSeed.tx_cle2 = 0xD0;

    // Load command and mask for GetStatus portion of DMA.
    pNandDmaDescriptor->NandEraseSeed.u8StatusCmd = 0x70;

    // Build the DMA that will program this sector.
    nand_BuildEraseDma(phy_DmaEraseDescriptor, 0, u32BlockAddressBytes);

    nand_StartDma((dma_cmd_t *) phy_DmaEraseDescriptor,
		  u32NandDeviceNumber);

    nfc_printf(NFC_DEBUG_MED, "Wait for NAND Erase complete\n");
    status = nand_WaitDma(MAX_TRANSACTION_TIMEOUT, 0);
    if (status != SUCCESS) {
	printf("Nand Erase Failed.\n");
	printf("DGB1: 0x%x, DGB2: 0x%x\n",
	       readl(APBH_DMA_CH4_DBG), readl(APBH_DMA_CH4_DBG2));
	printf("GPMI Data: 0x%x\n", readl(GPMI_DATA));
	printf("GPMI Status: 0x%x\n", readl(GPMI_STATUS));
	printf("GPMI Debug info: 0x%x\n", readl(GPMI_DEBUG));
	printf("GPMI TIME1: 0x%x\n", readl(GPMI_TIMEOUT1_ADDR));
	return FAIL;
    }

    return SUCCESS;

}


int nand_Write(unsigned int u32NandDeviceNumber,
	       unsigned int u32PageNum, unsigned char *p8PageBuf,
	       unsigned char *p8AuxillaryBuf)
{

    unsigned int i, val;

    int rtCode;

    unsigned int btNumRowBytes = 0x3;

    // Create a pointer to this device.
    NAND_dma_program_t *pDmaWriteDescriptor =
	(NAND_dma_program_t *) RSVBUFFADDR;
    NAND_dma_program_t *phy_DmaWriteDescriptor = 0;
    HAL_VIRT_TO_PHYS_ADDRESS(pDmaWriteDescriptor, phy_DmaWriteDescriptor);
    memset(pDmaWriteDescriptor, 0x0, sizeof(NAND_dma_program_t));

    // TT_Fixme - I need to add the ECC calculation in here.
    // Set address size to # row bytes + 2 (for column bytes).
    pDmaWriteDescriptor->NandProgSeed.NandSizeVars.uiAddressSize =
	btNumRowBytes + 2;

    pDmaWriteDescriptor->NandProgSeed.NandSizeVars.uiWriteSize = NF_PG_SZ + flash_dev_info->spare_size;	//u32TotalPageSize;

    // set the Word size
    // normally use 8 bit data width
    pDmaWriteDescriptor->NandProgSeed.NandSizeVars.uiWordSize =
	BV_GPMI_CTRL0_WORD_LENGTH__8_BIT;

    // Always 2 bytes and we're always going to be column 0.
    pDmaWriteDescriptor->NandProgSeed.bType2Columns[0] = (unsigned char) 0;
    pDmaWriteDescriptor->NandProgSeed.bType2Columns[1] = (unsigned char) 0;

    // Fill in the Row Address. (Can be 2 or 3 bytes)
    for (i = 0; i < btNumRowBytes; i++) {
	pDmaWriteDescriptor->NandProgSeed.bType2Rows[i] =
	    (unsigned char) ((u32PageNum >> (8 * i)) & 0xFF);
    }

    // Load Command Code for Serial Data Input (0x80)
    pDmaWriteDescriptor->NandProgSeed.tx_cle1 = 0x80;

    // Load Command Code for Page Program (0x10)
    pDmaWriteDescriptor->NandProgSeed.tx_cle2 = 0x10;

    // Load Command Code for Read Status (0x70)
    pDmaWriteDescriptor->NandProgSeed.u8StatusCmd = 0x70;

    nfc_printf(NFC_DEBUG_MAX,
	       "pDmaWriteDescriptor->NandProgSeed addr: 0x%x\n",
	       (unsigned int) &pDmaWriteDescriptor->NandProgSeed);

    // set bEnableHWECC of progseed
    pDmaWriteDescriptor->NandProgSeed.bEnableHWECC = 1;

    // Reset the APBH NAND channels and clr IRQs
    val = readl(APBH_DMA_CTRL_ADDR);
    val &= ~(0xff << 16);
    val |= (0x10 << 16);	// reset nand0 channel
    writel(val, APBH_DMA_CTRL_ADDR);

    writel(0xf0, APBH_DMA_CTRL1_CLR);

    val &= ~(0xff << 16);
    writel(val, APBH_DMA_CTRL_ADDR);

    /* BCH complete IRQ enable */
    //HW_BCH_CTRL_SET(1<<8);

    // Clear the ECC Complete flag.
    writel(0x1, BCH_CTRL_CLR);

    nfc_printf(NFC_DEBUG_MAX, "GPMI CTRL1: 0x%x\n",
	       readl(GPMI_CTRL1_ADDR));
    /* disable write protect */
    writel(1 << 3, GPMI_CTRL1_SET);

    /* Set for BCH */
    pDmaWriteDescriptor->NandProgSeed.zNANDEccParams.u32ECCEngine = nand_ecc_params.u32ECCEngine;	//BCH
    pDmaWriteDescriptor->NandProgSeed.
	zNANDEccParams.m_u32NumEccBlocksPerPage =
	nand_ecc_params.m_u32NumEccBlocksPerPage;
    pDmaWriteDescriptor->NandProgSeed.zNANDEccParams.m_u32MetadataBytes =
	nand_ecc_params.m_u32MetadataBytes;
    pDmaWriteDescriptor->NandProgSeed.zNANDEccParams.m_u32EccBlock0Size =
	nand_ecc_params.m_u32EccBlock0Size;
    pDmaWriteDescriptor->NandProgSeed.zNANDEccParams.m_u32EccBlockNSize =
	nand_ecc_params.m_u32EccBlockNSize;
    pDmaWriteDescriptor->NandProgSeed.
	zNANDEccParams.m_u32EccBlock0EccLevel =
	nand_ecc_params.m_u32EccBlock0EccLevel;
    pDmaWriteDescriptor->NandProgSeed.zNANDEccParams.m_u32EraseThreshold =
	nand_ecc_params.m_u32EraseThreshold;
    pDmaWriteDescriptor->NandProgSeed.zNANDEccParams.u32EccType =
	nand_ecc_params.u32EccType;
    pDmaWriteDescriptor->NandProgSeed.zNANDEccParams.m_u32PageSize =
	nand_ecc_params.m_u32PageSize;
    pDmaWriteDescriptor->NandProgSeed.bEnableHWECC = 1;	//enable ecc

    // Build the DMA that will program this sector.
    nand_BuildProgramDma(phy_DmaWriteDescriptor, u32NandDeviceNumber,
			 pDmaWriteDescriptor->NandProgSeed.NandSizeVars.
			 uiAddressSize,
			 pDmaWriteDescriptor->NandProgSeed.zNANDEccParams.
			 m_u32PageSize,
			 pDmaWriteDescriptor->NandProgSeed.zNANDEccParams.
			 u32EccType, p8PageBuf, p8AuxillaryBuf);

    //    hal_delay_us(10000);

    nfc_printf(NFC_DEBUG_MAX, "GPMI Ctrl0: 0x%x\n",
	       readl(GPMI_CTRL0_ADDR));
    nfc_printf(NFC_DEBUG_MAX, "GPMI Ctrl1: 0x%x\n",
	       readl(GPMI_CTRL1_ADDR));

    nand_StartDma((dma_cmd_t *) phy_DmaWriteDescriptor,
		  u32NandDeviceNumber);

    rtCode = nand_WaitDma(MAX_TRANSACTION_TIMEOUT, u32NandDeviceNumber);

    // Check the status of the write.
    // Check the Pass Fail bit (bit 0) and the inverted Write Protect bit (bit 7)
    if ((pDmaWriteDescriptor->NandProgSeed.u16Status & 0x81) ^ 0x80) {
	rtCode = 0x80508009;
	// Writes are not normally included in
    }

    return (rtCode);		// Success or failure?

}

int nand_ReadStatus(unsigned int u32NandDeviceNumber,
		    unsigned int *pStatusResult)
{

    int retCode;
    NAND_dma_read_status_t *pdma_read_status =
	(NAND_dma_read_status_t *) RSVBUFFADDR;
    NAND_dma_read_status_t *phy_dma_read_status;

    memset(pdma_read_status, 0xaa, sizeof(NAND_dma_read_status_t));
    HAL_VIRT_TO_PHYS_ADDRESS(pdma_read_status, phy_dma_read_status);

    // Send one byte
    pdma_read_status->uiReadStatusSize = 1;

    pdma_read_status->tx_cle1 = 0x70;

    // Receive one byte back.
    pdma_read_status->uiReadStatusResultSize = 1;

    nand_BuildReadStatusDma(pdma_read_status, u32NandDeviceNumber,
			    &pdma_read_status->rx_Result);

    nand_StartDma((dma_cmd_t *) phy_dma_read_status, u32NandDeviceNumber);

    retCode = nand_WaitDma(MAX_TRANSACTION_TIMEOUT, u32NandDeviceNumber);

    *pStatusResult = (unsigned int) pdma_read_status->rx_Result;

    return retCode;
}

void nand_ReadID(unsigned int u32NandDeviceNumber, void *pReadIDBuffer)
{
    int status;
    NAND_dma_read_id_t *pdma_read_id, *pChain;

    pdma_read_id = (NAND_dma_read_id_t *) RSVBUFFADDR;

    // convert from virtual address to physical address
    HAL_VIRT_TO_PHYS_ADDRESS(pdma_read_id, pChain);
    //    pChain = (NAND_dma_read_id_t *)((unsigned int)&dma_read_id + 0x40000000);

    // Change the default ReadID code to what is being passed in.
    pChain->dma_read_id_buffer.txCLE_txALE.txCLEByte = 0x90;
    pChain->dma_read_id_buffer.txCLE_txALE.txALEByte = 0x00;

    // First we want to wait for Ready.  The chip may be busy on power-up.
    // Wait for Ready.
    pChain->wait4rdy_dma.nxt =
	(apbh_dma_gpmi1_t *) & (pChain->sense_rdy_dma);
    pChain->wait4rdy_dma.cmd.U = NAND_DMA_WAIT4RDY_CMD;
    // BAR points to alternate branch if timeout occurs.
    pChain->wait4rdy_dma.bar = (apbh_dma_gpmi1_t *) 0x00;
    // Set GPMI wait for ready.
    pChain->wait4rdy_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.
	U = NAND_DMA_WAIT4RDY_PIO(u32NandDeviceNumber);

    // Now check for successful Ready.
    pChain->sense_rdy_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->tx_dma);
    pChain->sense_rdy_dma.cmd.U = NAND_DMA_SENSE_CMD(0);
    // BAR points to alternate branch if timeout occurs.
    pChain->sense_rdy_dma.bar = (apbh_dma_gpmi1_t *) & (pChain->timeout_dma);
    // Even though PIO is unused, set it to zero for comparison purposes.
    pChain->sense_rdy_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U = 0;

    // Next command in chain will be a read.
    pChain->tx_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->rx_dma);
    // Configure APBH DMA to push Read ID command (toggling CLE & ALE)
    // into GPMI_CTRL.
    // Transfer NAND_READ_ID_SIZE to GPMI when GPMI ready.
    // Transfer 1 word to GPMI_CTRL0 (see command below)
    // Wait for end command from GPMI before rx part of chain.
    // Lock GPMI to this NAND during transfer.
    // DMA_READ - Perform PIO word transfers then transfer
    //            from memory to peripheral for specified # of bytes.
    pChain->tx_dma.cmd.U = NAND_DMA_COMMAND_CMD(NAND_READ_ID_SIZE, 0, NAND_LOCK, 3);

    // Buffer Address Register being used to hold Read ID command.
    //FIXME - ReadID is packed inside chain.
    pChain->tx_dma.bar = pChain->dma_read_id_buffer.tx_readid_command_buf;

    // Setup GPMI bus for Read ID Command.  Need to set CLE high, then
    // low, then ALE toggles high and low.  Read ID Code sent during
    // CLE, 2nd byte (0x00) sent during ALE.
    pChain->tx_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_ctrl0.U =
	NAND_DMA_COMMAND_PIO(u32NandDeviceNumber, NAND_READ_ID_SIZE,
			     BV_GPMI_CTRL0_ADDRESS_INCREMENT__ENABLED, ASSERT_CS);

    // Nothing needs to happen to the compare.
    pChain->tx_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_compare.U = (unsigned int) NULL;

    // Disable the ECC.
    pChain->tx_dma.apbh_dma_gpmi3_u.apbh_dma_gpmi3_ctrl.gpmi_eccctrl.U =
	NAND_DMA_ECC_PIO(BV_GPMI_ECCCTRL_ENABLE_ECC__DISABLE);

    // Setup 2nd complete DMA sequence.
    // Setup to use SUCESS DMA sequence if successful.
    pChain->rx_dma.nxt = (apbh_dma_gpmi1_t *) & (pChain->success_dma);

    // Configure APBH DMA to push Read ID command
    // into GPMI_CTRL.
    // Transfer NAND_READ_ID_SIZE to GPMI when GPMI ready.
    // Transfer 1 word to GPMI_CTRL0 (see command below)
    // Wait for end command from GPMI before rx part of chain.
    // Lock GPMI to this NAND during transfer.
    // DMA_WRITE - Perform PIO word transfers then transfer to
    //             memory from peripheral for specified # of bytes.
    pChain->rx_dma.cmd.U = NAND_DMA_RX_NO_ECC_CMD(NAND_READ_ID_RESULT_SIZE, 0);
    // Buffer Address Register being used to hold Read ID result.
    
    pChain->rx_dma.bar = pReadIDBuffer;

    // Setup GPMI bus for Read ID Result.  Read data back in.
    // Read RESULT_SIZE bytes (8 bit) data
    pChain->rx_dma.apbh_dma_gpmi1_u.apbh_dma_gpmi1_ctrl.gpmi_ctrl0.U =
	NAND_DMA_RX_PIO(u32NandDeviceNumber, BV_GPMI_CTRL0_WORD_LENGTH__8_BIT, NAND_READ_ID_RESULT_SIZE);

    // Initialize the Terminator functions
    // Next function is null.
    pChain->success_dma.nxt = (apbh_dma_t *) 0x0;
    // Decrement semaphore, set IRQ, no DMA transfer.
    pChain->success_dma.cmd.U = ((unsigned int) (BF_APBH_CHn_CMD_IRQONCMPLT(1) |
				  BF_APBH_CHn_CMD_SEMAPHORE(1) | BV_FLD(APBH_CHn_CMD, COMMAND, NO_DMA_XFER)));

    // BAR points to success termination code.
    pChain->success_dma.bar = (void *) 0;

    // Next function is null.
    pChain->timeout_dma.nxt = (apbh_dma_t *) 0x0;

    // Decrement semaphore, set IRQ, no DMA transfer.
    pChain->timeout_dma.cmd.U = ((unsigned int) (BF_APBH_CHn_CMD_IRQONCMPLT(1) |
				  BF_APBH_CHn_CMD_SEMAPHORE(1) | BV_FLD(APBH_CHn_CMD, COMMAND, NO_DMA_XFER)));

    // BAR points to timeout termination code.
    pChain->timeout_dma.bar = (void *) 0x80508008;

    nfc_printf(NFC_DEBUG_MED, "Start DMA to get NAND ID: \n");

    nand_StartDma((dma_cmd_t *) pChain, 0);

    status = nand_WaitDma(MAX_TRANSACTION_TIMEOUT, 0);
    if (status != SUCCESS) {
	printf("Read ID Failed.\n");
    }
}

void do_nand_test(int argc, char *argv[])
{
    int status;
    int i;
    unsigned int pageNum;
    unsigned char *p8PageBuf, *phy_8PageBuf;
    unsigned char *p8AuxillaryBuf, *phy_8AuxillaryBuf;
    unsigned int StatusResult;
    ReadIDCode *pReadIDBuf;

    printf("do nand test:\n");

    printf("===========NAND Init==============\n");

    pReadIDBuf = (ReadIDCode *) (RSVBUFFADDR + BUFFER_SIZE);
    memset(pReadIDBuf, 0x34, sizeof(ReadIDCode));

    nand_Init(pReadIDBuf);

    /* Read NAND status */
//    nand_ReadStatus(0, &StatusResult);
//    printf("Return value: 0x%x\n", StatusResult);

    printf("===========Write TEST==============\n");
    p8PageBuf = (unsigned char *) (RSVBUFFADDR + BUFFER_SIZE);
    memset(p8PageBuf, 0x88, 0x2000);
    p8AuxillaryBuf = (unsigned char *) (RSVBUFFADDR + BUFFER_SIZE + 0x2000);
    memset(p8AuxillaryBuf, 0x00, 0x2000);

    HAL_VIRT_TO_PHYS_ADDRESS(p8PageBuf, phy_8PageBuf);
    HAL_VIRT_TO_PHYS_ADDRESS(p8AuxillaryBuf, phy_8AuxillaryBuf);

    pageNum = 0x000;

    for (i = 0; i < 20; i++) {
	memset(p8PageBuf, i, 0x2000);

	status = nand_Write(0, pageNum, phy_8PageBuf, phy_8AuxillaryBuf);
	if (status == SUCCESS) {
	    printf("Write Test Passed. 0x%x\n", i);
	}

	pageNum++;
    }

    printf("==========Read Test===============\n");
    nfc_printf(NFC_DEBUG_MED, "GPMI TIME1: 0x%x\n", readl(GPMI_TIMEOUT1_ADDR));

    p8PageBuf = (unsigned char *) (RSVBUFFADDR + BUFFER_SIZE);
    memset(p8PageBuf, 0x12, 0x2000);

    p8AuxillaryBuf = (unsigned char *) (RSVBUFFADDR + BUFFER_SIZE + 0x2000);
    memset(p8AuxillaryBuf, 0x89, 0x2000);

    HAL_VIRT_TO_PHYS_ADDRESS(p8PageBuf, phy_8PageBuf);
    HAL_VIRT_TO_PHYS_ADDRESS(p8AuxillaryBuf, phy_8AuxillaryBuf);

    pageNum = 0x000;

    for (i = 0; i < 20; i++) {
	status = nand_Read(0, 0, pageNum, 2112, phy_8PageBuf, phy_8AuxillaryBuf);
	if (status == SUCCESS) {
	    printf("0x%x 0x%x 0x%x \n", *p8PageBuf, *(p8PageBuf + 1), *(p8PageBuf + 2));
	    printf("Read Test Passed.\n");
	}

	pageNum++;
    }

    printf("===========Erase TEST==============\n");
    nand_Erase(0, 0x600 / 64);	//block 1

    nand_ReadStatus(0, &StatusResult);
    printf("Return value2: 0x%x\n", StatusResult);
}

static void nand_info(int argc, char *argv[])
{
    if (nand_flash_index == -1) {
	printf("Can't find valid NAND flash: %d\n", __LINE__);
	return;
    }

    printf("\nType:\t\t %s\n", NF_VEND_INFO);
    printf("Total size:\t 0x%08x bytes (%d MB)\n", NF_DEV_SZ,
	   NF_DEV_SZ / 0x100000);
    printf("Total blocks:\t 0x%x (%d)\n", NF_BLK_CNT, NF_BLK_CNT);
    printf("Block size:\t 0x%x (%d)\n", NF_BLK_SZ, NF_BLK_SZ);
    printf("Page size:\t 0x%x (%d)\n", NF_PG_SZ, NF_PG_SZ);
    printf("Pages per block: 0x%x (%d)\n", NF_PG_PER_BLK, NF_PG_PER_BLK);
}

int nandflash_hwr_init(void)
{
    int i;
    ReadIDCode *pReadIDBuf;
    unsigned short device_id2;

    nfc_printf(NFC_DEBUG_MED, "%s()\n", __FUNCTION__);

    pReadIDBuf = (ReadIDCode *) (RSVBUFFADDR + BUFFER_SIZE);
    memset(pReadIDBuf, 0x34, sizeof(ReadIDCode));

    nand_Init(pReadIDBuf);

    device_id2 = ((pReadIDBuf->SamsungHSSerialAccess << 15) |
	          (pReadIDBuf->Organization << 14) |
	          (pReadIDBuf->BlockSize << 12) |
	          (pReadIDBuf->Reserved0 << 11) |
	          (pReadIDBuf->RedundantAreaSize << 10) |
	          (pReadIDBuf->PageSize << 8) |
	          (pReadIDBuf->CacheProgram << 7) |
	          (pReadIDBuf->VendorSpecific0 << 4) |
	          (pReadIDBuf->CellType << 2) |
	          (pReadIDBuf->InternalChipNumber)) & 0xFFFF;

    flash_dev_info = supported_devices;
    for (i = 0; i < NUM_DEVICES; i++) {
	if ((flash_dev_info->device_id == pReadIDBuf->DeviceID_Code.usDeviceID)
	    && ((flash_dev_info->device_id2 == 0xFFFF) || (flash_dev_info->device_id2 == device_id2)))
	    break;

	flash_dev_info++;
    }

    // Do we find the device? If not, return error.
    if (NUM_DEVICES == i) {
	printf("Unrecognized NAND part: 0x%x 0x%x\n",
	       pReadIDBuf->DeviceID_Code.usDeviceID, device_id2);
	return FAIL;
    }

    nand_flash_index = i;
    nand_info(0, NULL);
#if 0
    flash_info.block_size = NF_BLK_SZ;
    flash_info.blocks = NF_BLK_CNT;
    flash_info.start = (void *) 0;
    flash_info.end = NF_DEV_SZ;
#endif
    return SUCCESS;
}

int nand_init()
{
	return nandflash_hwr_init();
}

static int do_mxnand(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	printf("argc = %d\n", argc);
	if (argc < 2)
		return 0;

	int i = argc;
	while (i--)
		printf("argv[%d]: %s\n", argc, argv[i]);
	
	uint32_t len = 0;
	void *from = NULL, *to = NULL;
	if (argc == 5) {
		from = (void *)simple_strtoul(argv[2], NULL, 16);
		to = (void *)simple_strtoul(argv[3], NULL, 16);
		len = simple_strtoul(argv[4], NULL, 16);
		printf("from = %p, to = %p, len = 0x%x\n", from, to, len);
	}

	char *cmd = argv[1];
	if (!strcmp(cmd, "init"))
		nandflash_hwr_init();
	else if (!strcmp(cmd, "test"))
		do_nand_test(argc, argv);
	else if (!strcmp(cmd, "read"))
		nandflash_read_buf(from, to, len);
	else if (!strcmp(cmd, "write"))
		nandflash_program_buf(from, to, len);

	return 0;
}

U_BOOT_CMD(mxnand, 5, 1, do_mxnand,
	"nand driver test for mx233",
	"mxnand init\n"
	"mxnand test\n"
	"mxnand read from_addr to_addr len\n"
	"mxnand write from_addr to_addr len\n"
	"mxnand erase"
);
