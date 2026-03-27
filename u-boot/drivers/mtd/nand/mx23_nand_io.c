/*
 * ==========================================================================
 *
 *       Filename:  mx23_nand_io.c
 *
 *    Description:  
 *
 *        Version:  0.01
 *        Created:  2011年04月14日 14时31分39秒
 *
 *         Author:  smmei (), 
 *        Company:  
 *
 * ==========================================================================
 */

#include <common.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <linux/mtd/nand.h>
#include <asm/arch/regsgpmi.h>
#include <asm/arch/regspinctrl.h>
#include <asm/arch/regsclkctrl.h>

#define pr_err(fmt, args...) 	printf(fmt, ##args)
#define pr_debug(fmt, args...) 	printf(fmt, ##args)

struct gpmi_timing {
	uint32_t t_ds;
	uint32_t t_dh;
	uint32_t t_as;
	uint32_t t_dsample;
	uint32_t t_busytimeout;
};

static int __mxs_reset_block(void __iomem *hwreg, int just_enable)
{
	u32 c;
	int timeout;

	/* the process of software reset of IP block is done
	   in several steps:

	   - clear SFTRST and wait for block is enabled;
	   - clear clock gating (CLKGATE bit);
	   - set the SFTRST again and wait for block is in reset;
	   - clear SFTRST and wait for reset completion.
	 */
	c = __raw_readl(hwreg);
	c &= ~(1 << 31);	/* clear SFTRST */
	__raw_writel(c, hwreg);
	for (timeout = 1000000; timeout > 0; timeout--)
		/* still in SFTRST state ? */
		if ((__raw_readl(hwreg) & (1 << 31)) == 0)
			break;
	if (timeout <= 0) {
		printk(KERN_ERR "%s(%p): timeout when enabling\n",
		       __func__, hwreg);
		return -ETIME;
	}

	c = __raw_readl(hwreg);
	c &= ~(1 << 30);	/* clear CLKGATE */
	__raw_writel(c, hwreg);

	if (!just_enable) {
		c = __raw_readl(hwreg);
		c |= (1 << 31);	/* now again set SFTRST */
		__raw_writel(c, hwreg);
		for (timeout = 1000000; timeout > 0; timeout--)
			/* poll until CLKGATE set */
			if (__raw_readl(hwreg) & (1 << 30))
				break;
		if (timeout <= 0) {
			printk(KERN_ERR "%s(%p): timeout when resetting\n",
			       __func__, hwreg);
			return -ETIME;
		}

		c = __raw_readl(hwreg);
		c &= ~(1 << 31);	/* clear SFTRST */
		__raw_writel(c, hwreg);
		for (timeout = 1000000; timeout > 0; timeout--)
			/* still in SFTRST state ? */
			if ((__raw_readl(hwreg) & (1 << 31)) == 0)
				break;
		if (timeout <= 0) {
			printk(KERN_ERR "%s(%p): timeout when enabling "
			       "after reset\n", __func__, hwreg);
			return -ETIME;
		}

		c = __raw_readl(hwreg);
		c &= ~(1 << 30);	/* clear CLKGATE */
		__raw_writel(c, hwreg);
	}
	for (timeout = 1000000; timeout > 0; timeout--)
		/* still in SFTRST state ? */
		if ((__raw_readl(hwreg) & (1 << 30)) == 0)
			break;

	if (timeout <= 0) {
		printk(KERN_ERR "%s(%p): timeout when unclockgating\n",
		       __func__, hwreg);
		return -ETIME;
	}

	return 0;
}

int mxs_reset_block(void __iomem *hwreg, int just_enable)
{
	int try = 10;
	int r;

	while (try--) {
		r = __mxs_reset_block(hwreg, just_enable);
		if (!r)
			break;
		pr_debug("%s: try %d failed\n", __func__, 10 - try);
	}
	return r;
}

static inline void clk_gpmi_enable(void)
{
	uint32_t val;

	val = readl(HW_CLKCTRL_GPMI_ADDR);
	val &= ~(1 << 31);
	writel(val, HW_CLKCTRL_GPMI_ADDR);
}

static inline void clk_gpmi_disable(void)
{
	uint32_t val;

	val = readl(HW_CLKCTRL_GPMI_ADDR);
	val |= 1 << 31;
	writel(val, HW_CLKCTRL_GPMI_ADDR);
}

static void gpmi_set_pinmux(void)
{
	uint32_t val;

	// By default, all NAND pins are GPIO's after reset
	// Connect NAND signals by configuring 2-bit pinmux value for each pin
	// Most NAND pins are the default peripheral = 0b00.
	// Startup already checked PINCTRL present bits
	val = readl(HW_PINCTRL_CTRL_ADDR); // out of reset and gate
	val &= ~(0x3<<30);
	writel(val, HW_PINCTRL_CTRL_ADDR);

	// -----------------------------------------------------------------
	// Always power up lower 8 bits of NAND Data bus.
	val = readl(HW_PINCTRL_MUXSEL0_ADDR);
	val &= 0xffff0000; // gpmi data0~7
	writel(val, HW_PINCTRL_MUXSEL0_ADDR);

	// Select the ALE, CLE, WRN, and RDN pins of NAND.
	val = readl(HW_PINCTRL_MUXSEL1_ADDR);
	val &= ~(0x3<<18); // gpmi_rdn
	val &= ~(0x3<<16);  //gpmi wrn
	val &= ~(0x3<<2);  //gpmi ale
	val &= ~(0x3<<0);  //gpmi cle
	writel(val, HW_PINCTRL_MUXSEL1_ADDR);

	// Set the pin drive for the RDN, WRN pin to 12mA.
	val = readl(HW_PINCTRL_DRIVE3_ADDR);
	val &= ~(0x3<<4); // gpmi_rdn
	val |= (0x2<<4);
	val &= ~(0x3<<0); // gpmi_wrn
	val |= (0x2<<0);
	writel(val, HW_PINCTRL_DRIVE3_ADDR);

	// Power up Ready Busy for NAND0
	val = readl(HW_PINCTRL_MUXSEL1_ADDR);
	val &= ~(0x3<<6); // gpmi_rdy0
	writel(val, HW_PINCTRL_MUXSEL1_ADDR);

	// Power up CE0 by setting bit field to b00.
	val = readl(HW_PINCTRL_MUXSEL5_ADDR);
	val &= ~(0x3<<24); // gpmi ce0
	writel(val, HW_PINCTRL_MUXSEL5_ADDR);

	// this is called gpmi_wpn in data spec!
	val = readl(HW_PINCTRL_MUXSEL1_ADDR);
	val &= ~(0x3<<14); // gpmi_wpn
	writel(val, HW_PINCTRL_MUXSEL1_ADDR);
}

static int gpmi_round_cycles(uint32_t ns, uint32_t period, uint32_t max_cycles)
{
	int i, cycle_time = period;

	// Assume a maximum of 15 tests
	for (i = 1; i < max_cycles; i++) {
		if (cycle_time > ns)
			break;
		else
			cycle_time += period;
	}

	return i;
}

static void gpmi_set_timming(const struct gpmi_timing *timing, uint32_t period)
{
	uint32_t val;
	uint32_t t_as;
	uint32_t t_ds;
	uint32_t t_dh;
	uint32_t t_dsample;

	// If u32GpmiPeriod is passed in as 0, we'll use the default 41nsec
	// for a 24MHz clock.
	if (period == 0)
		period = 42;

	t_as = gpmi_round_cycles(timing->t_as, period, 20);
	t_ds = gpmi_round_cycles(timing->t_ds, period, 20);
	t_dh = gpmi_round_cycles(timing->t_dh, period, 20);

	HW_GPMI_TIMING0_WR(BF_GPMI_TIMING0_ADDRESS_SETUP(t_as) |
			   BF_GPMI_TIMING0_DATA_HOLD(t_ds) |
			   BF_GPMI_TIMING0_DATA_SETUP(t_dh));

	// DSAMPLE is calculated in 1/2 GPMI clock units, so use shifts to compensate.
	// This one should not round up so I subtract the cycle back off.
	t_dsample = gpmi_round_cycles((timing->t_dsample + (period >> 2)), (period >> 1), 20) - 1;

	val = readl(HW_GPMI_CTRL1_ADDR);
	val &= ~(0xf << 12);
	val |= ((t_dsample & 0xf) << 12);
	writel(val, HW_GPMI_CTRL1_ADDR);

	/*
	 * busy timeout
	 */
	writel(timing->t_busytimeout << 16, HW_GPMI_TIMING1_ADDR);

	pr_debug("GPMI TIME0: 0x%x\n", readl(HW_GPMI_TIMING0_ADDR));
	pr_debug("GPMI TIME1: 0x%x\n", readl(HW_GPMI_TIMING1_ADDR));
}

static int nand_enable_gpmi(void)
{
	const struct gpmi_timing timing = {
		.t_ds = 12, 
		.t_dh = 5, 
		.t_as = 12, 
		.t_dsample = 10, 
		.t_busytimeout = 0x132, 
	};

	clk_gpmi_enable();

	mxs_reset_block((void *)HW_GPMI_CTRL0_ADDR, 0);

	gpmi_set_timming(&timing, 0);

	gpmi_set_pinmux();

	/* Choose NAND mode */
	REG_CLR(HW_GPMI_CTRL1_ADDR, BM_GPMI_CTRL1_GPMI_MODE);

	/* Set the IRQ polarity */
	//REG_SET(HW_GPMI_CTRL1_ADDR, BM_GPMI_CTRL1_ATA_IRQRDY_POLARITY);

	/* Dev-reset disable, Disable write protection */
	REG_SET(HW_GPMI_CTRL1_ADDR, BM_GPMI_CTRL1_DEV_RESET);

	/* Select BCH ECC */
	REG_SET(HW_GPMI_CTRL1_ADDR, BM_GPMI_CTRL1_BCH_MODE);

	clk_gpmi_disable();

	return 0;
}

static void gpmi_wait_busy(void)
{
	while (HW_GPMI_DEBUG_RD() & BM_GPMI_DEBUG_BUSY)
		/* wait */;
}

static void gpmi_send_cmd(uint16_t cmd)
{
	uint32_t val;

	val = HW_GPMI_CTRL0_RD();

	/* Write mode */
	val &= ~BM_GPMI_CTRL0_COMMAND_MODE;
	val |= BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__WRITE);

	/* CLE */
	val &= ~BM_GPMI_CTRL0_ADDRESS;
	val |= BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_CLE);

	HW_GPMI_CTRL0_WR(val);

	/* command */
	HW_GPMI_DATA_WR(cmd & 0xff);
}

static void gpmi_read_data(uint16_t xfer_count)
{
	uint32_t val;

	val = HW_GPMI_CTRL0_RD();

	/* Write mode */
	val &= ~BM_GPMI_CTRL0_COMMAND_MODE;
	val |= BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__READ);

	/* CLE */
	val &= ~BM_GPMI_CTRL0_ADDRESS;
	val |= BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_DATA);

	/* xfer count */
	val &= ~BM_GPMI_CTRL0_XFER_COUNT;
	val |= BF_GPMI_CTRL0_XFER_COUNT(xfer_count);

	HW_GPMI_CTRL0_WR(val);
}

static void gpmi_send_addr(uint8_t addr)
{
	uint32_t val;

	val = HW_GPMI_CTRL0_RD();

	/* Write mode */
	val &= ~BM_GPMI_CTRL0_COMMAND_MODE;
	val |= BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__WRITE);

	/* CLE */
	val &= ~BM_GPMI_CTRL0_ADDRESS;
	val |= BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_ALE);

	HW_GPMI_CTRL0_WR(val);

	/* Address */
	HW_GPMI_DATA_WR(addr & 0xff);
}

static void gpmi_init(void)
{
	REG_SET(HW_GPMI_CTRL0_ADDR, BM_GPMI_CTRL0_RUN);
	nand_enable_gpmi();
	//mxs_reset_block(BCH_CTRL_ADDR);

	clk_gpmi_enable();

	REG_CLR(HW_GPMI_CTRL0_ADDR, BM_GPMI_CTRL0_RUN);

	/* Disable IRQ */
	REG_CLR(HW_GPMI_CTRL0_ADDR, BM_GPMI_CTRL0_TIMEOUT_IRQ_EN);

	/* 8-bit Data Bus */
	REG_SET(HW_GPMI_CTRL0_ADDR, BM_GPMI_CTRL0_WORD_LENGTH);

	/* LOCK-CS */
	REG_SET(HW_GPMI_CTRL0_ADDR, BM_GPMI_CTRL0_LOCK_CS);

	/* Select chip */
	REG_SET(HW_GPMI_CTRL0_ADDR, BF_GPMI_CTRL0_CS(0));

	/* reset */
	gpmi_send_cmd(0xff);
	gpmi_wait_busy();


	clk_gpmi_disable();
}

static void gpmi_read_id(void)
{
	uint8_t buffer[4];
	uint8_t *p = buffer;
	uint32_t length = 4;

	clk_gpmi_enable();

	/* read id */
	gpmi_send_cmd(0x90);
	gpmi_wait_busy();

	gpmi_read_data(4);
	gpmi_wait_busy();

	printf("%s: HW_GPMI_STAT_RD() = %08x\n", __func__, HW_GPMI_STAT_RD());
	while (length && !(HW_GPMI_STAT_RD() & BF_GPMI_STAT_DEV0_ERROR(0))) {
		if (HW_GPMI_STAT_RD() & BM_GPMI_STAT_FIFO_EMPTY) {
			*p = HW_GPMI_DATA_RD();
			printf("%s: %x\n", __func__, *p);
			p++;
			length -= 1;
		}
		printf("%s: HW_GPMI_STAT_RD() = %08x\n", __func__, HW_GPMI_STAT_RD());
	}

	clk_gpmi_disable();
}

static int do_mxnand(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	uint32_t len = 0;
	void *from = NULL, *to = NULL;
	char *cmd = argv[1];

	if (argc < 2) {
		pr_err("Invalid argument\n");
		return 0;
	}

	if (argc == 5) {
		from = (void *)simple_strtoul(argv[2], NULL, 16);
		to = (void *)simple_strtoul(argv[3], NULL, 16);
		len = simple_strtoul(argv[4], NULL, 16);
		pr_debug("from = %p, to = %p, len = 0x%x\n", from, to, len);
	}

	cmd = argv[1];
	if (!strcmp(cmd, "init"))
		gpmi_init();
	else if (!strcmp(cmd, "readid"))
		gpmi_read_id();

	return 0;
}

U_BOOT_CMD(gpmi, 5, 1, do_mxnand,
	   "GPMI driver test for mx233",
	   "gpmi init\n"
	   "gpmi test\n"
	   "gpmi read from_addr to_addr len\n"
	   "gpmi write from_addr to_addr len\n"
	   "gpmi erase");

