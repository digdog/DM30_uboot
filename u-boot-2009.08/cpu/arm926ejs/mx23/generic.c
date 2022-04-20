/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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

#include <common.h>
#include <asm/errno.h>
#include <asm/arch/regs-clkctrl.h>
#include <asm/cache-cp15.h>
#include <asm/fec.h>
#include <asm/io.h>
#include <asm/arch/mxs-block.h>

/*
 * Initializes on-chip MMC controllers.
 */
#if defined(CONFIG_MMC_MX23)
int imx_ssp_mmc_initialize(bd_t *bis);
#endif

int cpu_mmc_init(bd_t *bis)
{
	int rc = ENODEV;
#if defined(CONFIG_MMC_MX23)
	rc = imx_ssp_mmc_initialize(bis);
#endif
	return rc;
}

unsigned imx_get_mpllclk(void)
{
	/* the main PLL runs at 480 MHz */
	return 480000000;
}

unsigned imx_get_xtalclk(void)
{
	/* the external reference runs at 24 MHz */
	return 24000000;
}

# define GET_IOFRAC(x) (((x) >> 24) & 0x3f)
# define GET_CPU_XTAL_DIV(x) (((x) >> 16) & 0x3ff)
# define GET_CPU_PLL_DIV(x) ((x) & 0x3f)
# define GET_CPUFRAC(x) ((x) & 0x3f)
# define GET_EMIFRAC(x) (((x) >> 8) & 0x3f)
# define GET_EMI_PLL_DIV(x) ((x) & 0x3f)
# define CLKCTRL_SSP_DIV_MASK 0x1ff
# define GET_SSP_DIV(x) ((x) & CLKCTRL_SSP_DIV_MASK)
# define GET_EMI_XTAL_DIV(x) (((x) >> 8) & 0xf)

/*
 * Source of ssp, gpmi, ir
 */
unsigned imx_get_ioclk(void)
{
	uint32_t reg;
	unsigned rate = imx_get_mpllclk() / 1000;

	reg = readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_FRAC);
	if (reg & BM_CLKCTRL_FRAC_CLKGATEIO)
		return 0U;	/* clock is off */

	rate *= 18U;
	rate /= GET_IOFRAC(reg);
	return rate * 1000;
}

/**
 * Setup a new frequency to the IOCLK domain.
 * @param nc New frequency in [Hz]
 *
 * The FRAC divider for the IOCLK must be between 18 (* 18/18) and 35 (* 18/35)
 */
unsigned imx_set_ioclk(unsigned nc)
{
	uint32_t reg;
	unsigned div;

	nc /= 1000;
	div = (imx_get_mpllclk() / 1000) * 18;
	div = DIV_ROUND_CLOSEST(div, nc);
	if (div > 0x3f)
		div = 0x3f;

	/* mask the current settings */
	reg = readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_FRAC) & ~BM_CLKCTRL_FRAC_IOFRAC;
	writel(reg | BF_CLKCTRL_FRAC_IOFRAC(div), REGS_CLKCTRL_BASE + HW_CLKCTRL_FRAC);

	/* enable the IO clock at its new frequency */
	writel(BM_CLKCTRL_FRAC_CLKGATEIO, REGS_CLKCTRL_BASE + HW_CLKCTRL_FRAC + 8);

	return imx_get_ioclk();
}

/* 'index' gets ignored on i.MX23 */
unsigned imx_get_sspclk(unsigned index)
{
	unsigned rate;

	if (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_SSP) & BM_CLKCTRL_SSP_CLKGATE)
		return 0U;	/* clock is off */

	if (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_CLKSEQ) & BM_CLKCTRL_CLKSEQ_BYPASS_SSP)
		rate = imx_get_xtalclk();
	else
		rate = imx_get_ioclk();

	return rate / GET_SSP_DIV(readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_SSP));
}

/**
 * @param nc New frequency in [Hz]
 * @param high != 0 if ioclk should be the source
 * @return The new possible frequency in [kHz]
 */
unsigned imx_set_sspclk(unsigned index, unsigned nc, int high)
{
	uint32_t reg;
	unsigned ssp_div;

	reg = readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_SSP) & ~BM_CLKCTRL_SSP_CLKGATE;
	/* Datasheet says: Do not change the DIV setting if the clock is off */
	writel(reg, REGS_CLKCTRL_BASE + HW_CLKCTRL_SSP);
	/* Wait while clock is gated */
	while (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_SSP) & BM_CLKCTRL_SSP_CLKGATE)
		;

	if (high)
		ssp_div = imx_get_ioclk();
	else
		ssp_div = imx_get_xtalclk();

	if (nc > ssp_div) {
		printf("Cannot setup SSP unit clock to %u Hz, base clock is only %u Hz\n", nc, ssp_div);
		ssp_div = 1U;
	} else {
		ssp_div = DIV_ROUND_UP(ssp_div, nc);
		if (ssp_div > BM_CLKCTRL_SSP_DIV)
			ssp_div = BM_CLKCTRL_SSP_DIV;
	}

	/* Set new divider value */
	reg = readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_SSP) & ~BM_CLKCTRL_SSP_DIV;
	writel(reg | BF_CLKCTRL_SSP_DIV(ssp_div), REGS_CLKCTRL_BASE + HW_CLKCTRL_SSP);

	/* Wait until new divider value is set */
	while (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_SSP) & BM_CLKCTRL_SSP_BUSY)
		;

	if (high)
		/* switch to ioclock */
		writel(BM_CLKCTRL_CLKSEQ_BYPASS_SSP, REGS_CLKCTRL_BASE + HW_CLKCTRL_CLKSEQ + 8);
	else
		/* switch to 24 MHz crystal */
		writel(BM_CLKCTRL_CLKSEQ_BYPASS_SSP, REGS_CLKCTRL_BASE + HW_CLKCTRL_CLKSEQ + 4);

	return imx_get_sspclk(index);
}

unsigned imx_get_gpmiclk(void)
{
	unsigned rate;

	if (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_GPMI) & BM_CLKCTRL_GPMI_CLKGATE)
		return 0U;	/* clock is off */

	if (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_CLKSEQ) & BM_CLKCTRL_CLKSEQ_BYPASS_GPMI)
		rate = imx_get_xtalclk();
	else
		rate = imx_get_ioclk();

	return rate / (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_GPMI) & BM_CLKCTRL_GPMI_DIV);
}

unsigned imx_set_gpmiclk(unsigned nc, int high)
{
	uint32_t reg;
	unsigned gpmi_div;

	reg = readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_GPMI) & ~BM_CLKCTRL_GPMI_CLKGATE;
	/* Datasheet says: Do not change the DIV setting if the clock is off */
	writel(reg, REGS_CLKCTRL_BASE + HW_CLKCTRL_GPMI);
	/* Wait while clock is gated */
	while (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_GPMI) & BM_CLKCTRL_GPMI_CLKGATE)
		;

	if (high)
		gpmi_div = imx_get_ioclk();
	else
		gpmi_div = imx_get_xtalclk();

	if (nc > gpmi_div) {
		printf("Cannot setup GPMI unit clock to %u Hz, base clock is only %u Hz\n", nc, gpmi_div);
		gpmi_div = 1U;
	} else {
		gpmi_div = DIV_ROUND_UP(gpmi_div, nc);
		if (gpmi_div > BM_CLKCTRL_GPMI_DIV)
			gpmi_div = BM_CLKCTRL_GPMI_DIV;
	}

	/* Set new divider value */
	reg = readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_GPMI) & ~BM_CLKCTRL_GPMI_DIV;
	writel(reg | BF_CLKCTRL_GPMI_DIV(gpmi_div), REGS_CLKCTRL_BASE + HW_CLKCTRL_GPMI);

	/* Wait until new divider value is set */
	while (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_GPMI) & BM_CLKCTRL_GPMI_BUSY)
		;

	if (high)
		/* switch to ioclock */
		writel(BM_CLKCTRL_CLKSEQ_BYPASS_GPMI, REGS_CLKCTRL_BASE + HW_CLKCTRL_CLKSEQ + 8);
	else
		/* switch to 24 MHz crystal */
		writel(BM_CLKCTRL_CLKSEQ_BYPASS_GPMI, REGS_CLKCTRL_BASE + HW_CLKCTRL_CLKSEQ + 4);

	return imx_get_gpmiclk();
}

/* this is CPU core clock */
unsigned imx_get_armclk(void)
{
	uint32_t reg;
	unsigned rate;

	if (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_CLKSEQ) & BM_CLKCTRL_CLKSEQ_BYPASS_CPU)
		return imx_get_xtalclk() / GET_CPU_XTAL_DIV(readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_CPU));

	reg = readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_FRAC);
	if (reg & BM_CLKCTRL_FRAC_CLKGATECPU)
		return 0U;	/* should not possible, shouldn't it? */

	rate = imx_get_mpllclk() / 1000;
	rate *= 18U;
	rate /= GET_CPUFRAC(reg);

	return (rate / GET_CPU_PLL_DIV(readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_CPU)))
			* 1000;
}

/* used for the SDRAM controller */
unsigned imx_get_emiclk(void)
{
	uint32_t reg;
	unsigned rate;

	if (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_EMI) & BM_CLKCTRL_EMI_CLKGATE)
		return 0U;	/* clock is off */

	if (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_CLKSEQ) & BM_CLKCTRL_CLKSEQ_BYPASS_EMI)
		return imx_get_xtalclk() / GET_EMI_XTAL_DIV(readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_EMI));

	rate = imx_get_mpllclk() / 1000;
	reg = readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_FRAC);
	if (!(reg & BM_CLKCTRL_FRAC_CLKGATEEMI)) {
		rate *= 18U;
		rate /= GET_EMIFRAC(reg);
	}

	return (rate / GET_EMI_PLL_DIV(readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_EMI)))
			* 1000;
}

/* this is the AHB and APBH bus clock */
unsigned imx_get_hclk(void)
{
	unsigned rate = imx_get_armclk() / 1000;

	if (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_HBUS) & 0x20) {
		rate *= readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_HBUS) & 0x1f;
		rate >>= 5U; /* / 32 */
	} else
		rate /= readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_HBUS) & 0x1f;
	return rate * 1000;
}

/*
 * Source of UART, debug UART, audio, PWM, dri, timer, digctl
 */
unsigned imx_get_xclk(void)
{
	unsigned rate = imx_get_xtalclk();	/* runs from the 24 MHz crystal reference */

	return rate / (readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_XBUS) & 0x3ff);
}

extern unsigned imx_get_lcdifclk(void);

void imx_dump_clocks(void)
{
	printf("mpll:    %10u kHz\n", imx_get_mpllclk() / 1000);
	printf("arm:     %10u kHz\n", imx_get_armclk() / 1000);
	printf("ioclk:   %10u kHz\n", imx_get_ioclk() / 1000);
	printf("emiclk:  %10u kHz\n", imx_get_emiclk() / 1000);
	printf("hclk:    %10u kHz\n", imx_get_hclk() / 1000);
	printf("xclk:    %10u kHz\n", imx_get_xclk() / 1000);
	printf("ssp:     %10u kHz\n", imx_get_sspclk(0) / 1000);
	printf("gpmi:    %10u kHz\n", imx_get_gpmiclk() / 1000);
	printf("lcdif:   %10u kHz\n", imx_get_lcdifclk() / 1000);
}

#if defined(CONFIG_DISPLAY_CPUINFO)
int print_cpuinfo(void)
{
	printf("Freescale i.MX23 family\n");
	imx_dump_clocks();
	return 0;
}
#endif

static int __mxs_reset_block(void *hwreg, int just_enable)
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
		printf("%s(%p): timeout when enabling\n",
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
			printf("%s(%p): timeout when resetting\n",
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
			printf("%s(%p): timeout when enabling "
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
		printf("%s(%p): timeout when unclockgating\n",
		       __func__, hwreg);
		return -ETIME;
	}

	return 0;
}

int mxs_reset_block(void *hwreg, int just_enable)
{
	int try = 10;
	int r;

	while (try--) {
		r = __mxs_reset_block(hwreg, just_enable);
		if (!r)
			break;
		printf("%s: try %d failed\n", __func__, 10 - try);
	}
	return r;
}
