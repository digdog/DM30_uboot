/*
 * ==========================================================================
 *
 *       Filename:  mx23_lcd_clk.c
 *
 *    Description:  
 *
 *        Version:  0.01
 *        Created:  2011年05月16日 14时51分05秒
 *
 *         Author:  smmei (), 
 *        Company:  
 *
 * ==========================================================================
 */

#include <common.h>
#include <asm/io.h>

#define BIT_SET 0x04
#define BIT_CLR 0x08
#define BIT_TGL 0x0C

#define IMX_CCM_BASE 0x80040000

# define HW_CLKCTRL_DIS_LCDIF 0x060
#  define CLKCTRL_DIS_LCDIF_GATE (1 << 31)
#  define CLKCTRL_DIS_LCDIF_BUSY (1 << 29)
#  define MASK_DIS_LCDIF_DIV 0xfff
#  define SET_DIS_LCDIF_DIV(x) ((x) & MASK_DIS_LCDIF_DIV)
#  define GET_DIS_LCDIF_DIV(x) ((x) & MASK_DIS_LCDIF_DIV)

# define HW_CLKCTRL_FRAC 0xf0
#  define MASK_PIXFRAC 0x3f
#  define GET_PIXFRAC(x) (((x) >> 16) & MASK_PIXFRAC)
#  define SET_PIXFRAC(x) (((x) & MASK_PIXFRAC) << 16)
#  define CLKCTRL_FRAC_CLKGATEPIX (1 << 23)

# define HW_CLKCTRL_CLKSEQ 0x110
#  define CLKCTRL_CLKSEQ_BYPASS_DIS_LCDIF (1 << 1)

#define abs(x) ({                               \
		long __x = (x);                 \
		(__x < 0) ? -__x : __x;         \
	})

extern unsigned imx_get_mpllclk(void);

unsigned imx_get_lcdifclk(void)
{
	unsigned rate = (imx_get_mpllclk() / 1000) * 18U;
	unsigned div;

	div = GET_PIXFRAC(readl(IMX_CCM_BASE + HW_CLKCTRL_FRAC));
	if (div != 0U) {
		rate /= div;
		div = GET_DIS_LCDIF_DIV(readl(IMX_CCM_BASE +
							HW_CLKCTRL_DIS_LCDIF));
		if (div != 0U)
			rate /= div;
		else
			debug("LCDIF clock has divisor 0!\n");
	} else
		debug("LCDIF clock has frac divisor 0!\n");

	return rate * 1000;
}

/*
 * The source of the pixel clock can be the external 24 MHz crystal or the
 * internal PLL running at 480 MHz. In order to support at least VGA sized
 * displays/resolutions this routine forces the PLL as the clock source.
 */
unsigned imx_set_lcdifclk(unsigned nc)
{
	unsigned frac, best_frac = 0, div, best_div = 0, result;
	int delta, best_delta = 0xffffff;
	unsigned i, parent_rate = imx_get_mpllclk() / 1000;
	uint32_t reg;

#define SH_DIV(NOM, DEN, LSH) ((((NOM) / (DEN)) << (LSH)) + \
			DIV_ROUND_CLOSEST(((NOM) % (DEN)) << (LSH), DEN))
#define SHIFT 4

	nc /= 1000;
	nc <<= SHIFT;

	for (frac = 18; frac <= 35; ++frac) {
		for (div = 1; div <= 255; ++div) {
			result = DIV_ROUND_CLOSEST(parent_rate *
						   SH_DIV(18U, frac, SHIFT), div);
			delta = nc - result;
			if (abs(delta) < abs(best_delta)) {
				best_delta = delta;
				best_frac = frac;
				best_div = div;
			}
		}
	}

	if (best_delta == 0xffffff) {
		debug("Unable to match the pixelclock\n");
		return 0;
	}

	debug("Programming PFD=%u,DIV=%u ref_pix=%u MHz PIXCLK=%u kHz\n",
			best_frac, best_div, 480 * 18 / best_frac,
			480000 * 18 / best_frac / best_div);

	reg = readl(IMX_CCM_BASE + HW_CLKCTRL_FRAC);
	reg &= ~SET_PIXFRAC(MASK_PIXFRAC);
	reg |= SET_PIXFRAC(best_frac);
	writel(reg, IMX_CCM_BASE + HW_CLKCTRL_FRAC);
	writel(reg & ~CLKCTRL_FRAC_CLKGATEPIX, IMX_CCM_BASE + HW_CLKCTRL_FRAC);

	reg = readl(IMX_CCM_BASE + HW_CLKCTRL_DIS_LCDIF) & ~MASK_DIS_LCDIF_DIV;
	reg &= ~CLKCTRL_DIS_LCDIF_GATE;
	reg |= SET_DIS_LCDIF_DIV(best_div);
	writel(reg, IMX_CCM_BASE + HW_CLKCTRL_DIS_LCDIF);

	/* Wait for divider update */
	for (i = 0; i < 10000; i++) {
		if (!(readl(IMX_CCM_BASE + HW_CLKCTRL_DIS_LCDIF) &
		      CLKCTRL_DIS_LCDIF_BUSY))
			break;
	}

	if (i >= 10000) {
		debug("Setting LCD clock failed\n");
		return 0;
	}

	writel(CLKCTRL_CLKSEQ_BYPASS_DIS_LCDIF,
	       IMX_CCM_BASE + HW_CLKCTRL_CLKSEQ + BIT_CLR);

	return imx_get_lcdifclk();
}

void imx_lcdif_clk_enable(void)
{
	writel(1 << 31, IMX_CCM_BASE + HW_CLKCTRL_FRAC + BIT_CLR);
}

void imx_lcdif_clk_disable(void)
{
	writel(1 << 31, IMX_CCM_BASE + HW_CLKCTRL_FRAC + BIT_SET);
}
