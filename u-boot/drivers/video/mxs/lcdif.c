/*
 * ==========================================================================
 *
 *       Filename:  lcdif.c
 *
 *    Description:  
 *
 *        Version:  0.01
 *        Created:  2011年05月18日 12时29分47秒
 *
 *         Author:  smmei (), 
 *        Company:  
 *
 * ==========================================================================
 */

#include <common.h>
#include <asm/io.h>
#include "lcdif.h"

void mxs_init_lcdif(void)
{
	__raw_writel(BM_LCDIF_CTRL_CLKGATE, HW_LCDIF_CTRL_CLR_ADDR);
	/* Reset controller */
	__raw_writel(BM_LCDIF_CTRL_SFTRST, HW_LCDIF_CTRL_SET_ADDR);
	udelay(10);

	/* Take controller out of reset */
	__raw_writel(BM_LCDIF_CTRL_SFTRST | BM_LCDIF_CTRL_CLKGATE,
		     HW_LCDIF_CTRL_CLR_ADDR);

	/* Setup the bus protocol */
	__raw_writel(BM_LCDIF_CTRL1_MODE86, HW_LCDIF_CTRL1_CLR_ADDR);
	__raw_writel(BM_LCDIF_CTRL1_BUSY_ENABLE, HW_LCDIF_CTRL1_CLR_ADDR);

#if 0
	/* Take display out of reset */
	__raw_writel(BM_LCDIF_CTRL1_RESET, HW_LCDIF_CTRL1_SET_ADDR);

	/* Reset display */
	__raw_writel(BM_LCDIF_CTRL1_RESET, HW_LCDIF_CTRL1_CLR_ADDR);
	udelay(10);
	__raw_writel(BM_LCDIF_CTRL1_RESET, HW_LCDIF_CTRL1_SET_ADDR);
	udelay(10);
#endif
}

int mxs_lcdif_dma_init(dma_addr_t phys, int memsize)
{
	__raw_writel(BM_LCDIF_CTRL_LCDIF_MASTER, HW_LCDIF_CTRL_SET_ADDR);

	__raw_writel(phys, HW_LCDIF_CUR_BUF_ADDR);
	__raw_writel(phys, HW_LCDIF_NEXT_BUF_ADDR);

	return 0;
}

void mxs_lcdif_dma_release(void)
{
	__raw_writel(BM_LCDIF_CTRL_LCDIF_MASTER, HW_LCDIF_CTRL_CLR_ADDR);
}

void mxs_lcdif_run(void)
{
	__raw_writel(BM_LCDIF_CTRL_LCDIF_MASTER, HW_LCDIF_CTRL_SET_ADDR);
	__raw_writel(BM_LCDIF_CTRL_RUN, HW_LCDIF_CTRL_SET_ADDR);

	//while(__raw_readl(HW_LCDIF_CTRL_ADDR) & BM_LCDIF_CTRL_RUN) ;
}

void mxs_lcdif_stop(void)
{
	__raw_writel(BM_LCDIF_CTRL_RUN, HW_LCDIF_CTRL_CLR_ADDR);
	__raw_writel(BM_LCDIF_CTRL_LCDIF_MASTER, HW_LCDIF_CTRL_CLR_ADDR);
	udelay(100);
}

int mxs_lcdif_pan_display(dma_addr_t addr)
{
	__raw_writel(addr, HW_LCDIF_NEXT_BUF_ADDR);

	return 0;
}
