/*
 * ==========================================================================
 *
 *       Filename:  mxs-dma.c
 *
 *    Description:  
 *
 *        Version:  0.01
 *        Created:  2011年07月18日 13时41分39秒
 *
 *         Author:  smmei (), 
 *        Company:  
 *
 * ==========================================================================
 */

#include 	<common.h>
#include 	<malloc.h>
#include 	<asm/io.h>
#include 	<asm/errno.h>
#include 	<asm/arch/registers/hw_dma.h>
#include 	<asm/arch/registers/regsapbh.h>
#include 	<asm/arch/registers/regsapbx.h>
#include 	<asm/arch/mxs-dma.h>

int mxs_dma_apbh_enable(void *pchain, unsigned int chan)
{
	if (!pchain)
		return -EFAULT;

	__raw_writel((uint32_t)pchain, HW_APBH_CHn_NXTCMDAR_ADDR(chan));
	__raw_writel(1, HW_APBH_CHn_SEMA_ADDR(chan));
	__raw_writel(1 << chan, HW_APBH_CTRL0_CLR_ADDR);

	return 0;
}

void mxs_dma_apbh_disable(unsigned int chan)
{
#if 0
	__raw_writel(1 << (chan + BP_APBH_CTRL0_CLKGATE_CHANNEL),
		     HW_APBH_CTRL0_SET_ADDR);
#endif
}

void mxs_dma_apbh_reset(unsigned int chan)
{
	__raw_writel(1 << (chan + BP_APBH_CTRL0_RESET_CHANNEL),
		     HW_APBH_CTRL0_SET_ADDR);
}

void mxs_dma_apbh_freeze(unsigned int chan)
{
	__raw_writel(1 << (chan + BP_APBH_CTRL0_FREEZE_CHANNEL),
		     HW_APBH_CTRL0_SET_ADDR);
}

void mxs_dma_apbh_unfreeze(unsigned int chan)
{
	__raw_writel(1 << (chan + BP_APBH_CTRL0_FREEZE_CHANNEL),
		     HW_APBH_CTRL0_CLR_ADDR);
}

int mxs_dma_apbh_read_semaphore(unsigned int chan)
{
	unsigned int reg;
	reg = __raw_readl(HW_APBH_CHn_SEMA_ADDR(chan));
	return (reg & BM_APBH_CHn_SEMA_PHORE) >> BP_APBH_CHn_SEMA_PHORE;
}

void mxs_dma_apbh_enable_irq(unsigned int chan, int enable)
{
	if (enable) {
		__raw_writel(1 << (chan + 16), HW_APBH_CTRL1_SET_ADDR);
	} else {
		__raw_writel(1 << (chan + 16), HW_APBH_CTRL1_CLR_ADDR);
	}
}

int mxs_dma_apbh_irq_is_pending(unsigned int chan)
{
	unsigned int reg;
	reg = __raw_readl(HW_APBH_CTRL1_ADDR);
	reg |= __raw_readl(HW_APBH_CTRL2_ADDR);
	return reg & (1 << chan);
}

void mxs_dma_apbh_ack_irq(unsigned int chan)
{
	__raw_writel(1 << chan, HW_APBH_CTRL1_CLR_ADDR);
	__raw_writel(1 << chan, HW_APBH_CTRL2_CLR_ADDR);
}

/* APBX */

int mxs_dma_apbx_enable(uint32_t pdesc, unsigned int chan)
{
	if (!pdesc)
		return -EFAULT;

	__raw_writel(pdesc, HW_APBX_CHn_NXTCMDAR_ADDR(chan));
	__raw_writel(1, HW_APBX_CHn_SEMA_ADDR(chan));
	return 0;
}

void mxs_dma_apbx_disable(unsigned int chan)
{
	__raw_writel(0, HW_APBX_CHn_SEMA_ADDR(chan));
}

void mxs_dma_apbx_reset(unsigned int chan)
{
	__raw_writel(1 << (chan + BP_APBX_CHANNEL_CTRL_RESET_CHANNEL),
		     HW_APBX_CHANNEL_CTRL_SET_ADDR);
}

void mxs_dma_apbx_freeze(unsigned int chan)
{
	__raw_writel(1 << chan, HW_APBX_CHANNEL_CTRL_SET_ADDR);
}

void mxs_dma_apbx_unfreeze(unsigned int chan)
{
	__raw_writel(1 << chan, HW_APBX_CHANNEL_CTRL_CLR_ADDR);
}

void mxs_dma_apbx_enable_irq(unsigned int chan, int enable)
{
	if (enable) {
		__raw_writel(1 << (chan + 16), HW_APBX_CTRL1_SET_ADDR);
	} else {
		__raw_writel(1 << (chan + 16), HW_APBX_CTRL1_CLR_ADDR);
	}
}

int mxs_dma_apbx_irq_is_pending(unsigned int chan)
{
	unsigned int reg;
	reg = __raw_readl(HW_APBX_CTRL1_ADDR);
	reg |= __raw_readl(HW_APBX_CTRL2_ADDR);
	return reg & (1 << chan);
}

void mxs_dma_apbx_ack_irq(unsigned int chan)
{
	__raw_writel(1 << chan, HW_APBX_CTRL1_CLR_ADDR);
	__raw_writel(1 << chan, HW_APBX_CTRL2_CLR_ADDR);
}

/**
 * mxs_dma_alloc_desc - alloc memory for 'struct mxs_dma_desc'
 */
struct mxs_dma_desc *mxs_dma_alloc_desc(void)
{
	struct mxs_dma_desc *pdesc;

	pdesc = malloc(sizeof(*pdesc));

	return pdesc;
};


void mxs_dma_free_desc(struct mxs_dma_desc *pdesc)
{
	if (pdesc)
		free(pdesc);
}

int mxs_dma_desc_append(int channel, struct mxs_dma_desc *pdesc)
{
	return 0;
}
