/*
 * --------------------------------------------------------------------------
 *
 *       Filename:  mxs-apbh-dma.h
 *
 *    Description:  
 *
 *        Version:  0.01
 *        Created:  2011年07月18日 17时22分15秒
 *
 *         Author:  smmei (), 
 *        Company:  
 * --------------------------------------------------------------------------
 */

#ifndef __MXS_DMA_H
#define __MXS_DMA_H

/**
 * struct mxs_dma_cmd_bits - MXS DMA hardware command bits.
 *
 * This structure describes the in-memory layout of the command bits in a DMA
 * command. See the appropriate reference manual for a detailed description
 * of what these bits mean to the DMA hardware.
 */
struct mxs_dma_cmd_bits {
	unsigned int command:2;
#define NO_DMA_XFER	0x00
#define DMA_WRITE	0x01
#define DMA_READ	0x02
#define DMA_SENSE	0x03

	unsigned int chain:1;
	unsigned int irq:1;
	unsigned int nand_lock:1;
	unsigned int nand_wait_4_ready:1;
	unsigned int dec_sem:1;
	unsigned int wait4end:1;
	unsigned int halt_on_terminate:1;
	unsigned int terminate_flush:1;
	unsigned int resv2:2;
	unsigned int pio_words:4;
	unsigned int bytes:16;
};

/**
 * struct mxs_dma_cmd - MXS DMA hardware command.
 *
 * This structure describes the in-memory layout of an entire DMA command,
 * including space for the maximum number of PIO accesses. See the appropriate
 * reference manual for a detailed description of what these fields mean to the
 * DMA hardware.
 */

#define DMA_PIO_WORDS	15

struct mxs_dma_cmd {
	unsigned long next;
	union {
		unsigned long data;
		struct mxs_dma_cmd_bits bits;
	} cmd;
	union {
		dma_addr_t address;
		unsigned long alternate;
	};
	unsigned long pio_words[DMA_PIO_WORDS];
};

/**
 * struct mxs_dma_desc - MXS DMA command descriptor.
 *
 * This structure incorporates an MXS DMA hardware command structure, along
 * with metadata.
 *
 * @cmd:      The MXS DMA hardware command block.
 * @flags:    Flags that represent the state of this DMA descriptor.
 * @address:  The physical address of this descriptor.
 * @buffer:   A convenient place for software to put the virtual address of the
 *            associated data buffer (the physical address of the buffer
 *            appears in the DMA command). The MXS platform DMA software doesn't
 *            use this field -- it is provided as a convenience.
 * @node:     Links this structure into a list.
 */
struct mxs_dma_desc {
	struct mxs_dma_cmd cmd;
#if 0
	unsigned int flags;
#define MXS_DMA_DESC_READY 0x80000000
#define MXS_DMA_DESC_FIRST 0x00000001
#define MXS_DMA_DESC_LAST  0x00000002
	dma_addr_t address;
	void *buffer;
	struct list_head node;
#endif
};

/* apbh */
extern int mxs_dma_apbh_enable(void *pchain, unsigned int chan);
extern void mxs_dma_apbh_disable(unsigned int chan);
extern void mxs_dma_apbh_reset(unsigned int chan);
extern void mxs_dma_apbh_freeze(unsigned int chan);
extern void mxs_dma_apbh_unfreeze(unsigned int chan);
extern int mxs_dma_apbh_read_semaphore(unsigned int chan);
extern void mxs_dma_apbh_enable_irq(unsigned int chan, int enable);
extern int mxs_dma_apbh_irq_is_pending(unsigned int chan);
extern void mxs_dma_apbh_ack_irq(unsigned int chan);

/* apbx */
extern int mxs_dma_apbx_enable(uint32_t pdesc, unsigned int chan);
extern void mxs_dma_apbx_disable(unsigned int chan);
extern void mxs_dma_apbx_reset(unsigned int chan);
extern void mxs_dma_apbx_freeze(unsigned int chan);
extern void mxs_dma_apbx_unfreeze(unsigned int chan);
extern void mxs_dma_apbx_enable_irq(unsigned int chan, int enable);
extern int mxs_dma_apbx_irq_is_pending(unsigned int chan);
extern void mxs_dma_apbx_ack_irq(unsigned int chan);

extern struct mxs_dma_desc *mxs_dma_alloc_desc(void);
extern void mxs_dma_free_desc(struct mxs_dma_desc *pdesc);
extern int mxs_dma_desc_append(int channel, struct mxs_dma_desc *pdesc);

#endif 	/*  */

