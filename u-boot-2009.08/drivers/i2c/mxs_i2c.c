/*
 * ==========================================================================
 *
 *       Filename:  mxs_i2c.c
 *
 *    Description:  
 *
 *        Version:  0.01
 *        Created:  2011年08月25日 09时48分46秒
 *
 *         Author:  smmei (), 
 *        Company:  
 *
 * ==========================================================================
 */

//#define DEBUG

#include 	<common.h>
#include 	<malloc.h>
#include 	<linux/string.h>
#include 	<asm/io.h>
#include 	<asm/errno.h>
#include 	<asm/arch/registers/hw_dma.h>
#include 	<asm/arch/registers/hw_irq.h>
#include 	<asm/arch/registers/regsi2c.h>
#include 	<asm/arch/registers/regsapbx.h>
#include 	<asm/arch/mxs-dma.h>
#include 	<asm/arch/mxs-block.h>
#include 	<asm/arch/regs-pinctrl.h>
#include 	<asm/arch/pinctrl.h>
#include 	<i2c.h>

#ifndef PAGE_SIZE
#define PAGE_SIZE 	0x1000
#endif

#define IRQ_NONE	0x0
#define IRQ_HANDLED	0x01

#define I2C_READ   1
#define I2C_WRITE  0

struct mxs_i2c_dev {
	struct device		*dev;
	void			*buf;
	unsigned long		regbase;
	u32			flags;
#define	MXS_I2C_DMA_MODE	0x1
#define	MXS_I2C_PIOQUEUE_MODE	0x2
	int			dma_chan;
	int			irq_dma;
	int			irq_err;
	//struct completion	cmd_complete;
	u32			cmd_err;
	//struct i2c_adapter	adapter;
	//spinlock_t		lock;
	//wait_queue_head_t	queue;
};

struct i2c_msg {
	__u16 addr;	/* slave address			*/
	__u16 flags;
#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RD		0x0001	/* read data, from slave to master */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */
	__u16 len;		/* msg length				*/
	__u8 *buf;		/* pointer to msg data			*/
};

/* 2 for read, 1 for write */
#define	NR_DESC		3
static struct mxs_i2c_dev mxs_i2c_dev;
static struct mxs_dma_desc *desc[NR_DESC];
static u8 *i2c_buf;

static inline void mdelay(unsigned long msec)
{
	unsigned long i;
	for (i = 0; i < msec; i++)
		udelay(1000);
}

#define	CMD_I2C_SELECT	(BM_I2C_CTRL0_RETAIN_CLOCK |	\
			BM_I2C_CTRL0_PRE_SEND_START |	\
			BM_I2C_CTRL0_MASTER_MODE |	\
			BM_I2C_CTRL0_DIRECTION |	\
			BF_I2C_CTRL0_XFER_COUNT(1))

#define	CMD_I2C_WRITE	(BM_I2C_CTRL0_PRE_SEND_START |	\
			BM_I2C_CTRL0_MASTER_MODE |	\
			BM_I2C_CTRL0_DIRECTION)

#define	CMD_I2C_READ	(BM_I2C_CTRL0_SEND_NAK_ON_LAST | \
			BM_I2C_CTRL0_POST_SEND_STOP | \
			BM_I2C_CTRL0_MASTER_MODE)

#define pr_debug(fmt, args...)	debug(fmt, ##args)

static struct pin_desc i2c_pins_desc[] = {
	{ PINID_LCD_ENABLE, PIN_FUN2, PAD_4MA, PAD_3V3, 1 },
	{ PINID_LCD_HSYNC,  PIN_FUN2, PAD_4MA, PAD_3V3, 1 },
	{ PINID_GPMI_RDY1,  PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
};

static struct pin_group i2c_pins = {
	.pins		= i2c_pins_desc,
	.nr_pins	= ARRAY_SIZE(i2c_pins_desc)
};

static int mxs_i2c_hw_init_i2c(void)
{
	pin_set_group(&i2c_pins);
	pin_gpio_direction(PINID_GPMI_RDY1, 0);
	return 0;
}

#ifdef DEBUG
int memory_display(char *addr, ulong offs, ulong nbytes, int size)
{
	ulong linebytes, i;
	u_char	*cp;

	#define DISP_LINE_LEN	16
	/* Print the lines.
	 *
	 * We buffer all read data, so we can make sure data is read only
	 * once, and all accesses are with the specified bus width.
	 */
	do {
		char	linebuf[DISP_LINE_LEN];
		uint	*uip = (uint   *)linebuf;
		ushort	*usp = (ushort *)linebuf;
		u_char	*ucp = (u_char *)linebuf;
		uint	count = 52;

		printf("%08lx:", offs);
		linebytes = (nbytes > DISP_LINE_LEN) ? DISP_LINE_LEN : nbytes;

		for (i = 0; i < linebytes; i += size) {
			if (size == 4) {
				printf(" %08x", (*uip++ = *((uint *)addr)));
				count -= 9;
			} else if (size == 2) {
				printf(" %04x", (*usp++ = *((ushort *)addr)));
				count -= 5;
			} else {
				printf(" %02x", (*ucp++ = *((u_char *)addr)));
				count -= 3;
			}
			addr += size;
			offs += size;
		}

		while(count--)
			serial_putc(' ');

		cp = (u_char *)linebuf;
		for (i=0; i<linebytes; i++) {
			if ((*cp < 0x20) || (*cp > 0x7e))
				serial_putc('.');
			else
				printf("%c", *cp);
			cp++;
		}
		serial_putc('\n');
		nbytes -= linebytes;
	} while (nbytes > 0);

	return 0;
}

static void dump_dma_context(struct mxs_dma_desc *d)
{
	int j;

	pr_info("    --\n");
	pr_info("    Physical Address: %p\n" , &(d->cmd));
	pr_info("    Next            : 0x%08lx\n", d->cmd.next);
	pr_info("    Command         : 0x%08lx\n", d->cmd.cmd.data);
	pr_info("    Buffer          : 0x%08x\n" , d->cmd.address);
	for (j = 0; j < 3; j++) {
		pr_info("    PIO[%u]          : 0x%08lx\n",
			j, d->cmd.pio_words[j]);
	}
}
#else
#define memory_display(a, b, c, d)
#define dump_dma_context(x)
#endif

static void hw_i2c_dmachan_reset(struct mxs_i2c_dev *dev)
{
	mxs_dma_apbx_reset(dev->dma_chan);
	mxs_dma_apbx_ack_irq(dev->dma_chan);
}

static int hw_i2c_dma_init(struct mxs_i2c_dev *mxs_i2c)
{
	int i;

	for (i = 0; i < NR_DESC; i++) {
		desc[i] = mxs_dma_alloc_desc();
		if (desc[i] == NULL)
			goto err;
	}

	i2c_buf = calloc(PAGE_SIZE, 1);
	if (i2c_buf == NULL)
		goto err;

	hw_i2c_dmachan_reset(mxs_i2c);
	mxs_dma_apbx_ack_irq(mxs_i2c->dma_chan);
	mxs_dma_apbx_enable_irq(mxs_i2c->dma_chan, 1);

	return 0;

err:
	while (--i >= 0)
		mxs_dma_free_desc(desc[i]);

	return -ENOMEM;
}

static void __attribute__((unused)) hw_i2c_dma_uninit(struct mxs_i2c_dev *mxs_i2c)
{
	int i;

	mxs_dma_apbx_enable_irq(mxs_i2c->dma_chan, 0);
	mxs_dma_apbx_disable(mxs_i2c->dma_chan);

	for (i = 0; i < NR_DESC; i++)
		mxs_dma_free_desc(desc[i]);

	hw_i2c_dmachan_reset(mxs_i2c);

	free(i2c_buf);
}

static void hw_i2c_dma_setup_read(u8 addr, void *buff, int len, int flags)
{
	if (len > (PAGE_SIZE - 4))
		BUG();

	memset(&desc[0]->cmd, 0, sizeof(desc[0]->cmd));
	memset(&desc[1]->cmd, 0, sizeof(desc[1]->cmd));

	desc[0]->cmd.cmd.bits.bytes = 1;
	desc[0]->cmd.cmd.bits.pio_words = 1;
	desc[0]->cmd.cmd.bits.wait4end = 1;
	desc[0]->cmd.cmd.bits.dec_sem = 0;
	desc[0]->cmd.cmd.bits.irq = 1;
	desc[0]->cmd.cmd.bits.chain = 1;
	desc[0]->cmd.cmd.bits.command = DMA_READ;
	desc[0]->cmd.address = (u32) i2c_buf;
	desc[0]->cmd.pio_words[0] = CMD_I2C_SELECT;

	desc[0]->cmd.next = (u32)desc[1];

	i2c_buf[0] = addr | I2C_READ;

	desc[1]->cmd.cmd.bits.bytes = len;
	desc[1]->cmd.cmd.bits.pio_words = 1;
	desc[1]->cmd.cmd.bits.wait4end = 1;
	desc[1]->cmd.cmd.bits.dec_sem = 1;
	desc[1]->cmd.cmd.bits.irq = 1;
	desc[1]->cmd.cmd.bits.command = DMA_WRITE;
	desc[1]->cmd.address = (u32) i2c_buf + 1;
	desc[1]->cmd.pio_words[0] = CMD_I2C_READ;
	desc[1]->cmd.pio_words[0] |= BF_I2C_CTRL0_XFER_COUNT(len) | flags;

	desc[1]->cmd.next = 0;
}

static void hw_i2c_dma_setup_write(u8 addr, void *buff, int len, int flags)
{
	memset(&desc[2]->cmd, 0, sizeof(desc[2]->cmd));

	desc[2]->cmd.cmd.bits.bytes = len + 1;
	desc[2]->cmd.cmd.bits.pio_words = 1;
	desc[2]->cmd.cmd.bits.wait4end = 1;
	desc[2]->cmd.cmd.bits.dec_sem = 1;
	desc[2]->cmd.cmd.bits.irq = 1;
	desc[2]->cmd.cmd.bits.command = DMA_READ;
	desc[2]->cmd.address = (u32) i2c_buf;
	desc[2]->cmd.pio_words[0] = CMD_I2C_WRITE;
	desc[2]->cmd.pio_words[0] |= BM_I2C_CTRL0_POST_SEND_STOP;
	desc[2]->cmd.pio_words[0] |= BF_I2C_CTRL0_XFER_COUNT(len + 1) | flags;

	desc[2]->cmd.next = 0;

	i2c_buf[0] = addr | I2C_WRITE;
	memcpy(&i2c_buf[1], buff, len);
	dump_dma_context(desc[2]);
}

static void hw_i2c_dma_run(struct mxs_i2c_dev *dev, int dir)
{
	mxs_dma_apbx_ack_irq(dev->dma_chan);
	memory_display((void *)0x80024000, 0x80024000, 0x80, 4);
	if (dir == I2C_READ)
		mxs_dma_apbx_enable((uint32_t)desc[0], dev->dma_chan);
	else
		mxs_dma_apbx_enable((uint32_t)desc[2], dev->dma_chan);
}

static void hw_i2c_finish_read(struct mxs_i2c_dev *dev, void *buff, int len)
{
	memcpy(buff, &i2c_buf[1], len);
}

static int mxs_i2c_isr(void);

static int wait_for_completion(unsigned int chan)
{
	uint32_t timout = 0xffffff;
	int ret;

	do {
		if (mxs_dma_apbx_irq_is_pending(chan)) {
			mxs_dma_apbx_ack_irq(chan);
			break;
		}
	} while (--timout);

	if (!timout)
		ret = -ETIMEDOUT;

	do {
		ret = mxs_i2c_isr();
	} while (ret > 0);

	return ret;
}

/*
 * Low level master read/write transaction.
 */
static int mxs_i2c_xfer_msg(struct mxs_i2c_dev *dev, 
			    struct i2c_msg *msg, int stop)
{
	int err;
	int flags;

	dev->cmd_err = 0;

	pr_debug("addr: 0x%04x, len: %d, flags: 0x%x, stop: %d\n",
		 msg->addr, msg->len, msg->flags, stop);

	if ((msg->len == 0) || (msg->len > (PAGE_SIZE - 1)))
		return -EINVAL;

	flags = stop ? BM_I2C_CTRL0_POST_SEND_STOP : 0;

	if (msg->flags & I2C_M_RD) {
		hw_i2c_dma_setup_read(msg->addr, msg->buf, msg->len, flags);
		hw_i2c_dma_run(dev, I2C_READ);
	} else {
		hw_i2c_dma_setup_write(msg->addr, msg->buf, msg->len, flags);
		hw_i2c_dma_run(dev, I2C_WRITE);
	}

	err = wait_for_completion(dev->dma_chan);
	if (err < 0) {
		pr_err("controller is timed out\n");
		return -ETIMEDOUT;
	}

	if ((!dev->cmd_err) && (msg->flags & I2C_M_RD))
		hw_i2c_finish_read(dev, msg->buf, msg->len);

	pr_debug("Done with err=%d\n", dev->cmd_err);

	return dev->cmd_err;
}

static int
mxs_i2c_xfer(struct mxs_i2c_dev *dev, struct i2c_msg msgs[], int num)
{
	int i;
	int err;

	if (!msgs->len)
		return -EINVAL;

	for (i = 0; i < num; i++) {
		err = mxs_i2c_xfer_msg(dev, &msgs[i], (i == (num - 1)));
		if (err)
			break;
	}

	if (err == 0)
		err = num;

	return err;
}

#define I2C_IRQ_MASK 0x000000FF
static int mxs_i2c_isr(void)
{
	struct mxs_i2c_dev *mxs_i2c = &mxs_i2c_dev;
	u32 stat;
	u32 done_mask = BM_I2C_CTRL1_DATA_ENGINE_CMPLT_IRQ | BM_I2C_CTRL1_BUS_FREE_IRQ;
	int ret = 1;

	stat = __raw_readl(HW_I2C_CTRL1_ADDR) & I2C_IRQ_MASK;
	if (!stat)
		return 1;

	if (stat & BM_I2C_CTRL1_NO_SLAVE_ACK_IRQ) {
		ret = mxs_i2c->cmd_err = -EREMOTEIO;

		/*
		 * Stop DMA
		 * Clear NAK
		 */
		__raw_writel(BM_I2C_CTRL1_CLR_GOT_A_NAK, HW_I2C_CTRL1_SET_ADDR);
		hw_i2c_dmachan_reset(mxs_i2c);
		mxs_reset_block((void *)HW_I2C_CTRL0_ADDR, 1);
		/* Will catch all error (IRQ mask) */
		__raw_writel(0x0000FF00, HW_I2C_CTRL1_SET_ADDR);
		goto done;
	}

	/* Don't care about BM_I2C_CTRL1_OVERSIZE_XFER_TERM_IRQ */
	if (stat & (BM_I2C_CTRL1_EARLY_TERM_IRQ |
		    BM_I2C_CTRL1_MASTER_LOSS_IRQ |
		    BM_I2C_CTRL1_SLAVE_STOP_IRQ | BM_I2C_CTRL1_SLAVE_IRQ)) {
		ret = mxs_i2c->cmd_err = -EIO;
		goto done;
	}

	if ((stat & done_mask) == done_mask) {
		ret = 0;
		goto done;
	}

	return 1;

done:
	__raw_writel(stat, HW_I2C_CTRL1_CLR_ADDR);
	return ret;
}

int mxs_i2c_init(int speed, int slaveaddr)
{
	struct mxs_i2c_dev *mxs_i2c = &mxs_i2c_dev;
	int err;

	mxs_i2c_hw_init_i2c();

	mxs_i2c->regbase = (unsigned long)REGS_I2C_BASE;
	mxs_i2c->dma_chan = (int)HW_APBX_DMA_I2C_CHANNEL;
	mxs_i2c->irq_err = HW_IRQ_I2C_ERROR;
	mxs_i2c->irq_dma = HW_IRQ_I2C_DMA;

	/* reset I2C module */
	mxs_reset_block((void *)HW_I2C_CTRL0_ADDR, 1);
	mxs_reset_block((void *)HW_APBX_CTRL0_ADDR, 1);

	mxs_i2c->buf = calloc(PAGE_SIZE, 1);
	if (mxs_i2c->buf == NULL) {
		pr_err("HW Init failed\n");
		err = -ENOMEM;
		goto init_failed;
	} else {
		err = hw_i2c_dma_init(mxs_i2c);
		if (err) {
			pr_err("HW Init failed\n");
			goto init_failed;
		}
	}

	/* Will catch all error (IRQ mask) */
	__raw_writel(0x0000FF00, HW_I2C_CTRL1_SET_ADDR);

	return 0;

init_failed:
	return err;
}

void i2c_init(int speed, int slaveaddr)
{
	mxs_i2c_init(speed, slaveaddr);
}

/*
 * Read/Write interface:
 *   chip:    I2C chip address, range 0..127
 *   addr:    Memory (register) address within the chip
 *   alen:    Number of bytes to use for addr (typically 1, 2 for larger
 *              memories, 0 for register type devices with only one
 *              register)
 *   buffer:  Where to read/write the data
 *   len:     How many bytes to read/write
 *
 *   Returns: 0 on success, not 0 on failure
 */
int i2c_read(uchar chip, uint addr, int alen, uchar *buf, int len)
{
	struct mxs_i2c_dev *mxs_i2c = &mxs_i2c_dev;
	struct i2c_msg msg;
	int ret;

	memset(&msg, 0, sizeof msg);

	msg.addr = addr;
	msg.flags |= I2C_M_RD;
	msg.len = len;
	msg.buf = buf;

	ret = mxs_i2c_xfer(mxs_i2c, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? len: ret;
}

int i2c_write(uchar chip, uint addr, int alen, uchar *buf, int len)
{
	struct mxs_i2c_dev *mxs_i2c = &mxs_i2c_dev;
	struct i2c_msg msg;
	int ret;

	memset(&msg, 0, sizeof msg);

	msg.addr = addr;
	//msg.flags |= I2C_M_RD;
	msg.len = len;
	msg.buf = buf;

	ret = mxs_i2c_xfer(mxs_i2c, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? len : ret;
}
