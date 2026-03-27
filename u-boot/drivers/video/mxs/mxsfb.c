/*
 * ==========================================================================
 *
 *       Filename:  mxsfb.c
 *
 *    Description:  lcd driver for mx23
 *
 *        Version:  0.01
 *        Created:  2011年05月18日 11时50分51秒
 *
 *         Author:  smmei (), 
 *        Company:  
 *
 * ==========================================================================
 */

#include <common.h>
#include <malloc.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/arch/registers/regslcdif.h>
#include <asm/arch/mx23_pins.h>
#include <asm/arch/pinctrl.h>
#include <lcd.h>
#include "lcdif.h"

DECLARE_GLOBAL_DATA_PTR;

#ifdef DEBUG_TIME
/* ******* time test******** */
#include <asm/arch/timrot.h>

#define TIMCTRL		TIMCTRL0
#define TIMCOUNT	TIMCOUNT0

#define READ_TIMER ((REG_RD(TIMROT_BASE + TIMCOUNT) & 0xffff0000) >> 16)

#define time_test_start() 				\
	uint32_t __t1, __t2, __t;			\
	__t1 = READ_TIMER;

#define time_test_end()					\
	do {						\
		__t2 = READ_TIMER;			\
	       if (__t1 > __t2)				\
		       __t = __t1 - __t2;		\
	       else					\
		       __t = __t1 + 0xffff - __t2;	\
	       pr_info("%u ms\n", __t);			\
	} while (0)
/* ******************* */
#else
#define time_test_start()
#define time_test_end()
#endif

#ifndef mdelay
#define mdelay(t)	udelay((t) * 1000)
#endif

enum {
	F_DISABLE = 0,
	F_ENABLE,
	F_REENABLE,
};

struct mxs_fb_data {
	struct mxs_platform_fb_entry *pentry;
	u32 state;
	u32 task_state;

	dma_addr_t screen_base;
	ssize_t screen_size;

	int irq;
	void *par;
};

extern struct mxs_platform_fb_entry fb_entry;

static struct mxs_fb_data fb_data = {
	.pentry = &fb_entry, 
};

/* forward declaration */
static void init_timings(struct mxs_fb_data *data);

static void mxsfb_enable_controller(struct mxs_fb_data *data)
{
	struct mxs_platform_fb_entry *pentry = data->pentry;

	if (!data || !data->pentry)
		return;

	mxs_init_lcdif();
	init_timings(data);
	pentry->init_panel(data->screen_base, data->screen_size, pentry);
	pentry->run_panel();
}

static void mxsfb_disable_controller(struct mxs_fb_data *data)
{
	struct mxs_platform_fb_entry *pentry = data->pentry;

	if (!data || !data->pentry)
		return;

	if (pentry->stop_panel)
		pentry->stop_panel();
	pentry->release_panel(pentry);
}

static void set_controller_state(struct mxs_fb_data *data, u32 state)
{
	//struct mxs_platform_fb_entry *pentry = data->pentry;
	u32 old_state;

	old_state = data->state;
	debug("%s, old_state %d, state %d\n", __func__, old_state, state);

	switch (state) {
	case F_DISABLE:
		/*
		 * Disable controller
		 */
		if (old_state != F_DISABLE) {
			data->state = F_DISABLE;
			mxsfb_disable_controller(data);
		}
		break;

	case F_REENABLE:
		/*
		 * Re-enable the controller when panel changed.
		 */
		if (old_state == F_ENABLE) {
			mxsfb_disable_controller(data);
			memset((void *)data->screen_base, 0, data->screen_size);
			mxsfb_enable_controller(data);

			data->state = F_ENABLE;
		} else if (old_state == F_DISABLE) {
			memset((void *)data->screen_base, 0, data->screen_size);
			data->state = F_DISABLE;
		}
		break;

	case F_ENABLE:
		if (old_state != F_ENABLE) {
			data->state = F_ENABLE;
			mxsfb_enable_controller(data);
		}
		break;
	}
}

static inline u_long get_line_length(int xres_virtual, int bpp)
{
	u_long length;

	length = xres_virtual * bpp;
	length = (length + 31) & ~31;
	length >>= 3;
	return length;
}

static int __attribute__((unused)) mxsfb_pan_display(struct mxs_fb_data *data)
{
	if (!data->pentry->pan_display)
		return -EINVAL;

	data->pentry->pan_display(data->screen_base);

	return 0;
}

static void init_timings(struct mxs_fb_data *data)
{
	unsigned phase_time;
	unsigned timings;

	/* Just use a phase_time of 1. As optimal as it gets, now. */
	phase_time = 1;

	/* Program all 4 timings the same */
	timings = phase_time;
	timings |= timings << 8;
	timings |= timings << 16;
	__raw_writel(timings, HW_LCDIF_TIMING_ADDR);
}

static int __attribute__((unused)) mxsfb_suspend(struct mxs_fb_data *data)
{
	set_controller_state(data, F_DISABLE);

	return 0;
}

static int __attribute__((unused)) mxsfb_resume(struct mxs_fb_data *data)
{
	set_controller_state(data, F_ENABLE);

	return 0;
}

static int __attribute__((unused)) mxsfb_remove(struct mxs_fb_data *data)
{
	set_controller_state(data, F_DISABLE);

	if (data->screen_base) {
		free((void *)data->screen_base);
		data->screen_base = 0;
	}

	return 0;
}

static int mxsfb_probe(void)
{
	int ret = 0;
	struct mxs_fb_data *data = &fb_data;
	struct mxs_platform_fb_entry *pentry = data->pentry;

#if 0
	data->screen_size = pentry->x_res * pentry->y_res * pentry->bpp / 8;
	data->screen_base = (dma_addr_t)realloc((void *)data->screen_base, data->screen_size);

	memset(data->screen_base, 0, data->screen_size);
#else
	data->screen_size = pentry->x_res * pentry->y_res * pentry->bpp / 8;
	data->screen_base = gd->fb_base;
#endif

	mxs_init_lcdif();

	ret = pentry->init_panel(data->screen_base, data->screen_size, pentry);
	if (ret) {
		pr_err("cannot initialize LCD panel\n");
		goto out;
	}
	debug("LCD panel initialized\n");

out:
	return ret;
}

void *lcd_base;			/* Start of framebuffer memory	*/
void *lcd_console_address;	/* Start of console buffer	*/

int lcd_line_length;
int lcd_color_fg;
int lcd_color_bg;

short console_col;
short console_row;

static ushort colormap[256];

vidinfo_t panel_info = {
	.vl_col		= 801,
	.vl_row		= 600,
	.vl_bpix	= 3,
	.cmap		= colormap,
};

void lcd_initcolregs(void)
{
}

void lcd_setcolreg(ushort regno, ushort red, ushort green, ushort blue)
{
}

void lcd_disable(void)
{
}

void lcd_ctrl_init(void *lcdbase)
{
	mxsfb_probe();
	//lcd_base = (void *)data->screen_base;
	lcd_line_length = 801;
}

void lcd_enable(void)
{
	struct mxs_fb_data *data = &fb_data;
	struct mxs_platform_fb_entry *pentry = data->pentry;
	pentry->run_panel();
}

ulong calc_fbsize (void)
{
	return 800 * 600;
}

static void slcd_line_movement(int factor)
{
	struct mxs_fb_data *data = &fb_data;
	struct mxs_platform_fb_entry *pentry = data->pentry;
	__u8 *frame = (__u8 *)fb_data.screen_base;
	__u16 color;
	ulong row = 0, column = 0;

	for (row = 0; row < 600; ++row) {
		color = factor ? 0xffff: ~0xffff;
		__u8 *buffer = frame;
		for (column = 0; column < 800; ++column) {
			*buffer = color;
			if (column && column % 100 == 0)
				color = ~color;
			buffer++;
		}
		frame += CONFIG_LCD_WIDTH;
		if (row != 0 && row % 100 == 0)
			factor = !factor;
	}
	pentry->stop_panel();
}

static int do_lcd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	struct mxs_fb_data *data = &fb_data;
	struct mxs_platform_fb_entry *pentry = data->pentry;

	if (argc < 2)
		return 1;

	if (!strcmp(argv[1], "init"))
		mxsfb_probe();
	else if (!strcmp(argv[1], "run"))
		pentry->run_panel();
	else if (!strcmp(argv[1], "off"))
		pentry->stop_panel();
	else if (!strcmp(argv[1], "demo")) {
		disable_ctrlc(0);
		for (;;) {
			if (ctrlc())
				break;
			slcd_line_movement(0);
			mdelay(500);
			slcd_line_movement(1);
			mdelay(500);
		}
	} else if (!strcmp(argv[1], "timetest")) {
		int i = 0;
		time_test_start();
		for (i = 0; i < 10; i++) {
			slcd_line_movement(i & 0x1);
		}
		time_test_end();
	}

	return 0;
}

U_BOOT_CMD (lcd, 5, 1, do_lcd,
	"lcd commands",
	"");
