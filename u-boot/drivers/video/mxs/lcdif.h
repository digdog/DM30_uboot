/*
 * --------------------------------------------------------------------------
 *
 *       Filename:  lcdif.h
 *
 *    Description:  
 *
 *        Version:  0.01
 *        Created:  2011年05月18日 12时21分42秒
 *
 *         Author:  smmei (), 
 *        Company:  
 * --------------------------------------------------------------------------
 */

#ifndef _ARCH_ARM_LCDIF_H
#define _ARCH_ARM_LCDIF_H

#include <linux/types.h>
#include <asm/arch/registers/regslcdif.h>

enum {
	MXS_LCD_PANEL_SYSTEM = 0,
	MXS_LCD_PANEL_VSYNC,
	MXS_LCD_PANEL_DOTCLK,
	MXS_LCD_PANEL_DVI,
};

struct mxs_platform_fb_entry {
	char name[16];
	u16 x_res;
	u16 y_res;
	u16 bpp;
	u32 cycle_time_ns;
	int lcd_type;
	int (*init_panel) (dma_addr_t, int, struct mxs_platform_fb_entry *);
	void (*release_panel) (struct mxs_platform_fb_entry *);
	void (*run_panel) (void);
	void (*stop_panel) (void);
	int (*pan_display) (dma_addr_t);
};

#define MXS_LCDIF_PANEL_INIT	1
#define MXS_LCDIF_PANEL_RELEASE	2

extern void 	mxs_init_lcdif(void);
extern int 	mxs_lcdif_dma_init(dma_addr_t phys, int memsize);
extern void 	mxs_lcdif_dma_release(void);
extern void 	mxs_lcdif_run(void);
extern void 	mxs_lcdif_stop(void);
extern int 	mxs_lcdif_pan_display(dma_addr_t addr);

#endif /* _ARCH_ARM_LCDIF_H */
