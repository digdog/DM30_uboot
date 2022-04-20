/*
 * ==========================================================================
 *
 *       Filename:  dm100_keyb.c
 *
 *    Description:  tx35894xbd
 *
 *        Version:  0.01
 *        Created:  2011年08月27日 16时23分49秒
 *
 *         Author:  smmei (), 
 *        Company:  
 *
 * ==========================================================================
 */

#include <common.h>
#include <asm/arch/regs-pinctrl.h>
#include <asm/arch/pinctrl.h>
#include <keyboard.h>
#include <i2c.h>

#define mdelay(t)	udelay(t * 1000)

enum {
	K_1 = 0x02, K_2, K_3, K_4, K_5, K_6, K_7, K_8, K_9, K_0,   
	K_Q = 0x10, K_W, K_E, K_R, K_T, K_Y, K_U, K_I, K_O, K_P,
	K_A = 0x1e, K_S, K_D, K_F, K_G, K_H, K_J, K_K, K_L,
	K_Z = 0x2c, K_X, K_C, K_V, K_B, K_N, K_M, 
};

#define K_ENTER 	0x1c
#define K_SPACE 	0x39
#define K_BACKSPACE     0x0e
#define K_CAPS          0x00
#define K_TAB           0x0f
#define K_ESC           0x00
#define K_F1            0x00
#define K_F2            0x00
#define K_F3            0x00
#define K_F4            0x00
#define K_F5            0x00
#define K_F6            0x00
#define K_F7            0x00
#define K_F8            0x00
#define K_F9            0x00
#define K_F10           0x00
#define K_F11           0x00
#define K_F12           0x00
#define K_RIGHT         0x00
#define K_PGDN          0x00
#define K_HOME          0x00
#define K_PGUP          0x00
#define K_BACKSLA       0x00
#define K_BRACK2        0x00
#define K_COLON         0x00
#define K_MENU          0x00
#define K_BRACK1        0x00
#define K_SEMICOL       0x00
#define K_SLASH         0x00
#define K_DOT           0x00
#define K_FUN2          0x00
#define K_FUN3          0x00
#define K_EQUAL         0x00
#define K_DEL           0x00
#define K_INSERT        0x00
#define K_COMMA         0x00
#define K_FUN4          0x00
#define K_AT            0x00
#define K_NONE          0x00

static const uint8_t kbd_translate_table[] = {
	K_CAPS,  K_TAB,  K_2, 	      K_1,      K_F2,	K_F1,	 K_NONE,   K_ESC,       /* 0x00 ~ 0x07 */
	K_NONE,  K_X, 	 K_Z, 	      K_S,      K_A,	K_W,	 K_Q,	   K_3, 	/* 0x80 ~ 0x0f */
	K_V, 	 K_C, 	 K_F, 	      K_D,      K_R, 	K_E, 	 K_4, 	   K_F3, 	/* 0x10 ~ 0x17 */
	K_RIGHT, K_PGDN, K_HOME,      K_PGUP,   K_BACKSLA, K_ENTER, K_BRACK2, K_COLON, 	/* 0x10 ~ 0x17 */
	K_MENU,  K_NONE, K_BACKSPACE, K_BRACK1, K_SEMICOL, K_SLASH, K_DOT,    K_NONE,  	/* 0x20 ~ 0x27 */
	K_NONE,  K_FUN2, K_FUN3,      K_EQUAL,  K_0, 	K_AT, 	 K_P, 	   K_L,  	/* 0x28 ~ 0x2f */
	K_SPACE, K_B, 	 K_G, 	      K_Y,      K_T, 	K_6, 	 K_5, 	   K_F4,  	/* 0x30 ~ 0x37 */
	K_NONE,  K_DEL,  K_INSERT,    K_F10,    K_9, 	K_O, 	 K_K, 	   K_COMMA,  	/* 0x38 ~ 0x3f */
	K_F9, 	 K_F8, 	 K_F7, 	      K_8,      K_I, 	K_J, 	 K_M, 	   K_FUN4,  	/* 0x40 ~ 0x47 */
	K_NONE,  K_NONE, K_N, 	      K_H,      K_U, 	K_7, 	 K_F6, 	   K_F5,  	/* 0x48 ~ 0x4f */
};

#define make_scancode(code) (((code & 0x70) >> 4) | ((code & 0x0f) << 3))

struct tx_command {
	int len;
	uint8_t cmds[10];
};

static const struct tx_command tx_cmds[] = {
	{ .len = 2, .cmds = { 0x84, 0x01 } }, 
	{ .len = 2, .cmds = { 0x89, 0x44 } }, 
	{ .len = 2, .cmds = { 0x82, 0x3f } },
	{ .len = 2, .cmds = { 0x82, 0x00 } }, 
	{ .len = 2, .cmds = { 0x8a, 0x01 } }, 
	{ .len = 2, .cmds = { 0x03, 0x8c } }, 
	{ .len = 2, .cmds = { 0xa7, 0xf8 } }, 
	{ .len = 3, .cmds = { 0xaa, 0xaa, 0xaa } }, 
	{ .len = 3, .cmds = { 0xac, 0x00, 0x00 } }, 
	{ .len = 3, .cmds = { 0xae, 0x00, 0x00 } }, 
	{ .len = 2, .cmds = { 0xec, 0x01 } }, 
	{ .len = 2, .cmds = { 0xed, 0x01 } }, 
	{ .len = 2, .cmds = { 0xee, 0x01 } }, 
	{ .len = 2, .cmds = { 0xef, 0x01 } }, 
	{ .len = 2, .cmds = { 0x01, 0xa3 } }, 
	{ .len = 2, .cmds = { 0x02, 0x40 } }, 
	{ .len = 2, .cmds = { 0xd2, 0x00 } }, 
	{ .len = 2, .cmds = { 0xd3, 0x00 } }, 
	{ .len = 2, .cmds = { 0xd4, 0x00 } }, 
	{ .len = 2, .cmds = { 0xf3, 0x03 } }, 
	{ .len = 2, .cmds = { 0xf2, 0x01 } }, 
	{ .len = 2, .cmds = { 0x09, 0x03 } }, 
	{ .len = 2, .cmds = { 0x08, 0x03 } }, 
	{ .len = 2, .cmds = { 0x88, 0x01 } }, 
	{ .len = 2, .cmds = { 0xe9, 0xff } }, 
	{ .len = 2, .cmds = { 0x8b, 0x01 } }, 
	{ .len = 3, .cmds = { 0x8c, 0x01, 0x00 } }, 
};

int tx_write(uint8_t *byte, int len)
{
	i2c_write(0, CONFIG_SYS_I2C_SLAVE, 0, byte, len);
	return 0;
}

int tx_read(uint8_t addr, uint8_t *byte, int len)
{
	i2c_write(0, CONFIG_SYS_I2C_SLAVE, 0, &addr, 1);
	i2c_read(0,  CONFIG_SYS_I2C_SLAVE, 0, byte, len);
	return 0;
}

int kbd_init_hw(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tx_cmds); i++) {
		tx_write((uint8_t *)tx_cmds[i].cmds, tx_cmds[i].len);
	}

	return 0;
}

int kbd_read_input(void)
{
	return 0;
}

void dm100_kbd_check(void)
{
	uint8_t scancode = 0;

	while (pin_gpio_get(PINID_GPMI_RDY1));

	tx_read(0x10, &scancode, sizeof(scancode));
	if (!(scancode & 0x80) && (scancode & 0x7f) != 0x7f) {
		uint8_t keycode;
		keycode = make_scancode(scancode);
		if (keycode < ARRAY_SIZE(kbd_translate_table))
			handle_scancode(kbd_translate_table[keycode]);
	}
}

int dm100_patching_key(void)
{
	uint8_t key1, key2, key3;

#define KEY_C	0x11
#define KEY_8	0x43 

	tx_read(0x0b, &key1, sizeof(key1));
	if (key1 == 0x7f)
		return 0;

	tx_read(0x0c, &key2, sizeof(key2));
	if (key2 == 0x7f)
		return 0;

	/* 若除C+8之外还有其他键按下，则表示误按 */
	tx_read(0x0d, &key3, sizeof(key3));
	if (key3 != 0x7f)
		return 0;

	key1 = make_scancode(key1);
	key2 = make_scancode(key2);

	return ((key1 == KEY_C && key2 == KEY_8) ||
		(key1 == KEY_8 && key2 == KEY_C));
}

void drv_keyboard_init(void)
{
	kbd_init();
}

void pckbd_leds(unsigned char leds)
{
}
