/*
 *
 * (C) Copyright 2009-2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __PINCTRL_H
#define __PINCTRL_H

/*
 * Each pin may be routed up to four different HW interfaces
 * including GPIO
 */
enum pin_fun {
	PIN_FUN1 = 0,
	PIN_FUN2,
	PIN_FUN3,
	PIN_GPIO
};

/*
 * Each pin may have different output drive strength in range from
 * 4mA to 20mA. The most common case is 4, 8 and 12 mA strengths.
 */
enum pad_strength {
	PAD_4MA = 0,
	PAD_8MA,
	PAD_12MA,
	PAD_RESV
};

/*
 * Each pin can be programmed for 1.8V or 3.3V
 */
enum pad_voltage {
	PAD_1V8 = 0,
	PAD_3V3
};

/*
 * Structure to define a group of pins and their parameters
 */
struct pin_desc {
	u32 id;
	enum pin_fun fun;
	enum pad_strength strength;
	enum pad_voltage voltage;
	u32 pullup:1;
};

struct pin_group {
	struct pin_desc *pins;
	int nr_pins;
};

extern void pin_gpio_direction(u32 id, u32 output);
extern u32 pin_gpio_get(u32 id);
extern void pin_gpio_set(u32 id, u32 val);
extern void pin_set_type(u32 id, enum pin_fun cfg);
extern void pin_set_strength(u32 id, enum pad_strength strength);
extern void pin_set_voltage(u32 id, enum pad_voltage volt);
extern void pin_set_pullup(u32 id, u32 pullup);
extern void pin_set_group(struct pin_group *pin_group);

#if 0
#define PIN_BITS		(5)
#define PINS_PER_BANK		(1 << PIN_BITS)
#define PINID_2_BANK(id)	((id) >> PIN_BITS)
#define PINID_2_PIN(id)		((id) & (PINS_PER_BANK - 1))
#define PINID_ENCODE(bank, pin)	(((bank) << PIN_BITS) + (pin))
#endif

/*
 * [23:0] -  PIN_PINID
 * [31:24] - PIN_BANK
 */
#define MXS_PIN_BANK_BIT	24
#define MXS_PIN_BANK_MAX	(0x7FFFFFFF >> (MXS_PIN_BANK_BIT - 1))
#define MXS_PIN_PINID_MAX	((1 << MXS_PIN_BANK_BIT) - 1)

#define MXS_PIN_ENCODE(b, p)	\
		((((b) & MXS_PIN_BANK_MAX) << MXS_PIN_BANK_BIT) |\
		 ((p) & MXS_PIN_PINID_MAX))

#define PINID_2_BANK(id) (((id) >> MXS_PIN_BANK_BIT) & MXS_PIN_BANK_MAX)
#define PINID_2_PIN(id) ((id) & MXS_PIN_PINID_MAX)

#include "mx23_pins.h"

#endif
