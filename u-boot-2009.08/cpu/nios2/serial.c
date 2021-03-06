/*
 * (C) Copyright 2004, Psyent Corporation <www.psyent.com>
 * Scott McNutt <smcnutt@psyent.com>
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
#include <watchdog.h>
#include <asm/io.h>
#include <nios2-io.h>

DECLARE_GLOBAL_DATA_PTR;

/*------------------------------------------------------------------
 * JTAG acts as the serial port
 *-----------------------------------------------------------------*/
#if defined(CONFIG_CONSOLE_JTAG)

static nios_jtag_t *jtag = (nios_jtag_t *)CONFIG_SYS_NIOS_CONSOLE;

void serial_setbrg( void ){ return; }
int serial_init( void ) { return(0);}

void serial_putc (char c)
{
	unsigned val;

	while (NIOS_JTAG_WSPACE ( readl (&jtag->control)) == 0)
		WATCHDOG_RESET ();
	writel (&jtag->data, (unsigned char)c);
}

void serial_puts (const char *s)
{
	while (*s != 0)
		serial_putc (*s++);
}

int serial_tstc (void)
{
	return ( readl (&jtag->control) & NIOS_JTAG_RRDY);
}

int serial_getc (void)
{
	int c;
	unsigned val;

	while (1) {
		WATCHDOG_RESET ();
		val = readl (&jtag->data);
		if (val & NIOS_JTAG_RVALID)
			break;
	}
	c = val & 0x0ff;
	return (c);
}

/*------------------------------------------------------------------
 * UART the serial port
 *-----------------------------------------------------------------*/
#else

static nios_uart_t *uart = (nios_uart_t *) CONFIG_SYS_NIOS_CONSOLE;

#if defined(CONFIG_SYS_NIOS_FIXEDBAUD)

/* Everything's already setup for fixed-baud PTF
 * assignment
 */
void serial_setbrg (void){ return; }
int serial_init (void) { return (0);}

#else

void serial_setbrg (void)
{
	unsigned div;

	div = (CONFIG_SYS_CLK_FREQ/gd->baudrate)-1;
	writel (&uart->divisor,div);
	return;
}

int serial_init (void)
{
	serial_setbrg ();
	return (0);
}

#endif /* CONFIG_SYS_NIOS_FIXEDBAUD */


/*-----------------------------------------------------------------------
 * UART CONSOLE
 *---------------------------------------------------------------------*/
void serial_putc (char c)
{
	if (c == '\n')
		serial_putc ('\r');
	while ((readl (&uart->status) & NIOS_UART_TRDY) == 0)
		WATCHDOG_RESET ();
	writel (&uart->txdata,(unsigned char)c);
}

void serial_puts (const char *s)
{
	while (*s != 0) {
		serial_putc (*s++);
	}
}

int serial_tstc (void)
{
	return (readl (&uart->status) & NIOS_UART_RRDY);
}

int serial_getc (void)
{
	while (serial_tstc () == 0)
		WATCHDOG_RESET ();
	return (readl (&uart->rxdata) & 0x00ff );
}

#endif /* CONFIG_JTAG_CONSOLE */
