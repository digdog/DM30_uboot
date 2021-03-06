/*
 * (C) Copyright 2004, Li-Pro.Net <www.li-pro.net>
 * Stephan Linz <linz@li-pro.net>
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
#include <nios-io.h>
#include <spi.h>

#if	defined(CONFIG_HW_WATCHDOG)
extern void ssv_wd_pio_init(void);	/* comes from ../common/wd_pio.c
					   included by ./misc.c */
#endif

void _default_hdlr (void)
{
	printf ("default_hdlr\n");
}

int board_early_init_f (void)
{
#if	defined(CONFIG_HW_WATCHDOG)
	ssv_wd_pio_init();
#endif
	return 0;
}

int checkboard (void)
{
	puts (	"Board: SSV DilNetPC ADNP/ESC1"
#if	defined(CONFIG_DNPEVA2)
		" on DNP/EVA2"
#endif
		"\n");
#if     defined(CONFIG_NIOS_BASE_32)
	puts ("Conf.: SSV Base 32 (nios_32)\n");
#endif

	return 0;
}

phys_size_t initdram (int board_type)
{
	return (0);
}

/*
 * The following are used to control the SPI chip selects for the SPI command.
 */
#if defined(CONFIG_CMD_SPI) && CONFIG_NIOS_SPI

#define	SPI_RTC_CS_MASK	0x00000001

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	return bus == 0 && cs == 0;
}

void spi_cs_activate(struct spi_slave *slave)
{
	nios_spi_t *spi = (nios_spi_t *)CONFIG_SYS_NIOS_SPIBASE;

	spi->slaveselect = SPI_RTC_CS_MASK;	/* activate (1) */
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	nios_spi_t *spi = (nios_spi_t *)CONFIG_SYS_NIOS_SPIBASE;

	spi->slaveselect = 0;			/* deactivate (0) */
}

#endif

#if	defined(CONFIG_POST)
/*
 * Returns 1 if keys pressed to start the power-on long-running tests
 * Called from board_init_f().
 */
int post_hotkeys_pressed(void)
{
	return 0;       /* No hotkeys supported */
}
#endif /* CONFIG_POST */
