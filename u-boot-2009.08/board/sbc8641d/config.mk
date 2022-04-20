# Copyright 2004 Freescale Semiconductor.
# Modified by Jeff Brown
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

#
# sbc8641 board
# default CCSRBAR is at 0xff700000
#
TEXT_BASE = 0xfff00000

PLATFORM_CPPFLAGS += -DCONFIG_MPC86xx=1
PLATFORM_CPPFLAGS += -DCONFIG_MPC8641=1 -maltivec -mabi=altivec -msoft-float
