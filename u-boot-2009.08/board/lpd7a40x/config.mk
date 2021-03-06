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

# Logic ZOOM LH7A400 SDK board w/Logic LH7A400-10 card engine
# w/Sharp LH7A400 SoC (ARM920T) cpu
#

#
# 32 or 64 MB SDRAM on SDCSC0 @ 0xc0000000
#
# Linux-Kernel is @ 0xC0008000, entry 0xc0008000
# params @ 0xc0000100
# optionally with a ramdisk at 0xc0300000
#
# we load ourself to 0xc1fc0000 (32M - 256K)
#
# download area is 0xc0f00000
#

TEXT_BASE = 0xc1fc0000
#TEXT_BASE = 0x00000000
