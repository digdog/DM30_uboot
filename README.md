## Pomera DM30 U-Boot  

Original [Pomera DM30][1] U-Boot source from King Jim CO.,LTD. 

### LICENSE

Distributed under terms of version 2 of the **GNU General Public License** as published by the Free Software Foundation.  

### Related Files

DM30 is basically DM100, additional comments (in Simp. Chinese) on how to patch the firmware can be found in this file:

* `common/dm100.c`

Keyboard driver. Use i2c to setup keyboard mapping.

* `drivers/input/dm100_keyb.c`

LCD driver

* `drivers/video/mxs/lcdif.h` 
* `drivers/video/mxs/lcdif.c` 
* `drivers/video/mxs/ortus.c`
* `drivers/video/mxs/mxsfb.c`

LCD clock

* `cpu/arm926ejs/mx23/mx23_lcd_clk.c`

SSP driver with DMA

* `drivers/mmc/mx23_mmc_dma.c`
* `drivers/mmc/mxs-mmc.c`

GPMI driver

* `drivers/mtd/nand/mx23_nand_io.c`

I2C driver

* `drivers/i2c/mxs_i2c.c`

DMA driver

* `cpu/arm926ejs/mx23/mxs-dma.h`
* `cpu/arm926ejs/mx23/mxs-dma.c`

mxs_reset_block()

* `include/asm-arm/arch-mx23/mxs-block.h`

[1]: https://www.kingjim.co.jp/pomera/dm30/
