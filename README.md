## Pomera DM30 U-Boot

[Pomera DM30][1] U-Boot source version 0.30 and [i.MX Bootlets][3] from [King Jim CO., LTD.][4]

The DM30 is a compact digital memo pad (digital typewriter) built around the Freescale (NXP) i.MX233 SoC with an E-Paper display.

### Hardware Overview

| Component | Detail |
|-----------|--------|
| SoC | Freescale i.MX233 (MX23) |
| CPU | ARM926EJS @ 120 MHz |
| RAM | 128 MB mDDR (2 x 64 MB banks, `0x40000000`-`0x47FFFFFF`) |
| Display | 800 x 600 E-Paper via IT8951 controller (I80 16-bit parallel) |
| Main Storage | eMMC (8-bit, mmc dev 1) |
| External Storage | SD card via SSP1 (mmc dev 0) |
| Keyboard | TX35894XBD matrix scanner via I2C |
| Serial | STMP3XXX Debug UART @ 115200 baud |
| Battery | Dry cell with LRADC voltage detection |

### Repository Structure

```
DM30_uboot/
├── u-boot/              # U-Boot source tree (version 0.30)
├── imx-bootlets/        # i.MX Bootlets for boot image packaging
├── build_dm30_uboot.sh  # Build script (macOS + arm-none-eabi-gcc 15)
├── README.md
└── LICENSE              # GNU GPL v2
```

### Build

Prerequisites:

```bash
brew install arm-none-eabi-gcc arm-none-eabi-binutils
```

[elftosb](https://github.com/digdog/elftosb) is required for the final `.sb` boot stream packaging:

```bash
git clone https://github.com/digdog/elftosb.git ../elftosb
cd ../elftosb && mkdir -p bld/macos
make -C bld/macos -f $(pwd)/makefile.rules SRC_DIR=$(pwd) UNAMES=Linux elftosb
```

Build U-Boot + Bootlets + `.sb` boot stream:

```bash
./build_dm30_uboot.sh
```

The script compiles U-Boot, bootlets (`power_prep` + `boot_prep`), and packages
them into `dm30-uboot.raw` via `elftosb`.

### Boot Flow

1. i.MX233 ROM loads `power_prep` and `boot_prep` from the `.sb` boot stream
2. `boot_prep` initializes DDR, clocks, and loads U-Boot
3. U-Boot initializes hardware (GPIO keys, eMMC, SD, LCD power, IT8951 E-Paper)
4. Checks for firmware update key combo (Right Ctrl + Right Shift); if pressed, enters update mode
5. Detects battery voltage via LRADC; powers off if too low (threshold ~2.4V / AD 15250)
6. Loads user program from eMMC User Program partition to DDR at `0x40000000`
7. Jumps to user program with version info (`0xdead << 16 | version << 8 | patchlevel`)

### Firmware Update

Triggered by holding **Right Ctrl + Right Shift** (GPIO) or **C + 8** (I2C keyboard) during power-on. Requires USB 5V power.

1. Reads `dm30-uboot.raw` from SD card and writes to eMMC Bootloader partition
2. Reads `Pomera_Update_Dm30_V*.bin` from SD card
3. Validates series identifier ("DM30"), version, and checksum
4. Writes User Program and Database to eMMC with read-back verification

### eMMC Storage Layout

Offsets are relative to the start of the first MBR partition.

| Partition | Size |
|-----------|------|
| Bootloader | ~8 MB |
| User Program | 16 MB |
| Database | 177 MB |
| Reserve | 55 MB |
| Filesystem1 | 512 MB |
| Filesystem2 | 7556 MB |

### DDR Memory Map

| Region | Address | Size |
|--------|---------|------|
| User Program | `0x40000000` | 8 MB |
| Database | `0x40800000` | 40 MB |
| Heap | `0x43000000` | 80 MB |

### DM30-Specific Source Files

Board configuration

* `u-boot/include/configs/dm30.h`

Board initialization, eMMC and SD card pin setup

* `u-boot/board/freescale/mx23_evk/mx23_evk.c`

Core DM30 logic: boot flow, firmware update, battery detection, power management

* `u-boot/common/dm30.c`

IT8951 E-Paper display driver (I80 parallel interface, LCDIF DMA)

* `u-boot/drivers/video/mxs/dm30.c`
* `u-boot/drivers/video/mxs/it8951_i80.c`
* `u-boot/drivers/video/mxs/it8951_i80.h`

LCDIF hardware abstraction

* `u-boot/drivers/video/mxs/lcdif.h`
* `u-boot/drivers/video/mxs/lcdif.c`
* `u-boot/drivers/video/mxs/mxsfb.c`

LCD clock configuration

* `u-boot/cpu/arm926ejs/mx23/mx23_lcd_clk.c`

TX35894XBD I2C keyboard driver

* `u-boot/drivers/input/dm30_keyb.c`

MMC/SD driver with DMA

* `u-boot/drivers/mmc/imx_ssp_mmc.c`
* `u-boot/drivers/mmc/mx23_mmc_dma.c`
* `u-boot/drivers/mmc/mxs-mmc.c`

I2C driver

* `u-boot/drivers/i2c/mxs_i2c.c`

DMA driver

* `u-boot/cpu/arm926ejs/mx23/mxs-dma.h`
* `u-boot/cpu/arm926ejs/mx23/mxs-dma.c`

MXS reset block utility

* `u-boot/include/asm-arm/arch-mx23/mxs-block.h`

### Version History

**v0.30** (current) — Major hardware revision for DM30 production model

* Rebranded from DM100 to DM30 throughout codebase
* Replaced Ortus TFT LCD with IT8951 E-Paper controller via I80 parallel interface
* Migrated primary storage from NAND Flash to eMMC
* Added TX35894XBD I2C matrix keyboard driver with full QWERTY scancode table
* Added eMMC 8-bit bus support via GPMI pin FUN3 muxing
* Reworked firmware update to write from SD card to eMMC with read-back verification
* Added Right Ctrl GPIO key for firmware update trigger

**v0.17** — Initial DM100-based bootloader

* Original code by [smmei][2] (2011)
* Ortus TFT LCD panel driver
* NAND Flash based storage
* GPIO-only key detection

### Authors

* **smmei** ([blog][2]) — Original DM100/DM30 bootloader, power management, NAND/eMMC logic (2011)
* **Zhong Yuan Huan** — IT8951 E-Paper I80 driver (2017)
* **cyliang** — Right Ctrl key support (2017)

### License

Distributed under the terms of version 2 of the [GNU General Public License][5] as published by the Free Software Foundation.

[1]: https://www.kingjim.co.jp/pomera/dm30/
[2]: https://www.cnblogs.com/sammei/
[3]: https://github.com/valeriosetti/imx-bootlets
[4]: https://www.kingjim.co.jp/
[5]: https://www.gnu.org/licenses/old-licenses/gpl-2.0.html
