#!/bin/bash
#
# Build dm30-uboot.raw from U-Boot v0.30 source + i.MX Bootlets
#
# Prerequisites:
#   brew install arm-none-eabi-gcc arm-none-eabi-binutils
#   elftosb (https://github.com/digdog/elftosb) built in ../elftosb/
#
# Usage:
#   cd /path/to/DM30_uboot
#   ./build_dm30_uboot.sh
#

set -e

BASEDIR="$(cd "$(dirname "$0")" && pwd)"
UBOOT_DIR="$BASEDIR/u-boot"
BOOTLETS_DIR="$BASEDIR/imx-bootlets"

# arm-none-eabi- works for bare-metal ARM; this old U-Boot (2009.08) may
# produce warnings with GCC 15 but should still generate valid code.
CROSS=arm-none-eabi-

echo "============================================"
echo " Checking toolchain"
echo "============================================"
MISSING=""
if ! command -v ${CROSS}gcc &>/dev/null; then
    MISSING="$MISSING arm-none-eabi-gcc"
fi
if ! command -v ${CROSS}ld &>/dev/null; then
    MISSING="$MISSING arm-none-eabi-binutils"
fi
if [ -n "$MISSING" ]; then
    echo "ERROR: Missing required tools."
    echo ""
    echo "Install with Homebrew:"
    echo "  brew install$MISSING"
    echo ""
    exit 1
fi
echo "Compiler: $(${CROSS}gcc --version | head -1)"
echo ""

# ──────────────────────────────────────────────
echo "============================================"
echo " Step 1: Build U-Boot"
echo "============================================"
cd "$UBOOT_DIR"

# Clean previous build
make distclean 2>/dev/null || true

# Configure for DM30
make dm30_config CROSS_COMPILE=$CROSS

# Workaround: the tools/ host build is broken on macOS because pattern rules
# using $(SRCTREE)/lib_generic/%.c and $(SRCTREE)/libfdt/%.c don't resolve
# in the sub-make context. We symlink all external source files into tools/
# so that the implicit rules can find them.
echo "Applying macOS tools/ build workaround..."
for f in lib_generic/crc32.c lib_generic/md5.c lib_generic/sha1.c \
         common/env_embedded.c common/image.c; do
    base=$(basename "$f")
    ln -sf "../$f" "tools/$base" 2>/dev/null || true
done
# libfdt sources
for f in libfdt/*.c; do
    base=$(basename "$f")
    ln -sf "../$f" "tools/$base" 2>/dev/null || true
done

# Build (single-threaded — this old Makefile has issues with parallel on macOS)
# GCC 15 compat flags are in board/freescale/mx23_evk/config.mk
make CROSS_COMPILE=$CROSS HOSTCC=cc -j1 2>&1 | tee "$BASEDIR/build_uboot.log"

if [ ! -f "$UBOOT_DIR/u-boot" ]; then
    echo ""
    echo "ERROR: U-Boot build failed. Check build_uboot.log"
    exit 1
fi

echo ""
echo "U-Boot ELF built: $UBOOT_DIR/u-boot"
${CROSS}size "$UBOOT_DIR/u-boot"
echo ""

# ──────────────────────────────────────────────
echo "============================================"
echo " Step 2: Build i.MX Bootlets"
echo "============================================"
cd "$BOOTLETS_DIR"

# Clean
make clean BOARD=stmp378x_dev CROSS_COMPILE=$CROSS || true

# Build power_prep and boot_prep
make boot_prep power_prep BOARD=stmp378x_dev CROSS_COMPILE=$CROSS 2>&1 | tee "$BASEDIR/build_bootlets.log"

if [ ! -f "$BOOTLETS_DIR/boot_prep/boot_prep" ] || [ ! -f "$BOOTLETS_DIR/power_prep/power_prep" ]; then
    echo "ERROR: Bootlets build failed. Check build_bootlets.log"
    exit 1
fi

echo ""
echo "Bootlets built:"
ls -la "$BOOTLETS_DIR/boot_prep/boot_prep"
ls -la "$BOOTLETS_DIR/power_prep/power_prep"
echo ""

# ──────────────────────────────────────────────
echo "============================================"
echo " Step 3: Package .sb boot stream"
echo "============================================"

# Copy U-Boot ELF to bootlets directory (uboot_prebuilt.db expects ./u-boot)
cp "$UBOOT_DIR/u-boot" "$BOOTLETS_DIR/u-boot"

# Find elftosb2: check PATH, then common local locations
ELFTOSB=""
if command -v elftosb2 &>/dev/null; then
    ELFTOSB=elftosb2
elif command -v elftosb &>/dev/null; then
    ELFTOSB=elftosb
elif [ -x "$BASEDIR/../elftosb/build/Release/elftosb" ]; then
    ELFTOSB="$BASEDIR/../elftosb/build/Release/elftosb"
elif [ -x "$BASEDIR/../elftosb/bld/macos/elftosb" ]; then
    ELFTOSB="$BASEDIR/../elftosb/bld/macos/elftosb"
fi

if [ -n "$ELFTOSB" ]; then
    cd "$BOOTLETS_DIR"
    "$ELFTOSB" -z -c ./uboot_prebuilt.db -o imx23_uboot.sb
    cp imx23_uboot.sb "$BASEDIR/dm30-uboot.raw"
    echo ""
    echo "============================================"
    echo " SUCCESS: dm30-uboot.raw generated!"
    echo "============================================"
    ls -la "$BASEDIR/dm30-uboot.raw"
else
    echo ""
    echo "WARNING: elftosb not found!"
    echo ""
    echo "The U-Boot ELF and bootlets are ready, but the final .sb packaging"
    echo "requires the elftosb tool. To build it:"
    echo ""
    echo "  git clone https://github.com/digdog/elftosb.git ../elftosb"
    echo "  cd ../elftosb"
    echo "  mkdir -p bld/macos"
    echo "  make -C bld/macos -f \$(pwd)/makefile.rules SRC_DIR=\$(pwd) UNAMES=Linux elftosb"
    echo ""
    echo "Then re-run this script."
    echo ""
    echo "Files ready for packaging:"
    ls -la "$BOOTLETS_DIR/u-boot"
    ls -la "$BOOTLETS_DIR/boot_prep/boot_prep"
    ls -la "$BOOTLETS_DIR/power_prep/power_prep"
fi
