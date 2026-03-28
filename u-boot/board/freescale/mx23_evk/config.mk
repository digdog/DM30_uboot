#
# image should be loaded at 0x41008000
#
LDSCRIPT := $(SRCTREE)/board/$(VENDOR)/$(BOARD)/u-boot.lds

#TEXT_BASE = 0x41080000
TEXT_BASE = 0x47f80000

# GCC 15 compatibility:
# - gnu11: GCC 15 defaults to C23 where 'bool' is a keyword
# - fgnu89-inline: allow weak alias to external symbols (GCC 15 rejects this
#   pattern used throughout U-Boot for providing default weak implementations)
# - Wno-dangling-pointer: false positive in fs/fat/fat.c (local ptr safe within loop)
#
# Note: -Wno-implicit-function-declaration goes in CFLAGS (appended after
# config.mk defines CFLAGS) because GCC 15 promotes it to a hard error and
# there are too many legacy implicit calls in this 2009-era codebase.
PLATFORM_CPPFLAGS += -std=gnu11 -fgnu89-inline \
	-Wno-dangling-pointer -Wno-implicit-function-declaration
