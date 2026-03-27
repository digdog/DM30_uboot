#
# image should be loaded at 0x41008000
#
LDSCRIPT := $(SRCTREE)/board/$(VENDOR)/$(BOARD)/u-boot.lds

#TEXT_BASE = 0x41080000
TEXT_BASE = 0x47f80000
