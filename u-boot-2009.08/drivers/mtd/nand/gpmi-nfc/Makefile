include $(TOPDIR)/config.mk

LIB	:= $(obj)libgpmi.a

ifdef CONFIG_NAND_GPMI
COBJS-y += gpmi-nfc-main.o
COBJS-y += gpmi-nfc-hal.o
COBJS-y += gpmi-nfc-hal-common.o
COBJS-y += gpmi-nfc-mil.o
COBJS-y += gpmi-nfc-rom-common.o
COBJS-y += gpmi-nfc-rom-v0.o
endif

COBJS	:= $(COBJS-y)
SRCS	:= $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS))

all:	$(LIB)

$(LIB):	$(obj).depend $(OBJS)
	$(AR) $(ARFLAGS) $@ $(OBJS)

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
