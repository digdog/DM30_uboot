include $(TOPDIR)/config.mk

LIB	:= $(obj)libmxsvideo.a

COBJS-$(CONFIG_VIDEO_MX23) += mxsfb.o lcdif.o dm30.o it8951_i80.o

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

