############################################################################
# configs/omega/src/Makefile
############################################################################

-include $(TOPDIR)/Make.defs

CFLAGS		+= -I$(TOPDIR)/sched

ASRCS		= 
AOBJS		= $(ASRCS:.S=$(OBJEXT))
CSRCS		= sam_boot.c sam_flash.c sam_loader.c sam_gpbr.c


ifdef CONFIG_HAVE_CXXINITIALIZE
CSRCS		+= up_cxxinitialize.c
endif

ifdef CONFIG_NET
CSRCS		+= up_network.c
endif

ifdef CONFIG_NSH_ARCHINIT
CSRCS		+= up_nsh.c
endif

ifdef CONFIG_ARCH_LEDS
CSRCS		+=	sam_autoleds.c
endif

ifdef CONFIG_SAM34_SLCDC
CSRCS		+=	sam_slcdc.c
endif

ifdef CONFIG_SAM34_METROLOGY
CSRCS		+=	sam_metrology.c
endif

ifdef CONFIG_SAM34_SPI0
CSRCS		+=	sam_spi.c
ifdef CONFIG_MTD_AT25
CSRCS		+=	sam_at25.c
endif
endif

ifdef CONFIG_MTD_AT24XX
CSRCS		+=	sam_at24.c
endif

ifdef CONFIG_SAM_KEXEC
CSRCS		+=	sam_kexec.c
endif

ifdef CONFIG_FS_FIXFS
CSRCS		+=	fix_files.c
endif

ifdef CONFIG_OMEGA_RF
CSRCS		+=	sam_rf.c
endif

COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

ARCH_SRCDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(WINTOOL),y)
  CFLAGS	+= -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}" \
  		   -I "${shell cygpath -w $(ARCH_SRCDIR)/common}" \
  		   -I "${shell cygpath -w $(ARCH_SRCDIR)/armv7-m}"
else
  CFLAGS	+= -I$(ARCH_SRCDIR)/chip -I$(ARCH_SRCDIR)/common -I$(ARCH_SRCDIR)/armv7-m
endif

all: libboard$(LIBEXT)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libboard$(LIBEXT): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, libboard$(LIBEXT))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
