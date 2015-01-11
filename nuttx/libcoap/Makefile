.EXPORT_ALL_VARIABLES:

include $(TOPDIR)/.config

OUTPUT       := "$(CURDIR)/output"
CONFIG_OPTS  := --prefix=$(OUTPUT) --build=pc

ifdef HOSTBUILD
	CFLAGS += -m32 -g
	AR     := $(AR) rcu
else
	include ${TOPDIR}/Make.defs
	CONFIG_OPTS += --host=arm-nuttx-eabi
endif

ifeq ($(CONFIG_LIB_COAP_WITH_MEMB), y)
	CONFIG_OPTS += --with-memb
endif

ifeq ($(CONFIG_LIB_COAP_WITHOUT_ASYNC), y)
	CONFIG_OPTS += --without-async
endif

ifeq ($(CONFIG_LIB_COAP_WITHOUT_OBSERVE), y)
	CONFIG_OPTS += --without-observe
endif

ifdef CONFIG_LIB_COAP_MAX_RESOURCES
	CFLAGS += -DCOAP_MAX_RESOURCES=$(CONFIG_LIB_COAP_MAX_RESOURCES)
endif

ifdef CONFIG_LIB_COAP_MAX_ATTRIBUTES
	CFLAGS += -DCOAP_MAX_ATTRIBUTES=$(CONFIG_LIB_COAP_MAX_ATTRIBUTES)
endif

ifdef CONFIG_LIB_COAP_MAX_SUBSCRIBERS
	CFLAGS += -DCOAP_MAX_SUBSCRIBERS=$(CONFIG_LIB_COAP_MAX_SUBSCRIBERS)
endif

ifdef CONFIG_LIB_COAP_MAX_PDU_SIZE
	CFLAGS += -DCOAP_MAX_PDU_SIZE=$(CONFIG_LIB_COAP_MAX_PDU_SIZE)
endif

ifdef CONFIG_LIB_COAP_PDU_MAXCNT
	CFLAGS += -DCOAP_PDU_MAXCNT=$(CONFIG_LIB_COAP_PDU_MAXCNT)
endif

ifdef CONFIG_LIB_COAP_RESOURCES_NOHASH
	CFLAGS += -DCOAP_RESOURCES_NOHASH
endif

SRC_DIR := libcoap-4.1.1

.PHONY: all libcoap.a

all: libcoap.a

libcoap.a: $(SRC_DIR)/Makefile
	$(MAKE) -C $(SRC_DIR) ARFLAGS="" $@
	cp $(SRC_DIR)/$@ $(CURDIR)/$@

$(SRC_DIR)/Makefile: Makefile
	cd $(SRC_DIR); LDFLAGS="$(LDFLAGS)" CFLAGS="$(CFLAGS)" ARFLAGS="" ./configure $(CONFIG_OPTS)

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CXX)" -- $(CXXFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	-$(MAKE) -C $(SRC_DIR) clean
	-rm -rf $(OUTPUT)
	-rm -rf $(SRC_DIR)/Makefile
	-rm -f $(SRC_DIR)/*.gcno $(SRC_DIR)/*.gcda $(SRC_DIR)/*.gcna

distclian: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)



