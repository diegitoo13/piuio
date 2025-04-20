obj-m         := piuio_hid.o

KVER          ?= $(shell uname -r)
KBUILD_PATH   := /lib/modules/$(KVER)/build
MOD_DIR       := /lib/modules/$(KVER)/kernel/drivers/misc

.PHONY: all modules install uninstall clean prepare-headers

all: modules

prepare-headers:
	# --- TAB --- below!
	$(MAKE) -C $(KBUILD_PATH) scripts

modules: prepare-headers
	# --- TAB --- below!
	$(MAKE) -C $(KBUILD_PATH) M=$(PWD) modules

install: modules
	# --- TAB --- below!
	sudo mkdir -p $(MOD_DIR)
	# --- TAB --- below!
	sudo cp piuio_hid.ko $(MOD_DIR)
	# --- TAB --- below!
	sudo depmod -a

uninstall:
	# --- TAB --- below!
	sudo rm -f $(MOD_DIR)/piuio_hid.ko
	# --- TAB --- below!
	sudo depmod -a

clean:
	# --- TAB --- below!
	$(MAKE) -C $(KBUILD_PATH) M=$(PWD) clean

