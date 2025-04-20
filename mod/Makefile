# Basic PIUIO HID driver Makefile

obj-m         := piuio_hid.o

# Kernel build settings
KVER          := $(shell uname -r)
KBUILD_PATH   := /lib/modules/$(KVER)/build

# Installation directory for the module
MOD_DIR       := /lib/modules/$(KVER)/kernel/drivers/misc

.PHONY: all modules install uninstall clean

# Default: build the module
all: modules

modules:
	$(MAKE) -C $(KBUILD_PATH) M=$(PWD) modules

install: modules
	sudo mkdir -p $(MOD_DIR)
	sudo cp piuio_hid.ko $(MOD_DIR)
	sudo depmod -a

uninstall:
	sudo rm -f $(MOD_DIR)/piuio_hid.ko
	sudo depmod -a

clean:
	$(MAKE) -C $(KBUILD_PATH) M=$(PWD) clean