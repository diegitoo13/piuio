obj-m := piuio_hid.o

KBUILD_PATH ?= /lib/modules/$(KVER)/build
KVER := $(shell uname -r)
MOD_DIR := /lib/modules/$(KVER)/kernel/drivers/misc/

default:
	$(MAKE) -C $(KBUILD_PATH) M=$(PWD) modules

install:
	sudo mkdir -p $(MOD_DIR)
	sudo cp $(PWD)/piuio_hid.ko $(MOD_DIR)
	sudo depmod -a
	@echo "Module piuio_hid.ko installed to $(MOD_DIR)"

uninstall:
	sudo rm -f $(MOD_DIR)/piuio_hid.ko
	sudo depmod -a
	@echo "Module piuio_hid.ko uninstalled from $(MOD_DIR)"

clean:
	$(MAKE) -C $(KBUILD_PATH) M=$(PWD) clean
