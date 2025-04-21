MODULE_NAME := piuio_hid
obj-m       := $(MODULE_NAME).o

KVER ?= $(shell uname -r)
KDIR := /lib/modules/$(KVER)/build
PWD  := $(shell pwd)

.PHONY: all clean install uninstall

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install: all
	sudo cp $(MODULE_NAME).ko /lib/modules/$(KVER)/extra/
	sudo depmod -a $(KVER)

uninstall:
	sudo rm -f /lib/modules/$(KVER)/extra/$(MODULE_NAME).ko
	sudo depmod -a $(KVER)
