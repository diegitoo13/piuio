ifneq ($(KERNELRELEASE),)
    # Kbuild part of makefile
    obj-m := piuio_hid.o 

# Building external module
else
    # Default to building for the currently running kernel
    KVER ?= $(shell uname -r)
    KDIR ?= /lib/modules/$(KVER)/build

    # If DESTDIR is given, output there for staging (e.g., packaging)
    INSTALL_MOD_PATH ?= $(DESTDIR)
    export INSTALL_MOD_PATH

    # Default installation directory within /lib/modules/$(KVER)/
    # "updates" is often checked by depmod. Override if needed.
    INSTALL_MOD_DIR ?= updates
    export INSTALL_MOD_DIR

default: modules

# Standard targets for external modules
modules modules_install clean help %::
	$(MAKE) -C $(KDIR) M=$(shell pwd) $@

endif