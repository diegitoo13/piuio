# Module object name (e.g., piuio_hid.o if your C file is piuio_hid.c)
obj-m         := piuio.o

# Kernel version/directories (allow overrides via command line)
KVER          ?= $(shell uname -r)
KDIR          ?= /lib/modules/$(KVER)/build
PWD           := $(shell pwd)

# Subdirectory within /lib/modules/$(KVER)/ for installation (e.g., "updates", "extra")
INSTALL_MOD_DIR ?= updates
export INSTALL_MOD_DIR

# Phony targets
.PHONY: all modules clean install uninstall modules_install

# Default target
all: modules

# Delegate build, clean, and Kbuild's install to the kernel build system
modules clean modules_install: %:
	# --- TAB --- below! Ensure Tab indentation!
	$(MAKE) -C "$(KDIR)" M="$(PWD)" $@

# Use Kbuild's 'modules_install' for the install step
install: modules_install

# Uninstall requires manual removal
uninstall:
	# --- TAB --- below! Ensure Tab indentation!
	sudo rm -f "/lib/modules/$(KVER)/$(INSTALL_MOD_DIR)/$(obj-m:.o=.ko)"
	# --- TAB --- below! Ensure Tab indentation!
	sudo depmod -a