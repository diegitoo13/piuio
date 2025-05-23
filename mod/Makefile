# Makefile — build & install the piuio.ko kernel module
# Usage:
#   make            → build the module
#   sudo make install   → install to /lib/modules/$(uname -r)/updates
#   sudo make uninstall → remove the installed module

# ── module name ────────────────────────────────────────────────
obj-m := piuio.o            # piuio.c → piuio.ko

# ── kernel build directory (override with KDIR=…) ─────────────
KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build
PWD  := $(shell pwd)

# ── install sub-dir under /lib/modules/$(KVER)/ ───────────────
INSTALL_MOD_DIR ?= updates
export INSTALL_MOD_DIR

# ── phony targets ──────────────────────────────────────────────
.PHONY: all modules clean install uninstall

# default
all: modules

# build / clean via the kernel build system
modules clean: %:
	$(MAKE) -C "$(KDIR)" M="$(PWD)" $@

# install via kernel ‘modules_install’
install: modules
	$(MAKE) -C "$(KDIR)" M="$(PWD)" modules_install
	@sudo depmod -a

# remove installed module
uninstall:
	@sudo rm -f "/lib/modules/$(KVER)/$(INSTALL_MOD_DIR)/$(obj-m:.o=.ko)"
	@sudo depmod -a