obj-m += drivers/hid/
obj-m += drivers/input/joystick/

# Enable Force Feedback support
subdir-ccflags-y += -DCONFIG_NINTENDO_FF

# Resolve private kernel headers (usbhid/usbhid.h) from the kernel build
# tree when they are not overridden by local copies.
KVERSION ?= $(shell uname -r)
KBUILD := /lib/modules/$(KVERSION)/build
subdir-ccflags-y += -I$(KBUILD)/drivers/hid

# Auto-detect if the kernel was built with the full LLVM toolchain (e.g. CachyOS)
KERNEL_CC := $(shell sed -n 's/^CONFIG_CC_IS_CLANG=y/clang/p' $(KBUILD)/.config 2>/dev/null)
KERNEL_LD := $(shell sed -n 's/^CONFIG_LD_IS_LLD=y/lld/p' $(KBUILD)/.config 2>/dev/null)
ifeq ($(KERNEL_CC)$(KERNEL_LD),clanglld)
  LLVM_FLAG := LLVM=1
else ifneq ($(KERNEL_CC),)
  LLVM_FLAG := CC=clang
endif

all:
	$(MAKE) -C $(KBUILD) M=$(PWD) $(LLVM_FLAG) modules

clean:
	$(MAKE) -C $(KBUILD) M=$(PWD) $(LLVM_FLAG) clean

# ---- DKMS packaging ----

DKMS_NAME := hid-nintendo-switch2
DKMS_VERSION := $(shell cat VERSION 2>/dev/null || echo 0.0.0)
DKMS_DEST := /usr/src/$(DKMS_NAME)-$(DKMS_VERSION)

dkms: dkms-install

dkms-install: dkms-src
	sudo dkms add $(DKMS_NAME)/$(DKMS_VERSION)
	sudo dkms build $(DKMS_NAME)/$(DKMS_VERSION)
	sudo dkms install $(DKMS_NAME)/$(DKMS_VERSION)

dkms-src:
	sudo mkdir -p $(DKMS_DEST)/drivers/hid $(DKMS_DEST)/drivers/input/joystick
	sudo cp Makefile VERSION $(DKMS_DEST)/
	sudo cp drivers/hid/Makefile \
		drivers/hid/hid-nintendo.c \
		drivers/hid/hid-nintendo.h \
		drivers/hid/hid-ids.h \
		$(DKMS_DEST)/drivers/hid/
	sudo cp drivers/input/joystick/Makefile \
		drivers/input/joystick/nintendo-switch2-usb.c \
		drivers/input/joystick/nintendo-switch2-ble.c \
		$(DKMS_DEST)/drivers/input/joystick/
	sudo sed 's/__VERSION__/$(DKMS_VERSION)/' dkms.conf \
		> /tmp/dkms.conf.$$$$ && \
		sudo mv /tmp/dkms.conf.$$$$ $(DKMS_DEST)/dkms.conf

dkms-remove:
	-sudo dkms remove $(DKMS_NAME)/$(DKMS_VERSION) --all
	sudo rm -rf $(DKMS_DEST)

.PHONY: all clean dkms dkms-install dkms-src dkms-remove
