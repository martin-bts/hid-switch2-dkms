obj-m += drivers/hid/
obj-m += drivers/input/joystick/

# Enable Force Feedback support
ccflags-y += -DCONFIG_NINTENDO_FF

# Resolve private kernel headers (usbhid/usbhid.h) from the kernel build
# tree when they are not overridden by local copies.
KVERSION ?= $(shell uname -r)
KBUILD := /lib/modules/$(KVERSION)/build
ccflags-y += -I$(KBUILD)/drivers/hid

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
