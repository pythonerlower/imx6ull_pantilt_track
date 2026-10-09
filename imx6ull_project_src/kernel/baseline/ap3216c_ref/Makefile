ifneq ($(KERNELRELEASE),)
obj-m += ap3216c.o
else
KDIR ?= /home/imx6ull/work/imx6ull/src
PWD := $(shell pwd)
ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-

CC := $(CROSS_COMPILE)gcc
STRIP := $(CROSS_COMPILE)strip

all:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules
	@echo "Build complete!"
	@echo "Output: ap3216c.ko"

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f *.o *.ko *.mod.c *.symvers *.order .*.cmd Module.markers modules.order
	rm -rf .tmp_versions

install:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules_install

.PHONY: all clean install
endif
