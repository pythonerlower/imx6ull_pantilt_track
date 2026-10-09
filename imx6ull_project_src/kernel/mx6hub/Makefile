ifneq ($(KERNELRELEASE),)
obj-m += mx6hub.o
mx6hub-y := mx6hub_core.o mx6hub_i2c.o mx6hub_spi.o
else
KDIR ?= /home/imx6ull/work/imx6ull/src
PWD := $(shell pwd)
ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-

all:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
