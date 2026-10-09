# 注意不能用 CC ?=：make 内置的 CC=cc 会让 ?= 失效，静默产出 x86 二进制。
# 用 := 赋值后，命令行 make CC=... 仍然可以覆盖。
CROSS_COMPILE ?= arm-linux-gnueabihf-
CC := $(CROSS_COMPILE)gcc
CFLAGS ?= -O2 -Wall -Wextra -Werror -std=c11
# 与内核驱动共用同一份 ABI 头，保证结构布局一致。
CFLAGS += -I$(CURDIR)/../../kernel/pantilt

all: pantilt_ctl

pantilt_ctl: pantilt_ctl.c ../../kernel/pantilt/pantilt_uapi.h
	$(CC) $(CFLAGS) pantilt_ctl.c -o $@

clean:
	$(RM) pantilt_ctl

.PHONY: all clean
