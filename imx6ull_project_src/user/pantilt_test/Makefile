# 注意不能用 CC ?=：make 内置的 CC=cc 会让 ?= 失效，静默产出 x86 二进制。
# 之前只因根 Makefile 显式传了 CC= 才没暴露；用 := 赋值后命令行 make CC=... 仍可覆盖。
CROSS_COMPILE ?= arm-linux-gnueabihf-
CC := $(CROSS_COMPILE)gcc
CFLAGS ?= -O2 -Wall -Wextra -Werror -std=c11

all: pantilt_test

pantilt_test: pantilt_test.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	$(RM) pantilt_test

.PHONY: all clean
