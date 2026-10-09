# 同 video_link / pantilt_daemon：不能用 CC ?=（make 内置 CC=cc 会让 ?= 失效，
# 静默产出 x86 二进制）。
CROSS_COMPILE ?= arm-linux-gnueabihf-
CC := $(CROSS_COMPILE)gcc
CFLAGS ?= -O2 -Wall -Wextra -Werror -std=c11

# Phase 7 的事件线格式在 common/proto/security_event.h。只引这一个头，
# 不引 pantilt_proto.h —— 保持"securityd 不含云台 ABI"这条结构性事实。
PROTO_DIR := ../../common/proto
CFLAGS += -I$(PROTO_DIR)

# 2026-10-08：振动判据从"合矢量幅值"换成"逐轴帧间差分"后不再用 sqrt，
# libm 也就用不着了（原来只为了那一次开方）。

TARGET := securityd

.PHONY: all clean

all: $(TARGET)

$(TARGET): securityd.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

securityd.o: securityd.c $(PROTO_DIR)/security_event.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	$(RM) $(TARGET) securityd.o
