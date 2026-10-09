# 同 pantilt_test：不能用 CC ?=（make 内置 CC=cc 会让 ?= 失效，静默产出 x86 二进制）。
CROSS_COMPILE ?= arm-linux-gnueabihf-
CC := $(CROSS_COMPILE)gcc
CFLAGS ?= -O2 -Wall -Wextra -Werror -std=c11 -pthread

# 线格式头文件在 common/proto 下，C 侧唯一权威来源。
PROTO_DIR := ../../common/proto
CFLAGS += -I$(PROTO_DIR)

TARGET := video_link
OBJS := video_link.o ringbuf.o

.PHONY: all test clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

video_link.o: video_link.c ringbuf.h $(PROTO_DIR)/pantilt_proto.h
ringbuf.o: ringbuf.c ringbuf.h

# 宿主端单测：不碰摄像头、不碰网络，在开发机上用本机 gcc 跑。
# 必须保持用 HOSTCC，否则 ARM 二进制在 x86 开发机上根本执行不了。
HOSTCC ?= gcc
HOSTCFLAGS ?= -O2 -Wall -Wextra -Werror -std=c11 -pthread

test: test_ringbuf
	./test_ringbuf
	$(HOSTCC) $(HOSTCFLAGS) -I$(PROTO_DIR) -o test_proto test_proto.c
	./test_proto

test_ringbuf: test_ringbuf.c ringbuf.c ringbuf.h
	$(HOSTCC) $(HOSTCFLAGS) -o $@ test_ringbuf.c ringbuf.c

clean:
	$(RM) $(TARGET) $(OBJS) test_ringbuf test_proto
