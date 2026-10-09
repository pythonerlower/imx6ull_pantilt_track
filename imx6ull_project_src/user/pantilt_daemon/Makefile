# 同 video_link：不能用 CC ?=（make 内置 CC=cc 会让 ?= 失效，静默产出 x86 二进制）。
CROSS_COMPILE ?= arm-linux-gnueabihf-
CC := $(CROSS_COMPILE)gcc
CFLAGS ?= -O2 -Wall -Wextra -Werror -std=c11

# 线格式在 common/proto；pantilt_uapi.h 在 kernel/pantilt。
PROTO_DIR := ../../common/proto
KERNEL_DIR := ../../kernel/pantilt
CFLAGS += -I$(PROTO_DIR) -I$(KERNEL_DIR)

TARGET := pantilt_daemon
OBJS := pantilt_daemon.o real_drv.o fake_drv.o

# 板端验证工具：确认 KEEPALIVE 只续命、且能防止看门狗掐断长移动。
# 它会真的动舵机，所以单独一个目标，不进 all 之外的地方。
CHECK := check_keepalive

.PHONY: all test clean

all: $(TARGET) $(CHECK)

$(CHECK): check_keepalive.c $(KERNEL_DIR)/pantilt_uapi.h
	$(CC) $(CFLAGS) -o $@ $<

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

pantilt_daemon.o: pantilt_daemon.c drv.h $(PROTO_DIR)/pantilt_proto.h \
                  $(PROTO_DIR)/security_event.h
real_drv.o: real_drv.c drv.h $(KERNEL_DIR)/pantilt_uapi.h
fake_drv.o: fake_drv.c drv.h $(KERNEL_DIR)/pantilt_uapi.h

# --- 宿主端整条控制环的联调 ---
#
# 用 --fake-driver 把 daemon 编成 x86 可执行文件，让 PC 侧真正的
# ControlClient / TelemReceiver 跑在环回上。这样协议校验、看门狗、
# 去重逻辑在开发机上就能验证，不必先冒险上板。
# 注意 fake_drv.c 只复刻钳位/爬升/看门狗，不代表任何电气或时序特性。
HOSTCC ?= gcc
HOSTCFLAGS ?= -O2 -Wall -Wextra -Werror -std=c11
HOST_DAEMON := pantilt_daemon_host
TRACKER_DIR := ../../pc/tracker

$(HOST_DAEMON): pantilt_daemon.c real_drv.c fake_drv.c drv.h
	$(HOSTCC) $(HOSTCFLAGS) -I$(PROTO_DIR) -I$(KERNEL_DIR) -o $@ \
		pantilt_daemon.c real_drv.c fake_drv.c

test: $(HOST_DAEMON)
	./$(HOST_DAEMON) --help >/dev/null
	$(MAKE) -C $(TRACKER_DIR) test
	python3 $(TRACKER_DIR)/test_plane.py --daemon $(abspath $(HOST_DAEMON))

clean:
	$(RM) $(TARGET) $(CHECK) $(OBJS) $(HOST_DAEMON)
