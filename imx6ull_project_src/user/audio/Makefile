# 注意不能用 CC ?= ：make 内置的 CC=cc 会让 ?= 失效，静默产出 x86 二进制。
CROSS_COMPILE ?= arm-linux-gnueabihf-
CC := $(CROSS_COMPILE)gcc
CFLAGS ?= -O2 -Wall -Wextra -Werror -std=c11

# alsa-lib 头文件与库取自与板端同源的 Yocto sysroot。
# 板端 /usr/lib/libasound.so.2.0.0 与本 sysroot 里的那份 md5 相同
# （5fc673aa8a6a00d57a235768df594727），所以直接链它就能保证 ABI 一致。
#
# 为什么不用 -L$(ALSA_SYSROOT)/usr/lib -lasound：
# 那个目录里还有 Yocto 的 libc.so 链接脚本（里面写死绝对路径 /lib/libc.so.6），
# 一旦进了 -L 搜索路径，ld 会优先用它，报 "cannot find /lib/libc.so.6"。
# 直接给出 .so 全路径即可绕开，同时不引入 sysroot 里其它的库。
ALSA_SYSROOT ?= /opt/fsl-imx-x11/4.1.15-2.1.0/sysroots/cortexa7hf-neon-poky-linux-gnueabi
ALSA_INC := $(ALSA_SYSROOT)/usr/include
ALSA_LIB := $(firstword $(wildcard $(ALSA_SYSROOT)/usr/lib/libasound.so.2.*) \
                          $(ALSA_SYSROOT)/usr/lib/libasound.so)

CFLAGS += -I$(ALSA_INC)
LDLIBS := $(ALSA_LIB) -lm

TARGETS := audio_capture audio_play
COMMON_OBJS := audio_common.o

.PHONY: all clean check-alsa

all: $(TARGETS)

# 依赖缺失时给出可执行的提示，而不是让 ld 抛一堆 undefined reference。
check-alsa:
	@test -n "$(ALSA_LIB)" -a -e "$(ALSA_LIB)" || { \
	  echo "ERROR: 找不到 alsa-lib: $(ALSA_LIB)" >&2; \
	  echo "      请设置 ALSA_SYSROOT=<yocto sysroot 路径>" >&2; \
	  exit 2; }
	@test -e "$(ALSA_INC)/alsa/asoundlib.h" || { \
	  echo "ERROR: 找不到 alsa 头文件: $(ALSA_INC)/alsa/asoundlib.h" >&2; \
	  exit 2; }

audio_capture: audio_capture.o $(COMMON_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

audio_play: audio_play.o $(COMMON_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.c audio_common.h check-alsa
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	$(RM) $(TARGETS) $(COMMON_OBJS)
