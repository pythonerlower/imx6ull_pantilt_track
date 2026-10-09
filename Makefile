ROOT := $(shell pwd)
# 程序源码与构建产物目录；本 Makefile 与 scripts/ 只做调度，不存放程序源文件。
SRC := $(ROOT)/imx6ull_project_src

.PHONY: all host-check build deploy pantilt-test pantilt-ko pantilt-ctl clean baseline \
	video-link pantilt-daemon test audio securityd

all: build

host-check:
	bash $(ROOT)/scripts/check_host_env.sh

build:
	bash $(ROOT)/scripts/build_all.sh

deploy:
	bash $(ROOT)/scripts/deploy_offline.sh

pantilt-test:
	$(MAKE) -C $(SRC)/user/pantilt_test CC=$${CC_USER:-arm-linux-gnueabihf-gcc}

# 正式控制接口：内核驱动 + 3A 验收工具
pantilt-ko:
	$(MAKE) -C $(SRC)/kernel/pantilt KDIR=$${KERNEL_DIR:-/home/imx6ull/work/imx6ull/src}

pantilt-ctl:
	$(MAKE) -C $(SRC)/user/pantilt_ctl CROSS_COMPILE=$${CROSS_COMPILE:-arm-linux-gnueabihf-}

# 3B 的板端二进制：视频链路与控制面 daemon
video-link:
	$(MAKE) -C $(SRC)/user/video_link CROSS_COMPILE=$${CROSS_COMPILE:-arm-linux-gnueabihf-}

pantilt-daemon:
	$(MAKE) -C $(SRC)/user/pantilt_daemon CROSS_COMPILE=$${CROSS_COMPILE:-arm-linux-gnueabihf-}

# 开发机上能跑的全部测试。刻意不碰板子：
# 视频链路的环形缓冲、线协议两端一致性、PC 侧重组/控制，以及
# 用 --fake-driver 把整条控制环跑通的端到端联调。
test:
	$(MAKE) -C $(SRC)/user/video_link test
	$(MAKE) -C $(SRC)/user/pantilt_daemon test

baseline:
	bash $(ROOT)/scripts/build_baseline.sh

# Phase 5 的板端二进制：WM8960 用户态 ALSA 录音/播放
audio:
	$(MAKE) -C $(SRC)/user/audio CROSS_COMPILE=$${CROSS_COMPILE:-arm-linux-gnueabihf-}

# Phase 6 的板端状态机。它不链 pantilt 的 ABI 头，所以这里不需要 KERNEL_DIR。
securityd:
	$(MAKE) -C $(SRC)/user/securityd CROSS_COMPILE=$${CROSS_COMPILE:-arm-linux-gnueabihf-}

clean:
	$(MAKE) -C $(SRC)/kernel/baseline/led clean || true
	$(MAKE) -C $(SRC)/kernel/baseline/beep clean || true
	$(MAKE) -C $(SRC)/kernel/baseline/ap3216c_myself clean || true
	$(MAKE) -C $(SRC)/kernel/baseline/ap3216c_ref clean || true
	$(MAKE) -C $(SRC)/kernel/baseline/icm20608 clean || true
	$(MAKE) -C $(SRC)/kernel/mx6hub clean || true
	$(MAKE) -C $(SRC)/user/baseline/led clean || true
	$(MAKE) -C $(SRC)/user/baseline/beep clean || true
	$(MAKE) -C $(SRC)/user/baseline/ap3216c_myself clean || true
	$(MAKE) -C $(SRC)/user/baseline/ap3216c_ref clean || true
	$(MAKE) -C $(SRC)/user/baseline/icm20608 clean || true
	$(MAKE) -C $(SRC)/user/hub_monitor clean || true
	$(MAKE) -C $(SRC)/user/v4l2_fb_preview clean || true
	$(MAKE) -C $(SRC)/kernel/pantilt clean || true
	$(MAKE) -C $(SRC)/user/pantilt_ctl clean || true
	$(MAKE) -C $(SRC)/user/video_link clean || true
	$(MAKE) -C $(SRC)/user/pantilt_daemon clean || true
	$(MAKE) -C $(SRC)/kernel/iio/icm20608_iio clean || true
	$(MAKE) -C $(SRC)/kernel/input/ap3216c_input clean || true
	$(MAKE) -C $(SRC)/user/icm20608_iio clean || true
	$(MAKE) -C $(SRC)/user/ap3216c_input clean || true
	$(MAKE) -C $(SRC)/user/audio clean || true
	$(MAKE) -C $(SRC)/user/securityd clean || true
	$(MAKE) -C $(SRC)/pc/tracker clean || true
