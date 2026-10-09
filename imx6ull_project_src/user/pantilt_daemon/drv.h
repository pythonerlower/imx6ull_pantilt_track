/*
 * 驱动访问的薄接口。
 *
 * 为什么要抽这一层：daemon 里最容易写错、又最难在板上验证的，恰恰是协议校验
 * （seq 判重、新鲜度、值域）和两级看门狗——板上要测这些得手造畸形包、还得
 * 盯着舵机别乱动。抽一层之后可以用内存模拟在开发机上把整条控制环跑通，
 * 见 fake_drv.c 与 test_plane.py。
 *
 * 真实实现（real_drv.c）就是 ioctl 的一层直译，不做任何判断——
 * 所有策略都在 daemon.c 里，避免"两处都以为对方会检查"。
 */
#ifndef PANTILT_DRV_H
#define PANTILT_DRV_H

#include <stdbool.h>
#include <stdint.h>

#include "pantilt_uapi.h"

struct drv;

/* 每次动作返回 0 成功、负 errno 失败。失败必须原样上报，不吞。 */
struct drv_ops {
	int (*set_target)(struct drv *d, uint16_t pan, uint16_t tilt);
	int (*home)(struct drv *d);
	int (*stop)(struct drv *d);
	/* 只推看门狗截止时刻，不产生任何运动。见 pantilt_uapi.h 的说明。 */
	int (*keepalive)(struct drv *d);
	int (*get_state)(struct drv *d, struct pantilt_state *out);
	/* 模拟实现用它推进内部的爬升与看门狗；真实实现里是空操作。 */
	void (*tick)(struct drv *d);
	void (*close)(struct drv *d);
};

struct drv {
	const struct drv_ops *ops;
	void *ctx;
	/*
	 * daemon 每轮开始前写入的单调毫秒时间。模拟实现要靠它推进爬升和看门狗，
	 * 尤其是 keepalive —— 它必须真的把截止时刻往后推，否则"续命能防止
	 * 长距离移动被掐断"这条在开发机上就测不出来，而那正是本次加 KEEPALIVE
	 * 的全部理由。真实实现完全不读这个字段。
	 */
	uint64_t now_ms;
};

/* 真实驱动：打开 /dev/pantilt0。失败返回 NULL 并填 errno。 */
struct drv *drv_open_real(const char *path);

/*
 * 模拟驱动：在内存里做同样的钳位、逐 tick 爬升与看门狗，不碰硬件。
 * 极限值取自 config/pantilt_calibration.conf，便于开发机上跑通整条控制环。
 */
struct drv *drv_open_fake(void);

#endif /* PANTILT_DRV_H */
