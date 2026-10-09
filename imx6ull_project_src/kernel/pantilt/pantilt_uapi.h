#ifndef __PANTILT_UAPI_H__
#define __PANTILT_UAPI_H__

/*
 * /dev/pantilt0 的用户态 ABI。内核与用户态共用本文件，保证结构布局一致。
 *
 * 单位一律是 PCA9685 在 50 Hz 下的 tick（1 tick ≈ 4.883 µs ≈ 0.4395°）。
 * 角度只用于打印和人工估算，任何下发值都用 tick —— 见 rules/Hardware.md §1.1。
 */

#include <linux/ioctl.h>
#include <linux/types.h>

#define PANTILT_IOC_MAGIC 'p'

/* 轴序，所有长度为 PANTILT_AXES 的数组都按这个顺序索引。 */
#define PANTILT_AXIS_PAN 0
#define PANTILT_AXIS_TILT 1
#define PANTILT_AXES 2

/*
 * 芯片状态，由驱动在 insmod 时只读探测得到，GET_STATE.chip_state 返回。
 * 只有 ASLEEP/RUNNING 两种已知态允许驱动动作；UNKNOWN 时驱动拒绝一切输出，
 * 避免覆盖别的控制器正在写出的 PWM。
 */
#define PANTILT_CHIP_UNKNOWN 0
#define PANTILT_CHIP_ASLEEP 1 /* 上电默认静止态：SLEEP=1、通道全关 */
#define PANTILT_CHIP_RUNNING 2 /* 已是本项目的 50 Hz 运行态，可读到有效脉宽 */

/* 驱动运行状态，GET_STATE.status 返回。 */
#define PANTILT_ST_UNINIT 0 /* 未 SET_LIMIT，不会输出任何脉宽 */
#define PANTILT_ST_READY 1 /* 已就绪，停在 cur_tick，无动作 */
#define PANTILT_ST_SLEW 2 /* 正按 slew_ms 逐 tick 逼近 tgt_tick */
#define PANTILT_ST_HOLD 3 /* 保持当前位置并拒绝继续动作，需 SET_TARGET/HOME 解除 */

/*
 * SET_LIMIT：配置 + 认领芯片 + 爬回 home。
 *
 * 这是本驱动唯一的"启动"入口，也是唯一会唤醒 PCA9685 的调用。它做四件事：
 *   1. 校验全部字段（非法值直接拒绝，不静默钳位）；
 *   2. 按 start[] 唤醒芯片，避免上电瞬间跳变；
 *   3. 以 slew_ms 的节奏缓慢爬回 home[]；
 *   4. 之后才允许 SET_TARGET。
 *
 * insmod 本身不写任何寄存器，因此加载模块永远是安全的。
 */
struct pantilt_limit {
	__u16 start[PANTILT_AXES]; /* 唤醒时的初始 tick；芯片已在运行时忽略 */
	__u16 home[PANTILT_AXES]; /* 安全姿态，必须落在 [safe_min, safe_max] 内 */
	__u16 safe_min[PANTILT_AXES];
	__u16 safe_max[PANTILT_AXES];
	__u16 slew_ms; /* 每 tick 的步进间隔，1..100，建议 15 */
	__u16 watchdog_ms; /* 超时无新目标 → HOLD，0 表示禁用 */
};

/* SET_TARGET：绝对目标 tick，驱动内钳位到 [safe_min, safe_max] 后限速逼近。 */
struct pantilt_target {
	__u16 tick[PANTILT_AXES];
};

struct pantilt_state {
	__u64 cmd_count; /* 累计接受的 SET_TARGET 次数 */
	__u64 clamp_low[PANTILT_AXES]; /* 目标低于 safe_min 被钳位的次数 */
	__u64 clamp_high[PANTILT_AXES]; /* 目标高于 safe_max 被钳位的次数 */
	__u64 i2c_err; /* 累计 I2C 传输错误 */
	__u64 watchdog_trips; /* 看门狗强制进入 HOLD 的次数 */
	__u64 last_update_us; /* 最近一次真正写出 PWM 的时刻（单调 µs） */
	__u64 last_latency_us; /* SET_TARGET 接收到首次写出 PWM 的耗时 */

	__u32 status; /* PANTILT_ST_* */
	__u32 chip_state; /* PANTILT_CHIP_* */
	__u32 slew_ms;
	__u32 watchdog_ms;

	__u16 cur_tick[PANTILT_AXES]; /* 驱动认为的当前 tick */
	__u16 tgt_tick[PANTILT_AXES]; /* 当前目标 tick */
	__u16 home[PANTILT_AXES];
	__u16 safe_min[PANTILT_AXES];
	__u16 safe_max[PANTILT_AXES];

	__u32 reserved[4]; /* 预留；加入新字段前先确认用户态兼容 */
};

#define PANTILT_IOC_SET_LIMIT \
	_IOW(PANTILT_IOC_MAGIC, 0x01, struct pantilt_limit)
#define PANTILT_IOC_SET_TARGET \
	_IOW(PANTILT_IOC_MAGIC, 0x02, struct pantilt_target)
#define PANTILT_IOC_GET_STATE \
	_IOR(PANTILT_IOC_MAGIC, 0x03, struct pantilt_state)
/* 再次回到 home：运行中的重定位入口，语义与 SET_LIMIT 的第 3 步相同。 */
#define PANTILT_IOC_HOME _IO(PANTILT_IOC_MAGIC, 0x04)
/* 立即停止爬升并保持当前位置进入 HOLD；不改变已输出的脉宽。 */
#define PANTILT_IOC_STOP _IO(PANTILT_IOC_MAGIC, 0x05)

/*
 * KEEPALIVE：只把看门狗的截止时刻往后推，别的什么都不动。
 *
 * 为什么不能靠"重复下发同一个 SET_TARGET"来续命：SET_TARGET 是无条件的
 * 副作用集合——它会 ++cmd_count、把 status 强制回 SLEW、并重新
 * schedule_delayed_work 从而把下一步推迟最多一个 slew_ms。以心跳频率
 * （本项目为 20 Hz）重复下发，会把 cmd_count 稀释成节拍计数器、
 * 让 status 在 SLEW/READY 之间抖动，并让爬升速率慢约 10%。
 *
 * 有了本 ioctl，上层就能明确表达"我还活着"与"目标变了"这两件不同的事：
 * 心跳发 KEEPALIVE，目标变化才发 SET_TARGET。于是看门狗仍然是
 * "上层进程死了就停下"的干净探测器，而命令计数与状态字段都保持准确。
 *
 * 未认领（UNINIT）时返回 EAGAIN，与其它会动作的 ioctl 一致。
 */
#define PANTILT_IOC_KEEPALIVE _IO(PANTILT_IOC_MAGIC, 0x06)

#endif /* __PANTILT_UAPI_H__ */
