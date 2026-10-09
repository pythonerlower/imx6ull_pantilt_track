// SPDX-License-Identifier: GPL-2.0
/*
 * IMX6ULL 安防云台正式控制接口 /dev/pantilt0
 *
 * 职责边界：本驱动只负责"把用户态算好的绝对目标 tick 安全地写进 PCA9685"——
 * 校验、钳位、限速渐进、看门狗和统计。目标定位、误差计算和控制律都在 PC 侧，
 * 驱动不参与（见 rules/imx6ull-pantilt-engineering.md §1）。
 *
 * 三条不可绕过的安全约束：
 *   1. insmod 不写任何寄存器；只有 SET_LIMIT 会唤醒芯片并爬回 home。
 *   2. 芯片处于未知状态时拒绝动作，不覆盖别的控制器正在写出的 PWM。
 *   3. 所有目标先钳位到 [safe_min, safe_max]，再与电气护栏取交集，逐 tick 逼近，
 *      绝不因为收到新目标就跳到该位置。
 *
 * 寄存器语义与用户态 pantilt_test 一致（MODE1=0x00、LED0_ON_L=0x06、PRE_SCALE=0xFE），
 * 差别只在启用 MODE1 的 AI 位：这样 CH0+CH1 的 8 字节脉宽可以在一次 I2C 事务内写完，
 * pantilt_test 里"跨 OFF 高低字节撕裂最坏差 256 tick"的问题在此不再存在。
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include "pantilt_uapi.h"

/* ---- PCA9685 寄存器 ---- */
#define PT_REG_MODE1 0x00
#define PT_REG_MODE2 0x01
#define PT_REG_LED0_ON_L 0x06
#define PT_REG_PRESCALE 0xfe

#define PT_MODE1_ALLCALL 0x01
#define PT_MODE1_SLEEP 0x10
#define PT_MODE1_AI 0x20
#define PT_MODE1_RESTART 0x80

#define PT_PRESCALE_50HZ 121 /* 25 MHz / (4096 * 50 Hz) - 1 */
#define PT_MODE2_OUTDRV 0x04 /* 推挽输出，与已实测的芯片状态一致 */

/*
 * 芯片缺省值：pt_probe_chip() 用它认上电默认态，
 * pantilt_exit() 用它写回一个自己下次一定认得的静止态。
 */
#define PT_PRESCALE_POWERON 0x1e
#define PT_MODE1_POWERON 0x11
/* 接受"上一轮留下的 50 Hz 运行态"时允许的预分频范围。 */
#define PT_PRESCALE_MIN 115
#define PT_PRESCALE_MAX 130

#define PT_DEFAULT_SLEW_MS 15 /* 与 pantilt_test 已验证的 CALIB_RAMP_MS 一致 */
#define PT_DEFAULT_WATCHDOG_MS 500
#define PT_MAX_SLEW_MS 100
#define PT_MAX_WATCHDOG_MS 60000
#define PT_MAX_TICK 4095

/*
 * 电气护栏，依据 rules/Hardware.md §2（2026-10-07 放宽后的值）。
 * 这是独立于配置文件的第二层保护：即使 pantilt_calibration.conf 写错，
 * 输出也不会越过这里的脉宽范围。要放宽必须先改 rules/Hardware.md，不得静默放宽。
 */
#define PT_GUARD_PAN_MIN 205
#define PT_GUARD_PAN_MAX 515
#define PT_GUARD_TILT_MIN 122
#define PT_GUARD_TILT_MAX 515

static const u16 pt_guard_min[PANTILT_AXES] = {
	PT_GUARD_PAN_MIN, PT_GUARD_TILT_MIN
};
static const u16 pt_guard_max[PANTILT_AXES] = {
	PT_GUARD_PAN_MAX, PT_GUARD_TILT_MAX
};

static const char *const pt_axis_name[PANTILT_AXES] = { "pan", "tilt" };

/* ---- 模块参数 ---- */
static int bus = 1;
module_param(bus, int, 0444);
MODULE_PARM_DESC(bus, "PCA9685 所在 I2C 总线号（默认 1）");

static int addr = 0x40;
module_param(addr, int, 0444);
MODULE_PARM_DESC(addr, "PCA9685 的 7 位 I2C 地址（默认 0x40）");

/*
 * 芯片处于未知状态时默认拒绝动作。若确认舵机姿态安全且确要接管，
 * 用 insmod pantilt.ko force_claim=1 显式覆盖——不要把它写进启动脚本。
 */
static bool force_claim;
module_param(force_claim, bool, 0444);
MODULE_PARM_DESC(force_claim, "允许接管处于未知状态的 PCA9685（默认关闭）");

struct pantilt_dev {
	struct i2c_adapter *adap;
	struct i2c_client *client;
	struct miscdevice miscdev;
	struct mutex lock; /* 保护下面所有可变字段与 I2C 访问 */
	struct delayed_work slew_work;

	unsigned long opened; /* bit0：已被某个进程打开 */
	bool shutting_down;

	/*
	 * insmod 时读到的 MODE1。唤醒时以它为基准组出运行态的 MODE1
	 * （保留 ALLCALL 这类他人设置的位）。
	 * 退出归还**不再**依赖任何快照，原因见 pantilt_exit()。
	 */
	u8 mode1;
	u32 chip_state; /* PANTILT_CHIP_* */

	bool initialized; /* SET_LIMIT 成功后才为真 */
	bool slew_from_cmd; /* 本次爬升是否由 SET_TARGET 发起（决定看门狗是否适用） */
	u32 status; /* PANTILT_ST_* */

	u16 cur_tick[PANTILT_AXES];
	u16 tgt_tick[PANTILT_AXES];
	u16 home[PANTILT_AXES];
	u16 safe_min[PANTILT_AXES];
	u16 safe_max[PANTILT_AXES];
	u32 slew_ms;
	u32 watchdog_ms;

	unsigned long last_cmd_jiffies;
	ktime_t cmd_ktime;
	bool latency_pending;

	/* 统计 */
	u64 cmd_count;
	u64 clamp_low[PANTILT_AXES];
	u64 clamp_high[PANTILT_AXES];
	u64 i2c_err;
	u64 watchdog_trips;
	u64 last_update_us;
	u64 last_latency_us;

	struct dentry *dbg_dir;
};

static struct pantilt_dev *g_dev;

/* ---------------- I2C ---------------- */

static int pt_write_reg(struct pantilt_dev *d, u8 reg, u8 value)
{
	u8 buf[2] = { reg, value };
	struct i2c_msg msg = {
		.addr = d->client->addr, .flags = 0, .len = sizeof(buf),
		.buf = buf,
	};
	int ret = i2c_transfer(d->client->adapter, &msg, 1);

	if (ret == 1)
		return 0;
	d->i2c_err++;
	return ret < 0 ? ret : -EIO;
}

static int pt_read_reg(struct pantilt_dev *d, u8 reg, u8 *value)
{
	struct i2c_msg msg[2] = {
		{ .addr = d->client->addr, .flags = 0, .len = 1, .buf = &reg },
		{ .addr = d->client->addr, .flags = I2C_M_RD, .len = 1,
		  .buf = value },
	};
	int ret = i2c_transfer(d->client->adapter, msg, 2);

	if (ret == 2)
		return 0;
	d->i2c_err++;
	return ret < 0 ? ret : -EIO;
}

/*
 * 一次事务写完两个通道的脉宽（LED0_ON_L 起连续 8 字节，依赖 MODE1 的 AI 位）。
 * ON 固定为 0，OFF 即脉宽；OFF_H 的 FULL 位（bit4）保持 0。
 */
static int pt_write_pulses(struct pantilt_dev *d, const u16 tick[PANTILT_AXES])
{
	struct i2c_msg msg;
	u8 buf[1 + PANTILT_AXES * 4];
	int i, ret;

	buf[0] = PT_REG_LED0_ON_L;
	for (i = 0; i < PANTILT_AXES; i++) {
		u16 off = tick[i] & 0x0fff;

		buf[1 + i * 4 + 0] = 0;
		buf[1 + i * 4 + 1] = 0;
		buf[1 + i * 4 + 2] = off & 0xff;
		buf[1 + i * 4 + 3] = (off >> 8) & 0x0f;
	}

	msg.addr = d->client->addr;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;

	ret = i2c_transfer(d->client->adapter, &msg, 1);
	if (ret == 1)
		return 0;
	d->i2c_err++;
	return ret < 0 ? ret : -EIO;
}

/* ---------------- 爬升 ---------------- */

static u16 pt_step_toward(u16 cur, u16 dst)
{
	if (cur < dst)
		return cur + 1;
	if (cur > dst)
		return cur - 1;
	return cur;
}

static void pt_arm_slew(struct pantilt_dev *d, bool from_cmd)
{
	d->status = PANTILT_ST_SLEW;
	d->slew_from_cmd = from_cmd;
	d->last_cmd_jiffies = jiffies;
	schedule_delayed_work(&d->slew_work, msecs_to_jiffies(d->slew_ms));
}

/*
 * 逐 tick 逼近 target。每一拍最多走 1 tick，因此收到新目标时的位移上限
 * 就是 1 tick（约 0.44°），不会出现"起步突然大幅运动"。
 */
static void pt_slew_work(struct work_struct *work)
{
	struct pantilt_dev *d = container_of(to_delayed_work(work),
					     struct pantilt_dev, slew_work);
	bool more = false, moved = false;
	u16 next[PANTILT_AXES];
	int i, ret;

	mutex_lock(&d->lock);

	if (d->shutting_down || !d->initialized)
		goto out;

	/*
	 * 看门狗只针对外部命令发起的爬升：PC 断流时停止动作并保持位置。
	 * 由 HOME / SET_LIMIT 发起的爬升是我们自己的有界动作，不受它影响。
	 */
	if (d->status == PANTILT_ST_SLEW && d->slew_from_cmd &&
	    d->watchdog_ms &&
	    time_after(jiffies, d->last_cmd_jiffies +
					msecs_to_jiffies(d->watchdog_ms))) {
		d->status = PANTILT_ST_HOLD;
		d->watchdog_trips++;
		dev_warn_ratelimited(&d->client->dev,
				     "pantilt: no command for %u ms, aborting slew and entering HOLD\n",
				     d->watchdog_ms);
		goto out;
	}

	if (d->status != PANTILT_ST_SLEW)
		goto out;

	for (i = 0; i < PANTILT_AXES; i++) {
		next[i] = pt_step_toward(d->cur_tick[i], d->tgt_tick[i]);
		if (next[i] != d->cur_tick[i])
			moved = true;
		if (next[i] != d->tgt_tick[i])
			more = true;
	}

	if (moved) {
		ret = pt_write_pulses(d, next);
		if (ret) {
			dev_err_ratelimited(&d->client->dev,
					    "pantilt: pulse write failed %d, entering HOLD\n",
					    ret);
			d->status = PANTILT_ST_HOLD;
			goto out;
		}
		for (i = 0; i < PANTILT_AXES; i++)
			d->cur_tick[i] = next[i];
		d->last_update_us = div_u64(ktime_to_ns(ktime_get()), 1000);
		if (d->latency_pending) {
			d->last_latency_us = div_u64(
				ktime_to_ns(ktime_sub(ktime_get(), d->cmd_ktime)),
				1000);
			d->latency_pending = false;
		}
	}

	if (more)
		schedule_delayed_work(&d->slew_work,
				      msecs_to_jiffies(d->slew_ms));
	else
		d->status = PANTILT_ST_READY;

out:
	mutex_unlock(&d->lock);
}

/* ---------------- 芯片探测与认领 ---------------- */

/*
 * 两通道是否都处于「输出常低」：ON=0 且 OFF_H 的 FULL 位（bit4）置位。
 * 这是「芯片没有在输出脉宽」的直接判据，比比对 MODE1/PRE_SCALE 的确切字节
 * 更贴近真正要保证的性质。两通道必须一起查：只查通道 0 的话，通道 1 仍在
 * 输出脉宽的芯片会被误判成静止。
 */
static int pt_channels_off(struct pantilt_dev *d, bool *off)
{
	u8 regs[PANTILT_AXES * 4];
	int i;

	for (i = 0; i < PANTILT_AXES * 4; i++) {
		if (pt_read_reg(d, PT_REG_LED0_ON_L + i, &regs[i]))
			return -EIO;
	}

	*off = true;
	for (i = 0; i < PANTILT_AXES; i++) {
		const u8 *r = &regs[i * 4];

		if (r[0] || r[1] || !(r[3] & 0x10)) {
			*off = false;
			break;
		}
	}
	return 0;
}

/*
 * 只读探测。接受两种已知态：
 *   ASLEEP  —— 静止态（没有 PWM 在驱动舵机），可以直接认领；
 *   RUNNING —— 本项目上一轮留下的 50 Hz 运行态，可读到有效脉宽。
 * 其余一律 UNKNOWN，拒绝动作。板子重启而 PCA9685 的 V+ 未断电时会出现这种情况：
 * 芯片仍在 50 Hz 输出，此时必须目视确认姿态后显式 force_claim。
 */
static u32 pt_probe_chip(struct pantilt_dev *d)
{
	u8 mode1, mode2, prescale;

	if (pt_read_reg(d, PT_REG_MODE1, &mode1) ||
	    pt_read_reg(d, PT_REG_MODE2, &mode2) ||
	    pt_read_reg(d, PT_REG_PRESCALE, &prescale)) {
		dev_err(&d->client->dev, "pantilt: failed to read PCA9685 registers\n");
		return PANTILT_CHIP_UNKNOWN;
	}
	d->mode1 = mode1;

	/*
	 * 静止态判据只看「没有 PWM 在驱动舵机」这一件事：MODE2 正确、
	 * 两通道都关、预分频属于本工程认识的取值。**不**比对 MODE1/PRE_SCALE
	 * 的确切字节。理由：
	 *
	 *   1) 旧写法要求 PRE_SCALE 恰为上电默认值 0x1e，只检查通道 0。
	 *      而 pantilt_exit() 把 PRE_SCALE 还原成“进模块时读到的值”，于是
	 *      「模块加载到一颗遗留运行态的芯片（断电重启后就是这种局面）→ 认领
	 *      → rmmod」会归还出 PRE_SCALE=121 + SLEEP 的组合：它既不是 ASLEEP
	 *      （预分频对不上）也不是 RUNNING（SLEEP 置位），判成 UNKNOWN，
	 *      于是模块再也插不进来，只能靠 force_claim 带人眼兜底。
	 *   2) 那个组合实际上比上电默认态**更安全**——它根本没有 PWM 在输出，
	 *      旧写法却拒收，等于把最安全的状态挡在门外。
	 *
	 * 「SLEEP 位置位」不再是必要条件：RUNNING 但通道全关的芯片同样没有
	 * 脉宽，认领时按静止态处理（那里本来也无脉宽可回读）。
	 */
	if (mode2 == PT_MODE2_OUTDRV &&
	    (prescale == PT_PRESCALE_POWERON ||
	     (prescale >= PT_PRESCALE_MIN && prescale <= PT_PRESCALE_MAX))) {
		bool off;

		if (pt_channels_off(d, &off))
			return PANTILT_CHIP_UNKNOWN;
		if (off)
			return PANTILT_CHIP_ASLEEP;
	}

	if (!(mode1 & PT_MODE1_SLEEP) && mode2 == PT_MODE2_OUTDRV &&
	    prescale >= PT_PRESCALE_MIN && prescale <= PT_PRESCALE_MAX)
		return PANTILT_CHIP_RUNNING;

	dev_warn(&d->client->dev,
		 "pantilt: PCA9685 in unknown state MODE1=0x%02x MODE2=0x%02x PRE_SCALE=0x%02x\n",
		 mode1, mode2, prescale);
	return PANTILT_CHIP_UNKNOWN;
}

static int pt_read_current(struct pantilt_dev *d, u16 tick[PANTILT_AXES])
{
	int i;

	for (i = 0; i < PANTILT_AXES; i++) {
		u8 regs[4];
		int j;

		for (j = 0; j < 4; j++) {
			if (pt_read_reg(d, PT_REG_LED0_ON_L + i * 4 + j,
					&regs[j]))
				return -EIO;
		}
		/* 只接受 ON=0 的普通舵机脉冲。 */
		if (regs[0] || regs[1] || (regs[3] & 0x10)) {
			dev_err(&d->client->dev,
				"pantilt: %s channel is not a plain ON=0 servo pulse, refusing to claim\n",
				pt_axis_name[i]);
			return -EINVAL;
		}
		tick[i] = regs[2] | ((u16)(regs[3] & 0x0f) << 8);
	}
	return 0;
}

/*
 * 唤醒芯片并预置脉宽。顺序照搬 pantilt_test 已验证的流程：
 * 先改预分频（只能 SLEEP 下改）→ 先写脉宽再唤醒（避免启动瞬间出现未定义脉冲）
 * → 退出 SLEEP → 若 RESTART 置位则等振荡器稳定后写 1 清除 → 回读确认。
 */
static int pt_wake(struct pantilt_dev *d, const u16 start[PANTILT_AXES])
{
	u8 mode1, value;
	int ret;

	ret = pt_write_reg(d, PT_REG_PRESCALE, PT_PRESCALE_50HZ);
	if (ret)
		return ret;

	ret = pt_write_reg(d, PT_REG_MODE2, PT_MODE2_OUTDRV);
	if (ret)
		return ret;

	ret = pt_write_pulses(d, start);
	if (ret)
		return ret;

	/* 打开 AI：多字节脉宽写入才能一次完成。 */
	mode1 = (d->mode1 & ~(PT_MODE1_SLEEP | PT_MODE1_RESTART)) | PT_MODE1_AI;
	ret = pt_write_reg(d, PT_REG_MODE1, mode1);
	if (ret)
		return ret;

	if (d->mode1 & PT_MODE1_RESTART) {
		msleep(2);
		ret = pt_write_reg(d, PT_REG_MODE1,
				   (d->mode1 & ~PT_MODE1_SLEEP) |
					   PT_MODE1_RESTART | PT_MODE1_AI);
		if (ret)
			return ret;
	}

	msleep(5);
	if (pt_read_reg(d, PT_REG_MODE1, &value))
		return -EIO;
	if (value & PT_MODE1_SLEEP) {
		dev_err(&d->client->dev, "pantilt: oscillator did not start (MODE1=0x%02x)\n",
			value);
		return -EIO;
	}

	/*
	 * 回读已确认 SLEEP 清掉，芯片在输出 50 Hz。必须同步更新 chip_state：
	 * 该字段只在 insmod 的 pt_probe_chip() 里写过一次，唤醒后不改的话
	 * 会一直显示 ASLEEP，而实际上舵机已带电有保持力矩——这正好是排查
	 * 「舵机为什么有劲」时最容易把人带偏的地方。
	 * 探测当时的原始状态仍保留在认领时那条 dev_info 里。
	 */
	d->chip_state = PANTILT_CHIP_RUNNING;
	return 0;
}

/* ---------------- 文件操作 ---------------- */

static int pt_validate_limit(const struct pantilt_limit *lim)
{
	int i;

	if (lim->slew_ms < 1 || lim->slew_ms > PT_MAX_SLEW_MS)
		return -EINVAL;
	if (lim->watchdog_ms > PT_MAX_WATCHDOG_MS)
		return -EINVAL;

	for (i = 0; i < PANTILT_AXES; i++) {
		if (lim->safe_min[i] > lim->safe_max[i])
			return -EINVAL;
		if (lim->safe_min[i] < pt_guard_min[i] ||
		    lim->safe_max[i] > pt_guard_max[i])
			return -EINVAL;
		if (lim->home[i] < lim->safe_min[i] ||
		    lim->home[i] > lim->safe_max[i])
			return -EINVAL;
		if (lim->start[i] < pt_guard_min[i] ||
		    lim->start[i] > pt_guard_max[i])
			return -EINVAL;
	}
	return 0;
}

static long pt_ioctl_set_limit(struct pantilt_dev *d, unsigned long arg)
{
	struct pantilt_limit lim;
	int i, ret;

	if (copy_from_user(&lim, (void __user *)arg, sizeof(lim)))
		return -EFAULT;

	/* 非法值一律拒绝，不静默钳位——与 pantilt_test 对配置文件的处理一致。 */
	if (pt_validate_limit(&lim)) {
		dev_err(&d->client->dev,
			"pantilt: bad SET_LIMIT arguments, rejected (need min<=home<=max, inside electrical guard, slew 1..%d)\n",
			PT_MAX_SLEW_MS);
		return -EINVAL;
	}

	mutex_lock(&d->lock);
	if (d->initialized) {
		mutex_unlock(&d->lock);
		return -EBUSY; /* 限位只在启动时设定；运行中改限位需要重新加载 */
	}

	for (i = 0; i < PANTILT_AXES; i++) {
		d->home[i] = lim.home[i];
		d->safe_min[i] = lim.safe_min[i];
		d->safe_max[i] = lim.safe_max[i];
	}
	d->slew_ms = lim.slew_ms;
	d->watchdog_ms = lim.watchdog_ms;

	if (d->chip_state == PANTILT_CHIP_RUNNING) {
		/*
		 * 芯片已在运行：以回读的脉宽为准，start[] 只是参考。
		 * 这里只补上 AI 位，不动 SLEEP / 预分频，输出不中断。
		 * 上一轮可能是 pantilt_test 留下的（AI 位关闭），所以这一步是必要的。
		 */
		ret = pt_read_current(d, d->cur_tick);
		if (ret)
			goto unlock;
		ret = pt_write_reg(d, PT_REG_MODE1,
				   (d->mode1 & ~PT_MODE1_SLEEP) | PT_MODE1_AI);
		if (ret)
			goto unlock;
		for (i = 0; i < PANTILT_AXES; i++) {
			if (d->cur_tick[i] != lim.start[i])
				dev_warn(&d->client->dev,
					 "pantilt: %s already running, read back %u tick, ignoring start %u\n",
					 pt_axis_name[i], d->cur_tick[i],
					 lim.start[i]);
		}
	} else {
		for (i = 0; i < PANTILT_AXES; i++) {
			if (lim.start[i] != lim.home[i])
				dev_warn(&d->client->dev,
					 "pantilt: %s waking at %u tick which differs from home %u; ramping back (this step may jump)\n",
					 pt_axis_name[i], lim.start[i],
					 lim.home[i]);
			d->cur_tick[i] = lim.start[i];
		}
		/* 唤醒会真正驱动舵机，是本驱动唯一"一调用就动"的位置。 */
		ret = pt_wake(d, lim.start);
		if (ret)
			goto unlock;
	}

	d->initialized = true;
	memcpy(d->tgt_tick, d->home, sizeof(d->tgt_tick));
	dev_info(&d->client->dev,
		 "pantilt: claimed (chip=%u), ramping home at %u ms/tick pan=%u tilt=%u\n",
		 d->chip_state, d->slew_ms, d->home[PANTILT_AXIS_PAN],
		 d->home[PANTILT_AXIS_TILT]);
	pt_arm_slew(d, false);
	ret = 0;

unlock:
	if (ret && d->chip_state == PANTILT_CHIP_ASLEEP) {
		/* 唤醒中途失败：回到休眠，不要留一个半初始化、正在输出的芯片。 */
		pt_write_reg(d, PT_REG_MODE1,
			     (d->mode1 & ~PT_MODE1_RESTART) | PT_MODE1_SLEEP);
		d->initialized = false;
	}
	mutex_unlock(&d->lock);
	return ret;
}

static long pt_ioctl_set_target(struct pantilt_dev *d, unsigned long arg)
{
	struct pantilt_target tgt;
	bool clamped = false;
	int i;

	if (copy_from_user(&tgt, (void __user *)arg, sizeof(tgt)))
		return -EFAULT;

	mutex_lock(&d->lock);
	if (!d->initialized) {
		mutex_unlock(&d->lock);
		return -EAGAIN; /* 必须先 SET_LIMIT 完成认领 */
	}

	for (i = 0; i < PANTILT_AXES; i++) {
		u16 want = tgt.tick[i];

		if (want > PT_MAX_TICK) {
			want = PT_MAX_TICK;
			clamped = true;
		}
		if (want < d->safe_min[i]) {
			d->clamp_low[i]++;
			want = d->safe_min[i];
			clamped = true;
		} else if (want > d->safe_max[i]) {
			d->clamp_high[i]++;
			want = d->safe_max[i];
			clamped = true;
		}
		d->tgt_tick[i] = want;
	}

	d->cmd_count++;
	d->cmd_ktime = ktime_get();
	d->latency_pending = true;
	d->last_cmd_jiffies = jiffies;
	/* SET_TARGET 是解除 HOLD 的显式动作；看门狗触发后必须由用户态再次下发才恢复。 */
	d->status = PANTILT_ST_SLEW;
	d->slew_from_cmd = true;
	schedule_delayed_work(&d->slew_work, msecs_to_jiffies(d->slew_ms));
	mutex_unlock(&d->lock);

	if (clamped)
		dev_warn_ratelimited(&d->client->dev,
				     "pantilt: target out of range, clamped to safe limits\n");
	return 0;
}

/*
 * 只续看门狗的命。刻意不碰 status / tgt_tick / 定时器——正因为它什么都不做，
 * 上层才能以心跳频率调用它而不污染 cmd_count（见 uapi 里的说明）。
 */
static long pt_ioctl_keepalive(struct pantilt_dev *d)
{
	mutex_lock(&d->lock);
	if (!d->initialized) {
		mutex_unlock(&d->lock);
		return -EAGAIN;
	}
	d->last_cmd_jiffies = jiffies;
	mutex_unlock(&d->lock);
	return 0;
}

static long pt_ioctl_home(struct pantilt_dev *d)
{
	mutex_lock(&d->lock);
	if (!d->initialized) {
		mutex_unlock(&d->lock);
		return -EAGAIN;
	}
	memcpy(d->tgt_tick, d->home, sizeof(d->tgt_tick));
	pt_arm_slew(d, false);
	mutex_unlock(&d->lock);
	return 0;
}

static long pt_ioctl_stop(struct pantilt_dev *d)
{
	mutex_lock(&d->lock);
	if (!d->initialized) {
		mutex_unlock(&d->lock);
		return -EAGAIN;
	}
	/*
	 * 立即停止爬升并保持当前位置：不改变已写出的脉宽，
	 * PCA9685 会持续输出，舵机保持在原处，不会因失力而掉落。
	 */
	memcpy(d->tgt_tick, d->cur_tick, sizeof(d->tgt_tick));
	d->status = PANTILT_ST_HOLD;
	d->slew_from_cmd = false;
	mutex_unlock(&d->lock);
	return 0;
}

static long pt_unlocked_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg)
{
	struct pantilt_dev *d = filp->private_data;

	if (_IOC_TYPE(cmd) != PANTILT_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case PANTILT_IOC_SET_LIMIT:
		return pt_ioctl_set_limit(d, arg);
	case PANTILT_IOC_SET_TARGET:
		return pt_ioctl_set_target(d, arg);
	case PANTILT_IOC_GET_STATE: {
		struct pantilt_state st;

		mutex_lock(&d->lock);
		memset(&st, 0, sizeof(st));
		st.status = d->status;
		st.chip_state = d->chip_state;
		st.slew_ms = d->slew_ms;
		st.watchdog_ms = d->watchdog_ms;
		memcpy(st.cur_tick, d->cur_tick, sizeof(st.cur_tick));
		memcpy(st.tgt_tick, d->tgt_tick, sizeof(st.tgt_tick));
		memcpy(st.home, d->home, sizeof(st.home));
		memcpy(st.safe_min, d->safe_min, sizeof(st.safe_min));
		memcpy(st.safe_max, d->safe_max, sizeof(st.safe_max));
		st.cmd_count = d->cmd_count;
		st.i2c_err = d->i2c_err;
		st.watchdog_trips = d->watchdog_trips;
		st.last_update_us = d->last_update_us;
		st.last_latency_us = d->last_latency_us;
		memcpy(st.clamp_low, d->clamp_low, sizeof(st.clamp_low));
		memcpy(st.clamp_high, d->clamp_high, sizeof(st.clamp_high));
		mutex_unlock(&d->lock);

		if (copy_to_user((void __user *)arg, &st, sizeof(st)))
			return -EFAULT;
		return 0;
	}
	case PANTILT_IOC_HOME:
		return pt_ioctl_home(d);
	case PANTILT_IOC_KEEPALIVE:
		return pt_ioctl_keepalive(d);
	case PANTILT_IOC_STOP:
		return pt_ioctl_stop(d);
	default:
		return -ENOTTY;
	}
}

static int pt_open(struct inode *inode, struct file *filp)
{
	struct pantilt_dev *d = g_dev;

	if (!d)
		return -ENODEV;
	/* 同一时刻只允许一个控制器，避免两方争抢同一路 PWM。 */
	if (test_and_set_bit(0, &d->opened))
		return -EBUSY;
	filp->private_data = d;
	return 0;
}

static int pt_release(struct inode *inode, struct file *filp)
{
	struct pantilt_dev *d = filp->private_data;

	if (d)
		clear_bit(0, &d->opened);
	return 0;
}

static const struct file_operations pt_fops = {
	.owner = THIS_MODULE,
	.open = pt_open,
	.release = pt_release,
	.unlocked_ioctl = pt_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pt_unlocked_ioctl,
#endif
	.llseek = no_llseek,
};

/* ---------------- debugfs ---------------- */

static const char *pt_status_name(u32 status)
{
	switch (status) {
	case PANTILT_ST_UNINIT:
		return "UNINIT";
	case PANTILT_ST_READY:
		return "READY";
	case PANTILT_ST_SLEW:
		return "SLEW";
	case PANTILT_ST_HOLD:
		return "HOLD";
	default:
		return "?";
	}
}

static const char *pt_chip_name(u32 chip)
{
	switch (chip) {
	case PANTILT_CHIP_ASLEEP:
		return "ASLEEP";
	case PANTILT_CHIP_RUNNING:
		return "RUNNING";
	default:
		return "UNKNOWN";
	}
}

static int pt_dbg_state_show(struct seq_file *s, void *unused)
{
	struct pantilt_dev *d = s->private;

	mutex_lock(&d->lock);
	seq_printf(s, "status      %s\n", pt_status_name(d->status));
	seq_printf(s, "chip_state  %s\n", pt_chip_name(d->chip_state));
	seq_printf(s, "pan         current=%u target=%u home=%u\n", d->cur_tick[0],
		   d->tgt_tick[0], d->home[0]);
	seq_printf(s, "tilt        current=%u target=%u home=%u\n", d->cur_tick[1],
		   d->tgt_tick[1], d->home[1]);
	mutex_unlock(&d->lock);
	return 0;
}

static int pt_dbg_limit_show(struct seq_file *s, void *unused)
{
	struct pantilt_dev *d = s->private;

	mutex_lock(&d->lock);
	seq_printf(s, "pan   safe %u..%u  electrical guard %u..%u\n",
		   d->safe_min[0], d->safe_max[0], pt_guard_min[0],
		   pt_guard_max[0]);
	seq_printf(s, "tilt  safe %u..%u  electrical guard %u..%u\n",
		   d->safe_min[1], d->safe_max[1], pt_guard_min[1],
		   pt_guard_max[1]);
	seq_printf(s, "slew_ms %u  watchdog_ms %u\n", d->slew_ms, d->watchdog_ms);
	mutex_unlock(&d->lock);
	return 0;
}

static int pt_dbg_stats_show(struct seq_file *s, void *unused)
{
	struct pantilt_dev *d = s->private;

	mutex_lock(&d->lock);
	seq_printf(s, "cmd_count        %llu\n", d->cmd_count);
	seq_printf(s, "clamp_low        pan=%llu tilt=%llu\n", d->clamp_low[0],
		   d->clamp_low[1]);
	seq_printf(s, "clamp_high       pan=%llu tilt=%llu\n", d->clamp_high[0],
		   d->clamp_high[1]);
	seq_printf(s, "i2c_err          %llu\n", d->i2c_err);
	seq_printf(s, "watchdog_trips   %llu\n", d->watchdog_trips);
	seq_printf(s, "last_update_us   %llu\n", d->last_update_us);
	seq_printf(s, "last_latency_us  %llu\n", d->last_latency_us);
	mutex_unlock(&d->lock);
	return 0;
}

#define PT_DBG_FOPS(name)                                     \
	static int name##_open(struct inode *inode,           \
			       struct file *filp)             \
	{                                                     \
		return single_open(filp, name##_show,         \
				   inode->i_private);         \
	}                                                     \
	static const struct file_operations name##_fops = {   \
		.owner = THIS_MODULE,                         \
		.open = name##_open,                          \
		.read = seq_read,                             \
		.llseek = seq_lseek,                          \
		.release = single_release,                    \
	}

PT_DBG_FOPS(pt_dbg_state);
PT_DBG_FOPS(pt_dbg_limit);
PT_DBG_FOPS(pt_dbg_stats);

static int pt_debugfs_init(struct pantilt_dev *d)
{
	d->dbg_dir = debugfs_create_dir("pantilt", NULL);
	if (!d->dbg_dir)
		return -ENOMEM;
	debugfs_create_file("state", 0444, d->dbg_dir, d, &pt_dbg_state_fops);
	debugfs_create_file("limit", 0444, d->dbg_dir, d, &pt_dbg_limit_fops);
	debugfs_create_file("stats", 0444, d->dbg_dir, d, &pt_dbg_stats_fops);
	return 0;
}

/* ---------------- 模块装载 ---------------- */

static int __init pantilt_init(void)
{
	struct pantilt_dev *d;
	struct i2c_board_info info = { };
	int ret;

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	mutex_init(&d->lock);
	INIT_DELAYED_WORK(&d->slew_work, pt_slew_work);
	d->status = PANTILT_ST_UNINIT;
	d->slew_ms = PT_DEFAULT_SLEW_MS;
	d->watchdog_ms = PT_DEFAULT_WATCHDOG_MS;

	/* 只取适配器，不通过 DTS 实例化：PCA9685 是外接模块，板级 DTS 里没有它。 */
	d->adap = i2c_get_adapter(bus);
	if (!d->adap) {
		pr_err("pantilt: I2C bus %d not found\n", bus);
		ret = -ENODEV;
		goto err_free;
	}

	strlcpy(info.type, "pantilt-pca9685", sizeof(info.type));
	info.addr = addr;
	d->client = i2c_new_device(d->adap, &info);
	if (!d->client) {
		pr_err("pantilt: cannot register device at %d-%04x (address may be taken)\n",
		       bus, addr);
		ret = -ENODEV;
		goto err_adap;
	}

	/*
	 * 只读探测，不写任何寄存器——因此 insmod 永远安全。
	 * 真正的唤醒推迟到 SET_LIMIT，那时才有用户态给的起始脉宽可用。
	 */
	d->chip_state = pt_probe_chip(d);
	if (d->chip_state == PANTILT_CHIP_UNKNOWN && !force_claim) {
		pr_err("pantilt: PCA9685 state unknown, refusing to claim; insmod with force_claim=1 after visually confirming the servos are safe\n");
		ret = -EBUSY;
		goto err_client;
	}

	d->miscdev.minor = MISC_DYNAMIC_MINOR;
	d->miscdev.name = "pantilt0";
	d->miscdev.fops = &pt_fops;
	d->miscdev.mode = 0600;
	ret = misc_register(&d->miscdev);
	if (ret) {
		pr_err("pantilt: misc_register failed %d\n", ret);
		goto err_client;
	}

	if (pt_debugfs_init(d))
		pr_warn("pantilt: debugfs setup failed, functionality unaffected\n");

	g_dev = d;
	pr_info("pantilt: %s ready, chip=%s, waiting for SET_LIMIT to claim (no servo output yet)\n",
		d->miscdev.name, pt_chip_name(d->chip_state));
	return 0;

err_client:
	i2c_unregister_device(d->client);
err_adap:
	i2c_put_adapter(d->adap);
err_free:
	kfree(d);
	return ret;
}

static void __exit pantilt_exit(void)
{
	struct pantilt_dev *d = g_dev;

	if (!d)
		return;
	g_dev = NULL;

	mutex_lock(&d->lock);
	d->shutting_down = true;
	mutex_unlock(&d->lock);
	cancel_delayed_work_sync(&d->slew_work);

	/*
	 * 有序归还，顺序与 pantilt_test 的回收路径一致：
	 * 先关闭两个通道（ON=0、OFF_H 置 FULL 位），再休眠，最后写回上电默认值。
	 * 归还后舵机失力，tilt 可能被重力带走——这与既有 pantilt_test 的退出行为相同，
	 * 也是 rules/Hardware.md §1.2「上电后不得假设舵机一定处于 home」的由来。
	 */
	if (d->initialized) {
		u8 buf[1 + PANTILT_AXES * 4];
		struct i2c_msg msg = { .addr = d->client->addr,
				       .flags = 0,
				       .len = sizeof(buf),
				       .buf = buf };
		int i;

		buf[0] = PT_REG_LED0_ON_L;
		for (i = 0; i < PANTILT_AXES; i++) {
			buf[1 + i * 4 + 0] = 0;
			buf[1 + i * 4 + 1] = 0;
			buf[1 + i * 4 + 2] = 0;
			buf[1 + i * 4 + 3] = 0x10; /* FULL：输出常低，通道关闭 */
		}
		i2c_transfer(d->client->adapter, &msg, 1);

		/*
		 * 先休眠（PRE_SCALE 只能在 SLEEP 下改），再写回**上电默认值**，
		 * 而不是写回“进模块时读到的快照”。
		 *
		 * 归还的目标是「芯片处于一个没有 PWM、且自己下次 insmod 一定认得
		 * 的状态」。快照式还原做不到这一点：若模块被加载到一颗遗留运行态的
		 * 芯片上（断电重启后就是这种局面），快照里 MODE1 带 AI、PRE_SCALE
		 * 是 121，写回去得到 SLEEP+AI+121 —— pt_probe_chip() 判它 UNKNOWN，
		 * 模块再也插不进来，只能 force_claim 带人眼兜底。写默认值才让
		 * insmod → 认领 → rmmod → insmod 成为闭环。
		 *
		 * 置 SLEEP 后芯片会自行把 RESTART 位重新拉高（表示“输出已关闭、
		 * 写 1 可恢复”），所以回读是 0x91 而不是 0x11，这是正确结果。
		 */
		pt_write_reg(d, PT_REG_MODE1, PT_MODE1_POWERON);
		pt_write_reg(d, PT_REG_PRESCALE, PT_PRESCALE_POWERON);
	}

	debugfs_remove_recursive(d->dbg_dir);
	misc_deregister(&d->miscdev);
	i2c_unregister_device(d->client);
	i2c_put_adapter(d->adap);
	kfree(d);
}

module_init(pantilt_init);
module_exit(pantilt_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("imx6ull project");
MODULE_DESCRIPTION("IMX6ULL pan-tilt servo control via PCA9685 (/dev/pantilt0)");
