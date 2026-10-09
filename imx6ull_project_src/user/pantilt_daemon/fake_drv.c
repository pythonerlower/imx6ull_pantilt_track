/*
 * 模拟驱动：在内存里复刻真驱动对 daemon 有意义的那部分行为——
 * 钳位、逐 tick 爬升、以及"上层断流就停下"的看门狗。
 *
 * 它存在的唯一理由是让 daemon.c 的协议校验与状态机能被自动测试，
 * 见 test_plane.py。**它不模拟任何电气/机械特性**，因此不能替代板上验收：
 * 时序、jitter、I2C 错误、舵机实际动作都不在这里体现。
 *
 * 极限值与 config/pantilt_calibration.conf 保持一致，改配置时这里要同步。
 */
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "drv.h"

/* 与 config/pantilt_calibration.conf 一致 */
#define FAKE_PAN_HOME 377
#define FAKE_TILT_HOME 242
#define FAKE_PAN_MIN 239
#define FAKE_PAN_MAX 515
#define FAKE_TILT_MIN 162
#define FAKE_TILT_MAX 342
#define FAKE_SLEW_MS 20 /* 与 daemon 的 --slew 默认值对应 */
#define FAKE_WD_MS 1500

struct fake {
	uint16_t home[PANTILT_AXES];
	uint16_t safe_min[PANTILT_AXES];
	uint16_t safe_max[PANTILT_AXES];
	uint16_t cur[PANTILT_AXES];
	uint16_t tgt[PANTILT_AXES];

	uint64_t cmd_count;
	uint64_t clamp_low[PANTILT_AXES];
	uint64_t clamp_high[PANTILT_AXES];
	uint64_t i2c_err;
	uint64_t watchdog_trips;
	uint64_t last_update_us;
	uint64_t last_latency_us;

	uint32_t status;
	uint32_t chip_state;
	uint32_t slew_ms;
	uint32_t watchdog_ms;

	bool initialized;
	bool slew_from_cmd;
	uint64_t last_cmd_ms;
	uint64_t last_step_ms;
	uint64_t cmd_ms;
	bool latency_pending;
};

static uint16_t step_toward(uint16_t cur, uint16_t dst)
{
	if (cur < dst)
		return (uint16_t)(cur + 1);
	if (cur > dst)
		return (uint16_t)(cur - 1);
	return cur;
}

/*
 * 推进爬升与看门狗。调用频率就是 daemon 的节拍频率，
 * 因此 fake 的"一步"与真驱动的 10 ms jiffy 一样粗糙——
 * 这正好用于暴露"依赖精确计时"的错误假设。
 */
static void fake_tick(struct drv *d)
{
	struct fake *f = d->ctx;
	uint64_t now_ms = d->now_ms;
	bool more = false;
	int i;

	if (!f->initialized)
		return;

	if (f->status == PANTILT_ST_SLEW && f->slew_from_cmd && f->watchdog_ms &&
	    now_ms > f->last_cmd_ms + f->watchdog_ms) {
		f->status = PANTILT_ST_HOLD;
		f->watchdog_trips++;
		return;
	}
	if (f->status != PANTILT_ST_SLEW)
		return;

	/*
	 * 按 slew_ms 的**速率**推进，而不是每个节拍只走一步。
	 * 后者会让爬升速度被节拍频率绑架：tick-hz 20 时 slew_ms=20 的驱动
	 * 会退化成 50 ms/tick，于是任何"长距离移动要花多久"的判断都是错的，
	 * 本次要验的"续命能让长移动走完"也就失去意义。
	 * 用 while 把落后的步数一次补齐，同时给 64 步封顶防止时钟跳变时卡死。
	 */
	{
		unsigned steps = 0;

		while (now_ms >= f->last_step_ms + f->slew_ms &&
		       steps++ < 64) {
			f->last_step_ms += f->slew_ms;
			more = false;
			for (i = 0; i < PANTILT_AXES; i++) {
				uint16_t next = step_toward(f->cur[i], f->tgt[i]);

				if (next != f->cur[i]) {
					f->cur[i] = next;
					f->last_update_us = now_ms * 1000;
					if (f->latency_pending) {
						f->last_latency_us =
							(now_ms - f->cmd_ms) * 1000;
						f->latency_pending = false;
					}
				}
				if (next != f->tgt[i])
					more = true;
			}
			f->status = more ? PANTILT_ST_SLEW : PANTILT_ST_READY;
			if (f->status != PANTILT_ST_SLEW)
				break;
		}
	}
}

static int fake_set_target(struct drv *d, uint16_t pan, uint16_t tilt)
{
	struct fake *f = d->ctx;
	uint16_t want[PANTILT_AXES];
	int i;

	if (!f->initialized)
		return -EAGAIN;

	want[PANTILT_AXIS_PAN] = pan;
	want[PANTILT_AXIS_TILT] = tilt;
	for (i = 0; i < PANTILT_AXES; i++) {
		if (want[i] < f->safe_min[i]) {
			f->clamp_low[i]++;
			want[i] = f->safe_min[i];
		} else if (want[i] > f->safe_max[i]) {
			f->clamp_high[i]++;
			want[i] = f->safe_max[i];
		}
		f->tgt[i] = want[i];
	}
	f->cmd_count++;
	f->status = PANTILT_ST_SLEW;
	f->slew_from_cmd = true;
	/* 从"现在"起算第一步，否则 tick 里的补步循环会以为欠了一大截 */
	f->last_step_ms = d->now_ms;
	f->last_cmd_ms = d->now_ms;
	f->cmd_ms = d->now_ms;
	f->latency_pending = true;
	return 0;
}

static int fake_home(struct drv *d)
{
	struct fake *f = d->ctx;

	if (!f->initialized)
		return -EAGAIN;
	memcpy(f->tgt, f->home, sizeof(f->tgt));
	f->status = PANTILT_ST_SLEW;
	f->slew_from_cmd = false;
	f->last_step_ms = d->now_ms;
	return 0;
}

static int fake_stop(struct drv *d)
{
	struct fake *f = d->ctx;

	if (!f->initialized)
		return -EAGAIN;
	memcpy(f->tgt, f->cur, sizeof(f->tgt));
	f->status = PANTILT_ST_HOLD;
	f->slew_from_cmd = false;
	return 0;
}

static int fake_keepalive(struct drv *d)
{
	struct fake *f = d->ctx;

	if (!f->initialized)
		return -EAGAIN;
	/*
	 * 真驱动在这里只做 last_cmd_jiffies = jiffies。模拟版必须做等价的事，
	 * 否则"以 20 Hz 续命能让长距离移动走完"这一条在开发机上无法验证——
	 * 而那正是本次引入 KEEPALIVE 的全部目的。
	 */
	f->last_cmd_ms = d->now_ms;
	return 0;
}

static int fake_get_state(struct drv *d, struct pantilt_state *out)
{
	struct fake *f = d->ctx;
	int i;

	memset(out, 0, sizeof(*out));
	out->cmd_count = f->cmd_count;
	for (i = 0; i < PANTILT_AXES; i++) {
		out->clamp_low[i] = f->clamp_low[i];
		out->clamp_high[i] = f->clamp_high[i];
		out->cur_tick[i] = f->cur[i];
		out->tgt_tick[i] = f->tgt[i];
		out->home[i] = f->home[i];
		out->safe_min[i] = f->safe_min[i];
		out->safe_max[i] = f->safe_max[i];
	}
	out->i2c_err = f->i2c_err;
	out->watchdog_trips = f->watchdog_trips;
	out->last_update_us = f->last_update_us;
	out->last_latency_us = f->last_latency_us;
	out->status = f->initialized ? f->status : PANTILT_ST_UNINIT;
	out->chip_state = f->chip_state;
	out->slew_ms = f->slew_ms;
	out->watchdog_ms = f->watchdog_ms;
	return 0;
}

static void fake_close(struct drv *d)
{
	if (!d)
		return;
	free(d->ctx);
	free(d);
}

static const struct drv_ops fake_ops = {
	.set_target = fake_set_target,
	.home = fake_home,
	.stop = fake_stop,
	.keepalive = fake_keepalive,
	.get_state = fake_get_state,
	.tick = fake_tick,
	.close = fake_close,
};

struct drv *drv_open_fake(void)
{
	struct drv *d;
	struct fake *f;

	f = calloc(1, sizeof(*f));
	d = malloc(sizeof(*d));
	if (!f || !d) {
		free(f);
		free(d);
		return NULL;
	}
	f->home[PANTILT_AXIS_PAN] = FAKE_PAN_HOME;
	f->home[PANTILT_AXIS_TILT] = FAKE_TILT_HOME;
	f->safe_min[PANTILT_AXIS_PAN] = FAKE_PAN_MIN;
	f->safe_max[PANTILT_AXIS_PAN] = FAKE_PAN_MAX;
	f->safe_min[PANTILT_AXIS_TILT] = FAKE_TILT_MIN;
	f->safe_max[PANTILT_AXIS_TILT] = FAKE_TILT_MAX;
	/*
	 * 模拟驱动默认处于"已认领、停在 home"，对应板上 pantilt_ctl limit 已跑完。
	 * 置 PANTILT_FAKE_UNINIT=1 则复刻"还没认领"的情形，用来验证 daemon
	 * 确实会拒绝在未认领的驱动上启动——否则这条安全闸门永远测不到。
	 */
	if (!getenv("PANTILT_FAKE_UNINIT")) {
		memcpy(f->cur, f->home, sizeof(f->cur));
		memcpy(f->tgt, f->home, sizeof(f->tgt));
		f->status = PANTILT_ST_READY;
		f->chip_state = PANTILT_CHIP_RUNNING;
		f->initialized = true;
	}
	/* slew/watchdog 无论认领与否都要填，否则 get_state 会回 0 让调用方误判。 */
	f->slew_ms = FAKE_SLEW_MS;
	f->watchdog_ms = FAKE_WD_MS;

	d->ops = &fake_ops;
	d->ctx = f;
	return d;
}
