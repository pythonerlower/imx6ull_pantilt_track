/*
 * check_keepalive —— 板端验证 PANTILT_IOC_KEEPALIVE 的三个断言。
 *
 * 为什么要在板上单独验一遍：daemon 的宿主端联调用的是内存模拟驱动，
 * 它能证明 daemon 逻辑对，但证明不了"真驱动里 KEEPALIVE 确实只续命"。
 * 驱动是这次唯一被改动的内核代码，必须拿真硬件对答案。
 *
 * 本程序会**真的让舵机转动**，所以：
 *   - 必须显式加 --yes 才会动作，否则只打印计划；
 *   - 全程只用驱动回报的 safe_min/safe_max，不写死任何"中心值"；
 *   - 结束前必定回 HOME 并确认到位（有序归还），失败也回。
 *
 * 验证项：
 *   (a) 只发 KEEPALIVE 不产生任何运动，也不动 cmd_count；
 *   (b) 20 Hz KEEPALIVE 期间长距离移动能走完，watchdog_trips 保持 0；
 *   (c) 【对照】同样一次长移动，不发 KEEPALIVE 时确实会被 1500 ms 看门狗
 *       掐断并进入 HOLD。没有这一条，(b) 可能只是因为看门狗压根没武装。
 *   (d) 被 (c) 打断后，重新下发 SET_TARGET 能解除 HOLD 并走完。
 *
 * 用法: ./check_keepalive [--dev /dev/pantilt0] [--yes] [--keepalive-hz 20]
 * 退出码: 0 全部通过 / 1 有断言失败 / 2 用法或前置条件不满足 / 3 驱动错误
 */
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "pantilt_uapi.h"

static int fd = -1;
static volatile sig_atomic_t stop;
static int failures;

static void on_signal(int sig)
{
	(void)sig;
	stop = 1;
}

static uint64_t now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}

static void sleep_ms(uint64_t ms)
{
	struct timespec ts = { .tv_sec = (time_t)(ms / 1000),
			       .tv_nsec = (long)((ms % 1000) * 1000000) };

	nanosleep(&ts, NULL);
}

static int get_state(struct pantilt_state *st)
{
	if (ioctl(fd, PANTILT_IOC_GET_STATE, st) < 0) {
		fprintf(stderr, "[ERROR] GET_STATE: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

static void show(const char *tag)
{
	struct pantilt_state st;

	if (get_state(&st) < 0)
		return;
	printf("    %-10s status=%-6s cur=(%u,%u) tgt=(%u,%u) cmd=%llu wd=%llu\n",
	       tag, st.status == PANTILT_ST_READY ? "READY" :
		    st.status == PANTILT_ST_SLEW ? "SLEW" :
		    st.status == PANTILT_ST_HOLD ? "HOLD" : "UNINIT",
	       st.cur_tick[PANTILT_AXIS_PAN], st.cur_tick[PANTILT_AXIS_TILT],
	       st.tgt_tick[PANTILT_AXIS_PAN], st.tgt_tick[PANTILT_AXIS_TILT],
	       (unsigned long long)st.cmd_count,
	       (unsigned long long)st.watchdog_trips);
}

static void check(bool cond, const char *what)
{
	if (cond)
		printf("  [ok] %s\n", what);
	else {
		failures++;
		printf("  [FAIL] %s\n", what);
	}
}

/* 等目标到位。超时返回 false。期间可选地以 hz 频率发 KEEPALIVE。 */
static bool wait_arrive(const struct pantilt_target *tgt, uint64_t timeout_ms,
			unsigned keepalive_hz)
{
	uint64_t t0 = now_ms(), next_ka = t0;
	uint64_t period = keepalive_hz ? 1000 / keepalive_hz : 0;
	struct pantilt_state st;

	while (now_ms() - t0 < timeout_ms) {
		if (stop)
			return false;
		if (keepalive_hz && now_ms() >= next_ka) {
			ioctl(fd, PANTILT_IOC_KEEPALIVE);
			next_ka += period;
		}
		if (get_state(&st) < 0)
			return false;
		if (st.cur_tick[PANTILT_AXIS_PAN] == tgt->tick[PANTILT_AXIS_PAN] &&
		    st.cur_tick[PANTILT_AXIS_TILT] == tgt->tick[PANTILT_AXIS_TILT])
			return true;
		sleep_ms(10);
	}
	return false;
}

static int go_home(const char *why)
{
	uint64_t t0;
	struct pantilt_state st;

	printf("[归还] %s → 回 HOME\n", why);
	if (ioctl(fd, PANTILT_IOC_HOME) < 0) {
		fprintf(stderr, "[ERROR] HOME: %s\n", strerror(errno));
		return -1;
	}
	if (get_state(&st) < 0)
		return -1;
	t0 = now_ms();
	while (now_ms() - t0 < 15000) {
		if (stop)
			break;
		ioctl(fd, PANTILT_IOC_KEEPALIVE); /* 长距离回程同样需要续命 */
		if (get_state(&st) < 0)
			return -1;
		if (st.cur_tick[PANTILT_AXIS_PAN] == st.home[PANTILT_AXIS_PAN] &&
		    st.cur_tick[PANTILT_AXIS_TILT] == st.home[PANTILT_AXIS_TILT])
			break;
		sleep_ms(20);
	}
	show("home");
	return 0;
}

static void usage(const char *p)
{
	fprintf(stderr,
		"用法: %s [--dev PATH] [--yes] [--keepalive-hz N]\n"
		"  不加 --yes 只打印计划，不动舵机。\n"
		"  本程序会让云台真的转动：先回 HOME，再把 pan 推到远端，\n"
		"  然后在那里做「不续命会被看门狗掐断」的对照，最后回 HOME。\n",
		p);
}

int main(int argc, char **argv)
{
	static const struct option opts[] = {
		{ "dev", required_argument, NULL, 'd' },
		{ "yes", no_argument, NULL, 'y' },
		{ "keepalive-hz", required_argument, NULL, 'k' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	const char *dev = "/dev/pantilt0";
	bool yes = false;
	unsigned ka_hz = 20;
	struct sigaction sa = { .sa_handler = on_signal };
	struct pantilt_state st;
	struct pantilt_target tgt;
	uint16_t far_pan;
	int opt;

	while ((opt = getopt_long(argc, argv, "h", opts, NULL)) != -1) {
		switch (opt) {
		case 'd':
			dev = optarg;
			break;
		case 'y':
			yes = true;
			break;
		case 'k':
			ka_hz = (unsigned)atoi(optarg);
			break;
		default:
			usage(argv[0]);
			return opt == 'h' ? 0 : 2;
		}
	}
	if (ka_hz < 5 || ka_hz > 100) {
		fprintf(stderr, "[ERROR] --keepalive-hz 必须为 5..100\n");
		return 2;
	}

	fd = open(dev, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "[ERROR] 打开 %s 失败：%s\n", dev, strerror(errno));
		return 2;
	}
	if (get_state(&st) < 0)
		return 3;
	if (st.status == PANTILT_ST_UNINIT) {
		fprintf(stderr,
			"[ERROR] 驱动未认领（UNINIT）。先跑 ./pantilt_ctl limit。\n");
		return 2;
	}
	if (!st.watchdog_ms) {
		fprintf(stderr,
			"[ERROR] 驱动看门狗被禁用（watchdog_ms=0），无法验证续命。\n"
			"        重新 limit 时给一个非零的 watchdog_ms。\n");
		return 2;
	}

	/*
	 * 远端取离当前位置更远的那一头，保证行程长到能在半路被看门狗掐断。
	 * 平局（home 恰在正中）时**取 safe_min**：标定文件写明 pan 上端 515
	 * 超出 MG90S 规格上限、不得长时间驻留，而对照组要在那里停约 2 秒。
	 * 宁可多看一眼也不会到的那一端，也不要把舵机按在超规格的位置上。
	 */
	far_pan = (st.cur_tick[PANTILT_AXIS_PAN] - st.safe_min[PANTILT_AXIS_PAN] >=
		   st.safe_max[PANTILT_AXIS_PAN] - st.cur_tick[PANTILT_AXIS_PAN])
			  ? st.safe_min[PANTILT_AXIS_PAN]
			  : st.safe_max[PANTILT_AXIS_PAN];

	{
		/*
		 * 单条命令能走多远 = watchdog_ms / slew_ms。若它已经够走完全程，
		 * 对照组 (c) 就会**假通过**——云台在看门狗到点前就到了，于是
		 * "不续命会被掐断"变成了"不续命也走完了"，而工具会报通过。
		 * 这种沉默的假阴性比失败更危险，所以宁可拒绝运行。
		 */
		uint32_t reach = st.watchdog_ms / (st.slew_ms ? st.slew_ms : 1);
		uint16_t travel = (far_pan > st.cur_tick[PANTILT_AXIS_PAN])
					  ? far_pan - st.cur_tick[PANTILT_AXIS_PAN]
					  : st.cur_tick[PANTILT_AXIS_PAN] - far_pan;

		printf("=== check_keepalive ===\n");
		printf("设备 %s  slew=%u ms/tick  看门狗=%u ms\n", dev, st.slew_ms,
		       st.watchdog_ms);
		if (reach >= travel) {
			fprintf(stderr,
				"[ERROR] 单条命令可走 %u tick（%u/%u），而本次行程只有 %u tick。\n"
				"        这样对照组不会触发看门狗，测试会假通过。\n"
				"        先重新认领，把看门狗调小：\n"
				"            ./pantilt_ctl limit --slew %u --watchdog 1500 --yes\n",
				reach, st.watchdog_ms, st.slew_ms, travel, st.slew_ms);
			close(fd);
			return 2;
		}
		printf("单条命令最多走 %u tick，本次行程 %u tick（会被掐断）\n",
		       reach, travel);
	}

	printf("pan  safe=%u..%u  home=%u  当前=%u\n", st.safe_min[PANTILT_AXIS_PAN],
	       st.safe_max[PANTILT_AXIS_PAN], st.home[PANTILT_AXIS_PAN],
	       st.cur_tick[PANTILT_AXIS_PAN]);
	printf("计划：回 HOME → 推到 pan=%u（约 %u tick，%llu ms）→ 在远端做对照 → 回 HOME\n",
	       far_pan,
	       (unsigned)(far_pan > st.cur_tick[PANTILT_AXIS_PAN]
				  ? far_pan - st.cur_tick[PANTILT_AXIS_PAN]
				  : st.cur_tick[PANTILT_AXIS_PAN] - far_pan),
	       (unsigned long long)(st.slew_ms *
			(far_pan > st.cur_tick[PANTILT_AXIS_PAN]
				 ? far_pan - st.cur_tick[PANTILT_AXIS_PAN]
				 : st.cur_tick[PANTILT_AXIS_PAN] - far_pan)));
	if (!yes) {
		printf("\n未加 --yes，仅打印计划，未动作。\n");
		close(fd);
		return 0;
	}

	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	if (go_home("起点") < 0)
		goto out;
	if (get_state(&st) < 0)
		goto out;

	/* ---- (a) 只发 KEEPALIVE，不许有任何动作 ---- */
	printf("\n(a) 只发 KEEPALIVE 2 秒，应无任何运动、cmd_count 不变\n");
	{
		uint64_t cmd0 = st.cmd_count, wd0 = st.watchdog_trips;
		uint16_t p0 = st.cur_tick[PANTILT_AXIS_PAN];
		uint16_t t0 = st.cur_tick[PANTILT_AXIS_TILT];
		uint64_t begin = now_ms();
		unsigned n = 0;

		while (now_ms() - begin < 2000 && !stop) {
			if (ioctl(fd, PANTILT_IOC_KEEPALIVE) < 0) {
				fprintf(stderr, "[ERROR] KEEPALIVE: %s\n",
					strerror(errno));
				goto out;
			}
			n++;
			sleep_ms(1000 / ka_hz);
		}
		if (get_state(&st) < 0)
			goto out;
		printf("    发了 %u 次续命\n", n);
		check(st.cur_tick[PANTILT_AXIS_PAN] == p0 &&
			      st.cur_tick[PANTILT_AXIS_TILT] == t0,
		      "姿态未变（KEEPALIVE 不产生运动）");
		check(st.cmd_count == cmd0,
		      "cmd_count 未被续命污染（这正是加这个 ioctl 的理由）");
		check(st.status != PANTILT_ST_SLEW,
		      "status 未被续命强制回 SLEW");
		check(st.watchdog_trips == wd0, "期间看门狗未误触发");
	}

	/* ---- (b) 有续命：长移动必须走完 ---- */
	printf("\n(b) 20 Hz 续命下推到 pan=%u，应走完且看门狗不触发\n", far_pan);
	{
		uint64_t wd0 = st.watchdog_trips;
		uint64_t t0;

		tgt.tick[PANTILT_AXIS_PAN] = far_pan;
		tgt.tick[PANTILT_AXIS_TILT] = st.cur_tick[PANTILT_AXIS_TILT];
		if (ioctl(fd, PANTILT_IOC_SET_TARGET, &tgt) < 0) {
			fprintf(stderr, "[ERROR] SET_TARGET: %s\n", strerror(errno));
			goto out;
		}
		t0 = now_ms();
		if (!wait_arrive(&tgt, 30000, ka_hz))
			printf("    [注] 未在 30 s 内到位\n");
		if (get_state(&st) < 0)
			goto out;
		printf("    用时 %llu ms\n",
		       (unsigned long long)(now_ms() - t0));
		check(st.cur_tick[PANTILT_AXIS_PAN] == far_pan,
		      "已到达远端目标");
		check(st.watchdog_trips == wd0,
		      "全程看门狗未触发（续命确实在起作用）");
	}

	/* ---- (c) 对照：同一段路，不续命，应被看门狗掐断 ---- */
	printf("\n(c) 对照组：回程不发续命，应被 %u ms 看门狗掐断并进入 HOLD\n",
	       st.watchdog_ms);
	{
		uint64_t wd0 = st.watchdog_trips;
		uint16_t near = st.home[PANTILT_AXIS_PAN];
		uint64_t t0;

		tgt.tick[PANTILT_AXIS_PAN] = near;
		tgt.tick[PANTILT_AXIS_TILT] = st.cur_tick[PANTILT_AXIS_TILT];
		if (ioctl(fd, PANTILT_IOC_SET_TARGET, &tgt) < 0) {
			fprintf(stderr, "[ERROR] SET_TARGET: %s\n", strerror(errno));
			goto out;
		}
		t0 = now_ms();
		/* 静默等到看门狗该发作之后再多给一点余量。 */
		while (now_ms() - t0 < st.watchdog_ms + 700 && !stop)
			sleep_ms(50);
		if (get_state(&st) < 0)
			goto out;
		printf("    %llu ms 后 cur=%u（目标 %u）status=%u wd=%llu\n",
		       (unsigned long long)(now_ms() - t0),
		       st.cur_tick[PANTILT_AXIS_PAN], near, st.status,
		       (unsigned long long)st.watchdog_trips);
		check(st.watchdog_trips > wd0,
		      "看门狗确实触发了（证明 (b) 不是因为它没武装）");
		check(st.status == PANTILT_ST_HOLD,
		      "驱动停在 HOLD");
		check(st.cur_tick[PANTILT_AXIS_PAN] != near,
		      "移动被中途打断，没有走到目标");
	}

	/* ---- (d) HOLD 解除：重新下发 + 续命，回 HOME ---- */
	printf("\n(d) 重新下发 SET_TARGET 解除 HOLD，并续命回 HOME\n");
	{
		if (get_state(&st) < 0)
			goto out;
		tgt.tick[PANTILT_AXIS_PAN] = st.home[PANTILT_AXIS_PAN];
		tgt.tick[PANTILT_AXIS_TILT] = st.home[PANTILT_AXIS_TILT];
		if (ioctl(fd, PANTILT_IOC_SET_TARGET, &tgt) < 0) {
			fprintf(stderr, "[ERROR] SET_TARGET: %s\n", strerror(errno));
			goto out;
		}
		check(wait_arrive(&tgt, 30000, ka_hz), "重新下发后能解除 HOLD 并到位");
	}

out:
	/* 无论走到哪一步、无论成功失败，都归还到 HOME 再退出。 */
	if (go_home("收尾") < 0)
		failures++;
	close(fd);

	if (stop) {
		printf("\n被信号中断，已归还。\n");
		return 1;
	}
	printf("\n=== 结果：%s（%d 项失败）===\n",
	       failures ? "有失败" : "全部通过", failures);
	return failures ? 1 : 0;
}
