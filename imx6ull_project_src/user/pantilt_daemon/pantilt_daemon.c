/*
 * pantilt_daemon —— 板端控制面。PC 通过 UDP 下发目标，本进程把它翻译成
 * /dev/pantilt0 的 ioctl，并把遥测回送。
 *
 * 设计要点（每条都对应一个具体的失败模式）：
 *
 * 1. 两级看门狗，职责不同。
 *    - daemon 侧 500 ms（默认）：PC 断流 → 主动下发 STOP 保持位置。
 *      它管的是"上位机没了"。
 *    - 驱动侧 1500 ms：靠 KEEPALIVE 以心跳频率续命，一旦 daemon 自己卡死
 *      或被 kill，驱动会在 1500 ms 后中止爬升并进入 HOLD。
 *      它管的是"daemon 没了"。
 *    两层都保留，是因为它们兜的是不同的东西——只在 daemon 里做看门狗的话，
 *    daemon 一挂就没人兜底了。
 *
 * 2. 续命与下发达标是两件事。SET_TARGET 会无条件 ++cmd_count、把 status 强制
 *    回 SLEW、并重排爬升定时器（见 pantilt.c）。以 20 Hz 重复下发会把命令计数
 *    稀释成节拍器、让 status 抖动、并让爬升慢约 10%。所以：
 *      每拍        → KEEPALIVE（只续命）
 *      目标变化时  → SET_TARGET
 *    这也是 PANTILT_IOC_KEEPALIVE 存在的理由。
 *
 * 3. daemon 不认领芯片。SET_LIMIT 会唤醒 PCA9685 并让舵机开始转动，
 *    那是必须由带确认提示的工具（pantilt_ctl limit）做的一次性物理动作。
 *    驱动未认领时本程序直接退出，不越权代做。
 *
 * 4. SET_TARGET 只在"值变化"或"驱动处于 HOLD"时下发。后者不可省：
 *    驱动看门狗触发后必须由用户态再下发一次才恢复（pantilt.c:586），
 *    少了它会出现"客户端目标没变、云台却永久停在 HOLD"。
 *
 * 5.（Phase 7）本进程多了一条**只读**的本地输入：--event-sock 指定的 AF_UNIX
 *    数据报套接字，securityd 往里丢"振动事件"。daemon 只做两件事——
 *    计数、记下最近一次的时刻，然后把它们编码进遥测 reserved[1]。
 *    它**不**据此改任何目标：触发巡检是 PC 侧 control law 的事。
 *    不传 --event-sock 时这条路径整体不存在，行为与 Phase 6 逐字节一致。
 */
#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <getopt.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "drv.h"
#include "pantilt_proto.h"
#include "security_event.h"

#define EXIT_OK 0
#define EXIT_USAGE 1
#define EXIT_NET 2
#define EXIT_DRIVER 3

/*
 * seq 判重失败后，连续拒绝多久就判定为"客户端换了一茬"，接受新 seq 并重置会话。
 *
 * 为什么必须有这条自愈路径：客户端的 seq 取自本机单调时钟毫秒，PC 重启后
 * 会大幅倒退。若一律拒绝，daemon 就会永久失聪——表现是"云台再也不动，
 * 而客户端显示发送成功、daemon 显示收到了包"，极难排查。
 * 1 秒足够长：真实乱序包的滞后远小于 1 秒（20 Hz 下 1 秒 = 20 个包）。
 */
#define SEQ_RESYNC_MS 1000

struct daemon {
	int ctrl_fd;
	int telem_fd;
	struct sockaddr_in telem_dst;
	bool telem_dst_fixed; /* true：由 --telem-dst 指定；false：回发给控制源地址 */
	bool have_ctrl_src;
	struct sockaddr_in ctrl_src;
	uint16_t telem_port; /* 回发时用的目的端口（客户端在此监听遥测） */

	struct drv *drv;
	struct pantilt_state st;

	/* 协议校验 */
	uint32_t ctrl_rx_ok;
	uint32_t rej_crc, rej_seq, rej_stale, rej_mode, rej_range;
	uint32_t session_resets;

	/* 会话状态 */
	uint32_t last_seq;
	bool have_last_seq;
	uint64_t last_ts_us;
	bool have_last_ts;
	uint64_t rej_seq_since_ms;
	bool rej_seq_streak;

	/* 已下发的目标，用于去重 */
	uint16_t last_pan, last_tilt;
	uint16_t last_mode;
	bool have_last_cmd;

	/* 看门狗 */
	uint32_t daemon_wd_ms;
	uint64_t last_accept_ms;
	bool wd_tripped;
	uint32_t daemon_wd_trips;

	/* Phase 7：本地振动事件输入（只读，见头注释 5） */
	int ev_fd;           /* -1 = 未启用/未打开 */
	uint32_t ev_count;   /* 收到并接受的事件数 */
	uint32_t ev_bad;     /* 线格式不对丢弃的数量 */
	uint64_t ev_last_ms; /* 最近一次事件的 monotonic 毫秒 */

	uint32_t state; /* PTP_DST_* */
	uint32_t telem_seq;
	uint32_t tick_ms;
	bool verbose;
	volatile sig_atomic_t stop;
};

static struct daemon g;

/* ---------------- 时间 ---------------- */

static uint64_t now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}

static uint64_t now_us(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static void on_signal(int sig)
{
	(void)sig;
	g.stop = 1;
}

static void usage(const char *p)
{
	fprintf(stderr,
		"用法: %s [选项]\n"
		"  --dev PATH        云台设备（默认 /dev/pantilt0）\n"
		"  --bind ADDR       控制口监听地址（默认 0.0.0.0）\n"
		"  --ctrl-port N     控制口端口（默认 %u）\n"
		"  --telem-port N    遥测口端口（默认 %u）\n"
		"  --telem-dst A:P   遥测目的地址；不给则回发给最近的控制包源地址\n"
		"  --daemon-wd MS    daemon 侧看门狗，0 禁用（默认 500）\n"
		"  --tick-hz N       节拍频率，即续命与遥测的频率（默认 20）\n"
		"  --fake-driver     用内存模拟驱动，不碰硬件（开发机自测用）\n"
		"  --event-sock PATH 本地振动事件输入（AF_UNIX/SOCK_DGRAM，来自 securityd）；\n"
		"                    不给则完全不收事件，行为与 Phase 6 一致\n"
		"  -v, --verbose     打印每一条被拒绝的包及原因\n"
		"退出码: 0 正常 / 1 参数 / 2 网络 / 3 驱动不可用或未认领\n",
		p, PTP_CTRL_PORT, PTP_TELEM_PORT);
}

static int parse_hostport(const char *s, char *host, size_t n, uint16_t *port)
{
	const char *colon = strrchr(s, ':');

	if (!colon || colon == s || (size_t)(colon - s) >= n)
		return -1;
	memcpy(host, s, (size_t)(colon - s));
	host[colon - s] = '\0';
	*port = (uint16_t)atoi(colon + 1);
	return *port ? 0 : -1;
}

/* ---------------- 驱动动作的薄封装 ---------------- */

static void drv_state_refresh(struct daemon *d)
{
	if (d->drv->ops->get_state(d->drv, &d->st) < 0)
		fprintf(stderr, "[WARN] GET_STATE 失败\n");
}

static void send_keepalive(struct daemon *d)
{
	if (d->drv->ops->keepalive(d->drv) < 0 && d->verbose)
		fprintf(stderr, "[WARN] KEEPALIVE 失败（驱动可能未认领）\n");
}

static int drv_set_target(struct daemon *d, uint16_t pan, uint16_t tilt)
{
	int rc;

	d->drv->now_ms = now_ms();
	rc = d->drv->ops->set_target(d->drv, pan, tilt);
	if (rc < 0)
		fprintf(stderr, "[ERROR] SET_TARGET(%u, %u) 失败: %s\n", pan, tilt,
			strerror(-rc));
	return rc;
}

static void drv_home(struct daemon *d)
{
	int rc = d->drv->ops->home(d->drv);

	if (rc < 0)
		fprintf(stderr, "[ERROR] HOME 失败: %s\n", strerror(-rc));
}

static void drv_stop(struct daemon *d)
{
	int rc = d->drv->ops->stop(d->drv);

	if (rc < 0)
		fprintf(stderr, "[ERROR] STOP 失败: %s\n", strerror(-rc));
}

static const char *mode_name(uint16_t m)
{
	switch (m) {
	case PTP_MODE_MANUAL:
		return "MANUAL";
	case PTP_MODE_TRACK:
		return "TRACK";
	case PTP_MODE_HOME:
		return "HOME";
	case PTP_MODE_STOP:
		return "STOP";
	case PTP_MODE_HOLD:
		return "HOLD";
	default:
		return "?";
	}
}

/* ---------------- 控制包处理 ---------------- */

static void reject(struct daemon *d, uint32_t *counter, const char *why,
		   const char *detail)
{
	(*counter)++;
	if (d->verbose)
		fprintf(stderr, "[REJ] %s（%s）\n", why, detail);
}

static void apply_command(struct daemon *d, const struct ptp_ctrl *c)
{
	uint16_t want_pan, want_tilt;

	switch (c->mode) {
	case PTP_MODE_HOME:
		if (!d->have_last_cmd || d->last_mode != PTP_MODE_HOME ||
		    d->st.status == PANTILT_ST_HOLD)
			drv_home(d);
		d->state = PTP_DST_HOME;
		break;

	case PTP_MODE_STOP:
	case PTP_MODE_HOLD:
		if (!d->have_last_cmd ||
		    (d->last_mode != PTP_MODE_STOP &&
		     d->last_mode != PTP_MODE_HOLD))
			drv_stop(d);
		d->state = PTP_DST_HOLD;
		break;

	default: /* MANUAL / TRACK：两者行为一致，仅标签不同 */
		want_pan = c->pan_tick;
		want_tilt = c->tilt_tick;
		/*
		 * 只在值变化、或驱动处于 HOLD 时才下发。
		 * HOLD 这一条不能省：驱动看门狗触发后必须由用户态再下发一次
		 * 才恢复（见 pantilt.c 的 pt_ioctl_set_target），否则会出现
		 * "客户端目标没变、云台却永久停住"。
		 */
		if (!d->have_last_cmd || d->last_mode != c->mode ||
		    want_pan != d->last_pan || want_tilt != d->last_tilt ||
		    d->st.status == PANTILT_ST_HOLD)
			drv_set_target(d, want_pan, want_tilt);
		d->state = (c->mode == PTP_MODE_TRACK) ? PTP_DST_TRACK
						      : PTP_DST_MANUAL;
		break;
	}

	d->last_pan = c->pan_tick;
	d->last_tilt = c->tilt_tick;
	d->last_mode = c->mode;
	d->have_last_cmd = true;
	d->last_accept_ms = now_ms();
	d->wd_tripped = false;
	d->ctrl_rx_ok++;
	if (d->verbose)
		fprintf(stderr, "[OK] seq=%u %s(%u) pan=%u tilt=%u%s\n", c->seq,
			mode_name(c->mode), c->mode, c->pan_tick, c->tilt_tick,
			(c->flags & PTP_FLAG_HEARTBEAT) ? " [HB]" : "");
}

static void handle_ctrl(struct daemon *d)
{
	uint8_t buf[512];
	struct sockaddr_in src;
	socklen_t slen = sizeof(src);
	struct ptp_ctrl c;
	ssize_t n;
	int32_t delta;
	char detail[128];

	n = recvfrom(d->ctrl_fd, buf, sizeof(buf), 0, (struct sockaddr *)&src,
		     &slen);
	if (n < 0)
		return;

	/*
	 * 记住来源，遥测默认回发给它。注意**只沿用源 IP、不用源端口**：
	 * 客户端的控制 socket 通常绑在临时端口上，而它收遥测用的是
	 * TelemReceiver 绑定的 9101。照着源端口回发的话，遥测会全部打到一个
	 * 没人监听的端口上，表现为"控制正常、遥测一条也收不到"。
	 */
	if (!d->telem_dst_fixed) {
		d->ctrl_src = src;
		d->ctrl_src.sin_port = htons(d->telem_port);
		d->have_ctrl_src = true;
	}

	if (ptp_ctrl_decode(buf, (size_t)n, &c) != 0) {
		reject(d, &d->rej_crc, "长度或 CRC 不符", "见 ptp_ctrl_decode");
		return;
	}
	if (c.magic != PTP_CTRL_MAGIC) {
		snprintf(detail, sizeof(detail), "魔数 0x%08x", c.magic);
		reject(d, &d->rej_crc, "魔数不符（可能串了端口）", detail);
		return;
	}
	if (c.version != PTP_VERSION) {
		snprintf(detail, sizeof(detail), "版本 %u", c.version);
		reject(d, &d->rej_crc, "版本不符", detail);
		return;
	}

	/* --- seq：判重、判过期，并在连续拒绝过久时按"新会话"自愈 --- */
	delta = d->have_last_seq ? (int32_t)(c.seq - d->last_seq) : 1;
	if (d->have_last_seq && delta <= 0) {
		uint64_t t = now_ms();

		if (!d->rej_seq_streak) {
			d->rej_seq_streak = true;
			d->rej_seq_since_ms = t;
		}
		/*
		 * 注意：启动计时和放行是**两个独立的判断**，不能写成 if/else if。
		 * 写成 else-if 会让"开始计时"那一支直接落到下面的接受路径上，
		 * 于是第一个乱序包被静默收下、还把 last_seq 往回带——
		 * 自愈机制本来只是逃生门，那样就成了主路径。
		 */
		if (t - d->rej_seq_since_ms > SEQ_RESYNC_MS) {
			/*
			 * 持续被 seq 挡住超过 SEQ_RESYNC_MS：判定为客户端换了一茬
			 * （PC 重启后其单调时钟归零，seq 会大幅倒退）。
			 * 接受这一包并重置会话，避免 daemon 永久失聪。
			 */
			d->session_resets++;
			d->rej_seq_streak = false;
			d->have_last_ts = false;
			d->have_last_cmd = false;
			fprintf(stderr,
				"[WARN] seq 持续倒退 %.0f ms，判定为客户端新会话，已重置（第 %u 次）\n",
				(double)(t - d->rej_seq_since_ms),
				d->session_resets);
		} else {
			snprintf(detail, sizeof(detail), "seq %u <= 上次 %u（已持续 %.0f ms）",
				 c.seq, d->last_seq, (double)(t - d->rej_seq_since_ms));
			reject(d, &d->rej_seq, "seq 重复或倒退", detail);
			return;
		}
	} else {
		d->rej_seq_streak = false;
	}
	d->last_seq = c.seq;
	d->have_last_seq = true;

	/* --- 时间戳新鲜度：PC 与板端不同源，只能判单调，不能比绝对值 --- */
	if (d->have_last_ts && c.timestamp_us <= d->last_ts_us) {
		snprintf(detail, sizeof(detail), "ts %llu <= 上次 %llu",
			 (unsigned long long)c.timestamp_us,
			 (unsigned long long)d->last_ts_us);
		reject(d, &d->rej_stale, "时间戳倒退", detail);
		return;
	}
	d->last_ts_us = c.timestamp_us;
	d->have_last_ts = true;

	/* --- 模式 --- */
	if (c.mode < PTP_MODE_MANUAL || c.mode > PTP_MODE_HOLD) {
		snprintf(detail, sizeof(detail), "模式 %u", c.mode);
		reject(d, &d->rej_mode, "非法模式", detail);
		return;
	}

	/* --- 值域：只有 MANUAL/TRACK 用 tick，其余模式忽略 tick --- */
	if (c.mode == PTP_MODE_MANUAL || c.mode == PTP_MODE_TRACK) {
		if (c.pan_tick < d->st.safe_min[PANTILT_AXIS_PAN] ||
		    c.pan_tick > d->st.safe_max[PANTILT_AXIS_PAN] ||
		    c.tilt_tick < d->st.safe_min[PANTILT_AXIS_TILT] ||
		    c.tilt_tick > d->st.safe_max[PANTILT_AXIS_TILT]) {
			snprintf(detail, sizeof(detail),
				 "pan %u 不 ∈ [%u,%u] 或 tilt %u 不 ∈ [%u,%u]",
				 c.pan_tick, d->st.safe_min[PANTILT_AXIS_PAN],
				 d->st.safe_max[PANTILT_AXIS_PAN], c.tilt_tick,
				 d->st.safe_min[PANTILT_AXIS_TILT],
				 d->st.safe_max[PANTILT_AXIS_TILT]);
			reject(d, &d->rej_range, "目标越出安全范围", detail);
			return;
		}
	}

	apply_command(d, &c);
}

/* ---------------- 看门狗与遥测 ---------------- */

static void check_watchdog(struct daemon *d)
{
	uint64_t t;

	if (!d->daemon_wd_ms || d->wd_tripped || !d->have_last_cmd)
		return;
	t = now_ms();
	if (t - d->last_accept_ms <= d->daemon_wd_ms)
		return;

	drv_stop(d);
	d->daemon_wd_trips++;
	d->wd_tripped = true;
	d->state = PTP_DST_LOST;
	fprintf(stderr,
		"[WARN] %u ms 未收到控制包，已下发 STOP 保持位置（第 %u 次）\n",
		d->daemon_wd_ms, d->daemon_wd_trips);
}

static void send_telem(struct daemon *d)
{
	uint8_t buf[PTP_TELEM_SIZE];
	struct ptp_telem t;
	const struct sockaddr_in *dst = NULL;

	if (d->telem_dst_fixed)
		dst = &d->telem_dst;
	else if (d->have_ctrl_src)
		dst = &d->ctrl_src;
	if (!dst)
		return; /* 还不知道该发给谁 */

	memset(&t, 0, sizeof(t));
	t.magic = PTP_TELEM_MAGIC;
	t.version = PTP_VERSION;
	t.drv_status = d->st.status;
	t.seq = d->telem_seq++;
	t.timestamp_us = now_us();
	t.chip_state = (uint8_t)d->st.chip_state;
	t.daemon_state = (uint8_t)d->state;
	t.slew_ms = (uint16_t)d->st.slew_ms;
	t.wd_ms = (uint16_t)d->st.watchdog_ms;
	t.daemon_wd_ms = (uint16_t)d->daemon_wd_ms;
	t.pan_cur = d->st.cur_tick[PANTILT_AXIS_PAN];
	t.tilt_cur = d->st.cur_tick[PANTILT_AXIS_TILT];
	t.pan_tgt = d->st.tgt_tick[PANTILT_AXIS_PAN];
	t.tilt_tgt = d->st.tgt_tick[PANTILT_AXIS_TILT];
	t.pan_home = d->st.home[PANTILT_AXIS_PAN];
	t.tilt_home = d->st.home[PANTILT_AXIS_TILT];
	t.pan_safe_min = d->st.safe_min[PANTILT_AXIS_PAN];
	t.pan_safe_max = d->st.safe_max[PANTILT_AXIS_PAN];
	t.tilt_safe_min = d->st.safe_min[PANTILT_AXIS_TILT];
	t.tilt_safe_max = d->st.safe_max[PANTILT_AXIS_TILT];
	t.ms_since_ctrl = d->have_last_cmd
				  ? (uint32_t)(now_ms() - d->last_accept_ms)
				  : 0;
	t.last_latency_us = (uint32_t)d->st.last_latency_us;
	t.cmd_count = (uint32_t)d->st.cmd_count;
	t.clamp_low_pan = (uint32_t)d->st.clamp_low[PANTILT_AXIS_PAN];
	t.clamp_low_tilt = (uint32_t)d->st.clamp_low[PANTILT_AXIS_TILT];
	t.clamp_high_pan = (uint32_t)d->st.clamp_high[PANTILT_AXIS_PAN];
	t.clamp_high_tilt = (uint32_t)d->st.clamp_high[PANTILT_AXIS_TILT];
	t.i2c_err = (uint32_t)d->st.i2c_err;
	t.watchdog_trips = (uint32_t)d->st.watchdog_trips;
	t.ctrl_rx_ok = d->ctrl_rx_ok;
	t.rej_crc = d->rej_crc;
	t.rej_seq = d->rej_seq;
	t.rej_stale = d->rej_stale;
	t.rej_mode = d->rej_mode;
	t.rej_range = d->rej_range;
	t.daemon_wd_trips = d->daemon_wd_trips;
	/* reserved[0] 暂用于回传会话重置次数（见 docs/protocol.md）。 */
	t.reserved[0] = d->session_resets;
	/*
	 * reserved[1]（Phase 7）：高 16 位=事件计数、低 16 位=距最近一次事件的毫秒数。
	 * 没收到过事件时 age 记 0xFFFF（"从没有过"），PC 侧靠计数变化判上升沿。
	 */
	t.reserved[1] = PTP_TELEM_EV_PACK(d->ev_count,
					  d->ev_count
						  ? (uint32_t)(now_ms() - d->ev_last_ms)
						  : 0xffffu);

	ptp_telem_encode(&t, buf);
	if (sendto(d->telem_fd, buf, sizeof(buf), 0, (struct sockaddr *)dst,
		   sizeof(*dst)) < 0 && d->verbose)
		fprintf(stderr, "[WARN] 遥测发送失败: %s\n", strerror(errno));
}

/* ---------------- 本地事件输入（Phase 7，只读） ---------------- */

/*
 * 绑定事件套接字。整段刻意只做"收"，不做"发"：daemon 不回答 securityd，
 * 也不给它任何回执——单向通道，securityd 挂掉对云台零影响。
 *
 * 关于 stale socket：bind 到已存在的路径会 EADDRINUSE。这里**不无条件 unlink**
 * （--event-sock 是调用方给的路径，误删别人的文件不可接受），只在
 * "它是个 SOCK_STREAM/SOCK_DGRAM 文件、且连不上（ECONNREFUSED）"才认定是上次
 * 崩溃留下的死文件，删掉重绑。连得上就说明另有活着的 daemon，本进程直接失败退出。
 */
/* 有没有别的进程正绑在这个地址上。connect 成功 = 有人听。 */
static bool unix_sock_is_live(const struct sockaddr_un *sa)
{
	int probe = socket(AF_UNIX, SOCK_DGRAM, 0);
	bool live = false;

	if (probe < 0)
		return true; /* 连探测都建不了，保守当成"有人"，不去删别人的东西 */
	live = connect(probe, (const struct sockaddr *)sa, sizeof(*sa)) == 0;
	close(probe);
	return live;
}

static int event_setup(struct daemon *d, const char *path)
{
	struct sockaddr_un sa;
	struct stat sb;

	if (!path || !*path)
		return 0; /* 未启用 */
	if (strlen(path) >= sizeof(sa.sun_path)) {
		fprintf(stderr, "[ERROR] --event-sock 路径过长\n");
		return -1;
	}

	memset(&sa, 0, sizeof(sa));
	sa.sun_family = AF_UNIX;
	snprintf(sa.sun_path, sizeof(sa.sun_path), "%s", path);

	d->ev_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (d->ev_fd < 0) {
		perror("[ERROR] socket(AF_UNIX)");
		return -1;
	}
	if (bind(d->ev_fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		if (errno != EADDRINUSE) {
			fprintf(stderr, "[ERROR] 绑定事件套接字 %s 失败：%s\n", path,
				strerror(errno));
			goto fail;
		}
		/* EADDRINUSE：要么是别的 daemon 活着，要么是上次崩溃留下的死文件。 */
		if (unix_sock_is_live(&sa)) {
			fprintf(stderr,
				"[ERROR] %s 已被占用（另有 daemon 在跑），拒绝启动\n",
				path);
			goto fail;
		}
		if (stat(path, &sb) != 0 || !S_ISSOCK(sb.st_mode)) {
			fprintf(stderr,
				"[ERROR] %s 已存在且不是套接字，拒绝删除，拒绝启动\n",
				path);
			goto fail;
		}
		printf("[WARN] 删掉上次遗留的事件套接字 %s\n", path);
		unlink(path);
		if (bind(d->ev_fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
			fprintf(stderr, "[ERROR] 重绑事件套接字 %s 失败：%s\n", path,
				strerror(errno));
			goto fail;
		}
	}
	/* 收紧权限：这条通道只有板内 root 的两个进程用。 */
	chmod(path, 0600);
	printf("[INFO] 事件输入 AF_UNIX/SOCK_DGRAM %s\n", path);
	return 0;

fail:
	close(d->ev_fd);
	d->ev_fd = -1;
	return -1;
}

static void event_close(struct daemon *d)
{
	if (d->ev_fd >= 0) {
		close(d->ev_fd);
		d->ev_fd = -1;
	}
}

static void handle_event(struct daemon *d)
{
	uint8_t buf[PTP_SEKEV_SIZE + 16];
	struct ptp_sekev e;
	ssize_t n;

	/* 收干：一次要处理掉积压，否则 poll 会一直说可读。 */
	while ((n = recv(d->ev_fd, buf, sizeof(buf), MSG_DONTWAIT)) > 0) {
		if (ptp_sekev_unpack(buf, (size_t)n, &e) < 0) {
			d->ev_bad++;
			if (d->verbose)
				fprintf(stderr, "[WARN] 事件线格式不对，丢弃 %zd 字节\n",
					n);
			continue;
		}
		d->ev_count++;
		d->ev_last_ms = now_ms();
		if (d->verbose)
			printf("[INFO] 事件 kind=%u seq=%u（sender t=%u ms）\n",
			       e.kind, e.seq, e.t_ms);
	}
}

/* ---------------- 启动与收尾 ---------------- */

static void print_banner(struct daemon *d, const char *dev, bool fake)
{
	printf("[INFO] 云台 daemon 启动%s\n", fake ? "（--fake-driver，不碰硬件）" : "");
	printf("       设备 %s  status=%u chip=%u\n", dev, d->st.status,
	       d->st.chip_state);
	printf("       pan  cur=%u home=%u safe=%u..%u\n",
	       d->st.cur_tick[PANTILT_AXIS_PAN], d->st.home[PANTILT_AXIS_PAN],
	       d->st.safe_min[PANTILT_AXIS_PAN],
	       d->st.safe_max[PANTILT_AXIS_PAN]);
	printf("       tilt cur=%u home=%u safe=%u..%u\n",
	       d->st.cur_tick[PANTILT_AXIS_TILT], d->st.home[PANTILT_AXIS_TILT],
	       d->st.safe_min[PANTILT_AXIS_TILT],
	       d->st.safe_max[PANTILT_AXIS_TILT]);
	printf("       slew=%u ms/tick  驱动看门狗=%u ms  daemon 看门狗=%u ms\n",
	       d->st.slew_ms, d->st.watchdog_ms, d->daemon_wd_ms);
	printf("       节拍 %u Hz（同时用于 KEEPALIVE 与遥测）\n",
	       1000 / (d->tick_ms ? d->tick_ms : 50));
}

int main(int argc, char **argv)
{
	static const struct option opts[] = {
		{ "dev", required_argument, NULL, 'd' },
		{ "bind", required_argument, NULL, 'b' },
		{ "ctrl-port", required_argument, NULL, 'c' },
		{ "telem-port", required_argument, NULL, 't' },
		{ "telem-dst", required_argument, NULL, 'T' },
		{ "daemon-wd", required_argument, NULL, 'w' },
		{ "tick-hz", required_argument, NULL, 'k' },
		{ "fake-driver", no_argument, NULL, 'F' },
		{ "event-sock", required_argument, NULL, 'e' },
		{ "verbose", no_argument, NULL, 'v' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	const char *dev = "/dev/pantilt0";
	const char *bind_addr = "0.0.0.0";
	uint16_t ctrl_port = PTP_CTRL_PORT, telem_port = PTP_TELEM_PORT;
	char telem_host[64] = "";
	uint16_t telem_dst_port = 0;
	bool fake = false;
	const char *event_sock = "";
	struct sockaddr_in a;
	struct sigaction sa = { .sa_handler = on_signal };
	int opt, rc;
	unsigned tick_hz = 20;
	uint64_t next_tick;

	/*
	 * 行缓冲。默认 stdout 接管道时是块缓冲，而 stderr 永远是无缓冲的，
	 * 于是 WARN/ERROR 会跑到启动横幅前面，日志读起来是乱的；更要紧的是
	 * 上层脚本要靠"控制口 UDP ..."这一行判断 daemon 已经可以收包了，
	 * 块缓冲会让它长时间看不到。诊断工具的输出顺序必须可信。
	 */
	setvbuf(stdout, NULL, _IOLBF, 0);

	memset(&g, 0, sizeof(g));
	g.ctrl_fd = g.telem_fd = -1;
	g.ev_fd = -1;
	g.daemon_wd_ms = 500;
	g.telem_seq = 0;

	while ((opt = getopt_long(argc, argv, "e:vh", opts, NULL)) != -1) {
		switch (opt) {
		case 'd':
			dev = optarg;
			break;
		case 'b':
			bind_addr = optarg;
			break;
		case 'c':
			ctrl_port = (uint16_t)atoi(optarg);
			break;
		case 't':
			telem_port = (uint16_t)atoi(optarg);
			break;
		case 'T':
			if (parse_hostport(optarg, telem_host, sizeof(telem_host),
					   &telem_dst_port) < 0) {
				fprintf(stderr, "[ERROR] --telem-dst 需为 HOST:PORT\n");
				return EXIT_USAGE;
			}
			g.telem_dst_fixed = true;
			break;
		case 'w':
			g.daemon_wd_ms = (uint32_t)atoi(optarg);
			break;
		case 'k':
			tick_hz = (unsigned)atoi(optarg);
			break;
		case 'F':
			fake = true;
			break;
		case 'e':
			event_sock = optarg;
			break;
		case 'v':
			g.verbose = true;
			break;
		default:
			usage(argv[0]);
			return opt == 'h' ? EXIT_OK : EXIT_USAGE;
		}
	}

	if (tick_hz < 2 || tick_hz > 100) {
		fprintf(stderr, "[ERROR] --tick-hz 必须为 2..100\n");
		return EXIT_USAGE;
	}
	g.tick_ms = 1000 / tick_hz;
	if (g.tick_ms == 0)
		g.tick_ms = 10;
	g.telem_port = telem_port;

	/* --- 驱动 --- */
	g.drv = fake ? drv_open_fake() : drv_open_real(dev);
	if (!g.drv) {
		fprintf(stderr, "[ERROR] 打开云台设备 %s 失败：%s\n", dev,
			strerror(errno));
		if (errno == ENOENT || errno == ENODEV)
			fprintf(stderr,
				"        模块是否已加载？板端执行：/sbin/insmod pantilt.ko\n");
		return EXIT_DRIVER;
	}
	drv_state_refresh(&g);
	if (g.st.status == PANTILT_ST_UNINIT) {
		fprintf(stderr,
			"[ERROR] 驱动尚未认领（status=UNINIT），拒绝启动。\n"
			"        SET_LIMIT 会唤醒 PCA9685 并让舵机真的转动，\n"
			"        这是必须由带确认提示的工具做的一次性物理动作，\n"
			"        daemon 刻意不代做。请先执行：\n"
			"            ./pantilt_ctl limit        （会提示按 Enter 确认）\n"
			"        然后重新启动本程序。\n");
		g.drv->ops->close(g.drv);
		return EXIT_DRIVER;
	}

	/* --- 网络 --- */
	g.ctrl_fd = socket(AF_INET, SOCK_DGRAM, 0);
	g.telem_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (g.ctrl_fd < 0 || g.telem_fd < 0) {
		perror("[ERROR] socket");
		return EXIT_NET;
	}
	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_port = htons(ctrl_port);
	if (inet_pton(AF_INET, bind_addr, &a.sin_addr) != 1) {
		fprintf(stderr, "[ERROR] --bind %s 不是合法 IPv4\n", bind_addr);
		return EXIT_USAGE;
	}
	if (bind(g.ctrl_fd, (struct sockaddr *)&a, sizeof(a)) < 0) {
		fprintf(stderr, "[ERROR] 绑定 UDP %s:%u 失败：%s\n", bind_addr,
			ctrl_port, strerror(errno));
		return EXIT_NET;
	}
	if (g.telem_dst_fixed) {
		memset(&g.telem_dst, 0, sizeof(g.telem_dst));
		g.telem_dst.sin_family = AF_INET;
		g.telem_dst.sin_port = htons(telem_dst_port);
		if (inet_pton(AF_INET, telem_host, &g.telem_dst.sin_addr) != 1) {
			fprintf(stderr, "[ERROR] --telem-dst 地址非法\n");
			return EXIT_USAGE;
		}
		printf("[INFO] 遥测固定发往 %s:%u\n", telem_host, telem_dst_port);
	} else {
		/*
		 * 遥测是主动外发，本机不需要绑定端口；这里只记住"回发到客户端
		 * 哪个端口"。客户端收遥测的是 TelemReceiver，端口是 telem_port。
		 */
		g.telem_port = telem_port;
		printf("[INFO] 遥测回发给最近控制包的源 IP 的 %u 端口（可用 --telem-dst 固定）\n",
		       telem_port);
	}

	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	/*
	 * 事件输在前、控制口在后：这条通道绑不上（另有 daemon / 路径被占）
	 * 应当整体失败退出，而不是带着半条链路跑起来。
	 */
	if (event_setup(&g, event_sock) < 0)
		return EXIT_NET;

	print_banner(&g, dev, fake);
	printf("[INFO] 控制口 UDP %s:%u；Ctrl-C 停止\n", bind_addr, ctrl_port);

	next_tick = now_ms();
	while (!g.stop) {
		struct pollfd pfd[2];
		nfds_t nfd = 1;
		int timeout;
		uint64_t t = now_ms();

		pfd[0].fd = g.ctrl_fd;
		pfd[0].events = POLLIN;
		pfd[0].revents = 0;
		if (g.ev_fd >= 0) {
			pfd[1].fd = g.ev_fd;
			pfd[1].events = POLLIN;
			pfd[1].revents = 0;
			nfd = 2;
		}

		timeout = (next_tick > t) ? (int)(next_tick - t) : 0;
		rc = poll(pfd, nfd, timeout);
		if (rc < 0) {
			if (errno == EINTR)
				continue;
			perror("[ERROR] poll");
			break;
		}
		if (rc > 0 && (pfd[0].revents & POLLIN))
			handle_ctrl(&g);
		if (nfd > 1 && (pfd[1].revents & POLLIN))
			handle_event(&g);

		t = now_ms();
		if (t < next_tick)
			continue;

		/* 节拍：先推进模拟驱动，再续命、检查看门狗、刷新状态、发遥测。 */
		g.drv->now_ms = t;
		if (g.drv->ops->tick)
			g.drv->ops->tick(g.drv);
		send_keepalive(&g);
		check_watchdog(&g);
		drv_state_refresh(&g);
		send_telem(&g);

		next_tick += g.tick_ms;
		if (next_tick <= t) /* 落后了就重置基准，不让欠账累积成突发 */
			next_tick = t + g.tick_ms;
	}

	printf("\n[INFO] 收到停止信号，收尾中\n");
	/* 收尾：主动 STOP 保持位置，再打一份最终状态。不写 SLEEP、不停输出，
	 * 因为那是 rmmod 的职责；daemon 退出后舵机应保持有劲留在原处。 */
	if (g.have_last_cmd)
		drv_stop(&g);
	drv_state_refresh(&g);
	printf("[INFO] 最终 status=%u cur=(%u,%u) cmd=%llu clamp=%llu+%llu wd=%llu/%u\n",
	       g.st.status, g.st.cur_tick[PANTILT_AXIS_PAN],
	       g.st.cur_tick[PANTILT_AXIS_TILT],
	       (unsigned long long)g.st.cmd_count,
	       (unsigned long long)(g.st.clamp_low[0] + g.st.clamp_low[1]),
	       (unsigned long long)(g.st.clamp_high[0] + g.st.clamp_high[1]),
	       (unsigned long long)g.st.watchdog_trips, g.daemon_wd_trips);
	printf("[INFO] 控制包 接受=%u 拒(crc/seq/stale/mode/range)=%u/%u/%u/%u/%u 会话重置=%u\n",
	       g.ctrl_rx_ok, g.rej_crc, g.rej_seq, g.rej_stale, g.rej_mode,
	       g.rej_range, g.session_resets);

	if (g.ev_fd >= 0) {
		event_close(&g);
		/* 套接字文件是共享资源，退出时删掉，免得下次起来撞上 EADDRINUSE。 */
		if (event_sock && *event_sock)
			unlink(event_sock);
		printf("[INFO] 事件 收到=%u 丢弃=%u\n", g.ev_count, g.ev_bad);
	}
	if (g.ctrl_fd >= 0)
		close(g.ctrl_fd);
	if (g.telem_fd >= 0)
		close(g.telem_fd);
	g.drv->ops->close(g.drv);
	return EXIT_OK;
}
