/*
 * securityd —— 板端安防状态机（Phase 6）
 *
 * 把五个 Phase 各自验过的模块拼成一条真正跑得通的链：
 *   AP3216C 接近事件 → ALERT → OV5640 视频进 PC → 录音落盘 → 跟踪就绪 → 回到 GUARD
 *
 * 设计要点（每条都对着一个具体的失败模式）：
 *
 * 1. ALERT 只由 AP3216C 的 Input 事件触发。ALS 只做定时轮询采样、进日志与统计，不作判据
 *    （它根本没有事件出口）。ICM20608 振动在 Phase 7 里多了一条"事件"出路，见下面 8。
 *    本程序**不提供任何注入 ALERT 的入口**：留一个后门就等于留一个能伪造验收的口子。
 *
 * 2. TRACK_READY 是"跟踪就绪"而不是"识别成功"，判据只有一条：video_link 真的在发包。
 *    securityd 不感知 PC 是否真在跟踪目标，也**刻意不引入任何旁证**——唯一可能的
 *    "对端在驱动控制面"旁证是 GET_STATE 的 cmd_count 增长率，而它被 EBUSY 挡死（见 7）。
 *    这个空洞写进 README 与验收文档，不包装成"识别成功"。
 *
 * 3. 子进程一律 fork+exec，收 stdout/stderr 到管道，退出码与原始输出都留档。
 *    就绪与否**不看退出码**：video_link 退出码 3 表示摄像头故障，但"进程活着、
 *    一个包都没发"它报不出来。所以判据取 [stats] 行里的发包数 > 0。
 *
 * 4. 信号全部走 signalfd，定时全走 timerfd。不用 signal handler + poll 超时，
 *    是因为"信号打断系统调用"这条路径在这里只会带来 EINTR 分支而没有收益。
 *
 * 5. 摄像头故障只在 securityd 内部有序重载，**绝不重启板子、绝不动云台**：
 *    先让 video_link 归还 /dev/video1，再 rmmod（顺序是 mx6s_capture → ov5640_camera，
 *    不是加载顺序的逆序，理由是运行时的引用而不是符号依赖），然后 insmod，最后静置。
 *
 * 6. 危险动作只有"起子进程"和"rmmod/insmod 摄像头"。本程序**完全不碰云台**：
 *    不 open /dev/pantilt0、不 include 它的 ABI 头、不发任何控制包。
 *    这不是一句承诺而是结构性事实（下面 7 说明了为什么连只读也做不到）。
 *    告警结束时也只收音频，视频保持常开——摄像头该不该关不是本程序说了算，
 *    而且掐掉它下一次告警又要从头 probe /dev/video1。
 *
 * 7. 为什么连只读 GET_STATE 也不做：驱动 pt_open() 用 test_and_set_bit 只允许
 *    一个打开者（pantilt.c，注释写明"避免两方争抢同一路 PWM"），而这条链上
 *    pantilt_daemon 一直在占着它。2026-10-07 实测：daemon 运行期间用
 *    pantilt_ctl info 打开 /dev/pantilt0 得到 EBUSY。
 *    因此"用 cmd_count 增长率当对端正在驱动控制面的旁证"这条路根本走不通，
 *    本轮明确放弃该旁证（见 plan/phase6_system_integration.md 的取舍表）。
 *
 * 8. Phase 7 的振动事件：**它不是中断，是软件阈值**。M1 实测（logs/phase7/
 *    m1_icm_int_probe_20261008.md）把 MX6UL_PAD_JTAG_MOD 临时配成 GPIO 输入后，
 *    该脚被外部恒定拉低、零边沿，`request_irq` 挂不上去，所以 plan §2.2 的备选生效：
 *    继续用 IIO 轮询读加速度，自己比阈值，超阈值时把采样间隔压到 vib_burst_ms（默认 20 ms）
 *    并保持 vib_burst_hold_ms（默认 2 s），期间通过本地 UNIX 数据报套接字
 *    （--event-sock）把事件发给 pantilt_daemon。
 *    判据是**逐轴帧间最大变化量**（--vib-thr-mg，mg），不是合矢量幅值 ——
 *    后者有 1 g 直流分量、单位也踩过坑，换掉的理由写在 vib_check 上面。
 *    文档、日志、变量名一律写"软件阈值"，**不得出现"唤醒中断/WoM 中断"字样**。
 *    不开 --vib-thr-mg（默认 0）时，本程序行为与 Phase 6 完全一致。
 */
#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/timerfd.h>
#include <sys/un.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <linux/input.h>
#include "security_event.h"

#define SD_EPOLL_MAX 8
#define SD_LINE_MAX  1024
#define SD_PATH_MAX  512

/* ---------------------------------------------------------------- 配置 */

struct cfg {
	const char *input_name;     /* Input 设备名，自动扫描匹配，不写死 eventX */
	const char *iio_name;       /* IIO 设备名（ICM20608） */
	const char *video_bin;
	const char *video_dev;
	const char *video_dst;
	unsigned video_w, video_h, video_fps, video_slots;
	unsigned video_ready_ms;
	const char *audio_bin;
	const char *records_dir;
	const char *audio_tag;
	unsigned audio_warmup_ms;
	unsigned alert_hold_ms, alert_max_ms;
	unsigned near_guard_ms, tick_ms;
	unsigned als_poll_ms, vib_poll_ms;
	unsigned vib_thr_mg;        /* 0 = 关闭振动事件（Phase 6 行为） */
	unsigned vib_burst_ms;      /* 超阈值期间的采样间隔 */
	unsigned vib_burst_hold_ms; /* 超阈值后保持高频采样的时长 */
	const char *event_sock;     /* 事件出口；空串 = 不发 */
	unsigned stats_ms;
	unsigned cam_attempts;
	const char *cam_mod_dir;
	const char *status_file;
	const char *log_dir;
	int dry_run;
	int verbose;
};

static struct cfg g = {
	.input_name = "ap3216c-proximity",
	.iio_name = "icm20608_iio",
	.video_bin = "./video_link",
	.video_dev = "/dev/video1",
	.video_dst = "192.168.137.100:9200",
	.video_w = 320, .video_h = 240, .video_fps = 15, .video_slots = 8,
	.video_ready_ms = 5000,
	.audio_bin = "./audio_capture",
	.records_dir = "/root/imx6hub_demo/records",
	.audio_tag = "alert",
	.audio_warmup_ms = 2000,
	.alert_hold_ms = 20000, .alert_max_ms = 120000,
	.near_guard_ms = 200, .tick_ms = 100,
	.als_poll_ms = 2000, .vib_poll_ms = 100,
	.vib_thr_mg = 0, .vib_burst_ms = 20, .vib_burst_hold_ms = 2000,
	.event_sock = "",
	.stats_ms = 1000,
	.cam_attempts = 3,
	.cam_mod_dir = "/root/imx6hub_demo",
	.status_file = "/run/securityd.status",
	.log_dir = "/root/imx6hub_demo/logs/securityd",
};

enum sd_state { ST_GUARD, ST_ALERT, ST_TRACK_READY, ST_STOPPING };

static const char *state_name(enum sd_state st)
{
	switch (st) {
	case ST_GUARD:       return "GUARD";
	case ST_ALERT:       return "ALERT";
	case ST_TRACK_READY: return "TRACK_READY";
	case ST_STOPPING:    return "STOPPING";
	}
	return "?";
}

/*
 * 一个受监督的子进程。字段刻意分成三块：
 *   进程本体（pid/fd/logfd/running/exit_code）、
 *   行重组缓冲（line/len）、
 *   以及它自己那门"语言"的快照（video 的 [stats] / audio 的 RESULT）。
 * 快照放在这里而不是散在 struct sd 上，是因为它天然按子进程归属；
 * 混在一起会看不出"这个数是谁报的"。
 */
struct child {
	const char *name;
	pid_t pid;
	int fd;
	int logfd;
	int running;
	int exit_code;
	uint64_t started_ms;
	char line[SD_LINE_MAX];
	size_t len;

	/* video_link 的 [stats] 快照 */
	int v_have;
	unsigned long long v_cap, v_in, v_out, v_ovr, v_send, v_err;

	/* audio_capture 的快照 */
	int a_have_result;
	unsigned long long a_frames, a_xrun;
	char wav_path[SD_PATH_MAX];
};

struct sd {
	enum sd_state state;
	uint64_t t_start;
	int quit;

	struct child video, audio;

	int ep, sfd, tfd;
	int input_fd;

	FILE *logfp;
	FILE *caplog;

	/* AP3216C */
	char i2c_dir[SD_PATH_MAX];
	int als_raw;
	unsigned long long ps_near, ps_far, ps_irq;
	uint64_t last_near_ms, last_edge_ms;

	/* ICM20608 */
	char iio_dir[SD_PATH_MAX];
	double vib_scale;      /* **g per LSB**；读不到则 0，vib_mg 退化为原始计数 */
	int vib_mg;            /* 逐轴帧间最大变化量，mg（不是合矢量幅值，见 vib_check） */
	long vib_px, vib_py, vib_pz;  /* 上一拍的原始计数 */
	int vib_have_prev;     /* 0 = 还没有上一拍，本拍不出差分 */
	unsigned vib_ev;       /* 累计超阈值事件数 */
	uint64_t vib_burst_until;  /* 高频采样保持到这一刻 */
	int ev_fd;             /* 事件出口套接字，-1 = 没开 */

	/* 会话统计 */
	unsigned alerts, recorded, video_ok, video_reload, near_ev, far_ev;
	uint64_t alert_start_ms;
	uint64_t stop_kill_audio_ms, stop_deadline_ms;

	/* 本轮（一次告警）状态 */
	unsigned cam_fail_count;
	int video_ready;          /* 视频此刻正在发包 */
	int alert_video_seen;     /* 本轮告警内是否见过视频发包 */

	/* 节拍 */
	uint64_t next_als_ms, next_vib_ms, next_stats_ms;
	uint64_t next_ev_retry_ms;
	uint64_t next_input_retry_ms;
	uint64_t next_audio_retry_ms;

	/* 退化标志：正交，不建独立状态 */
	int deg_sensor, deg_camera, deg_audio;
	int camera_degraded_told;
};

static struct sd s;

/* ---------------------------------------------------------------- 小工具 */

static uint64_t now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

static uint64_t uptime_ms(void) { return now_ms() - s.t_start; }

/* securityd 自己的行：stdout 一份（给操作者与脚本），log_dir 一份（供事后逐秒对账）。 */
static void sd_emit(const char *fmt, ...)
{
	char buf[2048];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	fputs(buf, stdout);
	fputc('\n', stdout);
	fflush(stdout);
	if (s.logfp) {
		fprintf(s.logfp, "[%8llums] %s\n",
			(unsigned long long)uptime_ms(), buf);
		fflush(s.logfp);
	}
}

static void warnx_(const char *fmt, ...)
{
	char buf[1024];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	fprintf(stderr, "securityd: WARN %s\n", buf);
	if (s.logfp) {
		fprintf(s.logfp, "[%8llums] WARN %s\n",
			(unsigned long long)uptime_ms(), buf);
		fflush(s.logfp);
	}
}

static int mkdir_p(const char *path)
{
	char tmp[SD_PATH_MAX];
	char *p;
	size_t n = strlen(path);

	if (n == 0 || n >= sizeof(tmp))
		return -1;
	memcpy(tmp, path, n + 1);
	for (p = tmp + 1; *p; p++) {
		if (*p != '/')
			continue;
		*p = '\0';
		if (mkdir(tmp, 0755) != 0 && errno != EEXIST)
			return -1;
		*p = '/';
	}
	if (mkdir(tmp, 0755) != 0 && errno != EEXIST)
		return -1;
	return 0;
}

static int read_str(const char *path, char *out, size_t n)
{
	int fd = open(path, O_RDONLY);
	ssize_t r;

	if (fd < 0)
		return -1;
	r = read(fd, out, n - 1);
	close(fd);
	if (r <= 0)
		return -1;
	out[r] = '\0';
	return 0;
}

static int read_int(const char *path, long *out)
{
	char buf[64];
	char *end;

	if (read_str(path, buf, sizeof(buf)) < 0)
		return -1;
	errno = 0;
	*out = strtol(buf, &end, 10);
	if (end == buf)
		return -1;
	return 0;
}

static double read_double(const char *path)
{
	char buf[64];
	char *end;
	double v;

	if (read_str(path, buf, sizeof(buf)) < 0)
		return -1.0;
	errno = 0;
	v = strtod(buf, &end);
	if (end == buf)
		return -1.0;
	return v;
}

/*
 * 在字符串里找 "<label>=<整数>"。label 里带 UTF-8，与被解析方用同一份字面量，
 * 所以不会因为编码或空格分叉；label 之间互不为前缀（"发包" 后面跟的是 "="，
 * 而 "发包错" 后面跟的是 "错"），所以不会串。
 */
static int lbl_u64(const char *line, const char *label, unsigned long long *out)
{
	const char *p = strstr(line, label);
	char *end;
	unsigned long long v;

	if (!p)
		return -1;
	p += strlen(label);
	if (*p != '=')
		return -1;
	errno = 0;
	v = strtoull(p + 1, &end, 10);
	if (end == p + 1)
		return -1;
	*out = v;
	return 0;
}

/*
 * execv 不查 PATH，也不认"没有斜杠的裸名字"。默认值写成 ./video_link 是有意的，
 * 但用户可能传 video_link，这里统一补 ./，把失败从 exec 报 ENOENT 提前成路径明确。
 */
static const char *resolve_bin(const char *bin)
{
	static char buf[SD_PATH_MAX];

	if (strchr(bin, '/'))
		return bin;
	snprintf(buf, sizeof(buf), "./%s", bin);
	return buf;
}

static FILE *open_logfile(const char *name)
{
	char path[SD_PATH_MAX];

	snprintf(path, sizeof(path), "%s/%s", g.log_dir, name);
	/* 每次启动截断：一次会话一个文件，免得混进上一轮的行。 */
	return fopen(path, "w");
}

/* ---------------------------------------------------------------- 子进程 */

static int set_nonblock(int fd)
{
	int fl = fcntl(fd, F_GETFL, 0);

	if (fl < 0)
		return -1;
	return fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

static void child_reset(struct child *c, const char *name)
{
	memset(c, 0, sizeof(*c));
	c->name = name;
	c->pid = -1;
	c->fd = -1;
	c->logfd = -1;
	c->exit_code = -1;
}

/*
 * fork 出来的子进程必须在 exec 前把信号掩码清空。
 *
 * 本程序为用 signalfd 把 SIGINT/SIGTERM/SIGCHLD/SIGUSR1 全 SIG_BLOCK 掉了
 * （见 main 里的 sigprocmask），而 fork 继承掩码、exec 也不清除它 —— 子进程
 * 起来时 SIGTERM 是**被屏蔽**的。子进程自己 sigaction 装 handler 也没用：
 * 信号递不进去，handler 永远不跑。实测表现是 audio_capture 收到 SIGTERM 后
 * 整整 4 s 毫无反应，被兜底 SIGKILL，WAV 头永远不回填（44 字节占位留在文件头，
 * 落盘的 5~11 MB PCM 无法播放）。清空掩码后同一个子进程 18 ms 就干净退出。
 */
static void child_unblock(void)
{
	sigset_t empty;

	sigemptyset(&empty);
	sigprocmask(SIG_SETMASK, &empty, NULL);
}

/*
 * 把子进程的 stdout+stderr 接到同一条管道：这样原始行只经过一次，
 * 既能原样落日志，又能顺手解析出 STATS 需要的那几个数。
 */
static int child_spawn(struct sd *d, struct child *c, char *const argv[])
{
	int pfd[2];
	pid_t pid;
	char logname[64];

	if (pipe(pfd) < 0) {
		warnx_("pipe 失败: %s", strerror(errno));
		return -1;
	}
	pid = fork();
	if (pid < 0) {
		warnx_("fork 失败: %s", strerror(errno));
		close(pfd[0]);
		close(pfd[1]);
		return -1;
	}
	if (pid == 0) {
		close(pfd[0]);
		if (dup2(pfd[1], STDOUT_FILENO) < 0 ||
		    dup2(pfd[1], STDERR_FILENO) < 0)
			_exit(126);
		if (pfd[1] > STDERR_FILENO)
			close(pfd[1]);
		/* 不 setsid：留在同一进程组，退出时能统一收尾。 */
		child_unblock();
		execv(argv[0], argv);
		fprintf(stderr, "securityd: exec %s 失败: %s\n",
			argv[0], strerror(errno));
		_exit(127);
	}

	close(pfd[1]);
	set_nonblock(pfd[0]);
	if (c->logfd >= 0)
		close(c->logfd);
	snprintf(logname, sizeof(logname), "%s.log", c->name);
	{
		FILE *f = open_logfile(logname);

		c->logfd = f ? dup(fileno(f)) : -1;
		if (f)
			fclose(f);
		if (c->logfd >= 0)
			set_nonblock(c->logfd);   /* 日志写不动不该拖住主循环 */
	}
	c->pid = pid;
	c->fd = pfd[0];
	c->running = 1;
	c->exit_code = -1;
	c->len = 0;
	c->started_ms = now_ms();
	c->v_have = 0;
	c->a_have_result = 0;

	{
		struct epoll_event ev;

		memset(&ev, 0, sizeof(ev));
		ev.events = EPOLLIN;
		ev.data.ptr = &c->fd;
		if (epoll_ctl(d->ep, EPOLL_CTL_ADD, c->fd, &ev) != 0)
			warnx_("epoll ADD %s 失败: %s", c->name, strerror(errno));
	}
	sd_emit("securityd: INFO  已启动 %s pid=%d", c->name, (int)pid);
	return 0;
}

static void child_signal(struct child *c, int sig)
{
	if (c->running && c->pid > 0)
		kill(c->pid, sig);
}

/*
 * 阻塞跑一条外部命令（rmmod/insmod/热身录音用），输出进 s.caplog。
 * 超时就 SIGKILL。返回值：子进程退出码；-1 = 起不来或超时。
 */
static int run_cmd(char *const argv[], unsigned timeout_ms)
{
	pid_t pid;
	uint64_t deadline;
	int st;

	pid = fork();
	if (pid < 0)
		return -1;
	if (pid == 0) {
		if (s.caplog) {
			dup2(fileno(s.caplog), STDOUT_FILENO);
			dup2(fileno(s.caplog), STDERR_FILENO);
		}
		child_unblock();
		execv(argv[0], argv);
		_exit(127);
	}

	deadline = now_ms() + timeout_ms;
	for (;;) {
		pid_t r = waitpid(pid, &st, WNOHANG);

		if (r == pid) {
			if (WIFEXITED(st))
				return WEXITSTATUS(st);
			return 128 + (WIFSIGNALED(st) ? WTERMSIG(st) : 0);
		}
		if (r < 0 && errno != EINTR)
			return -1;
		if (now_ms() >= deadline) {
			kill(pid, SIGKILL);
			waitpid(pid, &st, 0);
			return -1;
		}
		usleep(20 * 1000);
	}
}

/* ---------------------------------------------------------------- 设备定位 */

static int find_input_fd(const char *want)
{
	char path[64], name[128];
	int i;

	for (i = 0; i < 32; i++) {
		int fd;

		snprintf(path, sizeof(path), "/dev/input/event%d", i);
		fd = open(path, O_RDONLY | O_NONBLOCK);
		if (fd < 0)
			continue;
		memset(name, 0, sizeof(name));
		if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) >= 0 &&
		    strcmp(name, want) == 0) {
			sd_emit("securityd: INFO  Input 设备 %s name=%s", path, name);
			return fd;
		}
		close(fd);
	}
	return -1;
}

static int find_i2c_dir(char *out, size_t n)
{
	DIR *d = opendir("/sys/bus/i2c/devices");
	struct dirent *e;

	if (!d)
		return -1;
	while ((e = readdir(d)) != NULL) {
		size_t len = strlen(e->d_name);

		if (len >= 5 && strcmp(e->d_name + len - 5, "-001e") == 0) {
			snprintf(out, n, "/sys/bus/i2c/devices/%s", e->d_name);
			closedir(d);
			return 0;
		}
	}
	closedir(d);
	return -1;
}

static int find_iio_dir(const char *want, char *out, size_t n)
{
	DIR *d = opendir("/sys/bus/iio/devices");
	struct dirent *e;

	if (!d)
		return -1;
	while ((e = readdir(d)) != NULL) {
		char p[SD_PATH_MAX], buf[128];

		if (strncmp(e->d_name, "iio:device", 10) != 0)
			continue;
		snprintf(p, sizeof(p), "/sys/bus/iio/devices/%s/name", e->d_name);
		if (read_str(p, buf, sizeof(buf)) < 0)
			continue;
		if (strncmp(buf, want, strlen(want)) != 0)
			continue;
		snprintf(out, n, "/sys/bus/iio/devices/%s", e->d_name);
		closedir(d);
		return 0;
	}
	closedir(d);
	return -1;
}

/* ---------------------------------------------------------------- 采样 */

static void sample_ap3216c(void)
{
	char p[SD_PATH_MAX], buf[128];
	long v;
	unsigned long long n;

	if (s.i2c_dir[0] == '\0' && find_i2c_dir(s.i2c_dir, sizeof(s.i2c_dir)) < 0)
		return;

	snprintf(p, sizeof(p), "%s/ambient_light/als_raw_data", s.i2c_dir);
	if (read_int(p, &v) == 0)
		s.als_raw = (int)v;

	snprintf(p, sizeof(p), "%s/proximity/prox_counts", s.i2c_dir);
	if (read_str(p, buf, sizeof(buf)) == 0) {
		if (lbl_u64(buf, "near", &n) == 0)
			s.ps_near = n;
		if (lbl_u64(buf, "far", &n) == 0)
			s.ps_far = n;
		if (lbl_u64(buf, "irq", &n) == 0)
			s.ps_irq = n;
	}
}

/*
 * ICM20608 振动：设备树里没有 interrupts 属性，本版只能轮询。
 * （不是"没中断脚"——`6D_INT` 已引至 GPIO1_IO10，接入见 Phase 7。）
 * 注意复位后第 1 次读是垃圾值，丢弃动作在 probe_sensors 里做。
 */
static void sample_icm20608(void)
{
	char p[SD_PATH_MAX];
	long x, y, z, dx, dy, dz, d;

	if (s.iio_dir[0] == '\0')
		return;
	snprintf(p, sizeof(p), "%s/in_accel_x_raw", s.iio_dir);
	if (read_int(p, &x) < 0)
		return;
	snprintf(p, sizeof(p), "%s/in_accel_y_raw", s.iio_dir);
	if (read_int(p, &y) < 0)
		return;
	snprintf(p, sizeof(p), "%s/in_accel_z_raw", s.iio_dir);
	if (read_int(p, &z) < 0)
		return;

	/*
	 * 取**逐轴帧间差分**，不是合矢量幅值。理由见 vib_check。
	 * 第一拍没有上一拍可减，记 0 并把它存成基线。
	 */
	if (!s.vib_have_prev) {
		s.vib_mg = 0;
	} else {
		dx = x - s.vib_px;
		dy = y - s.vib_py;
		dz = z - s.vib_pz;
		if (dx < 0) dx = -dx;
		if (dy < 0) dy = -dy;
		if (dz < 0) dz = -dz;
		d = dx > dy ? dx : dy;
		if (dz > d) d = dz;
		/*
		 * 驱动的 in_accel_scale 是 **g/LSB**（icm20608_iio.c：以 ±2g 为例
		 * 4/2^16=0.000061035，满量程按 g 算），不是 IIO 惯常的 m/s^2/LSB。
		 * 所以只乘 scale 再换 mg，**不能**再除 9.80665 —— 除下去读数会小 9.8 倍。
		 */
		s.vib_mg = (s.vib_scale > 0.0)
			? (int)(d * s.vib_scale * 1000.0)
			: (int)d;
	}
	s.vib_px = x; s.vib_py = y; s.vib_pz = z;
	s.vib_have_prev = 1;
}

/* ---------------------------------------------------------------- 振动事件（软件阈值，非中断） */

/*
 * 打开事件出口。失败不致命：daemon 没起来的时候 securityd 照样要能跑，
 * 所以只退化（ev_fd = -1）并留一条日志，绝不 exit。
 * 特意用 SOCK_DGRAM 且不 connect：对端不在时 sendto 只丢这一个事件（ECONNREFUSED/ENOENT），
 * 不会像 SOCK_STREAM 那样把本进程的写路径卡住或者产生 SIGPIPE。
 */
static void event_open(void)
{
	struct sockaddr_un sa;

	s.ev_fd = -1;
	if (g.event_sock == NULL || g.event_sock[0] == '\0')
		return;
	if (strlen(g.event_sock) >= sizeof(sa.sun_path)) {
		warnx_("--event-sock 路径过长，事件出口关闭");
		return;
	}

	s.ev_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (s.ev_fd < 0) {
		warnx_("socket(AF_UNIX) 失败(%s)，事件出口关闭", strerror(errno));
		return;
	}

	memset(&sa, 0, sizeof(sa));
	sa.sun_family = AF_UNIX;
	strncpy(sa.sun_path, g.event_sock, sizeof(sa.sun_path) - 1);
	if (connect(s.ev_fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		/* 对端 socket 还不存在是常态（daemon 可能后起），先关掉，下个节拍再试。 */
		close(s.ev_fd);
		s.ev_fd = -1;
		return;
	}
	sd_emit("securityd: INFO  事件出口已连接 %s", g.event_sock);
}

static void event_send(uint16_t kind)
{
	struct ptp_sekev e;
	uint8_t buf[PTP_SEKEV_SIZE];

	if (s.ev_fd < 0)
		return;

	e.magic = PTP_SEKEV_MAGIC;
	e.version = PTP_SEKEV_VERSION;
	e.kind = kind;
	e.seq = s.vib_ev;             /* 调用方已自增，从 1 开始 */
	e.t_ms = (uint32_t)now_ms();
	ptp_sekev_pack(&e, buf);
	if (send(s.ev_fd, buf, sizeof(buf), MSG_DONTWAIT) < 0) {
		warnx_("事件发送失败(%s)", strerror(errno));
		close(s.ev_fd);
		s.ev_fd = -1;         /* 下一次 on_tick 会重连 */
	}
}

/*
 * 采样完一拍后判一次阈值。判据是**逐轴帧间最大变化量 ≥ 阈值**（mg）。
 *
 * 2026-10-08 换成差分，之前是"合矢量幅值 ≥ 阈值"。换的理由是原式有两处硬伤：
 *
 *   1. **单位差 9.8 倍。** 驱动的 in_accel_scale 是 g/LSB，原式当成 m/s²
 *      又除了一次 9.80665。实测静止时 raw_z=2037、scale=0.000488281
 *      → 物理上就是 1 g，可原式算出 vib_mg=101（它自己的注释写着"静止时应接近 1000"）。
 *      连带后果：--vib-thr-mg 300 实际要约 3 g 才触发，基本不灵。
 *   2. **合矢量有 1 g 直流分量。** 静止时 |a| 恒为 1000 mg 且与姿态无关，
 *      拿它比阈值等于在跟重力本身比：阈值设 300 会因为 1000>300 而**一直触发**；
 *      要绕开就得设成"|a| ≥ 1000+阈值"。而且横向晃动对 |a| 是二阶效应
 *      （|d|=300 mg 侧向只能把 |a| 抬到 1044，抬幅仅 44 mg），灵敏度很差；
 *      轴向推压还会让 |a| 变小，**根本抓不到**。
 *
 * 差分的性质正好对上：静止时差为 0（与姿态、量程都无关，不需要基线标定），
 * 任何方向的晃动都在**对应那一轴**上线性体现，轴向侧向一样灵。
 * 参考项目 IMX6ULL-Safe-Alarm 用的也是这个（逐轴 curr-prev 比阈值）。
 *
 * 已知代价：基础采样间隔（--vib-poll-ms）比振动本身慢的话会漏掉短促的一下。
 * 所以默认值从 500 ms 收到 100 ms（= 默认 tick），要更细就把 --tick-ms 一起调小。
 */
static void vib_check(uint64_t now)
{
	if (g.vib_thr_mg == 0)
		return;
	if (s.vib_mg < (int)g.vib_thr_mg)
		return;

	/*
	 * 阈值之上不再重复发事件，只续高频窗口。否则一次持续晃动会在每个
	 * 20 ms 采样点都发一个事件，把 PC 侧的巡检反复打断。
	 */
	if (now < s.vib_burst_until) {
		s.vib_burst_until = now + g.vib_burst_hold_ms;
		return;
	}

	s.vib_burst_until = now + g.vib_burst_hold_ms;
	s.vib_ev++;
	sd_emit("securityd: EVENT vib seq=%u vib_mg=%d thr=%u（软件阈值，非中断）",
		s.vib_ev, s.vib_mg, g.vib_thr_mg);
	event_send(PTP_SEKEV_KIND_VIB);
}

/* ---------------------------------------------------------------- 子进程输出解析 */

static void video_on_stats(struct child *c, const char *line)
{
	unsigned long long v;

	c->v_have = 1;
	if (lbl_u64(line, "采集", &v) == 0)
		c->v_cap = v;
	if (lbl_u64(line, "入环", &v) == 0)
		c->v_in = v;
	if (lbl_u64(line, "出环", &v) == 0)
		c->v_out = v;
	if (lbl_u64(line, "覆盖丢", &v) == 0)
		c->v_ovr = v;
	if (lbl_u64(line, "发包", &v) == 0)
		c->v_send = v;
	/* "发包错" 与上面的 "发包" 不是前缀关系（发 包 错 vs 发 包 =），不会串。 */
	if (lbl_u64(line, "发包错", &v) == 0)
		c->v_err = v;
}

static void audio_on_result(struct child *c, const char *line)
{
	unsigned long long v;

	c->a_have_result = 1;
	/* label **不带 '='**：lbl_u64() 自己会吃掉后面那个 '='（见其实现里
	 * `p += strlen(label); if (*p != '=') return -1;`）。原先这两行传的是
	 * "frames=" / "xrun="，于是它在找 "frames==" —— 永远匹配不上，
	 * a_frames 恒为 0。后果不只是 recorded 报 0：每次告警正常收尾时还会
	 * 打一条假警报 `DEGRADED audio —— 录音 frames=0，文件只有 WAV 头`。
	 * video_on_stats 那六个调用点都是不带 '=' 的，这里跟它对齐。 */
	if (lbl_u64(line, "frames", &v) == 0)
		c->a_frames = v;
	if (lbl_u64(line, "xrun", &v) == 0)
		c->a_xrun = v;
}

static void on_child_line(struct child *c, const char *line)
{
	if (strcmp(c->name, "video_link") == 0) {
		if (strncmp(line, "[stats] ", 8) == 0)
			video_on_stats(c, line);
	} else if (strcmp(c->name, "audio_capture") == 0) {
		if (strstr(line, "audio: RESULT ") != NULL)
			audio_on_result(c, line);
	}
}

/* 把管道里已到达的数据读走；返回 1 表示 EOF（对端已关闭），0 表示还在跑。 */
static int child_drain(struct child *c)
{
	char buf[1024];

	/* fd 已被别处关掉时按"已读完"处理。主循环的 epoll 那一路读到 EOF 会顺手
	 * 关掉 fd，之后 handle_sigchld() 再抽一次就是 EBADF —— 数据其实已经读干净
	 * 了，只会在日志里留一条吓人的 `读 ... 管道失败: Bad file descriptor`。 */
	if (c->fd < 0)
		return 1;

	for (;;) {
		ssize_t n = read(c->fd, buf, sizeof(buf));

		if (n > 0) {
			ssize_t off = 0;

			while (c->logfd >= 0 && off < n) {
				ssize_t w = write(c->logfd, buf + off,
						  (size_t)(n - off));

				if (w > 0) {
					off += w;
					continue;
				}
				if (w < 0 && errno == EINTR)
					continue;
				break;   /* 日志写不动就放弃这一段，不影响解析 */
			}
			for (ssize_t i = 0; i < n; i++) {
				char ch = buf[i];

				if (ch == '\r')
					continue;
				if (ch == '\n') {
					c->line[c->len] = '\0';
					if (c->len)
						on_child_line(c, c->line);
					c->len = 0;
					continue;
				}
				if (c->len < SD_LINE_MAX - 1)
					c->line[c->len++] = ch;
			}
			continue;
		}
		if (n == 0)
			return 1;
		if (errno == EAGAIN || errno == EWOULDBLOCK)
			return 0;
		if (errno == EINTR)
			continue;
		warnx_("读 %s 管道失败: %s", c->name, strerror(errno));
		return 1;
	}
}

static void child_close_pipe(struct sd *d, struct child *c)
{
	if (c->fd < 0)
		return;
	/* 先 DEL 再 close：epoll 里可能还有一条已排队的该 fd 事件。 */
	epoll_ctl(d->ep, EPOLL_CTL_DEL, c->fd, NULL);
	close(c->fd);
	c->fd = -1;
}

/* ---------------------------------------------------------------- 摄像头重载 */

/*
 * 顺序不可颠倒，理由见 scripts/lib/camera_reload.sh：持有关系是运行时的
 * （mx6s_capture 注册了 ov5640 这个 subdev 并握住它的引用），mx6s_capture 不先卸，
 * rmmod ov5640_camera 必定 in use。卸载顺序**不是**加载顺序的逆序。
 */
static int camera_reload(struct sd *d)
{
	char fixed[SD_PATH_MAX], ov[SD_PATH_MAX];
	unsigned i;
	int rc;

	snprintf(fixed, sizeof(fixed), "%s/mx6s_capture_fixed.ko", g.cam_mod_dir);
	snprintf(ov, sizeof(ov), "%s/ov5640_camera.ko", g.cam_mod_dir);

	/* 1. 先归还 /dev/video1：不归还就 rmmod 是必败的一步。 */
	if (d->video.running || d->video.pid > 0) {
		child_signal(&d->video, SIGTERM);
		for (i = 0; i < 60 && d->video.running; i++)
			usleep(100 * 1000);
		if (d->video.running) {
			child_signal(&d->video, SIGKILL);
			for (i = 0; i < 20 && d->video.running; i++)
				usleep(50 * 1000);
		}
		child_close_pipe(d, &d->video);
		if (d->video.running)
			warnx_("重载前 video_link 仍未回收，继续尝试（rmmod 可能 in use）");
	}
	if (s.caplog) {
		fprintf(s.caplog, "\n=== camera_reload @%llu ms ===\n",
			(unsigned long long)uptime_ms());
		fflush(s.caplog);
	}

	{
		char *a_unload[] = { "/sbin/rmmod", "mx6s_capture", NULL };
		char *a_unload2[] = { "/sbin/rmmod", "ov5640_camera", NULL };
		char *a_load[] = { "/sbin/insmod", fixed, NULL };
		char *a_load2[] = { "/sbin/insmod", ov, NULL };

		sd_emit("securityd: INFO  摄像头重载：有序归还模块");
		rc = run_cmd(a_unload, 30000);
		if (rc != 0)
			warnx_("rmmod mx6s_capture rc=%d（本来没加载时也会这样）", rc);
		rc = run_cmd(a_unload2, 30000);
		if (rc != 0)
			warnx_("rmmod ov5640_camera rc=%d", rc);

		rc = run_cmd(a_load, 30000);
		if (rc != 0) {
			warnx_("insmod %s rc=%d", fixed, rc);
			return -1;
		}
		rc = run_cmd(a_load2, 30000);
		if (rc != 0) {
			warnx_("insmod %s rc=%d", ov, rc);
			return -1;
		}
	}

	/* 传感器 probe 后 I2C 初始化偶发未收敛，静置是既有验收脚本里的固定动作。 */
	usleep(2000 * 1000);
	if (access(g.video_dev, F_OK) != 0) {
		warnx_("重载后 %s 仍不存在", g.video_dev);
		return -1;
	}
	return 0;
}

/* ---------------------------------------------------------------- 子进程启停 */

static int video_start(struct sd *d)
{
	char dst[128], w[16], h[16], f[16], sl[16];
	char *argv[] = {
		(char *)resolve_bin(g.video_bin), "--dev", (char *)g.video_dev,
		"--dst", dst, "--width", w, "--height", h, "--fps", f,
		"--slots", sl, "--stats-ms", (char *)"1000", NULL
	};

	snprintf(dst, sizeof(dst), "%s", g.video_dst);
	snprintf(w, sizeof(w), "%u", g.video_w);
	snprintf(h, sizeof(h), "%u", g.video_h);
	snprintf(f, sizeof(f), "%u", g.video_fps);
	snprintf(sl, sizeof(sl), "%u", g.video_slots);

	d->video_ready = 0;
	return child_spawn(d, &d->video, argv);
}

static int audio_start(struct sd *d)
{
	char out[SD_PATH_MAX], stamp[32];
	char *argv[] = {
		(char *)resolve_bin(g.audio_bin),
		"--output", out, "--duration", (char *)"0", NULL
	};
	time_t t = time(NULL);
	struct tm tm;

	/*
	 * 用 --output 自己拼好全过程唯一的名字，而不是让程序按时间戳自己生成：
	 * 那样 securityd 事后要解析一行带中文后缀的输出才能知道文件在哪。
	 * 命名格式与 Phase 5 的默认规则一致，目录扫描类的脚本不受影响。
	 */
	localtime_r(&t, &tm);
	strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);
	if (mkdir_p(g.records_dir) != 0)
		warnx_("建目录 %s 失败: %s", g.records_dir, strerror(errno));
	snprintf(out, sizeof(out), "%s/%s_%s.wav", g.records_dir, stamp,
		 g.audio_tag);
	snprintf(d->audio.wav_path, sizeof(d->audio.wav_path), "%s", out);
	d->audio.a_frames = 0;
	d->audio.a_xrun = 0;

	return child_spawn(d, &d->audio, argv);
}

/* ---------------------------------------------------------------- 状态机 */

static void deg_string(char *out, size_t n)
{
	int first = 1;

	out[0] = '\0';
#define ADD(f, name)                                                     \
	do {                                                             \
		if (f) {                                                 \
			snprintf(out + strlen(out), n - strlen(out),      \
				 "%s%s", first ? "" : ",", name);        \
			first = 0;                                       \
		}                                                        \
	} while (0)
	ADD(s.deg_sensor, "no_sensor");
	ADD(s.deg_camera, "camera");
	ADD(s.deg_audio, "audio");
#undef ADD
	if (first)
		snprintf(out, n, "none");
}

static void stats_line(char *buf, size_t n)
{
	char deg[64];
	char audio_state[16];

	deg_string(deg, sizeof(deg));
	if (s.audio.running)
		snprintf(audio_state, sizeof(audio_state), "rec");
	else
		snprintf(audio_state, sizeof(audio_state), "idle");

	snprintf(buf, n,
		 "securityd: STATS ts=%llu state=%s up=%.1f near=%u far=%u "
		 "ps_irq=%llu als=%d vib_mg=%d vib_ev=%u vib_thr=%u "
		 "video=%s vin=%llu vout=%llu vovr=%llu vsend=%llu verr=%llu "
		 "audio=%s arec_s=%.1f cam_reload=%u deg=%s",
		 (unsigned long long)uptime_ms(), state_name(s.state),
		 uptime_ms() / 1000.0,
		 s.near_ev, s.far_ev, s.ps_irq, s.als_raw, s.vib_mg,
		 s.vib_ev, g.vib_thr_mg,
		 !s.video.running ? "down" : (s.video_ready ? "up" : "init"),
		 s.video.v_in, s.video.v_out, s.video.v_ovr,
		 s.video.v_send, s.video.v_err,
		 audio_state,
		 s.audio.running ? (now_ms() - s.audio.started_ms) / 1000.0 : 0.0,
		 s.video_reload, deg);
}

/*
 * 状态文件与 STATS 同内容，只去掉前缀，供别的脚本无 IPC 读取。
 * 写临时文件再 rename：读的人要么看到旧整行、要么看到新整行，不会看到半行。
 * （绝不在原文件上就地改写——一次截断就什么都没有了。）
 */
static void status_write(void)
{
	char line[2048], tmp[SD_PATH_MAX];
	const char *body;
	FILE *f;

	stats_line(line, sizeof(line));
	body = strstr(line, " ts=");
	if (!body)
		return;
	body++;

	snprintf(tmp, sizeof(tmp), "%s.tmp", g.status_file);
	f = fopen(tmp, "w");
	if (!f)
		return;
	fprintf(f, "%s\n", body);
	fclose(f);
	if (rename(tmp, g.status_file) != 0)
		unlink(tmp);
}

static void enter_state(struct sd *d, enum sd_state to, const char *reason)
{
	if (d->state == to)
		return;
	sd_emit("securityd: EVT   ts=%llu from=%s to=%s reason=%s ps=%d",
		(unsigned long long)uptime_ms(), state_name(d->state),
		state_name(to), reason, s.als_raw);
	d->state = to;
}

static void alert_begin(struct sd *d)
{
	d->alerts++;
	d->alert_start_ms = now_ms();
	d->cam_fail_count = 0;
	/*
	 * 视频常开，进入告警时它多半已经在发包了 —— 那就已经是就绪状态，
	 * 不能清零重等（清零只会把 TRACK_READY 平白推后一个 tick，
	 * 还会让本轮 video_ok 记成 0）。留给 tick_video_health 去维护。
	 */
	d->alert_video_seen = d->video_ready;

	if (!g.dry_run) {
		/* 视频常开时这里多半已经在跑；不在跑（比如刚被重载掐掉）就补上。 */
		if (!d->video.running && video_start(d) != 0)
			d->deg_camera = 1;
		if (!d->audio.running && audio_start(d) != 0)
			d->deg_audio = 1;
	}
	enter_state(d, ST_ALERT, "prox_near");
}

static void stop_begin(struct sd *d, const char *reason)
{
	sd_emit("securityd: EVT   ts=%llu from=%s to=STOPPING reason=%s",
		(unsigned long long)uptime_ms(), state_name(d->state), reason);
	d->state = ST_STOPPING;
	if (d->alert_video_seen)
		d->video_ok++;
	/*
	 * 只收音频：视频是常开的，告警结束不等于摄像头该关 —— 掐掉它下一次
	 * 告警又要从头 probe /dev/video1，正是决策 #2 要避免的事。
	 * 音频必须等：WAV 头没回填就是 44 字节废文件，属不可恢复的数据损坏，
	 * 所以 SIGKILL 兜底给得晚（4 s），不是给得早。
	 */
	child_signal(&d->audio, SIGTERM);
	d->stop_kill_audio_ms = now_ms() + 4000;
	d->stop_deadline_ms = now_ms() + 12000;
}

/* ---------------------------------------------------------------- epoll 装配 */

static void ep_add(struct sd *d, int fd, void *ptr)
{
	struct epoll_event ev;

	memset(&ev, 0, sizeof(ev));
	ev.events = EPOLLIN;
	ev.data.ptr = ptr;
	if (epoll_ctl(d->ep, EPOLL_CTL_ADD, fd, &ev) != 0)
		warnx_("epoll ADD fd=%d 失败: %s", fd, strerror(errno));
}

static void input_attach(struct sd *d)
{
	int fd = find_input_fd(g.input_name);

	if (fd < 0) {
		if (!d->deg_sensor) {
			d->deg_sensor = 1;
			sd_emit("securityd: DEGRADED no_sensor —— 找不到 Input 设备 "
				"name=%s，ALERT 不会被触发。", g.input_name);
			sd_emit("securityd: INFO  排查：lsmod | grep ap3216c_input；"
				"并确认已执行 echo 0-001e > "
				"/sys/bus/i2c/drivers/ap3216c/unbind "
				"（每次开机 BSP 都会重新绑定，所以每次都要 unbind）");
		}
		return;
	}
	d->input_fd = fd;
	ep_add(d, fd, &d->input_fd);
	if (d->deg_sensor) {
		d->deg_sensor = 0;
		sd_emit("securityd: INFO  no_sensor 已解除（Input 设备出现）");
	}
}

/* ---------------------------------------------------------------- 探针与自检 */

/*
 * 启动自检。任何一项不通过都要**大声说**并置退化标志——
 * 静默停在 GUARD 是最糟的失败：看起来一切正常。
 */
static void probe_sensors(struct sd *d)
{
	long v;

	if (find_i2c_dir(d->i2c_dir, sizeof(d->i2c_dir)) == 0) {
		char p[SD_PATH_MAX];

		snprintf(p, sizeof(p), "%s/proximity/threshold_low", d->i2c_dir);
		if (read_int(p, &v) != 0)
			warnx_("读 threshold_low 失败（%s）", p);
		snprintf(p, sizeof(p), "%s/ambient_light/als_raw_data", d->i2c_dir);
		if (read_int(p, &v) != 0)
			warnx_("读 als_raw_data 失败（%s）", p);
	} else {
		warnx_("未找到 *-001e 的 I2C 设备目录（ap3216c_input 是否已 insmod？）");
	}

	if (find_iio_dir(g.iio_name, d->iio_dir, sizeof(d->iio_dir)) == 0) {
		char p[SD_PATH_MAX];

		snprintf(p, sizeof(p), "%s/in_accel_scale", d->iio_dir);
		d->vib_scale = read_double(p);
		if (d->vib_scale < 0.0) {
			d->vib_scale = 0.0;
			warnx_("未读到 in_accel_scale，vib_mg 退化为原始计数");
		}
		sample_icm20608();          /* 丢掉：复位后第一次读是垃圾值 */
		usleep(150 * 1000);
		sample_icm20608();
	} else {
		warnx_("未找到 IIO 设备 name=%s", g.iio_name);
	}
}

/*
 * 录音方向会进入持久坏状态（plan/phase5_audio_alsa.md §4.1），唯一恢复是重启板子；
 * 且重启后第一遍会擦边。所以开跑前先录一小段丢掉：
 * 它坏掉时要在**启动时**就知道，而不是等第一次真实告警才发现拿到空文件。
 * 本程序不自动重启板子，只置退化标志并保留视频。
 */
static void audio_warmup(void)
{
	char out[SD_PATH_MAX], sec[16], stamp[32];
	char *argv[] = {
		(char *)resolve_bin(g.audio_bin),
		"--output", out, "--duration", sec, "--quiet", NULL
	};
	time_t t = time(NULL);
	struct tm tm;
	int rc;

	if (g.audio_warmup_ms == 0 || g.dry_run)
		return;

	localtime_r(&t, &tm);
	strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);
	if (mkdir_p(g.records_dir) != 0)
		return;
	snprintf(out, sizeof(out), "%s/%s_warmup.wav", g.records_dir, stamp);
	snprintf(sec, sizeof(sec), "%u", (g.audio_warmup_ms + 999) / 1000u);

	rc = run_cmd(argv, g.audio_warmup_ms + 8000);
	if (rc == 0) {
		sd_emit("securityd: INFO  录音热身完成（%u ms），热身文件已删",
			g.audio_warmup_ms);
		s.deg_audio = 0;
	} else {
		s.deg_audio = 1;
		sd_emit("securityd: DEGRADED audio —— 热身录音 rc=%d。录音方向可能"
			"已进入持久坏状态，唯一恢复是 /sbin/reboot"
			"（绝对路径，/sbin 不在 PATH）。本程序不自动重启板子。", rc);
	}
	unlink(out);
}

/* ---------------------------------------------------------------- 事件处理 */

static void handle_input(struct sd *d)
{
	struct input_event ev;

	for (;;) {
		ssize_t n = read(d->input_fd, &ev, sizeof(ev));

		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK)
				return;
			if (errno == EINTR)
				continue;
			warnx_("读 Input 失败: %s", strerror(errno));
			return;
		}
		if (n != (ssize_t)sizeof(ev))
			return;
		if (ev.type != EV_SW || ev.code != SW_FRONT_PROXIMITY)
			continue;

		if (ev.value) {
			/*
			 * 驱动判 NEAR 已带迟滞、Input 层对 EV_SW 去重，边沿本身是干净的，
			 * 所以不做 N 帧确认（那会滤掉真实的快速靠近），只防一次靠近被报两回。
			 */
			if (now_ms() - d->last_edge_ms < g.near_guard_ms) {
				if (g.verbose)
					sd_emit("securityd: INFO  忽略 %u ms 内的重复 near",
						g.near_guard_ms);
				continue;
			}
			d->last_edge_ms = now_ms();
			d->last_near_ms = now_ms();
			d->near_ev++;
			if (d->state == ST_GUARD || d->state == ST_STOPPING)
				alert_begin(d);
			else
				sd_emit("securityd: INFO  告警期间新的 near，告警续期");
		} else {
			d->last_edge_ms = now_ms();
			d->far_ev++;
		}
	}
}

static void handle_sigchld(struct sd *d)
{
	for (;;) {
		int st;
		pid_t pid = waitpid(-1, &st, WNOHANG);
		struct child *c = NULL;

		if (pid <= 0)
			return;
		if (d->video.pid == pid)
			c = &d->video;
		else if (d->audio.pid == pid)
			c = &d->audio;
		if (!c)
			continue;

		c->running = 0;
		if (WIFEXITED(st))
			c->exit_code = WEXITSTATUS(st);
		else if (WIFSIGNALED(st))
			c->exit_code = 128 + WTERMSIG(st);

		/* 子进程退出前写的最后几行此刻还躺在管道里，其中就有 audio 的收尾行
		 * `audio: RESULT ... frames=N` —— 它是"录音到底落没落盘"唯一在带内的证据。
		 * **必须先抽干再关 fd**：child_close_pipe() 只 close 不读，丢了就是静默
		 * 哑火，a_frames 停在 0，于是 RESULT 报 recorded=0、连 `DEGRADED audio`
		 * 也打不出来 —— 录好了不报，录坏了也不报。
		 * final_stop() 这条路径必中：reap_until() 直接调本函数，抢在主循环读管道
		 * 之前就把 fd 关掉了。2026-10-07 两轮实测均 recorded=0 而 WAV 完好。
		 * 对端已死、写端全关，read 会一路读到 EOF，一次调用即可抽干。 */
		child_drain(c);
		sd_emit("securityd: INFO  %s 退出 pid=%d 退出码=%d",
			c->name, (int)pid, c->exit_code);
		child_close_pipe(d, c);

		if (strcmp(c->name, "video_link") == 0 && c->exit_code == 3)
			warnx_("video_link 报摄像头故障（退出码 3），将由健康检查触发重载");

		if (strcmp(c->name, "audio_capture") == 0) {
			if (c->a_frames > 0) {
				d->recorded++;
				sd_emit("securityd: INFO  录音落盘 %s frames=%llu xrun=%llu",
					c->wav_path, c->a_frames, c->a_xrun);
			}
			if (c->a_have_result && c->a_frames == 0)
				sd_emit("securityd: DEGRADED audio —— 录音 frames=0，"
					"文件只有 WAV 头");
			if ((d->state == ST_ALERT || d->state == ST_TRACK_READY) &&
			    c->exit_code != 0)
				d->deg_audio = 1;
		}
	}
}

static void handle_signal(struct sd *d)
{
	struct signalfd_siginfo si;

	for (;;) {
		ssize_t n = read(d->sfd, &si, sizeof(si));

		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK)
				return;
			if (errno == EINTR)
				continue;
			return;
		}
		if (n != (ssize_t)sizeof(si))
			return;

		switch (si.ssi_signo) {
		case SIGCHLD:
			handle_sigchld(d);
			break;
		case SIGUSR1: {
			char line[2048];

			stats_line(line, sizeof(line));
			sd_emit("%s", line);
			break;
		}
		case SIGINT:
		case SIGTERM:
			sd_emit("securityd: INFO  收到信号 %u，开始收尾", si.ssi_signo);
			d->quit = 1;
			return;
		default:
			break;
		}
	}
}

/* ---------------------------------------------------------------- 周期动作 */

static void tick_input_retry(struct sd *d)
{
	if (d->input_fd >= 0 || !d->deg_sensor)
		return;
	if (now_ms() < d->next_input_retry_ms)
		return;
	d->next_input_retry_ms = now_ms() + 5000;
	input_attach(d);
}

/*
 * 视频常开，所以健康检查不分状态、每 tick 都跑：GUARD 期间挂了同样要重载，
 * 否则人走过来那一刻才现抓摄像头，等于把"镜头就位"押在一次 probe 上。
 * 视频迟迟达不成就绪 → 判为摄像头故障，走有序重载。
 */
static void tick_video_health(struct sd *d)
{
	if (g.dry_run)
		return;

	/*
	 * 就绪判据只有一条：真的在发包。进程活着却一个包都没发是退出码
	 * 看不见的失效模式（事实 7），所以既不认退出码也不认进程存活。
	 */
	if (d->video.running && d->video.v_have && d->video.v_send > 0) {
		if (!d->video_ready)
			sd_emit("securityd: INFO  视频就绪：已发包 %llu",
				d->video.v_send);
		d->video_ready = 1;
		return;
	}

	if (!d->video.running) {
		if (d->cam_fail_count >= g.cam_attempts) {
			d->deg_camera = 1;
			if (!d->camera_degraded_told) {
				d->camera_degraded_told = 1;
				/* 耗尽重试：保留音频，绝不重启板子、绝不碰云台。 */
				sd_emit("securityd: DEGRADED camera —— 重载 %u 次仍失败。"
					"录音继续，云台未被下发过任何控制包，板子未重启。",
					g.cam_attempts);
			}
			return;
		}
		d->cam_fail_count++;
		d->video_reload++;
		sd_emit("securityd: INFO  摄像头重载第 %u/%u 次",
			d->cam_fail_count, g.cam_attempts);
		if (camera_reload(d) == 0) {
			d->deg_camera = 0;
			d->camera_degraded_told = 0;
			video_start(d);
		} else {
			d->deg_camera = 1;
		}
		return;
	}

	/* 进程还在，但一直没发出包：退出码看不见这种情况。 */
	if (!d->video_ready && now_ms() - d->video.started_ms > g.video_ready_ms) {
		warnx_("video_link 已跑 %u ms 仍无 [stats]/发包，判为摄像头故障",
		       g.video_ready_ms);
		child_signal(&d->video, SIGTERM);
	}
}

static void tick_fsm(struct sd *d)
{
	uint64_t now = now_ms();

	if (d->state == ST_ALERT || d->state == ST_TRACK_READY) {
		/*
		 * video_ready 由 tick_video_health 在任意状态维护；这里只把
		 * "本轮告警内确实见过视频在发包"记下来，供 stop 时计入 video_ok。
		 */
		if (d->video_ready)
			d->alert_video_seen = 1;

		/*
		 * 录音掉了就补上。两条路会走到这里：一是上一次告警正在 STOPPING
		 * 收尾时人又走近（alert_begin 看到录音还活着就没起新的，等它咽气
		 * 之后本轮就再也没有录音了）；二是录音中途自己挂了。没有这一步，
		 * 这两个洞都是静默的——告警在跑、视频在发，就是没有声音。
		 */
		if (!g.dry_run && !d->audio.running &&
		    now >= d->next_audio_retry_ms) {
			d->next_audio_retry_ms = now + 5000;
			if (audio_start(d) == 0) {
				d->deg_audio = 0;
				sd_emit("securityd: INFO  补起录音");
			} else {
				d->deg_audio = 1;
				warnx_("补起录音失败，5 s 后重试");
			}
		}
		/*
		 * 一个规则同时管 ALERT 与 TRACK_READY：从最近一次 near 起算。
		 * 比"只在 TRACK_READY 上判超时"更严——视频还没就绪而人已经走了，
		 * 同样要收尾，不然会一直录到 alert_max。
		 */
		if (now - d->last_near_ms > g.alert_hold_ms) {
			stop_begin(d, "alert_hold");
			return;
		}
		if (now - d->alert_start_ms > g.alert_max_ms) {
			stop_begin(d, "alert_max");
			return;
		}
		if (d->state == ST_ALERT && d->video_ready)
			enter_state(d, ST_TRACK_READY, "video_ready");
		return;
	}

	if (d->state == ST_STOPPING) {
		if (now >= d->stop_kill_audio_ms)
			child_signal(&d->audio, SIGKILL);
		if (!d->audio.running || now >= d->stop_deadline_ms) {
			if (d->audio.running)
				warnx_("录音收尾超时，未等到 WAV 头回填");
			child_close_pipe(d, &d->audio);
			d->audio.running = 0;
			/* video_ready 不动：视频一直在发包，它没参与这次收尾。 */
			sd_emit("securityd: INFO  告警结束，回到 GUARD");
			enter_state(d, ST_GUARD, "stopped");
		}
	}
}

static void on_tick(struct sd *d)
{
	uint64_t now = now_ms();

	tick_input_retry(d);
	tick_video_health(d);
	tick_fsm(d);

	if (now >= d->next_als_ms) {
		d->next_als_ms = now + g.als_poll_ms;
		sample_ap3216c();
	}
	if (now >= d->next_vib_ms) {
		/* 事件窗口内压到 vib_burst_ms；窗口外仍是 Phase 6 的 vib_poll_ms。 */
		d->next_vib_ms = now + (now < d->vib_burst_until ? g.vib_burst_ms
								 : g.vib_poll_ms);
		sample_icm20608();
		vib_check(now);
	}
	if (g.event_sock != NULL && g.event_sock[0] != '\0' && d->ev_fd < 0 &&
	    now >= d->next_ev_retry_ms) {
		/* 对端可能比我们晚起；低频重试，失败不打扰任何人。 */
		d->next_ev_retry_ms = now + 2000;
		event_open();
	}
	if (now >= d->next_stats_ms) {
		char line[2048];

		d->next_stats_ms = now + g.stats_ms;
		stats_line(line, sizeof(line));
		sd_emit("%s", line);
		status_write();
	}
	/* 云台不在这里，也不在别处：本程序不碰 /dev/pantilt0。 */
}

static void handle_timer(struct sd *d)
{
	uint64_t exp;

	if (read(d->tfd, &exp, sizeof(exp)) < 0 && errno != EAGAIN)
		return;
	on_tick(d);
}

/* ---------------------------------------------------------------- 收尾 */

static void reap_until(struct sd *d, struct child *c, uint64_t deadline)
{
	while (c->running && now_ms() < deadline) {
		handle_sigchld(d);
		if (c->running)
			usleep(20 * 1000);
	}
}

static void final_stop(struct sd *d)
{
	/* 1. 不再起新的采样/定时动作。 */
	sd_emit("securityd: INFO  收尾：停子进程");
	child_signal(&d->audio, SIGTERM);
	child_signal(&d->video, SIGTERM);

	/* 2. 先等 audio：WAV 头没回填就是 44 字节废文件，不可恢复。 */
	reap_until(d, &d->audio, now_ms() + 3000);
	if (d->audio.running) {
		warnx_("audio_capture 3 s 未收尾，SIGKILL（WAV 头可能未回填）");
		child_signal(&d->audio, SIGKILL);
		reap_until(d, &d->audio, now_ms() + 2000);
	}

	/* 3. 再等 video，它要归还 /dev/video1。 */
	reap_until(d, &d->video, now_ms() + 6000);
	if (d->video.running) {
		warnx_("video_link 6 s 未收尾，SIGKILL");
		child_signal(&d->video, SIGKILL);
		reap_until(d, &d->video, now_ms() + 2000);
	}
	child_close_pipe(d, &d->audio);
	child_close_pipe(d, &d->video);

	/* 4. 云台一律不动。 */
	sd_emit("securityd: INFO  未向云台下发任何控制包（保持位置是 daemon 的职责）");

	/* 5. 落盘结果核对：WAV 至少要有一个合法的 44 字节头。 */
	if (d->audio.wav_path[0]) {
		struct stat st;

		if (stat(d->audio.wav_path, &st) == 0)
			sd_emit("securityd: INFO  录音文件 %s 大小=%lld",
				d->audio.wav_path, (long long)st.st_size);
		else
			sd_emit("securityd: DEGRADED audio —— 录音文件 %s 不存在",
				d->audio.wav_path);
	}
}

/* ---------------------------------------------------------------- main */

static void usage(const char *p)
{
	fprintf(stderr,
"用法: %s [选项]\n"
"  --input-name NAME     Input 设备名（默认 %s）\n"
"  --iio-name NAME       IIO 设备名（默认 %s）\n"
"  --video-bin PATH      video_link 可执行文件（默认 %s）\n"
"  --video-dev PATH      V4L2 设备（默认 %s）\n"
"  --video-dst A:P       视频目的地址（默认 %s）\n"
"  --video-size WxH      采集分辨率（默认 %ux%u）\n"
"  --video-fps N         输出目标帧率（默认 %u）\n"
"  --video-slots N       环缓冲槽数（默认 %u）\n"
"  --video-ready-ms MS   多久没发包就判摄像头故障（默认 %u）\n"
"  --audio-bin PATH      audio_capture 可执行文件（默认 %s）\n"
"  --records-dir DIR     录音目录（默认 %s）\n"
"  --audio-tag TAG       文件名标签（默认 %s）\n"
"  --audio-warmup-ms MS  启动热身录音毫秒数，0 跳过（默认 %u）\n"
"  --alert-hold-ms MS    最近一次 near 之后保持告警的时长（默认 %u）\n"
"  --alert-max-ms MS     单次告警硬上限（默认 %u）\n"
"  --near-guard-ms MS    重复 near 的最短间隔（默认 %u）\n"
"  --tick-ms MS          主循环节拍（默认 %u）\n"
"  --als-poll-ms MS      ALS 轮询间隔（默认 %u）\n"
"  --vib-poll-ms MS      振动轮询间隔（默认 %u；比振动慢就会漏掉短促的一下）\n"
"  --vib-thr-mg N        振动事件阈值 = **逐轴帧间最大变化量**（mg）；0=关闭（默认 %u）\n"
"                        注意：这是**软件阈值**，不是中断（ICM20608 的 INT 脚挂不上，\n"
"                        见 logs/phase7/m1_icm_int_probe_20261008.md）\n"
"  --vib-burst-ms MS     超阈值窗口内的采样间隔（默认 %u）\n"
"  --vib-burst-hold-ms MS  超阈值后保持高频采样的时长（默认 %u）\n"
"  --event-sock PATH     事件出口（AF_UNIX/SOCK_DGRAM，对端 pantilt_daemon）；\n"
"                        空串=不发事件（默认 \"%s\"）\n"
"  --stats-ms MS         STATS 行间隔（默认 %u）\n"
"  --camera-reload-attempts N  摄像头重载上限（默认 %u）\n"
"  --camera-mod-dir DIR  摄像头 .ko 所在目录（默认 %s）\n"
"  --status-file PATH    状态文件（默认 %s）\n"
"  --log-dir DIR         日志目录（默认 %s）\n"
"  --dry-run             只跑状态机，不起任何子进程\n"
"  -v, --verbose\n"
"  -h, --help\n"
"\n"
"ALERT 只由 AP3216C 的 EV_SW/SW_FRONT_PROXIMITY 触发，没有任何注入入口。\n"
"SIGUSR1 立即打印一行 STATS。\n",
		p, g.input_name, g.iio_name, g.video_bin, g.video_dev,
		g.video_dst, g.video_w, g.video_h, g.video_fps, g.video_slots,
		g.video_ready_ms, g.audio_bin, g.records_dir,
		g.audio_tag, g.audio_warmup_ms, g.alert_hold_ms, g.alert_max_ms,
		g.near_guard_ms, g.tick_ms, g.als_poll_ms, g.vib_poll_ms,
		g.vib_thr_mg, g.vib_burst_ms, g.vib_burst_hold_ms, g.event_sock,
		g.stats_ms, g.cam_attempts, g.cam_mod_dir,
		g.status_file, g.log_dir);
}

static void parse_args(int argc, char **argv)
{
	static const struct option opts[] = {
		{ "input-name", required_argument, NULL, 2 },
		{ "iio-name", required_argument, NULL, 3 },
		{ "video-bin", required_argument, NULL, 4 },
		{ "video-dev", required_argument, NULL, 5 },
		{ "video-dst", required_argument, NULL, 6 },
		{ "video-size", required_argument, NULL, 7 },
		{ "video-fps", required_argument, NULL, 8 },
		{ "video-slots", required_argument, NULL, 9 },
		{ "video-ready-ms", required_argument, NULL, 10 },
		{ "audio-bin", required_argument, NULL, 12 },
		{ "records-dir", required_argument, NULL, 13 },
		{ "audio-tag", required_argument, NULL, 14 },
		{ "audio-warmup-ms", required_argument, NULL, 15 },
		{ "alert-hold-ms", required_argument, NULL, 16 },
		{ "alert-max-ms", required_argument, NULL, 17 },
		{ "near-guard-ms", required_argument, NULL, 18 },
		{ "tick-ms", required_argument, NULL, 19 },
		{ "als-poll-ms", required_argument, NULL, 20 },
		{ "vib-poll-ms", required_argument, NULL, 21 },
		{ "vib-thr-mg", required_argument, NULL, 29 },
		{ "vib-burst-ms", required_argument, NULL, 30 },
		{ "vib-burst-hold-ms", required_argument, NULL, 31 },
		{ "event-sock", required_argument, NULL, 32 },
		{ "stats-ms", required_argument, NULL, 23 },
		{ "camera-reload-attempts", required_argument, NULL, 24 },
		{ "camera-mod-dir", required_argument, NULL, 25 },
		{ "status-file", required_argument, NULL, 26 },
		{ "log-dir", required_argument, NULL, 27 },
		{ "dry-run", no_argument, NULL, 28 },
		{ "verbose", no_argument, NULL, 'v' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	int c;

	while ((c = getopt_long(argc, argv, "vh", opts, NULL)) != -1) {
		switch (c) {
		case 2: g.input_name = optarg; break;
		case 3: g.iio_name = optarg; break;
		case 4: g.video_bin = optarg; break;
		case 5: g.video_dev = optarg; break;
		case 6: g.video_dst = optarg; break;
		case 7:
			if (sscanf(optarg, "%ux%u", &g.video_w, &g.video_h) != 2 ||
			    g.video_w == 0 || g.video_h == 0) {
				fprintf(stderr, "securityd: --video-size 需要 WxH\n");
				exit(2);
			}
			break;
		case 8: g.video_fps = (unsigned)atoi(optarg); break;
		case 9: g.video_slots = (unsigned)atoi(optarg); break;
		case 10: g.video_ready_ms = (unsigned)atoi(optarg); break;
		case 12: g.audio_bin = optarg; break;
		case 13: g.records_dir = optarg; break;
		case 14: g.audio_tag = optarg; break;
		case 15: g.audio_warmup_ms = (unsigned)atoi(optarg); break;
		case 16: g.alert_hold_ms = (unsigned)atoi(optarg); break;
		case 17: g.alert_max_ms = (unsigned)atoi(optarg); break;
		case 18: g.near_guard_ms = (unsigned)atoi(optarg); break;
		case 19: g.tick_ms = (unsigned)atoi(optarg); break;
		case 20: g.als_poll_ms = (unsigned)atoi(optarg); break;
		case 21: g.vib_poll_ms = (unsigned)atoi(optarg); break;
		case 23: g.stats_ms = (unsigned)atoi(optarg); break;
		case 24: g.cam_attempts = (unsigned)atoi(optarg); break;
		case 25: g.cam_mod_dir = optarg; break;
		case 26: g.status_file = optarg; break;
		case 27: g.log_dir = optarg; break;
		case 28: g.dry_run = 1; break;
		case 29: g.vib_thr_mg = (unsigned)atoi(optarg); break;
		case 30: g.vib_burst_ms = (unsigned)atoi(optarg); break;
		case 31: g.vib_burst_hold_ms = (unsigned)atoi(optarg); break;
		case 32: g.event_sock = optarg; break;
		case 'v': g.verbose = 1; break;
		case 'h': usage(argv[0]); exit(0);
		default: usage(argv[0]); exit(2);
		}
	}
	if (g.tick_ms == 0)
		g.tick_ms = 100;
	/*
	 * 高频窗口的下限取 tick：on_tick 本身只在 tick 上跑，设得比 tick 还小
	 * 只会在同一个节拍里做无用功，还会让 next_vib_ms 永远追不上 now。
	 */
	if (g.vib_burst_ms < g.tick_ms)
		g.vib_burst_ms = g.tick_ms;
	if (g.alert_hold_ms > g.alert_max_ms)
		g.alert_max_ms = g.alert_hold_ms;
}

int main(int argc, char **argv)
{
	sigset_t mask;
	struct epoll_event evs[SD_EPOLL_MAX];
	struct itimerspec its;
	char deg[64];
	int i;

	setvbuf(stdout, NULL, _IOLBF, 0);
	parse_args(argc, argv);

	s.t_start = now_ms();
	s.state = ST_GUARD;
	s.ep = s.sfd = s.tfd = -1;
	s.input_fd = -1;
	s.ev_fd = -1;
	event_open();
	child_reset(&s.video, "video_link");
	child_reset(&s.audio, "audio_capture");

	if (mkdir_p(g.log_dir) != 0)
		warnx_("建日志目录 %s 失败: %s", g.log_dir, strerror(errno));
	else
		s.logfp = open_logfile("securityd.log");
	s.caplog = open_logfile("camera_reload.log");

	sd_emit("securityd: INFO  启动 tick=%ums hold=%ums max=%ums dry_run=%d",
		g.tick_ms, g.alert_hold_ms, g.alert_max_ms, g.dry_run);
	sd_emit("securityd: INFO  视频 %s %ux%u fps=%u slots=%u -> %s（常开）",
		g.video_dev, g.video_w, g.video_h, g.video_fps, g.video_slots,
		g.video_dst);
	sd_emit("securityd: INFO  录音 %s tag=%s warmup=%ums",
		g.records_dir, g.audio_tag, g.audio_warmup_ms);

	/* 信号一律走 signalfd：先屏蔽，再建 fd。 */
	sigemptyset(&mask);
	sigaddset(&mask, SIGINT);
	sigaddset(&mask, SIGTERM);
	sigaddset(&mask, SIGCHLD);
	sigaddset(&mask, SIGUSR1);
	if (sigprocmask(SIG_BLOCK, &mask, NULL) != 0) {
		warnx_("sigprocmask 失败: %s", strerror(errno));
		return 1;
	}
	s.sfd = signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC);
	if (s.sfd < 0) {
		warnx_("signalfd 失败: %s", strerror(errno));
		return 1;
	}

	s.tfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
	if (s.tfd < 0) {
		warnx_("timerfd_create 失败: %s", strerror(errno));
		return 1;
	}
	memset(&its, 0, sizeof(its));
	its.it_interval.tv_sec = g.tick_ms / 1000;
	its.it_interval.tv_nsec = (long)(g.tick_ms % 1000) * 1000000L;
	its.it_value = its.it_interval;
	if (timerfd_settime(s.tfd, 0, &its, NULL) != 0)
		warnx_("timerfd_settime 失败: %s", strerror(errno));

	s.ep = epoll_create1(EPOLL_CLOEXEC);
	if (s.ep < 0) {
		warnx_("epoll_create1 失败: %s", strerror(errno));
		return 1;
	}
	ep_add(&s, s.sfd, &s.sfd);
	ep_add(&s, s.tfd, &s.tfd);

	/* 自检：先探传感器，再定位 Input。缺哪一块都要立刻说出来。 */
	probe_sensors(&s);
	input_attach(&s);

	if (!g.dry_run) {
		audio_warmup();
		/*
		 * 视频常开：开机就起，之后只在故障时才重载。人走过来那一刻
		 * 摄像头必须已经在发包，不给 probe /dev/video1 留时间。
		 */
		if (video_start(&s) != 0)
			s.deg_camera = 1;
	}

	s.next_als_ms = now_ms();
	s.next_vib_ms = now_ms();
	s.next_stats_ms = now_ms() + g.stats_ms;
	sd_emit("securityd: INFO  进入主循环（Ctrl-C / SIGTERM 退出）");

	while (!s.quit) {
		int n = epoll_wait(s.ep, evs, SD_EPOLL_MAX, 1000);

		if (n < 0) {
			if (errno == EINTR)
				continue;
			warnx_("epoll_wait 失败: %s", strerror(errno));
			break;
		}
		for (i = 0; i < n; i++) {
			int *fd = evs[i].data.ptr;

			if (fd == &s.sfd) {
				handle_signal(&s);
			} else if (fd == &s.tfd) {
				handle_timer(&s);
			} else if (fd == &s.input_fd) {
				handle_input(&s);
			} else if (fd == &s.video.fd && s.video.fd >= 0) {
				if (child_drain(&s.video))
					child_close_pipe(&s, &s.video);
			} else if (fd == &s.audio.fd && s.audio.fd >= 0) {
				if (child_drain(&s.audio))
					child_close_pipe(&s, &s.audio);
			}
		}
	}

	final_stop(&s);

	deg_string(deg, sizeof(deg));
	{
		char line[2048];

		stats_line(line, sizeof(line));
		sd_emit("%s", line);
	}
	status_write();
	sd_emit("securityd: RESULT alerts=%u recorded=%u video_ok=%u "
		"video_reload=%u near=%u far=%u uptime_s=%llu deg=%s",
		s.alerts, s.recorded, s.video_ok, s.video_reload,
		s.near_ev, s.far_ev,
		(unsigned long long)(uptime_ms() / 1000), deg);

	if (s.logfp)
		fclose(s.logfp);
	if (s.caplog)
		fclose(s.caplog);
	return 0;
}
