/*
 * 中文说明：/dev/pantilt0 的最小命令行工具，用于 3A 阶段的板端验收。
 *
 * 它是"驱动能不能安全工作"的验证入口，不是正式控制程序——正式链路是
 * pantilt_daemon（3B）经同一套 ioctl 驱动云台。本工具只做四件事：
 * 读限位配置、认领并回 home、下发绝对目标、查询状态与统计。
 *
 * 与 pantilt_test 的关系：pantilt_test 走 /dev/i2c-1 直接操作芯片，是本工具之前
 * 的验证路径；两者不能同时控制同一个 PCA9685。
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "pantilt_uapi.h"

#define DEV_DEFAULT "/dev/pantilt0"
#define CONF_DEFAULT "/root/imx6hub_demo/pantilt_calibration.conf"
#define STATE_DEFAULT "/root/imx6hub_demo/pantilt_state.conf"
#define LINE_MAX_LEN 160

struct axis_conf {
	const char *name;
	const char *key_home;
	const char *key_min;
	const char *key_max;
	unsigned home, safe_min, safe_max;
};

static struct axis_conf axes[PANTILT_AXES] = {
	{ "pan", "pan_center_pwm", "pan_safe_min", "pan_safe_max", 0, 0, 0 },
	{ "tilt", "tilt_home_pwm", "tilt_safe_min", "tilt_safe_max", 0, 0, 0 },
};

static const char *dev_path(void)
{
	return getenv("PANTILT_DEV") ?: DEV_DEFAULT;
}

static const char *conf_path(void)
{
	return getenv("PANTILT_CONF") ?: CONF_DEFAULT;
}

static const char *state_path(void)
{
	return getenv("PANTILT_STATE") ?: STATE_DEFAULT;
}

static void usage(const char *prog)
{
	fprintf(stderr,
		"用法：\n"
		"  %s info                          只读查询驱动状态与统计\n"
		"  %s limit [CONF] [--slew MS] [--watchdog MS] [--yes]\n"
		"                                   读限位配置并认领芯片，随后缓慢爬回 home\n"
		"  %s target PAN TILT               下发绝对目标 tick（驱动内钳位+限速）\n"
		"  %s home                          再次回到 home\n"
		"  %s stop                          立即停止爬升并保持当前位置\n"
		"\n"
		"环境变量：PANTILT_DEV（默认 %s）、PANTILT_CONF（默认 %s）、\n"
		"          PANTILT_STATE（默认 %s，用于取唤醒起始位置）\n",
		prog, prog, prog, prog, prog, dev_path(), conf_path(), state_path());
}

static int open_dev(void)
{
	int fd = open(dev_path(), O_RDWR);

	if (fd < 0)
		fprintf(stderr, "[ERROR] 打开 %s 失败：%s\n", dev_path(),
			strerror(errno));
	return fd;
}

/* ---- 配置读取 ---- */

static char *trim(char *s)
{
	char *end;

	while (*s == ' ' || *s == '\t')
		s++;
	end = s + strlen(s);
	while (end > s && (end[-1] == ' ' || end[-1] == '\t' ||
			   end[-1] == '\n' || end[-1] == '\r'))
		*--end = '\0';
	return s;
}

static bool parse_u32(const char *s, unsigned *out)
{
	char *end;
	unsigned long n;

	if (!s || !*s || *s == '-')
		return false;
	errno = 0;
	n = strtoul(s, &end, 0);
	if (errno || *end || n > 0xffff)
		return false;
	*out = (unsigned)n;
	return true;
}

/*
 * 逐行扫描 key = value。找到就返回 true，并只认第一次出现的键。
 * 找不到时返回 false，由调用方报错——不填默认值，避免拼错键被静默忽略。
 */
static bool conf_get(const char *path, const char *key, unsigned *out)
{
	FILE *fp = fopen(path, "r");
	char line[LINE_MAX_LEN];
	bool found = false;

	if (!fp) {
		fprintf(stderr, "[ERROR] 打不开配置文件 %s：%s\n", path,
			strerror(errno));
		return false;
	}

	while (!found && fgets(line, sizeof(line), fp)) {
		char *s = line, *eq, *k, *v;

		if ((eq = strchr(s, '#')) != NULL)
			*eq = '\0';
		s = trim(s);
		if (!*s)
			continue;
		eq = strchr(s, '=');
		if (!eq)
			continue;
		*eq = '\0';
		k = trim(s);
		v = trim(eq + 1);
		if (strcmp(k, key) == 0) {
			if (!parse_u32(v, out))
				fprintf(stderr,
					"[ERROR] %s 的 %s 不是合法 tick：'%s'\n",
					path, key, v);
			else
				found = true;
		}
	}
	fclose(fp);

	if (!found)
		fprintf(stderr, "[ERROR] %s 缺少 %s（不填默认值，拒绝继续）\n",
			path, key);
	return found;
}

static bool conf_load_all(const char *path)
{
	unsigned i;

	for (i = 0; i < PANTILT_AXES; i++) {
		struct axis_conf *a = &axes[i];

		if (!conf_get(path, a->key_home, &a->home) ||
		    !conf_get(path, a->key_min, &a->safe_min) ||
		    !conf_get(path, a->key_max, &a->safe_max))
			return false;
	}
	return true;
}

/* pantilt_test 留下的位置记录，格式为每行 "<channel> <tick>"。 */
static bool state_load_start(unsigned tick[PANTILT_AXES], bool have[PANTILT_AXES])
{
	FILE *fp = fopen(state_path(), "r");
	char line[LINE_MAX_LEN];
	unsigned i;

	for (i = 0; i < PANTILT_AXES; i++)
		have[i] = false;
	if (!fp)
		return false;

	while (fgets(line, sizeof(line), fp)) {
		unsigned ch, val;

		if (sscanf(line, "%u %u", &ch, &val) == 2 && ch < PANTILT_AXES) {
			tick[ch] = val;
			have[ch] = true;
		}
	}
	fclose(fp);
	return have[0] || have[1];
}

/* ---- 输出 ---- */

static const char *status_name(unsigned s)
{
	switch (s) {
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

static const char *chip_name(unsigned c)
{
	switch (c) {
	case PANTILT_CHIP_ASLEEP:
		return "ASLEEP";
	case PANTILT_CHIP_RUNNING:
		return "RUNNING";
	default:
		return "UNKNOWN";
	}
}

/* 1 tick = 20 ms / 4096，约 0.4395°；仅用于打印估算。 */
static double tick_to_ms(unsigned tick)
{
	return tick * 20.0 / 4096.0;
}

static int cmd_state(int fd)
{
	struct pantilt_state st;
	unsigned i;

	if (ioctl(fd, PANTILT_IOC_GET_STATE, &st) < 0) {
		fprintf(stderr, "[ERROR] GET_STATE 失败：%s\n", strerror(errno));
		return 1;
	}

	printf("status      %s\n", status_name(st.status));
	printf("chip_state  %s\n", chip_name(st.chip_state));
	printf("slew        %u ms/tick   watchdog %u ms\n", st.slew_ms,
	       st.watchdog_ms);
	for (i = 0; i < PANTILT_AXES; i++) {
		printf("%-4s cur=%4u (%.2f ms)  tgt=%4u  home=%4u  safe=%u..%u\n",
		       i == PANTILT_AXIS_PAN ? "pan" : "tilt", st.cur_tick[i],
		       tick_to_ms(st.cur_tick[i]), st.tgt_tick[i], st.home[i],
		       st.safe_min[i], st.safe_max[i]);
	}
	printf("cmd=%llu  clamp_low=[%llu,%llu]  clamp_high=[%llu,%llu]\n",
	       st.cmd_count, st.clamp_low[0], st.clamp_low[1], st.clamp_high[0],
	       st.clamp_high[1]);
	printf("i2c_err=%llu  watchdog_trips=%llu  last_latency=%llu us\n",
	       st.i2c_err, st.watchdog_trips, st.last_latency_us);
	return 0;
}

static int cmd_limit(int fd, int argc, char **argv)
{
	struct pantilt_limit lim;
	unsigned start[PANTILT_AXES];
	bool have[PANTILT_AXES];
	const char *path = conf_path();
	unsigned slew = 15, watchdog = 500;
	unsigned i, dist, worst = 0;
	bool assume_yes = false;
	int argi;

	for (argi = 0; argi < argc; argi++) {
		if (strcmp(argv[argi], "--slew") == 0 && argi + 1 < argc) {
			if (!parse_u32(argv[++argi], &slew)) {
				fprintf(stderr, "[ERROR] --slew 非法\n");
				return 2;
			}
		} else if (strcmp(argv[argi], "--watchdog") == 0 &&
			   argi + 1 < argc) {
			if (!parse_u32(argv[++argi], &watchdog)) {
				fprintf(stderr, "[ERROR] --watchdog 非法\n");
				return 2;
			}
		} else if (strcmp(argv[argi], "--yes") == 0) {
			assume_yes = true;
		} else if (argv[argi][0] != '-') {
			path = argv[argi];
		} else {
			fprintf(stderr, "[ERROR] 未知参数 %s\n", argv[argi]);
			return 2;
		}
	}

	if (!conf_load_all(path))
		return 1;

	state_load_start(start, have);

	memset(&lim, 0, sizeof(lim));
	for (i = 0; i < PANTILT_AXES; i++) {
		struct axis_conf *a = &axes[i];

		if (a->safe_min > a->safe_max || a->home < a->safe_min ||
		    a->home > a->safe_max) {
			fprintf(stderr,
				"[ERROR] %s 的 min<=home<=max 不成立（%u %u %u）\n",
				a->name, a->safe_min, a->home, a->safe_max);
			return 1;
		}
		lim.home[i] = (uint16_t)a->home;
		lim.safe_min[i] = (uint16_t)a->safe_min;
		lim.safe_max[i] = (uint16_t)a->safe_max;

		if (have[i] && start[i] >= a->safe_min && start[i] <= a->safe_max) {
			lim.start[i] = (uint16_t)start[i];
		} else {
			lim.start[i] = (uint16_t)a->home;
			if (!have[i])
				fprintf(stderr,
					"[WARN] %s 里没有 CH%u 的可用记录，将以 home %u 唤醒；若舵机不在该姿态，这一步会跳动。\n",
					state_path(), i, a->home);
			else
				fprintf(stderr,
					"[WARN] CH%u 记录值 %u 不在 safe 范围内，忽略，改以 home %u 唤醒。\n",
					i, start[i], a->home);
		}
	}
	lim.slew_ms = (uint16_t)slew;
	lim.watchdog_ms = (uint16_t)watchdog;

	printf("[INFO] 限位来自 %s；唤醒位置来自 %s\n", path, state_path());
	for (i = 0; i < PANTILT_AXES; i++) {
		printf("       %-4s start=%u  home=%u  safe=%u..%u\n", axes[i].name,
		       lim.start[i], lim.home[i], lim.safe_min[i],
		       lim.safe_max[i]);
		dist = lim.start[i] > lim.home[i] ? lim.start[i] - lim.home[i]
						  : lim.home[i] - lim.start[i];
		if (dist > worst)
			worst = dist;
	}
	printf("[WARN] 该命令会唤醒 PCA9685 并以 %u ms/tick 爬回 home，预计最长 %.1f 秒，期间舵机会转动。\n",
	       slew, worst * slew / 1000.0);
	printf("[WARN] 手边保持可随时切断舵机电源。确认周围无遮挡");

	if (!assume_yes) {
		char line[16];

		printf("，按 Enter 继续：");
		fflush(stdout);
		if (!fgets(line, sizeof(line), stdin)) {
			fprintf(stderr, "\n[ERROR] 未确认，未做任何输出\n");
			return 1;
		}
	} else {
		printf("（--yes）。\n");
	}

	if (ioctl(fd, PANTILT_IOC_SET_LIMIT, &lim) < 0) {
		fprintf(stderr, "[ERROR] SET_LIMIT 失败：%s\n", strerror(errno));
		if (errno == EBUSY)
			fprintf(stderr, "        驱动已认领芯片；改限位需先 rmmod 再 insmod。\n");
		if (errno == EINVAL)
			fprintf(stderr,
				"        参数被驱动拒绝：检查是否越出电气护栏（rules/Hardware.md §2）。\n");
		return 1;
	}
	printf("[INFO] 已认领，正在爬回 home。用 'pantilt_ctl info' 观察进度。\n");
	return 0;
}

static int cmd_target(int fd, unsigned pan, unsigned tilt)
{
	struct pantilt_target tgt;

	tgt.tick[PANTILT_AXIS_PAN] = (uint16_t)pan;
	tgt.tick[PANTILT_AXIS_TILT] = (uint16_t)tilt;

	if (ioctl(fd, PANTILT_IOC_SET_TARGET, &tgt) < 0) {
		fprintf(stderr, "[ERROR] SET_TARGET 失败：%s\n", strerror(errno));
		if (errno == EAGAIN)
			fprintf(stderr, "        尚未认领，先执行 limit。\n");
		return 1;
	}
	return 0;
}

int main(int argc, char **argv)
{
	const char *cmd;
	int fd, rc;

	if (argc < 2) {
		usage(argv[0]);
		return 2;
	}
	cmd = argv[1];

	if (strcmp(cmd, "info") == 0) {
		if ((fd = open_dev()) < 0)
			return 1;
		rc = cmd_state(fd);
		close(fd);
		return rc;
	}

	if (strcmp(cmd, "limit") == 0) {
		if ((fd = open_dev()) < 0)
			return 1;
		rc = cmd_limit(fd, argc - 2, argv + 2);
		close(fd);
		return rc;
	}

	if (strcmp(cmd, "target") == 0) {
		unsigned pan, tilt;

		if (argc != 4 || !parse_u32(argv[2], &pan) ||
		    !parse_u32(argv[3], &tilt)) {
			fprintf(stderr, "[ERROR] 用法：%s target PAN TILT\n",
				argv[0]);
			return 2;
		}
		if ((fd = open_dev()) < 0)
			return 1;
		rc = cmd_target(fd, pan, tilt);
		close(fd);
		return rc;
	}

	if (strcmp(cmd, "home") == 0 || strcmp(cmd, "stop") == 0) {
		bool is_home = strcmp(cmd, "home") == 0;

		if ((fd = open_dev()) < 0)
			return 1;
		rc = is_home ? ioctl(fd, PANTILT_IOC_HOME)
			     : ioctl(fd, PANTILT_IOC_STOP);
		if (rc < 0) {
			fprintf(stderr, "[ERROR] %s 失败：%s\n", cmd,
				strerror(errno));
			close(fd);
			return 1;
		}
		close(fd);
		return 0;
	}

	fprintf(stderr, "[ERROR] 未知命令 %s\n", cmd);
	usage(argv[0]);
	return 2;
}
