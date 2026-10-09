/*
 * 中文说明：probe 只读；trial 仅从默认休眠态做中心附近小步试动；nudge 需已实测限位；
 * calib 交互式标定单轴；sweep 读 config/pantilt_calibration.conf 同时驱动两轴，
 * 用于验证双舵机同时动作时的供电表现。
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

enum {
	PCA_ADDR_DEFAULT = 0x40,
	PCA_MODE1 = 0x00,
	PCA_MODE2 = 0x01,
	PCA_LED0_ON_L = 0x06,
	PCA_PRESCALE = 0xfe,
	PCA_CHANNELS = 16,
	PCA_TICKS = 4096,
	PCA_FULL_BIT = 0x10,
	PCA_SLEEP_BIT = 0x10,
	PCA_RESTART_BIT = 0x80,
	SERVO_PRESCALE_MIN = 115,
	SERVO_PRESCALE_MAX = 130,
	MAX_NUDGE_TICKS = 4,
	HOLD_MS = 300,
	CENTER_TICK = 307, /* 25 MHz / (4096 * 50 Hz) 对应约 1.5 ms；仅供首次小范围试动。 */
	TRIAL_DELTA_TICK = 8,
	TRIAL_PRESCALE = 121,
	TRIAL_HOLD_MS = 500,

	/* calib：交互式小步标定。护栏按脉宽而非角度定义，不假定机械量程。 */
	CALIB_PRESCALE_50HZ = 121,
	CALIB_CH_PAN = 0,
	CALIB_CH_TILT = 1,
	/* 电气护栏下沿按通道区分：两轴的机械余量不同，依据见 rules/Hardware.md §2。 */
	CALIB_GUARD_PAN_MIN = 205,  /* 1.0 ms：pan 下探实测到 227，护栏再留一档。 */
	CALIB_GUARD_TILT_MIN = 122, /* 0.6 ms：操作者指定下限；估计止点 102±20。 */
	/* 2026-10-07 由 410(2.0 ms) 放宽到 515(2.51 ms)：pan 在 home=377 下实测
	 * 227(1.11 ms) 与 497(2.43 ms) 两端均无阻力、无堵转，原 2.0 ms 护栏是被
	 * 电气限而非机械边界卡住的；tilt 在 home=242 下同样两向均无堵转。 */
	CALIB_GUARD_MAX = 515,
	CALIB_DEFAULT_SPAN = 40,
	/* 只作请求值上限；实际窗口一律被电气护栏裁剪，见 calib_session()。 */
	CALIB_MAX_SPAN = 400,
	CALIB_DEFAULT_STEP = 2,
	CALIB_MAX_STEP = 20,
	CALIB_RAMP_MS = 15,   /* 每 tick 15 ms，约 22 °/s，避免起步突然。 */
	CALIB_SETTLE_MS = 300,
};

static volatile sig_atomic_t stop_requested;

static void on_signal(int signo)
{
	(void)signo;
	stop_requested = 1;
}

/* I2C_RDWR 将寄存器地址写入与数据读取组成一次事务，避免两次系统调用之间被插入操作。 */
static int read_reg(int fd, uint16_t addr, uint8_t reg, uint8_t *value)
{
	struct i2c_msg msg[2] = {
		{ .addr = addr, .flags = 0, .len = 1, .buf = &reg },
		{ .addr = addr, .flags = I2C_M_RD, .len = 1, .buf = value },
	};
	struct i2c_rdwr_ioctl_data data = { .msgs = msg, .nmsgs = 2 };

	int ret = ioctl(fd, I2C_RDWR, &data);

	if (ret == 2)
		return 0;
	if (ret >= 0)
		errno = EIO;
	return -1;
}

static int write_reg(int fd, uint16_t addr, uint8_t reg, uint8_t value)
{
	uint8_t bytes[2] = { reg, value };
	struct i2c_msg msg = { .addr = addr, .flags = 0,
				.len = sizeof(bytes), .buf = bytes };
	struct i2c_rdwr_ioctl_data data = { .msgs = &msg, .nmsgs = 1 };

	int ret = ioctl(fd, I2C_RDWR, &data);

	if (ret == 1)
		return 0;
	if (ret >= 0)
		errno = EIO;
	return -1;
}

static bool parse_u32(const char *s, uint32_t max, uint32_t *out)
{
	char *end;
	unsigned long n;

	if (!s || !*s || *s == '-')
		return false;
	errno = 0;
	n = strtoul(s, &end, 0);
	if (errno || *end || n > max)
		return false;
	*out = (uint32_t)n;
	return true;
}

static bool parse_delta(const char *s, int *delta)
{
	char *end;
	long n;

	errno = 0;
	n = strtol(s, &end, 0);
	if (errno || !s[0] || *end || n == 0 ||
	    n < -MAX_NUDGE_TICKS || n > MAX_NUDGE_TICKS)
		return false;
	*delta = (int)n;
	return true;
}

static int wait_ms(long ms)
{
	struct timespec delay = { .tv_sec = ms / 1000,
				  .tv_nsec = (ms % 1000) * 1000000L };

	while (!stop_requested) {
		if (nanosleep(&delay, &delay) == 0)
			return 0;
		if (errno != EINTR)
			return -1;
	}
	errno = EINTR;
	return -1;
}

/* 回收路径专用：不受 stop_requested 影响，保证退出前一定能把舵机送回 home。 */
static int safe_wait_ms(long ms)
{
	struct timespec delay = { .tv_sec = ms / 1000,
				  .tv_nsec = (ms % 1000) * 1000000L };

	while (nanosleep(&delay, &delay) < 0) {
		if (errno != EINTR)
			return -1;
	}
	return 0;
}

/*
 * 写单通道脉宽。LEDn_OFF 高低字节同时变化时，若按低字节先写，这中间会有一个
 * PWM 周期读到新旧混搭的脉宽，最坏相差 256 tick，足以把舵机拽到意外位置。
 * 因此跨字节时先把 OFF_H 的 FULL 位置起整体关闭该通道，再写入完整值。
 */
static int write_pulse(int fd, uint16_t addr, uint32_t channel, uint16_t off,
		       bool clear_first)
{
	uint8_t base = PCA_LED0_ON_L + channel * 4;

	if (clear_first && write_reg(fd, addr, base + 3, PCA_FULL_BIT) < 0)
		return -1;
	if (write_reg(fd, addr, base, 0) < 0 ||
	    write_reg(fd, addr, base + 1, 0) < 0 ||
	    write_reg(fd, addr, base + 2, off & 0xff) < 0 ||
	    write_reg(fd, addr, base + 3, (off >> 8) & 0x0f) < 0)
		return -1;
	return 0;
}

/*
 * 每 tick 一步地从 from 爬到 to；调用前必须保证 to 已过限位检查。
 * *reached 始终回填最后一次真正写出的值：中途被打断时回收路径要靠它知道舵机
 * 现在在哪，否则会按起点重写一次，把舵机拽回去。
 */
static int ramp_to(int fd, uint16_t addr, uint32_t channel, uint16_t from,
		   uint16_t to, uint32_t *reached)
{
	int step = (to >= from) ? 1 : -1;
	uint16_t old = from;
	uint16_t value = from;

	for (;;) {
		if (write_pulse(fd, addr, channel, value,
				(value >> 8) != (old >> 8)) < 0)
			return -1;
		old = value;
		*reached = value;
		if (value == to)
			break;
		if (stop_requested) {
			errno = EINTR;
			return -1;
		}
		if (wait_ms(CALIB_RAMP_MS) < 0)
			return -1;
		value = (uint16_t)(value + step);
	}
	return 0;
}

static int set_channel(int fd, uint16_t addr, uint32_t channel,
		       uint16_t on, uint16_t off)
{
	uint8_t base = PCA_LED0_ON_L + channel * 4;
	uint8_t value;

	/* LEDn_ON/OFF 高字节 bit4=全开/全关；按低字节到高字节顺序写，最后才解除全关。 */
	if (write_reg(fd, addr, base, on & 0xff) < 0 ||
	    write_reg(fd, addr, base + 1, (on >> 8) & 0x0f) < 0 ||
	    write_reg(fd, addr, base + 2, off & 0xff) < 0 ||
	    write_reg(fd, addr, base + 3, (off >> 8) & 0x0f) < 0)
		return -1;
	if (read_reg(fd, addr, base + 2, &value) < 0 || value != (off & 0xff)) {
		errno = EIO;
		return -1;
	}
	if (read_reg(fd, addr, base + 3, &value) < 0 ||
	    value != ((off >> 8) & 0x0f)) {
		errno = EIO;
		return -1;
	}
	return 0;
}

static int center_trial(int fd, uint16_t addr, uint32_t channel,
			uint8_t mode1, uint8_t mode2, uint8_t prescale)
{
	uint8_t old[2][4];
	uint8_t value;
	int rc = 1;

	/* 只接受本次读到的上电默认状态，防止覆盖其他控制器正在输出的 PWM。 */
	if ((mode1 & ~PCA_RESTART_BIT) != 0x11 ||
	    mode2 != 0x04 || prescale != 0x1e) {
		fprintf(stderr, "[ERROR] PCA9685 非预期默认状态，拒绝首次试动\n");
		return 1;
	}
	for (unsigned int ch = 0; ch < 2; ch++) {
		for (unsigned int i = 0; i < 4; i++) {
			if (read_reg(fd, addr, PCA_LED0_ON_L + ch * 4 + i,
				     &old[ch][i]) < 0) {
				perror("[ERROR] 保存原通道状态");
				return 1;
			}
		}
		if (old[ch][0] || old[ch][1] || old[ch][2] ||
		    old[ch][3] != PCA_FULL_BIT) {
			fprintf(stderr, "[ERROR] CH0/CH1 未处于全关状态，拒绝首次试动\n");
			return 1;
		}
	}
	printf("[INFO] 首次试动 CH%u：中心 %u tick，偏移 ±%u tick\n",
	       channel, CENTER_TICK, TRIAL_DELTA_TICK);
	/* MODE1 bit4=SLEEP 时才能修改 PRE_SCALE；先设 50 Hz，再写脉宽，最后启动振荡器。 */
	if (write_reg(fd, addr, PCA_PRESCALE, TRIAL_PRESCALE) < 0 ||
	    set_channel(fd, addr, channel, 0, CENTER_TICK) < 0 ||
	    write_reg(fd, addr, PCA_MODE1,
		      mode1 & ~(PCA_SLEEP_BIT | PCA_RESTART_BIT)) < 0) {
		perror("[ERROR] 初始化 50 Hz/中心脉宽");
		goto restore;
	}
	/* RESTART 为只读状态位；先唤醒并等待振荡器稳定，随后写 1 清除此位。 */
	if (mode1 & PCA_RESTART_BIT) {
		if (wait_ms(2) < 0 ||
		    write_reg(fd, addr, PCA_MODE1,
			      (mode1 & ~PCA_SLEEP_BIT) | PCA_RESTART_BIT) < 0) {
			perror("[ERROR] 重启 PCA9685 振荡器");
			goto restore;
		}
	}
	if (wait_ms(TRIAL_HOLD_MS) < 0)
		goto restore;
	if (read_reg(fd, addr, PCA_MODE1, &value) < 0 ||
	    (value & PCA_SLEEP_BIT)) {
		fprintf(stderr, "[ERROR] 振荡器未启动\n");
		goto restore;
	}
	printf("[INFO] CH%u 中心已输出，尝试 +%u tick\n",
	       channel, TRIAL_DELTA_TICK);
	if (set_channel(fd, addr, channel, 0, CENTER_TICK + TRIAL_DELTA_TICK) < 0 ||
	    wait_ms(TRIAL_HOLD_MS) < 0)
		goto restore;
	printf("[INFO] CH%u 尝试 -%u tick\n", channel, TRIAL_DELTA_TICK);
	if (set_channel(fd, addr, channel, 0, CENTER_TICK - TRIAL_DELTA_TICK) < 0 ||
	    wait_ms(TRIAL_HOLD_MS) < 0)
		goto restore;
	if (set_channel(fd, addr, channel, 0, CENTER_TICK) < 0 ||
	    wait_ms(TRIAL_HOLD_MS) < 0)
		goto restore;
	rc = 0;
restore:
	/* 先全关再休眠，避免 MODE1 bit7 的 RESTART 标志保留旧 PWM。 */
	if (write_reg(fd, addr, PCA_LED0_ON_L + channel * 4 + 3,
		      PCA_FULL_BIT) < 0 || wait_ms(25) < 0) {
		perror("[ERROR] 关闭当前通道");
		rc = 1;
	}
	if (write_reg(fd, addr, PCA_MODE1,
		      (mode1 & ~PCA_RESTART_BIT) | PCA_SLEEP_BIT) < 0) {
		perror("[ERROR] 进入 SLEEP");
		rc = 1;
	}
	for (unsigned int i = 0; i < 4; i++) {
		if (write_reg(fd, addr, PCA_LED0_ON_L + channel * 4 + i,
			      old[channel][i]) < 0) {
			perror("[ERROR] 恢复通道寄存器");
			rc = 1;
		}
	}
	if (write_reg(fd, addr, PCA_PRESCALE, prescale) < 0 ||
	    write_reg(fd, addr, PCA_MODE1,
		      (mode1 & ~PCA_RESTART_BIT) | PCA_SLEEP_BIT) < 0) {
		perror("[ERROR] 恢复 PCA9685 初始状态");
		rc = 1;
	}
	if (read_reg(fd, addr, PCA_MODE1, &value) < 0 ||
	    !(value & PCA_SLEEP_BIT)) {
		fprintf(stderr, "[ERROR] 恢复后 SLEEP 状态校验失败\n");
		rc = 1;
	}
	if (!rc)
		printf("[INFO] CH%u 试动命令完成，PCA9685 已休眠并关闭该通道\n", channel);
	return rc;
}

static int nudge(int fd, uint16_t addr, uint32_t channel, int delta,
		 uint32_t min_tick, uint32_t max_tick, uint8_t mode1,
		 uint8_t prescale)
{
	uint8_t regs[4];
	uint8_t base = PCA_LED0_ON_L + channel * 4;
	uint32_t current, target;
	struct timespec hold = { .tv_sec = 0,
				 .tv_nsec = HOLD_MS * 1000000L };
	int rc = 1;

	/* MODE1 bit4=SLEEP；预分频必须已由标定流程设为约 50 Hz，本程序不改频率。 */
	if (mode1 & PCA_SLEEP_BIT || prescale < SERVO_PRESCALE_MIN ||
	    prescale > SERVO_PRESCALE_MAX) {
		fprintf(stderr, "[ERROR] PWM 未运行在约 50 Hz，拒绝移动\n");
		return 1;
	}
	for (unsigned int i = 0; i < 4; i++) {
		if (read_reg(fd, addr, base + i, &regs[i]) < 0) {
			perror("[ERROR] 读取通道寄存器");
			return 1;
		}
	}
	/* LEDn_ON_H/OFF_H bit4 是全开/全关，均不能用于已标定的舵机脉冲。 */
	if (regs[0] || regs[1] || (regs[3] & PCA_FULL_BIT)) {
		fprintf(stderr, "[ERROR] 通道不是 ON=0 的普通舵机脉冲，拒绝移动\n");
		return 1;
	}
	current = regs[2] | ((uint32_t)(regs[3] & 0x0f) << 8);
	if (current < min_tick || current > max_tick) {
		fprintf(stderr, "[ERROR] 当前脉宽 %u 不在实测限位内\n", current);
		return 1;
	}
	target = (uint32_t)((int)current + delta);
	if (target < min_tick || target > max_tick) {
		fprintf(stderr, "[ERROR] 目标脉宽 %u 超出实测限位\n", target);
		return 1;
	}
	printf("[INFO] 通道 %u: %u -> %u tick，%d ms 后恢复\n",
	       channel, current, target, HOLD_MS);
	if (stop_requested)
		return 1;
	if (write_reg(fd, addr, base + 2, target & 0xff) < 0 ||
	    write_reg(fd, addr, base + 3, (target >> 8) & 0x0f) < 0) {
		perror("[ERROR] 写入目标脉宽");
		goto restore;
	}
	while (!stop_requested) {
		if (nanosleep(&hold, &hold) == 0)
			break;
		if (errno != EINTR) {
			perror("[ERROR] 等待动作完成");
			goto restore;
		}
	}
	rc = 0;
restore:
	/* 无论写入、等待或信号中断如何结束，都尽力恢复原脉宽。 */
	if (write_reg(fd, addr, base + 2, regs[2]) < 0 ||
	    write_reg(fd, addr, base + 3, regs[3]) < 0) {
		perror("[ERROR] 恢复原脉宽失败，立即检查舵机");
		return 1;
	}
	return rc;
}

/* 每个通道最后一次输出的 tick；用于下次唤醒时避免跳变，默认放在板端部署目录。 */
#define STATE_PATH_DEFAULT "/root/imx6hub_demo/pantilt_state.conf"

static const char *state_path(void)
{
	const char *path = getenv("PANTILT_STATE");

	return (path && *path) ? path : STATE_PATH_DEFAULT;
}

static void state_read_all(uint32_t tick[PCA_CHANNELS],
			   bool have[PCA_CHANNELS])
{
	unsigned int ch, tk;
	FILE *fp = fopen(state_path(), "r");

	for (unsigned int i = 0; i < PCA_CHANNELS; i++)
		have[i] = false;
	if (!fp)
		return;
	while (fscanf(fp, "%u %u", &ch, &tk) == 2) {
		if (ch < PCA_CHANNELS && tk < PCA_TICKS) {
			tick[ch] = tk;
			have[ch] = true;
		}
	}
	fclose(fp);
}

static bool state_load(uint32_t channel, uint32_t *tick)
{
	uint32_t ticks[PCA_CHANNELS];
	bool have[PCA_CHANNELS];

	state_read_all(ticks, have);
	if (!have[channel])
		return false;
	*tick = ticks[channel];
	return true;
}

/* 读改写整份状态；每次移动后都写，进程被强杀也不会丢掉最后位置。 */
static int state_store(uint32_t channel, uint32_t value)
{
	uint32_t ticks[PCA_CHANNELS];
	bool have[PCA_CHANNELS];
	FILE *fp;

	state_read_all(ticks, have);
	ticks[channel] = value;
	have[channel] = true;
	fp = fopen(state_path(), "w");
	if (!fp)
		return -1;
	for (unsigned int i = 0; i < PCA_CHANNELS; i++) {
		if (have[i] && fprintf(fp, "%u %u\n", i, ticks[i]) < 0) {
			fclose(fp);
			return -1;
		}
	}
	return fclose(fp) == 0 ? 0 : -1;
}

/* 电气护栏按通道区分：pan 与 tilt 的机械余量不同，依据见 rules/Hardware.md §2。 */
static void calib_guard(uint32_t channel, uint32_t *lo, uint32_t *hi)
{
	*lo = (channel == CALIB_CH_TILT) ? CALIB_GUARD_TILT_MIN
					 : CALIB_GUARD_PAN_MIN;
	*hi = CALIB_GUARD_MAX;
}

/*
 * 标定配置（config/pantilt_calibration.conf，扁平 key=value）解析。
 * sweep 用它决定两轴各自的通道与工作范围，不再把 tick 数写在命令行上。
 */
#define CFG_LINE_MAX 160
#define SWEEP_DEFAULT_CYCLES 2
#define SWEEP_MAX_CYCLES 20

struct pantilt_axis_cfg {
	uint32_t channel;
	uint32_t min_tick;
	uint32_t center_tick; /* pan_center_pwm / tilt_home_pwm */
	uint32_t max_tick;
	uint32_t safe_min;
	uint32_t safe_max;
};

struct pantilt_cfg {
	uint32_t bus;
	uint32_t addr;
	uint32_t prescale;
	struct pantilt_axis_cfg axis[2]; /* 0 = pan, 1 = tilt */
};

enum {
	AX_MIN = 1u << 0,
	AX_CENTER = 1u << 1,
	AX_MAX = 1u << 2,
	AX_SAFE_MIN = 1u << 3,
	AX_SAFE_MAX = 1u << 4,
	AX_ALL = 0x1f,
};

static const char *const AXIS_NAME[2] = { "pan", "tilt" };

static char *cfg_trim(char *s)
{
	char *end;

	while (*s == ' ' || *s == '\t')
		s++;
	end = s + strlen(s);
	while (end > s && strchr(" \t\r\n", end[-1]))
		*--end = '\0';
	return s;
}

/* 只接受整条字符串都是数字（基数十进制或 0x 前缀），拒绝 "239abc" 这类半截值。 */
static bool cfg_u32(const char *text, uint32_t limit, uint32_t *out)
{
	char *end;
	unsigned long n;

	errno = 0;
	n = strtoul(text, &end, 0);
	if (errno || end == text || *end || n > limit)
		return false;
	*out = (uint32_t)n;
	return true;
}

/*
 * 逐行读配置。任何一行不认识或值非法都直接失败——这份文件是安全边界，
 * 静默忽略一个拼错的键就等于静默放宽一个限位。
 */
static int cfg_load(const char *path, struct pantilt_cfg *cfg)
{
	static const struct {
		const char *suffix;
		unsigned int bit;
	} axis_keys[] = {
		{ "min_pwm", AX_MIN },
		{ "center_pwm", AX_CENTER },
		{ "home_pwm", AX_CENTER }, /* 计划里 tilt 用的是 tilt_home_pwm */
		{ "max_pwm", AX_MAX },
		{ "safe_min", AX_SAFE_MIN },
		{ "safe_max", AX_SAFE_MAX },
	};
	unsigned int have[2] = { 0, 0 };
	unsigned int lineno = 0;
	char line[CFG_LINE_MAX];
	FILE *fp;

	fp = fopen(path, "r");
	if (!fp) {
		fprintf(stderr, "[ERROR] 打开配置 %s: %s\n", path,
			strerror(errno));
		return -1;
	}
	memset(cfg, 0, sizeof(*cfg));
	cfg->addr = PCA_ADDR_DEFAULT;
	cfg->prescale = CALIB_PRESCALE_50HZ;
	cfg->axis[0].channel = CALIB_CH_PAN;
	cfg->axis[1].channel = CALIB_CH_TILT;

	while (fgets(line, sizeof(line), fp)) {
		const char *suffix;
		char *sep, *key, *val, *hash;
		bool known = false, ok = false;

		lineno++;
		hash = strchr(line, '#');
		if (hash)
			*hash = '\0';
		key = cfg_trim(line);
		if (!*key)
			continue;
		sep = strchr(key, '=');
		if (!sep) {
			fprintf(stderr, "[ERROR] %s:%u 这行没有 '=': %s\n", path,
				lineno, key);
			goto fail;
		}
		*sep = '\0';
		val = cfg_trim(sep + 1);
		key = cfg_trim(key);
		if (!*val) {
			fprintf(stderr, "[ERROR] %s:%u 键 '%s' 没有值\n", path,
				lineno, key);
			goto fail;
		}

		if (!strcmp(key, "prescale")) {
			known = true;
			ok = cfg_u32(val, 255, &cfg->prescale);
		} else if (!strcmp(key, "bus")) {
			known = true;
			ok = cfg_u32(val, 255, &cfg->bus);
		} else if (!strcmp(key, "i2c_addr")) {
			known = true;
			ok = cfg_u32(val, 0x7f, &cfg->addr);
		} else {
			for (int a = 0; a < 2 && !known; a++) {
				size_t n = strlen(AXIS_NAME[a]);

				if (strncmp(key, AXIS_NAME[a], n) ||
				    key[n] != '_')
					continue;
				suffix = key + n + 1;
				known = true;
				if (!strcmp(suffix, "channel")) {
					ok = cfg_u32(val, PCA_CHANNELS - 1,
						     &cfg->axis[a].channel);
				} else if (!strcmp(suffix, "direction")) {
					ok = true; /* 仅供人读，sweep 不依赖 */
				} else {
					unsigned int k;

					for (k = 0;
					     k < sizeof(axis_keys) /
							 sizeof(axis_keys[0]);
					     k++) {
						uint32_t *slot;

						if (strcmp(suffix,
							   axis_keys[k].suffix))
							continue;
						switch (axis_keys[k].bit) {
						case AX_MIN:
							slot = &cfg->axis[a].min_tick;
							break;
						case AX_CENTER:
							slot = &cfg->axis[a].center_tick;
							break;
						case AX_MAX:
							slot = &cfg->axis[a].max_tick;
							break;
						case AX_SAFE_MIN:
							slot = &cfg->axis[a].safe_min;
							break;
						default:
							slot = &cfg->axis[a].safe_max;
							break;
						}
						ok = cfg_u32(val, PCA_TICKS - 1,
							     slot);
						if (ok)
							have[a] |= axis_keys[k].bit;
						break;
					}
					if (k == sizeof(axis_keys) /
							 sizeof(axis_keys[0]))
						known = false;
				}
			}
		}
		if (!known) {
			fprintf(stderr, "[ERROR] %s:%u 不认识的键 '%s'\n", path,
				lineno, key);
			goto fail;
		}
		if (!ok) {
			fprintf(stderr, "[ERROR] %s:%u 键 '%s' 的值无效: %s\n",
				path, lineno, key, val);
			goto fail;
		}
	}
	if (fclose(fp) != 0) {
		perror("[ERROR] 读取配置");
		return -1;
	}

	/* 通道冲突要先查：否则错误会以"两轴共用的那个通道"的名义报出来。 */
	if (cfg->axis[0].channel == cfg->axis[1].channel) {
		fprintf(stderr, "[ERROR] 两轴都指向 CH%u\n",
			cfg->axis[0].channel);
		return -1;
	}
	/* tick 与毫秒的换算、以及配置里所有标定值都只在 50 Hz 下成立。 */
	if (cfg->prescale != CALIB_PRESCALE_50HZ) {
		fprintf(stderr,
			"[ERROR] prescale=%u，标定值只在 %u（50 Hz）下有效\n",
			cfg->prescale, CALIB_PRESCALE_50HZ);
		return -1;
	}

	for (int a = 0; a < 2; a++) {
		const struct pantilt_axis_cfg *ax = &cfg->axis[a];
		uint32_t lo, hi;

		if (have[a] != AX_ALL) {
			fprintf(stderr,
				"[ERROR] 配置里 %s 的字段不全（需要 min/max/center 与 safe_min/safe_max，已见掩码 0x%02x）\n",
				AXIS_NAME[a], have[a]);
			return -1;
		}
		if (ax->min_tick > ax->center_tick ||
		    ax->center_tick > ax->max_tick) {
			fprintf(stderr, "[ERROR] %s 需要 min <= center <= max，实为 %u/%u/%u\n",
				AXIS_NAME[a], ax->min_tick, ax->center_tick,
				ax->max_tick);
			return -1;
		}
		if (ax->safe_min > ax->safe_max || ax->safe_min < ax->min_tick ||
		    ax->safe_max > ax->max_tick) {
			fprintf(stderr, "[ERROR] %s 的工作范围 [%u, %u] 越出 [%u, %u]\n",
				AXIS_NAME[a], ax->safe_min, ax->safe_max,
				ax->min_tick, ax->max_tick);
			return -1;
		}
		if (ax->safe_min > ax->center_tick ||
		    ax->center_tick > ax->safe_max) {
			fprintf(stderr, "[ERROR] %s 的 home %u 不在工作范围 [%u, %u] 内\n",
				AXIS_NAME[a], ax->center_tick, ax->safe_min,
				ax->safe_max);
			return -1;
		}
		/* 护栏是硬约束：配置里写超了也不放行（rules/Hardware.md §2）。 */
		calib_guard(ax->channel, &lo, &hi);
		if (ax->safe_min < lo || ax->safe_max > hi) {
			fprintf(stderr,
				"[ERROR] %s 的工作范围 [%u, %u] 超出 CH%u 电气护栏 [%u, %u]，拒绝\n",
				AXIS_NAME[a], ax->safe_min, ax->safe_max,
				ax->channel, lo, hi);
			return -1;
		}
	}
	return 0;

fail:
	fclose(fp);
	return -1;
}

static void calib_help(void)
{
	printf("  + / -      当前通道按步长走一步\n");
	printf("  h          爬回 home\n");
	printf("  s N        设置步长（1..%d tick，当前默认 %d）\n", CALIB_MAX_STEP,
	       CALIB_DEFAULT_STEP);
	printf("  m min|max|home  把当前 tick 记为候选值\n");
	printf("  p          打印当前状态\n");
	printf("  q          结束并有序回收（先回 home，再关通道休眠）\n");
}

/*
 * 交互式小步标定：先由操作者确认 home 姿态，再逐条命令做小步探索。
 * 脉宽只在 [home-span, home+span] 与电气护栏的交集内取值；退出路径先爬回 home，
 * 再关闭通道并休眠，避免把舵机留在边界姿态上。
 */
static int calib_session(int fd, uint16_t addr, uint32_t channel, uint32_t home,
			 uint32_t span, uint8_t mode1, uint8_t mode2,
			 uint8_t prescale)
{
	uint8_t regs[4];
	uint8_t base = PCA_LED0_ON_L + channel * 4;
	uint8_t value;
	uint32_t lo, hi, current;
	uint32_t stored = 0;
	uint32_t guard_min, guard_max;
	uint32_t visited_min, visited_max;
	uint32_t mark_min = 0, mark_max = 0;
	uint32_t step = CALIB_DEFAULT_STEP;
	char line[64];
	bool fresh;
	/* 唤醒时通道上是什么：无法预知（会跳）/ 上次记录的位置 / 上一轮留下的运行态 */
	enum { START_JUMP, START_STORED, START_RESUME } origin;
	int rc = 1;

	calib_guard(channel, &guard_min, &guard_max);
	if (span == 0 || span > CALIB_MAX_SPAN || home < guard_min ||
	    home > guard_max) {
		fprintf(stderr,
			"[ERROR] home=%u span=%u 超出护栏：CH%u 为 [%u, %u]，span 上限 %u\n",
			home, span, channel, guard_min, guard_max,
			CALIB_MAX_SPAN);
		return 2;
	}
	/* home 靠近护栏端点时请求窗口会被裁掉一部分：安全边界是护栏，不是 span。 */
	lo = home > guard_min + span ? home - span : guard_min;
	hi = home + span < guard_max ? home + span : guard_max;
	if (lo != home - span || hi != home + span)
		printf("[INFO] 请求窗口 [%u, %u] 被电气护栏裁剪为 [%u, %u]\n",
		       home > span ? home - span : 0, home + span, lo, hi);

	/* 只接受上电默认静止态，或本工具上一轮留下的、已在 50 Hz 运行的状态。 */
	fresh = (mode1 & ~PCA_RESTART_BIT) == 0x11 && prescale == 0x1e;
	if ((!fresh && (mode1 & PCA_SLEEP_BIT)) ||
	    (!fresh && (prescale < SERVO_PRESCALE_MIN ||
			prescale > SERVO_PRESCALE_MAX)) ||
	    mode2 != 0x04) {
		fprintf(stderr,
			"[ERROR] PCA9685 不在已知静止态（MODE1=0x%02x MODE2=0x%02x PRE_SCALE=0x%02x），拒绝标定\n",
			mode1, mode2, prescale);
		return 1;
	}
	for (unsigned int i = 0; i < 4; i++) {
		if (read_reg(fd, addr, base + i, &regs[i]) < 0) {
			perror("[ERROR] 读取通道寄存器");
			return 1;
		}
	}
	if (fresh) {
		if (regs[0] || regs[1] || regs[3] != PCA_FULL_BIT) {
			fprintf(stderr, "[ERROR] CH%u 不在全关状态，拒绝标定\n",
				channel);
			return 1;
		}
		/* 有上次记录就照它唤醒，舵机停在原地，再缓慢爬回 home。 */
		if (state_load(channel, &stored) &&
		    stored >= guard_min && stored <= guard_max) {
			origin = START_STORED;
			current = stored;
		} else {
			origin = START_JUMP;
			current = home;
		}
	} else {
		if (regs[0] || regs[1] || (regs[3] & PCA_FULL_BIT)) {
			fprintf(stderr,
				"[ERROR] CH%u 不是 ON=0 的普通舵机脉冲，拒绝标定\n",
				channel);
			return 1;
		}
		current = regs[2] | ((uint32_t)(regs[3] & 0x0f) << 8);
		if (current < lo || current > hi) {
			fprintf(stderr,
				"[ERROR] 通道当前 %u tick 不在本次窗口 [%u, %u] 内\n",
				current, lo, hi);
			return 1;
		}
		origin = START_RESUME;
	}

	printf("\n[WARN] 即将把 CH%u 置为 50 Hz 并输出 %u tick（约 %.2f ms）。\n",
	       channel, current, current * 20.0 / PCA_TICKS);
	if (origin == START_JUMP)
		printf("[WARN] %s 里没有 CH%u 的可用记录，若舵机不在该姿态，这一步会产生跳动。\n",
		       state_path(), channel);
	else if (origin == START_STORED && current != home)
		printf("[INFO] 按上次记录 %u tick 唤醒（不跳变），随后缓慢爬回 home %u。\n",
		       current, home);
	else
		printf("[INFO] 唤醒位置与上次一致，不会跳变。\n");
	printf("[WARN] 本次窗口 %u..%u tick（home ±%u），电气护栏 %u..%u tick。\n",
	       lo, hi, span, guard_min, guard_max);
	printf("[WARN] 手边保持可随时切断舵机电源。确认周围无遮挡后按 Enter 开始：");
	fflush(stdout);
	if (!fgets(line, sizeof(line), stdin)) {
		fprintf(stderr, "\n[ERROR] 未确认，未做任何输出\n");
		return 1;
	}

	/* PRE_SCALE 只能在 SLEEP 下修改；恢复态已是 50 Hz，不再改写。 */
	if (fresh && write_reg(fd, addr, PCA_PRESCALE, CALIB_PRESCALE_50HZ) < 0) {
		perror("[ERROR] 设置 50 Hz 预分频");
		return 1;
	}
	/* 唤醒前先把脉宽写好，避免启动瞬间出现未定义脉冲。 */
	if (write_pulse(fd, addr, channel, (uint16_t)current, true) < 0) {
		perror("[ERROR] 预置起始脉宽");
		return 1;
	}
	if (write_reg(fd, addr, PCA_MODE1,
		      mode1 & ~(PCA_SLEEP_BIT | PCA_RESTART_BIT)) < 0) {
		perror("[ERROR] 退出 SLEEP");
		return 1;
	}
	/* RESTART 为只读状态位：先唤醒等振荡器稳定，再写 1 清除。 */
	if ((mode1 & PCA_RESTART_BIT) &&
	    (safe_wait_ms(2) < 0 ||
	     write_reg(fd, addr, PCA_MODE1,
		       (mode1 & ~PCA_SLEEP_BIT) | PCA_RESTART_BIT) < 0)) {
		perror("[ERROR] 重启 PCA9685 振荡器");
		return 1;
	}
	if (safe_wait_ms(5) < 0 || read_reg(fd, addr, PCA_MODE1, &value) < 0 ||
	    (value & PCA_SLEEP_BIT)) {
		fprintf(stderr, "[ERROR] 振荡器未启动\n");
		return 1;
	}

	visited_min = current;
	visited_max = current;
	printf("[INFO] CH%u 已输出 %u tick，振荡器已启动。\n", channel, current);
	state_store(channel, current);

	/* 唤醒位置与 home 不一致时，按标定同样的节奏爬过去，不跳变。 */
	if (current != home) {
		uint32_t dist = current > home ? current - home : home - current;

		printf("[INFO] 缓慢爬回 home %u tick（%s %u tick，约 %.1f s）...\n",
		       home, current < home ? "增大" : "减小", dist,
		       (double)dist * CALIB_RAMP_MS / 1000.0);
		fflush(stdout);
		if (ramp_to(fd, addr, channel, (uint16_t)current,
			    (uint16_t)home, &current) < 0) {
			perror("\n[ERROR] 爬回 home");
			goto recover;
		}
		current = home;
		if (current < visited_min)
			visited_min = current;
		if (current > visited_max)
			visited_max = current;
		if (safe_wait_ms(CALIB_SETTLE_MS) < 0)
			goto recover;
		printf("[INFO] 已到 home %u tick\n", current);
		state_store(channel, current);
	}
	calib_help();

	for (;;) {
		if (stop_requested) {
			printf("\n[INFO] 收到停止信号\n");
			break;
		}
		printf("[%u] cmd> ", current);
		fflush(stdout);
		if (!fgets(line, sizeof(line), stdin)) {
			printf("\n[INFO] 输入结束\n");
			break;
		}
		if (line[0] == '\n')
			continue;
		if (line[0] == 'q') {
			break;
		} else if (line[0] == 'p') {
			printf("  home=%u 窗口=[%u,%u] step=%u current=%u\n", home,
			       lo, hi, step, current);
			if (mark_min || mark_max)
				printf("  已标记 min=%u max=%u\n", mark_min,
				       mark_max);
			continue;
		} else if (line[0] == 's') {
			uint32_t v;

			if (sscanf(line + 1, "%u", &v) == 1 && v >= 1 &&
			    v <= CALIB_MAX_STEP)
				step = v;
			else
				printf("  步长须为 1..%u\n", CALIB_MAX_STEP);
			continue;
		} else if (line[0] == 'm') {
			char what[8] = { 0 };

			if (sscanf(line + 1, "%7s", what) != 1) {
				printf("  用法: m min|max|home\n");
			} else if (!strcmp(what, "min")) {
				mark_min = current;
				printf("  已记录 min=%u\n", current);
			} else if (!strcmp(what, "max")) {
				mark_max = current;
				printf("  已记录 max=%u\n", current);
			} else if (!strcmp(what, "home")) {
				home = current;
				printf("  已改用 home=%u\n", current);
			} else {
				printf("  用法: m min|max|home\n");
			}
			continue;
		} else if (line[0] != '+' && line[0] != '-' && line[0] != 'h') {
			printf("  未知命令\n");
			calib_help();
			continue;
		}

		uint32_t target;

		if (line[0] == 'h') {
			target = home;
		} else {
			long t = (long)current +
				 (line[0] == '+' ? (long)step : -(long)step);

			if (t < 0) {
				printf("  已到窗口边界\n");
				continue;
			}
			target = (uint32_t)t;
			if (target < lo || target > hi) {
				printf("  已到本次窗口边界 [%u,%u]\n", lo, hi);
				continue;
			}
		}
		if (target == current) {
			printf("  已在 %u tick\n", current);
			continue;
		}
		printf("  %u -> %u ... ", current, target);
		fflush(stdout);
		if (ramp_to(fd, addr, channel, (uint16_t)current,
			    (uint16_t)target, &current) < 0) {
			printf("中断\n");
			perror("[ERROR] 爬升脉宽");
			goto recover;
		}
		current = target;
		if (current < visited_min)
			visited_min = current;
		if (current > visited_max)
			visited_max = current;
		if (safe_wait_ms(CALIB_SETTLE_MS) < 0)
			goto recover;
		state_store(channel, current);
		printf("到位 %u tick\n", current);
	}
	rc = 0;

recover:
	/* 回收路径必须走完，因此清除停止标志，改为不受中断影响地爬回 home。 */
	stop_requested = 0;
	printf("[INFO] 回收：%u -> home %u ... ", current, home);
	fflush(stdout);
	if (ramp_to(fd, addr, channel, (uint16_t)current, (uint16_t)home,
		    &current) < 0) {
		printf("失败\n");
		perror("[ERROR] 爬回 home 失败，立即检查舵机");
		rc = 1;
	} else {
		printf("完成\n");
		state_store(channel, home);
	}
	if (safe_wait_ms(CALIB_SETTLE_MS) < 0)
		rc = 1;
	/* 先全关再休眠，最后把预分频和 MODE1 还原成进入前的值。 */
	if (write_reg(fd, addr, base + 3, PCA_FULL_BIT) < 0 ||
	    write_reg(fd, addr, PCA_MODE1,
		      (mode1 & ~PCA_RESTART_BIT) | PCA_SLEEP_BIT) < 0 ||
	    write_reg(fd, addr, PCA_PRESCALE, prescale) < 0 ||
	    write_reg(fd, addr, PCA_MODE1,
		      (mode1 & ~PCA_RESTART_BIT) | PCA_SLEEP_BIT) < 0) {
		perror("[ERROR] 恢复 PCA9685 初始状态");
		rc = 1;
	}
	if (read_reg(fd, addr, PCA_MODE1, &value) < 0 || !(value & PCA_SLEEP_BIT)) {
		fprintf(stderr, "[ERROR] 恢复后 SLEEP 校验失败\n");
		rc = 1;
	}

	printf("\n=== CH%u 标定记录 ===\n", channel);
	printf("home(基准)  = %u tick\n", home);
	printf("本次到达    = %u .. %u tick\n", visited_min, visited_max);
	if (mark_min)
		printf("操作者标记 min = %u tick\n", mark_min);
	if (mark_max)
		printf("操作者标记 max = %u tick\n", mark_max);
	printf("注意：本次到达范围只是会话实际走过的区间，不等同于机械安全边界；\n");
	printf("      确定边界后按 margins 后退若干 tick 写入 config/pantilt_calibration.conf。\n");
	return rc;
}

/*
 * 双通道同步爬升：步数取两轴中较大的位移，另一轴线性插值，两轴同时到位。
 * 每步仍是 CALIB_RAMP_MS，可被停止信号打断；reached 回填实际写出的最后位置，
 * 回收路径靠它在被打断后知道舵机在哪。
 */
static int ramp_both(int fd, uint16_t addr, const uint32_t ch[2],
		     const uint32_t from[2], const uint32_t to[2],
		     uint32_t reached[2])
{
	uint32_t steps = 0, old[2];

	for (int a = 0; a < 2; a++) {
		uint32_t d = from[a] > to[a] ? from[a] - to[a] : to[a] - from[a];

		reached[a] = from[a];
		old[a] = from[a];
		if (d > steps)
			steps = d;
	}
	for (uint32_t i = 1; i <= steps; i++) {
		for (int a = 0; a < 2; a++) {
			int64_t span = (int64_t)to[a] - (int64_t)from[a];
			uint32_t v = (uint32_t)((int64_t)from[a] +
						span * (int64_t)i /
							(int64_t)steps);

			if (write_pulse(fd, addr, ch[a], (uint16_t)v,
					(v >> 8) != (old[a] >> 8)) < 0)
				return -1;
			old[a] = v;
			reached[a] = v;
		}
		if (stop_requested) {
			errno = EINTR;
			return -1;
		}
		if (wait_ms(CALIB_RAMP_MS) < 0)
			return -1;
	}
	return 0;
}

/* 走到给定目标点，打印这一段的耗时估计。失败时 now[] 保留实际停下的位置。 */
static int sweep_leg(int fd, uint16_t addr, const uint32_t ch[2],
		     uint32_t now[2], const uint32_t target[2], const char *what)
{
	uint32_t dist = 0;

	for (int a = 0; a < 2; a++) {
		uint32_t d = now[a] > target[a] ? now[a] - target[a]
						: target[a] - now[a];

		if (d > dist)
			dist = d;
	}
	printf("  %s: pan %u→%u, tilt %u→%u（约 %.1f s）... ", what, now[0],
	       target[0], now[1], target[1],
	       (double)dist * CALIB_RAMP_MS / 1000.0);
	fflush(stdout);
	if (ramp_both(fd, addr, ch, now, target, now) < 0) {
		printf("中断\n");
		perror("[ERROR] 双轴爬升");
		return -1;
	}
	for (int a = 0; a < 2; a++) {
		now[a] = target[a];
		state_store(ch[a], now[a]);
	}
	if (safe_wait_ms(CALIB_SETTLE_MS) < 0)
		return -1;
	printf("到位\n");
	return 0;
}

/*
 * 双通道供电测试（Phase 2 "双舵机动作不引起明显供电异常" 的验证手段）。
 * 从 config/pantilt_calibration.conf 读两轴各自的通道与工作范围，走三角形：
 *   home → (pan_min, tilt_max) → (pan_max, tilt_min) → home
 * 三条边都在同时驱动两个舵机，是这套电源最吃力的工况；操作者盯着电源和舵机，
 * 出现掉电复位、堵转声、跳动或明显发热就按 Ctrl-C。
 */
static int sweep_run(int fd, uint16_t addr, const struct pantilt_cfg *cfg,
		     const char *cfg_path, uint32_t cycles, uint8_t mode1,
		     uint8_t mode2, uint8_t prescale)
{
	uint32_t ch[2], home[2], now[2], corner[2];
	uint8_t value;
	char line[64];
	bool fresh;
	int rc = 1;

	for (int a = 0; a < 2; a++) {
		ch[a] = cfg->axis[a].channel;
		home[a] = cfg->axis[a].center_tick;
	}

	fresh = (mode1 & ~PCA_RESTART_BIT) == 0x11 && prescale == 0x1e;
	if ((!fresh && (mode1 & PCA_SLEEP_BIT)) ||
	    (!fresh && (prescale < SERVO_PRESCALE_MIN ||
			prescale > SERVO_PRESCALE_MAX)) ||
	    mode2 != 0x04) {
		fprintf(stderr,
			"[ERROR] PCA9685 不在已知静止态（MODE1=0x%02x MODE2=0x%02x PRE_SCALE=0x%02x），拒绝 sweep\n",
			mode1, mode2, prescale);
		return 1;
	}

	for (int a = 0; a < 2; a++) {
		uint8_t regs[4];
		uint8_t base = PCA_LED0_ON_L + ch[a] * 4;
		uint32_t stored;

		for (unsigned int i = 0; i < 4; i++) {
			if (read_reg(fd, addr, base + i, &regs[i]) < 0) {
				perror("[ERROR] 读取通道寄存器");
				return 1;
			}
		}
		if (fresh) {
			if (regs[0] || regs[1] || regs[3] != PCA_FULL_BIT) {
				fprintf(stderr,
					"[ERROR] CH%u 不在全关状态，拒绝 sweep\n",
					ch[a]);
				return 1;
			}
			/* 有可用记录就照它唤醒，没有只能取 home，后者会跳变。 */
			now[a] = (state_load(ch[a], &stored) &&
				  stored >= cfg->axis[a].safe_min &&
				  stored <= cfg->axis[a].safe_max)
					 ? stored
					 : home[a];
		} else {
			if (regs[0] || regs[1] || (regs[3] & PCA_FULL_BIT)) {
				fprintf(stderr,
					"[ERROR] CH%u 不是 ON=0 的普通舵机脉冲，拒绝 sweep\n",
					ch[a]);
				return 1;
			}
			now[a] = regs[2] | ((uint32_t)(regs[3] & 0x0f) << 8);
			if (now[a] < cfg->axis[a].safe_min ||
			    now[a] > cfg->axis[a].safe_max) {
				fprintf(stderr,
					"[ERROR] CH%u 当前 %u tick 不在工作范围 [%u, %u] 内\n",
					ch[a], now[a],
					cfg->axis[a].safe_min,
					cfg->axis[a].safe_max);
				return 1;
			}
		}
	}

	printf("\n[WARN] sweep 会让两轴同时动作 %u 轮，工作范围取自 %s：\n",
	       cycles, cfg_path);
	printf("       Pan (CH%u)  %u .. %u tick（%.2f .. %.2f ms）\n", ch[0],
	       cfg->axis[0].safe_min, cfg->axis[0].safe_max,
	       cfg->axis[0].safe_min * 20.0 / PCA_TICKS,
	       cfg->axis[0].safe_max * 20.0 / PCA_TICKS);
	printf("       Tilt(CH%u)  %u .. %u tick（%.2f .. %.2f ms）\n", ch[1],
	       cfg->axis[1].safe_min, cfg->axis[1].safe_max,
	       cfg->axis[1].safe_min * 20.0 / PCA_TICKS,
	       cfg->axis[1].safe_max * 20.0 / PCA_TICKS);
	printf("[WARN] 三条边都在同时驱动两个舵机，是电流最大的工况。\n");
	printf("[WARN] Pan 上端 %u tick = %.2f ms 超出 MG90S 规格上限，只在转角短暂经过、不驻留。\n",
	       cfg->axis[0].safe_max, cfg->axis[0].safe_max * 20.0 / PCA_TICKS);
	printf("[WARN] 请盯住电源电压与两轴动作：出现掉电重启、堵转声、异常跳动或明显发热立即 Ctrl-C。\n");
	if (fresh && now[0] == home[0] && now[1] == home[1])
		printf("[INFO] 唤醒按记录取在 home，不会跳变。\n");
	else if (!fresh)
		printf("[INFO] 接着上次的运行态，从当前位置继续，不会跳变。\n");
	else
		printf("[WARN] 起始位置取自 %s，若舵机不在该姿态会先跳一次。\n",
		       state_path());
	printf("[WARN] 手边保持可随时切断舵机电源。确认周围无遮挡后按 Enter 开始：");
	fflush(stdout);
	if (!fgets(line, sizeof(line), stdin)) {
		fprintf(stderr, "\n[ERROR] 未确认，未做任何输出\n");
		return 1;
	}

	/* PRE_SCALE 只能在 SLEEP 下改写；先把两轴脉宽写好再唤醒。 */
	if (fresh && write_reg(fd, addr, PCA_PRESCALE, CALIB_PRESCALE_50HZ) < 0) {
		perror("[ERROR] 设置 50 Hz 预分频");
		return 1;
	}
	for (int a = 0; a < 2; a++) {
		if (write_pulse(fd, addr, ch[a], (uint16_t)now[a], true) < 0) {
			perror("[ERROR] 预置起始脉宽");
			return 1;
		}
	}
	if (write_reg(fd, addr, PCA_MODE1,
		      mode1 & ~(PCA_SLEEP_BIT | PCA_RESTART_BIT)) < 0) {
		perror("[ERROR] 退出 SLEEP");
		return 1;
	}
	if ((mode1 & PCA_RESTART_BIT) &&
	    (safe_wait_ms(2) < 0 ||
	     write_reg(fd, addr, PCA_MODE1,
		       (mode1 & ~PCA_SLEEP_BIT) | PCA_RESTART_BIT) < 0)) {
		perror("[ERROR] 重启 PCA9685 振荡器");
		return 1;
	}
	if (safe_wait_ms(5) < 0 || read_reg(fd, addr, PCA_MODE1, &value) < 0 ||
	    (value & PCA_SLEEP_BIT)) {
		fprintf(stderr, "[ERROR] 振荡器未启动\n");
		return 1;
	}
	for (int a = 0; a < 2; a++)
		state_store(ch[a], now[a]);
	printf("[INFO] 两轴已输出初始脉宽，振荡器已启动。\n");

	/* 起点不在 home 时先按同样的节奏爬回去，保证每轮都从同一姿态出发。 */
	if (now[0] != home[0] || now[1] != home[1]) {
		if (sweep_leg(fd, addr, ch, now, home, "回 home") < 0)
			goto recover;
	}

	for (uint32_t c = 1; c <= cycles && !stop_requested; c++) {
		printf("[第 %u/%u 轮] 三角形一圈\n", c, cycles);

		corner[0] = cfg->axis[0].safe_min;
		corner[1] = cfg->axis[1].safe_max;
		if (sweep_leg(fd, addr, ch, now, corner, "pan 到 min / tilt 到 max") < 0)
			goto recover;

		corner[0] = cfg->axis[0].safe_max;
		corner[1] = cfg->axis[1].safe_min;
		if (sweep_leg(fd, addr, ch, now, corner, "对角：pan 到 max / tilt 到 min") < 0)
			goto recover;

		if (sweep_leg(fd, addr, ch, now, home, "回到 home") < 0)
			goto recover;
	}
	if (stop_requested)
		printf("\n[INFO] 收到停止信号\n");
	else
		printf("[INFO] %u 轮走完\n", cycles);
	rc = 0;

recover:
	/* 回收路径必须走完，因此清除停止标志，改为不受中断影响地爬回 home。 */
	stop_requested = 0;
	printf("[INFO] 回收：pan %u→%u, tilt %u→%u ... ", now[0], home[0],
	       now[1], home[1]);
	fflush(stdout);
	if (ramp_both(fd, addr, ch, now, home, now) < 0) {
		printf("失败\n");
		perror("[ERROR] 双轴爬回 home 失败，立即检查舵机");
		rc = 1;
	} else {
		printf("完成\n");
		for (int a = 0; a < 2; a++)
			state_store(ch[a], home[a]);
	}
	if (safe_wait_ms(CALIB_SETTLE_MS) < 0)
		rc = 1;
	/* 先全关再休眠，最后把预分频和 MODE1 还原成进入前的值。 */
	for (int a = 0; a < 2; a++) {
		uint8_t base = PCA_LED0_ON_L + ch[a] * 4;

		if (write_reg(fd, addr, base + 3, PCA_FULL_BIT) < 0) {
			perror("[ERROR] 关闭通道");
			rc = 1;
		}
	}
	if (write_reg(fd, addr, PCA_MODE1,
		      (mode1 & ~PCA_RESTART_BIT) | PCA_SLEEP_BIT) < 0 ||
	    write_reg(fd, addr, PCA_PRESCALE, prescale) < 0 ||
	    write_reg(fd, addr, PCA_MODE1,
		      (mode1 & ~PCA_RESTART_BIT) | PCA_SLEEP_BIT) < 0) {
		perror("[ERROR] 恢复 PCA9685 初始状态");
		rc = 1;
	}
	if (read_reg(fd, addr, PCA_MODE1, &value) < 0 || !(value & PCA_SLEEP_BIT)) {
		fprintf(stderr, "[ERROR] 恢复后 SLEEP 校验失败\n");
		rc = 1;
	}
	printf("[INFO] 两轴已回到 home 并关闭，PCA9685 已还原。\n");
	return rc;
}

int main(int argc, char **argv)
{
	char path[32];
	uint32_t bus, addr = PCA_ADDR_DEFAULT, channel = 0, min_tick = 0;
	uint32_t max_tick = 0, home_tick = 0, span = 0;
	uint32_t cycles = SWEEP_DEFAULT_CYCLES;
	struct pantilt_cfg cfg;
	uint8_t mode1, mode2, prescale;
	bool move, trial, calib, sweep;
	int delta = 0, fd, rc = 1;
	struct sigaction sa = { .sa_handler = on_signal };

	move = argc > 1 && strcmp(argv[1], "nudge") == 0;
	trial = argc > 1 && strcmp(argv[1], "trial") == 0;
	calib = argc > 1 && strcmp(argv[1], "calib") == 0;
	sweep = argc > 1 && strcmp(argv[1], "sweep") == 0;
	if ((!move && !trial && !calib && !sweep &&
	     (argc != 3 || strcmp(argv[1], "probe"))) ||
	    (move && argc != 7) || (trial && argc != 4) || (calib && argc != 6) ||
	    (sweep && argc != 3 && argc != 4)) {
		fprintf(stderr,
			"用法: %s probe BUS | trial BUS CHANNEL | calib BUS CHANNEL HOME_TICK SPAN_TICK | nudge BUS CHANNEL DELTA MIN_TICK MAX_TICK | sweep CONF_PATH [CYCLES]\n",
			argv[0]);
		return 2;
	}
	if (sweep) {
		/* sweep 的通道与限位全部来自配置，命令行不再重复一遍。 */
		if (cfg_load(argv[2], &cfg) < 0)
			return 2;
		bus = cfg.bus;
		addr = cfg.addr;
		if (argc == 4 &&
		    (!parse_u32(argv[3], SWEEP_MAX_CYCLES, &cycles) ||
		     cycles == 0)) {
			fprintf(stderr, "[ERROR] CYCLES 须为 1..%u\n",
				SWEEP_MAX_CYCLES);
			return 2;
		}
	} else if (!parse_u32(argv[2], 255, &bus) ||
		   ((move || trial || calib) &&
		    !parse_u32(argv[3], PCA_CHANNELS - 1, &channel)) ||
		   (calib && (!parse_u32(argv[4], PCA_TICKS - 1, &home_tick) ||
			      !parse_u32(argv[5], PCA_TICKS - 1, &span))) ||
		   (move && (!parse_delta(argv[4], &delta) ||
			     !parse_u32(argv[5], PCA_TICKS - 1, &min_tick) ||
			     !parse_u32(argv[6], PCA_TICKS - 1, &max_tick) ||
			     min_tick >= max_tick))) {
		fprintf(stderr, "[ERROR] 参数无效\n");
		return 2;
	}
	int n = snprintf(path, sizeof(path), "/dev/i2c-%u", bus);

	if (n < 0 || n >= (int)sizeof(path))
		return 2;
	fd = open(path, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		perror("[ERROR] 打开 I2C 总线");
		return 1;
	}
	if (read_reg(fd, addr, PCA_MODE1, &mode1) < 0 ||
	    read_reg(fd, addr, PCA_MODE2, &mode2) < 0 ||
	    read_reg(fd, addr, PCA_PRESCALE, &prescale) < 0) {
		fprintf(stderr, "[ERROR] %s 地址 0x%02x 无响应: %s\n",
			path, addr, strerror(errno));
		goto out;
	}
	printf("[INFO] %s 地址 0x%02x 有响应: MODE1=0x%02x MODE2=0x%02x PRE_SCALE=0x%02x\n",
	       path, addr, mode1, mode2, prescale);
	if (mode1 & PCA_SLEEP_BIT)
		printf("[WARN] MODE1 bit4=SLEEP，当前 PWM 输出未启动\n");
	if (!move && !trial && !calib && !sweep) {
		rc = 0;
		goto out;
	}
	/*
	 * 互斥保护：pantilt.ko 认领 PCA9685 后会由 delayed_work 持续写脉宽。
	 * 此时再用本工具直接写同一颗芯片，两路会互相覆盖目标值，舵机会不受控地乱动。
	 * probe 是只读的，不做拦截；其余会动舵机的子命令一律先检查设备节点。
	 */
	if (access("/dev/pantilt0", F_OK) == 0 && getenv("PANTILT_ALLOW_CONFLICT") == NULL) {
		fprintf(stderr,
			"[ERROR] /dev/pantilt0 已存在：pantilt.ko 正在占用 PCA9685。\n"
			"        两者同时写同一颗芯片会让舵机不受控，已拒绝执行。\n"
			"        处理：先 /sbin/rmmod pantilt，或改用 ./pantilt_ctl 下发目标；\n"
			"        确知风险仍要强行并存时设 PANTILT_ALLOW_CONFLICT=1。\n");
		goto out;
	}
	if (sigaction(SIGINT, &sa, NULL) < 0 ||
	    sigaction(SIGTERM, &sa, NULL) < 0 ||
	    sigaction(SIGHUP, &sa, NULL) < 0) {
		perror("[ERROR] 安装停止信号处理器");
		goto out;
	}
	if (trial)
		rc = center_trial(fd, addr, channel, mode1, mode2, prescale);
	else if (calib)
		rc = calib_session(fd, addr, channel, home_tick, span, mode1,
				   mode2, prescale);
	else if (sweep)
		rc = sweep_run(fd, addr, &cfg, argv[2], cycles, mode1, mode2,
			       prescale);
	else
		rc = nudge(fd, addr, channel, delta, min_tick, max_tick,
			   mode1, prescale);
out:
	if (close(fd) < 0) {
		perror("[ERROR] 关闭 I2C 设备");
		rc = 1;
	}
	return rc;
}
