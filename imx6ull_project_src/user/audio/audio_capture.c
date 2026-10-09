/*
 * audio_capture.c —— Phase 5.1 录音程序（ALSA 用户态）
 *
 * 用法：
 *   audio_capture --output /root/imx6hub_demo/records/test.wav --duration 10
 *   audio_capture --dir /root/imx6hub_demo/records --tag alert --duration 30
 *   audio_capture --duration 0            # 一直录，收到 SIGINT/SIGTERM 后正常收尾
 *
 * 职责（对应总计划 Phase 5.1）：
 *   创建目标文件 → 初始化 WAV/PCM 参数 → snd_pcm_open → 设置 hw_params
 *   → snd_pcm_readi → 处理 XRUN → 写数据 → 正常结束并关闭文件。
 *
 * 若输出 WAV，程序自己维护 WAV 头；正常退出（含被信号打断）时回填长度，
 * 所以 securityd 可以用 SIGTERM 停止录音并拿到一个可播放的文件。
 *
 * 不做任何降噪/滤波 —— 那是 Phase 5.4 的可选增强，本轮明确不做。
 */
#define _GNU_SOURCE
#include "audio_common.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_DEVICE   "hw:0,0"
#define DEFAULT_RATE     44100
#define DEFAULT_CHANNELS 2
#define DEFAULT_FORMAT   "S16_LE"
#define DEFAULT_DIR      "/root/imx6hub_demo/records"
#define DEFAULT_TAG      "rec"
#define DEFAULT_DURATION 10
#define DEFAULT_PERIOD   1024
#define DEFAULT_PERIODS  4
#define PATH_MAX_LEN     512

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

static double now_seconds(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);
	return (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
}

static void usage(const char *prog)
{
	fprintf(stderr,
"用法: %s [选项]\n"
"  -D, --device DEV     ALSA PCM 设备名（默认 %s）\n"
"  -r, --rate HZ        采样率（默认 %u）\n"
"  -c, --channels N     声道数（默认 %u）\n"
"  -f, --format FMT     采样格式（默认 %s，可选 S16_LE/S32_LE/U8/S24_3LE 等）\n"
"  -d, --duration SEC   录音时长秒；0 = 直到收到 SIGINT/SIGTERM（默认 %u）\n"
"  -o, --output PATH    输出文件路径；给了就用它，文件名不做加工\n"
"      --dir DIR        未给 --output 时的保存目录（默认 %s）\n"
"      --tag TAG        未给 --output 时的文件名标签（默认 %s）\n"
"      --period-size N  周期大小（帧，默认 %d）\n"
"      --periods N      周期数（默认 %d）\n"
"      --raw            写裸 PCM，不写 WAV 头\n"
"      --quiet          不打印进度\n"
"  -h, --help           显示本帮助\n"
"\n"
"未给 --output 时文件名形如 YYYYMMDD_HHMMSS_<tag>.wav。\n",
		prog, DEFAULT_DEVICE, DEFAULT_RATE, DEFAULT_CHANNELS, DEFAULT_FORMAT,
		DEFAULT_DURATION, DEFAULT_DIR, DEFAULT_TAG,
		DEFAULT_PERIOD, DEFAULT_PERIODS);
}

/*
 * 周期峰值电平，用于"录到的到底是不是静音"这一观察项。
 * 返回归一化到 [-1,1] 的峰值；无法识别的格式返回 -1。
 */
static double peak_level(const void *buf, snd_pcm_uframes_t frames,
			 unsigned int channels, snd_pcm_format_t fmt)
{
	unsigned int i, total = (unsigned int)frames * channels;
	double peak = 0.0;

	switch (fmt) {
	case SND_PCM_FORMAT_U8: {
		const uint8_t *p = buf;
		for (i = 0; i < total; i++) {
			double v = fabs(((double)p[i] - 128.0) / 128.0);
			if (v > peak) peak = v;
		}
		break;
	}
	case SND_PCM_FORMAT_S16_LE: {
		const int16_t *p = buf;
		for (i = 0; i < total; i++) {
			double v = fabs((double)p[i] / 32768.0);
			if (v > peak) peak = v;
		}
		break;
	}
	case SND_PCM_FORMAT_S32_LE: {
		const int32_t *p = buf;
		for (i = 0; i < total; i++) {
			double v = fabs((double)p[i] / 2147483648.0);
			if (v > peak) peak = v;
		}
		break;
	}
	case SND_PCM_FORMAT_S24_3LE: {
		const uint8_t *p = buf;
		for (i = 0; i < total; i++) {
			int32_t s = (int32_t)((uint32_t)p[3 * i] << 8 |
					      (uint32_t)p[3 * i + 1] << 16 |
					      (uint32_t)p[3 * i + 2] << 24);
			double v = fabs((double)(s >> 8) / 8388608.0);
			if (v > peak) peak = v;
		}
		break;
	}
	default:
		return -1.0;
	}
	return peak;
}

static int mkdir_p(const char *path)
{
	char tmp[PATH_MAX_LEN];
	size_t len;
	char *p;

	if (strlen(path) >= sizeof(tmp)) {
		fprintf(stderr, "audio: 目录路径过长: %s\n", path);
		return -1;
	}
	strcpy(tmp, path);
	len = strlen(tmp);
	while (len > 1 && tmp[len - 1] == '/')
		tmp[--len] = '\0';

	for (p = tmp + 1; *p; p++) {
		if (*p != '/')
			continue;
		*p = '\0';
		if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
			fprintf(stderr, "audio: mkdir %s 失败: %s\n", tmp, strerror(errno));
			return -1;
		}
		*p = '/';
	}
	if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
		fprintf(stderr, "audio: mkdir %s 失败: %s\n", tmp, strerror(errno));
		return -1;
	}
	return 0;
}

int main(int argc, char *argv[])
{
	static struct option long_opts[] = {
		{"device",      required_argument, NULL, 'D'},
		{"rate",        required_argument, NULL, 'r'},
		{"channels",    required_argument, NULL, 'c'},
		{"format",      required_argument, NULL, 'f'},
		{"duration",    required_argument, NULL, 'd'},
		{"output",      required_argument, NULL, 'o'},
		{"dir",         required_argument, NULL, 1},
		{"tag",         required_argument, NULL, 2},
		{"period-size", required_argument, NULL, 3},
		{"periods",     required_argument, NULL, 4},
		{"raw",         no_argument,       NULL, 5},
		{"quiet",       no_argument,       NULL, 6},
		{"help",        no_argument,       NULL, 'h'},
		{NULL, 0, NULL, 0}
	};
	struct ac_pcm_cfg cfg = {
		.device = DEFAULT_DEVICE,
		.rate = DEFAULT_RATE,
		.channels = DEFAULT_CHANNELS,
		.format = DEFAULT_FORMAT,
		.period_size = DEFAULT_PERIOD,
		.periods = DEFAULT_PERIODS,
	};
	struct ac_pcm h;
	struct ac_wav_hdr wav_hdr;
	struct sigaction sa;
	char path[PATH_MAX_LEN];
	const char *output = NULL;
	const char *dir = DEFAULT_DIR;
	const char *tag = DEFAULT_TAG;
	double duration = DEFAULT_DURATION;
	int raw = 0, quiet = 0;
	int fd = -1, ret = 0;
	void *buf = NULL;
	unsigned long long data_bytes = 0, want_frames = 0, got_frames = 0;
	double t_start = 0.0, t_last = 0.0, t_now;
	int ch;

	/* 行缓冲：进度打在 stderr、结果打在 stdout，日志顺序才不会错乱。 */
	setvbuf(stdout, NULL, _IOLBF, 0);

	while ((ch = getopt_long(argc, argv, "D:r:c:f:d:o:h", long_opts, NULL)) != -1) {
		switch (ch) {
		case 'D': cfg.device = optarg; break;
		case 'r': cfg.rate = (unsigned int)strtoul(optarg, NULL, 0); break;
		case 'c': cfg.channels = (unsigned int)strtoul(optarg, NULL, 0); break;
		case 'f': cfg.format = optarg; break;
		case 'd': duration = strtod(optarg, NULL); break;
		case 'o': output = optarg; break;
		case 1: dir = optarg; break;
		case 2: tag = optarg; break;
		case 3: cfg.period_size = (snd_pcm_uframes_t)strtoul(optarg, NULL, 0); break;
		case 4: cfg.periods = (unsigned int)strtoul(optarg, NULL, 0); break;
		case 5: raw = 1; break;
		case 6: quiet = 1; break;
		case 'h': usage(argv[0]); return 0;
		default: usage(argv[0]); return 2;
		}
	}
	if (optind < argc) {
		fprintf(stderr, "audio: 不认识的参数 '%s'\n", argv[optind]);
		usage(argv[0]);
		return 2;
	}
	if (cfg.rate == 0 || cfg.channels == 0 || cfg.period_size == 0 || cfg.periods == 0) {
		fprintf(stderr, "audio: rate/channels/period-size/periods 必须为正数\n");
		return 2;
	}
	if (duration < 0) {
		fprintf(stderr, "audio: duration 不能为负\n");
		return 2;
	}
	if (strchr(tag, '/') != NULL) {
		fprintf(stderr, "audio: tag 不能含 '/'\n");
		return 2;
	}

	/* 目标路径：给了 --output 就原样用；否则用目录 + 时间戳 + tag 拼。 */
	if (output != NULL) {
		if (strlen(output) >= sizeof(path)) {
			fprintf(stderr, "audio: --output 路径过长\n");
			return 2;
		}
		strcpy(path, output);
	} else {
		time_t now = time(NULL);
		struct tm tm;
		char stamp[32];

		localtime_r(&now, &tm);
		strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);
		if (snprintf(path, sizeof(path), "%s/%s_%s.%s", dir, stamp, tag,
			     raw ? "pcm" : "wav") >= (int)sizeof(path)) {
			fprintf(stderr, "audio: 生成的文件路径过长\n");
			return 2;
		}
		if (mkdir_p(dir) != 0)
			return 1;
	}

	sa.sa_handler = on_signal;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;                        /* 不设 SA_RESTART：让 readi 能被信号打断 */
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (fd < 0) {
		fprintf(stderr, "audio: 打开输出文件 %s 失败: %s\n", path, strerror(errno));
		return 1;
	}

	if (ac_pcm_open(&h, &cfg, SND_PCM_STREAM_CAPTURE) < 0) {
		close(fd);
		return 1;
	}

	if (!raw) {
		unsigned int bits = (unsigned int)snd_pcm_format_physical_width(h.fmt);

		ac_wav_make_hdr(&wav_hdr, h.rate, h.channels, bits);
		if (ac_wav_write_hdr(fd, &wav_hdr) < 0) {
			ret = 1;
			goto out;
		}
	}

	buf = malloc((size_t)h.period_size * h.frame_bytes);
	if (buf == NULL) {
		fprintf(stderr, "audio: malloc 失败\n");
		ret = 1;
		goto out;
	}

	if (duration > 0)
		want_frames = (unsigned long long)(duration * (double)h.rate + 0.5);

	printf("audio: 录制 -> %s%s\n", path, duration > 0 ? "" : "（直到收到 SIGINT/SIGTERM）");

	t_start = t_last = now_seconds();
	while (!g_stop) {
		snd_pcm_uframes_t want = h.period_size;
		snd_pcm_sframes_t n;
		double peak;

		if (want_frames > 0) {
			if (got_frames >= want_frames)
				break;
			if ((unsigned long long)want > want_frames - got_frames)
				want = (snd_pcm_uframes_t)(want_frames - got_frames);
		}

		n = ac_pcm_read(&h, buf, want);
		if (n == AC_EINTR) {
			if (g_stop)
				break;
			continue;
		}
		if (n < 0) {
			fprintf(stderr, "audio: 录音 I/O 失败，已写入 %llu 字节\n", data_bytes);
			ret = 1;
			goto out;
		}
		if (n == 0)
			continue;

		got_frames += (unsigned long long)n;
		if (write(fd, buf, (size_t)n * h.frame_bytes) != (ssize_t)n * (ssize_t)h.frame_bytes) {
			fprintf(stderr, "audio: 写文件失败: %s\n", strerror(errno));
			ret = 1;
			goto out;
		}
		data_bytes += (unsigned long long)n * h.frame_bytes;

		if (quiet)
			continue;
		t_now = now_seconds();
		if (t_now - t_last >= 1.0) {
			peak = peak_level(buf, (snd_pcm_uframes_t)n, h.channels, h.fmt);
			if (peak < 0.0)
				fprintf(stderr, "[%6.1fs] frames=%llu peak=n/a\n",
					got_frames / (double)h.rate, got_frames);
			else if (peak <= 0.0)
				fprintf(stderr, "[%6.1fs] frames=%llu peak=(静音)\n",
					got_frames / (double)h.rate, got_frames);
			else
				fprintf(stderr, "[%6.1fs] frames=%llu peak=%.1f dBFS\n",
					got_frames / (double)h.rate, got_frames,
					20.0 * log10(peak));
			t_last = t_now;
		}
	}

out:
	if (!raw && fd >= 0)
		ac_wav_finalize(fd, &wav_hdr, (uint32_t)data_bytes);

	ac_pcm_stats_print(&h, stderr);
	ac_pcm_result_print(&h, "capture", (long long)data_bytes,
			    t_start > 0.0 ? now_seconds() - t_start : 0.0, stdout);
	printf("audio: 完成 %s | 帧=%llu | 时长=%.2f s | 数据=%llu 字节%s\n",
	       path, got_frames, h.rate ? got_frames / (double)h.rate : 0.0,
	       data_bytes, raw ? "（裸 PCM）" : "（WAV）");
	if (g_stop)
		printf("audio: 收到停止信号，已按正常路径收尾（WAV 头已回填）\n");

	free(buf);
	ac_pcm_close(&h);
	if (fd >= 0 && close(fd) != 0) {
		fprintf(stderr, "audio: 关闭文件失败: %s\n", strerror(errno));
		ret = 1;
	}
	return ret;
}
