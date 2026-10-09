/*
 * audio_play.c —— Phase 5.2 播放程序（ALSA 用户态）
 *
 * 两种用途：
 *   1) 播放 WAV 文件（用于回放本程序录下的文件，验证录音链路）：
 *        audio_play --file /root/imx6hub_demo/records/test.wav
 *   2) 生成并播放测试音/告警音（不依赖任何素材文件）：
 *        audio_play --tone 1000 --duration 2
 *        audio_play --alert --duration 3
 *
 * 底本参考：reference/.../28_alsa-lib/pcm_playback.c（ALIENTEK 邓涛）
 * 修正了底本按 period_size 而非实际写入帧数记账、无 XRUN 恢复的问题。
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
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_DEVICE   "hw:0,0"
#define DEFAULT_RATE     44100
#define DEFAULT_CHANNELS 2
#define DEFAULT_FORMAT   "S16_LE"
#define DEFAULT_PERIOD   1024
#define DEFAULT_PERIODS  4
#define DEFAULT_TONE_HZ  1000.0
#define DEFAULT_TONE_SEC 2.0
#define DEFAULT_ALERT_SEC 3.0

/* 告警音：880/1320 Hz 双音交替，0.25 s 响 + 0.15 s 停，带 10 ms 斜坡防爆音。 */
#define ALERT_ON     0.25
#define ALERT_OFF    0.15
#define ALERT_RAMP   0.01
#define ALERT_FREQ_A 880.0
#define ALERT_FREQ_B 1320.0
#define TONE_AMP     0.35
#define FADE_SEC     0.02

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
"用法: %s [选项] [文件]\n"
"  -D, --device DEV     ALSA PCM 设备名（默认 %s）\n"
"  -F, --file PATH      播放 WAV 文件（PCM，8/16/24/32 位）\n"
"  -t, --tone HZ        生成单音（Hz）\n"
"      --alert          生成告警双音（880/1320 Hz 交替）\n"
"  -d, --duration SEC   播放/生成时长上限（单音默认 %.1f s，告警默认 %.1f s）\n"
"  -r, --rate HZ        生成音频的采样率（默认 %u）\n"
"  -c, --channels N     生成音频的声道数（默认 %u）\n"
"  -f, --format FMT     生成音频的采样格式（默认 %s）\n"
"      --gain PCT       软件增益 0..200（默认 100；仅 U8/S16_LE/S32_LE 生效）\n"
"      --period-size N  周期大小（帧，默认 %d）\n"
"      --periods N      周期数（默认 %d）\n"
"      --quiet          不打印进度\n"
"  -h, --help           显示本帮助\n"
"\n"
"播放文件时按文件自身的采样率/声道/位宽打开设备，-r/-c/-f 被忽略；\n"
"--duration 在文件模式下表示“只播前 N 秒”。\n"
"--file/-t/--alert 都没给时，默认播放一次告警音。\n",
		prog, DEFAULT_DEVICE, DEFAULT_TONE_SEC, DEFAULT_ALERT_SEC,
		DEFAULT_RATE, DEFAULT_CHANNELS, DEFAULT_FORMAT,
		DEFAULT_PERIOD, DEFAULT_PERIODS);
}

/* 软件增益：只对定宽样本格式做，避免在 S24_3LE 这类打包格式上写出错。 */
static void apply_gain(void *buf, size_t bytes, snd_pcm_format_t fmt, double gain)
{
	size_t i;

	if (gain == 1.0)
		return;
	switch (fmt) {
	case SND_PCM_FORMAT_U8: {
		uint8_t *p = buf;
		for (i = 0; i < bytes; i++) {
			double v = ((double)p[i] - 128.0) * gain + 128.0;
			if (v < 0.0) v = 0.0;
			if (v > 255.0) v = 255.0;
			p[i] = (uint8_t)(v + 0.5);
		}
		break;
	}
	case SND_PCM_FORMAT_S16_LE: {
		int16_t *p = buf;
		for (i = 0; i < bytes / 2; i++) {
			double v = (double)p[i] * gain;
			if (v < -32768.0) v = -32768.0;
			if (v > 32767.0) v = 32767.0;
			p[i] = (int16_t)v;
		}
		break;
	}
	case SND_PCM_FORMAT_S32_LE: {
		int32_t *p = buf;
		for (i = 0; i < bytes / 4; i++) {
			double v = (double)p[i] * gain;
			if (v < -2147483648.0) v = -2147483648.0;
			if (v > 2147483647.0) v = 2147483647.0;
			p[i] = (int32_t)v;
		}
		break;
	}
	default:
		fprintf(stderr, "audio: WARN --gain 不支持格式 %s，本次未加增益\n",
			snd_pcm_format_name(fmt));
		break;
	}
}

/* 把 [-1,1] 的浮点样本按目标格式写到缓冲的第 idx 个样本位。 */
static void store_sample(void *buf, unsigned long long idx, snd_pcm_format_t fmt,
			 double v)
{
	if (v > 1.0) v = 1.0;
	if (v < -1.0) v = -1.0;
	switch (fmt) {
	case SND_PCM_FORMAT_U8:
		((uint8_t *)buf)[idx] = (uint8_t)((v * 127.0) + 128.0);
		break;
	case SND_PCM_FORMAT_S16_LE:
		((int16_t *)buf)[idx] = (int16_t)(v * 32767.0);
		break;
	case SND_PCM_FORMAT_S32_LE:
		((int32_t *)buf)[idx] = (int32_t)(v * 2147483647.0);
		break;
	default:
		break;
	}
}

/* 告警包络：0.25 s 响 + 0.15 s 停，响的段首尾各 10 ms 线性斜坡。 */
static double alert_envelope(double t, double cycle, double on, double ramp)
{
	double x = fmod(t, cycle);

	if (x >= on)
		return 0.0;
	if (x < ramp)
		return x / ramp;
	if (x > on - ramp)
		return (on - x) / ramp;
	return 1.0;
}

/*
 * 生成一批帧。返回实际生成的帧数（接近 total_frames 时可能不足 frames）。
 * 相位直接用 t 计算（无状态），因此可以任意分块，不会因分块产生相位跳变。
 */
static unsigned long long gen_block(struct ac_pcm *h, void *buf,
				    unsigned long long frames,
				    unsigned long long frame_index,
				    unsigned long long total_frames,
				    int alert, double freq, double gain)
{
	unsigned long long done;
	unsigned int c;
	double total_sec = (double)total_frames / (double)h->rate;

	for (done = 0; done < frames; done++) {
		unsigned long long f = frame_index + done;
		double t, env, s, f_now;

		if (f >= total_frames)
			break;
		t = (double)f / (double)h->rate;

		if (alert) {
			double cyc = floor(t / (ALERT_ON + ALERT_OFF));

			env = alert_envelope(t, ALERT_ON + ALERT_OFF, ALERT_ON, ALERT_RAMP);
			f_now = ((unsigned long long)cyc & 1ULL) ? ALERT_FREQ_B : ALERT_FREQ_A;
		} else {
			env = 1.0;
			f_now = freq;
		}

		/* 全局淡入/淡出，避免开头结尾的咔哒声。 */
		if (t < FADE_SEC)
			env *= t / FADE_SEC;
		if (total_sec - t < FADE_SEC)
			env *= (total_sec - t) / FADE_SEC;

		s = sin(2.0 * M_PI * f_now * t) * env * TONE_AMP * gain;
		for (c = 0; c < h->channels; c++)
			store_sample(buf, done * h->channels + c, h->fmt, s);
	}
	return done;
}

int main(int argc, char *argv[])
{
	static struct option long_opts[] = {
		{"device",      required_argument, NULL, 'D'},
		{"file",        required_argument, NULL, 'F'},
		{"tone",        required_argument, NULL, 't'},
		{"alert",       no_argument,       NULL, 1},
		{"duration",    required_argument, NULL, 'd'},
		{"rate",        required_argument, NULL, 'r'},
		{"channels",    required_argument, NULL, 'c'},
		{"format",      required_argument, NULL, 'f'},
		{"gain",        required_argument, NULL, 2},
		{"period-size", required_argument, NULL, 3},
		{"periods",     required_argument, NULL, 4},
		{"quiet",       no_argument,       NULL, 5},
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
	struct ac_wav_info info;
	struct ac_pcm h;
	struct sigaction sa;
	const char *file = NULL;
	double tone = -1.0, duration = -1.0, gain = 1.0;
	int alert = 0, quiet = 0;
	int fd = -1, ret = 0, ch, t_ok = 0;
	void *buf = NULL;
	unsigned long long played = 0, total_frames = 0, frame_index = 0;
	double t_begin = 0.0, t_end = 0.0, t_progress = 0.0;
	size_t buf_bytes;

	memset(&info, 0, sizeof(info));

	setvbuf(stdout, NULL, _IOLBF, 0);

	while ((ch = getopt_long(argc, argv, "D:F:t:d:r:c:f:h", long_opts, NULL)) != -1) {
		switch (ch) {
		case 'D': cfg.device = optarg; break;
		case 'F': file = optarg; break;
		case 't': tone = strtod(optarg, NULL); break;
		case 1: alert = 1; break;
		case 'd': duration = strtod(optarg, NULL); break;
		case 'r': cfg.rate = (unsigned int)strtoul(optarg, NULL, 0); break;
		case 'c': cfg.channels = (unsigned int)strtoul(optarg, NULL, 0); break;
		case 'f': cfg.format = optarg; break;
		case 2: gain = strtod(optarg, NULL) / 100.0; break;
		case 3: cfg.period_size = (snd_pcm_uframes_t)strtoul(optarg, NULL, 0); break;
		case 4: cfg.periods = (unsigned int)strtoul(optarg, NULL, 0); break;
		case 5: quiet = 1; break;
		case 'h': usage(argv[0]); return 0;
		default: usage(argv[0]); return 2;
		}
	}
	if (file == NULL && optind < argc)
		file = argv[optind++];
	if (optind < argc) {
		fprintf(stderr, "audio: 不认识的参数 '%s'\n", argv[optind]);
		usage(argv[0]);
		return 2;
	}
	if (file == NULL && tone < 0.0 && !alert)
		alert = 1;                      /* 都没给：默认放一次告警音 */
	if (file == NULL && tone == 0.0) {
		fprintf(stderr, "audio: --tone 频率不能为 0\n");
		return 2;
	}
	if (gain < 0.0 || gain > 2.0) {
		fprintf(stderr, "audio: --gain 需在 0..200 之间\n");
		return 2;
	}
	if (cfg.rate == 0 || cfg.channels == 0 || cfg.period_size == 0 || cfg.periods == 0) {
		fprintf(stderr, "audio: rate/channels/period-size/periods 必须为正数\n");
		return 2;
	}

	sa.sa_handler = on_signal;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;                        /* 不设 SA_RESTART：让 writei 能被信号打断 */
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	/* 文件模式：先解析文件头，再用文件自身的参数打开设备。 */
	if (file != NULL) {
		off_t end, avail;

		fd = open(file, O_RDONLY);
		if (fd < 0) {
			fprintf(stderr, "audio: 打开 %s 失败: %s\n", file, strerror(errno));
			return 1;
		}
		if (ac_wav_parse(fd, &info, file) < 0) {
			close(fd);
			return 1;
		}
		/*
		 * 录音被 SIGKILL 之类硬打断时 data_size 可能为 0 或大于实际长度，
		 * 这种情况下按文件实际长度播放，而不是直接报错。
		 */
		end = lseek(fd, 0, SEEK_END);
		avail = end - info.data_offset;
		if (avail < 0)
			avail = 0;
		if (info.data_size == 0 || (off_t)info.data_size > avail) {
			if (info.data_size == 0)
				fprintf(stderr,
					"audio: WARN %s 的 data 长度声明为 0（可能被硬打断），按实际 %lld 字节播放\n",
					file, (long long)avail);
			else
				fprintf(stderr,
					"audio: WARN %s 头里声明 %u 字节，实际只有 %lld 字节，按实际播放\n",
					file, info.data_size, (long long)avail);
			info.data_size = (uint32_t)((unsigned long long)avail > 0xFFFFFFFFULL
						    ? 0xFFFFFFFFULL
						    : (unsigned long long)avail);
		}

		cfg.rate = info.rate;
		cfg.channels = info.channels;
		cfg.format = snd_pcm_format_name(info.fmt);
		printf("audio: 文件 %s | %s %u Hz %u ch | data %u 字节\n",
		       file, cfg.format, info.rate, info.channels, info.data_size);
	}

	if (ac_pcm_open(&h, &cfg, SND_PCM_STREAM_PLAYBACK) < 0) {
		if (fd >= 0)
			close(fd);
		return 1;
	}

	/* 待播总帧数：两个模式都先算出来，主循环因此只有一条分支。 */
	if (file != NULL) {
		unsigned long long file_frames = info.data_size / h.frame_bytes;

		if (duration > 0.0) {
			unsigned long long cap = (unsigned long long)(duration * h.rate + 0.5);
			total_frames = cap < file_frames ? cap : file_frames;
			if (cap < file_frames)
				printf("audio: --duration %.2f s，只播文件前 %llu 帧\n",
				       duration, total_frames);
		} else {
			total_frames = file_frames;
		}
		if (lseek(fd, info.data_offset, SEEK_SET) != info.data_offset) {
			fprintf(stderr, "audio: 定位到 data 失败: %s\n", strerror(errno));
			ret = 1;
			goto out;
		}
		printf("audio: 待播 %llu 帧 (%.2f s)\n", total_frames,
		       h.rate ? total_frames / (double)h.rate : 0.0);
	} else {
		if (duration < 0.0)
			duration = alert ? DEFAULT_ALERT_SEC : DEFAULT_TONE_SEC;
		total_frames = (unsigned long long)(duration * h.rate + 0.5);
		if (alert)
			printf("audio: 生成告警双音(880/1320 Hz) | %.2f s\n", duration);
		else
			printf("audio: 生成单音 %.1f Hz | %.2f s\n", tone, duration);
	}

	buf_bytes = (size_t)h.period_size * h.frame_bytes;
	buf = malloc(buf_bytes);
	if (buf == NULL) {
		fprintf(stderr, "audio: malloc 失败\n");
		ret = 1;
		goto out;
	}

	t_progress = t_begin = now_seconds();
	t_ok = 1;

	while (!g_stop && frame_index < total_frames) {
		unsigned long long produced;
		snd_pcm_uframes_t want = h.period_size;

		if (total_frames - frame_index < (unsigned long long)want)
			want = (snd_pcm_uframes_t)(total_frames - frame_index);

		if (file != NULL) {
			ssize_t got = read(fd, buf, (size_t)want * h.frame_bytes);

			if (got < 0) {
				fprintf(stderr, "audio: 读文件失败: %s\n", strerror(errno));
				ret = 1;
				goto out;
			}
			if (got == 0)
				break;
			produced = (unsigned long long)got / h.frame_bytes;
			if (produced == 0)
				break;
			apply_gain(buf, (size_t)produced * h.frame_bytes, h.fmt, gain);
		} else {
			produced = gen_block(&h, buf, want, frame_index, total_frames,
					     alert, tone, gain);
			if (produced == 0)
				break;
		}
		frame_index += produced;

		{
			snd_pcm_uframes_t off = 0;

			while (off < (snd_pcm_uframes_t)produced) {
				snd_pcm_sframes_t n =
					ac_pcm_write(&h,
						     (char *)buf + (size_t)off * h.frame_bytes,
						     (snd_pcm_uframes_t)produced - off);

				if (n == AC_EINTR)
					break;
				if (n < 0) {
					fprintf(stderr, "audio: 播放 I/O 失败\n");
					ret = 1;
					goto out;
				}
				if (n == 0)
					break;
				off += (snd_pcm_uframes_t)n;
			}
			played += off;
			if (off < (snd_pcm_uframes_t)produced) {
				/* 被信号打断：不再继续喂，交给下面的 drain 收尾。 */
				break;
			}
		}

		if (!quiet) {
			double now = now_seconds();

			if (now - t_progress >= 1.0) {
				fprintf(stderr, "[%6.1fs] played=%llu / %llu 帧\n",
					now - t_begin, played, total_frames);
				t_progress = now;
			}
		}
	}

	if (!g_stop)
		snd_pcm_drain(h.pcm);
	t_end = now_seconds();

out:
	ac_pcm_stats_print(&h, stderr);
	ac_pcm_result_print(&h, "play", -1,
			    t_ok ? (t_end > 0.0 ? t_end - t_begin : now_seconds() - t_begin)
				 : 0.0,
			    stdout);
	{
		double dur = h.rate ? played / (double)h.rate : 0.0;
		double wall = t_ok ? (t_end > 0.0 ? t_end - t_begin : now_seconds() - t_begin)
				   : 0.0;

		printf("audio: 播放完成 %llu 帧 (%.2f s 音频)，耗时 %.2f s",
		       played, dur, wall);
		if (dur > 0.01 && wall > 0.01)
			printf("，实时率 %.2fx", wall / dur);
		printf("%s\n", g_stop ? "（被信号打断）" : "");
	}

	free(buf);
	ac_pcm_close(&h);
	if (fd >= 0)
		close(fd);
	return ret;
}
