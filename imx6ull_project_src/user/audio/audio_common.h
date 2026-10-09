/*
 * audio_common.h —— Phase 5 录音/播放程序的公共部分
 *
 * 只做两件事：
 *   1) 把 ALSA PCM 的打开、参数协商、读写与 XRUN 恢复收敛到一处；
 *   2) 提供最小 WAV(PCM) 头读写的辅助函数。
 *
 * 不包含任何降噪/滤波（Phase 5.4 可选增强，本轮明确不做）。
 */
#ifndef AUDIO_COMMON_H
#define AUDIO_COMMON_H

#include <alsa/asoundlib.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

/* ---------------------------------------------------------------- PCM */

/*
 * XRUN/错误计数。rules/imx6ull-pantilt-engineering.md §6 要求 ALSA XRUN 可观察，
 * 所以计数放在句柄里，程序退出前打印。
 */
struct ac_stats {
	unsigned long long xrun;        /* 欠载/过载（EPIPE）次数 */
	unsigned long long suspend;     /* ESTRPIPE 恢复次数 */
	unsigned long long frames;      /* 成功搬运的总帧数 */
	unsigned long long short_io;    /* 单次搬运不足请求帧数的次数 */
};

struct ac_pcm_cfg {
	const char *device;             /* ALSA 设备名，如 "hw:0,0" */
	unsigned int rate;              /* 采样率 Hz */
	unsigned int channels;          /* 声道数 */
	const char *format;             /* 采样格式名，如 "S16_LE" */
	snd_pcm_uframes_t period_size;  /* 周期大小（帧） */
	unsigned int periods;           /* 周期数（设备缓冲 = period_size * periods） */
};

struct ac_pcm {
	snd_pcm_t *pcm;
	char device[64];                /* 实际打开的设备名（副本，供日志用） */
	snd_pcm_format_t fmt;
	unsigned int rate;
	unsigned int channels;
	unsigned int frame_bytes;       /* 一帧的字节数（按物理位宽计算） */
	snd_pcm_uframes_t period_size;
	unsigned int periods;
	snd_pcm_uframes_t buffer_frames;/* 协商后设备实际的缓冲帧数 */
	struct ac_stats stats;
};

/* 把 "S16_LE" 之类的名字转成 snd_pcm_format_t；失败返回 -1。 */
int ac_format_parse(const char *name, snd_pcm_format_t *out);

/*
 * 打开 PCM 并按 cfg 设置 hw_params（交错访问、cfg 指定的格式/采样率/声道/周期）。
 * 采样率用 set_rate_near，实际生效值写回 h->rate；若与请求不同会打一条 WARN。
 * 成功返回 0，失败返回 -1 并已打印原因。
 */
int ac_pcm_open(struct ac_pcm *h, const struct ac_pcm_cfg *cfg,
		snd_pcm_stream_t stream);

void ac_pcm_close(struct ac_pcm *h);

void ac_pcm_stats_print(const struct ac_pcm *h, FILE *fp);

/*
 * 一行纯 ASCII 的机器可读结果，供脚本/securityd 解析（也是可观察性的一部分）：
 *   audio: RESULT op=capture dev=hw:0,0 frames=132300 rate=44100 xrun=0
 *          suspend=0 short_io=0 bytes=529200 wall_s=3.01
 * bytes < 0 时省略该字段。
 */
void ac_pcm_result_print(const struct ac_pcm *h, const char *op,
			 long long bytes, double wall_s, FILE *fp);

/*
 * 读写一个周期。返回实际帧数（>0）；被信号打断返回 AC_EINTR；
 * 不可恢复的错误返回 -1（已打印）。内部用 snd_pcm_recover 恢复 XRUN。
 */
#define AC_EINTR (-2)
snd_pcm_sframes_t ac_pcm_read(struct ac_pcm *h, void *buf, snd_pcm_uframes_t frames);
snd_pcm_sframes_t ac_pcm_write(struct ac_pcm *h, const void *buf, snd_pcm_uframes_t frames);

/* ---------------------------------------------------------------- WAV */

/* 规范 44 字节 PCM WAV 头。所有多字节字段小端。 */
struct ac_wav_hdr {
	char riff[4];           /* "RIFF" */
	uint32_t riff_size;     /* 文件总长 - 8 */
	char wave[4];           /* "WAVE" */
	char fmt_id[4];         /* "fmt " */
	uint32_t fmt_size;      /* 16 */
	uint16_t audio_format;  /* 1 = PCM */
	uint16_t channels;
	uint32_t rate;
	uint32_t byte_rate;     /* rate * channels * bits/8 */
	uint16_t block_align;   /* channels * bits/8 */
	uint16_t bits;          /* 每样本位数（物理位宽） */
	char data_id[4];        /* "data" */
	uint32_t data_size;     /* 数据字节数 */
} __attribute__((packed));

/* 解析出的 WAV 格式信息 + data 块在文件中的位置。 */
struct ac_wav_info {
	unsigned int rate;
	unsigned int channels;
	unsigned int bits;
	uint16_t audio_format;
	snd_pcm_format_t fmt;   /* 由 audio_format/bits 推出的 ALSA 格式 */
	off_t data_offset;      /* data 块第一个字节的文件偏移 */
	uint32_t data_size;     /* data 块声明的字节数 */
};

/*
 * 先写一个 data_size=0 的占位头；录完再调 ac_wav_finalize 回填真实长度。
 * 这样即使中途异常退出，也不会出现“头里是上一轮长度”的假数据。
 *
 * 头在调用方内存里保存、由 ac_wav_finalize 原样回写：
 * 输出文件是用 O_WRONLY 打开的，pread 需要读权限会直接 EBADF，
 * 所以不能靠“先把头读回来再改”这条路。
 */
void ac_wav_make_hdr(struct ac_wav_hdr *h, unsigned int rate,
		     unsigned int channels, unsigned int bits);

int ac_wav_write_hdr(int fd, const struct ac_wav_hdr *h);

/* 回填 riff_size 与 data_size 并落盘。data_bytes 为实际写入的音频字节数。 */
int ac_wav_finalize(int fd, const struct ac_wav_hdr *h, uint32_t data_bytes);

/*
 * 扫描 RIFF 块找到 fmt/data（容忍 LIST 等附加块与奇数长度填充）。
 * 成功返回 0，并把文件位置停在 data 块首字节；失败返回 -1。
 */
int ac_wav_parse(int fd, struct ac_wav_info *out, const char *path);

#endif /* AUDIO_COMMON_H */
