/*
 * audio_common.c —— Phase 5 录音/播放程序的公共实现
 *
 * 底本参考：reference/01、例程源码/11、Linux C应用编程例程源码/28_alsa-lib/
 *           （ALIENTEK 邓涛，V1.0 2021/7/20）—— 只借参数设置顺序，
 *           其示例缺少头文件、无 XRUN 恢复、播放时按 period_size 而非实际帧数写，
 *           这些在本文件里都做了修正。
 */
#define _GNU_SOURCE
#include "audio_common.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>

/* ---------------------------------------------------------------- PCM */

int ac_format_parse(const char *name, snd_pcm_format_t *out)
{
	snd_pcm_format_t f;

	if (name == NULL)
		return -1;
	f = snd_pcm_format_value(name);
	if (f == SND_PCM_FORMAT_UNKNOWN) {
		fprintf(stderr, "audio: 未知采样格式 '%s'（如 S16_LE/S32_LE/U8/S24_3LE）\n",
			name);
		return -1;
	}
	*out = f;
	return 0;
}

int ac_pcm_open(struct ac_pcm *h, const struct ac_pcm_cfg *cfg,
		snd_pcm_stream_t stream)
{
	snd_pcm_hw_params_t *hw;
	unsigned int rate;
	int dir = 0;
	int ret;
	int phys_bits;
	const char *what = (stream == SND_PCM_STREAM_CAPTURE) ? "录音" : "播放";

	memset(h, 0, sizeof(*h));
	h->period_size = cfg->period_size;
	h->periods = cfg->periods;
	/* 只存副本：调用方的 cfg 可能随栈回收，日志里还要用这个名字。 */
	snprintf(h->device, sizeof(h->device), "%s", cfg->device);

	if (ac_format_parse(cfg->format, &h->fmt) < 0)
		return -1;

	ret = snd_pcm_open(&h->pcm, cfg->device, stream, 0);
	if (ret < 0) {
		fprintf(stderr, "audio: snd_pcm_open(%s, %s) 失败: %s\n",
			cfg->device, what, snd_strerror(ret));
		return -1;
	}

	snd_pcm_hw_params_alloca(&hw);
	ret = snd_pcm_hw_params_any(h->pcm, hw);
	if (ret < 0) {
		fprintf(stderr, "audio: hw_params_any 失败: %s\n", snd_strerror(ret));
		goto err;
	}
	ret = snd_pcm_hw_params_set_access(h->pcm, hw, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (ret < 0) {
		fprintf(stderr, "audio: 设备不支持交错访问: %s\n", snd_strerror(ret));
		goto err;
	}
	ret = snd_pcm_hw_params_set_format(h->pcm, hw, h->fmt);
	if (ret < 0) {
		fprintf(stderr, "audio: 设备不支持格式 %s: %s\n",
			snd_pcm_format_name(h->fmt), snd_strerror(ret));
		goto err;
	}
	rate = cfg->rate;
	ret = snd_pcm_hw_params_set_rate_near(h->pcm, hw, &rate, &dir);
	if (ret < 0) {
		fprintf(stderr, "audio: 设置采样率 %u 失败: %s\n",
			cfg->rate, snd_strerror(ret));
		goto err;
	}
	if (rate != cfg->rate)
		fprintf(stderr, "audio: WARN 采样率 %u 不被支持，设备取近值 %u\n",
			cfg->rate, rate);
	h->rate = rate;

	ret = snd_pcm_hw_params_set_channels(h->pcm, hw, cfg->channels);
	if (ret < 0) {
		fprintf(stderr, "audio: 设置声道数 %u 失败: %s\n",
			cfg->channels, snd_strerror(ret));
		goto err;
	}
	h->channels = cfg->channels;

	/* period/buffer 是"想要"，设备可能取邻值；下面读回真实值。 */
	ret = snd_pcm_hw_params_set_period_size_near(h->pcm, hw, &h->period_size, &dir);
	if (ret < 0) {
		fprintf(stderr, "audio: 设置 period_size 失败: %s\n", snd_strerror(ret));
		goto err;
	}
	ret = snd_pcm_hw_params_set_periods_near(h->pcm, hw, &h->periods, &dir);
	if (ret < 0) {
		fprintf(stderr, "audio: 设置 periods 失败: %s\n", snd_strerror(ret));
		goto err;
	}

	ret = snd_pcm_hw_params(h->pcm, hw);
	if (ret < 0) {
		fprintf(stderr, "audio: snd_pcm_hw_params 失败: %s\n", snd_strerror(ret));
		goto err;
	}

	/* 读回实际生效值：观察输出以设备为准，不以"请求值"为准。 */
	{
		snd_pcm_uframes_t buf = 0;
		snd_pcm_hw_params_get_buffer_size(hw, &buf);
		h->buffer_frames = buf;
	}
	phys_bits = snd_pcm_format_physical_width(h->fmt);
	if (phys_bits <= 0) {
		fprintf(stderr, "audio: 无法确定格式 %s 的位宽\n",
			snd_pcm_format_name(h->fmt));
		goto err;
	}
	h->frame_bytes = (unsigned int)phys_bits / 8 * h->channels;

	ret = snd_pcm_prepare(h->pcm);
	if (ret < 0) {
		fprintf(stderr, "audio: snd_pcm_prepare 失败: %s\n", snd_strerror(ret));
		goto err;
	}

	printf("audio: %s %s | %s %u Hz %u ch | period %lu x %u | buffer %lu 帧 (%.1f ms)\n",
	       what, cfg->device, snd_pcm_format_name(h->fmt), h->rate, h->channels,
	       (unsigned long)h->period_size, h->periods,
	       (unsigned long)h->buffer_frames,
	       h->rate ? 1000.0 * (double)h->buffer_frames / (double)h->rate : 0.0);
	return 0;

err:
	snd_pcm_close(h->pcm);
	h->pcm = NULL;
	return -1;
}

void ac_pcm_close(struct ac_pcm *h)
{
	if (h->pcm != NULL) {
		snd_pcm_drop(h->pcm);
		snd_pcm_close(h->pcm);
		h->pcm = NULL;
	}
}

void ac_pcm_stats_print(const struct ac_pcm *h, FILE *fp)
{
	fprintf(fp, "audio: 统计 frames=%llu xrun=%llu suspend=%llu short_io=%llu\n",
		h->stats.frames, h->stats.xrun, h->stats.suspend, h->stats.short_io);
}

void ac_pcm_result_print(const struct ac_pcm *h, const char *op,
			 long long bytes, double wall_s, FILE *fp)
{
	fprintf(fp, "audio: RESULT op=%s dev=%s frames=%llu rate=%u xrun=%llu "
		    "suspend=%llu short_io=%llu",
		op, h->device, h->stats.frames, h->rate,
		h->stats.xrun, h->stats.suspend, h->stats.short_io);
	if (bytes >= 0)
		fprintf(fp, " bytes=%lld", bytes);
	fprintf(fp, " wall_s=%.2f\n", wall_s);
}

/* 统一的错误恢复：EPIPE=XRUN、ESTRPIPE=挂起、EINTR=被信号打断。 */
static int ac_pcm_recover(struct ac_pcm *h, int err)
{
	int rc;

	if (err == -EINTR)
		return AC_EINTR;

	rc = snd_pcm_recover(h->pcm, err, 1 /*silent*/);
	if (rc < 0) {
		fprintf(stderr, "audio: snd_pcm_recover(%s) 失败: %s\n",
			snd_strerror(err), snd_strerror(rc));
		return -1;
	}
	if (err == -EPIPE) {
		h->stats.xrun++;
		fprintf(stderr, "audio: XRUN(%s)，已 prepare 恢复（累计 %llu）\n",
			snd_strerror(err), h->stats.xrun);
	} else if (err == -ESTRPIPE) {
		h->stats.suspend++;
		fprintf(stderr, "audio: 设备曾挂起(ESTRPIPE)，已恢复（累计 %llu）\n",
			h->stats.suspend);
	}
	return 0;
}

static snd_pcm_sframes_t ac_pcm_io(struct ac_pcm *h, void *buf,
				   snd_pcm_uframes_t frames, int is_write)
{
	snd_pcm_sframes_t n;
	int attempts = 0;

	for (;;) {
		n = is_write ? snd_pcm_writei(h->pcm, buf, frames)
			     : snd_pcm_readi(h->pcm, buf, frames);
		if (n >= 0) {
			if ((snd_pcm_uframes_t)n < frames)
				h->stats.short_io++;
			h->stats.frames += (unsigned long long)n;
			return n;
		}
		/* 恢复成功就重试；连续失败超过 3 次说明是别的问题，不再叠加重试。 */
		{
			int rc = ac_pcm_recover(h, (int)n);
			if (rc != 0)
				return rc;      /* -1 不可恢复，AC_EINTR 被打断 */
		}
		if (++attempts > 3) {
			fprintf(stderr, "audio: 连续 %d 次 I/O 失败后放弃\n", attempts);
			return -1;
		}
	}
}

snd_pcm_sframes_t ac_pcm_read(struct ac_pcm *h, void *buf, snd_pcm_uframes_t frames)
{
	return ac_pcm_io(h, buf, frames, 0);
}

snd_pcm_sframes_t ac_pcm_write(struct ac_pcm *h, const void *buf, snd_pcm_uframes_t frames)
{
	return ac_pcm_io(h, (void *)buf, frames, 1);
}

/* ---------------------------------------------------------------- WAV */

void ac_wav_make_hdr(struct ac_wav_hdr *h, unsigned int rate,
		     unsigned int channels, unsigned int bits)
{
	memset(h, 0, sizeof(*h));
	memcpy(h->riff, "RIFF", 4);
	memcpy(h->wave, "WAVE", 4);
	memcpy(h->fmt_id, "fmt ", 4);
	h->fmt_size = 16;
	h->audio_format = 1;                    /* PCM */
	h->channels = (uint16_t)channels;
	h->rate = rate;
	h->byte_rate = rate * channels * (bits / 8);
	h->block_align = (uint16_t)(channels * (bits / 8));
	h->bits = (uint16_t)bits;
	memcpy(h->data_id, "data", 4);
	h->data_size = 0;
	h->riff_size = 0;
}

int ac_wav_write_hdr(int fd, const struct ac_wav_hdr *h)
{
	if (pwrite(fd, h, sizeof(*h), 0) != (ssize_t)sizeof(*h)) {
		fprintf(stderr, "audio: 写 WAV 头失败: %s\n", strerror(errno));
		return -1;
	}
	/* pwrite 不动文件位置，这里显式移到数据区起点。 */
	if (lseek(fd, sizeof(*h), SEEK_SET) != (off_t)sizeof(*h)) {
		fprintf(stderr, "audio: 定位到数据区失败: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

int ac_wav_finalize(int fd, const struct ac_wav_hdr *base, uint32_t data_bytes)
{
	struct ac_wav_hdr h = *base;

	h.riff_size = sizeof(h) - 8 + data_bytes;
	h.data_size = data_bytes;
	if (pwrite(fd, &h, sizeof(h), 0) != (ssize_t)sizeof(h)) {
		fprintf(stderr, "audio: 回填 WAV 头失败: %s\n", strerror(errno));
		return -1;
	}
	if (fsync(fd) != 0)
		fprintf(stderr, "audio: WARN fsync 失败: %s\n", strerror(errno));
	return 0;
}

/* 大端 16/32 位辅助（WAV 一律小端，这里只做小端读取）。 */
static uint32_t rd_u32le(const unsigned char *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t rd_u16le(const unsigned char *p)
{
	return (uint16_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8));
}

int ac_wav_parse(int fd, struct ac_wav_info *out, const char *path)
{
	unsigned char hdr[12];
	unsigned char chunk[8];
	int have_fmt = 0;

	memset(out, 0, sizeof(*out));

	if (pread(fd, hdr, sizeof(hdr), 0) != (ssize_t)sizeof(hdr) ||
	    memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) {
		fprintf(stderr, "audio: %s 不是 RIFF/WAVE 文件\n", path);
		return -1;
	}

	off_t pos = 12;
	for (;;) {
		uint32_t size;
		ssize_t n = pread(fd, chunk, sizeof(chunk), pos);

		if (n != (ssize_t)sizeof(chunk))
			break;                  /* 到文件尾还没找到 data */
		size = rd_u32le(chunk + 4);
		pos += 8;

		if (memcmp(chunk, "fmt ", 4) == 0) {
			unsigned char f[16];
			if (size < 16 || pread(fd, f, sizeof(f), pos) != (ssize_t)sizeof(f)) {
				fprintf(stderr, "audio: %s 的 fmt 块不合法\n", path);
				return -1;
			}
			out->audio_format = rd_u16le(f + 0);
			out->channels = rd_u16le(f + 2);
			out->rate = rd_u32le(f + 4);
			out->bits = rd_u16le(f + 14);
			have_fmt = 1;
		} else if (memcmp(chunk, "data", 4) == 0) {
			if (!have_fmt) {
				fprintf(stderr, "audio: %s 的 data 块出现在 fmt 之前\n", path);
				return -1;
			}
			if (out->audio_format != 1) {
				fprintf(stderr, "audio: %s 不是 PCM WAV（format=%u）\n",
					path, out->audio_format);
				return -1;
			}
			switch (out->bits) {
			case 8:  out->fmt = SND_PCM_FORMAT_U8;     break;
			case 16: out->fmt = SND_PCM_FORMAT_S16_LE; break;
			case 24: out->fmt = SND_PCM_FORMAT_S24_3LE; break;
			case 32: out->fmt = SND_PCM_FORMAT_S32_LE; break;
			default:
				fprintf(stderr, "audio: %s 的位宽 %u 不支持\n",
					path, out->bits);
				return -1;
			}
			out->data_offset = pos;
			out->data_size = size;
			if (lseek(fd, pos, SEEK_SET) != pos) {
				fprintf(stderr, "audio: 定位到 data 块失败: %s\n",
					strerror(errno));
				return -1;
			}
			return 0;
		}

		pos += size + (size & 1);       /* 奇数长度块带 1 字节填充 */
	}

	fprintf(stderr, "audio: %s 中找不到 data 块\n", path);
	return -1;
}
