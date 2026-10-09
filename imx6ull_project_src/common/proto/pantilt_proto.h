/*
 * 云台/视频链路的线格式。板端 C 侧的唯一实现，由 pantilt_daemon 和 video_link 共用。
 *
 * PC 端是 Python，无法共享本文件。规范以 docs/protocol.md 为准，本文件与
 * imx6ull_project_src/pc/tracker/ptproto.py 各自断言尺寸/偏移，并由
 * scripts/proto_selftest.sh 做十六进制对拍——任何一侧改动而另一侧没跟上，对拍会红。
 *
 * 全部字段小端。板端 ARM 与 PC x86 都是小端，因此无需字节序转换；
 * 即便如此也不使用原生结构体直传网络，一律经 ptp_pack/unpack 显式编码，
 * 避免编译器插入填充字节导致两端布局不一致。
 */
#ifndef PANTILT_PROTO_H
#define PANTILT_PROTO_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* ---- 端口：控制 9100（PC→板）、遥测 9101（板→PC）、视频 9200（板→PC） ---- */
#define PTP_CTRL_PORT  9100
#define PTP_TELEM_PORT 9101
#define PTP_VIDEO_PORT 9200

#define PTP_VERSION 1u

/*
 * 三份报文各用一个魔数：串端口（比如把视频流灌到控制端口）时能被立刻识别并计数，
 * 而不是被当成合法载荷解析出一个荒谬的 tick 值。
 */
#define PTP_CTRL_MAGIC  0x31544350u /* 'PCT1' */
#define PTP_TELEM_MAGIC 0x31544550u /* 'PET1' */
#define PTP_VIDEO_MAGIC 0x31565650u /* 'PVV1' */

/* ---- 控制包 PC→板端：32 字节 ---- */
#define PTP_CTRL_SIZE 32

/* 模式：板端只做限位/限速，不含决策，所以模式只影响"这一包要做什么"。 */
#define PTP_MODE_MANUAL 1u /* pan_tick/tilt_tick 当绝对目标 */
#define PTP_MODE_TRACK  2u /* 同上；3B 下与 MANUAL 行为一致，仅标签不同 */
#define PTP_MODE_HOME   3u /* 回 home，忽略 tick */
#define PTP_MODE_STOP   4u /* 停止爬升并保持当前位置，忽略 tick */
#define PTP_MODE_HOLD   5u /* 同 STOP；供上位机显式表达"按住别动" */

/*
 * 心跳：客户端按 20 Hz 恒定发包（手动模式下就是重发上一次的目标），
 * 让看门狗有稳定的时间基准。置位表示"这一包的目标与上一包相同"。
 * 板端并不依赖该位判重——它按 (mode, pan, tilt) 的值去重，
 * 这样即使客户端漏置标志也不会污染驱动的 cmd_count。该位只用于遥测统计。
 */
#define PTP_FLAG_HEARTBEAT 0x00000001u

struct ptp_ctrl {
	uint32_t magic;
	uint16_t version;
	uint16_t mode;
	uint32_t seq;
	uint64_t timestamp_us; /* PC 单调时钟；板端只用来判新鲜度，不参与控制 */
	uint32_t flags;
	uint16_t pan_tick;
	uint16_t tilt_tick;
	uint32_t crc32; /* 覆盖本结构 crc32 之前的 28 字节 */
};

/* ---- 遥测包 板端→PC：128 字节 ---- */
#define PTP_TELEM_SIZE 128

/* 驱动 status（与 kernel/pantilt/pantilt_uapi.h 的 PANTILT_ST_* 一致） */
#define PTP_DRV_UNINIT 0u
#define PTP_DRV_READY  1u
#define PTP_DRV_SLEW   2u
#define PTP_DRV_HOLD   3u

/* 芯片状态（与 uapi 的 PANTILT_CHIP_* 一致） */
#define PTP_CHIP_UNKNOWN 0u
#define PTP_CHIP_ASLEEP  1u
#define PTP_CHIP_RUNNING 2u

/* daemon 自身状态机（计划 §4：第一版只需 LOST→HOLD） */
#define PTP_DST_INIT   0u
#define PTP_DST_MANUAL 1u
#define PTP_DST_TRACK  2u
#define PTP_DST_HOME   3u
#define PTP_DST_LOST   4u /* 控制断流，已下发 STOP 保持 */
#define PTP_DST_HOLD   5u

struct ptp_telem {
	uint32_t magic;
	uint16_t version;
	uint16_t drv_status;   /* 驱动 status */
	uint32_t seq;
	uint64_t timestamp_us; /* 板端单调时钟（自启动毫秒转微秒），非墙钟 */
	uint8_t chip_state;
	uint8_t daemon_state;
	uint16_t slew_ms;
	uint16_t wd_ms;        /* 驱动侧看门狗 */
	uint16_t daemon_wd_ms; /* daemon 侧看门狗 */
	uint16_t pan_cur;
	uint16_t tilt_cur;
	uint16_t pan_tgt;
	uint16_t tilt_tgt;
	uint16_t pan_home;
	uint16_t tilt_home;
	uint16_t pan_safe_min;
	uint16_t pan_safe_max;
	uint16_t tilt_safe_min;
	uint16_t tilt_safe_max;
	uint32_t ms_since_ctrl;     /* 距最后一条被接受的控制包 */
	uint32_t last_latency_us;   /* 命令到第一帧写出的延迟（驱动侧） */
	/*
	 * 驱动里的计数是 __u64。这里取低 32 位：这些计数按秒级累加，
	 * 32 位要跑很久才回绕，遥测带宽比绝对精度更值钱。PC 端按 32 位无符号解读。
	 */
	uint32_t cmd_count;
	uint32_t clamp_low_pan;
	uint32_t clamp_low_tilt;
	uint32_t clamp_high_pan;
	uint32_t clamp_high_tilt;
	uint32_t i2c_err;
	uint32_t watchdog_trips; /* 驱动侧触发次数 */
	uint32_t ctrl_rx_ok;     /* daemon 接受的控制包数 */
	uint32_t rej_crc;        /* 校验失败 */
	uint32_t rej_seq;        /* 重复或倒退的 seq */
	uint32_t rej_stale;      /* 过期 timestamp */
	uint32_t rej_mode;       /* 非法模式 */
	uint32_t rej_range;      /* tick 越出驱动安全范围（提前拦下，不浪费一次 ioctl） */
	uint32_t daemon_wd_trips; /* daemon 侧看门狗触发次数 */
	uint32_t reserved[4];
};

/* ---- 视频分包 板端→PC：36 字节头 + 1400 字节载荷 ---- */
#define PTP_VIDEO_HDR_SIZE 36
#define PTP_VIDEO_PAYLOAD  1400
/*
 * 单包 1436 字节 → IP 层 1464 字节 < MTU 1500，不分片。
 * 已实测：到板端的路径能通过 1400 字节的 DF 载荷（见 plan/phase3 §0 与
 * logs/phase3 的链路段）。改大这个值必须先重测路径 MTU。
 */
#define PTP_VIDEO_DGRAM (PTP_VIDEO_HDR_SIZE + PTP_VIDEO_PAYLOAD)

#define PTP_FMT_YUYV  1u
#define PTP_FMT_MJPEG 2u

/* 视频包头 flags */
#define PTP_VIDEO_FLAG_SINGLE 0x0001u /* 整帧只占一个分包（帧长 < PTP_VIDEO_PAYLOAD） */

struct ptp_video_hdr {
	uint32_t magic;
	uint16_t version;
	uint16_t chunk_idx;   /* 0 起 */
	uint16_t chunk_total;
	uint16_t payload_len; /* 末包可小于 PTP_VIDEO_PAYLOAD */
	uint32_t frame_id;    /* 单调递增；PC 靠它判丢帧 */
	uint64_t timestamp_us;
	uint16_t width;
	uint16_t height;
	uint16_t format;
	uint16_t flags;
	uint32_t crc32; /* 本包 payload 的 CRC32（UDP 校验和只有 16 位且可关） */
};

/* ---- CRC32 / 编解码 ---- */

/* 标准 CRC-32/ISO-HDLC（多项式 0xEDB88320），与 zlib.crc32 一致。 */
static inline uint32_t ptp_crc32(const void *buf, size_t len)
{
	const uint8_t *p = (const uint8_t *)buf;
	uint32_t crc = 0xFFFFFFFFu;
	size_t i;
	int b;

	for (i = 0; i < len; i++) {
		crc ^= p[i];
		for (b = 0; b < 8; b++)
			crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
	}
	return crc ^ 0xFFFFFFFFu;
}

/*
 * 显式小端编解码。**结构体不得直接作为网络载荷**：上面那些 struct 是内存里的
 * 便利容器，编译器会为对齐插入填充字节——timestamp_us 是 u64，在 ptp_ctrl 里
 * 落在偏移 12 上，编译器要在它前面补 4 字节，于是 sizeof(struct ptp_ctrl) 实测
 * 是 40，而不是线上格式的 32；ptp_video_hdr 同理是 40 比 36。
 *
 * 所以"线上格式"由下面这些 ptp_put_* 的写出顺序唯一定义，sizeof(struct) 只是
 * 一个与协议无关的数字。反过来若依赖 sizeof，就会在两端编译器/目标架构不同时
 * 悄悄错位，且 memcmp 比较结构体还会被未定义的填充区内容干扰。
 */
static inline void ptp_put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static inline void ptp_put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
	p[2] = (uint8_t)((v >> 16) & 0xFFu);
	p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static inline void ptp_put_u64(uint8_t *p, uint64_t v)
{
	ptp_put_u32(p, (uint32_t)(v & 0xFFFFFFFFu));
	ptp_put_u32(p + 4, (uint32_t)(v >> 32));
}

static inline uint16_t ptp_get_u16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t ptp_get_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	       ((uint32_t)p[3] << 24);
}

static inline uint64_t ptp_get_u64(const uint8_t *p)
{
	return (uint64_t)ptp_get_u32(p) | ((uint64_t)ptp_get_u32(p + 4) << 32);
}

/* 控制包编码：crc 最后写，覆盖它之前的所有字节。 */
static inline void ptp_ctrl_encode(const struct ptp_ctrl *c, uint8_t out[PTP_CTRL_SIZE])
{
	ptp_put_u32(out + 0, c->magic);
	ptp_put_u16(out + 4, c->version);
	ptp_put_u16(out + 6, c->mode);
	ptp_put_u32(out + 8, c->seq);
	ptp_put_u64(out + 12, c->timestamp_us);
	ptp_put_u32(out + 20, c->flags);
	ptp_put_u16(out + 24, c->pan_tick);
	ptp_put_u16(out + 26, c->tilt_tick);
	ptp_put_u32(out + 28, ptp_crc32(out, 28));
}

/*
 * 控制包解码。crc 不符返回 -1；其余字段一律照收，由调用方按序校验
 * （魔数 → 版本 → 模式 → 新鲜度 → 值域），这样每一类拒绝都能各自计数。
 */
static inline int ptp_ctrl_decode(const uint8_t *in, size_t len, struct ptp_ctrl *c)
{
	if (len != PTP_CTRL_SIZE)
		return -1;
	if (ptp_get_u32(in + 28) != ptp_crc32(in, 28))
		return -1;

	c->magic = ptp_get_u32(in + 0);
	c->version = ptp_get_u16(in + 4);
	c->mode = ptp_get_u16(in + 6);
	c->seq = ptp_get_u32(in + 8);
	c->timestamp_us = ptp_get_u64(in + 12);
	c->flags = ptp_get_u32(in + 20);
	c->pan_tick = ptp_get_u16(in + 24);
	c->tilt_tick = ptp_get_u16(in + 26);
	c->crc32 = ptp_get_u32(in + 28);
	return 0;
}

/*
 * 遥测包编码。字段多，但一个都不能省——PC 端画波形、判故障全靠它。
 * 这里刻意不接受"部分填充"：结构体先 memset 再逐项赋值由调用方负责，
 * 本函数只做布局。
 */
static inline void ptp_telem_encode(const struct ptp_telem *t,
				    uint8_t out[PTP_TELEM_SIZE])
{
	ptp_put_u32(out + 0, t->magic);
	ptp_put_u16(out + 4, t->version);
	ptp_put_u16(out + 6, t->drv_status);
	ptp_put_u32(out + 8, t->seq);
	ptp_put_u64(out + 12, t->timestamp_us);
	out[20] = t->chip_state;
	out[21] = t->daemon_state;
	ptp_put_u16(out + 22, t->slew_ms);
	ptp_put_u16(out + 24, t->wd_ms);
	ptp_put_u16(out + 26, t->daemon_wd_ms);
	ptp_put_u16(out + 28, t->pan_cur);
	ptp_put_u16(out + 30, t->tilt_cur);
	ptp_put_u16(out + 32, t->pan_tgt);
	ptp_put_u16(out + 34, t->tilt_tgt);
	ptp_put_u16(out + 36, t->pan_home);
	ptp_put_u16(out + 38, t->tilt_home);
	ptp_put_u16(out + 40, t->pan_safe_min);
	ptp_put_u16(out + 42, t->pan_safe_max);
	ptp_put_u16(out + 44, t->tilt_safe_min);
	ptp_put_u16(out + 46, t->tilt_safe_max);
	ptp_put_u32(out + 48, t->ms_since_ctrl);
	ptp_put_u32(out + 52, t->last_latency_us);
	ptp_put_u32(out + 56, t->cmd_count);
	ptp_put_u32(out + 60, t->clamp_low_pan);
	ptp_put_u32(out + 64, t->clamp_low_tilt);
	ptp_put_u32(out + 68, t->clamp_high_pan);
	ptp_put_u32(out + 72, t->clamp_high_tilt);
	ptp_put_u32(out + 76, t->i2c_err);
	ptp_put_u32(out + 80, t->watchdog_trips);
	ptp_put_u32(out + 84, t->ctrl_rx_ok);
	ptp_put_u32(out + 88, t->rej_crc);
	ptp_put_u32(out + 92, t->rej_seq);
	ptp_put_u32(out + 96, t->rej_stale);
	ptp_put_u32(out + 100, t->rej_mode);
	ptp_put_u32(out + 104, t->rej_range);
	ptp_put_u32(out + 108, t->daemon_wd_trips);
	ptp_put_u32(out + 112, t->reserved[0]);
	ptp_put_u32(out + 116, t->reserved[1]);
	ptp_put_u32(out + 120, t->reserved[2]);
	ptp_put_u32(out + 124, t->reserved[3]);
}

/* 一帧的公共部分。分包时每片都要重复写进头部。 */
struct ptp_video_meta {
	uint32_t frame_id;
	uint64_t timestamp_us;
	uint16_t width;
	uint16_t height;
	uint16_t format;
	uint16_t flags;
};

/*
 * 编码一个视频分包的头。payload 只在算 crc 时读，调用方负责把载荷
 * memcpy 到 out + PTP_VIDEO_HDR_SIZE（本函数不碰载荷区）。
 * CRC 只覆盖载荷、不覆盖头：头里的 frame_id/chunk_idx 本来就是用来发现
 * 乱序与缺片的，把它们也算进 CRC 会让"哪一片坏了"变成"整个头都不可信"。
 */
static inline void ptp_video_hdr_encode(const struct ptp_video_meta *m,
					uint16_t chunk_idx, uint16_t chunk_total,
					const uint8_t *payload, uint16_t payload_len,
					uint8_t out[PTP_VIDEO_HDR_SIZE])
{
	ptp_put_u32(out + 0, PTP_VIDEO_MAGIC);
	ptp_put_u16(out + 4, PTP_VERSION);
	ptp_put_u16(out + 6, chunk_idx);
	ptp_put_u16(out + 8, chunk_total);
	ptp_put_u16(out + 10, payload_len);
	ptp_put_u32(out + 12, m->frame_id);
	ptp_put_u64(out + 16, m->timestamp_us);
	ptp_put_u16(out + 24, m->width);
	ptp_put_u16(out + 26, m->height);
	ptp_put_u16(out + 28, m->format);
	ptp_put_u16(out + 30, m->flags);
	ptp_put_u32(out + 32, ptp_crc32(payload, payload_len));
}

/* 编译期锁死布局：改字段而忘了同步 docs/protocol.md 与 pc/tracker/ptproto.py 时先炸在这。 */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(PTP_CTRL_SIZE == 32, "ctrl layout drift");
_Static_assert(PTP_TELEM_SIZE == 128, "telem layout drift");
_Static_assert(PTP_VIDEO_HDR_SIZE == 36, "video hdr layout drift");
#endif

#endif /* PANTILT_PROTO_H */
