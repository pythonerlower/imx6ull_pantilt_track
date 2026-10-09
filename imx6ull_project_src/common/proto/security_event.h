/*
 * securityd → pantilt_daemon 的板内事件通道（Phase 7）。
 *
 * 为什么不塞进 pantilt_proto.h：securityd 刻意不 include 云台 ABI 头，
 * 以此保证"securityd 完全不碰云台"是结构性事实而不是一句承诺（见 securityd.c 头注释 6）。
 * 本文件因此自成一体，只描述"有个事件发生了"，不含任何云台语义。
 * 云台侧的解码方是 pantilt_daemon.c，落地到遥测 reserved[1] 的编码宏也放在这里，
 * 让"发事件的人"和"转事件的人"用同一份定义。
 *
 * 载体：AF_UNIX / SOCK_DGRAM，路径由双方各自的 --event-sock 给出。
 * 用数据报而不是流，是因为一个事件就是一个完整消息，不需要拆包；
 * 用 UNIX 而不是 UDP，是因为这条链路不过网、没有对端地址可填。
 *
 * 全部字段小端，和 pantilt_proto.h 一样显式打包，不直传结构体。
 */
#ifndef SECURITY_EVENT_H
#define SECURITY_EVENT_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define PTP_SEKEV_MAGIC 0x31454553u /* 'SEE1' */
#define PTP_SEKEV_VERSION 1u
#define PTP_SEKEV_SIZE 16

/* 事件种类。Phase 7 只有振动一种；留字段是为了以后加别的来源不用改线格式。 */
#define PTP_SEKEV_KIND_VIB 1u

struct ptp_sekev {
	uint32_t magic;
	uint16_t version;
	uint16_t kind;
	uint32_t seq;   /* 单调递增，从 1 开始；接收方按它判"新事件" */
	uint32_t t_ms;  /* 发送方的 monotonic 毫秒，仅用于日志对账 */
};

static inline void ptp_sekev_put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xff);
	p[1] = (uint8_t)((v >> 8) & 0xff);
}

static inline void ptp_sekev_put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xff);
	p[1] = (uint8_t)((v >> 8) & 0xff);
	p[2] = (uint8_t)((v >> 16) & 0xff);
	p[3] = (uint8_t)((v >> 24) & 0xff);
}

static inline uint16_t ptp_sekev_get_u16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t ptp_sekev_get_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline size_t ptp_sekev_pack(const struct ptp_sekev *e,
				    uint8_t out[PTP_SEKEV_SIZE])
{
	ptp_sekev_put_u32(out + 0, e->magic);
	ptp_sekev_put_u16(out + 4, e->version);
	ptp_sekev_put_u16(out + 6, e->kind);
	ptp_sekev_put_u32(out + 8, e->seq);
	ptp_sekev_put_u32(out + 12, e->t_ms);
	return PTP_SEKEV_SIZE;
}

/* 返回 0 成功；-1 表示线格式不对（magic/version/长度任一不符）。 */
static inline int ptp_sekev_unpack(const uint8_t *in, size_t len,
				   struct ptp_sekev *e)
{
	if (len != PTP_SEKEV_SIZE)
		return -1;
	e->magic = ptp_sekev_get_u32(in + 0);
	e->version = ptp_sekev_get_u16(in + 4);
	e->kind = ptp_sekev_get_u16(in + 6);
	e->seq = ptp_sekev_get_u32(in + 8);
	e->t_ms = ptp_sekev_get_u32(in + 12);
	if (e->magic != PTP_SEKEV_MAGIC || e->version != PTP_SEKEV_VERSION)
		return -1;
	return 0;
}

/*
 * 遥测 reserved[1] 的编码（plan/phase7_wom_patrol_debug.md §2.2）：
 *   高 16 位 = 收到的事件计数（截断），低 16 位 = 距最近一次事件的毫秒数（饱和到 0xFFFF）。
 * PC 侧只需看高位有没有变，就知道有没有新事件；低位让 PC 自己判"还算新鲜吗"。
 * daemon 在 2 s 内始终报 age<2000，这就是 plan 里说的"锁存保持 ~2 s"。
 */
#define PTP_TELEM_EV_PACK(count, age_ms) \
	((uint32_t)(((uint32_t)(count) & 0xffffu) << 16) | \
	 ((uint32_t)(age_ms) > 0xffffu ? 0xffffu : (uint32_t)(age_ms)))
#define PTP_TELEM_EV_COUNT(v) ((uint32_t)(v) >> 16)
#define PTP_TELEM_EV_AGE(v)   ((uint32_t)(v) & 0xffffu)
#define PTP_TELEM_EV_FRESH_MS 2000u

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(PTP_SEKEV_SIZE == 16, "sekev layout drift");
#endif

#endif /* SECURITY_EVENT_H */
