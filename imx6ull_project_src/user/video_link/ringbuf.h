/*
 * 定长槽环形缓冲。生产者是 V4L2 采集线程，消费者是 UDP 发送线程。
 *
 * 核心约束（rules §4）：**采集永不因发送阻塞**。
 * 因此：
 *   - 生产端只在写槽时持锁，写满即覆盖最旧帧并计数，绝不等待消费者；
 *   - 消费端在锁内只做一次 memcpy 就放锁，全部分包发送都在锁外进行。
 * 这样即使对端完全不收、网卡队列打满，采集循环的阻塞上限也只是"一次帧拷贝"，
 * 而不是"等一次网络发送"。
 *
 * 之所以必须拷贝而不能直接把槽指针交给消费者：对端收得慢时，生产者要写的正是
 * 消费者正在读的那一槽（读指针==写指针且满）。拷贝是这里最省心的正确解，
 * 代价（每帧一次约 150 KB memcpy）单独计时并上报，不靠推断。
 */
#ifndef RINGBUF_H
#define RINGBUF_H

#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RB_DEFAULT_SLOTS 8
#define RB_MIN_SLOTS 4
#define RB_MAX_SLOTS 8

struct rb_meta {
	uint32_t frame_id;
	uint64_t timestamp_us;
	size_t len;
	uint16_t width;
	uint16_t height;
	uint16_t format;
};

struct rb_stats {
	uint64_t frames_in;        /* 生产端写入次数 */
	uint64_t frames_out;       /* 消费端取出次数 */
	uint64_t drop_overwrite;   /* 因满而覆盖最旧帧的次数（= 消费者跟不上的次数） */
	uint64_t copy_ns_total;    /* 消费端锁内拷贝累计耗时 */
	uint64_t wait_ns_total;    /* 消费端在锁外等待新帧的累计耗时 */
	int high_watermark;        /* 队列长度峰值 */
	int slots;
	int count;                 /* 当前在队帧数 */
};

struct ringbuf {
	pthread_mutex_t lock;
	pthread_cond_t not_empty;
	uint8_t *pool;          /* slots * slot_cap 的连续内存 */
	size_t slot_cap;
	struct rb_meta *meta;
	int slots;
	int read_idx;
	int write_idx;
	int count;
	bool stopping;
	/* 统计字段在 lock 下更新；读侧 rb_get_stats 也持锁，避免撕裂。 */
	uint64_t frames_in;
	uint64_t frames_out;
	uint64_t drop_overwrite;
	uint64_t copy_ns_total;
	uint64_t wait_ns_total;
	int high_watermark;
};

/* slots 会被夹到 [RB_MIN_SLOTS, RB_MAX_SLOTS]；返回实际槽数，<0 为错误。 */
int rb_init(struct ringbuf *rb, int slots, size_t slot_cap);
void rb_destroy(struct ringbuf *rb);

/*
 * 生产者：写入一帧。始终不失败（满则覆盖最旧帧），因此采集端不需要错误分支。
 * 返回本次是否发生了覆盖丢帧。
 */
bool rb_push(struct ringbuf *rb, const void *src, size_t len,
	     const struct rb_meta *meta);

/*
 * 消费者：取最旧一帧到 dst。dst 必须至少 slot_cap 字节。
 * 阻塞直到有帧可取或 rb_stop 被调用。返回 0 取到，-1 表示已停止且队列空。
 */
int rb_pop(struct ringbuf *rb, void *dst, struct rb_meta *meta_out);

/* 唤醒阻塞中的消费者并使其返回 -1。可重复调用。 */
void rb_stop(struct ringbuf *rb);

void rb_get_stats(struct ringbuf *rb, struct rb_stats *out);

#endif /* RINGBUF_H */
