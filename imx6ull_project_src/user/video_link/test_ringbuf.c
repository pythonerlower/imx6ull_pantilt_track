/*
 * 环形缓冲的宿主端单测。不碰摄像头、不碰网络，在开发机上直接跑：
 *     make -C imx6ull_project_src/user/video_link test
 *
 * 存在的理由：丢帧计数、"覆盖的是最旧帧"这类语义，看代码"觉得对"很容易，
 * 但这正是 rules §4 要求的可观测性，必须能被证伪。板端跑起来之后没有方便的
 * 单测入口，所以放在这里。
 */
#include "ringbuf.h"

#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define CAP 512
#define SLOTS 4 /* 用最小值，最容易触发边界 */

static void fill(uint8_t *buf, uint32_t tag)
{
	for (size_t i = 0; i < CAP; i++)
		buf[i] = (uint8_t)(tag + i);
}

static int check(const uint8_t *buf, uint32_t tag)
{
	for (size_t i = 0; i < CAP; i++)
		if (buf[i] != (uint8_t)(tag + i))
			return -1;
	return 0;
}

static struct rb_meta mk(uint32_t id)
{
	struct rb_meta m = { 0 };

	m.frame_id = id;
	m.len = CAP;
	m.width = 320;
	m.height = 240;
	m.format = 1;
	return m;
}

/* C 里取不到函数返回值的地址，包一层再用。 */
static bool push_n(struct ringbuf *rb, const void *src, size_t len, uint32_t id)
{
	struct rb_meta m = mk(id);

	return rb_push(rb, src, len, &m);
}

static void test_fifo_order(void)
{
	struct ringbuf rb;
	struct rb_meta m;
	uint8_t src[CAP], dst[CAP];

	assert(rb_init(&rb, SLOTS, CAP) == SLOTS);
	for (uint32_t i = 1; i <= 3; i++) {
		fill(src, i * 100);
		assert(!push_n(&rb, src, CAP, i));
	}
	for (uint32_t i = 1; i <= 3; i++) {
		assert(rb_pop(&rb, dst, &m) == 0);
		assert(m.frame_id == i);
		assert(check(dst, i * 100) == 0);
	}
	rb_destroy(&rb);
	printf("  [ok] 先进先出顺序保持\n");
}

static void test_overwrite_drops_oldest(void)
{
	struct ringbuf rb;
	struct rb_stats st;
	struct rb_meta m;
	uint8_t src[CAP], dst[CAP];

	assert(rb_init(&rb, SLOTS, CAP) == SLOTS);
	/* 写 SLOTS+3 帧：前 3 帧应被覆盖，队列里应只剩 4..7。 */
	for (uint32_t i = 1; i <= SLOTS + 3; i++) {
		fill(src, i * 100);
		bool dropped = push_n(&rb, src, CAP, i);

		assert(dropped == (i > (uint32_t)SLOTS));
	}

	rb_get_stats(&rb, &st);
	assert(st.frames_in == SLOTS + 3);
	assert(st.drop_overwrite == 3);
	assert((uint32_t)st.count == (uint32_t)SLOTS);
	assert(st.high_watermark == SLOTS);

	for (uint32_t i = 4; i <= (uint32_t)SLOTS + 3; i++) {
		assert(rb_pop(&rb, dst, &m) == 0);
		assert(m.frame_id == i);
		assert(check(dst, i * 100) == 0);
	}
	rb_get_stats(&rb, &st);
	assert(st.count == 0);
	rb_destroy(&rb);
	printf("  [ok] 满时覆盖最旧帧且计数准确（丢 3 帧，留下的是最新 4 帧）\n");
}

static void test_oversize_payload_clamped(void)
{
	struct ringbuf rb;
	struct rb_meta m;
	uint8_t big[CAP * 2], dst[CAP];

	assert(rb_init(&rb, SLOTS, CAP) == SLOTS);
	memset(big, 0xAB, sizeof(big));
	assert(!push_n(&rb, big, sizeof(big), 1));
	assert(rb_pop(&rb, dst, &m) == 0);
	assert(m.len == CAP); /* 超长载荷被截到槽容量，而不是越界写 */
	rb_destroy(&rb);
	printf("  [ok] 超长载荷被截断而非越界写\n");
}

struct blocked_arg {
	struct ringbuf *rb;
	int rc;
};

static void *blocked_consumer(void *arg)
{
	struct blocked_arg *a = arg;
	uint8_t dst[CAP];
	struct rb_meta m;

	a->rc = rb_pop(a->rb, dst, &m);
	return NULL;
}

static void test_stop_unblocks_consumer(void)
{
	struct ringbuf rb;
	struct blocked_arg a;
	pthread_t th;
	struct timespec ts = { .tv_sec = 0, .tv_nsec = 200 * 1000 * 1000 };

	assert(rb_init(&rb, SLOTS, CAP) == SLOTS);
	a.rb = &rb;
	a.rc = 99;
	assert(pthread_create(&th, NULL, blocked_consumer, &a) == 0);

	/* 让消费者真的进到 cond_wait 里，再停。 */
	nanosleep(&ts, NULL);
	rb_stop(&rb);
	pthread_join(th, NULL);

	assert(a.rc == -1); /* 停止且队列空 → -1，而不是永远挂着 */
	rb_destroy(&rb);
	printf("  [ok] rb_stop 能唤醒阻塞中的消费者并返回 -1\n");
}

/*
 * 模拟 rules §4 的场景：消费者卡死不动，生产者必须能继续跑完整轮次。
 * 这是本文件最重要的一个断言——采集被发送堵住是不可接受的。
 */
static void test_producer_never_blocks_with_dead_consumer(void)
{
	struct ringbuf rb;
	struct rb_stats st;
	uint8_t src[CAP];
	const int rounds = 50;
	struct timespec t0, t1;
	uint64_t elapsed_ms;

	assert(rb_init(&rb, SLOTS, CAP) == SLOTS);
	fill(src, 7);

	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (int i = 0; i < rounds; i++)
		push_n(&rb, src, CAP, (uint32_t)i);
	clock_gettime(CLOCK_MONOTONIC, &t1);

	elapsed_ms = (uint64_t)(t1.tv_sec - t0.tv_sec) * 1000ull +
		     (uint64_t)(t1.tv_nsec - t0.tv_nsec) / 1000000ull;

	rb_get_stats(&rb, &st);
	assert(st.frames_in == (uint64_t)rounds);          /* 一帧都没丢在生产者手里 */
	assert(st.drop_overwrite == (uint64_t)(rounds - SLOTS));
	assert(st.count == SLOTS);
	/* 50 次 512 字节拷贝，正常应远小于 100 ms；宽松阈值只用来抓"退化成阻塞"。 */
	assert(elapsed_ms < 100);
	rb_destroy(&rb);
	printf("  [ok] 消费者完全不动时，生产者仍跑完 %d 轮（%llu ms），未阻塞\n",
	       rounds, (unsigned long long)elapsed_ms);
}

int main(void)
{
	printf("ringbuf 单测（槽数=%d，槽容量=%d）\n", SLOTS, CAP);
	test_fifo_order();
	test_overwrite_drops_oldest();
	test_oversize_payload_clamped();
	test_stop_unblocks_consumer();
	test_producer_never_blocks_with_dead_consumer();
	printf("全部通过\n");
	return 0;
}
