/* -std=c11 是严格 ISO 模式，不声明它就没有 clock_gettime / CLOCK_MONOTONIC。
 * 在开发机上恰好能编过（本机 glibc 头文件的默认可见性不同），交叉编译时必炸，
 * 所以无条件写上，不依赖运气。 */
#define _POSIX_C_SOURCE 200809L

#include "ringbuf.h"

#include <stdlib.h>
#include <string.h>
#include <time.h>

static uint64_t now_ns(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

int rb_init(struct ringbuf *rb, int slots, size_t slot_cap)
{
	if (!rb || slot_cap == 0)
		return -1;

	if (slots < RB_MIN_SLOTS)
		slots = RB_MIN_SLOTS;
	if (slots > RB_MAX_SLOTS)
		slots = RB_MAX_SLOTS;

	memset(rb, 0, sizeof(*rb));
	rb->slots = slots;
	rb->slot_cap = slot_cap;
	rb->pool = calloc((size_t)slots, slot_cap);
	rb->meta = calloc((size_t)slots, sizeof(*rb->meta));
	if (!rb->pool || !rb->meta) {
		free(rb->pool);
		free(rb->meta);
		return -1;
	}
	if (pthread_mutex_init(&rb->lock, NULL) != 0) {
		free(rb->pool);
		free(rb->meta);
		return -1;
	}
	if (pthread_cond_init(&rb->not_empty, NULL) != 0) {
		pthread_mutex_destroy(&rb->lock);
		free(rb->pool);
		free(rb->meta);
		return -1;
	}
	return slots;
}

void rb_destroy(struct ringbuf *rb)
{
	if (!rb)
		return;
	pthread_cond_destroy(&rb->not_empty);
	pthread_mutex_destroy(&rb->lock);
	free(rb->pool);
	free(rb->meta);
	rb->pool = NULL;
	rb->meta = NULL;
}

bool rb_push(struct ringbuf *rb, const void *src, size_t len,
	     const struct rb_meta *meta)
{
	bool overwrote = false;

	if (len > rb->slot_cap)
		len = rb->slot_cap;

	pthread_mutex_lock(&rb->lock);

	if (rb->count == rb->slots) {
		/*
		 * 满：覆盖最旧帧。这就是 rules §4 要的行为——宁可丢帧，
		 * 也不让采集线程在这里等消费者。
		 */
		rb->read_idx = (rb->read_idx + 1) % rb->slots;
		rb->count--;
		rb->drop_overwrite++;
		overwrote = true;
	}

	memcpy(rb->pool + (size_t)rb->write_idx * rb->slot_cap, src, len);
	rb->meta[rb->write_idx] = *meta;
	rb->meta[rb->write_idx].len = len;

	rb->write_idx = (rb->write_idx + 1) % rb->slots;
	rb->count++;
	rb->frames_in++;
	if (rb->count > rb->high_watermark)
		rb->high_watermark = rb->count;

	pthread_cond_signal(&rb->not_empty);
	pthread_mutex_unlock(&rb->lock);
	return overwrote;
}

int rb_pop(struct ringbuf *rb, void *dst, struct rb_meta *meta_out)
{
	uint64_t t0, t1;

	pthread_mutex_lock(&rb->lock);
	while (rb->count == 0 && !rb->stopping) {
		t0 = now_ns();
		pthread_cond_wait(&rb->not_empty, &rb->lock);
		t1 = now_ns();
		rb->wait_ns_total += t1 - t0;
	}
	if (rb->count == 0) { /* stopping 且已空 */
		pthread_mutex_unlock(&rb->lock);
		return -1;
	}

	/*
	 * 锁内只做拷贝。分包发送必须留到锁外——那是本设计的关键，
	 * 否则对端一慢就会反过来把采集线程堵住。
	 */
	t0 = now_ns();
	memcpy(dst, rb->pool + (size_t)rb->read_idx * rb->slot_cap,
	       rb->meta[rb->read_idx].len);
	if (meta_out)
		*meta_out = rb->meta[rb->read_idx];
	t1 = now_ns();
	rb->copy_ns_total += t1 - t0;

	rb->read_idx = (rb->read_idx + 1) % rb->slots;
	rb->count--;
	rb->frames_out++;

	pthread_mutex_unlock(&rb->lock);
	return 0;
}

void rb_stop(struct ringbuf *rb)
{
	pthread_mutex_lock(&rb->lock);
	rb->stopping = true;
	pthread_cond_broadcast(&rb->not_empty);
	pthread_mutex_unlock(&rb->lock);
}

void rb_get_stats(struct ringbuf *rb, struct rb_stats *out)
{
	pthread_mutex_lock(&rb->lock);
	out->frames_in = rb->frames_in;
	out->frames_out = rb->frames_out;
	out->drop_overwrite = rb->drop_overwrite;
	out->copy_ns_total = rb->copy_ns_total;
	out->wait_ns_total = rb->wait_ns_total;
	out->high_watermark = rb->high_watermark;
	out->slots = rb->slots;
	out->count = rb->count;
	pthread_mutex_unlock(&rb->lock);
}
