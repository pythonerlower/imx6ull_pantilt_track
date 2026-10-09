#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "../../kernel/mx6hub/mx6hub_uapi.h"

#define DEFAULT_QUEUE_DEPTH 128U
#define DEFAULT_LAT_SAMPLES 4096U
#define MAX_WORKERS 32

struct queued_record {
    struct mx6hub_record rec;
};

struct monitor_stats {
    uint64_t src_i2c;
    uint64_t src_spi;
    uint64_t src_other;
    uint64_t processed;
    uint64_t app_queue_drops;
    uint64_t *lat_samples;
    size_t lat_sample_cap;
    size_t lat_sample_count;
};

struct record_queue {
    struct queued_record *items;
    size_t cap;
    size_t head;
    size_t tail;
    size_t count;
    bool stop;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
};

struct monitor_ctx {
    FILE *csv;
    struct monitor_stats stats;
    pthread_mutex_t stats_lock;
    pthread_mutex_t output_lock;
    struct record_queue queue;
};

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s [--dev /dev/mx6hub0] [--sec 30] [--csv out.csv]\n"
            "          [--nonblock] [--workers N] [--queue-depth N]\n"
            "          [--lat-samples N]\n",
            prog);
}

static int parse_positive_int(const char *text, int min, int max, int *out)
{
    char *end = NULL;
    long val;

    errno = 0;
    val = strtol(text, &end, 10);
    if (errno || !end || *end || val < min || val > max)
        return -EINVAL;

    *out = (int)val;
    return 0;
}

static int cmp_u64(const void *a, const void *b)
{
    const uint64_t av = *(const uint64_t *)a;
    const uint64_t bv = *(const uint64_t *)b;

    if (av < bv)
        return -1;
    if (av > bv)
        return 1;
    return 0;
}

static uint64_t percentile_u64(uint64_t *sorted, size_t count, unsigned int pct)
{
    size_t idx;

    if (!count)
        return 0;

    idx = ((count - 1) * (size_t)pct + 50U) / 100U;
    if (idx >= count)
        idx = count - 1;
    return sorted[idx];
}

static void print_record_unlocked(const struct mx6hub_record *rec, uint64_t lat_us)
{
    if (rec->source == MX6HUB_SRC_I2C_AP3216 &&
        rec->len >= sizeof(struct mx6hub_ap3216_payload)) {
        const struct mx6hub_ap3216_payload *p =
            (const struct mx6hub_ap3216_payload *)rec->payload;
        printf("src=%u(AP3216C) len=%u lat=%" PRIu64 "us als=%u ps=%u ir=%u seq=%u\n",
               rec->source, rec->len, lat_us, p->als, p->ps, p->ir, p->seq);
        return;
    }

    if (rec->source == MX6HUB_SRC_SPI_ICM20608 &&
        rec->len >= sizeof(struct mx6hub_icm20608_payload)) {
        const struct mx6hub_icm20608_payload *p =
            (const struct mx6hub_icm20608_payload *)rec->payload;
        printf("src=%u(ICM20608) len=%u lat=%" PRIu64
               "us ax=%d ay=%d az=%d gx=%d gy=%d gz=%d seq=%u\n",
               rec->source, rec->len, lat_us, p->accel_x, p->accel_y, p->accel_z,
               p->gyro_x, p->gyro_y, p->gyro_z, p->seq);
        return;
    }

    printf("src=%u len=%u lat=%" PRIu64 "us p0=%u p1=%u p2=%u p3=%u\n",
           rec->source, rec->len, lat_us,
           rec->payload[0], rec->payload[1], rec->payload[2], rec->payload[3]);
}

static void note_record_stats(struct monitor_ctx *ctx,
                              const struct mx6hub_record *rec,
                              uint64_t lat_us)
{
    pthread_mutex_lock(&ctx->stats_lock);
    if (rec->source == MX6HUB_SRC_I2C_AP3216)
        ctx->stats.src_i2c++;
    else if (rec->source == MX6HUB_SRC_SPI_ICM20608)
        ctx->stats.src_spi++;
    else
        ctx->stats.src_other++;

    ctx->stats.processed++;
    if (ctx->stats.lat_sample_count < ctx->stats.lat_sample_cap)
        ctx->stats.lat_samples[ctx->stats.lat_sample_count++] = lat_us;
    pthread_mutex_unlock(&ctx->stats_lock);
}

static void process_record(struct monitor_ctx *ctx, const struct mx6hub_record *rec)
{
    uint64_t lat_us = (now_ns() - rec->ts_ns) / 1000ULL;

    note_record_stats(ctx, rec, lat_us);

    pthread_mutex_lock(&ctx->output_lock);
    print_record_unlocked(rec, lat_us);
    if (ctx->csv) {
        fprintf(ctx->csv, "%" PRIu64 ",%u,%u,%" PRIu64 ",%u,%u,%u,%u\n",
                (uint64_t)rec->ts_ns, rec->source, rec->len, lat_us,
                rec->payload[0], rec->payload[1], rec->payload[2], rec->payload[3]);
    }
    pthread_mutex_unlock(&ctx->output_lock);
}

static int queue_init(struct record_queue *q, size_t cap)
{
    int ret;

    memset(q, 0, sizeof(*q));
    q->items = calloc(cap, sizeof(*q->items));
    if (!q->items)
        return -ENOMEM;
    q->cap = cap;

    ret = pthread_mutex_init(&q->lock, NULL);
    if (ret) {
        free(q->items);
        memset(q, 0, sizeof(*q));
        return -ret;
    }
    ret = pthread_cond_init(&q->not_empty, NULL);
    if (ret) {
        pthread_mutex_destroy(&q->lock);
        free(q->items);
        memset(q, 0, sizeof(*q));
        return -ret;
    }
    return 0;
}

static void queue_destroy(struct record_queue *q)
{
    pthread_cond_destroy(&q->not_empty);
    pthread_mutex_destroy(&q->lock);
    free(q->items);
    memset(q, 0, sizeof(*q));
}

static bool queue_push(struct monitor_ctx *ctx, const struct mx6hub_record *rec)
{
    struct record_queue *q = &ctx->queue;
    bool queued = false;

    pthread_mutex_lock(&q->lock);
    if (q->count < q->cap) {
        q->items[q->tail].rec = *rec;
        q->tail = (q->tail + 1U) % q->cap;
        q->count++;
        queued = true;
        pthread_cond_signal(&q->not_empty);
    }
    pthread_mutex_unlock(&q->lock);

    if (!queued) {
        pthread_mutex_lock(&ctx->stats_lock);
        ctx->stats.app_queue_drops++;
        pthread_mutex_unlock(&ctx->stats_lock);
    }
    return queued;
}

static bool queue_pop(struct record_queue *q, struct mx6hub_record *rec)
{
    bool ok = false;

    pthread_mutex_lock(&q->lock);
    while (!q->count && !q->stop)
        pthread_cond_wait(&q->not_empty, &q->lock);

    if (q->count) {
        *rec = q->items[q->head].rec;
        q->head = (q->head + 1U) % q->cap;
        q->count--;
        ok = true;
    }
    pthread_mutex_unlock(&q->lock);
    return ok;
}

static void queue_stop(struct record_queue *q)
{
    pthread_mutex_lock(&q->lock);
    q->stop = true;
    pthread_cond_broadcast(&q->not_empty);
    pthread_mutex_unlock(&q->lock);
}

static void *worker_main(void *arg)
{
    struct monitor_ctx *ctx = arg;
    struct mx6hub_record rec;

    while (queue_pop(&ctx->queue, &rec))
        process_record(ctx, &rec);

    return NULL;
}

static void print_app_latency_summary(struct monitor_ctx *ctx)
{
    uint64_t *copy;
    size_t count;

    pthread_mutex_lock(&ctx->stats_lock);
    count = ctx->stats.lat_sample_count;
    copy = calloc(count ? count : 1U, sizeof(*copy));
    if (copy && count)
        memcpy(copy, ctx->stats.lat_samples, count * sizeof(*copy));
    pthread_mutex_unlock(&ctx->stats_lock);

    if (!copy) {
        printf("app_latency samples=0 p50=0us p95=0us p99=0us note=oom\n");
        return;
    }

    qsort(copy, count, sizeof(*copy), cmp_u64);
    printf("app_latency samples=%zu p50=%" PRIu64 "us p95=%" PRIu64
           "us p99=%" PRIu64 "us\n",
           count,
           percentile_u64(copy, count, 50),
           percentile_u64(copy, count, 95),
           percentile_u64(copy, count, 99));
    free(copy);
}

int main(int argc, char **argv)
{
    const char *dev = "/dev/mx6hub0";
    const char *csv_path = NULL;
    int run_sec = 30;
    int worker_count = 0;
    int queue_depth = (int)DEFAULT_QUEUE_DEPTH;
    int lat_samples = (int)DEFAULT_LAT_SAMPLES;
    bool nonblock = false;
    int fd = -1;
    int ep = -1;
    int ret = 1;
    FILE *csv = NULL;
    pthread_t *workers = NULL;
    bool had_error = false;
    struct monitor_ctx ctx;
    struct mx6hub_stats stats;
    uint64_t deadline;

    memset(&ctx, 0, sizeof(ctx));

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--dev") && i + 1 < argc) {
            dev = argv[++i];
        } else if (!strcmp(argv[i], "--sec") && i + 1 < argc) {
            if (parse_positive_int(argv[++i], 1, 86400, &run_sec))
                goto usage_out;
        } else if (!strcmp(argv[i], "--csv") && i + 1 < argc) {
            csv_path = argv[++i];
        } else if (!strcmp(argv[i], "--nonblock")) {
            nonblock = true;
        } else if (!strcmp(argv[i], "--workers") && i + 1 < argc) {
            if (parse_positive_int(argv[++i], 0, MAX_WORKERS, &worker_count))
                goto usage_out;
        } else if (!strcmp(argv[i], "--queue-depth") && i + 1 < argc) {
            if (parse_positive_int(argv[++i], 1, 4096, &queue_depth))
                goto usage_out;
        } else if (!strcmp(argv[i], "--lat-samples") && i + 1 < argc) {
            if (parse_positive_int(argv[++i], 1, 1000000, &lat_samples))
                goto usage_out;
        } else {
            goto usage_out;
        }
    }

    if (pthread_mutex_init(&ctx.stats_lock, NULL)) {
        fprintf(stderr, "pthread_mutex_init(stats) failed\n");
        goto out;
    }
    if (pthread_mutex_init(&ctx.output_lock, NULL)) {
        fprintf(stderr, "pthread_mutex_init(output) failed\n");
        goto destroy_stats_lock;
    }

    ctx.stats.lat_samples = calloc((size_t)lat_samples, sizeof(*ctx.stats.lat_samples));
    if (!ctx.stats.lat_samples) {
        fprintf(stderr, "latency sample allocation failed\n");
        goto destroy_output_lock;
    }
    ctx.stats.lat_sample_cap = (size_t)lat_samples;

    fd = open(dev, O_RDONLY | (nonblock ? O_NONBLOCK : 0));
    if (fd < 0) {
        perror("open");
        goto free_latency;
    }

    ep = epoll_create1(0);
    if (ep < 0) {
        perror("epoll_create1");
        goto close_fd;
    }

    struct epoll_event ev = {0};
    ev.events = EPOLLIN;
    ev.data.fd = fd;
    if (epoll_ctl(ep, EPOLL_CTL_ADD, fd, &ev) < 0) {
        perror("epoll_ctl");
        goto close_ep;
    }

    if (csv_path) {
        csv = fopen(csv_path, "w");
        if (!csv) {
            perror("fopen csv");
            goto close_ep;
        }
        ctx.csv = csv;
        fprintf(csv, "ts_ns,source,len,latency_us,p0,p1,p2,p3\n");
    }

    if (worker_count > 0) {
        int err;

        err = queue_init(&ctx.queue, (size_t)queue_depth);
        if (err) {
            fprintf(stderr, "queue init failed: %d\n", err);
            goto close_csv;
        }
        workers = calloc((size_t)worker_count, sizeof(*workers));
        if (!workers) {
            fprintf(stderr, "worker allocation failed\n");
            goto destroy_queue;
        }
        for (int i = 0; i < worker_count; i++) {
            err = pthread_create(&workers[i], NULL, worker_main, &ctx);
            if (err) {
                fprintf(stderr, "pthread_create failed: %d\n", err);
                had_error = true;
                worker_count = i;
                queue_stop(&ctx.queue);
                goto join_workers;
            }
        }
    }

    deadline = now_ns() + (uint64_t)run_sec * 1000000000ULL;
    printf("monitor start: dev=%s sec=%d nonblock=%d workers=%d queue_depth=%d\n",
           dev, run_sec, nonblock ? 1 : 0, worker_count, queue_depth);

    while (now_ns() < deadline) {
        struct epoll_event out_events[2];
        int n = epoll_wait(ep, out_events, 2, 1000);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            perror("epoll_wait");
            had_error = true;
            break;
        }
        if (n == 0)
            continue;

        for (int i = 0; i < n; i++) {
            if (!(out_events[i].events & EPOLLIN))
                continue;

            while (1) {
                struct mx6hub_record rec;
                ssize_t r = read(fd, &rec, sizeof(rec));
                if (r == (ssize_t)sizeof(rec)) {
                    if (worker_count > 0)
                        queue_push(&ctx, &rec);
                    else
                        process_record(&ctx, &rec);
                } else {
                    if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                        break;
                    if (r < 0)
                        perror("read");
                    had_error = true;
                    break;
                }
            }
        }
    }

    if (worker_count > 0)
        queue_stop(&ctx.queue);

join_workers:
    for (int i = 0; i < worker_count; i++)
        pthread_join(workers[i], NULL);
    free(workers);

    memset(&stats, 0, sizeof(stats));
    if (ioctl(fd, MX6HUB_IOC_GET_STATS, &stats) == 0) {
        uint64_t src_i2c;
        uint64_t src_spi;
        uint64_t src_other;
        uint64_t processed;
        uint64_t app_queue_drops;

        pthread_mutex_lock(&ctx.stats_lock);
        src_i2c = ctx.stats.src_i2c;
        src_spi = ctx.stats.src_spi;
        src_other = ctx.stats.src_other;
        processed = ctx.stats.processed;
        app_queue_drops = ctx.stats.app_queue_drops;
        pthread_mutex_unlock(&ctx.stats_lock);

        printf("summary src_i2c=%" PRIu64 " src_spi=%" PRIu64
               " src_other=%" PRIu64 " processed=%" PRIu64
               " app_queue_drop=%" PRIu64 "\n",
               src_i2c, src_spi, src_other, processed, app_queue_drops);
        printf("stats total=%" PRIu64 " drop=%" PRIu64 " read=%" PRIu64
               " irq=%" PRIu64 " hb=%" PRIu64 " inj=%" PRIu64 "\n",
               (uint64_t)stats.total_events, (uint64_t)stats.dropped_events,
               (uint64_t)stats.read_events, (uint64_t)stats.irq_events,
               (uint64_t)stats.heartbeat_events, (uint64_t)stats.user_inject_events);
        printf("diag irq_top=%" PRIu64 " irq_thread=%" PRIu64
               " i2c_wq=%" PRIu64 " spi_wq=%" PRIu64 " fifo_high=%" PRIu64 "\n",
               (uint64_t)stats.irq_top_half_events, (uint64_t)stats.threaded_irq_events,
               (uint64_t)stats.i2c_work_events, (uint64_t)stats.spi_work_events,
               (uint64_t)stats.fifo_high_wm);
        printf("latency last=%" PRIu64 "us min=%" PRIu64 "us max=%" PRIu64
               "us avg=%" PRIu64 "us samples=%" PRIu64 "\n",
               (uint64_t)stats.latency_last_us, (uint64_t)stats.latency_min_us,
               (uint64_t)stats.latency_max_us, (uint64_t)stats.latency_avg_us,
               (uint64_t)stats.latency_samples);
        print_app_latency_summary(&ctx);
    } else {
        perror("ioctl(GET_STATS)");
    }

    ret = had_error ? 1 : 0;

destroy_queue:
    if (worker_count > 0)
        queue_destroy(&ctx.queue);
close_csv:
    if (csv)
        fclose(csv);
close_ep:
    close(ep);
close_fd:
    close(fd);
free_latency:
    free(ctx.stats.lat_samples);
destroy_output_lock:
    pthread_mutex_destroy(&ctx.output_lock);
destroy_stats_lock:
    pthread_mutex_destroy(&ctx.stats_lock);
out:
    return ret;

usage_out:
    usage(argv[0]);
    return 1;
}
