#ifndef __MX6HUB_H__
#define __MX6HUB_H__

#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "mx6hub_uapi.h"

#define MX6HUB_FIFO_DEPTH 1024

struct mx6hub_dev;
struct regmap;
struct spi_device;

struct mx6hub_diag {
    u64 irq_top_half_events;
    u64 threaded_irq_events;
    u64 i2c_work_events;
    u64 spi_work_events;
    u64 fifo_high_wm;
    u64 latency_last_us;
    u64 latency_min_us;
    u64 latency_max_us;
    u64 latency_total_us;
    u64 latency_samples;
};

struct mx6hub_i2c_ctx {
    struct work_struct work;
    struct mx6hub_dev *hub;
    struct mutex lock;
    struct i2c_adapter *adap;
    bool enabled;
    bool hw_ready;
    bool fallback_active;
    u32 seq;
    u32 read_err;
    u32 retry_cnt;
    u32 real_events;
    u32 fallback_events;
    int last_err;
};

struct mx6hub_spi_ctx {
    struct work_struct work;
    struct mx6hub_dev *hub;
    struct mutex lock;
    bool enabled;
    bool hw_ready;
    bool spi_created;
    bool use_sw_cs;
    bool wom_enabled;
    u32 seq;
    u32 read_err;
    u32 retry_cnt;
    u32 reinit_cnt;
    u32 real_events;
    u32 fallback_events;
    u32 wom_skipped;
    int last_err;
    u8 whoami;
    int cs_gpio;
    struct regmap *regmap;
    struct spi_device *spi;
};

struct mx6hub_dev {
    struct device *dev;
    dev_t devt;
    struct cdev cdev;
    struct class *cls;
    wait_queue_head_t rd_waitq;

    spinlock_t fifo_lock;
    spinlock_t stats_lock;
    struct kfifo fifo;
    struct mutex ioctl_lock;

    struct timer_list hb_timer;
    struct gpio_desc *irq_gpiod;
    int irq;

    struct workqueue_struct *wq;
    struct mx6hub_i2c_ctx i2c;
    struct mx6hub_spi_ctx spi;

    struct dentry *dbg_dir;

    struct mx6hub_stats stats;
    struct mx6hub_diag diag;
};

int mx6hub_enqueue_record(struct mx6hub_dev *hub, const struct mx6hub_record *rec);
void mx6hub_fill_record(struct mx6hub_record *rec, u32 src, const void *payload, u32 len);
void mx6hub_mark_irq_top(struct mx6hub_dev *hub);
void mx6hub_mark_threaded_irq(struct mx6hub_dev *hub);
void mx6hub_mark_i2c_work(struct mx6hub_dev *hub);
void mx6hub_mark_spi_work(struct mx6hub_dev *hub);

int mx6hub_i2c_init(struct mx6hub_dev *hub);
void mx6hub_i2c_exit(struct mx6hub_dev *hub);
void mx6hub_i2c_schedule(struct mx6hub_dev *hub);

int mx6hub_spi_init(struct mx6hub_dev *hub);
void mx6hub_spi_exit(struct mx6hub_dev *hub);
void mx6hub_spi_schedule(struct mx6hub_dev *hub);

#endif
