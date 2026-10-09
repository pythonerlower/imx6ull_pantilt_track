#ifndef __MX6HUB_UAPI_H__
#define __MX6HUB_UAPI_H__

#include <linux/ioctl.h>
#include <linux/types.h>

#define MX6HUB_IOC_MAGIC 'H'

enum mx6hub_source {
    MX6HUB_SRC_HEARTBEAT = 0,
    MX6HUB_SRC_IRQ_GPIO = 1,
    MX6HUB_SRC_INPUT_KEY = 2,
    MX6HUB_SRC_I2C_AP3216 = 3,
    MX6HUB_SRC_SPI_ICM20608 = 4,
    MX6HUB_SRC_USER_INJECT = 5,
};

struct mx6hub_record {
    __u64 ts_ns;
    __u32 source;
    __u32 len;
    __u8 payload[32];
};

/*
 * Stable payload ABI for AP3216C events (source=MX6HUB_SRC_I2C_AP3216).
 * Keep this in UAPI so kernel/user parsers share one canonical layout.
 */
struct mx6hub_ap3216_payload {
    __u16 als;
    __u16 ps;
    __u16 ir;
    __u16 seq;
} __attribute__((packed));

/*
 * Stable payload ABI for ICM20608 events (source=MX6HUB_SRC_SPI_ICM20608).
 * Values are raw sensor samples from device registers.
 */
struct mx6hub_icm20608_payload {
    __s16 accel_x;
    __s16 accel_y;
    __s16 accel_z;
    __s16 gyro_x;
    __s16 gyro_y;
    __s16 gyro_z;
    __u16 seq;
} __attribute__((packed));

struct mx6hub_stats {
    __u64 total_events;
    __u64 dropped_events;
    __u64 read_events;
    __u64 irq_events;
    __u64 heartbeat_events;
    __u64 user_inject_events;

    __u64 irq_top_half_events;
    __u64 threaded_irq_events;
    __u64 i2c_work_events;
    __u64 spi_work_events;
    __u64 fifo_high_wm;

    __u64 latency_last_us;
    __u64 latency_min_us;
    __u64 latency_max_us;
    __u64 latency_avg_us;
    __u64 latency_samples;
};

struct mx6hub_inject {
    __u32 source;
    __u32 len;
    __u8 payload[32];
};

#define MX6HUB_IOC_GET_STATS _IOR(MX6HUB_IOC_MAGIC, 0x01, struct mx6hub_stats)
#define MX6HUB_IOC_CLR_STATS _IO(MX6HUB_IOC_MAGIC, 0x02)
#define MX6HUB_IOC_INJECT _IOW(MX6HUB_IOC_MAGIC, 0x03, struct mx6hub_inject)

#endif
