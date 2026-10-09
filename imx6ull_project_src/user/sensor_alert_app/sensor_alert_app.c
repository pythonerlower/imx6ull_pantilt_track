/**
 * sensor_alert_app.c - 传感器联动告警应用
 *
 * 功能：
 * 1. 监听 mx6hub 获取 AP3216C + ICM20608 传感器数据
 * 2. 检测异常事件（靠近/剧烈移动）
 * 3. 触发告警：LED + BEEP + 告警音频
 * 4. 高性能/低功耗模式切换
 *
 * 编译：arm-linux-gnueabihf-gcc -o sensor_alert_app sensor_alert_app.c -lpthread
 * 运行：./sensor_alert_app --dev /dev/mx6hub0
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include "../../kernel/mx6hub/mx6hub_uapi.h"

/* ==================== 配置参数 ==================== */
#define DEFAULT_HUB_DEV      "/dev/mx6hub0"
#define DEFAULT_LED_DEV      "/dev/dtsplatled"
#define DEFAULT_BEEP_DEV     "/dev/miscbeep"
#define DEFAULT_AUDIO_DEV    "plughw:0,0"
#define DEFAULT_ALERT_WAV   "/tmp/alert_tone.wav"

#define EPOLL_TIMEOUT_MS    1000
#define STATS_INTERVAL_SEC  5

/* 触发阈值 */
#define PS_NEAR_THRESHOLD   100     /* 接近检测阈值 */
#define PS_DELTA_THRESHOLD  50      /* PS 变化阈值 */
#define MOTION_THRESHOLD    3000    /* 加速度变化阈值 (mg) */
#define MOTION_WINDOW_MS    500     /* 运动检测窗口 */

/* ==================== 状态枚举 ==================== */
typedef enum {
    MODE_LOW_POWER = 0,    /* 低功耗模式：低采样率 */
    MODE_HIGH_PERF = 1,    /* 高性能模式：正常采样率 */
} alert_mode_t;

typedef enum {
    ALERT_IDLE = 0,
    ALERT_NEAR = 1,       /* 接近告警 */
    ALERT_MOTION = 2,     /* 运动告警 */
} alert_state_t;

/* ==================== 全局状态 ==================== */
static volatile int g_running = 1;
static const char *g_hub_dev = DEFAULT_HUB_DEV;
static const char *g_led_dev = DEFAULT_LED_DEV;
static const char *g_beep_dev = DEFAULT_BEEP_DEV;
static const char *g_audio_dev = DEFAULT_AUDIO_DEV;

/* 告警状态 */
static alert_mode_t g_current_mode = MODE_LOW_POWER;
static alert_state_t g_alert_state = ALERT_IDLE;
static time_t g_last_alert_time = 0;
static int g_alert_cooldown_sec = 3;  /* 告警冷却时间 */

/* 传感器历史数据 */
static struct {
    uint16_t ps;          /* 最近 PS 值 */
    int16_t ax, ay, az;  /* 最近加速度 */
    time_t last_update;
} g_sensor_history;

/* 统计信息 */
static struct {
    uint64_t total_events;
    uint64_t near_alerts;
    uint64_t motion_alerts;
    uint64_t led_triggers;
    uint64_t beep_triggers;
    uint64_t audio_triggers;
    uint64_t mode_switches;
} g_stats = {0};

/* ==================== 工具函数 ==================== */
static uint64_t get_time_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void print_usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "Options:\n"
        "  --dev <path>       mx6hub device (default: %s)\n"
        "  --led <path>       LED device (default: %s)\n"
        "  --beep <path>      BEEP device (default: %s)\n"
        "  --audio <dev>      Audio device (default: %s)\n"
        "  --cooldown <sec>   Alert cooldown (default: 3)\n"
        "  --help             Show this help\n",
        prog, DEFAULT_HUB_DEV, DEFAULT_LED_DEV, DEFAULT_BEEP_DEV, DEFAULT_AUDIO_DEV);
}

/* ==================== 告警控制 ==================== */
static int led_on(void)
{
    int fd = open(g_led_dev, O_WRONLY);
    if (fd < 0) {
        perror("open led");
        return -1;
    }
    write(fd, "\x01", 1);
    close(fd);
    g_stats.led_triggers++;
    return 0;
}

static int led_off(void)
{
    int fd = open(g_led_dev, O_WRONLY);
    if (fd < 0)
        return -1;
    write(fd, "\x00", 1);
    close(fd);
    return 0;
}

static int beep_on(void)
{
    int fd = open(g_beep_dev, O_WRONLY);
    if (fd < 0) {
        perror("open beep");
        return -1;
    }
    write(fd, "\x01", 1);
    close(fd);
    g_stats.beep_triggers++;
    return 0;
}

static int beep_off(void)
{
    int fd = open(g_beep_dev, O_WRONLY);
    if (fd < 0)
        return -1;
    write(fd, "\x00", 1);
    close(fd);
    return 0;
}

/* 播放告警音（生成简单正弦波） */
static int play_alert_tone(int freq_hz, int duration_ms)
{
    int fd = -1;
    int ret = -1;
    int sample_rate = 44100;
    int num_samples = sample_rate * duration_ms / 1000;
    int16_t *buffer = NULL;
    int written;

    /* 打开音频设备 */
    fd = open("/dev/snd/pcmC0D0p", O_WRONLY);
    if (fd < 0) {
        /* 尝试 PulseAudio 默认设备 */
        fd = open("/dev/snd/pcmC0D0p", O_WRONLY);
        if (fd < 0) {
            perror("open audio");
            return -1;
        }
    }

    /* 生成正弦波 */
    buffer = calloc(num_samples, sizeof(int16_t));
    if (!buffer) {
        goto out;
    }

    for (int i = 0; i < num_samples; i++) {
        double t = (double)i / sample_rate;
        double val = sin(2.0 * M_PI * freq_hz * t);
        /* 渐出效果 */
        double envelope = 1.0;
        int fade_samples = sample_rate / 10;  /* 0.1秒渐变 */
        if (i < fade_samples)
            envelope = (double)i / fade_samples;
        else if (i > num_samples - fade_samples)
            envelope = (double)(num_samples - i) / fade_samples;
        buffer[i] = (int16_t)(val * 20000 * envelope);
    }

    /* 写入音频数据 */
    written = write(fd, buffer, num_samples * sizeof(int16_t));
    if (written > 0)
        ret = 0;

out:
    free(buffer);
    if (fd >= 0)
        close(fd);

    if (ret == 0)
        g_stats.audio_triggers++;

    return ret;
}

/* ==================== 告警触发 ==================== */
static void trigger_alert(alert_state_t state)
{
    time_t now = time(NULL);

    /* 冷却检查 */
    if (g_alert_cooldown_sec > 0 && 
        difftime(now, g_last_alert_time) < g_alert_cooldown_sec) {
        return;
    }

    g_alert_state = state;
    g_last_alert_time = now;

    switch (state) {
    case ALERT_NEAR:
        printf("[ALERT] Near detected! PS=%u\n", g_sensor_history.ps);
        g_stats.near_alerts++;
        /* 接近告警：LED 快闪 + 短促蜂鸣 */
        led_on();
        beep_on();
        usleep(100000);
        beep_off();
        usleep(100000);
        beep_on();
        usleep(100000);
        beep_off();
        led_off();
        /* 播放告警音 */
        play_alert_tone(880, 300);  /* A5 音符 */
        break;

    case ALERT_MOTION:
        printf("[ALERT] Motion detected! ax=%d ay=%d az=%d\n",
               g_sensor_history.ax, g_sensor_history.ay, g_sensor_history.az);
        g_stats.motion_alerts++;
        /* 运动告警：LED 常亮 + 持续蜂鸣 */
        led_on();
        beep_on();
        sleep(1);
        beep_off();
        led_off();
        /* 播放告警音 */
        play_alert_tone(440, 500);  /* A4 音符 */
        break;

    default:
        break;
    }

    /* 切换到高性能模式 */
    if (g_current_mode == MODE_LOW_POWER) {
        g_current_mode = MODE_HIGH_PERF;
        g_stats.mode_switches++;
        printf("[MODE] Switched to HIGH_PERFORMANCE\n");
    }
}

/* ==================== 模式切换 ==================== */
static void switch_to_low_power(void)
{
    if (g_current_mode == MODE_LOW_POWER)
        return;
    g_current_mode = MODE_LOW_POWER;
    g_stats.mode_switches++;
    printf("[MODE] Switched to LOW_POWER\n");
}

static void check_and_switch_mode(void)
{
    time_t now = time(NULL);

    /* 告警后一段时间无异常，切换回低功耗 */
    if (g_current_mode == MODE_HIGH_PERF && g_alert_state == ALERT_IDLE) {
        if (difftime(now, g_last_alert_time) > 10) {  /* 10秒无告警 */
            switch_to_low_power();
        }
    }
}

/* ==================== 传感器分析 ==================== */
static void analyze_ap3216c(const struct mx6hub_ap3216_payload *p)
{
    static uint16_t last_ps = 0;

    /* 更新历史 */
    g_sensor_history.ps = p->ps;
    g_sensor_history.last_update = time(NULL);

    /* 接近检测 */
    if (p->ps > PS_NEAR_THRESHOLD && last_ps <= PS_NEAR_THRESHOLD) {
        trigger_alert(ALERT_NEAR);
    }

    /* PS 剧烈变化检测 */
    if (p->ps > PS_DELTA_THRESHOLD && 
        (p->ps > last_ps + PS_DELTA_THRESHOLD * 2 || 
         (last_ps > p->ps && last_ps - p->ps > PS_DELTA_THRESHOLD * 2))) {
        trigger_alert(ALERT_NEAR);
    }

    last_ps = p->ps;
}

static void analyze_icm20608(const struct mx6hub_icm20608_payload *p)
{
    static int16_t last_ax = 0, last_ay = 0, last_az = 0;
    static uint64_t last_motion_ts = 0;

    int32_t delta_ax = abs(p->accel_x - last_ax);
    int32_t delta_ay = abs(p->accel_y - last_ay);
    int32_t delta_az = abs(p->accel_z - last_az);

    /* 更新历史 */
    g_sensor_history.ax = p->accel_x;
    g_sensor_history.ay = p->accel_y;
    g_sensor_history.az = p->accel_z;

    /* 运动检测 */
    uint64_t now = get_time_ms();
    if (delta_ax > MOTION_THRESHOLD || delta_ay > MOTION_THRESHOLD || 
        delta_az > MOTION_THRESHOLD) {
        if (now - last_motion_ts > MOTION_WINDOW_MS) {
            last_motion_ts = now;
            trigger_alert(ALERT_MOTION);
        }
    }

    last_ax = p->accel_x;
    last_ay = p->accel_y;
    last_az = p->accel_z;
}

/* ==================== 主循环 ==================== */
static void process_event(const struct mx6hub_record *rec)
{
    g_stats.total_events++;

    switch (rec->source) {
    case MX6HUB_SRC_I2C_AP3216:
        if (rec->len >= sizeof(struct mx6hub_ap3216_payload)) {
            const struct mx6hub_ap3216_payload *p = 
                (const struct mx6hub_ap3216_payload *)rec->payload;
            printf("[AP3216C] als=%u ps=%u ir=%u seq=%u\n",
                   p->als, p->ps, p->ir, p->seq);
            analyze_ap3216c(p);
        }
        break;

    case MX6HUB_SRC_SPI_ICM20608:
        if (rec->len >= sizeof(struct mx6hub_icm20608_payload)) {
            const struct mx6hub_icm20608_payload *p = 
                (const struct mx6hub_icm20608_payload *)rec->payload;
            printf("[ICM20608] ax=%d ay=%d az=%d gx=%d gy=%d gz=%d seq=%u\n",
                   p->accel_x, p->accel_y, p->accel_z,
                   p->gyro_x, p->gyro_y, p->gyro_z, p->seq);
            analyze_icm20608(p);
        }
        break;

    case MX6HUB_SRC_HEARTBEAT:
        /* 心跳事件，可用于检查链路健康 */
        break;

    default:
        break;
    }

    /* 检查是否需要切换模式 */
    check_and_switch_mode();
}

static void print_stats(void)
{
    printf("\n========== Statistics ==========\n");
    printf("Mode: %s\n", g_current_mode == MODE_LOW_POWER ? "LOW_POWER" : "HIGH_PERF");
    printf("Total events:    %lu\n", (unsigned long)g_stats.total_events);
    printf("Near alerts:     %lu\n", (unsigned long)g_stats.near_alerts);
    printf("Motion alerts:   %lu\n", (unsigned long)g_stats.motion_alerts);
    printf("LED triggers:    %lu\n", (unsigned long)g_stats.led_triggers);
    printf("BEEP triggers:   %lu\n", (unsigned long)g_stats.beep_triggers);
    printf("Audio triggers:  %lu\n", (unsigned long)g_stats.audio_triggers);
    printf("Mode switches:   %lu\n", (unsigned long)g_stats.mode_switches);
    printf("================================\n\n");
}

static void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

int main(int argc, char **argv)
{
    int hub_fd = -1;
    int ep_fd = -1;
    struct epoll_event ev;
    time_t last_stats_time = time(NULL);

    /* 命令行参数解析 */
    static struct option long_options[] = {
        {"dev",      required_argument, 0, 'd'},
        {"led",      required_argument, 0, 'l'},
        {"beep",     required_argument, 0, 'b'},
        {"audio",    required_argument, 0, 'a'},
        {"cooldown", required_argument, 0, 'c'},
        {"help",     no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:l:b:a:c:h", long_options, NULL)) != -1) {
        switch (opt) {
        case 'd': g_hub_dev = optarg; break;
        case 'l': g_led_dev = optarg; break;
        case 'b': g_beep_dev = optarg; break;
        case 'a': g_audio_dev = optarg; break;
        case 'c': g_alert_cooldown_sec = atoi(optarg); break;
        case 'h': print_usage(argv[0]); return 0;
        default:  print_usage(argv[0]); return 1;
        }
    }

    printf("=== Sensor Alert Application ===\n");
    printf("Hub: %s, LED: %s, BEEP: %s\n", g_hub_dev, g_led_dev, g_beep_dev);
    printf("Alert cooldown: %d sec\n", g_alert_cooldown_sec);
    printf("Press Ctrl+C to exit\n\n");

    /* 信号处理 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* 打开 mx6hub 设备 */
    hub_fd = open(g_hub_dev, O_RDONLY | O_NONBLOCK);
    if (hub_fd < 0) {
        perror("open mx6hub");
        return 1;
    }

    /* 创建 epoll 实例 */
    ep_fd = epoll_create1(0);
    if (ep_fd < 0) {
        perror("epoll_create1");
        close(hub_fd);
        return 1;
    }

    ev.events = EPOLLIN;
    ev.data.fd = hub_fd;
    if (epoll_ctl(ep_fd, EPOLL_CTL_ADD, hub_fd, &ev) < 0) {
        perror("epoll_ctl");
        close(hub_fd);
        close(ep_fd);
        return 1;
    }

    /* 主循环 */
    while (g_running) {
        struct epoll_event events[4];
        int n = epoll_wait(ep_fd, events, 4, EPOLL_TIMEOUT_MS);

        if (n < 0) {
            if (errno == EINTR)
                continue;
            perror("epoll_wait");
            break;
        }

        for (int i = 0; i < n; i++) {
            if (events[i].data.fd == hub_fd && (events[i].events & EPOLLIN)) {
                struct mx6hub_record rec;
                ssize_t r = read(hub_fd, &rec, sizeof(rec));
                if (r == sizeof(rec)) {
                    process_event(&rec);
                }
            }
        }

        /* 定期打印统计 */
        time_t now = time(NULL);
        if (difftime(now, last_stats_time) >= STATS_INTERVAL_SEC) {
            print_stats();
            last_stats_time = now;
        }
    }

    /* 清理 */
    printf("\nShutting down...\n");
    led_off();
    beep_off();
    close(hub_fd);
    close(ep_fd);

    print_stats();
    printf("Done.\n");
    return 0;
}
