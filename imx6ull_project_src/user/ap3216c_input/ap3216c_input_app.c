/***************************************************************
 * AP3216C Input 用户态测试程序（Phase 4 M1）
 *
 * 作用：监听 Input 子系统上报的接近事件（EV_SW / SW_FRONT_PROXIMITY），
 *       并在每次事件时顺带打印 sysfs 的 PS/ALS 原始值，便于人工验收。
 *
 * 设计要点：
 *   1. 不硬编码 eventX。扫描 /dev/input/event*，用 EVIOCGNAME 找到
 *      name == "ap3216c-proximity" 的那个节点。
 *   2. 阻塞读 struct input_event，只打印 type==EV_SW && code==SW_FRONT_PROXIMITY。
 *   3. sysfs 路径同样不硬编码总线号：扫描 /sys/bus/i2c/devices/ 下名字以 -001e 结尾的目录。
 *
 * 用法：./ap3216c_input_app [事件数(默认0=一直监听)]
 ***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/input.h>

#define INPUT_NAME_WANT     "ap3216c-proximity"
#define EVENT_DEV_MAX       32      /* 扫描 /dev/input/event0..31 */

/* 在 /dev/input/event* 中按 EVIOCGNAME 查找目标设备，返回 fd（失败 -1） */
static int find_event_fd(const char *want)
{
    char path[64];
    char name[128];
    int i, fd;

    for (i = 0; i < EVENT_DEV_MAX; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        fd = open(path, O_RDONLY);
        if (fd < 0)
            continue;
        memset(name, 0, sizeof(name));
        if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) >= 0 &&
            strcmp(name, want) == 0) {
            printf("找到 Input 设备: %s (name=%s)\r\n", path, name);
            return fd;
        }
        close(fd);
    }
    return -1;
}

/* 扫描 /sys/bus/i2c/devices/ 下名字以 -001e 结尾的目录，返回其路径（不含结尾 /） */
static int find_i2c_dir(char *out, size_t out_sz)
{
    DIR *d = opendir("/sys/bus/i2c/devices");
    struct dirent *e;

    if (!d)
        return -1;
    while ((e = readdir(d)) != NULL) {
        size_t len = strlen(e->d_name);
        if (len >= 5 && strcmp(e->d_name + len - 5, "-001e") == 0) {
            snprintf(out, out_sz, "/sys/bus/i2c/devices/%s", e->d_name);
            closedir(d);
            return 0;
        }
    }
    closedir(d);
    return -1;
}

/* 读一个整数型 sysfs 属性；失败返回 -1 并写 0 */
static int read_int_file(const char *path)
{
    char buf[64];
    int fd, n;

    fd = open(path, O_RDONLY);
    if (fd < 0)
        return -1;
    n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0)
        return -1;
    buf[n] = '\0';
    return atoi(buf);
}

int main(int argc, char *argv[])
{
    int fd;
    long want = 0;      /* 0 = 一直监听 */
    long got = 0;
    char i2c_dir[160];
    char p[256];
    int have_i2c;

    if (argc > 2) {
        printf("用法: %s [事件数(默认0=一直监听)]\r\n", argv[0]);
        return -1;
    }
    if (argc == 2)
        want = atol(argv[1]);

    /* 无缓冲：本程序常被 timeout/管道包着跑，全缓冲会丢掉被 SIGTERM 前的输出 */
    setvbuf(stdout, NULL, _IONBF, 0);

    have_i2c = (find_i2c_dir(i2c_dir, sizeof(i2c_dir)) == 0);
    if (have_i2c)
        printf("sysfs: %s\r\n", i2c_dir);
    else
        printf("警告: 未找到 *-001e I2C 设备目录，跳过 sysfs 打印\r\n");

    fd = find_event_fd(INPUT_NAME_WANT);
    if (fd < 0) {
        printf("未找到 Input 设备 name=%s（驱动是否已 insmod？）\r\n",
               INPUT_NAME_WANT);
        printf("提示: cat /proc/bus/input/devices | grep -A4 -i ap3216c\r\n");
        return -1;
    }

    printf("开始监听 EV_SW/SW_FRONT_PROXIMITY（手贴近/移开触发；Ctrl-C 退出）\r\n");

    while (want == 0 || got < want) {
        struct input_event ev;
        ssize_t n = read(fd, &ev, sizeof(ev));

        if (n != (ssize_t)sizeof(ev)) {
            if (n < 0)
                perror("read event");
            else
                printf("短读: %zd (期望 %zu)\r\n", n, sizeof(ev));
            break;
        }

        if (ev.type != EV_SW || ev.code != SW_FRONT_PROXIMITY)
            continue;

        got++;
        printf("type=%d code=%d value=%d  -> %s\r\n",
               ev.type, ev.code, ev.value,
               ev.value ? "NEAR(贴近)" : "FAR(远离)");

        if (have_i2c) {
            snprintf(p, sizeof(p), "%s/proximity/ps_raw_data", i2c_dir);
            printf("  ps_raw_data  = %d\r\n", read_int_file(p));
            snprintf(p, sizeof(p), "%s/ambient_light/als_raw_data", i2c_dir);
            printf("  als_raw_data = %d\r\n", read_int_file(p));
        }
        fflush(stdout);
    }

    close(fd);
    printf("\n共收到 %ld 个事件，退出。\r\n", got);
    return 0;
}
