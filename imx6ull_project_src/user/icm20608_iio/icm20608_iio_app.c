/***************************************************************
 * ICM20608 IIO 用户态测试程序（Phase 4）
 *
 * 底本：reference/01、例程源码/02、Linux驱动例程/27_iio/spi/icm20608App.c
 *       （正点原子，2021/08/13）
 *
 * 相对底本的关键改动：
 *   1. 不硬编码 iio:deviceN。底本要求用户手填 /sys/bus/iio/devices/iio:deviceX，
 *      本程序扫描 name 属性，找到值为 "icm20608_iio" 的那个再拼路径。
 *   2. 支持可选参数：采样次数（默认 10，0 表示一直采）。
 *
 * 用法：./icm20608_iio_app [采样次数]      # 默认 10 次
 ***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#define IIO_NAME_WANT   "icm20608_iio"
#define IIO_DEV_MAX     64      /* 扫描 iio:device0..63 */

/*
 * 扫描 /sys/bus/iio/devices/ 下的 iio:deviceN/name，找 name == want 的设备目录。
 * 成功返回 0，并把目录路径写入 path（不含结尾 /）；失败返回 -1。
 */
static int find_iio_device(const char *want, char *path, size_t path_sz)
{
    char npath[128];
    char buf[128];
    int i, fd, n;

    for (i = 0; i < IIO_DEV_MAX; i++) {
        snprintf(npath, sizeof(npath),
                 "/sys/bus/iio/devices/iio:device%d/name", i);
        fd = open(npath, O_RDONLY);
        if (fd < 0)
            continue;
        n = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        if (n <= 0)
            continue;
        buf[n] = '\0';
        /* 去掉行尾换行 */
        while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r'))
            buf[--n] = '\0';

        if (strcmp(buf, want) == 0) {
            snprintf(path, path_sz, "/sys/bus/iio/devices/iio:device%d", i);
            return 0;
        }
    }
    return -1;
}

/* 读一个整数型 sysfs 属性；失败返回 INT_MIN 以便区分 */
static int read_int_attr(const char *dir, const char *attr)
{
    char p[256];
    char buf[64];
    int fd, n;

    snprintf(p, sizeof(p), "%s/%s", dir, attr);
    fd = open(p, O_RDONLY);
    if (fd < 0)
        return -0x7fffffff;
    n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0)
        return -0x7fffffff;
    buf[n] = '\0';
    return atoi(buf);
}

int main(int argc, char *argv[])
{
    char dir[160];
    int loops = 10;
    long samples = 0;

    if (argc > 2) {
        printf("用法: %s [采样次数(默认10, 0=一直采)]\r\n", argv[0]);
        return -1;
    }
    if (argc == 2)
        loops = atoi(argv[1]);

    if (find_iio_device(IIO_NAME_WANT, dir, sizeof(dir)) != 0) {
        printf("未找到 IIO 设备 name=%s（驱动是否已 insmod？）\r\n", IIO_NAME_WANT);
        printf("可用设备:\r\n");
        system("for d in /sys/bus/iio/devices/iio:device*; do "
               "[ -e \"$d/name\" ] && echo \"  $d = $(cat $d/name)\"; done");
        return -1;
    }

    printf("找到 IIO 设备: %s (name=%s)\r\n", dir, IIO_NAME_WANT);

    while (loops == 0 || samples < loops) {
        int gx = read_int_attr(dir, "in_anglvel_x_raw");
        int gy = read_int_attr(dir, "in_anglvel_y_raw");
        int gz = read_int_attr(dir, "in_anglvel_z_raw");
        int ax = read_int_attr(dir, "in_accel_x_raw");
        int ay = read_int_attr(dir, "in_accel_y_raw");
        int az = read_int_attr(dir, "in_accel_z_raw");
        int temp = read_int_attr(dir, "in_temp_raw");

        printf("\r\n原始值:\r\n");
        printf("gx = %d, gy = %d, gz = %d\r\n", gx, gy, gz);
        printf("ax = %d, ay = %d, az = %d\r\n", ax, ay, az);
        printf("temp = %d\r\n", temp);
        printf("实际值:");
        printf("act gx = %.2f°/S, act gy = %.2f°/S, act gz = %.2f°/S\r\n",
               gx / 16.4, gy / 16.4, gz / 16.4);
        printf("act ax = %.2fg, act ay = %.2fg, act az = %.2fg\r\n",
               ax / 2048.0, ay / 2048.0, az / 2048.0);
        printf("act temp = %.2f°C\r\n", (temp - 25) / 326.8 + 25);

        samples++;
        if (loops != 0 && samples >= loops)
            break;
        usleep(200000); /* 200ms */
    }

    return 0;
}
