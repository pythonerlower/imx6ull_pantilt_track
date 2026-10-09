/*
 * AP3216C 测试应用程序
 * 
 * 功能: 读取 AP3216C 传感器数据 (IR, ALS, PS)
 * 设备: /dev/ap3216c_myself
 * 
 * 用法: ./ap3216c_myself_app [count]
 *   count - 读取次数，默认为无限循环
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

/* 读取数据结构: IR, ALS, PS 三个 16 位值 */
typedef struct {
    unsigned short ir;    /* 红外数据 */
    unsigned short als;    /* 环境光数据 */
    unsigned short ps;     /* 接近传感数据 */
} ap3216c_data_t;

int main(int argc, char *argv[])
{
    int fd;
    char *filename = "/dev/ap3216c_myself";
    ap3216c_data_t data;
    int ret;
    int loop = -1;  /* 默认无限循环 */
    
    /* 解析命令行参数 */
    if (argc > 1) {
        filename = argv[1];
    }
    
    if (argc > 2) {
        loop = atoi(argv[2]);
        if (loop <= 0)
            loop = -1;
    }
    
    printf("AP3216C Test Application\n");
    printf("Device: %s\n", filename);
    printf("Loop: %s\n", (loop < 0) ? "infinite" : "fixed");
    printf("------------------------------------\n");
    
    /* 打开设备 */
    fd = open(filename, O_RDWR);
    if (fd < 0) {
        perror("Error: Cannot open device");
        printf("Make sure the driver is loaded:\n");
        printf("  1. Unbind BSP driver: echo 0-001e > /sys/bus/i2c/drivers/ap3216c/unbind\n");
        printf("  2. Load driver: insmod ap3216c_myself.ko\n");
        return -1;
    }
    
    printf("Device opened successfully\n");
    printf("Reading sensor data...\n\n");
    
    /* 主循环: 读取数据 */
    while (loop != 0) {
        memset(&data, 0, sizeof(data));
        
        ret = read(fd, &data, sizeof(data));
        if (ret == sizeof(data)) {
            printf("ir = %5u, als = %5u, ps = %5u\r\n", 
                   data.ir, data.als, data.ps);
            fflush(stdout);
        } else if (ret < 0) {
            perror("Read error");
            break;
        } else {
            printf("Incomplete read: %d bytes\n", ret);
        }
        
        usleep(200000);  /* 200ms 间隔 */
        
        if (loop > 0)
            loop--;
    }
    
    close(fd);
    printf("\nTest complete.\n");
    
    return 0;
}
