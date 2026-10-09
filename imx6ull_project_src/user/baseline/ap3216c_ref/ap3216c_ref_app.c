#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: ap3216cApp.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: ap3216c设备测试APP。
其他	   	: 无
使用方法	 ：./ap3216c_ref_app /dev/ap3216c_ref
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/9/20 左忠凯创建
***************************************************************/

/*
 * @description		: main主程序
 * @param - argc 	: argv数组元素个数
 * @param - argv 	: 具体参数
 * @return 			: 0 成功;其他 失败
 */
int main(int argc, char *argv[])
{
	int fd;
	char *filename;
	unsigned short databuf[3];
	unsigned short ir, als, ps;
	int ret = 0;
	int loop = -1;

	if (argc != 2 && argc != 3) {
		printf("Error Usage!\r\n");
		printf("usage: %s /dev/ap3216c_ref [count]\r\n", argv[0]);
		return -1;
	}

	filename = argv[1];
	if (argc == 3) {
		loop = atoi(argv[2]);
		if (loop <= 0)
			loop = -1;
	}
	fd = open(filename, O_RDWR);
	if(fd < 0) {
		printf("can't open file %s\r\n", filename);
		return -1;
	}

	while (loop != 0) {
		ret = read(fd, databuf, sizeof(databuf));
		if(ret == 0) { 			/* 数据读取成功 */
			ir =  databuf[0]; 	/* ir传感器数据 */
			als = databuf[1]; 	/* als传感器数据 */
			ps =  databuf[2]; 	/* ps传感器数据 */
			printf("ir = %d, als = %d, ps = %d\r\n", ir, als, ps);
		}
		usleep(200000); /*100ms */
		if (loop > 0)
			loop--;
	}
	close(fd);	/* 关闭文件 */	
	return 0;
}
