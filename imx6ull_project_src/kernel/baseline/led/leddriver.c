#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: leddriver.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: 设备树下的platform驱动
其他	   	: 无
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/8/13 左忠凯创建
***************************************************************/

#define LEDDEV_CNT		1				/* 设备号长度 	*/
#define LEDDEV_NAME		"dtsplatled"	/* 设备名字 	*/
#define LEDOFF 			0
#define LEDON 			1

/* leddev设备结构体 */
struct leddev_dev{
	dev_t devid;				/* 设备号	*/
	struct cdev cdev;			/* cdev		*/
	struct class *class;		/* 类 		*/
	struct device *device;		/* 设备		*/
	int major;					/* 主设备号	*/	
	struct device_node *node;	/* LED设备节点 */
	int led0;					/* LED灯GPIO标号 */
};

struct leddev_dev leddev; 		/* led设备 */
static struct platform_device *led_fallback_pdev;

/*
 * @description		: LED打开/关闭
 * @param - sta 	: LEDON(0) 打开LED，LEDOFF(1) 关闭LED
 * @return 			: 无
 */
void led0_switch(u8 sta)
{
	if (sta == LEDON )
		gpio_set_value(leddev.led0, 0);
	else if (sta == LEDOFF)
		gpio_set_value(leddev.led0, 1);	
}

/*
 * @description		: 打开设备
 * @param - inode 	: 传递给驱动的inode
 * @param - filp 	: 设备文件，file结构体有个叫做private_data的成员变量
 * 					  一般在open的时候将private_data指向设备结构体。
 * @return 			: 0 成功;其他 失败
 */
static int led_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &leddev; /* 设置私有数据  */
	return 0;
}

/*
 * @description		: 向设备写数据 
 * @param - filp 	: 设备文件，表示打开的文件描述符
 * @param - buf 	: 要写给设备写入的数据
 * @param - cnt 	: 要写入的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 写入的字节数，如果为负值，表示写入失败
 */
static ssize_t led_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
	unsigned long not_copied;
	unsigned char databuf[1];
	unsigned char ledstat;

	/* 单字节控制协议：0 关灯，1 开灯，避免越界拷贝 */
	if (cnt < sizeof(databuf))
		return -EINVAL;
	not_copied = copy_from_user(databuf, buf, sizeof(databuf));
	if (not_copied) {
		printk("kernel write failed!\r\n");
		return -EFAULT;
	}
	
	ledstat = databuf[0];
	if (ledstat == LEDON) {
		led0_switch(LEDON);
	} else if (ledstat == LEDOFF) {
		led0_switch(LEDOFF);
	}
	/*
	 * write 语义：成功应返回“已消费字节数”。
	 * 返回 0 会让用户态（重定向/stdio）误判为未前进而重试，表现为“卡住”。
	 */
	return sizeof(databuf);
}

/* 设备操作函数 */
static struct file_operations led_fops = {
	.owner = THIS_MODULE,
	.open = led_open,
	.write = led_write,
};

/*
 * @description		: flatform驱动的probe函数，当驱动与
 * 					  设备匹配以后此函数就会执行
 * @param - dev 	: platform设备
 * @return 			: 0，成功;其他负值,失败
 */
static int led_probe(struct platform_device *dev)
{	
	int ret;

	printk("led driver and device was matched!\r\n");
	/* 1、设置设备号 */
	if (leddev.major) {
		leddev.devid = MKDEV(leddev.major, 0);
		ret = register_chrdev_region(leddev.devid, LEDDEV_CNT, LEDDEV_NAME);
	} else {
		ret = alloc_chrdev_region(&leddev.devid, 0, LEDDEV_CNT, LEDDEV_NAME);
		leddev.major = MAJOR(leddev.devid);
	}
	if (ret) {
		printk("alloc/register chrdev failed: %d\r\n", ret);
		return ret;
	}

	/* 2、注册设备      */
	cdev_init(&leddev.cdev, &led_fops);
	ret = cdev_add(&leddev.cdev, leddev.devid, LEDDEV_CNT);
	if (ret) {
		printk("cdev_add failed: %d\r\n", ret);
		goto err_unregister_chrdev;
	}

	/* 3、创建类      */
	leddev.class = class_create(THIS_MODULE, LEDDEV_NAME);
	if (IS_ERR(leddev.class)) {
		ret = PTR_ERR(leddev.class);
		goto err_del_cdev;
	}

	/* 4、创建设备 */
	leddev.device = device_create(leddev.class, NULL, leddev.devid, NULL, LEDDEV_NAME);
	if (IS_ERR(leddev.device)) {
		ret = PTR_ERR(leddev.device);
		goto err_destroy_class;
	}

	/* 5、初始化IO */	
	leddev.node = of_find_node_by_path("/gpioled");
	if (leddev.node == NULL)
		leddev.node = of_find_node_by_path("/leds/led1");
	if (leddev.node == NULL) {
		printk("gpioled/leds node not find!\r\n");
		ret = -EINVAL;
		goto err_destroy_device;
	}
	
	leddev.led0 = of_get_named_gpio(leddev.node, "led-gpio", 0);
	if (leddev.led0 < 0)
		leddev.led0 = of_get_named_gpio(leddev.node, "gpios", 0);
	of_node_put(leddev.node);
	leddev.node = NULL;
	if (leddev.led0 < 0) {
		printk("can't get led-gpio/gpios\r\n");
		ret = -EINVAL;
		goto err_destroy_device;
	}

	ret = gpio_request(leddev.led0, "led0");
	if (ret) {
		printk("gpio_request led0 failed: %d\r\n", ret);
		goto err_destroy_device;
	}

	ret = gpio_direction_output(leddev.led0, 1); /* led0 IO设置为输出，默认高电平	*/
	if (ret) {
		printk("gpio_direction_output led0 failed: %d\r\n", ret);
		gpio_free(leddev.led0);
		goto err_destroy_device;
	}
	return 0;

err_destroy_device:
	device_destroy(leddev.class, leddev.devid);
err_destroy_class:
	class_destroy(leddev.class);
err_del_cdev:
	cdev_del(&leddev.cdev);
err_unregister_chrdev:
	unregister_chrdev_region(leddev.devid, LEDDEV_CNT);
	return ret;
}

/*
 * @description		: platform驱动的remove函数，移除platform驱动的时候此函数会执行
 * @param - dev 	: platform设备
 * @return 			: 0，成功;其他负值,失败
 */
static int led_remove(struct platform_device *dev)
{
	if (gpio_is_valid(leddev.led0)) {
		gpio_set_value(leddev.led0, 1); 	/* 卸载驱动的时候关闭LED */
		gpio_free(leddev.led0);				/* 释放IO 			*/
	}

	cdev_del(&leddev.cdev);				/*  删除cdev */
	unregister_chrdev_region(leddev.devid, LEDDEV_CNT); /* 注销设备号 */
	device_destroy(leddev.class, leddev.devid);
	class_destroy(leddev.class);
	return 0;
}

/* 匹配列表 */
static const struct of_device_id led_of_match[] = {
	{ .compatible = "atkalpha-gpioled" },
	{ /* Sentinel */ }
};

/* platform驱动结构体 */
static struct platform_driver led_driver = {
	.driver		= {
		.name	= "imx6ul-led",			/* 驱动名字，用于和设备匹配 */
		.of_match_table	= led_of_match, /* 设备树匹配表 		 */
	},
	.probe		= led_probe,
	.remove		= led_remove,
};
		
/*
 * @description	: 驱动模块加载函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init leddriver_init(void)
{
	struct device_node *np;
	int ret;

	ret = platform_driver_register(&led_driver);
	if (ret)
		return ret;

	/*
	 * ALPHA 默认 DTS 多使用 /leds 节点，不一定存在 atkalpha-gpioled compatible。
	 * 无匹配节点时注册 fallback platform device，保证示例驱动可执行。
	 */
	np = of_find_compatible_node(NULL, NULL, "atkalpha-gpioled");
	if (!np) {
		led_fallback_pdev = platform_device_register_simple("imx6ul-led", -1, NULL, 0);
		if (IS_ERR(led_fallback_pdev)) {
			ret = PTR_ERR(led_fallback_pdev);
			led_fallback_pdev = NULL;
			platform_driver_unregister(&led_driver);
			return ret;
		}
	} else {
		of_node_put(np);
	}

	return 0;
}

/*
 * @description	: 驱动模块卸载函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit leddriver_exit(void)
{
	if (led_fallback_pdev)
		platform_device_unregister(led_fallback_pdev);
	platform_driver_unregister(&led_driver);
}

module_init(leddriver_init);
module_exit(leddriver_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zuozhongkai");



