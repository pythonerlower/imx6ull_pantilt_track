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
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "ap3216creg.h"
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: ap3216c.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: AP3216C驱动程序
其他	   	: 无
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/9/2 左忠凯创建
***************************************************************/
#define AP3216C_CNT	1
#define AP3216C_NAME	"ap3216c_ref"
/* Group-2 single-variable experiment:
 * 0: i2c_transfer path
 * 1: i2c_smbus path
 */
#define AP3216C_REF_IO_API_SMBUS 1
/* Group-3 single-variable experiment: reset settle delay after soft reset. */
#define AP3216C_REF_RESET_DELAY_MS 20

/* Group-4: IRQ + Input subsystem */
#define AP3216C_REF_IRQ_ENABLED 1

struct ap3216c_dev {
	dev_t devid;			/* 设备号 	 */
	struct cdev cdev;		/* cdev 	*/
	struct class *class;	/* 类 		*/
	struct device *device;	/* 设备 	 */
	struct device_node	*nd; /* 设备节点 */
	int major;			/* 主设备号 */
	void *private_data;	/* 私有数据 */
	unsigned short ir, als, ps;		/* 三个光传感器数据 */

#if AP3216C_REF_IRQ_ENABLED
	struct input_dev *input;           /* Input 设备       */
	int irq;                          /* 中断号           */
	struct work_struct work;           /* 底半部 work     */
	struct workqueue_struct *wq;       /* 专用工作队列    */
	int proximity_near;               /* 接近状态        */
	atomic_t irq_cnt;                 /* 中断计数        */
	struct mutex lock;                 /* 互斥锁          */
#endif
};

static struct ap3216c_dev ap3216cdev;

/*
 * @description	: 从ap3216c读取多个寄存器数据
 * @param - dev:  ap3216c设备
 * @param - reg:  要读取的寄存器首地址
 * @param - val:  读取到的数据
 * @param - len:  要读取的数据长度
 * @return 		: 操作结果
 */
static int ap3216c_read_regs(struct ap3216c_dev *dev, u8 reg, void *val, int len)
{
	int ret;
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
	u8 *p = val;
	int i;

#if AP3216C_REF_IO_API_SMBUS
	for (i = 0; i < len; i++) {
		ret = i2c_smbus_read_byte_data(client, reg + i);
		if (ret < 0) {
			printk("smbus rd failed=%d reg=%02x len=%d idx=%d\n",
			       ret, reg, len, i);
			return ret;
		}
		p[i] = (u8)ret;
	}
	return 0;
#else
	struct i2c_msg msg[2];

	/* msg[0]为发送要读取的首地址 */
	msg[0].addr = client->addr;			/* ap3216c地址 */
	msg[0].flags = 0;					/* 标记为发送数据 */
	msg[0].buf = &reg;					/* 读取的首地址 */
	msg[0].len = 1;						/* reg长度*/

	/* msg[1]读取数据 */
	msg[1].addr = client->addr;			/* ap3216c地址 */
	msg[1].flags = I2C_M_RD;			/* 标记为读取数据*/
	msg[1].buf = val;					/* 读取数据缓冲区 */
	msg[1].len = len;					/* 要读取的数据长度*/

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret == 2) {
		ret = 0;
	} else {
		printk("i2c rd failed=%d reg=%06x len=%d\n",ret, reg, len);
		ret = -EREMOTEIO;
	}
	return ret;
#endif
}

/*
 * @description	: 向ap3216c多个寄存器写入数据
 * @param - dev:  ap3216c设备
 * @param - reg:  要写入的寄存器首地址
 * @param - val:  要写入的数据缓冲区
 * @param - len:  要写入的数据长度
 * @return 	  :   操作结果
 */
static s32 ap3216c_write_regs(struct ap3216c_dev *dev, u8 reg, u8 *buf, u8 len)
{
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
	int ret = 0;
	int i;

#if AP3216C_REF_IO_API_SMBUS
	for (i = 0; i < len; i++) {
		ret = i2c_smbus_write_byte_data(client, reg + i, buf[i]);
		if (ret < 0) {
			printk("smbus wr failed=%d reg=%02x len=%d idx=%d\n",
			       ret, reg, len, i);
			return ret;
		}
	}
	return 0;
#else
	u8 b[256];
	struct i2c_msg msg;
	
	b[0] = reg;					/* 寄存器首地址 */
	memcpy(&b[1],buf,len);		/* 将要写入的数据拷贝到数组b里面 */
		
	msg.addr = client->addr;	/* ap3216c地址 */
	msg.flags = 0;				/* 标记为写数据 */

	msg.buf = b;				/* 要写入的数据缓冲区 */
	msg.len = len + 1;			/* 要写入的数据长度 */

	return i2c_transfer(client->adapter, &msg, 1);
#endif
}

/*
 * @description	: 读取ap3216c指定寄存器值，读取一个寄存器
 * @param - dev:  ap3216c设备
 * @param - reg:  要读取的寄存器
 * @return 	  :   读取到的寄存器值
 */
static unsigned char ap3216c_read_reg(struct ap3216c_dev *dev, u8 reg)
{
	u8 data = 0;

	ap3216c_read_regs(dev, reg, &data, 1);
	return data;
}

/*
 * @description	: 向ap3216c指定寄存器写入指定的值，写一个寄存器
 * @param - dev:  ap3216c设备
 * @param - reg:  要写的寄存器
 * @param - data: 要写入的值
 * @return   :    无
 */
static void ap3216c_write_reg(struct ap3216c_dev *dev, u8 reg, u8 data)
{
	u8 buf = 0;
	buf = data;
	ap3216c_write_regs(dev, reg, &buf, 1);
}

/*
 * @description	: 读取AP3216C的数据，读取原始数据，包括ALS,PS和IR, 注意！
 *				: 如果同时打开ALS,IR+PS的话两次数据读取的时间间隔要大于112.5ms
 * @param - ir	: ir数据
 * @param - ps 	: ps数据
 * @param - als : als数据 
 * @return 		: 无。
 */
void ap3216c_readdata(struct ap3216c_dev *dev)
{
	unsigned char i = 0;
	unsigned char buf[6];

	for (i = 0; i < 6; i++)
		buf[i] = ap3216c_read_reg(dev, AP3216C_IRDATALOW + i);

	if(buf[0] & 0X80)
		dev->ir = 0;
	else
		dev->ir = ((unsigned short)buf[1] << 2) | (buf[0] & 0X03);

	dev->als = ((unsigned short)buf[3] << 8) | buf[2];

	if(buf[4] & 0x40)
		dev->ps = 0;
	else
		dev->ps = ((unsigned short)(buf[5] & 0X3F) << 4) | (buf[4] & 0X0F);
}

#if AP3216C_REF_IRQ_ENABLED
/*
 * @description	: 设置 PS 阈值寄存器
 * @param - dev:  ap3216c设备
 * @return 		: 0 成功, 负值失败
 */
static int ap3216c_set_ps_threshold(struct ap3216c_dev *dev)
{
	u8 lthd, hthd;

	/* PS 阈值寄存器是低2位+高8位格式 */
	lthd = PS_THRESHOLD_NEAR & 0xFF;
	ap3216c_write_reg(dev, AP3216C_PSLTHDL, lthd);
	ap3216c_write_reg(dev, AP3216C_PSLTHDH, (PS_THRESHOLD_NEAR >> 8) & 0x03);

	hthd = PS_THRESHOLD_FAR & 0xFF;
	ap3216c_write_reg(dev, AP3216C_PSHTHDL, hthd);
	ap3216c_write_reg(dev, AP3216C_PSHTHDH, (PS_THRESHOLD_FAR >> 8) & 0x03);

	return 0;
}

/*
 * @description	: 初始化 Input 设备
 * @param - dev:  ap3216c设备
 * @return 		: 0 成功, 负值失败
 */
static int ap3216c_input_dev_init(struct ap3216c_dev *dev)
{
	int err;

	dev->input = input_allocate_device();
	if (!dev->input) {
		dev_err(dev->device ?: ap3216cdev.device,
			"input_allocate_device failed\n");
		return -ENOMEM;
	}

	dev->input->name = "ap3216c_proximity";
	dev->input->phys = "ap3216c/input0";
	dev->input->id.bustype = BUS_I2C;
	dev->input->id.vendor = 0x1234;
	dev->input->id.product = 0x5678;
	dev->input->id.version = 0x0100;

	__set_bit(EV_SW, dev->input->evbit);
	__set_bit(SW_FRONT_PROXIMITY, dev->input->swbit);

	err = input_register_device(dev->input);
	if (err) {
		dev_err(dev->device ?: ap3216cdev.device,
			"input_register_device failed: %d\n", err);
		input_free_device(dev->input);
		dev->input = NULL;
		return err;
	}

	return 0;
}

/*
 * @description	: 配置并使能 PS 中断
 * @param - dev:  ap3216c设备
 * @return 		: 0 成功, 负值失败
 */
static int ap3216c_ps_enable_irq(struct ap3216c_dev *dev)
{
	/* 设置 PS 阈值 */
	ap3216c_set_ps_threshold(dev);

	/* 配置 PS 中断使能: bit0=1 开启 PS 中断 */
	ap3216c_write_reg(dev, AP3216C_PSCONFIG, PS_CONFIG_DEFAULT);

	/* 配置 LED 驱动电流 */
	ap3216c_write_reg(dev, AP3216C_PSLEDCONFIG, PS_LED_CURRENT_DEFAULT);

	return 0;
}

/*
 * @description	: 带中断的硬件初始化
 * @param - dev:  ap3216c设备
 * @return 		: 0 成功, 负值失败
 */
static int ap3216c_hw_init_irq(struct ap3216c_dev *dev)
{
	/* 软复位 */
	ap3216c_write_reg(dev, AP3216C_SYSTEMCONG, 0x04);
	mdelay(AP3216C_REF_RESET_DELAY_MS);

	/* 配置 PS 中断 */
	ap3216c_ps_enable_irq(dev);

	/* 切换到 PS+ALS 模式 (IR 保持关闭) */
	ap3216c_write_reg(dev, AP3216C_SYSTEMCONG, 0x03);

	dev_info(dev->device ?: ap3216cdev.device,
		 "hw_init_irq: PS thresholds near=%d far=%d\n",
		 PS_THRESHOLD_NEAR, PS_THRESHOLD_FAR);

	return 0;
}

/*
 * @description	: 中断底半部工作回调
 * @param - work: work 结构
 * @return 		: 无
 */
static void ap3216c_work_callback(struct work_struct *work)
{
	struct ap3216c_dev *dev = container_of(work, struct ap3216c_dev, work);
	unsigned char int_status;
	unsigned char ps_data_low, ps_data_high;
	unsigned short ps_raw;
	int proximity_near;

	mutex_lock(&dev->lock);

	/* 读取中断状态寄存器 */
	int_status = ap3216c_read_reg(dev, AP3216C_INTSTATUS);

	/* 读取 PS 数据 */
	ps_data_low = ap3216c_read_reg(dev, AP3216C_PSDATALOW);
	ps_data_high = ap3216c_read_reg(dev, AP3216C_PSDATAHIGH);

	/* PS 数据解析 */
	if (ps_data_low & 0x40)
		ps_raw = 0;
	else
		ps_raw = ((unsigned short)(ps_data_high & 0x3F) << 4) | (ps_data_low & 0x0F);

	dev->ps = ps_raw;

	/* 判断近/远状态: PS 值越小越近 */
	if (ps_raw < PS_THRESHOLD_NEAR)
		proximity_near = 1;
	else if (ps_raw > PS_THRESHOLD_FAR)
		proximity_near = 0;
	else
		proximity_near = dev->proximity_near;  /* 中间区域保持原状态 */

	/* 状态变化时上报 Input 事件 */
	if (proximity_near != dev->proximity_near) {
		dev->proximity_near = proximity_near;
		input_report_switch(dev->input, SW_FRONT_PROXIMITY,
				   proximity_near ? 1 : 0);
		input_sync(dev->input);
		dev_info(dev->device ?: ap3216cdev.device,
			 "proximity: %s (ps=%d)\n",
			 proximity_near ? "NEAR" : "FAR", ps_raw);
	}

	/* 清除中断 (写任意值) */
	ap3216c_write_reg(dev, AP3216C_INTCLEAR, 0x01);

	mutex_unlock(&dev->lock);
}

/*
 * @description	: 中断顶半部处理
 * @param - irq:  中断号
 * @param - dev:  设备结构
 * @return 		: irqreturn_t
 */
static irqreturn_t ap3216c_irq_handler(int irq, void *dev_id)
{
	struct ap3216c_dev *dev = dev_id;

	atomic_inc(&dev->irq_cnt);

	/* 只调度底半部，不在顶半部做 I2C 操作 */
	if (dev->wq)
		schedule_work(&dev->work);

	return IRQ_HANDLED;
}
#endif

static int ap3216c_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &ap3216cdev;

#if AP3216C_REF_IRQ_ENABLED
	ap3216c_hw_init_irq(&ap3216cdev);
#else
	ap3216c_write_reg(&ap3216cdev, AP3216C_SYSTEMCONG, 0x04);
	mdelay(AP3216C_REF_RESET_DELAY_MS);
	ap3216c_write_reg(&ap3216cdev, AP3216C_SYSTEMCONG, 0X03);
#endif
	return 0;
}

static ssize_t ap3216c_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	short data[3];
	long err = 0;
	struct ap3216c_dev *dev = (struct ap3216c_dev *)filp->private_data;

	ap3216c_readdata(dev);

	data[0] = dev->ir;
	data[1] = dev->als;
	data[2] = dev->ps;
	err = copy_to_user(buf, data, sizeof(data));
	return 0;
}

static int ap3216c_release(struct inode *inode, struct file *filp)
{
#if AP3216C_REF_IRQ_ENABLED
	struct ap3216c_dev *dev = &ap3216cdev;
	if (dev->irq > 0) {
		disable_irq(dev->irq);
		free_irq(dev->irq, dev);
		dev->irq = 0;
	}
#endif
	return 0;
}

static const struct file_operations ap3216c_ops = {
	.owner = THIS_MODULE,
	.open = ap3216c_open,
	.read = ap3216c_read,
	.release = ap3216c_release,
};

static int ap3216c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	dev_info(&client->dev, "ap3216c_ref probe: io_api=%s reset_delay_ms=%d irq_enabled=%d\n",
#if AP3216C_REF_IO_API_SMBUS
		 "smbus", AP3216C_REF_RESET_DELAY_MS,
#else
		 "i2c_transfer", AP3216C_REF_RESET_DELAY_MS,
#endif
#if AP3216C_REF_IRQ_ENABLED
		 1
#else
		 0
#endif
	);

	if (ap3216cdev.major) {
		ap3216cdev.devid = MKDEV(ap3216cdev.major, 0);
		register_chrdev_region(ap3216cdev.devid, AP3216C_CNT, AP3216C_NAME);
	} else {
		alloc_chrdev_region(&ap3216cdev.devid, 0, AP3216C_CNT, AP3216C_NAME);
		ap3216cdev.major = MAJOR(ap3216cdev.devid);
	}

	cdev_init(&ap3216cdev.cdev, &ap3216c_ops);
	cdev_add(&ap3216cdev.cdev, ap3216cdev.devid, AP3216C_CNT);

	ap3216cdev.class = class_create(THIS_MODULE, AP3216C_NAME);
	if (IS_ERR(ap3216cdev.class)) {
		return PTR_ERR(ap3216cdev.class);
	}

	ap3216cdev.device = device_create(ap3216cdev.class, NULL,
					 ap3216cdev.devid, NULL, AP3216C_NAME);
	if (IS_ERR(ap3216cdev.device)) {
		return PTR_ERR(ap3216cdev.device);
	}

	ap3216cdev.private_data = client;

#if AP3216C_REF_IRQ_ENABLED
	/* 初始化 IRQ 相关字段 */
	atomic_set(&ap3216cdev.irq_cnt, 0);
	ap3216cdev.proximity_near = 0;
	mutex_init(&ap3216cdev.lock);
	INIT_WORK(&ap3216cdev.work, ap3216c_work_callback);

	/* 创建工作队列 */
	ap3216cdev.wq = create_singlethread_workqueue("ap3216c_wq");
	if (!ap3216cdev.wq) {
		dev_err(&client->dev, "create_workqueue failed\n");
		return -ENOMEM;
	}

	/* 初始化 Input 设备 */
	ap3216cdev.input = NULL;
	ap3216c_input_dev_init(&ap3216cdev);

	/* 获取中断号 */
	ap3216cdev.irq = client->irq;
	if (ap3216cdev.irq <= 0) {
		dev_warn(&client->dev, "no IRQ from DT, irq=%d\n",
			 ap3216cdev.irq);
	} else {
		int err;
		err = request_irq(ap3216cdev.irq, ap3216c_irq_handler,
				  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				  "ap3216c_irq", &ap3216cdev);
		if (err) {
			dev_err(&client->dev, "request_irq failed: %d\n", err);
			ap3216cdev.irq = 0;
		} else {
			dev_info(&client->dev, "IRQ requested: %d\n",
				 ap3216cdev.irq);
		}
	}
#endif

	return 0;
}

static int ap3216c_remove(struct i2c_client *client)
{
#if AP3216C_REF_IRQ_ENABLED
	if (ap3216cdev.irq > 0) {
		free_irq(ap3216cdev.irq, &ap3216cdev);
		ap3216cdev.irq = 0;
	}
	if (ap3216cdev.wq) {
		flush_workqueue(ap3216cdev.wq);
		destroy_workqueue(ap3216cdev.wq);
		ap3216cdev.wq = NULL;
	}
	if (ap3216cdev.input) {
		input_unregister_device(ap3216cdev.input);
		input_free_device(ap3216cdev.input);
		ap3216cdev.input = NULL;
	}
	mutex_destroy(&ap3216cdev.lock);
#endif

	cdev_del(&ap3216cdev.cdev);
	unregister_chrdev_region(ap3216cdev.devid, AP3216C_CNT);
	device_destroy(ap3216cdev.class, ap3216cdev.devid);
	class_destroy(ap3216cdev.class);
	return 0;
}

static const struct i2c_device_id ap3216c_id[] = {
	{"alientek,ap3216c", 0},
	{"ap3216c_ref", 0},
	{"ap3216c", 0},
	{}
};

static const struct of_device_id ap3216c_of_match[] = {
	{ .compatible = "alientek,ap3216c" },
	{ .compatible = "ap3216c" },
	{ }
};

static struct i2c_driver ap3216c_driver = {
	.probe = ap3216c_probe,
	.remove = ap3216c_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "ap3216c_ref",
		.of_match_table = ap3216c_of_match,
	},
	.id_table = ap3216c_id,
};

static int __init ap3216c_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&ap3216c_driver);
	return ret;
}

static void __exit ap3216c_exit(void)
{
	i2c_del_driver(&ap3216c_driver);
}

module_init(ap3216c_init);
module_exit(ap3216c_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zuozhongkai");
