/***************************************************************
 * ICM20608 IIO 驱动（Phase 4 迁移版）
 *
 * 底本：reference/01、例程源码/02、Linux驱动例程/27_iio/spi/icm20608.c
 *       （正点原子 Linux 团队，V1.2 2021/08/13，使用 IIO 框架）
 * 数据通路取自：imx6ull_project_src/kernel/baseline/icm20608/icm20608.c
 *       （本项目已实测调通的 ICM20608 驱动）
 *
 * 本文件相对底本的改动（原因见 plan/phase4_sensor_framework.md）：
 *   1. 删除 4 个 buffer/trigger 头（linux/iio/{buffer,trigger,
 *      triggered_buffer,trigger_consumer}.h）—— 底本本就是 INDIO_DIRECT_MODE
 *      纯 sysfs 方案，不引用 buffer/trigger；板端内核 IIO_BUFFER/IIO_TRIGGER 未开。
 *   2. 【关键】不用 regmap，改为手动 SPI 传输 + 软件片选。
 *      原因：本项目 DTS 的 ecspi3 节点用非标准属性
 *        cs-gpio = <&gpio1 20 GPIO_ACTIVE_LOW>;   （注意不是标准的 cs-gpios）
 *      且未给控制器注册 cs-gpios，控制器自带的 CS0 在这块板上并未接到片外。
 *      现场能跑通的 baseline/icm20608.c 正是靠手动拉 GPIO1_IO20 当片选，
 *      而 regmap_spi 无法在每次传输前后切 GPIO（4.1.15 也没有 cs_gpiod 支持），
 *      照搬底本 regmap 会读到 WHO_AM_I = 0x00、所有 raw 全 0。
 *      （2026-10-07 已实测：WHO_AM_I=0X0，全部 raw=0。）
 *   3. probe 顺序：先建数据通路（软件 CS + spi_setup + reginit），
 *      再 iio_device_register。底本把 iio_device_register 放在 regmap 之前，
 *      注册到通路就绪之间若有读会空指针。
 *   4. 模块/驱动/iio_dev 名改为 icm20608_iio，避免与旧 icm20608.ko
 *      及内核 BSP 驱动同名歧义；of_match 与 id_table 保持 alientek,icm20608。
 *   5. 去掉本驱动未使用的旧字符设备相关头。
 *
 * 注意：icm20608_channels[] 里 IIO_ACCEL 的轴序是 Y/X/Z，这是芯片物理轴序，
 *       不要"按字母排序"改成 X/Y/Z，否则轴向错乱。
 *
 * license: GPL（随正点原子例程）
 ***************************************************************/
#include <linux/spi/spi.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "icm20608reg.h"

#define ICM20608_NAME	"icm20608_iio"
#define ICM20608_TEMP_OFFSET	     0
#define ICM20608_TEMP_SCALE		     326800000

/* SPI 访问参数（沿用 baseline/icm20608.c，不自创） */
#define ICM20608_SPI_IO_MAX_LEN     64
#define ICM20608_SPI_RETRY_MAX      3
#define ICM20608_SPI_RETRY_DELAY_US 1200
#define ICM20608_RESET_DELAY_MS     50
#define ICM20608_SPI_MAX_SPEED_HZ   8000000

/* ecspi3 控制器节点：cs-gpio 属性挂在这里（本项目 DTS） */
#define ICM20608_ECSPI3_NODE_PATH \
	"/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02010000"

#define ICM20608_CHAN(_type, _channel2, _index)                    \
	{                                                             \
		.type = _type,                                        \
		.modified = 1,                                        \
		.channel2 = _channel2,                                \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	      \
				      BIT(IIO_CHAN_INFO_CALIBBIAS),   \
		.scan_index = _index,                                 \
		.scan_type = {                                        \
				.sign = 's',                          \
				.realbits = 16,                       \
				.storagebits = 16,                    \
				.shift = 0,                           \
				.endianness = IIO_BE,                 \
			     },                                       \
	}

/*
 * ICM20608的扫描元素，3轴加速度计、
 * 3轴陀螺仪、1路温度传感器，1路时间戳
 */
enum inv_icm20608_scan {
	INV_ICM20608_SCAN_ACCL_X,
	INV_ICM20608_SCAN_ACCL_Y,
	INV_ICM20608_SCAN_ACCL_Z,
	INV_ICM20608_SCAN_TEMP,
	INV_ICM20608_SCAN_GYRO_X,
	INV_ICM20608_SCAN_GYRO_Y,
	INV_ICM20608_SCAN_GYRO_Z,
	INV_ICM20608_SCAN_TIMESTAMP,
};

struct icm20608_dev {
	struct spi_device *spi;		/* spi设备 */
	struct mutex lock;		/* 串行化寄存器访问 */
	int cs_gpio;			/* 软件片选 GPIO（无效时为 -EINVAL） */
	bool use_sw_cs;			/* 是否使用软件片选 */
};

/*
 * icm20608陀螺仪分辨率，对应250、500、1000、2000，计算方法：
 * 以正负250度量程为例，500/2^16=0.007629，扩大1000000倍，就是7629
 */
static const int gyro_scale_icm20608[] = {7629, 15258, 30517, 61035};

/*
 * icm20608加速度计分辨率，对应2、4、8、16 计算方法：
 * 以正负2g量程为例，4/2^16=0.000061035，扩大1000000000倍，就是61035
 */
static const int accel_scale_icm20608[] = {61035, 122070, 244140, 488281};

/*
 * icm20608通道，1路温度通道，3路陀螺仪，3路加速度计
 */
static const struct iio_chan_spec icm20608_channels[] = {
	/* 温度通道 */
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
				| BIT(IIO_CHAN_INFO_OFFSET)
				| BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = INV_ICM20608_SCAN_TEMP,
		.scan_type = {
				.sign = 's',
				.realbits = 16,
				.storagebits = 16,
				.shift = 0,
				.endianness = IIO_BE,
			     },
	},

	ICM20608_CHAN(IIO_ANGL_VEL, IIO_MOD_X, INV_ICM20608_SCAN_GYRO_X),	/* 陀螺仪X轴 */
	ICM20608_CHAN(IIO_ANGL_VEL, IIO_MOD_Y, INV_ICM20608_SCAN_GYRO_Y),	/* 陀螺仪Y轴 */
	ICM20608_CHAN(IIO_ANGL_VEL, IIO_MOD_Z, INV_ICM20608_SCAN_GYRO_Z),	/* 陀螺仪Z轴 */

	ICM20608_CHAN(IIO_ACCEL, IIO_MOD_Y, INV_ICM20608_SCAN_ACCL_Y),	/* 加速度X轴 */
	ICM20608_CHAN(IIO_ACCEL, IIO_MOD_X, INV_ICM20608_SCAN_ACCL_X),	/* 加速度Y轴 */
	ICM20608_CHAN(IIO_ACCEL, IIO_MOD_Z, INV_ICM20608_SCAN_ACCL_Z),	/* 加速度Z轴 */
};

/* ================================================================
 * 软件片选 + 寄存器访问
 * （与 baseline/icm20608.c 同一套做法，缺了它在本板上读不到芯片）
 * ================================================================ */

static inline void icm20608_cs_active(struct icm20608_dev *dev)
{
	if (dev->use_sw_cs && gpio_is_valid(dev->cs_gpio))
		gpio_set_value(dev->cs_gpio, 0);
}

static inline void icm20608_cs_inactive(struct icm20608_dev *dev)
{
	if (dev->use_sw_cs && gpio_is_valid(dev->cs_gpio))
		gpio_set_value(dev->cs_gpio, 1);
}

/* 读 len 个连续寄存器；buf 存返回的 len 字节（不含地址回显字节） */
static int icm20608_read_regs(struct icm20608_dev *dev, u8 reg, void *buf, u8 len)
{
	struct spi_transfer xfer;
	u8 txbuf[ICM20608_SPI_IO_MAX_LEN + 1];
	u8 rxbuf[ICM20608_SPI_IO_MAX_LEN + 1];
	int ret;
	int retry;

	if (!dev || !dev->spi || !buf || !len || len > ICM20608_SPI_IO_MAX_LEN)
		return -EINVAL;

	memset(&xfer, 0, sizeof(xfer));
	txbuf[0] = reg | 0x80;			/* 读：寄存器最高位置 1 */
	memset(&txbuf[1], 0, len);
	memset(rxbuf, 0, sizeof(rxbuf));
	xfer.tx_buf = txbuf;
	xfer.rx_buf = rxbuf;
	xfer.len = len + 1;

	for (retry = 0; retry < ICM20608_SPI_RETRY_MAX; retry++) {
		icm20608_cs_active(dev);
		ret = spi_sync_transfer(dev->spi, &xfer, 1);
		icm20608_cs_inactive(dev);
		if (!ret) {
			memcpy(buf, rxbuf + 1, len);
			return 0;
		}
		usleep_range(ICM20608_SPI_RETRY_DELAY_US,
			     ICM20608_SPI_RETRY_DELAY_US + 300);
	}
	dev_err(&dev->spi->dev, "SPI read reg 0x%02x failed: %d\n", reg, ret);
	return ret;
}

/* 写 len 个连续寄存器 */
static int icm20608_write_regs(struct icm20608_dev *dev, u8 reg,
			       const u8 *buf, u8 len)
{
	struct spi_transfer xfer;
	u8 txbuf[ICM20608_SPI_IO_MAX_LEN + 1];
	int ret;
	int retry;

	if (!dev || !dev->spi || !buf || !len || len > ICM20608_SPI_IO_MAX_LEN)
		return -EINVAL;

	memset(&xfer, 0, sizeof(xfer));
	txbuf[0] = reg & ~0x80;			/* 写：寄存器最高位清 0 */
	memcpy(txbuf + 1, buf, len);
	xfer.tx_buf = txbuf;
	xfer.len = len + 1;

	for (retry = 0; retry < ICM20608_SPI_RETRY_MAX; retry++) {
		icm20608_cs_active(dev);
		ret = spi_sync_transfer(dev->spi, &xfer, 1);
		icm20608_cs_inactive(dev);
		if (!ret)
			return 0;
		usleep_range(ICM20608_SPI_RETRY_DELAY_US,
			     ICM20608_SPI_RETRY_DELAY_US + 300);
	}
	dev_err(&dev->spi->dev, "SPI write reg 0x%02x failed: %d\n", reg, ret);
	return ret;
}

/* 读单个寄存器；失败返回负值，成功返回值本身 */
static int icm20608_read_onereg(struct icm20608_dev *dev, u8 reg)
{
	u8 data = 0;
	int ret;

	ret = icm20608_read_regs(dev, reg, &data, 1);
	return ret ? ret : data;
}

/* 写单个寄存器 */
static int icm20608_write_onereg(struct icm20608_dev *dev, u8 reg, u8 value)
{
	return icm20608_write_regs(dev, reg, &value, 1);
}

/*
 * 软件片选初始化：cs-gpio 属性挂在本板 ecspi3 控制器节点上
 * （非标准属性名，见 imx6ull-14x14-evk.dts 的 &ecspi3）。
 */
static void icm20608_release_sw_cs(struct icm20608_dev *dev);

static int icm20608_setup_sw_cs(struct icm20608_dev *dev)
{
	struct device_node *np = NULL;
	int ret;

	dev->use_sw_cs = false;
	dev->cs_gpio = -EINVAL;

	if (dev->spi->dev.of_node) {
		np = dev->spi->dev.of_node->parent;
		if (np)
			of_node_get(np);	/* 统一持有一个引用 */
	}
	if (!np)
		np = of_find_node_by_path(ICM20608_ECSPI3_NODE_PATH);
	if (!np)
		return 0;

	dev->cs_gpio = of_get_named_gpio(np, "cs-gpio", 0);
	of_node_put(np);
	if (!gpio_is_valid(dev->cs_gpio)) {
		dev->cs_gpio = -EINVAL;
		return 0;
	}

	ret = gpio_request(dev->cs_gpio, "icm20608-cs");
	if (ret) {
		dev->cs_gpio = -EINVAL;
		return ret;
	}

	ret = gpio_direction_output(dev->cs_gpio, 1);
	if (ret) {
		gpio_free(dev->cs_gpio);
		dev->cs_gpio = -EINVAL;
		return ret;
	}

	dev->use_sw_cs = true;
	return 0;
}

static void icm20608_release_sw_cs(struct icm20608_dev *dev)
{
	if (dev->use_sw_cs && gpio_is_valid(dev->cs_gpio)) {
		gpio_set_value(dev->cs_gpio, 1);
		gpio_free(dev->cs_gpio);
	}
	dev->use_sw_cs = false;
	dev->cs_gpio = -EINVAL;
}

/*
 * @description  	: ICM20608内部寄存器初始化函数
 * @param - dev 	: icm20608设备
 * @return 			: 无
 */
static void icm20608_reginit(struct icm20608_dev *dev)
{
	int value = 0;

	icm20608_write_onereg(dev, ICM20_PWR_MGMT_1, 0x80);
	mdelay(ICM20608_RESET_DELAY_MS);
	icm20608_write_onereg(dev, ICM20_PWR_MGMT_1, 0x01);
	mdelay(ICM20608_RESET_DELAY_MS);

	value = icm20608_read_onereg(dev, ICM20_WHO_AM_I);
	printk("ICM20608 ID = %#X\r\n", value);

	icm20608_write_onereg(dev, ICM20_SMPLRT_DIV, 0x00); 	/* 输出速率是内部采样率		*/
	icm20608_write_onereg(dev, ICM20_GYRO_CONFIG, 0x18); 	/* 陀螺仪±2000dps量程 		*/
	icm20608_write_onereg(dev, ICM20_ACCEL_CONFIG, 0x18); 	/* 加速度计±16G量程 		*/
	icm20608_write_onereg(dev, ICM20_CONFIG, 0x04); 		/* 陀螺仪低通滤波BW=20Hz 	*/
	icm20608_write_onereg(dev, ICM20_ACCEL_CONFIG2, 0x04); /* 加速度计低通滤波BW=21.2Hz 	*/
	icm20608_write_onereg(dev, ICM20_PWR_MGMT_2, 0x00); 	/* 打开加速度计和陀螺仪所有轴 	*/
	icm20608_write_onereg(dev, ICM20_LP_MODE_CFG, 0x00); 	/* 关闭低功耗 				*/
	/* 0x01 = bit0 DATA_RDY_EN，只使能"数据就绪"中断（bit4 FIFO_OFLOW_EN 未置位）。
	 * 本驱动不注册 request_irq，该中断只在芯片内标志位可见。 */
	icm20608_write_onereg(dev, ICM20_INT_ENABLE, 0x01);
}

/*
  * @description  	: 设置ICM20608传感器，可以用于陀螺仪、加速度计设置
  * @param - dev	: icm20608设备
  * @param - reg  	: 要设置的通道寄存器首地址。
  * @param - anix  	: 要设置的通道，比如X，Y，Z。
  * @param - val   	: 要设置的值。
  * @return			: 0，成功；其他值，错误
  */
static int icm20608_sensor_set(struct icm20608_dev *dev, int reg,
				int axis, int val)
{
	u8 buf[2];
	int ind;

	ind = (axis - IIO_MOD_X) * 2;
	buf[0] = (val >> 8) & 0xFF;	/* 大端：高字节在前 */
	buf[1] = val & 0xFF;
	return icm20608_write_regs(dev, reg + ind, buf, 2);
}

/*
  * @description  	: 读取ICM20608传感器数据，可以用于陀螺仪、加速度计、温度的读取
  * @param - dev	: icm20608设备
  * @param - reg  	: 要读取的通道寄存器首地址。
  * @param - anix  	: 需要读取的通道，比如X，Y，Z。
  * @param - val   	: 保存读取到的值。
  * @return			: 0，成功；其他值，错误
  */
static int icm20608_sensor_show(struct icm20608_dev *dev, int reg,
				   int axis, int *val)
{
	u8 buf[2];
	int ind;
	int ret;

	ind = (axis - IIO_MOD_X) * 2;
	ret = icm20608_read_regs(dev, reg + ind, buf, 2);
	if (ret)
		return ret;
	*val = (short)(((u16)buf[0] << 8) | buf[1]);

	return IIO_VAL_INT;
}

/*
  * @description  		: 读取ICM20608陀螺仪、加速度计、温度通道值
  * @param - indio_dev	: iio设备
  * @param - chan  		: 通道。
  * @param - val   		: 保存读取到的通道值。
  * @return				: 0，成功；其他值，错误
  */
static int icm20608_read_channel_data(struct iio_dev *indio_dev,
					 struct iio_chan_spec const *chan,
					 int *val)
{
	struct icm20608_dev *dev = iio_priv(indio_dev);
	int ret = 0;

	switch (chan->type) {
	case IIO_ANGL_VEL:	/* 读取陀螺仪数据 */
		ret = icm20608_sensor_show(dev, ICM20_GYRO_XOUT_H, chan->channel2, val);  /* channel2为X、Y、Z轴 */
		break;
	case IIO_ACCEL:		/* 读取加速度计数据 */
		ret = icm20608_sensor_show(dev, ICM20_ACCEL_XOUT_H, chan->channel2, val); /* channel2为X、Y、Z轴 */
		break;
	case IIO_TEMP:		/* 读取温度 */
		ret = icm20608_sensor_show(dev, ICM20_TEMP_OUT_H, IIO_MOD_X, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

/*
  * @description  	: 设置ICM20608的陀螺仪计量程(分辨率)
  * @param - dev	: icm20608设备
  * @param - val   	: 量程(分辨率值)。
  * @return			: 0，成功；其他值，错误
  */
static int icm20608_write_gyro_scale(struct icm20608_dev *dev, int val)
{
	int i;
	u8 d;

	for (i = 0; i < ARRAY_SIZE(gyro_scale_icm20608); ++i) {
		if (gyro_scale_icm20608[i] == val) {
			d = (i << 3);
			return icm20608_write_onereg(dev, ICM20_GYRO_CONFIG, d);
		}
	}
	return -EINVAL;
}

 /*
  * @description  	: 设置ICM20608的加速度计量程(分辨率)
  * @param - dev	: icm20608设备
  * @param - val   	: 量程(分辨率值)。
  * @return			: 0，成功；其他值，错误
  */
static int icm20608_write_accel_scale(struct icm20608_dev *dev, int val)
{
	int i;
	u8 d;

	for (i = 0; i < ARRAY_SIZE(accel_scale_icm20608); ++i) {
		if (accel_scale_icm20608[i] == val) {
			d = (i << 3);
			return icm20608_write_onereg(dev, ICM20_ACCEL_CONFIG, d);
		}
	}
	return -EINVAL;
}

/*
  * @description     	: 读函数，当读取sysfs中的文件的时候最终此函数会执行，此函数
  * 					：里面会从传感器里面读取各种数据，然后上传给应用。
  * @param - indio_dev	: iio_dev
  * @param - chan   	: 通道
  * @param - val   		: 读取的值，如果是小数值的话，val是整数部分。
  * @param - val2   	: 读取的值，如果是小数值的话，val2是小数部分。
  * @param - mask   	: 掩码。
  * @return				: 0，成功；其他值，错误
  */
static int icm20608_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct icm20608_dev *dev = iio_priv(indio_dev);
	int ret = 0;
	int regdata = 0;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:								/* 读取ICM20608加速度计、陀螺仪、温度传感器原始值 */
		mutex_lock(&dev->lock);								/* 上锁 			*/
		ret = icm20608_read_channel_data(indio_dev, chan, val); 	/* 读取通道值 */
		mutex_unlock(&dev->lock);							/* 释放锁 			*/
		return ret;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			mutex_lock(&dev->lock);
			regdata = icm20608_read_onereg(dev, ICM20_GYRO_CONFIG);
			mutex_unlock(&dev->lock);
			if (regdata < 0)
				return regdata;
			regdata = (regdata & 0X18) >> 3;
			*val  = 0;
			*val2 = gyro_scale_icm20608[regdata];
			return IIO_VAL_INT_PLUS_MICRO;	/* 值为val+val2/1000000 */
		case IIO_ACCEL:
			mutex_lock(&dev->lock);
			regdata = icm20608_read_onereg(dev, ICM20_ACCEL_CONFIG);
			mutex_unlock(&dev->lock);
			if (regdata < 0)
				return regdata;
			regdata = (regdata & 0X18) >> 3;
			*val = 0;
			*val2 = accel_scale_icm20608[regdata];
			return IIO_VAL_INT_PLUS_NANO;/* 值为val+val2/1000000000 */
		case IIO_TEMP:
			*val = ICM20608_TEMP_SCALE/ 1000000;
			*val2 = ICM20608_TEMP_SCALE % 1000000;
			return IIO_VAL_INT_PLUS_MICRO;	/* 值为val+val2/1000000 */
		default:
			return -EINVAL;
		}
		return ret;
	case IIO_CHAN_INFO_OFFSET:		/* ICM20608温度传感器offset值 */
		switch (chan->type) {
		case IIO_TEMP:
			*val = ICM20608_TEMP_OFFSET;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
		return ret;
	case IIO_CHAN_INFO_CALIBBIAS:	/* ICM20608加速度计和陀螺仪校准值 */
		switch (chan->type) {
		case IIO_ANGL_VEL:		/* 陀螺仪的校准值 */
			mutex_lock(&dev->lock);
			ret = icm20608_sensor_show(dev, ICM20_XG_OFFS_USRH, chan->channel2, val);
			mutex_unlock(&dev->lock);
			return ret;
		case IIO_ACCEL:			/* 加速度计的校准值 */
			mutex_lock(&dev->lock);
			ret = icm20608_sensor_show(dev, ICM20_XA_OFFSET_H, chan->channel2, val);
			mutex_unlock(&dev->lock);
			return ret;
		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

/*
  * @description     	: 写函数，当向sysfs中的文件写数据的时候最终此函数会执行，一般在此函数
  * 					：里面设置传感器，比如量程等。
  * @param - indio_dev	: iio_dev
  * @param - chan   	: 通道
  * @param - val   		: 应用程序写入的值，如果是小数值的话，val是整数部分。
  * @param - val2   	: 应用程序写入的值，如果是小数值的话，val2是小数部分。
  * @return				: 0，成功；其他值，错误
  */
static int icm20608_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct icm20608_dev *dev = iio_priv(indio_dev);
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:	/* 设置陀螺仪和加速度计的分辨率 */
		switch (chan->type) {
		case IIO_ANGL_VEL:		/* 设置陀螺仪 */
			mutex_lock(&dev->lock);
			ret = icm20608_write_gyro_scale(dev, val2);
			mutex_unlock(&dev->lock);
			break;
		case IIO_ACCEL:			/* 设置加速度计 */
			mutex_lock(&dev->lock);
			ret = icm20608_write_accel_scale(dev, val2);
			mutex_unlock(&dev->lock);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case IIO_CHAN_INFO_CALIBBIAS:	/* 设置陀螺仪和加速度计的校准值*/
		switch (chan->type) {
		case IIO_ANGL_VEL:		/* 设置陀螺仪校准值 */
			mutex_lock(&dev->lock);
			ret = icm20608_sensor_set(dev, ICM20_XG_OFFS_USRH,
									    chan->channel2, val);
			mutex_unlock(&dev->lock);
			break;
		case IIO_ACCEL:			/* 加速度计校准值 */
			mutex_lock(&dev->lock);
			ret = icm20608_sensor_set(dev, ICM20_XA_OFFSET_H,
										    chan->channel2, val);
			mutex_unlock(&dev->lock);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

/*
  * @description     	: 用户空间写数据格式，比如我们在用户空间操作sysfs来设置传感器的分辨率，
  * 					：如果分辨率带小数，那么这个小数传递到内核空间应该扩大多少倍，此函数就是
  *						: 用来设置这个的。
  * @param - indio_dev	: iio_dev
  * @param - chan   	: 通道
  * @param - mask   	: 掩码
  * @return				: 0，成功；其他值，错误
  */
static int icm20608_write_raw_get_fmt(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:		/* 用户空间写的陀螺仪分辨率数据要乘以1000000 */
			return IIO_VAL_INT_PLUS_MICRO;
		default:				/* 用户空间写的加速度计分辨率数据要乘以1000000000 */
			return IIO_VAL_INT_PLUS_NANO;
		}
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}

/*
 * iio_info结构体变量
 */
static const struct iio_info icm20608_info = {
	.read_raw		= icm20608_read_raw,
	.write_raw		= icm20608_write_raw,
	.write_raw_get_fmt = &icm20608_write_raw_get_fmt,	/* 用户空间写数据格式 */
};

/*
  * @description    : spi驱动的probe函数，当驱动与
  *                    设备匹配以后此函数就会执行
  * @param - spi  	: spi设备
  * @return  		: 0,成功；其他值，失败
  */
static int icm20608_probe(struct spi_device *spi)
{
	int ret;
	struct icm20608_dev *dev;
	struct iio_dev *indio_dev;

	/* 1、申请iio_dev内存 */
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	/* 2、获取icm20608_dev结构体地址 */
	dev = iio_priv(indio_dev);
	dev->spi = spi;
	dev->cs_gpio = -EINVAL;
	dev->use_sw_cs = false;
	spi_set_drvdata(spi, indio_dev);    		/* 将indio_dev设置为spi->dev的driver_data */
	mutex_init(&dev->lock);

	/* 3、iio_dev的其他成员变量 */
	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &icm20608_info;
	indio_dev->name = ICM20608_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;	/* 直接模式，提供sysfs接口 */
	indio_dev->channels = icm20608_channels;
	indio_dev->num_channels = ARRAY_SIZE(icm20608_channels);

	/* 4、软件片选。本板控制器 CS 未接到片外，必须手动拉 GPIO1_IO20 */
	ret = icm20608_setup_sw_cs(dev);
	if (ret) {
		dev_err(&spi->dev, "setup sw cs failed: %d\n", ret);
		return ret;
	}
	if (!dev->use_sw_cs)
		dev_warn(&spi->dev,
			 "no cs-gpio found; falling back to controller CS (may read 0)\n");

	/* 5、SPI 参数（沿用 baseline/icm20608.c） */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->max_speed_hz = ICM20608_SPI_MAX_SPEED_HZ;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "spi_setup failed\n");
		goto err_release_swcs;
	}

	/* 6、初始化ICM20608内部寄存器（此时数据通路已就绪） */
	icm20608_reginit(dev);

	/* 7、最后注册iio_dev，对外暴露sysfs接口 */
	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&spi->dev, "iio_device_register failed\n");
		goto err_release_swcs;
	}

	dev_info(&spi->dev, "probed (sw_cs=%d cs_gpio=%d)\n",
		 dev->use_sw_cs ? 1 : 0, dev->cs_gpio);

	return 0;

err_release_swcs:
	icm20608_release_sw_cs(dev);
	return ret;
}

/*
 * @description     : spi驱动的remove函数，移除spi驱动的时候此函数会执行
 * @param - spi 	: spi设备
 * @return          : 0，成功;其他负值,失败
 */
static int icm20608_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct icm20608_dev *dev;

	dev = iio_priv(indio_dev);

	/* 1、注销IIO */
	iio_device_unregister(indio_dev);

	/* 2、释放软件片选 */
	icm20608_release_sw_cs(dev);

	return 0;
}

/* 传统匹配方式ID列表 */
static const struct spi_device_id icm20608_id[] = {
	{"alientek,icm20608", 0},
	{}
};

/* 设备树匹配列表 */
static const struct of_device_id icm20608_of_match[] = {
	{ .compatible = "alientek,icm20608" },
	{ /* Sentinel */ }
};

/* SPI驱动结构体 */
static struct spi_driver icm20608_driver = {
	.probe = icm20608_probe,
	.remove = icm20608_remove,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "icm20608_iio",
		   	.of_match_table = icm20608_of_match,
		   },
	.id_table = icm20608_id,
};

/*
 * @description	: 驱动入口函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init icm20608_init(void)
{
	return spi_register_driver(&icm20608_driver);
}

/*
 * @description	: 驱动出口函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit icm20608_exit(void)
{
	spi_unregister_driver(&icm20608_driver);
}

module_init(icm20608_init);
module_exit(icm20608_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("ALIENTEK (IIO example), adapted for IMX6ULL Phase 4");
MODULE_DESCRIPTION("ICM20608 IIO driver (INDIO_DIRECT_MODE, sysfs raw, software CS)");
