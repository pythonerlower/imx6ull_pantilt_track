#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>

#include "icm20608reg.h"

/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名      : icm20608.c
作者        : 左忠凯
版本        : V1.0
描述        : ICM20608 SPI驱动程序（WoM/FIFO阶段一增强版）
其他        : 无
论坛        : www.openedv.com
日志        : 初版V1.0 2019/9/2 左忠凯创建
***************************************************************/
#define ICM20608_CNT                     1
#define ICM20608_NAME                    "icm20608"
#define ICM20608_SPI_IO_MAX_LEN          64
#define ICM20608_SPI_RETRY_MAX           3
#define ICM20608_SPI_RETRY_DELAY_US      1200
#define ICM20608_RESET_DELAY_MS          50

#define ICM20608_IRQ_LABEL               "icm20608-wom"
#define ICM20608_ECSPI3_NODE_PATH        "/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02010000"

#define ICM20608_IRQ_GPIO_FALLBACK       10  /* GPIO1_IO10 */

#define ICM20608_WOM_THRESHOLD_DEFAULT   32
#define ICM20608_ACTIVE_WINDOW_MS_DEF    5000
#define ICM20608_ACTIVE_POLL_MS_DEF      50
#define ICM20608_LP_CLK_SEL_DEF          0x00

#define ICM20608_FIFO_SAMPLE_SIZE        12
#define ICM20608_FIFO_EN_ACTIVE          0x78

#define ICM20608_INT_PIN_CFG_DEFAULT     0xA0
#define ICM20608_INT_ENABLE_WOM          0xE0
#define ICM20608_INT_ENABLE_FIFO         0x10

#define ICM20608_ACCEL_INTEL_CTRL_WOM    0xC0

#define ICM20608_PWR_MGMT_1_CLKSEL_AUTO  0x01
#define ICM20608_PWR_MGMT_1_ACCEL_CYCLE  0x20
#define ICM20608_PWR_MGMT_2_GYRO_STBY    0x07

#define ICM20608_USER_CTRL_FIFO_EN       0x40
#define ICM20608_USER_CTRL_FIFO_RST      0x04

#define ICM20608_INT_STATUS_WOM_MASK     0xE0
#define ICM20608_INT_STATUS_FIFO_OVF     0x10

enum icm20608_state {
	ICM20608_ST_RESET = 0,
	ICM20608_ST_GUARD,
	ICM20608_ST_ACTIVE,
	ICM20608_ST_COOLDOWN,
	ICM20608_ST_ERROR,
	ICM20608_ST_SUSPEND,
};

struct icm20608_dev {
	dev_t devid;
	struct cdev cdev;
	struct class *class;
	struct device *device;
	struct spi_device *spi;
	struct mutex lock;
	struct work_struct irq_work;
	struct delayed_work sample_work;
	struct delayed_work cooldown_work;
	int major;
	int cs_gpio;
	int irq_gpio;
	int irq;
	bool use_sw_cs;
	bool own_irq_gpio;
	bool irq_enabled;
	bool hw_ready;
	u8 whoami;
	u8 wom_threshold;
	u8 lp_clk_sel;
	u32 active_window_ms;
	u32 active_poll_ms;
	enum icm20608_state state;
	signed int gyro_x_adc;
	signed int gyro_y_adc;
	signed int gyro_z_adc;
	signed int accel_x_adc;
	signed int accel_y_adc;
	signed int accel_z_adc;
	signed int temp_adc;
	u32 read_err;
	u32 retry_cnt;
	u32 irq_cnt;
	u32 wom_irq_cnt;
	u32 fifo_overflow;
	u32 batch_size_last;
	u32 batch_size_avg;
	u32 batch_stat_samples;
	u32 guard_to_active_cnt;
	u32 active_to_guard_cnt;
	u32 recover_cnt;
	unsigned long last_transition_jiffies;
	int last_err;
};

static struct icm20608_dev icm20608dev;

#define icm20608_err(dev, fmt, ...) \
	dev_err((dev)->device, "icm20608: " fmt, ##__VA_ARGS__)
#define icm20608_warn(dev, fmt, ...) \
	dev_warn((dev)->device, "icm20608: " fmt, ##__VA_ARGS__)
#define icm20608_info(dev, fmt, ...) \
	dev_info((dev)->device, "icm20608: " fmt, ##__VA_ARGS__)

static const char *icm20608_state_name(enum icm20608_state state)
{
	switch (state) {
	case ICM20608_ST_RESET:
		return "reset";
	case ICM20608_ST_GUARD:
		return "guard";
	case ICM20608_ST_ACTIVE:
		return "active";
	case ICM20608_ST_COOLDOWN:
		return "cooldown";
	case ICM20608_ST_ERROR:
		return "error";
	case ICM20608_ST_SUSPEND:
		return "suspend";
	default:
		return "unknown";
	}
}

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

static void icm20608_set_state_locked(struct icm20608_dev *dev,
				      enum icm20608_state st)
{
	if (dev->state != st && dev->device)
		icm20608_info(dev, "state %s -> %s\n",
			      icm20608_state_name(dev->state),
			      icm20608_state_name(st));

	dev->state = st;
	dev->last_transition_jiffies = jiffies;
}

static int icm20608_read_regs_locked(struct icm20608_dev *dev, u8 reg,
				     void *buf, u8 len)
{
	struct spi_transfer xfer = { 0 };
	u8 txbuf[ICM20608_SPI_IO_MAX_LEN + 1];
	u8 rxbuf[ICM20608_SPI_IO_MAX_LEN + 1];
	int ret = -EINVAL;
	int retry;

	if (!dev || !dev->spi || !buf || !len || len > ICM20608_SPI_IO_MAX_LEN)
		return -EINVAL;

	txbuf[0] = reg | 0x80;
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
			dev->last_err = 0;
			return 0;
		}
		dev->retry_cnt++;
		usleep_range(ICM20608_SPI_RETRY_DELAY_US,
			     ICM20608_SPI_RETRY_DELAY_US + 300);
	}

	dev->read_err++;
	dev->last_err = ret;
	return ret;
}

static int icm20608_write_regs_locked(struct icm20608_dev *dev, u8 reg,
				      const u8 *buf, u8 len)
{
	struct spi_transfer xfer = { 0 };
	u8 txbuf[ICM20608_SPI_IO_MAX_LEN + 1];
	int ret = -EINVAL;
	int retry;

	if (!dev || !dev->spi || !buf || !len || len > ICM20608_SPI_IO_MAX_LEN)
		return -EINVAL;

	txbuf[0] = reg & ~0x80;
	memcpy(txbuf + 1, buf, len);
	xfer.tx_buf = txbuf;
	xfer.len = len + 1;

	for (retry = 0; retry < ICM20608_SPI_RETRY_MAX; retry++) {
		icm20608_cs_active(dev);
		ret = spi_sync_transfer(dev->spi, &xfer, 1);
		icm20608_cs_inactive(dev);
		if (!ret) {
			dev->last_err = 0;
			return 0;
		}
		dev->retry_cnt++;
		usleep_range(ICM20608_SPI_RETRY_DELAY_US,
			     ICM20608_SPI_RETRY_DELAY_US + 300);
	}

	dev->last_err = ret;
	return ret;
}

static int icm20608_read_onereg_locked(struct icm20608_dev *dev, u8 reg, u8 *data)
{
	return icm20608_read_regs_locked(dev, reg, data, 1);
}

static int icm20608_write_onereg_locked(struct icm20608_dev *dev, u8 reg, u8 value)
{
	return icm20608_write_regs_locked(dev, reg, &value, 1);
}

static int icm20608_fifo_reset_locked(struct icm20608_dev *dev)
{
	int ret;

	ret = icm20608_write_onereg_locked(dev, ICM20_USER_CTRL,
					   ICM20608_USER_CTRL_FIFO_RST);
	if (ret)
		return ret;

	return icm20608_write_onereg_locked(dev, ICM20_USER_CTRL,
					    ICM20608_USER_CTRL_FIFO_EN);
}

static int icm20608_read_fifo_count_locked(struct icm20608_dev *dev, u16 *count)
{
	u8 buf[2];
	int ret;

	ret = icm20608_read_regs_locked(dev, ICM20_FIFO_COUNTH, buf, sizeof(buf));
	if (ret)
		return ret;

	*count = (((u16)buf[0] & 0x1F) << 8) | (u16)buf[1];
	return 0;
}

static int icm20608_read_live_sample_locked(struct icm20608_dev *dev)
{
	u8 data[14];
	int ret;

	ret = icm20608_read_regs_locked(dev, ICM20_ACCEL_XOUT_H, data, sizeof(data));
	if (ret)
		return ret;

	dev->accel_x_adc = (signed short)((data[0] << 8) | data[1]);
	dev->accel_y_adc = (signed short)((data[2] << 8) | data[3]);
	dev->accel_z_adc = (signed short)((data[4] << 8) | data[5]);
	dev->temp_adc = (signed short)((data[6] << 8) | data[7]);
	dev->gyro_x_adc = (signed short)((data[8] << 8) | data[9]);
	dev->gyro_y_adc = (signed short)((data[10] << 8) | data[11]);
	dev->gyro_z_adc = (signed short)((data[12] << 8) | data[13]);
	return 0;
}

static int icm20608_drain_fifo_locked(struct icm20608_dev *dev)
{
	u8 packet[ICM20608_FIFO_SAMPLE_SIZE];
	u16 fifo_count = 0;
	u32 batches = 0;
	int ret;

	ret = icm20608_read_fifo_count_locked(dev, &fifo_count);
	if (ret)
		return ret;

	if (fifo_count < ICM20608_FIFO_SAMPLE_SIZE)
		return icm20608_read_live_sample_locked(dev);

	if (fifo_count % ICM20608_FIFO_SAMPLE_SIZE) {
		ret = icm20608_fifo_reset_locked(dev);
		if (ret)
			return ret;
		return icm20608_read_live_sample_locked(dev);
	}

	while (fifo_count >= ICM20608_FIFO_SAMPLE_SIZE) {
		ret = icm20608_read_regs_locked(dev, ICM20_FIFO_R_W, packet,
						ICM20608_FIFO_SAMPLE_SIZE);
		if (ret)
			return ret;

		dev->accel_x_adc = (signed short)((packet[0] << 8) | packet[1]);
		dev->accel_y_adc = (signed short)((packet[2] << 8) | packet[3]);
		dev->accel_z_adc = (signed short)((packet[4] << 8) | packet[5]);
		dev->gyro_x_adc = (signed short)((packet[6] << 8) | packet[7]);
		dev->gyro_y_adc = (signed short)((packet[8] << 8) | packet[9]);
		dev->gyro_z_adc = (signed short)((packet[10] << 8) | packet[11]);
		batches++;
		fifo_count -= ICM20608_FIFO_SAMPLE_SIZE;
	}

	dev->batch_size_last = batches;
	dev->batch_stat_samples++;
	if (dev->batch_stat_samples == 1)
		dev->batch_size_avg = batches;
	else
		dev->batch_size_avg =
			((dev->batch_size_avg * (dev->batch_stat_samples - 1)) + batches) /
			dev->batch_stat_samples;

	return 0;
}

static int icm20608_config_int_pin_locked(struct icm20608_dev *dev)
{
	return icm20608_write_onereg_locked(dev, ICM20_INT_PIN_CFG,
					    ICM20608_INT_PIN_CFG_DEFAULT);
}

static int icm20608_probe_identity_locked(struct icm20608_dev *dev)
{
	u8 whoami = 0;
	int ret;

	ret = icm20608_write_onereg_locked(dev, ICM20_PWR_MGMT_1, 0x80);
	if (ret)
		return ret;
	mdelay(ICM20608_RESET_DELAY_MS);

	ret = icm20608_write_onereg_locked(dev, ICM20_PWR_MGMT_1,
					   ICM20608_PWR_MGMT_1_CLKSEL_AUTO);
	if (ret)
		return ret;
	mdelay(ICM20608_RESET_DELAY_MS);

	ret = icm20608_read_onereg_locked(dev, ICM20_WHO_AM_I, &whoami);
	if (ret)
		return ret;

	if (whoami != ICM20608G_ID && whoami != ICM20608D_ID)
		return -ENODEV;

	dev->whoami = whoami;
	return 0;
}

static int icm20608_switch_to_guard_locked(struct icm20608_dev *dev)
{
	u8 int_status;
	int ret;

	ret = icm20608_write_onereg_locked(dev, ICM20_PWR_MGMT_1,
					   ICM20608_PWR_MGMT_1_CLKSEL_AUTO);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_PWR_MGMT_2,
					   ICM20608_PWR_MGMT_2_GYRO_STBY);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_ACCEL_CONFIG2, 0x01);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_FIFO_EN, 0x00);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_USER_CTRL, 0x00);
	if (ret)
		return ret;
	ret = icm20608_config_int_pin_locked(dev);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_ACCEL_WOM_THR,
					   dev->wom_threshold);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_ACCEL_INTEL_CTRL,
					   ICM20608_ACCEL_INTEL_CTRL_WOM);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_LP_MODE_CFG,
					   dev->lp_clk_sel & 0x0F);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_INT_ENABLE,
					   ICM20608_INT_ENABLE_WOM);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_PWR_MGMT_1,
					   ICM20608_PWR_MGMT_1_CLKSEL_AUTO |
					   ICM20608_PWR_MGMT_1_ACCEL_CYCLE);
	if (ret)
		return ret;

	ret = icm20608_read_onereg_locked(dev, ICM20_INT_STATUS, &int_status);
	if (ret)
		return ret;

	dev->hw_ready = true;
	dev->last_err = 0;
	dev->active_to_guard_cnt++;
	icm20608_set_state_locked(dev, ICM20608_ST_GUARD);
	return 0;
}

static int icm20608_switch_to_active_locked(struct icm20608_dev *dev)
{
	u8 int_status;
	int ret;

	ret = icm20608_write_onereg_locked(dev, ICM20_PWR_MGMT_1,
					   ICM20608_PWR_MGMT_1_CLKSEL_AUTO);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_PWR_MGMT_2, 0x00);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_SMPLRT_DIV, 0x00);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_GYRO_CONFIG, 0x18);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_ACCEL_CONFIG, 0x18);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_CONFIG, 0x04);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_ACCEL_CONFIG2, 0x04);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_LP_MODE_CFG, 0x00);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_ACCEL_INTEL_CTRL, 0x00);
	if (ret)
		return ret;
	ret = icm20608_config_int_pin_locked(dev);
	if (ret)
		return ret;
	ret = icm20608_fifo_reset_locked(dev);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_FIFO_EN,
					   ICM20608_FIFO_EN_ACTIVE);
	if (ret)
		return ret;
	ret = icm20608_write_onereg_locked(dev, ICM20_INT_ENABLE,
					   ICM20608_INT_ENABLE_FIFO);
	if (ret)
		return ret;

	ret = icm20608_read_onereg_locked(dev, ICM20_INT_STATUS, &int_status);
	if (ret)
		return ret;

	dev->hw_ready = true;
	dev->last_err = 0;
	dev->guard_to_active_cnt++;
	icm20608_set_state_locked(dev, ICM20608_ST_ACTIVE);
	return 0;
}

static int icm20608_force_active(struct icm20608_dev *dev)
{
	int ret = 0;

	mutex_lock(&dev->lock);
	if (dev->state != ICM20608_ST_ACTIVE) {
		ret = icm20608_switch_to_active_locked(dev);
		if (!ret)
			dev->recover_cnt++;
	}
	mutex_unlock(&dev->lock);

	if (!ret) {
		cancel_delayed_work_sync(&dev->cooldown_work);
		schedule_delayed_work(&dev->sample_work, 0);
		schedule_delayed_work(&dev->cooldown_work,
				      msecs_to_jiffies(dev->active_window_ms));
	}

	return ret;
}

static void icm20608_sample_workfn(struct work_struct *work)
{
	struct icm20608_dev *dev =
		container_of(to_delayed_work(work), struct icm20608_dev, sample_work);
	int ret;

	mutex_lock(&dev->lock);
	if (dev->state != ICM20608_ST_ACTIVE) {
		mutex_unlock(&dev->lock);
		return;
	}

	ret = icm20608_drain_fifo_locked(dev);
	if (ret) {
		dev->hw_ready = false;
		dev->last_err = ret;
		icm20608_set_state_locked(dev, ICM20608_ST_ERROR);
		mutex_unlock(&dev->lock);
		return;
	}
	mutex_unlock(&dev->lock);

	schedule_delayed_work(&dev->sample_work,
			      msecs_to_jiffies(dev->active_poll_ms));
}

static void icm20608_cooldown_workfn(struct work_struct *work)
{
	struct icm20608_dev *dev =
		container_of(to_delayed_work(work), struct icm20608_dev, cooldown_work);
	int ret;

	cancel_delayed_work_sync(&dev->sample_work);

	mutex_lock(&dev->lock);
	icm20608_set_state_locked(dev, ICM20608_ST_COOLDOWN);
	ret = icm20608_switch_to_guard_locked(dev);
	if (ret) {
		dev->hw_ready = false;
		dev->last_err = ret;
		icm20608_set_state_locked(dev, ICM20608_ST_ERROR);
	}
	mutex_unlock(&dev->lock);
}

static void icm20608_irq_workfn(struct work_struct *work)
{
	struct icm20608_dev *dev = container_of(work, struct icm20608_dev, irq_work);
	u8 int_status = 0;
	bool activate = false;
	bool sample_now = false;
	int ret;

	mutex_lock(&dev->lock);
	ret = icm20608_read_onereg_locked(dev, ICM20_INT_STATUS, &int_status);
	if (ret) {
		dev->hw_ready = false;
		dev->last_err = ret;
		icm20608_set_state_locked(dev, ICM20608_ST_ERROR);
		mutex_unlock(&dev->lock);
		return;
	}

	if (int_status & ICM20608_INT_STATUS_WOM_MASK) {
		dev->wom_irq_cnt++;
		if (dev->state == ICM20608_ST_GUARD || dev->state == ICM20608_ST_COOLDOWN)
			activate = true;
	}

	if (int_status & ICM20608_INT_STATUS_FIFO_OVF)
		dev->fifo_overflow++;

	if (activate) {
		ret = icm20608_switch_to_active_locked(dev);
		if (!ret)
			sample_now = true;
		else {
			dev->hw_ready = false;
			dev->last_err = ret;
			icm20608_set_state_locked(dev, ICM20608_ST_ERROR);
		}
	} else if (dev->state == ICM20608_ST_ACTIVE) {
		sample_now = true;
	}
	mutex_unlock(&dev->lock);

	if (sample_now) {
		cancel_delayed_work_sync(&dev->cooldown_work);
		schedule_delayed_work(&dev->sample_work, 0);
		schedule_delayed_work(&dev->cooldown_work,
				      msecs_to_jiffies(dev->active_window_ms));
	}
}

static irqreturn_t icm20608_irq_handler(int irq, void *data)
{
	struct icm20608_dev *dev = data;

	dev->irq_cnt++;
	schedule_work(&dev->irq_work);
	return IRQ_HANDLED;
}

static int icm20608_setup_sw_cs(struct icm20608_dev *dev)
{
	struct device_node *ecspi_np;
	int ret;

	dev->use_sw_cs = false;
	dev->cs_gpio = -EINVAL;

	ecspi_np = of_find_node_by_path(ICM20608_ECSPI3_NODE_PATH);
	if (!ecspi_np)
		return 0;

	dev->cs_gpio = of_get_named_gpio(ecspi_np, "cs-gpio", 0);
	of_node_put(ecspi_np);
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

static int icm20608_setup_irq(struct icm20608_dev *dev)
{
	int ret;

	if (dev->spi->irq > 0) {
		dev->irq = dev->spi->irq;
		dev->irq_gpio = -EINVAL;
	} else {
		dev->irq_gpio = of_get_named_gpio(dev->spi->dev.of_node,
						 "irq-gpios", 0);
		if (!gpio_is_valid(dev->irq_gpio))
			dev->irq_gpio = ICM20608_IRQ_GPIO_FALLBACK;

		ret = gpio_request_one(dev->irq_gpio, GPIOF_IN, ICM20608_IRQ_LABEL);
		if (ret) {
			dev->irq_gpio = -EINVAL;
			return ret;
		}
		dev->own_irq_gpio = true;
		dev->irq = gpio_to_irq(dev->irq_gpio);
		if (dev->irq < 0) {
			gpio_free(dev->irq_gpio);
			dev->own_irq_gpio = false;
			return dev->irq;
		}
	}

	ret = request_irq(dev->irq, icm20608_irq_handler,
			  IRQF_TRIGGER_FALLING, ICM20608_IRQ_LABEL, dev);
	if (ret)
		return ret;

	dev->irq_enabled = true;
	return 0;
}

static ssize_t icm20608_state_show(struct device *device,
				   struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 icm20608_state_name(icm20608dev.state));
}

static ssize_t icm20608_whoami_show(struct device *device,
				    struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", icm20608dev.whoami);
}

static ssize_t icm20608_swcs_show(struct device *device,
				  struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", icm20608dev.use_sw_cs ? 1 : 0);
}

static ssize_t icm20608_hw_ready_show(struct device *device,
				      struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", icm20608dev.hw_ready ? 1 : 0);
}

static ssize_t icm20608_read_err_show(struct device *device,
				      struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", icm20608dev.read_err);
}

static ssize_t icm20608_retry_cnt_show(struct device *device,
				       struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", icm20608dev.retry_cnt);
}

static ssize_t icm20608_last_err_show(struct device *device,
				      struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", icm20608dev.last_err);
}

static ssize_t icm20608_wom_thr_show(struct device *device,
				     struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", icm20608dev.wom_threshold);
}

static ssize_t icm20608_wom_thr_store(struct device *device,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (val > 255)
		return -EINVAL;

	mutex_lock(&icm20608dev.lock);
	icm20608dev.wom_threshold = (u8)val;
	mutex_unlock(&icm20608dev.lock);
	return count;
}

static ssize_t icm20608_active_window_show(struct device *device,
					   struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", icm20608dev.active_window_ms);
}

static ssize_t icm20608_active_window_store(struct device *device,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (val < 100 || val > 60000)
		return -EINVAL;

	mutex_lock(&icm20608dev.lock);
	icm20608dev.active_window_ms = (u32)val;
	mutex_unlock(&icm20608dev.lock);
	return count;
}

static ssize_t icm20608_lp_clk_show(struct device *device,
				    struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", icm20608dev.lp_clk_sel);
}

static ssize_t icm20608_lp_clk_store(struct device *device,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (val > 0x0F)
		return -EINVAL;

	mutex_lock(&icm20608dev.lock);
	icm20608dev.lp_clk_sel = (u8)val;
	mutex_unlock(&icm20608dev.lock);
	return count;
}

static ssize_t icm20608_diag_show(struct device *device,
				  struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			 "irq=%u wom_irq=%u fifo_overflow=%u batch_last=%u batch_avg=%u guard_to_active=%u active_to_guard=%u recover=%u irq_gpio=%d irq=%d\n",
			 icm20608dev.irq_cnt, icm20608dev.wom_irq_cnt,
			 icm20608dev.fifo_overflow, icm20608dev.batch_size_last,
			 icm20608dev.batch_size_avg, icm20608dev.guard_to_active_cnt,
			 icm20608dev.active_to_guard_cnt, icm20608dev.recover_cnt,
			 icm20608dev.irq_gpio, icm20608dev.irq);
}

static DEVICE_ATTR(state, 0444, icm20608_state_show, NULL);
static DEVICE_ATTR(whoami, 0444, icm20608_whoami_show, NULL);
static DEVICE_ATTR(use_sw_cs, 0444, icm20608_swcs_show, NULL);
static DEVICE_ATTR(hw_ready, 0444, icm20608_hw_ready_show, NULL);
static DEVICE_ATTR(read_err, 0444, icm20608_read_err_show, NULL);
static DEVICE_ATTR(retry_cnt, 0444, icm20608_retry_cnt_show, NULL);
static DEVICE_ATTR(last_err, 0444, icm20608_last_err_show, NULL);
static DEVICE_ATTR(wom_threshold, 0644, icm20608_wom_thr_show, icm20608_wom_thr_store);
static DEVICE_ATTR(active_window_ms, 0644,
		   icm20608_active_window_show, icm20608_active_window_store);
static DEVICE_ATTR(lp_clk_sel, 0644, icm20608_lp_clk_show, icm20608_lp_clk_store);
static DEVICE_ATTR(diag_stats, 0444, icm20608_diag_show, NULL);

static struct device_attribute *icm20608_attrs[] = {
	&dev_attr_state,
	&dev_attr_whoami,
	&dev_attr_use_sw_cs,
	&dev_attr_hw_ready,
	&dev_attr_read_err,
	&dev_attr_retry_cnt,
	&dev_attr_last_err,
	&dev_attr_wom_threshold,
	&dev_attr_active_window_ms,
	&dev_attr_lp_clk_sel,
	&dev_attr_diag_stats,
};

static int icm20608_create_sysfs(struct icm20608_dev *dev)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(icm20608_attrs); i++) {
		ret = device_create_file(dev->device, icm20608_attrs[i]);
		if (ret)
			goto err_remove;
	}

	return 0;

err_remove:
	while (--i >= 0)
		device_remove_file(dev->device, icm20608_attrs[i]);
	return ret;
}

static void icm20608_remove_sysfs(struct icm20608_dev *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(icm20608_attrs); i++)
		device_remove_file(dev->device, icm20608_attrs[i]);
}

static int icm20608_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &icm20608dev;
	return 0;
}

static ssize_t icm20608_read(struct file *filp, char __user *buf, size_t cnt,
			     loff_t *off)
{
	struct icm20608_dev *dev = filp->private_data;
	signed int data[7];
	int ret;

	if (cnt < sizeof(data))
		return -EINVAL;

	ret = icm20608_force_active(dev);
	if (ret)
		return ret;

	mutex_lock(&dev->lock);
	ret = icm20608_read_live_sample_locked(dev);
	if (!ret) {
		data[0] = dev->gyro_x_adc;
		data[1] = dev->gyro_y_adc;
		data[2] = dev->gyro_z_adc;
		data[3] = dev->accel_x_adc;
		data[4] = dev->accel_y_adc;
		data[5] = dev->accel_z_adc;
		data[6] = dev->temp_adc;
	}
	mutex_unlock(&dev->lock);

	if (ret)
		return ret;
	if (copy_to_user(buf, data, sizeof(data)))
		return -EFAULT;

	return sizeof(data);
}

static int icm20608_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations icm20608_ops = {
	.owner = THIS_MODULE,
	.open = icm20608_open,
	.read = icm20608_read,
	.release = icm20608_release,
};

static int icm20608_probe(struct spi_device *spi)
{
	struct icm20608_dev *dev = &icm20608dev;
	int ret;

	memset(dev, 0, sizeof(*dev));
	mutex_init(&dev->lock);
	INIT_WORK(&dev->irq_work, icm20608_irq_workfn);
	INIT_DELAYED_WORK(&dev->sample_work, icm20608_sample_workfn);
	INIT_DELAYED_WORK(&dev->cooldown_work, icm20608_cooldown_workfn);
	dev->spi = spi;
	dev->cs_gpio = -EINVAL;
	dev->irq_gpio = -EINVAL;
	dev->state = ICM20608_ST_RESET;
	dev->wom_threshold = ICM20608_WOM_THRESHOLD_DEFAULT;
	dev->active_window_ms = ICM20608_ACTIVE_WINDOW_MS_DEF;
	dev->active_poll_ms = ICM20608_ACTIVE_POLL_MS_DEF;
	dev->lp_clk_sel = ICM20608_LP_CLK_SEL_DEF;

	if (dev->major) {
		dev->devid = MKDEV(dev->major, 0);
		ret = register_chrdev_region(dev->devid, 0 + ICM20608_CNT,
					     ICM20608_NAME);
	} else {
		ret = alloc_chrdev_region(&dev->devid, 0, ICM20608_CNT,
					  ICM20608_NAME);
		dev->major = MAJOR(dev->devid);
	}
	if (ret)
		return ret;

	cdev_init(&dev->cdev, &icm20608_ops);
	ret = cdev_add(&dev->cdev, dev->devid, ICM20608_CNT);
	if (ret)
		goto err_unregister_chrdev;

	dev->class = class_create(THIS_MODULE, ICM20608_NAME);
	if (IS_ERR(dev->class)) {
		ret = PTR_ERR(dev->class);
		goto err_del_cdev;
	}

	dev->device = device_create(dev->class, NULL, dev->devid, NULL, ICM20608_NAME);
	if (IS_ERR(dev->device)) {
		ret = PTR_ERR(dev->device);
		goto err_destroy_class;
	}

	ret = icm20608_setup_sw_cs(dev);
	if (ret)
		goto err_destroy_device;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->max_speed_hz = 8000000;
	ret = spi_setup(spi);
	if (ret)
		goto err_release_swcs;

	mutex_lock(&dev->lock);
	ret = icm20608_probe_identity_locked(dev);
	if (!ret)
		ret = icm20608_switch_to_guard_locked(dev);
	mutex_unlock(&dev->lock);
	if (ret)
		goto err_release_swcs;

	ret = icm20608_create_sysfs(dev);
	if (ret)
		goto err_release_swcs;

	ret = icm20608_setup_irq(dev);
	if (ret)
		goto err_remove_sysfs;

	spi_set_drvdata(spi, dev);
	icm20608_info(dev,
		      "probe success sw_cs=%u whoami=0x%02x irq=%d irq_gpio=%d wom_thr=%u\n",
		      dev->use_sw_cs ? 1 : 0, dev->whoami, dev->irq, dev->irq_gpio,
		      dev->wom_threshold);
	return 0;

err_remove_sysfs:
	icm20608_remove_sysfs(dev);
err_release_swcs:
	icm20608_release_sw_cs(dev);
err_destroy_device:
	device_destroy(dev->class, dev->devid);
err_destroy_class:
	class_destroy(dev->class);
err_del_cdev:
	cdev_del(&dev->cdev);
err_unregister_chrdev:
	unregister_chrdev_region(dev->devid, ICM20608_CNT);
	return ret;
}

static int icm20608_remove(struct spi_device *spi)
{
	struct icm20608_dev *dev = spi_get_drvdata(spi);

	if (!dev)
		return 0;

	cancel_delayed_work_sync(&dev->cooldown_work);
	cancel_delayed_work_sync(&dev->sample_work);
	cancel_work_sync(&dev->irq_work);

	if (dev->irq_enabled)
		free_irq(dev->irq, dev);
	if (dev->own_irq_gpio && gpio_is_valid(dev->irq_gpio))
		gpio_free(dev->irq_gpio);

	mutex_lock(&dev->lock);
	icm20608_set_state_locked(dev, ICM20608_ST_SUSPEND);
	mutex_unlock(&dev->lock);

	icm20608_remove_sysfs(dev);
	icm20608_release_sw_cs(dev);
	cdev_del(&dev->cdev);
	unregister_chrdev_region(dev->devid, ICM20608_CNT);
	device_destroy(dev->class, dev->devid);
	class_destroy(dev->class);
	dev->spi = NULL;
	dev->hw_ready = false;
	return 0;
}

static const struct spi_device_id icm20608_id[] = {
	{"alientek,icm20608", 0},
	{"icm20608", 0},
	{}
};

static const struct of_device_id icm20608_of_match[] = {
	{ .compatible = "alientek,icm20608" },
	{ .compatible = "icm20608" },
	{ }
};

static struct spi_driver icm20608_driver = {
	.probe = icm20608_probe,
	.remove = icm20608_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "icm20608",
		.of_match_table = icm20608_of_match,
	},
	.id_table = icm20608_id,
};

static int __init icm20608_init(void)
{
	return spi_register_driver(&icm20608_driver);
}

static void __exit icm20608_exit(void)
{
	spi_unregister_driver(&icm20608_driver);
}

module_init(icm20608_init);
module_exit(icm20608_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zuozhongkai");
