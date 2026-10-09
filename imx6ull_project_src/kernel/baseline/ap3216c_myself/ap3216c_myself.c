/*
 * AP3216C Sensor Driver for IMX6ULL - ap3216c_myself
 * 
 * 基于 rEDTii/AP3216C-Linux-Driver 参考设计
 * 架构: 顶半部 IRQ Handler + 底半部 Workqueue
 * 
 * 功能:
 * 1. Linux Input 子系统 (SW_FRONT_PROXIMITY 事件)
 * 2. 硬件中断驱动 (无轮询)
 * 3. 滞回阈值防抖
 * 
 * 开发时间: 2026-05-06
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>

#include "ap3216creg.h"

/*
 * GPIO 中断异常时仅用于调试：轮询 INTSTATUS（毫秒，0=关闭）。
 * insmod ap3216c_myself.ko irq_poll_ms=50
 */
static int irq_poll_ms;
module_param(irq_poll_ms, int, 0644);
MODULE_PARM_DESC(irq_poll_ms,
    "Poll INTSTATUS interval in ms when GPIO IRQ does not fire (0=disable)");

/*
 * PS 中断形式寄存器 0x22（须显式配置，否则 PS_INT 可能永不置位）:
 * 0 = ZONE — 测量值在 [low,high] 之外触发（AP3216_WE INT_MODE_ZONE）
 * 1 = HYSTERESIS — 滞回模式（AP3216_WE INT_MODE_HYSTERESIS，近手机场景）
 */
static int ps_int_mode;
module_param(ps_int_mode, int, 0644);
MODULE_PARM_DESC(ps_int_mode,
    "PS interrupt form reg 0x22: 0=ZONE outside band, 1=HYSTERESIS (default 0)");

/* 周期性打印 PS/INTSTATUS 的间隔(ms)，0=关闭（避免误用读清中断方式时干扰） */
static int debug_interval_ms;
module_param(debug_interval_ms, int, 0644);
MODULE_PARM_DESC(debug_interval_ms,
    "Periodic debug log interval ms (0=disable; reads PS regs each time)");

/* PS 阈值（可由 insmod 传入，缺省 20/80，适用于 HYSTERESIS 滞回模式） */
static int ps_threshold_low = 20;
module_param(ps_threshold_low, int, 0644);
MODULE_PARM_DESC(ps_threshold_low,
    "PS low/away threshold (default 20)");

static int ps_threshold_high = 80;
module_param(ps_threshold_high, int, 0644);
MODULE_PARM_DESC(ps_threshold_high,
    "PS high/near threshold (default 80)");

/*
 * PS 测量间隔寄存器 0x24 (PSLEDWT)：
 *   wait = CMD_PSLEDWT[4:0]（5位，0x04 约 1.1s，0x01 约 0.4s）
 *   间隔 ≈ 225 * wait + 212 ms（rEDTii 测量）
 * 默认 0x04 ≈ 1112 ms（低功耗巡检）。
 * 贴近场景可设为 0x01（≈437 ms）加快响应。
 */
static int ps_measure_interval = 0x04;
module_param(ps_measure_interval, int, 0644);
MODULE_PARM_DESC(ps_measure_interval,
    "PSLEDWT reg value (0x01~0x1F, default 0x04); lower=faster");

/* 驱动信息 */
#define AP3216C_MYSELF_CNT      1
#define AP3216C_MYSELF_NAME     "ap3216c_myself"
#define AP3216C_MYSELF_CLASS    "ap3216c_myself"
#define AP3216C_INPUT_NAME      "ap3216c-proximity"

/* 调试开关 */
#define AP3216C_DEBUG           1
#if AP3216C_DEBUG
#define ap3216c_dbg(dev, fmt, ...) dev_info(dev, fmt, ##__VA_ARGS__)
#else
#define ap3216c_dbg(dev, fmt, ...) do {} while (0)
#endif

#define ap3216c_warn(dev, fmt, ...) dev_warn(dev, "ap3216c_myself: " fmt, ##__VA_ARGS__)
#define ap3216c_err(dev, fmt, ...) dev_err(dev, "ap3216c_myself: " fmt, ##__VA_ARGS__)
#define ap3216c_info(dev, fmt, ...) dev_info(dev, "ap3216c_myself: " fmt, ##__VA_ARGS__)

/* I2C 重试参数 */
#define AP3216C_I2C_RETRY_MAX   3
#define AP3216C_I2C_RETRY_DELAY_US 1500

/* 复位等待时间 */
#define AP3216C_RESET_DELAY_MS   50

/*
 * 默认阈值 (insmod 可覆盖):
 *   high=200  — 物体贴近后 PS 需 >200 才报告 FAR (远离)
 *   low=50    — 物体远离后 PS 需 <50 才报告 NEAR (贴近)
 * 在此阈值窗口内芯片保持静默，无中断。
 */
#define DEFAULT_PS_THRESHOLD_HIGH  200
#define DEFAULT_PS_THRESHOLD_LOW    50

/* 传感器数据结构 */
struct ap3216c_data {
    u16 ir;
    u16 als;
    u16 ps;
};

/* AP3216C 设备结构 */
struct ap3216c_dev {
    struct cdev cdev;
    struct class *class;
    struct device *dev;
    struct i2c_client *client;
    struct input_dev *input;
    struct mutex lock;

    /* 顶半部 + 底半部中断处理 */
    struct work_struct work;      /* 底半部工作队列 */
    struct delayed_work debug_work;  /* 调试用定时器 */
    struct delayed_work irq_poller;   /* 可选：INTSTATUS 轮询（irq_poll_ms>0） */
    int irq;

    /* 阈值配置 */
    int ps_threshold_low;
    int ps_threshold_high;

    /* 传感器数据缓存 */
    struct ap3216c_data data;

    /* 统计 */
    atomic_t near_count;
    atomic_t far_count;
    atomic_t irq_count;

    dev_t devno;
    int major;

    bool hw_initialized;
};

/* 全局设备实例 */
static struct ap3216c_dev *ap3216c_device;

/* I2C 读写函数 */
static int ap3216c_i2c_read(struct ap3216c_dev *dev, u8 reg)
{
    struct i2c_msg msg[2];
    u8 data;
    int ret;

    msg[0].addr = dev->client->addr;
    msg[0].flags = 0;
    msg[0].buf = &reg;
    msg[0].len = 1;

    msg[1].addr = dev->client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = &data;
    msg[1].len = 1;

    ret = i2c_transfer(dev->client->adapter, msg, 2);
    if (ret != 2) {
        ap3216c_err(dev->dev, "I2C read failed at reg 0x%02x\n", reg);
        return -EIO;
    }

    return data;
}

static int ap3216c_i2c_write(struct ap3216c_dev *dev, u8 reg, u8 val)
{
    u8 buf[2];
    struct i2c_msg msg;
    int ret;

    buf[0] = reg;
    buf[1] = val;

    msg.addr = dev->client->addr;
    msg.flags = 0;
    msg.buf = buf;
    msg.len = 2;

    ret = i2c_transfer(dev->client->adapter, &msg, 1);
    if (ret != 1) {
        ap3216c_err(dev->dev, "I2C write failed at reg 0x%02x\n", reg);
        return -EIO;
    }

    return 0;
}

/* 清除芯片内部中断标志并读数据寄存器，有助于 INT 开漏脚释放为高（参见 gemini / 手册） */
static void ap3216c_flush_interrupt_state(struct ap3216c_dev *dev)
{
    ap3216c_i2c_write(dev, AP3216C_INTSTATUS,
                      AP3216C_ALSINT_STATE | AP3216C_PSINT_STATE);
    ap3216c_i2c_read(dev, AP3216C_PSDATALOW);
    ap3216c_i2c_read(dev, AP3216C_PSDATAHIGH);
    ap3216c_i2c_read(dev, AP3216C_ALSDATALOW);
    ap3216c_i2c_read(dev, AP3216C_ALSDATAHIGH);
    mdelay(2);
}

/* 硬件初始化 */
static int ap3216c_hw_init(struct ap3216c_dev *dev)
{
    int ret;

    ap3216c_info(dev->dev, "Initializing AP3216C hardware...\n");

    /* 1. 软件复位 */
    ret = ap3216c_i2c_write(dev, AP3216C_SYSTEMCONG, 0x04);
    if (ret < 0) {
        ap3216c_err(dev->dev, "Software reset failed\n");
        return ret;
    }
    mdelay(AP3216C_RESET_DELAY_MS);

    /* 2. 配置为 PS + ALS 模式 */
    ret = ap3216c_i2c_write(dev, AP3216C_SYSTEMCONG, AP3216C_MODE_ALS_PS_IR);
    if (ret < 0) {
        ap3216c_err(dev->dev, "Mode config failed\n");
        return ret;
    }

    /* 3. 配置中断清除模式为软件手动清除 (与 rEDTii 驱动一致) */
    ret = ap3216c_i2c_write(dev, AP3216C_INTCLEAR, 0x01);
    if (ret < 0) {
        ap3216c_err(dev->dev, "Interrupt clear mode config failed\n");
        return ret;
    }

    /* 4. 清除可能存在的中断标志 */
    ret = ap3216c_i2c_write(dev, AP3216C_INTSTATUS, 0x02);
    if (ret < 0) {
        ap3216c_err(dev->dev, "Clear interrupt flag failed\n");
        return ret;
    }

    /* 5. 配置LED驱动电流 (关键！与 rEDTii 驱动一致) */
    ret = ap3216c_i2c_write(dev, AP3216C_PSLEDCONFIG, AP3216C_PSLED_DEFAULT);
    if (ret < 0) {
        ap3216c_err(dev->dev, "PS LED config failed\n");
        return ret;
    }

    /* 6. 配置阈值 (与 rEDTii 驱动一致) */
    /* 阈值是 10 位值，低 2 位在 PSLTHDL/PSHTHDL，高 8 位在 PSLTHDH/PSHTHDH */
    ret = ap3216c_i2c_write(dev, AP3216C_PSLTHDL, dev->ps_threshold_low & 0x03);
    ret |= ap3216c_i2c_write(dev, AP3216C_PSLTHDH, (dev->ps_threshold_low >> 2) & 0xFF);
    ret |= ap3216c_i2c_write(dev, AP3216C_PSHTHDL, dev->ps_threshold_high & 0x03);
    ret |= ap3216c_i2c_write(dev, AP3216C_PSHTHDH, (dev->ps_threshold_high >> 2) & 0xFF);
    if (ret < 0) {
        ap3216c_err(dev->dev, "Threshold config failed\n");
        return ret;
    }

    /* 7. PS 测量间隔 (PSLEDWT reg 0x24)，可由 ps_measure_interval 覆盖 */
    ret = ap3216c_i2c_write(dev, AP3216C_PSLEDWT, ps_measure_interval & 0x1F);
    if (ret < 0) {
        ap3216c_err(dev->dev, "PS measure interval failed\n");
        return ret;
    }
    ap3216c_info(dev->dev, "REG 0x24 PSLEDWT: 0x%02x (interval ~%d ms)\n",
                 ps_measure_interval & 0x1F,
                 225 * (ps_measure_interval & 0x1F) + 212);

    /*
     * 8. PS 中断算法 (reg 0x22) — Wolles/AP3216_WE: ZONE=0x00, HYSTERESIS=0x01。
     * 未配置时部分芯片 PS_INT 永不置位，INT 引脚无脉冲，GPIO 下降沿为 0。
     */
    {
        u8 form = (ps_int_mode != 0) ? AP3216C_PS_INT_HYSTERESIS
                                     : AP3216C_PS_INT_ZONE;

        ret = ap3216c_i2c_write(dev, AP3216C_PS_INT_FORM, form);
        if (ret < 0) {
            ap3216c_err(dev->dev, "PS interrupt form (0x22) failed\n");
            return ret;
        }
    }

    /* 9. 显式使能 PS + PS 中断 (AP3216C_PSCONFIG_DEFAULT 0x05)，写在阈值/模式之后 */
    ret = ap3216c_i2c_write(dev, AP3216C_PSCONFIG, AP3216C_PSCONFIG_DEFAULT);
    if (ret < 0) {
        ap3216c_err(dev->dev, "PSCONFIG write failed\n");
        return ret;
    }

    /* 配置完成后再次清中断，确保 INT 引脚释放（开漏 + 内部上拉 → 高） */
    ap3216c_flush_interrupt_state(dev);

    /* 调试: 读取并打印寄存器值以验证配置 */
    ap3216c_info(dev->dev, "=== AP3216C Register Dump ===\n");
    ap3216c_info(dev->dev, "REG 0x00 SYSTEM: 0x%02x\n",
                 ap3216c_i2c_read(dev, AP3216C_SYSTEMCONG));
    ap3216c_info(dev->dev, "REG 0x01 INTSTATUS: 0x%02x\n",
                 ap3216c_i2c_read(dev, AP3216C_INTSTATUS));
    ap3216c_info(dev->dev, "REG 0x02 INTCLR_MODE: 0x%02x\n",
                 ap3216c_i2c_read(dev, AP3216C_INTCLEAR));
    ap3216c_info(dev->dev, "REG 0x22 PS_INT_FORM: 0x%02x (mode param %d)\n",
                 ap3216c_i2c_read(dev, AP3216C_PS_INT_FORM), ps_int_mode);
    ap3216c_info(dev->dev, "REG 0x20 PSCONFIG: 0x%02x\n",
                 ap3216c_i2c_read(dev, AP3216C_PSCONFIG));
    ap3216c_info(dev->dev, "REG 0x21 LEDCONFIG: 0x%02x\n",
                 ap3216c_i2c_read(dev, AP3216C_PSLEDCONFIG));
    ap3216c_info(dev->dev, "REG 0x2A-0x2D Threshold: L=%d H=%d\n",
                 dev->ps_threshold_low, dev->ps_threshold_high);
    ap3216c_info(dev->dev, "================================\n");

    dev->hw_initialized = true;
    ap3216c_info(dev->dev,
                 "AP3216C initialized (thr low=%d high=%d, ps_int_mode=%d ZONE/HYS)\n",
                 dev->ps_threshold_low, dev->ps_threshold_high, ps_int_mode);

    return 0;
}

/* ================================================================
 * 顶半部 IRQ Handler + 底半部 Workqueue
 * 
 * 参考 rEDTii/AP3216C-Linux-Driver 设计
 * ================================================================ */

/* 调试工作队列回调 - 定期读取传感器状态 */
static void ap3216c_debug_work_callback(struct work_struct *work)
{
    struct ap3216c_dev *dev = container_of(work, struct ap3216c_dev, debug_work.work);
    int int_status, ps_data_l, ps_data_h;

    mutex_lock(&dev->lock);

    int_status = ap3216c_i2c_read(dev, AP3216C_INTSTATUS);
    ps_data_l = ap3216c_i2c_read(dev, AP3216C_PSDATALOW);
    ps_data_h = ap3216c_i2c_read(dev, AP3216C_PSDATAHIGH);

    printk(KERN_INFO "ap3216c DEBUG: INT_STATUS=0x%02x PS_L=0x%02x PS_H=0x%02x proximity=%d\n",
           int_status, ps_data_l, ps_data_h, (ps_data_l >> 7) & 0x01);

    mutex_unlock(&dev->lock);

    if (debug_interval_ms > 0)
        schedule_delayed_work(&dev->debug_work,
                              msecs_to_jiffies(debug_interval_ms));
}

/* 可选：轮询 INTSTATUS，在 GPIO 线上无中断但芯片已置位 PS 时仍能处理（调试/兜底） */
static void ap3216c_irq_poller_fn(struct work_struct *work)
{
    struct delayed_work *dwork = to_delayed_work(work);
    struct ap3216c_dev *dev = container_of(dwork, struct ap3216c_dev, irq_poller);
    int st;

    if (!irq_poll_ms)
        return;

    if (mutex_lock_interruptible(&dev->lock) != 0)
        goto again;
    st = ap3216c_i2c_read(dev, AP3216C_INTSTATUS);
    mutex_unlock(&dev->lock);

    if (st & 0x02) {
        atomic_inc(&dev->irq_count);
        dev_info(dev->dev,
                 "PS interrupt pending in INTSTATUS (poller); scheduling bottom half\n");
        schedule_work(&dev->work);
    }

again:
    schedule_delayed_work(&dev->irq_poller, msecs_to_jiffies(irq_poll_ms));
}

/* 顶半部: 仅调度底半部，快速返回 */
static irqreturn_t ap3216c_irq_handler(int irq, void *dev_id)
{
    struct ap3216c_dev *dev = dev_id;

    atomic_inc(&dev->irq_count);
    printk(KERN_INFO "ap3216c IRQ triggered! count=%d\n", atomic_read(&dev->irq_count));

    /* 调度底半部处理 */
    schedule_work(&dev->work);

    return IRQ_HANDLED;
}

/* 底半部: 在工作队列上下文执行 I2C 访问 */
static void ap3216c_work_callback(struct work_struct *work)
{
    struct ap3216c_dev *dev = container_of(work, struct ap3216c_dev, work);
    int int_status;
    int ps_data_l;
    int ps_data_h;
    int proximity_state;

    /* 获取互斥锁 */
    if (mutex_lock_interruptible(&dev->lock) != 0)
        return;

    /* 读取中断状态寄存器 */
    int_status = ap3216c_i2c_read(dev, AP3216C_INTSTATUS);

    /* 只处理 PS 中断 (bit 1 = 0x02) */
    if (!(int_status & 0x02)) {
        ap3216c_dbg(dev->dev, "Not PS interrupt, int_status=0x%02x\n", int_status);
        mutex_unlock(&dev->lock);
        return;
    }

    /* 清除 PS 中断标志 (写 0x02 清除) */
    ap3216c_i2c_write(dev, AP3216C_INTSTATUS, 0x02);

    /* 读取 PS 数据 (与 rEDTii 驱动一致) */
    ps_data_l = ap3216c_i2c_read(dev, AP3216C_PSDATALOW);

    /* 检查数据有效性 (bit 6) */
    if (ps_data_l & 0x40) {
        ap3216c_dbg(dev->dev, "PS data invalid\n");
        mutex_unlock(&dev->lock);
        return;
    }

    /* 从 bit 7 获取接近状态: 1=near, 0=far */
    proximity_state = (ps_data_l >> 7) & 0x01;

    /* 读取高字节计算完整 PS 值 */
    ps_data_h = ap3216c_i2c_read(dev, AP3216C_PSDATAHIGH);
    dev->data.ps = ((ps_data_h & 0x3F) << 4) | (ps_data_l & 0x0F);

    /* 更新统计计数 */
    if (proximity_state)
        atomic_inc(&dev->near_count);
    else
        atomic_inc(&dev->far_count);

    /* 上报 Input 事件 */
    input_report_switch(dev->input, SW_FRONT_PROXIMITY, proximity_state);
    input_sync(dev->input);

    ap3216c_info(dev->dev, "proximity: %s PS=%d (near=%d, far=%d)\n",
                 proximity_state ? "NEAR" : "FAR", dev->data.ps,
                 atomic_read(&dev->near_count),
                 atomic_read(&dev->far_count));

    mutex_unlock(&dev->lock);
}

/* ================================================================
 * 字符设备接口
 * ================================================================ */

static int ap3216c_open(struct inode *inode, struct file *filp)
{
    struct ap3216c_dev *dev = ap3216c_device;

    if (!dev)
        return -ENODEV;

    filp->private_data = dev;
    ap3216c_info(dev->dev, "Device opened\n");

    return 0;
}

static ssize_t ap3216c_read(struct file *filp, char __user *buf,
                           size_t count, loff_t *f_pos)
{
    struct ap3216c_dev *dev = filp->private_data;
    struct ap3216c_data data;

    if (!dev || !buf)
        return -EINVAL;

    /*
     * 重要: 这里不发起I2C传输！
     * AP3216C的"读后清零"机制会清除中断标志
     * 数据由中断底半部更新，这里只返回缓存值
     */
    mutex_lock(&dev->lock);
    data = dev->data;
    mutex_unlock(&dev->lock);

    if (copy_to_user(buf, &data, sizeof(data)))
        return -EFAULT;

    return sizeof(data);
}

static int ap3216c_release(struct inode *inode, struct file *filp)
{
    struct ap3216c_dev *dev = filp->private_data;

    if (dev)
        ap3216c_info(dev->dev, "Device closed\n");

    return 0;
}

static const struct file_operations ap3216c_fops = {
    .owner   = THIS_MODULE,
    .open    = ap3216c_open,
    .read    = ap3216c_read,
    .release = ap3216c_release,
};

/* ================================================================
 * sysfs 属性接口
 * ================================================================ */

static ssize_t raw_data_show(struct device *dev_struct,
                            struct device_attribute *attr, char *buf)
{
    struct ap3216c_dev *dev = ap3216c_device;
    int ir, als, ps;

    if (!dev)
        return -ENODEV;

    mutex_lock(&dev->lock);

    ir = ap3216c_i2c_read(dev, AP3216C_IRDATALOW);
    ir |= (ap3216c_i2c_read(dev, AP3216C_IRDATAHIGH) << 8);

    als = ap3216c_i2c_read(dev, AP3216C_ALSDATALOW);
    als |= (ap3216c_i2c_read(dev, AP3216C_ALSDATAHIGH) << 8);

    ps = ap3216c_i2c_read(dev, AP3216C_PSDATALOW);
    ps |= ((ap3216c_i2c_read(dev, AP3216C_PSDATAHIGH) & 0x3F) << 8);

    mutex_unlock(&dev->lock);

    return scnprintf(buf, PAGE_SIZE, "ir=%d als=%d ps=%d\n", ir, als, ps);
}

static ssize_t prox_counts_show(struct device *dev_struct,
                              struct device_attribute *attr, char *buf)
{
    struct ap3216c_dev *dev = ap3216c_device;

    if (!dev)
        return -ENODEV;

    return scnprintf(buf, PAGE_SIZE,
                    "near=%d far=%d irq=%d\n",
                    atomic_read(&dev->near_count),
                    atomic_read(&dev->far_count),
                    atomic_read(&dev->irq_count));
}

static ssize_t irq_status_show(struct device *dev_struct,
                              struct device_attribute *attr, char *buf)
{
    struct ap3216c_dev *dev = ap3216c_device;

    if (!dev)
        return -ENODEV;

    return scnprintf(buf, PAGE_SIZE,
                    "irq=%d irq_cnt=%d\n",
                    dev->irq,
                    atomic_read(&dev->irq_count));
}

static ssize_t threshold_low_show(struct device *dev_struct,
                                 struct device_attribute *attr, char *buf)
{
    struct ap3216c_dev *dev = ap3216c_device;

    if (!dev)
        return -ENODEV;

    return scnprintf(buf, PAGE_SIZE, "%d\n", dev->ps_threshold_low);
}

static ssize_t threshold_low_store(struct device *dev_struct,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
    struct ap3216c_dev *dev = ap3216c_device;
    int val;

    if (!dev)
        return -ENODEV;

    if (kstrtoint(buf, 10, &val) < 0)
        return -EINVAL;

    if (val < 0 || val > 1023)
        return -EINVAL;

    if (val > dev->ps_threshold_high)
        return -EINVAL;

    mutex_lock(&dev->lock);
    ap3216c_i2c_write(dev, AP3216C_PSLTHDL, val & 0x03);  /* 低2位 */
    ap3216c_i2c_write(dev, AP3216C_PSLTHDH, (val >> 2) & 0xFF);  /* 高8位 */
    dev->ps_threshold_low = val;
    mutex_unlock(&dev->lock);

    return count;
}

static ssize_t threshold_high_show(struct device *dev_struct,
                                 struct device_attribute *attr, char *buf)
{
    struct ap3216c_dev *dev = ap3216c_device;

    if (!dev)
        return -ENODEV;

    return scnprintf(buf, PAGE_SIZE, "%d\n", dev->ps_threshold_high);
}

static ssize_t threshold_high_store(struct device *dev_struct,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
    struct ap3216c_dev *dev = ap3216c_device;
    int val;

    if (!dev)
        return -ENODEV;

    if (kstrtoint(buf, 10, &val) < 0)
        return -EINVAL;

    if (val < 0 || val > 1023)
        return -EINVAL;

    if (val < dev->ps_threshold_low)
        return -EINVAL;

    mutex_lock(&dev->lock);
    ap3216c_i2c_write(dev, AP3216C_PSHTHDL, val & 0x03);  /* 低2位 */
    ap3216c_i2c_write(dev, AP3216C_PSHTHDH, (val >> 2) & 0xFF);  /* 高8位 */
    dev->ps_threshold_high = val;
    mutex_unlock(&dev->lock);

    return count;
}

static DEVICE_ATTR_RO(raw_data);
static DEVICE_ATTR_RO(prox_counts);
static DEVICE_ATTR_RO(irq_status);
static DEVICE_ATTR_RW(threshold_low);
static DEVICE_ATTR_RW(threshold_high);

static struct attribute *ap3216c_attrs[] = {
    &dev_attr_raw_data.attr,
    &dev_attr_prox_counts.attr,
    &dev_attr_irq_status.attr,
    &dev_attr_threshold_low.attr,
    &dev_attr_threshold_high.attr,
    NULL,
};

static const struct attribute_group ap3216c_attr_group = {
    .attrs = ap3216c_attrs,
};

/* ================================================================
 * Input 子系统
 * ================================================================ */

static int ap3216c_input_init(struct ap3216c_dev *dev)
{
    int ret;

    dev->input = input_allocate_device();
    if (!dev->input)
        return -ENOMEM;

    dev->input->name = AP3216C_INPUT_NAME;
    dev->input->phys = "ap3216c_myself/input0";
    dev->input->id.bustype = BUS_I2C;
    dev->input->dev.parent = &dev->client->dev;

    set_bit(EV_SW, dev->input->evbit);
    set_bit(SW_FRONT_PROXIMITY, dev->input->swbit);

    ret = input_register_device(dev->input);
    if (ret) {
        input_free_device(dev->input);
        dev->input = NULL;
        return ret;
    }

    /* 初始化时上报一次远离状态 */
    input_report_switch(dev->input, SW_FRONT_PROXIMITY, 0);
    input_sync(dev->input);

    return 0;
}

static void ap3216c_input_exit(struct ap3216c_dev *dev)
{
    if (dev->input) {
        input_unregister_device(dev->input);
        dev->input = NULL;
    }
}

/* ================================================================
 * I2C 驱动接口
 * ================================================================ */

static int ap3216c_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    struct ap3216c_dev *dev;
    int ret;

    ap3216c_info(&client->dev, "ap3216c_myself probing...\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        ap3216c_err(&client->dev, "I2C not supported\n");
        return -ENODEV;
    }

    dev = kzalloc(sizeof(struct ap3216c_dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    dev->client = client;
    i2c_set_clientdata(client, dev);

    /* 初始化互斥锁 */
    mutex_init(&dev->lock);

    /* 初始化阈值 */
    dev->ps_threshold_low = DEFAULT_PS_THRESHOLD_LOW;
    dev->ps_threshold_high = DEFAULT_PS_THRESHOLD_HIGH;

    /* 初始化统计 */
    atomic_set(&dev->near_count, 0);
    atomic_set(&dev->far_count, 0);
    atomic_set(&dev->irq_count, 0);

    dev->hw_initialized = false;

    /* 注册字符设备 */
    ret = alloc_chrdev_region(&dev->devno, 0, AP3216C_MYSELF_CNT, AP3216C_MYSELF_NAME);
    if (ret) {
        ap3216c_err(&client->dev, "Failed to allocate chrdev region: %d\n", ret);
        goto err_free_dev;
    }

    dev->major = MAJOR(dev->devno);

    cdev_init(&dev->cdev, &ap3216c_fops);
    dev->cdev.owner = THIS_MODULE;

    ret = cdev_add(&dev->cdev, dev->devno, AP3216C_MYSELF_CNT);
    if (ret) {
        ap3216c_err(&client->dev, "Failed to add cdev: %d\n", ret);
        goto err_unreg_chrdev;
    }

    /* 创建设备类 */
    dev->class = class_create(THIS_MODULE, AP3216C_MYSELF_CLASS);
    if (IS_ERR(dev->class)) {
        ret = PTR_ERR(dev->class);
        ap3216c_err(&client->dev, "Failed to create class: %d\n", ret);
        goto err_cdev_del;
    }

    /* 创建设备节点 */
    dev->dev = device_create(dev->class, NULL, dev->devno, NULL, AP3216C_MYSELF_NAME);
    if (IS_ERR(dev->dev)) {
        ret = PTR_ERR(dev->dev);
        ap3216c_err(&client->dev, "Failed to create device: %d\n", ret);
        goto err_class_destroy;
    }

    /* 创建 sysfs 属性 */
    ret = sysfs_create_group(&dev->dev->kobj, &ap3216c_attr_group);
    if (ret) {
        ap3216c_err(&client->dev, "Failed to create sysfs: %d\n", ret);
        goto err_device_destroy;
    }

    /* 初始化 Input 设备 */
    ret = ap3216c_input_init(dev);
    if (ret) {
        ap3216c_err(&client->dev, "Failed to init input device: %d\n", ret);
        goto err_sysfs_remove;
    }

    /* 设置全局设备指针 */
    ap3216c_device = dev;

    /* 初始化硬件 */
    ret = ap3216c_hw_init(dev);
    if (ret) {
        ap3216c_err(&client->dev, "Failed to init hardware: %d\n", ret);
        goto err_input_exit;
    }

    /* 初始化底半部工作队列 */
    INIT_WORK(&dev->work, ap3216c_work_callback);
    INIT_DELAYED_WORK(&dev->irq_poller, ap3216c_irq_poller_fn);

    /* request_irq 前再清一次，避免 INT 长时间为低导致无边沿可检测 */
    ap3216c_flush_interrupt_state(dev);

    /* 请求中断 - 先初始化硬件和work，防止误触发 */
    dev->irq = client->irq;
    if (dev->irq <= 0) {
        ap3216c_err(&client->dev, "No IRQ available\n");
        ret = -EINVAL;
        goto err_hw_init;
    }

    ret = request_irq(dev->irq, ap3216c_irq_handler,
                     IRQF_TRIGGER_FALLING,
                     "ap3216c_irq", dev);
    if (ret) {
        ap3216c_err(&client->dev, "Failed to request IRQ: %d\n", ret);
        dev->irq = 0;
        goto err_hw_init;
    }

    if (irq_poll_ms > 0) {
        schedule_delayed_work(&dev->irq_poller,
                              msecs_to_jiffies(irq_poll_ms));
        ap3216c_info(&client->dev,
                     "irq_poll_ms=%d: polling INTSTATUS as fallback\n",
                     irq_poll_ms);
    }

    INIT_DELAYED_WORK(&dev->debug_work, ap3216c_debug_work_callback);
    if (debug_interval_ms > 0) {
        schedule_delayed_work(&dev->debug_work,
                              msecs_to_jiffies(debug_interval_ms));
        ap3216c_info(&client->dev,
                     "debug_interval_ms=%d: periodic PS/INTSTATUS logs enabled\n",
                     debug_interval_ms);
    }

    ap3216c_info(&client->dev, "ap3216c_myself probed successfully\n");
    ap3216c_info(&client->dev, "Device node: /dev/%s\n", AP3216C_MYSELF_NAME);
    ap3216c_info(&client->dev, "IRQ: %d\n", dev->irq);

    return 0;

err_hw_init:
    ap3216c_i2c_write(dev, AP3216C_SYSTEMCONG, 0x00);
err_input_exit:
    ap3216c_input_exit(dev);
err_sysfs_remove:
    sysfs_remove_group(&dev->dev->kobj, &ap3216c_attr_group);
err_device_destroy:
    device_destroy(dev->class, dev->devno);
err_class_destroy:
    class_destroy(dev->class);
err_cdev_del:
    cdev_del(&dev->cdev);
err_unreg_chrdev:
    unregister_chrdev_region(dev->devno, AP3216C_MYSELF_CNT);
err_free_dev:
    kfree(dev);

    return ret;
}

static int ap3216c_remove(struct i2c_client *client)
{
    struct ap3216c_dev *dev = i2c_get_clientdata(client);

    if (!dev)
        return 0;

    ap3216c_info(dev->dev, "ap3216c_myself removing...\n");

    /* 取消调试工作队列 */
    cancel_delayed_work_sync(&dev->debug_work);

    cancel_delayed_work_sync(&dev->irq_poller);

    /* 释放中断 */
    if (dev->irq > 0) {
        free_irq(dev->irq, dev);
        dev->irq = 0;
    }

    /* 取消工作队列 */
    cancel_work_sync(&dev->work);

    /* 关闭硬件 */
    ap3216c_i2c_write(dev, AP3216C_SYSTEMCONG, 0x00);

    /* 清理资源 */
    ap3216c_input_exit(dev);
    sysfs_remove_group(&dev->dev->kobj, &ap3216c_attr_group);
    device_destroy(dev->class, dev->devno);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devno, AP3216C_MYSELF_CNT);
    mutex_destroy(&dev->lock);
    kfree(dev);

    ap3216c_device = NULL;

    return 0;
}

static const struct i2c_device_id ap3216c_id[] = {
    {"ap3216c_myself", 0},
    {"alientek,ap3216c", 0},
    {"ap3216c", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, ap3216c_id);

static const struct of_device_id ap3216c_of_match[] = {
    {.compatible = "ap3216c_myself"},
    {.compatible = "alientek,ap3216c"},
    {.compatible = "ap3216c"},
    {},
};
MODULE_DEVICE_TABLE(of, ap3216c_of_match);

static struct i2c_driver ap3216c_driver = {
    .probe = ap3216c_probe,
    .remove = ap3216c_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = AP3216C_MYSELF_NAME,
        .of_match_table = ap3216c_of_match,
    },
    .id_table = ap3216c_id,
};

module_i2c_driver(ap3216c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("IMX6ULL Developer");
MODULE_DESCRIPTION("AP3216C Sensor Driver - Top-Half IRQ + Bottom-Half Workqueue");
MODULE_VERSION("2.0");
