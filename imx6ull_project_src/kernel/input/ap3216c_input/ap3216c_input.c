/*
 * AP3216C Proximity/ALS Sensor Driver for IMX6ULL - ap3216c_input
 *
 * Phase 4 迁移版（M1 最小版）：走 Linux Input 子系统 + sysfs raw。
 *
 * 底本：imx6ull_project_src/kernel/baseline/ap3216c_myself/ap3216c_myself.c
 *       （已实测调通的自研模块）
 * 蓝本：reference 指向的 王政 AP3216C-Linux-Driver（Input + runtime PM）
 *
 * 架构：顶半部 IRQ Handler（仅 schedule_work）+ 底半部 Workqueue（I2C 慢速传输）
 *
 * 相对底本的改动（原因见 plan/phase4_sensor_framework.md）：
 *   1. 删除整块字符设备接口（fops / alloc_chrdev_region / cdev_* / class_create /
 *      device_create / /dev/ap3216c_myself）—— 新模块数据出口只有 Input 事件与
 *      sysfs raw。M2 会在此基础上补 runtime PM 与抗抖动。
 *   2. sysfs 属性组改挂到 i2c client 设备（&client->dev.kobj），分两个组：
 *        proximity/     ps_raw_data(RO)、prox_counts(RO)
 *        ambient_light/ als_raw_data(RO)
 *      取代底本自定义 class 下的扁平属性。
 *   3. 去掉全局单例 ap3216c_device，统一走 i2c_set_clientdata/i2c_get_clientdata，
 *      sysfs 回调用 dev_get_drvdata（attributes 挂在 client->dev 上）。
 *   4. 改名 ap3216c_myself → ap3216c_input；Input 设备名保持 "ap3216c-proximity"。
 *
 * 注：底本 sysfs 的 ps 用 ((h&0x3F)<<8)|l 是 14 位写法，与其自身底半部
 *     ((h&0x3F)<<4)|(l&0x0F) 的 10 位写法不一致。AP3216C PS 数据实为 10 位
 *     （PSDATAHIGH[5:0]=bit9:4，PSDATALOW[3:0]=bit3:0），故这里统一取 10 位，
 *     与旧模块 /dev/ap3216c_myself 读回的 data.ps 语义一致，便于 A/B 对照。
 *
 * M2 增补（runtime PM + 抗抖动 + sysfs 校准节点）：
 *   A. runtime PM：suspend -> MODE_PSONLY(0x02)，resume -> MODE_PSALS(0x03)。
 *      **故意与蓝本相反**：蓝本 runtime_suspend 写 PSALS(全功能)、runtime_resume
 *      写 PSONLY(省电)，命名与语义颠倒，照抄会写出没人看得懂的驱动。
 *      这里"挂起"就是真的省电：PS 保持测量（下次靠近仍能产生中断，不会失联），
 *      关掉耗电的 ALS。idle 500ms 后自动挂起。
 *   B. 因为挂起时 ALS 是关的，sysfs 读 ALS 前取一次 PM 引用（get_sync /
 *      put_autosuspend），读数永远有效——蓝本"挂起期间 ALS 失效"这条约束
 *      在本驱动里不存在。
 *   C. 抗抖动有两层：软件阈值死区（low~high 之间保持上次状态）+ 芯片级
 *      HYSTERESIS 模式（ps_int_mode=1，**默认值**）。取 1 不取 0 的理由是实测
 *      中断风暴，写在下面 ps_int_mode 定义处；ZONE(0) 保留为回退路径。
 *   D. 系统级 suspend/resume 用 pm_runtime_force_suspend/resume（4.1.15 已有），
 *      配合 disable_irq + MODE_PWDOWN，恢复后补报一次 FAR 复位状态机。
 *
 * 开发：2026-10-07（Phase 4 M1 / M2）
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "ap3216creg.h"

/*
 * 默认阈值（模块参数与 fallback 共用一份，避免两处不一致）。
 * 判据（软件比较，与芯片 PS_INT_FORM 模式无关）：
 *   PS > high -> NEAR   PS < low -> FAR   二者之间 -> 保持上次状态（死区=抗抖动）
 *
 * 取值依据（2026-10-07 实机测量）：
 *   无遮挡 PS ≈ 0~7      手掌贴近 PS ≈ 580~910
 * 取 20/80 在噪声之上、贴近值之下，两侧都留有充足余量。
 */
#define DEFAULT_PS_THRESHOLD_LOW    20
#define DEFAULT_PS_THRESHOLD_HIGH   80

/* runtime PM：空闲多久后允许自动挂起（ms） */
#define AP3216C_AUTOSUSPEND_DELAY_MS   500

/*
 * GPIO 中断异常时仅用于调试：轮询 INTSTATUS（毫秒，0=关闭）。
 * insmod ap3216c_input.ko irq_poll_ms=50
 */
static int irq_poll_ms;
module_param(irq_poll_ms, int, 0644);
MODULE_PARM_DESC(irq_poll_ms,
    "Poll INTSTATUS interval in ms when GPIO IRQ does not fire (0=disable)");

/*
 * PS 中断形式寄存器 0x22（须显式配置，否则 PS_INT 可能永不置位）:
 * 0 = ZONE — 测量值在 [low,high] 之外触发（AP3216_WE INT_MODE_ZONE）
 * 1 = HYSTERESIS — 只在上/下阈穿越时触发（AP3216_WE INT_MODE_HYSTERESIS）
 *
 * 默认取 1（2026-10-07 实机对比，见 logs/phase4/m2_irq_storm_compare.log）：
 *   ZONE 模式下 PS 恒低于低阈值，芯片**每个测量周期都报一次中断**，
 *   空闲 6 秒发出 13 次中断，每次都把设备从 autosuspend 拽回 active，
 *   运行时省电完全失效；
 *   HYSTERESIS 模式下空闲 6 秒 **0 次中断**，设备稳定停在 suspended。
 * 若某块板子用 HYSTERESIS 检测不到靠近，可用 ps_int_mode=0 回退（M1 验收
 * 已实测 ZONE 模式检测正常，两个方向都能收到事件）。
 */
static int ps_int_mode = 1;
module_param(ps_int_mode, int, 0644);
MODULE_PARM_DESC(ps_int_mode,
    "PS interrupt form reg 0x22: 0=ZONE outside band, 1=HYSTERESIS (default 1)");

/* 周期性打印 PS/INTSTATUS 的间隔(ms)，0=关闭（避免误用读清中断方式时干扰） */
static int debug_interval_ms;
module_param(debug_interval_ms, int, 0644);
MODULE_PARM_DESC(debug_interval_ms,
    "Periodic debug log interval ms (0=disable; reads PS regs each time)");

/* PS 阈值（可由 insmod 传入；运行时也可通过 sysfs proximity/threshold_* 改） */
static int ps_threshold_low = DEFAULT_PS_THRESHOLD_LOW;
module_param(ps_threshold_low, int, 0644);
MODULE_PARM_DESC(ps_threshold_low,
    "PS low/away threshold (default 20)");

static int ps_threshold_high = DEFAULT_PS_THRESHOLD_HIGH;
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
#define AP3216C_INPUT_DRV_NAME  "ap3216c_input"
#define AP3216C_INPUT_NAME      "ap3216c-proximity"

/* 调试开关 */
#define AP3216C_DEBUG           1
#if AP3216C_DEBUG
#define ap3216c_dbg(dev, fmt, ...) dev_info(dev, fmt, ##__VA_ARGS__)
#else
#define ap3216c_dbg(dev, fmt, ...) do {} while (0)
#endif

#define ap3216c_warn(dev, fmt, ...) dev_warn(dev, "ap3216c_input: " fmt, ##__VA_ARGS__)
#define ap3216c_err(dev, fmt, ...) dev_err(dev, "ap3216c_input: " fmt, ##__VA_ARGS__)
#define ap3216c_info(dev, fmt, ...) dev_info(dev, "ap3216c_input: " fmt, ##__VA_ARGS__)

/* 复位等待时间 */
#define AP3216C_RESET_DELAY_MS   50

/* 传感器数据结构 */
struct ap3216c_data {
    u16 ir;
    u16 als;
    u16 ps;
};

/* AP3216C 设备结构 */
struct ap3216c_dev {
    struct i2c_client *client;
    struct input_dev *input;
    struct mutex lock;

    /* 顶半部 + 底半部中断处理 */
    struct work_struct work;          /* 底半部工作队列 */
    struct delayed_work debug_work;   /* 调试用定时器 */
    struct delayed_work irq_poller;   /* 可选：INTSTATUS 轮询（irq_poll_ms>0） */
    int irq;

    /* 阈值配置 */
    int ps_threshold_low;
    int ps_threshold_high;

    /* 上一次上报的接近状态（阈值带内时保持不变） */
    int prox_state;

    /* PS LED 驱动电流配置字节 (reg 0x21)，sysfs led_current_ratio 可改 */
    u8 ps_led_config;

    /* PS 测量间隔 wait 值 (reg 0x24)，实际间隔 = 225*wait + 212 ms */
    int ps_measure_wait;

    /* 传感器数据缓存 */
    struct ap3216c_data data;

    /* 统计 */
    atomic_t near_count;
    atomic_t far_count;
    atomic_t irq_count;

    bool hw_initialized;
};

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
        ap3216c_err(&dev->client->dev, "I2C read failed at reg 0x%02x\n", reg);
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
        ap3216c_err(&dev->client->dev, "I2C write failed at reg 0x%02x\n", reg);
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

/*
 * PS/ALS 读取。
 *
 * PS 是 10 位：PSDATAHIGH[5:0] 是 bit9:4，PSDATALOW[3:0] 是 bit3:0。
 * 这里必须对低字节取 &0x0F：PSDATALOW 的 bit6 是"数据无效"标志、bit7 是
 * 滞回模式下的状态位，不屏蔽就会混进结果（原 sysfs 版本正是漏了这一步，
 * 与底半部的 (low & 0x0F) 算法不一致）。统一走这两个函数，避免再分叉。
 *
 * 返回值：>=0 为 10 位 PS 值；-EINVAL 表示本帧数据无效；其他负值为 I2C 错误。
 */
static int ap3216c_read_ps(struct ap3216c_dev *dev)
{
    int low = ap3216c_i2c_read(dev, AP3216C_PSDATALOW);
    int high;

    if (low < 0)
        return low;
    if (low & 0x40)
        return -EINVAL;          /* PSDATALOW bit6: 数据无效 */
    high = ap3216c_i2c_read(dev, AP3216C_PSDATAHIGH);
    if (high < 0)
        return high;

    return ((high & 0x3F) << 4) | (low & 0x0F);
}

static int ap3216c_read_als(struct ap3216c_dev *dev)
{
    int low = ap3216c_i2c_read(dev, AP3216C_ALSDATALOW);
    int high;

    if (low < 0)
        return low;
    high = ap3216c_i2c_read(dev, AP3216C_ALSDATAHIGH);
    if (high < 0)
        return high;

    return (high << 8) | low;
}

/*
 * 阈值写入：10 位值拆成低 2 位（*L）与高 8 位（*H）。
 * hw_init 与 sysfs 的 threshold_* 写回调共用，避免两处各写一遍写错。
 */
static int ap3216c_write_thresholds(struct ap3216c_dev *dev)
{
    int ret;

    ret  = ap3216c_i2c_write(dev, AP3216C_PSLTHDL, dev->ps_threshold_low & 0x03);
    ret |= ap3216c_i2c_write(dev, AP3216C_PSLTHDH,
                             (dev->ps_threshold_low >> 2) & 0xFF);
    ret |= ap3216c_i2c_write(dev, AP3216C_PSHTHDL, dev->ps_threshold_high & 0x03);
    ret |= ap3216c_i2c_write(dev, AP3216C_PSHTHDH,
                             (dev->ps_threshold_high >> 2) & 0xFF);
    return ret;
}

/* PSLEDWT 的 wait 值与实际测量间隔的换算：interval ≈ 225*wait + 212 (ms) */
static int ap3216c_wait_to_ms(int wait)
{
    return 225 * wait + 212;
}

static int ap3216c_ms_to_wait(int ms)
{
    int wait = (ms - 212 + 112) / 225;   /* +112 为四舍五入 */

    if (wait < 1)
        wait = 1;
    if (wait > 0x1F)
        wait = 0x1F;
    return wait;
}

static int ap3216c_write_measure_interval(struct ap3216c_dev *dev, int wait)
{
    return ap3216c_i2c_write(dev, AP3216C_PSLEDWT, wait & 0x1F);
}

/* 硬件初始化 */
static int ap3216c_hw_init(struct ap3216c_dev *dev)
{
    struct device *ddev = &dev->client->dev;
    int ret;

    ap3216c_info(ddev, "Initializing AP3216C hardware...\n");

    /* 1. 软件复位 */
    ret = ap3216c_i2c_write(dev, AP3216C_SYSTEMCONG, 0x04);
    if (ret < 0) {
        ap3216c_err(ddev, "Software reset failed\n");
        return ret;
    }
    mdelay(AP3216C_RESET_DELAY_MS);

    /* 2. 配置为 PS + ALS 模式 */
    ret = ap3216c_i2c_write(dev, AP3216C_SYSTEMCONG, AP3216C_MODE_ALS_PS_IR);
    if (ret < 0) {
        ap3216c_err(ddev, "Mode config failed\n");
        return ret;
    }

    /* 3. 配置中断清除模式为软件手动清除 (与 rEDTii 驱动一致) */
    ret = ap3216c_i2c_write(dev, AP3216C_INTCLEAR, 0x01);
    if (ret < 0) {
        ap3216c_err(ddev, "Interrupt clear mode config failed\n");
        return ret;
    }

    /* 4. 清除可能存在的中断标志 */
    ret = ap3216c_i2c_write(dev, AP3216C_INTSTATUS, 0x02);
    if (ret < 0) {
        ap3216c_err(ddev, "Clear interrupt flag failed\n");
        return ret;
    }

    /* 5. 配置LED驱动电流 (关键！与 rEDTii 驱动一致) */
    ret = ap3216c_i2c_write(dev, AP3216C_PSLEDCONFIG, dev->ps_led_config);
    if (ret < 0) {
        ap3216c_err(ddev, "PS LED config failed\n");
        return ret;
    }

    /* 6. 配置阈值（10 位：低 2 位在 *L，高 8 位在 *H） */
    ret = ap3216c_write_thresholds(dev);
    if (ret < 0) {
        ap3216c_err(ddev, "Threshold config failed\n");
        return ret;
    }

    /* 7. PS 测量间隔 (PSLEDWT reg 0x24)，由模块参数 ps_measure_interval 决定 */
    ret = ap3216c_write_measure_interval(dev, dev->ps_measure_wait);
    if (ret < 0) {
        ap3216c_err(ddev, "PS measure interval failed\n");
        return ret;
    }
    ap3216c_info(ddev, "REG 0x24 PSLEDWT: 0x%02x (interval ~%d ms)\n",
                 dev->ps_measure_wait,
                 ap3216c_wait_to_ms(dev->ps_measure_wait));

    /*
     * 8. PS 中断算法 (reg 0x22) — Wolles/AP3216_WE: ZONE=0x00, HYSTERESIS=0x01。
     * 未配置时部分芯片 PS_INT 永不置位，INT 引脚无脉冲，GPIO 下降沿为 0。
     */
    {
        u8 form = (ps_int_mode != 0) ? AP3216C_PS_INT_HYSTERESIS
                                     : AP3216C_PS_INT_ZONE;

        ret = ap3216c_i2c_write(dev, AP3216C_PS_INT_FORM, form);
        if (ret < 0) {
            ap3216c_err(ddev, "PS interrupt form (0x22) failed\n");
            return ret;
        }
    }

    /* 9. 显式使能 PS + PS 中断 (AP3216C_PSCONFIG_DEFAULT 0x05)，写在阈值/模式之后 */
    ret = ap3216c_i2c_write(dev, AP3216C_PSCONFIG, AP3216C_PSCONFIG_DEFAULT);
    if (ret < 0) {
        ap3216c_err(ddev, "PSCONFIG write failed\n");
        return ret;
    }

    /* 配置完成后再次清中断，确保 INT 引脚释放（开漏 + 内部上拉 → 高） */
    ap3216c_flush_interrupt_state(dev);

    /* 调试: 读取并打印寄存器值以验证配置 */
    ap3216c_info(ddev, "=== AP3216C Register Dump ===\n");
    ap3216c_info(ddev, "REG 0x00 SYSTEM: 0x%02x\n",
                 ap3216c_i2c_read(dev, AP3216C_SYSTEMCONG));
    ap3216c_info(ddev, "REG 0x01 INTSTATUS: 0x%02x\n",
                 ap3216c_i2c_read(dev, AP3216C_INTSTATUS));
    ap3216c_info(ddev, "REG 0x02 INTCLR_MODE: 0x%02x\n",
                 ap3216c_i2c_read(dev, AP3216C_INTCLEAR));
    ap3216c_info(ddev, "REG 0x22 PS_INT_FORM: 0x%02x (mode param %d)\n",
                 ap3216c_i2c_read(dev, AP3216C_PS_INT_FORM), ps_int_mode);
    ap3216c_info(ddev, "REG 0x20 PSCONFIG: 0x%02x\n",
                 ap3216c_i2c_read(dev, AP3216C_PSCONFIG));
    ap3216c_info(ddev, "REG 0x21 LEDCONFIG: 0x%02x\n",
                 ap3216c_i2c_read(dev, AP3216C_PSLEDCONFIG));
    ap3216c_info(ddev, "REG 0x2A-0x2D Threshold: L=%d H=%d\n",
                 dev->ps_threshold_low, dev->ps_threshold_high);
    ap3216c_info(ddev, "================================\n");

    dev->hw_initialized = true;
    ap3216c_info(ddev,
                 "AP3216C initialized (thr low=%d high=%d, ps_int_mode=%d ZONE/HYS)\n",
                 dev->ps_threshold_low, dev->ps_threshold_high, ps_int_mode);

    return 0;
}

/* ================================================================
 * 顶半部 IRQ Handler + 底半部 Workqueue
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

    printk(KERN_INFO "ap3216c_input DEBUG: INT_STATUS=0x%02x PS=%d (bit7=%d) state=%s\n",
           int_status,
           ((ps_data_h & 0x3F) << 4) | (ps_data_l & 0x0F),
           (ps_data_l >> 7) & 0x01,
           dev->prox_state ? "NEAR" : "FAR");

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
        dev_info(&dev->client->dev,
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
    printk(KERN_INFO "ap3216c_input IRQ triggered! count=%d\n",
           atomic_read(&dev->irq_count));

    /* 调度底半部处理 */
    schedule_work(&dev->work);

    return IRQ_HANDLED;
}

/* 底半部: 在工作队列上下文执行 I2C 访问 */
static void ap3216c_work_callback(struct work_struct *work)
{
    struct ap3216c_dev *dev = container_of(work, struct ap3216c_dev, work);
    struct device *ddev = &dev->client->dev;
    int int_status;
    int ps;
    int proximity_state;

    /*
     * 先取一次 runtime PM 引用：挂起态下芯片只跑 PS（ALS 关闭），
     * 这里要读寄存器，必须确保芯片在工作态。get_sync 会触发 runtime_resume
     * （写回 MODE_PSALS）。注意 get_sync 失败时**不会**增加引用计数，
     * 因此必须手动 put_noidle 配平，否则引用计数泄漏、设备永不再挂起。
     */
    if (pm_runtime_get_sync(ddev) < 0) {
        pm_runtime_put_noidle(ddev);
        return;
    }

    /* 获取互斥锁 */
    if (mutex_lock_interruptible(&dev->lock) != 0)
        goto out_pm;

    /* 读取中断状态寄存器 */
    int_status = ap3216c_i2c_read(dev, AP3216C_INTSTATUS);

    /* 只处理 PS 中断 (bit 1 = 0x02) */
    if (!(int_status & 0x02)) {
        ap3216c_dbg(ddev, "Not PS interrupt, int_status=0x%02x\n", int_status);
        goto out_unlock;
    }

    /* 清除 PS 中断标志 (写 0x02 清除) */
    ap3216c_i2c_write(dev, AP3216C_INTSTATUS, 0x02);

    /* 读取 PS 数据（含 bit6 有效性检查） */
    ps = ap3216c_read_ps(dev);
    if (ps < 0) {
        ap3216c_dbg(ddev, "PS data invalid or read error: %d\n", ps);
        goto out_unlock;
    }
    dev->data.ps = ps;

    /*
     * near/far 由软件比较 PS 与阈值判定，不用 PSDATA bit7。
     *
     * 原因（2026-10-07 实测）：底本用 bit7，但那个位只在滞回模式下才有意义，
     * 且当时驱动跑的是 ZONE 模式，bit7 恒为 0 —— 结果 PS 已涨到 579
     * 仍报 FAR，Input 事件永不产生（EV_SW 在 4.1.15 只投递状态变化，
     * 状态不变就没有事件）。
     *
     * 软件比较的好处是**与芯片的中断模式无关**：无论 ps_int_mode 取 ZONE 还是
     * HYSTERESIS（默认已改为后者），判定逻辑都成立。阈值带内保持上次状态，
     * 这就是本驱动的软件抗抖动层。
     */
    if (dev->data.ps > dev->ps_threshold_high)
        proximity_state = 1;
    else if (dev->data.ps < dev->ps_threshold_low)
        proximity_state = 0;
    else
        proximity_state = dev->prox_state;
    dev->prox_state = proximity_state;

    /* 更新统计计数 */
    if (proximity_state)
        atomic_inc(&dev->near_count);
    else
        atomic_inc(&dev->far_count);

    /* 上报 Input 事件 */
    input_report_switch(dev->input, SW_FRONT_PROXIMITY, proximity_state);
    input_sync(dev->input);

    ap3216c_info(ddev,
                 "proximity: %s PS=%d (thr %d/%d, mode=%s, near=%d, far=%d)\n",
                 proximity_state ? "NEAR" : "FAR", dev->data.ps,
                 dev->ps_threshold_low, dev->ps_threshold_high,
                 ps_int_mode ? "HYS" : "ZONE",
                 atomic_read(&dev->near_count),
                 atomic_read(&dev->far_count));

out_unlock:
    mutex_unlock(&dev->lock);
out_pm:
    /* 记一次 busy 并交还引用，delay 到期后自动挂起 */
    pm_runtime_mark_last_busy(ddev);
    pm_runtime_put_autosuspend(ddev);
}

/* ================================================================
 * sysfs 属性接口（挂在 i2c client 设备上，两个属性组）
 * ================================================================ */

/*
 * sysfs 访问前先取一次 runtime PM 引用。
 * 挂起态下芯片只跑 PS、ALS 是关的，不 resume 就读不到有效数据；
 * 这样用户什么时候 cat 都是对的，"挂起期间 ALS 失效"这条约束在本驱动里不存在。
 */
static int ap3216c_pm_get(struct ap3216c_dev *dev)
{
    if (pm_runtime_get_sync(&dev->client->dev) < 0) {
        pm_runtime_put_noidle(&dev->client->dev);
        return -EIO;
    }
    return 0;
}

static void ap3216c_pm_put(struct ap3216c_dev *dev)
{
    pm_runtime_mark_last_busy(&dev->client->dev);
    pm_runtime_put_autosuspend(&dev->client->dev);
}

static ssize_t ps_raw_data_show(struct device *dev_struct,
                                struct device_attribute *attr, char *buf)
{
    struct ap3216c_dev *dev = dev_get_drvdata(dev_struct);
    int ps;

    if (!dev)
        return -ENODEV;

    if (ap3216c_pm_get(dev))
        return -EIO;

    mutex_lock(&dev->lock);
    ps = ap3216c_read_ps(dev);
    mutex_unlock(&dev->lock);

    ap3216c_pm_put(dev);

    if (ps < 0)
        return ps;

    return scnprintf(buf, PAGE_SIZE, "%d\n", ps);
}

static ssize_t prox_counts_show(struct device *dev_struct,
                                struct device_attribute *attr, char *buf)
{
    struct ap3216c_dev *dev = dev_get_drvdata(dev_struct);

    if (!dev)
        return -ENODEV;

    return scnprintf(buf, PAGE_SIZE,
                     "near=%d far=%d irq=%d\n",
                     atomic_read(&dev->near_count),
                     atomic_read(&dev->far_count),
                     atomic_read(&dev->irq_count));
}

/* ---------------- M2 运行时可调参数（校准节点） ---------------- */

/*
 * 阈值校准：写进去立即生效（重写 reg 0x2A~0x2D），不必重新 insmod。
 * 只允许 low < high 且都落在 10 位范围 [0,1023] 内，避免把死区写成负数或倒置。
 */
static ssize_t threshold_low_show(struct device *dev_struct,
                                  struct device_attribute *attr, char *buf)
{
    struct ap3216c_dev *dev = dev_get_drvdata(dev_struct);

    if (!dev)
        return -ENODEV;
    return scnprintf(buf, PAGE_SIZE, "%d\n", dev->ps_threshold_low);
}

static ssize_t threshold_low_store(struct device *dev_struct,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
    struct ap3216c_dev *dev = dev_get_drvdata(dev_struct);
    int val, ret;

    if (!dev)
        return -ENODEV;
    if (kstrtoint(buf, 10, &val) || val < 0 || val > 1023 ||
        val >= dev->ps_threshold_high)
        return -EINVAL;

    if (ap3216c_pm_get(dev))
        return -EIO;

    mutex_lock(&dev->lock);
    dev->ps_threshold_low = val;
    ret = ap3216c_write_thresholds(dev);
    mutex_unlock(&dev->lock);

    ap3216c_pm_put(dev);

    return ret < 0 ? ret : count;
}

static ssize_t threshold_high_show(struct device *dev_struct,
                                   struct device_attribute *attr, char *buf)
{
    struct ap3216c_dev *dev = dev_get_drvdata(dev_struct);

    if (!dev)
        return -ENODEV;
    return scnprintf(buf, PAGE_SIZE, "%d\n", dev->ps_threshold_high);
}

static ssize_t threshold_high_store(struct device *dev_struct,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
    struct ap3216c_dev *dev = dev_get_drvdata(dev_struct);
    int val, ret;

    if (!dev)
        return -ENODEV;
    if (kstrtoint(buf, 10, &val) || val < 0 || val > 1023 ||
        val <= dev->ps_threshold_low)
        return -EINVAL;

    if (ap3216c_pm_get(dev))
        return -EIO;

    mutex_lock(&dev->lock);
    dev->ps_threshold_high = val;
    ret = ap3216c_write_thresholds(dev);
    mutex_unlock(&dev->lock);

    ap3216c_pm_put(dev);

    return ret < 0 ? ret : count;
}

/*
 * PS 测量间隔：对用户暴露**毫秒**（直观），内部换算成 PSLEDWT 的 wait 值
 * （interval ≈ 225*wait + 212 ms，wait 限制在 1~31）。读回的是实际生效值。
 */
static ssize_t measure_interval_show(struct device *dev_struct,
                                     struct device_attribute *attr, char *buf)
{
    struct ap3216c_dev *dev = dev_get_drvdata(dev_struct);

    if (!dev)
        return -ENODEV;
    return scnprintf(buf, PAGE_SIZE, "%d\n",
                     ap3216c_wait_to_ms(dev->ps_measure_wait));
}

static ssize_t measure_interval_store(struct device *dev_struct,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count)
{
    struct ap3216c_dev *dev = dev_get_drvdata(dev_struct);
    int ms, wait, ret;

    if (!dev)
        return -ENODEV;
    if (kstrtoint(buf, 10, &ms) || ms <= 0)
        return -EINVAL;

    wait = ap3216c_ms_to_wait(ms);

    if (ap3216c_pm_get(dev))
        return -EIO;

    mutex_lock(&dev->lock);
    dev->ps_measure_wait = wait;
    ret = ap3216c_write_measure_interval(dev, wait);
    mutex_unlock(&dev->lock);

    ap3216c_pm_put(dev);

    return ret < 0 ? ret : count;
}

/*
 * PS LED 驱动电流比例：PSLEDCONFIG(0x21) 的低 2 位
 *   0=100%  1=50%  2=25%  3=12.5%
 * 保留该寄存器其余位（默认 0x11 → 低 2 位为 1，即 50%），只改比例位。
 */
static ssize_t led_current_ratio_show(struct device *dev_struct,
                                      struct device_attribute *attr, char *buf)
{
    struct ap3216c_dev *dev = dev_get_drvdata(dev_struct);

    if (!dev)
        return -ENODEV;
    return scnprintf(buf, PAGE_SIZE, "%d\n", dev->ps_led_config & 0x03);
}

static ssize_t led_current_ratio_store(struct device *dev_struct,
                                       struct device_attribute *attr,
                                       const char *buf, size_t count)
{
    struct ap3216c_dev *dev = dev_get_drvdata(dev_struct);
    int val, ret;
    u8 cfg;

    if (!dev)
        return -ENODEV;
    if (kstrtoint(buf, 10, &val) || val < 0 || val > 3)
        return -EINVAL;

    if (ap3216c_pm_get(dev))
        return -EIO;

    mutex_lock(&dev->lock);
    cfg = (dev->ps_led_config & ~0x03) | (val & 0x03);
    ret = ap3216c_i2c_write(dev, AP3216C_PSLEDCONFIG, cfg);
    if (ret >= 0)
        dev->ps_led_config = cfg;
    mutex_unlock(&dev->lock);

    ap3216c_pm_put(dev);

    return ret < 0 ? ret : count;
}

static DEVICE_ATTR_RO(ps_raw_data);
static DEVICE_ATTR_RO(prox_counts);
static DEVICE_ATTR_RW(threshold_low);
static DEVICE_ATTR_RW(threshold_high);
static DEVICE_ATTR_RW(measure_interval);
static DEVICE_ATTR_RW(led_current_ratio);

static struct attribute *proximity_attrs[] = {
    &dev_attr_ps_raw_data.attr,
    &dev_attr_prox_counts.attr,
    &dev_attr_threshold_low.attr,
    &dev_attr_threshold_high.attr,
    &dev_attr_measure_interval.attr,
    &dev_attr_led_current_ratio.attr,
    NULL,
};

static const struct attribute_group proximity_attr_group = {
    .name = "proximity",
    .attrs = proximity_attrs,
};

static ssize_t als_raw_data_show(struct device *dev_struct,
                                 struct device_attribute *attr, char *buf)
{
    struct ap3216c_dev *dev = dev_get_drvdata(dev_struct);
    int als;

    if (!dev)
        return -ENODEV;

    if (ap3216c_pm_get(dev))
        return -EIO;

    mutex_lock(&dev->lock);
    als = ap3216c_read_als(dev);
    mutex_unlock(&dev->lock);

    ap3216c_pm_put(dev);

    if (als < 0)
        return als;

    return scnprintf(buf, PAGE_SIZE, "%d\n", als);
}

static DEVICE_ATTR_RO(als_raw_data);

static struct attribute *ambient_light_attrs[] = {
    &dev_attr_als_raw_data.attr,
    NULL,
};

static const struct attribute_group ambient_light_attr_group = {
    .name = "ambient_light",
    .attrs = ambient_light_attrs,
};

static const struct attribute_group *ap3216c_groups[] = {
    &proximity_attr_group,
    &ambient_light_attr_group,
    NULL,
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
    dev->input->phys = "ap3216c_input/input0";
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

    /* 初始化时上报一次远离状态（符合内核 input 约定） */
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

    ap3216c_info(&client->dev, "ap3216c_input probing...\n");

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

    /*
     * 初始化阈值。
     * 底本这里写死 DEFAULT_PS_THRESHOLD_*(50/200)，声明的 ps_threshold_low/high
     * 模块参数从未被读取（死代码）。这里改成真正采用模块参数，非法则回退缺省。
     */
    dev->ps_threshold_low = ps_threshold_low;
    dev->ps_threshold_high = ps_threshold_high;
    if (dev->ps_threshold_low < 0 || dev->ps_threshold_high > 1023 ||
        dev->ps_threshold_low >= dev->ps_threshold_high) {
        ap3216c_warn(&client->dev,
                     "bad thresholds (%d/%d), fallback to %d/%d\n",
                     dev->ps_threshold_low, dev->ps_threshold_high,
                     DEFAULT_PS_THRESHOLD_LOW, DEFAULT_PS_THRESHOLD_HIGH);
        dev->ps_threshold_low = DEFAULT_PS_THRESHOLD_LOW;
        dev->ps_threshold_high = DEFAULT_PS_THRESHOLD_HIGH;
    }
    dev->prox_state = 0;

    /* M2: LED 电流配置与 PS 测量间隔（hw_init 写进芯片；sysfs 可运行时改） */
    dev->ps_led_config = AP3216C_PSLED_DEFAULT;

    /*
     * 注意：模块参数 ps_measure_interval 沿用旧语义 = PSLEDWT 原始 wait 值
     * （0x01~0x1F），不是毫秒。只有 sysfs 的 measure_interval 用毫秒。
     * 不要把参数也按毫秒换算——0x04 若当毫秒会被折算成 wait=1，
     * 默认测量间隔会从 1112ms 悄悄变成 437ms。
     */
    dev->ps_measure_wait = ps_measure_interval;
    if (dev->ps_measure_wait < 1 || dev->ps_measure_wait > 0x1F) {
        ap3216c_warn(&client->dev,
                     "bad ps_measure_interval %d (want 1~31), fallback to 4\n",
                     dev->ps_measure_wait);
        dev->ps_measure_wait = 0x04;
    }

    /* 初始化统计 */
    atomic_set(&dev->near_count, 0);
    atomic_set(&dev->far_count, 0);
    atomic_set(&dev->irq_count, 0);

    dev->hw_initialized = false;

    /* 创建 sysfs 属性（挂在 i2c client 设备上） */
    ret = sysfs_create_groups(&client->dev.kobj, ap3216c_groups);
    if (ret) {
        ap3216c_err(&client->dev, "Failed to create sysfs: %d\n", ret);
        goto err_free_dev;
    }

    /* 初始化 Input 设备 */
    ret = ap3216c_input_init(dev);
    if (ret) {
        ap3216c_err(&client->dev, "Failed to init input device: %d\n", ret);
        goto err_sysfs_remove;
    }

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

    /*
     * runtime PM 收尾。
     *
     * 这里**不能**用 pm_runtime_set_active() + pm_runtime_put_autosuspend()：
     * set_active 不增加使用计数，随后的 put 会把 usage_count 从 0 减到 -1，
     * 而内核 rpm_check_suspend_allowed() 判的是 usage_count != 0 就拒绝挂起，
     * -1 同样不等于 0 —— 结果是 autosuspend 永不触发，设备一直停在 active。
     *
     * 正确做法：把芯片真的置成"挂起态"该有的样子（只剩 PS，ALS 关闭），
     * 再 set_suspended 让 PM 状态与硬件自洽。之后每次 get/put 成对出现，
     * 引用计数从 1 回到 0 才会真正启动 autosuspend 定时器。
     */
    ap3216c_i2c_write(dev, AP3216C_SYSTEMCONG, AP3216C_MODE_PS_ONLY);
    pm_runtime_set_suspended(&client->dev);
    pm_runtime_set_autosuspend_delay(&client->dev,
                                     AP3216C_AUTOSUSPEND_DELAY_MS);
    pm_runtime_use_autosuspend(&client->dev);
    pm_runtime_enable(&client->dev);

    ap3216c_info(&client->dev, "ap3216c_input probed successfully (IRQ: %d)\n",
                 dev->irq);

    return 0;

err_hw_init:
    ap3216c_i2c_write(dev, AP3216C_SYSTEMCONG, 0x00);
err_input_exit:
    ap3216c_input_exit(dev);
err_sysfs_remove:
    sysfs_remove_groups(&client->dev.kobj, ap3216c_groups);
err_free_dev:
    i2c_set_clientdata(client, NULL);
    kfree(dev);

    return ret;
}

static int ap3216c_remove(struct i2c_client *client)
{
    struct ap3216c_dev *dev = i2c_get_clientdata(client);

    if (!dev)
        return 0;

    ap3216c_info(&client->dev, "ap3216c_input removing...\n");

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

    /* 关闭 runtime PM（必须在下面的 cancel/释放之前做，避免残留的 get/put） */
    pm_runtime_disable(&client->dev);

    /* 关闭硬件 */
    ap3216c_i2c_write(dev, AP3216C_SYSTEMCONG, 0x00);

    /* 清理资源 */
    ap3216c_input_exit(dev);
    sysfs_remove_groups(&client->dev.kobj, ap3216c_groups);
    i2c_set_clientdata(client, NULL);
    mutex_destroy(&dev->lock);
    kfree(dev);

    return 0;
}

/* ================================================================
 * runtime PM / 系统睡眠
 * ================================================================ */

/*
 * 挂起 = 省电：只保留 PS 测量，关掉耗电的 ALS。
 * 关键：PS 仍然在测，所以"远处物体靠近"依旧能产生中断把设备唤醒，
 * 不会出现"挂起后再也检测不到靠近"的失联。这与蓝本相反（蓝本把 suspend
 * 写成 MODE_PSALS 全功能、resume 写成 MODE_PSONLY 省电，命名与语义颠倒）。
 */
static int __maybe_unused ap3216c_runtime_suspend(struct device *dev)
{
    struct ap3216c_dev *dev_data = i2c_get_clientdata(to_i2c_client(dev));

    if (!dev_data)
        return 0;

    return ap3216c_i2c_write(dev_data, AP3216C_SYSTEMCONG,
                             AP3216C_MODE_PS_ONLY);
}

/* 恢复 = 全功能 PS+ALS（sysfs 读 ALS 前也是走这里） */
static int __maybe_unused ap3216c_runtime_resume(struct device *dev)
{
    struct ap3216c_dev *dev_data = i2c_get_clientdata(to_i2c_client(dev));

    if (!dev_data)
        return 0;

    return ap3216c_i2c_write(dev_data, AP3216C_SYSTEMCONG,
                             AP3216C_MODE_ALS_PS_IR);
}

/*
 * 系统级睡眠：先经 pm_runtime_force_suspend 把 runtime PM 状态收敛
 * （4.1.15 已提供，是 runtime PM + 系统睡眠共存时的标准写法），
 * 再关中断、进 PWDOWN 真正断电。
 */
static int __maybe_unused ap3216c_suspend(struct device *dev)
{
    struct ap3216c_dev *dev_data = i2c_get_clientdata(to_i2c_client(dev));
    int ret;

    ret = pm_runtime_force_suspend(dev);
    if (ret)
        return ret;

    if (dev_data && dev_data->irq > 0) {
        disable_irq(dev_data->irq);
        ap3216c_i2c_write(dev_data, AP3216C_SYSTEMCONG, AP3216C_MODE_PWDOWN);
    }

    return 0;
}

static int __maybe_unused ap3216c_resume(struct device *dev)
{
    struct ap3216c_dev *dev_data = i2c_get_clientdata(to_i2c_client(dev));

    if (dev_data) {
        ap3216c_i2c_write(dev_data, AP3216C_SYSTEMCONG,
                          AP3216C_MODE_ALS_PS_IR);
        ap3216c_flush_interrupt_state(dev_data);

        /*
         * 睡眠期间手是否在传感器上完全未知，醒来先无条件补报一次 FAR，
         * 让用户态状态机复位——否则可能停在"一直 NEAR"的僵尸状态。
         */
        dev_data->prox_state = 0;
        if (dev_data->input) {
            input_report_switch(dev_data->input, SW_FRONT_PROXIMITY, 0);
            input_sync(dev_data->input);
        }

        if (dev_data->irq > 0)
            enable_irq(dev_data->irq);
    }

    return pm_runtime_force_resume(dev);
}

static const struct dev_pm_ops ap3216c_pm_ops = {
    SET_RUNTIME_PM_OPS(ap3216c_runtime_suspend, ap3216c_runtime_resume, NULL)
    SET_SYSTEM_SLEEP_PM_OPS(ap3216c_suspend, ap3216c_resume)
};

static const struct i2c_device_id ap3216c_id[] = {
    {"ap3216c_input", 0},
    {"alientek,ap3216c", 0},
    {"ap3216c", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, ap3216c_id);

static const struct of_device_id ap3216c_of_match[] = {
    {.compatible = "alientek,ap3216c"},
    {.compatible = "ap3216c_input"},
    {.compatible = "ap3216c"},
    {},
};
MODULE_DEVICE_TABLE(of, ap3216c_of_match);

static struct i2c_driver ap3216c_driver = {
    .probe = ap3216c_probe,
    .remove = ap3216c_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = AP3216C_INPUT_DRV_NAME,
        .of_match_table = ap3216c_of_match,
        .pm = &ap3216c_pm_ops,
    },
    .id_table = ap3216c_id,
};

module_i2c_driver(ap3216c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("IMX6ULL Developer (Phase 4, forked from ap3216c_myself)");
MODULE_DESCRIPTION("AP3216C Proximity/ALS driver - Input + sysfs raw + runtime PM (M2)");
MODULE_VERSION("2.0");
