#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/kfifo.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "mx6hub.h"

static struct mx6hub_dev *g_hub;

/*------------------------------------------------------------
 * 填充事件记录：给 record 打上入队时间戳、来源标识和负载数据。
 * ts_ns 由内核高精度时钟 ktime_get_ns() 产生，是后续 latency_stat
 * 端到端时延测量的起点。
 *------------------------------------------------------------*/
void mx6hub_fill_record(struct mx6hub_record *rec, u32 src, const void *payload, u32 len)
{
    rec->ts_ns = ktime_get_ns();           /* 入队时间（ns），latency 测量基准 */
    rec->source = src;                      /* 事件来源：GPIO / I2C / SPI / HEARTBEAT */
    rec->len = min_t(u32, len, sizeof(rec->payload));
    memset(rec->payload, 0, sizeof(rec->payload));
    if (payload && rec->len)
        memcpy(rec->payload, payload, rec->len);
}

/*------------------------------------------------------------
 * 核心入队函数：所有事件（中断/workqueue/心跳/用户注入）   最终都走这里。
 *
 * 两段锁策略：
 *   1. fifo_lock 保护 kfifo 本身，操作必须原子。
 *   2. stats_lock 保护计数器，enqueue 成功则 total++ 并更新水位线；
 *      失败（-ENOSPC）则 dropped++。
 *   3. 两把锁分别加解锁，避免 ISR 路径因持锁时间过长而打乱中断响应。
 *
 * 入队成功后唤醒 rd_waitq 上等待的用户态 read()。
 *------------------------------------------------------------*/
int mx6hub_enqueue_record(struct mx6hub_dev *hub, const struct mx6hub_record *rec)
{
    unsigned long flags;
    int ret = 0;
    u32 fifo_level = 0;

    /* ---- 临界区1：kfifo 操作 ---- */
    spin_lock_irqsave(&hub->fifo_lock, flags);
    /* kfifo_avail 返回剩余可写空间；不够一条记录大小时直接丢包 */
    if (kfifo_avail(&hub->fifo) < sizeof(*rec)) {
        ret = -ENOSPC;                      /* FIFO 满，返回空间不足 */
    } else if (kfifo_in(&hub->fifo, rec, sizeof(*rec)) != sizeof(*rec)) {
        ret = -EIO;                         /* 内核kfifo内部写入失败（理论上不会发生）*/
    }
    fifo_level = kfifo_len(&hub->fifo) / sizeof(struct mx6hub_record);
    spin_unlock_irqrestore(&hub->fifo_lock, flags);

    /* ---- 临界区2：统计计数器更新 ---- */
    spin_lock_irqsave(&hub->stats_lock, flags);
    if (ret) {
        hub->stats.dropped_events++;       /* 丢包计数 +1 */
    } else {
        hub->stats.total_events++;          /* 累计入队成功数 */
        if (fifo_level > hub->diag.fifo_high_wm)
            hub->diag.fifo_high_wm = fifo_level;  /* 更新历史最高水位 */
    }
    spin_unlock_irqrestore(&hub->stats_lock, flags);

    /* 入队成功后唤醒阻塞在 read() 的用户进程 */
    if (!ret)
        wake_up_interruptible(&hub->rd_waitq);

    return ret;
}
EXPORT_SYMBOL_GPL(mx6hub_enqueue_record);

/*------------------------------------------------------------
 * 统计标记函数：分别在 stats_lock 保护下递增对应计数器。
 * 这些函数设计为纯"打点"，由 ISR / workqueue 调用，不做实际业务。
 *------------------------------------------------------------*/
void mx6hub_mark_irq_top(struct mx6hub_dev *hub)
{
    unsigned long flags;

    spin_lock_irqsave(&hub->stats_lock, flags);
    hub->stats.irq_events++;                /* 中断总次数（顶半部触发即计） */
    hub->diag.irq_top_half_events++;       /* 顶半部执行次数 */
    spin_unlock_irqrestore(&hub->stats_lock, flags);
}

void mx6hub_mark_threaded_irq(struct mx6hub_dev *hub)
{
    unsigned long flags;

    spin_lock_irqsave(&hub->stats_lock, flags);
    hub->diag.threaded_irq_events++;        /* 底半部（线程化中断）执行次数 */
    spin_unlock_irqrestore(&hub->stats_lock, flags);
}

void mx6hub_mark_i2c_work(struct mx6hub_dev *hub)
{
    unsigned long flags;

    spin_lock_irqsave(&hub->stats_lock, flags);
    hub->diag.i2c_work_events++;           /* I2C workqueue 入队次数 */
    spin_unlock_irqrestore(&hub->stats_lock, flags);
}

void mx6hub_mark_spi_work(struct mx6hub_dev *hub)
{
    unsigned long flags;

    spin_lock_irqsave(&hub->stats_lock, flags);
    hub->diag.spi_work_events++;           /* SPI workqueue 入队次数 */
    spin_unlock_irqrestore(&hub->stats_lock, flags);
}

/*------------------------------------------------------------
 * 端到端时延记录：record_ts_ns 是事件入队时打的 ktime，
 * 这里用当前时刻减去入队时刻，得到"事件在 FIFO 中等待的时间"。
 * min/max/last/total/samples 均在 stats_lock 临界区内原子更新。
 *
 * 典型值参考：
 *   - 有用户读时：几 μs ~ 几百 μs
 *   - 用户进程阻塞时：取决于用户何时调用 read()
 *------------------------------------------------------------*/
static void mx6hub_note_read_latency(struct mx6hub_dev *hub, u64 record_ts_ns)
{
    unsigned long flags;
    u64 latency_us;

    latency_us = div64_u64(ktime_get_ns() - record_ts_ns, 1000);

    spin_lock_irqsave(&hub->stats_lock, flags);
    hub->diag.latency_last_us = latency_us;
    if (hub->diag.latency_min_us == U64_MAX || latency_us < hub->diag.latency_min_us)
        hub->diag.latency_min_us = latency_us;
    if (latency_us > hub->diag.latency_max_us)
        hub->diag.latency_max_us = latency_us;
    hub->diag.latency_total_us += latency_us;
    hub->diag.latency_samples++;
    spin_unlock_irqrestore(&hub->stats_lock, flags);
}

/*------------------------------------------------------------
 * 快照函数：将驱动内部两套统计结构（stats + diag）完整拷贝到用户态 buf。
 * 调用方持有 ioctl_lock；内部用 stats_lock 确保读取原子性。
 * latency_avg_us = total_us / samples，无需在驱动内预除。
 *------------------------------------------------------------*/
static void mx6hub_snapshot_stats(struct mx6hub_dev *hub, struct mx6hub_stats *out)
{
    unsigned long flags;

    memset(out, 0, sizeof(*out));

    spin_lock_irqsave(&hub->stats_lock, flags);
    *out = hub->stats;                     /* 一次性拷贝通用计数器 */
    out->irq_top_half_events = hub->diag.irq_top_half_events;
    out->threaded_irq_events = hub->diag.threaded_irq_events;
    out->i2c_work_events = hub->diag.i2c_work_events;
    out->spi_work_events = hub->diag.spi_work_events;
    out->fifo_high_wm = hub->diag.fifo_high_wm;
    out->latency_last_us = hub->diag.latency_last_us;
    out->latency_min_us = (hub->diag.latency_min_us == U64_MAX) ? 0 : hub->diag.latency_min_us;
    out->latency_max_us = hub->diag.latency_max_us;
    out->latency_samples = hub->diag.latency_samples;
    if (hub->diag.latency_samples)
        out->latency_avg_us = div64_u64(hub->diag.latency_total_us, hub->diag.latency_samples);
    spin_unlock_irqrestore(&hub->stats_lock, flags);
}

/*------------------------------------------------------------
 * 清零函数：同时清 stats 和 diag，latency_min_us 重置为 U64_MAX
 *（作为"未设置"标识，在第一次采样时会被替换为真实最小值）。
 *------------------------------------------------------------*/
static void mx6hub_clear_stats(struct mx6hub_dev *hub)
{
    unsigned long flags;

    spin_lock_irqsave(&hub->stats_lock, flags);
    memset(&hub->stats, 0, sizeof(hub->stats));
    memset(&hub->diag, 0, sizeof(hub->diag));
    hub->diag.latency_min_us = U64_MAX;
    spin_unlock_irqrestore(&hub->stats_lock, flags);
}

/*------------------------------------------------------------
 * 心跳定时器（1 Hz）：
 *   1. 产生一个以当前 jiffies 毫秒数为负载的心跳事件，写入 FIFO。
 *      用于验证 FIFO 链路是否畅通（用户态可用 ts_ns 间隔推断系统负载）。
 *   2. 当板子未接传感器 IRQ 时，定时器同时调度 I2C/SPI 两个 workqueue，
 *      使轮询采集路径（fallback）保持活跃，保证 bring-up 可以继续。
 *   3. mod_timer 确保定时器在下一次 HZ 周期再次触发。
 *------------------------------------------------------------*/
static void mx6hub_hb_timer_fn(unsigned long data)
{
    struct mx6hub_dev *hub = (struct mx6hub_dev *)data;
    struct mx6hub_record rec;
    u32 payload = jiffies_to_msecs(jiffies);
    unsigned long flags;

    mx6hub_fill_record(&rec, MX6HUB_SRC_HEARTBEAT, &payload, sizeof(payload));
    if (!mx6hub_enqueue_record(hub, &rec)) {
        spin_lock_irqsave(&hub->stats_lock, flags);
        hub->stats.heartbeat_events++;
        spin_unlock_irqrestore(&hub->stats_lock, flags);
    }

    /*
     * Poll fallback path:
     * 当板子 IRQ 尚未接入时，心跳定时器也负责调度 I2C/SPI 轮询 workqueue，
     * 使 AP3216C / ICM20608 的 fallback 数据持续写入 FIFO，保证链路可验证。
     */
    mx6hub_i2c_schedule(hub);
    mx6hub_spi_schedule(hub);

    mod_timer(&hub->hb_timer, jiffies + HZ);
}

/*------------------------------------------------------------
 * 中断顶半部：仅标记计数后立刻返回 IRQ_WAKE_THREAD。
 * 真正的 GPIO 电平采样、I2C/SPI 读写均延后到线程化底半部执行。
 *------------------------------------------------------------*/
static irqreturn_t mx6hub_irq_top(int irq, void *data)
{
    struct mx6hub_dev *hub = data;

    mx6hub_mark_irq_top(hub);
    return IRQ_WAKE_THREAD;
}

/*------------------------------------------------------------
 * 中断底半部（线程化）：
 *   1. 读取 GPIO 电平并写入 FIFO（GPIO 边沿触发，记录当前电平值）。
 *   2. I2C/SPI 的实际读写（可能 sleep）不在 ISR 内执行，
 *      改为调度 workqueue：mx6hub_i2c_schedule / mx6hub_spi_schedule。
 *------------------------------------------------------------*/
static irqreturn_t mx6hub_irq_thread(int irq, void *data)
{
    struct mx6hub_dev *hub = data;
    struct mx6hub_record rec;
    u32 level = 0xdeadbeef;              /* 未接 GPIO 时的默认值，便于识别无效数据 */

    mx6hub_mark_threaded_irq(hub);

    if (hub->irq_gpiod)
        level = gpiod_get_value(hub->irq_gpiod);

    mx6hub_fill_record(&rec, MX6HUB_SRC_IRQ_GPIO, &level, sizeof(level));
    mx6hub_enqueue_record(hub, &rec);

    /* I2C/SPI 硬件读写均在 workqueue（进程上下文）中执行，避免 ISR 阻塞 */
    mx6hub_i2c_schedule(hub);
    mx6hub_spi_schedule(hub);

    return IRQ_HANDLED;
}

/*------------------------------------------------------------
 * 字符设备 open：container_of 从 inode 的 cdev 成员反推出 mx6hub_dev。
 * private_data 在后续 read/poll/ioctl 中可直接使用。
 *------------------------------------------------------------*/
static int mx6hub_open(struct inode *inode, struct file *filp)
{
    struct mx6hub_dev *hub = container_of(inode->i_cdev, struct mx6hub_dev, cdev);

    filp->private_data = hub;
    return 0;
}

/*------------------------------------------------------------
 * 字符设备 read：
 *   - 阻塞模式：FIFO 空时 wait_event_interruptible 睡眠，直到有数据。
 *   - 非阻塞模式：FIFO 空直接返回 -EAGAIN。
 *   - 有数据时弹出（kfifo_out），计算并记录端到端时延。
 *
 * 关键设计点：
 *   1. spin_lock 只保护 kfifo_out 操作本身，持锁时间极短。
 *   2. latency 在持 stats_lock 下更新。
 *   3. copy_to_user 可能在缺页时 sleep，但此时 FIFO 已被消费，不影响生产者。
 *------------------------------------------------------------*/
static ssize_t mx6hub_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
    struct mx6hub_dev *hub = filp->private_data;
    struct mx6hub_record rec;
    unsigned long flags;

    if (count < sizeof(rec))
        return -EINVAL;

    if (kfifo_is_empty(&hub->fifo)) {
        if (filp->f_flags & O_NONBLOCK)
            return -EAGAIN;                /* 非阻塞：立即返回 */
        if (wait_event_interruptible(hub->rd_waitq, !kfifo_is_empty(&hub->fifo)))
            return -ERESTARTSYS;          /* 信号打断：让系统调用重启 */
    }

    /* 弹出队首记录（持 fifo_lock） */
    spin_lock_irqsave(&hub->fifo_lock, flags);
    if (kfifo_out(&hub->fifo, &rec, sizeof(rec)) != sizeof(rec)) {
        spin_unlock_irqrestore(&hub->fifo_lock, flags);
        return -EAGAIN;
    }
    spin_unlock_irqrestore(&hub->fifo_lock, flags);

    if (copy_to_user(buf, &rec, sizeof(rec)))
        return -EFAULT;

    /* 更新读取计数 + 端到端时延统计（持 stats_lock） */
    spin_lock_irqsave(&hub->stats_lock, flags);
    hub->stats.read_events++;
    spin_unlock_irqrestore(&hub->stats_lock, flags);

    mx6hub_note_read_latency(hub, rec.ts_ns);
    return sizeof(rec);
}

/*------------------------------------------------------------
 * poll：给 epoll/select 提供 fd 可读就绪通知。
 * poll_wait 注册 rd_waitq，seq_file 的 poll hook 会自动处理唤醒逻辑。
 *------------------------------------------------------------*/
static unsigned int mx6hub_poll(struct file *filp, poll_table *wait)
{
    struct mx6hub_dev *hub = filp->private_data;
    unsigned int mask = 0;

    poll_wait(filp, &hub->rd_waitq, wait);
    if (!kfifo_is_empty(&hub->fifo))
        mask |= POLLIN | POLLRDNORM;      /* 告知调用方：可读 */

    return mask;
}

/*------------------------------------------------------------
 * ioctl：支持三个命令。
 *   - GET_STATS：获取完整统计快照（通用 + 诊断），用于用户态监控。
 *   - CLR_STATS：清零所有统计，清 latency_min_us 时用 U64_MAX 标记"未采样"。
 *   - INJECT：用户态注入测试事件，payload 由用户指定，用于链路压测。
 *
 * 所有子命令共享 ioctl_lock，防止并发修改 stats。
 *------------------------------------------------------------*/
static long mx6hub_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct mx6hub_dev *hub = filp->private_data;
    struct mx6hub_stats snapshot;
    struct mx6hub_inject inj;
    struct mx6hub_record rec;
    unsigned long flags;

    if (_IOC_TYPE(cmd) != MX6HUB_IOC_MAGIC)
        return -ENOTTY;

    mutex_lock(&hub->ioctl_lock);

    switch (cmd) {
    case MX6HUB_IOC_GET_STATS:
        mx6hub_snapshot_stats(hub, &snapshot);
        if (copy_to_user((void __user *)arg, &snapshot, sizeof(snapshot))) {
            mutex_unlock(&hub->ioctl_lock);
            return -EFAULT;
        }
        break;
    case MX6HUB_IOC_CLR_STATS:
        mx6hub_clear_stats(hub);
        break;
    case MX6HUB_IOC_INJECT:
        /* 将用户态数据注入 FIFO，用于模拟压测或故障注入 */
        if (copy_from_user(&inj, (void __user *)arg, sizeof(inj))) {
            mutex_unlock(&hub->ioctl_lock);
            return -EFAULT;
        }
        inj.len = min_t(u32, inj.len, sizeof(inj.payload));
        mx6hub_fill_record(&rec, inj.source, inj.payload, inj.len);
        if (!mx6hub_enqueue_record(hub, &rec)) {
            spin_lock_irqsave(&hub->stats_lock, flags);
            hub->stats.user_inject_events++;
            spin_unlock_irqrestore(&hub->stats_lock, flags);
        }
        break;
    default:
        mutex_unlock(&hub->ioctl_lock);
        return -ENOTTY;
    }

    mutex_unlock(&hub->ioctl_lock);
    return 0;
}

static const struct file_operations mx6hub_fops = {
    .owner = THIS_MODULE,
    .open = mx6hub_open,
    .read = mx6hub_read,
    .poll = mx6hub_poll,
    .unlocked_ioctl = mx6hub_ioctl,
    .llseek = no_llseek,
};

/*------------------------------------------------------------
 * debugfs seq_file 回调：每个节点一个 show 函数。
 *
 * fifo_stat：
 *   fifo_lock 读取当前水位，stats_lock 读取 total/dropped。
 *   两个锁分开加解锁，避免在 ISR 路径中因持多把锁导致死锁。
 *------------------------------------------------------------*/
static int mx6hub_fifo_stat_show(struct seq_file *s, void *unused)
{
    struct mx6hub_dev *hub = s->private;
    unsigned long flags;
    u32 fifo_level;
    u64 high_wm;
    u64 dropped;
    u64 total;

    spin_lock_irqsave(&hub->fifo_lock, flags);
    fifo_level = kfifo_len(&hub->fifo) / sizeof(struct mx6hub_record);
    spin_unlock_irqrestore(&hub->fifo_lock, flags);

    spin_lock_irqsave(&hub->stats_lock, flags);
    high_wm = hub->diag.fifo_high_wm;
    dropped = hub->stats.dropped_events;
    total = hub->stats.total_events;
    spin_unlock_irqrestore(&hub->stats_lock, flags);

    seq_printf(s, "fifo_level=%u\n", fifo_level);
    seq_printf(s, "fifo_high_wm=%llu\n", high_wm);
    seq_printf(s, "dropped_events=%llu\n", dropped);
    seq_printf(s, "total_events=%llu\n", total);
    return 0;
}

/*------------------------------------------------------------
 * irq_stat：
 *   计数器部分在 stats_lock 下读取；
 *   I2C/SPI 运行时字段（hw_ready / read_err / whoami 等）使用 READ_ONCE
 *   读取，避免 workqueue 更新时（进程上下文）对齐到非原子 read。
 *   这些字段与 counter 分离，无需 stats_lock 保护。
 *------------------------------------------------------------*/
static int mx6hub_irq_stat_show(struct seq_file *s, void *unused)
{
    struct mx6hub_dev *hub = s->private;
    unsigned long flags;
    bool i2c_hw_ready;
    bool i2c_fallback_active;
    u32 i2c_read_err;
    u32 i2c_retry_cnt;
    u32 i2c_real_events;
    u32 i2c_fallback_events;
    int i2c_last_err;
    bool spi_hw_ready;
    u8 spi_whoami;
    u32 spi_read_err;
    u32 spi_retry_cnt;
    u32 spi_reinit_cnt;
    u32 spi_real_events;
    u32 spi_fallback_events;
    int spi_last_err;

    spin_lock_irqsave(&hub->stats_lock, flags);
    seq_printf(s, "irq_events=%llu\n", hub->stats.irq_events);
    seq_printf(s, "irq_top_half_events=%llu\n", hub->diag.irq_top_half_events);
    seq_printf(s, "threaded_irq_events=%llu\n", hub->diag.threaded_irq_events);
    seq_printf(s, "i2c_work_events=%llu\n", hub->diag.i2c_work_events);
    seq_printf(s, "spi_work_events=%llu\n", hub->diag.spi_work_events);
    spin_unlock_irqrestore(&hub->stats_lock, flags);

    i2c_hw_ready = READ_ONCE(hub->i2c.hw_ready);
    i2c_fallback_active = READ_ONCE(hub->i2c.fallback_active);
    i2c_read_err = READ_ONCE(hub->i2c.read_err);
    i2c_retry_cnt = READ_ONCE(hub->i2c.retry_cnt);
    i2c_real_events = READ_ONCE(hub->i2c.real_events);
    i2c_fallback_events = READ_ONCE(hub->i2c.fallback_events);
    i2c_last_err = READ_ONCE(hub->i2c.last_err);

    /*
     * SPI 运行时字段由 workqueue 更新（进程上下文）。
     * READ_ONCE 防止在弱内存序架构（如 ARM）上读取到撕裂值。
     */
    spi_hw_ready = READ_ONCE(hub->spi.hw_ready);
    spi_whoami = READ_ONCE(hub->spi.whoami);
    spi_read_err = READ_ONCE(hub->spi.read_err);
    spi_retry_cnt = READ_ONCE(hub->spi.retry_cnt);
    spi_reinit_cnt = READ_ONCE(hub->spi.reinit_cnt);
    spi_real_events = READ_ONCE(hub->spi.real_events);
    spi_fallback_events = READ_ONCE(hub->spi.fallback_events);
    spi_last_err = READ_ONCE(hub->spi.last_err);

    seq_printf(s, "i2c_hw_ready=%u\n", i2c_hw_ready ? 1 : 0);
    seq_printf(s, "i2c_fallback_active=%u\n", i2c_fallback_active ? 1 : 0);
    seq_printf(s, "i2c_read_err=%u\n", i2c_read_err);
    seq_printf(s, "i2c_retry_cnt=%u\n", i2c_retry_cnt);
    seq_printf(s, "i2c_real_events=%u\n", i2c_real_events);
    seq_printf(s, "i2c_fallback_events=%u\n", i2c_fallback_events);
    seq_printf(s, "i2c_last_err=%d\n", i2c_last_err);
    seq_printf(s, "spi_hw_ready=%u\n", spi_hw_ready ? 1 : 0);
    seq_printf(s, "spi_whoami=0x%02x\n", spi_whoami);
    seq_printf(s, "spi_read_err=%u\n", spi_read_err);
    seq_printf(s, "spi_retry_cnt=%u\n", spi_retry_cnt);
    seq_printf(s, "spi_reinit_cnt=%u\n", spi_reinit_cnt);
    seq_printf(s, "spi_real_events=%u\n", spi_real_events);
    seq_printf(s, "spi_fallback_events=%u\n", spi_fallback_events);
    seq_printf(s, "spi_last_err=%d\n", spi_last_err);

    return 0;
}

/*------------------------------------------------------------
 * drop_stat：
 *   drop_permille = dropped * 1000 / attempted，单位是‰（千分比），
 *   比百分比精度更高，适合小丢包率场景。
 *------------------------------------------------------------*/
static int mx6hub_drop_stat_show(struct seq_file *s, void *unused)
{
    struct mx6hub_dev *hub = s->private;
    unsigned long flags;
    u64 dropped;
    u64 total;
    u64 attempted;
    u64 drop_permille = 0;

    spin_lock_irqsave(&hub->stats_lock, flags);
    dropped = hub->stats.dropped_events;
    total = hub->stats.total_events;
    spin_unlock_irqrestore(&hub->stats_lock, flags);

    attempted = dropped + total;
    if (attempted)
        drop_permille = div64_u64(dropped * 1000ULL, attempted);

    seq_printf(s, "dropped_events=%llu\n", dropped);
    seq_printf(s, "accepted_events=%llu\n", total);
    seq_printf(s, "drop_permille=%llu\n", drop_permille);
    return 0;
}

/*------------------------------------------------------------
 * latency_stat：
 *   min 初始值为 U64_MAX，首次采样时自动被替换为真实最小值；
 *   avg 在锁内计算（total / samples），避免在输出前再次进入临界区。
 *------------------------------------------------------------*/
static int mx6hub_latency_stat_show(struct seq_file *s, void *unused)
{
    struct mx6hub_dev *hub = s->private;
    unsigned long flags;
    u64 avg = 0;
    u64 min = 0;

    spin_lock_irqsave(&hub->stats_lock, flags);
    if (hub->diag.latency_samples)
        avg = div64_u64(hub->diag.latency_total_us, hub->diag.latency_samples);
    if (hub->diag.latency_min_us != U64_MAX)
        min = hub->diag.latency_min_us;

    seq_printf(s, "latency_last_us=%llu\n", hub->diag.latency_last_us);
    seq_printf(s, "latency_min_us=%llu\n", min);
    seq_printf(s, "latency_max_us=%llu\n", hub->diag.latency_max_us);
    seq_printf(s, "latency_avg_us=%llu\n", avg);
    seq_printf(s, "latency_samples=%llu\n", hub->diag.latency_samples);
    spin_unlock_irqrestore(&hub->stats_lock, flags);

    return 0;
}

/*------------------------------------------------------------
 * debugfs 文件打开回调：
 *   single_open 将 seq_file 与 show 函数绑定，并将 hub（private_data）
 *   通过 inode->i_private 传递给 show 函数。
 *------------------------------------------------------------*/
static int mx6hub_dbg_open_fifo(struct inode *inode, struct file *file)
{
    return single_open(file, mx6hub_fifo_stat_show, inode->i_private);
}

static int mx6hub_dbg_open_irq(struct inode *inode, struct file *file)
{
    return single_open(file, mx6hub_irq_stat_show, inode->i_private);
}

static int mx6hub_dbg_open_drop(struct inode *inode, struct file *file)
{
    return single_open(file, mx6hub_drop_stat_show, inode->i_private);
}

static int mx6hub_dbg_open_latency(struct inode *inode, struct file *file)
{
    return single_open(file, mx6hub_latency_stat_show, inode->i_private);
}

static const struct file_operations mx6hub_fifo_dbg_fops = {
    .owner = THIS_MODULE,
    .open = mx6hub_dbg_open_fifo,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static const struct file_operations mx6hub_irq_dbg_fops = {
    .owner = THIS_MODULE,
    .open = mx6hub_dbg_open_irq,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static const struct file_operations mx6hub_drop_dbg_fops = {
    .owner = THIS_MODULE,
    .open = mx6hub_dbg_open_drop,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static const struct file_operations mx6hub_latency_dbg_fops = {
    .owner = THIS_MODULE,
    .open = mx6hub_dbg_open_latency,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

/*------------------------------------------------------------
 * debugfs 初始化：
 *   debugfs_create_dir 在 /sys/kernel/debug/ 下创建 mx6hub 目录；
 *   debugfs_create_file 创建四个只读（0444）虚拟文件。
 *   所有节点权限 0444：板端任意用户均可 cat，但不可写入。
 *------------------------------------------------------------*/
static void mx6hub_debugfs_init(struct mx6hub_dev *hub)
{
    hub->dbg_dir = debugfs_create_dir("mx6hub", NULL);
    if (IS_ERR_OR_NULL(hub->dbg_dir)) {
        dev_warn(hub->dev, "debugfs dir create failed\n");
        hub->dbg_dir = NULL;
        return;
    }

    debugfs_create_file("fifo_stat", 0444, hub->dbg_dir, hub, &mx6hub_fifo_dbg_fops);
    debugfs_create_file("irq_stat", 0444, hub->dbg_dir, hub, &mx6hub_irq_dbg_fops);
    debugfs_create_file("drop_stat", 0444, hub->dbg_dir, hub, &mx6hub_drop_dbg_fops);
    debugfs_create_file("latency_stat", 0444, hub->dbg_dir, hub, &mx6hub_latency_dbg_fops);
}

/*------------------------------------------------------------
 * debugfs 清理：
 *   debugfs_remove_recursive 递归删除整个 mx6hub 目录及其下所有文件，
 *   防止模块卸载后残留虚拟文件（会导致后续 cat 读到已卸模块的内存）。
 *------------------------------------------------------------*/
static void mx6hub_debugfs_exit(struct mx6hub_dev *hub)
{
    if (hub->dbg_dir)
        debugfs_remove_recursive(hub->dbg_dir);
    hub->dbg_dir = NULL;
}

/*------------------------------------------------------------
 * platform_driver probe：
 *
 * 初始化顺序（严格按依赖关系）：
 *   1. devm_kzalloc      → 分配设备结构
 *   2. init_*             → 初始化 waitqueue、spinlock、mutex
 *   3. kfifo_alloc        → 分配环形缓冲区（1024 条记录）
 *   4. alloc_chrdev_region → 注册字符设备号
 *   5. cdev_add           → 将 cdev 注册到内核
 *   6. class_create/device_create → 创建 /dev/mx6hub0 节点
 *   7. alloc_workqueue    → 创建 workqueue（WQ_UNBOUND + HIGHPRI，保证低延迟调度）
 *   8. mx6hub_i2c_init    → 初始化 I2C 子模块（软绑定 AP3216C）
 *   9. mx6hub_spi_init    → 初始化 SPI 子模块（注册 icm20608 SPI driver）
 *  10. devm_gpiod_get_*   → 获取外部中断 GPIO
 *  11. request_threaded_irq → 注册线程化中断（顶半部 + 底半部）
 *  12. setup_timer/mod_timer → 启动心跳定时器（1 Hz）
 *  13. mx6hub_debugfs_init → 创建 debugfs 节点
 *
 * 错误路径全部向回跳转，逐层释放已申请资源（与初始化顺序相反）。
 *------------------------------------------------------------*/
static int mx6hub_probe(struct platform_device *pdev)
{
    struct mx6hub_dev *hub;
    int ret;

    hub = devm_kzalloc(&pdev->dev, sizeof(*hub), GFP_KERNEL);
    if (!hub)
        return -ENOMEM;

    hub->dev = &pdev->dev;
    init_waitqueue_head(&hub->rd_waitq);
    spin_lock_init(&hub->fifo_lock);
    spin_lock_init(&hub->stats_lock);
    mutex_init(&hub->ioctl_lock);

    mx6hub_clear_stats(hub);

    ret = kfifo_alloc(&hub->fifo, MX6HUB_FIFO_DEPTH * sizeof(struct mx6hub_record), GFP_KERNEL);
    if (ret)
        return ret;

    ret = alloc_chrdev_region(&hub->devt, 0, 1, "mx6hub");
    if (ret)
        goto err_fifo;

    cdev_init(&hub->cdev, &mx6hub_fops);
    hub->cdev.owner = THIS_MODULE;
    ret = cdev_add(&hub->cdev, hub->devt, 1);
    if (ret)
        goto err_chrdev;

    hub->cls = class_create(THIS_MODULE, "mx6hub");
    if (IS_ERR(hub->cls)) {
        ret = PTR_ERR(hub->cls);
        goto err_cdev;
    }

    if (!device_create(hub->cls, NULL, hub->devt, NULL, "mx6hub0")) {
        ret = -ENODEV;
        goto err_class;
    }

    /* WQ_UNBOUND：work 不绑定到特定 CPU，跨核调度更灵活；
       WQ_HIGHPRI：高优先级，确保传感器事件尽快被消费 */
    hub->wq = alloc_workqueue("mx6hub_wq", WQ_UNBOUND | WQ_HIGHPRI, 1);
    if (!hub->wq) {
        ret = -ENOMEM;
        goto err_device;
    }

    ret = mx6hub_i2c_init(hub);
    if (ret)
        goto err_wq;

    ret = mx6hub_spi_init(hub);
    if (ret)
        goto err_i2c;

    /* event GPIO：由板级 DTS 指定，用于传感器边沿触发中断 */
    hub->irq_gpiod = devm_gpiod_get_optional(&pdev->dev, "event", GPIOD_IN);
    if (IS_ERR(hub->irq_gpiod)) {
        ret = PTR_ERR(hub->irq_gpiod);
        dev_warn(&pdev->dev, "event-gpios not ready: %d\n", ret);
        hub->irq_gpiod = NULL;
    }

    if (hub->irq_gpiod) {
        hub->irq = gpiod_to_irq(hub->irq_gpiod);
        if (hub->irq > 0) {
            ret = devm_request_threaded_irq(&pdev->dev, hub->irq,
                                            mx6hub_irq_top, mx6hub_irq_thread,
                                            IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                                            "mx6hub-irq", hub);
            if (ret)
                dev_warn(&pdev->dev, "request irq failed: %d\n", ret);
        }
    }

    setup_timer(&hub->hb_timer, mx6hub_hb_timer_fn, (unsigned long)hub);
    mod_timer(&hub->hb_timer, jiffies + HZ);

    mx6hub_debugfs_init(hub);

    platform_set_drvdata(pdev, hub);
    g_hub = hub;
    dev_info(&pdev->dev, "mx6hub ready, /dev/mx6hub0\n");
    return 0;

err_i2c:
    mx6hub_i2c_exit(hub);
err_wq:
    destroy_workqueue(hub->wq);
err_device:
    device_destroy(hub->cls, hub->devt);
err_class:
    class_destroy(hub->cls);
err_cdev:
    cdev_del(&hub->cdev);
err_chrdev:
    /* 失败回滚必须传设备号本身；传地址会截断 dev_t，可能泄漏设备号。 */
    unregister_chrdev_region(hub->devt, 1);
err_fifo:
    kfifo_free(&hub->fifo);
    return ret;
}

/*------------------------------------------------------------
 * remove：
 *   1. del_timer_sync 等待正在执行的心跳回调完成后再继续
 *      （防止 use-after-free：如果 timer cb 正在引用 hub）。
 *   2. 之后按倒序释放所有资源。
 *------------------------------------------------------------*/
static int mx6hub_remove(struct platform_device *pdev)
{
    struct mx6hub_dev *hub = platform_get_drvdata(pdev);

    del_timer_sync(&hub->hb_timer);
    mx6hub_debugfs_exit(hub);
    mx6hub_spi_exit(hub);
    mx6hub_i2c_exit(hub);
    if (hub->wq)
        destroy_workqueue(hub->wq);
    device_destroy(hub->cls, hub->devt);
    class_destroy(hub->cls);
    cdev_del(&hub->cdev);
    unregister_chrdev_region(hub->devt, 1);
    kfifo_free(&hub->fifo);
    g_hub = NULL;
    dev_info(&pdev->dev, "mx6hub removed\n");
    return 0;
}

static const struct of_device_id mx6hub_of_match[] = {
    { .compatible = "imx6ull,mx6hub" },
    { }
};
MODULE_DEVICE_TABLE(of, mx6hub_of_match);

static struct platform_driver mx6hub_driver = {
    .probe = mx6hub_probe,
    .remove = mx6hub_remove,
    .driver = {
        .name = "mx6hub",
        .of_match_table = of_match_ptr(mx6hub_of_match),
    },
};

static struct platform_device *mx6hub_fallback_pdev;

/*------------------------------------------------------------
 * 模块初始化：
 *   1. 先注册 platform_driver，等待 DTS 中 imx6ull,mx6hub 节点匹配。
 *   2. 若 DTS 尚未加入 mx6hub 节点，则注册一个无 DTS 的 fallback platform_device，
 *      使驱动的 probe 在没有设备树的情况下也能执行，保证 bring-up 可以继续。
 *------------------------------------------------------------*/
static int __init mx6hub_init(void)
{
    struct device_node *np;
    int ret;

    ret = platform_driver_register(&mx6hub_driver);
    if (ret)
        return ret;

    /*
     * 若 DTS 中没有 imx6ull,mx6hub 节点，则注册 fallback 设备；
     * 这样在 DTS 集成前也可以通过 module params 或手动 device_register 触发 probe。
     */
    np = of_find_compatible_node(NULL, NULL, "imx6ull,mx6hub");
    if (!np) {
        mx6hub_fallback_pdev = platform_device_register_simple("mx6hub", -1, NULL, 0);
        if (IS_ERR(mx6hub_fallback_pdev)) {
            ret = PTR_ERR(mx6hub_fallback_pdev);
            mx6hub_fallback_pdev = NULL;
            platform_driver_unregister(&mx6hub_driver);
            return ret;
        }
        pr_info("mx6hub: fallback platform device registered (no DTS node)\n");
    } else {
        of_node_put(np);
    }

    return 0;
}

static void __exit mx6hub_exit(void)
{
    if (mx6hub_fallback_pdev)
        platform_device_unregister(mx6hub_fallback_pdev);
    platform_driver_unregister(&mx6hub_driver);
}

module_init(mx6hub_init);
module_exit(mx6hub_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("imx6ull project");
MODULE_DESCRIPTION("IMX6ULL modular hub driver with threaded irq/workqueue/debugfs");
