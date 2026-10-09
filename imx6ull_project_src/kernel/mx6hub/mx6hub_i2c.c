#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/module.h>

#include "mx6hub.h"

/*------------------------------------------------------------
 * 常量定义
 *
 * MX6HUB_AP3216_I2C_BUS       = 0
 *   IMX6ULL 平台上 AP3216C 挂载的 I2C 总线编号。
 *
 * MX6HUB_AP3216_I2C_ADDR      = 0x1e
 *   AP3216C 在 I2C 总线上的 7-bit 器件地址（ datasheet 规定）。
 *
 * MX6HUB_AP3216_I2C_RETRY_MAX = 3
 *   I2C 总线错误时自动重试次数，避免瞬时总线竞争导致采样失败。
 *
 * MX6HUB_AP3216_I2C_RETRY_DELAY_US = 1500 μs
 *   两次重试之间的等待时间，保证 AP3216C 内部 ADC 转换完成。
 *
 * AP3216C 寄存器（仅本文件使用的子集）：
 *   IRDATALOW (0x0A)    ：IR + ALS + PS 联合数据读取起始地址
 *
 * AP3216C 数据格式说明：
 *   IR  (10 bit)：红外/接近复合传感器，bit7 为无效标志
 *   ALS (16 bit)：环境光传感器的 10bit 有效值（高 8 bit 在 0x0B，低 2 bit 在 0x0A）
 *   PS  (10 bit)：红外接近传感器，高 6 bit 在 0x0D，低 4 bit 在 0x0C
 *------------------------------------------------------------*/
#define MX6HUB_AP3216_I2C_BUS             0
#define MX6HUB_AP3216_I2C_ADDR            0x1e
#define MX6HUB_AP3216_I2C_RETRY_MAX       3
#define MX6HUB_AP3216_I2C_RETRY_DELAY_US  1500

#define AP3216C_IRDATALOW                 0x0A
#define AP3216C_IR_INVALID_BIT            BIT(7)         /* IR 数据无效标志 */
#define AP3216C_IR_LOW_VALID_MASK         0x03           /* IR 低 2 bit 有效 */
#define AP3216C_PS_INVALID_BIT            BIT(6)         /* PS 数据无效标志（物体太近/太远）*/
#define AP3216C_PS_LOW_VALID_MASK         0x0F           /* PS 低 4 bit 有效 */
#define AP3216C_PS_HIGH_VALID_MASK        0x3F           /* PS 高 6 bit 有效 */

/*------------------------------------------------------------
 * AP3216C 读寄存器：
 *   使用 I2C 标准双消息协议：
 *     消息1（写）：寄存器地址（1 B）
 *     消息2（读）：返回数据（len B）
 *
 * 内部最多重试 3 次，每次失败后等待 1.5 ms 再重试。
 * AP3216C 内部 ADC 转换时间约 1 ms，等待可避免总线太快而 AP3216C 尚未完成转换。
 *
 * 为什么不直接用 i2c_smbus_read_i2c_block_data？
 *   AP3216C 的 datasheet 要求先发寄存器地址再连续读，
 *   i2c_transfer 两消息方式更贴近器件规范，兼容性更好。
 *------------------------------------------------------------*/
static int mx6hub_ap3216_read_regs(struct mx6hub_i2c_ctx *ctx, u8 reg, void *val, u8 len)
{
    struct i2c_msg msg[2];
    int ret = -ENODEV;
    int retry;

    if (!ctx->adap || !val || !len)
        return -EINVAL;

    msg[0].addr = MX6HUB_AP3216_I2C_ADDR;
    msg[0].flags = 0;                   /* 写 */
    msg[0].buf = &reg;
    msg[0].len = 1;

    msg[1].addr = MX6HUB_AP3216_I2C_ADDR;
    msg[1].flags = I2C_M_RD;           /* 读 */
    msg[1].buf = val;
    msg[1].len = len;

    for (retry = 0; retry < MX6HUB_AP3216_I2C_RETRY_MAX; retry++) {
        ret = i2c_transfer(ctx->adap, msg, ARRAY_SIZE(msg));
        if (ret == ARRAY_SIZE(msg))
            return 0;                   /* 成功：两消息均完成 */
        ctx->retry_cnt++;
        /* AP3216C ADC 转换约需 1 ms，等待确保下次读时数据已就绪 */
        usleep_range(MX6HUB_AP3216_I2C_RETRY_DELAY_US,
                     MX6HUB_AP3216_I2C_RETRY_DELAY_US + 300);
    }

    return (ret < 0) ? ret : -EREMOTEIO;  /* ret=0 说明总线无响应，返回通信错误 */
}

/*------------------------------------------------------------
 * 读取 AP3216C 三合一数据（IR + ALS + PS）：
 *
 * AP3216C 数据布局（从 0x0A 连续读 6 B）：
 *   buf[0] = IR low 2bit + IR 无效标志
 *   buf[1] = IR high 8bit
 *   buf[2] = ALS low 8bit
 *   buf[3] = ALS high 8bit（通常为 0）
 *   buf[4] = PS low 4bit + PS 无效标志
 *   buf[5] = PS high 6bit
 *
 * 无效数据处理：
 *   - IR bit7=1 → IR 值不可信，强制设为 0
 *   - PS bit6=1 → 物体太近/太远导致测量无效，强制设为 0
 *------------------------------------------------------------*/
static int mx6hub_ap3216_read_payload(struct mx6hub_i2c_ctx *ctx,
                                      struct mx6hub_ap3216_payload *payload)
{
    u8 buf[6];
    int ret;

    ret = mx6hub_ap3216_read_regs(ctx, AP3216C_IRDATALOW, buf, sizeof(buf));
    if (ret)
        return ret;

    /* IR：bit7=1 表示 IR 灯熄灭或强光饱和，无效 */
    if (buf[0] & AP3216C_IR_INVALID_BIT)
        payload->ir = 0;
    else
        payload->ir = ((u16)buf[1] << 2) | (buf[0] & AP3216C_IR_LOW_VALID_MASK);

    /* ALS：16 bit 环境光，bit9~bit2 → [1]=bit9~bit2，[0]=bit1~bit0（实际只有 10 bit 有效）*/
    payload->als = ((u16)buf[3] << 8) | (u16)buf[2];

    /* PS：bit6=1 表示物体超出量程 */
    if (buf[4] & AP3216C_PS_INVALID_BIT)
        payload->ps = 0;
    else
        payload->ps = (((u16)buf[5] & AP3216C_PS_HIGH_VALID_MASK) << 4) |
                      ((u16)buf[4] & AP3216C_PS_LOW_VALID_MASK);

    return 0;
}

/*------------------------------------------------------------
 * Fallback mock 数据：
 *   当 AP3216C 硬件不可用时（adap == NULL 或读失败），
 *   用 jiffies 生成缓慢递增的伪随机数，模拟传感器原始输出。
 *   仅用于驱动 bring-up 验证链路完整性；用户态应通过 hw_ready 字段判断数据来源。
 *------------------------------------------------------------*/
static void mx6hub_i2c_fill_fallback(struct mx6hub_i2c_ctx *ctx,
                                     struct mx6hub_ap3216_payload *payload)
{
    payload->als = (u16)(jiffies & 0x3ff);       /* ALS mock：0 ~ 1023 */
    payload->ps  = (u16)((jiffies >> 2) & 0x3ff); /* PS mock：0 ~ 1023 */
    payload->ir  = (u16)((jiffies >> 4) & 0x3ff);  /* IR mock：0 ~ 1023 */
    payload->seq = (u16)(ctx->seq & 0xffff);
}

/*------------------------------------------------------------
 * I2C workqueue 工作函数（由心跳定时器或中断底半部调度）：
 *
 * 执行顺序：
 *   1. 如果 adap==NULL（I2C 控制器不存在），直接切换 fallback 模式。
 *   2. 尝试读取 AP3216C 数据：
 *        成功 → hw_ready=true，fallback_active=false，real_events++
 *        失败 → hw_ready=false，fallback_active=true，read_err++，进入 fallback
 *   3. fallback 模式下填充 mock 数据，fallback_events++
 *   4. 调用 mx6hub_fill_record + mx6hub_enqueue_record 写入 FIFO
 *
 * 与 SPI 不同，I2C 模块没有 WoM 过滤：每次 workfn 触发都会入队一条记录，
 * 因为 AP3216C 的 IR/ALS/PS 数据变化相对缓慢，重复数据的开销可接受。
 *------------------------------------------------------------*/
static void mx6hub_i2c_workfn(struct work_struct *work)
{
    struct mx6hub_i2c_ctx *ctx = container_of(work, struct mx6hub_i2c_ctx, work);
    struct mx6hub_record rec;
    struct mx6hub_ap3216_payload payload;
    int ret = 0;

    if (!ctx->enabled)
        return;

    mx6hub_mark_i2c_work(ctx->hub);

    memset(&payload, 0, sizeof(payload));
    ctx->seq++;
    payload.seq = (u16)(ctx->seq & 0xffff);

    mutex_lock(&ctx->lock);
    if (ctx->adap) {
        ret = mx6hub_ap3216_read_payload(ctx, &payload);
        if (!ret) {
            ctx->hw_ready = true;
            ctx->fallback_active = false;
            ctx->real_events++;
            ctx->last_err = 0;
        } else {
            ctx->hw_ready = false;
            ctx->fallback_active = true;   /* 读失败，切换 fallback */
            ctx->read_err++;
            ctx->last_err = ret;
            pr_warn_ratelimited("mx6hub: ap3216c read failed (%d), fallback mock\n", ret);
        }
    } else {
        /* I2C 控制器不存在（全板未使能 I2C 总线）*/
        ret = -ENODEV;
        ctx->hw_ready = false;
        ctx->fallback_active = true;
        ctx->last_err = ret;
    }
    mutex_unlock(&ctx->lock);

    if (ret) {
        ctx->fallback_events++;
        mx6hub_i2c_fill_fallback(ctx, &payload);
    }

    mx6hub_fill_record(&rec, MX6HUB_SRC_I2C_AP3216, &payload, sizeof(payload));
    mx6hub_enqueue_record(ctx->hub, &rec);
}

/*------------------------------------------------------------
 * mx6hub_i2c_init：I2C 子模块初始化。
 *
 * 与 SPI 不同，I2C 采用软绑定（soft binding）：
 *   - 直接用 i2c_get_adapter(BUS) 手动获取 I2C 适配器，而非依赖设备树 match。
 *   - 原因：AP3216C 的 I2C 地址固定为 0x1e，板子上只有一个 I2C 适配器（BUS 0），
 *     软绑定比走 SPI 那套 spi_driver/probe 框架更简单直接。
 *
 * 如果适配器获取失败（内核未使能 I2C 控制器），不报错，直接激活 fallback 模式，
 * 整个 I2C 子模块继续以 jiffies mock 方式运行，不影响其他功能。
 *------------------------------------------------------------*/
int mx6hub_i2c_init(struct mx6hub_dev *hub)
{
    hub->i2c.hub = hub;
    hub->i2c.enabled = true;
    hub->i2c.hw_ready = false;
    hub->i2c.seq = 0;
    hub->i2c.read_err = 0;
    hub->i2c.retry_cnt = 0;
    hub->i2c.real_events = 0;
    hub->i2c.fallback_events = 0;
    hub->i2c.fallback_active = false;
    hub->i2c.last_err = 0;
    mutex_init(&hub->i2c.lock);
    hub->i2c.adap = i2c_get_adapter(MX6HUB_AP3216_I2C_BUS);
    INIT_WORK(&hub->i2c.work, mx6hub_i2c_workfn);

    if (!hub->i2c.adap) {
        pr_warn("mx6hub: ap3216c adapter %d unavailable, using mock fallback\n",
                MX6HUB_AP3216_I2C_BUS);
        hub->i2c.fallback_active = true;
        hub->i2c.last_err = -ENODEV;
        return 0;                         /* 不报错，fallback 路径已激活 */
    }

    return 0;
}

/*------------------------------------------------------------
 * mx6hub_i2c_exit：I2C 子模块退出。
 *   - cancel_work_sync 等待 workfn 完成，防止 use-after-free
 *   - 释放 I2C 适配器引用（i2c_put_adapter）
 *   - 标记 hw_ready=false、fallback_active=false
 *------------------------------------------------------------*/
void mx6hub_i2c_exit(struct mx6hub_dev *hub)
{
    cancel_work_sync(&hub->i2c.work);

    mutex_lock(&hub->i2c.lock);
    if (hub->i2c.adap) {
        i2c_put_adapter(hub->i2c.adap);
        hub->i2c.adap = NULL;
    }
    hub->i2c.hw_ready = false;
    hub->i2c.fallback_active = false;
    mutex_unlock(&hub->i2c.lock);
}

/*------------------------------------------------------------
 * I2C workqueue 调度入口：
 *   由 mx6hub_irq_thread（中断触发）或心跳定时器（fallback 轮询）调用。
 *------------------------------------------------------------*/
void mx6hub_i2c_schedule(struct mx6hub_dev *hub)
{
    if (!hub->i2c.enabled)
        return;
    queue_work(hub->wq, &hub->i2c.work);
}
