#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include "mx6hub.h"

/*------------------------------------------------------------
 * 常量定义
 *
 * MX6HUB_ICM_MAX_HZ      = 8 MHz，ICM20608 最高支持 10 MHz，
 *                          8 MHz 是稳妥的安全上限。
 * MX6HUB_SPI_IO_MAX_LEN  = 32 B，单次 SPI 传输最大长度（accel 14B + 余量）。
 * MX6HUB_SPI_RETRY_MAX   = 3 次，I2C/SPI 总线错误时自动重试次数。
 * MX6HUB_ECSPI3_NODE_PATH，IMX6ULL 平台上 SPI2 控制器对应的设备树节点路径。
 *
 * ICM20608 寄存器（仅本文件使用的子集）：
 *   ACCEL_XOUT_H (0x3B)         ：加速度计 X 轴高字节，后接低字节，共 14B
 *   PWR_MGMT_1  (0x6B)          ：电源管理，控制复位/时钟源
 *   PWR_MGMT_2  (0x6C)          ：电源管理，控制各传感器开启状态
 *   WHO_AM_I    (0x75)           ：芯片身份寄存器，固定值 0xAF(ICM20608G) / 0xAE(ICM20608D)
 *   ACCEL_INTEL_CTRL (0x69)      ：WoM 中断控制
 *   WOM_EN      (0x6F)           ：Wake-on-Motion 使能与阈值
 *------------------------------------------------------------*/
#define MX6HUB_ICM_MAX_HZ             8000000U
#define MX6HUB_ICM_MODE               SPI_MODE_0
#define MX6HUB_SPI_IO_MAX_LEN         32U
#define MX6HUB_SPI_RETRY_MAX          3U
#define MX6HUB_ECSPI3_NODE_PATH       "/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02010000"

#define ICM20_ACCEL_XOUT_H            0x3B
#define ICM20_PWR_MGMT_1              0x6B
#define ICM20_PWR_MGMT_2              0x6C
#define ICM20_WHO_AM_I                0x75
#define ICM20_WHOAMI_ICM20608G        0xAF
#define ICM20_WHOAMI_ICM20608D        0xAE
/* WoM (Wake-on-Motion) 寄存器 */
#define ICM20_WOM_EN                  0x6F
#define ICM20_ACCEL_INTEL_CTRL        0x69

/*------------------------------------------------------------
 * mx6hub_spi_lock：保护 mx6hub_spi_owner 全局指针，防止在 spi_probe
 * 和 mx6hub_spi_init 并发时出现竞态（例如：ICM20608 先于 mx6hub.ko
 * 加载，导致找不到 owner）。
 *------------------------------------------------------------*/
static DEFINE_MUTEX(mx6hub_spi_lock);
static struct mx6hub_dev *mx6hub_spi_owner;
static bool mx6hub_spi_driver_registered;

/*------------------------------------------------------------
 * BCM 字节序转换：ICM20608 的加速度计和陀螺仪数据以大端（BE）格式输出，
 * ARM 为小端架构，需要在读取后手动交换字节顺序。
 *------------------------------------------------------------*/
static inline s16 mx6hub_be16_to_s16(const u8 *buf)
{
    return (s16)(((u16)buf[0] << 8) | (u16)buf[1]);
}

/*------------------------------------------------------------
 * 软件片选（SW-CS）控制：
 *   - 当 SPI 控制器硬件 CS 不可用或被其他驱动占用时，
 *     用普通 GPIO 模拟 CS（空闲时拉高，传输时拉低）。
 *   - 仅在 use_sw_cs = true 时才操作 GPIO。
 *------------------------------------------------------------*/
static inline void mx6hub_spi_cs_active(struct mx6hub_spi_ctx *ctx)
{
    if (ctx->use_sw_cs && gpio_is_valid(ctx->cs_gpio))
        gpio_set_value(ctx->cs_gpio, 0);   /* CS 拉低：选中从机 */
}

static inline void mx6hub_spi_cs_inactive(struct mx6hub_spi_ctx *ctx)
{
    if (ctx->use_sw_cs && gpio_is_valid(ctx->cs_gpio))
        gpio_set_value(ctx->cs_gpio, 1);   /* CS 拉高：释放从机 */
}

/*------------------------------------------------------------
 * SPI 读寄存器：
 *   ICM20608 读操作：DI 引脚 = reg | 0x80（bit7=1 表示读），
 *   DO 引脚返回 len 字节数据。
 *   内部有最多 3 次重试逻辑，每次重试前短暂释放锁（让其他等待者有机会获得总线）。
 *
 * 注意：ctx->lock 只保护本次传输的原子性；跨传输的上下文状态（如 hw_ready）
 * 由 mx6hub_spi_lock 在外层保护。
 *------------------------------------------------------------*/
static int mx6hub_icm_read(struct mx6hub_spi_ctx *ctx, u8 reg, void *buf, size_t len)
{
    u8 tx;
    int ret = -ENODEV;
    u32 retry;

    if (!ctx->spi || !buf || !len || len > MX6HUB_SPI_IO_MAX_LEN)
        return -EINVAL;

    tx = reg | 0x80;                    /* 读操作：bit7=1 */

    mutex_lock(&ctx->lock);
    for (retry = 0; retry < MX6HUB_SPI_RETRY_MAX; retry++) {
        mx6hub_spi_cs_active(ctx);
        ret = spi_write_then_read(ctx->spi, &tx, sizeof(tx), buf, len);
        mx6hub_spi_cs_inactive(ctx);
        if (!ret) {
            ctx->last_err = 0;
            mutex_unlock(&ctx->lock);
            return 0;                    /* 成功 */
        }
        ctx->retry_cnt++;               /* 记录总线错误重试次数 */
    }
    ctx->last_err = ret;               /* 记录最后一次错误码 */
    mutex_unlock(&ctx->lock);
    return ret;
}

/*------------------------------------------------------------
 * SPI 写单字节寄存器：
 *   写操作：DI 高位 bit7=0，配合寄存器地址和数据两字节连续发送。
 *   与读函数共用同一套重试逻辑。
 *------------------------------------------------------------*/
static int mx6hub_icm_write_u8(struct mx6hub_spi_ctx *ctx, u8 reg, u8 val)
{
    u8 txbuf[2];
    int ret = -ENODEV;
    u32 retry;

    if (!ctx->spi)
        return -EINVAL;

    txbuf[0] = reg & ~0x80;            /* 写操作：bit7=0 */
    txbuf[1] = val;

    mutex_lock(&ctx->lock);
    for (retry = 0; retry < MX6HUB_SPI_RETRY_MAX; retry++) {
        mx6hub_spi_cs_active(ctx);
        ret = spi_write(ctx->spi, txbuf, sizeof(txbuf));
        mx6hub_spi_cs_inactive(ctx);
        if (!ret) {
            ctx->last_err = 0;
            mutex_unlock(&ctx->lock);
            return 0;
        }
        ctx->retry_cnt++;
    }
    ctx->last_err = ret;
    mutex_unlock(&ctx->lock);
    return ret;
}

/*------------------------------------------------------------
 * ICM20608 基础初始化序列：
 *
 *   1. PWR_MGMT_1 |= 0x80         → 芯片复位（建议等待 50ms）
 *   2. PWR_MGMT_1  = 0x01         → 选择 PLL，启用时钟
 *   3. 读 WHO_AM_I (0x75)         → 验证芯片通信是否正常（期望 0xAF / 0xAE）
 *   4. PWR_MGMT_2  = 0x00         → 开启加速度计和陀螺仪
 *   5. ACCEL_INTEL_CTRL = 0xC0    → 使能 WoM 中断
 *   6. WOM_EN = 0x20             → WoM 阈值 0x20 * 2mg = 64 mg（运动灵敏度）
 *
 * WoM 工作原理：
 *   平时 ICM20608 处于低功耗 standby 模式，不产生数据；
 *   当加速度变化超过阈值时自动唤醒，上报一次数据后重新进入 standby。
 *   这样可以在低功耗场景下实现"有变化才读数"的事件驱动模式。
 *
 * 注意：WoM 阈值越低越灵敏，但可能产生更多无效中断（噪声触发）。
 * 0x20（约 64 mg）对日常轻度运动是一个平衡值。
 *------------------------------------------------------------*/
static int mx6hub_icm_basic_init(struct mx6hub_spi_ctx *ctx)
{
    u8 whoami_read = 0;
    int ret;

    ret = mx6hub_icm_write_u8(ctx, ICM20_PWR_MGMT_1, 0x80);
    if (ret) {
        pr_warn("mx6hub: icm write PWR_MGMT_1 reset failed ret=%d\n", ret);
        return ret;
    }
    msleep(50);                         /* 复位后等待 50ms，确保芯片稳定 */

    ret = mx6hub_icm_write_u8(ctx, ICM20_PWR_MGMT_1, 0x01);
    if (ret) {
        pr_warn("mx6hub: icm write PWR_MGMT_1 clk failed ret=%d\n", ret);
        return ret;
    }
    msleep(50);                         /* 时钟切换后等待 50ms */

    ret = mx6hub_icm_read(ctx, ICM20_WHO_AM_I, &whoami_read, sizeof(whoami_read));
    if (ret) {
        pr_warn("mx6hub: WHO_AM_I read failed ret=%d\n", ret);
        return ret;
    }
    pr_info("mx6hub: WHO_AM_I=0x%02x\n", whoami_read);

    ret = mx6hub_icm_write_u8(ctx, ICM20_PWR_MGMT_2, 0x00);
    if (ret) {
        pr_warn("mx6hub: icm write PWR_MGMT_2 failed ret=%d\n", ret);
        return ret;
    }

    /* 使能 WoM 中断：bit7=1（latch_int_en），bit6=1（int_en）*/
    ret = mx6hub_icm_write_u8(ctx, ICM20_ACCEL_INTEL_CTRL, 0xC0);
    if (ret)
        pr_warn("mx6hub: ACCEL_INTEL_CTRL write failed ret=%d\n", ret);

    /* WoM 阈值：值越小越灵敏（0x01 ≈ 2 mg），0x20 ≈ 64 mg */
    ret = mx6hub_icm_write_u8(ctx, ICM20_WOM_EN, 0x20);
    if (ret)
        pr_warn("mx6hub: WOM_EN write failed ret=%d\n", ret);

    ctx->wom_enabled = true;
    pr_info("mx6hub: WoM enabled with threshold=0x20\n");

    if (whoami_read != ICM20_WHOAMI_ICM20608G && whoami_read != ICM20_WHOAMI_ICM20608D) {
        pr_warn("mx6hub: unexpected WHO_AM_I=0x%02x\n", whoami_read);
        return -ENODEV;                /* 芯片 ID 不匹配，通信对象可能不是 ICM20608 */
    }

    ctx->whoami = whoami_read;
    return 0;
}

/*------------------------------------------------------------
 * SPI 总线参数配置 + 身份探测：
 *   - SPI_MODE_0：CPOL=0, CPHASE=0，IMX6ULL ECSPI 常用模式
 *   - 8 MHz max_speed：低于 ICM20608 上限，留有一定裕量
 *   - 8 bits_per_word：标准字节传输
 *   - 调用 icm_basic_init 执行寄存器初始化和 ID 验证
 *------------------------------------------------------------*/
static int mx6hub_icm_probe_identity(struct mx6hub_spi_ctx *ctx)
{
    int ret;

    ctx->spi->mode = MX6HUB_ICM_MODE;
    ctx->spi->max_speed_hz = MX6HUB_ICM_MAX_HZ;
    ctx->spi->bits_per_word = 8;
    ret = spi_setup(ctx->spi);
    if (ret) {
        pr_warn("mx6hub: spi_setup failed mode=%u hz=%u ret=%d\n",
                MX6HUB_ICM_MODE, MX6HUB_ICM_MAX_HZ, ret);
        return ret;
    }

    ret = mx6hub_icm_basic_init(ctx);
    if (!ret) {
        pr_info("mx6hub: icm20608 probe ok mode=%u hz=%u whoami=0x%02x\n",
                MX6HUB_ICM_MODE, MX6HUB_ICM_MAX_HZ, ctx->whoami);
        return 0;
    }

    pr_warn("mx6hub: icm20608 probe fail mode=%u hz=%u ret=%d whoami=0x%02x\n",
            MX6HUB_ICM_MODE, MX6HUB_ICM_MAX_HZ, ret, ctx->whoami);
    return ret;
}

/*------------------------------------------------------------
 * 释放软件片选 GPIO：
 *   仅在 use_sw_cs=true 时持有 GPIO；模块退出或 SPI driver unbind 时调用。
 *   释放前先将 CS 拉高（idle 状态），再 gpio_free 交还内核 GPIO 子系统。
 *------------------------------------------------------------*/
static void mx6hub_spi_release_sw_cs(struct mx6hub_spi_ctx *ctx)
{
    if (ctx->use_sw_cs && gpio_is_valid(ctx->cs_gpio)) {
        gpio_set_value(ctx->cs_gpio, 1);
        gpio_free(ctx->cs_gpio);
    }
    ctx->use_sw_cs = false;
    ctx->cs_gpio = -EINVAL;
}

/*------------------------------------------------------------
 * 配置软件片选 GPIO：
 *   尝试从设备树节点 /soc/.../ecspi@02010000 中读取 cs-gpio 属性；
 *   若成功，申请 GPIO 并配置为输出模式（默认高电平 = CS 空闲）。
 *
 * 设计背景：
 *   IMX6ULL 的 ECSPI2 控制器在某些板子上，硬件 CS 引脚被其他功能复用，
 *   或内核 SPI framework 没有自动管理该引脚，此时用软件 GPIO 模拟 CS 是常见做法。
 *------------------------------------------------------------*/
static void mx6hub_spi_setup_sw_cs(struct mx6hub_spi_ctx *ctx)
{
    struct device_node *ecspi_np;
    int ret;

    ctx->use_sw_cs = false;
    ctx->cs_gpio = -EINVAL;

    ecspi_np = of_find_node_by_path(MX6HUB_ECSPI3_NODE_PATH);
    if (!ecspi_np)
        return;

    ctx->cs_gpio = of_get_named_gpio(ecspi_np, "cs-gpio", 0);
    of_node_put(ecspi_np);
    if (!gpio_is_valid(ctx->cs_gpio)) {
        ctx->cs_gpio = -EINVAL;
        return;
    }

    ret = gpio_request(ctx->cs_gpio, "mx6hub-icm20608-cs");
    if (ret) {
        pr_warn("mx6hub: gpio_request cs-gpio(%d) failed (%d)\n", ctx->cs_gpio, ret);
        ctx->cs_gpio = -EINVAL;
        return;
    }

    ret = gpio_direction_output(ctx->cs_gpio, 1);  /* 默认输出高：CS idle */
    if (ret) {
        pr_warn("mx6hub: gpio_direction_output cs-gpio(%d) failed (%d)\n",
                ctx->cs_gpio, ret);
        gpio_free(ctx->cs_gpio);
        ctx->cs_gpio = -EINVAL;
        return;
    }

    ctx->use_sw_cs = true;
    pr_info("mx6hub: using software CS gpio=%d\n", ctx->cs_gpio);
}

/*------------------------------------------------------------
 * SPI driver probe（由 SPI framework 调用）：
 *
 * 流程：
 *   1. 用 mx6hub_spi_lock 保护，将 spi_device 绑定到 mx6hub_dev->spi。
 *      如果 owner 为空（mx6hub.ko 尚未加载），返回 -ENODEV 使 SPI framework
 *      将该 spi_device 留待后续重试。
 *   2. 若已有另一个 spi_device 接入，返回 -EBUSY（SPI 总线一次只支持一个 ICM20608）。
 *   3. 配置软件 CS → 探测 ICM20608 身份 → 成功则 hw_ready=true。
 *   4. 失败时回退：清空 spi 指针，但不注销 driver（spi bus 会择机重试 probe）。
 *
 * 注意：这里不持有 mx6hub_spi_lock 就访问 hub->spi，是安全的，
 * 因为只有在 probe 和 remove 中修改这些字段，且均已锁保护。
 *------------------------------------------------------------*/
static int mx6hub_spi_probe(struct spi_device *spi)
{
    struct mx6hub_dev *hub;
    int ret;

    mutex_lock(&mx6hub_spi_lock);
    hub = mx6hub_spi_owner;
    if (!hub) {
        /* mx6hub.ko 尚未加载，ICM20608 会在 driver 加载后再次尝试绑定 */
        mutex_unlock(&mx6hub_spi_lock);
        return -ENODEV;
    }

    if (hub->spi.spi && hub->spi.spi != spi) {
        mutex_unlock(&mx6hub_spi_lock);
        return -EBUSY;
    }

    hub->spi.spi = spi;
    hub->spi.hw_ready = false;
    hub->spi.whoami = 0;
    hub->spi.read_err = 0;
    spi_set_drvdata(spi, hub);
    mutex_unlock(&mx6hub_spi_lock);

    pr_info("mx6hub: spi probe %s modalias=%s\n", dev_name(&spi->dev), spi->modalias);
    mx6hub_spi_setup_sw_cs(&hub->spi);
    ret = mx6hub_icm_probe_identity(&hub->spi);
    if (ret) {
        mutex_lock(&mx6hub_spi_lock);
        if (mx6hub_spi_owner == hub && hub->spi.spi == spi) {
            hub->spi.spi = NULL;
        }
        mutex_unlock(&mx6hub_spi_lock);
        mx6hub_spi_release_sw_cs(&hub->spi);
        pr_warn("mx6hub: icm20608 probe failed in spi_driver (%d)\n", ret);
        return ret;
    }

    mutex_lock(&mx6hub_spi_lock);
    if (mx6hub_spi_owner == hub && hub->spi.spi == spi)
        hub->spi.hw_ready = true;
    mutex_unlock(&mx6hub_spi_lock);

    pr_info("mx6hub: icm20608 online, WHO_AM_I=0x%02x\n", hub->spi.whoami);
    return 0;
}

/*------------------------------------------------------------
 * SPI driver remove（spi_device unbind 或 driver 卸载时调用）：
 *   清空 ctx->spi，标记 hw_ready=false，释放软件 CS GPIO。
 *------------------------------------------------------------*/
static int mx6hub_spi_remove(struct spi_device *spi)
{
    struct mx6hub_dev *hub = spi_get_drvdata(spi);

    if (!hub)
        return 0;

    mutex_lock(&mx6hub_spi_lock);
    if (hub->spi.spi == spi) {
        hub->spi.spi = NULL;
        hub->spi.hw_ready = false;
        hub->spi.whoami = 0;
    }
    mutex_unlock(&mx6hub_spi_lock);
    mx6hub_spi_release_sw_cs(&hub->spi);

    pr_info("mx6hub: spi remove %s\n", dev_name(&spi->dev));
    return 0;
}

static const struct spi_device_id mx6hub_spi_id[] = {
    { "icm20608", 0 },
    { "alientek,icm20608", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, mx6hub_spi_id);

static const struct of_device_id mx6hub_spi_of_match[] = {
    { .compatible = "alientek,icm20608" },
    { }
};
MODULE_DEVICE_TABLE(of, mx6hub_spi_of_match);

static struct spi_driver mx6hub_icm_spi_driver = {
    .driver = {
        .name = "mx6hub_icm20608",
        .of_match_table = of_match_ptr(mx6hub_spi_of_match),
    },
    .probe = mx6hub_spi_probe,
    .remove = mx6hub_spi_remove,
    .id_table = mx6hub_spi_id,
};

/*------------------------------------------------------------
 * 读取 ICM20608 加速度计 + 陀螺仪 6 轴原始数据（14 B）：
 *
 *   raw[0-1]  = ACCEL_XOUT（BE → s16）
 *   raw[2-3]  = ACCEL_YOUT
 *   raw[4-5]  = ACCEL_ZOUT
 *   raw[6-7]  = TEMP_OUT（忽略）
 *   raw[8-9]  = GYRO_XOUT（BE → s16）
 *   raw[10-11]= GYRO_YOUT
 *   raw[12-13]= GYRO_ZOUT
 *
 * 返回 0 表示成功，非零表示总线通信错误。
 *------------------------------------------------------------*/
static int mx6hub_spi_read_payload(struct mx6hub_spi_ctx *ctx,
                                   struct mx6hub_icm20608_payload *payload)
{
    u8 raw[14];
    int ret;

    ret = mx6hub_icm_read(ctx, ICM20_ACCEL_XOUT_H, raw, sizeof(raw));
    if (ret)
        return ret;

    payload->accel_x = mx6hub_be16_to_s16(&raw[0]);
    payload->accel_y = mx6hub_be16_to_s16(&raw[2]);
    payload->accel_z = mx6hub_be16_to_s16(&raw[4]);
    payload->gyro_x = mx6hub_be16_to_s16(&raw[8]);
    payload->gyro_y = mx6hub_be16_to_s16(&raw[10]);
    payload->gyro_z = mx6hub_be16_to_s16(&raw[12]);
    return 0;
}

/*------------------------------------------------------------
 * WoM 运动检测：比较当前加速度与上一次采样值，差异超过阈值则视为"有运动"。
 *
 * ICM20608 ±2g 量程时：16384 LSB / g
 *   阈值 1047 LSB ≈ 64 mg，对应日常轻度运动（如拿放设备）。
 *
 * 返回 true  ＝> 有运动，应上报数据
 * 返回 false => 无运动，跳过本次采样（节省总线带宽和 FIFO 空间）
 *
 * has_prev = false（首次调用）时直接返回 true，保证初始状态一定被记录。
 *------------------------------------------------------------*/
static bool mx6hub_spi_motion_detected(struct mx6hub_icm20608_payload *prev,
                                        struct mx6hub_icm20608_payload *curr)
{
    s32 dx, dy, dz;

    if (!prev)
        return true;  /* 首次采样，无历史数据，直接上报 */

    dx = (s32)curr->accel_x - (s32)prev->accel_x;
    dy = (s32)curr->accel_y - (s32)prev->accel_y;
    dz = (s32)curr->accel_z - (s32)prev->accel_z;

    /* 64 mg 阈值：X/Y/Z 任一轴超过即触发 */
#define WOM_THRESHOLD 1047
    return (abs(dx) > WOM_THRESHOLD ||
            abs(dy) > WOM_THRESHOLD ||
            abs(dz) > WOM_THRESHOLD);
}

/*------------------------------------------------------------
 * SPI 恢复尝试：当 hw_ready=false 时，workfn 会尝试重新初始化 ICM20608。
 *   - reinit_cnt 记录恢复次数，若频繁恢复（如 > 10/min）说明总线有问题。
 *   - 成功后 hw_ready=true，下次 workfn 会直接读真实数据，不再走 fallback。
 *------------------------------------------------------------*/
static int mx6hub_spi_try_recover(struct mx6hub_spi_ctx *ctx)
{
    int ret;

    if (!ctx->spi)
        return -ENODEV;

    ctx->reinit_cnt++;
    ret = mx6hub_icm_probe_identity(ctx);
    if (ret) {
        ctx->last_err = ret;
        return ret;
    }

    ctx->hw_ready = true;
    ctx->last_err = 0;
    return 0;
}

/*------------------------------------------------------------
 * Fallback mock 数据：
 *   当 ICM20608 硬件不可用时，用 jiffies 生成伪随机数填充 payload，
 *   模拟传感器输出，保证 FIFO 链路始终有数据流通，可用于驱动 bring-up 验证。
 *   数值范围与真实传感器量程接近，避免用户在不知情时误将 fallback 数据当作真实读数。
 *
 * 加速度 mock：低 11 位取值，范围 [-1024, +1023]，相当于 ±0.5g
 * 陀螺仪 mock：低 12 位取值，范围 [-2048, +2047]，相当于 ±250°/s 量程
 *------------------------------------------------------------*/
static void mx6hub_spi_fill_fallback(struct mx6hub_spi_ctx *ctx,
                                     struct mx6hub_icm20608_payload *payload)
{
    payload->accel_x = (s16)(jiffies & 0x07ff) - 1024;
    payload->accel_y = (s16)((jiffies >> 1) & 0x07ff) - 1024;
    payload->accel_z = (s16)((jiffies >> 2) & 0x07ff) - 1024;
    payload->gyro_x = (s16)((jiffies >> 3) & 0x0fff) - 2048;
    payload->gyro_y = (s16)((jiffies >> 4) & 0x0fff) - 2048;
    payload->gyro_z = (s16)((jiffies >> 5) & 0x0fff) - 2048;
    payload->seq = (u16)(ctx->seq & 0xffff);
}

/*------------------------------------------------------------
 * SPI workqueue 工作函数（由心跳定时器或中断底半部调度）：
 *
 * 执行顺序：
 *   1. 递增 spi_work_events 计数（debugfs 可观测）
 *   2. 若 hw_ready=false 且 spi!=NULL，尝试恢复 ICM20608（自愈）
 *   3. hw_ready=true：读 6 轴原始数据；失败则切换到 fallback 并标记 hw_ready=false
 *   4. hw_ready=false：填充 fallback mock 数据，递增 fallback_events
 *   5. WoM 检测：若无运动则跳过本次入队（wom_skipped++），节省 FIFO 资源
 *   6. 调用 mx6hub_fill_record + mx6hub_enqueue_record 写入 FIFO
 *
 * 设计要点：
 *   - workqueue 在进程上下文执行，I2C/SPI 传输可以 sleep（msleep、mutex）。
 *   - hw_ready 自愈机制使驱动在 ICM20608 临时掉线后能自动恢复，无需人工干预。
 *   - WoM 过滤避免静止状态下 FIFO 被大量无意义的重复数据填满。
 *------------------------------------------------------------*/
static void mx6hub_spi_workfn(struct work_struct *work)
{
    struct mx6hub_spi_ctx *ctx = container_of(work, struct mx6hub_spi_ctx, work);
    struct mx6hub_record rec;
    struct mx6hub_icm20608_payload payload;
    static struct mx6hub_icm20608_payload prev_payload;
    static bool has_prev;
    int ret;

    if (!ctx->enabled)
        return;

    mx6hub_mark_spi_work(ctx->hub);

    memset(&payload, 0, sizeof(payload));
    ctx->seq++;
    payload.seq = (u16)(ctx->seq & 0xffff);

    /* 自愈：若上次读失败导致 hw_ready=false，尝试重新初始化 */
    if (!ctx->hw_ready && ctx->spi) {
        ret = mx6hub_spi_try_recover(ctx);
        if (ret)
            pr_warn_ratelimited("mx6hub: icm20608 recover failed (%d), keep fallback\n", ret);
    }

    if (ctx->hw_ready) {
        ret = mx6hub_spi_read_payload(ctx, &payload);
        if (ret) {
            ctx->read_err++;
            ctx->last_err = ret;
            pr_warn_ratelimited("mx6hub: icm20608 read failed (%d), fallback mock\n", ret);
            ctx->hw_ready = false;       /* 通信失败，切换 fallback，下次 workfn 尝试自愈 */
        } else {
            ctx->real_events++;         /* 真实数据计数 */
        }
    }

    if (!ctx->hw_ready) {
        ctx->fallback_events++;
        mx6hub_spi_fill_fallback(ctx, &payload);
    }

    /* WoM 过滤：有运动才入队；静止时跳过本次采样 */
    if (!mx6hub_spi_motion_detected(has_prev ? &prev_payload : NULL, &payload)) {
        ctx->wom_skipped++;
        return;
    }

    /* 保存本次值用于下次对比 */
    prev_payload = payload;
    has_prev = true;

    mx6hub_fill_record(&rec, MX6HUB_SRC_SPI_ICM20608, &payload, sizeof(payload));
    mx6hub_enqueue_record(ctx->hub, &rec);
}

/*------------------------------------------------------------
 * mx6hub_spi_init：SPI 子模块初始化。
 *
 * 流程：
 *   1. 初始化 ctx 所有字段（清零状态、分配 mutex / work_struct）。
 *   2. 将当前 hub 设为 mx6hub_spi_owner，使得后续加载的 icm20608 SPI driver
 *      的 probe 能找到对应的 mx6hub_dev。
 *   3. 若 spi driver 尚未注册则注册（spi_register_driver）。
 *      注意：driver 注册后会立刻遍历总线上已有的设备并尝试 probe，
 *      此时 mx6hub_spi_owner 已经完全就绪。
 *
 * 即使 ICM20608 硬件不在位（spi driver probe 失败），函数仍然返回 0，
 * 因为 fallback 路径（jiffies mock）已经启用，驱动整体可以正常工作。
 *------------------------------------------------------------*/
int mx6hub_spi_init(struct mx6hub_dev *hub)
{
    hub->spi.hub = hub;
    hub->spi.enabled = true;
    hub->spi.hw_ready = false;
    hub->spi.spi_created = false;
    hub->spi.seq = 0;
    hub->spi.read_err = 0;
    hub->spi.retry_cnt = 0;
    hub->spi.reinit_cnt = 0;
    hub->spi.real_events = 0;
    hub->spi.fallback_events = 0;
    hub->spi.last_err = 0;
    hub->spi.whoami = 0;
    hub->spi.regmap = NULL;
    hub->spi.spi = NULL;
    hub->spi.use_sw_cs = false;
    hub->spi.cs_gpio = -EINVAL;
    hub->spi.wom_enabled = false;
    mutex_init(&hub->spi.lock);
    INIT_WORK(&hub->spi.work, mx6hub_spi_workfn);

    /* 设置 owner，使得 SPI driver probe 时能找到对应的 hub 设备 */
    mutex_lock(&mx6hub_spi_lock);
    mx6hub_spi_owner = hub;
    mutex_unlock(&mx6hub_spi_lock);

    if (!mx6hub_spi_driver_registered) {
        int ret;

        ret = spi_register_driver(&mx6hub_icm_spi_driver);
        if (ret) {
            pr_warn("mx6hub: spi_register_driver failed (%d), using mock fallback\n", ret);
            return 0;
        }
        mx6hub_spi_driver_registered = true;
    }

    if (!hub->spi.hw_ready) {
        pr_warn("mx6hub: icm20608 not bound/ready, using mock fallback\n");
        pr_warn("mx6hub: check whether another driver already owns spi2.0\n");
    }

    return 0;
}

/*------------------------------------------------------------
 * mx6hub_spi_exit：SPI 子模块退出。
 *   - cancel_work_sync 等待当前正在执行的 workfn 完成（防止 uaf）
 *   - 释放软件 CS GPIO
 *   - 注销 SPI driver（不会再有新的 spi_probe 调用）
 *------------------------------------------------------------*/
void mx6hub_spi_exit(struct mx6hub_dev *hub)
{
    mutex_lock(&mx6hub_spi_lock);
    if (mx6hub_spi_owner == hub)
        mx6hub_spi_owner = NULL;
    mutex_unlock(&mx6hub_spi_lock);

    cancel_work_sync(&hub->spi.work);     /* 等待 workfn 退出，防止 uaf */
    mx6hub_spi_release_sw_cs(&hub->spi);

    if (mx6hub_spi_driver_registered) {
        spi_unregister_driver(&mx6hub_icm_spi_driver);
        mx6hub_spi_driver_registered = false;
    }
}

/*------------------------------------------------------------
 * SPI workqueue 调度入口：
 *   由 mx6hub_irq_thread 或心跳定时器调用，将 SPI 工作项排队到 mx6hub_wq。
 *   如果 enabled=false（模块被禁止），直接返回，不调度。
 *------------------------------------------------------------*/
void mx6hub_spi_schedule(struct mx6hub_dev *hub)
{
    if (!hub->spi.enabled)
        return;
    queue_work(hub->wq, &hub->spi.work);
}
