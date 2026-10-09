# Phase 4 执行计划：传感器框架迁移（ICM20608 → IIO，AP3216C → Linux Input）

本文件是 [总计划](plan_imx6ull_pantilit_final.md) 第 Phase 4 章的展开，服从 `rules/` 全部规则。
**状态：2026-10-07 已完成，板端验收全部 PASS。**

## 0. 协商结论（开工前定下，执行中未动摇）

总计划原文写的是"AP3216C / ICM20608 都迁 IIO"。开工前与用户协商，修正为混合路线。
以下 6 条是协商结论，本文件与代码都以它为准：

| # | 决策 | 理由 |
| --- | --- | --- |
| 1 | **AP3216C → Linux Input（`EV_SW`/`SW_FRONT_PROXIMITY`），ICM20608 → IIO** | 参考材料（`reference/王政的项目.txt` 指向的 AP3216C 驱动）本身就是 Input 方案，且不含 ICM20608；PS 本质是近/远二值开关，switch 语义天然匹配，硬塞 IIO 要多写一层 channel 映射 |
| 2 | 本轮只到 **sysfs raw 级**，不做 IIO buffer/trigger | 内核 `CONFIG_IIO_BUFFER=n` / `CONFIG_IIO_TRIGGER=n`，本轮**不改内核配置** |
| 3 | **mx6hub 本轮不动** | 改动小、稳定优先 |
| 4 | AP3216C **分两步**：M1 最小版，M2 补 runtime PM + 抗抖动 | 先证明框架通，再补省电与稳定性 |
| 5 | AP3216C **新建独立模块**，以 `ap3216c_myself.c` 为底本 fork | 旧模块原样保留，做 A/B 与回退；rules 禁止直接删除当前可运行实现 |
| 6 | 回写修改总计划的 Phase 4 相关章节 | 已执行，见总计划 Phase 4 章 |

## 1. 关键约束（已确认，不是推断）

- 板端内核 = 开发机内核树 `/home/imx6ull/work/imx6ull/src` = **Linux 4.1.15**。
- DTS 已是 `ap3216c@1e`（compatible `alientek,ap3216c`，带 GPIO1_IO01 下降沿中断）、
  `icm20608@0`（compatible `alientek,icm20608`）—— **无需改 DTS**。
- 新旧驱动 match **同一 compatible**，同一时刻只能绑一个（含内核 BSP 驱动）。
  因此 A/B 一律"卸载一个 → 装另一个 → 顺序取数 → 事后比对"，**禁止并存**。
- `CONFIG_PM` / `CONFIG_PM_SLEEP` / `CONFIG_PM_SLEEP_SMP` / `CONFIG_PM_SLEEP_DEBUG` 均为 y；
  `CONFIG_PM_ADVANCED_DEBUG` **未设**（所以没有 `power/runtime_usage` 等节点，验收不要引用它们）。
- `#include <linux/pm_wakeirq.h>` 在 4.1.15 **不存在**（参考蓝本用到了）—— 必须删。

## 2. 交付物

按目标子系统分目录，沿用"一个模块一个目录、目录内自带 Makefile"的约定：

```
imx6ull_project_src/kernel/iio/icm20608_iio/     icm20608_iio.c  icm20608reg.h  Makefile
imx6ull_project_src/kernel/input/ap3216c_input/  ap3216c_input.c ap3216creg.h  Makefile
imx6ull_project_src/user/icm20608_iio/           icm20608_iio_app.c  Makefile
imx6ull_project_src/user/ap3216c_input/          ap3216c_input_app.c Makefile
scripts/board_verify_sensors.sh                  # 合并验收脚本（见 §6）
logs/phase4/                                     # 构建/验收/手测/中断对比记录
```

模块名用 `icm20608_iio` / `ap3216c_input`（而非 `icm20608`/`ap3216c`），避免与旧 `.ko`
及内核 BSP 驱动同名歧义。

## 3. M1-a：ICM20608 → IIO

底本：`reference/01、例程源码/02、Linux驱动例程/27_iio/spi/icm20608.c`。
该底本本身就是 `INDIO_DIRECT_MODE` 纯 sysfs 方案，完全不碰 buffer/trigger，
在 `IIO_BUFFER=n` 的 4.1.15 上原样可编译。移植动作：

- 原样复用：`ICM20608_CHAN` 宏、`gyro_scale_icm20608`/`accel_scale_icm20608` 表、
  `icm20608_channels[]`、`read_raw`/`write_raw`/`write_raw_get_fmt`、`icm20608_reginit`。
- 删 4 个未用头：`linux/iio/{buffer,trigger,triggered_buffer,trigger_consumer}.h`。
  保留 `linux/unaligned/be_byteshift.h`（`be16_to_cpup` 需要）。
- 改名：模块/`indio_dev->name` → `icm20608_iio`；`of_match`/`spi_device_id` 保持 `alientek,icm20608`。
- **修底本的 probe 顺序**：底本先 `iio_device_register` 再 `regmap_init_spi`，
  两者之间若有读会空指针。改成**先 regmap 就绪再注册**。
- 用户态 app 不硬编码 `iio:deviceN`，扫描 `/sys/bus/iio/devices/iio:device*/name`
  找值为 `icm20608_iio` 的那个再拼路径。

**不要"修正"的两个点：**

1. `icm20608_channels[]` 里 IIO_ACCEL 的通道顺序是 **Y/X/Z**。这是芯片物理轴序，
   不是笔误，按字母排序会得到错误的轴对应。
2. SPI 参数（CS、频率、模式）一律沿用旧 `icm20608.c` 的现有配置，不自创。

### 实测

```
WHO_AM_I = 0XAF
静止水平放置：in_accel_z_raw ≈ 2034（±16G 量程，2048 LSB/g，即 ≈1g），另两轴 ≈ 0
in_temp_raw = 3288
```

## 4. AP3216C → Input（M1 + M2）

### 4.1 M1：最小可用版

底本：`imx6ull_project_src/kernel/baseline/ap3216c_myself/ap3216c_myself.c`（已实测调通）。

**原样保留**：`ap3216c_i2c_read/write`、`ap3216c_flush_interrupt_state`、
`ap3216c_hw_init`（9 步时序，关键在**先写 `PS_INT_FORM(0x22)` 再写 `PSCONFIG=0x05`**）、
`ap3216c_irq_handler`（顶半部仅 `schedule_work`）、`ap3216c_work_callback`、
`mutex`、`request_irq(IRQF_TRIGGER_FALLING)`。

**改动**：

- 删掉字符设备整块（`fops`、`alloc_chrdev_region`、`cdev_*`、`class_create`、`device_create`）
  —— 新模块是纯 Input + sysfs。
- sysfs 属性组挂到 **i2c client 设备**（`sysfs_create_group(&client->dev.kobj, ...)`），
  而非自定义 class：`proximity/ps_raw_data`(RO)、`ambient_light/als_raw_data`(RO)，
  外加 `proximity/prox_counts` 便于验收。
- Input device name 固定 `"ap3216c-proximity"`。
- **判近/远改为软件比较 PS 与阈值**（详见 §5.1）。

**两处不要照抄参考蓝本**：

1. 不用 `of_property_read_string_index(..., "label", ...)` 取名 —— 本项目 DTS 无 `label` 属性，
   照抄会拿到 NULL。
2. 用底本已有的 `i2c_set_clientdata`/`i2c_get_clientdata`，不照抄 `client->dev.driver_data`。

### 4.2 M2：runtime PM + sysfs 校准节点

**sysfs 校准节点**（`DEVICE_ATTR_RW`，全挂 `proximity/` 下）：

| 节点 | 语义 | 落点 |
| --- | --- | --- |
| `threshold_low` / `threshold_high` | 10-bit PS 阈值 | 低 2 位写 `PSLTHDL/PSHTHDL`、高 8 位写 `PSLTHDH/PSHTHDH` |
| `measure_interval` | 用户给 **ms** | 内部换算 `225*wait+212`，反向 `+112` 取整，clamp 1..31 |
| `led_current_ratio` | 0..3 = 100/50/25/12.5 % | reg 0x21 低 2 位 |

实测往返：`threshold_low:30->30`、`threshold_high:150->150`、
`measure_interval:500->437`（500ms 落在 437/662 之间，取近者）、`led_current_ratio:2->2`。

**runtime PM**：`SET_RUNTIME_PM_OPS` + `SET_SYSTEM_SLEEP_PM_OPS`，
autosuspend 延迟 500ms；sysfs 读原始值时用 `pm_runtime_get_sync` 取引用、读完
`mark_last_busy` + `put_autosuspend`。system suspend 时 `disable_irq` + `MODE_PWDOWN`，
resume 时恢复模式、补报一次 far。

**踩过的坑（写进注释，别重蹈）**：probe 末尾若写
`pm_runtime_set_active()` 再 `pm_runtime_put_autosuspend()`，
`set_active()` 并**不**增加 `usage_count`，于是这次 put 把计数打到 **−1**；
之后 `atomic_dec_and_test` 永远不为真、`rpm_idle` 永不触发，`runtime_usage` 也拒绝了挂起。
正确写法是写 `MODE_PS_ONLY` 后直接 `pm_runtime_set_suspended()`，**再** `pm_runtime_enable()`。

### 4.3 实测

```
probe 后=suspended
挂起态读 ALS=70（读时取 PM 引用，自动唤醒芯片）
读的瞬间=active  ->  1s 后=suspended
空闲 3s 中断增量 = 0
```

## 5. 两个反直觉的坑（本轮的主要收获）

### 5.1 `PSDATA` 的 bit7 不能用来判远近

旧 `ap3216c_myself.c` 用 `PSDATALOW` 的 bit7 判 near/far。**该位只在 HYSTERESIS 模式
（reg 0x22 = 0x01）下才有意义，ZONE 模式（0x00，芯片默认）下恒为 0。**

后果：无论手怎么贴近，`input_report_switch` 永远报 far、`value` 永远是 0；
而 Linux 4.1.15 的 `drivers/input/input.c:305-312` 对 `EV_SW` 去重 —— 状态不变就完全不产生
事件。于是表现为"挥手完全没反应"。这个缺陷一直藏着没暴露。

**修法**：判远近用**软件比较 PS 与阈值**（`PS > high` 判 NEAR、`PS < low` 判 FAR、
带内保持上次状态）。此逻辑与芯片的中断模式无关，M2 换模式也不用改。阈值带内保持上次状态，
本身就是一层软件抗抖。

实测：无遮挡 PS≈0~19、手掌贴近 PS=140~591，阈值 20/80 余量 4 倍以上。

### 5.2 PS 中断模式必须用 HYSTERESIS，否则 runtime PM 白做

M2 加了 runtime PM 后，`runtime_status` 一直在 active/suspended 之间反复翻转，
看起来像 autosuspend 没生效。时间序列采样后发现 autosuspend **是**生效的，
只是每次刚挂起就被下一次中断唤醒。

| 模式 (reg 0x22) | 空闲 6s 内 runtime_status | 空闲 6s 内中断次数 |
| --- | --- | --- |
| HYSTERESIS (`ps_int_mode=1`) | 30/30 采样全部 suspended | **0** |
| ZONE (`ps_int_mode=0`) | active/suspended 反复翻转 | **13** |

原因：ZONE 模式只要 PS 落在阈值带外就报中断。无遮挡时 PS 恒在低阈值 20 之下，
于是**每次测量都触发一次中断**，节奏 ~604ms（= `PSLEDWT` 默认 0x04 → `225*4+212` ms，
与实测吻合）。中断 → work → 取 PM 引用 → 唤醒，循环往复，省电完全落空。

**结论**：`ps_int_mode` 默认取 1（HYSTERESIS）。这不是风格取舍，是 runtime PM 能不能省电的前提。
回退办法是 `insmod ap3216c_input.ko ps_int_mode=0`（保留 ZONE 路径）。

实测（HYSTERESIS 默认值下的人工手测）：40s 窗口内手掌贴近/移开 5 个来回，
`value=1` 出现 5 次、`value=0` 出现 5 次，**全部检出，未漏检**。所以功能与省电两者都成立。

## 6. 验证（合并脚本，不逐环节拆开跑）

按用户 2026-10-07 的要求，验收标准做了合并与压缩：原计划的两个独立板端脚本
（`board_verify_icm20608_iio.sh`、`board_verify_ap3216c_input.sh`）**合并为
`scripts/board_verify_sensors.sh`**，一次跑完 6 段：

| 段 | 内容 | 通过信号 |
| --- | --- | --- |
| [1/6] | ICM20608 → IIO | `iio:deviceN` 可读且非 0 |
| [2/6] | AP3216C → Input | 绑定 + sysfs raw + `/dev/input/eventX` 齐备 |
| [3/6] | 人工动作（`WITH_HAND=0` 可跳过） | `EV_SW` 双向事件都收到 |
| [4/6] | A/B + 回退 | ICM20608 新旧矢量模长差 < 5%；旧模块可重装读回 |
| [5/6] | M2 runtime PM + 校准节点 + 空闲中断回归 | suspended→active→suspended；4 个节点往返正确；空闲 3s 中断增量 ≤1 |
| [6/6] | 恢复 | 卸载新模块、回绑 BSP |

几条为省时间做的取舍：

- `WITH_HAND=0` 时跳过人工段，自动段一次跑完约 90 秒。
- A/B 用**矢量模长**而非逐轴比对（静止时三轴都该 ≈2048，模长比单轴更抗噪声），
  阈值取 5%，够区分"驱动错"和"芯片噪声"。
- 回退只验退出码与关键输出，不重跑全套。

### 两个取数陷阱（脚本里已固化，别再踩）

1. **ICM20608 复位后第一次读是无效值**。实测第 1 次 `ax=-262, ay=199, az=1107`，
   第 2 次起才是 `-36/34/2036`。脚本一律 `sed -n '2p'` 丢弃首个样本。
   曾因 `grep -m1` 取到垃圾样本，得出"两个驱动差 677 LSB"的假结论，白查一轮。
2. **旧 app 输出用 `\r\n`**。尾部 `\r` 会让数值 `case` 模式和 `[ -lt ]` 都失败，
   必须 `tr -d '\r '`。曾表现为 A/B 报 `old=0 new=4` 的假失败。

## 7. Phase PASS 对照

| 计划要求 | 结果 |
| --- | --- |
| ICM20608 可通过 IIO 读取 | PASS（`WHO_AM_I=0XAF`，静止 `|a|≈2034`） |
| AP3216C 可通过框架读取 | PASS（Input 事件 `EV_SW`/`SW_FRONT_PROXIMITY` + sysfs raw） |
| 原有字符设备路径仍可回退 | PASS（旧模块可重装并读回，A/B 通过） |
| 新旧接口数据基本一致 | PASS（ICM20608 静止矢量模长差 2~4 LSB） |
| IIO buffer / trigger | 本轮明确推后，**不计入 PASS** |

日志：`logs/phase4/` —— `m2_verify_sensors.log`（最终验收，6 段全跑）、
`m2_hand_ev_sw.log`（人工手测）、`m2_irq_storm_compare.log`（中断模式对比）。
M1 阶段的三份日志已被前两份完整覆盖，收尾时删除，见 `logs/phase4/README.md`。

## 8. 回退

```bash
rmmod ap3216c_input; insmod ./ap3216c_myself.ko; sh ./board_verify_ap3216c_myself.sh
rmmod icm20608_iio;  insmod ./icm20608.ko;       sh ./board_verify_icm20608.sh
```

旧模块与旧脚本**全部保留未改**，是回退证据。全程不碰 boot 分区。

## 9. 明确不做（本轮 Non-goals）

- 不改内核配置、不重编内核、不动 boot 分区。
- 不做 IIO buffer / kfifo / trigger / timestamp。
- 不动 mx6hub，不删任何旧模块。
- 不做 `securityd` 集成、不做性能调优（Phase 6）。
