# IMX6ULL 实验室工具箱安防监控云台 —— 执行计划

> 项目：`pythonerlower/IMX6ULL-Safe-Alarm`  
> 定位：在现有 IMX6ULL 安防感知系统基础上，引入 PCA9685 + 双 MG90S Pan-Tilt 云台与自动目标跟踪，形成“传感器感知—音视频采集—PC 目标定位—云台闭环控制—系统诊断优化”的完整嵌入式项目。  
> 开发原则：**先做到、再精进；先验证、再整理；先单模块闭环、再系统集成。**

## 开发指导

- 程序实现遵循项目 `rules/` 目录中的工程规范。
- 驱动、应用和算法实现优先参考 `reference/` 目录中的已有示例、原厂代码和可复用实现；两自由度跟踪项目索引见 [`reference/pantilt_tracking.md`](../reference/pantilt_tracking.md)。
- 已稳定运行的旧链路优先保留，通过新增模块和 A/B 验证逐步替换，不直接推倒重写。
- 每个 Phase 先完成最小可运行版本，验收通过后再补性能、抽象和复杂功能。
- 机械、电气和软件限位在云台开发中优先级高于控制效果，所有动作必须在实测安全范围内执行。

---

# 1. 项目目标与优先级

## 1.1 最终系统

```text
AP3216C：环境光 / 接近
ICM20608：振动 / 运动
        │
        ▼
     mx6hub / IIO / Input
        │
        ├──► OV5640 视频采集 ──► Ring Buffer ──► PC
        │                                         │
        │                                    目标位置计算
        │                                         │
        │                                    Pan/Tilt 控制量
        │                                         │
        │                                         ▼
        ├──────────────────────────────────── IMX6ULL
        │                                         │
        │                                    PCA9685
        │                                         │
        │                                 MG90S Pan / Tilt
        │
        └──► WM8960 ALSA 录音 / 告警

Debugfs / Ftrace / Kprobes / GDB
贯穿驱动、控制、音视频和系统联调
```

第一版自动跟踪只要求：

```text
摄像头看到目标
    ↓
PC 找到目标中心
    ↓
计算目标与画面中心偏差
    ↓
发送控制量
    ↓
IMX6ULL 控制 Pan/Tilt
    ↓
摄像头自动跟随目标
```

目标跟踪算法本身不是重点，自动跟踪效果和嵌入式控制闭环优先。

## 1.2 优先级

### P0：必须完成

1. 旧项目有效性验证
2. 宿主机工程和仓库整理
3. PCA9685 + MG90S + RK-770 硬件连接与机械标定
4. Pan/Tilt 控制、视频链路、PC 目标跟踪和自动闭环
5. PC 端可视化与基本云台保护

### P1：核心能力补强

1. 传感器框架迁移：ICM20608 → IIO；AP3216C → Linux Input 子系统
2. WM8960 自定义 ALSA 录音/播放应用
3. 传感器、音视频、云台和安防状态机系统集成
4. Debugfs / Ftrace / Kprobes / GDB 诊断
5. Ring Buffer、I/O、视频、音频和中断链路优化

### P2：可选增强

1. CamShift / CSRT 等通用 Tracker
2. PC 端轻量预训练 person 模型
3. P → PID
4. LCD 完整状态界面
5. 音频降噪
6. 目标丢失后的自动搜索
7. 长时间稳定性与更细性能测试

---

# 2. 当前软硬件基础

## 2.1 已有硬件

```text
IMX6ULL 开发板
OV5640
LCD
AP3216C
ICM20608
WM8960
双轴 Pan-Tilt 支架
MG90S × 2
PCA9685
RunesKee RK-770 电源转换板
12V DC-IN
杜邦线
以太网
```

## 2.2 已有软件基础

当前工程已经包含：

```text
AP3216C 驱动与测试
ICM20608 驱动与测试
mx6hub
hub_monitor
sensor_alert_app
OV5640 / V4L2 测试程序
远程视频链路
Debugfs
Ftrace / Kprobes 脚本
WM8960 板端检查脚本
```

当前 WM8960 主要完成声卡链路和命令级验证，自定义 ALSA 应用仍需要补充。

后续统一使用实际 Codec 名称：

```text
WM8960
```

---

# 3. 总体执行顺序

```text
Phase 0  旧系统有效性验证
        ↓
Phase 1  工程与仓库整理
        ↓
Phase 2  新硬件连接与机械标定
        ↓
Phase 3  Pan/Tilt + 视频 + 目标跟踪闭环
        ↓
Phase 4  传感器框架迁移（ICM20608→IIO，AP3216C→Input）
        ↓
Phase 5  WM8960 ALSA 应用补全
        ↓
Phase 6  系统集成、诊断与优化
```

其中：

```text
Phase 0~3：优先完成自动跟踪效果
Phase 4~6：在主闭环稳定后继续深化
```

---

# Phase 0：旧系统有效性验证

## 指导与边界

**优先级：P0**

先证明已有功能仍可运行，再修改目录、设备树或接入新硬件。

本阶段只验证和记录，不做结构性重构。

## 目标

重新验证：

```text
AP3216C
ICM20608
mx6hub
OV5640
WM8960
LED / BEEP
Debugfs
Ftrace / Kprobes
至少一条远程视频链路
```

## 任务

建立版本基线：

```text
git tag baseline-before-pantilt
```

或创建：

```text
feature/pantilt
```

复用已有验证脚本：

```text
board_verify_full_stack.sh
board_verify_ap3216c_myself.sh
board_verify_icm20608.sh
board_verify_mx6hub_poll.sh
check_wm8960_baseline.sh
remote_link/*
```

建立：

```text
docs/baseline_2026_pantilt.md
```

记录：

```text
功能是否可运行
当前设备节点
当前 DTB
当前 kernel/module
当前网络配置
已知异常
```

## Phase PASS

- AP3216C、ICM20608 能读到数据。
- OV5640 能出图。
- WM8960 能被 ALSA 枚举。
- mx6hub 能运行。
- 至少一条视频链路可用。

不要求此阶段解决所有旧问题。

---

# Phase 1：工程与仓库整理 —— 已实现（2026-10-07，`CURRENT_DTS` 溯源已于当天第二轮闭合，见本节末）

## 指导与边界

**优先级：P0**

先确认当前构建和部署实际使用哪些文件，再整理。

禁止在未确认用途前直接删除文件。

## 目标

解决：

```text
多个相似 DTS
多个相似驱动目录
backup / ref / myself 混杂
历史二进制和源码混放
正式代码与实验代码边界不清
```

## 任务

建立：

```text
docs/repo_inventory.md
```

记录：

```text
文件/目录
作用
是否参与当前编译
是否参与板端部署
当前状态
后续归类
```

重点检查（**2026-10-07 结构整改后，程序源码一律位于 `imx6ull_project_src/` 下**）：

```text
imx6ull_project_src/kernel/baseline/
imx6ull_project_src/kernel/experiments/
imx6ull_project_src/kernel/dts_patch/
imx6ull_project_src/kernel/mx6hub/

imx6ull_project_src/user/baseline/
imx6ull_project_src/user/experiments/
imx6ull_project_src/user/hub_monitor/
imx6ull_project_src/user/ioctl_test_app/
imx6ull_project_src/user/sensor_alert_app/
imx6ull_project_src/user/v4l2_fb_preview/

scripts/                          # 仍在工程根目录，只做调度
imx6ull_project_src/out/          # 构建/部署生成物
根目录历史可执行文件
```

逐步整理为：

```text
active      当前正式实现
reference   参考/原厂实现
experiments 实验实现
archive     历史版本
out/build   构建产物
```

DTS 至少明确一个：

```text
CURRENT_DTS
```

如果立即移动目录会破坏 Makefile，先完成清单和入口说明，再分步迁移。

统一常用入口：

```text
make baseline
make build
make deploy
make verify
```

## Phase PASS

- 当前正式 DTS 明确。
- AP3216C / ICM20608 / mx6hub 正式源码明确。
- 实验、参考、历史文件边界清晰。
- 整理后旧项目仍可重新构建和部署。

## 2026-10-07 核查结论

`make host-check`、`make build` 均通过；正式模块源码位置与实验/参考边界成立。核查记录见 [`logs/phase1_phase2/`](../logs/phase1_phase2/README.md)。

**保留项**：

- ~~`CURRENT_DTS` 只确认到二进制，未追溯到源码。~~ **2026-10-07 第二轮已闭合**：运行镜像
  （boot 分区 `/run/media/mmcblk1p1/imx6ull-14x14-emmc-4.3-480x272-c.dtb`，md5 `ea5ae0f0…`，
  与 `/sys/firmware/fdt` 过滤 phandle 后仅差 U-Boot 注入的 `/memreserve` 与 `/chosen/bootargs`）
  的源码就在内核树内：`imx6ull-14x14-emmc-4.3-480x272-c-ap3216c-irq.dts`。用内核树自己的 `dtc`
  重建该 `.dts` 得 md5 `ddaffc84…`（39685 B，与树内 `.dtb` 逐字节一致），与运行镜像逐节点比对
  只差 `ap3216c@1e` 的 `interrupts` 一个属性（运行 `EDGE_BOTH`，源码 `EDGE_FALLING`）。
  第一轮判"源码不在工程内"是漏查了这一支。详见
  [`logs/phase1_phase2/20261007_dtb_lineage.md`](../logs/phase1_phase2/20261007_dtb_lineage.md)。
- `scripts/deploy_offline.sh` 的 `DEPLOY_DTB` 默认值 **2026-10-07 已改**为
  `imx6ull-14x14-emmc-4.3-480x272-c-ap3216c-irq.dtb`（禁用 `dht11`/`ds18b20`、与运行镜像同源）。
  改前它指向的 `…-c.dtb`（`59dc8e0b…`）会把 `dht11`/`ds18b20` 置为 `okay`，与 AP3216C INT
  复用的 `GPIO1_IO01` 冲突。**覆盖 boot 分区始终是手动步骤**，脚本不会自动写。
- `interrupts` 那一行差异的实际影响待评估：驱动侧 `request_irq` 用的是 `IRQF_TRIGGER_FALLING`，
  会覆盖 DT 里的触发类型，表现上大概率无差别，但未实测确认。
- `make verify` 入口仍缺失（验收一律走 `scripts/` 下的板端脚本）。
- `imx6ull_project_src/temp/` 已删（2026-10-07 归类时并入 `kernel/`、`user/`、`out/`）。
- 工程根目录**已是 Git 仓库**（2026-10-07 `git init`，`phase3-before_delete` 起有提交历史）；
  但**没有打任何 tag**，`git tag baseline-before-pantilt` 这一步没做，也没有可回溯的 tag 恢复点。

---

# Phase 2：PCA9685 + MG90S 硬件验证与机械标定 —— 已实现（2026-10-07）

逐条核查见 `logs/phase1_phase2/20261007_phase2_verify.md`，5 条 Phase PASS 全部成立，无遗留项。
标定结果写入 `config/pantilt_calibration.conf`：pan home=377（工作范围 239..515）、
tilt home=242（工作范围 162..342）；过程见 `logs/phase1_phase2/20261007_calib_pan.log`、
`20261007_calib_tilt.log`、`20261007_dual_servo_power.log`。

## 指导与边界

**优先级：P0**

本阶段只解决：

```text
供电
I2C
舵机方向
机械范围
安全限位
```

云台安装方式按正常 Pan-Tilt 机构处理，可参考常见开源云台程序的控制方法，但具体 PWM 范围必须根据本机实测确定。

调试时禁止直接发送假定的 0°、180°、270° 极限位置。

## 2.1 当前机械安装模型

### Pan

```text
可转范围 > 180°
预计 < 270°
正常正向位置约位于机械量程中部
```

因此：

```text
Pan Home ≈ Center
```

### Tilt

摄像头没有安装在垂直舵机的机械端点。

当前正常工作位置约为：

```text
距离某一机械端点回转约 70°
```

因此正常工作位置两侧均可能存在运动空间，但靠近该端点的一侧剩余量程较小。

第一阶段不要假定正负角度，先通过小步 PWM 调整确认：

```text
tilt_min_pwm
tilt_home_pwm
tilt_max_pwm
```

## 2.2 供电

```text
12V
 ├── IMX6ULL
 └── RK-770 → 舵机电源

IMX6ULL 逻辑电源 → PCA9685 VCC
RK-770 舵机电源 → PCA9685 V+
所有模块共地
```

先实测输出，再接舵机。

## 2.3 I2C 验证

```bash
i2cdetect -y <bus>
```

确认：

```text
PCA9685 可见
AP3216C 原有访问正常
```

## 2.4 标定方式

第一阶段使用：

```text
PWM tick / pulse width
```

而不是直接使用角度。

Pan：

```text
当前正向位置
    ↓
记录 pan_center_pwm
    ↓
小步向一侧移动
    ↓
记录安全边界
    ↓
回 Center
    ↓
向另一侧重复
```

Tilt：

```text
当前正常工作位置
    ↓
记录 tilt_home_pwm
    ↓
分别向两侧小步探索
    ↓
一旦接近机械限制立即停止
```

保存：

```text
config/pantilt_calibration.conf
```

内容：

```text
pan_min_pwm
pan_center_pwm
pan_max_pwm
pan_direction

tilt_min_pwm
tilt_home_pwm
tilt_max_pwm
tilt_direction
```

实际测出总行程后再建立：

```text
PWM ↔ degree
```

映射。

## Phase PASS

- PCA9685 可访问。
- Pan/Tilt 均能在安全小范围内动作。
- 当前 Home 位置可重复返回。
- 双舵机动作不引起明显供电异常。
- 得到第一版安全 PWM 范围。

第一轮不要求精确测量完整角度。

---

# Phase 3：Pan/Tilt 控制 + 视频链路 + 自动目标跟踪

> **详细执行计划见 [`plan/phase3_pantilt_tracking.md`](phase3_pantilt_tracking.md)**（2026-10-07 制定）。
> 其中包含协商结论、3A–3D 里程碑拆分、接口/协议草案、板端资源账与待实测项。
> 本章以下内容为原始需求，两者冲突时以详细计划为准并回写本章。

## 指导与边界

**优先级：P0**

这是项目第一条核心闭环。

目标跟踪算法从最简单方案开始，重点是尽快实现：

```text
目标移动
    ↓
云台自动跟随
```

程序和算法实现优先参考 `reference/` 中的示例以及成熟开源实现。

本阶段不做复杂深度学习推理，不先做 PID，不追求高精度性能指标。

---

## 3.1 Pan/Tilt 最小控制程序

先建立（**Phase 2 已建成并完成标定**，路径按 2026-10-07 结构整改为）：

```text
imx6ull_project_src/user/pantilt_test/
```

控制路径：

```text
/dev/i2c-X
    ↓
PCA9685
    ↓
MG90S Pan / Tilt
```

第一版支持：

```text
home
pan
tilt
set
stop
```

所有命令先做软件限位。

---

## 3.2 正式云台控制接口

硬件控制稳定后建立：

```text
imx6ull_project_src/kernel/pantilt/
```

目标设备：

```text
/dev/pantilt0
```

建议接口：

```text
SET_TARGET
GET_STATE
HOME
STOP
SET_LIMIT
```

第一版允许继续使用 PWM/tick，不要求立即转换为真实角度。

增加：

```text
pantilt_daemon
```

用于 PC 与板端控制交互。

第一版先实现：

```text
PC 按键/滑块
    ↓
socket
    ↓
IMX6ULL
    ↓
/dev/pantilt0
    ↓
PCA9685
```

手动控制稳定后再接自动控制。

---

## 3.3 OV5640 视频与 Ring Buffer

视频路径：

```text
OV5640
  ↓
V4L2
  ↓
固定长度 Ring Buffer
  ↓
编码/发送
  ↓
PC
```

不使用 Latest Frame。

Ring Buffer 第一版建议：

```text
4~8 frame slots
```

每帧至少记录：

```text
frame_id
timestamp
size
buffer/slot
```

Producer：

```text
V4L2 capture
```

Consumer：

```text
encode / network
```

满时：

```text
覆盖或丢弃最旧帧
```

并记录：

```text
read_idx
write_idx
count
high_watermark
drop/overwrite_count
```

第一版复用现有已验证远程视频方案，不重新实现视频协议。

---

## 3.4 最简单目标跟踪

第一选择：

```text
PC OpenCV HSV 颜色目标
```

处理：

```text
HSV threshold
    ↓
binary mask
    ↓
简单形态学
    ↓
最大有效轮廓
    ↓
target center
```

演示目标可使用明显颜色标记。

如果颜色阈值不稳定，再换：

```text
MeanShift / CamShift
```

进一步增强才考虑：

```text
PC 端预训练 person 模型
```

深度学习模型只运行在 PC，不作为第一版条件。

将目标定位封装成：

```text
get_target_center(frame)
```

后续更换算法时，云台控制接口保持不变。

---

## 3.5 自动跟踪控制

计算：

```text
error_x = target_x - image_center_x
error_y = target_y - image_center_y
```

第一版：

```text
deadband
+
P control
+
最大单次步长
+
机械限位
```

控制链：

```text
target center
    ↓
error_x / error_y
    ↓
P control
    ↓
Pan/Tilt command
    ↓
pantilt_daemon
    ↓
/dev/pantilt0
```

任何控制结果都必须经过 Phase 2 标定出的安全范围。

---

## 3.6 基础状态和保护

云台状态：

```text
MANUAL
TRACK
LOST
HOLD
HOME
```

第一版：

```text
LOST → HOLD
```

即可。

网络命令增加：

```text
seq
timestamp
mode
pan command
tilt command
```

避免明显重复或过期命令。

---

## 3.7 PC 展示

PC 是第一版主要展示端。

叠加：

```text
视频
目标轮廓 / bbox
目标中心
画面中心
error_x / error_y
Pan/Tilt command
TRACK / LOST
FPS
Ring Buffer 状态
```

LCD 暂时只保留本地显示能力。

后续可显示：

```text
STATE
PAN
TILT
FPS
TARGET
SENSOR
NETWORK
```

---

## Phase PASS

第一版只要求：

- PC 可手动控制云台。
- 视频能持续传到 PC。
- 视频经过 Ring Buffer。
- 颜色目标能得到中心坐标。
- 目标左右移动时 Pan 能跟随。
- 目标上下移动时 Tilt 能在安全范围内跟随。
- 目标回到中心附近后云台停止明显持续调整。
- 目标丢失后进入 HOLD。
- 不出现明显越限动作。

达到以上条件，即认为自动跟踪主闭环完成。

---

# Phase 4：传感器框架迁移（ICM20608 → IIO，AP3216C → Linux Input）

> 详细执行计划与实测记录见 `plan/phase4_sensor_framework.md`。
> 本节原来的写法是"两颗传感器都迁 IIO"，2026-10-07 协商后修正为**分走两个框架**，理由见 4.2。

## 指导与边界

**优先级：P1**

自动跟踪完成后再改传感器框架。

迁移原则：

```text
保留旧字符设备
    ↓
新增框架模块
    ↓
A/B 验证
    ↓
系统集成
```

不直接删除当前可运行实现。

## 4.1 ICM20608 → IIO

先实现：

```text
iio_dev
iio_info
iio_chan_spec
read_raw
scale
```

第一版只要求：

```text
/sys/bus/iio/devices/iio:deviceX/
```

能够读取加速度和角速度数据。

之后再增加：

```text
IIO buffer
kfifo
trigger
timestamp
```

本轮只做到 sysfs raw 级。`CONFIG_IIO_BUFFER=n` / `CONFIG_IIO_TRIGGER=n`，**不改内核配置**。

## 4.2 AP3216C → Linux Input

**不迁 IIO**，走 Linux Input 子系统：

```text
EV_SW / SW_FRONT_PROXIMITY
```

理由：参考材料（`reference/王政的项目.txt` 指向的 AP3216C 驱动）本身就是 Input 方案；
AP3216C 的 PS 本质是"近/远"二值状态开关，Input 的 switch 语义天然匹配，
而 IIO 的通道模型更适合连续量。硬塞进 IIO 反而要多写一层 channel 映射。

- 新建独立模块 `ap3216c_input.ko`，以现有 `ap3216c_myself.c` 为底本 fork；
  旧模块原样保留，做 A/B 与回退。
- ALS 与 PS 原始值通过 sysfs（`proximity/`、`ambient_light/`）导出。
- 保留已有 IRQ/workqueue 处理方式。
- 分两步：M1 最小版（框架通、能读数、能出事件）；M2 补 runtime PM 与 sysfs 校准节点。
- 判近/远用**软件比较 PS 与阈值**，不用 `PSDATA` 的 bit7（该位在 ZONE 模式下恒为 0，
  旧实现因此完全失效）。阈值带内保持上次状态，天然带滞回。

## 4.3 与 mx6hub 的关系

根据实际复杂度选择：

```text
mx6hub 继续通过驱动内部接口取数据
```

或：

```text
用户态通过 IIO + epoll 获取
```

以改动小、稳定为优先。**本轮不动 mx6hub。**

## Phase PASS

- ICM20608 可通过 IIO 读取（`/sys/bus/iio/devices/iio:deviceX/`）。
- AP3216C 可通过 **Input 事件 + sysfs raw** 读取（不是 IIO）。
- 原有字符设备路径仍可回退。
- 新旧接口数据基本一致。
- IIO buffer / kfifo / trigger / timestamp 本轮推后，不计入 Phase 4 PASS。

## 实测结论（2026-10-07 完成）

板端 `scripts/board_verify_sensors.sh` 一次跑完 6 段，总判定 PASS：

```text
ICM20608 -> IIO      : PASS   WHO_AM_I=0XAF，静止 |a|≈2034（≈1g）
AP3216C  -> Input    : PASS   绑定 + sysfs raw + /dev/input/eventX 齐备
A/B 对照 + 回退      : PASS   ICM20608 新旧 |a| 差 2 LSB；旧模块可重装读回
EV_SW 人工双向事件   : PASS   手掌贴近/移开 5 个来回，value=1/0 全部检出
M2 runtime PM        : PASS   suspended -> active -> suspended
M2 sysfs 校准节点    : PASS   4 个节点读写往返正确
M2 空闲无中断风暴    : PASS   空闲 3s 中断增量 = 0
```

两个反直觉的坑（已写入 `logs/phase4/` 与记忆）：

1. **`PSDATA` bit7 不能用来判远近** —— 它只在 HYSTERESIS 模式有意义，ZONE 模式下恒为 0。
   旧实现据此判定，导致挥手完全没反应。
2. **PS 中断模式必须用 HYSTERESIS** —— ZONE 模式下手不在时 PS 恒在低阈值下方，
   每次测量都触发一次中断（实测 13 次/6s），把 runtime PM 的 autosuspend 反复打散，
   省电完全失效。HYSTERESIS 只在跨阈值时报中断，空闲 6s 零中断。

---

# Phase 5：WM8960 ALSA 应用补全 —— 已实现（2026-10-07）

展开文档：[`plan/phase5_audio_alsa.md`](phase5_audio_alsa.md)，验收日志：[`logs/phase5/`](../logs/phase5/)。

## 指导与边界

**优先级：P1**

当前需要补的是用户态 ALSA 调用程序，不重新开发 WM8960 Codec 驱动。

禁止在 IMX6ULL 上运行深度学习音频算法。

## 5.1 录音程序

建立：

```text
imx6ull_project_src/user/audio/
├── audio_capture.c
├── audio_play.c
├── audio_common.c / audio_common.h    # PCM 打开 + XRUN 恢复 + WAV 头，两个程序共用
├── Makefile                           # 含板端 libasound 的链接说明
└── README.md                          # 用法、板端参数限制、给 securityd 的接口
```

录音程序至少支持：

```text
声卡/PCM 设备选择
采样率
声道数
采样格式
录音时长
输出路径
```

建议调用形式：

```bash
./audio_capture \
    --output /root/imx6hub_demo/records/test.wav \
    --duration 10
```

也支持由系统传入任意合法路径：

```bash
./audio_capture --output /指定路径/xxx.wav
```

程序负责：

```text
创建目标文件
初始化 WAV/PCM 参数
snd_pcm_open
设置 hw_params
snd_pcm_readi
处理 XRUN
写入音频数据
正常结束并关闭文件
```

如果输出 WAV，程序需要正确维护 WAV header。

## 5.2 播放程序

实现基础：

```text
snd_pcm_open
hw_params
snd_pcm_writei
XRUN recovery
```

用于播放告警音或测试录音。

## 5.3 安防调用

后续 `securityd` 可以直接触发：

```text
开始录音
指定保存目录
停止录音
播放告警
```

建议默认保存目录：

```text
/root/imx6hub_demo/records/
```

文件名使用时间戳，例如：

```text
20261006_103700_alert.wav
```

## 5.4 可选增强

后续再考虑：

```text
DC removal
high-pass
noise gate
简单频域降噪
```

第一版不影响主系统。**本轮明确不做**（用户 2026-10-07 指示"没有把握暂时不做"），
降噪还会污染原始取证音频，留到确实有需求时再说。

## Phase PASS

- 自定义程序可录音。
- 可以通过命令参数指定保存路径。
- 录音文件可以正常回放。
- 自定义程序可播放测试音频。
- 能处理基本 ALSA XRUN/错误。

## 实测结论（2026-10-07 完成）

板端 `scripts/board_verify_audio.sh` 一次跑完 7 段，总判定 PASS：

```text
环境与依赖           : PASS   声卡 wm8960audio + 录放节点 + libasound + 两程序齐备
默认目录录音+WAV校验 : PASS   records/20261007_180505_vfy.wav，3.00s/132300 帧，峰值非静音
指定路径+参数生效    : PASS   22050/1ch/16bit 与 48000/2ch/32bit 均原样写进 WAV 头
播放(文件/单音/告警) : PASS   三种方式 wall_s 均与音频时长一致（真按实时速率跑）
声学回环 880/1320Hz  : PASS   +20.9dB / +21.6dB（麦克风录到喇叭的声音，不靠人耳）
XRUN 恢复            : PASS   SIGSTOP 1.2s 造 overrun -> xrun=1，恢复后跑满 176400 帧
SIGTERM 停止路径     : PASS   录到 2.46s 收尾，WAV 头已回填，文件可直接回放
```

三个"声明支持但其实不能用"的板端限制（用 `arecord` 独立复现过，不是程序 bug）：
**8000 Hz** 在声明范围内却装不上 hw_params（11025 起才正常）、**S20_3LE** 声明支持但 hw_params 失败、
**U8 / S24_3LE** 在 `set_format` 即被拒。详见 [`plan/phase5_audio_alsa.md`](phase5_audio_alsa.md) §5。

---

# Phase 6：系统集成、诊断与优化

> **执行时对本节的两处回写（2026-10-07）**，详见
> [`plan/phase6_system_integration.md` §5](phase6_system_integration.md)：
> §6.1 的 ALERT 触发源已收窄为**只由 AP3216C 接近事件**触发（ALS 无事件出口、ICM20608
> 在设备树里没有 `interrupts`，"光照变化/振动"不能作为判据）；§6.3 的 `mx6hub` debugfs
> 在本链**够不着**（与 Phase 4 两个新驱动争硬件，本链不加载它）。
> 照着本节原话实现会写出永远不成立的分支或读不到的文件。

## 指导与边界

**优先级：P1**

先完成工具箱安防业务闭环，再针对真实问题进行诊断和优化。

本阶段将原来的“系统集成”和“系统联调优化”合并。

第一轮不设置高性能门槛，只要求系统可运行、可观测、可定位。

---

## 6.1 安防状态机

建立用户态：

```text
securityd
```

负责：

```text
传感器事件
告警状态
视频启停
目标跟踪模式
云台状态
WM8960 录音/告警
网络状态
LCD 状态
```

第一版状态机：

```text
GUARD
  │
  ├─ AP3216C / ICM20608 event
  ▼
ALERT
  │
  ├─ video on
  ├─ audio capture
  └─ tracking ready
  ▼
TRACK
  │
  ├─ PC target available
  └─ Pan/Tilt follow
  ▼
HOLD / GUARD
```

典型事件：

```text
光照变化 → 可能开箱
接近事件 → 有人靠近
振动/运动 → 箱体被移动
```

> **2026-10-08 回写**（Phase 6 §5 承诺的"冲突回写"，此前未落）。第一版实现与本节的差异：
>
> - **ALERT 只由 AP3216C 接近事件触发**，不是 `AP3216C / ICM20608 event`。ALS 没有事件出口；
>   ICM20608 的 INT 脚本轮未接进设备树。**但"ICM20608 没有中断引脚"是错的**——`6D_INT`
>   已引至 `GPIO1_IO10`（＝`JTAG_MOD`），只是没声明 `interrupts`。见
>   [Phase 6 §1 事实 2](phase6_system_integration.md) 与 [Phase 7](phase7_wom_patrol_debug.md)。
>   **Phase 7 结果**：该脚在板上被外部恒定拉低、零边沿（M1 门没过），最终走的是
>   **软件阈值**（`securityd --vib-thr-mg`），**未改 DTS / 驱动**；振动事件经
>   `securityd → pantilt_daemon → 遥测 reserved[1] → PC` 触发 pan 定幅巡检。
> - **TRACK 的语义是"跟踪就绪"**，板端不感知 PC 是否真在跟踪目标；本节"PC target available /
>   Pan/Tilt follow"由 PC 侧单独成立，板端没有对应判据。
> - "LCD 状态"未实现（不在一版范围）。

第一版只使用明确规则，不引入复杂融合算法。

---

## 6.2 epoll 与进程协同

`securityd` 可统一监听：

```text
sensor fd
control socket
timerfd
```

其他模块保持独立：

```text
pantilt_daemon
video sender
audio_capture
```

先保证职责清晰，再决定是否进一步合并进程。

---

## 6.3 Debugfs

> **2026-10-08 回写**：本节的 `mx6hub: fifo/irq/drop/latency` **在本链上够不着** ——
> Phase 4/6 的链不加载 mx6hub（它与 `ap3216c_input`/`icm20608_iio` 争 AP3216C 的 `0-001e`
> 与 SPI），运行 DTB 里也没有 mx6hub 节点。实际的调试面是 **云台驱动自带的
> `/sys/kernel/debug/pantilt/{state,limit,stats}`** 加 `securityd` 自己的 STATS/status 文件。
> `scripts/debug/` 下四个对着 `mx6hub_*` 的脚本因此已是死件（见 Phase 7 备注）。

继续使用：

```text
mx6hub:
    fifo
    irq
    drop
    latency
```

云台增加：

```text
command
limit
i2c error
update latency
```

Ring Buffer 记录：

```text
count
high watermark
drop/overwrite
```

---

## 6.4 Ftrace / Kprobes

Ftrace 重点观察：

```text
IRQ
workqueue
I2C/SPI
pantilt control path
scheduler latency
```

Kprobes 仅在需要量化时挂点：

```text
i2c_transfer
spi_sync
mx6hub enqueue/dequeue
pantilt update
```

---

## 6.5 GDB / gdbserver

重点调试：

```text
securityd
pantilt_daemon
V4L2 用户态程序
audio_capture
控制协议
```

---

## 6.6 系统优化

### Ring Buffer

比较：

```text
N = 4
N = 6
N = 8
```

观察：

```text
drop/overwrite
queue depth
视频卡顿
端到端延迟
```

### 传感器

检查：

```text
IRQ 频率
workqueue 堆积
IIO buffer
mx6hub 丢包
```

### 视频

依次定位：

```text
V4L2
Ring Buffer
编码
socket
PC decode
tracking
```

### 音频

检查：

```text
ALSA period
buffer
XRUN
CPU
```

### 云台

检查：

```text
控制频率
命令延迟
I2C 错误
限位次数
抖动
目标误差
```

---

## 6.7 第一版 Benchmark

统一保存：

```text
FPS
video ring depth
video drop
sensor drop
control rate
control delay
I2C/SPI error
tracking error
ALSA XRUN
CPU
memory
```

不要求一开始达到严格阈值，只用于定位问题和做前后对比。

---

## Phase PASS

至少完成一条完整业务链：

```text
传感器触发
    ↓
ALERT
    ↓
OV5640 视频进入 PC
    ↓
目标被识别/跟踪
    ↓
云台自动跟随
    ↓
WM8960 录音保存到指定目录
```

并满足：

- 系统无明显死锁或失控。
- 云台不越过标定范围。
- 主要模块状态可观测。
- 至少使用一种调试工具定位过真实问题。
- 至少完成一次有数据的优化前后对比。

---

# 4. 推荐工程结构

逐步收敛为：

```text
IMX6ULL-Safe-Alarm/
├── Makefile
├── README.md
│
├── rules/
├── reference/
│   └── pantilt_tracking.md
│
├── config/
│   └── pantilt_calibration.conf
│
├── kernel/
│   ├── active/
│   │   ├── ap3216c/
│   │   ├── icm20608/
│   │   └── mx6hub/
│   ├── pantilt/
│   ├── iio/
│   ├── input/
│   ├── dts/
│   │   ├── active/
│   │   ├── reference/
│   │   └── archive/
│   └── experiments/
│
├── user/
│   ├── hub_monitor/
│   ├── sensor_alert_app/
│   ├── v4l2_fb_preview/
│   ├── pantilt_test/
│   ├── pantilt_daemon/
│   ├── audio/
│   └── securityd/
│
├── pc/
│   └── tracker/
│
├── scripts/
│   ├── verify/
│   ├── deploy/
│   ├── debug/
│   ├── pantilt/
│   └── remote_link/
│
├── docs/
│   ├── plan_imx6ull_pantilit.md
│   ├── repo_inventory.md
│   ├── baseline_2026_pantilt.md
│   ├── pantilt_hardware.md
│   └── protocol.md
│
├── archive/
└── out/
```

这是目标结构，不要求 Phase 1 一次性物理迁移完成。

2026-10-08 现状：程序源文件已统一收进 `imx6ull_project_src/`（`kernel/`、`user/`、`out/`、
`pc/`；`temp/` 已删），`Makefile` 与 `scripts/` 留在工程根目录只做调度，`config/` 在根目录
（`pantilt_calibration.conf`）。上图是**目标结构**：其中 `pc/` 已作为
`imx6ull_project_src/pc/` 于 Phase 3 建立；`archive/` 始终未建立；
`docs/plan_imx6ull_pantilit.md` 与 `docs/pantilt_hardware.md` **至今不存在**（总计划就在
`plan/` 下，不做 `docs/` 副本）；`docs/protocol.md` 已由 Phase 3 的 3B 里程碑产出。
`docs/` 下另有 `scripts_shell_guide.md`、`ssh_dev_workflow.md` 与一批 PDF 资料。
实际文件清单以 [工程清单](../docs/repo_inventory.md) 为准。

---

# 5. 自动跟踪技术路线

第一版：

```text
PC OpenCV HSV 颜色目标
+
P 控制
+
PCA9685 / MG90S
```

升级路线：

```text
HSV 颜色中心
    ↓
MeanShift / CamShift
    ↓
通用 ROI Tracker
    ↓
PC 预训练 person 模型
```

每次只替换：

```text
get_target_center(frame)
```

以下链路保持不变：

```text
error
control
network
pantilt
```

---

# 6. 云台机械模型与软件约束

机械安装按正常 Pan-Tilt 结构处理，不需要为安装方式设计特殊算法。

软件只需要基于实测标定值建立安全工作区。

```text
Pan:
    MIN < HOME/CENTER < MAX

Tilt:
    MIN < HOME < MAX
```

其中：

```text
Pan HOME：
正常正向位置，约在水平量程中部。

Tilt HOME：
当前摄像头正常工作姿态，
约距离某一机械端点 70°，
并非机械极限位置。
```

任何参考程序中的：

```text
0°
90°
180°
270°
```

均不得直接作为本机真实动作范围。

正式控制统一使用：

```text
calibrated limit
+
small-step bring-up
+
software clamp
```

---

# 7. Ring Buffer 约定

视频统一使用固定长度 Ring Buffer：

```text
struct VideoRing {
    Frame slots[N];
    uint32_t read_idx;
    uint32_t write_idx;
    uint32_t count;
};
```

满时：

```text
覆盖/丢弃最旧帧
```

并记录：

```text
high watermark
drop/overwrite count
```

目标：

```text
有限缓存
可观测
不无限积压
不使用单槽 Latest Frame
```

---

# 8. 显示策略

第一版主显示端：

```text
PC
```

显示：

```text
video
target
image center
error
Pan/Tilt command
TRACK/LOST
FPS
```

LCD 后续用于板端状态：

```text
GUARD / ALERT / TRACK
Pan/Tilt
FPS
sensor
network
```

LCD 不作为第一版自动跟踪的前置条件。

---

# 9. Phase 统一开发规则

每个 Phase：

```text
最小功能
    ↓
最小验证
    ↓
保存日志/配置
    ↓
再进行增强
```

第一版 PASS 后再增加：

```text
性能
抽象
复杂算法
更高稳定性
```

每个 Phase 至少保留：

```text
commit
测试命令
成功日志
关键配置
```

---

# 10. 第一版最终 Demo

```text
1. IMX6ULL 启动旧系统
2. AP3216C / ICM20608 检测安防事件
3. OV5640 开启视频
4. 视频经 Ring Buffer 发送到 PC
5. PC 找到颜色目标
6. 计算目标与画面中心偏差
7. PC 下发 Pan/Tilt 控制
8. IMX6ULL 驱动 PCA9685
9. MG90S 自动调整摄像头
10. 摄像头持续跟随目标
11. WM8960 开始录音
12. 音频保存到指定路径
13. Debugfs/Ftrace/GDB 用于联调和问题定位
```

---

# 11. 当前最近的实际任务

按以下顺序执行：

```text
① 旧项目重新验证
        ↓
② repo_inventory.md
        ↓
③ DTS / driver / experiment / output 整理
        ↓
④ RK-770 / 共地 / PCA9685 接线
        ↓
⑤ PCA9685 I2C 检测
        ↓
⑥ Pan/Tilt 小步标定
        ↓
⑦ pantilt_test
        ↓
⑧ PC 手动控制
        ↓
⑨ OV5640 + Ring Buffer
        ↓
⑩ PC HSV 颜色目标
        ↓
⑪ P 控制自动跟踪
        ↓
⑫ 传感器框架迁移（ICM20608→IIO，AP3216C→Input）
        ↓
⑬ WM8960 指定路径录音
        ↓
⑭ securityd 系统集成
        ↓
⑮ Debugfs/Ftrace/Kprobes/GDB 与性能优化
        ↓
⑯ Phase 7：ICM20608 WoM 中断、pan 定幅巡检、ftrace 诊断
        （⑮ 里只有 ftrace 一项被 ⑯ 接管；其余性能优化仍留在 ⑮）
```

在自动跟踪闭环成功前：

```text
不做 PID
不做 YOLO
不做复杂 LCD UI
不做音频降噪
不做大规模性能调优
```

---

# 12. 参考

- `云台任务规划和清单.md`
- `项目地址.txt`
- `IMX6ULL-Safe-Alarm/docs/plan.md`
- `IMX6ULL-Safe-Alarm/README.md`
- [RK3588-Blind/plan/plan_v2.md](https://github.com/pythonerlower/RK3588-Blind/blob/main/plan/plan_v2.md)
- [两自由度云台目标跟踪开源参考](../reference/pantilt_tracking.md)
- `rules/`
- `reference/`
- `I.MX6U 嵌入式 Linux C 应用编程指南 V1.4`
- `I.MX6U 嵌入式 Linux 驱动开发指南 V1.81`
- `IMX6ULL Reference Manual`
- `IMX6ULL 核心板引脚分配图`

---

# 13. 一句话执行路线

> **先验证并整理现有 IMX6ULL 工程，再完成 PCA9685 + 双 MG90S 的安全标定；随后将 Pan/Tilt 控制、OV5640 Ring Buffer 视频链路和 PC 端最简单目标跟踪合并开发，尽快实现自动跟随效果；主闭环完成后再做传感器框架迁移（ICM20608→IIO、AP3216C→Input）、补全 WM8960 指定路径录音应用，并最终完成工具箱安防系统集成、调试和优化。**
