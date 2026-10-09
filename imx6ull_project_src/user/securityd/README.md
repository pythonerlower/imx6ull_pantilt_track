# securityd：板端安防状态机

Phase 6 的交付主体。前五个 Phase 各自验过的模块（OV5640 / PCA9685+MG90S / AP3216C /
ICM20608 / WM8960）在 `securityd` 里第一次作为**一条链**一起跑：

```
AP3216C 接近事件 → ALERT → OV5640 视频进 PC + 录音落盘 → 跟踪就绪 → 回到 GUARD
```

它只做三件事：判接近、起停录音、盯住视频。**它不碰云台**——不开 `/dev/pantilt0`、
不 include 它的 ABI 头、不发任何控制包（原因见下）。跟踪与云台跟随全部由 PC 侧的
Phase 3 程序经 `pantilt_daemon` 完成。

| 文件 | 作用 |
| --- | --- |
| `securityd.c` | 全部实现（状态机 + 子进程监督 + 摄像头重载 + 可观测性） |
| `Makefile` | 交叉编译，产物 `securityd` |

## 用法

```bash
./securityd --dry-run                     # 只跑状态机，不起任何子进程（M1 用）
./securityd                               # 正常跑
./securityd --video-dst 192.168.137.100:9200 --alert-hold-ms 30000
kill -USR1 $(pidof securityd)             # 立刻 dump 一行 STATS
```

| 选项 | 默认 | 说明 |
| --- | --- | --- |
| `--input-name` | `ap3216c-proximity` | Input 设备名，按 `EVIOCGNAME` 扫 `/dev/input/event*` 自动定位，**不写死 eventX** |
| `--iio-name` | `icm20608_iio` | IIO 设备名 |
| `--video-bin` / `--video-dev` | `./video_link` / `/dev/video1` | 视频程序与 V4L2 设备 |
| `--video-dst` | `192.168.137.100:9200` | 视频目的地址 |
| `--video-size` / `--video-fps` / `--video-slots` | `320x240` / `15` / `8` | 采集参数 |
| `--video-ready-ms` | 5000 | 多久没发包就判摄像头故障（正常首条 `[stats]` 在 1 s 内） |
| `--audio-bin` | `./audio_capture` | 录音程序 |
| `--records-dir` / `--audio-tag` | `/root/imx6hub_demo/records` / `alert` | 录音目录与文件名标签 |
| `--audio-warmup-ms` | 2000 | 启动热身录音毫秒数，0 跳过（**必须有**，见下） |
| `--alert-hold-ms` / `--alert-max-ms` | 20000 / 120000 | 最近一次 near 之后保持告警的时长 / 单次告警硬上限 |
| `--near-guard-ms` | 200 | 重复 near 的最短间隔 |
| `--tick-ms` | 100 | 主循环节拍 |
| `--als-poll-ms` / `--vib-poll-ms` | 2000 / 100 | ALS、振动采样间隔（振动间隔必须快过要抓的那一下） |
| `--vib-thr-mg` | 0（关闭） | 振动事件阈值 = **逐轴帧间最大变化量**（mg）。**这是软件阈值，不是中断**；0 = 行为与 Phase 6 完全一致 |
| `--vib-burst-ms` / `--vib-burst-hold-ms` | 20 / 2000 | 超阈值期间压到的采样间隔 / 超阈值后保持高频的时长 |
| `--event-sock` | 空串（不发） | 事件出口，`AF_UNIX/SOCK_DGRAM`，对端 `pantilt_daemon` |
| `--stats-ms` | 1000 | STATS 行间隔 |
| `--camera-reload-attempts` / `--camera-mod-dir` | 3 / `/root/imx6hub_demo` | 摄像头重载上限与 `.ko` 所在目录 |
| `--status-file` / `--log-dir` | `/run/securityd.status` / `/root/imx6hub_demo/logs/securityd` | 状态文件与日志目录 |
| `--dry-run` / `-v` / `-h` | — | 只跑状态机 / 打印被忽略的事件 / 帮助 |

## 状态机

```text
GUARD ──(EV_SW value=1，near)──▶ ALERT ──(video_link 发包>0)──▶ TRACK_READY
                                   ▲                                 │
                                   └──────(新 near 续期)─────────────┘
TRACK_READY ──(now - last_near > alert_hold_ms 或 触及 alert_max_ms)──▶ STOPPING
STOPPING ──(录音收尾完成)──▶ GUARD
```

| 状态 | 在做什么 |
| --- | --- |
| `GUARD` | 等接近事件。视频**常开**（开机就起），录音停着 |
| `ALERT` | 录音中，等视频证明自己在发包 |
| `TRACK_READY` | 视频已确认在发包，人还在附近。等 `alert_hold` 到点 |
| `STOPPING` | **只收音频**：SIGTERM → 等 WAV 头回填。视频不动 |

`DEGRADED(no_sensor\|camera\|audio)` 是**正交标志位**不是状态，出现在 STATS 行的 `deg=`
字段里，可以同时多个。

### 判据为什么是这样

- **`near` 只由 AP3216C 的 `EV_SW`/`SW_FRONT_PROXIMITY` 触发**。ALS 根本没有事件出口；
  ICM20608 有 INT 脚（`6D_INT` → `GPIO1_IO10`）但**在本板挂不上中断**——Phase 7 的 M1
  把该脚临时配成 GPIO 输入高速采样，几万样本零边沿，`request_irq` 无从触发
  （证据 [`logs/phase7/m1_icm_int_probe_20261008.md`](../../../logs/phase7/m1_icm_int_probe_20261008.md)）。
  所以振动判据走的是**软件阈值**（`--vib-thr-mg`），不是唤醒中断；ALS 与振动都只做定时
  轮询采样，进日志与统计，不进 `near`。
- **`TRACK_READY` 只看"video_link 真的在发包"**，不看退出码也不看进程存活：
  `video_link` 退出码 3 表示摄像头故障，但"进程活着、一个包都没发"它报不出来。
  判据取 `[stats]` 行里的**发包数 > 0**。
- **不做 N 帧确认**：驱动判 NEAR 已带迟滞、Input 层对 `EV_SW` 去重，边沿本身是干净的；
  N 帧确认只会滤掉真实的快速靠近。`--near-guard-ms` 只防一次靠近被报两回。

## 有序退出与故障处置

**有两条路径，收尾动作不一样，别混。** 定时值都是硬编码的，不在命令行上：

| 路径 | 触发 | 音频 | 视频 |
| --- | --- | --- | --- |
| **告警收尾** `stop_begin()` | `alert_hold` 到点 / `alert_max` 到顶 | `SIGTERM`；**4 s** 未退 `SIGKILL`；整个 `STOPPING` **12 s** 死线 | **一个信号都不发**——视频常开，告警结束不掐它 |
| **程序退出** `final_stop()` | `SIGINT` / `SIGTERM` | `SIGTERM`；**3 s** 未退 `SIGKILL`（再等 2 s） | `SIGTERM`；**6 s** 未退 `SIGKILL`（再等 2 s）——它要归还 `/dev/video1` |

音频的兜底给得晚（4 s）不是给得早：WAV 头没回填就是 44 字节废文件，属不可恢复的
数据损坏，**宁可多等，不可杀早**。告警收尾刻意不碰视频：掐掉它下一次告警又得从头
`probe /dev/video1`，正是决策 #2 要避免的事。

两条路径共同的尾巴：`final_stop()` 之后**原子写状态文件**（临时文件 + `rename`，
**绝不在原文件上就地改写**）→ 打 `RESULT` 行；两条都**不动云台、不重启板子**。

> **子进程必须能真的收到 `SIGTERM`。** 本程序用 `signalfd`，`main` 里把
> `SIGINT/SIGTERM/SIGCHLD/SIGUSR1` 全 `SIG_BLOCK` 掉了；而 `fork()` **继承信号掩码**、
> `execv()` 也不清除它 —— 子进程起来时 SIGTERM 就是被屏蔽的，自己 `sigaction` 装 handler
> 也永远不跑。所以两处 fork 子分支在 `exec` 前都要 `sigprocmask(SIG_SETMASK, &empty, NULL)`
> （`child_unblock()`）。漏掉它的实测表现是 `audio_capture` 收到 SIGTERM 后 4 s 毫无反应、
> 被兜底 SIGKILL，**WAV 头停在 `RIFF 00000000` 占位值**，录制正常但文件播不了。

摄像头故障时在 securityd 进程内直接有序重载（不需要 ssh、不需要移植脚本）：

```
SIGTERM 上一份 video_link 并 waitpid     （否则 /dev/video1 被占用，rmmod 会 in use）
/sbin/rmmod mx6s_capture                 （顺序不是加载顺序的逆序：
/sbin/rmmod ov5640_camera                 理由是运行时的 subdev 引用，不是符号依赖）
/sbin/insmod <dir>/mx6s_capture_fixed.ko
/sbin/insmod <dir>/ov5640_camera.ko
test -e /dev/video1 → sleep 2            （probe 后静置）
```

重试至多 `--camera-reload-attempts` 次；耗尽置 `DEGRADED(camera)`，
**保留音频、绝不 reboot、绝不碰 pantilt**。板端 `/sbin` 不在 `PATH`，一律写绝对路径。

## 可观测性

统一前缀 `securityd:`，纯 ASCII，`key=value` 定序：

```text
securityd: EVT   ts=4102 from=GUARD to=ALERT reason=prox_near ps=142
securityd: STATS ts=5100 state=TRACK_READY up=5.1 near=1 far=0 ps_irq=7 als=44 vib_mg=3 vib_ev=0 vib_thr=0 \
                 video=up vin=612 vout=610 vovr=0 vsend=87320 verr=0 \
                 audio=rec arec_s=3.1 cam_reload=0 deg=none
securityd: RESULT alerts=1 recorded=1 video_ok=1 video_reload=0 near=1 far=1 uptime_s=64 deg=none
```

- 每 tick 用**与 STATS 同内容**原子写 `--status-file`，别的脚本无 IPC 即可读当前态。
- 子进程原始 stdout/stderr 原样落 `--log-dir/securityd.log`（含 `[stats]` 逐秒对账）。
- `video=` 三态：`down`（没跑）/ `init`（跑了但还没证实在发包）/ `up`（在发包）。
- `SIGUSR1` → 立刻 dump 一行 STATS。
- **没有任何"注入 ALERT"的入口**（socket / 信号 / 文件）。协商要求 ALERT 只由 AP3216C
  中断触发，留个后门等于留一个可以伪造验收的口子。

### 启动自检与热身

- 传感器：找不到 `*-001e` 的 I2C 目录、找不到 IIO 设备、找不到 Input 设备，
  一律置 `DEGRADED(no_sensor)` 并**大声打**——**不许静默停在 GUARD**。
  ICM20608 复位后第 1 次读是垃圾值，自检里先丢一次再 `usleep(150ms)` 重读。
- 录音：`--audio-warmup-ms` 先录一小段丢掉。录音方向坏掉时的唯一恢复是
  `/sbin/reboot`，且重启后第一遍会擦边——不热身的话重启后**第一个真实 ALERT
  很可能拿到空文件**。热身在**启动时**就暴露问题，而不是等第一次告警。
  本程序**不自动重启板子**，只置 `DEGRADED(audio)` 并保留视频。

## 已知局限（不掩盖）

1. **"锁定与否"只有 PC 能证。** 板端能看到的只有"video_link 真的在发包"与"录音在落盘"，
   这只是必要条件和不是充分条件。
2. **`cmd_count` 旁证已放弃。** 总计划 §6.1 设想的"用 `GET_STATE` 的 `cmd_count` 增长率
   作为对端正在驱动控制面的旁证"**做不了**：驱动 `pt_open()` 只允许一个打开者
   （`pantilt.c` 注释写明"避免两方争抢同一路 PWM"），这条链上 `pantilt_daemon` 一直占着它。
   2026-10-07 实测：daemon 运行期间用 `pantilt_ctl info` 打开 `/dev/pantilt0` 得到 `EBUSY`。
   代价是 M5 的板端断言因此更弱——如实记录，不包装成"识别成功"。
3. **视频常开。** `securityd` 启动即起 `video_link`，之后只在故障时重载；告警结束时
   也不收视频。代价是无人时也在占 CPU 与带宽（单核 A7 上这是实打实的开销），
   换来的是人走到跟前时摄像头已经在发包，不给 `probe /dev/video1` 留时间。
4. **`ps_int_mode` 必须是 1（HYSTERESIS），不得传 0**（ZONE 模式空闲期中断风暴），
   这是加载 `ap3216c_input.ko` 时的参数，不归本程序管，但链上少了它会出事。
5. **每次开机都要重新 unbind**：`echo 0-001e > /sys/bus/i2c/drivers/ap3216c/unbind`
   （BSP 每次开机都会重新绑定），然后 `insmod ap3216c_input.ko ps_measure_interval=0x01`。
6. **`prox_counts` 的 `irq=` 空闲期恒为 0**，不能当"中断还活着"的健康信号。
7. **`mx6hub` 不参与这条链**（与 Phase 4 两个新驱动争同一批硬件），本链**不加载**它。
   所以可观测性里没有 §6.3 提的 `mx6hub: fifo/irq/drop/latency`，改由 `ap3216c_input`
   的 sysfs 只读量与内核 debugfs `/sys/kernel/debug/pantilt/{state,limit,stats}` 承担。

## 编译

```bash
make            # 交叉编译，产物 securityd
make clean
```

`CC` 用 `CROSS_COMPILE ?= arm-linux-gnueabihf-` 显式赋值而不是 `CC ?=`：make 内置
`CC=cc` 会让 `?=` 失效，静默产出 x86 二进制。**不链 `-lm`**——Phase 7 把振动判据从
"合矢量幅值"换成"逐轴帧间差分"后就不再开方了。

## 验收

板端 `bash scripts/board_verify_phase6.sh`（分段、一次跑完、给 PASS/FAIL），
端到端 `bash scripts/phase6_e2e_run.sh`（开发机侧编排 + 证据收集）。
`scripts/board_verify_phase6.sh` 必须先热身跑一遍验收再判定录音段。

板端脚本共 11 段，其中两段要操作者在场，可用环境变量跳过（跳过记 `NOT RUN`，**不拉低总判定**）：

```bash
WITH_HAND=0 sh ./board_verify_phase6.sh      # 跳过 [4] 人工靠近
WITH_EXHAUST=0 sh ./board_verify_phase6.sh   # 跳过 [11] 重载耗尽（同样要操作者）
M4_STOP_DAEMON=1 sh ./board_verify_phase6.sh # [10] 允许临时停 pantilt_daemon 取 GET_STATE
```

**两段人工的动作要求正好相反**，现场别搞混：

| 段 | 要什么 | 为什么 |
| --- | --- | --- |
| [4] | 贴住 2~3 秒 → 移开 → **之后不要再靠近** | 这一段的终点是 `to=GUARD reason=stopped`。每次新的 near 都会把 8 s 的 `alert_hold` 重新计时，反复挥手会让它永远回不到 GUARD |
| [11] | 触发一次后，**在这十几秒里反复再靠近几次** | 这一段恰恰要告警一直续着：重载 3 次要 5~10 s，不续期录音会先于 `DEGRADED(camera)` 停下，「音频仍在录」就成了假 FAIL |

录音的判据在 **[6]** 而不在 [5]：录制中的 WAV 头是 `RIFF 00000000` 占位值，`wave.open()`
直接抛 "not a WAVE file"，就地判必然假失败。到 [6] 收尾之后文件才定稿，那时才判
WAV 头与长度自洽，并把 `RESULT` 的 `recorded` 与磁盘上的文件交叉核对。

`M4_STOP_DAEMON` **默认为 0**：脚本平时不碰 daemon。停它会让驱动的 1500 ms 看门狗
按设计触发一次 STOP（云台原地保持，不动），但那确实改变了现场状态，所以必须显式开。
