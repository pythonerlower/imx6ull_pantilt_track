# Phase 6 执行计划：系统集成、诊断与优化

本文件是 [总计划](plan_imx6ull_pantilit_final.md) 第 6 章的展开，服从 `rules/` 全部规则。
**状态（2026-10-08）：板端验收 11 段全 PASS；M0–M7 全部有结论，仅 M1 仍记部分。**
`board_verify_phase6.sh`（脚本 `9719eea7…`）一次跑完 11 段、**总判定 PASS**，其中 [4]/[11]
两段由操作者在场完成；同一版本的脚本在免操作者配置（`WITH_HAND=0 WITH_EXHAUST=0`）下另跑
一次 9 段亦 PASS。逐段结果与证据见 `logs/phase6/board_verify_20261008/`。
M5 抓出并修掉 `securityd` 的信号掩码缺陷，随后两轮实跑又修掉两处可观测性缺陷
（`recorded` 恒 0 与子进程收尾行被丢），并当场返工了 PC 端判据（见 §7、§4.6）。

## 0. 边界与协商结论

总计划 §6.1–6.7 是纲领，没有执行计划文件。本轮从零细化。开工前定下 4 条，执行中不动摇：

| # | 议题 | 结论 | 理由 |
| --- | --- | --- | --- |
| 1 | 范围 | **最小 PASS 第一轮**。ftrace/kprobes/gdb 是**按需工具**不是交付物；§6.6 逐项性能优化推后 | 宪章 §5「最小可验证修改」：先把链跑通、把问题定位，再谈优化 |
| 2 | ALERT 判据 | **只由 AP3216C 接近事件触发**（本轮唯一中断驱动的源）。ALS 与 ICM20608 振动只做定时轮询采样，进日志与统计，不作判据 | ALS 没有事件出口；ICM20608 的 INT 脚本轮未接进设备树（`interrupts` 属性不存在）。**注（2026-10-08 更正，见 §1 事实 2）**：该「未接」是「没接到设备树里」，不是「硬件上没接」——`6D_INT` 实际已引至 `GPIO1_IO10`。**Phase 7 已给出结果**：该脚零边沿、中断作废，振动改走软件阈值（ALERT 仍只由 AP3216C 触发） |
| 3 | TRACK 语义 | **"跟踪就绪"**，`securityd` 不感知 PC 是否真在跟踪目标，因此**不改已冻结的 3B 控制协议** | 板端没有能力判断"识别对不对"，假装能判就是造假 |
| 4 | 诊断对象 | daemon 看门狗触发次数随时长线性增长（Phase 3 遗留，约每 750 条命令 1 次） | §6.4/§6.5 的调试工具正好有一个真实靶子，比空跑一遍工具集有意义 |

### 0.1 开工后追加的两条取舍（2026-10-07）

| # | 议题 | 结论 | 理由 |
| --- | --- | --- | --- |
| 5 | `cmd_count` 旁证 | **放弃**。`securityd` 完全不碰 `/dev/pantilt0` | 见 §5.3，硬件层面做不到 |
| 6 | 视频生命周期 | **常开**：`securityd` 启动即起 `video_link`，只在故障时重载 | 人走到跟前时摄像头必须已经在发包，不给 `probe /dev/video1` 留时间 |

两条都不是我单方面决定的，都是先提出、协商一致后才动的代码（`rules/` 开工前先协商）。

## 1. 关键约束（已核实，不是推断）

1. **运行 DTB 没有 mx6hub 节点** → mx6hub 若加载退化成 1 Hz 心跳轮询。
2. **ICM20608 在 ecspi3，设备树节点无 `interrupts`** → 本轮振动只能轮询。
   AP3216C 反之，DT 里有真实 IRQ（GPIO1_IO01）。
   **2026-10-08 更正**：原文这句后半截写的是「INT 未接 CPU」，**不成立**。ICM20608 的 INT 脚是接了
   的：底板原理图里它引出网络 `6D_INT` → 核心板连接器 J2 第 22 脚 → SoC 的 **`JTAG_MOD` = GPIO1_IO10**
   （与 ATK 模块座的 `GBC_LED` 复用）。三处交叉印证：① 底板原理图 ICM-20608 pin6(INT)→`6D_INT`；
   ②《I.MX6U嵌入式Linux驱动开发指南V1.81》§5.3.22「GBC_LED/GBC_KEY 和 AP_INT/6D_INT 共用
   GPIO1_IO01 和 JTAG_MOD」；③ 核心板原理图 `JTAG_MOD/…/GPIO1_IO10/…`。
   准确表述是「**接了，但没接进设备树**」。两条推论：**(a)** `AP_INT`(GPIO1_IO01) 与
   `6D_INT`(GPIO1_IO10) **是两根不同的脚**，两个中断可以并存（`6D_INT` 的冲突对象是
   `GBC_LED`，不是 `AP_INT`）；**(b)** 运行 DTB 里 `spdif` 为 `disabled`（SPDIF_OUT 是
   `JTAG_MOD` 的复用功能），未见其他节点认领该脚 ——**这个结论只由图纸交叉得出**。
   **2026-10-08 Phase 7 M1 实测结论**：该脚确实未被任何节点认领（`(MUX UNCLAIMED)`），
   但把它临时配成 GPIO 输入高速采样时**电平恒低、零边沿**，`request_irq` 无从触发
   ——所以**中断路线作废**，振动改用软件阈值，未改 DTS / 驱动。
   M1 过程见 [`logs/phase7/m1_icm_int_probe_20261008.md`](../logs/phase7/m1_icm_int_probe_20261008.md)，
   执行计划见 [phase7_wom_patrol_debug.md](phase7_wom_patrol_debug.md)。
3. **`ap3216c_input` 只出 `SW_FRONT_PROXIMITY`**，ALS 只有 sysfs 只读。
   insmod 前必须 `echo 0-001e > /sys/bus/i2c/drivers/ap3216c/unbind`；**每次开机 BSP 都会重新绑定，
   所以每次都得重新 unbind**。`ps_int_mode` 默认 1=HYSTERESIS，**不得传 0**（ZONE 模式空闲期
   中断风暴，Phase 4 §5.2 实测空闲 6 s 13 次）。`prox_counts` 的 `irq=` 空闲期恒为 0，
   **不能当"中断还活着"的健康信号**。
4. **§6.3 要的可观测性基本已存在**：pantilt `stats` 有 `cmd_count`/`clamp_low`/`clamp_high`/
   `i2c_err`/`last_latency_us`；环缓冲有 count/高水位/覆盖丢且周期+退出都打印。
   **不需要新增内核 debugfs 节点。**
5. **`pantilt_daemon` 没有本地 IPC**，stdout 无周期性 stats；但遥测包里有诊断所需的
   `daemon_wd_trips`、`ms_since_ctrl`、`daemon_state`、`cmd_count`、`session_resets`。
   **`--telem-dst H:P` 一旦设置，`send_telem()` 就 20 Hz 无条件发**，不需要任何控制包
   （`pantilt_daemon.c:434-439`）——这是诊断看门狗的现成通道，且开发机侧本来就在收。
   驱动侧 `watchdog_trips`（`GET_STATE` 可读）**恒为 0**，与 daemon 侧不是一回事。
6. **`Auto.render()` 丢看门狗告警的缺陷已在 HEAD 修好**（`wd_warn()` + 回归测试
   `test_auto.py:1392`）。Phase 3 状态文档里"未修"的说法已过时。
7. `video_link` 退出码 **3 = 摄像头故障**；退出码**覆盖不到"进程活着但一个包都没发"**，
   所以子进程就绪判据必须用 `[stats]` 行的**发包数 > 0**。
8. `audio_capture` 用 SIGTERM 收尾会回填 WAV 头并打 `audio: RESULT ...` 可解析行。会话用 `--duration 0`。
9. **i.MX6ULL 是单核 Cortex-A7** —— `securityd` + `video_link` + `audio_capture` + daemon 挤一个核。
   daemon 心跳 20 Hz（50 ms）对 500 ms 看门狗只有 10× 余量，**一次 500 ms 调度停顿就触发一次 trip**。
   这是 #4 诊断与 §6.6 A/B 的主导混杂变量，必须控住。
10. **`/dev/pantilt0` 只允许一个打开者**：`pantilt.c:744` 的 `pt_open()` 用
    `test_and_set_bit(0, &d->opened)`，第二次打开返回 `-EBUSY`。2026-10-07 实测：
    daemon 运行期间 `pantilt_ctl info` 得到 EBUSY。

## 2. 交付物

### 新增

| 路径 | 内容 |
| --- | --- |
| `imx6ull_project_src/user/securityd/securityd.c` | 安防状态机主程序（单文件，与 `pantilt_daemon.c`/`video_link.c` 同构） |
| `imx6ull_project_src/user/securityd/Makefile` | 与 `user/audio/Makefile` 同构 |
| `imx6ull_project_src/user/securityd/README.md` | 用法、状态机图、给脚本的接口、已知局限 |
| `scripts/board_verify_phase6.sh` | 板端分段验收，一次跑完给 PASS/FAIL 总判定 |
| `scripts/phase6_e2e_run.sh` | 开发机侧端到端编排与两边证据收集 |
| `scripts/phase6_telem_capture.py` | M7 用：20 Hz 抓 daemon 遥测并打到达时间戳（**纯开发机侧，板端零改动**） |
| `logs/phase6/` | 分段证据 + 人工操作单 + A/B 原始数据 |

### 修改

| 文件 | 改动 |
| --- | --- |
| `scripts/build_all.sh` | 新增 `securityd` 构建段 |
| `Makefile` | 加 `securityd` phony 目标与 `clean` 条目 |
| `scripts/deploy_offline.sh` | 部署 `securityd` 与新脚本 |
| `README.md`、`docs/repo_inventory.md` | 状态与清单更新 |

**`pantilt_daemon.c` 与内核驱动一行都不改**：M7 所需的 `daemon_wd_trips`/`ms_since_ctrl` 已在遥测包里，
用开发机侧抓包即可归因。这满足宪章 §5「最小可验证修改」。

## 3. 实现要点

### 3.1 状态机

```text
GUARD ──(EV_SW value=1，near)──▶ ALERT ──(video_link 发包>0)──▶ TRACK_READY
                                   ▲                                 │
                                   └──────(新 near 续期)─────────────┘
TRACK_READY ──(now - last_near > alert_hold_ms 或 触及 alert_max_ms)──▶ STOPPING
STOPPING ──(录音收尾完成)──▶ GUARD
```

`DEGRADED(no_sensor|camera|audio)` 是**正交标志位**不建独立状态，出现在 STATS 行的 `deg=` 里。

四个刻意的设计：

- **`ALERT` 只由 `/dev/input/eventX` 的 `EV_SW`/`SW_FRONT_PROXIMITY` 触发**，设备按 `EVIOCGNAME`
  扫 `/dev/input/event*` 匹配 `ap3216c-proximity` 自动定位，**不写死 eventX 编号**。
- **不做 N 帧确认**：驱动判 NEAR 已带迟滞、Input 层对 `EV_SW` 去重，边沿本身是干净的；
  N 帧确认只会滤掉真实的快速靠近。`--near-guard-ms`（200 ms）只防一次靠近被报两回。
- **`TRACK_READY` 判据只有"发包数 > 0"**，不看退出码也不看进程存活（事实 7）。
- **告警结束时只收音频，视频保持常开**：摄像头该不该关不是本程序说了算，而且掐掉它
  下一次告警又要从头 probe。

### 3.2 epoll 装配

| fd | 用途 |
| --- | --- |
| `signalfd`(SIGINT/SIGTERM/SIGCHLD/SIGUSR1) | 退出 / 回收子进程 / 按需 dump 一行 STATS |
| AP3216C `/dev/input/eventX` | **唯一 ALERT 触发源** |
| `timerfd`(`--tick-ms` 100) | 所有周期采样、超时、子进程健康检查 |
| `video_link` / `audio_capture` 管道读端（非阻塞） | 转发日志 + 解析 `[stats]` / `audio: RESULT` |

不进 epoll：所有 sysfs（tick 里同步采样）、网络状态（tick 里读一次 `operstate`）。
用 `signalfd`+`timerfd` 而非 signal handler + poll 超时，是为了让"信号打断系统调用"这条路径消失
——Phase 5 的 `audio_capture` 恰好依赖 `EINTR`，`securityd` 不该有同类脆弱点。

### 3.3 有序退出与摄像头重载

退出顺序：`SIGTERM` 两个子进程 → **先等 audio（3 s，WAV 头没回填是硬损坏）** → 再等 video（6 s，
它要归还 `/dev/video1`）→ 原子写 status 文件 → **不动云台、不重启板子**。

**子进程必须能真的收到 SIGTERM**：本程序为用 `signalfd`，在 `main` 里把
`SIGINT/SIGTERM/SIGCHLD/SIGUSR1` 全 `SIG_BLOCK` 掉了，而 `fork()` 继承信号掩码、
`execv()` 也不清除它 —— 子进程起来时 SIGTERM 是**被屏蔽**的，自己 `sigaction` 装 handler
也没用（信号递不进去）。实测表现：`audio_capture` 收到 SIGTERM 后整整 4 s 毫无反应，
被兜底 SIGKILL，**WAV 头永远不回填**。修法是在两处 fork 子分支 exec 前
`sigprocmask(SIG_SETMASK, &empty, NULL)`。完整记录见 §4.6。

摄像头重载直接在进程内 fork/exec 板端命令（`securityd` 就在板上，不需要 ssh、不需要移植脚本），
顺序蓝本见 `scripts/lib/camera_reload.sh`，**顺序不可颠倒**：

```text
SIGTERM + waitpid 上一份 video_link     （否则 /dev/video1 被占用，rmmod 会 in use）
/sbin/rmmod mx6s_capture                （不是加载顺序的逆序：理由是运行时的 subdev 引用）
/sbin/rmmod ov5640_camera
/sbin/insmod <dir>/mx6s_capture_fixed.ko
/sbin/insmod <dir>/ov5640_camera.ko
test -e /dev/video1 → sleep 2           （probe 后静置）
```

重试至多 `--camera-reload-attempts`（默认 3）；耗尽置 `DEGRADED(camera)`，
**保留音频、绝不 reboot、绝不碰 pantilt**。板端 `/sbin` 不在 `PATH`，一律写绝对路径。

### 3.4 启动自检与热身

- 传感器自检不过（找不到 `*-001e` / IIO / Input）**直接置 `DEGRADED(no_sensor)` 并大声打**，
  不许静默停在 GUARD（宪章 §7）。ICM20608 复位后第 1 次读是垃圾值，自检里先丢一次再重读。
- `--audio-warmup-ms`（默认 2000）先录一段丢掉：`plan/phase5_audio_alsa.md` §4.1 记录重启后
  第一遍录音会擦边，不热身的话重启后**第一个真实 ALERT 很可能拿到空文件**。
  热身在**启动时**暴露"录音方向坏了"，而不是等第一次告警。本程序**不自动重启板子**。

## 4. 实测

### 4.1 M0：全链路共存 smoke（板端，2026-10-07）—— **PASS**

按加载序装齐后同时活 180 s（daemon + `video_link` + `audio_capture`）：

| 判据 | 结果 |
| --- | --- |
| ① 五类设备节点 t=0/t=T 可读 | PASS（`/dev/video0`、`/dev/video1`、`/dev/pantilt0`、`iio:device0`、`event2`） |
| ② dmesg 无 Oops/WARN/I2C·SPI 错误 | PASS（grep 计数 0） |
| ③ `video_link` 终局 | 189 s / 2843 帧 @ 15.0 fps，`覆盖丢=0 发包错=0 丢帧(发送)=0 高水位=1`，退出码 0，从未出现退出码 3 |
| ④ 人工贴近/移开全部检出 | **未跑**（需要操作者，留给 M5） |
| ⑤ 三进程 RSS 增长 <20% | PASS（完全不变：daemon 272 kB / video_link 2496 kB / audio 2072 kB） |
| ⑥ `i2c_err=0`、驱动 `watchdog_trips=0` | PASS |
| ⑦ `uptime` 未变、`lsmod` 无 `in use` | PASS（2289→2469 s，+180 s） |

UDP 接收端：9200 口收 312247 包、maxgap 77 ms、稳定 1650 pkt/s；9101 口收 3808 包、恰好 20/s、maxgap 102 ms。
录音：8216576 帧 / 186.32 s / `xrun=0 suspend=0 short_io=0`，WAV 头已回填。
证据：`logs/phase6/m0_*.log`。

### 4.2 M0 顺带暴露的一件事：`/dev/pantilt0` 拿不到

M0 期间试 `pantilt_ctl info` 得到 **EBUSY**（事实 10）。这不是缺陷是设计（驱动只允许一个打开者，
注释写明"避免两方争抢同一路 PWM"），但它直接推翻了 §6.1 的一个设想，见 §5.3。

### 4.3 M1：骨架（`--dry-run`）

开发机侧用 x86 原生编译跑过 `--dry-run` 冒烟：启动自检在设备缺失时**大声报** `deg=no_sensor` 而不是
静默停在 GUARD，STATS/`status` 文件/RESULT 行格式正常，SIGINT 有序收尾路径走通。
板端完整 M1（含真实 Input 事件驱动 FSM）见 `scripts/board_verify_phase6.sh` 的 [2]/[4] 段。

### 4.4 M6：Ring Buffer slots A/B（板端，2026-10-07）—— **PASS，结论是"N 不敏感"**

脚本 `scripts/phase6_m6_ab.sh`，每轮 120 s，同一会话（uptime 全程未变、无 reboot），
PC 侧 `video_rx.py` 无窗口。第 2 组把 PC 侧负载换成**真跑检测**的 `main.py --auto`
（`--deadband 100000` 保证一条运动指令都不发，云台不动）。

板端 `[stats]` 末行，**五轮逐字节相同**：

```
采集=1781(15.0 fps) 节流丢=0 入环=1781 出环=1781 覆盖丢=0 深=0 高水位=1
拷贝=0.34~0.35ms/帧 等待=89.4~89.7s 发包=195910 发包错=0 丢帧(发送)=0
```

| 组 | N | 板端 覆盖丢/深/高水位 | PC 侧 |
| --- | --- | --- | --- |
| 纯接收 | 4 | 0 / 0 / 1 | 完整 1780、残缺 1、跳号 0、片丢 0.02% |
| 纯接收 | 6 | 0 / 0 / 1 | 完整 1778、残缺 3、跳号 0、片丢 0.03% |
| 纯接收 | 8 | 0 / 0 / 1 | 完整 1781、残缺 0、跳号 0、片丢 0.00% |
| tracker 负载 | 4 | 0 / 0 / 1 | 完整 1781、残缺 0；闭环 下发 0 次 |
| tracker 负载 | 8 | 0 / 0 / 1 | 完整 1780、残缺 1；闭环 下发 0 次 |

**结论（按 §8 的"不结论也算结论"）**：三档 `覆盖丢` 全 0、`深` 恒 0、`高水位` 恒 1，
换真实检测负载后一模一样 —— **瓶颈不在 Ring Buffer，N 不敏感**。**不选"最优 N"**。

根因很清楚：`等待 ≈ 89.6 s / 118.7 s ≈ 75%` 的时长里消费端在**等下一帧**
（`ringbuf.c` 里消费者在锁外等新帧的累计），也就是摄像头 15 fps 才是限制环节 ——
生产者每 67 ms 才放一帧，缓冲区里**从来没有超过 1 帧**，深度给多少都用不上。
PC 侧那 0~3 个残缺帧与 0.00~0.03% 片丢在两组间随机出现，是网络噪声，与 N 无关。

证据：`logs/phase6/m6_ab_20261007_211933/`（纯接收）、`m6_ab_20261007_213108/`（tracker 负载）。

### 4.5 M7：daemon 看门狗触发次数"随时长线性增长"的归属（2026-10-07）—— **已定位**

工具与设计见 §9，**板端一行未改**。三组各跑 180 s，同一会话（uptime 7087→7593 s 全程未变），
板端 uptime 没变即没有 reboot：

| 组 | 控制源 | `securityd` | 会话样本 | **窗口内 trips** | 窗口内超 500 ms | 到达空档 | `seq` 断点 | PC 侧负载 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A | `phase6_m7_drive.py` 20 Hz HOLD | 无 | 3589 | **+0** | 0 | 0 | 0 | — |
| B | 同上 | **有**（`video_link` 常开，~19 Mbit/s） | 3584 | **+0** | 0 | 0 | 3 | — |
| C | 真 `main.py` 检测 | 无 | 3611 | **+0** | 0 | 0 | 0 | 检测 2701 帧 @15 fps |

窗口内 `ms_since_ctrl` 的分布：A p50 12 / p99 15 / max 464 ms，B p50 8 / p99 16 / max 460 ms，
C p50 23 / p99 28 / max 461 ms —— **三组都离 500 ms 的窗口很远**。

**根因陈述**

1. **看门狗只在"连续 >500 ms 没有控制包被接受"时才触发**，这是它唯一的行为。
   只要控制源按 20 Hz 恒发，它一次都不触发：三组共 **10784 个会话内样本，0 次触发**。
2. **板端调度与 `securityd` 争用被排除**。A 组（无 `securityd`）与 B 组（有 `securityd` + 常开
   `video_link`）**完全一致**：会话内各 0 次触发、0 处到达空档、`seq` 连续。单核 A7 上
   "20 Hz 心跳对 500 ms 看门狗只有 10× 余量"这个担心，实测余量约 **35×**（p99 ≈ 15 ms）。
   事实 9 的单核争用**不是**看门狗触发的原因。
3. **真 tracker 也守规矩**：C 组把控制源换成 `main.py` 的真实检测循环（2701 帧 / 15 fps），
   心跳线程照旧把 `ms_since_ctrl` 压在 p99 28 ms。
4. **每一组的那一次 +1，都是"会话结束"那一次**：三组都精确地在驱动器停发后的
   t≈183.4 s 处 `daemon_wd_trips` 加一，同一刻 `daemon_state` 由 `5`(`HOLD`) 转 `4`(`LOST`)。
   驱动器停发 → 0.5 s 内没人续命 → 看门狗按设计触发并锁存。**这不是故障，是它在干活。**

**结论**：Phase 3 记的"daemon 看门狗触发 9 次、随时长线性增长"**不是板端缺陷**。
看门狗计数器实际上在数**控制源的停发次数** —— 每一次 tracker 会话的开始/结束、每一段
PC 侧停发（`--roi` 择优、换轴、退出时的 `stop(send_stop=True)`）都恰好贡献一次。
按第 1 条，只要有一段控制流真的断了 >500 ms，就必然 +1。把 `daemon_wd_trips` 的
线性增长读成"板子不稳定"，是 Phase 3 的误读。**逐项优化按计划推后，本轮不修。**

**残留空洞（如实记）**：三组里 tracker 的 `下发` 都是 **0 次**（画面里没有真目标，
大死区又兜住了动作），所以"云台真的在动、`Auto.update` 全速下发"的那一种负载没测到。
PC 侧重活是检测（已全量跑过），下发只多一次 UDP 发送与几步算术，判断影响可忽略 ——
但这是**判断**不是本轮的实测，M5 有人操作时可顺手复核。

**本轮先踩后修的坑（脚本自己的）**：`phase6_telem_capture.py` 第一版把抓取窗口**两头**
的样本也算进"心跳超窗" —— 头部是上一次会话残留的 `last_accept_ms`（daemon 不重启
`have_last_cmd` 就不清零），尾部是停发后的尾巴。于是 A 组明明会话内干干净净，
却被 61806 ms 那条历史残留判成"PC 侧心跳断了"。**两组都被误判成"PC 侧"**，
而两组实测都是"无异常"。修法是只在"控制会话窗口"内判定（见 §9.5）。

证据：`logs/phase6/m7_20261007_213622/`（A、B）、`logs/phase6/m7c_tracker_20261007_214725/`（C），
修正后的判读另存 `m7_20261007_213622/m7_verdict_corrected.txt`；第一版的原始输出
`*_capture.log` 未删，两版并存便于核对。

### 4.6 M5：端到端业务链（开发机 + 板端，2026-10-07）—— **PASS**

`bash scripts/phase6_e2e_run.sh`，操作者在场，`DURATION=120`。两段独立断言同时成立：

| 段 | 实测 | 判定 |
| --- | --- | --- |
| 板端 A | `EVT ts=16224 from=GUARD to=ALERT reason=prox_near ps=80` → `to=TRACK_READY` → 告警续期 13 次 → `to=STOPPING` → `to=GUARD reason=stopped`；WAV 15876140 B、`riff == size-8`、`audio_play -F` 回放 2 s 成功；`deg=none`、无孤儿、`uptime` 未变 | **PASS** |
| PC 端 B | 进环 1736/1739 帧（99.8%）、下发 90 次、撞限位 0、锁定 (398,232)、片丢 0.08% | **PASS** |

#### M5 抓出的真缺陷：子进程收不到 SIGTERM（已修）

`securityd` 用 `signalfd`，必须先 `SIG_BLOCK` 掉 SIGTERM 等；而 `fork()` 继承信号掩码、
`execv()` 也不清除 —— 于是它起的**每个子进程**起来时 SIGTERM 都是被屏蔽的。
`audio_capture` 装了 handler 也永远不跑，4 s 后被兜底 SIGKILL，**WAV 头没回填**：
两条录音分别是 5115948 / 11145260 字节的真 PCM，头上却是 `RIFF 00000000` / `data 00000000`，
`audio_play` 播不了（这正是 `securityd.c` 里那句 `SIGKILL（WAV 头可能未回填）` 警告的分支）。

修法见 §3.3。重编重部后实测：子进程 `/proc/*/status` 的 `SigBlk` 由继承的非零变成
`0000000000000000`，SIGTERM 后 241 ms 干净退出、退出码 0。**这条路径在 M5 之前从未被跑到过**
—— M0 与 M1–M4 那两轮都没有真实告警，[6] 有序退出用的是 `video_link` 自己报的退出码 3。

#### PC 端 B 的判据当场返工（与操作者协商后改）

原判据是「CSV 的 `state` 列出现 `TRACK` 且连续 ≥30 帧」。但 `state` 列表达的是
"这条轴处在控制策略的哪个阶段"：`TRACK` = 连续闭环下发中，`HOLD_PROBE` = 择优搜索最佳位姿中，
**`HOLD` = 择优完成后锁定位姿的保持态**（`main.py` 的 `_probe_finish()` 里置的）。
所以"锁定成功"那一列是 `HOLD` 不是 `TRACK` —— **锁得越稳，`TRACK` 越短**。两轮实测坐实：

| 跑次 | 进环 | 下发 | 撞限位 | 最长连续 `TRACK` | 原判据 |
| --- | --- | --- | --- | --- | --- |
| `e2e_20261007_220545` | 99.8% | 603 次 | **86 次** | 104 帧 | PASS |
| `e2e_20261007_221330` | 99.8% | 90 次 | **0 次** | 23 帧 | FAIL |

跟踪更差的那轮（撞限位 86 次、反复追着发指令）反而通过，判据在奖励跟踪差的跑次。
改成量计划真正要证的四件事，全部取自跟踪器自己打的汇总行与 CSV：**进环占比 ≥90%**
（闭环真合上了）、**下发 >0**（PC 真的驱动了云台）、**片丢 ≤0.3%**（与 Phase 3 的
0.02~0.07% 同量级）、**出现过锁定**（日志有 `[roi] 择优完成` 且 CSV 有 `HOLD`）。
把脚本里**真实的** [4/5] 代码抽出来回判两轮，**两轮都通过** —— 新判据不是"只让失败的那轮过"。

#### 收证脚本自身的坑（已修）

[3/5] 原来写的是 `tail -60 stdout.log`，而 STATS 是 1 Hz —— 120 s 的跑里 stdout 有 160+ 行，
`to=ALERT` 那条 EVT 早在 60 行之外，于是 `grep` 判"本轮没有 ALERT"，把板端 A 判成假 FAIL。
改成"按关键词全量 grep"，行数与跑多久无关；同时补上 WAV 头回填校验与 `audio_play -F` 实播 2 s
—— 否则上面那个 WAV 缺陷会被静默放过。两条坏头录音留在板端 `records/` 里改名 `*_BADHDR_wav.txt` 存证。

**§4.5 的"残留空洞"被本轮补上了（2026-10-07 复核后改写）**：原先记的是"本轮没抓遥测，
所以带下发的遥测对照仍算未补"。复核证据后这句**不成立**——本轮遥测是在抓的：
`03_track.log` 的 5 条显示行**全部**标 `TELEM-OK`（无一行丢遥测），9101 并没有被"独占",
跟踪器自己就是遥测接收方，`dwd` 正是从遥测里取出来写进 CSV 的。

那么 §4.5 缺的那个条件——"控制源在真下发时的遥测对照"——本轮实测为：

| 量 | 值 |
| --- | --- |
| 窗口 | 2.891 → 119.955 s（**117.06 s**） |
| 遥测派生样本 | CSV **1739 行**，逐帧（≈14.5 Hz） |
| tracker 真下发 | `moves` 峰值 **90**（`main.py:346` "真正下发过目标的次数"，显示行的 `下发` 是同一个计数器） |
| 云台真的动了 | 显示行观测到目标位变化，如 `418→426 (+8 tick)`、`239→235 (-4 tick)` |
| **`daemon_wd_trips`** | **全程 = 7，1739 个样本一个不多** |
| 驱动 `watchdog_trips` | **全程 = 0** |
| 闭环帧 | 进环 97→337 帧递增；`TRACK`×67 / `HOLD`×1428 / `HOLD_PROBE`×241 / `ACQUIRE`×3 |

即"跟踪器全速下发 + 云台真实位移"这段负载下，daemon 看门狗**一次都没触发**。
§4.5 的结论（看门狗只在控制流真断 >500 ms 时 +1，线性增长是在数会话边界）**站住了**。

**仍然没有的，只有一样**：CSV 是帧率采样（≈14.5 Hz），不带 `ms_since_ctrl`，
所以 §4.5 那张"余量 ≈35×（p99 ≈15 ms）"的表仍然只来自 A/B/C 三组。
不过那条余量论证本来就不含下发负载，而下发只多一次 UDP 发送与几步算术 ——
**这一点仍是判断，不是实测**，照旧明写。

证据：`logs/phase6/e2e_20261007_221330/`（`00_preflight.log`、`01_board_start.log`、
`03_track.csv`、`03_track.log`、`04_board_collect.log`）；判据返工的反例证据
`logs/phase6/e2e_20261007_220545/` 一并保留。

## 5. 与总计划的冲突（明写，不静默处理）

`rules/README.md` 要求"若规则与项目计划冲突，先说明冲突，不得静默选择"。以下两处**回写总计划**，
避免下轮有人照着 §6.1/§6.3 的原话把不该接的东西接进来。

### 5.1 §6.1 ALERT 触发源

| 计划原文 | 冲突 | 处置 |
| --- | --- | --- |
| §6.1 ALERT 由"AP3216C / ICM20608 event"触发，典型事件含"光照变化/振动" | 与协商 #2 冲突。ALS 无事件出口；ICM20608 的 INT 脚**未接进设备树**（不是没接硬件，见 §1 事实 2 更正） | ALERT **只由 AP3216C 接近事件**触发。ALS 与振动降级为定时轮询采样，只进日志与统计。**ICM20608 中断接入不在本轮范围，交 Phase 7**（Phase 7 M1 实测该脚零边沿、中断作废，振动最终走软件阈值） |

### 5.2 §6.3 可观测性来源

| 计划原文 | 冲突 | 处置 |
| --- | --- | --- |
| §6.3 把 `mx6hub: fifo/irq/drop/latency` 当可观测性来源 | mx6hub 与 Phase 4 两个新驱动争硬件（AP3216C 的 `0-001e`、ICM20608 的 SPI），`board_verify_sensors.sh` 每段都先 `rmmod mx6hub`。本链不加载 mx6hub，**够不着它的 debugfs** | 可观测性改由 `ap3216c_input` 的 sysfs 只读 + `securityd` 自己的 STATS/status 文件承担；真实的 pantilt debugfs 仍是 `/sys/kernel/debug/pantilt/{state,limit,stats}`。mx6hub 依 `rules` §5 原样保留，只是不加载 |

### 5.3 §6.1 里"用 `cmd_count` 增长率当对端正在驱动的旁证"—— 做不到

计划设想用 `/dev/pantilt0` 的 `PANTILT_IOC_GET_STATE` 读 `cmd_count` 增长率，作为
"PC 正在下发跟踪命令"的旁证。**这条路被 EBUSY 堵死**（事实 10 / §4.2）：

- 驱动 `pt_open()` 只允许一个打开者，而这条链上 `pantilt_daemon` 一直占着它；
- 连**只读**也做不到——open 就 EBUSY，没有"只读模式"这种区分。

2026-10-07 与用户协商后**明确放弃该旁证**，并把 `securityd` 与云台的耦合整个删掉：
不 include `pantilt_uapi.h`、不开 `/dev/pantilt0`、不发任何控制包。让"不碰云台"成为**结构性事实**
而不是一句承诺。

**代价必须记下来**：板端少一条独立证据，M5 的板端断言因此更弱。计划本来就写明
"锁定与否只有 PC 能证"，这个空洞如实记在 README 与验收文档里，不包装成"识别成功"。

> **别把 §6.1（M4）的 debugfs 对账和这条混为一谈。** 放弃的是"`securityd` 在**运行时**
> 读 `cmd_count` 当旁证" —— 那会要求它一直占着 `/dev/pantilt0`，和 daemon 抢。M4 要的
> 是**验收脚本**在某一刻把两路读数对一次，属于一次性动作，不是常驻耦合：`board_verify_phase6.sh`
> 的 [10] 段默认**不碰 daemon**（debugfs 本就不受单打开者限制，照读），只有显式给
> `M4_STOP_DAEMON=1` 才临时停 daemon 去取一次 `GET_STATE`，取完照原命令行起回来。

## 6. 验证方式

### 6.1 板端 A

```bash
sh scripts/board_verify_phase6.sh          # 一次跑完 11 段，给 PASS/FAIL
#   WITH_HAND=0      跳过人工靠近段（现场没人时），[4] 记 NOT RUN，总判定仍可 PASS
#   WITH_EXHAUST=0   跳过 [11] 重载耗尽段（同样要操作者），记 NOT RUN
#   M4_STOP_DAEMON=1 [10] 允许临时停 daemon 取 GET_STATE（默认 0：平时不碰 daemon）
```

11 段：环境与设备准备 / 启动自检 / 常驻与可观测性 / 人工靠近 / 结果核对 / 有序退出 /
摄像头重载自愈 / SIGUSR1 即时 dump / 传感器缺失自检 / 可观测性对账 / 重载耗尽路径。
后四段补的是 §7 里明写的三处空洞，来历见 §7 表格下的"补段"说明。

判据尽量取**独立于被测程序的第三者**：

- WAV 合法性用板端 python3 的 `wave` 模块判；**`nframes=0` 直接判失败**——44 字节的空头在
  size/rate/ch 上是"自洽"的，只查头会漏判（Phase 5 §4.1 踩过）；
- 无孤儿用 `pidof` 判，没重启用 `/proc/uptime` 判，不靠 `securityd` 自述；
- "视频常开生效"的判据是**还没告警时 `video=up`** —— 这是常开决策的直接后果，不是间接指标。

### 6.2 端到端

```bash
bash scripts/phase6_e2e_run.sh             # 开发机侧编排 + 两边证据收集
#   DURATION=120   默认 120 s（Phase 3 定的规矩：只看 2 分钟窗口）
```

**两段独立断言**，因为它们各自只能在各自那一侧看到全部证据：

| 段 | 判据 | 在哪判 |
| --- | --- | --- |
| 板端 A | ALERT 触发 → 视频发包 → WAV 落盘且 `audio_play -F` 能播 → TRACK_READY → 回 GUARD，无孤儿无重启 | `phase6_e2e_run.sh` 的 [3/5] 取证 + [4/5] 判 |
| PC 端 B | 进环占比 ≥90% 且 下发 >0 且 片丢 ≤0.3% 且 出现过锁定（日志有 `[roi] 择优完成` 且 CSV 有 `HOLD`） | `phase6_e2e_run.sh` 的 [4/5] |

**PC 端 B 这四条的来历（2026-10-07 修订，不是初版）**：初版写的是"CSV 的 `state` 列出现
`TRACK` 且连续 ≥30 帧"，实测发现**判据是反的** —— `state` 列表达的是轴所处的控制阶段，
`HOLD` 才是"择优完成后锁定了"的保持态，所以锁得越稳 `TRACK` 越短；撞限位 86 次的那轮
反而通过。改用上面四条后，两轮的原始数据都通过。完整对照与理由见 §4.6。

PC 侧命令行沿用 Phase 3 已实机验收过的那条（`logs/phase3/20261007_lash_roi_acceptance.md` §8.1）：

```bash
DISPLAY=:0 XAUTHORITY=/run/user/1003/.mutter-Xwaylandauth.* \
python3 imx6ull_project_src/pc/tracker/main.py --host 192.168.137.50 --auto --video \
  --axis both --kp +0.50 --slope -1.83 --kp-tilt -0.50 --slope-tilt +1.12 \
  --deadband 2.0 --max-step 8 --roi 8 --hold-window 3.0 --duration 120 --csv both.csv
```

`--slope` 必须**逐轴用 `--calib-sign` 单独量**（`--calib-sign` 与 `--axis both` 不能同用）。
开发机是 Wayland，开窗需显式给 `DISPLAY=:0` + `XAUTHORITY`。

### 6.3 人工步骤

**一次 ALERT 需要两个动作同时成立**：手掌在 AP3216C 传感器上方掠过（贴近 → 移开），
**同时**红色目标在画面里。脚本会把这段提示打在跟踪器启动之前。

## 7. 里程碑与通过判据

| # | 目标 | 通过判据 | 状态 |
| --- | --- | --- | --- |
| M0 | 全链路共存 smoke | 见 §4.1 | **PASS** |
| M1 | 骨架（`--dry-run`） | EVT 行走完 GUARD→ALERT→TRACK_READY 但无子进程；`alert_hold` 到点回 GUARD；`SIGUSR1` dump；传感器缺失时**立即报** `DEGRADED(no_sensor)` | **部分**：`SIGUSR1` dump 由 [8] PASS、`DEGRADED(no_sensor)` 由 [9] PASS、EVT 全流程由 M5 补上（`GUARD→ALERT→TRACK_READY→STOPPING→GUARD` 五段全出现）。**仅剩一条未单独验**：纯 `--dry-run`（无子进程）下走完 EVT 全流程——现有 EVT 证据都出自带子进程的真实跑 |
| M2 | 子进程监督 | `records/` 出现 `*_alert.wav` 且 `wave` 判头与长度自洽；`[stats]` 且发包>0；退出后 `pidof video_link audio_capture` 为空 | **PASS**：无孤儿与未重启出 [6]；真实告警的 WAV 落盘、`riff == size-8`、`audio_play -F` 实播成功，以及 `[stats]` 发包数持续增长，由 M5 补齐（见 §4.6） |
| M3 | 摄像头故障与重载 | 重载次数 ≤ 上限、成功后 video 恢复 `up`；耗尽时 `DEGRADED(camera)` 且**音频仍在录、pantilt 未被动过、板子未重启** | **PASS**：成功路径出 [7]（故障→重载 1 次→`video=up vin=15 vout=15 vovr=0 verr=0`）；**耗尽路径出 [11]**（`--camera-mod-dir` 指向空目录 → 3 次后 `DEGRADED(camera)`，音频仍在录、板子未重启，收尾把模块装回） |
| M4 | 可观测性补齐与对账 | `status` 文件字段可被脚本解析；`board_verify_phase6.sh` 一次跑完给出总判定 | **PASS**：[3] status 文件可解析、脚本一次跑完给总判定；**debugfs 三项对账出 [10]**（`state`/`limit`/`stats` 三节点可读，`limit` 与标定文件逐轴对） |
| M5 | **端到端业务链**（PASS 核心） | 见 §6.2 的两段独立断言 | **PASS**（2026-10-07，操作者在场）：板端 A 与 PC 端 B 同时成立。本轮抓出并修掉 `securityd` 的**信号掩码缺陷**（`fork` 继承 `SIG_BLOCK` → 子进程收不到 SIGTERM → WAV 头不回填），PC 端判据当场返工（原判据在奖励跟踪差的跑次）。见 §4.6 |
| M6 | A/B：Ring Buffer slots | 见 §8 | **PASS**：N=4/6/8 三档板端计数逐字节相同（`覆盖丢=0 深=0 高水位=1`），换成 tracker 真实检测负载后仍是 0 → 结论"N 不敏感，瓶颈不在 Ring Buffer"。见 §4.4 |
| M7 | 用调试工具定位 daemon 看门狗增长 | 交付**根因陈述 + 证据**，不要求修复 | **已定位**（不要求修复，本轮不修）：三组 180 s 共 10784 个会话内样本 **0 次触发**；有无 `securityd` 完全一致 → 不是板端调度问题。看门狗只在控制流真断 >500 ms 时触发，每段会话结束恰好 +1，Phase 3 的"线性增长"是在数会话边界。见 §4.5 |

2026-10-08 板端终验收：**11 段全 PASS**（操作者在场，完成 [4] 与 [11] 两段）。
原始输出与各子进程日志见 [logs/phase6/](../logs/phase6/README.md) 下的 `board_verify_20261008/`
—— `p6_verify.out` 是整轮输出，其下是 `securityd` 自己的 stdout、status 文件与各段子进程日志。

2026-10-07 补段，把当时上表里三处**明写的空洞**各自配上一条自动判据，脚本从 7 段扩到 11 段：

| 新段 | 补的是哪处空洞 | 判据 |
| --- | --- | --- |
| [8] | M1 的 `SIGUSR1` dump | `--stats-ms 8000` 下发 `SIGUSR1`，新 STATS 行的 `ts` 差值 ≤ 3000 ms —— 用程序自己的单调时钟，不碰墙钟，"立即"与"碰巧赶上周期行"才分得开 |
| [9] | M1 的 `DEGRADED(no_sensor)` | 卸掉 `ap3216c_input` 后启动，必须同时出现命令行 `DEGRADED no_sensor` 与 STATS 的 `deg=no_sensor`；跑完把传感器装回去（[11] 还要用它触发真实告警） |
| [10] | M4 的 debugfs 对账 | 三节点可读；`limit` 与标定文件逐轴对；**再**与 `GET_STATE` 逐字段对（状态字、芯片态、两轴 cur/tgt/home、safe、slew/看门狗）。daemon 占着 `/dev/pantilt0` 时 GET_STATE 拿不到 —— 默认不碰 daemon，用 `M4_STOP_DAEMON=1` 显式开 |
| [11] | M3 的耗尽路径 | `--camera-mod-dir` 指向空目录 → `rmmod` 成功而 `insmod` 必失败 → 正好 3 次后 `DEGRADED(camera)`，且音频仍在录、板子未重启；收尾显式把摄像头模块装回去 |

**状态：已实现、已部署、板端实跑过。**
[8] 与 [10] 的解析器用真实格式的样本数据离线单测过（含 `%4u` 右对齐的 0 与 4 位数边界）。
需要操作者的两段：[4]（人工靠近）与 [11]（耗尽路径要先有一次真实 ALERT，否则"音频仍在录"无从判起）
—— 后者可用 `WITH_EXHAUST=0` 跳过，跳过时记 NOT RUN，不拉低总判定。

**两段人工的动作要求相反**，现场别搞混：[4] 要 `GUARD→ALERT→TRACK_READY→GUARD` 走完，
所以贴住 2~3 秒后移开就**别再靠近**（每次新 near 都会把 8 s 的 `alert_hold` 重新计时）；
[11] 恰恰要告警一直续着（重载 3 次要 5~10 s），触发后**要反复再靠近几次**，
否则录音会先于 `DEGRADED(camera)` 停下，「音频仍在录」就成了假 FAIL。2026-10-07 前两轮分别栽在这两点上。

实跑结果：① 2026-10-07（脚本 `de406af7…`）全 11 段 PASS；
② 2026-10-08（脚本 `9719eea7…`，与 ① 的差别只有 [3] 的 status 文件判定，[4]~[11] 未动）
先以 `WITH_HAND=0 WITH_EXHAUST=0` 跑免操作者的 9 段 PASS，同日有操作者时再全 11 段 PASS
—— **终验收用的是 ② 的 11 段那一次**，证据在 `board_verify_20261008/`。

**首轮实测暴露的两处 `securityd` 缺陷（已修，记在这里免得下次重犯）：**

| 位置 | 症状 | 根因 |
| --- | --- | --- |
| `audio_on_result()` | `recorded` **恒为 0**；且每次告警正常收尾都打一条假警报 `DEGRADED audio —— 录音 frames=0，文件只有 WAV 头` | 传的 label 是 `"frames="`，而 `lbl_u64()` 自己会再吃掉一个 `=`（等于在找 `frames==`），永远匹配不上。视频侧六个调用点都不带 `=`，是这两处写歪 |
| `handle_sigchld()` | 收尾时子进程最后几行被静默丢弃 —— `audio: RESULT`（录音是否落盘的唯一在带内证据）、`video_link` 的 `[INFO] 退出码` | 只 `close()` 不抽干管道。`final_stop()` 走 `reap_until()` 直接调它，抢在主循环读管道之前关掉 fd，该路径必中 |

两处都不改语义、只改可观测性，但都属"把没发生的事报成发生了/把发生的事丢掉"，正是本阶段要治的那类问题。
另：脚本 [3] 在 run6 出现一次"首条 STATS 为空 + status 文件仍是上一轮终态"，**成因未能复现**；
不改解释，只堵洞 —— 进 [3] 前先 `rm -f "$STATUS_FILE"`，并把脚本头早就承诺的"status 文件可解析"补成真判据。

## 8. M6 A/B 设计

- **受控变量**：`--video-slots N`，N ∈ {4, 6, 8}。
- **锁死的不变量**：同一块板、同一会话（期间不 reboot）、同一份摄像头模块与 DTB、`320x240`、
  `--fps 15`、`--stats-ms 1000`、同一 `--video-dst`，PC 侧固定 `video_rx.py` **无窗口**。
  云台保持 HOLD，静止画面，每轮 **120 s**。
- **测量口径**：`覆盖丢` / `高水位` / `深` / `出环 fps` / `丢帧(发送)` / `发包错`；
  PC 侧收帧数与 `frame_id` 跳号数。
- **可比性**：每轮必须是一条"干净轮"（无摄像头重载、无 PC 断链）才计入；不干净就作废重跑。
  每 N 先跑 1 轮；三档差异若落在噪声内再补 1 轮，最多 2 轮/N。
- **"不结论也算结论"**：受控条件下若三档 `覆盖丢` 全为 0，说明**瓶颈不在 Ring Buffer** ——
  此时正确的结论是"N 不敏感，瓶颈在别处"，**不要为了有结论硬选一个 N**。
  这种情况下追加真实负载对照：PC 跑 tracker 检测，只测 N=4 与 N=8 两档（看趋势即可）。

## 9. M7 诊断方法（板端零改动）

1. 开发机跑 `scripts/phase6_telem_capture.py`，抓 daemon 20 Hz 遥测并**打到达时间戳**，
   记录 `daemon_wd_trips`、`ms_since_ctrl`、`cmd_count`、`daemon_state` 的时间序列。
2. 判别逻辑（**这是本方法的支点**）：

   | 观测 | 结论 |
   | --- | --- |
   | 遥测**到达本身出现空档**（≥250 ms），且该窗口内 `daemon_wd_trips` 增长 | **板端**：daemon 主循环被抢占（单核争用，事实 9 是首要嫌疑）。下一步用 ftrace 的 `sched_switch`/`sched_wakeup` 定位抢占者 |
   | 遥测照常按 20 Hz 到达（无空档），而 `ms_since_ctrl` 冲过 `daemon_wd_ms` | **PC 侧**：板端主循环是活的，是 PC 的心跳真的断了 |

   这个区分靠的是"板端每 50 ms 发一包"本身的规律性：主循环被抢占时发包也会一起停 ——
   遥测的空档就是主循环停摆的直接证据，**不需要动板端加埋点**。
3. 必须控住混杂变量：至少给出**"无 `securityd`"与"有 `securityd`"两组**的 trip 增长率对比。
   （实测见 §4.5：这两组**完全一致**，`securityd` 的争用不是原因。）
4. 判不了就直说判不了：`trips` 增长但既无到达空档、也无 `ms_since_ctrl` 超窗时，
   工具返回"判不了，需要 ftrace 级别证据"，**不硬归因**。

### 9.5 上面那张表**必须先划出"控制会话窗口"再套用**（第一版栽在这）

第 2 步的表不写窗口就会**误判**，实测两组都被误判成"PC 侧"（§4.5 末尾）。原因：
daemon 的 `have_last_cmd`/`last_accept_ms` **跨会话留存**，抓取窗口两头天然带着两段
与本轮控制无关的样本 ——

- **头部**：上一次会话留下的 `last_accept_ms` 早已陈旧，`ms_since_ctrl` 一上来就是几万毫秒。
  这是历史残留，不是这一轮 PC 心跳断了。实测 A 组因此拿到 `最长 61806 ms`。
- **尾部**：驱动器按 `--duration` 停发之后，`ms_since_ctrl` 自然涨过 500 ms 并触发一次 trip。
  那是**看门狗在按设计工作**，是第 4 条里"每一组都恰好 +1"的那一次。

处置已落在工具里：先从数据推出会话窗口（`ms_since_ctrl` 的**首个回落点**是会话开始、
**末次未超窗点**是会话结束），只在窗口内判"超窗"与"trip 增长"；窗口外的样本照旧报出来
（标注为历史残留/停发尾巴），**不藏**。也支持 `--ctrl-from`/`--ctrl-to` 显式指定。
另加一条防倒挂：**一个会话都没认出来时直接返回"无法判定"**，
绝不能在空集上报"无异常"——那会把一轮彻底失败的抓取读成好消息。

## 10. 回退

全是新增：板端删掉 `securityd` 即回到 Phase 5 状态。内核模块、DTS、mixer、boot 分区
**全程未动**；`pantilt_daemon.c` 与内核驱动一行未改。验收脚本不删任何录音文件。

## 11. 明确不做（本轮 Non-goals）

> **2026-10-08 交接说明**：下列三条已被用户新提的 Phase 7 需求接管，**本文不再自行推进**——
> 「传感器 IRQ 频率」，本节的「自动搜索目标」，本节的「ftrace/kprobes/gdb 的交付物化」。
> 一并被推翻的还有本文件 §1 事实 2 的「INT 未接 CPU」（事实更正，非范围变更）。
> Phase 6 其余 Non-goals 继续有效。

- ftrace/kprobes/gdb 的**交付物化**（只在 M7 按需使用）
- §6.6 的逐项性能优化（传感器 IRQ 频率、编码、ALSA period/buffer 调参等）
- IIO buffer / trigger / kfifo（Phase 4 已明确推后）
- LCD 状态显示、音频降噪、自动搜索目标、PID/YOLO
- 修改已冻结的 3B 控制协议；修改 `pantilt_daemon.c` 与任何内核驱动
- 给 mx6hub 补设备树节点
- 任何"注入 ALERT"的入口（socket / 信号 / 文件）—— 留个后门等于留一个可以伪造验收的口子
- Phase 3 其余遗留（两轴交叉耦合定量测量、饱和出画与真丢目标的状态区分、30 fps 双轴、小时级长跑）
