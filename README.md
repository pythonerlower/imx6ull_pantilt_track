# IMX6ULL 安防云台工程

板端 ATK-IMX6U（单核 Cortex-A7 @792 MHz，内核 4.1.15）+ Ubuntu 开发机。
**板端负责采集、执行、安全；检测算法留在宿主机**——理由见 [study/study.md](study/study.md) §5。

执行计划：[plan/plan_imx6ull_pantilit_final.md](plan/plan_imx6ull_pantilit_final.md)。
工作原则：[rules/imx6ull-pantilt-engineering.md](rules/imx6ull-pantilt-engineering.md)。
硬件限位与红线：[rules/Hardware.md](rules/Hardware.md)。

## 当前状态

| Phase | 状态 | 一句话 | 记录 |
| --- | --- | --- | --- |
| 0 | 基线通过 | 旧系统的传感器、mx6hub、摄像头本地采集逐项验过 | [基线记录](docs/baseline_2026_pantilt.md) |
| 1 | 已核查 | 源码位置与实验/参考边界成立，`CURRENT_DTS` 溯源闭合 | [logs/phase1_phase2/](logs/phase1_phase2/README.md) |
| 2 | 已核查 | PCA9685 @ `/dev/i2c-1` `0x40`；两轴机械标定写入 `config/pantilt_calibration.conf` | 同上 |
| 3 | 已收尾 | 驱动限位/钳位/护栏、冻结的控制面协议、视频链路、双轴视觉闭环 | [phase3 状态](plan/phase3_status_20261007.md) |
| 4 | 已完成 | ICM20608→IIO、AP3216C→Input 两条新链路，合并验收 6 段 | `scripts/board_verify_sensors.sh` |
| 5 | 已完成 | WM8960 用户态 ALSA 录音/播放，板端合并验收 7/7 PASS | [phase5 计划](plan/phase5_audio_alsa.md) |
| 6 | 验收通过 | 板端状态机 `securityd` 把前五个 Phase 拼成一条链，**11 段全 PASS** | [logs/phase6/README.md](logs/phase6/README.md) |
| 7 | 已完工 | 振动事件 → pan 定幅巡检，外加 ftrace 诊断；**走软件阈值，不改 DTS/驱动** | [logs/phase7/](logs/phase7/README.md) |

### Phase 7 要点（2026-10-08）

三条链路，**默认全关，不开任何开关 = Phase 6 行为**：

1. **振动事件**（`securityd`）：M1 门没过 —— ICM20608 的 INT 脚原理图上确实接了
   （`6D_INT` → 核心板 J2-22 → `JTAG_MOD` = GPIO1_IO10，与 ATK 模块座 `GBC_LED` 复用），
   驱动也本来就配了 DATA_RDY 中断，但**这个脚在板上被外部恒定拉低、零边沿**
   （5 万样本 0 次高电平，IRQ 计数 0 个边沿），所以走计划 §2.2 的备选：
   IIO 轮询 + 软件阈值，**没有写任何 DTS / 驱动改动**。
   判据是**逐轴帧间最大变化量**（`--vib-thr-mg`，mg），不是合矢量幅值 ——
   后者静止时恒为 1 g（重力），横向晃只二阶地涨、轴向晃反而跌。常速轮询 100 ms，
   超阈值后进 20 ms 突发窗口并保持 2 s。见 [M1 探测记录](logs/phase7/m1_icm_int_probe_20261008.md)。
2. **事件 → 巡检**：`securityd` 经本地 AF_UNIX 数据报把事件交给 `pantilt_daemon`
   （ABI 单独立在 `common/proto/security_event.h`，**`securityd` 仍然不认识云台 ABI**），
   daemon 锁存 2 s 并编码进遥测 `reserved[1]`；PC 端只看**计数上升沿**，触发定幅 pan 扫描，
   扫到红目标即交给 TRACK。两条底线：扫描永不越出 `safe_min`/`safe_max`，且上界 < 515
   （515 tick = 2.51 ms 超 MG90S 规格）；超时必回退。见 [protocol.md](docs/protocol.md)。
3. **ftrace**：`scripts/debug/phase7_trace.sh`，`irq` / `sched` / `both` 三模式，只读 tracer。

**日常回归**用 [`scripts/verify_phase7_nr.sh`](scripts/verify_phase7_nr.sh)（约 40 秒、不需人在场：
线格式对拍 + PC 单测 + 板端默认参数行为 + 遥测可解 + 收尾干净）；上一阶段的完整验收
（`board_verify_phase6.sh`，11 段、3 分钟、要人在场）留给"真要动那条链"时再跑。

**两处事实更正**（都不改变各自 Phase 的验收结论）：

- Phase 6 文档曾写"ICM20608 无中断引脚 / INT 未接 CPU"，**不成立**：硬件上是接了的。
  Phase 7 的 M1 探测进一步查明"接了但取不到边沿"，见上。
- Phase 3 记的 daemon 看门狗"随时长线性增长"，实测是在数会话边界，**不是板端缺陷**（M7）。

## 设备树注意

板端运行的是 boot 分区 `/run/media/mmcblk1p1/imx6ull-14x14-emmc-4.3-480x272-c.dtb`
（md5 `ea5ae0f0…`），其源码在内核树内。`scripts/deploy_offline.sh` 的 `DEPLOY_DTB` 默认已指向
与它同源的 `…-c-ap3216c-irq.dtb`（禁用与 AP3216C INT 抢 `GPIO1_IO01` 的 `dht11`/`ds18b20`）。
**覆盖 boot 分区始终是手动步骤**，脚本不会自己做。
谱系调查见 [DTB 谱系](logs/phase1_phase2/20261007_dtb_lineage.md)。

## 目录

程序源文件只放在 `imx6ull_project_src/`；`Makefile` 与 `scripts/` 只做构建、部署和验证调度。

| 路径 | 内容 |
| --- | --- |
| `imx6ull_project_src/kernel/` | 内核模块：基线模块、`mx6hub`、`iio/`、`input/`、`pantilt/`、DTS 补丁 |
| `imx6ull_project_src/user/` | 板端用户态：`securityd`、`pantilt_daemon`、`video_link`、`audio`、基线应用等 |
| `imx6ull_project_src/pc/tracker/` | PC 侧跟踪器与巡检（含单测）；检测算法在这边 |
| `imx6ull_project_src/out/` | 部署脚本的上传输入（显式清单）；生成物，禁止当源码 |
| `Makefile`、`scripts/` | 构建、部署、板端检查与远程链路 |
| `connect/` | 板端连接参数与失联恢复步骤 |
| `reference/` | 厂商例程与开源方案索引 |
| `docs/`、`logs/` | 开发文档；各阶段验收证据 |
| `rules/`、`plan/`、`study/` | 规则、执行计划、[学习归档](study/study.md) |

详细归类见 [工程清单](docs/repo_inventory.md)。

## 常用入口

```bash
# 一律在工程根目录执行
cd /home/imx6ull/work/imx6ull/imx6ull_project
make host-check && make build          # 交叉编译环境检查 + 全量构建
bash scripts/deploy_offline.sh         # 离线部署（默认先重新构建）
bash scripts/verify_phase7_nr.sh       # 40 秒日常回归
make -C imx6ull_project_src/pc/tracker test-unit   # PC 侧单测（~15 s，不碰板子）

ssh imx6ull-board 'uname -r; ls -l /dev/i2c-* /dev/mx6hub0'   # 只读检查，细节见 connect/
```

板端状态机 `securityd` **刻意不碰云台**（不 open `/dev/pantilt0`）：
驱动只允许一个打开者，而 `pantilt_daemon` 一直占着它。它的板端验收是
`scripts/board_verify_phase6.sh`（11 段；两段需操作者在场，可用 `WITH_HAND=0` /
`WITH_EXHAUST=0` 跳过，两段的人工动作**正好相反**，现场别搞混，
用法见 [securityd/README.md](imx6ull_project_src/user/securityd/README.md)）。

## 一条红线

凡是硬件相关的结论，都必须在 IMX6ULL + 真实 PCA9685 + 双 MG90S + 实测限位下跑过才可标 `VERIFIED`。
标定用的 `sweep` 护栏只用于验证；正式控制程序必须走 `safe_min`/`safe_max`，
并且注意 **Pan 上端 515 tick（2.51 ms）超出 MG90S 规格上限，不得长时间驻留**。
