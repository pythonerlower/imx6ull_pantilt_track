# 工程清单（2026-10-08）

计划入口：[`plan/plan_imx6ull_pantilit_final.md`](../plan/plan_imx6ull_pantilit_final.md)。
本表回答一个问题：**仓库里每个目录是干什么的、还算不算数**。

## 当前归类

| 路径 | 作用 | 构建/部署 | 归类 |
| --- | --- | --- | --- |
| `imx6ull_project_src/kernel/baseline/` | 旧项目基线模块（`ap3216c_myself`、`icm20608`、`led`、`beep`） | 构建；离线部署 | active |
| `imx6ull_project_src/kernel/baseline/ap3216c_ref/` | 原厂/参考实现 | 构建；非默认部署 | reference，留着做 A/B 对照 |
| `imx6ull_project_src/kernel/mx6hub/` | 传感器汇聚驱动（`/dev/mx6hub0` + debugfs 统计） | 构建；单独验证 | active |
| `imx6ull_project_src/kernel/iio/icm20608_iio/` | Phase 4：ICM20608 → IIO（仅 sysfs raw，无 buffer/trigger） | 构建；离线部署 | active；与旧 `baseline/icm20608/` 并存做回退 |
| `imx6ull_project_src/kernel/input/ap3216c_input/` | Phase 4：AP3216C → Input（`EV_SW`/`SW_FRONT_PROXIMITY`）+ sysfs raw | 构建；离线部署 | active；以 `baseline/ap3216c_myself/` 为底本 fork |
| `imx6ull_project_src/kernel/pantilt/` | 正式云台驱动（`/dev/pantilt0`，限位/钳位/护栏在驱动里） | 构建；离线部署 | active |
| `imx6ull_project_src/kernel/dts_patch/` | DTS 补丁示例 | 不参与当前构建 | reference |
| `imx6ull_project_src/common/proto/` | 线协议定义：`pantilt_proto.h`（遥测/控制/视频）、`security_event.h`（Phase 7 事件本地 ABI） | 头文件，被两侧 include | active |
| `imx6ull_project_src/user/securityd/` | Phase 6 + 7：板端安防状态机（GUARD→ALERT→TRACK_READY→STOPPING）+ 振动事件。**不碰云台** | `make securityd`；离线部署 | active；验收 `scripts/board_verify_phase6.sh`、回归 `scripts/verify_phase7_nr.sh` |
| `imx6ull_project_src/user/pantilt_daemon/` | 3B 控制面：冻结协议 + 两级看门狗 + 遥测 | `make pantilt-daemon`；离线部署 | active |
| `imx6ull_project_src/user/video_link/` | 3C 视频链路：V4L2 采集 + 环形缓冲 + UDP | `make video-link`；离线部署 | active |
| `imx6ull_project_src/user/audio/` | Phase 5：WM8960 用户态 ALSA 录音/播放（无降噪、不碰 mixer） | `make audio`；离线部署 | active；验收 `scripts/board_verify_audio.sh` |
| `imx6ull_project_src/pc/tracker/` | PC 侧 3D 跟踪器 + Phase 7 巡检：显示、协议、单测（检测算法在这边） | `make -C … test-unit`（只跑开发机） | active |
| `imx6ull_project_src/user/pantilt_test/` | PCA9685 探测、试动与交互式 `calib` 标定会话 | `make pantilt-test`，手动部署 | active；限位结果在 `config/pantilt_calibration.conf` |
| `imx6ull_project_src/user/baseline/`、`hub_monitor/`、`v4l2_fb_preview/` | 板端基线应用与监控 | 构建；离线部署 | active |
| `imx6ull_project_src/user/sensor_alert_app/`、`ioctl_test_app/`、`experiments/` | 旧系统实验应用 | 非统一构建/部署 | experiments，保留 |
| `imx6ull_project_src/out/` | 部署脚本的上传输入（显式清单，不是 `out/*` 全量） | 部署时更新 | 生成物；**禁止当源码** |
| `Makefile`、`scripts/` | 构建、部署、板端检查与远程链路调度 | — | 只做调度，不存源码 |
| `connect/` | 板端连接参数与失联恢复步骤 | — | 文档 |
| `reference/` | 厂商例程与开源方案索引 | — | reference |
| `docs/`、`logs/` | 开发文档；各阶段证据 | — | 见下 |
| `rules/`、`plan/`、`study/` | 规则、执行计划、学习归档 | — | 见下 |

`logs/phaseN/` 是各阶段的**验收证据**，索引在各自的 `README.md`；`docs/` 放仍在用的开发文档
（协议、脚本说明、SSH 流程、本清单）；`study/` 只放开发收尾后的学习归档。

## 设备树（`CURRENT_DTS`）

板端运行的是 boot 分区 `/run/media/mmcblk1p1/imx6ull-14x14-emmc-4.3-480x272-c.dtb`
（md5 `ea5ae0f0…`，39685 B）。它的**源码在内核树内**：`work/imx6ull/src/arch/arm/boot/dts/`
下的 `imx6ull-14x14-emmc-4.3-480x272-c-ap3216c-irq.dts`（工程内副本在
`imx6ull_project_src/kernel/baseline/ap3216c_myself/`），重建后与运行镜像逐节点比对
只差 `ap3216c@1e` 的 `interrupts` 一个属性（运行 `EDGE_BOTH`，源码 `EDGE_FALLING`）。

同名的 `…-c.dts` 是**另一支**（md5 `59dc8e0b…`）：它把 `dht11`/`ds18b20` 置为 `okay`，
与 AP3216C INT 复用的 `GPIO1_IO01` 冲突。`scripts/deploy_offline.sh` 的 `DEPLOY_DTB`
默认值已改为 `…-c-ap3216c-irq.dtb`。**覆盖 boot 分区始终是手动步骤**，脚本不会自己做。
完整谱系比对见 [`logs/phase1_phase2/20261007_dtb_lineage.md`](../logs/phase1_phase2/20261007_dtb_lineage.md)。

## 生成物边界

离线部署原先用 `imx6ull_project_src/out/*` 全量上传，会把历史二进制混进去；已改为**显式产物清单**。
`out/` 里的一切都可重建，不得当作源码引用。旧文档提到的 `scripts/deploy_online.sh`、
`run_bench.sh`、`scripts/remote_link/ssh/run_preview.sh` 都不存在——`run_link.sh` 默认走 UDP 方案，
并拒绝 SSH 视频分支。

## 历次清理（一句话各记一次）

| 时间 | 做了什么 |
| --- | --- |
| 2026-10-07 | 把 `kernel/`、`user/`、`out/`、`temp/` 整体移入 `imx6ull_project_src/`；`Makefile` 与 `scripts/` 留在根目录只做调度 |
| 2026-10-07（Phase 3 收尾） | 删可重建的生成物与被取代的重复轮次；删前 `git init` + `phase3-before_delete` 作为恢复点 |
| 2026-10-07（Phase 5 收尾） | 清仓库外的调试草稿（`/tmp` 探针、失败录音、无效 WAV）；`records/` 里 8 条有效录音全部保留 |
| 2026-10-08（Phase 6 收尾） | 删 `logs/phase6/board_auto_20261007/`（被 `board_verify_20261008/` 全面覆盖）、0 字节废跑 `m7c_tracker_..._214553/`、旧的 `logs/bench_results/` |
| 2026-10-08（Phase 7 收尾） | 删 `study/summary/`（6 月的面试稿，基于大改版前的代码）；删 2 个摄像头长测跑次目录 + `udp_camera_native_20261006/` + 6 个未跟踪的 `video_link` 跑次目录（结论已入 `study.md` 与基线记录）。被文档**引用**的跑次（`3b_e2e_*`、`ov5640_662s_*`、两个 `video_link_20261007_*`）一律保留 |

判据始终是同一条：**只留问题调试、最终配置和关键记录**；被取代的跑次、可重建的生成物、
仓库外的一次性草稿删掉；只要**仍有文档引用**就跑次也留着。
