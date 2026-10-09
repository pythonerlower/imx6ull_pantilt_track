# 旧项目基线复核（2026-10-06）

板端 `ATK-IMX6U`，内核 `4.1.15 #2 SMP PREEMPT`；开发机经 `imx6ull-board` 连接。板端地址 `192.168.137.50`，开发机 `ens38=192.168.137.100/24`。首次摄像头异常造成 Oops 后，保存日志并重启了一次；未切换 DTB 或清空 `dmesg`。

这一步的目的是确认**旧系统的底子能不能用**——不能用的部分后面才值得重写。逐项结果：

| 项目 | 结果 | 依据 |
| --- | --- | --- |
| 主机编译 | VERIFIED | 修复失效路径后 `make host-check build` 成功；`mx6hub` 错误路径一处 `dev_t *` 编译警告，已修复 |
| AP3216C | VERIFIED（经 mx6hub） | 3 秒 `hub_monitor` 读到 `src=3`、`i2c_hw_ready=1`、`i2c_read_err=0`、`i2c_fallback_active=0` |
| ICM20608 | VERIFIED（经 mx6hub） | `WHO_AM_I=0xaf`、`src=4`、`spi_hw_ready=1`、`spi_read_err=0` |
| mx6hub / Debugfs | VERIFIED | `/dev/mx6hub0` 存在；3 秒窗口 `processed=22`、`drop=0`，四个统计节点可读 |
| WM8960 | PARTIAL | `aplay -l` 与 `arecord -l` 均枚举到 `wm8960-audio`；未播放或录制 |
| OV5640 / V4L2 | VERIFIED（原生插接） | 320×240 连续 10 秒 150 帧；640×480 输出 30 帧；画面见 `logs/ov5640_native_frame_20261006.png` |
| 远程视频 | VERIFIED（基本连通） | UDP 统计链路 6 秒窗口收到帧；接收端有 DTS 单调性告警，FPS 汇总不用于性能结论 |
| LED / BEEP | UNVERIFIED | `/dev/dtsplatled`、`/dev/miscbeep` 不存在 |
| Ftrace / Kprobes | PARTIAL | `available_tracers` 有 `function_graph function nop`，`kprobe_events` 存在；未注册探针 |

LED/BEEP 与音频实际录放后来各自单独验收（Phase 5）；本表是决策时的事实底稿，不再逐项回填。

## OV5640：一次内核 Oops 与它的修复

延长线阶段相机能过 I2C、`/dev/video1` 也在，但采集返回 `VIDIOC_STREAMON: Timer expired`、
内核报 `timeout when wait for SOF`。真正致命的是下一步：旧 `mx6s_capture` 在**上一次启动失败后
没有归还 vb2 缓冲**，再次 `STREAMON` 时对空链表解引用，直接 Oops（原始日志
`logs/ov5640_oops_20261006.log`）。在 `drivers/media/platform/mxc/subdev/mx6s_capture.c`
补上失败路径的缓冲归还与空链表检查后，同样的延长线状态下连续两次**只报 SOF 超时、不再 Oops**。

这条是"软件失败路径"的修复，与接线是否可靠是两件事——下面的长测只回答后者。

## OV5640：连续采集长测一览

跑法统一为 `bash scripts/verify_ov5640_2min.sh`（`CAPTURE_SEC` 控制时长）：
每轮先重载模块，失败或失联才进下一轮。

| 时长 | 结果 | 备注 |
| --- | --- | --- |
| 2 分钟 | 1801 帧，`drop=0`，退出码 0 | 最早一次；跑次日志已清理，结论保留 |
| 5 分钟 | 4503 帧，`drop=0` | 复跑一次结果逐字相同（4503 帧），只留 10-07 那份 `logs/ov5640_300s_20261007_001609/` |
| 662 秒（跨午夜） | 9936 帧，`drop=0` | 第 1 轮重载后立即 `/dev/video1: Operation not permitted`、0 帧；第 2 轮才跑通。证据 `logs/ov5640_662s_20261006_234858/` |
| 固定线束后 5 分钟 | 4503 帧，`drop=0`，约 15.0 fps | 第 1 轮即成功，未触发重试。证据同上目录族 |

**结论**：延长线正插可以连续采至少 11 分钟；但**重载后存在偶发的"首次打开失败"**，
初始化期 `ov5640_read_reg/write_reg` 报错也一直没消失。两者要分开看——
前者是真正打断采集的故障（`Operation not permitted`），后者是 probe 阶段可自愈的偶发重试，
报完之后照常 `camera ov5640, is found`。

据此定下的策略（现在是 `scripts/lib/camera_reload.sh` 与 `securityd` 的相机自愈）：
**出问题不硬撑，有序归还模块退出，再重载重试，默认最多 3 轮**；模块加载后静置 2 秒；
全部轮次都失败时按加载的相反顺序 `rmmod` 再退出。这是工程取舍，不是"修好了"——
所以 `verify_ov5640_2min.sh` 报告写 0 帧失败与"重试后通过"是两回事，不许混着说。

## 云台：PCA9685 与双 MG90S

PCA9685 在 `/dev/i2c-1`、地址 `0x40`；CH0 水平、CH1 垂直，用 50 Hz、中心 307 tick、±8 tick
的短时试动，用户均确认看到转动。**这只证明中间小范围可控**——机械端点、方向映射都还没标定，
真正的限位是 Phase 2 用 `pantilt_test calib` 标出来、写进 `config/pantilt_calibration.conf` 的。
