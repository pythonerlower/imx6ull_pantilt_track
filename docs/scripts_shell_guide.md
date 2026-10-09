# 脚本使用说明

本页只列当前实际存在的入口。脚本默认由 Ubuntu 开发机运行，名称以 `scripts/` 为根，程序源文件统一在 `imx6ull_project_src/` 下。执行前请先读 [基线结果](baseline_2026_pantilt.md)；摄像头链路在接线固定后已能连续采集 5 分钟以上，但重载后的偶发首次打开失败与初始化 I2C 错误尚未定位。

| 入口 | 用途与位置 | 注意 |
| --- | --- | --- |
| `check_host_env.sh` | 主机工具链和源码路径检查 | 只读，不连接开发板 |
| `build_baseline.sh` | 构建 LED、BEEP、AP3216C、ICM20608 基线模块及应用 | 使用当前 `ap3216c_myself/` 源码 |
| `build_all.sh` | 基线 + 应用 + `mx6hub` + `pantilt`/`video_link`/`pantilt_daemon` + Phase 4 传感器模块 + Phase 5 音频程序 + Phase 6 `securityd`，共 9 步 | `make build` 调用 |
| `deploy_offline.sh` | 先编译再经 SSH/SCP 上传到 `/root/imx6hub_demo` | 写板端文件；`SKIP_BUILD=1` 仅跳过构建 |
| `check_driver_srcversion.sh` | 对账板上**已加载**的 `pantilt` 驱动与本地构建是不是同一份源码（比 `srcversion`，并列出本地 `.ko` / 板上部署文件 / 板上已加载模块三个值） | 只读，不 `insmod`/`rmmod`，不碰舵机；0 一致 / 1 不一致 / 2 判不出 |
| `board_verify_full_stack.sh` | 板端旧系统短窗口检查 | 会卸载/加载模块并清空 `dmesg`，当前不宜反复运行 |
| `board_verify_icm20608.sh`、`board_verify_mx6hub_poll.sh` | 两种互斥的 SPI 驱动验证 | 两脚本会切换模块占用；先确认板端状态 |
| `board_verify_sensors.sh` | **Phase 4 合并验收**：ICM20608→IIO、AP3216C→Input、A/B 与回退、M2 runtime PM/校准节点/空闲中断回归，共 6 段一次跑完 | 会 `insmod`/`rmmod` 新模块并在结尾回绑 BSP；`WITH_HAND=0` 跳过需人工的手掌贴近段（自动段约 90 秒）。**取代**已删除的 `board_verify_icm20608_iio.sh` / `board_verify_ap3216c_input.sh` |
| `check_wm8960_baseline.sh` | 枚举 WM8960 和相关日志 | 枚举成功不代表录放音成功 |
| `board_verify_audio.sh` | **Phase 5 合并验收**：录音到默认目录/指定路径参数、回放文件/单音/告警音、声学回环、XRUN 恢复、SIGTERM 停止，共 7 段一次跑完；录音判据含 `nframes>0` 与退出码 0（空 WAV 头 / 半截数据都判失败） | 只读硬件状态：不改 mixer、不装模块、不碰 boot；`WITH_LOOPBACK=0` 跳过需喇叭的声学回环段；每段留的录音证据在板端 `/root/imx6hub_demo/records/` |
| `board_verify_phase6.sh` | **Phase 6 板端验收**：`securityd` 状态机、子进程监督、摄像头重载自愈/耗尽、`SIGUSR1` dump、传感器缺失自检、debugfs 对账，共 11 段一次跑完给 PASS/FAIL | 会 `rmmod`/`insmod` 传感器与摄像头模块并在收尾装回；**不重启板子、不给云台下控制包**；`WITH_HAND=0`/`WITH_EXHAUST=0` 跳过两段需人工的（记 NOT RUN，不拉低总判定），`M4_STOP_DAEMON=1` 才允许临时停 `pantilt_daemon` |
| `phase6_e2e_run.sh` | **Phase 6 端到端编排**（开发机侧）：同时拉起板端会话与 PC 侧 tracker，两侧一起取证 | 需操作者在场（手掌触发 + 画面里要有目标）；证据落 `logs/phase6/e2e_<ts>/` |
| `phase6_m6_ab.sh`、`phase6_m7_run.sh`、`phase6_m7_drive.py`、`phase6_telem_capture.py` | Phase 6 的 M6（Ring Buffer slots A/B）与 M7（看门狗归属）诊断工具 | **只在开发机侧跑，不上板**；结论见 `plan/phase6_system_integration.md` §4.4/§4.5 |
| `verify_phase7_nr.sh` | **Phase 7 日常回归**（约 40 秒、不需人在场）：线格式逐字节对拍、PC 单测、板端 `securityd` 默认参数行为、遥测可解、收尾无孤儿，共 9 条判定 | 故意**不传任何 Phase 7 开关**，验的是"全关 = Phase 6"；`TELEM_PORT` 默认 9111（避开在跑的 tracker 占的 9101） |
| `run_video_link.sh`、`verify_ov5640_2min.sh` | 视频链路单跑；摄像头长测（`CAPTURE_SEC` 控时长，每轮先重载模块） | 每次跑在 `logs/video_link_<ts>/` 留一份跑次记录 |
| `debug/phase7_trace.sh` | **Phase 7 ftrace 采集**：`irq` / `sched` / `both` 三模式，判据是日志里的数字 | 只读 tracer；**不碰 DTS、不碰驱动、不动云台** |
| `debug/phase7_m1_gpio_probe.sh` | ICM20608 的 INT 脚（`6D_INT` → `GPIO1_IO10`）板上可行性探测：临时配成 GPIO 输入高速采样，测完恢复 mux/pad | 结论已定（**门没过**，该脚在本板零边沿），一般不用再跑；见 `logs/phase7/m1_icm_int_probe_20261008.md` |
| `remote_link/udp_rtp/`、`rtsp/`、`http_mjpeg/` | 远程视频实验 | 先通过 `/dev/video1` 本地采集 |
| `debug/` | ftrace、kprobe、Debugfs 诊断 | 需按内核实际配置选择 |

`deploy_online.sh`、`run_bench.sh`、`check_board_probe_cmds.sh` 和 `remote_link/ssh/` 当前不存在。历史文档或比较脚本中仍可能出现这些名字，不要把它们当成可运行入口；后续使用前应补齐实现或清理调用链。

```bash
# 主机环境和构建验证，不更改板端。
cd /home/imx6ull/work/imx6ull/imx6ull_project
make host-check
make build

# 已确认待上传产物后，才写入板端部署目录。
make deploy

# 单独编译云台探测程序；动作限制见 imx6ull_project_src/user/pantilt_test/README.md。
make pantilt-test
```
