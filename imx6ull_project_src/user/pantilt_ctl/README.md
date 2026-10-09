# pantilt_ctl：云台驱动的验收与手动控制工具

`/dev/pantilt0` 的命令行前端。设计上只做三件事：把配置文件翻译成 ioctl、把状态打印成人能看的 tick 和毫秒、在会动舵机的命令前停下来等你确认。所有安全逻辑都在驱动里，本工具不重复实现，也不绕过。

## 子命令

```bash
./pantilt_ctl info                    # 只读；不打开写路径，可在控制进程运行时执行
./pantilt_ctl limit [CONF] [--slew MS] [--watchdog MS] [--yes]
./pantilt_ctl target PAN TILT         # 更新目标 tick
./pantilt_ctl home
./pantilt_ctl stop                    # 停止爬升，HOLD
```

`info` 用 `GET_STATE`，是唯一不会影响硬件状态的子命令。其余子命令都需要打开 `/dev/pantilt0` 写路径，因此与正在运行的控制进程互斥——如果 `info` 能用但 `target` 报 `EBUSY`，说明有别的进程正持有写路径，这属于预期行为，不是故障。

## 配置

`limit` 从配置文件读限位，缺项**直接报错退出**，不填默认值：

```bash
# 默认读 config/pantilt_calibration.conf
./pantilt_ctl limit
./pantilt_ctl limit /path/to/other.conf --slew 20 --watchdog 800
```

用到的键：`pan_center_pwm` / `pan_safe_min` / `pan_safe_max` 与 tilt 的对应项。缺任何一个都拒绝继续——限位缺项时用默认值顶上，等于让一个猜测的边界去约束真实的舵机，而 `safe_min`/`safe_max` 是实机标定出来的（见 `config/pantilt_calibration.conf`：pan home=377、tilt home=242）。

三个环境变量用于临时改路径，都不改变上述校验：

| 变量 | 默认 | 说明 |
| --- | --- | --- |
| `PANTILT_DEV` | `/dev/pantilt0` | 设备节点 |
| `PANTILT_CONF` | `config/pantilt_calibration.conf` | 限位配置 |
| `PANTILT_STATE` | `pantilt_state.conf` | 起始姿态 |

## 为什么 `limit` 要停下来等你按回车

`limit` 是唯一会唤醒 PCA9685、驱动舵机的调用（详见 `kernel/pantilt/README.md`）。它会先把认领后的动作打出来，等你确认再执行：

```bash
# 会输出预计爬回 home 的耗时，回车后才真正下发
./pantilt_ctl limit
```

`--yes` 跳过确认，只给已经确认过姿态的自动化脚本用。首次在一个新姿态下运行、或刚换过供电/机械装配后，不要加 `--yes`。

## 起始姿态 `pantilt_state.conf`

唤醒后驱动不会直接跳到 home，而是先写一个「使用者的目视确认位置」再渐进爬回。这个位置从 `pantilt_state.conf` 读，格式是每行 `<channel> <tick>`：

```
0 377
1 242
```

文件不存在或缺某一轴时，该轴退回 home 并打印 WARN。这不是错误——刚上电时它就是应该这样。但它意味着那一步会从 home 直接开始爬，如果舵机实际不在 home 附近，会先出现一段运动。首次试动已观察到突然的大幅运动，所以正式运行前应把当前实际 tick 写进这个文件。

## 构建

```bash
cd /home/imx6ull/work/imx6ull/imx6ull_project
make pantilt-ctl
```

`Makefile` 用 `CROSS_COMPILE ?= arm-linux-gnueabihf-` + `CC := $(CROSS_COMPILE)gcc`，**不能用 `CC ?=`**：make 有内置的 `CC=cc`，`?=` 会被内置值拦住、静默产出 x86-64 二进制，交叉编译看起来却「成功」了。

`user/pantilt_test/Makefile` 有同一个隐患，目前只是因为根 `Makefile` 显式传了 `CC=` 才没暴露。

编译时通过 `-I$(CURDIR)/../../kernel/pantilt` 直接包含驱动侧的 `pantilt_uapi.h`，不复制一份：结构布局一旦不同步，ioctl 会读到错位的字段。

```bash
# 部署（pantilt.ko 见 kernel/pantilt/README.md）
scp imx6ull_project_src/user/pantilt_ctl/pantilt_ctl imx6ull-board:/root/imx6hub_demo/
```

注意 `scp`/`ssh` 要走 `imx6ull-board` 别名：板端是旧版 OpenSSH，只提供 `ssh-rsa` 主机密钥，直接写 `root@192.168.137.50` 会报 `no matching host key type found`。别为此把 `StrictHostKeyChecking` 关掉。
