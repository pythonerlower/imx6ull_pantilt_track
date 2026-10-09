# pantilt_daemon —— 板端控制面

把 PC 下发的目标翻译成 `/dev/pantilt0` 的 ioctl，并把遥测回送。它是 3B 的
两条链路之一（另一条是 `user/video_link` 的视频链路）。

## 职责边界

| 事情 | 谁做 |
|---|---|
| 认领芯片（唤醒 PCA9685、设定限位、回 home） | `pantilt_ctl limit`，**人工确认** |
| 接收目标、校验、限位前置拦截 | `pantilt_daemon` |
| 绝对钳位、限速爬升、看门狗 | 内核驱动 |
| 视觉、决策 | PC 侧 `pc/tracker` |

daemon **刻意不做认领**。`SET_LIMIT` 会让舵机真的开始转动，那是必须由带确认
提示的工具做的一次性物理动作；daemon 若"顺手代做"，就会在无人看守时让云台
自己动起来。驱动处于 `UNINIT` 时本程序直接以退出码 3 拒绝启动。

## 快速开始

```sh
# 板端
/sbin/insmod pantilt.ko
./pantilt_ctl limit                 # 会提示按 Enter 确认，之后舵机有力
./pantilt_daemon                    # 默认监听 0.0.0.0:9100，遥测回发到 9101

# 开发机（先看遥测能不能通）
python3 pc/tracker/main.py --host 192.168.137.50
```

常用选项见 `./pantilt_daemon --help`。四个值得单独说的：

- `--tick-hz`（默认 20）：同时决定 **KEEPALIVE 与遥测**的频率。
- `--daemon-wd`（默认 500 ms）：`0` 表示禁用 daemon 侧看门狗（不推荐，
  那样 PC 一崩云台会朝最后的方向一直走）。
- `--fake-driver`：不碰硬件，用内存模拟驱动。**只在开发机上用**。
- `--event-sock PATH`（默认空）：本地振动事件输入，见下。

## 振动事件输入（Phase 7，`--event-sock`）

本进程多了一条**只读**的本地输入：`--event-sock` 指定的 `AF_UNIX/SOCK_DGRAM`
套接字，对端是 `securityd`（线格式在 `common/proto/security_event.h`）。收到事件
就累加计数、**锁存 2 s**，编码进遥测 `reserved[1]`（高 16 位计数、低 16 位年龄）。
daemon **自己不读任何传感器**，只搬运；协议细节见 [protocol.md](../../../docs/protocol.md) §3。

**不传 `--event-sock` 时这条路径整体不存在，行为与 Phase 6 逐字节一致**——
它是 3B 冻结协议之上的一次"用预留字段加功能"，不改任何已定义字段的偏移或语义。

## 两级看门狗，别把它们当成一个

| 层 | 阈值 | 触发后 | 兜的是什么 |
|---|---|---|---|
| daemon | 500 ms 无控制包 | 下发 `STOP`，保持位置，状态转 `LOST` | **上位机没了** |
| 驱动 | 1500 ms 无 KEEPALIVE/新目标 | 中止爬升进 `HOLD` | **daemon 自己死了** |

两层都保留是因为它们兜的不是同一件事。只在 daemon 里做看门狗的话，daemon
被 kill 就没人兜底了；只用驱动看门狗的话，PC 一断要等 1.5 s 才停。

## 为什么需要 KEEPALIVE 这个 ioctl

最自然的想法是"心跳就重复下发同一个 `SET_TARGET`"。不行——`SET_TARGET` 是
**无条件的副作用集合**：它 `++cmd_count`、把 `status` 强制回 `SLEW`、并重排
爬升定时器。以 20 Hz 重复下发会：

1. 把 `cmd_count` 稀释成节拍计数器，失去"下发过多少条命令"的含义；
2. 让 `status` 在 `SLEW`/`READY` 之间抖动；
3. 让爬升速率慢约 10%（每一步都被推迟最多一个 `slew_ms`）。

所以驱动侧新增了 `PANTILT_IOC_KEEPALIVE`：**只把看门狗截止时刻往后推，别的
什么都不动**。daemon 于是可以明确表达两件不同的事：

- 每拍 → `KEEPALIVE`（我还活着）
- 目标变化时 → `SET_TARGET`（要它去哪儿）

`check_keepalive.c` 就是给这条改动做的板上验收，见下。

## 遥测往哪儿发

默认回发给**最近一条控制包的源 IP 的 9101 端口**。注意是源 IP、不是源端口：
客户端的控制 socket 通常绑在临时端口上，而它收遥测用的是 `TelemReceiver`
绑定的 9101。照源端口回发的话遥测会全部落到没人监听的端口上，表现是
"控制正常、遥测一条都收不到"。

`--telem-dst HOST:PORT` 可以固定到别的地址（例如 PC 与板端不在同一网段时）。

## 测试

```sh
# 开发机：编宿主版 + 跑整条控制环的端到端联调（不碰板子）
make test

# 只跑 pc 侧单测
make -C ../../pc/tracker test
```

`test_plane.py` 用 `--fake-driver` 起真实 daemon 进程，配真实的
`ControlClient`/`TelemReceiver` 走 UDP 环回，覆盖：遥测连通性与几何参数、
KEEPALIVE 不污染 `cmd_count`、长移动不被看门狗掐断、越界/坏 CRC/重复 seq 的
分类计数、seq 大幅倒退后的会话自愈、断流 STOP 与 HOLD 解除、未认领时拒绝启动。

**它不验电气特性、I2C 错误、真实时序抖动，也不代表舵机真的会动**——
`fake_drv.c` 只是内存里复刻钳位/速率/看门狗。板上验收见下节。

## 板上验收（`check_keepalive`）

驱动是 3B 唯一改动的内核代码，宿主端模拟证明不了"真驱动里 KEEPALIVE 只续命"，
必须拿真硬件对一次答案。

```sh
./check_keepalive              # 只打印计划，不动舵机
./check_keepalive --yes        # 真的会动
```

**它会真的让舵机转动**，所以：不加 `--yes` 只打印计划；全程只用驱动回报的
`safe_min`/`safe_max`，不写死任何"中心值"；结束前必定回 `HOME` 并确认到位。

四个断言：

| | 内容 | 为什么 |
|---|---|---|
| a | 只发 KEEPALIVE 2 s：姿态不变、`cmd_count` 不变、`status` 不发散 | 证明续命真的只续命 |
| b | 20 Hz 续命下推到远端（约 138+ tick，远超 1500 ms）：走完且 `watchdog_trips` 保持 0 | 证明续命能防掐断 |
| c | **对照**：同样一段路不续命，应被看门狗掐断并进 `HOLD` | 没有这条，(b) 可能只是因为看门狗压根没武装 |
| d | 重新下发 `SET_TARGET` 能解除 `HOLD` 并走完 | 验证驱动的恢复路径（也是 daemon 必须重发同值目标的原因） |

`(c)` 是这套检查里最要紧的一条：只测"续命时能动"是没有分辨力的，
必须证明"不续命时确实会停"。

## 退出码

| 码 | 含义 |
|---|---|
| 0 | 正常退出（收到 SIGINT/SIGTERM，已下发 STOP 保持位置） |
| 1 | 参数错误 |
| 2 | 网络错误（socket / bind 失败） |
| 3 | 驱动打不开，或驱动未认领（`UNINIT`） |

daemon 退出**不**写 `SLEEP`、不停输出——那是 `rmmod` 的职责。退出后舵机应保持
有劲留在原处。

## 文件

| 文件 | 内容 |
|---|---|
| `pantilt_daemon.c` | 主程序：协议校验、会话状态机、看门狗、遥测组装 |
| `drv.h` | 驱动访问薄接口（vtable） |
| `real_drv.c` | 真实实现，ioctl 直译，不做任何判断 |
| `fake_drv.c` | 内存模拟，仅复刻钳位/速率/看门狗 |
| `check_keepalive.c` | 板上验收工具（会动舵机，需 `--yes`） |
