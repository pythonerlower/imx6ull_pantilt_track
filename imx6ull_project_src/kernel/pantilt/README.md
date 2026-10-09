# pantilt：PCA9685 云台字符驱动

正式控制接口。对上层只暴露 `/dev/pantilt0` 和一组 ioctl，把「目标 tick → 匀速渐进 → 越界钳位 → 超时归位」这套安全逻辑收进内核，用户态不再直接读写 I2C。

与 `user/pantilt_test` 的分工：`pantilt_test` 是**标定/诊断工具**，直接操作 `i2c-dev`，只在没有驱动占用芯片时使用；本驱动是**运行期唯一写方**。两者同时写同一颗 PCA9685 会让舵机不受控，`pantilt_test` 已在动舵机的子命令前检查 `/dev/pantilt0` 并拒绝启动（`probe` 只读，不受影响）。

## 为什么必须放在内核里

PCA9685 的写入是 I2C 传输，会睡眠；而定时器/软中断上下文不允许睡眠。所以渐进爬升用 `delayed_work` + `i2c_transfer` 实现，而不是 `hrtimer`/`timer_list`。这决定了整个驱动是「工作队列 + mutex」的形态，也决定了最小爬升步进受调度延迟限制（实测见下文 `last_latency_us`）。

## 上电到就绪：insmod 不碰芯片

`insmod` 只做只读探测，**不写任何寄存器**，因此不会让舵机抖一下。2026-10-07 实测：完整 `rmmod` → `insmod` 一轮前后，PCA9685 三个关键寄存器字节完全一致。

```bash
# 板端实测记录（rmmod/insmod 前后各查一次，值必须相同）
/root/imx6hub_demo/pantilt_test probe 1
# [INFO] /dev/i2c-1 地址 0x40 有响应: MODE1=0x91 MODE2=0x04 PRE_SCALE=0x1e
```

探测结果分三种，写在 `pantilt_ctl info` 的 `chip_state` 里：

| chip_state | 判据（只读回读） | 处理 |
| --- | --- | --- |
| `ASLEEP` | `MODE2 == 0x04`、`PRE_SCALE` ∈ {`0x1e`} ∪ [115,130]、且**两个通道都关**（每通道 `ON_L/ON_H == 0` 且 `OFF_H` 的 FULL 位 bit4 = 1） | 干净足迹，可直接认领 |
| `RUNNING` | `MODE1` 的 SLEEP 位已清、`MODE2 == 0x04`、`PRE_SCALE` 在 115..130 | 已由他人启动，认领时会回读当前 tick |
| `UNKNOWN` | 以上都不满足 | **拒绝认领**；须先目视确认舵机姿态安全，再 `insmod force_claim=1` |

`UNKNOWN` 拒绝是关键：无法判断芯片处于什么状态时贸然接管，等于用一个猜测的目标值驱动未知姿态的舵机。

判据里三处**故意不这样写**的地方，改回去会退化成已知的坏行为：

- **`ASLEEP` 不要求 `PRE_SCALE` 恰为 `0x1e`，接受整个 115..130。** 旧写法只认上电默认值。而历史上的 `pantilt_exit()` 是"快照式还原"（把进模块时读到的 `PRE_SCALE` 写回去），于是「把模块加载到一颗遗留运行态的芯片上 → 认领 → `rmmod`」会归还出 `SLEEP + PRE_SCALE=121` 这个组合：它既不是 `ASLEEP`（预分频对不上）也不是 `RUNNING`（SLEEP 置位）→ 判 `UNKNOWN` → **模块再也插不进来**，只能 `force_claim` 带人眼兜底。偏偏那个组合根本没有脉宽在输出，比上电默认态还安全，旧写法等于把最安全的状态挡在门外。
  （退出路径现已改为写回上电默认值，见下节，这类组合不会再由**我们**产生。判据仍然放宽，因为认不认得出**别人**留下的芯片，不该取决于我们去过哪里。）
- **`MODE1` 不参与 `ASLEEP` 判据。** 旧写法要求 `MODE1 & ~0x80 == 0x11`。这个掩码本身没错——置 SLEEP 后芯片会自行把 `RESTART` 位拉高，回读是 `0x91`，掩掉 bit7 正是 `0x11`。错在把"SLEEP 位置位"当成**必要条件**：它只是"我们去过哪里"的线索，不是"有没有脉宽"的答案；后者由通道寄存器直接回答。所以一颗 `RUNNING` 但两通道全关的芯片同样没有脉宽，按静止态认领是对的（那里本来也无脉宽可回读）。
- **两个通道一起查，不是只查 `CH0` 为 `0,0,0x10`。** 只查通道 0 的话，通道 1 仍在输出脉宽的芯片会被误判成静止，而认领对静止态是**直接唤醒**的——那一下就会让正带着力的 tilt 跳一下。

（判据与理由见 `pantilt.c` 的 `pt_probe_chip()` / `pt_channels_off()`；`ASLEEP` 与 `RUNNING` 同时成立时按 `ASLEEP` 处理，因为它先判。）

## 认领与爬回 home

`SET_LIMIT` 是**唯一**会唤醒芯片并真正驱动舵机的调用。它先校验再唤醒，顺序如下：

1. 校验参数（见下），不合法直接 `EINVAL`，不碰芯片；
2. `pt_probe_chip()` 判状态，`UNKNOWN` 且未 `force_claim` 则拒绝；
3. 写 `PRE_SCALE=121`（50 Hz）→ 写 `MODE2=0x04` → 写初始脉宽 → 带 AI 位清 SLEEP；
   若 `RESTART` 位为 1，先 `msleep(2)` 写 `RESTART|AI` 再 `msleep(5)`，最后回读确认 SLEEP 已清；
4. 按 `pantilt_state.conf` 给的起始 tick（缺项则用 home）写初值，再以 `slew_ms` 为步长爬回 home。

起始 tick 来自 `pantilt_state.conf` 而不是直接跳到 home，是因为上电后舵机真实位置未知——首次试动已观察到突然的大幅运动（见 `rules/Hardware.md` §1.2）。从「使用者目视确认的当前姿态」起步，才谈得上渐进。

## 运行期：匀速渐进 + 闸门

`SET_TARGET` 只更新目标，不直接写芯片。`pt_slew_work()` 每次把当前值往目标推 **1 tick**，间隔 `slew_ms`。模块默认 15 ms（`PT_DEFAULT_SLEW_MS`），现场用 `pantilt_ctl limit --slew 20` 设成 **20 ms**（即 50 tick/s 的步进速率）。这个过程里：

- **越界钳位计数**：目标超出 `safe_min..safe_max` 时钳到边界，`clamp_low`/`clamp_high` 计数加一。钳位而不是报错，是为了不让上位机因为一次超调攻击整个控制链。
- **看门狗**：只在「目标来自 SET_TARGET」时生效。超过 `watchdog_ms` 没收到新命令就停止爬升进入 `HOLD`，`watchdog_trips` 加一。内部爬回 home 不受影响，否则回家路上会被自己掐断。
  模块自身的默认值是 `PT_DEFAULT_WATCHDOG_MS = 500`，但**上线配置不取这个默认**：
  `SET_LIMIT` 会把它改成 `pantilt_ctl limit --watchdog 1500` 传进来的值，
  所以这条链上驱动看门狗实际是 **1500 ms**（与 [protocol.md](../../../docs/protocol.md) §5 一致）。
- **双层护栏**：`safe_min`/`safe_max` 由用户态配置，但驱动内还有一份硬编码的电气护栏 `{205,122}`..`{515,515}`（见 `pt_guard_min[]`/`pt_guard_max[]`），`SET_LIMIT` 超出它直接拒绝。要放宽必须先改 `rules/Hardware.md` 并留下实测证据。
- **`min`/`max` 不许当命令值下发**：Pan 上端 515 tick（2.51 ms）超出 MG90S 规格上限，不得长时间驻留。驱动只把它当钳位边界。

`SET_LIMIT` 的校验项（任一不满足即 `EINVAL`，不静默钳位）：

- `slew_ms` ∈ 1..100，`watchdog_ms` ≤ 60000；
- `safe_min ≤ safe_max`，且两轴都在电气护栏内；
- `home` ∈ `[safe_min, safe_max]`；
- `start` 在电气护栏内。

## 关模块即有序归还

`rmmod` 的顺序与 `pantilt_test` 的回收路径一致：置 `shutting_down` → `cancel_delayed_work_sync` → 两通道写 `ON=0, OFF_H FULL`（输出常低，一次性 8 字节事务）→ 写 `MODE1=0x11`（置 SLEEP）→ 写 `PRE_SCALE=0x1e`。

最后两步写的是**上电默认值**，不是"进模块时读到的快照"。归还的目标是「芯片处于一个没有 PWM、且自己下次 `insmod` 一定认得的状态」，快照式还原做不到：若模块被加载到一颗遗留运行态的芯片上（板子重启而 V+ 未断电就是这种局面），快照里 `MODE1` 带 AI、`PRE_SCALE` 是 121，写回去得到 `SLEEP|AI|121` —— `pt_probe_chip()` 判它 `UNKNOWN`，模块再也插不进来。写默认值才让 `insmod → 认领 → rmmod → insmod` 成为闭环。

> 回读是 `0x91` 而不是 `0x11`：置 SLEEP 后芯片会自行把 `RESTART` 位（bit7）拉高，表示"输出已关闭、写 1 可恢复"。这是正确结果，不是没生效。

归还后舵机失力，tilt 可能被重力带走——这与既有 `pantilt_test` 的退出行为相同，也是「上电后不得假设舵机一定处于 home」的由来。

若从未 `SET_LIMIT`（`initialized` 为假），退出路径**一个字节都不写**：只 `misc_deregister` + `i2c_unregister_device`。所以「加载后看一眼再卸载」不会对硬件产生任何副作用。

## 接口

ABI 头 `pantilt_uapi.h` 由内核态和用户态共用，结构布局必须一致：

```bash
PANTILT_IOC_SET_LIMIT  # 认领 + 唤醒 + 爬回 home；唯一会驱动舵机的调用
PANTILT_IOC_SET_TARGET # 更新目标 tick（不立即写芯片）
PANTILT_IOC_GET_STATE  # 读状态与计数器
PANTILT_IOC_HOME       # 以当前 slew 速率爬回 home
PANTILT_IOC_STOP       # 停止爬升，进入 HOLD
PANTILT_IOC_KEEPALIVE  # 续命：只把看门狗时钟拨回现在，不改目标、不动芯片
```

`KEEPALIVE`（3B 新增，编号 0x06）解决的是**长行程被自己的看门狗掐断**：一次命令能走的
最大 tick 数就是 `watchdog_ms / slew_ms`，超过它还没走到，看门狗就先超时进 `HOLD` 了。
续命让上层在长移动途中定期延期，而**不需要**反复下发 `SET_TARGET`——后者会顺带改写目标，
把"我还在走"和"我要换个地方"混成同一件事。

它只写 `last_cmd_jiffies`，**不**动 `chip_state`、**不**写任何寄存器，所以它可以被高频调用：
`pantilt_daemon` 就是**每一拍都发**（默认 20 Hz → 每 50 ms 一次，对 1500 ms 的驱动侧看门狗
有 30 倍余量）。未认领时返回 `-EAGAIN`，与 `HOME`/`STOP` 一致。

长行程的分工因此是：**`SET_TARGET` 说去哪儿，`KEEPALIVE` 说"我还在"**。见
[`docs/protocol.md`](../../../docs/protocol.md) §5.1。

`/dev/pantilt0` 用 `test_and_set_bit` 做**单写者独占**：同一时刻只允许一个进程打开写路径，避免两个控制进程互相覆盖目标值（与 `pantilt_test` 的互斥检查是同一类问题，只是发生在内核侧）。`GET_STATE` 在 `mutex` 下整体拷贝，避免读到半更新的结构。

只读诊断走 debugfs，不需要打开设备节点、也就不会和运行中的控制进程抢独占：

```bash
ssh imx6ull-board 'cat /sys/kernel/debug/pantilt/state'
ssh imx6ull-board 'cat /sys/kernel/debug/pantilt/stats'
ssh imx6ull-board 'cat /sys/kernel/debug/pantilt/limit'
```

## 内核日志用英文

本驱动的 `printk`/`dev_*` 全部用英文，中文只出现在源码注释里。原因是板端 `dmesg` 不按 UTF-8 解析 printk 缓冲区，中文会渲染成 `\xffffffe5\xffffffb0\xffffffb1` 这种转义乱码——而 `dmesg` 是排障的主要通道。`mx6hub` 也是同样的约定。

```bash
# 部署与加载；modprobe 不一定可用，用 /sbin 下的绝对路径
cd /home/imx6ull/work/imx6ull/imx6ull_project
make pantilt-ko
scp imx6ull_project_src/kernel/pantilt/pantilt.ko imx6ull-board:/root/imx6hub_demo/
ssh imx6ull-board '/sbin/insmod /root/imx6hub_demo/pantilt.ko'
ssh imx6ull-board 'dmesg | tail -n 3'
ssh imx6ull-board '/sbin/rmmod pantilt'
```

## 这板子跑的是哪份源码

**指纹记 `srcversion`，不记 `.ko` 的 md5**（2026-10-07 定案）。md5 只能证明
"两个文件相同/不同"，证明不了"它出自哪份源码"——而"板子跑的是不是我刚构建的
那份"要的恰恰是后者。srcversion 是内核按源码+配置算出的（ELF 的 `__versions`
段），确定、不含时间戳、**板上运行时可直接读**。

```bash
bash scripts/check_driver_srcversion.sh      # 只读，不 insmod/rmmod，不碰舵机
```

它同时打印三个值，对不上时分开看：本地 `.ko`（我打算部署哪份）、板上部署文件
（上次**真的**推上去哪份）、板上已加载模块（板子**此刻**在跑哪份，这个才是验收要的）。
退出码 0 一致 / 1 不一致 / 2 判不出 / 124 超时 / 255 断连。

手工等价做法：

```bash
modinfo -F srcversion imx6ull_project_src/kernel/pantilt/pantilt.ko
ssh imx6ull-board 'cat /sys/module/pantilt/srcversion'
```

## 模块参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `bus` | 1 | I2C 总线号 |
| `addr` | 0x40 | PCA9685 地址 |
| `force_claim` | 0 | `chip_state=UNKNOWN` 时仍然认领。**须先目视确认舵机姿态安全** |

驱动用 `i2c_new_device()` 注册 `info.type = "pantilt-pca9685"`，故意不去匹配 mainline 的 `pca9685` id：板端 `CONFIG_PWM_PCA9685` 未开启，也没有对应设备树节点，用独立 type 名可以避免将来内核配置变化时有第二个驱动来抢同一颗芯片。
