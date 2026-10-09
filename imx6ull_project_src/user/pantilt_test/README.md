# PCA9685 / Pan-Tilt 最小测试

程序只用 Linux `i2c-dev`。寄存器依据 [NXP PCA9685 数据手册](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf)：`MODE1` 的 bit4 为 SLEEP，`PRE_SCALE` 控制 PWM 频率，通道寄存器从 `LED0_ON_L=0x06` 起每路占 4 字节。

```bash
# Ubuntu 端交叉编译并复制到板端；仅在需要重新部署测试工具时运行。
cd /home/imx6ull/work/imx6ull/imx6ull_project/imx6ull_project_src/user/pantilt_test
make CC=arm-linux-gnueabihf-gcc
scp pantilt_test imx6ull-board:/root/imx6hub_demo/

# 只读探测，允许在未标定时执行；不初始化 PWM，也不驱动舵机。
ssh imx6ull-board '/root/imx6hub_demo/pantilt_test probe 1'
```

2026-10-06 实测：CH0=Pan、CH1=Tilt；总线 1、地址 `0x40` 可读，首次 `MODE1=0x11`、`MODE2=0x04`、`PRE_SCALE=0x1e`。总线 0 同地址无响应。根据 `../../../docs/舵机的相关原理与控制原理.pdf` 和 `../../../docs/利用单片机PWM信号进行舵机控制.pdf` 的 20 ms/1.5 ms 中位参考，在用户确认舵齿位于量程中间后，程序以 50 Hz、约 1.5 ms（307 tick）为起点，对 CH0 和 CH1 分别做 ±8 tick 的短时试动。用户已分别观察到水平和垂直舵机转动，但反馈起步突然、幅度偏大。试动后芯片回到休眠，双轴机械端点仍未标定。`trial` 只用于本次连接验证，正式程序须从已知姿态开始匀速渐进，不能直接复用此起步方式。

```bash
# 以下是本轮诊断命令记录；起步突兀，完成安全姿态标定前不要重复执行。
ssh imx6ull-board '/root/imx6hub_demo/pantilt_test trial 1 0'  # 水平 CH0
ssh imx6ull-board '/root/imx6hub_demo/pantilt_test trial 1 1'  # 垂直 CH1
```

2026-10-07 已完成两轴机械标定，结果见 `config/pantilt_calibration.conf`（pan home=377、tilt home=242）。`nudge` 的 `MIN_TICK`/`MAX_TICK` 应取该文件里的 `safe_min`/`safe_max`，不要另填：

```bash
# 下面的 MIN_TICK/MAX_TICK 取 config/pantilt_calibration.conf 的 safe_min/safe_max；
# 程序不接受未初始化的 200 Hz 状态。
ssh imx6ull-board '/root/imx6hub_demo/pantilt_test nudge 1 0 2 SAFE_MIN SAFE_MAX'
```

`nudge` 读取现有脉宽，只移动 1～4 tick、保持 300 ms 后尝试恢复；拒绝 SLEEP、非约 50 Hz、未配置通道或越界目标。若程序报恢复失败，应立即断开舵机电源并检查设备。无法抵御突然断电或 `SIGKILL`，因此观察机械姿态时保持可随时切断舵机电源。

## 交互式小步标定 `calib`

`calib` 用于在**没有实测安全边界**的情况下做第一次机械标定：先由操作者确认 home 姿态，再逐条命令小步探索，退出时先爬回 home 再关闭通道休眠。它不接受直接写角度或极限值。

```bash
# 交互式会话；SSH 需要 -t 才能正常收发命令。
ssh -t imx6ull-board '/root/imx6hub_demo/pantilt_test calib 1 0 307 40'
```

参数：`BUS CHANNEL HOME_TICK SPAN_TICK`。上例为总线 1、CH0(Pan)、基准 307 tick、本次窗口 ±40 tick。

会话内命令：

| 命令 | 作用 |
| --- | --- |
| `+` / `-` | 按当前步长走一步 |
| `h` | 爬回 home |
| `s N` | 设置步长（1～20 tick） |
| `m min` / `m max` / `m home` | 把当前 tick 记为候选边界或新基准 |
| `p` | 打印当前状态 |
| `q` | 结束并有序回收 |

安全设计：

- **两层限位**。脉宽必须同时落在本次会话窗口 `[home-span, home+span]` 与电气护栏之内，越界命令直接拒绝，不写入芯片。护栏按脉宽定义，不假定机械量程；`home` 靠近护栏端点时窗口会被裁剪并打印实际窗口。护栏当前为 **Pan (CH0) 205..515、Tilt (CH1) 122..515** tick（2026-10-07 放宽，依据见 `rules/Hardware.md` §2）。
- **缓慢爬升**。每 tick 间隔 15 ms（约 22 °/s），不会像 `trial` 那样起步突兀；跨 LEDn_OFF 高低字节时会先置 FULL 位整体关闭通道，避免一个 PWM 周期读到新旧混搭的脉宽（最坏相差 256 tick）。
- **状态准入**。只接受上电默认静止态或本工具上一轮留下的 50 Hz 运行态；其他状态一律拒绝，避免覆盖别的控制器正在输出的 PWM。
- **有序回收**。`q`、EOF、SIGINT 都走同一条回收路径：先爬回 home，再关闭通道、休眠、还原预分频与 MODE1。回收路径不受中断影响，保证走完。
- **仍然无法抵御突然断电**。观察机械姿态时保持可随时切断舵机电源。

### 位置记忆 `pantilt_state.conf`

每次成功移动后都会把该通道最后输出的 tick 写进板端 `/root/imx6hub_demo/pantilt_state.conf`（可用环境变量 `PANTILT_STATE` 覆盖路径），内容形如 `0 377` / `1 242`。下一次在**全关态**启动时，若该通道有落在护栏内的记录，就先按记录值输出再缓慢爬回 home，舵机停在原地不跳变；没有记录才只能取 `home`，此时会打印 `[WARN]` 明说这一步会跳一次。MG90S 没有位置反馈，这个文件是唯一的"上次在哪"，掉电后仍可能被重力带走（tilt 尤其如此）。

## 双通道供电测试 `sweep`

```bash
ssh imx6ull-board '/root/imx6hub_demo/pantilt_test sweep /root/imx6hub_demo/pantilt_calibration.conf 2'
```

参数：`CONF_PATH [CYCLES]`，`CYCLES` 默认 2、上限 20。通道号、home、工作范围全部取自配置文件，命令行不再重复。走三角形：

```text
home → (pan_safe_min, tilt_safe_max) → (pan_safe_max, tilt_safe_min) → home
```

三条边都在同时驱动两个舵机，是这套电源最吃力的工况，用于验证"双舵机动作不引起明显供电异常"。两轴按位移比例插值、同时到位（`ramp_both`），节拍仍是每 tick 15 ms。

配置在写入任何寄存器之前就会被完整校验，任何一条不过都直接退出：不认识的键、值非法、缺字段、`min <= center <= max` 不成立、`safe` 越出 `[min, max]`、`home` 不在工作范围内、工作范围越出该通道的电气护栏、两轴指向同一通道、`prescale` 不是 50 Hz。**不静默忽略**——这份文件是安全边界，忽略一个拼错的键就等于静默放宽一个限位。

回收路径与 `calib` 相同（爬回 home → 两路全关 → 休眠 → 还原预分频与 MODE1）。`calib` 与 `sweep` 的爬升函数都会回填"实际写到的最后位置"，因此中途 Ctrl-C 时回收是从舵机当前所在处爬回 home，而不是先跳回上一个路点。

`calib` 输出的是"本次实际到达的范围"和操作者的标记值，**不等于机械安全边界**；确认边界后要后退若干 tick 再写入 `config/pantilt_calibration.conf`。
