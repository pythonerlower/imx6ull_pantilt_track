# Phase 7：ICM20608 WoM 中断、pan 定幅巡检、ftrace 诊断

**状态（2026-10-08）：已完工。** 验收记录见
[`logs/phase7/phase7_acceptance_20261008.md`](../logs/phase7/phase7_acceptance_20261008.md)。
M1 **没过门**（ICM 的 INT 脚在本板接不出中断，见 [`logs/phase7/m1_icm_int_probe_20261008.md`](../logs/phase7/m1_icm_int_probe_20261008.md)），
①因此走 §2.2 的软件阈值备选；**没有写任何 DTS / 驱动改动**，门本身守住了。
本文件保留为当时的计划原文，实际落地的接口以验收记录与 `docs/protocol.md` 为准。

## 0. 协商结论（已定，不再议）

| 议题 | 结论 |
| --- | --- |
| 归属 | 新开 Phase 7，不重开 Phase 6 |
| ① ICM 中断 | **先做板上可行性验证（M1）**，通过前不动 DTS / 驱动 |
| ② 事件通道 | 扩遥测 `reserved[1]`；`securityd` → daemon 走**新增的本地输入**。**这条要改 `pantilt_daemon.c`，Phase 6 红线，已获同意** |
| ③ 工具 | ftrace 纳入交付；debugfs 沿用现有 `pantilt/{state,limit,stats}`，**不新增** |

## 1. 关键约束

1. **ICM20608 的 INT 是接了的**：`6D_INT` → 核心板 J2-22 → `JTAG_MOD` = **GPIO1_IO10**（与 ATK 座 `GBC_LED` 复用）。
   设备树**没有** `interrupts`。与 `AP_INT`(GPIO1_IO01) 是两根不同的脚 → 两个中断可并存。
   证据链见 [phase6 §1 事实 2](phase6_system_integration.md)。
2. **GPIO1_IO10 是否空闲未实测**（只由图纸交叉推出，运行 DTB 里 `spdif` 为 `disabled`）→ M1 去验。
3. **芯片侧 WoM 寄存器项目里已写好**（`ACCEL_WOM_THR`/`ACCEL_INTEL_CTRL`/`INT_PIN_CFG`/`INT_ENABLE`/`INT_STATUS`），
   但 `icm20608_iio.c` 只写了 `INT_ENABLE=0x01`，**没有 `request_irq`** —— 这就是"轮询"的由来。
4. **`securityd` 今天已在采样 ICM20608**（`sample_icm20608()`，500 ms，`vib_mg` 进 STATS）→ 高频采样的落点就在这个函数。
5. **遥测包只有 `pantilt_daemon` 会发**；`securityd` 无任何 socket；daemon 无本地 IPC → §0 ② 的缺口。
6. `/dev/pantilt0` **单打开者**，daemon 一直占着。
7. 内核配置齐全：`DEBUG_FS`/`FTRACE`/`FUNCTION_GRAPH_TRACER`/`CONTEXT_SWITCH_TRACER`/`KPROBE_EVENT` 均 `y`。
8. **Pan 上端 515 tick(2.51 ms) 超 MG90S 规格**（`rules/Hardware.md` §1.3）→ 巡检上界必须 < 515。
9. 单核 A7：巡检摆转与 video/audio 争一个核，节拍要定死。
10. `scripts/debug/` 四个脚本对着 `mx6hub_*`，本链不加载 mx6hub → **死件，本轮只记录不修**。

## 2. 实现

**改这几个文件：** `icm20608_iio.c`（WoM + `request_irq`）、`ap3216c_myself/*.dts`（icm20608 加 `interrupts` + GPIO1_IO10 pinctrl）、
`securityd.c`（阈值判定 + 转高频 + 事件写本地输入）、`pantilt_daemon.c`（新增只读本地输入 + 填 `reserved[1]`）、
`pc/tracker/`（巡检控制律 + 解析 `reserved[1]`）。前两个**只在 M1 过后动**，换 DTB 前先读 DTB 谱系调查。

### 2.1 M1（先做，是门）

板上只读探测，不改源码：`pinctrl`/`gpio` debugfs 看 GPIO1_IO10 有没有被占；再把脚临时配成输入，看电平是否随 ICM20608 的 INT 动作跳。
**能证明"没被认领 + 电平会跳"才过门。** 命令写成脚本，不散在命令行里。

### 2.2 ① WoM 中断（M1 过后）

DTS 加 `interrupts = <10 EDGE_FALLING>` + pinctrl（确认 `GBC_LED` 没在用）；驱动 `request_irq` + WoM 寄存器组，顶半部只置标志、与 `ap3216c_input` 同构。
**注册失败必须退回轮询并大声报**，不能静默成"永远没有事件"。

> **M1 不过门的备选**：退成软件阈值 + 高频采样 —— 不动 DTS/驱动，`sample_icm20608()` 里超阈值就把 `next_vib_ms` 压到 20~50 ms。**它不是中断，文档与说法都要如实这么写。**

### 2.3 ② 事件 → 巡检

`securityd` 写 daemon 本地输入 → daemon 锁存（保持 ~2 s）填进 `reserved[1]` → PC 判上升沿 → **没目标时限幅扫 pan，找到红目标即转 TRACK**。

两条写死：**上界 < 515**（约束 8）；**语义是"PC 扫、云台执行"**（云台无自主，别把 PC 断链误判成巡检停）。

**待定（开工前定）**：巡检超时兜底 `--patrol-max-s`（建议 ≤30 s），到点回 home 并报，**不允许无声无限摆转**。

### 2.4 ③ ftrace

两个靶子各抓一次即可：`irq_handler_entry/exit` 看**中断到底进不进得来**（摇动时有、空闲不刷屏）；
`sched_switch` 给 Phase 6 M7 遗留的单核抢占一次具体记录。脚本放 `scripts/debug/`，
**必须指向 `pantilt`/`icm20608_iio`**（约束 10）。kprobe 本轮不动。

## 3. 验收

**一条主判据：demo 跑通。** ICM 超阈值 → pan 在限幅内巡检 → 找到红目标 → 转正常跟踪，实机走一遍就是通过。

**两条底线**（会动真舵机，不可省）：巡检不越 `safe_min/max` 且上界 < 515；有超时兜底，不会无限摆转。

**一条不回归**：现有 Phase 6 那条链仍能跑（跑一次免操作者配置即可）。

证据留一份进 `logs/phase7/`，不做逐条取证。

## 4. 冲突（明写）

| 既有决议 | 处置 |
| --- | --- |
| Phase 6「不改 daemon 与内核驱动」 | **已获同意**；那条只约束 Phase 6 那一轮 |
| Phase 6「`securityd` 结构性不碰云台」 | **不破** —— 仍不 open `/dev/pantilt0`，只发事件 |
| Phase 6 事实 2「INT 未接 CPU」 | 已在 phase6 文档原地更正 |

## 5. 回退

① 换回原 `.ko` + DTB；② PC 关掉巡检开关即回现状（daemon 的本地输入不读不写时行为与现在完全一致）；③ 删脚本。

## 6. Non-goals

PID / YOLO / 换检测算法；新增内核 debugfs 节点；修 `scripts/debug/` 死件；kprobe / GDB；Phase 6 §6.6 其余性能优化；让 `securityd` 直接 open `/dev/pantilt0`。
