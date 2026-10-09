# AP3216C 中断调试说明（IMX6ULL / Alpha 底板）

参考：[rEDTii/AP3216C-Linux-Driver](https://github.com/rEDTii/AP3216C-Linux-Driver)（下降沿、`IRQF_TRIGGER_FALLING`、寄存器初始化顺序）。

## 已确认的根因：GPIO1_IO01 引脚冲突

`imx6ull-14x14-evk.dts` 中 **`dht11` 与 `ds18b20` 共用 `<&gpio1 1>`（即 GPIO1_IO01）**，与 AP3216C 的 **INT（原理图 AP_INT → GPIO1_IO01）** 为同一引脚。两者同时 `okay` 时会抢占 GPIO，导致中断脚被配置为单总线输出或错误电气状态，`/proc/interrupts` 中 GPIO 中断计数恒为 0。

**处理：** 在本仓库的 `imx6ull-14x14-emmc-4.3-480x272-c-ap3216c-irq.dts` 根节点合并：

```dts
dht11 { status = "disabled"; };
ds18b20 { status = "disabled"; };
```

使用 AP3216C 中断时请保持上述禁用（或改硬件换脚）。

## 设备树要点（与 Gemini / 手册一致）

- **MUX：** `MX6UL_PAD_GPIO1_IO01__GPIO1_IO01`
- **PAD：** `0x4001b0b0`（内部上拉等，AP3216C INT 为开漏，底板无上拉时需靠 SoC 上拉）
- **中断：** `interrupt-parent = <&gpio1>;` `interrupts = <1 IRQ_TYPE_EDGE_FALLING>;`
- **pinctrl：** `ap3216cgrp` 必须放在 `&iomuxc { imx6ul-evk { ... } }` 下（否则会出现 `no groups defined in ... ap3216cgrp`）

## 驱动与 rEDTii 对齐的寄存器流程

- 软复位 → PS+ALS → `INTCLEAR=0x01` → 写 `INTSTATUS` 清 PS 位 → LED/阈值/`PSLEDWT=0x04`
- **`0x22` PS interrupt form（必须显式配置）**：与 [AP3216_WE](https://github.com/wollewald/AP3216_WE) / 手册一致，`0x00`=ZONE（测量值在阈值带**之外**触发）、`0x01`=HYSTERESIS（滞回，类手机贴近）。若不上电后写入，部分芯片 **PS_INT 永不置位**，`/proc/interrupts` 与 GPIO 均为 0。
- **`PSCONFIG=0x05`**：在阈值与 `0x22` 之后显式写入，确保 PS 与 PS 中断使能。
- **probe 里 `request_irq` 前** 调用 `ap3216c_flush_interrupt_state()`（清 ALS+PS 标志并读 PS/ALS 数据寄存器），减轻 INT 长时间拉低导致“无下降沿”的情况

### 模块参数补充

| 参数 | 含义 |
|------|------|
| `ps_int_mode` | `0`（默认）= ZONE / reg `0x22=0x00`；`1` = HYSTERESIS / `0x01` |
| `irq_poll_ms` | 轮询 `INTSTATUS` 兜底（毫秒），用于区分 GPIO 问题与芯片未置位 |
| `debug_interval_ms` | 周期性打印 PS/INTSTATUS（毫秒），`0`（默认）关闭，避免多余 I2C 读干扰调试 |

## 内核内置 `ap3216c` 与自定义模块

板上若 `ap3216c` 为 **builtin**，会抢先绑定 `0-001e`。测试本模块时请：

```sh
echo 0-001e > /sys/bus/i2c/drivers/ap3216c/unbind
rmmod ap3216c_myself 2>/dev/null
insmod ap3216c_myself.ko
```

## GPIO IRQ 仍无计数时的兜底参数

模块参数 **`irq_poll_ms`**：大于 0 时按该间隔轮询 `INTSTATUS`，若发现 PS 中断位置位则调度与硬中断相同的底半部（用于确认芯片侧是否产生中断、排除纯 GPIO 问题）：

```sh
insmod ap3216c_myself.ko irq_poll_ms=50
```

## 验证命令

```sh
cat /proc/interrupts | grep ap3216c
cat /sys/devices/virtual/ap3216c_myself/ap3216c_myself/prox_counts
dmesg | grep -E 'IRQ triggered|poller|proximity'
```

若 `irq_poll_ms` 能触发 proximity 而 `/proc/interrupts` 仍为 0，重点查 **INT 走线、焊接、电平（示波器）** 及 **pinctrl 是否最终生效**。
