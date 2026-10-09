#!/bin/bash
# Phase 7 M1：ICM20608 的 INT（6D_INT → GPIO1_IO10 / MX6UL_PAD_JTAG_MOD）板上可行性探测。
#
# 只做两件事，不改源码、不改 DTB：
#   A. 只读：该脚是否被任何驱动认领（pinctrl / gpio debugfs + spdif 节点）
#   B. 临时把该脚配成 GPIO 输入（带 100k 上拉），高速采样，看电平随 ICM20608 的
#      data-ready 中断（INT_ENABLE=0x01，约 1 kHz、50 us 正脉冲）是否在跳。
#      测完恢复原来的 mux / pad 配置。
#
# 判据（plan/phase7_wom_patrol_debug.md §2.1）：
#   A 显示 UNCLAIMED 且 B 采到明显的 0/1 混合 → 过门；否则不过门，退软件阈值方案。
#
# 这个脚本里只有 icm20608 与 GPIO 相关的东西，不含任何 mx6hub 引用。
#
# 用法（在板端以 root 跑）：
#   sh /root/phase7_m1_gpio_probe.sh
#
# 寄存器（i.MX6ULL，手册 IOMUXC 章节）：
#   IOMUXC base              0x020E0000
#   SW_MUX_CTL_PAD_JTAG_MOD  0x020E0044   (imx6ul-pinfunc.h: MX6UL_PAD_JTAG_MOD__GPIO1_IO10 = 0x0044 0x02D0 0x0 0x5 0x0)
#   SW_PAD_CTL_PAD_JTAG_MOD  0x020E02D0
#   GPIO1 0x0209C000：DR 0x00 / GDIR 0x04 / PSR 0x08（GPIO1_IO10 → bit10）

set -u

MUX_REG=0x020E0044
PAD_REG=0x020E02D0
GPIO1_PSR=0x0209C008

MUX_GPIO_IN=0x15        # ALT5 = GPIO1_IO10，bit4 SION=1 强制输入通路
PAD_IN_PULLUP=0x1B0B0   # HYS + 100k 上拉 + PKE，输入用
GPIO_NUM=42             # 1*32 + 10
DEVMEM=/usr/bin/devmem2

NSAMP=${NSAMP:-50000}

say() { echo "== $*"; }

# devmem2 实际打的是 "Read at address 0x… (0x…): 0x…"，不是 "Value at address"。
# 第一版按后者匹配 → 读到空串 → 恢复分支失效（见 logs/phase7/m1_icm_int_probe_20261008.md §4）。
rd() { $DEVMEM "$1" 2>/dev/null | sed -n 's/^[^:]*): //p'; }

# ------------------------------------------------------------------ A. 只读
say "A. 只读：认领状态"
echo "-- model: $(cat /proc/device-tree/model)"
echo "-- spdif status: $(cat /proc/device-tree/soc/aips-bus@02000000/spba-bus@02000000/spdif@02004000/status 2>/dev/null)"
echo "-- pinctrl pin 17 (MX6UL_PAD_JTAG_MOD):"
grep -n "MX6UL_PAD_JTAG_MOD" /sys/kernel/debug/pinctrl/20e0000.iomuxc/pinmux-pins | sed 's/^/     /'
if grep -q "gpio1-10" /sys/kernel/debug/gpio 2>/dev/null; then
	echo "-- gpio debugfs: gpio1-10 已被认领（如下），M1 不通过"
	grep "gpio1-10" /sys/kernel/debug/gpio | sed 's/^/     /'
else
	echo "-- gpio debugfs: 未见 gpio1-10 认领行"
fi

echo "-- 寄存器现状（改动前，用于恢复）:"
MUX_ORIG=$(rd $MUX_REG)
PAD_ORIG=$(rd $PAD_REG)
echo "     mux[$MUX_REG] = $MUX_ORIG"
echo "     pad[$PAD_REG] = $PAD_ORIG"
echo "     GPIO1_PSR     = $(rd $GPIO1_PSR)"

# ------------------------------------------------------------------ B. 临时配输入 + 采样
cleanup() {
	say "恢复"
	echo "$GPIO_NUM" > /sys/class/gpio/unexport 2>/dev/null
	[ -n "${MUX_ORIG:-}" ] && $DEVMEM $MUX_REG w "$MUX_ORIG" >/dev/null 2>&1
	[ -n "${PAD_ORIG:-}" ] && $DEVMEM $PAD_REG w "$PAD_ORIG" >/dev/null 2>&1
	echo "-- 恢复后 mux=$(rd $MUX_REG) pad=$(rd $PAD_REG)"
}
trap cleanup EXIT

say "B. 临时 mux 成 GPIO 输入并采样"
$DEVMEM $MUX_REG w $MUX_GPIO_IN >/dev/null 2>&1
$DEVMEM $PAD_REG w $PAD_IN_PULLUP >/dev/null 2>&1
echo "-- mux 写入后 = $(rd $MUX_REG)（期望 $MUX_GPIO_IN）"

echo "$GPIO_NUM" > /sys/class/gpio/export 2>/dev/null
echo in > /sys/class/gpio/gpio$GPIO_NUM/direction
echo "-- gpio$GPIO_NUM direction=$(cat /sys/class/gpio/gpio$GPIO_NUM/direction)"

VAL=/sys/class/gpio/gpio$GPIO_NUM/value
# 先看静态电平
echo "-- 静态读 5 次: $(for i in 1 2 3 4 5; do read v < $VAL; printf '%s ' "$v"; done)"

# 高速采样：sysfs gpio 的 value 每次都要重新 open，循环本身就是采样时钟。
ones=0
t0=$(date +%s%N)
for ((i = 0; i < NSAMP; i++)); do
	read -r v < $VAL || v=?
	[ "$v" = "1" ] && ((ones++))
done
t1=$(date +%s%N)

dur_ns=$((t1 - t0))
dur_ms=$((dur_ns / 1000000))
[ "$dur_ms" -eq 0 ] && dur_ms=1
rate=$((NSAMP * 1000 / dur_ms))
pct=$((ones * 1000 / NSAMP))

echo "-- 采样 N=$NSAMP 用时 ${dur_ms}ms（约 ${rate} Hz）"
echo "-- 读到 1 的次数 = $ones（${pct}‰）"
echo

say "判读"
if [ "$ones" -eq 0 ]; then
	echo "结果：全程读到 0 —— 没看到任何脉冲，M1 不过门"
elif [ "$ones" -eq "$NSAMP" ]; then
	echo "结果：全程读到 1 —— 悬空上拉或恒高，没看到脉冲，M1 不过门"
else
	echo "结果：0/1 混合（1 占 ${pct}‰）—— 该脚在被外部驱动、有脉冲，M1 过门"
fi
echo
echo "对照：ICM20608 的 INT_ENABLE=0x01（data-ready），ODR≈1kHz、脉冲≈50us，"
echo "      期望 1 的占比在 50us/采样周期 附近（当前采样率 ${rate} Hz → 期望约 $((50 * rate / 1000000000 * 1000 + 5))‰ 量级，仅作数量级对照）。"
