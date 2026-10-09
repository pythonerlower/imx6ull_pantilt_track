#!/usr/bin/env sh
set -eu

BOARD_DIR="${1:-/root/imx6hub_demo}"
cd "$BOARD_DIR"

# 中文说明：
# - ICM20608 基线驱动与 mx6hub 都会尝试绑定同一个 spi2.0 设备。
# - 如果不先卸载 mx6hub，后续 insmod icm20608.ko 时 probe 很可能不会被调到，
#   最终表现就是 /dev/icm20608 不存在。
/sbin/rmmod mx6hub 2>/dev/null || true

# 清理可能占用同一设备的实验模块
/sbin/rmmod icm20608_ref62_b1_swcs 2>/dev/null || true
/sbin/rmmod icm20608_ref62 2>/dev/null || true
/sbin/rmmod icm20608 2>/dev/null || true

dmesg -c >/dev/null 2>&1 || true

echo "[verify] insmod icm20608.ko"
/sbin/insmod ./icm20608.ko

echo "[verify] key dmesg"
dmesg | grep -Ei "spi setup|software CS|sw-cs|ICM20608 ID|WHO_AM_I|icm20608" | tail -n 120 || true

if [ ! -e /dev/icm20608 ]; then
  echo "[verify] FAIL: /dev/icm20608 not found"
  echo "[verify] hint: check whether spi2.0 is still owned by another driver"
  ls /sys/bus/spi/drivers 2>/dev/null || true
  ls -l /sys/bus/spi/devices 2>/dev/null || true
  exit 2
fi

echo "[verify] sample read (2s)"
timeout 2 ./icm20608_app /dev/icm20608 2>/dev/null | head -n 24 || true

echo "[verify] PASS"
