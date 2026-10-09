#!/usr/bin/env sh
# Board-side diagnostic for AP3216C/LED/BEEP resource ownership conflicts.
# Usage:
#   sh ./board_diag_ap3216c_led_beep.sh
# Optional:
#   RUN_INSMOD_TEST=1 sh ./board_diag_ap3216c_led_beep.sh

set -u

RUN_INSMOD_TEST="${RUN_INSMOD_TEST:-1}"

echo "[diag] timestamp: $(date)"
echo "[diag] kernel: $(uname -r)"
echo

echo "[diag][H1] AP3216C maybe already registered/bound by BSP"
if [ -d /sys/bus/i2c/drivers/ap3216c ]; then
  echo "  - /sys/bus/i2c/drivers/ap3216c exists"
  ls -l /sys/bus/i2c/drivers/ap3216c 2>/dev/null || true
  echo "  - bound devices under ap3216c:"
  find /sys/bus/i2c/drivers/ap3216c -maxdepth 1 -type l 2>/dev/null || true
else
  echo "  - ap3216c i2c driver dir not found"
fi

if [ -e /sys/bus/i2c/devices/1-001e/name ]; then
  echo "  - i2c 1-001e name: $(cat /sys/bus/i2c/devices/1-001e/name 2>/dev/null)"
fi

if [ -e /sys/bus/i2c/devices/0-001e/name ]; then
  echo "  - i2c 0-001e name: $(cat /sys/bus/i2c/devices/0-001e/name 2>/dev/null)"
fi

if [ -e /dev/ap3216c ]; then
  echo "  - /dev/ap3216c exists:"
  ls -l /dev/ap3216c
else
  echo "  - /dev/ap3216c not found"
fi

echo "  - lsmod ap3216c:"
lsmod | grep -i ap3216c || true
echo

echo "[diag][H2] LED/BEEP gpio maybe occupied by BSP leds framework"
ls -l /sys/class/leds 2>/dev/null || true
mount -t debugfs none /sys/kernel/debug 2>/dev/null || true
if [ -e /sys/kernel/debug/gpio ]; then
  echo "  - gpio snapshot (filtered):"
  cat /sys/kernel/debug/gpio | grep -Ei "led|beep|gpio" || true
fi
echo

if [ "$RUN_INSMOD_TEST" = "1" ]; then
  echo "[diag][repro] try insmod baseline modules to reproduce conflict signatures"
  dmesg -c >/dev/null 2>&1 || true
  /sbin/rmmod leddriver 2>/dev/null || true
  /sbin/rmmod miscbeep 2>/dev/null || true

  if [ -f ./leddriver.ko ]; then
    /sbin/insmod ./leddriver.ko || true
  else
    echo "  - skip leddriver.ko (not found in current dir)"
  fi

  if [ -f ./miscbeep.ko ]; then
    /sbin/insmod ./miscbeep.ko || true
  else
    echo "  - skip miscbeep.ko (not found in current dir)"
  fi

  echo "  - key dmesg:"
  dmesg | grep -Ei "led|beep|gpio_request|failed|already registered|ap3216c" | tail -n 120 || true
fi
echo

echo "[diag][summary]"
if [ -d /sys/bus/i2c/drivers/ap3216c ] && find /sys/bus/i2c/drivers/ap3216c -maxdepth 1 -type l 2>/dev/null | grep -q .; then
  echo "  - AP3216C likely already bound by BSP (duplicate-register conflict expected)."
else
  echo "  - AP3216C not clearly pre-bound; need deeper i2c probe checks."
fi

if [ -d /sys/class/leds ] && ls /sys/class/leds 2>/dev/null | grep -Eq "beep|sys-led"; then
  echo "  - LED/BEEP likely owned by BSP leds framework (gpio EBUSY expected)."
else
  echo "  - LED/BEEP default ownership not obvious; inspect DTS and gpio users."
fi

echo "[diag] done"
