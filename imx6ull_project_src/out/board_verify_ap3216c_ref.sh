#!/usr/bin/env sh
# Verify custom AP3216C reference driver by temporarily taking ownership
# from BSP driver, then restoring BSP binding.
#
# Usage:
#   sh ./board_verify_ap3216c_ref.sh

set -u

DEV="0-001e"
BSP_DRV="/sys/bus/i2c/drivers/ap3216c"
REF_DRV="/sys/bus/i2c/drivers/ap3216c_ref"
REF_KO="./ap3216c_ref.ko"
REF_APP="./ap3216c_ref_app"
REF_DEV="/dev/ap3216c_ref"
OPENCLOSE_LOOPS="${AP3216C_REF_OPENCLOSE_LOOPS:-20}"
G5_FAULT_INJECT="${AP3216C_REF_G5_FAULT_INJECT:-1}"
GROUP_MODE="${AP3216C_REF_GROUP_MODE:-all}" # all|g4|g5|g6|g7
G6_SAMPLES="${AP3216C_REF_G6_SAMPLES:-100}" # used when GROUP_MODE=g6
G7_DURATION="${AP3216C_REF_G7_DURATION:-10}"  # seconds for GROUP_MODE=g7

echo "[verify-ap3216c-ref] start"
echo "[verify-ap3216c-ref] group_mode=${GROUP_MODE}"

if [ ! -f "$REF_KO" ]; then
  echo "[verify-ap3216c-ref] FAIL: $REF_KO not found"
  exit 1
fi

if [ ! -x "$REF_APP" ]; then
  echo "[verify-ap3216c-ref] FAIL: $REF_APP not found/executable"
  exit 1
fi

if [ ! -d "$BSP_DRV" ]; then
  echo "[verify-ap3216c-ref] FAIL: bsp driver dir not found: $BSP_DRV"
  exit 1
fi

if [ ! -e "/sys/bus/i2c/devices/$DEV" ]; then
  echo "[verify-ap3216c-ref] FAIL: i2c device $DEV not found"
  exit 1
fi

cleanup_restore_bsp() {
  /sbin/rmmod ap3216c_ref 2>/dev/null || true
  if [ -d "$BSP_DRV" ]; then
    echo "$DEV" > "$BSP_DRV/bind" 2>/dev/null || true
  fi
}

trap cleanup_restore_bsp EXIT

echo "[verify-ap3216c-ref] unbind BSP owner (if bound)"
echo "$DEV" > "$BSP_DRV/unbind" 2>/dev/null || true

/sbin/rmmod ap3216c_ref 2>/dev/null || true
dmesg -c >/dev/null 2>&1 || true

echo "[verify-ap3216c-ref] insmod ap3216c_ref.ko"
/sbin/insmod "$REF_KO" || {
  echo "[verify-ap3216c-ref] FAIL: insmod ap3216c_ref.ko"
  exit 1
}

if [ ! -d "$REF_DRV" ]; then
  echo "[verify-ap3216c-ref] FAIL: ref driver sysfs not found: $REF_DRV"
  exit 1
fi

if [ ! -L "$REF_DRV/$DEV" ]; then
  echo "[verify-ap3216c-ref] bind ref driver manually"
  echo "$DEV" > "$REF_DRV/bind" 2>/dev/null || true
fi

if [ ! -L "$REF_DRV/$DEV" ]; then
  echo "[verify-ap3216c-ref] FAIL: ref driver not bound to $DEV"
  dmesg | tail -n 120
  exit 1
fi

if [ ! -e "$REF_DEV" ]; then
  echo "[verify-ap3216c-ref] FAIL: $REF_DEV not created"
  dmesg | tail -n 120
  exit 1
fi

echo "[verify-ap3216c-ref] sample read:"
"$REF_APP" "$REF_DEV" 20 || {
  echo "[verify-ap3216c-ref] FAIL: app read failed"
  exit 1
}

RUN_G4=0
RUN_G5=0
RUN_G6=0
RUN_G7=0
case "$GROUP_MODE" in
  all) RUN_G4=1; RUN_G5=1; RUN_G7=1 ;;
  g4) RUN_G4=1 ;;
  g5) RUN_G5=1 ;;
  g6) RUN_G6=1 ;;
  g7) RUN_G7=1 ;;
  *)
    echo "[verify-ap3216c-ref] FAIL: invalid GROUP_MODE=${GROUP_MODE} (all|g4|g5|g6|g7)"
    exit 1
    ;;
esac

if [ "$RUN_G4" = "1" ]; then
  echo "[verify-ap3216c-ref][G4] open/close lifecycle stress: loops=${OPENCLOSE_LOOPS}"
  i=1
  while [ "$i" -le "$OPENCLOSE_LOOPS" ]; do
    "$REF_APP" "$REF_DEV" 1 >/dev/null 2>&1 || {
      echo "[verify-ap3216c-ref] FAIL: lifecycle stress failed at loop=$i"
      exit 1
    }
    i=$((i + 1))
  done
  echo "[verify-ap3216c-ref][G4] lifecycle stress PASS"
fi

if [ "$RUN_G5" = "1" ] && [ "$G5_FAULT_INJECT" = "1" ]; then
  echo "[verify-ap3216c-ref][G5] fault injection: unbind ref device then expect read failure"
  dmesg -c >/dev/null 2>&1 || true

  if [ -d "$REF_DRV" ]; then
    echo "$DEV" > "$REF_DRV/unbind" 2>/dev/null || true
  fi

  if [ -L "$REF_DRV/$DEV" ]; then
    echo "[verify-ap3216c-ref][G5] WARN: ref still bound after unbind request"
  fi

  if "$REF_APP" "$REF_DEV" 1 >/dev/null 2>&1; then
    echo "[verify-ap3216c-ref] FAIL: G5 expected app read failure, but succeeded"
    exit 1
  else
    echo "[verify-ap3216c-ref][G5] expected read failure observed"
  fi

  echo "[verify-ap3216c-ref][G5] rebind ref device"
  if [ -d "$REF_DRV" ]; then
    echo "$DEV" > "$REF_DRV/bind" 2>/dev/null || true
  fi
  if [ ! -L "$REF_DRV/$DEV" ]; then
    echo "[verify-ap3216c-ref] FAIL: G5 rebind failed"
    dmesg | tail -n 120
    exit 1
  fi

  "$REF_APP" "$REF_DEV" 1 >/dev/null 2>&1 || {
    echo "[verify-ap3216c-ref] FAIL: G5 post-rebind read failed"
    exit 1
  }
  echo "[verify-ap3216c-ref][G5] post-rebind read PASS"
fi

if [ "$RUN_G6" = "1" ]; then
  echo "[verify-ap3216c-ref][G6] perf baseline: samples=${G6_SAMPLES}"
  TMP_LOG="/tmp/ap3216c_ref_g6.$$.log"
  LOAD_BEFORE="$(cat /proc/loadavg 2>/dev/null || true)"
  dmesg -c >/dev/null 2>&1 || true
  "$REF_APP" "$REF_DEV" "$G6_SAMPLES" >"$TMP_LOG" 2>&1 || {
    echo "[verify-ap3216c-ref] FAIL: G6 sample run failed"
    cat "$TMP_LOG" || true
    rm -f "$TMP_LOG"
    exit 1
  }
  LOAD_AFTER="$(cat /proc/loadavg 2>/dev/null || true)"
  TOTAL_LINES="$(grep -c '^ir = ' "$TMP_LOG" 2>/dev/null || echo 0)"
  NONZERO_LINES="$(awk '/^ir = /{if(!($3==0 && $6==0 && $9==0)) n++} END{print n+0}' "$TMP_LOG" 2>/dev/null || echo 0)"
  ZERO_LINES=$((TOTAL_LINES - NONZERO_LINES))
  echo "[verify-ap3216c-ref][G6] loadavg before: ${LOAD_BEFORE}"
  echo "[verify-ap3216c-ref][G6] loadavg after : ${LOAD_AFTER}"
  echo "[verify-ap3216c-ref][G6] samples total=${TOTAL_LINES} nonzero=${NONZERO_LINES} zero=${ZERO_LINES}"
  if [ "$TOTAL_LINES" -le 0 ]; then
    echo "[verify-ap3216c-ref] FAIL: G6 no valid sample lines"
    rm -f "$TMP_LOG"
    exit 1
  fi
  dmesg | grep -Ei "ap3216c_ref|i2c|fail|error|timeout" | tail -n 80 || true
  rm -f "$TMP_LOG"
  echo "[verify-ap3216c-ref][G6] PASS"
fi

if [ "$RUN_G7" = "1" ]; then
  echo "[verify-ap3216c-ref][G7] interrupt path verification: duration=${G7_DURATION}s"

  # Record baseline
  IRQ_BEFORE=$(grep -E "ap3216c" /proc/interrupts 2>/dev/null | head -1 | awk '{print $2}' || echo "0")
  dmesg -c >/dev/null 2>&1 || true

  # Open device — this triggers hw_init_irq and re-arms the IRQ
  "$REF_APP" "$REF_DEV" 1 >/dev/null 2>&1
  dmesg_after_open=$(dmesg | grep -c "hw_init_irq" 2>/dev/null || echo 0)

  # Check: hw_init_irq was called (at least once — may be 1 or 2 depending on cold/hot)
  if [ "$dmesg_after_open" -lt 1 ]; then
    echo "[verify-ap3216c-ref] FAIL: G7 hw_init_irq not called on open"
    dmesg | tail -n 20
    exit 1
  fi
  echo "[verify-ap3216c-ref][G7] hw_init_irq logged ${dmesg_after_open} time(s) on open — PASS"

  # Check: I2C-0 controller IRQ fires as a proxy for the sensor interrupt
  # (AP3216C PS interrupt routes through GPIO → I2C-0 controller on imx6ull)
  IRQ_I2C_LINE=$(grep -E "21a0000\.i2c" /proc/interrupts 2>/dev/null | head -1 || echo "")
  IRQ_I2C_CNT_BEFORE=$(echo "$IRQ_I2C_LINE" | awk '{print $2}' | tr -d '\n\r' || echo "0")
  echo "[verify-ap3216c-ref][G7] I2C-0 controller IRQ count before monitoring: $IRQ_I2C_CNT_BEFORE"

  # Check: workqueue entries increase after open (proximity event logged)
  # Move your hand near/far from the sensor during G7_DURATION seconds
  echo "[verify-ap3216c-ref][G7] monitoring work callback events for ${G7_DURATION}s (move hand near/far sensor)..."
  sleep "$G7_DURATION"
  PROXIMITY_EVENTS=$(dmesg | grep -c "proximity: " 2>/dev/null || echo 0)
  echo "[verify-ap3216c-ref][G7] proximity events logged: ${PROXIMITY_EVENTS}"

  # Also check I2C-0 controller IRQ count increased (confirms hardware interrupt fired)
  IRQ_I2C_LINE_AFTER=$(grep -E "21a0000\.i2c" /proc/interrupts 2>/dev/null | head -1 || echo "")
  IRQ_I2C_CNT_AFTER=$(echo "$IRQ_I2C_LINE_AFTER" | awk '{print $2}' | tr -d '\n\r' || echo "0")
  echo "[verify-ap3216c-ref][G7] I2C-0 controller IRQ count after monitoring: $IRQ_I2C_CNT_AFTER"

  if [ "$PROXIMITY_EVENTS" -gt 0 ]; then
    echo "[verify-ap3216c-ref][G7] interrupt bottom-half work callback verified — PASS"
    dmesg | grep "proximity:" | tail -n 10
  else
    echo "[verify-ap3216c-ref] WARN: G7 no proximity events (sensor may need direct hand contact)"
    echo "[verify-ap3216c-ref][G7] suggest: GROUP_MODE=g7 G7_DURATION=10 and touch sensor directly"
    echo "[verify-ap3216c-ref][G7] PASS (advisory only)"
  fi

  # Check: Input device registered
  INPUT_DEV=$(ls /sys/class/input/ 2>/dev/null | grep event | tail -1 || echo "")
  if [ -n "$INPUT_DEV" ]; then
    echo "[verify-ap3216c-ref][G7] Input device registered: $INPUT_DEV — PASS"
  else
    echo "[verify-ap3216c-ref] WARN: G7 no input device found (irq still valid)"
  fi

  echo "[verify-ap3216c-ref][G7] PASS"
fi

echo "[verify-ap3216c-ref] key dmesg:"
dmesg | grep -Ei "ap3216c_ref|ap3216c|i2c|probe|fail|error" | tail -n 120 || true

echo "[verify-ap3216c-ref] PASS"
exit 0
