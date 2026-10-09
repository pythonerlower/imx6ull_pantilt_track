#!/usr/bin/env sh
# Verify custom AP3216C myself driver (with workqueue polling + Input subsystem)
#
# Usage:
#   sh ./board_verify_ap3216c_myself.sh
#
# This driver implements:
#   1. /dev/ap3216c_myself - character device for raw sensor data
#   2. workqueue polling - periodic data sampling (no IRQ required)
#   3. Input subsystem - SW_FRONT_PROXIMITY events on proximity change
#   4. sysfs interface - raw_data, poll_interval, stats nodes

set -u

DEV="0-001e"
BSP_DRV="/sys/bus/i2c/drivers/ap3216c"
MYSELF_DRV="/sys/bus/i2c/drivers/ap3216c_myself"
MYSELF_KO="./ap3216c_myself.ko"
MYSELF_APP="./ap3216c_myself_app"
MYSELF_DEV="/dev/ap3216c_myself"
SAMPLES="${AP3216C_MYSELF_SAMPLES:-20}"
POLL_DURATION="${AP3216C_MYSELF_POLL_DURATION:-10}"  # seconds for input monitoring

echo "[verify-ap3216c-myself] start"
echo "[verify-ap3216c-myself] device: $DEV"

# Check prerequisites
if [ ! -f "$MYSELF_KO" ]; then
  echo "[verify-ap3216c-myself] FAIL: $MYSELF_KO not found"
  exit 1
fi

if [ ! -x "$MYSELF_APP" ]; then
  echo "[verify-ap3216c-myself] FAIL: $MYSELF_APP not found/executable"
  exit 1
fi

if [ ! -d "$BSP_DRV" ]; then
  echo "[verify-ap3216c-myself] WARN: bsp driver dir not found: $BSP_DRV"
fi

if [ ! -e "/sys/bus/i2c/devices/$DEV" ]; then
  echo "[verify-ap3216c-myself] FAIL: i2c device $DEV not found"
  exit 1
fi

# Cleanup function
cleanup() {
  /sbin/rmmod ap3216c_myself 2>/dev/null || true
  if [ -d "$BSP_DRV" ]; then
    echo "$DEV" > "$BSP_DRV/bind" 2>/dev/null || true
  fi
}

trap cleanup EXIT

# Step 1: Unbind BSP driver (if bound)
echo "[verify-ap3216c-myself] unbind BSP owner (if bound)"
echo "$DEV" > "$BSP_DRV/unbind" 2>/dev/null || true

# Clear any previous driver
/sbin/rmmod ap3216c_myself 2>/dev/null || true
dmesg -c >/dev/null 2>&1 || true

# Step 2: Load custom driver
echo "[verify-ap3216c-myself] insmod $MYSELF_KO"
/sbin/insmod "$MYSELF_KO" || {
  echo "[verify-ap3216c-myself] FAIL: insmod ap3216c_myself.ko"
  dmesg | tail -n 50
  exit 1
}

# Step 3: Verify driver registered
if [ ! -d "$MYSELF_DRV" ]; then
  echo "[verify-ap3216c-myself] FAIL: driver sysfs not found: $MYSELF_DRV"
  dmesg | tail -n 50
  exit 1
fi

# Step 4: Verify device bound
if [ ! -L "$MYSELF_DRV/$DEV" ]; then
  echo "[verify-ap3216c-myself] bind device manually"
  echo "$DEV" > "$MYSELF_DRV/bind" 2>/dev/null || true
fi

if [ ! -L "$MYSELF_DRV/$DEV" ]; then
  echo "[verify-ap3216c-myself] FAIL: driver not bound to $DEV"
  dmesg | tail -n 50
  exit 1
fi
echo "[verify-ap3216c-myself] device bound successfully"

# Step 5: Check device node created
if [ ! -e "$MYSELF_DEV" ]; then
  echo "[verify-ap3216c-myself] FAIL: device node $MYSELF_DEV not created"
  dmesg | tail -n 50
  exit 1
fi
echo "[verify-ap3216c-myself] device node $MYSELF_DEV created"

# Step 6: Read sensor data via character device
echo ""
echo "[verify-ap3216c-myself] ===== STEP 1: Character Device Read Test ====="
"$MYSELF_APP" "$MYSELF_DEV" "$SAMPLES" || {
  echo "[verify-ap3216c-myself] FAIL: app read failed"
  dmesg | tail -n 50
  exit 1
}

# Step 7: Check sysfs interface
echo ""
echo "[verify-ap3216c-myself] ===== STEP 2: Sysfs Interface Test ====="
if [ -d "$MYSELF_DRV/$DEV" ]; then
  SYSFS_DIR="$MYSELF_DRV/$DEV"
else
  SYSFS_DIR="/sys/devices/platform/soc/21a0000.i2c/i2c-0/0-001e"
fi

if [ -d "$SYSFS_DIR" ]; then
  echo "[verify-ap3216c-myself] sysfs path: $SYSFS_DIR"
  
  # Find our driver device sysfs path
  for d in /sys/class/ap3216c_myself/*/; do
    if [ -d "$d" ]; then
      if [ -f "$d/raw_data" ]; then
        echo "[verify-ap3216c-myself] Found sysfs at: $d"
        echo "  raw_data: $(cat "$d/raw_data" 2>/dev/null || echo 'N/A')"
        echo "  poll_interval: $(cat "$d/poll_interval" 2>/dev/null || echo 'N/A')"
        echo "  stats: $(cat "$d/stats" 2>/dev/null || echo 'N/A')"
        break
      fi
    fi
  done
else
  echo "[verify-ap3216c-myself] WARN: sysfs path not found"
fi

# Step 8: Check Input subsystem
echo ""
echo "[verify-ap3216c-myself] ===== STEP 3: Input Subsystem Test ====="
INPUT_DEVICES=$(ls /sys/class/input/ 2>/dev/null | grep event || echo "")
if [ -n "$INPUT_DEVICES" ]; then
  echo "[verify-ap3216c-myself] Input devices found: $INPUT_DEVICES"
  
  # Find our input device
  for event in /sys/class/input/event*; do
    if [ -d "$event" ]; then
      NAME=$(cat "$event/device/name" 2>/dev/null || echo "")
      if [ -n "$NAME" ] && echo "$NAME" | grep -qi "ap3216c"; then
        echo "[verify-ap3216c-myself] Found input device: $NAME"
        echo "  path: $event"
        
        # Try to get the event device
        DEV_NAME=$(basename "$event")
        if [ -e "/dev/input/$DEV_NAME" ]; then
          echo "  device: /dev/input/$DEV_NAME"
          echo "[verify-ap3216c-myself] Input device ready"
        fi
      fi
    fi
  done
else
  echo "[verify-ap3216c-myself] WARN: no input devices found"
fi

# Step 9: Monitor proximity events
echo ""
echo "[verify-ap3216c-myself] ===== STEP 4: Proximity Event Monitor ====="
echo "[verify-ap3216c-myself] Monitoring for proximity events (SW_FRONT_PROXIMITY)..."
echo "[verify-ap3216c-myself] Move hand near/far from sensor during ${POLL_DURATION}s"
echo "[verify-ap3216c-myself] Check dmesg for 'proximity: NEAR/FAR' messages"

# Clear dmesg before monitoring
dmesg -c >/dev/null 2>&1 || true

# Wait for some polling cycles
sleep "$POLL_DURATION"

# Check for proximity events in dmesg
PROX_EVENTS=$(dmesg | grep -c "proximity:" 2>/dev/null || echo 0)
echo ""
echo "[verify-ap3216c-myself] Proximity events detected: $PROX_EVENTS"

if [ "$PROX_EVENTS" -gt 0 ]; then
  echo "[verify-ap3216c-myself] Proximity events from dmesg:"
  dmesg | grep "proximity:" | tail -n 20
fi

# Final dmesg check
echo ""
echo "[verify-ap3216c-myself] ===== Final dmesg (ap3216c related) ====="
dmesg | grep -Ei "ap3216c_myself|ap3216c|i2c|probe|fail|error" | tail -n 50 || true

echo ""
echo "[verify-ap3216c-myself] ===== VERIFICATION COMPLETE ====="
echo "[verify-ap3216c-myself] PASS"
exit 0
