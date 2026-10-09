#!/usr/bin/env bash
set -euo pipefail

# Board-side full-stack smoke and coordination check.
# It keeps each subsystem in a short, bounded window so failures are easy to
# isolate without leaving long-running processes behind.

WORK_DIR="${WORK_DIR:-/root/imx6hub_demo}"
LOG_DIR="${LOG_DIR:-$WORK_DIR/full_stack_logs}"
SENSOR_SEC="${SENSOR_SEC:-20}"
VIDEO_SEC="${VIDEO_SEC:-10}"
VIDEO_SIZE="${VIDEO_SIZE:-320x240}"
RUN_VIDEO="${RUN_VIDEO:-1}"
RUN_AUDIO="${RUN_AUDIO:-1}"
RUN_LED_BEEP="${RUN_LED_BEEP:-1}"

DBG_DIR="/sys/kernel/debug/mx6hub"
TS="$(date +%Y%m%d_%H%M%S)"
OUT="$LOG_DIR/full_stack_${TS}.log"

fail() {
  echo "[full] FAIL: $*" | tee -a "$OUT"
  exit 1
}

run_cmd() {
  local title="$1"
  shift
  echo "=== ${title} ===" | tee -a "$OUT"
  "$@" 2>&1 | tee -a "$OUT"
}

snapshot_proc() {
  echo "--- proc snapshot: $1 ---" | tee -a "$OUT"
  echo "loadavg=$(cat /proc/loadavg 2>/dev/null || true)" | tee -a "$OUT"
  awk '/MemTotal|MemFree|MemAvailable|Buffers|Cached|Slab/ {print}' /proc/meminfo 2>/dev/null | tee -a "$OUT"
  ps 2>/dev/null | grep -E "hub_monitor|v4l2_fb_preview|ffmpeg|aplay|arecord" | grep -v grep | tee -a "$OUT" || true
}

snapshot_debugfs() {
  local label="$1"

  echo "--- mx6hub debugfs: ${label} ---" | tee -a "$OUT"
  if [ -d "$DBG_DIR" ]; then
    for node in irq_stat fifo_stat drop_stat latency_stat; do
      if [ -r "$DBG_DIR/$node" ]; then
        echo "[$node]" | tee -a "$OUT"
        cat "$DBG_DIR/$node" | tee -a "$OUT"
      fi
    done
  else
    echo "debugfs dir missing: $DBG_DIR" | tee -a "$OUT"
  fi
}

safe_led_beep_probe() {
  echo "=== LED/BEEP minimal control ===" | tee -a "$OUT"
  if [ "$RUN_LED_BEEP" != "1" ]; then
    echo "skip: RUN_LED_BEEP=$RUN_LED_BEEP" | tee -a "$OUT"
    return
  fi

  if [ -e /dev/dtsplatled ]; then
    timeout 2 sh -c "printf '\001' > /dev/dtsplatled" 2>&1 | tee -a "$OUT" || true
    timeout 2 sh -c "printf '\000' > /dev/dtsplatled" 2>&1 | tee -a "$OUT" || true
  else
    echo "skip led: /dev/dtsplatled missing" | tee -a "$OUT"
  fi

  if [ -e /dev/miscbeep ]; then
    timeout 2 sh -c "printf '\001' > /dev/miscbeep" 2>&1 | tee -a "$OUT" || true
    timeout 2 sh -c "printf '\000' > /dev/miscbeep" 2>&1 | tee -a "$OUT" || true
  else
    echo "skip beep: /dev/miscbeep missing" | tee -a "$OUT"
  fi
}

run_audio_probe() {
  echo "=== WM8960/ALSA probe ===" | tee -a "$OUT"
  if [ "$RUN_AUDIO" != "1" ]; then
    echo "skip: RUN_AUDIO=$RUN_AUDIO" | tee -a "$OUT"
    return
  fi

  if [ -x ./check_wm8960_baseline.sh ]; then
    bash ./check_wm8960_baseline.sh 2>&1 | tee -a "$OUT" || true
  else
    cat /proc/asound/cards 2>/dev/null | tee -a "$OUT" || true
    ls -l /dev/snd 2>/dev/null | tee -a "$OUT" || true
  fi
}

run_video_probe() {
  echo "=== OV5640/V4L2 probe ===" | tee -a "$OUT"
  if [ "$RUN_VIDEO" != "1" ]; then
    echo "skip: RUN_VIDEO=$RUN_VIDEO" | tee -a "$OUT"
    return
  fi

  if [ ! -x ./v4l2_fb_preview ]; then
    echo "skip video: ./v4l2_fb_preview missing or not executable" | tee -a "$OUT"
    return
  fi
  if [ ! -e /dev/video1 ]; then
    echo "skip video: /dev/video1 missing" | tee -a "$OUT"
    return
  fi

  timeout "$((VIDEO_SEC + 5))" ./v4l2_fb_preview \
    --dev /dev/video1 --fb /dev/fb0 --size "$VIDEO_SIZE" \
    --mode stream --sec "$VIDEO_SEC" 2>&1 | tee -a "$OUT" || true
}

cd "$WORK_DIR"
mkdir -p "$LOG_DIR"

{
  echo "=== FULL STACK VERIFY ==="
  echo "timestamp=$(date)"
  echo "kernel=$(uname -a)"
  echo "work_dir=$WORK_DIR"
  echo "sensor_sec=$SENSOR_SEC"
  echo "video_sec=$VIDEO_SEC"
  echo "video_size=$VIDEO_SIZE"
  echo
} > "$OUT"

mount -t debugfs none /sys/kernel/debug 2>/dev/null || true

[ -f ./mx6hub.ko ] || fail "mx6hub.ko missing in $WORK_DIR"
[ -x ./hub_monitor ] || fail "hub_monitor missing or not executable in $WORK_DIR"

snapshot_proc "before"

if [ -x ./board_diag_ap3216c_led_beep.sh ]; then
  RUN_INSMOD_TEST=0 sh ./board_diag_ap3216c_led_beep.sh 2>&1 | tee -a "$OUT" || true
fi

echo "=== load mx6hub ===" | tee -a "$OUT"
/sbin/rmmod icm20608 2>/dev/null || true
/sbin/rmmod mx6hub 2>/dev/null || true
dmesg -c >/dev/null 2>&1 || true
/sbin/insmod ./mx6hub.ko 2>&1 | tee -a "$OUT"
sleep 1

run_cmd "mx6hub key dmesg" sh -c "dmesg | grep -Ei 'mx6hub|WHO_AM_I|software CS|fallback|recover|ap3216c' | tail -n 160 || true"
snapshot_debugfs "after load"

echo "=== sensor hub monitor with worker queue ===" | tee -a "$OUT"
./hub_monitor --sec "$SENSOR_SEC" --nonblock --workers 2 --queue-depth 128 --lat-samples 8192 2>&1 | tee -a "$OUT"
snapshot_debugfs "after sensor monitor"

safe_led_beep_probe
run_audio_probe
run_video_probe

snapshot_proc "after"

echo "=== final dmesg tail ===" | tee -a "$OUT"
dmesg | tail -n 120 | tee -a "$OUT" || true

echo "[full] PASS log=$OUT" | tee -a "$OUT"
