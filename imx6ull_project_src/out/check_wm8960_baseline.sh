#!/usr/bin/env bash
set -euo pipefail

echo "[wm8960] card list"
cat /proc/asound/cards 2>/dev/null || true

echo "[wm8960] devices"
ls -l /dev/snd 2>/dev/null || true

echo "[wm8960] dmesg hints"
dmesg | grep -Ei "wm8960|asoc|sai|snd" | tail -n 80 || true

echo "[wm8960] playback devices"
aplay -l 2>/dev/null || true

echo "[wm8960] capture devices"
arecord -l 2>/dev/null || true
