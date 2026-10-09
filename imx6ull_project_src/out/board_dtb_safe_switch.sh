#!/usr/bin/env sh
# Safe DTB switch helper with backup + rollback hint.
#
# Usage examples:
#   sh ./board_dtb_safe_switch.sh --bootdir /run/media/mmcblk1p1 \
#       --new imx6ull-alientek-emmc-selfdrv.dtb --current imx6ull-alientek-emmc.dtb
#
# Rollback:
#   cp -f <bootdir>/<current>.bak.<timestamp> <bootdir>/<current>

set -eu

BOOTDIR=""
NEW_DTB=""
CURRENT_DTB=""

while [ $# -gt 0 ]; do
  case "$1" in
    --bootdir) BOOTDIR="$2"; shift 2 ;;
    --new) NEW_DTB="$2"; shift 2 ;;
    --current) CURRENT_DTB="$2"; shift 2 ;;
    *)
      echo "unknown arg: $1"
      exit 2
      ;;
  esac
done

if [ -z "$BOOTDIR" ] || [ -z "$NEW_DTB" ] || [ -z "$CURRENT_DTB" ]; then
  echo "usage: $0 --bootdir <boot_mount> --new <new.dtb> --current <current.dtb>"
  exit 2
fi

if [ ! -d "$BOOTDIR" ]; then
  echo "[dtb-switch] FAIL: bootdir not found: $BOOTDIR"
  exit 1
fi

if [ ! -f "$BOOTDIR/$NEW_DTB" ]; then
  echo "[dtb-switch] FAIL: new dtb not found in bootdir: $BOOTDIR/$NEW_DTB"
  exit 1
fi

if [ ! -f "$BOOTDIR/$CURRENT_DTB" ]; then
  echo "[dtb-switch] FAIL: current dtb not found in bootdir: $BOOTDIR/$CURRENT_DTB"
  exit 1
fi

TS="$(date +%Y%m%d_%H%M%S)"
BAK="$BOOTDIR/$CURRENT_DTB.bak.$TS"

echo "[dtb-switch] backup current dtb -> $BAK"
cp -f "$BOOTDIR/$CURRENT_DTB" "$BAK"

echo "[dtb-switch] switch $CURRENT_DTB <= $NEW_DTB"
cp -f "$BOOTDIR/$NEW_DTB" "$BOOTDIR/$CURRENT_DTB"
sync

echo "[dtb-switch] DONE"
echo "[dtb-switch] rollback cmd:"
echo "  cp -f \"$BAK\" \"$BOOTDIR/$CURRENT_DTB\" && sync"
