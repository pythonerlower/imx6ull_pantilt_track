#!/usr/bin/env bash
set -euo pipefail

WORK_DIR="$(cd "$(dirname "$0")" && pwd)"
MX6HUB_KO="${MX6HUB_KO:-$WORK_DIR/mx6hub.ko}"
HUB_MON="${HUB_MON:-$WORK_DIR/hub_monitor}"
DBG_IRQ_STAT="/sys/kernel/debug/mx6hub/irq_stat"
MON_SEC="${MON_SEC:-8}"
LOG_FILE="${LOG_FILE:-$WORK_DIR/mx6hub_poll_verify.log}"

fail() {
  echo "[verify] FAIL: $*" >&2
  exit 1
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "missing command: $1"
}

need_cmd /sbin/insmod
need_cmd /sbin/rmmod
need_cmd dmesg
need_cmd grep
need_cmd awk
need_cmd tee

[[ -f "$MX6HUB_KO" ]] || fail "mx6hub ko not found: $MX6HUB_KO"
[[ -x "$HUB_MON" ]] || fail "hub_monitor not executable: $HUB_MON"

echo "[verify] unload old modules"
/sbin/rmmod icm20608 2>/dev/null || true
/sbin/rmmod mx6hub 2>/dev/null || true
dmesg -c >/dev/null 2>&1 || true

echo "[verify] insmod mx6hub.ko"
/sbin/insmod "$MX6HUB_KO"

echo "[verify] check key dmesg"
key_log="$(dmesg | grep -Ei "mx6hub: (spi probe|using software CS|WHO_AM_I|icm20608 online|probe fail|not bound/ready)" || true)"
echo "$key_log"

echo "$key_log" | grep -qi "using software CS" || fail "no software CS log"
echo "$key_log" | grep -Eqi "WHO_AM_I=0x(af|ae)" || fail "WHO_AM_I not valid"
echo "$key_log" | grep -qi "icm20608 online" || fail "icm20608 online log missing"

[[ -r "$DBG_IRQ_STAT" ]] || fail "debugfs stat missing: $DBG_IRQ_STAT"
echo "[verify] irq_stat"
cat "$DBG_IRQ_STAT"

spi_hw_ready="$(awk -F= '/^spi_hw_ready=/{print $2}' "$DBG_IRQ_STAT" | tr -d '[:space:]')"
spi_whoami="$(awk -F= '/^spi_whoami=/{print $2}' "$DBG_IRQ_STAT" | tr -d '[:space:]')"
[[ "$spi_hw_ready" == "1" ]] || fail "spi_hw_ready=$spi_hw_ready"
[[ "$spi_whoami" == "0xaf" || "$spi_whoami" == "0xae" ]] || fail "spi_whoami=$spi_whoami"

echo "[verify] run hub_monitor ${MON_SEC}s"
"$HUB_MON" --sec "$MON_SEC" --nonblock | tee "$LOG_FILE"

# 中文说明：
# - hub_monitor 当前会输出 `src=3(AP3216C)` / `src=4(ICM20608)` 这类带标签格式，
#   因此这里按 `^src=3` / `^src=4` 做前缀匹配，兼容纯数字和带设备名两种输出。
grep -q "^src=3" "$LOG_FILE" || fail "no i2c events (src=3)"
grep -q "^src=4" "$LOG_FILE" || fail "no spi events (src=4)"

stats_line="$(grep '^stats ' "$LOG_FILE" | tail -n 1 || true)"
[[ -n "$stats_line" ]] || fail "missing stats line"
drop_val="$(echo "$stats_line" | awk '{for(i=1;i<=NF;i++){if($i~"^drop="){split($i,a,"=");print a[2]}}}')"
[[ "$drop_val" == "0" ]] || fail "drop=$drop_val"

echo "[verify] post-run irq_stat"
cat "$DBG_IRQ_STAT"

# 中文说明：
# - AP3216C 的 hw_ready/real_events 只有在实际调度 workqueue 并完成一次 I2C 读后才会更新。
# - 因此这类验收必须放在 hub_monitor 跑完之后做，避免“还未采样就判失败”的假阴性。
i2c_hw_ready="$(awk -F= '/^i2c_hw_ready=/{print $2}' "$DBG_IRQ_STAT" | tr -d '[:space:]')"
i2c_fallback_active="$(awk -F= '/^i2c_fallback_active=/{print $2}' "$DBG_IRQ_STAT" | tr -d '[:space:]')"
i2c_real_events="$(awk -F= '/^i2c_real_events=/{print $2}' "$DBG_IRQ_STAT" | tr -d '[:space:]')"
spi_real_events="$(awk -F= '/^spi_real_events=/{print $2}' "$DBG_IRQ_STAT" | tr -d '[:space:]')"
spi_fallback_events="$(awk -F= '/^spi_fallback_events=/{print $2}' "$DBG_IRQ_STAT" | tr -d '[:space:]')"
spi_reinit_cnt="$(awk -F= '/^spi_reinit_cnt=/{print $2}' "$DBG_IRQ_STAT" | tr -d '[:space:]')"

[[ "$i2c_hw_ready" == "1" ]] || fail "i2c_hw_ready=$i2c_hw_ready"
[[ "$i2c_fallback_active" == "0" ]] || fail "i2c_fallback_active=$i2c_fallback_active"
[[ "${i2c_real_events:-0}" =~ ^[0-9]+$ ]] || fail "invalid i2c_real_events=$i2c_real_events"
[[ "$i2c_real_events" -gt 0 ]] || fail "i2c_real_events=$i2c_real_events"
[[ "${spi_real_events:-0}" =~ ^[0-9]+$ ]] || fail "invalid spi_real_events=$spi_real_events"
[[ "${spi_fallback_events:-0}" =~ ^[0-9]+$ ]] || fail "invalid spi_fallback_events=$spi_fallback_events"
[[ "${spi_reinit_cnt:-0}" =~ ^[0-9]+$ ]] || fail "invalid spi_reinit_cnt=$spi_reinit_cnt"

echo "[verify] PASS"
