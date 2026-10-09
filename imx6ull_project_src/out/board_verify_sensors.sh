#!/usr/bin/env sh
# Phase 4 合并验收：两颗传感器一次跑完（取代原 board_verify_icm20608_iio.sh /
# board_verify_ap3216c_input.sh）
#
# 设计原则（2026-10-07 依用户要求瘦身）：
#   - 一次 SSH 跑完全部自动项，每颗传感器只保留 2~3 条硬判据
#   - 等待窗口 <=5s（AP3216C 用 ps_measure_interval=0x01 加速到 ~437ms）
#   - A/B 背靠背读一次同轴 raw，只输出一行差值 + 结论
#   - 回退 = 旧模块能重新装载并读到数（不再翻日志）
#   - 人工动作只做一次，窗口 HAND_SECS（默认 15s），同时判 NEAR/FAR 双向
#   - 取样本一律避开"复位后第一次读"：实测 ICM20608 第 1 次读是无效值
#     (az=1107)，第 2 次起才稳定；旧 app 的 \r\n 也必须 tr 掉
#
# 判据：
#   ICM20608 (IIO)   : 按 name 找到设备 + in_accel_z_raw 非 0 + in_temp_raw 非 0
#   AP3216C  (Input) : 绑定成功 + ps_raw/als_raw 可读非 0 + input 节点存在
#   人工项           : 手贴近/移开时用户态收到 value=1 / value=0
#   M2 runtime PM    : 空闲后自动挂起(suspended) + 挂起态 ALS 仍读得到
#   M2 校准节点      : threshold_low/high、measure_interval、led_current_ratio 读写往返
#
# 用法：sh ./board_verify_sensors.sh [BOARD_DIR]
#      WITH_HAND=0 可跳过人工窗口（只跑自动项）
set -u

BOARD_DIR="${1:-/root/imx6hub_demo}"
cd "$BOARD_DIR"

DEV="0-001e"
BSP_DRV="/sys/bus/i2c/drivers/ap3216c"
NEW_I2C_DRV="/sys/bus/i2c/drivers/ap3216c_input"
CLIENT="/sys/bus/i2c/devices/$DEV"
HAND_SECS="${HAND_SECS:-15}"
WITH_HAND="${WITH_HAND:-1}"
# insmod 时未指定阈值，用的就是驱动里的默认值，这里同步一份仅供打印/诊断
THR_LOW=20
THR_HIGH=80

RC_ICM=1
RC_AP=1
RC_AB=1
RC_HAND=0          # 0=不适用/未跑，1=通过，2=失败
RC_PM=1
RC_CAL=1
RC_IRQ=0           # 0=通过/跳过，1=失败

echo "==================== Phase 4 合并验收 ===================="

# ---------------- 1. ICM20608 IIO 新链路 ----------------
echo ""
echo "--- [1/6] ICM20608 -> IIO ---"
/sbin/rmmod mx6hub 2>/dev/null || true
/sbin/rmmod icm20608 2>/dev/null || true
/sbin/rmmod icm20608_iio 2>/dev/null || true
dmesg -c >/dev/null 2>&1 || true
/sbin/insmod ./icm20608_iio.ko 2>/dev/null || echo "  insmod icm20608_iio.ko 失败"

IIO_DIR=""
for d in /sys/bus/iio/devices/iio:device*; do
  [ -e "$d/name" ] || continue
  [ "$(cat "$d/name" 2>/dev/null)" = "icm20608_iio" ] && { IIO_DIR="$d"; break; }
done

if [ -z "$IIO_DIR" ]; then
  echo "  FAIL: 未注册 icm20608_iio"
else
  AZ=$(cat "$IIO_DIR/in_accel_z_raw" 2>/dev/null || echo 0)
  TP=$(cat "$IIO_DIR/in_temp_raw"   2>/dev/null || echo 0)
  WHO=$(dmesg | grep -m1 "ICM20608 ID =" | sed 's/.*ID = //')
  echo "  $IIO_DIR  WHO_AM_I=${WHO:-?}  accel_z=$AZ  temp=$TP"
  if [ "$AZ" != "0" ] && [ "$TP" != "0" ]; then
    RC_ICM=0
    echo "  PASS: icm20608_iio 可读且非 0"
  else
    echo "  FAIL: raw 全 0（片选/SPI 问题）"
  fi
fi

# ---------------- 2. AP3216C Input 新链路 ----------------
echo ""
echo "--- [2/6] AP3216C -> Input ---"
echo "$DEV" > "$BSP_DRV/unbind" 2>/dev/null || true
/sbin/rmmod ap3216c_myself 2>/dev/null || true
/sbin/rmmod ap3216c_input 2>/dev/null || true
dmesg -c >/dev/null 2>&1 || true
# ps_measure_interval=0x01 -> ~437ms，缩短等待窗口
/sbin/insmod ./ap3216c_input.ko ps_measure_interval=0x01 2>/dev/null || \
  echo "  insmod ap3216c_input.ko 失败"

BOUND=0
[ -L "$NEW_I2C_DRV/$DEV" ] && BOUND=1
PS=$(cat "$CLIENT/proximity/ps_raw_data" 2>/dev/null || echo "")
ALS=$(cat "$CLIENT/ambient_light/als_raw_data" 2>/dev/null || echo "")
EVNODE=""
for ev in /sys/class/input/event*; do
  [ -e "$ev/device/name" ] || continue
  [ "$(cat "$ev/device/name" 2>/dev/null)" = "ap3216c-proximity" ] && { EVNODE="$ev"; break; }
done

echo "  bound=$BOUND  ps_raw=${PS:-?}  als_raw=${ALS:-?}  input=${EVNODE:-无}"
if [ "$BOUND" = "1" ] && [ -n "$PS" ] && [ -n "$ALS" ] && [ -n "$EVNODE" ]; then
  RC_AP=0
  echo "  PASS: 绑定 + sysfs raw + input 节点齐备"
else
  echo "  FAIL: 绑定/sysfs/input 节点缺失"
fi

# ---------------- 3. 人工动作（一次，判双向） ----------------
if [ "$WITH_HAND" = "1" ] && [ "$RC_AP" = "0" ]; then
  echo ""
  echo "--- [3/6] 人工动作：请手掌贴近再移开 AP3216C 传感器（${HAND_SECS}s 窗口）---"
  rm -f /tmp/sensors_ev.log /tmp/sensors_ps.log
  # 并行采样 PS：事件没来时用它区分"芯片没测到"还是"分类逻辑没生效"
  (
    i=0
    while [ "$i" -lt "$((HAND_SECS * 2))" ]; do
      cat "$CLIENT/proximity/ps_raw_data" 2>/dev/null >> /tmp/sensors_ps.log
      i=$((i + 1))
      sleep 0.5
    done
  ) &
  SAMPLER=$!
  # 参数 0 = 一直监听，由 timeout 收尾（写死事件数会在双向只来一半时提前退出）
  timeout "$HAND_SECS" ./ap3216c_input_app 0 > /tmp/sensors_ev.log 2>&1 &
  APP=$!
  wait $APP
  kill "$SAMPLER" 2>/dev/null || true
  wait "$SAMPLER" 2>/dev/null || true
  # 注意：grep -c 无匹配时输出 0 但退出码为 1，不能接 `|| echo 0`（会得到两行 "0"）
  NEAR_N=$(grep -c "value=1" /tmp/sensors_ev.log 2>/dev/null || true)
  FAR_N=$(grep -c "value=0" /tmp/sensors_ev.log 2>/dev/null || true)
  NEAR_N=${NEAR_N:-0}
  FAR_N=${FAR_N:-0}
  PS_MIN=$(sort -n /tmp/sensors_ps.log 2>/dev/null | head -1)
  PS_MAX=$(sort -n /tmp/sensors_ps.log 2>/dev/null | tail -1)
  echo "  窗口内 PS 采样: 最小=${PS_MIN:-?} 最大=${PS_MAX:-?}（阈值 $THR_LOW/$THR_HIGH）"
  echo "  EV_SW 事件: value=1(NEAR) x$NEAR_N, value=0(FAR) x$FAR_N"
  grep "type=5" /tmp/sensors_ev.log 2>/dev/null | head -6 | sed 's/^/    /'
  if [ "$NEAR_N" -gt 0 ] && [ "$FAR_N" -gt 0 ]; then
    RC_HAND=1
    echo "  PASS: 双向事件都收到"
  else
    RC_HAND=2
    if [ -n "$PS_MAX" ] && [ "$PS_MAX" -gt "$THR_HIGH" ]; then
      echo "  FAIL: PS 已越过阈值但没出事件 -> 分类/上报逻辑问题"
    else
      echo "  FAIL: 窗口内 PS 未越过阈值 -> 大概率是手没贴近（芯片侧正常）"
    fi
  fi
else
  echo ""
  echo "--- [3/6] 人工动作：跳过（WITH_HAND=$WITH_HAND）---"
fi

# ---------------- 4. A/B 对照 + 回退证据 ----------------
echo ""
echo "--- [4/6] A/B 对照 + 回退（旧模块能否重新装载并读到数）---"
AB_OK=0

# ICM20608：旧 icm20608.ko 字符设备
/sbin/rmmod icm20608_iio 2>/dev/null || true
dmesg -c >/dev/null 2>&1 || true
if /sbin/insmod ./icm20608.ko 2>/dev/null && [ -e /dev/icm20608 ]; then
  # 判据用重力向量模长 sqrt(ax^2+ay^2+az^2)，静止时恒 ≈2048 LSB（±16G 档 2048 LSB/g）。
  # 不用单轴比对：重力在某一轴上的投影随姿态变，单轴差值不是"驱动一致性"的度量；
  # 模长与姿态、轴序都无关，是这里唯一稳健的不变量。
  #
  # 取第 2 条而不是第 1 条：芯片复位后的第一次读是无效样本（实测第 1 次
  # ax=-262 ay=199 az=1107，第 2 次起才稳定在 ax=-32 ay=33 az=2038）。
  # 取第 1 条会拿到垃圾值，得出"两个驱动读数差 677 LSB"的假结论。
  OLD_LINES=$(timeout 3 ./icm20608_app /dev/icm20608 2>/dev/null | grep '^ax *=')
  OLD_LINE=$(echo "$OLD_LINES" | sed -n '2p')
  [ -z "$OLD_LINE" ] && OLD_LINE=$(echo "$OLD_LINES" | sed -n '1p')
  OLD_MAG=$(echo "$OLD_LINE" | awk -F'[=,]' '
      { printf "%d", sqrt($2*$2 + $4*$4 + $6*$6 + 0.5) }')
  echo "  旧 icm20608.ko  : /dev/icm20608 OK, ${OLD_LINE:-（无输出）}"
  echo "                     |a| = ${OLD_MAG:-?} LSB"
  /sbin/rmmod icm20608 2>/dev/null || true
  /sbin/insmod ./icm20608_iio.ko 2>/dev/null || true
  NEW_MAG=""
  for d in /sys/bus/iio/devices/iio:device*; do
    [ "$(cat "$d/name" 2>/dev/null)" = "icm20608_iio" ] || continue
    # tr 把三行并成一行，否则 awk 会按行跑三次、打出三个数
    NEW_MAG=$(cat "$d/in_accel_x_raw" "$d/in_accel_y_raw" "$d/in_accel_z_raw" \
              2>/dev/null | tr '\n' ' ' | \
              awk '{ printf "%d", sqrt($1*$1 + $2*$2 + $3*$3 + 0.5) }')
    break
  done
  echo "  新 icm20608_iio : |a| = ${NEW_MAG:-?} LSB"
  if [ -n "$OLD_MAG" ] && [ -n "$NEW_MAG" ] && [ "$OLD_MAG" -gt 0 ]; then
    DIFF=$((NEW_MAG - OLD_MAG))
    [ "$DIFF" -lt 0 ] && DIFF=$((-DIFF))
    # 静态 1g ≈ 2048；两条路径差 5%（约 100 LSB）以内即视为一致
    echo "  |a| 差 = $DIFF LSB（静止时两者都应 ≈2048，5% 内通过）"
    [ "$DIFF" -lt 100 ] && AB_OK=$((AB_OK + 1))
  else
    echo "  FAIL: 有一条路径没读到重力模长"
  fi
else
  echo "  FAIL: 回退失败，旧 icm20608.ko 装载或 /dev/icm20608 缺失"
fi

# AP3216C：旧 ap3216c_myself.ko 字符设备
/sbin/rmmod ap3216c_input 2>/dev/null || true
dmesg -c >/dev/null 2>&1 || true
if /sbin/insmod ./ap3216c_myself.ko 2>/dev/null; then
  # 同 ICM20608：取第 3 条（最后一次），避开复位后的首个无效样本。
  # 末尾必须 tr -d '\r'：旧 app 用 \r\n 换行，残留的 \r 会让下面 [ -lt ] 直接报错。
  OLD_PS=$(timeout 3 ./ap3216c_myself_app /dev/ap3216c_myself 3 2>/dev/null | \
           grep "ps =" | tail -1 | sed 's/.*ps *= *//' | tr -d '\r ')
  echo "  旧 ap3216c_myself: /dev/ap3216c_myself OK, ps=${OLD_PS:-?}"
  /sbin/rmmod ap3216c_myself 2>/dev/null || true
  /sbin/insmod ./ap3216c_input.ko ps_measure_interval=0x01 2>/dev/null || true
  NEW_PS=$(cat "$CLIENT/proximity/ps_raw_data" 2>/dev/null || echo "")
  echo "  新 ap3216c_input : ps_raw=${NEW_PS:-?}"
  # 判据：两条路径都读到数，且此刻无遮挡时都应接近 0（无遮挡 ~0，手掌贴近 ~580）
  case "$OLD_PS" in ''|*[!0-9]*) NUM_OK=0 ;; *) NUM_OK=1 ;; esac
  if [ "$NUM_OK" = "1" ] && [ -n "$NEW_PS" ] && \
     [ "$OLD_PS" -lt 40 ] && [ "$NEW_PS" -lt 40 ]; then
    echo "  两者都读到且都 ≈0（无遮挡），一致"
    AB_OK=$((AB_OK + 1))
  else
    echo "  FAIL: 旧/新 PS 不一致或读不到（old=${OLD_PS:-?} new=${NEW_PS:-?}）"
  fi
else
  echo "  FAIL: 回退失败，旧 ap3216c_myself.ko 装载失败"
fi

[ "$AB_OK" = "2" ] && RC_AB=0

# ---------------- 5. M2：runtime PM + sysfs 校准节点 ----------------
echo ""
echo "--- [5/6] M2: runtime PM + 校准节点 ---"

/sbin/rmmod ap3216c_input 2>/dev/null || true
dmesg -c >/dev/null 2>&1 || true
/sbin/insmod ./ap3216c_input.ko 2>/dev/null || echo "  insmod 失败"
sleep 1

PWR="$CLIENT/power/runtime_status"

# 完整走一遍 get/put 周期，而不是只看初始状态：
#   刚 probe 完 -> suspended（芯片只剩 PS）
#   读一次 ALS  -> 内部 get_sync 唤醒 -> active，随后 put_autosuspend 起 500ms 定时器
#   等 1s       -> 应回到 suspended
# 只测"初始是 suspended"证明不了 autosuspend 真的会跑，必须看这一整个来回。
STATUS_INIT=$(cat "$PWR" 2>/dev/null || echo "?")
ALS_SUSP=$(cat "$CLIENT/ambient_light/als_raw_data" 2>/dev/null || echo "")
STATUS_JUST=$(cat "$PWR" 2>/dev/null || echo "?")
sleep 1
STATUS_AFTER=$(cat "$PWR" 2>/dev/null || echo "?")
echo "  probe 后=$STATUS_INIT"
echo "  挂起态读 ALS=$ALS_SUSP（读时取 PM 引用，必须是有效值）"
echo "  读的瞬间=$STATUS_JUST  ->  1s 后=$STATUS_AFTER"

RC_PM=0
[ "$STATUS_INIT" = "suspended" ] || { echo "  FAIL: probe 后不是 suspended（=$STATUS_INIT）"; RC_PM=1; }
[ -n "$ALS_SUSP" ] && [ "$ALS_SUSP" != "0" ] || { echo "  FAIL: 挂起态读 ALS 失败/为 0（PM 引用没唤醒芯片）"; RC_PM=1; }
[ "$STATUS_JUST" = "active" ] || { echo "  FAIL: 读的瞬间未变 active（=$STATUS_JUST），PM 引用没生效"; RC_PM=1; }
[ "$STATUS_AFTER" = "suspended" ] || { echo "  FAIL: 1s 后未自动挂起（=$STATUS_AFTER），autosuspend 没跑"; RC_PM=1; }
[ "$RC_PM" = "0" ] && echo "  PASS: autosuspend 全程正确（suspended->active->suspended），挂起态 ALS 可读"

# 中断风暴回归：守住 ps_int_mode 的默认值。
# 2026-10-07 实测（空转 6s）：HYSTERESIS = 0 次中断；ZONE = 13 次（每 ~604ms 一次测量，
# 因 PS 恒在低阈值下方而每次都触发）。ZONE 下中断反过来把 autosuspend 打散，
# 于是 runtime_status 在 active/suspended 之间反复翻转、省不了电。
# 所以默认必须是 HYSTERESIS；这条断言就是防止它被改回去。
IRQ_N=$(awk '$3=="gpio-mxc" && $4=="1" && $5=="Edge" {gsub(":","",$1); print $1; exit}' \
        /proc/interrupts 2>/dev/null)
if [ -n "$IRQ_N" ]; then
  I0=$(awk -v n="$IRQ_N:" '$1==n {print $2; exit}' /proc/interrupts)
  sleep 3
  I1=$(awk -v n="$IRQ_N:" '$1==n {print $2; exit}' /proc/interrupts)
  NIRQ=$(( ${I1:-0} - ${I0:-0} ))
  echo "  空闲 3s 中断增量 = $NIRQ（ps_int_mode=1 期望 0）"
  [ "$NIRQ" -le 1 ] || { echo "  FAIL: 空闲期仍有中断，疑似退回 ZONE 模式"; RC_IRQ=1; }
else
  echo "  SKIP: 未在 /proc/interrupts 定位到 AP3216C 中断线"
fi

# 校准节点读写往返（改完立即恢复默认，不留状态）
RC_CAL=1
CAL=""
for pair in "threshold_low:30" "threshold_high:150" \
            "measure_interval:500" "led_current_ratio:2"; do
  node=${pair%%:*}
  val=${pair##*:}
  f="$CLIENT/proximity/$node"
  if [ ! -e "$f" ]; then
    CAL="$CAL $node=缺失"
    continue
  fi
  echo "$val" > "$f" 2>/dev/null || { CAL="$CAL $node=写入失败"; continue; }
  back=$(cat "$f" 2>/dev/null)
  CAL="$CAL $node:$val->$back"
done
echo "  校准节点往返:$CAL"
# 期望：measure_interval 500ms 会被折算到最近的 wait 值（437ms）
echo "$CAL" | grep -q "threshold_low:30->30" && \
echo "$CAL" | grep -q "threshold_high:150->150" && \
echo "$CAL" | grep -q "measure_interval:500->437" && \
echo "$CAL" | grep -q "led_current_ratio:2->2" && RC_CAL=0

if [ "$RC_CAL" = "0" ]; then
  echo "  PASS: 4 个校准节点读写往返正确"
else
  echo "  FAIL: 校准节点往返不正确"
fi

# 恢复默认阈值/间隔，避免影响后续
echo 20  > "$CLIENT/proximity/threshold_low"   2>/dev/null || true
echo 80  > "$CLIENT/proximity/threshold_high"  2>/dev/null || true
echo 1112 > "$CLIENT/proximity/measure_interval" 2>/dev/null || true
echo 1   > "$CLIENT/proximity/led_current_ratio" 2>/dev/null || true

# ---------------- 6. 恢复板子原状 ----------------
echo ""
echo "--- [6/6] 恢复 ---"
/sbin/rmmod ap3216c_input 2>/dev/null || true
/sbin/rmmod icm20608_iio 2>/dev/null || true
echo "$DEV" > "$BSP_DRV/bind" 2>/dev/null || true
echo "  已卸载新模块并回绑 BSP ap3216c"

# ---------------- 汇总 ----------------
echo ""
echo "==================== 汇总 ===================="
[ "$RC_ICM" = "0" ] && echo "  ICM20608 -> IIO      : PASS" || echo "  ICM20608 -> IIO      : FAIL"
[ "$RC_AP"  = "0" ] && echo "  AP3216C  -> Input    : PASS" || echo "  AP3216C  -> Input    : FAIL"
[ "$AB_OK"  = "2" ] && echo "  A/B 对照 + 回退      : PASS" || echo "  A/B 对照 + 回退      : FAIL"
case "$RC_HAND" in
  1) echo "  EV_SW 人工双向事件   : PASS" ;;
  2) echo "  EV_SW 人工双向事件   : FAIL" ;;
  *) echo "  EV_SW 人工双向事件   : NOT RUN" ;;
esac
[ "$RC_PM"  = "0" ] && echo "  M2 runtime PM        : PASS" || echo "  M2 runtime PM        : FAIL"
[ "$RC_CAL" = "0" ] && echo "  M2 sysfs 校准节点    : PASS" || echo "  M2 sysfs 校准节点    : FAIL"
[ "$RC_IRQ" = "0" ] && echo "  M2 空闲无中断风暴    : PASS" || echo "  M2 空闲无中断风暴    : FAIL"

if [ "$RC_ICM" = "0" ] && [ "$RC_AP" = "0" ] && [ "$AB_OK" = "2" ] && \
   [ "$RC_HAND" != "2" ] && [ "$RC_PM" = "0" ] && [ "$RC_CAL" = "0" ] && \
   [ "$RC_IRQ" = "0" ]; then
  echo "== 总判定: PASS =="
  exit 0
fi
echo "== 总判定: FAIL =="
exit 1
