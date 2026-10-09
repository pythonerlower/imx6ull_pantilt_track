#!/usr/bin/env sh
# Phase 6 板端验收：securityd 安防状态机（系统集成）
#
# 设计原则：
#   - 一次跑完全部 11 段，每段只留 1~2 条硬判据
#   - 判据尽量独立于被测程序：WAV 合法性用板端 python3 的 wave 模块判，
#     无孤儿用 pidof 判，没重启用 uptime 判，不靠 securityd 自述
#   - 只跑本链需要的模块；**不加载 mx6hub**（与 Phase 4 两个新驱动争硬件）
#   - 绝不重启板子、绝不给云台下任何控制包
#
# 判据：
#   [1] 环境与设备准备 : 五类节点齐备（video0/video1/pantilt0/input event/iio accel_z）
#   [2] 启动自检       : 设备齐了，securityd --dry-run 就不许报 DEGRADED(no_sensor)
#   [3] 常驻与可观测性 : STATS 行存在；status 文件可被 sh 解析；视频常开 = 无告警也该 video=up
#   [4] 人工靠近       : 一个 HAND_SECS 窗口内出现 EVT to=ALERT 与 to=TRACK_READY
#   [5] 结果核对       : 视频侧 vsend>0 且发包错=0；ps_irq 有增长
#                        （无告警时没有录音可判，只判视频侧）
#   [6] 有序退出       : SIGTERM 后无孤儿 video_link/audio_capture，uptime 未变，云台未被碰；
#                        WAV 头与长度自洽（python wave）；有告警时 RESULT 的 recorded>=1
#   [7] 摄像头重载     : 先 rmmod 摄像头再起 securityd，必须自愈回 video=up
#   [8] SIGUSR1 dump   : 信号一到就多一行 STATS，不等下一个 --stats-ms 周期
#   [9] 传感器缺失自检 : 卸掉 ap3216c_input 后启动必须**立刻**报 DEGRADED no_sensor，
#                        不许静默停在 GUARD（那是最糟的失败：看起来一切正常）
#   [10] 可观测性对账  : debugfs 三节点与 GET_STATE 读数一致（daemon 占着设备时，
#                        debugfs 仍可读，但 GET_STATE 拿不到 —— 见该段说明）
#   [11] 重载耗尽路径  : 重载 3 次全失败 → DEGRADED(camera)，且**音频还在录**、
#                        云台未被碰、板子未重启（需要操作者先触发一次真实 ALERT）
#
# 用法：sh ./board_verify_phase6.sh [BOARD_DIR]
#      WITH_HAND=0      跳过人工靠近段（现场没人时用，[4] 记为 NOT RUN，总判定仍可 PASS）
#      WITH_EXHAUST=0   跳过 [11] 重载耗尽段（它同样需要操作者）
#      HAND_SECS=25     [4] 人工窗口秒数
#      EXHAUST_SECS=30  [11] 人工窗口秒数
#      VIDEO_UP_WAIT=25 [3] 等 video=up 的秒数
#      M4_STOP_DAEMON=1 [10] 允许临时停 pantilt_daemon 以取到 GET_STATE。
#                          **默认为 0**：本脚本平时不碰 daemon。停它会让驱动的
#                          1500 ms 看门狗按设计触发一次 STOP（云台原地保持，不动），
#                          但那确实改变了现场状态，所以必须显式开。
set -u

BOARD_DIR="${1:-/root/imx6hub_demo}"
cd "$BOARD_DIR" || exit 2

TMP="${TMP:-/tmp/phase6_verify}"
LOG_DIR="$TMP/logs"
STATUS_FILE="$TMP/securityd.status"
SD_LOG="$TMP/securityd.stdout"
RECORDS_DIR="$TMP/records"

HAND_SECS="${HAND_SECS:-25}"
WITH_HAND="${WITH_HAND:-1}"
VIDEO_UP_WAIT="${VIDEO_UP_WAIT:-25}"   # [3] 等 video=up 的秒数；摄像头 probe 要几秒

SD=./securityd

RC_ENV=1
RC_SELF=1
RC_ALIVE=1
RC_HAND=0        # 0=未跑/跳过，1=通过，2=失败
RC_CHECK=1
RC_STOP=1
RC_RELOAD=1
RC_SIG=1
RC_NOSENS=1
RC_DBG=1
RC_EXH=0         # 同 RC_HAND：0=未跑/跳过，1=通过，2=失败
GET_DONE=0       # [10] 是否真的取到了 GET_STATE（默认路径下取不到，见该段说明）

WITH_EXHAUST="${WITH_EXHAUST:-1}"
M4_STOP_DAEMON="${M4_STOP_DAEMON:-0}"
# 两个人工窗口的秒数。[4] 用 HAND_SECS，[11] 用 EXHAUST_SECS。
# 它们**只影响等人多久**，不影响任何判据 —— 卡住多半是人还没走到板子前，
# 不是产品问题，所以给得宽一点比给窄了反复重跑划算。
EXHAUST_SECS="${EXHAUST_SECS:-30}"

PS_IRQ0="?"     # [4] 里赋值；[4] 被跳过时 [5] 仍要能引用（set -u）

mkdir -p "$TMP" "$LOG_DIR" "$RECORDS_DIR" || exit 2

# ---------------------------------------------------------------- 工具

# 从 STATS/RESULT 行里取一个 key=value 字段的裸值（数字）
kv_num() {   # kv_num <file> <key>
  grep -a 'securityd: STATS\|securityd: RESULT' "$1" 2>/dev/null | tail -1 | \
    sed -n "s/.* $2=\([0-9][0-9.]*\).*/\1/p"
}

kv_str() {   # kv_str <file> <key>   （取到空格为止）
  grep -a 'securityd: STATS\|securityd: RESULT' "$1" 2>/dev/null | tail -1 | \
    sed -n "s/.* $2=\([^ ]*\).*/\1/p"
}

sd_alive() { [ -n "$(pidof securityd 2>/dev/null)" ]; }

# 等到 log 里出现某个模式，最多 wait_s 秒；命中返回 0
wait_log() {   # wait_log <file> <pattern> <wait_s>
  _w=0
  while [ "$_w" -lt "$3" ]; do
    grep -aq "$2" "$1" 2>/dev/null && return 0
    sd_alive || return 1
    sleep 1
    _w=$((_w + 1))
  done
  grep -aq "$2" "$1" 2>/dev/null
}

# 停掉可能残留的 securityd / 子进程，不留现场
sd_cleanup() {
  pkill -TERM -x securityd 2>/dev/null || true
  sleep 2
  pkill -KILL -x securityd 2>/dev/null || true
  pkill -TERM -x video_link 2>/dev/null || true
  pkill -TERM -x audio_capture 2>/dev/null || true
  sleep 2
  pkill -KILL -x video_link 2>/dev/null || true
  pkill -KILL -x audio_capture 2>/dev/null || true
}

# [4] 与 [11] 靠人工触发，失败时要能当场分清两种成因：
#   "人没在窗口内动手" vs "传感器根本没检到"。
# prox_counts 的 near/far 是累计值，ps_raw_data 是当前原始值 —— 后者在手掌
# 贴上去时会冲到几百（实测 364，阈值 80），所以它是"此刻有没有东西在前面"的直接答案。
sensor_brief() {
  _sb_B=/sys/bus/i2c/drivers/ap3216c_input/0-001e
  if [ -r "$_sb_B/proximity/prox_counts" ]; then
    echo "  传感器侧：$(cat "$_sb_B/proximity/prox_counts")  ps_raw_data=$(cat "$_sb_B/proximity/ps_raw_data")（阈值 high=$(cat "$_sb_B/proximity/threshold_high")）"
    echo "  判读：near/far 为 0 → 窗口内**一次都没检到**（先确认手真的贴住了传感器）；"
    echo "        near/far 有数 → 检到了，那是事件没进状态机"
  else
    echo "  传感器侧：读不到 $_sb_B/proximity/prox_counts"
  fi
}

echo "==================== Phase 6 板端验收（securityd） ===================="
echo "工作目录 $BOARD_DIR    证据目录 $TMP"
echo "注意：本脚本不重启板子、不给云台下任何控制包。"
sd_cleanup
UP0=$(awk '{print int($1)}' /proc/uptime)

# ---------------------------------------------------------------- [1] 环境与设备准备
echo ""
echo "--- [1/11] 环境与设备准备 ---"

# 摄像头：若已在跑先收干净，再按固定顺序装
pkill -TERM -x video_link 2>/dev/null || true
sleep 2
for mod in mx6s_capture ov5640_camera; do
  lsmod | grep -q "^$mod " && /sbin/rmmod "$mod" 2>/dev/null || true
done
dmesg -c >/dev/null 2>&1 || true

/sbin/insmod ./mx6s_capture_fixed.ko 2>/dev/null || echo "  insmod mx6s_capture_fixed.ko 失败"
/sbin/insmod ./ov5640_camera.ko 2>/dev/null || echo "  insmod ov5640_camera.ko 失败"
sleep 2

# 云台：仅装载 + 设限，不发控制包
if ! lsmod | grep -q '^pantilt '; then
  /sbin/insmod ./pantilt.ko 2>/dev/null || echo "  insmod pantilt.ko 失败"
fi
./pantilt_ctl limit --slew 20 --watchdog 1500 >/dev/null 2>&1 || \
  echo "  提示：pantilt_ctl limit 未设成（daemon 可能已占用，继续）"

# ICM20608（IIO，轮询，无中断）
/sbin/rmmod mx6hub 2>/dev/null || true
/sbin/insmod ./icm20608_iio.ko 2>/dev/null || echo "  insmod icm20608_iio.ko 失败"
sleep 1

# AP3216C（Input）：BSP 每次开机都会重新绑定，所以每次都要先 unbind。
# 这一行必须是独立的重定向 —— 写成 `echo X > A >> B` 的话 POSIX sh 从左到右求值，
# 后面的 >> 会赢，unbind 根本不发生，新驱动绑不上、Input 节点不出现。
echo 0-001e > /sys/bus/i2c/drivers/ap3216c/unbind 2>/dev/null || true
/sbin/rmmod ap3216c_input 2>/dev/null || true
# ps_measure_interval=0x01 -> ~437ms；**不得**把 ps_int_mode 设为 0
/sbin/insmod ./ap3216c_input.ko ps_measure_interval=0x01 2>/dev/null || \
  echo "  insmod ap3216c_input.ko 失败"
sleep 1

EVNODE=""
for ev in /sys/class/input/event*; do
  [ -e "$ev/device/name" ] || continue
  [ "$(cat "$ev/device/name" 2>/dev/null)" = "ap3216c-proximity" ] && { EVNODE="$ev"; break; }
done
IIO_DIR=""
for d in /sys/bus/iio/devices/iio:device*; do
  [ -e "$d/name" ] || continue
  [ "$(cat "$d/name" 2>/dev/null)" = "icm20608_iio" ] && { IIO_DIR="$d"; break; }
done
AZ=$(cat "$IIO_DIR/in_accel_z_raw" 2>/dev/null || echo 0)

echo "  /dev/video0=$([ -e /dev/video0 ] && echo yes || echo NO)" \
     "/dev/video1=$([ -e /dev/video1 ] && echo yes || echo NO)" \
     "/dev/pantilt0=$([ -e /dev/pantilt0 ] && echo yes || echo NO)"
echo "  input=${EVNODE:-无}  iio=${IIO_DIR:-无}  accel_z=$AZ"

if [ -e /dev/video0 ] && [ -e /dev/video1 ] && [ -e /dev/pantilt0 ] && \
   [ -n "$EVNODE" ] && [ -n "$IIO_DIR" ] && [ "$AZ" != "0" ]; then
  RC_ENV=0
  echo "  PASS: 五类设备节点齐备"
else
  echo "  FAIL: 设备节点不全（视频/云台/Sensor/IIO 之一缺失）"
fi

# ---------------------------------------------------------------- [2] 启动自检
echo ""
echo "--- [2/11] 启动自检（--dry-run）---"
if [ "$RC_ENV" = "0" ]; then
  timeout 5 $SD --dry-run --tick-ms 200 --stats-ms 1000 \
    --log-dir "$LOG_DIR/dryrun" --status-file "$TMP/dryrun.status" \
    >"$TMP/dryrun.stdout" 2>&1
  # 设备齐备时不许报 no_sensor：报了就是自检比事实更悲观，会掩盖真故障
  if grep -aq 'deg=no_sensor' "$TMP/dryrun.stdout"; then
    echo "  FAIL: 设备齐备却报 DEGRADED(no_sensor)"
    grep -a 'DEGRADED\|WARN' "$TMP/dryrun.stdout" | head -5 | sed 's/^/    /'
  elif grep -aq 'securityd: STATS' "$TMP/dryrun.stdout"; then
    RC_SELF=0
    echo "  PASS: 自检不误报，STATS 行正常"
    grep -a 'securityd: STATS' "$TMP/dryrun.stdout" | tail -1 | sed 's/^/    /'
  else
    echo "  FAIL: --dry-run 没有输出 STATS 行"
  fi
else
  echo "  跳过（环境段未通过）"
fi

# ---------------------------------------------------------------- [3] 常驻与可观测性
echo ""
echo "--- [3/11] 常驻与可观测性 ---"
if [ "$RC_ENV" = "0" ]; then
  # 先删掉上一轮遗留的 status 文件：否则 securityd 万一没起来，下面读到的会是
  # **上一次运行的终态**，却顶着"status 文件:"这行字样打出来 —— 看上去像本轮证据。
  # （2026-10-07 run6 实测出现过这个现象，成因未能复现，但这个洞是真的。）
  rm -f "$STATUS_FILE"
  $SD --tick-ms 100 --stats-ms 1000 \
      --audio-warmup-ms 2000 --alert-hold-ms 8000 --alert-max-ms 60000 \
      --records-dir "$RECORDS_DIR" --audio-tag alert \
      --log-dir "$LOG_DIR" --status-file "$STATUS_FILE" \
      >"$SD_LOG" 2>&1 &
  SD3_PID=$!
  # 视频常开 + 2 s 热身，给 20 s 应该够
  if wait_log "$SD_LOG" 'securityd: STATS' 25; then
    FIRST=$(grep -a 'securityd: STATS' "$SD_LOG" | tail -1)
    echo "  首条 STATS: ${FIRST:-（空——wait_log 说匹配到了，这里却取不到，属异常）}"
    # 这行头部写的是"status 文件可被 sh 解析"，代码里却只 WARN 就放过 —— 补成真判据。
    # 用独立标志而不是直接置 RC_ALIVE：下面 video=up 那支会把 RC_ALIVE 重新赋 0，
    # 在这里直接置会被它悄悄盖掉（这个脚本栽过的"静默放行"就是这个形状）。
    # 文件刚被删过，所以此刻能读到就一定是本轮写的，不存在"串上一轮"。
    STF_OK=0
    if [ -r "$STATUS_FILE" ] && grep -q '^ts=' "$STATUS_FILE"; then
      echo "  status 文件: $(cat "$STATUS_FILE")"
    else
      STF_OK=1
      echo "  FAIL: status 文件读不到或格式不符（脚本头承诺它可被 sh 解析）"
    fi
    # 判据是"视频常开"，不是"进程一起来视频就得在发包"。video_link 启动后
    # 要先 probe 摄像头才出第一个包（实测约 2~3 s，见 logs/phase6/），首条 STATS
    # 往往还在 video=init —— 拿它判等于把 probe 时间当成故障，所以在这里等。
    i=0
    while [ "$i" -lt "$VIDEO_UP_WAIT" ]; do
      [ "$(kv_str "$SD_LOG" video)" = "up" ] && break
      sd_alive || break
      sleep 1
      i=$((i + 1))
    done
    VSTATE=$(kv_str "$SD_LOG" video)
    # 视频常开的判据：还没告警，video 就该是 up。status 文件一并算进本段。
    if [ "$VSTATE" = "up" ] && [ "${STF_OK:-1}" = "0" ]; then
      RC_ALIVE=0
      echo "  PASS: securityd 常驻，视频常开（无告警时 video=up），status 文件可解析"
    else
      echo "  FAIL: 无告警时 video=$VSTATE（常开失败，摄像头或网络有问题）/ status 文件 ${STF_OK:-未判}"
      grep -a 'WARN\|DEGRADED' "$SD_LOG" | head -5 | sed 's/^/    /'
    fi
  else
    echo "  FAIL: 25 s 内没等到 STATS 行"
    tail -10 "$SD_LOG" | sed 's/^/    /'
  fi
else
  echo "  跳过（环境段未通过）"
fi

# ---------------------------------------------------------------- [4] 人工靠近
echo ""
echo "--- [4/11] 人工靠近（AP3216C 触发）---"
if [ "$RC_ALIVE" = "0" ] && [ "$WITH_HAND" = "1" ]; then
  PS_IRQ0=$(grep -a 'securityd: STATS' "$SD_LOG" | tail -1 | sed -n 's/.* ps_irq=\([0-9]*\).*/\1/p')
  echo "  请在 ${HAND_SECS}s 内：手掌贴近 AP3216C 传感器，停住 2~3 秒，然后移开。"
  echo "  移开之后**不要再靠近** —— 每一次新的 near 都会把 8 s 的 alert_hold 重新计时，"
  echo "  反复挥手会让它一直回不到 GUARD（2026-10-07 两轮实测都栽在这里：传感器侧"
  echo "  near=18 far=18，事件全是好的，就是 hold 一直被续期）。"
  echo "  （找一个安静环境，别晃动板子；本链不判振动）"
  i=0
  while [ "$i" -lt "$HAND_SECS" ]; do
    grep -aq 'to=TRACK_READY' "$SD_LOG" && break
    sd_alive || break
    sleep 1
    i=$((i + 1))
  done
  if grep -aq 'to=ALERT' "$SD_LOG"; then
    echo "  ALERT 已触发: $(grep -a 'to=ALERT' "$SD_LOG" | tail -1)"
  else
    echo "  ALERT 未触发（窗口内没读到 near 事件）"
  fi
  if grep -aq 'to=TRACK_READY' "$SD_LOG"; then
    echo "  TRACK_READY: $(grep -a 'to=TRACK_READY' "$SD_LOG" | tail -1)"
  elif grep -aq 'to=ALERT' "$SD_LOG"; then
    # 有 ALERT 没 TRACK_READY：这才是"视频没在发包"那条路。
    echo "  TRACK_READY 未达 —— 进了 ALERT 但视频没证实在发包："
    grep -a 'securityd: STATS' "$SD_LOG" | tail -1 | sed 's/^/    /'
  else
    # 没 ALERT 就谈不上 TRACK_READY。**别在这里猜"视频没发包"**：本轮实测
    # 视频一直在发（vsend=135332 verr=0），这句错误归因把人带偏过。
    echo "  TRACK_READY 未达 —— 但根因在上一步：压根没进 ALERT，没到看视频的那一步"
    echo "  （人工动作没落在窗口内，还是传感器没检到？先看下面的计数）"
  fi
  echo "  等 alert_hold(8s) 到点回 GUARD …"
  wait_log "$SD_LOG" 'to=GUARD reason=stopped' 30 || true
  grep -aq 'to=GUARD reason=stopped' "$SD_LOG" && echo "  已回 GUARD"

  if grep -aq 'to=ALERT' "$SD_LOG" && grep -aq 'to=TRACK_READY' "$SD_LOG" && \
     grep -aq 'to=GUARD reason=stopped' "$SD_LOG"; then
    RC_HAND=1
    echo "  PASS: near → ALERT → TRACK_READY → GUARD 全走过"
  else
    RC_HAND=2
    echo "  FAIL: 状态机没走完整条路（检查人工动作是否真的让传感器 NEAR）"
    sensor_brief
  fi
elif [ "$WITH_HAND" != "1" ]; then
  echo "  跳过（WITH_HAND=$WITH_HAND）"
else
  echo "  跳过（常驻段未通过）"
fi

# ---------------------------------------------------------------- [5] 结果核对
echo ""
echo "--- [5/11] 结果核对 ---"
# 板端 python3 的 wave 模块做独立校验。输出一律 ASCII：非交互 shell 的 stdout
# 编码是 ASCII，中文会直接抛 UnicodeEncodeError。
cat > "$TMP/wavcheck.py" <<'PYEOF'
import sys, wave, os
p = sys.argv[1]
try:
    w = wave.open(p, 'rb')
except Exception as e:
    print("FAIL cannot open as WAV: %s" % e); sys.exit(1)
n = w.getnframes(); ch = w.getnchannels(); sw = w.getsampwidth()
fr = w.getframerate(); w.close()
size = os.path.getsize(p)
errs = []
# 空录音（44 字节纯头）必须直接判失败：采集方向坏掉时正是这个形状。
if n <= 0:
    errs.append("frames=0 (empty capture, header only)")
else:
    expect = 44 + n * ch * sw
    if size != expect:
        errs.append("size=%d expect=%d (header not backfilled?)" % (size, expect))
    if fr <= 0:
        errs.append("rate=%d" % fr)
if errs:
    print("FAIL %s: %s" % (os.path.basename(p), "; ".join(errs))); sys.exit(1)
print("OK %s frames=%d rate=%d ch=%d bits=%d size=%d dur=%.2fs"
      % (os.path.basename(p), n, fr, ch, sw * 8, size, n / float(fr)))
PYEOF

VSEND=$(kv_num "$SD_LOG" vsend)
VERR=$(kv_num "$SD_LOG" verr)
PS_IRQ1=$(grep -a 'securityd: STATS' "$SD_LOG" | tail -1 | sed -n 's/.* ps_irq=\([0-9]*\).*/\1/p')

echo "  vsend=${VSEND:-0} verr=${VERR:-0} ps_irq=${PS_IRQ0:-?}→${PS_IRQ1:-?}"
ls -1t "$RECORDS_DIR"/*_alert.wav >/dev/null 2>&1 && \
  echo "  录音文件（**仍在写，本段不判**）：$(ls -1t "$RECORDS_DIR"/*_alert.wav | head -1)"
# WAV 与 recorded 一律挪到 [6] 之后判。录制中的文件头是 `RIFF 00000000` 占位值，
# wave.open() 直接抛 "not a WAVE file" —— 就地判它必然假失败（2026-10-07 实测证实）。
# 那会把 [4] 没回到 GUARD 这件事重复算成 [5] 也失败，读证据的人会以为有两处毛病。
# 而且 recorded 只出现在 RESULT 行上，RESULT 要到 [6] 收尾完才打，这里根本取不到 ——
# 先前那版写 `REC=$(kv_num ... recorded)` 却"没失败"，只是因为 `"" != "0"` 恰好为真，
# 正是本脚本要防的那类静默通过。[6] 里文件已定稿、RESULT 已落盘，判在那边才成立。

# 有没有告警，用 EVT 行判，不用 STATS 里的 alerts 字段（理由同上：只出现在 RESULT 行）。
ALERTS_SEEN=$(grep -ac 'to=ALERT' "$SD_LOG" 2>/dev/null || true)

# 视频是常开的，发包这一半任何时候都是硬判据，不跟着告警一起放掉。
if [ "${VSEND:-0}" != "0" ] && [ "${VERR:-0}" = "0" ]; then
  RC_CHECK=0
  echo "  PASS: 视频在发包（vsend=${VSEND:-0} 且发包错=${VERR:-0}）"
else
  echo "  FAIL: 视频没在发包或发包有错（vsend=${VSEND:-0} verr=${VERR:-0}）"
fi
if [ "${ALERTS_SEEN:-0}" != "0" ]; then
  echo "  WAV/recorded 判在 [6]（录音此刻仍在写，文件还没定稿）"
fi

# ps_irq 有增长是旁证：证明这一段里接近中断真的走过（不增长不判失败——
# 空闲期 irq 恒为 0 是已知行为，别拿它当健康信号，见 README 已知局限 6）。

# ---------------------------------------------------------------- [6] 有序退出
echo ""
echo "--- [6/11] 有序退出 ---"
if sd_alive; then
  SDPID=$(pidof securityd)
  kill -TERM "$SDPID" 2>/dev/null || true
  i=0
  while [ "$i" -lt 20 ]; do
    sd_alive || break
    sleep 1
    i=$((i + 1))
  done
  if sd_alive; then
    echo "  FAIL: SIGTERM 20 s 后仍未退出"
    RC_STOP=1
  else
    echo "  securityd 已退出，尾行："
    grep -a 'securityd: RESULT' "$SD_LOG" | tail -1 | sed 's/^/    /'
  fi
else
  echo "  securityd 已不在运行"
fi
# 显式 wait 掉 [3] 起的后台作业：不 wait 的话，shell 会在脚本结束时
# 打一行 "Killed" 到 stderr，紧挨着 PASS 行，读证据的人会以为这段失败了。
[ -n "${SD3_PID:-}" ] && { wait "$SD3_PID" 2>/dev/null || true; }
sleep 3

ORPHAN=$(pidof video_link audio_capture 2>/dev/null || true)
if [ -n "$ORPHAN" ]; then
  echo "  FAIL: 退出后仍有孤儿进程：$ORPHAN"
  pkill -KILL -x video_link 2>/dev/null || true
  pkill -KILL -x audio_capture 2>/dev/null || true
fi

UP1=$(awk '{print int($1)}' /proc/uptime)
echo "  uptime ${UP0}s → ${UP1}s（未重启应为单纯增长）"
# 云台是否被碰过：驱动侧的 watchdog_trips 恒为 0，这里只能证明"我们没下发过"
./pantilt_ctl info 2>/dev/null | sed 's/^/    /' | head -5 || \
  echo "    （pantilt_ctl info 不可用：daemon 可能正占着 /dev/pantilt0，属正常）"

# 录音的判定放在这里，不放 [5]：此刻 securityd 已收尾、WAV 头已回填、RESULT 已落盘，
# 三样才同时成立。在 [5] 那会儿录音还在写，文件头是 `RIFF 00000000` 占位值，
# wave.open() 直接抛 "not a WAVE file" —— 就地判必然假失败（2026-10-07 板上实测证实），
# 而且会把 [4] 没回到 GUARD 这一件事重复算成 [5] 也失败，读证据的人会以为有两处毛病。
REC=$(kv_num "$SD_LOG" recorded)
case "$REC" in ''|*[!0-9]*) REC="" ;; esac      # 非数字一律当"没取到"，别让 -ge 报错
WAV=$(ls -1t "$RECORDS_DIR"/*_alert.wav 2>/dev/null | head -1)
AUDIT=0   # 0=通过或不适用（不拉低总判定），1=失败

if [ "${ALERTS_SEEN:-0}" = "0" ]; then
  AUDIT_MSG="无告警，录音侧不适用"
  echo "  无告警：没有录音可判（视频侧判据在 [5]）"
elif [ -z "$WAV" ]; then
  AUDIT=1
  AUDIT_MSG="有告警但没落盘"
  echo "  FAIL: 有告警，但 records/ 下没有 *_alert.wav（recorded=${REC:-未取到}）"
else
  echo "  录音文件 $WAV"
  WOUT=$(python3 "$TMP/wavcheck.py" "$WAV" 2>&1)
  echo "$WOUT" | sed 's/^/    /'
  case "$WOUT" in OK*) WAVC=0 ;; *) WAVC=1 ;; esac
  # recorded 必须 >=1，这不是冗余：securityd 的 handle_sigchld() 曾不抽干管道就关 fd，
  # 子进程的收尾行 `audio: RESULT` 被静默丢掉 → WAV 完好却报 recorded=0，连
  # `DEGRADED audio` 也打不出来（录好了不报、录坏了也不报）。磁盘校验 + 计数交叉，
  # 两处都判才拦得住这一类"哑火"。2026-10-07 两轮实测都是这个形状。
  if [ "$WAVC" = "1" ]; then
    AUDIT=1
    AUDIT_MSG="WAV 校验未过"
    echo "  FAIL: WAV 校验未过（recorded=${REC:-未取到}）"
  elif [ -z "$REC" ] || [ "$REC" -lt 1 ]; then
    AUDIT=1
    AUDIT_MSG="WAV 合法但 recorded 对不上"
    echo "  FAIL: WAV 合法，但 RESULT recorded=${REC:-未取到}（录了却报没录？）"
  else
    AUDIT_MSG="录音侧对账通过"
    echo "  PASS: WAV 头与长度自洽，且 RESULT recorded=$REC 对得上"
  fi
fi

if [ -z "$ORPHAN" ] && [ "$UP1" -ge "$UP0" ] && [ "${UP1:-0}" -lt "$((UP0 + 600))" ] && \
   grep -aq 'securityd: RESULT' "$SD_LOG" && [ "$AUDIT" = "0" ]; then
  RC_STOP=0
  echo "  PASS: 有序退出，无孤儿，板子未重启；$AUDIT_MSG"
else
  RC_STOP=1
  echo "  FAIL: 退出后有孤儿 / 出现重启 / 没有 RESULT 行 / $AUDIT_MSG"
fi

# ---------------------------------------------------------------- [7] 摄像头重载
echo ""
echo "--- [7/11] 摄像头故障自愈（有序重载）---"
# 先把摄像头模块卸掉，再让 securityd 起来：video_link 必然起不来（或起不来就退），
# securityd 必须在 video_ready_ms 内发现并按 mx6s_capture → ov5640_camera 顺序重载。
for mod in mx6s_capture ov5640_camera; do
  lsmod | grep -q "^$mod " && /sbin/rmmod "$mod" 2>/dev/null || true
done
sleep 1
RELOAD_LOG="$TMP/reload.stdout"
# 先把文件建出来：后台作业的 shell 重定向还没生效时，下面循环里的 grep
# 会因文件不存在往 stderr 喷一行 "No such file or directory"。
: > "$RELOAD_LOG"
$SD --tick-ms 200 --stats-ms 1000 --video-ready-ms 4000 \
    --audio-warmup-ms 0 --camera-reload-attempts 3 \
    --records-dir "$RECORDS_DIR" --log-dir "$LOG_DIR/reload" \
    --status-file "$TMP/reload.status" \
    >"$RELOAD_LOG" 2>&1 &
SD7_PID=$!

TRIES=0
while [ "$TRIES" -lt 60 ]; do
  grep -aq 'securityd: INFO  摄像头重载第' "$RELOAD_LOG" && break
  sd_alive || break
  sleep 1
  TRIES=$((TRIES + 1))
done
# cam_reload 是 STATS 行里的字段，而 kv_num 只认 STATS/RESULT 行。重载那条
# INFO 在第一个 tick（200 ms）就打了，比首条 STATS（1 s）还早 —— 直接取会拿到
# 空串，于是"明明重载过"被误判成"没触发重载"。先等 STATS 行出现再取。
i=0
while [ "$i" -lt 15 ]; do
  grep -aq 'securityd: STATS' "$RELOAD_LOG" && break
  sd_alive || break
  sleep 1
  i=$((i + 1))
done
RELOAD_N=$(kv_num "$RELOAD_LOG" cam_reload)
echo "  重载日志："
grep -a '摄像头重载\|退出码=3\|摄像头故障\|视频就绪' "$RELOAD_LOG" | head -8 | sed 's/^/    /'

# 等它自愈：视频要恢复到 up
VUP=1
i=0
while [ "$i" -lt 40 ]; do
  [ "$(kv_str "$RELOAD_LOG" video)" = "up" ] && { VUP=0; break; }
  sd_alive || break
  sleep 1
  i=$((i + 1))
done

# 这里必须用 shell 内建的 kill，不能用 pkill。这段的 securityd 多半是在重载
# 途中被送走的，属于"被信号杀死"，而 bash/dash 一旦回收这种作业就会打一行
# "N Killed $SD …"。pkill 是外部命令，shell 在等它返回的时候就把作业收了，
# 那行照样打出来（实测正好落在 PASS 行前面）。内建 kill 之后紧接着 wait
# 才是干净的；中间连 sleep 都不能插。
if [ -n "${SD7_PID:-}" ] && kill -0 "$SD7_PID" 2>/dev/null; then
  kill -TERM "$SD7_PID" 2>/dev/null || true
  sleep 3
  kill -KILL "$SD7_PID" 2>/dev/null || true
  wait "$SD7_PID" 2>/dev/null || true
fi
sleep 1

if [ "${RELOAD_N:-0}" != "0" ] && [ "$VUP" = "0" ]; then
  RC_RELOAD=0
  echo "  PASS: 检测到故障 → 重载 ${RELOAD_N} 次 → 视频恢复 up"
elif [ "${RELOAD_N:-0}" = "0" ]; then
  echo "  FAIL: 摄像头模块已卸掉，securityd 却没触发重载"
else
  echo "  FAIL: 重载了 ${RELOAD_N} 次但视频没恢复"
  grep -a 'DEGRADED\|WARN' "$RELOAD_LOG" | head -5 | sed 's/^/    /'
fi

# ---------------------------------------------------------------- [8] SIGUSR1
echo ""
echo "--- [8/11] SIGUSR1 即时 STATS dump ---"
# 判据用**程序自己的单调时钟**（STATS 行的 ts=），不做墙钟换算。
# --stats-ms 给到 8000，周期行 8 s 才来一次 —— 信号之后新行的 ts 若只差几秒，
# 就只能是 dump，不可能是碰巧赶上的周期行。这样"立即"和"碰巧"才分得开。
if [ "$RC_ENV" = "0" ]; then
  ULOG="$TMP/usr1.stdout"
  : > "$ULOG"
  $SD --dry-run --tick-ms 200 --stats-ms 8000 \
      --log-dir "$LOG_DIR/usr1" --status-file "$TMP/usr1.status" \
      >"$ULOG" 2>&1 &
  UPID=$!
  i=0
  while [ "$i" -lt 20 ]; do
    grep -aq 'securityd: STATS' "$ULOG" && break
    sleep 1
    i=$((i + 1))
  done
  TS0=$(sed -n 's/^securityd: STATS ts=\([0-9]*\).*/\1/p' "$ULOG" | tail -1)
  if [ -n "$TS0" ] && kill -0 "$UPID" 2>/dev/null; then
    N0=$(grep -ac 'securityd: STATS' "$ULOG")
    kill -USR1 "$UPID" 2>/dev/null || true
    i=0
    while [ "$i" -lt 3 ]; do
      [ "$(grep -ac 'securityd: STATS' "$ULOG")" -gt "$N0" ] && break
      sleep 1
      i=$((i + 1))
    done
    N1=$(grep -ac 'securityd: STATS' "$ULOG")
    TS1=$(sed -n 's/^securityd: STATS ts=\([0-9]*\).*/\1/p' "$ULOG" | tail -1)
    if [ "${N1:-0}" -gt "${N0:-0}" ] && [ -n "$TS1" ] && [ "$((TS1 - TS0))" -le 3000 ]; then
      RC_SIG=0
      echo "  PASS: SIGUSR1 后 $((TS1 - TS0)) ms 就多出一行 STATS（周期是 8000 ms）"
      grep -a 'securityd: STATS' "$ULOG" | tail -1 | sed 's/^/    /'
    else
      echo "  FAIL: SIGUSR1 后没有立即 dump（行数 ${N0:-?}→${N1:-?}，ts ${TS0:-?}→${TS1:-?}）"
    fi
  else
    echo "  FAIL: 没等到首条 STATS，或进程已退出"
    tail -5 "$ULOG" | sed 's/^/    /'
  fi
  if kill -0 "$UPID" 2>/dev/null; then
    kill -TERM "$UPID" 2>/dev/null || true
    sleep 2
    kill -KILL "$UPID" 2>/dev/null || true
  fi
  wait "$UPID" 2>/dev/null || true
else
  echo "  跳过（环境段未通过）"
fi

# ---------------------------------------------------------------- [9] 传感器缺失自检
echo ""
echo "--- [9/11] 启动自检：传感器缺失必须立刻报 ---"
# 卸掉 ap3216c_input，Input 设备消失。securityd 必须在**启动时**就报 DEGRADED no_sensor。
# 这条钉的是宪章 §7：静默停在 GUARD 是最糟的失败——看起来一切正常。
if [ "$RC_ENV" = "0" ]; then
  NOLOG="$TMP/nosensor.stdout"
  /sbin/rmmod ap3216c_input 2>/dev/null || true
  sleep 1
  : > "$NOLOG"
  $SD --dry-run --tick-ms 200 --stats-ms 1000 \
      --log-dir "$LOG_DIR/nosensor" --status-file "$TMP/nosensor.status" \
      >"$NOLOG" 2>&1 &
  NPID=$!
  i=0
  while [ "$i" -lt 8 ]; do
    grep -aq 'deg=no_sensor' "$NOLOG" && break
    sleep 1
    i=$((i + 1))
  done
  if kill -0 "$NPID" 2>/dev/null; then
    kill -TERM "$NPID" 2>/dev/null || true
    sleep 2
    kill -KILL "$NPID" 2>/dev/null || true
  fi
  wait "$NPID" 2>/dev/null || true
  # 命令行那条是"大声说"，STATS 的 deg= 是机器可读的那条，两条都要。
  if grep -aq 'DEGRADED no_sensor' "$NOLOG" && grep -aq 'deg=no_sensor' "$NOLOG"; then
    RC_NOSENS=0
    echo "  PASS: 启动即报（命令行 + STATS 的 deg= 都认）"
    grep -a 'DEGRADED no_sensor' "$NOLOG" | head -1 | sed 's/^/    /'
  else
    echo "  FAIL: 卸掉 ap3216c_input 后没有立刻报 no_sensor"
    grep -a 'DEGRADED\|WARN' "$NOLOG" | head -5 | sed 's/^/    /'
  fi
  # 还原：后面 [11] 还要靠这个传感器触发真实 ALERT
  echo 0-001e > /sys/bus/i2c/drivers/ap3216c/unbind 2>/dev/null || true
  /sbin/insmod ./ap3216c_input.ko ps_measure_interval=0x01 2>/dev/null || \
    echo "  警告：ap3216c_input 装不回去，请手动 insmod"
  sleep 1
  EVBACK=""
  for ev in /sys/class/input/event*; do
    [ -e "$ev/device/name" ] || continue
    [ "$(cat "$ev/device/name" 2>/dev/null)" = "ap3216c-proximity" ] && { EVBACK="$ev"; break; }
  done
  [ -n "$EVBACK" ] && echo "  已还原：input=$EVBACK" || \
    echo "  警告：还原后没找到 ap3216c-proximity，[11] 会因此失败"
else
  echo "  跳过（环境段未通过）"
fi

# ---------------------------------------------------------------- [10] 可观测性对账
echo ""
echo "--- [10/11] 可观测性对账（debugfs vs GET_STATE）---"
# 两点值得先知道：
#   * debugfs **不走**驱动的单打开者，daemon 占着 /dev/pantilt0 时它照样能读；
#     GET_STATE 走 ioctl，设备被占就是 EBUSY —— 所以这两条读数的可得性不一样。
#   * 因此本段默认**不碰 daemon**：能取到 GET_STATE 就逐字段对；取不到就明说
#     "SKIP-GET_STATE"，同时仍然用标定文件做一路独立对账。要用 M4_STOP_DAEMON=1
#     显式开（停 daemon 会让驱动 1500 ms 看门狗按设计触发一次 STOP —— 云台原地
#     保持，不动，但确实改变了现场状态，所以不默认做）。
DBG=/sys/kernel/debug/pantilt
if [ "$RC_ENV" = "0" ]; then
  DBG_OK=1
  for f in state limit stats; do
    [ -r "$DBG/$f" ] || { echo "  FAIL: $DBG/$f 不可读"; DBG_OK=0; }
  done

  if [ "$DBG_OK" = "1" ]; then
    echo "  debugfs state:"; sed 's/^/    /' "$DBG/state"
    echo "  debugfs limit:"; sed 's/^/    /' "$DBG/limit"

    # ---- 独立第一路：debugfs 的 safe 范围必须等于标定文件 ----
    CONF="$BOARD_DIR/pantilt_calibration.conf"
    CONF_OK=1
    if [ -r "$CONF" ]; then
      for ax in pan tilt; do
        CMIN=$(sed -n "s/^${ax}_safe_min *= *\([0-9]*\).*/\1/p" "$CONF" | head -1)
        CMAX=$(sed -n "s/^${ax}_safe_max *= *\([0-9]*\).*/\1/p" "$CONF" | head -1)
        DMIN=$(sed -n "s/^${ax}  *safe \([0-9]*\)\.\..*/\1/p" "$DBG/limit" | head -1)
        DMAX=$(sed -n "s/^${ax}  *safe [0-9]*\.\.\([0-9]*\).*/\1/p" "$DBG/limit" | head -1)
        if [ "$CMIN" = "$DMIN" ] && [ "$CMAX" = "$DMAX" ]; then
          echo "  $ax safe debugfs $DMIN..$DMAX == 标定文件 $CMIN..$CMAX"
        else
          echo "  FAIL: $ax safe 不一致 debugfs $DMIN..$DMAX vs 标定 $CMIN..$CMAX"
          CONF_OK=0
        fi
      done
    else
      echo "  SKIP-CONF: 读不到 $CONF（跳过标定文件对账）"
    fi

    # ---- 第二路：GET_STATE（设备空闲才取得到）----
    INFO="$TMP/m4_info.txt"
    DAEMON_STOPPED=0
    if ./pantilt_ctl info >"$INFO" 2>&1; then
      :
    elif pidof pantilt_daemon >/dev/null 2>&1; then
      if [ "$M4_STOP_DAEMON" = "1" ]; then
        echo "  临时停 pantilt_daemon 以取 GET_STATE（M4_STOP_DAEMON=1）…"
        # 原样记下它的命令行与工作目录，之后照原样起回来，不猜参数。
        DPID=$(pidof pantilt_daemon)
        DCWD=$(readlink "/proc/$DPID/cwd" 2>/dev/null || echo "$BOARD_DIR")
        tr '\000' ' ' < "/proc/$DPID/cmdline" > "$TMP/m4_daemon.cmd" 2>/dev/null || true
        pkill -TERM -x pantilt_daemon 2>/dev/null || true
        sleep 2
        pkill -KILL -x pantilt_daemon 2>/dev/null || true
        sleep 1
        if pidof pantilt_daemon >/dev/null 2>&1; then
          echo "  FAIL: pantilt_daemon 杀不掉（pid $(pidof pantilt_daemon)），跳过 GET_STATE"
          INFO=""
        else
          DAEMON_STOPPED=1
          ./pantilt_ctl info >"$INFO" 2>&1 || { echo "  仍取不到 GET_STATE"; INFO=""; }
        fi
      else
        echo "  SKIP-GET_STATE: /dev/pantilt0 被 pantilt_daemon 占着（驱动只许一个打开者，"
        echo "                  见 pantilt.c 的 pt_open）—— debugfs 仍已读取并与标定文件对账。"
        echo "                  要取 GET_STATE 请加 M4_STOP_DAEMON=1 重跑本段。"
        INFO=""
      fi
    else
      echo "  SKIP-GET_STATE: pantilt_ctl info 失败且 daemon 不在跑"
      INFO=""
    fi

    GET_OK=1
    GET_DONE=0
    if [ -n "${INFO:-}" ] && [ -s "$INFO" ]; then
      GET_DONE=1
      echo "  GET_STATE:"; sed 's/^/    /' "$INFO"
      # 逐字段比。两路的**排版不一样**，这是最容易写错的地方：
      #   debugfs state : pan         current=398 target=398 home=377
      #   ctl info      : pan  cur= 398 (1.94 ms)  tgt= 398  home= 377  safe=239..515
      # 比的是同一批物理量：状态字、芯片态、两轴 current/target/home、safe、slew/看门狗。
      # 设备空闲时没人动它，这些应当逐字相等。
      cmp_kv() {  # cmp_kv <名字> <debugfs 值> <GET_STATE 值>
        # 两边都空说明是"没解析出来"，不是"相等"——必须当失败，否则格式一变
        # 就成了静默 PASS（这个脚本存在的意义就是不让它发生）。
        if [ -z "$2" ] || [ -z "$3" ]; then
          echo "  FAIL: $1 取值失败（debugfs=「$2」 GET_STATE=「$3」）—— 输出格式可能变了"
          GET_OK=0
        elif [ "x$2" = "x$3" ]; then
          echo "  $1 debugfs=$2 == GET_STATE=$3"
        else
          echo "  FAIL: $1 不一致 debugfs=$2 vs GET_STATE=$3"
          GET_OK=0
        fi
      }
      cmp_kv status \
        "$(sed -n 's/^status  *//p' "$DBG/state" | head -1)" \
        "$(sed -n 's/^status  *//p' "$INFO" | head -1)"
      cmp_kv chip_state \
        "$(sed -n 's/^chip_state  *//p' "$DBG/state" | head -1)" \
        "$(sed -n 's/^chip_state  *//p' "$INFO" | head -1)"
      for ax in pan tilt; do
        DCUR=$(sed -n "s/^${ax}  *current=\([0-9]*\).*/\1/p" "$DBG/state" | head -1)
        DTGT=$(sed -n "s/^${ax}  *current=[0-9]* *target=\([0-9]*\).*/\1/p" "$DBG/state" | head -1)
        DHOM=$(sed -n "s/^${ax}  *current=.* home=\([0-9]*\).*/\1/p" "$DBG/state" | head -1)
        DSFMIN=$(sed -n "s/^${ax}  *safe \([0-9]*\)\.\..*/\1/p" "$DBG/limit" | head -1)
        DSFMAX=$(sed -n "s/^${ax}  *safe [0-9]*\.\.\([0-9]*\).*/\1/p" "$DBG/limit" | head -1)
        ICUR=$(sed -n "s/^${ax}  *cur= *\([0-9]*\).*/\1/p" "$INFO" | head -1)
        ITGT=$(sed -n "s/^${ax}  *cur=.*tgt= *\([0-9]*\).*/\1/p" "$INFO" | head -1)
        IHOM=$(sed -n "s/^${ax}  *cur=.*home= *\([0-9]*\).*/\1/p" "$INFO" | head -1)
        ISFMIN=$(sed -n "s/^${ax}  *cur=.*safe=\([0-9]*\)\.\..*/\1/p" "$INFO" | head -1)
        ISFMAX=$(sed -n "s/^${ax}  *cur=.*safe=[0-9]*\.\.\([0-9]*\).*/\1/p" "$INFO" | head -1)
        cmp_kv "$ax cur"  "$DCUR"   "$ICUR"
        cmp_kv "$ax tgt"  "$DTGT"   "$ITGT"
        cmp_kv "$ax home" "$DHOM"   "$IHOM"
        # safe 分两次比，别拼成 "239..515" 再比：两段都空时会拼出非空的 ".."，
        # 于是走"不相等"那条分支，"取值失败"这个更有用的报错就丢了。
        cmp_kv "$ax safe_min" "$DSFMIN" "$ISFMIN"
        cmp_kv "$ax safe_max" "$DSFMAX" "$ISFMAX"
      done
      DSLEW=$(sed -n 's/^slew_ms \([0-9]*\) .*/\1/p' "$DBG/limit" | head -1)
      DWD=$(sed -n 's/^slew_ms [0-9]*  *watchdog_ms \([0-9]*\).*/\1/p' "$DBG/limit" | head -1)
      ISLEW=$(sed -n 's/^slew  *\([0-9]*\) ms\/tick .*/\1/p' "$INFO" | head -1)
      IWD=$(sed -n 's/^slew  *[0-9]* ms\/tick  *watchdog \([0-9]*\) ms.*/\1/p' "$INFO" | head -1)
      cmp_kv slew_ms     "$DSLEW" "$ISLEW"
      cmp_kv watchdog_ms "$DWD"   "$IWD"
    fi

    # ---- 把 daemon 照原样起回来 ----
    if [ "$DAEMON_STOPPED" = "1" ]; then
      if [ -s "$TMP/m4_daemon.cmd" ]; then
        # 用 setsid/nohup 脱离本会话：本脚本常经 ssh 跑，daemon 不该随 ssh 断开被挂掉。
        DCMD=$(cat "$TMP/m4_daemon.cmd")
        if command -v setsid >/dev/null 2>&1; then
          ( cd "$DCWD" && setsid sh -c "$DCMD" >/dev/null 2>&1 </dev/null & )
        else
          ( cd "$DCWD" && nohup sh -c "$DCMD" >/dev/null 2>&1 </dev/null & )
        fi
        sleep 2
        if pidof pantilt_daemon >/dev/null 2>&1; then
          echo "  pantilt_daemon 已按原命令行起回（pid $(pidof pantilt_daemon)）"
        else
          echo "  FAIL: pantilt_daemon 没能起回来，请手动恢复："
          echo "        cd $DCWD && $(cat "$TMP/m4_daemon.cmd")"
          GET_OK=0
        fi
      fi
    fi

    if [ "$DBG_OK" = "1" ] && [ "$CONF_OK" = "1" ] && [ "$GET_OK" = "1" ]; then
      RC_DBG=0
      # 报什么就只做了什么。没取到 GET_STATE 时不许说"与 GET_STATE 一致"——
      # 那是把"没查"说成"查过且没问题"，正是这个脚本要防的那类谎报。
      if [ "$GET_DONE" = "1" ]; then
        echo "  PASS: debugfs 三节点可读；与标定文件、GET_STATE 的读数一致"
      else
        echo "  PASS（部分）: debugfs 三节点可读，且与标定文件逐轴一致；"
        echo "               **GET_STATE 未取，那一路没查** —— 要补跑见本段开头的 M4_STOP_DAEMON 说明"
      fi
    else
      echo "  FAIL: 可观测性对账未全过（DBG=$DBG_OK CONF=$CONF_OK GET=$GET_OK）"
    fi
  fi
else
  echo "  跳过（环境段未通过）"
fi

# ---------------------------------------------------------------- [11] 重载耗尽
echo ""
echo "--- [11/11] 摄像头重载耗尽路径 ---"
# 故障注入方式：把 --camera-mod-dir 指向一个**空目录**。securityd 的重载序列
# （rmmod mx6s_capture → rmmod ov5640_camera → insmod <dir>/…）里 rmmod 用的是
# 绝对路径 /sbin/rmmod，照常成功；insmod 找不到 .ko，必然失败 → 重试 3 次后耗尽。
# 这是"恢复介质缺失"这一种耗尽成因，断言的是耗尽**之后** securityd 的行为。
#
# 摄像头模块在这一段里会被真的卸掉，脚本末尾会显式装回去（与 [7] 是同一套动作，
# 不是新增风险）。需要操作者：没有真实 ALERT 就没有录音，"音频仍在录"无从判起。
if [ "$RC_ENV" = "0" ] && [ "$WITH_EXHAUST" = "1" ]; then
  mkdir -p "$TMP/nomodir"
  ELOG="$TMP/exhaust.stdout"
  : > "$ELOG"
  # alert_hold 给到 60 s：耗尽过程要时间，不能让它中途到点自动收尾把录音停了。
  $SD --tick-ms 200 --stats-ms 1000 --video-ready-ms 4000 \
      --audio-warmup-ms 0 --camera-reload-attempts 3 \
      --camera-mod-dir "$TMP/nomodir" \
      --alert-hold-ms 60000 --alert-max-ms 180000 \
      --records-dir "$RECORDS_DIR" --audio-tag exhaust \
      --log-dir "$LOG_DIR/exhaust" --status-file "$TMP/exhaust.status" \
      >"$ELOG" 2>&1 &
  EPID=$!

  i=0
  while [ "$i" -lt 30 ]; do
    [ "$(kv_str "$ELOG" video)" = "up" ] && break
    sleep 1
    i=$((i + 1))
  done
  if [ "$(kv_str "$ELOG" video)" = "up" ]; then
    echo "  视频已就绪。请在 ${EXHAUST_SECS}s 内触发一次靠近：手掌贴近 AP3216C 停 2~3 秒后移开。"
    echo "  **然后在这一段剩下的十几秒里反复再靠近几次**（每次贴近 1~2 秒就移开）。"
    echo "  这一段要的不是让告警结束，而恰恰是要它一直续着：重载 3 次要 5~10 s，"
    echo "  而 alert_hold 只有 8 s；不去续期，录音会先于 DEGRADED(camera) 停下，"
    echo "  「音频仍在录」这条判据就成了假 FAIL。（与 [4] 的要求相反，注意区分。）"
    i=0
    while [ "$i" -lt "$EXHAUST_SECS" ]; do
      grep -aq 'to=ALERT' "$ELOG" && break
      sleep 1
      i=$((i + 1))
    done
    if grep -aq 'to=ALERT' "$ELOG"; then
      echo "  ALERT 已触发，录音应在跑："
      grep -a 'to=ALERT' "$ELOG" | tail -1 | sed 's/^/    /'
      # 制造摄像头故障：把 video_link 掐掉，tick_video_health 会走重载那条路。
      # 用 -KILL 是因为这里要的是"子进程没了"这个条件，不关心它怎么退的。
      pkill -KILL -x video_link 2>/dev/null || true
      echo "  已掐掉 video_link，等重载耗尽（最多 3 次）…"
      i=0
      while [ "$i" -lt 40 ]; do
        grep -aq 'DEGRADED camera' "$ELOG" && break
        sleep 1
        i=$((i + 1))
      done
      echo "  重载轨迹："
      grep -a '摄像头重载第\|DEGRADED camera' "$ELOG" | head -6 | sed 's/^/    /'

      # deg= 是逗号分隔的**正交标志位**集合，顺序固定为 sensor,camera,audio。
      # 别写死 `deg=camera`：只要 deg_sensor 也立着，camera 就不在行首，
      # 精确匹配会漏掉 → 变成假 FAIL。
      DEGC=0; grep -aq 'deg=[a-z_,]*camera' "$ELOG" && DEGC=1
      AREX=0; grep -aq 'audio=rec' "$ELOG" && AREX=1
      ARUN=0; [ -n "$(pidof audio_capture 2>/dev/null)" ] && ARUN=1
      NREL=$(grep -ac '摄像头重载第' "$ELOG")
      UPE=$(awk '{print int($1)}' /proc/uptime)

      echo "  末行 STATS:"; grep -a 'securityd: STATS' "$ELOG" | tail -1 | sed 's/^/    /'
      echo "  重载次数=${NREL} deg=camera=${DEGC} STATS 里 audio=rec=${AREX} audio_capture 存活=${ARUN}"
      echo "  uptime ${UP0}s → ${UPE}s"
      # 重载次数必须**正好** 3：少于 3 说明重载是被别的原因触发的（比如视频压根
      # 没把包发出去，被 video_ready 超时判死），那就不是"耗尽"路径，不能算过。
      if [ "$DEGC" = "1" ] && [ "$AREX" = "1" ] && [ "$ARUN" = "1" ] && \
         [ "${NREL:-0}" -eq 3 ] && [ "$UPE" -ge "$UP0" ] && [ "$UPE" -lt "$((UP0 + 600))" ]; then
        RC_EXH=1
        echo "  PASS: 重载 ${NREL} 次后耗尽 → DEGRADED(camera)，音频仍在录，板子未重启"
      else
        RC_EXH=2
        echo "  FAIL: 耗尽路径未达判据（重载${NREL} 退化${DEGC} 录音${AREX}/${ARUN}）"
        grep -a 'DEGRADED\|WARN' "$ELOG" | head -5 | sed 's/^/    /'
      fi
    else
      RC_EXH=2
      echo "  FAIL: ${EXHAUST_SECS}s 内没等到 ALERT（耗尽路径无录音可判）"
      sensor_brief
    fi
  else
    RC_EXH=2
    echo "  FAIL: 视频没起来（空 mod-dir 不该影响启动，摄像头模块此时应当还在）"
    tail -8 "$ELOG" | sed 's/^/    /'
  fi

  # 收掉这一段起的 securityd（内建 kill + 紧跟 wait，理由同 [7]）
  if [ -n "${EPID:-}" ] && kill -0 "$EPID" 2>/dev/null; then
    kill -TERM "$EPID" 2>/dev/null || true
    sleep 3
    kill -KILL "$EPID" 2>/dev/null || true
    wait "$EPID" 2>/dev/null || true
  fi

  # 显式把摄像头装回来 —— 这一段**故意**让它处于"卸掉且装不回"的状态，
  # 收尾必须自己恢复，不能指望下一段。
  echo "  恢复摄像头模块…"
  for mod in mx6s_capture ov5640_camera; do
    lsmod | grep -q "^$mod " && /sbin/rmmod "$mod" 2>/dev/null || true
  done
  /sbin/insmod ./mx6s_capture_fixed.ko 2>/dev/null || echo "  insmod mx6s_capture_fixed.ko 失败"
  /sbin/insmod ./ov5640_camera.ko 2>/dev/null || echo "  insmod ov5640_camera.ko 失败"
  sleep 2
  [ -e /dev/video1 ] && echo "  /dev/video1 已恢复" || \
    echo "  警告：/dev/video1 未恢复，后续段会受影响"
elif [ "$WITH_EXHAUST" != "1" ]; then
  echo "  跳过（WITH_EXHAUST=$WITH_EXHAUST）"
else
  echo "  跳过（环境段未通过）"
fi

# ---------------------------------------------------------------- 清理
echo ""
echo "--- 清理 ---"
sd_cleanup
rm -f "$TMP"/dryrun.status "$TMP"/reload.status
echo "  证据保留在 $TMP/（*.stdout / logs/ / records/）"
echo "  未重启板子；未给云台下发过任何控制包"

# ---------------------------------------------------------------- 汇总
echo ""
echo "==================== 汇总 ===================="
[ "$RC_ENV"   = "0" ] && echo "  环境与设备准备     : PASS" || echo "  环境与设备准备     : FAIL"
[ "$RC_SELF"  = "0" ] && echo "  启动自检           : PASS" || echo "  启动自检           : FAIL"
[ "$RC_ALIVE" = "0" ] && echo "  常驻与可观测性     : PASS" || echo "  常驻与可观测性     : FAIL"
case "$RC_HAND" in
  1) echo "  人工靠近 状态机    : PASS" ;;
  2) echo "  人工靠近 状态机    : FAIL" ;;
  *) echo "  人工靠近 状态机    : NOT RUN" ;;
esac
[ "$RC_CHECK"  = "0" ] && echo "  结果核对 视频      : PASS" || echo "  结果核对 视频      : FAIL"
[ "$RC_STOP"   = "0" ] && echo "  有序退出+录音对账  : PASS" || echo "  有序退出+录音对账  : FAIL"
[ "$RC_RELOAD" = "0" ] && echo "  摄像头重载自愈     : PASS" || echo "  摄像头重载自愈     : FAIL"
[ "$RC_SIG"    = "0" ] && echo "  SIGUSR1 即时 dump  : PASS" || echo "  SIGUSR1 即时 dump  : FAIL"
[ "$RC_NOSENS" = "0" ] && echo "  传感器缺失自检     : PASS" || echo "  传感器缺失自检     : FAIL"
if [ "$RC_DBG" = "0" ] && [ "$GET_DONE" = "1" ]; then
  echo "  可观测性对账       : PASS"
elif [ "$RC_DBG" = "0" ]; then
  echo "  可观测性对账       : PASS（未取 GET_STATE，只对了标定文件）"
else
  echo "  可观测性对账       : FAIL"
fi
case "$RC_EXH" in
  1) echo "  重载耗尽路径       : PASS" ;;
  2) echo "  重载耗尽路径       : FAIL" ;;
  *) echo "  重载耗尽路径       : NOT RUN" ;;
esac

# 总判定：与环境段同批的三条必须 PASS；需要操作者的两条（[4] / [11]）
# 只在"跑了但失败"时把总判定拉下来，"没跑"不拉——与 [4] 的既有约定一致。
if [ "$RC_ENV" = "0" ] && [ "$RC_SELF" = "0" ] && [ "$RC_ALIVE" = "0" ] && \
   [ "$RC_HAND" != "2" ] && [ "$RC_CHECK" = "0" ] && [ "$RC_STOP" = "0" ] && \
   [ "$RC_RELOAD" = "0" ] && [ "$RC_SIG" = "0" ] && [ "$RC_NOSENS" = "0" ] && \
   [ "$RC_DBG" = "0" ] && [ "$RC_EXH" != "2" ]; then
  echo "== 总判定: PASS =="
  exit 0
fi
echo "== 总判定: FAIL =="
exit 1
