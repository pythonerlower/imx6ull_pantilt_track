#!/usr/bin/env sh
# Phase 5 合并验收：WM8960 用户态 ALSA 录音 / 播放（audio_capture / audio_play）
#
# 设计原则：
#   - 一次跑完全部 7 段，每段只留 1~2 条硬判据，不做时长/格式矩阵
#   - 判据尽量用**独立于被测程序**的第三者：WAV 合法性用板端 python3 的 wave 模块判，
#     播放是否真的出声用"声学回环 + Goertzel 频点检测"判，不靠人耳也不靠程序自述
#   - 不改任何 mixer 设置：实测板端 WM8960 出厂状态下 Speaker=101 即可被麦克风拾到
#   - 机器可读结果取程序自己打的一行 `audio: RESULT ...`（纯 ASCII），不解析中文
#
# 判据：
#   [1] 环境   : 声卡/设备节点/libasound/两个程序都在
#   [2] 默认目录录音 : 文件名符合 时间戳_tag.wav，WAV 头与文件长度自洽（python wave 判定）
#   [3] 指定路径 + 参数生效 : --output 原样使用；-r/-c/-f 真的写进了 WAV 头，退出码 0
#   [4] 播放   : 回放录音文件 / 生成单音 / 生成告警音，退出码 0 且 wall_s≈音频时长
#   [5] 声学回环 : 播放告警音的同时录音，频谱在 880 与 1320 Hz 明显高于本底
#   [6] XRUN   : SIGSTOP 制造 overrun，程序必须 recovery 并跑满全部帧
#   [7] 停止路径 : --duration 0 + SIGTERM，必须正常收尾且 WAV 头被回填
#
# 用法：sh ./board_verify_audio.sh [BOARD_DIR]
#      WITH_LOOPBACK=0  跳过声学回环（现场没有喇叭/麦克风时用，汇总记为 NOT RUN）
set -u

BOARD_DIR="${1:-/root/imx6hub_demo}"
cd "$BOARD_DIR" || exit 2

REC_DIR="${REC_DIR:-$BOARD_DIR/records}"
TMP="${TMP:-/tmp/phase5_audio}"
WITH_LOOPBACK="${WITH_LOOPBACK:-1}"

CAP=./audio_capture
PLAY=./audio_play

RC_ENV=1
RC_REC=1
RC_ARG=1
RC_PLAY=1
RC_LOOP=0        # 0=未跑/跳过，1=通过，2=失败
RC_XRUN=1
RC_STOP=1

mkdir -p "$TMP" || exit 2

# ---------------------------------------------------------------- 工具

# 从 `audio: RESULT ...` 行里取一个字段
res_field() {
  grep -a '^audio: RESULT' "$1" 2>/dev/null | tail -1 | \
    sed -n "s/.* $2=\([0-9][0-9.]*\).*/\1/p"
}

# 判断 wall_s 是否接近音频时长（容差 factor 倍以内），防止"写进去就返回"的假成功
wall_close() {   # wall_close <wall_s> <audio_s> <上限倍率>
  awk -v w="$1" -v a="$2" -v f="$3" 'BEGIN {
      if (a <= 0 || w <= 0) exit 1;
      exit !(w >= 0.85 * a && w <= f * a);
    }'
}

echo "==================== Phase 5 合并验收（WM8960 ALSA 应用） ===================="

# 板端 python3 的 wave 模块做独立校验；输出一律 ASCII，
# 因为非交互 shell 的 stdout 编码是 ASCII，中文会直接抛 UnicodeEncodeError。
cat > "$TMP/wavcheck.py" <<'PYEOF'
import sys, wave, os, array, math
p = sys.argv[1]
exp_rate = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] != '-' else 0
exp_ch   = int(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] != '-' else 0
exp_bits = int(sys.argv[4]) if len(sys.argv) > 4 and sys.argv[4] != '-' else 0
try:
    w = wave.open(p, 'rb')
except Exception as e:
    print("FAIL cannot open as WAV: %s" % e); sys.exit(1)
n = w.getnframes(); ch = w.getnchannels(); sw = w.getsampwidth()
fr = w.getframerate(); raw = w.readframes(n); w.close()
size = os.path.getsize(p)
errs = []
# 空录音（44 字节纯头）必须直接判失败：此时下面的静音检查被 n>0 挡掉，
# 只剩 size/rate/ch/bits 三条，全都会"自洽"通过 —— 采集方向坏掉时正是这个形状。
if n <= 0:
    errs.append("nframes=0 (no audio captured)")
if size != 44 + n * ch * sw:
    errs.append("size=%d != 44+%d" % (size, n * ch * sw))
if exp_rate and fr != exp_rate: errs.append("rate=%d != %d" % (fr, exp_rate))
if exp_ch and ch != exp_ch:     errs.append("ch=%d != %d" % (ch, exp_ch))
if exp_bits and sw * 8 != exp_bits: errs.append("bits=%d != %d" % (sw * 8, exp_bits))
rms = 0.0; pk = 0
code = {2: 'h', 4: 'l'}.get(sw)
if code and n > 0:
    a = array.array(code); a.frombytes(raw[:len(raw) // sw * sw])
    if a:
        m = sum(a) / len(a)
        rms = math.sqrt(sum((x - m) ** 2 for x in a) / len(a))
        pk = max(abs(x - m) for x in a)
        # 归一化到 16 位量级，便于不同位宽横向比较
        scale = 1 << (8 * sw - 16)
        rms /= float(scale); pk //= scale
    if rms < 1.0:
        errs.append("acrms=%.2f (capture path silent?)" % rms)
print("%s nframes=%d rate=%d ch=%d width=%d dur=%.2fs acrms=%.1f acpeak=%d %s"
      % (p, n, fr, ch, sw, n / float(fr) if fr else 0.0, rms, pk,
         "OK" if not errs else "FAIL " + "; ".join(errs)))
sys.exit(0 if not errs else 1)
PYEOF

cat > "$TMP/tonecheck.py" <<'PYEOF'
# Goertzel 频点检测：告警音是 880/1320 Hz 交替、且带 0.25s 响 / 0.15s 停的门控。
# 用整段做 DFT 会把静音间隙一起摊进去，信噪比被人为压低；
# 所以按 0.30s 子窗滑动（与"响"段同量级，单窗能盖住一整段），
# 每个频点各取自己最好的一窗——两个频点本来就不在同一时刻出现。
import sys, wave, array, math
p = sys.argv[1]; start = float(sys.argv[2]); dur = float(sys.argv[3])
w = wave.open(p, 'rb'); fr = w.getframerate(); ch = w.getnchannels(); sw = w.getsampwidth()
w.setpos(int(start * fr)); raw = w.readframes(int(dur * fr)); w.close()
if sw != 2:
    print("FAIL tone check needs 16-bit"); sys.exit(1)
a = array.array('h'); a.frombytes(raw); x = [float(v) for v in a[0::ch]]
if len(x) < 2000:
    print("FAIL window too short (%d samples)" % len(x)); sys.exit(1)

def goertzel(sig, f, fs):
    c = 2.0 * math.cos(2.0 * math.pi * f / fs); s1 = s2 = 0.0
    for v in sig:
        s0 = v + c * s1 - s2; s2 = s1; s1 = s0
    return math.sqrt(max(s1 * s1 + s2 * s2 - c * s1 * s2, 0.0)) / (len(sig) / 2.0)

OTHERS = (200, 400, 600, 1000, 1200, 1600, 2000, 2620, 4000)

def snr(sig):
    m = sum(sig) / len(sig); sig = [v - m for v in sig]
    fl = sum(goertzel(sig, f, fr) for f in OTHERS) / len(OTHERS)
    fl = max(fl, 1e-9)
    return (20.0 * math.log10(max(goertzel(sig, 880.0, fr), 1e-9) / fl),
            20.0 * math.log10(max(goertzel(sig, 1320.0, fr), 1e-9) / fl), fl)

# 只看"最好的一窗"会被多重比较带偏（几十个窗口里总有一个噪声窗冒头）。
# 改成计数：够多的窗口超过 THR 才判定有声。合成信号实测噪声最好一窗仅 +9.4dB，
# 真实鸣叫则常驻 +80dB 以上，用 14dB 阈值把两者彻底分开。
#
# 判据取"两个频点里任意一个超阈"的窗口数（不是各算各的）：告警音本来就是
# 880/1320 交替，一个窗里只有其中一个在响，各算各的会把要求凭空翻倍。
# 板端实测 4 轮同一手法：弱的 +18.9dB / 5 窗，强的 +34.2dB / 19 窗，
# 取并集后最弱一轮也有十几窗，噪声 0 窗 —— NEED=5 两侧都有余量。
THR, NEED = 14.0, 5
win = int(0.30 * fr)
hop = max(1, int(0.05 * fr))
n880 = n1320 = nany = total = 0
b880, b1320 = (-99.0, 0.0), (-99.0, 0.0)
for i in range(0, max(1, len(x) - win + 1), hop):
    d8, d13, fl = snr(x[i:i + win])
    total += 1
    if d8 >= THR:  n880 += 1
    if d13 >= THR: n1320 += 1
    if d8 >= THR or d13 >= THR: nany += 1
    if d8 > b880[0]:   b880 = (d8, fl)
    if d13 > b1320[0]: b1320 = (d13, fl)
print("880Hz best=%+.1fdB(%d win)  1320Hz best=%+.1fdB(%d win)  "
      "either>=%.0fdB: %d/%d win  (need >=%d)"
      % (b880[0], n880, b1320[0], n1320, THR, nany, total, NEED))
sys.exit(0 if nany >= NEED else 1)
PYEOF

# ---------------------------------------------------------------- [1/7] 环境
echo ""
echo "--- [1/7] 环境与依赖 ---"
CARD=$(grep -a -m1 "wm8960" /proc/asound/cards 2>/dev/null | sed 's/^ *//')
echo "  /proc/asound/cards : ${CARD:-（未找到 wm8960）}"
echo "  /dev/snd          : $(ls /dev/snd 2>/dev/null | tr '\n' ' ')"
ASO=$(ls /usr/lib/libasound.so.2* 2>/dev/null | head -1)
echo "  libasound         : ${ASO:-缺失}"
for f in "$CAP" "$PLAY"; do
  [ -x "$f" ] || echo "  FAIL 缺少可执行文件 $f"
done
if [ -n "$CARD" ] && [ -c /dev/snd/pcmC0D0c ] && [ -c /dev/snd/pcmC0D0p ] && \
   [ -n "$ASO" ] && [ -x "$CAP" ] && [ -x "$PLAY" ]; then
  RC_ENV=0
  echo "  PASS: 声卡 + 录放节点 + libasound + 两个程序齐备"
else
  echo "  FAIL: 环境不完整"
fi

# ---------------------------------------------------------------- [2/7] 默认目录录音
echo ""
echo "--- [2/7] 录音到默认目录（时间戳命名）---"
$CAP --tag vfy --duration 3 > "$TMP/rec_default.log" 2>&1
RC=$?
NEWEST=$(ls -t "$REC_DIR"/????????_??????_vfy.wav 2>/dev/null | head -1)
echo "  退出码=$RC  最新文件=${NEWEST:-无}"
sed 's/^/    /' "$TMP/rec_default.log"
KEEP_WAV=""
CHK_RC=1
if [ "$RC" = "0" ] && [ -n "$NEWEST" ]; then
  # 注意：判据不能用 `python3 ... | sed ...` —— 管道的退出码是 sed 的，恒为 0。
  CHK=$(python3 "$TMP/wavcheck.py" "$NEWEST" 44100 2 16)
  CHK_RC=$?
  echo "  $CHK"
fi
if [ "$CHK_RC" = "0" ]; then
  RC_REC=0
  KEEP_WAV="$NEWEST"
  echo "  PASS: 默认目录录音 + 文件名格式 + WAV 头自洽"
else
  echo "  FAIL: 默认目录录音或 WAV 校验失败"
fi

# ---------------------------------------------------------------- [3/7] 指定路径 + 参数
echo ""
echo "--- [3/7] --output 指定路径 + 参数生效 ---"
ARG_OK=0
# 退出码也要进判据：采集方向半坏时会写出"头合法但只有一小段数据"的文件，
# 光看 WAV 头仍然是 OK。（早先这里只 echo 不计数，坏状态照样 PASS。）
ARG_RC=0
$CAP --output "$TMP/custom.wav" --duration 1 -r 22050 -c 1 -f S16_LE --quiet \
    > "$TMP/rec_custom.log" 2>&1 || ARG_RC=$?
CHK=$(python3 "$TMP/wavcheck.py" "$TMP/custom.wav" 22050 1 16)
CHK_RC=$?
echo "  $CHK"
[ "$ARG_RC" != "0" ] && echo "  FAIL 录音退出码 $ARG_RC（22050/1ch/16bit）"
[ "$CHK_RC" = "0" ] && [ "$ARG_RC" = "0" ] && ARG_OK=$((ARG_OK + 1))
ARG_RC=0
$CAP --output "$TMP/custom32.wav" --duration 1 -r 48000 -c 2 -f S32_LE --quiet \
    > "$TMP/rec_custom32.log" 2>&1 || ARG_RC=$?
CHK=$(python3 "$TMP/wavcheck.py" "$TMP/custom32.wav" 48000 2 32)
CHK_RC=$?
echo "  $CHK"
[ "$ARG_RC" != "0" ] && echo "  FAIL 录音退出码 $ARG_RC（48000/2ch/32bit）"
[ "$CHK_RC" = "0" ] && [ "$ARG_RC" = "0" ] && ARG_OK=$((ARG_OK + 1))
# 设备限制（实测，不是代码缺陷）：hw:0,0 只接受 S16_LE/S24_LE/S32_LE/S20_3LE，
# 且 8000 Hz 装不上 hw_params（arecord -r 8000 同样失败）。这两条只做负向确认：
$CAP --output "$TMP/bad.wav" --duration 0.2 -r 8000 --quiet > "$TMP/rec_8k.log" 2>&1 && \
  echo "  WARN: 8000 Hz 竟然成功了，与既有实测不符" || \
  echo "  负向确认: 8000 Hz 被拒（已知设备限制，arecord 同样失败）"
if [ "$ARG_OK" = "2" ]; then
  RC_ARG=0
  echo "  PASS: --output 原样使用，-r/-c/-f 均写进了 WAV 头"
else
  echo "  FAIL: 参数没有正确落到 WAV 头（$ARG_OK/2）"
fi

# ---------------------------------------------------------------- [4/7] 播放
echo ""
echo "--- [4/7] 播放：回放录音文件 / 单音 / 告警音 ---"
PLAY_OK=0
if [ -n "$KEEP_WAV" ]; then
  $PLAY --file "$KEEP_WAV" > "$TMP/play_file.log" 2>&1
  RC=$?
  W=$(res_field "$TMP/play_file.log" wall_s)
  echo "  回放 $KEEP_WAV: 退出码=$RC wall_s=${W:-?}（音频 3.00s）"
  if [ "$RC" = "0" ] && wall_close "$W" 3.0 1.5; then
    PLAY_OK=$((PLAY_OK + 1))
    echo "    PASS: 录音文件可正常回放，设备按实时速率消费"
  else
    echo "    FAIL: 回放异常"
  fi
else
  echo "  回放跳过：第 2 段没有可用的录音文件"
fi

$PLAY --tone 1000 --duration 2 --quiet > "$TMP/play_tone.log" 2>&1
RC=$?
W=$(res_field "$TMP/play_tone.log" wall_s)
echo "  单音 1000Hz 2s : 退出码=$RC wall_s=${W:-?}"
[ "$RC" = "0" ] && wall_close "$W" 2.0 1.5 && PLAY_OK=$((PLAY_OK + 1)) || \
  echo "    FAIL: 单音播放异常"

$PLAY --alert --duration 3 --quiet > "$TMP/play_alert.log" 2>&1
RC=$?
W=$(res_field "$TMP/play_alert.log" wall_s)
echo "  告警双音 3s    : 退出码=$RC wall_s=${W:-?}"
[ "$RC" = "0" ] && wall_close "$W" 3.0 1.5 && PLAY_OK=$((PLAY_OK + 1)) || \
  echo "    FAIL: 告警音播放异常"

if [ "$PLAY_OK" = "3" ]; then
  RC_PLAY=0
  echo "  PASS: 三种播放方式都退出 0 且按实时速率消费"
else
  echo "  FAIL: 播放项 $PLAY_OK/3"
fi

# ---------------------------------------------------------------- [5/7] 声学回环
echo ""
if [ "$WITH_LOOPBACK" = "1" ]; then
  echo "--- [5/7] 声学回环：播放告警音的同时录音，检测 880/1320 Hz ---"
  echo "  （不改任何 mixer 设置；依赖板上喇叭与麦克风的声学耦合）"
  $PLAY --alert --duration 4 --quiet > "$TMP/loop_play.log" 2>&1 &
  PLAY_PID=$!
  sleep 1
  $CAP --output "$TMP/loop.wav" --duration 4 --quiet > "$TMP/loop_cap.log" 2>&1
  wait $PLAY_PID
  echo "  播放: $(grep -a '^audio: RESULT' "$TMP/loop_play.log" | tail -1)"
  echo "  录音: $(grep -a '^audio: RESULT' "$TMP/loop_cap.log" | tail -1)"
  # 录音比播放晚约 1s 才开始，所以只有录音 1.1s 之后的段落里才有声音
  CHK=$(python3 "$TMP/tonecheck.py" "$TMP/loop.wav" 1.1 2.8)
  CHK_RC=$?
  echo "  $CHK"
  if [ "$CHK_RC" = "0" ]; then
    RC_LOOP=1
    echo "  PASS: 录到的信号在 880 与 1320 Hz 明显高于本底，声音确实从喇叭出去又被麦克风收回来了"
  else
    RC_LOOP=2
    echo "  FAIL: 回环里没检出告警音频率（喇叭没接/静音，或麦克风被堵）"
  fi
else
  echo "--- [5/7] 声学回环：跳过（WITH_LOOPBACK=$WITH_LOOPBACK）---"
fi

# ---------------------------------------------------------------- [6/7] XRUN
echo ""
echo "--- [6/7] XRUN 恢复：录音途中 SIGSTOP 1.2s 制造 overrun ---"
# period 1024 x 2 = 2048 帧 = 46ms 设备缓冲；停 1.2s 必然冲掉，readi 应返回 EPIPE
$CAP --output "$TMP/xrun.wav" --duration 4 --period-size 1024 --periods 2 --quiet \
    > "$TMP/xrun.log" 2>&1 &
CAP_PID=$!
sleep 1
kill -STOP $CAP_PID
sleep 1.2
kill -CONT $CAP_PID
wait $CAP_PID
RC=$?
XRUNS=$(res_field "$TMP/xrun.log" xrun)
FRAMES=$(res_field "$TMP/xrun.log" frames)
echo "  退出码=$RC xrun=${XRUNS:-?} frames=${FRAMES:-?}（4s@44100 应为 176400）"
grep -a "XRUN" "$TMP/xrun.log" | sed 's/^/    /'
if [ "$RC" = "0" ] && [ "${XRUNS:-0}" -ge 1 ] && [ "${FRAMES:-0}" = "176400" ] && \
   python3 "$TMP/wavcheck.py" "$TMP/xrun.wav" 44100 2 16 >/dev/null 2>&1; then
  RC_XRUN=0
  echo "  PASS: XRUN 被检出并恢复，帧数与 WAV 头都完整"
else
  echo "  FAIL: XRUN 段异常（未检出 overrun，或恢复后数据不完整）"
fi

# ---------------------------------------------------------------- [7/7] 停止路径
echo ""
echo "--- [7/7] 停止路径：--duration 0 + SIGTERM（securityd 的用法）---"
$CAP --output "$TMP/stop.wav" --duration 0 --quiet > "$TMP/stop.log" 2>&1 &
CAP_PID=$!
sleep 2.5
kill -TERM $CAP_PID
wait $CAP_PID
RC=$?
FRAMES=$(res_field "$TMP/stop.log" frames)
echo "  退出码=$RC frames=${FRAMES:-?}（约 2.5s，应 ≈110250）"
grep -a "停止信号" "$TMP/stop.log" | sed 's/^/    /'
CHK=$(python3 "$TMP/wavcheck.py" "$TMP/stop.wav" 44100 2 16)
CHK_RC=$?
echo "  $CHK"
if [ "$RC" = "0" ] && [ "${FRAMES:-0}" -gt 88200 ] && [ "$CHK_RC" = "0" ]; then
  RC_STOP=0
  echo "  PASS: SIGTERM 后正常收尾，WAV 头已回填，文件可直接播放"
else
  echo "  FAIL: 停止路径异常"
fi

# ---------------------------------------------------------------- 清理
echo ""
echo "--- 清理 ---"
rm -f "$TMP"/custom.wav "$TMP"/custom32.wav "$TMP"/bad.wav \
      "$TMP"/loop.wav "$TMP"/xrun.wav "$TMP"/stop.wav
echo "  保留证据: ${KEEP_WAV:-（无）}"
echo "  保留日志: $TMP/"
echo "  未改动任何 mixer 设置，未动 boot 分区"

# ---------------------------------------------------------------- 汇总
echo ""
echo "==================== 汇总 ===================="
[ "$RC_ENV"  = "0" ] && echo "  环境与依赖           : PASS" || echo "  环境与依赖           : FAIL"
[ "$RC_REC"  = "0" ] && echo "  默认目录录音+WAV校验 : PASS" || echo "  默认目录录音+WAV校验 : FAIL"
[ "$RC_ARG"  = "0" ] && echo "  指定路径+参数生效    : PASS" || echo "  指定路径+参数生效    : FAIL"
[ "$RC_PLAY" = "0" ] && echo "  播放(文件/单音/告警) : PASS" || echo "  播放(文件/单音/告警) : FAIL"
case "$RC_LOOP" in
  1) echo "  声学回环 880/1320Hz  : PASS" ;;
  2) echo "  声学回环 880/1320Hz  : FAIL" ;;
  *) echo "  声学回环 880/1320Hz  : NOT RUN" ;;
esac
[ "$RC_XRUN" = "0" ] && echo "  XRUN 恢复            : PASS" || echo "  XRUN 恢复            : FAIL"
[ "$RC_STOP" = "0" ] && echo "  SIGTERM 停止路径     : PASS" || echo "  SIGTERM 停止路径     : FAIL"

if [ "$RC_ENV" = "0" ] && [ "$RC_REC" = "0" ] && [ "$RC_ARG" = "0" ] && \
   [ "$RC_PLAY" = "0" ] && [ "$RC_LOOP" != "2" ] && \
   [ "$RC_XRUN" = "0" ] && [ "$RC_STOP" = "0" ]; then
  echo "== 总判定: PASS =="
  exit 0
fi
echo "== 总判定: FAIL =="
exit 1
