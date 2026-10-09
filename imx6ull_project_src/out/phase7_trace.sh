#!/bin/bash
# Phase 7 ③：两条 ftrace 链路的采集脚本（plan/phase7_wom_patrol_debug.md §2.3）。
#
#   irq   —— irq_handler_entry / irq_handler_exit，只看与本项目有关的 IRQ。
#            Phase 7 走的是**软件阈值**路线（ICM20608 的 INT 脚挂不上中断，见
#            logs/phase7/m1_icm_int_probe_20261008.md），所以这里真正会跳的是
#            AP3216C 的 gpio-mxc 那一路，以及 pantilt 驱动自己的定时器回调。
#            留着 irq 这一路是为了证明"阈值轮询期间没有任何新的中断源冒出来"。
#   sched —— sched_switch，只看 pantilt_daemon / securityd / icm20608_iio 这几个
#            comm 的进出。用来回答"daemon 的 20 Hz 节拍有没有被别的进程挤掉"，
#            以及告警时 securityd 起子进程那一下有没有把 daemon 饿着。
#
# 判据是**日志里的数字**，不是"看起来没卡"：
#   irq   → 区间内 IRQ 条数（treated as 只有 ap3216c 那一路会动）
#   sched → 每个 comm 的切换次数与最大一段连续不在 CPU 上的时长
#
# 用法（板端 root）：
#   sh phase7_trace.sh irq   [秒数] [输出文件]
#   sh phase7_trace.sh sched [秒数] [输出文件]
#   sh phase7_trace.sh both  [秒数] [输出文件]
#
# 注意：本脚本**只读 tracer**，不碰 DTS、不碰驱动、不动云台。

set -u

MODE="${1:-both}"
DURATION="${2:-10}"
OUT="${3:-/tmp/phase7_trace_${MODE}.txt}"

TRACE=/sys/kernel/debug/tracing
[ -d "$TRACE" ] || { echo "[skip] $TRACE 不存在：内核没开 CONFIG_FTRACE"; exit 2; }

# 有些内核把 tracing 挂在 /sys/kernel/tracing 下
[ -d "$TRACE/events" ] || { echo "[skip] $TRACE 不是 tracing 目录"; exit 2; }

# 过滤用的进程名。板端进程名会被内核截到 15 字符（TASK_COMM_LEN-1），
# 所以这里必须用**截断后的名字**匹配，否则过滤器一个都匹配不上，
# 表现得像"那几个进程一次都没被调度"——完全是假的。
COMMS="pantilt_daemon securityd icm20608_iio"

cleanup() {
	echo 0 > "$TRACE/tracing_on" 2>/dev/null
	echo > "$TRACE/set_event" 2>/dev/null
	echo > "$TRACE/set_ftrace_filter" 2>/dev/null
	echo nop > "$TRACE/current_tracer" 2>/dev/null
}
trap cleanup EXIT

echo 0 > "$TRACE/tracing_on"
echo > "$TRACE/trace"
echo nop > "$TRACE/current_tracer"
echo > "$TRACE/set_event"
echo > "$TRACE/set_ftrace_filter"

ok=0
case "$MODE" in
  irq|both)
	if [ -d "$TRACE/events/irq/irq_handler_entry" ]; then
		echo 1 > "$TRACE/events/irq/irq_handler_entry/enable"
		echo 1 > "$TRACE/events/irq/irq_handler_exit/enable"
		ok=1
	else
		echo "[warn] 没有 irq_handler_entry：内核未开 CONFIG_IRQSOFF_TRACER/FTRACE_IRQ"
	fi
	;;
esac
case "$MODE" in
  sched|both)
	if [ -d "$TRACE/events/sched/sched_switch" ]; then
		# 过滤器里 comm 要加引号；不加引号时带下划线的名字会被当成标识符。
		FILTER=""
		for c in $COMMS; do
			[ -n "$FILTER" ] && FILTER="$FILTER || "
			FILTER="${FILTER}prev_comm == \"$c\" || next_comm == \"$c\""
		done
		echo "$FILTER" > "$TRACE/events/sched/sched_switch/filter" || \
			echo "[warn] sched_switch filter 写不进去，将采全部调度事件（量大）"
		echo 1 > "$TRACE/events/sched/sched_switch/enable"
		ok=1
	else
		echo "[warn] 没有 sched_switch 事件"
	fi
	;;
esac
[ "$ok" = 1 ] || { echo "[skip] 该模式下没有任何可用事件"; exit 2; }

echo "== 采集 $DURATION s（mode=$MODE），期间请在板上做你的事（碰一下模块/跑一轮链）"
echo 1 > "$TRACE/tracing_on"
sleep "$DURATION"
echo 0 > "$TRACE/tracing_on"

cat "$TRACE/trace" > "$OUT"
lines=$(wc -l < "$OUT")

echo "== 写入 $OUT（$lines 行）"
if [ "$MODE" != "sched" ]; then
	# 按 handler 名统计。逐条打出来是为了**能看见到底是谁在跳**，
	# 只给一个总数的话，"多出来的那一路是谁"就没法回答。
	echo "-- IRQ 按 handler 统计："
	sed -n 's/.*irq_handler_entry: irq=\([0-9]*\) name=\([^ ]*\).*/\2 irq=\1/p' \
		"$OUT" | sort | uniq -c | sort -rn
fi
if [ "$MODE" != "irq" ]; then
	echo "-- 每个 comm 参与的切换次数："
	for c in $COMMS; do
		n=$(grep -c -- "==> .*\b$c\b\|$c.*==>" "$OUT" 2>/dev/null || true)
		printf '   %-16s %s\n' "$c" "${n:-0}"
	done
fi
if [ "$lines" -eq 0 ]; then
	echo "   注意：0 行。要么这 $DURATION s 里目标进程真的没跑，要么进程名对不上"
	echo "   （内核把 comm 截到 15 字符）。先确认板端进程名：ps -eo comm"
fi
