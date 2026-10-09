"""控制面：PC→板端的指令（+ 恒定心跳）与板端→PC 的遥测。

心跳不是可选项。已定的方案是"客户端恒定 20 Hz 心跳 + daemon 侧 500 ms 看门狗"：
板端在 500 ms 内没收到控制包，就主动下发 STOP 保持位置（不是断电、不是松力，
而是停在原地）。这条规则的意义是**客户端一崩，云台不会继续朝最后的方向跑**。

所以本模块即使在"操作者什么都没按"的时候也必须持续发同一份目标值——
不能写成"有变化才发"，那样静止三秒就会被看门狗接管。

seq 的起点刻意取自 time.monotonic()：板端会用 seq 判重和判过期，
如果客户端每次启动都从 0 开始，而 daemon 还在运行，就会被当成"seq 倒退"全数拒绝。
用单调时钟做种子，客户端重启后 seq 自然继续增长。
"""

from __future__ import annotations

import socket
import threading
import time

import ptproto as P

# 看到问：为什么不直接 "目标不变就不发"？
# 因为 daemon 的看门狗只看"距最后一条被接受的控制包多久"，不看值有没有变。
# 恒发是让看门狗有稳定时间基准的唯一办法，这也正是 PTP_FLAG_HEARTBEAT 的用途。
DEFAULT_RATE_HZ = 20.0


class ControlClient:
    """向板端 daemon 发控制包，并由后台线程维持恒定心跳。

    线程模型：所有状态改动都在锁内，心跳线程每 1/rate 秒取一次最新状态发出去。
    这样调用方（main.py / 操作者按键）不需要自己把控节奏。
    """

    def __init__(self, host: str, port: int = P.CTRL_PORT,
                 rate_hz: float = DEFAULT_RATE_HZ):
        self.host = host
        self.port = port
        self.rate_hz = rate_hz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dst = (host, port)

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        # 起始值从单调时钟取，保证客户端重启后 seq 仍单调增长（见模块说明）。
        self._seq = int(time.monotonic() * 1000) & 0xFFFFFFFF
        self._mode = P.MODE_HOLD
        self._pan = 0
        self._tilt = 0
        self._last_change_seq = 0
        self.sent = 0
        self.send_err = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="ctrl-heartbeat")
        self._thread.start()

    def stop(self, send_stop: bool = True) -> None:
        """停心跳。默认先补发一条 STOP，让云台立刻进入保持而不是等看门狗超时。"""
        if send_stop:
            try:
                self.set_target(P.MODE_STOP, self._pan, self._tilt, immediate=True)
            except OSError:
                pass
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self.sock.close()

    def set_target(self, mode: int, pan_tick: int | None = None,
                   tilt_tick: int | None = None, immediate: bool = False) -> int:
        """更新目标。pan/tilt 传 None 表示沿用上一次的值（改模式时用）。

        返回本条指令的 seq。immediate=True 时立刻发一包，不等下一个心跳周期——
        手动点动靠它，否则延迟会长达一个心跳周期（50 ms），手感发黏。
        """
        with self._lock:
            if pan_tick is not None:
                self._pan = int(pan_tick) & 0xFFFF
            if tilt_tick is not None:
                self._tilt = int(tilt_tick) & 0xFFFF
            changed = mode != self._mode or pan_tick is not None or tilt_tick is not None
            self._mode = mode
            self._seq = (self._seq + 1) & 0xFFFFFFFF
            seq = self._seq
            if changed:
                self._last_change_seq = seq
            pkt = self._encode_locked(seq, changed)
        if immediate or changed:
            self._send(pkt)
        return seq

    def current_target(self) -> tuple[int, int, int]:
        """返回 (mode, pan, tilt)：客户端认定的当前目标。

        点动要以它为基准累加，而不是以遥测里的 pan_tgt —— 后者是驱动侧的
        目标，落后于客户端的请求，连按时会把余量吃掉，表现为越点越慢。
        """
        with self._lock:
            return self._mode, self._pan, self._tilt

    def _encode_locked(self, seq: int, changed: bool) -> bytes:
        # 心跳位只是给遥测统计用的标签；板端按 (mode, pan, tilt) 的值去重，
        # 不依赖这一位，所以漏标也不会污染驱动的命令计数。
        hb = 0 if changed else P.FLAG_HEARTBEAT
        return P.Ctrl(
            seq=seq,
            mode=self._mode,
            timestamp_us=time.monotonic_ns() // 1000,
            flags=hb,
            pan_tick=self._pan,
            tilt_tick=self._tilt,
        ).encode()

    def _send(self, pkt: bytes) -> None:
        try:
            self.sock.sendto(pkt, self.dst)
            self.sent += 1
        except OSError:
            self.send_err += 1

    def _loop(self) -> None:
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.05
        deadline = time.monotonic()
        while not self._stop.is_set():
            with self._lock:
                self._seq = (self._seq + 1) & 0xFFFFFFFF
                # 与上一包值相同的，就是心跳包。
                pkt = self._encode_locked(self._seq, changed=False)
            self._send(pkt)
            deadline += period
            nap = deadline - time.monotonic()
            if nap < 0:
                # 落后了就重置基准，不要让欠账累积成一次突发。
                deadline = time.monotonic()
                nap = 0
            self._stop.wait(nap)


class TelemReceiver:
    """接收板端遥测，后台线程持续更新 latest()。

    刻意不做成阻塞式 recv：跟踪器的主循环要以固定频率跑，不能被"等遥测"卡住。
    要的是"随时能拿到最新一份"，而不是"取走一份"。
    """

    def __init__(self, bind: str = "0.0.0.0", port: int = P.TELEM_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind, port))
        self.sock.settimeout(0.2)

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest: P.Telem | None = None
        self._latest_us = 0
        self.bad_pkts = 0
        self.received = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="telem-rx")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self.sock.close()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                pkt, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                t = P.Telem.decode(pkt)
            except ValueError:
                self.bad_pkts += 1
                continue
            with self._lock:
                self._latest = t
                self._latest_us = time.monotonic_ns() // 1000
                self.received += 1

    def latest(self) -> tuple[P.Telem | None, float]:
        """返回 (最新遥测, 距收到它的毫秒数)。没收到过时返回 (None, inf)。"""
        with self._lock:
            t, us = self._latest, self._latest_us
        if t is None:
            return None, float("inf")
        return t, (time.monotonic_ns() // 1000 - us) / 1000.0
