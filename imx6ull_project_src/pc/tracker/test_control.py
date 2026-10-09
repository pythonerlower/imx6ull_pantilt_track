"""control.py 的回环单测：本机起一个"假 daemon"收包，验证心跳与 seq。

要证伪的两件事，都是看代码看不出来的：
  1. 心跳是否真的恒定在 20 Hz —— 看门狗方案全靠它；"有变化才发"的写法
     在静止时会静默停发，而代码读起来完全正常。
  2. seq 是否严格单调 —— 板端按 seq 判重/判过期，一旦回绕或倒退，
     指令会被整批拒绝，表现为"云台不动"，而客户端日志显示发送成功。

    python3 test_control.py
"""

from __future__ import annotations

import socket
import threading
import time

import ptproto as P
from control import ControlClient


class FakeDaemon:
    """只收包、只记录，不做任何控制决策。"""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.1)
        self.port = self.sock.getsockname()[1]
        self.pkts: list[tuple[float, P.Ctrl]] = []
        self.bad = 0
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                self.pkts.append((time.monotonic(), P.Ctrl.decode(data)))
            except ValueError:
                self.bad += 1

    def stop(self):
        self._stop.set()
        self._th.join(timeout=1.0)
        self.sock.close()


def test_heartbeat_rate_is_steady():
    d = FakeDaemon()
    c = ControlClient("127.0.0.1", d.port, rate_hz=20.0)
    try:
        c.start()
        time.sleep(1.0)
        n = len(d.pkts)
        assert d.bad == 0, f"有 {d.bad} 个包解不开"
        # 1 秒 @20 Hz 应收到约 20 个。放宽容差只抓"根本没在发"或"发疯了"。
        assert 14 <= n <= 26, f"1 秒收到 {n} 个包，偏离 20 Hz 太多"
        span = d.pkts[-1][0] - d.pkts[0][0]
        rate = (n - 1) / span if span > 0 else 0
        print(f"  [ok] 心跳恒定：1 秒收到 {n} 包，实测 {rate:.1f} Hz")
    finally:
        c.stop(send_stop=False)
        d.stop()


def test_seq_is_strictly_increasing():
    d = FakeDaemon()
    c = ControlClient("127.0.0.1", d.port, rate_hz=50.0)
    try:
        c.start()
        # 发一串改动，检查改动不会打乱 seq 的单调性。
        for i in range(5):
            c.set_target(P.MODE_MANUAL, pan_tick=350 + i, tilt_tick=242,
                         immediate=True)
            time.sleep(0.02)
        time.sleep(0.2)
        seqs = [p.seq for _, p in d.pkts]
        assert seqs, "一个包都没收到"
        # 允许 32 位回绕：只要求相邻差的模 2^32 增量为正且很小。
        for a, b in zip(seqs, seqs[1:]):
            delta = (b - a) & 0xFFFFFFFF
            assert 0 < delta < 0x80000000, f"seq 非单调：{a} → {b}"
        print(f"  [ok] seq 严格单调（{len(seqs)} 包，{seqs[0]} → {seqs[-1]}）")
    finally:
        c.stop(send_stop=False)
        d.stop()


def test_heartbeat_flag_marks_unchanged_packets():
    d = FakeDaemon()
    c = ControlClient("127.0.0.1", d.port, rate_hz=50.0)
    try:
        c.start()
        c.set_target(P.MODE_MANUAL, pan_tick=400, tilt_tick=222, immediate=True)
        time.sleep(0.3)  # 之后全是重发同一目标，应带心跳位
        c.stop(send_stop=False)
        time.sleep(0.05)

        changed_pkts = [p for _, p in d.pkts if not (p.flags & P.FLAG_HEARTBEAT)]
        hb_pkts = [p for _, p in d.pkts if p.flags & P.FLAG_HEARTBEAT]
        assert len(changed_pkts) >= 1, "改动目标的那一包没有被标成'非心跳'"
        assert len(hb_pkts) >= 5, f"重发的包没有标心跳（只有 {len(hb_pkts)} 个）"
        # 改目标的那包必须是值正确的那包。
        assert changed_pkts[0].pan_tick == 400 and changed_pkts[0].tilt_tick == 222, \
            changed_pkts[0]
        print(f"  [ok] 心跳位标注正确（改动 {len(changed_pkts)} 包，心跳 {len(hb_pkts)} 包）")
    finally:
        d.stop()


def test_stop_sends_final_hold():
    d = FakeDaemon()
    c = ControlClient("127.0.0.1", d.port, rate_hz=50.0)
    try:
        c.start()
        c.set_target(P.MODE_MANUAL, pan_tick=377, tilt_tick=242, immediate=True)
        time.sleep(0.1)
        c.stop(send_stop=True)  # 默认行为：退出前补发 STOP
        time.sleep(0.05)
        last = d.pkts[-1][1]
        assert last.mode == P.MODE_STOP, \
            f"退出前最后一包不是 STOP，而是模式 {P.DAEMON_STATE_NAME.get(last.mode)}"
        assert last.pan_tick == 377 and last.tilt_tick == 242, \
            "STOP 包必须带上当前姿态，否则板端不知道保持在哪"
        print(f"  [ok] 退出前补发 STOP，且带上了当前姿态 "
              f"({last.pan_tick}, {last.tilt_tick})")
    finally:
        d.stop()


def test_immediate_send_is_prompt():
    d = FakeDaemon()
    c = ControlClient("127.0.0.1", d.port, rate_hz=1.0)  # 心跳很慢
    try:
        c.start()
        time.sleep(0.05)
        t0 = time.monotonic()
        # 心跳周期 1 秒，若 immediate 没生效，这里会等到下一个周期才收得到。
        c.set_target(P.MODE_MANUAL, pan_tick=300, tilt_tick=242, immediate=True)
        deadline = t0 + 0.3
        got = None
        while time.monotonic() < deadline:
            if any(p.pan_tick == 300 for _, p in d.pkts):
                got = time.monotonic() - t0
                break
            time.sleep(0.005)
        assert got is not None, "immediate=True 没有立刻发出，点动会发黏"
        print(f"  [ok] immediate=True 立刻送达（{got * 1000:.1f} ms，心跳周期 1000 ms）")
    finally:
        c.stop(send_stop=False)
        d.stop()


def main() -> int:
    print("control 回环单测")
    tests = [
        test_heartbeat_rate_is_steady,
        test_seq_is_strictly_increasing,
        test_heartbeat_flag_marks_unchanged_packets,
        test_stop_sends_final_hold,
        test_immediate_send_is_prompt,
    ]
    for t in tests:
        try:
            t()
        except AssertionError as exc:
            print(f"  [FAIL] {t.__name__}: {exc}")
            return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
