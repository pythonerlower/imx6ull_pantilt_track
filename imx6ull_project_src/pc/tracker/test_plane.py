#!/usr/bin/env python3
"""端到端联调：真实 daemon 进程 + 真实 ControlClient/TelemReceiver，走 UDP 环回。

被验的是三件在板上很难验、又最容易写错的事：

  1. KEEPALIVE 与 SET_TARGET 的分工。20 Hz 心跳必须既不断流、又不把
     cmd_count 稀释成节拍器——判据是"静止 1.5 s，cmd_count 只涨 1"。
  2. 驱动看门狗不会掐断正常的长距离移动。靠 daemon 每拍续命，
     138 tick（约 2.8 s，远超 1500 ms 看门狗）必须能走完。
  3. daemon 自身看门狗断流后确实 STOP，且恢复时能把 HOLD 解除
     （驱动进入 HOLD 后必须由用户态再下发一次，见 pantilt.c）。

不验的：电气特性、I2C 错误、真实时序抖动、舵机是否真的动了。
那些只能上板。fake_drv.c 只是内存里复刻钳位/速率/看门狗。

用法: python3 test_plane.py --daemon ../user/pantilt_daemon/pantilt_daemon_host
"""
from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import time

import ptproto as P
from control import ControlClient, TelemReceiver

EXE = ""
FAILED: list[str] = []
PASSED = 0

# 用非默认端口，避免和真的板端会话或别的测试撞车。
BASE_CTRL = 19100


def check(cond: bool, what: str) -> bool:
    global PASSED
    if cond:
        PASSED += 1
        print(f"  [ok] {what}")
    else:
        FAILED.append(what)
        print(f"  [FAIL] {what}")
    return cond


class Daemon:
    """起一个宿主版 daemon（--fake-driver），退出时确保进程不残留。"""

    def __init__(self, idx: int, *extra: str, env: dict | None = None):
        self.ctrl_port = BASE_CTRL + idx * 10
        self.telem_port = self.ctrl_port + 1
        self.args = [
            EXE, "--fake-driver",
            "--bind", "127.0.0.1",
            "--ctrl-port", str(self.ctrl_port),
            "--telem-port", str(self.telem_port),
            "-v",
            *extra,
        ]
        self.env = dict(os.environ)
        if env:
            self.env.update(env)
        self.proc: subprocess.Popen | None = None
        self.out: list[str] = []

    def __enter__(self) -> "Daemon":
        import threading

        self.proc = subprocess.Popen(self.args, env=self.env,
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True)
        # 持续排空 stdout。既避免管道写满把 daemon 卡住，也让失败时能回看日志。
        self._lock = threading.Lock()
        self._reader = threading.Thread(target=self._drain, daemon=True)
        self._reader.start()

        # 必须等 daemon 真的 bind 上控制口再返回。
        # 不等的话，第一个 UDP 包会打在还没监听的端口上被直接丢掉——
        # UDP 没有握手，丢了不会有任何提示，测试只会看到"没收到遥测"。
        if not wait_until(lambda: self.ready or self.proc.poll() is not None, 5.0):
            raise RuntimeError(f"daemon 5 秒内既未就绪也未退出：{self.tail()}")
        return self

    def _drain(self) -> None:
        for line in self.proc.stdout:
            with self._lock:
                self.out.append(line.rstrip("\n"))

    @property
    def ready(self) -> bool:
        with self._lock:
            return any("控制口 UDP" in ln for ln in self.out)

    def __exit__(self, *exc) -> None:
        if self.proc is None:
            return
        if self.proc.poll() is None:
            self.proc.send_signal(2)  # SIGINT → 走有序收尾
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=2)
        self._reader.join(timeout=1)
        # 收尾输出里带最终计数，失败时很有用。

    def tail(self, n: int = 25) -> str:
        with self._lock:
            return "\n".join(self.out[-n:])


class Raw:
    """直接造控制包。用它才能精确控制 seq、越界值、坏 CRC 这些情形。"""

    def __init__(self, port: int):
        self.dst = ("127.0.0.1", port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 100000

    def close(self) -> None:
        self.sock.close()

    def send(self, mode: int = P.MODE_MANUAL, pan: int = 377, tilt: int = 242,
             seq: int | None = None, flags: int = 0) -> int:
        if seq is None:
            self.seq += 1
            seq = self.seq
        pkt = P.Ctrl(seq=seq, mode=mode, timestamp_us=time.monotonic_ns() // 1000,
                     flags=flags, pan_tick=pan, tilt_tick=tilt).encode()
        self.sock.sendto(pkt, self.dst)
        return seq


def ctrl_bytes(seq: int, mode: int, pan: int, tilt: int) -> bytes:
    return P.Ctrl(seq=seq, mode=mode, timestamp_us=time.monotonic_ns() // 1000,
                  flags=0, pan_tick=pan, tilt_tick=tilt).encode()


def wait_telem(rx: TelemReceiver, timeout: float = 2.0) -> P.Telem:
    """等到有遥测为止。没等到就抛，避免后面用 None 去做断言。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        t, _ = rx.latest()
        if t is not None:
            return t
        time.sleep(0.02)
    raise RuntimeError(f"等了 {timeout}s 也没收到遥测")


def wait_until(fn, timeout: float, poll: float = 0.02) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if fn():
            return True
        time.sleep(poll)
    return False


# ------------------------------------------------------------------ 测试

def test_telem_stream_and_geometry():
    """遥测能连上、能解码，且带回来的安全范围与标定文件一致。

    范围这一条不是形式检查：daemon 用遥测里的 safe_min/safe_max 做前置拦截，
    如果它和 config/pantilt_calibration.conf 对不上，PC 侧整套限位判断都是错的。
    """
    print("\n[1] 遥测连通性与几何参数")
    with Daemon(1) as dm:
        rx = TelemReceiver("127.0.0.1", dm.telem_port)
        rx.start()
        raw = Raw(dm.ctrl_port)
        try:
            raw.send()
            t = wait_telem(rx)
            # 采样 1 秒估频率。期间必须持续供包：只发一包的话 500 ms 后
            # daemon 看门狗就会触发并进入 HOLD，量出来的就不是稳态了。
            n0 = rx.received
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.0:
                raw.send()
                time.sleep(0.05)
            t2, age = rx.latest()
            rate = (rx.received - n0)
            check(rate >= 15, f"遥测频率 ≥15 Hz（实测 {rate} 包/秒）")
            check(age < 200, f"最新遥测足够新（{age:.0f} ms）")
            check(rx.bad_pkts == 0, f"没有解不开的遥测包（坏 {rx.bad_pkts}）")

            check((t2.pan_safe_min, t2.pan_safe_max) == (239, 515),
                  f"pan 安全范围 {t2.pan_safe_min}..{t2.pan_safe_max} == 239..515")
            check((t2.tilt_safe_min, t2.tilt_safe_max) == (162, 342),
                  f"tilt 安全范围 {t2.tilt_safe_min}..{t2.tilt_safe_max} == 162..342")
            check((t2.pan_home, t2.tilt_home) == (377, 242),
                  f"home 为 (377, 242)，实为 ({t2.pan_home}, {t2.tilt_home})")
            check(t2.slew_ms == 20, f"标称 slew_ms=20，实为 {t2.slew_ms}")
            check(t2.daemon_wd_ms == 500,
                  f"daemon 看门狗 500 ms，实为 {t2.daemon_wd_ms}")
            check(t2.drv_status in (P.DRV_READY, P.DRV_SLEW),
                  f"驱动已就绪（status={P.DRV_STATUS_NAME.get(t2.drv_status)})")
        finally:
            raw.close()
            rx.stop()


def test_keepalive_does_not_inflate_cmd_count():
    """静止 1.5 s：目标只下发一次，心跳不许污染 cmd_count。

    这正是本次给驱动加 KEEPALIVE 的全部理由。若这条失败，说明有人在用
    重复 SET_TARGET 当心跳——那样 cmd_count 会变成节拍器，status 会抖动。
    """
    print("\n[2] KEEPALIVE 不污染 cmd_count")
    with Daemon(2) as dm:
        rx = TelemReceiver("127.0.0.1", dm.telem_port)
        rx.start()
        cli = ControlClient("127.0.0.1", dm.ctrl_port)
        try:
            cli.start()
            cli.set_target(P.MODE_MANUAL, 400, 250, immediate=True)
            t = wait_telem(rx)
            base = t.cmd_count
            seq0 = t.seq
            time.sleep(1.5)
            t2, _ = rx.latest()
            grew = t2.cmd_count - base
            check(grew == 0, f"1.5 s 静止期间 cmd_count 未增长（+{grew}）")
            check(t2.seq - seq0 >= 25,
                  f"遥测仍在流动（seq +{t2.seq - seq0}）")
            check(t2.ctrl_rx_ok - t.ctrl_rx_ok >= 25,
                  f"心跳被接受（ctrl_rx_ok +{t2.ctrl_rx_ok - t.ctrl_rx_ok}）")
        finally:
            cli.stop()
            rx.stop()


def test_long_slew_survives_driver_watchdog():
    """138 tick 的长移动必须走完——这是"续命能防看门狗掐断"的直接证据。

    距离 377→239 = 138 tick，按 slew_ms=20 计约 2.8 s，远超 1500 ms 看门狗。
    期间 watchdog_trips 必须保持 0，且最终 cur 要落到目标上。
    tick-hz 提到 50 是为了让模拟驱动有足够分辨率按 20 ms 推进。
    """
    print("\n[3] 长距离移动不被驱动看门狗掐断")
    with Daemon(3, "--tick-hz", "50") as dm:
        rx = TelemReceiver("127.0.0.1", dm.telem_port)
        rx.start()
        raw = Raw(dm.ctrl_port)
        try:
            raw.send(pan=377, tilt=242)
            t = wait_telem(rx)
            trips0 = t.watchdog_trips
            # 长距离移动期间必须持续供包（等价于客户端 20 Hz 心跳）。
            tgt_pan, tgt_tilt = 239, 242
            t0 = time.monotonic()
            arrived = False
            while time.monotonic() - t0 < 8.0:
                raw.send(pan=tgt_pan, tilt=tgt_tilt)
                t, _ = rx.latest()
                if t and t.pan_cur == tgt_pan and t.tilt_cur == tgt_tilt:
                    arrived = True
                    break
                time.sleep(0.05)
            t, _ = rx.latest()
            check(arrived,
                  f"跨 138 tick 走到目标（cur={t.pan_cur} 目标={tgt_pan}）")
            check(t.watchdog_trips == trips0,
                  f"期间驱动看门狗未触发（{trips0} → {t.watchdog_trips}）")
            check(t.pan_cur >= t.pan_safe_min,
                  f"终点仍在安全范围内（pan={t.pan_cur} ≥ {t.pan_safe_min}）")
        finally:
            raw.close()
            rx.stop()


def test_out_of_range_rejected_before_driver():
    """越界目标必须在 daemon 就被拦下，不能靠驱动钳位兜底。

    驱动当然也会钳位，但那样 clamp_low/high 会被无意义的请求刷屏，
    真实的越界事故就淹没在噪声里了。所以要求：rej_range 增长、cmd_count 不动。
    """
    print("\n[4] 越界目标在 daemon 侧被拒")
    with Daemon(4) as dm:
        rx = TelemReceiver("127.0.0.1", dm.telem_port)
        rx.start()
        raw = Raw(dm.ctrl_port)
        try:
            raw.send(pan=377, tilt=242)
            t = wait_telem(rx)
            cmd0, rej0 = t.cmd_count, t.rej_range
            pos0 = (t.pan_cur, t.tilt_cur)

            raw.send(pan=600, tilt=242)      # pan 超出 safe_max 515
            raw.send(pan=377, tilt=50)       # tilt 低于 safe_min 162
            time.sleep(0.4)
            t2, _ = rx.latest()
            check(t2.rej_range - rej0 >= 2,
                  f"两条越界包都被记为 rej_range（+{t2.rej_range - rej0}）")
            check(t2.cmd_count == cmd0,
                  f"越界包没有走到驱动（cmd_count {cmd0} → {t2.cmd_count}）")
            check((t2.pan_cur, t2.tilt_cur) == pos0,
                  f"云台没动（{pos0} → {(t2.pan_cur, t2.tilt_cur)}）")
            check(t2.clamp_low_pan + t2.clamp_low_tilt
                  + t2.clamp_high_pan + t2.clamp_high_tilt == 0,
                  "驱动的钳位计数保持 0（说明拦截确实在 daemon 完成）")
        finally:
            raw.close()
            rx.stop()


def test_corrupt_and_duplicate_packets_counted():
    """坏 CRC、重复 seq 必须分别计数，且不推动云台。"""
    print("\n[5] 坏 CRC 与重复 seq 被分类计数")
    with Daemon(5) as dm:
        rx = TelemReceiver("127.0.0.1", dm.telem_port)
        rx.start()
        raw = Raw(dm.ctrl_port)
        try:
            s = raw.send(pan=377, tilt=242)
            t = wait_telem(rx)
            cmd0 = t.cmd_count
            crc0, seq0 = t.rej_crc, t.rej_seq

            good = ctrl_bytes(s + 1, P.MODE_MANUAL, 380, 242)
            bad = bytearray(good)
            bad[10] ^= 0xFF  # 落在 payload 里，CRC 必然不符
            raw.sock.sendto(bytes(bad), raw.dst)

            raw.send(seq=s + 1, pan=380, tilt=242)  # 合法，推进一次
            raw.send(seq=s + 1, pan=380, tilt=242)  # 同 seq，应被拒
            time.sleep(0.4)
            t2, _ = rx.latest()
            check(t2.rej_crc - crc0 >= 1,
                  f"坏 CRC 被记为 rej_crc（+{t2.rej_crc - crc0}）")
            check(t2.rej_seq - seq0 >= 1,
                  f"重复 seq 被记为 rej_seq（+{t2.rej_seq - seq0}）")
            check(t2.cmd_count - cmd0 >= 1,
                  f"合法的那条仍然下达（cmd_count +{t2.cmd_count - cmd0}）")
        finally:
            raw.close()
            rx.stop()


def test_seq_resync_after_client_restart():
    """客户端重启导致 seq 大幅倒退时，daemon 必须自愈而不是永久失聪。

    若不实现这条，表现会是"云台再也不动，而两端都显示正常"——极难排查。
    判据：倒退包先被记 rej_seq，持续超过 1 s 后开始被接受（ctrl_rx_ok 增长）。
    """
    print("\n[6] seq 大幅倒退后自动重置会话")
    with Daemon(6) as dm:
        rx = TelemReceiver("127.0.0.1", dm.telem_port)
        rx.start()
        raw = Raw(dm.ctrl_port)
        try:
            raw.send(seq=500000, pan=377, tilt=242)
            wait_telem(rx)
            time.sleep(0.2)
            t = rx.latest()[0]
            ok0, rej0 = t.ctrl_rx_ok, t.rej_seq

            # 模拟 PC 重启：seq 从 500000 掉到 10。
            t0 = time.monotonic()
            while time.monotonic() - t0 < 2.0:
                raw.send(seq=10 + int((time.monotonic() - t0) * 20),
                         pan=377, tilt=242)
                time.sleep(0.05)
            t2, _ = rx.latest()
            check(t2.rej_seq - rej0 >= 5,
                  f"倒退的包先被拒（rej_seq +{t2.rej_seq - rej0}）")
            check(t2.ctrl_rx_ok - ok0 >= 5,
                  f"持续倒退后被判为新会话并接受（ctrl_rx_ok +{t2.ctrl_rx_ok - ok0}）")
        finally:
            raw.close()
            rx.stop()


def test_daemon_watchdog_stops_then_hold_is_reissued():
    """断流 → daemon 主动 STOP；恢复时同一目标必须能重新下达（解除 HOLD）。

    后半段是驱动的硬约束：看门狗触发后驱动进 HOLD，必须由用户态再下发一次
    才恢复。若 daemon 只做"值变了才发"的去重，客户端恢复后云台会永久停住。
    """
    print("\n[7] daemon 看门狗 STOP 与 HOLD 解除")
    with Daemon(7) as dm:
        rx = TelemReceiver("127.0.0.1", dm.telem_port)
        rx.start()
        raw = Raw(dm.ctrl_port)
        try:
            raw.send(pan=400, tilt=250)
            t = wait_telem(rx)
            check(wait_until(lambda: rx.latest()[0].pan_cur == 400, 4.0),
                  "先走到 400 作为起点")
            t = rx.latest()[0]
            wd0, cmd0 = t.daemon_wd_trips, t.cmd_count

            # 断流 900 ms（> 500 ms 看门狗），期间不再发任何包。
            time.sleep(0.9)
            t2, _ = rx.latest()
            check(t2.daemon_wd_trips > wd0,
                  f"断流后 daemon 看门狗触发（{wd0} → {t2.daemon_wd_trips}）")
            check(t2.drv_status == P.DRV_HOLD,
                  f"驱动进入 HOLD（status={t2.drv_status}）")
            check(t2.daemon_state == P.DST_LOST,
                  f"daemon 状态为 LOST（{t2.daemon_state}）")

            # 恢复：发**与之前完全相同**的目标，仅 seq 递增。
            held = 400
            t3 = None
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.0:
                raw.send(pan=held, tilt=250)
                time.sleep(0.05)
            t3, _ = rx.latest()
            check(t3.cmd_count > cmd0,
                  f"目标未变但仍重新下达以解除 HOLD（cmd_count {cmd0} → {t3.cmd_count}）")
            check(t3.daemon_state == P.DST_MANUAL,
                  f"恢复后 daemon 离开 LOST（state={t3.daemon_state}）")
            check(t3.drv_status != P.DRV_HOLD,
                  f"驱动已脱离 HOLD（status={P.DRV_STATUS_NAME.get(t3.drv_status)}）")
        finally:
            raw.close()
            rx.stop()


def test_refuses_unclaimed_driver():
    """驱动未认领（UNINIT）时必须拒绝启动。

    SET_LIMIT 会唤醒 PCA9685 并让舵机真的转动，那是必须人工确认的物理动作。
    daemon 若在这里"顺手代做"，就会在无人看守时让云台自己动起来。
    """
    print("\n[8] 未认领的驱动上拒绝启动")
    with Daemon(8, env={"PANTILT_FAKE_UNINIT": "1"}) as dm:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0 and dm.proc.poll() is None:
            time.sleep(0.05)
        rc = dm.proc.poll()
        check(rc == 3, f"退出码为 3（实为 {rc}）")
        out = dm.tail(30)
        check("尚未认领" in out or "UNINIT" in out,
              "输出里说明了拒绝理由是未认领")


def main() -> int:
    global EXE
    ap = argparse.ArgumentParser()
    ap.add_argument("--daemon", required=True,
                    help="宿主版 daemon 可执行文件（--fake-driver 版）")
    args = ap.parse_args()
    EXE = os.path.abspath(args.daemon)
    if not (os.path.isfile(EXE) and os.access(EXE, os.X_OK)):
        print(f"找不到可执行的 daemon：{EXE}", file=sys.stderr)
        print("先跑：make -C user/pantilt_daemon pantilt_daemon_host", file=sys.stderr)
        return 2

    print(f"用 {EXE} 做端到端联调（--fake-driver，不碰硬件）")
    tests = [
        test_telem_stream_and_geometry,
        test_keepalive_does_not_inflate_cmd_count,
        test_long_slew_survives_driver_watchdog,
        test_out_of_range_rejected_before_driver,
        test_corrupt_and_duplicate_packets_counted,
        test_seq_resync_after_client_restart,
        test_daemon_watchdog_stops_then_hold_is_reissued,
        test_refuses_unclaimed_driver,
    ]
    for fn in tests:
        try:
            fn()
        except Exception as e:  # 单条测试炸了不该带崩整轮
            FAILED.append(f"{fn.__name__}: {type(e).__name__}: {e}")
            print(f"  [ERROR] {fn.__name__}: {type(e).__name__}: {e}")

    print(f"\n通过 {PASSED} 项，失败 {len(FAILED)} 项")
    for f in FAILED:
        print(f"  - {f}")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
