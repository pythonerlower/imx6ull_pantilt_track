"""UDP 视频接收与整帧重组（PC 侧）。

两个用途共用一份实现：
  * 独立运行 —— 当链路测量工具用，量真实丢包率（plan §9 第 3 项）；
  * 被 main.py import —— 作为 3C 跟踪器的取帧接口（recv_frame）。

为什么要重组而不是"收到就用"：板端把一帧 320x240 YUYV（153600 字节）切成 110 个
1400 字节的 UDP 包。丢任意一片，整帧作废——半帧图像交给 OpenCV 只会得到
一块错位的乱码，比没有帧更糟（会把跟踪器带到完全错误的方向）。

丢包在数据里是查不到的（UDP 不会告诉你丢了什么），所以只能靠 chunk_idx 的缺口
来判断。这里不做重传、不做 FEC：规则要求的是"采集不因发送阻塞"，板端宁可丢帧
也不阻塞，那么 PC 侧就必须老实承认丢帧，而不是掩盖它。

实测提醒：本项目的链路是 VMware 桥接网卡，ICMP 表现异常（见 plan §9 第 3 项），
所以丢包率必须用真实视频流量测，不能拿 ping 的结果当结论。
"""

from __future__ import annotations

import argparse
import errno
import os
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field

import ptproto as P


def _now_us() -> int:
    return time.monotonic_ns() // 1000


@dataclass
class Frame:
    frame_id: int
    timestamp_us: int
    width: int
    height: int
    format: int
    data: bytes
    complete: bool
    chunks_got: int
    chunks_total: int
    received_us: int

    @property
    def is_yuyv(self) -> bool:
        return self.format == P.FMT_YUYV

    def transport_ms(self) -> float:
        """板端采集时刻到本机收齐时刻，单位毫秒。两地时钟不同源，只能算耗时差。"""
        return (self.received_us - self.timestamp_us) / 1000.0


@dataclass
class RxStats:
    datagrams: int = 0
    bytes: int = 0
    crc_err: int = 0
    bad_magic: int = 0
    short_pkts: int = 0            # 短于视频头，通常是串端口的别的报文
    dup_chunks: int = 0
    late_chunks: int = 0          # 属于"已结算帧"的迟到分片，见 VideoReceiver._settled
    reorder_chunks: int = 0
    frames_complete: int = 0
    frames_incomplete: int = 0
    frames_lost: int = 0          # frame_id 的跳号数，**不等于"丢了"**，见下
    # 跳号有三种成因，PC 侧**无法**区分，只能靠板端 [stats] 分辨：
    #   1. 板端 `--fps` 主动节流丢掉（板端 节流丢 计数器）
    #   2. 板端环缓冲被覆盖（板端 覆盖丢 计数器）
    #   3. 整帧的分片全丢在网络里（板端 采集/入环/出环 都对得上，PC 侧凭空少一帧）
    # 所以统计行里叫它"跳号"而不是"丢失"——叫丢失会把 1 说成故障。
    chunks_expected: int = 0      # 已结算帧理应含有的分片总数
    chunks_received: int = 0      # 其中真正收到的分片数
    last_frame_id: int | None = None
    first_us: int = 0
    last_us: int = 0

    def rate(self) -> float:
        span = (self.last_us - self.first_us) / 1e6
        return self.frames_complete / span if span > 0 else 0.0

    def chunk_loss_pct(self) -> float:
        """按片计的丢包率。

        这是链路质量最直接的指标：按帧算会把"丢 1 片"和"丢 100 片"都记成
        一个残缺帧，而按片算能看出链路到底是偶尔漏还是已经撑不住。
        只统计已结算（完整或放弃）的帧，进行中的帧不计入分母。
        """
        if self.chunks_expected == 0:
            return 0.0
        lost = self.chunks_expected - self.chunks_received
        return lost / self.chunks_expected * 100.0


class VideoReceiver:
    """收视频包、按 frame_id 重组整帧。

    窗口语义：最多同时挂着 window 个未完成的帧。这样丢了一片的那一帧不会
    永远占着内存等它——等更新的帧号超过窗口，它就作为"不完整帧"被丢弃并计数。
    """

    def __init__(self, port: int = P.VIDEO_PORT, bind: str = "0.0.0.0",
                 window: int = 3, rcvbuf: int = 4 * 1024 * 1024):
        self.port = port
        self.window = window
        self.stats = RxStats()
        self._pending: dict[int, dict[int, bytes]] = {}
        self._meta: dict[int, P.VideoHdr] = {}
        self._order: list[int] = []
        self._max_id: int | None = None
        # 已结算（完整或放弃）的帧号。存在的理由是链路上的**重复投递**：
        # 一帧结算后 _pending/_meta 里就没有它了，此时它迟到的重复包会被
        # 上面 `if fid not in self._pending` 当成"新帧"重新建槽，于是同一帧
        # 被反复结算成"残缺"，frames_incomplete 与 chunk_loss_pct 虚高。
        # 2026-10-07 板端实测就撞到了：板端只发了 580 帧，接收端却报
        # 580 完整 + 193 残缺（那 193 全是复活出来的）。
        # 记过的帧号一律不再重建——帧一旦交付，迟到的片对我们就只是噪声。
        self._settled: set[int] = set()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        except OSError:
            pass
        self.sock.bind((bind, port))
        # 记下内核真正给了多少：它会把请求值翻倍，且受 net.core.rmem_max 限制。
        # 一帧 110 个包在 15 fps 下是 1650 包/秒，突发时缓冲不够就会内核丢包，
        # 那种丢包连 datagrams 计数都看不到，必须知道实际值才能判断。
        self.rcvbuf = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)

    def close(self) -> None:
        self.sock.close()

    def __enter__(self) -> "VideoReceiver":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _ingest(self, pkt: bytes) -> None:
        self.stats.datagrams += 1
        self.stats.bytes += len(pkt)
        now = _now_us()
        if not self.stats.first_us:
            self.stats.first_us = now
        self.stats.last_us = now

        # 长度先单独判：短于视频头的包连魔数都读不出来，多半是别的端口串过来的
        # （控制包是 32 字节）。把它归到"坏 CRC"会掩盖真正的链路错误率。
        if len(pkt) < P.VIDEO_HDR_SIZE:
            self.stats.short_pkts += 1
            return
        try:
            hdr = P.VideoHdr.decode(pkt)
        except ValueError as exc:
            if "魔数" in str(exc):
                self.stats.bad_magic += 1
            else:
                self.stats.crc_err += 1
            return
        if hdr.chunk_total == 0 or hdr.chunk_idx >= hdr.chunk_total:
            self.stats.crc_err += 1
            return

        payload = pkt[P.VIDEO_HDR_SIZE:P.VIDEO_HDR_SIZE + hdr.payload_len]
        if len(payload) != hdr.payload_len:
            self.stats.crc_err += 1
            return

        fid = hdr.frame_id
        if fid not in self._pending:
            # 已结算帧的迟到片不是"新帧"，是重复投递的尾巴。单列一个计数而不是
            # 并进 dup_chunks：dup_chunks 量的是"在追的帧里收到重复片"（窗口内
            # 的重传/复制），late_chunks 量的是"整帧都交付完了还继续来的"（链路
            # 上的多路径复制）。两者的成因不同，混在一起的数看不出是哪一种。
            if fid in self._settled:
                self.stats.late_chunks += 1
                return
            # 新帧。号段有缺口就计一个跳号。板端的 frame_id 取自
            # V4L2 的 buf.sequence（DQBUF 一帧加一，节流丢掉的帧也占号），
            # 所以缺口只说明"这个号我们没见过"，不说明是谁弄没的——
            # 三种成因见 RxStats.frames_lost 的注释。
            if self._max_id is not None and fid > self._max_id + 1:
                self.stats.frames_lost += fid - self._max_id - 1
            self._pending[fid] = {}
            self._meta[fid] = hdr
            self._order.append(fid)
            if self._max_id is None or fid > self._max_id:
                self._max_id = fid

        slot = self._pending[fid]
        if hdr.chunk_idx in slot:
            self.stats.dup_chunks += 1
            return
        if slot and hdr.chunk_idx < max(slot):
            self.stats.reorder_chunks += 1
        slot[hdr.chunk_idx] = payload

    def _prune_settled(self) -> None:
        """丢掉 _settled 里已经不可能再有迟到片的帧号，免得长跑时无限增长。

        门槛取 16 倍窗口（15 fps 下约 3.2 秒）——真实链路的重复投递是毫秒级的
        （2026-10-07 板端实测：三份复制几乎同时到），16 倍窗口是三个数量级的余量。
        万一真有片迟到到被 prune 掉，后果也只是那一帧被复活、帧数虚高一次，
        也就是修这个 bug 之前的老行为，不会错交付——所以门槛宁可放宽。
        """
        if self._max_id is None:
            return
        cutoff = self._max_id - 16 * self.window
        if len(self._settled) > 64 and cutoff > 0:
            self._settled = {f for f in self._settled if f > cutoff}

    def _emit_ready(self) -> Frame | None:
        """取出一个可以交付的帧：最旧的已完成帧，或已滑出窗口的残帧。"""
        while self._order:
            fid = self._order[0]
            slot = self._pending.get(fid, {})
            hdr = self._meta[fid]
            complete = len(slot) == hdr.chunk_total
            abandoned = (self._max_id is not None and
                         self._max_id - fid >= self.window)
            if not complete and not abandoned:
                return None

            self._order.pop(0)
            del self._pending[fid]
            del self._meta[fid]
            self._settled.add(fid)
            self._prune_settled()
            # 帧一结算（不管完整与否）就把它的分片计入丢包统计。
            self.stats.chunks_expected += hdr.chunk_total
            self.stats.chunks_received += len(slot)
            if complete:
                data = b"".join(slot[i] for i in range(hdr.chunk_total))
                self.stats.frames_complete += 1
            else:
                data = b""
                self.stats.frames_incomplete += 1
            self.stats.last_frame_id = fid
            return Frame(
                frame_id=fid,
                timestamp_us=hdr.timestamp_us,
                width=hdr.width,
                height=hdr.height,
                format=hdr.format,
                data=data,
                complete=complete,
                chunks_got=len(slot),
                chunks_total=hdr.chunk_total,
                received_us=_now_us(),
            )
        return None

    def recv_frame(self, timeout: float | None = None) -> Frame | None:
        """返回下一个可交付的帧；超时或暂无可交付帧时返回 None。

        返回 None 不代表出错——包还在路上时本就无可交付。调用方应继续调用。
        """
        # 先把上一轮遗留的可交付帧吐出来，避免每次都要等新包。
        ready = self._emit_ready()
        if ready is not None:
            return ready

        if timeout is not None:
            self.sock.settimeout(max(0.0, timeout))
        else:
            self.sock.settimeout(None)
        try:
            pkt, _ = self.sock.recvfrom(65535)
        except socket.timeout:
            return None
        except OSError as exc:
            if exc.errno in (errno.EINTR, errno.EAGAIN):
                return None
            raise
        self._ingest(pkt)
        return self._emit_ready()


class FramePump:
    """在后台线程里持续收帧，主线程随时取走"最新一帧完整帧"。

    存在的理由（不是优化，是正确性）：3C 要把 OpenCV 检测和 imshow 加进来，
    而这两件事都可能偶尔卡上一两百毫秒（建窗口、换尺寸、系统忙）。
    只要它们和 `recv_frame` 在同一个线程里，卡住的那段时间就没人收包，
    内核 rcvbuf 会溢出开始丢包——实测这台机器只拿到 416 KiB ≈ 0.19 s。
    这种丢包**连 stats.datagrams 都看不到**：包在被 recvfrom 读到之前
    就没了，接收端会显示一切正常，画面却在掉帧。所以收包必须独占一个线程。

    生命周期：`stop()` 之后线程一定已经退出（带超时 join），
    不允许留下还在 recvfrom 的幽灵线程。
    """

    def __init__(self, rx: VideoReceiver, poll_timeout: float = 0.2):
        self.rx = rx
        self.poll_timeout = poll_timeout
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._new: Frame | None = None
        self._last: Frame | None = None
        self._dropped = 0          # 上一帧还没被取走就被这一帧顶替的次数
        self._thread: threading.Thread | None = None
        self.error: BaseException | None = None

    # ---- 生命周期 ----
    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("FramePump 已经在跑")
        self._thread = threading.Thread(target=self._run, name="video-rx",
                                        daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None

    def __enter__(self) -> "FramePump":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                f = self.rx.recv_frame(timeout=self.poll_timeout)
            except BaseException as exc:      # noqa: BLE001
                # 收包线程里的异常不能静默：主线程会一直显示最后一帧，
                # 看起来"视频卡住了"，而真因（比如权限、socket 被关）
                # 就此消失。记下来，由调用方决定怎么报。
                self.error = exc
                return
            if f is None or not f.complete:
                # 残帧不交给显示层：半帧解出来的是一块错位的乱码，
                # 交给跟踪器只会把它带到完全错误的方向（见本文件开头）。
                continue
            with self._lock:
                if self._new is not None:
                    self._dropped += 1
                self._new = f
                self._last = f

    # ---- 取帧 ----
    def take(self) -> Frame | None:
        """取走尚未被取走的最新完整帧；没有新的就返回 None。

        返回 None 不是错误——以 15 fps 收到、以更高的频率轮询时，
        大多数轮次本来就该是空的。调用方留着上一帧继续显示即可。
        """
        with self._lock:
            f, self._new = self._new, None
            return f

    def last(self) -> Frame | None:
        """最近一帧（可能是已经取过的）。用于"没新帧也要重画 HUD"。"""
        with self._lock:
            return self._last

    @property
    def dropped(self) -> int:
        """收帧比取帧快、被顶掉的帧数。持续增长说明主线程被拖住了。"""
        with self._lock:
            return self._dropped


def _fmt_stats(st: RxStats, rcvbuf: int) -> str:
    span = (st.last_us - st.first_us) / 1e6
    pkt_rate = st.datagrams / span if span > 0 else 0.0
    mbps = (st.bytes * 8 / 1e6 / span) if span > 0 else 0.0
    return (
        f"帧 {st.frames_complete} 完整 / {st.frames_incomplete} 残缺 / "
        f"{st.frames_lost} 跳号  {st.rate():.1f} fps  "
        f"片丢 {st.chunk_loss_pct():.2f}%  "
        f"包 {st.datagrams} ({mbps:.2f} Mbit/s, {pkt_rate:.0f} pkt/s)  "
        f"坏包 crc={st.crc_err} 魔数={st.bad_magic} 过短={st.short_pkts} "
        f"重复={st.dup_chunks} 迟到={st.late_chunks} 乱序={st.reorder_chunks}  "
        f"rcvbuf={rcvbuf // 1024}KiB  历时 {span:.1f}s"
    )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="接收板端 video_link 的 UDP 视频，统计链路质量",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--port", type=int, default=P.VIDEO_PORT)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--duration", type=float, default=0,
                    help="运行秒数，0 表示直到 Ctrl-C")
    ap.add_argument("--stats-ms", type=int, default=1000, help="0 表示不周期打印")
    ap.add_argument("--show", action="store_true",
                    help="用 OpenCV 显示画面（需要 numpy + opencv-python）")
    ap.add_argument("--save-dir", help="把完整帧的原始 YUYV 落盘（3C 调 HSV 用）")
    ap.add_argument("--save-frames", type=int, default=10,
                    help="最多落盘多少帧")
    args = ap.parse_args(argv)

    cv2 = None
    np = None
    if args.show:
        try:
            import cv2 as _cv2
            import numpy as _np
        except ImportError as exc:
            print(f"[ERROR] --show 需要 opencv-python 与 numpy：{exc}", file=sys.stderr)
            print("        安装：pip install --user numpy opencv-python", file=sys.stderr)
            return 2
        cv2, np = _cv2, _np

    try:
        rx = VideoReceiver(port=args.port, bind=args.bind)
    except OSError as exc:
        print(f"[ERROR] 绑定 UDP {args.bind}:{args.port} 失败：{exc}", file=sys.stderr)
        if exc.errno == errno.EADDRINUSE:
            print("        端口已被占用——可能上一次 video_rx 还在跑。", file=sys.stderr)
        return 2

    print(f"[INFO] 监听 UDP {args.bind}:{args.port}"
          f"（内核 rcvbuf={rx.rcvbuf // 1024} KiB，窗口={rx.window} 帧）")
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
        print(f"[INFO] 完整帧将落盘到 {args.save_dir}/（最多 {args.save_frames} 帧）")

    t0 = time.monotonic()
    t_last = t0
    saved = 0
    rc = 0
    try:
        while True:
            frame = rx.recv_frame(timeout=0.5)
            now = time.monotonic()
            if frame is not None and frame.complete:
                if args.save_dir and saved < args.save_frames:
                    h, w = frame.height, frame.width
                    stride = len(frame.data) // h if h else 0
                    name = (f"{args.save_dir}/frame_{frame.frame_id:05d}_"
                            f"{w}x{h}_stride{stride}.yuyv")
                    with open(name, "wb") as fp:
                        fp.write(frame.data)
                    saved += 1
                if args.show:
                    bgr = cv2.cvtColor(
                        np.frombuffer(frame.data, dtype=np.uint8)
                        .reshape(frame.height, frame.width, 2),
                        cv2.COLOR_YUV2BGR_YUYV)
                    cv2.imshow("video_link", bgr)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        print("[INFO] 收到 q，退出")
                        break
            if args.stats_ms > 0 and (now - t_last) * 1000 >= args.stats_ms:
                print(f"[stats] {_fmt_stats(rx.stats, rx.rcvbuf)}", flush=True)
                t_last = now
            if args.duration > 0 and now - t0 >= args.duration:
                break
    except KeyboardInterrupt:
        print("\n[INFO] 收到 Ctrl-C")
    finally:
        if cv2 is not None:
            cv2.destroyAllWindows()
        print(f"[final] {_fmt_stats(rx.stats, rx.rcvbuf)}")
        if rx.stats.datagrams == 0:
            print("[WARN] 一个包都没收到。检查：板端 --dst 是否指向本机 IP；"
                  "本机防火墙是否放行 UDP；两边是否在同一网段。")
            rc = 1
        rx.close()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
