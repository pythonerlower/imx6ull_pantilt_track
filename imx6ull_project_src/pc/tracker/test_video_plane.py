#!/usr/bin/env python3
"""3C 视频面的**无硬件**验收：合成帧 → UDP 自环 → 收帧线程 → 检测 → HUD 落盘。

为什么可以在没有板子、没有摄像头的情况下做验收：
这一段的输入是"一串 YUYV 帧分包"，不是"一束光"。用 `ptproto.video_pack()`
按板端一模一样的线格式把这串包发到回环口，PC 侧走的就是生产路径——
同一个 VideoReceiver、同一个 FramePump、同一个 get_target_center。
所以它能验的东西比看上去多：

  - 收帧线程有没有真的和显示线程分开（这是状态文档 §6.4 的硬要求，
    因为它一旦搞错，表现是"画面偶尔卡一下"，根本不像 bug，像网络抖动）
  - 残帧有没有被挡在检测之外（半帧会解出错位的图，检测器会给出一个
    完全错误但看上去很合理的中心）
  - 取帧比收帧慢时，丢的是**旧帧**还是把新帧一起堵住
  - 从 UDP 端口到 PNG 文件这一整条链真的能跑通，不是各自单测都过

它验不了的东西也要说清楚：光照下的颜色阈值、真实目标的颜色分布、
板端编码是否与这里镜像的实现一致——那些只能上真机。

    python3 test_video_plane.py
"""

from __future__ import annotations

import os
import socket
import tempfile
import threading
import time

import cv2
import numpy as np

import ptproto as P
from video_rx import FramePump, VideoReceiver

W, H = 320, 240
PORT = 9299                      # 故意不用 9200：本机可能正跑着 video_link
LOOPBACK = ("127.0.0.1", PORT)


# ------------------------------------------------------------------ 造帧/发帧

def yuyv_of(bgr: np.ndarray) -> bytes:
    """BGR → 板端那种打包 YUYV。"""
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_YUYV).reshape(H, W * 2).tobytes()


def scene(cx: float, cy: float = 120.0, radius: int = 22) -> np.ndarray:
    """中灰背景 + 一个红圆。用红圆是因为默认阈值就是找红。"""
    img = np.full((H, W, 3), 128, np.uint8)
    cv2.circle(img, (int(cx), int(cy)), radius, (0, 0, 230), -1)
    return img


def send_frame(sock, fid: int, yuyv: bytes, drop: tuple = ()) -> int:
    """把一帧切成板端那样的分包发出去。返回实际发了几个包。"""
    total = (len(yuyv) + P.VIDEO_PAYLOAD - 1) // P.VIDEO_PAYLOAD
    ts = time.monotonic_ns() // 1000
    sent = 0
    for i in range(total):
        if i in drop:
            continue
        part = yuyv[i * P.VIDEO_PAYLOAD:(i + 1) * P.VIDEO_PAYLOAD]
        sock.sendto(P.video_pack(fid, i, total, ts, W, H, P.FMT_YUYV, part),
                    LOOPBACK)
        sent += 1
    return sent


class Sender:
    """后台按指定 fps 发帧，直到 stop()。"""

    def __init__(self, cx_from_to=(60.0, 260.0), fps: float = 60.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        self.cx0, self.cx1 = cx_from_to
        self.dt = 1.0 / fps
        self.fid = 0
        self._stop = threading.Event()
        self._t: threading.Thread | None = None

    def start(self) -> None:
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self) -> None:
        n = max(int(self.cx1 - self.cx0), 1)
        while not self._stop.is_set():
            cx = self.cx0 + (self.fid % n)
            self.fid += 1
            send_frame(self.sock, self.fid, yuyv_of(scene(cx)))
            time.sleep(self.dt)

    def stop(self) -> None:
        self._stop.set()
        if self._t:
            self._t.join(1.0)
        self.sock.close()


def drain(pump: FramePump, seconds: float) -> list:
    """把 pump 里的帧取空，返回取到的帧列表（保持顺序）。

    必须**边收边取**：FramePump 只留最新一帧（这正是它该做的），
    发完再取的话中间那些已经被顶掉了，能看到什么全看运气。
    """
    out = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        f = pump.take()
        if f is not None:
            out.append(f)
        time.sleep(0.002)
    return out


def send_async(frames, gap: float = 0.03) -> threading.Thread:
    """后台逐帧发出去。frames 是 [(fid, drop_tuple), ...]。

    串行发完再收是不行的：收帧线程一直在跑，发到第 6 帧时前 5 帧早被顶掉了。
    要验"逐帧都交付对了"就得让发和收同时进行。
    """
    def run():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            for fid, drop in frames:
                send_frame(s, fid, yuyv_of(scene(160)), drop=drop)
                time.sleep(gap)
        finally:
            s.close()

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


# ------------------------------------------------------------------ FramePump

def test_pump_skips_incomplete_frames():
    """缺片的帧绝不能交给检测器。

    半帧解出来的是一张错位的图——上半截还是画面、下半截是上一帧的行，
    检测器会在这张图上找到一个"看上去完全合理"的中心。所以残帧必须在
    video_rx 里就被拦掉，而不是指望下游自己能发现。
    """
    rxC = VideoReceiver(port=PORT)
    try:
        pump = FramePump(rxC, poll_timeout=0.05)
        pump.start()
        frames = [(fid, (7,) if fid == 2 else ()) for fid in range(1, 7)]
        send_async(frames)
        got = drain(pump, 0.9)
        ids = [f.frame_id for f in got]
        assert 2 not in ids, f"缺片的第 2 帧被交出去了：{ids}"
        assert all(f.complete for f in got), "交出来的帧里有非完整的"
        assert ids == sorted(ids), f"交付顺序乱了：{ids}"
        assert pump.error is None, f"收帧线程报错：{pump.error!r}"
        pump.stop()
    finally:
        rxC.close()


def test_clean_run_delivers_every_frame_in_order():
    """没有丢包时，每一帧都要按序交付，一个不少。

    这条是上面那条的对照组：上面允许"少了几个 id"（丢一片会把后面几帧
    堵在队列里、放开时挤成一堆，消费者来不及逐帧取走），但如果**没丢片**
    还少帧，那就是收帧/交付本身有毛病。
    """
    rxC = VideoReceiver(port=PORT)
    try:
        pump = FramePump(rxC, poll_timeout=0.05)
        pump.start()
        frames = [(fid, ()) for fid in range(1, 9)]
        send_async(frames, gap=0.05)
        got = drain(pump, 1.0)
        ids = [f.frame_id for f in got]
        assert ids == list(range(1, 9)), f"干净链路应逐帧齐全，实际 {ids}"
        assert pump.dropped == 0, f"消费者没跟丢，却报 {pump.dropped} 帧被顶掉"
        pump.stop()
    finally:
        rxC.close()


def test_abandoned_frame_is_counted_incomplete():
    """丢了一片、后面又没有更新帧来顶它时，它必须在窗口滚过之后被记账。

    这条是"不完整帧会被安静地忘掉"的守门：忘了的话统计说 0 残缺，
    而画面上确实少了一帧，排查时就会往完全错误的方向找。

    顺带钉住一条已知行为：残帧会**堵住**它后面的帧，直到帧号滑出窗口
    （window=3、15 fps 下约 200 ms）才一起放行。这是 video_rx 的窗口语义
    决定的（先来先交付，不能跳过未结算的帧，否则帧序就乱了），
    不是缺陷——但 3D 闭环要知道存在这么一段最坏 200 ms 的成组延迟。
    """
    rxC = VideoReceiver(port=PORT, window=3)
    try:
        pump = FramePump(rxC, poll_timeout=0.05)
        pump.start()
        frames = [(1, (7,))] + [(fid, ()) for fid in range(2, 8)]
        send_async(frames)
        got = drain(pump, 0.9)
        st = rxC.stats
        assert st.frames_incomplete >= 1, \
            f"丢片的帧没被记成残缺：incomplete={st.frames_incomplete}"
        delivered = [f.frame_id for f in got]
        assert 1 not in delivered, f"残帧被交付了：{delivered}"
        assert delivered == sorted(delivered), f"交付顺序乱了：{delivered}"
        # 发了 7 帧、只有第 1 帧缺片，所以完整帧必然是 6 个；
        # 交付数可以更少（放开时挤成一堆，消费者来不及逐帧取走），
        # 但绝不可能更多——多出来就说明有帧被重复交付了。
        assert st.frames_complete == 6, \
            f"7 帧里只该有 1 帧残缺，统计却是 {st.frames_complete} 完整"
        assert len(delivered) <= st.frames_complete, \
            f"交付 {len(delivered)} 帧 > 完整帧 {st.frames_complete}"
        pump.stop()
    finally:
        rxC.close()


def test_take_returns_none_when_nothing_new():
    """没新帧时必须返回 None，不能把同一帧反复交出去。

    反复交同一帧在显示上完全看不出来（画面本来就没变），但下游任何
    "按帧计数"的逻辑都会算错——3D 的积分项尤其受不了这个。
    """
    rxC = VideoReceiver(port=PORT)
    try:
        pump = FramePump(rxC)
        pump.start()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            send_frame(s, 1, yuyv_of(scene(160)))
            time.sleep(0.3)
        finally:
            s.close()
        assert pump.take() is not None, "该有新帧的时候没给"
        assert pump.take() is None, "同一帧被交付了两次"
        assert pump.last() is not None, "last() 应仍能拿到最近一帧"
        pump.stop()
    finally:
        rxC.close()


def test_slow_consumer_drops_old_frames():
    """取帧慢时应该丢**旧**帧、保留最新帧，而不是把新的也堵回去。"""
    rxC = VideoReceiver(port=PORT)
    try:
        pump = FramePump(rxC)
        pump.start()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            for fid in range(1, 9):
                send_frame(s, fid, yuyv_of(scene(160)))
                time.sleep(0.03)
            time.sleep(0.3)
        finally:
            s.close()
        f = pump.take()
        assert f is not None
        assert f.frame_id == 8, f"应拿到最新帧 8，实际 {f.frame_id}"
        assert pump.dropped == 7, f"被顶掉的帧数应为 7，实际 {pump.dropped}"
        pump.stop()
    finally:
        rxC.close()


def test_pump_surfaces_socket_error():
    """收帧线程出问题时必须留下痕迹。

    静默死掉的收帧线程看起来就是"视频卡住了"——操作者会去查网络、
    查板子，而真因（socket 被关、权限、端口被占）已经无影无踪。
    """
    rxC = VideoReceiver(port=PORT)
    pump = FramePump(rxC, poll_timeout=0.05)
    pump.start()
    time.sleep(0.1)
    rxC.close()                     # 从下面把 socket 抽掉
    end = time.monotonic() + 2.0
    while pump.error is None and time.monotonic() < end:
        time.sleep(0.02)
    assert pump.error is not None, "socket 出错了却没有任何记录"
    pump.stop(0.5)
    assert not pump._thread, "stop() 之后不该还留着线程句柄"


def test_start_twice_is_refused():
    rxC = VideoReceiver(port=PORT)
    try:
        pump = FramePump(rxC)
        pump.start()
        try:
            pump.start()
        except RuntimeError:
            pump.stop()
            return
        raise AssertionError("重复 start() 应被拒绝")
    finally:
        rxC.close()


# ------------------------------------------------------------------ VideoPlane

class FakeManual:
    """冒充 Manual——VideoPlane 只用到 fresh/note/step 这三样。

    特意不去构造真的 Manual：那会 bind 9101 并开一个 ControlClient，
    把这个纯 PC 侧的单测变成依赖板端端口的测试。
    """

    def __init__(self):
        self.note = "单测"
        self.tag = "TESTCHAIN"                  # HUD 只准用这个 ASCII 字段
        self.step = 3
        self.max_age_ms = 500.0
        self.aligned = False                    # 遥测永远不来，故永远不 align

    def fresh(self):
        return None, float("inf")


def plane_with_sender():
    import main as M

    vp = M.VideoPlane(PORT)
    s = Sender(cx_from_to=(60.0, 260.0), fps=60.0)
    vp.start()
    s.start()
    return vp, s


def test_plane_tracks_moving_target():
    """目标横向扫过去，报回来的中心也得跟着走。

    断言的是"跟着走"（跨度足够大、方向对），不是"每帧都准到某个具体值"——
    具体值由 tracker 的单测在合成图上精确验；这里验的是从收帧线程到
    检测这一路上没有把坐标搞丢或者搞反。
    """
    vp, s = plane_with_sender()
    try:
        xs = []
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            if vp.poll():
                xs.append(vp.target.cx)
            time.sleep(0.005)
        assert len(xs) >= 5, f"只检测到 {len(xs)} 帧"
        found = [x for x in xs if x is not None]
        assert len(found) >= len(xs) * 0.8, \
            f"{len(xs) - len(found)}/{len(xs)} 帧没找到那个红圆"
        span = max(found) - min(found)
        assert min(found) <= 75, f"起始位置就偏了：最小 {min(found):.0f}"
        assert span >= 40, f"中心没跟着目标走，只动了 {span:.0f} 像素"
        # 目标只往右走，中心序列不该出现回头（检测抖动 1 像素以内不算）
        back = [b - a for a, b in zip(found, found[1:]) if b - a < -2]
        assert not back, f"中心出现了反向跳变 {back[:5]}"
    finally:
        s.stop()
        vp.close()


def test_plane_bgr_matches_what_was_detected():
    """检测和显示必须用同一张解码结果。

    各解各的时候，两边的 stride/格式假设一旦不一致，就会出现
    "图上明明有目标、框却偏在别处"——这种错最难解释，因为它每一半都自洽。
    """
    vp, s = plane_with_sender()
    try:
        end = time.monotonic() + 1.0
        while not vp.poll() and time.monotonic() < end:
            time.sleep(0.005)
        assert vp.bgr is not None and vp.frame is not None
        assert vp.bgr.shape == (H, W, 3), vp.bgr.shape
        from tracker import get_target_center
        again = get_target_center(vp.bgr)
        assert abs(again.cx - vp.target.cx) <= 0.5, \
            "同一张 BGR 两次检测结果不一致（检测器不是纯函数？）"
        x, y, bw, bh = vp.target.bbox
        # 报出来的中心必须落在同一张图的范围内——越界说明坐标体系串了
        assert 0 <= x and x + bw <= W and 0 <= y and y + bh <= H, \
            f"bbox {vp.target.bbox} 超出 {W}x{H}"
    finally:
        s.stop()
        vp.close()


def test_hud_reports_each_state():
    """跟踪状态与遥测状态**各自独立**可达。

    这两件事互不依赖：板端 daemon 没跑也照样看得见红物体，摄像头没出图
    也照样收得到遥测。合成一个状态字会让其中一个把另一个盖掉，
    而那正好是排查时最需要区分开的两件事——所以这条测试同时钉住
    "没遥测时也要报 TRACK"，那是现场用红色物体验收时最常见的组合。
    """
    vp, s = plane_with_sender()
    m = FakeManual()
    try:
        # 还没收到帧、也没遥测：跟踪侧是 NO-VIDEO，遥测侧是 NO-TELEM
        h = vp.hud(m, "", "")
        assert h.state == "NO-VIDEO", h.state
        assert h.telem == "NO-TELEM", h.telem

        class T:
            pan_cur = tilt_cur = 300
            pan_tgt = tilt_tgt = 300
            pan_safe_min, pan_safe_max = 20, 580
            tilt_safe_min, tilt_safe_max = 20, 580

        m.fresh = lambda: (T(), 5.0)
        end = time.monotonic() + 1.0
        while not vp.poll() and time.monotonic() < end:
            time.sleep(0.005)
        h = vp.hud(m, "link", "note")
        assert h.state == "TRACK", h.state
        assert h.telem == "TELEM-OK", h.telem
        assert h.tick_pan == 300 and h.cmd_pan == 300
        assert h.frame_id is not None
        assert h.error is not None and h.error[0] is not None

        # 有目标但遥测过期 → TRACK 与 STALE 并存，谁也不盖谁
        m.fresh = lambda: (T(), 9999.0)
        h = vp.hud(m, "", "")
        assert h.state == "TRACK" and h.telem == "STALE", (h.state, h.telem)

        # 没有 daemon（现场举红物体验收时就是这种）→ TRACK 与 NO-TELEM 并存
        m.fresh = lambda: (None, float("inf"))
        h = vp.hud(m, "", "")
        assert h.state == "TRACK", f"没遥测就把跟踪状态盖掉了：{h.state}"
        assert h.telem == "NO-TELEM", h.telem

        # 把检测换成必定失败，同一个循环的下一次就该报 LOST
        from tracker import Target
        vp.target = Target(found=False)
        assert vp.hud(m, "", "").state == "LOST"
    finally:
        s.stop()
        vp.close()


def test_run_video_writes_hud_pngs():
    """整条链一起跑：UDP 进 → 检测 → 画 HUD → 落盘 PNG。

    这是 3C 无硬件部分唯一一条真正意义上的端到端断言：
    前面每条都能过而这一条不过，说明拼装层（main.run_video）有缝，
    而那正是"各部分单测全绿、合起来不动"的经典位置。
    """
    import main as M

    out = tempfile.mkdtemp(prefix="3c_hud_")
    vp = M.VideoPlane(PORT)
    vp.start()
    s = Sender(fps=80.0)
    s.start()
    try:
        rc = M.run_video(FakeManual(), vp, 1.2, None, out, 5)
    finally:
        s.stop()
        vp.close()
    assert rc == 0, f"run_video 返回 {rc}"

    pngs = sorted(os.listdir(out))
    assert pngs, f"{out} 里没有 PNG——HUD 落盘这一环没跑起来"
    img = cv2.imread(os.path.join(out, pngs[-1]))
    assert img is not None and img.shape == (H, W, 3), img.shape
    # 绿框（找到目标）+ 灰色十字（画面中心）都必须在图上
    assert (img[:, :, 1].astype(int) - img[:, :, 2] > 100).sum() > 10, \
        "落盘的图里没有绿色 bbox"
    assert (np.abs(img.astype(int) - 170).sum(axis=2) < 60).sum() > 4, \
        "落盘的图里没有中心十字"


def main() -> int:
    print("video_plane 单测（无硬件）")
    tests = [
        test_pump_skips_incomplete_frames,
        test_clean_run_delivers_every_frame_in_order,
        test_abandoned_frame_is_counted_incomplete,
        test_take_returns_none_when_nothing_new,
        test_slow_consumer_drops_old_frames,
        test_pump_surfaces_socket_error,
        test_start_twice_is_refused,
        test_plane_tracks_moving_target,
        test_plane_bgr_matches_what_was_detected,
        test_hud_reports_each_state,
        test_run_video_writes_hud_pngs,
    ]
    for t in tests:
        try:
            t()
        except AssertionError as exc:
            print(f"  [FAIL] {t.__name__}: {exc}")
            return 1
        except Exception as exc:                       # noqa: BLE001
            print(f"  [ERROR] {t.__name__}: {exc.__class__.__name__}: {exc}")
            return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
