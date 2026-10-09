"""video_rx 的回环单测：不碰开发板，在本机用回环 UDP 自己发自己收。

存在的理由：整帧重组里最容易写错的是**残缺帧的判定**——丢了一片的那帧既不完整、
也不该永远占着内存。这类错误在肉眼审代码时看不出来（代码"看起来"在等齐所有片），
只有真的丢一片、再让帧号往前走，才知道它到底是被丢弃并计数，还是卡住了。

对照板端：板端把 320x240 YUYV 切成 110 片，本测试用 4 片，逻辑一样但跑得快。

    python3 test_video_rx.py
"""

from __future__ import annotations

import socket
import sys
import time

import ptproto as P
from video_rx import VideoReceiver

FRAME_PAYLOAD = bytes(range(256)) * 6  # 1536 字节，够切 2 片
CHUNK = 700


def payload_for(frame_id: int, idx: int, plen: int) -> bytes:
    """确定性的载荷：每帧每片都不一样，重组错了立刻能看出来。"""
    return bytes(((frame_id * 31 + idx * 7 + i) & 0xFF) for i in range(plen))


def chunks_of(frame_id: int, nchunks: int, plen: int = CHUNK):
    out = []
    for idx in range(nchunks):
        out.append((idx, payload_for(frame_id, idx, plen)))
    return out


def send_frame(sock, dst, frame_id, nchunks=4, *, plen=CHUNK,
               drop=(), dup=(), order=None, break_crc_at=()):
    """把一帧切成 nchunks 片发出去。drop/dup/order/break_crc_at 用来制造链路异常。"""
    items = [c for c in chunks_of(frame_id, nchunks, plen)
             if c[0] not in drop]
    if order is not None:
        items = sorted(items, key=lambda c: order(c[0]))
    for idx, data in items:
        pkt = P.video_pack(
            frame_id=frame_id, chunk_idx=idx, chunk_total=nchunks,
            timestamp_us=1000 + frame_id, width=320, height=240,
            fmt=P.FMT_YUYV, payload=data)
        if idx in break_crc_at:
            # 只破坏头部的 crc 字段，载荷本身完好——
            # 这样测的是"CRC 校验有没有生效"，而不是"载荷被改坏"。
            pkt = pkt[:-4] + bytes([pkt[-4] ^ 0xFF]) + pkt[-3:]
        sock.sendto(pkt, dst)
        if idx in dup:
            sock.sendto(pkt, dst)  # 重复投递同一个包


class Harness:
    def __init__(self, window: int = 3):
        self.rx = VideoReceiver(port=0, window=window)
        self.port = self.rx.sock.getsockname()[1]
        self.dst = ("127.0.0.1", self.port)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def close(self):
        self.tx.close()
        self.rx.close()

    def drain(self, want: int, timeout: float = 0.5) -> list:
        """收够 want 帧或超时为止。超时是正常的——残帧要等后续帧号推进才结算。"""
        got = []
        deadline = time.monotonic() + timeout
        while len(got) < want and time.monotonic() < deadline:
            f = self.rx.recv_frame(timeout=0.1)
            if f is not None:
                got.append(f)
        return got


def test_complete_frame():
    h = Harness()
    try:
        send_frame(h.tx, h.dst, 10, 4)
        frames = h.drain(1)
        assert len(frames) == 1, f"应收到 1 帧，实收 {len(frames)}"
        f = frames[0]
        assert f.complete, "完整帧被判为残缺"
        expect = b"".join(payload_for(10, i, CHUNK) for i in range(4))
        assert f.data == expect, "重组后的数据与发送不一致"
        assert h.rx.stats.chunk_loss_pct() == 0.0, "完整帧却算出丢包"
        print("  [ok] 完整帧重组正确，且无丢包计数")
    finally:
        h.close()


def test_missing_chunk_is_abandoned_and_counted():
    h = Harness(window=3)
    try:
        # 第 10 帧故意少发第 2 片，再用后续帧把帧号推过窗口。
        send_frame(h.tx, h.dst, 10, 4, drop=(2,))
        for fid in (11, 12, 13):
            send_frame(h.tx, h.dst, fid, 4)
        frames = h.drain(4)
        by_id = {f.frame_id: f for f in frames}
        assert 10 in by_id, "缺片的帧没有被结算出来（可能一直挂在校验窗口里）"
        assert not by_id[10].complete, "缺片的帧被判为完整"
        assert by_id[10].chunks_got == 3, \
            f"应收到 3 片，实收 {by_id[10].chunks_got}"
        assert by_id[10].chunks_total == 4
        assert by_id[10].data == b"", "残缺帧不该交付数据"
        for fid in (11, 12, 13):
            assert by_id[fid].complete, f"帧 {fid} 应完整"
        st = h.rx.stats
        assert st.frames_complete == 3, st.frames_complete
        assert st.frames_incomplete == 1, st.frames_incomplete
        # 16 片里丢 1 片 = 6.25%
        assert abs(st.chunk_loss_pct() - 6.25) < 0.01, st.chunk_loss_pct()
        print(f"  [ok] 缺片的帧被放弃并计数（片丢 {st.chunk_loss_pct():.2f}%）")
    finally:
        h.close()


def test_duplicate_chunk_does_not_break_frame():
    h = Harness()
    try:
        send_frame(h.tx, h.dst, 20, 4, dup=(1,))
        frames = h.drain(1)
        assert len(frames) == 1 and frames[0].complete, "重复包导致整帧被判残缺"
        expect = b"".join(payload_for(20, i, CHUNK) for i in range(4))
        assert frames[0].data == expect, "重复包污染了重组结果"
        assert h.rx.stats.dup_chunks == 1, h.rx.stats.dup_chunks
        print("  [ok] 重复包被计为重复，且未污染重组结果")
    finally:
        h.close()


def test_late_chunk_after_settle_does_not_resurrect_frame():
    h = Harness(window=3)
    try:
        # 先正常交付 4 帧（每帧齐全，逐帧立刻结算）。
        for fid in (70, 71, 72, 73):
            send_frame(h.tx, h.dst, fid, 4)
        got = h.drain(4)
        assert len(got) == 4, f"应交付 4 帧，实交付 {len(got)}"
        assert h.rx.stats.frames_incomplete == 0, h.rx.stats.frames_incomplete

        # 再补投第 70 帧的 2 片——链路多路径复制的典型形态：整帧都交付完了，
        # 副本才到。修这个 bug 之前，它会把已结算的 70 当成"新帧"重建槽位，
        # 随即被窗口判为残缺帧结算掉，于是 frames_incomplete 凭空多 1、分母
        # 也多一帧的分片数，把丢包率算虚。这是**计数**错误，不是交付错误，
        # 所以只有断言计数才抓得住。
        pkt = P.video_pack(
            frame_id=70, chunk_idx=0, chunk_total=4, timestamp_us=1000 + 70,
            width=320, height=240, fmt=P.FMT_YUYV,
            payload=payload_for(70, 0, CHUNK))
        h.tx.sendto(pkt, h.dst)
        h.tx.sendto(pkt, h.dst)
        for _ in range(4):
            h.rx.recv_frame(timeout=0.2)

        st = h.rx.stats
        assert st.frames_incomplete == 0, \
            f"已结算的帧被复活成残缺帧（残缺={st.frames_incomplete}）"
        assert st.frames_complete == 4, f"完整帧数被改动：{st.frames_complete}"
        assert st.late_chunks == 2, f"迟到片未计数：{st.late_chunks}"
        assert st.chunk_loss_pct() == 0.0, \
            f"复活的残帧污染了丢包率：{st.chunk_loss_pct()}"
        print(f"  [ok] 已结算帧的迟到片被丢弃（记到 {st.late_chunks} 片），"
              f"不复活也不污染统计")
    finally:
        h.close()


def test_reordered_chunks_still_reassemble():
    h = Harness()
    try:
        # 把分片顺序整个倒过来发。UDP 本就可能乱序，重组必须与到达顺序无关。
        send_frame(h.tx, h.dst, 30, 6, order=lambda i: -i)
        frames = h.drain(1)
        assert len(frames) == 1 and frames[0].complete, "乱序到达导致整帧被判残缺"
        expect = b"".join(payload_for(30, i, CHUNK) for i in range(6))
        assert frames[0].data == expect, "乱序重组结果错位——片序没有按 chunk_idx 还原"
        assert h.rx.stats.reorder_chunks > 0, "倒序发送却没有记到乱序"
        print(f"  [ok] 乱序到达仍能正确重组（记到 {h.rx.stats.reorder_chunks} 次乱序）")
    finally:
        h.close()


def test_bad_crc_chunk_is_rejected():
    h = Harness(window=3)
    try:
        # 只坏第 1 片。注意不能全坏：若一帧的每个包都被拒，接收端根本不会
        # 知道这一帧存在过（只能用帧号跳号去推断），这是 UDP 的固有性质，
        # 不是缺陷。要测"坏包被拒后这帧成为残帧"，就必须留下有效片。
        send_frame(h.tx, h.dst, 40, 4, break_crc_at=(1,))
        for fid in (41, 42, 43):
            send_frame(h.tx, h.dst, fid, 4)
        frames = h.drain(4)
        by_id = {f.frame_id: f for f in frames}
        assert 40 in by_id, "残帧没有被结算出来"
        assert not by_id[40].complete, "CRC 坏掉的片被当成有效片收下了"
        assert by_id[40].chunks_got == 3, by_id[40].chunks_got
        assert h.rx.stats.crc_err == 1, \
            f"应有 1 个坏包被拒，实为 {h.rx.stats.crc_err}"
        for fid in (41, 42, 43):
            assert by_id[fid].complete, f"帧 {fid} 不该受影响"
        print(f"  [ok] 载荷 CRC 不符的包被拒（记到 {h.rx.stats.crc_err} 个）")
    finally:
        h.close()


def test_whole_frame_gap_counted_as_lost():
    h = Harness(window=3)
    try:
        # 帧 50 发送，51~53 一个包都没发（号段缺口），再发 54。
        send_frame(h.tx, h.dst, 50, 4)
        send_frame(h.tx, h.dst, 54, 4)
        h.drain(2)
        st = h.rx.stats
        assert st.frames_lost == 3, \
            f"帧号 50→54 之间有 3 帧丢失，却记到 {st.frames_lost}"
        print(f"  [ok] 帧号跳号被记成 frames_lost（不区分成因）（记到 {st.frames_lost} 帧）")
    finally:
        h.close()


def test_non_video_packet_counted_not_crashed():
    h = Harness()
    try:
        # 串端口的包必须被识别并分类计数，而不是崩掉或污染链路统计。
        # 64 字节的垃圾包：长度够，但魔数不对 → bad_magic
        # 32 字节的控制包：比视频头还短，连魔数都读不到 → short_pkts
        h.tx.sendto(b"\x00" * 64, h.dst)
        h.tx.sendto(P.Ctrl(seq=1).encode(), h.dst)
        send_frame(h.tx, h.dst, 60, 4)
        frames = h.drain(1)
        assert len(frames) == 1 and frames[0].complete, "串端口包影响了正常收帧"
        assert h.rx.stats.bad_magic == 1, \
            f"应有 1 个魔数不符的包，实为 {h.rx.stats.bad_magic}"
        assert h.rx.stats.short_pkts == 1, \
            f"应有 1 个过短包，实为 {h.rx.stats.short_pkts}"
        assert h.rx.stats.crc_err == 0, \
            f"串端口包不该被算成 CRC 错误（实为 {h.rx.stats.crc_err}）"
        print("  [ok] 串端口包按长度/魔数分别计数，未影响正常收帧")
    finally:
        h.close()


def main() -> int:
    print("video_rx 回环单测")
    tests = [
        test_complete_frame,
        test_missing_chunk_is_abandoned_and_counted,
        test_duplicate_chunk_does_not_break_frame,
        test_late_chunk_after_settle_does_not_resurrect_frame,
        test_reordered_chunks_still_reassemble,
        test_bad_crc_chunk_is_rejected,
        test_whole_frame_gap_counted_as_lost,
        test_non_video_packet_counted_not_crashed,
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
