"""打印与 C 侧 test_proto.c 相同的固定向量，供 scripts/proto_selftest.sh 对拍。

只做两件事：断言本侧行为，然后按同样格式打十六进制。
向量值必须与 test_proto.c 逐项相同——改一侧就要改另一侧，
这是有意的：向量不一致本身就是"协议动了"的信号。

    python3 selftest_vectors.py          # 打向量
    python3 selftest_vectors.py --check   # 只跑断言，不打向量
"""

from __future__ import annotations

import sys

import ptproto as P

VEC_PAYLOAD = b"HELLO-IMX6ULL"


def print_hex(label: str, buf: bytes) -> None:
    print(f"{label:<6}{buf.hex()}")


def vector_ctrl() -> P.Ctrl:
    c = P.Ctrl(
        seq=7,
        mode=P.MODE_MANUAL,
        timestamp_us=1234567890123,
        flags=P.FLAG_HEARTBEAT,
        pan_tick=377,
        tilt_tick=242,
    )
    enc = c.encode()
    back = P.Ctrl.decode(enc)
    assert back == c, f"ctrl 往返不一致: {back} != {c}"
    print("  [ok] ctrl 往返一致")

    bad = bytearray(enc)
    bad[10] ^= 0x01
    try:
        P.Ctrl.decode(bytes(bad))
    except ValueError:
        print("  [ok] ctrl 坏 CRC 被拒")
    else:
        raise AssertionError("ctrl 坏 CRC 未被拒")

    try:
        P.Ctrl.decode(enc[:-1])
    except ValueError:
        print("  [ok] ctrl 错长度被拒")
    else:
        raise AssertionError("ctrl 错长度未被拒")

    return c


def vector_telem() -> P.Telem:
    t = P.Telem(
        seq=9,
        timestamp_us=9876543210987,
        drv_status=P.DRV_SLEW,
        chip_state=P.CHIP_RUNNING,
        daemon_state=P.DST_TRACK,
        slew_ms=20,
        wd_ms=1500,
        daemon_wd_ms=500,
        pan_cur=377,
        tilt_cur=242,
        pan_tgt=400,
        tilt_tgt=222,
        pan_home=377,
        tilt_home=242,
        pan_safe_min=239,
        pan_safe_max=515,
        tilt_safe_min=162,
        tilt_safe_max=342,
        ms_since_ctrl=1234,
        last_latency_us=20000,
        cmd_count=1000,
        clamp_low_pan=1,
        clamp_low_tilt=2,
        clamp_high_pan=3,
        clamp_high_tilt=4,
        i2c_err=5,
        watchdog_trips=6,
        ctrl_rx_ok=999,
        rej_crc=7,
        rej_seq=8,
        rej_stale=9,
        rej_mode=10,
        rej_range=11,
        daemon_wd_trips=12,
    )
    enc = t.encode()
    assert len(enc) == P._TELEM_SIZE, f"遥测编码长度 {len(enc)}"
    assert P.Telem.decode(enc) == t, "遥测往返不一致"
    print("  [ok] 遥测往返一致")
    # 解码器会拒绝错魔数；顺手确认它确实在看魔数，而不是照单全收。
    try:
        P.Telem.decode(b"\x00" * P._TELEM_SIZE)
    except ValueError:
        print("  [ok] 遥测错魔数被拒")
    else:
        raise AssertionError("遥测错魔数未被拒")
    return t


def vector_video() -> bytes:
    dgram = P.video_pack(
        frame_id=4096,
        chunk_idx=3,
        chunk_total=110,
        timestamp_us=1122334455667,
        width=320,
        height=240,
        fmt=P.FMT_YUYV,
        payload=VEC_PAYLOAD,
    )
    hdr = P.VideoHdr.decode(dgram)
    assert hdr.chunk_idx == 3 and hdr.chunk_total == 110, hdr
    assert hdr.payload_len == len(VEC_PAYLOAD), hdr
    print("  [ok] 视频包头解出且与编码一致")
    try:
        P.VideoHdr.decode(dgram[:-1] + bytes([dgram[-1] ^ 0xFF]))
    except ValueError:
        print("  [ok] 视频包坏载荷 CRC 被拒")
    else:
        raise AssertionError("视频包坏载荷 CRC 未被拒")
    return dgram


def main() -> int:
    if "--check" not in sys.argv:
        print("线格式自测（Python 侧）")
    else:
        print("线格式自测（Python 侧，仅断言）")

    assert P.crc32(b"123456789") == 0xCBF43926, "crc32 标准向量不符"
    print("  [ok] crc32 标准向量 0xcbf43926")

    c = vector_ctrl()
    t = vector_telem()
    dgram = vector_video()

    if "--check" not in sys.argv:
        print_hex("ctrl", c.encode())
        print_hex("telem", t.encode())
        print_hex("video", dgram)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
