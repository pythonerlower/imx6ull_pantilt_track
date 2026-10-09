"""云台/视频线格式的 PC 侧实现。

板端是 C（imx6ull_project_src/common/proto/pantilt_proto.h），本文件是它的镜像。
两份实现无法共享同一个头文件，所以用三道防线防漂移：

1. 规范以 docs/protocol.md 为准，人改代码前先改那份文档；
2. 两侧各自断言尺寸与偏移（本文件的 _check_layout，C 侧的 _Static_assert）；
3. scripts/proto_selftest.sh 用固定输入在两侧编码，逐字节对拍。

第 3 道是唯一能真正抓住漂移的——前两道只能抓住"改了尺寸"，抓不住"同尺寸但字段顺序换了"。

crc32 曾是本模块自实现的逐位循环，理由是想让 PC 侧与 C 侧是"两份独立实现"，
对拍时 `crc` 字段本身也在被验证。2026-10-07 改回 zlib.crc32 —— 理由是实测：
自实现是每包 11200 次解释器循环、2407 µs/包，把整条接收路径压到 306 pkt/s，
而 320x240@15fps 需要 1650 pkt/s，即无论链路多干净都收不到完整帧。见 crc32()。
"独立实现"这个顾虑不成立：对拍的另一侧是 C 的 ptp_crc32，zlib 与手写循环本就是
两份独立实现，且 _check_layout 里仍钉着标准向量。
"""

from __future__ import annotations

import dataclasses
import struct
import zlib
from dataclasses import dataclass

CTRL_PORT = 9100
TELEM_PORT = 9101
VIDEO_PORT = 9200

VERSION = 1

CTRL_MAGIC = 0x31544350  # 'PCT1'
TELEM_MAGIC = 0x31544550  # 'PET1'
VIDEO_MAGIC = 0x31565650  # 'PVV1'

MODE_MANUAL = 1
MODE_TRACK = 2
MODE_HOME = 3
MODE_STOP = 4
MODE_HOLD = 5

FLAG_HEARTBEAT = 0x00000001

DRV_UNINIT, DRV_READY, DRV_SLEW, DRV_HOLD = 0, 1, 2, 3
DRV_STATUS_NAME = {
    DRV_UNINIT: "UNINIT",
    DRV_READY: "READY",
    DRV_SLEW: "SLEW",
    DRV_HOLD: "HOLD",
}

CHIP_UNKNOWN, CHIP_ASLEEP, CHIP_RUNNING = 0, 1, 2
CHIP_STATE_NAME = {
    CHIP_UNKNOWN: "UNKNOWN",
    CHIP_ASLEEP: "ASLEEP",
    CHIP_RUNNING: "RUNNING",
}

DST_INIT, DST_MANUAL, DST_TRACK, DST_HOME, DST_LOST, DST_HOLD = 0, 1, 2, 3, 4, 5
DAEMON_STATE_NAME = {
    DST_INIT: "INIT",
    DST_MANUAL: "MANUAL",
    DST_TRACK: "TRACK",
    DST_HOME: "HOME",
    DST_LOST: "LOST",
    DST_HOLD: "HOLD",
}

FMT_YUYV, FMT_MJPEG = 1, 2

VIDEO_HDR_SIZE = 36
VIDEO_PAYLOAD = 1400
VIDEO_DGRAM = VIDEO_HDR_SIZE + VIDEO_PAYLOAD  # 1436 < MTU 1500

# '<' 前缀同时表示小端与"无填充字节"，与 C 侧显式编码的布局一致。
_CTRL_FMT = "<IHHIQIHHI"
_TELEM_FMT = "<IHHIQBB" + "H" * 13 + "I" * 16 + "I" * 4
_VIDEO_FMT = "<IHHHHIQHHHHI"

_ctrl_struct = struct.Struct(_CTRL_FMT)
_telem_struct = struct.Struct(_TELEM_FMT)
_video_struct = struct.Struct(_VIDEO_FMT)

_CTRL_SIZE = 32
_TELEM_SIZE = 128


def crc32(data: bytes) -> int:
    """CRC-32/ISO-HDLC，与 C 侧 ptp_crc32 逐位对应。

    用 zlib 而不是自实现的逐位循环，因为这是接收热路径：VideoHdr.decode 对**每个**
    1400 字节的视频分包都要算一次。原逐位实现在环回基准里占 recv_frame 总耗时的
    93.8%（2407 µs/包），把接收上限压到 306 pkt/s，而 320×240@15fps 需要 1650 pkt/s
    —— 瓶颈在我们自己这边，链路修得再干净也没用。换 zlib 后 20789 pkt/s（12.6 倍余量），
    板端实测 580/580 完整帧。

    换之前做过逐位等价性核对：355 组数据（含本文件自测向量与随机长数据）全部一致。
    `& 0xFFFFFFFF` 在 Python 3 下是冗余的（zlib.crc32 已返回无符号），保留是为了
    让"有符号/无符号"这件事在代码里显式可见——C 侧 ptp_crc32 返回 uint32_t。
    """
    return zlib.crc32(data) & 0xFFFFFFFF


@dataclass
class Ctrl:
    seq: int = 0
    mode: int = MODE_MANUAL
    timestamp_us: int = 0
    flags: int = 0
    pan_tick: int = 0
    tilt_tick: int = 0

    def encode(self) -> bytes:
        # crc 字段（最后一个）先填 0 占位，算完前 28 字节再回填——
        # 与 C 侧 ptp_ctrl_encode 的顺序一致。
        body = _ctrl_struct.pack(
            CTRL_MAGIC,
            VERSION,
            self.mode,
            self.seq & 0xFFFFFFFF,
            self.timestamp_us & 0xFFFFFFFFFFFFFFFF,
            self.flags,
            self.pan_tick,
            self.tilt_tick,
            0,
        )
        return body[:-4] + struct.pack("<I", crc32(body[:-4]))

    @classmethod
    def decode(cls, buf: bytes) -> "Ctrl":
        if len(buf) != _CTRL_SIZE:
            raise ValueError(f"控制包长度 {len(buf)} != {_CTRL_SIZE}")
        fields = _ctrl_struct.unpack(buf)
        if crc32(buf[:-4]) != fields[-1]:
            raise ValueError("控制包 CRC 不符")
        if fields[0] != CTRL_MAGIC:
            raise ValueError(f"控制包魔数 0x{fields[0]:08x} 不符")
        return cls(
            seq=fields[3],
            mode=fields[2],
            timestamp_us=fields[4],
            flags=fields[5],
            pan_tick=fields[6],
            tilt_tick=fields[7],
        )


# 字段表是这里唯一的"顺序真相"：encode 与 decode 都从它派生，
# 所以不可能出现"编码按 A 顺序、解码按 B 顺序"这种内部错位。
# 它必须与 pantilt_proto.h 里 ptp_telem_encode 的写出顺序逐项一致。
_TELEM_U16 = [
    "slew_ms", "wd_ms", "daemon_wd_ms",
    "pan_cur", "tilt_cur", "pan_tgt", "tilt_tgt",
    "pan_home", "tilt_home",
    "pan_safe_min", "pan_safe_max", "tilt_safe_min", "tilt_safe_max",
]
_TELEM_U32 = [
    "ms_since_ctrl", "last_latency_us", "cmd_count",
    "clamp_low_pan", "clamp_low_tilt", "clamp_high_pan", "clamp_high_tilt",
    "i2c_err", "watchdog_trips", "ctrl_rx_ok",
    "rej_crc", "rej_seq", "rej_stale", "rej_mode", "rej_range",
    "daemon_wd_trips",
]
_TELEM_FIELDS = ["drv_status", "seq", "timestamp_us",
                 "chip_state", "daemon_state"] + _TELEM_U16 + _TELEM_U32
_TELEM_MASKS = {
    **{n: 0xFFFF for n in _TELEM_U16},
    **{n: 0xFFFFFFFF for n in _TELEM_U32},
    "seq": 0xFFFFFFFF,
    "timestamp_us": 0xFFFFFFFFFFFFFFFF,
    "drv_status": 0xFFFF,
    "chip_state": 0xFF,
    "daemon_state": 0xFF,
}


@dataclass
class Telem:
    seq: int
    timestamp_us: int
    drv_status: int
    chip_state: int
    daemon_state: int
    slew_ms: int
    wd_ms: int
    daemon_wd_ms: int
    pan_cur: int
    tilt_cur: int
    pan_tgt: int
    tilt_tgt: int
    pan_home: int
    tilt_home: int
    pan_safe_min: int
    pan_safe_max: int
    tilt_safe_min: int
    tilt_safe_max: int
    ms_since_ctrl: int
    last_latency_us: int
    cmd_count: int
    clamp_low_pan: int
    clamp_low_tilt: int
    clamp_high_pan: int
    clamp_high_tilt: int
    i2c_err: int
    watchdog_trips: int
    ctrl_rx_ok: int
    rej_crc: int
    rej_seq: int
    rej_stale: int
    rej_mode: int
    rej_range: int
    daemon_wd_trips: int
    # reserved[4]（板端 C 里是同一块区域，见 pantilt_daemon.c 与 docs/protocol.md）：
    #   reserved[0] = session_resets（Phase 6）
    #   reserved[1] = 振动事件（Phase 7）：高 16 位计数、低 16 位"距上次事件的毫秒数"
    reserved0: int = 0
    reserved1: int = 0

    @classmethod
    def decode(cls, buf: bytes) -> "Telem":
        if len(buf) != _TELEM_SIZE:
            raise ValueError(f"遥测包长度 {len(buf)} != {_TELEM_SIZE}")
        f = _telem_struct.unpack(buf)
        if f[0] != TELEM_MAGIC:
            raise ValueError(f"遥测包魔数 0x{f[0]:08x} 不符")
        t = cls(**dict(zip(_TELEM_FIELDS, f[2:36])))
        t.reserved0, t.reserved1 = f[36], f[37]
        return t

    def encode(self) -> bytes:
        """PC 侧只在自测与回环仿真时编码遥测；权威实现是板端 C。"""
        vals = [getattr(self, n) & _TELEM_MASKS[n] for n in _TELEM_FIELDS]
        return _telem_struct.pack(TELEM_MAGIC, VERSION, *vals,
                                  self.reserved0 & 0xFFFFFFFF,
                                  self.reserved1 & 0xFFFFFFFF, 0, 0)

    # --- reserved[1] 的解读（板端编码见 common/proto/security_event.h） ---

    @property
    def ev_count(self) -> int:
        """securityd 送上来的振动事件累计数（截断到 16 位）。"""
        return (self.reserved1 >> 16) & 0xFFFF

    @property
    def ev_age_ms(self) -> int:
        """距最近一次振动事件的毫秒数；0xFFFF 表示"从来没有过"。"""
        return self.reserved1 & 0xFFFF

    @property
    def ev_fresh(self) -> bool:
        """事件还在 daemon 的 2 s 锁存窗口内。"""
        return self.ev_count > 0 and self.ev_age_ms < 2000

    def summary(self) -> str:
        return (
            f"{DAEMON_STATE_NAME.get(self.daemon_state, '?')}/"
            f"{DRV_STATUS_NAME.get(self.drv_status, '?')} "
            f"chip={CHIP_STATE_NAME.get(self.chip_state, '?')} "
            f"pan {self.pan_cur}->{self.pan_tgt} tilt {self.tilt_cur}->{self.tilt_tgt} "
            f"cmd={self.cmd_count} "
            f"clamp={self.clamp_low_pan}+{self.clamp_low_tilt}/"
            f"{self.clamp_high_pan}+{self.clamp_high_tilt} "
            f"i2c_err={self.i2c_err} wd={self.watchdog_trips}/{self.daemon_wd_trips} "
            f"rej(crc/seq/stale/mode/range)={self.rej_crc}/{self.rej_seq}/"
            f"{self.rej_stale}/{self.rej_mode}/{self.rej_range} "
            f"since_ctrl={self.ms_since_ctrl}ms lat={self.last_latency_us}us"
        )


@dataclass
class VideoHdr:
    chunk_idx: int
    chunk_total: int
    payload_len: int
    frame_id: int
    timestamp_us: int
    width: int
    height: int
    format: int
    flags: int

    @classmethod
    def decode(cls, buf: bytes) -> "VideoHdr":
        if len(buf) < VIDEO_HDR_SIZE:
            raise ValueError("视频包头不完整")
        f = _video_struct.unpack(buf[:VIDEO_HDR_SIZE])
        magic, ver = f[0], f[1]
        chunk_idx, chunk_total, payload_len = f[2], f[3], f[4]
        frame_id, timestamp_us = f[5], f[6]
        width, height, fmt, flags, crc = f[7], f[8], f[9], f[10], f[11]
        if magic != VIDEO_MAGIC:
            raise ValueError(f"视频包魔数 0x{magic:08x} 不符")
        if ver != VERSION:
            raise ValueError(f"视频包版本 {ver} != {VERSION}")
        # 必须先按**声明的载荷长度**核对实际带了多长。
        # 少了这一步的话，下面切片越界会被 Python 静默截断到"恰好剩下的字节"，
        # 于是 CRC 在某些取值下碰巧算对，把错误长度当成合法包放过去。
        if len(buf) < VIDEO_HDR_SIZE + payload_len:
            raise ValueError(f"视频包声明载荷 {payload_len} 字节，"
                             f"实际只带了 {len(buf) - VIDEO_HDR_SIZE} 字节")
        if crc != crc32(buf[VIDEO_HDR_SIZE:VIDEO_HDR_SIZE + payload_len]):
            raise ValueError("视频包 payload CRC 不符")
        return cls(
            chunk_idx=chunk_idx,
            chunk_total=chunk_total,
            payload_len=payload_len,
            frame_id=frame_id,
            timestamp_us=timestamp_us,
            width=width,
            height=height,
            format=fmt,
            flags=flags,
        )


def video_pack(
    frame_id: int,
    chunk_idx: int,
    chunk_total: int,
    timestamp_us: int,
    width: int,
    height: int,
    fmt: int,
    payload: bytes,
) -> bytes:
    """编码一个视频分包。板端 C 侧是权威实现，这里是自检用的镜像。"""
    hdr = _video_struct.pack(
        VIDEO_MAGIC,
        VERSION,
        chunk_idx,
        chunk_total,
        len(payload),
        frame_id,
        timestamp_us,
        width,
        height,
        fmt,
        0,
        crc32(payload),
    )
    return hdr + payload


def _check_layout() -> None:
    """尺寸断言。C 侧有等价断言，任一侧改了字段数/宽度就会在这里先炸。"""
    assert _ctrl_struct.size == _CTRL_SIZE, f"ctrl {_ctrl_struct.size}"
    assert _telem_struct.size == _TELEM_SIZE, f"telem {_telem_struct.size}"
    assert _video_struct.size == VIDEO_HDR_SIZE, f"video {_video_struct.size}"
    # 字段表必须刚好填满 format 里除 magic/version/reserved[4] 之外的每个标量。
    # 漏一个字段或漏写一个 H/I，这里先炸，而不是等到线上解出一堆错位的值。
    assert len(_telem_struct.unpack(bytes(_TELEM_SIZE))) == len(_TELEM_FIELDS) + 2 + 4, (
        f"遥测字段表 {len(_TELEM_FIELDS)} 项 + magic/version + reserved[4] 与 format 的 "
        f"{len(_telem_struct.unpack(bytes(_TELEM_SIZE)))} 个标量不符"
    )
    # reserved[4] 不在字段表里（它们在 decode/encode 里单独搬），所以要从比较中扣掉；
    # 扣掉的只有这两个名字，别的字段漏一个照样炸。
    _declared = {f.name for f in dataclasses.fields(Telem)} - {"reserved0", "reserved1"}
    assert set(_TELEM_FIELDS) == _declared, "字段表与 Telem 声明的字段不一致"
    assert VIDEO_HDR_SIZE + VIDEO_PAYLOAD + 28 <= 1500, "分包超出 MTU"
    # 标准向量（CRC-32/ISO-HDLC 的 check value）。换实现时这一条是最后的守门人：
    # 它钉住的是**算法**，而不是"我们的实现和我们的实现一致"。
    assert crc32(b"123456789") == 0xCBF43926, "crc32 标准向量不符"


_check_layout()
