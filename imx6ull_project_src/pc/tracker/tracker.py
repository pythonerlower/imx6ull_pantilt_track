#!/usr/bin/env python3
"""目标定位：HSV 颜色阈值 → 最大有效轮廓 → 中心。

这个文件只负责一件事：把「一帧画面 → 目标中心」做成**唯一的一道缝**。

    计划 §3.4 / §5 要求 `get_target_center(frame)` 是可替换的一层：
    HSV 换成 MeanShift / CamShift / 预训练模型时，只换这里，
    下游的 error → P 控制 → 协议 → 云台 一律不动。

所以下游（main.py、display.py）只准调用 `get_target_center()`，
不准自己碰 cv2 的阈值、形态学或轮廓——一旦有人绕开这道缝去写第二份检测，
"换算法只改一处"就守不住了，而那正是这条设计要求存在的全部理由。

坐标约定：一律以**图像素**为准，原点左上，`cx/cy` 是目标轮廓的外接矩形中心
（不是质心——质心会被形状缺口带偏，而云台要的是"把这块东西框住"）。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

import ptproto as P

# 红色在 HSV 里跨 0° 环绕，所以必须两段一起取，否则只能抓到深红或浅红的一半。
# H 用 OpenCV 的 0..179 量纲。
#
# 第二段的**下界 155 是标定出来的，别改回 170**（2026-10-07，735 帧真机数据）：
#
#   板端 OV5640 的自动白平衡会漂移。白平衡一漂，"红"目标的实测色相就从
#   H≈172 掉到 H≈160–168——正好落进 [0,10] 与 [170,180] 之间的**空档**，
#   于是画面里一个又大又明显的红卡片被判成 LOST。实测：只把下界从 170
#   挪到 155，676 帧真目标里的漏检从 26 帧变成 0 帧，负样本误报不变。
#
# H=150 是纯品红。取 155 是留 5 个单位的余量（实测最低掉到 160 附近），
# 同时不把饱和品红整体放进来。红→品红的整个过渡区（H 155..180）本来就
# 属于"红"这个色系，原来的 [170,180] 窄得没有道理。
#
# 下限（S>=80, V>=60）保持偏松：现场光照未标定时，松了最多多抓干扰，
# 紧了直接丢目标——丢目标会让云台进 HOLD，验收时看起来像"跟踪器坏了"。
DEFAULT_RANGES = (
    ((0, 80, 60), (10, 255, 255)),
    ((155, 80, 60), (180, 255, 255)),
)

# 500 而不是 300，同样是标定出来的（同一批 735 帧）。
# 300 定在噪声底**之下**：画面右下那张木桌的近端暖边稳定产出一块
# 194..372 px 的"红色"，于是 59 帧确认无目标的真实负样本里有 30 帧误报。
# 实测负样本面积排下来是 2380/1464/694/449/448…，而真目标最小 540——
# 500 落在这个空隙里：负样本噪声底 449 < 500 < 540 真目标下界。
# 例外是目标刚出画后的头 3 帧（见 logs/phase3/20261007_3c_calib.md）。
DEFAULT_MIN_AREA = 500
DEFAULT_MORPH_KERNEL = 5


@dataclass
class Target:
    """一次检测的结果。`found=False` 时其余字段一律为 None，调用方不必再判。"""

    found: bool
    cx: float | None = None
    cy: float | None = None
    area: float = 0.0
    bbox: tuple[int, int, int, int] | None = None   # x, y, w, h
    contour: np.ndarray | None = None

    def error(self, width: int, height: int) -> tuple[float, float] | None:
        """目标相对画面中心的偏差，单位像素、右/下为正。

        这是 3D 控制律的输入（`error_x = target_x - image_center_x`）。
        没找到目标时返回 None，而不是返回 (0, 0)——0 意味着"已经很准了，
        别动"，与"看不见目标"是完全相反的结论，混在一起会让云台
        对着空气稳如泰山。
        """
        if not self.found or self.cx is None or self.cy is None:
            return None
        return (self.cx - width / 2.0, self.cy - height / 2.0)


@dataclass
class ColorCfg:
    """检测参数。全部可配的理由见 DEFAULT_RANGES 上的说明。"""

    ranges: tuple = DEFAULT_RANGES
    min_area: float = DEFAULT_MIN_AREA
    morph_kernel: int = DEFAULT_MORPH_KERNEL


def to_bgr(frame) -> np.ndarray:
    """把一帧解码成 HxWx3 的 BGR。

    收下两种东西：`video_rx.Frame`（板端来的 YUYV）和已经是 BGR 的 ndarray
    （单测里直接造图用）。板端帧不在这里判 `complete`——残缺帧的判断属于
    video_rx，那是它的职责；这里只解码，拿到半帧就会被解出错位的图，
    所以在调用方就要拦掉。
    """
    if isinstance(frame, np.ndarray):
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(f"期望 HxWx3 的 BGR 图，实际 shape={frame.shape}")
        return frame

    fmt = getattr(frame, "format", None)
    if fmt != P.FMT_YUYV:
        # 不静默返回黑图：格式对不上时，下游会看到"永远找不到目标"，
        # 而真因是解码走错了路，那种失败最难查。
        raise ValueError(
            f"不支持的视频格式 {fmt!r}（只实现 YUYV；板端当前就是 YUYV）")

    raw = np.frombuffer(frame.data, dtype=np.uint8)
    h, w = frame.height, frame.width
    if h == 0 or w == 0:
        raise ValueError("帧的 width/height 为 0，无法解码")

    stride = len(raw) // h
    if stride < w * 2:
        raise ValueError(
            f"帧数据不足：{len(raw)} 字节 / {h} 行 = {stride} < {w * 2}")

    if stride != w * 2:
        # bytesperline 可能被对齐填充（板端 320 宽时恰好不填充，但不能假设）。
        # 按行裁掉填充再交给 cvtColor，否则第 2 行起会整体错位。
        raw = raw[:h * stride].reshape(h, stride)[:, :w * 2].copy()

    return cv2.cvtColor(raw.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUYV)


def _mask_from_ranges(hsv: np.ndarray, cfg: ColorCfg) -> np.ndarray:
    mask = None
    for lo, hi in cfg.ranges:
        part = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = part if mask is None else cv2.bitwise_or(mask, part)
    return mask


def get_target_center(frame, cfg: ColorCfg | None = None) -> Target:
    """一帧 → 目标中心。

    **这是替换检测算法的唯一接口。** 换成 CamShift/CSRT/预训练模型时，
    保持签名与 `Target` 的语义不变即可，调用方（display、3D 控制律）不用改。

    frame: `video_rx.Frame`（YUYV）或 HxWx3 的 BGR ndarray
    """
    cfg = cfg or ColorCfg()
    bgr = to_bgr(frame)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = _mask_from_ranges(hsv, cfg)

    # 开运算去孤立噪点、闭运算补轮廓里的洞。
    #
    # 顺序（开→闭）**不是** 2026-10-07 那轮漏检的原因——这一点当年判错过一次，
    # 别再改回去。在那批 735 帧真机上量过：只做开运算后的最大轮廓 / 原始掩膜
    # 最大轮廓，中位数 0.99、p10 0.95，**零帧损失超过 50%**。当时"开运算把掩膜
    # 啃掉 78%"是在两张画过 HUD 的手挑 PNG 上量的，不具普遍性。
    # 真正的病根是上面 DEFAULT_RANGES 里那个色相空档。
    if cfg.morph_kernel > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                      (cfg.morph_kernel, cfg.morph_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return Target(found=False)

    best = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(best))
    if area < cfg.min_area:
        # 面积不够就当作没有目标。这里**不**返回那个小轮廓：
        # 一个几十像素的色点被当成目标，会让云台朝噪点转过去。
        return Target(found=False, area=area)

    x, y, w, h = cv2.boundingRect(best)
    return Target(found=True, cx=x + w / 2.0, cy=y + h / 2.0,
                  area=area, bbox=(x, y, w, h), contour=best)
