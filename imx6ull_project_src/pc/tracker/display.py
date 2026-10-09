#!/usr/bin/env python3
"""叠加显示：把目标、误差、云台状态和链路状态画到画面上（计划 §3.7）。

`render()` 是纯函数：给它一帧和一份 `Hud`，它返回一张**新**图，不改入参。
这样它既能在窗口里显示，也能在没显示器时落盘成 PNG 当验收证据——
两条路走的是同一份绘制代码，不会出现"截图和窗口对不上"。

字体只用 ASCII：cv2.putText 画不了中文（Hershey 字体没有 CJK 字形，
中文会变成一串 '?'），所以界面文案一律用 TRACK/LOST/FPS 这类拉丁词。
中文只出现在终端输出和日志里。
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from tracker import Target

# 一律用 BGR 分量序写成元组，别用名字相近但通道序不同的常量。
GREEN = (0, 220, 0)
RED = (0, 0, 230)
YELLOW = (0, 210, 230)
GRAY = (170, 170, 170)
WHITE = (240, 240, 240)
BLACK = (0, 0, 0)

_TEXT_SCALE = 0.38
_FONT = cv2.FONT_HERSHEY_SIMPLEX


@dataclass
class Hud:
    """一帧要展示的全部状态。默认值刻意是"什么都不知道"。

    这样调用方少给字段时的表现是"显示为未知"，而不是显示成 0——
    0 是合法取值（tick=0、fps=0），拿它当"没有数据"会误导操作者。
    """

    # state 是**跟踪**状态（TRACK/LOST/NO-VIDEO），是画面上的主角；
    # telem 是**遥测链路**状态，单独一个字段。
    # 两者分开是必须的：合成一个"优先级更高"的状态字时，没有 daemon 就会
    # 写死 NO-TELEM，把"红物体已经被框住了"这件事整个盖掉——
    # 而"看得见目标"和"能动云台"是两件独立的事，操作者要能分别看到。
    state: str = "NO-VIDEO"
    telem: str = "NO-TELEM"
    target: Target | None = None
    error: tuple[float, float] | None = None
    tick_pan: int | None = None          # 遥测报回的当前 tick
    tick_tilt: int | None = None
    cmd_pan: int | None = None           # 本机下发的命令 tick
    cmd_tilt: int | None = None
    fps: float = 0.0
    frame_id: int | None = None
    link: str = ""                       # 链路/环形缓冲那一行
    note: str = ""                       # 操作提示（手动模式用）


def _ascii(s: str) -> str:
    """把非 ASCII 换成 '?'。

    这是最后一道兜底：调用方本来就不该往 HUD 里塞中文（Hershey 字体没有
    CJK 字形），但"不该"和"不会"是两回事——真塞进来时，cv2.putText 画出的
    是一串随机字形，看起来像字又读不出来，比一串问号难查得多。
    换成 '?' 至少一眼就能看出"这里有个不该出现的字符串"。
    """
    return s if s.isascii() else s.encode("ascii", "replace").decode("ascii")


def _text(img, s, org, color=WHITE, scale=_TEXT_SCALE) -> None:
    """带黑描边的文字：亮背景上白字会糊掉，描边比换颜色更稳。"""
    s = _ascii(s)
    cv2.putText(img, s, org, _FONT, scale, BLACK, 2, cv2.LINE_AA)
    cv2.putText(img, s, org, _FONT, scale, color, 1, cv2.LINE_AA)


def _fmt_pair(pan, tilt) -> str:
    f = lambda v: "--" if v is None else str(v)   # noqa: E731
    return f"P={f(pan)} T={f(tilt)}"


def _text_w(s: str, scale: float = _TEXT_SCALE) -> int:
    return cv2.getTextSize(_ascii(s), _FONT, scale, 1)[0][0]


def _wrap(s: str, width_px: int, scale: float = _TEXT_SCALE) -> list[str]:
    """按**实测**像素宽度折行，优先在空格处断。返回至少一行。

    宽度直接用 cv2.getTextSize 量，不按"平均字宽 × 字符数"估算：
    估出来的值对 "TICK P=377" 这类大写数字串会偏低三成，于是折行判定说
    "放得下"、实际却画到画面外——被裁掉的往往正是行尾的 dup/late/rcvbuf。
    """
    # 先净化再量宽度：量的是原始串、画的是净化后的串时，两者宽度对不上，
    # 折行判定说"放得下"而实际画出去——被裁掉的又是行尾那几个关键字段。
    s = _ascii(s)
    avail = width_px - 8
    if _text_w(s, scale) <= avail:
        return [s]
    out: list[str] = []
    cur = ""
    for word in s.split(" "):
        cand = f"{cur} {word}" if cur else word
        if _text_w(cand, scale) <= avail:
            cur = cand
            continue
        if cur:
            out.append(cur)
            cur = ""
        # 单词自己就超宽（例如一长串没空格的数字）时按宽度硬切
        while _text_w(word, scale) > avail and len(word) > 1:
            k = len(word)
            while k > 1 and _text_w(word[:k], scale) > avail:
                k -= 1
            out.append(word[:k])
            word = word[k:]
        cur = word
    if cur:
        out.append(cur)
    return out


def render(bgr: np.ndarray, hud: Hud) -> np.ndarray:
    """在 bgr 的副本上画 HUD。返回新图，入参不动。"""
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError(f"期望 HxWx3 的 BGR 图，实际 shape={bgr.shape}")
    img = bgr.copy()
    h, w = img.shape[:2]
    cxi, cyi = w / 2.0, h / 2.0

    # 画面中心：十字 + 一个空心圈。云台要做的就是把这个点怼到目标上。
    cv2.line(img, (int(cxi) - 8, int(cyi)), (int(cxi) + 8, int(cyi)), GRAY, 1)
    cv2.line(img, (int(cxi), int(cyi) - 8), (int(cxi), int(cyi) + 8), GRAY, 1)
    cv2.circle(img, (int(cxi), int(cyi)), 12, GRAY, 1)

    t = hud.target
    if t is not None and t.found:
        # 轮廓先画、bbox 后画：两者完全重合时（比如目标是个正方的色块）
        # 后画的会把先画的整个盖住。bbox 是中心点的来源，得让它露在外面。
        if t.contour is not None:
            cv2.drawContours(img, [t.contour], -1, YELLOW, 1)
        if t.bbox is not None:
            x, y, bw, bh = t.bbox
            cv2.rectangle(img, (x, y), (x + bw, y + bh), GREEN, 1)
        if t.cx is not None and t.cy is not None:
            cv2.circle(img, (int(t.cx), int(t.cy)), 3, RED, -1)
            # 中心 ↘ 目标 的连线：误差方向一眼可见，比读数字快。
            cv2.line(img, (int(cxi), int(cyi)), (int(t.cx), int(t.cy)),
                     RED, 1, cv2.LINE_AA)

    # 顶部状态行：跟踪状态（主角，带色）+ 遥测状态 + 帧率
    line = 12
    sx = 4
    _text(img, hud.state, (sx, line),
          GREEN if hud.state == "TRACK" else
          (RED if hud.state == "LOST" else GRAY))
    sx += _text_w(hud.state) + 8
    _text(img, hud.telem, (sx, line),
          GRAY if hud.telem == "TELEM-OK" else RED)
    sx += _text_w(hud.telem) + 8
    _text(img, f"FPS={hud.fps:4.1f}", (sx, line), WHITE)
    line += 12
    _text(img, f"tick {_fmt_pair(hud.tick_pan, hud.tick_tilt)}", (4, line))
    line += 12
    _text(img, f"cmd  {_fmt_pair(hud.cmd_pan, hud.cmd_tilt)}", (4, line))
    line += 12
    if hud.error is not None:
        ex, ey = hud.error
        _text(img, f"err  x={ex:+6.1f} y={ey:+6.1f}", (4, line), YELLOW)
    else:
        _text(img, "err  x=  --   y=  --", (4, line), GRAY)
    line += 12
    if hud.frame_id is not None:
        _text(img, f"frame {hud.frame_id}", (4, line), GRAY)

    # 底部：链路/环形缓冲 + 操作提示。
    # 必须按**画幅宽度**折行，不能靠截断：320 宽在 scale=0.38 下只放得下约
    # 70 个字符，之前那行链路摘要有 90 多字，后半截直接被画到画面外——
    # 而那一截恰好是 dup/late/rcvbuf，正是出问题时最需要看的字段。
    rows: list[tuple[str, tuple]] = []
    if hud.link:
        rows += [(s, GRAY) for s in _wrap(hud.link, w)]
    if hud.note:
        rows += [(s, WHITE) for s in _wrap(hud.note, w)]
    y = h - 6
    for text, color in reversed(rows):
        _text(img, text, (4, y), color)
        y -= 12

    # 目标状态角标：右上角一个大字，扫一眼就知道有没有丢
    tag = "TRACK" if (t is not None and t.found) else "LOST"
    color = GREEN if (t is not None and t.found) else RED
    size = cv2.getTextSize(tag, _FONT, 0.5, 1)[0]
    _text(img, tag, (w - size[0] - 6, 14), color, 0.5)

    return img


def link_line(rx_stats, rcvbuf: int, rb: str = "") -> str:
    """底部那行链路摘要。用 video_rx 的真实计数，不另算一份，
    否则窗口里的数字和 video_rx.py 的日志会对不上。"""
    st = rx_stats
    span = (st.last_us - st.first_us) / 1e6
    pkt = st.datagrams / span if span > 0 else 0.0
    s = (f"vid {st.frames_complete}c/{st.frames_incomplete}i/{st.frames_lost}l "
         f"{st.rate():.1f}fps {st.chunk_loss_pct():.2f}%lost "
         f"{pkt:.0f}pkt/s dup={st.dup_chunks} late={st.late_chunks} "
         f"rcvbuf={rcvbuf // 1024}K")
    return f"{s} | {rb}" if rb else s
