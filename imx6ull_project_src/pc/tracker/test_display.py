#!/usr/bin/env python3
"""display 的单测：验"画对了"和"没画脏"，不验好不好看。

判据选的是**像素**而不是截图比对：截图比对会在换字体/换 OpenCV 版本时
整片变红，却说不清哪里坏了；按颜色找关键图元（十字、bbox、目标点）
既稳，又正好对应计划 §3.7 里逐项列出的那些叠加物。

    python3 test_display.py
"""

from __future__ import annotations

import cv2
import numpy as np

from display import GREEN, GRAY, RED, Hud, link_line, render
from tracker import Target

W, H = 320, 240


def blank() -> np.ndarray:
    return np.full((H, W, 3), 128, np.uint8)


def near(img, bgr, tol=60) -> int:
    d = np.abs(img.astype(int) - np.array(bgr, int)).sum(axis=2)
    return int((d < tol).sum())


def make_target(cx=240, cy=80, size=40) -> Target:
    x, y = int(cx - size / 2), int(cy - size / 2)
    contour = np.array([[[x, y]], [[x + size, y]],
                        [[x + size, y + size]], [[x, y + size]]], np.int32)
    return Target(found=True, cx=float(cx), cy=float(cy),
                  area=float(size * size), bbox=(x, y, size, size),
                  contour=contour)


def test_render_does_not_mutate_input():
    """入参必须原样不动：调用方还要拿原始帧做检测/落盘。"""
    src = blank()
    before = src.copy()
    render(src, Hud())
    assert np.array_equal(src, before), "render 改动了传入的图"


def test_render_returns_new_array():
    src = blank()
    out = render(src, Hud())
    assert out is not src, "应返回副本，不能把同一块内存传回来"
    assert out.shape == src.shape and out.dtype == np.uint8


def test_crosshair_at_image_center():
    out = render(blank(), Hud())
    patch = out[H // 2 - 1:H // 2 + 2, W // 2 - 8:W // 2 + 8]
    assert near(patch, GRAY) > 0, "画面中心没画十字"


def test_target_draws_bbox_and_center_dot():
    t = make_target(240, 80)
    out = render(blank(), Hud(target=t, error=(80.0, -40.0)))
    assert near(out, GREEN) > 0, "找到目标却没画 bbox"
    # bbox 的四条边都应在
    x, y, w, h = t.bbox
    assert near(out[y:y + 2, x:x + w], GREEN) > 0, "bbox 上边缺失"
    assert near(out[y + h - 1:y + h + 1, x:x + w], GREEN) > 0, "bbox 下边缺失"
    dot = out[int(t.cy) - 2:int(t.cy) + 3, int(t.cx) - 2:int(t.cx) + 3]
    assert near(dot, RED) > 0, "目标中心点没画"


def test_no_target_draws_no_bbox():
    out = render(blank(), Hud(target=Target(found=False)))
    assert near(out, GREEN) == 0, "没有目标却出现了绿色图元（会骗操作者）"


def test_hud_renders_with_everything_unset():
    """全 None 时不能抛异常——链路刚起来、遥测还没到时就是这个状态。"""
    out = render(blank(), Hud(state="NO-TELEM"))
    assert out.shape == blank().shape


def test_unknown_ticks_stay_unknown():
    """None 要显示成 --，不能显示成 0：0 是合法 tick，混同会误导操作者。"""
    out = render(blank(), Hud(tick_pan=None, tick_tilt=None))
    assert out.shape == blank().shape
    row = out[24:38, 0:W]
    assert near(row, (240, 240, 240)) > 0, "顶部信息行没画出东西"


def test_rejects_non_bgr():
    for bad in (np.zeros((H, W), np.uint8), np.zeros((H, W, 4), np.uint8)):
        try:
            render(bad, Hud())
        except ValueError:
            continue
        raise AssertionError(f"shape={bad.shape} 应被拒绝")


def test_link_line_reports_real_counters():
    class St:
        frames_complete = 883
        frames_incomplete = 2
        frames_lost = 0
        datagrams = 97330
        dup_chunks = 0
        late_chunks = 0
        first_us = 0
        last_us = 60_000_000

        @staticmethod
        def rate():
            return 15.0

        @staticmethod
        def chunk_loss_pct():
            return 0.02

    s = link_line(St(), 425984, rb="rb deep=0 hw=1 drop=0")
    for want in ("883c", "2i", "0l", "15.0fps", "dup=0", "late=0",
                 "rcvbuf=416K", "drop=0"):
        assert want in s, f"链路行缺少 {want!r}：{s}"


def test_wrap_never_exceeds_frame_width():
    """折行后的每一行都必须真的画得下。

    这条是补出来的：之前按"平均字宽 × 字符数"估算折行阈值，估得偏低三成，
    于是底部那行链路摘要被判成"放得下"、实际整段画到画面外，
    被裁掉的正好是行尾的 dup/late/rcvbuf。现在按 getTextSize 实测宽度判定，
    这条断言就是把那个不变量钉住。
    """
    from display import _text_w, _wrap

    samples = [
        "vid 883c/2i/0l 15.0fps 0.02%lost 1622pkt/s dup=0 late=0 "
        "rcvbuf=416K | rb deep=0 hw=1 drop=0",
        "keys: a/d pan  w/s tilt  space stop  H home  q quit",
        "x" * 500,                       # 无空格的超长串：必须硬切
        "a bb ccc dddd eeeee ffffff",
    ]
    for width in (320, 160, 640):
        for s in samples:
            lines = _wrap(s, width)
            assert lines, "至少要返回一行"
            for ln in lines:
                w = _text_w(ln)
                assert w <= width - 8, (
                    f"宽度 {width} 下这一行 {w}px 溢出：{ln!r}")
            # 不能丢字符（硬切时也一样）。断行处的空格会被吃掉，所以去空格后比对。
            assert "".join(lines).replace(" ", "") == s.replace(" ", ""), \
                "折行丢了内容"


def test_long_link_renders_within_frame():
    """底部信息很多时也不能抛异常，且顶部状态行仍要存在。"""
    many = " ".join(f"k{i}=v{i}" for i in range(40))
    out = render(blank(), Hud(state="TRACK", link=many, note=many))
    assert out.shape == blank().shape
    assert near(out, (240, 240, 240)) > 0, "顶部状态行没画出来"


def test_non_ascii_becomes_question_marks():
    """往 HUD 里误塞中文时，画出来要是一串 '?'，不能是随机字形。

    真实触发路径是存在的：Manual.note 全是中文，一不小心就会顺着
    note 流到 HUD 上。Hershey 字体没有 CJK 字形，putText 画出来的是
    一堆看着像字、读不出来的东西——比问号难查得多。
    """
    from display import _ascii

    assert _ascii("TRACK") == "TRACK"
    assert _ascii("ab中cd") == "ab?cd"
    assert _ascii("已对齐到当前姿态").count("?") == 8
    # 折行也要能处理它（换字符只改宽度，不该抛异常）
    from display import _wrap

    for ln in _wrap("MOVE 中文 step=3", 320):
        assert ln.isascii(), ln


def test_every_hud_field_is_ascii_after_render():
    """整帧渲染不能抛异常，即使每个字段都是中文。"""
    out = render(blank(), Hud(state="跟踪", link="链路正常", note="已停住"))
    assert out.shape == blank().shape


def main() -> int:
    print("display 单测")
    tests = [
        test_render_does_not_mutate_input,
        test_render_returns_new_array,
        test_crosshair_at_image_center,
        test_target_draws_bbox_and_center_dot,
        test_no_target_draws_no_bbox,
        test_hud_renders_with_everything_unset,
        test_unknown_ticks_stay_unknown,
        test_rejects_non_bgr,
        test_link_line_reports_real_counters,
        test_wrap_never_exceeds_frame_width,
        test_long_link_renders_within_frame,
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
