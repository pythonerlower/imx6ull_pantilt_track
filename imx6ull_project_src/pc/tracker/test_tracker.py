#!/usr/bin/env python3
"""tracker 的单测：合成图上验几何精度，真实帧上验不误报。

为什么合成图是正当的：颜色检测的**几何**部分（阈值 → 掩膜 → 形态学 →
最大轮廓 → 外接矩形中心）与光照无关，可以精确验证——已知位置的色块必须
给出已知的中心，误差本身就有意义。而光照、白平衡、真实干扰只能上真机，
所以另一半用板端落盘的真实帧做**负样本**：那个场景里没有红色目标，
正确的行为是 `found=False`，而不是硬凑出一个中心出来。

    python3 test_tracker.py
"""

from __future__ import annotations

import glob
import os

import cv2
import numpy as np

import ptproto as P
from tracker import ColorCfg, Target, get_target_center, to_bgr

W, H = 320, 240


class FakeFrame:
    """冒充 video_rx.Frame——只需要 tracker 会读的那几个属性。"""

    def __init__(self, yuyv: bytes, width=W, height=H, fmt=P.FMT_YUYV):
        self.data = yuyv
        self.width = width
        self.height = height
        self.format = fmt


def blank() -> np.ndarray:
    """中灰底。不是纯黑：纯黑下 S/V 全为 0，颜色阈值的行为会退化。"""
    return np.full((H, W, 3), 128, np.uint8)


def draw_blob(img, cx, cy, radius, bgr):
    cv2.circle(img, (int(cx), int(cy)), int(radius), bgr, -1)


def bgr_of(hsv_lo_deg: int, s=220, v=220) -> tuple[int, int, int]:
    """按 OpenCV 的 H(0..179) 造一个 BGR 颜色，方便直接点名色相。"""
    px = np.uint8([[[hsv_lo_deg, s, v]]])
    b, g, r = cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0]
    return int(b), int(g), int(r)


RED = bgr_of(0)          # 红：跨 0°，双区间要能一起抓到
RED_HIGH = bgr_of(175)   # 另一端红，只有第二段区间能抓到

# 标定帧集：带真值标签的真机原始帧，见 logs/phase3/20261007_3c_calib.md
CALIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "..", "..", "logs", "phase3",
                         "20261007_3c_calib")

# 目标刚出画后的头 3 帧：画左那件**静态红衣服**会被顶上来。这三帧在颜色、
# 面积、形状上和真目标无法区分，任何全局阈值都做不到——处理它的时域迟滞
# 属于 3D 控制层的职责（已与操作者协商，见标定记录 §7）。列在这里是为了
# 让"它还存在"这件事留在代码里，而不是靠记忆。
EXIT_TAIL_KNOWN_FP = {479, 480, 481}


def _calib_frames(sub: str) -> list[str]:
    return sorted(glob.glob(os.path.join(CALIB_DIR, sub, "*.yuyv")))


def _fid_of(path: str) -> int:
    return int(os.path.basename(path).split("_")[1])


def _detect_file(path: str):
    with open(path, "rb") as fp:
        return get_target_center(FakeFrame(fp.read(), width=W, height=H))


# ---------------------------------------------------------------- 几何精度

def test_center_at_multiple_positions():
    """色块放哪，中心就该报哪。四角 + 正中 + 贴边，逐个查。"""
    spots = [(160, 120), (60, 60), (260, 60), (60, 180), (260, 180),
             (40, 120), (280, 120), (160, 40), (160, 200)]
    for cx, cy in spots:
        img = blank()
        draw_blob(img, cx, cy, 20, RED)
        t = get_target_center(img)
        assert t.found, f"({cx},{cy}) 处的色块没被找到"
        assert abs(t.cx - cx) <= 1.0 and abs(t.cy - cy) <= 1.0, (
            f"({cx},{cy}) 报成 ({t.cx:.1f},{t.cy:.1f})")


def test_picks_largest_blob():
    """两个同色块，取大的那个——小的可能是背景里的杂物。"""
    img = blank()
    draw_blob(img, 70, 120, 12, RED)
    draw_blob(img, 240, 100, 30, RED)
    t = get_target_center(img)
    assert t.found
    assert abs(t.cx - 240) <= 2 and abs(t.cy - 100) <= 2, \
        f"应取大块 (240,100)，实际 ({t.cx:.1f},{t.cy:.1f})"
    assert t.area > 3.14 * 25 ** 2, f"面积 {t.area} 不像半径 30 的圆"


def test_bbox_matches_blob():
    img = blank()
    draw_blob(img, 100, 150, 25, RED)
    t = get_target_center(img)
    x, y, w, h = t.bbox
    assert abs(w - h) <= 2, f"圆的 bbox 应近似正方形，实际 {w}x{h}"
    assert x <= 100 - 25 + 2 and x + w >= 100 + 25 - 2, "bbox 没框住色块"


# ---------------------------------------------------------------- 边界与拒绝

def test_no_target_found():
    img = blank()
    t = get_target_center(img)
    assert not t.found, "空白画面不该找到目标"
    assert t.cx is None and t.cy is None and t.bbox is None
    assert t.error(W, H) is None


def test_tiny_blob_rejected():
    """低于 min_area 的色点必须当作没目标，否则云台会朝噪点转过去。"""
    img = blank()
    draw_blob(img, 50, 50, 3, RED)          # 约 28 像素
    t = get_target_center(img)
    assert not t.found, f"半径 3 的色点被当成了目标（area={t.area}）"
    assert t.area > 0, "被拒时也应把面积报出来，便于标定阈值"


def test_wrong_color_rejected():
    """蓝色不能触发红色阈值。"""
    img = blank()
    draw_blob(img, 160, 120, 30, bgr_of(110))
    assert not get_target_center(img).found


def test_red_wraps_around_zero():
    """红色的两段阈值缺一不可：H≈175 与 H≈2 都要抓得到。"""
    for color, name in ((RED, "H≈0"), (RED_HIGH, "H≈175")):
        img = blank()
        draw_blob(img, 160, 120, 25, color)
        t = get_target_center(img)
        assert t.found, f"{name} 的红色没被找到——红色双区间少了一段"


def test_magenta_tinted_red_is_found():
    """偏品红的红块（H≈160–168）必须抓得到——色相第二段的下界是 155。

    2026-10-07 标定出来的：板端 OV5640 的自动白平衡会漂移，一漂，红目标的
    实测色相就从 H≈172 掉到 H≈165。而当时两段阈值是 [0,10] 与 [170,180]，
    中间 11..169 是**空的**，于是一个又大又明显的红卡片被判成 LOST——
    实测 676 帧真目标里有 26 帧漏检。下界挪到 155 之后漏检为 0。

    这条就是钉住那次修正的：谁要是把下界改回 170，这里立刻红。
    """
    for hue in (160, 162, 165, 168):
        img = blank()
        draw_blob(img, 160, 120, 30, bgr_of(hue))
        t = get_target_center(img)
        assert t.found, (f"H={hue} 的偏品红红块没被找到——"
                         f"色相第二段下界可能被改回了 170")


def test_min_area_sits_above_the_measured_noise_floor():
    """噪声底那一档的红块必须被拒，再大一档必须抓到。

    2026-10-07 那批 735 帧的真机上：画面右下木桌的近端暖边会**稳定**产出
    一块 194..372 px 的红色掩膜（S≈175、V≈180，和真目标同色系），而真目标
    最小 540 px。min_area=300 定在噪声底**之下**，于是 59 帧确认无目标的
    负样本里 30 帧误报；抬到 500 之后只剩 3 帧（那 3 帧见上面那个已知例外）。

    这里用半径 10 的圆（约 280 px，落在噪声底那一档）和半径 15 的圆
    （约 660 px，落在真目标那一档）把这段余量钉住。
    """
    small = blank()
    draw_blob(small, 160, 120, 10, RED)
    t = get_target_center(small)
    assert not t.found, (f"半径 10（约 280 px，木桌暖边那一档）被当成了目标，"
                         f"area={t.area:.0f}")

    big = blank()
    draw_blob(big, 160, 120, 15, RED)
    t = get_target_center(big)
    assert t.found, f"半径 15（约 660 px）没被找到，area={t.area:.0f}"


# ---------------------------------------------------------------- 标定帧集

def test_calib_negative_frames_do_not_false_positive():
    """标定帧集里那 59 帧**确认无目标**的真实负样本，不许误报。

    这批帧比 test_real_frames_do_not_false_positive 用的那 10 帧严苛得多：
    那个场景里根本不是红色环境，而这 59 帧里有两个**同色系的静态干扰物**
    ——木桌近端的暖边、画左那件红衣服。它们才是真正会咬人的东西。
    """
    files = _calib_frames("frames_neg")
    if not files:
        print("        [SKIP] 没有标定负样本帧（logs/phase3/20261007_3c_calib）")
        return

    bad, known = [], []
    for path in files:
        fid = _fid_of(path)
        t = _detect_file(path)
        if not t.found:
            continue
        if fid in EXIT_TAIL_KNOWN_FP:
            known.append(fid)
        else:
            bad.append(f"{fid}(area={t.area:.0f})")
    if known:
        print(f"        [INFO] 出画后头 3 帧的已知误报命中 {sorted(known)}"
              f"（留给 3D 时域迟滞，见标定记录 §7）")
    assert not bad, f"{len(bad)}/{len(files)} 帧无目标的真实帧被误报: {bad[:6]}"


def test_calib_positive_frames_are_all_found():
    """标定帧集里的正样本（含目标最小、最偏色、运动模糊的那些）必须全部命中。

    这些帧是 2026-10-07 那一轮被误判成 LOST 的主力：目标侧棱朝镜头时红色只剩
    一条窄缝、白平衡漂移时色相掉到 160 出头。它们的存在就是为了防止有人
    为了让负样本好看而把阈值收紧——收紧的代价现场会以"目标明明在却跟丢"出现。
    """
    files = _calib_frames("frames_pos")
    if not files:
        print("        [SKIP] 没有标定正样本帧（logs/phase3/20261007_3c_calib）")
        return

    missed = [_fid_of(p) for p in files if not _detect_file(p).found]
    assert not missed, f"正样本里 {len(missed)}/{len(files)} 帧漏检: {missed[:12]}"


# ---------------------------------------------------------------- 真实帧

def test_real_frames_do_not_false_positive():
    """板端落盘的真实帧：那个场景里没有红色目标，正确行为是 found=False。

    这是本文件里唯一"防幻觉"的一条：阈值放宽之后，最可能的坏结局不是
    找不到目标，而是把木柜、白袋子上的红字之类的东西报成目标。
    """
    pats = [
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "logs",
                     "phase3", "*", "frames", "*.yuyv"),
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "logs",
                     "video_link_*", "*frames*", "*.yuyv"),
    ]
    files = sorted({f for p in pats for f in glob.glob(p)})
    if not files:
        print("        [SKIP] 没有可用的真实帧（logs/phase3/*/frames/*.yuyv）")
        return

    hit = 0
    for path in files:
        raw = open(path, "rb").read()
        h, w = 240, 320
        ff = FakeFrame(raw, width=w, height=h)
        t = get_target_center(ff)
        if t.found:
            hit += 1
            print(f"        [INFO] {os.path.basename(path)} 报出目标 "
                  f"({t.cx:.0f},{t.cy:.0f}) area={t.area:.0f}")
    assert hit == 0, f"{hit}/{len(files)} 张真实帧被误报成有目标"


def test_yuyv_path_matches_bgr_path():
    """YUYV 解码这一层不能自己跑偏。

    板端帧走的是 YUYV；若这里解码错位，检测结果会和"看着同一张图的 BGR"
    对不上，而现象只是"中心老是偏一点"，很难联想到是解码。
    色度二次采样会带来误差，所以容差放到 2 像素。
    """
    img = blank()
    draw_blob(img, 90, 170, 30, RED)
    bgr_target = get_target_center(img)
    assert bgr_target.found

    # cvtColor 的 YUYV 输出是 (H, W, 2) 而不是单通道打包——
    # 这正是 to_bgr 里 reshape(h, w, 2) 的由来。
    yuyv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV_YUYV).tobytes()
    ff = FakeFrame(yuyv)
    assert len(ff.data) == W * H * 2, f"YUYV 长度 {len(ff.data)} 不对"
    yuyv_target = get_target_center(ff)
    assert yuyv_target.found, "YUYV 路径没找到同一个色块"
    assert abs(yuyv_target.cx - bgr_target.cx) <= 2, \
        f"x 偏差 {yuyv_target.cx - bgr_target.cx:.1f}"
    assert abs(yuyv_target.cy - bgr_target.cy) <= 2, \
        f"y 偏差 {yuyv_target.cy - bgr_target.cy:.1f}"


def test_padded_stride_decoded_correctly():
    """bytesperline 被对齐填充时按行裁剪，否则第 2 行起整体错位。"""
    img = blank()
    draw_blob(img, 160, 120, 25, RED)
    packed = cv2.cvtColor(img, cv2.COLOR_BGR2YUV_YUYV).reshape(H, W * 2)
    stride = W * 2 + 16                                   # 人为造出填充
    padded = np.zeros((H, stride), np.uint8)
    padded[:, :W * 2] = packed
    t = get_target_center(FakeFrame(padded.tobytes()))
    assert t.found, "带填充的帧没能找到目标"
    assert abs(t.cx - 160) <= 2 and abs(t.cy - 120) <= 2, \
        f"带填充时中心偏到 ({t.cx:.1f},{t.cy:.1f})"


# ---------------------------------------------------------------- 接口契约

def test_unsupported_format_raises():
    """MJPEG 不该静默返回黑图——那会变成"永远找不到目标"，真因却看不出来。"""
    ff = FakeFrame(b"\x00" * 16, fmt=P.FMT_MJPEG)
    try:
        to_bgr(ff)
    except ValueError as exc:
        assert "MJPEG" in str(exc) or "格式" in str(exc), str(exc)
        return
    raise AssertionError("非 YUYV 格式没有报错")


def test_error_sign_and_absence():
    """error 的符号决定 3D 是负反馈还是正反馈，这里先把语义钉死。"""
    img = blank()
    draw_blob(img, 260, 60, 20, RED)          # 右上
    t = get_target_center(img)
    ex, ey = t.error(W, H)
    assert ex > 0 and ey < 0, \
        f"右上目标应给出 (正,负)，实际 ({ex:.1f},{ey:.1f})"

    empty = Target(found=False)
    assert empty.error(W, H) is None, "没找到目标时必须是 None，不能是 (0,0)"


def test_cfg_overrides_threshold():
    """阈值可配是硬要求——现场光照下的最终值只能实机标定。"""
    img = blank()
    draw_blob(img, 160, 120, 25, bgr_of(110))     # 蓝
    assert not get_target_center(img).found
    blue_cfg = ColorCfg(ranges=(((105, 80, 60), (125, 255, 255)),))
    t = get_target_center(img, blue_cfg)
    assert t.found, "换成蓝色阈值后仍找不到蓝色色块"


def main() -> int:
    print("tracker 单测")
    tests = [
        test_center_at_multiple_positions,
        test_picks_largest_blob,
        test_bbox_matches_blob,
        test_no_target_found,
        test_tiny_blob_rejected,
        test_wrong_color_rejected,
        test_red_wraps_around_zero,
        test_magenta_tinted_red_is_found,
        test_min_area_sits_above_the_measured_noise_floor,
        test_calib_negative_frames_do_not_false_positive,
        test_calib_positive_frames_are_all_found,
        test_real_frames_do_not_false_positive,
        test_yuyv_path_matches_bgr_path,
        test_padded_stride_decoded_correctly,
        test_unsupported_format_raises,
        test_error_sign_and_absence,
        test_cfg_overrides_threshold,
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
