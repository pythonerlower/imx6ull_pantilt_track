"""Phase 7 事件巡检的单测：不碰硬件、不碰舵机。

只验 `Patrol` 这一层——「事件上升沿 / 一帧检测结果 → 一次 set_target 调用」。
要钉住的是 plan/phase7_wom_patrol_debug.md §3 那两条底线，外加几条
"错了就会以最难看的方式表现出来"的性质：

  1. **扫描行程恒在 [lo, hi] 内，且绝不停在机械上限 515。**
     2.51 ms/tick 已超出 MG90S 规格，停在那儿是慢慢烧舵机，
     而不是马上出错——最典型的事后归因失败。
  2. **超时一定到点，且到点回 home。** 停在扫描端点是最坏的结局：
     既没在找目标，也不在守位。
  3. **第一圈只记录计数、不触发。** 否则刚起来的进程会拿板端的历史计数
     凭空巡检一次，而那次事件可能发生在半小时前。
  4. **巡检握着控制权时闭环一帧都不许动**（两个写者抢同一条轴 = 谁都没发）。
     这一条由 run_video 的 if/elif 结构保证，这里只验 Patrol 自己的 active。
  5. **找到目标要连续 N 帧**，一帧噪声不算数——否则会在噪声上把控制权交出去，
     闭环随即在没目标的画面上乱转。

    python3 test_patrol.py
"""

from __future__ import annotations

import dataclasses

import ptproto as P
from main import Patrol
from test_auto import FakeCli, FakeManual, LOST, found
from tracker import Target

W, H = 320, 240


@dataclasses.dataclass
class FakeTelem:
    """只带 Patrol 会读的字段。真 Telem 解出来的包在 test_control.py 里验。"""

    pan_cur: int = 377
    tilt_cur: int = 242
    pan_home: int = 377
    tilt_home: int = 242
    pan_safe_min: int = 239
    pan_safe_max: int = 515
    tilt_safe_min: int = 162
    tilt_safe_max: int = 342
    ev_count: int = 0
    watchdog_trips: int = 0
    daemon_wd_trips: int = 0


_UNSET = object()


def mk(telem=_UNSET, **kw):
    # 显式传 telem=None 表示"没有遥测"，别和"用默认值"混在一起。
    m = FakeManual(FakeTelem() if telem is _UNSET else telem)
    kw.setdefault("max_s", 20.0)
    kw.setdefault("lo", 270)
    kw.setdefault("hi", 490)
    kw.setdefault("step", 24)
    return m, Patrol(m, **kw)


def modes(m):
    return [c[0] for c in m.cli.calls]


# ---------------------------------------------------------------- 触发

def test_first_reading_never_triggers():
    """刚起来的进程不该拿板端的历史计数凭空巡检一次。"""
    m, p = mk()
    assert p.trigger(7, 0.0) is False
    assert p.state == "IDLE"
    assert m.cli.calls == [], "第一圈只该记录计数"


def test_rising_edge_triggers():
    m, p = mk()
    p.trigger(7, 0.0)
    assert p.trigger(8, 0.1) is True
    assert p.active
    assert p.runs == 1
    assert m.cli.calls, "触发后立刻要站到起点（home）"


def test_flat_counter_does_not_retrigger():
    m, p = mk()
    p.trigger(7, 0.0)
    p.trigger(8, 0.1)
    n = len(m.cli.calls)
    for _ in range(5):
        assert p.trigger(8, 0.2) is False
    assert len(m.cli.calls) == n, "计数没变就不该有新动作"
    assert p.runs == 1


def test_event_during_patrol_extends_deadline_without_restarting():
    m, p = mk()
    p.trigger(0, 0.0)
    p.trigger(1, 1.0)
    t0 = p.t0
    assert p.trigger(2, 15.0) is False
    assert p.runs == 1, "巡检途中又来事件不该重开一轮"
    assert p.t0 == 15.0 and p.t0 != t0, "预算该顺延到现在"


def test_no_telemetry_refuses_to_start():
    """没有可信的 cur 就不发目标——和闭环同一条守则。"""
    m, p = mk(telem=None)
    p.trigger(0, 0.0)
    assert p.trigger(1, 0.1) is False
    assert p.state == "IDLE"
    assert m.cli.calls == []
    assert m.refused > 0


# ---------------------------------------------------------------- 底线 1：行程

def test_sweep_never_leaves_the_configured_range():
    """走满好几轮，每一次下发的 pan 都必须落在 [lo, hi] 里。"""
    m, p = mk(step=60)
    p.trigger(0, 0.0)
    p.trigger(1, 0.1)
    now = 0.1
    for _ in range(400):
        now += 0.05
        m.cli._pan = m.cli._pan  # cur 跟随目标：让每次 DWELL 都能到位
        telem = m._t
        telem.pan_cur = p.cmd_pan
        p.update(LOST, W, H, now=now)
    pans = [c[1] for c in m.cli.calls if c[1] is not None]
    assert pans, "应该下发过目标"
    assert min(pans) >= p.lo, f"扫到 {min(pans)} < lo={p.lo}"
    assert max(pans) <= p.hi, f"扫到 {max(pans)} > hi={p.hi}"


def test_hard_ceiling_is_514_not_515():
    """515 是机械极限，可以路过但**不许停**。hi 超界时必须被夹回来。"""
    assert Patrol.PAN_HARD_MAX == 514
    m, p = mk(hi=600)
    assert p.hi == 514, "hi 必须夹到硬上限"


def test_range_is_clamped_by_driver_safe_range():
    t = FakeTelem(pan_safe_min=300, pan_safe_max=420)
    m, p = mk(telem=t, lo=270, hi=490)
    p.trigger(0, 0.0)
    p.trigger(1, 0.1)
    assert p.lo == 300 and p.hi == 420, "驱动自报的安全区间比写死的数更可信"
    assert m.cli.calls[0][1] in range(300, 421)


def test_narrow_safe_range_refuses_loudly():
    """区间放不下一步就拒绝并说清楚，不要"扫一点点"糊过去。"""
    t = FakeTelem(pan_safe_min=300, pan_safe_max=301)
    m, p = mk(telem=t, lo=270, hi=490, step=24)
    p.trigger(0, 0.0)
    p.trigger(1, 0.1)
    assert not p.active
    assert p.tag == "PATROL-NARROW"
    assert m.cli.calls == []


def test_low_above_high_falls_back_to_hard_limits():
    m, p = mk(lo=500, hi=400)
    assert p.lo < p.hi


# ---------------------------------------------------------------- 底线 2：超时

def test_timeout_fires_and_goes_home():
    m, p = mk(max_s=20.0)
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    st = p.update(LOST, W, H, now=19.9)
    assert st != "TIMEOUT"
    st = p.update(LOST, W, H, now=20.5)
    assert st == "TIMEOUT"
    assert p.result == "TIMEOUT"
    assert not p.active
    assert modes(m)[-1] == P.MODE_HOME, "超时必须回 home"


def test_timeout_is_checked_before_target_handling():
    """超时优先于"刚好看见目标"：预算到了就是到了，不许因为一帧侥幸续命。"""
    m, p = mk(max_s=5.0)
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    assert p.update(found(160.0, 120.0), W, H, now=6.0) == "TIMEOUT"
    assert p.result == "TIMEOUT"


def test_tick_times_out_without_any_frames():
    """视频链路断了就没有新帧，超时必须照样到点。

    这一条防的是"摄像头一挂，巡检永远不到点"——云台停在扫描端点上不动，
    而日志上看不出任何异常。它只在帧之外的那条路径上，所以单测也得从那里走。
    """
    m, p = mk(max_s=5.0)
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    p.tick(4.9)
    assert p.active
    p.tick(5.5)
    assert not p.active
    assert p.result == "TIMEOUT"
    assert modes(m)[-1] == P.MODE_HOME


def test_timeout_state_ignores_further_frames():
    m, p = mk(max_s=1.0)
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    p.update(LOST, W, H, now=2.0)
    n = len(m.cli.calls)
    for _ in range(10):
        p.update(found(160.0), W, H, now=3.0)
    assert len(m.cli.calls) == n, "超时之后 Patrol 不该再发任何目标"


# ---------------------------------------------------------------- 交接

def test_found_needs_three_consecutive_frames():
    m, p = mk()
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    assert p.update(found(160.0), W, H, now=1.0) == "SWEEP"
    assert p.update(found(160.0), W, H, now=1.1) == "SWEEP"
    assert p.active, "两帧还不够"
    assert p.update(found(160.0), W, H, now=1.2) == "FOUND"
    assert p.result == "FOUND"
    assert not p.active


def test_a_single_bogus_frame_does_not_hand_off():
    m, p = mk()
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    p.update(found(10.0), W, H, now=1.0)
    p.update(LOST, W, H, now=1.1)
    p.update(found(10.0), W, H, now=1.2)
    p.update(found(10.0), W, H, now=1.3)
    assert p.active, "中间断了一帧就要重新数"


def test_handoff_leaves_the_gimbal_where_it_is():
    """交接只松手，不代 Auto 发任何目标。"""
    m, p = mk()
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    for i in range(3):
        p.update(found(160.0), W, H, now=1.0 + i * 0.1)
    n = len(m.cli.calls)
    assert p.update(found(160.0), W, H, now=2.0) == "FOUND"
    assert len(m.cli.calls) == n


# ---------------------------------------------------------------- 走位

def test_sweep_uses_manual_not_track():
    """按剧本走位不是闭环跟踪，遥测里的 daemon_state 要如实区分。"""
    m, p = mk()
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    assert set(modes(m)) == {P.MODE_MANUAL}


def test_starts_from_home():
    m, p = mk()
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    assert m.cli.calls[0][1] == p.hi or m.cli.calls[0][1] == p.lo or \
        m.cli.calls[0][1] == 377, "起点应当是 home，再由脚本决定往哪端走"


def test_tilt_is_held_constant():
    m, p = mk()
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    now = 0.0
    for _ in range(200):
        now += 0.05
        m._t.pan_cur = p.cmd_pan
        p.update(LOST, W, H, now=now)
    tilts = {c[2] for c in m.cli.calls if c[2] is not None}
    assert tilts == {242}, f"tilt 全程不该动，实际 {tilts}"


def test_dwells_before_moving_on():
    """到位后要停一会儿再看一眼：姿态没稳就检测等于拿旧画面判目标。"""
    m, p = mk()
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    m._t.pan_cur = p.cmd_pan
    p.update(LOST, W, H, now=1.0)
    assert p.state == "DWELL"
    n = len(m.cli.calls)
    p.update(LOST, W, H, now=1.0 + Patrol.DWELL_S * 0.5)
    assert len(m.cli.calls) == n, "停够之前不该发下一个目标"
    p.update(LOST, W, H, now=1.0 + Patrol.DWELL_S + 0.01)
    assert len(m.cli.calls) == n + 1


def test_reverses_at_both_ends():
    """扫到头要反向，而不是卡在端点上反复发同一个值。"""
    m, p = mk(step=100)  # 大跨步：两三次就到端点
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    seen = []
    now = 0.0
    for _ in range(6):
        m._t.pan_cur = p.cmd_pan
        now += 0.05
        p.update(LOST, W, H, now=now)
        m._t.pan_cur = p.cmd_pan
        now += Patrol.DWELL_S + 0.01
        p.update(LOST, W, H, now=now)
        seen.append(p.cmd_pan)
    assert p.hi in seen and p.lo in seen, f"两端都该去过，实际 {seen}"


# ---------------------------------------------------------------- 打断

def test_abort_stops_and_holds_position():
    m, p = mk()
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    p.abort()
    assert not p.active
    assert p.result == "ABORT"
    assert modes(m)[-1] == P.MODE_HOLD


def test_abort_when_idle_is_a_noop():
    m, p = mk()
    p.abort()
    assert m.cli.calls == []


def test_render_is_ascii_and_short():
    m, p = mk()
    p.trigger(0, 0.0)
    p.trigger(1, 0.0)
    r = p.render()
    assert r.isascii(), "HUD 只能是 ASCII（cv2.putText 没有 CJK 字形）"


def main() -> int:
    tests = [
        test_first_reading_never_triggers,
        test_rising_edge_triggers,
        test_flat_counter_does_not_retrigger,
        test_event_during_patrol_extends_deadline_without_restarting,
        test_no_telemetry_refuses_to_start,
        test_sweep_never_leaves_the_configured_range,
        test_hard_ceiling_is_514_not_515,
        test_range_is_clamped_by_driver_safe_range,
        test_narrow_safe_range_refuses_loudly,
        test_low_above_high_falls_back_to_hard_limits,
        test_timeout_fires_and_goes_home,
        test_timeout_is_checked_before_target_handling,
        test_tick_times_out_without_any_frames,
        test_timeout_state_ignores_further_frames,
        test_found_needs_three_consecutive_frames,
        test_a_single_bogus_frame_does_not_hand_off,
        test_handoff_leaves_the_gimbal_where_it_is,
        test_sweep_uses_manual_not_track,
        test_starts_from_home,
        test_tilt_is_held_constant,
        test_dwells_before_moving_on,
        test_reverses_at_both_ends,
        test_abort_stops_and_holds_position,
        test_abort_when_idle_is_a_noop,
        test_render_is_ascii_and_short,
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
