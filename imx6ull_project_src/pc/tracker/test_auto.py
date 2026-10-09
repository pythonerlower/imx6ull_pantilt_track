"""3D 闭环的决策逻辑单测：不碰硬件、不碰舵机。

这个文件只验 `Auto` 这一层——「一帧检测结果 → 一次 set_target 调用」。
不需要真云台，因为要证伪的性质全在决策里，跟电机无关：

  1. **迟滞的门槛是 4 / 5，不是 3 / 3。** 出画后那 3 帧误报是连续 3 帧，
     阈值取 3 就会在第 3 帧（也就是最后那帧误报）上认下来。这是实测出来的
     数，不是拍的，所以必须有测试钉住——它一旦被"顺手改成 3"，现场表现是
     "云台偶尔对着空处转一下"，没人会想到是阈值少了一。
  2. **死区内不下发。** 下发一个和当前目标一样、或比它更靠后的值，会把
     驱动手上没走完的余量抽掉。这是「看着一顿一顿」的直接成因，
     而且它**不会**表现为错误值——只会表现为动作破碎，最难事后归因。
  3. **目标从实测 cur 重新算，不累加。** 累加会把在途余量重复计一次，
     表现为越跟越超前、最后过冲振荡。
  4. **丢目标时先溜车、别急停。** M 帧窗口内一个包都不发。
  5. **限位在客户端就夹住**（pan 跟踪上限 490 比驱动的 515 紧）。
  6. **没有新鲜遥测就绝不下发**（复用 Manual.guard() 那道闸门）。

    python3 test_auto.py
"""

from __future__ import annotations

import contextlib
import dataclasses
import io
import math
import random
import socket
import threading
import time

import ptproto as P
from main import Auto, LashProbe, SignProbe, auto_banner
from tracker import Target

W, H = 320, 240


@dataclasses.dataclass
class FakeTelem:
    """只带 Auto 会读的字段。真 Telem 解出来的包在 test_control.py 里验。"""

    pan_cur: int = 377
    tilt_cur: int = 242
    pan_tgt: int = 377
    tilt_tgt: int = 242
    pan_safe_min: int = 239
    pan_safe_max: int = 515
    tilt_safe_min: int = 162
    tilt_safe_max: int = 342
    watchdog_trips: int = 0
    daemon_wd_trips: int = 0


class FakeCli:
    def __init__(self):
        self.calls: list[tuple] = []
        self._mode = P.MODE_HOLD
        self._pan = 0
        self._tilt = 0

    def set_target(self, mode, pan_tick=None, tilt_tick=None, immediate=False):
        self.calls.append((mode, pan_tick, tilt_tick, immediate))
        # 与真 ControlClient 同语义：None 表示"沿用上一次"。
        self._mode = mode
        if pan_tick is not None:
            self._pan = int(pan_tick)
        if tilt_tick is not None:
            self._tilt = int(tilt_tick)
        return len(self.calls)

    def current_target(self):
        return self._mode, self._pan, self._tilt

    def modes(self) -> list[int]:
        return [c[0] for c in self.calls]


class FakeManual:
    """Manual 的最小替身：只留 Auto 会用的 note/tag/cli/guard()。"""

    def __init__(self, telem: FakeTelem | None = FakeTelem()):
        self.cli = FakeCli()
        self._t = telem
        self.note = ""
        self.tag = ""
        self.refused = 0
        self.max_age_ms = 500.0

    def fresh(self):
        return self._t, 0.0

    def guard(self):
        # 与 Manual.guard() 同语义：没有可用遥测就返回 None。
        if self._t is None:
            self.note = "还没有收到遥测"
            self.tag = "NO-TELEM"
            self.refused += 1
            return None
        return self._t


def found(cx: float, cy: float = 120.0) -> Target:
    return Target(found=True, cx=cx, cy=cy, area=2000.0)


LOST = Target(found=False)


def mk(m=None, **kw):
    m = m or FakeManual()
    return m, Auto(m, **kw)


# ---------------------------------------------------------------- 迟滞

def test_acquire_needs_four_frames():
    """连续 3 帧误报（面积 2380/1464/694）绝不能换出一次下发。

    这三帧的真实来源：logs/phase3/20261007_3c_calib.md §7，目标出画后
    一块静态红衣服滑进画面左侧。它们在第 3 帧就够到 N=3 的门槛，
    所以 N 必须是 4。
    """
    m, a = mk()
    for i in range(1, 4):
        st = a.update(found(200.0), W, H)
        assert st == "ACQUIRE", f"第 {i} 帧就进 TRACK 了，N 太小"
        assert a.moves == 0, f"第 {i} 帧就下发了：误报会被当成目标"
    assert a.state == "ACQUIRE"
    # 第 4 帧才是真目标
    a.update(found(10.0), W, H)
    assert a.state == "TRACK", "第 4 帧该进闭环了"
    print("  [ok] 认目标要 4 帧：连续 3 帧误报一次都没换出下发")


def test_lost_frame_inside_window_resets_acquire():
    """确认途中插一帧 LOST，计数必须清零，不能"攒够 4 帧"就认。

    不然一段 2 帧误报 + 1 帧漏检 + 2 帧误报就能凑出 4 帧。
    """
    m, a = mk()
    a.update(found(200.0), W, H)
    a.update(found(200.0), W, H)
    a.update(LOST, W, H)
    assert a.found_streak == 0, "插了一帧 LOST 还留着确认计数"
    a.update(found(200.0), W, H)
    assert a.state == "ACQUIRE", "清零后第 1 帧就进闭环了"
    print("  [ok] 确认链中间插一帧 LOST 会清零，凑不出假目标")


def test_release_holds_five_frames():
    """进了闭环之后，连续丢 M 帧才放手，且窗口内**一个包都不发**。"""
    m, a = mk()
    for _ in range(4):
        a.update(found(180.0), W, H)
    assert a.state == "TRACK"
    n_after_acquire = len(m.cli.calls)

    for i in range(1, 6):
        st = a.update(LOST, W, H)
        assert st == "TRACK", f"第 {i} 帧 LOST 就退出闭环了，M 太小"
        assert len(m.cli.calls) == n_after_acquire, \
            f"第 {i} 帧 LOST 时发了包——溜车窗口里不该发任何东西"
        assert m.tag == "COAST"
    st = a.update(LOST, W, H)          # 第 6 帧，超窗口
    assert st == "IDLE", "丢了 6 帧还握着控制权"
    assert m.cli.calls[-1][0] == P.MODE_HOLD, "退出闭环该回 HOLD"
    assert m.cli.calls[-1][1] is None and m.cli.calls[-1][2] is None, \
        "HOLD 不准带 tick：驱动侧会忽略它们，带了只会误导读日志的人"
    print("  [ok] 丢 5 帧内保持且不发包，第 6 帧回 HOLD（不带 tick）")


# ---------------------------------------------------------------- 死区与步长

def test_deadband_issues_nothing():
    """误差在死区内时不下发。下发会抽掉在途余量 → 动作破碎。"""
    m, a = mk(deadband=5.0)
    for _ in range(4):
        a.update(found(160.0), W, H)     # 先靠死区外的一帧进闭环
    before = len(m.cli.calls)
    # 165 = 中心 + 5，正好在死区边上（|err| < 5 才算死区）
    a.update(Target(found=True, cx=164.0, cy=120.0, area=2000.0), W, H)
    assert len(m.cli.calls) == before, "死区内还是发了包"
    assert m.tag == "DEADBAND"
    print("  [ok] 死区内不发包（tag=DEADBAND）")


def test_step_is_capped_and_signed():
    """大误差时步长被 max_step 夹住，方向由 kp 的符号决定。

    符号约定（默认 kp<0）：目标偏右（err_x>0）→ tick 减小。
    推导：tick↑ = pan 向左（phase2/3A 实测，见 handle_key 里 a=+step），
    要移向右侧目标就得朝反方向。
    """
    m, a = mk(kp=-0.25, max_step=4, deadband=5)
    for _ in range(4):
        a.update(found(300.0), W, H)     # err_x = +140，远超上限
    mode, pan, tilt, imm = m.cli.calls[-1]
    assert mode == P.MODE_TRACK
    assert pan == 377 - 4, f"步长没被夹到 4：{377} → {pan}"
    assert tilt is None, "单轴模式不该动 tilt"
    assert imm is True, "闭环要立刻下发，不能等下一个心跳周期"
    print(f"  [ok] err=+140px 时夹到 -4 tick（377 → {pan}），tilt 不动")


def test_no_subtick_command():
    """换算下来不足 1 tick 就不发——驱动的分辨力就是 1 tick。"""
    m, a = mk(kp=-0.01, max_step=4, deadband=5)
    for _ in range(4):
        a.update(found(200.0), W, H)     # err_x = +40 → raw = -0.4
    # 进闭包装不上：kp 太小，第 4 帧进 TRACK 后 _aim 算出来是 0 → 不发
    assert a.state == "TRACK"
    assert m.cli.calls == [], f"不足 1 tick 还是发了：{m.cli.calls}"
    assert m.tag == "SUBTICK"
    print("  [ok] 不足 1 tick 不发（tag=SUBTICK）")


def test_target_is_recomputed_from_cur_not_accumulated():
    """每帧从**实测 cur** 重新瞄准，不累加在途余量。

    构造：遥测说 cur 一直没动（377）。连发 3 帧同样的误差，
    每一帧算出来的目标都该是 377-4=373，而不是 373/369/365。
    累加的写法会得到后者——那正是"越跟越超前"的来源。
    """
    m, a = mk(kp=-0.25, max_step=4, deadband=5)
    for _ in range(4):
        a.update(found(300.0), W, H)
    pans = [c[1] for c in m.cli.calls if c[0] == P.MODE_TRACK]
    assert pans, "一次都没下发"
    assert pans == [373] * len(pans), \
        f"目标被累加了（应全是 373）：{pans}"
    print(f"  [ok] cur 不动时目标恒为 373，连发 {len(pans)} 次没有累加")


# ---------------------------------------------------------------- 限位

def test_pan_max_tightens_client_side():
    """跟踪上限 490 必须在客户端就生效，不能只靠驱动的 515。"""
    t = FakeTelem(pan_cur=489)
    m, a = mk(FakeManual(t), kp=-1.0, max_step=8, deadband=1,
              pan_max=490)
    for _ in range(4):
        a.update(found(10.0), W, H)      # err_x = -150 → 想往 + 方向猛冲
    pans = [c[1] for c in m.cli.calls if c[0] == P.MODE_TRACK]
    assert pans, "一次都没下发"
    assert pans[0] == 490, f"没夹到跟踪上限 490，而是 {pans[0]}"
    assert all(p <= 490 for p in pans), f"越过了跟踪上限：{pans}"

    # 驱动力爬到边界之后：cur 到 490，目标也是 490，不该再发。
    # 这里必须让遥测真的跟上——静态的 cur 会让"贴边"这个条件永远不成立，
    # 那种情况下每帧重发同一个 490 是对的（板端按值去重，不产生动作）。
    t.pan_cur = 490
    before = len(m.cli.calls)
    a.update(found(10.0), W, H)
    assert len(m.cli.calls) == before, "cur 已到 490 还在每帧发指令"
    assert m.tag == "LIMIT"
    print("  [ok] 跟踪上限 490 在客户端夹住，贴边后不再空转发指令")


def test_driver_safe_range_also_applies():
    """即使没给 --track-pan-max，驱动的安全范围仍然是硬边界。"""
    t = FakeTelem(pan_cur=513, pan_safe_max=515)
    m, a = mk(FakeManual(t), kp=-1.0, max_step=8, deadband=1)
    for _ in range(4):
        a.update(found(10.0), W, H)
    for c in m.cli.calls:
        if c[0] == P.MODE_TRACK and c[1] is not None:
            assert c[1] <= 515, f"越过了驱动的安全上限：{c}"
    print("  [ok] 未指定上限时，驱动的 pan_safe_max 仍然夹住")


# ---------------------------------------------------------------- 安全闸门

def test_no_telem_means_no_command():
    """没有新鲜遥测时绝不下发。复用 Manual.guard()，这道闸门不许绕。"""
    m = FakeManual(telem=None)
    a = Auto(m, kp=-0.25, max_step=4, deadband=5)
    for _ in range(10):
        a.update(found(300.0), W, H)
    assert a.state == "TRACK", "确认流程本身不需要遥测，应该进 TRACK"
    assert m.cli.calls == [], f"没遥测还是下发了：{m.cli.calls}"
    assert m.refused > 0, "guard() 的拒绝计数没涨，说明没走那道闸门"
    print(f"  [ok] 无遥测时 10 帧一次都没下发，guard 拒绝 {m.refused} 次")


def test_suspend_and_resume():
    """空格 = 挂起：回 HOLD，且此后不再下发；恢复后重新走确认流程。"""
    m, a = mk()
    for _ in range(4):
        a.update(found(300.0), W, H)
    assert a.active
    a.suspend(m)
    assert not a.active
    assert m.cli.calls[-1][0] == P.MODE_HOLD
    assert m.cli.calls[-1][1] is None, "HOLD 不该带 tick"

    before = len(m.cli.calls)
    for _ in range(10):
        a.update(found(300.0), W, H)
    assert len(m.cli.calls) == before, "挂起后还在下发"
    assert a.update(found(300.0), W, H) == "SUSPENDED"

    a.resume(m)
    assert a.active
    for i in range(1, 4):
        assert a.update(found(300.0), W, H) == "ACQUIRE", \
            f"恢复后第 {i} 帧就进闭环了，没有重走确认流程"
    a.update(found(300.0), W, H)
    assert a.state == "TRACK"
    print("  [ok] 挂起回 HOLD 并停发；恢复后重走 4 帧确认")


# ---------------------------------------------------------------- 连续性自检

def test_pace_warns_when_lead_is_too_short():
    """max_step × slew_ms < 帧间隔时必须告警——那是"走一下停一下"的成因，
    而现象上极难归因，所以要主动量出来。"""
    import io
    import contextlib
    import time as _t

    m, a = mk(max_step=3, slew_ms=20.0)      # 3 × 20 = 60 ms < 66.7 ms
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        for _ in range(12):
            a.update(found(180.0), W, H)
            _t.sleep(0.07)                   # 造出 ~70 ms 的帧间隔
    out = buf.getvalue()
    assert "max_step" in out and "走一下停一下" in out, \
        f"领先量不够时没有告警：{out!r}"
    print("  [ok] max_step=3（60 ms 领先）在 70 ms 帧间隔下告警")


def test_pace_silent_when_lead_is_long_enough():
    import io
    import contextlib
    import time as _t

    m, a = mk(max_step=4, slew_ms=20.0)      # 4 × 20 = 80 ms > 66.7 ms
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        for _ in range(12):
            a.update(found(180.0), W, H)
            _t.sleep(0.07)
    out = buf.getvalue()
    assert "走一下停一下" not in out, f"领先量够却告警了：{out!r}"
    # 但低帧率的**提示**应该照常出——它是另一件事（见下一条测试）。
    assert "30 fps" in out, f"低帧率提示没出：{out!r}"
    print("  [ok] max_step=4（80 ms 领先）在 70 ms 帧间隔下不告警")


def test_low_frame_rate_hint():
    """帧率是连续性最大的杠杆，低帧率时要主动提示。

    理由：台阶下发的最小间隔就是检测周期，15 fps 下目标慢时云台
    每隔 67 ms 才拿到一次误差。这不是调 kp 能补的。
    而"把板端跑到 30 fps"不用改任何代码，最容易被人忽略。
    """
    import io
    import contextlib
    import time as _t

    m, a = mk(max_step=4, slew_ms=20.0)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        for _ in range(12):
            a.update(found(180.0), W, H)
            _t.sleep(0.07)                    # ~70 ms → 约 14 fps
    out = buf.getvalue()
    assert "--fps 30" in out, f"低帧率没提示：{out!r}"

    # 高帧率下不该提示
    m2, a2 = mk(max_step=4, slew_ms=20.0)
    buf2 = io.StringIO()
    with contextlib.redirect_stdout(buf2):
        for _ in range(12):
            a2.update(found(180.0), W, H)
            _t.sleep(0.033)                   # ~33 ms → 约 30 fps
    assert "30 fps" not in buf2.getvalue(), \
        f"已经 30 fps 还在提示：{buf2.getvalue()!r}"
    print("  [ok] 14 fps 提示「板端跑 30」，30 fps 不提示")


# ------------------------------------------------------- 连续性的数字
# 「看起来连续」是一条主观要求，不能靠"我觉得还行"来验收。这里把它换算成
# 一个可测的量：**闭环运行期间，相邻两次 tick 之间隔了多久**。
#
# 为什么这个量就等价于"看着连不连续"：驱动的速度是固定的（1 tick /
# slew_ms），所以云台的运动是一串离散的 0.43° 台阶。台阶之间停多久，
# 眼睛看到的就是"一顿一顿"还是"平滑"。0.43° 的台阶本身看不出来，
# 停顿看得出来。所以要求是：**台阶间隔稳定，没有长停顿**。

SLEW_MS = 20.0          # 对齐 pantilt 驱动：每 20 ms 一个 tick
FRAME_MS = 66.7         # 15 fps（传感器 S_PARM 能被驱动接受的那一档）
FRAME_MS_30 = 33.3      # 30 fps（传感器原生速率，3D 推荐跑这个）
PX_PER_TICK = 2.3       # 320 宽、约 60° 水平视场 → 0.43°/tick ≈ 2.3 px


class SimPlant:
    """把驱动 + 相机简化成一个模型：cur 以 1 tick / SLEW_MS 爬向 tgt。

    这不是在"验证驱动"，驱动是 C 那边的事。它验证的是**控制律能不能
    把驱动喂饱**——即目标是否始终领先于实际位置。这个性质和驱动的具体
    实现无关，只取决于"驱动的速度上限"和"控制器每帧派多少"。

    坐标系故意和控制器**独立**写一遍（不调 Auto 里的任何东西）：
    tick↑ = 相机左转 = 画面里的东西右移 = cx 变大。
    这样如果控制律的方向反了，这个仿真会发散，测试会挂。
    """

    def __init__(self, pan_ref=377.0, px_per_tick=PX_PER_TICK):
        self.pan_ref = pan_ref
        self.cur = float(pan_ref)
        self.tgt = int(pan_ref)
        self.px_per_tick = px_per_tick
        self.px_offset = 0.0        # 目标自己在画面里的位移（模拟目标移动）
        self.tick_times: list[float] = []
        self._acc = 0.0

    def cx(self) -> float:
        # px_per_tick 可正可负：符号标定那一组测试要一个"接反了"的云台，
        # 而"接反"的全部含义就是这个斜率变号。
        return (160.0 + (self.cur - self.pan_ref) * self.px_per_tick
                + self.px_offset)

    def set_target(self, tick: int) -> None:
        self.tgt = int(tick)

    def run(self, ms: float, t0: float) -> float:
        """推进 ms 毫秒，返回新的时刻。"""
        self._acc += ms
        t = t0
        while self._acc >= SLEW_MS:
            self._acc -= SLEW_MS
            t += SLEW_MS / 1000.0
            if self.cur < self.tgt:
                self.cur = min(self.cur + 1.0, float(self.tgt))
                self.tick_times.append(t)
            elif self.cur > self.tgt:
                self.cur = max(self.cur - 1.0, float(self.tgt))
                self.tick_times.append(t)
        return t


class SimManual(FakeManual):
    """遥测跟着仿真里的 cur 走。"""

    def __init__(self, plant: SimPlant):
        super().__init__()
        self.plant = plant

    def guard(self):
        t = FakeTelem(pan_cur=int(round(self.plant.cur)))
        return t


def _gap_stats(times: list[float]) -> tuple[float, float, float]:
    if len(times) < 2:
        return 0.0, 0.0, 0.0
    gaps = [(b - a) * 1000.0 for a, b in zip(times, times[1:])]
    gaps_sorted = sorted(gaps)
    mean = sum(gaps) / len(gaps)
    p95 = gaps_sorted[int(len(gaps_sorted) * 0.95)]
    return mean, p95, max(gaps)


def _run_sim(kp=-0.5, max_step=4, deadband=2.0, frames=90, drift_px=0.6,
             noise_px=0.0, frame_ms=FRAME_MS, seed=7):
    """跑一段闭环，返回 (plant, aut, m)。

    drift_px 是目标每一帧在画面里的位移——不动的目标是最容易的情形，
    真实场景里手一直在抖，所以要给一个持续的扰动。noise_px 是检测中心
    自身的抖动，它在**静止目标**上是防抖测试的主角。
    """
    import random

    rnd = random.Random(seed)
    plant = SimPlant()
    m = SimManual(plant)
    a = Auto(m, kp=kp, max_step=max_step, deadband=deadband)
    a.m._t = None                     # 让 guard() 走 SimManual 那条路

    cmd_steps: list[int] = []

    class Cli:
        def set_target(self, mode, pan_tick=None, tilt_tick=None,
                       immediate=False):
            if mode == P.MODE_TRACK and pan_tick is not None:
                if pan_tick != plant.tgt:
                    cmd_steps.append(abs(pan_tick - plant.tgt))
                plant.set_target(pan_tick)
            return 1

    m.cli = Cli()
    plant.cmd_steps = cmd_steps

    t = 0.0
    # 先让目标出现在画面里（对应"操作者把红盒子举进镜头"）
    plant.px_offset = 60.0
    for _ in range(frames):
        plant.px_offset += drift_px
        off = plant.px_offset + rnd.uniform(-noise_px, noise_px)
        a.update(Target(found=True,
                        cx=160.0 + (plant.cur - plant.pan_ref) * PX_PER_TICK + off,
                        cy=120.0, area=2000.0), W, H)
        t = plant.run(frame_ms, t)
    return plant, a, m


def _ideal_gap_ms(drift_px: float, frame_ms: float) -> float:
    """物理下限：走 1 tick 需要等目标自己挪 2.3 px，多久挪得到。

    这就是"看起来连不连续"的地板——闭环再怎么调都不可能低于它。
    """
    v = drift_px * (1000.0 / frame_ms) / PX_PER_TICK      # tick/s
    return 1000.0 / v if v > 0 else float("inf")


def test_closed_loop_converges_not_diverges():
    """闭环要收敛。方向反了这里会发散——这是"上电即飞车"的离线预演。

    注意它**不能替代**实机测符号：这里 Plant 的符号是我按物理写的，
    控制律的符号是我按同一套物理定的。两者一致只能证明控制律自洽，
    不能证明相机真的那么装。符号仍然必须实测（见 README 的实机流程）。
    """
    plant, a, m = _run_sim(frames=90, drift_px=2.0)
    final_err = abs(plant.cx() - 160.0)
    assert final_err < 15.0, \
        f"90 帧后误差还有 {final_err:.0f} px，闭环没收敛（方向反了？）"

    # 云台**该**离开起点：目标一直在往画面右边漂，要跟住就得一直往左转。
    # 所以判据是"跟上了漂移量"，不是"没怎么动"——按后者写会把
    # 一个正常工作的闭环判成发散。
    expect_cur = plant.pan_ref - plant.px_offset / PX_PER_TICK
    assert abs(plant.cur - expect_cur) < 8.0, \
        (f"云台在 {plant.cur:.0f}，跟上漂移应在 {expect_cur:.0f}"
         f"（差 {abs(plant.cur - expect_cur):.0f} tick）")
    assert a.moves > 10, f"只下发了 {a.moves} 次，闭环基本没在动"
    print(f"  [ok] 闭环收敛：末帧误差 {final_err:.1f} px（死区 {a.deadband:.0f}）；"
          f"云台 {plant.pan_ref:.0f} → {plant.cur:.0f}，"
          f"跟上漂移期望 {expect_cur:.0f}；下发 {a.moves} 次")


def test_no_added_stall_over_physical_floor():
    """**这是"看起来连续"的验收**，判据是相对物理下限的，不是绝对的。

    绝对判据（"间隔必须 < 40 ms"）是**错**的：它只有在目标比驱动还快
    （驱动饱和、全程 20 ms 一格）时才成立。目标慢下来，云台就只能按
    ``0.43° / v`` 的间隔一格一格走——那是 tick 分辨力定的，不是代码定的。

    所以真正该保证的是：**实测停顿不超过物理下限的一个小余量。**
    闭环能犯的错就是在这之上再叠停顿（死区攒着不下发、丢帧急停、
    从错的地方累加），这条测的正是"它没这么干"。
    """
    # 用 30 fps + 默认参数，也就是推荐的 3D 运行点
    for drift in (0.6, 2.0, 5.0):
        plant, a, m = _run_sim(drift_px=drift, frame_ms=FRAME_MS_30,
                               frames=int(12_000 / FRAME_MS_30))
        _, p95, worst = _gap_stats(plant.tick_times)
        floor = _ideal_gap_ms(drift, FRAME_MS_30)
        # 容差取「物理下限的 1.6 倍，或下限 + 3 个 tick 周期，取大者」。
        # 后者兜住下限很小时的情况（快目标下限只有 15 ms，1.6 倍才 24 ms，
        # 而一个 tick 周期就是 20 ms，量不出东西）。
        lim = max(floor * 1.6, floor + 3 * SLEW_MS)
        assert worst < lim, \
            (f"漂移 {drift} px/帧：最长停顿 {worst:.0f} ms，"
             f"物理下限 {floor:.0f} ms、容差到 {lim:.0f} ms —— "
             f"多出来的 {worst - floor:.0f} ms 是闭环自己加的停顿")
        print(f"  [ok] 漂移 {drift:>4.1f} px/帧：台阶间隔 p95={p95:>4.0f} "
              f"max={worst:>4.0f} ms，物理下限 {floor:>4.0f} ms")


def test_30fps_beats_15fps_on_continuity():
    """反证 + 记录：**30 fps 在每个指标上都优于 15 fps**，所以 3D 该跑 30。

    机理：台阶下发的最小间隔是 PC 的检测周期。15 fps 下闭环每秒只拿到
    15 次误差采样，累积到"够走一格"的延迟翻倍。这不是调参能补的，
    只能提帧率——而传感器本来就是 30 fps，不用改任何板端参数。

    这条挡住"为了省带宽把 3D 的帧率降回 15"这个看起来无害的优化。
    """
    # **同一物理运动**要比得公平：drift_px 是"每帧走几个像素"，
    # 帧率翻倍时同一个角速度对应的 px/帧要减半。两边都写 2.0 的话，
    # 30 fps 那组的"目标速度"其实是 15 fps 的两倍，测出来的差异
    # 有一部分只是目标真的更快了。
    r15 = _run_sim(drift_px=2.0, frame_ms=FRAME_MS, frames=int(12_000 / FRAME_MS))
    r30 = _run_sim(drift_px=1.0, frame_ms=FRAME_MS_30,
                   frames=int(12_000 / FRAME_MS_30))
    w15 = _gap_stats(r15[0].tick_times)[2]
    w30 = _gap_stats(r30[0].tick_times)[2]
    assert w30 < w15, f"30 fps 的最差停顿 {w30:.0f} 没比 15 fps 的 {w15:.0f} 好"
    print(f"  [ok] 最差台阶间隔：15 fps {w15:.0f} ms → 30 fps {w30:.0f} ms")


def test_max_step_one_gives_uniform_single_tick_steps():
    """``--max-step 1``：**每一次下发都恰好 1 tick**，幅度最小。

    这是用户那句"转动幅度"的极端取值——牺牲跟踪速度（上限掉到
    1 tick/帧 = 12.9°/s @30fps）换动作最匀。清单里留着它，
    是因为"看起来连不连续"最终得由操作者用眼睛判，而不是我替他定。
    """
    plant, a, m = _run_sim(max_step=1, drift_px=2.0, frame_ms=FRAME_MS_30,
                           frames=int(12_000 / FRAME_MS_30))
    steps = plant.cmd_steps
    assert steps, "一次都没下发"
    assert set(steps) == {1}, f"出现了非单 tick 的幅度：{sorted(set(steps))}"
    print(f"  [ok] --max-step 1：{len(steps)} 次下发全是 1 tick"
          f"（0.43°/步，{len(steps) / 12:.0f} 步/秒）")


def test_stationary_target_does_not_dither():
    """**静止目标 + 检测噪声不能产生来回蹭。**

    这是连续性的反面：云台停着不动时如果每帧 ±1 tick 地抖，
    舵机会啸叫、发热、磨损，画面上是持续的微震。
    死区取 0 或 kp 取太大都会触发它（实测 kp=-1.0/死区 0 是 38 次/秒）。
    默认参数必须两条都躲开：不抖，也不因为躲抖把死区开到 5 px 而攒步进。
    """
    plant, a, m = _run_sim(drift_px=0.0, noise_px=0.8, frame_ms=FRAME_MS_30,
                           frames=int(10_000 / FRAME_MS_30))
    n = len(plant.cmd_steps)
    # 收敛后应该基本不再动。10 秒里的下发次数就是"抖不抖"的直接读数。
    assert n <= 60, f"静止目标 10 秒下发了 {n} 次，{n / 10:.1f} 次/秒 —— 在抖"
    print(f"  [ok] 静止+噪声 10 秒只下发 {n} 次（{n / 10:.1f} 次/秒），不抖")


def test_deadband_five_batches_steps():
    """反证：死区开到 5 px（≈2 tick）会把运动攒成 2 tick 一迈。

    这条钉住"死区必须小于 1 tick 的像素当量"——5 px 看着只是个保守的
    死区，实际效果是让 70% 以外的下发都变成多 tick 的跳步。
    """
    good, _, _ = _run_sim(deadband=2.0, drift_px=0.6, frame_ms=FRAME_MS_30,
                          frames=int(12_000 / FRAME_MS_30))
    bad, _, _ = _run_sim(deadband=5.0, drift_px=0.6, frame_ms=FRAME_MS_30,
                         frames=int(12_000 / FRAME_MS_30))

    def one_ratio(p):
        s = p.cmd_steps
        return sum(1 for x in s if x == 1) / len(s) if s else 0.0

    g, b = one_ratio(good), one_ratio(bad)
    assert g > b, f"死区 2 px 的单 tick 占比 {g:.0%} 没比死区 5 px 的 {b:.0%} 好"
    print(f"  [ok] 单 tick 步进占比：死区 2 px {g:.0%} > 死区 5 px {b:.0%}")


# ---------------------------------------------------------- 端到端（真链路）
# 上面每一条验的是 `Auto` 这一个零件。这一条验的是**拼装层**：
# UDP 视频包进 → 真 VideoPlane → 真 get_target_center → Auto →
# 真 ControlClient → 真 ptproto 编码 → UDP 控制包出。
#
# 为什么值得单列：各部分单测全绿而合起来不动，是这类系统最经典的故障位置
# （3C 的记录里专门写过一条）。这里只有两端对家是假的——一个假 daemon
# 收控制包、一个假板端发遥测；中间全是生产路径上的真代码。

def _free_ports(n: int) -> list[int]:
    """一次要 n 个互不相同的空闲端口。

    **必须同时持有再一起关**：一个一个地"bind 到 0、取号、关掉"，
    内核每次都把刚释放的那个号再发一遍，三次会拿到同一个端口，
    然后视频、遥测、控制三条流全撞在一起——现象是"收不到遥测"，
    而根因是测试自己的洞。
    """
    socks = []
    try:
        for _ in range(n):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("127.0.0.1", 0))
            socks.append(s)
        return [s.getsockname()[1] for s in socks]
    finally:
        for s in socks:
            s.close()


class FakeDaemonUdp:
    """收控制包、解码、记录。不做任何控制决策。"""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.1)
        self.port = self.sock.getsockname()[1]
        self.pkts: list[P.Ctrl] = []
        self.bad = 0
        self.on_track = None            # 回调，用来驱动仿真云台
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                c = P.Ctrl.decode(data)
            except ValueError:
                self.bad += 1
                continue
            self.pkts.append(c)
            # 只把**值变了**的 TRACK 当成一次下发，和板端 daemon 的去重一致。
            if c.mode == P.MODE_TRACK and self.on_track is not None:
                self.on_track(c)

    def tracks(self) -> list[P.Ctrl]:
        return [p for p in self.pkts if p.mode == P.MODE_TRACK]

    def stop(self):
        self._stop.set()
        self._th.join(timeout=1.0)
        self.sock.close()


# 遥测模板。Telem 的字段全是必填的，一个一个写死会在板端加字段那天
# 悄悄漏掉一个（而漏掉的字段编码出来是 0，看起来像"板端报 0"）。
# 用 dataclasses.replace 改模板，字段增减时这里不用动。
_TELEM_TEMPLATE = P.Telem(**{
    **{f.name: 0 for f in dataclasses.fields(P.Telem)},
    **{
        "slew_ms": 20, "wd_ms": 1500, "daemon_wd_ms": 500,
        "pan_cur": 377, "tilt_cur": 242, "pan_tgt": 377, "tilt_tgt": 242,
        "pan_home": 377, "tilt_home": 242,
        "pan_safe_min": 239, "pan_safe_max": 515,
        "tilt_safe_min": 162, "tilt_safe_max": 342,
    },
})


class TelemPump:
    """以 20 Hz 把仿真云台的姿态当成遥测发出去。"""

    def __init__(self, port: int, plant: SimPlant):
        self.plant = plant
        self.dst = ("127.0.0.1", port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        while not self._stop.is_set():
            t = dataclasses.replace(
                _TELEM_TEMPLATE,
                timestamp_us=time.monotonic_ns() // 1000,
                pan_cur=int(round(self.plant.cur)),
                pan_tgt=self.plant.tgt,
            )
            try:
                self.sock.sendto(t.encode(), self.dst)
            except OSError:
                pass
            self._stop.wait(0.05)

    def stop(self):
        self._stop.set()
        self._th.join(timeout=1.0)
        self.sock.close()


class SceneSender:
    """按仿真云台的姿态实时造帧发出去。目标自己还会往画面右边漂。"""

    def __init__(self, port: int, plant: SimPlant, drift_px_per_s: float):
        self.plant = plant
        self.drift = drift_px_per_s
        self.dst = ("127.0.0.1", port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        self.fid = 0
        self.error: Exception | None = None
        self._stop = threading.Event()
        self._t0 = time.monotonic()
        self._th = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._th.start()

    def _loop(self):
        # 线程里的异常必须留下痕迹再退出。不然发送线程一死，
        # 现象是"收不到视频"，而真因（参数写错、numpy 报错）无影无踪
        # ——3C 那边给 FramePump 加 error 字段就是为这个。
        try:
            self._send_loop()
        except Exception as exc:            # noqa: BLE001 —— 故意全接
            self.error = exc

    def _send_loop(self):
        import cv2
        import numpy as np

        while not self._stop.is_set():
            el = time.monotonic() - self._t0
            self.plant.px_offset = 60.0 + self.drift * el
            cx = self.plant.cx()
            img = np.full((H, W, 3), 128, np.uint8)
            cv2.circle(img, (int(max(1, min(W - 2, cx))), 120), 22,
                       (0, 0, 230), -1)
            yuyv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV_YUYV).reshape(
                H, W * 2).tobytes()

            self.fid += 1
            total = (len(yuyv) + P.VIDEO_PAYLOAD - 1) // P.VIDEO_PAYLOAD
            ts = time.monotonic_ns() // 1000
            for i in range(total):
                chunk = yuyv[i * P.VIDEO_PAYLOAD:(i + 1) * P.VIDEO_PAYLOAD]
                pkt = P.video_pack(self.fid, i, total, ts, W, H,
                                   P.FMT_YUYV, chunk)
                try:
                    self.sock.sendto(pkt, self.dst)
                except OSError:
                    pass
            self._stop.wait(1.0 / 30.0)

    def stop(self):
        self._stop.set()
        self._th.join(timeout=1.0)
        self.sock.close()


def test_end_to_end_video_in_control_out():
    """整条生产路径跑一遍，断言的是"包真的出去了，而且都是合法的"。"""
    import main as M

    vport, cport, tport = _free_ports(3)
    plant = SimPlant()
    daemon = FakeDaemonUdp()
    telem = TelemPump(tport, plant)
    sender = SceneSender(vport, plant, drift_px_per_s=30.0)

    def on_track(c):
        # 板端驱动：每收到一次新目标就朝它爬（这里直接跳到位，20 ms/tick
        # 的爬行由 plant.run 推进）。只搬运目标值，不做别的判断。
        plant.set_target(c.pan_tick)

    daemon.on_track = on_track

    m = M.Manual("127.0.0.1", daemon.port, tport, 20.0, 3, 500.0)
    a = M.Auto(m, axis="pan", pan_max=490)
    vp = M.VideoPlane(vport)

    t0 = time.monotonic()
    ticks_t = t0
    try:
        telem.plant = plant
        m.start()
        vp.start()
        sender.start()

        # 等遥测和视频都到位（真链路上这是"确认板端在跑"那一步）
        deadline = t0 + 4.0
        while time.monotonic() < deadline:
            if m.fresh()[0] is not None and vp.poll():
                break
            time.sleep(0.01)
        assert m.fresh()[0] is not None, "收不到遥测，端到端跑不起来"
        assert sender.error is None, f"发帧线程死了：{sender.error!r}"
        assert vp.frame is not None, "收不到视频，端到端跑不起来"

        got = 0
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and got < 90:
            if not vp.poll():
                # 没新帧就推进仿真云台（1 tick / 20 ms）
                now = time.monotonic()
                plant.run((now - ticks_t) * 1000.0, 0.0)
                ticks_t = now
                time.sleep(0.002)
                continue
            got += 1
            now = time.monotonic()
            plant.run((now - ticks_t) * 1000.0, 0.0)
            ticks_t = now
            a.update(vp.target, vp.frame.width, vp.frame.height)

        tr = daemon.tracks()
        assert daemon.bad == 0, f"有 {daemon.bad} 个控制包解不开"
        assert got > 30, f"只收到 {got} 帧，链路没跑起来"
        assert tr, "端到端跑完了但一个 TRACK 控制包都没发出去"

        # 每一条下发过的 pan 都必须在跟踪上限内，而且真的是 pan 轴。
        for c in daemon.pkts:
            if c.mode == P.MODE_TRACK and c.pan_tick is not None:
                assert 239 <= c.pan_tick <= 490, \
                    f"下发了越界的 pan：{c.pan_tick}（应在 239..490）"
        assert a.moves > 0, "闭环一次都没下发"

        # 目标一直在往右漂，云台必须往 tick 减小的方向跟（默认 kp<0 的
        # 物理含义），否则就是方向反了——这是"上电即飞车"的端到端预演。
        assert plant.cur < plant.pan_ref, \
            (f"云台在 {plant.cur:.0f}，目标向右漂时应该往 tick 小的方向跟"
             f"（起点 {plant.pan_ref:.0f}）")
        print(f"  [ok] 端到端：{got} 帧视频 → {len(tr)} 个 TRACK 控制包，"
              f"云台 {plant.pan_ref:.0f} → {plant.cur:.0f}，"
              f"都在 239..490 内")
    finally:
        sender.stop()
        vp.close()
        telem.stop()
        daemon.stop()
        m.close()


# ------------------------------------------------- 画面坐标符号（实机前置）
# 这一节和上面所有测试的区别：上面证的是「控制律自洽」，这一节证的是
# 「符号判反会被拦下来」。两者不是一回事——仿真里相机的符号是按物理写的，
# 控制律的符号是按同一套物理推的，所以**仿真无论怎么跑都不可能发现符号
# 反了**。能发现它的只有真机测量，以及下面这个「测量结果与 kp 自相矛盾时
# 拒绝进环」的闸门。


class ProbeRig:
    """把 SimPlant 接到 SignProbe 上：喂帧、推进时间、让驱动跟着走。

    时间由 rig 自己记，probe 的时钟被换成读这个值，所以一次标定（真机上
    要十来秒）在这里是瞬时的，而且**完全确定**——不用 sleep。
    """

    def __init__(self, pan_ref=377.0, px_per_tick=PX_PER_TICK, jog=None,
                 pan_max=None, tick_ms=SLEW_MS, drive=True):
        self.plant = SimPlant(pan_ref, px_per_tick)
        self.plant.px_offset = 60.0      # 目标先偏离中心，位移才看得出来
        self.m = SimManual(self.plant)
        self.calls: list[tuple] = []
        self.drive = drive
        self.m.cli = self
        self.t = 0.0
        self.probe = SignProbe(self.m, pan_max=pan_max, slew_ms=tick_ms, jog=jog)
        self.probe._now = lambda: self.t

    def set_target(self, mode, pan_tick=None, tilt_tick=None, immediate=False):
        self.calls.append((mode, pan_tick, tilt_tick))
        if self.drive and pan_tick is not None:
            self.plant.set_target(pan_tick)
        return len(self.calls)

    def modes(self):
        return [c[0] for c in self.calls]

    def frame(self, dt_ms=FRAME_MS_30, lost=False):
        tgt = (LOST if lost else
               Target(found=True, cx=self.plant.cx(), cy=120.0, area=2000.0))
        st = self.probe.update(tgt, W, H)
        self.plant.run(dt_ms, self.t)
        self.t += dt_ms / 1000.0
        return st

    def run_to_end(self, max_frames=400):
        for _ in range(max_frames):
            if self.probe.done:
                break
            self.frame()
        assert self.probe.done, f"标定没在 {max_frames} 帧内结束"
        return self.probe


def test_probe_measures_positive_slope_and_asks_for_negative_kp():
    """云台装对了（tick↑ → 目标在画面里右移）→ kp 必须为负。"""
    p = ProbeRig().run_to_end()
    assert p.ok, p.m.note
    assert p.slope > 0, "tick↑ 让 cx 变大，斜率应当为正"
    assert 1.0 < p.slope < 4.0, f"斜率应该是 px/tick 的量级（仿真 2.3），得到 {p.slope}"
    assert p.kp_rec < 0, "斜率为正就必须用负 kp，否则是正反馈"


def test_probe_flips_kp_when_the_camera_is_mounted_backwards():
    """接反了（tick↑ → 目标在画面里左移）→ kp 必须翻成正的。

    这正是本节存在的理由：kp 不翻就是上电即飞车，而**只有真的量了**
    才知道要翻。
    """
    p = ProbeRig(px_per_tick=-PX_PER_TICK).run_to_end()
    assert p.ok, p.m.note
    assert p.slope < 0
    assert p.kp_rec > 0


def test_probe_reverses_direction_at_the_upper_limit():
    """起点贴着 pan 上限时改往反方向走，结论必须**一样**。

    方向是探测器自己选的，所以它必须参与那个除法——否则贴上限的那次
    标定会给出完全相反的结论，而这恰恰是最容易被忽略的一种现场。
    """
    rig = ProbeRig(pan_ref=510.0)        # 510 + 12 > 515，走不了
    p = rig.run_to_end()
    assert p.dir == -1, "上限走不动就该改往反方向"
    assert p.slope > 0 and p.kp_rec < 0, "方向翻了，结论不该跟着翻"
    assert max(c[1] for c in rig.calls) <= 515, "下发越过了驱动安全上限"


def test_probe_jog_stays_under_the_track_pan_max():
    """跟踪上限比驱动上限紧时，标定也不能越过它——否则量的是一个
    闭环永远不会去的区域。"""
    rig = ProbeRig(pan_ref=485.0, pan_max=490)
    p = rig.run_to_end()
    assert max(c[1] for c in rig.calls) <= 490
    assert p.dir == -1, "485 + 12 > 490，应该改往反方向"


def test_probe_restores_pan_to_the_starting_tick():
    """不管成功还是失败，最后一下都得把 pan 放回起点。"""
    for rig in (ProbeRig(), ProbeRig(drive=False)):
        p = rig.run_to_end()
        last = rig.calls[-1]
        assert last[0] == P.MODE_MANUAL
        assert last[1] == p.ax0, f"收尾没有归还到起点：{last!r}"


def test_probe_gives_up_when_motion_is_too_small():
    """驱动没动 → 判不出。宁可报失败，也不能猜一个符号出来。"""
    p = ProbeRig(drive=False).run_to_end()
    assert not p.ok
    assert p.phase == "FAIL"
    assert p.kp_rec is None, "判不出就不该给推荐值"
    assert p.slope == 0.0


def test_probe_resamples_when_the_target_disappears():
    """采样途中丢目标就清空重来，不拿半截样本算中位数。"""
    rig = ProbeRig()
    for _ in range(3):
        rig.frame()
    assert len(rig.probe.samples) == 3
    rig.frame(lost=True)
    assert rig.probe.samples == []
    assert rig.run_to_end().ok, "目标回来之后应当照样测得出来"


def test_probe_never_touches_track_or_home():
    """标定只走 MODE_MANUAL：不借闭环那套，也不借 home（home 会动 tilt）。"""
    rig = ProbeRig()
    rig.run_to_end()
    assert P.MODE_TRACK not in rig.modes()
    assert P.MODE_HOME not in rig.modes()
    assert set(rig.modes()) <= {P.MODE_MANUAL}, rig.modes()


def test_probe_sends_nothing_without_telem():
    """没有新鲜遥测时一个包都不发——Manual.guard() 那道闸门原样复用。"""
    m = FakeManual(telem=None)
    p = SignProbe(m, slew_ms=SLEW_MS)
    p._now = lambda: 0.0
    tgt = Target(found=True, cx=200.0, cy=120.0, area=2000.0)
    for _ in range(SignProbe.SAMPLES):
        p.update(tgt, W, H)
    assert p.phase == "FAIL" and p.done
    assert m.cli.calls == [], f"没遥测还发了 {m.cli.calls}"


def test_probe_abort_puts_pan_back():
    """中途按 q：也必须归还，不能把云台留在 起点+JOG 上。"""
    rig = ProbeRig()
    for _ in range(200):
        if rig.probe.phase == "MOVE":
            break
        rig.frame()
    assert rig.probe.phase == "MOVE"
    rig.probe.abort()
    assert rig.probe.done
    assert rig.calls[-1][1] == rig.probe.ax0, f"中断没有归还：{rig.calls[-1]!r}"


# ------------------------------------------------- tilt 轴（--axis tilt）
# `SignProbe` 原来把 pan 写死在里面：采样 err[0]、jog pan、回写 pan。
# 轴参数化之后多了两个**不报错、只给错结论**的失败方式：
#   ① 采样仍读 err_x —— tilt 动了它也不变，量出来是 0（或纯噪声）；
#   ② `_send_axis` 把 MODE_MANUAL 的 pan/tilt 两路写反 —— 于是标定动的是
#      另一个轴，甚至把未受控那一路填成 None（板端当 0，一把拽到底）。
# 两者都不会抛异常，只会让操作者拿着一个"看起来正常"的错斜率去跑闭环。
# 所以 tilt 这一支必须有自己的回归。


class TiltProbeRig:
    """tilt 版标定台：驱动模型与 ProbeRig 同源，但走的是**竖**轴。

    pan 侧全程不动（cx 恒为 160 ⟹ err_x 恒为 0），所以一旦采样读错分量，
    这里必然量不出位移而判失败——这正是要钉住的那一点。
    """

    def __init__(self, tilt_ref=242.0, px_per_tick=PX_PER_TICK, jog=None,
                 tick_ms=SLEW_MS, drive=True):
        self.tilt_ref = float(tilt_ref)
        self.cur = float(tilt_ref)
        self.tgt = int(tilt_ref)
        self.px_per_tick = px_per_tick    # px per tick-UP；可负 = 装反
        self.drive = drive
        self.calls: list[tuple] = []
        self.t = 0.0
        self._acc = 0.0
        self.m = FakeManual()
        self.m.cli = self
        self.m.guard = self._guard
        self.probe = SignProbe(self.m, axis="tilt", slew_ms=tick_ms, jog=jog)
        self.probe._now = lambda: self.t

    def _guard(self):
        # pan 侧的遥测值就是探针该原样带回去的那个数；写成别的值，
        # 「未受控轴必须保持原值」那条断言就失去意义了。
        return FakeTelem(pan_cur=377, tilt_cur=int(round(self.cur)))

    def set_target(self, mode, pan_tick=None, tilt_tick=None, immediate=False):
        self.calls.append((mode, pan_tick, tilt_tick))
        if self.drive and tilt_tick is not None:
            self.tgt = int(tilt_tick)
        return len(self.calls)

    def cy(self) -> float:
        return 120.0 + (self.cur - self.tilt_ref) * self.px_per_tick

    def _run(self, ms: float, t0: float) -> float:
        self._acc += ms
        t = t0
        while self._acc >= SLEW_MS:
            self._acc -= SLEW_MS
            t += SLEW_MS / 1000.0
            if self.cur < self.tgt:
                self.cur = min(self.cur + 1.0, float(self.tgt))
            elif self.cur > self.tgt:
                self.cur = max(self.cur - 1.0, float(self.tgt))
        return t

    def frame(self, dt_ms=FRAME_MS_30, lost=False):
        tgt = (LOST if lost else
               Target(found=True, cx=160.0, cy=self.cy(), area=2000.0))
        st = self.probe.update(tgt, W, H)
        self.t = self._run(dt_ms, self.t)
        self.t += dt_ms / 1000.0
        return st

    def run_to_end(self, max_frames=400):
        for _ in range(max_frames):
            if self.probe.done:
                break
            self.frame()
        assert self.probe.done, f"标定没在 {max_frames} 帧内结束"
        return self.probe


def test_tilt_probe_reads_the_vertical_component():
    """tilt 标定必须读 err_y：pan 全程不动，读错分量就量不出位移。"""
    p = TiltProbeRig().run_to_end()
    assert p.ok, p.m.note
    assert p.slope > 0, "tick↑ 让 cy 变大，斜率应当为正"
    assert 1.0 < p.slope < 4.0, f"斜率应当是 px/tick 的量级（仿真 2.3），得到 {p.slope}"
    assert p.kp_rec < 0, "斜率为正就必须用负 kp"


def test_tilt_probe_moves_tilt_and_never_drags_pan():
    """受控轴动、未受控轴**原样带上**。填 None 的话板端当 0，pan 会被拽到底。"""
    rig = TiltProbeRig()
    p = rig.run_to_end()
    assert p.dir == +1, "242 + 12 ≤ 342，应当往 tick 增大的方向走"
    assert any(c[2] != 242 for c in rig.calls), "根本没动 tilt"
    assert all(c[1] == 377 for c in rig.calls), \
        f"pan 被带着动了：{[c[1] for c in rig.calls]}"


def test_tilt_probe_restores_tilt_to_the_starting_tick():
    """不管成功还是失败，最后一下都得把 tilt 放回起点。"""
    rig = TiltProbeRig()
    p = rig.run_to_end()
    last = rig.calls[-1]
    assert last[0] == P.MODE_MANUAL
    assert p.ax0 == 242
    assert last[2] == 242, f"收尾没有归还到起点：{last!r}"
    assert last[1] == 377, f"归还时把 pan 也带歪了：{last!r}"


def test_tilt_probe_flips_kp_when_the_axis_is_mounted_backwards():
    """tilt 装反了 → kp 必须翻成正的，理由与 pan 同一套。"""
    p = TiltProbeRig(px_per_tick=-PX_PER_TICK).run_to_end()
    assert p.ok, p.m.note
    assert p.slope < 0
    assert p.kp_rec > 0


def test_tilt_probe_reverses_direction_at_the_upper_limit():
    """tilt 可用行程窄（162..342），贴上限是常态而不是特例。"""
    rig = TiltProbeRig(tilt_ref=338.0)     # 338 + 12 > 342，走不了
    p = rig.run_to_end()
    assert p.dir == -1, "上限走不动就该改往反方向"
    assert p.slope > 0 and p.kp_rec < 0, "方向翻了，结论不该跟着翻"
    assert max(c[2] for c in rig.calls) <= 342, "下发越过了驱动安全上限"


def test_auto_render_names_the_axis_it_controls():
    """控 tilt 时还印 errx 就是在撒谎——那一路根本没参与控制。"""
    m = FakeManual()
    a = Auto(m, axis="tilt")
    for _ in range(6):
        a.update(found(160.0, cy=150.0), W, H)
    s = a.render()
    assert "erry" in s and "errx" not in s, s


# ---------------------------------------------------------- 正反馈闸门

def test_sign_gate_refuses_a_kp_that_matches_the_slope():
    """kp × 斜率 > 0 → 误差逐帧被放大。拒绝进环，且拦下来之前不发任何动作。"""
    m = FakeManual()
    a = Auto(m, kp=+0.5, slope=+2.3)
    for _ in range(6):
        a.update(found(200.0), W, H)
    assert a.sign_tripped, "符号自相矛盾却没拦下来"
    assert P.MODE_TRACK not in m.cli.modes(), "拦下来之前居然发过跟踪指令"
    assert P.MODE_HOLD in m.cli.modes(), "拒绝之后应当回 HOLD"
    assert a.suspended


def test_sign_gate_allows_a_kp_that_opposes_the_slope():
    m = FakeManual()
    a = Auto(m, kp=-0.5, slope=+2.3)
    for _ in range(6):
        a.update(found(200.0), W, H)
    assert not a.sign_tripped
    assert P.MODE_TRACK in m.cli.modes()


def test_sign_gate_is_off_when_the_slope_was_never_measured():
    """没量过就不拦（否则第一次实机根本跑不起来），只在启动横幅上警告。"""
    m = FakeManual()
    a = Auto(m, kp=+0.5, slope=None)
    for _ in range(6):
        a.update(found(200.0), W, H)
    assert not a.sign_tripped
    assert P.MODE_TRACK in m.cli.modes()


def test_single_axis_tilt_takes_its_slope_from_dash_slope():
    """`--axis tilt` 只有一条轴：`--slope` 必须落到 tilt 上，闸门才量得起来。

    回归自 2026-10-07 实机：跑 `--axis tilt --kp -0.50 --slope +1.83` 时
    闸门全程是关的（横幅原文「--slope-tilt 未给 ... 该轴的正反馈闸门是开着的」），
    因为 tilt 的 slope 只从 --slope-tilt 取，而那个开关在非 both 下被
    parameter 校验直接拒掉。正斜率配正 kp 就是正反馈，闸门必须拦得住。
    """
    m = FakeManual()
    a = Auto(m, axis="tilt", kp=-0.5, slope=+1.12)
    assert a.param["tilt"]["slope"] == +1.12, "单轴 tilt 没接住 --slope"
    for _ in range(6):
        a.update(found(160.0, cy=150.0), W, H)
    assert not a.sign_tripped, "符号相反（收敛）却被拦了"

    m2 = FakeManual()
    a2 = Auto(m2, axis="tilt", kp=+0.5, slope=+1.12)      # 同号 = 正反馈
    for _ in range(6):
        a2.update(found(160.0, cy=150.0), W, H)
    assert a2.sign_tripped, "单轴 tilt 的闸门仍然没生效"
    assert P.MODE_TRACK not in m2.cli.modes(), "拦下来之前发过动作"


def test_single_axis_banner_names_the_flag_that_actually_works():
    """单轴时横幅不能叫人去给 --slope-tilt——那个开关会被校验直接拒掉。"""
    m = FakeManual()
    a = Auto(m, axis="tilt", kp=-0.5, slope=None)
    txt = "\n".join(auto_banner(a, 2.0, 4, 5, 490))
    assert "--slope-tilt" not in txt, f"单轴却指向了 both 专用开关：{txt}"
    assert "--slope" in txt, f"该用 --slope 却没说：{txt}"


def test_both_mode_still_keeps_the_two_slopes_apart():
    """`--axis both` 的老规矩不动：tilt 不回落 pan 的斜率（两轴反号）。"""
    m, a = mk_both(slope_tilt=None)
    assert a.param["pan"]["slope"] == -2.3
    assert a.param["tilt"]["slope"] is None, "both 下 tilt 回落了 pan 的斜率"
    txt = "\n".join(auto_banner(a, 2.0, 4, 5, 490))
    assert "--slope-tilt" in txt, f"both 下该点的是 --slope-tilt：{txt}"


def test_measured_slope_catches_a_wrong_kp_in_the_real_flow():
    """把标定量出来的斜率喂给闭环，反向的 kp 必须被拦。

    这条把两半接起来：标定只管量，闸门只管拦，中间那根线就是 --slope。
    """
    p = ProbeRig().run_to_end()
    assert p.ok
    m = FakeManual()
    a = Auto(m, kp=-p.kp_rec, slope=p.slope)     # 故意用反的那个
    for _ in range(6):
        a.update(found(200.0), W, H)
    assert a.sign_tripped, "标定量出来的斜率没有拦住反向 kp"


# ------------------------------------------------------------ 双轴同控（--axis both）

def mk_both(m=None, **kw):
    """双轴的标准配置：两条轴的 kp 是**反号**的。

    实测 pan 斜率 −1.83 px/tick、tilt +1.83 px/tick（大小一样、符号相反），
    下面这两个 kp 就是照这两条斜率配的。写反任何一个都会让闸门在真机上报警。
    """
    kw.setdefault("kp", +0.5)
    kw.setdefault("slope", -2.3)
    kw.setdefault("kp_tilt", -0.5)
    kw.setdefault("slope_tilt", +2.3)
    kw.setdefault("axis", "both")
    m = m or FakeManual()
    return m, Auto(m, **kw)


def test_both_axes_ride_in_one_packet():
    """协议一个包里本来就有两个 tick 字段，一帧只能发**一个**包。

    分两包发的后果不是浪费带宽，而是驱动会先后看到「新 pan + 旧 tilt」和
    「新 pan + 新 tilt」——两个中间态里有一个是假的，丢包时它可能变成终态。
    """
    m, a = mk_both()
    for _ in range(6):
        a.update(found(200.0, cy=150.0), W, H)      # err_x=+40 err_y=+30
    tracks = [c for c in m.cli.calls if c[0] == P.MODE_TRACK]
    assert len(tracks) == 3, f"6 帧里第 4~6 帧进闭环，应有 3 包，实得 {len(tracks)}"
    for mode, pan, tilt, _ in tracks:
        assert pan is not None and tilt is not None, \
            f"有一包只带了一条轴（另一条轴没参与控制）：{(mode, pan, tilt)}"


def test_both_axes_use_their_own_kp():
    """同样的误差，pan 与 tilt 必须朝**相反**方向动。

    这是双轴的核心：两条轴的画面坐标符号相反，共用一个 kp 必有一条是正反馈。
    只用 `self.kp` 的实现在这里会把 tilt 也往上推，测试立刻红。
    """
    m, a = mk_both()
    for _ in range(4):
        a.update(found(200.0, cy=150.0), W, H)      # err_x=+40 err_y=+30
    mode, pan, tilt, _ = m.cli.calls[-1]
    assert mode == P.MODE_TRACK, m.cli.calls[-1]
    assert pan > 377, f"kp=+0.50 × err_x=+40 应让 pan 增大，实得 {pan}"
    assert tilt < 242, f"kp_tilt=-0.50 × err_y=+30 应让 tilt 减小，实得 {tilt}"


def test_both_axes_deadband_is_per_axis():
    """一条轴落在死区里不该把另一条也拖停：仍然发一包，死区那条留 None。

    None 在协议里的含义是「沿用客户端上一次的值」——正因为它不动，
    才不能让这条安静下来的轴把另一条也拖住。

    **两个方向都要测**：`_aim` 是按 `self.axes` 顺序攒 `want` 的，
    只测「静的那条排在前面」时，"一轴返 None 就整包作废" 这种写法
    恰好也能蒙对——必须让静的那条排在**后面**才暴露它。
    """
    for cx, cy, still, moving, want in ((160.0, 150.0, "pan", "tilt", (None, 238)),
                                        (200.0, 120.0, "tilt", "pan", (381, None))):
        m, a = mk_both()
        for _ in range(4):
            a.update(found(cx, cy), W, H)
        assert m.cli.calls, \
            f"cx={cx} cy={cy}：整帧一包都没发——{moving} 被死区里的 {still} 拖停了"
        mode, pan, tilt, _ = m.cli.calls[-1]
        assert mode == P.MODE_TRACK, (cx, cy, m.cli.calls[-1])
        got = {"pan": pan, "tilt": tilt}
        assert (pan, tilt) == want, \
            f"cx={cx} cy={cy}（{still} 在死区）应发 {want}，实得 {(pan, tilt)}"
        assert got[still] is None, \
            f"{still} 在死区内不该给新 tick，实得 {got}"
        assert got[moving] is not None, \
            f"{moving} 该照动，却被死区里的 {still} 拖停了：{got}"


def test_sign_gate_is_per_axis_and_names_the_bad_one():
    """pan 合法、tilt 反号 → **整体**拒绝进环，且报错要指名道姓。

    只报「符号不对」而不说是哪条轴，操作者手上只有一个 kp 的时候根本不知道
    该改哪个。这条同时钉住：拒绝发生在下发之前（一个跟踪包都没有）。
    """
    m, a = mk_both(kp_tilt=+0.5)                    # 与 tilt 斜率 +2.3 同号
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        for _ in range(6):
            a.update(found(200.0, cy=150.0), W, H)
    out = buf.getvalue()
    assert a.sign_tripped, "tilt 反号却没拦下来"
    assert a._bad_axis == "tilt", f"点错了轴：{a._bad_axis}"
    assert P.MODE_TRACK not in m.cli.modes(), "拦下来之前居然发过跟踪指令"
    assert "--kp-tilt" in out and "tilt" in out, f"报错没指名轴：{out!r}"


def test_both_axis_render_shows_both_errors():
    """两条轴都参与控制，就两条误差都得印——只印一条是在藏信息。"""
    m, a = mk_both()
    for _ in range(6):
        a.update(found(200.0, cy=150.0), W, H)      # err_x=+40 err_y=+30
    s = a.render()
    assert "errx=+40" in s, s
    assert "erry=+30" in s, s


def test_auto_render_surfaces_the_watchdog():
    """`--auto` 下 HUD 走 Auto.render()，看门狗告警不能在这里丢掉。

    这条是实测撞出来的：2026-10-07 双轴跑 daemon 看门狗触发 9 次，
    `Manual.render()` 明明有那一句，`Auto.render()` 没有，整跑 HUD
    一次 `⚠看门狗` 都没出现。宪章第 7 条「失败必须可见」。
    """
    m, a = mk_both(m=FakeManual(FakeTelem(watchdog_trips=0, daemon_wd_trips=9)))
    for _ in range(6):
        a.update(found(200.0, cy=150.0), W, H)
    s = a.render()
    assert "⚠看门狗" in s, f"daemon 看门狗触发 9 次却没印：{s}"
    assert "驱动0/daemon9" in s, s

    m2, a2 = mk_both()
    for _ in range(6):
        a2.update(found(200.0, cy=150.0), W, H)
    assert "看门狗" not in a2.render(), "没触发过就别印告警"


def test_banner_warns_that_the_tilt_gate_is_wide_open():
    """只给了 pan 的 --slope 就跑双轴，横幅必须**明说 tilt 那条门是开的**。

    这是双轴唯一一个能让人上电就飞车的配置：pan 的闸门亮着绿灯，
    tilt 那条压根没量过。合印一行「闸门：收敛」会让人以为两条都验过了。
    """
    m, a = mk_both(slope_tilt=None)                  # 故意不量 tilt
    txt = "\n".join(auto_banner(a, 2.0, 4, 5, 490))
    assert "--slope-tilt" in txt, f"没点出缺哪个参数：{txt}"
    assert "tilt" in txt and "没实测" in txt, txt
    assert "pan 闸门" in txt and "收敛" in txt, f"pan 那条闸门状态丢了：{txt}"
    assert "tilt 闸门" not in txt, f"tilt 没量却印了闸门结论：{txt}"


def test_banner_shows_each_axis_gate_separately():
    """两条轴都量过：各印各的。符号相反的两个 kp 必须分行，不能合成一个数。"""
    m, a = mk_both()
    txt = "\n".join(auto_banner(a, 2.0, 4, 5, 490))
    assert "pan 闸门：kp=+0.50 × 实测斜率 -2.30 = -1.15" in txt, txt
    assert "tilt 闸门：kp=-0.50 × 实测斜率 +2.30 = -1.15" in txt, txt


def test_banner_names_the_axis_that_would_be_refused():
    """闸门预演：给一个会让 tilt 发散的双轴配置，横幅当场就说明会据谁。"""
    m, a = mk_both(kp_tilt=+0.5)
    txt = "\n".join(auto_banner(a, 2.0, 4, 5, 490))
    assert "tilt 闸门" in txt and "发散" in txt, txt
    assert "pan 闸门" in txt and "收敛" in txt, txt


# ===================== ROI 择优锁定（--roi） =====================
#
# 这一组的验的是什么、**不**验什么，先讲清楚：
#
# 验：给定每一格的样本，择优**选中的是不是该选的那一格**——尤其是
#     "按中位数"和"按最小值"会不会选出不同的格（这正是设计上的分水岭）。
# 不验：真机上 r 到底是多少。那是检测器的事，也是 `--calib-lash` /
#     静止目标那两次实机测量的事。这里把 r 脚本化，是为了让"选优逻辑"
#     可以被单独钉住，而不是把两件事混在一次实机跑里分不清谁错了。

ROI_ANCHOR_PAN = 377


class RoiCli:
    """记录下发的位姿，并把它当成云台"实际"到的地方。

    真机上云台要爬 20 ms/tick 才到位，这里瞬间到位——择优关心的是
    "各格的样本怎么比"，不是"爬得快不快"，后者由 SimPlant 那组管。
    """

    def __init__(self, rig):
        self.rig = rig
        self.calls: list = []

    def set_target(self, mode, pan_tick=None, tilt_tick=None,
                   immediate=False):
        self.calls.append((mode, pan_tick, tilt_tick))
        if mode == P.MODE_TRACK and pan_tick is not None:
            self.rig.pose = (pan_tick, tilt_tick)
            self.rig.t.pan_cur = pan_tick
            self.rig.t.tilt_cur = tilt_tick
        return len(self.calls)

    def modes(self) -> list:
        return [c[0] for c in self.calls]

    def poses(self) -> list:
        return [(c[1], c[2]) for c in self.calls if c[0] == P.MODE_TRACK]


class RoiRig:
    """ROI 择优的测试台。时钟是假的（否则一条 hold_s=2s 的用例要跑 2 秒）。

    ``r_of(pose, 该格第几帧)`` 给出这一帧的误差半径。keying 用**帧序号**
    而不是采样序号：`_probe_update` 会跳掉每格开头 POSE_SETTLE_S 的那几帧，
    用帧序号脚本才好和状态机的节奏对上。
    """

    def __init__(self, r_of, roi=10.0, hold_s=2.0, dt=0.05, axis="pan",
                 pan_cur=ROI_ANCHOR_PAN, tilt_cur=242,
                 pan_safe_min=239, pan_safe_max=515):
        self.t = FakeTelem(pan_cur=pan_cur, tilt_cur=tilt_cur,
                           pan_safe_min=pan_safe_min, pan_safe_max=pan_safe_max)
        self.m = FakeManual(self.t)
        self.pose = (pan_cur, tilt_cur)
        self.n: dict = {}
        self.r_of = r_of
        self.dt = dt
        self.angle = 0.0            # 误差全落在 errx 上（单轴的默认）
        self.now = [0.0]
        self.cli = RoiCli(self)
        self.m.cli = self.cli
        self.a = Auto(self.m, kp=0.5, slope=-1.83, deadband=2.0, max_step=7,
                      acquire_n=1, release_m=3, axis=axis, roi=roi,
                      hold_s=hold_s)
        self.a._now = lambda: self.now[0]

    def frame(self, r=None):
        """喂一帧：r 不给就按 (当前位姿, 该格第几帧) 查脚本。"""
        k = self.n.get(self.pose, 0)
        self.n[self.pose] = k + 1
        if r is None:
            r = self.r_of(self.pose, k)
        ex, ey = r * math.cos(self.angle), r * math.sin(self.angle)
        self.a.update(found(160.0 + ex, 120.0 + ey), 320, 240)
        self.now[0] += self.dt
        return r

    def run(self, frames):
        for _ in range(frames):
            self.frame()
        return self.a.state


def _flat(by_offset):
    """脚本：按"相对锚点的 pan 偏移"给一个固定 r。

    收 dict 而不是 **kwargs：**kwargs 的键必须是字符串，而这里的键
    恰恰是 -1/0/+1 这些**数字偏移**，写成 kwargs 直接 TypeError。
    """
    def r_of(pose, k):
        return by_offset.get(pose[0] - ROI_ANCHOR_PAN, 99.0)
    return r_of


def _settle_roi(rig, frames=120):
    """跑到锁定态（或超时）。返回状态。"""
    for _ in range(frames):
        rig.frame()
        if rig.a.state == "HOLD":
            return rig.a.state
    return rig.a.state


def test_roi_off_keeps_the_plain_p_behaviour():
    """没开 --roi 时必须还是老的纯 P：ROI 那套状态一个都不许进。

    这是回归护栏：ROI 是加在 `update()` 中间的一条分支，走错了就会把
    所有既有 3D 行为悄悄换掉，而既有那 40 多条用例根本不看 ROI 状态。
    """
    m, a = mk(kp=0.5, slope=-1.83, acquire_n=1)
    assert a.roi is None, f"没给 --roi，a.roi 却是 {a.roi}"
    seen = set()
    for cx in (200.0, 190.0, 175.0, 162.0, 160.0, 158.0):
        seen.add(a.update(found(cx), 320, 240))
    assert not (seen & {"HOLD", "HOLD_PROBE"}), \
        f"没开 --roi 却进了 ROI 状态：{seen}"
    assert a.hold_pose is None and a.hold_scores is None, \
        "没开 --roi 却记了择优的账"


def test_roi_probe_visits_every_candidate_and_locks_the_best():
    """探完所有候选格，锁定中位数最小的那格。"""
    rig = RoiRig(_flat({-1: 12.0, 0: 3.0, 1: 8.0}), roi=10.0)
    st = _settle_roi(rig)
    assert st == "HOLD", f"应锁定，实得 {st}"
    chosen = rig.a.hold_pose
    assert chosen == (ROI_ANCHOR_PAN, 242), \
        (f"应选 pan={ROI_ANCHOR_PAN}（中位数 3.0px），实得 {chosen}；"
         f"账={rig.a.hold_scores}")
    assert len(rig.a.hold_scores) == 3, \
        f"单轴应有 3 格候选，实得 {len(rig.a.hold_scores)}：{rig.a.hold_scores}"
    visited = {p for _, p in rig.a.hold_scores}
    assert visited == {(ROI_ANCHOR_PAN - 1, 242), (ROI_ANCHOR_PAN, 242),
                       (ROI_ANCHOR_PAN + 1, 242)}, f"探过的格不对：{visited}"


def test_roi_picks_median_not_min():
    """**按中位数**和**按最小值**必须选出不同的格——这条钉住设计的分水岭。

    -1 格：11 帧里 10 帧是 8.0px，1 帧是 0.4px（一次检测抖动）
      0 格：11 帧全是 3.0px
    最小值会选 -1 格（0.4 < 3.0），中位数选 0 格（8.0 > 3.0）。
    瞬时最小拿到的是**噪声底**而不是位姿质量，所以正确答案是 0 格。
    """
    spike = {-1: None, 0: 3.0, 1: 9.0}

    def r_of(pose, k):
        off = pose[0] - ROI_ANCHOR_PAN
        if off == -1:
            return 0.4 if k == 5 else 8.0
        return spike[off]

    rig = RoiRig(r_of, roi=10.0)
    st = _settle_roi(rig)
    assert st == "HOLD", f"应锁定，实得 {st}"
    med = dict((p, m) for m, p in rig.a.hold_scores)
    assert med[(ROI_ANCHOR_PAN - 1, 242)] == 8.0, \
        f"-1 格的中位数应是 8.0，实得 {med}（0.4 那一帧是离群点，不该主导）"
    assert rig.a.hold_pose == (ROI_ANCHOR_PAN, 242), \
        (f"按中位数应选 0 格（3.0px），实得 {rig.a.hold_pose}。"
         f"若选中 -1 格，说明退回了「窗口内瞬时最小」那种取法。账={med}")


def test_roi_locked_pose_stops_being_commanded():
    """锁定后一个目标值都不再改。"""
    rig = RoiRig(_flat({-1: 12.0, 0: 3.0, 1: 8.0}), roi=10.0)
    _settle_roi(rig)
    n0 = len(rig.cli.calls)
    for _ in range(30):
        rig.frame(r=2.0)
    assert rig.a.state == "HOLD", f"锁定期内跑出了 HOLD：{rig.a.state}"
    assert len(rig.cli.calls) == n0, \
        (f"锁定期又下发了 {len(rig.cli.calls) - n0} 次——锁定就等于"
         f"「目标值不动」，靠心跳续命就够了")


def test_roi_hysteresis_does_not_reprobe_at_the_boundary():
    """门限附近来回抖不许触发重探（那才是迟滞在这里的用处）。

    r 在 roi 与 roi*1.5 之间来回时：既不出锁，也不重跑一轮 hold_s。
    单阈值的话每次越过就会重探一次，2 s 一轮全花在这上面。
    """
    rig = RoiRig(_flat({-1: 8.0, 0: 3.0, 1: 9.0}), roi=10.0)
    _settle_roi(rig)
    assert rig.a.state == "HOLD"
    n0 = len(rig.cli.calls)
    for r in (12.0, 11.0, 14.0, 9.0, 13.0, 10.5):
        rig.frame(r=r)
    assert rig.a.state == "HOLD", \
        (f"r 在 {10.0:g}~{15.0:g} 之间（roi=10、出门限 15）却掉出了锁定："
         f"{rig.a.state}")
    assert len(rig.cli.calls) == n0, "门限附近抖动不该重新下发/重探"


def test_roi_releases_when_target_really_leaves():
    """真出了 ROI 就得放手，回去连续跟踪——否则锁在一个目标已经不在的位姿上。"""
    rig = RoiRig(_flat({-1: 8.0, 0: 3.0, 1: 9.0}), roi=10.0)
    _settle_roi(rig)
    assert rig.a.state == "HOLD"
    for _ in range(4):
        rig.frame(r=40.0)
    assert rig.a.state == "TRACK", \
        f"r=40px 连续 4 帧（门限 15px、release_m=3）应已放手，实得 {rig.a.state}"
    assert rig.a.hold_pose is None, "放手后锁定的位姿应该清掉"


def test_roi_single_axis_never_moves_the_other_axis():
    """--axis pan 的择优只准动 pan。

    `_send_pose` 是把 (pan, tilt) 一起下发的，候选偏移里带一个 dy 就会顺手
    把 tilt 挪一格——真机上那是"另一个轴自己动了"，而操作者会以为 tilt 是
    手动的。初版就漏了这一条。
    """
    rig = RoiRig(_flat({-1: 12.0, 0: 3.0, 1: 8.0}), roi=10.0, axis="pan")
    _settle_roi(rig)
    bad = [p for p in rig.cli.poses() if p[1] != 242]
    assert not bad, f"--axis pan 的择优动了 tilt：{bad}"


def test_roi_single_axis_probes_only_three_poses():
    """单轴 3 格、双轴 5 格。多出来的格是在探一条不归自己管的轴。"""
    one = RoiRig(_flat({-1: 8.0, 0: 3.0, 1: 9.0}), roi=10.0, axis="pan")
    assert len(one.a.probe_poses()) == 3, one.a.probe_poses()
    both = RoiRig(_flat({-1: 8.0, 0: 3.0, 1: 9.0}), roi=10.0, axis="both")
    assert len(both.a.probe_poses()) == 5, both.a.probe_poses()


def test_roi_degenerate_does_not_fake_a_score():
    """行程被夹成只有一个位姿时：就地锁定，且**账是空的**。

    不许拿一个"探过的格"去冒充择优结果——那会让操作者以为比较过。
    """
    rig = RoiRig(_flat({0: 3.0}), roi=10.0,
                 pan_safe_min=ROI_ANCHOR_PAN, pan_safe_max=ROI_ANCHOR_PAN)
    rig.frame()
    assert rig.a.state == "HOLD", f"应就地锁定，实得 {rig.a.state}"
    assert rig.m.tag == "HOLD-FIXED", \
        f"应是 HOLD-FIXED（明确说没择优），实得 {rig.m.tag}"
    assert rig.a.hold_scores is None, \
        f"没探过却记了账：{rig.a.hold_scores}"
    assert rig.a.hold_pose == (ROI_ANCHOR_PAN, 242), rig.a.hold_pose


def test_roi_abandons_the_round_when_target_leaves_midway():
    """择优中途目标跑了：整轮作废，不许拿不可比的样本去选。"""
    rig = RoiRig(_flat({-1: 8.0, 0: 3.0, 1: 9.0}), roi=10.0)
    rig.frame()
    assert rig.a.state == "HOLD_PROBE", rig.a.state
    for _ in range(3):
        rig.frame(r=30.0)
    assert rig.a.state == "TRACK", \
        f"中途出 ROI 应作废本轮回到 TRACK，实得 {rig.a.state}"
    assert rig.a.hold_scores is None, "作废的一轮不该留下账"


def test_roi_render_shows_the_locked_pose():
    """HUD 必须印出锁在哪。只印状态词的话操作者看不见它做了决定。"""
    rig = RoiRig(_flat({-1: 8.0, 0: 3.0, 1: 9.0}), roi=10.0)
    _settle_roi(rig)
    out = rig.a.render()
    assert "ROI10" in out, out
    assert f"锁({ROI_ANCHOR_PAN}, 242)" in out or "锁" in out, out


def test_banner_warns_when_roi_is_smaller_than_a_tick():
    """ROI 半径小于一格分度值就白开——横幅要先说。"""
    m, a = mk(kp=0.5, slope=-1.83, roi=1.0, hold_s=2.0)
    txt = "\n".join(auto_banner(a, 2.0, 4, 5, 490))
    assert "小于一格的像素当量" in txt, f"半径 1px < 1.83px 却没警告：\n{txt}"
    m2, a2 = mk(kp=0.5, slope=-1.83, roi=8.0, hold_s=2.0)
    txt2 = "\n".join(auto_banner(a2, 2.0, 4, 5, 490))
    assert "小于一格的像素当量" not in txt2, f"半径 8px 报什么警告：\n{txt2}"
    assert "ROI 择优" in txt2, txt2


def test_csv_writes_a_row_per_frame_with_both_errors():
    """逐帧 CSV 是可解性的一部分：列不全或漏帧，2 分钟跑完还是看不出极限环。

    这里钉的是**列与帧数**；数值对不对是上游（遥测/检测）的事。
    """
    import csv as _csv
    import os
    import tempfile
    from main import CsvLog

    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    try:
        m, a = mk(kp=0.5, slope=-1.83, acquire_n=1)
        m._t = FakeTelem(pan_cur=400, tilt_cur=250, watchdog_trips=2,
                         daemon_wd_trips=3)
        log = CsvLog(path)
        for cx in (200.0, 190.0, 180.0):
            a.update(found(cx), 320, 240)
            log.row(m, a)
        log.close()

        with open(path, newline="") as f:
            rows = list(_csv.DictReader(f))
        assert len(rows) == 3, f"3 帧应写 3 行，实得 {len(rows)}"
        want = {"t", "state", "tag", "errx", "erry", "pan_cur", "tilt_cur",
                "pan_tgt", "tilt_tgt", "cmd_pan", "cmd_tilt", "moves",
                "limit_hits", "wd", "dwd"}
        assert set(rows[0]) == want, f"列不对：缺 {want - set(rows[0])}，"\
                                     f"多 {set(rows[0]) - want}"
        assert float(rows[0]["errx"]) == 40.0, rows[0]
        assert rows[0]["pan_cur"] == "400" and rows[0]["wd"] == "2" \
            and rows[0]["dwd"] == "3", rows[0]
    finally:
        os.unlink(path)
    print("  [ok] 逐帧 CSV：15 列齐全、一帧一行、遥测与看门狗计数都在")


def test_csv_skips_a_frame_without_telemetry():
    """没有新鲜遥测的那一帧**不写**，不许填 0。

    填 0 会让"没有遥测"和"遥测说姿态在 0"在 CSV 里长得一模一样，
    而后者是一个根本不存在的姿态（tick 0 在护栏之外）。
    """
    import csv as _csv
    import os
    import tempfile
    from main import CsvLog

    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    try:
        m, a = mk(kp=0.5, slope=-1.83, acquire_n=1)
        log = CsvLog(path)
        m._t = FakeTelem(pan_cur=400)
        a.update(found(200.0), 320, 240)
        log.row(m, a)
        m._t = None                      # 遥测断了
        a.update(found(200.0), 320, 240)
        log.row(m, a)
        log.close()
        with open(path, newline="") as f:
            rows = list(_csv.DictReader(f))
        assert len(rows) == 1, \
            f"没遥测那帧应被跳过，实得 {len(rows)} 行：{rows}"
    finally:
        os.unlink(path)
    print("  [ok] 无遥测的帧不落 CSV（不拿 0 冒充姿态）")


# ------------------------------------------------------------- 回差（丢失行程）
#
# 这一组的关键是**仿真里必须真的有间隙**。理想 SimPlant 是 cur 精确追 tgt，
# 一点回差都没有；拿它跑 LashProbe，答案永远是 H=0——那什么也验证不了，
# 只会让人以为"量过了、没问题"。LashPlant 把间隙补上。
#
# 真值故意取 2.4 这个非整数 tick：回差和舵机静摩擦叠在一起，本来就很少落在
# 整格上，而"数步子"的写法只能给到 ⌈H⌉=3。取整数的话，两种估计法在测试里
# 长得一样，就白测了。

LASH_ANCHOR_PAN = 377
PLAY_TICKS = 2.4        # 丢失行程真值（tick）


class LashPlant(SimPlant):
    """带丢失行程的云台：输出只在 |tgt − cur| 超过 half 之后才跟。

    half = play/2（间隙在两侧各一半），所以**换向后要空走 play tick 画面才
    动**——驱动早就到位了，是传动间隙让输出每次都少走 half。play=0 就退化成
    理想 SimPlant。

    命令全是台阶且每步都等到走完，所以"输出直接落到 tgt ∓ half"和"输出连续
    爬过去再停下"在这里完全等价，不必真去模拟爬行。
    """

    def __init__(self, pan_ref=377.0, px_per_tick=PX_PER_TICK, play=PLAY_TICKS):
        super().__init__(pan_ref, px_per_tick)
        self.half = play / 2.0

    def run(self, ms: float, t0: float) -> float:
        t = t0
        self._acc += ms
        while self._acc >= SLEW_MS:
            self._acc -= SLEW_MS
            t += SLEW_MS / 1000.0
            if self.tgt > self.cur + self.half:
                self.cur = float(self.tgt) - self.half
            elif self.tgt < self.cur - self.half:
                self.cur = float(self.tgt) + self.half
        return t


class LashRig:
    """把 LashPlant 接到 LashProbe 上。时间由 rig 自己记、探针的时钟读它，
    所以真机上要 7 秒的一趟在这里是瞬时的，而且完全确定。"""

    def __init__(self, play=PLAY_TICKS, px_per_tick=PX_PER_TICK,
                 pan_ref=LASH_ANCHOR_PAN, pan_max=None, drive=True,
                 noise=0.0, seed=0):
        self.plant = LashPlant(pan_ref, px_per_tick, play)
        self.plant.px_offset = 60.0       # 目标先偏离中心，位移才看得出来
        self.m = SimManual(self.plant)
        self.calls: list[tuple] = []
        self.drive = drive
        self.m.cli = self
        self.t = 0.0
        # 检测噪声：单步位移才 2.3 px，而噪声是这一节真正的对手。种子写死，
        # 所以带噪声的用例照样是确定性的。
        self.noise = noise
        self.rnd = random.Random(seed)
        self.probe = LashProbe(self.m, pan_max=pan_max)
        self.probe._now = lambda: self.t

    def set_target(self, mode, pan_tick=None, tilt_tick=None, immediate=False):
        self.calls.append((mode, pan_tick, tilt_tick))
        if self.drive and pan_tick is not None:
            self.plant.set_target(pan_tick)
        return len(self.calls)

    def modes(self):
        return [c[0] for c in self.calls]

    def frame(self, dt_ms=FRAME_MS_30, lost=False):
        if lost:
            tgt = LOST
        else:
            n = (self.rnd.uniform(-self.noise, self.noise) if self.noise else 0.0)
            tgt = Target(found=True, cx=self.plant.cx() + n, cy=120.0,
                         area=2000.0)
        st = self.probe.update(tgt, W, H)
        self.plant.run(dt_ms, self.t)
        self.t += dt_ms / 1000.0
        return st

    def run_to_end(self, max_frames=800):
        for _ in range(max_frames):
            if self.probe.done:
                break
            self.frame()
        assert self.probe.done, f"回差标定没在 {max_frames} 帧内结束"
        return self.probe


def test_lash_probe_measures_the_lost_motion():
    """间隙 2.4 tick → H 必须量到 2.4，而不是数步子能给的 3。

    这是这一节的中心断言。d_i = k×(i−H)，所以相邻差的中位数是 k、截距是 H；
    第 3 步才动只说明 H ∈ (2,3]，而闭环要知道的是"要不要专门为它改策略"。
    """
    p = LashRig().run_to_end()
    assert p.ok, p.m.note
    assert p.first_move == 3, f"第 3 步才该动，得到 {p.first_move}"
    assert abs(p.lost - PLAY_TICKS) < 0.15, \
        f"H 应为 {PLAY_TICKS}，得到 {p.lost}"
    # 无噪声下必须**亚 tick 精确**：这就是拟合存在的理由。数步子只能给
    # first_move−1 = 2，和真值差 0.4——把这条钉住，退回数步子就会挂。
    assert abs(p.lost - (p.first_move - 1)) > 0.2, \
        f"H 落在整格上，说明它其实是在数步子：{p.lost}"
    assert p.k is not None and p.reliable
    assert abs(-p.dir * p.k - PX_PER_TICK) < 0.05, \
        f"光学斜率应为 {PX_PER_TICK}，得到 {-p.dir * p.k}"


def test_lash_probe_reports_zero_when_there_is_no_slack():
    """没有间隙就该报 0，而不是把"第一步就动"当成一次误判。"""
    p = LashRig(play=0.0).run_to_end()
    assert p.ok and p.fitted, p.m.note
    assert p.first_move == 1
    assert abs(p.lost) < 1e-6, f"没有间隙就是 0，得到 {p.lost}"
    assert p.phase == "DONE"


def test_lash_probe_gives_up_when_the_axis_never_moves():
    """驱动不动 → 量不到头就报"量不到头"，用 8 冒充 0.0 是最坏的答案。"""
    p = LashRig(drive=False).run_to_end()
    assert not p.ok and p.phase == "FAIL"
    assert p.lost is None, "没量到就不能有数"
    assert "≥" in p.summary(), p.summary()


def test_lash_probe_flags_a_slack_that_reaches_the_end_of_the_staircase():
    """间隙大到顶住台阶末端 → 数必须**标成不可信**，不能被当成普通读数用。

    台阶一共退 8 步；H=7.5 时斜段只剩半步，拟合只能在两个点上凑直线，解出来
    的 k（这里约 1.1）离真值（2.3）差一倍，H 也压在量程边界上。数不是没算
    出来，而是**条件数太差**——所以标 reliable=False，让 summary 和屏上那行
    都把话说明白，别让人拿着它去下结论。
    """
    p = LashRig(play=7.5).run_to_end()
    assert p.ok and p.fitted, p.m.note
    assert p.first_move == 8, f"第 8 步才该动，得到 {p.first_move}"
    assert not p.reliable, "斜段只有半步，不能算可信"
    assert abs(p.lost - 7.5) < 0.8, f"H 顶在量程边界上，得到 {p.lost}"
    assert "分辨率差" in p.summary(), p.summary()


def test_lash_probe_marks_a_healthy_reading_as_reliable():
    """对照面：斜段够长时不能顺手把数也标成不可信（那样标记就没信息了）。"""
    p = LashRig().run_to_end()
    assert p.fitted and p.reliable, p.summary()
    assert "分辨率差" not in p.summary(), p.summary()


def test_lash_probe_estimate_survives_detection_noise():
    """检测噪声下 H 不能飘：真值 2.4，±1.5 px 噪声时每个种子都得在 1 格内。

    噪声是这一节真正的对手——单步位移才 2.3 px，而中位数样本的 σ ≈ 0.9 px。
    种子写死，所以这是**确定性的**：换估计式（比如退回数步子）就会挂。
    """
    for seed in range(8):
        rig = LashRig(noise=1.5, seed=seed)
        p = rig.run_to_end()
        assert p.ok and p.fitted, f"seed{seed}: {p.m.note}"
        assert abs(p.lost - PLAY_TICKS) < 1.0, \
            f"seed{seed}: H 真值 {PLAY_TICKS}，估成 {p.lost:.2f}"
        assert abs(-p.dir * p.k - PX_PER_TICK) < 1.0, \
            f"seed{seed}: 光学斜率估成 {-p.dir * p.k:.2f}"


def test_lash_probe_restores_the_axis_to_the_start():
    """成功、失败、不动，三种收尾都得把 pan 放回起点。"""
    for rig in (LashRig(), LashRig(play=0.0), LashRig(drive=False)):
        p = rig.run_to_end()
        last = rig.calls[-1]
        assert last[0] == P.MODE_MANUAL
        assert last[1] == p.ax0, f"收尾没有归还到起点：{last!r}"


def test_lash_probe_never_touches_track_or_home():
    """只走 MODE_MANUAL：不借闭环那套，也不借 home（home 要动 tilt）。"""
    rig = LashRig()
    rig.run_to_end()
    assert P.MODE_TRACK not in rig.modes()
    assert P.MODE_HOME not in rig.modes()
    assert set(rig.modes()) <= {P.MODE_MANUAL}, rig.modes()


def test_lash_probe_steps_are_one_tick_each():
    """台阶必须是**每步一格**：一次退两格就分不出间隙是 1 格还是 2 格。

    pan 这条命令序列应当是：一大步吃掉间隙（起点+12），然后每步正好 −1 tick
    退出若干步，最后一下归回起点。中间的"每步一格"是这条断言的全部意义。
    """
    rig = LashRig()
    rig.run_to_end()
    pan = [c[1] for c in rig.calls if c[1] is not None]
    assert pan[0] - rig.probe.ax0 == rig.probe.dir * rig.probe.load, \
        f"第一条命令是吃掉间隙的那一大步：{pan[0]}"
    assert pan[-1] == rig.probe.ax0, "最后一下是归还"
    backs = pan[1:-1]                       # 反向退的那些步（去掉归还那一下）
    assert len(backs) == rig.probe.max_back, \
        f"该退满 {rig.probe.max_back} 步，得到 {len(backs)}"
    assert all(abs(b - a) == 1 for a, b in zip(backs, backs[1:])), \
        f"反向每步都该是一格：{backs}"


def test_lash_probe_sends_nothing_without_telem():
    """没有新鲜遥测时一个包都不发——Manual.guard() 那道闸门原样复用。"""
    m = FakeManual(telem=None)
    p = LashProbe(m)
    p._now = lambda: 0.0
    p.update(Target(found=True, cx=200.0, cy=120.0, area=2000.0), W, H)
    assert p.phase == "FAIL" and p.done
    assert m.cli.calls == [], f"没遥测还发了 {m.cli.calls}"


def test_lash_probe_abort_puts_the_axis_back():
    """中途按 q / 到点：也必须归还，不能把云台留在 起点+12 上。"""
    rig = LashRig()
    for _ in range(800):
        if rig.probe.phase == "LOAD":
            break
        rig.frame()
    assert rig.probe.phase == "LOAD", "还没进到载入阶段就没得中断了"
    rig.probe.abort()
    assert rig.probe.done
    assert rig.calls[-1][1] == rig.probe.ax0, f"中断没归还：{rig.calls[-1]!r}"


def test_lash_probe_reverses_direction_at_the_upper_limit():
    """起点贴着 pan 上限时改往反方向退，量出来的 H 必须**一样**。

    方向是探针自己选的，而 H 是机械性质，不该跟着方向变；变号的只有 k 的
    符号（所以"光学斜率 = −dir×k"那一步必须把 dir 算进去）。
    """
    rig = LashRig(pan_ref=510.0)          # 510 + 12 > 515，走不了
    p = rig.run_to_end()
    assert p.dir == -1, "上限走不动就该改往反方向"
    assert abs(p.lost - PLAY_TICKS) < 0.15, f"H 不该跟着方向变，得到 {p.lost}"
    assert abs(-p.dir * p.k - PX_PER_TICK) < 0.05
    assert max(c[1] for c in rig.calls) <= 515, "下发越过了驱动安全上限"


def test_lash_probe_jog_stays_under_the_track_pan_max():
    """跟踪上限比驱动上限紧时，标定也不能越过它。"""
    rig = LashRig(pan_ref=485.0, pan_max=490)
    p = rig.run_to_end()
    assert max(c[1] for c in rig.calls) <= 490
    assert p.dir == -1, "485 + 12 > 490，应该改往反方向"


def test_lash_probe_resamples_when_the_target_disappears():
    """采样途中丢目标就清空重来，不拿半截样本算中位数。"""
    rig = LashRig()
    for _ in range(240):
        if rig.probe.phase == "BASE":
            break
        rig.frame()
    assert rig.probe.phase == "BASE", rig.probe.phase
    rig.frame()
    n = len(rig.probe.samples)
    assert n > 0, "这一步该采到样本了"
    rig.frame(lost=True)
    assert rig.probe.samples == [], "丢目标必须清空，不能拿半截样本接着算"
    assert rig.run_to_end().ok, "目标回来之后应当照样量得出来"


def test_lash_probe_run_fits_in_the_two_minute_window():
    """整趟（含最坏情况：8 步退满）必须远短于 2 分钟，否则它会把闭环挤没。"""
    rig = LashRig()
    rig.run_to_end()
    assert rig.t < 12.0, f"整趟标定花了 {rig.t:.1f} s，太长"


def test_lash_tilt_rig_measures_the_vertical_slack():
    """tilt 侧：读 err_y、动 tilt、**pan 原样带上**。

    分量读错（comp 写死成 0）或把未受控轴填 None，在 pan 版测试里看不出来。
    """
    rig = LashTiltRig()
    p = rig.run_to_end()
    assert p.ok, p.m.note
    assert abs(p.lost - PLAY_TICKS) < 0.15, f"H 应为 {PLAY_TICKS}，得到 {p.lost}"
    assert all(c[0] == P.MODE_MANUAL for c in rig.calls), rig.modes()
    assert all(c[1] == 377 for c in rig.calls), \
        f"未受控轴 pan 必须原样带上：{[c[1] for c in rig.calls]}"


def test_hinge_fit_keeps_the_slant_at_least_two_points_long():
    """斜段不许短到只剩一点：单点拟合残差恒为 0，能凑出 |k|≈29 这种假斜率。

    k = d/x 对单点永远成立，所以残差检验对单点斜段没有分辨力——把"第 8 步
    才动"的真实台阶喂进去，折点会滑到 7.96、斜率变成 −28.75，**残差还比正解
    更小**。MIN_FIT_POINTS 把折线压在离台阶末端至少两点的地方，斜率才留在
    px/tick 的量级。这场合成的台阶直接钉住它，免得以后被"优化"掉。
    """
    h, k = LashProbe._fit_hinge([(i, 0.0) for i in range(1, 8)]
                                + [(8, -1.15)], 8)
    assert h is not None and h <= 7.0 + 1e-9, f"折点被顶到台阶末端：{h}"
    assert abs(k) < 4.0, f"斜率不是 px/tick 的量级，说明斜段只有一点：{k}"
    # 正面：斜段三点 → 折点和斜率都该解准，不能索性一概拒绝。
    h, k = LashProbe._fit_hinge([(i, 0.0) for i in range(1, 6)]
                                + [(6, -2.3), (7, -4.6), (8, -6.9)], 8)
    assert h is not None and abs(h - 5.0) < 0.05, f"折点应解在 5.0，得到 {h}"
    assert abs(k + 2.3) < 0.05, f"斜率应解在 −2.3，得到 {k}"


def test_lash_probe_summary_and_brief_are_ascii_and_short():
    """窗口上那行只能画 ASCII（见 Manual.tag 的说明），汇总行得能读。"""
    rig = LashRig()
    p = rig.run_to_end()
    assert p.brief().isascii(), p.brief()
    assert "回差标定" in p.summary(), p.summary()
    assert "2.4" in p.summary(), p.summary()


class LashTiltRig(TiltProbeRig):
    """tilt 版回差标定台：驱动模型换成带间隙的那一个，其余照搬。"""

    def __init__(self, tilt_ref=242.0, px_per_tick=PX_PER_TICK,
                 play=PLAY_TICKS, drive=True):
        super().__init__(tilt_ref, px_per_tick, tick_ms=SLEW_MS, drive=drive)
        self.half = play / 2.0
        self.probe = LashProbe(self.m, axis="tilt")
        self.probe._now = lambda: self.t

    def _run(self, ms: float, t0: float) -> float:
        self._acc += ms
        t = t0
        while self._acc >= SLEW_MS:
            self._acc -= SLEW_MS
            t += SLEW_MS / 1000.0
            if self.cur < self.tgt - self.half:
                self.cur = float(self.tgt) - self.half
            elif self.cur > self.tgt + self.half:
                self.cur = float(self.tgt) + self.half
        return t


def main() -> int:
    print("3D 闭环决策单测（无硬件）")
    tests = [
        test_acquire_needs_four_frames,
        test_lost_frame_inside_window_resets_acquire,
        test_release_holds_five_frames,
        test_deadband_issues_nothing,
        test_step_is_capped_and_signed,
        test_no_subtick_command,
        test_target_is_recomputed_from_cur_not_accumulated,
        test_pan_max_tightens_client_side,
        test_driver_safe_range_also_applies,
        test_no_telem_means_no_command,
        test_suspend_and_resume,
        test_pace_warns_when_lead_is_too_short,
        test_pace_silent_when_lead_is_long_enough,
        test_low_frame_rate_hint,
        test_closed_loop_converges_not_diverges,
        test_no_added_stall_over_physical_floor,
        test_30fps_beats_15fps_on_continuity,
        test_max_step_one_gives_uniform_single_tick_steps,
        test_stationary_target_does_not_dither,
        test_deadband_five_batches_steps,
        test_probe_measures_positive_slope_and_asks_for_negative_kp,
        test_probe_flips_kp_when_the_camera_is_mounted_backwards,
        test_probe_reverses_direction_at_the_upper_limit,
        test_probe_jog_stays_under_the_track_pan_max,
        test_probe_restores_pan_to_the_starting_tick,
        test_probe_gives_up_when_motion_is_too_small,
        test_probe_resamples_when_the_target_disappears,
        test_probe_never_touches_track_or_home,
        test_probe_sends_nothing_without_telem,
        test_probe_abort_puts_pan_back,
        test_tilt_probe_reads_the_vertical_component,
        test_tilt_probe_moves_tilt_and_never_drags_pan,
        test_tilt_probe_restores_tilt_to_the_starting_tick,
        test_tilt_probe_flips_kp_when_the_axis_is_mounted_backwards,
        test_tilt_probe_reverses_direction_at_the_upper_limit,
        test_auto_render_names_the_axis_it_controls,
        test_sign_gate_refuses_a_kp_that_matches_the_slope,
        test_sign_gate_allows_a_kp_that_opposes_the_slope,
        test_sign_gate_is_off_when_the_slope_was_never_measured,
        test_single_axis_tilt_takes_its_slope_from_dash_slope,
        test_single_axis_banner_names_the_flag_that_actually_works,
        test_both_mode_still_keeps_the_two_slopes_apart,
        test_measured_slope_catches_a_wrong_kp_in_the_real_flow,
        test_both_axes_ride_in_one_packet,
        test_both_axes_use_their_own_kp,
        test_both_axes_deadband_is_per_axis,
        test_sign_gate_is_per_axis_and_names_the_bad_one,
        test_both_axis_render_shows_both_errors,
        test_banner_warns_that_the_tilt_gate_is_wide_open,
        test_banner_shows_each_axis_gate_separately,
        test_banner_names_the_axis_that_would_be_refused,
        test_auto_render_surfaces_the_watchdog,
        # ---- ROI 择优锁定 ----
        test_csv_writes_a_row_per_frame_with_both_errors,
        test_csv_skips_a_frame_without_telemetry,
        test_roi_off_keeps_the_plain_p_behaviour,
        test_roi_probe_visits_every_candidate_and_locks_the_best,
        test_roi_picks_median_not_min,
        test_roi_locked_pose_stops_being_commanded,
        test_roi_hysteresis_does_not_reprobe_at_the_boundary,
        test_roi_releases_when_target_really_leaves,
        test_roi_single_axis_never_moves_the_other_axis,
        test_roi_single_axis_probes_only_three_poses,
        test_roi_degenerate_does_not_fake_a_score,
        test_roi_abandons_the_round_when_target_leaves_midway,
        test_roi_render_shows_the_locked_pose,
        test_banner_warns_when_roi_is_smaller_than_a_tick,
        # ---- 回差（丢失行程）标定 ----
        test_lash_probe_measures_the_lost_motion,
        test_lash_probe_reports_zero_when_there_is_no_slack,
        test_lash_probe_flags_a_slack_that_reaches_the_end_of_the_staircase,
        test_lash_probe_marks_a_healthy_reading_as_reliable,
        test_lash_probe_estimate_survives_detection_noise,
        test_lash_probe_restores_the_axis_to_the_start,
        test_lash_probe_never_touches_track_or_home,
        test_lash_probe_steps_are_one_tick_each,
        test_lash_probe_sends_nothing_without_telem,
        test_lash_probe_abort_puts_the_axis_back,
        test_lash_probe_reverses_direction_at_the_upper_limit,
        test_lash_probe_jog_stays_under_the_track_pan_max,
        test_lash_probe_resamples_when_the_target_disappears,
        test_lash_probe_run_fits_in_the_two_minute_window,
        test_lash_probe_gives_up_when_the_axis_never_moves,
        test_lash_tilt_rig_measures_the_vertical_slack,
        test_hinge_fit_keeps_the_slant_at_least_two_points_long,
        test_lash_probe_summary_and_brief_are_ascii_and_short,
        test_end_to_end_video_in_control_out,
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
