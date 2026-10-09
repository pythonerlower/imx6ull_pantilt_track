#!/usr/bin/env python3
"""手动控制台：键盘点动 + 实时遥测。

这是 3B 的人机入口，也是后面 P 控制闭环（3D）的底座——闭环要做的事，
无非是把"按键"换成"视觉算出来的目标"，其余（心跳、限位、看门狗、
状态回读）都复用这里已经跑通的部分。

三条安全约定，都是踩过或算过的：

  1. **从当前姿态起步，不跳到任何"中心值"。** 启动后先等遥测报回真实姿态，
     再把目标设成"就是现在这里"，然后才允许操作。直接下发一个固定中心值
     会造成上电即大幅运动——第一次试动时已经亲眼见过一次。
  2. **没有新鲜遥测就不许动。** 遥测超过 --max-age-ms 没更新时拒绝移动，
     只提示。盲操作正是最可能撞机械限位的情形。
  3. **客户端自己先按 safe_min/safe_max 夹一次。** 板端也会拒（那才是最终
     权威），但在这里就拦掉能立刻给操作者反馈，而不是发出去石沉大海。

用法:
  python3 main.py --host 192.168.137.50
  python3 main.py --host 192.168.137.50 --video            # 3C：带视频窗口
  python3 main.py --host 192.168.137.50 --video --no-window --duration 20 \
                  --out-dir /tmp/3c            # 无显示器时落盘当验收证据
按键:
  a / d    pan 左 / 右（tick 增 / 减）      w / s  tilt 抬 / 低
  空格      停住（MODE_STOP，保持位置）
  H        回 home
  + / -     调整每次点动的幅度
  r        重新对齐：把目标拉回当前姿态
  q        退出（会先补发 STOP）
"""
from __future__ import annotations

import argparse
import math
import sys
import termios
import time
import tty

import ptproto as P
from control import ControlClient, TelemReceiver

# 每个按键事件移动多少 tick。默认 3（60 ms 行程）：
# 足够看得出动，小到点错了也不会甩出去。
DEFAULT_STEP = 3

HELP = """按键: a/d=pan左/右  w/s=tilt抬/低  空格=停住  H=回home  r=对齐  +/-=幅度  q=退出"""


class CsvLog:
    """逐帧落盘。**为什么要它**：终端 HUD 是 `now - last_report >= 2.0` 才打一行，
    而闭环跑 16–30 fps——2 秒一行的日志在物理上分辨不出极限环的周期。3D 那三轮
    实测的记录就是这么来的：每行相隔 2 秒，反推不出"摆动是控制的还是机械的"。
    要判「锁定后为什么定不下来」，就得有逐帧的 (err, cur, 下发目标)。

    **写在 PC 侧而不是板端**：板端 daemon 只管转发姿态，闭环全在 PC 上，
    逐帧的 err 与下发目标只有 PC 有。落盘走 `--out-dir` 那套，和视频证据放一起。
    """

    FIELDS = ("t", "state", "tag", "errx", "erry",
              "pan_cur", "tilt_cur", "pan_tgt", "tilt_tgt",
              "cmd_pan", "cmd_tilt", "moves", "limit_hits", "wd", "dwd")

    def __init__(self, path: str):
        self.f = open(path, "w", buffering=1)      # 行缓冲：崩了也不丢尾巴
        self.f.write(",".join(self.FIELDS) + "\n")
        self.t0 = time.monotonic()
        self.rows = 0

    def row(self, m: "Manual", auto: "Auto") -> None:
        t, _age = m.fresh()
        if t is None:
            return
        mode, cp, ct = m.cli.current_target()
        e = auto.last_err or (float("nan"), float("nan"))
        vals = (
            f"{time.monotonic() - self.t0:.3f}", auto.state, m.tag,
            f"{e[0]:.1f}", f"{e[1]:.1f}",
            t.pan_cur, t.tilt_cur, t.pan_tgt, t.tilt_tgt,
            cp, ct, auto.moves, auto.limit_hits,
            t.watchdog_trips, t.daemon_wd_trips,
        )
        self.f.write(",".join(str(v) for v in vals) + "\n")
        self.rows += 1

    def close(self) -> None:
        try:
            self.f.close()
        except OSError:
            pass


def wd_warn(t) -> str:
    """两条看门狗（驱动 / daemon）触发过就返回告警串，否则空串。

    **两个 render 共用这一份。** 分开写过一次：`Manual.render()` 里有，
    `Auto.render()` 里漏了，而 `--auto` 下 HUD 走的是后者。后果实测撞上——
    2026-10-07 双轴跑 daemon 看门狗触发 9 次，整跑 HUD 一次都没显示，
    只在退出那一行才露出来。宪章第 7 条「失败必须可见」，别再复制一遍。
    """
    if t is None:
        return ""
    if not (t.watchdog_trips or t.daemon_wd_trips):
        return ""
    return f"  ⚠看门狗 驱动{t.watchdog_trips}/daemon{t.daemon_wd_trips}"


class Manual:
    def __init__(self, host: str, ctrl_port: int, telem_port: int,
                 rate_hz: float, step: int, max_age_ms: float):
        self.step = step
        self.max_age_ms = max_age_ms
        self.cli = ControlClient(host, ctrl_port, rate_hz=rate_hz)
        self.rx = TelemReceiver("0.0.0.0", telem_port)
        self.aligned = False
        self.note = "等待遥测以获取当前姿态…"
        # 与 note 同步的**纯 ASCII 短标签**，专供视频窗口的 HUD 用。
        # 不能直接把 note 画到画面上：cv2.putText 用的是 Hershey 字体，
        # 没有 CJK 字形，中文会变成一串方块/问号。所以中文留给终端，
        # 窗口上一律用这里的状态词。
        self.tag = "INIT"
        self.refused = 0

    # ---- 生命周期 ----
    def start(self) -> None:
        self.rx.start()
        self.cli.start()

    def close(self) -> None:
        # 先补发 STOP 再收心跳，别让板端等看门狗超时（那要 500 ms）。
        self.cli.stop(send_stop=True)
        self.rx.stop()

    # ---- 安全闸门 ----
    def fresh(self) -> tuple[P.Telem | None, float]:
        return self.rx.latest()

    def guard(self) -> P.Telem | None:
        """返回可用的遥测；不可用时置 note 并返回 None。"""
        t, age = self.fresh()
        if t is None:
            self.note = "还没有收到遥测，拒绝动作（先确认板端 daemon 在跑）"
            self.tag = "NO-TELEM"
            self.refused += 1
            return None
        if age > self.max_age_ms:
            self.note = f"遥测已过期 {age:.0f} ms（>{self.max_age_ms:.0f}），拒绝动作"
            self.tag = "STALE-TELEM"
            self.refused += 1
            return None
        return t

    def align(self, t: P.Telem) -> None:
        """把目标拉回驱动认为的当前姿态。启动和按 r 都走这里。"""
        self.cli.set_target(P.MODE_HOLD, t.pan_cur, t.tilt_cur, immediate=True)
        self.aligned = True
        self.note = f"已对齐到当前姿态 ({t.pan_cur}, {t.tilt_cur})"
        self.tag = "ALIGNED"

    # ---- 动作 ----
    def move(self, dpan: int, dtilt: int) -> None:
        t = self.guard()
        if t is None:
            return
        if not self.aligned:
            self.align(t)
            return
        # 以客户端记的目标为基准累加（而不是 cur），这样连按时不会因为
        # 云台还没爬到而把余量吃掉。
        _, pan, tilt = self.cli.current_target()
        pan += dpan
        tilt += dtilt
        cpan = min(max(pan, t.pan_safe_min), t.pan_safe_max)
        ctilt = min(max(tilt, t.tilt_safe_min), t.tilt_safe_max)
        clipped = (cpan != pan) or (ctilt != tilt)
        self.cli.set_target(P.MODE_MANUAL, cpan, ctilt, immediate=True)
        self.note = (f"越界已夹到安全范围 → ({cpan}, {ctilt})" if clipped
                     else f"目标 → ({cpan}, {ctilt})")
        self.tag = "CLAMPED" if clipped else "MOVE"

    def stop(self) -> None:
        t = self.guard()
        if t is None:
            # 即便没有遥测也要停：用客户端记的目标值，板端会忽略 tick。
            self.cli.set_target(P.MODE_STOP, immediate=True)
            self.note = "已下发 STOP（无遥测，用客户端记的目标）"
            self.tag = "STOP"
            return
        self.cli.set_target(P.MODE_STOP, t.pan_cur, t.tilt_cur, immediate=True)
        self.note = "已下发 STOP，保持当前位置"
        self.tag = "STOP"

    def home(self) -> None:
        if self.guard() is None:
            return
        self.cli.set_target(P.MODE_HOME, immediate=True)
        self.note = "回 home 中…"
        self.tag = "HOME"

    # ---- 显示 ----
    def render(self) -> str:
        t, age = self.fresh()
        if t is None:
            return f"\r{self.note:<78}"
        st = P.DRV_STATUS_NAME.get(t.drv_status, "?")
        dst = P.DAEMON_STATE_NAME.get(t.daemon_state, "?")
        warn = ""
        if age > self.max_age_ms:
            warn = "  ⚠遥测过期"
        warn += wd_warn(t)
        return (f"\r{st:<6} {dst:<6} cur=({t.pan_cur:>3},{t.tilt_cur:>3}) "
                f"tgt=({t.pan_tgt:>3},{t.tilt_tgt:>3}) "
                f"范围 {t.pan_safe_min}-{t.pan_safe_max}/{t.tilt_safe_min}-{t.tilt_safe_max} "
                f"cmd={t.cmd_count} 遥测{age:.0f}ms 步长{self.step}{warn:<28}")


# ------------------------------------------------------------------ 3D 闭环
# 从这一行往下是 3D 的：把像素误差变成 tick 目标。
#
# 控制律**不放进 tracker.py**。tracker.py 只回答"目标在哪"，驱动只回答
# "现在几 tick、能到哪"。3C 好不容易把 get_target_center 留成唯一一道缝，
# 把控制律塞进去就等于说"换检测算法要连控制律一起重测"——那道缝就白留了。
# 所以这里的输入只有两样：Target（检测结果）和 Telem（实测姿态）。

# 驱动的速度是**固定两档**：0，或者 1000/slew_ms tick/s。没有中间档。
# 这不是估算，是 pantilt 驱动里 delayed_work 每 slew_ms 走 1 tick 的直接后果。
DEFAULT_SLEW_MS = 20.0


class Auto:
    """3D P 控制闭环。与 Manual 同层，复用同一个 ControlClient 和 guard()。

    一次 ``update()`` 只做三件事：判时域迟滞、按误差算步长、限幅下发。

    参数默认值全部来自实测，不是拍的：

    * ``acquire_n=4`` —— 出画后那 3 帧误报（面积 2380/1464/694）是**连续 3 帧**，
      所以认目标要 4 帧。N=3 恰好会在最后那帧误报上认下来
      （见 plan/phase3_pantilt_tracking.md 时域迟滞一节的订正）。
    * ``release_m=5`` —— 残帧成组延迟最坏 200 ms ≈ 3 帧
      （见 logs/phase3/20261007_3c_software_acceptance.md §4.2）。M 小于它，
      就会把"一帧检测抖动"当成"目标没了"，云台停一下再起——顿挫正是这么来的。
    * ``deadband=2.0`` —— 死区要**小于 1 tick 的像素当量**（1 tick ≈ 2.3 px），
      否则它会变成"攒够 2 tick 的误差再一次性迈 2 tick"。实测（30 fps、
      目标 0.6 px/帧）单 tick 步进占比：死区 2 px **89%**，死区 5 px **13%**。
      这个 2.3 px 是**可以实测的**——``--calib-sign`` 会把它量出来，
      现场的光学（视场角、分辨率）一变它就该跟着变，别当常数用。
      但也不能取 0：``kp×误差`` 的取整本身就提供了约 1 px 的等效死区，
      再叠加检测噪声，取 0 会让静止目标出现 1 tick 的来回蹭
      （实测 kp=-1.0 / 死区 0 时静止目标是 38 次/秒的下发，那是舵机啸叫）。
    * ``max_step=4`` —— 驱动 50 tick/s，30 fps 下两帧之间爬 1.67 tick。
      上限取 4 意味着每帧派出去的行程要 80 ms 才能走完，**远长于帧间隔
      33 ms**——驱动的目标永远领先于实际位置，它一刻不停。
      想要"单步幅度最小"就取 1（实测 100% 都是单 tick），
      代价是速度上限掉到 1 tick/帧 = 30 tick/s = 12.9°/s，
      追不上比这快的目标，且差距**不会自己收回来**。

    **连续性的硬下限（写在参数里省得每轮重新推一遍）**：云台只能按
    tick 走，1 tick = 0.43°。所以目标速度 v 对应的台阶间隔就是
    ``0.43° / v``，改不掉。v ≈ 6.5°/s 时台阶间隔 66 ms，v ≈ 2°/s 时是
    215 ms。**闭环管不了这个**，它能管的是别在这之上再加停顿。
    """

    def __init__(self, m: "Manual", kp: float = -0.5, deadband: float = 2.0,
                 max_step: int = 4, acquire_n: int = 4, release_m: int = 5,
                 axis: str = "pan", pan_max: int | None = None,
                 slew_ms: float = DEFAULT_SLEW_MS, slope: float | None = None,
                 kp_tilt: float | None = None,
                 slope_tilt: float | None = None,
                 roi: float | None = None, hold_s: float = 2.0):
        self.m = m
        self.sign_tripped = False

        self.acquire_n = max(1, int(acquire_n))
        self.release_m = max(1, int(release_m))
        self.slew_ms = slew_ms
        self.axis = axis
        self.axes = ("pan", "tilt") if axis == "both" else (axis,)

        # **每条轴一份参数，不能共用。** 实测 pan 斜率 −1.83、tilt +1.83——
        # 大小一样、**符号相反**。拿同一个 kp 乘下去，一条收敛、另一条就是
        # 逐帧把误差放大 (1+kp·slope) 倍的正反馈。所以 kp / slope 必须按轴分开。
        step = max(1, int(max_step))
        self.param: dict[str, dict] = {
            "pan": {"kp": kp, "slope": slope, "deadband": deadband,
                    "max_step": step, "comp": 0, "max_tick": pan_max},
        }
        if "tilt" in self.axes:
            self.param["tilt"] = {
                "kp": kp if kp_tilt is None else kp_tilt,
                # both 下 tilt 的 slope **刻意不回落**到 pan 的：两轴符号可以
                # 相反，回落等于把一条已验证的斜率硬按到另一条轴上，而闸门
                # 恰恰就是靠这个符号来判断发散的。没量就留 None（不拦 + 警告）。
                #
                # 但**单轴**是另一回事：`--axis tilt` 只有这一条轴，`--kp` /
                # `--slope` 本来就是给它的（`--slope-tilt` 在 parameter 校验里
                # 被拒，见 main()），不接过来的话这条轴的斜率永远量不上、
                # 闸门永远是关的——与 `--help` 写的"单轴闭环请用 --kp / --slope"
                # 正好对不上。
                "slope": slope_tilt if len(self.axes) > 1 else slope,
                "deadband": deadband, "max_step": step, "comp": 1,
                "max_tick": None,
            }

        # 主轴的扁平副本：单轴的老代码与老测试仍按这些名字读。
        p0 = self.param[self.axes[0]]
        self.kp = p0["kp"]
        self.slope = p0["slope"]
        self.deadband = p0["deadband"]
        self.max_step = p0["max_step"]
        self.comp = p0["comp"]
        self.pan_max = pan_max
        self._pace_step = max(p["max_step"] for p in self.param.values())
        self._bad_axis = None

        # ---- ROI 择优锁定（--roi）----
        # 不做的话就是纯 P：一直想把 err 打到 0，而执行器的最小一格是
        # 1 tick = |slope| px，所以「打到 0」物理上不可达——环路会在
        # 分度值附近自己跟自己耗。ROI 把控制目标从「err→0」改成
        # 「err→ROI 内」，进 ROI 就换成一次**离散择优**：探几个相邻位姿、
        # 各按中位数打分、挑最好的锁住，之后一个包都不再改目标。
        #
        # **「不再下发」不等于「不再发包」**：ControlClient 的心跳线程
        # 20 Hz 一直在补发最后一次 (mode, tick)，所以驱动看门狗照样被喂，
        # 锁定期反而比连续跟踪更安全。锁定期不动的只是**目标值**。
        self.roi = roi
        self.hold_s = float(hold_s)
        self.hold_anchor = None      # 进 ROI 那一刻的 (pan, tilt)
        self.hold_pose = None        # 最终锁定的 (pan, tilt)
        self.hold_scores = None      # [(中位数 r, 位姿), ...]，择优的账
        self._probe_poses: list = []
        self._probe_i = 0
        self._probe_samples: list = []
        self._probe_scores: list = []
        self._pose_t0 = 0.0
        self._out_r = 0              # 连续在 ROI 外的帧数（退出滞环用）
        # 择优全是"这一格待了多久"，而"待了多久"在测试里不该真的等——
        # 否则一条 hold_s=2s 的用例要跑 2 秒。时钟做成可替换，同 SignProbe。
        self._now = time.monotonic

        self.state = "IDLE"        # IDLE / ACQUIRE / TRACK
        self.suspended = False
        self.found_streak = 0
        self.lost_streak = 0
        self.frames = 0            # 进过闭环的帧数
        self.moves = 0             # 真正下发过目标的次数
        self.limit_hits = 0        # 撞到安全边界 / 跟踪上限
        self.last_err = None
        self._gap_warned = False
        self._last_frame_t = None
        self._period_s = None      # 实测帧间隔（EMA）
        self._fps_hint_warned = False
        self._stall_ms = 0.0       # 派出去的行程喂不满帧间隔的累计毫秒数

    # ---- 操作者的出口 ----
    @property
    def active(self) -> bool:
        """闭环是否握着控制权。False = 已挂起，点动键此时可用。"""
        return not self.suspended

    def suspend(self, m: "Manual") -> None:
        """挂起：回 HOLD 并把控制权交给操作者。

        下发的是 HOLD 而不是 STOP。两者在驱动侧都是"停在原地、保持力矩"
        （不是断电、不是松力），但 HOLD 是"按要求停在某处"，
        STOP 是"出事了"。挂起是正常操作，别让它看起来像故障。
        """
        self.suspended = True
        self.state = "IDLE"
        self.found_streak = 0
        self.lost_streak = 0
        self._hold_clear()      # 挂起期间云台可能被手动挪走，锁定的位姿作废
        m.cli.set_target(P.MODE_HOLD, immediate=True)
        m.note = "闭环已挂起，控制权交回操作者（再按空格恢复）"
        m.tag = "SUSPEND"

    def resume(self, m: "Manual") -> None:
        """恢复：重新走一遍确认流程，不直接接着上次的误差。
        挂起期间云台可能已经被手动挪走，接着算等于拿旧误差打新姿态。"""
        self.suspended = False
        self.state = "IDLE"
        self.found_streak = 0
        self.lost_streak = 0
        self.last_err = None
        m.note = f"闭环已恢复，重新确认目标（{self.acquire_n} 帧）"
        m.tag = "RESUME"

    # ---- 一帧一次 ----
    def update(self, target, w: int, h: int) -> str:
        """喂一帧的检测结果。**没找到目标时也要喂**（found=False），
        因为"丢失 M 帧"这个决定只能由这里做。返回状态词。"""
        if self.suspended:
            return "SUSPENDED"
        self._pace_check()
        if not target.found:
            return self._lost()
        err = target.error(w, h)
        if err is None:
            # 约定上 found=True 就一定给得出误差；真出现就当没找到，
            # 不要退化成 (0,0)——那在闭环里等于"已经很准，别动"。
            return self._lost()

        self.lost_streak = 0
        # **TRACK / HOLD / HOLD_PROBE 三者都已过闸门**，不能再走一遍确认 +
        # 正反馈闸门。写成 `!= "TRACK"` 时，ROI 那两个状态每帧都会被判成
        # "还没开始跟踪"，于是被重置回 TRACK、再重新 _hold_begin——择优永远
        # 走不到第二轮（实测：状态在 TRACK/HOLD_PROBE 之间每帧翻一次）。
        if self.state not in ("TRACK", "HOLD", "HOLD_PROBE"):
            self.found_streak += 1
            if self.found_streak < self.acquire_n:
                self.state = "ACQUIRE"
                self.m.note = f"确认目标 {self.found_streak}/{self.acquire_n} 帧…"
                self.m.tag = "ACQUIRE"
                return self.state
            if not self._sign_ok():
                # 就在**将要进环、但还一个包都没发过**的这一刻拦下来：
                # 此刻拒绝的代价是零，进了环再发现就是飞车。
                self._trip_sign()
                return self.state
            self.state = "TRACK"
            self.m.note = f"已锁上，开始闭环（kp={self._kps()} 死区{self.deadband:g}px）"
            self.m.tag = "TRACK"

        self.frames += 1
        self.last_err = err
        if self.roi is None:
            self._aim(err)
        else:
            self._hold_update(err)
        return self.state

    # ---- ROI 择优锁定 ----
    ROI_RELEASE_K = 1.5     # 退出 ROI 的滞环倍数（进 1.0、出 1.5）
    POSE_SETTLE_S = 0.12    # 换位姿后先不采样：驱动要走 1 tick(20 ms)，
                            # 检测那一帧还是旧姿态的，再留一点余量。
    POSES = ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1))

    def _hold_update(self, err) -> None:
        """ROI 内换成离散择优 + 锁定；ROI 外仍是原来的连续 P 跟踪。

        这就是"保留摄像中心的功能，用于跟踪校正"：只有进到 ROI 里面，
        误差已经小到执行器分辨不出来的时候，才换成择优。
        """
        r = math.hypot(err[0], err[1])

        if self.state == "HOLD":
            if r <= self.roi * self.ROI_RELEASE_K:
                self._out_r = 0
                self.m.tag = "HOLD"
                self.m.note = (f"已锁定 pos={self.hold_pose} r={r:.0f}px"
                               f"（ROI {self.roi:g}）")
                return
            # 滞环：进 ROI 用 1.0×、出 ROI 用 1.5×。这里用迟滞是对的——
            # 它治的是**状态机在门限上翻来覆去**（每翻一次要重跑一整个
            # hold_s 的择优），不是控制律的极限环。两者不是一回事：
            # 前面离线量过，迟滞对"噪声把 err 顶出门限"那种摆动无效。
            self._out_r += 1
            if self._out_r < self.release_m:
                self.m.tag = "HOLD-OUT"
                self.m.note = (f"目标出 ROI {self._out_r}/{self.release_m} 帧"
                               f"r={r:.0f}px，先看看（要抖不抖）")
                return
            self._hold_clear()
            self.state = "TRACK"
            self.m.tag = "TRACK"
            self.m.note = f"目标出 ROI（r={r:.0f}px），恢复连续跟踪"
            return

        if self.state == "HOLD_PROBE":
            self._probe_update(err, r)
            return

        # TRACK
        if r <= self.roi:
            self._hold_begin()
            return
        self._aim(err)

    def _hold_begin(self) -> None:
        """进 ROI：算出候选位姿，开始本轮择优。"""
        t = self.m.guard()
        if t is None:
            # 没有新鲜遥测就无从谈起"从 cur 出发的位姿"，这一帧只提示。
            self.m.tag = "ROI-NOTELEM"
            self.m.note = "想择优但没有新鲜遥测，等遥测回来再说"
            return
        self.hold_anchor = (t.pan_cur, t.tilt_cur)
        self._probe_poses = self._build_poses(t)
        self.hold_scores = None
        self._out_r = 0

        if len(self._probe_poses) < 2:
            # 夹过之后只剩一个不同位姿（贴着边界 / 单轴）。**不许假装做了择优**：
            # 那种情况下 hold_scores 是空账，操作者会以为"选过了"。
            self.hold_pose = self.hold_anchor
            self.state = "HOLD"
            self.m.tag = "HOLD-FIXED"
            self.m.note = (f"ROI 内但无从择优：可用行程里只有 1 个不同位姿，"
                           f"就地锁定 {self.hold_anchor}（这不是择优结果）")
            print(f"\n[roi] 择优做不了：可用行程里只剩 1 个不同位姿，"
                  f"就地锁定在 {self.hold_anchor}。**这不是择优结果**，"
                  f"是因为探不动。")
            return

        self._probe_i = 0
        self._probe_samples = []
        self._probe_scores = []
        self.state = "HOLD_PROBE"
        self._send_pose(self._probe_poses[0])

    def probe_poses(self) -> tuple:
        """按**受控轴**过滤后的候选偏移。

        单轴闭环绝不准动另一条轴——这是 `_send_pose` 会把 (pan, tilt) 一起
        下发的直接后果：候选里带一个 dy 偏移，`--axis pan` 的择优就会顺手
        把 tilt 挪一格。过滤后单轴是 3 格（中心 + 两个方向），双轴才是 5 格。
        """
        if self.axes == ("pan", "tilt"):
            return self.POSES
        if self.axes == ("pan",):
            return ((0, 0), (-1, 0), (1, 0))
        return ((0, 0), (0, -1), (0, 1))

    def _build_poses(self, t) -> list:
        """候选位姿（锚点 + 偏移），按当前可用区间夹一遍并去重。

        位姿是**绝对** tick 对。夹过之后可能有两个偏移落到同一个 tick 上
        （贴边时），重复的必须去掉：留着重算会把它的中位数投两次票，
        把 argmin 往边界上拉。
        """
        pa, ta_ = self.hold_anchor
        _, lo_p, hi_p = self._range("pan", t)
        _, lo_t, hi_t = self._range("tilt", t)
        seen: set = set()
        out: list = []
        for dx, dy in self.probe_poses():
            pose = (max(lo_p, min(hi_p, pa + dx)),
                    max(lo_t, min(hi_t, ta_ + dy)))
            if pose in seen:
                continue
            seen.add(pose)
            out.append(pose)
        return out

    def _probe_update(self, err, r: float) -> None:
        """择优进行中：每个位姿停一段，收 r 的样本。"""
        now = self._now()
        n = len(self._probe_poses)
        dwell = self.hold_s / n
        el = now - self._pose_t0

        if r > self.roi * self.ROI_RELEASE_K:
            # 目标跑出 ROI，这一轮的样本已经不可比了（不同位姿的样本对应的
            # 目标位置不同），整轮作废、回连续跟踪，别拿它去选。
            self._hold_clear()
            self.state = "TRACK"
            self.m.tag = "TRACK"
            self.m.note = f"择优中途目标出了 ROI（r={r:.0f}px），本轮作废"
            return

        pose = self._probe_poses[self._probe_i]
        if el < self.POSE_SETTLE_S:
            self.m.tag = "ROI-PROBE"
            self.m.note = (f"择优 {self._probe_i + 1}/{n} pos={pose}："
                           f"等驱动走到位")
            return

        self._probe_samples.append(r)
        self.m.note = (f"择优 {self._probe_i + 1}/{n} pos={pose} "
                       f"r={r:.0f}px 采样 {len(self._probe_samples)}")
        self.m.tag = "ROI-PROBE"
        if el < dwell:
            return

        # 这一格到点了：记中位数（不是最小值，见 _probe_finish 的说明）
        self._probe_scores.append(
            (_median(self._probe_samples) if self._probe_samples
             else float("inf"), pose))
        self._probe_i += 1
        if self._probe_i >= n:
            self._probe_finish()
            return
        self._probe_samples = []
        self._send_pose(self._probe_poses[self._probe_i])

    def _probe_finish(self) -> None:
        """一轮探完：选**中位数最小**的位姿锁住。

        为什么不用「窗口内瞬时 |err| 最小」：err 是检测出来的含噪量。
        60 个样本里取最小值，取到的是**噪声底**而不是位姿质量——真值 2 px、
        σ=1.5 px 时，60 个样本的最小值期望落在 2 − 1.5×√(2ln60) ≈ 0，
        也就是不管位姿好坏都会读到 0 附近。按姿态聚合后取中位数才是
        「哪个位置真的更准」；中位数的标准误 σ/√n 在 n≈12 时约 0.43 px，
        小于 1 tick 的 1.83 px，分辨得开一格。
        """
        usable = [(m, p) for m, p in self._probe_scores
                  if m != float("inf")]
        if not usable:
            # 全程没采到样本（目标一直不在）。不做决定，退回连续跟踪。
            self._hold_clear()
            self.state = "TRACK"
            self.m.tag = "TRACK"
            self.m.note = "择优一轮没采到任何有效样本，已退回连续跟踪"
            return
        self.hold_scores = usable
        best = min(usable, key=lambda mp: mp[0])[1]
        self.hold_pose = best
        self.state = "HOLD"
        self._out_r = 0
        self._send_pose(best)

        txt = "  ".join(f"{p}→{m:.1f}px" for m, p in usable)
        print(f"\n[roi] 择优完成（每格 {self.hold_s / len(self._probe_poses):.2f}s"
              f"，按中位数）：{txt}  ⇒ 锁定 {best}")
        self.m.tag = "HOLD"
        self.m.note = f"择优锁定 {best}（中位数 {min(m for m, _ in usable):.1f}px）"

    def _send_pose(self, pose) -> None:
        """下发一个绝对位姿。**同时也重置该格的驻留时钟**——每换一格
        都要重新等驱动走到位，所以"什么时候开始这一格"只有这一个正确
        的产生点，散在调用方就会漏掉一处（初版就漏了 _probe_t0，
        结果是每格一进来就被判定为"已到点"）。"""
        self.m.cli.set_target(P.MODE_TRACK, pose[0], pose[1], immediate=True)
        self.moves += 1
        self._pose_t0 = self._now()

    def _hold_clear(self) -> None:
        """清掉择优/锁定的账面。**不动 self.state**——调用方自己决定去哪。"""
        self.hold_anchor = None
        self.hold_pose = None
        self.hold_scores = None
        self._probe_poses = []
        self._probe_i = 0
        self._probe_samples = []
        self._probe_scores = []
        self._out_r = 0

    # ---- 正反馈闸门 ----
    def _sign_ok(self) -> bool:
        """``kp × slope < 0`` 才收敛——这条是从递推式直接读出来的。

        下一帧的误差是 ``err' = err + slope × step = err × (1 + kp × slope)``。
        要收敛就得 ``|1 + kp×slope| < 1``，即 ``-2 < kp×slope < 0``。
        符号这一半是**过冲/离散化盖不住**的：``kp×slope > 0`` 时每一帧都把
        误差放大 ``1 + kp×slope`` 倍，第 n 帧是初始误差的 n 次方——这就是
        "上电即飞车"。另一半（``kp×slope ≤ -2``）只是震荡，不算飞车，
        所以这里不拦，留给 ``max_step`` 去夹。

        ``slope`` 为 None 表示没实测过，此时不拦（否则第一次实机根本跑不
        起来），但 ``main()`` 会在启动横幅上警告一次。

        **逐轴判**：两轴的斜率符号可以相反（实测 pan −1.83 / tilt +1.83），
        所以"pan 合法"推不出"tilt 也合法"。任何一条轴不合法就整轮拒绝——
        双轴是一起下发的，放进去的那一条就是飞车。
        """
        for ax in self.axes:
            p = self.param[ax]
            if p["slope"] is not None and p["kp"] * p["slope"] >= 0.0:
                self._bad_axis = ax
                return False
        return True

    def _flag(self, ax: str, base: str) -> str:
        """该轴对应的命令行开关名。

        ``both`` 下 tilt 用带 ``-tilt`` 后缀的那个（``--kp-tilt`` /
        ``--slope-tilt``）；**单轴时两条轴都只用不带后缀的**——``--axis tilt``
        只有这一条轴，``--kp`` / ``--slope`` 本来就是它的，带后缀的开关在
        parameter 校验里会被直接拒掉，印出来等于指错路。
        """
        if len(self.axes) > 1 and ax != "pan":
            return f"{base}-tilt"
        return base

    def _trip_sign(self) -> None:
        ax = self._bad_axis or self.axes[0]
        p = self.param[ax]
        self.suspended = True
        self.state = "IDLE"
        self.found_streak = 0
        self.sign_tripped = True
        self.m.cli.set_target(P.MODE_HOLD, immediate=True)
        self.m.note = f"正反馈闸门触发：{ax} 的 kp 方向与实测斜率同号，已拒绝进闭环"
        self.m.tag = "SIGN-BAD"
        print(f"\n[error] 拒绝进闭环：{ax} kp={p['kp']:+.2f} × 实测斜率 "
              f"{p['slope']:+.2f} px/tick = {p['kp'] * p['slope']:+.2f} ≥ 0，"
              f"误差会逐帧被放大而不是被修掉。\n"
              f"        改用 {self._flag(ax, '--kp')} {-abs(p['kp']):+.2f}，"
              f"或重跑 --calib-sign --axis {ax} 复核。"
              f"（此时一个动作都没下发过，云台是停的。）")

    # ---- 丢目标 ----
    def _lost(self) -> str:
        # HOLD / HOLD_PROBE 也要走这条：两者都是「握着控制权」的状态，
        # 目标没了却不退出，就会一直锁在一个已经没有目标的位姿上。
        if self.state in ("TRACK", "HOLD", "HOLD_PROBE"):
            self.lost_streak += 1
            if self.lost_streak <= self.release_m:
                # 保持窗口内**一个包都不发**，让驱动把手上那点余量走完自然停住。
                # 此时改发 HOLD、或把目标拉回 cur，都会在一帧抖动上凭空造出
                # 一次急停——顿挫就是从这种地方来的。
                #
                # **窗口内不清锁**：检测框偶尔掉一两帧是常态，一掉就清锁
                # 会让下一帧重新进 ROI、重跑一次 hold_s 的择优。锁着的位姿
                # 本来就是"上次量出来最好的那个"，没有理由因为一帧抖动丢掉。
                self.m.note = (f"目标丢了 {self.lost_streak}/{self.release_m} 帧，"
                               f"先溜车（让驱动跑完余量）")
                self.m.tag = "COAST"
                return self.state
            self._hold_clear()
            self.state = "IDLE"
            self.found_streak = 0
            # 真丢了：显式回 HOLD。驱动侧 HOLD 是"停在原地"（不是断电、不是松力），
            # 而且它**忽略 tick 字段**，所以这里不需要、也不准带 tick。
            self.m.cli.set_target(P.MODE_HOLD, immediate=True)
            self.m.note = f"连续丢 {self.lost_streak} 帧，退出跟踪回 HOLD"
            self.m.tag = "LOST"
            return self.state
        self.found_streak = 0
        self.state = "IDLE"
        self.m.note = "画面里没有目标"
        self.m.tag = "NO-TARGET"
        return self.state

    # ---- 算步长并下发 ----
    def _aim(self, err) -> None:
        """一帧一次：各轴各算一份，**合成一条** MODE_TRACK 发出去。

        为什么合成一包而不是每轴各发一包：协议一个包里本来就同时带 pan/tilt
        两个 tick，分两包发除了浪费一半带宽，还会让驱动在同一帧里先看到
        「新 pan + 旧 tilt」再看到「新 pan + 新 tilt」——两个中间态里有一个
        是假的，丢包时它就可能变成最终态。
        """
        # 新鲜遥测是硬前提：从一个陈旧的 cur 推目标等于闭着眼开车。
        # Manual.guard() 的那道闸门在这里一字不改地复用。
        t = self.m.guard()
        if t is None:
            return

        want: dict[str, int] = {}
        notes: list[str] = []
        tags: list[str] = []
        for ax in self.axes:
            new, note, tag = self._aim_axis(ax, err, t)
            notes.append(note)
            if new is None:
                tags.append(tag)
            else:
                want[ax] = new

        self.m.note = "；".join(notes)
        if not want:
            # 两轴都没动（都在死区 / 都贴边界）——状态词如实反映，
            # 别把它写成 TRACK，那会让操作者以为云台在跟。
            self.m.tag = tags[0] if len(set(tags)) == 1 else "MIXED"
            return

        self.m.cli.set_target(P.MODE_TRACK, want.get("pan"), want.get("tilt"),
                              immediate=True)
        self.moves += 1
        self.m.tag = "TRACK"

    def _aim_axis(self, ax: str, err, t):
        """算一条轴的这一步，返回 ``(新 tick 或 None, note, tag)``。

        返回 None 表示这一步**不发**（死区 / 不足 1 tick / 已贴边界）。
        """
        p = self.param[ax]
        e = err[p["comp"]]

        if abs(e) < p["deadband"]:
            # 死区内**不发**。发一个和当前目标一样、或比它更靠后的值，
            # 会把手上还没走完的余量抽掉，把一次平滑收尾切成"走一下停一下"。
            return None, f"{ax} 死区内 err={e:+.0f}px，保持", "DEADBAND"

        step = int(round(p["kp"] * e))
        lim = p["max_step"]
        step = max(-lim, min(lim, step))
        if step == 0:
            # 增益换算下来不足 1 tick。驱动的分辨力就是 1 tick，
            # 下发 0 或小数不会让它走得更细，只会把余量抽掉。
            return None, f"{ax} err={e:+.0f}px 不足 1 tick，保持", "SUBTICK"

        # 从**实测 cur**重新瞄准，绝不累加。驱动是"往 tgt 爬"，不是"走 dpan"；
        # 累加会把还在路上的余量重复计一次，表现为越跟越超前，最后过冲振荡。
        cur, lo, hi = self._range(ax, t)
        new = max(lo, min(hi, cur + step))
        if new == cur:
            # 已经贴在边界上，再朝这个方向要也没用。不发，免得每帧空转一条指令。
            self.limit_hits += 1
            return None, f"{ax} 已到边界 [{lo},{hi}]，不再朝 {step:+d} 方向走", "LIMIT"

        return new, (f"{ax} err={e:+.0f}px {cur}→{new} "
                     f"({step:+d} tick = {abs(step) * 0.43:.1f}°)"), "TRACK"

    def _range(self, ax: str, t):
        """该轴这一帧的可用区间 ``(cur, lo, hi)``，全部取自遥测。"""
        if ax == "pan":
            hi = t.pan_safe_max
            mt = self.param["pan"]["max_tick"]
            if mt is not None:
                hi = min(hi, mt)         # 跟踪专用收紧，不动 config/ 和驱动
            return t.pan_cur, t.pan_safe_min, hi
        return t.tilt_cur, t.tilt_safe_min, t.tilt_safe_max

    # ---- 连续性自检 ----
    def _pace_check(self) -> None:
        """量一量「每帧派出去的行程够不够喂满帧间隔」。

        驱动的速度是固定的，只有「目标领先于实际位置」它才不停。领先量 =
        每帧的 tick 数 × slew_ms，必须 ≥ 帧间隔，否则每帧末尾驱动都会跑空。
        这条不成立时云台就是「走一下停一下」，而现象上很难归因，
        所以在这里量出来、印一次，别让它变成玄学。
        """
        now = time.monotonic()
        if self._last_frame_t is not None:
            dt = now - self._last_frame_t
            # 帧间隔用实测的，EMA 抹掉调度抖动。异常大的 dt（卡了一下）
            # 不进 EMA，否则一次 GC 就能让这个警告误报。
            if 0.0 < dt < 0.5:
                self._period_s = (dt if self._period_s is None
                                  else 0.7 * self._period_s + 0.3 * dt)
        self._last_frame_t = now
        if self._period_s is None:
            return

        # 双轴时两路是**并行**爬的（同一个 20 ms 步进节拍推两个通道），
        # 所以预算看的是步幅最大的那条轴，不是两条相加。
        lead_ms = self._pace_step * self.slew_ms
        period_ms = self._period_s * 1e3
        if lead_ms < period_ms:
            self._stall_ms += period_ms - lead_ms
            if not self._gap_warned:
                self._gap_warned = True
                need = int(period_ms / self.slew_ms) + 1
                print(f"\n[warn] max_step={self._pace_step} × slew_ms="
                      f"{self.slew_ms:.0f} = {lead_ms:.0f} ms < 帧间隔 "
                      f"{period_ms:.0f} ms：每帧末尾驱动会跑空 "
                      f"{period_ms - lead_ms:.0f} ms，看上去是「走一下停一下」。"
                      f"把 --max-step 提到 {need}，或降 PC 侧帧率。")

        # 帧率是连续性最大的那个杠杆，而且不用改任何板端参数就能动：
        # 台阶下发的最小间隔就是检测周期。离线仿真里同一物理运动
        # 最差台阶间隔从 15 fps 的 140 ms 降到 30 fps 的 100 ms。
        # 传感器本来就是 30 fps（`--fps` 那轮实测的），所以提示里给的是
        # "把板端跑到 30"，不是"改代码"。
        if (self._fps_hint_warned is False and period_ms > 45.0):
            self._fps_hint_warned = True
            print(f"\n[hint] 实测帧间隔 {period_ms:.0f} ms（约 "
                  f"{1e3 / period_ms:.0f} fps）。3D 闭环下这是运动连续性的"
                  f"第一瓶颈——目标慢时云台只能每隔 {period_ms:.0f} ms 才"
                  f"拿到一次误差。传感器原生 30 fps，"
                  f"板端用 `run_video_link.sh --fps 30` 就能翻倍，"
                  f"带宽约 38 Mbit/s。")

    @staticmethod
    def _en(ax: str) -> str:
        return "errx" if ax == "pan" else "erry"

    def _kps(self) -> str:
        """按轴列 kp。双轴的符号是相反的，只印一个就是误导。"""
        return "/".join(f"{self.param[ax]['kp']:+.2f}" for ax in self.axes)

    def render(self) -> str:
        # 轴无关：控 tilt 时还印 errx 就是在撒谎（那一路根本没参与控制）。
        # 双轴则两条都印——只印一条等于把另一条藏起来。
        es = " ".join(
            f"{self._en(ax)}="
            + ("----" if self.last_err is None
               else f"{self.last_err[self.param[ax]['comp']]:+.0f}")
            for ax in self.axes)
        kps = self._kps()
        lock = (f"{self.found_streak}/{self.acquire_n}"
                if self.state == "ACQUIRE" else "--")
        # ROI 开着才印 ROI 那一段。**选中的位姿必须印出来**：择优是本轮
        # 唯一"有决定权"的动作，只印 state 的话操作者看不到它锁在哪、
        # 也看不到那一轮各格的账（宪章第 7 条：失败/决定必须可见）。
        roi_txt = ""
        if self.roi is not None:
            r = (math.hypot(*self.last_err) if self.last_err is not None
                 else float("nan"))
            roi_txt = f" ROI{self.roi:g} r={r:.0f}"
            if self.state == "HOLD_PROBE":
                roi_txt += f" 探{self._probe_i + 1}/{len(self._probe_poses)}"
            elif self.state == "HOLD":
                roi_txt += f" 锁{self.hold_pose}"
                if self.hold_scores:
                    roi_txt += (f" 账{min(m for m, _ in self.hold_scores):.1f}px")
        # 看门狗告警这一份由 wd_warn() 统一给，见该函数注释：分开写过一次就漏了。
        # **这里不带 \r**：render() 是被拼进 run_video 那条报告行中间的，
        # 自带 \r 会把光标弹回行首，后面半行把前半行覆盖掉——写进文件看不出
        # 来（文件里 \r 只是个字节），但**屏幕上那半行是真的没了**，而实机
        # 调试恰恰是看屏幕。\r 由调用方（print 那一处）给一次就够。
        t, _age = self.m.fresh()
        return (f"{self.state:<8} {self.m.tag:<9} {es:<16} "
                f"kp={kps} 步<={self._pace_step} 死区{self.deadband:g} "
                f"锁定{lock} 下发{self.moves} 限位{self.limit_hits} "
                f"闭环{self.frames}帧{roi_txt}{wd_warn(t)}")


def _median(xs):
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class Patrol:
    """Phase 7：振动事件触发的**有界**巡检。

    输入是遥测 reserved[1] 高位的事件计数（板端 securityd 的软件阈值事件，
    见 common/proto/security_event.h）。计数一跳就是一次新事件——用计数而不是
    "锁存标志"，是因为遥测可能丢包，而**计数不会因为丢一拍就消失**。

    **控制权是独占的。** active 期间 Auto.update() 一帧都不调用；只有"连续
    CONFIRM_N 帧见到目标"才把控制权交还给 Auto（它自己会重走一遍确认流程）。
    这就是 LashProbe 的同一条理由：两个写者往同一条轴上发目标，混着发等于谁都没发。

    两条底线（plan/phase7_wom_patrol_debug.md §3）：
      1. 扫描行程恒在 [lo, hi] 内，且 PAN_HARD_MAX = 514 < 515。515 是机械极限
         （2.51 ms/tick 超出 MG90S 规格），可以路过、**不许停在那儿**；
      2. 有超时兜底 --patrol-max-s(默认 20 s)：到点回 home 并如实报告 TIMEOUT，
         不假装"巡检完成"。停在那儿不动是最坏的结局——既不在扫描，也不在守位。

    没找到目标就再来一轮，而不是原地停住：停在某个角度等于"这台设备死了"。
    """

    # 515 是 pan 的机械上限（Hardware.md）。停在 515 会让舵机长时间吃超规格脉宽，
    # 所以硬上限取 514。下限同理留一格。
    PAN_HARD_MAX = 514
    PAN_HARD_MIN = 240
    ARRIVE_TOL = 3          # tick。驱动一格一格爬，差 3 tick 以内当作到位
    DWELL_S = 0.25          # 到位后再等一会儿：姿态没稳就检测，等于拿旧画面判目标
    CONFIRM_N = 3           # 连续几帧见到目标才切 TRACK。一帧噪声不算数

    def __init__(self, m: "Manual", max_s: float, lo: int, hi: int,
                 step: int, tilt: int | None = None):
        self.m = m
        self.max_s = max_s
        self.step = max(1, int(step))
        self.tilt_fixed = tilt
        # 夹两遍：先夹进机械硬界，再夹进**驱动自报的**安全区间。
        # 后者是板端 SET_LIMIT 之后的真实约束，比任何写死的数都可信。
        self.lo = max(self.PAN_HARD_MIN, int(lo))
        self.hi = min(self.PAN_HARD_MAX, int(hi))
        if self.lo >= self.hi:
            self.lo, self.hi = self.PAN_HARD_MIN, self.PAN_HARD_MAX

        self.state = "IDLE"
        self.ev_seen = None      # 上一次看到的事件计数（None = 还没同步过）
        self.runs = 0
        self.t0 = 0.0
        self.dwell_t0 = 0.0
        self.dir = 1
        self.cmd_pan = None
        self.confirm = 0
        # 这四个字段让 Patrol 能被 CsvWriter.row() 直接当 Auto 用（鸭子类型）。
        # 少了它们，巡检那一段的 CSV 要么断档、要么把 Auto 的旧 state 写进去——
        # 而 CSV 正是验收证据，写进去的每个数都得是这一段真实发生的。
        self.last_err = None
        self.moves = 0
        self.limit_hits = 0
        self.note = "待命：等振动事件"
        self.tag = "PATROL-IDLE"
        self.result = None       # "FOUND" / "TIMEOUT"
        self._now = time.monotonic

    # ---- 状态 ----
    @property
    def active(self) -> bool:
        """巡检是否握着控制权。FOUND/TIMEOUT 之后交还。"""
        return self.state in ("SWEEP", "DWELL")

    def trigger(self, ev_count: int | None, now: float | None = None) -> bool:
        """每个循环圈调一次（不是每帧）。计数上升沿 = 新事件。

        第一圈只**记录**当前计数不触发：刚起来的进程不该因为板端历史计数
        而凭空巡检一次（那个事件可能发生在半小时前）。
        """
        if now is None:
            now = self._now()
        if ev_count is None:
            return False
        if self.ev_seen is None:
            self.ev_seen = ev_count
            return False
        if ev_count == self.ev_seen:
            return False
        self.ev_seen = ev_count
        if self.active:
            # 巡检途中又来一次振动：不重开一轮，只把预算续到"从现在起 max_s"。
            # 重开会把已经在扫的那一段白白重扫，而且会把超时推得没边。
            self.t0 = now
            self.note = "巡检途中又收到振动事件，超时预算顺延"
            return False
        return self.start(now)

    def start(self, now: float | None = None) -> bool:
        """真的开始扫了才返回 True。拒绝（遥测不可用 / 区间太窄）返回 False，
        让调用方能如实打印"收到了事件但没动"，而不是报一次没发生的巡检。"""
        if now is None:
            now = self._now()
        t = self.m.guard()
        if t is None:
            # 遥测不可用就一次都不动。守则和闭环一致：没有可信的 cur 就不发目标。
            self.note = "收到振动事件，但遥测不可用，拒绝启动巡检"
            self.tag = "PATROL-NO-TELEM"
            return False
        lo = max(self.lo, int(t.pan_safe_min))
        hi = min(self.hi, int(t.pan_safe_max), self.PAN_HARD_MAX)
        if hi - lo < self.step * 2:
            self.note = (f"安全区间 {t.pan_safe_min}..{t.pan_safe_max} 太窄，"
                         f"放不下一步 {self.step} tick，拒绝巡检")
            self.tag = "PATROL-NARROW"
            return False

        self.runs += 1
        self.t0 = now
        self.dir = 1
        self.cmd_pan = max(lo, min(hi, int(t.pan_home)))
        self.lo, self.hi = lo, hi
        # tilt 全程不动：巡检是"横扫"，抬一个俯仰角只会让目标在竖直方向也跑掉，
        # 而"找得到"这件事在单轴扫描下本来就已经够难。要别的角度就传 --patrol-tilt。
        self.tilt_fixed = (int(t.tilt_home) if self.tilt_fixed is None
                           else int(self.tilt_fixed))
        self.confirm = 0
        self.result = None
        self.state = "SWEEP"
        self.tag = "PATROL"
        self.note = (f"第 {self.runs} 次巡检：pan {self.lo}..{self.hi}，一步 "
                     f"{self.step} tick，tilt={self.tilt_fixed} 不动，"
                     f"预算 {self.max_s:.0f} s")
        self._send(self.cmd_pan)   # 先站到 home，再往一端扫
        return True

    def _send(self, pan: int) -> None:
        pan = max(self.lo, min(self.hi, int(pan)))
        self.cmd_pan = pan
        # 用 MODE_MANUAL 而不是 MODE_TRACK：这是**按剧本走位**，不是闭环跟踪。
        # 两者在 3B 驱动下行为一致，但遥测里的 daemon_state 标签会如实区分，
        # 事后对账时"这一段到底在跟还是在扫"不用猜。
        self.m.cli.set_target(P.MODE_MANUAL, pan, self.tilt_fixed,
                              immediate=True)
        self.moves += 1

    # ---- 每个循环圈一次（**不依赖帧**） ----
    def tick(self, now: float | None = None) -> None:
        """只查超时。

        存在的理由是 update() 只在新帧上跑，而**视频链路断了就没有新帧**。
        把超时判在 update() 里等于「摄像头一挂，巡检就永远不到点」——
        云台会停在扫描端点上不动，而且看起来像"正在跑"。这正好是底线 2
        要防的结局，所以超时必须在帧之外也能到。
        """
        if now is None:
            now = self._now()
        if self.active and now - self.t0 > self.max_s:
            self._timeout()

    # ---- 一帧一次 ----
    def update(self, target, w: int, h: int,
               now: float | None = None) -> str:
        if now is None:
            now = self._now()
        if not self.active:
            return self.state

        if now - self.t0 > self.max_s:
            return self._timeout()

        if target.found:
            self.confirm += 1
            if self.confirm >= self.CONFIRM_N:
                # 控制权交给 Auto：它下一帧起自己走 ACQUIRE→TRACK，
                # 这里只负责**松手**，不代它发任何目标。
                self.state = "FOUND"
                self.result = "FOUND"
                self.tag = "PATROL-FOUND"
                self.note = (f"巡检第 {self.runs} 次找到目标（连续 "
                             f"{self.confirm} 帧），交回闭环")
                return self.state
            self.tag = "PATROL-ACQ"
            self.note = f"巡检中确认目标 {self.confirm}/{self.CONFIRM_N} 帧…"
            return self.state
        self.confirm = 0

        t = self.m.guard()
        if t is None:
            return self.state

        if self.state == "SWEEP":
            if abs(int(t.pan_cur) - self.cmd_pan) <= self.ARRIVE_TOL:
                self.state = "DWELL"
                self.dwell_t0 = now
                self.tag = "PATROL-DWELL"
                self.note = f"到位 pan={t.pan_cur}，停 {self.DWELL_S:g}s 看一眼"
            else:
                self.tag = "PATROL"
                self.note = (f"扫描中 cur={t.pan_cur}→{self.cmd_pan} "
                             f"（{self.lo}..{self.hi}） 剩 {self.max_s - (now - self.t0):.1f}s")
            return self.state

        # DWELL：停够了就换下一个端点，扫到头就反向
        if now - self.dwell_t0 < self.DWELL_S:
            return self.state
        if self.dir > 0 and self.cmd_pan >= self.hi - self.ARRIVE_TOL:
            self.dir = -1
        elif self.dir < 0 and self.cmd_pan <= self.lo + self.ARRIVE_TOL:
            self.dir = 1
        nxt = self.cmd_pan + self.dir * self.step
        if nxt > self.hi or nxt < self.lo:
            nxt = self.hi if self.dir > 0 else self.lo
        self.state = "SWEEP"
        self._send(nxt)
        return self.state

    def _timeout(self) -> str:
        self.state = "TIMEOUT"
        self.result = "TIMEOUT"
        self.tag = "PATROL-TIMEOUT"
        # 回 home。巡检到点还留在扫描端点上是**最坏**的结局：既没在找目标，
        # 也不在守位，而且可能正好停在行程边缘。home 是唯一有理由待着的位置。
        self.m.cli.set_target(P.MODE_HOME, immediate=True)
        self.note = f"巡检超时 {self.max_s:.0f}s 未找到目标，已回 home"
        return self.state

    def abort(self) -> None:
        """操作者打断（挂起 / 退出）：停在一个明确的位置，不留半路状态。"""
        if self.active:
            self.state = "ABORT"
            self.result = "ABORT"
            self.tag = "PATROL-ABORT"
            self.note = "巡检被打断"
            self.m.cli.set_target(P.MODE_HOLD, immediate=True)

    def render(self) -> str:
        return (f"patrol {self.state} run={self.runs} "
                f"range={self.lo}..{self.hi} cmd={self.cmd_pan} "
                f"ev={self.ev_seen}")


class SignProbe:
    """实测「tick↑ 之后 err_x 往哪走」，把符号从肉眼判断变成一次测量。

    **为什么非测不可。** err_x = cx − W/2（目标在画面中心右边为正），
    闭环算的是 ``step = kp × err_x``。这么乘要能收敛，必须成立：

        tick↑  ⟹  相机左转  ⟹  画面内容右移  ⟹  cx 变大  ⟹  err_x 变大

    这条是**装出来的物理性质，不是代码性质**：相机装反、镜像、
    甚至 pan 舵机线序反了，都会把它翻过来。翻过来之后 step 与 err 同号，
    回路成了正反馈——偏得越多、打角越大。离线仿真证明不了它：
    仿真里相机的符号是按物理写的，与 kp 的推导出自同一套假设，
    两者一致只说明**自洽**，不说明相机真那么装。

    **做法。** 记 6 帧 err_x 的中位数 → 朝一个方向走 JOG tick →
    等驱动走完（JOG × slew_ms 再留余量）→ 再记 6 帧 → 差值除以 JOG
    就是 px/tick 的**带符号斜率**。符号即所求（斜率 > 0 ⟹ kp 必须为负）。

    **顺带量出来的 px/tick 还有第二个用处**：死区必须小于它（见 Auto
    的参数说明），否则运动会被攒成 2 tick 一迈。所以这一趟同时验证了
    ``--deadband`` 的默认值在真机上到底成不成立。

    **只动 pan，且只用 MODE_MANUAL。** 不碰 MODE_TRACK（闭环那套），
    也不碰 MODE_HOME（home 要动 tilt，这一步只该动一个轴）。
    结束——含中途失败、含操作者按 q 打断——一律把 pan 放回起点。
    """

    JOG_TICKS = 12          # 12 tick ≈ 5.2°；按 2.3 px/tick 约 28 px
    MIN_DELTA_PX = 3.0      # 位移小于这个数就判不出，宁可说"失败"也别猜
    SAMPLES = 6             # 取中位数的帧数（30 fps 下 200 ms）

    def __init__(self, m: "Manual", axis: str = "pan",
                 pan_max: int | None = None,
                 slew_ms: float = DEFAULT_SLEW_MS, kp_hint: float = -0.5,
                 jog: int | None = None):
        self.m = m
        # **量哪个轴就控哪个轴**：tilt 的符号不能从 pan 搬过来——两轴装配
        # 不同、行程也不同，搬过去就是把正反馈当成已验证的斜率。所以这一层
        # 必须参数化；否则 `--calib-sign --axis tilt` 会**静默**量出 pan 的
        # 斜率，再拿去跑 tilt 闭环（宪章第 7 条：失败必须可见）。
        self.axis = axis
        self.comp = 0 if axis == "pan" else 1          # 取 err 的哪一路
        self.err_name = "err_x" if axis == "pan" else "err_y"
        self.pan_max = pan_max
        self.jog = int(jog if jog is not None else self.JOG_TICKS)
        self.kp_hint = kp_hint
        # 走完 JOG tick 要 JOG × slew_ms，再留 400 ms 给检测延迟和机械整定。
        self.settle_s = (self.jog * slew_ms + 400.0) / 1e3

        self.phase = "BASE"     # BASE/MOVE/POST/BACK/DONE/FAIL
        self.samples: list = []
        self.base = None        # 起点 err 中位数
        self.ax0 = None         # 起点 tick（最后要放回来）
        self.dir = 0            # +1 / -1，实际走的方向
        self.t_phase = None
        self.delta = None       # 位移，px
        self.slope = None       # px per tick-UP
        self.kp_rec = None      # 推荐 kp
        self.ok = False
        self.done = False
        # 时钟做成可替换的：状态机全是"过了多久"，而"过了多久"在测试里
        # 不该真的等——否则一个 12 秒的标定要跑 12 秒。
        self._now = time.monotonic

    # ---- 一帧一次 ----
    def update(self, target, w: int, h: int, now: float | None = None) -> str:
        if now is None:
            now = self._now()
        if self.done:
            return self.phase

        if self.phase in ("BASE", "POST"):
            return self._sample(target, w, h)

        # MOVE / BACK：这期间不采样，只等驱动把行程走完。
        if now - self.t_phase < self.settle_s:
            self.m.tag = f"CALIB-{self.phase}"
            return self.phase
        if self.phase == "MOVE":
            self.phase = "POST"
            self.samples = []
            self.m.note = "标定：动作走完，重新采样"
            return self.phase

        self.phase = "DONE" if self.ok else "FAIL"
        self.done = True
        self._announce()
        return self.phase

    def _sample(self, target, w: int, h: int) -> str:
        err = target.error(w, h) if target.found else None
        if err is None:
            # 采样期间丢目标就**清空重来**：半截样本混着"目标不在"，
            # 算出来的中位数没有意义，而失败的方向恰好是"符号判反"。
            self.samples = []
            self.m.note = "标定：目标不在画面里，等它回来（别用手挡住镜头）"
            self.m.tag = "CALIB-WAIT"
            return self.phase
        self.samples.append(err[self.comp])
        self.m.tag = f"CALIB-{self.phase}{len(self.samples)}"
        self.m.note = f"标定 {self.phase}：采样 {len(self.samples)}/{self.SAMPLES}"
        if len(self.samples) < self.SAMPLES:
            return self.phase
        med = _median(self.samples)
        return self._start_move(med) if self.phase == "BASE" else self._finish(med)

    # ---- 走一步 ----
    def _start_move(self, base: float) -> str:
        t = self.m.guard()
        if t is None:
            # guard() 失败时**一个包都没发过**，所以没什么要归还的。
            self.phase = "FAIL"
            self.done = True
            self.m.note = "标定中止：没有新鲜遥测（没有下发过任何动作）"
            self.m.tag = "CALIB-NOTELEM"
            return self.phase

        if self.axis == "pan":
            self.ax0, lo, hi = t.pan_cur, t.pan_safe_min, t.pan_safe_max
            if self.pan_max is not None:
                hi = min(hi, self.pan_max)
        else:
            # tilt 没有 --track-* 那层收紧：safe 162..342 对应 0.79–1.67 ms，
            # 本来就整段落在 MG90S 规格内，不需要再夹一道。
            self.ax0, lo, hi = t.tilt_cur, t.tilt_safe_min, t.tilt_safe_max
        # 优先往 tick 增大的方向走（那是 kp<0 的推导方向）。起点已经贴着
        # 上限时改往反方向走，最后用 dir 把符号翻回来——**方向是我们选的，
        # 所以它必须参与减法**，否则贴上限的那次标定会给出相反的结论。
        if self.ax0 + self.jog <= hi:
            self.dir, to = +1, self.ax0 + self.jog
        elif self.ax0 - self.jog >= lo:
            self.dir, to = -1, self.ax0 - self.jog
        else:
            self.phase = "FAIL"
            self.done = True
            self.m.note = (f"标定做不了：{self.axis} 可用行程不足 {self.jog} tick"
                           f"（cur={self.ax0} 范围[{lo},{hi}]）")
            self.m.tag = "CALIB-NOROOM"
            return self.phase

        self.base = base
        self._send_axis(to, t)
        self.t_phase = self._now()
        self.phase = "MOVE"
        self.m.tag = "CALIB-MOVE"
        self.m.note = (f"标定：{self.axis} {self.ax0} → {to}"
                       f"（{self.dir * self.jog:+d} tick），等驱动走完…")
        return self.phase

    def _finish(self, post: float) -> str:
        self.delta = post - self.base
        # 斜率按"每 tick 增大"归一：dir=-1 时位移要取反才是 tick↑ 的效果。
        self.slope = self.delta / (self.dir * self.jog)
        self.ok = abs(self.delta) >= self.MIN_DELTA_PX
        if self.ok:
            self.kp_rec = -abs(self.kp_hint) if self.slope > 0 else abs(self.kp_hint)
        self._return()
        return self.phase

    def _send_axis(self, tick: int, t) -> None:
        """把 ``tick`` 下到受控轴，另一轴按遥测原样带上。

        MODE_MANUAL 两个 tick 是一起带的，漏掉一个就会把**另一个轴**拖到
        0——所以这里必须显式补上未受控那一路，不能只发自己那一半。
        """
        if self.axis == "pan":
            self.m.cli.set_target(P.MODE_MANUAL, tick, t.tilt_cur,
                                  immediate=True)
        else:
            self.m.cli.set_target(P.MODE_MANUAL, t.pan_cur, tick,
                                  immediate=True)

    def _return(self) -> None:
        """归还：把受控轴放回起点。失败路径也要走这里。"""
        t = self.m.guard()
        if t is not None and self.ax0 is not None:
            self._send_axis(self.ax0, t)
        self.phase = "BACK"
        self.t_phase = self._now()

    def abort(self) -> None:
        """操作者按 q：立刻归还 pan（不等采样，也不走状态机）。"""
        if self.done:
            return
        try:
            self._return()
        except Exception:      # 收尾路径不许再抛，否则 q 就退不出去了
            pass
        self.done = True
        self.m.note = f"标定被中断，{self.axis} 已放回起点"
        self.m.tag = "CALIB-ABORT"

    def _announce(self) -> None:
        d = self.delta
        if not self.ok:
            print(f"\n[calib] 判不出：{self.dir * self.jog:+d} tick 只让 "
                  f"{self.err_name} 动了 {d:+.1f} px（需要 ≥{self.MIN_DELTA_PX:.0f}）。"
                  f"目标当时在动吗？红目标还认得出吗？重跑一次。")
            self.m.note = "标定失败：位移太小，判不出符号"
            self.m.tag = "CALIB-BAD"
            return
        print(f"\n[calib] {self.dir * self.jog:+d} tick → {self.err_name} {d:+.1f} px，"
              f"即 {self.slope:+.2f} px/tick（起点 {self.axis}={self.ax0}，已放回）")
        if self.slope > 0:
            print(f"[calib] 结论：tick↑ 让 {self.err_name} 变大，与默认 kp<0 的"
                  f"推导**一致**。")
        else:
            print(f"[calib] 结论：tick↑ 让 {self.err_name} **变小**，与默认 kp<0 的"
                  f"推导**相反**。必须用 kp>0，否则闭环是正反馈。")
        print(f"[calib] 死区必须小于 {abs(self.slope):.2f} px（默认 --deadband 2.0）")
        print(f"[calib] 闭环请这样跑（--slope 会让它自己拒绝反向的 kp）：\n"
              f"        --auto --axis {self.axis} --kp {self.kp_rec:+.2f} "
              f"--slope {self.slope:+.2f}")
        self.m.note = (f"符号实测完成：{self.slope:+.2f} px/tick，"
                       f"kp 应取 {self.kp_rec:+.2f}")
        self.m.tag = "CALIB-OK"

    def render(self) -> str:
        s = "----" if self.slope is None else f"{self.slope:+.2f}"
        d = "----" if self.delta is None else f"{self.delta:+.1f}"
        kp = "----" if self.kp_rec is None else f"{self.kp_rec:+.2f}"
        # 同 Auto.render()：不带 \r，由调用方给一次。
        return (f"{self.phase:<6} {self.m.tag:<12} "
                f"样本{len(self.samples)}/{self.SAMPLES} "
                f"Δ={d}px 斜率{s} px/tick 推荐 kp={kp}")

    def brief(self) -> str:
        """窗口上那一行的 ASCII 摘要（见 Manual.tag 的说明）。"""
        return f"{self.m.tag} jog={self.dir * self.jog:+d}tick"

    def summary(self) -> str:
        """收尾汇总那一行。两个 probe 共用同一个接口，收尾就只用写一处。"""
        s = "未测出" if self.slope is None else f"{self.slope:+.2f} px/tick"
        return f"符号标定：{self.phase}（{s}）"



class LashProbe:
    """量机械「丢失行程」H：换向后空走多少 tick，画面才开始动。

    **为什么非量不可。** ``--calib-sign`` 朝**同一个方向**走 12 tick，再把总
    位移除以 12，在 pan 上量出 1.83 px/tick；几何估算却是 2.3（见 Auto 的
    注释）——少掉的那 20% 正是回差：换向后 H tick 是空走的，画面真正走了的
    只有 12−H tick。更麻烦的是它对闭环的后果：丢失行程**不是死区的一部分，
    是死区之外的另一层**——输出的 1 tick 变化先被它整段吃掉，画面纹丝不动，
    控制器于是照喊，攒够 H 才挪一格，然后反向再来一遍。锁定后「一直摆、
    定不下来」就是它。

    **做法（反向 1-tick 台阶）。** 先朝一个方向走 ``LOAD_TICKS`` 把该侧回差
    吃干净 → 记基线 err 中位数 → **反向**每次只退 1 tick，每步等驱动走完 +
    取 ``SAMPLES`` 帧中位数，得 d_i。换向后 d_i = k×(i−H)（k 是扣掉回差的光学
    斜率），所以对「开始动之后」的点做最小二乘，**直线在 i 轴上的截距就是 H**，
    斜率就是被回差吃掉的那个真值 k。

    **为什么不数步子。** H 几乎不会是整数 tick（回差与舵机静摩擦叠在一起，
    2.4 这种值很常见），数步子只能给到 ⌈H⌉，还把 k 一起丢了。回归既给 H 又给 k，
    而且能和 ``--calib-sign`` 的读数对上：1.83 × 12/(12−2.4) ≈ 2.3。对不上
    就说明这一串读数不自洽，得先查为什么，别急着用。

    **量到的是回差，还是回差＋静摩擦？** 一起量到，也**只能**一起量——静止时
    两者表现完全一样（下了命令、输出不动），没有任何传感器分得开。好在闭环
    要知道的正是这个合计空走量，所以不亏。

    只动一条轴，只用 MODE_MANUAL（不碰 MODE_TRACK 那套闭环，也不碰
    MODE_HOME——home 要动 tilt，而这一步只该动一个轴）。结束——含失败、
    含超时、含按 q——一律把该轴放回起点。
    """

    LOAD_TICKS = 12         # 换向前先空走这么多，把该侧回差吃干净
    MAX_BACK = 8            # 反向最多退这么多步（约 6 s：8×(0.42 等 + 0.27 采)）
    MIN_DELTA_PX = 1.0      # 一格 ≈2.3 px，门限取不到一格才分得出"第一步"
    SAMPLES = 8             # 每步取中位数的帧数（30 fps 下 ~270 ms）
    MIN_FIT_POINTS = 2      # 折线斜段至少要有这么多点，否则 k 是硬凑的
    RELIABLE_SLANT = 3      # 斜段短于这么多步，H 就是"顶到量程尽头"的数
    MIN_SLOPE_PX = 0.5      # 估出的 |k| 小于它就不认，宁可退回数步子

    def __init__(self, m: "Manual", axis: str = "pan",
                 pan_max: int | None = None,
                 slew_ms: float = DEFAULT_SLEW_MS,
                 load: int | None = None, max_back: int | None = None):
        self.m = m
        # 和 SignProbe 一样必须参数化：两条轴的装配、行程、回差各是一个数，
        # 把 pan 的回差当成 tilt 的就是凭空造一个出来（宪章第 7 条）。
        self.axis = axis
        self.comp = 0 if axis == "pan" else 1          # 取 err 的哪一路
        self.err_name = "err_x" if axis == "pan" else "err_y"
        self.pan_max = pan_max
        self.load = int(load if load is not None else self.LOAD_TICKS)
        self.max_back = int(max_back if max_back is not None else self.MAX_BACK)
        # 12 tick 要 12×slew_ms；单步只要 1×slew_ms。但检测延迟那 400 ms
        # 一步也省不掉——画面要等下一帧才反映出来。
        self.settle_load_s = (self.load * slew_ms + 400.0) / 1e3
        self.settle_step_s = (slew_ms + 400.0) / 1e3

        self.phase = "START"    # START/LOAD/BASE/STEP_MOVE/STEP/BACK/DONE/FAIL
        self.samples: list = []
        self.base = None        # 基线 err 中位数
        self.ax0 = None         # 起点 tick（最后要放回来）
        self.dir = 0            # 先走的方向（+1/-1）；它决定 k 的符号
        self.step_i = 0
        self.steps: list = []   # [(i, d_i)]，d_i = 第 i 步中位数 − 基线
        self.first_move = None  # 第一个超过门限的步号
        self.lost = None        # H，tick（拟合出的下界；见 fitted/lost_hi）
        self.lost_hi = None     # 数步子时 H 的上界（拟合不成才有）
        self.k = None           # 光学斜率 px/tick（拟合斜率，带符号）
        self.fitted = False     # True = lost 是拟合出来的；False = 只能给区间
        self.reliable = False   # True = 斜段够长，H 和 k 都可信
        self.ok = False
        self.done = False
        self.t_phase = None
        # 时钟可替换，理由同 SignProbe：整趟标定 6 秒，测试里不该真等 6 秒。
        self._now = time.monotonic

    # ---- 一帧一次 ----
    def update(self, target, w: int, h: int, now: float | None = None) -> str:
        if now is None:
            now = self._now()
        if self.done:
            return self.phase
        if self.phase == "START":
            return self._start_load()
        if self.phase in ("BASE", "STEP"):
            return self._sample(target, w, h)

        # LOAD / STEP_MOVE / BACK：不采样，只等驱动把行程走完。
        settle = (self.settle_load_s if self.phase == "LOAD"
                  else self.settle_step_s)
        if now - self.t_phase < settle:
            self.m.tag = f"LASH-{self.phase}"
            return self.phase
        if self.phase == "LOAD":
            self.phase = "BASE"
            self.samples = []
            self.m.note = "回差标定：该侧回差已吃掉，采基线"
            return self.phase
        if self.phase == "STEP_MOVE":
            self.phase = "STEP"
            self.samples = []
            return self.phase
        # BACK 走完：受控轴已在起点上，收工。
        self._finish()
        return self.phase

    def _sample(self, target, w: int, h: int) -> str:
        err = target.error(w, h) if target.found else None
        if err is None:
            # 丢目标就**清空重来**：半截样本混着"目标不在"，中位数没有意义，
            # 而它算出的 d_i 是判"动没动"的唯一依据——宁可从这一步重采。
            self.samples = []
            self.m.note = "回差标定：目标不在画面里，等它回来（别用手挡镜头）"
            self.m.tag = "LASH-WAIT"
            return self.phase
        self.samples.append(err[self.comp])
        self.m.tag = f"LASH-{self.phase}{len(self.samples)}"
        self.m.note = (f"回差标定 {self.phase}：采样 {len(self.samples)}/"
                       f"{self.SAMPLES}")
        if len(self.samples) < self.SAMPLES:
            return self.phase
        med = _median(self.samples)
        if self.phase == "BASE":
            self.base = med
            return self._start_step(1)
        self.steps.append((self.step_i, med - self.base))
        return self._next_step()

    def _next_step(self) -> str:
        """退完一步。**一律退满，不提前收工**。

        早先版本见"第一步就动"就收工（H=0 嘛，省 5 秒）。那是错的：单步位移
        只有 |k| ≈ 2.3 px，而中位数样本的噪声 σ ≈ 0.8 px，门限 1.0 px 被噪声
        顶过去的概率接近两成——一步误判就收工，报出来的 H=0.0 是**一个很自信
        的错数**（宪令第 7 条）。退满之后由 `_finish` 用尾部几步一起估，噪声
        摊掉，H=0 也照样拟合得出来，不必单开一条路。
        """
        if self.step_i >= self.max_back:
            self._return()
            return self.phase
        return self._start_step(self.step_i + 1)

    # ---- 走一步 ----
    def _start_load(self) -> str:
        t = self.m.guard()
        if t is None:
            # guard() 失败时**一个包都没发过**，所以没什么要归还的。
            self.phase = "FAIL"
            self.done = True
            self.m.note = "回差标定中止：没有新鲜遥测（没有下发过任何动作）"
            self.m.tag = "LASH-NOTELEM"
            return self.phase

        if self.axis == "pan":
            self.ax0, lo, hi = t.pan_cur, t.pan_safe_min, t.pan_safe_max
            if self.pan_max is not None:
                hi = min(hi, self.pan_max)
        else:
            self.ax0, lo, hi = t.tilt_cur, t.tilt_safe_min, t.tilt_safe_max
        # 往哪边先走随起点定，但**方向必须记下来参与符号**：k 的符号跟着 dir 翻，
        # 而"光学斜率 = −dir×k"。贴边时改走另一侧，结论一样成立。
        if self.ax0 + self.load <= hi:
            self.dir = +1
        elif self.ax0 - self.load >= lo:
            self.dir = -1
        else:
            self.phase = "FAIL"
            self.done = True
            self.m.note = (f"回差标定做不了：{self.axis} 可用行程不足 {self.load}"
                           f" tick（cur={self.ax0} 范围[{lo},{hi}]）")
            self.m.tag = "LASH-NOROOM"
            return self.phase

        self._send_axis(self.ax0 + self.dir * self.load, t)
        self.t_phase = self._now()
        self.phase = "LOAD"
        self.m.tag = "LASH-LOAD"
        self.m.note = (f"回差标定：{self.axis} {self.ax0} → "
                       f"{self.ax0 + self.dir * self.load}"
                       f"（{self.dir * self.load:+d} tick）吃掉回差，等驱动走完…")
        return self.phase

    def _start_step(self, i: int) -> str:
        t = self.m.guard()
        if t is None:
            # 已经发过包了（起点那条），所以必须归还。
            self.ok = False
            self._return()
            return self.phase
        self.step_i = i
        to = self.ax0 + self.dir * (self.load - i)
        self._send_axis(to, t)
        self.t_phase = self._now()
        self.phase = "STEP_MOVE"
        self.m.tag = f"LASH-STEP{i}"
        self.m.note = (f"回差标定：反向第 {i}/{self.max_back} 步 → "
                       f"{self.axis}={to}，等驱动走完…")
        return self.phase

    @classmethod
    def _fit_hinge(cls, steps, max_back):
        """把折线 d_i = k·max(0, i−H) 最小二乘拟合到台阶上，返回 (H, k)。

        **为什么是折线，而不是"取最后几个点做直线"。** 台阶**平的那一段**
        （还没咬合的那几步）同样带信息：它说明 H 不小于哪几步。只用斜段等于
        丢掉一半数据，估出来的 k 就抖——而 H = i − d_i/k 对 k 的误差是按
        d_i/k ≈ 5.6 倍放大的（上一版在 ±1.5 px 噪声下就飘出半格，正是这个）。

        H 只有一维，直接扫网格：每个候选 H 用最小二乘定出 k，取残差平方和
        最小的那个。步长 max_back/200 比任何传感器的分辨率都细。
        """
        best = None
        for g in range(201):
            h = max_back * g / 200.0
            num = den = 0.0
            cnt = 0
            for i, d in steps:
                x = i - h
                if x > 0:
                    num += x * d
                    den += x * x
                    cnt += 1
            # 斜段太短就不认：一个点的斜段能把 k 凑成任意值。
            if cnt < cls.MIN_FIT_POINTS or den <= 0:
                continue
            k = num / den
            if abs(k) < cls.MIN_SLOPE_PX:
                continue
            res = 0.0
            for i, d in steps:
                x = i - h
                res += (d - (k * x if x > 0 else 0.0)) ** 2
            if best is None or res < best[0]:
                best = (res, h, k)
        return (None, None) if best is None else (best[1], best[2])

    def _finish(self) -> None:
        """收工：从台阶读 H 和 k。

        主路是 `_fit_hinge`——把整条台阶（平的+斜的）一起拟合，H 和 k 同时出来。

        拟合不成（斜段不足两点、|k| 太小、H 出界）才**退回数步子**：第
        ``first_move`` 步才动，说明前 first_move−1 步是空走的。那只能给到
        ⌈H⌉——所以此时报的是一个**区间**而不是一个数（见 _announce），否则
        就成了"编一个很自信的数"。
        """
        self.first_move = None
        for i, d in self.steps:
            if abs(d) >= self.MIN_DELTA_PX:
                self.first_move = i
                break
        h, k = self._fit_hinge(self.steps, self.max_back)
        if h is not None and 0.0 <= h <= self.max_back:
            self.k, self.lost, self.fitted = k, h, True
            # 斜段越短，k 的条件数越差，H 跟着飘：仿真里 H=6（斜段 2 步）在
            # ±3 px 噪声下能飘到 ±2.5 tick，而 H=2.4（斜段 5.6 步）只飘 ±0.7。
            # 所以"斜段剩几步"要单独报出来，不能拿同一个 H 一视同仁。
            self.reliable = (self.max_back - self.lost) >= self.RELIABLE_SLANT
        elif self.first_move is not None:
            self.lost, self.fitted = float(self.first_move - 1), False
            self.lost_hi = float(self.first_move)
        self.ok = self.lost is not None
        self.phase = "DONE" if self.ok else "FAIL"
        self.done = True
        self._announce()

    def _send_axis(self, tick: int, t) -> None:
        """把 ``tick`` 下到受控轴，另一轴按遥测原样带上。

        MODE_MANUAL 两个 tick 是一起带的，漏掉一个就会把**另一个轴**拖到
        0——所以这里必须显式补上未受控那一路，不能只发自己那一半。
        """
        if self.axis == "pan":
            self.m.cli.set_target(P.MODE_MANUAL, tick, t.tilt_cur,
                                  immediate=True)
        else:
            self.m.cli.set_target(P.MODE_MANUAL, t.pan_cur, tick,
                                  immediate=True)

    def _return(self) -> None:
        """归还：把受控轴放回起点。失败路径也要走这里。"""
        t = self.m.guard()
        if t is not None and self.ax0 is not None:
            self._send_axis(self.ax0, t)
        self.phase = "BACK"
        self.t_phase = self._now()

    def abort(self) -> None:
        """操作者按 q / 到点：立刻归还（不等采样，也不走状态机）。"""
        if self.done:
            return
        try:
            self._return()
        except Exception:      # 收尾路径不许再抛，否则 q 就退不出去了
            pass
        self.done = True
        self.m.note = f"回差标定中断，{self.axis} 已放回起点"
        self.m.tag = "LASH-ABORT"

    def brief(self) -> str:
        # 只能 ASCII：这一行画在窗口上（cv2.putText 认不得中文，会画成方块）。
        return f"{self.m.tag} back{self.step_i}/{self.max_back}"

    def summary(self) -> str:
        if not self.ok:
            return f"回差标定：{self.phase}（H≥{self.max_back} tick，量不到头）"
        if not self.fitted:
            return (f"回差标定：{self.phase}（H ∈ ({self.lost:.0f}, "
                    f"{self.lost_hi:.0f}] tick，数步子给的区间）")
        if not self.reliable:
            return (f"回差标定：{self.phase}（H≈{self.lost:.1f} tick，"
                    f"顶到台阶末端、分辨率差）")
        return f"回差标定：{self.phase}（H≈{self.lost:.1f} tick）"

    def _announce(self) -> None:
        tbl = "  ".join(f"{i}:{d:+.1f}" for i, d in self.steps)
        print(f"\n[lash] {self.axis}：先朝 {self.dir:+d} 走 {self.load} tick 吃掉"
              f"回差，再反向每次退 1 tick。逐步中位数偏离基线（步:d/px）：")
        print(f"[lash]   {tbl}")
        if not self.ok:
            print(f"[lash] 退到 {self.max_back} tick 画面仍没动（全部 |d| < "
                  f"{self.MIN_DELTA_PX:.1f} px）：丢失行程 **≥{self.max_back}"
                  f" tick**——比一整格分度值大得多，闭环靠调参数是治不了的。")
            print(f"[lash] 先查机械：舵盘螺丝松没松、连杆有没有旷量、舵机是不是"
                  f"被打到限位卡住了。{self.axis} 已放回起点。")
            self.m.note = (f"回差标定：{self.axis} 的 H≥{self.max_back} tick，"
                           f"量不到头——先查机械")
            self.m.tag = "LASH-BAD"
            return
        if self.fitted:
            print(f"[lash] 画面在第 {self.first_move} 步才动，丢失行程 "
                  f"H ≈ {self.lost:.1f} tick（{self.lost * 0.4395:.2f}°）")
            if not self.reliable:
                # H 顶到台阶末端时斜段只剩一两步，解算吃紧、噪声放大得厉害
                # （仿真 ±3 px 上能飘 ±2.5 tick）。这不是错，是量程到头了。
                print(f"[lash] 注意：H 已经顶到 {self.max_back} 步台阶的末端"
                      f"（斜段只剩 {self.max_back - self.lost:.1f} 步），这个数"
                      f"的分辨率很差。要更准就单独跑一趟 --calib-lash 并把 "
                      f"LOAD_TICKS/MAX_BACK 调大，或者直接去看机械间隙。")
            opt = -self.dir * self.k          # 翻回「tick↑」方向的光学斜率
            pred = opt * (self.load - self.lost) / self.load
            print(f"[lash] 扣掉回差后的光学斜率 ≈ {opt:+.2f} px/tick。"
                  f"--calib-sign 是走 {self.load} tick 再把总位移除以 {self.load}，"
                  f"其中 {self.lost:.1f} tick 是空走的，所以它必然偏小：按这串"
                  f"读数它应当是 {pred:+.2f} px/tick。拿实测值比一下，对得上就"
                  f"说明两趟标定自洽。")
        else:
            # 拟合不成时**只给区间**。数步子数不出 H 落在第几步之内，硬报一个
            # 小数就是编——而编出来的数最像真的，也最容易被拿去做决定。
            print(f"[lash] 台阶的斜段太短（{self.max_back} 步里只有最后 "
                  f"{self.max_back - self.first_move + 1} 步在动），拟合出不来"
                  f"光学斜率，只能给区间：")
            print(f"[lash] 丢失行程 **H ∈ ({self.lost:.0f}, {self.lost_hi:.0f}] "
                  f"tick**——第 {self.first_move} 步才动，所以它比 {self.lost:.0f} "
                  f"大、不超过 {self.lost_hi:.0f}。要更细就把 --calib-lash 单独"
                  f"跑，多退几步。")
        # 结论按"下界是否已 ≥1"来分：区间 (1,2] 的下界 1.0 就足以说明问题，
        # 不必等拟合——拟合只是把 H 定得更细，不改变量级上的判断。
        h_txt = (f"H ≈ {self.lost:.1f} tick" if self.fitted
                 else f"H ∈ ({self.lost:.0f}, {self.lost_hi:.0f}] tick")
        if self.lost >= 1.0:
            print(f"[lash] 结论：{h_txt} ≥ 一格分度值。输出的 1 tick 变化会先被"
                  f"回差整段吃掉、画面不动，闭环于是「喊了不动、攒够才动」，"
                  f"锁定后在分度值附近摆动**是必然的**，调 kp / deadband 都治"
                  f"不了它。要么 --roi 择优锁定（进 ROI 就不再连续追 0），要么"
                  f"把 H 从机械上消掉。")
        else:
            print(f"[lash] 结论：{h_txt}，不到一格，回差不是摆动的主因——回去看"
                  f" --calib-sign 的斜率与 --deadband。")
        if self.fitted:
            self.m.note = (f"回差标定完成：H≈{self.lost:.1f} tick，光学斜率 "
                           f"{-self.dir * self.k:+.2f} px/tick")
        else:
            self.m.note = (f"回差标定完成：H ∈ ({self.lost:.0f}, "
                           f"{self.lost_hi:.0f}] tick")
        self.m.tag = "LASH-OK"

    def render(self) -> str:
        h = "----" if self.lost is None else f"{self.lost:.1f}"
        k = "----" if self.k is None else f"{-self.dir * self.k:+.2f}"
        return (f"{self.phase:<8} {self.m.tag:<12} 步{self.step_i}/"
                f"{self.max_back} 采样{len(self.samples)}/{self.SAMPLES} "
                f"H={h}t 光学斜率{k} px/tick")


def read_key(fd: int, timeout: float) -> str | None:
    """非阻塞读一个字符。用 select 而不是设 VMIN=0，免得空转吃满 CPU。"""
    import os
    import select

    r, _, _ = select.select([fd], [], [], timeout)
    if not r:
        return None
    try:
        # 直接用 os.read 而不是 sys.stdin.read：后者是带缓冲的 TextIOWrapper，
        # 在非规范模式下可能为了凑缓冲再多等一会儿，按键手感会发黏。
        b = os.read(fd, 1)
    except OSError:
        return None
    return b.decode("utf-8", "ignore") or None


def handle_key(m: Manual, key: str, auto: "Auto | None" = None) -> bool:
    """处理一个按键，返回 False 表示要退出。

    抽出来是为了让**终端和视频窗口走同一条命令路径**（计划 §5：手动模式
    必须复用自动模式那套下发路径）。如果窗口那一支另写一份按键处理，
    两边迟早会在"空格是停还是复位""步长改谁"这类细节上分叉，
    而验收时只测了其中一支。

    ``auto`` 非空（3D 闭环在跑）时，点动键一律拒绝：闭环每帧都在改目标，
    手动那一下会在下一帧被覆盖掉，操作者只会看到"按键没反应"，
    比起效更糟。空格仍然是**停机键**——它挂起闭环并回 HOLD，
    这是操作者随时能把云台从他的控制环里摘出来的唯一出口。
    """
    k = key.lower()
    if k == "q":
        return False
    elif auto is not None and auto.active and k in ("a", "d", "w", "s"):
        m.note = "自动跟踪中：点动已禁用，先按空格挂起闭环"
        m.tag = "AUTO-ON"
        return True
    elif auto is not None and key == " ":
        # 空格在闭环里是**挂起/恢复**，不是 Manual.stop()：两者都下发 HOLD
        # （也就是停住），但挂起还多一件事——把控制权交回给操作者。
        if patrol is not None:
            # 挂起巡检要显式停住：巡检不是 Auto 那种"等下一帧再判"的状态机
            # （它自己握着 cmd_pan），不打断就会在操作者手里继续发目标。
            patrol.abort()
        (auto.suspend(m) if auto.active else auto.resume(m))
        return True
    elif k == "a":
        m.move(+m.step, 0)
    elif k == "d":
        m.move(-m.step, 0)
    elif k == "w":
        m.move(0, +m.step)
    elif k == "s":
        m.move(0, -m.step)
    elif key == " ":
        m.stop()
    elif k == "h":
        m.home()
    elif k == "r":
        tt = m.guard()
        if tt:
            m.align(tt)
    elif key in "+=":
        m.step = min(m.step + 1, 50)
        m.note = f"步长 → {m.step}"
        m.tag = f"STEP={m.step}"
    elif key in "-_":
        m.step = max(m.step - 1, 1)
        m.note = f"步长 → {m.step}"
        m.tag = f"STEP={m.step}"
    return True


def run_interactive(m: Manual) -> int:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(HELP)
    try:
        tty.setcbreak(fd)
        while True:
            t, _ = m.fresh()
            # 拿到第一份遥测就把目标对齐到当前姿态，之后才允许操作。
            if t is not None and not m.aligned:
                m.align(t)

            key = read_key(fd, 0.05)  # 50 ms ≈ 心跳周期，显示刷新够跟手
            if key is not None and not handle_key(m, key):
                break
            sys.stdout.write(m.render())
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n收尾：")
    return 0


# ------------------------------------------------------------------ 视频面
# 从这一行往下都是 3C 的：视频进来、算出目标、画到屏幕上。
# 全部**惰性导入** cv2/numpy（在函数里 import，不在文件头）：
# 没有装 OpenCV 的机器上，`main.py --stream` 这条 3B 的只读路径必须照常能用。
# 把 cv2 提到文件头会让 3B 的验收依赖一个 3C 才需要的东西。

def open_window(name: str) -> bool:
    """试着开一个显示窗口。开不出来返回 False（而不是抛异常）。

    远程 SSH、没有 X 转发、"DISPLAY 指向一个没在跑的 X"这三种情况下
    imshow 都会失败。失败不该让整个采集跑不起来——那时应当退化成
    "照常收帧、照常检测、把带 HUD 的图存成 PNG"，这才是有用的行为。
    """
    import cv2

    try:
        cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
        return True
    except cv2.error as exc:
        print(f"[warn] 开不了显示窗口（{exc.__class__.__name__}），"
              f"改用无窗口模式：只落盘 PNG。", file=sys.stderr)
        return False


class VideoPlane:
    """视频面：收帧线程 + 检测 + HUD。

    `--video` 不打开时这个类根本不会被构造，3B 的行为一点都不变。
    """

    def __init__(self, port: int, cfg=None):
        from tracker import ColorCfg, Target, get_target_center, to_bgr
        from video_rx import FramePump, VideoReceiver

        self.rx = VideoReceiver(port=port)
        self.pump = FramePump(self.rx)
        self.cfg = cfg or ColorCfg()
        self._Target = Target
        self._detect = get_target_center
        self._to_bgr = to_bgr

        self.target = Target(found=False)
        self.frame = None            # 最近一帧原始数据（用于"没新帧也重画"）
        self.bgr = None              # 上面那帧解码后的 BGR，检测和显示共用
        self.detected = 0            # 跑过检测的帧数
        self.lost_streak = 0         # 连续没找到目标的帧数

    def start(self) -> None:
        self.pump.start()

    def close(self) -> None:
        self.pump.stop()
        self.rx.close()

    def poll(self) -> bool:
        """取走最新一帧、解码、检测。返回 False 表示这一轮没有新帧。

        检测放在**主线程**（也就是显示线程），不是收帧线程：HSV + 轮廓
        在 320×240 上是几毫秒的事，但它终究是可变开销，放在收帧线程里
        就等于给收包加了一个随内容变化的延迟。

        解码只做一次，检测和显示共用同一张 BGR：既省一遍 cvtColor，
        也让落盘的验收图里框住的东西**确实就是**检测器看到的那张图。
        各解各的时候，两边的 stride/格式假设一旦不一致，
        会出现"图上明明有目标，框却偏到别处"这种最难解释的现象。
        """
        f = self.pump.take()
        if f is None:
            return False
        self.frame = f
        self.bgr = self._to_bgr(f)
        self.detected += 1
        self.target = self._detect(self.bgr, self.cfg)
        if self.target.found:
            self.lost_streak = 0
        else:
            self.lost_streak += 1
        return True

    def hud(self, m: "Manual", link: str, note: str) -> "object":
        """把遥测 + 检测 + 链路拼成一份 Hud。纯组装，不画。

        跟踪状态和遥测状态**分开报**：它们互不依赖（板端 daemon 没跑
        也照样看得见目标，反之亦然），合成一个字段就会让其中一个把
        另一个盖掉，而那正好是排查时最需要区分开的两件事。
        """
        from display import Hud

        t, age = m.fresh()
        if self.frame is None:
            state = "NO-VIDEO"
        elif self.target.found:
            state = "TRACK"
        else:
            state = "LOST"

        if t is None:
            telem = "NO-TELEM"
        elif age > m.max_age_ms:
            telem = "STALE"
        else:
            telem = "TELEM-OK"

        err = None
        if self.frame is not None:
            err = self.target.error(self.frame.width, self.frame.height)

        return Hud(
            state=state,
            telem=telem,
            target=self.target,
            error=err,
            tick_pan=t.pan_cur if t else None,
            tick_tilt=t.tilt_cur if t else None,
            cmd_pan=t.pan_tgt if t else None,
            cmd_tilt=t.tilt_tgt if t else None,
            fps=self.rx.stats.rate(),
            frame_id=self.frame.frame_id if self.frame is not None else None,
            link=link,
            note=note,
        )


def video_keys(fd: int, window: str | None, wait_ms: int = 1) -> list[str]:
    """这一轮收到的按键，来自**终端和窗口两条**。

    两条都要收：操作者眼睛盯着窗口，手却很可能在终端里——只收窗口的话
    焦点在终端上时按 q 退不掉，只收终端的话焦点在窗口上时一个键都进不来。
    两边返回的字符是同一套（a/d/w/s/空格/H/r/+/-/q），所以下游共用
    handle_key，不会分叉。
    """
    keys: list[str] = []
    k = read_key(fd, 0.0)
    if k is not None:
        keys.append(k)
    if window is not None:
        import cv2

        code = cv2.waitKey(wait_ms) & 0xFF
        if code == 27:                        # ESC 也当退出
            keys.append("q")
        elif code != 255:                     # 255 = 这一轮没按键
            keys.append(chr(code))
    return keys


def run_video(m: Manual, vp: VideoPlane, seconds: float,
              window: str | None, out_dir: str | None,
              save_every: int, auto: "Auto | None" = None,
              probe: "SignProbe | LashProbe | None" = None,
              csv_path: str | None = None,
              lash: "LashProbe | None" = None,
              lash_budget: float = 0.0,
              patrol: "Patrol | None" = None) -> int:
    """3C/3D 主循环：视频 + 手动控制（+ 可选的闭环）。

    收帧在 FramePump 的线程里，本线程只做检测/绘制/读取按键。
    这不是为了跑得快，是为了**收包不被绘制挡住**——一旦挡住，
    内核 rcvbuf（实测 416 KiB）溢出开始丢包，而且丢得无声无息。

    ``auto`` 非空就是 3D：闭环**和检测同一节拍**跑，都在这一个线程里。
    这不是巧合——闭环的输入就是这一帧的检测结果，放到别的线程去
    只会引入一个"拿的是哪一帧"的问题，而那个问题没有任何传感器能回答。
    """
    import os

    from display import link_line, render

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd) if sys.stdin.isatty() else None
    if old is not None:
        tty.setcbreak(fd)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    # CSV 在 try 外开、在 finally 里关：run_video 有十几条 return 路径
    # （收到 q、时长到、收帧线程异常、probe 判失败…），漏掉一条就会
    # 留下半截文件——而"少一段的日志"比"没有日志"更容易被误读成没问题。
    csv = CsvLog(csv_path) if csv_path else None
    print(HELP)
    print(f"视频面已就绪：UDP {vp.rx.port}，rcvbuf={vp.rx.rcvbuf // 1024} KiB"
          + (f"，窗口 '{window}'" if window else "，无窗口（只落盘）"))

    t0 = time.monotonic()
    last_report = 0.0
    saved = 0
    warned_no_video = False
    frame_img = None             # 最近一张画好的 HUD 图，供 imshow 复用
    try:
        while True:
            now = time.monotonic()
            if seconds > 0 and now - t0 >= seconds:
                print(f"\n到点收工（{seconds:.0f} s）")
                break

            t, _ = m.fresh()
            if t is not None and not m.aligned:
                m.align(t)

            fresh = vp.poll()
            if vp.pump.error is not None:
                print(f"\n[error] 收帧线程异常退出：{vp.pump.error!r}",
                      file=sys.stderr)
                return 1
            if vp.frame is None and not warned_no_video and now - t0 > 3.0:
                warned_no_video = True
                print("\n[warn] 3 秒了还没收到一帧视频。检查：板端 video_link "
                      "是否在跑、9200 是否被本机防火墙挡了。")

            # 闭环跟着**新帧**走，不是跟着循环走。无窗口时这个 while 一圈
            # 只有 5 ms，按圈跑就等于每帧重复下发 13 次同一个目标——白烧
            # 50 倍的指令，还会把"每帧一次"的控制周期算错。
            #
            # ``lash`` 是 --calib-lash --auto 时的前奏：它用 MODE_MANUAL 独占
            # 受控轴，所以在它 done 之前闭环**一帧都不许动**——两个写者往同一
            # 条轴上发目标，谁后发谁赢，混在一起两条都没意义。
            # 巡检的触发不跟帧走（它是"事件来了没有"，与画面无关），
            # 每圈都判一次，免得在低帧率下把事件拖到下一帧才响应。
            if patrol is not None:
                t_ev, age_ev = m.fresh()
                ev = (t_ev.ev_count
                      if (t_ev is not None and age_ev <= m.max_age_ms) else None)
                if patrol.trigger(ev, now):
                    print(f"\n[patrol] 收到振动事件（计数 {ev}），启动第 "
                          f"{patrol.runs} 次巡检 {patrol.lo}..{patrol.hi}，"
                          f"预算 {patrol.max_s:.0f}s")
                # 帧之外也要能到点：视频链路一断就没有新帧了（见 Patrol.tick）。
                was = patrol.state
                patrol.tick(now)
                if was != patrol.state:
                    print(f"\n[patrol] {patrol.tag} {patrol.note}")

            if lash is not None and not lash.done:
                if lash_budget > 0 and now - t0 >= lash_budget:
                    if not lash.done:
                        lash.abort()
                        print(f"\n[lash] {lash_budget:.0f} s 到点，已把 "
                              f"{lash.axis} 放回起点，交给闭环")
                elif vp.frame is not None:
                    lash.update(vp.target, vp.frame.width, vp.frame.height)
            elif patrol is not None and patrol.active:
                # 巡检握着控制权时闭环一帧都不许动（同 LashProbe 的理由）。
                if fresh and vp.frame is not None:
                    was = patrol.state
                    patrol.update(vp.target, vp.frame.width, vp.frame.height)
                    if was != patrol.state and patrol.state in ("FOUND",
                                                                "TIMEOUT"):
                        print(f"\n[patrol] {patrol.tag} {patrol.note}")
                    if csv is not None:
                        # 巡检这一段也要落盘：它才是 Phase 7 的证据主体，
                        # 断在 Auto 分支里等于最该留档的一段没有档。
                        csv.row(m, patrol)
            elif auto is not None and fresh and vp.frame is not None:
                auto.update(vp.target, vp.frame.width, vp.frame.height)
                if csv is not None:
                    csv.row(m, auto)

            # 标定也跟着**新帧**走，理由和闭环一样：它的输入就是检测结果。
            # 它自己按时间推进状态，所以中途没有新帧只会让它慢一点，不会错。
            if probe is not None and not probe.done and vp.frame is not None:
                probe.update(vp.target, vp.frame.width, vp.frame.height)
                if probe.done:
                    return 0 if probe.ok else 2

            link = link_line(vp.rx.stats, vp.rx.rcvbuf)
            # 用 m.tag 而不是 m.note：窗口上只能画 ASCII（见 Manual.tag 的说明）。
            # 回差标定在前奏时占着受控轴，它的进度比闭环的更该看。
            if lash is not None and not lash.done:
                note = lash.brief()
                hud_extra = lash.render()
            elif patrol is not None and patrol.active:
                note = f"{patrol.tag} {patrol.cmd_pan}→({patrol.lo}..{patrol.hi})"
                hud_extra = patrol.render()
            elif auto is not None:
                e = auto.last_err
                note = (f"{m.tag} err={e[0]:+.0f}px kp={auto.kp:+.2f}"
                        if e is not None else f"{m.tag} kp={auto.kp:+.2f}")
                hud_extra = auto.render()
            elif probe is not None:
                note = probe.brief()
                hud_extra = probe.render()
            else:
                note = f"{m.tag} step={m.step} drop={vp.pump.dropped}"
                hud_extra = ""
            if fresh and vp.bgr is not None:
                # 只在**新帧**上重画。vp.bgr 收到第一帧之后就永远非 None，
                # 不判 fresh 的话这个循环会以几百 Hz 反复重画同一帧、
                # 反复 imwrite 同一张 PNG——2026-10-07 实机那一轮 1542 帧
                # 变成了 691 次写盘（实际只有 153 张不同的图），白烧 CPU。
                # render 返回新图，不改 vp.bgr——下一帧还要拿它做检测。
                frame_img = render(vp.bgr, vp.hud(m, link, note))
                if out_dir and vp.detected % max(save_every, 1) == 0:
                    import cv2

                    path = os.path.join(out_dir,
                                        f"f{vp.frame.frame_id:06d}.png")
                    cv2.imwrite(path, frame_img)
                    saved += 1
            if frame_img is not None and window:
                # imshow 每轮都要调（它同时负责把 GUI 事件泵出去），
                # 但喂的是上一张画好的图，不重新绘制。
                import cv2

                cv2.imshow(window, frame_img)

            for key in video_keys(fd, window):
                if not handle_key(m, key, auto):
                    print("\n收到退出键")
                    if probe is not None:
                        # 标定中途退出：把受控轴放回起点再退。这条**不能省**，
                        # 否则退出时云台正停在 起点+JOG 上，而用户以为
                        # "只是按了个 q"。
                        probe.abort()
                    if lash is not None and not lash.done:
                        lash.abort()
                    return 0

            if now - last_report >= 2.0:
                last_report = now
                h = vp.hud(m, link, "")
                print(f"\r{h.state:<8} {h.telem:<9} "
                      f"检测{vp.detected}帧 收帧{vp.rx.stats.frames_complete}"
                      f" 丢{vp.pump.dropped} {hud_extra} "
                      f"btn={m.note:<40}", end="")
                sys.stdout.flush()

            # 无窗口时这个循环没有任何天然节拍：read_key 是非阻塞的，
            # poll() 也几乎立刻返回，不睡就会空转烧掉整个核——
            # 一次 20 秒的无窗口验收能白烧 20 秒 CPU。有窗口时不用睡，
            # video_keys 里的 cv2.waitKey(1) 已经带了 1 ms 让出；
            # 但还没收到第一帧时也睡一下，别在这几秒里空转到 1000 Hz。
            if window is None or frame_img is None:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        if csv is not None:
            csv.close()
        if window:
            import cv2

            cv2.destroyAllWindows()
        if old is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if csv is not None:
            print(f"\n逐帧 CSV：{csv_path}（{csv.rows} 行）")
        print(f"\n共检测 {vp.detected} 帧"
              + (f"，落盘 {saved} 张 PNG 到 {out_dir}" if out_dir else ""))
        if auto is not None:
            print(f"闭环：进环 {auto.frames} 帧 下发 {auto.moves} 次 "
                  f"撞限位 {auto.limit_hits} 次 "
                  f"（喂不满帧间隔累计 {auto._stall_ms:.0f} ms）")
        # 收尾前先把还停在半路的标定归还：到点、异常、按 q 都会走到这里，
        # 而标定把轴挪出去之后是要放回来的。漏掉这一条，云台会停在
        # 「起点 + 若干 tick」上，操作者却以为"只是跑完了"——一个不报的位移。
        for _pr in (probe, lash):
            if _pr is not None and not _pr.done:
                _pr.abort()
        if probe is not None:
            print(probe.summary())
        if lash is not None:
            print(lash.summary())
        st = vp.rx.stats
        print(f"视频：完整 {st.frames_complete} 残缺 {st.frames_incomplete} "
              f"跳号 {st.frames_lost} 片丢 {st.chunk_loss_pct():.2f}% "
              f"dup={st.dup_chunks} late={st.late_chunks}")
    return 0


def _parse_hsv(spec: str) -> tuple:
    """解析 'h:s:v,h:s:v;h:s:v,h:s:v' → ((lo,hi),(lo,hi),...) 。"""
    out = []
    for seg in spec.split(";"):
        parts = seg.split(",")
        if len(parts) != 2:
            raise ValueError(f"阈值段 {seg!r} 应形如 'h:s:v,h:s:v'")
        bounds = []
        for p in parts:
            n = p.split(":")
            if len(n) != 3:
                raise ValueError(f"{p!r} 应是三个用 : 分隔的数 h:s:v")
            bounds.append(tuple(int(x) for x in n))
        out.append(tuple(bounds))
    return tuple(out)


def _build_cfg(args):
    """把命令行上的阈值变成 ColorCfg；一个都没给时返回 None。

    返回 None 而不是 ColorCfg() 是有意的：`None` 让 tracker 用它自己的
    默认值。两处各写一份默认值，改了一处另一处就会悄悄分叉，
    而分叉的表现是"命令行跑出来和单测跑出来不一样"，很难联想到原因。

    阈值必须能从命令行给：现场光照下的最终值只能实机标定，
    写死在代码里就意味着调一次阈值要改一次代码、再跑一遍单测。
    """
    from tracker import ColorCfg

    kw = {}
    if args.hsv:
        kw["ranges"] = _parse_hsv(args.hsv)
    if args.min_area is not None:
        kw["min_area"] = args.min_area
    if args.morph is not None:
        if args.morph % 2 == 0 or args.morph < 1:
            raise ValueError(f"--morph 必须是正奇数，给了 {args.morph}")
        kw["morph_kernel"] = args.morph
    return ColorCfg(**kw) if kw else None


def run_stream(m: Manual, seconds: float) -> int:
    """非交互模式：只被动显示遥测。用于冒烟检查链路，绝不发动作。"""
    print("非交互模式：只读遥测，不发任何动作（Ctrl-C 结束）")
    t0 = time.monotonic()
    try:
        while seconds <= 0 or time.monotonic() - t0 < seconds:
            t, age = m.fresh()
            if t is None:
                sys.stdout.write(f"\r等待遥测… {time.monotonic() - t0:5.1f}s")
            else:
                sys.stdout.write(m.render())
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print()
    t, _ = m.fresh()
    if t is None:
        print("一条遥测都没收到。检查：daemon 是否在跑、9101 是否被本机防火墙挡了。")
        return 1
    print(f"共收到 {m.rx.received} 包遥测，坏包 {m.rx.bad_pkts}")
    return 0


def auto_banner(auto: "Auto", deadband: float, acquire: int, release: int,
                pan_max: int) -> list[str]:
    """闭环启动横幅。**独立成函数是为了能测**——这几行是操作者上电前
    唯一会看到的判据，尤其「哪条轴的闸门其实没开」这一句。

    两条轴各印一行闸门：它们的斜率是**反号**的（实测 pan −1.83 / tilt +1.83），
    所以「pan 没问题」推不出「tilt 没问题」，合印一行必然误导。
    """
    lines: list[str] = []
    ax_txt = " + ".join(auto.axes)
    kp_txt = auto._kps()
    lines.append(f"3D 闭环：轴={ax_txt} kp={kp_txt} 死区={deadband:g}px "
                 f"步≤{auto._pace_step} 认{acquire}/放{release} "
                 f"pan 上限 {pan_max}")
    lines.append(f"       连续性预算：{auto._pace_step} tick × "
                 f"{auto.slew_ms:.0f} ms = {auto._pace_step * auto.slew_ms:.0f} ms "
                 f"领先量；帧间隔超过这个数就会「走一下停一下」"
                 f"（30 fps 是 33 ms 安全，15 fps 是 67 ms 会告警）")
    lines.append("       空格=挂起/恢复闭环  q=退出  点动键在闭环中禁用")
    for ax in auto.axes:
        p = auto.param[ax]
        if p["slope"] is None:
            flag = auto._flag(ax, "--slope")
            lines.append(f"       ⚠ {ax} 的画面坐标符号**没实测**（{flag} 未给），"
                         f"**该轴的正反馈闸门是开着的**。"
                         f"先跑 `--calib-sign --axis {ax}` 把它量出来。")
        else:
            prod = p["kp"] * p["slope"]
            lines.append(f"       {ax} 闸门：kp={p['kp']:+.2f} × 实测斜率 "
                         f"{p['slope']:+.2f} = {prod:+.2f} "
                         f"{'（收敛，放行）' if prod < 0 else '（发散！会拒绝进环）'}")
    if len(auto.axes) > 1:
        lines.append("       两条轴同时进闭环：**任一条**闸门判发散就整体"
                     "拒绝进环（一次都不下发）。")
    if auto.roi is not None:
        n = len(auto.probe_poses())
        # ROI 半径相对分度值够不够大，是这一条的全部要害：半径小于一格的
        # 像素当量时，ROI 内部照样执行不到，白开一层。
        tic = "  ".join(
            (f"{ax}={abs(p['slope']):.2f}px/tick" if p["slope"] is not None
             else f"{ax}=斜率未实测") for ax, p in auto.param.items())
        lines.append(f"       ROI 择优：半径 {auto.roi:g}px，窗口 "
                     f"{auto.hold_s:g}s（{n} 格 × {auto.hold_s / n:.2f}s，"
                     f"每格按**中位数**打分再取最小）")
        lines.append(f"       进 ROI 门限 {auto.roi:g}px / 出门限 "
                     f"{auto.roi * auto.ROI_RELEASE_K:g}px"
                     f"（滞环 {auto.ROI_RELEASE_K:g}×）。{tic}")
        if any(p["slope"] is not None and auto.roi < abs(p["slope"])
               for p in auto.param.values()):
            lines.append(f"       ⚠ ROI 半径 {auto.roi:g}px 小于一格的像素当量"
                         f"（{tic}）：ROI 内部可能也落不进去，建议取 4 格左右。")
    return lines


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="板端 IP")
    ap.add_argument("--ctrl-port", type=int, default=P.CTRL_PORT)
    ap.add_argument("--telem-port", type=int, default=P.TELEM_PORT)
    ap.add_argument("--rate-hz", type=float, default=20.0)
    ap.add_argument("--step", type=int, default=DEFAULT_STEP,
                    help=f"每次点动的 tick 数（默认 {DEFAULT_STEP}）")
    ap.add_argument("--max-age-ms", type=float, default=500.0,
                    help="遥测超过这么旧就拒绝动作（默认 500）")
    ap.add_argument("--stream", type=float, default=0.0, metavar="SEC",
                    help="非交互：只读遥测 SEC 秒（0 表示一直跑）")

    # ---- 3C：视频 ----
    ap.add_argument("--video", action="store_true",
                    help="打开视频面（UDP 收帧 + 检测 + HUD）。"
                         "不加这个开关就是纯 3B 行为。")
    ap.add_argument("--video-port", type=int, default=P.VIDEO_PORT)
    ap.add_argument("--duration", type=float, default=0.0, metavar="SEC",
                    help="视频模式跑 SEC 秒后收工（0 表示一直跑到按 q）")
    ap.add_argument("--no-window", action="store_true",
                    help="不显示窗口，只收帧/检测/落盘（远程 SSH 用）")
    ap.add_argument("--out-dir", default=None,
                    help="把带 HUD 的图按每 --save-every 帧存一张 PNG 到这里")
    ap.add_argument("--save-every", type=int, default=15,
                    help="每多少帧存一张 PNG（默认 15 ≈ 1 秒一张）")
    ap.add_argument("--csv", default=None, metavar="PATH",
                    help="逐帧落盘 (err, cur, 下发目标, 状态) 到 CSV。**要判闭环"
                         "为什么定不下来就非它不可**：终端 HUD 是 2 秒一行，"
                         "分辨不出极限环的周期。只在 --auto 下有内容")
    ap.add_argument("--hsv", default=None,
                    help="覆盖颜色阈值：'h:s:v,h:s:v;h:s:v,h:s:v'（多段用 ; 分隔）")
    ap.add_argument("--min-area", type=float, default=None,
                    help="最小目标面积（默认用 tracker 里的 300）")
    ap.add_argument("--morph", type=int, default=None,
                    help="形态学核大小（必须奇数，默认 5）")

    # ---- 3D：闭环 ----
    ap.add_argument("--auto", action="store_true",
                    help="开启 3D 视觉闭环（隐含 --video）。空格挂起/恢复，q 退出。")
    ap.add_argument("--kp", type=float, default=-0.5,
                    help="像素→tick 的 P 增益，**带符号**（默认 -0.5，"
                         "对应「目标偏右 → pan tick 减小」。"
                         "符号错了就是正反馈，第一次跑必须先实测确认）")
    ap.add_argument("--deadband", type=float, default=2.0, metavar="PX",
                    help="误差死区，像素（默认 2，小于 1 tick 的像素当量 2.3。"
                         "开到 5 会把运动攒成 2 tick 一迈，单 tick 步进占比从 "
                         "89%% 掉到 13%%；开到 0 则静止目标会来回蹭）")
    ap.add_argument("--max-step", type=int, default=4, metavar="TICK",
                    help="单帧最大步进（默认 4）。这个值决定连续性："
                         "必须 >= 帧间隔/slew_ms，否则每帧末尾驱动会跑空")
    ap.add_argument("--acquire", type=int, default=4, metavar="N",
                    help="连续 N 帧认出目标才进闭环（默认 4，"
                         "3 帧出画误报要求的最小值）")
    ap.add_argument("--release", type=int, default=5, metavar="M",
                    help="连续丢 M 帧才退出闭环（默认 5，要盖住 200 ms 残帧延迟）")
    ap.add_argument("--axis", choices=("pan", "tilt", "both"), default="pan",
                    help="闭环控哪个轴（默认 pan）。both = 两轴同时闭环——"
                         "**两条轴的正反馈闸门各自独立**，必须分别给 --slope 和 "
                         "--slope-tilt；只给一个时另一条轴的门是**关着**的，"
                         "会打横幅警告。只控一个轴时 --calib-sign 量的就是该轴。")
    ap.add_argument("--track-pan-max", type=int, default=490, metavar="TICK",
                    help="跟踪时的 pan 上限（默认 490，比驱动的 515 收紧。"
                         "只作用在 PC 侧，不动 config/ 和驱动）")
    ap.add_argument("--slew-ms", type=float, default=DEFAULT_SLEW_MS,
                    help=f"驱动的每 tick 步进周期，用于连续性自检（默认 "
                         f"{DEFAULT_SLEW_MS:.0f}，对齐 pantilt 驱动）")
    ap.add_argument("--slope", type=float, default=None, metavar="PX/TICK",
                    help="**被控轴**的 --calib-sign 量出的 px/tick（带符号）。"
                         "单轴（--axis pan 或 tilt）时就是这一条轴的；--axis both "
                         "时只算 pan 的，tilt 必须另给 --slope-tilt。给了它就启用"
                         "该轴的正反馈闸门：kp × slope ≥ 0 直接拒绝进闭环")
    ap.add_argument("--kp-tilt", type=float, default=None, metavar="GAIN",
                    help="tilt 轴的 P 增益。**符号与 pan 相反不是笔误**：实测 pan "
                         "斜率 -1.83、tilt +1.83 px/tick，一个 kp 套两条轴必有一条"
                         "是正反馈。不给就沿用 --kp（此时 tilt 的门若没关会报警）")
    ap.add_argument("--slope-tilt", type=float, default=None, metavar="PX/TICK",
                    help="tilt 轴 --calib-sign --axis tilt 量出的 px/tick。"
                         "**刻意不回落**到 --slope：把 pan 的斜率套给 tilt 正好是"
                         "反号，会让闸门失效。不给则该轴的闸门保持关闭")
    ap.add_argument("--patrol", action="store_true",
                    help="Phase 7 事件巡检：遥测 reserved[1] 的事件计数一跳，"
                         "就在 pan 的 [--patrol-lo, --patrol-hi] 之间来回扫，"
                         "连续几帧见到目标就交给闭环；超过 --patrol-max-s "
                         "没找到就回 home 并报告 TIMEOUT。需要 --auto。")
    ap.add_argument("--patrol-max-s", type=float, default=20.0, metavar="SEC",
                    help="单次巡检的时间预算（默认 20 s）。到点无条件回 home，"
                         "绝不「再扫一会儿」——超时兜底存在的意义就是它一定会到")
    ap.add_argument("--patrol-lo", type=int, default=270, metavar="TICK",
                    help="巡检 pan 下端（默认 270，会再被驱动的 safe_min 收紧）")
    ap.add_argument("--patrol-hi", type=int, default=490, metavar="TICK",
                    help="巡检 pan 上端（默认 490）。硬上限 514：515 是机械极限，"
                         "2.51 ms/tick 超出 MG90S 规格，可以路过但不许停")
    ap.add_argument("--patrol-step", type=int, default=24, metavar="TICK",
                    help="巡检每步跨度（默认 24 tick，约 1/10 行程）")
    ap.add_argument("--patrol-tilt", type=int, default=None, metavar="TICK",
                    help="巡检期间固定的 tilt（默认用驱动自报的 tilt_home）")
    ap.add_argument("--roi", type=float, default=None, metavar="PX",
                    help="画面中心的圆形 ROI 半径。给了它就换成「进 ROI 就择优锁定」："
                         "err 小于半径时不再连续追 0（物理上也追不到——执行器一格 "
                         "就是 |slope| px），而是探几个相邻位姿、按各格的**中位数**"
                         "误差挑最好的一个锁住。不给就还是原来的纯 P（锁定后会在"
                         "分度值附近摆）。半径要**大于一格分度值**，否则 ROI 里面"
                         "照样落不进去；经验值是 4 tick ≈ 4×|slope| px")
    ap.add_argument("--hold-window", type=float, default=2.0, metavar="SEC",
                    help="择优一轮的总时长（默认 2 s，摊到 5 个候选位姿上）。"
                         "下限由噪声决定：要在 σ≈1.5 px 下分辨 1 tick=1.83 px，"
                         "每格约需 9 帧，3 格 ≈ 0.9 s。2 s 有 2 倍余量，别往下砍")
    ap.add_argument("--calib-sign", action="store_true",
                    help="先测画面坐标符号再谈跟踪：按 --axis 走 12 tick、看"
                         "对应 err 分量怎么变，印出该用的 kp 和实测 px/tick，"
                         "然后把该轴放回起点。只动 --axis 那一个轴，"
                         "不与 --auto 同用。")
    ap.add_argument("--calib-lash", action="store_true",
                    help="量机械丢失行程（回差＋静摩擦）：先把该轴朝一个方向"
                         "走 12 tick 把回差吃掉，再反向每次退 1 tick，看画面第"
                         "几步才开始动，由台阶斜率反推 H（tick）和扣掉回差后的"
                         "光学斜率。单独用 = 只量；配 --auto 用 = 先量，量完"
                         "自动把轴放回起点交给闭环，一次 2 分钟跑完两件事。"
                         "只动 --axis 那一个轴。")
    ap.add_argument("--lash-seconds", type=float, default=20.0, metavar="SEC",
                    help="--calib-lash 配 --auto 时留给标定的时长（默认 20 s，"
                         "整趟台阶约 6 s，余量给丢帧重采）。到点无论量到哪一步"
                         "都会把轴放回起点、交给闭环，剩下的时间照跑闭环。"
                         "0 = 不限时（标定卡住就吃掉整趟运行）。")
    a = ap.parse_args()

    if a.calib_lash and a.calib_sign:
        ap.error("--calib-lash 和 --calib-sign 各占一趟标定，不能同时跑："
                 "先 `--calib-sign --axis pan` 量符号，再 `--calib-lash "
                 "--axis pan` 量回差。两个数对得上才说明读数自洽。")
    if a.calib_lash and a.axis == "both":
        ap.error("--calib-lash 一次只量一条轴：两条轴的装配、行程、回差各是"
                 "一个数，--axis both 下量出来的 H 是哪条轴的说不清。"
                 "先 `--calib-lash --axis pan`，再 `--calib-lash --axis tilt`。")
    if a.lash_seconds < 0:
        ap.error(f"--lash-seconds {a.lash_seconds:g} 不成时长。给正数秒，"
                 f"0 = 不限。")
    if a.calib_sign and a.auto:
        ap.error("--calib-sign 和 --auto 不能同时用：先量符号，再拿量出来的 "
                 "kp 去跑闭环。")
    if a.calib_sign and a.axis == "both":
        ap.error("--calib-sign 一次只量一条轴：--axis both 下量出来的 px/tick "
                 "是**哪条轴**的说不清（两条轴斜率反号，套错就是正反馈）。"
                 "先 `--calib-sign --axis pan`，再 `--calib-sign --axis tilt`。")
    if not a.auto and (a.kp_tilt is not None or a.slope_tilt is not None):
        ap.error("--kp-tilt / --slope-tilt 只在 --axis both 下有意义；"
                 "单轴闭环请用 --kp / --slope。")
    if a.auto and a.axis != "both" and (a.kp_tilt is not None
                                        or a.slope_tilt is not None):
        ap.error(f"--kp-tilt / --slope-tilt 给了，但 --axis 是 {a.axis}："
                 f"tilt 参数不会生效。要么 --axis both，要么把这两个参数去掉。")
    if a.roi is not None and not a.auto:
        ap.error("--roi 只在 --auto 下有意义：它是闭环里「进 ROI 就不追了」的"
                 "那个判据，手动模式下没有闭环可挂。")
    if a.roi is not None and a.roi <= 0:
        ap.error(f"--roi {a.roi:g} 不成半径。给正数像素值。")
    if a.hold_window <= 0:
        ap.error(f"--hold-window {a.hold_window:g} 不成窗口。给正数秒。")
    if a.csv and not a.auto:
        # 不拦的话会落一个只有表头的空文件，而操作者拿到它会以为"跑过了"。
        ap.error("--csv 只在 --auto 下有内容（逐帧 err 与下发目标都是闭环才有的）。"
                 "要么加 --auto，要么去掉 --csv。")
    if a.csv and not a.video:
        ap.error("--csv 挂在视频主循环上（闭环跟着新帧走），要配 --video。")

    try:
        cfg = _build_cfg(a)
    except ValueError as exc:
        ap.error(str(exc))

    m = Manual(a.host, a.ctrl_port, a.telem_port, a.rate_hz, a.step,
               a.max_age_ms)
    m.start()
    vp = None
    try:
        if a.video or a.auto or a.calib_sign or a.calib_lash:
            window = None
            if not a.no_window:
                name = f"pantilt-track {a.host}"
                window = name if open_window(name) else None
            vp = VideoPlane(a.video_port, cfg)
            vp.start()
            auto = None
            probe = None
            lash = None
            patrol = None
            if a.patrol:
                if not a.auto:
                    ap.error("--patrol 需要 --auto：找到目标之后要交给闭环，"
                             "没有闭环就没有交接对象。")
                if a.patrol_hi > Patrol.PAN_HARD_MAX:
                    ap.error(f"--patrol-hi {a.patrol_hi} 超过机械硬上限 "
                             f"{Patrol.PAN_HARD_MAX}：515 是 pan 的极限，"
                             f"停在那儿会让舵机长时间吃超规格脉宽。")
                if a.patrol_lo >= a.patrol_hi:
                    ap.error(f"--patrol-lo {a.patrol_lo} 必须小于 "
                             f"--patrol-hi {a.patrol_hi}。")
                if a.patrol_max_s <= 0:
                    ap.error("--patrol-max-s 必须为正：巡检没有超时兜底就等于"
                             "没有超时兜底。")
                if a.patrol_step <= 0:
                    ap.error("--patrol-step 必须为正。")
                patrol = Patrol(m, a.patrol_max_s, a.patrol_lo, a.patrol_hi,
                                a.patrol_step, tilt=a.patrol_tilt)
                print(f"[patrol] 事件巡检已就绪：pan {patrol.lo}..{patrol.hi}，"
                      f"一步 {patrol.step} tick，预算 {patrol.max_s:.0f}s，"
                      f"tilt={a.patrol_tilt if a.patrol_tilt is not None else 'home'}")
                print("         触发源 = 遥测 reserved[1] 高 16 位的事件计数；"
                      "计数一跳就开始扫。停掉它：不传 --patrol")
            if a.calib_sign:
                probe = SignProbe(m, axis=a.axis, pan_max=a.track_pan_max,
                                  slew_ms=a.slew_ms, kp_hint=a.kp)
                print(f"符号标定：轴={probe.axis}，走 {probe.jog} tick → 等 "
                      f"{probe.settle_s * 1e3:.0f} ms → 再采样 {probe.SAMPLES} 帧")
                print(f"       只动 {probe.axis}、只用 MODE_MANUAL，结束（含中断）"
                      f"都会把 {probe.axis} 放回起点")
                print("       镜头前放一个稳定的红目标，别挡镜头  q=中断退出")
                return run_video(m, vp, a.duration, window, a.out_dir,
                                 a.save_every, None, probe, a.csv)
            if a.calib_lash and not a.auto:
                # 只量不跑：和 --calib-sign 一样，量完就退。
                probe = LashProbe(m, axis=a.axis, pan_max=a.track_pan_max,
                                  slew_ms=a.slew_ms)
                print(f"回差标定：轴={probe.axis}，先走 {probe.load} tick 吃掉回差"
                      f"（等 {probe.settle_load_s * 1e3:.0f} ms），再反向每步退 "
                      f"1 tick（每步等 {probe.settle_step_s * 1e3:.0f} ms + 采 "
                      f"{probe.SAMPLES} 帧），最多退 {probe.max_back} 步")
                print(f"       只动 {probe.axis}、只用 MODE_MANUAL，结束（含中断）"
                      f"都会把 {probe.axis} 放回起点")
                print("       镜头前放一个稳定的红目标，全程别动它   "
                      "q=中断退出")
                return run_video(m, vp, a.duration, window, a.out_dir,
                                 a.save_every, None, probe, a.csv)
            if a.calib_lash:
                lash = LashProbe(m, axis=a.axis, pan_max=a.track_pan_max,
                                 slew_ms=a.slew_ms)
                print(f"回差标定：轴={lash.axis}，先走 {lash.load} tick 吃掉回差，"
                      f"再反向每步退 1 tick，最多 {lash.max_back} 步，"
                      f"预算 {a.lash_seconds:.0f} s")
                print(f"       标定期间闭环不动（两个写者抢同一条轴，混着发等于"
                      f"谁都没发）；标定一完就把 {lash.axis} 放回起点交给闭环")
            if a.auto:
                auto = Auto(m, kp=a.kp, deadband=a.deadband,
                            max_step=a.max_step, acquire_n=a.acquire,
                            release_m=a.release, axis=a.axis,
                            pan_max=a.track_pan_max, slew_ms=a.slew_ms,
                            slope=a.slope, kp_tilt=a.kp_tilt,
                            slope_tilt=a.slope_tilt,
                            roi=a.roi, hold_s=a.hold_window)
                for line in auto_banner(auto, a.deadband, a.acquire, a.release,
                                        a.track_pan_max):
                    print(line)
                if lash is not None:
                    print(f"       闭环实跑时长 ≈ {a.duration - a.lash_seconds:.0f} s"
                          f"（总 {a.duration:.0f} s 里先扣掉标定预算）")
            return run_video(m, vp, a.duration, window, a.out_dir,
                             a.save_every, auto, None, a.csv,
                             lash=lash, lash_budget=a.lash_seconds,
                             patrol=patrol)
        if a.stream:
            return run_stream(m, a.stream)
        if not sys.stdin.isatty():
            print("stdin 不是终端，改用只读模式；要操作请在真终端里跑。",
                  file=sys.stderr)
            return run_stream(m, 0)
        return run_interactive(m)
    finally:
        if vp is not None:
            vp.close()
        m.close()
        t, _ = m.fresh()
        if t is not None:
            print(f"最终：驱动 {P.DRV_STATUS_NAME.get(t.drv_status, '?')} "
                  f"cur=({t.pan_cur},{t.tilt_cur}) cmd={t.cmd_count} "
                  f"看门狗 驱动{t.watchdog_trips}/daemon{t.daemon_wd_trips} "
                  f"拒包 crc{t.rej_crc}/seq{t.rej_seq}/stale{t.rej_stale}/"
                  f"mode{t.rej_mode}/range{t.rej_range}")
        if m.refused:
            print(f"因遥测不可用而拒绝的动作次数：{m.refused}")


if __name__ == "__main__":
    sys.exit(main())
