# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Assemble a shock-corded antenna mast and lay it across two stands.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A three-section shock-corded mast (green glass-fibre tube, 18 mm square, 130 mm sections, 45 g each) lies on the
  table in a zig-zag. The base section (black rubber foot) lies along y at x 0.36, foot at y +0.195. The middle
  section is bent 60 deg toward the robot at joint 1; the top section (orange cap) is bent 60 deg back at joint 2 and
  lies parallel to the base section at x 0.222. Both spigot tips rest on the rims, 3.5 mm outside the mouths.
- Each joint is an aluminium ferrule socket (26 mm deep, 14 mm bore) on the lower section and a 12 mm square, 26 mm
  spigot on the upper one, joined by an internal shock cord. The cord pulls a lined-up spigot into its ferrule, but
  the snug fit in the last 15 mm stops it 11.7 to 15 mm short of home; pushing it home takes 1.75 to 2.25 N.
- Two grey aluminium saddle stands are fixed at (0.30, -0.30) and (0.30, -0.45), in front of the right arm and
  past the right end of the line the assembled mast would lie on; each has a 28 mm slot along y whose floor is
  40 mm above the table: the transit cradle the finished pole goes in while the antenna head is fitted.

Goal: one straight pole with both joints fully seated (no gap at either ferrule mouth), resting in both stands, both
grippers let go.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import linkkit as lk  # noqa: E402
import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
VARIANT = os.environ.get("DEMO_VARIANT", "")
EPS = 1e-6

# -- mast ---------------------------------------------------------------------------------------------------------
# Linkage frame: x along the mast from the foot of the base section, z up; yawed -90 deg so +x points to world -y.
TUBE = 0.018  # square section
WALL = 0.002  # ferrule wall (bore 14 mm)
SEG = 0.130  # section length (shoulder to end)
D = 0.026  # ferrule depth = spigot length
SPIG = 0.012  # spigot section (1 mm clearance in the bore)
SEG_MASS = 0.045
CORD_MASS = 0.005  # cord anchor (a hidden link inside the tube)
X0, Y0 = 0.36, 0.195  # base section's line and its foot (world)
MAST_POS = (X0, Y0, TOP + TUBE / 2)
MAST_YAW = -90.0
KINK = (-60.0, 60.0)  # start angles of joints 1 and 2 (deg, + = counter-clockwise from above)

CORD_K = 150.0  # N/m
CORD_REST = -0.005  # m: the cord pulls toward 5 mm past seated (0.75 N preload at seated, 4.65 N at the mouth)
CORD_C = 30.0  # N s/m
# The ferrule's snug fit: the last FIT_LEN before home grips the spigot. Stick-slip, emulated on the slide's drive
# every control step (fit_next): it holds until the net push or pull on it passes FIT_STATIC, then slides against
# FIT_DYNAMIC until it stops. (PhysX joint friction is a coefficient on the joint's reaction loads, so it would jam
# under the arms' side loads instead of acting as a fixed fit.)
FIT_LEN = 0.015
FIT_STATIC, FIT_DYNAMIC = 3.0, 2.5  # N
FIT_HOLD_K, FIT_HOLD_C = 3000.0, 30.0  # N/m, N s/m: the stuck slide's stiffness (3 N moves it 1 mm)
STICK_V = 0.002  # m/s: a slide slower than this sticks
REST_IN = CORD_REST + FIT_DYNAMIC / CORD_K  # sliding in: the cord's pull less the fit's drag
REST_OUT = CORD_REST - FIT_DYNAMIC / CORD_K  # pulled out: the cord's pull plus the fit's drag
S_MAX = D + 0.040  # cord fully stretched: the spigot 40 mm outside the mouth
FOLD_DEG = 70.0  # a bent joint stops where the spigot meets the rim
ENTRY_DEG = 3.0  # a spigot enters its ferrule only within this of straight
LOCK_DEG = 0.3  # an engaged spigot holds the joint this straight (1 mm clearance over 20 mm of engagement)
LOCK_DEPTH = 0.020
ENGAGED = 0.0005
RIM_MARGIN = 0.0005


def rim_offset(th: float) -> float:
    """How far (m) a bent spigot's tip must sit outside the mouth for its corner to clear the ferrule's rim: with the
    joint bent ``th`` rad about the spigot tip, the corner of the 12 mm spigot reaches into the 14 mm bore."""
    phi = abs(th)
    if phi < 1e-6:
        return 0.0
    return max(0.0, (SPIG / 2 + RIM_MARGIN - (TUBE / 2 - WALL) * math.cos(phi)) / math.sin(phi))


S_START = D + rim_offset(math.radians(abs(KINK[0])))

GREEN = (0.20, 0.50, 0.22)
ALU = (0.78, 0.78, 0.80)
BLACK = (0.08, 0.08, 0.08)
ORANGE = (0.95, 0.45, 0.08)
GREY = (0.45, 0.47, 0.50)
FIBRE_FRICTION = (0.5, 0.45)
FINE = {"contact_offset": 0.0008}


def _ferrule(x_mouth):
    """The four walls of a ferrule socket ending at ``x_mouth`` (link frame)."""
    c = x_mouth - D / 2
    h = TUBE / 2 - WALL / 2
    return [lk.box((D, TUBE, WALL), (c, 0, h), color=ALU, **FINE), lk.box((D, TUBE, WALL), (c, 0, -h), color=ALU, **FINE),
            lk.box((D, WALL, TUBE - 2 * WALL), (c, h, 0), color=ALU, **FINE),
            lk.box((D, WALL, TUBE - 2 * WALL), (c, -h, 0), color=ALU, **FINE)]


def _spigot():
    return lk.box((D, SPIG, SPIG), (-D / 2, 0, 0), color=ALU, **FINE)


_t = lk.Tree("mast", pos=MAST_POS, yaw_deg=MAST_YAW, color=GREEN)
_t.link("base", [lk.box((0.010, TUBE, TUBE), (0.005, 0, 0), color=BLACK),
                 lk.box((SEG - D - 0.010, TUBE, TUBE), ((0.010 + SEG - D) / 2, 0, 0)), *_ferrule(SEG)],
        mass=SEG_MASS, friction=FIBRE_FRICTION)
_t.link("cord1", [lk.box((0.006, 0.006, 0.006), (-0.050, 0, 0))], mass=CORD_MASS, at=(SEG - D, 0, 0))
_t.link("mid", [_spigot(), lk.box((SEG - D, TUBE, TUBE), ((SEG - D) / 2, 0, 0)), *_ferrule(SEG)], mass=SEG_MASS,
        at=(SEG, 0, 0), friction=FIBRE_FRICTION)
_t.link("cord2", [lk.box((0.006, 0.006, 0.006), (-0.050, 0, 0))], mass=CORD_MASS, at=(2 * SEG - D, 0, 0))
_t.link("top", [_spigot(), lk.box((SEG - 0.010, TUBE, TUBE), ((SEG - 0.010) / 2, 0, 0)),
                lk.box((0.010, TUBE, TUBE), (SEG - 0.005, 0, 0), color=ORANGE)], mass=SEG_MASS, at=(2 * SEG, 0, 0),
        friction=FIBRE_FRICTION)
for _i, (_lo, _hi) in enumerate((("base", "mid"), ("mid", "top")), start=1):
    _t.prismatic(f"s{_i}", _lo, f"cord{_i}", anchor=(_i * SEG, 0, 0), axis=(1, 0, 0), limits=(S_START, S_MAX),
                 rest=CORD_REST, init=S_START, stiffness=CORD_K, damping=CORD_C,
                 brake=lk.Brake(stiffness=FIT_HOLD_K, damping=FIT_HOLD_C, engaged=False))
    _t.revolute(f"f{_i}", f"cord{_i}", _hi, anchor=(_i * SEG - D, 0, 0), axis=(0, 0, 1),
                limits_deg=(-FOLD_DEG, FOLD_DEG), init_deg=KINK[_i - 1], damping=0.002, friction=0.003)
MAST = _t.build()
SECTIONS = ("base", "mid", "top")

# -- stands ---------------------------------------------------------------------------------------------------------
X_ST = 0.30
STAND_Y = (-0.30, -0.45)  # off the right end of the mast's line, clear of everything that swings or slides
FLOOR = 0.040  # slot floor above the table
SLOT = 0.028
_shapes = []
for _y in STAND_Y:
    _shapes += [lk.box((0.040, 0.024, FLOOR), (0, _y, FLOOR / 2)),
                lk.box((0.006, 0.024, 0.012), (SLOT / 2 + 0.003, _y, FLOOR + 0.006)),
                lk.box((0.006, 0.024, 0.012), (-SLOT / 2 - 0.003, _y, FLOOR + 0.006))]
_t = lk.Tree("stands", pos=(X_ST, 0.0, TOP), fixed=True, color=GREY)
_t.link("stands", _shapes, mass=0.40, friction=(0.5, 0.45))
STANDS = _t.build()

OBJECTS = [MAST, STANDS]

# -- the ferrule: entry and lock (joint limits switched every control step) --------------------------------------------
_FER: dict = {}


def ferrule_limits(s: float, th: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """(slide limits m, hinge limits rad) for a joint at slide ``s`` and angle ``th``. Pure Python.

    Spigot out of the ferrule: the hinge is free to the rim stops, and the slide may enter only when the joint is
    within ENTRY_DEG of straight; otherwise the spigot's tip rests on the rim, at the mouth or (bent past about
    20 deg) just outside it (rim_offset). Spigot in: the slide is free and the hinge is held straight, ENTRY_DEG at
    the mouth narrowing to LOCK_DEG once LOCK_DEPTH is engaged."""
    e = D - s
    if e > ENGAGED:
        lim = ENTRY_DEG - (ENTRY_DEG - LOCK_DEG) * min(e, LOCK_DEPTH) / LOCK_DEPTH
        return (0.0, S_MAX), (-math.radians(lim), math.radians(lim))
    fold = math.radians(FOLD_DEG)
    return ((0.0, S_MAX) if abs(th) <= math.radians(ENTRY_DEG) else (D + rim_offset(th), S_MAX)), (-fold, fold)


def cord_force(s: float) -> float:
    """The cord's pull (N) drawing the spigot in at slide ``s``."""
    return CORD_K * (s - CORD_REST)


def fit_next(mode: str, hold, s: float, v: float):
    """Stick-slip of the snug fit, one control step: (mode, hold) -> (mode, hold). Modes: "free" (outside the fit,
    the cord pulls freely), "stuck" (held at ``hold``), "in" / "out" (sliding against FIT_DYNAMIC). ``v`` is the
    slide's speed (m/s, + = out). Pure Python."""
    if s >= FIT_LEN:
        return "free", None
    if mode == "stuck":
        push = FIT_HOLD_K * (hold - s)  # what the stuck slide carries: + pushed in, - pulled out
        if push + cord_force(hold) > FIT_STATIC:
            return "in", None
        if -push - cord_force(hold) > FIT_STATIC:
            return "out", None
        return "stuck", hold
    if mode == "in":
        return ("stuck", s) if v > -STICK_V else ("in", None)
    if mode == "out":
        return ("stuck", s) if v < STICK_V else ("out", None)
    return ("in", None) if v < -STICK_V else ("stuck", s)  # just came into the fit


@lk.on_reset
def _ferrule_reset(tick):
    _FER.clear()


def _set_fit(i: int, mode: str) -> None:
    j = f"s{i}"
    if mode == "stuck":
        lk.brake("mast", j, True)
        return
    lk.set_rest("mast", j, {"in": REST_IN, "out": REST_OUT}.get(mode, CORD_REST))
    if lk.braked("mast", j):
        lk.brake("mast", j, False)


@lk.on_step
def _ferrule(tick):
    st = tick.states["mast"]
    for i in (1, 2):
        s = st.q[f"s{i}"]
        mode, hold = _FER.get(("fit", i), ("free", None))
        new = fit_next(mode, hold, s, st.qd[f"s{i}"])
        if new[0] != mode:
            _set_fit(i, new[0])
        _FER[("fit", i)] = new
        sl, hl = ferrule_limits(s, st.q[f"f{i}"])
        old = _FER.get(("s", i))
        if old is None or abs(old[0] - sl[0]) > 1e-4:
            lk.set_limits("mast", f"s{i}", *sl)
            _FER[("s", i)] = sl
        old = _FER.get(("f", i))
        if old is None or abs(old[1] - hl[1]) > 2e-4:
            lk.set_limits("mast", f"f{i}", *hl)
            _FER[("f", i)] = hl


# -- measurement and whole-run record ---------------------------------------------------------------------------------
SEAT_MM = 1.0  # a joint is fully seated with at most this gap at the ferrule mouth
STRAIGHT_DEG = 1.0
SLOT_MM = 6.0  # mast centreline within this of a slot's centre
DZ_LO, DZ_HI = -2.0, 4.5  # mast bottom above the slot floor (mm); a square tube resting on an edge sits 3.7 mm up
OVER_MM = 12.0  # the nearest point of the mast's centreline lies over the stand (along the slot)
STILL = 0.005  # m/s
CLEAR_M = 0.030
TCP_STILL = 0.005
REST_S = 1.0
RELEASE_S = 1.0


def section_centres(st) -> np.ndarray:
    return np.array([st.point(n, (SEG / 2, 0.0, 0.0)) for n in SECTIONS])


def support(st) -> list[tuple[float, float, float]]:
    """Per stand: (dx, dz, dy) mm. dx: the mast centreline's offset across the slot; dz: the tube bottom above the
    slot floor; dy: the nearest centreline point's offset along the slot from the stand's centre."""
    out = []
    for y in STAND_Y:
        p = np.array([X_ST, y, TOP + FLOOR + TUBE / 2])
        best = None
        for n in SECTIONS:
            m = st.poses[n]
            loc = m[:3, :3].T @ (p - m[:3, 3])
            c = m[:3, :3] @ np.array([min(max(loc[0], 0.0), SEG), 0.0, 0.0]) + m[:3, 3]
            d = float(np.linalg.norm(c - p))
            if best is None or d < best[0]:
                best = (d, c)
        c = best[1]
        out.append((1000 * (c[0] - X_ST), 1000 * (c[2] - TUBE / 2 - TOP - FLOOR), 1000 * (c[1] - y)))
    return out


def measure(st, tools) -> dict:
    """One sample of the scene (pure Python on a LinkState and gripper Tools)."""
    return {
        "s": tuple(1000 * st.q[f"s{i}"] for i in (1, 2)),
        "th": tuple(math.degrees(st.q[f"f{i}"]) for i in (1, 2)),
        "support": support(st),
        "speed": 0.0,
        "clear": {k: lk.clearance(t, st) for k, t in tools.items()},
        "tcp_speed": {k: t.speed for k, t in tools.items()},
    }


def seated(m, i: int) -> bool:
    return m["s"][i] <= SEAT_MM + EPS and abs(m["th"][i]) <= STRAIGHT_DEG + EPS


def on_stands(m) -> bool:
    return all(abs(dx) <= SLOT_MM + EPS and DZ_LO - EPS <= dz <= DZ_HI + EPS and abs(dy) <= OVER_MM + EPS
               for dx, dz, dy in m["support"])


def assembled_on_stands(m) -> bool:
    return seated(m, 0) and seated(m, 1) and on_stands(m)


class Record:
    """Whole-run record: a linkkit History updated every control step and cleared only by the simulator reset, plus
    the latest sample. Pure Python."""

    def __init__(self):
        self.H = lk.History()
        self.last: dict | None = None
        self.n = 0
        self.trace: list = []

    def reset(self) -> None:
        self.H.reset()
        self.last, self.n, self.trace = None, 0, []

    def update(self, m: dict, t: float | None = None) -> None:
        """Feed one control step. ``t`` is only for offline tests (in the simulator the kit advances the clock)."""
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        H.ever("one_seated", seated(m, 0) or seated(m, 1))
        H.ever("both_seated", seated(m, 0) and seated(m, 1))
        for i in (0, 1):
            H.low(f"s{i + 1}_min", m["s"][i])
            H.peak(f"s{i + 1}_max", m["s"][i])
        H.held("resting", assembled_on_stands(m) and m["speed"] < STILL)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Score and the reason, from the top rung down."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if assembled_on_stands(m) and m["speed"] < STILL:
        ends = {"resting on the stands 1 s": H.runs.get("resting", 0.0) >= REST_S - EPS,
                "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS}
        if all(ends.values()):
            return 1.0, "both joints fully seated, the mast resting in both stands, both grippers let go"
        return 0.75, "assembled and on the stands; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if "both_seated" in H.evers:
        return 0.5, "both joints were fully seated together at some time, but the mast is not assembled on the stands"
    if "one_seated" in H.evers:
        return 0.25, "one joint was fully seated at some time"
    return 0.0, (f"no joint was ever fully seated (closest: joint 1 {H.lows.get('s1_min', 99):.1f} mm, joint 2 "
                 f"{H.lows.get('s2_min', 99):.1f} mm from home; seated is {SEAT_MM} mm)")


REC = Record()
_PREV: dict = {}
CTX: list = []  # the demo context, for diagnostics only (arm command vs measured TCP)


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    st = tick.states["mast"]
    m = measure(st, tick.tools)
    c = section_centres(st)
    if _PREV and tick.t > _PREV["t"]:
        m["speed"] = float(np.max(np.linalg.norm(c - _PREV["c"], axis=1))) / (tick.t - _PREV["t"])
    _PREV.update(c=c, t=tick.t)
    REC.update(m)
    if REC.n % 10 == 0:  # diagnostics every 0.5 s
        REC.trace.append((round(tick.t, 2), [round(v, 1) for v in m["s"]], [round(v, 1) for v in m["th"]],
                          [[round(v, 1) for v in s] for s in m["support"]], round(1000 * m["speed"], 1),
                          [round(1000 * m["clear"][k], 0) for k in ("left", "right")],
                          [round(1000 * tick.tools[k].opening, 1) for k in ("left", "right")],
                          [round(1000 * float(np.linalg.norm(tick.tools[k].tcp - CTX[0].arm(k).pos)), 1)
                           for k in ("left", "right")] if CTX else None))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
# linkdemo.py runs demo_script(ctx). It reads only what a camera could measure (the three sections' poses, from which
# the joint angles and the gaps at the ferrule mouths follow) and the robot's own grippers; never joint values, the
# hidden cord anchors or the record.

VARIANTS = ("", "nothing", "pull_straight", "align_only", "one_arm", "tap")
LOG: dict = {}
GRIP_OPEN, GRIP_CLOSED = 0.070, 0.010  # the tube is 18 mm across: 8 mm squeeze
TCP_ABOVE = 0.012  # TCP above the tube's centre: pad tips 1 mm below the centre, 8 mm above the table
HOVER = 0.06
OVERPUSH = 0.004
TAP_OVER = 0.010  # tap cheat: each strike commands 10 mm past home in 0.1 s
LIFT = 0.060  # carried with its bottom 8 mm above the stands' cheeks
DROP = 0.001  # the mast is let go this far above the slot floors


def _h(v) -> np.ndarray:
    v = np.array([v[0], v[1], 0.0], dtype=float)
    return v / np.linalg.norm(v)


def _rz(deg: float) -> np.ndarray:
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _signed(a, b) -> float:
    return math.degrees(math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1]))


def _top_down(closing) -> np.ndarray:
    z = np.array([0.0, 0.0, -1.0])
    x = _h(closing)
    return np.column_stack([x, np.cross(z, x), z])


class Mast:
    """What the cameras give: section poses, their axes, joint angles and the gaps at the ferrule mouths."""

    def __init__(self, seen):
        self.seen = seen

    def axis(self, n):
        return _h(self.seen.pose("mast", n)[:3, 0])

    def pt(self, n, x):
        return self.seen.point("mast", n, (x, 0.0, 0.0))

    def angle(self, i):  # joint i (1, 2): child's direction from the parent's, deg about +z
        lo, hi = SECTIONS[i - 1], SECTIONS[i]
        return _signed(self.axis(lo), self.axis(hi))

    def gap(self, i):  # child's shoulder past the parent's ferrule mouth, along the parent (m)
        lo, hi = SECTIONS[i - 1], SECTIONS[i]
        return float((self.pt(hi, 0.0) - self.pt(lo, SEG)) @ self.axis(lo))

    def pivot(self, i):  # the child's spigot tip, where the cord comes out
        return self.pt(SECTIONS[i], -D)


def _pick(arm, pos, closing, *, turn=0.0, centre=None, extra=()) -> np.ndarray:
    """Of the two equivalent top-down grips (pads closing along +-``closing``), the one the arm reaches best at
    ``pos``, along a turn of ``turn`` deg about the vertical through ``centre`` and at the ``extra`` positions."""
    from yam_arena import ik

    best = None
    for sgn in (1.0, -1.0):
        r0 = _top_down(sgn * np.asarray(closing, dtype=float))
        q, worst = arm.q.copy(), 0.0
        pts = [(np.asarray(pos, float), r0)]
        if centre is not None:
            c = np.asarray(centre, float)
            for f in (0.25, 0.5, 0.75, 1.0):
                R = _rz(turn * f)
                pts.append((c + R @ (np.asarray(pos, float) - c), R @ r0))
        pts += [(np.asarray(p, float), r0) for p in extra]
        for p, r in pts:
            q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(r), q, base_pos=arm.base, iters=60)
            worst = max(worst, 1000 * pe + math.degrees(re))
        if best is None or worst < best[0]:
            best = (worst, r0)
    return best[1]


def _tcp(ctx, arm) -> np.ndarray:
    return np.asarray(ctx.observe().tools[arm.side].tcp, dtype=float)


def _grasp(ctx, arm, where, *, turn=None, extra=(), tag=""):
    """Top-down grasp of a tube, pads across it. ``where(mast) -> (point on its centre line, its axis)`` is
    re-read from a fresh look just before the descent, in case the mast moved. ``turn(mast) -> (deg, centre)`` is the
    swing that follows (for choosing between the two equivalent grips). The YAM's soft wrist leaves the TCP 10 to
    20 mm off its command, so the arm also measures its own TCP in free air above the tube and again just before
    closing, and offsets the command (servo checks on the robot's own state)."""
    mast = Mast(ctx.observe())
    point, along = where(mast)
    deg, centre = turn(mast) if turn else (0.0, None)
    closing = np.cross([0.0, 0.0, 1.0], _h(along))
    p = np.asarray(point, float) + [0.0, 0.0, TCP_ABOVE]
    r = _pick(arm, p, closing, turn=deg, centre=centre, extra=extra)
    yield from arm.move(p + [0.0, 0.0, HOVER], r, width=GRIP_OPEN, duration=1.5)
    yield from arm.wait(0.4)
    point, along = where(Mast(ctx.observe()))  # fresh look
    p = np.asarray(point, float) + [0.0, 0.0, TCP_ABOVE]
    r2 = _top_down(np.cross([0.0, 0.0, 1.0], _h(along)))
    flip = _rz(180.0) @ r2
    r = r2 if np.trace(r.T @ r2) >= np.trace(r.T @ flip) else flip
    corr = arm.pos - _tcp(ctx, arm)
    arm.sag = corr  # the command offset that holds the TCP still in free air near here
    yield from arm.move(p + [0.0, 0.0, HOVER] + corr, r, duration=0.5)
    yield from arm.move(p + corr, r, duration=1.2)
    yield from arm.wait(0.5)
    err = p - _tcp(ctx, arm)
    err[2] = 0.0
    yield from arm.move(arm.pos + err, r, duration=0.4)
    yield from arm.wait(0.2)
    LOG.setdefault("grasp_err_mm", []).append((tag, [round(1000 * v, 1) for v in (p - _tcp(ctx, arm))]))
    yield from arm.grip(GRIP_CLOSED, 0.6)
    LOG.setdefault("grasps", []).append((tag, round(1000 * ctx.observe().tools[arm.side].opening, 1)))


def _relax(ctx, arm):
    """Bring the command to where the TCP actually is (plus the free-air sag offset measured at the last grasp), so a
    pushing or holding arm stores no force before it lets go."""
    yield from arm.move(_tcp(ctx, arm) + getattr(arm, "sag", np.zeros(3)), duration=0.6)
    yield from arm.wait(0.2)


def _letgo(ctx, arm, up=HOVER):
    yield from _relax(ctx, arm)
    yield from arm.grip(GRIP_OPEN, 0.5)
    yield from arm.move(arm.pos + [0.0, 0.0, up], duration=0.8)


def _straighten(ctx, arm, i, key):
    """Swing joint i's upper section about its spigot tip until it lines up with the lower one (closed loop)."""
    mast = Mast(ctx.observe())
    a0 = mast.angle(i)
    yield from arm.arc(mast.pivot(i), (0.0, 0.0, 1.0), -a0, follow=1.0, duration=max(1.5, abs(a0) / 18.0))
    log = [round(a0, 2)]
    for _ in range(3):
        yield from arm.wait(0.3)
        mast = Mast(ctx.observe())
        a = mast.angle(i)
        log.append(round(a, 2))
        if abs(a) <= 0.4:
            break
        yield from arm.arc(mast.pivot(i), (0.0, 0.0, 1.0), -a, follow=1.0, duration=0.8)
    LOG[key] = log


def _push_home(ctx, arm, i, key):
    """Push joint i's upper section along the lower one's axis until the gap at the ferrule mouth is gone. Each try
    adds the gap seen plus OVERPUSH to the command: both soft arms give way about 5 mm per newton."""
    log = []
    for _ in range(6):
        mast = Mast(ctx.observe())
        g = mast.gap(i)
        log.append(round(1000 * g, 2))
        if g <= 0.0006:
            break
        d = -mast.axis(SECTIONS[i - 1])
        if VARIANT == "tap":  # cheat: a fast strike, then ease back off, with nothing holding the other part
            yield from arm.move(arm.pos + d * (g + TAP_OVER), duration=0.1)
            yield from arm.wait(0.2)
            yield from _relax(ctx, arm)
        else:
            yield from arm.move(arm.pos + d * (g + OVERPUSH), duration=max(1.0, g / 0.02))
        yield from arm.wait(0.3)
    log.append(round(1000 * Mast(ctx.observe()).gap(i), 2))
    LOG[key] = log


def _to_stands(ctx, arms):
    """The arm(s) holding the straight mast lift it, turn and carry it over the stands' line and lower it into the
    slots: one rigid move of the grip(s), planned from the mast's seen foot and cap."""
    mast = Mast(ctx.observe())
    foot, cap = mast.pt("base", 0.0), mast.pt("top", SEG)
    c_old = 0.5 * (foot + cap)
    yaw_old = math.degrees(math.atan2(cap[1] - foot[1], cap[0] - foot[0]))
    dyaw = (-90.0 - yaw_old + 180.0) % 360.0 - 180.0
    c_new = np.array([X_ST, float(np.mean(STAND_Y)), TOP + FLOOR + TUBE / 2 + DROP])
    R = _rz(dyaw)
    LOG["carry"] = {"dyaw": round(dyaw, 1), "shift_mm": [round(1000 * v, 1) for v in (c_new - c_old)]}

    def go(arm):
        p0, r0 = arm.pos.copy(), arm.rot.copy()
        p1 = c_new + R @ (p0 - c_old)
        p1[2] = p0[2] + (c_new[2] - c_old[2])
        yield from arm.move(p0 + [0.0, 0.0, LIFT], duration=1.5)
        yield from arm.move(p1 + [0.0, 0.0, LIFT], R @ r0, duration=2.5)
        yield from arm.move(p1, duration=1.5)
        yield from arm.wait(0.4)

    yield from ctx.both(*(go(a) for a in arms))
    LOG["carry"]["tcp_err_mm"] = [round(1000 * float(np.linalg.norm(_tcp(ctx, a) - a.pos)), 1) for a in arms]


def section_centres_seen(mast: Mast) -> np.ndarray:
    return np.array([mast.pt(n, SEG / 2) for n in SECTIONS])


def _carry_extra(point):
    """Positions the holding arm passes on the way to the stands (for choosing its grip)."""
    p = np.asarray(point, float) + [0.0, 0.0, TCP_ABOVE]
    dx, dy = X_ST - X0, float(np.mean(STAND_Y))
    return [p + [0, 0, LIFT], p + [dx, dy, LIFT], p + [dx, dy, FLOOR]]


def demo_report() -> dict:
    H = REC.H
    m = REC.last or {}
    return {"variant": VARIANT, "ladder": ladder(REC), "evers": dict(H.evers), "lows": dict(H.lows),
            "peaks": dict(H.peaks), "runs": dict(H.runs), "last": {k: v for k, v in m.items()}, "seen": dict(LOG),
            "trace": REC.trace}


def demo_script(ctx):
    LOG.clear()
    CTX[:] = [ctx]
    v = VARIANT
    L, R = ctx.left, ctx.right
    yield from ctx.wait(1.0)
    if v == "nothing":
        yield from ctx.wait(8.0)
        return

    if v == "pull_straight":
        # left: the base section near its foot; right: the top section near its cap; pull the mast straight (cords
        # stretched), carry it to the stands and let go
        mast = Mast(ctx.observe())
        ax = mast.axis("base")
        target = mast.pt("base", 0.030) + ax * (2 * SEG + 0.100 - 0.030 + 2 * 0.045)
        target[2] = mast.pt("top", 0.100)[2] + TCP_ABOVE
        yield from ctx.both(_grasp(ctx, L, lambda m: (m.pt("base", 0.030), m.axis("base")),
                                   extra=_carry_extra(mast.pt("base", 0.030))),
                            _grasp(ctx, R, lambda m: (m.pt("top", 0.100), m.axis("top")), extra=[target]))
        mid = R.pos + 0.5 * (target - R.pos) + [0.0, -0.05, 0.0]
        yield from R.move(mid, duration=2.0)
        yield from R.move(target, duration=2.0)
        yield from ctx.wait(0.5)
        m2 = Mast(ctx.observe())
        LOG["pulled"] = {"angles": [round(m2.angle(i), 1) for i in (1, 2)],
                         "gaps_mm": [round(1000 * m2.gap(i), 1) for i in (1, 2)]}
        yield from _to_stands(ctx, (L, R))
        yield from ctx.both(_letgo(ctx, L, 0.08), _letgo(ctx, R, 0.08))
        yield from ctx.wait(3.0)
        return

    one = v in ("one_arm", "tap")
    push = v != "align_only"

    # 1. Joint 2: the left arm holds the middle section near joint 1, the right swings the top section in line and
    # pushes it home.
    gens = [_grasp(ctx, R, lambda m: (m.pt("top", 0.050), m.axis("top")),
                   turn=lambda m: (-m.angle(2), m.pivot(2)), tag="right:top")]
    if not one:
        gens.append(_grasp(ctx, L, lambda m: (m.pt("mid", 0.040), m.axis("mid")), tag="left:mid"))
    yield from ctx.both(*gens)
    yield from _straighten(ctx, R, 2, "j2_angles")
    if push:
        yield from _push_home(ctx, R, 2, "j2_gaps_mm")
    yield from _letgo(ctx, R)
    if not one:  # 2. Joint 1: the left moves to the base section near its foot (the mast settles first)
        yield from _letgo(ctx, L)
        yield from ctx.wait(0.2)
    gens = [_grasp(ctx, R, lambda m: (m.pt("mid", 0.050), m.axis("mid")),
                   turn=lambda m: (-m.angle(1), m.pivot(1)), tag="right:mid")]
    if not one:
        gens.append(_grasp(ctx, L, lambda m: (m.pt("base", 0.035), m.axis("base")),
                           extra=_carry_extra(Mast(ctx.observe()).pt("base", 0.035)), tag="left:base"))
    yield from ctx.both(*gens)
    yield from _straighten(ctx, R, 1, "j1_angles")
    if push:
        yield from _push_home(ctx, R, 1, "j1_gaps_mm")

    # 3. Lay it across the stands. The right holds the middle section 15 mm from the mast's centre and carries it
    # alone, so no pull between two grips can open a joint (one_arm: it regrasps the middle at its centre).
    mast = Mast(ctx.observe())
    LOG["assembled"] = {"angles": [round(mast.angle(i), 2) for i in (1, 2)],
                        "gaps_mm": [round(1000 * mast.gap(i), 2) for i in (1, 2)]}
    if one:
        yield from _letgo(ctx, R)
        g = mast.pt("mid", SEG / 2)
        yield from _grasp(ctx, R, lambda m: (m.pt("mid", SEG / 2), m.axis("mid")), extra=_carry_extra(g),
                          tag="right:mid-carry")
        yield from _to_stands(ctx, (R,))
        yield from _letgo(ctx, R, 0.08)
    else:  # the pusher eases off first, then the left lets go of the base section; the right carries it alone
        yield from _relax(ctx, R)
        yield from _letgo(ctx, L, 0.08)
        yield from _to_stands(ctx, (R,))
        yield from _letgo(ctx, R, 0.08)
    m = Mast(ctx.observe())
    LOG["on_stands"] = {"angles": [round(m.angle(i), 2) for i in (1, 2)],
                        "gaps_mm": [round(1000 * m.gap(i), 2) for i in (1, 2)]}
    yield from ctx.wait(3.0)
