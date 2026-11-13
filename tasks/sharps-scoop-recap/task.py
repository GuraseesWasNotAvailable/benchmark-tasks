# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Recap a used practice syringe with the one-handed scoop, the cap lying on its tray, the other hand kept clear.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A grey ABS cap tray (fixed) stands 20 mm high at x 0.150. The cap lies on it between two 5 mm rails, its mouth
  toward the robot's right (toward -y), its closed end 1 mm from a 14 mm back stop. Mouth centre at (0.150, -0.045).
- The cap is a translucent pink 18 G needle cap (polypropylene, 3 g in the simulator): an octagonal tube, 7.6 mm across the bore
  flats, 1.0 mm walls, 49 mm bore, closed at the far end.
- A used 3 mL practice syringe with an 18 G x 38 mm needle (6.5 g, clear barrel 9.5 mm across, luer hub 6.6 mm)
  lies in a rest 0.11 m to the right of the tray, needle toward +y, tip at y -0.107. Its barrel is bridged over a
  28 mm slot so a gripper can reach the barrel's underside.
- A second tray block carries the barrel's rear end, so a capped syringe can rest on the tray, level, once let go.

Goal: the cap seated on the needle (the barrel's front shoulder within 3 mm of the cap's mouth, axis within 8 deg);
the whole run done with one hand, the other kept out of a taped zone and off the cap; both grippers let go.
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
MM = 0.001

# -- layout ----------------------------------------------------------------------------------------------------------
TRAY_H = 0.020  # tray and rest height above the table
CAP_X, CAP_MY = 0.150, -0.045  # cap mouth centre (x, y); the cap's axis (u) runs along world +y
ZC = TOP + TRAY_H + 0.00475  # barrel and cap axis height
REST_X, REST_SY = 0.260, -0.155  # syringe rest: x and the barrel's front shoulder y
PINK = (0.95, 0.55, 0.70)
GREY = (0.55, 0.57, 0.60)
CLEAR = (0.85, 0.88, 0.90)

# cap: octagonal tube along its local +x (mouth at x 0)
BORE_A = 0.0038  # apothem of the bore (7.6 mm across flats)
WALL = 0.0010
CAP_L = 0.049  # bore length
CAP_MASS = 0.0030
_cap_shapes = []
for k in range(8):
    th = 45.0 * k
    d = BORE_A + WALL / 2
    wid = 2 * (BORE_A + WALL) * math.tan(math.radians(22.5)) + 0.0004
    _cap_shapes.append(lk.box((CAP_L, wid, WALL), (CAP_L / 2, d * math.cos(math.radians(th)),
                                                   d * math.sin(math.radians(th))), rpy_deg=(th - 90.0, 0, 0),
                              contact_offset=0.0005))
_cap_shapes.append(lk.box((0.0015, 0.0098, 0.0098), (CAP_L + 0.00075, 0, 0), contact_offset=0.0005))
_t = lk.Tree("cap", pos=(CAP_X, CAP_MY, ZC + 0.0004), yaw_deg=90.0, color=PINK)
_t.link("cap", _cap_shapes, mass=CAP_MASS, friction=(0.35, 0.3), contact_offset=0.0005)
CAP = _t.build()

# syringe: link frame origin at the barrel's front shoulder, +x toward the needle tip
BARREL_R, BARREL_L = 0.00475, 0.062
HUB_R, HUB_L = 0.0033, 0.010
NEEDLE_R, NEEDLE_L = 0.000635, 0.038
TIP = HUB_L + NEEDLE_L  # 48 mm
SYR_MASS = 0.0065
_t = lk.Tree("syringe", pos=(REST_X, REST_SY, ZC + 0.0003), yaw_deg=90.0, color=CLEAR)
_t.link("syringe", [
    lk.cylinder(BARREL_R, BARREL_L, (-BARREL_L / 2, 0, 0), axis="X", contact_offset=0.0005),
    lk.box((0.002, 0.024, 0.0095), (-BARREL_L - 0.001, 0, 0), contact_offset=0.0005),  # finger flange
    lk.cylinder(HUB_R, HUB_L, (HUB_L / 2, 0, 0), axis="X", contact_offset=0.0005),
    lk.cylinder(NEEDLE_R, NEEDLE_L, (HUB_L + NEEDLE_L / 2, 0, 0), axis="X", color=(0.75, 0.75, 0.78),
                contact_offset=0.0003),
], mass=SYR_MASS, friction=(0.5, 0.4), contact_offset=0.0005)
SYRINGE = _t.build()

# cap tray (fixed), frame: origin at the cap mouth centre on the table, +x = u toward the cap's closed end (world +y)
_t = lk.Tree("tray", pos=(CAP_X, CAP_MY, TOP), yaw_deg=90.0, fixed=True, color=GREY)
_t.link("tray", [
    lk.box((0.070, 0.070, TRAY_H), (0.019, 0, TRAY_H / 2)),  # tray block, u -16..54, lateral +-35
    lk.box((0.020, 0.070, TRAY_H), (-0.054, 0, TRAY_H / 2)),  # barrel rear block, u -64..-44
    lk.box((0.049, 0.002, 0.005), (0.0245, 0.0064, TRAY_H + 0.0025)),  # cap rails
    lk.box((0.049, 0.002, 0.005), (0.0245, -0.0064, TRAY_H + 0.0025)),
    lk.box((0.004, 0.020, 0.014), (0.053, 0, TRAY_H + 0.007)),  # back stop, u 51..55
], mass=0.5, friction=(0.45, 0.4), contact_offset=0.0005)
TRAY = _t.build()
# syringe rest (fixed): two blocks with lips either side of the barrel, a 28 mm slot between them
_t = lk.Tree("rest", pos=(REST_X, REST_SY, TOP), yaw_deg=90.0, fixed=True, color=GREY)
_rest = []
for u0, u1 in ((-0.064, -0.044), (-0.016, 0.004)):
    c, w = (u0 + u1) / 2, u1 - u0
    _rest.append(lk.box((w, 0.040, TRAY_H), (c, 0, TRAY_H / 2)))
    for s in (1, -1):
        _rest.append(lk.box((w, 0.002, 0.006), (c, s * 0.0062, TRAY_H + 0.003)))
_t.link("rest", _rest, mass=0.5, friction=(0.45, 0.4), contact_offset=0.0005)
REST = _t.build()

OBJECTS = [TRAY, REST, CAP, SYRINGE]

# -- scoring ---------------------------------------------------------------------------------------------------------
ZONE = (0.05, 0.36, -0.30, 0.03)  # taped zone x0, x1, y0, y1 (m): the off hand's pads must never be over it
CAP_TOUCH_M = 0.003
HELD_OPENING, HELD_NEAR = 0.060, 0.004
ENTER_MM, DEEP_MM, DEEP_DEG = 5.0, 30.0, 15.0
SEAT_GAP_MM, SEAT_LAT_MM, SEAT_DEG = 3.0, 3.0, 8.0
BORE_LAT_MM = 3.5
STEADY_S, RELEASE_S = 2.0, 1.0
STILL_V, STILL_W = 0.003, math.radians(3.0)
CLEAR_M, TCP_STILL = 0.030, 0.005
EPS = 1e-6


def pose_rates(A_prev, A, dt: float) -> tuple[float, float]:
    """Speed (m/s) and spin (rad/s) of a body from its pose change over ``dt`` (what a camera sees)."""
    if dt <= 0.0:
        return 0.0, 0.0
    v = float(np.linalg.norm(A[:3, 3] - A_prev[:3, 3])) / dt
    R = A_prev[:3, :3].T @ A[:3, :3]
    sin = 0.5 * float(np.linalg.norm((R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1])))
    return v, math.atan2(sin, (float(np.trace(R)) - 1.0) / 2.0) / dt


def in_zone(p) -> bool:
    return ZONE[0] <= p[0] <= ZONE[1] and ZONE[2] <= p[1] <= ZONE[3]


def measure(syr, cap, tools) -> dict:
    """One sample: needle and shoulder in the cap frame (mm, deg), who holds the syringe, pad clearances."""
    S, C = syr.poses["syringe"], cap.poses["cap"]
    tip = C[:3, :3].T @ (S[:3, :3] @ np.array([TIP, 0.0, 0.0]) + S[:3, 3] - C[:3, 3])
    sh = C[:3, :3].T @ (S[:3, 3] - C[:3, 3])
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(S[:3, 0] @ C[:3, 0])))))
    held, near, clear, zone, touch = {}, {}, {}, {}, {}
    for side, tool in tools.items():
        ds, dc = lk.clearance(tool, syr), lk.clearance(tool, cap)
        near[side] = ds
        held[side] = tool.opening <= HELD_OPENING and ds <= HELD_NEAR
        clear[side] = min(ds, dc)
        zone[side] = any(in_zone(p) for p in tool.pad_points())
        touch[side] = dc <= CAP_TOUCH_M
    return {
        "depth": float(tip[0]) * 1000.0, "tip_lat": float(math.hypot(tip[1], tip[2])) * 1000.0,
        "gap": float(sh[0]) * 1000.0, "sh_lat": float(math.hypot(sh[1], sh[2])) * 1000.0, "ang": ang,
        "held": held, "zone": zone, "touch": touch, "clear": clear,
        "tcp_speed": {s: float(t.speed) for s, t in tools.items()},
        "sy_speed": float(np.linalg.norm(syr.lin_vels["syringe"])), "sy_spin": float(np.linalg.norm(syr.ang_vels["syringe"])),
    }


def in_bore(m) -> bool:
    return m["tip_lat"] <= BORE_LAT_MM and 0.0 <= m["depth"] <= CAP_L * 1000.0


def seated(m) -> bool:
    return (in_bore(m) and abs(m["gap"]) <= SEAT_GAP_MM and m["sh_lat"] <= SEAT_LAT_MM and m["ang"] <= SEAT_DEG)


class Record:
    """Whole-run record: a linkkit History updated every control step, cleared only by the simulator reset."""

    def __init__(self):
        self.H = lk.History()
        self.holder: str | None = None  # the arm that first held the syringe
        self.last: dict | None = None
        self.n = 0
        self.trace: list = []

    def reset(self) -> None:
        self.H.reset()
        self.holder, self.last, self.n, self.trace = None, None, 0, []

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        if self.holder is None:
            for s, h in m["held"].items():
                if h:
                    self.holder = s
                    break
        off = [s for s in m["zone"] if self.holder is not None and s != self.holder]
        H.ever("off_hand", any(m["zone"][s] for s in off))
        H.ever("cap_touched", any(m["touch"].values()))
        held = any(m["held"].values())
        H.peak("depth_max", m["depth"] if m["tip_lat"] <= BORE_LAT_MM else 0.0)
        H.ever("entered", held and in_bore(m) and m["depth"] >= ENTER_MM)
        H.ever("deep", held and in_bore(m) and m["depth"] >= DEEP_MM and m["ang"] <= DEEP_DEG)
        H.held("steady", seated(m) and m["sy_speed"] < STILL_V and m["sy_spin"] < STILL_W)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "off_hand" in H.evers:
        return 0.0, f"gate: the off hand's pads were over the taped zone at {H.evers['off_hand']:.2f} s"
    if "cap_touched" in H.evers:
        return 0.0, f"gate: a gripper pad touched the cap at {H.evers['cap_touched']:.2f} s"
    if seated(m):
        ends = {
            "syringe steady 2 s": H.runs.get("steady", 0.0) >= STEADY_S - EPS,
            "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS,
        }
        if all(ends.values()):
            return 1.0, "cap seated on the needle, syringe still, both grippers let go"
        return 0.75, "cap seated at the end; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if "deep" in H.evers:
        return 0.5, "the held needle went 30 mm into the cap at some point, but the cap is not seated at the end"
    if "entered" in H.evers:
        return 0.25, "the held needle's tip went 5 mm into the cap, no deeper"
    return 0.0, "the held needle never entered the cap"


REC = Record()
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    m = measure(tick.states["syringe"], tick.states["cap"], tick.tools)
    A = np.array(tick.states["syringe"].poses["syringe"], dtype=float)
    if _PREV:
        m["sy_speed"], m["sy_spin"] = pose_rates(_PREV["A"], A, tick.t - _PREV["t"])
    _PREV.update(A=A, t=tick.t)
    REC.update(m)
    if REC.n % 5 == 0:
        REC.trace.append((round(tick.t, 2), round(m["depth"], 1), round(m["tip_lat"], 1), round(m["gap"], 1),
                          round(m["ang"], 1), {s: int(v) for s, v in m["held"].items()},
                          {s: round(1000 * v) for s, v in m["clear"].items()}, {s: int(v) for s, v in m["zone"].items()}))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Reads only what a camera could measure: the syringe and cap poses and the robot's own grippers.

VARIANTS = ("", "nothing", "left_hovers", "left_pins_cap", "drop_on_cap")
LOG: dict = {}
CONTACT = 0.009  # pad contact centre below the TCP
GRIP_OPEN, GRIP_CLOSED = 0.060, 0.0065
GRASP_U = -0.030  # grasp point along the barrel from the shoulder
# (ty, tz, lat) tip target in the cap frame (m) and needle pitch, tip down (deg)
SCOOP = (
    (-0.045, 0.030, 0.0, 20.0),
    (-0.012, 0.0025, 0.0, 20.0),
    (0.000, 0.0010, 0.0, 18.0),
    (0.010, -0.0010, 0.0, 14.0),
    (0.020, -0.0010, 0.0, 8.0),
    (0.030, 0.0000, 0.0, 3.0),
    (0.040, 0.0000, 0.0, 0.0),
)
PUSH_END = 0.0495
TOP_DOWN_X = np.array([1.0, 0.0, 0.0])


def _smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def pose_for(C, ty, tz, lat, phi_deg):
    """World syringe pose with its needle tip at (ty, lat, tz) in the cap frame ``C`` and pitched ``phi`` tip down."""
    p = math.radians(phi_deg)
    u, cy, cz = C[:3, 0], C[:3, 1], C[:3, 2]
    n = math.cos(p) * u - math.sin(p) * cz
    zs = math.sin(p) * u + math.cos(p) * cz
    ys = np.cross(zs, n)
    tipw = C[:3, 3] + ty * u + lat * cy + tz * cz
    S = np.eye(4)
    S[:3, :3] = np.column_stack([n, ys, zs])
    S[:3, 3] = tipw - TIP * n
    return S


class Hand:
    """The right arm carrying the syringe: it commands the TCP so the seen syringe follows a planned pose."""

    def __init__(self, ctx):
        self.ctx, self.arm = ctx, ctx.right
        self.G = None
        self.dp = np.zeros(3)
        self.goal = np.zeros(3)
        self.width = GRIP_CLOSED
        self.log: list = []

    def tcp(self, S):
        T = S @ self.G
        return T[:3, 3] + self.dp, T[:3, :3]

    def follow(self, fn, seconds):
        def at(s):
            self.dp += 0.15 * (self.goal - self.dp)
            p, r = self.tcp(fn(s))
            return p, r, self.width

        yield from self.arm.follow(at, seconds)

    def seen(self):
        return self.ctx.observe().pose("syringe", "syringe")

    def correct(self, S_des, gain=0.8, max_mm=8.0):
        e = S_des[:3, 3] - self.seen()[:3, 3]
        self.goal = np.clip(self.goal + gain * e, -max_mm * MM, max_mm * MM)
        return 1000 * float(np.linalg.norm(e))

    def servo(self, S_des, tag, tries=3, tol_mm=0.4):
        for k in range(tries):
            yield from self.arm.wait(0.3)
            e = self.correct(S_des)
            self.log.append((tag, k, round(e, 2)))
            if e <= tol_mm:
                break
            self.dp = self.goal.copy()
            p, r = self.tcp(S_des)
            yield from self.arm.move(p, r, width=self.width, duration=0.5)


def lerp_pose(a, b, s):
    from linkdemo import _slerp

    M = np.eye(4)
    M[:3, :3] = _slerp(a[:3, :3], b[:3, :3], s)
    M[:3, 3] = a[:3, 3] + s * (b[:3, 3] - a[:3, 3])
    return M


def right_pick(ctx, hand, st):
    from linkdemo import grip_frame

    arm = ctx.right
    S0 = hand.seen()
    P = S0[:3, :3] @ np.array([GRASP_U, 0.0, 0.0]) + S0[:3, 3]
    r = grip_frame(np.array([0.0, 0.0, -1.0]), TOP_DOWN_X)
    tcp = P + np.array([0.0, 0.0, CONTACT])
    yield from arm.move(tcp + np.array([0.0, 0.0, 0.07]), r, width=GRIP_OPEN, duration=2.0)
    yield from arm.move(tcp, r, duration=1.2)
    yield from arm.grip(GRIP_CLOSED, 0.8)
    yield from arm.wait(0.4)
    S = hand.seen()
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = arm.rot, arm.pos
    hand.G = np.linalg.inv(S) @ T
    st["grasp_mm"] = [round(1000 * v, 2) for v in (hand.G[:3, 3])]
    lift = S.copy()
    lift[2, 3] += 0.05
    p, r = hand.tcp(lift)
    yield from arm.move(p, r, duration=1.0)


def right_scoop(ctx, hand, st, short=False):
    arm = ctx.right
    C = ctx.observe().pose("cap", "cap")
    st["cap_seen"] = [round(float(v), 4) for v in C[:3, 3]]
    pts = [pose_for(C, *w) for w in SCOOP]
    cur = hand.seen()
    yield from hand.follow(lambda s: lerp_pose(cur, pts[0], _smooth(s)), 3.0)
    yield from hand.servo(pts[0], "above")
    yield from hand.follow(lambda s: lerp_pose(pts[0], pts[1], _smooth(s)), 2.0)
    yield from hand.servo(pts[1], "mouth", tries=4, tol_mm=0.4)
    st["t_mouth"] = round(ctx.t, 2)
    for a, b, secs in zip(range(1, len(pts) - 1), range(2, len(pts)), (1.5, 1.5, 1.5, 1.5, 1.5)):
        yield from hand.follow((lambda a, b: lambda s: lerp_pose(pts[a], pts[b], s))(a, b), secs)
        C = ctx.observe().pose("cap", "cap")
        pts = [pose_for(C, *w) for w in SCOOP]
        e = hand.correct(pts[b], gain=0.6, max_mm=3.0)
        hand.log.append(("scoop", b, round(e, 2)))
    st["t_level"] = round(ctx.t, 2)
    C = ctx.observe().pose("cap", "cap")
    start = pose_for(C, 0.040, 0.0, 0.0, 0.0)
    end = pose_for(C, PUSH_END, 0.0, 0.0, 0.0)
    yield from hand.follow(lambda s: lerp_pose(start, end, _smooth(s)), 2.5)
    for k in range(6):  # creep on until the seen shoulder is at the mouth (the arm lags a few mm under load)
        yield from arm.wait(0.4)
        seen = ctx.observe()
        S, C = seen.pose("syringe", "syringe"), seen.pose("cap", "cap")
        gap = float((S[:3, 3] - C[:3, 3]) @ C[:3, 0])
        st.setdefault("creep", []).append(round(1000 * gap, 2))
        if gap >= -0.0008:
            break
        hand.goal = hand.goal + 0.9 * (-gap) * C[:3, 0]
        hand.dp = hand.goal.copy()
        p, r = hand.tcp(end)
        yield from arm.move(p, r, width=hand.width, duration=0.8)
    st["t_push"] = round(ctx.t, 2)
    yield from arm.wait(0.5)


def right_release(ctx, st):
    arm = ctx.right
    yield from arm.grip(0.0939, 0.6)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.07]), duration=1.2)


def left_cheat(ctx, st, mode):
    from linkdemo import grip_frame

    arm = ctx.left
    yield from ctx.wait(3.0)
    C = ctx.observe().pose("cap", "cap")
    top = C[:3, 3] + C[:3, :3] @ np.array([0.040, 0.0, 0.0]) + np.array([0.0, 0.0, 0.006])
    r = grip_frame(np.array([0.0, 0.0, -1.0]), TOP_DOWN_X)
    if mode == "hover":
        yield from arm.move(top + np.array([0.0, 0.0, 0.06]) - r @ np.array([0, 0, 0.0]) , r, width=0.0939, duration=3.0)
        yield from arm.wait(20.0)
    else:
        yield from arm.move(top + np.array([0.0, 0.0, 0.06]), r, width=0.0939, duration=3.0)
        yield from arm.move(top + np.array([0.0, 0.0, 0.014]) , r, width=0.0939, duration=2.0)
        yield from arm.grip(0.012, 0.6)
        yield from arm.wait(20.0)


def demo_script(ctx):
    variant = VARIANT
    LOG.clear()
    st = LOG
    st["variant"] = variant
    yield from ctx.wait(1.0)
    if variant == "nothing":
        yield from ctx.wait(8.0)
        return
    hand = Hand(ctx)
    st["hand_log"] = hand.log

    def left():
        if variant == "left_hovers":
            yield from left_cheat(ctx, st, "hover")
        elif variant == "left_pins_cap":
            yield from left_cheat(ctx, st, "pin")

    def right():
        yield from right_pick(ctx, hand, st)
        if variant == "drop_on_cap":
            C = ctx.observe().pose("cap", "cap")
            above = pose_for(C, 0.030, 0.030, 0.0, 0.0)
            cur = hand.seen()
            yield from hand.follow(lambda s: lerp_pose(cur, above, _smooth(s)), 3.0)
            yield from ctx.right.grip(0.0939, 0.4)
            yield from ctx.right.wait(4.0)
            return
        yield from right_scoop(ctx, hand, st)
        yield from right_release(ctx, st)

    yield from ctx.both(left(), right())
    yield from ctx.wait(3.5)


def demo_report() -> dict:
    score_, why = ladder(REC)
    H = REC.H
    m = REC.last or {}
    return {
        "score": score_, "reason": why, "variant": VARIANT, "holder": REC.holder,
        "evers": {k: round(v, 2) for k, v in H.evers.items()},
        "peaks": {k: round(v, 2) for k, v in H.peaks.items()},
        "runs": {k: round(v, 2) for k, v in H.runs.items()},
        "last": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in m.items()},
        "log": {k: v for k, v in LOG.items()},
        "trace": REC.trace,
    }
