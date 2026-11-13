# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Change the blade in a butterfly (twist-to-open) safety razor, holding the blade only by its short ends.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A butterfly safety razor stands in a weighted stand, fixed to the table at (0.24, 0.0): a 70 x 70 x 35 mm dark steel
  base, a 12 mm steel handle up to a 34 x 27 x 4 mm chrome head plate whose top is 130 mm above the table. Two 2.8 mm
  posts, 8 mm either side of the plate's centre along x, stand 1.6 mm proud of it. Two chrome doors (34 x 10.5 x 1.2 mm)
  are hinged along x at y = +-12.5 mm; closed, they lie 0.6 mm above the plate and leave a 4 mm slit over the posts.
  A black 22 mm knurled knob (12 mm tall) at the foot of the handle turns clockwise from above (the spring returns it
  to 0): at 90 deg the doors stand open at 100 deg, so the plate is clear to the blade.
- A double-edge blade (43 x 22 x 0.4 mm steel, 3 g, with two 4.4 mm square holes 8 mm either side of its centre; its
  long sides are the cutting edges, the outer 3 mm) lies on a 26 x 14 mm shelf on a 130 mm post at (0.24, -0.12), its
  short ends overhanging the shelf by 8.5 mm. Its long axis lies along x, as in the razor head.

Goal: the blade lying on the plate with the posts through its holes, the doors closed on it, and both grippers let go,
without a gripper ever touching the blade's long edges.
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

RX, RY = 0.24, 0.0  # razor stand centre
BX, BY = 0.24, -0.12  # blade shelf centre
ZP = 0.130  # head plate top above the table top
BL, BW, BT = 0.043, 0.022, 0.0004  # blade length (x), width (y, the cutting edges at the long sides), thickness
BLADE_Z0 = ZP + BT / 2  # blade centre height, resting on the shelf or the plate
POST_X, POST_H, POST_R = 0.008, 0.0016, 0.0014
HOLE = 0.0044
KNOB_R, KNOB_Z = 0.011, 0.046
KNOB_OPEN, WING_OPEN = 90.0, 100.0  # deg: knob turn that stands the doors open at WING_OPEN
WING_Y, WING_W, WING_T, WING_L = 0.0125, 0.0105, 0.0012, 0.034

STEEL = (0.75, 0.77, 0.80)
DARK = (0.18, 0.19, 0.21)
BLACK = (0.08, 0.08, 0.09)
BLADE_C = (0.86, 0.87, 0.90)
SHELF_C = (0.30, 0.45, 0.65)

# -- scene -----------------------------------------------------------------------------------------------------------
t = lk.Tree("razor", pos=(RX, RY, TOP), fixed=True, color=DARK)
t.link("frame", [lk.box((0.070, 0.070, 0.035), (0, 0, 0.0175)),
                 lk.cylinder(0.006, 0.091, (0, 0, 0.0805)),
                 lk.box((0.034, 0.027, 0.004), (0, 0, ZP - 0.002), color=STEEL),
                 lk.cylinder(POST_R, POST_H, (POST_X, 0, ZP + POST_H / 2), color=STEEL),
                 lk.cylinder(POST_R, POST_H, (-POST_X, 0, ZP + POST_H / 2), color=STEEL)],
       mass=0.8, friction=(0.6, 0.5))
t.link("knob", [lk.cylinder(KNOB_R, 0.012, (0, 0, 0))], mass=0.03, at=(0, 0, KNOB_Z), color=BLACK, friction=(1.0, 0.9))
t.revolute("turn", "frame", "knob", anchor=(0, 0, KNOB_Z), axis=(0, 0, -1), limits_deg=(0, 100), rest_deg=0,
           stiffness=0.01, damping=0.002, friction=0.0005)
for name, sgn in (("wingP", 1.0), ("wingN", -1.0)):
    t.link(name, [lk.box((WING_L, WING_W, WING_T), (0, -sgn * WING_W / 2 + 0.0, WING_T / 2))], mass=0.0033,
           at=(0, sgn * WING_Y, ZP + 0.0006), color=STEEL, friction=(0.5, 0.4), contact_offset=0.0005)
    t.revolute("door" + name[-1], "frame", name, anchor=(0, sgn * WING_Y, ZP + 0.0006), axis=(-sgn, 0, 0), limits_deg=(0, 105),
               rest_deg=0, stiffness=0.01, damping=0.001)
RAZOR = t.build()

# blade: a 43 x 22 plate with two square holes, built from seven boxes
hx = POST_X
xs = hx - HOLE / 2  # hole inner edge from the centre
xo = hx + HOLE / 2  # hole outer edge
_bar = lambda x0, x1, y0, y1: lk.box((x1 - x0, y1 - y0, BT), ((x0 + x1) / 2, (y0 + y1) / 2, 0.0))  # noqa: E731
BLADE_SHAPES = [
    _bar(-BL / 2, -xo, -BW / 2, BW / 2), _bar(-xs, xs, -BW / 2, BW / 2), _bar(xo, BL / 2, -BW / 2, BW / 2),
    _bar(-xo, -xs, HOLE / 2, BW / 2), _bar(-xo, -xs, -BW / 2, -HOLE / 2),
    _bar(xs, xo, HOLE / 2, BW / 2), _bar(xs, xo, -BW / 2, -HOLE / 2),
]
t = lk.Tree("blade", pos=(BX, BY, TOP + BLADE_Z0), color=BLADE_C)
t.link("blade", BLADE_SHAPES, mass=0.003, friction=(1.0, 0.8), contact_offset=0.0003)
BLADE = t.build()

t = lk.Tree("shelf", pos=(BX, BY, TOP), fixed=True, color=SHELF_C)
t.link("shelf", [lk.box((0.026, 0.014, 0.130), (0, 0, 0.065)), lk.box((0.060, 0.060, 0.012), (0, 0, 0.006))],
       mass=0.5, friction=(0.5, 0.4))
SHELF = t.build()

OBJECTS = [RAZOR, BLADE, SHELF]

# -- scoring: whole-run record -----------------------------------------------------------------------------------
EDGE_V = 0.008  # |v| from the blade's centre line: the outer 3 mm of the 22 mm width are the cutting edges
PAD_HALF = 0.006  # pad footprint half-width along the tool's y (a 12 mm pad)
SEAT_XY_MM, SEAT_DZ_MM, SEAT_YAW, SEAT_TILT = 1.5, 0.6, 5.0, 3.0
OPEN_DEG, CLOSED_DEG, KNOB_BACK_DEG = 60.0, 2.0, 15.0
LIFT_M = 0.020
CLEAR_M, TCP_STILL, STILL_MM_S, STILL_DEG_S = 0.030, 0.010, 1.0, 2.0
RELEASE_S = STILL_S = 1.0
EPS = 1e-6


def edge_touch(tool, B) -> bool:
    """A pad (footprint 12 mm along the tool's y) within 1 mm of the blade's flat faces, over its outer 3 mm."""
    R, c = np.asarray(B)[:3, :3], np.asarray(B)[:3, 3]
    for p in tool.pad_points():
        for k in (-1.0, 0.0, 1.0):
            loc = R.T @ (p + k * PAD_HALF * np.asarray(tool.rot)[:, 1] - c)
            if abs(loc[0]) <= BL / 2 + 0.001 and abs(loc[2]) <= 0.001 and EDGE_V <= abs(loc[1]) <= BW / 2 + 0.001:
                return True
    return False


def blade_rel(B, plate_top) -> dict:
    """The blade's pose in the head: mm from the plate's centre (dz from the plate top), yaw and tilt in deg."""
    B = np.asarray(B)
    d = (B[:3, 3] - np.asarray(plate_top)) / MM
    return {"dx": float(d[0]), "dy": float(d[1]), "dz": float(d[2]),
            "yaw": math.degrees(math.atan2(B[1, 0], B[0, 0])),
            "tilt": math.degrees(math.acos(max(-1.0, min(1.0, float(B[2, 2])))))}


def seated(m) -> bool:
    return (abs(m["dx"]) <= SEAT_XY_MM and abs(m["dy"]) <= SEAT_XY_MM and -0.3 <= m["dz"] - BT / 2 / MM <= SEAT_DZ_MM
            and abs(m["yaw"]) <= SEAT_YAW and m["tilt"] <= SEAT_TILT)


def measure(razor, blade, tools) -> dict:
    B = np.array(blade.poses["blade"], dtype=float)
    P = razor.point("frame", (0.0, 0.0, ZP))
    m = blade_rel(B, P)
    m["knob"] = razor.deg("turn")
    wp, wn = razor.deg("doorP"), razor.deg("doorN")
    m["wing_min"], m["wing_max"] = min(wp, wn), max(wp, wn)
    m["lift"] = float(B[2, 3] - (TOP + BLADE_Z0))
    m["edge"] = {s: edge_touch(tl, B) for s, tl in tools.items()}
    m["held"] = {s: bool(tl.opening <= 0.006 and lk.clearance(tl, blade) <= 0.002) for s, tl in tools.items()}
    m["clear"] = {s: min(lk.clearance(tl, blade), lk.clearance(tl, razor)) for s, tl in tools.items()}
    m["tcp_speed"] = {s: float(tl.speed) for s, tl in tools.items()}
    m["speed"], m["spin"] = 0.0, 0.0
    return m


def pose_rates(A0, A1, dt: float) -> tuple[float, float]:
    """Linear (mm/s) and angular (deg/s) speed from the pose change between two control steps (what a camera sees)."""
    A0, A1 = np.asarray(A0), np.asarray(A1)
    v = float(np.linalg.norm(A1[:3, 3] - A0[:3, 3])) / dt / MM
    R = A0[:3, :3].T @ A1[:3, :3]
    w = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1) / 2)))) / dt
    return v, w


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
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        opened = m["wing_min"] >= OPEN_DEG
        H.ever("edge", any(m["edge"].values()))
        H.peak("knob_max", m["knob"])
        H.peak("lift_max", m["lift"])
        H.ever("carried", any(m["held"].values()) and m["lift"] >= LIFT_M and opened)
        H.ever("seated_open", seated(m) and opened)
        H.held("still", m["speed"] < STILL_MM_S and m["spin"] < STILL_DEG_S)
        H.held("released", min(m["clear"].values()) >= CLEAR_M - EPS and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Score and the reason. The gate is checked first (any time in the run), then the rungs from the top."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "edge" in H.evers:
        return 0.0, f"gate: a gripper pad touched the blade's cutting edges at {H.evers['edge']:.2f} s"
    if seated(m) and m["wing_max"] <= CLOSED_DEG and m["knob"] <= KNOB_BACK_DEG and "seated_open" in H.evers:
        ends = {"blade still 1 s": H.runs.get("still", 0.0) >= STILL_S - EPS,
                "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS}
        if all(ends.values()):
            return 1.0, "blade seated on the posts, doors closed on it, both grippers let go"
        return 0.75, "blade seated and doors closed; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if "seated_open" in H.evers:
        return 0.5, "the blade was seated on the posts with the doors open, but is not seated under closed doors"
    if "carried" in H.evers:
        return 0.25, "the blade was carried 20 mm up with the doors open, but never seated on the posts"
    return 0.0, "the blade was never carried held by its ends with the doors open"


REC = Record()
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _couple(tick):
    """The knob's cam: the doors follow the knob (0 to 90 deg of knob turn = 0 to 100 deg of door opening)."""
    kn = tick.states["razor"].deg("turn")
    r = math.radians(max(0.0, min(WING_OPEN, kn * WING_OPEN / KNOB_OPEN)))
    lk.set_rest("razor", "doorP", r)
    lk.set_rest("razor", "doorN", r)


@lk.on_step
def _track(tick):
    m = measure(tick.states["razor"], tick.states["blade"], tick.tools)
    B = np.array(tick.states["blade"].poses["blade"], dtype=float)
    if _PREV:
        m["speed"], m["spin"] = pose_rates(_PREV["B"], B, tick.t - _PREV["t"])
    _PREV.update(B=B, t=tick.t)
    REC.update(m)
    if any(m["edge"].values()) and "edge_dbg" not in LOG:
        side = [k for k, v in m["edge"].items() if v][0]
        tl = tick.tools[side]
        LOG["edge_dbg"] = {"side": side, "t": round(tick.t, 2), "tcp": [round(float(v), 4) for v in tl.tcp],
                           "opening": round(tl.opening, 4), "blade": [round(float(v), 4) for v in B[:3, 3]],
                           "pads": [[round(float(v), 4) for v in p] for p in tl.pad_points()]}
    if REC.n % 5 == 0:
        REC.trace.append((round(tick.t, 2), round(m["knob"], 1), round(m["wing_min"], 1), round(m["wing_max"], 1),
                          round(m["dx"], 2), round(m["dy"], 2), round(m["dz"], 2), round(m["yaw"], 1),
                          round(m["lift"] * 1000, 1), [int(v) for v in m["held"].values()],
                          [int(v) for v in m["edge"].values()],
                          [round(1000 * m["clear"][s]) for s in ("left", "right")],
                          [round(1000 * m["tcp_speed"][s], 1) for s in ("left", "right")]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Reads only what a camera could measure: object and link poses (the knob and door angles computed from link poses)
# and the robot's own grippers. Never joint values, the record or the scorer.

VARIANTS = ("", "nothing", "twist_only", "one_arm", "edge_grip")
LOG: dict = {}
D_KNOB = 0.0115  # pad contact centre past the TCP when pinching the knob
LEFT_PITCH = 15.0  # deg: the left hand points 15 deg down at the knob
TAB = 0.004  # pad tips lie 4 mm over the blade's end
HOVER = 0.004  # blade bottom above the post tops (m) before the final descent


def _rot_z(deg):
    from linkdemo import rot_about

    return rot_about((0, 0, 1), deg)


def left_twist(ctx, st):
    """Pinch the knob (pads across its 22 mm, hand 15 deg down), turn it 90 deg clockwise and hold it."""
    from linkdemo import grip_frame, OPEN_WIDTH  # noqa: F401

    arm = ctx.left
    seen = ctx.observe()
    c = seen.point("razor", "knob")
    app = np.array([math.cos(math.radians(LEFT_PITCH)), 0.0, -math.sin(math.radians(LEFT_PITCH))])
    r = grip_frame(app, (0, 1, 0))
    tcp = c - r @ np.array([0.0, 0.0, D_KNOB])
    yield from arm.move(tcp - app * 0.07, r, width=OPEN_WIDTH, duration=2.0)
    yield from arm.move(tcp, r, duration=1.5)
    yield from arm.grip(2 * KNOB_R - 0.002, 0.8)
    yield from arm.wait(0.4)
    tl = ctx.observe().tools["left"]
    st["left_pinch_opening_mm"] = round(1000 * tl.opening, 2)
    st["left_tcp_vs_cmd_mm"] = [round(1000 * float(v), 2) for v in (tl.tcp - arm.pos)]
    st["left_tcp_vs_knob_mm"] = [round(1000 * float(v), 2) for v in (tl.tcp + tl.rot @ np.array([0, 0, D_KNOB]) - c)]
    yield from arm.arc(c, (0, 0, -1), 92.0, point=(0.0, 0.0, D_KNOB), follow=1.0, duration=4.0)
    st["knob_after_arc"] = round(ctx.observe().angle_deg("razor", "turn"), 2)
    for _ in range(2):
        yield from arm.wait(0.4)
        a = ctx.observe().angle_deg("razor", "turn")
        if a >= KNOB_OPEN + 1.0 or a < 20.0:
            break
        yield from arm.arc(c, (0, 0, -1), KNOB_OPEN + 2.0 - a, point=(0.0, 0.0, D_KNOB), follow=1.0, duration=1.0)
    st["knob_seen_deg"] = round(ctx.observe().angle_deg("razor", "turn"), 2)
    st["left_twisted"] = True


def left_release(ctx, st):
    """Ease the knob back over 3 s so the doors close on the blade, let go, back off."""
    arm = ctx.left
    seen = ctx.observe()
    c = seen.point("razor", "knob")
    a = seen.angle_deg("razor", "turn")
    yield from arm.arc(c, (0, 0, -1), -(a + 2.0), point=(0.0, 0.0, D_KNOB), follow=1.0, duration=3.0)
    yield from arm.open(0.6)
    yield from arm.retreat(0.07, 1.2)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.05]) - arm.rot[:, 2] * 0.04, duration=1.2)
    st["left_done"] = True


def _blade_seen(ctx):
    return ctx.observe().pose("blade", "blade")


def _servo(ctx, arm, target, iters=3, settle=0.5, yaw=True):
    """Move the hand-and-blade assembly rigidly so the seen blade centre reaches ``target`` and its yaw is zero."""
    for _ in range(iters):
        B = _blade_seen(ctx)
        c = B[:3, 3]
        ang = -math.degrees(math.atan2(B[1, 0], B[0, 0])) if yaw else 0.0
        Rz = _rot_z(ang)
        pos = np.asarray(target) + Rz @ (arm.pos - c)
        yield from arm.move(pos, Rz @ arm.rot, duration=0.7)
        yield from arm.wait(settle)


def _enter(arm, tcp, r):
    """From above and 20 mm short, then in along +x (a straight run in from the ready pose stalls the arm's IK)."""
    from linkdemo import OPEN_WIDTH

    yield from arm.move(tcp + [-0.02, 0, 0.08], r, width=OPEN_WIDTH, duration=3.0)
    yield from arm.move(tcp + [-0.02, 0, 0.0], r, duration=2.0)
    yield from arm.move(tcp, r, duration=1.5)


def right_place(ctx, st, wait_open=True):
    from linkdemo import OPEN_WIDTH, grip_frame  # noqa: F401

    arm = ctx.right
    seen = ctx.observe()
    B0 = seen.pose("blade", "blade")
    plate = seen.point("razor", "frame", (0.0, 0.0, ZP))
    xend = B0[0, 3] - BL / 2
    r = grip_frame((1, 0, 0), (0, 0, 1))
    tcp = np.array([xend + TAB - 0.013, B0[1, 3], B0[2, 3]])
    yield from _enter(arm, tcp, r)
    yield from arm.grip(0.0, 0.8)
    yield from arm.wait(0.4)
    st["right_grip_opening_mm"] = round(1000 * ctx.observe().tools["right"].opening, 2)
    yield from arm.move(arm.pos + [0, 0, 0.045], duration=1.0)
    seat = np.array([plate[0], plate[1], plate[2] + BT / 2])
    yield from _servo(ctx, arm, seat + [0, -0.06, 0.020], iters=1, settle=0.3, yaw=False)
    yield from _servo(ctx, arm, seat + [0, 0, 0.020], iters=3)
    st["carried_at"] = round(ctx.t, 2)
    if wait_open:
        t0 = ctx.t
        while ctx.observe().angle_deg("razor", "doorP") < 85.0 and ctx.t - t0 < 25.0:
            yield from arm.wait(0.1)
        st["wings_open_seen"] = round(ctx.observe().angle_deg("razor", "doorP"), 1)
    yield from _servo(ctx, arm, seat + [0, 0, HOVER], iters=3)
    for h in (0.0025, 0.0012, 0.0004):
        yield from _servo(ctx, arm, seat + [0, 0, h], iters=1, settle=0.4)
    for _ in range(4):
        B = _blade_seen(ctx)
        if B[2, 3] <= seat[2] + 0.0003:
            break
        yield from arm.move(arm.pos - [0, 0, 0.0004], duration=0.4)
        yield from arm.wait(0.3)
    st["seat_seen_mm"] = [round(1000 * v, 2) for v in (_blade_seen(ctx)[:3, 3] - seat)]
    yield from arm.grip(OPEN_WIDTH, 0.6)
    yield from arm.retreat(0.06, 1.2)
    st["right_done"] = True


def right_one_arm(ctx, st):
    """Blade carried over the shut razor and lowered onto the closed doors, then let go."""
    from linkdemo import OPEN_WIDTH, grip_frame  # noqa: F401

    arm = ctx.right
    seen = ctx.observe()
    B0 = seen.pose("blade", "blade")
    plate = seen.point("razor", "frame", (0.0, 0.0, ZP))
    r = grip_frame((1, 0, 0), (0, 0, 1))
    tcp = np.array([B0[0, 3] - BL / 2 + TAB - 0.013, B0[1, 3], B0[2, 3]])
    yield from _enter(arm, tcp, r)
    yield from arm.grip(0.0, 0.8)
    yield from arm.wait(0.4)
    yield from arm.move(arm.pos + [0, 0, 0.045], duration=1.0)
    seat = np.array([plate[0], plate[1], plate[2] + BT / 2])
    yield from _servo(ctx, arm, seat + [0, -0.06, 0.020], iters=1, settle=0.3, yaw=False)
    yield from _servo(ctx, arm, seat + [0, 0, 0.020], iters=3)
    yield from _servo(ctx, arm, seat + [0, 0, 0.0], iters=2)
    yield from arm.grip(OPEN_WIDTH, 0.6)
    yield from arm.retreat(0.06, 1.2)


def right_edge_grip(ctx, st):
    """Pinch the blade's end tab 7 mm off its centre line, over a cutting edge, and lift it."""
    from linkdemo import grip_frame

    arm = ctx.right
    B0 = ctx.observe().pose("blade", "blade")
    r = grip_frame((1, 0, 0), (0, 0, 1))
    tcp = np.array([B0[0, 3] - BL / 2 + TAB - 0.013, B0[1, 3] + 0.007, B0[2, 3]])
    yield from _enter(arm, tcp, r)
    yield from arm.grip(0.0, 0.8)
    yield from arm.wait(0.4)
    yield from arm.move(arm.pos + [0, 0, 0.04], duration=1.0)
    yield from arm.wait(1.0)


def demo_script(ctx):
    variant = VARIANT
    LOG.clear()
    st = LOG
    st["variant"] = variant
    yield from ctx.wait(1.0)
    if variant == "nothing":
        yield from ctx.wait(6.0)
        return
    if variant == "twist_only":
        yield from left_twist(ctx, st)
        yield from ctx.wait(3.0)
        return
    if variant == "one_arm":
        yield from right_one_arm(ctx, st)
        yield from ctx.wait(2.0)
        return
    if variant == "edge_grip":
        yield from right_edge_grip(ctx, st)
        return

    def left():
        yield from left_twist(ctx, st)
        while not st.get("right_done"):
            yield from ctx.left.wait(0.1)
        yield from left_release(ctx, st)

    yield from ctx.both(left(), right_place(ctx, st))
    yield from ctx.wait(3.5)


def demo_report() -> dict:
    score_, why = ladder(REC)
    H = REC.H
    m = REC.last or {}
    last = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items() if k not in ("clear", "tcp_speed")}
    if m:
        last["clear_mm"] = {s: round(1000 * v, 1) for s, v in m["clear"].items()}
    return {"score": score_, "reason": why, "variant": VARIANT,
            "evers": {k: round(v, 2) for k, v in H.evers.items()},
            "peaks": {k: round(v, 3) for k, v in H.peaks.items()},
            "runs": {k: round(v, 2) for k, v in H.runs.items()},
            "last": last, "log": dict(LOG), "trace": REC.trace}
