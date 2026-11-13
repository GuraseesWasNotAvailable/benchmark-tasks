# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Lock out a lever ball valve: close a spring-opened clamshell lockout around its handle and slide its lock bar through.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475. The device frame ("dev", mm) is used for the lockout: origin on the valve stem at
the lever's mid-plane, +x along the shut lever (world -y), +y world +x, +z up (yaw -90 deg); stem at world (0.17, 0.09).

- A 1/2 inch brass lever ball valve sits on a 21 mm pipe stub fixed to the table, the pipe running along world x.
  Its yellow steel lever (95 x 20 x 4 mm, 50 g) turns about the vertical stem against 1.2 N m of friction and lies
  shut (crosswise to the pipe), pointing toward the robot's right arm.
- A red lockout clamshell is bolted to the valve's bracket: two 100 mm long, 28 mm high ABS halves hinged on
  horizontal pins at the lever's lower edge, one each side of the lever, each held open at 38 deg by a spring
  (0.10 N m/rad). Shut, the two walls leave the lever 1.5 mm each side; each half ends in a 5 mm plate with a
  10.5 mm square hole, the two plates side by side so that the holes line up only when both halves are shut.
- A steel lock bar (8 x 8 x 60 mm, 43 g, with a 16 mm cap) lies on a 6 mm rib on the table to the right of the valve.

Goal: both halves shut around the lever, the bar through both plates' holes, resting in them; both grippers let go.
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

X0, Y0 = 0.17, 0.09  # stem, world
ZH = TOP + 0.075  # lever mid-plane height
YAW = -90.0
R_DEV = lk.axis_rotation((0, 0, 1), math.radians(YAW))  # dev -> world
ORIGIN = np.array([X0, Y0, ZH])


def dev_to_world(p_mm) -> np.ndarray:
    return R_DEV @ (np.asarray(p_mm, dtype=float) * MM) + ORIGIN


def world_to_dev(p) -> np.ndarray:
    return 1000.0 * (R_DEV.T @ (np.asarray(p, dtype=float) - ORIGIN))


# -- dimensions (dev mm) ---------------------------------------------------------------------------------------------
HINGE_Y, HINGE_Z = 13.0, -14.0  # hinge pins at y = +-13, 14 mm below the lever's mid-plane
WALL_X0, WALL_X1, WALL_T, WALL_H = 4.0, 104.0, 3.0, 28.0
HOLE = 10.5  # square hole side; the bar is 8 x 8
PLATE_A = (104.0, 109.0)  # half A (the +y half, far from the robot): its plate
PLATE_B = (109.0, 114.0)  # half B (the -y half): its plate, nearer the bar
OPEN_DEG = 38.0
SPRING = 0.10  # N m/rad each half
BAR_L, BAR_W = 60.0, 8.0
BAR_START_X = 190.0  # bar centre, dev x (world y = 0.09 - 0.19 = -0.10)
RIB_TOP = 0.012
RED, YELLOW, BRASS, STEEL = (0.80, 0.12, 0.10), (0.95, 0.78, 0.10), (0.78, 0.60, 0.25), (0.62, 0.63, 0.67)

t = lk.Tree("valve", pos=(X0, Y0, ZH), yaw_deg=YAW, fixed=True, color=BRASS)
t.link("body", [
    lk.cylinder(0.0107, 0.200, (0, 0, -0.048), axis="Y"),  # pipe stub, along dev y
    lk.box((0.036, 0.040, 0.034), (0, 0, -0.033)),  # valve body
    lk.cylinder(0.008, 0.010, (0, 0, -0.011)),  # stem neck
    lk.box((0.030, 0.014, 0.016), (0, 0.075, -0.067)),  # pipe stand, +y
    lk.box((0.030, 0.014, 0.016), (0, -0.075, -0.067)),  # pipe stand, -y
    lk.box((0.014, 0.044, 0.030), (0.011, 0, -0.030)),  # bracket from the body to the clamshell base
    lk.box((0.100, 0.044, 0.011), (0.054, 0, -0.0205), color=(0.30, 0.30, 0.32)),  # clamshell base plate
], mass=1.0, friction=(0.6, 0.5), contact_offset=0.001)
t.link("handle", [lk.box((0.095, 0.020, 0.004), (0.0375, 0, 0)), lk.cylinder(0.009, 0.008, (0, 0, 0))], mass=0.050,
       color=YELLOW, friction=(0.5, 0.4), contact_offset=0.001)
t.revolute("stem", "body", "handle", anchor=(0, 0, 0), axis=(0, 0, 1), limits_deg=(-1.0, 100.0), friction=1.2,
           damping=0.02)


def _plate(s, x0, x1):
    """Boxes of a half's end plate in dev mm as (size, centre); the plate spans y -13..16 for s=+1 (mirrored for -1)
    and z -14..14, with a square hole at y 0, z 0."""
    h = HOLE / 2
    y_lo, y_hi = (-13.0, 16.0) if s > 0 else (-16.0, 13.0)
    xc, tx = (x0 + x1) / 2, x1 - x0
    rows = [((y_hi - y_lo, 14 - h), ((y_lo + y_hi) / 2, (14 + h) / 2)),
            ((y_hi - y_lo, 14 - h), ((y_lo + y_hi) / 2, -(14 + h) / 2)),
            ((-h - y_lo, 2 * h), ((y_lo - h) / 2, 0.0)),
            ((y_hi - h, 2 * h), ((y_hi + h) / 2, 0.0))]
    return [(np.array([tx, wy, wz]), np.array([xc, cy, cz])) for (wy, wz), (cy, cz) in rows]


def _shell(s, name, plate, axis):
    origin = np.array([0.0, s * HINGE_Y, HINGE_Z])
    wall = (np.array([WALL_X1 - WALL_X0, WALL_T, WALL_H]), np.array([(WALL_X0 + WALL_X1) / 2, s * HINGE_Y, 0.0]))
    boxes = [wall] + _plate(s, *plate)
    shapes = [lk.box(sz * MM, (c - origin) * MM) for sz, c in boxes]
    t.link(name, shapes, mass=0.060, at=tuple(origin * MM), color=RED, friction=(0.25, 0.2), contact_offset=0.001)
    t.revolute("hinge" + name[-1], "body", name, anchor=tuple(origin * MM), axis=axis, limits_deg=(0.0, 42.0),
               rest_deg=OPEN_DEG, init_deg=OPEN_DEG, stiffness=SPRING, damping=0.01)


_shell(+1, "shellA", PLATE_A, (-1, 0, 0))
_shell(-1, "shellB", PLATE_B, (1, 0, 0))
VALVE = t.build()

t = lk.Tree("bar", pos=(X0, Y0 - BAR_START_X * MM, TOP + RIB_TOP + BAR_W * MM / 2), yaw_deg=YAW, color=STEEL)
t.link("bar", [lk.box((BAR_L * MM, BAR_W * MM, BAR_W * MM)), lk.box((0.004, 0.016, 0.016), (0.032, 0, 0), color=RED)],
       mass=0.043, friction=(0.4, 0.3), contact_offset=0.001)
BAR = t.build()

t = lk.Tree("rib", pos=(X0, Y0 - BAR_START_X * MM, TOP), yaw_deg=YAW, fixed=True, color=(0.30, 0.30, 0.32))
t.link("rib", [lk.box((0.035, 0.006, RIB_TOP), (0, 0, RIB_TOP / 2))], mass=0.05, friction=(0.4, 0.3),
       contact_offset=0.001)
RIB = t.build()

OBJECTS = [VALVE, BAR, RIB]

# -- scoring ---------------------------------------------------------------------------------------------------------
GATE_STEM_DEG = 5.0  # lever turned this far from shut at any time -> 0
CLOSED_DEG = 6.0  # a half counts as shut at or below this (the bar's play in the holes allows about 5 deg)
TIP_PAST_MM = 99.0  # bar tip at or beyond this dev x: 5 mm past the far face of the second plate
TIP_IN_B_MM = 109.0  # tip at or beyond the first plate's far face (that plate spans 109 to 114)
CENTRE_TOL_MM, TILT_TOL_DEG = 3.0, 20.0
STEADY_S, RELEASE_S = 2.0, 1.0
BAR_STILL, BAR_SPIN = 0.003, math.radians(3.0)
SHELL_RATE = 1.0  # deg/s
CLEAR_M, TCP_STILL = 0.030, 0.005
HOLD_SHELL_OPEN, HOLD_BAR_OPEN, HELD_NEAR = 0.040, 0.022, 0.006  # opening reads the finger joints: 15.5 mm on the 8 mm bar
EPS = 1e-6


def _bar_line(bar):
    """Bar centre, unit axis (tail to tip is -x) and tip, in dev mm."""
    A = np.array(bar.poses["bar"], dtype=float)
    R = R_DEV.T @ A[:3, :3]
    c = world_to_dev(A[:3, 3])
    a = R[:, 0]
    return c, a, c - 0.5 * BAR_L * a


def _at_x(c, a, x):
    return c + a * ((x - c[0]) / a[0])


def measure(valve, bar, tools) -> dict:
    """One sample of everything the scorer uses (dev mm, deg, m/s)."""
    c, a, tip = _bar_line(bar)
    pA = _at_x(c, a, sum(PLATE_A) / 2)
    pB = _at_x(c, a, sum(PLATE_B) / 2)
    near_shell = {s: lk.clearance(tl, valve, ("shellA", "shellB")) for s, tl in tools.items()}
    near_bar = {s: lk.clearance(tl, bar) for s, tl in tools.items()}
    clear = {s: min(lk.clearance(tl, valve), near_bar[s]) for s, tl in tools.items()}
    return {
        "qA": valve.deg("hingeA"), "qB": valve.deg("hingeB"), "stem": valve.deg("stem"),
        "tip_x": float(tip[0]),
        "off": float(max(abs(pA[1]), abs(pA[2]), abs(pB[1]), abs(pB[2]))),
        "tilt": math.degrees(math.acos(max(-1.0, min(1.0, abs(float(a[0])))))),
        "held_shell": any(tools[s].opening <= HOLD_SHELL_OPEN and near_shell[s] <= HELD_NEAR for s in tools),
        "held_bar": any(tools[s].opening <= HOLD_BAR_OPEN and near_bar[s] <= HELD_NEAR for s in tools),
        "bar_speed": 0.0, "bar_spin": 0.0, "shell_rate": 0.0,
        "open": {s: tl.opening for s, tl in tools.items()}, "near_bar": near_bar,
        "clear": clear, "tcp_speed": {s: float(tl.speed) for s, tl in tools.items()},
    }


def pose_rates(A_prev, A, dt: float) -> tuple[float, float]:
    """Speed (m/s) and spin (rad/s) from the pose change over ``dt``: what a camera sees."""
    if dt <= 0.0:
        return 0.0, 0.0
    v = float(np.linalg.norm(A[:3, 3] - A_prev[:3, 3])) / dt
    R = A_prev[:3, :3].T @ A[:3, :3]
    sin = 0.5 * float(np.linalg.norm((R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1])))
    return v, math.atan2(sin, (float(np.trace(R)) - 1.0) / 2.0) / dt


def shut(m) -> bool:
    return m["qA"] <= CLOSED_DEG and m["qB"] <= CLOSED_DEG


def through(m) -> bool:
    """Bar tip at least 5 mm past the second plate, bar centred in both holes, within 20 deg of the clamshell axis."""
    return m["tip_x"] <= TIP_PAST_MM and m["off"] <= CENTRE_TOL_MM and m["tilt"] <= TILT_TOL_DEG


class Record:
    """Whole-run record: a linkkit History updated every control step, cleared only by the simulator reset."""

    def __init__(self):
        self.H = lk.History()
        self.last = None
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
        H.ever("turned", m["stem"] >= GATE_STEM_DEG)
        H.peak("stem_max", m["stem"])
        H.ever("locked", shut(m) and through(m))
        H.ever("bar_in_b", m["held_bar"] and m["tip_x"] <= TIP_IN_B_MM and m["off"] <= CENTRE_TOL_MM)
        H.held("steady", shut(m) and through(m) and m["bar_speed"] < BAR_STILL and m["bar_spin"] < BAR_SPIN
               and m["shell_rate"] < SHELL_RATE)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Score and the reason: the gate first (any time in the run), then the rungs from the top."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "turned" in H.evers:
        return 0.0, f"gate: the lever turned {H.peaks['stem_max']:.1f} deg from shut at {H.evers['turned']:.2f} s"
    if shut(m) and through(m):
        ends = {"bar and halves steady 2 s": H.runs.get("steady", 0.0) >= STEADY_S - EPS,
                "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS}
        if all(ends.values()):
            return 1.0, "both halves shut, bar through both holes, both grippers let go"
        return 0.75, "locked at the end; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if "locked" in H.evers:
        return 0.5, "the lock was made at some time (halves shut, bar through both holes) but is not in place at the end"
    if "bar_in_b" in H.evers:
        return 0.25, "the held bar went through the first plate's hole, but never through both"
    return 0.0, "the held bar never went through the first plate's hole"


REC = Record()
CTX: list = []
DIAG: list = []
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    s = tick.states
    m = measure(s["valve"], s["bar"], tick.tools)
    A = np.array(s["bar"].poses["bar"], dtype=float)
    if _PREV:
        dt = tick.t - _PREV["t"]
        m["bar_speed"], m["bar_spin"] = pose_rates(_PREV["A"], A, dt)
        if dt > 0:
            m["shell_rate"] = max(abs(m["qA"] - _PREV["qA"]), abs(m["qB"] - _PREV["qB"])) / dt
    _PREV.update(A=A, t=tick.t, qA=m["qA"], qB=m["qB"])
    REC.update(m)
    if CTX and REC.n % 10 == 0:
        c = CTX[0]
        DIAG.append((round(tick.t, 1), [round(1000 * float(np.linalg.norm(tick.tools[k].tcp - c.arm(k).pos)), 1) for k in ("left", "right")],
                     [round(float(v), 3) for v in tick.tools["left"].tcp], [round(float(v), 3) for v in tick.tools["right"].tcp]))
    if REC.n % 5 == 0:
        REC.trace.append((round(tick.t, 2), round(m["qA"], 1), round(m["qB"], 1), round(m["stem"], 2),
                          round(m["tip_x"], 1), round(m["off"], 1), round(m["tilt"], 1), int(m["held_shell"]),
                          int(m["held_bar"]), [round(1000 * m["clear"][k]) for k in ("left", "right")],
                          round(1000 * m["open"]["right"], 1), round(1000 * m["near_bar"]["right"], 1)))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


def demo_report() -> dict:
    return {"ladder": ladder(REC), "variant": VARIANT, "trace": REC.trace[::2], "log": dict(LOG),
            "diag": DIAG[::1], "values": {k: round(v, 3) for k, v in REC.H.peaks.items()}}


# -- scripted demo (reads only camera-observable poses and the grippers' own state) ------------------------------------
VARIANTS = ("", "nothing", "left_only", "right_only")
LOG: dict = {}
PAD_DEPTH = 0.0115
SHELL_X = 20.0  # dev x of the left pinch
LEAN = math.radians(35.0)  # both grippers lean away from each other about the pad axis, so the wrists clear
SHELL_Z = 2.0  # dev z of the pad contact centre
OPEN_W, SQUEEZE_W = 0.080, 0.012  # approach opening; commanded closed width (the shut halves stop it at 29 mm, so the fingers press hard)
GRASP_LOCAL = 16.0  # bar-frame x of the right pinch (toward the tail)
TIP_START, TIP_NEAR, TIP_END = 140.0, 119.0, 91.0


def left(ctx):
    import linkdemo as ld

    arm = ctx.left
    yield from ctx.wait(1.0)
    app = np.array([0.0, -math.sin(LEAN), -math.cos(LEAN)])
    r = ld.grip_frame(app, (1.0, 0.0, 0.0), ref=ld.down(-90))
    base = ctx.observe().pose("valve", "body")
    p_c = base[:3, :3] @ (np.array([SHELL_X, 0.0, SHELL_Z]) * MM) + base[:3, 3]
    tcp = p_c - r @ np.array([0.0, 0.0, PAD_DEPTH])
    yield from arm.move(tcp - 0.07 * app, r, width=OPEN_W, duration=2.5)
    yield from arm.move(tcp, r, duration=1.5)
    yield from arm.wait(0.3)
    yield from arm.grip(SQUEEZE_W, 1.5)
    yield from arm.wait(0.5)
    seen = ctx.observe()
    LOG["shell_deg_closed"] = [round(seen.angle_deg("valve", "hingeA"), 2), round(seen.angle_deg("valve", "hingeB"), 2)]
    LOG["left_closed_at"] = round(ctx.t, 2)
    LOG["closed"] = True
    while not LOG.get("right_done"):
        yield from arm.wait(0.2)
    yield from arm.grip(OPEN_W, 0.8)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.10]), duration=1.2)
    LOG["left_done"] = True


class Hand:
    """The right arm carrying the bar: commands the TCP so that the seen bar follows a planned tip and orientation."""

    def __init__(self, ctx):
        self.ctx, self.arm = ctx, ctx.right
        self.G = None
        self.bias = np.zeros(3)
        self.Rcmd = R_DEV.copy()

    def seen(self):
        return np.array(self.ctx.observe().pose("bar", "bar"), dtype=float)

    def tip_of(self, A):
        return A[:3, :3] @ np.array([-0.5 * BAR_L * MM, 0, 0]) + A[:3, 3]

    def pose_for(self, tip_x, dy=0.0, dz=0.0):
        tip = dev_to_world((tip_x, dy, dz)) + self.bias
        A = np.eye(4)
        A[:3, :3] = self.Rcmd
        A[:3, 3] = tip + self.Rcmd @ np.array([0.5 * BAR_L * MM, 0, 0])
        return A

    def tcp(self, A):
        T = A @ self.G
        return T[:3, 3], T[:3, :3]

    def go(self, tip_x, seconds, dz=0.0):
        p, r = self.tcp(self.pose_for(tip_x, 0.0, dz))
        yield from self.arm.move(p, r, duration=seconds)

    def servo(self, tip_x, tries=3, tol_mm=0.3, tol_deg=0.5):
        for _ in range(tries):
            yield from self.arm.wait(0.3)
            A = self.seen()
            e_dev = R_DEV.T @ (dev_to_world((tip_x, 0, 0)) - self.tip_of(A))
            ax = (R_DEV.T @ A[:3, :3])[:, 0]
            ang = math.degrees(math.acos(max(-1.0, min(1.0, float(ax[0])))))
            tool = self.ctx.observe().tools["right"]
            slip = np.linalg.inv(A) @ np.r_[tool.tcp, 1.0]
            track = 1000 * float(np.linalg.norm(tool.tcp - self.arm.pos))
            LOG.setdefault("servo", []).append((round(tip_x), [round(1000 * v, 2) for v in e_dev], round(ang, 2),
                                                round(track, 1), [round(1000 * v, 1) for v in slip[:3]],
                                                [round(1000 * v, 1) for v in (self.G[:3, 3])]))
            if float(np.linalg.norm(e_dev[1:])) * 1000 < tol_mm and ang < tol_deg:
                return
            self.bias += R_DEV @ np.array([0.0, e_dev[1], e_dev[2]])
            self.Rcmd = (R_DEV @ A[:3, :3].T) @ self.Rcmd  # rotate the command by the seen axis error
            yield from self.go(tip_x, 0.5)


def right(ctx):
    import linkdemo as ld

    arm = ctx.right
    hand = Hand(ctx)
    yield from ctx.wait(1.0)
    app = np.array([0.0, math.sin(LEAN), -math.cos(LEAN)])
    r = ld.grip_frame(app, (1.0, 0.0, 0.0), ref=ld.down(90))
    A0 = hand.seen()
    grasp_w = A0[:3, 3] + A0[:3, :3] @ np.array([GRASP_LOCAL * MM, 0, 0])
    tcp = grasp_w - r @ np.array([0.0, 0.0, 0.0105])
    yield from arm.move(tcp - 0.07 * app, r, width=0.024, duration=2.5)
    yield from arm.move(tcp, r, duration=1.5)
    yield from arm.grip(0.0, 0.8)
    yield from arm.wait(0.4)
    A0 = hand.seen()  # the bar as held (the pads moved it a little)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = arm.rot, arm.pos
    hand.G = np.linalg.inv(A0) @ T  # the grasp as commanded, in the bar frame
    hand.Rcmd = A0[:3, :3].copy()
    c_dev = world_to_dev(A0[:3, 3])
    lift = A0.copy()
    lift[2, 3] += 0.06
    p, rr = hand.tcp(lift)
    yield from arm.move(p, rr, duration=1.2)
    hand.Rcmd = R_DEV.copy()
    yield from hand.go(c_dev[0] - 0.5 * BAR_L, 0.8, dz=60.0)
    while not LOG.get("closed") and VARIANT != "right_only":
        yield from arm.wait(0.2)
    yield from hand.go(TIP_START, 2.0, dz=15.0)
    yield from hand.go(TIP_START, 1.0)
    yield from hand.servo(TIP_START)
    yield from hand.go(TIP_NEAR, 1.5)
    yield from hand.servo(TIP_NEAR)
    for x in (112.0, 104.0, 97.0, TIP_END):
        yield from hand.go(x, 1.2)
        yield from hand.servo(x, tries=2)
    LOG["bar_seated_at"] = round(ctx.t, 2)
    yield from arm.wait(0.5)
    yield from arm.grip(0.024, 0.8)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.10]), duration=1.2)
    LOG["right_done"] = True


def demo_script(ctx):
    LOG.clear()
    LOG["variant"] = VARIANT
    CTX[:] = [ctx]
    DIAG.clear()
    if VARIANT == "nothing":
        yield from ctx.wait(8.0)
        return
    if VARIANT == "left_only":
        LOG["right_done"] = True
        yield from left(ctx)
        return
    if VARIANT == "right_only":
        LOG["left_done"] = True
        yield from right(ctx)
        return
    yield from ctx.both(left(ctx), right(ctx))
    yield from ctx.wait(4.0)
