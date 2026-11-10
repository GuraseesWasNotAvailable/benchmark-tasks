# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Pin the foot of a five-bar robot leg: join the two lower links so the orange stud clicks into the blue jaw.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

A five-bar leg for a small direct-drive quadruped hangs from a grey aluminium hip block on a stand, its plane facing
the robot (every bearing axis points at the robot, along x):

- two hip bearings 100 mm apart (y +-0.05) on the block, 255 mm above the table;
- on each, a black 3D-printed thigh (75 mm bearing to bearing, 40 g) and on its knee bearing a carbon shin (110 mm
  from the knee to the foot point). The motors are not fitted, so every bearing turns freely (0.002 N m);
- the left shin's foot end carries an orange steel stud (8 mm, with a 14 mm head) pointing back through the right
  shin's layer; the right shin's foot end carries a blue jaw: a 9 x 9 mm pocket whose mouth faces the left chain,
  with a 16 mm guide lip, closed by a sprung steel tongue with a 45 deg lead-in that clicks shut behind the stud.

At reset both chains hang straight down, feet 100 mm apart. Neither chain can reach the other's hanging foot (at full
stretch it stops 25 mm short), and a held chain cannot get round the other one to draw it in, because its stud runs
through the other chain's layers. A free chain gives way to any push (its weight holds it back with 0.4 N at most),
while the tongue's spring starts at 1.0 N and rises to 1.65 N over the 6.5 mm the stud pushes it back through the
lead-in. So the stud goes in only when both shins are held and pressed together: each arm bends its chain knee out
into the leg's diamond and the two press the stud into the jaw against each other.

Goal: the stud snapped into the jaw (within 1.5 mm of the pocket centre, tongue home), both grippers let go.
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

# -- geometry (linkage frame = world axes, origin midway between the hip bearings) -------------------------------------
LEG_X = 0.30  # leg plane (world x of the frame origin)
HIP_H = 0.255  # hip bearings above the table
ORIGIN = (LEG_X, 0.0, TOP + HIP_H)
HIP_Y = 0.050  # hip bearings at y +-50 mm
THIGH, SHIN = 0.075, 0.110
X_TL, X_SL, X_SR, X_TR = -0.015, -0.005, 0.005, 0.015  # layer centres along x (left thigh .. right thigh)
THICK = 0.008
FOOT_Z = -(THIGH + SHIN)  # foot points hang 185 mm below the hip line
F_CLOSE = 0.150  # the demo closes the loop with the foot 150 mm below the hip line, knees out

RS = 0.004  # stud radius
PK = 0.0045  # pocket half-size (0.5 mm play around the stud)
WALL = 0.004
TIP = -0.0025  # tongue tip across the mouth at rest (jaw b, m)
T_THK = 0.0015  # tongue strip thickness
RAMP_L = 0.012
LATCH_K, LATCH_REST, LATCH_TRAVEL = 100.0, -0.010, 0.010  # N/m, set-point (m: 1.0 N preload), travel (m)
RAIL = 0.016  # the jaw's lower lip runs 16 mm out past the pocket centre as a guide rail

GREY = (0.62, 0.64, 0.67)
BLACK = (0.10, 0.10, 0.11)
CARBON = (0.20, 0.20, 0.22)
ORANGE = (0.95, 0.45, 0.05)
BLUE = (0.15, 0.35, 0.85)
STEEL = (0.75, 0.76, 0.78)


def chain_angles(foot_yz, side: int) -> tuple[float, float]:
    """Hip and knee angles (rad, about +x: + swings a hanging link toward +y) that put a chain's foot point at
    ``foot_yz`` (m, in-plane, from the frame origin), knee out (away from the leg's centre). ``side`` +1 left."""
    hy = side * HIP_Y
    dy, dz = foot_yz[0] - hy, foot_yz[1]
    d = math.hypot(dy, dz)
    d = min(max(d, abs(SHIN - THIGH) + 1e-6), THIGH + SHIN - 1e-6)
    psi = math.atan2(dy, -dz)
    alpha = math.acos(max(-1.0, min(1.0, (THIGH**2 + d**2 - SHIN**2) / (2 * THIGH * d))))
    th1 = psi + side * alpha
    ky, kz = hy + THIGH * math.sin(th1), -THIGH * math.cos(th1)
    th2 = math.atan2(foot_yz[0] - ky, -(foot_yz[1] - kz))
    return th1, th2 - th1


_H, _K = chain_angles((0.0, -F_CLOSE), -1)
JAW_DEG = math.degrees(_H + _K)  # the right shin's tilt in the demo's closing pose: its jaw mouth then faces +y
_RJ = lk.axis_rotation((1, 0, 0), math.radians(-JAW_DEG))  # jaw axes in the hanging right shin: a = mouth, b = tongue
JAW_A, JAW_B = _RJ @ np.array([0.0, 1.0, 0.0]), _RJ @ np.array([0.0, 0.0, 1.0])
FOOT_R = np.array([X_SR, 0.0, -SHIN])  # jaw centre in the right shin's frame (origin at its knee)
STUD_L = np.array([X_SR, 0.0, -SHIN])  # stud centre (on the jaw layer) in the left shin's frame


def _jaw(size_a, size_b, a, b, extra_deg=0.0, origin=FOOT_R, color=BLUE, thick=THICK):
    """A box in the jaw's (a, b) plane: centre (a, b) m from the pocket centre, sides along a and b turned by
    ``extra_deg``, in a link frame whose jaw centre sits at ``origin``."""
    p = np.asarray(origin, dtype=float) + _RJ @ np.array([0.0, a, b])
    return lk.box((thick, size_a, size_b), tuple(p), rpy_deg=(extra_deg - JAW_DEG, 0.0, 0.0), color=color,
                  contact_offset=0.001)


_S2 = math.sqrt(0.5)
_t = lk.Tree("leg", pos=ORIGIN, yaw_deg=0.0, fixed=True, color=GREY)
_t.link("hip", [
    lk.box((0.040, 0.170, 0.045), (0.042, 0.0, 0.0025)),  # hip block behind the links
    lk.cylinder(0.004, 0.0435, (0.00025, HIP_Y, 0.0), axis="X", color=STEEL),  # left hip axle, through to the front
    lk.cylinder(0.004, 0.0115, (0.01625, -HIP_Y, 0.0), axis="X", color=STEEL),  # right hip axle
    lk.box((0.030, 0.040, HIP_H - 0.020), (0.045, 0.0, -0.020 - (HIP_H - 0.020) / 2)),  # stand post
    lk.box((0.090, 0.160, 0.008), (0.065, 0.0, -HIP_H + 0.004)),  # stand foot on the table
], mass=1.5, friction=(0.4, 0.3))
for s, name in ((1, "l"), (-1, "r")):
    xt = X_TL if s > 0 else X_TR
    _t.link(f"thigh_{name}", [lk.box((THICK, 0.016, THIGH + 0.016), (xt, 0.0, -THIGH / 2))], mass=0.040,
            at=(0.0, s * HIP_Y, 0.0), color=BLACK, friction=(0.5, 0.4))
_t.link("shin_l", [
    lk.box((THICK, 0.012, SHIN + 0.012), (X_SL, 0.0, -SHIN / 2)),  # carbon bar, knee to 6 mm past the foot point
    lk.cylinder(RS, 0.0115, (0.00375, 0.0, -SHIN), axis="X", color=ORANGE),  # stud, x -2 to 9.5 mm
    lk.cylinder(0.007, 0.003, (0.011, 0.0, -SHIN), axis="X", color=ORANGE),  # its head behind the jaw
], mass=0.038, at=(0.0, HIP_Y, -THIGH), color=CARBON, friction=(0.4, 0.3), contact_offset=0.001)
_BAR_END = 0.011  # the right bar stops 11 mm short of the jaw centre (clear of the pocket)
_CH = np.array([RAIL, -PK]) + 0.0025 * np.array([_S2, -_S2]) + 0.001 * np.array([-_S2, -_S2])  # rail's lead-in
_t.link("shin_r", [
    lk.box((THICK, 0.012, SHIN - _BAR_END + 0.006), (X_SR, 0.0, -(SHIN - _BAR_END - 0.006) / 2)),
    _jaw(WALL, 0.021, -PK - WALL / 2, 0.002),  # bottom wall: a -8.5..-4.5, b -8.5..12.5 mm
    _jaw(RAIL + PK + WALL, WALL, (RAIL - PK - WALL) / 2, -PK - WALL / 2),  # lower wall and rail: a -8.5..16 mm
    _jaw(0.013, 0.008, -0.002, PK + 0.004),  # housing over the tongue: a -8.5..4.5, b 4.5..12.5 mm
    _jaw(0.005, 0.002, float(_CH[0]), float(_CH[1]), -45.0),  # 45 deg lead-in on the rail's end
], mass=0.042, at=(0.0, -HIP_Y, -THIGH), color=CARBON, friction=(0.4, 0.3), contact_offset=0.001)
_TONGUE_ORIGIN = np.array([X_SR, 0.0, 0.0])
_q0 = np.array([PK + T_THK, TIP])  # outer tip corner of the strip (a, b)
_rc = _q0 + 0.5 * RAMP_L * np.array([_S2, _S2]) + 0.5 * T_THK * np.array([-_S2, _S2])
_t.link("tongue", [
    _jaw(T_THK, 0.020, PK + T_THK / 2, TIP + 0.010, origin=_TONGUE_ORIGIN, color=STEEL, thick=0.007),  # strip
    _jaw(RAMP_L, T_THK, float(_rc[0]), float(_rc[1]), 45.0, origin=_TONGUE_ORIGIN, color=STEEL, thick=0.007),
], mass=0.003, at=(0.0, -HIP_Y, FOOT_Z), color=STEEL, friction=(0.15, 0.10), contact_offset=0.001)

_J = dict(axis=(1.0, 0.0, 0.0), limits_deg=(-150.0, 150.0), friction=0.002, damping=0.001)
_t.revolute("hip_l", "hip", "thigh_l", anchor=(0.0, HIP_Y, 0.0), **_J)
_t.revolute("knee_l", "thigh_l", "shin_l", anchor=(0.0, HIP_Y, -THIGH), **_J)
_t.revolute("hip_r", "hip", "thigh_r", anchor=(0.0, -HIP_Y, 0.0), **_J)
_t.revolute("knee_r", "thigh_r", "shin_r", anchor=(0.0, -HIP_Y, -THIGH), **_J)
_t.prismatic("latch", "shin_r", "tongue", anchor=(0.0, -HIP_Y, FOOT_Z), axis=tuple(JAW_B),
             limits=(0.0, LATCH_TRAVEL), rest=LATCH_REST, stiffness=LATCH_K, damping=0.5)
LEG = _t.build()

OBJECTS = [LEG]

# -- scoring -----------------------------------------------------------------------------------------------------------
# Everything is measured from link poses (what a camera, a calliper and a feeler see), every control step from reset.
SNAP_MM = 1.5  # stud centre within 1.5 mm of the pocket centre
LATCH_HOME_MM = 1.0  # tongue back within 1 mm of its stop
NEAR_MM = 10.0  # "feet together": stud within 10 mm of the jaw centre
HOLD_GAP, HOLD_OPEN = 0.004, 0.030  # a gripper holds a shin: a pad within 4 mm of it, opening at most 30 mm
CLEAR_M, TCP_STILL = 0.030, 0.005
SNAP_S = RELEASE_S = 1.0
EPS = 1e-6


def jaw_frame(poses) -> np.ndarray:
    """4x4 world pose of the jaw (origin at the pocket centre; x = mouth direction a, y = tongue direction b)."""
    R = poses["shin_r"][:3, :3]
    M = np.eye(4)
    M[:3, 0], M[:3, 1] = R @ JAW_A, R @ JAW_B
    M[:3, 2] = np.cross(M[:3, 0], M[:3, 1])
    M[:3, 3] = R @ FOOT_R + poses["shin_r"][:3, 3]
    return M


def stud_point(poses) -> np.ndarray:
    return poses["shin_l"][:3, :3] @ STUD_L + poses["shin_l"][:3, 3]


def stud_in_jaw(poses) -> tuple[float, float]:
    """The stud's centre in the jaw (mm along the mouth direction a and across it b), from the two shin poses alone."""
    M = jaw_frame(poses)
    p = M[:3, :3].T @ (stud_point(poses) - M[:3, 3])
    return 1000.0 * float(p[0]), 1000.0 * float(p[1])


def jaw_reading(poses) -> tuple[float, float, float]:
    """The stud's centre in the jaw (mm, as stud_in_jaw) and the tongue's travel back from its stop (mm)."""
    a, b = stud_in_jaw(poses)
    latch = float((poses["tongue"][:3, 3] - poses["shin_r"][:3, 3] - poses["shin_r"][:3, :3] @ np.array(
        [0.0, 0.0, FOOT_Z + THIGH])) @ (poses["shin_r"][:3, :3] @ JAW_B))
    return a, b, 1000.0 * latch


def is_snapped(a: float, b: float, latch: float) -> bool:
    return math.hypot(a, b) <= SNAP_MM + EPS and latch <= LATCH_HOME_MM + EPS


def holder(tool, leg, link: str) -> bool:
    return tool.opening <= HOLD_OPEN and lk.clearance(tool, leg, [link]) <= HOLD_GAP


def measure(leg, tools) -> dict:
    """One sample of everything the scorer uses (mm, m, m/s)."""
    a, b, latch = jaw_reading(leg.poses)
    return {
        "a": a, "b": b, "latch": latch, "dist": math.hypot(a, b), "snapped": is_snapped(a, b, latch),
        "hold_l": sorted(s for s, t in tools.items() if holder(t, leg, "shin_l")),
        "hold_r": sorted(s for s, t in tools.items() if holder(t, leg, "shin_r")),
        "clear": {s: lk.clearance(t, leg) for s, t in tools.items()},
        "tcp_speed": {s: float(t.speed) for s, t in tools.items()},
    }


def both_held(m) -> bool:
    """Each shin held by its own gripper."""
    return any(g != h for g in m["hold_l"] for h in m["hold_r"])


class Record:
    """Whole-run record: a linkkit History updated every control step and cleared only by the simulator reset, plus
    the latest sample. Pure Python, so the tests can feed it synthetic samples."""

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
        H.low("dist_min", m["dist"])
        H.ever("together", both_held(m) and m["dist"] <= NEAR_MM + EPS)
        H.ever("snapped_any", m["snapped"])
        H.held("snapped", m["snapped"])
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M - EPS
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Score and reason, from the top rung down."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if m["snapped"]:
        ends = {
            "snapped 1 s": H.runs.get("snapped", 0.0) >= SNAP_S - EPS,
            "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS,
        }
        if all(ends.values()):
            return 1.0, f"foot pinned (stud {m['dist']:.2f} mm from the jaw centre, tongue home), both grippers let go"
        return 0.75, "foot pinned at the end; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if "snapped_any" in H.evers:
        return 0.5, f"the stud clicked in at {H.evers['snapped_any']:.2f} s but is out at the end"
    if "together" in H.evers:
        return 0.25, "both shins held with the stud at the jaw, never clicked in"
    return 0.0, f"the stud never reached the jaw with both shins held (closest {H.lows.get('dist_min', 0):.1f} mm)"


REC = Record()


@lk.on_reset
def _reset(tick):
    REC.reset()


@lk.on_step
def _track(tick):
    leg = tick.states["leg"]
    m = measure(leg, tick.tools)
    REC.update(m)
    if REC.n % 5 == 0:  # diagnostics every 0.25 s: the stud in the jaw, holds, clearances, thigh and shin angles
        P = leg.poses
        ang = [round(math.degrees(math.atan2(P[k][2, 1], P[k][1, 1])), 1) for k in ("thigh_l", "shin_l", "thigh_r",
                                                                                   "shin_r")]
        REC.trace.append((round(tick.t, 2), round(m["a"], 2), round(m["b"], 2), round(m["latch"], 2),
                          int(m["snapped"]), "".join(x[0] for x in m["hold_l"]) or "-",
                          "".join(x[0] for x in m["hold_r"]) or "-",
                          [round(1000 * m["clear"][s]) for s in ("left", "right")], ang))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo -----------------------------------------------------------------------------------------------------
# linkdemo.py runs demo_script(ctx). It reads only what a camera could measure (the thigh and shin poses, and from
# the two shins the stud's place in the jaw) and the robot's own grippers; never the tongue, the latch state, joint
# readouts or the record. It decides the stud is home from its depth in the pocket alone. Each arm grips its shin
# from the leg's outside, pads on the bar's front and back faces, and then moves the shin only along poses its chain
# can take (chain kinematics from the hip bearing), so the held chain bends instead of fighting its bearings.

VARIANTS = ("", "nothing", "one_arm", "one_arm_right", "push_free")
SIDES = {"left": (1, "hip_l", "knee_l", "shin_l", X_SL), "right": (-1, "hip_r", "knee_r", "shin_r", X_SR)}
U_GRIP = 0.060  # grasp 60 mm below the knee (50 mm above the foot point)
GAMMA = math.radians(45.0)  # approach from the leg's outside, 45 deg below horizontal (hanging shin)
OPEN_W, SHUT_W = 0.022, 0.002  # bar 8 mm thick: 6 mm squeeze (about 15 N a pad)
STAGE_MM, BIAS_MM, SEAT_MM = 22.0, -0.2, -5.0  # stud targets in the jaw (mm): staging, rail side, press
PRESS_MM = 3.0  # the right arm presses its jaw this far onto the stud
FLAGS: dict = {}
LOG: dict = {}
G: dict = {}
OFF: dict = {}
QCMD: dict = {}


_READY_ROT = np.array([[0.0, -0.565, 0.825], [-1.0, 0.0, 0.0], [0.0, -0.825, -0.565]])  # the arms' ready grip


def _grip_rot(approach, closing) -> np.ndarray:
    """TCP rotation: z along the approach, x (pads closing) along ``closing``; of the two mirror choices the one
    nearer the ready grip, which keeps the wrist inside its limits."""
    z = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    x = np.asarray(closing, dtype=float)
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    best = None
    for sx in (1.0, -1.0):
        r = np.column_stack([sx * x, np.cross(z, sx * x), z])
        err = float(np.arccos(np.clip((np.trace(_READY_ROT.T @ r) - 1) / 2, -1, 1)))
        if best is None or err < best[0]:
            best = (err, r)
    return best[1]


def _inplane(p) -> np.ndarray:
    """World point -> (y, z) from the frame origin."""
    q = np.asarray(p, dtype=float) - np.asarray(ORIGIN)
    return np.array([q[1], q[2]])


def _shin_pose(side: str, h: float, k: float) -> np.ndarray:
    _, hj, kj, link, _ = SIDES[side]
    return LEG.link_poses({hj: h, kj: k})[link]


def _seen_q(seen, side: str) -> tuple[float, float]:
    _, hj, kj, _, _ = SIDES[side]
    return math.radians(seen.angle_deg("leg", hj)), math.radians(seen.angle_deg("leg", kj))


def _tool_pose(seen, side: str) -> np.ndarray:
    t = seen.tools[side]
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = t.rot, t.tcp
    return M


def grasp_plan(shin_pose, side: str):
    """TCP pose for gripping a shin from the outside of the leg: pads closing along the bearing axis (x) on the
    bar's front and back faces, the approach in the leg plane, tilted GAMMA below horizontal toward the leg's centre
    (for a hanging shin), pad tips 3 mm past the bar's far edge, U_GRIP below the knee."""
    sgn, _, _, _, xc = SIDES[side]
    R = shin_pose[:3, :3]
    c = R @ np.array([xc, 0.0, -U_GRIP]) + shin_pose[:3, 3]
    app = R @ np.array([0.0, -sgn * math.cos(GAMMA), -math.sin(GAMMA)])
    rot = _grip_rot(app, (1.0, 0.0, 0.0))
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = rot, c + app * (0.006 + 0.003 - 0.013)
    return M


def grasp_shin(ctx, side: str):
    """Grip the hanging shin from the outside (see grasp_plan); records the grasp offset and the arm's sag."""
    arm = ctx.arm(side)
    link = SIDES[side][3]
    seen = ctx.observe()
    M = grasp_plan(seen.pose("leg", link), side)
    app = M[:3, 2]
    yield from arm.move(M[:3, 3] - 0.06 * app, M[:3, :3], width=OPEN_W, duration=2.5)
    yield from arm.move(M[:3, 3], M[:3, :3], duration=1.5)
    yield from arm.grip(SHUT_W, 0.8)
    yield from arm.wait(0.4)
    seen = ctx.observe()
    G[side] = np.linalg.inv(seen.pose("leg", link)) @ _tool_pose(seen, side)
    OFF[side] = arm.pos - seen.tools[side].tcp
    QCMD[side] = _seen_q(seen, side)
    LOG[f"grasp_{side}"] = {"sag_mm": [round(1000 * v, 2) for v in OFF[side]]}


def chain_to(ctx, side: str, foot_yz, duration: float):
    """Move the held shin so its foot point goes to ``foot_yz`` (in-plane, m), along the chain's own joint path."""
    arm = ctx.arm(side)
    sgn = SIDES[side][0]
    q0 = np.array(QCMD[side])
    q1 = np.array(chain_angles(foot_yz, sgn))
    QCMD[side] = tuple(q1)

    def at(s):
        e = min(max(s, 0.0), 1.0)
        q = q0 + e * e * (3 - 2 * e) * (q1 - q0)
        T = _shin_pose(side, float(q[0]), float(q[1])) @ G[side]
        return T[:3, 3] + OFF[side], T[:3, :3], arm.width

    yield from arm.follow(at, duration)


def _foot_of(side: str, q) -> np.ndarray:
    """In-plane foot point (m) of a chain at joint angles ``q``."""
    P = _shin_pose(side, float(q[0]), float(q[1]))
    return _inplane(P[:3, :3] @ np.array([0.0, 0.0, -SHIN]) + P[:3, 3])


def _seen_jaw(seen):
    """The jaw frame, the stud point and the stud in the jaw (mm), from the two shin poses a camera sees.
    The tongue and the scorer's latch reading are never read."""
    poses = {k: seen.pose("leg", k) for k in ("shin_l", "shin_r")}
    return jaw_frame(poses), stud_point(poses), stud_in_jaw(poses)


def let_go(arm):
    yield from arm.grip(OPEN_W, 0.5)
    yield from arm.retreat(0.09, 1.2)
    yield from arm.move(arm.pos + [-0.04, 0.0, 0.05], duration=1.0)


def _reachable(foot_yz, side: int):
    """Clip a foot target into the chain's reach (2 mm short of full stretch)."""
    h = np.array([side * HIP_Y, 0.0])
    v = np.asarray(foot_yz, dtype=float) - h
    r = np.linalg.norm(v)
    lim = THIGH + SHIN - 0.002
    return h + v * (lim / r) if r > lim else np.asarray(foot_yz, dtype=float)


def one_arm(ctx, side: str):
    """The natural one-arm plan: hold one shin and chase the other chain's foot with it, pressing home."""
    sgn = SIDES[side][0]
    yield from grasp_shin(ctx, side)
    for k in range(4):
        M, stud, (a, b) = _seen_jaw(ctx.observe())
        LOG.setdefault("chase", []).append([round(a, 1), round(b, 1)])
        if side == "left":  # stud toward the jaw: in front of the mouth, then home
            out = _inplane(M[:3, 3] + M[:3, 0] * (STAGE_MM / 1000))
            home = _inplane(M[:3, 3] + M[:3, 0] * (SEAT_MM / 1000))
        else:  # jaw onto the stud: mouth in front of it, then over it
            out = _inplane(stud - M[:3, 0] * (STAGE_MM / 1000))
            home = _inplane(stud - M[:3, 0] * (SEAT_MM / 1000))
        yield from chain_to(ctx, side, _reachable(out, sgn), 2.0)
        yield from chain_to(ctx, side, _reachable(home, sgn), 2.0)
        yield from ctx.arm(side).wait(0.3)
    yield from let_go(ctx.arm(side))


def demo_script(ctx):
    FLAGS.clear()
    LOG.clear()
    yield from ctx.wait(0.5)
    v = VARIANT
    if v == "nothing":
        yield from ctx.wait(3.0)
        return
    if v in ("one_arm", "one_arm_right"):
        yield from one_arm(ctx, "left" if v == "one_arm" else "right")
        yield from ctx.wait(2.0)
        return
    if v == "push_free":  # hold the left shin and shove the free right shin with the stud, 15 mm past first touch
        yield from grasp_shin(ctx, "left")
        yield from chain_to(ctx, "left", np.array([-0.030, -0.150]), 3.0)
        yield from chain_to(ctx, "left", np.array([-0.055, -0.145]), 3.0)
        yield from ctx.left.wait(1.0)
        yield from let_go(ctx.left)
        yield from ctx.wait(2.0)
        return

    jaw_home = np.array([0.0, -F_CLOSE])
    stage = jaw_home + np.array([STAGE_MM / 1000, 0.0])

    def right():
        yield from grasp_shin(ctx, "right")
        FLAGS["r_grip"] = True
        while not FLAGS.get("l_grip"):
            yield
        yield from chain_to(ctx, "right", jaw_home, 3.0)  # bend the right chain knee-out: jaw mouth faces left
        FLAGS["r_set"] = True
        while not FLAGS.get("press"):
            yield
        yield from chain_to(ctx, "right", jaw_home + np.array([PRESS_MM / 1000, 0.0]), 1.0)  # equal and opposite
        while not FLAGS.get("done"):
            if FLAGS.pop("more", False):
                q = QCMD["right"]
                yield from chain_to(ctx, "right", _foot_of("right", q) + np.array([0.0015, 0.0]), 0.8)
            yield
        yield from ctx.right.wait(0.3)
        yield from let_go(ctx.right)

    def left():
        yield from grasp_shin(ctx, "left")
        FLAGS["l_grip"] = True
        while not FLAGS.get("r_grip"):
            yield
        yield from chain_to(ctx, "left", stage, 3.0)  # stud 20 mm in front of the mouth
        while not FLAGS.get("r_set"):
            yield
        yield from ctx.left.wait(0.4)
        target = stage.copy()
        for k in range(3):  # line the stud up with the mouth (seen poses)
            M, stud, (a, b) = _seen_jaw(ctx.observe())
            want = M[:3, 3] + M[:3, 0] * (STAGE_MM / 1000) + M[:3, 1] * (BIAS_MM / 1000)
            err = _inplane(want) - _inplane(stud)
            LOG.setdefault("align", []).append([round(a, 2), round(b, 2)])
            if np.linalg.norm(err) < 0.0003:
                break
            target = target + err
            yield from chain_to(ctx, "left", target, 0.8)
            yield from ctx.left.wait(0.3)
        M, _, _ = _seen_jaw(ctx.observe())
        mouth = np.array([M[1, 0], M[2, 0]])
        target = target - mouth * ((STAGE_MM - SEAT_MM) / 1000)
        FLAGS["press"] = True
        yield from chain_to(ctx, "left", target, 3.0)  # press the stud home along the mouth
        for k in range(3):
            yield from ctx.left.wait(0.4)
            _, _, (a, b) = _seen_jaw(ctx.observe())
            LOG.setdefault("seat", []).append([round(a, 2), round(b, 2)])
            if math.hypot(a, b) <= SNAP_MM:  # stud home in the pocket, seen from the shin poses only
                break
            target = target - mouth * 0.0015
            FLAGS["more"] = True
            yield from chain_to(ctx, "left", target, 0.8)
        FLAGS["done"] = True
        yield from ctx.left.wait(0.3)
        yield from let_go(ctx.left)

    yield from ctx.both(left(), right())
    yield from ctx.wait(2.5)


def demo_report() -> dict:
    s, why = ladder(REC)
    m = REC.last or {}
    H = REC.H
    return {
        "score": s, "reason": why, "variant": VARIANT, "jaw_deg": round(JAW_DEG, 2),
        "final": {k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items()
                  if k not in ("clear", "tcp_speed")},
        "clear_mm": {k: round(1000 * v, 1) for k, v in m.get("clear", {}).items()},
        "evers": H.evers, "runs": {k: round(v, 2) for k, v in H.runs.items()},
        "lows": {k: round(v, 2) for k, v in H.lows.items()},
        "log": LOG,
        "trace (t, a, b, latch, snapped, hold_l, hold_r, clear_mm, deg thigh_l shin_l thigh_r shin_r)": REC.trace,
    }
