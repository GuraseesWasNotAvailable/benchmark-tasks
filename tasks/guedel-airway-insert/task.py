# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Open an airway-trainer manikin's airway with an oral (Guedel) airway.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- An airway-trainer head (skin-tone vinyl over rigid foam, about 200 x 150 x 80 mm; its mouth cavity and tongue are
  about 1.6x adult width) lies face up, fixed to the table between the arms. Its chin points back toward the robot's
  left arm and the top of its head points forward and to the right. The lips are 80 mm above the table, centred at
  (0.17, -0.03).
- The lower jaw is hinged and sprung: a spring holds the mouth shut (no gap between the lips) unless the chin is held
  open. Inside the mouth a pink silicone tongue base sits on a sprung hinge about 50 mm below the lips, 2 mm above the
  back wall of the throat.
- A green size-2 Guedel oropharyngeal airway (80 mm, PVC, 12 g) hangs flange-up in the pocket of a grey ABS kit tray
  at (0.24, -0.21), to the right of the head. Its curved end points toward the chin, the way it finally sits. Seated
  with its flange on the lips, the end of its curve rests on the back wall of the throat.

Goal: the airway in the mouth with its flange resting at the lips and its curved end behind the tongue base, holding
the tongue base clear of the back of the throat; the jaw closed on the airway's bite block; both grippers let go.
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

# -- head frame ------------------------------------------------------------------------------------------------------
# Head frame (mm): origin at the centre of the lip plane; +u caudal (toward the chin) = world (-0.788, 0.616, 0),
# +v the patient's left = world (-0.616, -0.788, 0), +w up. The head linkage is built in this frame (yaw 142 deg).
HEAD_POS = (0.17, -0.03, TOP + 0.080)
HEAD_YAW = 142.0
HEAD_FRAME = np.eye(4)
HEAD_FRAME[:3, :3] = lk.axis_rotation((0, 0, 1), math.radians(HEAD_YAW))
HEAD_FRAME[:3, 3] = HEAD_POS


def head_point(p_mm, frame=HEAD_FRAME) -> np.ndarray:
    """World point (m) of a head-frame point (mm)."""
    return frame[:3, :3] @ (np.asarray(p_mm, dtype=float) * MM) + frame[:3, 3]


def to_head(p_world, frame=HEAD_FRAME) -> np.ndarray:
    """Head-frame point (mm) of a world point (m)."""
    return 1000.0 * (frame[:3, :3].T @ (np.asarray(p_world, dtype=float) - frame[:3, 3]))


SKIN = (0.87, 0.72, 0.62)
PINK = (0.85, 0.45, 0.50)
GREEN = (0.15, 0.65, 0.30)
TRAY_GREY = (0.55, 0.57, 0.60)

# (size u v w, centre u v w) in mm; boxes of the fixed skull. The cavity under the lips spans u -48..+46, |v| <= 40,
# w -64..-12; the mouth opening is u -14..+6, |v| <= 22, closed by the jaw when it is shut.
SKULL_BOXES = (
    ((200, 150, 16), (-10, 0, -72)),  # base plate under the head; its top face (w -64) is the back wall of the throat
    ((62, 150, 64), (-79, 0, -32)),  # forehead and palate vault (the cavity's cranial wall at u -48)
    ((34, 80, 12), (-31, 0, -6)),  # palate shelf with the upper lip and teeth (top w 0; mouth edge at u -14)
    ((54, 35, 64), (-21, 57.5, -32)),  # left cheek wall (the cavity's side wall at v +40, up to the lip corner)
    ((54, 35, 64), (-21, -57.5, -32)),  # right cheek wall
    ((20, 18, 12), (-4, 31, -6)),  # left lip corner (the flange rests on the lip corners)
    ((20, 18, 12), (-4, -31, -6)),  # right lip corner
    ((32, 32, 18), (-63, 0, 9)),  # nose
    ((42, 80, 16), (69, 0, -56)),  # tongue root and hyoid block (the floor caudal of the tongue)
)
JAW_BOXES = (
    ((20, 43, 10), (-4, 0, -7)),  # lower lip and teeth bar (top w -2; meets the upper lip at u -14 when shut)
    ((24, 50, 10), (18, 0, -7)),  # chin plate
)
# (size, centre, rpy deg) of the tongue base; at rest its outline is T (4, -50) front top edge, B (24, -62) lowest
# corner, U (44, -55), D (44, -43) in (u, w). The face T-B is a ramp facing down and toward the head top (31 deg).
TONGUE_BOXES = (
    ((24, 72, 6), (15.5, 0, -53.4), (0, 31, 0)),  # under-ramp T-B
    ((41, 72, 6), (24.5, 0, -49.5), (0, -9.9, 0)),  # top face T-D
    ((21, 72, 6), (33, 0, -55.7), (0, -19.3, 0)),  # underside B-U
    ((6, 72, 12), (41, 0, -49), (0, 0, 0)),  # rear
    ((24, 72, 8), (28, 0, -51), (0, 0, 0)),  # core fill
)
TONGUE_MASS = 0.025
TONGUE_HINGE = (42.0, 0.0, -50.0)  # mm
TONGUE_K = 0.025  # N m/rad
JAW_HINGE = (-15.0, 0.0, -72.0)  # mm, the temporomandibular joint


def _boxes(rows):
    out = []
    for row in rows:
        size, centre = row[0], row[1]
        rpy = row[2] if len(row) > 2 else (0, 0, 0)
        out.append(lk.box(np.asarray(size, float) * MM, np.asarray(centre, float) * MM, rpy_deg=rpy))
    return out


def _tongue_com() -> np.ndarray:
    """Volume-weighted centre of the tongue colliders (mm), the centre of mass PhysX derives from them."""
    vols = np.array([np.prod(r[0]) for r in TONGUE_BOXES], dtype=float)
    cents = np.array([r[1] for r in TONGUE_BOXES], dtype=float)
    return (vols[:, None] * cents).sum(0) / vols.sum()


TONGUE_COM = _tongue_com()
# The spring rest pre-compensates the tongue's own weight so that it settles at 0 deg (base 2 mm off the throat wall).
TONGUE_REST_DEG = -math.degrees(TONGUE_MASS * 9.81 * (TONGUE_HINGE[0] - TONGUE_COM[0]) * MM / TONGUE_K)

_t = lk.Tree("head", pos=HEAD_POS, yaw_deg=HEAD_YAW, fixed=True, color=SKIN)
_t.link("skull", _boxes(SKULL_BOXES), mass=2.0, friction=(0.3, 0.25), contact_offset=0.001)  # wet, lubricated
_t.link("jaw", _boxes(JAW_BOXES), mass=0.10, friction=(0.9, 0.8), contact_offset=0.001)
_t.link("tongue", _boxes(TONGUE_BOXES), mass=TONGUE_MASS, color=PINK, friction=(0.15, 0.1), contact_offset=0.001,
        com=tuple(TONGUE_COM * MM))
# + opens the mouth (the chin swings caudal and down); the spring pulls toward -12 deg, so at its 0 deg stop the jaw
# is shut with a 0.063 N m preload (0.04 N m net of the jaw's weight).
_t.revolute("tmj", "skull", "jaw", anchor=np.asarray(JAW_HINGE) * MM, axis=(0, 1, 0), limits_deg=(0, 35),
            rest_deg=-12.0, init_deg=0.0, stiffness=0.3, damping=0.01)
# + pushes the tongue base back toward the throat wall (its stop at +6.7 deg is the wall), - lifts it forward.
_t.revolute("tongue_base", "skull", "tongue", anchor=np.asarray(TONGUE_HINGE) * MM, axis=(0, -1, 0),
            limits_deg=(-42, 6.7), rest_deg=TONGUE_REST_DEG, init_deg=0.0, stiffness=TONGUE_K, damping=0.002)
HEAD = _t.build()

# -- the airway: one rigid PVC body. Frame: origin at the flange-bottom centre, +z out of the mouth along the bite
# block, +x the way the curve (tip) points, y across the 56 mm flange. --------------------------------------------
BITE = 31.0  # mm from the flange bottom to the start of the curve: seated on the lips, the tip rests on the throat wall
_shapes = [lk.box((0.022, 0.056, 0.007), (0, 0, 0.0035))]
for y in (-0.003, 0.003):  # bite block: two capsules give a 16 x 10 mm stadium section, 31 mm long
    _shapes.append(lk.capsule(0.005, (BITE - 10.0) * MM, (0, y, -BITE / 2 * MM), axis="Z"))
for k in range(8):  # curve: centreline (28 - 28 cos t, -31 - 28 sin t) mm, t 0..90 deg, 8 segments
    t0, t1 = math.radians(11.25 * k), math.radians(11.25 * (k + 1))
    pa = np.array([28 - 28 * math.cos(t0), -BITE - 28 * math.sin(t0)])
    pb = np.array([28 - 28 * math.cos(t1), -BITE - 28 * math.sin(t1)])
    mid = (pa + pb) / 2
    pitch = math.degrees(math.atan2(-(pb - pa)[1], (pb - pa)[0]))
    for y in (-0.003, 0.003):
        _shapes.append(lk.capsule(0.005, 0.0056, (mid[0] * MM, y, mid[1] * MM), axis="X", rpy_deg=(0, pitch, 0)))
AIRWAY_SHAPES = tuple(_shapes)
TIP_LOCAL = np.array([0.028, 0.0, -(BITE + 28.0) * MM])  # the tip end of the curve's centreline, airway frame (m)
AIRWAY_START = (0.240, -0.210, TOP + 0.080 + 0.0005)  # flange bottom just above the tray rim
_t = lk.Tree("airway", pos=AIRWAY_START, yaw_deg=HEAD_YAW, color=GREEN)
_t.link("airway", AIRWAY_SHAPES, mass=0.012, friction=(0.4, 0.3), contact_offset=0.001)
AIRWAY = _t.build()

# -- kit tray (fixed): pocket 60 x 32 mm, 72 mm deep, long side along the airway's curve; the flange bridges the
# pocket's long rims. Tray frame = the airway's frame at the start, dropped to the table. -------------------------
_t = lk.Tree("tray", pos=(AIRWAY_START[0], AIRWAY_START[1], TOP), yaw_deg=HEAD_YAW, fixed=True, color=TRAY_GREY)
_t.link("tray", [
    lk.box((0.074, 0.046, 0.008), (0.008, 0, 0.004)),  # floor
    lk.box((0.007, 0.046, 0.072), (-0.0255, 0, 0.044)),  # end wall behind the bite block
    lk.box((0.007, 0.046, 0.072), (0.0415, 0, 0.044)),  # end wall beyond the tip
    lk.box((0.060, 0.007, 0.072), (0.008, 0.0195, 0.044)),  # long rim (+y)
    lk.box((0.060, 0.007, 0.072), (0.008, -0.0195, 0.044)),  # long rim (-y)
], mass=0.3, friction=(0.6, 0.5), contact_offset=0.001)
TRAY = _t.build()

OBJECTS = [HEAD, AIRWAY, TRAY]

# -- scoring ---------------------------------------------------------------------------------------------------------
T_LOCAL = np.array([4.0, 0.0, -50.0]) * MM  # tongue front top edge (tongue link frame = head frame at rest)
B_LOCAL = np.array([24.0, 0.0, -62.0]) * MM  # tongue base's lowest corner
WALL_W = -64.0  # mm, back wall of the throat

PUSH_GATE_DEG = 3.0  # tongue pushed back this far (base about 1.3 mm toward the wall) at any time -> 0
LIFTED_DEG = -10.0
OPEN_QT_DEG = -15.0
OPEN_GAP_MM = 8.0
TIP_PAST_T_MM = 5.0
SEAT_FW_MM, SEAT_U_MM, SEAT_TOL_MM, SEAT_TILT_DEG = 4.0, -9.0, 8.0, 15.0
ALPHA_TOL_DEG = 25.0
TIP_IN_W_MM = -35.0
JAW_OPEN_DEG = 10.0
JAW_SHUT_DEG = 12.0
STEADY_S, RELEASE_S = 2.0, 1.0
AIRWAY_STILL, AIRWAY_SPIN = 0.003, math.radians(3.0)  # m/s, rad/s
CLEAR_M, TCP_STILL = 0.030, 0.005
HELD_OPENING, HELD_NEAR = 0.060, 0.004  # a closed gripper with its pads on the airway
EPS = 1e-6


def measure(head, airway, tools) -> dict:
    """One sample of everything the scorer uses (head-frame mm and deg; speeds in m/s and rad/s). The airway's speed
    and spin here are the solver's; the on-step record replaces them with pose_rates() between control steps."""
    frame = head.poses["skull"]
    T = to_head(head.point("tongue", T_LOCAL), frame)
    B = to_head(head.point("tongue", B_LOCAL), frame)
    A = airway.poses["airway"]
    R = frame[:3, :3].T @ A[:3, :3]
    F = to_head(A[:3, 3], frame)
    tip = to_head(A[:3, :3] @ TIP_LOCAL + A[:3, 3], frame)
    near, clear = {}, {}
    for side, tool in tools.items():
        near[side] = lk.clearance(tool, airway)
        clear[side] = min(near[side], lk.clearance(tool, head))
    return {
        "q_t": head.deg("tongue_base"),
        "q_j": head.deg("tmj"),
        "gap": float(min(T[2], B[2]) - WALL_W),
        "T_u": float(T[0]),
        "F": tuple(float(v) for v in F),
        "tilt": math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2]))))),
        "alpha": math.degrees(math.atan2(float(R[1, 0]), float(R[0, 0]))),
        "tip": tuple(float(v) for v in tip),
        "held": any(tools[s].opening <= HELD_OPENING and near[s] <= HELD_NEAR for s in tools),
        "aw_speed": float(np.linalg.norm(airway.lin_vels["airway"])),
        "aw_spin": float(np.linalg.norm(airway.ang_vels["airway"])),
        "clear": clear,
        "tcp_speed": {s: float(t.speed) for s, t in tools.items()},
    }


def pose_rates(A_prev, A, dt: float) -> tuple[float, float]:
    """Speed (m/s) and spin (rad/s) of a body from its pose change over ``dt``: what a camera sees. The solver's
    velocity buffers of a body clamped between several contacts carry sub-step jitter (up to 1.5 rad/s for a resting
    airway whose pose does not change), so stillness is judged from poses, not from those buffers."""
    if dt <= 0.0:
        return 0.0, 0.0
    v = float(np.linalg.norm(A[:3, 3] - A_prev[:3, 3])) / dt
    R = A_prev[:3, :3].T @ A[:3, :3]
    sin = 0.5 * float(np.linalg.norm((R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1])))
    return v, math.atan2(sin, (float(np.trace(R)) - 1.0) / 2.0) / dt


def in_mouth(m) -> bool:
    """The airway's flange is at or above the lips and its tip is deep inside the mouth cavity."""
    u, v, w = m["tip"]
    return w <= TIP_IN_W_MM and -48.0 <= u <= 46.0 and abs(v) <= 40.0 and m["F"][2] >= -10.0


def seated(m) -> bool:
    fu, fv, fw = m["F"]
    return fw <= SEAT_FW_MM and abs(fu - SEAT_U_MM) <= SEAT_TOL_MM and abs(fv) <= SEAT_TOL_MM and m["tilt"] <= SEAT_TILT_DEG


def open_airway(m) -> bool:
    """Seated the right way round with its tip behind the tongue base, which it holds lifted clear of the wall."""
    return (seated(m) and abs(m["alpha"]) <= ALPHA_TOL_DEG and m["gap"] >= OPEN_GAP_MM and m["q_t"] <= OPEN_QT_DEG
            and m["tip"][0] >= m["T_u"] + TIP_PAST_T_MM)


class Record:
    """Whole-run record: a linkkit History updated every control step and cleared only by the simulator reset (so
    pulling the airway out and starting again never clears a push-back), plus the latest sample. Pure Python."""

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
        H.ever("pushed_back", m["q_t"] >= PUSH_GATE_DEG)
        H.peak("qt_max", m["q_t"])
        H.low("qt_min", m["q_t"])
        H.low("gap_min", m["gap"])
        H.peak("qj_max", m["q_j"])
        H.ever("entered", m["held"] and in_mouth(m) and m["q_j"] >= JAW_OPEN_DEG)
        H.ever("lifted", m["q_t"] <= LIFTED_DEG and in_mouth(m))
        H.held("open_steady", open_airway(m) and m["aw_speed"] < AIRWAY_STILL and m["aw_spin"] < AIRWAY_SPIN)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Score and the reason. The gate is checked first (any time in the run), then the rungs from the top."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "pushed_back" in H.evers:
        return 0.0, (f"gate: tongue base pushed back toward the throat at {H.evers['pushed_back']:.2f} s "
                     f"(peak {H.peaks['qt_max']:.1f} deg, gate {PUSH_GATE_DEG})")
    if open_airway(m):
        ends = {
            "jaw shut on the bite block": m["q_j"] <= JAW_SHUT_DEG,
            "airway steady 2 s": H.runs.get("open_steady", 0.0) >= STEADY_S - EPS,
            "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS,
        }
        if all(ends.values()):
            return 1.0, "airway open and seated, jaw shut on the bite block, both grippers let go"
        return 0.75, "airway open at the end; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if "lifted" in H.evers:
        return 0.5, "the airway lifted the tongue base 10 deg at some point, but is not open at the end"
    if "entered" in H.evers:
        return 0.25, "the held airway's tip went 35 mm into the open mouth, but never lifted the tongue base"
    return 0.0, "the held airway never went 35 mm into the open mouth"


REC = Record()
_PREV: dict = {}  # the airway's pose at the previous control step, for pose_rates()


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    m = measure(tick.states["head"], tick.states["airway"], tick.tools)
    A = np.array(tick.states["airway"].poses["airway"], dtype=float)
    if _PREV:
        m["aw_speed"], m["aw_spin"] = pose_rates(_PREV["A"], A, tick.t - _PREV["t"])
    _PREV.update(A=A, t=tick.t)
    REC.update(m)
    if REC.n % 5 == 0:  # diagnostics every 0.25 s
        A = tick.states["airway"].poses["airway"]
        z = np.asarray(tick.tools["right"].rot)[:, 2]
        slip = math.degrees(math.acos(max(-1.0, min(1.0, float(-A[:3, 2] @ z)))))
        REC.trace.append((round(tick.t, 2), round(m["q_t"], 2), round(m["q_j"], 1), round(m["gap"], 1),
                          [round(v, 1) for v in m["F"]], round(m["alpha"], 1), round(m["tilt"], 1),
                          [round(v, 1) for v in m["tip"]], int(m["held"]),
                          [round(1000 * m["clear"][s], 0) for s in ("left", "right")], round(slip, 1)))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
# linkdemo.py runs demo_script(ctx). It reads only what a camera could measure (link and object poses, the jaw angle
# computed from the skull's and jaw's poses) and the robot's own grippers; never joint values or the record.

VARIANTS = ("", "nothing", "as_presented", "straight_down", "one_arm", "turn_early", "drop_over_mouth")
JAW_OPEN = 28.0  # deg
JAW_READY = 26.0  # the right arm waits for at least this much opening
CHIN_LOCAL = (0.022, 0.0, -0.007)  # chin-plate pinch point, jaw link frame (m)
CHIN_WIDTH = 0.050
CHIN_SQUEEZE = 0.002
CHIN_DEPTH = 0.009  # pad contact centre past the TCP: pad tips 4 mm past the plate's mid-plane
LEFT_LEAN = 20.0  # deg: the left gripper's body leans toward the chin, away from the mouth
TCP_Z = 0.0145  # TCP above the flange bottom along the airway's +z: the pads cover the flange's top 5.5 mm
GRIP_OPEN, GRIP_CLOSED = 0.066, 0.044  # the flange is 56 mm end to end: 12 mm squeeze (about 30 N a pad)
SIGN = 1  # pads' closing axis = SIGN x the airway's +y (the gripper is symmetric; this picks the wrist turn)
D1 = 1  # the in-hand inversion turns the airway to alpha = 180 * D1, the turn at depth comes back the other way
CARRY_W = 96.0  # mm, flange height while carrying over the face
# Threading the upside-down airway through the open mouth: (u, w, tilt) of the flange-bottom centre (mm, deg), a
# window found offline (>= 1 mm from the teeth, lips and tongue with the jaw at 26 deg): tilted 12 deg (tip down,
# flange toward the head top) near the lips, straightening as it goes deeper.
THREAD = ((12.0, 96.0, 12.0), (12.0, 66.0, 12.0), (11.0, 53.0, 12.0), (8.0, 47.0, 12.0), (1.0, 41.0, 12.0),
          (-4.0, 35.0, 12.0), (-6.0, 29.0, 11.0), (-6.0, 23.0, 10.0), (-5.0, 17.0, 9.0), (-4.0, 11.0, 7.0),
          (-3.0, 7.0, 4.0), (-2.0, 3.5, 0.0))
THREAD_SPEED = 16.0  # mm/s along the path
TURN_W = 2.5  # flange bottom height above the lips during the turn at depth (tip 1.5 mm above the throat wall)
AXIS_U, AXIS_MID = -8.0, 3.0  # turn axis u (bite block 1 mm off the upper teeth all turn), mid-turn caudal slide
LATE_RISE = 2.0  # mm the airway rises over the last 60 deg of the turn, once its tip is under the tongue's ramp
SEAT_W = 0.5
PRESS_W = -2.0  # the seat command: 2 mm below the lips, so the arm presses the flange onto them
SEAT_U = -8.8  # seated with the bite block against the upper teeth, so the jaw closes to about 10 deg
TURN_RATE = 20.0  # deg/s
AS_PRESENTED_SWING = 30.0  # deg, the as_presented cheat's swing back about the curve's centre
LOG: dict = {}


def _opts():
    v = VARIANT
    return {"turn_w": 20.0 if v == "turn_early" else TURN_W}


def airway_pose(F_mm, alpha, beta=0.0, frame=HEAD_FRAME) -> np.ndarray:
    """World pose of the airway with its flange-bottom centre at F (head mm), its curve at azimuth ``alpha`` (deg,
    0 = toward the chin) and its bite-block axis tilted ``beta`` deg toward the head top (tip down)."""
    R = lk.axis_rotation((0, 1, 0), math.radians(-beta)) @ lk.axis_rotation((0, 0, 1), math.radians(alpha))
    M = np.eye(4)
    M[:3, :3] = frame[:3, :3] @ R
    M[:3, 3] = head_point(F_mm, frame)
    return M


def grasp_nominal(sign=SIGN) -> np.ndarray:
    """TCP pose in the airway frame: approach down the bite block, pads on the flange's two ends (closing along
    its 56 mm length), so a sideways push on the tip is held by friction on a 28 mm lever."""
    s = float(sign)
    G = np.eye(4)
    G[:3, :3] = np.array([[0.0, s, 0.0], [s, 0.0, 0.0], [0.0, 0.0, -1.0]])
    G[:3, 3] = (0.0, 0.0, TCP_Z)
    return G


def _smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def seg(a, b, *, rise_first=False, linear=False, axis_slide=False, late_rise=0.0):
    """Path between two airway parameter sets (F_mm, alpha, beta) as s -> (F, alpha, beta)."""
    Fa, Fb = np.asarray(a[0], float), np.asarray(b[0], float)

    def at(s):
        e = s if linear else _smooth(s)
        F = Fa + e * (Fb - Fa)
        if rise_first:  # climb first, then cross over
            F[2] = Fa[2] + (Fb[2] - Fa[2]) * _smooth(min(1.0, 2.5 * s))
        if axis_slide:  # turn at depth: the axis slides caudally mid-turn to keep the bite block off the teeth
            F[0] = Fa[0] + e * (Fb[0] - Fa[0]) + AXIS_MID * math.sin(math.pi * e)
        if late_rise:  # lift the tube off the throat wall once it is under the tongue (last third of the turn)
            F[2] += late_rise * min(1.0, max(0.0, (e - 2.0 / 3.0) * 3.0))
        return F, a[1] + e * (b[1] - a[1]), a[2] + e * (b[2] - a[2])

    return at


def polyline(points, alpha):
    """Constant-speed path through (u, w, tilt) points at azimuth ``alpha``; returns (path, length mm)."""
    P = np.asarray(points, dtype=float)
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1))]

    def at(s):
        x = _smooth(s) * d[-1]
        k = min(int(np.searchsorted(d, x, side="right")) - 1, len(P) - 2)
        f = 0.0 if d[k + 1] == d[k] else (x - d[k]) / (d[k + 1] - d[k])
        u, w, b = P[k] + f * (P[k + 1] - P[k])
        return (u, 0.0, w), alpha, b

    return at, float(d[-1])


def thread_points(turn_w):
    """THREAD cut at the turn height (straightened there)."""
    pts = []
    for a, b in zip(THREAD[:-1], THREAD[1:]):
        pts.append(a)
        if b[1] < turn_w <= a[1]:
            f = (a[1] - turn_w) / (a[1] - b[1])
            pts.append((a[0] + f * (b[0] - a[0]), turn_w, a[2] + f * (b[2] - a[2])))
            break
    else:
        pts.append(THREAD[-1])
    u, w, b = pts[-1]
    if b > 0.0:
        pts.append((u, w, 0.0))
    return pts


def right_plan(start, variant=""):
    """The right arm's airway path after the lift, as (name, seconds, path, servo_after) with paths over head
    parameters (F_mm, alpha, beta). ``start`` is the lifted airway's parameters."""
    tw = 20.0 if variant == "turn_early" else TURN_W
    inv = 180.0 * D1
    pts = thread_points(tw)
    u0, w0, b0 = pts[0]
    pre = ((u0, 0.0, w0), inv, b0)
    thread, length = polyline(pts, inv)
    ul, wl, _ = pts[-1]
    pdn = ((ul, 0.0, wl), inv, 0.0)
    pax = ((AXIS_U, 0.0, tw), inv, 0.0)
    pturn = ((AXIS_U, 0.0, tw + LATE_RISE), 0.0, 0.0)
    pseat = ((SEAT_U, 0.0, PRESS_W), 0.0, 0.0)
    return [
        ("carry", 3.0, seg(start, pre, rise_first=True), "servo"),
        ("wait_jaw", 0.0, None, None),
        ("thread", max(1.0, length / THREAD_SPEED), thread, "servo"),
        ("slide", 1.0, seg(pdn, pax), "servo"),
        ("turn", 180.0 / TURN_RATE, seg(pax, ((AXIS_U, 0.0, tw), 0.0, 0.0), linear=True, axis_slide=True,
                                         late_rise=LATE_RISE), None),
        ("seat", max(1.5, (tw + LATE_RISE - PRESS_W) / 6.0), seg(pturn, pseat), None),
    ]


def head_params(A, frame=HEAD_FRAME):
    """(F_mm, alpha, beta) of a world airway pose (beta from the bite block's tilt toward -u)."""
    F = to_head(A[:3, 3], frame)
    R = frame[:3, :3].T @ A[:3, :3]
    alpha = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    beta = math.degrees(math.atan2(-R[0, 2], R[2, 2]))
    return tuple(F), alpha, beta


class Hand:
    """The right arm carrying the airway. It commands the TCP so that the airway follows a planned pose, through a
    grasp transform measured once in the air and a slowly updated correction measured from the airway's seen pose."""

    def __init__(self, ctx, frame):
        self.ctx, self.arm, self.frame = ctx, ctx.right, frame
        self.G = grasp_nominal()
        self.dp = np.zeros(3)
        self.rv = np.zeros(3)  # rotation correction as a rotation vector (world)
        self.dp_goal = np.zeros(3)
        self.rv_goal = np.zeros(3)
        self.log: list = []

    def _rot(self, rv):
        a = float(np.linalg.norm(rv))
        return np.eye(3) if a < 1e-9 else lk.axis_rotation(rv / a, a)

    def tcp(self, A):
        T = A @ self.G
        dR = self._rot(self.rv)
        return A[:3, 3] + dR @ (T[:3, 3] - A[:3, 3]) + self.dp, dR @ T[:3, :3]

    def pose(self, params):
        return airway_pose(params[0], params[1], params[2], self.frame)

    def follow(self, path, seconds, width=None):
        w = self.arm.width if width is None else width

        def at(s):
            self.dp += 0.15 * (self.dp_goal - self.dp)
            self.rv += 0.15 * (self.rv_goal - self.rv)
            p, r = self.tcp(self.pose(path(s)))
            return p, r, w

        yield from self.arm.follow(at, seconds)

    def seen(self):
        return self.ctx.observe().pose("airway", "airway")

    def correct(self, A_des, gain=1.0, max_mm=8.0, max_deg=12.0, tilt_only=False):
        """Measure the seen airway against where it should be and move the correction goal by ``gain`` x error.
        ``tilt_only``: correct the bite block's direction but not the turn about it (the wrist lags while turning)."""
        A = self.seen()
        e = A_des[:3, 3] - A[:3, 3]
        if tilt_only:
            zo, zd = A[:3, 2], A_des[:3, 2]
            ax = np.cross(zo, zd)
            ang = math.atan2(float(np.linalg.norm(ax)), float(zo @ zd))
            rv = np.zeros(3) if ang < 1e-9 else ax / np.linalg.norm(ax) * ang
        else:
            Re = A_des[:3, :3] @ A[:3, :3].T
            q = lk.matrix_to_quat(Re)
            ang = 2 * math.atan2(float(np.linalg.norm(q[:3])), float(q[3]))
            rv = np.zeros(3) if ang < 1e-9 else np.asarray(q[:3]) / np.linalg.norm(q[:3]) * ang
        self.dp_goal = np.clip(self.dp_goal + gain * e, -max_mm * MM, max_mm * MM)
        self.rv_goal = self.rv_goal + gain * rv
        n = float(np.linalg.norm(self.rv_goal))
        if n > math.radians(max_deg):
            self.rv_goal *= math.radians(max_deg) / n
        return 1000 * float(np.linalg.norm(e)), math.degrees(ang)

    def servo(self, params, tag, tries=3, tol_mm=0.5, tol_deg=1.0):
        """Hold still at ``params`` and null the seen error (the arm sags a few mm; the grasp may have slipped)."""
        A_des = self.pose(params)
        for k in range(tries):
            yield from self.arm.wait(0.3)
            e, a = self.correct(A_des)
            self.log.append((tag, k, round(e, 2), round(a, 2)))
            if e <= tol_mm and a <= tol_deg:
                break
            self.dp, self.rv = self.dp_goal.copy(), self.rv_goal.copy()
            p, r = self.tcp(A_des)
            yield from self.arm.move(p, r, duration=0.5)


def _jaw_deg(ctx) -> float:
    return ctx.observe().angle_deg("head", "tmj")


def left_open_jaw(ctx, st):
    """Pinch the chin plate (pads closing along the jaw hinge, gripper body leaning toward the chin) and pivot the
    jaw open 28 deg, closed loop on the angle seen from the skull and jaw poses; hold it until told."""
    from linkdemo import grip_frame

    arm = ctx.left
    seen = ctx.observe()
    st["jaw0"] = a0 = seen.angle_deg("head", "tmj")
    hinge, axis = seen.axis("head", "tmj")
    grasp = seen.point("head", "jaw", CHIN_LOCAL)
    lean = math.radians(LEFT_LEAN)
    app = HEAD_FRAME[:3, :3] @ np.array([-math.sin(lean), 0.0, -math.cos(lean)])
    r = grip_frame(app, axis)
    tcp = grasp - r @ np.array([0.0, 0.0, CHIN_DEPTH])
    yield from arm.move(tcp - 0.06 * app, r, width=CHIN_WIDTH + 0.016, duration=2.0)  # fingers 8 mm clear a side
    yield from arm.move(tcp, r, duration=1.5)
    yield from arm.grip(CHIN_WIDTH - CHIN_SQUEEZE, 0.8)
    yield from arm.wait(0.4)
    arm.pivot = {"depth": CHIN_DEPTH, "follow": 0.5}
    yield from arm.arc(hinge, axis, JAW_OPEN, point=(0.0, 0.0, CHIN_DEPTH), follow=0.5, duration=3.0)
    log = st["pivot_log"] = [round(_jaw_deg(ctx) - a0, 2)]
    for _ in range(4):  # close the loop on the seen angle (the open-loop arc lags by the arm's compliance)
        yield from arm.wait(0.3)
        seen = ctx.observe()
        err = a0 + JAW_OPEN - seen.angle_deg("head", "tmj")
        if abs(err) <= 1.0 or abs(err) > 15.0:
            break
        hinge, axis = seen.axis("head", "tmj")
        yield from arm.arc(hinge, axis, err, point=(0.0, 0.0, CHIN_DEPTH), follow=0.5, duration=max(0.6, abs(err) / 8.0))
        log.append(round(_jaw_deg(ctx) - a0, 2))
    st["jaw_open_at"] = round(ctx.t, 2)
    fixes = []
    while not st.get("right_done") and not st.get("seated"):
        yield from arm.wait(0.5)
        seen = ctx.observe()
        err = st["jaw0"] + JAW_OPEN - seen.angle_deg("head", "tmj")
        if 1.0 < abs(err) < 15.0:
            hinge, axis = seen.axis("head", "tmj")
            yield from arm.arc(hinge, axis, err, point=(0.0, 0.0, arm.pivot["depth"]), follow=0.5,
                               duration=max(0.5, abs(err) / 8.0))
            fixes.append((round(ctx.t, 2), round(err, 2)))
    st["jaw_fixes"] = fixes
    st["jaw_before_shut"] = round(_jaw_deg(ctx), 2)


def left_ease_shut(ctx, st, rate=8.0):
    """Turn the pinch back toward closed at ``rate`` deg/s until the jaw stops on the bite block (seen angle no
    longer following), then let go and back away."""
    arm = ctx.left
    seen = ctx.observe()
    hinge, axis = seen.axis("head", "tmj")
    a0 = seen.angle_deg("head", "tmj")
    off = np.array([0.0, 0.0, arm.pivot["depth"]])
    r0 = arm.rot.copy()
    c0 = arm.pos + r0 @ off
    hist = [a0]
    cmd = 0.0
    from linkdemo import rot_about

    while cmd > -(a0 + 2.0):
        cmd -= rate / 20.0
        c = hinge + rot_about(axis, cmd) @ (c0 - hinge)
        r = rot_about(axis, 0.5 * cmd) @ r0
        arm._solve(c - r @ off, r, arm.width)
        yield
        hist.append(_jaw_deg(ctx))
        if len(hist) > 8 and hist[-1] < 16.0 and hist[-4] - hist[-1] < 0.3:  # stopped on the bite block
            break
        if hist[-1] <= 2.0:
            break
    st["jaw_stop"] = round(hist[-1], 2)
    st["jaw_shut_trace"] = [round(h, 1) for h in hist[::4]]
    yield from arm.grip(0.075, 0.6)
    st["jaw_after_release"] = round(_jaw_deg(ctx), 2)
    yield from arm.retreat(0.06, 1.0)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.05]), duration=0.8)


def right_pick(ctx, hand, st):
    """Top-down on the flange in the tray, pads closing on its two ends; lift."""
    arm = ctx.right
    A0 = hand.seen()
    up = np.eye(4)
    up[2, 3] = 0.06
    p, r = hand.tcp(up @ A0)
    yield from arm.move(p, r, width=GRIP_OPEN, duration=2.0)
    p, r = hand.tcp(A0)
    yield from arm.move(p, r, duration=1.2)
    yield from arm.grip(GRIP_CLOSED, 0.8)
    yield from arm.wait(0.4)
    A = hand.seen()  # the grasp as commanded: TCP command expressed in the airway's seen frame
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = arm.rot, arm.pos
    hand.G = np.linalg.inv(A) @ T
    st["grasp_offset_mm"] = [round(1000 * v, 2) for v in (hand.G[:3, 3] - grasp_nominal()[:3, 3])]
    lift = A.copy()
    lift[2, 3] += 0.10
    p, r = hand.tcp(lift)
    yield from arm.move(p, r, duration=1.0)
    F, alpha, beta = head_params(lift, hand.frame)
    start = (F, round(alpha), 0.0)
    p, r = hand.tcp(hand.pose(start))
    yield from arm.move(p, r, duration=0.5)
    return start


def right_run_plan(ctx, hand, st, plan, *, wait_jaw=True):
    for name, secs, path, after in plan:
        if name == "wait_jaw":
            t0 = ctx.t
            while wait_jaw and _jaw_deg(ctx) < JAW_READY and ctx.t - t0 < 20.0:
                yield from ctx.right.wait(0.1)
            st["entry_at"] = round(ctx.t, 2)
            continue
        if name == "turn":  # in 10 deg pieces, nudging the tilt correction from the seen airway after each
            n = 18
            for k in range(n):
                sub = (lambda k: (lambda s: path((k + s) / n)))(k)
                yield from hand.follow(sub, secs / n)
                e, a = hand.correct(hand.pose(path((k + 1) / n)), gain=0.6, max_mm=3.0, tilt_only=True)
                if k % 3 == 2:
                    hand.log.append(("turn", k, round(e, 2), round(a, 2)))
        else:
            yield from hand.follow(path, secs)
        if after == "servo":
            yield from hand.servo(path(1.0), name)
        st.setdefault("t_" + name, round(ctx.t, 2))


def right_release(ctx, st):
    arm = ctx.right
    yield from arm.grip(GRIP_OPEN, 0.6)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.06]), duration=1.2)


def demo_script(ctx):
    variant = VARIANT
    LOG.clear()
    st = LOG
    st["variant"] = variant
    yield from ctx.wait(1.0)
    if variant == "nothing":
        yield from ctx.wait(8.0)
        return
    frame = ctx.observe().pose("head", "skull")  # the head frame, as seen
    hand = Hand(ctx, frame)
    st["hand_log"] = hand.log

    def left():
        if variant == "one_arm":
            return
        yield from left_open_jaw(ctx, st)
        while not st.get("right_done") and not st.get("seated"):
            yield from ctx.left.wait(0.1)
        if st.get("seated"):
            yield from left_ease_shut(ctx, st)
        st["left_done"] = True

    def right():
        start = yield from right_pick(ctx, hand, st)
        if variant in ("", "turn_early", "one_arm"):
            plan = right_plan(start, variant)
            yield from right_run_plan(ctx, hand, st, plan, wait_jaw=variant != "one_arm")
            st["seated"] = True
            yield from ctx.right.wait(0.5)
            while not st.get("left_done") and variant != "one_arm":
                yield from ctx.right.wait(0.1)
        elif variant == "drop_over_mouth":
            plan = right_plan(start, variant)[:2]  # carry upside down over the face, wait for the jaw
            yield from right_run_plan(ctx, hand, st, plan)
            over = ((AXIS_U, 0.0, 30.0 + BITE + 33.0), 180.0 * D1, 0.0)  # its lowest point 30 mm above the lips
            yield from hand.follow(seg(plan[0][2](1.0), over), 2.0)
        elif variant == "straight_down":  # as presented (curve toward the chin), straight down onto the open mouth
            above = ((AXIS_U, 0.0, CARRY_W), 0.0, 0.0)
            yield from hand.follow(seg(start, above, rise_first=True), 3.0)
            yield from right_run_plan(ctx, hand, st, [("wait_jaw", 0, None, None)])
            yield from hand.follow(seg(above, ((AXIS_U, 0.0, SEAT_W), 0.0, 0.0)), (CARRY_W - SEAT_W) / 16.0)
        elif variant == "as_presented":  # as presented, tip first, swung in along its own curve
            yield from as_presented(ctx, hand, st, start)
        st["right_done"] = True
        yield from right_release(ctx, st)

    yield from ctx.both(left(), right())
    st["corr_mm"] = [round(1000 * v, 2) for v in hand.dp]
    st["corr_deg"] = round(math.degrees(float(np.linalg.norm(hand.rv))), 2)
    yield from ctx.wait(3.5)


def as_presented(ctx, hand, st, start):
    """Curve toward the chin as in the tray, tip first: the natural 'follow the curve' insertion. The airway is held
    swung 30 deg back about its curve's centre (tip down, flange toward the chin), lowered 60 mm straight down into the
    open mouth in that pose, then swung home along its own curve. (A 60 deg swing is out of the right arm's reach;
    30 deg is the most it reaches with the tip over the mouth, IK error under 0.1 mm.)"""
    seat = airway_pose((AXIS_U, 0.0, SEAT_W), 0.0, 0.0, hand.frame)
    C = seat[:3, :3] @ np.array([0.028, 0.0, -BITE * MM]) + seat[:3, 3]  # centre of the curve
    vaxis = hand.frame[:3, :3] @ np.array([0.0, 1.0, 0.0])

    def swung(p_deg, lift=0.0):
        Rv = lk.axis_rotation(vaxis, math.radians(p_deg))
        M = np.eye(4)
        M[:3, :3] = Rv @ seat[:3, :3]
        M[:3, 3] = C + Rv @ (seat[:3, 3] - C) + np.array([0.0, 0.0, lift])
        return M

    def path(fn):
        return lambda s: head_params(fn(s), hand.frame)

    low = head_params(swung(AS_PRESENTED_SWING), hand.frame)  # the tip in the open mouth, its end over the tongue
    high = ((low[0][0], low[0][1], low[0][2] + 60.0), low[1], low[2])
    yield from hand.follow(seg(start, high, rise_first=True), 3.0)
    yield from right_run_plan(ctx, hand, st, [("wait_jaw", 0, None, None)])
    yield from hand.follow(seg(high, low), 60.0 / 20.0)
    yield from hand.follow(path(lambda s: swung(AS_PRESENTED_SWING * (1 - s))), 3.0)  # follow the curve home


def demo_report() -> dict:
    score_, why = ladder(REC)
    H = REC.H
    m = REC.last or {}
    last = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in m.items()
            if k not in ("clear", "tcp_speed", "F", "tip")}
    if m:
        last["F"] = [round(v, 2) for v in m["F"]]
        last["tip"] = [round(v, 2) for v in m["tip"]]
        last["clear_mm"] = {s: round(1000 * v, 1) for s, v in m["clear"].items()}
    return {
        "score": score_, "reason": why, "variant": VARIANT,
        "evers": {k: round(v, 2) for k, v in H.evers.items()},
        "peaks": {k: round(v, 2) for k, v in H.peaks.items()},
        "lows": {k: round(v, 2) for k, v in H.lows.items()},
        "runs": {k: round(v, 2) for k, v in H.runs.items()},
        "last": last,
        "tongue_rest_deg": round(TONGUE_REST_DEG, 3),
        "log": {k: v for k, v in LOG.items()},
        "trace": REC.trace,
    }
