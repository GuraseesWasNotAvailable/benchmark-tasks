"""On the station table, in front of the left arm, a 1 g bench analogue of moving a survey fixture during spacecraft
alignment. A laser tracker measures to a spherically mounted retroreflector (SMR), a 38 mm ball. SMRs normally sit in
magnetic nests; on a flat, non-magnetic seat such as the top of this dark grey target bar, one sits loose. The ball is
a solid aluminium stand-in, resting loose on the bar's top. A white station pad lies 150 mm to the bar's right, along
the bar's own length, and a blue drift nest, a fixed cradle of four blocks around a square opening, stands to the
bar's left, nearer the arm.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/smr-bar-relocate/smr_bar_relocate.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Move the dark grey bar onto the white pad and leave the silver ball resting on top of it." \\
           --scorer task_score --max-steps 2400
"""

# Physics and scoring in brief. They sit in comments, not in the docstring above, because yam_arena hands the docstring
# to the policy as the scene description, and the instruction names only the goal: keeping the ball safe is for the
# policy to work out. A ball rolling without slip on a level carrier moves 2/7 of the carrier's travel at any speed, so
# it rolls back 5/7 of every move: carrying the bar to the pad rolls the ball 107 mm along a top 100 mm long, and it
# drops off the end after 35 mm. A top-down grip can only close across the bar's 50 mm width, so the fingers stand
# beside its long sides and do not stop that roll. The scripted demo parks the ball in the nest, moves the bar, then
# reseats the ball; the instruction does not mention the nest. A record kept on every control step remembers whether
# the ball ever went low enough to touch the table or the pad, and only the simulator's reset clears it, so a ball
# dropped and put back still scores 0. Nothing scores while a gripper is within 30 mm of the bar or the ball. Every
# test reads the ball's centre and the bar's centre and quaternion.

import math
import pathlib
import sys

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import linkkit as lk  # noqa: E402  MIT, ours: only its gripper read-out and its simulator-reset hook are used

try:
    from yam_arena.task_file import Sphere
except ImportError:  # only a minimal stand-in of the package that has no Sphere, used for offline checks, lands here

    class Sphere:
        def __init__(self, name, pos, color=(0.7, 0.7, 0.7), mass=None, yaw=0.0, radius=0.03):
            self.name, self.pos, self.color, self.mass, self.yaw, self.radius = name, pos, color, mass, yaw, radius


SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
BAR = (0.050, 0.100, 0.030)  # 50 mm across (x), 100 mm long (y, the way it travels), 30 mm tall; the 100 x 50 faces are flat
BAR_MASS = 0.100
BAR_START = (0.220, 0.260)
BAR_YAW = -math.pi / 2  # the bar grip: fingers close along x, across the 50 mm width (the length is wider than the open hand)
BALL_R = 0.019  # 38 mm across, the size of a 1.5 in SMR
BALL_MASS = 0.078  # solid aluminium
BALL_START = (0.220, 0.285)  # on the bar's top face, 25 mm from its centre toward +y
BALL_START_Z = TOP + BAR[2] + BALL_R  # 49 mm up
NEST = (0.160, 0.430)  # centre of the drift nest's square opening
NEST_H = 0.022
NEST_OPEN = 0.030
NEST_T = 0.012
SEAT_Z = TOP + NEST_H + math.sqrt(BALL_R**2 - (NEST_OPEN / 2) ** 2)  # 33.7 mm: the ball on the four inner top edges
PAD = (0.220, 0.110)  # 150 mm from the bar's start along -y, the bar's own long axis
PAD_SIZE = (0.070, 0.170, 0.002)
PAD_TOP = TOP + PAD_SIZE[2]

BAR_COLOR = (0.30, 0.31, 0.33)
BALL_COLOR = (0.85, 0.85, 0.87)
NEST_COLOR = (0.15, 0.30, 0.65)
PAD_COLOR = (0.93, 0.93, 0.90)

_wall_off = NEST_OPEN / 2 + NEST_T / 2  # 21 mm, a block's centre from the nest centre
_wall_len = NEST_OPEN + 2 * NEST_T  # 54 mm
NEST_BLOCKS = [
    Box("nest_wall_left", pos=(NEST[0], NEST[1] + _wall_off, TOP + NEST_H / 2), size=(_wall_len, NEST_T, NEST_H), color=NEST_COLOR),
    Box("nest_wall_right", pos=(NEST[0], NEST[1] - _wall_off, TOP + NEST_H / 2), size=(_wall_len, NEST_T, NEST_H), color=NEST_COLOR),
    Box("nest_wall_near", pos=(NEST[0] - _wall_off, NEST[1], TOP + NEST_H / 2), size=(NEST_T, NEST_OPEN, NEST_H), color=NEST_COLOR),
    Box("nest_wall_far", pos=(NEST[0] + _wall_off, NEST[1], TOP + NEST_H / 2), size=(NEST_T, NEST_OPEN, NEST_H), color=NEST_COLOR),
]
PAD_BOX = Box("station_pad", pos=(PAD[0], PAD[1], TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=PAD_COLOR)
BAR_BOX = Box("target_bar", pos=(BAR_START[0], BAR_START[1], TOP + BAR[2] / 2), size=BAR, mass=BAR_MASS, color=BAR_COLOR)
BALL = Sphere("reflector_ball", pos=(BALL_START[0], BALL_START[1], BALL_START_Z), radius=BALL_R, mass=BALL_MASS, color=BALL_COLOR)
OBJECTS = [BAR_BOX, BALL, PAD_BOX, *NEST_BLOCKS]

# Release: a trial scores only while both grippers have let go. linkkit reads each gripper's pads after every
# control step, before this file is scored; offline (self_test) nothing is recorded and the gate is open.
LET_GO = 0.030  # m: both grippers' pads at least 30 mm from every loose part
LET_GO_PARTS = [o.name for o in OBJECTS if getattr(o, "mass", None) is not None]  # the bar and the ball
_CLEAR: dict[str, float] = {}  # loose part -> nearest pad distance at the last control step


@lk.on_step
def _measure_grippers(tick) -> None:
    tools = list(tick.tools.values())
    for n in LET_GO_PARTS:
        _CLEAR[n] = min(lk.clearance(t, tick.states[n]) for t in tools)


def _released() -> bool:
    """Both grippers at least 30 mm from every loose part at the last control step (always true offline)."""
    return all(c >= LET_GO for c in _CLEAR.values())


SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

TOUCH_Z = PAD_TOP + BALL_R + 0.0015  # 22.5 mm: 1.5 mm above a ball touching the pad (21 mm); the table 19, bar 49, nest 33.7
PLACED_XY = 0.010  # bar centre to the pad centre
PLACED_YAW = math.radians(10.0)  # long axis to y, either way round
NEAR_XY = 0.030
NEAR_YAW = math.radians(20.0)
TILT_TOL = math.radians(3.0)  # the bar's 30 mm axis to vertical, either face up
BOTTOM_TOL = 0.0015  # the bar's lowest corner to the pad top; the table is 2 mm lower and fails
INSET = 0.008  # the ball's centre this far in from every edge of the bar's top face
BALL_LIFT_TOL = 0.002  # the ball's centre to 19 mm above the top face, along the face's normal
NEST_XY_TOL = 0.006
NEST_Z_TOL = 0.003
EPS = 1e-6
LADDER = {(True, "bar"): 1.0, (False, "bar"): 0.6, (True, "nest"): 0.5, (False, "nest"): 0.3}  # (bar placed, ball)

# The ball may never touch the table or the pad, at any moment of the trial, so the rule needs history: a ball dropped
# and put back must still fail. success() and score() both update this record on every control step: it latches the
# first step the ball's centre is under 22.5 mm, or its pose is not finite (a solver blow-up). Only the simulator's
# reset clears it: linkkit calls _reset_records() from the embodiment's reset(), before the first step of each trial
# or epoch. Nothing a policy does to the scene reaches that call, so no pose, however exactly restored, clears the
# record.
_HISTORY = {"touched": False}


@lk.on_reset
def _reset_records(tick=None) -> None:
    """Runs on every simulator reset (a new trial or epoch); self_test calls it between cases."""
    _HISTORY["touched"] = False
    _CLEAR.clear()


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


_SIGNS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)


def _bottom(bar) -> float:
    """The z of the bar's lowest corner, from its centre and quaternion."""
    corners = np.asarray(bar.pos, float) + (_SIGNS * (np.asarray(BAR) / 2)) @ _rot(bar.quat_xyzw).T
    return float(corners[:, 2].min())


def _yaw_off(r) -> float:
    """Angle between the bar's 50 mm axis and x, either way round: the same as its long axis to y."""
    off = abs(math.atan2(float(r[1, 0]), float(r[0, 0])))
    return min(off, math.pi - off)


def _bar_at(bar, xy_tol, yaw_tol) -> bool:
    """At rest, flat on the pad (30 mm axis within 3 degrees of vertical, either face up, lowest corner within
    1.5 mm of the pad top), centre within ``xy_tol`` of the pad centre, long axis within ``yaw_tol`` of y."""
    if not bar.resting():
        return False
    r = _rot(bar.quat_xyzw)
    if abs(float(r[2, 2])) < math.cos(TILT_TOL):
        return False
    if math.hypot(float(bar.pos[0]) - PAD[0], float(bar.pos[1]) - PAD[1]) > xy_tol + EPS:
        return False
    if _yaw_off(r) > yaw_tol + EPS:
        return False
    return abs(_bottom(bar) - PAD_TOP) <= BOTTOM_TOL + EPS


def _on_bar(ball, bar) -> bool:
    """The ball at rest on the bar's upward face: in the bar's frame its centre lies 19 mm (within 2 mm) off that face
    along the face's normal and at least 8 mm in from each of the face's edges. The bar is at rest and flat."""
    if not (ball.resting() and bar.resting()):
        return False
    r = _rot(bar.quat_xyzw)
    if abs(float(r[2, 2])) < math.cos(TILT_TOL):
        return False
    up = 1.0 if float(r[2, 2]) > 0 else -1.0  # the bar's local z points up, or down when it lies upside down
    local = r.T @ (np.asarray(ball.pos, float) - np.asarray(bar.pos, float))
    if abs(up * float(local[2]) - (BAR[2] / 2 + BALL_R)) > BALL_LIFT_TOL + EPS:
        return False
    return abs(float(local[0])) <= BAR[0] / 2 - INSET + EPS and abs(float(local[1])) <= BAR[1] / 2 - INSET + EPS


def _in_nest(ball) -> bool:
    """The ball at rest on the nest's four edges: centre within 6 mm of the nest centre and within 3 mm of 33.7 mm."""
    off = math.hypot(float(ball.pos[0]) - NEST[0], float(ball.pos[1]) - NEST[1])
    return ball.resting() and off <= NEST_XY_TOL + EPS and abs(float(ball.pos[2]) - SEAT_Z) <= NEST_Z_TOL + EPS


def _track(objects) -> None:
    ball = np.asarray(objects[BALL.name].pos, dtype=float)
    if not np.all(np.isfinite(ball)) or float(ball[2]) < TOUCH_Z:
        _HISTORY["touched"] = True


def _result(objects):
    """(bar placed, where the ball is) for the ladder, or None when nothing scores."""
    bar, ball = objects[BAR_BOX.name], objects[BALL.name]
    placed = _bar_at(bar, PLACED_XY, PLACED_YAW)
    if not (placed or _bar_at(bar, NEAR_XY, NEAR_YAW)):
        return None
    if _on_bar(ball, bar):
        return placed, "bar"
    if _in_nest(ball):
        return placed, "nest"
    return None


def _success_held(objects) -> bool:
    """The ball never touched the table or the pad, the bar lies placed on the pad and the ball rests on its top,
    everything at rest."""
    _track(objects)
    return not _HISTORY["touched"] and _result(objects) == (True, "bar")


def success(objects) -> bool:
    """The ball never touched the table or the pad, the bar lies placed on the pad and the ball rests on its top,
    everything at rest, with both grippers at least 30 mm clear; holds for 1 s."""
    return _success_held(objects) and _released()


def distance(objects) -> float:
    """Metres still to go: the bar's centre to the pad centre, plus the ball's centre to the bar's centre, across."""
    bar, ball = objects[BAR_BOX.name], objects[BALL.name]
    bx, by = float(bar.pos[0]), float(bar.pos[1])
    return math.hypot(bx - PAD[0], by - PAD[1]) + math.hypot(float(ball.pos[0]) - bx, float(ball.pos[1]) - by)


def _score_held(objects) -> float:
    """0 if the ball has touched the table or the pad at any step since the simulator's reset. Otherwise 1.0 for the
    bar placed with the ball on it, 0.6 for the bar near with the ball on it, 0.5 for the bar placed with the ball in
    the nest, 0.3 for the bar near with the ball in the nest, and 0 for everything else."""
    _track(objects)
    if _HISTORY["touched"]:
        return 0.0
    res = _result(objects)
    return LADDER[res] if res else 0.0


def score(objects) -> float:
    """The ladder of _score_held, and 0 while either gripper is within 30 mm of the bar or the ball: let go to score."""
    value = _score_held(objects)  # always called first, so the record is updated on every step
    return value if _released() else 0.0


TRAVEL = TOP + 0.150
PAD_TIP = 0.013  # the pad tips sit this far below the TCP
BALL_PICK_Z = BALL_START_Z + 0.006  # TCP 6 mm above the ball's centre: pad tips 12 mm above the bar top, equator 7 mm up the pads
NEST_SET_Z = TOP + NEST_H + 0.007 + PAD_TIP  # pad tips 7 mm above the nest blocks: ball centre 36 mm up, 2.3 mm above its seat
NEST_ENTRY = 0.015  # the pause above the nest: the ball's bottom 10 mm above the blocks
BAR_PICK_Z = TOP + 0.018  # TCP 18 mm up the bar: pad tips 5 mm above the table, 25 mm of pad on the bar
BAR_SET_Z = PAD_TOP + 0.005 + 0.018  # the bar's bottom 5 mm above the pad, pad tips 10 mm above it
BAR_ENTRY = 0.010  # the pause above the pad: the bar's bottom 15 mm above it
NEST_PICK_Z = TOP + NEST_H + 0.005 + PAD_TIP  # pad tips 5 mm above the blocks: TCP 6.3 mm above the seated ball's centre
RESEAT_Z = PAD_TOP + BAR[2] + 0.003 + BALL_R + (NEST_PICK_Z - SEAT_Z)  # the ball's bottom 3 mm above the bar top
RESEAT_ENTRY = 0.007  # the pause above the bar: the ball's bottom 10 mm above its top
EASED_SQUEEZE = 0.001  # before the ball is let go, the grip eases to 1 mm narrower than it and the ball slides onto the top
EASED_WAIT = 0.6  # s held still in the eased grip, the ball resting on the top, before the fingers open
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part


def _pick_place(pick, place, *, width, entry, travel_z, yaw=0.0, place_yaw=None):
    """A top-down pick and place paced for the simulated arm, whose soft wrist trails a moving target by several
    millimetres. The arm settles before every close and release, stops ``entry`` above the place with the part
    clear of every fixture, and makes the last descent slowly. The fingers close to 8 mm narrower than the part's
    ``width`` across the pads rather than fully shut: each finger then pushes 4 mm x 5000 N/m = 20 N, under its
    50 N cap, so the grip centres the part, whereas a fully commanded close saturates both fingers and lets the part
    slide onto one of them. The gripper opens and closes over 1.5 s, inside the default 0.05 per step limiter.
    16.2 s."""
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    grip = -(width - SQUEEZE) / 2  # finger joint target, m
    return [
        Waypoint(1.5, (px, py, travel_z), FINGER_OPEN, "to the object", yaw),
        Waypoint(1.5, (px, py, pz), FINGER_OPEN, "descend", yaw),
        Waypoint(0.6, (px, py, pz), FINGER_OPEN, "settle", yaw),
        Waypoint(1.5, (px, py, pz), grip, "close", yaw),
        Waypoint(1.2, (px, py, travel_z), grip, "lift", yaw),
        Waypoint(2.0, (qx, qy, travel_z), grip, "carry", end),
        Waypoint(0.6, (qx, qy, travel_z), grip, "settle over the place", end),
        Waypoint(1.2, (qx, qy, qz + entry), grip, "lower to the entry", end),
        Waypoint(0.8, (qx, qy, qz + entry), grip, "settle at the entry", end),
        Waypoint(2.0, (qx, qy, qz), grip, "set in", end),
        Waypoint(0.8, (qx, qy, qz), grip, "settle", end),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, "release", end),
        Waypoint(1.0, (qx, qy, travel_z), FINGER_OPEN, "retreat", end),
    ]


def demo():
    """Left arm only. Offload: the ball off the bar at yaw 0 (fingers along y, pads on its equator, tips 12 mm above
    the bar top) and down into the nest, let go 2.3 mm above its seat. Move: the bar at yaw -90 degrees, fingers
    closing along x across its 50 mm width, carried 150 mm along its length to the pad and released 5 mm above it.
    Restore: the ball out of the nest at yaw 0 (tips 5 mm above the blocks) over the middle of the bar's top, held
    still with its bottom 3 mm above it. There the grip eases over 1.5 s from 8 mm to 1 mm narrower than the ball, so
    the ball slides down the pads onto the top as the squeeze fades. The arm then holds still for 0.6 s while the
    pads barely touch the resting ball: the soft wrist, which the hard squeeze twists about 1.5 degrees, unwinds, and
    the ball settles. Only then do the fingers open, along the bar's length. In the simulator, opening straight from
    the hard squeeze left the ball rolling across the top at 8 mm/s, and opening before the ball had settled on the
    top flicked it off the bar. The arm then rises to travel height, its pad tips over 60 mm above the ball, so the
    30 mm release gate opens during that retreat. 50.7 s of plan, then the 3 s hold: 1,074 steps at 20 Hz."""
    ball_w = 2 * BALL_R
    left = _pick_place((*BALL_START, BALL_PICK_Z), (*NEST, NEST_SET_Z), width=ball_w, entry=NEST_ENTRY, travel_z=TRAVEL)
    left += _pick_place((*BAR_START, BAR_PICK_Z), (*PAD, BAR_SET_Z), width=BAR[0], entry=BAR_ENTRY, travel_z=TRAVEL, yaw=BAR_YAW)
    reseat = _pick_place((*NEST, NEST_PICK_Z), (*PAD, RESEAT_Z), width=ball_w, entry=RESEAT_ENTRY, travel_z=TRAVEL)
    held = reseat[10]
    assert held.label == "settle" and reseat[11].label == "release"
    eased = -(ball_w - EASED_SQUEEZE) / 2
    reseat[11:11] = [
        Waypoint(1.5, held.pos, eased, "ease the grip", held.yaw),
        Waypoint(EASED_WAIT, held.pos, eased, "wait for the wrist", held.yaw),
    ]
    left += reseat
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states, on short histories of states (``RESET`` in a history stands for the
    simulator's reset, the only thing that clears the record) and the release gate; runs with the real ObjectState or
    any stand-in with the same fields. Leaves the record cleared."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    # The claims the task rests on. Rolling without slip, the ball moves 2/7 of a level carrier's travel, so it rolls
    # back 5/7 of it: 107 mm for the 150 mm move. A move that keeps the bar's heading rolls it that far in a straight
    # line across the top, and no straight line between two points 8 mm in from the edges is longer than the 91 mm
    # diagonal, so no such move can end with the ball scoring. The move runs along the bar's length, so from its start
    # 25 mm along the ball leaves the top's end after 35 mm of travel. A top-down grip fits only across the 50 mm
    # width (the 100 mm length is wider than the 94 mm open hand), so the fingers stand beside the ball's path.
    travel = math.hypot(PAD[0] - BAR_START[0], PAD[1] - BAR_START[1])
    roll = 5 / 7 * travel
    chord = math.hypot(BAR[0] - 2 * INSET, BAR[1] - 2 * INSET)
    assert abs(roll - 0.1071) < 1e-4 and abs(chord - 0.0906) < 1e-4 and roll > chord
    assert abs(PAD[0] - BAR_START[0]) < 1e-9 and abs(BALL_START[0] - BAR_START[0]) < 1e-9  # travel and ball along y
    assert abs((BAR[1] / 2 - (BALL_START[1] - BAR_START[1])) * 7 / 5 - 0.035) < 1e-9
    assert 0.015 <= BAR[0] <= 0.070 and BAR[1] > 0.094
    # The touch line sits 1.5 mm above a ball on the pad (21 mm) and under the nest seat (33.7 mm); the seated ball
    # clears the table.
    assert PAD_TOP + BALL_R < TOUCH_Z < SEAT_Z and abs(TOUCH_Z - TOP - 0.0225) < 1e-9
    assert abs(SEAT_Z - TOP - 0.03366) < 1e-4 and SEAT_Z - BALL_R - TOP > 0.014

    def quat(yaw=0.0, roll=0.0, pitch=0.0):  # yaw about z, then roll about the body's x, then pitch about its y
        r = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        r = r @ np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        r = r @ np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        w = math.sqrt(max(1e-12, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
        if w > 1e-3:
            return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])
        k = int(np.argmax(np.diag(r)))  # a half turn: take the largest diagonal term instead
        i, j = (k + 1) % 3, (k + 2) % 3
        s = math.sqrt(max(1e-12, 1 + r[k, k] - r[i, i] - r[j, j])) * 2
        q = np.zeros(4)
        q[k], q[i], q[j], q[3] = s / 4, (r[i, k] + r[k, i]) / s, (r[j, k] + r[k, j]) / s, (r[j, i] - r[i, j]) / s
        return q

    def bar(x, y, bottom, yaw=0.0, roll=0.0, pitch=0.0, vel=(0.0, 0.0, 0.0)):
        q = quat(yaw, roll, pitch)
        half = np.abs(_rot(q)) @ (np.asarray(BAR) / 2)
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def ball(x, y, z, vel=(0.0, 0.0, 0.0)):
        pos = np.array([x, y, z], float)
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - BALL_R, pos + BALL_R)

    def on(b, along=0.0, across=0.0, lift=0.0, vel=(0.0, 0.0, 0.0)):
        """The ball on bar state ``b``'s upward face, ``along`` its long axis (local y) and ``across`` it from the
        centre."""
        r = _rot(b.quat_xyzw)
        up = 1.0 if r[2, 2] > 0 else -1.0
        p = np.asarray(b.pos, float) + r @ np.array([across, along, up * (BAR[2] / 2 + BALL_R + lift)])
        return ball(*p, vel=vel)

    start_bar, start_ball = bar(*BAR_START, TOP), ball(*BALL_START, BALL_START_Z)

    def scene(b=None, s=None):
        return {BAR_BOX.name: start_bar if b is None else b, BALL.name: start_ball if s is None else s}

    placed = bar(*PAD, PAD_TOP)
    seated = ball(*NEST, SEAT_Z)
    off9 = bar(PAD[0] + 0.006, PAD[1] - 0.0067, PAD_TOP, yaw=math.radians(9))
    off11, off20, off29, off31 = bar(PAD[0], PAD[1] + 0.011, PAD_TOP), bar(PAD[0] - 0.020, PAD[1], PAD_TOP), bar(PAD[0] - 0.029, PAD[1], PAD_TOP), bar(PAD[0], PAD[1] + 0.031, PAD_TOP)
    yaw11, yaw19, yaw21 = (bar(*PAD, PAD_TOP, yaw=math.radians(a)) for a in (11, -19, 21))
    flipped, upside, side = bar(*PAD, PAD_TOP, yaw=math.pi), bar(*PAD, PAD_TOP, roll=math.pi), bar(*PAD, PAD_TOP, pitch=math.pi / 2)
    tilt2, tilt4 = bar(*PAD, PAD_TOP, pitch=math.radians(2)), bar(*PAD, PAD_TOP, pitch=math.radians(4))
    held3, on_table, moving = bar(*PAD, PAD_TOP + 0.003), bar(*PAD, TOP), bar(*PAD, PAD_TOP, vel=(0.0, 0.02, 0.0))
    centred_at_start = bar(*BAR_START, TOP)
    table_ball = ball(0.220, 0.340, TOP + BALL_R)  # rolled off the bar's +y end and lying on the table
    pad_ball = ball(PAD[0], 0.182, PAD_TOP + BALL_R)  # on the pad beyond the bar's +y end
    floor_ball = ball(0.300, 0.700, BALL_R)  # rolled off the table
    cases = {
        "start": (scene(), 0.0, False),
        "bar placed, ball at the middle of its top": (scene(placed, on(placed)), 1.0, True),
        "ball 25 mm along the top, as it started": (scene(placed, on(placed, along=0.025)), 1.0, True),
        "ball 17 mm off the long axis, 8 mm in from the side": (scene(placed, on(placed, across=-0.017)), 1.0, True),
        "ball 18 mm off the long axis, 7 mm in": (scene(placed, on(placed, across=0.018)), 0.0, False),
        "ball 42 mm along, 8 mm from the end": (scene(placed, on(placed, along=-0.042)), 1.0, True),
        "ball 43 mm along, 7 mm from the end": (scene(placed, on(placed, along=0.043)), 0.0, False),
        "bar 9 mm off and yawed 9 degrees": (scene(off9, on(off9)), 1.0, True),
        "bar 11 mm off": (scene(off11, on(off11)), 0.6, False),
        "bar 20 mm off": (scene(off20, on(off20)), 0.6, False),
        "bar 29 mm off, overhanging the pad's side": (scene(off29, on(off29)), 0.6, False),
        "bar 31 mm off": (scene(off31, on(off31)), 0.0, False),
        "bar yawed 11 degrees": (scene(yaw11, on(yaw11)), 0.6, False),
        "bar yawed 19 degrees the other way": (scene(yaw19, on(yaw19)), 0.6, False),
        "bar yawed 21 degrees": (scene(yaw21, on(yaw21)), 0.0, False),
        "bar turned end for end": (scene(flipped, on(flipped, along=0.025)), 1.0, True),
        "bar upside down": (scene(upside, on(upside, across=0.010)), 1.0, True),
        "bar on its side, ball on its 100 x 30 face": (scene(side, ball(PAD[0], PAD[1], PAD_TOP + BAR[0] + BALL_R)), 0.0, False),
        "bar tilted 2 degrees": (scene(tilt2, on(tilt2)), 1.0, True),
        "bar tilted 4 degrees": (scene(tilt4, on(tilt4)), 0.0, False),
        "bar held 3 mm above the pad": (scene(held3, on(held3)), 0.0, False),
        "bar flat on the table at the pad centre, 2 mm below the pad top": (scene(on_table, on(on_table)), 0.0, False),
        "bar still moving": (scene(moving, on(moving)), 0.0, False),
        "ball still rolling at 8 mm/s, under the rest limit": (scene(placed, on(placed, vel=(0.0, 0.008, 0.0))), 1.0, True),
        "ball rolling at 20 mm/s": (scene(placed, on(placed, vel=(0.0, 0.020, 0.0))), 0.0, False),
        "ball held still 3 mm above the bar": (scene(placed, on(placed, lift=0.003)), 0.0, False),
        "ball centre 1.5 mm high on the bar still counts": (scene(placed, on(placed, lift=0.0015)), 1.0, True),
        "ball in the nest, bar placed": (scene(placed, seated), 0.5, False),
        "ball in the nest, bar 20 mm off": (scene(off20, seated), 0.3, False),
        "ball in the nest, bar yawed 11 degrees": (scene(yaw11, seated), 0.3, False),
        "ball in the nest, bar 31 mm off": (scene(off31, seated), 0.0, False),
        "ball in the nest, bar still at its start": (scene(start_bar, seated), 0.0, False),
        "ball in the nest, bar on its side on the pad": (scene(side, seated), 0.0, False),
        "ball 5 mm off the nest centre": (scene(placed, ball(NEST[0] + 0.005, NEST[1], SEAT_Z)), 0.5, False),
        "ball 7 mm off the nest centre": (scene(placed, ball(NEST[0], NEST[1] + 0.007, SEAT_Z)), 0.0, False),
        "ball held 4 mm above its seat": (scene(placed, ball(*NEST, SEAT_Z + 0.004)), 0.0, False),
        "ball on a nest block's top": (scene(placed, ball(NEST[0], NEST[1] + _wall_off, TOP + NEST_H + BALL_R)), 0.0, False),
        "ball on the table, bar placed": (scene(placed, table_ball), 0.0, False),
        "ball on the pad beyond the bar's end": (scene(placed, pad_ball), 0.0, False),
        "ball on the floor": (scene(placed, floor_ball), 0.0, False),
        "ball moved to the middle of the bar, bar not moved": (scene(centred_at_start, on(centred_at_start)), 0.0, False),
    }
    try:
        for name, (st, want, want_success) in cases.items():
            _reset_records()
            got_success = success(st)
            got = score(st)
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)

        carried = ball(0.200, 0.350, TOP + 0.130, vel=(0.1, 0.0, 0.0))
        over_pad = bar(*PAD, TOP + 0.040, vel=(0.0, -0.05, 0.0))
        mid = bar(BAR_START[0], BAR_START[1] - 0.035, TOP + 0.060, vel=(0.0, -0.08, 0.0))
        start = scene()
        clean = [scene(start_bar, carried), scene(start_bar, seated), scene(over_pad, seated), scene(placed, seated), scene(placed, on(placed))]
        drop = scene(start_bar, table_ball)
        mark = (*BALL_START, BALL_START_Z)
        lost = ball(math.nan, math.nan, math.nan)
        RESET = object()  # the simulator's reset between two trials: linkkit calls _reset_records() from it
        sequences = {
            "offload, move, reseat": ([start, *clean], 1.0, True),
            "offload and move, ball left in the nest": ([start, scene(start_bar, seated), scene(placed, seated)], 0.5, False),
            "carried with the ball on: it rolls off the end onto the table, is put back": ([start, scene(mid, on(mid, along=0.050, vel=(0.0, -0.02, 0.0))), scene(mid, table_ball), scene(placed, table_ball), scene(placed, on(placed))], 0.0, False),
            "ball dropped on the pad, then reseated": ([start, scene(placed, pad_ball), scene(placed, on(placed))], 0.0, False),
            "ball dropped on the table, then parked in the nest": ([start, drop, scene(placed, seated)], 0.0, False),
            "ball carried low, centre 24 mm up, then reseated": ([start, scene(start_bar, ball(0.220, 0.340, TOP + 0.024, vel=(0.0, 0.05, 0.0))), scene(placed, on(placed))], 1.0, True),
            "ball carried with its centre 22 mm up, then reseated": ([start, scene(start_bar, ball(0.220, 0.340, TOP + 0.022, vel=(0.0, 0.05, 0.0))), scene(placed, on(placed))], 0.0, False),
            "a dropped ball, then both bodies back on their marks in one step at rest, as a reset leaves them, then a clean run": ([start, drop, start, *clean], 0.0, False),
            "the bar on the pad and the ball fallen under its mark, then both back on their marks in one step, then a clean run": ([start, scene(placed, ball(*BALL_START, TOP + BALL_R)), start, *clean], 0.0, False),
            "a dropped ball, the simulator's reset, then a clean run": ([start, drop, RESET, start, *clean], 1.0, True),
            "the bar on the pad and the ball fallen under its mark, the simulator's reset, then a clean run": ([start, scene(placed, ball(*BALL_START, TOP + BALL_R)), RESET, start, *clean], 1.0, True),
            "a solver blow-up loses the ball's pose, then a clean run": ([start, scene(start_bar, lost), start, *clean], 0.0, False),
            "ball dropped, then lowered back onto its mark by hand, then a clean run": ([start, drop, scene(start_bar, ball(mark[0], mark[1], mark[2] + 0.040, vel=(0.0, 0.0, -0.05))), scene(start_bar, ball(mark[0], mark[1], mark[2] + 0.004, vel=(0.0, 0.0, -0.02))), start, *clean], 0.0, False),
            "ball dropped, then let fall 30 mm onto its mark, then a clean run": ([start, drop, scene(start_bar, ball(mark[0], mark[1], mark[2] + 0.030)), start, *clean], 0.0, False),
            "ball dropped, both set back 5 mm from their marks, then a clean run": ([start, drop, scene(bar(BAR_START[0], BAR_START[1] + 0.005, TOP), ball(BALL_START[0], BALL_START[1] + 0.005, BALL_START_Z)), *clean], 0.0, False),
        }
        for name, (steps, want, want_success) in sequences.items():
            _reset_records()
            for st in steps:
                if st is RESET:
                    _reset_records()
                    continue
                got_success = success(st)
                got = score(st)
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)

        # The release gate: a full-score state with a gripper 10 mm from the ball (held on the bar) scores 0 and does
        # not succeed; with both grippers exactly 30 mm from the bar and the ball it scores 1.0 again.
        done = scene(placed, on(placed))
        _reset_records()
        _CLEAR.update({n: 0.050 for n in LET_GO_PARTS})
        _CLEAR[BALL.name] = 0.010
        assert score(done) == 0.0 and not success(done), "a gripper 10 mm from the ball"
        _CLEAR.update({n: LET_GO for n in LET_GO_PARTS})
        assert score(done) == 1.0 and success(done), "both grippers 30 mm clear"
    finally:
        _reset_records()
    return len(cases) + len(sequences) + 2


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states and histories and the release gate, then the left
    # arm's plan replayed through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under
    # 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
