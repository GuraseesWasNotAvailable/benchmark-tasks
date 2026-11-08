"""On the station table, in front of the right arm, a 1 g bench analogue of replacing a damaged crib under a raised
load. A grey aluminium beam lies along y. Its right end rests on a fixed steel pier between two saddle lips, which
stop it sliding along x or turning. Its left end rests on a loose red oak crib block, which is cracked. A green oak
crib, 1 mm lower than the red one, lies on the table nearby, and a grey scrap tray sits to the right. The beam's
centre of mass lies midway between the pier and the red crib. Pull the red crib first and the beam pivots on the
pier's inner edge and its left end drops 40 mm to the table, a tilt of about 11.5 degrees. Slide the green crib in
beside the red one first and the beam settles only 1 mm onto it, about 0.5 degrees, when the red crib comes out.
The order is never stated: physics punishes the wrong one. A disturbance record, kept on every control step and
cleared only when a new trial starts, scores 0 for the trial once the beam has tilted more than 2.5 degrees or its
centre has moved more than 6 mm. Every test uses each body's centre and quaternion.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/crib-swap-make-before-break/crib_swap_make_before_break.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "The red crib block under the beam is cracked. Swap it for the green block and put the red one in the grey tray. The beam must not drop or move." \\
           --scorer task_score --max-steps 2400
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The right arm's base is at (0, -0.305, 0.75).
BEAM = (0.040, 0.260, 0.020)  # x, y, z: a 40 x 20 mm box section lying along y
BEAM_MASS = 0.168
BEAM_XY = (0.290, -0.210)  # spans y -0.340 to -0.080 and x 0.270 to 0.310
BEAM_BOTTOM = TOP + 0.040  # resting on the pier and the red crib
BEAM_REST = np.array([BEAM_XY[0], BEAM_XY[1], BEAM_BOTTOM + BEAM[2] / 2])
# The beam spawns 0.6 mm above its supports and settles onto them in the 0.5 s settle before the first control step.
# The offline clearance model holds every free object it does not carry at its spawn pose, so the gap keeps the red
# crib, carried 0.3 mm up through its slide (see TAKE_UP), from reading as inside the beam.
BEAM_SPAWN_GAP = 0.0006
PIER = (0.080, 0.060, 0.040)
PIER_XY = (0.290, -0.310)  # spans x 0.250 to 0.330, y -0.340 to -0.280
PIER_INNER_Y = PIER_XY[1] + PIER[1] / 2  # -0.280: the edge the beam pivots on if its left end loses support
LIP = (0.010, 0.060, 0.020)
LIP_GAP = 0.0005  # per side between a lip and the beam
LIP_X = (BEAM_XY[0] - BEAM[0] / 2 - LIP_GAP - LIP[0] / 2, BEAM_XY[0] + BEAM[0] / 2 + LIP_GAP + LIP[0] / 2)  # 0.2645, 0.3155
CRIB_RED = (0.100, 0.030, 0.040)  # long axis along x, across the beam
CRIB_GREEN = (0.100, 0.030, 0.039)  # 1 mm lower than the red crib, so it slides under the beam with 1 mm to spare
CRIB_MASS = (0.084, 0.082)  # red, green
RED_START = (0.270, -0.110)  # spans x 0.220 to 0.320 under the beam's left end, 50 mm proud of it toward the arm
GREEN_START = (0.150, -0.185)  # on the table, spans x 0.100 to 0.200
TRAY_XY = (0.150, -0.400)
TRAY_IN = (0.130, 0.070)  # inside, x by y
TRAY_FLOOR_T = 0.003
TRAY_WALL_T = 0.004
TRAY_WALL_H = 0.015  # standing on the table round the floor
TRAY_FLOOR_TOP = TOP + TRAY_FLOOR_T

GREY = (0.55, 0.57, 0.60)
STEEL = (0.30, 0.31, 0.33)
RED = (0.80, 0.15, 0.12)
GREEN = (0.20, 0.60, 0.25)
TRAY_GREY = (0.45, 0.45, 0.47)

BEAM_BOX = Box("beam", pos=(BEAM_XY[0], BEAM_XY[1], float(BEAM_REST[2]) + BEAM_SPAWN_GAP), size=BEAM, mass=BEAM_MASS, color=GREY)
PIER_BOX = Box("pier", pos=(PIER_XY[0], PIER_XY[1], TOP + PIER[2] / 2), size=PIER, color=STEEL)
LIPS = [Box(f"lip_{side}", pos=(x, PIER_XY[1], TOP + PIER[2] + LIP[2] / 2), size=LIP, color=STEEL) for side, x in (("near", LIP_X[0]), ("far", LIP_X[1]))]
RED_BOX = Box("crib_red", pos=(RED_START[0], RED_START[1], TOP + CRIB_RED[2] / 2), size=CRIB_RED, mass=CRIB_MASS[0], color=RED)
GREEN_BOX = Box("crib_green", pos=(GREEN_START[0], GREEN_START[1], TOP + CRIB_GREEN[2] / 2), size=CRIB_GREEN, mass=CRIB_MASS[1], color=GREEN)
_wx = TRAY_IN[0] / 2 + TRAY_WALL_T / 2  # 67 mm
_wy = TRAY_IN[1] / 2 + TRAY_WALL_T / 2  # 37 mm
TRAY = [
    Box("tray_floor", pos=(TRAY_XY[0], TRAY_XY[1], TOP + TRAY_FLOOR_T / 2), size=(TRAY_IN[0], TRAY_IN[1], TRAY_FLOOR_T), color=TRAY_GREY),
    Box("tray_wall_near", pos=(TRAY_XY[0] - _wx, TRAY_XY[1], TOP + TRAY_WALL_H / 2), size=(TRAY_WALL_T, TRAY_IN[1] + 2 * TRAY_WALL_T, TRAY_WALL_H), color=TRAY_GREY),
    Box("tray_wall_far", pos=(TRAY_XY[0] + _wx, TRAY_XY[1], TOP + TRAY_WALL_H / 2), size=(TRAY_WALL_T, TRAY_IN[1] + 2 * TRAY_WALL_T, TRAY_WALL_H), color=TRAY_GREY),
    Box("tray_wall_left", pos=(TRAY_XY[0], TRAY_XY[1] + _wy, TOP + TRAY_WALL_H / 2), size=(TRAY_IN[0], TRAY_WALL_T, TRAY_WALL_H), color=TRAY_GREY),
    Box("tray_wall_right", pos=(TRAY_XY[0], TRAY_XY[1] - _wy, TOP + TRAY_WALL_H / 2), size=(TRAY_IN[0], TRAY_WALL_T, TRAY_WALL_H), color=TRAY_GREY),
]
OBJECTS = [BEAM_BOX, RED_BOX, GREEN_BOX, PIER_BOX, *LIPS, *TRAY]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

TRIP_TILT = math.radians(2.5)  # the beam's tilt from level that trips the disturbance record
TRIP_MOVE = 0.006  # the beam's centre, from its rest, in any direction
LEVEL_TOL = math.radians(1.5)  # the beam's tilt for full credit
FLAT_TOL = math.radians(3.0)  # a crib's height axis to vertical, either face up
CRIB_YAW_TOL = math.radians(10.0)  # a seated crib's long axis to x, either way round
BOTTOM_TOL = 0.002  # a seated crib's lowest corner to the table
BIN_TOL = 0.003  # a binned crib's lowest corner to the tray floor
SEAT_X = (BEAM_XY[0] - 0.035, BEAM_XY[0] + 0.035)  # a seated green crib's centre, x 0.255 to 0.325: within 35 mm of
# the beam's centreline either way, so the 100 mm crib covers at least 35 mm of the beam's 40 mm width
SEAT_Y = (-0.205, -0.090)  # and under its left half, at least 5 mm toward the left end from the beam's middle
TRAY_X = (TRAY_XY[0] - TRAY_IN[0] / 2, TRAY_XY[0] + TRAY_IN[0] / 2)  # inside the walls: x 0.085 to 0.215
TRAY_Y = (TRAY_XY[1] - TRAY_IN[1] / 2, TRAY_XY[1] + TRAY_IN[1] / 2)  # y -0.435 to -0.365
START_BEAM_TOL = 0.001  # the start state: the beam within 1 mm and 0.5 degrees of its rest
START_TILT_TOL = math.radians(0.5)
START_CRIB_TOL = 0.002  # each crib within 2 mm of its start mark, flat on the table
RESET_JUMP = 0.015  # a new trial: some free body's centre jumps more than 15 mm in one step
RESET_SPEED = 0.010  # while it reports under 10 mm/s both before and after the jump
FREE = ("beam", "crib_red", "crib_green")
EPS = 1e-6
SIGNS = np.array([(sx, sy, sz) for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _corners(state, size) -> np.ndarray:
    """The box's eight corners in the world, from its centre and quaternion."""
    return np.asarray(state.pos, float) + (SIGNS * (np.asarray(size) / 2)) @ _rot(state.quat_xyzw).T


def _tilt(state) -> float:
    """Angle between the body's own vertical axis and the world vertical, radians."""
    return math.acos(max(-1.0, min(1.0, float(_rot(state.quat_xyzw)[2, 2]))))


def _flat(state) -> bool:
    """The crib's height axis within 3 degrees of vertical, either face up."""
    return abs(float(_rot(state.quat_xyzw)[2, 2])) >= math.cos(FLAT_TOL)


def _along_x(state) -> bool:
    """The crib's long axis within 10 degrees of world x, either way round."""
    ex = _rot(state.quat_xyzw)[:, 0]
    off = abs(math.atan2(float(ex[1]), float(ex[0])))
    return min(off, math.pi - off) <= CRIB_YAW_TOL


def _hull(points) -> np.ndarray:
    """Convex hull of 2-D points, counter-clockwise (monotone chain): a box's outline seen from above."""
    pts = sorted({(round(float(p[0]), 9), round(float(p[1]), 9)) for p in points})

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and (out[-1][0] - out[-2][0]) * (p[1] - out[-2][1]) - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0]) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]

    return np.array(half(pts) + half(pts[::-1]))


def _overlap(a, b) -> float:
    """Penetration depth of two convex outlines (separating-axis test): zero or negative when they only touch or are apart."""
    depth = math.inf
    for poly in (a, b):
        for k in range(len(poly)):
            edge = poly[(k + 1) % len(poly)] - poly[k]
            axis = np.array([-edge[1], edge[0]]) / max(float(np.hypot(*edge)), 1e-12)
            pa, pb = a @ axis, b @ axis
            depth = min(depth, min(pa.max(), pb.max()) - max(pa.min(), pb.min()))
    return float(depth)


def _finite(objects) -> bool:
    """Every free body's centre and quaternion are finite numbers (a solver blow-up is not)."""
    return all(bool(np.all(np.isfinite(np.asarray(objects[n].pos, float))) and np.all(np.isfinite(np.asarray(objects[n].quat_xyzw, float)))) for n in FREE)


def _disturbed(beam) -> bool:
    """The beam tilted more than 2.5 degrees from level, or its centre more than 6 mm from its rest in any direction."""
    return _tilt(beam) > TRIP_TILT + EPS or float(np.linalg.norm(np.asarray(beam.pos, float) - BEAM_REST)) > TRIP_MOVE + EPS


def _on_mark(state, size, xy) -> bool:
    lowest = float(_corners(state, size)[:, 2].min())
    near = math.hypot(float(state.pos[0]) - xy[0], float(state.pos[1]) - xy[1]) <= START_CRIB_TOL
    return near and abs(lowest - TOP) <= START_CRIB_TOL and _flat(state) and _along_x(state)


def _at_start(objects) -> bool:
    """The start state: the beam at rest on its supports within 1 mm and 0.5 degrees, each crib on its mark."""
    beam = objects["beam"]
    home = float(np.linalg.norm(np.asarray(beam.pos, float) - BEAM_REST)) <= START_BEAM_TOL and _tilt(beam) <= START_TILT_TOL
    return bool(home and _on_mark(objects["crib_red"], CRIB_RED, RED_START) and _on_mark(objects["crib_green"], CRIB_GREEN, GREEN_START))


# The beam must hold still for the whole trial, not only at the end: a beam that dropped and was lifted back, or was
# shoved or lifted and put back before either crib moved, still counts as disturbed. The record is updated on every
# control step by success() and score(), which the runner calls after each step, and it is cleared only when a new
# trial starts: on the first call, or when the scene is in its start state (the beam on its supports, both cribs on
# their marks) and some free body's centre has jumped more than 15 mm since the previous call while reporting under
# 10 mm/s both before and after. env.reset() teleports the bodies back to their spawn poses like that. Physics cannot:
# a body that starts and ends a 50 ms step at rest, falling or sliding against 1 g of gravity or friction, covers at
# most 12 mm in it, so neither a slow return to the start state nor a short drop onto a mark (a beam lifted 4 mm and
# let go, a crib fumbled) clears the record. The cost: a trial that ends tripped with every body already within 15 mm
# of its mark passes its trip to the next epoch of the same run. A non-finite pose (a solver blow-up) trips it.
_HISTORY = {"tripped": False, "prev": None}


def _new_trial(objects) -> bool:
    prev = _HISTORY["prev"]
    if prev is None:
        return True
    if not _at_start(objects):
        return False
    for name in FREE:
        p0, v0 = prev[name]
        if not (np.all(np.isfinite(p0)) and math.isfinite(v0)):
            return True
        state = objects[name]
        jump = float(np.linalg.norm(np.asarray(state.pos, float) - p0))
        if jump > RESET_JUMP and v0 < RESET_SPEED and float(np.linalg.norm(state.lin_vel)) < RESET_SPEED:
            return True
    return False


def _track(objects) -> None:
    if _new_trial(objects):
        _HISTORY["tripped"] = False
    if not _finite(objects) or _disturbed(objects["beam"]):
        _HISTORY["tripped"] = True
    _HISTORY["prev"] = {n: (np.array(objects[n].pos, float), float(np.linalg.norm(objects[n].lin_vel))) for n in FREE}


def _calm(objects) -> bool:
    """Every free body's pose finite, and the beam undisturbed now and at every earlier step of the trial."""
    return not _HISTORY["tripped"] and _finite(objects) and not _disturbed(objects["beam"])


def _seated(green) -> bool:
    """The green crib at rest, flat, long axis within 10 degrees of x, its lowest corner within 2 mm of the table and its
    centre at x 0.255 to 0.325 and y -0.205 to -0.090: across at least 35 mm of the beam's 40 mm width and under its
    left half, the red crib's own place included."""
    if not green.resting() or not _flat(green) or not _along_x(green):
        return False
    x, y = float(green.pos[0]), float(green.pos[1])
    if not (SEAT_X[0] - EPS <= x <= SEAT_X[1] + EPS and SEAT_Y[0] - EPS <= y <= SEAT_Y[1] + EPS):
        return False
    return abs(float(_corners(green, CRIB_GREEN)[:, 2].min()) - TOP) <= BOTTOM_TOL + EPS


def _out(red, beam) -> bool:
    """The red crib at rest with no part of its outline, seen from above, inside the beam's outline seen from above,
    both built from the body's centre and quaternion, so a beam nudged or turned between the lips is followed."""
    return bool(red.resting() and _overlap(_hull(_corners(red, CRIB_RED)[:, :2]), _hull(_corners(beam, BEAM)[:, :2])) <= EPS)


def _binned(red) -> bool:
    """The red crib at rest, its centre inside the tray walls and its lowest corner within 3 mm of the tray floor."""
    if not red.resting():
        return False
    x, y = float(red.pos[0]), float(red.pos[1])
    if not (TRAY_X[0] <= x <= TRAY_X[1] and TRAY_Y[0] <= y <= TRAY_Y[1]):
        return False
    return abs(float(_corners(red, CRIB_RED)[:, 2].min()) - TRAY_FLOOR_TOP) <= BIN_TOL + EPS


def _ladder(objects) -> float:
    green, red, beam = objects["crib_green"], objects["crib_red"], objects["beam"]
    if not _seated(green):
        return 0.0
    if _binned(red) and beam.resting() and _tilt(beam) <= LEVEL_TOL + EPS:
        return 1.0
    if _out(red, beam):
        return 0.7
    return 0.4


def success(objects) -> bool:
    """The green crib seated, the red crib in the tray, the beam within 1.5 degrees of level, everything at rest and
    the beam never disturbed; holds for 1 s."""
    _track(objects)
    return _calm(objects) and _ladder(objects) == 1.0


def distance(objects) -> float:
    """Metres still to go: the green crib's centre to the middle of its seat, plus the red crib's centre to the tray's."""
    g, r = objects["crib_green"], objects["crib_red"]
    seat = (sum(SEAT_X) / 2, GREEN_START[1])
    return math.hypot(float(g.pos[0]) - seat[0], float(g.pos[1]) - seat[1]) + math.hypot(float(r.pos[0]) - TRAY_XY[0], float(r.pos[1]) - TRAY_XY[1])


def score(objects) -> float:
    """0 for the whole trial once the beam has tilted more than 2.5 degrees or its centre has moved more than 6 mm
    at any step. Otherwise 0 unless the green crib is seated; 0.4 seated; 0.7 seated with the red crib out from under
    the beam; 1.0 seated with the red crib in the tray and the beam within 1.5 degrees of level, all at rest."""
    _track(objects)
    return _ladder(objects) if _calm(objects) else 0.0


TRAVEL = TOP + 0.150
GREEN_GRIP_Z = TOP + 0.025  # pad tips 12 mm above the table, 27 mm of pad on the green crib
RED_GRIP_Z = TOP + 0.026  # pad tips 13 mm above the table, 27 mm of pad on the red crib
LIFT_GRIP_Z = TOP + 0.030  # the regrasp for the carry: pad tips 17 mm up, so they stay 10 mm over the tray walls at the release
# After each close the TCP takes up 0.3 mm, straight up, before it slides. In the simulator the arm's droop under the
# crib keeps the crib on the table (or under its load) anyway. In the offline clearance model the carried crib then
# rides 0.3 mm clear of the table and of the beam through the IK's tracking error (about 0.03 mm), instead of dipping
# into them by that error at the start of a diagonal move.
TAKE_UP = 0.0003
GRIP_DX = 0.005  # each slide grips 5 mm in from the crib's near (-x) end, as far from the beam as the crib allows
SLIDE = 0.120  # both slides: green in along +x, red out along -x
GREEN_SEAT = (GREEN_START[0] + SLIDE, GREEN_START[1])  # (0.270, -0.185): spans x 0.220 to 0.320 under the beam
TRAY_PLACE_Z = TRAY_FLOOR_TOP + 0.005 + (LIFT_GRIP_Z - TOP)  # red crib bottom 5 mm above the tray floor
TRAY_ENTRY = TOP + TRAY_WALL_H + 0.010 - (TRAY_FLOOR_TOP + 0.005)  # the pause: crib bottom 10 mm above the walls
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part
RELEASE_OPEN = -0.028  # 56 mm: 13 mm clear of a 30 mm crib per side as the fingers rise off it, 20 mm or more off the other crib
REACH_OPEN = -0.025  # 50 mm: the opening for the descents onto the red crib, 10 mm clear per side


def _pick_place(pick, place, *, width, entry, travel_z, yaw=0.0, place_yaw=None, pick_open=FINGER_OPEN, release_open=FINGER_OPEN):
    """A top-down pick and place paced for the simulated arm, whose soft wrist trails a moving target by several
    millimetres. The arm settles before every close and release, stops ``entry`` above the place with the part
    clear of every fixture, and makes the last descent slowly. The fingers close to 8 mm narrower than the part's
    ``width`` across the pads rather than fully shut: each finger then pushes 4 mm x 5000 N/m = 20 N, under its
    50 N cap, so the grip centres the part, whereas a fully commanded close saturates both fingers and lets the part
    slide onto one of them. The gripper opens and closes over 1.5 s, inside the default 0.05 per step limiter.
    ``pick_open`` and ``release_open`` open the fingers only part way where a full 94 mm opening would reach a
    fixture. 16.2 s."""
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    grip = -(width - SQUEEZE) / 2  # finger joint target, m
    return [
        Waypoint(1.5, (px, py, travel_z), pick_open, "to the object", yaw),
        Waypoint(1.5, (px, py, pz), pick_open, "descend", yaw),
        Waypoint(0.6, (px, py, pz), pick_open, "settle", yaw),
        Waypoint(1.5, (px, py, pz), grip, "close", yaw),
        Waypoint(1.2, (px, py, travel_z), grip, "lift", yaw),
        Waypoint(2.0, (qx, qy, travel_z), grip, "carry", end),
        Waypoint(0.6, (qx, qy, travel_z), grip, "settle over the place", end),
        Waypoint(1.2, (qx, qy, qz + entry), grip, "lower to the entry", end),
        Waypoint(0.8, (qx, qy, qz + entry), grip, "settle at the entry", end),
        Waypoint(2.0, (qx, qy, qz), grip, "set in", end),
        Waypoint(0.8, (qx, qy, qz), grip, "settle", end),
        Waypoint(1.5, (qx, qy, qz), release_open, "release", end),
        Waypoint(1.0, (qx, qy, travel_z), release_open, "retreat", end),
    ]


def _slide(start, end, z, *, width, label, approach_open, release_open):
    """Grip a crib near its end where it lies on the table, take up 0.3 mm, slide it along the table to ``end`` (TCP
    positions), let go and rise. The slide takes 4 s (about 45 mm/s at its fastest) with the crib riding on the table,
    and the arm settles before the close and before the release. 13.1 s."""
    grip = -(width - SQUEEZE) / 2
    (sx, sy), (ex, ey) = start, end
    return [
        Waypoint(1.5, (sx, sy, TRAVEL), approach_open, "to the crib"),
        Waypoint(1.5, (sx, sy, z), approach_open, "descend"),
        Waypoint(0.6, (sx, sy, z), approach_open, "settle"),
        Waypoint(1.5, (sx, sy, z), grip, "close"),
        Waypoint(0.5, (sx, sy, z + TAKE_UP), grip, "take up"),
        Waypoint(4.0, (ex, ey, z + TAKE_UP), grip, label),
        Waypoint(0.8, (ex, ey, z + TAKE_UP), grip, "settle"),
        Waypoint(1.5, (ex, ey, z + TAKE_UP), release_open, "release"),
        Waypoint(1.2, (ex, ey, TRAVEL), release_open, "rise"),
    ]


def demo():
    """Right arm only, every grip at yaw 0 with the fingers closing along y across a crib's 30 mm width. First the green
    crib: gripped 5 mm in from its near end and slid 120 mm along x on the table, under the beam beside the red crib,
    where it carries nothing yet (its top is 1 mm below the beam). Then the red crib: gripped 5 mm in from its near end
    and slid 120 mm out toward the arm, carrying the beam's left end until it clears and the beam settles 1 mm onto
    the green crib. Then the red crib is gripped again at its middle, lifted, carried to the tray at 150 mm and set
    in 5 mm above the tray floor. 42.4 s of plan, then the 3 s hold: 908 steps at 20 Hz."""
    g0 = (GREEN_START[0] - CRIB_GREEN[0] / 2 + GRIP_DX, GREEN_START[1])  # (0.105, -0.185)
    r0 = (RED_START[0] - CRIB_RED[0] / 2 + GRIP_DX, RED_START[1])  # (0.225, -0.110)
    right = _slide(g0, (g0[0] + SLIDE, g0[1]), GREEN_GRIP_Z, width=CRIB_GREEN[1], label="slide in", approach_open=FINGER_OPEN, release_open=RELEASE_OPEN)
    right += _slide(r0, (r0[0] - SLIDE, r0[1]), RED_GRIP_Z, width=CRIB_RED[1], label="pull out", approach_open=REACH_OPEN, release_open=RELEASE_OPEN)
    right += _pick_place(
        (RED_START[0] - SLIDE, RED_START[1], LIFT_GRIP_Z), (TRAY_XY[0], TRAY_XY[1], TRAY_PLACE_Z), width=CRIB_RED[1],
        entry=TRAY_ENTRY, travel_z=TRAVEL, pick_open=REACH_OPEN, release_open=RELEASE_OPEN,
    )
    return {"right": right}


def self_test(ObjectState=None):
    """The scoring rules and the disturbance record on hand-built states and sequences; runs with the real ObjectState
    or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def rmat(yaw=0.0, roll=0.0, pitch=0.0):  # yaw about z, then roll about the body's x, then pitch about its y
        r = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        r = r @ np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        return r @ np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])

    def quat(r):
        w = math.sqrt(max(1e-12, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
        return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])

    def state(size, centre, r, vel=(0.0, 0.0, 0.0)):
        half = np.abs(r) @ (np.asarray(size) / 2)
        pos = np.asarray(centre, float)
        return ObjectState(pos, quat(r), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def crib(size, x, y, bottom=TOP, vel=(0.0, 0.0, 0.0), **turn):
        r = rmat(**turn)
        half = np.abs(r) @ (np.asarray(size) / 2)
        return state(size, (x, y, bottom + half[2]), r, vel)

    def red(x=RED_START[0], y=RED_START[1], **kw):
        return crib(CRIB_RED, x, y, **kw)

    def green(x=GREEN_START[0], y=GREEN_START[1], **kw):
        return crib(CRIB_GREEN, x, y, **kw)

    def beam(dx=0.0, dy=0.0, dz=0.0, yaw=0.0, vel=(0.0, 0.0, 0.0)):
        return state(BEAM, BEAM_REST + np.array([dx, dy, dz]), rmat(yaw), vel)

    def beam_about(pivot_y, th, vel=(0.0, 0.0, 0.0)):
        """The beam turned by ``th`` about a line along x at its underside's rest height through y = ``pivot_y``; a
        positive ``th`` raises its left end."""
        pivot = np.array([BEAM_XY[0], pivot_y, BEAM_BOTTOM])
        rel = BEAM_REST - pivot
        ry = rel[1] * math.cos(th) - rel[2] * math.sin(th)
        rz = rel[1] * math.sin(th) + rel[2] * math.cos(th)
        return state(BEAM, pivot + np.array([rel[0], ry, rz]), rmat(roll=th), vel)

    def beam_pivoted(th, vel=(0.0, 0.0, 0.0)):
        """The beam turned by ``th`` about the pier's inner edge, its left end down."""
        return beam_about(PIER_INNER_Y, -th, vel)

    def beam_lifted(lift, vel=(0.0, 0.0, 0.0)):
        """The beam's free end raised ``lift``, the beam turning about its right end's lower edge on the pier."""
        return beam_about(BEAM_XY[1] - BEAM[1] / 2, math.asin(lift / BEAM[1]), vel)

    def beam_yawed(yaw):
        """The beam turned ``yaw`` about the vertical through the lips' centre, as far as the lips' play allows."""
        arm = BEAM_XY[1] - PIER_XY[1]  # 0.100
        return state(BEAM, BEAM_REST + np.array([-arm * math.sin(yaw), arm * (math.cos(yaw) - 1.0), 0.0]), rmat(yaw))

    seated_green = green(*GREEN_SEAT)
    red_pulled = red(RED_START[0] - SLIDE, RED_START[1])
    red_binned = red(*TRAY_XY, bottom=TRAY_FLOOR_TOP)
    nan_beam = state(BEAM, (math.nan, math.nan, math.nan), rmat())
    # Green slid in beside the red crib, the red crib pulled: the beam turns on the pier's inner edge and first meets the
    # green crib's far top edge, 110 mm out and 1 mm lower: 0.52 degrees, 1.8 mm down at the free end.
    settled = beam_pivoted(math.atan2(CRIB_RED[2] - CRIB_GREEN[2], (GREEN_SEAT[1] + CRIB_GREEN[1] / 2) - PIER_INNER_Y))
    # The red crib pulled first: the beam pivots until the lower corner of its left end, 200 mm out, reaches the table.
    dropped = beam_pivoted(math.asin((BEAM_BOTTOM - TOP) / ((BEAM_XY[1] + BEAM[1] / 2) - PIER_INNER_Y)))
    assert abs(math.degrees(_tilt(settled)) - 0.521) < 0.01 and abs(math.degrees(_tilt(dropped)) - 11.54) < 0.01
    # Lifting the free end trips the record at about 11 mm (2.5 degrees), before the centre has moved 6 mm.
    assert not _disturbed(beam_lifted(0.0113)) and _disturbed(beam_lifted(0.0120))

    def scene(b=None, r=None, g=None):
        return {"beam": b or beam(), "crib_red": r or red(), "crib_green": g or green()}

    cases = {
        "start": (scene(), 0.0, False),
        "green slid in beside the red crib, red still under": (scene(g=seated_green), 0.4, False),
        "green in, red pulled out onto the table, beam settled on green": (scene(settled, red_pulled, seated_green), 0.7, False),
        "done": (scene(settled, red_binned, seated_green), 1.0, True),
        "done with the beam still level": (scene(None, red_binned, seated_green), 1.0, True),
        "done, green under the beam end where the red crib stood": (scene(None, red_binned, green(*RED_START)), 1.0, True),
        "red pulled first: the beam dropped onto the table": (scene(dropped, red_pulled), 0.0, False),
        "red pulled first, green then under the dropped beam, red binned": (scene(dropped, red_binned, seated_green), 0.0, False),
        "beam lifted 8 mm and held, green in, red binned": (scene(beam(dz=0.008), red_binned, seated_green), 0.0, False),
        "beam dragged 7 mm along x, done otherwise": (scene(beam(dx=-0.007), red_binned, seated_green), 0.0, False),
        "beam centre 5.9 mm off, done otherwise": (scene(beam(dx=-0.0059), red_binned, seated_green), 1.0, True),
        "beam turned 1 degree between the lips, 2.5 mm off, done": (scene(beam(dx=-0.0025, yaw=math.radians(1)), red_binned, seated_green), 1.0, True),
        "beam tilted 2 degrees, done otherwise": (scene(beam_pivoted(math.radians(2.0)), red_binned, seated_green), 0.7, False),
        "beam tilted 3 degrees, done otherwise": (scene(beam_pivoted(math.radians(3.0)), red_binned, seated_green), 0.0, False),
        "done, the beam still moving": (scene(beam(vel=(0.0, 0.0, 0.02)), red_binned, seated_green), 0.7, False),
        "green seated but still moving": (scene(g=green(*GREEN_SEAT, vel=(0.02, 0.0, 0.0))), 0.0, False),
        "green held 3 mm up under the beam": (scene(g=green(*GREEN_SEAT, bottom=TOP + 0.003)), 0.0, False),
        "green crosswise under the beam": (scene(None, red_binned, green(0.290, -0.160, yaw=math.pi / 2)), 0.0, False),
        "green on its side under the beam": (scene(None, red_binned, green(*GREEN_SEAT, roll=math.pi / 2)), 0.0, False),
        "green upside down, seated": (scene(None, red_binned, green(*GREEN_SEAT, roll=math.pi)), 1.0, True),
        "green turned end for end, seated": (scene(None, red_binned, green(*GREEN_SEAT, yaw=math.pi)), 1.0, True),
        "green yawed 8 degrees": (scene(None, red_binned, green(*GREEN_SEAT, yaw=math.radians(8))), 1.0, True),
        "green yawed 12 degrees": (scene(None, red_binned, green(*GREEN_SEAT, yaw=math.radians(12))), 0.0, False),
        "green tilted 4 degrees": (scene(g=green(*GREEN_SEAT, pitch=math.radians(4))), 0.0, False),
        "green near the beam's middle, centre y -0.215": (scene(None, red_binned, green(0.270, -0.215)), 0.0, False),
        "green centre at y -0.205, the seat's inner limit": (scene(g=green(0.270, -0.205)), 0.4, False),
        "green centre at y -0.090, the seat's outer limit": (scene(None, red_binned, green(0.270, -0.090)), 1.0, True),
        "green on the beam's centreline, x 0.290": (scene(None, red_binned, green(0.290, -0.185)), 1.0, True),
        "green at x 0.310, under all 40 mm of the beam's width": (scene(None, red_binned, green(0.310, -0.185)), 1.0, True),
        "green at x 0.325, the seat's far limit, under 35 mm": (scene(None, red_binned, green(0.325, -0.185)), 1.0, True),
        "green 40 mm too far, centre x 0.330, under 30 mm": (scene(None, red_binned, green(0.330, -0.185)), 0.0, False),
        "green 20 mm short, centre x 0.250, under 30 mm": (scene(None, red_binned, green(0.250, -0.185)), 0.0, False),
        "green left on the table at its start": (scene(None, red_binned), 0.0, False),
        "green in the tray, red still under the beam": (scene(g=green(*TRAY_XY, bottom=TRAY_FLOOR_TOP)), 0.0, False),
        "red out but still moving": (scene(settled, red(*red_pulled.pos[:2], vel=(-0.02, 0.0, 0.0)), seated_green), 0.4, False),
        "red pulled 45 mm, 5 mm still under the beam": (scene(g=seated_green, r=red(RED_START[0] - 0.045)), 0.4, False),
        "red pulled 51 mm, 1 mm clear of the beam": (scene(settled, red(RED_START[0] - 0.051), seated_green), 0.7, False),
        "beam turned 0.95 degrees between the lips, red 3 mm under its near face": (scene(beam_yawed(math.radians(0.95)), red(RED_START[0] - 0.0505), seated_green), 0.4, False),
        "beam pose not finite (a solver blow-up), done otherwise": (scene(nan_beam, red_binned, seated_green), 0.0, False),
        "red pushed out the far side": (scene(settled, red(RED_START[0] + 0.092), seated_green), 0.7, False),
        "red lying on top of the beam": (scene(settled, red(0.290, -0.200, bottom=BEAM_BOTTOM + BEAM[2]), seated_green), 0.4, False),
        "red standing on end in the tray": (scene(settled, red(*TRAY_XY, bottom=TRAY_FLOOR_TOP, pitch=math.pi / 2), seated_green), 1.0, True),
        "red across the tray walls": (scene(settled, red(*TRAY_XY, bottom=TOP + TRAY_WALL_H, yaw=math.pi / 2), seated_green), 0.7, False),
        "red held 6 mm over the tray floor, moving": (scene(settled, red(*TRAY_XY, bottom=TRAY_FLOOR_TOP + 0.006, vel=(0.0, 0.0, -0.02)), seated_green), 0.4, False),
        "red on the table beside the tray": (scene(settled, red(0.150, -0.300), seated_green), 0.7, False),
        "red binned, green never moved, beam level": (scene(None, red_binned), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        _HISTORY.update(tripped=False, prev=None)
        got_success = success(st)
        got = score(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)

    start = scene()
    green_in = scene(g=seated_green)
    pulled = scene(settled, red_pulled, seated_green)
    done = scene(settled, red_binned, seated_green)
    up, down, out_y, back_y = (0.0, 0.0, 0.03), (0.0, 0.0, -0.03), (0.0, -0.05, 0.0), (0.0, 0.05, 0.0)
    sequences = {
        "make before break: green in, red out, red binned": ([start, green_in, pulled, done], 1.0, True),
        "break before make: red out, beam drops, green slid in, beam lifted level, red binned": ([start, scene(dropped, red_pulled), scene(None, red_pulled, seated_green), scene(None, red_binned, seated_green)], 0.0, False),
        "beam shoved 8 mm during the pull and pushed back": ([start, green_in, scene(beam(dx=-0.008), red(RED_START[0] - 0.030), seated_green), pulled, done], 0.0, False),
        "beam lifted 10 mm, red swapped under it, beam set down": ([start, scene(beam(dz=0.010)), scene(beam(dz=0.010), red_pulled, green(*RED_START)), scene(None, red_binned, green(*RED_START))], 0.0, False),
        "beam nudged 4 mm during the pull, then settled": ([start, green_in, scene(beam(dx=-0.004), red(RED_START[0] - 0.030), seated_green), pulled, done], 1.0, True),
        "beam slid 10 mm along y and back before either crib moved, then a clean swap": (
            [start, scene(beam(dy=-0.005, vel=out_y)), scene(beam(dy=-0.010)), scene(beam(dy=-0.005, vel=back_y)), scene(beam(dy=-0.002, vel=back_y)), start, green_in, pulled, done], 0.0, False),
        "free end lifted 20 mm and set back down before either crib moved, then a clean swap": (
            [start, scene(beam_lifted(0.010, up)), scene(beam_lifted(0.020)), scene(beam_lifted(0.010, down)), scene(beam_lifted(0.002, down)), start, green_in, pulled, done], 0.0, False),
        "beam slid 10 mm and pushed back to 0.9 mm off its rest, then a clean swap": (
            [start, scene(beam(dy=-0.005, vel=out_y)), scene(beam(dy=-0.010)), scene(beam(dy=-0.004, vel=back_y)), scene(beam(dy=-0.0009)), scene(beam(dy=-0.0009), g=seated_green), pulled, done], 0.0, False),
        "red pulled first, the beam lifted back and the red crib slid home, then a clean swap": (
            [start, scene(dropped, red_pulled), scene(beam_pivoted(math.radians(5.0), up), red_pulled), scene(beam_pivoted(math.radians(1.0), up), red(RED_START[0] - 0.030, vel=(0.03, 0.0, 0.0))), scene(None, red(RED_START[0] - 0.002, vel=(0.03, 0.0, 0.0))), start, green_in, pulled, done], 0.0, False),
        "coarse snapshots at rest: beam slid 10 mm, 5 mm, then home, then a clean swap": (
            [start, scene(beam(dy=-0.010)), scene(beam(dy=-0.005)), start, green_in, pulled, done], 0.0, False),
        "coarse snapshots at rest: free end lifted 20 mm, then home, then a clean swap": (
            [start, scene(beam_lifted(0.020)), start, green_in, pulled, done], 0.0, False),
        "coarse snapshots at rest: beam slid 10 mm, then 0.9 mm off its rest, then a clean swap": (
            [start, scene(beam(dy=-0.010)), scene(beam(dy=-0.0009)), scene(beam(dy=-0.0009), g=seated_green), pulled, done], 0.0, False),
        "beam tripped and put back, lifted 4 mm level and dropped onto its supports in one step, then a clean swap": (
            [start, scene(beam(dy=-0.005, vel=out_y)), scene(beam(dy=-0.010)), scene(beam(dy=-0.002, vel=back_y)), start, scene(beam(dz=0.004)), start, green_in, pulled, done], 0.0, False),
        "beam tripped and put back, green crib lifted 12 mm on its mark and fumbled in one step, then a clean swap": (
            [start, scene(beam(dy=-0.005, vel=out_y)), scene(beam(dy=-0.010)), scene(beam(dy=-0.002, vel=back_y)), start, scene(g=green(bottom=TOP + 0.012)), start, green_in, pulled, done], 0.0, False),
        "a dropped trial, then the simulator's reset puts every body back in one step, then a clean swap": ([start, scene(dropped, red_pulled), start, green_in, pulled, done], 1.0, True),
        "a clean trial, the reset, then a second clean trial": ([start, green_in, pulled, done, start, green_in, pulled, done], 1.0, True),
        "a trial ending in a solver blow-up, the reset, then a clean swap": ([start, scene(nan_beam), start, green_in, pulled, done], 1.0, True),
        "tripped, then green and red back near their marks but the beam still down": ([start, scene(dropped, red_pulled), scene(dropped), green_in, done], 0.0, False),
    }
    for name, (steps, want, want_success) in sequences.items():
        _HISTORY.update(tripped=False, prev=None)
        for st in steps:
            got_success = success(st)
            got = score(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    _HISTORY.update(tripped=False, prev=None)
    return len(cases) + len(sequences)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules and the disturbance record on hand-built states and sequences, then the
    # right arm's plan replayed through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error
    # under 0.5 mm.
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
