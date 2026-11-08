"""On the station table, in front of the left arm, a 1 g bench analogue of taking one panel out of a rack of panels
stored on edge. Four loose panel blocks, 50 x 60 x 100 mm, stand in a row across the table on four fixed aluminium
rails 20 mm tall, 0.5 mm apart at the foot, with open slots to the table between the rails: from right to left a grey
panel, a red panel, an inner blue panel and an outer blue panel. The grey and red panels stand centred on wide rails
and stand on their own. Each blue panel stands on a narrow rail under only the outer third of its foot, its centre
10 mm past the rail's inner edge, so the inner blue panel leans on the red one and the outer blue panel leans on the
inner blue one. A black mat lies in front of the rack's left end and a white inspection plate in front of its right
end. A trial scores 0 if any panel tilts more than 10 degrees at any control step; otherwise each blue panel standing
on the mat, the red panel standing on the plate once every moved panel is on the mat, and the grey panel left where it
stood earn credit.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/keystone-panel-unrack/keystone_panel_unrack.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Stand the red panel upright on the white inspection plate. Move only the panels you must to free it, and stand each one you move upright on the black mat. No panel may tip past 10 degrees, even in the gripper." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +0.305, 0.75) and (0, -0.305, 0.75).
PANEL = (0.050, 0.060, 0.100)  # depth along x, width along y, height
RACK_X = 0.270  # the panels' centre line
RAIL_X = 0.260  # the rails run from x 0.215 to 0.305: 30 mm proud in front of the panels, so the top camera
RAIL_LEN = 0.090  # sees how much of each panel's foot has a rail under it, and 10 mm behind
RAIL_H = 0.020
GAP = 0.0005  # between neighbouring panels at the foot
RED_Y = 0.240
B2_Y = RED_Y + PANEL[1] + GAP  # 0.3005, the inner blue panel
B1_Y = B2_Y + PANEL[1] + GAP  # 0.3610, the outer blue panel
G_Y = RED_Y - PANEL[1] - GAP  # 0.1795, the grey panel
LEAN = 0.010  # each blue panel's centre of mass past its rail's inner edge, about 2 px in the 320 x 176 top camera
# Rails as (y from, y to). The grey and red rails are centred under their panels (44 and 48 mm of the 60 mm foot); each
# blue rail lies wholly outside its panel's centre (14 and 20 mm). A blue panel overhangs its rail's inner edge by
# 40 mm over a 46.5 mm slot, so a blue panel that loses its support swings 30 degrees before its foot reaches the
# table (its face would meet the far rail at 30.7 degrees); its top stays clear of the grey panel.
RAILS = {
    "rail_grey": (G_Y - 0.022, G_Y + 0.022),  # 157.5 to 201.5 mm
    "rail_red": (RED_Y - 0.024, RED_Y + 0.024),  # 216.0 to 264.0 mm
    "rail_blue_inner": (B2_Y + LEAN, B2_Y + 0.024),  # 310.5 to 324.5 mm
    "rail_blue_outer": (B1_Y + LEAN, B1_Y + 0.030),  # 371.0 to 391.0 mm, flush with the panel's outer face
}
RED, B2, B1, G = "red_panel", "blue_inner", "blue_outer", "grey_panel"
STARTS = {RED: RED_Y, B2: B2_Y, B1: B1_Y, G: G_Y}  # y of each panel's centre, at x = RACK_X
HOME_Z = TOP + RAIL_H + PANEL[2] / 2
# The red panel's rail gives it a righting moment of 0.033 N m against the 0.014 N m with which the two blue panels
# push it (0.14 N at the top), 2.4 times over without the grey panel beside it.
MASS = {RED: 0.14, B2: 0.07, B1: 0.07, G: 0.07}
DEPENDANTS = (B1, B2)  # the panels that must move to free the red one
MAT_C = (0.160, 0.470)
MAT = (0.080, 0.240, 0.002)
MAT_TOP = TOP + MAT[2]
PLATE_C = (0.170, 0.080)
PLATE = (0.100, 0.100, 0.003)
PLATE_TOP = TOP + PLATE[2]

REDC = (0.80, 0.15, 0.12)
BLUE = (0.20, 0.40, 0.80)
GREY = (0.55, 0.57, 0.60)
RAILC = (0.75, 0.76, 0.78)
BLACK = (0.12, 0.12, 0.13)
WHITE = (0.92, 0.92, 0.90)
COLOURS = {RED: REDC, B2: BLUE, B1: BLUE, G: GREY}

PANELS = [Box(n, pos=(RACK_X, y, HOME_Z), size=PANEL, mass=MASS[n], color=COLOURS[n]) for n, y in STARTS.items()]
RAIL_BOXES = [Box(n, pos=(RAIL_X, (a + b) / 2, TOP + RAIL_H / 2), size=(RAIL_LEN, b - a, RAIL_H), color=RAILC) for n, (a, b) in RAILS.items()]
MAT_BOX = Box("mat", pos=(MAT_C[0], MAT_C[1], TOP + MAT[2] / 2), size=MAT, color=BLACK)
PLATE_BOX = Box("plate", pos=(PLATE_C[0], PLATE_C[1], TOP + PLATE[2] / 2), size=PLATE, color=WHITE)
OBJECTS = [*PANELS, *RAIL_BOXES, MAT_BOX, PLATE_BOX]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

FALL = math.radians(10.0)  # a panel's long axis this far from vertical, at any step, ends the trial's credit
UPRIGHT = math.radians(5.0)  # standing, for the end state
BOTTOM_TOL = 0.003
STAY_TOL = 0.010  # the grey panel counts as left in place within 10 mm of where it stood
START_TOL = 0.005  # the start pose, for the reset test: every panel within 5 mm and 3 degrees of where it spawns
START_TILT = math.radians(3.0)
JUMP = 0.020  # a reset moves a panel's centre more than 20 mm, or turns it more than 8 degrees, between two steps
JUMP_TILT = math.radians(8.0)
EPS = 1e-6


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


def _tilt(state) -> float:
    """Angle of the panel's 100 mm axis from vertical, either end up, from its quaternion."""
    return math.acos(min(1.0, abs(float(_rot(state.quat_xyzw)[2, 2]))))


def _bottom(state) -> float:
    """Height of the panel's lowest corner, from its centre and quaternion."""
    corners = np.asarray(state.pos, float) + (_SIGNS * (np.asarray(PANEL) / 2)) @ _rot(state.quat_xyzw).T
    return float(corners[:, 2].min())


def _base_inside(state, centre, size) -> bool:
    """All four corners of the panel's lower 50 x 60 mm face inside a ``size`` outline centred at ``centre``."""
    r = _rot(state.quat_xyzw)
    down = -1.0 if r[2, 2] >= 0 else 1.0  # the end of the 100 mm axis that faces down
    local = np.array([[sx * PANEL[0] / 2, sy * PANEL[1] / 2, down * PANEL[2] / 2] for sx in (-1, 1) for sy in (-1, 1)])
    xy = (np.asarray(state.pos, float) + local @ r.T)[:, :2]
    return bool(np.all(np.abs(xy[:, 0] - centre[0]) <= size[0] / 2 + EPS) and np.all(np.abs(xy[:, 1] - centre[1]) <= size[1] / 2 + EPS))


def _standing_on(state, surface) -> bool:
    """At rest, the long axis within 5 degrees of vertical and the lowest corner within 3 mm of ``surface``."""
    return bool(state.resting() and _tilt(state) <= UPRIGHT + EPS and abs(_bottom(state) - surface) <= BOTTOM_TOL + EPS)


def _parked(state) -> bool:
    """Standing on the mat with its whole base inside the mat's 80 x 240 mm outline."""
    return _base_inside(state, MAT_C, MAT) and _standing_on(state, MAT_TOP)


def _inspected(state) -> bool:
    """Standing on the plate with its whole base inside the plate's 100 x 100 mm outline."""
    return _base_inside(state, PLATE_C, PLATE) and _standing_on(state, PLATE_TOP)


def _home(name):
    return np.array([RACK_X, STARTS[name], HOME_Z])


def _stayed(state, name) -> bool:
    """At rest, upright within 5 degrees and its centre within 10 mm of where it spawned."""
    near = np.linalg.norm(np.asarray(state.pos, float) - _home(name)) <= STAY_TOL + EPS
    return bool(near and state.resting() and _tilt(state) <= UPRIGHT + EPS)


# The fall rule covers the whole trial, not only the end: a panel that tips past 10 degrees and is stood up again
# still counts, and so does a rack rebuilt by hand. The record is updated on every control step by success() and
# score(). It restarts only on an episode reset, which the scorer recognises as a jump back to the start pose: every
# panel within 5 mm and 3 degrees of where it spawns (the settled lean is 0.3 to 0.6 degrees) right after a step in
# which some panel's centre moved more than 20 mm or its tilt changed more than 8 degrees. No arm moves a panel that far
# in one 50 ms step and leaves the rack exactly at its start; a reset does it every time, from any end state, including
# a blue panel left fallen at 30 degrees in the rack (26 mm and 30 degrees).
_REC = {"fallen": False, "prev": None}


def _at_start(objects) -> bool:
    for name in STARTS:
        s = objects[name]
        if np.any(np.abs(np.asarray(s.pos, float) - _home(name)) > START_TOL) or _tilt(s) > START_TILT:
            return False
    return True


def _track(objects) -> None:
    now = {n: (np.asarray(objects[n].pos, float).copy(), _tilt(objects[n])) for n in STARTS}
    prev, _REC["prev"] = _REC["prev"], now
    jumped = prev is None or any(
        np.linalg.norm(now[n][0] - prev[n][0]) > JUMP or abs(now[n][1] - prev[n][1]) > JUMP_TILT for n in STARTS
    )
    if jumped and _at_start(objects):
        _REC["fallen"] = False
    elif any(now[n][1] > FALL for n in STARTS):
        _REC["fallen"] = True


def _red_done(objects) -> bool:
    """The red panel on the plate, both blue panels on the mat, and the grey panel either where it stood or on the mat."""
    grey = objects[G]
    return _inspected(objects[RED]) and all(_parked(objects[n]) for n in DEPENDANTS) and (_stayed(grey, G) or _parked(grey))


def success(objects) -> bool:
    """No panel ever past 10 degrees, both blue panels standing on the mat, the red panel standing on the plate and the
    grey panel still where it stood, all at rest; holds for 1 s."""
    _track(objects)
    return not _REC["fallen"] and _red_done(objects) and _stayed(objects[G], G)


def distance(objects) -> float:
    """Metres still to go: the red panel's centre to the plate centre, plus each blue panel's centre to the mat."""
    red = objects[RED]
    total = math.hypot(float(red.pos[0]) - PLATE_C[0], float(red.pos[1]) - PLATE_C[1])
    for n in DEPENDANTS:
        p = objects[n]
        dx = max(0.0, abs(float(p.pos[0]) - MAT_C[0]) - MAT[0] / 2)
        dy = max(0.0, abs(float(p.pos[1]) - MAT_C[1]) - MAT[1] / 2)
        total += math.hypot(dx, dy)
    return total


def score(objects) -> float:
    """0 if any panel has tilted past 10 degrees at any step of the trial. Otherwise 0.2 for each blue panel standing
    on the mat; 0.4 more if the red panel stands on the plate while both blue panels stand on the mat and the grey panel
    is where it stood or on the mat; and 0.2 more if, on top of that, the grey panel was left where it stood:
    0, 0.2, 0.4, 0.8 or 1.0."""
    _track(objects)
    if _REC["fallen"]:
        return 0.0
    tenths = 2 * sum(_parked(objects[n]) for n in DEPENDANTS)
    if _red_done(objects):
        tenths += 4 + (2 if _stayed(objects[G], G) else 0)
    return tenths / 10


TRAVEL = TOP + 0.150  # a carried panel's bottom 59 mm up, under the rack's 120 mm tops: it goes round in front
GRIP_Z = TOP + 0.111  # pad tips 98 mm up, 22 mm below the panel tops; the fingers' 59 mm wide section starts about
#                       30 mm above the neighbours' tops, and their narrow tips stay 16.8 mm from the neighbours' faces
TCP_TO_BOTTOM = GRIP_Z - (TOP + RAIL_H)  # 91 mm
LIFT_Z = GRIP_Z + 0.015  # panel bottom 15 mm above the rails while it leaves the rack
AWAY = 0.010  # the sideways step toward +y, into the space the panel before it left, before the slide out
MAT_PLACE_Z = MAT_TOP + 0.005 + TCP_TO_BOTTOM  # panel bottom 5 mm above the mat at release
PLATE_PLACE_Z = PLATE_TOP + 0.005 + TCP_TO_BOTTOM
ENTRY = 0.010  # the pause above the place: panel bottom 15 mm above the mat or plate
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part
SPOTS = {B1: (0.160, 0.540), B2: (0.160, 0.460), RED: PLATE_C}  # each base 15 mm inside the mat's long sides
# Gripper yaw at the pick and at the place. At either yaw the fingers close along x, across the 50 mm depth, and a
# panel turned a half turn looks the same. joint6 must stay inside +-120 degrees: the mat, left of the arm's base,
# needs +90; the plate, right of it, needs -90, so the red panel is picked at -90 after a wrist turn through yaw 0
# above the emptied left half of the rack.
YAWS = {B1: (math.pi / 2, math.pi / 2), B2: (math.pi / 2, math.pi / 2), RED: (-math.pi / 2, -math.pi / 2)}


def _unrack(name, place_z, came_from_yaw):
    """One panel out of the rack and onto its place, paced for the simulated arm, whose soft wrist trails a moving
    target by several millimetres. A top-down pick and place with a rack exit added: after the close the panel is
    lifted 15 mm off its rail, stepped 10 mm toward +y into the space the previous panel left, and slid out along -x
    to the place's x, so it never rubs a panel still in the rack and never passes over one. Then it rises to travel
    height, is carried along y, paused 10 mm above the place, set down slowly and released 5 mm up. The fingers close
    to 8 mm narrower than the 50 mm depth: each pushes about 20 N, under its 50 N cap, and the pair centres the panel.
    The gripper opens and closes over 1.5 s, inside the 0.05 per step limiter. A wrist that has to turn a half turn
    does it in two quarter turns through yaw 0. 20.5 s, plus 0.4 s for a half turn."""
    sx, sy = RACK_X, STARTS[name]
    ay = sy + AWAY
    qx, qy = SPOTS[name]
    pick_yaw, place_yaw = YAWS[name]
    grip = -(PANEL[0] - SQUEEZE) / 2  # finger joint target, m
    out = []
    if abs(came_from_yaw - pick_yaw) > math.pi / 2:  # turn on the way over, above the emptied rack
        out.append(Waypoint(1.2, ((0.160 + sx) / 2, sy + 0.075, TRAVEL), FINGER_OPEN, "turn the wrist", 0.0))
        out.append(Waypoint(1.2, (sx, sy, TRAVEL), FINGER_OPEN, "over the panel", pick_yaw))
    else:
        out.append(Waypoint(2.0, (sx, sy, TRAVEL), FINGER_OPEN, "over the panel", pick_yaw))
    out += [
        Waypoint(1.5, (sx, sy, GRIP_Z), FINGER_OPEN, "descend", pick_yaw),
        Waypoint(0.6, (sx, sy, GRIP_Z), FINGER_OPEN, "settle", pick_yaw),
        Waypoint(1.5, (sx, sy, GRIP_Z), grip, "close", pick_yaw),
        Waypoint(1.0, (sx, sy, LIFT_Z), grip, "lift off the rail", pick_yaw),
        Waypoint(1.0, (sx, ay, LIFT_Z), grip, "step away from the neighbour", pick_yaw),
        Waypoint(2.0, (qx, ay, LIFT_Z), grip, "slide out of the rack", pick_yaw),
        Waypoint(1.0, (qx, ay, TRAVEL), grip, "rise", pick_yaw),
    ]
    if abs(place_yaw - pick_yaw) > math.pi / 2:  # turn the panel in front of the rack, clear of everything
        out.append(Waypoint(1.2, (qx, (ay + qy) / 2, TRAVEL), grip, "carry, turning", 0.0))
        out.append(Waypoint(1.2, (qx, qy, TRAVEL), grip, "carry, turned", place_yaw))
    else:
        out.append(Waypoint(2.0, (qx, qy, TRAVEL), grip, "carry", place_yaw))
    out += [
        Waypoint(0.6, (qx, qy, TRAVEL), grip, "settle over the place", place_yaw),
        Waypoint(1.2, (qx, qy, place_z + ENTRY), grip, "lower to the entry", place_yaw),
        Waypoint(0.8, (qx, qy, place_z + ENTRY), grip, "settle at the entry", place_yaw),
        Waypoint(2.0, (qx, qy, place_z), grip, "set down", place_yaw),
        Waypoint(0.8, (qx, qy, place_z), grip, "settle", place_yaw),
        Waypoint(1.5, (qx, qy, place_z), FINGER_OPEN, "release", place_yaw),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, "retreat", place_yaw),
    ]
    return out


def demo():
    """Left arm only, along the lean chain: the outer blue panel to the far end of the mat, the inner blue panel to
    its middle, then the red panel to the plate. The grey panel is never touched. Every grip is top-down with the
    fingers closing along x across the panel's 50 mm depth. 61.9 s of plan, then the 3 s hold: 1,298 steps at 20 Hz."""
    left, yaw = [], math.pi / 2
    for name in (B1, B2, RED):
        left += _unrack(name, PLATE_PLACE_Z if name == RED else MAT_PLACE_Z, yaw)
        yaw = YAWS[name][1]
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states and the fall record on sequences of states; runs with the real
    ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw=0.0, roll=0.0, pitch=0.0):  # yaw about z, then roll about the body's x, then pitch about its y
        r = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        r = r @ np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        r = r @ np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        w = math.sqrt(max(1e-12, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
        if w < 1e-3:  # a half turn: take the quaternion from the largest diagonal term
            i = int(np.argmax(np.diag(r)))
            j, k = (i + 1) % 3, (i + 2) % 3
            s = math.sqrt(max(1e-12, 1 + r[i, i] - r[j, j] - r[k, k])) * 2
            q = np.zeros(4)
            q[i], q[j], q[k], q[3] = s / 4, (r[j, i] + r[i, j]) / s, (r[k, i] + r[i, k]) / s, (r[k, j] - r[j, k]) / s
            return q
        return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])

    def make(pos, q, vel=(0.0, 0.0, 0.0)):
        half = np.abs(_rot(q)) @ (np.asarray(PANEL) / 2)
        pos = np.asarray(pos, float)
        return ObjectState(pos, np.asarray(q, float), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def panel(x, y, bottom, yaw=0.0, roll=0.0, pitch=0.0, vel=(0.0, 0.0, 0.0)):
        q = quat(yaw, roll, pitch)
        half = np.abs(_rot(q)) @ (np.asarray(PANEL) / 2)
        return make((x, y, bottom + half[2]), q, vel)

    def leaning(name, roll):
        """A blue panel in the rack, turned ``roll`` about its rail's inner top edge (positive tips the top toward -y)."""
        pivot_y = RAILS[{B2: "rail_blue_inner", B1: "rail_blue_outer"}[name]][0]
        c = np.array([STARTS[name] - pivot_y, PANEL[2] / 2])  # centre from the pivot, in (y, z), upright
        rc = np.array([c[0] * math.cos(roll) - c[1] * math.sin(roll), c[0] * math.sin(roll) + c[1] * math.cos(roll)])
        return make((RACK_X, pivot_y + rc[0], TOP + RAIL_H + rc[1]), quat(0.0, roll))

    def path(a, b, step=0.008, turn=math.radians(3.0)):
        """States from ``a`` to ``b`` at most ``step`` metres and ``turn`` radians apart, as a carry at 20 Hz gives."""
        n = 1
        for k in a:
            pa, pb = np.asarray(a[k].pos, float), np.asarray(b[k].pos, float)
            n = max(n, math.ceil(np.linalg.norm(pb - pa) / step), math.ceil(abs(_tilt(b[k]) - _tilt(a[k])) / turn))
        out = []
        for i in range(1, n + 1):
            t, st = i / n, {}
            for k in a:
                qa, qb = np.asarray(a[k].quat_xyzw, float), np.asarray(b[k].quat_xyzw, float)
                qb = -qb if qa @ qb < 0 else qb
                q = (1 - t) * qa + t * qb
                st[k] = make((1 - t) * np.asarray(a[k].pos, float) + t * np.asarray(b[k].pos, float), q / np.linalg.norm(q))
            out.append(st if i < n else b)
        return out

    lean = {B2: math.radians(0.29), B1: math.radians(0.57)}
    spawn = {n: panel(RACK_X, y, TOP + RAIL_H) for n, y in STARTS.items()}
    start = {**spawn, B2: leaning(B2, lean[B2]), B1: leaning(B1, lean[B1])}

    def on_mat(name, dx=0.0, dy=0.0, **kw):
        x, y = {**SPOTS, G: (0.160, 0.390)}[name]
        return panel(x + dx, y + dy, MAT_TOP, **kw)

    def on_plate(dx=0.0, dy=0.0, bottom=PLATE_TOP, **kw):
        return panel(PLATE_C[0] + dx, PLATE_C[1] + dy, bottom, **kw)

    def held(x, y, bottom, **kw):
        return panel(x, y, bottom, vel=(0.0, 0.05, 0.0), **kw)

    def scene(**moved):
        names = {"red": RED, "b1": B1, "b2": B2, "g": G}
        return {**start, **{names[k]: v for k, v in moved.items()}}

    def finished(**moved):
        return scene(**{"b1": on_mat(B1), "b2": on_mat(B2), "red": on_plate(), **moved})

    done = finished()
    fell = math.radians(30.0)
    red_first = scene(b2=leaning(B2, fell), b1=leaning(B1, math.radians(26.0)), red=held(0.170, RED_Y + AWAY, TOP + 0.035))
    b2_first = scene(b1=leaning(B1, fell), b2=held(0.160, B2_Y + AWAY, TOP + 0.035))
    left_fallen = scene(b1=leaning(B1, fell))
    mat_far = (MAT_C[0] + MAT[0] / 2 - PANEL[0] / 2, MAT_C[1] + MAT[1] / 2 - PANEL[1] / 2)  # base on the far corner
    cases = {
        "spawn, upright": ({**spawn}, 0.0, False),
        "start, settled lean": (scene(), 0.0, False),
        "done": (done, 1.0, True),
        "outer blue parked": (scene(b1=on_mat(B1)), 0.2, False),
        "both blues parked": (scene(b1=on_mat(B1), b2=on_mat(B2)), 0.4, False),
        "both blues parked, red held over the plate": (finished(red=held(*PLATE_C, PLATE_TOP + 0.006)), 0.4, False),
        "outside in: the grey panel parked too": (finished(g=on_mat(G)), 0.8, False),
        "the grey panel moved to the table": (finished(g=panel(0.170, 0.200, TOP)), 0.4, False),
        "the grey panel slid 12 mm along its rail": (finished(g=panel(RACK_X, G_Y - 0.012, TOP + RAIL_H)), 0.4, False),
        "done, the grey panel bumped 8 mm": (finished(g=panel(RACK_X, G_Y - 0.008, TOP + RAIL_H)), 1.0, True),
        "done, the grey panel still rocking": (finished(g=panel(RACK_X, G_Y, TOP + RAIL_H, vel=(0.0, 0.02, 0.0))), 0.4, False),
        "the blues slid on their rails until each stands alone, red on the plate": (
            scene(b1=panel(RACK_X, B1_Y + 0.012, TOP + RAIL_H), b2=panel(RACK_X, B2_Y + 0.012, TOP + RAIL_H), red=on_plate()), 0.0, False),
        "red on the plate, blues still in the rack": (scene(red=on_plate()), 0.0, False),
        "red pulled first: both blues swung to 26 and 30 degrees": (red_first, 0.0, False),
        "inner blue pulled first: outer blue swung 30 degrees": (b2_first, 0.0, False),
        "done, but a blue panel lies on the mat": (finished(b2=panel(*SPOTS[B2], MAT_TOP, roll=math.pi / 2)), 0.0, False),
        "done, but the grey panel leans 11 degrees": (finished(g=panel(RACK_X, G_Y, TOP + RAIL_H, roll=math.radians(11))), 0.0, False),
        "a blue panel leaning 9 degrees, nothing parked": (scene(b2=leaning(B2, math.radians(9))), 0.0, False),
        "done, a mat panel upside down": (finished(b1=on_mat(B1, roll=math.pi)), 1.0, True),
        "done, panels turned on the mat and the plate": (finished(b1=on_mat(B1, yaw=math.pi / 2), b2=on_mat(B2, yaw=math.pi), red=on_plate(yaw=1.0)), 1.0, True),
        "done, red 24 mm off the plate centre, base inside": (finished(red=on_plate(dx=0.024)), 1.0, True),
        "red 21 mm off the plate centre, base 1 mm over the edge": (finished(red=on_plate(dy=-0.021)), 0.4, False),
        "done, red tilted 4 degrees on the plate": (finished(red=on_plate(pitch=math.radians(4))), 1.0, True),
        "red tilted 6 degrees on the plate": (finished(red=on_plate(pitch=math.radians(6))), 0.4, False),
        "red on the plate, still moving": (finished(red=on_plate(vel=(0.02, 0.0, 0.0))), 0.4, False),
        "red held 4 mm above the plate, still": (finished(red=on_plate(bottom=PLATE_TOP + 0.004)), 0.4, False),
        "red standing on the mat, both blues parked": (finished(red=on_mat(G)), 0.4, False),
        "red on the table beside the plate": (finished(red=panel(PLATE_C[0], PLATE_C[1] + 0.090, TOP)), 0.4, False),
        "a blue panel standing on the plate, the other parked": (scene(b1=on_plate(), b2=on_mat(B2)), 0.2, False),
        "a blue panel on the table beside the mat": (scene(b1=panel(MAT_C[0] + 0.080, 0.500, TOP)), 0.0, False),
        "a blue panel's base 1 mm past the mat's far corner": (scene(b1=panel(mat_far[0] + 0.001, mat_far[1], MAT_TOP)), 0.0, False),
        "a blue panel's base just inside the mat's far corner": (scene(b1=panel(mat_far[0] - 0.0005, mat_far[1] - 0.0005, MAT_TOP)), 0.2, False),
        "a blue panel standing on the other on the mat": (scene(b2=on_mat(B2), b1=panel(*SPOTS[B2], MAT_TOP + PANEL[2])), 0.2, False),
        "a parked panel leaning 4 degrees on its neighbour": (scene(b1=on_mat(B1, roll=math.radians(4)), b2=on_mat(B2)), 0.4, False),
        "a parked panel rocking, still moving": (scene(b1=on_mat(B1, vel=(0.0, 0.03, 0.0))), 0.0, False),
        "a panel lying on the rails": (scene(b1=panel(RACK_X, 0.330, TOP + RAIL_H, pitch=math.pi / 2)), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        _REC.update(fallen=False, prev=None)
        got_success = success(st)
        got = score(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)

    b1_up = scene(b1=held(0.160, B1_Y + AWAY, TOP + 0.035, roll=lean[B1]))
    s1 = scene(b1=on_mat(B1))
    b2_up = scene(b1=on_mat(B1), b2=held(0.160, B2_Y + AWAY, TOP + 0.035))
    s2 = scene(b1=on_mat(B1), b2=on_mat(B2))
    red_up = finished(red=held(0.170, RED_Y + AWAY, TOP + 0.035))
    slipped = scene(b1=on_mat(B1), b2=held(0.160, 0.400, TOP + 0.060, roll=math.radians(12)))
    wobble = scene(b1=on_mat(B1, roll=math.radians(8), vel=(0.0, 0.05, 0.0)))
    in_order = [start, b1_up, s1, b2_up, s2, red_up, done]
    sequences = {
        "along the chain: outer blue, inner blue, red, grey untouched": ([spawn, *in_order], 1.0, True),
        "red first, then everything stood up and placed": ([start, red_first, done], 0.0, False),
        "inner blue first, outer blue swings, then all placed": ([start, b2_first, done], 0.0, False),
        "a panel slips 12 degrees in the grip, then is placed": ([start, s1, slipped, done], 0.0, False),
        "a parked panel rocks 8 degrees and settles": ([start, s1, wobble, s1, done], 1.0, True),
        "a fall, then the rack rebuilt exactly to its start and unloaded again": ([start, red_first, *path(red_first, start), *in_order[1:]], 0.0, False),
        "a fall, an episode reset, then a clean trial": ([start, red_first, done, *in_order], 1.0, True),
        "a blue panel left fallen in the rack, an episode reset, then a clean trial": ([start, left_fallen, *in_order], 1.0, True),
        "a fall, then an episode that ends in the rack, still fallen": ([start, b2_first, left_fallen], 0.0, False),
    }
    for name, (steps, want, want_success) in sequences.items():
        _REC.update(fallen=False, prev=None)
        for st in steps:
            got_success = success(st)
            got = score(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    _REC.update(fallen=False, prev=None)
    return len(cases) + len(sequences)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states and sequences, then the left arm's plan replayed
    # through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
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
