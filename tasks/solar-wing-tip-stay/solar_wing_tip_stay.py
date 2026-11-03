"""On the station table, a 1 g bench analogue of solar array integration: a wing is set with its root in a yoke and
its free end on a support stand set to the hinge-line height, because the root cannot carry the cantilever. A
fixed grey yoke just left of the midline holds a 68 mm rail channel 50 mm above the table, closed by an end stop.
A fixed collar centred 100 mm to the right of the yoke's edge takes one stay. Two orange stays of one diameter wait
in the right arm's reach, 50 mm and 30 mm tall; only the 50 mm one is as tall as the yoke block. A dark blue wing,
220 x 60 x 20 mm, lies in the left arm's reach. Set in the channel with no stay, the wing tips over the yoke's edge
onto the collar; on the 30 mm stay it tilts about 10 degrees; on the 50 mm stay it rests level. The instruction
names no height, so the stay must be matched to the block top. The score reads the stay from its centre and axis
and the wing from its centre and quaternion, at the bottom edges of its two end faces, and gives nothing until
the 50 mm stay stands in the collar.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/solar-wing-tip-stay/solar_wing_tip_stay.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "In the collar, stand the orange stay whose height matches the top of the yoke block, then seat the blue wing with its root between the rails against the stop and its tip level on the stay." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
WING = (0.220, 0.060, 0.020)  # long, wide, thick; long axis along x at the start
WING_MASS = 0.15
WING_START = (0.250, 0.350)
STAY_R = 0.015
STAY_H = 0.050  # as tall as the yoke block
SHORT_H = 0.030
STAY_START = (0.220, -0.320)
SHORT_START = (0.320, -0.330)

AXIS_X = 0.220  # the yoke's channel centreline and the collar centre, along x
YOKE_TOP = 0.050  # block top above the table: the hinge-line height
BLOCK = ((AXIS_X, 0.095), (0.100, 0.090, YOKE_TOP))  # spans x 0.170 to 0.270, y 0.050 to 0.140
RAIL = (0.010, 0.060, 0.040)  # on the block from y 0.070 to 0.130, tops 90 mm above the table
RAIL_OFF = 0.039  # rail centre from the centreline: inner faces at x 0.186 and 0.254, a 68 mm channel
RAIL_Y = 0.100
STOP = (0.088, 0.010, 0.040)  # on the block from y 0.130 to 0.140, joining the rail ends
STOP_Y = 0.135
STOP_FACE = 0.130
EDGE_Y = 0.050  # the block's edge toward the collar
COLLAR = (AXIS_X, -0.050)
POCKET = 0.040  # square, 5 mm of slop per side around the 30 mm stay
COLLAR_T = 0.007
COLLAR_H = 0.008

GREY = (0.75, 0.75, 0.75)
BLUE = (0.10, 0.15, 0.45)
ORANGE = (0.95, 0.55, 0.10)

_off = POCKET / 2 + COLLAR_T / 2
_fix_z = TOP + YOKE_TOP + RAIL[2] / 2
WING_BOX = Box("wing", pos=(WING_START[0], WING_START[1], TOP + WING[2] / 2), size=WING, mass=WING_MASS, color=BLUE)
STAY = Cylinder("stay", pos=(STAY_START[0], STAY_START[1], TOP + STAY_H / 2), radius=STAY_R, height=STAY_H, mass=0.04, color=ORANGE)
SHORT_STAY = Cylinder("short_stay", pos=(SHORT_START[0], SHORT_START[1], TOP + SHORT_H / 2), radius=STAY_R, height=SHORT_H, mass=0.025, color=ORANGE)
YOKE = [
    Box("yoke_block", pos=(BLOCK[0][0], BLOCK[0][1], TOP + YOKE_TOP / 2), size=BLOCK[1], color=GREY),
    Box("yoke_rail_p", pos=(AXIS_X + RAIL_OFF, RAIL_Y, _fix_z), size=RAIL, color=GREY),
    Box("yoke_rail_n", pos=(AXIS_X - RAIL_OFF, RAIL_Y, _fix_z), size=RAIL, color=GREY),
    Box("yoke_stop", pos=(AXIS_X, STOP_Y, _fix_z), size=STOP, color=GREY),
]
COLLAR_WALLS = [
    Box("collar_wall_yp", pos=(COLLAR[0], COLLAR[1] + _off, TOP + COLLAR_H / 2), size=(POCKET + 2 * COLLAR_T, COLLAR_T, COLLAR_H), color=GREY),
    Box("collar_wall_yn", pos=(COLLAR[0], COLLAR[1] - _off, TOP + COLLAR_H / 2), size=(POCKET + 2 * COLLAR_T, COLLAR_T, COLLAR_H), color=GREY),
    Box("collar_wall_xp", pos=(COLLAR[0] + _off, COLLAR[1], TOP + COLLAR_H / 2), size=(COLLAR_T, POCKET, COLLAR_H), color=GREY),
    Box("collar_wall_xn", pos=(COLLAR[0] - _off, COLLAR[1], TOP + COLLAR_H / 2), size=(COLLAR_T, POCKET, COLLAR_H), color=GREY),
]
OBJECTS = [WING_BOX, STAY, SHORT_STAY, *YOKE, *COLLAR_WALLS]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

EPS = 1e-4  # 0.1 mm, so a value exactly on a limit passes
STAY_XY_TOL = 0.006  # stay centre to the collar centre, along x and along y; the pocket allows 5
STAY_BOTTOM_TOL = 0.003
STAY_TILT = math.radians(5.0)
ROOT_X_TOL = 0.005  # root end to the centreline; the channel allows 4
ROOT_Y = (STOP_FACE - 0.010, STOP_FACE + 0.002)  # within 10 mm of the stop face, up to 2 mm of solver penetration
ROOT_MAX_Z = 0.085  # seated root 50 mm, tipped root about 74 mm, rail tops 90 mm
LEVEL_Z = (0.046, 0.054)  # both ends' undersides above the table: level within 2 degrees
TIP_X_TOL = 0.010
ROLL_TOL = math.radians(10.0)  # the wing's 60 mm width within 10 degrees of horizontal: lying on a broad face
LONG_TILT = math.radians(25.0)  # its 220 mm length: a seated wing is level, a tipped one 18.3 deg, one on its end near 90
SEAT_CENTRE = (AXIS_X, STOP_FACE - 0.004 - WING[0] / 2)  # the nominal wing centroid, root end 4 mm from the stop


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _bottom(state, height, r) -> float:
    """A cylinder's lowest point from its centre and axis, which the axis-aligned bounds of a tilted body overstate."""
    uz = float(_rot(state.quat_xyzw)[2, 2])  # world z of the cylinder axis
    return float(state.pos[2]) - (height / 2 * abs(uz) + r * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _stay_set(stay) -> bool:
    """S: the 50 mm stay upright in the collar pocket, at rest: centre within 6 mm of the collar centre along x and
    along y, bottom within 3 mm of the table, axis within 5 degrees of vertical."""
    if not stay.resting():
        return False
    near = abs(float(stay.pos[0]) - COLLAR[0]) <= STAY_XY_TOL + EPS and abs(float(stay.pos[1]) - COLLAR[1]) <= STAY_XY_TOL + EPS
    upright = abs(float(_rot(stay.quat_xyzw)[2, 2])) >= math.cos(STAY_TILT)
    return bool(near and upright and abs(_bottom(stay, STAY_H, STAY_R) - TOP) <= STAY_BOTTOM_TOL + EPS)


def _wing_ends(wing):
    """The midpoints of the bottom edges of the wing's two end faces, from its centre and quaternion, root first: the
    root is the end nearer the stop. The bottom is whichever broad face points down, so an upside-down wing reads
    the same. Also returns the world z of the wing's width axis."""
    rot = _rot(wing.quat_xyzw)
    long_axis, normal = rot[:, 0], rot[:, 2]
    down = -normal if normal[2] >= 0 else normal
    c = np.asarray(wing.pos, float)
    a = c + long_axis * WING[0] / 2 + down * WING[2] / 2
    b = c - long_axis * WING[0] / 2 + down * WING[2] / 2

    def to_stop(p):
        return math.hypot(float(p[0]) - AXIS_X, float(p[1]) - STOP_FACE)

    root, tip = (a, b) if to_stop(a) <= to_stop(b) else (b, a)
    return root, tip, float(rot[2, 1])


def _wing_levels(wing):
    """(W, L). W: the root captured, the wing at rest lying on a broad face (width within 10 degrees of horizontal and
    length within 25 degrees, so a wing stood on its end does not count), the root end's bottom edge within 5 mm of
    the centreline, 0 to 10 mm from the stop face and at most 85 mm above the table. L: W, and both end bottoms 46 to
    54 mm above the table with the tip end within 10 mm of the centreline."""
    if not wing.resting():
        return False, False
    root, tip, width_z = _wing_ends(wing)
    captured = (
        abs(width_z) <= math.sin(ROLL_TOL)
        and abs(float(_rot(wing.quat_xyzw)[2, 0])) <= math.sin(LONG_TILT)
        and abs(float(root[0]) - AXIS_X) <= ROOT_X_TOL + EPS
        and ROOT_Y[0] - EPS <= float(root[1]) <= ROOT_Y[1] + EPS
        and float(root[2]) - TOP <= ROOT_MAX_Z + EPS
    )
    if not captured:
        return False, False
    level = all(LEVEL_Z[0] - EPS <= float(p[2]) - TOP <= LEVEL_Z[1] + EPS for p in (root, tip))
    return True, bool(level and abs(float(tip[0]) - AXIS_X) <= TIP_X_TOL + EPS)


def _rungs(objects):
    s = _stay_set(objects["stay"])
    w, lv = _wing_levels(objects["wing"])
    return s, w, lv


def success(objects) -> bool:
    """The 50 mm stay set in the collar and the wing seated level on the yoke and the stay, both at rest; holds 1 s."""
    s, _, lv = _rungs(objects)
    return s and lv


def distance(objects) -> float:
    """Metres still to go: the stay's centre to the collar centre plus the wing's centre to its seated centre."""
    st, wg = objects["stay"], objects["wing"]
    d_stay = math.hypot(float(st.pos[0]) - COLLAR[0], float(st.pos[1]) - COLLAR[1])
    d_wing = math.hypot(float(wg.pos[0]) - SEAT_CENTRE[0], float(wg.pos[1]) - SEAT_CENTRE[1])
    return d_stay + d_wing


def score(objects) -> float:
    """Gated on S, the support that must go in first: 0 without S, whatever the wing does (seated with no stay it
    tips onto the collar, and on the 30 mm stay it tilts, the two failures this task tests); 0.25 for S alone; 0.5
    for S and W without L; 1.0 for S and L. S, W and L are defined in _stay_set and _wing_levels."""
    s, w, lv = _rungs(objects)
    if not s:
        return 0.0
    return 1.0 if lv else (0.5 if w else 0.25)


TRAVEL = TOP + 0.150
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part
STAY_GRIP_Z = TOP + STAY_H / 2  # TCP at the stay's mid-height, pad tips 12 mm above the table
STAY_PLACE_Z = TOP + STAY_H / 2 + 0.005  # stay bottom 5 mm above the pocket floor, pad tips 9 mm above the walls
STAY_ENTRY = 0.015  # the pause above the collar: stay bottom 12 mm above the 8 mm walls
WING_GRIP = (WING_START[0] + 0.010, WING_START[1])  # 10 mm toward the wing's +x end, which becomes the tip
WING_TCP_UP = 0.018  # TCP above the wing's bottom face: pad tips 5 mm above the table at the pick
WING_GRIP_Z = TOP + WING_TCP_UP
WING_PLACE = (AXIS_X, SEAT_CENTRE[1] - 0.010)  # the TCP 10 mm toward the tip from the seated centroid
WING_PLACE_Z = TOP + YOKE_TOP + 0.008 + WING_TCP_UP  # wing bottom 8 mm above both supports, 5.5 mm once the arm droops
WING_ENTRY = 0.042  # the pause above the yoke: wing bottom 10 mm above the 90 mm rail tops
WING_PLACE_YAW = -math.pi / 2  # the +x end swings to -y; +90 degrees would drive joint6 to its limit
PARK_RIGHT = (0.200, -0.300)
LEFT_WAIT = 11.5  # the left arm's carry starts after the right arm has parked


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
    """Right arm: the 50 mm stay into the collar, then park; 17.7 s. Left arm: waits over the wing until the right
    arm has parked, then picks the wing at yaw 0 (fingers across its 60 mm width, 10 mm toward its +x end) and sets
    it at place yaw -90 degrees, so the +x end becomes the tip, the root lands 4 mm from the stop between the rails
    and the wing drops about 5 mm onto the block and the stay at once; 27.7 s. Then the 3 s hold: 614 steps at 20 Hz.
    The release is commanded 8 mm up because the arm droops about 2.5 mm under the wing at this reach; held any
    lower, the still wing would read as seated before the fingers open."""
    right = _pick_place(
        (STAY_START[0], STAY_START[1], STAY_GRIP_Z), (COLLAR[0], COLLAR[1], STAY_PLACE_Z),
        width=2 * STAY_R, entry=STAY_ENTRY, travel_z=TRAVEL,
    )
    right.append(Waypoint(1.5, (PARK_RIGHT[0], PARK_RIGHT[1], TRAVEL), FINGER_OPEN, "park"))
    hover = (WING_GRIP[0], WING_GRIP[1], TRAVEL)
    left = [
        Waypoint(2.0, hover, FINGER_OPEN, "over the wing"),
        Waypoint(LEFT_WAIT - 2.0, hover, FINGER_OPEN, "wait for the stay"),
        *_pick_place(
            (WING_GRIP[0], WING_GRIP[1], WING_GRIP_Z), (WING_PLACE[0], WING_PLACE[1], WING_PLACE_Z),
            width=WING[1], entry=WING_ENTRY, travel_z=TRAVEL, yaw=0.0, place_yaw=WING_PLACE_YAW,
        ),
    ]
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def rz(a):
        return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])

    def rx(a):
        return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])

    def ry(a):
        return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])

    def quat(r):  # rotation matrix to x, y, z, w, stable for any angle
        t = r[0, 0] + r[1, 1] + r[2, 2]
        if t > 0:
            s = 2 * math.sqrt(1 + t)
            q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4]
        elif r[0, 0] >= r[1, 1] and r[0, 0] >= r[2, 2]:
            s = 2 * math.sqrt(1 + r[0, 0] - r[1, 1] - r[2, 2])
            q = [s / 4, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
        elif r[1, 1] >= r[2, 2]:
            s = 2 * math.sqrt(1 + r[1, 1] - r[0, 0] - r[2, 2])
            q = [(r[0, 1] + r[1, 0]) / s, s / 4, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
        else:
            s = 2 * math.sqrt(1 + r[2, 2] - r[0, 0] - r[1, 1])
            q = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, s / 4, (r[1, 0] - r[0, 1]) / s]
        return np.array(q)

    def body(size, centre, r, vel=(0.0, 0.0, 0.0)):
        half = np.abs(r) @ (np.asarray(size) / 2)
        pos = np.asarray(centre, float)
        return ObjectState(pos, quat(r), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def wing_by_root(root, yaw=-math.pi / 2, pitch=0.0, roll=0.0, flip=False, vel=(0.0, 0.0, 0.0)):
        """The wing posed by the bottom-edge midpoint of its -x end face (the root once turned -90 degrees), its yaw,
        a pitch that lowers the +x end, a roll about its long axis, optionally upside down."""
        r = rz(yaw) @ ry(pitch) @ rx(roll + (math.pi if flip else 0.0))
        down = r @ np.array([0.0, 0.0, 1.0 if flip else -1.0])
        centre = np.asarray(root, float) + r @ np.array([WING[0] / 2, 0.0, 0.0]) - down * WING[2] / 2
        return body(WING, centre, r, vel)

    def wing_flat(x, y, z_bottom=TOP, yaw=0.0):
        return body(WING, (x, y, z_bottom + WING[2] / 2), rz(yaw))

    def stay(x, y, height=STAY_H, bottom=TOP, tilt=0.0, vel=(0.0, 0.0, 0.0)):
        r = rx(tilt)
        uz = math.cos(tilt)
        centre = (x, y, bottom + height / 2 * abs(uz) + STAY_R * math.sqrt(max(0.0, 1 - uz * uz)))
        return body((2 * STAY_R, 2 * STAY_R, height), centre, r, vel)

    seat_root = (AXIS_X, STOP_FACE - 0.004, TOP + YOKE_TOP)

    def pivot_root(beta):
        """The root edge of a wing that pivoted about the block edge by ``beta``, no slip."""
        arm = STOP_FACE - 0.004 - EDGE_Y
        return (AXIS_X, EDGE_Y + arm * math.cos(beta), TOP + YOKE_TOP + arm * math.sin(beta))

    tipped = math.atan2(YOKE_TOP - COLLAR_H, EDGE_Y - (COLLAR[1] - POCKET / 2 - COLLAR_T))  # onto the far collar wall
    propped = math.atan2(YOKE_TOP - SHORT_H, EDGE_Y - (COLLAR[1] - STAY_R))  # onto the short stay's far rim
    stacked = math.atan2(SHORT_H, STOP_FACE - 0.004 - (COLLAR[1] + STAY_R))  # root on the block, tip end on a stack

    start = {"wing": wing_flat(*WING_START), "stay": stay(*STAY_START), "short_stay": stay(*SHORT_START, height=SHORT_H)}
    set_stay = stay(*COLLAR)
    seated = wing_by_root(seat_root)

    def scene(**kw):
        s = dict(start)
        s.update(kw)
        return s

    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(stay=set_stay, wing=seated), 1.0, True),
        "stay set, wing untouched": (scene(stay=set_stay), 0.25, False),
        "wing in the channel with no stay, tipped onto the collar wall": (scene(wing=wing_by_root(pivot_root(tipped), pitch=tipped)), 0.0, False),
        "wing propped on the short stay in the collar": (scene(short_stay=stay(*COLLAR, height=SHORT_H), wing=wing_by_root(pivot_root(propped), pitch=propped)), 0.0, False),
        "short stay in the collar, nothing else": (scene(short_stay=stay(*COLLAR, height=SHORT_H)), 0.0, False),
        "wing level on the tall stay standing outside the collar": (scene(stay=stay(AXIS_X, 0.010), wing=seated), 0.0, False),
        "both placed, wing yawed 6 degrees, tip 23 mm off the centreline": (scene(stay=set_stay, wing=wing_by_root(seat_root, yaw=-math.pi / 2 + math.radians(6))), 0.5, False),
        "stay set, wing level but its root 12 mm from the stop": (scene(stay=set_stay, wing=wing_by_root((AXIS_X, STOP_FACE - 0.012, TOP + YOKE_TOP))), 0.25, False),
        "stay set, short stay stacked on it, wing tip 90 mm high": (scene(stay=set_stay, short_stay=stay(*COLLAR, height=SHORT_H, bottom=TOP + STAY_H), wing=wing_by_root(seat_root, pitch=-stacked)), 0.5, False),
        "wing stood on its end between the rails, leaning on the stop": (scene(stay=set_stay, wing=wing_by_root((AXIS_X, STOP_FACE - 0.005, TOP + YOKE_TOP), yaw=math.pi / 2, pitch=-(math.pi / 2 - 0.1))), 0.25, False),
        "stay bottom exactly 3 mm up, on the limit, wing seated": (scene(stay=stay(*COLLAR, bottom=TOP + STAY_BOTTOM_TOL), wing=seated), 1.0, True),
        "wing across the yoke on the rail tops": (scene(wing=wing_flat(AXIS_X, RAIL_Y, z_bottom=TOP + YOKE_TOP + RAIL[2])), 0.0, False),
        "wing flat on the table under the seat": (scene(stay=set_stay, wing=wing_flat(AXIS_X, -0.200, yaw=-math.pi / 2)), 0.25, False),
        "wing held level in the seated pose, moving": (scene(stay=set_stay, wing=wing_by_root(seat_root, vel=(0, 0, 0.02))), 0.25, False),
        "done but the stay still moving": (scene(stay=stay(*COLLAR, vel=(0.02, 0, 0)), wing=seated), 0.0, False),
        "stay leaning 8 degrees in the collar": (scene(stay=stay(*COLLAR, tilt=math.radians(8))), 0.0, False),
        "stay held 10 mm above the collar": (scene(stay=stay(*COLLAR, bottom=TOP + 0.010)), 0.0, False),
        "stay lying on the table by the collar": (scene(stay=stay(AXIS_X + 0.080, COLLAR[1], tilt=math.pi / 2)), 0.0, False),
        "stay in a pocket corner, 5 mm off on both axes, wing seated": (scene(stay=stay(COLLAR[0] + 0.005, COLLAR[1] - 0.005), wing=seated), 1.0, True),
        "stay 7 mm off the collar centre": (scene(stay=stay(COLLAR[0] + 0.007, COLLAR[1]), wing=seated), 0.0, False),
        "wing seated upside down": (scene(stay=set_stay, wing=wing_by_root(seat_root, flip=True)), 1.0, True),
        "wing seated reversed end for end": (scene(stay=set_stay, wing=wing_by_root((AXIS_X, STOP_FACE - 0.004 - WING[0], TOP + YOKE_TOP), yaw=math.pi / 2)), 1.0, True),
        "wing on its edge in the channel": (scene(stay=set_stay, wing=wing_by_root((AXIS_X, STOP_FACE - 0.004, TOP + YOKE_TOP + WING[1] / 2), roll=math.pi / 2)), 0.25, False),
        "wing seated 4 mm off the centreline against a rail": (scene(stay=set_stay, wing=wing_by_root((AXIS_X + 0.004, STOP_FACE - 0.004, TOP + YOKE_TOP))), 1.0, True),
        "wing touching the stop with 1.5 mm of solver penetration": (scene(stay=set_stay, wing=wing_by_root((AXIS_X, STOP_FACE + 0.0015, TOP + YOKE_TOP))), 1.0, True),
        "wing tip 3 mm high": (scene(stay=set_stay, wing=wing_by_root(seat_root, pitch=-math.asin(0.003 / WING[0]))), 1.0, True),
        "wing tip 6 mm high": (scene(stay=set_stay, wing=wing_by_root(seat_root, pitch=-math.asin(0.006 / WING[0]))), 0.5, False),
        "wing in the channel with no stay, held level": (scene(wing=seated), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
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
