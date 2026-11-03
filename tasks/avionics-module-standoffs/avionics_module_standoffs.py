"""On the station table, one lane per arm, a 1 g bench analogue of a spacecraft avionics stack-up: a board is seated
on its chassis standoffs, long side along the deck, then its cover is fitted on top to the same chassis datum. Each
lane has a fixed dark grey deck with four light grey standoffs, 8 mm across and 20 mm tall, at the corners of a
60 x 40 mm rectangle. A gold module, 70 x 50 x 30 mm, starts on a white pad with its long side across the table; a
dark blue cover, 90 x 60 x 20 mm, starts on a second pad. The posts set the tolerance physically: within 9 to 13 mm
of the centre, depending on direction, the module rests level on them, further off it tips onto the deck, and turned
90 degrees it cannot rest on all four (centred within 1 mm it drops between them, otherwise it catches one pair and
tips about 25 degrees). The cover is judged from the deck centre, not from the module, so any error in the module
under it eats into the cover's 5 mm. The score reads each module's and cover's centre and quaternion, never the
axis-aligned bounds.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/avionics-module-standoffs/avionics_module_standoffs.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Seat each gold module on its four standoffs, then the blue cover flat on top. Centre both over the standoffs within 5 mm, long sides along the deck's long side." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75). Coordinates are for the
# left lane; the right lane mirrors y to -y.
DECK = (0.140, 0.100, 0.004)  # long side along x
DECK_TOP = TOP + DECK[2]  # 0.7515
POST_R = 0.004
POST_H = 0.020
POST_DX, POST_DY = 0.030, 0.020  # the posts stand at the corners of a 60 x 40 mm rectangle centred on the deck
POST_TOP = DECK_TOP + POST_H  # 0.7715
PAD_H = 0.002
PAD_TOP = TOP + PAD_H  # 0.7495
MODULE = (0.070, 0.050, 0.030)  # long side along local x
MODULE_MASS = 0.10
COVER = (0.090, 0.060, 0.020)  # long side along local x
COVER_MASS = 0.08
MODULE_PAD = (0.080, 0.080, PAD_H)
COVER_PAD = (0.100, 0.070, PAD_H)

PATTERN = (0.160, 0.300)  # the deck and post-pattern centre
MODULE_START = (0.360, 0.300)
COVER_START = (0.290, 0.420)
SIDES = (("left", 1.0), ("right", -1.0))

DARK_GREY = (0.20, 0.20, 0.22)
LIGHT_GREY = (0.80, 0.80, 0.80)
WHITE = (0.95, 0.95, 0.93)
GOLD = (0.85, 0.70, 0.25)
BLUE = (0.10, 0.15, 0.45)


def _side_objects(side, s):
    px, py = PATTERN[0], s * PATTERN[1]
    posts = [
        Cylinder(f"{side}_standoff_{k + 1}", pos=(px + dx, py + dy, DECK_TOP + POST_H / 2), radius=POST_R, height=POST_H, color=LIGHT_GREY)
        for k, (dx, dy) in enumerate(((-POST_DX, -POST_DY), (-POST_DX, POST_DY), (POST_DX, -POST_DY), (POST_DX, POST_DY)))
    ]
    fixed = [
        Box(f"{side}_deck", pos=(px, py, TOP + DECK[2] / 2), size=DECK, color=DARK_GREY),
        *posts,
        Box(f"{side}_module_pad", pos=(MODULE_START[0], s * MODULE_START[1], TOP + PAD_H / 2), size=MODULE_PAD, color=WHITE),
        Box(f"{side}_cover_pad", pos=(COVER_START[0], s * COVER_START[1], TOP + PAD_H / 2), size=COVER_PAD, color=WHITE),
    ]
    module = Box(f"{side}_module", pos=(MODULE_START[0], s * MODULE_START[1], PAD_TOP + MODULE[2] / 2), size=MODULE,
                 mass=MODULE_MASS, color=GOLD, yaw=math.pi / 2)  # long side across the table
    cover = Box(f"{side}_cover", pos=(COVER_START[0], s * COVER_START[1], PAD_TOP + COVER[2] / 2), size=COVER, mass=COVER_MASS, color=BLUE)
    return module, cover, fixed


_PARTS = [_side_objects(side, s) for side, s in SIDES]
MODULES = [m for m, _, _ in _PARTS]
COVERS = [c for _, c, _ in _PARTS]
FIXTURES = [f for _, _, fixed in _PARTS for f in fixed]
OBJECTS = [*MODULES, *COVERS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

EPS = 1e-4  # 0.1 mm or 0.1 mrad, so an exact limit counts
HEIGHT_TOL = 0.002  # bottom face to the post tops, cover bottom to the module top
SEAT_OFF = 0.005  # module centre to the pattern centre, seated
ROUGH_OFF = 0.010  # roughly seated
SEAT_YAW = math.radians(5.0)  # long axis to the deck's long side (x), either way round
ROUGH_YAW = math.radians(15.0)
TILT = math.radians(5.0)
COVER_OFF = 0.005  # cover centre to the pattern centre (the deck datum, not the module)
COVER_YAW = math.radians(5.0)
MODULE_SEATED, MODULE_ROUGH, COVER_ON = 0.30, 0.15, 0.20


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _pose(state, size):
    """Bottom and top heights, whether the large face is level (either way up), and the long axis's angle to x folded
    to 0..90 degrees, all from the centre and the quaternion."""
    r = _rot(state.quat_xyzw)
    half_z = sum(abs(float(r[2, i])) * size[i] / 2 for i in range(3))
    z = float(state.pos[2])
    level = abs(float(r[2, 2])) >= math.cos(TILT + EPS)
    off = abs(math.atan2(float(r[1, 0]), float(r[0, 0])))
    return z - half_z, z + half_z, level, min(off, math.pi - off)


def _module_level(state, centre) -> int:
    """2 seated, 1 roughly seated, 0 otherwise: at rest, level, bottom face on the post tops, then judged by its
    centre offset from the pattern centre and its long axis to x."""
    if not state.resting():
        return 0
    bottom, _, level, yaw = _pose(state, MODULE)
    if not level or abs(bottom - POST_TOP) > HEIGHT_TOL + EPS:
        return 0
    off = math.hypot(float(state.pos[0]) - centre[0], float(state.pos[1]) - centre[1])
    if off <= SEAT_OFF + EPS and yaw <= SEAT_YAW + EPS:
        return 2
    if off <= ROUGH_OFF + EPS and yaw <= ROUGH_YAW + EPS:
        return 1
    return 0


def _cover_on(cover, module, centre) -> bool:
    """At rest and level on the module's top face, its centre within 5 mm of the pattern centre (the deck datum, not
    the module, so a module's own offset eats into the cover's 5 mm), long axis within 5 degrees of x."""
    if not cover.resting():
        return False
    bottom, _, level, yaw = _pose(cover, COVER)
    _, module_top, _, _ = _pose(module, MODULE)
    off = math.hypot(float(cover.pos[0]) - centre[0], float(cover.pos[1]) - centre[1])
    return level and abs(bottom - module_top) <= HEIGHT_TOL + EPS and off <= COVER_OFF + EPS and yaw <= COVER_YAW + EPS


def _side_score(objects, s) -> float:
    """One lane at its best module: 0.50 seated and covered, 0.30 seated, 0.15 roughly seated, else 0. The modules
    are identical and so are the covers, so any module may seat on either pattern and any cover may cap it."""
    centre = (PATTERN[0], s * PATTERN[1])
    best = 0.0
    for m in MODULES:
        st = objects[m.name]
        level = _module_level(st, centre)
        if level == 2:
            covered = any(_cover_on(objects[c.name], st, centre) for c in COVERS)
            best = max(best, MODULE_SEATED + (COVER_ON if covered else 0.0))
        elif level == 1:
            best = max(best, MODULE_ROUGH)
    return best


def _sides(objects):
    return [_side_score(objects, s) for _, s in SIDES]


def success(objects) -> bool:
    """Both modules seated and covered, every part at rest; holds for 1 s."""
    return all(abs(v - (MODULE_SEATED + COVER_ON)) < 1e-9 for v in _sides(objects))


def distance(objects) -> float:
    """Metres still to go: per lane, the nearest module's centre to the pattern centre plus the nearest cover's
    centre to the cover's goal centre above it."""
    total = 0.0
    for _, s in SIDES:
        cx, cy = PATTERN[0], s * PATTERN[1]
        goal_cover_z = POST_TOP + MODULE[2] + COVER[2] / 2
        total += min(math.hypot(float(objects[m.name].pos[0]) - cx, float(objects[m.name].pos[1]) - cy) for m in MODULES)
        total += min(
            math.dist((float(objects[c.name].pos[0]), float(objects[c.name].pos[1]), float(objects[c.name].pos[2])), (cx, cy, goal_cover_z))
            for c in COVERS
        )
    return total


def score(objects) -> float:
    """The two lanes summed, each 0, 0.15, 0.30 or 0.50: module seated 0.30 (on the post tops within 2 mm, centre
    within 5 mm of the pattern centre, long axis within 5 degrees of x, tilt 5 degrees or less, at rest), or roughly
    seated 0.15 (5 to 10 mm off or 5 to 15 degrees turned), plus 0.20 for a cover level on a seated module's top, its
    centre within 5 mm of the pattern centre and its long axis within 5 degrees of x."""
    return sum(_sides(objects))


TRAVEL = TOP + 0.150
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part
MODULE_TIPS = 0.005  # pad tips 5 mm above the module's bottom face, so 25 mm of its 30 mm is between the pads
COVER_TIPS = 0.006  # pad tips 6 mm above the cover's bottom face, 14 mm of its 20 mm between the pads
TIP = 0.013  # pad tips below the TCP
MODULE_GRIP_Z = PAD_TOP + MODULE_TIPS + TIP  # 0.7675, 3 mm above the module's centre
COVER_GRIP_Z = PAD_TOP + COVER_TIPS + TIP  # 0.7685, 9 mm above the cover's centre
MODULE_GAP = 0.003  # module bottom above the post tops at release
COVER_GAP = 0.004  # cover bottom above the module's top at release
MODULE_PLACE_Z = POST_TOP + MODULE_GAP + MODULE_TIPS + TIP  # 0.7925: tips 8 mm above the post tops, outside them
COVER_PLACE_Z = POST_TOP + MODULE[2] + COVER_GAP + COVER_TIPS + TIP  # 0.8245: tips 10 mm above the module top
ENTRY = 0.012  # the pause above each place: the part's bottom 15 to 16 mm above what it lands on


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


def _lane(s):
    """One arm's plan in its own lane. The module is gripped across its 50 mm faces with the fingers closing along
    x, then turned 90 degrees during the carry so it lands long side along x; the cover is gripped across its 60 mm
    faces at yaw 0 and lowered square onto the module."""
    px, py = PATTERN[0], s * PATTERN[1]
    module = _pick_place((MODULE_START[0], s * MODULE_START[1], MODULE_GRIP_Z), (px, py, MODULE_PLACE_Z), width=MODULE[1],
                         entry=ENTRY, travel_z=TRAVEL, yaw=s * math.pi / 2, place_yaw=0.0)
    cover = _pick_place((COVER_START[0], s * COVER_START[1], COVER_GRIP_Z), (px, py, COVER_PLACE_Z), width=COVER[1],
                        entry=ENTRY, travel_z=TRAVEL, yaw=0.0)
    return module + cover


def demo():
    """Both arms at once, each in its own lane, mirror images of each other: module onto the standoffs, turned 90
    degrees in the carry, then the cover onto the module. 32.4 s of plan, then the 3 s hold: 708 steps at 20 Hz."""
    return {side: _lane(s) for side, s in SIDES}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw=0.0, tilt=0.0):  # yaw about the vertical, then tilt about the part's long axis
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        return np.array([cy * st, sy * st, sy * ct, cy * ct])

    def part(size, x, y, bottom, yaw=0.0, tilt=0.0, vel=(0.0, 0.0, 0.0), spin=(0.0, 0.0, 0.0)):
        q = quat(yaw, tilt)
        half = np.abs(_rot(q)) @ (np.asarray(size) / 2)
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.asarray(spin, float), pos - half, pos + half)

    def module(s, dx=0.0, dy=0.0, bottom=POST_TOP, **kw):
        return part(MODULE, PATTERN[0] + dx, s * PATTERN[1] + dy, bottom, **kw)

    def cover(s, dx=0.0, dy=0.0, bottom=POST_TOP + MODULE[2], **kw):
        return part(COVER, PATTERN[0] + dx, s * PATTERN[1] + dy, bottom, **kw)

    L, R = 1.0, -1.0
    start = {
        **{m.name: part(MODULE, MODULE_START[0], s * MODULE_START[1], PAD_TOP, yaw=math.pi / 2) for m, (_, s) in zip(MODULES, SIDES)},
        **{c.name: part(COVER, COVER_START[0], s * COVER_START[1], PAD_TOP) for c, (_, s) in zip(COVERS, SIDES)},
    }

    def scene(lm=None, lc=None, rm=None, rc=None):
        st = dict(start)
        for name, v in (("left_module", lm), ("left_cover", lc), ("right_module", rm), ("right_cover", rc)):
            if v is not None:
                st[name] = v
        return st

    done = dict(lm=module(L), lc=cover(L), rm=module(R), rc=cover(R))

    def but(**kw):
        return scene(**{**done, **kw})

    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(**done), 1.0, True),
        "left seated and covered, right untouched": (scene(lm=module(L), lc=cover(L)), 0.5, False),
        "left module seated, nothing else": (scene(lm=module(L)), 0.3, False),
        "both modules seated, no covers": (scene(lm=module(L), rm=module(R)), 0.6, False),
        "done but the right cover left on its pad": (but(rc=part(COVER, COVER_START[0], -COVER_START[1], PAD_TOP)), 0.8, False),
        "an exact 5.0 mm offset seats": (but(lm=module(L, dx=0.005), lc=cover(L, dx=0.005)), 1.0, True),
        "a 5.3 mm offset is roughly seated, its cover earns nothing": (but(lm=module(L, dy=0.0053), lc=cover(L, dy=0.0053)), 0.65, False),
        "a 7 mm diagonal offset is roughly seated": (but(lm=module(L, dx=0.005, dy=0.005), lc=cover(L, dx=0.005, dy=0.005)), 0.65, False),
        "an exact 10.0 mm offset is roughly seated": (but(lm=module(L, dx=-0.010), lc=None), 0.65, False),
        "an 11 mm offset at post height earns nothing": (but(lm=module(L, dx=0.011)), 0.5, False),
        "4 degrees of yaw seats": (but(lm=module(L, yaw=math.radians(4)), lc=cover(L, yaw=math.radians(4))), 1.0, True),
        "10 degrees of yaw is roughly seated": (but(lm=module(L, yaw=math.radians(10))), 0.65, False),
        "16 degrees of yaw earns nothing": (but(lm=module(L, yaw=math.radians(16))), 0.5, False),
        "turned 90 degrees, dropped between the posts onto the deck": (but(lm=module(L, yaw=math.pi / 2, bottom=DECK_TOP)), 0.5, False),
        "turned 90 degrees at post height": (but(lm=module(L, yaw=math.pi / 2)), 0.5, False),
        "a module reversed end for end seats": (but(lm=module(L, yaw=math.pi), lc=cover(L, yaw=math.pi)), 1.0, True),
        "a module upside down seats": (but(rm=module(R, tilt=math.pi), rc=cover(R, tilt=math.pi)), 1.0, True),
        "a module tilted 4 degrees on the posts seats": (but(lm=module(L, tilt=math.radians(4)), lc=None), 0.8, False),
        "a module tilted exactly 5.0 degrees on the posts seats": (but(lm=module(L, tilt=math.radians(5.0)), lc=None), 0.8, False),
        "a module tilted 5.1 degrees earns nothing": (but(lm=module(L, tilt=math.radians(5.1)), lc=None), 0.5, False),
        "a module turning at 0.6 rad/s on the posts earns nothing": (but(lm=module(L, spin=(0.0, 0.0, 0.6))), 0.5, False),
        "a module tipped 30 degrees onto the deck": (but(lm=module(L, dx=0.020, tilt=math.radians(30), bottom=DECK_TOP)), 0.5, False),
        "a module flat on the deck, centred": (but(lm=module(L, bottom=DECK_TOP)), 0.5, False),
        "a module 1.9 mm above the post tops still seats": (but(lm=module(L, bottom=POST_TOP + 0.0019), lc=cover(L, bottom=POST_TOP + 0.0019 + MODULE[2])), 1.0, True),
        "a module still held 10 mm above the posts, moving": (but(lm=module(L, bottom=POST_TOP + 0.010, vel=(0, 0, -0.02)), lc=None), 0.5, False),
        "a module on its side between the posts": (but(lm=part(MODULE, PATTERN[0], PATTERN[1], DECK_TOP, tilt=math.pi / 2), lc=None), 0.5, False),
        "a module still sliding on the posts": (but(lm=module(L, vel=(0.02, 0, 0))), 0.5, False),
        "the cover set on the posts, the module on top of it": (but(lc=cover(L, bottom=POST_TOP), lm=module(L, bottom=POST_TOP + COVER[2])), 0.5, False),
        "the cover set on the posts, the module still on its pad": (scene(lc=cover(L, bottom=POST_TOP), rm=module(R), rc=cover(R)), 0.5, False),
        "the cover 6 mm off a centred module": (but(lc=cover(L, dx=0.006)), 0.8, False),
        "a module 4 mm off with its cover centred on it": (but(lm=module(L, dx=0.004), lc=cover(L, dx=0.004)), 1.0, True),
        "a module 4 mm off, its cover 2 mm further out, 6 mm off the deck centre": (but(lm=module(L, dx=0.004), lc=cover(L, dx=0.006)), 0.8, False),
        "a module 4 mm off, its cover centred on the deck, 4 mm off the module": (but(lm=module(L, dx=0.004), lc=cover(L)), 1.0, True),
        "a module 4 mm off one way, its cover 2 mm off the other, 6 mm off the module": (but(rm=module(R, dy=0.004), rc=cover(R, dy=-0.002)), 1.0, True),
        "a cover tilted exactly 5.0 degrees counts": (but(lc=cover(L, tilt=math.radians(5.0))), 1.0, True),
        "a cover tilted 5.1 degrees is not credited": (but(lc=cover(L, tilt=math.radians(5.1))), 0.8, False),
        "the cover turned 90 degrees on the module": (but(lc=cover(L, yaw=math.pi / 2)), 0.8, False),
        "the cover turned 6 degrees": (but(lc=cover(L, yaw=math.radians(6))), 0.8, False),
        "the cover tilted 8 degrees": (but(lc=cover(L, tilt=math.radians(8))), 0.8, False),
        "the cover on the table beside the deck": (but(lc=part(COVER, PATTERN[0], PATTERN[1] + 0.09, TOP)), 0.8, False),
        "a module resting 2.5 mm above the post tops": (but(lm=module(L, bottom=POST_TOP + 0.0025), lc=None), 0.5, False),
        "a module set back on its start pad the right way round": (but(lm=part(MODULE, MODULE_START[0], MODULE_START[1], PAD_TOP), lc=None), 0.5, False),
        "the cover still moving on the module": (but(rc=cover(R, vel=(0, 0.02, 0))), 0.8, False),
        "the cover centred on a module turned 10 degrees": (but(lm=module(L, yaw=math.radians(10)), lc=cover(L)), 0.65, False),
        "the modules swapped between the lanes": (scene(lm=module(R), rm=module(L), lc=cover(L), rc=cover(R)), 1.0, True),
        "both modules stacked on the left pattern": (scene(lm=module(L), rm=module(L, bottom=POST_TOP + MODULE[2])), 0.3, False),
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
