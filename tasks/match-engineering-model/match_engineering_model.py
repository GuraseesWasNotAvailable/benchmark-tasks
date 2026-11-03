"""On the station table, a 1 g bench analogue of building an assembly to match a physical reference beside it. A
three-block model is glued to a white plate on the midline: a blue base block, an orange cube on top of one end and
a green side block standing beside the other end at a right angle. An identical white plate in the left arm's lane
is empty, and the three loose twins of the model's blocks wait beside it. Each plate carries a red datum strip along
one edge, and the empty plate is turned 90 degrees to the model's plate, so the copy is built relative to the datum,
not to the room. The cube can go on only after the base. The score checks each loose block's centre and long axis in
the empty plate's own frame, from its centre and quaternion; only the bottom height uses the bounds.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/match-engineering-model/match_engineering_model.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Use the three loose blocks to build a copy of the glued model on the empty white plate. Place each block relative to this plate's red edge as its twin sits relative to the model plate's red edge, within 8 mm and 10 degrees." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
# Plate frame: origin at the centre of a plate's white top, u toward its red strip, v 90 degrees left of u.
PLATE = (0.140, 0.140, 0.010)  # white top, along u, along v, thickness
STRIP = (0.015, 0.140, 0.010)  # red datum strip, flush along the plate's +u edge
PLATE_TOP = TOP + PLATE[2]
MODEL_PLATE = ((0.260, -0.040), 0.0)  # centre, direction of u as a yaw: the strip on the far edge
BUILD_PLATE = ((0.240, 0.190), math.pi / 2)  # the strip on the outboard (left) edge

BASE = (0.100, 0.040, 0.030)  # long, wide, tall
CUBE = (0.040, 0.040, 0.040)
SIDE = (0.060, 0.030, 0.040)
BASE_TOP = PLATE_TOP + BASE[2]

# The model in the plate frame: centre (u, v), the block's long axis as an angle from u, its yaw symmetry, and what
# it stands on. The base runs along v; the cube covers the +v end of the base; the side block runs along u toward the
# strip, its outer face flush with the base's -v end, 10 mm clear of the base's +u face.
MODEL = {
    "base": (BASE, (-0.035, 0.000), math.pi / 2, math.pi, PLATE_TOP),
    "side": (SIDE, (0.025, -0.035), 0.0, math.pi, PLATE_TOP),
    "cube": (CUBE, (-0.035, 0.030), 0.0, math.pi / 2, BASE_TOP),
}
STARTS = {  # loose blocks on the table, outboard of the empty plate: centre, yaw
    "cube": ((0.140, 0.400), 0.0),
    "side": ((0.225, 0.415), 0.0),
    "base": ((0.330, 0.395), math.pi / 2),
}

WHITE = (0.92, 0.92, 0.90)
RED = (0.80, 0.12, 0.10)
BLUE = (0.15, 0.35, 0.80)
ORANGE = (0.90, 0.45, 0.10)
GREEN = (0.20, 0.60, 0.30)
COLOURS = {"base": BLUE, "cube": ORANGE, "side": GREEN}
MASSES = {"base": 0.070, "cube": 0.040, "side": 0.045}


def _to_world(plate, u, v):
    (cx, cy), yaw = plate
    c, s = math.cos(yaw), math.sin(yaw)
    return cx + c * u - s * v, cy + s * u + c * v


def _plate_fixtures(prefix, plate):
    (cx, cy), yaw = plate
    sx, sy = _to_world(plate, PLATE[0] / 2 + STRIP[0] / 2, 0.0)
    return [
        Box(f"{prefix}_plate", pos=(cx, cy, TOP + PLATE[2] / 2), size=PLATE, color=WHITE, yaw=yaw),
        Box(f"{prefix}_datum", pos=(sx, sy, TOP + STRIP[2] / 2), size=STRIP, color=RED, yaw=yaw),
    ]


def _model_block(key):
    size, (u, v), axis, _, support = MODEL[key]
    x, y = _to_world(MODEL_PLATE, u, v)
    return Box(f"model_{key}", pos=(x, y, support + size[2] / 2), size=size, color=COLOURS[key], yaw=MODEL_PLATE[1] + axis)


def _loose_block(key):
    size = MODEL[key][0]
    (x, y), yaw = STARTS[key]
    return Box(f"{key}_block", pos=(x, y, TOP + size[2] / 2), size=size, mass=MASSES[key], color=COLOURS[key], yaw=yaw)


LOOSE = {key: _loose_block(key) for key in ("base", "side", "cube")}
OBJECTS = [
    *LOOSE.values(),
    *_plate_fixtures("model", MODEL_PLATE),
    *_plate_fixtures("build", BUILD_PLATE),
    *(_model_block(key) for key in ("base", "side", "cube")),
]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

POS_TOL = 0.008  # block centre to its place in the empty plate's frame
YAW_TOL = math.radians(10.0)  # long axis to its place, either way round (the cube every 90 degrees)
TILT_TOL = math.radians(5.0)  # the block's height axis to vertical, either way up
BOTTOM_TOL = 0.004  # lowest point to the surface it should stand on
EPS = 1e-4  # 0.1 mm, so an exact 8.0 mm offset counts
YAW_EPS = 1e-6  # rad, so an exact 10.0 degree turn counts every way round


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _target(key):
    """World centre (x, y) and long-axis yaw of a block's place on the empty plate."""
    _, (u, v), axis, _, _ = MODEL[key]
    return _to_world(BUILD_PLATE, u, v), BUILD_PLATE[1] + axis


def _placed(key, state) -> bool:
    """At rest, flat with its height axis vertical (either way up), its lowest point within 4 mm of the plate top
    (the base and the side block) or of the base's top (the cube), its centre within 8 mm of its place in the empty
    plate's frame and its long axis within 10 degrees of its place, either way round (the cube every 90 degrees).
    Judged from the block's centre and quaternion; only the bottom height uses the bounds."""
    _, _, _, period, support = MODEL[key]
    if not state.resting():
        return False
    rot = _rot(state.quat_xyzw)
    up = int(np.argmax(np.abs(rot[2, :]))) if key == "cube" else 2  # every face of the cube is the same
    if abs(float(rot[2, up])) < math.cos(TILT_TOL) or abs(float(state.low[2]) - support) > BOTTOM_TOL:
        return False
    (tx, ty), tyaw = _target(key)
    if math.hypot(float(state.pos[0]) - tx, float(state.pos[1]) - ty) > POS_TOL + EPS:
        return False
    side = 0 if up != 0 else 1  # a horizontal body axis: the long axis for the base and the side block
    yaw = math.atan2(float(rot[1, side]), float(rot[0, side]))
    off = (yaw - tyaw) % period
    return min(off, period - off) <= YAW_TOL + YAW_EPS


def _matches(objects):
    return {key: _placed(key, objects[f"{key}_block"]) for key in ("base", "side", "cube")}


def success(objects) -> bool:
    """All three blocks in place on the empty plate, at rest; holds for 1 s."""
    return all(_matches(objects).values())


def distance(objects) -> float:
    """Metres still to go: the sum over the three loose blocks of their centre's distance to its place."""
    total = 0.0
    for key in ("base", "side", "cube"):
        (tx, ty), _ = _target(key)
        s = objects[f"{key}_block"]
        total += math.hypot(float(s.pos[0]) - tx, float(s.pos[1]) - ty)
    return total


def score(objects) -> float:
    """1/3 for each block in place: the base, the side block, and the cube, which counts only on a base that is in
    place, since its place is on the base and a side block lying on a long face is also 30 mm tall: 0, 1/3, 2/3 or 1."""
    m = _matches(objects)
    return (m["base"] + m["side"] + (m["base"] and m["cube"])) / 3.0


TRAVEL = TOP + 0.150
DROP = 0.005  # release with the block's bottom 5 mm above its support
LOW_GRIP = 0.024  # TCP above the bottom of the 30 mm base: pad tips 11 mm up, 19 mm of pad on the block
TALL_GRIP = 0.028  # TCP above the bottom of the 40 mm cube and side block: pad tips 15 mm up, 25 mm of pad
GRIP_UP = {"base": LOW_GRIP, "side": TALL_GRIP, "cube": TALL_GRIP}
WIDTH = {"base": BASE[1], "side": SIDE[1], "cube": CUBE[1]}
ENTRY = {"base": 0.015, "side": 0.035, "cube": 0.015}  # the pause above the place: the block 10 mm or more above everything it passes
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part
# Gripper yaw at the pick and at the place. At yaw 0 the fingers close along world y, across a block lying along x,
# so the yaw equals the block's long-axis yaw modulo its symmetry; each is chosen to keep the wrist joint well inside
# its +-120 degree range (the joint runs at about the base joint's angle minus the yaw).
DEMO_YAWS = {"base": (math.pi / 2, 0.0), "side": (0.0, -math.pi / 2), "cube": (0.0, 0.0)}


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
    """Left arm only. The base first, since the cube stands on it, turned from lying across the table to pointing
    forward; then the side block, turned from pointing forward to lying across the table, its near end 10 mm outboard
    of the base's long side and its far face flush with the base's far end; then the cube onto the base's near end. Each a top-down pick, a carry at 15 cm and a 5 mm drop. 48.6 s
    of plan, then the 3 s hold: 1,032 steps at 20 Hz."""
    left = []
    for key in ("base", "side", "cube"):
        (sx, sy), _ = STARTS[key]
        (tx, ty), _ = _target(key)
        support = MODEL[key][4]
        pick_yaw, put_yaw = DEMO_YAWS[key]
        left += _pick_place(
            (sx, sy, TOP + GRIP_UP[key]),
            (tx, ty, support + DROP + GRIP_UP[key]),
            width=WIDTH[key],
            entry=ENTRY[key],
            travel_z=TRAVEL,
            yaw=pick_yaw,
            place_yaw=put_yaw,
        )
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw=0.0, roll=0.0, pitch=0.0):  # yaw about z, then roll about the body's x, then pitch about its y
        r = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        r = r @ np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        r = r @ np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        w = math.sqrt(max(1e-12, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
        if w < 1e-3:  # a half turn: take the quaternion from the largest diagonal term instead
            k = int(np.argmax(np.diag(r)))
            i, j = (k + 1) % 3, (k + 2) % 3
            q = np.zeros(4)
            q[k] = math.sqrt(max(1e-12, 1 + r[k, k] - r[i, i] - r[j, j])) / 2
            q[i] = (r[i, k] + r[k, i]) / (4 * q[k])
            q[j] = (r[j, k] + r[k, j]) / (4 * q[k])
            q[3] = (r[j, i] - r[i, j]) / (4 * q[k])
            return q
        return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])

    def block(key, x, y, bottom, yaw=0.0, roll=0.0, pitch=0.0, vel=(0.0, 0.0, 0.0)):
        q = quat(yaw, roll, pitch)
        half = np.abs(_rot(q)) @ (np.asarray(MODEL[key][0]) / 2)
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def start(key):
        (x, y), yaw = STARTS[key]
        return block(key, x, y, TOP, yaw)

    def placed(key, du=0.0, dv=0.0, dyaw=0.0, bottom=None, **kw):
        """The block at its place on the empty plate, shifted (du, dv) in that plate's frame and turned dyaw."""
        _, (u, v), axis, _, support = MODEL[key]
        x, y = _to_world(BUILD_PLATE, u + du, v + dv)
        return block(key, x, y, support if bottom is None else bottom, BUILD_PLATE[1] + axis + dyaw, **kw)

    def on_model_plate_layout(key):
        """The block where its twin would be if the model were copied with the room's orientation, not the datum's:
        the model's world offsets from its plate centre, moved to the empty plate's centre."""
        _, (u, v), axis, _, support = MODEL[key]
        mx, my = _to_world(MODEL_PLATE, u, v)
        x, y = mx - MODEL_PLATE[0][0] + BUILD_PLATE[0][0], my - MODEL_PLATE[0][1] + BUILD_PLATE[0][1]
        return block(key, x, y, support, MODEL_PLATE[1] + axis)

    def scene(**placed_states):
        states = {key: start(key) for key in ("base", "side", "cube")}
        states.update(placed_states)
        return {f"{key}_block": st for key, st in states.items()}

    done = dict(base=placed("base"), side=placed("side"), cube=placed("cube"))
    third = 1 / 3
    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(**done), 1.0, True),
        "base only": (scene(base=placed("base")), third, False),
        "base and side block": (scene(base=placed("base"), side=placed("side")), 2 * third, False),
        "base and cube": (scene(base=placed("base"), cube=placed("cube")), 2 * third, False),
        "side block in place, base still on the table": (scene(side=placed("side")), third, False),
        "copied in the room's orientation, not the datum's": (
            scene(base=on_model_plate_layout("base"), side=on_model_plate_layout("side"), cube=on_model_plate_layout("cube")),
            0.0,
            False,
        ),
        "mirrored end for end along the red edge: only the base counts": (
            scene(base=placed("base"), side=placed("side", dv=0.070), cube=placed("cube", dv=-0.060)),
            third,
            False,
        ),
        "mirrored toward and away from the red edge": (
            scene(base=placed("base", du=0.070), side=placed("side", du=-0.050), cube=placed("cube", du=0.070)),
            0.0,
            False,
        ),
        "built with the datum reversed": (
            scene(base=placed("base", du=0.070), side=placed("side", du=-0.050, dv=0.070), cube=placed("cube", du=0.070, dv=-0.060)),
            0.0,
            False,
        ),
        "base 7 mm off along v": (scene(**{**done, "base": placed("base", dv=0.007)}), 1.0, True),
        "base exactly 8.0 mm off along u": (scene(**{**done, "base": placed("base", du=0.008)}), 1.0, True),
        "base 9 mm off: only the side block counts": (scene(**{**done, "base": placed("base", du=0.009)}), third, False),
        "base turned 9 degrees": (scene(**{**done, "base": placed("base", dyaw=math.radians(9))}), 1.0, True),
        "base turned 11 degrees: only the side block counts": (
            scene(**{**done, "base": placed("base", dyaw=math.radians(11))}),
            third,
            False,
        ),
        "base end for end": (scene(**{**done, "base": placed("base", dyaw=math.pi)}), 1.0, True),
        "base upside down": (scene(**{**done, "base": placed("base", roll=math.pi)}), 1.0, True),
        "base on its long side, 40 mm tall": (scene(base=placed("base", roll=math.pi / 2)), 0.0, False),
        "base tilted 6 degrees": (scene(base=placed("base", pitch=math.radians(6))), 0.0, False),
        "base held 20 mm above the plate, moving": (scene(base=placed("base", bottom=PLATE_TOP + 0.020, vel=(0, 0, -0.03))), 0.0, False),
        "base resting on the side block": (scene(side=placed("side"), base=placed("base", bottom=PLATE_TOP + SIDE[2])), third, False),
        "side block 8 mm toward the base, 2 mm short of it": (scene(**{**done, "side": placed("side", du=-0.008)}), 1.0, True),
        "side block pushed against the base: no credit for it": (scene(**{**done, "side": placed("side", du=-0.010)}), 2 * third, False),
        "side block turned 8 degrees": (scene(**{**done, "side": placed("side", dyaw=math.radians(8))}), 1.0, True),
        "side block turned exactly +10.0 degrees": (scene(**{**done, "side": placed("side", dyaw=math.radians(10.0))}), 1.0, True),
        "side block turned exactly -10.0 degrees": (scene(**{**done, "side": placed("side", dyaw=math.radians(-10.0))}), 1.0, True),
        "side block along v instead of u": (scene(**{**done, "side": placed("side", dyaw=math.pi / 2)}), 2 * third, False),
        "side block lying on a long face, 30 mm tall": (scene(**{**done, "side": placed("side", roll=math.pi / 2)}), 2 * third, False),
        "side block standing on end, 60 mm tall": (scene(**{**done, "side": placed("side", pitch=math.pi / 2)}), 2 * third, False),
        "side block on top of the base instead": (
            scene(base=placed("base"), side=block("side", *_target("base")[0], BASE_TOP, BUILD_PLATE[1])),
            third,
            False,
        ),
        "cube turned 90 degrees": (scene(**{**done, "cube": placed("cube", dyaw=math.pi / 2)}), 1.0, True),
        "cube turned exactly +10.0 degrees": (scene(**{**done, "cube": placed("cube", dyaw=math.radians(10.0))}), 1.0, True),
        "cube turned exactly -10.0 degrees": (scene(**{**done, "cube": placed("cube", dyaw=math.radians(-10.0))}), 1.0, True),
        "cube 5 mm off, on a base 9 mm off: no credit for either": (
            scene(**{**done, "base": placed("base", du=0.009), "cube": placed("cube", du=0.005)}),
            third,
            False,
        ),
        "cube turned 45 degrees": (scene(**{**done, "cube": placed("cube", dyaw=math.pi / 4)}), 2 * third, False),
        "cube rolled onto another face": (scene(**{**done, "cube": placed("cube", roll=math.pi / 2)}), 1.0, True),
        "cube on the plate beside the base": (scene(**{**done, "cube": placed("cube", du=0.045, bottom=PLATE_TOP)}), 2 * third, False),
        "cube on the base's far end": (scene(**{**done, "cube": placed("cube", dv=-0.060)}), 2 * third, False),
        "cube on the side block": (scene(**{**done, "cube": block("cube", *_target("side")[0], PLATE_TOP + SIDE[2])}), 2 * third, False),
        "done but the cube still moving": (scene(**{**done, "cube": placed("cube", vel=(0.02, 0, 0))}), 2 * third, False),
        "done but the base still moving": (scene(**{**done, "base": placed("base", vel=(0.02, 0, 0))}), third, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert distance(scene(**done)) < 1e-9 and distance(scene()) > 0.3
    for key, (pick_yaw, put_yaw) in DEMO_YAWS.items():  # the demo's grips line up with each block's start and place
        period = MODEL[key][3]
        for a, b in ((pick_yaw, STARTS[key][1]), (put_yaw, _target(key)[1])):
            off = (a - b) % period
            assert min(off, period - off) < 1e-9, (key, a, b)
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through the
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
