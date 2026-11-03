"""On the station table, in the left arm's lane, a 1 g bench analogue of spacecraft fit-up: avionics boxes go onto a
bracket plate whose keyed footprints run at odd angles, and a box set down across its rails does not seat. Three
identical orange boxes wait in a row. Three fixed rail bays, keyed at 0, 90 and 45 degrees, each take one box flat
between the rails against an end stop with 3 mm of clearance per side. The score counts bays, seated or merely
channelled, from the box centre and its quaternion for every horizontal test; only the bottom height uses the
bounds' lowest point, which yaw does not change.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/keyed-bay-fitup/keyed_bay_fitup.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Seat the three orange boxes in the three rail bays, one per bay, each flat between its rails with one end within 5 mm of the end stop." \\
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
BOX = (0.070, 0.035, 0.045)  # the free boxes, long axis along x at the start
BOX_MASS = 0.10
RAIL = (0.076, 0.008, 0.012)  # long axis along the rails
STOP = (0.008, 0.041, 0.012)  # thickness along the rails, spanning the gap between the rails
GAP = 0.041  # between the rails' inner faces: 3 mm per side around the 35 mm box
STOP_FACE = 0.037  # the stop's inner face, measured from the bay centre along the rails
RAIL_OFF = GAP / 2 + RAIL[1] / 2  # 24.5 mm, rail centre from the bay axis
STOP_OFF = STOP_FACE + STOP[0] / 2  # 41 mm, stop centre from the bay centre
FIX_Z = TOP + RAIL[2] / 2

BAYS = (  # name, centre, rail direction as a yaw; the stop closes the end the direction points to
    ("a", (0.180, 0.140), 0.0),
    ("b", (0.180, 0.270), math.pi / 2),
    ("c", (0.180, 0.400), math.pi / 4),
)
STARTS = ((0.330, 0.140), (0.330, 0.270), (0.330, 0.400))

GREY = (0.5, 0.5, 0.5)
ORANGE = (0.9, 0.45, 0.1)

SEAT_GAP = 0.005  # end face to stop face: seated
CHANNEL_GAP = 0.025  # channelled
PENETRATION = 0.002  # the solver may report the box this far into the stop; it still counts as touching
EPS = 1e-4  # 0.1 mm, so an exact 5.0 mm gap seats
AXIS_TOL = 0.004  # box centre to the bay axis
ANGLE_TOL = math.radians(5.0)  # tilt
YAW_TOL = math.radians(7.0)  # yaw to the rails: the rails hold a box to about 5 degrees seated and 7 channelled
BOTTOM_TOL = 0.004  # box bottom to the table
SEAT_ALONG = STOP_FACE - BOX[0] / 2 - SEAT_GAP / 2  # the seated centre along the rails, from the bay centre


def _bay_fixtures(name, centre, yaw):
    cx, cy = centre
    ux, uy = math.cos(yaw), math.sin(yaw)  # along the rails, toward the stop
    px, py = -uy, ux  # across the rails
    return [
        Box(f"bay_{name}_rail_p", pos=(cx + px * RAIL_OFF, cy + py * RAIL_OFF, FIX_Z), size=RAIL, color=GREY, yaw=yaw),
        Box(f"bay_{name}_rail_n", pos=(cx - px * RAIL_OFF, cy - py * RAIL_OFF, FIX_Z), size=RAIL, color=GREY, yaw=yaw),
        Box(f"bay_{name}_stop", pos=(cx + ux * STOP_OFF, cy + uy * STOP_OFF, FIX_Z), size=STOP, color=GREY, yaw=yaw),
    ]


BOXES = [Box(f"box_{k}", pos=(x, y, TOP + BOX[2] / 2), size=BOX, mass=BOX_MASS, color=ORANGE) for k, (x, y) in enumerate(STARTS)]
FIXTURES = [f for name, centre, yaw in BAYS for f in _bay_fixtures(name, centre, yaw)]
OBJECTS = [*BOXES, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _bay_level(state, centre, yaw) -> int:
    """2 seated, 1 channelled, 0 otherwise, judged from the box centre and its quaternion. A yawed box reports an
    enlarged axis-aligned bound, so low/high are used only for the bottom height, which yaw does not change."""
    if not state.resting():
        return 0
    rot = _rot(state.quat_xyzw)
    long_axis, up = rot[:, 0], rot[:, 2]
    if abs(up[2]) < math.cos(ANGLE_TOL) or abs(float(state.low[2]) - TOP) > BOTTOM_TOL:  # flat, either face up
        return 0
    ux, uy = math.cos(yaw), math.sin(yaw)
    hx, hy = float(long_axis[0]), float(long_axis[1])
    off = abs(math.atan2(hx * uy - hy * ux, hx * ux + hy * uy))
    if min(off, math.pi - off) > YAW_TOL:  # the long axis along the rails, either way round
        return 0
    dx, dy = float(state.pos[0]) - centre[0], float(state.pos[1]) - centre[1]
    along, across = dx * ux + dy * uy, dy * ux - dx * uy
    if abs(across) > AXIS_TOL:
        return 0
    gap = STOP_FACE - (along + BOX[0] / 2)  # the end face to the stop's inner face
    if -PENETRATION <= gap <= SEAT_GAP + EPS:
        return 2
    if SEAT_GAP + EPS < gap <= CHANNEL_GAP + EPS:
        return 1
    return 0  # further from the stop, or past it


def _levels(objects):
    """Each bay at its best box: a second box in a bay adds nothing, and a box can satisfy only one bay's end test."""
    return [max(_bay_level(objects[b.name], centre, yaw) for b in BOXES) for _, centre, yaw in BAYS]


def success(objects) -> bool:
    """All three bays seated, every box at rest; holds for 1 s."""
    return all(level == 2 for level in _levels(objects))


def distance(objects) -> float:
    """Metres still to go: for each bay, its nearest box centre to the bay's seated position."""
    total = 0.0
    for _, (cx, cy), yaw in BAYS:
        sx, sy = cx + math.cos(yaw) * SEAT_ALONG, cy + math.sin(yaw) * SEAT_ALONG
        total += min(math.hypot(float(objects[b.name].pos[0]) - sx, float(objects[b.name].pos[1]) - sy) for b in BOXES)
    return total


def score(objects) -> float:
    """Per bay, 1/3 seated (flat between the rails, along them, end face 0 to 5 mm from the stop), 1/6 channelled
    (the same but 5 to 25 mm from the stop), else 0; summed: 0, 1/6, 1/3, 1/2, 2/3, 5/6 or 1."""
    return sum(level / 6.0 for level in _levels(objects))


TRAVEL = TOP + 0.140
GRIP_Z = TOP + 0.035  # pad tips 22 mm up, on the 35 mm faces of the 45 mm tall box
PLACE_Z = TOP + 0.040  # box bottom 5 mm above the table, 7 mm down inside the channel; pad tips 15 mm above the rails
ENTRY = 0.017  # the pause above a bay: box bottom 10 mm above the 12 mm rails
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
    """Left arm only. Box 0 into bay A as it lies, box 1 into bay B turned +90 degrees, box 2 into bay C turned
    +45 degrees (a -45 turn would lay it across the rails); each a top-down pick, a carry at 14 cm and a drop 5 mm
    into the channel. 48.6 s of plan, then the 3 s hold: 1,032 steps at 20 Hz."""
    left = []
    for (sx, sy), (_, (cx, cy), yaw) in zip(STARTS, BAYS):
        left += _pick_place((sx, sy, GRIP_Z), (cx, cy, PLACE_Z), width=BOX[1], entry=ENTRY, travel_z=TRAVEL, yaw=0.0, place_yaw=yaw)
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw, tilt=0.0):  # yaw about the vertical, then tilt about the box's long axis
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        return np.array([cy * st, sy * st, sy * ct, cy * ct])

    def box(x, y, yaw=0.0, tilt=0.0, bottom=TOP, vel=(0.0, 0.0, 0.0)):
        q = quat(yaw, tilt)
        half = np.abs(_rot(q)) @ (np.asarray(BOX) / 2)
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def in_bay(k, gap=SEAT_GAP / 2, yaw=None, across=0.0, **kw):
        _, (cx, cy), bay_yaw = BAYS[k]
        along = STOP_FACE - BOX[0] / 2 - gap
        ux, uy = math.cos(bay_yaw), math.sin(bay_yaw)
        return box(cx + ux * along - uy * across, cy + uy * along + ux * across, bay_yaw if yaw is None else yaw, **kw)

    A, B, C = 0, 1, 2
    start = [box(x, y) for x, y in STARTS]

    def scene(*placed):
        states = list(start)
        for k, st in placed:
            states[k] = st
        return {b.name: st for b, st in zip(BOXES, states)}

    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene((0, in_bay(A)), (1, in_bay(B)), (2, in_bay(C))), 1.0, True),
        "a seated, b channelled, c untouched": (scene((0, in_bay(A)), (1, in_bay(B, gap=0.015))), 0.5, False),
        "a box on a rail top": (scene((0, box(0.180, 0.1645, bottom=TOP + RAIL[2]))), 0.0, False),
        "a box across bay a": (scene((0, in_bay(A, yaw=math.pi / 2))), 0.0, False),
        "a box in bay a tilted 8 degrees": (scene((0, in_bay(A, tilt=math.radians(8)))), 0.0, False),
        "a box held over bay a": (scene((0, in_bay(A, bottom=TOP + 0.05, vel=(0, 0, 0.02)))), 0.0, False),
        "an exact 5.0 mm gap seats": (scene((0, in_bay(A, gap=0.005))), 1 / 3, False),
        "a 5.3 mm gap is channelled": (scene((0, in_bay(A, gap=0.0053))), 1 / 6, False),
        "a 26 mm gap earns nothing": (scene((0, in_bay(A, gap=0.026))), 0.0, False),
        "touching the stop with 1 mm of solver penetration": (scene((0, in_bay(A, gap=-0.001))), 1 / 3, False),
        "a second box stacked on a seated one": (scene((0, in_bay(A)), (1, in_bay(A, bottom=TOP + BOX[2]))), 1 / 3, False),
        "bay c seated at plus 45": (scene((2, in_bay(C))), 1 / 3, False),
        "bay c box turned minus 45 lies across": (scene((2, in_bay(C, yaw=-math.pi / 4))), 0.0, False),
        "a box 5 mm off the bay axis": (scene((0, in_bay(A, across=0.005))), 0.0, False),
        "done but one box still moving": (scene((0, in_bay(A)), (1, in_bay(B)), (2, in_bay(C, vel=(0.02, 0, 0)))), 2 / 3, False),
        "a reversed box seats": (scene((0, in_bay(A, yaw=math.pi))), 1 / 3, False),
        "bay b accepts minus 90": (scene((1, in_bay(B, yaw=-math.pi / 2))), 1 / 3, False),
        "a box flat behind the stop": (scene((0, in_bay(A, gap=-0.085))), 0.0, False),
        "4 degrees of yaw and 4 degrees of tilt still seat": (scene((0, in_bay(A, yaw=math.radians(4), tilt=math.radians(4)))), 1 / 3, False),
        "6 degrees of yaw still seats": (scene((0, in_bay(A, yaw=math.radians(6)))), 1 / 3, False),
        "8 degrees of yaw does not": (scene((0, in_bay(A, yaw=math.radians(8)))), 0.0, False),
        "an upside-down box seats": (scene((0, in_bay(A, tilt=math.pi))), 1 / 3, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
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
