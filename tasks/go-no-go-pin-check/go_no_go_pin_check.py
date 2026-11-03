"""On the station table, in the left arm's lane, a receiving-inspection job done in 1 g exactly as on a spacecraft
integration floor. Six grey pins of one finish stand on marks: three of 20 mm diameter and three of 28 mm, all
60 mm tall. A fixed grey gauge plate carries three square pockets 24 x 24 mm and 25 mm deep: a 20 mm pin drops in,
a 28 mm pin cannot enter and stands on the walls. A fixed red reject pad, 120 x 80 mm, lies beyond the plate. Pins
that fit go in the pockets, pins that do not go on the pad; the score nets every misrouted pin.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/go-no-go-pin-check/go_no_go_pin_check.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Gauge the six grey pins. Seat every pin that drops fully into a gauge pocket and put every pin that does not fit on the red reject pad." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
PIN_H = 0.060
GO_R = 0.010  # 20 mm pins fit the 24 mm pockets with 2 mm per side
NOGO_R = 0.014  # 28 mm pins cannot enter a 24 mm pocket in any orientation
POCKET = 0.024
POCKET_DEPTH = 0.025
POCKET_XS = (0.132, 0.176, 0.220)
POCKET_Y = 0.200
PAD = (0.200, 0.330)
PAD_SIZE = (0.120, 0.080, 0.005)
PLATE_X = (0.100, 0.252)  # the plate's footprint along x
PLATE_Y = (0.183, 0.217)

GREY = (0.75, 0.75, 0.75)
GAUGE = (0.25, 0.25, 0.27)
RED = (0.85, 0.12, 0.12)

GO_START = ((0.310, 0.170), (0.310, 0.370), (0.380, 0.270))
NOGO_START = ((0.310, 0.270), (0.380, 0.170), (0.380, 0.370))

_wall_z = TOP + POCKET_DEPTH / 2
PLATE = [
    Box("gauge_side_a", pos=(0.176, 0.1855, _wall_z), size=(0.152, 0.005, POCKET_DEPTH), color=GAUGE),
    Box("gauge_side_b", pos=(0.176, 0.2145, _wall_z), size=(0.152, 0.005, POCKET_DEPTH), color=GAUGE),
    *[Box(f"gauge_cross_{k}", pos=(x, POCKET_Y, _wall_z), size=(0.020, POCKET, POCKET_DEPTH), color=GAUGE) for k, x in enumerate((0.110, 0.154, 0.198, 0.242))],
]
PAD_BOX = Box("reject_pad", pos=(PAD[0], PAD[1], TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=RED)
GO_PINS = [Cylinder(f"pin_go_{k}", pos=(x, y, TOP + PIN_H / 2), radius=GO_R, height=PIN_H, mass=0.03, color=GREY) for k, (x, y) in enumerate(GO_START)]
NOGO_PINS = [Cylinder(f"pin_nogo_{k}", pos=(x, y, TOP + PIN_H / 2), radius=NOGO_R, height=PIN_H, mass=0.05, color=GREY) for k, (x, y) in enumerate(NOGO_START)]
OBJECTS = [*GO_PINS, *NOGO_PINS, *PLATE, PAD_BOX]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

BOTTOM_TOL = 0.003
PAD_TOP = TOP + PAD_SIZE[2]


def _bottom(state, r) -> float:
    """The pin's lowest point from its centre and axis. The axis-aligned bounds of a lying cylinder come from its
    box corners and read up to r(sqrt 2 - 1) too low, so they are not used."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    uz = 1.0 - 2.0 * (x * x + y * y)  # world z of the pin axis
    return float(state.pos[2]) - (PIN_H / 2 * abs(uz) + r * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _in_pocket(state, r=GO_R) -> bool:
    """A pin standing on the pocket floor with its centre inside a 24 x 24 mm opening; only a 20 mm pin can."""
    x, y = float(state.pos[0]), float(state.pos[1])
    on_floor = abs(_bottom(state, r) - TOP) < BOTTOM_TOL
    return bool(on_floor and state.resting() and abs(y - POCKET_Y) < POCKET / 2 and any(abs(x - cx) < POCKET / 2 for cx in POCKET_XS))


def _on_pad(state, r) -> bool:
    """Resting on the red pad, standing or lying, with its centre inside the pad's footprint."""
    x, y = float(state.pos[0]), float(state.pos[1])
    on_pad = abs(_bottom(state, r) - PAD_TOP) < BOTTOM_TOL + 0.001
    return bool(on_pad and state.resting() and abs(x - PAD[0]) < PAD_SIZE[0] / 2 and abs(y - PAD[1]) < PAD_SIZE[1] / 2)


def _on_plate(state, r=NOGO_R) -> bool:
    """Resting on the gauge plate's walls or leaning on them: centre over the plate (grown by 1 cm), bottom off the table."""
    x, y = float(state.pos[0]), float(state.pos[1])
    over = PLATE_X[0] - 0.010 < x < PLATE_X[1] + 0.010 and PLATE_Y[0] - 0.010 < y < PLATE_Y[1] + 0.010
    return bool(over and state.resting() and _bottom(state, r) > TOP + 0.010)


def _tally(objects):
    correct = wrong = 0
    for p in GO_PINS:
        st = objects[p.name]
        if _in_pocket(st):
            correct += 1
        elif _on_pad(st, GO_R):
            wrong += 1
    for p in NOGO_PINS:
        st = objects[p.name]
        if _on_pad(st, NOGO_R):
            correct += 1
        elif _on_plate(st):
            wrong += 1
    return correct, wrong


def success(objects) -> bool:
    """All six pins routed correctly, at rest; holds for 1 s."""
    correct, wrong = _tally(objects)
    return correct == 6 and wrong == 0


def distance(objects) -> float:
    """Metres still to go: each go pin to its nearest pocket centre plus each no-go pin to the pad centre."""
    total = 0.0
    for p in GO_PINS:
        st = objects[p.name]
        total += min(math.hypot(float(st.pos[0]) - cx, float(st.pos[1]) - POCKET_Y) for cx in POCKET_XS)
    for p in NOGO_PINS:
        st = objects[p.name]
        total += math.hypot(float(st.pos[0]) - PAD[0], float(st.pos[1]) - PAD[1])
    return total


def score(objects) -> float:
    """Net routing: (correct minus wrong) over six, floored at 0. A 20 mm pin seated in a pocket or a 28 mm pin on
    the pad is correct; a 20 mm pin on the pad or a 28 mm pin left on the gauge is wrong; anything else is neither.
    Dumping all six on the pad therefore scores 0, and random routing scores 0 in expectation."""
    correct, wrong = _tally(objects)
    return max(0, correct - wrong) / 6.0


TRAVEL = TOP + 0.140
GRIP_Z = TOP + 0.045  # TCP 15 mm above the pin centre, pad tips 32 mm up, the pin top 15 mm above the TCP
PLACE_Z = TOP + 0.055  # pin bottom 10 mm above a pocket floor, 15 mm inside the pocket; 5 mm above the pad
ENTRY = 0.025  # the pause above a pocket: pin bottom 10 mm above the 25 mm walls


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
    """Left arm only, yaw 0. Three go pins into the three pockets, then three no-go pins onto the pad, each a
    top-down pick and place at 14 cm travel height. 97.2 s of plan; the episode ends on success about 1 s after
    the last release (the scripted policy would otherwise stop after a 3 s hold, at step 2,004 at 20 Hz)."""
    left = []
    for (x, y), cx in zip(GO_START, POCKET_XS):
        left += _pick_place((x, y, GRIP_Z), (cx, POCKET_Y, PLACE_Z), width=2 * GO_R, entry=ENTRY, travel_z=TRAVEL)
    for (x, y), px in zip(NOGO_START, (0.155, 0.200, 0.245)):
        left += _pick_place((x, y, GRIP_Z), (px, PAD[1], PLACE_Z), width=2 * NOGO_R, entry=ENTRY, travel_z=TRAVEL)
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def pin(r, x, y, z=None, lying=False, vel=(0.0, 0.0, 0.0)):
        if lying:
            pos = np.array([x, y, (TOP if z is None else z) + r])
            half = np.array([PIN_H / 2, r, r])
            quat = np.array([0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)])
        else:
            pos = np.array([x, y, (TOP if z is None else z) + PIN_H / 2])
            half = np.array([r, r, PIN_H / 2])
            quat = np.array([0.0, 0.0, 0.0, 1.0])
        return ObjectState(pos, quat, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def rolled(r, x, y, surface, roll):
        """A pin lying along x, turned ``roll`` about its own axis; low/high from the rotated box corners, as the
        simulator reports them."""
        ca, sa = math.cos(roll / 2), math.sin(roll / 2)
        cb, sb = math.cos(math.pi / 4), math.sin(math.pi / 4)
        quat = np.array([sa * cb, ca * sb, sa * sb, ca * cb])  # roll about x after 90 degrees about y
        xq, yq, zq, wq = quat
        rot = np.array([[1 - 2 * (yq * yq + zq * zq), 2 * (xq * yq - zq * wq), 2 * (xq * zq + yq * wq)],
                        [2 * (xq * yq + zq * wq), 1 - 2 * (xq * xq + zq * zq), 2 * (yq * zq - xq * wq)],
                        [2 * (xq * zq - yq * wq), 2 * (yq * zq + xq * wq), 1 - 2 * (xq * xq + yq * yq)]])
        half = np.abs(rot) @ np.array([r, r, PIN_H / 2])
        pos = np.array([x, y, surface + r])
        return ObjectState(pos, quat, np.zeros(3), np.zeros(3), pos - half, pos + half)

    def scene(go=None, nogo=None):
        go = go or [pin(GO_R, x, y) for x, y in GO_START]
        nogo = nogo or [pin(NOGO_R, x, y) for x, y in NOGO_START]
        return {**{p.name: s for p, s in zip(GO_PINS, go)}, **{p.name: s for p, s in zip(NOGO_PINS, nogo)}}

    seated = [pin(GO_R, cx, POCKET_Y) for cx in POCKET_XS]
    on_pad_nogo = [pin(NOGO_R, px, PAD[1], z=PAD_TOP) for px in (0.155, 0.200, 0.245)]
    on_pad_go = [pin(GO_R, px, PAD[1], z=PAD_TOP) for px in (0.155, 0.200, 0.245)]
    on_walls = [pin(NOGO_R, cx, POCKET_Y, z=TOP + POCKET_DEPTH) for cx in POCKET_XS]
    cases = {
        "start": (scene(), 0.0, False),
        "all six dumped on the pad": (scene(go=on_pad_go, nogo=on_pad_nogo), 0.0, False),
        "go pins seated, rejects untouched": (scene(go=seated), 0.5, False),
        "one go pin seated": (scene(go=[seated[0], pin(GO_R, *GO_START[1]), pin(GO_R, *GO_START[2])]), 1 / 6, False),
        "rejects on the pad, go pins untouched": (scene(nogo=on_pad_nogo), 0.5, False),
        "every pin pushed at the gauge": (scene(go=seated, nogo=on_walls), 0.0, False),
        "done": (scene(go=seated, nogo=on_pad_nogo), 1.0, True),
        "done but one go pin lying on the table": (scene(go=[seated[0], seated[1], pin(GO_R, 0.30, 0.25, lying=True)], nogo=on_pad_nogo), 5 / 6, False),
        "done but one go pin still held": (scene(go=[seated[0], seated[1], pin(GO_R, POCKET_XS[2], POCKET_Y, z=TOP + 0.08, vel=(0, 0, 0.02))], nogo=on_pad_nogo), 5 / 6, False),
        "a reject lying on the pad counts": (scene(go=seated, nogo=[on_pad_nogo[0], on_pad_nogo[1], pin(NOGO_R, 0.24, PAD[1], z=PAD_TOP, lying=True)]), 1.0, True),
        "a reject left standing on a pocket": (scene(go=seated, nogo=[on_pad_nogo[0], on_pad_nogo[1], on_walls[2]]), 4 / 6, False),
        "a reject lying on the pad at 45 degrees of roll counts": (scene(go=seated, nogo=[on_pad_nogo[0], on_pad_nogo[1], rolled(NOGO_R, 0.24, PAD[1], PAD_TOP, math.pi / 4)]), 1.0, True),
        "a go pin lying on the pad at 45 degrees of roll is wrong": (scene(go=[seated[0], seated[1], rolled(GO_R, 0.24, PAD[1], PAD_TOP, math.pi / 4)], nogo=on_pad_nogo), 4 / 6, False),
        "a go pin on the pad edge, centre outside": (scene(go=[seated[0], seated[1], pin(GO_R, PAD[0] + 0.065, PAD[1], z=PAD_TOP)], nogo=on_pad_nogo), 5 / 6, False),
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
