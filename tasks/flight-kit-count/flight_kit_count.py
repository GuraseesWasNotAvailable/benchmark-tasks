"""On the station table, in the left arm's lane, a 1 g bench analogue of building a flight fastener kit: the kit gets an
exact count and every other fastener goes to the spares tray, because a loose bolt on a spacecraft is foreign object
debris. Seven identical grey bolts (plain cylinders 16 mm across and 50 mm tall) stand in a row 40 mm apart. A white
kit tray and a black spares tray, each four fixed walls 20 mm tall round an open 200 x 56 mm pocket, lie either side
of the row; each holds all seven, so neither tray gives the count away. Five of seven is not an even split, so
alternating trays or halving the row does not reach it. The score counts the bolts in each tray from their centres
and axes: exactly five in the kit comes first, one short earns a little, one over (the extra bolt that becomes
debris) earns nothing, and the spares tray adds credit only once the kit count is right.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/flight-kit-count/flight_kit_count.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Put exactly five grey bolts in the white kit tray and all the other bolts in the black spares tray." \\
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
BOLT_R = 0.008  # 16 mm across
BOLT_H = 0.050
BOLT_MASS = 0.020
ROW_Y = 0.305  # the row runs straight out from the left arm's base
ROW_XS = (0.130, 0.170, 0.210, 0.250, 0.290, 0.330, 0.370)  # 40 mm apart, 24 mm of air between neighbours

INNER = (0.200, 0.056)  # each tray's open pocket, along x and along y
WALL_T = 0.006
WALL_H = 0.020
KIT = (0.250, 0.410)  # pocket centres; each tray's nearer wall stands 63 mm clear of the row's bolts
SPARES = (0.250, 0.200)
KIT_COUNT = 5  # not an even split of seven: taking the trays in turn ends with four or three in the kit
BAND = 0.070  # a bolt counts only with its lowest point under 70 mm (wall plus one bolt length): in the pocket or resting on bolts that are in it
SINK = 0.005  # and no more than 5 mm under the table top, so a bolt on the floor under a pocket does not count

GREY = (0.62, 0.62, 0.64)
WHITE = (0.92, 0.92, 0.92)
BLACK = (0.08, 0.08, 0.08)


def _tray(name, centre, color):
    cx, cy = centre
    ix, iy = INNER
    z = TOP + WALL_H / 2
    side = (ix + 2 * WALL_T, WALL_T, WALL_H)
    end = (WALL_T, iy, WALL_H)
    return [
        Box(f"{name}_wall_out", pos=(cx, cy + iy / 2 + WALL_T / 2, z), size=side, color=color),
        Box(f"{name}_wall_in", pos=(cx, cy - iy / 2 - WALL_T / 2, z), size=side, color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - WALL_T / 2, cy, z), size=end, color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + WALL_T / 2, cy, z), size=end, color=color),
    ]


BOLTS = [Cylinder(f"bolt_{k}", pos=(x, ROW_Y, TOP + BOLT_H / 2), radius=BOLT_R, height=BOLT_H, mass=BOLT_MASS, color=GREY) for k, x in enumerate(ROW_XS)]
TRAYS = [*_tray("kit", KIT, WHITE), *_tray("spares", SPARES, BLACK)]
OBJECTS = [*BOLTS, *TRAYS]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _bottom(state) -> float:
    """The bolt's lowest point from its centre and axis, standing, lying or leaning. The axis-aligned bounds of a
    lying cylinder come from its box corners and read up to r(sqrt 2 - 1) too low, so they are not used."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    uz = 1.0 - 2.0 * (x * x + y * y)  # world z of the bolt's axis
    return float(state.pos[2]) - (BOLT_H / 2 * abs(uz) + BOLT_R * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _gap(state, centre) -> float:
    """Horizontal distance from the bolt's centre to a tray's inner pocket, 0 when the centre is inside it."""
    dx = max(0.0, abs(float(state.pos[0]) - centre[0]) - INNER[0] / 2)
    dy = max(0.0, abs(float(state.pos[1]) - centre[1]) - INNER[1] / 2)
    return math.hypot(dx, dy)


def _in_tray(state, centre) -> bool:
    """At rest, the centre strictly inside the inner walls seen from above, the lowest point from 5 mm under the
    table top to 70 mm over it: standing, lying or leaning in the pocket, or resting on bolts that are, but not held
    above it, still moving, or on the floor under the open table."""
    x, y = float(state.pos[0]), float(state.pos[1])
    inside = abs(x - centre[0]) < INNER[0] / 2 and abs(y - centre[1]) < INNER[1] / 2
    return bool(inside and state.resting() and TOP - SINK < _bottom(state) < TOP + BAND)


def _counts(objects):
    kit = spares = 0
    for b in BOLTS:
        st = objects[b.name]
        if _in_tray(st, KIT):
            kit += 1
        elif _in_tray(st, SPARES):
            spares += 1
    return kit, spares


def success(objects) -> bool:
    """Exactly five bolts in the kit tray and the other two in the spares tray, all at rest; holds for 1 s."""
    kit, spares = _counts(objects)
    return kit == KIT_COUNT and spares == len(BOLTS) - KIT_COUNT


def distance(objects) -> float:
    """Metres still to go: the five bolts nearest the kit pocket to it, the other two to the spares pocket."""
    ranked = sorted(BOLTS, key=lambda b: _gap(objects[b.name], KIT))
    return sum(_gap(objects[b.name], KIT) for b in ranked[:KIT_COUNT]) + sum(_gap(objects[b.name], SPARES) for b in ranked[KIT_COUNT:])


def score(objects) -> float:
    """The kit count gates everything. Exactly five in the kit: 0.5 plus 1/4 per bolt in the spares tray, so 0.5,
    0.75 or 1.0. Four in the kit (one short, which stops the job): 0.25 whatever the spares tray holds. Six in the
    kit (one over, the extra bolt that can become debris) and any other count (0 to 3, or 7): 0. The start state
    and all seven in either tray score 0."""
    kit, spares = _counts(objects)
    if kit == KIT_COUNT:
        return 0.5 + spares / (2.0 * (len(BOLTS) - KIT_COUNT))
    if kit == KIT_COUNT - 1:
        return 0.25
    return 0.0


TRAVEL = TOP + 0.140
GRIP_Z = TOP + 0.037  # TCP 13 mm under the bolt top; pad tips 24 mm up, 4 mm above the tray walls
PLACE_Z = TOP + 0.042  # bolt bottom 5 mm above the table inside a pocket; pad tips 9 mm above the walls on release
ENTRY = 0.025  # the pause above a pocket: bolt bottom 10 mm above the 20 mm walls
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part

KIT_SPOTS = tuple((x, KIT[1]) for x in (0.170, 0.210, 0.250, 0.290, 0.330))  # 40 mm apart, 12 mm of air to the end walls
SPARE_SPOTS = ((0.230, SPARES[1]), (0.270, SPARES[1]))
KIT_PICKS = (2, 3, 4, 5, 6)  # row bolts, near to far
SPARE_PICKS = (0, 1)


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
    """Left arm only, yaw 0, the fingers closing across the row so each bolt's neighbours sit beside the pads, not in
    front of them. The five far bolts of the row go standing into the kit tray, then the two near ones into the
    spares tray, each a top-down pick, a carry at 14 cm and a drop of 5 mm onto the table inside the walls. 113.4 s
    of plan; the episode ends on success about 1 s after the last release (the scripted policy would otherwise stop
    after a 3 s hold, at step 2,328 at 20 Hz)."""
    left = []
    for k, (qx, qy) in zip(KIT_PICKS, KIT_SPOTS):
        left += _pick_place((ROW_XS[k], ROW_Y, GRIP_Z), (qx, qy, PLACE_Z), width=2 * BOLT_R, entry=ENTRY, travel_z=TRAVEL)
    for k, (qx, qy) in zip(SPARE_PICKS, SPARE_SPOTS):
        left += _pick_place((ROW_XS[k], ROW_Y, GRIP_Z), (qx, qy, PLACE_Z), width=2 * BOLT_R, entry=ENTRY, travel_z=TRAVEL)
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw, tilt):  # tilt about the bolt's own x axis, then yaw about the vertical
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        return np.array([cy * st, sy * st, sy * ct, cy * ct])

    def bolt(x, y, bottom=TOP, tilt=0.0, yaw=0.0, vel=(0.0, 0.0, 0.0)):
        """A bolt with its lowest point at ``bottom``; low/high from the rotated box corners, as the simulator
        reports them."""
        q = quat(yaw, tilt)
        xq, yq, zq, wq = q
        rot = np.array([[1 - 2 * (yq * yq + zq * zq), 2 * (xq * yq - zq * wq), 2 * (xq * zq + yq * wq)],
                        [2 * (xq * yq + zq * wq), 1 - 2 * (xq * xq + zq * zq), 2 * (yq * zq - xq * wq)],
                        [2 * (xq * zq - yq * wq), 2 * (yq * zq + xq * wq), 1 - 2 * (xq * xq + yq * yq)]])
        half = np.abs(rot) @ np.array([BOLT_R, BOLT_R, BOLT_H / 2])
        pos = np.array([x, y, bottom + BOLT_H / 2 * abs(math.cos(tilt)) + BOLT_R * abs(math.sin(tilt))])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = [bolt(x, ROW_Y) for x in ROW_XS]
    kit = [bolt(x, y) for x, y in KIT_SPOTS] + [bolt(0.190, KIT[1] - 0.015), bolt(0.310, KIT[1] + 0.015)]
    spare = [bolt(x, y) for x, y in SPARE_SPOTS] + [bolt(x, SPARES[1] + dy) for x, dy in ((0.190, 0.015), (0.310, -0.015), (0.170, 0.0), (0.330, 0.0), (0.250, 0.0))]

    def scene(n_kit, n_spare, extra=()):
        """The first n_kit bolts in the kit tray, the next n_spare in the spares tray, the rest left on their marks;
        ``extra`` replaces single bolts by index."""
        states = list(start)
        for k in range(n_kit):
            states[k] = kit[k]
        for k in range(n_spare):
            states[n_kit + k] = spare[k]
        for k, st in extra:
            states[k] = st
        return {b.name: st for b, st in zip(BOLTS, states)}

    def in_turn(kit_first):
        """Every bolt taken in row order to the two trays in turn, with no counting."""
        states, n_kit, n_spare = [], 0, 0
        for k in range(len(BOLTS)):
            if (k % 2 == 0) == kit_first:
                states.append(kit[n_kit])
                n_kit += 1
            else:
                states.append(spare[n_spare])
                n_spare += 1
        return {b.name: st for b, st in zip(BOLTS, states)}

    lying_in_kit = bolt(0.330, KIT[1], tilt=math.pi / 2, yaw=0.3)  # axis across the pocket
    leaning_in_kit = bolt(0.335, KIT[1] + 0.008, tilt=-math.radians(25))  # top against the outer wall
    across_kit = bolt(0.230, KIT[1], bottom=TOP + BOLT_H, tilt=math.pi / 2, yaw=math.pi / 2)  # along x on two kit bolts' tops
    on_kit_wall = bolt(0.250, KIT[1] + INNER[1] / 2 + WALL_T / 2, bottom=TOP + WALL_H)  # centre over the wall
    beside_kit = bolt(0.250, KIT[1] + INNER[1] / 2 + WALL_T + 0.030)  # outside the outer wall
    under_kit = bolt(0.250, KIT[1], bottom=0.0, tilt=math.pi / 2)  # on the floor below the open table, under the pocket
    cases = {
        "start": (scene(0, 0), 0.0, False),
        "done": (scene(5, 2), 1.0, True),
        "five in the kit, the rest on their marks": (scene(5, 0), 0.5, False),
        "five in the kit, one spare": (scene(5, 1), 0.75, False),
        "four in the kit, three spares (one short)": (scene(4, 3), 0.25, False),
        "four in the kit, the rest loose": (scene(4, 0), 0.25, False),
        "six in the kit, one spare (one over)": (scene(6, 1), 0.0, False),
        "six in the kit, the rest loose": (scene(6, 0), 0.0, False),
        "three in the kit, four spares": (scene(3, 4), 0.0, False),
        "two in the kit, five spares": (scene(2, 5), 0.0, False),
        "all seven in the kit": (scene(7, 0), 0.0, False),
        "all seven in the spares tray": (scene(0, 7), 0.0, False),
        "one in the kit": (scene(1, 0), 0.0, False),
        "trays in turn, kit first, no counting": (in_turn(True), 0.25, False),
        "trays in turn, spares first, no counting": (in_turn(False), 0.0, False),
        "a kit bolt lying in the pocket counts": (scene(4, 2, [(6, lying_in_kit)]), 1.0, True),
        "a kit bolt leaning on the wall inside counts": (scene(4, 2, [(6, leaning_in_kit)]), 1.0, True),
        "a fifth bolt lying across two kit bolts counts": (scene(4, 2, [(6, across_kit)]), 1.0, True),
        "a sixth bolt standing on a kit bolt counts": (scene(5, 1, [(6, bolt(KIT_SPOTS[0][0], KIT[1], bottom=TOP + BOLT_H))]), 0.0, False),
        "a sixth bolt held 75 mm up over the kit does not count": (scene(5, 1, [(6, bolt(0.250, KIT[1], bottom=TOP + 0.075))]), 0.75, False),
        "a bolt on the kit wall does not count": (scene(4, 2, [(6, on_kit_wall)]), 0.25, False),
        "a bolt outside the kit wall does not count": (scene(4, 2, [(6, beside_kit)]), 0.25, False),
        "a bolt on the floor under the kit pocket does not count": (scene(4, 2, [(6, under_kit)]), 0.25, False),
        "centre 1 mm inside the pocket counts": (scene(4, 2, [(6, bolt(KIT[0] + INNER[0] / 2 - 0.001, KIT[1], tilt=math.pi / 2))]), 1.0, True),
        "centre 1 mm outside the pocket does not": (scene(4, 2, [(6, bolt(KIT[0] + INNER[0] / 2 + 0.001, KIT[1], bottom=TOP + WALL_H, tilt=math.pi / 2))]), 0.25, False),
        "done but a spare still moving": (scene(5, 1, [(6, bolt(0.270, SPARES[1], vel=(0.0, 0.0, -0.05)))]), 0.75, False),
        "done but a kit bolt still rolling": (scene(4, 2, [(6, bolt(0.330, KIT[1], tilt=math.pi / 2, vel=(0.0, 0.02, 0.0)))]), 0.25, False),
        "five in the kit, a spare knocked over on the table": (scene(5, 1, [(6, bolt(0.370, ROW_Y, tilt=math.pi / 2))]), 0.75, False),
        "five in the kit, a spare off the table": (scene(5, 1, [(6, bolt(0.950, 0.300, bottom=0.0, tilt=math.pi / 2))]), 0.75, False),
        "every bolt knocked over where it stood": (scene(0, 0, [(k, bolt(x, ROW_Y, tilt=math.pi / 2, yaw=1.0)) for k, x in enumerate(ROW_XS)]), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        assert not want_success or got == 1.0, name
    assert distance(scene(5, 2)) == 0.0 and distance(scene(0, 0)) > 0.0
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
