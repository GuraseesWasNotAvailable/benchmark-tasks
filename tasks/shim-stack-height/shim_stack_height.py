"""On the station table, in the left arm's lane, a 1 g bench analogue of shimming a mount to its set height during
spacecraft integration. Five loose shims share one 60 x 50 mm footprint and are colour coded by thickness: purple
20 mm, green 25, blue 30, red 35 and white 45. A fixed black pad, 70 x 60 x 6 mm, is the mount. The stack must stand
flat on the pad with its top 80 mm above the pad's top face, within 2 mm, built from as few shims as possible, so the
policy has to pick the fewest shims whose thicknesses add up to 80 mm (red and white) and stack them level, centred
and each resting on the one below. The three-shim set that also adds up to 80 mm earns 0.75. No single shim, no shim
on its edge or end, and no shim held, bridged or propped above the one below can count. The score reads the stack
from each shim's centre and quaternion.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/shim-stack-height/shim_stack_height.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Stack shims flat on the black pad so the top of the stack is 80 mm above the pad, within 2 mm, using as few shims as possible. The purple, green, blue, red and white shims are 20, 25, 30, 35 and 45 mm thick." \\
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
SHIM_L, SHIM_W = 0.060, 0.050  # every shim's footprint: 60 mm along x and 50 mm along y at the start
DENSITY = 700.0  # kg/m^3, a hardwood such as beech, or a part-infill print, so mass follows thickness
PAD_C = (0.220, 0.305)  # the pad's centre, 220 mm straight out from the left arm's base
PAD_SIZE = (0.070, 0.060, 0.006)
PAD_TOP = TOP + PAD_SIZE[2]
TARGET = 0.080  # the stack's top above the pad's top face
TARGET_Z = PAD_TOP + TARGET

PURPLE = (0.55, 0.25, 0.70)
GREEN = (0.20, 0.60, 0.25)
BLUE = (0.15, 0.35, 0.80)
RED = (0.80, 0.12, 0.10)
WHITE = (0.92, 0.92, 0.90)
BLACK = (0.05, 0.05, 0.05)

SHIM_SPECS = (  # colour name, thickness, sRGB, start centre (x, y); 100 mm apart across the lane
    ("purple", 0.020, PURPLE, (0.330, 0.405)),
    ("green", 0.025, GREEN, (0.220, 0.405)),
    ("blue", 0.030, BLUE, (0.330, 0.205)),
    ("red", 0.035, RED, (0.220, 0.205)),
    ("white", 0.045, WHITE, (0.330, 0.305)),
)
THICK = {f"shim_{c}": t for c, t, _, _ in SHIM_SPECS}
SHIMS = [
    Box(f"shim_{c}", pos=(x, y, TOP + t / 2), size=(SHIM_L, SHIM_W, t), mass=round(DENSITY * SHIM_L * SHIM_W * t, 4), color=rgb)
    for c, t, rgb, (x, y) in SHIM_SPECS
]
PAD = Box("mount_pad", pos=(PAD_C[0], PAD_C[1], TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=BLACK)
OBJECTS = [*SHIMS, PAD]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

OFF_TABLE = 0.003  # a shim whose lowest point is more than 3 mm above the table is off the table
CENTRE_TOL = 0.015  # every stack shim's centre to the pad centre, horizontally
LEVEL_TOL = math.radians(3.0)  # a shim's thickness axis to the vertical
GAP_TOL = 0.002  # each stack shim's lowest point to the surface under it: the pad's top face or the shim below's top
GAP_SUM = 0.002  # those gaps added up, so a 75 mm set propped or bridged up cannot read 78 mm
EPS = 1e-4  # 0.1 mm, so the exact limits count
LADDER = ((0.002, 1.0), (0.007, 0.5), (0.012, 0.25))  # height error limits; sums move in 5 mm steps
MORE_SHIMS = 0.75  # within 2 mm of 80 mm but built from more shims than the fewest that reach it
_SETS = [[t for k, t in enumerate(THICK.values()) if m >> k & 1] for m in range(1, 2 ** len(THICK))]
FEWEST = min(len(s) for s in _SETS if abs(sum(s) - TARGET) < 1e-9)  # 2: red and white; purple, green and red is 3


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _shim(state, t):
    """(bottom, top, level, off-centre) from the shim's centre and quaternion. The bottom is the lowest corner; the
    top is the centre of the face whose normal is the thickness axis, so a level stack reads the pad top plus the
    thicknesses whichever face is up."""
    rot = _rot(state.quat_xyzw)
    z = float(state.pos[2])
    bottom = z - float(np.abs(rot[2, :]) @ np.array([SHIM_L / 2, SHIM_W / 2, t / 2]))
    up = abs(float(rot[2, 2]))
    off = math.hypot(float(state.pos[0]) - PAD_C[0], float(state.pos[1]) - PAD_C[1])
    return bottom, z + t / 2 * up, up >= math.cos(LEVEL_TOL), off


def _over_pad(state, t):
    """True when any part of the shim's outline, seen from above, lies over the pad's 70 x 60 mm outline: a
    separating-axis test of the shim's eight corners, from its centre and quaternion, against the pad's rectangle.
    The axes are the pad's two and the normals of the shim's three edge directions as seen from above."""
    rot = _rot(state.quat_xyzw)
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float)
    corners = (signs * [SHIM_L / 2, SHIM_W / 2, t / 2]) @ rot.T[:, :2] + np.asarray(state.pos, float)[:2]
    pad = np.array([[PAD_C[0] + i * PAD_SIZE[0] / 2, PAD_C[1] + j * PAD_SIZE[1] / 2] for i in (-1, 1) for j in (-1, 1)])
    axes = [np.array([1.0, 0.0]), np.array([0.0, 1.0])]
    axes += [np.array([-rot[1, k], rot[0, k]]) for k in range(3) if math.hypot(rot[0, k], rot[1, k]) > 1e-6]
    return all(min((corners @ a).max(), (pad @ a).max()) > max((corners @ a).min(), (pad @ a).min()) for a in axes)


def _stack(objects):
    """("ok", top, count) for a valid stack, ("none", None, 0) when no shim is over the pad, ("spoiled", None, count)
    otherwise. Every shim off the table with any part of its outline over the pad's outline is in the stack. Each must
    be at rest, level and centred within 15 mm, and rest on the surface under it: its lowest point within 2 mm of the
    pad's top face (the lowest shim) or of the top face of the shim below, and those gaps added up within 2 mm. So a
    shim held, bridged or propped above the one below spoils the stack instead of making up the height."""
    stack = []
    for b in SHIMS:
        st, t = objects[b.name], THICK[b.name]
        bottom, top, level, off = _shim(st, t)
        if bottom > TOP + OFF_TABLE and _over_pad(st, t):
            stack.append((bottom, top, st.resting() and level and off <= CENTRE_TOL + EPS))
    if not stack:
        return "none", None, 0
    stack.sort(key=lambda s: s[0])  # bottom first
    under = [PAD_TOP] + [top for _, top, _ in stack[:-1]]  # what each shim must sit on: the pad, then the shim below
    gaps = [bottom - u for (bottom, _, _), u in zip(stack, under)]
    if not all(ok for _, _, ok in stack) or any(abs(g) > GAP_TOL + EPS for g in gaps) or sum(gaps) > GAP_SUM + EPS:
        return "spoiled", None, len(stack)
    return "ok", stack[-1][1], len(stack)


def _ladder(err):
    """The height rung: 1.0 within 2 mm, 0.5 within 7 mm, 0.25 within 12 mm, else 0; the exact limits count."""
    for limit, value in LADDER:
        if err <= limit + EPS:
            return value
    return 0.0


def score(objects) -> float:
    """1.0 when the stack's top is within 2 mm of 80 mm above the pad and it uses the fewest shims that reach 80 mm
    (two), 0.75 within 2 mm with more shims, 0.5 within 7 mm (a set one 5 mm step off), 0.25 within 12 mm (two steps
    off), else 0; 0 with no shim over the pad or a spoiled stack."""
    status, top, count = _stack(objects)
    if status != "ok":
        return 0.0
    value = _ladder(abs(top - TARGET_Z))
    return MORE_SHIMS if value == 1.0 and count > FEWEST else value


def success(objects) -> bool:
    """The full-score stack, every shim in it at rest; holds for 1 s."""
    return score(objects) == 1.0


def distance(objects) -> float:
    """Metres still to go: the stack's height error plus 5 mm for each shim beyond the fewest, or with no valid stack
    the full 80 mm plus the nearest shim's centre to the pad centre."""
    status, top, count = _stack(objects)
    if status == "ok":
        return abs(top - TARGET_Z) + 0.005 * max(0, count - FEWEST)
    return TARGET + min(math.hypot(float(objects[b.name].pos[0]) - PAD_C[0], float(objects[b.name].pos[1]) - PAD_C[1]) for b in SHIMS)


TRAVEL = TOP + 0.140
GRIP_DZ = 0.019  # TCP above a shim's bottom: pad tips 6 mm up its sides
DROP = 0.005  # release with the shim's bottom 5 mm above the surface it lands on
ENTRY = 0.015  # the pause above a place: the shim's bottom 20 mm above that surface
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
    """Left arm only, yaw 0, the fingers closing across each shim's 50 mm width. The white 45 mm shim onto the pad,
    then the red 35 mm shim onto it: 80 mm. Each a top-down pick with the pad tips 6 mm up the shim's sides, a
    carry at 14 cm and a 5 mm drop. 32.4 s of plan, then the 3 s hold: 708 steps at 20 Hz."""
    spec = {c: (t, xy) for c, t, _, xy in SHIM_SPECS}
    left, support = [], PAD_TOP
    for colour in ("white", "red"):
        t, (x, y) = spec[colour]
        pick = (x, y, TOP + GRIP_DZ)
        place = (PAD_C[0], PAD_C[1], support + DROP + GRIP_DZ)
        left += _pick_place(pick, place, width=SHIM_W, entry=ENTRY, travel_z=TRAVEL)
        support += t
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw, tilt=0.0, axis=0):  # yaw about the vertical, then tilt about the shim's own x (0) or y (1) axis
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        if axis == 0:
            return np.array([cy * st, sy * st, sy * ct, cy * ct])
        return np.array([-sy * st, cy * st, sy * ct, cy * ct])

    def shim(colour, x, y, bottom=TOP, yaw=0.0, tilt=0.0, axis=0, vel=(0.0, 0.0, 0.0)):
        t = THICK[f"shim_{colour}"]
        q = quat(yaw, tilt, axis)
        half = np.abs(_rot(q)) @ np.array([SHIM_L / 2, SHIM_W / 2, t / 2])
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {f"shim_{c}": shim(c, x, y) for c, _, _, (x, y) in SHIM_SPECS}

    def scene(*placed):
        states = dict(start)
        for colour, st in placed:
            states[f"shim_{colour}"] = st
        return states

    def stack(*colours, dx=0.0, dy=0.0, at=PAD_C, base=PAD_TOP, **kw):
        """The colours stacked bottom first on the pad (or at ``at`` from height ``base``), centred; ``kw`` and the
        offsets apply to the top shim."""
        placed, z = [], base
        for k, c in enumerate(colours):
            last = k == len(colours) - 1
            placed.append((c, shim(c, at[0] + (dx if last else 0.0), at[1] + (dy if last else 0.0), z, **(kw if last else {}))))
            z += THICK[f"shim_{c}"]
        return placed

    def raised(colours, dz):  # the stack with its top shim lifted dz: a dz gap under it, the top dz higher
        placed = stack(*colours)
        c, st = placed[-1]
        placed[-1] = (c, shim(c, PAD_C[0], PAD_C[1], float(st.low[2]) + dz))
        return placed

    cx, cy = PAD_C
    # The bridge: purple and green stand on their 50 mm edges on the table either side of the pad, tops 44 mm above
    # it; blue lies flat on the pad; red, yawed 50.2 degrees so its corners reach the supports, rests on them, level
    # and centred, its top 79 mm above the pad. The set is 30 + 35 = 65 mm.
    bridge = (
        ("purple", shim("purple", cx, cy + 0.040, TOP, tilt=math.pi / 2)),
        ("green", shim("green", cx, cy - 0.0425, TOP, tilt=math.pi / 2)),
        ("blue", shim("blue", cx, cy, PAD_TOP)),
        ("red", shim("red", cx, cy, TOP + SHIM_W, yaw=math.radians(50.2))),
    )
    # A spare pair stacked off the pad's front-right corner, yawed -45 degrees: the corners' bounding box overlaps
    # the pad, the outline stays 12 mm clear of it.
    corner = (cx + PAD_SIZE[0] / 2 + 0.030, cy - PAD_SIZE[1] / 2 - 0.030)
    cases = {
        "start": (scene(), 0.0, False),
        "red on white, 80 mm": (scene(*stack("white", "red")), 1.0, True),
        "red under white, 80 mm": (scene(*stack("red", "white")), 1.0, True),
        "purple, green and red, 80 mm with three shims": (scene(*stack("purple", "green", "red")), 0.75, False),
        "blue on white, 75 mm": (scene(*stack("white", "blue")), 0.5, False),
        "purple, green and blue, 75 mm": (scene(*stack("purple", "green", "blue")), 0.5, False),
        "purple, blue and red, 85 mm": (scene(*stack("purple", "blue", "red")), 0.5, False),
        "green on white, 70 mm": (scene(*stack("white", "green")), 0.25, False),
        "green, blue and red, 90 mm": (scene(*stack("green", "blue", "red")), 0.25, False),
        "purple on white, 65 mm": (scene(*stack("white", "purple")), 0.0, False),
        "white alone on the pad": (scene(*stack("white")), 0.0, False),
        "all five stacked, 155 mm": (scene(*stack("purple", "green", "blue", "red", "white")), 0.0, False),
        "red resting with a 2.0 mm gap over white": (scene(*raised(("white", "red"), 0.002)), 1.0, True),
        "red with a 2.3 mm gap over white": (scene(*raised(("white", "red"), 0.0023)), 0.0, False),
        "white 2.0 mm above the pad, red on it": (scene(*stack("white", "red", base=PAD_TOP + 0.002)), 1.0, True),
        "white 1.5 mm above the pad, red 1.5 mm above white": (scene(*stack("white", base=PAD_TOP + 0.0015), ("red", shim("red", cx, cy, PAD_TOP + 0.0015 + 0.045 + 0.0015))), 0.0, False),
        "blue propped 3 mm above white, top 78 mm": (scene(*raised(("white", "blue"), 0.003)), 0.0, False),
        "purple held still 15 mm above white, top 80 mm": (scene(*stack("white"), ("purple", shim("purple", cx, cy, PAD_TOP + 0.060))), 0.0, False),
        "red bridged on two edge-standing shims over blue, top 79 mm": (scene(*bridge), 0.0, False),
        "red on white, top shim tilted 2.5 degrees": (scene(*stack("white", "red", tilt=math.radians(2.5))), 1.0, True),
        "red on white, top shim tilted 4 degrees": (scene(*stack("white", "red", tilt=math.radians(4))), 0.0, False),
        "red on white, top shim upside down": (scene(*stack("white", "red", tilt=math.pi)), 1.0, True),
        "red on white, top shim turned 90 degrees": (scene(*stack("white", "red", yaw=math.pi / 2)), 1.0, True),
        "red on white, top shim turned 45 degrees": (scene(*stack("white", "red", yaw=math.pi / 4)), 1.0, True),
        "red on white, top shim 14 mm off centre": (scene(*stack("white", "red", dx=0.014)), 1.0, True),
        "red on white, top shim 16 mm off centre": (scene(*stack("white", "red", dy=0.016)), 0.0, False),
        "red on white, top shim still moving": (scene(*stack("white", "red", vel=(0.0, 0.0, 0.02))), 0.0, False),
        "red held 20 mm above the white shim, moving": (scene(*stack("white"), ("red", shim("red", cx, cy, PAD_TOP + 0.065, vel=(0.0, 0.0, 0.03)))), 0.0, False),
        "red on white, the pair floating 4 mm above the pad": (scene(*stack("white", "red", base=PAD_TOP + 0.004)), 0.0, False),
        "red on edge on the pad (50 mm) with blue flat on it": (scene(("red", shim("red", cx, cy, PAD_TOP, tilt=math.pi / 2)), ("blue", shim("blue", cx, cy, PAD_TOP + SHIM_W))), 0.0, False),
        "white on edge on the pad (50 mm) with blue flat on it": (scene(("white", shim("white", cx, cy, PAD_TOP, tilt=math.pi / 2)), ("blue", shim("blue", cx, cy, PAD_TOP + SHIM_W))), 0.0, False),
        "blue standing on end on the pad (60 mm) with purple on it": (scene(("blue", shim("blue", cx, cy, PAD_TOP, tilt=math.pi / 2, axis=1)), ("purple", shim("purple", cx, cy, PAD_TOP + SHIM_L))), 0.0, False),
        "red on white beside the pad, on the table": (scene(*stack("white", "red", at=(0.120, 0.305), base=TOP)), 0.0, False),
        "80 mm stack plus a spare shim lying on the table by the pad": (scene(*stack("white", "red"), ("blue", shim("blue", cx, cy - 0.056))), 1.0, True),
        "80 mm stack plus a spare stack far from the pad": (scene(*stack("white", "red"), ("blue", shim("blue", 0.330, 0.205)), ("purple", shim("purple", 0.330, 0.205, TOP + 0.030))), 1.0, True),
        "80 mm stack plus a spare pair stacked beside the pad, clear of its outline": (scene(*stack("white", "red"), ("blue", shim("blue", cx, cy - 0.056)), ("purple", shim("purple", cx, cy - 0.056, TOP + 0.030))), 1.0, True),
        "80 mm stack plus a spare pair yawed off the pad's corner, clear of its outline": (scene(*stack("white", "red"), ("blue", shim("blue", *corner, yaw=-math.pi / 4)), ("purple", shim("purple", *corner, TOP + 0.030, yaw=-math.pi / 4))), 1.0, True),
        "80 mm stack plus a spare on a spare, overhanging the pad by 3 mm": (scene(*stack("white", "red"), ("blue", shim("blue", cx, cy - 0.056)), ("purple", shim("purple", cx, cy - 0.052, TOP + 0.030))), 0.0, False),
        "80 mm stack plus the purple shim leaning on it from the table": (scene(*stack("white", "red"), ("purple", shim("purple", cx, cy - 0.050, tilt=math.radians(30)))), 1.0, True),
        "80 mm stack plus the purple shim leaning on it from the pad": (scene(*stack("white", "red"), ("purple", shim("purple", cx, cy - 0.040, PAD_TOP, tilt=math.radians(30)))), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    # The height ladder on its own, at and 0.3 mm past each exact limit.
    for err, want in ((0.0, 1.0), (0.002, 1.0), (0.0023, 0.5), (0.007, 0.5), (0.0073, 0.25), (0.012, 0.25), (0.0123, 0.0)):
        assert _ladder(err) == want, (err, _ladder(err), want)
    # The outline test at the pad's corner: a shim yawed 45 degrees is clear of the pad 30 mm out along each axis
    # (its corners' bounding box would still overlap) and over it 18 mm out.
    assert not _over_pad(shim("blue", *corner, yaw=-math.pi / 4), THICK["shim_blue"])
    assert _over_pad(shim("blue", cx + PAD_SIZE[0] / 2 + 0.018, cy - PAD_SIZE[1] / 2 - 0.018, yaw=-math.pi / 4), THICK["shim_blue"])
    assert FEWEST == 2 and sorted(len(s) for s in _SETS if abs(sum(s) - TARGET) < 1e-9) == [2, 3]
    # No single shim, flat or not, and no flat pair of other shims can reach 80 mm by accident.
    sums = sorted({round(sum(t for t, keep in zip(THICK.values(), mask) if keep) * 1000) for mask in np.ndindex(*(2,) * len(THICK))})
    assert 80 in sums and all(abs(s - 80) >= 5 or s == 80 for s in sums), sums
    assert max(THICK.values()) < min(SHIM_L, SHIM_W), "a shim's thickness must be its smallest dimension"
    assert max(SHIM_L, SHIM_W) < TARGET - LADDER[-1][0], "no single shim, on any face, may earn credit alone"
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
