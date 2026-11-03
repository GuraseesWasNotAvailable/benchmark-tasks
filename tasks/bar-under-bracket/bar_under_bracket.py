"""On the station table, in the left arm's lane, a 1 g bench analogue of fitting a pre-formed rigid line (small-bore
tubing, semi-rigid coax or a bus bar) on a spacecraft panel. The run has to pass beneath a fixed standoff bracket, so
it cannot be set down from above: a black square bar must travel along the channel through the 55 mm tunnel under the
bracket's plate, then drop between the posts of two white saddle clips, one either side of the bracket, with its
leading end registered within 8 mm of a red end stop that stands in for the fitting. The score climbs with how far
along that route the bar got: partly under the plate, through it, clipped, registered. Every test uses the bar's
centre and quaternion (its centreline and end points), never its axis-aligned bounds.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/bar-under-bracket/bar_under_bracket.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Route the black bar under the grey bracket so it lies flat between the posts of both white clips, with its end within 8 mm of the red stop." \\
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
LINE_X = 0.220  # the channel line, running along y
BAR = (0.016, 0.260, 0.016)  # 16 mm square tube, long axis along y
BAR_MASS = 0.042
BAR_START = (0.300, 0.360)  # parallel to the channel, 80 mm further from the arms, spanning y 0.230 to 0.490

PLATE = (0.100, 0.060, 0.010)  # the bracket's plate, spanning x 0.170 to 0.270 and y 0.160 to 0.220
PLATE_Y = (0.160, 0.220)
CLEAR_H = 0.055  # the plate's underside above the table
LEG = (0.010, 0.060, CLEAR_H)  # two legs under the plate's x edges; the tunnel between them is 80 mm wide
TUNNEL_X = (0.180, 0.260)  # the legs' inner faces
BRACKET_Y = 0.190

POST = (0.010, 0.015, 0.012)  # clip posts: 10 mm thick across the channel, 15 mm along it, 12 mm tall
CLIP_GAP = 0.026  # between a clip's posts: 5 mm of play per side around the 16 mm bar
CLIP_YS = (0.120, 0.230)  # clip A between the stop and the bracket, clip B just past the bracket
STOP = (0.030, 0.005, 0.012)  # the end stop, 30 mm across the channel, 5 mm thick along it, 12 mm tall
STOP_Y = 0.055
STOP_FACE = STOP_Y + STOP[1] / 2  # 0.0575, the face the bar's end registers against

GREY = (0.6, 0.6, 0.6)
WHITE = (0.92, 0.92, 0.9)
RED = (0.8, 0.15, 0.1)
BLACK = (0.05, 0.05, 0.05)

BAR_BOX = Box("bar", pos=(BAR_START[0], BAR_START[1], TOP + BAR[2] / 2), size=BAR, mass=BAR_MASS, color=BLACK)
BRACKET = [
    Box("bracket_plate", pos=(LINE_X, BRACKET_Y, TOP + CLEAR_H + PLATE[2] / 2), size=PLATE, color=GREY),
    Box("bracket_leg_near", pos=(TUNNEL_X[0] - LEG[0] / 2, BRACKET_Y, TOP + LEG[2] / 2), size=LEG, color=GREY),
    Box("bracket_leg_far", pos=(TUNNEL_X[1] + LEG[0] / 2, BRACKET_Y, TOP + LEG[2] / 2), size=LEG, color=GREY),
]
CLIPS = [
    Box(f"clip_{tag}_{side}", pos=(LINE_X + sign * (CLIP_GAP / 2 + POST[0] / 2), y, TOP + POST[2] / 2), size=POST, color=WHITE)
    for tag, y in zip("ab", CLIP_YS)
    for side, sign in (("near", -1), ("far", 1))
]
STOP_BOX = Box("end_stop", pos=(LINE_X, STOP_Y, TOP + STOP[2] / 2), size=STOP, color=RED)
OBJECTS = [BAR_BOX, *BRACKET, *CLIPS, STOP_BOX]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

FLAT_TOL = 0.003  # each end's underside to the table top
FACE_TOL_DEG = 10.0  # a face down: the bar rolled less than this from lying on one of its four long faces
PROP_MAX = 0.025  # through but not flat: both ends' undersides at most this high keeps the whole bar under the plate
CLIP_TOL = CLIP_GAP / 2 - BAR[0] / 2 + 0.001  # 6 mm: the 5 mm of play plus 1 mm for the solver
END_GAP = 0.008  # the leading end face to the stop face
PENETRATION = 0.002  # the solver may report the bar this far into the stop; it still counts as touching
PARTIAL_MIN = 0.020  # the least length of the plate's span the bar must lie under for the first rung
EPS = 1e-4  # 0.1 mm, so the exact stated limits count
SEAT_GAP = 0.004  # the demo's nominal end gap
SEAT_Y = STOP_FACE + SEAT_GAP + BAR[1] / 2  # 0.1915, the seated centre


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _geometry(state):
    """The bar's centreline end points in the table plane, the underside height under each end, and its nearest
    point to the stop along y, all from the centre and the quaternion."""
    r = _rot(state.quat_xyzw)
    c = np.asarray(state.pos, dtype=float)
    u = r[:, 1] * BAR[1] / 2  # half the long axis
    sink = abs(r[2, 0]) * BAR[0] / 2 + abs(r[2, 2]) * BAR[2] / 2  # end-face centre to its lowest point
    ends = (c + u, c - u)
    bottoms = [float(e[2]) - sink for e in ends]
    reach_y = abs(r[1, 0]) * BAR[0] / 2 + abs(r[1, 1]) * BAR[1] / 2 + abs(r[1, 2]) * BAR[2] / 2
    lead_y = float(c[1]) - reach_y
    return [(float(e[0]), float(e[1])) for e in ends], bottoms, lead_y


def _x_at(seg, y):
    """The centreline's x where it crosses the line at ``y``, or None if the bar does not reach it."""
    (x1, y1), (x2, y2) = seg
    if abs(y2 - y1) < 1e-9:
        return None
    t = (y - y1) / (y2 - y1)
    if t < -1e-9 or t > 1 + 1e-9:
        return None
    return x1 + t * (x2 - x1)


def _under_plate(seg) -> float:
    """Length of the plate's 60 mm span (along y) over which the centreline lies inside the tunnel, 0 if any part of
    that overlap runs outside the legs. The centreline is straight, so testing both ends of the overlap suffices."""
    lo = max(min(seg[0][1], seg[1][1]), PLATE_Y[0])
    hi = min(max(seg[0][1], seg[1][1]), PLATE_Y[1])
    if hi - lo <= 0:
        return 0.0
    for y in (lo, hi):
        x = _x_at(seg, y)
        if x is None or not TUNNEL_X[0] < x < TUNNEL_X[1]:
            return 0.0
    return hi - lo


def _face_down(state) -> bool:
    """One of the four long faces is down: the bar's local x or z axis within 10 degrees of vertical, so it is not
    balanced or leaning on an edge."""
    r = _rot(state.quat_xyzw)
    return max(abs(r[2, 0]), abs(r[2, 2])) >= math.cos(math.radians(FACE_TOL_DEG))


def _flat(state, bottoms) -> bool:
    """At rest, a face down, both ends' undersides within 3 mm of the table top: not propped on a post, a leg or the
    stop, and not on an edge."""
    return bool(state.resting() and _face_down(state) and all(abs(b - TOP) <= FLAT_TOL + EPS for b in bottoms))


def _low(state, bottoms) -> bool:
    """At rest with both ends' undersides at most 25 mm above the table top. The underside is straight, so the bar's
    top stays under 25 + 23 mm everywhere, below the plate's 55 mm underside: whatever it rests on, it is not on the
    plate."""
    return bool(state.resting() and all(-FLAT_TOL - EPS <= b - TOP <= PROP_MAX + EPS for b in bottoms))


def _clipped(seg) -> bool:
    """The centreline crosses both clip lines within 6 mm of the channel line: between the posts of both clips."""
    for y in CLIP_YS:
        x = _x_at(seg, y)
        if x is None or abs(x - LINE_X) > CLIP_TOL + EPS:
            return False
    return True


def _registered(lead_y) -> bool:
    gap = lead_y - STOP_FACE
    return -PENETRATION <= gap <= END_GAP + EPS


def score(objects) -> float:
    """1.0 flat, through the tunnel, between both clips' posts and registered at the stop; 0.75 the same with the end
    more than 8 mm from the stop; 0.5 flat and through the tunnel but not between both clips' posts; 0.25 flat with at
    least 20 mm but not all 60 mm of the plate's span over the bar, or through the tunnel and at rest but not flat with
    both ends at most 25 mm up (an end on a clip post or the stop, or the bar on an edge); 0 for everything else."""
    bar = objects["bar"]
    seg, bottoms, lead_y = _geometry(bar)
    under = _under_plate(seg)
    through = under >= PLATE_Y[1] - PLATE_Y[0] - EPS
    if not _flat(bar, bottoms):
        return 0.25 if through and _low(bar, bottoms) else 0.0
    if through:
        if _clipped(seg):
            return 1.0 if _registered(lead_y) else 0.75
        return 0.5
    if under >= PARTIAL_MIN - EPS:
        return 0.25
    return 0.0


def success(objects) -> bool:
    """The full-score state, at rest; holds for 1 s."""
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Metres from the bar's centre to its seated centre."""
    bar = objects["bar"]
    return math.hypot(float(bar.pos[0]) - LINE_X, float(bar.pos[1]) - SEAT_Y)


TRAVEL = TOP + 0.140
YAW = math.pi / 2  # fingers close along x on the bar's 16 mm side faces; -90 degrees would pin joint6 at its limit
# Measured in the simulator: holding the bar, the hand and grip settle about 2 degrees past the commanded yaw (joint 6
# lags about 1.6 degrees; the rest is the bar turning in the grip).
WRIST_TWIST = 0.034
YAW_HELD = YAW - WRIST_TWIST / 2  # from the lift on: the bar is about 1 degree off the channel with the twist or without it
TCP_ABOVE_BOTTOM = 0.018  # TCP over the bar's underside: pad tips 5 mm above the table when it lies there
GRIP_Z = TOP + TCP_ABOVE_BOTTOM  # pads overlap the 16 mm bar by 11 mm
SLIDE_Z = TOP + 0.025 + TCP_ABOVE_BOTTOM  # bar bottom 13 mm over the 12 mm posts and stop, top 14 mm under the plate
PLACE_Z = TOP + 0.005 + TCP_ABOVE_BOTTOM  # bar bottom 5 mm above the table, 7 mm down between the clip posts
HOLD_FROM_END = 0.020  # the grip centre from the bar's trailing end
PICK = (BAR_START[0], BAR_START[1] + BAR[1] / 2 - HOLD_FROM_END)  # (0.300, 0.470)
SEAT_TCP_Y = SEAT_Y + BAR[1] / 2 - HOLD_FROM_END  # 0.3015: the fingers stay at least 52 mm from clip B's posts
LEAD_IN = 0.250  # the leading end's y while the bar is lowered to the slide height, 30 mm past the plate's edge
START_TCP_Y = LEAD_IN + BAR[1] - HOLD_FROM_END  # 0.490
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the bar
GRIP = -(BAR[0] - SQUEEZE) / 2  # finger joint target: each finger pushes 4 mm x 5000 N/m = 20 N, under its 50 N cap


def demo():
    """Left arm only, yaw +90 degrees. Grip the bar 20 mm from its trailing end, lift it, carry it over the channel
    with its leading end 30 mm short of the bracket, lower it to the slide height, slide it 188 mm along -y under the
    plate and over both clips until its end is 4 mm short of the stop, then lower it slowly between the clip posts
    and release it 5 mm above the table. The arm settles before every close, slide, set-down and release, because
    the simulated wrist trails a moving target by several millimetres. While it holds the bar, the hand and grip also
    settle about 2 degrees past the commanded yaw (joint 6 lags about 1.6 degrees; the rest is the bar turning in the
    grip); from the lift on the demo commands half of that less, so the bar lies within about 1 degree of the channel
    and clears clip A's posts by about 2 mm whether the wrist twists or not. 20.3 s of plan, then the 3 s hold: 466
    steps at 20 Hz."""
    px, py = PICK
    over = (LINE_X, START_TCP_Y)
    seat = (LINE_X, SEAT_TCP_Y)
    left = [
        Waypoint(1.5, (px, py, TRAVEL), FINGER_OPEN, "to the bar", YAW),
        Waypoint(1.5, (px, py, GRIP_Z), FINGER_OPEN, "descend", YAW),
        Waypoint(0.6, (px, py, GRIP_Z), FINGER_OPEN, "settle", YAW),
        Waypoint(1.5, (px, py, GRIP_Z), GRIP, "close", YAW),
        Waypoint(1.2, (px, py, TRAVEL), GRIP, "lift", YAW_HELD),
        Waypoint(1.8, (*over, TRAVEL), GRIP, "carry over the channel", YAW_HELD),
        Waypoint(0.6, (*over, TRAVEL), GRIP, "settle over the channel", YAW_HELD),
        Waypoint(1.5, (*over, SLIDE_Z), GRIP, "lower to the slide height", YAW_HELD),
        Waypoint(0.8, (*over, SLIDE_Z), GRIP, "settle at the slide height", YAW_HELD),
        Waypoint(3.0, (*seat, SLIDE_Z), GRIP, "slide under the bracket", YAW_HELD),
        Waypoint(1.0, (*seat, SLIDE_Z), GRIP, "settle over the clips", YAW_HELD),
        Waypoint(2.0, (*seat, PLACE_Z), GRIP, "set in", YAW_HELD),
        Waypoint(0.8, (*seat, PLACE_Z), GRIP, "settle", YAW_HELD),
        Waypoint(1.5, (*seat, PLACE_Z), FINGER_OPEN, "release", YAW_HELD),
        Waypoint(1.0, (*seat, TRAVEL), FINGER_OPEN, "retreat", YAW_HELD),
    ]
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw=0.0, pitch=0.0, roll=0.0):  # yaw about z, then pitch about the bar's x (end up), then roll about its long axis
        r = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        r = r @ np.array([[1, 0, 0], [0, math.cos(pitch), -math.sin(pitch)], [0, math.sin(pitch), math.cos(pitch)]])
        r = r @ np.array([[math.cos(roll), 0, math.sin(roll)], [0, 1, 0], [-math.sin(roll), 0, math.cos(roll)]])
        w = math.sqrt(max(1e-12, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
        return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])

    def bar(x, y, bottom=TOP, yaw=0.0, pitch=0.0, roll=0.0, vel=(0.0, 0.0, 0.0)):
        q = quat(yaw, pitch, roll)
        half = np.abs(_rot(q)) @ (np.asarray(BAR) / 2)
        pos = np.array([x, y, bottom + half[2]])
        return {"bar": ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)}

    def lead(gap, x=LINE_X, **kw):  # along the channel with the leading end ``gap`` from the stop face
        return bar(x, STOP_FACE + gap + BAR[1] / 2, **kw)

    # A negative pitch raises the leading (-y) end. Leading underside edge on the stop's 12 mm top, trailing end on
    # the table:
    tilt_stop = -math.asin(STOP[2] / BAR[1])
    # Between clip B's posts, yawed 9.5 degrees so its underside rests on the top of clip A's near post, trailing end
    # on the table, leading end hanging 16 mm up beside the stop (pitch solved so the underside just touches the post):
    yaw_b, pitch_b = math.radians(9.5), -0.0625
    yaw_a = math.radians(20)  # centreline through clip B's centre, passing outside clip A's far post
    cases = {
        "start": (bar(*BAR_START), 0.0, False),
        "seated, 4 mm from the stop": (lead(SEAT_GAP), 1.0, True),
        "seated, touching the stop": (lead(0.0), 1.0, True),
        "seated, 1.5 mm of solver penetration into the stop": (lead(-0.0015), 1.0, True),
        "an exact 8.0 mm gap is registered": (lead(0.008), 1.0, True),
        "a 9 mm gap is clipped only": (lead(0.009), 0.75, False),
        "clipped, end 30 mm short of the stop": (lead(0.030), 0.75, False),
        "seated, reversed end for end": (lead(SEAT_GAP, yaw=math.pi), 1.0, True),
        "seated, rolled 90 degrees about its long axis": (lead(SEAT_GAP, roll=math.pi / 2), 1.0, True),
        "seated, rolled 8 degrees, still face down": (lead(SEAT_GAP, roll=math.radians(8)), 1.0, True),
        "seated, 5 mm off the channel line": (lead(SEAT_GAP, x=LINE_X + 0.005), 1.0, True),
        "seated, yawed 1 degree": (lead(SEAT_GAP, yaw=math.radians(1)), 1.0, True),
        "through the tunnel, end short of clip A": (lead(0.080), 0.5, False),
        "through the tunnel beside both clips' near posts": (bar(0.1885, 0.180), 0.5, False),
        "through the tunnel beside both clips' far posts": (bar(0.2515, 0.180), 0.5, False),
        "through the tunnel, yawed 20 degrees, in clip B but outside clip A's far post": (
            bar(LINE_X + (CLIP_YS[1] - 0.212) * math.tan(yaw_a), 0.212, yaw=yaw_a), 0.5, False),
        "partly under: 30 mm of the plate's span": (bar(LINE_X, PLATE_Y[1] - 0.030 + BAR[1] / 2), 0.25, False),
        "partly under: exactly 20 mm": (bar(LINE_X, PLATE_Y[1] - 0.020 + BAR[1] / 2), 0.25, False),
        "partly under: 15 mm earns nothing": (bar(LINE_X, PLATE_Y[1] - 0.015 + BAR[1] / 2), 0.0, False),
        "on the channel line short of the bracket": (bar(LINE_X, PLATE_Y[1] + 0.005 + BAR[1] / 2), 0.0, False),
        "lying beside the bracket outside the legs": (bar(0.300, 0.190), 0.0, False),
        "on top of the plate": (lead(SEAT_GAP, bottom=TOP + CLEAR_H + PLATE[2]), 0.0, False),
        "through, leading end propped on the stop": (lead(-0.003, pitch=tilt_stop), 0.25, False),
        "through, in clip B but resting on clip A's near post, leading end 16 mm up": (
            bar(LINE_X - (CLIP_YS[1] - 0.190) * math.tan(yaw_b), 0.190, yaw=-yaw_b, pitch=pitch_b), 0.25, False),
        "seated pose, leading end 4 mm up: through, not flat": (lead(SEAT_GAP, pitch=-math.asin(0.004 / BAR[1])), 0.25, False),
        "seated pose, leading end 2.5 mm up still flat": (lead(SEAT_GAP, pitch=-math.asin(0.0025 / BAR[1])), 1.0, True),
        "seated but rolled 47 degrees onto an edge, leaning on a post": (
            lead(SEAT_GAP, x=0.2183, roll=math.radians(47)), 0.25, False),
        "balanced on an edge at the start, beside the bracket": (bar(*BAR_START, roll=math.pi / 4), 0.0, False),
        "carried 10 mm above the seat, moving": (lead(SEAT_GAP, bottom=TOP + 0.010, vel=(0, -0.03, 0)), 0.0, False),
        "seated but still sliding": (lead(SEAT_GAP, vel=(0, -0.02, 0)), 0.0, False),
        "standing on end": (bar(LINE_X, 0.300, pitch=math.pi / 2), 0.0, False),
        "off the table": (bar(0.300, 0.360, bottom=0.0), 0.0, False),
        "across the channel at 90 degrees, past clip B": (bar(0.320, 0.300, yaw=math.pi / 2), 0.0, False),
        "through the tunnel but held 30 mm up": (lead(SEAT_GAP, bottom=TOP + 0.030), 0.0, False),
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
