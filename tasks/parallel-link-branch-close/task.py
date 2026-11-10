# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Refit a desktop arm's parallel link on the closure branch that keeps its wrist level.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

A bench model of the level-keeping four-bar of a desktop parallelogram arm (110 mm links) stands on the table, its
plane facing the robot (every hinge axis points at the robot):

- a grey steel base, fixed, with the arm pivot A 100 mm above the table at (0.32, 0.00) and, 53 mm to the robot's
  right of it at the same height, a red steel pin B (8 mm) sticking 24 mm out toward the robot;
- a yellow upper arm, 110 mm pivot to pivot, held by its brake at 65 deg above horizontal, reaching up and to the
  robot's left to its tip C;
- a blue wrist plate (122 x 20 x 10 mm), pinned at its middle to the arm tip C and balanced there;
- a grey parallel link, 110 mm pin to pin, already pinned to the plate's rear end D. Its free end is a snap eye: a
  pocket with a sprung steel tongue across its mouth that clicks shut behind the red pin.

With the link off its pin the plate is free: the link's weight hangs the plate's rear end straight down, so the plate
stands on end, 90 deg off level. Closing the loop A-C-D-B fixes everything (the arm is braked), and there are two ways
to close it. With the plate turned level first (rear end pointing back), the link swings 25 deg onto the pin and the
loop is a parallelogram: the plate stays level. Swung straight up onto the pin from where it hangs (65 deg), the link
closes the crossed loop (an anti-parallelogram): the snap clicks, but the plate is locked 90 deg off level.

Goal: the link's eye snapped onto the red pin with the blue plate level (within 3 deg), both grippers let go.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import linkkit as lk  # noqa: E402
import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
VARIANT = os.environ.get("DEMO_VARIANT", "")
MM = 0.001

# -- geometry (linkage frame, m): origin at the arm pivot A; x in the plane toward the arm's reach (world +y), y the
# hinge axis toward the robot (world -x), z up. Positive joint motion about +y turns x toward -z (clockwise as the
# robot sees it). ---------------------------------------------------------------------------------------------------
A_POS = (0.32, 0.0, TOP + 0.100)
YAW = 90.0
FRAME = np.eye(4)
FRAME[:3, :3] = lk.axis_rotation((0, 0, 1), math.radians(YAW))
FRAME[:3, 3] = A_POS

L = 0.110  # upper arm and parallel link, pivot to pivot
ARM_DEG = 65.0  # the arm's braked angle above horizontal
# Base pivot spacing = plate lug length. G = L (sin a - cos a) puts the crossed closure's lug straight below C, which
# is exactly where the link's weight hangs it with the link unpinned.
G = L * (math.sin(math.radians(ARM_DEG)) - math.cos(math.radians(ARM_DEG)))
C = np.array([L * math.cos(math.radians(ARM_DEG)), 0.0, L * math.sin(math.radians(ARM_DEG))])
B = np.array([-G, 0.0, 0.0])  # red pin axis
D_P = C + [-G, 0.0, 0.0]  # plate's rear pin, parallelogram (plate level)
D_X = C + [0.0, 0.0, -G]  # plate's rear pin, crossed closure = where it hangs
LINK_DEG = -(180.0 - ARM_DEG)  # the link's in-plane direction in the parallelogram (parallel to the arm): -115 deg
CROSS_LINK_DEG = math.degrees(math.atan2(B[2] - D_X[2], B[0] - D_X[0]))  # -155 deg

Y_ARM, Y_PLATE, Y_LINK = 0.0, 0.013, 0.025  # layer centres along the hinge axis (arm 12 mm, plate 10, link 8 thick)
PIN_R = 0.004
X_END = L - 0.0047  # eye's inner wall face (link frame x): the loaded pin rests 0.7 mm short of L, about 0.8 deg tilt
X_OUT = L + 0.0055  # eye's outer wall face
Z_POCKET = 0.007  # pocket half-height (along the swing): the tongue clicks in when the pin is 3 mm from centre
TONGUE_TIP = L - 0.0025

GREY = (0.42, 0.43, 0.45)
YELLOW = (0.95, 0.72, 0.10)
BLUE = (0.15, 0.35, 0.80)
LIGHT_GREY = (0.72, 0.74, 0.77)
RED = (0.85, 0.10, 0.10)
BLACK = (0.12, 0.12, 0.13)

_t = lk.Tree("pal", pos=A_POS, yaw_deg=YAW, fixed=True, color=GREY)
# base (root, fixed to the table): back block behind the arm's layer, a post under the red pin, the red pin
_t.link("base", [
    lk.box((0.090, 0.022, 0.112), (-0.025, -0.019, -0.044)),  # back block, top 12 mm above A
    lk.box((0.024, 0.020, 0.110), (-G, 0.002, -0.045)),  # post under B, front face at y 12 mm
    lk.box((0.120, 0.052, 0.008), (-0.025, -0.014, -0.096)),  # foot plate on the table
    lk.capsule(PIN_R, 0.024, (-G, 0.024, 0.0), axis="Y", color=RED),  # red pin, y 12 to 36 mm (+4 mm rounded ends)
], mass=1.5, friction=(0.15, 0.10), contact_offset=0.001)
# upper arm: link frame x along the arm
_t.link("arm", [
    lk.box((L + 0.016, 0.012, 0.016), (L / 2, 0.0, 0.0)),
    lk.cylinder(0.004, 0.018, (L, 0.009, 0.0), axis="Y", color=GREY),  # C pin through the plate
], mass=0.080, at=(0, 0, 0), rpy_deg=(0, -ARM_DEG, 0), color=YELLOW, contact_offset=0.001)
# wrist plate: bar centred on C, balanced about it (centre of mass on the pivot)
_t.link("plate", [lk.box((2 * G + 0.016, 0.010, 0.020), (0.0, Y_PLATE, 0.0))], mass=0.066, at=tuple(C),
        color=BLUE, friction=(0.6, 0.5), contact_offset=0.001, com=(0.0, Y_PLATE, 0.0))
# parallel link: link frame at D_P, x along the link toward the eye (-115 deg), z = x turned +90 deg in the plane;
# the eye's mouth faces -z (the side that leads when the link swings clockwise onto the pin)
_t.link("link", [
    lk.cylinder(0.007, 0.008, (0.0, Y_LINK, 0.0), axis="Y"),  # D boss
    lk.box((X_END - 0.004 + 0.007, 0.008, 0.012), ((X_END - 0.004 - 0.007) / 2, Y_LINK, 0.0)),  # bar
    lk.box((0.004, 0.008, 0.016), (X_END - 0.002, Y_LINK, 0.003)),  # eye's inner wall, z -5 to 11 mm
    lk.box((X_OUT + 0.004 - X_END + 0.004, 0.008, 0.004), ((X_OUT + X_END) / 2, Y_LINK, Z_POCKET + 0.002)),  # roof
    lk.box((0.004, 0.008, 2 * Z_POCKET), (X_OUT + 0.002, Y_LINK, 0.0)),  # outer wall
    lk.box((L + 0.025 - X_OUT, 0.008, 0.010), ((L + 0.025 + X_OUT) / 2, Y_LINK, -Z_POCKET - 0.005)),  # tongue housing
], mass=0.040, at=tuple(D_P), rpy_deg=(0, -LINK_DEG, 0), color=LIGHT_GREY, friction=(0.3, 0.25),
        contact_offset=0.001)
# tongue: slides along the link's x (+ retracts toward the housing); flat top face holds the pin, 45 deg lead-in
_t.link("tongue", [
    lk.box((L + 0.014 - TONGUE_TIP, 0.008, 0.0015), ((L + 0.014 + TONGUE_TIP) / 2, Y_LINK, -Z_POCKET - 0.00075)),
    lk.box((0.010, 0.008, 0.0065), (L + 0.009, Y_LINK, -0.01175)),
    lk.box((0.00919, 0.008, 0.0015), (L + 0.00128, Y_LINK, -0.01122), rpy_deg=(0, 45, 0)),  # lead-in ramp
], mass=0.003, at=tuple(D_P), rpy_deg=(0, -LINK_DEG, 0), color=BLACK, friction=(0.15, 0.10), contact_offset=0.001)

_t.revolute("shoulder", "base", "arm", anchor=(0, 0, 0), axis=(0, 1, 0), init_deg=0.0, damping=0.01,
            brake=lk.Brake(stiffness=200.0, damping=2.0, engaged=True))
_t.revolute("wrist", "arm", "plate", anchor=tuple(C), axis=(0, 1, 0), init_deg=-90.0, damping=0.003)
# hanging: the plate turned -90 deg (rear end down), the link straight down = 25 deg back from its design direction
_t.revolute("lug", "plate", "link", anchor=tuple(D_P), axis=(0, 1, 0), init_deg=180.0 + LINK_DEG, damping=0.003)
LINK_AXIS = (math.cos(math.radians(LINK_DEG)), 0.0, math.sin(math.radians(LINK_DEG)))
_t.prismatic("latch", "link", "tongue", anchor=tuple(D_P + L * np.array(LINK_AXIS)), axis=LINK_AXIS,
             limits=(0.0, 0.010), rest=-0.003, stiffness=40.0, damping=0.1)
PAL = _t.build()

OBJECTS = [PAL]

# -- scoring -------------------------------------------------------------------------------------------------------
# Everything is measured from link poses (what a camera, a protractor and a feeler see), every control step from reset.
FRAME_R = FRAME[:3, :3]
B_WORLD = FRAME_R @ B + FRAME[:3, 3]
LEVEL_DEG = 3.0  # plate within 3 deg of level
BRAKE_GATE_DEG = 1.5  # the upper arm forced this far off its braked angle at any time -> 0
NEAR_MM = 10.0  # "eye at the pin": pocket centre within 10 mm of the pin's axis
EX_RANGE = ((X_END - L + PIN_R) * 1000 - 0.3, (X_OUT - L - PIN_R) * 1000 + 0.3)  # pin centre inside the pocket, mm
EZ_MAX = (Z_POCKET - PIN_R) * 1000 + 0.3
LATCH_HOME_MM = 1.0  # tongue back within 1 mm of its stop
CLEAR_M, TCP_STILL = 0.030, 0.005
PLATE_STILL, PLATE_SPIN = 0.003, math.radians(3.0)
STILL_S = RELEASE_S = 1.0
EPS = 1e-6


def wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def plane_deg(rot_world, local=(1.0, 0.0, 0.0)) -> float:
    """In-plane direction (deg, linkage frame x toward z) of a link's local axis: the arm 65, the plate 0 when level
    (+90 standing on its rear end), the link -115 in the parallelogram and -155 in the crossed closure."""
    v = FRAME_R.T @ (np.asarray(rot_world, dtype=float) @ np.asarray(local, dtype=float))
    return math.degrees(math.atan2(float(v[2]), float(v[0])))


def eye_reading(poses) -> tuple[float, float, float]:
    """The red pin's centre in the link's eye (mm from the pocket centre along the link and across it) and the
    tongue's retraction (mm), from the link and tongue poses."""
    Lk, Tg = poses["link"], poses["tongue"]
    p = Lk[:3, :3].T @ (B_WORLD - Lk[:3, 3])
    latch = float((Tg[:3, 3] - Lk[:3, 3]) @ Lk[:3, 0])
    return 1000.0 * (float(p[0]) - L), 1000.0 * float(p[2]), 1000.0 * latch


def is_snapped(ex: float, ez: float, latch: float) -> bool:
    return EX_RANGE[0] <= ex <= EX_RANGE[1] and abs(ez) <= EZ_MAX and latch <= LATCH_HOME_MM


def measure(pal, tools) -> dict:
    """One sample of everything the scorer uses (deg, mm, m/s). Plate speed and spin are filled in from its pose
    change between control steps by the on-step record."""
    P = pal.poses
    ex, ez, latch = eye_reading(P)
    return {
        "arm_dev": wrap(plane_deg(P["arm"][:3, :3]) - ARM_DEG),
        "tilt": wrap(plane_deg(P["plate"][:3, :3])),
        "link_deg": plane_deg(P["link"][:3, :3]),
        "ex": ex, "ez": ez, "latch": latch,
        "eye_gap": math.hypot(ex, ez),
        "snapped": is_snapped(ex, ez, latch),
        "plate_speed": 0.0, "plate_spin": 0.0,
        "clear": {s: lk.clearance(t, pal) for s, t in tools.items()},
        "tcp_speed": {s: float(t.speed) for s, t in tools.items()},
    }


def pose_rates(A_prev, A, dt: float) -> tuple[float, float]:
    """Speed (m/s) and spin (rad/s) of a body from its pose change over ``dt`` (what a camera sees)."""
    if dt <= 0.0:
        return 0.0, 0.0
    v = float(np.linalg.norm(A[:3, 3] - A_prev[:3, 3])) / dt
    R = A_prev[:3, :3].T @ A[:3, :3]
    sin = 0.5 * float(np.linalg.norm((R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1])))
    return v, math.atan2(sin, (float(np.trace(R)) - 1.0) / 2.0) / dt


def level(m) -> bool:
    return abs(m["tilt"]) <= LEVEL_DEG


class Record:
    """Whole-run record: a linkkit History updated every control step and cleared only by the simulator reset, plus
    the latest sample. Pure Python, so the tests can feed it synthetic samples."""

    def __init__(self):
        self.H = lk.History()
        self.last: dict | None = None
        self.n = 0
        self.trace: list = []

    def reset(self) -> None:
        self.H.reset()
        self.last, self.n, self.trace = None, 0, []

    def update(self, m: dict, t: float | None = None) -> None:
        """Feed one control step. ``t`` is only for offline tests (in the simulator the kit advances the clock)."""
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        H.ever("brake_forced", abs(m["arm_dev"]) >= BRAKE_GATE_DEG)
        H.peak("arm_dev_max", abs(m["arm_dev"]))
        H.ever("aligned", level(m) and m["eye_gap"] <= NEAR_MM)
        H.ever("snapped_level", level(m) and m["snapped"])
        H.ever("snapped_any", m["snapped"])
        H.held("plate_still", m["plate_speed"] < PLATE_STILL and m["plate_spin"] < PLATE_SPIN)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Score and reason. The gate first (any time in the run), then the rungs from the top."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "brake_forced" in H.evers:
        return 0.0, (f"gate: upper arm forced {H.peaks['arm_dev_max']:.1f} deg off its brake at "
                     f"{H.evers['brake_forced']:.2f} s (gate {BRAKE_GATE_DEG})")
    if m["snapped"] and level(m):
        ends = {
            "plate still 1 s": H.runs.get("plate_still", 0.0) >= STILL_S - EPS,
            "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS,
        }
        if all(ends.values()):
            return 1.0, f"snapped on the parallelogram, plate {m['tilt']:+.1f} deg, both grippers let go"
        return 0.75, "snapped with the plate level at the end; not yet: " + ", ".join(
            k for k, ok in ends.items() if not ok)
    if "snapped_level" in H.evers:
        return 0.5, "snapped with the plate level at some time, but not at the end"
    if "aligned" in H.evers:
        return 0.25, "the eye reached the pin with the plate level, but never snapped there"
    if "snapped_any" in H.evers:
        return 0.0, f"snapped on the crossed closure: plate {m['tilt']:+.1f} deg off level"
    return 0.0, "the eye never reached the pin with the plate level"


REC = Record()
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    pal = tick.states["pal"]
    m = measure(pal, tick.tools)
    A = np.array(pal.poses["plate"], dtype=float)
    if _PREV:
        m["plate_speed"], m["plate_spin"] = pose_rates(_PREV["A"], A, tick.t - _PREV["t"])
    _PREV.update(A=A, t=tick.t)
    REC.update(m)
    if REC.n % 5 == 0:  # diagnostics every 0.25 s
        REC.trace.append((round(tick.t, 2), round(m["tilt"], 2), round(m["link_deg"], 2), round(m["ex"], 2),
                          round(m["ez"], 2), round(m["latch"], 2), int(m["snapped"]), round(m["arm_dev"], 3),
                          [round(1000 * m["clear"][s]) for s in ("left", "right")]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
# linkdemo.py runs demo_script(ctx). It reads only what a camera could measure (link poses, the pin's place in the
# eye) and the robot's own grippers; never joint values, the brake or the record. Both grips come from the front
# (from the robot's side, along the hinge axes), pads closing across the bar, so the wrist roll turns the part.

VARIANTS = ("", "nothing", "short_swing", "plate_only", "force_level")
APP_L = np.array([1.0, 0.0, -0.15]) / math.hypot(1.0, 0.15)  # left approach: forward, 9 deg down
APP_R = np.array([1.0, 0.0, -0.25]) / math.hypot(1.0, 0.25)  # right approach: forward, 14 deg down
PLATE_GRASP = (0.8 * G, 0.010, 0.0)  # pad-tip point, plate frame: front half of the bar, 2 mm short of its back
LINK_GRASP = (0.78 * L, 0.022, 0.0)  # pad-tip point, link frame: 24 mm short of the eye, 1 mm short of its back
OPEN_L, SHUT_L = 0.044, 0.019  # plate bar 20 mm wide
OPEN_R, SHUT_R = 0.036, 0.011  # link bar 12 mm wide
TIP_PAST_TCP = 0.013
OVERSHOOT_DEG = 1.0  # swing the link 1 deg past the parallel (the pin stops on the pocket's far wall 1.6 deg past)
FLAGS: dict = {}
LOG: dict = {}


def _grip_rot(approach, closing) -> np.ndarray:
    z = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    x = np.asarray(closing, dtype=float)
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _seen_deg(seen, link: str) -> float:
    return plane_deg(seen.pose("pal", link)[:3, :3])


def _seen_eye(seen) -> tuple[float, float, float]:
    return eye_reading({"link": seen.pose("pal", "link"), "tongue": seen.pose("pal", "tongue")})


def grasp_and_turn(ctx, side, link, joint, local, approach, open_w, shut_w, target_deg, *, turn_s, tol=0.3,
                   fixes=3, tag=""):
    """Grip ``link`` from the front at ``local`` (pad tips), pads across the bar, then turn it about ``joint`` with
    the wrist following (closed loop on the seen in-plane angle) until it points at ``target_deg``. Stays gripped."""
    arm = ctx.arm(side)
    seen = ctx.observe()
    P = seen.pose("pal", link)
    tip = P[:3, :3] @ np.asarray(local, dtype=float) + P[:3, 3]
    rot = _grip_rot(approach, (0.0, -1.0, 0.0))  # the bar hangs vertical: pads close along world y
    tcp = tip - TIP_PAST_TCP * np.asarray(approach)
    yield from arm.move(tcp - 0.06 * np.asarray(approach), rot, width=open_w, duration=2.0)
    yield from arm.move(tcp, rot, duration=1.5)
    yield from arm.grip(shut_w, 0.8)
    yield from arm.wait(0.3)
    log = LOG.setdefault(f"turn_{tag or link}", [])
    for k in range(fixes + 1):
        seen = ctx.observe()
        err = _seen_deg(seen, link) - target_deg  # + turns clockwise (about +y), toward lower angles
        log.append(round(err, 2))
        if k and (abs(err) <= tol or abs(err) > 20.0):
            break
        hinge, axis = seen.axis("pal", joint)
        yield from arm.arc(hinge, axis, err, follow=1.0, duration=turn_s if k == 0 else max(0.8, abs(err) / 6.0))
        yield from arm.wait(0.3)


def let_go(arm, open_w):
    yield from arm.grip(open_w, 0.6)
    yield from arm.retreat(0.06, 1.2)
    yield from arm.move(arm.pos + [-0.03, 0.0, 0.05], duration=1.0)


def swing_link_on(ctx, target_deg, tag):
    """Right arm: grip the hanging link near its eye, swing it onto the red pin, check the snap from the seen pose
    (pin inside the eye, tongue home) and press 1 deg more if it has not clicked."""
    yield from grasp_and_turn(ctx, "right", "link", "lug", LINK_GRASP, APP_R, OPEN_R, SHUT_R, target_deg, turn_s=3.0,
                              tol=0.5, tag=tag)
    for k in range(3):
        yield from ctx.right.wait(0.4)
        ex, ez, latch = _seen_eye(ctx.observe())
        LOG.setdefault(f"eye_{tag}", []).append([round(ex, 2), round(ez, 2), round(latch, 2)])
        if is_snapped(ex, ez, latch) or k == 2:
            break
        hinge, axis = ctx.observe().axis("pal", "lug")
        yield from ctx.right.arc(hinge, axis, 1.0, follow=1.0, duration=0.6)


def demo_script(ctx):
    FLAGS.clear()
    LOG.clear()
    yield from ctx.wait(0.5)
    seen = ctx.observe()
    LOG["start"] = {"plate_deg": round(_seen_deg(seen, "plate"), 2), "link_deg": round(_seen_deg(seen, "link"), 2)}
    v = VARIANT
    if v == "nothing":
        yield from ctx.wait(3.0)
        return
    if v in ("short_swing", "force_level"):  # the natural plan: swing the hanging link straight onto the pin
        yield from swing_link_on(ctx, CROSS_LINK_DEG - OVERSHOOT_DEG, "crossed")
        yield from let_go(ctx.right, OPEN_R)
        if v == "force_level":  # then try to turn the locked plate level
            yield from grasp_and_turn(ctx, "left", "plate", "wrist", PLATE_GRASP, APP_L, OPEN_L, SHUT_L, 0.0,
                                      turn_s=4.0, fixes=1, tag="force")
            yield from ctx.left.wait(1.0)
            yield from let_go(ctx.left, OPEN_L)
        yield from ctx.wait(2.0)
        return
    if v == "plate_only":  # level the plate and let go, never fit the link
        yield from grasp_and_turn(ctx, "left", "plate", "wrist", PLATE_GRASP, APP_L, OPEN_L, SHUT_L, 0.0, turn_s=4.0)
        yield from ctx.left.wait(1.0)
        yield from let_go(ctx.left, OPEN_L)
        yield from ctx.wait(3.0)
        return

    def left():  # turn the plate level and hold it until the link is on
        yield from grasp_and_turn(ctx, "left", "plate", "wrist", PLATE_GRASP, APP_L, OPEN_L, SHUT_L, 0.0, turn_s=4.0)
        FLAGS["level"] = True
        while not FLAGS.get("right_done"):
            yield
        yield from ctx.left.wait(0.5)
        yield from let_go(ctx.left, OPEN_L)

    def right():  # once the plate is level, swing the link 25 deg onto the pin
        while not FLAGS.get("level"):
            yield
        yield from ctx.right.wait(1.0)  # the link swings while the plate turns; let it hang still
        yield from swing_link_on(ctx, LINK_DEG - OVERSHOOT_DEG, "parallel")
        yield from ctx.right.wait(0.3)
        yield from let_go(ctx.right, OPEN_R)
        FLAGS["right_done"] = True

    yield from ctx.both(left(), right())
    yield from ctx.wait(2.0)


def demo_report() -> dict:
    s, why = ladder(REC)
    m = REC.last or {}
    H = REC.H
    return {
        "score": s, "reason": why, "variant": VARIANT,
        "final": {k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items()
                  if k not in ("clear", "tcp_speed")},
        "clear_mm": {k: round(1000 * v, 1) for k, v in m.get("clear", {}).items()},
        "evers": H.evers, "runs": {k: round(v, 2) for k, v in H.runs.items()},
        "peaks": {k: round(v, 3) for k, v in H.peaks.items()},
        "log": LOG,
        "trace (t, tilt, link_deg, ex, ez, latch, snapped, arm_dev, clear_mm)": REC.trace,
    }
