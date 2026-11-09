# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Gurasees Singh Dhanoa
"""Passive range of motion for a patient manikin's right elbow.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A blue-grey vinyl bed pad, 64 x 66 x 2 cm, lies on the table from x 0.06 to 0.70 and y -0.25 to 0.41.
- On it lies a child-size nursing manikin in a pale green gown, face up, head at the far left (+y), its right side
  toward the robot. The torso, head, left arm (along its far side) and legs (running off the pad's -y edge onto the
  table) are fixed to the table. Its right arm lies straight on the pad, palm up, slightly away from the body:
  shoulder at (0.30, 0.21), elbow at (0.20, 0.04), the arm running toward the robot's right arm. The upper arm and
  forearm are skin-coloured; a light blue padded resting-hand splint (9 x 5 x 4 cm, flat-sided) covers the wrist and
  hand, ending at (0.09, -0.15).
- The shoulder and the elbow are hinges about the same horizontal axis, so the forearm can bend straight up toward
  the shoulder. The elbow has muscle tone: a spring pulls it back toward straight.

Goal: bend the elbow once to a right angle (forearm vertical), bring it back down flat onto the pad without letting it
fall, keep the elbow on the pad the whole time, and let go.
"""

from __future__ import annotations

import math
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

# -- geometry ---------------------------------------------------------------------------------------------------
# The arm runs along U (hand to shoulder); the shoulder and elbow turn about C; +angle lifts the wrist (C x -U = +z).
YAW_DEG = 60.0
U = np.array([math.cos(math.radians(YAW_DEG)), math.sin(math.radians(YAW_DEG)), 0.0])
C = np.array([-U[1], U[0], 0.0])
AXIS_H = 0.045  # arm axis above the table top
ELBOW_XY = (0.200, 0.041)
E = np.array([*ELBOW_XY, TOP + AXIS_H])  # elbow centre, world
UPPER_LEN, UPPER_R = 0.200, 0.025
FORE_CAP_LEN, FORE_R = 0.130, 0.022  # forearm capsule straight part, from the elbow
SPLINT = (0.090, 0.050, 0.040)  # along the forearm, across (along C), thickness
SPLINT_F, SPLINT_DZ = 0.170, -0.005  # splint centre: 17 cm from the elbow, 5 mm below the arm axis (rests on the pad)
GRASP_F = 0.185  # pinch point on the splint, from the elbow
UPPER_MASS, FORE_MASS = 0.25, 0.20  # kg
TONE, TONE_DAMPING = 1.0, 0.30  # elbow spring toward straight (N m/rad) and damping (N m s/rad)
PAD_TOP = TOP + 0.020
SKIN = (0.93, 0.76, 0.64)
SPLINT_BLUE = (0.55, 0.75, 0.92)
GOWN = (0.66, 0.82, 0.76)  # pale green patient gown and pyjama trousers
LEG_R, LEG_X, LEG_Y = 0.045, (0.42, 0.52), (-0.15, -0.52)  # legs: radius, centre lines (x), hip and foot ends (y)
LEG_DROP = math.atan2(0.020, -0.25 - LEG_Y[1])  # off the pad's edge (y -0.25) down to the table at the foot end

# -- objects ----------------------------------------------------------------------------------------------------
_bed = lk.Tree("bed_pad", pos=(0.38, 0.08, TOP + 0.010), fixed=True, color=(0.45, 0.55, 0.65))
_bed.link("pad", [lk.box((0.64, 0.66, 0.020))], mass=0.7, friction=(0.8, 0.7))
BED_PAD = _bed.build()


def _world_to_frame(p):
    """World point -> manikin frame (origin at the elbow's table point, x along U, y along C)."""
    d = np.asarray(p, dtype=float) - np.array([E[0], E[1], TOP])
    return (float(d @ U), float(d @ C), float(d[2]))


_t = lk.Tree("manikin", pos=(E[0], E[1], TOP), yaw_deg=YAW_DEG, fixed=True, color=SKIN, self_collision=False)
def _leg(x):
    """A leg along -y from the hip, resting on the pad and sloping off its edge onto the table."""
    yc = (LEG_Y[0] + LEG_Y[1]) / 2
    bottom = PAD_TOP + (yc + 0.25) * math.tan(LEG_DROP)
    return lk.capsule(LEG_R, LEG_Y[0] - LEG_Y[1], _world_to_frame((x, yc, bottom + LEG_R / math.cos(LEG_DROP))),
                      axis="X", rpy_deg=(0, -math.degrees(LEG_DROP), 90 - YAW_DEG), color=GOWN)


_t.link("torso", [  # fixed: torso, head, shoulders, the left arm and the legs
    lk.box((0.26, 0.44, 0.12), _world_to_frame((0.47, 0.06, TOP + 0.082)), rpy_deg=(0, 0, -YAW_DEG), color=GOWN),
    lk.box((0.05, 0.06, 0.05), _world_to_frame((0.32, 0.214, TOP + 0.047)), rpy_deg=(0, 0, -YAW_DEG)),
    lk.box((0.05, 0.06, 0.05), _world_to_frame((0.62, 0.214, TOP + 0.047)), rpy_deg=(0, 0, -YAW_DEG)),
    lk.capsule(0.072, 0.03, _world_to_frame((0.47, 0.37, TOP + 0.094)), axis="X", rpy_deg=(0, 0, 90 - YAW_DEG)),
    lk.capsule(0.024, 0.30, _world_to_frame((0.635, 0.06, PAD_TOP + 0.024)), axis="X", rpy_deg=(0, 0, 90 - YAW_DEG)),
    _leg(LEG_X[0]), _leg(LEG_X[1]),
], mass=3.0, friction=(1.0, 0.9))
_t.link("upper_arm", [lk.capsule(UPPER_R, UPPER_LEN, axis="X")], mass=UPPER_MASS,
        at=(UPPER_LEN / 2, 0.0, AXIS_H), friction=(1.0, 0.9))
_t.link("forearm", [
    lk.capsule(FORE_R, FORE_CAP_LEN, (-FORE_CAP_LEN / 2, 0.0, 0.0), axis="X"),
    lk.box(SPLINT, (-SPLINT_F, 0.0, SPLINT_DZ), color=SPLINT_BLUE),
], mass=FORE_MASS, at=(0.0, 0.0, AXIS_H), com=(-0.110, 0.0, 0.0), friction=(1.0, 0.9))
_t.revolute("shoulder", "torso", "upper_arm", anchor=(UPPER_LEN, 0.0, AXIS_H), axis=(0, 1, 0), limits_deg=(-3, 45),
            damping=0.05, armature=0.001)
_t.revolute("elbow", "upper_arm", "forearm", anchor=(0.0, 0.0, AXIS_H), axis=(0, 1, 0), limits_deg=(-3, 140),
            stiffness=TONE, damping=TONE_DAMPING, armature=0.001)
MANIKIN = _t.build()

OBJECTS = [BED_PAD, MANIKIN]
ARM_LINKS = ("upper_arm", "forearm")
E_IN_UPPER = (-UPPER_LEN / 2, 0.0, 0.0)  # the elbow centre in the upper arm's frame
GRASP_IN_FORE = (-GRASP_F, 0.0, 0.0)  # the pinch point in the forearm's frame

# -- scoring rules ----------------------------------------------------------------------------------------------
LIFT_GATE = 0.015  # m: elbow centre ever more than this above its resting height -> 0
OVER_GATE = 110.0  # deg: elbow ever bent past this -> 0
DROP_RATE = -90.0  # deg/s: falling faster than this (from 15 deg or more) -> 0
DROP_FROM = 15.0
PEAK_MIN, RIGHT_LO, RIGHT_HI = 45.0, 80.0, 100.0
RETURNED_BELOW = 10.0
END_ANGLE, END_LOW, END_LIFT = 5.0, 0.005, 0.005
CLEAR = 0.030  # m from every pad point of each gripper to the upper arm and forearm
STILL_TCP, STILL_OMEGA, STILL_STEPS = 0.005, 3.0, 20


class Rom:
    """Whole-episode record of the elbow, fed every control step and cleared only at reset (pure Python)."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.n = 0
        self.hist: list[tuple[float, float]] = []  # last three (t, theta)
        self.peak = -math.inf
        self.max_lift = -math.inf
        self.over = False
        self.drop = False
        self.drop_at = None
        self.right_at = None  # step index of the first theta >= 80
        self.low_since = None  # step index from which theta has stayed <= 10 (after the right angle)
        self.was_back = False  # theta has been back to 10 deg or less after the right angle (kept to reset)
        self.second_rep = False  # bent again to 45 deg or more after that return (kept to reset)
        self.still = 0  # consecutive steps with every TCP and the forearm still
        self.prev_tcp: dict = {}
        self.theta = self.lift = self.low = self.omega = 0.0
        self.clear: dict = {}
        self.tcp_speed: dict = {}
        self.trace: list = []

    def update(self, t: float, theta: float, lift: float, low: float, tcp: dict, clear: dict) -> None:
        """theta: elbow angle (deg); lift: elbow centre above rest (m); low: forearm's lowest point above the pad top
        (m); tcp: {side: TCP position}; clear: {side: smallest pad distance to the arm (m)}."""
        self.n += 1
        self.hist = (self.hist + [(float(t), float(theta))])[-3:]
        t0, th0 = self.hist[0]
        self.omega = (theta - th0) / (t - t0) if len(self.hist) == 3 and t > t0 else 0.0
        self.theta, self.lift, self.low, self.clear = float(theta), float(lift), float(low), dict(clear)
        self.peak = max(self.peak, theta)
        self.max_lift = max(self.max_lift, lift)
        self.over = self.over or theta > OVER_GATE
        if len(self.hist) == 3 and th0 >= DROP_FROM and self.omega < DROP_RATE and not self.drop:
            self.drop, self.drop_at = True, round(float(t), 3)
        if self.right_at is None and theta >= RIGHT_LO:
            self.right_at = self.n
        if self.right_at is not None:
            if theta <= RETURNED_BELOW:
                self.low_since = self.n if self.low_since is None else self.low_since
            else:
                self.low_since = None
        self.was_back = self.was_back or self.low_since is not None
        self.second_rep = self.second_rep or (self.was_back and theta >= PEAK_MIN)
        dt = t - self.hist[-2][0] if len(self.hist) >= 2 else 0.0
        speeds = {}
        for side, p in tcp.items():
            p = np.asarray(p, dtype=float)
            q = self.prev_tcp.get(side)
            speeds[side] = float(np.linalg.norm(p - q) / dt) if q is not None and dt > 0 else math.inf
            self.prev_tcp[side] = p
        self.tcp_speed = speeds
        quiet = bool(speeds) and all(v < STILL_TCP for v in speeds.values()) and abs(self.omega) < STILL_OMEGA
        self.still = self.still + 1 if quiet else 0

    @property
    def returned(self) -> bool:
        return self.low_since is not None


def ladder(r: Rom) -> tuple[float, str]:
    """Score and the rung's reason. Gates first (any time in the run), then the ladder."""
    if r.n == 0:
        return 0.0, "no steps yet"
    if r.max_lift > LIFT_GATE:
        return 0.0, f"elbow lifted {1000 * r.max_lift:.1f} mm off the pad"
    if r.over:
        return 0.0, f"elbow bent past {OVER_GATE:.0f} deg (peak {r.peak:.1f})"
    if r.drop:
        return 0.0, f"forearm dropped (at {r.drop_at} s)"
    if r.peak < PEAK_MIN:
        return 0.0, f"peak {r.peak:.1f} deg < {PEAK_MIN:.0f}"
    if not RIGHT_LO <= r.peak <= RIGHT_HI:
        return 0.25, f"peak {r.peak:.1f} deg outside {RIGHT_LO:.0f}-{RIGHT_HI:.0f}"
    if r.second_rep:
        return 0.5, f"bent again to {PEAK_MIN:.0f} deg or more after coming back (one repetition asked)"
    if not r.returned:
        return 0.5, "not back within 10 deg of flat (and staying there) after the right angle"
    ends = {
        "angle": r.theta <= END_ANGLE,
        "on_pad": r.low <= END_LOW,
        "elbow_down": r.lift <= END_LIFT,
        "let_go": bool(r.clear) and min(r.clear.values()) >= CLEAR,
        "still_1s": r.still >= STILL_STEPS,
    }
    if all(ends.values()):
        return 1.0, "full repetition, elbow on the pad, released"
    return 0.75, "returned; end check failed: " + ", ".join(k for k, ok in ends.items() if not ok)


ROM = Rom()


def lowest_point(state: lk.LinkState, link: str) -> float:
    """Lowest z (m) of a link's colliders."""
    m = state.poses[link]
    low = math.inf
    for s in MANIKIN.link(link).shapes:
        if s.kind == "box":
            h = s.half_extents()
            corners = np.array(np.meshgrid(*[[-1, 1]] * 3)).T.reshape(-1, 3) * h
            pts = (m[:3, :3] @ (np.asarray(s.pos) + corners @ s.rot.T).T).T + m[:3, 3]
            low = min(low, float(pts[:, 2].min()))
        elif s.kind == "capsule":
            i = "XYZ".index(s.axis)
            half = np.zeros(3)
            half[i] = s.dims[1] / 2
            ends = [m[:3, :3] @ (np.asarray(s.pos) + s.rot @ (k * half)) + m[:3, 3] for k in (-1, 1)]
            low = min(low, min(float(p[2]) for p in ends) - s.dims[0])
        else:
            raise ValueError(f"lowest_point: no rule for {s.kind}")
    return low


def measure(state: lk.LinkState) -> tuple[float, float, float]:
    """(elbow angle deg, elbow lift m, forearm lowest point above the pad top m) from a manikin state."""
    return (state.deg("elbow"), float(state.point("upper_arm", E_IN_UPPER)[2] - (TOP + AXIS_H)),
            lowest_point(state, "forearm") - PAD_TOP)


@lk.on_reset
def _reset(tick):
    ROM.reset()


@lk.on_step
def _track(tick):
    s = tick.states["manikin"]
    theta, lift, low = measure(s)
    tools = tick.tools
    ROM.update(tick.t, theta, lift, low, {k: v.tcp for k, v in tools.items()},
               {k: lk.clearance(v, s, ARM_LINKS) for k, v in tools.items()})
    if ROM.n % 5 == 0:
        ROM.trace.append((round(tick.t, 2), round(theta, 1), round(1000 * lift, 1), round(ROM.omega, 0),
                          round(1000 * ROM.clear.get("left", -1), 0), round(1000 * ROM.clear.get("right", -1), 0)))


def score(objects) -> float:
    return ladder(ROM)[0]


def success(objects) -> bool:
    return ladder(ROM)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(ROM)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo (python linkdemo.py task.py OUT [--variant NAME]); reads only camera-observable poses ----------

VARIANTS = ("", "nothing", "one_arm", "one_arm_follow", "one_arm_push", "straight_lift", "drop", "overshoot",
            "side_push")
ONE_ARM = ("one_arm", "one_arm_follow", "one_arm_push", "straight_lift")
PUSH_IN = 0.03  # one_arm_push: the wrist target sits 3 cm in along the forearm, so the hand presses toward the elbow
OPEN_W = 0.075
GRIP_W, STEADY_W = 0.040, 0.046  # the splint and the upper arm are both 50 mm across: pads closed to 40 / 46 mm
PRESS_D, PRESS_TILT = 0.06, 45.0  # left pinch: 6 cm above the elbow, gripper leaning 45 deg toward the shoulder
RATE, SLOW_RATE, LAG_MAX = 25.0, 10.0, 8.0  # deg/s, deg/s, deg
KI, LEAD_MAX = 0.25, 25.0  # lead added per deg of error each step; its bound (deg)
LOG: dict = {}


def _frame(seen):
    """Observed elbow centre and the upper arm's U, C, Z directions."""
    m = seen.pose("manikin", "upper_arm")
    u, c = m[:3, :3] @ [1.0, 0.0, 0.0], m[:3, :3] @ [0.0, 1.0, 0.0]
    return seen.point("manikin", "upper_arm", E_IN_UPPER), u, c, np.cross(u, c)


def _wrist_pose(ref, theta_deg, follow=False, inward=0.0):
    """Right TCP target on the arc about the elbow's resting centre (``ref`` = _frame() seen before the first
    move: the bed does not move, so the arc must not follow an elbow that rises), approach pitched theta/3 back
    toward the right base. ``follow`` pitches it with the forearm instead (theta), for the one_arm_follow cheat;
    ``inward`` shortens the radius, so the hand pushes the wrist along the forearm toward the elbow (one_arm_push)."""
    import linkdemo as ld

    e, u, c, z = ref
    th, g = math.radians(theta_deg), math.radians(theta_deg if follow else theta_deg / 3.0)
    p = e + (GRASP_F - inward) * (math.cos(th) * -u + math.sin(th) * z)
    return p, ld.grip_frame(math.sin(g) * u - math.cos(g) * z, c)


def _steer(ctx, state, ref, width, lead_lo, lead_hi):
    """One control step of the angle loop: the arm is compliant (the forearm lags the commanded arc by up to
    ~15 deg under the spring), so the command leads the reference by an integral of the seen error."""
    th = ctx.observe().angle_deg("manikin", "elbow")
    err = state["ref"] - th
    state["lead"] = float(np.clip(state["lead"] + KI * err, lead_lo, lead_hi))
    ctx.right._solve(*_wrist_pose(ref, state["ref"] + state["lead"], state.get("follow", False),
                                  state.get("inward", 0.0)), width)
    return err


def _arc(ctx, target, rate, *, width, state, ref, lead_lo=-LEAD_MAX, lead_hi=LEAD_MAX):
    """Closed loop: ramp the reference angle to ``target`` at ``rate`` deg/s (pausing while the seen angle lags by
    more than LAG_MAX), tracking it with :func:`_steer`."""
    dt = 1.0 / 20
    for _ in range(round(3 * (abs(target - state["ref"]) / rate + 1.0) / dt)):  # gives up on a lost grip
        err = _steer(ctx, state, ref, width, lead_lo, lead_hi)
        if abs(err) <= LAG_MAX:
            step = rate * dt
            state["ref"] = min(target, state["ref"] + step) if target >= state["ref"] else max(target, state["ref"] - step)
        yield
        if state["ref"] == target:
            return


def _hold(ctx, target, seconds, *, width, state, ref, lead_lo=-LEAD_MAX, lead_hi=LEAD_MAX):
    state["ref"] = target
    for _ in range(round(seconds * 20)):
        _steer(ctx, state, ref, width, lead_lo, lead_hi)
        yield


def demo_script(ctx):
    import linkdemo as ld

    name = ctx.variant or ""
    if name not in VARIANTS:
        raise ValueError(f"unknown variant {name!r}; one of {VARIANTS}")
    LOG.clear()
    LOG["variant"] = name
    yield from ctx.wait(0.5)
    if name == "nothing":
        yield from ctx.wait(12.0)
        return
    seen = ctx.observe()
    ref = _frame(seen)
    e, u, c, z = ref
    grasp = seen.point("manikin", "forearm", GRASP_IN_FORE)
    top_down = ld.grip_frame(-z, c)
    right, left = ctx.right, ctx.left
    LOG["start_deg"] = round(seen.angle_deg("manikin", "elbow"), 2)

    if name == "side_push":  # closed gripper beside the splint, pushed 60 mm along the elbow axis: no flexion
        def push():
            yield from right.grip(0.0, 0.5)
            yield from right.move(grasp - 0.08 * c + 0.06 * z, top_down, duration=1.5)
            yield from right.move(grasp - 0.08 * c, top_down, duration=1.0)
            yield from right.move(grasp - 0.02 * c, top_down, duration=2.0)
            yield from right.wait(1.0)
            yield from right.move(grasp - 0.08 * c, top_down, duration=1.0)
            yield from right.move(grasp - 0.08 * c + 0.08 * z, top_down, duration=1.0)
        yield from push()
        yield from ctx.wait(3.0)
        return

    tilt = math.radians(PRESS_TILT)
    a_left = -math.sin(tilt) * u - math.cos(tilt) * z
    r_left = ld.grip_frame(a_left, c)
    press = e + PRESS_D * u

    def steady():  # left: pinch the upper arm just above the elbow and hold it on the pad
        yield from left.move(press - 0.07 * a_left, r_left, width=OPEN_W, duration=1.6)
        yield from left.move(press, r_left, duration=1.2)
        yield from left.grip(STEADY_W, 0.6)

    def take():  # right: pinch the splint's flat sides, closing along the elbow axis
        yield from right.move(grasp + 0.08 * z, top_down, width=OPEN_W, duration=1.6)
        yield from right.move(grasp, top_down, duration=1.2)
        yield from right.grip(GRIP_W, 0.6)
        yield from right.wait(0.4)

    two = name not in ONE_ARM
    yield from ctx.both(*([steady()] if two else []), take())
    LOG["grasped_deg"] = round(ctx.observe().angle_deg("manikin", "elbow"), 2)

    if name == "straight_lift":  # carry the wrist straight up 12 cm with one hand
        yield from right.move(right.pos + 0.12 * z, duration=3.0)
        yield from right.wait(1.0)
    else:
        state = {"ref": LOG["grasped_deg"], "lead": 0.0, "follow": name == "one_arm_follow",
                 "inward": PUSH_IN if name == "one_arm_push" else 0.0}
        top = 118.0 if name == "overshoot" else 90.0
        yield from _arc(ctx, top, RATE, width=GRIP_W, state=state, ref=ref)
        yield from _hold(ctx, top, 1.0, width=GRIP_W, state=state, ref=ref)
        LOG["held_deg"] = round(ctx.observe().angle_deg("manikin", "elbow"), 2)
        LOG["held_lead"] = round(state["lead"], 2)
        if name != "drop":
            yield from _arc(ctx, 10.0, RATE, width=GRIP_W, state=state, ref=ref)
            yield from _arc(ctx, 0.0, SLOW_RATE, width=GRIP_W, state=state, ref=ref, lead_lo=-3.0)
            yield from _hold(ctx, 0.0, 1.5, width=GRIP_W, state=state, ref=ref, lead_lo=-2.0, lead_hi=2.0)
            LOG["rest_deg"] = round(ctx.observe().angle_deg("manikin", "elbow"), 2)

    def let_go_right():  # open, back out along the approach, then up: pads end > 50 mm from the arm
        yield from right.open(0.6)
        yield from right.move(right.pos - right.rot[:, 2] * 0.10, duration=1.2)
        yield from right.move(right.pos + 0.04 * z, duration=0.6)

    def let_go_left():
        if two:
            yield from left.wait(0.6)
            yield from left.open(0.6)
            yield from left.move(left.pos - left.rot[:, 2] * 0.12, duration=1.4)

    yield from ctx.both(let_go_left(), let_go_right())
    LOG["final_deg"] = round(ctx.observe().angle_deg("manikin", "elbow"), 2)
    yield from ctx.wait(3.0)


def demo_report() -> dict:
    s, why = ladder(ROM)
    return {"score": s, "why": why, "peak_deg": round(ROM.peak, 2), "max_lift_mm": round(1000 * ROM.max_lift, 2),
            "over": ROM.over, "drop": ROM.drop, "drop_at": ROM.drop_at, "returned": ROM.returned,
            "second_rep": ROM.second_rep,
            "end": {"theta": round(ROM.theta, 2), "lift_mm": round(1000 * ROM.lift, 2), "low_mm": round(1000 * ROM.low, 2),
                    "clear_mm": {k: round(1000 * v, 1) for k, v in ROM.clear.items()},
                    "tcp_speed_mm_s": {k: round(1000 * v, 2) for k, v in ROM.tcp_speed.items()}, "still": ROM.still},
            "log": dict(LOG), "trace": ROM.trace}
