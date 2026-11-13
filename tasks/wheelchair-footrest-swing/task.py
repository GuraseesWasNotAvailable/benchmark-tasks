# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Swing a wheelchair's swing-away footrest out of the way and lift it off its hinge pin.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- The front-frame bracket of a wheelchair (powder-coated steel plate on a post, fixed to the table) carries a vertical
  hinge pin at (0.15, -0.15), 100 mm above the table. A swing-away footrest hanger (steel tube 25 mm across, 0.8 kg
  with its sleeve and flipped-up aluminium footplate) sits on the pin with its tube pointing forward (+x, 0 deg).
- A spring release latch, a lever pad on the bracket 130 mm to the left of the pin, holds the hanger
  at 0 deg. Pressed down 6 mm or more it frees the hanger to swing about the pin; whenever the pad is let up (under
  3 mm) the latch pawl drops into the notch plate and locks the hanger where it stands.
- The hanger can only be lifted off the pin once it has swung out 85 deg (its keeper lug lines up with the slot in the
  pin collar). It then rises 35 mm to clear the pin top and is set down on the bracket's parking stop 40 mm up.

Goal: the hanger swung out 90 deg (right, toward -y), lifted clear of the hinge pin and resting on the parking stop,
both grippers let go.
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

PIN_XY = (0.15, -0.15)
ZS = 0.100  # bracket seat above the table (m)
SLEEVE_R, SLEEVE_H = 0.014, 0.060
TUBE_R, TUBE_X1 = 0.0125, 0.300  # tube radius; end of the tube from the pin axis
TUBE_Z = 0.045  # tube axis above the seat
LEVER_XY = (0.0, 0.13)  # latch pad relative to the pin
STEEL = (0.20, 0.22, 0.26)
RED = (0.80, 0.10, 0.10)
BLUE = (0.10, 0.30, 0.75)
SILVER = (0.78, 0.80, 0.84)

PRESS_ON, PRESS_OFF = 0.006, 0.003  # latch pad travel to free / lock the pawl (m)
KEEPER_DEG = -85.0  # swing at which the hanger may rise
PARK_TRIGGER, PARK_LIFT = 0.045, 0.040  # lifted this far the sleeve is clear of the pin; parking stop height (m)
PIN_TOP = 0.035

t = lk.Tree("chair", pos=(PIN_XY[0], PIN_XY[1], TOP), fixed=True, color=STEEL)
t.link("frame", [lk.box((0.20, 0.34, 0.008), (-0.06, 0.04, 0.004)),
                 lk.box((0.030, 0.030, ZS), (-0.060, 0.0, ZS / 2)),
                 lk.box((0.090, 0.320, 0.010), (-0.015, 0.03, ZS - 0.005))], mass=3.0)
t.link("carrier", [lk.cylinder(0.008, PIN_TOP, (0, 0, ZS + PIN_TOP / 2))], mass=0.02, at=(0, 0, 0), color=SILVER)
t.link("hanger", [lk.cylinder(SLEEVE_R, SLEEVE_H, (0, 0, ZS + 0.0005 + SLEEVE_H / 2)),
                  lk.cylinder(TUBE_R, TUBE_X1 - 0.012, ((TUBE_X1 + 0.012) / 2, 0, ZS + TUBE_Z), axis="X"),
                  lk.box((0.004, 0.120, 0.080), (TUBE_X1 + 0.002, 0, ZS + TUBE_Z + 0.040))],
       mass=0.80, at=(0, 0, 0), color=BLUE, friction=(0.8, 0.7))
t.link("button", [lk.box((0.028, 0.028, 0.014))], mass=0.02, at=(LEVER_XY[0], LEVER_XY[1], ZS + 0.015), color=RED)
t.revolute("swing", "frame", "carrier", anchor=(0, 0, ZS), axis=(0, 0, 1), limits_deg=(-95, 3), init_deg=0,
           stiffness=0.0, damping=0.05, brake=lk.Brake(stiffness=100.0, damping=2.0, engaged=True))
t.prismatic("lift", "carrier", "hanger", anchor=(0, 0, ZS), axis=(0, 0, 1), limits=(0.0, 0.08), rest=0.0,
            stiffness=0.0, damping=30.0, brake=lk.Brake(stiffness=20000.0, damping=200.0, engaged=True))
t.prismatic("press", "frame", "button", anchor=(LEVER_XY[0], LEVER_XY[1], ZS + 0.015), axis=(0, 0, -1),
            limits=(0.0, 0.008), rest=-0.003, stiffness=400.0, damping=6.0)
CHAIR = t.build()
OBJECTS = [CHAIR]

# -- whole-run record -----------------------------------------------------------------------------------------

H = lk.History()
GATE_CLEAR_M = 0.030
GATE_SPEED = 0.005
YAW_QUARTER, YAW_HALF, YAW_END_TOL = -45.0, -85.0, 5.0
LIFT_LO, LIFT_HI = 0.038, 0.044
HANGER_STILL_S, HANGER_STILL_MM_S, HANGER_STILL_DEG_S = 1.0, 3.0, 3.0
_PREV: dict = {}
_STATE: dict = {"park": False}
LOG: dict = {}


def pose_rates(p0, p1, dt):
    """Speed (m/s) and spin (deg/s) of a body from two 4x4 poses one control step apart (what a camera sees)."""
    if dt <= 0:
        return 0.0, 0.0
    speed = float(np.linalg.norm(p1[:3, 3] - p0[:3, 3])) / dt
    c = (float(np.trace(p0[:3, :3].T @ p1[:3, :3])) - 1.0) / 2.0
    return speed, math.degrees(math.acos(max(-1.0, min(1.0, c)))) / dt


def update(h, m):
    """Feed one control step's measurement into the record (the kit ticks the History's clock before on_step). m: yaw (deg, 0 latched, negative = swung to the right),
    lift (m above the seat), press (m), hspeed (m/s), hspin (deg/s), clear (m, nearest pad of either gripper to
    the chair), tcp_speed (m/s, faster gripper)."""
    h.low("yaw", m["yaw"])
    h.peak("lift", m["lift"])
    h.peak("press", m["press"])
    h.set("yaw_end", m["yaw"])
    h.set("lift_end", m["lift"])
    h.held("still", m["hspeed"] * 1000 < HANGER_STILL_MM_S and m["hspin"] < HANGER_STILL_DEG_S)
    h.held("gate", m["clear"] >= GATE_CLEAR_M and m["tcp_speed"] < GATE_SPEED)


def ladder(h):
    """Rung reached and why. 0.25: swung out at least 45 deg; 0.5: at least 85 deg; 0.75: at the end swung out
    90 +- 5 deg, lifted 38 to 44 mm (clear of the pin, on the parking stop) and still 1 s; 1.0: and both grippers
    30 mm clear of the chair and moving under 5 mm/s for the last 1 s."""
    lo = h.lows.get("yaw", 0.0)
    if lo > YAW_QUARTER:
        return 0.0, "swung under 45 deg"
    if lo > YAW_HALF:
        return 0.25, "swung under 85 deg"
    yaw, lift = h.values.get("yaw_end", 0.0), h.values.get("lift_end", 0.0)
    parked = (abs(yaw + 90.0) <= YAW_END_TOL and LIFT_LO <= lift <= LIFT_HI
              and h.runs.get("still", 0.0) >= HANGER_STILL_S)
    if not parked:
        return 0.5, "swung out; not parked clear of the pin"
    if h.runs.get("gate", 0.0) < 1.0:
        return 0.75, "parked; grippers not clear and still for 1 s"
    return 1.0, "done"


@lk.on_reset
def _reset(tick):
    _PREV.clear()
    _STATE.update(park=False)
    LOG.clear()


@lk.on_step
def _track(tick):
    s = tick.states["chair"]
    yaw, lift, press = s.deg("swing"), s.q["lift"], s.q["press"]
    # the latch pawl: pressed 6 mm frees the swing; let up under 3 mm it drops in and locks where the hanger stands
    if lk.braked("chair", "swing"):
        if press >= PRESS_ON:
            lk.brake("chair", "swing", False)
    elif press < PRESS_OFF:
        lk.brake("chair", "swing", True)
    # the keeper: the hanger may rise only once it has swung out 85 deg
    if lk.braked("chair", "lift") and yaw <= KEEPER_DEG:
        lk.brake("chair", "lift", False)
    # parking stop: once the sleeve has cleared the pin the hanger comes to rest 40 mm up
    if not _STATE.get("park") and lift >= PARK_TRIGGER:
        lk.set_limits("chair", "lift", PARK_LIFT, 0.08)
        _STATE["park"] = True
    P = np.array(s.poses["hanger"], dtype=float)
    hs, hp = (0.0, 0.0)
    if _PREV:
        hs, hp = pose_rates(_PREV["P"], P, tick.t - _PREV["t"])
    _PREV.update(P=P, t=tick.t)
    clear = min(lk.clearance(tool, s) for tool in tick.tools.values())
    update(H, {"yaw": yaw, "lift": lift, "press": press, "hspeed": hs, "hspin": hp, "clear": clear,
               "tcp_speed": max(tool.speed for tool in tick.tools.values())})
    H.trace("yaw_lift_press_clear", (round(yaw, 1), round(1000 * lift, 1), round(1000 * press, 1),
                                     round(1000 * clear, 0), round(1000 * hs, 2), round(hp, 2),
                                     [round(1000 * tool.speed, 2) for tool in tick.tools.values()]), every=1.0)


def score(objects) -> float:
    return ladder(H)[0]


def success(objects) -> bool:
    return ladder(H)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(H)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


def demo_report() -> dict:
    return {"rung": ladder(H), "lows": dict(H.lows), "peaks": dict(H.peaks), "values": dict(H.values),
            "runs": dict(H.runs), "trace": H.traces.get("yaw_lift_press_clear", []), "log": dict(LOG),
            "variant": VARIANT}


# -- scripted demo (reads only what a camera sees: link poses and the robot's own grippers) --------------------------

VARIANTS = ("", "nothing", "left_only", "right_only", "early_release", "lift_first")
GRASP_X = 0.075  # grasp on the tube, from the pin axis (m)
CONTACT = 0.0115  # pad contact centre past the TCP
GRASP_SQUEEZE_SWING, GRASP_SQUEEZE_LIFT = 0.001, 0.004
TUBE_D = 2 * TUBE_R
PRESS_DEPTH = 0.016
DOWN = np.array([0.0, 0.0, -1.0])
WORLD = {"left_done": False}


def demo_script(ctx):
    import linkdemo as ld
    from yam_arena.constants import PAD_TIP_PAST_TCP

    WORLD["left_done"] = False
    yield from ctx.wait(1.0)
    seen = ctx.observe()
    pin, axis = seen.axis("chair", "swing")
    btn = seen.point("chair", "button", (0, 0, 0.007))
    tube = seen.point("chair", "hanger", (GRASP_X, 0.0, ZS + TUBE_Z - 0.003))
    LOG["seen"] = {"pin": np.round(pin, 3).tolist(), "button_top": np.round(btn, 3).tolist(),
                   "grasp": np.round(tube, 3).tolist()}
    release_at = -25.0 if VARIANT == "early_release" else -88.0

    def yaw():
        return ctx.observe().angle_deg("chair", "swing")

    def lift():
        return ctx.observe().angle_deg("chair", "lift")

    def pressed():
        return ctx.observe().angle_deg("chair", "press")

    def left():
        arm = ctx.left
        top = btn + np.array([0.0, 0.0, PAD_TIP_PAST_TCP])
        r = ld.down(0.0)
        yield from arm.grip(0.0, 0.5)
        yield from arm.move(top + [0, 0, 0.06], r, duration=2.0)
        yield from arm.move(top - [0, 0, PRESS_DEPTH], r, duration=1.5)
        for k in range(int(40 * ld.CONTROL_HZ)):  # hold the latch pad down until the hanger is swung out
            if yaw() <= release_at or (VARIANT == "left_only" and k > 25 * ld.CONTROL_HZ):
                break
            yield
        LOG["left_released_at_yaw"] = round(yaw(), 1)
        WORLD["left_done"] = True
        yield from arm.move(top + [0, 0, 0.08], r, duration=1.0)

    def right():
        arm = ctx.right
        r = ld.grip_frame(DOWN, (0.0, 1.0, 0.0))
        tcp = tube - r @ np.array([0.0, 0.0, CONTACT])
        yield from arm.move(tcp + [0, 0, 0.06], r, width=ld.OPEN_WIDTH, duration=2.0)
        yield from arm.move(tcp, r, duration=1.5)
        yield from arm.grip(TUBE_D - GRASP_SQUEEZE_SWING, 0.8)
        yield from arm.wait(0.4)
        if VARIANT != "right_only":  # wait until the latch pad is seen pressed
            for _ in range(int(20 * ld.CONTROL_HZ)):
                if pressed() >= PRESS_ON + 0.001:
                    break
                yield
        if VARIANT == "lift_first":
            yield from arm.grip(TUBE_D - GRASP_SQUEEZE_LIFT, 0.5)
            yield from arm.move(arm.pos + [0, 0, 0.06], duration=2.5)
            yield from arm.wait(1.0)
            return
        yield from arm.arc(pin, (0, 0, 1), -90.0, point=(0.0, 0.0, CONTACT), follow=1.0, duration=5.0)
        for _ in range(3):
            yield from arm.wait(0.3)
            err = -90.0 - yaw()
            if abs(err) <= 1.0 or abs(err) > 15.0:
                break
            yield from arm.arc(pin, (0, 0, 1), err, point=(0.0, 0.0, CONTACT), follow=1.0,
                               duration=max(0.6, abs(err) / 8.0))
        LOG["yaw_after_swing"] = round(yaw(), 2)
        if VARIANT == "early_release":
            yield from arm.wait(3.0)
            return
        for _ in range(int(10 * ld.CONTROL_HZ)):  # wait for the latch hand to let go (the pawl re-locks the swing)
            if WORLD["left_done"]:
                break
            yield
        yield from arm.wait(1.0)
        yield from arm.grip(TUBE_D - GRASP_SQUEEZE_LIFT, 0.6)
        yield from arm.wait(0.5)
        base = lift()
        z0 = arm.pos.copy()
        yield from arm.move(z0 + [0, 0, 0.065], duration=3.0)
        LOG["lift_after_raise_mm"] = round(1000 * (lift() - base), 1)
        yield from arm.wait(0.5)
        for _ in range(3):  # lower until the hanger is seen 2 mm above the parking stop
            err = lift() - (PARK_LIFT + 0.002)
            if abs(err) < 0.0008:
                break
            yield from arm.move(arm.pos - [0, 0, err], duration=max(0.8, abs(err) / 0.012))
            yield from arm.wait(0.4)
        LOG["lift_before_release_mm"] = round(1000 * lift(), 1)
        yield from arm.open(0.8)
        yield from arm.wait(0.5)
        yield from arm.retreat(0.06, 1.0)
        yield from arm.move(arm.pos + [0, 0, 0.05], duration=1.0)

    if VARIANT == "nothing":
        yield from ctx.wait(10.0)
        return
    if VARIANT == "left_only":
        yield from left()
        return
    if VARIANT == "right_only":
        yield from right()
        yield from ctx.wait(2.0)
        return
    yield from ctx.both(left(), right())
    yield from ctx.wait(3.0)
    LOG["final"] = {"yaw": round(yaw(), 2), "lift_mm": round(1000 * lift(), 2)}
