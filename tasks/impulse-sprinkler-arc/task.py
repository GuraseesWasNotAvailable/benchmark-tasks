# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Set an impulse sprinkler's two trip collars so it waters a 90-degree arc.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A brass impulse sprinkler on a spike base stands in a 200 x 200 x 30 mm soil tray fixed to the table at (0.15, 0.0).
  The spike and riser (28 mm brass tube, 50 mm tall above the soil) are fixed. The swivel head (32 mm brass tube,
  84 mm tall, 0.12 kg) turns on a bearing that takes 0.004 N m to turn. Total height above the soil 138 mm.
- Two trip collars, a red one (lower) and a blue one (upper), sit on the head's ring, each a 49 mm plastic ring with a
  radial trip arm ending in a vertical pin (8 mm) 45 mm from the axis. A collar grips the ring by friction: it slips at 0.06 N m, which
  is 1.3 N pushed on its pin (the head turns at 0.004 N m, 0.09 N on a pin). The blue pin is 200 degrees counter-clockwise (seen from above) of the red pin; the arc
  of the watering is the angle from the red pin counter-clockwise to the blue pin.

Goal: the blue pin 90 degrees (+-5) counter-clockwise of the red pin, both grippers let go.
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

CX, CY = 0.15, 0.0  # sprinkler axis
SOIL = 0.030  # soil tray height; the linkage frame origin is the soil top
Z0 = TOP + SOIL
PIN_R = 0.045  # trip pin axis radius (the trip tab reaches 49 mm from the axis)
PIN_RAD = 0.004
THETA_A = -150.0  # red pin azimuth at the start, deg (0 = world +x, counter-clockwise from above)
ARC0 = 200.0  # start arc red -> blue, ccw
THETA_B = THETA_A + ARC0
TARGET, TOL = 90.0, 5.0
HEAD_FRICTION = 0.004  # N m
COLLAR_FRICTION = 0.06  # N m

BRASS, RED, BLUE = (0.78, 0.62, 0.22), (0.80, 0.10, 0.10), (0.10, 0.25, 0.80)


def _pin(theta, z0, z1):
    t = math.radians(theta)
    return lk.cylinder(PIN_RAD, z1 - z0, (PIN_R * math.cos(t), PIN_R * math.sin(t), (z0 + z1) / 2), axis="Z")


def _bar(theta, zc):
    """Radial trip arm from the collar ring out to the pin: 30 x 10 x 6 mm, r 21 to 51 mm."""
    t = math.radians(theta)
    return lk.box((0.030, 0.010, 0.006), (0.036 * math.cos(t), 0.036 * math.sin(t), zc), rpy_deg=(0, 0, theta))


PIN_A_LOCAL = (PIN_R * math.cos(math.radians(THETA_A)), PIN_R * math.sin(math.radians(THETA_A)), 0.082)
PIN_B_LOCAL = (PIN_R * math.cos(math.radians(THETA_B)), PIN_R * math.sin(math.radians(THETA_B)), 0.082)

_t = lk.Tree("sprk", pos=(CX, CY, Z0), fixed=True, color=BRASS)
_t.link("base", [lk.box((0.200, 0.200, SOIL), (0, 0, -SOIL / 2)), lk.cylinder(0.014, 0.050, (0, 0, 0.025))], mass=6.0,
        friction=(0.7, 0.6), contact_offset=0.001)
_t.link("head", [lk.cylinder(0.016, 0.084, (0, 0, 0.092)), lk.cylinder(0.019, 0.004, (0, 0, 0.136))], mass=0.12,
        friction=(0.9, 0.8), contact_offset=0.001)
_t.link("collarA", [lk.cylinder(0.0245, 0.010, (0, 0, 0.057)), _bar(THETA_A, 0.059), _pin(THETA_A, 0.056, 0.090)], mass=0.02, color=RED,
        friction=(0.9, 0.8), contact_offset=0.001)
_t.link("collarB", [lk.cylinder(0.0245, 0.010, (0, 0, 0.071)), _bar(THETA_B, 0.073), _pin(THETA_B, 0.070, 0.090)], mass=0.02, color=BLUE,
        friction=(0.9, 0.8), contact_offset=0.001)
_t.revolute("swivel", "base", "head", anchor=(0, 0, 0.05), axis=(0, 0, 1), limits_deg=(-720, 720),
            damping=1e-4, friction=HEAD_FRICTION)
_t.revolute("slipA", "head", "collarA", anchor=(0, 0, 0.057), axis=(0, 0, 1), limits_deg=(-400, 400),
            damping=0.02, armature=1e-3, friction=COLLAR_FRICTION)
_t.revolute("slipB", "head", "collarB", anchor=(0, 0, 0.071), axis=(0, 0, 1), limits_deg=(-400, 400),
            damping=0.02, armature=1e-3, friction=COLLAR_FRICTION)
SPK = _t.build()
OBJECTS = [SPK]

# -- scoring ---------------------------------------------------------------------------------------------------------
MOVED_ARC = 160.0  # rung 0.25: arc ever at most this (40 deg closed)
NEAR_TOL = 20.0  # rung 0.5
STEADY_RATE = 0.5  # deg/s
CLEAR_M, TCP_STILL = 0.030, 0.005
RELEASE_S, STEADY_S = 1.0, 1.0
EPS = 1e-6


def azimuth(p) -> float:
    return math.degrees(math.atan2(p[1] - CY, p[0] - CX))


def arc_deg(pin_a, pin_b) -> float:
    """Arc from the red pin counter-clockwise to the blue pin, deg in [0, 360)."""
    return (azimuth(pin_b) - azimuth(pin_a)) % 360.0


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def measure(spk, tools) -> dict:
    pa, pb = spk.point("collarA", PIN_A_LOCAL), spk.point("collarB", PIN_B_LOCAL)
    clear = {s: lk.clearance(t, spk) for s, t in tools.items()}
    return {
        "arc": arc_deg(pa, pb),
        "az_a": azimuth(pa),
        "az_b": azimuth(pb),
        "head_az": math.degrees(math.atan2(spk.poses["head"][1, 0], spk.poses["head"][0, 0])),
        "q": [round(spk.deg(j), 2) for j in ("swivel", "slipA", "slipB")],
        "clear": clear,
        "tcp_speed": {s: float(t.speed) for s, t in tools.items()},
    }


class Record:
    """Whole-run record (a linkkit History updated every control step, cleared only by the simulator reset)."""

    def __init__(self):
        self.H = lk.History()
        self.last: dict | None = None
        self.n = 0
        self.trace: list = []

    def reset(self) -> None:
        self.H.reset()
        self.last, self.n, self.trace = None, 0, []

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        rate = spin = 0.0
        if self.last is not None and H.dt > 0:
            rate = abs(_wrap(m["arc"] - self.last["arc"])) / H.dt
            spin = math.radians(abs(_wrap(m["head_az"] - self.last["head_az"]))) / H.dt
        self.last = m
        m["arc_rate"], m["spin"] = rate, spin
        H.peak("spin", spin)
        H.low("arc_min", m["arc"])
        H.ever("moved", m["arc"] <= MOVED_ARC)
        ok = abs(m["arc"] - TARGET) <= TOL
        H.held("steady", ok and rate < STEADY_RATE)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    err = abs(m["arc"] - TARGET)
    if err <= TOL:
        ends = {"arc still 1 s": H.runs.get("steady", 0.0) >= STEADY_S - EPS,
                "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS}
        if all(ends.values()):
            return 1.0, "arc 90 deg, collars still, both grippers let go"
        return 0.75, "arc within 5 deg at the end; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if err <= NEAR_TOL:
        return 0.5, f"arc {m['arc']:.1f} deg at the end, within 20 of 90 but not 5"
    if "moved" in H.evers:
        return 0.25, "the arc closed at least 40 deg at some time but is not near 90 at the end"
    return 0.0, f"arc {m['arc']:.1f} deg, never closed 40 deg"


REC = Record()


@lk.on_reset
def _reset(tick):
    REC.reset()


@lk.on_step
def _track(tick):
    m = measure(tick.states["sprk"], tick.tools)
    REC.update(m)
    if REC.n % 20 == 0:  # every second
        REC.trace.append((round(tick.t, 1), round(m["arc"], 1), round(m["head_az"], 1), m["q"],
                          [round(1000 * m["clear"][s]) for s in ("left", "right")]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
import linkdemo as ld  # noqa: E402

VARIANTS = ("", "nothing", "left_only", "right_only", "wrong_way")
HEAD_Z = Z0 + 0.122  # left pads on the head tube
HEAD_W = 0.032 - 0.004  # closed pad opening on the 32 mm tube
PIN_Z = Z0 + 0.082  # pad contact height on the pins
PIN_W = 2 * PIN_RAD - 0.0006
D = 0.0115  # pad contact centre past the TCP
LOG: dict = {}


def seen_arc(seen) -> float:
    return arc_deg(seen.point("sprk", "collarA", PIN_A_LOCAL), seen.point("sprk", "collarB", PIN_B_LOCAL))


def left_hold(ctx, st):
    arm = ctx.left
    R = ld.grip_frame((0, -1, 0), (1, 0, 0))
    tcp = np.array([CX, CY, HEAD_Z]) + np.array([0.0, D, 0.0])
    yield from arm.move(tcp + np.array([0.0, 0.08, 0.04]), R, width=ld.OPEN_WIDTH, duration=2.5)
    yield from arm.move(tcp, R, width=0.040, duration=2.0)
    yield from arm.grip(HEAD_W, 0.8)
    st["left_held"] = True
    while not st.get("right_done"):
        yield from arm.wait(0.1)
    yield from arm.grip(ld.OPEN_WIDTH, 0.8)
    yield from arm.retreat(0.10, 1.2)
    st["left_done"] = True


def right_push(ctx, st, sign=1.0):
    arm = ctx.right
    seen = ctx.observe()
    pin = seen.point("sprk", "collarA", PIN_A_LOCAL)
    th = math.radians(azimuth(pin))
    R = ld.grip_frame((0, 0, -1), (-math.sin(th), math.cos(th), 0.0))
    tcp = np.array([pin[0], pin[1], PIN_Z + D])
    hover = tcp + np.array([0.0, 0.0, 0.025])
    yield from arm.move(hover + np.array([0.0, 0.0, 0.04]), R, width=ld.OPEN_WIDTH, duration=3.0)
    yield from arm.move(hover, R, width=0.030, duration=1.5)
    for _ in range(2):  # null the arm's sag: pad contact centre over the pin axis, seen from poses
        yield from arm.wait(0.4)
        sn = ctx.observe()
        tl = sn.tools["right"]
        contact = tl.tcp + tl.rot @ np.array([0.0, 0.0, D])
        pin = sn.point("sprk", "collarA", PIN_A_LOCAL)
        hover = hover + np.array([pin[0] - contact[0], pin[1] - contact[1], 0.0])
        yield from arm.move(hover, R, duration=0.8)
    st["align_mm"] = [round(1000 * float(v), 1) for v in (pin - contact)[:2]]
    yield from arm.move(hover - np.array([0.0, 0.0, 0.025]), R, duration=1.5)
    yield from arm.wait(0.5)
    sn = ctx.observe()
    tl = sn.tools["right"]
    contact = tl.tcp + tl.rot @ np.array([0.0, 0.0, D])
    st["at_pin_mm"] = [round(1000 * float(v), 1) for v in (sn.point("sprk", "collarA", PIN_A_LOCAL) - contact)]
    yield from arm.grip(PIN_W, 0.6)
    yield from arm.wait(0.3)
    st["closed_opening"] = round(float(ctx.observe().tools["right"].opening), 4)
    centre = np.array([CX, CY, 0.0])
    for _ in range(24):
        seen = ctx.observe()
        err = seen_arc(seen) - TARGET
        st.setdefault("arcs", []).append(round(err + TARGET, 2))
        if abs(err) <= 0.7 or (sign < 0 and len(st["arcs"]) > 5) or (VARIANT == "right_only" and len(st["arcs"]) > 9):
            break
        step = float(np.clip(err, -12.0, 12.0))  # pushing the red pin counter-clockwise shrinks the arc
        yield from arm.arc(centre, (0, 0, 1), sign * step, point=(0.0, 0.0, D), follow=1.0,
                           duration=max(0.8, abs(step) / 8.0))
        yield from arm.wait(0.4)
    yield from arm.grip(ld.OPEN_WIDTH, 0.8)
    yield from arm.retreat(0.08, 1.2)
    st["right_done"] = True


def demo_script(ctx):
    v = VARIANT
    LOG.clear()
    st = LOG
    st["variant"] = v
    yield from ctx.wait(1.0)
    if v == "nothing":
        yield from ctx.wait(8.0)
        return

    def left():
        if v == "right_only":
            return
        yield from left_hold(ctx, st)

    def right():
        if v == "left_only":
            yield from ctx.right.wait(14.0)
            st["right_done"] = True
            return
        while not st.get("left_held") and v != "right_only":
            yield from ctx.right.wait(0.1)
        yield from right_push(ctx, st, -1.0 if v == "wrong_way" else 1.0)

    yield from ctx.both(left(), right())
    yield from ctx.wait(3.0)


def demo_report() -> dict:
    s, why = ladder(REC)
    H = REC.H
    m = REC.last or {}
    return {"score": s, "reason": why, "variant": VARIANT,
            "evers": {k: round(v, 2) for k, v in H.evers.items()},
            "peaks": {k: round(v, 2) for k, v in H.peaks.items()},
            "lows": {k: round(v, 2) for k, v in H.lows.items()},
            "runs": {k: round(v, 2) for k, v in H.runs.items()},
            "last": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in m.items()
                     if k not in ("clear", "tcp_speed", "q")},
            "clear_mm": {k: round(1000 * v, 1) for k, v in m.get("clear", {}).items()},
            "log": dict(LOG), "trace": REC.trace}
