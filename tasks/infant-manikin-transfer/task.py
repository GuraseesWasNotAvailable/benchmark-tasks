# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Move a newborn training manikin from a bassinet mattress to a changing pad with its head and neck supported.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A bassinet (fixed): a 0.26 x 0.60 m tray holding a 0.22 x 0.56 x 0.03 m mattress (top 50 mm above the table), with
  30 mm side rims and end walls, centred at x 0.34. A newborn training manikin lies on its back on the mattress with
  its head toward the left arm and its feet toward the right arm: 2.5 kg, crown to heels 0.42 m (legs flexed), a
  85 mm wide head on a passive neck hinge (stops at 45 deg forward and 60 deg back, a very weak spring), a 95 mm
  chest, a 75 mm pelvis and legs 140 mm across.
- A changing pad (fixed): foam, 0.166 x 0.46 m, top 35 mm above the table, centred at x 0.125, 15 mm below the mattress.

Goal: the manikin lies on its back on the changing pad with its head resting there too, neck straight, both
grippers let go and clear. Whole run: the neck must never bend more than 20 deg and the body never tilt more than 30 deg.
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

SKIN = (0.93, 0.76, 0.66)
BASS_GREY = (0.78, 0.80, 0.84)
MATT = (0.90, 0.92, 0.78)
PAD_BLUE = (0.45, 0.65, 0.82)

# -- geometry (m) ------------------------------------------------------------------------------------------------------
XC = 0.34  # bassinet and manikin centre line
PAD_XC = 0.125
ZM = TOP + 0.050  # mattress top
ZP = TOP + 0.035  # pad top
CROWN_Y = 0.20  # manikin crown (body axis along -y: head toward the left arm, feet toward the right arm)
SY = -1.0  # direction of the body axis (from crown to feet) along world y
S_NECK, S_HEAD, S_BODY = 0.13, 0.060, 0.276  # along the body from the crown: neck hinge, head centre, body mass centre
R_HEAD = 0.0425
BODY_Y = CROWN_Y + SY * S_BODY  # the body link's y, which is also the hips grip
HALF_T = 0.0425  # chest half thickness: link centres sit this high above the surface
PAD_HALF_X, PAD_HALF_Y = 0.083, 0.23

# -- scoring constants -------------------------------------------------------------------------------------------------
NECK_GATE_DEG = 20.0
TILT_GATE_DEG = 30.0
LIFT_M = 0.035  # lifted clear of the mattress: body and head both 35 mm up (rim top is 30 mm)
CARRIED_X = 0.20  # centre line past the bassinet rim, over the pad
REST_TOL_M = (-0.003, 0.006)  # body bottom relative to the pad top
HEAD_TOL_M = 0.008
ON_PAD_X, ON_PAD_Y = 0.045, 0.060  # centre line within this of the pad centre / start y
SUPINE_DEG = 25.0
NECK_END_DEG = 10.0
STILL_MS, SPIN_DS = 0.005, 3.0
CLEAR_M = 0.030
TCP_STILL = 0.005
RELEASE_S = 1.0
EPS = 1e-6

# -- bassinet (fixed) --------------------------------------------------------------------------------------------------
_t = lk.Tree("bassinet", pos=(XC, 0.0, TOP), fixed=True, color=BASS_GREY)
_t.link("tray", [
    lk.box((0.26, 0.60, 0.02), (0, 0, 0.01)),
    lk.box((0.22, 0.56, 0.03), (0, 0, 0.035), color=MATT),
    lk.box((0.01, 0.56, 0.03), (0.115, 0, 0.065)),
    lk.box((0.01, 0.56, 0.03), (-0.115, 0, 0.065)),
    lk.box((0.22, 0.01, 0.03), (0, 0.285, 0.065)),
    lk.box((0.22, 0.01, 0.03), (0, -0.285, 0.065)),
], mass=3.0, friction=(0.6, 0.5), contact_offset=0.001)
BASSINET = _t.build()

# -- changing pad (fixed) ----------------------------------------------------------------------------------------------
_t = lk.Tree("pad", pos=(PAD_XC, 0.0, TOP), fixed=True, color=PAD_BLUE)
_t.link("pad", [lk.box((2 * PAD_HALF_X, 2 * PAD_HALF_Y, 0.035), (0, 0, 0.0175))], mass=1.0, friction=(0.6, 0.5),
        contact_offset=0.001)
PAD = _t.build()

# -- newborn training manikin ------------------------------------------------------------------------------------------
_t = lk.Tree("manikin", pos=(XC, BODY_Y, ZM + 0.0005 + HALF_T), color=SKIN)  # root = the body link
_t.link("body", [
    lk.box((0.095, 0.10, 0.085), (0, SY * (0.19 - S_BODY), 0)),  # chest
    lk.box((0.075, 0.08, 0.06), (0, SY * (0.27 - S_BODY), 0.03 - HALF_T)),  # pelvis (the hips grip)
    lk.box((0.140, 0.11, 0.05), (0, SY * (0.365 - S_BODY), 0.025 - HALF_T)),  # flexed legs
], mass=2.10, friction=(1.3, 1.2), contact_offset=0.001)
_t.link("head", [lk.capsule(R_HEAD, 0.035, (0, 0, 0), axis="Y")], mass=0.40, at=(0.0, SY * (S_HEAD - S_BODY), R_HEAD - HALF_T),
        friction=(0.8, 0.7), contact_offset=0.001)
_t.revolute("neck", "body", "head", anchor=(0.0, SY * (S_NECK - S_BODY), R_HEAD - HALF_T), axis=(-SY, 0, 0), limits_deg=(-45, 60), rest_deg=0,
            stiffness=0.03, damping=0.003, friction=0.0)
MANIKIN = _t.build()

OBJECTS = [BASSINET, PAD, MANIKIN]


# -- scoring ---------------------------------------------------------------------------------------------------------
def pose_rates(a: np.ndarray, b: np.ndarray, dt: float) -> tuple[float, float]:
    """Speed (m/s) and spin (deg/s) from the pose change between two control steps (what a camera sees)."""
    if dt <= 0:
        return 0.0, 0.0
    dv = float(np.linalg.norm(b[:3, 3] - a[:3, 3])) / dt
    c = (float(np.trace(a[:3, :3].T @ b[:3, :3])) - 1.0) / 2.0
    return dv, math.degrees(math.acos(max(-1.0, min(1.0, c)))) / dt


def measure(state, tools, speed=0.0, spin=0.0) -> dict:
    """Everything the scorer needs from one control step."""
    B, Hd = np.asarray(state.poses["body"], float), np.asarray(state.poses["head"], float)
    rest_z = ZM + HALF_T
    return {
        "neck": abs(state.deg("neck")),
        "pitch": math.degrees(math.asin(min(1.0, abs(B[2, 1])))),
        "roll": math.degrees(math.asin(min(1.0, abs(B[2, 0])))),
        "up_deg": math.degrees(math.acos(max(-1.0, min(1.0, B[2, 2])))),
        "rise_b": B[2, 3] - rest_z, "rise_h": Hd[2, 3] - rest_z,
        "bx": B[0, 3], "by": B[1, 3], "hx": Hd[0, 3],
        "low_b": B[2, 3] - HALF_T - ZP, "low_h": Hd[2, 3] - R_HEAD - ZP,
        "speed": speed, "spin": spin,
        "clear": {s: lk.clearance(tools[s], state) for s in ("left", "right")},
        "tcp_speed": {s: float(tools[s].speed) for s in ("left", "right")},
    }


class Record:
    """Whole-run record: a linkkit History fed every control step, cleared only by the simulator reset."""

    def __init__(self):
        self.H = lk.History()
        self.reset()

    def reset(self) -> None:
        self.H.reset()
        self.last, self.n, self.trace = None, 0, []

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        H.peak("neck_max", m["neck"])
        H.peak("pitch_max", m["pitch"])
        H.peak("roll_max", m["roll"])
        H.peak("rise_max", min(m["rise_b"], m["rise_h"]))
        lifted = m["rise_b"] >= LIFT_M and m["rise_h"] >= LIFT_M
        H.ever("lifted", lifted)
        H.ever("carried", lifted and m["bx"] <= CARRIED_X)
        H.held("final", on_pad(m) and m["speed"] < STILL_MS and m["spin"] < SPIN_DS and clear_and_still(m))


def clear_and_still(m: dict) -> bool:
    return (min(m["clear"].values()) >= CLEAR_M - EPS and max(m["tcp_speed"].values()) < TCP_STILL)


def on_pad(m: dict) -> bool:
    """Lying on the pad: body and head resting on it, centre line over it, face up, neck nearly straight."""
    return (abs(m["bx"] - PAD_XC) <= ON_PAD_X and abs(m["hx"] - PAD_XC) <= ON_PAD_X
            and abs(m["by"] - BODY_Y) <= ON_PAD_Y
            and REST_TOL_M[0] <= m["low_b"] <= REST_TOL_M[1] and abs(m["low_h"]) <= HEAD_TOL_M
            and m["up_deg"] <= SUPINE_DEG and m["neck"] <= NECK_END_DEG)


def ladder(rec: Record) -> tuple[float, str]:
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if H.peaks["neck_max"] > NECK_GATE_DEG + EPS:
        return 0.0, f"gate: neck bent {H.peaks['neck_max']:.1f} deg (limit {NECK_GATE_DEG})"
    if max(H.peaks["pitch_max"], H.peaks["roll_max"]) > TILT_GATE_DEG + EPS:
        return 0.0, (f"gate: body tilted {max(H.peaks['pitch_max'], H.peaks['roll_max']):.1f} deg "
                     f"(limit {TILT_GATE_DEG})")
    if "lifted" not in H.evers:
        return 0.0, "the manikin was never lifted 35 mm clear of the mattress with head and body together"
    if "carried" not in H.evers:
        return 0.25, "lifted, never carried over the pad"
    if not on_pad(m):
        return 0.5, "carried over the pad; not lying on it at the end"
    if H.runs.get("final", 0.0) >= RELEASE_S - EPS:
        return 1.0, "lying on the pad, still, both grippers 30 mm clear and still for 1 s"
    return 0.75, "lying on the pad; grippers not yet 30 mm clear and still for 1 s, or the manikin not still"


REC = Record()
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    s = tick.states["manikin"]
    B = np.array(s.poses["body"], dtype=float)
    sp = spin = 0.0
    if _PREV:
        sp, spin = pose_rates(_PREV["B"], B, tick.t - _PREV["t"])
    _PREV.update(B=B, t=tick.t)
    m = measure(s, tick.tools, sp, spin)
    REC.update(m)
    if REC.n % 5 == 0:
        REC.trace.append((round(tick.t, 2), round(m["neck"], 1), round(m["pitch"], 1), round(m["rise_b"] * 1000, 0),
                          round(m["rise_h"] * 1000, 0), round(m["bx"], 3), round(m["by"], 3),
                          [round(1000 * m["clear"][k], 0) for k in ("left", "right")]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


def demo_report() -> dict:
    sc, why = ladder(REC)
    H, m = REC.H, REC.last or {}
    return {"score": sc, "reason": why, "variant": VARIANT,
            "evers": {k: round(v, 2) for k, v in H.evers.items()},
            "peaks": {k: round(v, 2) for k, v in H.peaks.items()},
            "runs": {k: round(v, 2) for k, v in H.runs.items()},
            "last": {k: (round(v, 4) if isinstance(v, float) else v) for k, v in m.items()},
            "log": dict(LOG), "trace": REC.trace}


# -- scripted demo ---------------------------------------------------------------------------------------------------
LOG: dict = {}
HEAD_GRIP_W, HIP_GRIP_W = 0.080, 0.040  # commanded opening: pads close to a light squeeze (head) / a firm one (hips)
LIFT = 0.06
SERVO_GAIN = 0.05  # per control step, on the seen body position error


def demo_script(ctx):
    """Left arm holds the head at the ears, right arm holds the hips; both lift, carry 21.5 cm toward the robot, set the
    manikin on the pad and let go. Reads only what a camera could measure: the body and head poses."""
    import linkdemo as ld

    variant = VARIANT
    LOG.clear()
    LOG["variant"] = variant
    yield from ctx.wait(1.0)
    if variant == "nothing":
        yield from ctx.wait(8.0)
        return
    seen = ctx.observe()
    hp, bp = seen.pose("manikin", "head")[:3, 3], seen.pose("manikin", "body")[:3, 3]
    rot = ld.grip_frame((0, 0, -1), (1, 0, 0))
    lift = 0.008 if variant == "slide" else LIFT
    z_head, z_hip = ZM + HALF_T + 0.001, ZM + 0.040
    dx = PAD_XC - XC
    drop = lift + (ZM - ZP) - 0.003  # set down 3 mm above the pad

    legs = [(0.0, 0.0, lift, 1.5), (dx, 0.0, 0.0, 3.0), (0.0, 0.0, -drop, 1.5)]  # dx, dy, dz, seconds
    marks, t_end, cur = [], 0.0, np.zeros(3)
    for ddx, ddy, ddz, dur in legs:
        nxt = cur + np.array([ddx, ddy, ddz])
        marks.append((t_end, t_end + dur, cur, nxt))
        t_end, cur = t_end + dur, nxt
    PATH_S = t_end

    def nominal_at(t):  # the planned lift, carry and set-down, as an offset from the grip
        t = min(t, PATH_S)
        for a, b, c0, c1 in marks:
            if t <= b + 1e-9:
                return c0 + ld.smooth((t - a) / (b - a)) * (c1 - c0)

    def approach(arm, p, z, w):
        px, py = float(p[0]), float(p[1])
        top = z + 0.07
        yield from arm.move((px, py, top), rot, width=ld.OPEN_WIDTH, duration=2.5)
        yield from arm.move((px, py, z), rot, duration=1.5)
        yield from arm.grip(w, 0.8)
        yield from arm.wait(0.5)
        return px, py, top

    def leave(arm, pos, top):
        yield from arm.wait(0.5)
        yield from arm.open(0.8)
        yield from arm.wait(0.3)
        yield from arm.move((pos[0], pos[1], top), duration=1.5)

    def hips():  # drives the lift, the carry and the set-down; the loaded arm sags, so it servos on the seen body pose
        arm = ctx.right
        px, py, top = yield from approach(arm, bp, z_hip, HIP_GRIP_W)
        p0 = np.array([px, py, z_hip])
        b0 = ctx.observe().pose("manikin", "body")[:3, 3].copy()
        corr = np.zeros(3)
        state = {"t": 0.0}

        def at(_s):
            state["t"] += 1.0 / 20.0
            nominal = nominal_at(state["t"])
            err = (b0 + nominal) - ctx.observe().pose("manikin", "body")[:3, 3]
            corr[:] = np.clip(corr + SERVO_GAIN * err, -0.08, 0.08)
            state["p"] = p0 + nominal + corr
            return state["p"], rot, HIP_GRIP_W

        yield from arm.follow(at, PATH_S)
        yield from leave(arm, state["p"][:2], top)

    def head():  # keeps the head where the hips arm takes the body: it follows the seen body pose
        arm = ctx.left
        px, py, top = yield from approach(arm, hp, z_head, HEAD_GRIP_W)
        p0 = np.array([px, py, z_head])
        b0 = ctx.observe().pose("manikin", "body")[:3, 3].copy()
        state = {"p": p0.copy()}
        clock = {"t": 0.0}

        def at(_s):  # low-passed so the seen body's wobble does not shake the head
            clock["t"] += 1.0 / 20.0
            if variant == "left_only":  # no hips arm to follow: the head arm alone flies the whole plan
                want = p0 + nominal_at(clock["t"])
            else:
                want = p0 + (ctx.observe().pose("manikin", "body")[:3, 3] - b0)
            state["p"] = state["p"] + 0.25 * (want - state["p"])
            return state["p"], rot, HEAD_GRIP_W

        yield from arm.follow(at, PATH_S + 0.5)
        yield from leave(arm, state["p"], top)

    gens = []
    if variant != "left_only":  # the right arm holds the hips
        gens.append(hips())
    if variant != "right_only":  # the left arm holds the head
        gens.append(head())
    yield from ctx.both(*gens)
    yield from ctx.wait(3.0)
