# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Check that a seat-belt retractor locks on a sudden pull.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A seat-belt emergency-locking retractor (steel frame about 90 x 70 x 110 mm, 0.5 kg, on a grey steel test plate
  fixed to the table) sits at x 0.55, y -0.12 with its webbing slot facing the robot, 60 mm above the table. 46 mm wide
  black polyester webbing runs out of the slot to a silver steel tongue plate (85 x 45 x 4 mm).
- A return spring draws the webbing back (about 0.1 N per cm of pull-out, plus damping). A webbing-speed lock catches
  the spool when the webbing is yanked (it speeds up by 0.10 m/s to at least 0.15 m/s in one 0.05 s step); the lock
  holds against a continued pull; it lets go when no hand holds the tongue and the spring winds the webbing back in.

Goal: pull the webbing out 150 mm at a steady pace, jerk it so that it locks, keep pulling against the lock, then let
go so it winds back in.
"""

from __future__ import annotations

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
DT = 0.05  # control step, 20 Hz

# -- the retractor -----------------------------------------------------------------------------------------------
BELT_POS = (0.55, -0.12, TOP)
SLOT_Z = 0.060  # webbing centre above the table
WEB_W, WEB_T = 0.046, 0.0012
TONGUE = (0.085, 0.045, 0.004)  # x (along the webbing), y (width), z
TONGUE_X = 0.100 + TONGUE[0] / 2  # tongue centre from the slot at rest (0.1 m of webbing outside)
TRAVEL = 0.30
SPRING_K, SPRING_REST, DAMP = 10.0, -0.02, 5.0  # N/m, m (the stop is at 0), N s/m
LOCK_K, LOCK_C = 20000.0, 200.0  # N/m, N s/m

BLACK, STEEL, PLATE = (0.08, 0.08, 0.09), (0.78, 0.79, 0.82), (0.50, 0.52, 0.55)

_t = lk.Tree("belt", pos=BELT_POS, yaw_deg=180.0, fixed=True, color=PLATE)
_t.link("frame", [lk.box((0.20, 0.16, 0.008), (-0.045, 0, 0.004)),
                  lk.box((0.090, 0.070, 0.110), (-0.045, 0, 0.063), color=(0.30, 0.31, 0.34))], mass=0.5)
_t.link("web", [lk.box((0.40, WEB_W, WEB_T), (-0.10, 0, 0), color=BLACK, contact_offset=0.001),
                lk.box(TONGUE, (TONGUE_X, 0, 0), color=STEEL, contact_offset=0.001)],
        mass=0.15, at=(0.0, 0.0, SLOT_Z))
_t.prismatic("pull", "frame", "web", anchor=(0.0, 0.0, SLOT_Z), axis=(1.0, 0.0, 0.0), limits=(0.0, TRAVEL),
             rest=SPRING_REST, stiffness=SPRING_K, damping=DAMP, brake=lk.Brake(stiffness=LOCK_K, damping=LOCK_C))
BELT = _t.build()
OBJECTS = [BELT]

# -- scoring constants (mm and m/s: every one is read from webbing marks and a hand marker on a video) -------------
SLOW_E = 0.150  # pulled out at least this far ...
SLOW_V = 0.20  # ... at under this speed (webbing speed over 0.1 s)
JERK_V = 0.15  # a jerk: the hand moves at least this fast over one 0.05 s step ...
JERK_GAIN = 0.10  # ... having gained at least this much speed in that step (2 m/s2, about 0.2 g)
RUNON_MM = 15.0  # the locked webbing may run on at most this far from where the hand started the jerk
RESIST_M = 0.045  # the hand must keep pulling this far past the jerk start, holding the tongue
HOME_MM = 5.0  # wound back in to within this at the end
CLEAR_M, RELEASE_S, TCP_STILL = 0.030, 1.0, 0.005
HOLD_M, HOLD_OPEN = 0.004, 0.060  # a hand holds the tongue: pads within 4 mm of it, opened under 60 mm
EPS = 1e-6


def speed(es: list) -> float:
    """Pay-out speed (m/s) over the last 0.1 s from the last three extension samples (what marks and video give)."""
    return (es[-1] - es[-3]) / (2 * DT) if len(es) >= 3 else 0.0


class Record:
    """Whole-run record, cleared only by the simulator reset. It follows attempts: an attempt starts when the webbing
    leaves the slot and ends when it is back home, so a wrong first try that was let go and wound in does not spoil a
    later correct one (the best finished attempt counts). Pure Python."""

    def __init__(self):
        self.H = lk.History()
        self.reset()

    def reset(self) -> None:
        self.H.reset()
        self.es: list = []
        self.last: dict | None = None
        self.v = 0.0  # webbing speed over 0.1 s
        self.v1 = 0.0  # webbing speed over the last step
        self.v1_prev = 0.0
        self.trace: list = []
        self.prev_h: dict = {}  # side -> hand pull coordinate last step (None when not holding the tongue)
        self.prev_vh: dict = {}
        self.bank = 0.0  # best finished attempt: 0.75 when it was good and the webbing came home
        self.new_attempt()

    def new_attempt(self) -> None:
        self.att = {"slow": False, "vslow": 0.0, "onset": False, "early": False, "e_on": 0.0, "h_on": 0.0,
                    "side": None, "emax": 0.0, "resisted": False, "steps": 0, "travel": 0.0}

    def good(self) -> bool:
        a = self.att
        return (a["onset"] and a["slow"] and not a["early"] and a["resisted"]
                and (a["emax"] - a["e_on"]) * 1000 <= RUNON_MM + EPS)

    def update(self, m: dict, t: float | None = None) -> None:
        H, a = self.H, self.att
        if t is not None:
            H._tick(t)
        e = m["e"]
        e_prev = self.es[-1] if self.es else e
        self.es = (self.es + [e])[-3:]
        self.last, self.v = m, speed(self.es)
        self.v1_prev, self.v1 = self.v1, (e - e_prev) / DT
        H.peak("e_max", e)
        H.peak("v_max", self.v)
        hands = m.get("hands", {})  # side -> pull coordinate h (m) if that hand holds the tongue, else None
        if a["onset"]:
            a["steps"] += 1
            if e * 1000 <= HOME_MM + EPS and a["steps"] >= 3:  # attempt over: the webbing is home again
                if self.good():
                    self.bank = 0.75
                self.new_attempt()
                a = self.att
        # hand jerk onset: a holding hand moves fast over one step and has just gained speed
        jerk_side = None
        for side, h in hands.items():
            ph = self.prev_h.get(side)
            if h is not None and ph is not None:
                vh = (h - ph) / DT
                if vh >= JERK_V - EPS and vh - self.prev_vh.get(side, 0.0) >= JERK_GAIN - EPS:
                    jerk_side = side
                self.prev_vh[side] = vh
            else:
                self.prev_vh[side] = 0.0
        if not a["onset"]:
            if jerk_side is not None:
                a.update(onset=True, side=jerk_side, e_on=e_prev, h_on=self.prev_h[jerk_side], early=not a["slow"],
                         emax=e, steps=0)
                H.ever("jerk", True)
            elif not a["slow"]:
                a["vslow"] = max(a["vslow"], self.v)
                if e >= SLOW_E and a["vslow"] < SLOW_V:
                    a["slow"] = True
        else:
            a["emax"] = max(a["emax"], e)
            h = hands.get(a["side"])
            if h is not None:
                a["travel"] = max(a["travel"], h - a["h_on"])
                if a["travel"] >= RESIST_M - EPS and (a["emax"] - a["e_on"]) * 1000 <= RUNON_MM + EPS:
                    a["resisted"] = True
        self.prev_h = dict(hands)
        clear = m["clear"]
        H.held("released", bool(clear) and min(clear.values()) >= CLEAR_M and max(m["tcp_speed"].values()) < TCP_STILL)

    def lock_now(self) -> bool:
        """The retractor's own trip: the webbing is yanked, i.e. it is paying out fast and has just sped up."""
        return self.v1 >= JERK_V - EPS and self.v1 - self.v1_prev >= JERK_GAIN - EPS


def ladder(rec: Record) -> tuple[float, str]:
    H, m, a = rec.H, rec.last, rec.att
    if m is None:
        return 0.0, "no steps yet"
    home = m["e"] * 1000 <= HOME_MM + EPS
    clear = H.runs.get("released", 0.0) >= RELEASE_S - EPS
    if rec.bank >= 0.75:
        if home and clear:
            return 1.0, "steady 150 mm pull-out, jerk locked the webbing, the hand kept pulling against it, let go, home"
        if home:
            return 0.75, "locked, resisted a continued pull, wound back in; grippers not yet 30 mm clear and still 1 s"
        return 0.5, "an earlier attempt was good; the webbing is out again"
    if rec.good():
        return 0.5, "steady pull, jerk, the lock held against a continued pull; not yet let go and wound back in"
    if not a["slow"] and not a["onset"]:
        return 0.0, "no steady 150 mm pull-out"
    if a["onset"] and a["early"]:
        return 0.0, f"the webbing was jerked at {a['e_on'] * 1000:.0f} mm, before 150 mm had come out steadily"
    if not a["onset"]:
        return 0.0, "the webbing was never jerked (hand over 0.15 m/s having just sped up 0.10 m/s in one 0.05 s step)"
    if (a["emax"] - a["e_on"]) * 1000 > RUNON_MM + EPS:
        return 0.0, "the webbing ran on more than 15 mm after the jerk started: the lock did not hold"
    return 0.0, f"the hand did not keep pulling past the jerk ({a['travel'] * 1000:.0f} of 45 mm): no test of the lock"


REC = Record()
LOCK = {"on": False, "free": 0}


@lk.on_reset
def _reset(tick):
    REC.reset()
    LOCK.update(on=False, free=0)


@lk.on_step
def _track(tick):
    s = tick.states["belt"]
    clear = {side: lk.clearance(tool, s) for side, tool in tick.tools.items()}
    held = {side: t.opening < HOLD_OPEN and lk.clearance(t, s, ["web"]) < HOLD_M for side, t in tick.tools.items()}
    hands = {side: (-float(tick.tools[side].tcp[0]) if held[side] else None) for side in held}  # pulls toward -x
    REC.update({"e": float(s.q["pull"]), "clear": clear, "tcp_speed": {k: t.speed for k, t in tick.tools.items()},
                "hands": hands})
    # the lock: catches the spool when the webbing is yanked, lets go once no hand holds the webbing
    anyheld = any(held.values())
    if not LOCK["on"] and REC.lock_now():
        lk.brake("belt", "pull", True)
        LOCK["on"], LOCK["free"] = True, 0
    elif LOCK["on"]:
        LOCK["free"] = 0 if anyheld else LOCK["free"] + 1
        if LOCK["free"] >= 5:
            lk.brake("belt", "pull", False)
            LOCK["on"] = False
    if len(REC.trace) < 4000 and REC.last:
        REC.trace.append((round(tick.t, 2), round(1000 * REC.last["e"], 1), round(REC.v1, 3), int(LOCK["on"]),
                          [None if h is None else round(1000 * h, 1) for h in hands.values()]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Reads only what a camera could measure: the webbing link's pose (its pull-out from marks) and the robot's own
# grippers; never the lock state or the record.

VARIANTS = ("", "nothing", "slow_only", "jerk_only", "short_slow", "soft_pull", "stop_release", "left_only")
LOG: dict = {}
SLOW_SPEED = 0.08  # m/s for the steady pull
JERK_STEPS = (0.012, 0.018, 0.018, 0.006, 0.006, 0.006)  # the jerk (12 mm in the first step) and the pull that goes on
SHORT_JERK = (0.012, 0.012)  # the cheat: a jerk, then the hand stops
TCP_DZ = 0.0095 + TONGUE[2] / 2  # TCP above the tongue's centre: pad tips 3.5 mm into its 4 mm edge
GRIP_OPEN, GRIP_CLOSED = 0.075, 0.043


def _pullout(ctx, p0) -> float:
    return float(np.linalg.norm(ctx.observe().pose("belt", "web")[:3, 3] - p0))


def demo_script(ctx):
    variant = VARIANT
    LOG.clear()
    LOG["variant"] = variant
    yield from ctx.wait(1.0)
    if variant == "nothing":
        yield from ctx.wait(8.0)
        return
    from linkdemo import down

    arm = ctx.left if variant == "left_only" else ctx.right
    pose0 = ctx.observe().pose("belt", "web")
    p0 = pose0[:3, 3].copy()
    d = pose0[:3, :3] @ np.array([1.0, 0.0, 0.0])  # pull-out direction (toward the robot)
    LOG["d"] = [round(float(v), 3) for v in d]
    tongue = pose0[:3, :3] @ np.array([TONGUE_X, 0.0, 0.0]) + p0
    LOG["tongue"] = [round(float(v), 3) for v in tongue]
    rot = down(0.0)
    top = tongue - 0.030 * d + np.array([0.0, 0.0, TCP_DZ])  # held at the slot end: room to slip toward the free end
    yield from arm.move(top + np.array([0.0, 0.0, 0.08]), rot, width=GRIP_OPEN, duration=2.0)
    yield from arm.move(top, rot, duration=1.2)
    yield from arm.grip(GRIP_CLOSED, 0.8)
    yield from arm.wait(0.4)
    pos = arm.pos.copy()

    def drive(speed, until):
        nonlocal pos
        while _pullout(ctx, p0) < until:
            pos = pos + d * speed * DT
            arm._solve(pos, rot, arm.width)
            yield

    def jerk(steps=JERK_STEPS):
        nonlocal pos
        for st in steps:
            pos = pos + d * st
            arm._solve(pos, rot, arm.width)
            yield

    if variant == "jerk_only":
        yield from jerk()
    elif variant == "short_slow":
        yield from drive(SLOW_SPEED, 0.08)
        yield from arm.wait(0.5)
        yield from jerk()
    else:
        yield from drive(SLOW_SPEED, SLOW_E + 0.015)
        LOG["e_slow_mm"] = round(1000 * _pullout(ctx, p0), 1)
        yield from arm.wait(0.6)
        if variant == "soft_pull":
            yield from drive(0.12, SLOW_E + 0.075)
        elif variant == "stop_release":
            yield from jerk(SHORT_JERK)
        elif variant != "slow_only":
            yield from jerk()
    LOG["e_end_pull_mm"] = round(1000 * _pullout(ctx, p0), 1)
    yield from arm.wait(0.4)
    yield from arm.grip(GRIP_OPEN, 0.5)  # let go: the lock frees and the spring winds the webbing in
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.08]), duration=0.8)
    yield from arm.wait(0.5)
    yield from arm.move(arm.pos + np.array([0.0, 0.10, 0.0]), duration=1.0)  # clear of the tongue
    yield from ctx.wait(4.0)
    LOG["e_final_mm"] = round(1000 * _pullout(ctx, p0), 1)


def demo_report() -> dict:
    s, why = ladder(REC)
    H = REC.H
    return {"score": s, "reason": why, "variant": VARIANT, "evers": {k: round(v, 2) for k, v in H.evers.items()},
            "peaks": {k: round(v, 3) for k, v in H.peaks.items()},
            "values": {k: round(v, 4) for k, v in H.values.items()},
            "runs": {k: round(v, 2) for k, v in H.runs.items()}, "log": dict(LOG), "trace": REC.trace}
