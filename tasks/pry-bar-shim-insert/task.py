# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Lever a door's hinge side up with a flat pry bar on a fulcrum and slide a packer under it, then let go.

Station table (it stands in for the floor); world frame in metres from the midpoint between the two arm bases, x
forward, y to the robot's left, z up; the table top is at z 0.7475.

- A pine cabinet door leaf, 28 x 350 x 500 mm, 2.5 kg, hangs on two lift-off hinge pins on a jamb post at its left
  edge (the robot's left, y +0.175). The pins let it slide up 20 mm; its hinge washers stop it 4.5 mm above the floor.
  The pins are greased (damping 100 N s/m, 0.5 N friction). The door faces the robot, its front face at x 0.286.
- A red steel flat pry bar, 30 x 4 mm, with a 160 mm handle and a 45 mm foot (3 mm thick) bent down to the floor, is
  pinned between two cheek plates of a fulcrum bolted to the floor at x 0.261, y 0.15. The foot lies 20 mm under the
  door's hinge-side bottom edge (1 mm below it). Pressing the handle down tilts the foot up and lifts the door.
- A yellow plastic packer, 100 x 40 x 6 mm, 34 g, lies on the floor in front of the door at y 0.01.

Goal: the packer under the hinge side of the door, the door resting on it, both grippers let go.
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

# -- the door --------------------------------------------------------------------------------------------------

DX, DY, DOOR_T, DOOR_W, DOOR_H = 0.300, 0.0, 0.028, 0.350, 0.500
FRONT, BACK = DX - DOOR_T / 2, DX + DOOR_T / 2  # 0.286, 0.314
GAP0 = 0.0045  # m, door bottom above the floor on its hinge washers
LIFT_MAX = 0.020
HINGE_Y = DOOR_W / 2  # 0.175 (world y 0.175)
WOOD, DARK, RED, STEEL, YELLOW = (0.80, 0.62, 0.40), (0.55, 0.40, 0.25), (0.80, 0.10, 0.10), (0.55, 0.57, 0.60), (0.95, 0.80, 0.10)

_t = lk.Tree("door", pos=(DX, DY, TOP), fixed=True, color=WOOD)
_t.link("jamb", [lk.box((0.040, 0.020, 0.520), (0.0, HINGE_Y + 0.010, 0.260), color=DARK)], mass=1.0)
_t.link("leaf", [lk.box((DOOR_T, DOOR_W, DOOR_H))], mass=2.5, at=(0.0, 0.0, GAP0 + DOOR_H / 2),
        friction=(0.4, 0.3), contact_offset=0.001)
_t.prismatic("lift", "jamb", "leaf", anchor=(0.0, HINGE_Y, GAP0 + DOOR_H / 2), axis=(0, 0, 1), limits=(0.0, LIFT_MAX),
             damping=100.0, friction=0.5)
DOOR = _t.build()

# -- the fulcrum and pry bar ------------------------------------------------------------------------------------

PX, PY, PZ = 0.261, 0.15, 0.040  # pivot axis (along y) at 40 mm above the floor
FOOT_X = 0.045  # foot tip, from the pivot
HANDLE = 0.160
_t = lk.Tree("bar", pos=(PX, PY, TOP), fixed=True, color=STEEL)
_t.link("post", [lk.box((0.030, 0.005, 0.054), (0.0, 0.0195, 0.027)), lk.box((0.030, 0.005, 0.054), (0.0, -0.0195, 0.027))],
        mass=0.5)
_t.link("pry", [lk.box((HANDLE + 0.005, 0.030, 0.004), (-HANDLE / 2 + 0.0025, 0.0, 0.0)),
                lk.box((0.008, 0.030, 0.0415), (0.0, 0.0, -0.0185)),
                lk.box((FOOT_X - 0.004, 0.030, 0.003), ((FOOT_X + 0.004) / 2, 0.0, -0.038))],
        mass=0.28, at=(0.0, 0.0, PZ), color=RED, friction=(0.4, 0.3), contact_offset=0.001)
_t.revolute("pivot", "post", "pry", anchor=(0.0, 0.0, PZ), axis=(0, -1, 0), limits_deg=(0.0, 20.0), init_deg=1.0,
            damping=0.02, friction=0.02)
BAR = _t.build()
GRIP_LOCAL = (-0.140, 0.0, 0.002)  # handle top face, 140 mm behind the pivot

# -- the packer ---------------------------------------------------------------------------------------------------

PK_L, PK_W, PK_T = 0.100, 0.040, 0.006
PK0 = (0.220, 0.010)
_t = lk.Tree("packer", pos=(PK0[0], PK0[1], TOP + PK_T / 2), color=YELLOW)
_t.link("body", [lk.box((PK_L, PK_W, PK_T))], mass=0.034, friction=(0.5, 0.4), contact_offset=0.001)
PACKER = _t.build()
OBJECTS = [DOOR, BAR, PACKER]

# -- scoring -----------------------------------------------------------------------------------------------------

Q_UP = 0.007  # m: door lifted this far above its hinge washers (gap under it 11.5 mm or more)
F_PART = 0.296  # packer front edge: at least 10 mm under the door's 28 mm thickness
F_SEAT, F_MAX = 0.306, 0.345  # front edge at least 20 mm under the door and no more than 31 mm past its back face
Y_LO, Y_HI = 0.0, 0.09  # packer centre across the door: on the hinge half, clear of the pry bar foot
YAW_MAX, TILT_MAX, Z_TOL = 12.0, 5.0, 0.0015
REST_LO, REST_HI = 0.0008, 0.0035  # door lift when it rests on the 6 mm packer is 1.5 mm
STILL_MS = 0.002
STILL_S = 1.0
CLEAR_M = 0.03
TCP_STILL = 0.005
RELEASE_S = 1.0
EPS = 1e-6


class Record:
    """Whole-run record (a linkkit History updated every control step, cleared only by the simulator reset) plus the
    latest sample. Pure Python."""

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
        self.last = m
        H.peak("q_max", m["q"])
        H.peak("front_max", m["front"])
        H.ever("part_up", m["q"] >= Q_UP and m["front"] >= F_PART)
        H.ever("seat_up", m["q"] >= Q_UP and seated(m))
        H.held("still", m["dspeed"] < STILL_MS)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def seated(m: dict) -> bool:
    """The packer lies flat on the floor across the door's bottom edge, on the hinge half."""
    yaw = abs(m["yaw"]) % 180.0
    return (F_SEAT <= m["front"] <= F_MAX and Y_LO <= m["y"] <= Y_HI and min(yaw, 180.0 - yaw) <= YAW_MAX
            and m["tilt"] <= TILT_MAX and abs(m["z"] - PK_T / 2) <= Z_TOL)


def ladder(rec: Record) -> tuple[float, str]:
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "part_up" not in H.evers:
        return 0.0, "the packer was never 10 mm under the door while the door was lifted 7 mm"
    if "seat_up" not in H.evers:
        return 0.25, "the packer went 10 mm under the lifted door but was never fully under it"
    if seated(m) and REST_LO <= m["q"] <= REST_HI:
        ends = {
            "door still 1 s": H.runs.get("still", 0.0) >= STILL_S - EPS,
            "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS,
        }
        if all(ends.values()):
            return 1.0, "packer under the hinge side, door resting on it, both grippers let go"
        return 0.75, "door resting on the packer at the end; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    return 0.5, "the packer was fully under the lifted door at some time, but the door does not rest on it at the end"


def measure(door, bar, pack, tools) -> dict:
    P = pack.poses["body"]
    R, c = P[:3, :3], P[:3, 3]
    xs = [float((c + R @ np.array([s * PK_L / 2, 0.0, 0.0]))[0]) for s in (-1, 1)]
    return {
        "q": float(door.q["lift"]),
        "lever": float(bar.deg("pivot")),
        "front": max(xs),
        "rear": min(xs),
        "y": float(c[1]),
        "yaw": math.degrees(math.atan2(R[1, 0], R[0, 0])),
        "tilt": math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2]))))),
        "z": float(c[2] - TOP),
        "dspeed": 0.0,
        "clear": {s: float(min(lk.clearance(tl, st) for st in (door, bar, pack))) for s, tl in tools.items()},
        "tcp_speed": {s: float(tl.speed) for s, tl in tools.items()},
    }


REC = Record()
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    st = tick.states
    m = measure(st["door"], st["bar"], st["packer"], tick.tools)
    if _PREV:
        m["dspeed"] = abs(m["q"] - _PREV["q"]) / max(tick.t - _PREV["t"], 1e-6)
    _PREV.update(q=m["q"], t=tick.t)
    REC.update(m)
    if REC.n % 5 == 0:
        REC.trace.append((round(tick.t, 2), round(1000 * m["q"], 1), round(m["lever"], 1), round(1000 * m["front"], 1),
                          round(1000 * m["y"], 1), round(m["yaw"], 1), round(1000 * m["z"], 1),
                          [round(1000 * m["clear"][s]) for s in ("left", "right")],
                          [round(1000 * float(v)) for v in tick.tools["right"].tcp]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


def demo_report() -> dict:
    s, why = ladder(REC)
    H = REC.H
    return {"variant": VARIANT, "score": s, "why": why, "q_max_mm": round(1000 * H.peaks.get("q_max", 0), 2),
            "front_max_mm": round(1000 * H.peaks.get("front_max", 0), 2), "last": REC.last, "evers": H.evers,
            "runs": H.runs, "trace": REC.trace, "log": LOG}


# -- scripted demo -----------------------------------------------------------------------------------------------

VARIANTS = ("", "nothing", "left_only", "right_only", "release_early")
LOG: dict = {}
LEVER_DEG = float(os.environ.get("LEVER_DEG", "13"))
PUSH_Z = float(os.environ.get("PUSH_Z", "0.0025"))  # pad tip height above the floor when pushing the packer
PUSH_TO = float(os.environ.get("PUSH_TO", "0.326"))  # packer front edge target (m)
PUSH_YAW = float(os.environ.get("PUSH_YAW", "90"))


def demo_script(ctx):
    import linkdemo as ld
    from yam_arena.constants import PAD_TIP_PAST_TCP as PT

    yield from ctx.wait(1.0)
    R = ld.down(0.0)
    pad = np.array([0.0, 0.0, PT])
    state = {"packer_in": False}

    def handle():
        return ctx.observe().point("bar", "pry", GRIP_LOCAL)

    def left(hold_s=1.0, early=False):
        arm = ctx.left
        yield from arm.grip(0.0, 0.5)
        h = handle()
        yield from arm.move(h + pad + [0, 0, 0.07], R, duration=2.0)
        yield from arm.move(h + pad + [0, 0, 0.003], R, duration=1.0)
        depth = 0.0
        for _ in range(14):  # press until the seen bar angle reaches the target
            err = LEVER_DEG - ctx.observe().angle_deg("bar", "pivot")
            if err <= 0.4:
                break
            depth += max(0.002, 0.0012 * err)
            yield from arm.move(handle() + pad + [0, 0, 0.003 - depth], R, duration=0.7)
            yield from arm.wait(0.2)
        LOG["bar_deg_pressed"] = round(ctx.observe().angle_deg("bar", "pivot"), 2)
        LOG["press_depth_mm"] = round(1000 * depth, 1)
        if early:
            yield from arm.wait(0.5)
        else:
            while not state["packer_in"]:
                yield
            yield from arm.wait(hold_s)
        up = arm.pos.copy()
        yield from arm.move(up + [0, 0, 0.03], R, duration=2.0)  # ease up: the door comes down onto the packer
        yield from arm.wait(1.0)
        yield from arm.move(up + [0, 0, 0.14], R, duration=1.5)
        yield from arm.wait(0.5)

    def right(delay=0.0, wait_bar=True):
        arm = ctx.right
        R = ld.down(PUSH_YAW)
        yield from arm.grip(0.0, 0.5)
        rear = ctx.observe().point("packer", "body", (-PK_L / 2, 0.0, 0.0))
        start = np.array([rear[0] - 0.010, rear[1], TOP + PUSH_Z])
        yield from arm.move(start + pad + [0, 0, 0.06], R, duration=2.0)  # wait just behind the packer, pad down
        yield from arm.move(start + pad, R, duration=1.0)
        LOG["right_at_start"] = [round(float(v), 4) for v in arm.pos]
        if wait_bar:
            while ctx.observe().angle_deg("bar", "pivot") < LEVER_DEG - 2.0:
                yield
        yield from arm.wait(delay)
        front0 = ctx.observe().point("packer", "body", (PK_L / 2, 0.0, 0.0))[0]
        end = start + np.array([PUSH_TO - front0, 0.0, 0.0])
        yield from arm.move(end + pad, R, duration=3.0)
        LOG["right_at_end"] = [round(float(v), 4) for v in arm.pos]
        LOG["end_cmd"] = [round(float(v), 4) for v in end + pad]
        LOG["front_mm_after_push"] = round(1000 * ctx.observe().point("packer", "body", (PK_L / 2, 0.0, 0.0))[0], 1)
        state["packer_in"] = True
        yield from arm.wait(1.0)
        yield from arm.move(end + pad + [-0.06, 0, 0], R, duration=1.5)
        yield from arm.move(end + pad + [-0.06, 0, 0.09], R, duration=1.0)
        yield from arm.wait(0.5)

    if VARIANT == "nothing":
        yield from ctx.wait(20.0)
        return
    if VARIANT == "left_only":
        state["packer_in"] = True
        yield from left(hold_s=3.0)
        yield from ctx.wait(3.0)
        return
    if VARIANT == "right_only":
        yield from right(wait_bar=False)
        yield from ctx.wait(3.0)
        return
    if VARIANT == "release_early":  # the bar is let go before the packer is pushed; the door has closed on it
        yield from ctx.both(left(early=True), right(delay=5.0))
        yield from ctx.wait(3.0)
        return
    yield from ctx.both(left(), right())
    yield from ctx.wait(3.0)
