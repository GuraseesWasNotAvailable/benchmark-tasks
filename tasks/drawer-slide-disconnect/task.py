# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Take a drawer out of its ball-bearing slides by working both disconnect levers at once.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A plywood drawer bay (300 x 316 x 200 mm, front at x 0.4115) is fixed to the table on an 85 mm plinth. A drawer
  (280 x 240 x 75 mm, 1.2 kg) hangs on two ball-bearing slides, standing pulled out 199 mm, against the end stop.
- Each slide has a spring-return disconnect lever on its inner member. The left (+y) lever is a tab under the
  drawer's left wall that is pushed UP; the right (-y) lever is a tab over the right wall that is pushed DOWN. Both
  tabs move 6 mm and a lever counts as pressed at 4.5 mm. The end stop only clears while BOTH levers are pressed (one
  alone leaves the other slide locked); once the inner members are past the stop they run free to 34 cm.
- The drawer is out when it is 280 mm or more from closed, at rest.
"""

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

B = 0.4115  # bay front, x
Z0 = 0.10  # drawer bottom above the table
START_EXT = 0.199
STOP = 0.200  # latched stop (m of extension)
LO = 0.12
FREE_HI = 0.34
PAST = 0.215  # rung: drawer seen past the stop
CLEARED = 0.280  # rung: drawer out
DOG = 0.204  # inner members past the stop dog
TRAVEL = 0.006
ON = 0.0045  # lever pressed
CLEAR_M = 0.03
TCP_STILL = 0.01
DRAWER_STILL = 0.005
RELEASE_S = 1.0
STILL_S = 1.0
EPS = 1e-6

WOOD = (0.70, 0.55, 0.35)
PLY = (0.86, 0.76, 0.56)
STEEL = (0.62, 0.63, 0.66)
RED = (0.85, 0.15, 0.12)
BLUE = (0.15, 0.30, 0.85)


def bx(x0, x1, y0, y1, z0, z1):
    """Box from drawer-frame ranges (m); z is measured from the drawer bottom, lifted by Z0 to the tree frame."""
    return lk.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2 + Z0))


def bay_box(x0, x1, y0, y1, z0, z1):
    return lk.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


_t = lk.Tree("drawer", pos=(B, 0.0, TOP), fixed=True, color=WOOD)
_t.link("bay", [
    bay_box(0.0, 0.30, -0.158, 0.158, 0.0, 0.085),                  # plinth
    bay_box(0.0, 0.30, 0.1454, 0.158, 0.085, 0.20),                 # side walls
    bay_box(0.0, 0.30, -0.158, -0.1454, 0.085, 0.20),
    bay_box(0.0, 0.30, -0.158, 0.158, 0.19, 0.20),                  # top
    bay_box(0.288, 0.30, -0.158, 0.158, 0.085, 0.20),               # back
    lk.box((0.25, 0.0127, 0.035), (0.145, 0.1391, 0.1325), color=STEEL),   # bay-side slide members
    lk.box((0.25, 0.0127, 0.035), (0.145, -0.1391, 0.1325), color=STEEL),
], mass=8.0)
_t.link("drawer", [
    bx(0.0, 0.015, -0.12, 0.12, 0.0, 0.060),                        # front (inset)
    bx(0.015, 0.28, -0.12, 0.12, 0.0, 0.006),                       # bottom
    bx(0.015, 0.28, 0.108, 0.12, 0.0, 0.065),                       # walls
    bx(0.015, 0.28, -0.12, -0.108, 0.0, 0.065),
    bx(0.268, 0.28, -0.12, 0.12, 0.0, 0.065),                       # back
    bx(0.015, 0.07, 0.12, 0.145, 0.060, 0.065),                     # left flange (on top)
    bx(0.015, 0.07, -0.145, -0.12, 0.0, 0.005),                     # right flange (underneath)
    lk.box((0.23, 0.0127, 0.035), (0.155, 0.12635, 0.1325), color=STEEL),   # inner slide members
    lk.box((0.23, 0.0127, 0.035), (0.155, -0.12635, 0.1325), color=STEEL),
], mass=1.2, color=PLY, contact_offset=0.001)
_t.link("levL", [bx(0.012, 0.040, 0.1332, 0.1372, -0.012, 0.030),
                 bx(0.010, 0.045, 0.120, 0.145, -0.012, -0.009)], mass=0.02, color=RED, contact_offset=0.001)
_t.link("levR", [bx(0.012, 0.040, -0.1372, -0.1332, 0.030, 0.077),
                 bx(0.010, 0.045, -0.145, -0.120, 0.074, 0.077)], mass=0.02, color=BLUE, contact_offset=0.001)
_t.prismatic("slide", "bay", "drawer", anchor=(0.0, 0.0, Z0), axis=(-1, 0, 0), limits=(LO, STOP), init=START_EXT,
             stiffness=0.0, damping=3.0, friction=0.5)
_t.prismatic("pressL", "drawer", "levL", anchor=(0.0275, 0.12, Z0 - 0.01), axis=(0, 0, 1), limits=(0.0, TRAVEL),
             stiffness=1200.0, damping=8.0)
_t.prismatic("pressR", "drawer", "levR", anchor=(0.0275, -0.12, Z0 + 0.075), axis=(0, 0, -1), limits=(0.0, TRAVEL),
             stiffness=1200.0, damping=8.0)
DRAWER = _t.build()
OBJECTS = [DRAWER]


def measure(d, tools) -> dict:
    return {
        "ext": float(d.q["slide"]), "lL": float(d.q["pressL"]), "lR": float(d.q["pressR"]),
        "d_speed": float(d.max_speed()),
        "clear": {s: float(lk.clearance(tools[s], d)) for s in ("left", "right")},
        "tcp_speed": {s: float(tools[s].speed) for s in ("left", "right")},
    }


class Record:
    """Whole-run record (a linkkit History fed every control step, cleared only by the simulator reset)."""

    def __init__(self):
        self.H = lk.History()
        self.last = None
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
        H.ever("both_pressed", m["lL"] >= ON and m["lR"] >= ON)
        H.ever("past_stop", m["ext"] >= PAST)
        H.peak("ext_max", m["ext"])
        H.peak("lL_max", m["lL"])
        H.peak("lR_max", m["lR"])
        H.held("out_still", m["ext"] >= CLEARED and m["d_speed"] < DRAWER_STILL)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "both_pressed" not in H.evers:
        return 0.0, "the two levers were never pressed at the same time"
    if "past_stop" not in H.evers:
        return 0.25, "both levers were pressed together, but the drawer never passed the stop"
    ends = {
        "drawer out 280 mm and still 1 s": H.runs.get("out_still", 0.0) >= STILL_S - EPS,
        "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS,
    }
    if m["ext"] >= CLEARED and ends["drawer out 280 mm and still 1 s"]:
        if ends["both grippers 30 mm clear and still 1 s"]:
            return 1.0, "drawer out of its slides, at rest, both grippers let go"
        return 0.75, "drawer out and at rest; not yet: both grippers 30 mm clear and still 1 s"
    return 0.5, "the drawer passed the stop but is not out and at rest at the end"


REC = Record()
LATCH = {"free": False}


@lk.on_reset
def _reset(tick):
    REC.reset()
    LATCH["free"] = False


@lk.on_step
def _track(tick):
    d = tick.states["drawer"]
    m = measure(d, tick.tools)
    both = m["lL"] >= ON and m["lR"] >= ON
    ext = m["ext"]
    if not LATCH["free"] and both and ext >= STOP - 0.003:
        lk.set_limits("drawer", "slide", LO, FREE_HI)
        LATCH["free"] = True
    elif LATCH["free"] and not both and ext < DOG:  # let go before the members are past the dog: it drops back in
        lk.set_limits("drawer", "slide", LO, STOP)
        LATCH["free"] = False
    REC.update(m)
    if REC.n % 5 == 0:
        REC.trace.append((round(tick.t, 2), round(1000 * ext, 1), round(1000 * m["lL"], 2), round(1000 * m["lR"], 2),
                          int(LATCH["free"]), [round(1000 * m["clear"][s]) for s in ("left", "right")]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():
    return {}


def demo_report() -> dict:
    return {"ladder": ladder(REC), "peaks": dict(REC.H.peaks), "evers": dict(REC.H.evers), "runs": dict(REC.H.runs),
            "trace": REC.trace[::2], "variant": VARIANT}


# -- scripted demo ----------------------------------------------------------------------------------------------

VARIANTS = ("", "nothing", "left_only", "right_only")
SQUEEZE_W = 0.0685
CONTACT = 0.0115  # pad contact centre past the TCP along the approach
PITCH = 15.0  # deg the approach points down
PULL = 0.095
LEFT_Z = 0.0265
RIGHT_Z = 0.0385
WALL_MID = 0.1325
LEV_X = 0.0275


def demo_script(ctx):
    from linkdemo import grip_frame

    yield from ctx.wait(1.0)
    seen = ctx.observe()

    def P(loc):
        return seen.point("drawer", "drawer", (loc[0], loc[1], loc[2] + Z0))

    p = math.radians(PITCH)
    a = np.array([math.cos(p), 0.0, -math.sin(p)])
    r_right = grip_frame(a, (0, 0, 1))
    r_left = grip_frame(a, (0, 0, 1), ref=np.column_stack([-r_right[:, 0], -r_right[:, 1], r_right[:, 2]]))
    var = VARIANT
    use_left = var not in ("nothing", "right_only")
    use_right = var not in ("nothing", "left_only")
    pull = PULL

    def side(arm, c, rot, sgn, use):
        if not use:
            return
        tcp = c - a * CONTACT
        out = tcp - np.array([0.08, 0.0, 0.0])
        yield from arm.move(out, rot, width=0.094, duration=3.0)
        yield from arm.move(out, rot, duration=1.0)
        yield from arm.move(tcp, rot, width=0.094, duration=1.5)
        yield from arm.grip(SQUEEZE_W, 1.0)
        yield from arm.wait(0.5)
        if var == "tap_release":
            yield from arm.grip(0.094, 0.6)
            yield from arm.wait(1.0)
            yield from arm.move(tcp - [pull, 0, 0], rot, duration=2.0)
            return
        yield from arm.move(tcp - [pull, 0.0, 0.0], rot, duration=5.0)
        yield from arm.wait(0.5)
        yield from arm.grip(0.094, 0.6)
        yield from arm.move(tcp - [pull + 0.07, 0.0, 0.0], rot, duration=1.5)

    yield from ctx.both(side(ctx.left, P((LEV_X, WALL_MID, LEFT_Z)), r_left, +1, use_left),
                        side(ctx.right, P((LEV_X, -WALL_MID, RIGHT_Z)), r_right, -1, use_right))
    yield from ctx.wait(4.0)
