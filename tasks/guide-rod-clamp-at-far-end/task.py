# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Lock a floating guide rod at the end of travel, where the carriage sets it.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475. Axis frame R: origin on the orange rod's pivot axis at rod-centre height, world
(0.350, 0.130, 0.7875), axes parallel to the world's; d is the distance from the pivot along -y.

- A manual lead-screw linear slide lies across the table along y on a grey tooling plate: a fixed steel guide rod A
  on the near side and, 80 mm further out, a loose orange guide rod B. Rod B is pinned at its near (left) end and its
  far end floats in a slotted clamp block, whose light preload spring keeps it from rattling by pushing it against
  one side of the slot, 0.5 deg out of parallel (2 mm at the far end).
- A blue carriage rides on both rods: a long linear bearing on rod A guides it, and a short plain bushing with
  0.3 mm of radial play rides on rod B. It is driven along the rods by a self-locking lead screw from a black
  geared hand crank at the left end (32 mm per crank turn, five turns end to end); the crank's slip clutch slips at
  3 mm of screw lag (about 12 N), so a carriage that binds stops while the crank turns on.
- At the right end a red cam lever on rod B's far clamp locks the rod where it is when the lever is pressed flat.

Rod B's skew grows along the stroke, so the bushing starts to bear on it 34 mm out from the pivot and drags its far
end into line from there on; the farther out the carriage, the straighter it holds the rod (0.09 deg at the far
stop). Locked where it lies, the rod jams the bushing a few millimetres out; locked part-way, it jams further out;
locked after the carriage has gone home, the preload spring has already pushed it back out of line. Only a rod locked
while the carriage sits at the far stop lets the carriage run home freely.

Goal: rod B locked, the carriage run to the far stop and back home with the lock closed and never stuck, and both
grippers let go.
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

ORIGIN = np.array([0.350, 0.130, TOP + 0.040])  # axis frame R: rod B pivot, rod-centre height
CRANK_AT = np.array([0.310, 0.200, TOP + 0.060])  # crank axis on the gearbox top (R (-0.040, 0.070, 0.020))

ALU = (0.72, 0.73, 0.75)
STEEL = (0.62, 0.63, 0.66)
DARK = (0.18, 0.18, 0.20)
BLACK = (0.08, 0.08, 0.09)
ORANGE = (0.95, 0.50, 0.10)
BLUE = (0.15, 0.35, 0.85)
RED = (0.80, 0.10, 0.10)

# geometry (m; R frame)
ROD_R = 0.004  # both rods 8 mm
PLAY = 0.0003  # bushing B radial play each side (0.6 mm diametral)
BUSH_L = 0.012  # bushing B length along the rod
D_HOME = 0.024  # bushing B centre's distance from the pivot at the home stop (q = 0)
Q_FAR = 0.160  # travel: home stop q 0, far stop q 0.160 (bushing centre at d 0.184)
ROD_LEN = 0.232  # d -0.006 .. 0.226
SPACING = 0.080  # rod A at R x -0.080
LEVER_HINGE = (0.0, -0.222, 0.0105)
LEVER_LEN = 0.050
CRANK_R = 0.045

# rod B and its clamp
SKEW_DEG = 0.5  # preload-spring rest: far end 2.0 mm outward (+x)
ROD_K, ROD_C = 0.3, 0.05  # preload spring N m/rad, damping
BRAKE_K, BRAKE_C = 20000.0, 15.0  # clamp: stiff hold where the rod is when it closes
LEVER_OPEN_DEG = 50.0
CLAMP_ON_DEG, CLAMP_OFF_DEG = 10.0, 20.0  # clamp closes within 10 deg of flat, frees past 20 deg

# screw drive
LEAD = 0.032  # m of carriage travel per crank turn
SLIP = 0.003  # slip clutch: the screw can run at most 3 mm ahead of (or behind) the carriage
DRIVE_K, DRIVE_C = 4000.0, 60.0  # nut stiffness N/m, damping N s/m
CARR_FRICTION = 0.5  # N, rod A bearing

# scoring
JAM_MM, JAM_S = 1.5, 0.15  # stuck: carriage 1.5 mm behind its screw for 0.15 s
STOP_MM = 1.0  # at a stop: within 1 mm
ROD_TOL_DEG = 0.12  # rod B locked within 0.12 deg of parallel (0.47 mm over its length)
CLEAR_M = 0.030
TCP_STILL = 0.005
RELEASE_S = 1.0
EPS = 1e-6
MOVING = ("carriage", "rod", "lever")

# -- the axis -----------------------------------------------------------------------------------------------------

_base = [
    lk.box((0.130, 0.400, 0.010), (-0.040, -0.090, -0.035)),  # tooling plate, top at R z -0.030
    lk.capsule(ROD_R, 0.242, (-SPACING, -0.105, 0.0), axis="Y", color=STEEL),  # rod A
    lk.box((0.020, 0.012, 0.038), (-SPACING, 0.014, -0.011), color=DARK),  # rod A end blocks
    lk.box((0.020, 0.012, 0.038), (-SPACING, -0.224, -0.011), color=DARK),
    lk.box((0.024, 0.016, 0.038), (0.0, 0.0, -0.011), color=DARK),  # rod B pivot block
    lk.box((0.026, 0.016, 0.038), (0.0, -0.218, -0.011), color=DARK),  # rod B floating far clamp block
    lk.box((0.028, 0.004, 0.0008), (0.0, -0.218, 0.0084), color=ORANGE),  # slot mark on the clamp block
    lk.cylinder(0.005, 0.260, (-0.040, -0.080, -0.018), axis="Y", color=BLACK),  # lead screw
    lk.box((0.020, 0.012, 0.024), (-0.040, -0.216, -0.018), color=DARK),  # screw end bearing
    lk.box((0.010, 0.004, 0.012), (-SPACING, -0.003, -0.012), color=BLACK),  # travel stop marks on rod A's blocks
]

_t = lk.Tree("axis", pos=tuple(ORIGIN), fixed=True, color=ALU)
_t.link("base", _base, mass=3.0)
_t.link("carriage", [
    lk.box((0.110, 0.030, 0.008), (-0.040, 0.0, 0.009)),  # plate, bottom 1 mm above both rods
    lk.box((0.005, BUSH_L, 0.014), (ROD_R + PLAY + 0.0025, 0.0, 0.0)),  # bushing B side walls (0.3 mm play)
    lk.box((0.005, BUSH_L, 0.014), (-(ROD_R + PLAY + 0.0025), 0.0, 0.0)),
    lk.box((0.0186, BUSH_L, 0.004), (0.0, 0.0, -0.007)),  # bushing B bottom, 1 mm below the rod
    lk.box((0.024, 0.040, 0.018), (-SPACING, 0.0, -0.004)),  # linear-bearing housing on rod A
    lk.box((0.022, 0.020, 0.022), (-0.040, 0.0, -0.008), color=DARK),  # screw nut
], mass=0.40, at=(0.0, -D_HOME, 0.0), color=BLUE, friction=(0.35, 0.30), contact_offset=0.0005)
_t.link("rod", [lk.capsule(ROD_R, ROD_LEN - 2 * ROD_R, (0.0, -0.110, 0.0), axis="Y")], mass=0.090, color=ORANGE,
        friction=(0.35, 0.30), contact_offset=0.0005)
_t.link("lever", [
    lk.box((0.010, LEVER_LEN, 0.005), (0.0, 0.0, 0.0)),
    lk.box((0.014, 0.012, 0.008), (0.0, -0.019, 0.001)),  # grip
], mass=0.020, at=(0.0, LEVER_HINGE[1] - LEVER_LEN / 2, LEVER_HINGE[2]), color=RED, friction=(0.8, 0.8))
_t.prismatic("slide", "base", "carriage", anchor=(0.0, -D_HOME, 0.0), axis=(0, -1, 0), limits=(-0.0005, Q_FAR + 0.0005),
             init=0.0, stiffness=DRIVE_K, damping=DRIVE_C, friction=CARR_FRICTION)
_t.revolute("pivot", "base", "rod", anchor=(0.0, 0.0, 0.0), axis=(0, 0, 1), limits_deg=(-1.0, 1.0), rest_deg=SKEW_DEG,
            init_deg=SKEW_DEG, stiffness=ROD_K, damping=ROD_C, armature=0.001,
            brake=lk.Brake(stiffness=BRAKE_K, damping=BRAKE_C, engaged=False))
_t.revolute("cam", "base", "lever", anchor=LEVER_HINGE, axis=(-1, 0, 0), limits_deg=(-1.0, 55.0),
            rest_deg=LEVER_OPEN_DEG, init_deg=LEVER_OPEN_DEG, stiffness=0.15, damping=0.003, armature=1e-5)
AXIS = _t.build()

# The crank is its own articulation (no joint limits anywhere in it), so its joints turn without end.
_t = lk.Tree("crank", pos=tuple(CRANK_AT), fixed=True, color=BLACK)
_t.link("gearbox", [lk.box((0.040, 0.040, 0.050), (0.0, 0.0, -0.025))], mass=0.8)
_t.link("arm", [
    lk.cylinder(0.010, 0.008, (0.0, 0.0, 0.004)),
    lk.box((0.055, 0.012, 0.006), (0.0225, 0.0, 0.005)),
], mass=0.060)
_t.link("handle", [lk.capsule(0.007, 0.020, (0.0, 0.0, 0.0), axis="Z")], mass=0.015, at=(CRANK_R, 0.0, 0.025),
        friction=(1.0, 1.0))
_t.revolute("turn", "gearbox", "arm", anchor=(0.0, 0.0, 0.0), axis=(0, 0, 1), damping=0.005, friction=0.02,
            armature=1e-4)
_t.revolute("spin", "arm", "handle", anchor=(CRANK_R, 0.0, 0.008), axis=(0, 0, 1), damping=1e-4, armature=1e-5)
CRANK = _t.build()

OBJECTS = [AXIS, CRANK]

lk.detent("axis", "cam", [0.0, math.radians(LEVER_OPEN_DEG)])  # over-centre cam lever: snaps open or flat

# -- the screw and the whole-run record (pure Python, so the tests can drive them) -----------------------------------


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Screw:
    """Crank angle -> screw position s (the carriage's drive target, m). The screw runs only inside the travel
    (0..Q_FAR) and its slip clutch keeps it within SLIP of the carriage. ``step`` returns (target, lag) where lag
    is how far the carriage trailed the previous target (m, + = behind on the way out)."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.s, self.phi = 0.0, None

    def step(self, phi: float, q: float) -> tuple[float, float]:
        lag = self.s - q
        if self.phi is not None:
            self.s += wrap(phi - self.phi) * LEAD / (2.0 * math.pi)
        self.phi = phi
        self.s = min(max(self.s, 0.0, q - SLIP), Q_FAR, q + SLIP)
        return self.s, lag


class Record:
    """Whole-run record (a linkkit History plus clamp and jam events), updated every control step and cleared only
    by the simulator reset."""

    def __init__(self):
        self.H = lk.History()
        self.reset()

    def reset(self) -> None:
        self.H.reset()
        self.since: dict | None = None  # the current clamp: when it closed, where, and what the carriage did since
        self.clamps: list = []
        self.jam: dict | None = None
        self.last: dict | None = None
        self.n = 0
        self.trace: list = []

    def update(self, m: dict, t: float | None = None) -> None:
        if t is not None:
            self.H._tick(t)
        self.n += 1
        self.last = m
        q, H = m["q_mm"], self.H
        H.peak("lag_mm", abs(m["lag_mm"]))
        stuck = H.held("stuck", abs(m["lag_mm"]) >= JAM_MM - EPS)
        if stuck >= JAM_S - EPS and self.jam is None:
            self.jam = {"t": round(H.t, 2), "q_mm": round(q, 2), "lag_mm": round(m["lag_mm"], 2),
                        "clamped": bool(m["clamped"]), "rod_deg": round(m["rod_deg"], 3)}
        H.ever("far", q >= Q_FAR * 1000.0 - STOP_MM - EPS)
        if m["clamped"]:
            if self.since is None:
                self.since = {"t": round(H.t, 2), "q_mm": round(q, 2), "rod_deg": round(m["rod_deg"], 4),
                              "far": False, "home": False}
                self.clamps.append(dict(self.since))
            if q >= Q_FAR * 1000.0 - STOP_MM - EPS:
                self.since["far"] = True
            elif self.since["far"] and q <= STOP_MM + EPS:
                self.since["home"] = True
        else:
            self.since = None
        clear = m["clear"]
        H.held("released", bool(clear) and min(clear.values()) >= CLEAR_M - EPS
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    m = rec.last
    if m is None:
        return 0.0, "no steps yet"
    if rec.jam is not None:
        j = rec.jam
        return 0.0, (f"carriage stuck {j['lag_mm']:+.1f} mm behind its screw at {j['q_mm']:.1f} mm "
                     f"(t {j['t']:.1f} s, rod {'clamped' if j['clamped'] else 'free'} at {j['rod_deg']:+.3f} deg)")
    s = rec.since
    if not m["clamped"] or s is None:
        return 0.0, "rod B not clamped"
    if abs(m["rod_deg"]) > ROD_TOL_DEG + EPS:
        return 0.0, f"rod B clamped {m['rod_deg']:+.3f} deg out of parallel (carriage at {s['q_mm']:.1f} mm)"
    if not s["far"]:
        return 0.25, f"rod B clamped {m['rod_deg']:+.3f} deg; carriage not yet at the far stop since"
    if not s["home"]:
        return 0.5, f"rod B clamped {m['rod_deg']:+.3f} deg; carriage at the far stop, not yet back home"
    if rec.H.runs.get("released", 0.0) >= RELEASE_S - EPS:
        return 1.0, f"rod B clamped {m['rod_deg']:+.3f} deg; carriage ran far and home freely; let go"
    return 0.75, f"rod B clamped {m['rod_deg']:+.3f} deg; carriage ran far and home; grippers not clear and still 1 s"


def measure(ax, cr, tools, lag: float, clamped: bool, near: dict | None = None) -> dict:
    if near is None:
        near = {s: min(lk.clearance(t, ax, MOVING), lk.clearance(t, cr, ["arm", "handle"])) for s, t in tools.items()}
    return {"q_mm": ax.q["slide"] * 1000.0, "lag_mm": lag * 1000.0, "rod_deg": ax.deg("pivot"),
            "lever_deg": ax.deg("cam"), "clamped": bool(clamped), "clear": near,
            "tcp_speed": {s: float(t.speed) for s, t in tools.items()}}


SCREW = Screw()
REC = Record()


@lk.on_reset
def _reset(tick):
    SCREW.reset()
    REC.reset()
    lk.set_rest("axis", "slide", 0.0)


@lk.on_step
def _track(tick):
    ax, cr = tick.states["axis"], tick.states["crank"]
    lever = ax.deg("cam")
    clamped = lk.braked("axis", "pivot")
    if lever <= CLAMP_ON_DEG and not clamped:
        lk.brake("axis", "pivot", True)
        clamped = True
    elif lever >= CLAMP_OFF_DEG and clamped:
        lk.brake("axis", "pivot", False)
        clamped = False
    target, lag = SCREW.step(cr.q["turn"], ax.q["slide"])
    lk.set_rest("axis", "slide", target)
    m = measure(ax, cr, tick.tools, lag, clamped)
    REC.update(m)
    if REC.n % 5 == 0:  # diagnostics every 0.25 s
        REC.trace.append((round(tick.t, 2), round(m["q_mm"], 2), round(target * 1000, 2), round(m["lag_mm"], 2),
                          round(m["rod_deg"], 3), round(lever, 1), round(math.degrees(cr.q["turn"]), 1),
                          int(clamped), [round(1000 * v) for v in m["clear"].values()]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo -----------------------------------------------------------------------------------------------------
# linkdemo.py runs demo_script(ctx). It reads only what a camera could measure: the carriage's place along the rods
# and the lever's and crank's angles (from link poses), the handle's position, and the robot's own grippers. It never
# reads rod B's angle, the screw, the clamp state or the record.

DEMO_LOG: list = []
PAD_TIP = 0.013
HOVER_R = np.array([0.350, -0.120, TOP + 0.170])  # right hand waits high over the lever


def _q(ctx) -> float:
    return ctx.observe().angle_deg("axis", "slide")  # m: the carriage's place along the rods


def grasp_handle(ctx):
    """Left hand straight down on the crank handle, pads across it 10 mm above the crank arm."""
    import linkdemo as ld

    arm = ctx.left
    p = ctx.observe().point("crank", "handle", (0.0, 0.0, -0.007 + PAD_TIP))
    yield from arm.move(p + [0, 0, 0.06], ld.down(0.0), width=0.040, duration=1.8)
    yield from arm.move(p, ld.down(0.0), duration=1.0)
    yield from arm.grip(0.008, 0.5)


def crank(ctx, turns: float, rate: float = 0.37):
    """Turn the crank ``turns`` (+ = carriage out) by circling the held handle about the crank axis; the wrist keeps
    its orientation (the handle spins on the crank arm)."""
    hinge, axis = ctx.observe().axis("crank", "turn")
    yield from ctx.left.arc(hinge, axis, 360.0 * turns, follow=0.0, duration=max(1.0, abs(turns) / rate))


def crank_to(ctx, goal: float, tries: int = 3):
    """Crank until the carriage is seen at ``goal`` (a stop: 0 or Q_FAR), going a sixth of a turn past it."""
    for _ in range(tries):
        err = goal - _q(ctx)
        if abs(err) <= 0.0005:
            break
        q0 = _q(ctx)
        yield from crank(ctx, err / LEAD + math.copysign(1 / 6, err))
        DEMO_LOG.append({"crank_to_mm": round(goal * 1000, 1), "from_mm": round(q0 * 1000, 2),
                         "seen_mm": round(_q(ctx) * 1000, 2), "t": round(ctx.t, 2)})
        if abs(_q(ctx) - q0) < 0.002:  # no progress: the carriage is stuck
            break


def crank_by(ctx, dq: float):
    q0 = _q(ctx)
    yield from crank(ctx, dq / LEAD)
    DEMO_LOG.append({"crank_by_mm": round(dq * 1000, 1), "from_mm": round(q0 * 1000, 2),
                     "seen_mm": round(_q(ctx) * 1000, 2), "t": round(ctx.t, 2)})


def press_lever(ctx, tries: int = 2):
    """Right hand, fingers closed, presses the open lever down 40 mm out from its hinge until it lies flat."""
    arm = ctx.right
    for _ in range(tries):
        seen = ctx.observe()
        if seen.angle_deg("axis", "cam") <= 3.0:
            break
        p = seen.point("axis", "lever", (0.0, LEVER_LEN / 2 - 0.040, 0.0025))  # top face, 40 mm from the hinge
        flat = ORIGIN[2] + LEVER_HINGE[2] + 0.0025
        yield from arm.press(p, p[2] - flat + 0.003, yaw_deg=0.0, hover=0.04, duration=1.5, hold=0.5)
        DEMO_LOG.append({"lever_seen_deg": round(ctx.observe().angle_deg("axis", "cam"), 1), "t": round(ctx.t, 2)})
    yield from arm.move(HOVER_R + [0.0, -0.04, 0.0], duration=1.2)


def let_go(ctx):
    import linkdemo as ld

    arm = ctx.left
    yield from arm.grip(0.040, 0.5)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.08]), ld.down(0.0), duration=1.0)


def demo_script(ctx):
    import linkdemo as ld

    v = VARIANT
    if v == "nothing":
        yield from ctx.wait(55.0)
        return
    yield from ctx.both(grasp_handle(ctx), ctx.right.move(HOVER_R, ld.down(0.0), width=0.0, duration=2.0))
    if v == "clamp_as_found":  # lock the rod where it lies, then run the check
        yield from press_lever(ctx)
    elif v == "clamp_mid":  # lock it with the carriage half-way out
        yield from crank_by(ctx, Q_FAR / 2)
        yield from press_lever(ctx)
    yield from crank_to(ctx, Q_FAR)  # 1. crank the carriage to the far stop: its bushing draws rod B into line
    if v == "out_and_back":  # run out and home first, then lock
        yield from crank_to(ctx, 0.0)
        yield from press_lever(ctx)
        yield from crank_to(ctx, Q_FAR)
    elif v == "":
        yield from press_lever(ctx)  # 2. lock rod B while the parked carriage holds it square
    yield from crank_to(ctx, 0.0)  # 3. crank home: the check run
    yield from let_go(ctx)
    yield from ctx.wait(3.0)


def demo_report() -> dict:
    return {"ladder": ladder(REC)[1], "clamps": REC.clamps, "jam": REC.jam, "demo": DEMO_LOG,
            "peak_lag_mm": REC.H.peaks.get("lag_mm"), "trace": REC.trace}
