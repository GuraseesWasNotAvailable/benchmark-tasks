# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Zig-zag fold a four-panel array wing whose hinges fold alternate ways, flat onto its stowage plate.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A grey aluminium stowage plate (80 x 50 x 20 mm) is fixed to the table at x 0.18 to 0.26, centred on y 0. The
  wing's root panel P1 is bolted on top of it, cells up.
- Panels P2, P3 and P4 lie deployed in a line beyond it along +x, cells up, on a grey ground-support rail of the same
  height (x 0.262 to 0.507). Every panel is 80 x 60 x 4 mm (a white composite back and a dark blue cell face), 20 g.
- Piano hinges join the panels along their 60 mm edges (axes along y). Each hinge folds only toward the face its
  knuckle is on, up to 180 deg, and stops 12 deg past flat the other way: h1 (P1-P2) and h3 (P3-P4) have the knuckle
  on the cell face, h2 (P2-P3) on the back. Each hinge is drawn as a steel knuckle and leaves on its knuckle face, so
  the cameras show which way it folds (looks only, no colliders).

Physics, not words, defeats the natural plan. Folding from the tip, every fold the same way (P4 over onto P3, then
that pair over onto P2), jams at h2: its knuckle is on the back, so it cannot fold toward P2's cell face, and lifting
the pair only tips P2 up with it about h1 as one piece. The only flat stack is the zig-zag: P2 turned over onto P1 (h1
+180, cells to cells), which brings P3 back-to-back onto P2 (h2 -180), then P4 turned over onto P3 (h3 +180). The
hinges carry 0.003 N m of friction, so a panel turned past upright and let go falls onto the stack fast enough to
crack cells (the gate); it has to be lowered.

Goal: one flat stack on the stowage plate, panels square over each other, both grippers let go.
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

# -- geometry (m) ----------------------------------------------------------------------------------------------
X0 = 0.18  # the plate's and P1's near edge (toward the robot)
YC = 0.0  # the wing's centre line
L = 0.080  # panel length along the wing (x)
W = 0.060  # panel width along the hinges (y)
PANEL_T = 0.004  # panel thickness: 3 mm back + 1 mm cell face
BACK_T = 0.003
CELL_T = 0.001
ZB = 0.020  # height of the stowage plate and the rail above the table: the deployed panels' underside
PLATE_W = 0.050  # plate and rail width (y): the panels overhang 5 mm each side, room for the pads
PANEL_MASS = 0.020
PLATE_MASS = 0.216  # 80 x 50 x 20 mm aluminium
RAIL_X0 = X0 + L + 0.002
RAIL_LEN = 0.245
RAIL_MASS = 0.66
HINGE_FRICTION = 0.003  # N m, static and dynamic
HINGE_DAMPING = 5e-4  # N m s/rad
STOP_DEG = 12.0  # each hinge stops this far past flat, away from its knuckle face
PANELS = ("p1", "p2", "p3", "p4")
HINGES = ("h1", "h2", "h3")

BACK_WHITE = (0.92, 0.92, 0.90)
CELL_BLUE = (0.08, 0.12, 0.35)
PLATE_GREY = (0.55, 0.56, 0.58)
RAIL_GREY = (0.40, 0.41, 0.43)


def _panel(extra=()):
    """A panel's colliders in its link frame (origin at the panel's centre, cell face on +z at the design pose)."""
    return [lk.box((L, W, BACK_T), (0.0, 0.0, -PANEL_T / 2 + BACK_T / 2), color=BACK_WHITE, contact_offset=0.001),
            lk.box((L, W, CELL_T), (0.0, 0.0, PANEL_T / 2 - CELL_T / 2), color=CELL_BLUE, contact_offset=0.001),
            *extra]


# linkage frame at P1's centre (a fixed root link must sit at the linkage origin: the simulator reset puts the root
# body at the linkage pose)
_t = lk.Tree("wing", pos=(X0 + L / 2, YC, TOP + ZB + PANEL_T / 2), fixed=True, color=BACK_WHITE)
_t.link("p1", _panel([lk.box((L, PLATE_W, ZB), (0.0, 0.0, -PANEL_T / 2 - ZB / 2), color=PLATE_GREY)]),
        mass=PANEL_MASS + PLATE_MASS, friction=(0.5, 0.4), contact_offset=0.001)
for _i, _n in enumerate(PANELS[1:], start=1):
    _t.link(_n, _panel(), mass=PANEL_MASS, at=(L * _i, 0.0, 0.0), friction=(0.5, 0.4), contact_offset=0.001)
_HINGE = {"friction": HINGE_FRICTION, "dynamic_friction": HINGE_FRICTION, "damping": HINGE_DAMPING}
# + about (0, -1, 0) turns a panel's free end up and back toward the robot (over its parent's cell face)
_t.revolute("h1", "p1", "p2", anchor=(L / 2, 0.0, PANEL_T / 2), axis=(0, -1, 0), limits_deg=(-STOP_DEG, 180.0),
            **_HINGE)  # cell face of P1's far edge
_t.revolute("h2", "p2", "p3", anchor=(1.5 * L, 0.0, -PANEL_T / 2), axis=(0, -1, 0), limits_deg=(-180.0, STOP_DEG),
            **_HINGE)  # back face
_t.revolute("h3", "p3", "p4", anchor=(2.5 * L, 0.0, PANEL_T / 2), axis=(0, -1, 0), limits_deg=(-STOP_DEG, 180.0),
            **_HINGE)  # cell face
WING = _t.build()

_t = lk.Tree("rail", pos=(RAIL_X0 + RAIL_LEN / 2, YC, TOP + ZB / 2), fixed=True, color=RAIL_GREY)
_t.link("bar", [lk.box((RAIL_LEN, PLATE_W, ZB), contact_offset=0.001)], mass=RAIL_MASS, friction=(0.25, 0.2))
RAIL = _t.build()

OBJECTS = [WING, RAIL]

# -- looks only: piano-hinge knuckles and leaves, and the panel borders (no colliders, no mass) -------------------
# A piano hinge's knuckle stands proud on the face it folds toward, which is how a technician reads the fold
# direction. Each hinge is drawn as a steel knuckle (3.5 mm across) on its hinge line with a 5 mm leaf on each panel,
# on its knuckle face; on the other face the panels' 1.5 mm borders show as a seam (white between the blue cell faces,
# grey between the white backs). They are plain USD shapes with no collision API, added under the wing's link prims
# right after linkkit spawns it, so physics, the demo and the scorer see exactly the colliders above.
STEEL = (0.66, 0.67, 0.70)
SEAM_GREY = (0.30, 0.30, 0.32)
KNUCKLE_R = 0.00175
HINGE_LEN = 0.056
LEAF_W = 0.005
BORDER_W = 0.0015
SKIN_T = 0.0003
SKIN_Z = PANEL_T / 2 + SKIN_T / 2 - 0.0001  # a leaf or border stands 0.2 mm proud of its face
KNUCKLE_FACE = {"h1": 1, "h2": -1, "h3": 1}  # +1 the cell face, -1 the back


def hinge_looks() -> list:
    """The visual-only pieces: (link, kind, dims, centre in the link frame, sRGB). Pure Python."""
    out = []
    for i, h in enumerate(HINGES):
        a, b, f = PANELS[i], PANELS[i + 1], KNUCKLE_FACE[h]
        x_leaf = L / 2 - KNUCKLE_R - LEAF_W / 2
        x_seam = L / 2 - BORDER_W / 2
        seam = BACK_WHITE if f < 0 else SEAM_GREY
        out += [(a, "cylinder", (KNUCKLE_R, HINGE_LEN), (L / 2, 0.0, f * PANEL_T / 2), STEEL),  # on the hinge axis
                (a, "box", (LEAF_W, HINGE_LEN, SKIN_T), (x_leaf, 0.0, f * SKIN_Z), STEEL),
                (b, "box", (LEAF_W, HINGE_LEN, SKIN_T), (-x_leaf, 0.0, f * SKIN_Z), STEEL),
                (a, "box", (BORDER_W, W, SKIN_T), (x_seam, 0.0, -f * SKIN_Z), seam),
                (b, "box", (BORDER_W, W, SKIN_T), (-x_seam, 0.0, -f * SKIN_Z), seam)]
    return out


_KIT_SPAWN = lk._spawn_linkage


def _spawn_with_hinge_looks(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """linkkit's spawner, then (for the wing) the visual-only hinge pieces under its link prims (pxr only)."""
    prim = _KIT_SPAWN(prim_path, cfg, translation, orientation, **kwargs)
    if getattr(cfg, "linkage_name", "") != WING.name:
        return prim
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    mats: dict = {}
    for k, (link, kind, dims, at, rgb) in enumerate(hinge_looks()):
        path = f"{prim_path}/{link}/look{k}"
        if kind == "box":
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            lk._set_xform(g, at, scale=dims)
        else:
            g = UsdGeom.Cylinder.Define(stage, path)
            g.CreateAxisAttr("Y")
            g.CreateRadiusAttr(dims[0])
            g.CreateHeightAttr(dims[1])
            lk._set_xform(g, at)
        if rgb not in mats:
            mats[rgb] = f"{prim_path}/Looks/hinge{len(mats)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=0.45)
            vis.func(mats[rgb], vis)
        bind_visual_material(path, mats[rgb], stage=stage)
    return prim


lk._spawn_linkage = _spawn_with_hinge_looks  # linkkit looks its spawner up when the scene is built

# -- scoring -----------------------------------------------------------------------------------------------------
GATE_RATE = 180.0  # deg/s: any panel turning faster than this at any time gives 0 (let fall onto the stack)
ZZ_DEG = 170.0  # the three-panel zig-zag: h1 >= this and h2 <= -this
TURN_DEG = 95.0  # P4 turned past upright toward the stack
FLAT_DEG = 177.0  # one flat stack: every hinge within 3 deg of fully folded
STILL_RATE = 3.0  # deg/s
STILL_SPEED = 0.003  # m/s
STEADY_S = 1.0
CLEAR_M = 0.030  # release gate: every pad point of both grippers this far from the wing and the rail
TCP_STILL = 0.005  # m/s
RELEASE_S = 1.0
EPS = 1e-6


def measure(wing, rail, tools) -> dict:
    """One sample: hinge angles (deg, joint readouts; a protractor reads the same), grippers' clearance (m) and TCP
    speed (m/s). ``rate`` (deg/s) and ``speed`` (m/s), the fastest panel, are filled in from the pose change since the
    previous control step."""
    clear = {s: min(lk.clearance(t, wing), lk.clearance(t, rail)) for s, t in tools.items()}
    return {
        "a1": wing.deg("h1"), "a2": wing.deg("h2"), "a3": wing.deg("h3"),
        "rate": 0.0, "speed": 0.0,
        "clear": clear,
        "tcp_speed": {s: float(t.speed) for s, t in tools.items()},
    }


def zigzag3(m) -> bool:
    return m["a1"] >= ZZ_DEG and m["a2"] <= -ZZ_DEG


def stowed(m) -> bool:
    return m["a1"] >= FLAT_DEG and m["a2"] <= -FLAT_DEG and m["a3"] >= FLAT_DEG


def pose_rates(prev: dict, now: dict, dt: float) -> tuple[float, float]:
    """Fastest panel turn (deg/s) and centre speed (m/s) from two sets of 4x4 link poses ``dt`` s apart."""
    rate = speed = 0.0
    for n, m in now.items():
        p = prev.get(n)
        if p is None:
            continue
        c = (np.trace(p[:3, :3].T @ m[:3, :3]) - 1.0) / 2.0
        rate = max(rate, math.degrees(math.acos(max(-1.0, min(1.0, float(c))))) / dt)
        speed = max(speed, float(np.linalg.norm(m[:3, 3] - p[:3, 3])) / dt)
    return rate, speed


class Record:
    """Whole-run record: a linkkit History updated every control step and cleared only by the simulator reset, so a
    panel let fall is never erased by lifting it again. Pure Python."""

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
        H.peak("rate_max", m["rate"])
        H.ever("slammed", m["rate"] > GATE_RATE)
        for k in ("a1", "a3"):
            H.peak(f"{k}_max", m[k])
            H.low(f"{k}_min", m[k])
        H.low("a2_min", m["a2"])
        H.peak("a2_max", m["a2"])
        z3 = zigzag3(m)
        H.ever("zigzag3", z3)
        H.ever("turned4", z3 and m["a3"] >= TURN_DEG)
        H.held("steady", stowed(m) and m["rate"] < STILL_RATE and m["speed"] < STILL_SPEED)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Score and the reason. The gate is checked first (any time in the run), then the rungs from the top."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "slammed" in H.evers:
        return 0.0, (f"gate: a panel turned faster than {GATE_RATE:.0f} deg/s at {H.evers['slammed']:.2f} s "
                     f"(peak {H.peaks['rate_max']:.0f} deg/s): it was let fall onto the stack")
    if stowed(m):
        ends = {
            "stack still 1 s": H.runs.get("steady", 0.0) >= STEADY_S - EPS,
            "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS,
        }
        if all(ends.values()):
            return 1.0, "one flat zig-zag stack on the stowage plate, panels square, both grippers let go"
        return 0.75, "flat stack at the end; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if "turned4" in H.evers:
        return 0.5, "P4 was turned past upright onto the zig-zag, but the stack is not flat at the end"
    if "zigzag3" in H.evers:
        return 0.25, "P2 turned over onto P1 with P3 back-to-back on it, but P4 was never turned onto the stack"
    return 0.0, (f"no zig-zag: h1 reached {H.peaks.get('a1_max', 0):.0f} deg and h2 {H.lows.get('a2_min', 0):.0f} deg "
                 f"(needs {ZZ_DEG:.0f} and -{ZZ_DEG:.0f} together)")


REC = Record()
_PREV: dict = {}  # the movable panels' poses at the previous control step


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    wing = tick.states["wing"]
    m = measure(wing, tick.states["rail"], tick.tools)
    poses = {n: wing.poses[n].copy() for n in PANELS[1:]}
    if _PREV and tick.t > _PREV["t"]:
        m["rate"], m["speed"] = pose_rates(_PREV["poses"], poses, tick.t - _PREV["t"])
        for n in PANELS[1:]:  # diagnostics: each panel's own peak turn rate
            REC.H.peak(f"rate_{n}", pose_rates({n: _PREV["poses"][n]}, {n: poses[n]}, tick.t - _PREV["t"])[0])
    _PREV.update(poses=poses, t=tick.t)
    REC.update(m)
    if REC.n % 5 == 0:  # diagnostics every 0.25 s
        REC.trace.append((round(tick.t, 2), round(m["a1"], 1), round(m["a2"], 1), round(m["a3"], 1),
                          round(m["rate"], 1), [round(1000 * m["clear"][s]) for s in ("left", "right")]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ----------------------------------------------------------------------------------------------
# linkdemo.py runs demo_script(ctx). It reads only what a camera could measure (link poses and hinge angles computed
# from link poses) and the robot's own grippers; never joint readouts or the record.

VARIANTS = ("", "nothing", "same_way", "drop")
PINCH_IN = 0.020  # pinch centre this far in from a panel's free edge, on its mid-plane
FREE_LOCAL = (L / 2 - PINCH_IN, 0.0, 0.0)
NEAR_LOCAL = (-(L / 2 - PINCH_IN), 0.0, 0.0)  # the same, near the edge a panel shares with the hinge before it
CONTACT_DEPTH = 0.0115
SQUEEZE = 0.0006
PIVOT_S = 4.0
UP_DEG = 92.0  # the lifting arm parks a panel just past upright, where hinge friction holds it
DOWN_DEG = 179.5  # the laying arm brings it down onto the stack
DROP_DEG = 125.0  # drop cheat: the laying arm lets go here instead of lowering the panel onto the stack
PARK: dict = {}  # each arm's ready-pose TCP (pos, rot), recorded when the demo starts
LOG: dict = {"pivots": []}


def _tilted(deg: float) -> np.ndarray:
    """Approach direction tilted ``deg`` from straight down toward +x (for far reaches)."""
    a = math.radians(deg)
    return np.array([math.sin(a), 0.0, -math.cos(a)])


def _turn(ctx, side: str, joint: str, target: float, *, link=None, local=FREE_LOCAL, tilt: float = 0.0):
    """Pinch a panel near its free edge and pivot it about ``joint`` until the hinge angle seen from the link poses
    reads ``target`` (closed loop, pads closing along the hinge, wrist following half the turn). Stays pinched."""
    a0 = ctx.observe().angle_deg("wing", joint)
    yield from ctx.pinch_pivot(side, "wing", joint, target - a0, local=local, link=link, width=W, squeeze=SQUEEZE,
                               contact_depth=CONTACT_DEPTH, follow=0.5, pivot_s=PIVOT_S, approach=_tilted(tilt))
    LOG["pivots"].append({"side": side, "joint": joint, "from": round(a0, 1), "target": target,
                          "seen": getattr(ctx, "pivot_log", None), "t": round(ctx.t, 2)})


def _let_go(ctx, side: str):
    arm = ctx.arm(side)
    yield from arm.open(0.5)
    yield from arm.retreat(0.05, 0.8)


def _park(ctx, side: str):
    pos, rot = PARK[side]
    yield from ctx.arm(side).move(pos, rot, duration=1.5)


def _fold(ctx, joint: str, *, park_left_first: bool):
    """Right turns the panel up past upright; left lays it down onto the stack."""
    first = _turn(ctx, "right", joint, UP_DEG)
    yield from (ctx.both(_park(ctx, "left"), first) if park_left_first else first)
    yield from _let_go(ctx, "right")
    yield from ctx.both(_park(ctx, "right"), _turn(ctx, "left", joint, DOWN_DEG))
    yield from _let_go(ctx, "left")


def demo_script(ctx):
    for side in ("left", "right"):
        PARK[side] = (ctx.arm(side).pos.copy(), ctx.arm(side).rot.copy())
    yield from ctx.wait(0.5)
    if VARIANT == "nothing":
        yield from ctx.wait(3.0)
        return
    if VARIANT == "drop":  # the demo's folds, but each panel is turned past upright and let fall onto the stack
        for joint in ("h1", "h3"):
            yield from _turn(ctx, "right", joint, UP_DEG)
            yield from _let_go(ctx, "right")
            yield from ctx.both(_park(ctx, "right"), _turn(ctx, "left", joint, DROP_DEG))
            yield from _let_go(ctx, "left")
            yield from ctx.both(_park(ctx, "left"), ctx.wait(1.5))
        return
    if VARIANT == "same_way":  # from the tip, every fold the same way: P4 onto P3, then the pair onto P2
        yield from _turn(ctx, "right", "h3", UP_DEG, tilt=20.0)
        yield from _let_go(ctx, "right")
        yield from ctx.both(_park(ctx, "right"), _turn(ctx, "left", "h3", DOWN_DEG, tilt=20.0))
        yield from _let_go(ctx, "left")
        # the P3/P4 pair, pinched near h3, turned up about h2 toward P2's cell face, the same way as the first fold
        yield from ctx.both(_park(ctx, "left"), _turn(ctx, "right", "h2", UP_DEG, link="p3", tilt=20.0))
        yield from _let_go(ctx, "right")
        yield from ctx.both(_park(ctx, "right"), ctx.wait(2.0))
        return
    yield from _fold(ctx, "h1", park_left_first=False)  # P2 over onto P1; P3 comes back-to-back onto P2
    yield from _fold(ctx, "h3", park_left_first=True)  # P4 over onto P3
    yield from ctx.both(_park(ctx, "left"), ctx.wait(1.5))
    yield from ctx.wait(2.0)


def demo_report() -> dict:
    H = REC.H
    s, why = ladder(REC)
    return {
        "variant": VARIANT, "score": s, "reason": why,
        "evers": {k: round(v, 2) for k, v in H.evers.items()},
        "peaks": {k: round(v, 2) for k, v in H.peaks.items()},
        "lows": {k: round(v, 2) for k, v in H.lows.items()},
        "runs": {k: round(v, 2) for k, v in H.runs.items()},
        "last": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in (REC.last or {}).items()
                 if k in ("a1", "a2", "a3", "rate", "speed")},
        "pivots": LOG["pivots"],
        "trace": REC.trace,
    }
