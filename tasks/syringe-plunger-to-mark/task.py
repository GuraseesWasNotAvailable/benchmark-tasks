# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Draw a 10 mL syringe's plunger back to the 5 mL line, the syringe resting on its tray.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A stainless instrument tray (300 x 160 x 20 mm, 2 mm sheet) stands on the table at x 0.18, long side along y.
- On its floor lies a standard 10 mL Luer syringe, axis along y, nozzle toward the robot's left: a clear barrel 16 mm
  across and 100 mm long (plus a 10 mm nozzle), a 34 x 24 mm finger flange, and a white plunger (black rubber stopper,
  cruciform rod, 24 mm thumb rest) pushed fully in. Graduations 6.0 mm per mL, the 5 mL line 30 mm from the empty
  stopper position. The barrel is a free body; the stopper's seal drags 5 N against the barrel (a sliding joint with
  5 N of Coulomb friction), about a hundred times the friction of the loose syringe on the tray.

Goal: the stopper's front edge on the 5 mL line (within 0.25 mL = 1.5 mm), the syringe resting on the tray, both
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
MM = 0.001

# -- layout ---------------------------------------------------------------------------------------------------------
TRAY_X, TRAY_Y = 0.18, 0.0
TRAY_W, TRAY_L, TRAY_H, SHEET = 0.160, 0.300, 0.020, 0.002  # x, y, wall height, sheet thickness
FLANGE_Y = -0.050  # world y of the flange face
AXIS_Z = SHEET + 0.012 + 0.0003  # syringe axis above the table top: flange and thumb rest (24 mm) stand on the floor
SYR_COLOR = (0.80, 0.90, 0.95)
STEEL = (0.72, 0.74, 0.77)

# -- syringe (mm in the syringe frame: x along the axis from the flange face toward the nozzle, z up) ----------------
BARREL_L = 100.0
BARREL_R = 8.0  # across flats 16 mm (12 slats)
WALL = 1.0
SLAT_W = 4.5
NOZZLE_L, NOZZLE_R = 10.0, 3.3
STOPPER_X0 = 98.0  # the stopper's front face when fully in (the barrel's end wall inner face is at 99)
MM_PER_ML = 6.0
MARK_ML = 5.0
MARK_MM = MARK_ML * MM_PER_ML  # 30 mm of draw
TOL_ML = 0.25  # +-1.5 mm
DRAW_MAX = 0.0625
SEAL_N = 5.0
ROD_END_X = -14.0  # rod end (thumb rest face) 14 mm behind the flange when fully in
THUMB_R, THUMB_T = 12.0, 3.0

t = lk.Tree("syringe", pos=(TRAY_X, FLANGE_Y, TOP), yaw_deg=90.0, color=SYR_COLOR)
slats = []
r_c = BARREL_R - WALL / 2
for k in range(12):
    th = 30.0 * k
    slats.append(lk.box((BARREL_L * MM, WALL * MM, SLAT_W * MM),
                        at=(BARREL_L / 2 * MM, r_c * MM * math.cos(math.radians(th)), r_c * MM * math.sin(math.radians(th))),
                        rpy_deg=(th, 0.0, 0.0), contact_offset=0.0005))
flange = [
    lk.box((3 * MM, 9 * MM, 24 * MM), at=(1.5 * MM, 12.5 * MM, 0.0), contact_offset=0.0005),
    lk.box((3 * MM, 9 * MM, 24 * MM), at=(1.5 * MM, -12.5 * MM, 0.0), contact_offset=0.0005),
    lk.box((3 * MM, 16 * MM, 4 * MM), at=(1.5 * MM, 0.0, 10 * MM), contact_offset=0.0005),
    lk.box((3 * MM, 16 * MM, 4 * MM), at=(1.5 * MM, 0.0, -10 * MM), contact_offset=0.0005),
]
RED, BLACK = (0.85, 0.10, 0.10), (0.08, 0.08, 0.08)
marks = [lk.box((1.0 * MM, 6 * MM, 0.3 * MM), at=((STOPPER_X0 - ml * MM_PER_ML) * MM, 0.0, (BARREL_R + 0.15) * MM),
                color=RED if ml == MARK_ML else BLACK, contact_offset=0.0003) for ml in (0.0, MARK_ML, 10.0)]
tip = [lk.cylinder(BARREL_R * MM, 2 * MM, at=((BARREL_L - 0.0) * MM, 0, 0), axis="X"),
       lk.cylinder(NOZZLE_R * MM, NOZZLE_L * MM, at=((BARREL_L + 1 + NOZZLE_L / 2) * MM, 0, 0), axis="X")]
t.link("barrel", slats + flange + marks + tip, mass=0.006, at=(0.0, 0.0, AXIS_Z), friction=(0.5, 0.5))

rod_len = STOPPER_X0 - 8.0 - ROD_END_X
rod_c = -(8.0 + rod_len / 2)
plunger = [
    lk.cylinder(6.7 * MM, 8 * MM, at=(-4 * MM, 0, 0), axis="X", color=BLACK),
    lk.box((rod_len * MM, 9 * MM, 1.8 * MM), at=(rod_c * MM, 0, 0), color=(0.95, 0.95, 0.95)),
    lk.box((rod_len * MM, 1.8 * MM, 9 * MM), at=(rod_c * MM, 0, 0), color=(0.95, 0.95, 0.95)),
    lk.cylinder(THUMB_R * MM, THUMB_T * MM, at=(-(STOPPER_X0 - ROD_END_X + THUMB_T / 2) * MM, 0, 0), axis="X",
                color=(0.95, 0.95, 0.95)),
]
t.link("plunger", plunger, mass=0.0045, at=(STOPPER_X0 * MM, 0.0, AXIS_Z), friction=(0.5, 0.5))
t.prismatic("draw", "barrel", "plunger", anchor=(STOPPER_X0 * MM, 0.0, AXIS_Z), axis=(-1.0, 0.0, 0.0),
            limits=(0.0, DRAW_MAX), stiffness=0.0, damping=0.5, friction=SEAL_N, dynamic_friction=SEAL_N)
SYRINGE = t.build()
THUMB_LOCAL = np.array([-(STOPPER_X0 - ROD_END_X + THUMB_T / 2) * MM, 0.0, 0.0])  # thumb rest centre, plunger frame
BARREL_GRASP = np.array([0.085, 0.0, 0.0])  # barrel frame: 85 mm past the flange face (100 mm from the thumb rest)
NOZZLE_LOCAL = np.array([(BARREL_L + 1 + NOZZLE_L) * MM, 0.0, 0.0])

t = lk.Tree("tray", pos=(TRAY_X, TRAY_Y, TOP), fixed=True, color=STEEL)
t.link("tray", [
    lk.box((TRAY_W, TRAY_L, SHEET), at=(0, 0, SHEET / 2)),
    lk.box((TRAY_W, SHEET, TRAY_H), at=(0, TRAY_L / 2 - SHEET / 2, TRAY_H / 2)),
    lk.box((TRAY_W, SHEET, TRAY_H), at=(0, -TRAY_L / 2 + SHEET / 2, TRAY_H / 2)),
    lk.box((SHEET, TRAY_L, TRAY_H), at=(TRAY_W / 2 - SHEET / 2, 0, TRAY_H / 2)),
    lk.box((SHEET, TRAY_L, TRAY_H), at=(-TRAY_W / 2 + SHEET / 2, 0, TRAY_H / 2)),
], mass=0.4, friction=(0.4, 0.4))
TRAY = t.build()

OBJECTS = [SYRINGE, TRAY]

# -- measurement and scoring ----------------------------------------------------------------------------------------
IN_X = (TRAY_X - TRAY_W / 2 + SHEET, TRAY_X + TRAY_W / 2 - SHEET)
IN_Y = (TRAY_Y - TRAY_L / 2 + SHEET, TRAY_Y + TRAY_L / 2 - SHEET)
OFF_MARGIN = 0.010  # a flange, nozzle or thumb rest more than 10 mm outside the tray's floor area: gate
SEEN_ML = MARK_ML - TOL_ML  # the 0.25 rung: the plunger was drawn to within one tolerance of the line at some point
REST_Z_TOL = 0.004
REST_TILT_DEG = 8.0
STILL_V, STILL_W = 0.003, 0.1  # m/s, rad/s, from pose change between control steps
CLEAR_M = 0.030
TCP_STILL = 0.01
RELEASE_S = 1.0
REST_S = 1.0
EPS = 1e-6


def measure(syr, tools) -> dict:
    B = np.asarray(syr.poses["barrel"], dtype=float)
    P = np.asarray(syr.poses["plunger"], dtype=float)
    axis = B[:3, :3][:, 0]
    pts = {"flange": B[:3, 3], "nozzle": B[:3, :3] @ NOZZLE_LOCAL + B[:3, 3], "thumb": P[:3, :3] @ THUMB_LOCAL + P[:3, 3]}
    clear = {s: lk.clearance(tl, syr) for s, tl in tools.items()}
    return {
        "draw_mm": float(syr.q["draw"]) * 1000.0,
        "pts": {k: tuple(float(v) for v in p) for k, p in pts.items()},
        "z_axis": float(B[2, 3]) - TOP,
        "tilt": math.degrees(math.asin(max(-1.0, min(1.0, float(axis[2]))))),
        "B": B,
        "clear": clear,
        "tcp_speed": {s: float(tl.speed) for s, tl in tools.items()},
    }


def on_tray(m, margin=0.0) -> bool:
    return all(IN_X[0] - margin <= p[0] <= IN_X[1] + margin and IN_Y[0] - margin <= p[1] <= IN_Y[1] + margin
               for p in m["pts"].values())


def vol_ml(m) -> float:
    return m["draw_mm"] / MM_PER_ML


def resting(m) -> bool:
    return on_tray(m) and abs(m["z_axis"] - AXIS_Z) <= REST_Z_TOL and abs(m["tilt"]) <= REST_TILT_DEG


def pose_rates(A_prev, A, dt: float) -> tuple[float, float]:
    """Speed (m/s) and spin (rad/s) from the pose change over ``dt``: what a camera sees."""
    if dt <= 0.0:
        return 0.0, 0.0
    v = float(np.linalg.norm(A[:3, 3] - A_prev[:3, 3])) / dt
    R = A_prev[:3, :3].T @ A[:3, :3]
    sin = 0.5 * float(np.linalg.norm((R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1])))
    return v, math.atan2(sin, (float(np.trace(R)) - 1.0) / 2.0) / dt


class Record:
    """Whole-run record: a linkkit History updated every control step and cleared only by the simulator reset (so a
    syringe knocked off the tray and put back never clears the gate), plus the latest sample. Pure Python."""

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
        H.peak("draw_max_ml", vol_ml(m))
        H.ever("off_tray", not on_tray(m, OFF_MARGIN))
        H.ever("drawn", vol_ml(m) >= SEEN_ML)
        at_mark = abs(vol_ml(m) - MARK_ML) <= TOL_ML + EPS
        H.held("rest", at_mark and resting(m) and m["b_speed"] < STILL_V and m["b_spin"] < STILL_W)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Score and the reason: the gate first (any time in the run), then the rungs from the top."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "off_tray" in H.evers:
        return 0.0, f"gate: the syringe went more than 10 mm off the tray at {H.evers['off_tray']:.2f} s"
    if abs(vol_ml(m) - MARK_ML) <= TOL_ML + EPS:
        if resting(m) and H.runs.get("rest", 0.0) >= REST_S - EPS:
            if H.runs.get("released", 0.0) >= RELEASE_S - EPS:
                return 1.0, "stopper on the 5 mL line, syringe resting on the tray, both grippers let go"
            return 0.75, "stopper on the line and the syringe resting; the grippers are not 30 mm clear and still for 1 s"
        return 0.5, "stopper on the 5 mL line at the end, but the syringe is not resting still on the tray for 1 s"
    if "drawn" in H.evers:
        return 0.25, "the plunger was drawn to the line at some point, but the stopper is not on it at the end"
    return 0.0, "the plunger was never drawn to the 5 mL line"


REC = Record()
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    m = measure(tick.states["syringe"], tick.tools)
    m["b_speed"] = m["b_spin"] = 0.0
    if _PREV:
        m["b_speed"], m["b_spin"] = pose_rates(_PREV["B"], m["B"], tick.t - _PREV["t"])
    _PREV.update(B=m["B"], t=tick.t)
    REC.update(m)
    if REC.n % 5 == 0:
        REC.trace.append((round(tick.t, 2), round(vol_ml(m), 2), round(m["z_axis"] * 1000, 1), round(m["tilt"], 1),
                          [round(1000 * m["pts"]["flange"][i], 1) for i in (0, 1)],
                          [round(1000 * m["clear"][s], 0) for s in ("left", "right")]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo (camera-observable state only) ----------------------------------------------------------------------
VARIANTS = ("", "nothing", "left_only", "right_only", "pull_short")
LOG: dict = {}
CTX: list = []
HOVER = 0.06
TCP_ABOVE = 0.004  # TCP above the grasp point: the pads cover the syringe axis (tips 13 mm below the TCP)
OPEN_BARREL = 0.040
OPEN_THUMB = 0.045
SQUEEZE_BARREL = 0.006  # commanded 6 mm under the 16 mm barrel: about 15 N per finger
SQUEEZE_THUMB = 0.008
PULL_S = 5.0


def seen_draw_mm(ctx) -> float:
    s = ctx.observe()
    B, P = s.pose("syringe", "barrel"), s.pose("syringe", "plunger")
    return 1000.0 * (STOPPER_X0 * MM - float((B[:3, :3].T @ (P[:3, 3] - B[:3, 3]))[0]))


LEAN_DEG = 25.0  # each gripper's body leans away from the other arm (outward along y), pads still closing along x


def grasp(arm, ctx, point, width_open, width_shut, st, key, outward):
    """Grip with the approach pointing down and ``outward`` (+1 left, -1 right) toward the table centre, so the two
    gripper bodies splay apart over the 100 mm between the barrel grip and the thumb rest; pads close along world x."""
    th = math.radians(LEAN_DEG)
    app = np.array([0.0, -outward * math.sin(th), -math.cos(th)])
    cx = np.array([-outward, 0.0, 0.0])  # the wrist turn each arm reaches (the gripper itself is symmetric)
    rot = np.column_stack([cx, np.cross(app, cx), app])
    tcp = point - app * TCP_ABOVE
    yield from arm.move(tcp - app * HOVER, rot, width=width_open, duration=2.5)
    yield from arm.wait(0.4)
    tl = ctx.observe().tools[arm.side]
    st[key + "_hover_err_mm"] = [round(1000 * float(v), 1) for v in (tl.tcp - arm.pos)]
    yield from arm.move(tcp, rot, width=width_open, duration=1.5)
    yield from arm.grip(width_shut, 0.8)
    yield from arm.wait(0.5)
    st[key] = round(ctx.t, 2)
    tool = ctx.observe().tools[arm.side]


def release(arm):
    yield from arm.grip(0.094, 0.6)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.07]), duration=1.2)


def demo_script(ctx):
    variant = VARIANT
    LOG.clear()
    st = LOG
    st["variant"] = variant
    CTX[:] = [ctx]
    yield from ctx.wait(1.0)
    if variant == "nothing":
        yield from ctx.wait(8.0)
        return
    seen = ctx.observe()
    B = seen.pose("syringe", "barrel")
    axis = B[:3, :3][:, 0]  # toward the nozzle
    barrel_pt = seen.point("syringe", "barrel", BARREL_GRASP)
    thumb_pt = seen.point("syringe", "plunger", THUMB_LOCAL)
    target = 15.0 if variant == "pull_short" else MARK_MM
    st["draw0_mm"] = round(seen_draw_mm(ctx), 2)

    def left():
        if variant == "right_only":
            return
        yield from grasp(ctx.left, ctx, barrel_pt, OPEN_BARREL, 0.016 - SQUEEZE_BARREL, st, "left_gripped", 1.0)
        if variant == "left_only":
            yield from ctx.left.wait(10.0)
            yield from release(ctx.left)
            return
        while not st.get("right_done"):
            yield from ctx.left.wait(0.1)
        yield from release(ctx.left)

    def right():
        if variant == "left_only":
            return
        yield from grasp(ctx.right, ctx, thumb_pt, OPEN_THUMB, 0.024 - SQUEEZE_THUMB, st, "right_gripped", -1.0)
        while variant != "right_only" and "left_gripped" not in st:
            yield from ctx.right.wait(0.1)
        arm = ctx.right
        yield from arm.move(arm.pos - axis * (target - 3.0) * MM, duration=PULL_S)
        st["pull_log"] = []
        for k in range(8):
            # the arm is soft (about 0.3 N/mm at the hand), so the stopper creeps toward the hand's position until the
            # pull falls to the 5 N seal drag: wait until it stops, then move the hand by the error seen from the poses
            last = None
            for _ in range(24):
                yield from arm.wait(0.4)
                now = seen_draw_mm(ctx)
                if last is not None and abs(now - last) < 0.06:
                    break
                last = now
            st["pull_log"].append(round(now, 2))
            err = target - now
            if variant in ("right_only", "left_only") or abs(err) <= 0.4:
                break
            yield from arm.move(arm.pos - axis * err * MM, duration=max(0.8, abs(err) / 4.0))
        st["draw_end_mm"] = round(seen_draw_mm(ctx), 2)
        st["right_done"] = True
        yield from release(arm)

    yield from ctx.both(left(), right())
    yield from ctx.wait(3.5)


def demo_report() -> dict:
    score_, why = ladder(REC)
    H = REC.H
    m = REC.last or {}
    last = {}
    if m:
        last = {"vol_ml": round(vol_ml(m), 3), "draw_mm": round(m["draw_mm"], 2), "z_axis_mm": round(1000 * m["z_axis"], 2),
                "tilt_deg": round(m["tilt"], 2), "clear_mm": {s: round(1000 * v, 1) for s, v in m["clear"].items()},
                "flange_xy_mm": [round(1000 * v, 1) for v in m["pts"]["flange"][:2]]}
    return {
        "score": score_, "reason": why, "variant": VARIANT,
        "evers": {k: round(v, 2) for k, v in H.evers.items()},
        "peaks": {k: round(v, 2) for k, v in H.peaks.items()},
        "runs": {k: round(v, 2) for k, v in H.runs.items()},
        "last": last, "log": dict(LOG), "trace": REC.trace,
    }
