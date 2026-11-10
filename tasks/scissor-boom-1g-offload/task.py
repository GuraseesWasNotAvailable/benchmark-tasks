# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Deploy a scissor boom at 1 g, carrying its tip level by its offload sling so it never sags onto the solar cells.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475. Boom frame R: origin on the root hinge's axis, world (0.300, 0.100, 0.8375),
axes parallel to the world's; the boom extends along -y (u = -y_R is the distance out from the hinge).

- A grey bus bracket with a gold face stands on the table. On its root hinge (a revolute about x with a torsion
  spring, 0.9 N m/rad, preloaded 10 deg upward) hangs the white boom: a root fitting with a rail, two root sliders
  driven toward each other by a self-locking left/right-hand lead screw, and a lazy-tongs scissor of two full bays
  (130 mm bars crossing at their middles) closed by two half bars at a tip apex. A white dummy magnetometer canister
  pivots on the apex pin. On its top stands the offload sling: a grey steel bail, pinned about x 12 mm above the
  apex, 40 mm up to a black grip sleeve that turns freely on its axle; a light spring stands it upright.
- A black hand crank on its own gearbox (left of the bracket) turns the screw: 11 mm of slider travel per turn, 2.98
  turns from stowed to full length. The screw is self-locking (nothing but the crank moves the sliders) and a slip
  clutch in the crank hub slips once the screw is 1.5 mm ahead of a stalled slider.
- A blue solar-cell test panel lies under the boom's path, 30 mm below the canister when the boom is level.

The hinge is sized for zero g. Stowed, the spring holds the short boom about 1.2 deg below level; as the scissor
extends the cantilever moment doubles and, unsupported, the hinge droops past 5 deg, so cranking alone lays the
canister on the cells a little over half-way out. Pulling the tip out does not extend it (the screw is self-locking).
The scissor, not the hand, sets where the tip is: per crank turn it runs out about 5 times faster at the start of the
stroke than at the end, so a hand that holds the sling still, or moves out at a steady speed, tilts the sling. The
root latch only engages at full length with the hinge within 2 deg of level.

Goal: latched at full length, then both grippers let go. For the whole run: the canister never within 10 mm of the
cells (the flight cells' keep-out); the hinge never more than 3 deg from level (the root hinge's rated deflection);
no finger within 5 mm of the boom anywhere but its sling (flight hardware is handled only at its ground-support
points); and, while a hand is on the sling and the tip is moving, the sling never more than 15 deg off vertical (a
tilted offload cable side-loads the boom's pins).
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

ORIGIN = np.array([0.300, 0.100, TOP + 0.090])  # boom frame R: root hinge axis
CRANK_AT = np.array([0.300, 0.250, TOP + 0.060])  # crank axis on the gearbox top

WHITE = (0.92, 0.92, 0.90)
GREY = (0.62, 0.63, 0.66)
DARK = (0.30, 0.31, 0.33)
GOLD = (0.83, 0.65, 0.22)
BLACK = (0.08, 0.08, 0.09)
CELL_BLUE = (0.10, 0.18, 0.55)
ALU = (0.72, 0.73, 0.75)

# scissor geometry (m; R frame)
L_BAR = 0.130  # pin to pin, full bars
PHI_S = math.radians(68.0)  # stowed: bars 68 deg from the boom axis
PHI_D = math.radians(25.0)  # full length
R0 = 0.020  # root pins' distance out from the hinge axis
W_S = L_BAR * math.sin(PHI_S)  # root pin spacing stowed (0.1205)
W_D = L_BAR * math.sin(PHI_D)  # at full length (0.0549)
Q_FULL = (W_S - W_D) / 2.0  # each slider's travel (0.0328)
Z_A, Z_B = 0.003, -0.003  # bar layers (top, bottom)
BAR_W, BAR_T = 0.010, 0.005
Z_RAIL = 0.012  # slider pin height
CAN_R, CAN_BOTTOM = 0.018, -0.0365  # tip canister radius, bottom (tip frame)
BAIL_PIN_Z, BAIL_L = 0.012, 0.040  # offload sling (bail): pinned on the canister's post, 40 mm pin to grip axle
GRIP_Z = BAIL_PIN_Z + BAIL_L  # grip sleeve axis (tip frame, bail upright)
PAD_TIP = 0.013  # TCP to pad tips
GRASP_Z = GRIP_Z - 0.004 + PAD_TIP  # TCP in the tip frame: pad tips 4 mm below the sleeve's axis

# root hinge
HINGE_K, HINGE_C = 0.9, 0.03  # N m/rad, N m s/rad
HINGE_REST_DEG = -10.0  # spring rest (preloaded upward)
HINGE_INIT_DEG = 1.2  # stowed equilibrium
LATCH_K, LATCH_C = 40.0, 1.0

# screw drive
LEAD = 0.011  # m of slider travel per crank turn
SLIP = 0.0015  # slip clutch: the screw runs at most 1.5 mm ahead of the sliders
SCREW_K, SCREW_C = 60000.0, 600.0  # N/m, N s/m

# cells and scoring
CELLS_TOP = TOP + 0.0235  # 30 mm below the canister of a level boom
CLEAR_MIN_MM = 10.0
SLING_DEG = 15.0  # while carrying the moving tip, the sling hangs within 15 deg of vertical (grip within 10.4 mm)
TOUCH_MM = 5.0  # no finger within 5 mm of the boom except its sling (bail and grip sleeve)
TIP_MOVING = 0.002  # m/s: the tip counts as moving above this speed, or on any step the screw advances
LEVEL_DEG = 3.0
LATCH_Q = 0.0005  # sliders within 0.5 mm of full travel
LATCH_DEG = 2.0  # hinge within 2 deg of level for the latch pawl to drop in
HALF = 0.5
CLEAR_M = 0.030
TCP_STILL = 0.005
RELEASE_S = 1.0
EPS = 1e-6


def phi_of(q: float) -> float:
    """Bar angle from the boom axis at slider travel q (each slider moved q inward)."""
    return math.asin(min(1.0, max(0.0, (W_S - 2.0 * q) / L_BAR)))


def u_tip(q: float) -> float:
    """Apex distance out from the hinge axis (along -y_R) at slider travel q."""
    return R0 + 2.5 * L_BAR * math.cos(phi_of(q))


U_S, U_D = u_tip(0.0), u_tip(Q_FULL)  # 0.142, 0.315

# -- the boom -----------------------------------------------------------------------------------------------------

_c, _w = L_BAR * math.cos(PHI_S), W_S
P = {  # pin points (x, y) in R at the stowed design pose
    "root_p": (_w / 2, -R0), "root_n": (-_w / 2, -R0),
    "j1_p": (_w / 2, -(R0 + _c)), "j1_n": (-_w / 2, -(R0 + _c)),
    "j2_p": (_w / 2, -(R0 + 2 * _c)), "j2_n": (-_w / 2, -(R0 + 2 * _c)),
    "x1": (0.0, -(R0 + 0.5 * _c)), "x2": (0.0, -(R0 + 1.5 * _c)), "apex": (0.0, -(R0 + 2.5 * _c)),
}


def _pin(name: str, z: float = 0.0) -> tuple:
    return (P[name][0], P[name][1], z)


def _bar(t: lk.Tree, name: str, a: str, b: str, z: float, mass: float) -> None:
    (x0, y0), (x1, y1) = P[a], P[b]
    length = math.hypot(x1 - x0, y1 - y0)
    t.link(name, [lk.box((length + BAR_W, BAR_W, BAR_T))], mass=mass, at=((x0 + x1) / 2, (y0 + y1) / 2, z),
           rpy_deg=(0.0, 0.0, math.degrees(math.atan2(y1 - y0, x1 - x0))), color=WHITE, friction=(0.5, 0.4))


Z_AXIS = (0.0, 0.0, 1.0)
PIN = {"axis": Z_AXIS, "damping": 5e-4, "armature": 1e-5}

_t = lk.Tree("boom", pos=tuple(ORIGIN), fixed=True, color=WHITE, self_collision=False)
_t.link("bus", [
    lk.box((0.060, 0.040, 0.082), (0.0, 0.025, -0.049), color=GREY),  # bracket tower on the bus side panel
    lk.box((0.100, 0.080, 0.006), (0.0, 0.025, -0.087), color=GREY),  # foot on the table
    lk.box((0.058, 0.002, 0.070), (0.0, 0.004, -0.045), color=GOLD),  # gold MLI face
    lk.box((0.006, 0.016, 0.020), (0.021, 0.0, 0.0), color=GREY),  # hinge cheeks
    lk.box((0.006, 0.016, 0.020), (-0.021, 0.0, 0.0), color=GREY),
], mass=2.0)
_t.link("root", [
    lk.cylinder(0.007, 0.034, (0.0, 0.0, 0.0), axis="X"),  # hinge knuckle
    lk.box((0.030, R0, 0.008), (0.0, -R0 / 2, 0.008)),  # fitting out to the rail
    lk.box((0.150, 0.012, 0.006), (0.0, -R0, 0.021)),  # rail along x
    lk.cylinder(0.003, 0.140, (0.0, -R0 - 0.009, 0.016), axis="X", color=BLACK),  # left/right-hand lead screw
], mass=0.050, color=WHITE)
for side, sx in (("sp", 1.0), ("sn", -1.0)):
    _t.link(side, [
        lk.box((0.014, 0.016, 0.010), (0.0, 0.0, 0.0)),  # slider block on the rail
        lk.cylinder(0.003, 0.012, (0.0, 0.0, -0.007), axis="Z", color=GREY),  # pin down to the bars
    ], mass=0.008, at=(sx * _w / 2, -R0, Z_RAIL), color=DARK)
_bar(_t, "a1", "root_p", "j1_n", Z_A, 0.015)
_bar(_t, "b1", "root_n", "j1_p", Z_B, 0.015)
_bar(_t, "a2", "j1_p", "j2_n", Z_A, 0.015)
_bar(_t, "b2", "j1_n", "j2_p", Z_B, 0.015)
_bar(_t, "c", "j2_n", "apex", Z_B, 0.008)
_bar(_t, "d", "j2_p", "apex", Z_A, 0.008)
_t.link("tip", [
    lk.cylinder(CAN_R, 0.030, (0.0, 0.0, CAN_BOTTOM + 0.015), axis="Z"),  # dummy magnetometer canister
    lk.cylinder(0.005, BAIL_PIN_Z - 0.0055, (0.0, 0.0, (BAIL_PIN_Z + 0.0055) / 2), axis="Z", color=GREY),  # post
    lk.box((0.014, 0.008, 0.008), (0.0, 0.0, BAIL_PIN_Z), color=GREY),  # sling clevis
], mass=0.060, at=_pin("apex"), color=WHITE, friction=(0.9, 0.8))
_t.link("bail", [
    lk.box((0.005, 0.005, BAIL_L), (0.0, 0.0, 0.0)),  # steel strap, pin to axle
    lk.cylinder(0.003, 0.036, (0.0, 0.0, BAIL_L / 2), axis="X"),  # grip axle along x
], mass=0.010, at=(P["apex"][0], P["apex"][1], BAIL_PIN_Z + BAIL_L / 2), color=GREY)
_t.link("sleeve", [lk.cylinder(0.006, 0.026, (0.0, 0.0, 0.0), axis="X")], mass=0.005,
        at=(P["apex"][0], P["apex"][1], GRIP_Z), color=BLACK, friction=(1.0, 1.0))
_t.revolute("hinge", "bus", "root", anchor=(0.0, 0.0, 0.0), axis=(1, 0, 0), limits_deg=(-12.0, 30.0),
            rest_deg=HINGE_REST_DEG, init_deg=HINGE_INIT_DEG, stiffness=HINGE_K, damping=HINGE_C, armature=1e-4,
            brake=lk.Brake(stiffness=LATCH_K, damping=LATCH_C, engaged=False))
_t.prismatic("slide_p", "root", "sp", anchor=_pin("root_p", Z_RAIL), axis=(-1, 0, 0),
             limits=(-0.0005, Q_FULL + 0.0005), stiffness=SCREW_K, damping=SCREW_C)
_t.prismatic("slide_n", "root", "sn", anchor=_pin("root_n", Z_RAIL), axis=(1, 0, 0),
             limits=(-0.0005, Q_FULL + 0.0005), stiffness=SCREW_K, damping=SCREW_C)
_t.revolute("pin_a1", "sp", "a1", anchor=_pin("root_p"), **PIN)
_t.revolute("x1", "a1", "b1", anchor=_pin("x1"), **PIN)
_t.revolute("j1", "b1", "a2", anchor=_pin("j1_p"), **PIN)
_t.revolute("x2", "a2", "b2", anchor=_pin("x2"), **PIN)
_t.revolute("j2c", "a2", "c", anchor=_pin("j2_n"), **PIN)
_t.revolute("j2d", "b2", "d", anchor=_pin("j2_p"), **PIN)
_t.revolute("apex", "d", "tip", anchor=_pin("apex"), axis=Z_AXIS, damping=1e-3, armature=1e-5)
_t.revolute("sling", "tip", "bail", anchor=(P["apex"][0], P["apex"][1], BAIL_PIN_Z), axis=(1, 0, 0),
            limits_deg=(-40.0, 40.0), stiffness=0.05, damping=0.002, armature=1e-5)  # light spring: stands upright
_t.revolute("grip", "bail", "sleeve", anchor=(P["apex"][0], P["apex"][1], GRIP_Z), axis=(1, 0, 0), damping=1e-4,
            armature=1e-5)  # the grip sleeve turns freely on the axle
_t.loop("loop_root", "sn", "b1", anchor=_pin("root_n"), axis=Z_AXIS)
_t.loop("loop_j1", "a1", "b2", anchor=_pin("j1_n"), axis=Z_AXIS)
_t.loop("loop_apex", "c", "d", anchor=_pin("apex"), axis=Z_AXIS)
BOOM = _t.build()
MOVING = ("root", "sp", "sn", "a1", "b1", "a2", "b2", "c", "d", "tip", "bail", "sleeve")
LOOPS = ("loop_root", "loop_j1", "loop_apex")
SLING = ("bail", "sleeve")  # the only handling point: the offload sling's bail and its grip sleeve
HANDS_OFF = tuple(k for k in MOVING if k not in SLING)  # every other moving part of the boom

# The crank is its own articulation (no joint limits anywhere in it), so its joints turn without end.
_t = lk.Tree("crank", pos=tuple(CRANK_AT), fixed=True, color=BLACK)
_t.link("gearbox", [lk.box((0.040, 0.040, 0.050), (0.0, 0.0, -0.025))], mass=0.8)
_t.link("arm", [
    lk.cylinder(0.010, 0.008, (0.0, 0.0, 0.004)),
    lk.box((0.055, 0.012, 0.006), (0.0225, 0.0, 0.005)),
], mass=0.060)
_t.link("handle", [lk.capsule(0.007, 0.020, (0.0, 0.0, 0.0), axis="Z")], mass=0.015, at=(0.045, 0.0, 0.025),
        friction=(1.0, 1.0))
_t.revolute("turn", "gearbox", "arm", anchor=(0.0, 0.0, 0.0), axis=(0, 0, 1), damping=0.005, friction=0.02,
            armature=1e-4)
_t.revolute("spin", "arm", "handle", anchor=(0.045, 0.0, 0.008), axis=(0, 0, 1), damping=1e-4, armature=1e-5)
CRANK = _t.build()

# Solar-cell test panel: one fixed compound body (kinematic), top 30 mm below a level canister.
CELLS_AT = (0.300, -0.0925, CELLS_TOP - 0.004)
_t = lk.Tree("cells", pos=CELLS_AT, fixed=True, color=CELL_BLUE)
_t.link("panel", [
    lk.box((0.120, 0.305, 0.008), (0.0, 0.0, 0.0), color=CELL_BLUE),  # cells on an aluminium honeycomb substrate
    lk.box((0.100, 0.012, CELLS_TOP - 0.008 - TOP), (0.0, 0.140, -(CELLS_TOP - TOP) / 2), color=ALU),  # legs
    lk.box((0.100, 0.012, CELLS_TOP - 0.008 - TOP), (0.0, -0.140, -(CELLS_TOP - TOP) / 2), color=ALU),
], mass=0.6)
CELLS = _t.build()

OBJECTS = [BOOM, CRANK, CELLS]

# -- the screw and the whole-run record (pure Python, so the tests can drive them) -----------------------------------


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Screw:
    """Crank angle -> the sliders' drive target s (m, 0 stowed .. Q_FULL). Self-locking: s changes only when the
    crank turns, never because something pushes or pulls the boom. Slip clutch: while cranking out, s runs at most
    SLIP ahead of the slower slider (and while cranking in, at most SLIP behind the faster one). Once latched, the
    screw is locked at full length."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.s, self.phi, self.locked, self.slipped = 0.0, None, False, 0.0

    def step(self, phi: float, q_lo: float, q_hi: float) -> float:
        if self.phi is not None and not self.locked:
            d = wrap(phi - self.phi) * LEAD / (2.0 * math.pi)
            want = self.s + d
            if d > 0.0:
                new = min(want, max(self.s, q_lo + SLIP))
            elif d < 0.0:
                new = max(want, min(self.s, q_hi - SLIP))
            else:
                new = self.s
            self.slipped += abs(want - new)
            self.s = min(max(new, 0.0), Q_FULL)
        self.phi = phi
        return self.s


class Record:
    """Whole-run record (a linkkit History plus latch and touch events), updated every control step and cleared only
    by the simulator reset."""

    def __init__(self):
        self.H = lk.History()
        self.reset()

    def reset(self) -> None:
        self.H.reset()
        self.latch: dict | None = None
        self.low_event: dict | None = None
        self.tilt_event: dict | None = None
        self.sling_event: dict | None = None
        self.touch_event: dict | None = None
        self.last: dict | None = None
        self.n = 0
        self.trace: list = []

    def update(self, m: dict, t: float | None = None) -> None:
        if t is not None:
            self.H._tick(t)
        self.n += 1
        self.last = m
        H = self.H
        H.low("clear_mm", m["clear_mm"])
        H.peak("abs_hinge_deg", abs(m["hinge_deg"]))
        H.peak("hinge_deg", m["hinge_deg"])
        H.low("hinge_deg", m["hinge_deg"])
        H.peak("ext", m["ext"])
        H.peak("strain_mm", m["strain_mm"])
        if m["clear_mm"] < CLEAR_MIN_MM - EPS and self.low_event is None:
            self.low_event = {"t": round(H.t, 2), "clear_mm": round(m["clear_mm"], 1), "ext": round(m["ext"], 3),
                              "hinge_deg": round(m["hinge_deg"], 2)}
        if abs(m["hinge_deg"]) > LEVEL_DEG + EPS and self.tilt_event is None:
            self.tilt_event = {"t": round(H.t, 2), "hinge_deg": round(m["hinge_deg"], 2), "ext": round(m["ext"], 3)}
        H.peak("abs_sling_deg", abs(m["sling_deg"]))
        carrying = bool(m["on_sling"]) and bool(m["moving"])  # a hand on the sling while the tip moves
        if carrying:
            H.peak("abs_sling_deg_carrying", abs(m["sling_deg"]))
        if carrying and abs(m["sling_deg"]) > SLING_DEG + EPS and self.sling_event is None:
            self.sling_event = {"t": round(H.t, 2), "sling_deg": round(m["sling_deg"], 1), "ext": round(m["ext"], 3)}
        H.low("touch_mm", m["touch_mm"])
        if m["touch_mm"] < TOUCH_MM - EPS and self.touch_event is None:
            self.touch_event = {"t": round(H.t, 2), "touch_mm": round(m["touch_mm"], 1), "by": m["touch_by"],
                                "ext": round(m["ext"], 3)}
        if m["latched"] and self.latch is None:
            self.latch = {"t": round(H.t, 2), "hinge_deg": round(m["hinge_deg"], 3), "q_mm": round(m["q_mm"], 2)}
        clear = m["clear"]
        H.held("released", bool(clear) and min(clear.values()) >= CLEAR_M - EPS
               and max(m["tcp_speed"].values()) < TCP_STILL)


def gates(rec: Record, skip: tuple = ()) -> list:
    """Every whole-run gate that fired, as (time, reason), earliest first."""
    out = []
    e = rec.low_event
    if e is not None and "cells" not in skip:
        out.append((e["t"], f"tip came {e['clear_mm']:.1f} mm from the cells (under {CLEAR_MIN_MM:.0f} mm) at t "
                            f"{e['t']:.1f} s, {100 * e['ext']:.0f}% out, hinge {e['hinge_deg']:+.1f} deg"))
    e = rec.tilt_event
    if e is not None and "hinge" not in skip:
        out.append((e["t"], f"root hinge {e['hinge_deg']:+.2f} deg from level (over {LEVEL_DEG:.0f} deg) at t "
                            f"{e['t']:.1f} s, {100 * e['ext']:.0f}% out"))
    e = rec.sling_event
    if e is not None and "sling" not in skip:
        side = "behind its pin (toward the root)" if e["sling_deg"] > 0 else "ahead of its pin"
        out.append((e["t"], f"sling {abs(e['sling_deg']):.1f} deg off vertical, its grip {side}, while carrying the "
                            f"moving tip at t {e['t']:.1f} s, {100 * e['ext']:.0f}% out"))
    e = rec.touch_event
    if e is not None and "touch" not in skip:
        out.append((e["t"], f"{e['by']} {e['touch_mm']:.1f} mm away (under {TOUCH_MM:.0f} mm; only the sling may be "
                            f"handled) at t {e['t']:.1f} s"))
    return sorted(out)


def ladder(rec: Record, skip: tuple = ()) -> tuple[float, str]:
    m = rec.last
    if m is None:
        return 0.0, "no steps yet"
    H = rec.H
    fired = gates(rec, skip)
    if fired:
        return 0.0, fired[0][1]
    ext = H.peaks.get("ext", 0.0)
    if ext < HALF - EPS:
        return 0.0, f"boom only {100 * max(ext, 0.0):.0f}% deployed"
    if not m["full"]:
        return 0.25, f"boom {100 * m['ext']:.0f}% deployed, not at full length"
    if not m["latched"]:
        return 0.5, f"at full length, not latched (hinge {m['hinge_deg']:+.2f} deg)"
    tail = (f"latched at {rec.latch['hinge_deg']:+.2f} deg; low clearance {H.lows.get('clear_mm', 0.0):.1f} mm; "
            f"peak hinge {H.peaks.get('abs_hinge_deg', 0.0):.2f} deg")
    if H.runs.get("released", 0.0) >= RELEASE_S - EPS:
        return 1.0, tail + "; let go"
    return 0.75, tail + "; grippers not clear and still 1 s"


def sling_deg(pin, top) -> float:
    """The sling's tilt from vertical in the boom's plane (deg): + when its grip is behind its pin (toward the root,
    +y): the hand lags the tip; - when it leads."""
    d = np.asarray(top, dtype=float) - np.asarray(pin, dtype=float)
    return math.degrees(math.atan2(d[1], d[2]))


def tip_low_z(b) -> float:
    """Lowest point of the canister's bottom rim (world z)."""
    return min(float(b.point("tip", (CAN_R * math.cos(a), CAN_R * math.sin(a), CAN_BOTTOM))[2])
               for a in np.linspace(0.0, 2.0 * math.pi, 12, endpoint=False))


FINGER_REACH = 0.066  # m: no finger point lies further than this from the TCP


def finger_points(tool) -> np.ndarray:
    """World points filling both fingers' last 48 mm (tool frame: x closing, z approach; from 35 mm behind the TCP
    to the pad tips 13 mm past it), from each pad face 7 mm outward and 16 to 24 mm across, no more than 6 mm apart
    (sized from the YAM finger meshes)."""
    pts = []
    for z in np.linspace(-0.035, PAD_TIP, 9):
        h = 0.008 if z > 0.006 else 0.012
        for y in np.linspace(-h, h, 5):
            for t in (0.0, 0.0035, 0.007):
                for sgn in (-1.0, 1.0):
                    pts.append((sgn * (tool.opening / 2 + t), y, z))
    return np.asarray(tool.tcp, dtype=float) + np.asarray(pts) @ np.asarray(tool.rot, dtype=float).T


def near_mm(poses: dict, spec, links, pts) -> float:
    """Smallest distance (mm) from any of the world points ``pts`` to the colliders of ``links``."""
    pts = np.atleast_2d(np.asarray(pts, dtype=float))
    best = math.inf
    for name in links:
        m = poses[name]
        local = (pts - m[:3, 3]) @ m[:3, :3]
        for s in spec.link(name).shapes:
            p = (local - np.asarray(s.pos)) @ s.rot
            if s.kind == "box":
                d = np.linalg.norm(np.maximum(np.abs(p) - np.asarray(s.dims) / 2, 0.0), axis=1)
            else:
                i = "XYZ".index(s.axis)
                along = np.abs(p[:, i])
                radial = np.sqrt(np.maximum((p * p).sum(axis=1) - p[:, i] ** 2, 0.0))
                r, h = s.dims[0], s.dims[1] / 2
                if s.kind == "capsule":
                    d = np.maximum(np.hypot(radial, np.maximum(along - h, 0.0)) - r, 0.0)
                else:
                    d = np.hypot(np.maximum(radial - r, 0.0), np.maximum(along - h, 0.0))
            best = min(best, float(d.min()))
    return 1000.0 * best


def hands(poses: dict, spec, tools: dict) -> dict:
    """Per gripper: (its fingers' distance in mm to the boom outside the sling, that link, their distance in mm to
    the sling); 999 for a hand too far away to matter."""
    out = {}
    for k, t in tools.items():
        if near_mm(poses, spec, HANDS_OFF + SLING, t.tcp) > 1000.0 * FINGER_REACH + TOUCH_MM + 1.0:
            out[k] = (999.0, "", 999.0)
            continue
        pts = finger_points(t)
        per = {n: near_mm(poses, spec, (n,), pts) for n in HANDS_OFF}
        link = min(per, key=per.get)
        out[k] = (min(per[link], 999.0), link, min(near_mm(poses, spec, SLING, pts), 999.0))
    return out


def measure(b, cr, tools, s: float, latched: bool, advanced: bool = False, near: dict | None = None,
            touch: dict | None = None) -> dict:
    qp, qn = b.q["slide_p"], b.q["slide_n"]
    if near is None:
        near = {k: min(lk.clearance(t, b, MOVING), lk.clearance(t, cr, ["arm", "handle"])) for k, t in tools.items()}
    if touch is None:
        touch = hands(b.poses, b.spec, tools)
    side = min(touch, key=lambda k: touch[k][0])
    apex = b.point("tip")
    u = float(ORIGIN[1] - apex[1])
    tip_speed = float(np.linalg.norm(b.lin_vels["tip"]))
    return {"q_mm": 500.0 * (qp + qn), "hinge_deg": b.deg("hinge"), "ext": (u - U_S) / (U_D - U_S),
            "clear_mm": (tip_low_z(b) - CELLS_TOP) * 1000.0,
            "sling_deg": sling_deg(b.point("bail", (0.0, 0.0, -BAIL_L / 2)), b.point("bail", (0.0, 0.0, BAIL_L / 2))),
            "strain_mm": 1000.0 * max(abs(qp - s), abs(qn - s)),
            "full": min(qp, qn) >= Q_FULL - LATCH_Q - EPS, "latched": bool(latched), "clear": near,
            "tcp_speed": {k: float(t.speed) for k, t in tools.items()},
            "touch_mm": touch[side][0], "touch_by": f"{side} finger at the boom's {touch[side][1] or 'parts'}",
            "on_sling": any(v[2] <= TOUCH_MM for v in touch.values()),
            "moving": bool(advanced) or tip_speed > TIP_MOVING, "tip_speed_mm_s": 1000.0 * tip_speed}


SCREW = Screw()
REC = Record()


@lk.on_reset
def _reset(tick):
    SCREW.reset()
    REC.reset()
    lk.set_rest("boom", "slide_p", 0.0)
    lk.set_rest("boom", "slide_n", 0.0)


@lk.on_step
def _track(tick):
    b, cr = tick.states["boom"], tick.states["crank"]
    qp, qn = b.q["slide_p"], b.q["slide_n"]
    latched = lk.braked("boom", "hinge")
    if not latched and min(qp, qn) >= Q_FULL - LATCH_Q and abs(b.deg("hinge")) <= LATCH_DEG:
        lk.brake("boom", "hinge", True)  # the root latch pawl drops in: the hinge is held where it is
        SCREW.locked = True
        latched = True
    s0 = SCREW.s
    s = SCREW.step(cr.q["turn"], min(qp, qn), max(qp, qn))
    lk.set_rest("boom", "slide_p", s)
    lk.set_rest("boom", "slide_n", s)
    m = measure(b, cr, tick.tools, s, latched, advanced=abs(s - s0) > 1e-9)
    REC.update(m)
    if REC.n % 5 == 0:  # diagnostics every 0.25 s
        REC.trace.append((round(tick.t, 2), round(m["q_mm"], 2), round(s * 1000, 2), round(m["strain_mm"], 2),
                          round(m["ext"], 3), round(m["hinge_deg"], 2), round(m["sling_deg"], 1), round(m["clear_mm"], 1),
                          round(math.degrees(cr.q["turn"]), 1), int(latched),
                          round(1000 * max(b.joint_error(j) for j in LOOPS), 3),
                          [round(1000 * v) for v in m["clear"].values()], round(m["touch_mm"], 1),
                          int(m["on_sling"] and m["moving"])))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo -----------------------------------------------------------------------------------------------------
# linkdemo.py runs demo_script(ctx). It reads only what a camera could measure: the two root sliders' places on the
# rail, the root hinge's angle, the sling's tilt and its grip sleeve's position (from link poses), the crank's axis,
# and the robot's own grippers.
# It never reads the screw, the latch, the clearance record or any joint readout.

DEMO_LOG: list = []
T_LEAD = 0.10  # s of velocity lead on the carry target (command latency plus arm lag)
K_LEVEL = 0.3  # integral gain of the height correction, per control step
K_SLING = 0.3  # integral gain of the along-boom correction from the sling's seen tilt, per control step
CRANK_TURNS = Q_FULL / LEAD + 1.0 / 6.0
CRANK_S = 14.0


def _seen_q(seen) -> float:
    return 0.5 * (seen.angle_deg("boom", "slide_p") + seen.angle_deg("boom", "slide_n"))


def _clamp_z(z: float) -> float:
    """The height correction stays within -20 mm .. +30 mm of the level point (a lost grip cannot wind it up)."""
    return min(max(z, -0.020), 0.030)


def _seen_sling(seen) -> float:
    return sling_deg(seen.point("boom", "bail", (0.0, 0.0, -BAIL_L / 2)), seen.point("boom", "bail", (0.0, 0.0, BAIL_L / 2)))


def _level_tcp(q: float) -> np.ndarray:
    """Where the right TCP sits on the sling's grip sleeve of a level boom at slider travel q."""
    return ORIGIN + np.array([0.0, -u_tip(q), GRASP_Z])


def grasp_handle(ctx):
    """Left hand straight down on the crank handle, pads across it 10 mm above the crank arm."""
    import linkdemo as ld

    arm = ctx.left
    p = ctx.observe().point("crank", "handle", (0.0, 0.0, -0.007 + PAD_TIP))
    yield from arm.move(p + [0, 0, 0.06], ld.down(0.0), width=0.040, duration=1.8)
    yield from arm.move(p, ld.down(0.0), duration=1.0)
    yield from arm.grip(0.008, 0.5)


def crank(ctx, turns: float, duration: float):
    """Turn the crank ``turns`` (+ = deploy) by circling the held handle about the crank axis."""
    hinge, axis = ctx.observe().axis("crank", "turn")
    yield from ctx.left.arc(hinge, axis, 360.0 * turns, follow=0.0, duration=duration)


def grasp_sleeve(ctx):
    """Right hand straight down on the sling's grip sleeve, pads closing along the boom (y) across it."""
    import linkdemo as ld

    arm = ctx.right
    p = ctx.observe().point("boom", "sleeve") + np.array([0.0, 0.0, GRASP_Z - GRIP_Z])
    yield from arm.move(p + [0, 0, 0.030], ld.down(0.0), width=0.040, duration=2.0)
    yield from arm.move(p, ld.down(0.0), duration=1.0)
    yield from arm.grip(0.006, 0.6)


class Carry:
    """Streams the right hand's target every control step: the level tip position for the sliders' seen travel (plus
    T_LEAD of their seen speed), raised or lowered by an integral of the hinge angle seen from the link poses."""

    def __init__(self, ctx):
        self.ctx, self.zc, self.yc, self.q0, self.v = ctx, 0.0, 0.0, None, 0.0

    def target(self, follow: bool = True, hold_at: float | None = None) -> np.ndarray:
        seen = self.ctx.observe()
        q = _seen_q(seen)
        if self.q0 is not None:
            self.v = 0.6 * self.v + 0.4 * (q - self.q0) * 20.0
        self.q0 = q
        th = seen.angle_deg("boom", "hinge")
        self.zc = _clamp_z(self.zc + K_LEVEL * u_tip(q) * math.sin(math.radians(th)))  # droop (+) -> lift
        qt = min(max(q + (self.v * T_LEAD if follow else 0.0), 0.0), Q_FULL)
        if hold_at is not None:  # cheat: keep the hand where the tip was, height still corrected
            return _level_tcp(hold_at) + [0.0, 0.0, self.zc]
        tilt = _seen_sling(seen)  # + : the hand is behind the tip
        self.yc = min(max(self.yc - K_SLING * BAIL_L * math.sin(math.radians(tilt)), -0.015), 0.015)
        return _level_tcp(qt) + [0.0, self.yc, self.zc]

    def run(self, duration: float, follow: bool = True, hold_at: float | None = None):
        arm = self.ctx.right
        yield from arm.follow(lambda s: (self.target(follow, hold_at), arm.rot, arm.width), duration)


def steady(ctx, carry: Carry, duration: float):
    """Cheat: carry the tip level but move the hand out at a steady speed over the crank's duration, not at the
    scissor's own pace."""
    arm = ctx.right

    def at(s):
        seen = ctx.observe()
        th = seen.angle_deg("boom", "hinge")
        carry.zc = _clamp_z(carry.zc + K_LEVEL * u_tip(_seen_q(seen)) * math.sin(math.radians(th)))
        return ORIGIN + np.array([0.0, -(U_S + s * (U_D - U_S)), GRASP_Z + carry.zc]), arm.rot, arm.width

    yield from arm.follow(at, duration)


def steady_line(ctx, carry: Carry, p0, p1, duration: float):
    """Cheat helper: move the right hand from p0 to p1 at a steady speed, its height still corrected from the seen
    hinge angle."""
    arm = ctx.right
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)

    def at(s):
        seen = ctx.observe()
        th = seen.angle_deg("boom", "hinge")
        carry.zc = _clamp_z(carry.zc + K_LEVEL * u_tip(_seen_q(seen)) * math.sin(math.radians(th)))
        return p0 + s * (p1 - p0) + np.array([0.0, 0.0, carry.zc]), arm.rot, arm.width

    yield from arm.follow(at, duration)


def _from_beyond(deg: float) -> tuple:
    """Approach from beyond the tip (along +y, toward the root), tilted ``deg`` down; pads closing across the boom."""
    import linkdemo as ld

    a = math.radians(deg)
    app = np.array([0.0, math.cos(a), -math.sin(a)])
    return app, ld.grip_frame(app, (1.0, 0.0, 0.0))


def grasp_canister(ctx):
    """Cheat: the right hand grips the white canister itself from beyond the tip, pads across it at mid-height."""
    arm = ctx.right
    app, rot = _from_beyond(35.0)
    p = ctx.observe().point("boom", "tip", (0.0, 0.0, CAN_BOTTOM + 0.015))
    yield from arm.move(p - 0.06 * app, rot, width=0.050, duration=2.2)
    yield from arm.move(p, duration=1.0)
    yield from arm.grip(0.028, 0.6)
    return np.array([0.0, -(U_D - U_S), 0.0])


def grasp_bar(ctx):
    """Cheat: the right hand pinches the half bar d (next to the apex) at its middle, straight down, pads across it."""
    import linkdemo as ld

    arm = ctx.right
    m = ctx.observe().pose("boom", "d")
    rot = ld.grip_frame((0.0, 0.0, -1.0), m[:3, 1], ref=ld.down(0.0))
    p = m[:3, 3] + np.array([0.0, 0.0, -BAR_T / 2 + 0.001 + PAD_TIP])  # pad tips 1 mm above d's underside
    yield from arm.move(p + [0.0, 0.0, 0.030], rot, width=0.030, duration=2.2)
    yield from arm.move(p, duration=1.0)
    yield from arm.grip(0.006, 0.6)
    c_s, c_d = L_BAR * math.cos(PHI_S), L_BAR * math.cos(PHI_D)
    return np.array([(W_D - W_S) / 4.0, -2.25 * (c_d - c_s), 0.0])  # where d's middle ends up


def grasp_prop(ctx):
    """Cheat: the right hand's closed fingertips slide in from beyond the tip under the canister and lift it 5 mm."""
    arm = ctx.right
    app, rot = _from_beyond(30.0)
    up = np.array([0.0, math.sin(math.radians(30.0)), math.cos(math.radians(30.0))])  # across the fingers, upward
    rest = ctx.observe().point("boom", "tip", (0.0, 0.0, CAN_BOTTOM))
    p = rest - 0.010 * app - 0.0077 * up - np.array([0.0, 0.0, 0.003])  # fingertips' top edge 3 mm under the canister
    yield from arm.move(p - np.array([0.0, 0.06, 0.0]), rot, width=0.0, duration=2.2)
    yield from arm.move(p, duration=1.2)
    yield from arm.move(p + np.array([0.0, 0.0, 0.008]), duration=0.8)
    return np.array([0.0, -(U_D - U_S), 0.0])


def lift_level(ctx, carry: Carry):
    """Lift the held tip to level: move to the level point, then let the integral settle the seen hinge angle."""
    q = _seen_q(ctx.observe())
    yield from ctx.right.move(_level_tcp(q), duration=1.2)
    yield from carry.run(1.0, follow=False)
    DEMO_LOG.append({"lifted_hinge_deg": round(ctx.observe().angle_deg("boom", "hinge"), 2), "t": round(ctx.t, 2)})


def let_go(arm, up: float = 0.08):
    yield from arm.grip(0.040, 0.5)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, up]), duration=1.0)


def demo_script(ctx):
    v = VARIANT
    if v == "nothing":
        yield from ctx.wait(55.0)
        return
    carry = Carry(ctx)
    if v == "crank_only":  # crank to full length; the right hand stays up out of the way
        yield from grasp_handle(ctx)
        yield from crank(ctx, CRANK_TURNS, CRANK_S)
        yield from let_go(ctx.left)
        yield from ctx.wait(4.0)
        return
    if v in ("grip_canister", "grip_bar", "prop_under"):
        grasp = {"grip_canister": grasp_canister, "grip_bar": grasp_bar, "prop_under": grasp_prop}[v]
        out = []

        def take():
            out.append((yield from grasp(ctx)))

        yield from ctx.both(grasp_handle(ctx), take())
        p0 = ctx.right.pos.copy()
        yield from steady_line(ctx, carry, p0, p0, 1.0)  # level it first
        yield from ctx.both(crank(ctx, CRANK_TURNS, CRANK_S), steady_line(ctx, carry, p0, p0 + out[0], CRANK_S))
        yield from steady_line(ctx, carry, p0 + out[0], p0 + out[0], 1.0)
        yield from ctx.both(let_go(ctx.right, 0.05), let_go(ctx.left))
        yield from ctx.wait(3.0)
        return
    yield from ctx.both(grasp_handle(ctx), grasp_sleeve(ctx))
    yield from lift_level(ctx, carry)
    if v == "pull_tip":  # pull the tip out along the boom by hand, never cranking
        yield from ctx.right.move(_level_tcp(Q_FULL) + [0.0, 0.0, carry.zc], duration=CRANK_S)
        yield from ctx.wait(1.0)
    elif v == "hold_still":  # carry the tip level but keep the hand where the tip started while cranking
        q0 = _seen_q(ctx.observe())
        yield from ctx.both(crank(ctx, CRANK_TURNS, CRANK_S), carry.run(CRANK_S + 1.0, hold_at=q0))
    elif v == "steady_pace":  # carry the tip level, moving out at a steady speed while cranking
        yield from ctx.both(crank(ctx, CRANK_TURNS, CRANK_S), steady(ctx, carry, CRANK_S))
        yield from carry.run(1.0, follow=False)
    elif v == "lift_after":  # let go of the tip, crank to full length, then lift the tip back up to level
        yield from let_go(ctx.right)
        yield from crank(ctx, CRANK_TURNS, CRANK_S)
        yield from grasp_sleeve(ctx)
        yield from lift_level(ctx, carry)
        yield from carry.run(1.5, follow=False)
    else:  # the demo: crank and carry together
        yield from ctx.both(crank(ctx, CRANK_TURNS, CRANK_S), carry.run(CRANK_S + 1.0))
        DEMO_LOG.append({"deployed_seen_mm": round(1000 * _seen_q(ctx.observe()), 2),
                         "hinge_seen_deg": round(ctx.observe().angle_deg("boom", "hinge"), 2), "t": round(ctx.t, 2)})
    yield from ctx.both(let_go(ctx.right, 0.05), let_go(ctx.left))  # right: 50 mm keeps it inside its reach
    yield from ctx.wait(3.0)


def demo_report() -> dict:
    return {"ladder": ladder(REC)[1], "latch": REC.latch, "low_event": REC.low_event, "tilt_event": REC.tilt_event,
            "sling_event": REC.sling_event, "touch_event": REC.touch_event,
            "without_touch_gate": ladder(REC, skip=("touch",)), "demo": DEMO_LOG, "peaks": REC.H.peaks, "lows": REC.H.lows, "screw_slipped_mm": 1000 * SCREW.slipped,
            "trace_cols": "t q_mm s_mm strain_mm ext hinge_deg sling_deg clear_mm crank_deg latched loop_err_mm pads_mm "
                          "touch_mm carrying",
            "trace": REC.trace}
