# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Move the flap lever of a bench flap-quadrant trainer through its gated slot to FULL (flap-system operational check).

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475. Trainer frame L: origin on the table top under the lever pivot, at world
(0.33, -0.12), turned 180 deg, so L x = world -x (from UP toward FULL, toward the robot), L y = world -y (along the
pivot axis), z up.

- A grey free-standing quadrant trainer (0.50 kg with its lever) on rubber feet: a base plate with a hold-down handle
  tab on its left end, a quadrant housing, and a steel gate plate arched over the lever pivot (40 mm above the table).
- The white flap lever comes up through a slot in the gate plate. Its knob slides 12 mm along the lever against a
  6 N preloaded spring and carries a steel gate pin across the slot. The pin sits in detent notches (UP, 5, 15, 30,
  FULL, 10 deg apart) between 6 mm teeth on the gate plate. The end stop past FULL is painted red.
- The slot has two lanes: the inboard lane runs from UP to the 15 detent and ends there in the gate wall (the
  go-around gate); the outboard lane, 16 mm to the side, runs from 15 to FULL. A sideways spring holds the lever in
  the inboard lane.

To reach FULL the lever must be lifted (pin clear of the teeth), carried to 15, jogged 16 mm sideways round the gate
wall, carried to FULL and lowered into the notch. The trainer weighs less than the lift spring's preload, so the
knob cannot be lifted unless the trainer is held down: one arm holds it, the other works the lever.

Goal: the pin seated in the FULL notch, the trainer never moved more than 10 mm, both grippers let go.
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
EPS = 1e-6

# -- layout (trainer frame L, metres and degrees) ----------------------------------------------------------------

ORIGIN = (0.33, -0.12)  # world x, y of L's origin (under the pivot)
YAW = 180.0
HP = 0.040  # pivot height above the table
GREY = (0.55, 0.57, 0.60)
STEEL = (0.25, 0.26, 0.28)
WHITE = (0.95, 0.95, 0.93)
SHAFT_WHITE = (0.92, 0.92, 0.90)
RED = (0.80, 0.12, 0.10)  # FULL end stop

# radii from the pivot along the lever
R_PLATE = (0.070, 0.090)  # gate plate (slot depth 20 mm)
R_TOOTH = (0.0885, 0.099)  # detent teeth: top 6 mm above the seated pin's bottom
R_STOP = (0.089, 0.115)  # end stops: above the pin at full lift
R_PIN = 0.097  # gate-pin axis at lift 0 (its bottom 3 mm above the gate plate)
PIN_R = 0.004
PIN_HALF = 0.015  # the pin is 30 mm long, across the slot
R_KNOB = 0.140  # knob centre
SHAFT_R = 0.005
R_SLOT = 0.080  # mid-depth of the slot (lane geometry is given there)

# angles (pitch theta, deg, + from UP toward FULL)
NOTCHES = {"UP": 0.0, "5": 10.0, "15": 20.0, "30": 30.0, "FULL": 40.0}
FULL = NOTCHES["FULL"]
TEETH = ((3.0, 7.0), (13.0, 17.0), (23.0, 27.0), (33.0, 37.0))
STOPS = ((-9.0, -3.0), (43.0, 49.0))
PLATE_TH = (-12.0, 50.0)
PLATE_Y = 0.040
# slot strips across the gate plate (y, metres) and the theta ranges (deg) where each strip is cut open
Y_EDGES = (-0.040, -0.015, -0.001, 0.001, 0.015, 0.040)
OPEN = {  # strip index -> open theta intervals
    0: (),
    1: ((14.8, 45.2),),  # outboard lane (15 to FULL)
    2: ((14.8, 25.2),),  # the jog at the 15 detent
    3: ((-5.3, 25.2),),  # inboard lane (UP to 15); its end at 25.2 deg is the gate wall
    4: (),
}
# The lever tilts sideways about the pivot, so its shaft sits further out at the top of the slot than at the bottom.
PHI_A = math.degrees(math.asin(0.1))  # 5.74 deg: design tilt = inboard lane (shaft 1 mm off both lane walls)
LANE_B_REST = math.degrees(math.asin(-0.006 / R_PLATE[0])) - PHI_A  # -10.66: outboard lane, shaft on the gate wall
LANE_B_OUT = math.degrees(math.asin(-0.010 / R_PLATE[1])) - PHI_A  # -12.12: shaft on the outboard lane's outer wall
LAT_LIMITS = (-14.0, 0.0)  # lateral joint, 0 = inboard lane against its stop
LIFT_MAX = 0.012
LIFT_K = 60.0  # N/m
LIFT_PRELOAD = 6.0  # N at lift 0
M_BASE, M_YOKE, M_LEVER, M_KNOB = 0.42, 0.01, 0.03, 0.04
HANDLE = (-0.025, -0.100)  # hold-down tab centre (L x, y)
HANDLE_TOP = 0.054


def _deg_seg(a0: float, a1: float, max_seg: float = 5.7, end_seg: float = 1.0):
    """Split a0..a1 (deg) into segments of at most max_seg, with end_seg-wide segments at both ends so the band's
    end faces are within end_seg/2 of radial (slot ends and gate wall)."""
    if a1 - a0 <= 2 * end_seg + 1e-9:
        return [(a0, a1)]
    mid = (a0 + end_seg, a1 - end_seg)
    n = max(1, math.ceil((mid[1] - mid[0]) / max_seg - 1e-9))
    inner = [(mid[0] + (mid[1] - mid[0]) * k / n, mid[0] + (mid[1] - mid[0]) * (k + 1) / n) for k in range(n)]
    return [(a0, mid[0])] + inner + [(mid[1], a1)]


def arc_boxes(th0, th1, r0, r1, y0, y1, color=None) -> list:
    """Chord boxes approximating an arc band about the pivot axis (L y) between theta th0..th1 (deg), radii r0..r1
    and y0..y1. Each box's outer face is tangent at the segment's middle (at most 0.15 mm proud at its ends); its
    tangential size is the chord at mid-depth."""
    out = []
    for a, b in _deg_seg(th0, th1):
        tm = math.radians((a + b) / 2)
        rc = (r0 + r1) / 2
        chord = 2 * rc * math.sin(math.radians(b - a) / 2)
        out.append(lk.box((chord, y1 - y0, r1 - r0), (rc * math.sin(tm), (y0 + y1) / 2, HP + rc * math.cos(tm)),
                          rpy_deg=(0.0, math.degrees(tm), 0.0), color=color))
    return out


def _merge(rects):
    """Merge (th0, th1, y0, y1) rectangles that share a theta interval and touch in y."""
    rects = sorted(rects)
    out = []
    for r in rects:
        if out and out[-1][:2] == r[:2] and abs(out[-1][3] - r[2]) < 1e-9:
            out[-1] = (r[0], r[1], out[-1][2], r[3])
        else:
            out.append(r)
    return out


def plate_rects():
    """Solid parts of the gate plate: (th0, th1, y0, y1)."""
    rects = []
    for i in range(len(Y_EDGES) - 1):
        cuts = sorted(OPEN[i])
        a = PLATE_TH[0]
        for c0, c1 in cuts:
            if c0 > a:
                rects.append((a, c0, Y_EDGES[i], Y_EDGES[i + 1]))
            a = max(a, c1)
        if a < PLATE_TH[1]:
            rects.append((a, PLATE_TH[1], Y_EDGES[i], Y_EDGES[i + 1]))
    return _merge(rects)


def tooth_rects():
    """Teeth on the solid parts beside the slot: a strip carries a tooth unless the slot is open in it anywhere
    along the tooth."""
    rects = []
    for t0, t1 in TEETH:
        for i in range(len(Y_EDGES) - 1):
            if not any(c0 < t1 and c1 > t0 for c0, c1 in OPEN[i]):
                rects.append((t0, t1, Y_EDGES[i], Y_EDGES[i + 1]))
    return _merge(rects)


def base_shapes() -> list:
    s = [
        lk.box((0.170, 0.230, 0.004), (0.025, -0.045, 0.002)),  # base plate
        lk.box((0.040, 0.006, 0.050), (0.0, 0.031, 0.029)),  # pivot cheeks
        lk.box((0.040, 0.006, 0.050), (0.0, -0.031, 0.029)),
        lk.box((0.110, 0.006, 0.092), (0.025, 0.046, 0.050)),  # housing side plates
        lk.box((0.110, 0.006, 0.092), (0.025, -0.046, 0.050)),
        lk.box((0.040, 0.010, HANDLE_TOP - 0.004), (HANDLE[0], HANDLE[1], 0.004 + (HANDLE_TOP - 0.004) / 2)),  # tab
    ]
    for th0, th1, y0, y1 in plate_rects():
        s += arc_boxes(th0, th1, *R_PLATE, y0, y1, STEEL)
    for th0, th1, y0, y1 in tooth_rects():
        s += arc_boxes(th0, th1, *R_TOOTH, y0, y1, STEEL)
    for (th0, th1), color in zip(STOPS, (STEEL, RED)):  # the end stop just past the FULL notch is painted red
        s += arc_boxes(th0, th1, *R_STOP, -PLATE_Y, PLATE_Y, color)
    return s


U_A = (0.0, math.sin(math.radians(PHI_A)), math.cos(math.radians(PHI_A)))  # lever axis at the design pose

_t = lk.Tree("quadrant", pos=(ORIGIN[0], ORIGIN[1], TOP), yaw_deg=YAW, color=GREY)
_t.link("base", base_shapes(), mass=M_BASE, com=(0.020, -0.010, 0.035), friction=(1.0, 0.9))
_t.link("yoke", [lk.box((0.008, 0.008, 0.008))], mass=M_YOKE, at=(0.0, 0.0, HP), color=STEEL)
_t.link("lever", [lk.cylinder(0.008, 0.016, axis="Y"), lk.cylinder(SHAFT_R, 0.078, (0.0, 0.0, 0.049))],
        mass=M_LEVER, at=(0.0, 0.0, HP), rpy_deg=(-PHI_A, 0.0, 0.0), color=SHAFT_WHITE, friction=(0.25, 0.2),
        friction_combine="min")
_t.link("knob", [lk.cylinder(PIN_R, 2 * PIN_HALF, (0.0, 0.0, R_PIN), axis="Y", color=STEEL),
                 lk.cylinder(0.003, 0.0175, (0.0, 0.0, 0.09275), color=STEEL),
                 lk.box((0.012, 0.012, 0.0235), (0.0, 0.0, 0.11325)),
                 lk.box((0.030, 0.022, 0.030), (0.0, 0.0, R_KNOB))],
        mass=M_KNOB, at=(0.0, 0.0, HP), rpy_deg=(-PHI_A, 0.0, 0.0), color=WHITE, friction=(0.25, 0.2),
        friction_combine="min")
_t.revolute("pitch", "base", "yoke", anchor=(0.0, 0.0, HP), axis=(0.0, 1.0, 0.0), limits_deg=(-6.0, 46.0),
            damping=0.02, armature=1e-4)
_t.revolute("lateral", "yoke", "lever", anchor=(0.0, 0.0, HP), axis=(-1.0, 0.0, 0.0), limits_deg=LAT_LIMITS,
            rest_deg=4.0, stiffness=0.6, damping=0.01, armature=1e-4)
_t.prismatic("lift", "lever", "knob", anchor=(0.0, 0.0, HP), axis=U_A, limits=(0.0, LIFT_MAX),
             rest=-LIFT_PRELOAD / LIFT_K, stiffness=LIFT_K, damping=3.0)
QUADRANT = _t.build()

OBJECTS = [QUADRANT]

# -- measurement and whole-run record ----------------------------------------------------------------------------

MOVE_GATE = 0.010  # trainer displacement, any time
SEAT_LIFT = 0.001  # pin seated: lift under 1 mm
FULL_TOL = 1.0  # deg
PAST_GATE = 25.0  # deg: a seated pin this far is in the 30 or FULL notch, past the go-around gate
STILL_V, STILL_W, STILL_S = 0.003, math.radians(3.0), 1.0
CLEAR_M, TCP_STILL, RELEASE_S = 0.030, 0.005, 1.0
BASE_PTS = np.array([[-0.06, -0.16, 0.0], [0.11, -0.16, 0.0], [-0.06, 0.07, 0.0], [0.11, 0.07, 0.0],
                     [R_PLATE[1] * math.sin(math.radians(19)), 0.035, HP + R_PLATE[1] * math.cos(math.radians(19))],
                     [HANDLE[0], HANDLE[1], HANDLE_TOP]])
BASE0 = QUADRANT.link_poses()["base"]  # placed pose (design pose; object_jitter=0)


def base_points(B) -> np.ndarray:
    return (np.asarray(B, dtype=float)[:3, :3] @ BASE_PTS.T).T + np.asarray(B, dtype=float)[:3, 3]


PTS0 = base_points(BASE0)


def measure(st, tools) -> dict:
    """One sample from the simulator (scorer side: joint readouts allowed). Lengths in metres, angles in degrees."""
    B = np.asarray(st.poses["base"], dtype=float)
    disp = float(np.max(np.linalg.norm(base_points(B) - PTS0, axis=1)))
    clear = {side: lk.clearance(tool, st) for side, tool in tools.items()}
    return {
        "pitch": float(st.deg("pitch")),
        "lat": float(st.deg("lateral")),
        "lift": float(st.q["lift"]),
        "disp": disp,
        "v": float(np.linalg.norm(st.lin_vels.get("knob", np.zeros(3)))),
        "w": float(np.linalg.norm(st.ang_vels.get("knob", np.zeros(3)))),
        "clear": clear,
        "tcp_speed": {s: float(t.speed) for s, t in tools.items()},
    }


def pose_rates(A_prev, A, dt: float) -> tuple[float, float]:
    """Speed (m/s) and spin (rad/s) of a body from its pose change over ``dt``."""
    if dt <= 0.0:
        return 0.0, 0.0
    v = float(np.linalg.norm(A[:3, 3] - A_prev[:3, 3])) / dt
    R = A_prev[:3, :3].T @ A[:3, :3]
    sin = 0.5 * float(np.linalg.norm((R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1])))
    return v, math.atan2(sin, (float(np.trace(R)) - 1.0) / 2.0) / dt


def seated(m) -> bool:
    return m["lift"] < SEAT_LIFT


def at_full(m) -> bool:
    return seated(m) and abs(m["pitch"] - FULL) <= FULL_TOL


def past_gate(m) -> bool:
    return seated(m) and m["pitch"] >= PAST_GATE


class Record:
    """Whole-run record: a linkkit History updated every control step and cleared only by the simulator reset, plus
    the latest sample. Pure Python."""

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
        H.peak("disp", m["disp"])
        H.ever("moved", m["disp"] > MOVE_GATE)
        H.peak("pitch", m["pitch"])
        H.peak("lift", m["lift"])
        H.low("lat", m["lat"])
        H.held("still", m["v"] < STILL_V and m["w"] < STILL_W)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Score and the reason. The gate first (any time in the run), then the rungs from the top."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "moved" in H.evers:
        return 0.0, (f"gate: the trainer moved {1000 * H.peaks['disp']:.1f} mm (limit {1000 * MOVE_GATE:.0f}) "
                     f"at {H.evers['moved']:.2f} s")
    if at_full(m):
        still = H.runs.get("still", 0.0) >= STILL_S - EPS
        released = H.runs.get("released", 0.0) >= RELEASE_S - EPS
        if still and released:
            return 1.0, "pin seated in the FULL notch, lever still, both grippers let go"
        if still:
            return 0.75, "pin seated in the FULL notch and the lever still; not yet: both grippers 30 mm clear 1 s"
        return 0.5, "pin seated in the FULL notch; not yet: lever still 1 s"
    if past_gate(m):
        return 0.25, f"pin seated past the go-around gate at {m['pitch']:.1f} deg, not in FULL"
    if seated(m):
        return 0.0, f"pin seated at {m['pitch']:.1f} deg, short of the go-around gate"
    return 0.0, f"pin not seated: lift {1000 * m['lift']:.1f} mm at {m['pitch']:.1f} deg"


REC = Record()
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    st = tick.states["quadrant"]
    m = measure(st, tick.tools)
    K = np.array(st.poses["knob"], dtype=float)
    if _PREV:
        m["v"], m["w"] = pose_rates(_PREV["K"], K, tick.t - _PREV["t"])
    _PREV.update(K=K, t=tick.t)
    REC.update(m)
    if REC.n % 5 == 0:  # diagnostics every 0.25 s
        REC.trace.append((round(tick.t, 2), round(m["pitch"], 2), round(m["lat"], 2), round(1000 * m["lift"], 2),
                          round(1000 * m["disp"], 2), [round(1000 * m["clear"][s]) for s in ("left", "right")]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


DEMO_LOG: dict = {"servo": []}


def demo_report() -> dict:
    H = REC.H
    return {"ladder": ladder(REC), "variant": VARIANT, "evers": H.evers, "peaks": H.peaks, "lows": H.lows,
            "best_runs": H.best_runs, "runs": H.runs, "last": REC.last, "trace": REC.trace, "log": DEMO_LOG}


# -- scripted demo ---------------------------------------------------------------------------------------------------
# linkdemo.py runs demo_script(ctx). It reads only what a camera could measure: the poses of the trainer's base, lever
# shaft and knob (pitch, sideways tilt and knob lift follow from them) and the robot's own grippers. It never reads
# joint values, solver velocities or the record.

VARIANTS = ("", "nothing", "push_only", "lift_straight", "no_hold", "leave_lifted")
GRIP_TILT = 15.0  # deg: the right hand's approach leans this much past the lever axis toward FULL (reach)
GRIP_OPEN = 0.060
LIFT_T = 0.0095  # carry lift (pin bottom 9.5 mm up; teeth 6 mm; the pin's ends tilt up to 1.4 mm with the lever)
JOG = -11.4  # outboard lane centre (lateral deg)
HOLD_PUSH = 0.030  # the left hand pushes the gripped handle 30 mm down (arm compliance: a few N)
LIFT_BIAS0 = 0.015


def lever_from_poses(B, L, K) -> tuple[float, float, float]:
    """Pitch (deg), lateral (deg, 0 = inboard lane) and lift (m) from the base, lever and knob link poses."""
    B, L, K = (np.asarray(x, dtype=float) for x in (B, L, K))
    u_w = L[:3, :3] @ np.array([0.0, 0.0, 1.0])
    u = B[:3, :3].T @ u_w
    th = math.degrees(math.atan2(u[0], u[2]))
    phi = math.degrees(math.asin(max(-1.0, min(1.0, float(u[1]))))) - PHI_A
    lift = float((K[:3, 3] - L[:3, 3]) @ u_w)
    return th, phi, lift


def _pose(p, r) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, p
    return m


def demo_script(ctx):
    import linkdemo as ld

    v = ctx.variant or VARIANT
    if v not in VARIANTS:
        raise ValueError(f"unknown variant {v!r}; one of {VARIANTS}")
    flags = {"held": v in ("no_hold",), "done": False}
    log = DEMO_LOG
    log["variant"] = v
    yield from ctx.wait(0.5)
    if v == "nothing":
        yield from ctx.wait(4.0)
        return

    def seen_lever():
        s = ctx.observe()
        return s, lever_from_poses(s.pose("quadrant", "base"), s.pose("quadrant", "lever"), s.pose("quadrant", "knob"))

    def left():
        arm = ctx.left
        if v == "no_hold":
            return
        s = ctx.observe()
        B = s.pose("quadrant", "base")
        tab = B[:3, :3] @ np.array([HANDLE[0], HANDLE[1], 0.042]) + B[:3, 3]
        r = ld.down(0.0)  # pads closing along world y, across the 10 mm tab
        yield from arm.move(tab + [0.0, 0.0, 0.045], r, width=0.050, duration=2.0)
        yield from arm.move(tab, r, duration=1.5)
        yield from arm.grip(0.0, 0.8)
        yield from arm.wait(0.3)
        yield from arm.move(tab - [0.0, 0.0, HOLD_PUSH], r, duration=1.0)
        yield from arm.wait(0.3)
        flags["held"] = True
        log["held_t"] = round(ctx.t, 2) if hasattr(ctx, "t") else None
        while not flags["done"]:
            yield
        yield from arm.move(tab, r, duration=0.8)  # ease off the push before letting go
        yield from arm.move(tab, r, width=0.050, duration=0.6)
        yield from arm.wait(0.3)
        yield from arm.move(tab + [0.0, 0.0, 0.040], r, duration=1.0)  # fingertips above the tab
        yield from arm.move(tab + [0.0, 0.060, 0.080], r, duration=1.2)  # away from the housing

    def right():
        arm = ctx.right
        s, (th, ph, lf) = seen_lever()
        B = s.pose("quadrant", "base")
        K = s.pose("quadrant", "knob")
        Lw = s.pose("quadrant", "lever")
        centre = K[:3, :3] @ np.array([0.0, 0.0, R_KNOB]) + K[:3, 3]
        u = Lw[:3, :3] @ np.array([0.0, 0.0, 1.0])
        axp = B[:3, :3] @ np.array([0.0, 1.0, 0.0])
        app = ld.rot_about(axp, GRIP_TILT) @ (-u)
        R = ld.grip_frame(app, axp)
        log["start"] = [round(th, 2), round(ph, 2), round(1000 * lf, 2)]
        yield from arm.move(centre - app * 0.07, R, width=GRIP_OPEN, duration=2.5)
        yield from arm.move(centre, R, duration=1.5)
        yield from arm.grip(0.0, 0.8)
        yield from arm.wait(0.4)
        s, (th, ph, lf) = seen_lever()
        K = s.pose("quadrant", "knob")
        B = s.pose("quadrant", "base")
        rel = np.linalg.inv(K) @ _pose(arm.pos, arm.rot)
        bias = [0.0, 0.0, LIFT_BIAS0]
        last = [th, ph, lf]  # commanded (biased) joint values

        def cmd(q):
            Kt = QUADRANT.link_poses({"pitch": math.radians(q[0]), "lateral": math.radians(q[1]), "lift": q[2]},
                                     frame=B)["knob"]
            T = Kt @ rel
            return T[:3, 3], T[:3, :3]

        def go(target, dur):
            q0 = list(last)
            q1 = [target[i] + bias[i] for i in range(3)]

            def at(x):
                e = ld.smooth(x)
                p, r = cmd([q0[i] + e * (q1[i] - q0[i]) for i in range(3)])
                return p, r, arm.width

            yield from arm.follow(at, dur)
            last[:] = q1

        def servo(target, tol=(0.4, 0.5, 0.0007), iters=7, dur=0.6, tag=""):
            for k in range(iters):
                yield from go(target, dur)
                yield from arm.wait(0.3)
                _, seen = seen_lever()
                err = [target[i] - seen[i] for i in range(3)]
                log["servo"].append([tag, k, [round(x, 2) for x in seen[:2]] + [round(1000 * seen[2], 2)],
                                     [round(bias[0], 2), round(bias[1], 2), round(1000 * bias[2], 1)]])
                if all(abs(err[i]) <= tol[i] for i in range(3)):
                    return True
                bias[0] += max(-5.0, min(5.0, err[0]))
                bias[1] += max(-5.0, min(5.0, err[1]))
                bias[2] += max(-0.015, min(0.015, err[2]))
            return False

        while not flags["held"]:
            yield
        if v == "push_only":
            bias[2] = 0.0
            yield from go([FULL, 0.0, 0.0], 3.0)
            yield from arm.wait(1.0)
        else:
            ok = yield from servo([th, 0.0, LIFT_T], tag="lift")
            if v == "lift_straight":
                yield from go([FULL, 0.0, LIFT_T], 3.0)
                yield from arm.wait(1.0)
            else:
                bias[2] -= 0.0025  # the arm is stiffer along the lever as it tips toward FULL
                yield from go([NOTCHES["15"], 0.0, LIFT_T], 2.5)
                yield from servo([NOTCHES["15"], 0.0, LIFT_T], tag="to15")
                yield from go([NOTCHES["15"], JOG, LIFT_T], 1.5)
                yield from servo([NOTCHES["15"], JOG, LIFT_T], tag="jog")
                bias[2] -= 0.003
                yield from go([FULL, JOG, LIFT_T], 2.5)
                yield from servo([FULL, JOG, LIFT_T], tag="toFULL")
                log["lift_ok"] = ok
                if v == "leave_lifted":
                    while True:
                        yield
            # lower the pin: lift command 3 mm below the seat, the arm's lift bias dropped
            _, seen = seen_lever()
            bias[2] = 0.0
            yield from go([seen[0], seen[1], -0.003], 1.5)
            yield from arm.wait(0.5)
            _, seen = seen_lever()
            log["dropped"] = [round(seen[0], 2), round(seen[1], 2), round(1000 * seen[2], 2)]
        yield from arm.move(arm.pos, arm.rot, width=GRIP_OPEN, duration=0.6)
        yield from arm.wait(0.3)
        yield from arm.retreat(0.07, 1.2)
        yield from arm.move(arm.pos + [0.0, 0.0, 0.05], arm.rot, duration=1.0)
        flags["done"] = True

    yield from ctx.both(left(), right())
    _, seen = seen_lever()
    log["final_seen"] = [round(seen[0], 2), round(seen[1], 2), round(1000 * seen[2], 2)]
    yield from ctx.wait(2.5)
