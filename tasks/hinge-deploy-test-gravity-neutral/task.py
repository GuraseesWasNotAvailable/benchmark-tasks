# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Run a spring-hinge deployment test with the hinge turned gravity-neutral.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475. Fixture frame F: origin on the table top at world (0.330, 0.000), axes parallel to
the world's while the fixture is as delivered (tilt 0).

- A single-sided tilt fixture (a worm-driven trunnion positioner) stands on the table. Its dark grey gearbox column
  is at the left end (F y 0.115 to 0.185). An aluminium tilt table (the cradle, 120 x 170 x 8 mm) is cantilevered
  from it on a trunnion shaft along y, 110 mm above the table. A black crank on top of the column drives the
  trunnion through a self-locking worm: one crank turn is 30 deg of tilt (crank anticlockwise seen from above tilts
  the table's top face away from the robot), from 0 to 95 deg.
- On the cradle lies a gold 1U-size deployable solar-panel dummy (100 x 120 x 4 mm, 150 g), stowed flat on top. Its
  spring hinge runs along x at the panel's left (+y) edge: a flight torsion spring (0.0191 N m/rad, rest at 150 deg:
  0.050 N m stowed, 0.020 N m at 90 deg) with a viscous damper, and a latch pawl that drops in at 89 deg against a
  90 deg hard stop.
- A red hold-down pin at the right (-y) end runs along y through a bracket on the cradle into a slotted boss on the
  panel's free edge. Pulling it 8 mm frees the panel (in the simulator: the hinge's holding brake is released).

As delivered the hinge axis is horizontal and the panel must swing upward: its weight gives 0.088 N m about the hinge
when stowed, more than the 0.050 N m flight spring, so a pulled pin leaves it stowed. Turning the crank three turns
first puts the hinge axis vertical; the panel then swings in a horizontal plane and the same spring carries it to the
latch.

Goal: the panel deployed by its own spring to its latch from a release made with the hinge axis within 10 deg of
vertical, nothing touching the panel after the release, and both grippers let go.
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

XF, YF = 0.330, 0.000  # fixture frame origin (on the table top)
AX_Z = 0.110  # trunnion axis above the table
COL_Y = 0.150  # gearbox column and crank axis
RATIO = 12.0  # worm: crank angle / tilt angle (30 deg of tilt per crank turn)
TILT_MAX = math.radians(95.0)
TILT_K, TILT_C = 50.0, 0.5  # trunnion drive (N m/rad, N m s/rad): the worm holds the table where the crank puts it

PANEL_MASS = 0.150
HINGE_Y, HINGE_Z = 0.055, 0.1145  # hinge line (F), along x, at the panel's bottom face
PANEL_C = (0.0, -0.005, 0.1165)  # board centre (F), stowed; 60 mm from the hinge line
SPRING_K = 0.0191  # N m/rad
SPRING_REST_DEG = 150.0  # preloaded: 0.050 N m at 0 deg, 0.020 N m at 90 deg
DEPLOY_C = 0.008  # viscous damper, N m s/rad
DEPLOY_F = 0.003  # hinge friction, N m
LATCH_DEG = 89.0  # the pawl drops in here...
PAWL_DEG = 88.5  # ...and then holds the panel between 88.5 deg and the 90 deg stop
PIN_Z = 0.1225  # pin line (F, stowed)
PIN_RELEASE_M = 0.008  # 6 mm engaged in the boss: 8 mm out frees the panel
KNOB_C = (0.0, -0.150, PIN_Z)  # pull knob centre (F, pin home), 47 mm past the cradle's end
CRANK_R = 0.040
HANDLE_C = (-CRANK_R, COL_Y, 0.170)  # handle sleeve centre (F, crank at 0: arm toward the robot)

VERT_TOL_DEG = 10.0  # hinge axis within 10 deg of vertical
CONTACT_M = 0.005  # a pad point this close to the panel after the release counts as touching it
CLEAR_M = 0.030
TCP_STILL = 0.005
RELEASE_S = 1.0
END_DEG = 88.0
STILL_MM_S = 3.0
STILL_DEG_S = 3.0
EPS = 1e-6
FIX_MOVING = ("cradle", "panel", "pin")
CRANK_MOVING = ("crank", "handle")

GOLD = (0.85, 0.68, 0.22)
ALU = (0.76, 0.77, 0.79)
DARK = (0.30, 0.31, 0.33)
IRON = (0.45, 0.46, 0.48)
BLACK = (0.10, 0.10, 0.11)
RED = (0.85, 0.08, 0.08)
STEEL = (0.62, 0.63, 0.66)

# -- the tilt fixture with the panel and the hold-down pin (one articulation, fixed to the table) -------------------
_t = lk.Tree("fixture", pos=(XF, YF, TOP), fixed=True, color=DARK)
_t.link("stand", [
    lk.box((0.140, 0.090, 0.010), (0.0, COL_Y, 0.005), color=IRON),  # cast-iron foot
    lk.box((0.070, 0.070, 0.130), (0.0, COL_Y, 0.075)),  # worm gearbox column, top at 0.140
    lk.cylinder(0.018, 0.010, (0.0, 0.110, AX_Z), axis="Y"),  # trunnion bearing boss on its right face
], mass=3.5)
_t.link("cradle", [
    lk.box((0.120, 0.170, 0.008), (0.0, -0.010, 0.0)),  # tilt table, top at F z 0.114
    lk.cylinder(0.010, 0.045, (0.0, 0.0975, 0.0), axis="Y"),  # trunnion shaft into the column
    lk.cylinder(0.004, 0.030, (-0.035, HINGE_Y, HINGE_Z - AX_Z), axis="X"),  # hinge knuckles
    lk.cylinder(0.004, 0.030, (0.035, HINGE_Y, HINGE_Z - AX_Z), axis="X"),
    lk.box((0.016, 0.015, 0.017), (0.0, -0.0855, PIN_Z - AX_Z)),  # pin bracket
], mass=0.45, at=(0.0, 0.0, AX_Z), color=ALU)
_t.link("panel", [
    lk.box((0.100, 0.120, 0.004)),  # honeycomb board
    lk.box((0.014, 0.008, 0.006), (0.0, -0.064, 0.001)),  # hold-down boss, below the pin slot
    lk.box((0.014, 0.008, 0.006), (0.0, -0.064, 0.011)),  # and above it (4 mm slot at the pin line)
    lk.cylinder(0.004, 0.038, (0.0, HINGE_Y - PANEL_C[1], HINGE_Z - PANEL_C[2]), axis="X"),  # hinge knuckle
], mass=PANEL_MASS, at=PANEL_C, color=GOLD, com=(0.0, 0.0, 0.0), contact_offset=0.001, friction=(0.5, 0.4))
_t.link("pin", [
    lk.cylinder(0.0015, 0.075, (0.0, 0.0455, 0.0), axis="Y", color=STEEL),  # pull rod, tip 6 mm into the boss
    lk.cylinder(0.006, 0.016, (0.0, 0.0, 0.0), axis="Y"),  # red pull knob
], mass=0.015, at=KNOB_C, color=RED)
_t.revolute("tilt", "stand", "cradle", anchor=(0.0, 0.110, AX_Z), axis=(0, 1, 0), limits_deg=(-1.0, 96.0),
            stiffness=TILT_K, damping=TILT_C)
_t.revolute("deploy", "cradle", "panel", anchor=(0.0, HINGE_Y, HINGE_Z), axis=(-1, 0, 0), limits_deg=(0.0, 90.0),
            rest_deg=SPRING_REST_DEG, stiffness=SPRING_K, damping=DEPLOY_C, friction=DEPLOY_F,
            brake=lk.Brake(stiffness=20.0, damping=1.0, engaged=True))
_t.prismatic("pull", "cradle", "pin", anchor=KNOB_C, axis=(0, -1, 0), limits=(0.0, 0.020), damping=2.0,
             friction=0.5)
FIXTURE = _t.build()

# -- the worm's input crank on top of the column (a second articulation: its joints turn without limits) -----------
_t = lk.Tree("crank", pos=(XF, YF, TOP), fixed=True, color=BLACK)
_t.link("post", [lk.cylinder(0.010, 0.006, (0.0, COL_Y, 0.143))], mass=0.05, color=DARK)  # input-shaft bearing
_t.link("crank", [
    lk.cylinder(0.012, 0.006, (0.0, 0.0, 0.0)),  # hub
    lk.box((0.052, 0.012, 0.006), (-0.020, 0.0, 0.0)),  # arm, toward the robot at reset
], mass=0.06, at=(0.0, COL_Y, 0.150))
_t.link("handle", [lk.cylinder(0.008, 0.030)], mass=0.010, at=HANDLE_C)  # free-turning handle sleeve
_t.revolute("turn", "post", "crank", anchor=(0.0, COL_Y, 0.150), axis=(0, 0, 1), damping=0.01, friction=0.03)
_t.revolute("spin", "crank", "handle", anchor=HANDLE_C, axis=(0, 0, 1), damping=1e-4)
CRANK = _t.build()

OBJECTS = [FIXTURE, CRANK]


# -- geometry ---------------------------------------------------------------------------------------------------------


def hinge_axis(cradle_rot: np.ndarray) -> np.ndarray:
    """The hinge line's direction in the world, from the cradle's world rotation (its design rotation is identity)."""
    return np.asarray(cradle_rot, dtype=float) @ np.array([-1.0, 0.0, 0.0])


def axis_from_vertical_deg(cradle_rot: np.ndarray) -> float:
    a = hinge_axis(cradle_rot)
    return math.degrees(math.acos(min(1.0, abs(float(a[2])) / float(np.linalg.norm(a)))))


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Worm:
    """The positioner's worm drive: the trunnion's target follows the crank's accumulated angle / RATIO, between the
    0 and 95 deg stops (past a stop the crank turns without moving the table). The table never drives the crank."""

    def __init__(self):
        self.reset(0.0)

    def reset(self, crank_q: float) -> None:
        self.prev = float(crank_q)
        self.acc = 0.0

    def step(self, crank_q: float) -> float:
        self.acc = min(max(self.acc + wrap(float(crank_q) - self.prev), 0.0), TILT_MAX * RATIO)
        self.prev = float(crank_q)
        return self.acc / RATIO


# -- whole-run record and scoring (pure Python: the tests feed it synthetic samples) --------------------------------


class Record:
    """Whole-run record (a linkkit History plus the release), updated every control step, cleared only by the
    simulator reset: a release made in the wrong orientation, or a hand on the panel after it, is never erased."""

    def __init__(self):
        self.H = lk.History()
        self.reset()

    def reset(self) -> None:
        self.H.reset()
        self.last: dict | None = None
        self.release: dict | None = None
        self.n = 0
        self.trace: list = []

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        H.ever("turned", m["axis_err"] <= VERT_TOL_DEG + EPS)
        H.peak("tilt", m["tilt"])
        H.peak("q", m["q"])
        if m["released"] and self.release is None:
            self.release = {"t": round(H.t, 3), "axis_err": round(m["axis_err"], 2), "tilt": round(m["tilt"], 2),
                            "pin_mm": round(m["pin_mm"], 2)}
        if self.release is not None:
            near = min(m["touch"].values())
            H.low("touch_after", near)
            H.ever("touched", near < CONTACT_M - EPS)
            H.ever("latched", bool(m["latched"]))
            H.peak("q_after", m["q"])
        H.held("still", m["panel_mm_s"] < STILL_MM_S and m["panel_deg_s"] < STILL_DEG_S)
        clear = m["clear"]
        H.held("let_go", min(clear.values()) >= CLEAR_M - EPS and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    m, H, r = rec.last, rec.H, rec.release
    if m is None:
        return 0.0, "no steps yet"
    if r is not None and r["axis_err"] > VERT_TOL_DEG + EPS:
        return 0.0, (f"hold-down released with the hinge axis {r['axis_err']:.1f} deg from vertical "
                     f"(fixture at {r['tilt']:.1f} deg): invalid deployment test")
    if r is not None and "touched" in H.evers:
        return 0.0, (f"a gripper came within {1000 * H.lows['touch_after']:.1f} mm of the panel "
                     f"{H.evers['touched'] - r['t']:.2f} s after the release: assisted deployment, invalid test")
    if r is None:
        if "turned" in H.evers:
            return 0.25, f"hinge axis turned vertical (fixture peak {H.peaks['tilt']:.1f} deg); hold-down not pulled"
        return 0.0, f"hinge axis never within {VERT_TOL_DEG:.0f} deg of vertical (fixture peak {H.peaks['tilt']:.1f} deg)"
    if "latched" not in H.evers:
        return 0.5, (f"valid release at {r['t']:.2f} s (axis {r['axis_err']:.1f} deg off vertical); panel reached "
                     f"{H.peaks.get('q_after', 0.0):.1f} deg, not the latch")
    dt = H.evers["latched"] - r["t"]
    ends = {
        "latched at the end": bool(m["latched"]) and m["q"] >= END_DEG - EPS,
        "hinge axis still vertical": m["axis_err"] <= VERT_TOL_DEG + EPS,
        "panel still 1 s": H.runs.get("still", 0.0) >= RELEASE_S - EPS,
    }
    if not all(ends.values()):
        return 0.75, f"latched {dt:.2f} s after the release; not: " + ", ".join(k for k, ok in ends.items() if not ok)
    if H.runs.get("let_go", 0.0) >= RELEASE_S - EPS:
        return 1.0, (f"deployed by its spring to the latch {dt:.2f} s after a release with the axis "
                     f"{r['axis_err']:.1f} deg off vertical; grippers let go")
    return 0.75, f"latched {dt:.2f} s after the release; grippers not 30 mm clear and still for 1 s"


# -- simulator side ---------------------------------------------------------------------------------------------------

WORM = Worm()
REC = Record()
STATE: dict = {"released": False, "latched": False, "prev": None}


def _rot_deg(r0: np.ndarray, r1: np.ndarray) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(r0.T @ r1)) - 1.0) / 2.0))))


@lk.on_reset
def _reset(tick):
    WORM.reset(tick.states["crank"].q["turn"])
    REC.reset()
    STATE.update(released=False, latched=False, prev=None)


@lk.on_step
def _track(tick):
    fx, cr = tick.states["fixture"], tick.states["crank"]
    lk.set_rest("fixture", "tilt", WORM.step(cr.q["turn"]))
    if not STATE["released"] and fx.q["pull"] >= PIN_RELEASE_M:  # the hold-down lets go of the panel
        lk.brake("fixture", "deploy", False)
        STATE["released"] = True
    q = fx.deg("deploy")
    if STATE["released"] and not STATE["latched"] and q >= LATCH_DEG:  # the pawl drops in
        lk.set_limits("fixture", "deploy", math.radians(PAWL_DEG), math.radians(90.0))
        STATE["latched"] = True
    pose = fx.poses["panel"]
    v = w = 0.0
    if STATE["prev"] is not None and tick.t > STATE["prev"][0]:
        dt = tick.t - STATE["prev"][0]
        v = 1000.0 * float(np.linalg.norm(pose[:3, 3] - STATE["prev"][1][:3, 3])) / dt
        w = _rot_deg(STATE["prev"][1][:3, :3], pose[:3, :3]) / dt
    STATE["prev"] = (tick.t, pose.copy())
    tools = tick.tools
    m = {
        "axis_err": axis_from_vertical_deg(fx.rotation("cradle")),
        "tilt": fx.deg("tilt"),
        "pin_mm": 1000.0 * fx.q["pull"],
        "released": STATE["released"],
        "q": q,
        "latched": STATE["latched"],
        "touch": {s: lk.clearance(t, fx, ["panel"]) for s, t in tools.items()},
        "clear": {s: min(lk.clearance(t, fx, FIX_MOVING), lk.clearance(t, cr, CRANK_MOVING)) for s, t in tools.items()},
        "tcp_speed": {s: float(t.speed) for s, t in tools.items()},
        "panel_mm_s": v,
        "panel_deg_s": w,
    }
    REC.update(m)
    if REC.n % 5 == 0:  # diagnostics every 0.25 s
        REC.trace.append((round(tick.t, 2), round(m["tilt"], 2), round(m["axis_err"], 2), round(m["pin_mm"], 2),
                          round(q, 2), int(STATE["latched"]), [round(1000 * x) for x in m["touch"].values()],
                          [round(1000 * x) for x in m["clear"].values()]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ----------------------------------------------------------------------------------------------------
# linkdemo.py runs demo_script(ctx). It reads only what a camera could measure: the fixture's tilt from the cradle's
# pose against the stand, the crank handle's and the pull knob's positions, and the robot's own grippers. It never
# reads the brake, the latch, joint values or the record.

PAD_TIP = 0.013
DEMO_LOG: list = []
PARK = {"left": np.array([0.220, 0.240, TOP + 0.170]), "right": np.array([0.240, -0.240, TOP + 0.170])}


def _log(ctx, tag: str, **kw) -> None:
    from yam_arena import ik

    seen = ctx.observe()
    for side in ("left", "right"):
        arm, tool = ctx.arm(side), seen.tools[side]
        p, _ = ik.tcp_pose(arm.q, arm.base)
        kw[side] = {"ik_mm": round(1000 * float(np.linalg.norm(p - arm.pos)), 1),
                    "tcp_err_mm": [round(1000 * float(x), 1) for x in tool.tcp - arm.pos],
                    "open_mm": round(1000 * tool.opening, 1)}
    DEMO_LOG.append({"at": tag, "tilt_seen": round(seen.angle_deg("fixture", "tilt"), 2),
                     "panel_seen": round(seen.angle_deg("fixture", "deploy"), 2), **kw})


def crank(ctx, tilt_deg: float, *, fix: bool = True):
    """Left arm: grip the handle sleeve from above, turn the crank anticlockwise (seen from above) by tilt x RATIO,
    correct from the seen tilt, let go and park."""
    import linkdemo as ld

    arm = ctx.left
    seen = ctx.observe()
    h = seen.point("crank", "handle")
    centre, _ = seen.axis("crank", "turn")
    start = seen.angle_deg("fixture", "tilt")
    rot = ld.grip_frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))  # pads close along world x
    tcp = h + np.array([0.0, 0.0, PAD_TIP - 0.011])  # pad tips 11 mm below the sleeve centre (4 mm above its foot)
    yield from arm.move(tcp + [0.0, 0.0, 0.035], rot, width=0.050, duration=1.8)
    yield from arm.move(tcp, rot, duration=1.2)
    yield from arm.grip(0.010, 0.6)
    yield from arm.wait(0.3)
    c = np.array([centre[0], centre[1], arm.pos[2]])
    turn = tilt_deg * RATIO
    yield from arm.arc(c, (0.0, 0.0, 1.0), turn, follow=0.0, duration=max(2.0, abs(turn) / 75.0))
    yield from arm.wait(0.4)
    _log(ctx, "cranked", target=round(start + tilt_deg, 1))
    for _ in range(3 if fix else 0):
        err = start + tilt_deg - ctx.observe().angle_deg("fixture", "tilt")
        if abs(err) <= 0.7:
            break
        yield from arm.arc(c, (0.0, 0.0, 1.0), err * RATIO, follow=0.0, duration=max(1.0, abs(err * RATIO) / 60.0))
        yield from arm.wait(0.4)
        _log(ctx, "crank fix", err=round(err, 2))
    yield from arm.open(0.5)
    yield from arm.move(arm.pos + [-0.03, 0.0, 0.035], duration=1.0)  # up and back: straight up is past the reach
    yield from arm.move(PARK["left"], duration=1.5)


def pull_pin(ctx, out_m: float = 0.016):
    """Right arm: pinch the red knob from above (pads closing along x), pull it straight out along its axis, let go,
    lift and park."""
    import linkdemo as ld

    arm = ctx.right
    seen = ctx.observe()
    k = seen.point("fixture", "pin")  # knob centre
    _, out = seen.axis("fixture", "pull")
    rot = ld.grip_frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))
    tcp = k + np.array([0.0, 0.0, PAD_TIP - 0.004])
    yield from arm.move(tcp + [0.0, 0.0, 0.06], rot, width=0.050, duration=1.5)
    yield from arm.move(tcp, rot, duration=1.2)
    yield from arm.grip(0.006, 0.6)
    yield from arm.wait(0.3)
    _log(ctx, "knob gripped", knob_mm=[round(1000 * float(x), 1) for x in ctx.observe().tools["right"].tcp - k])
    yield from arm.move(tcp + out * out_m, rot, duration=1.5)
    yield from arm.wait(0.4)
    _log(ctx, "pulled", pin_mm=round(1000 * float((ctx.observe().point("fixture", "pin") - k) @ out), 2))
    yield from arm.open(0.5)
    yield from arm.move(arm.pos + [0.0, 0.0, 0.08], duration=1.0)
    yield from arm.move(PARK["right"], duration=1.5)


def lift_panel(ctx):
    """Cheat: pinch the panel's boss (pads closing along the hinge) and swing it up to the latch by hand."""
    seen = ctx.observe()
    local = (0.0, -0.064, 0.008)  # boss, above the board
    yield from ctx.pinch_pivot("right", "fixture", "deploy", 92.0 - seen.angle_deg("fixture", "deploy"), local=local,
                               width=0.014, squeeze=0.002, follow=0.5, contact_depth=PAD_TIP - 0.001, pivot_s=5.0)
    _log(ctx, "lifted by hand", pivot=getattr(ctx, "pivot_log", None))
    arm = ctx.right
    yield from arm.open(0.5)
    yield from arm.retreat(0.06, 1.0)
    yield from arm.move(PARK["right"], duration=1.5)


def demo_script(ctx):
    v = VARIANT
    yield from ctx.wait(0.5)
    if v == "nothing":
        yield from ctx.wait(59.0)
        return
    if v == "as_delivered":  # pull the pin with the fixture as delivered (hinge axis horizontal)
        yield from pull_pin(ctx)
    elif v == "help_up":  # as delivered, then lift the panel to its latch by hand
        yield from pull_pin(ctx)
        yield from ctx.wait(1.0)
        yield from lift_panel(ctx)
    elif v == "pull_then_turn":  # release first, then turn the hinge axis vertical
        yield from pull_pin(ctx)
        yield from crank(ctx, 90.0)
    elif v == "partial_tilt":  # one and a half crank turns (45 deg), then release
        yield from crank(ctx, 45.0)
        yield from pull_pin(ctx)
    else:  # the demo: hinge axis vertical first, then release
        yield from crank(ctx, 90.0)
        yield from pull_pin(ctx)
    yield from ctx.wait(3.0)
    _log(ctx, "end")
    yield from ctx.wait(30.0)


def demo_report() -> dict:
    H = REC.H
    return {"ladder": ladder(REC)[1], "variant": VARIANT, "release": REC.release, "evers": dict(H.evers),
            "peaks": dict(H.peaks), "lows": dict(H.lows), "runs": dict(H.runs), "demo": DEMO_LOG,
            "worm_acc_deg": round(math.degrees(WORM.acc), 1), "trace": REC.trace}
