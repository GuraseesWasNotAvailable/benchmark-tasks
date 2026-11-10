# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Stow a deployable radiator panel that is held open by an over-centre (knee-locked) strut.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A stowage fixture (dark grey anodised aluminium) is fixed to the table. On it a grey 200 x 120 x 5 mm honeycomb
  radiator panel (250 g) turns on a hinge line along y at x 0.33, 40 mm above the table. The panel stands open at
  75 deg, leaning toward the robot, with a 30 mm grab tab on its free edge (on the robot's left half).
- A two-link deployment strut (yellow anodised aluminium bars, a steel knee pin) runs beside the panel's right edge
  from a pivot on the fixture, 120 mm in front of the hinge line, to a lug 60 mm up the panel. It is locked straight:
  its knee sits 3 deg past straight against a stop, on the side away from the robot and down.
- Under the stowed position a grey support block carries two black rubber rest pads.

Physics, not words, defeats the natural plan. Pushing the panel down loads the strut's ends and compresses it, which
drives the knee harder into its stop, so the panel does not move (the press_face cheat drives closed fingers 30 mm
past contact into the panel's face for 3 s); lifting the panel only straightens the strut, and the links' weight drops
the knee back onto its stop. Pushing the knee pin up and back across centre, perpendicular to the strut, unlocks it;
then nothing holds the panel, and if it is not held it falls 75 deg and slams onto the pads.

Goal: the panel resting on its rest pads, brought down without slamming, with both grippers let go.
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

# -- geometry ---------------------------------------------------------------------------------------------------
# Linkage frame: origin on the hinge line at the panel's centre (world (0.33, 0, TOP + 0.040)), axes as the world's.
# Panel angle theta: 0 stowed flat on the pads, 75 deployed; the panel runs from the hinge line toward the robot.

HINGE_X = 0.33
HINGE_H = 0.040  # hinge line above the table (m)
HINGE_Z = TOP + HINGE_H
DEPLOY_DEG = 75.0  # the panel's locked-open angle
LOCK_DEG = 3.0  # the knee's lock: this far past straight, against its stop
OVER_DEG = 80.0  # panel over-travel stop
A_B = 0.120  # strut base pivot B: this far in front of the hinge line (toward the robot), at hinge height
B_P = 0.060  # strut-to-panel pivot P: this far along the panel from the hinge line
Y_UP, Y_LO = -0.117, -0.127  # mid-planes of the strut's upper and lower links (beside the panel's right edge)
PANEL_LEN, PANEL_W, PANEL_T = 0.120, 0.200, 0.005  # hinge line to free edge, along the hinge, thickness
PANEL_R0 = 0.005  # the panel starts 5 mm from the hinge line
TAB = (0.025, 0.030, 0.005)  # free-edge grab tab: out from the edge, across, thick
TAB_Y = 0.060
PANEL_MASS = 0.250
LINK_MASS = 0.015
BAR = (0.012, 0.008)  # strut bar section: in the strut plane, across it
PIN_R, PIN_LEN = 0.005, 0.025  # steel knee pin, sticking out toward -y from the lower link's outer face

BASE_GREY = (0.26, 0.27, 0.29)
PANEL_GREY = (0.72, 0.73, 0.75)
STRUT_YELLOW = (0.93, 0.72, 0.12)
STEEL = (0.80, 0.80, 0.82)
SUPPORT_GREY = (0.46, 0.48, 0.51)
PAD_BLACK = (0.05, 0.05, 0.05)


def u_dir(theta_deg: float) -> np.ndarray:
    """Unit vector along the panel, from the hinge line toward its free edge (linkage frame)."""
    t = math.radians(theta_deg)
    return np.array([-math.cos(t), 0.0, math.sin(t)])


def _xz(v) -> np.ndarray:
    return np.array([float(v[0]), float(v[2])])


B_PT = np.array([-A_B, Y_LO, 0.0])
P_PT = B_P * u_dir(DEPLOY_DEG) + np.array([0.0, Y_UP, 0.0])
_BP = _xz(P_PT) - _xz(B_PT)
BP_LEN = float(np.linalg.norm(_BP))
LINK_LEN = BP_LEN / (2.0 * math.cos(math.radians(LOCK_DEG / 2)))  # both links; straight they would span B-P exactly
_E = _BP / BP_LEN
N_UB = np.array([-_E[1], _E[0]])  # unit normal to B-P, up and back toward the robot: the knee's unlocking side
_K = _xz(B_PT) + _BP / 2 - N_UB * LINK_LEN * math.sin(math.radians(LOCK_DEG / 2))  # locked knee: 3 deg past, down/fwd
K_PT = np.array([_K[0], (Y_UP + Y_LO) / 2, _K[1]])


def _bar_pose(a, b, y):
    """Mid-point and pitch (deg) of a bar from a to b in the x-z plane (local x along it)."""
    d = _xz(b) - _xz(a)
    mid = (_xz(a) + _xz(b)) / 2
    return (float(mid[0]), y, float(mid[1])), math.degrees(math.atan2(-d[1], d[0]))


LOWER_AT, LOWER_PITCH = _bar_pose(B_PT, K_PT, Y_LO)
UPPER_AT, UPPER_PITCH = _bar_pose(K_PT, P_PT, Y_UP)
PANEL_PITCH = -(180.0 - DEPLOY_DEG)  # panel link frame: x along the panel (u), z out of its inner face

# -- bodies -----------------------------------------------------------------------------------------------------

_t = lk.Tree("radiator", pos=(HINGE_X, 0.0, HINGE_Z), fixed=True, color=BASE_GREY)
_t.link("base", [
    lk.box((0.200, 0.275, 0.006), (-0.065, -0.0275, -HINGE_H + 0.003)),  # foot plate on the table
    lk.box((0.024, 0.190, HINGE_H - 0.002), (0.018, 0.0, -HINGE_H / 2 + 0.003)),  # hinge pedestal
    lk.box((0.037, 0.012, HINGE_H + 0.001), (0.0115, 0.106, -HINGE_H / 2 + 0.0065)),  # hinge cheeks
    lk.box((0.037, 0.012, HINGE_H + 0.001), (0.0115, -0.106, -HINGE_H / 2 + 0.0065)),
    lk.cylinder(0.006, 0.012, (0.0, 0.106, 0.0), axis="Y"),  # hinge knuckles
    lk.cylinder(0.006, 0.012, (0.0, -0.106, 0.0), axis="Y"),
    lk.box((0.012, 0.010, HINGE_H + 0.001), (-A_B, -0.136, -HINGE_H / 2 + 0.0065)),  # strut post (pivot B)
], mass=2.0)
_t.link("panel", [
    lk.box((PANEL_LEN, PANEL_W, PANEL_T), (PANEL_R0 + PANEL_LEN / 2, 0.0, 0.0)),
    lk.box(TAB, (PANEL_R0 + PANEL_LEN + TAB[0] / 2, TAB_Y, 0.0)),
    lk.box((0.014, 0.013, PANEL_T), (B_P, -(PANEL_W / 2 + 0.0065), 0.0)),  # strut lug on the right edge
], mass=PANEL_MASS, rpy_deg=(0.0, PANEL_PITCH, 0.0), color=PANEL_GREY, friction=(0.8, 0.8), contact_offset=0.001)
_t.link("lower", [
    lk.box((LINK_LEN + BAR[0], BAR[1], BAR[0])),
    lk.cylinder(PIN_R, PIN_LEN, (LINK_LEN / 2, -(BAR[1] / 2 + PIN_LEN / 2), 0.0), axis="Y", color=STEEL),
], mass=LINK_MASS, at=LOWER_AT, rpy_deg=(0.0, LOWER_PITCH, 0.0), color=STRUT_YELLOW, friction=(0.6, 0.6))
_t.link("upper", [lk.box((LINK_LEN + BAR[0], BAR[1], BAR[0]))], mass=LINK_MASS, at=UPPER_AT,
        rpy_deg=(0.0, UPPER_PITCH, 0.0), color=STRUT_YELLOW, friction=(0.6, 0.6))
# joint angles are 0 at the design pose (panel at 75 deg, knee on its stop); positive about +y
_t.revolute("hinge", "base", "panel", anchor=(0.0, 0.0, 0.0), axis=(0, 1, 0),
            limits_deg=(-DEPLOY_DEG - 3.0, OVER_DEG - DEPLOY_DEG), damping=1e-3)
_t.revolute("root", "base", "lower", anchor=tuple(B_PT), axis=(0, 1, 0), damping=1e-4)
_t.revolute("knee", "lower", "upper", anchor=tuple(K_PT), axis=(0, 1, 0), limits_deg=(0.0, 150.0 + LOCK_DEG),
            damping=1e-4)
_t.loop("lug", "upper", "panel", anchor=tuple(P_PT), axis=(0, 1, 0))
RADIATOR = _t.build()

PAD_TOP = -PANEL_T / 2  # rest pads' top face: the stowed panel's underside (linkage frame z)
_t = lk.Tree("rest", pos=(HINGE_X, 0.0, HINGE_Z), fixed=True, color=SUPPORT_GREY)
_t.link("support", [
    lk.box((0.070, 0.170, PAD_TOP - 0.003 + HINGE_H - 0.006), (-0.080, 0.0, (PAD_TOP - 0.003 - HINGE_H + 0.006) / 2)),
    lk.box((0.020, 0.024, 0.003), (-0.095, 0.065, PAD_TOP - 0.0015), color=PAD_BLACK, contact_offset=0.001),
    lk.box((0.020, 0.024, 0.003), (-0.095, -0.065, PAD_TOP - 0.0015), color=PAD_BLACK, contact_offset=0.001),
], mass=0.5, friction=(0.9, 0.9))
REST = _t.build()

OBJECTS = [RADIATOR, REST]

# -- scoring ----------------------------------------------------------------------------------------------------

GATE_RATE = 60.0  # deg/s: faster than this at any time in the run gives 0 (a slam)
UNLOCK_DEG = 8.0  # knee bent this far past straight, toward folding: the strut is unlocked
HALF_DEG = 40.0  # panel lowered to this angle or below at some time
PADS_TOL_DEG = 2.0  # at the end: panel within this of its pads (stowed, theta 0)
STILL_RATE = 3.0  # deg/s
STEADY_S = 1.0
CLEAR_M = 0.030  # release gate: every pad point of both grippers this far from the fixture, panel, strut and pads
TCP_STILL = 0.005  # m/s
RELEASE_S = 1.0
LOOP_TOL = 0.002  # m: the strut's lug joint opened this far at any time means the simulated strut came apart (gate)
EPS = 1e-6


def measure(rad, rest, tools) -> dict:
    """One sample (deg; clearances m; speeds m/s). ``rate`` (panel deg/s) is filled in from the previous step."""
    clear = {s: min(lk.clearance(t, rad), lk.clearance(t, rest)) for s, t in tools.items()}
    return {
        "theta": DEPLOY_DEG + rad.deg("hinge"),
        "bend": rad.deg("knee") - LOCK_DEG,
        "lug": float(rad.joint_error("lug")),  # m: anchor gap of the strut-to-panel loop joint
        "rate": 0.0,
        "clear": clear,
        "tcp_speed": {s: float(t.speed) for s, t in tools.items()},
    }


def on_pads(m) -> bool:
    return m["theta"] <= PADS_TOL_DEG


class Record:
    """Whole-run record: a linkkit History updated every control step and cleared only by the simulator reset, so a
    slam is never erased by lifting the panel again. Pure Python."""

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
        H.peak("lug_max", m.get("lug", 0.0))
        H.ever("strut_apart", m.get("lug", 0.0) > LOOP_TOL)
        H.peak("rate_max", abs(m["rate"]))
        H.ever("slammed", abs(m["rate"]) > GATE_RATE)
        H.peak("bend_max", m["bend"])
        H.low("bend_min", m["bend"])
        H.ever("unlocked", m["bend"] >= UNLOCK_DEG)
        H.low("theta_min", m["theta"])
        H.peak("theta_max", m["theta"])
        H.ever("lowered", m["theta"] <= HALF_DEG)
        H.held("steady", on_pads(m) and abs(m["rate"]) < STILL_RATE)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Score and the reason. The gate is checked first (any time in the run), then the rungs from the top."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "strut_apart" in H.evers:
        return 0.0, (f"gate: the strut's lug joint opened {1000 * H.peaks['lug_max']:.1f} mm at "
                     f"{H.evers['strut_apart']:.2f} s (over {1000 * LOOP_TOL:.0f} mm: the simulated strut came apart)")
    if "slammed" in H.evers:
        return 0.0, (f"gate: the panel moved faster than {GATE_RATE:.0f} deg/s at {H.evers['slammed']:.2f} s "
                     f"(peak {H.peaks['rate_max']:.0f} deg/s)")
    if on_pads(m):
        ends = {
            "panel still 1 s": H.runs.get("steady", 0.0) >= STEADY_S - EPS,
            "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS,
        }
        if all(ends.values()):
            return 1.0, "panel on its rest pads, brought down without slamming, both grippers let go"
        return 0.75, "panel on its pads at the end; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if "lowered" in H.evers:
        return 0.5, f"the panel came down to {HALF_DEG:.0f} deg at some point but is not on its pads at the end"
    if "unlocked" in H.evers:
        return 0.25, "the strut's knee was pushed across centre, but the panel never came down to 40 deg"
    return 0.0, "the strut stayed locked: the panel never came down"


REC = Record()
_PREV: dict = {}  # the panel angle at the previous control step, for its rate


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    m = measure(tick.states["radiator"], tick.states["rest"], tick.tools)
    if _PREV and tick.t > _PREV["t"]:
        m["rate"] = (m["theta"] - _PREV["theta"]) / (tick.t - _PREV["t"])
    _PREV.update(theta=m["theta"], t=tick.t)
    REC.update(m)
    if REC.n % 5 == 0:  # diagnostics every 0.25 s
        rad = tick.states["radiator"]
        REC.trace.append((round(tick.t, 2), round(m["theta"], 2), round(m["bend"], 2), round(m["rate"], 1),
                          [round(1000 * m["clear"][s]) for s in ("left", "right")],
                          round(1000 * rad.joint_error("lug"), 2)))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ----------------------------------------------------------------------------------------------
# linkdemo.py runs demo_script(ctx). It reads only what a camera could measure (link poses, hinge and knee angles
# computed from link poses) and the robot's own grippers; never joint readouts or the record.

VARIANTS = ("", "nothing", "push_panel", "knee_only", "press_face")
TAB_LOCAL = (PANEL_R0 + PANEL_LEN + 0.015, TAB_Y, 0.0)  # pinch point on the tab, 15 mm out from the free edge
CONTACT_DEPTH = 0.0115  # pinch centre this far past the TCP
TAB_TILT = 15.0  # deg: the left approaches the tab tilted this far from straight down, leaning back toward the robot
SQUEEZE = 0.0010
PIN_LOCAL = (LINK_LEN / 2, -(BAR[1] / 2 + 0.015), 0.0)  # on the knee pin, 15 mm out from the lower link's face
FINGER_DX = 0.020  # closed fingers start this far in front of the pin centre (TCP)
FINGER_TIPS_BELOW = 0.040  # with the pad tips this far below the pin centre
PUSH_STEP = 0.0006  # m per control step (12 mm/s) along the knee's unlocking direction
DEMO_BEND = 10.0  # push until the knee is seen this far past straight
CHEAT_BEND = 35.0
PRESS_LOCAL = (PANEL_R0 + PANEL_LEN - 0.012, -0.060, -PANEL_T / 2)  # on the panel's top face, 12 mm in from the edge
PRESS_TILT = 15.0  # deg: press_face approach tilted from straight down, leaning back toward the robot
PRESS_PAST = 0.030  # press_face: the fingers are commanded this far past the face
PRESS_HOLD_S = 3.0
COMMIT_DEG = 5.0  # the left arm lowers this far before the right lets go of the knee
LOG: dict = {}


def _knee_dir(seen) -> np.ndarray:
    """The knee pin's direction of motion when it unlocks (perpendicular to the lower link, up and back), from the
    lower link's pose."""
    x = seen.pose("radiator", "lower")[:3, 0]
    return np.array([-x[2], 0.0, x[0]]) / math.hypot(x[0], x[2])


def demo_script(ctx):
    """Left pinches the free-edge tab and holds the panel; right's closed fingers push the knee pin up and back
    across centre and hold it until the left has started lowering; the left carries the panel onto its pads."""
    from linkdemo import OPEN_WIDTH, down, grip_frame
    from yam_arena.constants import PAD_TIP_PAST_TCP

    variant = VARIANT or getattr(ctx, "variant", "")
    LOG.clear()
    LOG["variant"] = variant
    flags = {"held": False, "unlocked": False, "committed": False}
    yield from ctx.wait(1.0)
    seen = ctx.observe()
    LOG["seen_start"] = {"theta": round(DEPLOY_DEG + seen.angle_deg("radiator", "hinge"), 2),
                         "bend": round(seen.angle_deg("radiator", "knee") - LOCK_DEG, 2)}
    if variant == "nothing":
        yield from ctx.wait(5.0)
        return

    def theta_seen():
        return DEPLOY_DEG + ctx.observe().angle_deg("radiator", "hinge")

    def left():
        arm = ctx.left
        s = ctx.observe()
        tab = s.point("radiator", "panel", TAB_LOCAL)
        hinge, axis = s.axis("radiator", "hinge")
        app = np.array([math.sin(math.radians(TAB_TILT)), 0.0, -math.cos(math.radians(TAB_TILT))])
        r = grip_frame(app, axis)
        tcp = tab - r @ np.array([0.0, 0.0, CONTACT_DEPTH])
        yield from arm.move(tcp - app * 0.06, r, width=OPEN_WIDTH, duration=2.0)
        yield from arm.move(tcp, r, duration=1.5)
        yield from arm.grip(TAB[1] - SQUEEZE, 0.8)
        yield from arm.wait(0.4)
        flags["held"] = True
        if variant == "push_panel":  # the natural plan: fold the panel down by its tab, never touching the strut
            yield from arm.arc(hinge, axis, -(DEPLOY_DEG + 1.0), point=(0, 0, CONTACT_DEPTH), follow=0.5,
                               duration=6.0)
            LOG["theta_after_push"] = round(theta_seen(), 2)
        else:
            while not flags["unlocked"]:
                yield
            yield from arm.arc(hinge, axis, -COMMIT_DEG, point=(0, 0, CONTACT_DEPTH), follow=0.5, duration=1.2)
            LOG["theta_committed"] = round(theta_seen(), 2)
            flags["committed"] = True
            th = theta_seen()
            yield from arm.arc(hinge, axis, -(th + 1.0), point=(0, 0, CONTACT_DEPTH), follow=0.5,
                               duration=max(1.0, (th + 1.0) / 14.0))
            for _ in range(2):  # close the loop on the angle seen from the link poses
                yield from arm.wait(0.3)
                th = theta_seen()
                if th <= 0.8:
                    break
                yield from arm.arc(hinge, axis, -(th + 0.5), point=(0, 0, CONTACT_DEPTH), follow=0.5,
                                   duration=max(0.6, th / 8.0))
            LOG["theta_down"] = round(theta_seen(), 2)
        yield from arm.wait(0.5)
        yield from arm.open(0.6)
        yield from arm.retreat(0.07, 1.2)
        yield from arm.move(arm.pos + [0.0, 0.0, 0.04], duration=0.8)

    def right():
        arm = ctx.right
        s = ctx.observe()
        pin = s.point("radiator", "lower", PIN_LOCAL)
        rot = down(90.0)  # straight down, pads closing along x
        start = pin + np.array([FINGER_DX, 0.0, PAD_TIP_PAST_TCP - FINGER_TIPS_BELOW])
        yield from arm.grip(0.0, 0.5)
        yield from arm.move(start + [0.0, 0.0, 0.08], rot, width=0.0, duration=2.0)
        yield from arm.move(start, rot, duration=1.5)
        while not flags["held"]:
            yield
        target = DEMO_BEND if variant != "knee_only" else CHEAT_BEND
        travel, bend = 0.0, -LOCK_DEG
        while travel < 0.075:
            w = _knee_dir(ctx.observe())
            yield from arm.move(arm.pos + w * PUSH_STEP, rot, duration=0.05)
            travel += PUSH_STEP
            bend = ctx.observe().angle_deg("radiator", "knee") - LOCK_DEG
            if bend >= target:
                break
        LOG["push"] = {"travel_mm": round(1000 * travel, 1), "bend_seen": round(bend, 2)}
        flags["unlocked"] = True
        if variant != "knee_only":
            while not flags["committed"]:
                yield
        yield from arm.move(arm.pos + [0.015, 0.0, 0.0], rot, duration=0.5)  # off the pin, forward
        yield from arm.move(arm.pos + [0.0, -0.05, 0.0], rot, duration=0.8)  # outboard, clear of the pin's end
        yield from arm.move(arm.pos + [-0.03, 0.0, 0.10], rot, duration=1.2)  # up and back

    def press_face():  # cheat: closed fingers driven hard into the panel's top face near its free edge, folding way
        arm = ctx.right
        s = ctx.observe()
        app = np.array([math.sin(math.radians(PRESS_TILT)), 0.0, -math.cos(math.radians(PRESS_TILT))])
        rot = grip_frame(app, (0.0, 1.0, 0.0))  # pads closing along y; the closed fingers' face meets the panel
        fold = s.pose("radiator", "panel")[:3, 2]  # out of the panel's underside: toward the robot and down
        tips = s.point("radiator", "panel", PRESS_LOCAL) - fold * 0.025 - [0.0, 0.0, 0.010]  # 25 mm off, beyond it
        tcp = tips - app * PAD_TIP_PAST_TCP  # pad tips 10 mm below the press point
        yield from arm.grip(0.0, 0.5)
        yield from arm.move(tcp + [0.0, 0.0, 0.08], rot, width=0.0, duration=2.0)
        yield from arm.move(tcp, rot, duration=1.5)
        travel, worst = 0.0, DEPLOY_DEG
        while travel < 0.025 + PRESS_PAST - 1e-9:  # 12 mm/s through contact to 30 mm past the face
            yield from arm.move(arm.pos + fold * PUSH_STEP, rot, duration=0.05)
            travel += PUSH_STEP
            worst = min(worst, theta_seen())
        for _ in range(round(PRESS_HOLD_S / 0.25)):  # hold the command 30 mm past the face
            yield from arm.wait(0.25)
            worst = min(worst, theta_seen())
        short = float(np.linalg.norm(arm.pos - ctx.observe().tools["right"].tcp))  # held back by the panel
        LOG["press"] = {"commanded_past_face_mm": round(1000 * (travel - 0.025), 1), "theta_min_seen": round(worst, 2),
                        "tcp_short_of_command_mm": round(1000 * short, 1)}
        yield from arm.move(arm.pos - fold * 0.03, rot, duration=1.5)
        yield from arm.move(arm.pos + [0.0, 0.0, 0.10], rot, duration=1.2)

    if variant == "press_face":
        gens = [press_face()]
    elif variant == "knee_only":  # push the knee across centre and back away; nothing holds the panel
        flags["held"] = True
        gens = [right()]
    elif variant == "push_panel":  # fold the panel down by its tab; the strut is never touched
        gens = [left()]
    else:
        gens = [left(), right()]
    yield from ctx.both(*gens)
    yield from ctx.wait(1.5)
    seen = ctx.observe()
    LOG["seen_end"] = {"theta": round(DEPLOY_DEG + seen.angle_deg("radiator", "hinge"), 2),
                       "bend": round(seen.angle_deg("radiator", "knee") - LOCK_DEG, 2)}


def demo_report() -> dict:
    H = REC.H
    s, why = ladder(REC)
    return {"score": s, "why": why, "variant": VARIANT, "log": dict(LOG),
            "peaks": {k: round(v, 3) for k, v in H.peaks.items()}, "lows": {k: round(v, 3) for k, v in H.lows.items()},
            "evers": dict(H.evers), "runs": {k: round(v, 2) for k, v in H.runs.items()},
            "last": REC.last and {k: (round(v, 3) if isinstance(v, float) else v) for k, v in REC.last.items()},
            "trace": REC.trace[::2]}
