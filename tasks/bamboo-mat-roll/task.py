# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Roll a maki with a bamboo mat, seam side down, then open the mat.

Station table; world frame in metres from the midpoint between the two arm bases, x forward (away from the robot),
y to the robot's left, z up; the table top is at z 0.7475.

- A 100 mm end-grain maple butcher block, 280 x 190 mm, is fixed to the table from x 0.245 to 0.525, y -0.095 to
  0.095.
- On it lies a bamboo sushi mat (makisu), 240 x 240 mm: 24 slats, each 9 mm wide (along x), 240 mm long (along y)
  and 3 mm thick, strung every 10 mm and hinged to each other along their long edges. Its near edge (toward the
  robot) is at x 0.2655, its far edge at x 0.5045; the slat ends overhang the block by 25 mm on each side.
- On the mat lies a sheet of nori spread with sushi rice, rice side up: 190 mm wide (along y) and 87.6 mm long
  (along x, one turn round the cucumber), from x 0.3312 to x 0.4188, centred on y 0. It bends freely but does not
  stretch.
- A green cucumber, 26 mm across and 180 mm long, is pressed into the rice along the sheet's near edge, its axis along
  y at x 0.335 (84 mm of sheet ahead of it). The rice holds it to the strip of sheet under it.

Goal: the sheet wrapped right round the cucumber with its far edge underneath (seam side down), the mat lying open
and flat again with the roll on it, and both grippers let go and clear. The mat is the tool: the grippers must never
touch the food, and the mat must have been curled over the cucumber before the sheet is half wrapped.
"""

from __future__ import annotations

import math
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

# -- layout (design poses; object_jitter = 0) ---------------------------------------------------------------------
BOARD_X = (0.245, 0.525)
BOARD_H = 0.100  # a standard 4 in end-grain butcher block
BOARD_TOP = TOP + BOARD_H
SLAT = (0.009, 0.240, 0.003)
PITCH = 0.010
N_SLATS = 24
MAT_X0 = 0.270  # slat 0 centre (the near edge the robot lifts)
MAT_Z = BOARD_TOP + SLAT[2] / 2
NORI_LINK = (0.0076, 0.190, 0.0017)  # one strip of the sheet: 0.5 mm of nori under 1.2 mm of rice
NORI_SKIN = 0.0005
NORI_PITCH = 0.008
N_NORI = 11  # one turn round the cucumber: rolled up, the far edge meets the near edge
CORE_X = 0.335  # cucumber axis, 65 mm from slat 0's centre: the flap of mat that curls over it
TACK = 0  # the strip the cucumber is pressed into: the sheet's near edge
NORI_EDGE = CORE_X - NORI_LINK[0] / 2  # near edge of the sheet (x 0.3312); its far edge is at x 0.4188
LINK_X = tuple(NORI_EDGE + NORI_LINK[0] / 2 + i * NORI_PITCH for i in range(N_NORI))
CORE_R = 0.013
CORE_LEN = 0.180
CORE_MASS = 0.095
CORE_ROLL_DAMPING = 150.0  # 1/s: the soft rice roll's rolling resistance (a rigid cylinder in PhysX has none)
NORI_Z = BOARD_TOP + SLAT[2] + NORI_LINK[2] / 2
CORE_Z0 = BOARD_TOP + SLAT[2] + NORI_LINK[2] + CORE_R  # cucumber axis height at reset

MAPLE = (0.78, 0.62, 0.42)
BAMBOO = [(0.80, 0.68, 0.42), (0.74, 0.62, 0.36)]
NORI_GREEN = (0.10, 0.22, 0.12)
RICE_WHITE = (0.97, 0.97, 0.94)
CUCUMBER = (0.30, 0.55, 0.20)

_b = lk.Tree("board", pos=((BOARD_X[0] + BOARD_X[1]) / 2, 0.0, TOP + BOARD_H / 2), fixed=True, color=MAPLE)
_b.link("block", [lk.box((BOARD_X[1] - BOARD_X[0], 0.190, BOARD_H))], mass=3.7, friction=(0.35, 0.30))
BOARD = _b.build()

MAT = lk.chain("mat", N_SLATS, start=(MAT_X0, 0.0, MAT_Z), pitch=PITCH,
               shapes=[lk.box(SLAT, contact_offset=0.001, rest_offset=0.0)], mass=0.0045, axis=(0, 1, 0),
               limits_deg=(-160, 160), damping=2e-4, friction=5e-4, armature=1e-5, color=BAMBOO[0], colors=BAMBOO,
               link_kw={"friction": (0.35, 0.30), "contact_offset": 0.001})

# The sheet and the cucumber are one articulation: the cucumber (root) is welded to strip l0, the rice it is pressed
# into; the strips are hinged to their neighbours with a weak spring to flat.
_n = lk.Tree("nori", pos=(CORE_X, 0.0, CORE_Z0), color=NORI_GREEN)
_n.link("core", [lk.cylinder(CORE_R, CORE_LEN, axis="Y", color=CUCUMBER)], mass=CORE_MASS, color=CUCUMBER,
        friction=(1.5, 1.3), angular_damping=CORE_ROLL_DAMPING)
_STRIP = [lk.box((NORI_LINK[0], NORI_LINK[1], NORI_SKIN), (0.0, 0.0, (NORI_SKIN - NORI_LINK[2]) / 2),
                 color=NORI_GREEN, contact_offset=0.001, rest_offset=0.0),
          lk.box((NORI_LINK[0], NORI_LINK[1], NORI_LINK[2] - NORI_SKIN), (0.0, 0.0, NORI_SKIN / 2),
                 color=RICE_WHITE, contact_offset=0.001, rest_offset=0.0)]
# The near-edge strip under the cucumber ends in a 31 deg ramp instead of a square 1.7 mm step, as the soft rice at a
# real sheet's edge does: wound round, its edge then slides under the mat instead of butting into a slat. Its flat
# part (x -1.0 to +3.8 mm) still carries the cucumber's weight; the ramp runs from there to the strip's top face at
# the near edge.
_RAMP_DEG = math.degrees(math.atan2(NORI_LINK[2], NORI_LINK[0] / 2 - 0.001))  # 31.3 deg
_RAMP_LEN = math.hypot(NORI_LINK[2], NORI_LINK[0] / 2 - 0.001)
_LEAD = [lk.box((NORI_LINK[0] / 2 + 0.001, NORI_LINK[1], NORI_SKIN), (0.0014, 0.0, (NORI_SKIN - NORI_LINK[2]) / 2),
                color=NORI_GREEN, contact_offset=0.001, rest_offset=0.0),
         lk.box((NORI_LINK[0] / 2 + 0.001, NORI_LINK[1], NORI_LINK[2] - NORI_SKIN), (0.0014, 0.0, NORI_SKIN / 2),
                color=RICE_WHITE, contact_offset=0.001, rest_offset=0.0),
         lk.box((_RAMP_LEN, NORI_LINK[1], 0.001),
                (-0.0024 + 0.0005 * math.sin(math.radians(_RAMP_DEG)), 0.0, 0.0005 * math.cos(math.radians(_RAMP_DEG))),
                rpy_deg=(0.0, _RAMP_DEG, 0.0), color=RICE_WHITE, contact_offset=0.001, rest_offset=0.0)]
for _i in range(N_NORI):
    _n.link(f"l{_i}", _LEAD if _i == TACK else _STRIP, 0.003, at=(LINK_X[_i] - CORE_X, 0.0, NORI_Z - CORE_Z0),
            friction=(1.0, 0.9), contact_offset=0.001)
_n.fixed_joint("tack", "core", f"l{TACK}")
for _i in range(1, N_NORI):  # hinge j{i} joins l{i-1} and l{i}; the strip nearer the cucumber is the parent
    _par, _ch = (f"l{_i - 1}", f"l{_i}") if _i > TACK else (f"l{_i}", f"l{_i - 1}")
    _n.revolute(f"j{_i}", _par, _ch, anchor=(LINK_X[_i] - NORI_PITCH / 2 - CORE_X, 0.0, NORI_Z - CORE_Z0),
                axis=(0, 1, 0), limits_deg=(-150, 150), stiffness=2e-5, damping=1e-5)
NORI = _n.build()

OBJECTS = [BOARD, MAT, NORI]

# -- scoring ------------------------------------------------------------------------------------------------------
HUG = (0.0125, 0.0175)  # m from the cucumber's axis: a strip lies on the roll (layer 1 at 13.85 mm, layer 2 15.55)
LINK_SPAN_DEG = math.degrees(NORI_PITCH / (CORE_R + NORI_LINK[2] / 2))  # one strip's own arc, 33.1 deg
RUNG_CURL, RUNG_TUCK, RUNG_WRAP = 180.0, 300.0, 330.0  # wrap now (deg) for the 0.25 / 0.5 / 0.75 rungs
MAT_CURL_DEG = 120.0  # the mat must have been curled this far before the wrap first reaches RUNG_CURL
SEAM_DOWN_DEG = 60.0
SQUARE_DEG = 10.0
OPEN_DEG = 20.0
OPEN_RISE = 0.010  # m: no slat centre more than this above flat
LIFT_GATE = 0.020  # m
TOUCH = 0.004  # m: a pad point this close to the sheet or the cucumber touches the food
CLEAR = 0.030  # m from every pad point of each gripper to the mat and the food
STILL = 0.010  # m/s
RELEASE_S = 1.0
BOARD_GATE_X = (0.25, 0.52)
CORE_Y_MAX = 0.06


def core_axis(nori) -> tuple[np.ndarray, np.ndarray]:
    """The cucumber's axis line: its centre and its local +y in the world."""
    m = nori.poses["core"]
    return m[:3, 3].copy(), m[:3, 1] / np.linalg.norm(m[:3, 1])


def nori_radii(nori) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per strip: distance from the cucumber's axis, offset along it, and the perpendicular offset vector."""
    o, a = core_axis(nori)
    r, along, perp = [], [], []
    for i in range(N_NORI):
        rel = nori.poses[f"l{i}"][:3, 3] - o
        s = float(rel @ a)
        p = rel - s * a
        r.append(float(np.linalg.norm(p)))
        along.append(s)
        perp.append(p)
    return np.array(r), np.array(along), np.array(perp)


def hugging(nori) -> np.ndarray:
    r, along, _ = nori_radii(nori)
    return (r >= HUG[0]) & (r <= HUG[1]) & (np.abs(along) <= CORE_LEN / 2)


def wrap_deg(nori) -> float:
    """Arc of the cucumber the sheet covers: winding of the longest run of consecutive hugging strips, plus one
    strip's own span."""
    hug = hugging(nori)
    runs, cur = [], []
    for i in range(N_NORI):
        if hug[i]:
            cur.append(i)
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    if not runs:
        return 0.0
    o, a = core_axis(nori)
    best = (-1, 0.0)
    for run in runs:
        pts = [nori.poses[f"l{i}"][:3, 3] for i in run]
        w = abs(math.degrees(lk.winding(pts, o, a))) + LINK_SPAN_DEG if len(run) > 1 else LINK_SPAN_DEG
        best = max(best, (len(run), w))
    return best[1]


def seam_angle_deg(nori) -> tuple[float, bool]:
    """Angle (deg) about the cucumber's axis between the sheet's outer end and straight down, and whether that end
    lies on the roll. The outer end is whichever end strip is farther from the axis."""
    r, along, perp = nori_radii(nori)
    i = 0 if r[0] >= r[N_NORI - 1] else N_NORI - 1
    _, a = core_axis(nori)
    down = np.array([0.0, 0.0, -1.0])
    down = down - (down @ a) * a
    if np.linalg.norm(down) < 1e-6 or r[i] < 1e-6:
        return 180.0, False
    c = float(perp[i] @ down / (np.linalg.norm(perp[i]) * np.linalg.norm(down)))
    on_roll = bool(HUG[0] <= r[i] <= HUG[1] and abs(along[i]) <= CORE_LEN / 2)
    return math.degrees(math.acos(max(-1.0, min(1.0, c)))), on_roll


def seam_down(nori) -> bool:
    ang, on_roll = seam_angle_deg(nori)
    return on_roll and ang <= SEAM_DOWN_DEG


def core_square(nori) -> bool:
    _, a = core_axis(nori)
    return abs(float(a[1])) >= math.cos(math.radians(SQUARE_DEG))


def mat_curl_deg(mat) -> float:
    """How far slat 0 is turned over relative to slat 23 (the sum of the mat's hinge angles), deg."""
    return abs(math.degrees(sum(mat.q.values())))


def mat_open(mat) -> bool:
    if max(abs(math.degrees(q)) for q in mat.q.values()) > OPEN_DEG:
        return False
    return max(float(mat.poses[f"l{i}"][2, 3]) for i in range(N_SLATS)) <= MAT_Z + OPEN_RISE


def core_on_mat(nori, mat) -> bool:
    x0, x1 = float(mat.poses["l0"][0, 3]), float(mat.poses[f"l{N_SLATS - 1}"][0, 3])
    p = nori.poses["core"][:3, 3]
    return min(x0, x1) <= float(p[0]) <= max(x0, x1) and abs(float(p[1])) <= CORE_Y_MAX


def off_board(nori) -> bool:
    p = nori.poses["core"][:3, 3]
    return not (BOARD_GATE_X[0] <= float(p[0]) <= BOARD_GATE_X[1]) or abs(float(p[1])) > CORE_Y_MAX


def touching_food(states: dict, tools: dict) -> bool:
    return any(lk.clearance(tool, states["nori"]) < TOUCH for tool in tools.values())


def clear_and_still(states: dict, tools: dict) -> bool:
    for tool in tools.values():
        if tool.speed >= STILL:
            return False
        for name in ("mat", "nori"):
            if lk.clearance(tool, states[name]) < CLEAR:
                return False
    roll_speed = float(np.linalg.norm(states["nori"].lin_vels["core"]))  # the roll as a whole (its 3 g strips
    return states["mat"].max_speed() < STILL and roll_speed < STILL     # chatter in contact without moving)


H = lk.History()  # whole-run record: updated every control step, cleared only by a simulator reset


def finite(states: dict) -> bool:
    return all(np.isfinite(m).all() for n in ("mat", "nori") for m in states[n].poses.values())


def update_history(hist: lk.History, states: dict, tools: dict) -> None:
    nori, mat = states["nori"], states["mat"]
    hist.ever("unstable", not finite(states))
    hist.peak("lift", float(nori.poses["core"][2, 3]) - CORE_Z0)
    hist.ever("off_board", off_board(nori))
    hist.ever("touch", touching_food(states, tools))
    hist.peak("curl", mat_curl_deg(mat))
    w = wrap_deg(nori)
    hist.peak("wrap", w)
    if w >= RUNG_CURL and "curl_at_half_wrap" not in hist.values:
        hist.set("curl_at_half_wrap", hist.peaks["curl"])
    hist.held("release", clear_and_still(states, tools))


def evaluate(states: dict, hist: lk.History) -> tuple[float, str]:
    """(score, reason). Gates first (whole-run records), then the ladder on the state now."""
    if "unstable" in hist.evers:
        return 0.0, f"the simulation went unstable (non-finite poses) at {hist.evers['unstable']:.2f} s"
    if hist.peaks.get("lift", 0.0) > LIFT_GATE:
        return 0.0, f"cucumber lifted {1000 * hist.peaks['lift']:.0f} mm"
    if "off_board" in hist.evers:
        return 0.0, f"cucumber left the block at {hist.evers['off_board']:.2f} s"
    if "touch" in hist.evers:
        return 0.0, f"a gripper touched the food at {hist.evers['touch']:.2f} s"
    curl = hist.values.get("curl_at_half_wrap")
    if curl is not None and curl < MAT_CURL_DEG:
        return 0.0, f"sheet half wrapped with the mat curled only {curl:.0f} deg (needs {MAT_CURL_DEG:.0f})"
    if hist.runs.get("release", 0.0) < RELEASE_S:
        return 0.0, f"grippers not clear and still for 1 s ({hist.runs.get('release', 0.0):.2f} s)"
    nori, mat = states["nori"], states["mat"]
    w = wrap_deg(nori)
    if w < RUNG_CURL:
        return 0.0, f"wrap {w:.0f} deg < {RUNG_CURL:.0f}"
    if w < RUNG_TUCK:
        return 0.25, f"wrap {w:.0f} deg < {RUNG_TUCK:.0f}"
    ang, on_roll = seam_angle_deg(nori)
    if not (w >= RUNG_WRAP and on_roll and ang <= SEAM_DOWN_DEG and core_square(nori)):
        return 0.5, f"wrap {w:.0f} deg, seam {ang:.0f} deg from down (on roll {on_roll}), square {core_square(nori)}"
    if not (mat_open(mat) and core_on_mat(nori, mat)):
        return 0.75, f"mat open {mat_open(mat)}, roll on mat {core_on_mat(nori, mat)}"
    return 1.0, "rolled seam down, mat open, released"


LAST = {"score": 0.0, "why": "", "t": 0.0}
DIAG: dict = {"traces": [], "seen": {}}


@lk.on_reset
def _reset(tick):
    LAST.update(score=0.0, why="", t=0.0)
    DIAG["traces"] = []
    DIAG["seen"] = {}


@lk.on_step
def _track(tick):
    s = tick.states
    update_history(H, s, tick.tools)
    if tick.n % 5 == 0:  # diagnostics every 0.25 s: link centres (x, z) in mm, for side-view plots
        row = {"t": round(tick.t, 2)}
        for name, n in (("mat", N_SLATS), ("nori", N_NORI)):
            row[name] = [[round(1000 * float(s[name].poses[f"l{i}"][k, 3]), 1) for k in (0, 2)] for i in range(n)]
        p = s["nori"].poses["core"]
        row["core"] = [round(1000 * float(p[0, 3]), 1), round(1000 * float(p[2, 3]), 1),
                       round(math.degrees(math.atan2(p[0, 2], p[0, 0])), 1)]
        row["wrap"] = round(wrap_deg(s["nori"]), 1)
        row["curl"] = round(mat_curl_deg(s["mat"]), 1)
        row["v"] = [round(1000 * s[n].max_speed(), 1) for n in ("mat", "nori")]  # mm/s
        row["tv"] = {k: [round(1000 * t.speed, 1), round(1000 * min(lk.clearance(t, s[n]) for n in ("mat", "nori")), 1)]
                     for k, t in tick.tools.items()}
        row["rel"] = round(H.runs.get("release", 0.0), 2)
        row["tcp"] = {k: [round(1000 * float(v), 1) for v in t.tcp] for k, t in tick.tools.items()}
        DIAG["traces"].append(row)


def score(objects) -> float:
    sc, why = evaluate(objects, H)
    LAST.update(score=sc, why=why, t=H.t)
    return sc


def success(objects) -> bool:
    return evaluate(objects, H)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - evaluate(objects, H)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo (linkdemo.py): closed loop on camera-observable poses only ------------------------------------
VARIANTS = ("", "nothing", "push", "hand", "drag", "lift", "hold")
OPTS_DEFAULT = {"fold_s": 5.0, "slack": 0.0, "slack_end": 0.004, "gap": 0.0, "v_log": 0.008, "roll_lead": 0.003,
                "v_slow": 0.004, "slow_zone": 0.010, "roll_lag": 1.0,
                "open_s": 7.0, "open_slack": 0.004, "flat_pull": 0.0005, "stop": 0.004, "roll_slack": 0.0, "raise": 0.010}
CORE_X_MAX = 0.45  # the demo never rolls the cucumber past here (the sheet ends at 0.435, the block at 0.525)


def _opts(variant: str) -> tuple[str, dict]:
    name, _, rest = variant.partition(":")
    if "=" in name:
        name, rest = "", variant
    o = dict(OPTS_DEFAULT)
    o.update({k: float(v) for k, v in (kv.split("=") for kv in rest.split(",") if "=" in kv)})
    return name, o


def _ry(deg: float) -> np.ndarray:
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


GRASP_DX = -0.0095  # TCP 9.5 mm in front of slat 0's centre: the pads cover slat 0 and half of slat 1
GRASP_Y = 0.116  # TCP 21 mm outside the block's side: pad tips 8 mm clear of the block and of the sheet's edge


def _grip_rot(side: str) -> np.ndarray:
    """Side pinch of a slat end: approach along -y (left arm) or +y (right arm), pads closing across the slat's
    thickness (world z when the slat is flat). The sign is the one that leaves the wrist room to turn 180 deg."""
    s = 1.0 if side == "left" else -1.0
    z = np.array([0.0, -s, 0.0])
    x = np.array([0.0, 0.0, -s])
    return np.column_stack([x, np.cross(z, x), z])


def _tcp(side: str, c, th: float):
    """TCP pose that holds slat 0 with its centre at ``c`` turned ``th`` deg about +y."""
    s = 1.0 if side == "left" else -1.0
    r = _ry(th)
    return np.asarray(c, float) + r @ np.array([GRASP_DX, s * GRASP_Y, 0.0]), r @ _grip_rot(side)


def _hold_slat(ctx, c, th, width: float = 0.0) -> None:
    for side in ("left", "right"):
        p, r = _tcp(side, c, th)
        ctx.arm(side)._solve(p, r, width)


def _move_slat(ctx, c0, th0, c1, th1, duration: float, width: float = 0.0):
    import linkdemo
    n = max(1, round(duration * linkdemo.CONTROL_HZ))
    c0, c1 = np.asarray(c0, float), np.asarray(c1, float)
    for k in range(1, n + 1):
        e = linkdemo.smooth(k / n)
        _hold_slat(ctx, c0 + e * (c1 - c0), th0 + e * (th1 - th0), width)
        yield


def _slat0(seen) -> tuple[np.ndarray, float]:
    m = seen.pose("mat", "l0")
    return m[:3, 3].copy(), math.degrees(math.atan2(m[0, 2], m[0, 0]))


def _core(seen) -> np.ndarray:
    """The cucumber's centre (its end faces stay in view from the side: the mat curls round it, not over its ends)."""
    return seen.point("nori", "core")


def _ik(arm, pos, rot, seed):
    from yam_arena import ik

    return ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(seed, float), base_pos=arm.base,
                    iters=200)


def _joint_move(arm, q1, pos, rot, width: float, duration: float):
    """Free-space transit: interpolate the joints (not the TCP) to ``q1``, a solved pose for ``pos``/``rot``."""
    import linkdemo
    q0, w0 = arm.q.copy(), arm.width
    n = max(1, round(duration * linkdemo.CONTROL_HZ))
    for k in range(1, n + 1):
        e = linkdemo.smooth(k / n)
        w = w0 + e * (width - w0)
        arm.cmd = np.r_[q0 + e * (q1 - q0), float(np.clip(w / linkdemo.OPEN_WIDTH, 0.0, 1.0))]
        yield
    arm.q, arm.pos, arm.rot, arm.width = np.asarray(q1, float).copy(), np.asarray(pos, float), rot, width


BACK, UP = 0.04, 0.04  # pre-grasp: pads level with the slat, 40 mm toward the robot from the grasp, 40 mm higher


def _pinch_slat0(ctx):
    """Both arms take slat 0's overhanging ends in a side pinch, coming in from the robot side. Reads slat 0's
    observed pose."""
    from yam_arena.constants import READY
    seen = ctx.observe()
    c, th = _slat0(seen)
    DIAG["seen"]["slat0_start"] = [round(1000 * v, 1) for v in c]

    def arm(side):
        a = ctx.arm(side)
        p, r = _tcp(side, c, th)
        back = r[:, 1] * BACK  # tool +y is the slat's own +x (toward slat 1): back off along it
        q = _ik(a, p, r, READY)[0]  # the pinch, then walk the pre-grasp path backwards on the same IK branch
        for k in range(1, 11):
            q = _ik(a, p - back * k / 10, r, q)[0]
        for k in range(1, 11):
            q = _ik(a, p - back + np.array([0.0, 0.0, UP * k / 10]), r, q)[0]
        yield from _joint_move(a, q, p - back + [0.0, 0.0, UP], r, 0.022, 2.5)
        yield from a.move(p - back, r, duration=1.0)
        yield from a.move(p, r, duration=1.2)
        yield from a.grip(0.0, 0.8)
    yield from ctx.both(arm("left"), arm("right"))
    yield from ctx.wait(0.3)
    return c, th


def _release_and_clear(ctx, width: float = 0.05, slide: float = 0.05):
    """Open the pads while sliding them off slat 0's free edge in the slat's own plane, then rise clear."""
    def arm(side):
        a = ctx.arm(side)
        out = -a.rot[:, 1] * slide  # tool -y: away from slat 1, off the free edge
        yield from a.move(a.pos, width=0.012, duration=0.3)
        yield from a.move(a.pos + out, width=width, duration=1.0)
        yield from a.move(a.pos + np.array([0.0, 0.0, 0.05]), duration=0.8)
    yield from ctx.both(arm("left"), arm("right"))


def _to_ready(ctx, duration: float = 2.5):
    from yam_arena import ik
    from yam_arena.constants import READY

    def arm(side):
        a = ctx.arm(side)
        p, r = ik.tcp_pose(np.array(READY, float), a.base)
        yield from _joint_move(a, np.array(READY, float), p, r, a.width, duration)
    yield from ctx.both(arm("left"), arm("right"))


def _rolled_out(seen, far_x: float, o) -> bool:
    """The cucumber has rolled up to the sheet's far edge (seen flat on the mat before the roll began), so the
    whole sheet is round it with that edge underneath."""
    return float(_core(seen)[0]) >= far_x - o["stop"]


def _fold(ctx, c0, o, far_x):
    """Curl the mat's near edge up round the back of the cucumber and over its top: a tight involute about where the
    cucumber was seen before the curl. Held at both ends (slat 0 by the arms, the rest under the cucumber), the curl
    clamps it."""
    import linkdemo
    lc = _core(ctx.observe())
    rm = CORE_R + NORI_LINK[2] + SLAT[2] / 2 + o["gap"]  # mat mid-plane radius with the sheet between mat and roll
    DIAG["seen"]["core_before_fold"] = [round(1000 * v, 1) for v in lc]
    n = max(1, round(o["fold_s"] * linkdemo.CONTROL_HZ))
    c = np.asarray(c0, float)
    for k in range(1, n + 1):
        if _rolled_out(ctx.observe(), far_x, o):
            break
        phi = math.pi * linkdemo.smooth(k / n)
        q = np.array([lc[0] - rm * math.sin(phi), 0.0, lc[2] - rm * math.cos(phi)])
        t = np.array([-math.cos(phi), 0.0, math.sin(phi)])
        slack = o["slack"] + (o["slack_end"] - o["slack"]) * phi / math.pi
        rest = max(float(lc[0] - c0[0]) - slack - rm * phi, 0.004)
        c = q + rest * t
        c[2] = max(c[2], c0[2])  # at the very start the slat is still on the block
        _hold_slat(ctx, c, math.degrees(phi))
        yield
    now = _core(ctx.observe())
    DIAG["seen"]["fold_end"] = {"core": [round(1000 * v, 1) for v in now], "slat0_cmd": [round(1000 * v, 1) for v in c]}
    return c, float(lc[0])


def _roll(ctx, c0, ref, o, far_x, max_s: float = 16.0):
    """Roll the curled mat forward with the cucumber in it: keep the curl tight round the back of the roll and over
    its top, with the top layer rising forward from there to slat 0, about a reference that advances at ``v_log``
    (``v_slow`` over the last 10 mm) and never runs more than ``roll_lead`` ahead of the cucumber as seen, so slat 0
    moves at twice the roll's speed. It never chases a roll that runs ahead (``roll_lag`` 1 m): slat 0 is commanded
    at twice the roll's position, so a chasing feed pushes a moving roll on and runs away with it. The curl's length
    follows the roll: the mat wraps bare cucumber at first and the wound sheet as the roll proceeds. Stops when the
    roll reaches the sheet's far edge."""
    import linkdemo
    lc = _core(ctx.observe())
    x_start = float(lc[0])
    r_bare = CORE_R + SLAT[2] / 2 + o["gap"]  # mat mid-plane radius on bare cucumber
    r_cov = r_bare + NORI_LINK[2]  # and on the wound sheet
    dt = 1.0 / linkdemo.CONTROL_HZ
    DIAG["seen"]["roll"] = {"core_x0": round(1000 * ref, 1), "far_x": round(1000 * far_x, 1)}
    c = None
    for k in range(1, round(max_s * linkdemo.CONTROL_HZ) + 1):
        seen = ctx.observe()
        lc = _core(seen)
        if _rolled_out(seen, far_x, o) or lc[0] >= CORE_X_MAX:
            break
        x = float(lc[0])
        v = o["v_log"] if x < far_x - o["stop"] - o["slow_zone"] else o["v_slow"]  # slower over the last 10 mm
        ref = max(ref, min(max(ref + v * dt, x - o["roll_lag"]), x + o["roll_lead"]))
        cov = min(max(x - x_start, 0.0) / (CORE_R + NORI_LINK[2] / 2), math.pi)  # wound arc up the roll's back
        curl = cov * r_cov + (math.pi - cov) * r_bare  # mat length round the back, bottom to top
        e = min(1.0, k / linkdemo.CONTROL_HZ)  # over the first second: take up the slack, raise the mat's edge
        slack = o["slack_end"] + e * (o["roll_slack"] - o["slack_end"])
        rest = max(ref - float(c0[0]) - slack - curl, 0.004)
        # the top layer rises from the top of the roll to slat 0, so the mat leaves the roll there and is never
        # dragged down its front
        c = np.array([ref + rest, 0.0, float(lc[2]) + r_cov + e * o["raise"]])
        _hold_slat(ctx, c, 180.0)
        yield
    if c is None:
        c = np.array([float(lc[0]) + 0.01, 0.0, float(lc[2]) + r_cov])
    DIAG["seen"]["roll_end"] = {"core_x": round(1000 * float(_core(ctx.observe())[0]), 1), "ref": round(1000 * ref, 1),
                                "slat0": [round(1000 * v, 1) for v in c]}
    return c, 180.0


def _open_mat(ctx, c, th, o, release: bool = True):
    """Peel the mat back off the roll and lay it flat where it started: the fold run backwards, an involute that
    unwraps the curl from the roll as it now lies, so the mat lifts off the roll instead of sliding over it (a strap
    swung over the roll drags it back and unwinds the sheet). Slat 0's flat place is slat 23's observed position
    minus the mat's 230 mm (the slats under and ahead of the roll never move)."""
    import linkdemo
    seen = ctx.observe()
    lc = _core(seen)
    x0 = float(seen.point("mat", f"l{N_SLATS - 1}")[0]) - (N_SLATS - 1) * PITCH  # slat 0's flat place
    rm = CORE_R + NORI_LINK[2] + SLAT[2] / 2 + o["gap"]

    # start taut: a slack mat would slide over the roll and drag it back. Taking up the slack the feed left rolls
    # the roll on a few mm, which the seam-down window allows.
    slack0 = o["open_slack"]

    def at(phi):
        q = np.array([lc[0] - rm * math.sin(phi), 0.0, lc[2] - rm * math.cos(phi)])
        t = np.array([-math.cos(phi), 0.0, math.sin(phi)])
        rest = max(float(lc[0]) - x0 - slack0 * phi / math.pi - rm * phi, 0.004)
        p = q + rest * t
        p[2] = max(p[2], MAT_Z)
        return p

    DIAG["seen"]["open"] = {"core": [round(1000 * v, 1) for v in lc], "flat_x0": round(1000 * x0, 1),
                            "slack0": round(1000 * slack0, 1),
                            "start": [round(1000 * v, 1) for v in at(math.pi)]}
    yield from _move_slat(ctx, c, th, at(math.pi), 180.0, 1.0)  # drop the raise, take up the involute's start
    n = max(1, round(o["open_s"] * linkdemo.CONTROL_HZ))
    for k in range(1, n + 1):
        phi = math.pi * (1.0 - linkdemo.smooth(k / n))
        _hold_slat(ctx, at(phi), math.degrees(phi))
        yield
    # the peel can shift the mat under the roll a few mm: lay slat 0 at the flat place seen now, so no hinge is
    # left tented up
    seen = ctx.observe()
    x1 = float(seen.point("mat", f"l{N_SLATS - 1}")[0]) - (N_SLATS - 1) * PITCH + o["flat_pull"]
    DIAG["seen"]["open"]["flat_x1"] = round(1000 * x1, 1)
    yield from _move_slat(ctx, at(0.0), 0.0, np.array([x1, 0.0, MAT_Z]), 0.0, 0.8)
    yield from ctx.wait(0.4)
    if release:
        yield from _release_and_clear(ctx)


def _finger_push(ctx, sides, dist: float, dz: float, y: float):
    """Cheat helper: closed fingers come down behind the cucumber and push it ``dist`` forward by hand."""
    from linkdemo import down
    lc = _core(ctx.observe())

    def arm(side):
        a = ctx.arm(side)
        s = 1.0 if side == "left" else -1.0
        tip = np.array([lc[0] - CORE_R - 0.012, s * y, lc[2] + dz])
        yield from a.grip(0.0, 0.5)
        yield from a.move(tip + [0, 0, 0.08 + 0.013], down(90), duration=2.5)
        yield from a.move(tip + [0, 0, 0.013], duration=1.5)
        yield from a.move(tip + [dist, 0, 0.013], duration=max(3.0, dist / 0.01))
        yield from a.move(a.pos + [0, 0, 0.08], duration=1.0)
    yield from ctx.both(*(arm(s) for s in sides))


def demo_script(ctx):
    name, o = _opts(ctx.variant)
    DIAG["seen"]["variant"] = ctx.variant
    if name == "nothing":
        yield from ctx.wait(8.0)
        return
    if name in ("push", "hand"):  # push: one hand rolls the cucumber 40 mm; hand: both roll it the whole sheet
        if name == "push":
            yield from _finger_push(ctx, ("right",), 0.040, 0.0, 0.03)
        else:
            yield from _finger_push(ctx, ("left", "right"), 0.085, 0.008, 0.05)
        yield from _to_ready(ctx)
        yield from ctx.wait(2.0)
        return
    if name == "drag":  # pinch the mat edge and slide the whole mat 50 mm toward the robot, roll and all
        c, th = yield from _pinch_slat0(ctx)
        yield from _move_slat(ctx, c, th, c + [-0.050, 0.0, 0.0], th, 3.0)
        yield from _release_and_clear(ctx)
        yield from _to_ready(ctx)
        yield from ctx.wait(2.0)
        return
    if name == "lift":  # pick the cucumber up 30 mm and lay it back where it was
        from linkdemo import down
        lc = _core(ctx.observe())

        def arm(side):
            a = ctx.arm(side)
            s = 1.0 if side == "left" else -1.0
            p = np.array([lc[0], s * 0.05, lc[2] + 0.004])
            yield from a.move(p + [0, 0, 0.08], down(90.0), width=0.05, duration=2.5)
            yield from a.move(p, duration=1.5)
            yield from a.grip(0.0, 0.8)
            yield from a.move(p + [0, 0, 0.030], duration=1.5)
            yield from a.wait(1.0)
            yield from a.move(p, duration=1.5)
            yield from a.grip(0.05, 0.6)
            yield from a.move(p + [0, 0, 0.08], duration=1.0)
        yield from ctx.both(arm("left"), arm("right"))
        yield from _to_ready(ctx)
        yield from ctx.wait(2.0)
        return
    # the demo (and "hold": the same, then both arms take the mat edge again and keep holding it)
    seen = ctx.observe()
    far_x = float(seen.point("nori", f"l{N_NORI - 1}", (NORI_LINK[0] / 2, 0.0, 0.0))[0])  # flat, in view
    DIAG["seen"]["far_edge_x"] = round(1000 * far_x, 1)
    c0, th0 = yield from _pinch_slat0(ctx)
    c, ref = yield from _fold(ctx, c0, o, far_x)
    c, th = yield from _roll(ctx, c0, ref, o, far_x)
    yield from ctx.wait(0.4)
    yield from _open_mat(ctx, c, th, o)
    yield from _to_ready(ctx)
    yield from ctx.wait(1.5)
    if name == "hold":
        yield from _pinch_slat0(ctx)
        yield from ctx.wait(4.0)
        return
    yield from ctx.wait(2.0)


def demo_report() -> dict:
    return {"score": LAST.get("score"), "why": LAST.get("why", ""), "history": H.summary(), "seen": DIAG["seen"],
            "traces": DIAG["traces"]}
