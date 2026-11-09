# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Lay a curled site plan flat on a plan table and weight both of its ends.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the station table top is at z 0.7475.

- Plan table: a white melamine board, 24 x 52 x 1.8 cm, fixed on the station table, centred at (0.18, 0.0); its top
  is at z 0.7655. Grey tape (5 mm wide) outlines an 18 x 40 cm rectangle centred at (0.18, 0.0), long side along y.
- Site plan: a laminated drawing, 18 cm wide (along x) by 40 cm long (along y), white with blue linework, 48 g. It
  was stored rolled and keeps its curl: laid on the board it rests on its middle 15 cm with both ends rolled up and
  inward to about 8 cm high (natural curl radius 72 mm). Modelled as 16 rigid 25 mm slats joined by sprung hinges.
- Two steel paperweights, 16 x 2.5 x 1.5 cm, 470 g each, dark grey with a green felt base, lie on the board along x
  beside the plan's ends, centred at (0.18, +0.235) (left) and (0.18, -0.235) (right).

Goal: the plan flat on the board inside the outline, a weight on each end, both hands clear.

Why it is hard: the curl springs back wherever the plan is not held. A weight holds flat only what lies between it
and another weight, so the free end beyond a weight curls up again: each weight must end on the plan's last 25 mm,
and the middle must stay pinned while the first half is opened. Pressing a hand-flattened plan and letting go, one
weight in the middle, or weights a few centimetres in from the ends all leave curled ends.
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
TOP = TABLE_TOP[SCENE]  # station table top, 0.7475
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

# -- scene ---------------------------------------------------------------------------------------------------------

BOARD = (0.24, 0.52, 0.018)  # plan table: x, y, thickness
CENTRE = np.array([0.18, 0.0])  # outline centre (and board centre)
PLAN_TOP = TOP + BOARD[2]  # 0.7655
OUTLINE = (0.18, 0.40)
TAPE_W, TAPE_T = 0.005, 0.0003

N = 16  # slats
PITCH = 0.025
SLAT = (0.18, PITCH, 0.0015)  # across the sheet (x), along it (y), thickness
SLAT_MASS = 0.003
REST_DEG = 20.0  # sprung hinge rest angle: 72 mm curl radius
STIFFNESS = 0.05  # N*m/rad per hinge (laminated A3 sheet: bending stiffness about 1.3e-3 N*m^2 over its width)
DAMPING = 0.0008
LIMITS_DEG = (-30.0, 60.0)
# hinge angles j1..j15 at rest on a flat board (2D static solve of this chain under gravity): the middle lies flat,
# the ends curl up about 9 cm
INIT_DEG = (20.1, 19.9, 18.8, 16.6, 13.2, 8.6, 3.2, 0.0, 3.2, 8.6, 13.2, 16.6, 18.8, 19.9, 20.1)
ROOT = 7  # slat s7 is the articulation root

W_SIZE = (0.16, 0.025, 0.015)  # paperweight: x, y, height
W_MASS = 0.47
FELT_T = 0.002
WEIGHTS = ("weight_L", "weight_R")
W_START = {"weight_L": (0.18, 0.235), "weight_R": (0.18, -0.235)}

WHITE = (0.96, 0.96, 0.94)
LINEWORK = (0.80, 0.87, 0.98)
STEEL = (0.30, 0.30, 0.32)
FELT = (0.16, 0.32, 0.20)
TAPE = (0.55, 0.55, 0.55)

# plan table: one fixed link, the tape strips 0.3 mm proud just outside the outline
_t = lk.Tree("plan_table", pos=(CENTRE[0], CENTRE[1], TOP), fixed=True, color=WHITE)
_ox, _oy = OUTLINE[0] / 2 + TAPE_W / 2, OUTLINE[1] / 2 + TAPE_W / 2
_tz = BOARD[2] + TAPE_T / 2
_t.link("board", [lk.box(BOARD, (0.0, 0.0, BOARD[2] / 2)),
                  lk.box((TAPE_W, OUTLINE[1] + 2 * TAPE_W, TAPE_T), (-_ox, 0.0, _tz), color=TAPE),
                  lk.box((TAPE_W, OUTLINE[1] + 2 * TAPE_W, TAPE_T), (_ox, 0.0, _tz), color=TAPE),
                  lk.box((OUTLINE[0], TAPE_W, TAPE_T), (0.0, -_oy, _tz), color=TAPE),
                  lk.box((OUTLINE[0], TAPE_W, TAPE_T), (0.0, _oy, _tz), color=TAPE)],
        mass=5.0, friction=(0.45, 0.40))
PLAN_TABLE = _t.build()


def slat_y(i: int) -> float:
    """Design (flat) y of slat i's centre in the plan's frame (the plan's middle is 0)."""
    return (i - (N - 1) / 2) * PITCH


# the plan: s0 at the right end (-y) ... s15 at the left end (+y); hinge j_i joins s(i-1) and s(i). From the root s7
# outward every hinge is a curl-up spring (positive = the far slat rises), so its axis is +x on the left half and
# -x on the right half.
# The root slat sits at the linkage origin (Isaac Lab writes the root body's pose to the linkage pos at reset).
_t = lk.Tree("plan", pos=(CENTRE[0], CENTRE[1] + slat_y(ROOT), PLAN_TOP + SLAT[2] / 2 + 0.0002), color=WHITE)
_order = [ROOT] + [i for i in range(N) if i != ROOT]
for _i in _order:
    _t.link(f"s{_i}", [lk.box(SLAT)], mass=SLAT_MASS, at=(0.0, slat_y(_i) - slat_y(ROOT), 0.0),
            color=LINEWORK if _i % 2 else WHITE, friction=(0.35, 0.30), contact_offset=0.001)
for _i in range(1, N):
    _left = _i > ROOT  # hinge on the left (+y) half
    _t.revolute(f"j{_i}", f"s{_i - 1}" if _left else f"s{_i}", f"s{_i}" if _left else f"s{_i - 1}",
                anchor=(0.0, (_i - N / 2) * PITCH - slat_y(ROOT), 0.0), axis=(1.0, 0.0, 0.0) if _left else (-1.0, 0.0, 0.0),
                limits_deg=LIMITS_DEG, rest_deg=REST_DEG, init_deg=INIT_DEG[_i - 1], stiffness=STIFFNESS,
                damping=DAMPING)
PLAN = _t.build()


def _weight(name: str) -> lk.Linkage:
    x, y = W_START[name]
    t = lk.Tree(name, pos=(x, y, PLAN_TOP + W_SIZE[2] / 2 + 0.0002), color=STEEL)
    t.link("bar", [lk.box((W_SIZE[0], W_SIZE[1], FELT_T), (0.0, 0.0, (FELT_T - W_SIZE[2]) / 2), color=FELT),
                   lk.box((W_SIZE[0], W_SIZE[1], W_SIZE[2] - FELT_T), (0.0, 0.0, FELT_T / 2))],
           mass=W_MASS, friction=(0.25, 0.22))
    return t.build()


WEIGHT_L, WEIGHT_R = _weight("weight_L"), _weight("weight_R")
OBJECTS = [PLAN_TABLE, PLAN, WEIGHT_L, WEIGHT_R]

# -- scoring -------------------------------------------------------------------------------------------------------

FLAT_DZ = 0.004  # m: slat centre from the board top + half its thickness
FLAT_TILT = 6.0  # deg
COVER = 0.44  # share of an end slat's area under a weight's footprint: half the bar's 25 mm width on that slat
SEAT = 0.003  # m: weight bottom from that slat's top face
CENTRE_TOL = 0.015  # m: plan centre from the outline centre
AXIS_TOL = 5.0  # deg: plan's long axis from y
CLEAR = 0.030  # m: every pad point from the plan and both weights
STILL_OBJ = 0.003  # m/s: every slat and weight, from poses at the 20 Hz control rate
STILL_TCP = 0.005  # m/s: both grippers
FLAT_FREE_S = 3.0
RELEASE_S = 1.0
DROP = 0.005  # m: a weight's lowest point this far below the station-table top -> it fell off the station table
EPS = 1e-6
END_SLATS = {"right": 0, "left": N - 1}
HALVES = {"right": range(0, N // 2), "left": range(N // 2, N)}


def _area(p: np.ndarray) -> float:
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _ccw(p: np.ndarray) -> np.ndarray:
    return p if _area(p) >= 0 else p[::-1]


def _clip(subject: np.ndarray, clipper: np.ndarray) -> np.ndarray:
    """Sutherland-Hodgman: the part of convex polygon ``subject`` inside convex polygon ``clipper`` (both CCW)."""
    out = [np.asarray(v, dtype=float) for v in subject]
    for k in range(len(clipper)):
        a, b = np.asarray(clipper[k], float), np.asarray(clipper[(k + 1) % len(clipper)], float)
        d = b - a

        def inside(p, a=a, d=d):
            return d[0] * (p[1] - a[1]) - d[1] * (p[0] - a[0]) >= -1e-15

        def cross(p, q, a=a, d=d):
            e = q - p
            den = e[0] * d[1] - e[1] * d[0]
            t = ((a[0] - p[0]) * d[1] - (a[1] - p[1]) * d[0]) / den if abs(den) > 1e-18 else 0.0
            return p + t * e

        inp, out = out, []
        if not inp:
            break
        s = inp[-1]
        for e in inp:
            if inside(e):
                if not inside(s):
                    out.append(cross(s, e))
                out.append(e)
            elif inside(s):
                out.append(cross(s, e))
            s = e
    return np.array(out) if out else np.zeros((0, 2))


def overlap_area(p, q) -> float:
    """Area (m^2) shared by two convex polygons given as (n, 2) xy arrays."""
    c = _clip(_ccw(np.asarray(p, float)), _ccw(np.asarray(q, float)))
    return abs(_area(c)) if len(c) >= 3 else 0.0


def footprint(m: np.ndarray, sx: float, sy: float) -> np.ndarray:
    """xy corners of an sx-by-sy rectangle centred on pose ``m``'s origin, in its x-y plane."""
    pts = [m[:3, :3] @ np.array([u * sx / 2, v * sy / 2, 0.0]) + m[:3, 3] for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return np.array(pts)[:, :2]


def lowest(m: np.ndarray, size) -> float:
    """Lowest z (m) of a box of edges ``size`` centred on pose ``m``."""
    h = np.asarray(size, float) / 2
    corners = np.array(np.meshgrid(*[[-1, 1]] * 3)).T.reshape(-1, 3) * h
    return float(((m[:3, :3] @ corners.T).T + m[:3, 3])[:, 2].min())


def measure(slats, weights: dict) -> dict:
    """Everything the ladder needs from the plan's 16 slat poses (s0..s15, 4x4) and the weights' poses (4x4)."""
    slats = [np.asarray(m, float) for m in slats]
    z0 = PLAN_TOP + SLAT[2] / 2
    dz = [float(m[2, 3] - z0) for m in slats]
    tilt = [math.degrees(math.acos(float(np.clip(m[2, 2], -1.0, 1.0)))) for m in slats]
    flat = [abs(d) <= FLAT_DZ and t < FLAT_TILT for d, t in zip(dz, tilt, strict=True)]
    run = best = 0
    for f in flat:
        run = run + 1 if f else 0
        best = max(best, run)
    centre = np.mean([m[:2, 3] for m in slats], axis=0)
    axis = slats[-1][:2, 3] - slats[0][:2, 3]
    axis_deg = math.degrees(math.atan2(abs(float(axis[0])), abs(float(axis[1])))) if np.linalg.norm(axis) > 0 else 90.0
    slat_area = SLAT[0] * SLAT[1]
    cover, seat, weighted = {}, {}, {}
    for end, i in END_SLATS.items():
        m = slats[i]
        top = float(m[2, 3] + m[2, 2] * SLAT[2] / 2)
        foot = footprint(m, SLAT[0], SLAT[1])
        best_w = None
        for w, mw in weights.items():
            mw = np.asarray(mw, float)
            c = overlap_area(footprint(mw, W_SIZE[0], W_SIZE[1]), foot) / slat_area
            s = abs(lowest(mw, W_SIZE) - top)
            ok = c >= COVER and s <= SEAT
            if best_w is None or (ok, c) > (best_w[2], best_w[0]):
                best_w = (c, s, ok)
        cover[end], seat[end], weighted[end] = best_w if best_w else (0.0, math.inf, False)
    half_flat = {e: all(flat[i] for i in HALVES[e]) for e in END_SLATS}  # one half flat out to its end slat
    one_end = {e: weighted[e] and half_flat[e] for e in END_SLATS}
    return {
        "dz": dz, "tilt": tilt, "flat_each": flat, "n_flat": int(sum(flat)), "flat_run": best, "flat": all(flat),
        "centre": centre, "centre_err": float(np.linalg.norm(centre - CENTRE)), "axis_deg": axis_deg,
        "centred": float(np.linalg.norm(centre - CENTRE)) <= CENTRE_TOL and axis_deg <= AXIS_TOL,
        "cover": cover, "seat": seat, "weighted": weighted, "both_ends": all(weighted.values()),
        "half_flat": any(half_flat.values()), "one_end": any(one_end.values()),
        "weight_low": min((lowest(np.asarray(mw, float), W_SIZE) for mw in weights.values()), default=math.inf),
    }


def update(h: lk.History, m: dict, clear: dict, tcp_speed: dict, obj_speed: float) -> None:
    """Feed one control step into the whole-run record ``h`` (call ``h._tick(t)`` first, as linkkit does)."""
    h.set("m", m)
    h.set("clear", dict(clear))
    h.ever("weight_off", m["weight_low"] < TOP - DROP)  # setting a weight down on the station table is allowed
    h.ever("half_open", m["half_flat"])
    h.ever("one_end", m["one_end"])
    free = bool(clear) and min(clear.values()) >= CLEAR
    h.held("released", free and bool(tcp_speed) and max(tcp_speed.values()) < STILL_TCP)
    h.held("flat_free", m["flat"] and m["both_ends"] and free and obj_speed < STILL_OBJ)


def ladder(h: lk.History) -> tuple[float, str]:
    """Score and reason. The gate is checked first, over the whole run; then the rungs, top down."""
    m = h.values.get("m")
    if m is None:
        return 0.0, "no steps yet"
    if h.when("weight_off") is not None:
        return 0.0, f"a weight fell off the station table (at {h.when('weight_off'):.2f} s)"
    ff, rel = h.runs.get("flat_free", 0.0), h.runs.get("released", 0.0)
    if ff >= FLAT_FREE_S - EPS and rel >= RELEASE_S - EPS and m["centred"]:
        return 1.0, "flat, both ends weighted, centred, hands clear and still"
    if m["flat"] and m["both_ends"]:
        why = "not centred" if not m["centred"] else f"flat and free {ff:.2f} s, released {rel:.2f} s"
        return 0.75, f"flat with both ends weighted ({why})"
    if h.when("one_end") is not None:
        return 0.5, f"one half flat with its end weighted at {h.when('one_end'):.2f} s"
    if h.when("half_open") is not None:
        return 0.25, f"one half flat out to its end at {h.when('half_open'):.2f} s"
    return 0.0, f"longest flat run {m['flat_run']} slats"


H = lk.History()
TRACE: list = []
PREV: dict = {}  # last control step's body centres, for pose_speed; cleared only by the simulator reset


def pose_speed(prev: dict | None, now: dict, dt: float) -> float:
    """Fastest body (m/s), from its centre's displacement over one control step: what a 20 Hz camera sees.
    (The thin slats chatter in contact at sub-millimetre amplitude, a solver artefact that instantaneous
    body velocities would report as tens of mm/s.) No previous step: infinitely fast, so never still."""
    if not prev or dt <= 0:
        return math.inf
    return max(float(np.linalg.norm(np.asarray(now[k]) - np.asarray(prev[k]))) for k in now) / dt


def slat_poses(state: lk.LinkState) -> list:
    return [state.poses[f"s{i}"] for i in range(N)]


@lk.on_reset
def _reset(tick):
    TRACE.clear()
    PREV.clear()


@lk.on_step
def _track(tick):
    st = tick.states
    plan = st["plan"]
    weights = {w: st[w].poses["bar"] for w in WEIGHTS}
    m = measure(slat_poses(plan), weights)
    tools = tick.tools
    clear = {side: min(lk.clearance(tool, st[n]) for n in ("plan", *WEIGHTS)) for side, tool in tools.items()}
    now = {f"s{i}": plan.poses[f"s{i}"][:3, 3].copy() for i in range(N)}
    now.update({w: weights[w][:3, 3].copy() for w in WEIGHTS})
    obj_speed = pose_speed(PREV.get("p"), now, tick.t - PREV.get("t", tick.t))
    PREV.update(p=now, t=tick.t)
    update(H, m, clear, {s: t.speed for s, t in tools.items()}, obj_speed)
    if tick.n % 10 == 0:
        TRACE.append((round(tick.t, 1), m["flat_run"], m["n_flat"], round(m["cover"]["right"], 2),
                      round(m["cover"]["left"], 2), round(1000 * m["centre"][1], 1),
                      round(1000 * min(clear.values()), 0), round(1000 * obj_speed, 1), ladder(H)[0],
                      [round(1000 * float(weights[w][k, 3]), 0) for w in WEIGHTS for k in (1, 2)]))


def score(objects) -> float:
    return ladder(H)[0]


def success(objects) -> bool:
    return ladder(H)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(H)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo (python linkdemo.py task.py OUT [--variant NAME]); reads only camera-observable poses ----------

VARIANTS = ("", "nothing", "weights_on_tube", "middle_weight_only", "five_cm_in", "hold", "release_left_early")
OPEN_W = 0.06  # pad opening to approach and release a 25 mm bar
GRIP_W = 0.010  # commanded opening when gripping (the fingers stop on the bar, force-limited)
PAD_UP = 0.003  # pad tips this far above the bar's bottom
HOVER = 0.08
LIFT = 0.07
PRESS = 0.0015  # command the bar this far below its resting height while it lies on the plan
Y_SET = {"weight_R": -0.030, "weight_L": 0.075}  # where each bar is first set down, from the plan's observed middle
WALK_SPEED = 0.025  # m/s while a bar is walked out over the curl
PARK = {"left": (0.12, 0.30, 0.92), "right": (0.12, -0.30, 0.92)}
CARRY_UP = 0.05  # carry a bar with its bottom this far above the plan's highest point
END_SHIFT = {"five_cm_in": 0.05}
RUN_S = 60.0  # the run limit (1,200 steps at 20 Hz): cheat variants idle to it
LOG: dict = {}


def _yaw_deg(m: np.ndarray) -> float:
    a = math.degrees(math.atan2(m[1, 0], m[0, 0]))
    return (a + 90.0) % 180.0 - 90.0


def demo_script(ctx):
    """Closed-loop scripted demo. It reads only camera-observable state: link and bar poses and its own grippers.

    The right arm sets its weight on the plan's flat middle and keeps holding it down. The left arm sets its weight
    just left of it and walks it out over the left half (a slow slide with the bar pressed down, which lays each
    curled slat flat ahead of it) until the bar sits on the observed end slat, then holds it there. The right arm
    then walks its weight out to the right end the same way while the left arm keeps its weight pressed (the
    scorer does not ask for that). Both let go together and move clear."""
    import linkdemo as ld
    from yam_arena.constants import PAD_TIP_PAST_TCP

    variant = getattr(ctx, "variant", "") or os.environ.get("DEMO_VARIANT", "")
    LOG.clear()
    LOG["variant"] = variant
    flags: dict = {}
    yield from ctx.wait(1.0)
    seen = ctx.observe()
    mid = 0.5 * (seen.pose("plan", "s7")[:3, 3] + seen.pose("plan", "s8")[:3, 3])
    xs = float(np.mean([seen.pose("plan", f"s{i}")[0, 3] for i in range(N)]))
    LOG["mid_mm"] = [round(1000 * float(v), 1) for v in (xs, mid[1])]
    z_on = PLAN_TOP + SLAT[2] + W_SIZE[2] / 2  # bar centre when it lies on the flat plan
    z_carry = max(seen.pose("plan", f"s{i}")[2, 3] for i in range(N)) + W_SIZE[2] / 2 + CARRY_UP  # over the curl
    shift = END_SHIFT.get(variant, 0.0)

    def bar(name):
        return ctx.observe().pose(name, "bar")

    def pick(arm, name):
        m = bar(name)
        rot = ld.down(_yaw_deg(m))
        g = m[:3, 3] + np.array([0.0, 0.0, -W_SIZE[2] / 2 + PAD_UP + PAD_TIP_PAST_TCP])
        yield from arm.move(g + [0, 0, HOVER], rot, width=OPEN_W, duration=1.5)
        yield from arm.move(g, rot, duration=1.2)
        yield from arm.grip(GRIP_W, 0.6)
        yield from arm.wait(0.3)
        yield from arm.move(g + [0, 0, LIFT], rot, duration=1.0)

    def carry_to(arm, name, xy, z_centre, duration):
        """Move so the held bar's centre goes to (xy, z_centre), using the bar's observed offset from the TCP."""
        c = bar(name)[:3, 3]
        tgt = np.array([xy[0], xy[1], z_centre]) - (c - arm.pos)
        yield from arm.move(tgt, duration=duration)

    def slide_to(arm, name, y, speed=WALK_SPEED, min_s=0.6):
        """Slide the held bar along y to its centre at ``y`` (x on the plan's centre line), keeping the press."""
        c = bar(name)[:3, 3]
        d = np.array([xs - c[0], y - c[1], 0.0])
        yield from arm.move(arm.pos + d, duration=max(min_s, abs(d[1]) / speed))

    def end_slat(slat):
        s = ctx.observe().pose("plan", slat)
        tilt = math.degrees(math.acos(float(np.clip(s[2, 2], -1.0, 1.0))))
        return s[:3, 3], tilt

    def walk_out(arm, name, slat, sign):
        """Walk the bar from where it was set to the end slat ``slat``: one slide to the end slat's flat position,
        then closed-loop nudges from the end slat's observed pose (further out while it is still curled, onto its
        centre once it lies flat under the bar)."""
        yield from slide_to(arm, name, mid[1] + sign * (abs(slat_y(0)) - shift))
        errs = []
        for _ in range(0 if shift else 4):  # the five_cm_in variant stops short on purpose
            yield from arm.wait(0.3)
            p, tilt = end_slat(slat)
            c = bar(name)[:3, 3]
            if tilt > FLAT_TILT or p[2] > PLAN_TOP + SLAT[2] / 2 + FLAT_DZ:
                errs.append(("curled", round(tilt, 1)))
                yield from slide_to(arm, name, c[1] + sign * 0.015, min_s=0.6)
                continue
            target = p[1] - sign * shift
            errs.append(round(1000 * (target - c[1]), 1))
            if abs(target - c[1]) < 0.002:
                break
            yield from slide_to(arm, name, target, min_s=0.6)
        LOG.setdefault("walk", {})[name] = errs

    def release(arm, side, keep=False):
        if keep:
            return
        yield from arm.grip(OPEN_W, 0.6)
        yield from arm.wait(0.2)
        yield from arm.move(arm.pos + [0, 0, 0.10], duration=1.0)
        yield from arm.move(np.array(PARK[side]), duration=1.2)

    def wait_for(*keys):
        while not all(flags.get(k) for k in keys):
            yield

    def idle_to_limit():
        yield from ctx.wait(max(0.5, RUN_S - ctx.t))

    if variant == "nothing":
        yield from idle_to_limit()
        return

    if variant == "weights_on_tube":  # set each weight on the crest of its half's curl, and let go

        def put_on_curl(arm, name, sign):
            yield from pick(arm, name)
            s = ctx.observe()
            edges = [s.point("plan", f"s{i}", (0.0, e * PITCH / 2, 0.0)) for i in range(N) for e in (-1, 1)
                     if (s.pose("plan", f"s{i}")[1, 3] - mid[1]) * sign > 0]
            crest = max(edges, key=lambda p: p[2])  # the highest edge of this half's curl
            y = float(crest[1])
            z = float(crest[2]) + SLAT[2] + W_SIZE[2] / 2 + 0.003
            yield from carry_to(arm, name, (xs, y), max(z + 0.05, z_carry), 2.0)
            yield from carry_to(arm, name, (xs, y), z, 1.2)
            yield from release(arm, "left" if sign > 0 else "right")

        yield from ctx.both(put_on_curl(ctx.left, "weight_L", 1.0), put_on_curl(ctx.right, "weight_R", -1.0))
        yield from idle_to_limit()
        return

    def right_seq():
        arm = ctx.right
        yield from pick(arm, "weight_R")
        y0 = mid[1] + Y_SET["weight_R"]
        yield from carry_to(arm, "weight_R", (xs, y0), z_carry, 2.0)
        yield from carry_to(arm, "weight_R", (xs, y0), z_on - PRESS, 1.5)
        if variant == "middle_weight_only":
            yield from release(arm, "right")
            return
        flags["R_set"] = True
        yield from wait_for("L_out")
        yield from walk_out(arm, "weight_R", "s0", -1.0)
        flags["R_out"] = True
        yield from wait_for("L_out", "R_out")
        yield from release(arm, "right", keep=variant == "hold")

    def left_seq():
        arm = ctx.left
        if variant == "middle_weight_only":
            return
        yield from pick(arm, "weight_L")
        c = bar("weight_L")[:3, 3]
        yield from carry_to(arm, "weight_L", (c[0], c[1]), z_carry, 1.0)  # wait up high, clear of the right arm
        yield from wait_for("R_set")
        y0 = mid[1] + Y_SET["weight_L"]
        yield from carry_to(arm, "weight_L", (xs, y0), z_carry, 1.5)
        yield from carry_to(arm, "weight_L", (xs, y0), z_on - PRESS, 1.5)
        yield from arm.wait(0.3)
        yield from walk_out(arm, "weight_L", f"s{N - 1}", 1.0)
        flags["L_out"] = True
        if variant == "release_left_early":  # let go of the left weight before the right arm walks out
            yield from release(arm, "left")
            return
        yield from wait_for("L_out", "R_out")  # keep holding it down while the right half is opened
        yield from release(arm, "left", keep=variant == "hold")

    yield from ctx.both(right_seq(), left_seq())
    LOG["script_end_s"] = round(ctx.t, 2)
    if variant in ("", "hold", "release_left_early"):
        yield from ctx.wait(6.0)
    else:
        yield from idle_to_limit()


def demo_report() -> dict:
    m = H.values.get("m") or {}
    s, why = ladder(H)
    return {
        "score": s, "why": why, "log": dict(LOG),
        "evers": {k: round(v, 2) for k, v in H.evers.items()},
        "runs": {k: round(v, 2) for k, v in H.best_runs.items()},
        "end": {k: (round(float(m[k]), 4) if isinstance(m.get(k), int | float) else m.get(k))
                for k in ("flat_run", "n_flat", "flat", "centre_err", "axis_deg", "centred", "both_ends")} if m else {},
        "end_cover": {k: round(v, 3) for k, v in m.get("cover", {}).items()},
        "end_seat_mm": {k: round(1000 * v, 2) for k, v in m.get("seat", {}).items()},
        "end_dz_mm": [round(1000 * v, 1) for v in m.get("dz", [])],
        "end_tilt_deg": [round(v, 1) for v in m.get("tilt", [])],
        "trace(t,run,nflat,covR,covL,cy_mm,clear_mm,v_mm_s,score,[yL,zL,yR,zR]mm)": list(TRACE),
    }
