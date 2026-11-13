# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Screw four propellers down onto a 4-inch quadcopter's motors. Two motors spin clockwise (left-hand shaft thread,
the prop tightens turning anticlockwise seen from above) and two spin anticlockwise (right-hand thread, the prop
tightens turning clockwise). Each prop starts seated on the first thread, 4 mm above its motor bell. A prop turned the
wrong way for the thread rides over the thread crests and never advances.

Frame (x forward from the arm bases, y left, z up, metres; table top TOP). Quad centre (0.17, 0.0). Motors at
(+-0.057, +-0.057) from the centre: front left and back right spin clockwise (red bells and red hub caps), front right
and back left spin anticlockwise (blue bells and blue hub caps).
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

CX, CY = 0.17, 0.0
ARM = 0.057
LEAD = 0.006  # m of travel per turn (3-start, 2 mm pitch)
TRAVEL = 0.004  # m from the thread start to the bell
STOP_DEG = 360.0 * TRAVEL / LEAD  # 240
TIGHT_EXTRA_DEG = 15.0
FREE_DEG = 400.0  # a prop turned the wrong way rides over the thread crests this far, then the hub jams
TIGHT_M = 0.0036  # hub within 0.2 mm of the bell
WRONG_GATE_DEG = 180.0
CLEAR_M = 0.030
HUB_AF = 0.016
HUB_H = 0.014
BELL_TOP = 0.036
HUB_Z0 = BELL_TOP + TRAVEL + HUB_H / 2  # hub centre above the table at the thread start

# name, x, y, sign (+1: clockwise motor, left-hand thread, tightens turning anticlockwise)
MOTORS = (("fl", CX + ARM, CY + ARM, +1), ("bl", CX - ARM, CY + ARM, -1),
          ("fr", CX + ARM, CY - ARM, -1), ("br", CX - ARM, CY - ARM, +1))
SIGN = {n: s for n, _, _, s in MOTORS}
NAMES = [n for n, _, _, _ in MOTORS]
RED, BLUE = (0.80, 0.12, 0.12), (0.15, 0.30, 0.85)

# -- frame ---------------------------------------------------------------------------------------------------------
_t = lk.Tree("frame", pos=(CX, CY, TOP), fixed=True, color=(0.12, 0.12, 0.14))
_t.link("body", [lk.box((0.19, 0.012, 0.004), (0, 0, 0.022), rpy_deg=(0, 0, 45)),
                 lk.box((0.19, 0.012, 0.004), (0, 0, 0.022), rpy_deg=(0, 0, -45)),
                 lk.box((0.036, 0.036, 0.016), (0, 0, 0.014)), lk.box((0.05, 0.05, 0.006), (0, 0, 0.003))], mass=0.5)
FRAME = _t.build()

OBJECTS = [FRAME]
for _n, _x, _y, _s in MOTORS:
    _c = RED if _s > 0 else BLUE
    _t = lk.Tree(_n, pos=(_x, _y, TOP), fixed=True, color=(0.7, 0.7, 0.72))
    _t.link("motor", [lk.cylinder(0.012, 0.012, (0, 0, 0.030), color=_c)], mass=0.03)
    _t.link("slide", [lk.box((0.002, 0.002, 0.002))], mass=0.003, at=(0, 0, HUB_Z0))
    h = HUB_AF / 2
    _t.link("prop", [lk.box((HUB_AF, 0.005, HUB_H), (0, 0.0055, 0), color=_c),
                     lk.box((HUB_AF, 0.005, HUB_H), (0, -0.0055, 0), color=_c),
                     lk.box((0.005, 0.006, HUB_H), (0.0055, 0, 0), color=_c),
                     lk.box((0.005, 0.006, HUB_H), (-0.0055, 0, 0), color=_c),
                     lk.box((0.043, 0.012, 0.002), (h + 0.0215, 0, -HUB_H / 2 + 0.001), color=(0.2, 0.2, 0.22)),
                     lk.box((0.043, 0.012, 0.002), (-h - 0.0215, 0, -HUB_H / 2 + 0.001), color=(0.2, 0.2, 0.22))],
            mass=0.008, at=(0, 0, HUB_Z0), friction=(0.6, 0.5), contact_offset=0.001)
    _t.prismatic("lift", "motor", "slide", anchor=(0, 0, HUB_Z0), axis=(0, 0, -1), limits=(0.0, TRAVEL),
                 stiffness=3000.0, damping=60.0)
    _lim = (-FREE_DEG, STOP_DEG + TIGHT_EXTRA_DEG) if _s > 0 else (-(STOP_DEG + TIGHT_EXTRA_DEG), FREE_DEG)
    _t.revolute("spin", "slide", "prop", anchor=(0, 0, HUB_Z0), axis=(0, 0, 1), limits_deg=_lim, damping=0.0004,
                friction=0.004)
    OBJECTS.append(_t.build())

# -- whole-run record -------------------------------------------------------------------------------------------------
H = lk.History()


@lk.on_step
def track(tick):
    s = tick.states
    near = []
    for n in NAMES:
        st = s[n]
        th = math.degrees(st.q["spin"])
        w = SIGN[n] * th
        qt = min(max(SIGN[n] * LEAD * st.q["spin"] / (2 * math.pi), 0.0), TRAVEL)
        lk.set_rest(n, "lift", qt)
        lk.set_limits(n, "lift", max(qt - 0.0002, 0.0), min(qt + 0.0002, TRAVEL))
        H.set(f"q.{n}", st.q["lift"])
        H.peak(f"wrong.{n}", max(0.0, -w))
        H.set(f"theta.{n}", th)
        near.append(st)
    ok = True
    for tool in tick.tools.values():
        ok = ok and tool.speed < 0.01 and all(lk.clearance(tool, st) >= CLEAR_M for st in near)
    H.held("release", ok)


def ladder() -> tuple[float, str]:
    p, v = H.peaks, H.values
    if any(p.get(f"wrong.{n}", 0.0) >= WRONG_GATE_DEG for n in NAMES):
        return 0.0, "gate: a prop turned the wrong way for its thread (cross-threaded)"
    n = sum(v.get(f"q.{k}", 0.0) >= TIGHT_M for k in NAMES)
    if n == 4:
        if H.runs.get("release", 0.0) >= 1.0:
            return 1.0, "four props tight, grippers clear and still"
        return 0.8, "four props tight, not released"
    return 0.2 * n, f"{n} props tight"


def score(objects) -> float:
    return ladder()[0]


def success(objects) -> bool:
    return ladder()[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder()[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


def demo_report() -> dict:
    return {"ladder": ladder(), "values": {k: round(v, 4) for k, v in H.values.items()},
            "peaks": {k: round(v, 2) for k, v in H.peaks.items()}, "release_s": round(H.runs.get("release", 0.0), 2),
            "variant": VARIANT, "log": LOG}


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Reads only what a camera sees: each prop's pose (hub height and yaw) and the grippers.
VARIANTS = ("", "nothing", "same_way_cw", "same_way_ccw", "press_down")
LOG: dict = {}
DEPTH = 0.0115  # pad contact centre past the TCP
HUB_W, SQUEEZE = HUB_AF, 0.0015
STROKE = 80.0
CENTRE = {"left": -25.0, "right": 25.0}  # wrist yaw the strokes are centred on (each wrist spans about -120..+70 or the mirror)
SIDE = {"fl": "left", "bl": "left", "fr": "right", "br": "right"}
Z0 = TOP + HUB_Z0


def _yaw(M) -> float:
    return math.degrees(math.atan2(M[1, 0], M[0, 0]))


def _prop(ctx, n, direction):
    """Screw one prop: strokes of the wrist about the hub axis while the arm follows the hub down."""
    import linkdemo as ld

    arm = ctx.arm(SIDE[n])
    up = np.array([0.0, 0.0, 1.0])
    for k in range(5):
        M = ctx.observe().pose(n, "prop")
        c, drop = M[:3, 3].copy(), Z0 - M[2, 3]
        engaged = direction == SIGN[n]
        remain = (TRAVEL - drop) / LEAD * 360.0 + 8.0 if engaged else 240.0
        if remain <= 5.0 or (engaged and drop >= TIGHT_M - 0.0002):
            break
        d = min(remain, STROKE)
        phi = _yaw(M) - 90.0
        target = CENTRE[SIDE[n]] - direction * d / 2.0
        phi += 90.0 * round((target - phi) / 90.0)
        r0 = ld.down(phi)
        tcp = c + up * DEPTH
        yield from arm.move(tcp + up * 0.045, r0, width=0.034, duration=1.0)
        yield from arm.move(tcp, r0, width=0.034, duration=0.7)
        yield from arm.grip(HUB_W - SQUEEZE, 0.5)
        dz = (LEAD * d / 360.0) if engaged else 0.0

        def at(s, tcp=tcp, r0=r0, d=d, dz=dz):
            a = ld.smooth(s) * d * direction
            return tcp - up * dz * ld.smooth(s), ld.rot_about((0, 0, 1), a) @ r0, HUB_W - SQUEEZE

        yield from arm.follow(at, 1.3)
        yield from arm.open(0.4)
        yield from arm.move(arm.pos + up * 0.045, width=ld.OPEN_WIDTH, duration=0.5)
    LOG[f"{n}.drop_mm"] = round(1000 * (Z0 - ctx.observe().pose(n, "prop")[2, 3]), 2)


def _park(ctx, n):
    arm = ctx.arm(SIDE[n])
    y = 0.22 if SIDE[n] == "left" else -0.22
    yield from arm.move(np.array([arm.pos[0], arm.pos[1], TOP + 0.13]), duration=0.8)
    yield from arm.move(np.array([0.13, y, TOP + 0.13]), duration=1.0)


def _press(ctx, n):
    arm = ctx.arm(SIDE[n])
    top = ctx.observe().pose(n, "prop")[:3, 3] + np.array([0.0, 0.0, HUB_H / 2])
    yield from arm.press(top, 0.008)
    yield from arm.retreat(0.06)


def demo_script(ctx):
    import linkdemo as ld

    yield from ctx.wait(1.0)
    if VARIANT == "nothing":
        yield from ctx.wait(30.0)
        return

    def side(names):
        for n in names:
            if VARIANT == "press_down":
                yield from _press(ctx, n)
                continue
            direction = {"same_way_cw": -1, "same_way_ccw": +1}.get(VARIANT, SIGN[n])
            yield from _prop(ctx, n, direction)
            yield from _park(ctx, n)  # the other wrist works one motor away: get out of its way
        yield from ctx.arm(SIDE[names[0]]).open(0.4)

    yield from ctx.both(side(("fl", "bl")), side(("br", "fr")))
    yield from ctx.wait(2.5)
