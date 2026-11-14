# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Put a pair of reading glasses on a training manikin head.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A training manikin head (hard vinyl, 145 mm wide, 190 mm deep, 230 mm tall) stands on its neck stub at x 0.20 and
  faces the robot (the face looks toward -x). It has a nose and two ears, fixed to the table.
- A pair of black reading glasses (140 mm frame, 135 mm hinge to hinge, 140 mm temples, 25 g, friction hinges that
  open 12 deg past square and fold in freely) lies open on the table in front of the head, its lenses on the table and
  its temples on a low wooden rest block.

Goal: the glasses worn, the bridge on the nose and both temples lying over the ears with their tips behind the ears;
both grippers let go and clear.
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
VARIANTS = ("", "nothing", "left_only", "right_only")

# -- manikin head (fixed). Head frame origin: the skull axis at the table; +x toward the back of the head ------------
XC = 0.20  # skull axis, world x
R_SKULL = 0.0725  # 145 mm head width
XF = XC - R_SKULL  # the face front
ZC = TOP + 0.020  # chin level; the neck stub is 20 mm tall
Z_N = ZC + 0.115  # nose-bridge top
EAR_X0, EAR_X1 = XC + 0.012, XC + 0.040  # ear front and rear edges
EAR_Y0, EAR_Y1 = R_SKULL, R_SKULL + 0.020  # an ear stands 20 mm out from the skull wall
Z_E = Z_N + 0.002  # ear top
SKIN = (0.87, 0.72, 0.62)
BLACK = (0.08, 0.08, 0.10)
LENS = (0.75, 0.88, 0.98)
WOOD = (0.62, 0.45, 0.28)

_t = lk.Tree("head", pos=(XC, 0.0, TOP), fixed=True, color=SKIN)
_t.link("skull", [
    lk.cylinder(0.045, 0.021, (0.0, 0.0, 0.0105)),  # neck stub
    lk.cylinder(R_SKULL, 0.230, (0.0, 0.0, 0.020 + 0.115)),  # front half of the skull
    lk.box((0.115, 2 * R_SKULL, 0.230), (0.0575, 0.0, 0.020 + 0.115)),  # back of the skull
    lk.box((0.035, 0.010, 0.065), (XF - XC - 0.0025, 0.0, Z_N - TOP - 0.0325)),  # nose, top flat at Z_N
    lk.box((EAR_X1 - EAR_X0, EAR_Y1 - EAR_Y0, 0.055), ((EAR_X0 + EAR_X1) / 2 - XC, (EAR_Y0 + EAR_Y1) / 2, Z_E - TOP - 0.0275)),
    lk.box((EAR_X1 - EAR_X0, EAR_Y1 - EAR_Y0, 0.055), ((EAR_X0 + EAR_X1) / 2 - XC, -(EAR_Y0 + EAR_Y1) / 2, Z_E - TOP - 0.0275)),
], mass=2.0, friction=(0.6, 0.5), contact_offset=0.001)
HEAD = _t.build()

# -- the glasses. Frame origin: centre of the front, +x toward the back of the head, +z up -----------------------------
X0S, Z0S = 0.020, TOP + 0.027  # start: lens bottoms on the table
Z0F = Z_N - 0.005  # seated: the bridge underside (frame z +0.005) on the nose top
X0F = XF - 0.0155  # seated: bridge plate over the nose
HINGE_X, HINGE_Y, TEMPLE_Z = 0.001, 0.0665, 0.011
TEMPLE_L = 0.138
_g = lk.Tree("glasses", pos=(X0S, 0.0, Z0S), color=BLACK)
_g.link("frame", [
    lk.box((0.004, 0.140, 0.006), (0.0, 0.0, 0.012)),  # brow bar, 140 mm over the hinges
    lk.box((0.002, 0.050, 0.036), (0.0, 0.037, -0.009), color=LENS),  # left lens
    lk.box((0.002, 0.050, 0.036), (0.0, -0.037, -0.009), color=LENS),  # right lens
    lk.box((0.012, 0.024, 0.004), (0.008, 0.0, 0.007)),  # bridge plate, underside 5 mm above the frame origin
], mass=0.014, friction=(0.5, 0.4), contact_offset=0.001)
for _n, _s in (("tl", 1.0), ("tr", -1.0)):
    _g.link(_n, [lk.box((TEMPLE_L, 0.003, 0.008), (TEMPLE_L / 2, 0.0, 0.0))], mass=0.0055,
            at=(HINGE_X, _s * HINGE_Y, TEMPLE_Z), friction=(0.5, 0.4), contact_offset=0.001)
    # positive = folding in toward the other temple; open stop at square, 12 deg of extra opening (friction hinge)
    _g.revolute("h" + _n[1], "frame", _n, anchor=(HINGE_X, _s * HINGE_Y, TEMPLE_Z), axis=(0, 0, -_s),
                limits_deg=(-12.0, 95.0), friction=0.0006, damping=1e-4)
GLASSES = _g.build()

# the wooden rest block under the temples at the start (its top is at the temples' bottoms)
_r = lk.Tree("rest", pos=(X0S + 0.08, 0.0, TOP), fixed=True, color=WOOD)
_r.link("block", [lk.box((0.040, 0.150, 0.034), (0.0, 0.0, 0.017))], mass=0.5, friction=(0.6, 0.5))
REST = _r.build()

OBJECTS = [HEAD, GLASSES, REST]

# -- scoring ---------------------------------------------------------------------------------------------------------
BRIDGE_LOCAL = np.array([0.008, 0.0, 0.005])  # bridge plate underside centre, frame frame
TIP_BOTTOM = np.array([HINGE_X * 0 + TEMPLE_L, 0.0, -0.004])  # temple tip bottom, temple frame
BRIDGE_X, BRIDGE_Y_TOL, BRIDGE_DX_TOL, BRIDGE_DZ = XF - 0.0075, 0.006, 0.008, 0.003
TIP_PAST_EAR = 0.003  # tip at least 3 mm behind the ear's rear edge
EAR_LIFT_TOL = 0.004  # temple tip bottom at most 4 mm above the ear top (and 2 mm below it at most)
OPEN_DEG = 3.0  # a temple folded in more than 3 deg is not open
LIFT_M = 0.04
STILL_MM_S, STILL_DEG_S = 3.0, 3.0
CLEAR_M, GRIP_TOUCH_M, TOOL_STILL = 0.030, 0.004, 0.005

H = lk.History()
LOG: dict = {}
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    LOG.clear()
    _PREV.clear()


def measure(g, tools, head=None) -> dict:
    """What the record needs from the glasses' LinkState and the grippers."""
    m = {}
    pb = g.point("frame", BRIDGE_LOCAL)
    m["bridge"] = (float(pb[0] - BRIDGE_X), float(pb[1]), float(pb[2] - Z_N))
    m["frame_z"] = float(g.poses["frame"][2, 3])
    for side, link, joint in (("l", "tl", "hl"), ("r", "tr", "hr")):
        tip = g.point(link, TIP_BOTTOM)
        m[f"tip_{side}"] = (float(tip[0] - EAR_X1), float(abs(tip[1])), float(tip[2] - Z_E))
        m[f"q_{side}"] = float(g.deg(joint))
    m["held"] = {s: lk.clearance(tools[s], g) <= GRIP_TOUCH_M for s in ("left", "right")}
    m["clear"] = {s: min(lk.clearance(tools[s], g), lk.clearance(tools[s], head) if head is not None else 9.0)
                  for s in ("left", "right")}
    m["tool_speed"] = {s: float(tools[s].speed) for s in ("left", "right")}
    return m


def bridge_ok(m) -> bool:
    dx, y, dz = m["bridge"]
    return abs(dx) <= BRIDGE_DX_TOL and abs(y) <= BRIDGE_Y_TOL and -0.002 <= dz <= BRIDGE_DZ


def temple_ok(m, side) -> bool:
    past, ay, dz = m[f"tip_{side}"]
    return (past >= TIP_PAST_EAR and EAR_Y0 <= ay <= EAR_Y1 and -0.002 <= dz <= EAR_LIFT_TOL
            and m[f"q_{side}"] <= OPEN_DEG)


def worn(m) -> bool:
    return bridge_ok(m) and temple_ok(m, "l") and temple_ok(m, "r")


def rates(prev, cur, dt):
    """Camera-style speeds from the pose change between control steps: (mm/s, deg/s)."""
    d = np.linalg.norm(cur[:3, 3] - prev[:3, 3]) * 1000.0 / dt
    c = (np.trace(prev[:3, :3].T @ cur[:3, :3]) - 1.0) / 2.0
    return float(d), math.degrees(math.acos(max(-1.0, min(1.0, float(c))))) / dt


class Record:
    def __init__(self, h=None):
        self.h = H if h is None else h

    def update(self, m, t):
        h = self.h
        h.peak("fold", max(m["q_l"], m["q_r"]))
        both_held = all(m["held"].values())
        h.ever("lifted", both_held and m["frame_z"] - Z0S >= LIFT_M)
        h.ever("nose", bridge_ok(m))
        w = worn(m)
        h.held("worn", w)
        h.held("still", m.get("speed", 9e9) <= STILL_MM_S and m.get("spin", 9e9) <= STILL_DEG_S)
        h.held("released", all(m["clear"][s] >= CLEAR_M and m["tool_speed"][s] <= TOOL_STILL for s in ("left", "right")))
        h.set("worn_now", w)


def ladder(h) -> tuple[float, str]:
    w = h.values.get("worn_now", False)
    if w and h.runs.get("still", 0.0) >= 1.0 and h.runs.get("released", 0.0) >= 1.0 and h.runs.get("worn", 0.0) >= 1.0:
        return 1.0, "worn, still, both grippers clear"
    if w:
        return 0.75, "bridge on the nose and both temples over the ears; not yet still and released"
    if "nose" in h.evers:
        return 0.5, "the bridge reached the nose, but the pair is not worn at the end"
    if "lifted" in h.evers:
        return 0.25, "the pair was lifted by both grippers, never on the nose"
    return 0.0, "the pair was never lifted by both grippers"


@lk.on_step
def _track(tick):
    g = tick.states["glasses"]
    m = measure(g, tick.tools, tick.states["head"])
    P = np.array(g.poses["frame"], dtype=float)
    if _PREV:
        m["speed"], m["spin"] = rates(_PREV["P"], P, max(tick.t - _PREV["t"], 1e-6))
        for link in ("tl", "tr"):
            ds, _ = rates(_PREV[link], np.array(g.poses[link], dtype=float), max(tick.t - _PREV["t"], 1e-6))
            m["speed"] = max(m["speed"], ds)
    _PREV.update(P=P, t=tick.t, tl=np.array(g.poses["tl"], dtype=float), tr=np.array(g.poses["tr"], dtype=float))
    REC.update(m, tick.t)
    LOG["last"] = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in m.items() if k not in ("clear", "held", "tool_speed")}
    if int(round(tick.t * 20)) % 10 == 0:
        H.trace("t", (round(tick.t, 2), [round(1000 * v, 1) for v in m["bridge"]],
                      [round(1000 * v, 1) for v in m["tip_l"]], [round(1000 * v, 1) for v in m["tip_r"]],
                      round(m["q_l"], 1), round(m["q_r"], 1), int(all(m["held"].values())),
                      [round(1000 * m["clear"][s]) for s in ("left", "right")],
                      [round(1000 * float(v), 1) for v in tick.tools["left"].tcp], round(1000 * tick.tools["left"].opening, 1),
                      [round(1000 * float(v), 1) for v in g.point("tl", (0.03, 0, 0))]))


REC = Record()


def score(objects) -> float:
    return ladder(H)[0]


def success(objects) -> bool:
    return ladder(H)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(H)[0]


def demo():  # yam_scripted: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Both arms grip a temple 30 mm behind its hinge, lift the pair, spread the temples 7 deg to clear the head's width,
# carry the pair in front of the face, slide it back over the nose and ears, lower it and let go.

GRASP_S = 0.030  # grip 30 mm behind the hinge
CONTACT_D = 0.008  # pad contact centre past the TCP: the pad tips 1 mm under the temple's bottom
LEAN = 25.0  # deg: each gripper leans outward, away from the other (the two hands are 133 mm apart)
OPEN_W, GRIP_W = 0.014, 0.0024  # pads close across the 3 mm temple, 0.6 mm squeeze
PHI = 7.0  # deg each temple is spread outward


def tcp_for(side, x0, z0, phi, standoff=0.0):
    """Grasp for a temple: the gripper points down, leaning outward, its pads closing across the temple."""
    import linkdemo as ld

    sgn = 1.0 if side == "left" else -1.0
    ph = math.radians(phi)
    d = np.array([math.cos(ph), sgn * math.sin(ph), 0.0])
    c = np.array([x0 + HINGE_X, sgn * HINGE_Y, z0 + TEMPLE_Z]) + GRASP_S * d
    ni = sgn * np.array([d[1], -d[0], 0.0])
    lean = math.radians(LEAN)
    r = ld.grip_frame(ni * math.sin(lean) + np.array([0.0, 0.0, -math.cos(lean)]), ni)
    return c - r[:, 2] * (CONTACT_D + standoff), r


class Hand:
    def __init__(self, ctx, side):
        self.arm, self.side = ctx.arm(side), side

    def go(self, k0, k1, width0, width1, dur):
        """Move the grasp from frame parameters k0=(x0, z0, phi[, standoff]) to k1; pad width width0 -> width1."""
        import linkdemo as ld

        k0, k1 = list(k0) + [0.0] * (4 - len(k0)), list(k1) + [0.0] * (4 - len(k1))

        def at(s):
            e = ld.smooth(s)
            k = [a + e * (b - a) for a, b in zip(k0, k1)]
            p, r = tcp_for(self.side, *k)
            return p, r, width0 + e * (width1 - width0)

        yield from self.arm.follow(at, dur)


def one_arm(ctx, side, variant):
    h = Hand(ctx, side)
    start = ctx.observe().pose("glasses", "frame")
    x0, z0 = float(start[0, 3]), float(start[2, 3])
    phi = PHI
    p, r = tcp_for(side, x0, z0, 0.0, 0.06)
    yield from h.arm.move(p, r, width=OPEN_W, duration=2.5)
    yield from h.go((x0, z0, 0, 0.06), (x0, z0, 0, 0.0), OPEN_W, OPEN_W, 1.5)
    yield from h.go((x0, z0, 0), (x0, z0, 0), OPEN_W, GRIP_W, 0.8)
    yield from h.arm.wait(0.5)
    yield from h.go((x0, z0, 0), (x0, Z0S + 0.10, 0), GRIP_W, GRIP_W, 1.5)
    yield from h.go((x0, Z0S + 0.10, 0), (x0, Z0S + 0.10, phi), GRIP_W, GRIP_W, 1.2)
    yield from h.go((x0, Z0S + 0.10, phi), (X0F - 0.025, Z0F + 0.012, phi), GRIP_W, GRIP_W, 3.0)
    yield from h.go((X0F - 0.025, Z0F + 0.012, phi), (X0F, Z0F + 0.012, phi), GRIP_W, GRIP_W, 1.5)
    yield from h.go((X0F, Z0F + 0.012, phi), (X0F, Z0F + 0.0005, phi), GRIP_W, GRIP_W, 1.5)
    yield from h.arm.wait(0.4)
    yield from h.go((X0F, Z0F + 0.0005, phi), (X0F, Z0F + 0.0005, phi), GRIP_W, OPEN_W, 0.8)
    yield from h.arm.wait(0.4)
    yield from h.go((X0F, Z0F + 0.0005, phi, 0.0), (X0F, Z0F + 0.0005, phi, 0.07), OPEN_W, OPEN_W, 1.5)


def idle(ctx, seconds):
    yield from ctx.wait(seconds)


def demo_script(ctx):
    v = VARIANT
    yield from ctx.wait(1.0)
    if v == "nothing":
        yield from ctx.wait(10.0)
        return
    left = one_arm(ctx, "left", v) if v != "right_only" else idle(ctx, 20.0)
    right = one_arm(ctx, "right", v) if v != "left_only" else idle(ctx, 20.0)
    yield from ctx.both(left, right)
    yield from ctx.wait(3.0)


def demo_report() -> dict:
    return {"variant": VARIANT, "ladder": ladder(H), "peaks": H.peaks, "runs": H.runs, "evers": sorted(H.evers) if hasattr(H, "evers") else None,
            "last": LOG.get("last"), "trace": H.traces.get("t", [])[::2]}
