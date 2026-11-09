# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Help a person carry a pipe onto its stands.

On the station table (x forward from the arm bases, y to the robot's left, z up; cm from the arm-base midpoint):
- a silver aluminium pipe, 2.5 cm across and 60 cm long (300 g), lies along x from x 14 to x 74 at y -9, resting on
  two grey foam blocks (at x 40 and x 66, 4 cm tall);
- two orange pipe jack stands (four splayed legs, a post and a V-head) stand at x 40 and x 66, y +9, 18 cm to the
  robot's left of the pipe; a pipe resting in both heads has its centre line 9.5 cm above the table;
- a worker in a hi-vis vest stands across the table, past its far edge, and grips the far end of the pipe in their
  right hand (fist round x 70 to 74, next to the red band).

Instruction: Help the person lift the silver pipe onto the two pipe stands, then let go of it.
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
CONTROL_HZ = 20
VARIANT = os.environ.get("DEMO_VARIANT", "")

# -- layout (m, world frame) ------------------------------------------------------------------------------------

PIPE_R, PIPE_LEN, PIPE_MASS = 0.0125, 0.60, 0.30
PIPE_C = np.array([0.44, -0.09, TOP + 0.0525])  # start centre, resting on the foam; the axis runs along +x
HAND_AT = 0.27  # the person's grip point, m along the pipe from its centre (world x 0.71 at the start)
H0 = PIPE_C + np.array([HAND_AT, 0.0, 0.0])  # the person's grip point at the start
FOAM_X, FOAM_Y, FOAM_H = (0.40, 0.66), -0.09, 0.040
STAND_X, STAND_Y = (0.40, 0.66), 0.09
V_VERTEX = 0.0773  # V-head vertex height above the table
REST_Z = TOP + 0.095  # centre line of a pipe resting in both V-heads (vertex + r * sqrt 2)
MOUNT = np.array([1.085, -0.26, TOP + 0.20])  # inside the person's torso: fixed mount of the hand drives

# The person's protocol after the robot takes hold: wait, lift, pause, carry across (stopping once part way),
# pause, lower into the V-heads, keep hold. Every entry is drawn uniformly per trial from the seed, in this order.
LIFT, CARRY, LOWER = 0.10, 0.18, 0.0575  # m: up, then +y, then down (the pipe ends in the V-heads)
PHASES = (("wait", 0.5, 5.0), ("lift", 1.0, 2.5), ("pause1", 0.3, 3.0), ("carry", 2.0, 5.0),
          ("frac", 0.3, 0.7), ("stop", 0.0, 3.0), ("pause2", 0.3, 2.0), ("lower", 0.8, 2.0))
# s, except frac: the share of the 18 cm at which the person stops for ``stop`` s (carry is the moving time)
MID_PLAN = {name: (lo + hi) / 2 for name, lo, hi in PHASES}  # mid-range timing (the blind cheat's person)

# The grip spring between the hand and the pipe (a person's arm compliance) and the hand's own drives.
GRIP_K, GRIP_C = 800.0, 20.0  # N/m, N s/m on x, y and z
BEND_K, BEND_C = 2.0, 0.05  # N m/rad, N m s/rad about y and z; free roll about the pipe
HAND_K, HAND_C = 5.0e4, 1000.0  # the hand follows its scripted path (stiff, overdamped)

# -- scoring thresholds -------------------------------------------------------------------------------------------

STRETCH_MAX = 0.025  # m between the hand and its grip point on the pipe: the person loses their grip beyond it
TILT_MAX_DEG = 4.0  # pipe pitch or yaw from level and straight
HOLD_RADIUS, HOLD_OPENING = 0.015, 0.030  # a gripper holds: TCP within 15 mm of the axis, opening <= 30 mm
HOLD_ALONG = (-0.30, 0.19)  # ... anywhere from the robot's end (x 0.14) to 8 cm short of the person's fist (x 0.63)
TRIGGER_STEPS = 20  # control steps of hold before the person starts
OPEN_RELEASE = 0.032  # a gripper that held the pipe has let go once it opens past this ...
OFF_RELEASE = 0.010  # ... or once its pads and TCP are all this far off the pipe surface (it slid off or moved away)
LIFTED_DZ = 0.040  # rung 0.25: both ends this far above their start height
CARRIED_Z, CARRIED_Y = TOP + 0.110, STAND_Y - 0.045  # rung 0.5: both ends at least this high, centre y at least this
SEAT_DZ, SEAT_DY, SEAT_DX = 0.005, 0.008, 0.020  # rung 0.75 tolerances
CLEAR = 0.030  # rung 1.0: every pad and TCP this far from the pipe surface
STILL_SPEED, STILL_STEPS = 0.005, 20  # rung 1.0: TCPs and pipe slower than 5 mm/s for the last 20 steps

# -- bodies ---------------------------------------------------------------------------------------------------------

SILVER, RED, CAP = (0.75, 0.76, 0.78), (0.80, 0.08, 0.06), (0.08, 0.08, 0.09)
SKIN, ORANGE, FOAM = (0.80, 0.58, 0.45), (0.95, 0.45, 0.05), (0.35, 0.35, 0.38)
SLEEVE, VEST, STRIPE, HAT = (0.12, 0.17, 0.32), (0.80, 0.90, 0.12), (0.82, 0.83, 0.85), (0.96, 0.96, 0.94)
ELBOW = np.array([0.90, -0.10, TOP + 0.27])  # world, at the start (on the hand: it rises with the hand)
SHOULDER_R, SHOULDER_L = np.array([1.00, -0.11, TOP + 0.56]), np.array([1.00, -0.41, TOP + 0.56])


def _rel(p, origin=(0.0, 0.0, TOP)) -> tuple:
    """World point -> a linkage frame at ``origin`` (default: on the table under the arm-base midpoint). The root
    link of every linkage sits at its frame's origin: Isaac Lab writes the linkage pose to the root body."""
    return tuple(float(a - b) for a, b in zip(p, origin, strict=True))


def _mt(p) -> tuple:
    """World point -> the carry linkage's frame (origin at the person's mount)."""
    return _rel(p, MOUNT)


def _segment(x0: float, x1: float, color) -> lk.Shape:
    return lk.cylinder(PIPE_R, x1 - x0, at=((x0 + x1) / 2, 0.0, 0.0), axis="X", color=color)


def _rod(a, b, radius: float, color) -> lk.Shape:
    """A capsule from point ``a`` to point ``b`` in its link's frame."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    d = b - a
    return lk.capsule(radius, float(np.linalg.norm(d)), at=tuple(float(v) for v in (a + b) / 2), axis="X",
                      quat=lk.matrix_to_quat(lk.x_to(d)), color=color)


def _ring(x: float, radius: float, deg: tuple, thick: float, color) -> list:
    """Capsules round the pipe axis at ``x`` (hand frame), through the angles ``deg`` (from +y toward +z)."""
    pts = [np.array([x, radius * math.cos(math.radians(a)), radius * math.sin(math.radians(a))]) for a in deg]
    return [_rod(p, q, thick, color) for p, q in zip(pts[:-1], pts[1:], strict=True)]


def _person_hand() -> list:
    """The right hand round the pipe end (hand frame: origin on the pipe axis at the grip point, world-aligned):
    palm on the +y side, four fingers wrapped under the pipe, the thumb over the top, the wrist and a work-shirt
    sleeve up to the elbow. Self-collision is off, so none of it pushes on the pipe."""
    shapes = [lk.box((0.044, 0.016, 0.044), at=(0.008, PIPE_R + 0.009, 0.004), color=SKIN)]  # palm
    for k in range(4):  # index (x -0.009) to little finger (x +0.0255): proximal, middle and distal segments
        shapes += _ring(-0.009 + 0.0115 * k, PIPE_R + 0.006, (-20.0, -80.0, -140.0, -195.0), 0.0055, SKIN)
    shapes += _ring(-0.010, PIPE_R + 0.007, (35.0, 90.0, 140.0), 0.0065, SKIN)  # thumb over the top
    wrist, cuff = np.array([0.030, 0.006, 0.014]), np.array([0.062, 0.0, 0.040])
    return shapes + [_rod(wrist, cuff, 0.021, SKIN), _rod(cuff, ELBOW - H0, 0.030, SLEEVE)]


def _person_body() -> list:
    """Torso in a hi-vis vest, right upper arm, the left arm hanging, head and hard hat. They sit on the walking
    carriage, so they follow the person's steps across but not the lift of their hand."""
    shapes = [lk.box((0.22, 0.38, 0.90), at=_mt((1.085, -0.26, TOP + 0.15)), color=VEST)]  # x 0.975 to 1.195
    for z in (TOP + 0.06, TOP + 0.24):  # two reflective stripes across the vest front
        shapes.append(lk.box((0.004, 0.38, 0.025), at=_mt((0.974, -0.26, z)), color=STRIPE))
    elbow_l, wrist_l = (0.985, -0.43, TOP + 0.16), (0.98, -0.43, TOP + 0.06)
    return shapes + [_rod(_mt(ELBOW), _mt(SHOULDER_R), 0.036, SLEEVE),
                     _rod(_mt(SHOULDER_L), _mt(elbow_l), 0.034, SLEEVE),
                     _rod(_mt(elbow_l), _mt(wrist_l), 0.024, SKIN),
                     _rod(_mt((1.085, -0.26, TOP + 0.60)), _mt((1.085, -0.26, TOP + 0.66)), 0.05, SKIN),
                     lk.capsule(0.085, 0.03, at=_mt((1.085, -0.26, TOP + 0.75)), axis="Z", color=SKIN),
                     lk.cylinder(0.125, 0.012, at=_mt((1.085, -0.26, TOP + 0.80)), axis="Z", color=HAT),
                     lk.cylinder(0.10, 0.06, at=_mt((1.085, -0.26, TOP + 0.835)), axis="Z", color=HAT)]


# 1. The pipe and the person holding it: one articulation fixed at the person's mount. Two stiff slides move the
#    hand along its scripted path (walk: y, which also carries the torso; lift: z); a 3-axis spring (grip_x/y/z)
#    and two bending springs (bend_y/z) join the hand to the pipe at x 0.71, and the pipe rolls freely in the hand.
#    The five spring stages are 10 g blocks hidden inside the pipe; self-collision is off, so the hand never pushes
#    on the pipe. The figure's shapes stay at least 7 mm from the table, the foam and the stands.
t = lk.Tree("carry", pos=tuple(MOUNT), fixed=True, self_collision=False, color=SKIN)
tiny = [lk.box((0.006, 0.006, 0.006))]
t.link("mount", tiny, mass=0.5)  # root, at the frame origin
t.link("carriage", tiny + _person_body(), mass=0.1, com=(0.0, 0.0, 0.0))
t.link("hand", _person_hand(), mass=0.30, at=_mt(H0), com=(0.0, 0.0, 0.0))
for stage in ("sx", "sy", "sz", "sb", "sc"):
    t.link(stage, [lk.box((0.006, 0.006, 0.006), color=SILVER)], mass=0.01, at=_mt(H0), com=(0.0, 0.0, 0.0))
half = PIPE_LEN / 2
t.link("pipe", [lk.cylinder(PIPE_R, 0.006, at=(-half + 0.003, 0, 0), axis="X", color=CAP),
                _segment(-half + 0.006, HAND_AT - 0.025, SILVER),
                _segment(HAND_AT - 0.025, HAND_AT - 0.005, RED),  # red band at the person's grip
                _segment(HAND_AT - 0.005, half - 0.006, SILVER),
                lk.cylinder(PIPE_R, 0.006, at=(half - 0.003, 0, 0), axis="X", color=CAP)],
       mass=PIPE_MASS, at=_mt(PIPE_C), color=SILVER, friction=(0.6, 0.5), com=(0.0, 0.0, 0.0))
t.prismatic("walk", "mount", "carriage", anchor=(0.0, 0.0, 0.0), axis=(0, 1, 0), limits=(-0.03, 0.22),
            stiffness=HAND_K, damping=HAND_C)
t.prismatic("lift", "carriage", "hand", anchor=_mt(H0), axis=(0, 0, 1), limits=(-0.04, 0.14),
            stiffness=HAND_K, damping=HAND_C)
for name, parent, child, axis in (("grip_x", "hand", "sx", (1, 0, 0)), ("grip_y", "sx", "sy", (0, 1, 0)),
                                  ("grip_z", "sy", "sz", (0, 0, 1))):
    t.prismatic(name, parent, child, anchor=_mt(H0), axis=axis, limits=(-0.10, 0.10), stiffness=GRIP_K,
                damping=GRIP_C)
t.revolute("bend_y", "sz", "sb", anchor=_mt(H0), axis=(0, 1, 0), limits_deg=(-60, 60), stiffness=BEND_K,
           damping=BEND_C)
t.revolute("bend_z", "sb", "sc", anchor=_mt(H0), axis=(0, 0, 1), limits_deg=(-60, 60), stiffness=BEND_K,
           damping=BEND_C)
t.revolute("roll", "sc", "pipe", anchor=_mt(H0), axis=(1, 0, 0), damping=0.001)
CARRY_OBJ = t.build()


def _v_plates(x: float) -> list:
    """Two 4 mm plates at +-45 deg, 40 mm long, whose inner faces meet at the vertex (x, STAND_Y, V_VERTEX) and
    rise 32 mm (the V is 45 mm wide at the top, for the 25 mm pipe)."""
    c, s = math.cos(math.pi / 4), math.sin(math.pi / 4)
    out = []
    for side in (1.0, -1.0):  # +y plate, -y plate
        along = np.array([0.0, side * c, s])  # up the inner face, away from the vertex
        normal = np.array([0.0, -side * s, c])  # inner face normal (toward the pipe)
        centre = np.array([x, STAND_Y, V_VERTEX]) + 0.016 * along - 0.002 * normal
        out.append(lk.box((0.040, 0.032, 0.004), at=tuple(centre), rpy_deg=(side * 45.0, 0.0, 0.0)))
    return out


def _jack_stand(x: float) -> list:
    """A pipe jack stand at (x, STAND_Y), stands-linkage frame: four splayed legs on feet, a hub, a round post with
    its height pin, a saddle under the head, and the V-head."""
    y, hub = STAND_Y, np.array([x, STAND_Y, 0.022])
    shapes = [lk.box((0.026, 0.026, 0.016), at=tuple(hub)),
              lk.cylinder(0.011, V_VERTEX - 0.040, at=(x, y, 0.030 + (V_VERTEX - 0.040) / 2)),
              lk.cylinder(0.0025, 0.036, at=(x, y, 0.052), axis="Y", color=SILVER),
              lk.box((0.040, 0.024, 0.008), at=(x, y, V_VERTEX - 0.008))]
    for dx, dy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        foot = np.array([x + 0.032 * dx, y + 0.032 * dy, 0.005])
        shapes += [_rod(hub, foot, 0.0045, ORANGE), lk.box((0.014, 0.014, 0.004), at=(foot[0], foot[1], 0.002))]
    return shapes + _v_plates(x)


# 2. Two steel pipe jack stands (fixed).
t = lk.Tree("stands", pos=(0.0, 0.0, TOP), fixed=True, color=ORANGE)
t.link("stands", [s for x in STAND_X for s in _jack_stand(x)], mass=2.0, friction=(0.5, 0.4))
STANDS = t.build()

# 3. Two EVA foam blocks the pipe starts on (fixed).
t = lk.Tree("foam", pos=(0.0, 0.0, TOP), fixed=True, color=FOAM)
t.link("foam", [lk.box((0.05, 0.05, FOAM_H), at=(x, FOAM_Y, FOAM_H / 2)) for x in FOAM_X], mass=0.02,
       friction=(0.9, 0.9))
FOAM_OBJ = t.build()

OBJECTS = [CARRY_OBJ, STANDS, FOAM_OBJ]

# -- the person ---------------------------------------------------------------------------------------------------


def partner_plan(seed) -> dict:
    """The person's timing for this trial (s; ``frac`` is a share of the carry), drawn from ``seed``."""
    rng = np.random.default_rng(seed)
    return {name: float(rng.uniform(lo, hi)) for name, lo, hi in PHASES}


def plan_length(plan: dict) -> float:
    """Seconds from the trigger to the end of the lowering."""
    return float(sum(plan[name] for name, _, _ in PHASES if name != "frac"))


def smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


def partner_offset(plan: dict, tau: float) -> tuple[float, float]:
    """(dy, dz) of the person's hand from its start position, ``tau`` s after the trigger."""
    t = tau - plan["wait"]
    dz = LIFT * smooth(t / plan["lift"])
    t -= plan["lift"] + plan["pause1"]
    f = plan["frac"]
    leg1, leg2 = plan["carry"] * f, plan["carry"] * (1.0 - f)
    dy = CARRY * f * smooth(t / leg1)
    t -= leg1 + plan["stop"]
    dy += CARRY * (1.0 - f) * smooth(t / leg2)
    t -= leg2 + plan["pause2"]
    dz -= LOWER * smooth(t / plan["lower"])
    return dy, dz


# -- measuring ------------------------------------------------------------------------------------------------------


def pipe_axis(pose) -> tuple[np.ndarray, np.ndarray]:
    """(centre, unit axis) of the pipe from its 4x4 pose."""
    pose = np.asarray(pose, dtype=float)
    u = pose[:3, 0] / np.linalg.norm(pose[:3, 0])
    return pose[:3, 3].copy(), u


def axis_z_at(c, u, x: float) -> float:
    """Height of the pipe's centre line where it crosses world x (nan if the pipe is across x)."""
    if abs(u[0]) < 1e-6:
        return float("nan")
    return float(c[2] + (x - c[0]) / u[0] * u[2])


def surface_gap(p, c, u) -> float:
    """Distance (m) from a point to the pipe's surface (0 or less inside)."""
    rel = np.asarray(p, dtype=float) - c
    s = float(np.clip(rel @ u, -PIPE_LEN / 2, PIPE_LEN / 2))
    return float(np.linalg.norm(rel - s * u)) - PIPE_R


def measure(pipe_pose, hand_pos, pipe_vel, pipe_spin, tools: dict) -> dict:
    """Everything the scorer needs at one control step, from the pipe's pose and velocity, the hand's position and
    the grippers (lk.Tool: tcp, rot, opening, speed)."""
    c, u = pipe_axis(pipe_pose)
    far = c + HAND_AT * u
    ends = (c - PIPE_LEN / 2 * u, c + PIPE_LEN / 2 * u)
    v, w = np.asarray(pipe_vel, dtype=float), np.asarray(pipe_spin, dtype=float)
    speed = max(float(np.linalg.norm(v + np.cross(w, e - c))) for e in (c, *ends))
    m = {
        "centre": c, "axis": u,
        "stretch": float(np.linalg.norm(np.asarray(hand_pos, dtype=float) - far)),
        "pitch": math.degrees(math.asin(float(np.clip(u[2], -1.0, 1.0)))),
        "yaw": math.degrees(math.atan2(float(u[1]), float(u[0]))),
        "end_z": (float(ends[0][2]), float(ends[1][2])),
        "stand_z": tuple(axis_z_at(c, u, x) for x in STAND_X),
        "pipe_speed": speed,
        "holding": {}, "opening": {}, "gap": {}, "tool_speed": {},
    }
    for side, tool in tools.items():
        rel = np.asarray(tool.tcp, dtype=float) - c
        along = float(rel @ u)
        radial = float(np.linalg.norm(rel - along * u))
        m["holding"][side] = bool(radial <= HOLD_RADIUS and HOLD_ALONG[0] <= along <= HOLD_ALONG[1]
                                  and tool.opening <= HOLD_OPENING)
        m["opening"][side] = float(tool.opening)
        m["gap"][side] = min(surface_gap(p, c, u) for p in tool.pad_points())
        m["tool_speed"][side] = float(tool.speed)
    return m


def measure_states(states: dict, tools: dict) -> dict:
    s = states["carry"]
    return measure(s.poses["pipe"], s.poses["hand"][:3, 3], s.lin_vels["pipe"], s.ang_vels["pipe"], tools)


# -- whole-run record (cleared only by the simulator reset) ------------------------------------------------------

H = lk.History()


def record_step(rec: lk.History, m: dict, t: float, plan: dict) -> None:
    """Update the whole-run record with one control step's measurement ``m`` at time ``t`` (s since reset)."""
    rec.peak("stretch", m["stretch"])
    rec.peak("pitch", abs(m["pitch"]))
    rec.peak("yaw", abs(m["yaw"]))
    rec.ever("lifted", min(m["end_z"]) >= PIPE_C[2] + LIFTED_DZ)
    rec.ever("carried", min(m["end_z"]) >= CARRIED_Z and m["centre"][1] >= CARRIED_Y)
    vals = rec.values
    run = vals.get("hold_run", 0) + 1 if any(m["holding"].values()) else 0
    rec.set("hold_run", run)
    if "t_trigger" not in vals and run >= TRIGGER_STEPS:
        rec.set("t_trigger", float(t))
        rec.set("t_lower_end", float(t) + plan_length(plan))
    if "t_trigger" in vals:
        for side, holding in m["holding"].items():
            if holding:
                rec.set(f"held_{side}", True)
        # The robot has let go once every gripper that held the pipe since the trigger is off it (opened past 32 mm
        # or moved 10 mm away) and none holds it: regrasping with one gripper while another holds is not letting go.
        held = [side for side in m["holding"] if vals.get(f"held_{side}")]
        let_go = bool(held) and all(not m["holding"][side] and (m["opening"][side] > OPEN_RELEASE
                                                                or m["gap"][side] > OFF_RELEASE) for side in held)
        if let_go and "t_release" not in vals:
            rec.set("t_release", float(t))
    still = m["pipe_speed"] < STILL_SPEED and all(v < STILL_SPEED for v in m["tool_speed"].values())
    rec.set("still_run", vals.get("still_run", 0) + 1 if still else 0)


def gated(rec: lk.History) -> bool:
    """A whole-run gate failed: the person lost their grip, or the pipe tipped or swung."""
    p = rec.peaks
    return p.get("stretch", 0.0) > STRETCH_MAX or p.get("pitch", 0.0) > TILT_MAX_DEG or p.get("yaw", 0.0) > TILT_MAX_DEG


def seated(m: dict) -> bool:
    c = m["centre"]
    return (all(abs(z - REST_Z) <= SEAT_DZ for z in m["stand_z"]) and abs(c[1] - STAND_Y) <= SEAT_DY
            and abs(c[0] - PIPE_C[0]) <= SEAT_DX)


def ladder(rec: lk.History, m: dict) -> float:
    """The score: gates first, then 0.25 lifted, 0.5 carried over the stands, 0.75 set in both V-heads now,
    1.0 set, let go only after the person finished lowering, grippers 30 mm clear and everything still for 1 s."""
    if gated(rec) or "lifted" not in rec.evers:
        return 0.0
    if "carried" not in rec.evers:
        return 0.25
    if not seated(m):
        return 0.5
    vals = rec.values
    t_rel, t_low = vals.get("t_release"), vals.get("t_lower_end")
    if (all(g >= CLEAR for g in m["gap"].values()) and vals.get("still_run", 0) >= STILL_STEPS
            and t_rel is not None and t_low is not None and t_rel >= t_low):
        return 1.0
    return 0.75


# -- simulator hooks ----------------------------------------------------------------------------------------------

_RUN: dict = {"seed": 0, "plan": partner_plan(0)}


def _capture_seed(module) -> None:
    """Remember the seed each reset is called with (inspect-robots passes one per trial) on the embodiment."""
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_cocarry_seed_hook", False):
        return
    inner = cls.reset

    def reset(self, *a, **k):
        self._cocarry_seed = k.get("seed")
        return inner(self, *a, **k)

    cls.reset = reset
    cls._cocarry_seed_hook = True


if "yam_arena.inspect_embodiment" in sys.modules:
    _capture_seed(sys.modules["yam_arena.inspect_embodiment"])
else:
    _kit_patch = lk._TARGETS["yam_arena.inspect_embodiment"]
    lk._TARGETS["yam_arena.inspect_embodiment"] = lambda m: (_kit_patch(m), _capture_seed(m))


@lk.on_reset
def _start(tick):
    seed = getattr(tick.emb, "_cocarry_seed", None)
    if seed is None:
        seed = int(os.environ.get("COCARRY_SEED", "0"))
    _RUN["seed"], _RUN["plan"] = int(seed), partner_plan(int(seed))
    H.set("seed", int(seed))
    lk.set_rest("carry", "walk", 0.0)
    lk.set_rest("carry", "lift", 0.0)


@lk.on_step
def _track(tick):
    m = measure_states(tick.states, tick.tools)
    record_step(H, m, tick.t, _RUN["plan"])
    t0 = H.values.get("t_trigger")
    if t0 is not None:  # the person moves only after the robot has taken hold, and never reacts to it
        dy, dz = partner_offset(_RUN["plan"], tick.t - t0)
        lk.set_rest("carry", "walk", dy)
        lk.set_rest("carry", "lift", dz)
    if tick.n % 5 == 0:
        H.trace("pose", (round(m["stretch"] * 1000, 1), round(m["pitch"], 2), round(m["yaw"], 2),
                         round(float(m["centre"][1]), 4), round(float(m["centre"][2] - TOP), 4)))


def score(objects) -> float:
    return ladder(H, measure_states(objects, lk.grippers()))


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    return 1.0 - score(objects)


def demo():
    return {}  # yam_scripted holds both arms at ready; the full demo is demo_script (python linkdemo.py task.py OUT)


# -- scripted demo (linkdemo): closed loop on the observed hand and pipe poses -----------------------------------

GRIP_X = {"right": 0.165, "left": 0.265}  # world x of each TCP on the pipe, 100 mm apart near the robot's end
W_OPEN, W_CLOSE, HOVER = 0.060, 0.022, 0.060
GRIP_TILT_DEG = 20.0  # each wrist leans this far toward its own arm's base (the left arm reaches 35 cm across)
LEAD_S = 0.10  # s of the hand's observed velocity added ahead (arm lag)
LEVEL_GAIN, LEVEL_MAX = 0.10, 0.015  # integral correction toward a level, straight pipe (per step, m)
RUSH_S = 1.0  # rush cheat: carries the full 0.18 m in this time once the person starts across (they take 2-8 s)
MEAN_SEEDS = 500  # blind_mean cheat: plays the person's path averaged over seeds 0 to 499
CHEAT_WAIT_S = 25.0  # stay and lead cheats hold on this long: past the slowest person's lowering (23.5 s in)
DEMO_LOG: dict = {}


def _axis_point(c, u, x: float) -> np.ndarray:
    return c + (x - c[0]) / u[0] * u


def _hand(seen) -> np.ndarray:
    return seen.pose("carry", "hand")[:3, 3].copy()


def _grip_rot(ld, side: str) -> np.ndarray:
    """Pads closing across the pipe, the approach tilted GRIP_TILT_DEG from straight down toward the arm's base."""
    th, s = math.radians(GRIP_TILT_DEG), (1.0 if side == "left" else -1.0)
    approach = np.array([0.0, -s * math.sin(th), -math.cos(th)])
    closing = np.array([0.0, math.cos(th), -s * math.sin(th)])
    return ld.grip_frame(approach, closing, ref=ld.down(0.0))


def demo_script(ctx):
    import linkdemo as ld

    v = VARIANT
    dt = 1.0 / CONTROL_HZ
    arms = {"left": ctx.left, "right": ctx.right}
    home = {s: (a.pos.copy(), a.rot.copy()) for s, a in arms.items()}
    rot = {s: _grip_rot(ld, s) for s in arms}
    up = {s: -HOVER * rot[s][:, 2] for s in arms}  # back out along each gripper's own approach
    DEMO_LOG.clear()
    if v == "nothing":
        yield from ctx.wait(15.0)
        return
    c, u = pipe_axis(ctx.observe().pose("carry", "pipe"))
    g = {s: _axis_point(c, u, x) for s, x in GRIP_X.items()}
    yield from ctx.both(*(arms[s].move(g[s] + up[s], rot[s], width=W_OPEN, duration=2.0) for s in arms))
    yield from ctx.both(*(arms[s].move(g[s], rot[s], duration=1.2) for s in arms))
    yield from ctx.both(*(arms[s].grip(W_CLOSE, duration=0.6) for s in arms))
    if v == "stay":  # hold on and do not move
        yield from ctx.wait(CHEAT_WAIT_S)
        return
    if v == "lead":  # lift its end 100 mm straight away, before the person starts
        yield from ctx.both(*(arms[s].move(g[s] + np.array([0.0, 0.0, LIFT]), rot[s], duration=1.0) for s in arms))
        yield from ctx.wait(CHEAT_WAIT_S)
        return
    if v in ("blind", "blind_mean"):  # a fixed path played open loop, timed from its own grasp; never looks
        n_path = int((sum(hi for name, _, hi in PHASES if name != "frac") + 0.5) * CONTROL_HZ) + 1
        if v == "blind":  # the mid-range person
            path = np.array([partner_offset(MID_PLAN, k * dt) for k in range(n_path + 1)])
        else:  # the person's path averaged over many seeds (the best guess without looking)
            plans = [partner_plan(seed) for seed in range(MEAN_SEEDS)]
            path = np.array([np.mean([partner_offset(p, k * dt) for p in plans], axis=0) for k in range(n_path + 1)])
        n = 0
        while n < TRIGGER_STEPS:
            n = n + 1 if all(tool.opening <= HOLD_OPENING for tool in ctx.observe().tools.values()) else 0
            yield
        for k in range(n_path):
            (y0, z0), (y1, z1) = path[k], path[k + 1]
            d = np.array([0.0, y0 + LEAD_S * (y1 - y0) / dt, z0 + LEAD_S * (z1 - z0) / dt])
            for s in arms:
                arms[s]._solve(g[s] + d, rot[s], W_CLOSE)
            yield
    else:
        # follow: each step, move both grips with the observed hand (plus a velocity lead) and level the pipe
        h_prev = _hand(ctx.observe())
        vels, corr, peak_z, still, k, rush = [], np.zeros(3), h_prev[2], 0, 0, None
        worst = {"pitch": 0.0, "yaw": 0.0, "stretch": 0.0}
        while True:
            seen = ctx.observe()
            h = _hand(seen)
            vels = (vels + [(h - h_prev) / dt])[-3:]
            h_prev, vel = h, np.mean(vels, axis=0)
            peak_z = max(peak_z, h[2])
            c, u = pipe_axis(seen.pose("carry", "pipe"))
            far, mid = c + HAND_AT * u, _axis_point(c, u, 0.5 * (GRIP_X["left"] + GRIP_X["right"]))
            err = far - mid  # a level, straight pipe has its far grip point at our grips' height and y
            err[0] = 0.0
            corr = np.clip(corr + LEVEL_GAIN * err, -LEVEL_MAX, LEVEL_MAX)
            d = h - H0 + LEAD_S * vel + corr
            worst["pitch"] = max(worst["pitch"], abs(math.degrees(math.asin(np.clip(u[2], -1, 1)))))
            worst["yaw"] = max(worst["yaw"], abs(math.degrees(math.atan2(u[1], u[0]))))
            worst["stretch"] = max(worst["stretch"], float(np.linalg.norm(h - far)))
            if v == "rush" and rush is None and h[1] - H0[1] > 0.003:  # the person starts across: go on its own
                rush = (k, d[1])
            if rush is not None:
                d[1] = rush[1] + (CARRY - rush[1]) * smooth((k - rush[0]) * dt / RUSH_S)
            for s in arms:
                arms[s]._solve(g[s] + d, rot[s], W_CLOSE)
            k += 1
            if v == "early_release" and h[1] - H0[1] > CARRY - 0.005:
                break
            lowered = h[1] - H0[1] > CARRY - 0.02 and peak_z - h[2] > LOWER - 0.0075
            still = still + 1 if lowered and float(np.linalg.norm(vel)) < 0.002 else 0
            if still >= 10:  # the person has set their end down and stopped for 0.5 s
                break
            yield
        DEMO_LOG.update({"follow_steps": k, "seen_worst": {a: round(b * (1000 if a == "stretch" else 1), 2)
                                                           for a, b in worst.items()},
                         "level_corr_mm": [round(x * 1000, 1) for x in corr]})
    if v == "early_release":  # let go at the end of the carry, before the person lowers
        yield from ctx.both(*(arms[s].grip(W_OPEN, duration=0.3) for s in arms))
        yield from ctx.both(*(arms[s].move(arms[s].pos + up[s], duration=1.0) for s in arms))
        yield from ctx.wait(15.0)
        return
    yield from ctx.both(*(arms[s].grip(W_OPEN, duration=0.6) for s in arms))
    yield from ctx.both(*(arms[s].move(arms[s].pos + up[s], duration=1.0) for s in arms))
    yield from ctx.both(*(arms[s].move(home[s][0], home[s][1], duration=2.0) for s in arms))
    yield from ctx.wait(3.0)


def demo_report() -> dict:
    vals = H.values
    keep = ("seed", "t_trigger", "t_lower_end", "t_release", "still_run")
    return {
        "variant": VARIANT,
        "plan": {k: round(v, 3) for k, v in _RUN["plan"].items()},
        "record": {k: (round(vals[k], 3) if isinstance(vals[k], float) else vals[k]) for k in keep if k in vals},
        "peaks": {"stretch_mm": round(H.peaks.get("stretch", 0.0) * 1000, 2),
                  "pitch_deg": round(H.peaks.get("pitch", 0.0), 2), "yaw_deg": round(H.peaks.get("yaw", 0.0), 2)},
        "evers": {k: round(v, 2) for k, v in H.evers.items()},
        "held": {k: vals[k] for k in vals if k.startswith("held_")},
        "demo": dict(DEMO_LOG),
        "trace": H.traces.get("pose", [])[::4],
    }
