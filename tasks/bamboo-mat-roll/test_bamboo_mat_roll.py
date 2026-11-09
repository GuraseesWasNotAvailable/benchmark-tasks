# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for bamboo-mat-roll, with no simulator: synthetic object states fed to the task's own
history update and scoring code. Run: pytest -q (needs numpy and yam_arena importable, no Isaac Sim)."""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import linkkit as lk  # noqa: E402
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control step
R1, R2 = 0.01385, 0.01555  # strip centres on the first and second layer of a tight roll


def ry(deg):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rz(deg):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def pose(p, r=None):
    m = np.eye(4)
    m[:3, :3] = np.eye(3) if r is None else r
    m[:3, 3] = p
    return m


def state(spec, poses, q=None, speed=0.0):
    pts = np.array([m[:3, 3] for m in poses.values()])
    root = spec.links[0].name
    v = np.array([speed, 0.0, 0.0])
    return lk.LinkState(pos=poses[root][:3, 3].copy(), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=v,
                        ang_vel=np.zeros(3), low=pts.min(0) - 0.01, high=pts.max(0) + 0.01, spec=spec, poses=poses,
                        lin_vels={n: v.copy() for n in poses}, ang_vels={n: np.zeros(3) for n in poses},
                        q=dict(q or {j.name: 0.0 for j in spec.moving_joints}), qd={})


def mat_state(q=None):
    q = q or {}
    full = {j.name: q.get(j.name, 0.0) for j in T.MAT.moving_joints}
    return state(T.MAT, T.MAT.link_poses(full), full)


def curled_mat(deg=180.0):
    """The mat's near 60 mm turned over by ``deg`` in all (spread over hinges j1..j6)."""
    return mat_state({f"j{i}": -math.radians(deg) / 6 for i in range(1, 7)})


def core_pose(x=0.40, z=None, tilt_deg=0.0, y=0.0):
    z = T.CORE_Z0 + 0.0017 if z is None else z  # a wrapped roll sits one sheet thicker
    return pose((x, y, z), rz(tilt_deg))


def flat_nori(core=None):
    poses = T.NORI.link_poses()
    if core is not None:
        poses["core"] = core
    return state(T.NORI, poses)


def wrapped_nori(core, outer_deg=20.0, outer=None, outer_r=R2, inner_r=R1, speed=0.0):
    """A tight roll: from the inner end, strips follow the cucumber at ``inner_r`` for one turn, then lie over it at
    ``outer_r``. The outer end sits ``outer_deg`` from straight down (positive toward +x). By default the far edge
    (l10) is outermost and the near edge innermost; ``outer="l0"`` mirrors the sheet."""
    o = core[:3, 3]
    n = T.N_NORI
    outer = outer or f"l{n - 1}"
    ang, rad = [0.0], [inner_r]
    for _ in range(1, n):
        a = ang[-1] + math.degrees(T.NORI_PITCH / rad[-1])
        ang.append(a)
        rad.append(R2 if a > 345.0 else inner_r)
    rad[-1] = outer_r
    shift = outer_deg - ang[-1]
    names = [f"l{k}" for k in range(n)] if outer == f"l{n - 1}" else [f"l{n - 1 - k}" for k in range(n)]
    poses = {"core": core}
    for name, a_deg, r in zip(names, [x + shift for x in ang], rad, strict=True):
        a = math.radians(a_deg)
        poses[name] = pose(o + r * np.array([math.sin(a), 0.0, -math.cos(a)]), ry(a_deg))
    return state(T.NORI, poses, speed=speed)


def arc_nori(core, m):
    """Strips l0..l{m-1} round the cucumber on the first layer (a wrap of m strip spans); the rest flat well ahead."""
    o = core[:3, 3]
    step = math.degrees(T.NORI_PITCH / R1)
    poses = {"core": core}
    for k in range(T.N_NORI):
        if k < m:
            a = math.radians(k * step)
            poses[f"l{k}"] = pose(o + R1 * np.array([math.sin(a), 0.0, -math.cos(a)]), ry(k * step))
        else:
            poses[f"l{k}"] = pose((o[0] + 0.03 + 0.008 * k, 0.0, T.NORI_Z))
    return state(T.NORI, poses)


def tools(far=True, speed=0.0, near_point=None):
    out = {}
    rot = lk.quat_to_matrix((1.0, 0.0, 0.0, 0.0))  # pointing down
    for side, y in (("left", 0.30), ("right", -0.30)):
        tcp = np.array([0.32, y, T.TOP + 0.25]) if far else np.array([0.40, y * 0.1, T.TOP + 0.13])
        out[side] = lk.Tool(side, tcp, rot, 0.03, speed)
    if near_point is not None:
        out["right"] = lk.Tool("right", np.asarray(near_point, float), rot, 0.0, 0.0)
    return out


def run(frames):
    """frames: list of (states, tools) per control step. Returns (score, reason) at the last step."""
    h = lk.History()
    t = 0.0
    for st, tl in frames:
        t += DT
        h._tick(t)
        T.update_history(h, st, tl)
    return T.evaluate(frames[-1][0], h)


def hold(st, seconds=2.0, tl=None):
    return [(st, tl or tools())] * round(seconds / DT)


def curl_phase(seconds=0.5):
    """The mat curled over the cucumber before anything is wrapped (what the demo does first)."""
    return hold({"mat": curled_mat(), "nori": flat_nori()}, seconds)


def good_states(mat=None, **kw):
    core = core_pose(**{k: v for k, v in kw.items() if k in ("x", "z", "tilt_deg", "y")})
    nori = wrapped_nori(core, **{k: v for k, v in kw.items() if k in ("outer_deg", "outer", "outer_r")})
    return {"board": None, "mat": mat or mat_state(), "nori": nori}


def finish(st, seconds=2.0, tl=None):
    return curl_phase() + hold(st, seconds, tl)


# -- the layout ---------------------------------------------------------------------------------------------------

def test_layout_is_consistent():
    assert [o.name for o in T.OBJECTS] == ["board", "mat", "nori"]
    p = T.NORI.link_poses()
    near = p["l0"][0, 3] - T.NORI_LINK[0] / 2
    far = p[f"l{T.N_NORI - 1}"][0, 3] + T.NORI_LINK[0] / 2
    assert math.isclose(near, 0.3312, abs_tol=1e-6) and math.isclose(far, 0.4188, abs_tol=1e-6)
    assert math.isclose(far - near, 0.0876, abs_tol=1e-9)  # one turn round the cucumber (2 pi x 13.85 = 87.0 mm)
    assert T.MAT_X0 - T.SLAT[0] / 2 < near and far < T.MAT_X0 + (T.N_SLATS - 1) * T.PITCH  # sheet on the mat
    assert T.NORI_LINK[1] / 2 <= 0.0951 and T.SLAT[1] / 2 >= 0.1199  # sheet within the block; slat ends overhang
    assert math.isclose(p["l0"][2, 3] - T.NORI_LINK[2] / 2, T.BOARD_TOP + T.SLAT[2], abs_tol=1e-6)  # on the slats
    assert math.isclose(p["core"][0, 3], p["l0"][0, 3], abs_tol=1e-9)  # pressed into the near-edge strip
    assert math.isclose(p["core"][2, 3] - T.CORE_R, p["l0"][2, 3] + T.NORI_LINK[2] / 2, abs_tol=1e-6)
    assert math.isclose(T.CORE_X - T.MAT_X0, 0.065, abs_tol=1e-9)  # a 65 mm flap of mat behind the cucumber
    tack = T.NORI.joint("tack")
    assert tack.kind == "fixed" and {tack.parent, tack.child} == {"core", "l0"}
    assert len(T.NORI.moving_joints) == T.N_NORI - 1  # every strip hinged to the next
    nori, rice = T._STRIP
    assert nori.color == T.NORI_GREEN and rice.color == T.RICE_WHITE  # nori under, rice on top
    assert math.isclose(nori.pos[2] + nori.dims[2] / 2, rice.pos[2] - rice.dims[2] / 2, abs_tol=1e-9)
    assert math.isclose(nori.dims[2] + rice.dims[2], T.NORI_LINK[2], abs_tol=1e-9)
    assert math.isclose(T.LINK_SPAN_DEG, 33.09, abs_tol=0.05)


def test_near_edge_strip_ends_in_a_ramp_and_still_carries_the_cucumber():
    flat_nori_box, _, ramp = T._LEAD
    lo, hi = flat_nori_box.pos[0] - flat_nori_box.dims[0] / 2, flat_nori_box.pos[0] + flat_nori_box.dims[0] / 2
    assert math.isclose(lo, -0.001, abs_tol=1e-9) and math.isclose(hi, T.NORI_LINK[0] / 2, abs_tol=1e-9)  # under the axis
    c, h = np.array(ramp.pos), ramp.half_extents()
    inner = c + ramp.rot @ np.array([h[0], 0.0, -h[2]])  # the ramp face's two ends (strip frame)
    outer = c + ramp.rot @ np.array([-h[0], 0.0, -h[2]])
    assert np.allclose(inner[[0, 2]], [-0.001, -T.NORI_LINK[2] / 2], atol=1e-7)  # starts at the flat bottom
    assert np.allclose(outer[[0, 2]], [-T.NORI_LINK[0] / 2, T.NORI_LINK[2] / 2], atol=1e-7)  # ends at the top face
    assert 30.0 < T._RAMP_DEG < 32.0


def test_flat_sheet_at_reset_scores_zero():
    st = {"mat": mat_state(), "nori": flat_nori()}
    assert 50.0 < T.wrap_deg(st["nori"]) < 110.0  # the strips under the cucumber
    assert T.mat_curl_deg(st["mat"]) == 0.0
    assert run(hold(st))[0] == 0.0


# -- the ladder (on the state at the end) ---------------------------------------------------------------------------

def test_full_wrap_seam_down_mat_open_released_scores_one():
    st = good_states()
    assert T.wrap_deg(st["nori"]) >= 330.0 and T.seam_down(st["nori"])
    sc, why = run(finish(st))
    assert sc == 1.0, why


def test_mirrored_wrap_counts_the_same():
    assert run(finish(good_states(outer_deg=-25.0, outer="l0")))[0] == 1.0


def test_outer_end_90_deg_from_down_is_half():
    assert run(finish(good_states(outer_deg=90.0)))[0] == 0.5


def test_outer_end_lifted_off_the_roll_is_half():
    assert run(finish(good_states(outer_deg=10.0, outer_r=0.021)))[0] == 0.5


def test_cucumber_turned_15_deg_is_half():
    assert run(finish(good_states(tilt_deg=15.0)))[0] == 0.5


def test_one_mat_joint_bent_35_deg_is_three_quarters():
    assert run(finish(good_states(mat=mat_state({"j12": math.radians(35.0)}))))[0] == 0.75


def test_mat_raised_but_joints_small_is_three_quarters():
    poses = {n: pose(m[:3, 3] + [0, 0, 0.012], m[:3, :3]) for n, m in T.MAT.link_poses().items()}
    assert run(finish(good_states(mat=state(T.MAT, poses))))[0] == 0.75


def test_roll_off_the_mat_but_on_the_block_is_three_quarters():
    assert run(finish(good_states(x=0.512)))[0] == 0.75


def test_partial_wraps_score_on_the_state_now():
    core = core_pose()
    for m, want in ((5, 0.0), (6, 0.25), (9, 0.25), (10, 0.5)):  # 165, 199, 298 and 331 deg; l10 lies off the roll
        st = {"mat": mat_state(), "nori": arc_nori(core, m)}
        assert math.isclose(T.wrap_deg(st["nori"]), m * T.LINK_SPAN_DEG, abs_tol=0.5)
        assert run(finish(st))[0] == want, m


def test_unrolling_after_a_full_wrap_loses_the_credit():
    good = good_states()
    flat = {"mat": mat_state(), "nori": flat_nori(core_pose(x=T.CORE_X, z=T.CORE_Z0))}
    sc, why = run(finish(good, 1.0) + hold(flat, 2.0))  # peaked at ~440 deg, then lay flat again
    assert sc == 0.0, why
    loose = dict(good, nori=wrapped_nori(good["nori"].poses["core"], outer_deg=150.0))  # free end hangs open
    assert run(finish(good, 1.0) + hold(loose, 2.0))[0] == 0.5


# -- gates and whole-run history -----------------------------------------------------------------------------------

def test_cucumber_lifted_25_mm_once_then_returned_scores_zero():
    good = good_states()
    frames = finish(good, 4.0)
    up = dict(good, nori=wrapped_nori(core_pose(z=T.CORE_Z0 + 0.025)))
    frames[50] = (up, tools())
    sc, why = run(frames)
    assert sc == 0.0 and "lifted" in why


def test_cucumber_lifted_15_mm_is_allowed():
    good = good_states()
    frames = finish(good, 2.5)
    frames[20] = (dict(good, nori=wrapped_nori(core_pose(z=T.CORE_Z0 + 0.015))), tools())
    assert run(frames)[0] == 1.0


def test_cucumber_off_the_block_then_returned_scores_zero():
    good = good_states()
    frames = finish(good, 2.5)
    frames[15] = (dict(good, nori=wrapped_nori(core_pose(x=0.53))), tools())
    sc, why = run(frames)
    assert sc == 0.0 and "block" in why
    frames[15] = (dict(good, nori=wrapped_nori(core_pose(y=0.07))), tools())
    assert run(frames)[0] == 0.0


def test_a_pad_touching_the_food_once_scores_zero():
    good = good_states()
    c = good["nori"].poses["core"][:3, 3]
    for gap, want in ((0.003, 0.0), (0.006, 1.0)):  # pad tip beside the cucumber's end face
        tip = c + np.array([0.0, T.CORE_LEN / 2 + gap, 0.0])
        if gap > 0.005:
            tip[1] = max(tip[1], T.NORI_LINK[1] / 2 + gap)  # and clear of the sheet's side edge too
        near = tip + np.array([0.0, 0.0, 0.013])  # TCP 13 mm above the pad tips (pointing down)
        frames = finish(good, 0.5) + [(good, tools(near_point=near))] * 2 + hold(good, 2.0)
        sc, why = run(frames)
        assert sc == want, (gap, why)
        if want == 0.0:
            assert "touched" in why


def test_hand_roll_without_the_mat_scores_zero():
    """Wrapped while the mat lay flat: the sheet passed half a wrap before the mat was curled."""
    good = good_states()
    half = {"mat": mat_state(), "nori": arc_nori(core_pose(), 7)}  # 232 deg, mat flat
    sc, why = run(hold(half, 1.0) + hold(good, 2.0))
    assert sc == 0.0 and "curled only 0" in why
    late = hold(half, 1.0) + curl_phase() + hold(good, 2.0)  # curling the mat afterwards does not undo it
    assert run(late)[0] == 0.0


def test_mat_curled_120_deg_before_the_wrap_is_enough():
    good = good_states()
    pre = hold({"mat": curled_mat(125.0), "nori": flat_nori()}, 0.5)
    assert run(pre + hold(good, 2.0))[0] == 1.0
    pre = hold({"mat": curled_mat(110.0), "nori": flat_nori()}, 0.5)
    assert run(pre + hold(good, 2.0))[0] == 0.0


def test_gripper_25_mm_from_the_roll_scores_zero():
    good = good_states()
    o = good["nori"].poses["core"][:3, 3]
    near = o + np.array([0.0, 0.0, T.CORE_R + 0.0034 + 0.025 + 0.013])  # pad tips 25 mm above the roll
    sc, why = run(finish(good, 2.5, tools(near_point=near)))
    assert sc == 0.0 and "clear" in why


def test_release_held_only_0_9_s_scores_zero_then_1_s_scores_one():
    good = good_states()
    busy = tools(speed=0.05)
    assert run(finish(good, 1.0, busy) + hold(good, 0.9))[0] == 0.0
    assert run(finish(good, 1.0, busy) + hold(good, 1.05))[0] == 1.0


def test_moving_roll_fails_the_release_gate():
    good = good_states()
    rolling = dict(good, nori=wrapped_nori(good["nori"].poses["core"], speed=0.02))
    assert run(finish(good, 1.0) + hold(rolling, 0.2))[0] == 0.0


def test_one_strip_chattering_in_place_does_not_block_the_release():
    import dataclasses

    good = good_states()
    vels = {n: np.zeros(3) for n in good["nori"].poses}
    vels["l5"] = np.array([0.0, 0.0, 0.015])  # a 3 g strip chattering in contact, the roll itself at rest
    chatter = dict(good, nori=dataclasses.replace(good["nori"], lin_vels=vels))
    assert run(finish(chatter, 2.0))[0] == 1.0


def test_success_needs_the_full_score():
    good = good_states()
    T.H.reset()
    for st, tl in finish(good, 1.5):
        T.H._tick(T.H.t + DT)
        T.update_history(T.H, st, tl)
    assert T.success(good) and T.score(good) == 1.0 and T.distance(good) == 0.0
    half = good_states(outer_deg=90.0)
    assert not T.success(half) and T.distance(half) == 0.5
    T.H.reset()


# -- the scripted demo -----------------------------------------------------------------------------------------------

def test_demo_stops_on_observed_quantities_only():
    """The roll stops on the cucumber's centre against the sheet's far edge as seen flat before the roll."""
    o = dict(T.OPTS_DEFAULT)

    class Seen:
        def __init__(self, x):
            self.x = x

        def point(self, obj, link="", local=(0.0, 0.0, 0.0)):
            assert (obj, link) == ("nori", "core")
            return np.array([self.x, 0.0, T.CORE_Z0])

    assert not T._rolled_out(Seen(0.40), 0.4186, o)
    assert T._rolled_out(Seen(0.4186 - o["stop"]), 0.4186, o)


def test_demo_waypoints_are_reachable():
    """Run the demo script against a stand-in scene (design poses; the cucumber rolls half as far as the mat edge is
    drawn) and check every commanded pose solves within 3 mm and 2 deg."""
    import linkdemo

    class Emb:
        class rig:
            from yam_arena.station import BASES as bases

    class FakeCtx(linkdemo.Ctx):
        def __init__(self):
            super().__init__(Emb(), "")

        def observe(self):
            roll = getattr(self, "roll", 0.0)
            nori = flat_nori(core_pose(x=T.CORE_X + roll, z=T.CORE_Z0))
            return linkdemo.Seen({"mat": mat_state(), "nori": nori}, {}, lk._SPECS)

    ctx = FakeCtx()
    orig = T._hold_slat
    start = {}

    def hold_slat(ctx_, c, th, width=0.0):
        orig(ctx_, c, th, width)
        if 170.0 <= th <= 181.0 and "x" not in start:
            start["x"] = float(c[0])
        if "x" in start:  # during the roll the cucumber follows at half the mat edge's travel
            ctx.roll = max(getattr(ctx, "roll", 0.0), (float(c[0]) - start["x"]) / 2 - 0.004)

    T._hold_slat = hold_slat
    try:
        steps = 0
        for _ in T.demo_script(ctx):
            steps += 1
            assert steps < 1200
    finally:
        T._hold_slat = orig
    assert T.CORE_X + ctx.roll >= 0.4188 - T.OPTS_DEFAULT["stop"]  # it rolled to the far edge
    for arm in (ctx.left, ctx.right):
        assert arm.worst_mm < 3.0 and arm.worst_deg < 2.0, (arm.side, arm.worst_mm, arm.worst_deg)
    assert steps * DT < 55.0  # leaves time for the release hold inside 1,200 steps (60 s)
