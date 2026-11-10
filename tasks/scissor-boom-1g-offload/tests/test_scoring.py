# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Screw, scorer and geometry edge cases for scissor-boom-1g-offload, pure Python (no simulator). Run from the task
folder: pytest -q tests

The screw model and the whole-run Record are plain Python; the tests turn the screw with synthetic crank angles and
feed the Record synthetic samples. The geometry tests use the boom's own link specs (masses, design poses) and the
scissor kinematics in task.py.
"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control
FULL_MM = T.Q_FULL * 1000.0


def sample(ext=0.0, **kw):
    m = {"q_mm": 0.0, "hinge_deg": 0.5, "ext": ext, "clear_mm": 28.0, "strain_mm": 0.1, "sling_deg": 2.0, "full": False,
         "latched": False, "clear": {"left": 0.08, "right": 0.12}, "tcp_speed": {"left": 0.0, "right": 0.0},
         "touch_mm": 40.0, "touch_by": "right finger at the boom's tip", "on_sling": True, "moving": True}
    m.update(kw)
    return m


def feed(rec, samples, t0=0.0):
    t = t0
    for m in samples:
        t += DT
        rec.update(m, t)
    return t


HELD = {"clear": {"left": 0.0, "right": 0.0}, "tcp_speed": {"left": 0.05, "right": 0.03}}


def deploy(rec, t0=0.0, **kw):
    """Crank and carry out to full length and latch, both hands on."""
    out = [sample(k / 40, q_mm=FULL_MM * k / 40, **{**HELD, **kw}) for k in range(41)]
    t = feed(rec, out, t0)
    return feed(rec, [sample(1.0, q_mm=FULL_MM, full=True, latched=True, **{**HELD, **kw})] * 10, t)


def demo_record(n_free=30, **kw):
    rec = T.Record()
    t = deploy(rec)
    feed(rec, [sample(1.0, q_mm=FULL_MM, full=True, latched=True, **kw)] * n_free, t)
    return rec


# -- the screw -----------------------------------------------------------------------------------------------------


def turn(screw, phis, q_of=lambda sc: (sc.s, sc.s)):
    for phi in phis:
        screw.step(phi, *q_of(screw))


def wrapped(a0, a1, n=41):
    return [math.atan2(math.sin(a), math.cos(a)) for a in np.linspace(a0, a1, n)]


def test_one_turn_is_one_lead_and_wraps_across_pi():
    s = T.Screw()
    turn(s, wrapped(0.0, 2 * math.pi))
    assert s.s == pytest.approx(T.LEAD, abs=1e-9)
    turn(s, wrapped(2 * math.pi, 0.0))
    assert s.s == pytest.approx(0.0, abs=1e-9)


def test_full_length_is_just_under_three_turns_and_stops_there():
    s = T.Screw()
    assert T.Q_FULL / T.LEAD == pytest.approx(2.98, abs=0.01)
    turn(s, wrapped(0.0, 2 * math.pi * 3.5, 200))
    assert s.s == pytest.approx(T.Q_FULL, abs=1e-12)
    turn(s, wrapped(0.0, -2 * math.pi * 4.0, 200))
    assert s.s == 0.0


def test_self_locking_nothing_but_the_crank_moves_the_screw():
    s = T.Screw()
    turn(s, wrapped(0.0, math.pi))
    s0 = s.s
    for q in (0.0, s0 + 0.010, s0 - 0.003, T.Q_FULL):  # boom pulled out, pushed in: crank held at pi
        s.step(math.pi, q, q)
    assert s.s == s0


def test_slip_clutch_keeps_the_screw_1p5_mm_ahead_of_a_stalled_slider_and_backs_off_at_once():
    s = T.Screw()
    turn(s, wrapped(0.0, 4 * math.pi, 81), q_of=lambda sc: (0.0, 0.0))  # two turns against a held tip
    assert s.s == pytest.approx(T.SLIP, abs=1e-12)
    assert s.slipped == pytest.approx(2 * T.LEAD - T.SLIP, abs=1e-9)
    s.step(4 * math.pi - 0.3, 0.0, 0.0)  # reverse: the screw backs off with the first reverse step
    assert s.s < T.SLIP


def test_latched_screw_is_locked_and_reset_clears_it():
    s = T.Screw()
    turn(s, wrapped(0.0, 2 * math.pi * 3.2, 200))
    s.locked = True
    turn(s, wrapped(0.0, -2 * math.pi, 41))
    assert s.s == pytest.approx(T.Q_FULL)
    s.reset()
    assert (s.s, s.phi, s.locked, s.slipped) == (0.0, None, False, 0.0)


# -- the record and the ladder -------------------------------------------------------------------------------------


def test_demo_shaped_record_scores_one():
    assert T.ladder(demo_record())[0] == 1.0


def test_release_gate_needs_one_second_clear_and_still():
    assert T.ladder(demo_record(n_free=19))[0] == 0.75  # 0.95 s
    assert T.ladder(demo_record(n_free=20))[0] == 1.0
    assert T.ladder(demo_record(clear={"left": 0.029, "right": 0.1}))[0] == 0.75
    assert T.ladder(demo_record(clear={"left": 0.030, "right": 0.1}))[0] == 1.0
    assert T.ladder(demo_record(tcp_speed={"left": 0.006, "right": 0.0}))[0] == 0.75


def test_release_restarts_when_a_hand_comes_back():
    rec = demo_record(n_free=15)
    t = rec.H.t
    t = feed(rec, [sample(1.0, full=True, latched=True, **HELD)], t)
    feed(rec, [sample(1.0, full=True, latched=True)] * 15, t)
    assert T.ladder(rec)[0] == 0.75


@pytest.mark.parametrize("clear_mm,expect", [(9.9, 0.0), (10.0, 1.0), (-0.5, 0.0)])
def test_cells_clearance_gate_holds_for_the_whole_run(clear_mm, expect):
    rec = T.Record()
    t = feed(rec, [sample(0.3, clear_mm=clear_mm, **HELD)], 0.0)  # one step, early
    t = deploy(rec, t)
    feed(rec, [sample(1.0, full=True, latched=True)] * 30, t)
    assert T.ladder(rec)[0] == expect


@pytest.mark.parametrize("hinge,expect", [(3.0, 1.0), (3.01, 0.0), (-3.0, 1.0), (-3.01, 0.0)])
def test_hinge_gate_droop_and_pushed_up(hinge, expect):
    rec = T.Record()
    t = feed(rec, [sample(0.6, hinge_deg=hinge, **HELD)], 0.0)
    t = deploy(rec, t)
    feed(rec, [sample(1.0, full=True, latched=True)] * 30, t)
    assert T.ladder(rec)[0] == expect


@pytest.mark.parametrize("sling,expect", [(15.0, 1.0), (15.01, 0.0), (-15.0, 1.0), (-15.01, 0.0)])
def test_sling_gate_behind_and_ahead(sling, expect):
    rec = T.Record()
    t = feed(rec, [sample(0.4, sling_deg=sling, **HELD)], 0.0)  # one step, mid-stroke
    t = deploy(rec, t)
    feed(rec, [sample(1.0, full=True, latched=True)] * 30, t)
    assert T.ladder(rec)[0] == expect


def test_sling_gate_is_whole_run_and_reset_clears_it():
    rec = T.Record()
    t = feed(rec, [sample(0.3, sling_deg=25.0, **HELD)] * 3, 0.0)
    t = feed(rec, [sample(0.3, **HELD)] * 10, t)  # caught up again
    t = deploy(rec, t)
    feed(rec, [sample(1.0, full=True, latched=True)] * 30, t)
    assert T.ladder(rec)[0] == 0.0 and "behind its pin" in T.ladder(rec)[1]
    rec.reset()
    deploy(rec)
    feed(rec, [sample(1.0, full=True, latched=True)] * 30, rec.H.t)
    assert T.ladder(rec)[0] == 1.0


def test_sling_knocked_before_cranking_does_not_gate():
    """A brush that swings the 10 g bail while the tip is still (reaching, grasping, letting go) is not a carry."""
    rec = T.Record()
    t = feed(rec, [sample(0.0, sling_deg=35.0, moving=False, **HELD)] * 5, 0.0)
    t = deploy(rec, t)
    feed(rec, [sample(1.0, full=True, latched=True, sling_deg=-30.0, moving=False)] * 30, t)
    assert T.ladder(rec)[0] == 1.0


def test_free_sling_swinging_while_the_tip_moves_does_not_gate():
    """With no hand on it the bail carries nothing; the other gates judge a run that never uses it."""
    rec = T.Record()
    t = feed(rec, [sample(0.3, sling_deg=25.0, on_sling=False, **HELD)] * 3, 0.0)
    t = deploy(rec, t)
    feed(rec, [sample(1.0, full=True, latched=True)] * 30, t)
    assert T.ladder(rec)[0] == 1.0


@pytest.mark.parametrize("touch,expect", [(4.99, 0.0), (5.0, 1.0), (0.0, 0.0)])
def test_touch_gate_holds_for_the_whole_run(touch, expect):
    rec = T.Record()
    t = feed(rec, [sample(0.2, touch_mm=touch, **HELD)], 0.0)  # one step, early
    t = deploy(rec, t)
    feed(rec, [sample(1.0, full=True, latched=True)] * 30, t)
    assert T.ladder(rec)[0] == expect
    if expect == 0.0:
        assert "only the sling may be handled" in T.ladder(rec)[1]
        assert T.ladder(rec, skip=("touch",))[0] == 1.0  # the report's what-if, never the score


def test_earliest_gate_is_reported():
    rec = T.Record()
    t = feed(rec, [sample(0.2, touch_mm=1.0, **HELD)], 0.0)
    t = feed(rec, [sample(0.6, clear_mm=2.0, **HELD)], t)
    assert T.ladder(rec)[0] == 0.0 and "only the sling" in T.ladder(rec)[1]
    assert "from the cells" in T.ladder(rec, skip=("touch",))[1]


def _down(closing_y: bool = True) -> np.ndarray:
    """Straight-down tool rotation (x: closing axis, z: approach)."""
    x = np.array([0.0, 1.0, 0.0]) if closing_y else np.array([1.0, 0.0, 0.0])
    z = np.array([0.0, 0.0, -1.0])
    return np.column_stack([x, np.cross(z, x), z])


def test_finger_points_fill_the_fingers_closely():
    tool = lk.Tool("right", np.zeros(3), np.eye(3), 0.012, 0.0)
    pts = T.finger_points(tool)
    assert len(pts) == 2 * 9 * 5 * 3
    assert np.abs(pts[:, 0]).min() == pytest.approx(0.006) and np.abs(pts[:, 0]).max() == pytest.approx(0.013)
    assert pts[:, 2].min() == pytest.approx(-0.035) and pts[:, 2].max() == pytest.approx(T.PAD_TIP)
    assert np.linalg.norm(pts, axis=1).max() <= T.FINGER_REACH
    zs = np.unique(np.round(pts[:, 2], 6))
    assert np.diff(zs).max() <= 0.006 + 1e-9


def test_touch_reads_the_hand_on_the_sleeve_as_the_sling_and_on_the_canister_as_a_touch():
    poses = T.BOOM.link_poses()
    apex = poses["tip"][:3, 3]
    # the demo's grasp: pads across the sleeve along the boom, 4 mm below its axle
    on_sleeve = lk.Tool("right", apex + [0.0, 0.0, T.GRASP_Z], _down(), 0.012, 0.0)
    # pads across the canister at mid-height
    on_can = lk.Tool("right", apex + [0.0, 0.0, T.CAN_BOTTOM + 0.015 + T.PAD_TIP], _down(False), 2 * T.CAN_R, 0.0)
    far = lk.Tool("left", apex + [0.0, 0.3, 0.2], _down(), 0.04, 0.0)
    h = T.hands(poses, T.BOOM, {"right": on_sleeve, "left": far})
    assert h["right"][2] <= T.TOUCH_MM and h["right"][0] > 25.0  # on the sling, well clear of everything else
    assert h["left"] == (999.0, "", 999.0)
    h = T.hands(poses, T.BOOM, {"right": on_can})
    assert h["right"][0] < 1.0 and T.near_mm(poses, T.BOOM, ("tip",), T.finger_points(on_can)) < 1.0


def test_sling_angle_sign():
    pin = np.array([0.3, -0.1, 0.85])
    assert T.sling_deg(pin, pin + [0.0, 0.0, T.BAIL_L]) == pytest.approx(0.0)
    behind = pin + [0.0, T.BAIL_L * math.sin(math.radians(20)), T.BAIL_L * math.cos(math.radians(20))]
    assert T.sling_deg(pin, behind) == pytest.approx(20.0)  # grip toward the root (+y): the hand lags
    assert T.sling_deg(pin, behind * [1, -1, 1] + [0, 2 * pin[1], 0]) == pytest.approx(-20.0)
    # 15 deg is a 10.4 mm offset of the grip over its 40 mm sling
    assert T.BAIL_L * math.sin(math.radians(T.SLING_DEG)) == pytest.approx(0.0104, abs=1e-4)


def test_ladder_rungs():
    rec = T.Record()
    feed(rec, [sample(0.49)])
    assert T.ladder(rec)[0] == 0.0
    rec = T.Record()
    feed(rec, [sample(0.5)])
    assert T.ladder(rec)[0] == 0.25
    feed(rec, [sample(1.0, full=True)])
    assert T.ladder(rec)[0] == 0.5
    feed(rec, [sample(1.0, full=True, latched=True, **HELD)])
    assert T.ladder(rec)[0] == 0.75


def test_reset_clears_the_record():
    rec = T.Record()
    feed(rec, [sample(0.6, clear_mm=0.0, hinge_deg=12.0)])
    assert T.ladder(rec)[0] == 0.0
    rec.reset()
    rec.H._tick(0.0)
    deploy(rec)
    feed(rec, [sample(1.0, full=True, latched=True)] * 30, rec.H.t)
    assert T.ladder(rec)[0] == 1.0


# -- geometry and statics ------------------------------------------------------------------------------------------


def test_scissor_kinematics():
    assert T.U_S == pytest.approx(0.1418, abs=5e-4)
    assert T.U_D == pytest.approx(0.3146, abs=5e-4)
    assert T.U_D - T.U_S == pytest.approx(0.173, abs=1e-3)
    assert T.Q_FULL == pytest.approx(0.0328, abs=1e-4)
    qs = np.linspace(0.0, T.Q_FULL, 50)
    us = [T.u_tip(q) for q in qs]
    assert all(b > a for a, b in zip(us, us[1:]))  # the tip only moves out
    rate = np.diff(us) / np.diff(qs)
    assert rate[0] / rate[-1] > 4.0  # it runs out over 4x faster per mm of screw early than late: an uneven pace


def test_design_pose_closes_every_loop():
    poses = T.BOOM.link_poses()
    for name in T.LOOPS:
        j = T.BOOM.joint(name)
        a = np.r_[np.asarray(j.anchor), 1.0]
        p = poses[j.parent] @ np.linalg.inv(T.BOOM.link(j.parent).design) @ a
        c = poses[j.child] @ np.linalg.inv(T.BOOM.link(j.child).design) @ a
        assert np.linalg.norm((p - c)[:3]) < 1e-9


def _moment(q):
    """Gravity moment (N m) of everything outboard of the hinge, the boom level, at slider travel q."""
    c = T.L_BAR * math.cos(T.phi_of(q))
    m = {lk.name: lk.mass for lk in T.BOOM.links}
    arms = {"root": 0.012, "sp": T.R0, "sn": T.R0, "a1": T.R0 + c / 2, "b1": T.R0 + c / 2, "a2": T.R0 + 1.5 * c,
            "b2": T.R0 + 1.5 * c, "c": T.R0 + 2.25 * c, "d": T.R0 + 2.25 * c, "tip": T.R0 + 2.5 * c,
            "bail": T.R0 + 2.5 * c, "sleeve": T.R0 + 2.5 * c}
    return 9.81 * sum(m[k] * u for k, u in arms.items())


def _droop_deg(q):
    return math.degrees(_moment(q) / T.HINGE_K) + T.HINGE_REST_DEG


def test_spring_holds_the_stowed_boom_near_level_but_not_the_deployed_one():
    assert 0.0 < _droop_deg(0.0) < 2.0
    assert _droop_deg(T.Q_FULL) > 10.0


def test_level_clearance_and_where_cranking_alone_fails():
    level = T.ORIGIN[2] + T.CAN_BOTTOM - T.CELLS_TOP
    assert level == pytest.approx(0.030, abs=1e-9)

    def clear_mm(q, deg):
        u = T.u_tip(q) + T.CAN_R
        return 1000 * (level - u * math.sin(math.radians(deg)))

    # carried within the 3 deg gate, the canister still clears the cells by more than 10 mm at full length (12.6 mm)
    assert clear_mm(T.Q_FULL, T.LEVEL_DEG) > T.CLEAR_MIN_MM + 2.0
    # cranked alone, the unsupported boom crosses the 10 mm gate before it is 60% out and reaches the cells
    qs = np.linspace(0.0, T.Q_FULL, 400)
    gate = next(q for q in qs if clear_mm(q, _droop_deg(q)) < T.CLEAR_MIN_MM)
    assert (T.u_tip(gate) - T.U_S) / (T.U_D - T.U_S) < 0.6
    assert clear_mm(T.Q_FULL, _droop_deg(T.Q_FULL)) < 0.0


def test_latch_window_is_inside_the_level_gate():
    assert T.LATCH_DEG < T.LEVEL_DEG
    assert T.LATCH_Q < 0.001
