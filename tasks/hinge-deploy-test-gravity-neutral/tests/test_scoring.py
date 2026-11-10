# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer, worm-drive, physics and geometry edge cases for hinge-deploy-test-gravity-neutral. They never start Isaac
Sim, but task.py and linkkit.py import yam_arena.task_file, so run them where yam_arena is importable (the Brev
launchable, after sourcing yam.env). Run from the task folder: pytest -q tests

The whole-run Record and the ladder are plain Python; the tests feed the Record synthetic samples at 20 Hz. The worm
is driven with crank readings; the geometry tests use the fixture's own link poses (Linkage.link_poses)."""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control
G = 9.81


def sample(**kw):
    """A finished state: fixture at 90 deg, released, latched, panel still, grippers clear and still."""
    m = {"axis_err": 0.0, "tilt": 90.0, "pin_mm": 16.0, "released": True, "q": 89.9, "latched": True,
         "touch": {"left": 0.20, "right": 0.08}, "clear": {"left": 0.15, "right": 0.07},
         "tcp_speed": {"left": 0.0, "right": 0.0}, "panel_mm_s": 0.0, "panel_deg_s": 0.0}
    m.update(kw)
    return m


def feed(samples, rec=None, t0=0.0):
    rec = rec or T.Record()
    t = t0
    for m in samples:
        t += DT
        rec.update(m, t)
    return rec


def demo_trace(release_err=0.4, touch_mm=40.0, end=None, end_steps=40):
    """Stowed and turned, released at 90 deg, swings to the latch in 0.5 s, then still with the hands clear."""
    s = [sample(axis_err=90.0, tilt=0.0, pin_mm=0.0, released=False, q=0.0, latched=False,
                clear={"left": 0.0, "right": 0.2}, tcp_speed={"left": 0.05, "right": 0.0})] * 40
    s += [sample(axis_err=90.0 - a, tilt=a, pin_mm=0.0, released=False, q=0.0, latched=False,
                 clear={"left": 0.0, "right": 0.2}) for a in np.linspace(0.0, 90.0 - release_err, 60)]
    s += [sample(axis_err=release_err, tilt=90.0 - release_err, pin_mm=5.0, released=False, q=0.0, latched=False,
                 clear={"left": 0.1, "right": 0.0}, touch={"left": 0.2, "right": 0.045})] * 10
    s += [sample(axis_err=release_err, tilt=90.0 - release_err, pin_mm=9.0 + i, q=9.0 * i, latched=9.0 * i >= 89.0,
                 clear={"left": 0.1, "right": 0.0}, touch={"left": 0.2, "right": touch_mm / 1000.0},
                 panel_mm_s=300.0, panel_deg_s=200.0) for i in range(11)]
    s += [sample(axis_err=release_err, tilt=90.0 - release_err, clear={"left": 0.1, "right": 0.01},
                 tcp_speed={"left": 0.0, "right": 0.05})] * 30
    s += [end or sample(axis_err=release_err, tilt=90.0 - release_err)] * end_steps
    return s


# -- ladder --------------------------------------------------------------------------------------------------------


def test_demo_shaped_trace_scores_one():
    rec = feed(demo_trace())
    score, why = T.ladder(rec)
    assert score == 1.0, why
    assert rec.release["axis_err"] == pytest.approx(0.4)


def test_nothing_scores_zero():
    rec = feed([sample(axis_err=90.0, tilt=0.0, pin_mm=0.0, released=False, q=0.0, latched=False)] * 100)
    assert T.ladder(rec)[0] == 0.0


def test_turned_but_not_released_is_a_quarter():
    s = [sample(axis_err=90.0 - a, tilt=a, released=False, pin_mm=0.0, q=0.0, latched=False)
         for a in np.linspace(0, 80.0, 50)]
    assert T.ladder(feed(s))[0] == 0.25  # axis 10.0 deg off vertical counts as turned
    s = [sample(axis_err=90.0 - a, tilt=a, released=False, pin_mm=0.0, q=0.0, latched=False)
         for a in np.linspace(0, 79.9, 50)]
    assert T.ladder(feed(s))[0] == 0.0


@pytest.mark.parametrize("err, gated", [(10.0, False), (10.1, True), (45.0, True), (90.0, True)])
def test_release_orientation_gate(err, gated):
    score, why = T.ladder(feed(demo_trace(release_err=err)))
    assert (score == 0.0) is gated, why
    if not gated:
        assert score == 1.0


def test_pull_first_then_turn_stays_gated():
    """The release was made as delivered; turning the fixture and a full latch afterwards never clear it."""
    s = [sample(axis_err=90.0, tilt=0.0, pin_mm=12.0, released=True, q=0.0, latched=False)] * 40
    s += [sample(axis_err=90.0 - a, tilt=a, pin_mm=12.0, q=max(0.0, a - 60.0) * 3, latched=a >= 90)
          for a in np.linspace(0, 90, 60)]
    s += [sample()] * 60
    rec = feed(s)
    score, why = T.ladder(rec)
    assert score == 0.0 and "90.0 deg from vertical" in why
    assert rec.release["tilt"] == 0.0


@pytest.mark.parametrize("mm, gated", [(5.0, False), (4.9, True), (0.0, True)])
def test_touching_the_panel_after_release_gates(mm, gated):
    score, why = T.ladder(feed(demo_trace(touch_mm=mm)))
    assert (score == 0.0) is gated, why


def test_touch_before_release_does_not_count():
    s = [sample(released=False, latched=False, q=0.0, pin_mm=0.0, touch={"left": 0.2, "right": 0.0})] * 20
    s += demo_trace()
    assert T.ladder(feed(s))[0] == 1.0


def test_touch_long_after_latch_still_gates():
    s = demo_trace() + [sample(touch={"left": 0.2, "right": 0.003})] + [sample()] * 40
    assert T.ladder(feed(s))[0] == 0.0


def test_valid_release_without_latch_is_a_half():
    s = [sample(released=True, latched=False, q=6.0)] * 60
    score, why = T.ladder(feed(s))
    assert score == 0.5 and "6.0 deg" in why


@pytest.mark.parametrize("end", [
    sample(q=87.9),  # pawl at the end below 88 deg (cannot happen in the sim; the ladder checks it anyway)
    sample(latched=False),
    sample(axis_err=10.1, tilt=79.9),  # fixture turned back after the deploy
    sample(panel_mm_s=3.0),
    sample(panel_deg_s=3.0),
])
def test_latched_but_end_state_wrong_is_three_quarters(end):
    assert T.ladder(feed(demo_trace(end=end)))[0] == 0.75


def test_release_gate_timing():
    assert T.ladder(feed(demo_trace(end_steps=20)))[0] == 1.0  # 1.00 s
    assert T.ladder(feed(demo_trace(end_steps=19)))[0] == 0.75  # 0.95 s
    s = demo_trace() + [sample(clear={"left": 0.1, "right": 0.029})] + [sample()] * 19
    assert T.ladder(feed(s))[0] == 0.75  # a pad at 29 mm restarts the timer
    s = demo_trace() + [sample(clear={"left": 0.1, "right": 0.030})] + [sample()] * 19
    assert T.ladder(feed(s))[0] == 1.0  # 30 mm is clear
    s = demo_trace() + [sample(tcp_speed={"left": 0.006, "right": 0.0})] + [sample()] * 19
    assert T.ladder(feed(s))[0] == 0.75  # a hand at 6 mm/s restarts it


def test_record_is_clean_after_reset():
    rec = feed(demo_trace(release_err=45.0))
    assert T.ladder(rec)[0] == 0.0
    rec.reset()
    assert rec.release is None and rec.last is None and not rec.H.evers
    assert T.ladder(feed(demo_trace(), rec))[0] == 1.0


def test_only_the_first_release_counts():
    s = demo_trace(release_err=0.4)
    s += [sample(axis_err=30.0, tilt=60.0)] * 5 + [sample()] * 40
    rec = feed(s)
    assert rec.release["axis_err"] == pytest.approx(0.4)


# -- worm drive ---------------------------------------------------------------------------------------------------


def turn(worm, deg, step=7.0, wrapped=False, q0=0.0):
    """Feed crank readings in steps of ``step`` deg (``wrapped``: readings wrapped to +-180 deg, as a joint without
    limits may report them)."""
    q = q0
    n = int(abs(deg) // step)
    out = None
    for _ in range(n):
        q += math.copysign(step, deg)
        out = worm.step(T.wrap(math.radians(q)) if wrapped else math.radians(q))
    rest = deg - math.copysign(step * n, deg)
    if rest:
        q += rest
        out = worm.step(T.wrap(math.radians(q)) if wrapped else math.radians(q))
    return math.degrees(out), q


@pytest.mark.parametrize("wrapped", [False, True])
def test_three_crank_turns_tilt_90(wrapped):
    w = T.Worm()
    tilt, _ = turn(w, 1080.0, wrapped=wrapped)
    assert tilt == pytest.approx(90.0, abs=1e-9)
    w = T.Worm()
    tilt, _ = turn(w, 540.0, wrapped=wrapped)
    assert tilt == pytest.approx(45.0, abs=1e-9)


def test_worm_stops_at_0_and_95():
    w = T.Worm()
    tilt, q = turn(w, -400.0)  # backwards from the delivered position: the table stays at 0
    assert tilt == 0.0
    tilt, q = turn(w, 360.0, q0=q)
    assert tilt == pytest.approx(30.0)
    tilt, q = turn(w, 1500.0, q0=q)  # past the 95 deg stop
    assert tilt == pytest.approx(95.0)
    tilt, q = turn(w, -60.0, q0=q)
    assert tilt == pytest.approx(90.0)


def test_worm_reset():
    w = T.Worm()
    turn(w, 500.0)
    w.reset(math.radians(37.0))
    assert w.acc == 0.0 and w.step(math.radians(37.0)) == 0.0


# -- physics: why the natural plan fails ----------------------------------------------------------------------------


def spring(q_rad):
    return T.SPRING_K * (math.radians(T.SPRING_REST_DEG) - q_rad)


def gravity(q_rad, tilt_deg, r=0.060):
    """Gravity torque about the hinge resisting the deploy, with the fixture at ``tilt_deg`` (0: axis horizontal)."""
    return T.PANEL_MASS * G * r * math.cos(math.radians(tilt_deg)) * math.cos(q_rad)


def test_board_centre_is_60_mm_from_the_hinge():
    assert T.HINGE_Y - T.PANEL_C[1] == pytest.approx(0.060)
    assert T.PANEL_C[2] - T.HINGE_Z == pytest.approx(0.002)


def test_spring_loses_to_gravity_as_delivered():
    assert spring(0.0) == pytest.approx(0.050, abs=5e-4)
    assert spring(math.radians(90)) == pytest.approx(0.020, abs=5e-4)
    assert gravity(0.0, 0.0) == pytest.approx(0.0883, abs=5e-4)
    assert spring(0.0) < gravity(0.0, 0.0)  # pulled as delivered, the panel never leaves its stowed stop


def test_partial_tilt_fails_and_vertical_axis_deploys():
    qs = np.radians(np.linspace(0.0, 90.0, 181))

    def deploys(tilt):
        return all(spring(q) - gravity(q, tilt) - T.DEPLOY_F > 0 for q in qs)

    assert not deploys(45.0)
    assert not deploys(55.0)
    assert deploys(80.0) and deploys(90.0)
    # the threshold: the spring/gravity ratio is lowest about 26 deg into the swing
    worst = min(spring(q) / gravity(q, 0.0) for q in qs)
    assert 0.50 < worst < 0.53
    assert 58.0 < math.degrees(math.acos(worst)) < 61.0


# -- geometry ------------------------------------------------------------------------------------------------------


def poses(tilt=0.0, deploy=0.0, pull=0.0):
    return T.FIXTURE.link_poses({"tilt": math.radians(tilt), "deploy": math.radians(deploy), "pull": pull})


@pytest.mark.parametrize("tilt, err", [(0.0, 90.0), (45.0, 45.0), (80.0, 10.0), (90.0, 0.0), (95.0, 5.0)])
def test_axis_from_vertical(tilt, err):
    assert T.axis_from_vertical_deg(poses(tilt)["cradle"][:3, :3]) == pytest.approx(err, abs=1e-6)


def test_tilt_turns_the_panel_face_away_from_the_robot():
    p = poses(90.0)
    n = p["panel"][:3, :3] @ np.array([0.0, 0.0, 1.0])
    assert n == pytest.approx(np.array([1.0, 0.0, 0.0]), abs=1e-9)
    d = poses(90.0, 90.0)["panel"][:3, 3] - poses(90.0, 0.0)["panel"][:3, 3]
    assert d[2] == pytest.approx(0.0, abs=1e-9)  # with the axis vertical the board's centre swings level
    assert d[0] > 0.05  # outward, away from the robot


def test_deploy_swings_the_board_up_as_delivered():
    up = poses(0.0, 90.0)["panel"][:3, 3] - poses(0.0, 0.0)["panel"][:3, 3]
    assert up[2] == pytest.approx(0.058, abs=0.002)


def shape_distance(link_pose, shapes, point):
    p = link_pose[:3, :3].T @ (np.asarray(point) - link_pose[:3, 3])
    return min(s.distance(p) for s in shapes)


def test_pin_engagement():
    """Pin tip 6 mm into the boss's slot; 8 mm out it is 2 mm clear of the boss."""
    for pull, inside in ((0.0, True), (T.PIN_RELEASE_M, False)):
        p = poses(0.0, 0.0, pull)
        tip = p["pin"][:3, :3] @ np.array([0.0, 0.0455 + 0.0375, 0.0]) + p["pin"][:3, 3]
        assert bool(tip[1] - T.YF > -0.073) == inside  # the boss face is at F y -0.073
    tip0 = -0.150 + 0.0455 + 0.0375
    assert tip0 == pytest.approx(-0.067)
    assert -0.073 - (tip0 - T.PIN_RELEASE_M) == pytest.approx(0.002)


def test_deployed_panel_and_pin_clear_the_fixture():
    spec = T.FIXTURE
    stand = spec.link("stand").shapes
    for tilt in (0.0, 45.0, 90.0, 95.0):
        for q in (0.0, 45.0, 90.0):
            p = poses(tilt, q)
            corners = [p["panel"][:3, :3] @ np.array([sx * 0.05, sy * 0.06, 0.0]) + p["panel"][:3, 3]
                       for sx in (-1, 1) for sy in (-1, 1)]
            assert min(shape_distance(p["stand"], stand, c) for c in corners) > 0.02
        knob = poses(tilt)["pin"][:3, 3]
        assert shape_distance(poses(tilt)["stand"], stand, knob) > 0.05


def test_knob_and_handle_positions():
    k = poses(90.0)["pin"][:3, 3]
    assert k - np.array([T.XF, T.YF, T.TOP]) == pytest.approx(np.array([0.0125, -0.150, T.AX_Z]), abs=1e-9)
    h = T.CRANK.link_poses()["handle"][:3, 3]
    assert h - np.array([T.XF, T.YF, T.TOP]) == pytest.approx(np.array([-0.040, T.COL_Y, 0.170]), abs=1e-9)
