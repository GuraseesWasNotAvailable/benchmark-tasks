# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Screw, scorer and geometry edge cases for guide-rod-clamp-at-far-end, pure Python (no simulator). Run from the
task folder: pytest -q tests

The screw model and the whole-run Record are plain Python; the tests turn the screw with synthetic crank angles and
feed the Record synthetic samples. The geometry tests use the axis's own link poses (Linkage.link_poses) to find
where the bushing's walls meet rod B.
"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control
FAR = T.Q_FAR * 1000.0


def sample(q_mm=0.0, **kw):
    m = {"q_mm": q_mm, "lag_mm": 0.3, "rod_deg": 0.09, "lever_deg": 0.0, "clamped": True,
         "clear": {"left": 0.08, "right": 0.12}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def feed(rec, samples, t0=0.0):
    t = t0
    for m in samples:
        t += DT
        rec.update(m, t)
    return t


def ramp(a, b, n=40, **kw):
    return [sample(a + (b - a) * k / n, **kw) for k in range(n + 1)]


def demo_record(**kw):
    """Crank out unclamped, clamp at the far stop, crank home, let go."""
    rec = T.Record()
    held = {"clear": {"left": 0.0, "right": 0.12}, "tcp_speed": {"left": 0.05, "right": 0.0}}
    t = feed(rec, ramp(0.0, FAR, clamped=False, rod_deg=0.3, **held))
    t = feed(rec, [sample(FAR, **held)] * 10, t)
    t = feed(rec, ramp(FAR, 0.0, **held), t)
    feed(rec, [sample(0.0, **kw)] * 30, t)
    return rec


# -- the screw -----------------------------------------------------------------------------------------------------


def turn(screw, q_of, phis):
    out = []
    for phi in phis:
        out.append(screw.step(phi, q_of(screw)))
    return out


def test_one_turn_is_one_lead_and_wraps_across_pi():
    s = T.Screw()
    phis = [math.atan2(math.sin(a), math.cos(a)) for a in np.linspace(0.0, 2 * math.pi, 41)]  # wrapped readings
    turn(s, lambda sc: sc.s, phis)
    assert s.s == pytest.approx(T.LEAD, abs=1e-9)
    turn(s, lambda sc: sc.s, phis[::-1])
    assert s.s == pytest.approx(0.0, abs=1e-9)


def test_screw_stops_at_the_travel_ends():
    s = T.Screw()
    phis = np.linspace(0.0, 2 * math.pi * 6, 241)
    turn(s, lambda sc: sc.s, phis)
    assert s.s == pytest.approx(T.Q_FAR)
    turn(s, lambda sc: sc.s, -phis)  # back past home
    assert s.s == pytest.approx(0.0)


def test_slip_clutch_holds_the_screw_within_3_mm_of_a_stuck_carriage():
    s = T.Screw()
    res = turn(s, lambda sc: 0.005, np.linspace(0.0, 2 * math.pi, 41))
    assert s.s == pytest.approx(0.005 + T.SLIP)
    assert res[-1][1] * 1000 == pytest.approx(3.0)  # lag: 3 mm behind
    s.step(s.phi - 0.2, 0.005)  # a reverse turn moves it back at once
    assert s.s < 0.005 + T.SLIP


def test_reset_clears_the_screw():
    s = T.Screw()
    turn(s, lambda sc: sc.s, np.linspace(0.0, 3.0, 10))
    s.reset()
    assert s.s == 0.0 and s.phi is None


# -- the record and the ladder ------------------------------------------------------------------------------------


def test_demo_shaped_record_scores_1():
    sc, why = T.ladder(demo_record())
    assert sc == 1.0, why


def test_release_needs_a_full_second_clear_and_still():
    assert T.ladder(demo_record())[0] == 1.0
    rec = T.Record()
    t = feed(rec, ramp(0.0, FAR, clamped=False))
    t = feed(rec, [sample(FAR)] * 5, t)
    t = feed(rec, ramp(FAR, 0.0, clear={"left": 0.0, "right": 0.1}), t)
    feed(rec, [sample(0.0)] * 19, t)  # 0.95 s
    assert T.ladder(rec)[0] == 0.75
    feed(rec, [sample(0.0)], rec.H.t)
    assert T.ladder(rec)[0] == 1.0


@pytest.mark.parametrize("kw", [{"clear": {"left": 0.029, "right": 0.1}},
                                {"tcp_speed": {"left": 0.0, "right": 0.006}}])
def test_a_near_or_moving_hand_restarts_the_release_timer(kw):
    rec = demo_record()
    feed(rec, [sample(0.0, **kw)], rec.H.t)
    assert T.ladder(rec)[0] == 0.75


def test_release_at_exactly_30_mm_counts():
    assert T.ladder(demo_record(clear={"left": 0.030, "right": 0.030}))[0] == 1.0


@pytest.mark.parametrize("lag", [1.5, -1.5, 3.0])
def test_a_stuck_or_forced_carriage_gates_the_whole_run(lag):
    rec = T.Record()
    t = feed(rec, ramp(0.0, 6.0, clamped=True, rod_deg=0.5))
    t = feed(rec, [sample(6.0, lag_mm=lag, rod_deg=0.5)] * 3, t)  # 0.15 s
    t = feed(rec, [sample(6.0, clamped=False, lag_mm=0.0)] * 4, t)  # unclamp, then do it right
    t = feed(rec, ramp(6.0, FAR, clamped=False), t)
    t = feed(rec, ramp(FAR, 0.0), t)
    feed(rec, [sample(0.0)] * 30, t)
    sc, why = T.ladder(rec)
    assert sc == 0.0 and "stuck" in why


def test_brief_or_small_lag_is_not_a_jam():
    rec = T.Record()
    t = feed(rec, ramp(0.0, FAR, clamped=False, lag_mm=1.49))
    t = feed(rec, [sample(FAR, lag_mm=2.0)] * 2, t)  # 0.10 s only
    t = feed(rec, [sample(FAR)], t)
    t = feed(rec, [sample(FAR, lag_mm=2.0)] * 2, t)
    t = feed(rec, ramp(FAR, 0.0), t)
    feed(rec, [sample(0.0)] * 30, t)
    assert rec.jam is None
    assert T.ladder(rec)[0] == 1.0


@pytest.mark.parametrize("rod,expect", [(0.12, 1.0), (-0.12, 1.0), (0.121, 0.0), (0.5, 0.0)])
def test_rod_must_be_locked_within_0_12_deg(rod, expect):
    rec = T.Record()
    t = feed(rec, ramp(0.0, FAR, clamped=False))
    t = feed(rec, ramp(FAR, 0.0, rod_deg=rod), t)
    feed(rec, [sample(0.0, rod_deg=rod)] * 30, t)
    assert T.ladder(rec)[0] == expect


def test_unclamped_scores_0_even_after_the_runs():
    rec = T.Record()
    t = feed(rec, ramp(0.0, FAR, clamped=False, rod_deg=0.2))
    t = feed(rec, ramp(FAR, 0.0, clamped=False, rod_deg=0.5), t)
    feed(rec, [sample(0.0, clamped=False, rod_deg=0.5)] * 30, t)
    sc, why = T.ladder(rec)
    assert sc == 0.0 and "not clamped" in why


def test_clamped_square_but_never_at_the_far_stop_since_is_0_25():
    rec = T.Record()
    t = feed(rec, ramp(0.0, FAR, clamped=False))  # out
    t = feed(rec, ramp(FAR, 0.0, clamped=False), t)  # and back, then clamp (the rod would be skewed; here square)
    feed(rec, [sample(0.0)] * 30, t)
    assert T.ladder(rec)[0] == 0.25


def test_clamped_at_the_far_stop_without_the_check_run_is_0_5():
    rec = T.Record()
    t = feed(rec, ramp(0.0, FAR, clamped=False))
    feed(rec, [sample(FAR)] * 30, t)
    assert T.ladder(rec)[0] == 0.5


@pytest.mark.parametrize("far_mm,home_mm,expect", [(FAR - 1.0, 1.0, 1.0), (FAR - 1.1, 1.0, 0.25),
                                                   (FAR - 1.0, 1.1, 0.5)])
def test_stop_tolerance_is_1_mm(far_mm, home_mm, expect):
    rec = T.Record()
    t = feed(rec, ramp(0.0, far_mm, clamped=False))
    t = feed(rec, [sample(far_mm)] * 5, t)
    t = feed(rec, ramp(far_mm, home_mm), t)
    feed(rec, [sample(home_mm)] * 30, t)
    assert T.ladder(rec)[0] == expect


def test_far_then_home_order_matters():
    rec = T.Record()
    t = feed(rec, [sample(0.0)] * 5)  # clamped at home ...
    t = feed(rec, ramp(0.0, FAR), t)  # ... out to the far stop (it would jam; here it does not)
    feed(rec, [sample(FAR)] * 30, t)
    assert T.ladder(rec)[0] == 0.5  # the far stop alone is not the return


def test_reopening_the_clamp_restarts_its_record():
    rec = demo_record()
    t = feed(rec, [sample(0.0, clamped=False, lever_deg=50.0)] * 5, rec.H.t)
    assert T.ladder(rec)[0] == 0.0
    feed(rec, [sample(0.0)] * 30, t)  # closed again at home: the run since does not count
    assert T.ladder(rec)[0] == 0.25
    assert len(rec.clamps) == 2


def test_reset_clears_the_record():
    rec = demo_record()
    rec.update(sample(0.0, lag_mm=5.0), rec.H.t + 1.0)
    rec.reset()
    assert rec.jam is None and rec.since is None and rec.clamps == [] and rec.H.runs == {}
    assert T.ladder(rec) == (0.0, "no steps yet")


# -- geometry: where the bushing meets rod B (link poses at given joint values) ----------------------------------------


WALL_X = T.ROD_R + T.PLAY  # inner faces of bushing B's side walls (carriage frame)


def gaps(q, rod_deg):
    """Smallest clearance (m) from rod B's surface to bushing B's outer (+x) and inner (-x) walls."""
    poses = T.AXIS.link_poses({"slide": q, "pivot": math.radians(rod_deg)})
    car, rod = poses["carriage"], poses["rod"]
    rel = np.linalg.inv(rod) @ car  # carriage in the rod's frame (rod axis along y through x = z = 0)
    out = []
    for side in (1.0, -1.0):
        pts = [rel[:3, :3] @ np.array([side * WALL_X, y, 0.0]) + rel[:3, 3] for y in (-T.BUSH_L / 2, T.BUSH_L / 2)]
        out.append(min(abs(p[0]) - T.ROD_R for p in pts))
    return out


def touch_deg(q):
    """Rod angle at which the outer wall just touches the rod with the carriage at ``q`` (bisection)."""
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = (lo + hi) / 2
        (lo, hi) = (mid, hi) if gaps(q, mid)[0] > 0 else (lo, mid)
    return lo


def test_bushing_has_0_3_mm_play_each_side_on_a_square_rod():
    for q in (0.0, 0.08, T.Q_FAR):
        assert gaps(q, 0.0) == pytest.approx([0.0003, 0.0003], abs=1e-7)


def test_the_skewed_rod_clears_the_bushing_at_home_and_meets_it_4_to_5_mm_out():
    assert gaps(0.0, T.SKEW_DEG)[0] > 0.00002
    assert gaps(0.004, T.SKEW_DEG)[0] > 0
    assert gaps(0.005, T.SKEW_DEG)[0] < 0


def test_only_the_far_stop_squares_the_rod_within_tolerance():
    far, mid = touch_deg(T.Q_FAR), touch_deg(T.Q_FAR / 2)
    assert far == pytest.approx(math.degrees(0.0003 / 0.190), abs=1e-3)  # 0.090 deg
    assert far < T.ROD_TOL_DEG < mid
    assert mid == pytest.approx(0.156, abs=0.002)


def test_a_rod_locked_at_the_far_stop_clears_the_bushing_all_the_way_home():
    far = touch_deg(T.Q_FAR) - 1e-4
    assert all(gaps(q, far)[0] > 0 for q in np.linspace(0.0, T.Q_FAR, 33))


def test_a_rod_locked_part_way_overlaps_the_bushing_at_the_far_stop():
    mid = touch_deg(T.Q_FAR / 2)
    assert gaps(T.Q_FAR, mid)[0] < -0.0002  # 0.23 mm of interference


def test_carriage_clears_the_crank_and_the_lever_at_its_stops():
    poses = T.AXIS.link_poses({"slide": 0.0})
    car_y = poses["carriage"][1, 3]
    assert car_y + 0.015 < T.CRANK_AT[1] - T.CRANK_R - 0.007  # plate's left edge short of the handle's circle
    poses = T.AXIS.link_poses({"slide": T.Q_FAR, "cam": 0.0})
    assert poses["carriage"][1, 3] - 0.020 > poses["lever"][1, 3] + T.LEVER_LEN / 2  # housing's right end
