# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Gurasees Singh Dhanoa
"""Scorer edge cases for passive-elbow-rom, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is the Rom record (fed every control step) and ladder(); these tests feed it synthetic traces.
"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control


def ramp(a, b, rate):
    n = max(1, int(round(abs(b - a) / rate / DT)))
    return [a + (b - a) * k / n for k in range(1, n + 1)]


def hold(v, seconds):
    return [v] * int(round(seconds / DT))


def demo_angles(top=90.0):
    """Rest 1 s, flex at 25 deg/s, hold 1 s, extend at 25 deg/s, rest 3 s (what the scripted demo does)."""
    return hold(0.0, 1.0) + ramp(0.0, top, 25.0) + hold(top, 1.0) + ramp(top, 0.0, 25.0) + hold(0.0, 3.0)


def feed(angles, *, lift=None, low=None, clear=None, tcp_step=None, rom=None):
    """Feed one step per angle. lift/low/clear/tcp_step: per-step callables of (k, n) or constants.
    By default the grippers hold on until 2 s before the end, then sit 50 mm clear and still."""
    rom = rom or T.Rom()
    n = len(angles)
    pick = lambda f, k, d: (f(k, n) if callable(f) else f) if f is not None else d  # noqa: E731
    pos = {"left": np.zeros(3), "right": np.array([0.0, 0.3, 0.0])}
    for k, th in enumerate(angles):
        released = k >= n - 40
        step = pick(tcp_step, k, 0.0 if released else 0.002)
        for p in pos.values():
            p += [step, 0.0, 0.0]
        c = pick(clear, k, 0.05 if released else 0.0)
        rom.update((k + 1) * DT, th, pick(lift, k, 0.0), pick(low, k, 0.0 if th < 1 else 0.003),
                   {s: p.copy() for s, p in pos.items()}, {"left": c, "right": c})
    return rom


def score(rom):
    return T.ladder(rom)[0]


def test_demo_trace_scores_one():
    r = feed(demo_angles())
    assert score(r) == 1.0, T.ladder(r)


def test_nothing_scores_zero():
    assert score(feed(hold(0.0, 10.0))) == 0.0
    assert score(T.Rom()) == 0.0


def test_partial_flex_below_45_is_zero():
    assert score(feed(hold(0.0, 1.0) + ramp(0, 40, 25) + ramp(40, 0, 25) + hold(0.0, 3.0))) == 0.0


@pytest.mark.parametrize("top,expected", [(79.0, 0.25), (101.0, 0.25), (111.0, 0.0), (100.0, 1.0), (80.0, 1.0)])
def test_peak_window_and_over_gate(top, expected):
    assert score(feed(demo_angles(top))) == expected


def test_lift_once_then_back_is_zero():
    angles = demo_angles()
    r = feed(angles, lift=lambda k, n: 0.016 if k == 60 else 0.0)  # 16 mm for one step mid-flex
    assert r.max_lift == pytest.approx(0.016) and score(r) == 0.0


def test_lift_just_under_gate_still_scores():
    r = feed(demo_angles(), lift=lambda k, n: 0.0149 if 40 <= k < 60 else 0.0)
    assert score(r) == 1.0


def test_fast_fall_from_40_deg_is_a_drop():
    angles = hold(0.0, 1.0) + ramp(0, 90, 25) + hold(90, 1.0) + ramp(90, 40.0, 25) + [40.0 - 4.75 * k for k in range(1, 9)]
    angles += hold(angles[-1], 0.2) + ramp(angles[-1], 0.0, 25) + hold(0.0, 3.0)  # -95 deg/s from 40 deg
    r = feed(angles)
    assert r.drop and score(r) == 0.0


def test_fast_motion_below_15_deg_is_not_a_drop():
    angles = hold(0.0, 1.0) + ramp(0, 90, 25) + hold(90, 1.0) + ramp(90, 10.0, 25)
    angles += [10.0 - 4.75 * k for k in range(1, 3)] + hold(0.5, 3.0)  # -95 deg/s, but starting at 10 deg
    r = feed(angles)
    assert not r.drop and score(r) == 1.0


def test_spring_snap_after_letting_go_at_90_is_a_drop():
    angles = hold(0.0, 1.0) + ramp(0, 90, 25) + hold(90, 1.0) + [90.0 - 15.0 * k for k in range(1, 7)] + hold(0.0, 3.0)
    assert score(feed(angles)) == 0.0


def test_never_back_below_10_is_half():
    angles = hold(0.0, 1.0) + ramp(0, 90, 25) + hold(90, 1.0) + ramp(90, 15.0, 25) + hold(15.0, 3.0)
    assert score(feed(angles)) == 0.5


def test_returned_must_stay_down_to_the_end():
    angles = demo_angles()[:-60] + ramp(0.0, 20.0, 25) + hold(20.0, 2.0)  # back down, then up again to 20 deg
    assert score(feed(angles)) == 0.5


def test_two_repetitions_score_half():
    one = ramp(0.0, 90.0, 25) + hold(90.0, 1.0) + ramp(90.0, 0.0, 25) + hold(0.0, 1.0)
    r = feed(hold(0.0, 1.0) + one + one + hold(0.0, 2.0))  # flex, return, then a second full flex and return
    assert r.second_rep and score(r) == 0.5, T.ladder(r)


def test_second_flex_to_45_counts_as_a_repetition():
    angles = demo_angles()[:-60] + ramp(0.0, 45.0, 25) + ramp(45.0, 0.0, 25) + hold(0.0, 3.0)
    assert score(feed(angles)) == 0.5


def test_small_rebend_after_return_is_not_a_repetition():
    angles = demo_angles()[:-60] + ramp(0.0, 30.0, 25) + ramp(30.0, 0.0, 25) + hold(0.0, 3.0)
    r = feed(angles)
    assert not r.second_rep and score(r) == 1.0, T.ladder(r)


def test_gripper_25_mm_from_forearm_is_three_quarters():
    r = feed(demo_angles(), clear=lambda k, n: 0.025 if k >= n - 40 else 0.0)
    assert score(r) == 0.75 and "let_go" in T.ladder(r)[1]


def test_gripper_exactly_30_mm_clear_counts():
    assert score(feed(demo_angles(), clear=lambda k, n: 0.030 if k >= n - 40 else 0.0)) == 1.0


def test_tcp_moving_6_mm_s_in_last_second_is_three_quarters():
    r = feed(demo_angles(), tcp_step=lambda k, n: 0.0003 if k >= n - 20 else (0.0 if k >= n - 40 else 0.002))
    assert score(r) == 0.75 and "still_1s" in T.ladder(r)[1]


def test_still_for_19_steps_is_not_enough():
    r = feed(demo_angles(), tcp_step=lambda k, n: 0.001 if k == n - 20 else (0.0 if k >= n - 40 else 0.002))
    assert r.still == 19 and score(r) == 0.75


def test_end_angle_7_deg_is_three_quarters():
    angles = demo_angles()[:-60] + hold(7.0, 3.0)
    r = feed(angles)
    assert r.returned and score(r) == 0.75 and "angle" in T.ladder(r)[1]


def test_forearm_held_above_pad_is_three_quarters():
    r = feed(demo_angles(), low=lambda k, n: 0.008 if k >= n - 30 else 0.0)
    assert score(r) == 0.75 and "on_pad" in T.ladder(r)[1]


def test_elbow_left_6_mm_up_at_end_is_three_quarters():
    r = feed(demo_angles(), lift=lambda k, n: 0.006 if k >= n - 30 else 0.0)
    assert score(r) == 0.75 and "elbow_down" in T.ladder(r)[1]


def test_history_resets_only_on_reset():
    r = feed(demo_angles(111.0))
    assert r.over and score(r) == 0.0
    feed(hold(0.0, 2.0), rom=r)  # putting the arm back does not erase the violation
    assert r.over and score(r) == 0.0
    r.reset()
    assert r.n == 0 and not r.over and r.peak == -math.inf and score(r) == 0.0
    assert score(feed(demo_angles(), rom=r)) == 1.0
    feed(ramp(0.0, 90.0, 25) + ramp(90.0, 0.0, 25) + hold(0.0, 3.0), rom=r)  # a second repetition sticks to reset
    assert r.second_rep and score(r) == 0.5
    r.reset()
    assert not r.was_back and not r.second_rep


def test_success_and_distance_follow_the_ladder():
    T.ROM.reset()
    feed(demo_angles(), rom=T.ROM)
    assert T.success({}) and T.score({}) == 1.0 and T.distance({}) == 0.0
    T.ROM.reset()
    feed(demo_angles(79.0), rom=T.ROM)
    assert not T.success({}) and T.distance({}) == pytest.approx(0.75)
    T.ROM.reset()


def test_design_pose_rests_on_the_pad():
    import linkkit as lk

    poses = T.MANIKIN.link_poses()
    s = lk.LinkState(pos=np.zeros(3), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=np.zeros(3), high=np.zeros(3), spec=T.MANIKIN, poses=poses,
                     q={"shoulder": 0.0, "elbow": 0.0}, qd={})
    theta, lift, low = T.measure(s)
    assert theta == 0.0 and abs(lift) < 1e-9 and abs(low) < 1e-9
    assert T.lowest_point(s, "upper_arm") == pytest.approx(T.PAD_TOP)
    bent = T.MANIKIN.link_poses({"elbow": math.radians(90)})
    grasp = bent["forearm"][:3, :3] @ np.asarray(T.GRASP_IN_FORE) + bent["forearm"][:3, 3]
    assert np.allclose(grasp, T.E + [0, 0, T.GRASP_F], atol=1e-9)  # +90 deg stands the forearm straight up


def test_demo_poses_reachable():
    try:
        from yam_arena import ik
    except (ImportError, OSError):  # yam_arena (and its URDF) not installed here
        pytest.skip("needs the yam-isaac-lab-arena package")
    from yam_arena.constants import READY
    from yam_arena.station import BASES

    import linkdemo as ld

    z = np.array([0.0, 0.0, 1.0])
    q = np.array(READY, dtype=float)
    for th in range(0, 121, 5):
        g = math.radians(th / 3)
        p = T.E + T.GRASP_F * (math.cos(math.radians(th)) * -T.U + math.sin(math.radians(th)) * z)
        r = ld.grip_frame(math.sin(g) * T.U - math.cos(g) * z, T.C)
        best = None
        for seed in [q] + [np.r_[q[:5], s] for s in (-1.6, 0.0, 1.6)]:
            out = ik.solve(p, ik.matrix_to_quat_xyzw(r), seed, base_pos=BASES["right"], iters=200)
            if best is None or out[1] < best[1]:
                best = out
        q = best[0]
        assert best[1] < 5e-4 and best[2] < math.radians(0.5), th
    tilt = math.radians(T.PRESS_TILT)
    a = -math.sin(tilt) * T.U - math.cos(tilt) * z
    _, pe, re = ik.solve(T.E + T.PRESS_D * T.U, ik.matrix_to_quat_xyzw(ld.grip_frame(a, T.C)), np.array(READY, float),
                         base_pos=BASES["left"], iters=200)
    assert pe < 5e-4 and re < math.radians(0.5)
