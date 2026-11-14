# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for impulse-sprinkler-arc, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it
synthetic samples at 20 Hz.
"""

import math
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05


def sample(arc=90.0, head=0.0, clear=0.08, speed=0.0):
    return {"arc": arc, "az_a": -40.0, "az_b": -40.0 + arc, "head_az": head, "q": [0, 0, 0],
            "clear": {"left": clear, "right": clear}, "tcp_speed": {"left": speed, "right": speed}}


def run(samples):
    rec = T.Record()
    for k, m in enumerate(samples):
        rec.update(m, t=(k + 1) * DT)
    return rec


def hold(n, **kw):
    return [sample(**kw) for _ in range(n)]


def test_finished_state_scores_one():
    s, why = T.ladder(run(hold(40)))
    assert s == 1.0, why


def test_arc_tolerance_edges():
    assert T.ladder(run(hold(40, arc=95.0)))[0] == 1.0
    assert T.ladder(run(hold(40, arc=85.0)))[0] == 1.0
    assert T.ladder(run(hold(40, arc=95.1)))[0] == 0.5
    assert T.ladder(run(hold(40, arc=84.9)))[0] == 0.5


def test_rung_half_ends_at_20_deg():
    assert T.ladder(run(hold(40, arc=110.0)))[0] == 0.5
    assert T.ladder(run(hold(40, arc=70.0)))[0] == 0.5
    assert T.ladder(run([sample(arc=200.0)] + hold(40, arc=110.1)))[0] == 0.25


def test_rung_quarter_needs_the_arc_to_have_closed_40_deg():
    path = [sample(arc=200.0 - k) for k in range(0, 61)] + hold(20, arc=140.0)
    assert T.ladder(run(path))[0] == 0.25
    assert T.ladder(run(hold(40, arc=161.0)))[0] == 0.0


def test_start_state_and_nothing_score_zero():
    s, _ = T.ladder(run(hold(100, arc=200.0)))
    assert s == 0.0


def test_pinch_the_wrong_way_scores_zero():
    assert T.ladder(run([sample(arc=200.0 + k) for k in range(30)] + hold(20, arc=230.0)))[0] == 0.0


def test_head_carried_round_with_the_collar_keeps_the_arc():
    path = [sample(arc=200.0, head=k * 1.0) for k in range(60)]  # 20 deg/s, head and both collars together
    s, _ = T.ladder(run(path))
    assert s == 0.0


def test_a_fast_head_spin_that_ends_right_still_scores():
    fast = math.degrees(5.0 * DT)
    path = hold(10) + [sample(head=fast)] + [sample(head=fast) for _ in range(60)]
    assert T.ladder(run(path))[0] == 1.0


def test_record_is_clean_after_reset():
    rec = run([sample(arc=200.0), sample(arc=200.0, head=170.0)])
    assert rec.last is not None
    rec.reset()
    assert rec.last is None and not rec.H.evers and rec.n == 0
    for k in range(40):
        rec.update(sample(), t=(k + 1) * DT)
    assert T.ladder(rec)[0] == 1.0


def test_arc_must_be_still_for_one_second():
    moving = [sample(arc=90.0 + 0.1 * (k % 2) + 0.05 * k) for k in range(40)]  # drifting 1 deg/s
    s, why = T.ladder(run(moving))
    assert s == 0.75 and "arc still" in why
    assert T.ladder(run(hold(19)))[0] == 0.75  # 0.95 s is not enough
    assert T.ladder(run(hold(22)))[0] == 1.0


def test_release_gate_30_mm_and_5_mm_per_s():
    assert T.ladder(run(hold(40, clear=0.029)))[0] == 0.75
    assert T.ladder(run(hold(40, clear=0.030)))[0] == 1.0
    assert T.ladder(run(hold(40, speed=0.006)))[0] == 0.75
    assert T.ladder(run(hold(40, speed=0.004)))[0] == 1.0


def test_one_pad_step_near_the_end_restarts_the_release_timer():
    path = hold(40) + [sample(clear=0.02)] + hold(10)
    assert T.ladder(run(path))[0] == 0.75
    path = hold(40) + [sample(clear=0.02)] + hold(25)
    assert T.ladder(run(path))[0] == 1.0


def test_arc_wraps_at_360():
    class P:  # a point at azimuth deg about the axis
        def __init__(self, deg):
            self.p = (T.CX + math.cos(math.radians(deg)), T.CY + math.sin(math.radians(deg)))

        def __getitem__(self, i):
            return self.p[i]

    assert T.arc_deg(P(-150.0), P(50.0)) == pytest.approx(200.0)
    assert T.arc_deg(P(170.0), P(-100.0)) == pytest.approx(90.0)
    assert T.arc_deg(P(50.0), P(-150.0)) == pytest.approx(160.0)


def test_start_layout_is_200_deg_and_target_is_reachable_one_way():
    assert (T.THETA_B - T.THETA_A) % 360.0 == pytest.approx(T.ARC0)
    assert T.ARC0 - T.TARGET == pytest.approx(110.0)
    assert T.COLLAR_FRICTION / T.HEAD_FRICTION == pytest.approx(15.0)  # the head turns more easily than a collar slips
