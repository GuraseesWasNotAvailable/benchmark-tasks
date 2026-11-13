# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for pry-bar-shim-insert, pure Python (no simulator run). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it synthetic
samples at 20 Hz. The module imports yam_arena (for the table height), so source the environment file first.
"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05


def sample(**kw):
    """The finished state: packer 313 mm forward, flat, on the hinge half; door resting on it; hands clear."""
    m = {"q": 0.0015, "lever": 3.0, "front": 0.313, "rear": 0.213, "y": 0.02, "yaw": -5.0, "tilt": 0.0, "z": 0.003,
         "dspeed": 0.0, "clear": {"left": 0.07, "right": 0.11}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def run(samples):
    rec = T.Record()
    for i, m in enumerate(samples):
        rec.update(m, t=(i + 1) * DT)
    return rec


def score(samples):
    return T.ladder(run(samples))[0]


def lifted(n=30, **kw):
    """The door held 10.7 mm up with the packer at the demo's final place, a hand on the bar."""
    return [sample(q=0.0107, lever=13.8, dspeed=0.0, clear={"left": 0.0, "right": 0.08}, **kw) for _ in range(n)]


def start(n=20):
    return [sample(q=0.0, lever=1.3, front=0.270, rear=0.170, y=0.01, yaw=0.0, clear={"left": 0.06, "right": 0.08})
            for _ in range(n)]


def demo(end=30, **end_kw):
    return start() + lifted() + [sample(q=0.0107 - 0.0092 * k / 10, dspeed=0.02, clear={"left": 0.0, "right": 0.1})
                                 for k in range(1, 11)] + [sample(**end_kw) for _ in range(end)]


def test_demo_shape_scores_one():
    assert score(demo()) == 1.0


def test_no_steps_scores_zero():
    assert T.ladder(T.Record())[0] == 0.0


def test_nothing_moves_scores_zero():
    assert score(start(80)) == 0.0


def test_left_only_door_lifted_no_packer_scores_zero():
    s = start() + [sample(q=0.0107, front=0.270, rear=0.170, y=0.01, yaw=0.0) for _ in range(40)] + start(40)
    assert score(s) == 0.0


def test_right_only_packer_against_a_shut_door_scores_zero():
    assert score([sample(q=0.0, front=0.286, rear=0.186, y=0.01, yaw=0.0) for _ in range(80)]) == 0.0


def test_release_early_scores_zero():
    s = start() + [sample(q=0.0107, front=0.270, rear=0.170, yaw=0.0) for _ in range(10)] + start(40)
    s += [sample(q=0.0, front=0.286, rear=0.186, yaw=0.0) for _ in range(40)]
    assert score(s) == 0.0


@pytest.mark.parametrize("q, want", [(0.0069, 0.0), (0.0070, 0.5)])
def test_door_lift_threshold_for_a_run_that_ended_without_the_packer(q, want):
    s = [sample(q=q) for _ in range(5)] + [sample(q=0.0, front=0.270, rear=0.170) for _ in range(40)]
    assert score(s) == want


@pytest.mark.parametrize("front, want", [(0.2959, 0.0), (0.2960, 0.25), (0.3059, 0.25), (0.3060, 0.5)])
def test_packer_depth_rungs_for_a_run_that_ended_with_the_packer_out(front, want):
    s = [sample(q=0.0107, front=front) for _ in range(5)] + [sample(q=0.0, front=0.270, rear=0.170) for _ in range(40)]
    assert score(s) == want


def test_packer_too_far_through_is_not_seated():
    assert score(demo(front=0.345)) == 1.0
    assert score(demo(front=0.346)) == 0.5


@pytest.mark.parametrize("field, ok, bad", [("y", 0.0, -0.001), ("y", 0.09, 0.091), ("yaw", 12.0, 12.1),
                                            ("yaw", -12.0, -12.1), ("yaw", 175.0, 167.0), ("tilt", 5.0, 5.1),
                                            ("z", 0.0045, 0.0046), ("z", 0.0015, 0.0014)])
def test_seated_limits(field, ok, bad):
    assert score(demo(**{field: ok})) == 1.0
    assert score(demo(**{field: bad})) == 0.5


@pytest.mark.parametrize("q, want", [(0.00079, 0.5), (0.0008, 1.0), (0.0035, 1.0), (0.00351, 0.5)])
def test_door_must_rest_on_the_packer(q, want):
    assert score(demo(q=q)) == want


def test_door_still_held_up_by_the_bar_at_the_end_is_half():
    assert score(start() + lifted() + lifted(40)) == 0.5


def test_door_back_on_its_washers_at_the_end_is_half():
    assert score(demo(q=0.0)) == 0.5


def test_door_must_be_still_for_1_s():
    s = demo(end=0) + [sample(dspeed=0.01)] + [sample() for _ in range(19)]
    assert score(s) == 0.75
    s = demo(end=0) + [sample(dspeed=0.01)] + [sample() for _ in range(21)]
    assert score(s) == 1.0


def test_release_gate_30_mm_and_still():
    assert score(demo(clear={"left": 0.030, "right": 0.05})) == 1.0
    assert score(demo(clear={"left": 0.029, "right": 0.05})) == 0.75
    assert score(demo(tcp_speed={"left": 0.0, "right": 0.0049})) == 1.0
    assert score(demo(tcp_speed={"left": 0.0, "right": 0.005})) == 0.75


def test_release_must_hold_one_second():
    s = demo(end=0) + [sample(clear={"left": 0.0, "right": 0.05})] + [sample() for _ in range(19)]
    assert score(s) == 0.75
    s = demo(end=0) + [sample(clear={"left": 0.0, "right": 0.05})] + [sample() for _ in range(21)]
    assert score(s) == 1.0


def test_history_keeps_the_half_rung_after_the_packer_is_pulled_out():
    s = start() + lifted() + start(40)
    assert score(s) == 0.5


def test_history_is_not_cleared_by_a_later_clean_finish():
    s = start() + lifted(front=0.27) + demo()
    assert score(s) == 1.0  # a failed first attempt does not gate: the ladder rewards the best whole-run evidence


def test_hand_still_pressing_the_bar_at_the_end_is_075():
    assert score(demo(clear={"left": 0.0, "right": 0.1})) == 0.75


def test_record_clean_after_reset():
    rec = run(demo())
    assert T.ladder(rec)[0] == 1.0
    rec.reset()
    assert T.ladder(rec)[0] == 0.0
    rec.update(sample(), t=0.05)
    assert T.ladder(rec)[0] == 0.0
