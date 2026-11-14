# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for infant-manikin-transfer, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); every test feeds it
synthetic samples shaped like the measure() output.
"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05


def sample(**kw):
    """Finished state: lying on the pad, neck straight, still, both grippers clear and still."""
    m = {
        "neck": 2.0, "pitch": 1.0, "roll": 1.0, "up_deg": 2.0, "rise_b": -0.014, "rise_h": -0.014,
        "bx": T.PAD_XC, "by": T.BODY_Y, "hx": T.PAD_XC, "low_b": 0.001, "low_h": 0.001,
        "speed": 0.0, "spin": 0.0, "clear": {"left": 0.08, "right": 0.06}, "tcp_speed": {"left": 0.0, "right": 0.0},
    }
    m.update(kw)
    return m


def carrying(**kw):
    """Mid-carry: lifted 60 mm, over the pad, neck straight."""
    base = dict(rise_b=0.060, rise_h=0.060, bx=0.14, low_b=0.075, low_h=0.075, clear={"left": 0.0, "right": 0.0})
    base.update(kw)
    return sample(**base)


def run(*samples, hold_last=0.0):
    rec = T.Record()
    t = 0.0
    for m in samples:
        t += DT
        rec.update(m, t)
    n = int(round(hold_last / DT))
    for _ in range(n):
        t += DT
        rec.update(samples[-1], t)
    return rec


def at_start(**kw):
    return sample(rise_b=0.0, rise_h=0.0, bx=T.XC, hx=T.XC, low_b=0.015, low_h=0.015, clear={"left": 0.1, "right": 0.1}, **kw)


def demo(final_hold=1.2, **kw):
    return run(at_start(), carrying(**kw), sample(clear={"left": 0.0, "right": 0.0}), sample(), hold_last=final_hold)


def test_demo_shape_scores_one():
    assert T.ladder(demo())[0] == 1.0


def test_nothing_scores_zero():
    assert T.ladder(run(at_start(), at_start()))[0] == 0.0


def test_neck_gate_20_deg_is_exact():
    assert T.ladder(demo(neck=20.0))[0] == 1.0
    assert T.ladder(demo(neck=20.1))[0] == 0.0


def test_neck_gate_is_whole_run_not_just_the_end():
    rec = run(at_start(), carrying(neck=35.0), carrying(), sample(), hold_last=1.2)
    assert T.ladder(rec)[0] == 0.0


def test_tilt_gate_pitch_and_roll():
    assert T.ladder(demo(pitch=30.0))[0] == 1.0
    assert T.ladder(demo(pitch=30.5))[0] == 0.0
    assert T.ladder(demo(roll=31.0))[0] == 0.0


def test_lift_threshold_35_mm_needs_head_and_body():
    assert T.ladder(run(at_start(), carrying(rise_b=0.0349)))[0] == 0.0
    assert T.ladder(run(at_start(), carrying(rise_h=0.0349)))[0] == 0.0
    assert T.ladder(run(at_start(), carrying(rise_b=0.036, rise_h=0.036, bx=0.3)))[0] == 0.25


def test_carried_needs_the_centre_line_over_the_pad():
    assert T.ladder(run(at_start(), carrying(bx=T.CARRIED_X + 0.001)))[0] == 0.25
    assert T.ladder(run(at_start(), carrying(bx=T.CARRIED_X)))[0] == 0.5


def test_not_on_pad_at_end_scores_half():
    assert T.ladder(run(at_start(), carrying(), carrying(bx=0.30)))[0] == 0.5


def test_ends_on_the_pad_without_lift_scores_zero():
    assert T.ladder(run(at_start(), sample(), hold_last=2.0))[0] == 0.0


def test_end_neck_must_be_within_10_deg():
    rec = run(at_start(), carrying(), sample(neck=10.5), hold_last=1.2)
    assert T.ladder(rec)[0] == 0.5
    rec = run(at_start(), carrying(), sample(neck=10.0), hold_last=1.2)
    assert T.ladder(rec)[0] == 1.0


def test_supine_limit_25_deg():
    rec = run(at_start(), carrying(), sample(up_deg=25.5), hold_last=1.2)
    assert T.ladder(rec)[0] == 0.5


def test_head_must_rest_on_the_pad_too():
    rec = run(at_start(), carrying(), sample(low_h=0.012), hold_last=1.2)
    assert T.ladder(rec)[0] == 0.5


def test_body_set_down_height_window():
    assert T.ladder(demo())[0] == 1.0
    assert T.ladder(run(at_start(), carrying(), sample(low_b=0.0061), hold_last=1.2))[0] == 0.5
    assert T.ladder(run(at_start(), carrying(), sample(low_b=-0.0031), hold_last=1.2))[0] == 0.5


def test_release_gate_needs_30_mm_and_one_second():
    rec = run(at_start(), carrying(), sample(clear={"left": 0.0299, "right": 0.08}), hold_last=1.2)
    assert T.ladder(rec)[0] == 0.75
    rec = run(at_start(), carrying(), sample(clear={"left": 0.03, "right": 0.08}), hold_last=0.9)
    assert T.ladder(rec)[0] == 0.75
    rec = run(at_start(), carrying(), sample(clear={"left": 0.03, "right": 0.08}), hold_last=1.1)
    assert T.ladder(rec)[0] == 1.0


def test_gripper_or_manikin_moving_blocks_the_release():
    rec = run(at_start(), carrying(), sample(tcp_speed={"left": 0.006, "right": 0.0}), hold_last=1.2)
    assert T.ladder(rec)[0] == 0.75
    rec = run(at_start(), carrying(), sample(speed=0.006), hold_last=1.2)
    assert T.ladder(rec)[0] == 0.75
    rec = run(at_start(), carrying(), sample(spin=3.5), hold_last=1.2)
    assert T.ladder(rec)[0] == 0.75


def test_a_late_nudge_restarts_the_release_timer():
    rec = T.Record()
    rec.update(at_start(), 0.05)
    rec.update(carrying(), 0.10)
    t = 0.10
    for i in range(30):
        t += DT
        rec.update(sample(), t)
    t += DT
    rec.update(sample(speed=0.01), t)
    for i in range(10):
        t += DT
        rec.update(sample(), t)
    assert T.ladder(rec)[0] == 0.75


def test_record_is_clean_after_reset():
    rec = demo(neck=40.0)
    assert T.ladder(rec)[0] == 0.0
    rec.reset()
    assert rec.last is None and not rec.H.peaks and not rec.H.evers
    assert T.ladder(rec)[0] == 0.0


def test_distance_and_success_follow_the_ladder():
    T.REC.reset()
    for i, m in enumerate([at_start(), carrying(), sample()] + [sample()] * 25, start=1):
        T.REC.update(m, DT * i)
    assert T.success(None) and T.distance(None) == pytest.approx(0.0)
    T.REC.reset()


def test_pose_rates():
    import numpy as np

    a = np.eye(4)
    b = np.eye(4)
    b[0, 3] = 0.001
    v, w = T.pose_rates(a, b, DT)
    assert v == pytest.approx(0.02) and w == pytest.approx(0.0)
    assert T.pose_rates(a, b, 0.0) == (0.0, 0.0)
