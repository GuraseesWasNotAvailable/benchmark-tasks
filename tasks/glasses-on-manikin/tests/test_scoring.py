# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for glasses-on-manikin, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it
synthetic samples shaped like measure()'s output.
"""

import copy
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control


def sample(**kw):
    """A finished state: bridge on the nose, both temples over the ears, still, both grippers clear."""
    m = {
        "bridge": (0.001, 0.0045, 0.0), "frame_z": T.Z0F, "tip_l": (0.009, 0.0845, 0.0), "tip_r": (0.0127, 0.0854, 0.0),
        "q_l": -3.7, "q_r": -11.7, "held": {"left": False, "right": False}, "speed": 0.0, "spin": 0.0,
        "clear": {"left": 0.08, "right": 0.08}, "tool_speed": {"left": 0.0, "right": 0.0},
    }
    m.update(kw)
    return m


def carrying(**kw):
    """Mid-carry: both grippers on the pair, 60 mm up, far from the head."""
    base = dict(bridge=(-0.09, 0.0, 0.05), frame_z=T.Z0S + 0.06, tip_l=(-0.08, 0.0665, 0.04), tip_r=(-0.08, 0.0665, 0.04),
                q_l=0.0, q_r=0.0, held={"left": True, "right": True}, speed=40.0, spin=0.0,
                clear={"left": 0.0, "right": 0.0}, tool_speed={"left": 0.05, "right": 0.05})
    base.update(kw)
    return sample(**base)


def feed(samples, rec=None):
    rec = rec or T.Record(lk.History())
    t = 0.0
    for m in samples:
        t += DT
        rec.h._tick(t)
        rec.update(m, t)
    return rec


def rung(samples):
    return T.ladder(feed(samples).h)[0]


def test_finished_demo_scores_one():
    assert rung([carrying()] * 5 + [sample()] * 40) == 1.0


def test_one_second_of_stillness_is_needed():
    assert rung([carrying()] * 5 + [sample()] * 19) == 0.75  # 0.95 s
    assert rung([carrying()] * 5 + [sample()] * 21) == 1.0


def test_worn_but_still_in_hand_is_075():
    held = sample(held={"left": True, "right": True}, clear={"left": 0.0, "right": 0.0})
    assert rung([carrying()] * 5 + [held] * 40) == 0.75


@pytest.mark.parametrize("dz,ok", [(0.003, True), (0.0031, False), (-0.002, True), (-0.0021, False)])
def test_bridge_height_window(dz, ok):
    m = sample(bridge=(0.0, 0.0, dz))
    assert T.bridge_ok(m) is ok


@pytest.mark.parametrize("y,ok", [(0.006, True), (-0.006, True), (0.0061, False)])
def test_bridge_centred_on_nose(y, ok):
    assert T.bridge_ok(sample(bridge=(0.0, y, 0.001))) is ok


@pytest.mark.parametrize("dx,ok", [(0.008, True), (-0.008, True), (0.0081, False)])
def test_bridge_along_nose(dx, ok):
    assert T.bridge_ok(sample(bridge=(dx, 0.0, 0.001))) is ok


@pytest.mark.parametrize("past,ok", [(0.003, True), (0.0029, False)])
def test_tip_behind_the_ear(past, ok):
    assert T.temple_ok(sample(tip_l=(past, 0.085, 0.0)), "l") is ok


@pytest.mark.parametrize("ay,ok", [(0.0725, True), (0.0925, True), (0.0724, False), (0.0926, False)])
def test_tip_over_the_ear_not_beside_it(ay, ok):
    assert T.temple_ok(sample(tip_r=(0.01, ay, 0.0)), "r") is ok


@pytest.mark.parametrize("dz,ok", [(0.004, True), (0.0041, False), (-0.002, True), (-0.0021, False)])
def test_tip_rests_on_the_ear(dz, ok):
    assert T.temple_ok(sample(tip_l=(0.01, 0.085, dz)), "l") is ok


@pytest.mark.parametrize("q,ok", [(3.0, True), (3.1, False), (-12.0, True)])
def test_temple_open(q, ok):
    assert T.temple_ok(sample(q_r=q), "r") is ok


def test_one_temple_not_over_its_ear_is_not_worn():
    m = sample(tip_l=(-0.05, 0.075, 0.03))
    assert not T.worn(m)
    assert rung([carrying()] * 5 + [m] * 40) == 0.5  # the bridge was on the nose, the left temple is not over its ear


def test_a_temple_folded_and_opened_again_is_scored_on_the_final_state():
    assert rung([carrying()] * 5 + [sample(q_l=45.0)] + [sample()] * 60) == 1.0
    assert rung([carrying()] * 3 + [sample(q_r=75.0)] + [sample()] * 60) == 1.0


def test_a_temple_folded_at_the_end_is_not_worn():
    assert rung([carrying()] * 5 + [sample(q_l=45.0)] * 60) == 0.5


def test_lifted_needs_both_grippers():
    one = carrying(held={"left": True, "right": False})
    assert rung([one] * 10) == 0.0
    assert rung([carrying()] * 10) == 0.25


def test_lifted_needs_40_mm():
    assert rung([carrying(frame_z=T.Z0S + 0.039)] * 10) == 0.0
    assert rung([carrying(frame_z=T.Z0S + 0.04)] * 10) == 0.25


def test_bridge_on_nose_at_some_time_is_05():
    on_nose = carrying(bridge=(0.0, 0.0, 0.001))
    away = carrying(bridge=(0.0, 0.0, 0.03))
    assert rung([carrying()] * 3 + [on_nose] * 3 + [away] * 10) == 0.5


def test_release_gate_30_mm():
    close = sample(clear={"left": 0.029, "right": 0.08})
    assert rung([carrying()] * 5 + [sample()] * 30 + [close] + [sample()] * 10) == 0.75  # 0.5 s since the break
    assert rung([carrying()] * 5 + [sample()] * 30 + [sample(clear={"left": 0.030, "right": 0.08})] * 10) == 1.0


def test_gripper_still_5_mm_s():
    moving = sample(tool_speed={"left": 0.006, "right": 0.0})
    assert rung([carrying()] * 5 + [moving] * 40) == 0.75
    assert rung([carrying()] * 5 + [sample(tool_speed={"left": 0.005, "right": 0.0})] * 40) == 1.0


def test_glasses_still_3_mm_s_3_deg_s():
    assert rung([carrying()] * 5 + [sample(speed=3.1)] * 40) == 0.75
    assert rung([carrying()] * 5 + [sample(spin=3.1)] * 40) == 0.75
    assert rung([carrying()] * 5 + [sample(speed=3.0, spin=3.0)] * 40) == 1.0


def test_worn_must_hold_one_second_at_the_end():
    slipped = sample(tip_l=(-0.02, 0.07, 0.02))
    assert rung([carrying()] * 5 + [sample()] * 30 + [slipped]) == 0.5


def test_record_is_clean_after_a_reset():
    rec = feed([carrying()] * 5 + [sample(q_l=40.0)])
    assert T.ladder(rec.h)[0] == 0.5 and rec.h.peaks
    rec.h.reset()
    assert T.ladder(rec.h)[0] == 0.0 and not rec.h.peaks and not rec.h.evers


def test_nothing_scores_zero():
    assert rung([sample(bridge=(-0.09, 0.0, -0.1), tip_l=(-0.08, 0.0665, -0.1), tip_r=(-0.08, 0.0665, -0.1),
                        q_l=0.0, q_r=0.0, frame_z=T.Z0S)] * 60) == 0.0


def test_rates_from_pose_change():
    import numpy as np

    a = np.eye(4)
    b = np.eye(4)
    b[0, 3] = 0.0002  # 0.2 mm in one 50 ms step = 4 mm/s
    v, w = T.rates(a, b, DT)
    assert v == pytest.approx(4.0) and w == pytest.approx(0.0, abs=1e-6)
    c = copy.deepcopy(a)
    c[:3, :3] = lk.axis_rotation((0, 0, 1), 0.00349)  # 0.2 deg
    assert T.rates(a, c, DT)[1] == pytest.approx(4.0, abs=0.05)
