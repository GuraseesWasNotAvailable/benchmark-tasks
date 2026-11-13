# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for drawer-slide-disconnect, pure Python (no simulator). Run from the task folder: pytest -q tests"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05


def sample(**kw):
    m = {"ext": 0.199, "lL": 0.0, "lR": 0.0, "d_speed": 0.0, "clear": {"left": 0.08, "right": 0.08},
         "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def feed(samples, rec=None, t0=0.0):
    rec = rec or T.Record()
    t = t0
    for m in samples:
        t += DT
        rec.update(m, t)
    return rec


def secs(m, s):
    return [m] * int(round(s / DT))


def pressed(**kw):
    return sample(lL=0.006, lR=0.006, clear={"left": 0.0, "right": 0.0}, **kw)


def full_run():
    out = secs(sample(), 1.0) + secs(pressed(), 1.0)
    out += [pressed(ext=0.199 + 0.1 * k / 40, d_speed=0.1) for k in range(40)]
    out += secs(pressed(ext=0.294), 0.5)
    out += secs(sample(ext=0.294, clear={"left": 0.005, "right": 0.005}), 0.5)
    out += secs(sample(ext=0.294), 2.0)
    return out


def test_nothing_scores_zero():
    assert T.ladder(feed(secs(sample(), 5.0)))[0] == 0.0


def test_one_lever_alone_scores_zero():
    for lever in ("lL", "lR"):
        m = sample(**{lever: 0.006})
        assert T.ladder(feed(secs(m, 5.0)))[0] == 0.0


def test_lever_below_threshold_scores_zero():
    assert T.ladder(feed(secs(sample(lL=0.004, lR=0.004), 3.0)))[0] == 0.0


def test_levers_alternately_never_together_scores_zero():
    rec = feed(secs(sample(lL=0.006), 2.0) + secs(sample(lR=0.006), 2.0))
    assert T.ladder(rec)[0] == 0.0


def test_both_pressed_but_stop_not_passed_is_quarter():
    assert T.ladder(feed(secs(pressed(), 2.0) + secs(sample(), 2.0)))[0] == 0.25


def test_full_run_is_one():
    score, why = T.ladder(feed(full_run()))
    assert score == 1.0, why


def test_passed_stop_but_pushed_back_is_half():
    rec = feed(full_run() + secs(sample(ext=0.21), 3.0))
    assert T.ladder(rec)[0] == 0.5


def test_out_but_moving_is_half():
    rec = feed(secs(pressed(), 1.0) + secs(pressed(ext=0.29, d_speed=0.1), 3.0))
    assert T.ladder(rec)[0] == 0.5


def test_gripper_still_on_the_drawer_caps_at_three_quarters():
    rec = feed(secs(pressed(), 1.0) + secs(pressed(ext=0.29), 3.0))
    assert T.ladder(rec)[0] == 0.75


def test_release_needs_one_second_clear_and_still():
    rec = feed(secs(pressed(), 1.0) + secs(pressed(ext=0.29), 1.5) +
               secs(sample(ext=0.29, tcp_speed={"left": 0.0, "right": 0.05}), 3.0))
    assert T.ladder(rec)[0] == 0.75


def test_short_of_clearance_is_half():
    rec = feed(secs(pressed(), 1.0) + secs(sample(ext=0.25), 3.0))
    assert T.ladder(rec)[0] == 0.5


def test_history_survives_letting_go():
    rec = feed(secs(pressed(), 1.0) + secs(sample(), 3.0))
    assert "both_pressed" in rec.H.evers and T.ladder(rec)[0] == 0.25


def test_reset_clears_history():
    rec = feed(full_run())
    rec.reset()
    assert rec.H.evers == {} and T.ladder(rec)[0] == 0.0
