# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for tire-pressure-gauge, pure Python (no simulator). Run from the task folder: pytest -q tests"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05


def sample(**kw):
    m = {"tilt": 0.5, "off": 0.0005, "collar": 0.0025, "reading": 0.035, "pos": (0.3, -0.3, 0.7555), "speed": 0.0,
         "home_xy": 0.0, "home_z": 0.0, "clear": {"left": 0.1, "right": 0.1}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def feed(rec, rows):
    t = 0.0
    for secs, kw in rows:
        for _ in range(round(secs / DT)):
            t += DT
            rec.update(sample(**kw), t=t)
    return rec


PRESS = dict(home_xy=0.2, home_z=0.05, clear={"left": 0.1, "right": 0.0}, tcp_speed={"left": 0.0, "right": 0.02})
HOME = dict(collar=0.0)


def test_full_run_scores_one():
    rec = feed(T.Record(), [(2.0, PRESS), (2.5, HOME), (2.5, HOME)])
    assert T.ladder(rec)[0] == 1.0


def test_nothing_is_zero():
    rec = feed(T.Record(), [(5.0, dict(collar=0.0, reading=0.0))])
    assert T.ladder(rec)[0] == 0.0


def test_angled_press_leaks_and_scores_zero():
    rec = feed(T.Record(), [(4.0, dict(PRESS, tilt=5.0, reading=0.012)), (3.0, dict(HOME, reading=0.012))])
    assert T.ladder(rec)[0] == 0.0


def test_light_press_scores_zero():
    rec = feed(T.Record(), [(4.0, dict(PRESS, collar=0.0006, reading=0.0)), (3.0, dict(HOME, reading=0.0))])
    assert T.ladder(rec)[0] == 0.0


def test_short_seal_scores_zero():
    rec = feed(T.Record(), [(1.0, PRESS), (3.0, dict(HOME, reading=0.0187))])
    assert T.ladder(rec)[0] == 0.0


def test_stick_pulled_by_hand_without_seal_scores_zero():
    rec = feed(T.Record(), [(3.0, dict(HOME, reading=0.035))])
    assert T.ladder(rec)[0] == 0.0


def test_seal_held_but_reading_out_of_band_is_point_four():
    rec = feed(T.Record(), [(2.0, PRESS), (2.0, dict(HOME, reading=0.031))])
    assert T.ladder(rec)[0] == 0.4


def test_gauge_left_on_valve_is_point_seven():
    rec = feed(T.Record(), [(2.0, PRESS), (3.0, PRESS)])
    assert T.ladder(rec)[0] == 0.7


def test_gripper_still_holding_is_point_seven():
    rec = feed(T.Record(), [(2.0, PRESS), (3.0, dict(HOME, clear={"left": 0.1, "right": 0.0}))])
    assert T.ladder(rec)[0] == 0.7


def test_seal_run_resets_when_the_press_is_interrupted():
    rec = feed(T.Record(), [(1.0, PRESS), (0.5, dict(PRESS, collar=0.0)), (1.0, PRESS)])
    assert "seal_hold" not in rec.H.evers


def test_reset_clears_the_whole_run():
    rec = feed(T.Record(), [(2.0, PRESS), (2.5, HOME)])
    rec.reset()
    assert T.ladder(rec)[0] == 0.0


def test_leak_and_seal_predicates():
    assert T.sealed(sample())
    assert not T.sealed(sample(tilt=3.5))
    assert not T.sealed(sample(collar=0.0015))
    assert T.leaking(sample(tilt=5.0))
    assert not T.leaking(sample(collar=0.0005, tilt=5.0))
