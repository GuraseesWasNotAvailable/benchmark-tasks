# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for syringe-plunger-to-mark, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it
synthetic samples shaped like measure()'s output.
"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402,F401
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control
Y0 = T.FLANGE_Y


def sample(**kw):
    """The finished state: 30 mm drawn (5 mL), resting on the tray, still, both hands 60 mm clear and still."""
    m = {
        "draw_mm": 30.0,
        "pts": {"flange": (T.TRAY_X, Y0, 0.0), "nozzle": (T.TRAY_X, Y0 + 0.111, 0.0), "thumb": (T.TRAY_X, Y0 - 0.045, 0.0)},
        "z_axis": T.AXIS_Z - 0.0002, "tilt": -2.3, "b_speed": 0.0, "b_spin": 0.0,
        "clear": {"left": 0.07, "right": 0.06}, "tcp_speed": {"left": 0.0, "right": 0.0},
    }
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


def demo_trace():
    """Rest, both hands on, the plunger drawn 0 to 30 mm, hands let go, 3 s still."""
    held = {"left": 0.0, "right": 0.0}
    out = secs(sample(draw_mm=0.0), 2.0)
    out += secs(sample(draw_mm=0.0, clear=held), 3.0)
    for k in range(30):
        out.append(sample(draw_mm=float(k + 1), clear=held, tcp_speed={"left": 0.0, "right": 0.006}))
    out += secs(sample(clear=held), 1.0)
    out += secs(sample(clear={"left": 0.0, "right": 0.05}), 1.0)
    out += secs(sample(), 3.0)
    return out


def score(samples, **kw):
    rec = feed(samples, **kw)
    return T.ladder(rec)[0]


def test_demo_trace_scores_one():
    assert score(demo_trace()) == 1.0


def test_constants_match_the_setup():
    assert T.MARK_MM == 30.0 and T.MM_PER_ML == 6.0 and T.SEAL_N == 5.0 and T.TOL_ML * T.MM_PER_ML == pytest.approx(1.5)
    assert T.THUMB_R * 2 == 24.0 and T.BARREL_R * 2 == 16.0 and T.BARREL_L == 100.0


@pytest.mark.parametrize("mm,expect", [(30.0, 1.0), (31.5, 1.0), (28.5, 1.0), (31.6, 0.25), (28.4, 0.0), (45.0, 0.25),
                                       (0.0, 0.0), (15.0, 0.0)])
def test_mark_window(mm, expect):
    # the plunger passes through the whole window on its way (a drawn plunger always crosses 28.5 mm) unless it stops short
    path = [sample(draw_mm=min(mm, d), clear={"left": 0.0, "right": 0.0}) for d in np.arange(0.0, 46.0, 1.5) if d <= max(mm, 0.0) or d == 0.0]
    end = secs(sample(draw_mm=mm), 4.0)
    assert score(path + end) == expect


def test_never_drawn_scores_zero_even_when_hands_are_clear():
    assert score(secs(sample(draw_mm=0.0), 6.0)) == 0.0  # "nothing", and left_only and right_only end this way


def test_pulling_the_whole_syringe_is_not_a_draw():
    # right_only: the syringe slides 25 mm along the tray with the plunger still fully in
    slid = sample(draw_mm=0.0, pts={"flange": (T.TRAY_X, Y0 - 0.025, 0.0), "nozzle": (T.TRAY_X, Y0 + 0.086, 0.0),
                                    "thumb": (T.TRAY_X, Y0 - 0.07, 0.0)})
    assert score(secs(slid, 6.0)) == 0.0


def test_gate_off_tray_is_final():
    off = sample(pts={"flange": (T.TRAY_X, T.IN_Y[0] - 0.0101, 0.0), "nozzle": (T.TRAY_X, 0.0, 0.0), "thumb": (T.TRAY_X, 0.0, 0.0)})
    assert score(demo_trace()[:200] + secs(off, 0.1) + demo_trace()[200:]) == 0.0


def test_just_inside_the_margin_is_no_gate():
    edge = sample(pts={"flange": (T.TRAY_X, T.IN_Y[0] - 0.0099, 0.0), "nozzle": (T.TRAY_X, 0.0, 0.0), "thumb": (T.TRAY_X, 0.0, 0.0)})
    assert score(demo_trace()[:200] + secs(edge, 0.1) + demo_trace()[200:]) == 1.0


def test_ending_outside_the_floor_area_is_not_resting():
    out = sample(pts={"flange": (T.TRAY_X, T.IN_Y[0] - 0.005, 0.0), "nozzle": (T.TRAY_X, 0.0, 0.0), "thumb": (T.TRAY_X, 0.0, 0.0)})
    assert score(demo_trace()[:-60] + secs(out, 3.0)) == 0.5


@pytest.mark.parametrize("kw,expect", [
    ({"z_axis": T.AXIS_Z + 0.0039}, 1.0), ({"z_axis": T.AXIS_Z + 0.0041}, 0.5),
    ({"tilt": 7.9}, 1.0), ({"tilt": 8.1}, 0.5), ({"tilt": -8.1}, 0.5),
    ({"b_speed": 0.004}, 0.5), ({"b_spin": 0.2}, 0.5),
])
def test_resting_limits(kw, expect):
    assert score(demo_trace()[:-60] + secs(sample(**kw), 3.0)) == expect


def _drawn_moving():
    """Drawn to the line with the barrel still moving (the hand is on), so the rest timer has not started."""
    return (secs(sample(draw_mm=0.0), 2.0)
            + [sample(draw_mm=float(d), b_speed=0.01, clear={"left": 0.0, "right": 0.0}) for d in range(1, 31)])


def test_rest_needs_one_second():
    assert score(_drawn_moving() + secs(sample(), 0.9)) == 0.5
    assert score(_drawn_moving() + secs(sample(), 1.05)) >= 0.75


def test_overshoot_then_ending_below_the_window_scores_a_quarter():
    path = [sample(draw_mm=d, clear={"left": 0.0, "right": 0.0}) for d in (10, 30, 40)]
    assert score(path + secs(sample(draw_mm=28.4), 4.0)) == 0.25


@pytest.mark.parametrize("kw,expect", [
    ({"clear": {"left": 0.0301, "right": 0.06}}, 1.0), ({"clear": {"left": 0.0299, "right": 0.06}}, 0.75),
    ({"tcp_speed": {"left": 0.0, "right": 0.0099}}, 1.0), ({"tcp_speed": {"left": 0.0, "right": 0.0101}}, 0.75),
])
def test_release_gate(kw, expect):
    assert score(demo_trace()[:-60] + secs(sample(**kw), 3.0)) == expect


def test_release_timer_restarts_on_a_touch():
    base = demo_trace()[:-60]
    tail = secs(sample(), 2.0) + secs(sample(clear={"left": 0.029, "right": 0.06}), 0.05) + secs(sample(), 0.9)
    assert score(base + tail) == 0.75
    assert score(base + tail + secs(sample(), 0.2)) == 1.0


def test_drawn_then_pushed_back_to_the_line_still_scores():
    # overshoot to 8 mL and push the plunger back in to the line is allowed: only the final state is judged
    path = [sample(draw_mm=d, clear={"left": 0.0, "right": 0.0}) for d in (10, 30, 48, 40, 30)]
    assert score(path + secs(sample(), 4.0)) == 1.0


def test_reset_clears_the_record():
    rec = feed(demo_trace())
    assert T.ladder(rec)[0] == 1.0
    rec.reset()
    assert T.ladder(rec) == (0.0, "no steps yet") and rec.H.evers == {}
    feed(secs(sample(draw_mm=0.0), 1.0), rec)
    assert T.ladder(rec)[0] == 0.0


def test_off_tray_gate_clears_only_on_reset():
    rec = feed(secs(sample(pts={"flange": (0.5, 0.5, 0.0), "nozzle": (0.5, 0.5, 0.0), "thumb": (0.5, 0.5, 0.0)}), 0.2))
    assert "off_tray" in rec.H.evers
    feed(demo_trace(), rec, t0=1.0)
    assert T.ladder(rec)[0] == 0.0
    rec.reset()
    feed(demo_trace(), rec)
    assert T.ladder(rec)[0] == 1.0


def test_pose_rates_from_pose_change():
    A = np.eye(4)
    B = np.eye(4)
    B[0, 3] = 0.0002  # 0.2 mm in one 50 ms step is 4 mm/s
    v, w = T.pose_rates(A, B, DT)
    assert v == pytest.approx(0.004) and w == pytest.approx(0.0, abs=1e-9)
    c, s = math.cos(math.radians(1.0)), math.sin(math.radians(1.0))
    B = np.eye(4)
    B[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    assert T.pose_rates(A, B, DT)[1] == pytest.approx(math.radians(1.0) / DT)
    assert T.pose_rates(A, B, 0.0) == (0.0, 0.0)


def test_success_and_distance_follow_the_ladder():
    T.REC.reset()
    for k, m in enumerate(demo_trace()):
        T.REC.update(m, (k + 1) * DT)
    assert T.success(None) and T.score(None) == 1.0 and T.distance(None) == 0.0
    T.REC.reset()
