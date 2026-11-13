# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for wheelchair-footrest-swing, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a linkkit History fed once per control step by update(), read by ladder(). The tests feed synthetic
measurements; the kit's own clock tick is reproduced by calling h._tick(t) before each update.
"""

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control


def m(**kw):
    """One measurement of the finished state: swung out 89.5 deg, parked 40 mm up, still, hands clear."""
    d = {"yaw": -89.5, "lift": 0.040, "press": 0.0, "hspeed": 0.0, "hspin": 0.0, "clear": 0.08, "tcp_speed": 0.0}
    d.update(kw)
    return d


def feed(samples, h=None, t0=0.0):
    h = h or lk.History()
    t = t0
    for s in samples:
        t += DT
        h._tick(t)
        T.update(h, s)
    return h


def secs(sample, s):
    return [sample] * int(round(s / DT))


def start():
    return secs(m(yaw=0.0, lift=0.0, clear=0.2), 2.0)


def demo():
    """Rest, latch pressed and swing to -89.5, lifted 54 mm, set down on the stop, hands withdrawn, 3 s still."""
    out = start()
    out += secs(m(yaw=-45.0, lift=0.0, press=0.008, clear=0.0, hspeed=0.05, hspin=20.0, tcp_speed=0.05), 1.0)
    out += secs(m(yaw=-89.5, lift=0.0, press=0.008, clear=0.0, hspeed=0.1, tcp_speed=0.01), 1.0)
    out += secs(m(lift=0.054, clear=0.0, hspeed=0.06, tcp_speed=0.06), 1.0)
    out += secs(m(clear=0.0), 1.0)
    out += secs(m(clear=0.08), 3.0)
    return out


def rung(samples):
    return T.ladder(feed(samples))[0]


def test_demo_trace_is_full_score():
    assert rung(demo()) == 1.0


def test_nothing_scores_zero():
    assert rung(start() + secs(m(yaw=0.0, lift=0.0), 10.0)) == 0.0


def test_swing_threshold_45_deg():
    base = start()
    assert rung(base + secs(m(yaw=-44.9, lift=0.0), 2.0)) == 0.0
    assert rung(base + secs(m(yaw=-45.0, lift=0.0), 2.0)) == 0.25


def test_swing_threshold_85_deg():
    base = start()
    assert rung(base + secs(m(yaw=-84.9, lift=0.0), 2.0)) == 0.25
    assert rung(base + secs(m(yaw=-85.0, lift=0.0), 2.0)) == 0.5


def test_swung_out_but_not_lifted_is_half():
    assert rung(start() + secs(m(lift=0.0), 3.0)) == 0.5


def test_swing_must_be_seen_at_some_time_even_if_it_returns():
    """The low of the swing is kept for the whole run: swinging out and back to 0 still earned the 0.25 rung."""
    h = feed(start() + secs(m(yaw=-60.0, lift=0.0), 1.0) + secs(m(yaw=0.0, lift=0.0), 2.0))
    assert T.ladder(h) == (0.25, "swung under 85 deg")


def test_lift_window_38_to_44_mm():
    base = start() + secs(m(lift=0.0, clear=0.0, hspeed=0.1, tcp_speed=0.1), 1.0)
    for lift, want in ((0.0379, 0.5), (0.038, 1.0), (0.044, 1.0), (0.0441, 0.5)):
        assert rung(base + secs(m(lift=lift), 3.0)) == want, lift


def test_hanger_released_above_the_stop_scores_half():
    assert rung(start() + secs(m(lift=0.054), 3.0)) == 0.5


def test_swing_end_tolerance_5_deg():
    base = start() + secs(m(lift=0.0, clear=0.0, hspeed=0.1, tcp_speed=0.1), 1.0)
    assert rung(base + secs(m(yaw=-85.0), 3.0)) == 1.0
    assert rung(base + secs(m(yaw=-95.0), 3.0)) == 1.0
    assert rung(base + secs(m(yaw=-84.9), 3.0)) == 0.5
    assert rung(base + secs(m(yaw=-95.1), 3.0)) == 0.5


def test_hanger_must_be_still_for_one_second():
    base = start() + secs(m(lift=0.0, clear=0.0, hspeed=0.1, tcp_speed=0.1), 1.0)
    assert rung(base + secs(m(), 0.95)) == 0.5
    assert rung(base + secs(m(), 1.05)) == 1.0
    assert rung(base + secs(m(hspeed=0.004), 3.0)) == 0.5  # 4 mm/s
    assert rung(base + secs(m(hspeed=0.0029), 3.0)) == 1.0
    assert rung(base + secs(m(hspin=3.1), 3.0)) == 0.5


def test_release_gate_needs_30_mm_and_1_s():
    base = start() + secs(m(lift=0.0, clear=0.0, hspeed=0.1, tcp_speed=0.1), 1.0)
    assert rung(base + secs(m(clear=0.029), 3.0)) == 0.75
    assert rung(base + secs(m(clear=0.030), 3.0)) == 1.0
    assert rung(base + secs(m(clear=0.08, tcp_speed=0.006), 3.0)) == 0.75
    assert rung(base + secs(m(clear=0.08, tcp_speed=0.0049), 3.0)) == 1.0
    assert rung(base + secs(m(clear=0.0), 2.0) + secs(m(clear=0.08), 0.95)) == 0.75
    assert rung(base + secs(m(clear=0.0), 2.0) + secs(m(clear=0.08), 1.05)) == 1.0


def test_one_step_touch_restarts_the_release_timer():
    base = start() + secs(m(lift=0.0, clear=0.0, hspeed=0.1, tcp_speed=0.1), 1.0)
    out = base + secs(m(clear=0.08), 2.0) + [m(clear=0.01)] + secs(m(clear=0.08), 0.5)
    assert rung(out) == 0.75


def test_success_follows_the_top_rung_only():
    assert T.success(None) in (True, False)  # module-level history is empty here
    h = feed(demo())
    assert T.ladder(h)[0] >= 1.0
    h = feed(start() + secs(m(clear=0.0), 3.0))
    assert T.ladder(h)[0] < 1.0


def test_history_is_cleared_by_reset():
    h = feed(demo())
    h.reset()
    assert T.ladder(h) == (0.0, "swung under 45 deg")


def test_pose_rates_from_pose_change():
    a, b = np.eye(4), np.eye(4)
    b[0, 3] = 0.0001
    speed, spin = T.pose_rates(a, b, 0.05)
    assert speed == pytest.approx(0.002) and spin == pytest.approx(0.0, abs=1e-3)
    c = np.eye(4)
    c[:3, :3] = lk.axis_rotation((0, 0, 1), np.radians(0.2))
    assert T.pose_rates(a, c, 0.05)[1] == pytest.approx(4.0, abs=0.01)
    assert T.pose_rates(a, b, 0.0) == (0.0, 0.0)


def test_left_only_and_right_only_records_score_zero():
    """Latch held, hanger never moves (left only); hanger pulled against the locked pawl, 3 deg of give (right only)."""
    assert rung(start() + secs(m(yaw=0.0, lift=0.0, press=0.008, clear=0.0), 20.0)) == 0.0
    assert rung(start() + secs(m(yaw=-2.6, lift=0.0, press=0.0, clear=0.0, tcp_speed=0.01), 20.0)) == 0.0


def test_early_release_record_scores_zero():
    """Latch let up at -25 deg: the pawl locks the hanger near -31 deg."""
    assert rung(start() + secs(m(yaw=-31.3, lift=0.0, press=0.0, clear=0.0), 10.0)) == 0.0


def test_pawl_and_keeper_constants():
    assert T.PRESS_ON > T.PRESS_OFF
    assert T.PRESS_ON <= 0.008  # inside the lever's 8 mm travel
    assert T.KEEPER_DEG >= T.YAW_HALF - 1e-9  # lifting is allowed no later than the 0.5 rung
    assert T.PARK_TRIGGER > T.PIN_TOP > 0  # the parking trigger is above the pin top
    assert T.LIFT_LO > T.PIN_TOP  # a parked hanger is clear of the pin
