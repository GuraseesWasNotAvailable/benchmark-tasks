# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for sharps-scoop-recap, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it
synthetic samples shaped like measure() output.
"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402
import task as T  # noqa: E402

DT = 0.05
AWAY = dict(depth=-64.0, tip_lat=110.0, gap=-112.0, sh_lat=110.0)  # syringe at rest in its holder


def sample(**kw):
    """The finished state: cap seated, syringe still, both grippers clear and still."""
    m = {
        "depth": 48.0, "tip_lat": 0.3, "gap": -0.2, "sh_lat": 0.3, "ang": 0.5,
        "held": {"left": False, "right": False}, "zone": {"left": False, "right": False},
        "touch": {"left": False, "right": False}, "clear": {"left": 0.30, "right": 0.06},
        "tcp_speed": {"left": 0.0, "right": 0.0}, "sy_speed": 0.0, "sy_spin": 0.0,
    }
    m.update(kw)
    return m


def inserting(**kw):
    base = dict(depth=20.0, tip_lat=1.0, gap=-28.0, sh_lat=1.0, ang=5.0, held={"left": False, "right": True},
                zone={"left": False, "right": True}, clear={"left": 0.30, "right": 0.0},
                tcp_speed={"left": 0.0, "right": 0.01})
    base.update(kw)
    return sample(**base)


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
    out = secs(sample(clear={"left": 0.38, "right": 0.2}, **AWAY), 2.0)
    out += secs(inserting(depth=3.0, gap=-45.0), 1.0)
    out += secs(inserting(depth=20.0), 1.0)
    out += secs(inserting(depth=47.9, gap=0.0, tcp_speed={"left": 0.0, "right": 0.0}), 1.0)
    out += secs(sample(), 3.5)
    return out


def test_demo_trace_scores_one():
    assert T.ladder(feed(demo_trace()))[0] == 1.0


def test_nothing_scores_zero():
    assert T.ladder(feed(secs(sample(**AWAY), 5.0)))[0] == 0.0


def test_empty_record_is_zero():
    assert T.ladder(T.Record())[0] == 0.0


def test_off_hand_in_zone_gates_everything():
    out = secs(inserting(), 0.5)
    out += secs(inserting(zone={"left": True, "right": True}), 0.05)
    s, why = T.ladder(feed(out + demo_trace()))
    assert s == 0.0 and "off hand" in why


def test_working_hand_over_zone_is_fine():
    assert "off_hand" not in feed(demo_trace()).H.evers


def test_off_hand_before_anyone_holds_is_no_gate_but_a_pad_on_the_cap_is():
    rec = feed(secs(sample(zone={"left": True, "right": False}, **AWAY), 1.0) + demo_trace())
    assert T.ladder(rec)[0] == 1.0
    rec = feed(secs(sample(touch={"left": True, "right": False}, **AWAY), 0.05) + demo_trace())
    s, why = T.ladder(rec)
    assert s == 0.0 and "touched the cap" in why


def test_holder_is_the_first_arm_to_hold_and_the_other_is_the_off_hand():
    rec = feed(secs(inserting(held={"left": True, "right": False}, zone={"left": True, "right": True}), 0.5))
    assert rec.holder == "left" and "off_hand" in rec.H.evers


def test_a_touch_by_the_working_hand_gates_too():
    rec = feed(secs(inserting(touch={"left": False, "right": True}), 0.05) + demo_trace())
    assert T.ladder(rec)[0] == 0.0


def test_gate_survives_everything_after():
    rec = feed(secs(inserting(zone={"left": True, "right": True}), 0.5) + secs(inserting(), 0.5))
    rec.update(sample(), 100.0)
    assert T.ladder(rec)[0] == 0.0


def test_enter_rung_needs_a_held_syringe():
    assert T.ladder(feed(secs(sample(depth=10.0, gap=-38.0), 2.0)))[0] == 0.0
    assert T.ladder(feed(secs(inserting(depth=5.0, gap=-43.0), 1.0)))[0] == 0.25
    assert T.ladder(feed(secs(inserting(depth=4.9, gap=-43.1), 1.0)))[0] == 0.0


def test_enter_rung_needs_the_tip_inside_the_bore():
    assert T.ladder(feed(secs(inserting(depth=20.0, tip_lat=3.6), 1.0)))[0] == 0.0
    assert T.ladder(feed(secs(inserting(depth=20.0, tip_lat=3.5), 1.0)))[0] == 0.25


def test_deep_rung():
    away = secs(sample(**AWAY), 1.0)
    assert T.ladder(feed(secs(inserting(depth=30.0, ang=15.0), 1.0) + away))[0] == 0.5
    assert T.ladder(feed(secs(inserting(depth=30.0, ang=15.1), 1.0) + away))[0] == 0.25
    assert T.ladder(feed(secs(inserting(depth=29.9, ang=5.0), 1.0) + away))[0] == 0.25


@pytest.mark.parametrize("gap,expected", [(-3.0, True), (-3.1, False), (3.0, True), (3.1, False)])
def test_seat_gap_limit(gap, expected):
    assert T.seated(sample(gap=gap)) is expected


@pytest.mark.parametrize("ang,expected", [(8.0, True), (8.1, False)])
def test_seat_angle_limit(ang, expected):
    assert T.seated(sample(ang=ang)) is expected


@pytest.mark.parametrize("lat,expected", [(3.0, True), (3.1, False)])
def test_seat_lateral_limit(lat, expected):
    assert T.seated(sample(sh_lat=lat)) is expected


def test_seated_but_tip_outside_the_bore_is_not_seated():
    assert not T.seated(sample(tip_lat=4.0))


def test_seated_while_held_scores_075():
    s, why = T.ladder(feed(secs(inserting(depth=48.0, gap=0.0), 3.0)))
    assert s == 0.75 and "30 mm clear" in why


def test_release_needs_a_full_second_30mm_clear():
    base = secs(inserting(depth=48.0, gap=0.0), 1.0)
    assert T.ladder(feed(base + secs(sample(clear={"left": 0.3, "right": 0.03}), 0.95)))[0] == 0.75
    assert T.ladder(feed(base + secs(sample(clear={"left": 0.3, "right": 0.03}), 3.0)))[0] == 1.0
    assert T.ladder(feed(base + secs(sample(clear={"left": 0.3, "right": 0.029}), 3.0)))[0] == 0.75


def test_a_moving_hand_blocks_release():
    base = secs(inserting(depth=48.0, gap=0.0), 1.0)
    assert T.ladder(feed(base + secs(sample(tcp_speed={"left": 0.0, "right": 0.006}), 3.0)))[0] == 0.75


def test_syringe_must_be_still_two_seconds():
    base = secs(inserting(depth=48.0, gap=0.0, sy_speed=0.05), 1.0)
    assert T.ladder(feed(base + secs(sample(), 1.9)))[0] == 0.75
    assert T.ladder(feed(base + secs(sample(), 2.1)))[0] == 1.0
    assert T.ladder(feed(base + secs(sample(sy_speed=0.004), 4.0)))[0] == 0.75
    assert T.ladder(feed(base + secs(sample(sy_spin=math.radians(3.5)), 4.0)))[0] == 0.75


def test_cap_pulled_off_again_loses_the_end_rungs():
    rec = feed(demo_trace() + secs(sample(depth=20.0, gap=-28.0), 1.0))
    assert T.ladder(rec)[0] == 0.5


def test_record_is_clean_after_a_reset():
    rec = feed(secs(inserting(zone={"left": True, "right": True}), 0.5))
    assert T.ladder(rec)[0] == 0.0
    rec.reset()
    assert rec.holder is None and not rec.H.evers
    assert T.ladder(feed(demo_trace(), rec))[0] == 1.0


def test_zone_membership():
    assert T.in_zone((0.15, -0.1, 0.9))
    assert not T.in_zone((0.04, -0.1, 0.9))
    assert not T.in_zone((0.15, 0.04, 0.9))
    assert T.in_zone((0.15, 0.03, 0.9))


def test_pose_rates():
    A, B = np.eye(4), np.eye(4)
    B[0, 3] = 0.0002
    v, w = T.pose_rates(A, B, DT)
    assert v == pytest.approx(0.004) and w == pytest.approx(0.0)
    B = np.eye(4)
    B[:3, :3] = lk.axis_rotation((0, 0, 1), math.radians(0.2))
    v, w = T.pose_rates(A, B, DT)
    assert v == pytest.approx(0.0) and math.degrees(w) == pytest.approx(4.0, rel=1e-3)


def test_pose_for_puts_the_tip_where_asked_and_points_the_needle():
    C = np.eye(4)
    C[:3, 3] = (0.15, -0.045, 0.77)
    S = T.pose_for(C, 0.01, 0.002, 0.0, 20.0)
    tip = S[:3, :3] @ np.array([T.TIP, 0, 0]) + S[:3, 3]
    assert tip == pytest.approx(C[:3, 3] + np.array([0.01, 0.0, 0.002]), abs=1e-9)
    assert math.degrees(math.asin(-S[2, 0])) == pytest.approx(20.0, abs=1e-6)
    assert np.linalg.det(S[:3, :3]) == pytest.approx(1.0)


def test_measure_on_real_poses():
    class St:
        pass

    cap, syr = St(), St()
    cap.poses = {"cap": np.eye(4)}
    S = np.eye(4)
    S[0, 3] = -0.0001
    syr.poses = {"syringe": S}
    syr.lin_vels = {"syringe": np.zeros(3)}
    syr.ang_vels = {"syringe": np.zeros(3)}
    m = T.measure(syr, cap, {})
    assert m["depth"] == pytest.approx(47.9) and m["gap"] == pytest.approx(-0.1)
    assert m["tip_lat"] == pytest.approx(0.0) and m["ang"] == pytest.approx(0.0, abs=1e-3)
    assert T.seated(m)
