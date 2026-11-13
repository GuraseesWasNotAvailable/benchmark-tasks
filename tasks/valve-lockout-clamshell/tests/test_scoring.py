# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for valve-lockout-clamshell, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it synthetic
samples shaped like measure()'s output, and check the frame helpers and the hole geometry directly.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402,F401
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control


def sample(**kw):
    """A sample of the finished state: halves a few degrees off their stops (the bar wedges them), bar through both
    plates and tilted 13 deg on its play, both hands clear and still."""
    m = {
        "qA": 4.8, "qB": 3.0, "stem": 0.0, "tip_x": 94.8, "off": 1.5, "tilt": 13.0, "held_shell": False,
        "held_bar": False, "bar_speed": 0.0, "bar_spin": 0.0, "shell_rate": 0.0,
        "open": {"left": 0.08, "right": 0.024}, "near_bar": {"left": 0.1, "right": 0.1},
        "clear": {"left": 0.09, "right": 0.09}, "tcp_speed": {"left": 0.0, "right": 0.0},
    }
    m.update(kw)
    return m


def at_rest(**kw):
    """Start: halves open, bar on its rib, hands at ready."""
    base = dict(qA=40.0, qB=40.0, tip_x=160.0, off=59.0, tilt=0.0, clear={"left": 0.3, "right": 0.3})
    base.update(kw)
    return sample(**base)


def holding_shut(**kw):
    """The left hand holds both halves on their stops; the bar is on its way in, held by the right hand."""
    base = dict(qA=0.0, qB=0.0, tip_x=140.0, off=0.5, tilt=0.3, held_shell=True, held_bar=True,
                clear={"left": 0.0, "right": 0.0}, tcp_speed={"left": 0.0, "right": 0.01})
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
    """Rest, left holds the halves shut, the bar goes in tip first, right lets go, left lets go, 3.5 s still."""
    out = secs(at_rest(), 2.0)
    out += secs(holding_shut(tip_x=160.0, off=10.0), 1.0)
    for k in range(40):  # tip 140 -> 91
        out.append(holding_shut(tip_x=140.0 - 1.225 * (k + 1), off=0.6))
    out += secs(holding_shut(tip_x=91.0, held_bar=True), 1.0)
    out += secs(sample(held_shell=True, qA=0.0, qB=0.0, tip_x=91.0, tilt=0.5, clear={"left": 0.0, "right": 0.09}), 1.0)
    out += secs(sample(clear={"left": 0.09, "right": 0.09}), 3.5)
    return out


def score(rec):
    return T.ladder(rec)[0]


def test_demo_trace_scores_one():
    r = feed(demo_trace())
    assert score(r) == 1.0, T.ladder(r)


def test_no_steps_scores_zero():
    assert score(T.Record()) == 0.0


def test_doing_nothing_scores_zero():
    assert score(feed(secs(at_rest(), 10))) == 0.0


def test_left_only_scores_zero():
    """The left hand shuts and holds both halves, nothing else happens."""
    r = feed(secs(at_rest(), 2.0) + secs(holding_shut(tip_x=160.0, off=59.0, held_bar=False), 6.0))
    assert score(r) == 0.0


def test_right_only_scores_zero():
    """The held bar reaches the open halves' first plate (its face is at 114 mm) and stops there."""
    r = feed(secs(at_rest(), 2.0) + secs(at_rest(held_bar=True, tip_x=114.1, off=0.4), 6.0))
    assert score(r) == 0.0


def test_first_plate_rung_025():
    r = feed(secs(at_rest(), 1.0) + secs(holding_shut(tip_x=108.9, off=0.5), 1.0) + secs(at_rest(), 2.0))
    assert score(r) == 0.25
    r = feed(secs(at_rest(), 1.0) + secs(holding_shut(tip_x=109.1, off=0.5), 1.0) + secs(at_rest(), 2.0))
    assert score(r) == 0.0
    r = feed(secs(at_rest(), 1.0) + secs(holding_shut(tip_x=108.9, off=0.5, held_bar=False), 1.0) + secs(at_rest(), 2.0))
    assert score(r) == 0.0, "a bar that is not held earns nothing"


def test_locked_then_undone_scores_05():
    r = feed(demo_trace() + secs(at_rest(), 1.0))
    assert score(r) == 0.5 and "some time" in T.ladder(r)[1]


def test_locked_at_end_but_hand_still_on_scores_075():
    tr = demo_trace()
    tr[-90:] = [dict(m, clear={"left": 0.0, "right": 0.09}) for m in tr[-90:]]
    r = feed(tr)
    assert score(r) == 0.75 and "30 mm" in T.ladder(r)[1]


def test_lever_gate_at_five_degrees_not_below():
    r = feed(demo_trace()[:60] + [holding_shut(stem=4.99)] + demo_trace()[60:])
    assert score(r) == 1.0
    r = feed(demo_trace()[:60] + [holding_shut(stem=5.0)] + demo_trace()[60:])
    assert score(r) == 0.0, "a turned lever is never cleared by turning it back"
    assert "gate" in T.ladder(r)[1]


def test_gate_beats_a_lock_at_the_end():
    r = feed([at_rest(stem=9.0)] + demo_trace())
    assert score(r) == 0.0


def test_shut_limit_is_6_degrees_for_each_half():
    assert T.shut(sample(qA=6.0, qB=6.0)) and not T.shut(sample(qA=6.1, qB=0.0)) and not T.shut(sample(qA=0.0, qB=6.1))


def test_through_limits():
    assert T.through(sample(tip_x=99.0)) and not T.through(sample(tip_x=99.1))
    assert T.through(sample(off=3.0)) and not T.through(sample(off=3.1))
    assert T.through(sample(tilt=20.0)) and not T.through(sample(tilt=20.1))


def test_a_half_left_open_at_the_end_is_not_a_lock():
    tr = demo_trace()
    tr[-90:] = [dict(m, qB=6.5) for m in tr[-90:]]
    assert score(feed(tr)) == 0.5


def test_steady_two_seconds():
    tr = demo_trace()
    tr[-35] = dict(tr[-35], bar_speed=0.004)  # 1.7 s from the end, 4 mm/s: the 2 s window restarts
    assert score(feed(tr)) == 0.75
    tr = demo_trace()
    tr[-60] = dict(tr[-60], bar_speed=0.004)  # 3 s from the end: harmless
    assert score(feed(tr)) == 1.0
    tr = demo_trace()
    tr[-30] = dict(tr[-30], bar_spin=math.radians(3.5))  # a bar turning 3.5 deg/s breaks the window too
    assert score(feed(tr)) == 0.75


def test_halves_still_moving_blocks_one():
    tr = demo_trace()
    tr[-10] = dict(tr[-10], shell_rate=2.0)
    assert score(feed(tr)) == 0.75


def test_release_timer_restarts_when_a_pad_comes_back():
    tr = demo_trace()
    tr[-15] = dict(tr[-15], clear={"left": 0.09, "right": 0.029})  # one step 29 mm away: under 1 s since
    assert score(feed(tr)) == 0.75
    tr = demo_trace()
    tr[-15] = dict(tr[-15], clear={"left": 0.09, "right": 0.030})
    assert score(feed(tr)) == 1.0


def test_a_moving_hand_blocks_the_release():
    tr = demo_trace()
    tr[-5] = dict(tr[-5], tcp_speed={"left": 0.006, "right": 0.0})
    assert score(feed(tr)) == 0.75


def test_record_is_clean_after_a_reset():
    r = feed([at_rest(stem=9.0)])
    assert score(r) == 0.0 and "gate" in T.ladder(r)[1]
    r.reset()
    r = feed(demo_trace(), rec=r)
    assert score(r) == 1.0


def test_dev_frame_round_trip_and_directions():
    p = np.array([40.0, -12.0, 7.0])
    assert np.allclose(T.world_to_dev(T.dev_to_world(p)), p)
    w0, w1 = T.dev_to_world((0, 0, 0)), T.dev_to_world((100, 0, 0))
    assert np.allclose(w1 - w0, (0.0, -0.1, 0.0)), "the lever and the ears lie toward the right arm (world -y)"
    assert np.allclose(T.dev_to_world((0, 100, 0)) - w0, (0.1, 0.0, 0.0)), "half A is the far half (world +x)"


class FakeBar:
    def __init__(self, centre_dev, tilt_deg=0.0):
        A = np.eye(4)
        A[:3, :3] = T.R_DEV @ lk.axis_rotation((0, 1, 0), math.radians(tilt_deg))
        A[:3, 3] = T.dev_to_world(centre_dev)
        self.poses = {"bar": A}


def test_bar_line_tip_and_centre_at_planes():
    c, a, tip = T._bar_line(FakeBar((190.0, 0.0, -59.0)))
    assert np.allclose(tip, (160.0, 0.0, -59.0), atol=1e-6) and np.allclose(a, (1, 0, 0), atol=1e-9)
    c, a, tip = T._bar_line(FakeBar((124.0, 0.0, 0.0), tilt_deg=13.0))
    p = T._at_x(c, a, 106.5)
    assert abs(p[0] - 106.5) < 1e-9 and abs(math.degrees(math.acos(a[0])) - 13.0) < 1e-6


def test_holes_line_up_only_when_both_halves_are_shut():
    r = math.hypot(T.HINGE_Y, -T.HINGE_Z)  # hole centre to hinge pin: 19.1 mm
    play = (T.HOLE - T.BAR_W) / 2  # 1.25 mm each side
    shift = lambda deg: r * math.radians(deg)  # noqa: E731
    assert shift(0.0) < play and shift(38.0) > 10 * play, "open, the holes are far from the bar's line"
    # both halves open the same angle: their holes move apart by about 2 x 14 sin(q) across y; the bar fits both
    # only while that spread stays within the play on both sides (about 5 deg), which CLOSED_DEG covers.
    spread = lambda deg: 2 * T.HINGE_Z * -1 * math.sin(math.radians(deg))  # noqa: E731
    assert spread(5.0) <= 2 * play + 0.1 and spread(7.0) > 2 * play
    assert T.CLOSED_DEG < 7.0


def test_bar_cannot_reach_the_first_plate_hole_when_open():
    """Open 38 deg, the first plate's hole is ~12 mm from the bar's line: the tip stops on the plate face."""
    assert math.hypot(T.HINGE_Y, -T.HINGE_Z) * math.radians(T.OPEN_DEG) > 12.0
    assert T.TIP_IN_B_MM == T.PLATE_B[0] and T.TIP_PAST_MM == T.PLATE_A[0] - 5.0
