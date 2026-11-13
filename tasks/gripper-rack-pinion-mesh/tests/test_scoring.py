# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for gripper-rack-pinion-mesh, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it
synthetic samples shaped like measure() output.
"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control


def sample(sN=20.0, sF=20.0, **kw):
    """A sample of the finished state: far rack seated, both jaws open `s` mm, hands clear."""
    m = {
        "sN": sN, "sF": sF, "F": (T.FAR_X, T.YC0 - sF * T.MM, T.Z0), "N_z": T.Z0, "tilt": 0.0, "yaw_err": 0.0,
        "clear": {"left": 0.08, "right": 0.06}, "tcp_speed": {"left": 0.0, "right": 0.0},
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
    """Rest, the near rack closed, the far rack lowered, the jaws pulled open, hands withdrawn, 3 s still."""
    lying = sample(0.0, -30.0, F=(0.172, 0.058, T.Z0), clear={"left": 0.2, "right": 0.2})
    out = secs(lying, 2.0)
    out += secs(sample(1.0, 1.0, clear={"left": 0.0, "right": 0.0}, tcp_speed={"left": 0.0, "right": 0.01}), 1.0)
    out += [sample(1.0 + k, 1.0 + k - 1.0, clear={"left": 0.1, "right": 0.0}) for k in range(0, 21)]
    out += secs(sample(21.0, 20.0, clear={"left": 0.1, "right": 0.0}), 0.5)
    out += secs(sample(21.0, 20.0), 3.0)
    return out


def test_demo_trace_scores_one():
    s, why = T.ladder(feed(demo_trace()))
    assert s == 1.0, why


def test_far_rack_not_seated_scores_zero():
    assert T.ladder(feed(secs(sample(0.0, 0.0, F=(0.172, 0.058, T.Z0)), 3.0)))[0] == 0.0


@pytest.mark.parametrize("dz,ok", [(0.0015, True), (0.0016, False)])
def test_seat_height(dz, ok):
    m = sample(20.0, 20.0, F=(T.FAR_X, T.YC0 - 0.020, T.Z0 + dz))
    assert (T.ladder(feed(secs(m, 3.0)))[0] > 0.0) == ok


@pytest.mark.parametrize("dx,ok", [(0.0019, True), (0.0021, False)])
def test_seat_sideways(dx, ok):
    m = sample(20.0, 20.0, F=(T.FAR_X + dx, T.YC0 - 0.020, T.Z0))
    assert (T.ladder(feed(secs(m, 3.0)))[0] > 0.0) == ok


@pytest.mark.parametrize("tilt,ok", [(3.0, True), (3.1, False)])
def test_seat_tilt(tilt, ok):
    assert (T.ladder(feed(secs(sample(tilt=tilt), 3.0)))[0] > 0.0) == ok


def test_one_tooth_off_is_gated_even_if_later_fixed():
    rec = feed(secs(sample(0.0, 9.42), 1.0))  # far rack seated 9.42 mm open, near rack closed: offset 4.71 mm
    s, why = T.ladder(rec)
    assert s == 0.0 and "teeth off" in why
    s, _ = T.ladder(feed(demo_trace(), rec, t0=1.0))
    assert s == 0.0


@pytest.mark.parametrize("d,gate", [(3.4, False), (3.6, True)])
def test_gate_threshold(d, gate):
    rec = feed(secs(sample(0.0, 2 * d), 0.5))
    assert ("off_tooth" in rec.H.evers) == gate


def test_symmetry_rung():
    # offset 3.0 mm: symmetric enough for the open rungs; offset 3.2 mm: not symmetric, no gate yet
    assert T.ladder(feed(secs(sample(26.0, 20.0), 3.0)))[0] > 0.25
    assert T.ladder(feed(secs(sample(26.4, 20.0), 3.0)))[0] == 0.25


def test_open_15_each():
    assert T.ladder(feed(secs(sample(15.0, 15.0), 3.0)))[0] == 1.0
    s, why = T.ladder(feed(secs(sample(15.0, 14.9), 3.0)))
    assert s == 0.5 and "15 mm" in why


def test_closed_jaws_are_half():
    assert T.ladder(feed(secs(sample(1.0, 1.0), 3.0)))[0] == 0.5


def test_release_distance_and_time():
    near = sample(clear={"left": 0.029, "right": 0.08})
    s, why = T.ladder(feed(secs(near, 3.0)))
    assert s == 0.75 and "30 mm" in why
    s, _ = T.ladder(feed(secs(sample(clear={"left": 0.030, "right": 0.08}), 3.0)))
    assert s == 1.0
    s, _ = T.ladder(feed(secs(sample(), 0.9)))  # 0.9 s clear is not 1 s
    assert s == 0.75


def test_hand_still_for_release():
    moving = sample(tcp_speed={"left": 0.006, "right": 0.0})
    assert T.ladder(feed(secs(moving, 3.0)))[0] == 0.75


def test_racks_must_be_still():
    sl = [sample(20.0 + 0.5 * k, 20.0 + 0.5 * k) for k in range(60)]  # 10 mm/s
    assert T.ladder(feed(sl))[0] == 0.75


def test_dropped_rack_gates_forever():
    rec = feed(secs(sample(), 1.0))
    rec = feed(secs(sample(F=(T.FAR_X, T.YC0 - 0.02, T.TOP - 0.02)), 0.2), rec, t0=1.0)
    rec = feed(demo_trace(), rec, t0=1.2)
    s, why = T.ladder(rec)
    assert s == 0.0 and "fell" in why


def test_reset_clears_the_record():
    rec = feed(secs(sample(0.0, 9.42), 1.0))
    assert "off_tooth" in rec.H.evers
    rec.reset()
    assert T.ladder(rec)[0] == 0.0 and not rec.H.evers
    assert T.ladder(feed(demo_trace(), rec))[0] == 1.0


def test_offset_formula():
    assert T.offset_mm({"sN": 10.0, "sF": 4.0}) == 3.0
    assert T.offset_mm({"sN": 4.0, "sF": 10.0}) == -3.0
