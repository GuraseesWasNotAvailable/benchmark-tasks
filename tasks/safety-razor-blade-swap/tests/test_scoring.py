# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for safety-razor-blade-swap, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it
synthetic samples, and build real Tool objects for the cutting-edge gate.
"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control
NOEDGE = {"left": False, "right": False}


def sample(**kw):
    """The finished state: blade seated, doors closed, knob back, hands clear and still."""
    m = {"dx": 0.0, "dy": 0.3, "dz": 0.2, "yaw": 0.0, "tilt": 0.0, "knob": 0.0, "wing_min": 0.0, "wing_max": 0.0,
         "lift": 0.0, "edge": dict(NOEDGE), "held": {"left": False, "right": False},
         "clear": {"left": 0.045, "right": 0.068}, "tcp_speed": {"left": 0.0, "right": 0.0}, "speed": 0.0,
         "spin": 0.0}
    m.update(kw)
    return m


def carrying(**kw):
    """Held by the right gripper 30 mm above the plate, doors open."""
    base = dict(dy=0.0, dz=30.0, knob=90.0, wing_min=100.0, wing_max=100.0, lift=0.03,
                held={"left": False, "right": True}, clear={"left": 0.0, "right": 0.0})
    base.update(kw)
    return sample(**base)


def seated_open(**kw):
    base = dict(knob=90.0, wing_min=100.0, wing_max=100.0, clear={"left": 0.0, "right": 0.0})
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
    """Rest, blade carried with the doors open, seated, doors eased shut, hands withdrawn, 2 s still."""
    out = secs(sample(dy=-120.0, clear={"left": 0.2, "right": 0.2}), 2.0)
    out += secs(carrying(), 2.0)
    out += secs(seated_open(), 1.0)
    out += secs(sample(), 3.0)
    return out


def score(samples):
    return T.ladder(feed(samples))[0]


# -- the ladder -----------------------------------------------------------------------------------------------------


def test_demo_shaped_trace_scores_1():
    assert score(demo_trace()) == 1.0


def test_nothing_scores_0():
    assert score(secs(sample(dy=-120.0, clear={"left": 0.2, "right": 0.2}), 5.0)) == 0.0


def test_carried_with_doors_shut_is_0():
    assert score(secs(carrying(wing_min=0.0, wing_max=0.0, knob=0.0), 3.0)) == 0.0


def test_carried_needs_the_doors_open_60_deg():
    assert score(secs(carrying(wing_min=59.0, wing_max=59.0), 2.0)) == 0.0
    assert score(secs(carrying(wing_min=60.0, wing_max=60.0), 2.0)) == 0.25


def test_carried_needs_20_mm_lift_and_a_grip():
    assert score(secs(carrying(lift=0.0199), 2.0)) == 0.0
    assert score(secs(carrying(lift=0.020), 2.0)) == 0.25
    assert score(secs(carrying(held={"left": False, "right": False}), 2.0)) == 0.0


def test_left_hand_holding_the_blade_also_counts():
    assert score(secs(carrying(held={"left": True, "right": False}), 2.0)) == 0.25


def test_seated_with_doors_open_is_half():
    assert score(secs(carrying(), 1.0) + secs(seated_open(), 2.0)) == 0.5


def test_blade_resting_on_the_post_tops_is_not_seated():
    assert score(secs(carrying(), 1.0) + secs(seated_open(dz=2.0), 2.0)) == 0.25


def test_doors_closed_on_a_seated_blade_without_the_release_is_three_quarters():
    held_tight = sample(clear={"left": 0.02, "right": 0.068})
    r = feed(secs(carrying(), 1.0) + secs(seated_open(), 1.0) + secs(held_tight, 3.0))
    s, why = T.ladder(r)
    assert s == 0.75 and "30 mm clear" in why


def test_doors_closed_on_a_blade_that_was_never_seated_open_is_not_a_pass():
    # e.g. the blade carried in with the doors shut, then the knob turned: no seated-with-doors-open moment
    assert score(secs(carrying(), 1.0) + secs(sample(), 4.0)) == 0.25


def test_doors_stalled_open_at_2_deg_or_more_cap_at_half():
    tr = secs(carrying(), 1.0) + secs(seated_open(), 1.0) + secs(sample(wing_max=2.1, wing_min=0.0), 4.0)
    assert score(tr) == 0.5
    tr = secs(carrying(), 1.0) + secs(seated_open(), 1.0) + secs(sample(wing_max=2.0), 4.0)
    assert score(tr) == 1.0


def test_knob_not_returned_caps_at_half():
    tr = secs(carrying(), 1.0) + secs(seated_open(), 1.0) + secs(sample(knob=15.1), 4.0)
    assert score(tr) == 0.5
    tr = secs(carrying(), 1.0) + secs(seated_open(), 1.0) + secs(sample(knob=15.0), 4.0)
    assert score(tr) == 1.0


@pytest.mark.parametrize("field, ok, bad", [("dx", 1.5, 1.6), ("dy", 1.5, 1.6), ("yaw", 5.0, 5.1), ("tilt", 3.0, 3.1)])
def test_seat_limits(field, ok, bad):
    assert T.seated(sample(**{field: ok})) and not T.seated(sample(**{field: bad}))
    assert T.seated(sample(**{field: -ok})) or field == "tilt"


def test_seat_height_limits():
    assert T.seated(sample(dz=0.2 + 0.59)) and not T.seated(sample(dz=0.2 + 0.61))
    assert T.seated(sample(dz=-0.09)) and not T.seated(sample(dz=-0.2))


def test_blade_moving_at_the_end_is_three_quarters():
    tr = secs(carrying(), 1.0) + secs(seated_open(), 1.0) + secs(sample(speed=1.5), 4.0)
    assert score(tr) == 0.75
    tr = secs(carrying(), 1.0) + secs(seated_open(), 1.0) + secs(sample(speed=0.9, spin=1.9), 4.0)
    assert score(tr) == 1.0
    tr = secs(carrying(), 1.0) + secs(seated_open(), 1.0) + secs(sample(spin=2.1), 4.0)
    assert score(tr) == 0.75


def test_release_needs_one_full_second():
    tr = secs(carrying(), 1.0) + secs(seated_open(), 1.0) + secs(sample(clear={"left": 0.0, "right": 0.0}), 3.0)
    assert score(tr + secs(sample(), 0.95)) == 0.75
    assert score(tr + secs(sample(), 1.0)) == 1.0


def test_hand_still_moving_restarts_the_release_timer():
    tr = secs(carrying(), 1.0) + secs(seated_open(), 1.0) + secs(sample(clear={"left": 0.0, "right": 0.0}), 3.0)
    tr += secs(sample(), 0.5) + [sample(tcp_speed={"left": 0.011, "right": 0.0})] + secs(sample(), 0.5)
    assert score(tr) == 0.75
    tr = tr + secs(sample(), 1.0)
    assert score(tr) == 1.0


def test_clear_of_30_mm_is_enough_but_29_is_not():
    base = secs(carrying(), 1.0) + secs(seated_open(), 1.0) + secs(sample(clear={"left": 0.0, "right": 0.0}), 3.0)
    assert score(base + secs(sample(clear={"left": 0.030, "right": 0.030}), 1.2)) == 1.0
    assert score(base + secs(sample(clear={"left": 0.029, "right": 0.050}), 1.2)) == 0.75


# -- the gate ------------------------------------------------------------------------------------------------------


def test_edge_touch_gate_beats_a_perfect_finish():
    tr = demo_trace()
    tr[10] = carrying(edge={"left": False, "right": True})
    s, why = T.ladder(feed(tr))
    assert s == 0.0 and "cutting edges" in why


def test_edge_touch_is_not_cleared_by_starting_again():
    tr = secs(carrying(edge={"left": True, "right": False}), 1.0) + secs(sample(dy=-120.0), 2.0) + demo_trace()
    assert score(tr) == 0.0


def test_the_record_is_clean_after_a_reset():
    rec = feed(secs(carrying(edge={"left": False, "right": True}), 1.0))
    assert "edge" in rec.H.evers
    rec.reset()
    assert rec.last is None and not rec.H.evers
    feed(demo_trace(), rec)
    assert T.ladder(rec)[0] == 1.0


def _pinch(v_mm, opening=0.0004, u_over=0.004, z_mm=0.0, blade_yaw=0.0):
    """A gripper approaching along +x with its pads closing vertically on the blade's near end, ``v_mm`` off the centre."""
    B = np.eye(4)
    c, s = math.cos(math.radians(blade_yaw)), math.sin(math.radians(blade_yaw))
    B[:2, :2] = [[c, -s], [s, c]]
    rot = np.column_stack([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]])
    tcp = np.array([-T.BL / 2 + u_over - 0.013, v_mm / 1000.0, z_mm / 1000.0])
    return lk.Tool("right", tcp, rot, opening, 0.0), B


def test_pinch_on_the_centre_line_is_clean():
    tool, B = _pinch(0.0)
    assert not T.edge_touch(tool, B)


def test_pad_footprint_reaching_the_outer_3_mm_is_a_touch():
    assert not T.edge_touch(*_pinch(1.9))
    assert T.edge_touch(*_pinch(2.1))
    assert T.edge_touch(*_pinch(-2.1))
    assert not T.edge_touch(*_pinch(-1.9))


def test_pad_far_from_the_blade_plane_is_no_touch():
    tool, B = _pinch(8.0, opening=0.094)  # open 94 mm: pads 47 mm above and below
    assert not T.edge_touch(tool, B)


def test_pad_over_the_edge_band_but_off_the_blade_is_no_touch():
    assert not T.edge_touch(*_pinch(8.0, u_over=-0.010))  # 10 mm short of the blade's end


def test_a_pad_beyond_the_blade_sides_is_no_touch():
    tool, B = _pinch(30.0)
    assert not T.edge_touch(tool, B)


def test_edge_gate_follows_a_yawed_blade():
    tool, B = _pinch(0.0, blade_yaw=15.0)
    assert T.edge_touch(tool, B)  # a 15 deg yaw puts the pads' footprint over an edge of the turned blade


# -- measurements --------------------------------------------------------------------------------------------------


def test_blade_rel_reports_mm_and_degrees():
    B = np.eye(4)
    B[:3, 3] = [0.2410, -0.0005, 0.1304]
    m = T.blade_rel(B, [0.24, 0.0, 0.130])
    assert m["dx"] == pytest.approx(1.0) and m["dy"] == pytest.approx(-0.5) and m["dz"] == pytest.approx(0.4)
    assert m["yaw"] == pytest.approx(0.0) and m["tilt"] == pytest.approx(0.0)


def test_blade_rel_yaw_and_tilt():
    B = lk.axis_rotation([0, 0, 1], math.radians(4.0))
    M = np.eye(4)
    M[:3, :3] = B[:3, :3] if B.shape == (4, 4) else B
    m = T.blade_rel(M, [0, 0, 0])
    assert m["yaw"] == pytest.approx(4.0, abs=1e-6)
    M[:3, :3] = lk.axis_rotation([1, 0, 0], math.radians(2.5))[:3, :3]
    assert T.blade_rel(M, [0, 0, 0])["tilt"] == pytest.approx(2.5, abs=1e-6)


def test_pose_rates_from_pose_change():
    A0 = np.eye(4)
    A1 = np.eye(4)
    A1[0, 3] = 0.0001  # 0.1 mm in one 50 ms step is 2 mm/s
    v, w = T.pose_rates(A0, A1, DT)
    assert v == pytest.approx(2.0) and w == pytest.approx(0.0, abs=1e-6)
    A1 = np.eye(4)
    A1[:3, :3] = lk.axis_rotation([0, 0, 1], math.radians(0.1))[:3, :3]
    assert T.pose_rates(A0, A1, DT)[1] == pytest.approx(2.0, abs=1e-3)


def test_blade_holes_clear_the_posts_by_0_8_mm():
    assert T.HOLE / 2 - T.POST_R == pytest.approx(0.0008)
    assert T.POST_H > T.BT + 0.0010  # a blade perched on the post tops is 1.2 mm or more above the seat


def test_door_and_cam_geometry():
    assert T.WING_OPEN == 100.0 and T.KNOB_OPEN == 90.0
    assert T.WING_Y - T.BW / 2 == pytest.approx(0.0015)  # open doors stand 1.5 mm outside each cutting edge
    assert 0.0006 > T.BT  # the closed doors underside (0.6 mm) is above the seated blade top (0.4 mm)
