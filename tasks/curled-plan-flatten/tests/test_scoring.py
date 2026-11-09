# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for curled-plan-flatten, on hand-built poses (no simulator). Run: pytest -q tests"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402
import task as T  # noqa: E402

DT = 0.05  # one control step at 20 Hz
Z_SLAT = T.PLAN_TOP + T.SLAT[2] / 2
Z_BAR = T.PLAN_TOP + T.SLAT[2] + T.W_SIZE[2] / 2


def pose(x, y, z, roll_deg=0.0, yaw_deg=0.0):
    m = np.eye(4)
    m[:3, :3] = lk.axis_rotation((0, 0, 1), math.radians(yaw_deg)) @ lk.axis_rotation((1, 0, 0), math.radians(roll_deg))
    m[:3, 3] = (x, y, z)
    return m


def flat_plan(dy=0.0, dx=0.0):
    return [pose(0.18 + dx, T.slat_y(i) + dy, Z_SLAT) for i in range(T.N)]


def bars_on_ends(slats, out_mm=0.0):
    """Weights centred on both end slats, each shifted ``out_mm`` past its slat's centre toward the plan's end."""
    yl = slats[-1][1, 3] + out_mm / 1000
    yr = slats[0][1, 3] - out_mm / 1000
    return {"weight_L": pose(slats[-1][0, 3], yl, Z_BAR), "weight_R": pose(slats[0][0, 3], yr, Z_BAR)}


def bars_parked():
    return {"weight_L": pose(0.18, 0.235, T.PLAN_TOP + T.W_SIZE[2] / 2),
            "weight_R": pose(0.18, -0.235, T.PLAN_TOP + T.W_SIZE[2] / 2)}


def run(steps, h=None, t0=0.0):
    """Feed (m, clear_m, tcp_speed, obj_speed) tuples, one per control step, into a fresh or given record."""
    h = h or lk.History()
    t = t0 if h.t == 0 else h.t
    for m, clear, tcp, obj in steps:
        t += DT
        h._tick(t)
        T.update(h, m, {"left": clear, "right": clear}, {"left": tcp, "right": tcp}, obj)
    return h


def hold(m, seconds, clear=0.10, tcp=0.0, obj=0.0):
    return [(m, clear, tcp, obj)] * round(seconds / DT)


def done():
    s = flat_plan()
    return T.measure(s, bars_on_ends(s))


# -- geometry -------------------------------------------------------------------------------------------------------


def test_overlap_area_rectangles():
    a = T.footprint(pose(0, 0, 0), 0.18, 0.025)
    assert T.overlap_area(a, a) == pytest.approx(0.18 * 0.025, rel=1e-9)
    b = T.footprint(pose(0, 0.01, 0), 0.16, 0.025)
    assert T.overlap_area(a, b) == pytest.approx(0.16 * 0.015, rel=1e-9)
    c = T.footprint(pose(0, 0, 0, yaw_deg=90), 0.16, 0.025)  # crossed: 25 x 25 mm square
    assert T.overlap_area(a, c) == pytest.approx(0.025 * 0.025, rel=1e-9)
    assert T.overlap_area(a, T.footprint(pose(0, 0.05, 0), 0.16, 0.025)) == 0.0


def test_rest_pose_is_curled_and_scores_zero():
    poses = T.PLAN.link_poses()  # the initial hinge angles (the resting curl)
    m = T.measure([poses[f"s{i}"] for i in range(T.N)], bars_parked())
    assert m["flat_run"] < 8 and not m["half_flat"]
    assert not m["weighted"]["left"] and not m["weighted"]["right"]
    h = run(hold(m, 5.0))
    assert T.ladder(h)[0] == 0.0


def test_design_pose_is_the_flat_plan_in_the_outline():
    poses = T.PLAN.link_poses({j.name: 0.0 for j in T.PLAN.moving_joints})
    m = T.measure([poses[f"s{i}"] for i in range(T.N)], bars_parked())
    assert m["flat"] and m["flat_run"] == T.N
    assert m["centre_err"] < 1e-9 and m["axis_deg"] < 1e-6


# -- flat, cover and seat thresholds --------------------------------------------------------------------------------


@pytest.mark.parametrize("dz_mm, flat", [(3.9, True), (4.1, False), (-3.9, True), (-4.1, False)])
def test_flat_height_threshold(dz_mm, flat):
    s = flat_plan()
    s[5][2, 3] += dz_mm / 1000
    m = T.measure(s, bars_on_ends(s))
    assert m["flat_each"][5] is flat and m["flat"] is flat


@pytest.mark.parametrize("tilt, flat", [(5.9, True), (6.1, False)])
def test_flat_tilt_threshold(tilt, flat):
    s = flat_plan()
    s[9] = pose(0.18, T.slat_y(9), Z_SLAT, roll_deg=tilt)
    assert T.measure(s, bars_on_ends(s))["flat_each"][9] is flat


@pytest.mark.parametrize("cover, weighted", [(0.45, True), (0.43, False)])  # 0.44: half the bar on the end slat
def test_end_weighted_cover_threshold(cover, weighted):
    # bar along x over the slat's full width: shared area = 0.16 m x (25 mm - shift)
    shift_mm = 1000 * (T.SLAT[1] - cover * T.SLAT[0] * T.SLAT[1] / T.W_SIZE[0])
    s = flat_plan()
    m = T.measure(s, bars_on_ends(s, out_mm=shift_mm))
    assert m["cover"]["left"] == pytest.approx(cover, abs=1e-6)
    assert m["weighted"]["left"] is weighted and m["weighted"]["right"] is weighted


@pytest.mark.parametrize("gap_mm, weighted", [(2.9, True), (3.1, False)])
def test_end_weighted_seat_threshold(gap_mm, weighted):
    s = flat_plan()
    b = bars_on_ends(s)
    b["weight_R"][2, 3] += gap_mm / 1000  # held above the end slat
    assert T.measure(s, b)["weighted"]["right"] is weighted


def test_one_weight_cannot_weight_both_ends():
    s = flat_plan()
    b = bars_on_ends(s)
    b["weight_R"] = b["weight_L"].copy()  # both on the left end
    m = T.measure(s, b)
    assert m["weighted"]["left"] and not m["weighted"]["right"] and not m["both_ends"]


# -- the ladder -----------------------------------------------------------------------------------------------------


def test_demo_end_state_scores_one():
    h = run(hold(done(), 3.0))
    assert T.ladder(h)[0] == 1.0


def test_flat_free_needs_three_seconds():
    h = run(hold(done(), 2.9))
    assert T.ladder(h)[0] == 0.75
    run(hold(done(), 0.1), h)
    assert T.ladder(h)[0] == 1.0


def test_flat_free_restarts_on_a_touch():
    h = run(hold(done(), 2.5))
    run(hold(done(), 0.05, clear=0.02), h)  # a pad 20 mm from a weight for one step
    run(hold(done(), 2.5), h)
    assert T.ladder(h)[0] == 0.75  # 2.5 s since the touch, not 5.0
    run(hold(done(), 0.5), h)
    assert T.ladder(h)[0] == 1.0


def test_released_needs_still_grippers():
    h = run(hold(done(), 2.5) + hold(done(), 0.6, tcp=0.02))  # clear but still moving away
    assert T.ladder(h)[0] == 0.75
    run(hold(done(), 0.9), h)
    assert T.ladder(h)[0] == 0.75  # 0.9 s still: not yet 1.0 s
    run(hold(done(), 0.1), h)
    assert T.ladder(h)[0] == 1.0


def test_moving_plan_is_not_flat_free():
    h = run(hold(done(), 4.0, obj=0.004))
    assert T.ladder(h)[0] == 0.75


def test_holding_a_weight_caps_at_075():
    h = run(hold(done(), 6.0, clear=0.0))
    assert T.ladder(h)[0] == 0.75


@pytest.mark.parametrize("off_mm, score", [(14.0, 1.0), (16.0, 0.75)])
def test_centred_tolerance(off_mm, score):
    s = flat_plan(dy=off_mm / 1000)
    h = run(hold(T.measure(s, bars_on_ends(s)), 3.5))
    assert T.ladder(h)[0] == score


@pytest.mark.parametrize("yaw, score", [(4.9, 1.0), (5.1, 0.75)])
def test_axis_tolerance(yaw, score):
    r = lk.axis_rotation((0, 0, 1), math.radians(yaw))
    s = []
    for m in flat_plan():
        p = m.copy()
        p[:3, 3] = np.r_[r[:2, :2] @ (m[:2, 3] - T.CENTRE) + T.CENTRE, m[2, 3]]
        p[:3, :3] = r
        s.append(p)
    b = {}
    for name, i in (("weight_L", -1), ("weight_R", 0)):
        q = s[i].copy()
        q[2, 3] = Z_BAR
        b[name] = q
    h = run(hold(T.measure(s, b), 3.5))
    assert T.ladder(h)[0] == score


def test_weight_off_the_station_table_is_zero_even_after_return():
    s = flat_plan()
    b = bars_on_ends(s)
    off = dict(b, weight_L=pose(0.18, 0.60, T.TOP - T.DROP - 0.001 + T.W_SIZE[2] / 2))  # fell off the station table
    h = run(hold(T.measure(s, off), 0.05) + hold(done(), 5.0))
    assert T.ladder(h)[0] == 0.0
    assert "fell off the station table" in T.ladder(h)[1]


def test_weight_set_down_on_the_station_table_is_not_off():
    s = flat_plan()
    b = dict(bars_on_ends(s), weight_L=pose(0.18, 0.40, T.TOP + T.W_SIZE[2] / 2))  # a regrasp beside the plan table
    h = run(hold(T.measure(s, b), 0.5) + hold(done(), 3.5))
    assert T.ladder(h)[0] == 1.0


def test_weight_lifted_in_the_air_is_not_off():
    s = flat_plan()
    b = dict(bars_on_ends(s), weight_L=pose(0.18, 0.10, T.PLAN_TOP + 0.05))
    h = run(hold(T.measure(s, b), 0.5) + hold(done(), 3.0))
    assert T.ladder(h)[0] == 1.0


@pytest.mark.parametrize("tilt_end, score", [(5.9, 0.25), (6.1, 0.0)])
def test_half_open_needs_one_half_flat_out_to_its_end(tilt_end, score):
    s = flat_plan()
    for i in range(0, 8):  # the right half curled up 10 mm
        s[i][2, 3] += 0.010
    s[15] = pose(0.18, T.slat_y(15), Z_SLAT, roll_deg=tilt_end)  # the left end slat
    m = T.measure(s, bars_parked())
    assert m["half_flat"] is (score > 0)
    h = run(hold(m, 0.05) + hold(T.measure([pose(0.18, 0, 0.9)] * T.N, bars_parked()), 1.0))  # then curls back
    assert T.ladder(h)[0] == score


def test_weights_crushing_the_rolled_ends_score_zero():
    """The weights_on_tube outcome in the simulator: set on the rolled ends, the weights squash the curls, so 14
    middle slats lie flat, but both end slats stay curled up 20 deg and neither end is weighted."""
    s = flat_plan()
    s[0] = pose(0.18, T.slat_y(0), Z_SLAT + 0.0043, roll_deg=20)
    s[15] = pose(0.18, T.slat_y(15), Z_SLAT + 0.0042, roll_deg=-20)
    b = {"weight_L": pose(0.18, 0.155, Z_BAR), "weight_R": pose(0.18, -0.158, Z_BAR)}
    m = T.measure(s, b)
    assert m["flat_run"] == 14 and not m["half_flat"] and not m["both_ends"]
    assert T.ladder(run(hold(m, 50.0)))[0] == 0.0


def test_one_end_is_remembered_as_half():
    s = flat_plan()
    b = dict(bars_on_ends(s), weight_R=pose(0.18, 0.0, Z_BAR))  # left end weighted, right weight in the middle
    for i in range(0, 4):  # the right end curls
        s[i][2, 3] += 0.02
    m = T.measure(s, b)
    assert m["one_end"] and not m["both_ends"]
    h = run(hold(m, 0.5) + hold(T.measure([pose(0.18, 0, 0.9)] * T.N, bars_parked()), 2.0))
    assert T.ladder(h)[0] == 0.5


def test_one_end_needs_its_whole_half_flat():
    s = flat_plan()
    b = dict(bars_on_ends(s), weight_R=pose(0.18, -0.235, T.PLAN_TOP + T.W_SIZE[2] / 2))
    s[9][2, 3] += 0.01  # a bump in the left half
    assert not T.measure(s, b)["one_end"]


def test_reset_clears_the_record():
    s = flat_plan()
    off = dict(bars_on_ends(s), weight_L=pose(0.18, 0.60, T.TOP - 0.10))
    h = run(hold(T.measure(s, off), 0.5))
    assert T.ladder(h)[0] == 0.0
    h.reset()
    run(hold(done(), 3.0), h)
    assert T.ladder(h)[0] == 1.0


def test_no_steps_scores_zero():
    assert T.ladder(lk.History())[0] == 0.0


# -- stillness: speed from body poses at the 20 Hz control rate ------------------------------------------------------


def _centres(move_mm=0.0):
    s = flat_plan()
    b = bars_on_ends(s)
    now = {f"s{i}": s[i][:3, 3].copy() for i in range(T.N)}
    now.update({w: b[w][:3, 3].copy() for w in T.WEIGHTS})
    now["s3"] = now["s3"] + np.array([0.0, move_mm / 1000, 0.0])
    return now


@pytest.mark.parametrize("move_mm, still", [(0.14, True), (0.16, False)])
def test_pose_speed_threshold(move_mm, still):
    v = T.pose_speed(_centres(), _centres(move_mm), DT)  # 0.14 mm in one 50 ms step = 2.8 mm/s
    assert v == pytest.approx(move_mm / 1000 / DT)
    assert (v < T.STILL_OBJ) is still


def test_pose_speed_needs_a_previous_step():
    assert T.pose_speed(None, _centres(), DT) == math.inf
    assert T.pose_speed({}, _centres(), DT) == math.inf
    assert T.pose_speed(_centres(), _centres(), 0.0) == math.inf
