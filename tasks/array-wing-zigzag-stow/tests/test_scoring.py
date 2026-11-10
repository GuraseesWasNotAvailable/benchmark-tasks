# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for array-wing-zigzag-stow, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); most tests feed it
synthetic samples at 20 Hz. The kinematics tests pose the wing's links from hinge angles and check the stack geometry
the task relies on.
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
DROOP = -math.degrees(math.asin(3 * T.PANEL_T / T.L))  # P4 hanging from h3 onto the rail after the first fold


def sample(a1=0.0, a2=0.0, a3=0.0, rate=0.0, speed=0.0, clear=(0.0, 0.0), tcp=(0.0, 0.0)):
    return {"a1": a1, "a2": a2, "a3": a3, "rate": rate, "speed": speed,
            "clear": {"left": clear[0], "right": clear[1]}, "tcp_speed": {"left": tcp[0], "right": tcp[1]}}


class Run:
    """Feeds samples at 20 Hz; unless given, the panel rate is the fastest hinge-angle change (a fair stand-in for
    the pose-change rate the simulator measures)."""

    def __init__(self):
        self.rec = T.Record()
        self.t = 0.0
        self.prev = None

    def step(self, rate=None, **kw):
        m = sample(**kw)
        if rate is not None:
            m["rate"] = rate
        elif self.prev is not None:
            m["rate"] = max(abs(m[k] - self.prev[k]) for k in ("a1", "a2", "a3")) / DT
        self.prev = m
        self.t += DT
        self.rec.update(m, self.t)
        return self

    def hold(self, seconds, **kw):
        for _ in range(round(seconds / DT)):
            self.step(**kw)
        return self

    def move(self, start: dict, end: dict, deg_s: float, **kw):
        span = max(abs(end[k] - start[k]) for k in end)
        n = max(1, round(span / deg_s / DT))
        for k in range(1, n + 1):
            self.step(**{j: start[j] + (end[j] - start[j]) * k / n for j in end}, **kw)
        return self

    def score(self):
        return T.ladder(self.rec)[0]


FLAT = {"a1": 0.0, "a2": 0.0, "a3": 0.0}
UP = {"a1": 92.0, "a2": -180.0, "a3": 60.0}
FOLD1 = {"a1": 180.0, "a2": -180.0, "a3": DROOP}
STOWED = {"a1": 180.0, "a2": -180.0, "a3": 180.0}
CLEAR = {"clear": (0.06, 0.08)}


def demo_run(release=1.2, steady=None):
    r = Run().hold(0.5, **FLAT)
    r.move(FLAT, UP, 30).move(UP, FOLD1, 30)
    r.move(FOLD1, {"a1": 180.0, "a2": -180.0, "a3": 92.0}, 30)
    r.move({"a1": 180.0, "a2": -180.0, "a3": 92.0}, STOWED, 30)
    r.hold(0.5, **STOWED)
    r.hold(release, **STOWED, **CLEAR)
    return r


def test_demo_shaped_run_scores_one():
    r = demo_run()
    assert r.score() == 1.0
    assert T.success(None) is False  # the module record is untouched by test runs


def test_release_must_last_one_second():
    assert demo_run(release=0.95).score() == 0.75
    assert demo_run(release=1.0).score() == 1.0


def test_gripper_29_mm_from_the_wing_is_not_released():
    r = demo_run(release=0.0).hold(1.2, **STOWED, clear=(0.029, 0.08))
    assert r.score() == 0.75
    r = demo_run(release=0.0).hold(1.2, **STOWED, clear=(0.030, 0.030))
    assert r.score() == 1.0


def test_moving_gripper_is_not_released():
    r = demo_run(release=0.0).hold(1.2, **STOWED, clear=(0.06, 0.08), tcp=(0.005, 0.0))
    assert r.score() == 0.75
    r = demo_run(release=0.0).hold(1.2, **STOWED, clear=(0.06, 0.08), tcp=(0.0049, 0.0))
    assert r.score() == 1.0


def test_stack_must_be_still():
    r = demo_run(release=0.0).hold(1.2, **STOWED, **CLEAR, rate=3.0)
    assert r.score() == 0.75
    r = demo_run(release=0.0).hold(1.2, **STOWED, **CLEAR, speed=0.003)
    assert r.score() == 0.75
    r = demo_run(release=0.0).hold(1.2, **STOWED, **CLEAR, rate=2.9, speed=0.0029)
    assert r.score() == 1.0


def test_gate_fall_onto_stack_beats_a_perfect_end():
    r = Run().hold(0.5, **FLAT).step(**FLAT, rate=180.1)
    r.move(FLAT, FOLD1, 30).move(FOLD1, STOWED, 30).hold(2.0, **STOWED, **CLEAR)
    assert r.score() == 0.0
    assert "gate" in T.ladder(r.rec)[1]


def test_gate_threshold_is_strict():
    r = Run().hold(0.5, **FLAT).step(**FLAT, rate=180.0)
    r.move(FLAT, FOLD1, 30).move(FOLD1, STOWED, 30).hold(2.0, **STOWED, **CLEAR)
    assert r.score() == 1.0


def test_gate_is_never_erased():
    r = Run().hold(0.5, **FLAT).step(**FLAT, rate=400.0)
    r.move(FLAT, UP, 30).move(UP, FOLD1, 30).move(FOLD1, STOWED, 30).hold(2.0, **STOWED, **CLEAR)
    assert r.score() == 0.0


def test_dropped_panel_rate_is_over_the_gate():
    # a 20 g, 80 mm panel let go at 120 deg against 0.003 N m friction reaches the stack well over the gate
    m, L, f = T.PANEL_MASS, T.L, T.HINGE_FRICTION
    inertia = m * L * L / 3
    work = m * 9.81 * L / 2 * math.sin(math.radians(60)) - f * math.radians(60)
    assert math.degrees(math.sqrt(2 * work / inertia)) > 3 * T.GATE_RATE
    # it does start falling there: gravity torque at 120 deg beats the friction
    assert m * 9.81 * L / 2 * math.cos(math.radians(60)) > f
    # and a panel parked just past upright (92 deg) is held by the friction alone
    assert m * 9.81 * L / 2 * abs(math.cos(math.radians(92))) < f


@pytest.mark.parametrize("a1,a2,want", [(170.0, -170.0, 0.25), (169.9, -175.0, 0.0), (175.0, -169.9, 0.0)])
def test_zigzag_rung_thresholds(a1, a2, want):
    r = Run().hold(0.5, **FLAT).move(FLAT, {"a1": a1, "a2": a2, "a3": 0.0}, 30).hold(1.0, a1=a1, a2=a2)
    r.move({"a1": a1, "a2": a2, "a3": 0.0}, FLAT, 30).hold(0.5, **FLAT)
    assert r.score() == want


def test_zigzag_needs_both_hinges_at_once():
    r = Run().hold(0.5, **FLAT)
    r.move(FLAT, {"a1": 175.0, "a2": 0.0, "a3": 0.0}, 30).move({"a1": 175.0, "a2": 0.0, "a3": 0.0}, FLAT, 30)
    r.move(FLAT, {"a1": 0.0, "a2": -175.0, "a3": 0.0}, 30).hold(0.5, a2=-175.0)
    assert r.score() == 0.0


@pytest.mark.parametrize("a3,want", [(95.0, 0.5), (94.9, 0.25)])
def test_p4_turned_rung(a3, want):
    r = Run().hold(0.5, **FLAT).move(FLAT, FOLD1, 30)
    r.move(FOLD1, {"a1": 180.0, "a2": -180.0, "a3": a3}, 30).hold(0.5, a1=180.0, a2=-180.0, a3=a3)
    assert r.score() == want


def test_same_way_tip_fold_earns_nothing():
    # P4 turned fully onto P3 and the pair then jammed on h2's stop with P2 tipped up: no zig-zag at any time
    r = Run().hold(0.5, **FLAT).move(FLAT, {"a1": 0.0, "a2": 0.0, "a3": 180.0}, 30)
    r.move({"a1": 0.0, "a2": 0.0, "a3": 180.0}, {"a1": 150.0, "a2": T.STOP_DEG, "a3": 180.0}, 30)
    r.hold(2.0, a1=150.0, a2=T.STOP_DEG, a3=180.0, **CLEAR)
    assert r.score() == 0.0


@pytest.mark.parametrize("hinge,val", [("a1", 176.9), ("a2", -176.9), ("a3", 176.9)])
def test_flat_stack_tolerance(hinge, val):
    end = dict(STOWED, **{hinge: val})
    r = Run().hold(0.5, **FLAT).move(FLAT, FOLD1, 30).move(FOLD1, end, 30).hold(2.0, **end, **CLEAR)
    assert r.score() == 0.5
    end = dict(STOWED, **{hinge: math.copysign(177.0, val)})
    r = Run().hold(0.5, **FLAT).move(FLAT, FOLD1, 30).move(FOLD1, end, 30).hold(2.0, **end, **CLEAR)
    assert r.score() == 1.0


def test_unfolding_at_the_end_drops_to_the_history_rung():
    r = demo_run()
    r.move(STOWED, FOLD1, 30).hold(2.0, **FOLD1, **CLEAR)
    assert r.score() == 0.5


def test_reset_clears_the_record():
    r = Run().hold(0.5, **FLAT).step(**FLAT, rate=999.0)
    assert r.score() == 0.0
    r.rec.reset()
    assert r.rec.last is None and not r.rec.H.evers and not r.rec.H.peaks
    r.t = 0.0
    r.prev = None
    r.hold(0.5, **FLAT).move(FLAT, FOLD1, 30).move(FOLD1, STOWED, 30).hold(2.0, **STOWED, **CLEAR)
    assert r.score() == 1.0


def test_pose_rates():
    a = np.eye(4)
    b = np.eye(4)
    b[:3, :3] = lk.axis_rotation(np.array([0.0, 1.0, 0.0]), math.radians(9.0))
    b[:3, 3] = [0.0001, 0.0, 0.0]
    rate, speed = T.pose_rates({"p2": a}, {"p2": b}, 0.05)
    assert rate == pytest.approx(180.0, abs=1e-6)
    assert speed == pytest.approx(0.002, abs=1e-9)
    assert T.pose_rates({}, {"p2": b}, 0.05) == (0.0, 0.0)


# -- kinematics of the wing ------------------------------------------------------------------------------------


def poses(a1, a2, a3):
    return T.WING.link_poses({"h1": math.radians(a1), "h2": math.radians(a2), "h3": math.radians(a3)})


def cell_normal(m):
    return m[:3, :3] @ np.array([0.0, 0.0, 1.0])


def test_deployed_wing_lies_flat_on_plate_and_rail():
    p = poses(0.0, 0.0, 0.0)
    for i, n in enumerate(T.PANELS):
        c = p[n][:3, 3]
        assert c[0] == pytest.approx(T.X0 + T.L * (i + 0.5))
        assert c[2] == pytest.approx(T.TOP + T.ZB + T.PANEL_T / 2)
        assert cell_normal(p[n])[2] == pytest.approx(1.0)
    rail = T.RAIL.link_poses()["bar"][:3, 3]
    assert rail[2] + T.ZB / 2 == pytest.approx(T.TOP + T.ZB)  # rail top = plate top = panels' underside
    assert rail[0] - T.RAIL_LEN / 2 > T.X0 + T.L  # the rail starts past the plate
    assert rail[0] + T.RAIL_LEN / 2 > T.X0 + 4 * T.L  # and carries P4 to its free edge


def test_zigzag_stow_is_one_square_flat_stack():
    p = poses(180.0, -180.0, 180.0)
    for i, n in enumerate(T.PANELS):
        c = p[n][:3, 3]
        assert c[0] == pytest.approx(T.X0 + T.L / 2, abs=1e-9)  # square over P1
        assert c[1] == pytest.approx(T.YC, abs=1e-9)
        assert c[2] == pytest.approx(T.TOP + T.ZB + T.PANEL_T * (i + 0.5), abs=1e-9)  # layer i, no gap, no overlap
        assert cell_normal(p[n])[2] == pytest.approx(1.0 if i % 2 == 0 else -1.0)  # cells to cells, back to back


def test_hinges_fold_only_toward_their_knuckle_face():
    j = {jt.name: jt for jt in T.WING.joints}
    for h in ("h1", "h3"):
        assert math.degrees(j[h].upper) == pytest.approx(180.0)
        assert math.degrees(j[h].lower) == pytest.approx(-T.STOP_DEG)
    assert math.degrees(j["h2"].lower) == pytest.approx(-180.0)
    assert math.degrees(j["h2"].upper) == pytest.approx(T.STOP_DEG)
    # same-way at h2: P3 would have to reach +180 (over P2's cell face); its stop is at +12
    assert math.radians(180.0) > j["h2"].upper


def test_turning_p2_over_brings_p3_back_to_back():
    # with P2 upright and P3 folded flat on its back, P3's far end (h3) sits just outside h1 at P1's top: the
    # dragged tail closes onto P2 without crossing P1
    p = poses(90.0, -180.0, 90.0)
    h1 = np.array([T.X0 + T.L, T.YC, T.TOP + T.ZB + T.PANEL_T])
    p3_far = p["p3"][:3, :3] @ np.array([T.L / 2, 0.0, 0.0]) + p["p3"][:3, 3]
    assert p3_far[0] > h1[0]
    assert p3_far[0] == pytest.approx(h1[0] + 1.5 * T.PANEL_T, abs=1e-9)
    assert p3_far[2] == pytest.approx(h1[2], abs=1e-9)  # level with P1's top, 4 mm above the rail
    # after the first fold P4 hangs from h3 onto the rail at less than the hinge's 12 deg stop
    assert -DROOP < T.STOP_DEG - 2.0


def test_hinge_looks_show_each_knuckle_face_and_add_no_colliders():
    joints = {jt.name: jt for jt in T.WING.joints}
    looks = T.hinge_looks()
    knuckles = [k for k in looks if k[1] == "cylinder"]
    assert [k[0] for k in knuckles] == ["p1", "p2", "p3"]
    for (link, _, _, at, _), h in zip(knuckles, T.HINGES):
        m = T.WING.link(link).design
        centre = m[:3, :3] @ np.asarray(at) + m[:3, 3]
        assert centre == pytest.approx(np.asarray(joints[h].anchor), abs=1e-12)  # on the hinge axis: turns with no offset
        assert np.sign(at[2]) == T.KNUCKLE_FACE[h]  # cell face (+z) for h1 and h3, the back for h2
    for link, kind, dims, at, rgb in looks:  # leaves and borders lie on a panel face, inside the panel's outline
        if kind == "box":
            assert abs(at[0]) + dims[0] / 2 <= T.L / 2 + 1e-12 and dims[1] <= T.W
            assert abs(at[2]) - dims[2] / 2 == pytest.approx(T.PANEL_T / 2 - 0.0001)
    # looking down on the deployed wing: steel at h1 and h3, a white border seam at h2
    top = {(k[0], round(k[3][0], 4)): k[4] for k in looks if k[3][2] > 0 and k[1] == "box"}
    seam = round(T.L / 2 - T.BORDER_W / 2, 4)
    assert top[("p2", seam)] == top[("p3", -seam)] == T.BACK_WHITE
    assert top[("p1", round(T.L / 2 - T.KNUCKLE_R - T.LEAF_W / 2, 4))] == T.STEEL
    # the physics bodies are unchanged: the plate and three box colliders on P1, two on every other panel
    assert [len(T.WING.link(n).shapes) for n in T.PANELS] == [3, 2, 2, 2]
