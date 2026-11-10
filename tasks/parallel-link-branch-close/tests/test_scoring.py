# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for parallel-link-branch-close, pure Python (no simulator). Run from the task folder:
pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); most tests feed it
synthetic samples. The geometry and measure() tests build real LinkStates from the linkage's kinematics.
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


def sample(**kw):
    """The finished state: snapped on the parallelogram, plate level and still, both grippers clear and still."""
    m = {"arm_dev": 0.01, "tilt": 0.8, "link_deg": -115.0, "ex": -0.7, "ez": -3.0, "latch": 0.0, "eye_gap": 3.1,
         "snapped": True, "plate_speed": 0.0, "plate_spin": 0.0, "clear": {"left": 0.07, "right": 0.08},
         "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    m["snapped"] = kw.get("snapped", T.is_snapped(m["ex"], m["ez"], m["latch"]))
    m["eye_gap"] = kw.get("eye_gap", math.hypot(m["ex"], m["ez"]))
    return m


def hanging(**kw):
    """At reset: plate standing on its rear end (90 deg), link hanging, eye far below the pin, hands at ready."""
    base = dict(tilt=90.0, link_deg=-90.0, ex=-63.5, ez=-99.7, clear={"left": 0.2, "right": 0.2})
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


def held(m):
    """The same sample with a gripper still on a part."""
    return dict(m, clear={"left": 0.0, "right": 0.0}, tcp_speed={"left": 0.0, "right": 0.01})


def demo_trace():
    out = secs(hanging(), 2.0)
    out += secs(held(hanging(tilt=40.0)), 2.0)  # left turning the plate
    out += secs(held(hanging(tilt=0.2, ex=-10.0, ez=-46.0)), 2.0)  # plate level, link still hanging
    out += secs(held(sample(tilt=0.2, ex=-0.3, ez=-8.0, latch=6.2, plate_speed=0.002)), 1.0)  # tongue pushed back
    out += secs(held(sample(tilt=0.2, ex=-0.1, ez=1.9)), 1.0)  # clicked, still held
    out += secs(sample(tilt=0.8, plate_speed=0.01), 0.5)  # let go: the pin settles in the eye
    out += secs(sample(), 1.5)
    return out


def score(rec):
    return T.ladder(rec)[0]


def test_demo_trace_scores_one():
    r = feed(demo_trace())
    assert T.ladder(r) == (1.0, T.ladder(r)[1]) and score(r) == 1.0


def test_brake_gate_threshold_and_it_beats_a_perfect_end():
    assert score(feed(demo_trace() + [sample(arm_dev=1.49)])) == 1.0
    r = feed(demo_trace()[:20] + [hanging(arm_dev=-1.5)] + demo_trace()[20:])
    assert score(r) == 0.0 and "gate" in T.ladder(r)[1]


def test_crossed_closure_scores_zero():
    r = feed(secs(hanging(), 1.0) + secs(held(hanging(ex=-5.0, ez=-12.0)), 1.0)
             + secs(sample(tilt=90.0, link_deg=-155.0, ex=-0.7, ez=-3.0), 3.0))
    s, why = T.ladder(r)
    assert s == 0.0 and "crossed" in why and r.H.evers.get("snapped_any") is not None


@pytest.mark.parametrize("tilt,expect", [(3.0, 1.0), (-3.0, 1.0), (3.01, 0.0), (-3.01, 0.0), (89.0, 0.0)])
def test_level_tolerance_at_the_end(tilt, expect):
    r = feed(secs(hanging(), 1.0) + secs(sample(tilt=tilt), 2.5))
    assert score(r) == expect


@pytest.mark.parametrize("ex,ez,latch,ok", [
    (T.EX_RANGE[0], 0.0, 0.0, True), (T.EX_RANGE[0] - 0.01, 0.0, 0.0, False),
    (T.EX_RANGE[1], 0.0, 0.0, True), (T.EX_RANGE[1] + 0.01, 0.0, 0.0, False),
    (0.0, 3.3, 0.0, True), (0.0, -3.31, 0.0, False),
    (0.0, 0.0, 1.0, True), (0.0, 0.0, 1.01, False)])
def test_snap_window(ex, ez, latch, ok):
    assert T.is_snapped(ex, ez, latch) is ok


def test_aligned_rung_needs_level_and_10_mm():
    near = held(hanging(tilt=0.5, ex=-6.0, ez=-8.0))  # 10.0 mm
    assert score(feed(secs(hanging(), 1.0) + secs(near, 1.0) + secs(hanging(), 1.0))) == 0.25
    far = held(hanging(tilt=0.5, ex=-6.0, ez=-8.01))
    assert score(feed(secs(hanging(), 1.0) + secs(far, 1.0) + secs(hanging(), 1.0))) == 0.0
    tilted = held(hanging(tilt=3.5, ex=-1.0, ez=-2.0))
    assert score(feed(secs(hanging(), 1.0) + secs(tilted, 1.0) + secs(hanging(), 1.0))) == 0.0


def test_snapped_level_then_lost_scores_half():
    snapped = held(sample(tilt=0.3))
    r = feed(secs(hanging(), 1.0) + secs(snapped, 1.0) + secs(hanging(), 2.0))  # unclipped again
    assert score(r) == 0.5


def test_still_held_or_too_close_scores_three_quarters():
    assert score(feed(secs(hanging(), 1.0) + secs(held(sample()), 3.0))) == 0.75
    near = sample(clear={"left": 0.0299, "right": 0.08})
    assert score(feed(secs(hanging(), 1.0) + secs(near, 3.0))) == 0.75
    moving = sample(tcp_speed={"left": 0.006, "right": 0.0})
    assert score(feed(secs(hanging(), 1.0) + secs(moving, 3.0))) == 0.75


def test_release_and_stillness_need_a_full_second():
    r = feed(secs(hanging(), 1.0) + secs(held(sample()), 1.0) + secs(sample(), 0.95))
    assert score(r) == 0.75
    r.update(sample(), r.H.t + DT)
    assert score(r) == 1.0
    r = feed(secs(hanging(), 1.0) + secs(sample(), 3.0) + [sample(plate_spin=math.radians(3.0))] + secs(sample(), 0.9))
    assert score(r) == 0.75  # one 3 deg/s step restarts the stillness timer


def test_record_clears_on_reset():
    r = feed(secs(hanging(arm_dev=2.0), 1.0))
    assert score(r) == 0.0 and r.H.evers
    r.reset()
    assert r.last is None and not r.H.evers and score(r) == 0.0
    feed(demo_trace(), r)
    assert score(r) == 1.0


# -- geometry and measure() on real link states --------------------------------------------------------------------


def state(q):
    poses = T.PAL.link_poses(q)
    z = {n: np.zeros(3) for n in poses}
    return lk.LinkState(pos=poses["base"][:3, 3], quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                        ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), spec=T.PAL, poses=poses, lin_vels=z,
                        ang_vels=dict(z), q=dict(q), qd={k: 0.0 for k in q})


FAR = {"left": lk.Tool("left", np.array([0.0, 0.3, 1.1]), np.diag([1.0, -1.0, -1.0]), 0.09, 0.0),
       "right": lk.Tool("right", np.array([0.0, -0.3, 1.1]), np.diag([1.0, -1.0, -1.0]), 0.09, 0.0)}


def test_geometry_closes_both_branches():
    assert np.linalg.norm(T.D_X - T.B) == pytest.approx(T.L, abs=1e-9)  # the hanging lug is on the crossed closure
    assert np.linalg.norm(T.D_P - T.B) == pytest.approx(T.L, abs=1e-9)
    assert T.CROSS_LINK_DEG == pytest.approx(-155.0, abs=0.1) and T.LINK_DEG == -115.0


def test_measure_at_reset_parallel_and_crossed():
    m = T.measure(state({}), FAR)  # init joints: hanging
    assert m["tilt"] == pytest.approx(90.0, abs=1e-6) and m["link_deg"] == pytest.approx(-90.0, abs=1e-6)
    assert not m["snapped"] and m["eye_gap"] > 100.0 and m["arm_dev"] == pytest.approx(0.0, abs=1e-9)
    m = T.measure(state({"wrist": 0.0, "lug": 0.0, "latch": 0.0}), FAR)  # design: the parallelogram
    assert m["snapped"] and m["tilt"] == pytest.approx(0.0, abs=1e-9) and m["eye_gap"] == pytest.approx(0.0, abs=1e-6)
    crossed = {"wrist": math.radians(-90.0), "lug": math.radians(130.0), "latch": 0.0}
    m = T.measure(state(crossed), FAR)
    assert m["snapped"] and m["tilt"] == pytest.approx(90.0, abs=1e-6) and m["link_deg"] == pytest.approx(-155.0, abs=0.01)
    m = T.measure(state({"wrist": 0.0, "lug": 0.0, "latch": 0.005}), FAR)  # tongue held back 5 mm
    assert not m["snapped"] and m["latch"] == pytest.approx(5.0, abs=1e-6)
    m = T.measure(state({"wrist": math.radians(2.9), "lug": math.radians(-2.9), "latch": 0.0}), FAR)
    assert m["tilt"] == pytest.approx(-2.9, abs=1e-6) and T.level(m)


def test_clearance_sees_a_hand_on_the_plate():
    poses = T.PAL.link_poses({"wrist": 0.0, "lug": 0.0})
    on = poses["plate"][:3, :3] @ np.array([0.04, 0.03, 0.0]) + poses["plate"][:3, 3]
    hand = lk.Tool("left", on, np.diag([1.0, -1.0, -1.0]), 0.02, 0.0)
    m = T.measure(state({"wrist": 0.0, "lug": 0.0, "latch": 0.0}), {"left": hand, "right": FAR["right"]})
    assert m["clear"]["left"] < 0.03 and m["clear"]["right"] > 0.1


def test_eye_sweeps_onto_the_pin_without_touching_its_walls():
    """Along both swings the link's own walls stay clear of the pin; only the tongue must give way (at most 6.4 mm of
    its 10 mm travel), and it is home again before the pin reaches the pocket centre."""
    def gap(q, link):
        P = T.PAL.link_poses(q)
        worst = 1.0
        for sh in T.PAL.link(link).shapes:
            p = P[link][:3, :3].T @ (T.B_WORLD - P[link][:3, 3])
            for dy in (0.0215, 0.025, 0.0285):
                worst = min(worst, sh.distance(np.array([p[0], dy, p[2]])) - T.PIN_R)
        return worst

    for wrist, lug_end in ((0.0, 0.0), (-90.0, 130.0)):
        need_max = 0.0
        for d in np.arange(-8.0, 1.51, 0.25):
            q = {"wrist": math.radians(wrist), "lug": math.radians(lug_end + d), "latch": 0.0}
            assert gap(q, "link") > 0.1e-3
            need = next(r for r in np.arange(0.0, 0.0101, 0.0002) if gap(dict(q, latch=r), "tongue") >= 0.0)
            need_max = max(need_max, need)
            if d >= -1.25:
                assert need == 0.0
        assert 0.005 < need_max < 0.0075


def test_pose_rates():
    A = np.eye(4)
    B = np.eye(4)
    B[:3, 3] = (0.0001, 0.0, 0.0)
    v, w = T.pose_rates(A, B, DT)
    assert v == pytest.approx(0.002) and w == pytest.approx(0.0)
    B[:3, :3] = lk.axis_rotation((0, 1, 0), math.radians(0.2))
    assert T.pose_rates(A, B, DT)[1] == pytest.approx(math.radians(4.0))
