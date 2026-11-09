# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for guedel-airway-insert, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); most tests feed it
synthetic samples. The measure() tests build real LinkStates from the head's kinematics at chosen joint angles.
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
    """A sample of the finished state: airway seated the right way round, tongue lifted, jaw shut, hands clear."""
    m = {
        "q_t": -25.0, "q_j": 8.0, "gap": 10.8, "T_u": 8.0, "F": (-8.0, 0.0, 0.5), "tilt": 2.0, "alpha": 0.0,
        "tip": (20.0, 0.0, -57.0), "held": False, "aw_speed": 0.0, "aw_spin": 0.0,
        "clear": {"left": 0.08, "right": 0.06}, "tcp_speed": {"left": 0.0, "right": 0.0},
    }
    m.update(kw)
    return m


def inserting(**kw):
    """A sample mid-insertion: held upside down in the open mouth, tongue untouched."""
    base = dict(q_t=0.0, q_j=28.0, gap=2.0, T_u=4.0, F=(-7.5, 0.0, 2.5), alpha=180.0, tip=(-35.5, 0.0, -55.5),
                held=True, clear={"left": 0.0, "right": 0.0}, tcp_speed={"left": 0.0, "right": 0.01})
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
    """Rest, insertion upside down, the turn lifting the tongue, the jaw eased shut, hands withdrawn, 3.5 s still."""
    out = secs(sample(q_t=0.0, q_j=0.0, gap=2.0, T_u=4.0, F=(-150.0, -170.0, -50.0), alpha=0.0,
                      tip=(-122.0, -170.0, -108.0), clear={"left": 0.2, "right": 0.2},
                      tcp_speed={"left": 0.1, "right": 0.1}), 2.0)
    out += secs(inserting(), 3.0)
    for k in range(20):  # the turn: alpha 180 -> 0, tongue lifted to -25 in the second half
        a = 180.0 - 9.0 * (k + 1)
        q = 0.0 if a > 90 else -25.0 * (90 - a) / 90
        out.append(inserting(alpha=a, q_t=q, gap=2.0 - 0.35 * q, tip=(-7.5 + 28 * math.cos(math.radians(a)), 0, -55.5)))
    out += secs(inserting(alpha=0.0, q_t=-25.0, gap=10.8, tip=(20.5, 0.0, -55.5), F=(-7.5, 0.0, 0.5)), 1.0)
    out += secs(sample(held=True, clear={"left": 0.0, "right": 0.0}, tcp_speed={"left": 0.01, "right": 0.0}), 2.0)
    out += secs(sample(), 3.5)
    return out


def score(rec):
    return T.ladder(rec)[0]


def test_demo_trace_scores_one():
    r = feed(demo_trace())
    assert score(r) == 1.0, T.ladder(r)


def test_no_steps_scores_zero():
    assert score(T.Record()) == 0.0


def test_doing_nothing_scores_zero():
    r = feed(secs(sample(q_t=0.0, q_j=0.0, gap=2.0, F=(-150.0, -170.0, -50.0), tip=(-122.0, -170.0, -108.0)), 10))
    assert score(r) == 0.0


def test_push_back_gate_at_three_degrees_not_below():
    r = feed(demo_trace()[:60] + [inserting(q_t=2.99)] + demo_trace()[60:])
    assert score(r) == 1.0
    r = feed(demo_trace()[:60] + [inserting(q_t=3.00)] + demo_trace()[60:])
    assert score(r) == 0.0, "a push-back is never cleared by lifting the tongue and seating later"
    assert "gate" in T.ladder(r)[1]


def test_gate_beats_an_open_airway_at_the_end():
    r = feed([inserting(q_t=6.7)] + demo_trace())
    assert score(r) == 0.0


def test_alpha_25_open_26_not():
    assert T.open_airway(sample(alpha=25.0)) and T.open_airway(sample(alpha=-25.0))
    assert not T.open_airway(sample(alpha=26.0)) and not T.open_airway(sample(alpha=-26.0))


def test_gap_8_0_open_7_9_not():
    assert T.open_airway(sample(gap=8.0))
    assert not T.open_airway(sample(gap=7.9))


def test_seated_limits():
    assert T.seated(sample(F=(-8.0, 0.0, 4.0)))
    assert not T.seated(sample(F=(-8.0, 0.0, 4.1)))
    assert not T.seated(sample(F=(-17.1, 0.0, 0.5))) and T.seated(sample(F=(-17.0, 0.0, 0.5)))
    assert not T.seated(sample(F=(-1.0 + 0.1, 0.0, 0.5)))
    assert not T.seated(sample(F=(-8.0, 8.1, 0.5)))
    assert T.seated(sample(tilt=15.0)) and not T.seated(sample(tilt=15.1))


def test_tongue_must_be_lifted_15_and_tip_behind_it():
    assert T.open_airway(sample(q_t=-15.0)) and not T.open_airway(sample(q_t=-14.9))
    assert T.open_airway(sample(tip=(13.0, 0.0, -57.0), T_u=8.0))
    assert not T.open_airway(sample(tip=(12.9, 0.0, -57.0), T_u=8.0))


def test_jaw_held_open_scores_075():
    tr = demo_trace()
    tr[-90:] = [dict(m, q_j=28.0) for m in tr[-90:]]
    r = feed(tr)
    assert score(r) == 0.75 and "jaw" in T.ladder(r)[1]
    tr = demo_trace()
    tr[-90:] = [dict(m, q_j=12.0) for m in tr[-90:]]
    assert score(feed(tr)) == 1.0


def test_release_timer_restarts_when_a_pad_comes_back():
    tr = demo_trace()
    tr[-15] = dict(tr[-15], clear={"left": 0.08, "right": 0.029})  # one step 29 mm from the head: < 1 s since
    assert score(feed(tr)) == 0.75
    tr[-15] = dict(tr[-15], clear={"left": 0.08, "right": 0.030})
    assert score(feed(tr)) == 1.0


def test_moving_hand_or_airway_at_the_end_scores_075():
    tr = demo_trace()
    tr[-10] = dict(tr[-10], tcp_speed={"left": 0.0, "right": 0.006})
    assert score(feed(tr)) == 0.75
    tr = demo_trace()
    tr[-30] = dict(tr[-30], aw_speed=0.004)  # airway still for only 1.5 s
    assert score(feed(tr)) == 0.75


def test_released_needs_a_full_second():
    tr = demo_trace()[:-70] + secs(sample(), 0.95)
    r = feed(tr)
    assert r.H.runs["released"] < 1.0 and score(r) == 0.75
    r = feed(demo_trace()[:-70] + secs(sample(), 2.0))
    assert score(r) == 1.0


def test_entered_needs_a_held_airway_deep_in_the_open_mouth():
    assert score(feed(secs(inserting(), 1.0))) == 0.25
    assert score(feed(secs(inserting(held=False), 1.0))) == 0.0, "a dropped airway earns nothing"
    assert score(feed(secs(inserting(q_j=9.9), 1.0))) == 0.0, "the jaw must be open"
    assert score(feed(secs(inserting(alpha=90.0, tip=(-7.5, 28.0, -56.5)), 1.0))) == 0.25, "any technique counts"
    assert score(feed(secs(inserting(alpha=0.0, tip=(20.5, 0.0, -36.0)), 1.0))) == 0.25, "the right way up too"
    assert score(feed(secs(inserting(tip=(-35.5, 0.0, -34.0)), 1.0))) == 0.0, "the tip must be 35 mm deep"
    assert score(feed(secs(inserting(alpha=0.0, tip=(20.5, 0.0, -50.0)), 1.0) + [inserting(alpha=0.0, q_t=3.0)]))         == 0.0, "entering the right way up and pushing the tongue back is gated"


def test_lifted_scores_half_and_needs_the_airway_in_the_mouth():
    r = feed(secs(inserting(), 1.0) + [inserting(alpha=0.0, q_t=-10.0, tip=(20.0, 0.0, -55.0))]
             + secs(sample(F=(-150.0, -170.0, -50.0), tip=(-122.0, -170.0, -108.0), q_t=0.0, gap=2.0), 2.0))
    assert score(r) == 0.5
    # the tongue lifted by something else while the airway lies on the table
    r = feed(secs(sample(q_t=-20.0, gap=9.0, F=(-150.0, -170.0, -50.0), tip=(-122.0, -170.0, -108.0)), 2.0))
    assert score(r) == 0.0


def test_record_resets_clean():
    r = feed([inserting(q_t=5.0)] + demo_trace())
    assert score(r) == 0.0
    r.reset()
    assert score(r) == 0.0 and not r.H.evers
    feed(demo_trace(), r)
    assert score(r) == 1.0


# -- measure() on real link states ------------------------------------------------------------------------------


def head_state(q_jaw=0.0, q_t=0.0):
    poses = T.HEAD.link_poses({"tmj": math.radians(q_jaw), "tongue_base": math.radians(q_t)})
    z = {n: np.zeros(3) for n in poses}
    return lk.LinkState(pos=poses["skull"][:3, 3], quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                        ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), spec=T.HEAD, poses=poses,
                        lin_vels=z, ang_vels=dict(z), q={"tmj": math.radians(q_jaw), "tongue_base": math.radians(q_t)},
                        qd={"tmj": 0.0, "tongue_base": 0.0})


def airway_state(A):
    return lk.LinkState(pos=A[:3, 3], quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                        low=np.zeros(3), high=np.zeros(3), spec=T.AIRWAY, poses={"airway": A},
                        lin_vels={"airway": np.zeros(3)}, ang_vels={"airway": np.zeros(3)}, q={}, qd={})


def tool(side, pos, opening=0.07):
    return lk.Tool(side, np.asarray(pos, dtype=float), np.diag([1.0, -1.0, -1.0]), opening, 0.0)


def test_measure_at_rest_and_seated():
    far = {"left": tool("left", (0.0, 0.3, 1.1)), "right": tool("right", (0.0, -0.3, 1.1))}
    m = T.measure(head_state(), airway_state(T.airway_pose((-8.0, 0.0, 0.5), 0.0)), far)
    assert m["gap"] == pytest.approx(2.0, abs=1e-6) and m["q_t"] == pytest.approx(0.0)
    assert m["F"] == pytest.approx((-8.0, 0.0, 0.5), abs=1e-6) and m["alpha"] == pytest.approx(0.0, abs=1e-6)
    assert m["tip"] == pytest.approx((20.0, 0.0, -58.5), abs=1e-6) and m["tilt"] == pytest.approx(0.0, abs=1e-6)
    assert T.seated(m) and not T.open_airway(m), "tongue at rest: not open"
    assert min(m["clear"].values()) > 0.1 and not m["held"]
    m = T.measure(head_state(q_jaw=8.0, q_t=-25.0), airway_state(T.airway_pose((-8.0, 0.0, 0.5), 0.0)), far)
    assert m["gap"] == pytest.approx(10.7, abs=0.2) and T.open_airway(m)


def test_seated_airway_tip_rests_on_the_throat_wall():
    """Flange bottom on the lips (w 0): the curve's lowest point touches the throat wall (w -64), so a released airway
    cannot pitch its tip down and back under the tongue's push."""
    A = T.airway_pose((-9.0, 0.0, 0.0), 0.0)
    W = (A[:3, :3] @ np.array([s.pos for s in T.AIRWAY_SHAPES[3:]]).T).T + A[:3, 3]
    low = min(T.to_head(w)[2] for w in W) - 5.0  # capsule radius 5 mm
    assert low == pytest.approx(-64.0, abs=0.3)


def test_measure_wall_stop_and_inverted_airway():
    far = {"left": tool("left", (0.0, 0.3, 1.1)), "right": tool("right", (0.0, -0.3, 1.1))}
    m = T.measure(head_state(q_t=6.7), airway_state(T.airway_pose((-7.5, 0.0, 2.5), 180.0)), far)
    assert m["gap"] == pytest.approx(0.0, abs=0.1), "the +6.7 deg stop is the throat wall"
    assert abs(m["alpha"]) == pytest.approx(180.0, abs=1e-6) and m["tip"][0] == pytest.approx(-35.5, abs=1e-6)


def test_held_needs_a_closed_gripper_at_the_airway():
    A = T.airway_pose((-7.5, 0.0, 2.5), 180.0)
    G = A @ T.grasp_nominal()
    near = lk.Tool("right", G[:3, 3], G[:3, :3], 0.056, 0.0)
    far = tool("left", (0.0, 0.3, 1.1))
    m = T.measure(head_state(q_jaw=28.0), airway_state(A), {"left": far, "right": near})
    assert m["held"]
    wide = lk.Tool("right", G[:3, 3], G[:3, :3], 0.070, 0.0)
    assert not T.measure(head_state(q_jaw=28.0), airway_state(A), {"left": far, "right": wide})["held"]


def test_tongue_rest_compensates_its_weight():
    torque = T.TONGUE_MASS * 9.81 * (T.TONGUE_HINGE[0] - T.TONGUE_COM[0]) * 1e-3
    assert torque + T.TONGUE_K * math.radians(T.TONGUE_REST_DEG) == pytest.approx(0.0, abs=1e-9)


def test_stillness_comes_from_the_pose_change_between_steps():
    A = T.airway_pose((-8.8, 0.0, 0.2), 3.0)
    assert T.pose_rates(A, A.copy(), 0.05) == pytest.approx((0.0, 0.0), abs=1e-9), "an unchanged pose is still"
    B = T.airway_pose((-8.8, 0.0, 0.2 + 0.2), 3.0)  # 0.2 mm in one 50 ms step = 4 mm/s
    v, w = T.pose_rates(A, B, 0.05)
    assert v == pytest.approx(0.004, abs=1e-9) and v > T.AIRWAY_STILL and w == pytest.approx(0.0, abs=1e-6)
    C = T.airway_pose((-8.8, 0.0, 0.2), 3.2)  # 0.2 deg in one step = 4 deg/s
    v, w = T.pose_rates(A, C, 0.05)
    assert math.degrees(w) == pytest.approx(4.0, abs=0.01) and w > T.AIRWAY_SPIN
