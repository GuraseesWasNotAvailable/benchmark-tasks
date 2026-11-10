# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer and geometry edge cases for five-bar-leg-foot-pin, pure Python (no simulator). Run from the task folder:
pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); most tests feed it
synthetic samples. The geometry tests use the leg's own kinematics (Linkage.link_poses), the same poses the
simulator reports.
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
    """The finished state: stud home in the jaw, tongue home, both grippers clear and still."""
    m = {"a": -0.3, "b": 0.2, "latch": 0.05, "hold_l": [], "hold_r": [],
         "clear": {"left": 0.08, "right": 0.09}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    m["dist"] = kw.get("dist", math.hypot(m["a"], m["b"]))
    m["snapped"] = kw.get("snapped", T.is_snapped(m["a"], m["b"], m["latch"]))
    return m


def hanging(**kw):
    """At reset: both chains hanging, the stud 100 mm from the jaw, hands at ready."""
    base = dict(a=70.0, b=-70.0, latch=0.0, clear={"left": 0.2, "right": 0.2})
    base.update(kw)
    return sample(**base)


def held(m, left=("left",), right=("right",)):
    """The same sample with the grippers on the shins (the left gripper on the left shin, the right on the right)."""
    return dict(m, hold_l=sorted(left), hold_r=sorted(right), clear={"left": 0.0, "right": 0.0},
                tcp_speed={"left": 0.0, "right": 0.01})


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
    out = secs(hanging(), 2.0)
    out += secs(held(hanging()), 2.0)  # both shins gripped
    out += secs(held(hanging(a=20.0, b=0.4)), 2.0)  # bent into the diamond, stud 20 mm in front of the mouth
    out += secs(held(sample(a=6.0, b=0.2, latch=6.5)), 1.0)  # pushing the tongue back
    out += secs(held(sample(a=-0.4, b=0.1, latch=0.0)), 1.0)  # clicked, still held
    out += secs(sample(a=0.3, b=-0.4), 0.5)  # let go: the closed leg swings down to its V
    out += secs(sample(), 1.5)
    return out


def score(rec):
    return T.ladder(rec)[0]


def test_demo_trace_scores_one():
    r = feed(demo_trace())
    s, why = T.ladder(r)
    assert s == 1.0 and "let go" in why


@pytest.mark.parametrize("a,b,latch,ok", [
    (1.5, 0.0, 0.0, True), (1.51, 0.0, 0.0, False), (0.0, -1.5, 0.0, True), (0.0, -1.51, 0.0, False),
    (0.9, 1.2, 0.0, True), (0.9, 1.21, 0.0, False), (0.0, 0.0, 1.0, True), (0.0, 0.0, 1.01, False)])
def test_snap_window(a, b, latch, ok):
    assert T.is_snapped(a, b, latch) is ok


def test_pushed_in_but_tongue_still_back_is_not_snapped():
    r = feed(secs(hanging(), 1.0) + secs(sample(a=0.0, b=0.0, latch=3.0), 3.0))
    assert score(r) == 0.0 and "snapped_any" not in r.H.evers


@pytest.mark.parametrize("dist,expect", [(10.0, 0.25), (10.01, 0.0)])
def test_together_rung_needs_10_mm(dist, expect):
    near = held(hanging(a=dist, b=0.0))
    r = feed(secs(hanging(), 1.0) + secs(near, 1.0) + secs(hanging(), 1.0))
    assert score(r) == expect


def test_together_rung_needs_each_shin_held_by_its_own_gripper():
    one_shin = dict(held(hanging(a=5.0, b=0.0)), hold_r=[])  # only the left shin held
    assert score(feed(secs(hanging(), 1.0) + secs(one_shin, 2.0))) == 0.0
    same = held(hanging(a=5.0, b=0.0), left=("left",), right=("left",))  # one gripper on both (cannot hold)
    assert score(feed(secs(hanging(), 1.0) + secs(same, 2.0))) == 0.0
    crossed = held(hanging(a=5.0, b=0.0), left=("right",), right=("left",))  # arms crossed over: still both held
    assert score(feed(secs(hanging(), 1.0) + secs(crossed, 2.0))) == 0.25


def test_one_arm_snap_still_scores():
    """Physics, not the scorer, stops a one-arm plan: a snap made any way counts."""
    snap_one_arm = dict(held(sample()), hold_r=[], clear={"left": 0.0, "right": 0.2})
    r = feed(secs(hanging(), 1.0) + secs(snap_one_arm, 1.0) + secs(sample(), 2.0))
    assert score(r) == 1.0


def test_clicked_then_lost_scores_half():
    r = feed(secs(hanging(), 1.0) + secs(held(sample()), 1.0) + secs(hanging(a=30.0, b=5.0), 2.0))
    s, why = T.ladder(r)
    assert s == 0.5 and "out at the end" in why


def test_still_held_at_the_end_scores_three_quarters():
    r = feed(demo_trace()[:-40] + secs(held(sample()), 2.0))
    assert score(r) == 0.75


@pytest.mark.parametrize("clear,expect", [(0.030, 1.0), (0.0299, 0.75)])
def test_release_clearance_30_mm(clear, expect):
    r = feed(demo_trace() + secs(sample(clear={"left": clear, "right": 0.09}), 1.5))
    assert score(r) == expect


@pytest.mark.parametrize("speed,expect", [(0.0049, 1.0), (0.006, 0.75)])
def test_release_hands_still(speed, expect):
    r = feed(demo_trace() + secs(sample(tcp_speed={"left": 0.0, "right": speed}), 1.5))
    assert score(r) == expect


def test_release_and_snap_must_last_one_second():
    base = demo_trace()[:-40] + secs(held(sample()), 1.0)
    assert score(feed(base + secs(sample(), 0.95))) == 0.75
    assert score(feed(base + secs(sample(), 1.0))) == 1.0
    # a snap that only just came back (stud out and in again) needs its own full second
    r = feed(demo_trace() + [sample(a=3.0, b=0.0)] + secs(sample(), 0.9))
    assert score(r) == 0.75


def test_record_is_whole_run_and_clears_on_reset():
    r = feed(secs(hanging(), 1.0) + secs(held(sample()), 1.0) + secs(hanging(), 2.0))
    assert "snapped_any" in r.H.evers
    r.reset()
    assert r.last is None and not r.H.evers and score(r) == 0.0
    feed(secs(hanging(), 1.0), rec=r)
    assert score(r) == 0.0


# -- geometry: the leg's own kinematics ----------------------------------------------------------------------------


def poses(**q):
    return T.LEG.link_poses({k: v for k, v in q.items()})


def seated_q(foot=(0.0, -T.F_CLOSE)):
    hr, kr = T.chain_angles(foot, -1)
    hl, kl = T.chain_angles(foot, +1)
    return dict(hip_l=hl, knee_l=kl, hip_r=hr, knee_r=kr)


def test_hanging_feet_100_mm_apart_and_reading():
    P = poses()
    a, b, latch = T.jaw_reading(P)
    assert math.hypot(a, b) == pytest.approx(2000 * T.HIP_Y, abs=0.01) and latch == pytest.approx(0.0, abs=1e-9)
    stud = T.stud_point(P)
    assert stud[2] == pytest.approx(T.TOP + T.HIP_H - 0.185, abs=1e-9)


def test_chain_angles_put_the_foot_where_asked_knee_out():
    for side, hip, knee, link in ((1, "hip_l", "knee_l", "shin_l"), (-1, "hip_r", "knee_r", "shin_r")):
        foot = (0.012 * side, -0.155)
        h, k = T.chain_angles(foot, side)
        P = T.LEG.link_poses({hip: h, knee: k})
        p = P[link][:3, :3] @ np.array([0.0, 0.0, -T.SHIN]) + P[link][:3, 3] - np.asarray(T.ORIGIN)
        assert p[1] == pytest.approx(foot[0], abs=1e-9) and p[2] == pytest.approx(foot[1], abs=1e-9)
        kpt = P[link][:3, 3] - np.asarray(T.ORIGIN)
        assert side * kpt[1] > T.HIP_Y  # the knee sits outside its hip bearing (the diamond)


def test_demo_closing_pose_mouth_faces_the_left_chain():
    P = poses(**seated_q())
    M = T.jaw_frame(P)
    assert M[:3, 0] == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)  # mouth along +y
    a, b, latch = T.jaw_reading(P)
    assert math.hypot(a, b) < 1e-6 and T.is_snapped(a, b, latch)


def test_latch_reading_follows_the_tongue():
    q = seated_q()
    for travel in (0.0, 0.0035, 0.0065):
        a, b, latch = T.jaw_reading(T.LEG.link_poses(dict(q, latch=travel)))
        assert latch == pytest.approx(1000 * travel, abs=1e-6)
    assert not T.is_snapped(0.0, 0.0, 6.5)


def test_demo_view_reads_only_the_shins_never_the_tongue():
    # the scripted demo sees the stud's depth from the two shin poses; asking it for the tongue (or anything else)
    # fails, and with the tongue anywhere on its travel the demo's reading is the same as the scorer's stud reading
    q = seated_q()

    class Seen:
        def __init__(self, P):
            self.P = P

        def pose(self, obj, link):
            assert obj == "leg" and link in ("shin_l", "shin_r"), link
            return self.P[link]

    for travel in (0.0, 0.0065):
        P = T.LEG.link_poses(dict(q, latch=travel))
        _, _, (a, b) = T._seen_jaw(Seen(P))
        a2, b2, _ = T.jaw_reading(P)
        assert (a, b) == pytest.approx((a2, b2), abs=1e-12) and math.hypot(a, b) <= T.SNAP_MM


def test_tongue_must_travel_6_5_mm_to_pass_the_stud_and_clicks_within_half_a_mm_of_the_bottom():
    # the strip's tip sits 2.5 mm past the pocket centre, so a 4 mm stud needs 6.5 mm of travel (stop at 10 mm)
    assert T.RS - T.TIP == pytest.approx(0.0065) and T.LATCH_TRAVEL > T.RS - T.TIP
    # the guide rail reaches past the lead-in's first contact (stud centre 14.2 mm out), so the rail takes the push
    first = T.PK + T.T_THK - T.TIP + T.RS * math.sqrt(2)
    assert first == pytest.approx(0.01416, abs=1e-4) and T.RAIL > first
    # it can drop back only once the stud's outer side is inside the strip's face: centre at most 0.5 mm out
    assert T.PK - T.RS == pytest.approx(0.0005)


def test_physics_one_hanging_chain_cannot_hold_against_the_click():
    g = 9.81
    for thigh, shin in (("thigh_l", ("shin_l",)), ("thigh_r", ("shin_r", "tongue"))):
        thigh_m = T.LEG.link(thigh).mass
        shin_m = sum(T.LEG.link(s).mass for s in shin)
        torque = g * (thigh_m * T.THIGH / 2 + shin_m * (T.THIGH + T.SHIN / 2))  # weight torque, chain horizontal
        resist = torque / (T.THIGH + T.SHIN)  # the most a free chain pushes back at its foot
        preload = T.LATCH_K * abs(T.LATCH_REST)  # tongue spring force before it moves
        assert torque < 0.075 and resist < 0.4 and preload > 2.5 * resist


def test_physics_neither_chain_reaches_the_other_hanging_foot_and_one_gripper_cannot_span_both():
    reach = T.THIGH + T.SHIN
    far = math.hypot(4 * T.HIP_Y, reach)  # left hip to the right chain's hanging foot
    assert far - reach > 0.020
    outer = 2 * T.HIP_Y + 0.012  # outer faces of the two hanging shins
    assert outer > 0.094  # the gripper opens 94 mm


def test_layers_only_the_stud_crosses_between_chains():
    # left links sit in front (x < 0) of the right links; the stud alone reaches back through the jaw layer
    xl = [T.X_TL, T.X_SL]
    xr = [T.X_SR, T.X_TR]
    assert max(xl) + T.THICK / 2 < min(xr) - T.THICK / 2
    stud = [s for s in T.LEG.link("shin_l").shapes if s.color == T.ORANGE]
    assert len(stud) == 2
