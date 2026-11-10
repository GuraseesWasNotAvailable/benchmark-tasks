# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for knee-brace-break-stow, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); most tests feed it
synthetic samples. The kinematics tests pose the fixture's links from its joint angles (the strut solved in closed
form for a panel angle) and check the geometry the task relies on.
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


def sample(theta=75.0, bend=-3.0, rate=0.0, clear=(0.0, 0.0), tcp=(0.0, 0.0), lug=0.0):
    return {"theta": theta, "bend": bend, "rate": rate, "lug": lug, "clear": {"left": clear[0], "right": clear[1]},
            "tcp_speed": {"left": tcp[0], "right": tcp[1]}}


class Run:
    """Feeds samples at 20 Hz; the panel rate comes from the angle change, as in the simulator."""

    def __init__(self):
        self.rec = T.Record()
        self.t = 0.0
        self.theta = None

    def step(self, rate=None, **kw):
        m = sample(**kw)
        if rate is not None:
            m["rate"] = rate
        elif self.theta is not None:
            m["rate"] = (m["theta"] - self.theta) / DT
        self.theta = m["theta"]
        self.t += DT
        self.rec.update(m, self.t)
        return self

    def hold(self, seconds, **kw):
        for _ in range(round(seconds / DT)):
            self.step(**kw)
        return self

    def lower(self, start, end, deg_s, **kw):
        n = max(1, round(abs(start - end) / deg_s / DT))
        for k in range(1, n + 1):
            self.step(theta=start + (end - start) * k / n, **kw)
        return self

    def score(self):
        return T.ladder(self.rec)[0]


def demo_run(lower_deg_s=14.0, release_s=1.2, end_theta=0.0, clear=0.08, tcp=0.0):
    r = Run().hold(2.0)
    for b in np.linspace(-3.0, 10.0, 20):  # the knee pushed across centre while the panel is held
        r.step(theta=75.0 - 0.03 * (b + 3.0), bend=float(b))
    r.lower(74.6, end_theta, lower_deg_s, bend=60.0)
    r.hold(0.5, theta=end_theta, bend=119.7)
    r.hold(release_s, theta=end_theta, bend=119.7, clear=(clear, clear), tcp=(tcp, tcp))
    return r


def test_demo_shaped_run_scores_one():
    assert demo_run().score() == 1.0


def test_no_steps_and_locked_panel_score_zero():
    assert T.ladder(T.Record())[0] == 0.0
    assert Run().hold(10.0).score() == 0.0


def test_gate_at_60_deg_per_s():
    assert demo_run(lower_deg_s=59.0).score() == 1.0
    r = demo_run()
    r.step(theta=0.0, bend=119.7, rate=-60.0, clear=(0.08, 0.08))
    assert "slammed" not in r.rec.H.evers, "exactly 60 deg/s is allowed"
    r.step(theta=0.0, bend=119.7, rate=-60.1, clear=(0.08, 0.08))
    assert r.score() == 0.0 and "slammed" in r.rec.H.evers


def test_slam_is_never_erased():
    r = Run().hold(1.0)
    r.step(theta=75.0, bend=12.0)
    r.lower(75.0, 0.0, 700.0, bend=60.0)  # knee pushed and let go: the panel falls in about 0.1 s
    assert r.score() == 0.0
    r.lower(0.0, 75.0, 10.0, bend=5.0)  # lifted back up and lowered properly afterwards
    r.lower(75.0, 0.0, 10.0, bend=60.0)
    r.hold(2.5, theta=0.0, bend=119.7, clear=(0.08, 0.08))
    assert r.score() == 0.0 and "gate" in T.ladder(r.rec)[1]


def test_upward_speed_also_gates():
    r = Run().hold(1.0)
    r.step(theta=78.5, bend=-3.0)  # 70 deg/s up toward the over-travel stop
    assert r.score() == 0.0 and "slammed" in r.rec.H.evers


def test_unlock_rung_needs_8_deg_past_straight():
    r = Run().hold(1.0).hold(1.0, bend=7.9)
    assert r.score() == 0.0
    r.hold(0.1, bend=8.0)
    assert r.score() == 0.25
    r.hold(1.0, bend=-3.0)  # relocked afterwards: the record keeps the rung
    assert r.score() == 0.25


def test_pushing_the_panel_scores_zero():
    r = Run().hold(1.0)
    r.hold(6.0, theta=74.9, bend=-3.0)  # pressed into its stop: the panel does not move
    r.hold(2.0, theta=75.0, clear=(0.08, 0.08))
    assert r.score() == 0.0


def test_lowered_rung_at_40_deg():
    r = Run().hold(1.0).step(theta=74.8, bend=12.0)
    r.lower(74.8, 40.1, 10.0, bend=60.0)
    assert r.score() == 0.25
    r.step(theta=40.0, bend=84.0)
    assert r.score() == 0.5
    r.lower(40.0, 60.0, 10.0, bend=50.0)  # lifted again: lowered once is kept, not on the pads
    assert r.score() == 0.5


def test_on_pads_tolerance_2_deg():
    assert demo_run(end_theta=2.0).score() == 1.0
    assert demo_run(end_theta=2.1).score() == 0.5


def test_release_gate():
    assert demo_run(clear=0.030).score() == 1.0
    assert demo_run(clear=0.0299).score() == 0.75
    assert demo_run(tcp=0.006).score() == 0.75
    assert demo_run(release_s=0.95).score() == 0.75
    r = demo_run()
    r.step(theta=0.0, bend=119.7, clear=(0.08, 0.029))  # one step too close restarts the timer
    r.hold(0.9, theta=0.0, bend=119.7, clear=(0.08, 0.08))
    assert r.score() == 0.75
    r.hold(0.15, theta=0.0, bend=119.7, clear=(0.08, 0.08))
    assert r.score() == 1.0


def test_panel_must_be_still_on_the_pads():
    r = demo_run()
    r.step(theta=0.0, bend=119.7, rate=3.0, clear=(0.08, 0.08))
    r.hold(0.9, theta=0.0, bend=119.7, clear=(0.08, 0.08))
    assert r.score() == 0.75, "a 3 deg/s step restarts the 1 s stillness timer"
    r.hold(0.15, theta=0.0, bend=119.7, clear=(0.08, 0.08))
    assert r.score() == 1.0


def test_strut_coming_apart_gates():
    r = demo_run()
    r.step(theta=0.0, bend=119.7, lug=0.002, clear=(0.08, 0.08))
    assert r.score() == 1.0, "a 2.0 mm loop gap is tolerated"
    r = Run().hold(1.0)
    r.hold(3.0, lug=0.0021)  # the panel pressed so hard that the simulated strut stretched open
    r.step(theta=74.0, bend=12.0)  # whatever follows, even a proper stow
    r.lower(74.0, 0.0, 14.0, bend=60.0)
    r.hold(2.5, theta=0.0, bend=119.7, clear=(0.08, 0.08))
    assert r.score() == 0.0 and "came apart" in T.ladder(r.rec)[1]


def test_reset_clears_the_record():
    r = Run().hold(1.0).step(theta=75.0, bend=12.0)
    r.lower(75.0, 0.0, 700.0)
    assert "slammed" in r.rec.H.evers
    r.rec.reset()
    assert r.rec.last is None and not r.rec.H.evers and T.ladder(r.rec)[0] == 0.0


# -- kinematics --------------------------------------------------------------------------------------------------


def _angle_y(a, b):
    """Signed rotation about +y taking x-z vector a onto b (rad)."""
    return math.atan2(a[1] * b[0] - a[0] * b[1], a[0] * b[0] + a[1] * b[1])


def strut_q(theta):
    """Joint angles with the panel at ``theta`` and the strut folded on its unlocking side (closed form)."""
    B = T._xz(T.B_PT)
    P = T.B_P * T._xz(T.u_dir(theta))
    v = P - B
    L = float(np.linalg.norm(v))
    e = v / L
    n = np.array([-e[1], e[0]])
    K = B + v / 2 + n * math.sqrt(max(T.LINK_LEN ** 2 - (L / 2) ** 2, 0.0))
    if theta == T.DEPLOY_DEG:
        K = T._xz(T.K_PT)
    root = _angle_y(T._xz(T.K_PT) - B, K - B)
    upper = _angle_y(T._xz(T.P_PT) - T._xz(T.K_PT), P - K)
    return {"hinge": math.radians(theta - T.DEPLOY_DEG), "root": root, "knee": upper - root}


def radiator_state(theta):
    q = strut_q(theta)
    poses = T.RADIATOR.link_poses(q)
    z = {n: np.zeros(3) for n in poses}
    return lk.LinkState(pos=poses["base"][:3, 3], quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                        ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), spec=T.RADIATOR, poses=poses,
                        lin_vels=z, ang_vels=dict(z), q={k: q[k] for k in ("hinge", "root", "knee")},
                        qd={"hinge": 0.0, "root": 0.0, "knee": 0.0})


def rest_state():
    poses = T.REST.link_poses()
    return lk.LinkState(pos=poses["support"][:3, 3], quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                        ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), spec=T.REST, poses=poses,
                        lin_vels={"support": np.zeros(3)}, ang_vels={"support": np.zeros(3)}, q={}, qd={})


def tool(side, pos):
    return lk.Tool(side, np.asarray(pos, dtype=float), np.diag([1.0, -1.0, -1.0]), 0.09, 0.0)


FAR = {"left": tool("left", (0.0, 0.3, 1.1)), "right": tool("right", (0.0, -0.3, 1.1))}


def test_locked_design_pose():
    s = radiator_state(75.0)
    assert s.joint_error("lug") < 1e-9
    m = T.measure(s, rest_state(), FAR)
    assert m["theta"] == pytest.approx(75.0) and m["bend"] == pytest.approx(-3.0)
    assert m["lug"] < 1e-9
    assert min(m["clear"].values()) > 0.2
    # the knee sits on the hinge side of the B-P line, below it: gravity on the links and the strut's compression
    # both press it onto its stop; unlocking moves it up and back toward the robot
    assert T.N_UB[1] > 0 and T.N_UB[0] < 0
    mid = (T._xz(T.B_PT) + T._xz(T.P_PT)) / 2
    assert float((T._xz(T.K_PT) - mid) @ T.N_UB) < 0
    assert T.LINK_LEN == pytest.approx(0.0598, abs=1e-4)


@pytest.mark.parametrize("theta", [74.0, 60.0, 40.0, 20.0, 0.0])
def test_strut_folds_closed_through_the_stroke(theta):
    s = radiator_state(theta)
    assert s.joint_error("lug") < 1e-9, "the loop joint closes: link lengths and joint signs agree"
    bend = s.deg("knee") - T.LOCK_DEG
    assert 0.0 < bend < 150.0, "within the knee's travel"
    assert T.measure(s, rest_state(), FAR)["theta"] == pytest.approx(theta)


def test_stowed_panel_lies_on_the_pads_and_the_strut_clears_them():
    s = radiator_state(0.0)
    assert s.deg("knee") - T.LOCK_DEG == pytest.approx(119.7, abs=0.1)
    rest = rest_state()
    for y in (0.065, -0.065):  # panel underside over each pad centre, in the linkage frame
        under = s.point("panel", (0.095, y, T.PANEL_T / 2))
        assert under[2] == pytest.approx(T.HINGE_Z + T.PAD_TOP, abs=1e-9)
        assert rest.distance(under) == pytest.approx(0.0, abs=1e-6)
    for link in ("lower", "upper"):
        for x in np.linspace(-0.5, 0.5, 11):
            p = s.point(link, (x * T.LINK_LEN, 0.0, 0.0))
            assert rest.distance(p) > 0.02, "the folded strut stays beside the support"


def test_free_fall_from_75_deg_would_slam():
    I = T.PANEL_MASS * (T.PANEL_R0 + T.PANEL_LEN + T.TAB[0]) ** 2 / 3  # thin plate about its edge, upper bound
    w = math.sqrt(2 * T.PANEL_MASS * 9.81 * 0.066 * math.sin(math.radians(75.0)) / I)
    assert math.degrees(w) > 5 * T.GATE_RATE


def test_clearance_reads_the_pads_and_the_tab():
    s = radiator_state(0.0)
    tab = s.point("panel", T.TAB_LOCAL)
    near = {"left": tool("left", tab + [0.0, 0.0, 0.03]), "right": FAR["right"]}
    assert T.measure(s, rest_state(), near)["clear"]["left"] < T.CLEAR_M
    up = {"left": tool("left", tab + [0.0, 0.0, 0.08]), "right": FAR["right"]}
    assert T.measure(s, rest_state(), up)["clear"]["left"] >= T.CLEAR_M
