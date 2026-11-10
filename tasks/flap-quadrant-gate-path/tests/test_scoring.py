# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer and geometry edge cases for flap-quadrant-gate-path, pure Python (no simulator).
Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); most tests feed it
synthetic samples. The geometry tests place the trainer's links with its own kinematics (Linkage.link_poses) and
measure the distance from the lever's shaft and gate pin to the gate plate's colliders.
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
FAR = {"left": 0.20, "right": 0.15}
LANE_B = T.LANE_B_REST  # lateral angle with the shaft resting on the outboard face of the gate wall


def sample(**kw):
    """The finished state: pin seated in FULL, trainer in place, knob still, both hands clear and still."""
    m = {"pitch": T.FULL, "lat": LANE_B, "lift": 0.0, "disp": 0.0005, "v": 0.0, "w": 0.0, "clear": dict(FAR),
         "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def busy(**kw):
    """Hands on the trainer and moving."""
    base = dict(clear={"left": 0.0, "right": 0.0}, tcp_speed={"left": 0.01, "right": 0.02})
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


def ramp(key, a, b, s, **kw):
    n = int(round(s / DT))
    return [busy(**{key: a + (b - a) * k / n}, **kw) for k in range(1, n + 1)]


def demo_trace():
    """Seated at UP; lift 9.5 mm; carry to 15; jog; carry to FULL; lower; hands away; 2 s still."""
    out = secs(sample(pitch=0.0, lat=0.0, clear=dict(FAR)), 3.0)
    out += ramp("lift", 0.0, 0.0095, 2.0, pitch=0.0, lat=0.0)
    out += ramp("pitch", 0.0, 20.0, 2.5, lift=0.0095, lat=0.0)
    out += ramp("lat", 0.0, -11.4, 1.5, lift=0.0095, pitch=20.0)
    out += ramp("pitch", 20.0, 40.0, 2.5, lift=0.0095, lat=-11.4)
    out += ramp("lift", 0.0095, 0.0, 1.5, pitch=40.0, lat=-11.4)
    out += secs(busy(clear={"left": 0.0, "right": 0.06}), 1.5)
    out += secs(sample(), 2.0)
    return out


def score(samples):
    return T.ladder(feed(samples))


def test_demo_trace_scores_one():
    s, why = score(demo_trace())
    assert s == 1.0, why


def test_move_gate_whole_run_and_threshold():
    tr = demo_trace()
    assert score(tr[:5] + [busy(disp=0.0100)] + tr[5:])[0] == 1.0
    s, why = score(tr[:5] + [busy(disp=0.0101)] + tr[5:])
    assert s == 0.0 and "gate" in why
    # a lift of the trainer at the end of the run is caught too, and putting it back does not clear it
    assert score(tr[:-10] + [sample(disp=0.03)] + secs(sample(), 2.0))[0] == 0.0


def test_seat_threshold():
    assert score(demo_trace()[:-40] + secs(sample(lift=0.00099), 2.0))[0] == 1.0
    s, why = score(demo_trace()[:-40] + secs(sample(lift=0.0010), 2.0))
    assert s == 0.0 and "not seated" in why


def test_full_tolerance():
    assert score(demo_trace()[:-40] + secs(sample(pitch=41.0), 2.0))[0] == 1.0
    assert score(demo_trace()[:-40] + secs(sample(pitch=39.0), 2.0))[0] == 1.0
    assert score(demo_trace()[:-40] + secs(sample(pitch=41.1), 2.0))[0] == 0.25


def test_past_gate_rung():
    assert score(demo_trace()[:-40] + secs(sample(pitch=30.0), 2.0))[0] == 0.25
    assert score(demo_trace()[:-40] + secs(sample(pitch=25.0), 2.0))[0] == 0.25
    s, why = score(demo_trace()[:-40] + secs(sample(pitch=24.9), 2.0))
    assert s == 0.0 and "short of" in why


def test_cheat_shapes_score_zero():
    # push only: never lifted, back at UP
    assert score(secs(sample(pitch=0.0, lat=0.0), 3.0) + ramp("pitch", 0.0, 1.2, 2.0, lat=0.0)
                 + secs(sample(pitch=0.6, lat=0.0), 2.0))[0] == 0.0
    # lift and push straight: stopped by the gate wall, dropped into the 15 notch
    assert score(secs(sample(pitch=0.0, lat=0.0), 2.0) + ramp("pitch", 0.0, 21.6, 2.0, lift=0.0095, lat=0.0)
                 + secs(sample(pitch=20.5, lat=0.0), 2.0))[0] == 0.0
    # left lifted at FULL with the hand still on the knob
    assert score(demo_trace()[:-70] + secs(busy(lift=0.0095, tcp_speed={"left": 0.0, "right": 0.0}), 3.0))[0] == 0.0
    # let go over the 30-FULL tooth: the pin rests on it, unseated
    assert score(demo_trace()[:-40] + secs(sample(pitch=34.5, lift=0.0062), 2.0))[0] == 0.0


def test_still_and_release_rungs():
    assert score(demo_trace() + secs(sample(v=0.004), 0.5))[0] == 0.5
    assert score(demo_trace() + secs(sample(w=math.radians(3.1)), 0.5))[0] == 0.5
    assert score(demo_trace() + [sample(clear={"left": 0.20, "right": 0.029})] + secs(sample(), 0.9))[0] == 0.75
    assert score(demo_trace() + [sample(clear={"left": 0.20, "right": 0.030})] + secs(sample(), 0.9))[0] == 1.0
    assert score(demo_trace() + [sample(tcp_speed={"left": 0.006, "right": 0.0})] + secs(sample(), 0.9))[0] == 0.75
    # held seated at FULL, still, hand on: 0.75; 0.95 s of release is not enough
    assert score(demo_trace()[:-40] + secs(busy(tcp_speed={"left": 0.0, "right": 0.0}), 2.0))[0] == 0.75
    tr = demo_trace()[:-40] + secs(busy(tcp_speed={"left": 0.0, "right": 0.0}), 1.0) + secs(sample(), 0.95)
    assert score(tr)[0] == 0.75


def test_record_reset_clears_gate():
    rec = feed(demo_trace()[:10] + [busy(disp=0.05)])
    assert T.ladder(rec)[0] == 0.0
    rec.reset()
    assert rec.last is None and not rec.H.evers and not rec.H.peaks
    assert T.ladder(feed(demo_trace(), rec))[0] == 1.0


def test_pose_rates():
    A, B = np.eye(4), np.eye(4)
    B[:3, 3] = (0.0002, 0.0, 0.0)
    v, w = T.pose_rates(A, B, DT)
    assert v == pytest.approx(0.004) and w == pytest.approx(0.0, abs=1e-9)
    C = np.eye(4)
    C[:3, :3] = lk.rpy_matrix((0.0, 0.0, 0.2))
    assert T.pose_rates(A, C, DT)[1] == pytest.approx(math.radians(4.0), rel=1e-6)


# -- geometry ------------------------------------------------------------------------------------------------------

BASE_SHAPES = T.QUADRANT.link("base").shapes


def poses(pitch, lat, lift):
    return T.QUADRANT.link_poses({"pitch": math.radians(pitch), "lateral": math.radians(lat), "lift": lift},
                                 frame=np.eye(4))


def gap(points):
    """Smallest distance (m) from points in the trainer frame to the base's colliders (0 inside one)."""
    return min(s.distance(p) for p in points for s in BASE_SHAPES)


def pin_surface(pitch, lat, lift, n=13):
    """Points on the gate pin's surface (its lower half, where it meets teeth and plate) in the trainer frame."""
    K = poses(pitch, lat, lift)["knob"]
    pts = []
    for y in np.linspace(-T.PIN_HALF, T.PIN_HALF, n):
        for a in np.linspace(math.pi / 2, 3 * math.pi / 2, 9):  # around the pin, the half facing the plate
            local = np.array([T.PIN_R * math.sin(a), y, T.R_PIN + T.PIN_R * math.cos(a)])
            pts.append(K[:3, :3] @ local + K[:3, 3])
    return pts


def shaft_surface(pitch, lat, n=8):
    L = poses(pitch, lat, 0.0)["lever"]
    pts = []
    for r in np.linspace(T.R_PLATE[0], 0.088, 5):
        for a in np.linspace(0, 2 * math.pi, n, endpoint=False):
            local = np.array([T.SHAFT_R * math.cos(a), T.SHAFT_R * math.sin(a), r])
            pts.append(L[:3, :3] @ local + L[:3, 3])
    return pts


@pytest.mark.parametrize("notch,lat", [("UP", 0.0), ("5", 0.0), ("15", 0.0), ("15", LANE_B - 0.3),
                                       ("30", LANE_B - 0.3), ("FULL", LANE_B - 0.3)])
def test_seated_pin_and_shaft_fit_their_notch(notch, lat):
    th = T.NOTCHES[notch]
    assert gap(pin_surface(th, lat, 0.0)) > 0.0002
    assert gap(shaft_surface(th, lat)) > 0.0003


def test_seated_pin_is_blocked_by_the_next_tooth():
    # pushed 3 deg from UP without a lift, the pin runs into the UP-5 tooth
    assert gap(pin_surface(3.0, 0.0, 0.0)) == 0.0


def test_lifted_pin_clears_the_teeth_along_both_lanes():
    for th in np.arange(0.0, 20.6, 0.5):
        assert gap(pin_surface(th, 0.0, T.LIFT_T)) > 0.0005, th
    for th in np.arange(19.5, 40.6, 0.5):
        assert gap(pin_surface(th, T.JOG, T.LIFT_T)) > 0.0005, th
    for lat in np.arange(0.0, T.JOG - 0.1, -0.5):
        assert gap(pin_surface(20.0, lat, T.LIFT_T)) > 0.0005, lat
    # with 6 mm of lift the pin's lowered end still meets the teeth
    assert gap(pin_surface(5.0, 0.0, 0.006)) == 0.0


def test_gate_wall_stops_the_inboard_lane_and_the_jog_passes():
    assert gap(shaft_surface(21.0, 0.0)) > 0.0
    assert gap(shaft_surface(22.5, 0.0)) == 0.0  # the gate wall: no lift clears it (the shaft does not lift)
    assert gap(shaft_surface(22.5, T.JOG)) > 0.0005
    assert gap(shaft_surface(10.0, T.JOG)) == 0.0  # the outboard lane is closed before the gate
    assert gap(shaft_surface(40.0, LANE_B - 0.3)) > 0.0002
    # the outboard lane holds the shaft between the gate wall and its outer wall
    assert gap(shaft_surface(35.0, LANE_B - 0.2)) > 0.0 and gap(shaft_surface(35.0, LANE_B + 0.3)) == 0.0
    assert gap(shaft_surface(35.0, T.LANE_B_OUT + 0.2)) > 0.0 and gap(shaft_surface(35.0, T.LANE_B_OUT - 0.3)) == 0.0


def test_lever_angles_from_poses_match_the_joints():
    for th, lat, lift in ((0.0, 0.0, 0.0), (20.0, -11.4, 0.0095), (40.0, -10.7, 0.0), (33.0, -5.0, 0.004)):
        P = poses(th, lat, lift)
        got = T.lever_from_poses(P["base"], P["lever"], P["knob"])
        assert got[0] == pytest.approx(th, abs=1e-6)
        assert got[1] == pytest.approx(lat, abs=1e-6)
        assert got[2] == pytest.approx(lift, abs=1e-9)


def test_measure_on_real_link_states():
    P = T.QUADRANT.link_poses({"pitch": math.radians(40.0), "lateral": math.radians(LANE_B), "lift": 0.0})
    z = {k: np.zeros(3) for k in P}
    st = lk.LinkState(pos=P["base"][:3, 3], quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                      ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), spec=T.QUADRANT, poses=P, lin_vels=z,
                      ang_vels=dict(z), q={"pitch": math.radians(40.0), "lateral": math.radians(LANE_B), "lift": 0.0},
                      qd={"pitch": 0.0, "lateral": 0.0, "lift": 0.0})
    knob = P["knob"][:3, :3] @ np.array([0.0, 0.0, T.R_KNOB]) + P["knob"][:3, 3]
    far = lk.Tool("left", np.array([0.2, 0.3, 1.1]), np.diag([1.0, -1.0, -1.0]), 0.09, 0.0)
    on = lk.Tool("right", knob, np.diag([1.0, -1.0, -1.0]), 0.022, 0.0)
    m = T.measure(st, {"left": far, "right": on})
    assert m["pitch"] == pytest.approx(40.0) and m["lat"] == pytest.approx(LANE_B) and m["lift"] == 0.0
    assert m["disp"] == pytest.approx(0.0, abs=1e-9)
    assert m["clear"]["right"] == 0.0 and m["clear"]["left"] > 0.2
    assert T.at_full(m)
    # the base lifted 11 mm is a move; tilted 3 deg about its far (+y) edge, its handle end rises over 10 mm
    B = P["base"].copy()
    B[2, 3] += 0.011
    assert np.max(np.linalg.norm(T.base_points(B) - T.PTS0, axis=1)) == pytest.approx(0.011)
    edge = T.base_points(P["base"])[2]  # a corner on the far edge
    R = lk.axis_rotation(P["base"][:3, 0], math.radians(3.0))
    B = P["base"].copy()
    B[:3, :3] = R @ B[:3, :3]
    B[:3, 3] = R @ (B[:3, 3] - edge) + edge
    assert np.max(np.linalg.norm(T.base_points(B) - T.PTS0, axis=1)) > 0.010


def test_trainer_is_lighter_than_the_lift_spring():
    total = T.M_BASE + T.M_YOKE + T.M_LEVER + T.M_KNOB
    assert (total - T.M_KNOB) * 9.81 < T.LIFT_PRELOAD  # the trainer leaves the table before the knob moves
    assert T.LIFT_PRELOAD + T.LIFT_K * T.LIFT_MAX < 7.0  # inside what the soft arm can pull


def test_slot_layout():
    rects = T.plate_rects()
    # inboard lane open UP..gate wall, outboard lane open from the jog to FULL, the jog open between them
    def solid(th, y):
        return any(a <= th <= b and c <= y <= d for a, b, c, d in rects)
    assert not solid(0.0, 0.0075) and not solid(20.0, 0.0) and not solid(40.0, -0.0075)
    assert solid(30.0, 0.0075) and solid(5.0, -0.0075) and solid(5.0, 0.0)
