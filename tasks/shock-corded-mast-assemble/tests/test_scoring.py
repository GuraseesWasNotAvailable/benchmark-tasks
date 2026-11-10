# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for shock-corded-mast-assemble, pure Python (no simulator). Run from the task folder:
pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); most tests feed it
synthetic samples. The measure() tests build real LinkStates from the mast's kinematics at chosen joint values.
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
    """A sample of the finished state: both joints home, the mast in both slots, hands clear and still."""
    m = {"s": (0.2, 0.3), "th": (0.1, -0.1), "support": [(0.5, 0.0, 0.0), (-0.5, 0.0, 0.0)], "speed": 0.0,
         "clear": {"left": 0.08, "right": 0.08}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def run(samples, rec=None):
    rec = rec or T.Record()
    t = 0.0
    for m in samples:
        t += DT
        rec.update(m, t)
    return rec


def secs(s):
    return round(s / DT)


def demo_like():
    """Start bent and open, joint 2 pushed home, joint 1 pushed home, carried to the stands, let go."""
    bent = sample(s=(26.0, 26.0), th=(-60.0, 60.0), support=[(-70.0, -40.0, 0.0), (-70.0, -40.0, 0.0)],
                  clear={"left": 0.2, "right": 0.2})
    j2 = sample(s=(26.0, 0.4), th=(-60.0, 0.1), support=bent["support"], clear={"left": 0.0, "right": 0.0})
    both = sample(s=(0.4, 0.4), support=bent["support"], clear={"left": 0.0, "right": 0.0})
    held = sample(clear={"left": 0.0, "right": 0.0})
    return ([bent] * secs(2) + [j2] * secs(5) + [both] * secs(5) + [held] * secs(2) + [sample()] * secs(1.2))


def test_demo_shaped_trace_scores_one():
    assert T.ladder(run(demo_like()))[0] == 1.0


def test_no_steps_and_nothing_done():
    assert T.ladder(T.Record())[0] == 0.0
    bent = sample(s=(26.0, 26.0), th=(-60.0, 60.0), support=[(-70.0, -40.0, 0.0)] * 2)
    assert T.ladder(run([bent] * secs(5)))[0] == 0.0


def test_seated_limits():
    assert T.seated(sample(s=(1.0, 0.0)), 0)
    assert not T.seated(sample(s=(1.1, 0.0)), 0)
    assert T.seated(sample(th=(1.0, 0.0)), 0)
    assert not T.seated(sample(th=(1.1, 0.0)), 0)
    assert not T.seated(sample(th=(0.0, -1.1)), 1)


@pytest.mark.parametrize("support, ok", [
    ([(6.0, 0.0, 0.0), (0.0, 0.0, 0.0)], True), ([(6.1, 0.0, 0.0), (0.0, 0.0, 0.0)], False),
    ([(0.0, -2.0, 0.0), (0.0, 4.5, 0.0)], True), ([(0.0, -2.1, 0.0), (0.0, 0.0, 0.0)], False),
    ([(0.0, 0.0, 0.0), (0.0, 4.6, 0.0)], False), ([(0.0, 0.0, 12.0), (0.0, 0.0, -12.0)], True),
    ([(0.0, 0.0, 12.1), (0.0, 0.0, 0.0)], False)])
def test_on_stands_limits(support, ok):
    assert T.on_stands(sample(support=support)) is ok


def test_cord_drawn_in_but_not_pushed_home_scores_zero():
    """Lined up and let go, the cord leaves each spigot 12 to 15 mm short: never seated, even on the stands."""
    part = sample(s=(13.0, 14.5), th=(0.2, 0.1))
    assert T.ladder(run([part] * secs(5)))[0] == 0.0


def test_one_joint_seated_is_kept_by_the_record():
    j2 = sample(s=(26.0, 0.5), th=(-60.0, 0.0), support=[(-70.0, -40.0, 0.0)] * 2)
    pulled = sample(s=(30.0, 40.0), th=(-30.0, 2.0), support=[(-70.0, -40.0, 0.0)] * 2)
    rec = run([j2] * 3 + [pulled] * secs(5))
    assert T.ladder(rec) == (0.25, "one joint was fully seated at some time")


def test_both_seated_once_then_pulled_apart_scores_half():
    rec = run([sample(support=[(-70.0, -40.0, 0.0)] * 2)] * 2 + [sample(s=(20.0, 0.5))] * secs(3))
    assert T.ladder(rec)[0] == 0.5


def test_both_seated_but_not_on_the_stands_scores_half():
    on_table = sample(support=[(-100.0, -40.0, 0.0)] * 2)
    assert T.ladder(run([on_table] * secs(5)))[0] == 0.5


def test_release_gate():
    base = demo_like()[:-secs(1.2)]
    assert T.ladder(run(base + [sample()] * secs(0.95)))[0] == 0.75
    near = sample(clear={"left": 0.029, "right": 0.08})
    assert T.ladder(run(base + [sample()] * secs(1.2) + [near]))[0] == 0.75
    assert T.ladder(run(base + [sample(clear={"left": 0.030, "right": 0.08})] * secs(1.2)))[0] == 1.0
    moving = sample(tcp_speed={"left": 0.006, "right": 0.0})
    assert T.ladder(run(base + [moving] * secs(1.2)))[0] == 0.75


def test_mast_must_be_still_on_the_stands():
    base = demo_like()[:-secs(1.2)]
    assert T.ladder(run(base + [sample(speed=0.006)] * secs(1.2)))[0] == 0.5  # moving: not resting at the end
    assert T.ladder(run(base + [sample(speed=0.0049)] * secs(1.2)))[0] == 1.0


def test_reset_clears_the_record():
    rec = run(demo_like())
    rec.reset()
    assert rec.last is None and not rec.H.evers and not rec.H.runs
    bent = sample(s=(26.0, 26.0), th=(-60.0, 60.0), support=[(-70.0, -40.0, 0.0)] * 2)
    assert T.ladder(run([bent] * 3, rec))[0] == 0.0


def test_ferrule_entry_and_lock():
    D, fold = T.D, math.radians(T.FOLD_DEG)
    sl, hl = T.ferrule_limits(D, math.radians(5.0))  # bent 5 deg at the mouth: the spigot cannot enter
    assert sl == (D, T.S_MAX) and hl == (-fold, fold)
    sl, _ = T.ferrule_limits(D + 0.01, math.radians(60.0))  # bent 60 deg: the tip rides 3.5 mm out on the rim
    assert sl[0] - D == pytest.approx((0.0065 - 0.007 * 0.5) / math.sin(math.radians(60.0)))
    assert T.rim_offset(math.radians(21.0)) == 0.0 and T.rim_offset(math.radians(23.0)) > 0.0
    sl, hl = T.ferrule_limits(D + 0.02, math.radians(3.0))  # lined up within 3 deg: it may enter
    assert sl == (0.0, T.S_MAX) and hl == (-fold, fold)
    sl, hl = T.ferrule_limits(D - 0.001, 0.0)  # 1 mm in: held within 2.865 deg
    assert sl == (0.0, T.S_MAX) and math.degrees(hl[1]) == pytest.approx(3.0 - 2.7 * 1 / 20)
    sl, hl = T.ferrule_limits(0.0, 0.0)  # home: held within 0.3 deg
    assert math.degrees(hl[1]) == pytest.approx(T.LOCK_DEG)


# -- measure() on real link states ------------------------------------------------------------------------------


def mast_state(s1=0.0, f1=0.0, s2=0.0, f2=0.0, shift=(0.0, 0.0, 0.0)):
    q = {"s1": s1, "f1": math.radians(f1), "s2": s2, "f2": math.radians(f2)}
    frame = T.MAST.frame.copy()
    frame[:3, 3] += np.asarray(shift, dtype=float)
    poses = T.MAST.link_poses(q, frame)
    z = {n: np.zeros(3) for n in poses}
    return lk.LinkState(pos=poses["base"][:3, 3], quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                        ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), spec=T.MAST, poses=poses,
                        lin_vels=z, ang_vels=dict(z), q=q, qd={k: 0.0 for k in q})


def far_tool(side):
    return lk.Tool(side, np.array([0.3, 0.3 if side == "left" else -0.3, 1.1]), np.diag([1.0, -1.0, -1.0]), 0.05, 0.0)


TOOLS = {"left": far_tool("left"), "right": far_tool("right")}
ON_STANDS = (T.X_ST - T.X0, float(np.mean(T.STAND_Y)), T.FLOOR)  # mast centred over the stands


def test_measure_assembled_on_the_stands():
    st = mast_state(shift=ON_STANDS)
    m = T.measure(st, TOOLS)
    assert m["s"] == (0.0, 0.0) and T.seated(m, 0) and T.seated(m, 1)
    for dx, dz, dy in m["support"]:
        assert abs(dx) < 1e-6 and abs(dz) < 1e-6 and abs(dy) < 1e-6
    assert T.on_stands(m) and min(m["clear"].values()) > 0.2


def test_measure_assembled_on_the_table_is_not_on_the_stands():
    m = T.measure(mast_state(), TOOLS)
    assert all(abs(dz + 1000 * T.FLOOR) < 1e-6 for _, dz, _ in m["support"])
    assert not T.on_stands(m)


def test_measure_rolled_onto_an_edge_is_still_in_the_slots():
    """Rolled 45 deg about its own axis, the square tube's centre sits 3.7 mm higher in the slot."""
    up = (math.sqrt(2) - 1) * T.TUBE / 2
    m = T.measure(mast_state(shift=(ON_STANDS[0], ON_STANDS[1], T.FLOOR + up)), TOOLS)
    assert all(abs(dz - 1000 * up) < 1e-6 for _, dz, _ in m["support"]) and T.on_stands(m)


def test_measure_resting_on_the_cheeks_is_not_in_the_slots():
    m = T.measure(mast_state(shift=(ON_STANDS[0] + 0.017, ON_STANDS[1], T.FLOOR + 0.012)), TOOLS)
    assert not T.on_stands(m)


def test_measure_start_pose():
    """The start pose as spawned: each bent spigot's tip on its ferrule's rim, 3.5 mm outside the 26 mm mouth."""
    assert 1000 * (T.S_START - T.D) == pytest.approx(3.46, abs=0.01)
    st = mast_state(T.S_START, T.KINK[0], T.S_START, T.KINK[1])
    m = T.measure(st, TOOLS)
    assert m["s"] == pytest.approx((29.46, 29.46), abs=0.01) and m["th"] == pytest.approx((-60.0, 60.0))
    assert not T.seated(m, 0) and not T.on_stands(m)
    top_axis = st.rotation("top")[:, 0]
    assert top_axis == pytest.approx([0.0, -1.0, 0.0], abs=1e-9)  # the top section lies parallel to the base
    x_top = T.X0 - (T.SEG + T.S_START) * math.sin(math.radians(60))  # 0.222
    assert st.point("top", (0.0, 0.0, 0.0))[0] == pytest.approx(x_top, abs=1e-6) and x_top == pytest.approx(0.222, abs=5e-4)


def test_cord_and_fit_forces():
    """At the fit's entrance the cord pulls exactly the fit's grip; sliding in, the cord out-pulls the fit's drag
    only above 11.7 mm, so a spigot drawn in by the cord stops 11.7 to 15 mm short; home needs a 1.75 N push."""
    assert T.cord_force(T.D) == pytest.approx(4.65)
    assert T.cord_force(T.FIT_LEN) == pytest.approx(T.FIT_STATIC)
    s_drag = T.FIT_DYNAMIC / T.CORD_K + T.CORD_REST
    assert 0.0115 < s_drag < 0.0118
    assert T.FIT_DYNAMIC - T.cord_force(0.0) == pytest.approx(1.75)


def test_fit_stick_slip():
    nxt = T.fit_next
    assert nxt("stuck", 0.012, 0.016, 0.0) == ("free", None)  # outside the fit the cord pulls freely
    assert nxt("free", None, 0.0149, -0.05) == ("in", None)  # drawn into the fit by the cord, still moving
    assert nxt("in", None, 0.012, -0.0019) == ("stuck", 0.012)  # slowed below 2 mm/s: sticks
    assert nxt("in", None, 0.012, -0.0021) == ("in", None)
    hold = 0.011  # stuck 11 mm short: the cord gives 2.4 N, so a 0.6 N push starts it
    assert nxt("stuck", hold, hold - 0.55 / T.FIT_HOLD_K, 0.0) == ("stuck", hold)
    assert nxt("stuck", hold, hold - 0.65 / T.FIT_HOLD_K, 0.0) == ("in", None)
    hold = 0.0  # home: pulling it out takes the cord's 0.75 N plus the fit's 3 N
    assert nxt("stuck", hold, hold + 3.7 / T.FIT_HOLD_K, 0.0) == ("stuck", hold)
    assert nxt("stuck", hold, hold + 3.8 / T.FIT_HOLD_K, 0.0) == ("out", None)
    assert nxt("out", None, 0.003, 0.0019) == ("stuck", 0.003)
    assert nxt("free", None, 0.010, 0.0) == ("stuck", 0.010)
