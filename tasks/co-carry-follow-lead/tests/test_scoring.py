# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for co-carry-follow-lead, without the simulator. Run from the task folder: pytest -q tests

Each case feeds synthetic control steps (pipe pose, the person's hand, both grippers) through the task's own
measure(), record_step() and ladder(), the same functions the simulator calls every step.
"""

import importlib.util
import math
import pathlib
import sys
import zlib

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
_spec = importlib.util.spec_from_file_location("cocarry_task", HERE / "task.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
lk = T.lk

DT = 1.0 / T.CONTROL_HZ
DOWN = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # tool frame: x closes along world y
PARK = {"left": np.array([0.10, 0.25, T.TOP + 0.25]), "right": np.array([0.10, -0.25, T.TOP + 0.25])}
GRASP_N = 40  # first step the grippers hold
from yam_arena.constants import PAD_TIP_PAST_TCP as PAD_TIP  # noqa: E402  (pad tips past the TCP, m)


def pose(centre, pitch_deg=0.0, yaw_deg=0.0) -> np.ndarray:
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    x = np.array([math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), math.sin(p)])
    m = np.eye(4)
    m[:3, 0] = x
    m[:3, 1] = np.cross([0.0, 0.0, 1.0], x) / np.linalg.norm(np.cross([0.0, 0.0, 1.0], x))
    m[:3, 2] = np.cross(m[:3, 0], m[:3, 1])
    m[:3, 3] = centre
    return m


def frames(seed=0, *, carry=True, set_down=True, release_s=0.5, park=True, gap_mm=None, still_steps=40,
           holds=True, trigger_break=False, open_w=0.060):
    """An ideal co-carry, step by step: grasp at step 40, the person starts 20 steps later, the pipe follows the
    hand exactly, the grippers open ``release_s`` after the person's lowering ends, rise 60 mm and park."""
    plan = T.partner_plan(seed)
    n_trig = GRASP_N + T.TRIGGER_STEPS - 1
    t_trig = n_trig * DT
    t_rel = t_trig + T.plan_length(plan) + release_s
    n_end = int(round((t_rel + 3.0) * T.CONTROL_HZ)) + still_steps
    grasp = {s: np.array([x, T.PIPE_C[1], T.PIPE_C[2]]) for s, x in T.GRIP_X.items()}
    out, prev = [], None
    for n in range(n_end):
        t = n * DT
        dy, dz = T.partner_offset(plan, t - t_trig) if n > n_trig else (0.0, 0.0)
        if not carry:
            dy = 0.0
        if not holds:  # nobody took hold, so the person never starts
            dy = dz = 0.0
        if not set_down and t > t_trig + T.plan_length(plan) - plan["lower"]:
            dz = T.LIFT
        off = np.array([0.0, dy, dz])
        centre = T.PIPE_C + off
        tools = {}
        for s in grasp:
            if n < GRASP_N or not holds:
                tcp, opening = grasp[s] + [0, 0, 0.06], 0.060
            elif trigger_break and n == GRASP_N + 10:
                tcp, opening = grasp[s] + off, 0.035  # one step loose: the hold count restarts
            elif t < t_rel:
                tcp, opening = grasp[s] + off, 0.025
            elif t < t_rel + 0.6:
                tcp, opening = grasp[s] + off, open_w
            elif t < t_rel + 1.6 or not park:
                tcp, opening = grasp[s] + off + [0, 0, min(0.06, (t - t_rel - 0.6) * 0.06)], open_w
            else:
                tcp, opening = PARK[s], open_w
            tools[s] = tcp, opening
        if gap_mm is not None and t > t_rel + 1.6:  # left gripper parked closed just above the pipe instead
            tools["left"] = centre + [-0.17, 0.0, T.PIPE_R + gap_mm / 1000 + PAD_TIP], 0.0
        out.append({"t": t, "pose": pose(centre), "hand": T.H0 + off, "tools": tools})
    for i, f in enumerate(out):  # speeds from differences
        g = out[max(i - 1, 0)]
        f["pipe_vel"] = (f["pose"][:3, 3] - g["pose"][:3, 3]) / DT
        f["tool_speed"] = {s: float(np.linalg.norm(f["tools"][s][0] - g["tools"][s][0])) / DT for s in f["tools"]}
    return plan, out


def run(plan, fs, edit=None):
    """Feed frames through measure/record_step; ``edit(i, frame)`` may change a frame. Returns (score, record)."""
    rec = lk.History()
    m = None
    for i, f in enumerate(fs):
        if edit:
            f = dict(f, tools=dict(f["tools"]))  # edits never leak into the shared frames
            edit(i, f)
        tools = {s: lk.Tool(s, np.asarray(tcp, float), DOWN, float(op), f["tool_speed"][s])
                 for s, (tcp, op) in f["tools"].items()}
        m = T.measure(f["pose"], f["hand"], f["pipe_vel"], np.zeros(3), tools)
        rec._tick(f["t"])
        T.record_step(rec, m, f["t"], plan)
    return T.ladder(rec, m), rec


def test_demo_trace_scores_one():
    for seed in (0, 42, 156):
        plan, fs = frames(seed)
        score, rec = run(plan, fs)
        assert score == 1.0, seed
        assert rec.values["t_release"] >= rec.values["t_lower_end"]
        assert rec.peaks["stretch"] < 1e-9 and rec.peaks["pitch"] < 1e-9


def test_stretch_26mm_for_one_step_is_zero_even_after_coming_back():
    plan, fs = frames()
    mid = len(fs) // 3

    def edit(i, f):
        if i == mid:
            f["hand"] = f["hand"] + np.array([0.0, 0.0, 0.026])

    assert run(plan, fs, edit)[0] == 0.0

    def edit_ok(i, f):
        if i == mid:
            f["hand"] = f["hand"] + np.array([0.0, 0.0, 0.024])

    assert run(plan, fs, edit_ok)[0] == 1.0


@pytest.mark.parametrize("pitch,yaw,expect", [(4.5, 0.0, 0.0), (-4.5, 0.0, 0.0), (0.0, 4.5, 0.0), (3.5, -3.5, 1.0)])
def test_tilt_gate_any_step(pitch, yaw, expect):
    plan, fs = frames()
    mid = len(fs) // 2

    def edit(i, f):
        if i == mid:
            f["pose"] = pose(f["pose"][:3, 3], pitch, yaw)
            u = f["pose"][:3, 0]
            f["hand"] = f["pose"][:3, 3] + T.HAND_AT * u  # tipped as a rigid pipe, no stretch

    assert run(plan, fs, edit)[0] == expect


def test_lifted_but_never_carried_is_quarter():
    plan, fs = frames(carry=False)
    assert run(plan, fs)[0] == 0.25


def test_carried_but_not_set_is_half():
    plan, fs = frames(set_down=False)
    assert run(plan, fs)[0] == 0.5


def test_set_but_a_gripper_20mm_away_is_three_quarters():
    plan, fs = frames(gap_mm=20)
    assert run(plan, fs)[0] == 0.75
    plan, fs = frames(gap_mm=35)
    assert run(plan, fs)[0] == 1.0


def test_released_before_the_person_lowered_is_capped():
    plan, fs = frames(release_s=-0.6)  # opened 0.6 s before the lowering ended, then set and parked
    score, rec = run(plan, fs)
    assert rec.values["t_release"] < rec.values["t_lower_end"]
    assert score == 0.75


def test_letting_go_by_moving_away_counts_as_the_release():
    plan, fs = frames(open_w=0.031)  # opens only to 31 mm, then rises clear: the release is when it leaves the pipe
    score, rec = run(plan, fs)
    assert rec.values["t_release"] >= rec.values["t_lower_end"] + 0.6
    assert score == 1.0
    plan, fs = frames(open_w=0.031, release_s=-1.2)  # ... and doing that before the person lowered is still capped
    score, rec = run(plan, fs)
    assert rec.values["t_release"] < rec.values["t_lower_end"]
    assert score == 0.75


def test_opening_past_32mm_on_the_pipe_is_a_release():
    plan, fs = frames(open_w=0.035)  # opens to 35 mm and stays on the pipe for 0.6 s: released at once
    score, rec = run(plan, fs)
    assert rec.values["t_release"] == pytest.approx(rec.values["t_lower_end"] + 0.5, abs=DT + 1e-9)
    assert score == 1.0
    plan, fs = frames(open_w=0.035, release_s=-0.6)
    assert run(plan, fs)[0] == 0.75


def _regrasp(plan, fs, sides):
    """An edit: mid-carry, ``sides`` open to 60 mm and rise 30 mm for 1 s, then come back and close again."""
    t_trig = (GRASP_N + T.TRIGGER_STEPS - 1) * DT
    t0 = t_trig + plan["wait"] + plan["lift"] + plan["pause1"] + 0.5 * plan["carry"] * plan["frac"]

    def edit(i, f):
        if t0 <= f["t"] < t0 + 1.0:
            for s in sides:
                tcp, _ = f["tools"][s]
                f["tools"][s] = (np.asarray(tcp) + [0.0, 0.0, 0.03], 0.060)

    return edit


def test_one_arm_regrasp_mid_carry_is_not_letting_go():
    plan, fs = frames()
    score, rec = run(plan, fs, _regrasp(plan, fs, ("left",)))  # the right gripper holds throughout
    assert rec.values["t_release"] >= rec.values["t_lower_end"]
    assert score == 1.0
    score, rec = run(plan, fs, _regrasp(plan, fs, ("left", "right")))  # both off the pipe at once: let go
    assert rec.values["t_release"] < rec.values["t_lower_end"]
    assert score == 0.75


def test_needs_twenty_still_steps():
    plan, fs = frames(still_steps=0)
    n_park = next(i for i, f in enumerate(fs) if all(np.allclose(f["tools"][s][0], PARK[s]) for s in PARK))
    fs = fs[:n_park + 18]  # 17 still steps after the last parking move
    assert run(plan, fs)[0] == 0.75
    plan, fs = frames()
    assert run(plan, fs)[0] == 1.0


def test_trigger_needs_twenty_steps_of_hold():
    plan, fs = frames()
    score, rec = run(plan, fs[:GRASP_N + 19])
    assert "t_trigger" not in rec.values
    score, rec = run(plan, fs[:GRASP_N + 20])
    assert rec.values["t_trigger"] == pytest.approx((GRASP_N + 19) * DT)
    plan, fs = frames(trigger_break=True)  # a loose step restarts the count
    score, rec = run(plan, fs[:GRASP_N + 25])
    assert "t_trigger" not in rec.values
    score, rec = run(plan, fs[:GRASP_N + 31])
    assert rec.values["t_trigger"] == pytest.approx((GRASP_N + 30) * DT)


def test_hold_anywhere_short_of_the_persons_fist_and_tight():
    c, y = T.PIPE_C, T.PIPE_C[1]
    for tcp, opening, ok in [((0.20, y, c[2]), 0.025, True), ((0.20, y, c[2] + 0.016), 0.025, False),
                             ((0.44, y, c[2]), 0.025, True), ((0.62, y, c[2]), 0.025, True),
                             ((0.64, y, c[2]), 0.025, False), ((0.20, y, c[2]), 0.031, False),
                             ((0.13, y, c[2]), 0.025, False), ((0.145, y, c[2]), 0.025, True)]:
        m = T.measure(pose(c), T.H0, np.zeros(3), np.zeros(3), {"right": lk.Tool("right", np.array(tcp), DOWN, opening, 0.0)})
        assert m["holding"]["right"] is ok, (tcp, opening)


def test_doing_nothing_is_zero_and_the_person_never_starts():
    plan, fs = frames(holds=False)
    score, rec = run(plan, fs)
    assert score == 0.0 and "t_trigger" not in rec.values


def test_open_gripper_from_before_the_trigger_is_not_a_release():
    plan, fs = frames()

    def edit(i, f):  # the right gripper never closes; the left one holds and triggers the person
        f["tools"]["right"] = (PARK["right"], 0.060)

    score, rec = run(plan, fs, edit)
    assert rec.values.get("held_right") is None
    assert rec.values["t_release"] >= rec.values["t_lower_end"]
    assert score == 1.0


def test_seed_draws_stay_inside_their_ranges():
    seen = set()
    for seed in range(300):
        plan = T.partner_plan(seed)
        for name, lo, hi in T.PHASES:
            assert lo <= plan[name] <= hi
        assert plan == T.partner_plan(seed)  # same seed, same person
        seen.add(round(plan["wait"], 6))
    assert len(seen) == 300


def test_partner_path_ends_with_the_pipe_resting_in_the_v_heads():
    for seed in (0, 1, 42, 156, 170):
        plan = T.partner_plan(seed)
        dy, dz = T.partner_offset(plan, T.plan_length(plan))
        assert dy == pytest.approx(T.CARRY) and dz == pytest.approx(T.LIFT - T.LOWER)
        assert T.PIPE_C[2] + dz == pytest.approx(T.REST_Z, abs=1e-9)
        assert T.PIPE_C[1] + dy == pytest.approx(T.STAND_Y, abs=1e-9)
        assert T.partner_offset(plan, plan["wait"] - 1e-6) == (0.0, 0.0)  # waits before lifting
        t_stop = plan["wait"] + plan["lift"] + plan["pause1"] + plan["carry"] * plan["frac"]
        for tau in (t_stop + 1e-6, t_stop + plan["stop"] - 1e-6):  # stops part way across, at the lifted height
            dy, dz = T.partner_offset(plan, tau)
            assert dy == pytest.approx(T.CARRY * plan["frac"], abs=1e-6) and dz == pytest.approx(T.LIFT)
        assert T.partner_offset(plan, 1e3) == T.partner_offset(plan, T.plan_length(plan))  # then holds still
    assert T.REST_Z == pytest.approx(T.TOP + T.V_VERTEX + T.PIPE_R * math.sqrt(2), abs=0.0005)


def test_history_resets_only_on_reset():
    plan, fs = frames()
    score, rec = run(plan, fs[:200])
    assert rec.values and rec.evers
    assert rec in lk._HISTORIES and T.H in lk._HISTORIES  # the kit clears these on every simulator reset
    rec.reset()
    assert not rec.values and not rec.evers and not rec.peaks and rec.t == 0.0


def test_slowest_person_fits_the_run_limit():
    slowest = sum(hi for name, _, hi in T.PHASES if name != "frac")
    assert slowest == pytest.approx(22.5)
    # the demo takes hold about 4.8 s in and is done about 6 s after the lowering ends
    assert (5.0 + slowest + 6.5) * T.CONTROL_HZ < 1200


def _stray(plan, blind, dt=0.05):
    """Largest gap (m, y or z) between the person's hand path and a path played blind on another timing."""
    ts = np.arange(0.0, max(T.plan_length(plan), T.plan_length(blind)) + 1.0, dt)
    return max(float(np.max(np.abs(np.subtract(T.partner_offset(plan, t), T.partner_offset(blind, t))))) for t in ts)


def test_a_blind_average_timing_strays_past_the_tilt_gate():
    """35 mm off over the 0.495 m from the demo's grips to the hand tips the pipe 4 degrees: the tilt gate. The
    simulator turns 0.73 to 0.91 of a path error into tilt (the arms smooth brief peaks), so 45 mm is a safe bound."""
    gate = 0.495 * math.tan(math.radians(T.TILT_MAX_DEG))
    assert 0.034 < gate < 0.035 and gate / 0.73 < 0.048
    dream_trial_seed = zlib.crc32(b"0:None:0")  # inspect-robots --seed 0, scene 0, epoch 0
    assert dream_trial_seed == 1415158438
    assert _stray(T.partner_plan(dream_trial_seed), T.MID_PLAN) > 0.060
    strays = [_stray(T.partner_plan(seed), T.MID_PLAN) for seed in range(400)]
    assert np.mean(np.array(strays) <= 0.045) < 0.06
