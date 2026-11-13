# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for drone-prop-handedness, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer reads the whole-run record (a linkkit History); the tests fill it the way the on-step callback does.
"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

NAMES = T.NAMES


@pytest.fixture(autouse=True)
def fresh():
    T.H.reset()
    yield
    T.H.reset()


def finish(tight=NAMES, q=0.0038, wrong=0.0, release=2.0, wrong_on=()):
    for n in NAMES:
        T.H.set(f"q.{n}", q if n in tight else 0.0)
        T.H.peak(f"wrong.{n}", wrong if n in wrong_on else 0.0)
    T.H.runs["release"] = release


def test_all_tight_and_released_is_one():
    finish()
    assert T.ladder()[0] == 1.0 and T.success(None)


def test_all_tight_but_not_released_is_point_eight():
    finish(release=0.0)
    assert T.ladder()[0] == 0.8 and not T.success(None)


def test_release_must_hold_one_second():
    finish(release=0.95)
    assert T.score(None) == 0.8
    finish(release=1.0)
    assert T.score(None) == 1.0


@pytest.mark.parametrize("n", [0, 1, 2, 3])
def test_ladder_is_a_fifth_per_tight_prop(n):
    finish(tight=NAMES[:n])
    assert T.score(None) == pytest.approx(0.2 * n)
    assert not T.success(None)


def test_nothing_done_scores_zero():
    finish(tight=())
    assert T.score(None) == 0.0


def test_tight_threshold_is_3_6_mm():
    finish(q=0.0036)
    assert T.score(None) == 1.0
    finish(q=0.0035)
    assert T.score(None) == 0.0


def test_a_prop_not_screwed_all_the_way_is_not_tight():
    finish(q=0.0030)
    assert T.score(None) == 0.0


def test_wrong_way_gate_is_half_a_turn():
    finish(wrong=179.9, wrong_on=("fl",))
    assert T.score(None) == 1.0
    finish(wrong=180.0, wrong_on=("fl",))
    assert T.score(None) == 0.0


def test_gate_beats_a_finished_quad():
    finish(wrong=300.0, wrong_on=("br",))
    assert T.score(None) == 0.0 and not T.success(None)


def test_gate_applies_to_any_one_motor():
    for n in NAMES:
        finish(wrong=200.0, wrong_on=(n,))
        assert T.score(None) == 0.0, n


def test_gate_holds_even_with_no_prop_tight():
    finish(tight=(), wrong=400.0, wrong_on=("fr", "bl"))
    assert T.ladder()[0] == 0.0 and "gate" in T.ladder()[1]


def test_reset_clears_the_record():
    finish(wrong=300.0, wrong_on=("fl",))
    T.H.reset()
    finish()
    assert T.score(None) == 1.0


def test_two_motors_each_way_on_the_diagonals():
    s = {n: sign for n, _, _, sign in T.MOTORS}
    assert sorted(s.values()) == [-1, -1, 1, 1]
    assert s["fl"] == s["br"] == 1 and s["fr"] == s["bl"] == -1


def test_motor_positions_are_a_square_about_the_centre():
    for n, x, y, _ in T.MOTORS:
        assert abs(abs(x - T.CX) - T.ARM) < 1e-9 and abs(abs(y - T.CY) - T.ARM) < 1e-9


def test_thread_geometry():
    assert T.STOP_DEG == pytest.approx(240.0)  # 4 mm travel at 6 mm lead
    assert T.TIGHT_M < T.TRAVEL
    assert T.STOP_DEG + T.TIGHT_EXTRA_DEG > T.STOP_DEG
    assert T.FREE_DEG > T.WRONG_GATE_DEG  # a wrong-way turn can pass the gate before the hub jams


def test_hub_clears_the_bell_only_at_the_thread_start():
    assert T.HUB_Z0 - T.HUB_H / 2 == pytest.approx(T.BELL_TOP + T.TRAVEL)


def test_neighbouring_props_do_not_overlap():
    radius = T.HUB_AF / 2 + 0.043
    assert 2 * T.ARM > 2 * radius
