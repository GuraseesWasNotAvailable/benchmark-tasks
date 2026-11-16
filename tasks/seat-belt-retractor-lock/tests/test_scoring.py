# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for seat-belt-retractor-lock, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record fed every control step (20 Hz) with the webbing pull-out e (m) and the holding hand's
pull coordinate h (m); every test feeds it synthetic traces.
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402,F401
import task as T  # noqa: E402

DT = 0.05
FAR = {"left": 0.2, "right": 0.2}
NEAR = {"left": 0.2, "right": 0.0}
STILL = {"left": 0.0, "right": 0.0}


class Run:
    """Feeds the Record. The right hand holds the tongue; the webbing follows the hand unless the lock has caught."""

    def __init__(self, lock=True):
        self.rec, self.t, self.e, self.h, self.lock, self.locked = T.Record(), 0.0, 0.0, 0.0, lock, False
        self.holding = True
        self.feed()

    def feed(self, clear=None, tcp=STILL):
        m = {"e": self.e, "clear": clear or (NEAR if self.holding else FAR), "tcp_speed": tcp,
             "hands": {"left": None, "right": self.h if self.holding else None}}
        self.rec.update(m, t=self.t)
        self.t += DT
        if self.lock and self.rec.lock_now():
            self.locked = True

    def pull(self, steps_mm):
        """Hand moves by each step (mm); the webbing follows unless locked."""
        for d in steps_mm:
            self.h += d / 1000
            if not self.locked:
                self.e += d / 1000
            self.feed()

    def steady(self, to=0.17, v=0.08):
        self.pull([v * DT * 1000] * round(to / (v * DT)))
        self.pull([0.0] * 10)

    def let_go(self, home=True, clear_s=1.2):
        self.holding, self.locked = False, False
        if home:
            self.e = 0.0
        for _ in range(round(clear_s / DT) + 2):
            self.feed(clear=FAR)

    def score(self):
        return T.ladder(self.rec)[0]


JERK = [9, 12, 15, 15, 6, 6]  # hand steps (mm): first step 0.18 m/s, then it keeps pulling 63 mm in all


def good(lock=True):
    r = Run(lock)
    r.steady()
    r.pull(JERK)
    return r


def test_good_run_scores_1():
    r = good()
    assert r.score() == 0.5
    r.let_go()
    assert r.score() == 1.0


def test_no_pull_is_0():
    r = Run()
    r.let_go()
    assert r.score() == 0.0


def test_steady_only_is_0():
    r = Run()
    r.steady()
    r.let_go()
    assert r.score() == 0.0


def test_loophole_jerk_then_stop_pulling_and_let_go_is_0():
    r = Run()
    r.steady()
    r.pull([9, 12])  # a real jerk, the belt locks, then the hand stops and lets go
    assert r.locked
    r.let_go()
    assert r.score() == 0.0


def test_loophole_belt_that_does_not_lock_is_0():
    r = Run(lock=False)
    r.steady()
    r.pull(JERK)  # nothing resists: the belt runs on with the hand
    r.let_go()
    assert r.score() == 0.0


def test_runon_over_15mm_is_0_but_15mm_passes():
    r = Run(lock=False)
    r.steady()
    r.pull([9, 6])  # belt moves 15 mm then the lock is made to hold
    r.locked = True
    r.pull([15, 15, 15])
    r.let_go()
    assert r.score() == 1.0
    r = Run(lock=False)
    r.steady()
    r.pull([9, 7])
    r.locked = True
    r.pull([15, 15, 15])
    r.let_go()
    assert r.score() == 0.0


def test_hand_must_keep_pulling_45mm():
    r = Run()
    r.steady()
    r.pull([9, 12, 12, 8])  # 41 mm
    r.let_go()
    assert r.score() == 0.0
    r = Run()
    r.steady()
    r.pull([9, 12, 12, 12])  # 45 mm
    r.let_go()
    assert r.score() == 1.0


def test_early_jerk_then_wound_in_and_a_good_attempt_still_scores():
    r = Run()
    r.pull(JERK)  # a jerk at once: locks the webbing at about 10 mm, a wrong first try
    r.let_go(clear_s=0.1)
    r.holding = True
    r.steady()
    r.pull(JERK)
    assert r.score() == 0.5
    r.let_go()
    assert r.score() == 1.0


def test_early_jerk_alone_is_0():
    r = Run()
    r.pull(JERK)
    r.let_go()
    assert r.score() == 0.0


def test_jerk_at_100mm_is_early():
    r = Run()
    r.steady(to=0.10)
    r.pull(JERK)
    r.let_go()
    assert r.score() == 0.0


def test_steady_pull_alone_does_not_trip_the_lock():
    r = Run()
    r.steady(to=0.17, v=0.08)
    assert not r.locked


def test_gentle_pull_at_0_12_ms_is_not_a_jerk():
    r = Run()
    r.steady()
    r.pull([6.0] * 20)
    assert not r.locked and not r.rec.att["onset"]


def test_a_hand_not_holding_the_tongue_is_not_a_jerk():
    r = Run()
    r.steady()
    r.holding = False
    r.pull(JERK)
    assert not r.rec.att["onset"]


def test_not_wound_in_is_05_and_5mm_home_tolerance():
    r = good()
    r.let_go(home=False)
    assert r.score() == 0.5
    r.e = 0.0051
    r.feed(clear=FAR)
    assert r.score() == 0.5
    r.e = 0.005
    for _ in range(30):
        r.feed(clear=FAR)
    assert r.score() == 1.0


def test_hand_29mm_from_tongue_restarts_the_release_timer():
    r = good()
    r.let_go(clear_s=0.0)
    for _ in range(30):
        r.feed(clear={"left": 0.2, "right": 0.029})
    assert r.score() == 0.75
    for _ in range(20):
        r.feed(clear={"left": 0.2, "right": 0.030})
    assert r.score() == 1.0


def test_hand_moving_6mm_s_is_not_still():
    r = good()
    r.let_go(clear_s=0.0)
    for _ in range(40):
        r.feed(clear=FAR, tcp={"left": 0.0, "right": 0.006})
    assert r.score() == 0.75


def test_release_needs_1s():
    r = good()
    r.let_go(clear_s=0.0)
    for _ in range(16):
        r.feed(clear=FAR)
    assert r.score() == 0.75


def test_reset_clears_the_record():
    r = good()
    r.let_go()
    r.rec.reset()
    assert r.rec.last is None and r.rec.bank == 0.0 and not r.rec.att["onset"]


def test_lock_trigger_is_an_acceleration_not_a_travel():
    rec = T.Record()
    for k, e in enumerate([0.0, 0.0, 0.0, 0.0085]):  # 8.5 mm in one step from rest: 0.17 m/s, a gain of 0.17
        rec.update({"e": e, "clear": FAR, "tcp_speed": STILL}, t=k * DT)
    assert rec.lock_now()
    rec = T.Record()
    for k, e in enumerate([0.0, 0.004, 0.008, 0.012]):  # a steady 0.08 m/s
        rec.update({"e": e, "clear": FAR, "tcp_speed": STILL}, t=k * DT)
    assert not rec.lock_now()
