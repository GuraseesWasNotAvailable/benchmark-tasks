# Test that a seat belt retractor locks: pull the webbing out steadily, then jerk it

The robot does the test everyone has done with a car seat belt. It pulls the webbing out 150 mm at a steady pace (the
retractor must pay out freely), jerks it (the retractor must lock within 15 mm of where the jerk started), keeps
pulling against the locked belt (the lock must hold, not merely stop because the hand stopped), then lets go so that the
lock frees and the spring winds the webbing back in. The tongue plate is the handle; the retractor is bolted to a steel
test plate on the table.

Category: Societal relatability. Pulling a seat belt out slowly and feeling it glide, then yanking it and feeling it
lock and refuse to give however hard you keep pulling, is a familiar experience for anyone who has ridden in a car;
drivers and garages repeat it on purpose after a repair, a recall or a used-car purchase.

The ability is controlling the speed and the acceleration of a pull, and then keeping the pull on: the retractor's lock
is sensitive to how suddenly the webbing is pulled out, so the same pull at two speeds gives two different outcomes,
and only a belt that holds against the continued pull has really locked. Nothing is placed or fastened.

Physics, not the instruction, defeats the natural plan. A policy that pulls the first 15 cm abruptly (or begins with the
jerk) locks the webbing before 150 mm have come out, and that attempt is wasted (a wrong first try that is let go and
wound in costs nothing, the next attempt counts). A policy that jerks too softly never locks the webbing and it simply
runs on. A policy that jerks, sees the belt stop and lets go has not shown a lock: the hand must keep pulling.

Instruction given to the policy (23 words; it states the goal only, never the speeds):

`Pull the seat belt webbing out 15 cm, then jerk it so the retractor locks, then let go so it winds back in.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the
submitted runs:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: bf13f3a1c7eb (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the retractor as one fixed PhysX articulation (a frame link and a webbing link joined by a
spring-damper prismatic joint whose drive is switched to a stiff hold by the lock, which here engages on webbing acceleration), built from box colliders in
`task.py`. No fork of yam_arena and no third-party assets.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

19 tests feed synthetic hand and webbing traces (20 Hz) to the scorer: a steady pull, a jerk and a continued pull then
letting go scores 1.0; no pull and a steady pull alone score 0; the loophole (a real jerk, the belt locks, the hand stops
and lets go) scores 0; a belt that does not lock scores 0; a run-on of 15 mm passes and 16 mm does not; a hand that pulls
41 mm past the jerk start scores 0 and 45 mm passes; a jerk at once or at 100 mm is early and scores 0, but a wrong first
try that is let go and wound in does not spoil a later good attempt; a steady pull and a gentle 0.12 m/s pull never trip
the lock; a hand that is not holding the tongue is not a jerk; the 5 mm wound-in tolerance and the release gate; and the
lock trigger is an acceleration (8.5 mm in one step from rest trips it, a steady 0.08 m/s does not).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pull the seat belt webbing out 15 cm, then jerk it so the retractor locks, then let go so it winds back in." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/seat-belt-retractor-lock-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

## Scored runs

| Run | yam-isaac-lab-arena | Commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 1.0.0 | 0059d02a1a91 | bf13f3a1c7eb | not run | not run min | dream_log.json | not run | not run |
| Scripted demo (`linkdemo.py`) | git checkout | 0059d02a1a91 | bf13f3a1c7eb | 277 (success held 1 s) | not timed | result.json | 1.0 | steady pull to 165 mm at 0.08 m/s, jerk locked at 179 mm (11 mm run-on), hand kept pulling 65 mm against the lock, let go, wound back in, grippers clear |

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant stop_release   # a cheat variant (or DEMO_VARIANT=stop_release)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the webbing link's pose (the pull-out) and the robot's own grippers. It never reads the
lock state, the joint values or the record.

1. Right arm, top-down, pads closing across the tongue's 45 mm width at its slot end (30 mm from the tongue's centre,
   so the pads have room to slip toward its free end), opened 75 mm on the approach, lowered until the pad tips are
   3.5 mm into the tongue's 4 mm edges, closed to 43 mm.
2. Steady pull toward the robot at 0.08 m/s (4 mm a control step) until the seen pull-out passes 165 mm; holds 0.6 s.
3. The jerk: the hand moves 12, 18 and 18 mm in three control steps, then 6 mm a step three more times (66 mm in all).
   The first step already trips the lock; the webbing stops about 11 mm after the jerk started and does not move again
   while the pads slip on the locked tongue and the hand goes on pulling.
4. Opens the gripper (the lock frees when no hand holds the tongue; the spring winds the webbing in), lifts 80 mm and
   moves 100 mm sideways, then waits.

The demo scores 1.0 (`success`, episode ended at step 277 of 1,200): jerk onset read from the hand, the webbing stopped
11 mm after the jerk started, the hand pulled 65 mm against the locked belt, the webbing wound back in to 0 mm, both
grippers at least 30 mm clear and still for 1.95 s.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (bf13f3a1c7eb); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. | 0.0 |
| `slow_only` | Steady pull to 169 mm, hold, let go. | The webbing never locks; it winds back in. | 0.0 |
| `jerk_only` | The jerk straight away, then on, then let go. | A jerk before 150 mm had come out steadily: early, no good attempt. | 0.0 |
| `short_slow` | Steady pull to 80 mm, then the jerk. | Early jerk at 80 mm. | 0.0 |
| `soft_pull` | Steady pull to 165 mm, then on to 240 mm at 0.12 m/s. | Never a jerk (no 0.10 m/s gain in a step), the belt never locks and runs on. | 0.0 |
| `stop_release` | The old full-score loophole: steady pull, a real jerk (12 mm then 12 mm) that locks the belt, then the hand stops and lets go. | The belt locks and winds back in, but the hand pulled only 24 mm past the jerk start, so nothing showed that the lock holds: no good attempt. | 0.0 |
| `left_only` | The left arm does the demo and the right arm stays home. | The left arm cannot reach the tongue at y -0.12 (IK fails); nothing is pulled. | 0.0 |

The task is a one-hand job (the retractor is bolted down), so the demo is the right arm alone; a `right_only` variant
would be the demo itself.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step the on-step record (`Record` in `task.py`) measures the webbing's pull-out e (mm, 0 with the tongue
against the slot), the pull coordinate h (mm) of any gripper holding the tongue (pads within 4 mm of it, opened under
60 mm), and each gripper's pad clearance to the tongue, webbing and retractor and its speed.

- Steady pull: e reaches 150 mm with the webbing speed over 0.1 s under 0.20 m/s the whole way.
- Jerk onset (read from the hand, not from belt travel): the first control step (0.05 s) in which the tongue-holding
  hand moves at 0.15 m/s or more and has gained 0.10 m/s in that step (about 2 m/s2). e_on is e one step earlier. A
  stationary belt therefore only has to lock within 15 mm of where the hand started, not after 30 mm of travel.
- Good attempt: the steady pull was complete before the jerk onset; after the onset e never exceeds e_on + 15 mm; and
  the hand, still holding the tongue, keeps pulling to at least 45 mm past its jerk start (the pads slip on the locked
  tongue, so the lock resists a continued pull instead of merely stopping when the hand stops).
- An attempt ends when the webbing is back home (5 mm or less); the best finished attempt counts, so a wrong first try
  that was let go and wound in does not zero the run. Nothing is permanently gated.
- Ladder: 0 without a good attempt (including jerk, stop pulling, let go); 0.5 a good attempt; 0.75 and the webbing
  wound back in to 5 mm or less; 1.0 and both grippers at least 30 mm from the tongue, webbing and retractor and moving
  under 5 mm/s for the last 1 s (the release gate).

Real-trial measurement line: film from above at 60 fps with a 0.5 m steel rule beside the webbing and a 10 mm dot on the
gripper. Read e at the slot from the printed ticks (every 5 mm; interpolate by eye) and h from the dot against the rule.
Over three frames (0.05 s) the hand speed and its gain give the jerk onset; the 15 mm run-on is the tick count past e_on
and the 45 mm continued pull is the dot's travel while the tongue stays on its tick. Before the trials, hang a hook
spring scale on the tongue and pull a yanked, locked belt to 20 N (50 N for the real unit): the tongue must move less
than 2 mm. The release gate is the pads' distance from the tongue on the rule and the hand's speed over 6 frames.

## Setup and materials

- Retractor: a passenger-car emergency-locking retractor, steel frame 90 x 70 x 110 mm, dark grey painted steel,
  0.5 kg, bolted with four M8 bolts to a 200 x 160 x 8 mm grey steel test plate that is clamped to the table. Its
  webbing slot faces the robot, 60 mm above the table. Webbing-sensitive (flywheel) lock, tuned on the bench so that a
  0.08 m/s pull runs free and a yank (0.15 m/s reached in one 0.05 s step from rest, about 2 m/s2) catches within 15 mm and then holds a 20 N pull with under 2 mm of travel (verify with the video and spring-scale procedure above).
- Webbing and tongue: 46 mm wide, 1.2 mm thick black polyester webbing ending in a silver steel tongue plate
  85 x 45 x 4 mm (120 g); 100 mm of webbing plus the tongue lie outside the slot at rest (the tongue centre at
  (0.408, -0.12, 0.808)); 0.15 kg in all. White ticks every 5 mm are printed along the webbing from the slot.
- Return spring: 0.2 N at rest rising to about 2 N at 0.2 m of pull-out (check with a 5 N spring scale at 0 and
  0.2 m), with a light damper so that the webbing winds in at about 0.2 m/s.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

Fidelity: the webbing is a rigid 1.2 mm blade 0.40 m long on a slider with 0.30 m of pull-out (its tail sticks out
behind the frame where a real belt winds on a spool). The lock is a stiff brake that holds the slider from the step after the pull-out speeds up by 0.10 m/s to 0.15 m/s or more in one 0.05 s step (real units trip on webbing acceleration) and frees when no hand holds the tongue.

## Reset between trials

Nothing is consumed. Let the webbing wind fully in (tongue against the slot, ticks at 0), check the lock is free by
one slow hand pull of 100 mm (it must run free) and one yank of 20 mm (it must catch and hold the 20 N scale pull), then let go and let it
wind in. Re-seat the tongue and check the spring with the scale at 0.2 m (about 2 N). The test plate stays clamped on
its taped outline.

## Revision

Changes made after the reviewer's first read of this task:

| Reviewer point | What changed |
|---|---|
| Importance: category and the familiar experience | The category is now Societal relatability, and the context describes the familiar experience of a seat belt gliding on a slow pull and locking when yanked. |
| Construct validity: jerk, stop pulling and release earned full credit | A good attempt now needs the hand to keep pulling at least 45 mm past the jerk start while the belt stays within 15 mm of e_on, so a belt that holds against a continued pull is required. The `stop_release` cheat (a real jerk, then stop and let go) scores 0. |
| Scoring: jerk timed only after 30 mm of belt travel over 0.1 s | Jerk onset is read from the gripper: 0.15 m/s over one 0.05 s step with a 0.10 m/s gain, and the run-on is counted from e one step before. The simulated lock trips on that same acceleration, so a belt that locks within 15 mm of a stationary start passes. |
| Practical real-world measurement and full-score demonstration | The real-trial line uses 60 fps video, a steel rule, the printed ticks, a dot on the gripper and a hook spring-scale check of the lock. The scripted demo is re-run and scores 1.0. |
| Our own checks | A wrong first try that is let go and wound in no longer zeroes the run (the old permanent early-jerk gate is gone); only the best finished attempt counts. |

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.

## Run logs

The demo and cheat results above were run in Isaac Sim on the program's run box with this task.py (SHA-256 prefix bf13f3a1c7eb). Their logs stayed on that box, which ran out of compute credits before they could be copied, so no log is committed with this revision. The scripted demo is deterministic: `python linkdemo.py task.py demo_out` reproduces it. The videos linked from task.json show the previous revision.
