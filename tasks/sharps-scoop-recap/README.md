# Recap a used practice syringe with the one-handed scoop, the cap lying on its tray, the other hand kept clear

The robot puts a loose needle cap back on the needle of a used practice syringe with one arm: the cap lies on a tray
and the needle is scooped into its mouth and pushed home, while the other arm stays out of a taped zone and never
touches the cap. At the end the cap is seated on the needle (the barrel's front shoulder at the cap's mouth, axes
aligned), the syringe is still, and both grippers have let go. Nurses, medical assistants and phlebotomists are taught
this when a used needle has to be recapped and there is no sharps container at hand, because the hand that steadies a
cap is the hand that gets stuck.

The ability is placing a thin rigid needle into a loose, light, open-mouthed cap from a held syringe, with the cap held
only by its tray and a back stop, and one hand forbidden from helping. The cap is a 7.6 mm bore for a 6.6 mm hub and a
1.27 mm needle, so the tip must find the mouth within a few millimetres and enter it before the syringe levels, and a
pad touching the cap or the free arm entering the taped zone ends the run at 0. The natural two-handed plan (one
gripper pins the cap, the other pushes the syringe) is exactly what the gates forbid.

Instruction given to the policy (20 words; it states the goal only, never the technique or the rules):

`Put the pink cap that lies on the tray back onto the needle of the practice syringe, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner), `tests/test_scoring.py` (scorer
edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo and
cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 1.0.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: 01017917c0cb (`sha256sum task.py | cut -c1-12`; equals
  `eval.environment_revision` in the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the bodies (the syringe and the cap as compound rigid bodies, the tray and the rest as fixed
bodies) and calls the task's per-step record after every control step. No fork of yam_arena and no third-party assets:
every body is built from box and cylinder colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests        # needs yam_arena importable (linkkit imports it); no simulator is started
```

31 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0 and doing nothing scores 0; an off-hand pad
over the zone gates the run for good, even with the cap seated afterwards, but only once someone holds the syringe (the
holder is the first arm to close on it, either arm); a pad within 3 mm of the cap gates either hand; the 0.25 rung needs
a held syringe and a tip in the bore (5.0 vs 4.9 mm deep, 3.5 vs 3.6 mm off the axis); the 0.5 rung (30 mm and 15 deg
vs 29.9 mm and 15.1 deg); the seat limits (3.0 vs 3.1 mm gap on either side of the mouth plane, 3.0 vs 3.1 mm off the
axis, 8.0 vs 8.1 deg) and a seated shoulder with the tip outside the bore; seated while still held scores 0.75; 0.95 s of
release is not enough, 3 s is, and a pad at 29 mm or a hand at 6 mm/s blocks it; the syringe must be still for 2 s
(1.9 s is 0.75, 2.1 s is 1.0), judged from pose change; a cap pulled off again drops back to 0.5; the record is clean
after a reset; the zone edges; the demo's pose builder puts the tip where asked at the asked pitch; and measure() on real
poses reads the needle's depth, the shoulder gap and the angle.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the pink cap that lies on the tray back onto the needle of the practice syringe, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/sharps-scoop-recap-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with
`termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). With `object_jitter=0` the seed changes nothing in the scene.

Dream-1 run (made at filing, on this exact `task.py`): Dream-1 beta 1.0.0, commit 0059d02a1a91, task SHA-256 prefix 01017917c0cb,
1200 steps in 8 minutes, log adhoc_3ac215f4.json, task_score 0.0, with this reason: max_steps.

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                            # scripted demo
python linkdemo.py task.py cheat_out --variant left_hovers     # a cheat variant (or DEMO_VARIANT=left_hovers)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (add `--no-record` to skip the videos). The script reads only what a
camera could measure: the syringe and cap poses and the robot's own grippers. It never reads joint values or the record.

1. The right arm goes top-down over the barrel's middle (pads across it, 9 mm of the pads' contact centre below the TCP,
   slot beneath), closes to 6.5 mm (3 mm squeeze), lifts 50 mm and records the grasp from the seen syringe pose.
2. It tilts the needle 20 deg tip down and brings the tip 12 mm short of the mouth, 2.5 mm above the bore axis, servoing
   the seen tip against the seen cap (the arm sags a millimetre or two).
3. It scoops: over six waypoints the tip goes 12 mm in front of the mouth, through it at 18 deg, then 10, 20, 30 and
   40 mm in while the pitch falls 14, 8, 3 and 0 deg, so the barrel comes level as the tip goes deep. After every
   waypoint the cap's seen pose is re-read and the position correction is nudged (capped at 3 mm).
4. It pushes the level syringe in 2.5 s, then creeps on from the seen shoulder-to-mouth gap until it is under 0.8 mm (the
   arm lags 2 to 3 mm under load), lets go (pads open to 94 mm) and backs away 70 mm; the left arm stays at the ready pose.

The demo scores 1.0 (`success`, episode ended at step 618 of 1,200): cap seated (shoulder 0.01 mm from the mouth plane, tip
0.2 mm off the axis, 0.15 deg), syringe still, the right pads 73 mm from the syringe, the left gripper 381 mm away. Worst
IK error 0.09 mm and 0.01 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file; every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; the syringe stays in its rest. | 0.0 |
| `left_hovers` | The normal right-arm scoop, and the left arm reaches over the cap's closed end without touching it. | Gate (a) at 6.25 s, once the right arm held the syringe (the left arm is at its reach limit and the run is disturbed after that). | 0.0 |
| `left_pins_cap` | The normal scoop, and the left arm reaches down onto the cap's far end to pin it. | Gate (a) at 6.25 s, before the left gripper reaches the cap. | 0.0 |
| `drop_on_cap` | The right arm carries the syringe level over the cap and lets go from 30 mm. | The syringe falls and ends with its tip 29 mm in the bore but never held there, so no rung is earned. | 0.0 |

The left arm cannot actually touch the cap in the simulator (it is at its reach limit there), so gate (b), a pad within
3 mm of the cap, is covered by the pytest cases only.

## Build log (6 simulator runs after a failed load)

| Run | Change | Result |
|---|---|---|
| 0 | first file | refused to load: a 1 g cap is under the kit's 3 g minimum link mass, so the cap is 3 g |
| 1 | `nothing` | everything at rest, nothing drifts; the left gripper 381 mm from the zone, the right in it (it is the holder when it works) |
| 2 | first demo | 1.0 at step 586, but the arm lagged 4 mm under load and left the shoulder 2.5 mm from the mouth (limit 3.0) |
| 3 | creep on until the seen gap is under 0.8 mm | 1.0 at step 618, shoulder 0.01 mm from the mouth plane |
| 4 to 6 | `left_hovers`, `left_pins_cap`, `drop_on_cap` | 0.0 each, as above |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures, in the cap's
frame (u along the bore from the mouth centre, mm): the needle tip's depth and its lateral offset from the bore axis, the
barrel's front shoulder's gap to the mouth plane and its lateral offset, the angle between needle and bore, which
gripper holds the syringe (opening 60 mm or less with a pad within 4 mm), each gripper's pad points over the taped zone
or within 3 mm of the cap, the pad clearance to the syringe and cap, and speeds from pose changes between control steps.
The record is cleared only by the simulator reset.

- Gate (a), any time in the run, gives 0: a pad of the off hand (the arm that did not first close on the syringe) over the
  taped zone (x 0.05 to 0.36 m, y -0.30 to +0.03 m) once the holder holds.
- Gate (b), any time in the run, gives 0: a pad of either gripper within 3 mm of the cap.
- 0.25: the held needle's tip went at least 5 mm into the bore (3.5 mm or less off its axis).
- 0.5: it went 30 mm in with the axis within 15 deg of the bore.
- 0.75: at the end the cap is seated: tip in the bore, shoulder within 3 mm of the mouth plane and 3 mm off the axis, angle
  at most 8 deg.
- 1.0: seated at the end, the syringe under 3 mm/s and 3 deg/s for 2 s, and the release gate: every pad point of both
  grippers at least 30 mm from the syringe and cap and both grippers under 5 mm/s for the last 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial, all by video: a top camera on the taped rectangle
for gate (a); a side camera for gate (b) and the 5 mm ink ticks on the translucent cap for the 5 and 30 mm tip depths; a
ruler on the shoulder-to-mouth gap (3 mm) and a protractor on a top frame for 8 deg; a 30 mm spacer block for the
release; frame differencing for stillness.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 618.
- Runtime on one L40S shared with another build: a demo run took a few minutes plus about 2 min of Isaac Sim start-up.
- Seeds: `--seed 0` for every submitted run; `-E object_jitter=0` fixes the layout. The scripted demo is deterministic.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 618.
- Dream-1 run: see the line above (1.0.0, 0059d02a1a91, 01017917c0cb, 1200, 8, adhoc_3ac215f4.json, 0.0, max_steps).

## Materials

- Cap tray: grey ABS, 0.5 kg, fixed. A 70 x 70 x 20 mm block with two 49 x 2 x 5 mm rails 10.8 mm apart and a 20 x 4 x
  14 mm back stop 1 mm beyond the cap's closed end; a 20 x 70 x 20 mm rear block carries the barrel's rear end; a 28 mm
  slot between them lets the pads reach under the barrel. Friction 0.45 / 0.40.
- Needle cap: translucent pink polypropylene, 3 g (a real 18 G cap is about 1 g), an octagonal tube 9.6 mm across the
  outer flats, 1.0 mm walls, 7.6 mm across the bore flats, 49 mm bore, closed by a 1.5 mm plate, 5 mm ink ticks. To
  check a real cap: a 6.9 mm pin gauge enters the full bore and an 8.0 mm gauge does not.
- Practice syringe: 3 mL, 6.5 g, clear plastic: a 9.5 mm barrel 62 mm long, a 24 mm finger flange, a 6.6 mm hub 10 mm
  long and a steel 18 G needle (1.27 mm, 38 mm), 48 mm from shoulder to tip. A practice (training) syringe is used, not a
  clinical one.
- Syringe rest: grey ABS, two 20 x 40 x 20 mm blocks with 6 mm lips either side of the barrel (11 mm apart) and a 28 mm
  slot.
- Taped zone: red tape on the table, x 0.05 to 0.36, y -0.30 to +0.03 (310 x 330 mm).
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Lay the cap between the rails, mouth toward the robot's right, with a 1 mm feeler between its closed
end and the back stop. Lay the syringe in its rest, needle toward +y, flange against the rear block, shoulder at y -0.155.
The tray, rest and zone tape are fixed and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
