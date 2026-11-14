# Move a newborn training manikin from the bassinet mattress to the changing pad with its head supported

The robot lifts a newborn training manikin off a bassinet mattress with both arms, carries it 21.5 cm to a blue
changing pad and sets it down on its back, then lets go. One arm holds the head at the ears and the other holds the
hips, which is the manikin's centre of mass below the neck. The neck is a passive hinge with a very weak spring, so a
head that is not held falls back to its stop. Nurses and nursing assistants (about 3.2 million and 1.3 million US jobs,
BLS May 2023) and parents do this move every day.

The ability is carrying a jointed body on two grips so that its passive neck stays inside a bend limit. Nothing is
posed, straightened or closed; the neck angle is set by where the two grips go together.

Physics, not the instruction, defeats the natural plan. Lifted by the hips alone, the 0.40 kg head hangs from the
neck and falls back to its 60 deg stop (the gate is 20 deg). Lifted by the head alone, the 2.1 kg body stays on the
mattress and the neck is pulled past 20 deg before the body leaves it. Only a head grip that follows the body, with
the hips grip doing the lifting, keeps the neck straight.

Instruction given to the policy (26 words; it states the goal only, never the neck limit):

`Move the newborn training manikin from the bassinet mattress onto the blue changing pad so it lies on its back on the pad, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the
demo and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: d6e0fb02ff0c (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the manikin as a PhysX articulation (a body link and a head link on a sprung revolute neck
with limits, spawned through Isaac Lab's `ArticulationCfg`) and the bassinet and pad as fixed compound bodies, and
calls the task's per-step record after every control step. No fork of yam_arena and no third-party assets: every body
is built from box and capsule colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

19 tests feed synthetic samples to the scorer: a demo-shaped trace scores 1.0 and doing nothing scores 0; the neck gate
is exact (20.0 deg passes, 20.1 deg is 0) and holds for the whole run, not just the end; the tilt gate at 30 deg for
pitch and roll; the lift needs head and body both 35 mm up (34.9 mm is 0); carried needs the centre line at or
under x 0.20 m; ending on the pad without ever lifting scores 0; the end neck limit (10.0 vs 10.5 deg), supine limit
(25 vs 25.5 deg), head resting on the pad and the body's set-down height window (-3 to +6 mm); the release gate
(29.9 mm fails, 30 mm passes, 0.9 s fails, 1.1 s passes); a moving gripper, manikin speed or spin blocks the 1.0 rung;
a late nudge restarts the timer; the record is clean after a reset.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move the newborn training manikin from the bassinet mattress onto the blue changing pad so it lies on its back on the pad, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/infant-manikin-transfer-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). With `object_jitter=0` the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                         # scripted demo
python linkdemo.py task.py cheat_out --variant left_only    # a cheat variant (or DEMO_VARIANT=left_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json`. The script reads only what a camera could measure: the body and head
poses and the robot's own grippers. It never reads joint values or the record.

1. Both arms, top-down with the pads closing across the body (along x): the left arm goes over the head and the right
   arm over the pelvis, and both descend (the head to its equator, the pelvis to 40 mm above the mattress) and close
   (the head to an 80 mm opening, a light squeeze; the pelvis to 40 mm, a firm one).
2. The right arm lifts 60 mm in 1.5 s, carries 21.5 cm toward the robot in 3 s and lowers to 3 mm above the pad in
   1.5 s. The loaded arm sags, so it servos its command on the seen body position (gain 0.05 per step). The left arm
   follows the seen body displacement through a low-pass filter, so the head goes where the body goes.
3. Both wait 0.5 s, open to 94 mm, wait, and retreat straight up 70 mm.

The demo scores 1.0 (`success`, episode ended at step 360 of 1,200): the neck peaked at 15.1 deg (gate 20), the body's
tilt at 4.4 deg, the lift reached 60 mm, and at the end the manikin lies on the pad (body 0 mm, head 0 mm above the
pad top, neck 0.0 deg, face up), still, with the grippers 43 mm and 38 mm clear for 1.95 s. The demo is deterministic
(two runs gave the same neck peak to 0.01 deg).

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (d6e0fb02ff0c); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. | 0.0 |
| `left_only` | Only the head arm acts (the hips arm stays at ready) and flies the whole lift, carry and set-down. | The head rises and the 2.1 kg body stays on the mattress, so the neck is pulled to its 60 deg stop: gate (60.0 deg). | 0.0 |
| `right_only` | Only the hips arm acts (the head arm stays at ready) and flies the whole path. | The body is lifted by the pelvis and the unheld head falls back: the neck reaches 34.7 deg, gate. | 0.0 |
| `slide` | Both arms grip as in the demo but lift only 8 mm and drag the manikin toward the pad. | The body catches on the 30 mm rim and rolls over it (roll 83 deg) while the neck reaches its 45 deg forward stop: gate (45.0 deg). | 0.0 |

## Build log (8 demo runs, 4 to 8 min each on the shared L40S)

| Run | Change | Result |
|---|---|---|
| 1 | first build; the body link's offset was applied twice | 0.0: the manikin spawned 0.33 m off its place and the head fell |
| 2 | body link at the tree origin; head toward the right arm | 0.0: the left arm, holding the pelvis, could not keep its wrist orientation at the near target |
| 3 | mattress 0.34 m out, pad 0.125 m, lift 60 mm | 0.0: the loaded arm sagged about 30 mm, so the head rose and the body did not (neck 41 deg) |
| 4 | roles swapped: head toward the left arm, hips on the right arm (which reaches both places) | 0.0: neck 37 deg from the same sag |
| 5 | head arm follows the seen body; firmer hips squeeze, body skin friction 1.3 | 0.0: tilt gate 38 deg (the squeeze was too hard and the body rolled) |
| 6 | squeeze eased; hips arm servos on the seen body, gain 0.3 | 0.0: the servo oscillated (rise 40 to 83 mm), neck 45 deg |
| 7 | gain 0.05, head follow low-passed | 1.0 at step 360 |
| 8 | final task file (left_only flies the whole plan, otherwise identical) | 1.0, same end state; all four variants 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures the neck
angle (the neck hinge, from the link poses), the body's pitch and roll from horizontal, how far face-up the body is,
the body's and head's height above where they lay on the mattress and above the pad, the centre line's x and y, the
body's speed and spin (from its pose change between control steps, what a camera sees), and each gripper's pad
clearance to the manikin and its speed. The record keeps peaks, first-true times and held times for the whole episode
and is cleared only by the simulator reset, so setting it down and starting again never clears a violation.

- Gate, any time in the run, gives 0: the neck bent more than 20 deg, or the body tilted more than 30 deg from lying
  flat (pitch or roll).
- 0.25: body and head both at least 35 mm above where they lay (the rim top is 30 mm).
- 0.5: and the centre line carried to x 0.20 m or less while lifted (past the rim, over the pad).
- 0.75: at the end lying on the pad: centre line within 45 mm of the pad centre and 60 mm of its start y, body bottom
  between -3 and +6 mm of the pad top, head centre within 8 mm of resting on it, back within 25 deg of up, neck within
  10 deg.
- 1.0: also the manikin still (under 5 mm/s and 3 deg/s) and every pad point of both grippers at least 30 mm from the
  manikin and moving under 5 mm/s, all for 1 s.

`distance()` is 1 - score. Real trial, with a 60 fps side video and a second from above: a protractor card at the neck
hinge (20 deg mark) and a 30 deg card for the body's tilt, read frame by frame; a ruler for the 35 mm lift; taped
outlines of the mattress line and the pad with a tape measure from the arm-base line for the 0.20 m carry and the
45 and 60 mm placement limits; 3 mm and 6 mm spacers under the body for the set-down window; a 30 mm spacer block
held against each gripper pad for the release gap, and a stopwatch on the video for the 1 s still.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 360.
- Runtime on one L40S: a recorded demo run took about 6 min wall clock, including about 2 min of Isaac Sim start-up
  (3 to 5 control steps/s). A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout. The scripted demo is
  deterministic; Dream-1 is a hosted model without a seed.

## Scored runs

| Run | inspect-robots-dream | yam-arena commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 0.1.0 | 0059d02a1a91 | d6e0fb02ff0c | 1200 | 8 min | adhoc_63072a19.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | - | 0059d02a1a91 | d6e0fb02ff0c | 360 (success held 1 s) | about 6 min | demo_out/result.json | 1.0 | lying on the pad, still, grippers 30 mm clear and still for 1 s; neck peak 15.1 deg |

- Demo: `python linkdemo.py task.py demo_out` writes `demo_out/result.json` with `task_score = 1.0`. The video is
  `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`), and the screenshot is `demo_out/setup.png`.
- Dream-1 run: made at filing with the command above, on this exact `task.py`.

```bash
inspect-robots inspect adhoc_63072a19.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_63072a19.json
```

## Materials

- Bassinet: grey ABS tray 0.26 x 0.60 x 0.02 m (3.0 kg, fixed, centred at x 0.34) with a pale-green foam mattress
  0.22 x 0.56 x 0.03 m (top 50 mm above the table), 30 mm side rims and end walls. A compact travel-bassinet size.
- Changing pad: blue foam 0.166 x 0.46 x 0.035 m (1.0 kg, fixed, centred at x 0.125), top 35 mm above the table,
  15 mm below the mattress, touching the rim's outer face.
- Newborn training manikin: skin-tone vinyl, 2.5 kg, crown to heels 0.42 m with the legs flexed. Body link 2.10 kg
  (chest 95 x 100 x 85 mm, pelvis 75 x 80 x 60 mm, legs 140 x 110 x 50 mm), centre of mass in the pelvis, friction
  1.3 / 1.2. Head 0.40 kg, an 85 mm wide capsule 120 mm long, friction 0.8 / 0.7, on a revolute neck hinge across x
  130 mm from the crown (stops 45 deg forward and 60 deg back, spring 0.03 N m/rad, damping 0.003 N m s/rad). Every
  grasp is 75 to 85 mm across, inside the 94 mm gripper opening; the hips grip holds 2.1 kg (about 21 N) with the
  fingers at or under 50 N, the head grip 0.4 kg.
- Layout chosen for reach: the head and pelvis grips are 0.34 m out from the arm bases and the set-down 0.125 m out,
  with the head toward the left arm and the hips on the right arm, which reaches both places with the pads closing
  across the body.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Lay the manikin on its back on the mattress along the taped line, crown 0.20 m left of the
centre, centre line on the mattress centre, neck straight, head resting on the mattress, arms at its sides. The
bassinet and the pad are fixed on taped outlines and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
