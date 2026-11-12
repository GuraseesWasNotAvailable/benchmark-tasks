# Give a patient's elbow one passive range-of-motion repetition

The robot carries a joint that has muscle tone through a set range and back while pinning the segment above it: it bends
a manikin patient's right elbow to a right angle, lays the forearm back flat on the bed and lets go, without the elbow
lifting off the bed or the forearm dropping. A child-size nursing manikin in a pale green gown lies face up on a
blue-grey bed pad with its right arm straight along the pad toward the robot's right arm. Its elbow has muscle tone, a
spring that pulls the forearm back toward straight. The natural one-handed carry fails: with nothing on the upper arm,
lifting the wrist swings the whole arm up at the shoulder, and the elbow leaves the pad as soon as the wrist starts to
rise (16 to 27 mm up by 3.5 to 6 deg of flexion, against a 15 mm limit). Three scripted one-arm versions all score 0:
the demo's arc with a fixed grip, the same arc with the gripper pitching with the forearm, and the arc with the hand
pressing the wrist 3 cm in along the forearm toward the elbow (elbow 10 mm up at 4 deg, 22 mm at 6 deg). Statics alone
does not rule out a cleverer one-handed force profile, so the claim is that the natural carry fails, not that one arm
can never do it. The way that works is the support-above-and-below technique taught to nursing assistants, home health
aides and physical therapist aides: one arm steadies the upper arm just above the elbow while the other carries the
forearm through the arc and back.

Nearest published tasks: Assistive Gym's ArmManipulation scoops a limp arm that hangs off the bed back onto the bed
beside the torso; RCareWorld's limb repositioning lifts an arm onto a wheelchair handle; Manip4Care moves a passive
limb, with no joint tone, to target poses for bed bathing. Each moves a limb to a place. None bends a toned joint to a
set angle and back with the segment above it held still, and none scores a lifted elbow, a dropped forearm or the
release.

Instruction given to the policy (26 words; it states the goal and never says to support the elbow):

`Give the training manikin's right elbow one passive range-of-motion repetition: bend it to a right angle, then straighten it back onto the bed and let go.`

Files in this folder: `task.json`, `task.py` (scene, history record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `demo_out/` (the demo's videos, `setup.png` and `result.json`),
`README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena 0.3.0, Isaac Sim 6.1, Isaac Lab 17.0.2, one L40S). Versions
used for the submitted runs:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 422602313b78 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in
  the log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the manikin as a PhysX articulation (USD revolute joints with limits and drives, spawned
through Isaac Lab's `ArticulationCfg`) and calls the task's per-step history record after every control step. No fork
of yam_arena and no third-party assets: every body is built from box and capsule colliders in `task.py`.

## Offline check, no simulator

No GPU or Isaac Sim needed, only the yam-arena package importable (after `source ~/IsaacLab-Arena/yam.env`, or
with its source on `PYTHONPATH`):

```bash
python -m pytest -q tests
```

29 tests feed synthetic angle, lift and gripper traces to the scorer: the demo's trace scores 1.0; peak 79 or 101 deg
scores 0.25 and 111 deg scores 0; a single 16 mm elbow lift scores 0 even after the elbow comes back down, and 14.9 mm
does not; a -95 deg/s fall from 40 deg is a drop (0) but the same rate starting at 10 deg is not; never back below 10
deg scores 0.5, and coming back down then rising again to 20 deg also scores 0.5; two full repetitions score 0.5, and so
does a second bend to 45 deg after the return, while a re-bend to 30 deg and back still scores 1.0; a gripper 25 mm from
the forearm, a tool point moving 6 mm/s in the last second, only 19 still steps, an end angle of 7 deg, the forearm held
8 mm above the pad or the elbow 6 mm up each score 0.75; the record keeps a violation (and a second repetition) until
reset and is clean after it; and every pose of the demo arc (0 to 120 deg) is reachable by `yam_arena.ik.solve` within
0.5 mm and 0.5 deg. The IK test is skipped where yam_arena's IK and robot model cannot load (29 passed on the
launchable).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Give the training manikin's right elbow one passive range-of-motion repetition: bend it to a right angle, then straighten it back onto the bed and let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/passive-elbow-rom-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                       # full-score demo
python linkdemo.py task.py cheat_out --variant one_arm    # a cheat variant (or DEMO_VARIANT=one_arm)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the upper arm's and forearm's poses, the elbow angle computed from those two poses, and the
robot's own gripper state. It never reads joint values or the history record.

1. Both arms together. The left gripper, tilted 45 deg toward the shoulder with its pads closing along the elbow
   axis, pinches the upper arm 6 cm above the elbow (46 mm on the 50 mm arm) and holds it on the pad. The right
   gripper comes straight down over the splint, pads closing along the elbow axis, and closes to 40 mm on its 50 mm
   flat sides, so the forearm can turn between the pads.
2. Flex. A reference angle ramps at 25 deg/s to 90 deg. The right tool point follows the arc about the elbow's resting
   centre (radius 18.5 cm), its approach pitching back by a third of the angle. The command leads the reference by an
   integral of the observed lag (up to 25 deg, about 10 deg at the top), and the ramp pauses while the observed angle
   lags by more than 8 deg. It holds 90 deg for 1 s (observed 89.4 deg).
3. Extend. 90 to 10 deg at 25 deg/s, then 10 to 0 deg at 10 deg/s, then 1.5 s at 0 deg.
4. Release. The right gripper opens, backs 10 cm out along its approach and rises 4 cm; the left gripper opens and
   backs 12 cm out. Both hold still.

Result: peak 94.1 deg, largest elbow lift 3.5 mm, steepest fall -35 deg/s, end angle 0.0 deg, pads 48 mm and 121 mm
clear. Success holds for 1 s and ends the episode at step 389 (19.5 s simulated; 75 to 180 s wall clock with start-up on
a quiet GPU, 430 to 495 s while five other simulations shared it); task_score 1.0, the same in all three runs of the
final task file. A reset afterwards (`--check-reset`) puts both joints back at 0 deg and at rest. The record clears on
reset in the simulator too: a development session that ran the demo and then every cheat in one simulator gave each
cheat the same 0 (the `nothing` run right after the demo read a peak of 0 deg, not 94). When several simulations share
one GPU, the RTX renderer can hand back an unrendered, uniform frame; `linkdemo.py` renders such a frame again, without
a physics step, before keeping it (`render` in `result.json` counts these; scoring is unaffected).

Cheat variants, each run in the simulator with `python linkdemo.py task.py out --variant <name>` and the files in this
folder:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | peak 0 deg | 0.0 |
| `one_arm` | The demo's arc with the right arm alone, no steadying pinch. | elbow 16 mm up by 3.5 deg of flexion, 45 mm at most (gate) | 0.0 |
| `one_arm_follow` | As `one_arm`, but the gripper pitches with the forearm, so the forearm never has to turn between the pads. | elbow 27 mm up by 5 deg, 31 mm at most (gate) | 0.0 |
| `one_arm_push` | As `one_arm`, but the wrist target sits 3 cm in along the forearm, so the hand presses the wrist toward the elbow all the way round. | elbow 10 mm up at 4 deg, 22 mm at 6 deg, 29 mm at most (gate) | 0.0 |
| `straight_lift` | The right arm pinches the splint and rises 12 cm straight up, nothing on the upper arm. | the straight arm swings up at the shoulder, elbow 55 mm up (gate) | 0.0 |
| `drop` | Two-arm flex to 90 deg, then the right gripper opens. | forearm falls at -189 deg/s (gate) | 0.0 |
| `overshoot` | Two-arm flex to 118 deg and back. | peak 120.6 deg (gate) | 0.0 |
| `side_push` | A closed right gripper pushes the splint sideways along the elbow axis. | peak 0 deg | 0.0 |

The three one-arm variants are the natural wrong plan: with nothing on the upper arm, the wrist force swings the
whole arm up at the shoulder before the elbow bends far, and the 15 mm lift gate trips within 5 s of the first move.
A second full repetition is not a cheat but a broken instruction; the scorer caps it at 0.5 (see Scoring and the
tests). Through the official CLI (`inspect-robots run ... --policy yam_scripted`, where the task file's `demo()` is
empty, so both arms hold the ready pose) the task loads, ends by `policy_stop` and scores 0.0, and the log's
`eval.environment_revision` equals the task file's hash (422602313b78).

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Rom` in `task.py`) reads the elbow angle theta (deg, 0 straight,
90 forearm vertical), the elbow lift (height of the upper arm's elbow-hinge point above its resting height), the
forearm's lowest collider point above the pad top, and each gripper's tool point and pad clearance. It keeps the
peak angle, the largest lift and the gate flags for the whole episode and is cleared only by a reset, so putting the
arm back never erases a violation. The rate is omega = (theta now - theta two steps ago) / 0.10 s.

- Gates, any time in the run, each gives 0: the elbow lifted more than 15 mm; theta past 110 deg; a drop (omega below
  -90 deg/s while theta two steps ago was at least 15 deg).
- 0.25: peak theta at least 45 deg.
- 0.5: peak theta between 80 and 100 deg (a right angle, never past 100).
- 0.75: and after first reaching 80 deg, theta came back to 10 deg or less and stayed there to the end, and it never
  bent again to 45 deg or more after that return (the instruction asks for one repetition; a second one caps the
  score at 0.5, and the flag is kept until reset like the gates).
- 1.0: and at the end theta is at most 5 deg, the forearm's lowest point is within 5 mm of the pad top, the elbow is
  lifted at most 5 mm, every sampled pad point and tool point of both grippers is at least 30 mm from the upper arm and
  forearm colliders, and for the last 20 control steps (1 s) both tool points moved slower than 5 mm/s and |omega|
  stayed under 3 deg/s.

`score()` returns the rung above, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: a protractor on a side photo taken along the
elbow axis (tape marks along the upper arm and forearm) for theta and its rate, and a ruler against a tape mark 45 mm
above the pad beside the elbow for lift.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo finishes in 389.
- Runtime on one L40S: the demo 75 to 180 s wall clock including Isaac Sim start-up (430 to 495 s when five other
  simulations share the GPU); a Dream-1 trial 3 min for 1200 steps (Dream round trips pause the simulator).
  The first run after boot adds a few minutes of warm-up.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (the manikin is never jittered).
  The scripted demo is deterministic; Dream-1 is a hosted model without a seed.

## Submitted videos and scores

- Dream-1 run: log `adhoc_f71b62cb.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 1200 steps.
- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`, `task_score = 1.0`, ended by `success` after
  389 steps. The video is `demo_out/top_cam.mp4`; the screenshot is the top camera 0.5 s into the demo, before the arms move (`ffmpeg -ss 0.5 -i demo_out/top_cam.mp4 -frames:v 1 setup.png`).

```bash
inspect-robots inspect adhoc_f71b62cb.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_f71b62cb.json
```

## Materials

- Bed pad: 64 x 66 x 2 cm blue-grey vinyl-covered closed-cell foam pad, about 0.7 kg, fixed to the table.
- Manikin: a child-size jointed nursing-skills manikin (PVC frame, vinyl skin) in a pale green cotton gown and pyjama
  trousers. Torso, head, left arm (straight along its far side) and legs (9 cm across, running off the pad's -y edge
  onto the table) are bolted to the table.
  Right arm: upper arm 5 cm across and 20 cm hinge to hinge, 0.25 kg; forearm 4.4 cm across, with a light blue padded
  resting-hand splint (thermoplastic shell, foam lining, 9 x 5 x 4 cm, flat sides) over the wrist and hand, forearm
  and splint 0.20 kg. The shoulder and elbow are hinges about one horizontal axis; the elbow carries a torsion spring
  (1.0 N m/rad toward straight, the muscle tone) and damping.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Lay the arm straight along its tape line (60 deg to the robot's forward axis), palm up, with the
elbow hinge over its mark and the splint's underside flat on the pad; both joints at 0 deg. Check the pad's near corner
sits on its marks. The torso, head, left arm and legs are bolted and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `linkkit.py` and `linkdemo.py` carry the full MIT text; `task.py` and
`tests/test_scoring.py` carry the SPDX identifier. This is our own code, written with public Isaac Lab and USD Physics
techniques; it copies no code from other tasks in this repository.
