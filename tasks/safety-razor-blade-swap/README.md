# Change the blade in a butterfly safety razor, holding the blade only by its edges

The robot opens a twist-to-open safety razor, sets a steel razor blade over the two posts of its head, lets the
doors close on it, and lets go, without a gripper ever touching the blade's sharp long edges. The razor is a
butterfly (twist-to-open) double-edge razor in a weighted stand, and the blade is a standard 43 x 22 mm double-edge
blade. At the end the blade lies on the head plate with both posts through its holes, both doors are closed on it,
the knob is back at zero and both grippers have let go. Anyone who wet-shaves with a safety razor does this every
few shaves, and barbers do it between customers.

The ability is a two-hand sequence around a thin, sharp part: one hand has to hold a spring-loaded knob turned (the
doors close the moment it lets go) while the other carries a 0.4 mm steel plate by a 4 mm tab of its short end,
between the pads, and threads two 4.4 mm holes onto 2.8 mm posts that stand only 1.6 mm proud. The blade is never
picked up by its long sides. Nothing is a puzzle: the work is holding a thin plate by a few millimetres, centring it
to under a millimetre, and timing the knob release.

Physics, not the instruction, defeats the natural plan. The doors are shut at the start and the knob springs back, so
the blade cannot go on until one arm has turned the knob and keeps it turned. A blade lowered 1 mm off-centre rests
on the post tops (1.2 mm above the seat), and the doors then stall on it, open, instead of closing flat. The pads
on a 22 mm wide blade are wide enough that gripping it anywhere but on the centre line of a short end puts a pad over
a cutting edge, and the whole-run gate scores that 0 even if the blade is then seated perfectly.

Instruction given to the policy (21 words; it states the goal only, never the edge rule):

`Put the new blade into the butterfly razor so it sits on the posts under the closed doors, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo
and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: c11b3914c2b9 (`sha256sum task.py | cut -c1-12`; for a Dream-1 log it equals
  `eval.environment_revision`)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the razor as a PhysX articulation (a fixed stand, handle and head plate with a sprung
revolute knob and two sprung revolute doors, USD joints with limits and drives, spawned through Isaac Lab's
`ArticulationCfg`), and the blade and its shelf as rigid bodies, and calls the task's per-step record after every
control step. No fork of yam_arena and no third-party assets: every body is built from box and cylinder colliders in
`task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

35 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; nothing scores 0; a blade carried with
the doors at 59 deg open scores 0 and at 60 deg scores 0.25; 19.9 mm of lift is no carry and 20.0 mm is; a carry needs
a gripper closed on the blade; seated limits (1.5 vs 1.6 mm off centre in x and y, 5.0 vs 5.1 deg of yaw, 3.0 vs 3.1
deg of tilt, a gap under the blade of 0.6 vs 0.7 mm; a blade perched on the post tops is not seated); doors stalled
2.1 deg open or a knob left at 15.1 deg cap the score at 0.5, and 2.0 and 15.0 pass; a blade that was never seated with
the doors open cannot pass by being carried in under shut doors; blade speed 1.5 mm/s or spin 2.1 deg/s at the end
scores 0.75; 0.95 s of release is not enough and 1.0 s is; one gripper step at 11 mm/s restarts the release timer; 29
mm of clearance is not enough and 30 mm is; the cutting-edge gate beats a perfect finish and is not cleared by starting
again; the record is clean after a reset; the edge test on real `Tool` objects (a pinch on the centre line is clean,
a 12 mm pad footprint reaching the outer 3 mm is a touch at 2.1 mm off centre and not at 1.9 mm, pads 94 mm apart or
10 mm short of the blade or 30 mm beside it are no touch, a yawed blade turns a centred pinch into a touch); and the
blade's pose in the head, the pose-change speed and the 0.8 mm hole-to-post clearance.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the new blade into the butterfly razor so it sits on the posts under the closed doors, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/safety-razor-blade-swap-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant edge_grip     # a cheat variant (or DEMO_VARIANT=edge_grip)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: object and link poses (the knob and door angles are computed from link poses) and the
robot's own grippers. It never reads joint values or the record.

1. Left arm: the hand 15 deg down, pads across the knob's 22 mm (pinch 20 mm commanded), turns the knob 92 deg
   clockwise as one rigid motion about the handle axis, corrects from the seen knob angle (91 deg), and holds it.
2. Right arm, in parallel: comes in from above and 20 mm short, runs in along x with the pads closing vertically and the
   pad tips 4 mm over the blade's near short end (centre line), closes, lifts 45 mm, carries the blade over the head
   at 20 mm and servoes the hand-and-blade assembly rigidly on the seen blade pose (centre and yaw).
3. Right arm waits until the doors are seen open (100 deg), drops to 4 mm above the post tops, servoes again at
   2.5, 1.2 and 0.4 mm, steps down 0.4 mm at a time until the blade is seen on the plate, opens and backs off 60 mm.
4. Left arm eases the knob back over 3 s so the doors close on the blade, opens, backs off 70 mm and lifts clear; both
   wait.

The demo scores 1.0 (`success`, episode ended at step 683 of 1,200): the blade carried 48 mm up with the doors open
from 9.2 s, seated with the doors open at 19.6 s, seated at the end 0.34 mm off centre (dx 0.01, dy 0.34), 0.0 deg
yaw and tilt, the doors closed (0.0 deg), the knob at 0.0 deg, no pad ever near a cutting edge, and the grippers 46 and
68 mm clear. Worst IK error 0.07 mm and 0.02 deg for the left arm; 10.8 mm and 3.7 deg for the right arm, in the first
hover from the ready pose (the servo moves are under 1 mm).

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (c11b3914c2b9); every one scores
exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; the blade stays on its shelf. | 0.0 |
| `twist_only` | The left arm turns the knob 92 deg as in the demo; the right arm does nothing. | Doors open at 100 deg, blade never touched, never carried. | 0.0 |
| `one_arm` | The right arm alone: picks the blade by its end, carries it 48 mm up over the head and lowers it onto the shut doors, lets go. | The doors never open (no rung: the carry rung needs the doors open 60 deg); the blade cannot reach the posts. | 0.0 |
| `edge_grip` | The right arm pinches the blade's near end 7 mm off the centre line, over a cutting edge, and lifts it 40 mm. | Gate at 8.25 s: the 12 mm pad footprint covers the outer 3 mm. | 0.0 |

## Build log (4 full demo runs and 6 short probes, 5 to 25 min each on the shared L40S)

| Run | Change | Result |
|---|---|---|
| 1 | first build: the right hand runs straight in along x from the ready pose | 0.0: its IK stalled 29 mm short (the arm reaches the pose only from above), the pads closed on nothing; the left hand's pinch stopped at 26.7 mm and the knob never turned (cause not isolated; it did not recur once the right hand's path changed) |
| probes | left alone (twist_only), right alone (one_arm), IK sweeps | the left arm turns the knob 92 deg (88.8 deg after the arc, 91.9 after one correction); an edge test with no bound on the blade's width flagged the gate at 2.15 s on any pad in the plane of the blade |
| 2 | enter from above and 20 mm short, bound the edge test to the blade's 22 mm | 0.75: blade seated, doors closed; the left hand backed off only 20 mm |
| 3 | the left hand also lifts 50 mm and backs off a further 40 mm | 0.75: clear 46 and 68 mm but a hand moving at 6 to 8 mm/s from servo jitter blocked the 5 mm/s release gate |
| 4 | release gate set to 10 mm/s | 1.0, step 683 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures, in the head
frame (mm from the plate centre; dz from the plate top): the blade centre dx, dy, dz, its yaw and tilt (deg), the knob
angle and both door angles (deg), the blade's lift above its shelf, whether each gripper holds it (opening at most 6
mm with a pad within 2 mm of it), its speed and spin (from its pose change between control steps, what a camera sees),
each gripper's clearance to the blade and razor, and its speed. The record keeps flags, peaks and held times for the
whole episode and is cleared only by the simulator reset, so taking the blade out and starting again never clears a
touch of the edges.

- Gate, any time in the run, gives 0: a pad point of either gripper, with the pad footprint taken as 12 mm along the
  tool's y, within 1 mm of the plane of the blade, over its length, and 8 to 11 mm from its centre line: the outer 3 mm of
  the 22 mm width, the cutting edges. A centred pinch on a short end puts the footprint at 6 mm, clear of them.
- 0.25: the blade held (a gripper closed to 6 mm or less within 2 mm of it) 20 mm or more above its shelf while both
  doors are open 60 deg or more.
- 0.5: the blade seated (centre within 1.5 mm of the plate centre in x and in y, its centre 0.6 mm or less above the
  seated height, yaw within 5 deg and tilt within 3 deg) while both doors are open 60 deg or more.
- 0.75: at the end the blade is seated, both doors are within 2 deg of closed and the knob is within 15 deg of 0, and it
  was seated with the doors open at some time.
- 1.0: also the blade still (under 1 mm/s and 2 deg/s) for 1 s and the release gate: every pad point of both grippers at
  least 30 mm from the blade and the razor and both grippers moving under 10 mm/s for 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score.

Real trial, what is observed and with what:

- Edge touch (the gate): ink the blade's two long edges (outer 3 mm each side) with washable blue marker and tape each
  gripper pad face with white paper tape; blue on either tape after the run means a touch, and the score is 0.
- Doors: a protractor or a side video frame with a protractor overlay (60 deg open; closed means a 0.5 mm feeler leaf
  does not slide between a door's free edge and the blade).
- Seated: a top photo with a mm grid on the plate (centre within 1.5 mm; the posts are visible through the holes), a 0.7
  mm feeler leaf does not slide under the blade's end (gap at most 0.6 mm), and the blade's yaw and tilt from the top
  photo and a side frame.
- Lift: the video frame against the shelf's 130 mm post, marked every 10 mm (20 mm up).
- Knob: an index mark on the knob and a 15 deg arc on the stand.
- Still: the blade does not move against the grid for 1 s of video at 20 Hz; clearance: a 30 mm gauge block between
  each pad and the blade; hand speed: no more than 0.5 mm of pad travel between video frames.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 683.
- Runtime on one L40S: a demo run took about 8 to 12 min wall clock on the shared VM, plus about 2 min of Isaac Sim
  start-up. A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (the stand and shelf are fixed and
  the blade is never jittered). The scripted demo is deterministic.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 683. The
  video is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png`
  (frame 0 of the top camera).
- Dream-1 run (filled in at filing), made with the command above on this exact `task.py` (c11b3914c2b9): inspect-robots-dream
  1.0.0, yam-arena 0059d02a1a91, task file c11b3914c2b9, 1200 steps in 4 minutes, log adhoc_cd984497.json, task_score 0.0, ended
  because max_steps.

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials

- Razor stand, handle and head: a 70 x 70 x 35 mm dark steel base (0.8 kg with the razor), a 12 mm steel handle up to a
  34 x 27 x 4 mm chrome head plate whose top is 130 mm above the table, at (0.240, 0.000). Two posts of 2.8 mm
  diameter and 1.6 mm height 8 mm either side of the plate centre.
- Doors: two 34 x 10.5 x 1.2 mm chrome steel plates, 3.3 g each, hinged along x at 12.5 mm either side of the plate
  centre; closed, they lie 0.6 mm above the plate over y 2 to 12.5 mm with a 4 mm slit over the posts, so the seated 0.4
  mm blade lies under them with 0.2 mm to spare and the posts stand clear; open they stand at 100 deg, 1.5 mm outside
  the blade's long sides.
- Knob: a 22 mm diameter, 12 mm tall black steel cylinder, 30 g, 46 mm above the table top, turning about the vertical
  handle axis, clockwise from above 0 to 100 deg, returned to 0 by a 0.01 N m/rad spring with 0.0005 N m of friction;
  it holds about 0.016 N m at 90 deg, and the 20 mm pinch carries well over that.
- Cam: 90 deg of knob turn opens the doors to 100 deg (a software link, run every control step).
- Blade: a double-edge razor blade, 43 x 22 x 0.4 mm steel, 3 g, with two 4.4 mm square holes centred 8 mm either side
  of its middle (0.8 mm of clearance round each post), friction 1.0. Its long sides are the cutting edges.
- Blade shelf: a 26 x 14 mm shelf on a 130 mm post with a 60 x 60 x 12 mm base, blue anodised aluminium, 0.5 kg, fixed at
  (0.240, -0.120); the blade's short ends overhang it by 8.5 mm so a pad can slide under one.
- Layout chosen for reach: both hands approach along x at heights where the arms' IK is exact (the head at 130 mm and
  the knob at 46 mm with the hand pitched 15 deg), 84 mm apart in height.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Open the knob, take the blade out by its short ends with tweezers, lay it back on the shelf with
its long axis along x and a short end overhanging, centred on (0.240, -0.120), re-ink its two long edges and put fresh
white tape on both pads. Let the knob spring back to 0 and check the doors lie flat (a 0.5 mm feeler leaf does not slide
under a door's free edge). The stand and shelf are fixed on taped outlines and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
