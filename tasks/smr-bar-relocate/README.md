# Move a target bar without dropping the loose reflector ball on it

Instruction given to the policy: `Move the dark grey bar onto the white pad and leave the silver ball resting on top of it.`

Files in this folder: `task.json`, `smr_bar_relocate.py` (scene, success, partial credit, the touch record, the release gate and the scripted demo), `linkkit.py` (MIT, our own articulation kit; this task uses only its gripper read-out and its simulator-reset hook), `README.md`.

Licence: every file in this folder is original work under the MIT licence. No third-party assets are used.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 5a1d6cf03a1f (`sha256sum smr_bar_relocate.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/smr-bar-relocate/smr_bar_relocate.py
```

It first asserts the numbers the task rests on: the bar travels 150 mm along its own length, the ball rolls back 5/7 of that, 107 mm, while no straight line between two points 8 mm in from the edges of the top is longer than its 91 mm diagonal, and from its start 25 mm along the top the ball leaves the end after 35 mm of travel; the bar is 50 mm wide and 100 mm long, so a top-down grip fits only across its width; the touch line at 22.5 mm lies 1.5 mm above a ball on the pad (21 mm) and under a ball in the nest (33.7 mm), and a ball in the nest stands 14.7 mm clear of the table. Then the scoring rules on hand-built states: start 0; the bar placed with the ball on it 1.0, with the ball at the middle of the top, 25 mm along it as it started, 17 mm off its axis or 42 mm along it (8 mm in from an edge), the bar 9 mm off and yawed 9 degrees, turned end for end, upside down or tilted 2 degrees, the ball still creeping at 8 mm/s; the ball 18 mm off the axis or 43 mm along it 0; the bar 11, 20 or 29 mm off or yawed 11 or 19 degrees with the ball on it 0.6, and 31 mm off or yawed 21 degrees 0; the ball in the nest with the bar placed 0.5, with the bar near 0.3, with the bar 31 mm off, still at its start or on its side 0; the ball 5 mm off the nest centre still counting, 7 mm off, held 4 mm above its seat or lying on a nest block's top not; the bar on its side, tilted 4 degrees, held 3 mm above the pad, flat on the table 2 mm below the pad top or still moving 0; the ball rolling at 20 mm/s or held 3 mm above the bar 0; the ball on the table, on the pad or on the floor 0; the ball moved to the middle of a bar that never moved 0. Then short histories of states, scored step by step as the simulator does: parking the ball, moving the bar and reseating the ball 1.0, and stopping with the ball in the nest 0.5; carrying the bar with the ball on it, the ball rolling off the end onto the table and being put back, 0; the ball dropped on the pad and reseated 0; the ball dropped on the table and then parked in the nest 0, not 0.5; the ball carried with its centre 24 mm up then reseated 1.0, and 22 mm up 0; a dropped ball lowered back onto its mark with a gripper, let fall 30 mm onto it, set back 5 mm off it, or put back on its mark with the bar in one step and at rest, as a reset would leave them, does not clear the record, nor does a ball whose pose is lost to a solver blow-up; only the simulator's reset clears it, so with the reset in between, a clean run after a dropped ball scores 1.0. Then the release gate: the full-score state with a gripper 10 mm from the ball scores 0 and does not succeed, with both grippers 30 mm clear 1.0. Finally it replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

Why the ball cannot ride on the bar: a ball rolling without slip on a level carrier that starts and ends at rest moves 2/7 of the carrier's displacement, whatever the speed or the path, so relative to the carrier it rolls back 5/7 of it. Slipping instead would need a carrier acceleration above 3.5 times the friction coefficient times g, about 34 m/s2, which no arm motion reaches. The bar goes 150 mm toward -y, along its own length, so the ball rolls 107 mm toward +y along a top that is 100 mm long; from its start it rolls off the end after 35 mm of bar travel, falls 30 mm to the table and keeps rolling. The fingers cannot help: the bar's 100 mm length is wider than the 94 mm open hand, so a top-down grip closes across its 50 mm width, and the fingers stand beside the long sides, clear of the ball's path along the top (checked against the arm's collision hulls for grips at the middle and 25 mm toward either end). A move that keeps the bar's heading rolls the ball 107 mm in a straight line across the top, and no straight line between two points 8 mm in from its edges is longer than the 91 mm diagonal. Turning the bar during the carry, to fence the ball with a finger or steer it along curves, is a far harder route to the same end that counts if it works.

| Where the ball ends | Bar placed (10 mm, 10 degrees) | Bar near (30 mm, 20 degrees) | Bar elsewhere |
|---|---|---|---|
| resting on the bar's top, 8 mm in from every edge | 1.0 (success) | 0.6 | 0 |
| resting in the nest | 0.5 | 0.3 | 0 |
| anywhere else | 0 | 0 | 0 |
| touched the table or the pad at any moment | 0 | 0 | 0 |

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/smr-bar-relocate/smr_bar_relocate.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move the dark grey bar onto the white pad and leave the silver ball resting on top of it." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/smr-bar-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is 0 if the ball touched the table or the pad at any step, and otherwise 1.0, 0.6, 0.5, 0.3 or 0 from where the bar and the ball rest, as defined in `task.json`; otherwise 0.0. The touch record is kept inside the task file: `success()` and `score()` update it on every control step, and only the simulator's reset clears it: importing `linkkit` hooks the yam-arena embodiment's `reset()`, which calls `_reset_records()` after the scene is put back and before the first step of each trial, so each trial and each epoch begins clean while a ball put back, however exactly, does not. `score()` reads 0, and `success()` is false, while either gripper's pads are within 30 mm of the bar or the ball: `linkkit` measures both grippers after every control step, before the task file is scored, so a ball held on the bar earns nothing. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/smr-bar-relocate/smr_bar_relocate.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move the dark grey bar onto the white pad and leave the silver ball resting on top of it." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/smr-bar-demo
```

The demo is `demo()` in the task file, open loop and deterministic: three top-down pick and place moves by the left arm. First it takes the ball off the bar at yaw 0, the fingers closing along y with the pads on its equator and their tips 12 mm above the bar top, carries it at 15 cm, pauses with its bottom 10 mm above the nest blocks, lowers it slowly and lets it go 2.3 mm above its seat, the pad tips 7 mm above the blocks. Then it turns the wrist to yaw -90 degrees, takes the bar across its 50 mm width with the pad tips 5 mm above the table, carries it 150 mm along its length and sets it on the pad, released 5 mm above it. Last it lifts the ball out of the nest at yaw 0, the pad tips 5 mm above the blocks, carries it over the middle of the bar and holds it with its bottom 3 mm above the top. There the grip eases over 1.5 s from 8 mm to 1 mm narrower than the ball, so the ball slides down the pads onto the top as the squeeze fades. The arm then holds still for 0.6 s while the soft wrist, which the hard squeeze twists about 1.5 degrees, unwinds and the ball settles, and only then do the fingers open, along the bar's length, where the ball has 42 mm to the inset line either way. In the simulator, opening straight from the hard squeeze left the ball rolling across the top at 8 mm/s, and opening before it had settled flicked it off the bar. The ball comes to rest on the top just before the fingers open; the arm then rises to 15 cm, its pad tips over 60 mm above the ball, and success, which needs 1 s at rest with both grippers at least 30 mm clear, comes during that retreat. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses above each place and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 1026 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 11 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 4 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and both the scene and the open-loop demo need it: jitter moves the ball and the bar independently and can start the ball off the bar. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_5bea8a07.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_07e3d79e.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 1026 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the ball on the bar, the nest opening and the pad are legible; both scored runs and their videos are at 320 x 176. The bar is dark grey so that the silver ball stands out against it from above.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Set the bar back on its mark, flat with its long axis along y, and rest the ball on its top 25 mm from the centre toward +y until it is still; the nest and the pad do not move. In simulation only the simulator's reset, at the start of each trial, clears the touch record.
