# Free a panel from a leaning rack, moving only what leans on it

Instruction given to the policy: `Stand the red panel upright on the white inspection plate, and stand any panel you move upright on the black mat.`

Files in this folder: `task.json`, `keystone_panel_unrack.py` (scene, success, partial credit, the whole-trial record, the release gate and the scripted demo), `linkkit.py` (MIT, our own articulation kit; this task uses only its gripper read-out and its simulator-reset hook), `README.md`.

Licence: every file in this folder is original work under the MIT licence. No third-party assets are used.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 613928fcd969 (`sha256sum keystone_panel_unrack.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/keystone-panel-unrack/keystone_panel_unrack.py
```

It asserts the scoring rules on hand-built states (the spawn pose and the settled lean 0; both blue panels parked, the red panel on the plate and the grey panel untouched 1.0; one or both blue panels parked 0.2 or 0.4; the grey panel parked as well 0.8; the grey panel moved to the table or slid 12 mm along its rail, or the blue panels slid along their rails until each stands alone, and the red panel's credit is withheld; the grey panel bumped 8 mm still counts as untouched; both blue panels swung to 26 to 30 degrees after the red panel is pulled, or the outer blue panel swung 30 degrees after the inner one is pulled, 0; a panel lying on the mat or leaning 11 degrees 0; a panel leaning 9 degrees no fall; a parked panel upside down, any panel turned about the vertical, the red panel 24 mm off the plate centre with its base still on the plate or tilted 4 degrees, and a parked panel leaning 4 degrees on its neighbour still count; the red panel's base 1 mm over the plate edge, tilted 6 degrees, moving, held 4 mm up, on the mat or on the table, a blue panel on the plate, on the table, with its base 1 mm past the mat's corner, standing on another panel or still rocking, and a panel lying on the rails, no credit), then runs the whole-trial record over sequences of states (the order along the chain 1.0; the red panel pulled first, the inner blue panel pulled first, or a panel slipping 12 degrees in the grip, 0 however well everything is placed afterwards; a parked panel rocking 8 degrees and settling 1.0; after a fall, the rack rebuilt step by step to its exact start, or put back at its exact start in one step as a reset would leave it, and then unloaded, 0; the grey panel lifted 15 mm, slid 12 mm along its rail or rocked 7 degrees and set back, step by step or in one step, then the chain unloaded, 0.8, not 1.0, while the grey panel bumped 8 mm and left there still counts as unmoved; only the simulator's reset clears the record, so a clean trial after it scores 1.0, whatever the trial before it did), checks the release gate (the full-score state with a gripper 10 mm from the red panel 0 and no success; with both grippers 30 mm clear 1.0), and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/keystone-panel-unrack/keystone_panel_unrack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the red panel upright on the white inspection plate, and stand any panel you move upright on the black mat." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/keystone-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is 0 if any panel tilted more than 10 degrees at any control step of the trial, and otherwise 0.2 for each blue panel standing on the mat, 0.4 more if the red panel stands on the plate while both blue panels are on the mat and the grey panel is on its rail or the mat, and 0.2 more if the grey panel never moved, as defined in `task.json`; otherwise 0.0. `score()` reads 0, and `success()` is false, while either gripper's pads are within 30 mm of a panel: `linkkit` measures both grippers after every control step, before the task file is scored, so a panel held in a scoring place earns nothing. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene. The fall and grey-panel record lives in the task module and only the simulator's reset clears it: importing `linkkit` hooks the yam-arena embodiment's `reset()`, which calls `_reset_records()` after the scene is put back and before the first step of each trial. So each epoch is scored on its own, while a rack rebuilt by the arm, however exactly, cannot clear the record.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/keystone-panel-unrack/keystone_panel_unrack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the red panel upright on the white inspection plate, and stand any panel you move upright on the black mat." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/keystone-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm takes the outer blue panel to the far end of the mat, the inner blue panel to its middle, then the red panel to the plate, and never touches the grey panel. Every grip is top-down with the fingers closing along x across the panel's 50 mm depth, the pad tips 22 mm below the panel top, so the fingers' narrow tips stay 16.8 mm clear of the neighbours' faces and their 59 mm wide section starts about 30 mm above the neighbours' tops. Each panel is lifted 15 mm off its rail, stepped 10 mm toward the emptied side of the rack, slid out along -x in front of the rack, raised to 15 cm, carried along y, paused 10 mm above its place, lowered slowly and released 5 mm above the mat or plate. The wrist works at yaw +90 degrees on the mat side and -90 on the plate side, and turns through yaw 0 above the emptied half of the rack before it picks the red panel, so joint 6 stays inside its limits. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses above each place and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. After the last release the arm rises to 15 cm, which leaves its pad tips about 40 mm from the red panel, outside the 30 mm release gate. It reached `task_score: 1.0` in 1253 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 12 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 5 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and the leaning start and the open-loop demo need it. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_5c1ebb7b.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_25113dba.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 1253 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the rails under each panel's foot (wide and centred under the grey and red panels, narrow and to the outside under the blue ones), the open slots between them and the panel colours are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the red and grey panels centred on their rails, then the inner blue panel on its rail with its inner face against the red one and the outer blue panel on its rail with its inner face against the inner blue one, and let both lean; the outer blue panel's outer face is flush with its rail's outer edge and the inner blue panel's is 6 mm past its rail. The rack, mat and plate do not move.
