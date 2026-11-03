# Seat a solar wing on a height-matched tip stay

Instruction given to the policy: `In the collar, stand the orange stay whose height matches the top of the yoke block, then seat the blue wing with its root between the rails against the stop and its tip level on the stay.`

Files in this folder: `task.json`, `solar_wing_tip_stay.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 62fb9d998140 (`sha256sum solar_wing_tip_stay.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/solar-wing-tip-stay/solar_wing_tip_stay.py
```

It asserts the scoring rules on hand-built states (start 0, done 1.0, the stay set alone 0.25, both placed with the wing skewed 6 degrees or its tip 90 mm high on the 30 mm stay stacked on the 50 mm one 0.5; 0 for anything without the 50 mm stay set in the collar, such as the wing seated and tipped onto the collar, propped on the 30 mm stay, level on the 50 mm stay outside the collar or held level with no stay; the wing 12 mm from the stop, on its edge, stood on its end, across the rail tops or still moving not captured; a stay leaning 8 degrees, held 10 mm up, 7 mm off centre or still moving not set; a stay in a pocket corner or exactly 3 mm up, an upside-down or reversed wing and a wing against a rail still full score) and replays both arms' plans through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/solar-wing-tip-stay/solar_wing_tip_stay.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "In the collar, stand the orange stay whose height matches the top of the yoke block, then seat the blue wing with its root between the rails against the stop and its tip level on the stay." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/wing-stay-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the 0 / 0.25 / 0.5 / 1.0 ladder defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/solar-wing-tip-stay/solar_wing_tip_stay.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "In the collar, stand the orange stay whose height matches the top of the yoke block, then seat the blue wing with its root between the rails against the stop and its tip level on the stay." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/wing-stay-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the right arm stands the 50 mm stay in the collar and parks; the left arm waits over the wing until the right arm has parked, picks it at yaw 0 with the fingers across its 60 mm width, carries it at 150 mm while turning it -90 degrees so its far end becomes the tip, pauses with the wing 10 mm above the rail tops, lowers it between the rails and releases it about 5 mm above the block and the stay top, which it reaches at the same moment (the release is commanded 8 mm up because the arm droops about 2.5 mm under the wing at this reach). Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses above each pocket and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. The place yaw is -90 degrees because +90 degrees drives the wrist joint to its limit. It reached `task_score: 1.0` in 532 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 11 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 2 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_ef50d9ba.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_daaca8b8.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 532 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the rails, the stop and the two stay heights are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Lay the wing back flat on its mark with its long axis forward and stand both stays back on their marks; the yoke and the collar do not move.
