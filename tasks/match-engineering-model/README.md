# Copy a reference assembly onto a turned fixture

Instruction given to the policy: `Use the three loose blocks to build a copy of the glued model on the empty white plate. Place each block relative to this plate's red edge as its twin sits relative to the model plate's red edge, within 8 mm and 10 degrees.`

Files in this folder: `task.json`, `match_engineering_model.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 644eef957d38 (`sha256sum match_engineering_model.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/match-engineering-model/match_engineering_model.py
```

It asserts the scoring rules on hand-built states (start 0, done 1.0, the base alone or the side block alone 0.33, the base with the side block or with the cube 0.67, a copy made in the room's orientation, with the datum reversed or mirrored toward and away from the red edge 0, a copy mirrored end for end along the red edge 0.33, the base 7 mm or exactly 8.0 mm off, turned 9 degrees, end for end or upside down still in place, the base 9 mm off, turned 11 degrees, on its side, tilted 6 degrees, held, moving or resting on the side block no credit for it or for the cube on it, the side block 8 mm toward the base and 2 mm short of it, turned 8 degrees or exactly 10.0 degrees either way still in place, the side block pushed against the base, turned 90 degrees, lying on a long face, standing on end or stacked on the base no credit, the cube turned 90 degrees, exactly 10.0 degrees either way or rolled onto another face still in place, the cube turned 45 degrees, on the plate, on the base's far end, on the side block or moving no credit) and that the demo's grips line up with each block's start and place, then replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/match-engineering-model/match_engineering_model.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Use the three loose blocks to build a copy of the glued model on the empty white plate. Place each block relative to this plate's red edge as its twin sits relative to the model plate's red edge, within 8 mm and 10 degrees." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/eng-model-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is 1/3 for each block in place, the cube only on a base that is in place, as defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/match-engineering-model/match_engineering_model.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Use the three loose blocks to build a copy of the glued model on the empty white plate. Place each block relative to this plate's red edge as its twin sits relative to the model plate's red edge, within 8 mm and 10 degrees." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/eng-model-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm places the base first, since the cube stands on it, turned 90 degrees from lying across the table to lying along it, then the side block, turned 90 degrees the other way and set down beside the base, its near end 10 mm outboard of the base's long side and its far face flush with the base's far end, then the cube onto the base's near end; three top-down pick and place moves that each release the block 5 mm above its support. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses above each place and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 948 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 10 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 4 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_6fe3d3cd.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_853959d4.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 948 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the red datum strips and the cube on the model's base are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the three loose blocks back on their marks: the base long axis across the table, the side block long axis forward, the cube square to the table edges; the plates and the glued model do not move.
