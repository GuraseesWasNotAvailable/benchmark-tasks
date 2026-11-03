# Shim a mount to a set height

Instruction given to the policy: `Stack shims flat on the black pad so the top of the stack is 80 mm above the pad, within 2 mm, using as few shims as possible. The purple, green, blue, red and white shims are 20, 25, 30, 35 and 45 mm thick.`

Files in this folder: `task.json`, `shim_stack_height.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 42100c201e44 (`sha256sum shim_stack_height.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/shim-stack-height/shim_stack_height.py
```

It asserts the scoring rules on hand-built states (start 0; red on white or white on red 1.0; purple, green and red, the three-shim 80 mm set, 0.75; a set 5 mm off 0.5 and 10 mm off 0.25; a set 15 mm off, one shim alone and all five stacked 0; a top shim tilted 4 degrees, 16 mm off centre or still moving 0; a shim on its edge or end making up the height 0; a shim held still 15 mm above the stack, a shim bridged over the stack on two edge-standing shims, a shim propped 3 mm up, a 2.3 mm gap under the top shim or two 1.5 mm gaps 0, while one 2 mm gap still scores 1.0; the right pair stacked beside the pad or floating 4 mm above it 0; a shim on a spare that overhangs the pad's outline, or a shim leaning on the stack from the pad, 0; a top shim upside down, turned 45 or 90 degrees, tilted 2.5 degrees or 14 mm off centre still 1.0; spare shims on the table, leaning on the stack from the table, or stacked beside the pad or off its corner but clear of its outline ignored), the height ladder at and 0.3 mm past its exact 2, 7 and 12 mm limits, the outline test at the pad's corner, that the fewest shims reaching 80 mm is two and the only other such set is three, and that no set of shims lands 1 to 4 mm from 80 mm, and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/shim-stack-height/shim_stack_height.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stack shims flat on the black pad so the top of the stack is 80 mm above the pad, within 2 mm, using as few shims as possible. The purple, green, blue, red and white shims are 20, 25, 30, 35 and 45 mm thick." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/shim-stack-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the 0 / 0.25 / 0.5 / 0.75 / 1.0 ladder defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/shim-stack-height/shim_stack_height.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stack shims flat on the black pad so the top of the stack is 80 mm above the pad, within 2 mm, using as few shims as possible. The purple, green, blue, red and white shims are 20, 25, 30, 35 and 45 mm thick." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/shim-stack-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm sets the white 45 mm shim flat on the pad, then the red 35 mm shim flat on top of it, 45 + 35 = 80 mm from the fewest shims that reach it; two top-down pick and place moves with the fingers closing across each shim's 50 mm width and the pad tips 6 mm up its sides, a carry at 14 cm, and each shim released 5 mm above the surface it lands on. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses 20 mm above the pad or the stack and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 625 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 7 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 2 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_1b0d39c8.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_5d33b0e1.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 625 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the five shim colours and the 6 mm pad are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the five shims back on their marks, flat with the 60 mm side pointing forward; the pad does not move.
