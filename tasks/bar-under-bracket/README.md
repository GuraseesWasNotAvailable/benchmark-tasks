# Slide a rigid bar under a standoff bracket to its stop

Instruction given to the policy: `Route the black bar under the grey bracket so it lies flat between the posts of both white clips, with its end within 8 mm of the red stop.`

Files in this folder: `task.json`, `bar_under_bracket.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 36e9aadcda80 (`sha256sum bar_under_bracket.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/bar-under-bracket/bar_under_bracket.py
```

It asserts the scoring rules on hand-built states (start 0, seated 1.0, touching the stop or an exact 8.0 mm gap 1.0, a 9 mm gap 0.75, through the tunnel but short of clip A, beside both clips' posts or yawed past clip A's posts 0.5, under 30 mm or exactly 20 mm of the plate 0.25, through the tunnel but propped on the stop or a clip post, one end 4 mm up or leaning on an edge 0.25, under 15 mm, beside the bracket, on top of the plate, held 30 mm up, on an edge beside the bracket, moving, standing on end or off the table 0, reversed end for end, rolled onto another face, rolled 8 degrees or 5 mm off the line still seated) and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/bar-under-bracket/bar_under_bracket.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Route the black bar under the grey bracket so it lies flat between the posts of both white clips, with its end within 8 mm of the red stop." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/bar-bracket-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the 0 / 0.25 / 0.5 / 0.75 / 1.0 ladder defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/bar-under-bracket/bar_under_bracket.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Route the black bar under the grey bracket so it lies flat between the posts of both white clips, with its end within 8 mm of the red stop." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/bar-bracket-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm grips the bar 20 mm from its trailing end with the fingers closing across the channel, carries it over the channel line with its leading end 30 mm short of the bracket, lowers it until its underside is 13 mm above the clip posts and its top 14 mm below the plate, slides it 188 mm along the channel under the plate and over both clips until its end is 4 mm short of the stop, then lowers it slowly between the clip posts and releases it 5 mm above the table. The hand never passes over the bracket or a clip. The grip closes to 8 mm narrower than the bar rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the bar (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before the close, the slide, the set-down and the release, because the simulated wrist trails a moving target by several millimetres. While it holds the bar, the hand and grip also settle about 2 degrees past the commanded yaw (measured in the simulator: joint 6 lags about 1.6 degrees; the rest is the bar turning in the grip); from the lift on the demo commands half of that less, so the held bar lies within about 1 degree of the channel and clears clip A's posts by about 2 mm whether the wrist twists or not. It reached `task_score: 1.0` in 381 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 10 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 2 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_b281ad39.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_4fbc49da.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 381 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the clip posts and the tunnel under the plate are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Lay the bar back on its marks, 80 mm beyond the channel line and parallel to it; the bracket, clips and stop do not move.
