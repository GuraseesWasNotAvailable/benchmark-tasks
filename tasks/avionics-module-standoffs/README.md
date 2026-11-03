# Stack avionics modules and covers on standoffs

Instruction given to the policy: `Seat each gold module on its four standoffs, then the blue cover flat on top. Centre both over the standoffs within 5 mm, long sides along the deck's long side.`

Files in this folder: `task.json`, `avionics_module_standoffs.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 7cb56fb0deb8 (`sha256sum avionics_module_standoffs.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/avionics-module-standoffs/avionics_module_standoffs.py
```

It asserts the scoring rules on hand-built states (start 0, done 1.0, one lane seated and covered 0.5, a module seated without its cover 0.3, a module 5 to 10 mm off or 5 to 15 degrees turned 0.15 with its cover earning nothing, an exact 5.0 mm offset and an exact 5.0 degree tilt seated, a module turned 90 degrees, on the deck, tipped, on its side, held, moving, turning or more than 10 mm off 0, a cover set on the posts 0, a cover judged from the deck centre rather than its module (a module 4 mm off with its cover 2 mm further out not credited), a cover off-centre, turned, tilted or moving not credited, a reversed or upside-down module seated, the modules swapped between the lanes 1.0) and replays both arms' plans through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/avionics-module-standoffs/avionics_module_standoffs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Seat each gold module on its four standoffs, then the blue cover flat on top. Centre both over the standoffs within 5 mm, long sides along the deck's long side." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/avionics-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the sum over the two lanes of 0.30 for a seated module (0.15 roughly seated) plus 0.20 for a cover on a seated module, centred over the standoffs within 5 mm, as defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/avionics-module-standoffs/avionics_module_standoffs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Seat each gold module on its four standoffs, then the blue cover flat on top. Centre both over the standoffs within 5 mm, long sides along the deck's long side." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/avionics-demo
```

The demo is `demo()` in the task file, open loop and deterministic: both arms work at once, each in its own lane, as mirror images. Each arm grips its module across the 50 mm faces with the fingers closing along x, turns it 90 degrees during the carry and lowers it onto the four standoffs, releasing it 3 mm above the post tops; then it grips the cover across its 60 mm faces and lowers it square onto the module, releasing it 4 mm above the module's top. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses above each place and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 626 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 12 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 3 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_9b9d9add.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_45b8e11f.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 626 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the 8 mm standoffs are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand each module back on its module pad with its long side across the table, and each cover on its cover pad with its long side along x; the decks, standoffs and pads do not move.
