# Clear the deck, leave the arming pin

Instruction given to the policy: `Stow every cylinder in the tray. Leave the red arming pin in its socket.`

Files in this folder: `task.json`, `arming_pin_deck_clear.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 4cbd24ce0e5d (`sha256sum arming_pin_deck_clear.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/arming-pin-deck-clear/arming_pin_deck_clear.py
```

It asserts the scoring rules on hand-built states (start 0, done 1.0, two and three cylinders stowed 0.5 and 0.75; the pin stowed in the tray, pulled and put back, pulled first and put back before the job, left on the socket rim, lying across the socket or knocked onto the table all 0; the pin nudged up 8 mm and back, or leaning 15 degrees in its socket, still passing; a cylinder held 5 mm above the floor, still sliding, on a tray wall, stacked, beside the tray, on the bracket or off the table earning nothing; a cylinder lying on the tray floor counting) and replays the right arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/arming-pin-deck-clear/arming_pin_deck_clear.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stow every cylinder in the tray. Leave the red arming pin in its socket." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/arming-pin-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is 0 if the pin has ever risen 10 mm out of its socket or is not standing in it, and otherwise 0.25 per grey cylinder resting on the tray floor, as defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: run the command again, one process per trial, rather than adding `--epochs K`. The pin's never-left-the-socket check is a latch kept in the task file's module state for the life of the process, so a pin pulled in one epoch would zero every later epoch of the same process. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/arming-pin-deck-clear/arming_pin_deck_clear.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stow every cylinder in the tray. Leave the red arming pin in its socket." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/arming-pin-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the right arm stows the four grey cylinders in turn, the one nearest the pin first, each a top-down grip at mid-height, a carry 15 cm above the table and a set-down 5 mm above the tray floor, in two rows of two. It never touches the pin; its fingertips stay at least 30 mm from it while lifting the first cylinder, and a carried cylinder's bottom stays 20 mm above the pin's top. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses above each slot and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 1272 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 8 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 5 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and the pin needs it, since any jitter would spawn the pin inside the socket walls. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_46bd3b2c.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_837b1ac0.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 1272 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the 20 mm pin, its socket and the 15 mm tray lips are legible; both scored runs and their videos are at 320 x 176. On real arms the record for the pin's never-left-the-socket condition is a separate side video, not the top camera, which looks almost straight down and cannot show a 10 mm rise. The pin's lowest 10 mm is wrapped in white tape, and every trial is filmed with a phone on a stand at bracket height, 0.3 m from the socket and looking at it side-on. While the pin is seated the white band sits inside the 20 mm socket; it shows above the rim only once the pin has risen more than 10 mm, so the gate fails if white shows above the socket rim in any frame of the side video. In simulation the per-step latch applies.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the four grey cylinders back on their tape dots and the pin back in its socket, upright and down on the bracket; the bracket, socket and tray do not move. Start each trial in a new process, because the pin's latch lives for the life of the process.
