# Count five bolts into the flight kit

Instruction given to the policy: `Put exactly five grey bolts in the white kit tray and all the other bolts in the black spares tray.`

Files in this folder: `task.json`, `flight_kit_count.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 6a12dadaf505 (`sha256sum flight_kit_count.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/flight-kit-count/flight_kit_count.py
```

It asserts the scoring rules on hand-built states (start 0, done 1.0, five in the kit with no or one spare 0.5 and 0.75, four in the kit (one short) 0.25 whatever the spares tray holds, six in the kit (one over) 0, one, two, three or seven in the kit 0, all seven in the spares tray 0, taking the trays in turn without counting 0.25 from the kit tray and 0 from the spares tray, a kit bolt lying or leaning in the pocket counted, a bolt lying or standing on the kit bolts counted, a bolt held 75 mm up, on a wall, outside a wall, on the floor under a pocket, still moving, knocked over or off the table not counted, the pocket edge exact to 1 mm) and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/flight-kit-count/flight_kit_count.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put exactly five grey bolts in the white kit tray and all the other bolts in the black spares tray." \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/flight-kit-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the count ladder defined in `task.json` (exactly five in the kit 0.5 plus 1/4 per bolt in the spares tray, four in the kit 0.25, six or any other kit count 0); otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/flight-kit-count/flight_kit_count.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put exactly five grey bolts in the white kit tray and all the other bolts in the black spares tray." \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/flight-kit-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm takes the five far bolts of the row into the kit tray, standing 40 mm apart, then the two near ones into the spares tray, seven top-down pick and place moves with the fingers closing across the row, so each bolt's neighbours sit beside the pads rather than in front of them. Each bolt is carried at 14 cm, paused with its bottom 10 mm above the tray walls, lowered slowly and released 5 mm above the table inside the walls; the pad tips stay above the 20 mm walls throughout. Every grip closes to 8 mm narrower than the bolt rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the bolt (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 2244 steps (`stats.total_steps`), ended by `success`, within the run limit of 3,600.

## Run limit, runtime and seeds

- Run limit: 3,600 control steps at 20 Hz (180 s of simulated time), `--max-steps 3600` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 10 min wall clock for the submitted 3,600-step trial (Dream round trips pause the simulator); the demo about 8 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_7fbfb171.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 3600 steps.
- Demo: log `adhoc_c8a780e5.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 2244 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the 16 mm bolts and the 6 mm tray walls are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the seven bolts back on their marks in the row; the trays do not move.
