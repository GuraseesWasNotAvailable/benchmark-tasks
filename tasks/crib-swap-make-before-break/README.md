# Swap a cracked crib block under a loaded beam

Instruction given to the policy: `The red crib block under the beam is cracked. Swap it for the green block and put the red one in the grey tray. The beam must not drop or move.`

Files in this folder: `task.json`, `crib_swap_make_before_break.py` (scene, success, partial credit, the disturbance record and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: c10c14e2421c (`sha256sum crib_swap_make_before_break.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/crib-swap-make-before-break/crib_swap_make_before_break.py
```

It asserts the scoring rules on hand-built states: start 0; the green crib slid in beside the red one 0.4; the red crib then pulled out onto the table 0.7; the red crib in the tray 1.0, also with the green crib where the red one stood, on the beam's centreline, at x 0.310 or at the seat's far limit x 0.325, upside down, turned end for end or yawed 8 degrees, and with the red crib standing on end in the tray; the beam dropped onto the table after the red crib was pulled first, lifted 8 mm, dragged 7 mm, tilted 3 degrees or with a non-finite pose 0 whatever else is done, while 5.9 mm of movement or a 1 degree turn between the lips still counts and a 2 degree tilt caps the score at 0.7; a green crib crosswise, on its side, tilted 4 degrees, yawed 12 degrees, held 3 mm up, moving, too near the beam's middle, covering only 30 mm of the beam's width on either side, in the tray or left at its start 0; a red crib partly under the beam, lying on the beam, moving, or 3 mm under a beam turned 0.95 degrees between the lips not out; a red crib across the tray walls or still held over the tray floor not binned. It checks the physics numbers the text quotes: 11.54 degrees for the dropped beam, 0.52 degrees for the beam settled on the green crib's far top edge, and a free-end lift that trips the record between 11.3 and 12.0 mm. It then runs whole sequences through the disturbance record: green in, red out, red binned 1.0; red pulled first, the beam dropped and then lifted back level with the swap finished 0; the beam shoved 8 mm during the pull and pushed back 0; the beam jacked 10 mm for an in-place swap 0; the beam nudged 4 mm 1.0; before either crib moves, the beam slid 10 mm along y and back, its free end lifted 20 mm and set down, or the beam pushed back to 0.9 mm off its rest, each followed by a clean swap, 0; the same three as coarse snapshots with every body at rest, 0; the beam tripped and put back, then lifted 4 mm and dropped onto its supports in one step, or the green crib fumbled 12 mm onto its mark in one step, then a clean swap, 0; the red crib pulled first and everything put back by hand to the start state, then a clean swap, 0; a dropped trial, then the simulator's reset putting every body back in one step, then a clean swap 1.0; two clean trials in a row 1.0; a solver blow-up, the reset, then a clean swap 1.0. Last, it replays the right arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/crib-swap-make-before-break/crib_swap_make_before_break.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The red crib block under the beam is cracked. Swap it for the green block and put the red one in the grey tray. The beam must not drop or move." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/crib-swap-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the 0 / 0.4 / 0.7 / 1.0 ladder behind the seated-green gate and the whole-trial disturbance record, as defined in `task.json`; otherwise 0.0. The record is kept in the task file and updated on every control step, so a beam that dropped or moved at any moment scores 0 even if it was put back. It is cleared only when a new trial starts: at the first scored step, or when the simulator's reset puts every body back on its mark, which shows as a body's centre jumping more than 15 mm in one step while it reports under 10 mm/s before and after. A body that starts and ends a 50 ms step at rest, falling or sliding against 1 g, covers at most 12 mm, so no slow return to the start state and no short drop onto a mark can clear it. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene. The reset between trials puts every body back in one step, which restarts the record. The one case it cannot see is a trial that ends with the record tripped and every body already within 15 mm of its mark, since the reset then moves nothing far enough to show; that trip carries into the next trial. For strictly independent repeats, run each trial as its own command.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/crib-swap-make-before-break/crib_swap_make_before_break.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The red crib block under the beam is cracked. Swap it for the green block and put the red one in the grey tray. The beam must not drop or move." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/crib-swap-demo
```

The demo is `demo()` in the task file, open loop and deterministic, right arm only, every grip at yaw 0 with the fingers closing across a crib's 30 mm width. The arm grips the green crib 5 mm in from its near end with the pad tips 12 mm above the table and slides it 120 mm along the table, under the beam beside the red crib, where it carries nothing yet because its top is 1 mm below the beam. It lets go with a 56 mm opening and rises. It then grips the red crib 5 mm in from its near end and slides it 120 mm out toward itself; the red crib carries the beam's left end until it clears, and the beam settles 1 mm onto the green crib's far top edge, about 0.5 degrees. Last, it grips the red crib again at its middle, lifts it to a carry 150 mm above the table, pauses with it 10 mm above the tray walls, lowers it slowly and releases it 5 mm above the tray floor. Gripping each crib at its near end keeps the fingers more than 10 mm from the beam. After each close the arm takes up 0.3 mm before it slides, and the beam is spawned 0.6 mm above its supports and settles in the 0.5 s before the first step; both keep the offline clearance model, which holds uncarried parts at their spawn pose, from reading the slides as contact through the IK's tracking error. Every grip closes to 8 mm narrower than the crib rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the crib (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release and makes the last descent into the tray slowly, because the simulated wrist trails a moving target by several millimetres. The plan is 42.4 s; with the 3 s hold that is 908 steps. It reached `task_score: 1.0` in 827 steps (`stats.total_steps`), ended by `success`, ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 9 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 4 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and the open-loop demo needs it. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_c9357aff.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_baf174ff.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 827 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the saddle lips on the pier and the red crib under the beam's end are legible; both scored runs and their videos are at 320 x 176. A dropped beam is plain in the video: its left end lies on the table, 40 mm below the pier top.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Lift the beam off, set the red crib on its mark centred at (0.270, -0.110) with its long axis along x, lay the beam back level on the pier between the lips and on the red crib, then lay the green crib on its mark at (0.150, -0.185) and empty the tray. The pier, lips and tray do not move; check them against their marks.
