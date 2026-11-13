# Lock out a ball valve: close a clamshell lockout around its lever and slide its lock bar through

The robot closes two spring-opened halves of a red lockout around the yellow lever of a lever ball valve and slides a
steel lock bar through the holes in the halves' end plates, so that the lever cannot be turned. At the end both halves
are shut, the bar rests through both plates, and both grippers have let go. General maintenance and repair workers
(BLS 49-9071, about 1.5 million US jobs) and industrial machinery mechanics (BLS 49-9041, about 0.4 million) fit a
lockout like this on every valve they isolate before servicing a line (OSHA 29 CFR 1910.147), at each shutdown.

This is not a padlock or breaker lockout: there is no thumbscrew, lever clamp or padlock to fit, and no wrong-target choice. The ability is holding two sprung hinged halves shut with one hand while the other hand slides a bar through holes that
exist only when both halves sit on their stops. The bar is not a latch, a thread or a snap: nothing holds the halves shut
except the hand until the bar is through.

Physics, not the instruction, forces both arms. Each half is held open at 38 deg by a spring (0.10 N m/rad, about
4.7 N at a pad on the wall). Open, each end plate's hole is about 12 mm off the bar's line, so a bar pushed at the open
halves stops on the first plate's face. The bar cannot push the halves shut itself: its push is along the hinge axis,
perpendicular to the halves' swing. One hand therefore has to press both walls onto their stops (a single pinch across
the closed shell does both) while the other pushes the bar 18 mm through two plates with 1.25 mm of play. Left alone
with only the pinch, the halves spring back open the moment it lets go.

Instruction given to the policy (24 words; it states the goal only):

`Close the red clamshell around the valve's yellow lever, slide the steel bar through both holes until it rests in them, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo and
cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: 8c5821f22a93 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the valve as a PhysX articulation (a fixed body with a friction-held revolute lever and two
spring-driven revolute halves with limits, USD joints spawned through Isaac Lab's `ArticulationCfg`) and the bar and its
rib as compound rigid bodies, and calls the task's per-step record after every control step. No fork of yam_arena and no
third-party assets: every body is built from box and cylinder colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

22 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; doing nothing, the left hand alone
(halves shut and held, no bar) and the right hand alone (bar stopped on the open halves' first plate) each score 0; the
0.25 rung needs a held bar whose tip is 109 mm or less (108.9 passes, 109.1 does not) and an unheld bar earns nothing;
a lock made and then undone scores 0.5; the lever gate is no gate at 4.99 deg and a gate at 5.00 deg, and it beats a lock
at the end and is cleared only by a reset; the shut limit is 6.0 vs 6.1 deg for each half; the through limits (tip 99.0
vs 99.1 mm, centring 3.0 vs 3.1 mm, tilt 20.0 vs 20.1 deg); a half left at 6.5 deg is no lock; a bar at 4 mm/s or
turning 3.5 deg/s, or halves moving 2 deg/s, inside the last 2 s hold the score at 0.75 and the same fault 3 s before the
end does not; one pad step 29 mm from the valve restarts the release timer and 30 mm does not; a hand at 6 mm/s blocks
the release; the frame helpers and the bar's tip and plane crossings are checked from a fake pose; and the hole geometry
shows the holes sitting about 12 mm off the bar's line when open and within the bar's play only when shut.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Close the red clamshell around the valve's yellow lever, slide the steel bar through both holes until it rests in them, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/valve-lockout-clamshell-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with
`termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                             # scripted demo
python linkdemo.py task.py cheat_out --variant right_only       # a cheat variant (or DEMO_VARIANT=right_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the valve body's, the halves' and the bar's poses, the halves' angles computed from link
poses, and the robot's own grippers. It never reads joint values or the record.

1. Left: top-down with the gripper leaning 35 deg away from the right arm (about the pad axis, so the pads stay flat
   on the walls), fingers opened 80 mm, lowered beside the open halves 20 mm from the stem end of the walls, closed to
   the end of its travel. The pads press the two walls onto their stops (29 mm across) and it holds there.
2. Right, at the same time: top-down on the bar in its rib, pads across the 8 mm width 16 mm toward the cap from the
   bar's middle, gripper leaning 35 deg away from the left arm, fully closed. It measures the held bar's pose, lifts it
   60 mm and swings it to the clamshell's axis. Once the left hand has the halves shut it brings the bar's tip to 140 mm,
   then 119 mm, from the stem along the axis (5 mm before the first plate's face), correcting its lateral position
   and its axis direction from the seen bar pose until they are within 0.3 mm and 0.5 deg.
3. Right: pushes the bar through the first plate (112 mm), the second (104, 97 mm) to a tip at 91 mm, 6 mm short of the
   lever's tip, with the same correction at each stop. It lets go (pads opened to 24 mm), lifts 100 mm clear.
4. Left: lets go (pads opened to 80 mm), lifts 100 mm clear. Both wait.

The demo scores 1.0 (`success`, the episode ended at step 624 of 1,200): the lever never moved (peak 0.0 deg). At the
end the halves rest 4.7 and 3.1 deg off their stops (the springs wedge them on the bar), the bar's tip is at 94.8 mm
(9.2 mm past the second plate) and centred within 1.5 mm in both holes, tilted 12.5 deg on its play, and both
grippers are 91 and 87 mm clear. Worst IK error 0.09 mm and 0.02 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (8c5821f22a93); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. | 0.0 |
| `left_only` | The left hand pinches both halves onto their stops and holds; the right arm stays at the ready pose. | Both halves reach 0 deg; the bar never leaves its rib; when the hand lets go the halves spring open. | 0.0 |
| `right_only` | The left arm stays at the ready pose; the right hand carries the bar to the open halves along the demo's path. | The tip stops at 111.2 mm on the first plate's face (the holes are about 12 mm off its line), the arm cannot push on, and the bar is let go lying across the plate; the halves stay at 40 deg. | 0.0 |

## Build log (7 runs to the first 1.0, then the final runs)

| Run | Change | Result |
|---|---|---|
| 1 | first build: left squeeze to 27 mm, bar squeezed 0.6 mm, grippers upright | 0.25: the left pads lost the halves (they opened again within 0.5 s of closing) and the bar slipped in the right hand |
| 2 | both grippers commanded fully closed (12 mm and 0 mm) | 0.0: the halves held, but the two wrist modules met above the valve (right arm tracking error up to 357 mm) and the bar knocked the lever 8 deg: gate |
| 3 | recorded, with tracking logs | the same; the video shows the wrist links touching |
| 4 | tracking logs of both arms | the right arm lags 31 to 177 mm from the moment the left hand is in place |
| 5 | `right_only` (diagnostic) | the right arm alone tracks within 1 to 3 mm: the fault was arm against arm |
| 6 | grippers lean 35 deg away from each other, left pinch moved to 20 mm from the stem, bar pinch 16 mm toward the cap | 0.25: the bar went through to 98.8 mm, 6 mm short of command (the grasp was measured before the pads moved the bar) and, let go, tilted 13.1 deg against a 12 deg tolerance |
| 7 | grasp measured after closing, final tip 91 mm, tilt tolerance 20 deg | 1.0 at step 624 |
| final | thresholds for held-bar detection and the ladder (a left-hand-only run no longer earns 0.25); first-plate rung at 109 mm | demo 1.0 at step 624; `nothing`, `left_only`, `right_only` 0.0 |
| final 2 | lever stem friction raised from 0.15 to 1.2 N m (a real 1/2 inch valve); real-trial stillness checks restated as frame pairs on a taped rule | demo 1.0 at step 624, peak lever 0.0 deg; `left_only` 0.0; `right_only` 0.0 |

The change that mattered is run 6: with the grippers upright the two wrists collide at the middle of the table, which
looked like a lag in the right arm. The lean about the pad axis keeps the pads flat on the walls and the bar's sides.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures: the lever's
angle from shut; each half's angle from its stop; the bar's tip along the clamshell axis (mm from the stem), its centring
in both holes at the plates' mid-planes, and its tilt; whether a gripper holds the bar (pads within 6 mm of it, opening
at most 22 mm, which reads about 15.5 mm on the 8 mm bar) and whether one holds the halves; the bar's speed and spin from
its pose change between control steps, and the halves' angular rate; and each gripper's pad clearance to the valve, halves
and bar and its speed. The record is cleared only by the simulator reset.

- Gate, any time in the run, gives 0: the lever turned 5.0 deg from shut. The lever's friction is 1.2 N m; 5 deg is far
  outside its settle noise (the demo's peak is 0.0 deg) and a moved lever is never made safe by moving it back. It reads
  on a degree scale at the stem.
- 0.25: the held bar's tip went through the first plate (tip at 109 mm or less, centred within 3 mm), at some time.
- 0.5: shut and through at some time: both halves within 6 deg of their stops and the bar through (tip at 99 mm or less,
  5 mm past the second plate; centred within 3 mm in both holes; within 20 deg of the axis), but not so at the end.
- 0.75: shut and through at the end.
- 1.0: shut and through for the last 2 s, the bar under 3 mm/s and 3 deg/s and the halves under 1 deg/s, and the release
  gate: every pad point of both grippers at least 30 mm from the valve, halves and bar and both grippers under 5 mm/s for
  the last 1 s.

6 deg is the wedge limit (the bar fits both holes only while the halves are within about 5 deg of their stops, since the
holes spread by 2 x 14 mm x sin q across y); 20 deg is the bar's tilt on its play once released (the demo ends at 12.5).
`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score.

Real trial: the lever turn on a degree scale at the stem; each half's wall top within 3 mm of its stop on a 3 mm feeler
gauge (6 deg); a steel rule for the bar's tip past the second plate (at least 5 mm) and its centring in the holes (at most
3 mm each side); a protractor on the bar for its tilt (at most 20 deg); 60 fps video on a fixed tripod, comparing two frames 2 s apart against a taped steel rule and a protractor: the bar's tail cap moved under 6 mm and turned under 6 deg (3 mm/s, 3 deg/s over 2 s) and each wall top under 2 mm (1 deg/s); in the frame 1 s from the end, every part of both grippers at least 30 mm from the valve and bar and each gripper moved under 5 mm in the last second.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 624.
- Runtime on one L40S: a demo run took under 10 min wall clock on the shared VM (with other simulations running, plus about
  2 min of Isaac Sim start-up). A Dream-1 trial is slower, since Dream round trips pause
  the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (the valve, the clamshell and the
  rib are fixed or sprung and the bar is never jittered). The scripted demo is deterministic (four runs ended in the same
  state to 0.1 mm).

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 624. The video
  is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png` (frame 0 of
  the top camera).
- Dream-1 run: task_score 0.0, ended by max_steps after 1200 steps (3 min wall clock), log adhoc_d1ca9c4b.json, on this exact
  `task.py` (8c5821f22a93).

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials

- Lever ball valve: a 1/2 inch lever valve, brass body 40 x 36 x 34 mm (x, y, z) on a 21.4 mm brass pipe stub 200 mm long
  along x, held by two grey stand blocks, fixed to the table with the stem at (0.170, 0.090); about 1.0 kg with stands and
  base. The lever is yellow-coated steel, 95 x 20 x 4 mm, 50 g, on a vertical stem at z 0.8225, lying shut along y toward
  the right arm; stem friction 1.2 N m (a 1/2 inch lever ball valve takes about 1 to 3 N m; a spring scale at the lever tip reads about 13 N).
- Clamshell lockout: red ABS, 60 g a half. Each half is a 3 x 100 x 28 mm wall hinged on a horizontal pin along y at the
  lever's lower edge (z 0.8085, x 0.183 and 0.157) plus a 5 mm end plate 29 x 28 mm with a 10.5 mm square hole at the
  lever's height; plate A at y -0.014 to -0.019, plate B at -0.019 to -0.024. A spring of 0.10 N m/rad holds each half
  open at 38 deg (limit stops at 0 and 42 deg). Shut, the walls stand 1.5 mm from the lever's sides. Surfaces: friction
  0.25 / 0.2 (hinged halves), 0.6 / 0.5 (base). The base plate is grey ABS, 44 x 100 x 11 mm, fixed to the valve's bracket.
- Lock bar: steel, 8 x 8 x 60 mm, 43 g, with a red 16 x 16 x 4 mm cap on its tail; friction 0.4 / 0.3. It lies along y
  on the rib, tip toward the valve, centre (0.170, -0.100, 0.7635).
- Bar rib: dark grey polyethylene, 6 x 35 x 12 mm, fixed to the table at (0.170, -0.100); the pads straddle it.
- Layout and physics chosen for reach and feasibility: the valve sits 0.17 m ahead of the arm bases so both hands reach
  the clamshell's two ends (the left pinch is at y 0.07, the right hand's last position at y -0.05); the bar is a square-section steel lock bar with 1.25 mm play in square holes.

## Reset between trials

Nothing is consumed. Lay the bar on its rib along y with its tip toward the valve and its cap toward the right arm; let
both halves spring open to their 38 deg stops; set the lever shut (along y, against its stop mark) and check it turns
only above 1.2 N m. The valve, clamshell base and rib are fixed on taped outlines and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our own
code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public Isaac Lab
and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no third-party assets
or brands.
