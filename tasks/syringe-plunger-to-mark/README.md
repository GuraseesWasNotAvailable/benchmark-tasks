# Draw a 10 mL syringe's plunger back to the 5 mL line, the syringe resting on its tray

The robot draws the plunger of a standard 10 mL Luer syringe back until its stopper sits on the 5 mL line, with the
syringe resting on its tray at the end and both grippers let go. Registered nurses, licensed practical nurses and
medical assistants (about 3.2 million, 0.63 million and 0.74 million US workers, BLS May 2023) draw a plunger back to a
mark many times a shift. The syringe barrel lies loose on a stainless tray and the stopper's seal drags about 5 N,
about a hundred times the friction of the loose barrel on the tray.

The ability: hold a light free tube against a sliding seal's drag while drawing the seal to a graduation.

Both arms are needed, and the physics says why. The right arm grips the thumb rest and pulls. If the left arm does not
hold the barrel, the 5 N seal drag drags the whole syringe along the tray and the plunger does not move relative to
the barrel (simulated `right_only`, the syringe slid 25 mm, draw 0 mL). If the left arm holds the barrel and nothing
pulls the plunger, nothing moves (`left_only`). The soft arm adds a second difficulty: a 5 N pull makes the hand lag
the stopper by about 15 mm, so the stopper creeps to the hand until the pull falls to the seal drag, and the policy has
to settle and correct from what it sees to land within 1.5 mm.

Instruction given to the policy (24 words; it states the goal only):

`Draw the syringe's plunger back until its stopper sits on the 5 mL line, with the syringe resting on the tray, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`. `linkkit.py` and `linkdemo.py` are our own code; no file of
another team's task is reused.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo
and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: f0efd6462899 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the syringe as a PhysX articulation (a free barrel with a prismatic plunger joint that has
a Coulomb friction effort) and the tray as a fixed compound body, and calls the task's per-step record after every
control step. No fork of yam_arena and no third-party assets: every body is built from box and cylinder colliders in
`task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

34 tests feed synthetic records to the scorer (they need the yam_arena package for `TABLE_TOP`): a demo-shaped trace
scores 1.0; the mark window is 28.5 to 31.5 mm of draw (4.75 to 5.25 mL) and 28.4 or 31.6 mm are outside it; a plunger
drawn and pushed back to the line still scores; a plunger overshot to 8 mL and ending at 28.4 mm scores 0.25; never
drawn (nothing, left only, right only, or the whole syringe slid 25 mm) scores 0; the tray gate trips at 10.1 mm outside
the floor area and not at 9.9 mm and never clears except by a reset; the resting limits (axis 3.9 vs 4.1 mm off its
height, tilt 7.9 vs 8.1 deg either way, 4 mm/s of barrel speed, 0.2 rad/s of spin); resting needs 1 s (0.9 s is 0.5);
the release gate (30.1 vs 29.9 mm clear, 9.9 vs 10.1 mm/s of hand speed) and its timer restart after a touch; the
record is clean after a reset; and speed and spin come from the pose change between control steps.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Draw the syringe's plunger back until its stopper sits on the 5 mL line, with the syringe resting on the tray, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/syringe-plunger-to-mark-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). For repeat trials, add `--epochs K`; with `object_jitter=0` the seed changes nothing in the
scene.

```bash
inspect-robots inspect adhoc_c34fb841.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_c34fb841.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant right_only     # a cheat variant (or DEMO_VARIANT=right_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json`. The script reads only what a camera could measure: the barrel and
plunger poses (the draw is the plunger's offset along the barrel axis) and the robot's own grippers. It never reads a
joint value or the record.

1. Both arms, straight down with each gripper's body leaning 25 deg away from the other arm (the two grips are 100 mm
   apart, and upright grippers collide there). Left: pinches the barrel 85 mm past the flange, pads across the
   16 mm barrel, about 15 N a finger. Right: pinches the thumb rest across its 24 mm diameter.
2. Right pulls the thumb rest along the barrel axis to 3 mm short of the line (open loop), waits until the stopper
   stops creeping (under 0.06 mm per 0.4 s), reads the draw from the two poses, moves the hand by the error and waits
   again, until within 0.4 mm (two corrections here, 13.8 then 29.9 mm).
3. Right lets go and rises 70 mm; left lets go and rises 70 mm; both wait.

The demo scores 1.0 (`success`, episode ended at step 777 of 1,200): the stopper rests at 29.95 mm of draw (4.99 mL),
the barrel's axis is 14.1 mm above the table top (2.3 deg nose-down, resting on its flange and the barrel's far end),
it is inside the tray, and the grippers end 66 and 63 mm clear. Worst IK error 0.1 mm and 0.01 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (f0efd6462899); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves (220 steps). | 0.0 |
| `left_only` | Left grips the barrel and holds 10 s; right stays at home. | The barrel stays put; the draw stays 0.00 mL (480 steps). | 0.0 |
| `right_only` | Right grips the thumb rest and makes the same 24 mm pull; left stays at home. | The 5 N seal drag outweighs the barrel's friction, so the whole syringe slides 25 mm along the tray; the draw reads 0.01 mm at most (396 steps). | 0.0 |
| `pull_short` | Both arms as in the demo, but the target is 15 mm (2.5 mL). | The stopper settles at 15.1 mm (peak 17.2 mm, 2.87 mL), never reaching 4.75 mL (1,200 steps). | 0.0 |

On bracing: in `right_only` the tray's end wall is 98 mm beyond the flange, so the barrel cannot be pinned against it
by a pull of the demo's length; a pull of more than about 98 mm would be a different cheat, and the arm could not do
it without leaving the 0.35 m work zone.

## Build log (8 demo runs on the shared L40S, about 6 min each)

| Run | Change | Result |
|---|---|---|
| 1 | first build: grips 70 mm apart, grippers upright | 0.0: the two gripper bodies collided over the tray and pushed each other 70 to 80 mm off their targets |
| 2 | grips 100 mm apart (the barrel grip moved to 85 mm past the flange) | 0.0: still colliding (50 to 60 mm off) |
| 3 | each gripper leaned 25 deg away from the other | 0.25: the grips landed; the left wrist turn was out of reach (19 deg), so the barrel was dragged 23 mm sideways |
| 4 | the wrist turn each arm reaches (left and right differ by 180 deg), pad tips 9 mm below the axis | 0.25: both grips held, but the stopper crept toward the hand and overshot the line (6.5 mL) |
| 5 | found with a hanging-weight probe that the plunger joint's drag is a Coulomb effort of exactly 5 N (0.50 kg held, 0.52 kg fell, 0.40 and 0.60 kg agreed); set the sliding drag equal to the breakaway | 0.25: the same overshoot, so the cause is the arm, not the drag |
| 6 | wait until the stopper stops creeping before each correction | 1.0 at step 777 |
| 7 | recorded demo with `--check-reset` | 1.0, the same end state |
| 8 | final task file (comment fixes only), demo and the four cheats | 1.0; cheats 0.0 |

The finding that matters: the arm's hand compliance along the pull is about 0.3 N/mm, so a 5 N seal drag makes the
stopper stop about 15 mm short of the commanded hand position and approach it slowly. A policy that pulls open loop to
30 mm draws about 14 mm; one that keeps pulling without waiting overshoots; settling and correcting from the seen
stopper position lands within 0.2 mm.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures the plunger's
travel along its joint (draw, 6.0 mm per mL), the flange, nozzle and thumb-rest points, the barrel's axis height and
tilt, its speed and spin from the change of pose between control steps, and each gripper's pad distance to the syringe
and its speed.

Gate (any time in the run, cleared only by the simulator reset; a real hazard, since a sterile-field syringe that leaves the tray is contaminated and is discarded, as a nurse would, and a syringe that gets over the tray wall is no longer drawable at the tray): the flange, nozzle or thumb rest more than 10 mm
outside the tray's floor area gives 0 for good (a syringe knocked off the tray and put back does not clear it).

| Score | Condition |
|---|---|
| 0 | gate tripped, or the plunger was never drawn to 4.75 mL (28.5 mm) |
| 0.25 | drawn to 4.75 mL or more at some time, but the stopper is not within 5.0 +/- 0.25 mL (1.5 mm) at the end |
| 0.5 | stopper within 5.0 +/- 0.25 mL at the end, but the syringe has not rested still on the tray for 1 s |
| 0.75 | also resting for 1 s (axis within 4 mm of its height on the tray floor, within 8 deg of horizontal, under 3 mm/s and 0.1 rad/s, inside the tray's floor area), but a gripper is not released |
| 1.0 | also both grippers 30 mm clear of the syringe and under 10 mm/s for 1 s |

`SUCCESS_HOLD_S = 1.0` s. `distance()` is 1 - score.

Real trial, with the tools to check each condition:

- stopper's front edge on the 5 mL line: a ruler or the barrel's printed scale, within 1.5 mm (0.25 mL), photographed from above;
- seal drag: a spring scale hooked on the thumb rest, 4 to 6 N breakaway (choose a syringe that reads so);
- resting: the barrel's axis height with a ruler (within 4 mm of its resting height on the tray floor) and a protractor (8 deg);
- gate: tape on the tray marking the floor area plus 10 mm;
- stillness and release: 60 fps video; the hands 30 mm clear (a 30 mm spacer) and still for 1 s.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 777.
- Runtime on one L40S shared with other builds: a recorded demo run took about 6 min wall clock, including about 1 min
  of Isaac Sim start-up. A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout. The scripted demo is deterministic,
  and Dream-1 is a hosted model without a seed.

## Scored runs

| Run | inspect-robots-dream | yam-arena commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 0.1.0 | 0059d02a1a91 | f0efd6462899 | 1200 | 3 min | adhoc_c34fb841.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | - | 0059d02a1a91 | f0efd6462899 | 777 (success held 1 s) | about 6 min | demo_out/result.json | 1.0 | left clamped the barrel, right drew the stopper to 29.95 mm and let go, left let go; syringe resting on the tray |

- Demo: `python linkdemo.py task.py demo_out` writes `demo_out/result.json` with `task_score = 1.0`. The video is
  `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`), and the screenshot is `demo_out/setup.png`
  (frame 0 of the top camera).
- Dream-1 run: made at filing with the command above, on this exact `task.py`.

## Materials

- Instrument tray: fixed, brushed stainless steel, 300 x 160 x 20 mm, 2 mm sheet, floor and four walls, 0.4 kg, friction
  0.4, centred at (0.18, 0.00) with its long side along y. It is the common 12 inch instrument tray, rectangular
  rather than kidney-shaped.
- Syringe: a standard 10 mL Luer syringe, lying on the tray floor on its flange and thumb rest. Barrel: clear
  polypropylene, 16 mm across (modelled as 12 flat slats of 1 mm), 100 mm long to the end wall, a 10 mm Luer nozzle,
  finger flange 34 x 24 x 3 mm, 6.0 g, friction 0.5, painted marks at 0, 5 and 10 mL (6.0 mm per mL, the 5 mL mark red).
  Plunger: white polypropylene cruciform rod 9 mm across, black rubber stopper 13.4 mm across and 8 mm long, round thumb
  rest 24 mm across and 3 mm thick, 4.5 g, friction 0.5; fully in, the thumb rest is 14 mm behind the flange. It draws up
  to 62.5 mm. The seal drags 5 N in both directions.
- Layout: the syringe's flange face at (0.18, -0.050), nozzle toward the robot's left, so the two grips are 100 mm apart
  before the draw and 130 mm after; both grips are inside 0.35 m of their arm.
- Licences: `linkkit.py`, `linkdemo.py`, `task.py` and the tests are MIT, Copyright (c) 2026 the task authors. No
  third-party assets.

## Real-trial notes and reset

- Reset: push the plunger fully in, lay the syringe on the tray floor with its flange face 50 mm to the robot's right of
  the tray's centre line, axis along the long side, nozzle toward the robot's left, resting on the flange and thumb
  rest. Check the seal drag with a spring scale and wipe the barrel. Nothing is consumed.
- Fidelity: the barrel is a ring of flat slats, the stopper a rigid cylinder and the seal drag a joint friction; there
  is no liquid, air or needle. A real syringe's seal drag varies from one syringe to the next, so a real trial should
  pick syringes that read 4 to 6 N.
