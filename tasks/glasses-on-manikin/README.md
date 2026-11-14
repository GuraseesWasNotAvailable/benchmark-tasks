# Put a pair of reading glasses on a training manikin's head, bridge on its nose and both temples over its ears

The robot picks up an open pair of black reading glasses, one hand on each temple near its hinge, and puts them on a
training manikin head. At the end the bridge rests on the nose, both temples lie over the ears with their tips behind
them, and both grippers have let go and are clear. Anyone with a tremor, stiff
hands or a stroke needs someone else to put their reading glasses on, and a robot helper at home would be asked to do
exactly this; the head stays where it is, so the glasses go to the head.

The ability is guiding the two hinged arms of one object over two separate features at once while lowering it into
place. Nothing is latched, screwed or plugged in: the pair is simply worn.

Both hands hold the pair in the demo because the temples turn on friction hinges whose axes are vertical: a gripper on
one temple holds the frame by a single hinge, like a door on one pin, and the other temple has nothing holding it. The
first lift swings it shut (the `left_only` and `right_only` variants both score 0: the unheld temple folds 77 to 80 deg
and the pair ends on the table). We also tried a one-hand grip of the bridge from above; the YAM arm could not reach the
wrist orientation that needs (54 to 70 deg orientation error), so that is untested rather than excluded. The score is
the end state only, never which hand does what. The head is 145 mm wide and the hinges are 133 mm apart, so the temples
have to open a few degrees to pass the widest part of the head (the demo spreads them first); the pair then goes in
12 mm above its seat, slides back over the nose and is lowered onto the nose and ears.

Instruction given to the policy (23 words; it states the goal only):

`Put the black reading glasses on the training manikin's head, bridge on its nose and both temples over its ears, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the
submitted runs:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: d9b328afde7e (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the glasses as one free PhysX articulation (a frame link and two temple links on vertical
revolute hinges with limits and friction), and the head and the rest block as fixed compound bodies, all built from
box and cylinder colliders in `task.py`. It also calls the task's per-step record after every control step. No fork of
yam_arena and no third-party assets.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

39 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; a temple folded in 45 deg or 75 deg and
opened again scores on the final state only, and one still folded at the end is not worn; 0.95 s of stillness scores 0.75 and
1.05 s scores 1.0; a bridge 3.0 mm above the nose top is worn and 3.1 mm is not (2 mm below is the lower limit), 6.0 mm
off the nose's centre line is worn and 6.1 is not, 8.0 mm along the nose is worn and 8.1 is not; a tip 3.0 mm behind the
ear's rear edge is worn and 2.9 is not; a tip 72.5 or 92.5 mm out lies over the ear and 72.4 or 92.6 does not; a tip
4.0 mm above the ear top is worn and 4.1 is not; a temple folded in 3.0 deg is open and 3.1 is not; both grippers must hold the pair for the 0.25 rung
and the frame must be 40 mm up (39 mm earns nothing); the bridge reaching the nose gives 0.5; a pad 29 mm from the pair
restarts the release timer and 30 mm does not; a gripper at 6 mm/s or the pair at 3.1 mm/s or 3.1 deg/s blocks 1.0; the
record is clean after a reset; and speeds come from the pose change between control steps (0.2 mm in one 50 ms step is
4 mm/s).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the black reading glasses on the training manikin's head, bridge on its nose and both temples over its ears, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/glasses-on-manikin-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

```bash
inspect-robots inspect adhoc_ab34c736.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_ab34c736.json
```

## Scored runs

| Run | yam-isaac-lab-arena | Commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | git checkout | 0059d02a1a91 | d9b328afde7e | 1200 | 3 min | adhoc_ab34c736.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | git checkout | 0059d02a1a91 | d9b328afde7e | 404 (success held 1 s) | about 5 min | result.json | 1.0 | worn, still, both grippers 36 mm clear |

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                            # scripted demo
python linkdemo.py task.py cheat_out --variant left_only      # a cheat variant (or DEMO_VARIANT=left_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads
only what a camera could see: the pair's starting pose and the robot's own grippers. It never reads joint values or
the record.

1. Both arms, mirrored: top-down grippers leaning 25 deg outward (the two hands are 133 mm apart and collide when
   upright), pads closing across the 3 mm temple 30 mm behind its hinge; start 60 mm outboard and up, then down onto
   the temple; a 0.6 mm squeeze.
2. Both lift the pair 100 mm, then turn each temple 7 deg outward about its hinge (each hand arcs about its own hinge
   while the frame stays put).
3. Both carry the pair in front of the face at 12 mm above its seat, slide it 25 mm back so the bridge plate is over
   the nose and the temples (spread 7 deg) pass the head's widest part, then lower it to 0.5 mm above its seat.
4. Both open the pads, wait 0.4 s and back away 70 mm outboard and up; both wait.

The demo scores 1.0 (`success`, episode ended at step 404 of 1,200). At the end the bridge underside is 0.0 mm above
the nose top, 1.3 mm along it and 4.5 mm off its centre line; the left tip is 9.3 mm behind its ear's rear edge, 84.5 mm
out, resting on the ear top, and the right tip 12.7 mm behind, 85.4 mm out; the left hinge reads 3.7 deg and the right
11.7 deg open past square (the head's width spreads them); no temple was folded in more than 0.9 deg; the pair is still
and both grippers are 36 mm clear. Worst IK error 0.09 mm and 0.01 deg; a second reset returns every body to its start
(history cleared, joints at 0.0 deg).

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (d9b328afde7e); every one scores
exactly 0 (`nothing`, both arms at the ready pose, is covered by the offline test `test_nothing_scores_zero`):

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `left_only` | The left arm runs its half of the demo; the right arm idles at home. | The pair hangs from one hinge. The unheld right temple swings shut (77 deg at its peak, 36 deg at the end) and the pair ends on the table 0.21 m to the left of the head, bridge 116 mm below the nose top. | 0.0 |
| `right_only` | The right arm runs its half; the left idles. | The same by symmetry: the unheld left temple folds to 80 deg at its peak (59 deg at the end) and the pair ends 0.08 m to the right of the head, 124 mm below the nose top. | 0.0 |

Neither variant is stopped by a rule; the pair is simply not on the head at the end, so the ladder stops at 0.

## Build log (5 demo runs, 3 to 5 min each on the shared L40S)

| Run | Change | Result |
|---|---|---|
| 1 | first build: grippers straight down, pads across the temples | 0.0: with the hands 133 mm apart the two grippers touched; the arms stopped 88 mm short of the temples |
| 2 | diagnostic: a trace of the gripper's measured point | confirmed the arms never reached the grasp |
| 3 | grippers leaning 25 deg outward, 14 mm opening | 0.25 at 15 s: both held the pair, carried it and spread the temples (diagnostic run, stopped at 300 steps) |
| 4 | full run | 1.0 at step 404 |
| 5 | final task file of the first pass (one-line edit to the scorer's constructor, `--check-reset`) | 1.0 at step 404, the same end state to 0.01 mm |
| 6, 7 | audit pass: a one-hand bridge grip attempted twice (lean 20 deg and 0 deg) | unreachable (IK error 70 and 54 deg); that variant was dropped |
| 8 | demo with the audit task file (the permanent fold gate removed; only the final state is scored) | 1.0 at step 404, worn to 0.01 mm |
| 9, 10 | `left_only`, `right_only` with the audit task file | 0.0 each |

A sideways approach (gripper horizontal, pads above and below the temple) was ruled out before any run: the arm cannot
reach it (IK error 17 to 163 mm).

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures: the bridge
plate's underside centre against the nose top (dx along the nose, y across it, dz above it); each temple tip's bottom
against its ear (mm behind the ear's rear edge, lateral position, mm above the ear top); each hinge angle (deg folded
inward); whether a pad is within 4 mm of the pair; the pad clearance to the pair and the head; the pair's and each
temple's speed and spin from their pose change between control steps; and each gripper's speed.

- No fold rule: a temple that folded on the way and was opened again costs nothing, only the final state counts (a
  temple still folded at the end fails the 3 deg open limit of "worn").
- Worn: bridge within 8 mm along and 6 mm across the nose and from 2 mm below to 3 mm above its top; each tip at least
  3 mm behind the ear's rear edge, 72.5 to 92.5 mm out from the head's centre line (over the ear), from 2 mm below to
  4 mm above the ear top, and its hinge folded in at most 3 deg.
- 0.25: both grippers touched the pair (a pad within 4 mm) while it was 40 mm above its start.
- 0.5: the bridge reached the nose at some time.
- 0.75: worn at the end.
- 1.0: worn for the last 1 s, the pair still (under 3 mm/s and 3 deg/s) for the last 1 s, and both grippers at least
  30 mm from the pair and the head and moving under 5 mm/s for the last 1 s (release gate).

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: tape for the bridge and tip offsets, a 3 mm
feeler gauge under the bridge and temples, a protractor against the frame's perpendicular for each hinge, and 60 fps
video for stillness (under 3 mm/s), the 30 mm release gate and the hinge angles.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 404.
- Runtime on one L40S: a demo run took about 5 min wall clock on the shared VM, plus about 2 min of Isaac Sim start-up.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout. The scripted demo is deterministic
  (runs 4 and 5 ended in the same state to 0.01 mm); Dream-1 is a hosted model without a seed.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 404. The video
  is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png`.
- Dream-1 run: score 0.0, max_steps, 1200 steps, 3 minutes, log adhoc_ab34c736.json.

## Materials

- Training manikin head: hard vinyl over rigid foam, skin tone, 2.0 kg, fixed to the table on a 20 mm neck stub, 145 mm
  wide, 190 mm front to back, 230 mm chin to crown, facing the robot; nose a 10 mm wide flat-topped ridge, top 115 mm above
  the chin; two ears 28 x 55 mm, 20 mm proud of the skull, flat tops 2 mm above the nose top. Friction 0.6.
- Reading glasses: black acetate, 25 g. Front 140 mm wide (brow bar 140 x 4 x 6 mm, clear lenses 50 x 36 x 2 mm, bridge
  plate 24 x 12 x 4 mm; 14 g). Temples 138 x 3 x 8 mm, 5.5 g each, on friction hinges 133 mm apart (0.6 mN m static
  friction, a temple stays where it is left), open stop 12 deg past square, free to fold inward to 95 deg. Friction 0.5.
- Rest block: pine, 40 x 150 x 34 mm, 0.5 kg, fixed centre (0.100, 0); the pair starts with its lenses on the table and
  its temples across the block.
- Building the real pair: take a plain acetate frame with 5-barrel hinges, remove any hinge spring and file each
  temple's stop so it opens to 102 deg (12 deg past square; a protractor against the frame's perpendicular reads
  -12). Tighten each hinge screw, with a 0.1 mm PTFE washer under its head, until a temple released at 45 deg stays
  there and a spring scale at the tip, 69 mm from the pin, reads about 5 mN to move it. The hinges are 133 mm apart.
  Ears: two blocks of 20 mm rigid foam 28 x 20 x 55 mm glued 20 mm out from the head's sides, 72.5 mm to either side of
  the centre line, tops 2 mm above the nose top. Nose: a 10 x 35 mm hardwood strip glued to the face, its top flat and
  115 mm above the chin; check the three heights with a depth gauge.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.
- The hinge friction check is the spring scale or the 45 deg test above.

## Reset between trials

Nothing is consumed. Lay the glasses open with the lenses on the table and the temples across the rest block, the frame
centred at x 0.020 and the temples square to it (a protractor reads 0 deg). Check that a temple pushed to 45 deg stays
there. The head and the block are fixed on taped outlines and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
