# Zig-zag fold a four-panel array wing whose hinges fold alternate ways

The robot zig-zag folds a chain of four hinged panels, whose hinges each fold only one way with the direction
alternating from hinge to hinge, into one flat stack: a four-panel solar-array wing (a ground-test model) on its
stowage plate. The root panel P1 is bolted to the plate; P2, P3 and P4 lie deployed beyond it on a ground-support rail.
At the end P2 lies cells-down on P1, P3 back-to-back on P2, P4 cells-down on P3, every hinge folded flat, the panels
square over each other, and both grippers have let go. A folding carpenter's rule and a folding exhibition screen close
the same way; avionics technicians (O\*NET 49-2091) and aerospace technicians (17-3021) fold array-wing ground-test
models onto their stowage plates this way.

The ability is the zig-zag fold of a serial hinged chain whose hinges fold alternate ways. Nothing is placed or
inserted and nothing latches: what is tested is choosing, at each hinge, which side turns over, read from the face its
knuckle is on, and turning it under control while the rest of the chain follows. It is not closing a carton's parallel
flaps in order (TK-LABKXT, pilot TK-S2WMM8), unfolding a two-section ramp (TK-W67CJE), lowering one hinged cover without
slamming it (TK-9JFBH4) or breaking an over-centre strut's knee to stow one panel (TK-YBH54G): there each panel turns on
its own single hinge, here every fold drags the rest of the chain and the fold direction alternates along it.

Physics, not the instruction, defeats the natural plan. The hinges are piano hinges with their knuckles on alternate
faces: h1 (P1-P2) and h3 (P3-P4) on the cell face, h2 (P2-P3) on the back. A hinge folds up to 180 deg toward its
knuckle face and stops 12 deg past flat the other way. The cameras see this: from above, h1 and h3 show a steel knuckle
and leaves across the blue cell faces, while h2 shows only a thin white seam, its knuckle being underneath. Folding from
the tip, every fold the same way (P4 over onto P3, then that pair over onto P2) jams at h2: it cannot fold toward P2's
cell face, so lifting the pair only tips P2 up with it about h1 as one piece. The fold that works turns P2 over onto P1
(h1 +180, cells to cells), which drags P3 back to back onto P2 (h2 -180), and then turns P4 over onto P3 (h3 +180). The
hinges carry only 0.003 N m of friction, so a panel turned well past upright (about 115 deg for a bare panel) and let go
falls onto the stack at several hundred deg/s, which cracks cells (the gate): each panel has to be lowered.

Instruction given to the policy (21 words; it states the goal only, never the fold order or direction):

`Fold the four-panel array wing flat into one stack on the stowage plate, panels square over each other, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `demo_out/` (the demo's videos, screenshot and result), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab). Versions used for the demo and cheat
runs below (on an L4 GPU):

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: febe19df55a1 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision`
  in the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the wing as a PhysX articulation (a fixed root link, the stowage plate with P1 bolted on it,
and three revolute hinges with limits and joint friction, USD joints spawned through Isaac Lab's `ArticulationCfg`), the
rail as a fixed compound body, and calls the task's per-step record after every control step. No fork of yam_arena and
no third-party assets: every body is built from box colliders in `task.py`. `task.py` also wraps linkkit's spawner to
draw each hinge's steel knuckle and leaves and the panels' borders as plain USD shapes with no collision API, under the
wing's link prims (looks only: physics, the demo and the scorer see the same colliders as without them).

## Offline check, no simulator

```bash
python -m pytest -q tests
```

27 tests, all passing, feed synthetic records to the scorer and pose the wing from hinge angles: a demo-shaped trace
scores 1.0; a panel at 180.1 deg/s is a gate and 180.0 is not, and the gate beats a perfect stack at the end and is
never erased by restacking; the zig-zag rung needs h1 >= 170 and h2 <= -170 together (169.9 or -169.9 gives 0, and each
hinge on its own at different times gives 0); P4 turned to 95 deg on the zig-zag gives 0.5 and 94.9 gives 0.25; the
same-way trace (P4 onto P3, then P2 tipped up with h2 on its +12 deg stop) gives 0; each hinge at 176.9 deg at the end
gives 0.5 and 177.0 gives 1.0; a gripper 29 mm from the wing, a gripper at 5 mm/s, 0.95 s of release, or a panel at
3 deg/s or 3 mm/s gives 0.75; unfolding P4 at the end drops to 0.5; the record is clean after a reset; the pose-change
rate (9 deg in one 50 ms step is 180 deg/s); a 20 g panel let go at 120 deg reaches the stack well over the gate, while
the friction holds one parked at 92 deg; and the geometry: deployed, every panel lies flat on the plate or rail; stowed
(h1 +180, h2 -180, h3 +180), the four panels sit in four 4 mm layers exactly over P1 with cell faces up, down, up,
down; h2's stop is at +12 deg; with P2 upright, P3 folded on its back ends just outside h1; and the drawn hinges: each
knuckle sits on its hinge axis on its knuckle face (cell face at h1 and h3, the back at h2), from above h2 shows a white
seam, and the wing's colliders are unchanged (the plate and three boxes on P1, two on every other panel).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fold the four-panel array wing flat into one stack on the stowage plate, panels square over each other, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/array-wing-zigzag-stow-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.
Probe the instruction first with a 40-step run (`--max-steps 40`, no frames) to rule out a refusal.

```bash
inspect-robots inspect adhoc_964619bc.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_964619bc.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out --max-steps 1200 --check-reset   # scripted demo, then a second reset
python linkdemo.py task.py cheat_out --max-steps 1200 --variant drop  # a cheat variant (or DEMO_VARIANT=drop)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the panels' poses and the hinge angles computed from them, and the robot's own grippers.
It never reads joint values or the record.

1. Right: pinches P2 20 mm in from its free edge, top-down, the pads closing along the hinge line on the panel's two
   60 mm-apart side faces (0.6 mm squeeze), and pivots it about h1 to 92 deg in 4 s with the wrist following half the
   turn, correcting once from the seen angle (91.4 deg). The tail follows: P3, dragged by h2, folds onto P2's back
   while P4 slides along the rail. The right lets go and backs away; with the tail's weight on it P2 settles back to
   78 deg, held there by the tail resting on the rail and the hinge friction.
2. Left (while the right returns to its ready pose): pinches P2's top end, with P3 sandwiched against it, top-down, and
   lays it from 78 deg down to 179.5 deg about h1 in 4 s (two corrections), then lets go and backs away. P2 lies on P1
   and P3 on P2; P4 hangs from h3 onto the rail at -5.8 deg.
3. Right (while the left returns): pinches P4 20 mm in from its free edge and turns it about h3 to 92 deg; it stays
   there on the hinge friction when the right lets go.
4. Left (while the right returns): lays P4 from 92 deg down onto P3, lets go and backs away; both wait at their ready
   poses.

The demo scores 1.0 (`success`, episode ended at step 909 of 1,200): the zig-zag was reached at 18.1 s, P4 was turned
past upright at 35.8 s, and at the end h1 = 180.0, h2 = -180.0 and h3 = 180.0 deg, the stack still for 4.9 s and both
grippers clear and still for 2.0 s. The fastest panel turn in the whole run was 84 deg/s (P3 folding onto P2's back),
under half the gate. Worst IK error 0.08 mm and 0.02 deg. Reset check (`--check-reset`): after a second reset every
hinge is back at 0.0 deg, the wing at rest (peak 0.2 mm/s over 10 steps), the plate and rail unmoved and the record
cleared.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the task file above (febe19df55a1); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; the wing stays deployed (ended after 110 steps). | 0.0 |
| `same_way` | From the tip, every fold the same way: P4 turned over onto P3 by the same handover as the demo, then the P3/P4 pair, pinched near h3, turned up about h2 toward P2's cell face. | P4 lies on P3 at 22 s (h3 180 deg). Lifting the pair drives h2 onto its +12 deg stop; P2 tips up with it to 7 deg about h1, the pinch slips off and the pair falls back onto the rail at 307 deg/s: gate at 29.5 s. h2 never folds back to back (lowest 0 deg), so no rung either. | 0.0 |
| `drop` | The demo's folds, but the laying hand lets go at 125 deg instead of lowering each panel onto the stack. | P2 (with P3 on its back) falls onto P1 at 290 deg/s: gate at 20.1 s; P4 later falls onto P3 at 367 deg/s. The wing ends as a flat zig-zag stack (h1 180, h2 -180, h3 180, both hands clear), which would score 1.0 without the gate. | 0.0 |

## Build log (demo runs on the shared L4 VM)

| Run | Change | Result |
|---|---|---|
| 1 | first build | 0.0: the wing's fixed root link sat 40 mm off the linkage origin, and the simulator reset puts a fixed root body at the linkage pose, so the whole wing spawned 22 mm low and P2 to P4 ended up under the rail; the pinches closed on nothing. The arms' park pose (TCP 200 mm up) was out of reach (IK error 487 mm). |
| 2 | linkage frame moved to P1's centre (root link at the origin); arms park at their ready pose | 1.0 at step 909 |
| 3 | the `drop` cheat now lets go from the laying hand at 125 deg (its first version, a 120 deg turn by the right alone, lost its pinch at 66 deg), and `same_way` ends after the jammed lift; demo unchanged | 1.0 at step 909, the same end state as run 2 |
| 4 | final task file: each hinge drawn as a steel knuckle and leaves on its knuckle face, and the panels' borders as seams on the other face (looks only, no colliders), so the cameras show which way each hinge folds; physics, demo and scorer unchanged | 1.0 at step 909 with every rate, angle and hold time the same as run 3; the three cheats still score 0 with the same gate times and peaks |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures the three hinge
angles (h1 and h3: 0 flat, +180 folded cells to cells; h2: 0 flat, -180 folded back to back), the fastest panel turn
rate and speed from each panel's pose change between control steps (what a camera sees), and each gripper's pad
clearance to the wing and the rail and its speed. The record keeps flags, peaks and held times for the whole episode and
is cleared only by the simulator reset, so unfolding the wing and folding it again never clears a fall.

- Gate, any time in the run, gives 0: a panel turned faster than 180 deg/s. A panel let go past upright reaches the
  stack at several hundred deg/s (in the `drop` cheat, 290 and 367 deg/s from 125 deg; a bare 20 g panel from 120 deg
  without drag, about 750 deg/s); the demo's peak is 84 deg/s. The gate covers every panel at all times, P3 included
  as it swings back to back onto P2 while P2 is turned over: a panel that reaches the stack that fast strikes it as
  hard whether it was let go or flung, so the lift has to be paced too (the demo's 4 s lift brings P3 over at
  84 deg/s).
- 0.25: at some time h1 >= 170 and h2 <= -170 together: P2 turned over onto P1 with P3 back to back on it, the
  three-panel zig-zag. Folding P4 onto P3 first earns nothing on its own.
- 0.5: at some time, with that zig-zag held, h3 >= 95: P4 turned past upright toward the stack.
- 0.75: at the end, one flat stack: h1 >= 177, h2 <= -177 and h3 >= 177. Each panel is then within 3 deg of flat and
  square over P1 (the hinges keep the panels aligned across the wing), the stack top 16 mm above the plate.
- 1.0: that stack still (every panel under 3 deg/s and 3 mm/s) for the last 1 s, and the release gate: every pad point
  of both grippers at least 30 mm from the wing and the rail and both grippers moving under 5 mm/s for the last 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: a feeler gauge or protractor at each hinge for
flatness (3 deg is 4 mm at the free edge), a top photo for squareness, a 30 fps video for the panel rates and the
hands' clearance.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 909.
- Runtime on the shared L4 VM: a demo run took about 8 min wall clock (2.3 to 2.7 control steps/s with another
  simulation running, plus about 2 min of Isaac Sim start-up). A Dream-1 trial is slower, since Dream round trips pause
  the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (the plate, rail and P1 are fixed and
  the wing is never jittered). The scripted demo is deterministic; Dream-1 is a hosted model without a seed.

## Submitted videos and scores

| Run | inspect-robots | yam-arena commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Demo | 1.0.0 | 0059d02a1a91 | febe19df55a1 | 909 | about 8 min | `demo_out/result.json` | 1.0 | success |
| Dream-1 | 1.0.0 | 0059d02a1a91 | febe19df55a1 | 1200 | 3 min | adhoc_964619bc.json | 0.0 | max_steps |

- Demo: `python linkdemo.py task.py demo_out --max-steps 1200 --check-reset`, `demo_out/result.json`: `task_score = 1.0`, success at
  step 909. The video is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot
  `demo_out/setup.png` (frame 0 of the top camera).
- Dream-1 run: made at filing with the command above, on this exact `task.py`, after a 40-step probe of the instruction.

## Materials

- Stowage plate: grey anodised aluminium block, 80 x 50 x 20 mm, 216 g, fixed to the table (x 0.18 to 0.26, centred on
  y 0). The root panel P1 is bolted on top of it, cells up.
- Array wing: four panels P1 to P4 of a ground-test model, each 80 x 60 x 4 mm, 20 g: an aluminium-skinned foam core
  with a white composite back (3 mm) and a dark blue printed cell face (1 mm), friction 0.5 / 0.4. Deployed in a line
  along +x (P2 at x 0.26 to 0.34, P3 0.34 to 0.42, P4 0.42 to 0.50), overhanging the rail 5 mm on each side, which
  leaves room for the pads.
- Hinges: steel piano hinges along the 60 mm edges, 0.003 N m friction, no springs or dampers. h1 and h3 have the
  knuckle on the cell face, h2 on the back; each folds up to 180 deg toward its knuckle and stops 12 deg past flat the
  other way (chamfered panel edges). Drawn as a steel knuckle 3.5 mm across and 56 mm long on the hinge line with a
  5 mm leaf on each panel, on the knuckle face; on the other face the panels' 1.5 mm borders show as a seam (white
  between the cell faces, grey between the backs). These are looks only, with no colliders.
- Ground-support rail: grey aluminium bar with a PTFE top strip, 245 x 50 x 20 mm, 660 g, friction 0.25 / 0.2, fixed
  (x 0.262 to 0.507), its top level with the plate's. The plate and rail are 20 mm high so the hand that lays a panel
  down onto the stack, its wrist turned about 45 deg, clears the table.
- Scale and simplifications: the wing is about 1/12 of a real one; a stacked panel rests on its hinge's 180 deg limit,
  since panels sharing a hinge do not collide in the simulator (the others do). Positions, colours and friction for
  every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Unfold the wing flat along the rail, cells up: lift P4 off P3, then P2 (with P3) off P1, and lay
P2, P3 and P4 in line on the rail with every hinge straight (a straightedge along the backs). P1 stays bolted to the
stowage plate; the plate and the rail are fixed on taped outlines and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
