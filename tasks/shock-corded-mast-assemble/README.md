# Assemble a shock-corded antenna mast

The robot seats the joints of a three-section shock-corded mast one after another, each pushed past the point where
the internal cord stops drawing it in, then rests the finished pole in a two-saddle transit cradle. Field radio and
tower installers (O*NET 49-2021.00) do this when they put up sectional antenna masts and shock-corded whips, and the
finished pole goes in a cradle while the antenna head is fitted. Tent poles and deployable booms use the same ferrule
joints. The mast starts on the table folded in a zig-zag. For each joint the robot swings the hinged section into
line until its spigot can enter the ferrule at all, lets the cord draw it part of the way in, and pushes it fully
home. At the end both joints are fully seated, the mast rests in the slots of both stands, and both grippers have let
go.

The ability is sequential seating along an elastic-linked chain: line each hinge up within 3 deg so its spigot can
enter, then push the joint past the elastic's short stop, joint by joint. Each seated joint turns two sections into
one rigid piece that is swung and pushed as a unit at the next joint. Unlike TK-2K8E9N, TK-NSB3C4 and TK-RB5CWP, where
one rigid part is steadied while a single insert is pushed home, the parts here are a folded, cord-linked chain worked
joint by joint, and the elastic decides how far each joint goes on its own. Steadying the chain with the second arm
only enables each push. Nothing is placed until the end: the cradle takes only the finished pole, and a pole with an
unseated joint scores 0 on it.

Physics, not the instruction, defeats the natural wrong plans:

- **A bent joint cannot take its spigot.** A spigot enters its ferrule only within 3 deg of straight. Bent past
  22 deg, its tip rests on the ferrule's rim.
- **The cord stops short.** Lined up and let go, the cord draws each spigot in, but the snug fit in the last 15 mm
  stops it 11.7 to 15 mm short of home. Pulling both ends straight stretches the cords and opens both joints.
- **An unbraced push only shoves the mast along the table.** Pushing a spigot home takes 1.75 to 2.25 N against the
  fit, but the rest of the mast slides on the table at about 0.5 N. One arm pushing alone gets no joint closer than
  3.3 mm (cheat table below).

Instruction given to the policy (24 words; it states the goal only, never the method):

`Assemble the green shock-corded mast into one straight pole with every joint fully seated, lay it across the two grey stands, then let go.`

Files in this folder: `task.json`, `task.py` (scene, ferrule model, whole-run record, scoring and the scripted demo),
`linkkit.py` (articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab). Versions used for the demo and the
cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0 with dreamlabs-client 1.1.3 (for the Dream-1 trial)
- task file SHA-256, first 12 hex: b93b16af5687 (`sha256sum task.py | cut -c1-12`; equals
  `eval.environment_revision` in the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that lets
`yam_arena.scene` spawn the mast as a PhysX articulation. The mast is five links: three sections and two hidden cord
anchors. Each joint has a prismatic slide (the cord, a sprung USD drive) and a revolute hinge about the vertical. The
hook spawns the stands as a fixed compound body and calls the task's per-step code after every control step. No fork
of yam_arena and no third-party assets: every body is built from box colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

The 25 tests feed synthetic records to the scorer:

- A demo-shaped trace scores 1.0. A lined-up mast drawn in by the cords but never pushed home (gaps 13 and 14.5 mm)
  scores 0, even resting on the stands.
- Seated thresholds: a 1.0 mm gap is seated and 1.1 mm is not; a 1.0 deg bend is seated and 1.1 deg is not.
- On-stands thresholds: 6.0 vs 6.1 mm across the slot, -2.0 / +4.5 vs -2.1 / +4.6 mm above the slot floor, and
  12.0 vs 12.1 mm along it.
- One joint seated once and then pulled apart still scores 0.25. Both seated once and then pulled apart scores 0.5,
  and both seated but on the table scores 0.5.
- Release gate: 0.95 s of release is not enough, one pad step at 29 mm restarts the timer, and a hand moving at
  6 mm/s scores 0.75. A mast moving at 6 mm/s on the stands is not resting.
- The record is clean after a reset.
- The ferrule's entry and lock limits, the rim offset of a bent spigot, the cord and fit forces, and the stick-slip
  transitions are checked.
- measure() on real link states built from the mast's kinematics: the assembled mast centred over the stands
  measures 0 mm everywhere. Rolled 45 deg onto an edge it sits 3.7 mm higher and is still in the slots. On the table
  it is 40 mm low. Resting on a cheek it is not in the slots. The start pose as spawned (each bent spigot's tip on its
  rim, 3.5 mm outside the 26 mm mouth) reads 29.5 mm gaps and -60 / +60 deg bends, with the top section at x 0.222.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Assemble the green shock-corded mast into one straight pole with every joint fully seated, lay it across the two grey stands, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/shock-corded-mast-assemble-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`. Otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). For repeat trials add `--epochs K`; with `object_jitter=0` the seed changes nothing in the
scene. Before the scored run, probe the instruction with a 40-step run (`--max-steps 40`, no frames) to rule out a
refusal.

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant pull_straight  # a cheat variant (or DEMO_VARIANT=pull_straight)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz. It writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the three sections' poses, and from them each joint's bend and the gap at each ferrule
mouth. It also reads the robot's own grippers. It never reads joint values, the hidden cord anchors or the record.

1. Both arms at once. The left arm grasps the middle section near joint 1. The right arm grasps the top section 76 mm
   from its spigot tip. Every grasp is top-down with the pads across the tube. Each grasp re-reads the section's
   pose just before the descent. It corrects the YAM's 10 to 20 mm wrist sag from the arm's own TCP, measured in
   free air above the tube and again just before closing.
2. The right arm swings the top section about its spigot tip until it lines up with the middle section (60 deg, then
   two closed-loop corrections to under 0.4 deg). The cord draws the spigot in to 14.6 mm short. The right arm then
   pushes it along the middle section's axis. Each push adds the gap it sees plus 4 mm to the command, because both
   soft arms give way about 5 mm per newton. The gap goes 14.6, 8.0, 3.9, 1.5, then 0.05 mm.
3. The right arm eases off: it moves its command to its measured TCP plus the free-air sag, so it stores no force.
   It lets go, then the left lets go. The left grasps the base section near its foot. The right grasps the middle
   section 50 mm from joint 1.
4. The right arm swings the middle and top sections (now one rigid piece) 32 deg into line about joint 1's spigot tip.
   The cord draws it to 14.8 mm short, and the pushes close it 14.8, 8.5, 3.7, 2.1, then 0.14 mm.
5. The right arm eases off and the left arm lets go. The right arm carries the mast alone (it holds 15 mm from the
   mast's middle), so no pull between two grips can open a joint. In one rigid move planned from the seen foot and
   cap, it lifts the mast 60 mm, turns it 26.8 deg, carries it 36 mm back and 383 mm to the right over the stands,
   and lowers it into both slots. It eases off, lets go and backs away. Both arms wait.

The demo scores 1.0 (`success`, episode ended at step 949 of 1,200, 47.5 s). At the end both gaps are 0.35 and
0.34 mm and both bends 0.3 deg. The mast lies in both slots, 1.5 mm off their centres, its centre line 3.5 mm higher
than a tube lying flat on the slot floor (consistent with the square tube resting tipped onto an edge). Both grippers
are 339 and 82 mm clear. Worst IK error 0.09 mm and 0.02 deg. A second reset after the run (`--check-reset`) puts
every joint back at its start value (0.000 error) with the base section within 0.0 mm of its start, and clears the
record.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (b93b16af5687). Every one scores exactly
0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | The mast stays folded: gaps 29.5 mm, bends -60 and +60 deg. | 0.0 |
| `pull_straight` | The left grasps the base section near its foot, the right the top section near its cap. The right pulls the mast straight, then both arms carry it over the stands and let go. | Pulled straight with both joints open (gaps 25.7 and 23.7 mm). During the two-arm carry the joints pull open and bend, so once let go each spigot sits at its ferrule mouth (26 mm gaps, bends 8 and 19 deg). The mast lies across one slot and one cheek. No joint ever came closer than 19.4 mm. | 0.0 |
| `align_only` | The demo without the pushes: each joint is swung into line and the cord draws it in, then the mast is laid on the stands. | The cords stop the spigots 14.8 and 13.4 mm short. The mast lies straight in both slots with gaps of 13.2 and 13.1 mm. | 0.0 |
| `one_arm` | The right arm alone: it lines up and pushes each joint with nothing holding the other part, then regrasps the middle and carries the mast to the stands. | Each push shoves the mast along the table. Joint 2 stops at 7.1 mm and joint 1 at 4.0 mm (closest 3.3 mm). The mast lands on the table beside the stands. | 0.0 |

## Build log (11 demo runs on the shared L4, 4 to 12 min each)

| Run | Change | Result |
|---|---|---|
| 1 | first build: centre-pivot hinge, ferrule fit as PhysX joint friction | 0.0. During the settle the bent spigots hit the ferrule walls, and the 60 deg bends relaxed to 29 and 46 deg. Pushes moved a joint only 1 mm each. |
| 2 | a bent spigot's tip rests on the rim, just outside the mouth (`rim_offset`) | 0.25. The start pose held. The left grasp closed above the middle section (TCP 20 mm off). Pushes stalled again. |
| 3 | grasp 6 mm higher, wait before closing; fit modelled as stick-slip instead of joint friction | 0.0. Both phase-1 grasps missed: the soft wrist leaves the TCP 16 to 24 mm off sideways. |
| 4 | free-air servo checks before closing; grips spread apart; softer cord and fit (150 N/m, 3 N) | 0.25. Joint 2 seated (0.05 mm), then popped to 2.5 mm when the pusher let go mid-push. The phase-2 grasps missed because the mast moved after it was seen. |
| 5 | re-read the pose just before each descent; ease off before letting go | 0.5. Both joints seated. The two-arm carry pulled joint 1 out to 1.1 mm, and the mast lay 20 deg skewed and missed one slot. |
| 6 | ease-off includes the arm's sag; one-arm carry; the carry turns the mast onto the stands' line | 0.5. The joints stayed at 0.3 mm, but stands at x 0.46 were out of the right arm's reach at carry height (TCP 45 to 57 mm off), so the mast landed on the table. |
| 7 | stands moved to x 0.40, beside the right half of the mast's line; carry height 60 mm | 1.0 at step 950. |
| 8 | same layout, comment text only; then the cheats | 1.0 at step 950. But `one_arm` dragged the loose mast against a stand and used it as an anvil to seat joint 1 (0.25). |
| 9 | stands moved to (0.30, -0.30) and (0.30, -0.45), off the right end of the mast's line, away from everything that swings or slides | 0.5. The mast was in both slots with its centre line 3.5 mm above flat, as for a tube tipped onto an edge. The slot-floor tolerance (+3 mm) rejected it. |
| 10 | slot-floor tolerance +4.5 mm: an 18 mm square tube on an edge sits up to 3.7 mm high, while one on the cheek tops sits 12 mm up | 1.0 at step 949, the same end state as run 9 to 0.01 mm. All four cheats score 0. |
| 11 | the mast's colour from olive to a clear green, so it matches the instruction's "green mast"; the start-pose test built at the spawned start (29.5 mm) | 1.0 at step 949, the same end state as run 10 (the colour changes only the render). The four cheats score 0, as in run 10. |

Two findings matter for anyone using linkkit:

- PhysX articulation joint friction, as Isaac Lab writes it, is a coefficient on the joint's reaction loads. It is
  not a fixed effort. Under the arms' side loads the slide jammed (run 1), so the fit is emulated as stick-slip on
  the slide's drive (`fit_next`).
- The YAM's 10 N m/rad wrist gives about 0.2 N/mm at the TCP. The cord and fit forces are sized to it, and every
  hand-off eases the arm's stored force first.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures:

- each joint's gap at its ferrule mouth (mm, 0 = home) and its bend (deg);
- where the mast's centre line crosses each stand: offset across the slot, tube bottom above the slot floor, and the
  nearest centre-line point's offset along the slot;
- the mast's speed, from its sections' pose change between control steps (what a camera sees);
- each gripper's pad clearance to the mast, and its speed.

The record keeps flags, lows and held times for the whole episode. Only the simulator reset clears it, so pulling a
joint apart and starting again never clears what happened.

- Fully seated: a gap of at most 1.0 mm and a bend of at most 1 deg.
- On the stands: at each stand the centre line is within 6 mm of the slot centre, the tube bottom is -2 to +4.5 mm
  from the slot floor, and the nearest centre-line point is within 12 mm of the stand along the slot. A square tube
  tipped onto an edge sits up to 3.7 mm high; one resting on the cheek tops sits 12 mm up.
- 0.25: one joint fully seated at some time.
- 0.5: both joints fully seated at the same time, at some time.
- 0.75: at the end both joints are fully seated and the mast is on both stands and still (under 5 mm/s).
- 1.0: 0.75 held for the last 1 s, plus the release gate. The release gate: every pad point of both grippers at
  least 30 mm from the mast and both grippers under 5 mm/s for the last 1 s.

`score()` returns the rung and `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s). `distance()` is 1 - score.

Real trial: a feeler gauge (or a top photo with a mm scale) at each ferrule mouth for the gap, a straightedge along
the mast for the bend, a top photo for the mast in both slots, and the video for stillness and release.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 949.
- Runtime on the shared L4: the final demo run took 4 min 15 s of wall clock, including about 2 min of Isaac Sim
  start-up and the reset check (4 control steps/s). Earlier runs took up to 12 min while two other simulations
  ran. The Dream-1 trial, on an L40S, took 3 min 43 s of wall clock for its 1,200 steps, start-up included.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (the stands are fixed and the mast
  is never jittered). The scripted demo is deterministic: runs 9, 10 and 11 ended in the same state to 0.01 mm. Dream-1 is
  a hosted model without a seed.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`; `demo_out/result.json` gives `task_score = 1.0`, success at step 949.
  The videos are `demo_out/top_cam.mp4`, `left_cam.mp4` and `right_cam.mp4`. The screenshot is `demo_out/setup.png`
  (frame 0 of the top camera).
- Dream-1 run: made at filing with the command above, on this exact `task.py` (SHA-256 prefix b93b16af5687, equal to
  `eval.environment_revision` in its log). Dream-1 did not refuse the instruction and ran the full 1,200 steps:
  `task_score = 0.0`, ended by `max_steps`.

## Building the snug fit

The last 15 mm of each ferrule holds a compliant insert: a black nitrile rubber sleeve (Shore 60A, 1.1 mm walls) bonded inside the bore, which narrows it to an 11.8 mm square opening the 12 mm spigot squeezes into, lightly greased with silicone. Check every joint with a 0 to 10 N spring scale pulling the spigot along the axis: it must start to move at 3 N (2.7 to 3.3) and keep sliding at 2.5 N (2.2 to 2.8); trim or replace the sleeve until it does. So the insert holds the spigot until 3 N net acts on it and drags at 2.5 N while it slides. The simulator models the insert as the 3 N breakaway and 2.5 N sliding resistance above.

## Materials

- Mast: three sections of green glass-fibre tube, sRGB (0.20, 0.50, 0.22), 18 mm square, 130 mm long, 45 g each,
  friction 0.5 / 0.45. The base section has a 10 mm black rubber foot and the top section a 10 mm orange cap.
  Assembled length 390 mm.
- Joints: an aluminium ferrule socket on the lower section, 26 mm deep with a 14 mm bore (2 mm walls). A 12 mm square
  aluminium spigot, 26 mm long, on the upper section. An internal shock cord: 150 N/m, 0.75 N at home, 4.65 N with
  the spigot at the mouth.
- The snug fit: the last 15 mm of each ferrule holds the spigot until 3 N net acts on it, and drags at 2.5 N while it
  slides. The hinge is a free pivot about the vertical at the spigot tip (light PhysX friction coefficient 0.003),
  limited to 70 deg. A spigot enters only within 3 deg of straight. When the joint is bent past 22 deg, the spigot's
  tip rests on the rim up to 4.4 mm outside the mouth. Once the spigot is in, the joint is held straight: 3 deg at
  the mouth, 0.3 deg from 20 mm in.
- Real equivalent at this scale: a shock-corded fibreglass tent-pole repair kit (aluminium ferrules and elastic cord)
  cut to three 130 mm sections, or a three-section shock-corded field whip. The cord and fit forces are sized for the
  YAM's soft wrist (about 0.2 N/mm at the TCP), lighter than a full-size mast.
- Transit cradle: the finished pole rests in it while the antenna head is fitted. Two grey anodised aluminium
  saddle stands fixed to the table at (0.30, -0.30) and (0.30, -0.45), in front
  of the right arm and past the right end of the mast's line. Each is a 40 x 24 x 40 mm block with two 6 mm cheeks
  forming a 28 mm wide, 12 mm deep slot along y; the slot floor is 40 mm above the table. 0.2 kg each, friction
  0.5 / 0.45.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Pull each joint apart and lay the mast back in its zig-zag on the taped outline:

- the base section along its line with the foot on its mark;
- joint 1 bent 60 deg toward the robot;
- joint 2 bent 60 deg back, so the top section lies parallel to the base section;
- each spigot tip resting on its ferrule's rim (a 60 deg template card at each joint).

The stands are fixed on taped outlines and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques. They copy no code from other tasks in this repository, so no kit credit is
needed. The scene uses no third-party assets or brands.
