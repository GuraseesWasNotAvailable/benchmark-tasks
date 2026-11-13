# Screw four propellers onto a quadcopter's motors, each the way its motor's thread runs

The robot screws the four propellers of a 4-inch quadcopter down onto their motors. Two motors spin clockwise and have
left-hand shaft threads (red bells, red hub caps); two spin anticlockwise and have right-hand threads (blue). Each prop
starts on the first turn of its thread, 4 mm above the bell, and tightens by turning the way its thread runs: the red
props anticlockwise seen from above, the blue props clockwise. Turned the other way a prop only spins on a plain
relief and, forced, is cross-threaded. At the end all four hub feet sit on the bells and both grippers have let go.
Drone builders and technicians fit props this way on every build and repair, and robot builders do the same on the
drones they use to collect data.

The ability is turning a small threaded part the way its thread runs, one stroke at a time, when the wrist cannot turn
a full circle. Physics, not the instruction, defeats the natural plan: turning all four props the same way tightens
two and cross-threads the other two. Pushing a hub straight down does nothing, because a thread only advances by
turning. The task is not placement: each prop is already on its motor, and the work is gripping a 16 mm hub, turning
it about 240 deg in strokes the wrist can reach, and following it down 4 mm.

Instruction given to the policy (21 words; it states the goal only, never the thread hand or the method):

`Screw all four propellers down onto the quadcopter's motors until each hub sits tight on its motor bell, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: 654f21464586 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision`
  in the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn each motor as a PhysX articulation (a fixed motor, a prismatic slide along the shaft and a
revolute hub, USD joints with limits and drives, spawned through Isaac Lab's `ArticulationCfg`). The frame is a compound
rigid body. No fork of yam_arena and no third-party assets: every body is built from box and cylinder colliders in
`task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

20 tests feed synthetic records to the scorer: four tight props with the release held score 1.0 and without it 0.8;
0.95 s of release is not enough; one to four tight props give 0.2 per prop; 3.6 mm of drop is tight and 3.5 is not;
a wrong-way turn of 179.9 deg passes and 180.0 deg is the gate, on any one of the four motors, and the gate beats a
finished quad and an empty one; the record is clean after a reset; and the layout checks (the two clockwise motors are
diagonal, the motors form a square, the thread geometry, the hub's foot 4 mm above the bell, neighbouring props do not
overlap). The tests need `yam_arena` importable (run them on the VM after sourcing `yam.env`).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Screw all four propellers down onto the quadcopter's motors until each hub sits tight on its motor bell, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/drone-prop-handedness-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). With `object_jitter=0` the seed changes nothing in the scene.

```bash
inspect-robots inspect adhoc_1673a005.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_1673a005.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant same_way_cw   # a cheat variant (or DEMO_VARIANT=same_way_cw)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json`. The script reads only what a camera could measure: each prop's pose
(hub height and yaw) and the robot's own grippers. It never reads joint values or the record.

1. Both arms at once, on diagonal motors so their wrists stay apart: the left arm takes the front-left prop and the
   right arm the back-right prop (both clockwise motors).
2. Per prop, up to five strokes: hover, lower the open fingers (34 mm) over the hub, close on its flats (1.5 mm
   squeeze command on a 16 mm hub), then turn the wrist 80 deg about the hub axis in the thread's direction while the
   hand follows the hub down by 6 mm x 80/360, open and lift. Each stroke starts from the hub's seen yaw, picking the
   grip orientation that keeps the wrist inside its range (about -120 to +70 deg on the left arm, mirrored on the
   right). The strokes end when the seen hub drop reaches 3.8 mm.
3. After each prop the hand parks 0.22 m out on its own side of the quad, so the other wrist can work the next
   diagonal motor: left back-left, right front-right.
4. Both hands open and wait with the grippers parked.

The demo scores 1.0 (`success`, episode ended at step 660 of 1,200): all four hubs end 3.8 mm down (foot on the
bell), no prop was turned against its thread, and the grippers were clear and still for 1.95 s at the end. Worst IK
error 0.08 mm and 0.02 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (654f21464586); every one scores
exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; no hub drops. | 0.0 |
| `same_way_cw` | The demo's strokes, but every prop turned clockwise. | The two blue props tighten. The two red props spin on their relief to 378 deg wrong-way and jam: the gate. | 0.0 (gate) |
| `same_way_ccw` | Every prop turned anticlockwise. | The two red props tighten; the two blue props go 378 deg wrong-way: the gate. | 0.0 (gate) |
| `press_down` | Closed fingers press each hub's top 8 mm down, no turn. | The threads hold the hubs: no drop, no prop tight. | 0.0 |

## Build log (9 demo runs on the shared L40S)

| Run | Change | Result |
|---|---|---|
| 1 | first build: 5 mm travel, 150 deg strokes | 0.0: the left wrist's yaw range (about -120 to +70 deg) cannot take 150 deg strokes; one prop dropped 3.4 mm |
| 2 | 4 mm travel, 6 mm lead, 80 deg strokes centred in each wrist's range | 0.0: both arms worked adjacent motors at once and their wrists collided |
| 3 to 4 | diagonal pairing (front left with back right) | 0.2 to 0.4: the first prop of each arm tightened; the second prop's approach was blocked |
| 5 to 8 | diagnostics (grip opening, tool error, clearance) | the hand left hovering over its first prop was in the other hand's path; tool error 80 to 110 mm at the second prop |
| 9 | park each hand 0.22 m out after its prop | 1.0 at step 660 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (a `linkkit.History` in `task.py`) measures each hub's drop (m below
its start; 4.0 mm = foot on the bell), its wrong-way turn (degrees against its own thread's direction), and a release
flag (both grippers at least 30 mm from every prop's colliders and slower than 10 mm/s). The record is cleared only by
the simulator reset, so unscrewing and starting again never clears a wrong-way turn.

- Gate, any time in the run, gives 0: any one prop turned 180 deg or more against its thread (cross-threaded).
- 0.2 per prop whose hub drop is at least 3.6 mm at the end (within 0.4 mm of the bell): 0.2, 0.4, 0.6, 0.8.
- 1.0: all four tight and the release flag held for 1.0 s.

The coupling that makes the threads: each control step, a hub's slide target is lead x turn angle / 360 for a thread
of its hand, clamped to 0 to 4 mm, and the slide's limits follow that target within 0.2 mm, so a hub can neither be
pushed nor pulled along the shaft without turning. A hub turned against its thread spins freely for 400 deg and then
meets a stop.

Real-trial measurement (the line the scorer matches):

- tight: a 0.4 mm feeler gauge does not enter between a hub's foot and its bell (checked after both hands are clear);
- wrong-way turn: a white dot on each hub cap read frame by frame from the top camera, summing each turn made against
  the thread's direction (180 deg is the dot reaching the opposite side of the hub);
- release: a ruler in the top frame shows 30 mm between each gripper and the nearest hub, and the gripper does not move
  for 1 s (30 frames at 30 fps).

## Scored runs

| Run | yam-isaac-lab-arena | Commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 1.0.0 | 0059d02a1a91 | 654f21464586 | 1200 | 3 min | adhoc_1673a005.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | git checkout | 0059d02a1a91 | 654f21464586 | 660 (success held 1 s) | about 12 min | result.json | 1.0 | four props tight, hands clear |
| `nothing` (`--no-record`) | git checkout | 0059d02a1a91 | 654f21464586 | 660 | about 8 min | result.json | 0.0 | nothing moved |
| `same_way_cw` (`--no-record`) | git checkout | 0059d02a1a91 | 654f21464586 | 894 | about 10 min | result.json | 0.0 | gate, 378 deg wrong-way on two |
| `same_way_ccw` (`--no-record`) | git checkout | 0059d02a1a91 | 654f21464586 | 894 | about 10 min | result.json | 0.0 | gate, 378 deg wrong-way on two |
| `press_down` (`--no-record`) | git checkout | 0059d02a1a91 | 654f21464586 | 382 | about 5 min | result.json | 0.0 | no hub moved |

## Versions, runtime and seeds

yam-isaac-lab-arena at commit 0059d02a1a91, inspect-robots 1.0.0, Isaac Sim, Isaac Lab and Isaac Lab-Arena as on the
program's launchable. The layout is fixed (`object_jitter=0`, seed 0). The demo reaches success at step 660 (33 s).

## Materials

| Prop | Real item | Size | Mass | Colour | Position (world, m) |
|---|---|---|---|---|---|
| Frame | 4-inch carbon-fibre cross frame with electronics stack and base plate, on a clamped 12 mm plywood bench plate | arms 190 mm tip to tip, 12 x 4 mm; stack 36 x 36 x 16 mm | 0.5 kg (fixed) | matt black | centre (0.170, 0.000); arm tops 24 mm above the table |
| Motors (4) | brushless outrunner, aluminium bell, M5 3-start shaft thread (2 mm pitch, 6 mm lead, 4 mm long) | 24 mm dia x 12 mm | 30 g each (fixed) | red (clockwise, left-hand thread) or blue (anticlockwise, right-hand thread) | front left (0.227, 0.057) red, back left (0.113, 0.057) blue, front right (0.227, -0.057) blue, back right (0.113, -0.057) red |
| Propellers (4) | 4-inch two-blade glass-filled nylon prop with a threaded square hub cap | hub 16 x 16 x 14 mm; blades 43 x 12 x 2 mm; 102 mm span | 8 g each | hub red or blue to match its motor; blades dark grey | hub centre 0.048 m above the table over its motor, foot 4 mm above the bell |

A thread gauge and a spare prop check each motor before the trial: with the thread, a hub advances 1 mm per 60 deg and
reaches the bell after 240 deg; against it, it spins freely. All colliders are kit primitives (boxes and cylinders);
there are no third-party meshes, textures or assets.

## Reset notes

Unscrew every prop to its starting position: hub foot 4 mm above the bell (a 4 mm spacer under each hub, removed
after), hub corners aligned to the arm directions. Check each shaft's thread hand and that a spare prop spins freely
against the thread. Wipe the threads with a drop of light oil and put a white dot on each hub cap. The frame and bench
plate are clamped on taped outlines and need no reset. Nothing is consumed.

## Licence

`task.py`, `test_scoring.py`, `linkkit.py` and `linkdemo.py` are our own code, MIT-licensed (header in each file).
Nothing is copied from cablekit, partkit or scenekit.
