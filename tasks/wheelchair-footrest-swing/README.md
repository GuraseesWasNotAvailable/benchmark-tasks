# Swing a wheelchair's swing-away footrest out of the way and lift it off its hinge pin

The robot clears a wheelchair's swing-away footrest so a person can transfer. One hand holds the spring release latch
down while the other swings the footrest hanger 90 deg to the right about its vertical hinge pin. Then the hanger is
lifted off the pin and lowered until it rests on the bracket's parking stop, and both hands let go. Nursing assistants
(BLS 31-1131, about 1.3 million US workers) and home health and personal care aides (31-1120, about 3.7 million) do
this at nearly every transfer to a bed, toilet or car.

The ability is holding a release open through a swing: the latch only frees the hanger while it is pressed, and the
moment it is let up the pawl drops into the notch plate and locks the hanger where it stands. Nothing is pushed in or
pulled out along a line. The swing is an arc about a pin, and the lift off the pin is allowed only once the hanger is
fully swung out.

Physics, not the instruction, forces both arms:

1. Latch hand. The pawl is locked whenever the latch pad is up (under 3 mm). A pull on the hanger with the pad up
   only deflects it about 3 deg against the lock (`right_only`).
2. Swing hand. Pressing the pad frees the hanger but nothing turns it (`left_only`). The pad (130 mm left of the pin)
   and the grip point on the tube (75 mm along it) are about 150 mm apart, and the 94 mm gripper cannot press the pad
   and hold the tube at once.
3. Let up early and the pawl locks the hanger again mid-swing (`early_release`, locked at about 31 deg).
4. The keeper lug blocks lifting until the hanger has swung out 85 deg (`lift_first`).

Instruction given to the policy (28 words; it states the goal only, never the latch or the keeper):

`Swing the wheelchair's blue footrest hanger out to the side, lift it off its hinge pin, and lower it until it rests on the bracket, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the
demo and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: cc83e3f70614 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the bracket as a PhysX articulation (USD revolute and prismatic joints with limits and
drives, spawned through Isaac Lab's `ArticulationCfg`) and calls the task's per-step record after every control step.
No fork of yam_arena and no third-party assets: every body is built from box and cylinder colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

18 tests feed synthetic measurements to the scorer: a demo-shaped trace scores 1.0; nothing scores 0; the swing rungs
at 44.9 vs 45.0 deg and 84.9 vs 85.0 deg; a swing seen once is kept after the hanger returns (0.25); the lift window
38 to 44 mm (37.9 and 44.1 mm fail); the end swing 90 +- 5 deg; the hanger still for 1 s (0.95 s, 4 mm/s and 3.1 deg/s
fail); the release gate at 30 mm (29 mm fails), 5 mm/s and 1 s; one step of pad contact restarts the release timer; a
reset clears the record; pose-change speed and spin; and the left-only, right-only and early-release records score 0.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Swing the wheelchair's blue footrest hanger out to the side, lift it off its hinge pin, and lower it until it rests on the bracket, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/wheelchair-footrest-swing-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). With `object_jitter=0` the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant left_only      # a cheat variant (or DEMO_VARIANT=left_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json`. The script reads only what a camera could measure: link poses (the
latch pad's travel, the hanger's swing and lift measured from poses) and the robot's own grippers. It never reads the
record or the pawl and keeper state.

1. Both arms together. Left: closes its fingers, goes over the latch pad and presses it down (pad tips 16 mm below the
   pad's top; the pad's 8 mm stop is reached), and holds. Right: top-down on the tube 75 mm from the pin, pads closing
   across the tube (1 mm squeeze); it waits until the pad is seen pressed 7 mm.
2. Right arcs the grip point 90 deg to the right about the pin (wrist turning with it, 5 s), then corrects from the
   seen swing angle (to within 1 deg). Left lets go once the swing is seen at -88 deg or more and rises 80 mm.
3. The pawl locks the hanger at -89.5 deg. Right re-grips (4 mm squeeze), lifts 65 mm (the hanger rises 54 mm), lowers
   until the lift is seen 2 mm above the 40 mm stop, opens, backs off along its approach and rises 50 mm.

The demo scores 1.0 (`success`, episode ended at step 496 of 1,200): the latch pad was held at 7.9 mm from about 5 s
to 10.5 s, the hanger reached -89.5 deg, rose to 54.5 mm and settled on the stop at 40.0 mm, and both grippers were
80 mm clear and still at the end. Worst IK error: left 10.7 mm and 2.7 deg, right 127.8 mm and 32.4 deg (during the
swing the right wrist is near its reach limit; the arm lags the command but the closed loop still lands the swing).

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (cc83e3f70614); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. | 0.0 |
| `left_only` | Left holds the latch pad down for 25 s; right idle. | Hanger stays at 0 deg. | 0.0 |
| `right_only` | Right grips the tube and arcs it 90 deg without the latch. | The pawl holds: 2.6 deg of give at most. | 0.0 |
| `early_release` | Left lets the pad up at -25 deg; right keeps pushing. | The pawl locks the hanger near -31 deg (lowest -33.5 deg). | 0.0 |
| `lift_first` | Left holds the pad; right grips and lifts the hanger at 0 deg. | The keeper holds: the lift stays 0.0 mm while the swing is under 85 deg. | 0.0 |

## Build log (4 demo runs, 5 cheat runs, one at a time on the shared L40S)

| Run | Change | Result |
|---|---|---|
| 1 | first layout (pin at 0.12, -0.04; pad spring 800 N/m) | 0.0: the right arm could not reach the hanger (IK error 153 mm) and the stiff pad only went 2.7 mm |
| 2 | pin moved to (0.15, -0.15), pad moved 130 mm left of the pin (reachable by the left arm), pad spring 400 N/m, press 16 mm | 0.5: swung to -89.5 and lifted to 40.0 mm, but "still" and the release gate never counted |
| 3 | scorer trace extended | the same: the record's clock was ticked twice per step (a zero time step) |
| 4 | the record no longer ticks the clock itself | 1.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, `update()` feeds a linkkit `History` (cleared only by the simulator reset): the swing
(hanger yaw about the pin, 0 latched, negative to the right), the lift above the seated height, the latch pad's travel,
the hanger's speed and spin from its pose change between steps (what a camera sees), the nearest pad point of either
gripper to the fixture, and the faster gripper's speed.

- 0: the swing never reached -45 deg.
- 0.25: swung out at least 45 deg at some time (the low is kept for the whole run).
- 0.5: swung out at least 85 deg at some time.
- 0.75: at the end the swing is 90 +- 5 deg, the lift 38 to 44 mm (clear of the 35 mm pin and on the 40 mm stop) and the
  hanger still (under 3 mm/s and 3 deg/s) for 1 s.
- 1.0: also released: every pad point of both grippers at least 30 mm from the fixture and both grippers under 5 mm/s
  for 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung, and `distance()` is 1 - score. Real trial: a
protractor scale on the bracket (swing, 5 deg marks); a steel rule from the seat plate to the sleeve's bottom edge
(lift, 38 to 44 mm); a spring scale on the latch pad (about 4 N to press) with a depth mark at 6 mm; a 30 mm spacer
block held against each gripper's pads for the clearance; and 60 fps video with a 1 mm scale for stillness (a mark on
the footplate moves under 3 mm in the last second) and for gripper speed.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 496.
- Runtime on one L40S, shared with other builders: a demo run took about 5 to 9 min wall clock (2.5 to 5.7 control
  steps/s, plus about 2 min of Isaac Sim start-up). A Dream-1 trial is slower, since Dream round trips pause the
  simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (the bracket is fixed and the
  hanger starts seated). The scripted demo is deterministic, and Dream-1 is a hosted model without a seed.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`. The video is
  `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png` (frame 0 of
  the top camera).
- Dream-1 run: made at filing with the command above, on this exact `task.py`.

| Run | inspect-robots-dream | yam-arena commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 0.1.0 | 0059d02a1a91 | cc83e3f70614 | 1200 | 3 min | adhoc_0cdbcc09.json | 0.0 | max_steps |
| Scripted demo (`linkdemo.py`) | - | 0059d02a1a91 | cc83e3f70614 | 496 (success held 1 s) | about 5 min | demo_out/result.json | 1.0 | latch held, swung to -89.5 deg, lifted 54 mm, set down on the stop at 40.0 mm, both grippers 80 mm clear |

```bash
inspect-robots inspect adhoc_0cdbcc09.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_0cdbcc09.json
```

## Materials

Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

- Front-frame bracket: powder-coated steel, dark grey, 3.0 kg, bolted to the table. A 200 x 340 x 8 mm base plate, a
  30 x 30 mm post 100 mm tall, and a 90 x 320 x 10 mm seat plate (top face 100 mm above the table). The hinge pin is
  silver steel, 16 mm across and 35 mm tall above the seat, at (0.15, -0.15).
- Footrest hanger: blue powder-coated steel with an aluminium footplate, 0.80 kg in all (a typical swing-away hanger
  with its sleeve and footplate weighs 0.8 to 1.2 kg). A 28 mm across, 60 mm tall sleeve on the pin, a 25 mm across
  tube 288 mm long pointing forward with its axis 45 mm above the seat, and the footplate flipped up at the tube end,
  4 x 120 x 80 mm. The legrest is raised to horizontal so that it fits the 0.35 m reach. Friction 0.8 / 0.7.
- Release latch: a red lever pad 28 x 28 x 14 mm at (0.15, -0.02), 8 mm travel, 400 N/m with a 3 mm preload (about
  4 N at the stop). 6 mm of travel frees the pawl and under 3 mm locks it (the pawl is a stiff brake of 100 N m/rad).
- Keeper and parking stop: the hanger lifts only after 85 deg of swing (the keeper lug lines up with the slot in the pin
  collar); it rises 35 mm to clear the pin top, then rests on a stop 40 mm above the seat.

## Reset between trials

Nothing is consumed. Set the hanger back on the pin, seated on the seat plate with its tube pointing forward (+x, 0
deg). Let the latch pad spring up, so the pawl locks the hanger. Check that the pad rests 22 mm above the seat plate and
that a 5 N push at the footplate does not turn the hanger.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
