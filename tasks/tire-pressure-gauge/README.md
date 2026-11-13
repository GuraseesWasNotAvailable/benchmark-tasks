# Read a wheel's tyre pressure with a pencil gauge pressed square onto the valve, then put the gauge back

The robot takes a pencil tyre gauge from its holder, presses its chuck flat and firm onto the valve of a pneumatic
hand-truck wheel, holds it until the stick has drawn out to the tyre's pressure, and puts the gauge back upright in its
holder. At the end the stick reads 33 to 37 (the tyre is at 35 psi), the gauge is home and still, and both grippers
have let go. Anyone with a car, bicycle, wheelbarrow or sack truck does this; a courier or warehouse picker does it on a
soft wheel.

The ability is a square, firm press onto a small stem as the way to make a seal. The gauge's chuck has a 12 mm square
socket that drops over an 8.5 mm valve stem with 1.75 mm to spare each side, and its rubber face has to sit flat on the
valve's spring-loaded collar. Pressed at an angle or too lightly, air hisses out and the stick only draws out to 12 psi.
Nothing is placed, locked or fastened: the seal exists only while the press is square, firm and held.

Physics, not the instruction, defeats the natural plan. A tilted press can still get the socket over the stem and push
the collar, but the gasket never seals, so the stick stalls low and the run scores 0. A dab that does not push the
collar down 2 mm reads nothing, and a press that is let go early draws the stick only part way.

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo and
cheat runs below:

- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: 157787adf336 (`sha256sum task.py | cut -c1-12`)
- inspect-robots-dream 1.0.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial; yam-arena commit
  0059d02a1a91; task file 157787adf336 equals `eval.environment_revision` in the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the wheel (with its valve collar on a sprung slider) and the gauge (with its sliding stick) as
PhysX articulations, and calls the task's per-step record after every control step. No fork of yam_arena and no
third-party assets: every body is built from box and cylinder colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

12 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; nothing, an angled press (5 deg, stick at
12 mm), a light press (collar 0.6 mm), a short seal (1.0 s) and a stick pulled out by hand with no seal all score 0; a
held seal with the stick at 31 mm scores 0.4; a gauge left on the valve, or a gripper still holding at the end, scores
0.7; an interrupted press restarts the 1.5 s timer; a reset clears the record; and the seal and leak limits (3 deg,
2 mm collar, 1 mm collar).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Read the tyre pressure of the wheel with the pencil gauge, then put the gauge back upright in its holder." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/tire-pressure-gauge-dream
```

Scoring is automatic: `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end (`SCORE_OVER = "end"`).

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                       # scripted demo
python linkdemo.py task.py cheat_out --variant angled     # a cheat variant (or DEMO_VARIANT=angled)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json`. The script reads only what a camera could measure: the gauge's pose and
the robot's own gripper. It never reads joint values, the collar's depth or the record.

1. Right arm: approaches the gauge in its holder with the approach 45 deg downward (the arm cannot reach a level
   approach this low), pinches the 12 mm barrel (8 mm of squeeze command), records the grasp from the seen pose and
   lifts it 90 mm.
2. Carries it over the valve, hovers with the chuck 22 mm above the collar, and nulls the seen position error to
   0.3 mm; lowers in two steps (servo at 10 mm above the collar), so the socket drops over the stem.
3. Presses down until the collar bottoms out (2.5 mm travel, commanded 8 mm below its top; the arm's soft joints give
   only a couple of newtons), holds 3 s (the stick draws out over the first 1.5 s of sealed press), then lifts.
4. Carries the gauge back over its holder, hovers 30 mm above, nulls the seen error, lowers into the pocket, lets go,
   backs away 80 mm, rises 60 mm and waits.

The demo scores 1.0 (`success`, 987 of 1,200 steps): sealed for 4.05 s, stick at 34.1 mm at the end, gauge back to
0.0 mm in its holder and still, the grippers clear. Worst IK error 0.1 mm and 0.01 deg. A second reset left the wheel,
gauge and holder at their initial values and cleared the history.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (157787adf336); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. | 0.0 |
| `angled` | The full demo with the gauge held 5 deg off the valve's axis. | The socket drops over the stem and pushes the collar, but the gasket never seals: the stick stalls near 12 mm and no sealed press is ever held. | 0.0 |
| `light` | The full demo, pressed only 0.4 mm below the collar's top. | The collar goes down well under 1 mm: no seal, no reading. | 0.0 |
| `quick` | Presses squarely but lifts after about 1 s. | Sealed for 0.35 s at best, so the stick never draws out; no sealed press of 1.5 s. | 0.0 |

## Build log (demo runs on the shared L40S)

| Run | Change | Result |
|---|---|---|
| 1 | level approach, round barrel | 0.0: the level approach is out of the arm's reach (IK error 215 mm), the pads closed on the chuck head and the gauge tilted 15 to 25 deg |
| 2 | approach 45 deg downward, 12 mm square barrel, 25 mm holder pocket | 0.0: the grasp held (tilt under 1 deg), but the socket missed the stem by 2.4 mm and landed on its tip |
| 3 | hover 22 mm above the collar with a 0.3 mm servo, a second servo mid-descent, socket widened to 12 mm | 0.0: socket on the stem, but the arm's soft joints pushed the collar only 0.6 mm |
| 4 | collar spring 2.4 to 0.8 N/mm, 3 mm squeeze command | 0.0: the gauge slid 1 mm in the pads and the collar went down only 0.9 mm |
| 5 | press commanded 8 mm below the collar so it bottoms out | 0.4: sealed for 4 s, but the 100 N/m stick spring stalled at 31 mm against 0.5 N of friction; the return missed the holder |
| 6 | stick spring 1,000 N/m, hover and servo above the holder before lowering | 1.0 |
| 7 | the four cheats; `quick` first scored 1.0 because its slow lift kept the seal on past 1.5 s | `quick` shortened (0.3 s press and lift), then 0 |
| 8 | final run with a reset check | 1.0, the same end state |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures the gauge's tilt
from the valve's axis, the offset of its socket from the stem, the collar's depth, the stick's extension, the gauge's
pose and speed (from its pose change between steps) and each gripper's pad clearance and speed. The record is cleared
only by the simulator reset.

A press is sealed while the collar is down at least 2.0 mm, the tilt is at most 3 deg and the socket is within 2 mm of
the stem, continuously. While sealed, the stick is drawn toward 35 mm in proportion to the time sealed (full at 1.5 s);
a leaking press (collar down at least 1.0 mm, not sealed) draws it only to 12 mm, and nothing ever pushes it back in.

- 0: no sealed press of 1.5 s at any time in the run.
- 0.4: a sealed press of 1.5 s was held.
- 0.7: and the stick ends between 33 and 37 mm.
- 1.0: and the gauge ends within 6 mm across and 4 mm up of its holder position, within 4 deg of upright and still
  (under 2 mm/s) for 1 s, and the release gate holds: every pad point of both grippers at least 30 mm from the gauge and
  both moving under 10 mm/s for 1 s.

`success()` is true on the 1.0 rung and `distance()` is 1 - score. Real trial: read the stick against its 1 mm per psi
marks with a steel rule at the end (33 to 37). The sealed press and its 1.5 s come from a fixed side video at 30 frames
per second, with a plumb line and a 3 deg printed wedge in frame for the tilt, and a 2 mm mark on the collar post for the
collar depth; the stick's rise during the hold is the visible check that the seal held. The gauge in its holder is read
from a top photo with a 6 mm square drawn round the pocket and a level on the gauge's barrel; the grippers' clearance
from the video.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo ends at step 987.
- Seeds: `--seed 0` for every run. `-E object_jitter=0` fixes the layout. The scripted demo is deterministic.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`; `demo_out/result.json`: `task_score = 1.0`, 987 steps. Video
  `demo_out/top_cam.mp4` (with `left_cam.mp4`, `right_cam.mp4`) and `demo_out/setup.png`.
- Dream-1 run: made at filing with the command above on this exact `task.py` (157787adf336): 1200 steps, 11 minutes,
  log adhoc_7a3c8669.json, task_score 0.0, ending max_steps.

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials and how to build and check each part

- Wheel: a 10 inch pneumatic hand-truck wheel, rubber tyre 260 mm across by 70 mm wide on a steel rim, 1.4 kg, lying
  flat with its axle at (0.275, -0.100) m, chocked by two wooden blocks and a taped outline so it cannot move. Inflate to
  35 psi and confirm with a reference gauge before each session.
- Valve stem: brass Schrader stem, 8.5 mm diameter, 19 mm above the upward rim face, 55 mm from the axle at
  (0.220, -0.100) m. The flat rim face is a turned plate bolted over the rim, with the stem fitted through it.
- Seat collar: a 26 mm square, 3 mm tall red anodised plate with a 9 mm square opening round the stem, sliding on two
  3 mm guide pins with a 0.8 N/mm compression spring and 2.5 mm of travel, and a printed 2 mm mark on its post. Verify
  the spring with a kitchen scale: 2.0 N at 2.5 mm.
- Pencil gauge: steel and brass, 124 mm long, 12 mm square barrel, 60 g, a 20 mm square chuck head with a 12 mm square,
  24 mm deep socket and a rubber gasket on its face, a 5 mm square stick with 70 mm of travel marked 1 mm per psi, held
  where it stops by 0.5 N of sliding friction. Verify friction by pulling the stick with a spring scale (0.4 to 0.6 N).
  A commercial pencil gauge has a round barrel and a nonlinear scale; this one has the same size and layout with a
  square barrel and a linear scale so that both can be read with a rule.
- Gauge holder: grey ABS, 50 x 50 mm base, a 25 mm square pocket 18 mm deep, 8 mm floor, at (0.300, -0.300) m.
- Fidelity: the air seal is a rule in the scorer (the stick is drawn out only while the press is square and held), since
  tyre air flow is not simulated. The collar's spring and the 3 deg squareness limit stand in for the valve core and
  the gasket; in the real build the gasket's seal is felt as a hiss and seen as the stick not rising, which the
  measurement above checks.

## Reset between trials

Push the stick back to 0 mm, stand the gauge upright in its holder, release the collar to its top, and check the wheel
and holder against their taped outlines and the tyre at 35 psi.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public Isaac
Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no third-party
assets or brands.
