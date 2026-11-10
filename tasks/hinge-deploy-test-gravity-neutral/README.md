# Run a spring-hinge deployment test with the hinge turned gravity-neutral

The robot runs a ground deployment test of a spring-hinged CubeSat solar panel: it turns the tilt fixture so the
hinge axis is vertical, then pulls the red hold-down pin and lets the panel's own flight spring swing it to its latch,
without touching the panel. Aerospace engineering and operations technicians (O*NET 17-3021.00) and avionics
technicians (49-2091.00) do this for every deployable panel and antenna before flight. Flight springs are sized for
zero g, so on the ground the hinge is turned axis-vertical and the panel swings in a horizontal plane. This is an
honest 1 g bench practice: no microgravity is claimed.

The ability is to reorient a mechanism so gravity no longer acts along its motion before setting it off. Nothing is
placed and nothing is lowered: the fixture is turned so that the release works, and the release is only valid in that
orientation. Unlike turning a rollover fixture to reach a far face (TK-XLTXT8) or freeing a fire door for its closer
(TK-VNC9LD), the turn here reaches and unblocks nothing: it is needed only because gravity would beat the flight
spring, and the same pin pull is a failed test in any other orientation.

Physics, not the instruction, defeats the natural plan. As delivered the hinge axis is horizontal and the 150 g panel
lies flat on top of the fixture, so it must swing upward. Its weight gives 0.088 N m about the hinge when stowed; the
flight spring gives 0.050 N m. Pull the pin as delivered and the panel stays on its stowed stop. Helping it up by hand
is caught as gripper contact after the release, which voids the test. A partial tilt also fails: the spring beats
gravity along the whole swing only once the axis is about 59 deg or more out of horizontal (the spring-to-gravity
ratio is lowest, 0.52, about 26 deg into the swing). Three turns of the crank put the fixture at 90 deg, and the same
spring then carries the panel through 90 deg to its latch in under half a second.

The two arms have separate jobs at opposite ends of the fixture. The left arm turns the crank on the gearbox at the
left end; the self-locking worm then holds the fixture at 90 deg with no hand on it. The right arm pulls the red pin at
the right end and withdraws clear. Neither arm can do the other's job with a top-down grip: in our IK check the left
arm misses the pull knob by 54 mm and the right arm misses the crank's circle by 28 to 104 mm. The pin pull is a
valid test only once the crank arm has finished the turn, so the order between the arms is enforced by the scorer as
well as by the physics.

Instruction given to the policy (38 words; it states the goal and the trigger, never the tilt):

`Run a bench deployment test of the gold solar-panel dummy: pull the red hold-down pin so its light hinge spring alone opens it to its latch, then let go. The spring is a flight spring, sized for orbit.`

Files in this folder: `task.json`, `task.py` (scene, worm drive, hold-down and latch, whole-run record, scoring and
the scripted demo), `linkkit.py` (articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop
scripted-demo runner), `tests/test_scoring.py` (scorer, worm, physics and geometry edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab). Versions used for the demo and cheat
runs below, on one NVIDIA L40S:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- inspect-robots-dream 0.1.0 (for the Dream-1 trial)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: 1224a2a8fee2 (`sha256sum task.py | cut -c1-12`; equals
  `eval.environment_revision` in a run log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the fixture as one PhysX articulation (the fixed stand, the tilt table on a revolute
trunnion, the panel on a sprung revolute hinge with a holding brake, and the pin on a prismatic joint) and the crank as
a second one (a revolute crank and a free-turning handle sleeve, joints without limits), spawned through Isaac Lab's
`ArticulationCfg`, and calls the task's per-step record after every control step. No fork of yam_arena and no
third-party assets: every body is built from box and cylinder colliders in `task.py`.

## Offline check, no Isaac Sim

```bash
python -m pytest -q tests
```

39 tests. They need `yam_arena` importable (any shell on the Brev launchable after `source yam.env`; `task.py` and
`linkkit.py` import `yam_arena.task_file`), but they never start Isaac Sim. The ladder: a demo-shaped record scores 1.0, and doing nothing scores 0; the axis turned to
10.0 deg from vertical without a release is 0.25 and 10.1 deg is 0; a release with the axis 10.0 deg off vertical is
valid and 10.1, 45 or 90 deg is a gate; a release as delivered followed by turning the fixture and a full latch stays
at 0; a pad 4.9 mm from the panel after the release is a gate and 5.0 mm is not, a touch before the release does not
count and a touch long after the latch still does; a valid release without the latch is 0.5; latched but the pawl
under 88 deg, unlatched, the fixture turned back to 10.1 deg or a panel at 3 mm/s or 3 deg/s at the end is 0.75; the
let-go gate needs 1.0 s (0.95 s is not enough) and restarts on a pad at 29 mm (30 mm is clear) or a hand at 6 mm/s;
only the first release counts; the record is clean after a reset. The worm: three crank turns tilt 90 deg and one and
a half 45 deg, with readings unwrapped or wrapped to +-180 deg; cranking backwards from 0 or past 95 deg moves nothing,
and turning back from the 95 deg stop moves at once. The physics: the board's centre of mass is 60 mm from the hinge
line; the spring gives 0.050 N m stowed and 0.020 N m at 90 deg against 0.088 N m of gravity as delivered; with the
fixture at 45 or 55 deg the spring loses part of the swing, at 80 and 90 deg it wins all of it, and the threshold is
58 to 61 deg. The geometry, from the fixture's own link poses: the axis is 90, 45, 10, 0 and 5 deg from vertical at
tilts of 0, 45, 80, 90 and 95 deg; at 90 deg the panel faces away from the robot and its centre swings level; as
delivered the deploy lifts the board's centre 58 mm; the pin is 6 mm into the boss and 2 mm clear of it at 8 mm out;
the panel's corners and the pull knob stay clear of the stand at every tilt and hinge angle; the knob and handle are
where the demo expects them.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Run a bench deployment test of the gold solar-panel dummy: pull the red hold-down pin so its light hinge spring alone opens it to its latch, then let go. The spring is a flight spring, sized for orbit." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/hinge-deploy-test-gravity-neutral-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

```bash
inspect-robots inspect adhoc_48da101e.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_48da101e.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                            # scripted demo
python linkdemo.py task.py cheat_out --variant as_delivered    # a cheat variant (or DEMO_VARIANT=as_delivered)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the fixture's tilt from the tilt table's pose against the stand, the panel's angle from
its pose, the handle's and the knob's positions, and the robot's own grippers. It never reads the brake, the latch,
joint values or the record.

1. Left: top-down on the crank's handle sleeve (pads closing along x, 6 mm squeeze), turns the crank three turns
   anticlockwise about its seen axis with the wrist's orientation held (the sleeve turns in the pads) over 14.4 s,
   checks the seen tilt (89.92 deg) and stops, as it is within 0.7 deg of 90; lets go and parks up and back.
2. Right: top-down on the red pull knob (pads closing along x across it, 6 mm squeeze), pulls it 16 mm straight out
   along the pin's seen axis, lets go, lifts 80 mm and parks.

The demo scores 1.0 (`success`, episode ended at step 653 of 1,200): the crank turned 1,078.8 deg and put the
fixture at 89.96 deg; the pin came 16.65 mm out, and the hold-down released at 26.6 s with the hinge axis 0.04 deg
from vertical; the panel swung 90 deg to its latch in 0.45 s on its own spring, with no gripper pad closer than 84 mm
to it after the release; both grippers ended clear and still. Worst IK error 0.17 mm and 1.42 deg. A second reset
(`--check-reset`) returned every joint of both articulations to its initial value, at rest, with the record cleared.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (1224a2a8fee2); every one scores
exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; the fixture stays at 0 deg. | 0.0 |
| `as_delivered` | The right arm pulls the pin 16 mm with the fixture as delivered (hinge axis horizontal), then parks. | The hold-down released at 4.9 s with the axis 89.97 deg from vertical; the panel never left its stowed stop (peak 0.000 deg). Gate: invalid release. | 0.0 |
| `help_up` | As delivered, then the right hand pinches the panel's boss (pads along the hinge) and swings it up toward the latch. | Released at 4.9 s as delivered (gate). The hand touched the panel at 13.65 s (pad 1.0 mm from it, also recorded) and lifted it 12.9 deg before the swing left the arm's reach (IK error up to 147 mm); the panel fell back. | 0.0 |
| `pull_then_turn` | The right arm pulls the pin first; the left arm then turns the crank three turns. | Released at 4.9 s as delivered (gate). As the fixture passed about 61 deg the spring began to win: the panel latched at 23.1 s with the fixture near 72 deg, and the fixture ended at 90.01 deg. The run still scores 0. | 0.0 |
| `partial_tilt` | The left arm turns the crank one and a half turns (45 deg), then the right arm pulls the pin. | Released at 19.4 s with the fixture at 44.8 deg (axis 45.2 deg from vertical, gate); the panel never left its stowed stop (peak 0.000 deg). | 0.0 |

## Build log (3 demo runs, the final batch, then a re-check)

| Run | Change | Result |
|---|---|---|
| 1 (shared L4) | first build: pull knob 32 mm closer in, 23 mm past the tilt table's end | 0.25: three crank turns put the fixture at 89.94 deg, but closing on the knob the gripper pushed the upright tilt table to 91.5 deg (most likely a finger on the table's end, 23 mm from the knob) and the grip slid off; the pin moved 1.3 mm |
| 2 (shared L4) | pull rod lengthened so the knob sits 47 mm past the tilt table's end; reachable park poses | lost: the shared VM ran out of memory and stopped answering |
| 3 (idle L40S) | the same file on another box with the same versions | 1.0 at step 653 |
| final | the submitted file, recorded with `--check-reset`, then the five cheats | 1.0 at step 653, the same end state; cheats below |
| re-check | after README and task.json wording edits, the same `task.py`, `linkkit.py` and `linkdemo.py` re-run on the L40S: pytest, the recorded demo with `--check-reset`, the five cheats | 39 tests pass; demo 1.0 at step 653 with a clean reset check; every cheat 0.0, each run's record identical to the final batch |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, `task.py` runs the fixture's mechanisms and the on-step record. The worm (`Worm`)
accumulates the crank's turning (unwrapped between control steps) and sets the trunnion drive's target to it / 12,
between 0 and 95 deg; past a stop the crank turns freely, and the table never drives the crank. When the pin first
passes 8 mm out, the hold-down releases the panel (the hinge's holding brake gives way to the flight spring). When the
released panel reaches 89 deg the latch pawl drops in and holds it between 88.5 deg and the 90 deg stop.

The record (`Record` in `task.py`, a `linkkit.History` plus the release record) logs the hinge axis's angle from
vertical (from the tilt table's pose), the fixture's tilt, the pin's pull, the panel's angle and the latch, each
gripper's pad clearance to the panel and to the cradle, panel, pin, crank and handle, its speed, and the panel's speed
and spin from its pose change between control steps. It is cleared only by the simulator reset, so turning the fixture
after a wrong release, or taking a hand away after touching the panel, never clears the run.

- Gate, giving 0: the release was made with the hinge axis more than 10 deg from vertical. As delivered (90 deg off)
  the spring fights gravity; a release then is not a valid deployment test, whatever happens afterwards.
- Gate, giving 0: any pad point of either gripper came within 5 mm of the panel at any time after the release (an
  assisted deployment).
- 0.25: the hinge axis was within 10 deg of vertical at some time.
- 0.5: a valid release.
- 0.75: after the release the panel reached its latch.
- 1.0: also at the end the panel is latched at 88 deg or more, the axis still within 10 deg of vertical, the panel
  still (under 3 mm/s and 3 deg/s) for 1 s, and the let-go gate: every pad point of both grippers at least 30 mm from
  the cradle, panel, pin, crank and handle and both grippers moving under 5 mm/s for the last 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: an inclinometer on the tilt table (the hinge
axis within 10 deg of vertical), a scale on the pin, and the video for the moment of release, any touch and the latch.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 653.
- Runtime on one L40S: a demo run took about 2.2 min wall clock (recorded, alone on the box, including about 1.5 min
  of Isaac Sim start-up); a cheat run 2.5 to 5 min (the last two shared the box with another simulation). On a shared L4 the first run took 8 min. A Dream-1
  trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (both articulations are fixed to
  the table). The scripted demo is deterministic (run 3 and the final run ended at the same step with the same record);
  Dream-1 is a hosted model without a seed.

## Scored runs

| Run | inspect-robots-dream | yam-arena commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 0.1.0 | 0059d02a1a91 | 1224a2a8fee2 | 1200 | 3 min | adhoc_48da101e.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | - | 0059d02a1a91 | 1224a2a8fee2 | 653 (success held 1 s) | about 2.2 min | demo_out/result.json | 1.0 | fixture turned to 89.96 deg, released 0.04 deg off vertical, latched 0.45 s later untouched, both grippers let go |

The demo video is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot
`demo_out/setup.png` (frame 0 of the top camera). Make the Dream-1 run on this exact `task.py` after a 40-step probe
of the instruction (`--max-steps 40`, no frames) to rule out a refusal.

## Materials

- Tilt fixture stand, fixed: a grey cast-iron foot 140 x 90 x 10 mm and a dark grey painted cast-iron worm gearbox
  column 70 x 70 x 130 mm, both centred at (0.330, 0.150), with a 36 mm trunnion bearing boss on the column's right
  face; 3.5 kg. A single-sided (cantilever) tilt positioner of the kind used for bench tests.
- Input-shaft bearing post: dark grey steel, 20 mm across and 6 mm tall, 50 g, on the column top under the crank hub.
- Crank: black steel, 60 g, on a vertical input shaft on the column top at (0.330, 0.150): a 24 mm hub and a 40 mm
  arm pointing toward the robot at the start, with a black acetal handle sleeve 16 x 30 mm (10 g) that turns freely
  on it, centred at (0.290, 0.150, 0.9175). Crank drag 0.03 N m plus 0.01 N m s/rad. A self-locking worm, 12:1 (one
  crank turn is 30 deg of tilt, anticlockwise seen from above turns the table's top face away from the robot), with
  stops at 0 and 95 deg.
- Tilt table: a natural aluminium plate 120 x 170 x 8 mm, 0.45 kg, on a 20 mm trunnion shaft along y out of the
  column, axis 110 mm above the table; an aluminium pin bracket 16 x 15 x 17 mm on its right end; two aluminium hinge
  knuckles 8 mm across and 30 mm long on the panel's hinge line (included in the 0.45 kg).
- Gold panel: a 1U-size deployable solar-panel dummy, an aluminium honeycomb board with a gold-coloured face,
  100 x 120 x 4 mm, 150 g (centre of mass at the board's centre, 60 mm from the hinge line), stowed flat on the tilt
  table 0.5 mm above it. Spring hinge along x at its left edge: a flight torsion spring of 0.0191 N m/rad preloaded to
  0.050 N m stowed (rest at 150 deg) and 0.020 N m at 90 deg, a viscous damper of 0.008 N m s/rad, 0.003 N m of
  friction, a 90 deg hard stop and a latch pawl that drops in at 89 deg. A gold hinge knuckle 8 mm across and 38 mm
  long between the tilt table's two, and a 14 x 8 x 16 mm hold-down boss with a 4 mm slot on its right end (both
  included in the 150 g).
- Red hold-down pin: a 3 mm steel pull rod 75 mm long, 6 mm engaged in the boss's slot through the bracket, with a red
  nylon pull knob 12 x 16 mm 47 mm past the tilt table's end; 15 g; slides 0 to 20 mm on 0.5 N of friction.
- Layout and physics chosen for the simulator and the reach: the worm ratio is coarse (12:1, three crank turns for
  90 deg) to fit the 60 s run; the knob sits 47 mm past the tilt table's end so the gripper's fingers clear the table
  when it stands upright; the crank and the knob are at opposite ends, each reachable by one arm only with a top-down
  grip.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Swing the panel back flat onto the tilt table (lift the latch pawl first) and push the red pin
home through the bracket into the boss's slot, 6 mm engaged, knob 47 mm past the table's end. Turn the crank clockwise
until the tilt table stops at 0 deg (top face up, hinge axis horizontal), then on until the arm points toward the
robot. The stand is fixed on taped outlines and needs no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
