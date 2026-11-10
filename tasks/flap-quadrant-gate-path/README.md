# Move a flap lever through its gated quadrant to full flap

The robot carries a lever along a gated slot path. It lifts the lever clear of each detent, jogs it sideways round the
gate, and drops it into the far detent. The lever is the white flap selector of a free-standing bench flap-quadrant
trainer. It has to go from UP, through the 5 and 15 detents and round the go-around gate, to FULL. At the end the
lever's gate pin sits in the FULL notch, the trainer has not moved, and both grippers have let go.

Aircraft mechanics and service technicians (O*NET 49-3011.00) do this in the flap-system operational check from the
flight deck. The task uses a light desk-top flap-quadrant procedure trainer. A free-standing desk-top trainer has to
be steadied while its spring-loaded handle is lifted, and this one weighs less than the knob's lift spring.

The ability is the gated path itself: lift out of a detent, carry, jog sideways round a wall that no lift clears, carry
on, and drop into a notch. It is not a single detent change (a pin pulled out of one hole and dropped into the next).
The lever has to follow a slot with two lanes, and three different moves (lift, carry, jog) have to come in the right
order at the right places. Holding the trainer down is the second arm's role.

Instruction given to the policy (23 words; it states the goal only, never the lift, the gate or the hold-down; the
red end stop just past the FULL notch is the visible landmark for FULL):

`Put the white flap lever of the grey quadrant trainer in its FULL detent, next to the red end stop, then let go.`

Files in this folder:

- `task.json`
- `task.py`: scene, whole-run record, scoring and the scripted demo
- `linkkit.py`: articulated rigid-link bodies for yam_arena
- `linkdemo.py`: the closed-loop scripted-demo runner
- `tests/test_scoring.py`: scorer and geometry edge cases
- `README.md`

## Physics, not the instruction, defeats the natural plans

All of this is contact and joint physics. There are no scripted locks and no state switches.

- Push toward FULL. The gate pin under the knob sits 6 mm deep in the UP notch, between the end stop and the UP-5
  tooth. The lever stands at 0.6 deg with the pin against the tooth face, so a push moves nothing (`push_only`).
- Lift and push straight. The teeth stand 6 mm above the seated pin, and the pin's ends tilt with the lever, so a lift
  of about 8 mm clears them. But the lever's shaft runs in the slot's
  inboard lane, and that lane ends at the 15 detent in the gate wall. The shaft meets the wall at 21.0 deg. The knob
  lifts and the shaft does not, so no lift clears the wall (`lift_straight`). Only a sideways jog of 16 mm into the
  outboard lane passes it. The lane holds the shaft between the gate wall and its outer wall from 15 to FULL.
- Lift without holding the trainer. The trainer stands free on rubber feet and weighs 0.50 kg (4.9 N). The knob's
  lift spring is preloaded to 6.0 N, so the knob moves only when the pull passes 6.4 N (preload plus knob weight). The
  trainer leaves the table at a 4.5 N pull. Lifting the knob alone lifts the whole trainer, which is a move of more
  than 10 mm (`no_hold`). One arm has to hold the trainer down while the other lifts, carries, jogs and drops the
  lever. Neither arm can do both.
- Let go with the pin between notches, or keep the lever lifted. The pin is not seated, and every rung needs it seated
  (`leave_lifted`).

## Requirements

Built on the program's Brev launchable: Isaac Lab-Arena, Isaac Sim and Isaac Lab, with YAM arms. The demo and the
cheats below ran on one NVIDIA L4. Versions used:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 1.0.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: c3602b28d5f3 (`sha256sum task.py | cut -c1-12`). It equals
  `eval.environment_revision` in the Dream-1 log.

Before every shell, run `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook. The hook
teaches `yam_arena.scene` to spawn the trainer as one free PhysX articulation, built with USD joints with limits and
drives and spawned through Isaac Lab's `ArticulationCfg`:

- a base with the gate plate
- a pitch revolute to a gimbal yoke
- a sprung sideways revolute to the lever
- a sprung lift prismatic to the knob

The hook also calls the task's per-step record after every control step. There is no fork of yam_arena and there are
no third-party assets: every body is built from box and cylinder colliders in `task.py`. No helper code from other
teams' tasks is used. `linkkit.py` and `linkdemo.py` are our own kit, also used unchanged in our other linkkit tasks.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

22 tests. Most feed synthetic records to the scorer:

- A demo-shaped trace scores 1.0.
- Trainer move gate: 10.0 mm is no gate and 10.1 mm is, at any time. A lift at the end is caught, and setting the
  trainer back does not clear it.
- Seat and angle limits:
  - lift 0.99 mm counts as seated and 1.0 mm does not;
  - FULL is 40 deg within 1.0 deg (41.1 deg falls to 0.25);
  - "past the gate" starts at 25.0 deg (24.9 deg earns nothing).
- The cheat shapes score 0: a push stopped at the tooth, a straight lift stopped by the gate wall and seated at 15,
  the lever held lifted at FULL, and a pin left resting on the 30-FULL tooth.
- Still and release rungs:
  - a knob at 4 mm/s or 3.1 deg/s in the last second scores 0.5;
  - one pad step at 29 mm restarts the release timer, and 30 mm does not;
  - a gripper at 6 mm/s scores 0.75;
  - 0.95 s of release is not enough;
  - a hand held still on the seated lever scores 0.75.
- The record is clean after a reset, and pose rates come from pose change between control steps.

The rest place the trainer's own links with its kinematics and measure the gaps to the gate plate's colliders:

- The pin and shaft fit every notch they rest in, with the shaft in the lane that reaches that notch.
- A seated pin pushed 3 deg runs into the UP-5 tooth.
- Lifted 9.5 mm, the pin clears every tooth along both lanes and through the jog. Lifted 6 mm, its lowered end still
  meets a tooth.
- The shaft clears the inboard lane at 21 deg and meets the gate wall at 22.5 deg.
- The jogged shaft passes at 22.5 deg, but the outboard lane is closed before the gate.
- The outboard lane holds the shaft between the gate wall and its outer wall.
- The pitch, sideways angle and lift seen from link poses (what the demo uses) match the joints.
- `measure()` on real link states gives the right angles, the right clearances and no move. A base lifted 11 mm, or
  tilted 3 deg about its far edge, is a move.
- The trainer is lighter than the spring's preload.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the white flap lever of the grey quadrant trainer in its FULL detent, next to the red end stop, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/flap-quadrant-gate-path-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`. Otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). For repeat trials, add `--epochs K`. With `object_jitter=0` the seed changes nothing in the
scene.

```bash
inspect-robots inspect adhoc_8dd35b75.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_8dd35b75.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant lift_straight  # a cheat variant (or DEMO_VARIANT=lift_straight)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz. It writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record).

The script reads only what a camera could measure: the poses of the trainer's base, lever shaft and knob, and the
robot's own grippers. It works out the lever's pitch, sideways angle and knob lift from those poses. It never reads
joint values, solver velocities or the record.

The YAM arm is soft in simulation: about 2 to 4 mm of TCP sag per newton at the knob. So the right hand works the
lever with a servo. It commands the knob pose for a target (pitch, sideways angle, lift) from the trainer's kinematics
and its grasp. It then corrects a per-joint bias from the angles it sees, every 0.9 s, until it is within 0.4 deg,
0.5 deg and 0.7 mm.

1. Both arms at once.
   - Left: comes down on the handle tab with its pads closing across the tab (along world y) and grips it. It then
     pushes the gripped tab 30 mm down, which is a few newtons of hold-down through the arm's compliance.
   - Right: comes down on the knob, open to 60 mm, its approach leaning 15 deg past the lever toward FULL (for reach).
     Its pads close along the pivot axis on the knob's 22 mm faces, with a full squeeze. It records the grasp from
     the knob pose it sees.
2. Once the left holds (6.4 s), the right lifts the knob. Three servo steps reach a 9.2 mm lift, with the arm
   commanded about 18 mm higher than the knob rose.
3. Right carries the lever to the 15 detent (20.3 deg).
4. Right jogs it sideways into the outboard lane (-11.5 deg).
5. Right carries it to FULL (39.9 deg) at a lift of 9.6 mm. The lift command is trimmed 2.5 mm and 3 mm before the two
   carries, because the arm stiffens along the lever as it tips.
6. Right lowers the knob, commanding 3 mm below the seat. The pin drops into the FULL notch and the shaft springs back
   onto the gate wall's outboard face.
7. Right opens to 60 mm, backs off 70 mm along its approach and rises 50 mm.
8. Left eases off its push, opens, rises 40 mm and moves 60 mm away from the housing. Both wait.

The demo scores 1.0 (`success`; the episode ended at step 658 of 1,200):

- End state: the pin seated in FULL at 39.58 deg (lift 0.0 mm). The shaft rests on the gate wall at -10.65 deg.
- The trainer's largest move was 5.0 mm, during the push-down and the carries (limit 10). It ended 3.8 mm from where it
  was placed.
- Both grippers ended 63 and 79 mm clear.
- Worst IK error: left 0.08 mm and 0.01 deg, right 0.09 mm and 0.02 deg.
- A second reset puts the lever back in the UP notch (joint errors 0.6 deg pitch, 0 sideways and 0 lift, within the
  notch's play) and the base within 0.13 mm of its place. The record was cleared.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (c3602b28d5f3). Every one scores exactly
0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | The lever stays seated at UP (0.6 deg). | 0.0 |
| `push_only` | Left holds as in the demo. Right grips the knob and drives it toward FULL with no lift. | The pin meets the UP-5 tooth, so the lever never passes 0.6 deg. The arm keeps shoving and drags the held trainer 41 mm (gate at 7.9 s). Let go, the lever is still seated at UP. | 0.0 |
| `lift_straight` | Left holds. Right lifts as in the demo and drives the lever straight toward FULL in the inboard lane, with no jog. | The pin clears the teeth, but the shaft stops at 21.0 deg on the gate wall. The continued push drags the held trainer 24 mm (gate at 11.2 s). Lowered, the pin seats in the 15 notch (20.6 deg). | 0.0 |
| `no_hold` | Left stays at ready. Right runs the demo's full lever sequence (lift, carry, jog, carry, drop, let go). | The trainer comes up with the knob: the knob never slides more than 3.2 mm up the lever, and the trainer is 23 mm off its place by 6.3 s (gate at 6.05 s). The arm then carries the trainer about by its knob and drops it 0.43 m away, the lever still at UP. | 0.0 |
| `leave_lifted` | Left holds. Right lifts, carries, jogs and carries to FULL, then keeps the knob lifted and never lets go. | It holds the lever at FULL (39.9 deg) with the pin 10.1 mm up until the run ends at 1,200 steps. The pin is never seated and the hand never lets go. | 0.0 |

## Build log

Before the simulator, the offline geometry tests caught three faults, all fixed:

- the gate pin's ends dip as the lever tilts sideways, so the pin was raised to 3 mm above the gate plate;
- the shaft's sideways position grows toward the top of the slot, so the lanes were widened to 14 mm with 2 mm
  between them;
- the chord boxes' end faces were not radial, so they now end in 1 deg segments.

Then 14 runs on the shared L4, about 3 to 6 min each:

| Run | Change | Result |
|---|---|---|
| 1 | First build | 0.75. The lever reached FULL and seated (39.58 deg), but the left hand opened and rose straight up only 25 mm from the housing side plate. Its 90 mm rise was partly out of reach (11 mm IK error). |
| 2 | The left eases off its push before opening, rises 40 mm and moves 60 mm away. Hover lowered. Lift command trimmed before each carry. | 1.0 at step 658 |
| 3 | Recorded demo, same file (c5c85d51c8c5) | 1.0 at step 658, the same end state to 0.01 deg |
| 4 to 8 | Cheats on the same file | `nothing` 0.0, `push_only` 0.0, `lift_straight` 0.0, `no_hold` 0.0, `leave_lifted` 0.0 |
| 9 | Review fixes (c3602b28d5f3): the docstring's lane offset corrected to 16 mm, and the red FULL mark, which sat 0.5 mm under the gate plate's surface and never showed, replaced by a red end stop just past the FULL notch (colour only, no physics change). Recorded demo. | 1.0 at step 658, the same end state to 0.01 deg; the red stop shows in the top and wrist cameras |
| 10 to 14 | Cheats on the same file | `nothing` 0.0, `push_only` 0.0, `lift_straight` 0.0, `no_hold` 0.0, `leave_lifted` 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

The on-step record is `Record` in `task.py`, a `linkkit.History`. Every control step from reset it measures:

- the lever's pitch (deg from UP toward FULL), its sideways angle and the knob's lift (mm), from the joints;
- the trainer's displacement: the largest move, from where it was placed, of 6 points on the base (the plate's four
  corners, a point on the gate plate and the handle's top);
- the knob's speed and spin, from its pose change between control steps (what a camera sees);
- each gripper's pad clearance to every trainer link, and its speed.

The record keeps flags, peaks and held times for the whole episode. It is cleared only by the simulator reset, so
standing the trainer back on its outline never clears a move.

- Gate, at any time in the run, gives 0: the trainer moved more than 10 mm. A lift without a hold-down, or a shove
  that drags the trainer, both trip it.
- Seated means a lift under 1 mm: the pin is down in a notch.
- 0.25: at the end, the pin is seated past the go-around gate (pitch 25 deg or more, which is the 30 or FULL notch).
- 0.5: at the end, the pin is seated in the FULL notch (pitch within 1 deg of 40).
- 0.75: 0.5, and the knob was still (under 3 mm/s and 3 deg/s) for the last 1 s.
- 1.0: 0.75, plus the release gate: every pad point of both grippers is at least 30 mm from the trainer, and both
  grippers moved under 5 mm/s, for the last 1 s.

`score()` returns the rung. `success()` is true on the 1.0 rung, and yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s. `distance()` is 1 - score.

Real trial:

- a top photo for the detent the lever stands in, and for the pin down in its notch;
- a taped outline and a ruler for the 10 mm move;
- the video for stillness and the release.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 658.
- Runtime on one L4 (shared with another builder): a demo run took about 4 min wall clock at 4.1 control steps/s. That
  includes about 1.5 min of Isaac Sim start-up. A full 1,200-step run takes about 6 min. A Dream-1 trial is slower,
  since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout: the trainer is never jittered. The
  scripted demo is deterministic (runs 2 and 3 ended in the same state). Dream-1 is a hosted model without a seed.

## Scored runs

| Run | inspect-robots-dream | yam-arena commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 0.1.0 | 0059d02a1a91 | c3602b28d5f3 | 1200 | 6 min | adhoc_8dd35b75.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | - | 0059d02a1a91 | c3602b28d5f3 | 658 (success held 1 s) | about 4 min | demo_out/result.json | 1.0 | left held the trainer down, right lifted, carried, jogged round the gate, carried to FULL and dropped the pin in; both let go |

- Demo: `python linkdemo.py task.py demo_out` writes `demo_out/result.json` with `task_score = 1.0`. The video is
  `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`). The screenshot is `demo_out/setup.png` (frame 0
  of the top camera).
- Dream-1 run: made at filing with the command above, on this exact `task.py`. It was not refused and ran to its stop condition; the measured result is in the Scored runs table.

## Materials

Every position, colour and friction value is also in `task.json` (setup) and at the top of `task.py`.

- Flap-quadrant trainer, free-standing, 0.50 kg in all. Its pivot is at world (0.33, -0.12), 40 mm above the table.
  The lever travels from UP (vertical) toward the robot (world -x) to FULL, and the pivot axis lies along world y.
- Base: grey anodised aluminium, 0.42 kg with its housing and gate plate.
  - A 170 x 230 x 4 mm base plate on rubber feet (friction 1.0 / 0.9).
  - Pivot cheeks and housing side plates.
  - A 40 x 10 x 50 mm hold-down handle tab at its left end, 100 mm from the lever along the pivot axis.
- Gate plate: dark steel, part of the base. It is an arc band 20 mm deep (70 to 90 mm from the pivot) and 80 mm wide,
  from 12 deg behind UP to 50 deg.
  - Slot: the inboard lane runs from UP to 15 and ends in the gate wall. The outboard lane, 16 mm over, runs from 15 to
    FULL. The lanes overlap only at the 15 detent.
  - Teeth: 6 mm detent teeth on top leave the UP, 5, 15, 30 and FULL notches, 10 deg apart (FULL at 40 deg). End stops stand
    at both ends; the one just past FULL is painted red.
- Lever: a white-painted aluminium shaft, 10 mm in diameter, 0.03 kg, on a 10 g gimbal yoke at the pivot. A sideways
  spring (0.6 N m/rad) holds it in the inboard lane.
- Knob: a white ABS flap-handle block, 30 x 22 x 30 mm, 140 mm from the pivot, 0.04 kg. It rides on a 12 x 12 mm
  sleeve that slides 12 mm up the lever against a spring of 60 N/m, preloaded to 6.0 N. Under it, a steel gate pin
  8 mm in diameter and 30 mm long lies across the slot.
- Layout and physics chosen for the simulated arm:
  - The trainer is light and the spring soft (a real trainer is heavier, with a stiffer spring) because the YAM arm
    sags 2 to 4 mm per newton.
  - The trainer is turned so the right hand meets the knob from above with its approach leaning toward the robot. A
    straight-down grip at that height is past the wrist's limit.

## Reset between trials

Nothing is consumed.

1. Lift the knob, move the lever back through the jog into the inboard lane, and drop it into the UP notch (lever
   vertical, pin seated).
2. Stand the trainer back on its taped outline at the placed position, with the handle tab on the robot's left.
3. Check that the knob drops under its spring, and that the lever springs to the inboard lane on its own.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code, with an SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text. They use only
public Isaac Lab and USD Physics techniques and copy no code from other tasks in this repository. The scene uses no
third-party assets or brands.
