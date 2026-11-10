# Deploy a scissor boom at 1 g, carrying its tip by its offload sling

The robot runs a 1 g deployment check of a small spacecraft scissor boom. The left arm turns the hand crank that drives
the boom's self-locking root screw. The right arm takes the weight of the boom's tip through its offload sling, the
boom's only handling point, and keeps the sling hanging straight while the tip moves out, until the root latch engages
at full length. Then both arms let go. Aerospace engineering and operations technologists and technicians (O*NET
17-3021.00) and aircraft structure, surfaces, rigging and systems assemblers (51-2011.00) run these checks on booms,
antennas and solar arrays before flight. Flight hinges are sized for zero g, so on the ground an offload carriage on an
overhead track takes the moving end's weight by a vertical cable while the drive extends it, and the hardware is handled
only at its ground-support points. This is an honest bench analogue at 1 g: no microgravity is claimed. The arm stands
in for the offload carriage, and a 40 mm bail stands in for its cable.

The ability is gravity offload of a driven deployable: one arm drives a self-locking mechanism, and the other takes the
weight of its moving end through an offload sling that has to hang vertical while the end moves. The scissor, not the
hand, sets where the end goes and how fast: per crank turn the tip runs out about 5 times faster at the start of the
stroke than at the end, 173 mm in all. Nearest tasks on the board (a snapshot taken at 09:00 UTC on Nov 10):

- Cranking a hose reel while the other hand lays the turns (TK-KJA94U). The guiding hand there is paced by the crank
  too: its scorer fails any turn laid outside 8 to 17 mm of the last. But that hand carries no load and its traverse
  per turn is constant, so a steady traverse in step with the crank works. Here the carried end's travel per crank
  turn changes about 5 times along the stroke, the hand carries the boom's weight, and a steady carry fails
  (`steady_pace` below).
- Running a spring-hinge deployment test with the hinge turned gravity-neutral (TK-KMZF6A, ours): the same 1 g
  problem, met the other way. There the fixture is turned until gravity no longer acts on the hinge, and any touch
  after the release voids the test. Here gravity stays on: one hand carries the moving end's weight for the whole
  stroke, on a path and at a pace the scissor sets.
- Holding a probe on passing pallets (TK-M63JJD) and hanging parts on a moving conveyor (TK-EECKTP): the hand tracks a
  target that moves at a constant speed set from outside, and carries no weight.
- Helping a person carry a pipe onto its stands (TK-8EEVCG, ours): a person moves the shared load and the robot
  complies. Here nobody else moves anything: the robot drives the boom with one arm and offloads it with the other.
- Moving an extraction hood's foot while its mouth stays over a mark (TK-DX3Z5N), holding a coffee grinder while
  cranking it (TK-HFPKM7) and backing up a panel while pressing a clip (TK-F3VZB7): in each the held or supported part
  stays where it is. Here it travels 173 mm.

What defeats the natural plans, from the simulator runs on the submitted task file:

- Crank alone (physics). The hinge spring (0.9 N m/rad, preloaded 10 deg up) holds the short stowed boom 1.2 deg below
  level. The cantilever moment roughly doubles as the boom extends. The hinge passes 3 deg at 15% out, the canister
  comes within 10 mm of the cells at 34% out and rests on them from about 55% out. At full length the hinge sags
  5.2 deg, so the latch cannot engage.
- Pull the tip out by its sling, without cranking (physics). The screw is self-locking. The tip moves at most 24 mm
  (14%), as far as the stiff drive gives, and the sling leans toward the hand past 15 deg.
- Hold the sling still while cranking (physics). The tip runs out from under the hand, the sling passes 15 deg at 7%
  out, and the scissor then drags the arm until the boom drops onto the cells at 83% out.
- Crank first, then lift the tip to level (physics). The canister has already lain on the cells. The latch then
  engages, but the whole-run record has the touch.
- Carry it level by its sling, moving out at a steady speed (the offload cable's limit). The boom does latch level
  (0.05 deg), but the tip outruns the hand early in the stroke: the sling tilts 15 deg, its grip 10.4 mm behind its
  pin, at 36% out and 39 deg by 70%. On a real rig that cable side-loads the boom's pins, which is why offload
  carriages are run to keep the cable vertical.
- Carry it by the canister or a scissor bar, or prop the canister from below (the handling rule the instruction
  states). The arm is soft enough that the scissor drags a firm grip along: carried at a steady speed by the canister
  or by a bar, the boom latches level with the cells 26 mm clear and the sling never tilts past 4 deg, so only the
  handling gate scores those runs 0. The prop bumped the boom 3.1 deg down as it slid in, so the hinge gate also
  catches it.

The roles are also forced by reach. The right arm cannot reach the crank's circle with a straight-down hand (IK error
56 to 138 mm). The left arm cannot hold a straight-down grip anywhere on the tip's path (IK error 19 to 31 deg, and 11
to 88 mm past y -0.08).

Instruction given to the policy (33 words; it states the goal and the handling point, never the carry, the pace or the
sling's angle):

`Deploy the white scissor boom to its latched full length, handling it only by its black sling and keeping its tip at least 1 cm above the blue solar cells, then let go.`

Files in this folder: `task.json`, `task.py` (scene, screw model, latch, whole-run record, scoring and the scripted
demo), `linkkit.py` (articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (screw, scorer, finger-proximity, geometry and statics edge cases), `demo_out/` (the recorded
demo: three camera videos, `setup.png` and `result.json`), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab). Versions used for the demo and cheat
runs below, on one NVIDIA L4:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: 66e7d739d22e (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision`
  in a run log)

The recorded demo and the cheat runs used this file with CRLF line endings (f3139a34fb15), the same code; git stores it
with LF endings. The scripted demo was then run again on the committed file, before the Dream-1 trial, on one NVIDIA
L40S (Scored runs, below).

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn three bodies:

- The boom as one PhysX articulation. The bus bracket is fixed. The root fitting sits on a sprung revolute hinge with
  a switchable brake (the latch). Two root sliders ride on stiff prismatic drives. The scissor is six bars on revolute
  pins, with three pin joints closing its loops outside the articulation. The canister, the sling and its grip sleeve
  hang on revolute pins.
- The crank as a second articulation: a fixed gearbox, the crank on a revolute joint and a freely spinning handle. It
  has no joint limits, so it turns without end.
- The cell panel as a fixed compound body.

It spawns them through Isaac Lab's `ArticulationCfg` and calls the task's per-step record after every control step.
There is no fork of yam_arena and there are no third-party assets: every body is built from box, cylinder and capsule
colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

36 tests, no simulator (they import yam_arena for the table height).

- The screw:
  - one crank turn moves it one lead (11 mm), including when the crank angle reads wrapped across +-180 deg, and back
  - full length is 2.98 turns, and it stops at both ends
  - it is self-locking: with the crank still, pulling or pushing the sliders never moves it
  - its slip clutch keeps it 1.5 mm ahead of a stalled slider and backs off on the first reverse step
  - once latched it is locked, and a reset clears it
- The record and the ladder:
  - a demo-shaped record scores 1.0
  - the release gate needs 1.0 s (0.95 s is not enough) and restarts when a hand comes back; a pad at 29 mm or a hand
    at 6 mm/s fails it, and 30 mm passes
  - one step with the canister 9.9 mm above the cells gates the whole run, even after a perfect deployment; 10.0 mm
    does not
  - the hinge at +-3.00 deg passes and +-3.01 deg gates, both when drooped and when pushed up
  - one step with a finger 4.99 mm from the boom outside its sling gates the whole run; 5.0 mm does not
  - with a hand on the sling and the tip moving, the sling at +-15.0 deg passes and +-15.01 deg gates, behind or ahead
    of its pin; a 25 deg tilt that is later made good still gates
  - a 35 deg knock on the bail while the tip is still (before cranking, or at release) does not gate, and nor does a
    free sling swinging with no hand on it
  - the earliest gate is the one reported
  - the rungs are 0 below 50% out, 0.25 from 50%, 0.5 at full travel and 0.75 latched
  - a reset clears every gate
- Finger proximity:
  - 270 points fill the last 48 mm of both fingers, no more than 6 mm apart
  - the demo's grasp on the grip sleeve reads as on the sling and more than 25 mm from every other boom part; pads
    across the canister read as touching it; a hand 0.3 m away is skipped
- Geometry and statics, from the boom's own link specs:
  - the scissor's stroke (tip 142 to 315 mm from the hinge, 32.8 mm per slider)
  - the tip only moves out, over 4 times faster per mm of screw early than late
  - every loop closes at the design pose
  - the sling's tilt sign: + when the grip is behind its pin, toward the root; 15 deg is a 10.4 mm offset
  - the hinge statics: stowed within 2 deg of level, more than 10 deg unsupported at full length
  - the level canister is 30 mm above the cells; carried at the 3 deg limit it still clears them by 12.6 mm at full
    length
  - cranked alone, the boom crosses the 10 mm gate before it is 60% out and would sink below the panel top at full
    length
  - the 2 deg latch window lies inside the 3 deg gate

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Deploy the white scissor boom to its latched full length, handling it only by its black sling and keeping its tip at least 1 cm above the blue solar cells, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/scissor-boom-1g-offload-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`. Otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). For repeat trials add `--epochs K`. With `object_jitter=0` the seed changes nothing in the scene.

```bash
inspect-robots inspect adhoc_f6e85e46.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_f6e85e46.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                         # scripted demo
python linkdemo.py task.py cheat_out --variant steady_pace  # a cheat variant (or DEMO_VARIANT=steady_pace)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz. It writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record).

The script reads only what a camera could measure, all from link poses: the two root sliders' places on the rail, the
root hinge's angle, the sling's tilt and the grip sleeve's position, and the crank's axis. It also reads the robot's
own grippers. It never reads the screw, the latch, the clearance record or any joint readout.

1. Both arms together:
   - Left: straight down on the crank handle, pads across it 10 mm above the crank arm. It opens to 40 mm and
     closes to 8 mm on the 14 mm handle.
   - Right: straight down on the sling's grip sleeve, pads closing along the boom (y) across it 4 mm below its axis.
     It opens to 40 mm and closes to 6 mm on the 12 mm sleeve.
2. Right: lifts the tip to level. It moves to the level point for the sliders' seen travel, then corrects its height
   for 1 s from the hinge angle seen in the link poses. The hinge is seen at 0.08 deg at 5.8 s.
3. Both arms together for 14 s:
   - Left: turns the crank 3.15 turns, a sixth of a turn past full length, by circling the held handle about the
     crank's axis. The wrist keeps its orientation while the handle spins on the crank arm.
   - Right: each control step it aims for the level tip position at the sliders' seen travel plus 0.1 s of their seen
     speed. It corrects that aim upward from the seen hinge angle and along the boom from the sling's seen tilt.
   - The tip runs 173 mm out, fastest early in the stroke. While cranking, the sling stays within 0.8 deg of vertical,
     the hinge within 0.16 deg of level, and the canister at least 29.6 mm above the cells.
   - The latch engages at 17.7 s with the hinge at +0.06 deg; the sliders are seen at 32.36 mm.
4. Both arms let go. The right opens to 40 mm and rises 50 mm, the left opens and rises 80 mm. Both then wait, clear
   and still. The latched boom settles 0.59 deg below level, with the canister 26.6 mm above the cells.

The demo scores 1.0. It reaches `success` and the episode ends at step 489 of 1,200.

- The sling's largest tilt, 1.8 deg, came as the hand closed on it (the gate is 15 deg).
- The hinge's largest angle was its stowed rest, 1.21 deg (the gate is 3 deg).
- The canister was never closer to the cells than its stowed rest, 26.6 mm (the gate is 10 mm).
- No finger came closer than 31.5 mm to any boom part but the sling (the gate is 5 mm).
- The peak slider strain against the screw was 0.19 mm, and the loop joints held to 0.01 mm.
- At the end both grippers were 59 and 49 mm clear.
- The worst IK error was 0.08 mm and 0.01 deg on the left, and 0.1 mm and 0.01 deg on the right.

A second reset (`--check-reset`) returned every joint to its initial value at rest, within 0.02 deg. The
sling was within 0.13 deg, the boom's fastest point moved at 0.013 mm/s, and the record was cleared.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (as f3139a34fb15, its CRLF copy). Every one scores exactly 0. The
last column is what the run would score with the handling gate set aside (`without_touch_gate` in `result.json`, a
report only, never the score):

| Variant | What it does | Result | task_score | Without the handling gate |
|---|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. The boom stays stowed, 0% out. | 0.0 | 0.0 |
| `crank_only` | The left arm cranks to full length; the right arm stays at the ready pose. | The hinge passes 3 deg at 15% out (5.7 s). The canister comes within 10 mm of the cells at 34% out (7.2 s) and lies on them from about 55% out. At full length the hinge hangs at 5.2 deg, so the latch never engages. | 0.0 | 0.0 |
| `pull_tip` | The right hand grips the sleeve, lifts the tip level and pulls it 173 mm out along the boom in 14 s. The crank is never turned. | The screw holds: the sliders give 1.5 mm and the tip at most 14% (24 mm). The sling leans forward past 15 deg at 8.2 s and up to 39 deg against its stop. The hand then twists the hinge 3 deg up (14.9 s), loses the sleeve, and the boom drops to 7.5 deg, 9.3 mm above the cells. | 0.0 | 0.0 |
| `hold_still` | The right hand carries the tip level but stays where the tip started while the left arm cranks. | The tip runs out from under the hand. The sling passes 15 deg behind at 7.35 s (7% out) and reaches its 40 deg stop. The scissor then drags the arm, and the boom drops onto the cells at 83% out (14.4 s). | 0.0 | 0.0 |
| `steady_pace` | The right hand carries the tip level by its sling and moves out at a steady speed over the crank's 14 s. | The sling leans ahead at the crank's slow start. The tip then outruns the hand: the sling is 15.2 deg behind at 9.85 s (36% out) and 39 deg by 70%. The boom still latches level at the end (0.05 deg), but the record keeps the tilt. | 0.0 | 0.0 |
| `lift_after` | The right hand lets go, the left arm cranks to full length, then the right hand regrasps the sleeve and lifts the tip to level. | The hinge passes 3 deg at 15% out (9.65 s) and the canister comes within 10 mm of the cells at 34% out (11.2 s) and lies on them. Lifted level, the boom latches at 1.91 deg (25.7 s), but the record keeps both. | 0.0 | 0.0 |
| `grip_canister` | The right hand grips the white canister itself, from beyond the tip, tilted 35 deg down with its pads across it, levels the boom, and moves out at a steady speed while the left arm cranks. | A finger comes within 4.9 mm of the canister at 2.7 s as the hand closes in. The scissor drags the firm grip along: the boom latches at -0.22 deg (16.8 s), the canister never under 26.3 mm from the cells and the sling never past 1.5 deg. | 0.0 | 0.75: latched level; only its scripted release, which left the hand within 30 mm of the boom, kept it off 1.0 |
| `grip_bar` | The right hand pinches the half bar next to the apex at its middle, straight down with its pads across it, levels the boom, and moves out at a steady speed while the left arm cranks. | A finger comes within 3.6 mm of the neighbouring bar at 3.05 s as the hand closes. The boom latches at +0.04 deg (16.8 s), the canister never under 26.5 mm from the cells and the sling never past 3.8 deg. | 0.0 | 0.75: latched level; only its scripted release, which left the hand within 30 mm of the boom, kept it off 1.0 |
| `prop_under` | The right hand's closed fingertips slide in under the canister from beyond the tip, lift it 5 mm and carry it out at a steady speed while the left arm cranks. | A finger comes within 4.7 mm of the canister at 2.95 s. Sliding in, the fingers bump the boom 3.1 deg down (3.25 s, 4.3 deg at most). Carried on the fingertips, it then latches level (0.01 deg, 17.15 s) with the canister never under 18.5 mm from the cells. | 0.0 | 0.0 (the hinge gate, from the bump) |

## Build log

| Run | Change | Result |
|---|---|---|
| 1 | First build. A vertical 12 mm lift lug pinched across the boom, and hinge preload 8.5 deg. | The demo scored 1.0 at step 489, and `crank_only`, `pull_tip`, `lift_after` and `nothing` scored 0. But a hand moving out at a steady speed also scored 1.0: the soft arm was dragged along by the lug. |
| 2 | A crossbar handle pinched along the boom, plus a gate on slider strain (1.2 mm for 0.15 s). | The demo scored 1.0. Steady pace still scored 1.0 (peak strain 0.43 mm against 0.17 mm in the demo), and holding still reached only 1.07 mm. The arm, not the grip, is the compliant part, so screw strain cannot separate a good carry from a bad one, and that gate was dropped. |
| 3 | The offload sling: a 40 mm bail pinned on the canister with a free grip sleeve, and a 15 deg tilt gate. The hinge preload went from 8.5 to 10 deg to carry the sling's 15 g. | The demo scored 1.0 at step 489, with a peak sling tilt of 2.3 deg. The right arm had a 13.8 mm IK error when it rose 80 mm at release. |
| 4 | The right hand's release lift went from 80 to 50 mm. | All six cheats scored 0, and the recorded demo scored 1.0. A 10.1 mm IK error remained at the approach waypoint 45 mm above the stowed sleeve. |
| 5 | The approach waypoint went from 45 to 30 mm, and stale comments were corrected. | All six cheats scored 0.0 and the demo 1.0 at step 489. |
| 6 | Review fixes. Runs 1 and 2 showed that a grip anywhere but the sling is dragged along, and the sling gate cannot see it. So the instruction now names the handling point and the 1 cm keep-out, and a handling gate zeroes any finger within 5 mm of the boom outside the sling, measured from points filling both fingers. The sling gate now counts only while a hand is on the sling and the tip moves, so a knock on the 10 g bail before cranking no longer gates. Three cheats that never touch the sling were added. Physics constants are unchanged. | All nine cheats scored 0.0 (table above). `grip_canister` and `grip_bar` latched level with the cells 26 mm clear: they reach 0.75 with the handling gate set aside. The demo again scored 1.0 at step 489, latching at 17.7 s. |

The first `hold_still` script still followed the sliders (a script bug) and was fixed before run 2. Apart from the
hinge preload, which was raised for the sling's weight, the physics constants are the design values from run 1: the
spring rate, the latch window, the screw (11 mm per turn, 60,000 N/m, a 1.5 mm clutch) and the 30 mm cell gap.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

The screw model in `task.py` (`Screw`) turns the crank's angle into the two root sliders' drive target: 11 mm of
travel per crank turn, only inside the travel (stowed 0 to full length 32.8 mm). It is self-locking: the target
changes only when the crank turns, never because the boom is pushed or pulled. While cranking, the target never runs
more than 1.5 mm ahead of the slower slider (the crank's slip clutch). The root latch (`_track`) engages when both
sliders are within 0.5 mm of full travel and the hinge is within 2 deg of level. It then holds the hinge where it is
with a 40 N m/rad brake and locks the screw.

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History` plus first-event
records) logs:

- the canister's lowest bottom-rim point above the cell panel's top
- the root hinge's angle from level
- the sling's tilt from vertical, measured from its link pose in the boom's plane
- the tip's extension (0 stowed, 1 at full length, measured along the boom from the hinge) and its speed
- whether the screw advanced that step
- how close each gripper's fingers come to the boom's parts (`hands()`: 270 points filling the last 48 mm of both
  fingers, from 35 mm behind the TCP to the pad tips, 7 mm out from each pad face and 16 to 24 mm across, sized from
  the YAM finger meshes and no more than 6 mm apart, against every collider of every link)
- both sliders against the screw (reported, not scored)
- the latch
- each gripper's pad clearance to the boom and the crank, and its speed

The record is cleared only by the simulator reset. A canister that once came near the cells, a hand that once touched
the boom outside its sling, or a sling that once tilted can never be scored again by lifting the boom back up, letting
go or catching up.

- Gates, any time in the run, give 0 (the earliest is reported):
  - The canister ever within 10 mm of the cells (30 mm when level). This is the 1 cm in the instruction, the flight
    cells' keep-out.
  - The root hinge ever more than 3 deg from level, drooped or pushed up. This is the hinge's rated deflection. It is
    stricter than the keep-out (at 3 deg the canister still clears the cells by 12.6 mm at full length), because a
    root hinge sized for zero g is overloaded well before its boom reaches anything.
  - A finger ever within 5 mm of any boom part but the sling's bail and grip sleeve: the root fitting and screw, the
    sliders, the six bars and the canister with its clevis. This is the handling point the instruction names; flight
    hardware is handled only at its ground-support points.
  - The sling more than 15 deg off vertical (its grip 10.4 mm behind or ahead of its pin) on a step when a finger is
    within 5 mm of the sling and the tip is moving (faster than 2 mm/s, or the screw advancing that step). A tilted
    offload cable side-loads the boom's pins. A brush that swings the 10 g bail while the tip is still, while reaching,
    grasping or letting go, does not count.
- 0: the tip never got 50% of the way out.
- 0.25: the tip has been at least 50% out.
- 0.5: both sliders at full travel (within 0.5 mm).
- 0.75: and latched.
- 1.0: and the release gate: every pad point of both grippers at least 30 mm from the boom and crank, and both
  grippers moving under 5 mm/s, for the last 1 s.

`score()` returns the rung. `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score.

A real trial would use:

- a feeler gauge or video against a scale on the cell panel for the clearance
- a digital inclinometer on the root fitting for the hinge
- the video for the sling's angle, where the hands touch the boom, and the latch's click

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 489
  (24.5 s simulated).
- Runtime on one L4: the recorded demo took 2.1 min wall clock, alone on the VM: 4.8 control steps/s plus about
  25 s of Isaac Sim start-up. A full 1,200-step run takes about 4.7 min. The Dream-1 trial below ran on one L40S: 1,200
  steps in 3.7 min wall clock.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout: all three bodies are fixed to the
  table. The scripted demo is deterministic. Dream-1 is a hosted model without a seed.

## Scored runs

| Run | inspect-robots-dream | yam-arena commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 0.1.0 | 0059d02a1a91 | 66e7d739d22e | 1200 | 4 min | adhoc_f6e85e46.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | - | 0059d02a1a91 | f3139a34fb15 | 489 (success held 1 s) | about 2 min | demo_out/result.json | 1.0 | latched at +0.06 deg; the canister never under 26.6 mm from the cells, the hinge within 1.21 deg of level, the sling within 1.8 deg of vertical and no finger within 31.5 mm of the boom outside its sling; both grippers let go |
| Scripted demo, run again before the Dream-1 trial on one L40S (`linkdemo.py`; the submitted demo video) | - | 0059d02a1a91 | 66e7d739d22e | 489 (success held 1 s) | about 2 min | result.json | 1.0 | latched at +0.03 deg; otherwise as recorded: the canister never under 26.6 mm from the cells, the hinge within 1.21 deg of level, the sling within 1.8 deg of vertical and no finger within 31.5 mm of the boom outside its sling; both grippers let go |

The demo video is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot is
`demo_out/setup.png` (frame 0 of the top camera). Make the Dream-1 run on this exact `task.py` after a 40-step probe
of the instruction (`--max-steps 40`, no frames) to rule out a refusal.

## Materials

- Bus bracket, fixed: a grey aluminium tower 60 x 40 x 82 mm on a 100 x 80 x 6 mm foot, centred at (0.300, 0.125), with
  a gold MLI-film face toward the boom and two hinge cheeks. The root hinge's axis runs along x at (0.300, 0.100,
  0.8375).
- Boom, white-painted aluminium (sRGB 0.92, 0.92, 0.90), 0.22 kg in all:
  - Root fitting (50 g): a 14 mm hinge knuckle, an arm out to a 150 x 12 x 6 mm rail along x 20 mm from the hinge,
    and a black 6 mm left/right-hand lead screw beside it.
  - Root hinge: a torsion spring of 0.9 N m/rad preloaded 10 deg up, damping 0.03 N m s/rad, limits 12 deg up to
    30 deg down. The spring stands in for the boom's own compliance under 1 g.
  - Two dark grey root sliders, 14 x 16 x 10 mm (8 g each), 120.5 mm apart stowed and 54.9 mm at full length.
  - Four scissor bars, 140 x 10 x 5 mm (130 mm pin to pin, 15 g each), in two layers 6 mm apart, crossing at their
    middles.
  - Two half bars, 75 x 10 x 5 mm (8 g each), meeting at the apex.
  - Ten pins about z (seven in the articulation, three closing its loops).
- Tip: a white dummy magnetometer canister, 36 mm across x 30 mm (60 g), pivoting on the apex pin. Friction 0.9/0.8.
  - Offload sling, the boom's only handling point: a grey 5 x 5 mm steel bail (10 g), pinned about x 12 mm above the
    apex on a clevis, 40 mm up to a 36 mm axle. Its light spring (0.05 N m/rad) stands it upright when nobody holds it,
    within limits of +-40 deg.
  - Grip sleeve: black rubber, 12 mm across x 26 mm (5 g), turning freely on the axle. Friction 1.0.
- Hand crank, fixed: a black right-angle gearbox 40 x 40 x 50 mm (0.8 kg) at (0.300, 0.250), top at z 0.8075.
  - A black crank on top turns about a vertical axis (20 mm hub, 55 mm arm, 60 g).
  - A black rubber handle, 14 mm across and 34 mm tall (15 g, friction 1.0), spins freely at 45 mm radius.
  - 11 mm of slider travel per turn (2.98 turns stowed to full length).
  - The screw is self-locking; a slip clutch in the crank hub slips 1.5 mm ahead of a stalled slider.
- Solar-cell test panel, fixed: 120 x 305 x 8 mm, blue cells (sRGB 0.10, 0.18, 0.55) on an aluminium honeycomb
  substrate, centred at (0.300, -0.0925), top at z 0.7710, on two 100 x 12 x 15.5 mm aluminium legs (0.6 kg in all).
  Its top is 30 mm below the canister of a level boom.
- Layout chosen for reach: the crank sits where only the left arm reaches it, and the tip's path where only the right
  arm can follow it. The crank's vertical axis and freely spinning handle let a straight-down hand turn it without
  rolling the wrist. The sling's sleeve lets a straight-down hand pinch it while the bail swings.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed.

1. Release the root latch and crank the boom fully home (2.98 turns back), so the two root sliders sit 120.5 mm apart.
2. Let the tip hang on the hinge spring (1.2 deg below level, canister 26.7 mm above the cells) with the sling standing
   upright.
3. Leave the crank handle pointing forward (+x).

The bracket, gearbox and cell panel are fixed on taped outlines and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code. Each file carries an SPDX header, and `linkkit.py` and `linkdemo.py` carry the full MIT text. They use only
public Isaac Lab and USD Physics techniques and copy no code from other tasks in this repository. The scene uses no
third-party assets or brands.
