# Lock a floating guide rod at the end of travel, where the carriage sets it

The robot locks a loose guide rod at the moment a moving carriage holds it straight. A manual lead-screw linear slide,
the hand-cranked kind used on fixtures, jigs and inspection stages, has two parallel guide rods: steel rod A is fixed,
and the orange rod B is pinned at its near (left) end while its far end floats in a slotted clamp block, whose light
preload spring keeps it from rattling by pushing it against one side of the slot, 0.5 deg out of line. The blue
carriage rides on both rods and is driven by the slide's hand crank through a self-locking lead screw. The robot
cranks the carriage out to the far stop, where its bushing has drawn rod B into line, presses the red cam lever flat
to lock the rod there, cranks the carriage home again to show it runs freely, and lets go. Electromechanical equipment
assemblers (O*NET 51-2023.00) building linear slides and positioning stages do exactly this: they leave the second
rod loose, run the carriage to the end of its travel and tighten the clamp there, so the over-constrained pair lines
itself up. Industrial machinery mechanics (49-9041.00) do it again after a rod or bearing is replaced.

The ability is letting a moving part set a loose member and locking the member where that part holds it straightest:
not where it lies, not part-way along the stroke, and not after the part has left. Nothing holds rod B but the moving
carriage, and the decision is where along its stroke to lock. That separates it from squaring a racked gantry beam
(TK-RWRT5A), where the robot moves the member; from setting a carriage preload for low drag (TK-ZEREPP), where the
robot dials in an adjustment; from holding a sprung belt tensioner at its mark and turning its lock knob (TK-B5F4YZ),
where the robot's hand holds the member; and from drifting a sprung splice plate into line and bolting it (TK-K8D6ZZ),
where the robot's own tool is the aligner. Squaring rod B by hand is allowed, and a rod pushed within 0.12 deg and
locked is also a good real-world result that scores. It is the hard way: the far end must be set to within 0.47 mm
of a line that nothing on the table shows (the 0.5 deg skew is 2 mm at the rod's tip and cannot be seen by the
320 x 176 cameras), while the carriage sets it to 0.09 deg for free. The task tests the procedure, not visual
alignment.

Physics, not the instruction, defeats the natural plans. Rod B's skew grows along the stroke; the carriage's plain
bushing on it has 0.3 mm of radial play, so it first bears on the rod 34 mm out from the pivot and from there drags the
rod's far end into line, the straighter the farther out it is (0.50 deg at the start, 0.16 deg half-way to the far
stop, 0.09 deg at it; in the demo run 0.36 deg at 18 mm, 0.14 deg at 90 mm and 0.090 deg at the stop).

- Lock it as found, then crank: the bushing meets the rigid, skewed rod a few millimetres out, wedges, and the screw's
  slip clutch slips: the carriage sticks.
- Lock it with the carriage half-way: the rod is held only to 0.16 deg, so the bushing jams on the way to the far end.
- Crank out and back, then lock: the preload spring has pushed the rod back out of line as the carriage left, so it
  sticks like the first.
- Lock it while the carriage sits at the far stop: the rod is held to 0.09 deg and the carriage runs home freely.

The crank is at the left end and the lever at the right, about 300 mm apart. The right arm cannot reach the crank's circle
with a straight-down hand (IK error 35 to 145 mm) and the left arm cannot reach the lever (29 to 37 mm), so the left
arm cranks and the right arm locks. The self-locking screw parks the carriage at the far stop with no hand on it.

Instruction given to the policy (29 words; it states the goal and asks for the crank run that shows it, never where
or when to lock):

`Lock the loose orange guide rod so the blue carriage runs the full length of the slide and back without sticking, show it with the crank, then let go.`

Files in this folder: `task.json`, `task.py` (scene, screw model, whole-run record, scoring and the scripted demo),
`linkkit.py` (articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (screw, scorer and geometry edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab). Versions used for the demo and cheat
runs below, on one NVIDIA L4:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: 98984313ac85 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in
  a run log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the axis as one PhysX articulation (the fixed plate with rod A and the blocks, the carriage
on a prismatic joint with a stiff drive, rod B on a sprung revolute joint with a switchable brake, and the cam lever on
a revolute joint with an over-centre detent) and the crank as a second one (the fixed gearbox, the crank on a revolute
joint and the handle spinning freely on it; no joint limits, so they turn without end), spawned through Isaac Lab's
`ArticulationCfg`, and calls the task's per-step record after every control step. No fork of yam_arena and no
third-party assets: every body is built from box, cylinder and capsule colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

32 tests, no simulator (they import yam_arena for the table height). The screw: one crank turn moves it one lead
(32 mm), also when the crank angle reads wrapped across +-180 deg, and back; it stops at both ends of the travel; its
slip clutch keeps it 3 mm ahead of a stuck carriage and a reverse turn backs it off at once; a reset clears it. The
Record and ladder: a demo-shaped record scores 1.0; the release gate needs 1.0 s (0.95 s is not enough), restarts on a
pad at 29 mm or a hand at 6 mm/s, and accepts 30 mm; a lag of 1.5 mm behind or ahead of the screw for 0.15 s gates the
whole run even if the rod is later unlocked, squared and run properly, while 1.49 mm, or 2 mm for 0.10 s, does not;
rod B locked 0.120 deg out passes and 0.121 deg does not; an unlocked rod scores 0 even after both runs; a rod locked
after the carriage has gone home (square in the synthetic record) scores 0.25, locked at the far stop without the
return 0.5; the stops count within 1.0 mm and not 1.1 mm; the far stop alone is not the return (far, then home);
reopening the lever restarts the clamp's record; a reset clears the record. Geometry, from the axis's own link poses:
the bushing has 0.3 mm of play each side of a square rod all along the travel; the rod as it arrives clears the
bushing at the home stop and meets it between 4 and 5 mm out; the rod angle at which the bushing just touches it is
0.090 deg at the far stop and 0.156 deg half-way, either side of the 0.12 deg tolerance; a rod locked at the far stop
clears the bushing all the way home, and one locked half-way overlaps it by 0.22 mm at the far stop; the carriage
clears the crank's handle circle and the lever at its stops.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lock the loose orange guide rod so the blue carriage runs the full length of the slide and back without sticking, show it with the crank, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/guide-rod-clamp-at-far-end-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

```bash
inspect-robots inspect adhoc_551b9081.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_551b9081.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                            # scripted demo
python linkdemo.py task.py cheat_out --variant clamp_as_found  # a cheat variant (or DEMO_VARIANT=clamp_as_found)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the carriage's place along the rods, the crank's axis and the handle's position, the
lever's angle (all from link poses), and the robot's own grippers. It never reads rod B's angle, the screw, the clamp
state or the record.

1. Both together. Left: straight down on the crank handle, pads across it 10 mm above the crank arm (opened to
   40 mm, closed to 8 mm on the 14 mm handle). Right: waits 170 mm above the lever with its fingers closed.
2. Left: turns the crank 5.17 turns, a sixth of a turn past the far stop, by circling the held handle about the
   crank's axis in 14 s; the wrist keeps its orientation while the handle spins on the crank arm. The carriage is
   seen at the far stop (159.98 mm) at 17.3 s. On the way its bushing draws rod B from 0.50 to 0.090 deg.
3. Right: presses the open lever down with closed fingers 40 mm out from its hinge, to 3 mm past flat; it snaps flat
   (seen -1.0 deg, its stop) and the clamp locks rod B at 0.0905 deg at 21.0 s. The right hand lifts away. The left
   hand keeps hold of the handle, and the self-locking screw holds the carriage at the stop.
4. Left: cranks home (seen 0.02 mm at 37.9 s), lets go and lifts 80 mm; both hands wait, clear and still.

The demo scores 1.0 (`success`, episode ended at step 832 of 1,200). The carriage ran out and home with a peak lag
of 0.13 mm behind its screw (the jam gate is 1.5 mm), rod B was locked at 0.0905 deg with the carriage at the far
stop, and at the end both grippers were 59 and 104 mm clear. Worst IK error 0.09 mm and 0.01 deg. A second reset (`--check-reset`) returned
every joint to its initial value at rest (the open lever sags 1.6 deg under its own weight, as at the first reset),
with the record cleared.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (98984313ac85); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; rod B stays 0.50 deg out and unlocked. | 0.0 |
| `no_clamp` | Cranks out to the far stop and home again, never touching the lever. | The carriage runs freely both ways (peak lag 0.13 mm); rod B springs back to 0.50 deg as it comes home and is never locked. | 0.0 |
| `clamp_as_found` | Presses the lever flat first, then cranks out and home. | Rod B is locked at 0.50 deg. 4.7 mm out the bushing wedges on it, the screw runs 3 mm ahead and the clutch slips: gate at 11.8 s. The wedged carriage would not crank back either. | 0.0 |
| `clamp_mid` | Cranks out 80 mm (half the travel), presses the lever flat, cranks on to the far stop, then home. | Rod B is locked at 0.157 deg. The bushing starts to bind at 88 mm (gate at 18.4 s) and stops dead at 94 mm; it cranks home. | 0.0 |
| `out_and_back` | Cranks out to the far stop and home, presses the lever flat, then cranks out and home again. | Coming home, the preload spring walks rod B back from 0.09 deg to 0.50 deg (0.23 deg at 44 mm, 0.38 deg at 15 mm). Locked at 0.50 deg, the carriage sticks 4.7 mm out on the next run: gate at 39.7 s. | 0.0 |

## Build log

| Run | Change | Result |
|---|---|---|
| 1 | first build | 1.0 at step 832: rod B drawn from 0.50 to 0.090 deg, locked at the far stop, home with a peak lag of 0.13 mm |
| cheats | all five on the first task file | all five 0.0 |
| 2 | review edits only: retitled, framed as a manual lead-screw slide, rod B's spring named as its floating end block's preload spring, instruction reworded; physics, scorer and demo unchanged | 32 tests pass; the recorded demo with `--check-reset` scores 1.0 at step 832, the same end state as run 1 |
| cheats | all five on the submitted task file (98984313ac85), one at a time on one L4, each `result.json` kept | all five 0.0 (table above), each gate at the same time and place as in run 1 |

The first build worked as designed, so the bushing play (0.3 mm), the preload spring (0.3 N m/rad), the clamp's
stiffness (20,000 N m/rad) and the clutch (3 mm at 4,000 N/m) are the design values. Each cheat failed the way the
geometry predicts (the tests compute the same touch points from the link poses).

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

The screw model in `task.py` (`Screw`) turns the crank's angle into the carriage's drive target: 32 mm of travel per
crank turn, only inside the travel (home stop 0 to far stop 160 mm), and never more than 3 mm from where the carriage
actually is (the crank's slip clutch, about 12 N at the 4,000 N/m drive). A carriage that binds therefore stops while
the crank turns on, and the gap between the screw and the carriage (the lag) grows to 3 mm.

Every control step from reset the on-step record (`Record` in `task.py`, a `linkkit.History` plus the clamp and jam
events) logs the carriage's place q, its lag behind the screw, rod B's angle from parallel to rod A, the clamp (closed
while the lever is within 10 deg of flat; it frees again past 20 deg), and each gripper's pad clearance to the
carriage, rod B, lever, crank and handle and its speed. Each time the clamp closes it starts a record of where the
carriage has been since; reopening the lever ends it. The record is cleared only by the simulator reset, so a carriage
that stuck once can never be scored again by unlocking, squaring and re-running.

- Gate, any time in the run, gives 0: the carriage 1.5 mm or more behind (or ahead of) its screw for 0.15 s: it
  stuck, or a hand forced it. The demo's peak lag is 0.13 mm.
- 0.25: the clamp is closed with rod B within 0.12 deg of parallel to rod A (0.47 mm over the rod's length). A rod
  locked at the far stop sits at 0.09 deg; half-way it would be 0.16 deg and as found 0.5 deg.
- 0.5: and since that clamp closed the carriage has been at the far stop (within 1 mm); locked while parked there
  counts.
- 0.75: and after that it has been back at the home stop (within 1 mm): the full length out and back with the rod
  locked and never stuck.
- 1.0: and the release gate: every pad point of both grippers at least 30 mm from the carriage, rod B, lever, crank
  and handle and both grippers moving under 5 mm/s for the last 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: a dial indicator on rod B's far end against rod
A for the angle, a top photo of the lever, and the video for the carriage's travel to both stops and any stall (the
crank's slip clutch clicks).

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 832.
- Runtime on one shared L4: the recorded demo took 3.0 min wall clock alone on the VM (5.5 control steps/s plus about 1.5 min of
  Isaac Sim start-up); with two other simulations sharing the GPU it ran at 2.2 to 3.8 steps/s (about 6 to 9 min for
  1,200 steps). A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (both articulations are fixed to
  the table). The scripted demo is deterministic (the first run and the recorded run ended in the same state, step for step); Dream-1 is a hosted model without a seed.

## Scored runs

| Run | inspect-robots-dream | yam-arena commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 1.0.0 | 0059d02a1a91 | 98984313ac85 | 1200 | 7 min | adhoc_551b9081.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | - | 0059d02a1a91 | 98984313ac85 | 832 (success held 1 s) | about 3 min | demo_out/result.json | 1.0 | rod B locked at 0.09 deg with the carriage at the far stop; it cranked home freely; both grippers let go |

The demo video is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot
`demo_out/setup.png` (frame 0 of the top camera). Make the Dream-1 run on this exact `task.py` after a 40-step probe
of the instruction (`--max-steps 40`, no frames) to rule out a refusal.

## Materials

- Base, fixed: a grey aluminium tooling plate 130 x 400 x 10 mm (3.0 kg with its blocks) centred at (0.310, 0.040),
  top at z 0.7575, with dark grey steel rod blocks (20 x 12 x 38 mm for rod A; a 24 x 16 x 38 mm pivot block for
  rod B at (0.350, 0.130); a 26 x 16 x 38 mm floating far clamp block for rod B at (0.350, -0.088) with an orange slot
  mark), a black 10 mm lead screw along y at x 0.310 and its end bearing.
- Rod A: a ground steel guide rod, 8 mm x 250 mm, fixed along y at x 0.270, centre height z 0.7875.
- Rod B: an orange-painted steel guide rod, 8 mm x 232 mm, 90 g, along y at x 0.350, 80 mm from rod A; pinned about a
  vertical axis at its near end (y 0.130); a light preload spring in the floating far clamp block (0.3 N m/rad) pushes its
  far end 2.0 mm outward against one side of the slot (0.5 deg), so it cannot rattle; the slot allows -1.0 to +1.0 deg. Friction 0.35 / 0.30.
- Carriage: blue anodised aluminium, 0.40 kg: a 110 x 30 x 8 mm plate over both rods (1 mm above them), a 24 x 40 x
  18 mm linear-bearing housing on rod A (0.5 N of friction), a 12 mm-long plain polymer bushing on rod B with 0.3 mm
  radial play (8.6 mm between its side walls), and a dark screw nut. Travel 160 mm, from the home stop (bushing on rod
  B 24 mm from the pivot) to the far stop. Friction 0.35 / 0.30.
- Hand crank: a black right-angle gearbox 40 x 40 x 50 mm (0.8 kg) at (0.310, 0.200) driving the lead screw, with a
  black crank on top turning about a vertical axis (20 mm hub, 55 mm arm, 60 g) and a black handle, 14 mm across and
  34 mm tall (15 g, rubber grip, friction 1.0), spinning freely at 45 mm radius. 32 mm of carriage travel per crank
  turn, five turns end to end; the screw is self-locking (the carriage stays where the crank leaves it) and a slip
  clutch in the crank hub slips at about 12 N of screw thrust.
- Cam clamp lever: red, 50 x 10 x 5 mm with a 14 x 12 x 8 mm grip, 20 g, hinged along x on top of the far clamp block
  (hinge at (0.350, -0.092, 0.798)); open 50 deg up, flat when locked, with an over-centre spring (detents at both).
- Layout chosen for reach: the crank sits where only the left arm reaches it and the lever where only the right arm
  does; the crank's vertical axis and freely spinning handle let a straight-down hand turn it without rolling the wrist.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Flip the red cam lever open (50 deg up), which frees rod B. Crank the carriage home to the left
stop. Check rod B's far end rests against its preload spring 2.0 mm outward (a 2 mm feeler gauge against the slot mark), and
leave the crank handle pointing forward (+x). The plate, rod A, the gearbox and the blocks are fixed on taped outlines
and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques and copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
