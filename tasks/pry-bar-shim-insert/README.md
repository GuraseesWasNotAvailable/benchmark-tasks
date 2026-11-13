# Lever a door's hinge side up with a flat pry bar and slide a packer under it

One hand holds a flat steel pry bar down on its fulcrum, which keeps the foot of the bar lifting the door's hinge side.
The other hand slides a 6 mm plastic packer into the gap that opens under the door. Let the bar go and the door settles
back onto its hinge washers, so the packer only stays under the door if it went in while the bar was held. At the end
the packer lies flat across the door's bottom edge on the hinge side, the door rests on it, and both grippers have let
go. Carpenters and general maintenance and repair workers do this to cure a sagging, rubbing door.

Instruction given to the policy (26 words; it states the goal only, never the technique):

`Fit the yellow packer under the hinge side of the door using the red pry bar, so the door rests on the packer, then let go.`

Both arms have a forced role, and the physics, not the instruction, forces it:

- Right arm only: the door hangs on its hinge washers 4.5 mm above the floor, and the packer is 6 mm thick. A packer
  pushed at the door butts against its bottom edge and goes no further (`right_only`).
- Left arm only: the bar lifts the door 10.7 mm, but the door's 2.5 kg rests on the bar's foot, so it stays up only while
  the handle is held down; with no packer in, letting go closes the gap (`left_only`). A bar let go before the packer
  goes in gives the same result (`release_early`).

The ability is holding a pivoted lever down to keep a gap open while the other hand fits a spacer into it. It is not
a pick and place: the packer is pushed along the floor, and nothing is placed by choosing a spot.

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the
submitted runs:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 8a94042be6e5 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the door as a fixed PhysX articulation (a jamb post and a door leaf on a vertical prismatic
joint, the lift-off hinge pins), the pry bar and its fulcrum as another (a bar on a revolute joint between two cheek
plates), and the packer as a compound rigid body, all built from box colliders in `task.py`. It also calls the task's
per-step record after every control step. No fork of yam_arena and no third-party assets. `linkkit.py` and
`linkdemo.py` are the program's own helpers, written for these tasks, and carry their own licence text.

## Offline check, no simulator run

```bash
python -m pytest -q tests
```

34 tests feed synthetic records to the scorer (the task module imports yam_arena for the table height, so source the
environment file first): a demo-shaped trace scores 1.0; nothing, `left_only`, `right_only` and `release_early` shaped
traces score 0; the door lift of 6.9 mm earns nothing and 7.0 mm earns the packer rungs; the packer's front edge at
295.9 mm scores 0, 296.0 mm scores 0.25, 305.9 mm still 0.25 and 306.0 mm scores 0.5; 345 mm is the farthest it may
sit; the seated limits (centre 0 to 90 mm across the door, yaw 12 deg either way or turned end for end, tilt 5 deg,
height within 1.5 mm); the door's rest window 0.8 to 3.5 mm above its washers; a door still held up by the bar at the
end is 0.5; the door still 1 s and both grippers 30 mm clear and under 5 mm/s for 1 s (0.75 until then); the 0.5 rung
survives the packer being pulled out again, so a later attempt cannot clear it; and the record is clean after a reset.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the yellow packer under the hinge side of the door using the red pry bar, so the door rests on the packer, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/pry-bar-shim-insert-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

```bash
inspect-robots inspect adhoc_4b902152.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_4b902152.json
```

## Scored runs

| Run | yam-isaac-lab-arena | Commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 1.0.0 | 0059d02a1a91 | 8a94042be6e5 | 1200 | 7 min | adhoc_4b902152.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | git checkout | 0059d02a1a91 | 8a94042be6e5 | 365 (success held 1 s) | about 2 min | result.json | 1.0 | packer 313 mm forward, door 1.4 mm up on it, both hands clear |

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant left_only     # a cheat variant (or DEMO_VARIANT=left_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the poses of the bar, the door and the packer, the bar's angle computed from its link
poses, and the robot's own grippers. It never reads joint values or the record.

1. Right: with the closed fingers pointing down, goes behind the packer (its pad tip 2.5 mm above the floor, 10 mm
   behind the rear end) and waits there. Left: closed fingers on top of the pry bar's handle, 140 mm behind its pivot.
2. Left presses the handle down in steps until the seen bar angle is 13 deg, the door's hinge side rising 10.7 mm (the
   gap under it is then 15 mm). It holds.
3. Right, once it sees the bar at 11 deg or more, pushes the packer 56 mm along the floor under the door; the packer
   stops with its front end 313 mm from the arm bases, 27 mm past the door's front face, flat and turned 7 deg.
4. Left eases up 30 mm over 2 s, so the door settles onto the packer (1.4 mm up), then lifts clear to 140 mm above the
   handle; right backs off 60 mm and lifts clear. Both wait.

The demo scores 1.0 (`success`, episode ended at step 365 of 1,200). Worst IK error 0.1 mm and 0.01 deg. A second reset
returns the door, bar and packer to their start values and clears the record.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (8a94042be6e5); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. | 0.0 |
| `left_only` | Left presses the bar to 13 deg, holds 3 s and lets go; right stays at ready. | The door goes up 10.7 mm and comes back down; the packer never moves. | 0.0 |
| `right_only` | Right pushes the packer at the shut door; left stays at ready. | The packer's front end stops at the door's front face (286 mm), 6 mm against a 4.5 mm gap. | 0.0 |
| `release_early` | Left lets the bar go 0.5 s after reaching 13 deg; right pushes the packer 5 s after it sees the bar up. | The door has closed again; the packer stops at the door's front face (286 mm). | 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures: the door's lift
q above its hinge washers (mm), the bar's angle (deg), the packer's front end along x (mm from the arm bases), centre
across the door (y, mm), yaw and tilt (deg) and height (mm), the door's speed from its change in lift between control
steps, and each gripper's clearance to the door, bar and packer and its speed. The record keeps flags and held times
for the whole episode and is cleared only by the simulator reset, so pulling the packer out and starting again never
clears what has been earned.

- 0: no step had the door lifted at least 7 mm with the packer's front end at least 10 mm under the door
  (front end at 296 mm or more).
- 0.25: that happened (the packer went at least 10 mm under the lifted door), but the packer was never fully under it
  while the door was up.
- 0.5: the packer was seated while the door was lifted 7 mm or more (front end 306 to 345 mm, centre 0 to 90 mm across
  the door toward the hinge, yaw at most 12 deg, tilt at most 5 deg, within 1.5 mm of lying flat), but at the end the door
  does not rest on it (door lift outside 0.8 to 3.5 mm) or it is no longer seated.
- 0.75: seated and the door resting on it at the end (lift 0.8 to 3.5 mm, a gap under the hinge corner of 5.3 to 8.0 mm),
  but the door has not been still for 1 s (under 2 mm/s) or a gripper is within 30 mm of the door, bar or packer or
  moving at 5 mm/s or more.
- 1.0: the 0.75 state, the door still for the last 1 s, and the release gate: every pad point of both grippers at least
  30 mm from the door, bar and packer and both grippers moving under 5 mm/s for the last 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S` = 1.0 s), and `distance()` is 1 - score.

Real trial, each check with a stated tool: a tape from the door's front face to the packer's front end (20 mm or more
past the face and no more than 59 mm), the packer's centre 0 to 90 mm from the door's mid-line toward the hinge; a try
square for the packer's yaw (12 deg) and a 6.0 mm feeler for it lying flat; feeler gauges under the hinge corner for the
door's gap (5.3 mm enters, 8.1 mm does not); 60 fps video with a mark on the door edge for the 7 mm lift (a gap of 11.5 mm
or more) and for the door and each hand being still (2 mm/s and 5 mm/s over 1 s); a ruler for the 30 mm hand clearance.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 365.
- Runtime on one L40S: a demo run took about 2 min wall clock including Isaac Sim start-up, a few minutes more on the
  first run after boot. A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout. The scripted demo is deterministic
  (repeat runs ended in the same state to 0.01 mm); Dream-1 is a hosted model without a seed.

## Materials

- Cabinet door leaf: pine, 28 x 350 x 500 mm (a real wardrobe or cabinet door size), 2.5 kg, natural wood colour,
  its bottom 4.5 mm above the floor, centred at x 0.300 m, y 0 (front face at x 0.286 m). It hangs on two lift-off hinge
  pins in a jamb post of 40 x 20 x 520 mm pine at its left edge (y 0.175 to 0.195 m); the pins let it slide 20 mm up and its
  hinge washers stop it 4.5 mm above the floor. The pins are greased (modelled as 100 N s/m of damping and 0.5 N of
  friction).
- Flat pry bar: red-painted steel flat, 30 mm wide and 4 mm thick, 0.28 kg: a 160 mm handle behind the pivot, a vertical web
  and a 45 mm foot of 3 mm thick bent down to the floor and 20 mm under the door's hinge-side bottom edge (1 mm below it
  at rest). The pivot axis is 40 mm above the floor, along y, at x 0.261 m, y 0.150 m, between two grey steel cheek plates
  (30 x 5 x 54 mm each) bolted to the floor. The handle's top face is 140 mm behind the pivot where the left hand presses.
  Friction 0.4 / 0.3; the bar's travel is limited to 20 deg.
- Packer: yellow plastic (PVC), 100 x 40 x 6 mm, 34 g, lying on the floor in front of the door, centre x 0.220 m, y
  0.010 m, long axis toward the door. Friction 0.5 / 0.4.
- Layout and physics chosen for reach and feasibility: the door is a cabinet-size leaf (500 mm tall) so it stands inside the
  arms' 0.35 m reach, the table top stands in for the floor, the hinge pins are a vertical slide, and the packer lies
  140 mm across from the bar's handle so the two hands do not meet.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Pull the packer out and lay it on the floor in front of the door with its centre 220 mm from the arm
bases and 10 mm to the left of the arm-base midline (a pencil outline helps). Let the door down onto its hinge washers
(a 4.5 mm feeler gauge fits under its bottom edge at both ends). Set the bar's foot 20 mm under the door with the handle
up: the top of the foot touches the door's bottom edge (the bar rests at about 1 deg). The fulcrum and jamb stay bolted.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
