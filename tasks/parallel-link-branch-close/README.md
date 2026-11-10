# Refit a desktop arm's parallel link on the closure branch that keeps its wrist level

The robot pins the free end of a desktop parallelogram arm's parallel link onto its base pin so that the four-bar loop
closes as a parallelogram, which keeps the arm's wrist plate level, and not as the crossed loop, which locks it off
level. The bench model is the passive level-keeping four-bar of a desktop parallelogram arm at about its own scale
(110 mm links): a braked upper arm, a wrist plate pinned to its tip, and a parallel link from the plate's rear end to a
pin on the base. Palletizing robots keep their wrist flange level with the same four-bar, about ten times larger.
Robotics technicians, electromechanical assemblers, industrial electronics repairers and machinery mechanics (O*NET
17-3024.01, 51-2023, 49-2094, 49-9041) refit this link by hand after a motor, bearing or link swap; pinned on the wrong
branch, it locks the wrist off level.

The ability is choosing the closure branch of a linkage loop. Both closures snap, so pinning the link is not what is
tested: which loop the link closes is.

How it differs from earlier hold-and-pin tasks: in TK-W6W94V the instruction names the 90 deg turn and the pin's hole,
in TK-B5F4YZ it names the mark to hold, and in TK-46W443 the fallen platform has one pinned pose to restore. Here
nothing says to turn the plate and there is no mark or hole to find. The link clicks shut in two places, and pinned
where it hangs it closes the crossed loop and locks the plate off level, so the robot has to see which way the loop
will close before it fits the link.

Physics, not the instruction, defeats the natural plan. With the link off its pin the wrist plate is free on its
pivot (it is balanced there), so the link's weight hangs the plate's rear end straight down: the plate stands on end.
The geometry puts that hanging pose exactly on the crossed closure (base pivot spacing = plate lug length =
110 mm x (sin 65 deg - cos 65 deg) = 53.2 mm). Swinging the hanging link 65 deg straight up onto the red pin clicks
the snap, but the loop is then an anti-parallelogram with no freedom left: the plate is locked 89 deg off level, and
turning it by force only loads the arm's brake. The right plan turns the plate 90 deg to level first and holds it
there, so the link needs only a 25 deg swing onto the pin; once it clicks, the closed parallelogram holds the plate
level on its own. The demo does this with two arms (left holds the plate, right swings the link). One arm gripping
the link firmly could in principle drag the plate round with it, so the two-arm role is natural rather than forced.

Instruction given to the policy (21 words; it states the goal only, never the branch or the order):

`Fit the grey parallel link onto the red pin so the yellow arm's blue wrist plate sits level, then let go.`

## Files

- `task.json`: task card (setup, scoring, run limit, reset notes, fidelity).
- `task.py`: the scene (one linkkit articulation), the whole-run record, scoring and the scripted demo with its cheat
  variants.
- `linkkit.py`: articulated rigid-link bodies for yam_arena (an import hook spawns the linkage through Isaac Lab's
  `ArticulationCfg`; USD revolute and prismatic joints with limits, drives and a switchable brake).
- `linkdemo.py`: closed-loop scripted-demo runner (full-pose IK, both arms, observable state only).
- `tests/test_scoring.py`: 26 scorer and geometry tests, no simulator.

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. No fork of yam_arena and no third-party assets: every body
is built from box, cylinder and capsule colliders in `task.py`.

## Run a trial

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the grey parallel link onto the red pin so the yellow arm's blue wrist plate sits level, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames \
  --log-dir ~/yam-eval/parallel-link-branch-close-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Read a log with:

```bash
inspect-robots inspect adhoc_98f6e28d.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_98f6e28d.json
```

Offline scorer check, no simulator (needs `yam_arena` importable, as on the launchable):

```bash
python -m pytest -q tests
```

The tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; the brake gate at 1.49 vs 1.5 deg,
and the gate beats a perfect end; a snap on the crossed closure (plate 90 deg) scores 0; level at +-3.0 vs 3.01 deg;
the snap window edges (along the link -1.0 and +1.8 mm, across it 3.3 mm, tongue 1.0 mm); the 0.25 rung at 10.0 vs
10.01 mm and only with the plate level; snapped level and then unclipped scores 0.5; still held, a pad 29.9 mm away or
a hand moving 6 mm/s scores 0.75; 0.95 s of release is not enough and one 3 deg/s plate step restarts the stillness
timer; the record is clean after a reset. On real link states from the linkage's kinematics: both closures are
110 mm from the pin, the reset pose reads plate 90 deg and link -90 deg, the parallelogram reads snapped and level,
the crossed closure snapped and 90 deg off, a tongue held back 5 mm is not snapped, and a hand on the plate is seen
by the clearance. The sweep test swings the eye onto the pin along both branches: the link's own walls never touch
the pin (0.58 mm closest), the tongue must give way at most 6.4 mm of its 10 mm, and it is home again 1.5 deg before
the pocket centre.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                         # scripted demo
python linkdemo.py task.py cheat_out --variant short_swing  # a cheat variant (or DEMO_VARIANT=short_swing)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the link poses (the plate's and link's in-plane angles, the pin's place in the eye, the
tongue's position) and the robot's own grippers. It never reads joint values, the brake or the record. The tongue is a
separate black body whose front face is flush with the link's robot-facing face, beside the pin's exposed end, so its
place in the eye is a camera-visible body pose, not an internal latch flag. Both grips come from the robot's side, along
the hinge axes, with the pads closing across the bar, so the wrist roll turns the part about its pin.

1. Left: grips the standing plate 43 mm above its pivot (pads across its 20 mm width, approach 9 deg down), turns it
   90 deg clockwise about the seen pivot with the wrist following, corrects once from the seen angle (89.7 deg, then
   1.0 deg left) and holds it level.
2. Right, once the plate is level and the link has hung still for 1 s: grips the link 24 mm short of its eye
   (approach 14 deg down), swings it 26.6 deg clockwise about its pin on the plate toward 1 deg past the parallel (plus
   a 0.9 deg correction from the seen angle); the eye's lead-in pushes the tongue back (3.9 mm at 18.25 s, 6.7 mm at 18.5 s) and it clicks
   shut behind the pin at 18.75 s. A seen-pose check confirms the pin is inside the eye with the tongue home, then the
   right hand lets go and backs off.
3. Left lets go and backs off; both wait.

The demo scores 1.0 (`success`, episode ended at step 589 of 1,200): plate 0.8 deg off level at the end (the pin
rests on the tongue and the eye's inner wall, 0.7 mm short of centre, as designed), link at -113.6 deg, the arm
0.02 deg off its brake (peak 0.06 deg), plate still for the last 4.75 s, both grippers 83 and 81 mm clear. Worst IK
error 0.08 mm and 0.02 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (724526df9148); every one scores
exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Plate stands on end (90.0 deg), link hangs, eye 120 mm from the pin. | 0.0 |
| `short_swing` | The natural plan: right grips the hanging link and swings it 66 deg straight up onto the pin; the plate is not touched. | The tongue clicks shut at 8.0 s on the crossed closure; the plate is locked 89.2 deg off level, link at -153.3 deg. | 0.0 |
| `plate_only` | Left turns the plate level (within 0.3 deg), holds 1 s and lets go; the link is never fitted. | The link's weight swings the plate back on end within 2 s (90.0 deg). | 0.0 |
| `force_level` | `short_swing`, then left grips the locked plate and drives a 89 deg turn toward level. | The closed crossed loop does not give: the plate moves 1.2 deg at most, the arm bends 0.98 deg on its brake (gate 1.5), and both spring back to 89.2 deg when the hand lets go. | 0.0 |

## Scoring (`score()` at the end of the run; `success()` = 1.0 held 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures from link
poses: the plate's tilt from level (deg; +90 standing on its rear end), the upper arm's angle, the red pin's centre in
the link's eye (mm along and across the link from the pocket centre), the tongue's retraction, the plate's speed and
spin (from its pose change between control steps, what a camera sees), and each gripper's pad clearance to the whole
linkage and its speed. The record keeps flags, peaks and held times for the whole run and is cleared only by the
simulator reset.

- Snapped: pin centre inside the pocket (-1.0 to +1.8 mm along the link, within 3.3 mm across it) and the tongue back
  within 1 mm of its stop. Level: tilt within 3 deg.
- Gate, any time in the run, gives 0: the upper arm forced 1.5 deg or more off its braked 65 deg. A real arm's brake
  must never be back-driven; it also closes the only way to move a closed loop.

| Score | Condition |
|---|---|
| 0.25 | at some time the plate level while the eye's pocket centre was within 10 mm of the pin |
| 0.5 | at some time snapped with the plate level |
| 0.75 | at the end snapped with the plate level |
| 1.0 | also the plate still (under 3 mm/s and 3 deg/s) for the last 1 s, and both grippers' pads at least 30 mm from the base, arm, plate, link and tongue and moving under 5 mm/s for the last 1 s |

A snap on the crossed closure leaves the plate about 90 deg off level and earns nothing. `distance()` is 1 - score.
Real trial: a protractor or a top photo for the plate tilt and the arm angle; the snap is visible and audible; the
video for stillness and the release.

## Scored runs

| Run | yam-isaac-lab-arena | Commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Scripted demo, recorded (`linkdemo.py`) | git checkout | 0059d02a1a91 | 724526df9148 | 589 (success held 1 s) | about 4 min | result.json | 1.0 | snapped on the parallelogram, plate +0.8 deg, both grippers let go |
| Scripted demo, repeat (`--no-record --check-reset`) | git checkout | 0059d02a1a91 | 724526df9148 | 589 (success held 1 s) | about 4 min | result.json | 1.0 | same end state to 0.001 deg and 0.01 mm; after a second reset every joint is back at its start (the link hangs 1.4 deg off vertical, its own centre of mass) and the record is clear |
| Dream-1 (`dreamlabs/dream-1-beta`), seed 0 | git checkout | 0059d02a1a91 | 724526df9148 | 1200 | about 3 min | adhoc_98f6e28d.json | 0.0 | max_steps |

Dream-1 run: log `adhoc_98f6e28d.json`, trial `scene-0-e0`, made at filing with the command in "Run a trial" on this
exact `task.py` (724526df9148): `task_score = 0.0`, ended by `max_steps` after 1200 steps (about 3 min on one L40S).

## Versions, runtime and seeds

yam-isaac-lab-arena at commit 0059d02a1a91, inspect-robots 1.0.0, inspect-robots-yam 0.38.0, inspect-robots-dream
0.1.0 (checkpoint `dreamlabs/dream-1-beta`), isaaclab_arena 0.3.0, Isaac Lab 17.0.2, Isaac Sim 6.1.0.0. The layout is
fixed (`object_jitter=0`, `--seed 0`; nothing in the scene is jittered). Run limit: 1,200 control steps at 20 Hz
(60 s simulated); the demo succeeds at step 589 (29.5 s). On one L4 the demo ran at 6 to 7 control steps/s, about
4 minutes wall clock including about 2 minutes of Isaac Sim start-up; the cheat variants end 2 s after their scripts
(110 to 588 steps). The scripted demo is deterministic: the recorded run and the repeat ended in the same
state. A Dream-1 trial is slower, since Dream round trips pause the simulator.

## Materials

| Prop | Real item | Size | Mass | Colour | Position (world, m) |
|---|---|---|---|---|---|
| Base | steel bench fixture, bolted to the table | back block 22 x 90 x 112 mm, post 20 x 24 x 110 mm, foot 52 x 120 x 8 mm | 1.5 kg | grey | arm pivot A at (0.320, 0.000, 0.848) |
| Red pin | hardened steel pin in the base | 8 mm diameter, 24 mm proud | part of the base | red | axis along x at (y -0.053, z 0.848), x 0.308 to 0.284 |
| Upper arm | aluminium bar, held by its brake at 65 deg | 126 x 16 x 12 mm, 110 mm pivot to pivot | 80 g | yellow | from A up to C at (0.320, 0.046, 0.947) |
| Wrist plate | aluminium bar, balanced on pivot C at its middle | 122 x 20 x 10 mm, rear pin 53 mm from C | 66 g | blue | layer x 0.312 to 0.302; starts on end, rear pin D at (0.307, 0.046, 0.894) |
| Parallel link | aluminium bar with a snap eye | 110 mm pin to pin (135 mm overall), 12 x 8 mm section, pocket 10 x 14 mm | 40 g | light grey | layer x 0.299 to 0.291; hangs from D, eye near (0.295, 0.049, 0.784) |
| Snap tongue | sprung steel latch tongue | 16.5 x 8 x 8 mm, 45 deg lead-in, 10 mm travel, 40 N/m spring, 0.12 N preload | 3 g | black | in the eye |

Friction: pin, eye and tongue 0.15 static / 0.10 dynamic (greased steel); link 0.3 / 0.25; plate 0.6 / 0.5. Hinges are
frictionless. Every prop is a cut or turned aluminium or steel part at desktop-arm scale; desktop parallelogram arms
carry the same kind of rear link.

## Reset between trials

Nothing is consumed. Press the eye's tongue back with a finger and lift the link off the red pin. Let the link hang
from the plate: the plate turns under it until its rear end points straight down and the link hangs vertical (the
start pose). Check with a protractor that the arm still sits at 65 deg on its brake. The base is bolted on taped
outlines and needs no reset.

## Fidelity

Desktop-arm scale (110 mm links, close to a desktop parallelogram arm's own); a palletizer's level-keeping four-bar is
about ten times larger. The upper arm's brake is a stiff PD hold (200 N m/rad), the plate is balanced on its pivot, the
hinges are frictionless, and the snap eye is rigid contact geometry with a sprung sliding tongue standing in for a
spring clip. The links sit in separate layers, so they can pass each other and both closures are reachable. The eye's
1 mm play along the link leaves the plate 0.8 deg off level at rest.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
