# Connect a garden hose to its tap with a quick-connect coupler and let it lock

The robot connects a garden hose to an outdoor tap through a ball-lock quick-connect coupler. One hand pulls the
coupler's sprung collar back and holds it there while the other lifts the hose's brass male fitting, lines it up with
the bore and pushes it in until its flange meets the coupler; the collar is then released, springs forward over the
locking pawl, and the fitting is held. At the end the collar is forward, the nipple is seated and locked, and both
grippers have let go. Everyone who waters a garden, washes a car or fills a paddling pool does this.

The ability is the engage direction of a sprung-collar coupling: hold a spring-loaded sleeve back with one hand for the
whole length of a push made by the other, then release it at the right moment so it locks. Nothing is extracted,
released or unplugged, and nothing is placed on a target: the collar's land holds the pawl down, so the nipple's nose
cannot pass the pawl until the collar is back 6 mm or more. The same fitting pushed against a closed collar stops at
the pawl; the collar pulled back and let go before the nipple is in leaves the pawl riding on the shaft, and the collar
cannot return forward.

Physics, not the instruction, defeats the natural plans. A hose end lies on the table with its hose anchored at the
other end, so it has to be lifted and carried in line with the bore (the hose resists being dragged straight at the
coupler from the side). The pawl and the collar's land hold the nipple out until the collar is held back; both
hands are busy at once.

Instruction given to the policy (20 words; it states the goal only, never the technique or the collar):

`Connect the garden hose to the tap's quick-connect coupler so that it locks on, then let go of both parts.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo
and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: 969c6e6c4e2b (`sha256sum task.py | cut -c1-12`)
- for the Dream-1 trial: inspect-robots-dream 1.0.0, yam-arena 0059d02a1a91, task file 969c6e6c4e2b (equals
  `eval.environment_revision` in the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the coupler (a fixed body with a sprung sliding collar and a sliding pawl) and the hose with
its nipple (a chain of revolute links) as PhysX articulations, built through Isaac Lab's `ArticulationCfg` with USD
joints, limits and drives, and calls the task's per-step record after every control step. No fork of yam_arena and no
third-party assets: every body is built from box, cylinder and capsule colliders in `task.py`. `linkkit.py` and
`linkdemo.py` are our own code under the MIT licence.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

21 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; doing nothing, pushing against the
closed collar and pulling the collar with no nipple in all score 0; collar back 7.0 mm counts and 6.9 does not; nose 10
mm in counts and 9.9 does not; seated with the collar still held back is 0.5 and with the collar forward but the hands
on is 0.75; flange 3.0 mm from the mouth is seated and 3.1 is not; tilt 12.0 deg vs 12.1; offset 4.0 vs 4.1 mm; collar
1.0 vs 1.1 mm; a pad point 29 mm clear restarts the release timer and 30 mm does not; a hand at 4.9 vs 6.0 mm/s; the
fitting at 2.9 vs 3.1 mm/s; 0.95 s of release is not enough and 1.05 s is; seating once and then pulling the nipple out
again keeps the best rung at 0.5; the record is clean after a reset; measure() on stand-in link states gives the right
collar travel, flange offset, nose depth, tilt and sideways offset, and the plug's speed from its pose change between
control steps.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Connect the garden hose to the tap's quick-connect coupler so that it locks on, then let go of both parts." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/hose-quick-connect-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant no_collar      # a cheat variant (or DEMO_VARIANT=no_collar)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the coupler body, collar and fitting poses and the robot's own grippers. It never reads
joint values, the pawl or the record.

1. Right arm: top-down on the fitting's boot (pads on its two flats, 10 mm squeeze), lifts it 45 mm, backs it off
   along the bore line level with the staging pose, then slides it sideways to the bore axis, 40 mm in front of the
   mouth, with the nipple pointing along the bore; it nulls any slip from the seen fitting pose.
2. Left arm, once the fitting is staged: pads over the collar's finger rails, just in front of their thumb ridges;
   pulls the collar back 8.5 mm, closed loop on the seen collar position (the ridges give a positive stop for the pads).
3. Right arm, once the collar is seen back 7 mm: pushes the fitting in 3 mm at a time, correcting its sideways and height
   error from the seen pose at every step, until the flange is seen at the mouth. Its nose cams the pawl up on the way.
4. Left arm lets go: the collar springs forward over the pawl, which has dropped into the groove. Right arm lets go and
   backs away; left arm backs away; both wait.

The demo scores 1.0 (`success`, episode ended at step 817 of 1,200): the collar peaked at 10.0 mm, the pawl rode up to 4.5
mm on the shaft, and at the end the flange is 2 to 3 mm from the mouth with the pawl in the groove, tilt 9 deg, offset
2.5 mm, collar 0.1 mm, both grippers 107 and 172 mm clear. Worst IK error 0.1 mm and 0.01 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (969c6e6c4e2b); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. | 0.0 |
| `no_collar` | The right arm brings the fitting to the mouth and pushes it in, the collar left forward. | The nose stops on the pawl 6 mm in; the collar was never back, so no rung is earned. | 0.0 |
| `collar_only` | Left arm pulls the collar back 8.5 mm and lets go; the fitting is staged and never pushed in. | The collar springs forward; the nipple never enters. | 0.0 |
| `drop_at_mouth` | Right arm stages the fitting with its nose 8 mm in front of the mouth, collar forward, and lets go. | The fitting drops on the table at the mouth and never goes in. | 0.0 |

## Build log (44 runs, short ones to find the pull and the push, 3 to 6 min each on the shared L40S)

| Stage | Change | Result |
|---|---|---|
| 1 | first scene (hose 0.28 m, plug 36 mm body, collar pulled by squeezing its sides) | 0.0: the hose, anchored to one side, was too short to bring the fitting in line with the bore |
| 2 | plug boot 62 mm, hose clamp behind the coupler, hose 0.20 m straight | staged the fitting to 0.5 mm; the hands no longer touched |
| 3 | finger rails and thumb ridges on the collar | the squeezed sleeve had jammed on the body; the ridges give a positive stop |
| 4 | collar spring 800 to 300 N/m | the arm could pull 4 N but not 11 N at this reach: collar 10 mm back |
| 5 | bore 18 to 20 mm; the pawl 5 mm above the axis, 1.5 mm under the land; the nose rounded | the flat nose could not cam the pawl up; with the round nose the nipple slides in |
| 6 | push in 3 mm steps with sideways correction; seat window 3 mm / 12 deg / 4 mm (the pawl has 1.25 mm of slack in the groove, the hose tips the fitting) | 1.0 at 1,148 steps |
| 7 | shorter waits | 1.0 at 817 steps, `success` true |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures, in the coupler
frame: the collar's travel from the seen link states, the nipple's flange offset f along the bore (mm from the mouth,
negative in front), nose depth (f + 28 mm), the fitting's tilt from the bore axis and sideways offset, its speed (from
its pose change between control steps, what a camera sees), and each gripper's pad clearance to the coupler and the
fitting and its speed. The record keeps flags, peaks and held times for the whole episode and is cleared only by the
simulator reset, so unplugging and starting again never clears it.

- Seated: flange within 3.0 mm of the mouth (-3.0 to +1.5 mm), tilt at most 12 deg, offset at most 4 mm.
- 0.25: the nose went at least 10 mm in while the collar was held back at least 7 mm.
- 0.5: after that, the nipple was seated at some time.
- 0.75: at the end, seated with the collar forward (at most 1 mm back). With the pawl riding on the shaft the collar
  cannot come forward, so this means the pawl is in the groove.
- 1.0: that, with the fitting still (under 3 mm/s) and the release gate (every pad point of both grippers at least 30 mm
  from the coupler, collar and fitting, both grippers moving under 5 mm/s), together for 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: a steel rule fixed along the bore on the table
(1 mm marks from -30 to +3 mm) and a scribed 7 mm line on the body, read in the top video at 20 Hz for collar travel,
flange offset and nose depth; a try-square for tilt; the top and side video for the 1 s hold, stillness and 30 mm
clearance. A 10 N spring-scale pull along the bore afterwards confirms the lock (under 1 mm of movement); it is a check
of the build and does not enter the score.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 817.
- Runtime on one L40S: a demo run took about 4 min wall clock on the shared VM, plus about 2 min of Isaac Sim start-up.
  A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout. The scripted demo is deterministic
  (runs 42 to 44 ended in the same state to 0.1 mm).

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 817. The video
  is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png` (frame 0
  of the top camera).
- Dream-1 run: score 0.0, ended with max_steps after 1200 steps, about 3 minutes wall clock, log adhoc_0d9974f1.json, on this
  `task.py` (969c6e6c4e2b).

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials

- Tap-side coupler: brass, a 30 x 30 x 50 mm square body with a 20 x 20 mm bore 40 mm deep, mouth at world (0.150, 0),
  axis 30 mm above the table, on a tap stand with a 22 mm spigot (0.5 kg, fixed, friction 0.3).
- Locking pawl: steel, 3.5 x 10 x 10 mm, 5 g, sliding in a 4.5 mm slot 8 mm behind the mouth; its tip stands 4 mm into the
  bore; a 100 N/m spring holds it down; 6 mm of travel.
- Sprung collar: black anodised aluminium, 44 x 24 x 42 mm, 40 g, friction 1.0 / 0.9, 3 mm clear of the body, 10 mm of
  travel on a 300 N/m spring (1.2 N at rest, 4.2 N at the stop); a 3.5 mm roof whose land sits 1.5 mm over the pawl,
  open in front of the land; two finger rails 52 mm over both and 3 mm thumb ridges at the rear.
- Hose: green PVC, 25 mm outer diameter, 5 links of 40 mm (12 g each), alternating vertical and horizontal hinges with
  0.003 N m/rad springs, anchored to the table by a 30 x 50 x 25 mm clamp post at (-0.100, 0).
- Male nipple fitting: brass, 120 g, friction 0.4 / 0.3: a 15.5 mm shaft 28 mm long with a rounded nose, a 6 mm groove
  (11 mm core) 5 to 11 mm behind the flange, a 26 x 26 x 8 mm flange and a 24 x 24 x 62 mm crimp body with
  strain-relief boot.
- Layout and physics chosen for reach and feasibility: the coupler is 0.15 m from the arm bases so that the left arm
  can pull the collar and the right arm can bring the fitting in line; the collar spring is light (4.2 N at full stroke)
  because the arm could not pull a stiffer one at this reach; the hose is anchored behind the coupler so the fitting can
  be lifted into line with the bore.
- How to build and verify it: the collar's spring rate by spring scale on the rails (1.2 N at the start, 4.2 N at
  10 mm); the pawl's intrusion by depth gauge (4 mm); the land clearance (1.5 mm) and sleeve clearance (3 mm) by feeler
  gauge; the groove (6 mm wide, 11 mm core) and shaft (15.5 mm) by calliper; the lock by a 10 N pull after the demo.
- Fidelity limits: one steel pawl stands in for the balls of a ball-lock; the bore is square around a round shaft; the
  hose is 5 stiff links 0.20 m long; the collar spring is light.

## Licence

`task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py`: MIT, Copyright (c) 2026 the task authors.
