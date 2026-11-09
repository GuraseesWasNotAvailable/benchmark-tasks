# Open a manikin's airway with an oral airway without pushing the tongue back

The robot inserts a curved tool upside down and turns it at depth, so that its tip sweeps under a sprung soft obstacle
and lifts it instead of pushing it back. The tool is a green oropharyngeal (Guedel) airway and the obstacle is the
tongue base of an airway-trainer manikin. At the end the airway's flange rests at the lips, its curved end lies behind
the tongue base and holds it clear of the back of the throat, the jaw is closed on its bite block, and both grippers
have let go. EMTs, paramedics and respiratory therapists do this for every unconscious adult whose tongue blocks the
airway.

The ability is the turn at depth as the way to move a passive sprung body. The turn is not a lock or a fastening (no
bayonet, no T-nut, nothing latches) and nothing is placed: the 180 deg sweep is what lifts the tongue, and the same
airway pushed in the natural way, or turned too early, drives the tongue back instead. Holding the jaw open is only the
enabling role for the second arm.

Physics, not the instruction, defeats the natural plan. The airway waits in its kit tray the way it finally sits
(curve toward the chin). Pushed in like that, its tip lands on the top of the sprung tongue base and drives it back
toward the throat wall, which gates the run to 0 for good. The adult technique is to go in upside down (curve toward
the palate, clear of the tongue), turn it 180 deg at depth so the tip sweeps under the tongue base's lower ramp and
lifts it, and then seat it. The jaw is sprung shut (no gap between the lips), so one arm holds the chin open while the
other inserts.

Instruction given to the policy (19 words; it states the goal only, never the technique or the tongue rule):

`Open the manikin's airway with the green oral airway so its flange rests at the lips, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the
demo and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: 7f12487c9f0e (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision`
  in the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the head as a PhysX articulation (a fixed skull with a sprung revolute jaw and a sprung
revolute tongue base, USD joints with limits and drives, spawned through Isaac Lab's `ArticulationCfg`), the airway and
the tray as compound rigid bodies, and calls the task's per-step record after every control step. No fork of yam_arena
and no third-party assets: every body is built from box and capsule colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

22 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; q_t 2.99 deg is no gate but 3.00 deg
is, even when the tongue is lifted and the airway seated afterwards, and the gate beats an open airway at the end; alpha
25 deg is open and 26 is not; gap 8.0 mm is open and 7.9 is not; the seated limits (4.0 vs 4.1 mm high, 8.0 vs 8.1 mm
off centre, 15.0 vs 15.1 deg tilt); tongue lifted 15.0 vs 14.9 deg and the tip 5.0 vs 4.9 mm past its front edge; the
jaw held open at 28 deg scores 0.75 and 12 deg scores 1.0; one pad step 29 mm from the head restarts the release timer
(0.75) and 30 mm does not; a hand at 6 mm/s or an airway at 4 mm/s near the end scores 0.75; 0.95 s of release is not
enough; the 0.25 rung needs a held airway (a dropped one earns nothing), an open jaw and a tip 35 mm deep, with any
technique (upside down, sideways or the right way up), and the right way up followed by a push-back is gated; lifted
needs the airway in the mouth (the tongue lifted while the airway lies on the table earns nothing); the record is clean
after a reset; stillness comes from the airway's pose change between control steps (0.2 mm or 0.2 deg in one 50 ms
step is too fast); measure() on real link states (from the head's kinematics) gives a 2.0 mm gap at rest, 0 mm at the
+6.7 deg wall stop, about 10.7 mm at -25 deg, and the right alpha, tip and held flag; and the seated airway's curve
touches the throat wall.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the manikin's airway with the green oral airway so its flange rests at the lips, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/guedel-airway-insert-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant as_presented  # a cheat variant (or DEMO_VARIANT=as_presented)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the skull, jaw and airway poses, the jaw angle computed from the skull and jaw poses, and
the robot's own grippers. It never reads joint values, the tongue (hidden in the mouth) or the record.

1. Both arms together. Left: pinches the chin plate (fingers opened only 66 mm on the approach, pads closing along the
   jaw hinge, body leaning 20 deg toward the chin), pivots the jaw open 28 deg (wrist following half the angle) and
   corrects from the seen angle; it holds there. Right: top-down on the airway's flange in the tray, pads on the
   flange's two ends (12 mm squeeze), lifts it, records the grasp from the seen airway pose, and carries it over the
   face at 96 mm while turning it upside down in hand (curve toward the head top) and tilting it 12 deg.
2. Right, once the jaw is seen open 26 deg or more: threads the upside-down airway through the mouth along an
   offline-checked window (at least 2.9 mm from teeth and lips, 7 mm from the tongue), straightening to upright
   3.5 mm above the lips, and slides to u = -8 with the bite block 1 mm off the upper teeth. Servo checks in free air
   null the arm's sag (about 1 mm).
3. Right: turns the airway 180 deg about its bite block at 20 deg/s, 2.5 mm above the lips (its tip 1.5 mm above the
   throat wall), the axis sliding 3 mm caudally mid-turn so the bite block keeps 1 mm off the upper teeth; over the
   last 60 deg, once its tip is under the tongue's ramp, it rises 2 mm. Every 10 deg it corrects the bite block's tilt
   from the seen airway pose (position correction capped at 3 mm).
4. Right presses the flange down onto the lips (commanded 2 mm below them), which seats the end of the curve on the
   throat wall under the lifted tongue; left eases the jaw shut until it stops on the bite block (seen angle), lets go
   and backs away; right lets go and backs away; both wait.

The demo scores 1.0 (`success`, episode ended at step 842 of 1,200): the tongue was never pushed back (peak q_t
0.0 deg), the turn lifted it to -36.1 deg, and at the end it rests at -22.5 deg with a 9.8 mm gap, the airway seated
(flange 0.2 mm above the lips, tilt 0.5 deg, curve 3.1 deg from the chin), the jaw closed to 9.9 deg on the bite
block, and both grippers 53 and 95 mm clear. Worst IK error 0.09 mm and 0.02 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (7f12487c9f0e); every one scores
exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; the airway stays in its tray. | 0.0 |
| `as_presented` | Curve toward the chin as in the tray: held swung 30 deg back about its curve's centre, lowered tip first into the open mouth, then swung home along its curve. | Its tip is 35 mm deep at 12.6 s, lands on the top of the tongue base and drives it onto the throat wall: gate at 12.9 s (peak 6.7 deg). | 0.0 |
| `straight_down` | As presented, lowered straight down into the open mouth at the seat. | The curve reaches 38 mm toward the chin and cannot pass the 33 mm mouth straight down: its tip lands on the lower teeth 10 mm below the lips, forces the jaw to its 35 deg stop and twists in the grasp; the airway is dropped across the open jaw, never 35 mm deep. | 0.0 |
| `one_arm` | No jaw hold: the right arm drives the upside-down airway along the demo's path into the shut mouth, its curve and bite block pressing on the lower lip and chin as a lever. | The curve presses the shut lips and chin and levers the jaw open (29 deg at most), but the airway twists in the grasp (tilt up to 83 deg) and is never 35 mm deep while held; it falls out at 31 s and hits the tongue: gate (6.7 deg). | 0.0 |
| `turn_early` | Upside down, turned 20 mm above the lips, then pressed home. | Enters upside down (0.25 at 15.2 s); turned above the lips, its tip ends over the tongue, and pressing home drives the tongue onto the throat wall: gate at 32.0 s (6.7 deg). | 0.0 |
| `drop_over_mouth` | Released upside down 30 mm above the open mouth. | Falls on its curve, tips over and comes to rest across the lower lip and chin, never in the mouth while held. | 0.0 |

On one arm levering the jaw with the airway: the jaw spring holds 0.21 N m at 28 deg (about 3 N at the chin), so the
airway itself can push the jaw open (`one_arm` reached 29 deg), but the same airway then has to be threaded and
turned at depth while it props the jaw, and in the simulator that attempt twisted it out of the grasp. We did not
stiffen the jaw: the left arm's pinch-and-pivot needs a light hinge.

## Build log (13 demo runs, 12 to 25 min each on the shared L40S)

| Run | Change | Result |
|---|---|---|
| 1 | first build (head yaw 52 deg, pads across the flange's 22 mm) | 0.0: linkdemo's pinch opens the fingers 94 mm, they landed on the cheek walls and the jaw never opened |
| 2 | 66 mm finger opening; cheek walls end at the lip corners | 0.0: the jaw opened and the airway entered upside down; mid-turn the sideways load at the tip twisted it about the pads' closing axis and the tip rode onto the tongue: gate (3.4 deg) |
| 3 | tongue 0.05 N m/rad and wet, 8 mm squeeze, turn 1.5 mm above the lips | 0.5: tongue lifted to -18.5 deg, then the twisted flange caught a lip corner and the turn stopped at 42 deg |
| 4 | pads on the flange's ends; head yaw 142 deg and lips 80 mm above the table (wrist roll and joint-4 reach) | 0.5: turn completed, jaw closed to 9.6 deg; the tongue's push pitched the airway in the grasp and its tip stalled on the throat wall short of the tongue base |
| 5 | tongue 0.025 N m/rad, 12 mm squeeze, tilt-only correction every 10 deg | 0.5: same pitch slip |
| 6 | 2 mm late rise, wet inner surfaces | 0.5: open airway at the end of the turn (tongue -24.5 deg), lost when it was lowered to seat: with its flange off the lips and its tip 1 mm off the wall, nothing held it level |
| 7 | press the flange onto the lips after zeroing the tilt correction | 0.5: zeroing the correction swung the airway to alpha 23 deg |
| 8 | pads 1.5 mm higher on the flange | 0.25: covering only 4 mm of the flange, the grip lost the airway |
| 9 | bite block 30 to 31 mm (a seated airway now rests on the lips and on the throat wall), 20 mm squeeze, bite block 0.4 mm off the teeth, press to seat | 0.5: the airway slid down 3 mm in the grip mid-turn and the hand's position correction climbed off it |
| 10 | back to the 12 mm squeeze, teeth 1 mm, position correction capped at 3 mm | 0.75: airway open and seated, jaw 9.9 deg, hands clear; "airway steady" failed only because the solver's velocity buffer read 0.76 rad/s for an airway whose pose did not change |
| 11 | stillness judged from the airway's pose change between control steps | 1.0 |
| 12 | the `as_presented` cheat made reachable (its first 60 deg swing was out of the arm's reach) | 1.0, the same end state as runs 10 and 11 |
| 13 | final task file (licence header only) | 1.0, the same end state |

The geometry change in run 9 is the one that matters: with the bite block 31 mm long, the seated airway is held by the
lips under its flange, the upper teeth at its bite block and the throat wall under the end of its curve, so the
tongue's push cannot pitch it once the hand lets go, as a real airway rests against the posterior pharyngeal wall.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures, in the head
frame (mm from the lip centre; u toward the chin, v to the patient's left, w up): the tongue angle q_t (deg, + pushed
back toward the throat, - lifted), the tongue base's gap (the lower of its front edge and its lowest corner above the
throat wall; 2.0 mm at rest), the jaw opening q_j, the airway's flange-bottom centre F, its tilt (bite block from
vertical), its azimuth alpha (where the curve points; 0 = toward the chin, 180 = upside down), its tip (the end of the
curve's centreline), whether a gripper holds it (opening at most 60 mm with a pad within 4 mm of it), its speed and
spin (from its pose change between control steps, what a camera sees), and each gripper's pad clearance to the head
and airway and its speed. The record keeps flags, peaks and held times for the whole episode and is cleared only by the
simulator reset, so pulling the airway out and starting again never clears a push-back.

- Gate, any time in the run, gives 0: q_t reached +3.0 deg (the tongue base's 2 mm rest gap to the throat wall closed
  to 0.7 mm; the wall itself is the joint's +6.7 deg stop). Any push-back means the tip rode on top of the tongue, the
  failure the technique exists to prevent. 3 deg sits clear of contact jitter (the demo's peak is 0.0 deg) and reads
  on the tongue-hinge protractor and through the clear cheek panel.
- 0.25: the held airway's tip went at least 35 mm below the lips inside the cavity while the jaw was open at least
  10 deg, with any technique.
- 0.5: with the airway's tip in the mouth, the tongue base was lifted 10 deg at some time.
- 0.75: open airway at the end: seated (flange-bottom centre at most 4 mm above the lips, within 8 mm of u = -9 and of
  v = 0, tilt at most 15 deg), curve within 25 deg of the chin, tongue lifted at least 15 deg, its gap at least 8 mm,
  and the tip at least 5 mm past the tongue's front edge.
- 1.0: open airway at the end, the jaw closed onto the bite block (q_j at most 12 deg), the airway still (under 3 mm/s
  and 3 deg/s) for the last 2 s, and the release gate: every pad point of both grippers at least 30 mm from the head,
  jaw, tongue and airway and both grippers moving under 5 mm/s for the last 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: a top photo for the flange at the lips and its
direction; a side view through the trainer's clear cheek panel with a mm scale for the tongue base's gap to the throat
wall (and a video frame for any push-back); a protractor on the jaw and tongue hinges; the video for stillness.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 842.
- Runtime on one L40S: a demo run took about 12 min wall clock on the shared VM (1.0 to 1.4 control steps/s with
  other simulations running, plus about 2 min of Isaac Sim start-up). The first run after boot adds a few minutes of
  warm-up. A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (the head and tray are fixed and
  the airway is never jittered). The scripted demo is deterministic (runs 10 to 13 ended in the same state to
  0.01 mm); Dream-1 is a hosted model without a seed.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 842. The
  video is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png`
  (frame 0 of the top camera).
- Dream-1 run: made at filing with the command above, on this exact `task.py` (7f12487c9f0e), after a 40-step probe of
  the instruction (`--max-steps 40`, no frames) to rule out a refusal.

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials

- Airway-trainer head: an adult-length trainer whose mouth cavity and tongue are about 1.6x adult width, skin-tone
  vinyl over rigid foam, 2.0 kg, about 200 x 150 mm and 80 mm from the table to the lips, face up and fixed to the
  table (lips centred at (0.170, -0.030), chin pointing toward the robot's left arm, along world yaw 142 deg). Its
  mouth cavity is 94 x 80 mm and 52 mm deep under the lips, with the back wall of the throat 64 mm below the lips.
  Inside surfaces are wet (friction 0.3 / 0.25).
- Layout and physics chosen for reach and feasibility: the head yaw is 142 deg so the right wrist's 180 deg turn fits
  its +-120 deg roll; the lips are 80 mm above the table (about 180 mm in life) to keep joint 4 inside its limit above
  the mouth; the cavity and tongue are about 1.6x adult width so a rigid airway can turn where tissue would yield; the
  tongue spring is light (0.025 N m/rad) and the inner surfaces wet, to keep the tongue's load inside what the grip can
  hold.
- Lower jaw: a hinged vinyl mandible shell (0.10 kg; a 20 x 43 x 10 mm lip and teeth bar and a 24 x 50 x 10 mm chin
  plate) on a horizontal hinge 72 mm below the lips, held shut by a 0.3 N m/rad torsion spring (0.04 N m net preload,
  no gap between the lips); it opens to 35 deg (28 deg gives a 33 mm mouth).
- Tongue base: a pink silicone insert, 25 g, 72 mm wide and 40 x 19 mm in profile with a 31 deg ramp on its
  front-lower face, on a horizontal hinge 50 mm below the lips. A 0.025 N m/rad spring (its rest offset cancels the
  insert's weight) holds its base 2 mm above the throat wall; 6.7 deg back it touches the wall; it lifts up to 42 deg.
  Wet silicone (friction 0.15 / 0.10).
- Oral airway: a Guedel-pattern oropharyngeal airway, size 2 (80 mm), green PVC, 12 g: a 56 x 22 x 7 mm flange, a
  31 mm bite block of 16 x 10 mm stadium section and a 90 deg curve of 28 mm centreline radius, 16 x 10 mm in section,
  82 mm from the flange top to the tip along its centreline. Seated with its flange on the lips, the end of its curve
  rests on the back wall of the throat. Friction 0.4 / 0.3.
- Airway kit tray: grey ABS, 0.3 kg, fixed: a 74 x 46 x 80 mm block with a 60 x 32 mm pocket 72 mm deep; the airway
  hangs from the pocket's long rims by its flange.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Take the airway out and hang it flange-up in the tray pocket with its curve pointing toward the
head's chin side (world yaw 142 deg). Let the jaw spring shut the mouth (0 deg, lips touching) and check the tongue
insert rests with its base 2 mm above the throat wall (a 2 mm feeler gauge through the clear cheek panel). Re-wet the
inner surfaces with the trainer's lubricant spray. The head and the tray are fixed on taped outlines and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
