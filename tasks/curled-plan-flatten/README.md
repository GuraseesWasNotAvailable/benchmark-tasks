# Lay a curled drawing flat on the plan table and weight both ends

The robot spreads a curled site plan flat on a plan table, centred on a taped outline, and pins both ends with two
steel paperweights, then lets go. The laminated plan was stored rolled and keeps its curl: laid on the table it rests
on its middle 15 cm with both ends rolled up and inward to about 8 cm. Cloth stays where it is laid; this sheet
springs back, and wherever it is not held it rolls up again. So the skill is working out where the anchors go and in
what order, not holding something down while the other hand works. A weight holds flat only the stretch of sheet
between it and another weight, so the free end beyond a weight curls up again: each weight has to finish on the
sheet's last 2.5 cm, and the middle has to stay pinned while the first half is opened. The natural wrong plans fail
on the physics, not on a stated rule: set on the rolled ends, the weights squash the curls but leave both end slats
curled up 20 deg; one weight in the middle leaves both halves rolled; weights stopped 5 cm in from the ends leave the
last slats curled. So each weight goes down where the sheet already lies flat, in the middle, and is walked out to
its end. Each end is in reach of only its own arm: the right arm's weight pins the middle while the left arm walks its
weight out over the left half to the left end, then the right arm walks its weight from the middle out to the right
end. Site supervisors and drafters do this every time they open a drawing on a plan table.

Instruction given to the policy (23 words; it states the goal and never says how to place the weights):

`Lay the rolled site plan flat on the plan table, centred on the grey outline, using the two steel weights, then let go.`

Files in this folder: `task.json`, `task.py` (scene, history record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena with yam-isaac-lab-arena, one L40S). The simulator results
below were run with inspect-robots 1.0.0, yam-isaac-lab-arena at commit 0059d02a1a91 and task.py SHA-256
e5c7ca81fddf (first 12 hex). Versions used for the submitted runs:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: e5c7ca81fddf (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in
  the log)

Before every shell: `source ~/IsaacLab-Arena/yam.env`.

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the plan as a PhysX articulation (16 slats joined by USD revolute joints with limits and
spring drives, spawned through Isaac Lab's `ArticulationCfg`) and calls the task's per-step history record after every
control step. No fork of yam_arena and no third-party assets: every body is built from box colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

37 tests feed hand-built slat and weight poses and gripper traces to the scorer. A slat 3.9 mm off its flat height is
flat and 4.1 mm is not; 5.9 deg tilt is flat and 6.1 deg is not; a weight covering 45 % of the end slat weights it and
43 % does not; 2.9 mm above the slat counts and 3.1 mm does not; one weight cannot weight both ends. The demo's end
state scores 1.0; flat-and-free must last 3 s and restarts from zero after a gripper touches a weight; a tool point
still for only 0.9 s, a moving plan, or a hand still on a weight each cap the score at 0.75; centred at 14 mm passes and
16 mm fails, axis 4.9 deg passes and 5.1 deg fails. A weight that falls off the station table scores 0 even after it is
put back; a weight set down on the station table for a regrasp, or lifted in the air, does not count as off. One half
flat out to its end slat is 0.25 (and stays earned after the sheet curls back); the same half with its end slat tilted
6.1 deg is 0; the squashed-curl outcome of the `weights_on_tube` cheat (14 flat middle slats, both end slats curled) is
0. One end weighted with its half flat is remembered as 0.5. The record is clean after a reset. Stillness is measured
from poses: 0.14 mm of movement in one 50 ms step (2.8 mm/s) is still and 0.16 mm is not, and with no previous step (the
first step after a reset) the speed is infinite, so never still.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the rolled site plan flat on the plan table, centred on the grey outline, using the two steel weights, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/curled-plan-flatten-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.
The same command with `--policy yam_scripted` checks the set-up without Dream: `demo()` is empty, so both arms hold the
ready pose until the scripted policy stops (our run: 61 steps, `termination_reason="policy_stop"`, task_score 0). The full-score demo is closed loop and runs through `linkdemo.py`.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                                  # full-score demo
python linkdemo.py task.py cheat_out --variant weights_on_tube       # a cheat variant (or DEMO_VARIANT=...)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the poses of the plan's slats and of the two bars, and the robot's own gripper state. It
never reads joint values or the history record.

1. Both arms pick their weight top-down, pads closing across the 2.5 cm bar.
2. The right arm carries its weight over the curl and sets it on the plan's flat middle, 3 cm right of the observed
   centre line, pressed 1.5 mm, and keeps holding it.
3. The left arm sets its weight 7.5 cm left of the centre line, pressed 1.5 mm, which lays the first rising slats
   flat, then slides it out at 25 mm/s to the left end slat's flat position. Pressed down, the bar's leading edge
   lays each curled slat flat in turn; the sheet behind it is pinned at both sides and cannot bulge. A closed loop on
   the end slat's observed pose nudges the bar onto the end slat's centre. The script keeps the left weight held
   down while the right half is opened, although the bar holds that end on its own (`release_left_early` below).
4. The right arm slides its weight from the middle out to the right end slat the same way.
5. Both grippers open, rise 10 cm and park clear; the arms wait.

Result: the left half lies flat out to its end at 16.5 s and its end is weighted at 16.8 s; the right weight reaches the
right end at about 24.5 s; both grippers let go at 26 to 28 s. At the end all 16 slats are flat (largest offset 0.5 mm,
largest tilt 2.6 deg), each weight covers 88 % of its end slat (a 16 cm bar can cover at most 89 % of an 18 cm slat)
with its underside 1.2 to 1.4 mm from the slat's top, the plan's centre is 3.8 mm from the outline centre and its axis
is on y. Both grippers park more than 140 mm clear. Success holds for 1 s and ends the episode at step 676 (33.8 s
simulated, 5 to 14 min wall clock with start-up on an L40S shared with other jobs); task_score 1.0. Worst IK error
0.09 mm and 0.01 deg. A reset afterwards (`--check-reset`) puts both weights back within 0.2 mm of their marks and the
plan's middle slat within 0.3 mm of its start, and the record restarts from zero.

Cheat variants, each run in the simulator for the full 1,200-step run limit with the files in this folder, through the
same closed loop as `python linkdemo.py task.py out --variant <name>` (up to three variants per simulator session,
with a reset between them, to save start-up time):

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose for all 1,200 steps. | The plan keeps its curl: never more than 6 flat slats, both ends rolled up. | 0.0 |
| `weights_on_tube` | Each arm sets its weight on the crest of its half's curl and lets go. | The 470 g weights squash both curls, so 14 middle slats lie flat, but both end slats stay curled up 20 deg and 4 mm off the table, and the weights end about 3.5 cm short of the end slats (0 % cover). No half is ever flat out to its end. | 0.0 |
| `middle_weight_only` | The right arm sets one weight on the flat middle and lets go; the left arm does nothing. | Both halves stay rolled up: never more than 6 flat slats. | 0.0 |
| `five_cm_in` | The full two-arm demo, but each weight stops 5 cm in from its end and is let go there. | The middle 12 slats lie flat between the weights; beyond each weight the last two slats curl back up (end slats 38 deg and 16 mm off the table). No half is ever flat out to its end. | 0.0 |

Other plans, for reference, run in the simulator with the same files:

- `hold` runs the full demo but never lets go. It ends with the whole plan flat and both ends weighted (77 and 86 %
  cover) with both grippers still on their weights, so it stops at task_score 0.75 (664 steps).
- `release_left_early` lets go of the left weight as soon as it sits on the left end, before the right arm walks out.
  The bar holds the left end down on its own and the run scores 1.0 (success at step 621): the scorer asks for the
  finished state, not for a held anchor.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`H` in `task.py`, a linkkit `History`) reads the poses of the 16
slats and both weights and each gripper's pads and tool point. It is cleared only by the simulator reset, so putting a
weight back never erases a violation.

- A slat is flat when its centre is within 4 mm of its flat height (plan-table top plus half the 1.5 mm thickness) and
  it is tilted less than 6 deg.
- An end is weighted when a weight's 16 x 2.5 cm footprint covers at least 44 % of that end's last 18 x 2.5 cm slat
  (a bar square to the sheet does so with half its width on that slat; a 16 cm bar can cover at most 89 %) and the
  weight's underside is within 3 mm of that slat's top face. The last 2.5 cm is the drawing's border margin, where a
  drafter sets a weight so it hides no linework, and a weight has to be at least partly on it to hold the end down.
- Centred: the mean of the slat centres within 15 mm of the outline centre and the end-to-end axis within 5 deg of y.
- Gate, checked first, gives 0 for the run: a weight's lowest point more than 5 mm below the station-table top at
  any time (a 470 g steel bar knocked off the bench, a hazard to feet and floor). Setting a weight down on the
  station table, for example to regrasp it, is allowed.
- 0.25: at any time one half (8 slats) flat out to its end slat.
- 0.5: at any time one end weighted with its whole half flat.
- 0.75: at the end, the whole plan flat with both ends weighted (hands anywhere).
- 1.0: and the plan centred; for the last 3 s the plan flat with both ends weighted, every sampled pad point of both
  grippers at least 30 mm from the plan and both weights, and every slat and weight slower than 3 mm/s; and for the
  last 1 s both grippers 30 mm clear with their tool points slower than 5 mm/s (released).

Speeds of the slats and weights are measured from their poses at the 20 Hz control rate (displacement per control
step), as a camera would see them. The thin slats chatter in contact at sub-millimetre amplitude, a solver artefact
that instantaneous body velocities report as 20 to 60 mm/s on a sheet that does not move. The same chatter makes the
curled plan creep along y at about 0.3 mm/s while nothing holds it (16 mm over a 60 s run with no robot action); once a
weight is on it, it stays put (the demo ends 3.8 mm from the outline centre).

`score()` returns the rung above, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: a top photo against the tape outline, a feeler
gauge under each 2.5 cm line of the sheet, and a pencil mark at each weight's end position on the last 2.5 cm.

Change from the design spec: the first rung was "10 slats flat in a row". In the simulator the `weights_on_tube` cheat
squashed both curls and flattened 14 middle slats while leaving both end slats curled, which that rule scored 0.25; the
rung now asks for a half flat out to its end slat, which only opening the sheet earns.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo finishes in 676.
- Runtime on one L40S shared with other jobs (0.8 to 2 control steps per second, by load): the demo 5 to 14 min wall
  clock including Isaac Sim start-up, a full 1,200-step cheat run 10 to 25 min; a Dream-1 trial 6 min for 1200
  steps (Dream round trips pause the simulator). The first run after boot adds a few minutes of warm-up.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (the plan is never jittered). The
  scripted demo is deterministic (every run gave the same 676 steps and the same end state); Dream-1 is a hosted model
  without a seed.

## Submitted videos and scores

- Dream-1 run: log `adhoc_b0441a4f.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 1200 steps.
- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`, `task_score = 1.0`, ended by `success` after
  676 steps. The video is `demo_out/top_cam.mp4` and the screenshot is the top camera 0.5 s into the demo, before the arms move (`ffmpeg -ss 0.5 -i demo_out/top_cam.mp4 -frames:v 1 setup.png`).

```bash
inspect-robots inspect adhoc_b0441a4f.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_b0441a4f.json
```

## Materials

- Plan table: a 24 x 52 x 1.8 cm white melamine board, fixed to the table, with a 5 mm grey tape outline of the
  18 x 40 cm sheet.
- Site plan: a laminated drawing 18 x 40 cm (paper in a 2 x 125 micron laminating pouch, about 0.3 mm thick, about
  27 g), white with blue linework, stored rolled so it keeps a curl of about 7 cm radius.
- Two steel paperweights (drafting weights): 16 x 2.5 x 1.5 cm steel bars, 470 g each, dark grey, with a 2 mm green
  felt base.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Roll the plan back up for a few seconds and lay it on the plan table centred on the outline, ends
up; it relaxes to its curl. Put both weights back on their marks at (18, +23.5) and (18, -23.5) cm (x forward, y left
of the midpoint between the arm bases) with the long side along x. The plan table is fixed and needs no reset.

## Licence

MIT; the licence text and SPDX identifiers are in the file headers. Our own code, written with public Isaac Lab and
USD Physics techniques; no third-party assets, and no code copied from other tasks in this repository.
