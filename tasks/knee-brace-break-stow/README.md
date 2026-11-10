# Unlock an over-centre strut by its knee to stow a radiator panel

The robot unlocks a two-link strut that is locked straight over centre by pushing its knee pin across centre, square
to the strut, because every push on the panel the strut holds only drives the knee harder into its stop. Then it
carries the freed panel down onto its rest pads. The panel is a grey honeycomb radiator panel on a stowage fixture,
held open at 75 deg by a yellow deployment strut whose knee sits 3 deg past straight against a stop. At the end the
panel rests on its two black pads and both grippers have let go.

Aerospace technicians (O*NET 17-3021) re-stow deployable radiator and antenna panels by hand on their ground fixtures
after deployment tests, and aircraft mechanics (49-3011) break a landing-gear downlock brace with the same move. The
same over-centre lock holds a folding table's leg brace, a stepladder spreader and a tailgate stay: none of them gives
way to a push on what it holds up, and each folds once its knee is pushed through. A honeycomb panel dropped onto its
pads is damaged and has to be re-inspected, so the panel is carried down, not dropped. This is a 1 g tabletop
analogue of a metre-scale part on a stowage fixture; it makes no microgravity claim.

Physics, not the instruction, defeats the natural plan. Pushing or folding the panel down compresses the strut, which
drives its knee harder into the stop, so the panel does not move: closed fingers driven 30 mm past contact into its
face for 3 s deflect it 0.06 deg (the `press_face` cheat below). Lifting the panel only straightens the strut, and the
links' own weight drops the knee back onto its stop. Pushing the knee pin up and back across centre unlocks the strut,
and from then on nothing holds the panel: let go, it falls 75 deg and hits the pads at about 650 deg/s. So one hand
holds the panel by its free-edge tab while the other breaks the knee and keeps it broken until the panel has started
down (with the panel held, the strut's own weight relocks the knee if it is let go too early), and the holding hand
then carries the panel down.

What is new, and what is not. The ability is breaking an over-centre lock at its knee: the lock is pure geometry,
with no pawl, lever, button or latch to release, and no force on the strut's ends (the panel, the lug, the base)
unlocks it; only a push on the knee perpendicular to the strut does. That separates it from a released hold such as
TK-BUD2WM, where a crank takes the load off a ratchet pawl and the pawl is then lifted: here taking the load off
(lifting the panel) unlocks nothing. Nor is the lowering the new part: a closing cover such as TK-9JFBH4 is free to
move from the start, while this panel cannot be moved at all until the knee is broken, and from that instant only the
hand holds it, so the hand must already be on the panel when the knee goes.

Instruction given to the policy (14 words; it states the goal only, never the strut or the method):

`Fold the grey radiator panel down onto its black rest pads, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one GPU). Versions used for the demo
and cheat runs below: inspect-robots 1.0.0, inspect-robots-yam 0.38.0, inspect-robots-dream 0.1.0, isaaclab_arena
0.3.0, Isaac Lab 17.0.2, Isaac Sim 6.1.0.0, yam-arena (`~/yam-isaac-lab-arena`) at commit 0059d02a1a91. Task file
SHA-256, first 12 hex: dbeab380e1d5 (`sha256sum task.py | cut -c1-12`; it equals `eval.environment_revision` in the
Dream-1 log).

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the fixture as one PhysX articulation (a fixed base, the panel on a revolute hinge, the
strut's lower link on a revolute pivot and its upper link on a revolute knee with limits, closed onto the panel by a
loop joint outside the articulation), the rest support as a fixed compound body, and calls the task's per-step record
after every control step. No fork of yam_arena and no third-party assets: every body is built from box and cylinder
colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

22 tests feed synthetic records to the scorer or pose the fixture from its joint angles: a demo-shaped trace scores
1.0; a locked panel and an empty record score 0; 60.0 deg/s is allowed and 60.1 deg/s gates, falling or rising; a
slam stays a 0 even if the panel is lifted and lowered properly afterwards; the knee rung needs 8.0 deg past straight
(7.9 earns nothing) and is kept if the knee relocks; pressing the panel into the locked strut earns nothing; the
40 deg rung (40.1 vs 40.0); on the pads within 2.0 deg (2.1 is not); the release gate (29.9 vs 30.0 mm, a hand at
6 mm/s, 0.95 s is not enough, one close step restarts the timer); a 3 deg/s step restarts the panel's stillness timer;
a strut joint gap of 2.1 mm (the simulated strut coming apart) gives 0
even if a proper stow follows, while 2.0 mm is tolerated; the record is clean after a reset. The kinematics tests close the strut's loop in closed form at 75, 74, 60, 40, 20
and 0 deg (loop error under 1e-9 m, knee inside its travel, 119.7 deg folded at 0 deg), check that the locked knee
lies below the B-P line so its weight and the panel's load both press it onto the stop, that the stowed panel's
underside lies on both pads, that the folded strut stays 20 mm or more from the support, and that the clearance reads
the tab.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fold the grey radiator panel down onto its black rest pads, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/knee-brace-break-stow-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

```bash
inspect-robots inspect adhoc_a6cb0b2b.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_a6cb0b2b.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant push_panel    # a cheat variant (or DEMO_VARIANT=push_panel)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: link poses, the hinge and knee angles computed from those poses, and the robot's own
grippers. It never reads joint values or the record.

1. Left: approaches the panel's free-edge tab tilted 15 deg from straight down (leaning back toward the robot), pads
   closing along the hinge line, and pinches it lightly (1 mm squeeze). Right: closes its fingers and lowers them
   (pads closing along x) 20 mm in front of the knee pin with the pad tips 40 mm below the pin.
2. Right, once the tab is held: moves 0.6 mm per control step along the pin's own direction of travel (perpendicular
   to the lower link, up and back, read from the lower link's pose) until the knee is seen 10 deg past straight
   (33 mm of travel; seen 10.4 deg past straight), and holds there.
3. Left: lowers the panel 5 deg about the hinge line (wrist following half the angle), which commits the strut to
   folding; right then backs off forward, outboard and up.
4. Left: carries the panel down the remaining 70 deg at about 14 deg/s and corrects from the angle seen in the link
   poses until it is on the pads, opens and backs away along its approach; both wait.

The demo scores 1.0 (`success`, episode ended at step 405 of 1,200): the knee was pushed 10.4 deg past straight, the left's first 5 deg took the panel to 69.8 deg with the knee 34 deg past straight, the panel came down at no more than 20.0 deg/s (gate 60), and it rests on its pads at 0.0 deg with the strut folded 119.7 deg; both grippers ended 94 and 73 mm clear and still. A second reset (`--check-reset`) returned every joint to its start within 0.0001 rad. Worst IK error
0.08 mm and 0.01 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (dbeab380e1d5); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | The strut stays locked; the panel stays at 75.0 deg (peak speed 0.003 deg/s). | 0.0 |
| `push_panel` | The natural plan: the left pinches the tab as in the demo and folds the panel 76 deg down about its hinge over 6 s; the strut is never touched. | The strut is compressed and its knee never leaves the stop (-3.0 deg throughout); the panel stays between 74.98 and 75.00 deg while the pinch slips along the tab. | 0.0 |
| `knee_only` | The right breaks the knee as in the demo (on to 35 deg past straight) and backs away; nothing holds the panel. | The knee is pushed 37.7 deg past straight (40 mm), the right backs away and the panel falls 75 deg onto the pads: gate at 8.4 s (peak 657 deg/s). | 0.0 |
| `press_face` | Brute force on the natural plan: the right closes its fingers, comes down beyond the panel 12 mm in from its free edge (approach tilted 15 deg), drives them at 12 mm/s through contact to 30 mm past the face, toward folding, and holds that command for 3 s. | The arm is held 46 mm short of its command; the panel deflects to 74.94 deg and springs back to 75.00, the knee never leaves its stop (-3.0 deg), and the strut's lug joint opens at most 0.08 mm (gate 2 mm). | 0.0 |

Two arms, and where the complexity lies. The complexity is the hidden dependency, not a forced two-arm role: the panel
cannot move until the knee is broken, and once it is broken nothing but a hand holds the panel. The strut and panel
form a one-degree-of-freedom linkage, so a single gripper that grasps the knee pin and lets the strut fold slowly could
also lower the panel without touching it. The task does not forbid that (it is still the knee move, not the ends); the
demo uses the two-hand method technicians use, one hand on the panel and one on the knee.

## Build log (4 demo runs and 4 cheat batches on shared VMs, 1 to 9 min per run)

| Run | Change | Result |
|---|---|---|
| 1 | first build: hinge line at x 0.36, 75 mm above the table; left approaches the tab straight down | 0.0: the left arm could not reach the tab (IK error 60 mm, 26 deg); the knee push worked (10.4 deg past straight after 32 mm), and with nothing holding the panel it slammed at 657 deg/s (gate) |
| 2 | hinge line at x 0.33 and 40 mm above the table; left approach tilted 15 deg (checked offline with the IK solver along the whole arc) | 1.0 at step 405, peak panel speed 20 deg/s |
| 3 | cheat runs; the `push_panel` variant also ran the right arm's knee push (a script bug: 0.25) | fixed: in `push_panel` only the left arm moves |
| 4 | task file a408703f93c5 | 1.0 at step 405, the same end state as run 2; `nothing`, `push_panel` and `knee_only` all 0.0 |
| 5 | review changes: a `press_face` cheat and a strut-apart gate (lug joint gap over 2 mm gives 0); first `press_face` try with the left arm straight down | the press pose was out of reach (IK error 49 mm, 20 deg); moved to the right arm with a 15 deg tilt, checked offline with the IK solver along the whole push (0.1 mm) |
| 6 | final task file dbeab380e1d5 | demo 1.0 at step 405, the same end state; `nothing`, `push_panel`, `knee_only` and `press_face` all 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures the panel
angle theta (0 on the pads, 75 deg locked open, from the hinge), the knee bend (0 straight, -3 on the lock stop,
positive folding), the panel's angular speed from its angle change between control steps, and each gripper's pad
clearance to the fixture, panel, strut and rest support and its speed. The record keeps flags, peaks and held times
for the whole episode and is cleared only by the simulator reset, so lifting the panel and lowering it again never
clears a slam.

- Gates, any time in the run, give 0: the panel moved faster than 60 deg/s (the demo peaks at 20 deg/s; a panel
  let go at 75 deg reaches about 650 deg/s), or the strut's lug joint opened more than 2 mm, which means the simulated
  strut came apart (every demo and cheat run stays at or under 0.08 mm, `press_face` included).
- 0.25: the knee was bent 8 deg or more past straight at some time (the strut unlocked).
- 0.5: the panel came down to 40 deg or below at some time.
- 0.75: the panel is within 2 deg of its pads at the end.
- 1.0: on its pads, still (under 3 deg/s) for 1 s, and the release gate: every pad point of both grippers at least
  30 mm from the fixture, panel, strut and rest support and both grippers moving under 5 mm/s for the last 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: a protractor on the hinge cheek for the panel
angle, a 30 fps video for the panel's speed and the knee crossing centre, and a top photo for the hands' clearance.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step
  405.
- Runtime: a demo run took about 2.5 min wall clock on the shared L4 VM (3.5 to 7 control steps/s, plus 1 to 3 min
  of Isaac Sim start-up). A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (both bodies are fixed). The
  scripted demo is deterministic; Dream-1 is a hosted model without a seed.

## Scored runs

| Run | inspect-robots-dream | yam-arena commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 0.1.0 | 0059d02a1a91 | dbeab380e1d5 | 1200 | 3 min | adhoc_a6cb0b2b.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | - | 0059d02a1a91 | dbeab380e1d5 | 405 (success held 1 s) | about 2 min | demo_out/result.json | 1.0 | panel on its pads, no slam, both grippers let go |

The demo video is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot
`demo_out/setup.png` (frame 0 of the top camera). Make the Dream-1 run on this exact `task.py` after a 40-step probe
of the instruction (`--max-steps 40`, no frames) to rule out a refusal.

## Materials

- Stowage fixture, fixed to the table: dark grey anodised aluminium, 2.0 kg. A 200 x 275 x 6 mm foot plate, a
  24 x 190 mm hinge pedestal with two cheeks and knuckles behind the hinge line (along y at x 0.33, 40 mm above the
  table), and a 12 x 10 mm post carrying the strut's base pivot 120 mm in front of the hinge line at hinge height.
- Radiator panel: aluminium honeycomb with grey thermal paint, 250 g, 200 mm along the hinge, 120 mm deep, 5 mm
  thick, with a 30 x 25 x 5 mm grab tab on its free edge (on the robot's left half) and a strut lug on its right
  edge 60 mm from the hinge line. Locked open at 75 deg; hinge travel 0 to 80 deg.
- Deployment strut: two yellow anodised aluminium bars, 12 x 8 mm section, 59.8 mm between pins, 15 g each, beside
  the panel's right edge, from the base pivot to the panel lug. A steel knee pin, 10 mm across, sticks out 25 mm
  toward the robot's right. The knee locks 3 deg past straight (down and away from the robot) against its stop and
  folds up to 150 deg; there is no lock spring. Folded with the panel stowed, the knee is bent 120 deg and stands
  52 mm above the hinge line beside the panel.
- Rest support: a grey block 70 x 170 x 28.5 mm on the foot plate under the stowed panel, with two black rubber rest
  pads 20 x 24 x 3 mm whose tops are at the stowed panel's underside.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Lift the panel by its tab past 75 deg until the strut is straight, let the knee drop down and
away onto its stop, then lower the panel until the strut carries it at 75 deg (protractor on the hinge cheek; the
knee pin rests 3 deg past straight). The fixture and the rest support are fixed on taped outlines and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
