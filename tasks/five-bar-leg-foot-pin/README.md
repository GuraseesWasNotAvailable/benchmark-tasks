# Pin the foot of a five-bar robot leg

The robot brings the free ends of two dangling jointed chains together and locks them into one closed loop. The
chains are the two halves of a five-bar leg for a small direct-drive legged robot, hanging from their hip bearings
with no motors fitted; the lock is the foot joint, an orange steel stud on the left lower link that has to click into
a blue spring-latched jaw on the right one. At the end the stud is home in the jaw, the latch tongue is shut behind
it, and both grippers have let go. Electromechanical equipment assemblers (O*NET 51-2023.00) do this as the last step
of building every such leg, and again after a fall that splits a foot. The hips are
100 mm apart, the textbook five-bar layout; Ghost Robotics' Minitaur and Stanford Doggo, named only as references,
put both hip motors on one axis.

The ability is closing a loop between two passive chains: neither end can be pushed onto the other, because the other
end gives way. Each arm has to take one chain, bend it into the leg's closing pose, and the two arms then press the
ends together, equal and opposite.

What is new is that both parts are kinematically constrained and passive: each end hangs on its own two-joint chain,
can move only along that chain's joint path, gives way to any push, and cannot reach the other at full stretch.
Unlike TK-VL43QQ (one free link fitted onto a fixed pin, where the skill is choosing the closure branch), nothing here
is fixed to work against; unlike TK-K65XCN (two connectors on loose cords) or TK-CGNU3G (two arch halves set down
together), neither part can be lifted free and placed.

Physics, not the instruction, defeats the one-arm plan:

- Neither chain can reach the other's hanging foot. The hip bearings are 100 mm apart and each chain is 185 mm long,
  so a chain at full stretch stops 25 mm short of the other foot.
- A held chain cannot get round the other one to draw it in: the stud runs back through the other chain's layers,
  so it meets the other shin wherever they cross, and pushing the other shin only swings it away.
- A free chain pushes back with 0.4 N at most (its whole weight torque about the hip, read at the foot), while the
  latch tongue's spring starts at 1.0 N and rises to 1.65 N over the 6.5 mm the stud drives it back through its
  45 deg lead-in.
- One gripper cannot span both hanging shins (their outer faces are 112 mm apart; the gripper opens 94 mm), and
  nothing rigid lies in the leg's plane to brace a chain on: the hip block and stand sit behind the links.

Instruction given to the policy (22 words; it states the goal only):

`Join the robot leg's two lower links at the foot so the orange stud clicks into the blue jaw, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer and geometry edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one GPU). Versions used for the demo
and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`), inspect-robots-yam 0.38.0
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: 456c8f9e9b0e (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision`
  in the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the leg as one PhysX articulation (a fixed hip block, two free two-link chains on revolute
bearings and a sprung prismatic latch tongue, USD joints with limits and drives, spawned through Isaac Lab's
`ArticulationCfg`) and calls the task's per-step record after every control step. No fork of yam_arena and no
third-party assets: every body is built from box and cylinder colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

31 tests (they need `yam_arena` importable, as on the launchable). The scorer tests feed synthetic records: a
demo-shaped trace scores 1.0; the snap window (stud 1.5 mm from the pocket centre is in, 1.51 mm is out; tongue
1.0 mm back is home, 1.01 mm is not; a stud pushed home with the tongue still back is not snapped); the 0.25 rung at
10.0 mm but not 10.01 mm, and only with each shin held by its own gripper (one shin held, or one gripper listed on
both, earns nothing; arms crossed over still count); a snap made one-armed still scores, since physics and not the
scorer stops that plan; clicked and then pulled out scores 0.5; still held at the end scores 0.75; the release gate
(a pad 29.9 mm away or a hand moving 6 mm/s scores 0.75, and 0.95 s of release is not enough); a snap that only just
came back needs its own full second; the record is whole-run and clears only on reset. The geometry tests use the
leg's own kinematics: the hanging feet 100 mm apart; `chain_angles` puts a foot exactly where asked with the knee
out; in the demo's closing pose the jaw's mouth faces +y and the stud sits in the pocket centre; the latch reading
follows the tongue; the tongue needs 6.5 mm of its 10 mm travel to pass the stud and drops back only within 0.5 mm of
the pocket bottom; the guide lip reaches past the lead-in's first touch; a free chain pushes back with under 0.4 N,
under 1/2.5 of the latch preload; neither chain reaches the other's hanging foot and one gripper cannot span both
shins; only the stud crosses between the two chains' layers; the demo's view asks for the two shin poses only (never
the tongue) and reads the stud in the jaw exactly as the scorer does.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Join the robot leg's two lower links at the foot so the orange stud clicks into the blue jaw, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/five-bar-leg-foot-pin-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Read a log with:

```bash
inspect-robots inspect adhoc_fa720ab2.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_fa720ab2.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                        # scripted demo
python linkdemo.py task.py cheat_out --variant one_arm     # a cheat variant (or DEMO_VARIANT=one_arm)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the thigh and shin poses (the hinge angles it needs come from them, and the stud's place
in the jaw from the two shins), and the robot's own grippers. It never reads the latch tongue, the latch state, joint
values or the record: it decides the stud is home from its depth in the pocket alone (`_seen_jaw` in `task.py`; a
test fails if it asks for any other link).

1. Both arms at once: each grips its hanging shin 60 mm below the knee from the leg's outside, the approach in the
   leg's plane and 45 deg below horizontal, pads closing along the bearing axis on the shin's front and back faces
   (6 mm squeeze). It records the grasp from the seen shin pose and the arm's sag (0.5 mm).
2. Both arms bend their chains knee-out into the leg's diamond along each chain's own joint path (computed from the
   hip bearing and the link lengths, so the held chain bends instead of fighting its bearings): the right jaw goes to
   150 mm below the hip line with its mouth facing the left chain (right shin tilted 43.6 deg); the left stud stops
   22 mm in front of the mouth.
3. The left arm lines the stud up with the mouth from the seen poses (22.2 mm out, 0.0 mm across after two
   corrections), then drives it home along the mouth over 3 s to 5 mm past the pocket bottom while the right arm
   presses its jaw 3 mm onto it, equal and opposite. The stud rides the guide lip, pushes the tongue back through the
   lead-in and the tongue clicks shut behind it (0.4 s later the shin poses show the stud 0.65 mm from the pocket
   centre, inside the 1.5 mm window, so the press ends; the scorer's record shows the tongue home).
4. Both grippers open and back away 90 mm along their approach and lift. The closed leg swings down and settles
   hanging on its own, foot below the hips (thighs 8 deg and shins 21 deg off vertical).

The demo scores 1.0 (`success`, episode ended at step 398 of 1,200): the stud first came within 10 mm of the jaw with
both shins held at 13.10 s, clicked in at 13.75 s and stayed snapped for the last 6.2 s; at the end it sits 0.71 mm
from the pocket centre (0.5 mm along and 0.5 mm across, the pocket's play) with the tongue home, and the grippers are
56.5 and 68.4 mm clear and still. Worst IK error 0.07 mm and 0.03 deg. With `--check-reset`, a second reset puts
every joint back at its start (largest error 0.1 deg, the right knee), the leg at rest (0.16 mm/s) and the record
cleared.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (456c8f9e9b0e); every one scores
exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Both chains hang still; the stud stays 100 mm from the jaw. | 0.0 |
| `one_arm` | Left arm only: grips its shin, then four times reads the right jaw's pose and drives the stud to the front of the mouth and home along it, each target clipped to the chain's reach. | The left chain reaches full stretch and stops with the stud 28 mm from the hanging jaw's centre; it never touches the right chain (right shin within 1.8 deg of hanging). | 0.0 |
| `one_arm_right` | Right arm only, the mirror: four tries at driving the jaw's mouth over the hanging stud. | The right chain stops at full stretch 27 mm short; the left chain never moves. | 0.0 |
| `push_free` | Left arm grips its shin and shoves the free right shin with the stud, 15 mm past first touch, then lets go. | The stud meets the right shin's inside face about 30 mm above the jaw and the free chain swings away (shin 4.3 deg, thigh 2.9 deg) instead of pushing back; it swings home when the left lets go. The arm ran up to 6 mm short of the shove's far end (IK). A harder shove cannot help: the free chain's restoring force is 0.4 N at most at any swing, below the latch's 1.0 N preload, so it always swings further rather than drive the tongue. | 0.0 |

## Build log (demo runs on the shared L4)

Offline first (`ik.solve` and the arms' URDF hulls, no simulator): gripping the shins from the front (pads across the
bar, wrist pointing forward) made the two forearms collide by 7 to 23 mm at every layout tried, because both wrists
meet between the hips. Gripping each shin from the leg's outside, pads on its front and back faces, keeps the arms
apart (no arm-to-arm contact anywhere on the path) and every pose inside reach (0.07 mm IK error); the grasp moved to
60 mm below the knee to keep the fingertips 3 mm or more off the thighs.

| Run | Change | Result |
|---|---|---|
| 1 | first build: outside grips with a 2 mm squeeze, latch 200 N/m with 2.0 N preload, the stud driven 1.5 mm past home | 0.0: the stud stopped 10.9 mm out; the lead-in pushed it 3.3 mm sideways onto the end of the jaw's lower wall and the tongue never moved. The arms' position control is soft (wrist joints 10 N m/rad), so a 1.5 mm overshoot could not make the click force |
| 2 | latch 100 N/m with 1.0 N preload; the jaw's lower wall runs 16 mm out as a guide lip under the stud; 6 mm squeeze; the stud driven 5 mm past home and the right jaw pressed 3 mm onto it | 1.0, success at step 398 |
| 3 | final text, `push_free` cheat added (df499637ea4e) | 1.0, the same end state; with a second reset the leg returned to its start |
| 4 | `push_free` kept inside the arm's reach (its first version asked for a pose 13 mm out of reach), hinge angles added to the diagnostic trace (6e520fcbed51) | 1.0, the same end state to 0.01 mm |
| 5 | the demo stops reading the tongue: it sees only the shin poses and ends the press on the stud's depth in the pocket (within 1.5 mm of its centre) instead of the scorer's snap test (456c8f9e9b0e) | 1.0, success at step 398 with the same end state (stud 0.71 mm from the centre, tongue home, grippers 56.5 and 68.4 mm clear): the first depth check, 0.4 s after the press, saw the stud 0.65 mm from the pocket centre and ended the press, as the snap test had in run 4 |

## Scoring (`score()` at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures, from link
poses: the stud's centre in the jaw frame (mm from the pocket centre, along the mouth `a` and across it `b`), the
tongue's travel back from its stop, which gripper holds which shin (opening at most 30 mm with a pad within 4 mm of
it), and each gripper's pad clearance to every part of the leg and its speed. Snapped means the stud's centre within
1.5 mm of the pocket centre (the pocket allows 0.71 mm at most) and the tongue back within 1.0 mm of its stop. The
record keeps flags, lows and held times for the whole episode and is cleared only by the simulator reset, so pulling
the foot apart again never erases an earlier rung.

| Score | Condition |
|---|---|
| 0.0 | none of the below |
| 0.25 | both shins held, each by its own gripper, with the stud within 10 mm of the jaw centre at some time |
| 0.5 | snapped at some time |
| 0.75 | snapped at the end |
| 1.0 | snapped for the last 1 s, and the release gate: every pad of both grippers at least 30 mm from every part of the leg and both grippers moving under 5 mm/s for the last 1 s |

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: look into the jaw's mouth (and a 0.5 mm feeler)
for the stud home and the tongue across it, a tape for the 30 mm gripper clearance, and the video for stillness.

## Scored runs

| Run | yam-isaac-lab-arena | Commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Scripted demo, recorded (`linkdemo.py --check-reset`) | git checkout | 0059d02a1a91 | 456c8f9e9b0e | 398 (success held 1 s) | about 2 min | demo_out/result.json | 1.0 | foot pinned (stud 0.71 mm from the jaw centre, tongue home), both grippers let go |
| Dream-1 (`--policy dream`, command above) | git checkout | 0059d02a1a91 | 456c8f9e9b0e | 1200 (run limit) | about 3 min | adhoc_fa720ab2.json | 0.0 | max_steps |

The Dream-1 run was made at filing on this exact `task.py` (`eval.environment_revision` 456c8f9e9b0e in its log),
with the instruction and command above; Dream-1 did not refuse the instruction and ran the full 1,200 steps.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 398
  (19.9 s).
- Runtime on one L4 (shared with other builders' simulations): the recorded demo took about 2 minutes wall clock
  including about 1 minute of Isaac Sim start-up (3.5 to 7.5 control steps/s); the cheat variants take 1 to 3 minutes.
  The first run after the VM boots adds a few minutes of warm-up. A Dream-1 trial is slower, since Dream round trips
  pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (the leg is fixed to the table and
  never jittered). The scripted demo is deterministic (runs 2 to 4 ended in the same state); Dream-1 is a hosted model
  without a seed.

## Materials

- Hip block and stand: grey aluminium, 1.5 kg, fixed to the table on a taped outline: a 40 x 170 x 45 mm block on a
  30 x 40 x 235 mm post and a 90 x 160 x 8 mm foot. Two 8 mm steel hip axles 100 mm apart, 255 mm above the table.
  The block and stand sit behind the links, out of the leg's plane.
- Two thighs: black 3D-printed PLA, 40 g each with bearings, 8 x 16 x 91 mm (75 mm bearing to bearing).
- Two shins: carbon-fibre plate, 8 x 12 mm section, 110 mm from the knee bearing to the foot point. The left one
  (122 mm long, 38 g) carries the stud; the right one (42 g) ends in the jaw.
- Stud: orange-painted steel, 8 mm diameter, 11.5 mm long, with a 14 x 3 mm head, pressed into the left shin's foot
  end and pointing back through the right shin's layer.
- Jaw: blue anodised aluminium, 8 mm thick, on the right shin's foot end: a 9 x 9 mm pocket (0.5 mm play around the
  stud) whose mouth faces the leg's inside at 46 deg to the shin, with a 16 mm guide lip and a 45 deg lead-in.
- Latch tongue: spring steel, 3 g, a 20 x 1.5 x 7 mm strip with a 12 mm 45 deg lead-in, on a 100 N/m spring with
  1.0 N preload and 10 mm travel; its tip sits 2.5 mm past the pocket centre.
- Bearings: every hip and knee bearing turns freely (0.002 N m), limits +-150 deg.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Push the tongue back with a fingernail, lift the stud out of the jaw and let both chains hang
straight down at rest (feet 100 mm apart, about 70 mm above the table). Check that the tongue springs back across the
mouth. The hip block and stand are fixed on a taped outline and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, and the scene uses no third-party assets; real robots are named only as
references. The contact snap (a sprung
slider tongue that the part itself clicks past) is the same idea as the latches of `partkit.py` in accepted task
TK-7FTC8C; no code from it is used.
