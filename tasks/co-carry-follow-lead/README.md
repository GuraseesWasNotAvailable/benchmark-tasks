# Help a person carry a pipe onto its stands, following their lead

Instruction given to the policy: `Help the person lift the silver pipe onto the two pipe stands, then let go of it.`

The ability tested is leader-follower co-manipulation: matching a person's motion of a load you share with them,
as they make it. A 60 cm silver aluminium pipe lies on two grey foam blocks in front of the robot. A worker in a
hi-vis vest stands across the table and grips its far end in their right hand. Two orange pipe jack stands with
V-heads stand 18 cm to the robot's left. Once the robot has hold of the pipe, the person lifts their end 10 cm,
carries the pipe across, stops once part way, goes on to the stands and lowers it into the V-heads. They do it at
their own pace, which changes widely with the trial seed: they wait 0.5 to 5 s before lifting, take 1 to 2.5 s to
lift, 2 to 5 s to move across, and stop for up to 3 s somewhere between 30 and 70 percent of the way. The robot
has to watch the person and move with them: lift when they lift, move across at their speed, stop when they stop,
set down when they do, and let go only once the pipe is down.

Physics defeats the natural wrong plans. The pipe hangs from the person's hand on a spring grip and pivots in the
robot's grippers. A robot that lifts first, holds still, moves faster or slower than the person, or lets go early
tips or swings the pipe past 4 degrees (or stretches the person's grip past 25 mm), and the run scores 0. No fixed
timing works. Four degrees over the 0.5 m from the robot's grips to the hand is 35 mm, and an open-loop script that
plays the mid-range person's path is more than 45 mm off the person's hand at some point on 97 percent of seeds. In
the simulator it scores 0 at the scored trial's seed and at two others, and so does a script that plays the
person's path averaged over 500 seeds. Without the robot's hold the person never lifts, and the robot cannot raise
the pipe level on its own.

Files in this folder:

- `task.py`: the scene, the person, the scorer and the scripted demo with its cheat variants.
- `linkkit.py`: our articulated-body kit (import hook, linkages, joints, springs, whole-run history).
- `linkdemo.py`: the closed-loop demo runner.
- `tests/test_scoring.py`: scorer edge cases.
- `task.json` and `README.md`.

## Environment

Built on the program's Brev launchable (Isaac Lab-Arena with the YAM arms, Isaac Sim, one L40S). Versions used for
the submitted runs:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: 04298a9da18b (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision`
  in the log)

Before every shell, source the launchable's `yam.env` (on our VM: `source /data/IsaacLab-Arena/yam.env`).

## Offline check, no simulator

```bash
cd tasks/co-carry-follow-lead && python -m pytest -q tests
```

Twenty-three tests feed synthetic control steps through the task's own `measure()`, `record_step()` and `ladder()`:
an ideal co-carry scores 1.0 at three seeds; 26 mm of grip stretch for one step scores 0 even after it comes back
(24 mm does not); 4.5 degrees of pitch or yaw at any step scores 0 (3.5 does not); lifted but never carried 0.25;
carried but not set 0.5; set with a gripper 20 mm from the pipe 0.75 (35 mm is 1.0); let go before the person
finished lowering 0.75; opening past 32 mm on the pipe is letting go at once, and opening to 31 mm counts when the
gripper moves 10 mm off the pipe; a one-arm regrasp in the middle of the carry, while the other gripper holds, is
not letting go (both off at once is); fewer than 20 still steps at the end 0.75; the person starts only after 20
consecutive steps of hold, anywhere from the robot's end of the pipe to 8 cm short of the person's fist, with the
gripper closed to 30 mm or less; doing nothing scores 0 and the person never starts; a gripper that never held is
not a release; the seed draws stay inside their ranges; the person's path stops part way across and ends with the
pipe in the V-heads; the slowest person fits the run limit; the mid-range person's timing played blind strays more
than 60 mm at the scored trial's seed and stays within 45 mm on under 6 percent of 400 seeds; and the history
clears only on reset.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/co-carry-follow-lead/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Help the person lift the silver pipe onto the two pipe stands, then let go of it." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/co-carry-follow-lead-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, on the ladder
below (0, 0.25, 0.5, 0.75, 1.0). The CLI prints `task_score: <value>`, and the log keeps it at
`results.metrics.task_score` and `samples[0].epochs[0].task_score`. With `--policy yam_scripted` the same command
holds both arms at ready (`demo()` in `task.py` is empty) until the policy stops, and scores 0: we ran it with
`--max-steps 200`, and it ended by `policy_stop` after 61 steps with `task_score: 0` and
`eval.environment_revision` equal to the task file's SHA. The closed-loop demo runs through `linkdemo.py` below.

## Scripted demo and cheat variants

```bash
cd tasks/co-carry-follow-lead
python linkdemo.py task.py demo_out --max-steps 1200                             # seed 0
COCARRY_SEED=1415158438 python linkdemo.py task.py demo_dream --max-steps 1200   # the scored trial's person
python linkdemo.py task.py cheat_blind --max-steps 1200 --variant blind         # any variant from the table
```

`linkdemo.py` steps the same yam_arena embodiment and scorer as inspect-robots and writes `top_cam.mp4`,
`left_cam.mp4`, `right_cam.mp4`, `setup.png` and `result.json` (task_score, steps, worst IK error, and the
whole-run record). Outside inspect-robots the person's seed comes from `COCARRY_SEED` (default 0).

The demo (`demo_script` in `task.py`) is closed loop and reads only what the cameras show: the poses of the person's
hand and of the pipe (`ctx.observe()`), plus its own gripper openings. It never reads the seed, the person's plan or
any joint of the person.

1. Grasp. Both arms come down onto the pipe along their grippers' axes, each wrist leaning 20 degrees toward its own
   arm's base (the left arm reaches 35 cm across), with the pads closing across the pipe at x 16.5 and 26.5 cm
   (10 cm apart near the robot's end). They close to 22 mm on the 25 mm tube and hold still. After 1 s of hold the
   person starts.
2. Follow. Every control step both grips move by the observed hand displacement since the start, plus 0.1 s of the
   hand's observed velocity (to cover the arm's lag), plus a small integral correction that keeps the pipe level
   and straight as seen from its pose.
3. Set down and let go. When the hand has come down over the stands and stayed still for 0.5 s, both grippers open
   to 60 mm, back off 6 cm along their axes and return to ready.

Seed 0: `task_score` 1.0, ended by `success` after 479 steps (24.0 s): rungs reached at steps 173 (0.25), 284 (0.5),
346 (0.75) and 460 (1.0). Peak grip stretch 2.4 mm, pitch 0.31 degrees, yaw 0.82 degrees; the grippers let go 0.87 s
after the person finished lowering. The scored trial's person (seed 1415158438): `task_score` 1.0, ended by `success`
after 484 steps (24.2 s), peak stretch 2.7 mm, pitch 0.58 degrees, yaw 0.44 degrees, let go 0.88 s after the lowering
ended. The slowest person among seeds 0 to 199,999 (seed 21575: 21.4 s from the trigger to the end of the lowering,
against a 22.5 s maximum): `task_score` 1.0, ended by `success` after 647 steps (32.4 s), peak stretch 2.3 mm, pitch
0.15 degrees, yaw 0.23 degrees. Worst IK error on both arms 0.1 mm and 0.01 degrees in every run.

Each variant below was run in the simulator against the final `task.py` (the same `demo_script` with `DEMO_VARIANT`
set, in one Isaac session with a simulator reset between runs; `python linkdemo.py task.py OUT --variant <name>`
reproduces any one, with `COCARRY_SEED` for the seed), and each scored exactly 0:

| Variant | What the robot does | Seed | Simulator score | Why it scores 0 |
|---|---|---|---|---|
| `nothing` | Never takes hold, so the person never lifts | 0 | 0.0 | No rung reached: the pipe never leaves the foam (peak grip stretch 0.11 mm) |
| `stay` | Takes hold, then holds still while the person moves | 0 | 0.0 | Pitch 10.8 degrees as the person lifts their end, then yaw 18.2 degrees as they carry it across (stretch 26.3 mm) |
| `lead` | Takes hold, then lifts its end 10 cm in 1 s before the person starts | 0 | 0.0 | Pitch 11.0 degrees as its end rises before the person's; yaw 17.9 degrees later |
| `rush` | Follows the lift, then carries the full 18 cm in 1.0 s as soon as the person starts across (they take 2 to 8 s, stop included) | 0 | 0.0 | Yaw 6.7 degrees as its end runs ahead of the person's, who stops part way |
| `early_release` | Follows, then opens both grippers at the end of the carry, before the person lowers | 0 | 0.0 | Pitch 9.5 degrees as its end drops into the near V-head while the person's is still up; the early release alone would cap it at 0.75 |
| `blind` | Never looks: plays the mid-range person's path (every draw at the middle of its range) open loop from its own grasp, holds, and lets go 23.5 s after the person starts | 1415158438 | 0.0 | Pitch 5.4 degrees as its end rises 0.9 s before the person's, then yaw 5.8 degrees as it moves across out of step |
| `blind` | (same) | 0 | 0.0 | Yaw 11.3 degrees as it keeps going while the person stops part way for 2.7 s (pitch 5.2 degrees) |
| `blind` | (same) | 156 | 0.0 | Pitch 10.5 degrees as the person lifts 1.6 s before it does; yaw 14.2 degrees later |
| `blind_mean` | Never looks: plays the person's path averaged over seeds 0 to 499 (the best guess without looking) open loop, holds, and lets go 23.5 s after the person starts | 1415158438 | 0.0 | Pitch 4.6 degrees as its smoothed lift runs ahead of the person's late lift (yaw 4.5 degrees later) |

## Scoring (`score()`, `success()` and `distance()` in `task.py`)

Every control step `on_step` measures the pipe's pose and speed, the person's hand and both grippers (`measure()`)
and updates the whole-run record `H` (`record_step()`). The kit clears `H` only when the simulator resets for a new
trial, so putting the pipe back never clears a violation. `score()` is `ladder(H, measure(now))`, gates first:

- **0** if, at any step, the person's grip point on the pipe (27 cm from its centre) was more than 25 mm from their
  hand (they lose their grip), or the pipe's axis pitched or yawed more than 4 degrees from level and straight (the
  load tipped or swung).
- **0** if no rung below was reached.
- **0.25**: at some step both pipe ends were at least 40 mm above their start height.
- **0.5**: at some step both ends' centre line was at least 11 cm above the table with the pipe's centre at y +4.5 cm
  or more (carried over the stands).
- **0.75**: now the pipe rests in both V-heads: its centre line within 5 mm of 9.5 cm above the table where it
  crosses each stand, its centre within 8 mm of y +9 cm and within 20 mm of x 44 cm.
- **1.0**: 0.75, and now every pad point and both TCPs are at least 30 mm from the pipe surface, both grippers and the
  pipe have moved slower than 5 mm/s for the last 20 steps (1 s), and the robot let go only after the person
  finished lowering. The robot has let go at the first step after the person started at which no gripper holds the
  pipe and every gripper that held it since then has opened past 32 mm or has its pads and TCP all more than 10 mm
  off the pipe. Regrasping with one gripper while another still holds is not letting go. Letting go before the
  lowering ended caps the score at 0.75.

A gripper holds the pipe when its TCP is within 15 mm of the pipe's axis, anywhere from the robot's end (x 14 cm) to
8 cm short of the person's fist (x 63 cm), with the gripper closed to 30 mm or less. The person starts after 20
consecutive control steps (1 s) in which a gripper holds.

`SCORE_OVER = "end"`: the episode's score is `score()` at its last step. `success()` is `score() >= 1.0` and must
hold for 1.0 s (`SUCCESS_HOLD_S`), which ends the episode. `distance()` is `1 - score()`.

A real trial is scored the same way: the person holds the far end through a spring handle (800 N/m) with a red and
green band that shows 25 mm of stretch, a phone inclinometer on the pipe reads pitch, a top photo reads yaw and
position, and tape marks on the V-heads show the resting height.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s of simulated time), `--max-steps 1200` above, recorded as
  `eval.max_steps`. The demo ends by success at 479 to 647 steps (24 to 32 s) at the three seeds above; the slowest person the ranges allow (22.5 s from the trigger to the end of the lowering) would end near 33.5 s (about 670 steps), well inside the limit.
- Runtime per trial on an L40S: the demo took 5 to 13 min in `linkdemo.py` (0.9 to 1.6 control steps/s while other
  simulations shared the GPU), plus about 2 min of Isaac Sim start-up. A 1,200-step trial is 60 s of simulated
  time; a policy's wall clock adds its own inference time.
- Seeds: the layout is fixed by `-E object_jitter=0`; the seed sets only the person's timing (`partner_plan(seed)`,
  eight uniform draws from a numpy `default_rng`). inspect-robots resets the embodiment with
  `derive_seed(--seed, scene seed, epoch)`, so `--seed 0` gives trial `scene-0-e0` the seed 1415158438: the person
  waits 3.67 s, lifts over 1.05 s, pauses 2.50 s, moves across for 2.57 s in all with a 1.28 s stop at 54 percent
  of the way, pauses 0.68 s and lowers over 1.46 s (13.2 s from the trigger to the end of the lowering).
  `--epochs K` gives each trial a new person. Dream-1 is a hosted model without a seed, so its trials are not
  bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run (filled in from the 1,200-step run at filing): log `@@DREAM_LOG@@`, trial `scene-0-e0`,
  `task_score = @@DREAM_VALUE@@`, ended by @@DREAM_REASON@@ after @@DREAM_STEPS@@ steps.
- Demo: `python linkdemo.py task.py demo_out --max-steps 1200` (seed 0), `result.json` `task_score = 1.0`, ended by
  `success` after 479 steps; the uploaded video is `demo_out/top_cam.mp4` and the setup screenshot
  `demo_out/setup.png`.

## Verify

```bash
inspect-robots inspect adhoc_3cd2cc74.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_3cd2cc74.json
```

## Materials

- Pipe: 25 mm outside-diameter aluminium tube, 600 mm long, about 300 g, with black plastic end caps and a 20 mm
  red tape band from 245 to 265 mm out from its centre, at the person's end.
- Two steel pipe jack stands with V-heads (orange), about 1 kg each, clamped to the table 26 cm apart: four splayed
  legs on 9 cm centres, a round post with a height pin, and a V-head 40 mm long opening 45 mm wide, vertex 7.7 cm
  above the table. Tape marks on the V-heads at the resting height.
- Two grey EVA foam blocks, 5 x 5 x 4 cm, about 10 g each, taped to the table.
- The person: one adult in work clothes (or a second robot arm) standing across the table and holding the far end
  of the pipe through a spring handle, following a printed count table of the eight timing draws for the trial's
  seed. A phone inclinometer on the pipe.

In the simulator every prop is a primitive authored by `linkkit.py` (the person is a rigid figure: torso in a
hi-vis vest, sleeved arms, a hand of capsule fingers round the pipe, head and hard hat); there are no third-party
assets.

## Reset between trials

Nothing is consumed. Lay the pipe back on the two foam blocks at its tape marks (x 14 to 74 cm, y -9 cm) with the
red band at the person's end. The person takes hold at x 71 cm, steps back to their start mark and keeps their hand
still at its start height until the robot has held the pipe for 1 s. Check that both stands are on their marks. In
the simulator the kit restores every joint, spring target and the whole-run record on reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques and copy no code from other tasks' kits.
