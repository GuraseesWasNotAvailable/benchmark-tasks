# Take a drawer out of its slides by pressing two opposite-sense release levers while pulling

The robot takes a drawer out of its two ball-bearing slides. Each slide has its own spring disconnect lever, and the
two work in opposite senses: the left lever is a tab under the drawer wall that is pushed up, the right lever is a tab
over the wall that is pushed down. The end stop clears only while both levers are held, so one lever alone leaves the
other slide locked and the drawer jams at the stop. The robot must hold both levers and pull at the same time. At the
end the drawer is 280 mm or more from closed and at rest, and both grippers have let go. Taking a drawer out to clean,
paint or fix a cabinet is a routine household job.

Physics, not the instruction, defeats the natural plan. The drawer starts pulled out 199 mm against the stop. Pulling
harder does nothing, one hand on one lever does nothing, and tapping both levers and letting go before the dogs are
past the stop drops the latch back in.

Instruction given to the policy (18 words; goal only, no lever or technique):

`Take the drawer fully out of its slides and set it down on the table in front of the cabinet, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo
and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task.py sha 534d07c9dab6 (`sha256sum task.py | cut -c1-12`)

## Offline check, no simulator

```bash
pytest -q tests        # 13 scorer tests, pure Python (yam_arena must be importable for TABLE_TOP)
```

## Run and score one Dream-1 trial

Placeholders for the filing run, to fill from the log: Dream-1 run on 1.0.0 at 0059d02a1a91, task 534d07c9dab6, 1200 steps
(3 minutes), log adhoc_6b9a1ded.json, score 0.0, max_steps.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant left_only      # a cheat variant (or DEMO_VARIANT=left_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json`. The script reads only the drawer's link poses (what a camera could
measure) and the robot's own grippers; it never reads the levers' travel or the record.

1. Both arms together. Each arm starts 80 mm in front of its lever, tilted 15 deg down, then goes forward until its
   pads straddle the lever tab and the fixed flange (pad span 77 mm, opened to 94 mm) and closes to 68.5 mm. Closing
   pushes each tab 6 mm toward its flange (left up, right down), which clears the stop.
2. Holding the squeeze, both arms draw 95 mm toward the robot over 5 s; the drawer goes from 199 to 292 mm.
3. Both open, back off 70 mm along the pull and wait until the drawer and grippers are still.

The demo scores 1.0 (`success`, step 350 of 1,200). Both levers reach 6.0 mm, the drawer passes the stop at 9.5 s and
ends at 292 mm, and both grippers end 35 mm clear.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file; every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. | 0.0 |
| `left_only` | Only the left arm presses its lever and pulls. | Left lever reaches 6 mm, right stays at 0.16 mm; the drawer stays at the 200 mm stop. | 0.0 |
| `right_only` | Only the right arm presses its lever and pulls. | The mirror: right 6 mm, left 0; the drawer stays at the stop. | 0.0 |

Pressing both levers and letting go before pulling scores 0.25 (the ladder rung for pressing them together), not 0.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Whole-run record (a linkkit History fed every control step and cleared only by the simulator reset), so letting go,
pushing the drawer back or starting again never clears it.

| Score | Condition |
|---|---|
| 0 | the two levers were never pressed (4.5 mm or more) at the same time |
| 0.25 | both pressed together at some moment, but the drawer never reached 215 mm from closed |
| 0.5 | reached 215 mm, but at the end not 280 mm or more out and still |
| 0.75 | at the end 280 mm or more out, still (under 5 mm/s) for 1 s, a gripper not yet clear |
| 1.0 | also both grippers 30 mm or more from every part of the drawer and under 10 mm/s for 1 s (release gate) |

Real-trial measurement: a side video at 30 fps and a tape on the bench top. Each lever stem has a printed 6 mm scale
and a 1.5 mm gauge strip at its flange; a lever counts as pressed when the tab-to-flange gap is within the strip,
read frame by frame. Extension is read from the front face against the tape; still is no tape movement over 30
frames; clearance is checked with a 30 mm spacer card passed around each gripper and the drawer.

## Run limit, runtime and seeds

1,200 steps (1 minute at 20 Hz). The demo finishes at step 350. The scene is deterministic; there is no random seed.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 350.
- Dream-1 run: `adhoc_6b9a1ded.json`, `task_score = 0.0`, ended by `max_steps` after 1200 steps (3 min).

## Materials

- Drawer bay: plywood, fixed to the table on an 85 mm plinth, 300 x 316 x 200 mm, 8 kg, with two steel ball-bearing
  slide members 250 x 12.7 x 35 mm.
- Drawer: 12 mm plywood, 1.2 kg, 280 x 240 x 75 mm with an inset front, 6 mm bottom, 100 mm above the table, with two
  steel inner members 230 x 12.7 x 35 mm. It stands pulled out 199 mm against the end stop.
- Left lever (red, ABS, 20 g): a 35 x 25 x 3 mm tab 9 to 12 mm under the left wall, pushed up 6 mm against a 1.2 N/mm
  spring (5.4 N at 4.5 mm, 7.2 N at 6 mm), with a fixed 55 x 25 x 5 mm steel flange on the wall top (pad span 77 mm).
- Right lever (blue, ABS, 20 g): a 35 x 25 x 3 mm tab 74 to 77 mm over the right wall, pushed down 6 mm against the
  same spring, with a fixed 55 x 25 x 5 mm flange under the wall (span 77 mm).
- Latch: the stop clears only while both levers are at 4.5 mm or more. Verify on the bench: clamp either lever and
  pull 30 N, the drawer must not pass the stop; with both pressed it passes at under 5 N.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Fidelity

The drawer, slides, springs and levers are real full-size parts. The simulator models the latch as a joint limit that
lifts while both levers are down and declares the drawer out at 280 mm because the arms reach 0.35 m; a real inner
member runs on to its end. The inset front and the wall flanges give the second pad surface the pinch needs.

## Reset between trials

Nothing is consumed. Push the drawer back until the stop dogs seat at the 199 mm stop, let both levers spring back to
rest, and check the 5.4 N lever force at 4.5 mm with a spring scale.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
