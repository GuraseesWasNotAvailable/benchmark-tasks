# Set an impulse sprinkler's two trip collars to a 90-degree arc

The robot sets the watering arc of a brass impulse sprinkler. The arc is the angle between two plastic trip collars that
grip the head's ring by friction, each with a vertical trip pin. The red pin starts 200 deg round from the blue one
(counter-clockwise, seen from above) and has to end 90 deg round (plus or minus 5 deg). The head turns on its bearing
at 0.004 N m, while a collar slips at 0.06 N m, so the head turns more easily than a collar slips. At the end the pins
stand 90 deg apart and still, and both grippers have let go. Landscaping and groundskeeping workers (BLS OEWS, SOC
37-3011, about 0.9 million in the US) set sprinkler arcs as a regular duty.

The ability is setting the angle between two friction collars on a free-spinning body by holding the body against its
own bearing. Nothing latches, nothing is placed, and no thread or snap fit is involved.

Physics, not the instruction, defeats the natural plan, and each arm alone fails. A collar pushed round by one arm
carries the free head and the other collar with it, because the head's bearing gives way first (0.09 N on a pin
against 1.3 N for the collar), so the angle between the pins never changes. The other arm alone can hold the head but
has nothing to push. The two pins also start 88.6 mm apart axis to axis (96.6 mm across their outer sides), wider than the
94 mm opening, so one gripper cannot pinch them together, and a pinch would only widen the arc. One arm holds the head tube still; the
other slides one collar round by its pin.

Instruction given to the policy (23 words; goal only):

`Set the sprinkler's trip collars so the blue pin stands 90 degrees counter-clockwise from the red pin, seen from above, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo
and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task.py sha 5199357a114b (`sha256sum task.py | cut -c1-12`)

## Offline check, no simulator

```bash
pytest -q tests        # 14 scorer tests, pure Python (yam_arena must be importable for TABLE_TOP)
```

## Run and score one Dream-1 trial

Placeholders for the filing run, to fill from the log: Dream-1 run at 0059d02a1a91, task 5199357a114b, 1200 steps
(9 minutes), log adhoc_0f604528.json, score 0.0, max_steps.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant left_only      # a cheat variant (or DEMO_VARIANT=left_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json`. The script reads only link poses (what a camera could measure) and the
robot's own grippers; it never reads joint angles or the record.

1. Left arm: comes in from the left, pads closing along x on the 32 mm head tube 122 mm above the soil, closes to a
   28 mm opening and holds the head still.
2. Right arm, in parallel once the head is held: top-down over the red pin (pads closing across the pin, 30 mm open),
   nulls the arm's sag from the seen pad centre (under 1 mm), lowers onto the pin, closes to 7.4 mm, then swings the pin
   counter-clockwise about the sprinkler axis in steps of at most 12 deg with the wrist following, reading the angle
   between the pins from the two pins' poses after every step until it is within 0.7 deg of 90 (the angle comes in
   at 90.08 deg).
3. Right lets go and lifts 80 mm; then left opens and backs away 100 mm; both wait until everything is still.

The demo scores 1.0 (`success`, step 887 of 1,200). The arc goes from 200.0 to 90.08 deg, the head turns 1.6 deg in the
left grip, the blue collar does not move, and the grippers end 70 and 45 mm clear. Worst IK error 0.09 mm and 0.01 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file; every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; the arc stays 200.0 deg. | 0.0 |
| `left_only` | Only the left arm holds the head tube, then lets go. | The arc stays 200.0 deg. | 0.0 |
| `right_only` | Only the right arm swings the red pin counter-clockwise, 108 deg of sweep in steps, head free. | The head and both collars turn round together (head 108 deg), the arc stays at 200.8 deg. | 0.0 |
| `wrong_way` | Both arms, but the red pin is pushed clockwise. | The arc opens to 228.3 deg. | 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Whole-run record (a linkkit History fed every control step and cleared only by the simulator reset). arc is the angle
from the red pin counter-clockwise to the blue pin about the sprinkler axis, from the two pins' positions.

| Score | Condition |
|---|---|
| 0 | none of the rungs below |
| 0.25 | the arc was at some time 160 deg or less (closed 40 deg), but at the end it is not within 20 deg of 90 |
| 0.5 | at the end the arc is within 20 deg of 90 (70 to 110) |
| 0.75 | at the end the arc is within 5 deg of 90 (85 to 95); not yet still 1 s or not yet released |
| 1.0 | also the arc has been inside 85 to 95 and moving under 0.5 deg/s for 1 s, and every pad point of both grippers is 30 mm or more from the sprinkler and both grippers move under 5 mm/s for 1 s (release gate) |

Real-trial measurement: a printed 360 deg disc under the ring and a top photo (or a protractor on the ring) give the
angle between the pin axes, 1 deg division; the arc is still if the pins move under 0.5 deg in 1 s (a mark on the disc); clearance by a 30 mm spacer
card passed around each gripper and the sprinkler.

## Run limit, runtime and seeds

1,200 steps (1 minute at 20 Hz). The demo finishes at step 887. The scene is deterministic; there is no random seed
(`object_jitter=0`).

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 887.
- Dream-1 run: score 0.0, max_steps, 1200 steps, 9 minutes, log adhoc_0f604528.json.

## Materials

- Soil tray with spike and riser: one fixed body, 6.0 kg in the simulation (the tray block is 200 x 200 x 30 mm; the
  brass riser is 28 mm dia and 50 mm tall above the soil; the spike is buried), at (0.150, 0.000) on the table.
- Swivel head: brass tube 32 mm dia and 84 mm tall (soil +50 to +134 mm), 38 mm cap, 0.12 kg, on a vertical bearing that
  turns at 0.004 N m (0.09 N tangent at a pin). Total height above the soil 138 mm. Bench check: a 0.5 N spring scale
  hooked on a pin turns the head at 0.09 N, plus or minus 0.03 N.
- Red trip collar: plastic ring 49 mm dia and 10 mm tall, a radial arm 30 x 10 x 6 mm and a vertical pin 8 mm dia,
  its axis 45 mm from the sprinkler axis, 0.02 kg. It slips on the head at 0.06 N m (1.3 N on the pin; spring scale
  1.3 N, plus or minus 0.2 N, pulled tangent to the pin).
- Blue trip collar: the same, ring 14 mm higher, pin 4 mm shorter at its foot (both pins end 90 mm above the soil).
- Positions and colours for every body are in `task.json` (setup) and at the top of `task.py`.

## Fidelity

Parts are close to full size; the pins stand 45 mm from the axis, a little farther than a typical trip lever. The
sprinkler is simplified: no nozzle, swing arm, spring or water. Each collar's grip is a joint friction of 0.06 N m
(a real collar grips by a clamped ring or O-ring), with some added joint inertia and damping so the friction does not
chatter. The spike and riser are fixed in a rigid soil block. The trip pins trip nothing.

## Reset between trials

Nothing is consumed. Slide the red collar so its pin is at azimuth -150 deg (toward the robot, to its right) and the
blue collar so its pin is 200 deg counter-clockwise of it, using the printed disc. Check the head turns with 0.09 N on a
pin and each collar slips at 1.3 N. The tray is fixed on a taped outline.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
