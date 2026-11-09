# Roll a maki with a bamboo mat, seam side down, then open the mat

A bimanual food-preparation task with a hinged tool. A bamboo sushi mat (makisu, 24 slats strung side by side) lies
on a maple butcher block. On it is a sheet of nori spread with rice, with a green cucumber pressed into the rice
along the sheet's near edge. The robot must use the mat, not its fingers, to roll the sheet round the cucumber, leave
the sheet's far edge underneath (seam side down), lay the mat open and flat again and let go.

The mat is a chain of free hinges, so pushing or dragging it does nothing useful. It rolls the maki only when both
arms lift its near edge, curl it up behind and over the cucumber, and then feed it forward so the roll turns inside
the curl and winds the sheet onto itself. The edge must be kept lifted off the front of the roll, or the mat is
rolled in with the food. The roll must stop when the far edge reaches the bottom: stop early and the seam is still
open on the mat, roll on and the seam climbs the back. The mat must be peeled off the roll, not slid over it, or it
drags the roll back and unwinds the sheet. A pad that touches the food zeroes the run, as kitchen food codes require.

## Files

| File | What |
|---|---|
| `task.py` | yam_arena task file: objects, whole-run history, `score()`, `success()`, `distance()`, the scripted demo and its cheat variants |
| `linkkit.py`, `linkdemo.py` | our articulated-link kit and closed-loop demo runner (MIT) |
| `task.json` | intake fields |
| `test_bamboo_mat_roll.py` | 26 scorer edge-case tests (including a reach check of the demo), no simulator needed |

## Run a trial

After `source /data/IsaacLab-Arena/yam.env`, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Use the bamboo mat to roll the rice-covered nori sheet round the green cucumber, seam side down, then open the mat flat and let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir runs/dream
```

## Scripted demo

```bash
python linkdemo.py task.py demo_out --max-steps 1200                    # the demo
python linkdemo.py task.py out_hand --max-steps 1200 --variant hand     # a cheat variant
pytest -q test_bamboo_mat_roll.py                                       # scorer edge cases
```

The demo is closed loop on camera-observable state only (`ctx.observe()`: link and object poses, no joint readouts):

1. Before touching anything, it reads the sheet's far edge, which lies flat in the top view.
2. Both arms take the two overhanging ends of slat 0 in a side pinch.
3. Fold: slat 0 follows an involute about the cucumber, curling the mat up its back and over its top.
4. Roll: the curl is fed forward at 8 mm/s (4 mm/s over the last 10 mm). The feed stays at most 3 mm
   ahead of the cucumber as seen and never chases a roll that runs ahead. Slat 0 rides 10 mm high, so the mat leaves the
   roll at its top. It stops when the cucumber reaches the far edge (its end faces stay in view from the side).
5. Open: the fold run backwards, an involute about the roll where it now lies, so the mat peels off the roll instead
   of sliding over it and dragging it back. Slat 0 ends flat where it started (slat 23's seen position minus 230 mm).
   The pads slide off and the arms go to ready.

## Cheat variants (run in the simulator, `--variant NAME`)

| Variant | What it does | task_score | Reason |
|---|---|---|---|
| nothing | arms stay at ready | 0.0 | wrap 63 deg < 180 (the strips under the cucumber only) |
| push | closed fingers roll the cucumber 40 mm forward | 0.0 | a gripper touched the food at 5.70 s |
| hand | both closed hands push the cucumber over the sheet from behind, no mat | 0.0 | a gripper touched the food at 6.10 s |
| drag | pinch slat 0 and slide the whole mat 50 mm toward the robot | 0.0 | wrap 63 deg < 180 |
| lift | pick the cucumber up 30 mm and put it back | 0.0 | cucumber lifted 30 mm |
| hold | the full demo, then both arms take slat 0 again and keep holding it | 0.0 | grippers not clear and still for 1 s (the roll itself ended as in the demo: wrap 343 deg, seam down) |

## Scoring (`score()` at the end of the run; `success()` = 1.0 held 1 s)

Gates first, each 0, read from an `lk.History` that is updated every control step and cleared only by the
simulator reset:
- the cucumber was ever lifted more than 20 mm;
- it ever left the block (x outside 0.25 to 0.52 m or |y| > 0.06 m);
- a gripper pad ever came within 4 mm of the rice, nori or cucumber;
- when the sheet first covered 180 deg of the cucumber, the mat had not yet been curled 120 deg (the sum of its
  hinge angles), so the mat did not do the rolling;
- the grippers have not been 30 mm clear of the mat and the food, with the grippers, the mat and the roll's centre
  still (< 0.01 m/s), for the last 1 s;
- the simulation went unstable (non-finite poses).

Ladder, on the state at the end:

| Score | Condition |
|---|---|
| 0.25 | the sheet covers at least 180 deg of the cucumber |
| 0.5 | at least 300 deg |
| 0.75 | at least 330 deg, the outer edge on the roll within 60 deg of straight down, the cucumber square within 10 deg |
| 1.0 | also the mat open (every hinge <= 20 deg, every slat <= 10 mm above flat) with the roll on it |

Wrap is the arc covered by the longest run of consecutive 8 mm strips lying 12.5 to 17.5 mm from the cucumber's
axis. A sheet that is wound and then unrolled keeps no credit.

## Scored runs

| Run | yam-isaac-lab-arena | Commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | git checkout | 0059d02a1a91 | 522719a33075 | 1200 | 7 min | adhoc_6e52c8db.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | git checkout | 0059d02a1a91 | 522719a33075 | 733 (success held 1 s) | about 8 min | result.json | 1.0 | rolled seam down, mat open, released |
| Scripted demo, repeat (`--no-record`) | git checkout | 0059d02a1a91 | 20b9f6900af9 | 733 (success held 1 s) | about 10 min | result.json | 1.0 | rolled seam down, mat open, released |

## Versions, runtime and seeds

yam-isaac-lab-arena at commit 0059d02a1a91, inspect-robots 1.0.0, inspect-robots-yam 0.38.0, isaaclab_arena 0.3.0,
Isaac Sim 6.1.0.0. The layout is fixed (`object_jitter=0`, seed 0). The demo reaches success at step 733 (36.7 s);
the two demo runs and the demo part of the `hold` variant ended in the same state to 0.1 mm and 0.1 deg. On a
shared L40S it ran at 1.6 to 2.2 control steps/s, plus about 2 minutes of start-up. The two task.py hashes differ
only in one docstring.

## Materials

| Prop | Real item | Size | Mass | Colour | Position (world, m) |
|---|---|---|---|---|---|
| Block | oiled end-grain maple butcher block on rubber feet | 280 x 190 x 100 mm | 3.7 kg (fixed) | light brown | x 0.245 to 0.525, centred on y 0 |
| Mat | bamboo makisu, 24 slats bound with cotton string | 240 x 240 mm; slats 9 x 240 x 3 mm every 10 mm | 108 g | alternating tan | near edge x 0.2655; slat ends overhang the block 25 mm |
| Sheet | nori spread thinly with sushi rice, cut to one turn round the cucumber | 190 x 87.6 x 1.7 mm | 33 g | white rice on top, dark green nori under | x 0.3312 to 0.4188 |
| Cucumber | Japanese cucumber, ends trimmed | 26 mm dia x 180 mm | 95 g | green | axis along y at x 0.335, on the sheet's near edge |

The sheet is 11 rigid 8 mm strips (0.5 mm nori under 1.2 mm rice) hinged with a weak spring to flat. It is cut to
one turn, so its edges meet without overlapping. The near-edge strip ends in a 31 deg ramp, which stands in for the
soft rice at a sheet's edge. The cucumber is welded to that strip, which stands in for the rice tack that holds a
filling in place; elsewhere tack is friction only. The roll's rolling resistance is angular damping of 150/s on the
cucumber: a rigid cylinder in PhysX has none, and a soft rice roll on a slatted mat has plenty. All colliders are
kit primitives (boxes and a cylinder); there are no third-party meshes, textures or assets.

## Reset notes

Block flush with a tape line 24.5 cm in front of the arm-base line, centred. Lay the mat flat with its slats across
the table and its near edge on a tape mark at x 26.55 cm. Spread a fresh sheet, cut to 8.76 cm, with rice and lay it
rice side up, centred, with its near edge on a tick at x 33.12 cm. Press a trimmed cucumber into the rice along the near edge,
its axis over a tick at x 33.5 cm. Use fresh food for every trial.

## Licence

`task.py`, `test_bamboo_mat_roll.py`, `linkkit.py` and `linkdemo.py` are our own code, MIT-licensed (header in each
file). Nothing is copied from cablekit, partkit or scenekit.
