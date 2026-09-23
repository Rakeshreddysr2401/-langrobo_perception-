# phase1/harness — record once, grade many (SENSOR_FUSION_PLAN.md §5, stage A)

```
./rover record SCENARIO [--dist M] [--wz RAD_S] [--v M_S] [--secs S] [--images] [--note TEXT]
./rover grade [RUN ...]          # default: the newest run
./rover grade --summary          # every graded run -> logs/bags/SUMMARY.md
```

| file | what |
|---|---|
| `run.py` | drives one scenario on RAW sensors (turns on the gyro, straights on the four encoders), bags everything, writes `scenario.json` |
| `grade.py` | LiDAR truth at every still moment, then each estimate's error against it; `--self-test RUN` checks the truth itself |
| `common.py` | bag reading, still detection, point-to-line ICP with its own uncertainty |

Runs land in `logs/bags/<local time>_<scenario>/`, which git ignores. Copy the
numbers that matter into the docs with the run's name.

## How the truth works, and why it can be trusted

The rover stops for 3 s after every move. While it is still, the scans in
that window are merged by taking each beam's median, and the result is matched
point-to-line against the scan from the first still moment. That gives where
the rover really is relative to its start, measured from the room's walls with
**no odometry involved**. A checkpoint is only used if:

- the match is tight: median residual ≤ 2 cm and ≥ 150 points on lines;
- the room constrains every direction (not corridor-degenerate);
- two independent starting guesses converge to the same answer (1 cm, 0.3°).

Checked on 2026-09-24:

- **noise floor:** a 12 s still run graded 0.0 cm / 0.01° on both checkpoints;
- **self-test:** real scans moved by known amounts, up to 1 m and 180°, were
  recovered within 0.03 cm / 0.01°, starting from guesses 5 cm / 4° off.

What the self-test cannot check is a genuinely different viewpoint, where
walls get occluded or new ones appear. The residual and two-guess gates cover
that on driven runs. A checkpoint that fails them is printed as unusable, never
graded.

## Scenarios

| scenario | moves | what it measures |
|---|---|---|
| `straight` | forward `--dist` (1.0 m), back | distance scale, straight-line drift |
| `pivot90` | +90 ×4, then −90 ×4 | turn accuracy both ways; the pivot slide (the truth's x,y at each stop) |
| `pivot360` | +360, −360 | gyro scale, accumulated turn error |
| `square` | (forward 0.5 m, +90) ×4 | the realistic mix; returns near the start |
| `small` | ±10°, ±5°, ±5 cm | the edge moves: deadband, start/stop |
| `still` | nothing for `--secs` | drift while parked; walk past it for the people case |
| `manual` | you drive or push it for `--secs` | rug edge, threshold, lift-and-place, spinning wheels |

`--images` also records stereo IR (~25 MB/s). Use it only for runs meant to
re-run cuVSLAM variants (stage D).

## A baseline session (stage A's exit), about 30 minutes

The stack has to be up **with the 2026-09-24 geometry**, so restart the layers
first: `./rover lidar`, `./rover pose`, `./rover fused`, `./rover slam`. The
teleop has to be in AUTO. Clear a ~1.5 m circle and put the rover on the X.

1. `./rover record still --secs 30`
2. `./rover record straight`
3. `./rover record pivot90`
4. `./rover record pivot360`
5. `./rover record square`
6. `./rover record small`
7. `./rover grade --summary`

That table is the bar every later stage has to beat (§5 acceptance): end
point ≤ 3 cm on the out-and-back, ≤ 5 cm after the square, heading ≤ 1° after
360°, and never worse than today's `fused` on any scenario.
