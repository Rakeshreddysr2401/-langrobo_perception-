# slam — the lidar in the pose stack

## What it does, in one picture

```
 wheels ─┐
 gyro   ─┼─► fusion_node ──► odom -> base_link      (dead reckoning: fast, drifts)
 cuVSLAM┘
 lidar /scan ─► slam_toolbox ──► map -> odom         (the correction: what the
                                  /map                 walls say dead reckoning
                                                       got wrong)
                 map -> base_link = the corrected pose
```

`fusion_node` is good at *motion* — it picks the sensor least likely to be
lying for heading and distance. It has no way to notice accumulated error.
`slam_toolbox` matches every scan against the walls it has already seen, so
error is measured against the room and pushed into `map -> odom`. Nothing
under it changes: `odom -> base_link` is still `fusion_node`'s.

## Run

```
./rover lidar            # /scan + base_link -> laser
./rover fused            # (needs ./rover pose) odom -> base_link
./rover slam             # map -> odom + /map; prints the correction so far
./rover slam --check     # the correction right now — re-run after driving
./rover slam --save room # maps/room.posegraph + room.pgm/.yaml, for later
./rover view             # RViz on the laptop: 'Lidar' points + 'Lidar map'
```

`./rover status` has a `slam` row (`/map` ≥ 0.3 Hz).

## What "working" looks like

Drive a zig-zag and two pivots, then `./rover slam --check`. The `correction`
row is what dead reckoning got wrong; the `lidar-corrected` row should put the
rover where it really is. Pivots are the case that matters — that is where the
gyro's bias and cuVSLAM's +11 % used to show — and where the walls do not move.

## One owner of `map -> odom`

cuVSLAM publishes `map -> odom` whenever `vo_node` runs with `slam:=true`,
which is `./rover pose`'s default. Two publishers of one transform make TF
non-deterministic, and these two disagree by construction. slam_toolbox owns
it: `./rover up` starts pose with `SLAM=false`, and `./rover slam` refuses to
start while cuVSLAM holds the frame, printing the hand-over:

```
SLAM=false ./rover pose && ./rover fused && ./rover slam
```

(`SLAM=false` silently did nothing until 2026-09-22 — it was expanded inside the
container, where it was never set. Fixed.)

## Timestamps

slam_toolbox takes the pose at each scan's stamp, so the stamp must be when
the scan was measured. The C1 driver's stamps were ~82 ms early; on turns that
gave slam an 8° heading error over two same-direction +90° turns. Fixed with a
driver patch and `LIDAR_TIME_OFFSET=0.082`; `./rover lidar --lag` re-measures
it. See [lidar/README.md](../../lidar/README.md).

## What is NOT done yet — the map frame

nav2 still plans in `odom`; the Pi 5 brain still sends goals in `odom`
(`LANGROBO_NAV_FRAME`). The correction is *used* by one consumer so far —
`./rover drive` steers on `map -> base_link` (LOCALIZATION.md §4) — but not by
nav2 or the brain. Moving nav2's global frame and the brain's
`NAV_FRAME` to `map` is the step that makes "go to this point" accurate — and
it touches two repos and the "everything is odom, never map" rule in the Pi 5's
CLAUDE.md, which exists because a `map` frame nobody published broke
`navigate_to_pose` silently for months. Do it as a driven change, with a tape
measure, not as an edit.

After that, *if* a room is ever to be kept: `--save` it and start
`slam_toolbox` in `localization` mode against it
(`mapper_params_localization.yaml` in the package). **The owner has said not to
keep maps across power-off** — each power-on maps fresh and the origin is
wherever the rover starts — so this is recorded, not planned.

## Orientation

`LIDAR_YAW` must be right before any of the above is trusted: a scan rotated
from the body fights odometry on every move, and slam turns any residual into a
correction that rotates with every heading change. It is **measured, +88.60°**
(`./rover lidar --calibrate`, by driving — see lidar/README.md). The box test in
`./rover lidar` is a quadrant check only; two values set by eye were both wrong.
