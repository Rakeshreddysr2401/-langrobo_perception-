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

## What is NOT done yet — the map frame

nav2 still plans in `odom`; the Pi 5 brain still sends goals in `odom`
(`LANGROBO_NAV_FRAME`). So today the correction is *visible* (RViz, `--check`)
but nav2 does not *use* it. Moving nav2's global frame and the brain's
`NAV_FRAME` to `map` is the step that makes "go to this point" accurate — and
it touches two repos and the "everything is odom, never map" rule in the Pi 5's
CLAUDE.md, which exists because a `map` frame nobody published broke
`navigate_to_pose` silently for months. Do it as a driven change, with a tape
measure, not as an edit.

After that: `--save` the finished room, and start `slam_toolbox` in
`localization` mode against it (`mapper_params_localization.yaml` in the
package) — same node, second mode, the "exact physical map" the owner asked for.

## Orientation

`LIDAR_YAW` in `./rover` must be right before any of the above is trusted: a
scan rotated 90° from the body fights odometry on every move. The box test
(`./rover lidar`, box in front of the nose, "front" must drop) sets it. The
depth camera could not referee it — the IR emitter is off for cuVSLAM, and
passive stereo returned almost no depth in this room.
