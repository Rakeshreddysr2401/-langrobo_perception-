# lidar — RPLidar C1 on the rover

| | |
|---|---|
| sensor | Slamtec RPLidar C1 — 360°, 10 Hz, 720 beams (0.5°), 0.05–16 m, 5 kHz samples |
| link | USB, Silicon Labs CP2102N bridge → `/dev/ttyUSB0`, 460800 baud |
| mount | top of the rover, directly above the camera, 3.5 cm behind its lens, centred; window ≈21 cm off the floor |
| frame | `laser`, child of `base_link` (x 0.1342, z 0.2498 from description/params.yaml, measured 2026-09-24; **yaw +1.5463 rad / +88.60° measured**) |
| timing | driver stamps shifted **+82 ms** (`LIDAR_TIME_OFFSET`), measured — see below |
| topic | `/scan` (`sensor_msgs/LaserScan`) |
| driver | `sllidar_ros2` (Slamtec's own), pinned in `build.sh`, built once into `ws/install` |

```
lidar/build.sh             # one-off: clone + patch + compile inside the image
./rover lidar              # start the layer; rate, the frame, and the rover's 4 directions
./rover lidar --calibrate  # MEASURE LIDAR_YAW by driving (moves 0.20 m legs)
./rover lidar --watch      # live: where the nearest object renders on the rover
./rover lidar --yaw        # one-shot yaw from a target held at the nose
./rover lidar --lag        # MEASURE the scan timestamp error (short turns)
./rover logs lidar         # the driver's log
```

| file | what it is |
|---|---|
| `build.sh` | clones `sllidar_ros2` at a pinned commit, applies `patches/`, builds in the image |
| `patches/0001-scan-time-offset.patch` | adds a `scan_time_offset` parameter (default 0 = upstream behaviour) |
| `scan_check.py` | the gate: one scan, the frame resolves, the rover's four directions |
| `yaw_calibrate.py` | `LIDAR_YAW` from how the room slides while driving out and back |
| `yaw_find.py` | `LIDAR_YAW` from a target at the nose; `--watch` for the live readout |
| `scan_lag.py` | stamp error from registering mid-turn scans against a still one |
| `pivot_test.py` | grades turns against the walls (`./rover pivot`) — see LOCALIZATION.md |

## LIDAR_YAW — measured by driving, not by eye

Which way the C1's zero beam points is not marked on the case. Two values set
from descriptions of the RViz picture — 0.0, then −1.5708 — were **both wrong,
with the sign inverted**. A quarter-turn error looks the same from several
descriptions, and the sign flips easily between eye and file.

`./rover lidar --calibrate` takes people out of the loop. Drive straight a
known distance and every fixed point in the room slides by the same vector in
the laser frame; its direction is the mount angle
(`LIDAR_YAW = alpha − atan2(t)`, derivation in `yaw_calibrate.py`). Out-and-back
pairs cancel drive curvature, which biases the two legs oppositely.

**Measured 2026-09-22: +1.5463 rad (+88.60°)** — five usable legs, spread
0.76°, ICP residuals 0.5–0.7 cm, and the LiDAR's and odometry's distances for
the same move agreeing to 1%. That is 1.4° off a clean quarter turn and larger
than the spread, so it is real skew: do not "tidy" it to π/2.

## Timestamps — the driver stamped scans ~82 ms early

`sllidar_node` reads `now()` *before* `grabScanDataHq()` blocks for the next
full revolution, so the stamp leads the measurement. slam_toolbox looks the pose
up at the stamp, so during a turn every scan was matched against a pose ~0.6°
away, and two same-direction +90° turns left slam with an 8° heading error.

`./rover lidar --lag` holds still for a reference scan, turns each way, and
compares each mid-turn scan's rotation (registered against the reference) with
`/odom`'s yaw at the scan's stamp. A stamp error τ shows as ω·τ, with the sign
following the turn. **+68 / +94 / +85 ms** over three runs, no fixed bias.
`patches/0001-scan-time-offset.patch` adds `scan_time_offset`, `./rover lidar`
passes `LIDAR_TIME_OFFSET=0.082`, and it re-measured at **−2 ms**.

If the driver is ever re-pinned or the C1 replaced, re-run `--lag` and
`--calibrate`: both numbers belong to this unit and this driver.

`ws/src`, `ws/build`, `ws/install`, `ws/log` are not in git — `build.sh`
recreates all four. The image is never touched.

Verified 2026-09-22 on the bench: firmware 1.02, health OK, 9.97 Hz, 92 % of
beams valid indoors. After a power cycle on 2026-09-23: 10.0 Hz, 95 % valid.
