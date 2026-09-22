# lidar — RPLidar C1 on the rover

| | |
|---|---|
| sensor | Slamtec RPLidar C1 — 360°, 10 Hz, 720 beams (0.5°), 0.05–16 m, 5 kHz samples |
| link | USB, Silicon Labs CP2102N bridge → `/dev/ttyUSB0`, 460800 baud |
| mount | top of the rover, directly above the camera, 3.5 cm behind its lens, centred; window ≈21 cm off the floor |
| frame | `laser`, child of `base_link` (x 0.135, z 0.21, yaw **not yet measured** — see below) |
| topic | `/scan` (`sensor_msgs/LaserScan`) |
| driver | `sllidar_ros2` (Slamtec's own), pinned in `build.sh`, built once into `ws/install` |

```
lidar/build.sh       # one-off: clone + compile inside the image, output on the host
./rover lidar        # start the layer; prints rate + front/left/back/right ranges
./rover lidar --yaw  # MEASURE LIDAR_YAW off a target held at the nose
./rover logs lidar   # the driver's log
```

## LIDAR_YAW is not yet measured — do this before trusting slam

`LIDAR_YAW` is still at its placeholder `0.0`. Which way the C1's zero beam
points is not marked on the case, and the quadrant test in `./rover lidar`
("front must be the one that drops") only answers it to the nearest 90°. That
is not enough: a scan 10° off the body fights odometry on every move, and
`slam_toolbox` pushes that error straight into `map -> odom`.

`./rover lidar --yaw` measures it. Put a flat target — a box, a book — about
50 cm **directly in front of the nose**, with nothing else inside 1.5 m, and
run it. It averages 10 scans, finds the nearest cluster, and prints the
`LIDAR_YAW` that puts that cluster at 0°.

It refuses to give a clean answer if the spread across those 10 scans is over
5°, which means it is jumping between objects rather than holding one target.
Clear the area and re-run. Confirm the answer by repeating with the target off
to the LEFT instead — the printed `LIDAR_YAW` must not move.

Then set it permanently in `./rover` (the `LIDAR_YAW=${LIDAR_YAW:-...}` default),
not just in the environment — two processes read that file and only one would
get an exported value.

`ws/src`, `ws/build`, `ws/install`, `ws/log` are not in git — `build.sh`
recreates all four. The image is never touched.

Verified 2026-09-22 on the bench: firmware 1.02, health OK, 9.97 Hz, 92 % of
beams valid indoors.
