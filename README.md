# rover

**Goal:** give it a pose (x, y, θ) — from the VLM brain, a map click, or a
command — and it gets there precisely, knowing at every moment where it is, and
moving safely through tight spaces because it knows its own shape.

Home and office first, a parking assistant later, an arm to pick things up after
that. Built one tested layer at a time; every claim below is measured.

> **Just powered everything on?** Follow **[STARTUP.md](STARTUP.md)**.

## Where it stands (2026-09-26)

| | what | status |
|---|---|---|
| **Shape** | measured dimensions → URDF + CAD, one source | ✅ [description/](description/README.md) |
| **M1 Calibrate** | camera ↔ LiDAR, camera tilt, lever arm, gyro | ✅ camera within 4 mm of two independent methods; gyro scale 1.001 |
| **M2 LiDAR odometry** | the pose from the walls, every scan | ✅ worst 0.9 cm / 0.39° over 17 runs; ~1 cm vs the owner's tape marks |
| **M3 Fusion** | gyro + LiDAR + VO + wheels, each weighted by its own evidence | ✅ **live**, owns `/odom`: worst 0.8 cm / 0.45° over 19 runs (Phase 1 fusion: 20.5 cm / 13°) |
| **M4 Motion control** | exact x, y, θ moves closed on the pose; firmware fixes | ✅ **live**: four-goal loop closes within 1.4 cm / 0.8° vs LiDAR truth (`./rover goto`) |
| **M5 Safe navigation** | nav2 on the fused pose, LiDAR + depth obstacles, 5 cm margin, collision monitor | nav2 drives on fusion2; nav2 route + goal_exec finish reached 1.3 cm (`./rover navto --exact`); gaps and objects next |

Numbers and how each was measured: [LOCALIZATION.md](LOCALIZATION.md).
What is still missing, measured: [LOCALIZATION_GAPS.md](LOCALIZATION_GAPS.md).
The roadmap: [SENSOR_FUSION_PLAN.md](SENSOR_FUSION_PLAN.md) §8.

## The repo

| where | what |
|---|---|
| `rover` | the launcher: one layer at a time, each gated on the one below |
| `description/` | **the rover's shape**: `params.yaml` (every measurement, with status and date) → `rover.urdf`, `cad/rover.step`; `description/build` also checks every other copy of those numbers |
| `phase1/nodes/` | the pose: `vo_node` (cuVSLAM), `gyro_node`, `lidar_odom` (+ node), `fusion2` (+ node), `health`, the calibration tools (`floor_calib`, `cam_lidar_calib`) |
| `phase1/harness/` | **the test harness**: `./rover record <scenario>` drives or records a run, `./rover grade` scores every estimate against LiDAR truth |
| `phase1/firmware/` | ESP32: motors, encoders, telemetry |
| `lidar/` | the RPLidar C1 driver build and its calibration (yaw by driving, timestamp lag) |
| `phase2/` | mapping: nvblox (depth → obstacles 5-27 cm), slam_toolbox, RViz |
| `phase3/` | nav2 (costmaps: nvblox + LiDAR), `./rover drive` exact moves |
| `phase4/` | the VLM bridge: pixel → nav2 goal |
| `docs/archive/` | history: the phase logs, the old TODO and READINESS, the knowledge notes |

The Pi 5 brain (LangGraph, voice, teleop) is a separate repo: `~/ros2_ws` on the Pi.

## Everyday commands

```bash
./rover up                         # everything, cold (see STARTUP.md for the layers)
./rover status                     # what is alive, and whether the pose is honest

./rover record return              # move it by hand, put it back on the tape: all ~0
./rover grade                      # the newest run vs LiDAR truth

./rover camera --floor             # camera tilt and height off the floor (moves nothing)
./rover camera --lidar [--solve]   # camera vs LiDAR, pose by pose (moves nothing)
./rover lidar --calibrate          # LiDAR mount yaw, by driving
description/build                  # after any measurement change: URDF, CAD, drift check

./rover goto X Y [DEG]             # an exact move, relative (DRIVES); --map / --odom for frames
./rover navto X Y DEG [--exact]    # nav2 plans the route; --exact: goal_exec finishes to ~1 cm (DRIVES)
```

[OPERATIONS.md](OPERATIONS.md) has the rest and the troubleshooting;
[ARCHITECTURE.md](ARCHITECTURE.md) the topics, frames and nodes.
