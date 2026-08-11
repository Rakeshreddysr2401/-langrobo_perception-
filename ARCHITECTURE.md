# ARCHITECTURE — every block, and what goes in and out

---

## 1. Frames — the idea that explains most confusion

```
map ──────────> odom ──────────> base_link ──────────> camera0_link
      ▲                 ▲                     ▲
   cuVSLAM            EKF              static TF (measured:
 (jumps when      (smooth, but         x 0.10, z 0.163)
  it recognises    drifts slowly)
  a place)
```

| Frame | Means | Behaviour |
|---|---|---|
| `map` | the world, corrected | **accurate but jumps** at loop closure |
| `odom` | dead-reckoned world | **smooth but drifts** — never jumps |
| `base_link` | the rover itself | origin on the **ground**, x forward |
| `camera0_link` | the camera | 10 cm forward, 16.3 cm up |

**Why two world frames instead of one.** A controller cannot cope with the robot
teleporting mid-manoeuvre, so it needs `odom`. A goal must not drift away over
ten minutes, so it needs `map`. You cannot have both properties in one frame, so
ROS uses two and keeps the difference in the `map → odom` transform.

**`base_link`'s origin is on the floor.** That is why open floor must deproject
to z ≈ 0.000, and why a 3.7 cm error in the camera height mapped the floor as a
wall.

---

## 2. The blocks

### `realsense2_camera` — the eyes

| | |
|---|---|
| **In** | the D555 over DDS/ethernet at `192.168.11.55` |
| **Out** | `/camera/camera0/infra1/image_rect_raw` (left IR, 896×504@30)<br>`/camera/camera0/infra2/image_rect_raw` (right IR)<br>`/camera/camera0/depth/image_rect_raw`<br>`/camera/camera0/motion/*` (gyro + accel)<br>a `camera_info` beside each |

Two flags are load-bearing and must not be "tidied": `enable_sync:=false` and
`depth_module.emitter_enabled:=0`. See `FACTS.md §1`.

### `static_transform_publisher` — the tape measure

| | |
|---|---|
| **In** | nothing; fixed numbers |
| **Out** | TF `base_link → camera0_link`, x 0.10, z 0.163 |

### `cuvslam_node.py` — *where am I?*

| | |
|---|---|
| **In** | `infra1/image_rect_raw` + `infra2/image_rect_raw` — the **stereo IR pair**<br>`infra1/camera_info` + `infra2/camera_info` |
| **Out** | `/odom` — position and orientation<br>`/visual_slam/tracking/odometry` — same data<br>`/slam/status` — includes `slam_pose_ok`<br>TF `map → odom`<br>TF `odom → base_link` *only if* `publish_odom_tf:=true` |

Finds features in the left image, matches them to the right to get depth by
triangulation, then tracks them over time — feature motion becomes camera
motion. Keeps up to 300 keyframes so it can recognise a place and correct drift.

**It uses IR, not colour, and not the depth image.**

At L3 it is relaunched with `publish_odom_tf:=false`, because the EKF takes over
`odom → base_link`. Two publishers of one transform fight and neither wins.

### `imu_to_base.py` — rotate the gyro

| | |
|---|---|
| **In** | `/camera/camera0/motion/*` — gyro in the camera's frame |
| **Out** | `/imu/base` — the same gyro expressed in `base_link` |

### `ekf_filter_node` (robot_localization) — the pose in charge

| Input | Topic | What is actually taken |
|---|---|---|
| `odom0` | `/odom` (cuVSLAM) | **x, y, yaw** |
| `odom1` | `/wheel_odom` (encoders) | **vx only** |
| `imu0` | `/imu/base` (gyro) | **yaw rate only** |

| | |
|---|---|
| **Out** | `/odometry/filtered` and TF `odom → base_link` |

Three deliberate choices:

- **`two_d_mode: true`** pins z, roll, pitch to zero. Without it cuVSLAM reported
  `z = -0.272 m` and the map smeared.
- **The accelerometer is not fused at all** — it diverges by hundreds of metres.
- **Wheels give speed, not heading** — wheels slip in pivots; the gyro is trusted
  for rotation.

### `nvblox_node` — *what does the world look like?*

| | |
|---|---|
| **In** | `/camera/camera0/depth/image_rect_raw` + `camera_info`<br>**TF `odom → base_link`** ← the dependency that matters |
| **Out** | `/nvblox_node/static_occupancy_grid` — what RViz draws<br>`/nvblox_node/static_map_slice` — what nav2's costmap eats<br>`/nvblox_node/static_esdf_pointcloud`<br>`/nvblox_node/mesh` |

Three steps:

1. **TSDF** — each depth pixel becomes a 3D point *using the pose*, written into
   5 cm voxels storing "how far to the nearest surface".
2. **ESDF** — a second grid storing *distance to nearest obstacle*, far cheaper
   for a planner to query than raw geometry.
3. **2D slice** — nav2 cannot use 3D, so a horizontal band (0.12–0.40 m) is
   flattened to a 2D grid.

**This block inherits every pose error.** It does not see a room; it writes depth
wherever the pose claims the robot is.

### nav2 — decide and move

```
goal (RViz "2D Goal Pose", in the map frame)
   │
   ▼  bt_navigator            global_frame: map
   ▼  planner_server          NavfnPlanner  ──> /plan   (the green line)
   ▼  controller_server       MPPI: samples many candidate futures, scores them
   │                          vx_max 0.30 m/s, wz_max 1.0 rad/s
   │                          ──> /cmd_vel_nav
   ▼  velocity_smoother       caps acceleration ──> /cmd_vel_smoothed
   ▼  collision_monitor       watches /perception/depth_points ──> /cmd_vel_shim
   ▼  safety_guard            final veto ──> /cmd_vel ──> ESP32 ──> wheels
```

Both costmaps take obstacles from `NvbloxCostmapLayer` + `InflationLayer` at
5 cm resolution, in `global_frame: odom`.

The behaviour tree deliberately **omits Spin and BackUp**. The plugins are still
loaded in `nav2.yaml`, but the BT never calls them — blind recovery moves on a
tethered rover with a forward-only camera drive it into things it cannot see.

⚠️ The collision monitor's source is effectively dead — see `TODO.md §3`.

### Visualization and monitoring

| Node | In | Out |
|---|---|---|
| `rover_marker.py` | TF only | `/rover/model` — body, nose arrow, camera, FOV wedge |
| `rover_trail.py` | `/odometry/filtered` | `/rover/trail` — the driven line |
| `odom_health.py` | `/odom`, `/cmd_vel`, `/wheel_odom` | `/odom/health` — catches cuVSLAM lying |
| `stack_status.py` | everything | the `status` table |

`rover_trail.py` reads the **fused** pose, never `/odom` — the raw one is the
drifting one.

---

## 3. The drivetrain (other machines)

```
ESP32 firmware ──/wheel_state──> Pi5 relay ──/wheel_odom──> EKF
     ▲
     └── /cmd_vel   (subscribes; 50 Hz PID onto BTS7960 drivers)
```

`/wheel_state` is a `Vector3`: left velocity, right velocity, commanded vx. The
relay converts it to Odometry using `WHEEL_BASE_M = 0.34`.

Currently dead — `TODO.md §2`.

---

## 4. Where things live

```
rover/
  PRD.md            what we are building, and what we are NOT
  FACTS.md          measured truths — the most valuable file here
  ARCHITECTURE.md   this file
  TODO.md           every known bug and open question
  rover.sh          the one command; brings the stack up in LAYERS
  tasks/            one file per layer: learn, build, test, commit
  src/
    nodes/          long-running nodes the stack needs
    tools/          diagnostics you run by hand during a gate
    config/         ekf, nvblox, nav2, behaviour tree, rviz layouts
```

`src/` is mounted **read-only** at `/opt/rover` inside the container. Edit on the
host, restart the layer, done — no image rebuild.

The old repo at `langrobo_perception/` is the **parts bin**: read it, never edit
it. `$HOME/orin-nav-stack` is a symlink into it and must not be deleted.
