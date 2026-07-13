# langrobo_perception

Jetson Orin perception bringup for the LangRobo rover: **nvblox** (GPU 3D
reconstruction from a depth camera) + **Nav2** (planning/control), per
`~/robot/ARCHITECTURE.md`. The LangGraph brain on the Pi 5 sends
`NavigateToPose` goals; this stack turns them into `TwistStamped` wheel
commands.

## Profiles

| | `mode:=sim` (working today) | `mode:=real` (pending D555) |
|---|---|---|
| Depth source | rover_sim Gazebo on laptop, `/cam_1/depth/*` | RealSense D555 `/camera0/depth/*` |
| Localization | wheel odometry from sim (`odom` frame) | cuVSLAM (`map` frame) |
| Clock | sim `/clock` (`use_sim_time:=True`) | wall clock |

## Run (sim profile)

1. Laptop: start the sim with its DDS peers profile
   (`WORLD=cafe rosmaster_x3_gazebo.sh` with
   `FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/ros2_ws/fastdds_peers.xml`).
2. Jetson host:
   ```bash
   docker exec -it isaac_ros_dev_container \
     /workspaces/isaac_ros-dev/src/langrobo_perception/scripts/run_perception_sim.sh
   ```
3. Send a goal from anywhere on the DDS mesh (Pi 5 in production):
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
     "{pose: {header: {frame_id: odom}, pose: {position: {x: 1.5}, orientation: {w: 1.0}}}}"
   ```

## Build

```bash
docker exec -it isaac_ros_dev_container bash -c \
  "cd /workspaces/isaac_ros-dev && source /opt/ros/jazzy/setup.bash && \
   colcon build --symlink-install --packages-select langrobo_perception"
```

## Networking

All DDS traffic is unicast-peered over wifi (router drops wifi↔wifi
multicast): container profile `/workspaces/isaac_ros-dev/config/fastdds_unicast.xml`
lists the Pi (192.168.1.16) and the sim laptop (192.168.1.12); the laptop's
counterpart is `/workspace/ros2_ws/fastdds_peers.xml`. Master copy:
`~/robot/config/fastdds_unicast.xml` — keep them in sync.

## When the D555 arrives (real profile TODO)

1. Camera driver: `realsense_example.launch.py` in `nvblox_examples_bringup`
   is the template (splitter + emitter config ship with the container).
2. Localization: add `isaac_ros_visual_slam` (see
   `launch/perception/vslam.launch.py` in `nvblox_examples_bringup`);
   switch `global_frame` to `map` in an `nvblox_real.yaml` + `nav2_real.yaml`
   (copy the sim ones, change frames/topics/`use_sim_time`).
3. `cmd_vel_stamper` stays — the real rover's ESP32 path also takes
   TwistStamped (verify topic name against the micro-ROS firmware).

## Notes

- MPPI runs `motion_model: Omni` — the mecanum base strafes; keep `vy`
  limits in sync with `velocity_smoother` if you tune speeds.
- Collision monitor watches the **depth point cloud**, not lidar, so the
  stack works on a camera-only robot. If the sim lidar is on, adding a
  `scan` source is a 5-line change in `nav2_sim.yaml`.
- Costmaps read nvblox's `/nvblox_node/static_map_slice`
  (`NvbloxCostmapLayer`); there is no occupancy-grid SLAM anywhere in this
  stack.
