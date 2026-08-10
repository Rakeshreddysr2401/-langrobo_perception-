# archive — kept for history, not for reading

Nothing in here describes the robot as it runs today. It is kept because it
contains measurements and reasoning that were expensive to get, and git history
alone is harder to browse.

**If you are looking for how the robot works, go to [`../learn/`](../learn/).**

## `docs/` — superseded documentation

| File | Why it is here |
|---|---|
| `CUVSLAM_ORIN_GUIDE.md` | Setup guide from before the stack was containerised |
| `ISSUES_AND_SOLUTIONS.md` | RTAB-Map-era bug log |
| `MAPPING_WORKFLOW.md` | RTAB-Map mapping workflow — replaced by cuVSLAM |
| `NAVIGATION_PIPELINE.md` | Described the pipeline with the deadband shim still in it |
| `SIM_REAL_PARITY.md` | Gazebo sim parity — the sim is archived too |
| `HOW_MOVEMENT_WORKS.md` | Pre-firmware-v2 (open-loop L298N) |
| `SENSOR_FUSION_NOTES.md` | Pre-EKF fusion notes |
| `REQUIREMENTS.md` | Old product requirements — replaced by `learn/PRD.md` |
| `SESSION_2026-08-10.md` | Debugging session log (7 silent failures) |
| `ThingsTodo.md` | The 7-phase rebuild plan — replaced by `learn/` issues 00–08 |

Every measurement worth keeping from `ThingsTodo.md` and `SESSION_2026-08-10.md`
has been carried into `learn/ARCHITECTURE.md`. Nothing was lost.

## `rtabmap-stack/` — the old ROS 2 workspace

The July 2026 stack, from before `orin-nav-stack/` existed. It ran as a loose
collection of shell scripts inside an Isaac ROS dev container at
`/workspaces/isaac_ros-dev/src/langrobo_perception/` — **a path that no longer
exists on this machine**, so none of these scripts can run as written.

`scripts/ launch/ langrobo_perception/ docker/ rover_sim/ config/ resource/
systemd/ package.xml setup.py setup.cfg`

It used **RTAB-Map** as the localizer. The current stack uses **cuVSLAM**, which
is the entire premise of `orin-nav-stack/`. Several nodes here (`detections_3d`,
`pixel_to_goal`, `visual_approach`, `imu_to_base`, `drive_test`) have direct,
improved descendants in `orin-nav-stack/nodes/` — use those.

`rover_sim/` is a Gazebo simulation. It was never brought forward to the cuVSLAM
stack. If sim is wanted again it starts from here, but it is not on the roadmap.
