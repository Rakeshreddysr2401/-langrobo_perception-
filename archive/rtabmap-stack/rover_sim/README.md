# rover_sim — the laptop half of sim/real parity

Implements the **hardware impersonation** side of `../SIM_REAL_PARITY.md`:
Gazebo (headless) + a contract bridge that publishes the *exact* D555 topic
contract (real captured intrinsics, frames, formats, wall-clock stamps) and
consumes `/cmd_vel` with the ESP32 firmware's behavior (PWM deadband,
watchdog) emulated. The Jetson/Pi5/Mac-mini stack runs unchanged and cannot
tell it is in simulation.

```
laptop (rakhi24@192.168.1.12, Ubuntu 24.04, ROS Jazzy, gz Harmonic 8, Intel iGPU)
  gz sim -s --headless-rendering  worlds/langrobo_home.sdf   (RTF 1.0)
  ros_gz parameter_bridge          internal /rover_sim/* names
  bridge/contract_bridge.py        -> /camera/camera0/*  + /rover_sim/status
                                   <- /cmd_vel (firmware emulation) -> DiffDrive
```

## Files

- `worlds/langrobo_home.sdf` — 6×5 m room, doorway, furniture, a "bottle"
  on the table, and colored wall/floor patches (RTAB-Map needs features).
- `models/rover/model.sdf` — diff-drive rover, D555-matched sensor rig
  (rgbd at the infra1 viewpoint, color at its real −5.9 cm offset, 200 Hz
  imu). Footprint/mount mirror `nav2_real.yaml` + `run_robot_tf.sh`.
- `bridge/contract_bridge.py` — the impersonation layer; port of
  `rover_firmware.ino` drive behavior (`PWM_MIN 130`, 0.30 m/s full scale,
  500 ms watchdog, firmware's uncalibrated wz/2.0 mixing kept on purpose).
- `d555_contract/` — deployed copy of `../config/d555_contract/` (captured
  real camera_infos + static frame tree). Never edit; recapture from real.

## Run

Deployed by `scripts/deploy_rover_sim.sh` (repo → `~/langrobo/rover_sim/`,
repo copy is the source of truth):

```bash
ssh rakhi24@192.168.1.12 '~/langrobo/rover_sim/run_sim.sh'   # start
ssh rakhi24@192.168.1.12 '~/langrobo/rover_sim/stop_sim.sh'  # stop
# then on the Jetson:
echo sim > config/hardware && robot restart
scripts/check_contract.sh
```

## Known divergences (accepted, documented)

- 2 driven wheels + frictionless casters vs the real 4-wheel skid-steer:
  the sim turns with less resistance.
- Rendered images are undistorted; the published color `camera_info`
  carries the real (small) distortion coefficients.
- WiFi transport: fine for bring-up; full-rate gate runs prefer ethernet
  (~300 Mbit/s total at nominal rates).
