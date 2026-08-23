# `logs/` — the diagnostic tools

Each answers one question. None of them is a wrapper around `./rover`; they exist
for the moments when a layer is *running* and *wrong*, which is the hard case.

All are run the same way:

```bash
docker exec -it rover bash -lc 'source /opt/ros/jazzy/setup.bash; \
  export ROS_DOMAIN_ID=0; python3 -u /logs/SCRIPT.py'
```

## Driving

| script | question |
|---|---|
| `goto.py x y [theta] [--rel]` | send the rover somewhere. Refuses if the goal is unreachable or something else owns `/cmd_vel` |
| `loop_test.py` | how far is the *estimate* off? Asks for a tape reading — the raw number grades your parking, not the pose |
| `turn_diag.py 45` | do the wheels counter-rotate on a pivot, and at what duty? |
| `wzsweep.py` | what commanded `wz` actually rotates this chassis? There is a deadband below 0.8 |

## Map and planning

| script | question |
|---|---|
| `map_view.py --once` | the map as ASCII, rover marked |
| `map_stats.py` | free / occupied / unknown, in m² |
| `esdf.py` | is the map good enough to plan on? The number nav2 actually cares about |
| `around.py` | why won't it plan? Shows the costmap where the rover stands |

## Hardware

| script | question |
|---|---|
| `check_rates.py` | every ESP32 topic against what it should be |
| `check_encoders.py` | do the encoders work? By hand, rover lifted |
| `check_all_wheels.py` | all four in one pass |
| `wheels_selfpaced.py` | all four, with no timing coordination at all |
| `spin_one.py` | spin ONE named wheel; which channel reacts? |
| `check_yaw_sign.py` | does a LEFT command give POSITIVE yaw, per REP-103? |
| `watch_teleop.py` | follow a button press from the phone to the wheels |
| `check_equivalence.py` | does `fusion_node`'s `/odom` agree with `compare.py`'s FUSED row? |

## Calibration

| script | question |
|---|---|
| `calibrate_encoders.py 200` | metres-per-count per wheel, against a tape. Forward push only |
| `calibrate_rotation.py` | the effective track width this rover turns about — not the physical one |

## Kept as a fallback

`keepalive.py` publishes zero `/cmd_vel` continuously. It existed to work around
`rclc_executor_spin_some` blocking, and the firmware fix removed the need — but
keep it in case a future flash regresses. **Do not run it by default:** it feeds
the 500 ms motion watchdog permanently and would mask a real teleop fault.
