#!/usr/bin/env python3
"""One honest look at whether the pose is real (fusion2).

    python3 health.py         exit 0 = trustworthy, 1 = not

Reads /fusion/status properly (`ros2 topic echo --once` truncates its JSON)
for ~4 s and answers per source, because rates alone cannot. /odom stays at
20 Hz whatever happens underneath.

WHAT MATTERS NOW. fusion2 (SENSOR_FUSION_PLAN.md M3) gates every source, so a
diverged cuVSLAM no longer drags the pose with it: its fixes fail the gate
and are dropped. The pose depends on the LiDAR odometry above all
(LOCALIZATION.md §10-11), so:

  LIDAR    lidar_ok false for the whole window: the pose is running on gyro +
           wheels + VO. Usable for a short while, not trustworthy.       FAIL
  GYRO     fusion2 publishes from the gyro callback; if its status is silent,
           the gyro (the D555's IMU, LOCALIZATION_GAPS G12) has stopped.  FAIL
  CUVSLAM  reported, not fatal: vo_alive, vo_z (a ground rover stays within
           0.30 m of the floor), and how much of VO the gate accepted in the
           last second. Mostly rejected = cuVSLAM has lost it; fusion2 is
           ignoring it, and ./rover pose restarts it.
  WHEELS   alive, slip and stuck flags.
"""
import json
import sys
import time

import rclpy
from std_msgs.msg import String

VO_Z_LIMIT = 0.30


def main():
    rclpy.init()
    n = rclpy.create_node('health')
    got = []
    n.create_subscription(String, '/fusion/status', lambda m: got.append(json.loads(m.data)), 10)
    end = time.time() + 12.0
    while time.time() < end and len(got) < 4:
        rclpy.spin_once(n, timeout_sec=0.1)
    if rclpy.ok():
        rclpy.shutdown()

    if not got:
        print('  pose      ✗ no /fusion/status: fusion2 is not running, or the gyro it runs on has')
        print('              stopped (the D555 IMU). Check: ros2 topic hz /gyro/base; then ./rover fused')
        return 1
    s = got[-1]
    if s.get('estimator') != 'fusion2':
        print('  pose      ? /fusion/status is not from fusion2: an old fusion_node still running?')
        return 1

    rc = 0
    lidar_ok = [g.get('lidar_ok') for g in got]
    sd, sdd = s.get('sd_cm'), s.get('sd_deg')
    if not any(lidar_ok):
        print(f'  pose      ✗ LiDAR odometry unhealthy for the whole window: running on gyro + wheels')
        print(f'              + VO (sd {sd} cm / {sdd} deg). Check ./rover lidar and /lidar/odom_status.')
        rc = 1
    else:
        print(f'  pose      ok — LiDAR-anchored, sd {sd} cm / {sdd} deg, origin {s.get("origin_epoch")}')

    acc = s.get('vo_accept_1s')
    z = float(s.get('vo_z', 0.0))
    if not s.get('vo_alive'):
        print('  cuvslam   ! not publishing: fusion2 carries on without it; ./rover pose restarts it')
    elif abs(z) > VO_Z_LIMIT or (acc is not None and acc < 0.5):
        print(f'  cuvslam   ! lost (vo_z {z:+.2f} m, {acc if acc is not None else "-"} of fixes accepted): '
              'fusion2 is ignoring it; ./rover pose restarts it')
    else:
        print(f'  cuvslam   ok — {s.get("landmarks")} landmarks, vo_z {z:+.3f} m, '
              f'{acc if acc is not None else "-"} of fixes accepted')

    w = '✗ silent' if not s.get('wheels_alive') else ('! STUCK' if s.get('stuck') else 'ok')
    print(f'  wheels    {w} — slip {s.get("slip")}; accepted/rejected {s.get("accepted")} / {s.get("rejected")}')
    return rc


if __name__ == '__main__':
    sys.exit(main())
