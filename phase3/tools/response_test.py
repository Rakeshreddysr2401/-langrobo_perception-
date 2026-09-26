#!/usr/bin/env python3
"""response_test.py — how this rover REALLY answers a command, on this floor.

    ./rover response [pivot] [arc] [creep]    (default all; moves within ~30 cm)

The goal executor's first live goals (LOCALIZATION.md §12) failed where the
simulator's model of the rover was wrong: small in-place turns stalled, and
steering while driving corrected less than modelled. This measures both,
plus the slowest straight speed that moves at all:

  pivot   wz = 0.6 .. 1.5 rad/s in place, each direction, 2 s: the actual
          turn rate (gyro, steady part) and the centre's slide (fused pose)
  arc     vx = 0.10 m/s with wz = 0.15 / 0.30 / 0.50, 1.5 s forward then the
          same arc back: the actual turn rate while driving, and the speed
  creep   vx = 0.02 / 0.035 / 0.05 m/s, 1.5 s: does it move at all

Results go to /logs/calib/response_<time>.json; the table printed is what the
executor and the simulator are set from. Per surface, re-run it.
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

OUT = Path('/logs/calib')
LASER = (0.1342, 0.0, 1.5463)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class T(Node):
    def __init__(self):
        super().__init__('response_test')
        self.gz, self.odom, self.scan = [], None, None
        self.create_subscription(Imu, '/gyro/base', lambda m: self.gz.append(
            (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.z)), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def drive(self, vx, wz, s):
        tw = Twist()
        tw.linear.x, tw.angular.z = vx, wz
        end = time.time() + s
        while time.time() < end:
            self.cmd.publish(tw)
            self.spin(0.05)

    def stop(self, s=1.2):
        self.drive(0.0, 0.0, s)

    def pose(self):
        p = self.odom.pose.pose
        return np.array([p.position.x, p.position.y, yaw_of(p.orientation)])

    def nearest(self):
        m = self.scan
        r = np.asarray(m.ranges)
        a = m.angle_min + np.arange(r.size) * m.angle_increment
        ok = np.isfinite(r) & (r > 0.05) & (r < 8)
        p = np.stack([r[ok] * np.cos(a[ok] + LASER[2]) + LASER[0], r[ok] * np.sin(a[ok] + LASER[2])], 1)
        own = (p[:, 0] < 0.202) & (p[:, 0] > -0.198) & (np.abs(p[:, 1]) < 0.21)
        p = p[~own]
        return float(np.min(np.hypot(p[:, 0], p[:, 1]))) if len(p) else 9.0

    def rate(self, t0, t1, bias):
        g = [w for t, w in self.gz if t0 <= t <= t1]
        return (float(np.mean(g)) - bias) if g else float('nan')


def main():
    rclpy.init()
    n = T()
    end = time.time() + 10
    while time.time() < end and (n.odom is None or n.scan is None or len(n.gz) < 100):
        n.spin(0.1)
    if n.odom is None or n.scan is None:
        sys.exit('  need /odom and /scan: ./rover fused, ./rover lidar')
    n.gz.clear()
    n.stop(3.0)
    bias = float(np.median([w for _, w in n.gz])) if n.gz else 0.0
    print(f'  gyro bias {bias:+.4f} rad/s\n')
    res = {'bias': bias, 'pivot': [], 'arc': [], 'creep': []}

    parts = [a for a in sys.argv[1:] if a in ('pivot', 'arc', 'creep')] or ['pivot', 'arc', 'creep']
    # Sustained pivoting has frozen the drive for seconds at a time (the
    # BTS7960s or the pack, ROVER_BUILD_PLAN.md §4.5), which invalidates any
    # part run after it; run arcs and creep on a rested drive, or alone.
    if 'pivot' in parts:
      print('  PIVOT in place (2 s each)         actual rad/s   ratio   centre slid')
    for wz in ((0.6, 0.8, 1.0, 1.2, 1.5) if 'pivot' in parts else ()):
        for sgn in (1, -1):
            if n.nearest() < 0.36:
                print('  ✗ something inside 0.36 m: stopping the test'); n.stop(); return 1
            p0 = n.pose()
            tstart = time.time()
            n.drive(0.0, sgn * wz, 2.0)
            gz_t = [t for t, _ in n.gz]
            w = n.rate(gz_t[-1] - 1.2, gz_t[-1], bias) if gz_t else float('nan')
            n.stop()
            p1 = n.pose()
            slid = float(np.hypot(*(p1[:2] - p0[:2])))
            turned = math.degrees(math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2])))
            res['pivot'].append({'cmd': sgn * wz, 'actual': w, 'turned_deg': turned, 'slid_m': slid})
            print(f'    cmd {sgn * wz:+.1f}   {w:+.3f}   {w / (sgn * wz):5.2f}   {slid * 100:4.1f} cm   ({turned:+.0f} deg)')

    if 'arc' in parts:
        print('\n  ARC at vx 0.10 (1.5 s out, 1.5 s back)   actual wz   ratio   speed')
    for wz in ((0.15, 0.30, 0.50) if 'arc' in parts else ()):
        if n.nearest() < 0.36:
            print('  ✗ something inside 0.36 m: stopping the test'); n.stop(); return 1
        for vx in (0.10, -0.10):
            p0 = n.pose()
            n.drive(vx, wz, 1.5)
            gz_t = [t for t, _ in n.gz]
            w = n.rate(gz_t[-1] - 0.8, gz_t[-1], bias) if gz_t else float('nan')
            n.stop(0.8)
            p1 = n.pose()
            d = float(np.hypot(*(p1[:2] - p0[:2])))
            res['arc'].append({'vx': vx, 'cmd': wz, 'actual': w, 'dist_m': d})
            print(f'    vx {vx:+.2f} wz {wz:.2f}   {w:+.3f}   {w / wz:5.2f}   {d / 1.5:.3f} m/s')

    if 'creep' in parts:
        print('\n  CREEP straight (1.5 s)                moved')
    for vx in ((0.02, 0.035, 0.05) if 'creep' in parts else ()):
        for s in (1, -1):
            p0 = n.pose()
            n.drive(s * vx, 0.0, 1.5)
            n.stop(0.8)
            d = float(np.hypot(*(n.pose()[:2] - p0[:2])))
            res['creep'].append({'vx': s * vx, 'moved_m': d})
            print(f'    vx {s * vx:+.3f}                           {d * 100:4.1f} cm')
    n.stop()
    OUT.mkdir(parents=True, exist_ok=True)
    f = OUT / f'response_{time.strftime("%Y%m%d-%H%M%S")}.json'
    f.write_text(json.dumps(res, indent=1))
    print(f'\n  saved {f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
