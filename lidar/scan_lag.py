#!/usr/bin/env python3
"""scan_lag.py — is /scan stamped at the moment it was measured?

WHY
    slam_toolbox looks up the pose at each scan's header.stamp. If the stamp is
    off by tau, every scan taken during a turn is matched against a pose that
    is omega*tau away from where the rover really was -- and the error adds up
    with total rotation in one direction. On 2026-09-23 two +90 turns left slam
    with an 8 deg heading correction that odometry and direct scan
    registration both said was wrong.

    sllidar_node stamps a scan with now() taken BEFORE grabScanDataHq(), which
    blocks until the SDK has a complete scan -- so whether the stamp leads or
    lags the measurement depends on the SDK's buffering. Measure it.

HOW
    Hold still, take a reference scan. Turn at a steady rate. For each scan
    taken while turning, register it against the reference (the rotation the
    ROOM says) and read /odom's yaw AT THE SCAN'S STAMP (interpolated from
    the /odom history). If the stamp is right the two agree; if it is off by
    tau they differ by omega*tau, with the sign following the turn direction.
    Doing both directions separates a lag from any fixed bias.

    Drives: two short turns, one each way, left side held. ./rover lidar --lag
"""
import bisect
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scan_tools import icp, scan_in_base, turn_twist, wrap, yaw_of  # noqa: E402

WZ = 1.5
TURN_S = 3.5


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Lag(Node):
    def __init__(self):
        super().__init__("scan_lag")
        self.odom_t, self.odom_y = [], []
        self.scans = []
        self.recording = False
        self.create_subscription(Odometry, "/odom", self._o, 50)
        self.create_subscription(LaserScan, "/scan", self._s, qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.last_scan = None

    def _o(self, m):
        y = yaw_of(m.pose.pose.orientation)
        if self.odom_y:                       # unwrap, so interpolation is sane
            y = self.odom_y[-1] + wrap(y - self.odom_y[-1])
        self.odom_t.append(stamp_s(m.header))
        self.odom_y.append(y)

    def _s(self, m):
        self.last_scan = m
        if self.recording:
            self.scans.append(m)

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def yaw_at(self, t):
        i = bisect.bisect_left(self.odom_t, t)
        if i <= 0 or i >= len(self.odom_t):
            return None
        t0, t1 = self.odom_t[i - 1], self.odom_t[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return self.odom_y[i - 1] + a * (self.odom_y[i] - self.odom_y[i - 1])

    def stop(self):
        for _ in range(8):
            self.cmd.publish(Twist())
            self.spin(0.03)

    def trial(self, sign, lx, lyaw):
        self.spin(1.0)
        ref = self.last_scan
        y_ref = self.yaw_at(stamp_s(ref.header))
        s_ref = scan_in_base(ref, lx, lyaw)
        self.scans = []
        self.recording = True
        t = turn_twist(sign * WZ)
        t0 = time.time()
        while time.time() - t0 < TURN_S and rclpy.ok():
            self.cmd.publish(t)
            self.spin(0.02)
        self.recording = False
        self.stop()
        self.spin(0.8)
        rows = []
        for m in self.scans:
            ts = stamp_s(m.header)
            y = self.yaw_at(ts)
            y_a, y_b = self.yaw_at(ts - 0.05), self.yaw_at(ts + 0.05)
            if y is None or y_a is None or y_b is None:
                continue
            omega = (y_b - y_a) / 0.1
            if abs(omega) < 0.05:                 # not turning yet (start-up) — useless
                continue
            d_odom = y - y_ref
            r = icp(scan_in_base(m, lx, lyaw), s_ref, d_odom)
            if r is None or r[2] > 0.03:
                continue
            rows.append((omega, wrap(r[0] - d_odom)))
        return rows


def main():
    rclpy.init()
    n = Lag()
    try:
        for _ in range(60):
            n.spin(0.25)
            if n.last_scan is not None and len(n.odom_t) > 20 and \
               n.buf.can_transform("base_link", "laser", rclpy.time.Time()):
                break
        else:
            print("      need /scan, /odom and base_link -> laser")
            return 1
        tf = n.buf.lookup_transform("base_link", "laser", rclpy.time.Time()).transform
        lx, lyaw = tf.translation.x, yaw_of(tf.rotation)
        print(f"      two {TURN_S:.1f} s turns (left side held), one each way ...")
        allrows = []
        for sign in (+1, -1):
            rows = n.trial(sign, lx, lyaw)
            if rows:
                om = np.mean([r[0] for r in rows])
                de = np.mean([r[1] for r in rows])
                print(f"      {'left ' if sign > 0 else 'right'}: {len(rows):3d} scans while turning at "
                      f"{om:+.3f} rad/s, room minus odom {math.degrees(de):+.2f} deg")
            allrows += rows
        if len(allrows) < 6:
            print("      too few usable scans to fit a lag")
            return 1
        om = np.array([r[0] for r in allrows])
        de = np.array([r[1] for r in allrows])
        # de = omega * tau + bias : least squares over both directions
        A = np.column_stack([om, np.ones_like(om)])
        (tau, bias), *_ = np.linalg.lstsq(A, de, rcond=None)
        # room minus odom-at-stamp = omega*tau  =>  the scan was really measured at stamp + tau
        print()
        print(f"      fitted: the scan was measured {tau*1000:+.0f} ms from its stamp "
              f"(bias {math.degrees(bias):+.2f} deg)")
        if abs(tau) < 0.015:
            print("      ✓ stamps are good to within 15 ms — not the cause")
        else:
            print(f"      ✗ at this turn rate that is {math.degrees(abs(tau*np.mean(np.abs(om)))):.1f} deg "
                  f"of heading error on every scan slam matches during a turn")
        return 0
    finally:
        try:
            n.stop()
        except Exception:
            pass
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
