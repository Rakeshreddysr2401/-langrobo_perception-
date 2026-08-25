#!/usr/bin/env python3
"""Rotate in place and ask all four yaw sources what just happened.

THE POINT: no tape measure. A 360 deg turn is the one manoeuvre whose ground
truth you know without measuring anything -- you end up facing where you
started. So a powered spin grades every heading source at once, and they are
independent enough that agreement means something:

    gyro        integrated /gyro/base, bias-corrected. Does not care about light.
    cuvslam     /vo/odom yaw. Rotation is visual odometry's WEAKEST case (TODO 5).
    fused       /odom yaw -- what nav2 steers on and RViz draws.
    wheels      (velR - velL) / WHEEL_BASE_ROT_M. Blind to texture, fooled by scrub.

A pivot should also END WHERE IT STARTED. Translation during a pure rotation is
drift, so start/end position is reported too -- for cuvslam that is the number
TODO 4 predicts will be worst.

Safety: hard motion timeout, stop published on every exit path, and it aborts if
vo_node re-creates the tracker mid-run (the pose origin would move under it).
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

# fusion.py:86 -- the width this skid-steer BEHAVES as, not the 0.34 m tape
# reading. Scrub makes the effective track wider; see logs/calibrate_rotation.py.
WHEEL_BASE_ROT_M = 0.5216

TARGET_DEG = float(sys.argv[1]) if len(sys.argv) > 1 else 360.0
WZ = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0   # 0.80 is breakaway
MOTION_TIMEOUT_S = 45.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Spin(Node):
    def __init__(self):
        super().__init__('yaw_crosscheck')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.vo = None
        self.gyro_yaw = 0.0          # integrated
        self.wheel_yaw = 0.0         # integrated
        self._g_last = None
        self._w_last = None
        self.resets = None
        self.reset_now = None
        self.create_subscription(Odometry, '/odom', self._odom, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/vo/odom', self._vo_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/gyro/base', self._gyro, qos_profile_sensor_data)
        self.create_subscription(Vector3, '/wheel_state', self._wheel, qos_profile_sensor_data)
        self.create_subscription(String, '/vo/status', self._status, 10)

    def _odom(self, m):
        self.odom = m

    def _vo_cb(self, m):
        self.vo = m

    def _gyro(self, m):
        t = time.time()
        if self._g_last is not None:
            self.gyro_yaw += m.angular_velocity.z * (t - self._g_last)
        self._g_last = t

    def _wheel(self, m):
        # x = left m/s, y = right m/s
        t = time.time()
        if self._w_last is not None:
            self.wheel_yaw += ((m.y - m.x) / WHEEL_BASE_ROT_M) * (t - self._w_last)
        self._w_last = t

    def _status(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self.reset_now = d.get('tracker_resets')
        if self.resets is None:
            self.resets = self.reset_now

    def stop(self):
        t = Twist()
        for _ in range(15):
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init()
    n = Spin()

    print(f'  waiting for /odom, /vo/odom, /gyro/base, /wheel_state, /vo/status ...')
    e = time.time() + 20
    while time.time() < e and not (n.odom and n.vo and n.resets is not None
                                   and n._g_last and n._w_last):
        rclpy.spin_once(n, timeout_sec=0.1)
    missing = [x for x, v in (('/odom', n.odom), ('/vo/odom', n.vo),
                              ('/vo/status', n.resets is not None),
                              ('/gyro/base', n._g_last), ('/wheel_state', n._w_last)) if not v]
    if missing:
        print(f'  ABORT: no data on {", ".join(missing)}')
        n.stop(); rclpy.shutdown(); sys.exit(1)

    # Settle, then mark.
    for _ in range(40):
        rclpy.spin_once(n, timeout_sec=0.02)
    o0, v0 = n.odom.pose.pose, n.vo.pose.pose
    f_y0, c_y0 = yaw_of(o0.orientation), yaw_of(v0.orientation)
    n.gyro_yaw = n.wheel_yaw = 0.0
    f_prev, c_prev = f_y0, c_y0
    f_acc = c_acc = 0.0
    resets0 = n.resets

    target = math.radians(TARGET_DEG)
    sign = 1.0 if target >= 0 else -1.0
    print(f'  spinning {TARGET_DEG:+.0f} deg at wz={WZ * sign:+.2f} rad/s '
          f'(timeout {MOTION_TIMEOUT_S:.0f} s). tap MANUAL to stop.')

    tw = Twist(); tw.angular.z = float(WZ * sign)
    t0 = time.time()
    aborted = None
    try:
        while True:
            n.pub.publish(tw)
            rclpy.spin_once(n, timeout_sec=0.02)
            f_y, c_y = yaw_of(n.odom.pose.pose.orientation), yaw_of(n.vo.pose.pose.orientation)
            f_acc += (f_y - f_prev + math.pi) % (2 * math.pi) - math.pi
            c_acc += (c_y - c_prev + math.pi) % (2 * math.pi) - math.pi
            f_prev, c_prev = f_y, c_y
            if abs(f_acc) >= abs(target):
                break
            if time.time() - t0 > MOTION_TIMEOUT_S:
                aborted = f'motion timeout after {MOTION_TIMEOUT_S:.0f} s'
                break
            if n.reset_now is not None and n.reset_now != resets0:
                aborted = 'vo_node re-created the tracker mid-spin'
                break
    finally:
        n.stop()

    for _ in range(30):
        rclpy.spin_once(n, timeout_sec=0.02)
    dt = time.time() - t0
    o1, v1 = n.odom.pose.pose, n.vo.pose.pose

    print()
    if aborted:
        print(f'  ABORTED: {aborted}')
    print(f'  ── yaw over {dt:.1f} s ' + '─' * 34)
    print(f'  {"source":10} {"degrees":>10} {"err vs target":>15}')
    rows = [('gyro', math.degrees(n.gyro_yaw)),
            ('cuvslam', math.degrees(c_acc)),
            ('fused', math.degrees(f_acc)),
            ('wheels', math.degrees(n.wheel_yaw))]
    for name, deg in rows:
        print(f'  {name:10} {deg:10.2f} {deg - TARGET_DEG:+15.2f}')

    spread = max(d for _, d in rows) - min(d for _, d in rows)
    print(f'\n  spread across sources   {spread:.2f} deg')
    print(f'  GATE (TODO 5): |error| <= 10 deg on the fused row -> '
          f'{"PASS" if abs(rows[2][1] - TARGET_DEG) <= 10 else "FAIL"}')

    dxy_f = math.hypot(o1.position.x - o0.position.x, o1.position.y - o0.position.y)
    dxy_c = math.hypot(v1.position.x - v0.position.x, v1.position.y - v0.position.y)
    print(f'\n  ── a pivot should end where it started ' + '─' * 18)
    print(f'  fused   drifted {dxy_f * 100:6.1f} cm')
    print(f'  cuvslam drifted {dxy_c * 100:6.1f} cm   (TODO 4 predicts this is worse)')
    print(f'  tracker resets during run: {(n.reset_now or 0) - resets0}')

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
