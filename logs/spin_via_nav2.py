#!/usr/bin/env python3
"""Trigger nav2's own Spin action and log /odom x,y,yaw through the motion.

WHY NOT logs/yaw_crosscheck.py: that script publishes /cmd_vel directly,
bypassing nav2's velocity_smoother and the behavior server entirely (which
route through cmd_vel_nav since TODO 40). It cannot tell us whether NAV2'S
capped rotation (1.5 rad/s) pivots or curves -- only calling /spin itself can.

WHY 180 DEGREES, NOT 360: a rover curving at constant angular+linear velocity
traces a circle, and 360 deg of heading change closes that circle exactly --
it returns to the start (x,y) once per lap, curve or not. A full-turn test
cannot tell pivot from curve by looking at the endpoint. Displacement from the
start point is maximized at 180 deg of heading change for any given curve
radius, so that is the target here -- and drift is tracked continuously
(not just start/end) so a mid-motion peak is never missed.

Safety: publishes a zero Twist on every exit path; hard wall-clock timeout
beyond the action's own time_allowance.
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from nav2_msgs.action import Spin
from geometry_msgs.msg import Twist

TARGET_DEG = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
TIME_ALLOWANCE_S = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SpinTest(Node):
    def __init__(self):
        super().__init__('spin_via_nav2')
        self.odom = None
        self.samples = []  # (t, x, y, yaw)
        self.create_subscription(Odometry, '/odom', self._odom, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.client = ActionClient(self, Spin, '/spin')

    def _odom(self, m):
        self.odom = m
        p = m.pose.pose.position
        self.samples.append((time.time(), p.x, p.y, yaw_of(m.pose.pose.orientation)))

    def stop(self):
        t = Twist()
        for _ in range(10):
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init()
    n = SpinTest()

    print('  waiting for /odom and the /spin action server...')
    e = time.time() + 15
    while time.time() < e and n.odom is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.odom is None:
        print('  ABORT: no /odom'); n.stop(); rclpy.shutdown(); sys.exit(1)
    if not n.client.wait_for_server(timeout_sec=10.0):
        print('  ABORT: /spin action server not available (is nav2 up?)')
        n.stop(); rclpy.shutdown(); sys.exit(1)

    x0, y0 = n.odom.pose.pose.position.x, n.odom.pose.pose.position.y
    yaw0 = yaw_of(n.odom.pose.pose.orientation)
    n.samples = [(time.time(), x0, y0, yaw0)]

    goal = Spin.Goal()
    goal.target_yaw = math.radians(TARGET_DEG)
    goal.time_allowance.sec = int(TIME_ALLOWANCE_S)

    print(f'  sending Spin goal: {TARGET_DEG:+.0f} deg, time_allowance {TIME_ALLOWANCE_S:.0f}s')
    fb = {'last': 0.0}

    def feedback_cb(msg):
        fb['last'] = math.degrees(msg.feedback.angular_distance_traveled)

    send_future = n.client.send_goal_async(goal, feedback_callback=feedback_cb)
    rclpy.spin_until_future_complete(n, send_future, timeout_sec=10.0)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        print('  ABORT: Spin goal rejected'); n.stop(); rclpy.shutdown(); sys.exit(1)

    result_future = goal_handle.get_result_async()
    t0 = time.time()
    while not result_future.done():
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - t0 > TIME_ALLOWANCE_S + 10:
            print('  ABORT: hard timeout waiting for Spin result')
            n.stop(); rclpy.shutdown(); sys.exit(1)
    result = result_future.result()
    n.stop()
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.02)

    print(f'\n  Spin result: error_code={result.result.error_code} '
          f'({result.result.error_msg or "ok"}), '
          f'elapsed={result.result.total_elapsed_time.sec}s, '
          f'last feedback angular_distance_traveled={fb["last"]:.1f} deg')

    max_drift, max_at = 0.0, None
    for t, x, y, yaw in n.samples:
        d = math.hypot(x - x0, y - y0)
        if d > max_drift:
            max_drift, max_at = d, (t - n.samples[0][0], math.degrees(yaw - yaw0))

    xf, yf = n.odom.pose.pose.position.x, n.odom.pose.pose.position.y
    yawf = yaw_of(n.odom.pose.pose.orientation)
    final_drift = math.hypot(xf - x0, yf - y0)

    print(f'\n  -- pivot vs curve ' + '-' * 30)
    print(f'  start   x={x0:+.3f} y={y0:+.3f} yaw={math.degrees(yaw0):+.1f} deg')
    print(f'  end     x={xf:+.3f} y={yf:+.3f} yaw={math.degrees(yawf):+.1f} deg')
    print(f'  heading changed  {math.degrees(yawf - yaw0):+.1f} deg (commanded {TARGET_DEG:+.0f})')
    print(f'  final drift      {final_drift * 100:6.1f} cm')
    print(f'  MAX drift during motion  {max_drift * 100:6.1f} cm'
          + (f'  (at t={max_at[0]:.1f}s, {max_at[1]:+.1f} deg into the turn)' if max_at else ''))
    print(f'  samples logged: {len(n.samples)}')
    verdict = ('PIVOT (clean)' if max_drift < 0.05 else
               'CURVING' if max_drift > 0.15 else 'BORDERLINE -- inspect by eye')
    print(f'\n  VERDICT: {verdict}  (< 5 cm = pivot, > 15 cm = curve, by max drift)')

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
