#!/usr/bin/env python3
"""Drive a loop and come back. Does the line close?

Grades /odom -- the SAME estimate fusion_node publishes and RViz draws the track
from -- rather than a separate instance inside compare.py. So the number this
prints is exactly what you are looking at on screen.

The rover does not have to start at the origin: it marks wherever it is when you
say go, so you can run this at any point in a session without restarting
anything and losing the map.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
import json


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Loop(Node):
    def __init__(self):
        super().__init__('loop_test')
        self.create_subscription(Odometry, '/odom', self._o, qos_profile_sensor_data)
        self.create_subscription(String, '/fusion/status', self._s, 10)
        self.now = None
        self.health = {}
        self.peak = 0.0
        self.prev = None

    def _o(self, m):
        p = m.pose.pose.position
        t = time.time()
        self.now = (p.x, p.y, yaw_of(m.pose.pose.orientation))
        if self.prev is not None:
            dt = t - self.prev[0]
            d = math.hypot(p.x - self.prev[1], p.y - self.prev[2])
            if 0 < dt < 1.0 and d / dt < 1.0:
                self.peak = max(self.peak, d / dt)
        self.prev = (t, p.x, p.y)

    def _s(self, m):
        try:
            self.health = json.loads(m.data)
        except (ValueError, TypeError):
            pass


def main():
    rclpy.init()
    n = Loop()
    end = time.time() + 10
    while n.now is None and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.now is None:
        print('  no /odom — is ./rover fused up?')
        return

    sx, sy, sth = n.now
    j0 = n.health.get('jumps', 0)
    d0 = n.health.get('vo_dropped', 0)
    print(f'\n  START MARKED at x {sx:+.3f}  y {sy:+.3f}  yaw {math.degrees(sth):+.1f} deg')
    print('  Drive wherever you like, come back to this exact spot, then Ctrl-C.')
    print('  Put the rover back facing the same way too — heading is graded.\n')
    print('      from start    x       y      yaw      peak speed')
    print('  ' + '-' * 56)
    last = 0.0
    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.02)
            if time.time() - last >= 0.5:
                last = time.time()
                x, y, th = n.now
                dx, dy = x - sx, y - sy
                print(f'      {math.hypot(dx, dy) * 100:7.1f} cm  {dx:+6.2f} {dy:+6.2f}  '
                      f'{math.degrees(th - sth):+7.1f}   {n.peak * 100:5.1f} cm/s')
    except KeyboardInterrupt:
        pass

    x, y, th = n.now
    dx, dy = x - sx, y - sy
    err = math.hypot(dx, dy)
    dth = math.degrees((th - sth + math.pi) % (2 * math.pi) - math.pi)
    jumps = n.health.get('jumps', 0) - j0
    dropped = n.health.get('vo_dropped', 0) - d0

    print('\n  ── result ' + '─' * 46)
    print(f'   back at the start, /odom is {err * 100:.1f} cm away   '
          f'-> {"PASS" if err <= 0.10 else "FAIL"} (want <= 10.0 cm)')
    print(f'   heading is {dth:+.1f} deg off   '
          f'-> {"PASS" if abs(dth) <= 10 else "FAIL"} (want <= 10 deg)')
    print(f'   peak speed {n.peak * 100:.0f} cm/s'
          + ('   <- over the 25 cm/s tracking limit' if n.peak > 0.25 else ''))
    print()
    print(f'   during the run: {jumps} cuvslam teleport(s), '
          f'{dropped} frames with cuvslam dropped')
    lm = n.health.get('landmarks', -1)
    print(f'   landmarks now {lm}' + ('   <- thin, tracking is fragile' if 0 <= lm < 30 else ''))
    print()
    print('   This is the same estimate RViz draws /fusion/path from, so the')
    print('   number above is exactly how far the green line missed by.')
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
