#!/usr/bin/env python3
"""Does fusion_node's /odom agree with compare.py's FUSED row?

fusion.py was extracted from compare.py verbatim so that one algorithm serves
both. "Verbatim" is a claim about the source, though, not about behaviour --
the two run as separate instances, calibrate their gyro bias in different
windows, and see messages in a different order.

So this measures it instead of asserting it. Run compare.py at the same time,
drive, and read the two endpoints off. They should agree to within a
centimetre or two; a large divergence means the extraction changed something.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class Rec(Node):
    def __init__(self):
        super().__init__('check_equivalence')
        self.create_subscription(Odometry, '/odom', self._o, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/vo/odom', self._v, qos_profile_sensor_data)
        self.f = None
        self.v = None
        self.n = 0

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _o(self, m):
        p = m.pose.pose.position
        self.f = (p.x, p.y, self._yaw(m.pose.pose.orientation))
        self.n += 1

    def _v(self, m):
        p = m.pose.pose.position
        self.v = (p.x, p.y, self._yaw(m.pose.pose.orientation))


def main():
    rclpy.init()
    n = Rec()
    for _ in range(40):
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.f is None:
        print('  no /odom — is ./rover fused up?')
        return
    print('\n  recording /odom (fused) beside /vo/odom (raw). Ctrl-C when done.\n')
    print('      fused x      y     yaw        raw x      y     yaw')
    print('  ' + '-' * 56)
    last = 0.0
    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.02)
            if time.time() - last >= 0.5:
                last = time.time()
                fx, fy, ft = n.f
                vx, vy, vt = n.v if n.v else (0, 0, 0)
                print('   %8.3f %6.3f %7.2f     %8.3f %6.3f %7.2f'
                      % (fx, fy, math.degrees(ft), vx, vy, math.degrees(vt)))
    except KeyboardInterrupt:
        pass

    fx, fy, ft = n.f
    vx, vy, vt = n.v if n.v else (0, 0, 0)
    print('\n  ── final ' + '─' * 46)
    print('   /odom  (fused)  x %+.3f  y %+.3f  yaw %+.2f deg   -> straight %.1f cm'
          % (fx, fy, math.degrees(ft), math.hypot(fx, fy) * 100))
    print('   /vo/odom (raw)  x %+.3f  y %+.3f  yaw %+.2f deg   -> straight %.1f cm'
          % (vx, vy, math.degrees(vt), math.hypot(vx, vy) * 100))
    print('\n   %d /odom messages' % n.n)
    print('\n   Compare the fused line against compare.py\'s FUSED row. Agreement to')
    print('   a centimetre or two means fusion.py and compare.py are the same')
    print('   algorithm; a large gap means the extraction changed behaviour.')
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
