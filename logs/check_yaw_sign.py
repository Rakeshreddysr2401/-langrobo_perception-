#!/usr/bin/env python3
"""Does a LEFT command actually produce a POSITIVE yaw rate?

REP-103 fixes the convention: +z is counter-clockwise, i.e. LEFT. The teleop
app agrees -- its MOVES table comments "+wz turns LEFT (CCW)" and sends
angular.z = +2.0 for the left button. Everything downstream assumes this, and
nav2 will too: a flipped sign makes the robot turn away from every goal.

This compares the SIGN of what was commanded with the SIGN of what the gyro
measured, over the same moment. It is one of the few things that cannot be
checked by reading code, because the answer depends on how the IMU is bolted in
and how gyro_node re-frames it.

Nothing here commands motion; it only listens while YOU press a button.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class Sign(Node):
    def __init__(self):
        super().__init__('check_yaw_sign')
        self.create_subscription(Twist, '/cmd_vel', self._c, 10)
        self.create_subscription(Imu, '/gyro/base', self._g,
                                 qos_profile_sensor_data)
        self.cmd_wz = 0.0
        self.gyro_z = 0.0
        self.pairs = []          # (commanded wz, measured z) while commanding

    def _c(self, m):
        self.cmd_wz = m.angular.z

    def _g(self, m):
        self.gyro_z = m.angular_velocity.z
        if abs(self.cmd_wz) > 0.1 and abs(self.gyro_z) > 0.05:
            self.pairs.append((self.cmd_wz, self.gyro_z))


def main():
    window = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    n = Sign()
    for _ in range(30):
        rclpy.spin_once(n, timeout_sec=0.05)

    print(f'\n  YAW SIGN CHECK — {window:.0f} s')
    print('  Press and hold the LEFT button for a few seconds.\n')
    print('     commanded wz    measured gyro z')
    print('  ' + '-' * 40)
    end = time.time() + window
    last = 0.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
        if time.time() - last >= 0.4:
            last = time.time()
            tag = ''
            if abs(n.cmd_wz) > 0.1:
                tag = '  <- commanding'
            print(f'      {n.cmd_wz:+8.3f}      {n.gyro_z:+8.3f}{tag}')

    if not n.pairs:
        print('\n  nothing was commanded — press and HOLD the LEFT button '
              'during the window.')
        return
    agree = sum(1 for c, g in n.pairs if (c > 0) == (g > 0))
    frac = agree / len(n.pairs)
    mc = sum(c for c, _ in n.pairs) / len(n.pairs)
    mg = sum(g for _, g in n.pairs) / len(n.pairs)
    print(f'\n  {len(n.pairs)} samples while commanding')
    print(f'  mean commanded {mc:+.3f} rad/s, mean measured {mg:+.3f} rad/s')
    print(f'  signs agree on {frac * 100:.0f}% of samples\n')
    if frac > 0.9:
        print('  CONVENTION IS CORRECT. A left command yields a positive yaw')
        print('  rate, matching REP-103. nav2 will turn the way it intends.')
    elif frac < 0.1:
        print('  SIGN IS INVERTED. A left command yields a NEGATIVE yaw rate.')
        print('  Everything downstream is affected: the fused heading, and later')
        print('  nav2, which would steer away from every goal. Fix it in')
        print('  gyro_node.py where the IMU is re-framed into base_link, not by')
        print('  negating it further down the chain.')
    else:
        print('  INCONCLUSIVE — signs agreed on only %.0f%%. Likely the rover was'
              % (frac * 100))
        print('  rocking rather than turning steadily. Hold the button longer.')

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
