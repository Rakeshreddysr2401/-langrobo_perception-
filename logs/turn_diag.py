#!/usr/bin/env python3
"""Press LEFT and RIGHT, and see what the wheels and the gyro actually do.

A pivot needs the two sides to COUNTER-ROTATE. If velL and velR come back with
the same sign, the rover is driving, not turning. If they counter-rotate but the
gyro barely moves, the wheels are scrubbing without the chassis coming round --
not enough torque to break four tyres loose sideways.

Reads only; it commands nothing. Press the buttons yourself during the window.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu


class Diag(Node):
    def __init__(self):
        super().__init__('turn_diag')
        self.create_subscription(Twist, '/cmd_vel', self._c, 10)
        self.create_subscription(Vector3, '/wheel_state', self._w, qos_profile_sensor_data)
        self.create_subscription(Imu, '/gyro/base', self._g, qos_profile_sensor_data)
        self.vx = self.wz = 0.0
        self.velL = self.velR = 0.0
        self.rate = 0.0
        self.samples = []

    def _c(self, m):
        self.vx, self.wz = m.linear.x, m.angular.z

    def _w(self, m):
        self.velL, self.velR = m.x, m.y
        if abs(self.wz) > 0.1:
            self.samples.append((self.wz, m.x, m.y, self.rate))

    def _g(self, m):
        self.rate = m.angular_velocity.z


def main():
    window = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    rclpy.init()
    n = Diag()
    for _ in range(30):
        rclpy.spin_once(n, timeout_sec=0.05)

    print(f'\n  TURN DIAGNOSTIC — {window:.0f} s')
    print('  Hold LEFT for ~4 s, release, then hold RIGHT for ~4 s.\n')
    print('    cmd wz     velL     velR    gyro deg/s   what')
    print('  ' + '-' * 58)
    end = time.time() + window
    last = 0.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
        if time.time() - last >= 0.4:
            last = time.time()
            what = ''
            if abs(n.wz) > 0.1:
                if n.velL * n.velR < -1e-6:
                    what = 'COUNTER-ROTATING (pivot)'
                elif abs(n.velL) < 0.02 and abs(n.velR) < 0.02:
                    what = 'commanded, wheels STALLED'
                elif n.velL * n.velR > 1e-6:
                    what = 'SAME DIRECTION — driving, not turning'
            print(f'   {n.wz:+7.2f}  {n.velL:+8.3f} {n.velR:+8.3f}   {math.degrees(n.rate):+8.1f}   {what}')

    print()
    if not n.samples:
        print('  no turn was commanded — hold LEFT or RIGHT during the window.')
        return

    for name, sel in (('LEFT  (wz > 0)', lambda w: w > 0),
                      ('RIGHT (wz < 0)', lambda w: w < 0)):
        s = [x for x in n.samples if sel(x[0])]
        if not s:
            print(f'  {name}: not pressed')
            continue
        pl = max(abs(x[1]) for x in s)
        pr = max(abs(x[2]) for x in s)
        counter = sum(1 for x in s if x[1] * x[2] < -1e-6)
        peak_rate = max(abs(x[3]) for x in s)
        print(f'  {name}: {len(s)} samples   peak |velL| {pl:.3f}  |velR| {pr:.3f}   '
              f'peak turn {math.degrees(peak_rate):.1f} deg/s')
        print(f'      counter-rotating in {100 * counter / len(s):.0f}% of them')

    print('\n  READING')
    allc = sum(1 for x in n.samples if x[1] * x[2] < -1e-6)
    peak = max(abs(x[3]) for x in n.samples)
    stalled = sum(1 for x in n.samples if abs(x[1]) < 0.02 and abs(x[2]) < 0.02)
    if stalled > len(n.samples) * 0.5:
        print('    The wheels were commanded and did not move. Not enough duty to')
        print('    break four tyres loose sideways -- a pivot scrubs all of them.')
        print('    Raise WZ in the teleop: this firmware reads wz as rad/s, not as')
        print('    a PWM fraction, so 2.0 asks for only ~40% duty per wheel.')
    elif allc < len(n.samples) * 0.5:
        print('    The sides did NOT counter-rotate. That is driving, not pivoting --')
        print('    suspect a motor direction constant rather than a torque problem.')
    elif math.degrees(peak) < 10:
        print('    The wheels counter-rotated but the chassis barely came round')
        print(f'    (peak {math.degrees(peak):.1f} deg/s). They are scrubbing in place.')
    else:
        print(f'    Pivoting correctly, peak {math.degrees(peak):.1f} deg/s.')

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
