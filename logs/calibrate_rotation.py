#!/usr/bin/env python3
"""Measure the effective track width this skid-steer rover turns about.

WHY IT IS NOT 34 cm. A differential-drive robot converts a left/right speed
difference into a yaw rate using its physical track:  wz = (vR - vL) / W.
This rover has four driven wheels and NO steering, so turning drags every tyre
sideways across the floor. The width it behaves as if it has is larger than the
one you measure with a tape, and by how much depends on the tyres and the floor.

Using the physical 0.34 m made the wheels read 146 deg on a hand-set 90 deg turn
-- 63% too much. One measurement is not a calibration though, which is why this
exists: rotate a known angle, in both directions, and read the number off.

REFERENCE. Your physical alignment is the truth. The gyro is printed beside it
as an independent check -- it measured 89.80 on that same 90 deg turn, so if the
two disagree by much, trust your floor marking and re-run.

BOTH DIRECTIONS MATTER. Scrub is not guaranteed symmetric: unequal tyre wear or
weight distribution shows up as a left/right difference, and that is worth
knowing before nav2 relies on it.

Usage:  calibrate_rotation.py <true_angle_deg>     e.g. 360, or 90
        Positive angle = counter-clockwise (LEFT).  Use a negative angle for CW.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu

WHEEL_BASE_M = 0.34          # physical, tape-measured
BIAS_S = 4.0                 # seconds of stillness to measure gyro bias


class Rot(Node):
    def __init__(self):
        super().__init__('calibrate_rotation')
        self.create_subscription(Vector3, '/wheel_state', self._w,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/gyro/base', self._g,
                                 qos_profile_sensor_data)
        self.create_subscription(Quaternion, '/wheel_ticks', self._t,
                                 qos_profile_sensor_data)
        self.tick0 = None
        self.tick = None
        self.t0 = time.time()
        # unwrapped integrals — a full turn must read 360, not wrap to 0
        self.wheel_yaw = 0.0      # from (vR - vL) / physical track
        self.gyro_yaw = 0.0
        self.bias = None
        self.cal = []
        self.wt = None
        self.gt = None
        self.peak_rate = 0.0

    def _t(self, m):
        self.tick = [int(m.x), int(m.y), int(m.z), int(m.w)]
        if self.tick0 is None and self.bias is not None:
            self.tick0 = list(self.tick)

    def _w(self, m):
        now = time.time()
        if self.wt is not None and self.bias is not None:
            dt = now - self.wt
            if 0 < dt < 0.5:
                self.wheel_yaw += ((m.y - m.x) / WHEEL_BASE_M) * dt
        self.wt = now

    def _g(self, m):
        now = time.time()
        z = m.angular_velocity.z
        if self.bias is None:
            self.cal.append(z)
            if now - self.t0 >= BIAS_S and len(self.cal) > 50:
                self.bias = sum(self.cal) / len(self.cal)
                self.gt = now
                self.wt = now
            return
        if self.gt is not None:
            dt = now - self.gt
            if 0 < dt < 0.5:
                rate = z - self.bias
                self.gyro_yaw += rate * dt
                self.peak_rate = max(self.peak_rate, abs(rate))
        self.gt = now


def main():
    if len(sys.argv) < 2:
        print('usage: calibrate_rotation.py <true_angle_deg>   '
              '(+ = left/CCW, - = right/CW)')
        return
    truth = float(sys.argv[1])

    rclpy.init()
    n = Rot()
    print(f'\n  ROTATION CALIBRATION — hold STILL {BIAS_S:.0f} s while the gyro '
          f'bias is measured…')
    while n.bias is None and time.time() - n.t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.bias is None:
        print('  no gyro data — is ./rover pose up?')
        return
    print(f'  bias {math.degrees(n.bias):+.4f} deg/s removed\n')
    print(f'  Now rotate EXACTLY {truth:+.0f} deg, then Ctrl-C.')
    print('  Turn in short taps rather than one long spin — a pivot swings the')
    print('  camera at ~34 cm/s and speed is what loses tracking.\n')

    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.02)
            sys.stdout.write('\r   wheels %+8.2f deg   gyro %+8.2f deg   '
                             'peak %5.1f deg/s   '
                             % (math.degrees(n.wheel_yaw),
                                math.degrees(n.gyro_yaw),
                                math.degrees(n.peak_rate)))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        # rclpy raises this if Ctrl-C lands while the executor is mid-take on a
        # subscription. It killed the results of a real 360 deg calibration run
        # on 2026-08-21 -- the numbers were on screen and the summary never
        # printed. Same guard as compare.py.
        if 'convert call argument' not in str(e):
            raise

    w = math.degrees(n.wheel_yaw)
    g = math.degrees(n.gyro_yaw)
    print('\n\n  ── result ' + '─' * 52)
    print(f'   you turned      {truth:+8.2f} deg   (your reference)')
    print(f'   gyro read       {g:+8.2f} deg   ({g - truth:+.2f} vs truth)')
    print(f'   wheels read     {w:+8.2f} deg   ({w - truth:+.2f} vs truth)   '
          f'using the physical {WHEEL_BASE_M} m')
    print(f'   peak turn rate  {math.degrees(n.peak_rate):8.1f} deg/s')

    if abs(truth) < 1e-6 or abs(w) < 1e-6:
        print('\n   nothing to calibrate — no rotation recorded')
        return

    ratio = w / truth
    eff = WHEEL_BASE_M * ratio
    print(f'\n   wheels over-read by {ratio:.3f}x')
    print(f'   => effective track width  {eff:.4f} m')

    if abs(g - truth) > 0.1 * abs(truth):
        print(f'\n   WARNING: the gyro disagrees with your reference by '
              f'{g - truth:+.1f} deg.')
        print('   One of the two is wrong. Re-check the floor marking and re-run')
        print('   before trusting this number.')

    # Per-wheel travel during the turn. On a rigid four-wheel chassis the two
    # wheels on a side are bolted to the same body and must sweep the same arc;
    # any difference is one of them slipping. The operator reported the wheels
    # "not in sync" during the pivot, and this is where that shows up as a number.
    if n.tick0 is not None and n.tick is not None:
        d = [abs(a - b) for a, b in zip(n.tick, n.tick0)]
        names = ['LF', 'LR', 'RF', 'RR']
        print('\n   per-wheel counts through the turn:')
        for nm, v in zip(names, d):
            print(f'     {nm}  {v:8d}')
        for side, i, j in (('LEFT', 0, 1), ('RIGHT', 2, 3)):
            hi, lo = max(d[i], d[j]), min(d[i], d[j])
            if hi > 0:
                print(f'     {side} front/rear mismatch {100 * (hi - lo) / hi:5.1f}%'
                      + ('   <- slipping' if (hi - lo) / hi > 0.10 else ''))

    print(f'\n   Put this in phase1/nodes/compare.py:')
    print(f'     WHEEL_BASE_ROT_M = {eff:.4f}')
    print('\n   Then run again turning the OTHER way. Scrub need not be')
    print('   symmetric, and a large left/right difference is itself a finding.')

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
