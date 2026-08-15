#!/usr/bin/env python3
"""Measure each wheel's true metres-per-count against a tape measure.

RESULT, 2026-08-15, 200 cm by tape:

    wheel   counts   implied CPR   vs configured 1560
    LF      11434       1526.6          0.98x
    LR      11378       1519.2          0.97x
    RF      11623       1551.9          0.99x
    RR      11275       1505.4          0.97x

ENCODER_CPR 1560 is RIGHT, within 3%, and all four wheels agree within 3%. No
firmware change needed.

This test exists because an earlier analysis claimed the opposite -- 2x out,
with rear reading 25% more than front. That analysis compared cumulative wheel
counts, which measure ARC LENGTH, against cuVSLAM's `straight`, which is
DISPLACEMENT from the origin. On a push that curves or doubles back the two are
not the same quantity at all, and the "error" was the difference between them.
Calibrate against a tape on a straight forward push, and against nothing else.

WHAT IT NEEDS FROM YOU. One straight forward push of a known distance, measured
with a tape. Forward only: reversing is fine for the encoders but it lets a
counting error cancel itself out and hide.

The tape is the reference, not cuVSLAM -- cuVSLAM reads ~2% under, which would
be baked into every wheel if we calibrated against it. cuVSLAM is printed only
as a cross-check.

Usage: calibrate_encoders.py <true_distance_cm>
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

NAMES = ['LF', 'LR', 'RF', 'RR']
WHEEL_D = 0.085          # m, tyre OD, confirmed with a tape 2026-08-15
CUR_CPR = 1560.0         # what the firmware believes today


class Cal(Node):
    def __init__(self):
        super().__init__('calibrate_encoders')
        self.create_subscription(Quaternion, '/wheel_ticks', self._t,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/vo/odom', self._vo,
                                 qos_profile_sensor_data)
        self.ticks = None
        self.base = None
        self.vo = None
        self.vo_base = None

    def _t(self, m):
        self.ticks = [int(m.x), int(m.y), int(m.z), int(m.w)]
        if self.base is None:
            self.base = list(self.ticks)

    def _vo(self, m):
        p = m.pose.pose.position
        self.vo = (p.x, p.y)
        if self.vo_base is None:
            self.vo_base = (p.x, p.y)


def main():
    if len(sys.argv) < 2:
        print('usage: calibrate_encoders.py <true_distance_cm>')
        return
    truth_cm = float(sys.argv[1])

    rclpy.init()
    n = Cal()
    for _ in range(60):
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.base is None:
        print('  no /wheel_ticks — is the ESP32 up?')
        return

    print(f'\n  CALIBRATION — push straight forward exactly {truth_cm:.0f} cm, then Ctrl-C')
    print(f'  baseline  LF {n.base[0]}  LR {n.base[1]}  RF {n.base[2]}  RR {n.base[3]}')
    print('  forward only — do not reverse, it lets errors cancel and hide\n')

    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.05)
            d = [c - b for c, b in zip(n.ticks, n.base)]
            vo = 0.0
            if n.vo and n.vo_base:
                vo = math.hypot(n.vo[0] - n.vo_base[0], n.vo[1] - n.vo_base[1])
            sys.stdout.write('\r  counts  LF %+7d  LR %+7d  RF %+7d  RR %+7d    '
                             'cuVSLAM %6.1f cm   ' % (d[0], d[1], d[2], d[3], vo * 100))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass

    d = [c - b for c, b in zip(n.ticks, n.base)]
    truth_m = truth_cm / 100.0
    print('\n\n  ── per-wheel calibration ' + '─' * 44)
    print('   wheel   counts    m/count      implied CPR    vs current 1560')
    print('   ' + '-' * 62)
    mpcs = []
    for i, name in enumerate(NAMES):
        if d[i] == 0:
            print(f'   {name}      {d[i]:7d}    NO COUNTS — this encoder did not register')
            mpcs.append(None)
            continue
        mpc = truth_m / d[i]
        cpr = math.pi * WHEEL_D / mpc
        mpcs.append(mpc)
        print(f'   {name}      {d[i]:7d}    {mpc:.8f}   {cpr:8.1f}        '
              f'{cpr / CUR_CPR:5.2f}x')

    good = [m for m in mpcs if m]
    if not good:
        print('\n  nothing counted — nothing to calibrate')
        return

    if n.vo and n.vo_base:
        vo = math.hypot(n.vo[0] - n.vo_base[0], n.vo[1] - n.vo_base[1])
        print(f'\n   cross-check: cuVSLAM read {vo * 100:.1f} cm for your '
              f'{truth_cm:.0f} cm '
              f'({(vo * 100 - truth_cm) / truth_cm * 100:+.1f}%)')

    spread = (max(good) - min(good)) / (sum(good) / len(good)) * 100
    print(f'\n   spread between wheels: {spread:.1f}%')
    if spread > 10:
        print('   The wheels genuinely differ. A single ENCODER_CPR cannot serve')
        print('   all four — the firmware needs one constant PER WHEEL, otherwise')
        print('   averaging front and rear per side mixes two scale factors.')
        print('\n   Suggested firmware constants:')
        for i, name in enumerate(NAMES):
            if mpcs[i]:
                print(f'     METRES_PER_COUNT_{name}  {mpcs[i]:.8f}f')
    else:
        avg = sum(good) / len(good)
        print(f'   Close enough for one shared constant:')
        print(f'     METRES_PER_COUNT  {avg:.8f}f    '
              f'(ENCODER_CPR {math.pi * WHEEL_D / avg:.0f})')

    # Ctrl-C already ran rclpy's signal handler, which shuts the context down.
    # Calling it again raises RCLError and buries the results under a traceback.
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
