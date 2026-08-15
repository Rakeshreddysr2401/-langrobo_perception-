#!/usr/bin/env python3
"""Spin exactly ONE named wheel; report which channel reacts.

check_encoders.py showed velL flat at 0.000 while velR reacted during BOTH
phases. That is ambiguous between:

  (a) the left encoders are dead, or
  (b) the left and right encoder inputs are swapped in the firmware pin map.

The difference matters: (a) is a soldering iron, (b) is two constants. Telling
them apart needs one wheel spun in isolation, with the channel that reacts
recorded.

Usage: spin_one.py <label> [seconds]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Vector3

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)


class One(Node):
    def __init__(self):
        super().__init__('spin_one')
        self.create_subscription(Vector3, '/wheel_state', self.cb, BEST_EFFORT)
        self.s = []

    def cb(self, m):
        self.s.append((time.time(), m.x, m.y))


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else 'the wheel'
    window = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

    rclpy.init()
    n = One()
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)
    n.s = []

    print(f'\n  SPIN ONLY: {label}   -- {window:.0f} s, starting NOW')
    print('  (keep every other wheel completely still)\n')
    t0 = time.time()
    end = t0 + window
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)

    if not n.s:
        print('  no samples arrived')
        rclpy.shutdown()
        return

    print('   t(s)     velL     velR')
    print('  ------------------------')
    for t, x, y in n.s:
        mark = ''
        if abs(x) > 0.02:
            mark += '  <- LEFT channel'
        if abs(y) > 0.02:
            mark += '  <- RIGHT channel'
        print(f'  {t - t0:5.1f}  {x:+7.3f}  {y:+7.3f}{mark}')

    pl = max(abs(s[1]) for s in n.s)
    pr = max(abs(s[2]) for s in n.s)
    nl = sum(1 for s in n.s if abs(s[1]) > 0.02)
    nr = sum(1 for s in n.s if abs(s[2]) > 0.02)
    print(f'\n  {len(n.s)} samples.  peak |velL| {pl:.3f} ({nl} non-zero)   '
          f'peak |velR| {pr:.3f} ({nr} non-zero)')

    print('\n  READING')
    if nl and not nr:
        print(f'    Spinning {label} moved the LEFT channel only.')
    elif nr and not nl:
        print(f'    Spinning {label} moved the RIGHT channel only.')
    elif nl and nr:
        print('    BOTH channels moved -- either both wheels turned, or the two')
        print('    sides share an encoder line. Re-run holding the other wheel.')
    else:
        print(f'    NEITHER channel moved while spinning {label}.')
        print('    That side reads nothing at all: dead encoder, or unpowered.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
