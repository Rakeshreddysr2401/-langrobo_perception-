#!/usr/bin/env python3
"""Verify all four encoders in one pass, using cumulative ticks.

This test only became possible with /wheel_ticks. /wheel_state carries
0.5*(dLF+dLR) per side, so a dead front encoder merely halves the reading and
looks like a healthy wheel turning slowly -- a hand-spin test passed at 0.068 m/s
with LF contributing nothing at all.

Counters are cumulative and were zeroed by the power-cycle, so whichever wheel
moves is unambiguous. Spin each wheel in turn; the timeline shows which counter
responded.

Any wheel whose counter never leaves 0 while you spin it has an encoder fault on
its own A/B lines -- LF 34/35, LR 36/39, RF 32/33, RR 25/26.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Quaternion

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=50)
NAMES = ['LF', 'LR', 'RF', 'RR']
PINS = {'LF': '34/35', 'LR': '36/39', 'RF': '32/33', 'RR': '25/26'}
PER_WHEEL = 10.0


class Ticks(Node):
    def __init__(self):
        super().__init__('check_all_wheels')
        self.create_subscription(Quaternion, '/wheel_ticks', self.cb, BE)
        self.cur = [0, 0, 0, 0]
        self.got = False

    def cb(self, m):
        self.cur = [int(m.x), int(m.y), int(m.z), int(m.w)]
        self.got = True


def spin_for(n, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)


def main():
    rclpy.init()
    n = Ticks()
    for _ in range(40):
        rclpy.spin_once(n, timeout_sec=0.05)
    if not n.got:
        print('  no /wheel_ticks — is the ESP32 up?')
        rclpy.shutdown()
        return

    print('\n  FOUR-WHEEL ENCODER CHECK — rover LIFTED, wheels free')
    print(f'  Spin each wheel when prompted, {PER_WHEEL:.0f} s each.\n')
    print(f'  baseline: LF {n.cur[0]}  LR {n.cur[1]}  RF {n.cur[2]}  RR {n.cur[3]}\n')

    results = {}
    for i, name in enumerate(NAMES):
        corner = {'LF': 'FRONT-LEFT', 'LR': 'REAR-LEFT',
                  'RF': 'FRONT-RIGHT', 'RR': 'REAR-RIGHT'}[name]
        print(f'  >>> spin the {corner} wheel now  ({PER_WHEEL:.0f} s) …',
              flush=True)
        before = list(n.cur)
        spin_for(n, PER_WHEEL)
        after = list(n.cur)
        d = [abs(a - b) for a, b in zip(after, before)]
        results[name] = d
        moved = [NAMES[j] for j in range(4) if d[j] > 20]
        print(f'      deltas  LF {d[0]:+6d}  LR {d[1]:+6d}  '
              f'RF {d[2]:+6d}  RR {d[3]:+6d}   -> moved: '
              f'{", ".join(moved) if moved else "NOTHING"}')
        time.sleep(1.5)

    print('\n  RESULT')
    bad, crossed = [], []
    for i, name in enumerate(NAMES):
        d = results[name]
        own = d[i]
        others = [NAMES[j] for j in range(4) if j != i and d[j] > 20]
        if own > 20:
            note = 'OK'
            if others:
                note += f'  (but {", ".join(others)} also moved)'
                crossed.append(name)
        else:
            note = 'NO SIGNAL'
            bad.append(name)
            if others:
                note += f'  — {", ".join(others)} moved instead: wiring swapped?'
        print(f'    {name}  pins {PINS[name]:<6}  own delta {own:+7d}   {note}')

    print()
    if not bad and not crossed:
        print('    All four encoders read their own wheel and nothing else.')
        print('    Wheel odometry can be trusted as a distance reference.')
    if bad:
        print(f'    DEAD: {", ".join(bad)}. Check those A/B lines and 3V3/GND.')
        print('    Until fixed, distL = 0.5*(dLF+dLR) reads HALF the true value on')
        print('    that side, so /wheel_odom will believe the rover is permanently')
        print('    turning. Do not use wheel odometry for the drift gate yet.')
    if crossed:
        print(f'    CROSSTALK on {", ".join(crossed)} — more than one counter moved.')
        print('    Either two wheels turned together, or lines are shared.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
