#!/usr/bin/env python3
"""Verify all four encoders with no timing coordination at all.

The prompted version of this test failed for a dumb reason: its output was piped
through `tail`, so the operator never saw the per-wheel prompts and the whole run
recorded zeros. Anything that needs the human and the script to agree on WHEN is
fragile over a buffered pipe.

So this asks for nothing but a total. Spin all four wheels in any order, at any
pace, within one window. Cumulative counts make order irrelevant: a counter that
moves at all has a working encoder, and one that never leaves zero does not.

What it cannot tell you is which physical wheel maps to which counter -- for that
use spin_one.py, which already confirmed left-wheel -> velL. Alive-or-dead is the
open question here, and this answers it.
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


class T(Node):
    def __init__(self):
        super().__init__('wheels_selfpaced')
        self.create_subscription(Quaternion, '/wheel_ticks', self.cb, BE)
        self.cur = None
        self.lo = None
        self.hi = None

    def cb(self, m):
        v = [int(m.x), int(m.y), int(m.z), int(m.w)]
        self.cur = v
        if self.lo is None:
            self.lo = list(v)
            self.hi = list(v)
        else:
            for i in range(4):
                self.lo[i] = min(self.lo[i], v[i])
                self.hi[i] = max(self.hi[i], v[i])


def main():
    window = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    n = T()
    for _ in range(40):
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.cur is None:
        print('  no /wheel_ticks — is the ESP32 up?')
        rclpy.shutdown()
        return

    base = list(n.cur)
    n.lo = list(base)
    n.hi = list(base)
    end = time.time() + window
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)

    print('\n  baseline  LF %d  LR %d  RF %d  RR %d' % tuple(base))
    print('  final     LF %d  LR %d  RF %d  RR %d\n' % tuple(n.cur))
    print('    wheel  pins     travel (counts)   verdict')
    print('    ' + '-' * 46)
    dead = []
    for i, name in enumerate(NAMES):
        travel = n.hi[i] - n.lo[i]          # total swing, so direction cannot cancel
        ok = travel > 20
        if not ok:
            dead.append(name)
        print('    %-5s  %-7s  %13d   %s'
              % (name, PINS[name], travel, 'OK' if ok else 'NO SIGNAL'))

    print()
    if not dead:
        print('    All four encoders respond. Wheel odometry is trustworthy as a')
        print('    distance reference, and /wheel_ticks will expose a slipping')
        print('    wheel that the per-side average would hide.')
    elif len(dead) == 4:
        print('    Nothing moved at all. Either no wheel was actually spun during')
        print('    the window, or the encoders share a supply that is down —')
        print('    four independent failures at once is not plausible. Check 3V3')
        print('    and GND to the encoder harness before suspecting signal lines.')
    else:
        print('    DEAD: %s  (pins %s)' % (', '.join(dead),
                                           ', '.join(PINS[d] for d in dead)))
        print('    A dead front or rear encoder does NOT zero its side: velL is')
        print('    0.5*(dLF+dLR), so it reads HALF true. /wheel_odom then believes')
        print('    the rover is permanently turning. Fix before the drift gate.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
