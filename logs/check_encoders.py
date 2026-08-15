#!/usr/bin/env python3
"""Prove each side's encoders work, by hand, with the rover lifted.

/wheel_state carries x = measured velL, y = measured velR (m/s), computed in the
ESP32's 50 Hz control task from the encoder counts. That task runs on its own
FreeRTOS core and does not care about micro-ROS, WiFi, or the publish rate -- so
even at 1 Hz these values are trustworthy as a yes/no on the wiring.

This is the ROS equivalent of the serial check in phase1/firmware/FLASHING.md,
which we cannot run without a USB cable.

Phase 1: spin the LEFT wheel forward by hand   -> x must go clearly positive
Phase 2: spin the RIGHT wheel forward by hand  -> y must go clearly positive

If a side stays at 0.00 while you spin it, that side's encoder wiring is the
fault -- not the software, and not the publish rate.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Vector3

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
PHASE = 15.0    # seconds per side


class Enc(Node):
    def __init__(self):
        super().__init__('check_encoders')
        self.create_subscription(Vector3, '/wheel_state', self.cb, BEST_EFFORT)
        self.samples = []

    def cb(self, m):
        self.samples.append((m.x, m.y))


def run(n, label, seconds):
    n.samples = []
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
        left = end - time.time()
        if n.samples:
            lx = max(abs(s[0]) for s in n.samples)
            ly = max(abs(s[1]) for s in n.samples)
        else:
            lx = ly = 0.0
        sys.stdout.write(f'\r  {label}  {left:4.1f}s left   '
                         f'peak |velL| {lx:5.2f}   peak |velR| {ly:5.2f}   '
                         f'({len(n.samples)} samples)   ')
        sys.stdout.flush()
    print()
    if not n.samples:
        return 0.0, 0.0, 0
    return (max(abs(s[0]) for s in n.samples),
            max(abs(s[1]) for s in n.samples),
            len(n.samples))


def main():
    rclpy.init()
    n = Enc()
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)

    print('\n  ENCODER CHECK -- rover must be LIFTED, wheels free\n')
    print('  >>> spin the LEFT wheel by hand, steadily, for 15 s. Starting now.')
    l1, r1, n1 = run(n, 'LEFT ', PHASE)

    print('\n  >>> now spin the RIGHT wheel by hand, steadily, for 15 s.')
    time.sleep(2.0)
    l2, r2, n2 = run(n, 'RIGHT', PHASE)

    print('\n  RESULT')
    print(f'    while spinning LEFT :  peak |velL| {l1:.3f}   peak |velR| {r1:.3f}'
          f'   ({n1} samples)')
    print(f'    while spinning RIGHT:  peak |velL| {l2:.3f}   peak |velR| {r2:.3f}'
          f'   ({n2} samples)')
    print()

    THRESH = 0.02
    ok_l, ok_r = l1 > THRESH, r2 > THRESH
    print(f'    LEFT  encoder: {"OK" if ok_l else "NO SIGNAL"}')
    print(f'    RIGHT encoder: {"OK" if ok_r else "NO SIGNAL"}')
    if ok_l and ok_r:
        print('\n    Both sides measure motion. The encoders and the 50 Hz control')
        print('    task are working; the only wheel problem is the publish rate.')
    else:
        print('\n    A side reading zero while you spun it is a wiring fault on that')
        print('    side, independent of the 1 Hz issue. Check its encoder A/B lines.')
    if n1 < 5 or n2 < 5:
        print('\n    NOTE: very few samples arrived (publish rate is ~1 Hz), so a')
        print('    zero could be a missed sample. Re-run if a side reads NO SIGNAL.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
