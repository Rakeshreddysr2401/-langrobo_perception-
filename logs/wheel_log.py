#!/usr/bin/env python3
"""Log /wheel_state (left/right, m/s) with wall-clock timestamps.

WHY: the operator reported one side's wheels not moving during a commanded
pivot. /wheel_state is the firmware's own per-side report (x=velL, y=velR,
z=cmd vx, per rover_firmware_v2.ino) -- if one side stays near zero here
while the other shows real values, that is a drive fault (motor channel,
wiring, or firmware PID), not a nav2/velocity tuning problem.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3

DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0


class WheelLog(Node):
    def __init__(self):
        super().__init__('wheel_log')
        self.rows = []
        self.create_subscription(Vector3, '/wheel_state', self._cb, qos_profile_sensor_data)

    def _cb(self, m):
        self.rows.append((time.time(), m.x, m.y, m.z))


def main():
    rclpy.init()
    n = WheelLog()
    print(f'  logging /wheel_state for {DURATION_S:.0f}s -- drive the pivot now')
    t0 = time.time()
    while time.time() - t0 < DURATION_S:
        rclpy.spin_once(n, timeout_sec=0.05)
    print(f'  {len(n.rows)} samples')
    if n.rows:
        left = [r[1] for r in n.rows]
        right = [r[2] for r in n.rows]
        al = sum(abs(v) for v in left) / len(left)
        ar = sum(abs(v) for v in right) / len(right)
        print(f'  LEFT   min {min(left):+.3f}  max {max(left):+.3f}  |mean| {al:.3f} m/s')
        print(f'  RIGHT  min {min(right):+.3f}  max {max(right):+.3f}  |mean| {ar:.3f} m/s')
        print('\n  t(s)   velL     velR')
        last_t = None
        for t, l, r, cv in n.rows:
            rt = t - t0
            if last_t is None or rt - last_t >= 0.5:
                print(f'  {rt:5.1f}  {l:+7.3f}  {r:+7.3f}')
                last_t = rt
        if al < 0.02 or ar < 0.02:
            dead = 'LEFT' if al < ar else 'RIGHT'
            print(f'\n  FLAG: {dead} side barely moved (|mean| < 0.02 m/s) while the other did.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
