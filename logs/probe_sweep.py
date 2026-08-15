#!/usr/bin/env python3
"""Does the ESP32's telemetry rate follow the rate we send TO it?

Reliable QoS did not fix the 1 Hz, so the output stream flush theory is dead.
What survived is that /wheel_state speeds up while we publish /cmd_vel.

If loop() were healthy, the telemetry rate would be a flat 20 Hz regardless of
what we send. If instead rclc_executor_spin_some() blocks until a message
arrives -- rather than honouring its 5 ms timeout -- then each inbound message
releases exactly one loop() iteration, and the telemetry rate should TRACK the
inbound rate, with a floor around 1 Hz set by whatever the idle timeout is.

Sweeping the send rate separates those cleanly:

    telemetry flat ~20 Hz            -> loop() fine, look downstream
    telemetry ~= send rate           -> loop() is unblocked by arriving messages
    telemetry flat ~1 Hz             -> inbound traffic is irrelevant after all

SAFETY: unchanged -- linear.x peaks at 0.004 m/s, under the 0.01 dead zone in
pidStep(), so both sides get zero duty. Zero command sent at the end.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Vector3

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=100)
RATES = [0.0, 2.0, 5.0, 10.0, 20.0, 40.0]
WINDOW = 12.0
STEP = 2e-5


class Sweep(Node):
    def __init__(self):
        super().__init__('probe_sweep')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Vector3, '/wheel_state', self.cb, BEST_EFFORT)
        self.t = []
        self.on = False

    def cb(self, _m):
        if self.on:
            self.t.append(time.time())


def phase(n, tx_hz, window):
    n.t = []
    n.on = True
    t0 = time.time()
    i = 0
    if tx_hz <= 0:
        while time.time() < t0 + window:
            rclpy.spin_once(n, timeout_sec=0.02)
    else:
        period = 1.0 / tx_hz
        while time.time() < t0 + window:
            i += 1
            m = Twist()
            m.linear.x = (i % 200) * STEP
            m.angular.z = 0.0
            n.pub.publish(m)
            deadline = t0 + i * period
            while time.time() < deadline:
                rclpy.spin_once(n, timeout_sec=0.001)
    n.on = False
    k = len(n.t)
    if k < 2:
        return k, 0.0
    return k, (k - 1) / (n.t[-1] - n.t[0])


def main():
    rclpy.init()
    n = Sweep()
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)

    print(f'\n  sweeping /cmd_vel send rate, {WINDOW:.0f} s per step\n')
    print('   cmd_vel out    wheel_state in     ratio')
    print('  ----------------------------------------')
    rows = []
    for r in RATES:
        k, hz = phase(n, r, WINDOW)
        ratio = (hz / r) if r > 0 else float('nan')
        rstr = 'silent' if r == 0 else f'{r:5.1f} Hz'
        rat = '   -  ' if r == 0 else f'{ratio:5.2f}'
        print(f'   {rstr:>10}    {hz:6.2f} Hz ({k:3d})    {rat}')
        rows.append((r, hz))
        time.sleep(1.0)

    z = Twist()
    for _ in range(5):
        n.pub.publish(z)
        time.sleep(0.02)

    idle = rows[0][1]
    driven = [(r, h) for r, h in rows if r > 0]
    print('\n  READING')
    if all(abs(h - idle) < max(0.5, idle * 0.3) for _, h in driven):
        print('    Telemetry rate is flat regardless of what we send. Inbound')
        print('    traffic is NOT the variable -- the earlier correlation was noise.')
    elif all(h > 0.6 * r for r, h in driven):
        print('    Telemetry rate TRACKS the send rate. Each arriving message is')
        print('    releasing one loop() iteration, so loop() is blocking on the')
        print('    executor instead of honouring its 5 ms timeout.')
    else:
        best = max(driven, key=lambda p: p[1])
        print(f'    Telemetry rises with traffic but does not track it '
              f'(best {best[1]:.1f} Hz at {best[0]:.0f} Hz in).')
        print('    Partial coupling -- see the ratio column for where it saturates.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
