#!/usr/bin/env python3
"""A/B: does inbound /cmd_vel traffic change the /wheel_state publish rate?

probe_loop_rate.py measured /wheel_state at 11.5 Hz while it was itself
publishing /cmd_vel at 20 Hz. Every previous measurement -- ros2 topic hz,
compare.py, values.py -- saw a metronomic 1.000 Hz with nothing being sent TO
the board. Both used best-effort QoS, so QoS is not the difference.

That leaves the traffic itself. This measures the same topic, the same way,
twice: once silent, once while sending /cmd_vel. Nothing else changes.

SAFETY: identical to probe_loop_rate.py -- commanded velocity peaks at
0.0048 m/s, under the 0.01 dead zone in pidStep(), so duty is zero on both
sides. A zero command is sent at the end of phase B.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Vector3

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
WINDOW = 15.0      # seconds per phase
TX_RATE = 20.0     # Hz, phase B only
STEP = 2e-5


class AB(Node):
    def __init__(self):
        super().__init__('probe_ab')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Vector3, '/wheel_state', self.on_wheel, BEST_EFFORT)
        self.stamps = []
        self.collect = False

    def on_wheel(self, m):
        if self.collect:
            self.stamps.append(time.time())


def stats(name, stamps, window):
    n = len(stamps)
    if n < 2:
        print(f'  {name:<28} {n} msgs in {window:.0f} s  -- too few to characterise')
        return None
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    mean = sum(gaps) / len(gaps)
    sd = (sum((g - mean) ** 2 for g in gaps) / len(gaps)) ** 0.5
    hz = (n - 1) / (stamps[-1] - stamps[0])
    print(f'  {name:<28} {n:4d} msgs   {hz:6.3f} Hz   '
          f'gap {mean * 1000:7.1f} ms +/- {sd * 1000:5.1f} ms')
    return hz


def main():
    rclpy.init()
    n = AB()
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)

    # ── phase A: silent ─────────────────────────────────────────────────────
    print(f'\n  phase A: listening for {WINDOW:.0f} s, sending NOTHING')
    n.stamps = []
    n.collect = True
    t_end = time.time() + WINDOW
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.02)
    n.collect = False
    a_stamps = list(n.stamps)

    time.sleep(1.0)

    # ── phase B: sending /cmd_vel at 20 Hz ──────────────────────────────────
    print(f'  phase B: listening for {WINDOW:.0f} s while sending /cmd_vel at '
          f'{TX_RATE:.0f} Hz')
    n.stamps = []
    n.collect = True
    t0 = time.time()
    period = 1.0 / TX_RATE
    i = 0
    while time.time() < t0 + WINDOW:
        i += 1
        t = Twist()
        t.linear.x = (i % 200) * STEP
        t.angular.z = 0.0
        n.pub.publish(t)
        deadline = t0 + i * period
        while time.time() < deadline:
            rclpy.spin_once(n, timeout_sec=0.002)
    n.collect = False
    b_stamps = list(n.stamps)

    z = Twist()
    for _ in range(5):
        n.pub.publish(z)
        time.sleep(0.02)

    print()
    ha = stats('A  silent', a_stamps, WINDOW)
    hb = stats('B  /cmd_vel at 20 Hz', b_stamps, WINDOW)
    print()

    if ha is None or hb is None:
        print('  inconclusive')
    elif hb > ha * 3:
        print(f'  VERDICT: sending to the board raises its publish rate '
              f'{hb / ha:.1f}x ({ha:.2f} -> {hb:.2f} Hz).')
        print('  The ESP32 only flushes its telemetry when the micro-ROS session')
        print('  has a reason to run. With no inbound traffic it idles, and the')
        print('  1 Hz we have been measuring is the floor, not the publish rate.')
    elif abs(hb - ha) / max(ha, 1e-9) < 0.3:
        print(f'  VERDICT: no meaningful difference ({ha:.2f} vs {hb:.2f} Hz).')
        print('  Inbound traffic is not the variable; look elsewhere.')
    else:
        print(f'  VERDICT: partial effect ({ha:.2f} -> {hb:.2f} Hz). Inconclusive.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
