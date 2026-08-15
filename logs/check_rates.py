#!/usr/bin/env python3
"""Measure every ESP32 topic, and read the board's own self-report.

Run with NOTHING publishing to the board. That is the case the 1 Hz fault
appeared in: loop() blocked in rclc_executor_spin_some until a message arrived,
so telemetry only moved when we talked to the board first.

/rover_diag closes the loop that took a day to close by inference:
    x = loop() Hz as the board measures it
    y = free heap KB
    z = agent state (3 = AGENT_CONNECTED)
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=100)
WINDOW = 15.0

TOPICS = [
    ('/wheel_state', Vector3,    20.0, 'velL, velR, cmd vx'),
    ('/wheel_ticks', Quaternion, 20.0, 'cumulative counts LF LR RF RR'),
    ('/wheel_odom',  Vector3,    20.0, 'x, y, theta on-board'),
    ('/rover_diag',  Vector3,     1.0, 'loop Hz, heap KB, agent state'),
    ('/vo/odom',     Odometry,   25.0, 'cuVSLAM pose'),
    ('/gyro/base',   Imu,        50.0, 'IMU re-framed'),
]


class Rates(Node):
    def __init__(self):
        super().__init__('check_rates')
        self.t = {n: [] for n, _, _, _ in TOPICS}
        self.last = {}
        for name, typ, _, _ in TOPICS:
            self.create_subscription(
                typ, name, self._mk(name), BE)

    def _mk(self, name):
        def cb(m):
            self.t[name].append(time.time())
            self.last[name] = m
        return cb


def main():
    rclpy.init()
    n = Rates()
    for _ in range(30):
        rclpy.spin_once(n, timeout_sec=0.05)
    for k in n.t:
        n.t[k] = []

    print(f'\n  measuring for {WINDOW:.0f} s, publishing NOTHING to the board\n')
    end = time.time() + WINDOW
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.005)

    print('  topic           msgs     rate      want    jitter        verdict')
    print('  ' + '-' * 70)
    fails = []
    for name, _, want, _desc in TOPICS:
        st = n.t[name]
        if len(st) < 2:
            print(f'  {name:<14} {len(st):5d}        -    >={want:5.1f}          '
                  f'   NO DATA')
            fails.append(name)
            continue
        hz = (len(st) - 1) / (st[-1] - st[0])
        gaps = [b - a for a, b in zip(st, st[1:])]
        mean = sum(gaps) / len(gaps)
        sd = (sum((g - mean) ** 2 for g in gaps) / len(gaps)) ** 0.5
        ok = hz >= want * 0.75
        if not ok:
            fails.append(name)
        print(f'  {name:<14} {len(st):5d}  {hz:7.2f} Hz  >={want:5.1f}  '
              f'{sd * 1000:6.1f} ms      {"OK" if ok else "LOW"}')

    d = n.last.get('/rover_diag')
    print()
    if d is None:
        print('  /rover_diag never arrived — cannot read loop() rate')
    else:
        states = {0: 'WAITING_AGENT', 1: 'AGENT_AVAILABLE',
                  2: 'AGENT_CONNECTED', 3: 'AGENT_DISCONNECTED'}
        print('  THE BOARD\'S OWN REPORT')
        print(f'    loop() rate    {d.x:9.1f} Hz')
        print(f'    free heap      {d.y:9.0f} KB')
        print(f'    agent state    {int(d.z)} = {states.get(int(d.z), "?")}')
        print()
        if d.x >= 200:
            print('    loop() is free-running. The executor fix worked: it no longer')
            print('    blocks waiting for a message, so the 50 ms telemetry timer')
            print('    fires on its own schedule.')
        elif d.x >= 30:
            print(f'    loop() at {d.x:.0f} Hz — running, but slower than the ~1000 Hz')
            print('    delay(1) allows. Enough for 20 Hz telemetry; something is')
            print('    still costing time (reliable-publish ACK waits are the suspect).')
        else:
            print(f'    loop() at {d.x:.1f} Hz — still starved. The timeout-0 change')
            print('    did not take effect; the block is elsewhere.')

    t = n.last.get('/wheel_ticks')
    if t is not None:
        print(f'\n  /wheel_ticks now: LF {t.x:.0f}  LR {t.y:.0f}  '
              f'RF {t.z:.0f}  RR {t.w:.0f}')
    o = n.last.get('/wheel_odom')
    if o is not None:
        import math
        print(f'  /wheel_odom  now: x {o.x * 100:.1f} cm  y {o.y * 100:.1f} cm  '
              f'th {math.degrees(o.z):.2f} deg')

    print()
    if not fails:
        print('  ALL RATES OK — no keepalive needed.')
    else:
        print('  LOW/MISSING: ' + ', '.join(fails))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
