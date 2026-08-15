#!/usr/bin/env python3
"""Measure the ESP32's loop() rate from the outside, over the network.

WHY. /wheel_state arrives at a metronomic 1.000 Hz while the firmware's only
telemetry path is EXECUTE_EVERY_N_MS(50,...) = 20 Hz. Two very different faults
produce that identical symptom:

  A. loop() itself is running at ~1 Hz, so the 50 ms timer only gets a chance
     to fire once a second.
  B. loop() runs fast and really does publish at 20 Hz, and something
     downstream -- the UDP link, the agent, DDS -- delivers only one of them.

Reading the source cannot tell these apart. This can.

HOW. The firmware echoes targetVx straight back:

    cmdVelCb()  : targetVx = m->linear.x          (executor, loop() context)
    loop()      : wheelStateMsg.z = targetVx      (published telemetry)

So /cmd_vel -> /wheel_state.z is a round trip through loop(). We publish a
sequence number encoded in linear.x at 20 Hz and decode, from every telemetry
message that comes back, which command the board had consumed by then.

  lag stays ~0        -> the executor is keeping up -> loop() is fast -> fault B
  lag grows ~20/s     -> one command consumed per second -> loop() is at 1 Hz
                         -> fault A

SAFETY. pidStep() returns 0 and zeroes the integrator whenever |target| < 0.01
(firmware line 182), and we never exceed 0.004 m/s. Both sides therefore get
exactly zero duty and the motors cannot turn. angular.z is held at 0 throughout,
and we send an explicit zero command on the way out.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Vector3

# /wheel_state is created with rclc_publisher_init_best_effort, so a default
# (reliable) subscription is QoS-incompatible and silently receives nothing.
BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)

STEP = 2e-5        # m/s per sequence number -- 200 steps stays under 0.004
RATE = 20.0        # Hz, matching the firmware's intended telemetry rate
SECONDS = 12.0
N = int(RATE * SECONDS)


class Probe(Node):
    def __init__(self):
        super().__init__('probe_loop_rate')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Vector3, '/wheel_state', self.on_wheel, BEST_EFFORT)
        self.rx = []
        self.i = 0
        self.t0 = None

    def on_wheel(self, m):
        if self.t0 is None:
            return
        self.rx.append((time.time() - self.t0, m.z, m.x, m.y, self.i))

    def send(self, i):
        t = Twist()
        t.linear.x = i * STEP
        t.angular.z = 0.0
        self.pub.publish(t)


def main():
    rclpy.init()
    n = Probe()

    # let the subscription match before we start the clock
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)

    print(f'  probing for {SECONDS:.0f} s: /cmd_vel at {RATE:.0f} Hz, '
          f'seq encoded as linear.x = seq * {STEP:g}')
    print('  max commanded velocity %.4f m/s -- below the 0.01 dead zone, '
          'motors stay still\n' % (N * STEP))

    n.t0 = time.time()
    period = 1.0 / RATE
    for i in range(1, N + 1):
        n.i = i
        n.send(i)
        deadline = n.t0 + i * period
        while time.time() < deadline:
            rclpy.spin_once(n, timeout_sec=0.002)

    # drain, then park the target back at zero
    for _ in range(30):
        rclpy.spin_once(n, timeout_sec=0.02)
    z = Twist()
    for _ in range(5):
        n.pub.publish(z)
        time.sleep(0.02)

    print('  t(s)    seq_sent  seq_seen   lag(cmds)   lag(s)     velL     velR')
    print('  ' + '-' * 68)
    prev_t = None
    gaps = []
    for t, zval, vl, vr, sent in n.rx:
        seen = int(round(zval / STEP))
        lag = sent - seen
        print(f'  {t:6.2f}  {sent:8d}  {seen:8d}   {lag:9d}   {lag / RATE:6.2f}   '
              f'{vl:+7.3f}  {vr:+7.3f}')
        if prev_t is not None:
            gaps.append(t - prev_t)
        prev_t = t

    print()
    if len(n.rx) < 2:
        print('  only %d telemetry messages arrived -- cannot conclude' % len(n.rx))
        rclpy.shutdown()
        return

    hz = (len(n.rx) - 1) / (n.rx[-1][0] - n.rx[0][0])
    mean_gap = sum(gaps) / len(gaps)
    var = sum((g - mean_gap) ** 2 for g in gaps) / len(gaps)
    print(f'  telemetry: {len(n.rx)} msgs, {hz:.3f} Hz, '
          f'gap {mean_gap * 1000:.1f} ms +/- {var ** 0.5 * 1000:.1f} ms')

    first, last = n.rx[0], n.rx[-1]
    seen_first = int(round(first[1] / STEP))
    seen_last = int(round(last[1] / STEP))
    span = last[0] - first[0]
    consumed = (seen_last - seen_first) / span if span > 0 else 0.0
    print(f'  commands consumed by the ESP32: {consumed:.2f} per second '
          f'(we sent {RATE:.0f} per second)')
    print()

    lag_first = first[4] - seen_first
    lag_last = last[4] - seen_last
    growth = (lag_last - lag_first) / span if span > 0 else 0.0
    print(f'  lag at start {lag_first} cmds, at end {lag_last} cmds, '
          f'growing {growth:+.1f} cmds/s')
    print()
    print('  VERDICT')
    if consumed >= RATE * 0.5:
        print('    The executor is draining /cmd_vel at close to the rate we send it,')
        print('    so loop() is running fast and the 50 ms telemetry timer must be')
        print('    firing. The messages are being lost AFTER the board publishes them.')
        print('    => fault is in the transport (client stream, agent, or DDS), not loop().')
    elif consumed <= 3.0:
        print('    The executor consumed only ~%.1f commands per second while we sent'
              % consumed)
        print('    %.0f. rclc_executor_spin_some() takes at most one message per')
        print('    subscription per call, so this is loop() itself iterating at ~1 Hz.')
        print('    => fault is INSIDE loop(): something in it blocks for ~1 s.')
    else:
        print('    Intermediate: %.1f commands/s consumed. Inconclusive; see the'
              % consumed)
        print('    per-message table above for the pattern.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
