#!/usr/bin/env python3
"""Passively measure /wheel_state with best-effort QoS. Sends nothing.

Run the SAME script on the Pi 5 (beside the micro-ROS agent) and on the Jetson
at the same time. The agent turns the ESP32's UDP into DDS locally on the Pi 5,
so the Pi 5 reading is the rate the board actually achieves; the Jetson reading
is what survives the WiFi DDS hop. If they differ, the loss is in that hop and
has nothing to do with the firmware.

Usage: listen_wheel.py [seconds]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Vector3

# /wheel_state is created with rclc_publisher_init_best_effort. A default
# (reliable) subscription is QoS-incompatible and receives nothing -- which is
# how `ros2 topic hz` produced misleading numbers here.
BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=100)


class Listen(Node):
    def __init__(self):
        super().__init__('listen_wheel_%d' % (time.time() % 100000))
        self.create_subscription(Vector3, '/wheel_state', self.cb, BEST_EFFORT)
        self.t = []

    def cb(self, _m):
        self.t.append(time.time())


def main():
    window = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    n = Listen()
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)
    n.t = []
    end = time.time() + window
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)

    k = len(n.t)
    if k < 2:
        print('RESULT msgs=%d  (too few)' % k)
        rclpy.shutdown()
        return
    gaps = [b - a for a, b in zip(n.t, n.t[1:])]
    mean = sum(gaps) / len(gaps)
    sd = (sum((g - mean) ** 2 for g in gaps) / len(gaps)) ** 0.5
    hz = (k - 1) / (n.t[-1] - n.t[0])
    # how many gaps look like a clean 50 ms publish period vs a whole second
    fast = sum(1 for g in gaps if g < 0.10)
    slow = sum(1 for g in gaps if g > 0.80)
    print('RESULT msgs=%d  rate=%.3f Hz  gap=%.1f+/-%.1f ms  '
          'gaps<100ms=%d  gaps>800ms=%d  min=%.1f ms  max=%.1f ms'
          % (k, hz, mean * 1000, sd * 1000, fast, slow,
             min(gaps) * 1000, max(gaps) * 1000))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
