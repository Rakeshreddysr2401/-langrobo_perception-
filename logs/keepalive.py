#!/usr/bin/env python3
"""Publish zero /cmd_vel continuously so the ESP32's loop() keeps turning.

WHY THIS EXISTS. The board's loop() blocks in rclc_executor_spin_some() until a
message arrives, or ~1 s if none does. The telemetry publish sits after that
call, so /wheel_state comes out at the rate we talk TO the board:

    silent -> 1.0 Hz     10 Hz in -> 10.1 Hz out     40 Hz in -> 16.4 Hz out

During autonomous driving this never shows up, because nav2 publishes /cmd_vel
continuously. It only bites when nobody is driving -- which is exactly the
hand-pushed Phase 1 gates.

SAFETY. Every message is all-zero, so targetVx = targetWz = 0 and pidStep()
returns 0 for both sides. It cannot cause motion; it only keeps lastCmdMs fresh.

DO NOT RUN THIS WHILE TELEOPERATING. It publishes to /cmd_vel, so it would
interleave with real commands and make the rover stutter. It is for hand-pushed
measurement runs only. The permanent fix is in the firmware -- see TODO.md.
"""
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HZ = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0


class KeepAlive(Node):
    def __init__(self):
        super().__init__('wheel_keepalive')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.msg = Twist()          # all zeros, and stays that way
        self.create_timer(1.0 / HZ, self.tick)
        self.n = 0

    def tick(self):
        self.pub.publish(self.msg)
        self.n += 1


def main():
    rclpy.init()
    n = KeepAlive()
    n.get_logger().info(
        f'zero /cmd_vel at {HZ:.0f} Hz — keeps the ESP32 loop() turning so '
        f'/wheel_state stays fast. Motion is impossible: all fields are zero.')
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
