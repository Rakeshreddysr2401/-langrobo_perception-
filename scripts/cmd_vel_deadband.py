#!/usr/bin/env python3
"""Motor dead-zone compensation: /cmd_vel_nav -> /cmd_vel.

The ESP32 firmware maps commanded m/s linearly onto PWM (0.30 m/s = 100%),
open loop. The L298N + gearmotors need roughly >=60% PWM to break static
friction (measured 2026-07-16: 0.30 moves the rover even on a mattress,
0.07-0.15 only hums). MPPI's approach speeds therefore stalled every
mission a few tens of cm from the goal.

This node rescales every nonzero command onto the effective range:

    |vx| in (0, VX_MAX]  ->  [VX_FLOOR, VX_MAX]   (sign preserved)
    |wz| in (0, WZ_IN ]  ->  [WZ_FLOOR, WZ_OUT]   (sign preserved)
    exact zeros pass through as zeros (stop stays stop)

Nav2 keeps planning in ideal velocities; visual odometry closes the loop on
the (faster) real motion. The right long-term fix is the same remap inside
rover_firmware.ino (Pi5 repo) — delete this node and set the collision
monitor's cmd_vel_out_topic back to "cmd_vel" once the firmware is
reflashed.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

VX_FLOOR = 0.20   # min effective linear command (~65% PWM)
VX_MAX = 0.30     # firmware full PWM
WZ_IN = 1.0       # MPPI wz_max (nav2_real.yaml)
WZ_FLOOR = 0.80   # min effective angular command
WZ_OUT = 1.40     # max angular actually sent (README rotation-test value)


def _rescale(v, floor, in_max, out_max):
    if v == 0.0:
        return 0.0
    mag = min(abs(v), in_max)
    out = floor + (out_max - floor) * (mag / in_max)
    return out if v > 0 else -out


class Deadband(Node):
    def __init__(self):
        super().__init__('cmd_vel_deadband')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self._cb, 10)

    def _cb(self, m):
        m.linear.x = _rescale(m.linear.x, VX_FLOOR, VX_MAX, VX_MAX)
        m.angular.z = _rescale(m.angular.z, WZ_FLOOR, WZ_IN, WZ_OUT)
        self._pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Deadband())


if __name__ == '__main__':
    main()
