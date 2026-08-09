#!/usr/bin/env python3
"""DEPRECATED (2026-08-10) — DO NOT RUN. Kept only as calibration history.

Retired for two reasons:

  1. Its premise is dead. Firmware v2 (firmware/rover_firmware_v2.ino) runs a
     50 Hz CLOSED-LOOP PID on encoder velocity (MAX_WHEEL_VEL 0.86 m/s) with
     gMinDuty 0.08 for static-friction breakaway — exactly the job this node
     was invented to do, now done properly at the motor.
  2. It was destroying navigation. Re-flooring every nonzero command to
     |vx|>=0.20 and |wz|>=0.80 meant MPPI's fine corrections (e.g. vx 0.02,
     wz 0.05) reached the wheels as (0.21, 0.80) — a hard swerve. The
     controller had no fine authority left, so it oscillated: overshoot,
     correct, overshoot the other way. That was the rover's "drunken" weave.

It also sat on /cmd_vel_nav, which (per nav2_bringup's remappings) carries the
RAW controller_server output — so it forwarded un-smoothed, collision-monitor-
BYPASSING commands straight to the wheels. See config/nav2.yaml collision_monitor.

Original notes follow.
--------------------------------------------------------------------------
Motor dead-zone compensation: /cmd_vel_nav -> /cmd_vel_shim (-> safety_guard -> /cmd_vel).

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
VX_MAX = 0.30     # firmware full PWM (input scale)
VX_OUT = 0.22     # SAFETY CAP: max linear actually sent — slow moves only
WZ_IN = 1.0       # MPPI wz_max (nav2_real.yaml)
WZ_FLOOR = 0.80   # min effective angular command
WZ_OUT = 0.90   # SAFETY CAP (was 1.40): slow rotations only — tethered rover     # max angular actually sent (README rotation-test value)


def _rescale(v, floor, in_max, out_max):
    if v == 0.0:
        return 0.0
    mag = min(abs(v), in_max)
    out = floor + (out_max - floor) * (mag / in_max)
    return out if v > 0 else -out


class Deadband(Node):
    def __init__(self):
        super().__init__('cmd_vel_deadband')
        self._pub = self.create_publisher(Twist, '/cmd_vel_shim', 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self._cb, 10)

    def _cb(self, m):
        m.linear.x = _rescale(m.linear.x, VX_FLOOR, VX_MAX, VX_OUT)
        m.angular.z = _rescale(m.angular.z, WZ_FLOOR, WZ_IN, WZ_OUT)
        self._pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Deadband())


if __name__ == '__main__':
    main()
