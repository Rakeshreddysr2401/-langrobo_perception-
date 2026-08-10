#!/usr/bin/env python3
"""Re-frame + re-stamp the D555 IMU so robot_localization can fuse its gyro.

The EKF was outputting exactly 0 yaw-rate during real turns — the gyro was
IGNORED. Two reasons, both fixed here:
  1. The IMU is in the camera's 'optical' frame (z = forward). A real YAW turn
     shows up on the optical Y axis, not Z, and robot_localization was not
     transforming it into the robot frame. We rotate the angular-velocity (and
     accel) vector into base_link ourselves, so its z IS the true yaw rate.
  2. The IMU stamps lag the odom/host clock ~130-250ms. We stamp with now().

Publishes /imu/base (frame_id base_link). Point the EKF's imu0 at it. Run inside
the isaac_ros container. Rotation Q is the static base_link<-optical transform
(verified live 2026-07-18).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

Q = (-0.5, 0.5, -0.5, 0.5)  # base_link <- camera0_motion_optical_frame (xyzw)


def rot(q, v):
    x, y, z, w = q
    qv = np.array([x, y, z]); vv = np.array(v, dtype=float)
    t = 2.0 * np.cross(qv, vv)
    return vv + w * t + np.cross(qv, t)


class ImuToBase(Node):
    def __init__(self):
        super().__init__("imu_to_base")
        self.pub = self.create_publisher(Imu, "/imu/base", qos_profile_sensor_data)
        self.create_subscription(Imu, "/camera/camera0/motion/sample",
                                 self.cb, qos_profile_sensor_data)
        self.get_logger().info("imu_to_base: /camera/camera0/motion/sample "
                               "-> /imu/base (base_link, host-clock stamp)")

    def cb(self, m):
        o = Imu()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "base_link"
        av = rot(Q, [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        la = rot(Q, [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        o.angular_velocity.x, o.angular_velocity.y, o.angular_velocity.z = map(float, av)
        o.linear_acceleration.x, o.linear_acceleration.y, o.linear_acceleration.z = map(float, la)
        # valid diagonal covariances so robot_localization accepts the data;
        # orientation unavailable on the RealSense IMU (-1).
        o.orientation_covariance[0] = -1.0
        for i in (0, 4, 8):
            o.angular_velocity_covariance[i] = 0.01
            o.linear_acceleration_covariance[i] = 0.1
        self.pub.publish(o)


def main():
    rclpy.init()
    try:
        rclpy.spin(ImuToBase())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
