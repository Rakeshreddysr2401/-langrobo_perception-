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
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

Q = (-0.5, 0.5, -0.5, 0.5)  # base_link <- camera0_motion_optical_frame (xyzw)

# PERFORMANCE (2026-08-10): the D555 IMU streams at ~199 Hz, and the old callback
# built four numpy arrays and did four np.cross calls PER SAMPLE — 37% of a core
# on a 6-core Orin that is already perception-bound. That mattered: when CPU
# saturates, the D555's DDS reader callbacks fall behind ("callback took too
# long!") and the IR stereo pair collapses from ~22 Hz to ~1 Hz, which starves
# cuVSLAM until it freezes and stops publishing /odom entirely. Cheap nodes are a
# safety property here, not just tidiness.
#
# Q is constant, so the rotation is a fixed 3x3 matrix — precompute it once and
# apply it with plain float arithmetic (no allocation) in the callback.
def _quat_to_matrix(q):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)),
        (2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)),
    )


class ImuToBase(Node):
    def __init__(self):
        super().__init__("imu_to_base")
        # EKF runs at 20 Hz (config/ekf.yaml); 199 Hz of gyro is far more than it
        # can use. 100 Hz is still 5x oversampled and halves the work again.
        self.declare_parameter("max_rate_hz", 100.0)
        r = float(self.get_parameter("max_rate_hz").value)
        self._min_dt = (1.0 / r) if r > 0 else 0.0
        self._last_pub = 0.0
        self._R = _quat_to_matrix(Q)
        self.pub = self.create_publisher(Imu, "/imu/base", qos_profile_sensor_data)
        self.create_subscription(Imu, "/camera/camera0/motion/sample",
                                 self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f"imu_to_base: /camera/camera0/motion/sample -> /imu/base "
            f"(base_link, host-clock stamp, capped at {r:.0f} Hz)")

    def _rot(self, vx, vy, vz):
        (a, b, c), (d, e, f), (g, h, i) = self._R
        return (a * vx + b * vy + c * vz,
                d * vx + e * vy + f * vz,
                g * vx + h * vy + i * vz)

    def cb(self, m):
        now = time.monotonic()
        if self._min_dt and (now - self._last_pub) < self._min_dt:
            return
        self._last_pub = now
        o = Imu()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "base_link"
        a = m.angular_velocity
        l = m.linear_acceleration
        (o.angular_velocity.x, o.angular_velocity.y,
         o.angular_velocity.z) = self._rot(a.x, a.y, a.z)
        (o.linear_acceleration.x, o.linear_acceleration.y,
         o.linear_acceleration.z) = self._rot(l.x, l.y, l.z)
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
