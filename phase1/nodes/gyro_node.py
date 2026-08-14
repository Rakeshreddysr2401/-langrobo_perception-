#!/usr/bin/env python3
"""
gyro_node.py — the D555's gyro, expressed in base_link.

The IMU sits inside the camera, so it reports rotation in the camera's OPTICAL
axes (x right, y down, z forward). A yaw of the rover is therefore a rotation
about the gyro's -y. This node applies the same axis swap vo_node.py uses and
republishes, so everything downstream sees one convention.

    /gyro/base   sensor_msgs/Imu   angular velocity in base_link, frame_id=base_link

ONLY THE GYRO IS FORWARDED. The accelerometer is passed through untouched but
should not be integrated for position: on a slow ground robot, gravity leakage
and bias dominate the real signal and the result diverges by hundreds of metres.
Its honest use is levelling (finding "down"), not odometry.

The topic name differs between RealSense driver versions and modules, so this
node discovers it: it looks for any sensor_msgs/Imu topic under the camera
namespace rather than hard-coding one.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

# optical (x right, y down, z fwd) -> base (x fwd, y left, z up); see vo_node.py
M = np.array([[0.0, 0.0, 1.0],
              [-1.0, 0.0, 0.0],
              [0.0, -1.0, 0.0]])


class GyroNode(Node):
    def __init__(self):
        super().__init__('gyro_node')
        self.declare_parameter('camera_ns', '/camera/camera0')
        self.declare_parameter('imu_topic', '')   # empty = discover
        self.ns = self.get_parameter('camera_ns').value
        self.forced = self.get_parameter('imu_topic').value

        self.pub = self.create_publisher(Imu, '/gyro/base', qos_profile_sensor_data)
        self.sub = None
        self.n = 0
        self.create_timer(1.0, self._ensure_subscribed)

    def _ensure_subscribed(self):
        if self.sub is not None:
            return
        topic = self.forced
        if not topic:
            cands = [n for n, types in self.get_topic_names_and_types()
                     if n.startswith(self.ns) and 'sensor_msgs/msg/Imu' in types]
            if not cands:
                self.get_logger().warn(f'no sensor_msgs/Imu topic under {self.ns} yet', once=True)
                return
            # Prefer a combined/motion stream over a bare gyro stream if both exist.
            cands.sort(key=lambda n: (0 if ('motion' in n or n.endswith('/imu')) else 1, len(n)))
            topic = cands[0]
        self.sub = self.create_subscription(Imu, topic, self._on_imu, qos_profile_sensor_data)
        self.get_logger().info(f'gyro source: {topic}')

    def _on_imu(self, m):
        w = M @ np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        a = M @ np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        out = Imu()
        out.header.stamp = m.header.stamp
        out.header.frame_id = 'base_link'
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = map(float, w)
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = map(float, a)
        out.orientation_covariance[0] = -1.0   # we publish no orientation
        self.pub.publish(out)
        self.n += 1


def main():
    rclpy.init()
    node = GyroNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
