#!/usr/bin/env python3
"""fusion2_node.py — fusion2, live, beside fusion.py (SENSOR_FUSION_PLAN.md M3).

    /gyro/base, /wheel_ticks, /cmd_vel, /vo/odom, /vo/status, /lidar/odom
        -> /fused2/odom     nav_msgs/Odometry, frame odom2, with covariance
           /fused2/status   JSON: flags, accepted / rejected per source

The estimator is phase1/nodes/fusion2.py, the class graded offline, so what
was measured on the recordings is what runs here.

SINCE 2026-09-26 (the owner's switch, after the live return test) this is
what the rover uses: ./rover fused runs it with publish_tf:=true on /odom,
frame odom, owning odom -> base_link. fusion.py keeps running beside it as a
fallback on /odom_legacy without TF; FUSION=1 ./rover fused swaps them back.
Defaults below are the side-by-side mode, so running it bare cannot steal TF.

A /lidar/odom fix is stamped with its SCAN time and arrives ~0.13 s later.
fusion2 carries it forward by its own motion since then (motion only), so
passing the scan stamp is what makes the late fix land correctly.
A /lidar/odom whose covariance is the failed-fit value (1 m) is a failed fit.
"""
import json
import math
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fusion2 import Fusion2  # noqa: E402


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Fusion2Node(Node):
    def __init__(self):
        super().__init__('fusion2')
        self.f = Fusion2()
        self.landmarks, self.vo_healthy = 100.0, True
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('odom_topic', '/fused2/odom')
        self.declare_parameter('frame_id', 'odom2')
        self.frame = self.get_parameter('frame_id').value
        self.tf = TransformBroadcaster(self) if self.get_parameter('publish_tf').value else None
        self.pub = self.create_publisher(Odometry, self.get_parameter('odom_topic').value, 10)
        self.pub_status = self.create_publisher(String, '/fused2/status', 10)
        self.create_subscription(Imu, '/gyro/base', self._gyro, qos_profile_sensor_data)
        self.create_subscription(Quaternion, '/wheel_ticks', self._ticks, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self._cmd, 10)
        self.create_subscription(Odometry, '/vo/odom', self._vo, qos_profile_sensor_data)
        self.create_subscription(String, '/vo/status', self._vo_status, 10)
        self.create_subscription(Odometry, '/lidar/odom', self._lidar, 10)
        # NO TIMERS. Output is driven from the gyro callback: every 10th sample
        # (20 Hz) publishes /odom + TF, every 200th the status. On 2026-09-26
        # rclpy's single-threaded executor starved this node's timers while the
        # subscriptions kept flowing -- both timers ~0.5-0.8 s overdue and never
        # called, the estimate current, /odom and odom -> base_link silent. Tied
        # to the gyro, the pose is published exactly when it advances, and if
        # the gyro stops, the pose honestly stops with it.
        self.n_gyro = 0
        self.get_logger().info(f'fusion2 up: VO mode {self.f.VO_MODE}; publishing '
                               f'{self.get_parameter("odom_topic").value} in {self.frame}, '
                               f'TF {"odom -> base_link" if self.tf else "none"}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _gyro(self, m):
        self.f.on_gyro(stamp_s(m.header), m.angular_velocity.z)
        self.n_gyro += 1
        if self.n_gyro % 10 == 0:
            self._publish(m.header.stamp)
        if self.n_gyro % 200 == 0:
            self._status()

    def _ticks(self, m):
        self.f.on_ticks(self._now(), (m.x, m.y, m.z, m.w))   # no header: stamped on arrival

    def _cmd(self, m):
        self.f.on_cmd(self._now(), m.linear.x, m.angular.z)

    def _vo_status(self, m):
        try:
            j = json.loads(m.data)
        except ValueError:
            return
        self.landmarks = float(j.get('landmarks', self.landmarks))
        self.vo_healthy = bool(j.get('healthy', True))

    def _vo(self, m):
        p = m.pose.pose
        self.f.on_vo(stamp_s(m.header), (p.position.x, p.position.y, yaw_of(p.orientation)),
                     self.landmarks, self.vo_healthy)

    def _lidar(self, m):
        p = m.pose.pose
        c = m.pose.covariance
        ok = c[0] < 0.5                       # the node publishes 1.0 for a failed fit
        self.f.on_lidar(stamp_s(m.header), (p.position.x, p.position.y, yaw_of(p.orientation)),
                        ok, max(c[0], 0.005 ** 2), c[35])

    def _publish(self, stamp):
        if self.f.t is None:
            return
        x, y, th = self.f.pose
        o = Odometry()
        now = stamp                           # the gyro sample's own time: the state's time
        o.header.stamp = now
        o.header.frame_id = self.frame
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        P = self.f.P
        cov = [0.0] * 36
        cov[0], cov[1], cov[5] = P[0, 0], P[0, 1], P[0, 2]
        cov[6], cov[7], cov[11] = P[1, 0], P[1, 1], P[1, 2]
        cov[30], cov[31], cov[35] = P[2, 0], P[2, 1], P[2, 2]
        cov[14] = cov[21] = cov[28] = 1e6
        o.pose.covariance = cov
        o.twist.twist.linear.x, o.twist.twist.linear.y = float(self.f.X[3]), float(self.f.X[4])
        o.twist.twist.angular.z = float(self.f.gz_last - self.f.X[5])
        self.pub.publish(o)
        if self.tf is not None:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.frame
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation.z, tf.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
            self.tf.sendTransform(tf)

    def _status(self):
        if self.f.t is None:
            return
        self.pub_status.publish(String(data=json.dumps({
            'flags': {k: (round(v, 3) if isinstance(v, float) else v) for k, v in self.f.flags.items()},
            'accepted': self.f.accepted, 'rejected': self.f.rejected,
            'gyro_bias': round(float(self.f.X[5]), 5),
            'sd_cm': round(math.sqrt(max(self.f.P[0, 0], self.f.P[1, 1])) * 100, 2),
            'sd_deg': round(math.degrees(math.sqrt(self.f.P[2, 2])), 3)})))


def main():
    rclpy.init()
    n = Fusion2Node()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
