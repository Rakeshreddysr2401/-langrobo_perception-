#!/usr/bin/env python3
"""lidar_odom_node.py — LiDAR odometry, live (SENSOR_FUSION_PLAN.md M2).

    /scan + /gyro/base  ->  /lidar/odom          nav_msgs/Odometry, frame lidar_odom
                            /lidar/odom_status   JSON: fit quality, failures, ms/scan

The algorithm is phase1/nodes/lidar_odom.py, the same class the harness grades
offline, so what was measured on the bags is what runs here.

It publishes NO TF. odom -> base_link stays with the fusion until M3 decides
how the two combine. Two owners of one transform is how TF goes wrong.

Each scan is held until the gyro has covered its whole sweep (~0.12 s), so the
de-skew sees every beam's rotation, exactly as offline.

COVARIANCE, from the fit rather than a constant:
    position  sigma^2 = residual^2 / min translation eigenvalue, floor 1 cm^2
    heading   sigma   = 0.3 deg for a good fit
    a failed fit (prediction only) is published with 1 m / 10 deg, so a
    consumer treats it as the guess it is.
"""
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lidar_odom import LidarOdom  # noqa: E402

HOLD_S = 0.12      # wait this long past a scan's stamp for the gyro to cover it


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class LidarOdomNode(Node):
    def __init__(self):
        super().__init__('lidar_odom')
        self.lo = None
        self.pending = deque()
        self.gyro_t = None
        self.ms = deque(maxlen=100)
        self.pub = self.create_publisher(Odometry, '/lidar/odom', 10)
        self.pub_status = self.create_publisher(String, '/lidar/odom_status', 10)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/gyro/base', self._gyro, qos_profile_sensor_data)
        self.create_timer(1.0, self._status)

        # the laser mount, once, from robot_state_publisher (description/)
        buf = Buffer()
        tl = TransformListener(buf, self)
        end = time.time() + 15.0
        while not buf.can_transform('base_link', 'laser', Time()) and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not buf.can_transform('base_link', 'laser', Time()):
            tl.unregister()
            raise SystemExit('no base_link -> laser: ./rover lidar starts it')
        t = buf.lookup_transform('base_link', 'laser', Time()).transform
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tl.unregister()
        self.lo = LidarOdom((t.translation.x, t.translation.y, yaw))
        self.get_logger().info(f'laser mount x {t.translation.x:.4f} y {t.translation.y:+.4f} '
                               f'yaw {math.degrees(yaw):+.2f} deg; de-skew dir {self.lo.dir} ref {self.lo.ref}')

    def _gyro(self, m):
        if self.lo is None:
            return
        t = stamp_s(m.header)
        self.lo.on_gyro(t, m.angular_velocity.z)
        self.gyro_t = t
        # release every scan the gyro now covers
        while self.pending and self.gyro_t >= self.pending[0][0] + HOLD_S:
            self._process(*self.pending.popleft())

    def _scan(self, m):
        if self.lo is None:
            return
        self.pending.append((stamp_s(m.header), m.header, np.asarray(m.ranges, dtype=np.float64),
                             m.angle_min, m.angle_increment, m.scan_time))
        while len(self.pending) > 20:            # gyro gone: do not grow without bound
            self.pending.popleft()

    def _process(self, t, header, ranges, amin, ainc, st):
        t0 = time.perf_counter()
        f = self.lo.on_scan(t, ranges, amin, ainc, st)
        self.ms.append((time.perf_counter() - t0) * 1000)
        if f is None:
            return
        o = Odometry()
        o.header.stamp = header.stamp
        o.header.frame_id = 'lidar_odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.lo.x, self.lo.y
        o.pose.pose.orientation.z = math.sin(self.lo.th / 2)
        o.pose.pose.orientation.w = math.cos(self.lo.th / 2)
        if f.ok:
            var_xy = max(1e-4, f.residual ** 2 / max(f.eig, 1e-6) * 100.0)
            var_th = math.radians(0.3) ** 2
        else:
            var_xy, var_th = 1.0, math.radians(10.0) ** 2
        c = [0.0] * 36
        c[0] = c[7] = var_xy
        c[14] = c[21] = c[28] = 1e6          # z, roll, pitch: not estimated
        c[35] = var_th
        o.pose.covariance = c
        self.pub.publish(o)

    def _status(self):
        if self.lo is None or self.lo.last is None:
            return
        f = self.lo.last
        ms = np.array(self.ms) if self.ms else np.array([0.0])
        self.pub_status.publish(String(data=json.dumps({
            'ok': bool(f.ok), 'why': f.why, 'residual_cm': round(f.residual * 100, 2),
            'inliers': f.inliers, 'min_eig': round(f.eig, 1), 'failed_total': self.lo.n_fail,
            'ms_median': round(float(np.median(ms)), 1), 'ms_p95': round(float(np.percentile(ms, 95)), 1),
            'pending': len(self.pending),
            'pose': [round(self.lo.x, 4), round(self.lo.y, 4), round(math.degrees(self.lo.th), 2)]})))


def main():
    rclpy.init()
    n = LidarOdomNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
