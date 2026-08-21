#!/usr/bin/env python3
"""fusion_node.py — publish the fused pose that nav2 will navigate on.

WHAT IT OWNS
    /odom            nav_msgs/Odometry   the fused pose, with covariance
    TF odom -> base_link                 taken over from vo_node
    /fusion/path     nav_msgs/Path       where it has been -- the line in RViz
    /fusion/status   std_msgs/String     JSON health, for humans and for logging

WHY IT TAKES THE TF FROM vo_node
    Only one thing may publish a transform, and the raw cuVSLAM pose is the one
    that teleports. Measured 2026-08-21: 12 teleports in a single drive, one of
    them 651 cm in a single frame, with cuVSLAM ending 79.6 cm from a start it
    had returned to. nav2 consuming that would react violently to a position the
    rover was never in -- a safety problem, not an accuracy one. So the guarded
    estimate owns the transform and the raw one stays on /vo/odom for comparison.

    Launch vo_node with publish_tf:=false. Two publishers of odom -> base_link
    make TF non-deterministic, and the symptom is a robot that jitters between
    two poses with nothing in any log.

THE ALGORITHM IS NOT HERE
    It is in fusion.py, unchanged, so that the thing measured over two days of
    tape-measured runs is the thing that ships. compare.py grades the same
    module. See its docstring for what each sensor is trusted for and why.

COVARIANCE IS MEASURED, NOT GUESSED
    nav2 weights a pose by its covariance, so a confident wrong pose is worse
    than an honest uncertain one. These numbers come from the gates:

      healthy      2 cm, 1.0 deg   -- 2.5 cm hand-pushed, 4.9 cm driven hard
      degraded    10 cm, 3.0 deg   -- running on wheels+gyro with cuVSLAM blind
      no estimate   1 m,   90 deg  -- before the gyro bias is measured

    Degraded is not a guess either: 4.9 cm was achieved THROUGH 12 teleports
    with 101 frames dead-reckoned, so 10 cm is a conservative bound on that same
    condition rather than a number chosen to look safe.
"""
import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fusion import PoseFusion, LOW_LANDMARKS          # noqa: E402

PULSE_HZ = 20.0        # independent of any sensor; see PoseFusion.pulse
STATUS_HZ = 1.0

# The travelled path, for RViz. Appended in CHORDS, not per pulse: at 20 Hz a
# stationary rover would otherwise add 20 identical poses a second until the
# message is megabytes and RViz stutters. 2 cm is fine enough to see the shape
# of a room-sized drive and coarse enough that standing still costs nothing.
PATH_CHORD_M = 0.02
PATH_MAX = 5000        # ~100 m of travel at 2 cm; oldest dropped after that

# (position sigma, yaw sigma) per health state -- see the module docstring
COV_HEALTHY = (0.02, math.radians(1.0))
COV_DEGRADED = (0.10, math.radians(3.0))
COV_NONE = (1.0, math.radians(90.0))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_of(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_topic', '/odom')
        self.publish_tf = self.get_parameter('publish_tf').value
        topic = self.get_parameter('odom_topic').value

        self.f = PoseFusion()
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.pub = self.create_publisher(Odometry, topic, 10)
        self.pub_status = self.create_publisher(String, '/fusion/status', 10)
        self.pub_path = self.create_publisher(Path, '/fusion/path', 10)
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self._path_anchor = None

        self.create_subscription(Odometry, '/vo/odom', self._vo, qos_profile_sensor_data)
        self.create_subscription(Imu, '/gyro/base', self._gyro, qos_profile_sensor_data)
        self.create_subscription(Vector3, '/wheel_state', self._wheels, qos_profile_sensor_data)
        self.create_subscription(Quaternion, '/wheel_ticks', self._ticks, qos_profile_sensor_data)
        self.create_subscription(String, '/vo/status', self._status, 10)

        self.create_timer(1.0 / PULSE_HZ, self._pulse)
        self.create_timer(1.0 / STATUS_HZ, self._report)
        self._last_log = None

        self.get_logger().info(
            f'fusion_node up — publishing {topic}'
            + (' and TF odom -> base_link' if self.publish_tf else ' (TF disabled)')
            + '. Launch vo_node with publish_tf:=false.')

    # ── clock ──────────────────────────────────────────────────────────────
    def _now(self):
        s, ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns * 1e-9

    # ── inputs ─────────────────────────────────────────────────────────────
    def _vo(self, m):
        p = m.pose.pose.position
        self.f.on_vo(self._now(), p.x, p.y, yaw_of(m.pose.pose.orientation))

    def _gyro(self, m):
        self.f.on_gyro(self._now(),
                       m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                       m.linear_acceleration.x, m.linear_acceleration.y,
                       m.linear_acceleration.z)

    def _wheels(self, m):
        self.f.on_wheels(self._now(), m.x, m.y)

    def _ticks(self, m):
        self.f.on_ticks(self._now(), m.x, m.y, m.z, m.w)

    def _status(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        n = d.get('landmarks')
        if isinstance(n, (int, float)):
            self.f.on_landmarks(n)

    # ── output ─────────────────────────────────────────────────────────────
    def _pulse(self):
        t = self._now()
        self.f.pulse(t)
        h = self.f.health()

        if not h['ready']:
            sp, sy = COV_NONE
        elif (not h['vo_alive'] or not h['wheels_alive'] or not h['gyro_alive']
              or 0 <= h['landmarks'] < LOW_LANDMARKS):
            sp, sy = COV_DEGRADED
        else:
            sp, sy = COV_HEALTHY

        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = float(self.f.x)
        od.pose.pose.position.y = float(self.f.y)
        od.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quat_of(self.f.yaw)
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw

        # Body velocity, from the sensors that measure it directly rather than
        # by differencing a pose: forward speed from the wheels, yaw rate from
        # the gyro. nav2's controller uses these, and differencing a pose that
        # is itself filtered would feed the filter's own lag back into control.
        velL, velR = self.f.wheel_prev
        od.twist.twist.linear.x = float(0.5 * (velL + velR))
        if self.f.gyro_bias is not None:
            od.twist.twist.angular.z = float(self.f.gyro_xyz[2] - self.f.gyro_bias)

        # Diagonal only: x, y, z, roll, pitch, yaw. z/roll/pitch are large
        # because this is a planar estimate and claiming certainty about a
        # dimension nothing measures would be a lie to whatever consumes it.
        pv, yv = sp * sp, sy * sy
        cov = [0.0] * 36
        cov[0] = pv
        cov[7] = pv
        cov[14] = 1e6
        cov[21] = 1e6
        cov[28] = 1e6
        cov[35] = yv
        od.pose.covariance = cov
        tw = [0.0] * 36
        tw[0] = 0.01 ** 2
        tw[7] = 1e6
        tw[14] = 1e6
        tw[21] = 1e6
        tw[28] = 1e6
        tw[35] = math.radians(1.0) ** 2
        od.twist.covariance = tw
        self.pub.publish(od)

        if h['ready']:
            self._append_path(od)

        if self.tf and h['ready']:
            tf = TransformStamped()
            tf.header = od.header
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = od.pose.pose.position.x
            tf.transform.translation.y = od.pose.pose.position.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = od.pose.pose.orientation
            self.tf.sendTransform(tf)

    def _append_path(self, od):
        """Extend the travelled line, in chords so standing still costs nothing."""
        x, y = od.pose.pose.position.x, od.pose.pose.position.y
        if self._path_anchor is not None:
            ax, ay = self._path_anchor
            if math.hypot(x - ax, y - ay) < PATH_CHORD_M:
                return
        self._path_anchor = (x, y)

        ps = PoseStamped()
        ps.header = od.header
        ps.pose = od.pose.pose
        self.path.poses.append(ps)
        if len(self.path.poses) > PATH_MAX:
            self.path.poses.pop(0)
        self.path.header.stamp = od.header.stamp
        self.pub_path.publish(self.path)

    def _report(self):
        h = self.f.health()
        msg = String()
        msg.data = json.dumps({k: (round(v, 5) if isinstance(v, float) else v)
                               for k, v in h.items()})
        self.pub_status.publish(msg)

        # Log only when the health state CHANGES. A line every second is noise
        # nobody reads; a line the moment a sensor drops out is the one thing
        # worth finding in a log afterwards.
        state = (h['ready'], h['vo_alive'], h['wheels_alive'], h['gyro_alive'],
                 0 <= h['landmarks'] < LOW_LANDMARKS)
        if state != self._last_log:
            self._last_log = state
            if not h['ready']:
                self.get_logger().info('waiting for gyro bias — hold still')
            else:
                down = [k for k in ('vo', 'wheels', 'gyro') if not h[k + '_alive']]
                if down:
                    self.get_logger().warn(
                        f'{", ".join(down)} DOWN — carrying on with the rest')
                elif 0 <= h['landmarks'] < LOW_LANDMARKS:
                    self.get_logger().warn(
                        f'cuvslam blind ({h["landmarks"]} landmarks) — '
                        f'wheels+gyro only')
                else:
                    self.get_logger().info('all three sensors contributing')


def main():
    rclpy.init()
    n = FusionNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        if 'convert call argument' not in str(e):
            raise
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
