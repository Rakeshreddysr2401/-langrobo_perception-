#!/usr/bin/env python3
"""fusion_node.py — publish the fused pose that nav2 will navigate on.

WHAT IT OWNS
    /odom            nav_msgs/Odometry   the fused pose, with covariance
    TF odom -> base_link                 taken over from vo_node
    /fusion/path     nav_msgs/Path       where it has been, in `odom`
    /fusion/path_map nav_msgs/Path       the same track in `map` -- see below
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

TWO PATHS, AND WHY THE SECOND ONE EXISTS
    /fusion/path is stamped `odom`, and `odom` is continuous by definition: it
    never jumps, and it never gets corrected. So a loop closure -- which is the
    only thing that ever removes drift -- is invisible on it BY CONSTRUCTION.
    The rover can close a loop, take a 28 cm correction, and the green line on
    screen does not move by a pixel. That is not a rendering bug; it is what
    `odom` means.

    /fusion/path_map is the same track with each point transformed through
    map -> odom AT THE MOMENT IT WAS RECORDED. Old points therefore do NOT move
    when a new correction arrives, so the two lines separate exactly where the
    estimate was corrected, and by exactly how much. Drift stops being a number
    in a status string and becomes the gap between two lines you can measure
    on screen with the RViz ruler.

    Re-transforming the WHOLE path on every publish was the obvious alternative
    and it is useless: it slides the entire track rigidly, so the two lines stay
    parallel and the correction is invisible again in a new way.

    It is published only once a map -> odom transform actually exists. Before
    the first closure there is no `map` frame, and a Path stamped with a frame
    nothing publishes makes RViz raise a display error that reads exactly like
    a dead publisher.

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
from rclpy.time import Time as RclTime
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       DurabilityPolicy, ReliabilityPolicy, HistoryPolicy)
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

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
PATH_PUB_HZ = 2.0      # republish rate, independent of whether it moved

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
        # TRANSIENT_LOCAL: the path is state, not a stream. Without it an RViz
        # started after the rover has been driving shows an empty screen until
        # the next 2 cm of travel, and shows nothing at all if the rover is
        # parked -- which reads as a broken publisher rather than a design
        # choice about when to append.
        self.pub_path = self.create_publisher(
            Path, '/fusion/path',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self._path_anchor = None

        # The `map`-framed twin. Same QoS and the same reasoning: a viewer that
        # connects late must get the track it missed, not an empty screen.
        self.pub_path_map = self.create_publisher(
            Path, '/fusion/path_map',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))
        self.path_map = Path()
        self.path_map.header.frame_id = 'map'
        # Listening, not broadcasting. map -> odom belongs to vo_node's loop
        # closure; this node only reads it to place the corrected track.
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self._map_seen = False

        self.create_subscription(Odometry, '/vo/odom', self._vo, qos_profile_sensor_data)
        self.create_subscription(Imu, '/gyro/base', self._gyro, qos_profile_sensor_data)
        self.create_subscription(Vector3, '/wheel_state', self._wheels, qos_profile_sensor_data)
        self.create_subscription(Quaternion, '/wheel_ticks', self._ticks, qos_profile_sensor_data)
        self.create_subscription(String, '/vo/status', self._status, 10)

        self.create_timer(1.0 / PULSE_HZ, self._pulse)
        self.create_timer(1.0 / STATUS_HZ, self._report)
        # Appending and publishing are separate concerns: append only on real
        # travel so a parked rover does not grow the message, but publish
        # steadily so a viewer always has the current track.
        self.create_timer(1.0 / PATH_PUB_HZ, self._publish_path)
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
        self.f.on_vo(self._now(), p.x, p.y, yaw_of(m.pose.pose.orientation), p.z)

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
        self._append_path_map(ps)

    def _map_from_odom(self):
        """The live loop-closure correction, or None before the first closure.

        LATEST, not the pose's own stamp. The correction is a slow-moving frame
        offset published at closure rate, so asking for it at an exact past time
        forces TF to extrapolate between two points that may be seconds apart --
        which raises ExtrapolationException on a transform that is, physically,
        perfectly well defined. Time(0) means "the newest you have", and for a
        frame that changes only when a loop closes that is the right question.
        """
        try:
            t = self.tf_buf.lookup_transform('map', 'odom', RclTime())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        return t.transform

    def _append_path_map(self, ps):
        """Place one already-recorded pose into `map`, at today's correction."""
        tr = self._map_from_odom()
        if tr is None:
            return
        if not self._map_seen:
            self._map_seen = True
            self.get_logger().info(
                'map -> odom is live — /fusion/path_map now shows the '
                'loop-closure-corrected track')

        # Planar, like every correction this stack publishes. A 2D rotation and
        # an offset is the whole transform; pulling in tf2_geometry_msgs to do
        # it in 3D would add a dependency for arithmetic that is two lines.
        c = math.cos(yaw_of(tr.rotation))
        sn = math.sin(yaw_of(tr.rotation))
        x, y = ps.pose.position.x, ps.pose.position.y

        m = PoseStamped()
        m.header.stamp = ps.header.stamp
        m.header.frame_id = 'map'
        m.pose.position.x = tr.translation.x + c * x - sn * y
        m.pose.position.y = tr.translation.y + sn * x + c * y
        m.pose.position.z = 0.0
        qz, qw = quat_of(yaw_of(ps.pose.orientation) + yaw_of(tr.rotation))[2:]
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        self.path_map.poses.append(m)
        if len(self.path_map.poses) > PATH_MAX:
            self.path_map.poses.pop(0)
        self.path_map.header.stamp = ps.header.stamp

    def _publish_path(self):
        if self.path.poses:
            self.pub_path.publish(self.path)
        if self.path_map.poses:
            self.pub_path_map.publish(self.path_map)

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
                elif h.get('vo_implausible', 0) and h['vo_z'] and abs(h['vo_z']) > 0.3:
                    self.get_logger().warn(
                        f'cuvslam DIVERGED (z={h["vo_z"]:.1f} m) — ignoring it, '
                        f'running on wheels+gyro')
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
