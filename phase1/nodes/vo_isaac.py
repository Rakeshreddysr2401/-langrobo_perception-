#!/usr/bin/env python3
"""
vo_isaac.py — the rover's side of the packaged Isaac ROS cuVSLAM.

WHAT IT IS
    vo_node.py drove the standalone pyCuVSLAM wheel itself. From orin-nav:1.2 on
    the tracker is Isaac ROS 4.6's isaac_ros_visual_slam (launch/vo_isaac.launch.py)
    and this node keeps the contract everything downstream was built against:

        /vo/odom   nav_msgs/Odometry   pose of base_link in the odom frame --
                                       published BY THE ISAAC NODE (remapped),
                                       not here
        /vo/status std_msgs/String     the same JSON keys vo_node published
        TF         map -> odom         SLAM's correction, flattened and guarded
                                       (slam:=true only)

    fusion2 reads landmarks / healthy, health.py landmarks, the harness both.

WHY THE POSES DO NOT PASS THROUGH PYTHON
    The first version re-published every pose: 22 % of a core on its own, next
    to the tracker's 27 % -- twice vo_node's 26 % for the same job, on a rover
    whose #1 problem is CPU. rclpy's per-message cost is the price, mostly
    building messages with 72 covariance doubles. So the Isaac node publishes
    /vo/odom itself (C++), and every subscription here is RAW: the few fields
    needed -- stamp, pose, a point count, vo_state -- are read straight from
    the CDR bytes. Each topic's first message is also fully deserialized and
    compared; on any mismatch that topic falls back to normal decoding.

THE ONE BEHAVIOUR THAT CHANGED
    vo_node withheld a pose whose base_link z was past VO_MAX_Z_ODOM (a ground
    rover cannot be there). The Isaac node publishes it. So the moment one is
    seen, /vo/status goes out at once with healthy=false -- fusion2 drops VO
    while it is unhealthy -- instead of waiting for the 1 Hz report.

THE FRAMES
    The Isaac node tracks in two frames this node publishes under camera0_link:

        vslam_left_optical    camera0_link's origin (the LEFT imager), with
                              vo_node's axis swap: optical x right, y down, z fwd
        vslam_right_optical   + baseline along optical x

    camera0_link comes from robot_state_publisher (description/params.yaml,
    calibrated against LiDAR); the baseline from infra2's camera_info as
    |P[3]/P[0]| -- the D555 driver's sign is the opposite of REP-104
    (vo_node.py), which is why the driver's own infra TF is not used. The
    conjugation vo_node did by hand (B . R . B^-1) is TF's job here.

WHAT IT GUARDS (vo_node.py's guards, same thresholds, imported from there)
    - base_link z past VO_MAX_Z_ODOM for Z_BAD_GRACE_S: the tracker has
      diverged -> /visual_slam/reset.
    - SLAM corrections are projected onto the floor (x, y, yaw), then rejected
      past MAX_CORRECTION_M planar.
"""
import json
import math
import struct
import time

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, PointCloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from isaac_ros_visual_slam_interfaces.srv import Reset

from vo_node import VO_MAX_Z_ODOM, Z_BAD_GRACE_S, MAX_CORRECTION_M, MAX_CORRECTION_Z

# optical (x right, y down, z fwd) -> camera0_link (x fwd, y left, z up), the M
# of vo_node.base_from_optical: columns are the optical axes in link axes.
M_OPTICAL = np.array([[0.0, 0.0, 1.0],
                      [-1.0, 0.0, 0.0],
                      [0.0, -1.0, 0.0]])
VS = '/visual_slam'


# ── raw CDR reading ─────────────────────────────────────────────────────────
# Little-endian XCDR1, as the RMW serializes on this Orin: a 4-byte
# encapsulation header, then fields aligned to their size relative to the end
# of that header. Every message here starts with std_msgs/Header.
def _align(o, n):
    return o + (-(o - 4) % n)


def _header(b):
    sec, nsec = struct.unpack_from('<iI', b, 4)
    n = struct.unpack_from('<I', b, 12)[0]            # frame_id, length incl. NUL
    return sec, nsec, 16 + n


def raw_odometry(b):
    """(sec, nsec, (px, py, pz, qx, qy, qz, qw)) from a nav_msgs/Odometry."""
    sec, nsec, o = _header(b)
    o = _align(o, 4)
    o += 4 + struct.unpack_from('<I', b, o)[0]        # child_frame_id
    return sec, nsec, struct.unpack_from('<7d', b, _align(o, 8))


def raw_cloud_points(b):
    _, _, o = _header(b)
    h, w = struct.unpack_from('<II', b, _align(o, 4))
    return h * w


def raw_vo_state(b):
    _, _, o = _header(b)
    return b[o]


def pose_matrix(p):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(p[3:7]).as_matrix()
    T[:3, 3] = p[:3]
    return T


class VoIsaac(Node):
    def __init__(self):
        super().__init__('vo_isaac')
        self.declare_parameter('camera_ns', '/camera/camera0')
        self.declare_parameter('camera_frame', 'camera0_link')
        self.declare_parameter('slam', False)
        g = lambda n: self.get_parameter(n).value
        self.ns, self.cam_frame, self.want_slam = g('camera_ns'), g('camera_frame'), g('slam')

        self.frames_up = False
        self.frames = self.tracked = self.last_frames = 0
        self.rate, self.last_report = 0.0, time.time()
        self.landmarks = 0
        self.vo_state = 0
        self.last_odom_wall = None
        self.prev_ts, self.max_gap_s, self.gaps_over_limit = None, 0.0, 0
        self.z_bad_since, self.resets, self.reset_pending = None, 0, False
        self.frozen, self.last_pose = 0, None
        self.correction_m = 0.0
        self.loop_closures, self._last_lc_n = 0, 0
        self.correction_rejected = self.correction_flattened = 0
        # odom poses by stamp, to pair each SLAM pose with the odometry pose of
        # the same frame
        self.odom_by_stamp = {}
        self.raw_ok = {}       # topic -> raw reader verified against deserialization

        self.static_tf = StaticTransformBroadcaster(self)
        self.tf_map = TransformBroadcaster(self) if self.want_slam else None
        self.pub_status = self.create_publisher(String, '/vo/status', 1)
        self.reset_cli = self.create_client(Reset, f'{VS}/reset')

        self.create_subscription(CameraInfo, f'{self.ns}/infra2/camera_info',
                                 self._right_info, qos_profile_sensor_data)
        self._raw(Odometry, '/vo/odom', self._odom, qos_profile_sensor_data)
        self._raw(VisualSlamStatus, f'{VS}/status', self._status)
        self._raw(PointCloud2, f'{VS}/vis/landmarks_cloud', self._landmarks)
        if self.want_slam:
            self._raw(Odometry, f'{VS}/vis/slam_odometry', self._slam)
            self._raw(PointCloud2, f'{VS}/vis/loop_closure_cloud', self._lc)
        self.create_timer(1.0, self._report)
        self.get_logger().info(f'vo_isaac up — waiting for {self.ns}/infra2/camera_info '
                               f'(slam={"on" if self.want_slam else "off"})')

    def _raw(self, typ, topic, handler, qos=10):
        """Subscribe raw; check the raw reader once against the real decoder."""
        reader, field = {
            Odometry: (raw_odometry, lambda m: (m.header.stamp.sec, m.header.stamp.nanosec, (
                m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z,
                m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                m.pose.pose.orientation.z, m.pose.pose.orientation.w))),
            PointCloud2: (raw_cloud_points, lambda m: m.width * m.height),
            VisualSlamStatus: (raw_vo_state, lambda m: m.vo_state),
        }[typ]

        def cb(b):
            ok = self.raw_ok.get(topic)
            if ok is None:
                real = field(deserialize_message(b, typ))
                try:
                    ok = b[1] == 1 and reader(b) == real
                except (struct.error, IndexError):
                    ok = False
                self.raw_ok[topic] = ok
                if not ok:
                    self.get_logger().error(f'raw read of {topic} disagrees with the decoder '
                                            '-- decoding it normally (slower)')
            handler(reader(b) if ok else field(deserialize_message(b, typ)))
        self.create_subscription(typ, topic, cb, qos, raw=True)

    # ── the frames the tracker works in ─────────────────────────────────────
    def _right_info(self, m):
        if self.frames_up:
            return
        signed = -m.p[3] / m.p[0] if m.p[0] else 0.0
        baseline = abs(signed)
        if not 0.01 < baseline < 1.0:
            self.get_logger().error(
                f'implausible baseline {signed:+.4f} m from camera_info — '
                'infra2/camera_info is wrong, not just sign-flipped',
                throttle_duration_sec=10.0)
            return
        q = Rotation.from_matrix(M_OPTICAL).as_quat()
        now = self.get_clock().now().to_msg()
        tfs = []
        # the right imager sits +baseline along optical x = -y in camera0_link
        for child, y in (('vslam_left_optical', 0.0), ('vslam_right_optical', -baseline)):
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = now, self.cam_frame, child
            t.transform.translation.y = y
            t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
            t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
            tfs.append(t)
        self.static_tf.sendTransform(tfs)
        self.frames_up = True
        self.get_logger().info(f'optical frames up under {self.cam_frame}: '
                               f'baseline {baseline * 100:.2f} cm')

    # ── odometry (read, not re-published) ───────────────────────────────────
    def _odom(self, r):
        sec, nsec, p = r
        self.frames += 1
        self.last_odom_wall = time.time()
        ts = sec + nsec * 1e-9
        if self.prev_ts is not None:
            gap = ts - self.prev_ts
            self.max_gap_s = max(self.max_gap_s, gap)
            if gap > 1.0:
                self.gaps_over_limit += 1
                self.get_logger().warning(f'frame gap {gap:.2f} s — tracking may reset.')
        self.prev_ts = ts
        if self.vo_state == 1:
            self.tracked += 1

        # A ground rover cannot leave the floor: a metre of z means the
        # tracker's state is wrong, whatever its landmark count (vo_node.py).
        if abs(p[2]) > VO_MAX_Z_ODOM:
            now = time.time()
            if self.z_bad_since is None:
                self.z_bad_since = now
                self.get_logger().error(
                    f'cuvslam base_link z = {p[2]:+.2f} m — a ground rover cannot be '
                    f'there. landmarks={self.landmarks} worst_frame_gap='
                    f'{self.max_gap_s * 1000:.0f} ms gaps_over_1s={self.gaps_over_limit}')
                self._report()                    # healthy=false NOW, not in up to 1 s
            elif now - self.z_bad_since > Z_BAD_GRACE_S and not self.reset_pending:
                self.get_logger().error(
                    f'DIVERGED for {Z_BAD_GRACE_S:.0f}s — resetting the tracker. '
                    'The odom origin resets and any map built in it is invalid.')
                self._reset()
            return
        if self.z_bad_since is not None:
            self.get_logger().warning('cuvslam z recovered on its own')
            self.z_bad_since = None

        self.frozen = self.frozen + 1 if p == self.last_pose else 0
        self.last_pose = p
        if self.want_slam:
            self.odom_by_stamp[(sec, nsec)] = p
            if len(self.odom_by_stamp) > 300:
                self.odom_by_stamp.pop(next(iter(self.odom_by_stamp)))   # oldest

    def _reset(self):
        if not self.reset_cli.service_is_ready():
            self.get_logger().error(f'{VS}/reset is not available', throttle_duration_sec=10.0)
            return
        self.reset_pending = True

        def done(_):
            self.reset_pending = False
            self.z_bad_since, self.prev_ts, self.last_pose = None, None, None
            self.odom_by_stamp.clear()
            self.resets += 1
        self.reset_cli.call_async(Reset.Request()).add_done_callback(done)

    def _status(self, vo_state):
        self.vo_state = vo_state

    def _landmarks(self, n):
        self.landmarks = n

    # ── map -> odom ─────────────────────────────────────────────────────────
    def _lc(self, n):
        if n > self._last_lc_n:
            self.loop_closures += 1
        self._last_lc_n = n

    def _slam(self, r):
        """map_from_odom = map_from_base . odom_from_base^-1, then the floor
        projection and the plausibility gate of vo_node._publish_correction --
        see the reasoning there; the thresholds are the same constants."""
        sec, nsec, p = r
        p_odom = self.odom_by_stamp.get((sec, nsec))
        if p_odom is None:
            return
        corr = pose_matrix(p) @ np.linalg.inv(pose_matrix(p_odom))
        z_raw = float(corr[2, 3])
        corr[2, 3] = 0.0
        yaw = math.atan2(corr[1, 0], corr[0, 0])
        mag = float(np.linalg.norm(corr[:3, 3]))
        if abs(z_raw) > MAX_CORRECTION_Z:
            self.correction_flattened += 1
            self.get_logger().warning(
                f'SLAM correction had z={z_raw:+.2f} m — flattened to the floor; '
                f'using its {mag:.2f} m planar part. ({self.correction_flattened} so far)',
                throttle_duration_sec=30.0)
        if mag > MAX_CORRECTION_M:
            self.correction_rejected += 1
            self.get_logger().warning(
                f'SLAM correction implausible ({mag:.1f} m planar, z was {z_raw:+.1f} m) '
                '— not publishing map->odom. Odometry is unaffected.',
                throttle_duration_sec=10.0)
            return
        self.correction_m = mag
        t = TransformStamped()
        t.header.stamp.sec, t.header.stamp.nanosec = sec, nsec
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x, t.transform.translation.y = float(corr[0, 3]), float(corr[1, 3])
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf_map.sendTransform(t)

    # ── /vo/status, vo_node's keys ──────────────────────────────────────────
    def _report(self):
        now = time.time()
        dt = now - self.last_report
        if dt > 0.5:                     # an early report must not distort the rate
            self.rate = (self.frames - self.last_frames) / dt
            self.last_report, self.last_frames = now, self.frames
        up = self.last_odom_wall is not None and now - self.last_odom_wall < 2.0
        self.pub_status.publish(String(data=json.dumps({
            'rate_hz': round(self.rate, 2),
            'frames': self.frames,
            'tracked': self.tracked,
            'tracker_up': up,
            'landmarks': self.landmarks,
            'slam': self.want_slam,
            'correction_m': round(self.correction_m, 3),
            'loop_closures': self.loop_closures,
            'tracker_resets': self.resets,
            'worst_frame_gap_ms': round(self.max_gap_s * 1000, 1),
            'gaps_over_1s': self.gaps_over_limit,
            'correction_rejected': self.correction_rejected,
            'correction_flattened': self.correction_flattened,
            'frozen_frames': self.frozen,
            # Rate alone is NOT enough: a dead tracker can still publish.
            # Landmarks, the tracker's own state and a floor-bound z say it is
            # really tracking.
            'healthy': (up and self.vo_state == 1 and self.rate >= 10.0
                        and self.landmarks > 20 and self.z_bad_since is None),
            'backend': 'isaac_ros_visual_slam',
            'vo_state': self.vo_state,
            'raw_read_ok': self.raw_ok,
        })))


def main():
    rclpy.init()
    node = VoIsaac()
    # The default executor rebuilds its wait set on every spin: at the ~90
    # callbacks/s this node takes (SLAM on: ~150), that bookkeeping was most of
    # its CPU. The events executor dispatches per callback instead.
    try:
        from rclpy.experimental import EventsExecutor
        ex = EventsExecutor()
    except ImportError:
        ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
