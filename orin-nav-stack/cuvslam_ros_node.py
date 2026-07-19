#!/usr/bin/env python3
# cuVSLAM ROS 2 (Jazzy) wrapper node — the Orin-native replacement for the
# Thor-only isaac_ros_visual_slam GEM. Runs the standalone pyCuVSLAM (cu12 wheel)
# on the D555 stereo topics and publishes the same contract nvblox/nav2 expect:
#   * TF   odom -> base_link
#   * topic /visual_slam/tracking/odometry (nav_msgs/Odometry)
#
# Frame handling: cuVSLAM tracks in the left-IR optical frame, with its world
# frame == the rig pose at t0 (optical convention, y-down). We re-express the
# result in base_link so `odom` is anchored to base_link at t0 (gravity-aligned,
# z-up) — required for nvblox's 2D ESDF height slice and nav2:
#     odom_from_base(t) = B · world_from_rig(t) · B^-1,     B = base_link <- left_optical (static)
# At t0 world_from_rig = I  =>  odom_from_base = I (robot at origin). ✔
#
# Run inside the unified container (needs cuVSLAM cu12 libs on LD_LIBRARY_PATH).
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import message_filters
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
import cuvslam as vslam


def mat_from_pose(translation, quat_xyzw) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = Rotation.from_quat(list(quat_xyzw)).as_matrix()
    M[:3, 3] = list(translation)
    return M


class CuvslamNode(Node):
    def __init__(self) -> None:
        super().__init__('cuvslam_node')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('left_optical_frame', 'camera0_infra1_optical_frame')
        self.declare_parameter('left_ns', '/camera/camera0/infra1')
        self.declare_parameter('right_ns', '/camera/camera0/infra2')
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.optical_frame = self.get_parameter('left_optical_frame').value
        left_ns = self.get_parameter('left_ns').value
        right_ns = self.get_parameter('right_ns').value

        self.bridge = CvBridge()
        self.tracker = None
        self.left_info = None
        self.right_info = None
        self.B = None          # base_link <- left_optical (4x4, static)
        self.Binv = None
        self.n_tracked = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        # plain /odom too (drive_test / tooling expect it)
        self.odom_pub2 = self.create_publisher(Odometry, '/odom', 10)

        self.create_subscription(CameraInfo, f'{left_ns}/camera_info', self._left_info_cb, 10)
        self.create_subscription(CameraInfo, f'{right_ns}/camera_info', self._right_info_cb, 10)
        left_sub = message_filters.Subscriber(self, Image, f'{left_ns}/image_rect_raw',
                                              qos_profile=qos_profile_sensor_data)
        right_sub = message_filters.Subscriber(self, Image, f'{right_ns}/image_rect_raw',
                                               qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=10, slop=0.01)
        self.sync.registerCallback(self._stereo_cb)
        self.get_logger().info('cuvslam_node up — waiting for camera_info + stereo pairs')

    def _left_info_cb(self, msg): self.left_info = msg
    def _right_info_cb(self, msg): self.right_info = msg

    def _ensure_tracker(self) -> bool:
        if self.tracker is not None:
            return True
        if self.left_info is None or self.right_info is None:
            return False
        li, ri = self.left_info, self.right_info
        fx, fy, cx, cy = li.k[0], li.k[4], li.k[2], li.k[5]
        # baseline from the right projection matrix P[3] = -fx*baseline (sign on
        # the D555 driver is unreliable, so use magnitude; right cam is +x of left)
        baseline = abs(ri.p[3]) / (ri.p[0] if ri.p[0] else fx)

        left = vslam.Camera()
        left.distortion = vslam.Distortion(vslam.Distortion.Model.Pinhole)
        left.focal = (fx, fy); left.principal = (cx, cy); left.size = (li.width, li.height)

        right = vslam.Camera()
        right.distortion = vslam.Distortion(vslam.Distortion.Model.Pinhole)
        right.focal = (ri.k[0], ri.k[4]); right.principal = (ri.k[2], ri.k[5])
        right.size = (ri.width, ri.height)
        right.rig_from_camera = vslam.Pose(rotation=Rotation.identity().as_quat(),
                                           translation=[baseline, 0.0, 0.0])

        rig = vslam.Rig(); rig.cameras = [left, right]
        cfg = vslam.Tracker.OdometryConfig(
            async_sba=False, enable_final_landmarks_export=False,
            enable_observations_export=False, rectified_stereo_camera=True)
        self.tracker = vslam.Tracker(rig, cfg)
        self.get_logger().info(
            f'cuVSLAM tracker built: {li.width}x{li.height} fx={fx:.1f} baseline={baseline:.4f} m')
        return True

    def _ensure_static(self) -> bool:
        if self.B is not None:
            return True
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.optical_frame,
                                                rclpy.time.Time())
        except Exception:
            return False
        tr, q = t.transform.translation, t.transform.rotation
        self.B = mat_from_pose([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w])
        self.Binv = np.linalg.inv(self.B)
        self.get_logger().info(f'static {self.base_frame} <- {self.optical_frame} acquired')
        return True

    def _stereo_cb(self, left_msg: Image, right_msg: Image) -> None:
        if not self._ensure_tracker() or not self._ensure_static():
            return
        limg = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
        rimg = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')
        ts = int(left_msg.header.stamp.sec * 1_000_000_000 + left_msg.header.stamp.nanosec)
        est, _ = self.tracker.track(ts, (np.ascontiguousarray(limg), np.ascontiguousarray(rimg)))
        if est.world_from_rig is None:
            self.get_logger().warn('pose invalid', throttle_duration_sec=2.0)
            return
        p = est.world_from_rig.pose
        Mrig = mat_from_pose(p.translation, p.rotation)     # odom0(optical) <- optical(t)
        odom_from_base = self.B @ Mrig @ self.Binv          # anchor odom to base_link(t0)
        self._publish(odom_from_base, left_msg.header.stamp)
        self.n_tracked += 1
        if self.n_tracked % 90 == 0:
            t = odom_from_base[:3, 3]
            self.get_logger().info(f'tracked={self.n_tracked} '
                                   f'base@odom=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m')

    def _publish(self, M: np.ndarray, stamp) -> None:
        q = Rotation.from_matrix(M[:3, :3]).as_quat()   # x,y,z,w
        t = M[:3, 3]
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = float(t[0])
        tf.transform.translation.y = float(t[1])
        tf.transform.translation.z = float(t[2])
        tf.transform.rotation.x = float(q[0])
        tf.transform.rotation.y = float(q[1])
        tf.transform.rotation.z = float(q[2])
        tf.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(tf)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = float(t[0])
        od.pose.pose.position.y = float(t[1])
        od.pose.pose.position.z = float(t[2])
        od.pose.pose.orientation.x = float(q[0])
        od.pose.pose.orientation.y = float(q[1])
        od.pose.pose.orientation.z = float(q[2])
        od.pose.pose.orientation.w = float(q[3])
        self.odom_pub.publish(od)
        self.odom_pub2.publish(od)


def main() -> None:
    rclpy.init()
    node = CuvslamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
