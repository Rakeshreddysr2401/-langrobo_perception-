#!/usr/bin/env python3
# cuVSLAM ROS 2 (Jazzy) wrapper node — the Orin-native replacement for the
# Thor-only isaac_ros_visual_slam GEM. Runs the standalone pyCuVSLAM (cu12 wheel)
# on the D555 stereo topics in FULL SLAM MODE (loop closure + pose-graph
# optimization + persistent map) and publishes the contract nvblox/nav2 expect:
#   * TF   odom -> base_link   (visual odometry, smooth, high rate)
#   * TF   map  -> odom        (SLAM correction; identity until first loop
#                               closure / relocalization, then jumps to keep
#                               map->base_link globally consistent)
#   * topic /visual_slam/tracking/odometry + /odom (nav_msgs/Odometry)
#   * topic /slam/status (std_msgs/String, JSON, 1 Hz) — lc/pgo status etc.
#   * srv   /slam/save_map (Trigger)  — persist SLAM db to <map_dir>
#   * srv   /slam/localize (Trigger)  — relocalize in the saved <map_dir>
#
# Frame handling: cuVSLAM tracks in the left-IR optical frame, with its world
# frame == the rig pose at t0 (optical convention, y-down). We re-express the
# result in base_link so `odom` is anchored to base_link at t0 (gravity-aligned,
# z-up) — required for nvblox's 2D ESDF height slice and nav2:
#     odom_from_base(t) = B · world_from_rig(t) · B^-1,     B = base_link <- left_optical (static)
# At t0 world_from_rig = I  =>  odom_from_base = I (robot at origin). ✔
# The SLAM pose lives in the same world convention, so:
#     map_from_base(t) = B · slam_from_rig(t) · B^-1
#     map_from_odom(t) = map_from_base(t) · odom_from_base(t)^-1
#
# Run inside the unified container (needs cuVSLAM cu12 libs on LD_LIBRARY_PATH).
import json
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nvblox_msgs.srv import FilePath as NvbloxFilePath
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import message_filters
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
import cuvslam as vslam
from cuvslam import pycuvslam


def mat_from_pose(translation, quat_xyzw) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = Rotation.from_quat(list(quat_xyzw)).as_matrix()
    M[:3, 3] = list(translation)
    return M


def tf_msg(M: np.ndarray, stamp, parent: str, child: str) -> TransformStamped:
    q = Rotation.from_matrix(M[:3, :3]).as_quat()
    t = M[:3, 3]
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x = float(t[0])
    tf.transform.translation.y = float(t[1])
    tf.transform.translation.z = float(t[2])
    tf.transform.rotation.x = float(q[0])
    tf.transform.rotation.y = float(q[1])
    tf.transform.rotation.z = float(q[2])
    tf.transform.rotation.w = float(q[3])
    return tf


class CuvslamNode(Node):
    def __init__(self) -> None:
        super().__init__('cuvslam_node')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('left_optical_frame', 'camera0_infra1_optical_frame')
        self.declare_parameter('left_ns', '/camera/camera0/infra1')
        self.declare_parameter('right_ns', '/camera/camera0/infra2')
        self.declare_parameter('enable_slam', True)
        self.declare_parameter('map_dir', '/maps/current')   # /maps = host volume
        self.declare_parameter('planar_constraints', True)   # floor rover
        self.declare_parameter('max_map_size', 300)          # keyframes
        # 8GB Orin is CPU-tight: keep SBA off the track thread and throttle
        # loop-closure searches — LC bursts at revisit moments starved the
        # tracker and exploded odometry (2 pose explosions, 2026-07-19 night)
        self.declare_parameter('async_sba', True)
        self.declare_parameter('lc_throttle_ms', 2000)
        # relocalization search window around the guess pose
        self.declare_parameter('loc_search_radius_m', 2.0)
        self.declare_parameter('loc_search_vertical_m', 0.5)
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.optical_frame = self.get_parameter('left_optical_frame').value
        self.enable_slam = bool(self.get_parameter('enable_slam').value)
        self.map_dir = self.get_parameter('map_dir').value
        left_ns = self.get_parameter('left_ns').value
        right_ns = self.get_parameter('right_ns').value

        self.bridge = CvBridge()
        self.tracker = None
        self.left_info = None
        self.right_info = None
        self.B = None          # base_link <- left_optical (4x4, static)
        self.Binv = None
        self.n_tracked = 0
        self.map_from_odom = np.eye(4)      # identity until SLAM corrects
        self.last_slam_ok = False
        self.last_pair = None               # (ts, limg, rimg) for relocalize
        self.lc_count = 0                   # loop closures seen (status)
        self._track_lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        # plain /odom too (drive_test / tooling expect it)
        self.odom_pub2 = self.create_publisher(Odometry, '/odom', 10)
        self.status_pub = self.create_publisher(String, '/slam/status', 10)

        track_group = MutuallyExclusiveCallbackGroup()
        srv_group = ReentrantCallbackGroup()
        self.create_subscription(CameraInfo, f'{left_ns}/camera_info', self._left_info_cb, 10,
                                 callback_group=track_group)
        self.create_subscription(CameraInfo, f'{right_ns}/camera_info', self._right_info_cb, 10,
                                 callback_group=track_group)
        left_sub = message_filters.Subscriber(self, Image, f'{left_ns}/image_rect_raw',
                                              qos_profile=qos_profile_sensor_data,
                                              callback_group=track_group)
        right_sub = message_filters.Subscriber(self, Image, f'{right_ns}/image_rect_raw',
                                               qos_profile=qos_profile_sensor_data,
                                               callback_group=track_group)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=10, slop=0.01)
        self.sync.registerCallback(self._stereo_cb)

        self.create_service(Trigger, '/slam/save_map', self._save_map_cb,
                            callback_group=srv_group)
        self.create_service(Trigger, '/slam/localize', self._localize_cb,
                            callback_group=srv_group)
        # nvblox owns the WALLS the user sees — save its map alongside ours
        self.nvblox_save_cli = self.create_client(NvbloxFilePath, '/nvblox_node/save_map',
                                                  callback_group=srv_group)
        self.create_timer(1.0, self._status_tick, callback_group=srv_group)
        self.get_logger().info(
            f'cuvslam_node up (SLAM={"ON" if self.enable_slam else "off"}, '
            f'map_dir={self.map_dir}) — waiting for camera_info + stereo pairs')

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
            async_sba=bool(self.get_parameter('async_sba').value),
            enable_final_landmarks_export=False,
            enable_observations_export=False, rectified_stereo_camera=True)
        slam_cfg = None
        if self.enable_slam:
            slam_cfg = vslam.Tracker.SlamConfig()
            slam_cfg.planar_constraints = bool(self.get_parameter('planar_constraints').value)
            slam_cfg.max_map_size = int(self.get_parameter('max_map_size').value)
            slam_cfg.sync_mode = False        # loop closure on a worker thread
            slam_cfg.throttling_time_ms = int(self.get_parameter('lc_throttle_ms').value)
        self.tracker = vslam.Tracker(rig, cfg, slam_cfg)
        self.get_logger().info(
            f'cuVSLAM tracker built: {li.width}x{li.height} fx={fx:.1f} baseline={baseline:.4f} m'
            f' SLAM={"ON (planar)" if slam_cfg is not None else "off"}')
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
        limg = np.ascontiguousarray(
            self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8'))
        rimg = np.ascontiguousarray(
            self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8'))
        ts = int(left_msg.header.stamp.sec * 1_000_000_000 + left_msg.header.stamp.nanosec)
        try:
            with self._track_lock:
                est, slam_pose = self.tracker.track(ts, (limg, rimg))
        except Exception as e:
            self.get_logger().error(f'track() failed: {e}', throttle_duration_sec=2.0)
            return
        self.last_pair = (ts, limg, rimg)
        if est.world_from_rig is None:
            self.get_logger().warn('pose invalid', throttle_duration_sec=2.0)
            return
        p = est.world_from_rig.pose
        Mrig = mat_from_pose(p.translation, p.rotation)     # odom0(optical) <- optical(t)
        odom_from_base = self.B @ Mrig @ self.Binv          # anchor odom to base_link(t0)

        if slam_pose is not None:
            Mslam = mat_from_pose(slam_pose.translation, slam_pose.rotation)
            map_from_base = self.B @ Mslam @ self.Binv
            new_corr = map_from_base @ np.linalg.inv(odom_from_base)
            jump = np.linalg.norm(new_corr[:3, 3] - self.map_from_odom[:3, 3])
            if jump > 0.05:
                self.lc_count += 1
                self.get_logger().info(
                    f'SLAM correction jump {jump:.3f} m (loop closure / pgo)')
            self.map_from_odom = new_corr
            self.last_slam_ok = True
        else:
            self.last_slam_ok = False

        self._publish(odom_from_base, left_msg.header.stamp)
        self.n_tracked += 1
        if self.n_tracked % 90 == 0:
            t = odom_from_base[:3, 3]
            c = self.map_from_odom[:3, 3]
            self.get_logger().info(
                f'tracked={self.n_tracked} base@odom=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m '
                f'map->odom corr=({c[0]:+.3f},{c[1]:+.3f},{c[2]:+.3f}) m slam={self.last_slam_ok}')

    def _publish(self, M: np.ndarray, stamp) -> None:
        self.tf_broadcaster.sendTransform([
            tf_msg(self.map_from_odom, stamp, self.map_frame, self.odom_frame),
            tf_msg(M, stamp, self.odom_frame, self.base_frame),
        ])
        q = Rotation.from_matrix(M[:3, :3]).as_quat()
        t = M[:3, 3]
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

    # ----- SLAM services ---------------------------------------------------
    # Service bodies are fully wrapped: an exception here must NEVER kill the
    # node — tracking dying mid-run leaves nav blind (learned 2026-07-19: the
    # first /slam/localize call crashed the whole node on a TypeError).
    def _save_map_cb(self, req, res):
        try:
            return self._save_map_impl(req, res)
        except Exception as e:
            res.success = False
            res.message = f'save_map error: {e}'
            self.get_logger().error(res.message)
            return res

    def _save_map_impl(self, req, res):
        if self.tracker is None or not self.enable_slam:
            res.success = False
            res.message = 'SLAM not running'
            return res
        done = threading.Event()
        result = {'ok': False}

        def cb(ok):
            result['ok'] = bool(ok)
            done.set()

        self.tracker.save_map(self.map_dir, cb)
        if not done.wait(timeout=60.0):
            res.success = False
            res.message = f'save_map timed out (60 s) for {self.map_dir}'
            return res
        res.success = result['ok']
        res.message = (f'map saved to {self.map_dir}' if result['ok']
                       else f'cuVSLAM failed to save map to {self.map_dir}')
        if result['ok'] and self.nvblox_save_cli.service_is_ready():
            req2 = NvbloxFilePath.Request()
            req2.file_path = f'{self.map_dir}/nvblox_map.nvblx'
            fut = self.nvblox_save_cli.call_async(req2)
            deadline = threading.Event()
            fut.add_done_callback(lambda _f: deadline.set())
            if deadline.wait(timeout=30.0) and fut.result() is not None and fut.result().success:
                res.message += ' + nvblox walls (nvblox_map.nvblx)'
            else:
                res.message += ' (nvblox wall save FAILED)'
        self.get_logger().info(res.message)
        return res

    def _localize_cb(self, req, res):
        try:
            return self._localize_impl(req, res)
        except Exception as e:
            res.success = False
            res.message = f'localize error: {e}'
            self.get_logger().error(res.message)
            return res

    def _localize_impl(self, req, res):
        if self.tracker is None or not self.enable_slam:
            res.success = False
            res.message = 'SLAM not running'
            return res
        if self.last_pair is None:
            res.success = False
            res.message = 'no stereo frame yet'
            return res
        ts, limg, rimg = self.last_pair
        # guess = current pose in map frame, expressed as a rig pose (optical)
        map_from_rig = self.map_from_odom @ self.B  # odom_from_base≈latest; coarse guess is fine
        gq = Rotation.from_matrix(map_from_rig[:3, :3]).as_quat()
        guess = vslam.Pose(rotation=list(gq), translation=list(map_from_rig[:3, 3]))
        r = float(self.get_parameter('loc_search_radius_m').value)
        v = float(self.get_parameter('loc_search_vertical_m').value)
        # NOTE: must be the NESTED Slam.LocalizationSettings type —
        # Tracker.SlamLocalizationSettings aliases a different class the
        # binding rejects. start_cb must be a real callable, not None.
        settings = pycuvslam.Slam.LocalizationSettings(
            horizontal_search_radius=r, vertical_search_radius=v,
            horizontal_step=0.25, vertical_step=0.15, angular_step_rads=0.5)
        done = threading.Event()
        result = {'pose': None, 'err': ''}

        def started(*_):
            self.get_logger().info(f'relocalization search started in {self.map_dir}')

        def finish(pose, err):
            result['pose'] = pose
            result['err'] = str(err) if err else ''
            done.set()

        with self._track_lock:
            self.tracker.localize_in_map(self.map_dir, ts, guess, (limg, rimg),
                                         settings, started, finish)
        if not done.wait(timeout=90.0):
            res.success = False
            res.message = 'localize_in_map timed out (90 s)'
            return res
        if result['pose'] is None:
            res.success = False
            res.message = f'relocalization failed: {result["err"]}'
            self.get_logger().warn(res.message)
            return res
        res.success = True
        res.message = f'relocalized in {self.map_dir}'
        self.get_logger().info(res.message)
        return res

    def _status_tick(self):
        st = {'slam': self.enable_slam, 'tracking': self.n_tracked > 0,
              'frames': self.n_tracked, 'slam_pose_ok': self.last_slam_ok,
              'corrections': self.lc_count,
              'map_from_odom_xy': [round(float(self.map_from_odom[0, 3]), 3),
                                   round(float(self.map_from_odom[1, 3]), 3)]}
        if self.tracker is not None and self.enable_slam:
            try:
                m = self.tracker.get_slam_metrics()
                st['lc_status'] = bool(m.lc_status)
                st['pgo_status'] = bool(m.pgo_status)
                st['lc_landmarks'] = int(m.lc_good_landmarks_count)
            except Exception:
                pass
        self.status_pub.publish(String(data=json.dumps(st)))


def main() -> None:
    rclpy.init()
    node = CuvslamNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
