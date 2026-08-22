#!/usr/bin/env python3
"""
vo_node.py — stereo visual odometry for the D555, on this Orin.

WHAT IT IS
    A thin ROS 2 wrapper around the standalone pyCuVSLAM wheel (cuvslam 16.0.0).
    The packaged isaac_ros_visual_slam in this image is built for Thor and does
    not run here, so we drive the tracker library directly.

WHAT IT PUBLISHES
    /vo/odom   nav_msgs/Odometry   pose of base_link in the odom frame
    /vo/status std_msgs/String     JSON: tracking state, rate, frame count
    TF         odom -> base_link   (disable with publish_tf:=false)

SLAM / LOOP CLOSURE — off by default, on with slam:=true
    Loop closure exists to *hide* accumulated drift by snapping the pose back
    when it recognises a place. That is exactly the quantity Phase 1 is trying
    to measure: push the rover out 2 m and back, and the residual IS the drift.
    With SLAM enabled the tracker would quietly correct it at the moment you
    returned to the start, and the out-and-back test would report a great number
    while telling you nothing. Set slam:=true once Phase 1 has its numbers.

THE FRAME PROBLEM, WHICH IS THE WHOLE DIFFICULTY
    cuVSLAM tracks in the left IR camera's OPTICAL frame: x right, y DOWN,
    z forward. ROS wants base_link: x forward, y left, z UP. Its world frame is
    wherever the rig happened to be at t0.

    Let B = base_link <- left_optical, the fixed transform from the camera's
    optical frame to the robot body (a 90 deg axis swap, plus where the camera
    is bolted). Let R(t) = world_from_rig(t), what the tracker returns.

        odom_from_base(t) = B . R(t) . B^-1

    Derivation, because getting this wrong produces a plausible-looking pose
    rather than an error:
        S = rig_from_base = B^-1                      (constant)
        world_from_base(t) = R(t) . S
        odom := base(t0), and world == rig(t0) = S . base(t0)
        so   odom_from_world = S^-1
        thus odom_from_base(t) = S^-1 . R(t) . S = B . R(t) . B^-1
    At t=0, R=I, so odom_from_base=I and the robot starts at the origin.

    Run `vo_node.py --self-test` to check this numerically with no hardware.

CAMERA MOUNT — CONFIRM WITH A TAPE MEASURE
    cam_x / cam_z are where the camera sits relative to base_link, whose origin
    is ON THE GROUND directly under the rover's centre of rotation. These
    defaults are unverified on your rig. cam_x matters for Phase 1's 360 deg
    spin test (a camera mounted forward of the centre swings on a lever arm),
    and cam_z will matter for mapping later.
"""
import argparse
import json
import math
import sys
import time

import numpy as np

# Measured on the rover 2026-08-15. base_link's origin is on the GROUND at the
# centre of the four wheel contact patches, so cam_x is taken from the pivot, not
# from the front face of the body:
#     wheelbase (front axle to rear axle, one side) = 25.0 cm
#     camera lens back to the front axle line       =  4.5 cm
#     cam_x = 4.5 + 25.0/2                          = 17.0 cm
# This was 10.0 cm as a guess, i.e. 7 cm short.
CAM_X_DEFAULT = 0.170  # metres forward of base_link origin   -- MEASURED
CAM_Y_DEFAULT = 0.00   # metres left (camera on the centreline)
CAM_Z_DEFAULT = 0.163  # metres above the ground              -- confirmed

# The camera is not quite square on its bracket. Two straight 2 m hand pushes,
# 2026-08-15, with the heading changing by under 0.6 deg in each:
#     197.2 cm forward, 6.7 cm sideways  ->  1.95 deg
#     194.9 cm forward, 7.4 cm sideways  ->  2.17 deg
# Agreeing to 0.2 deg over two runs makes this a fixed mounting offset, not
# noise. A 4-wheeled rover cannot crab sideways, so the rover was travelling
# where it pointed and the CAMERA was rotated.
#
# A mount yaw is invisible in the two numbers you check first: it does not
# affect distance travelled, and it cancels out of any measured rotation. It
# only tilts the reported DIRECTION of travel — which is why it survived a
# passing scale gate.
# A loop-closure correction is accumulated drift made visible. On a ground rover
# over a room that is centimetres to a metre or two -- never tens of metres, and
# never vertical. Beyond these the SLAM optimiser has diverged and the correction
# is discarded; odom -> base_link is unaffected either way.
MAX_CORRECTION_Z = 0.30
MAX_CORRECTION_M = 5.0

CAM_YAW_DEG_DEFAULT = 2.06


def base_from_optical(cam_x, cam_y, cam_z, cam_yaw_deg=0.0):
    """The B matrix above: optical (x right, y down, z fwd) -> base (x fwd, y left, z up).

    Columns of M are where each optical axis lands in base coordinates:
        optical +x (right)   -> base -y
        optical +y (down)    -> base -z
        optical +z (forward) -> base +x

    cam_yaw_deg corrects the camera not being square on its bracket. It rotates
    the camera's axes about base z; the translation is the physically measured
    mount position and is NOT rotated with it.
    """
    M = np.array([[0.0, 0.0, 1.0],
                  [-1.0, 0.0, 0.0],
                  [0.0, -1.0, 0.0]])
    a = math.radians(cam_yaw_deg)
    Rz = np.array([[math.cos(a), -math.sin(a), 0.0],
                   [math.sin(a), math.cos(a), 0.0],
                   [0.0, 0.0, 1.0]])
    B = np.eye(4)
    B[:3, :3] = Rz @ M
    B[:3, 3] = [cam_x, cam_y, cam_z]
    return B


def self_test():
    """Verify the frame conjugation without a camera, a rover or a container."""
    from scipy.spatial.transform import Rotation

    # Axis tests use zero mount yaw; the yaw correction is checked separately below.
    B = base_from_optical(CAM_X_DEFAULT, CAM_Y_DEFAULT, CAM_Z_DEFAULT, 0.0)
    Binv = np.linalg.inv(B)
    ok = True

    def check(name, got, want, tol=1e-9):
        nonlocal ok
        good = np.allclose(got, want, atol=tol)
        ok &= good
        print(f"  [{'ok' if good else 'FAIL'}] {name}\n        got  {np.round(got, 6)}\n        want {np.round(want, 6)}")

    # 1. At t0 the tracker returns identity, so the robot must be at the origin.
    check("t0 identity -> pose at origin", (B @ np.eye(4) @ Binv)[:3, 3], [0, 0, 0])

    # 2. Rig moves 1 m along optical +z (straight ahead) -> base +x.
    T = np.eye(4); T[:3, 3] = [0, 0, 1.0]
    check("1 m optical +z -> base +x", (B @ T @ Binv)[:3, 3], [1, 0, 0])

    # 3. Rig moves 1 m along optical +x (to its right) -> base -y.
    T = np.eye(4); T[:3, 3] = [1.0, 0, 0]
    check("1 m optical +x -> base -y", (B @ T @ Binv)[:3, 3], [0, -1, 0])

    # 4. A left turn. ROS yaw is +z (up); optical y is down, so a +30 deg ROS
    #    yaw is -30 deg about the optical y axis.
    th = math.radians(30.0)
    T = np.eye(4); T[:3, :3] = Rotation.from_euler('y', -th).as_matrix()
    got = Rotation.from_matrix((B @ T @ Binv)[:3, :3]).as_euler('xyz')
    check("30 deg left turn -> +30 deg yaw, no roll/pitch", got, [0, 0, th], tol=1e-6)

    # 5. Spinning in place swings a forward-mounted camera on a lever arm: the
    #    camera translates even though base_link does not. Rotating the RIG in
    #    place about the body axis must still leave base_link's origin put.
    #    (Camera orbits the body centre: rig translation of r*sin/1-cos.)
    r = CAM_X_DEFAULT
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('y', -th).as_matrix()
    T[:3, 3] = [-r * math.sin(th), 0.0, -r * (1 - math.cos(th))]
    check("pure body spin -> base_link origin does not move", (B @ T @ Binv)[:3, 3], [0, 0, 0], tol=1e-6)

    # 6. The mount-yaw correction, checked against the two real pushes that
    #    measured it. Uncorrected, a straight 2 m push reported ~7 cm sideways.
    a = math.radians(CAM_YAW_DEG_DEFAULT)
    Rz = np.array([[math.cos(a), -math.sin(a), 0.0],
                   [math.sin(a), math.cos(a), 0.0],
                   [0.0, 0.0, 1.0]])
    for label, measured in (("run 1", [1.972, -0.067, 0.0]),
                            ("run 2", [1.949, -0.074, 0.0])):
        corrected = Rz @ np.array(measured)
        good = abs(corrected[1]) < 0.01          # under 1 cm of residual sideways
        ok &= good
        print(f"  [{'ok' if good else 'FAIL'}] mount yaw fixes {label}: "
              f"y {measured[1] * 100:+.1f} cm -> {corrected[1] * 100:+.1f} cm "
              f"(x {corrected[0] * 100:.1f} cm)")

    print(f"\n  self-test: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument('--self-test', action='store_true')
    known, _ = ap.parse_known_args()
    if known.self_test:
        sys.exit(self_test())

    # Imported here so --self-test works outside the container.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    from cv_bridge import CvBridge
    from scipy.spatial.transform import Rotation
    import message_filters
    import cuvslam as vslam

    class VoNode(Node):
        def __init__(self):
            super().__init__('vo_node')
            self.declare_parameter('camera_ns', '/camera/camera0')
            self.declare_parameter('publish_tf', True)
            self.declare_parameter('slam', False)
            self.declare_parameter('cam_x', CAM_X_DEFAULT)
            self.declare_parameter('cam_y', CAM_Y_DEFAULT)
            self.declare_parameter('cam_z', CAM_Z_DEFAULT)
            self.declare_parameter('cam_yaw_deg', CAM_YAW_DEG_DEFAULT)

            g = lambda n: self.get_parameter(n).value
            self.ns = g('camera_ns')
            self.publish_tf = g('publish_tf')
            self.want_slam = g('slam')
            self.B = base_from_optical(g('cam_x'), g('cam_y'), g('cam_z'), g('cam_yaw_deg'))
            self.Binv = np.linalg.inv(self.B)

            self.bridge = CvBridge()
            self.tracker = None
            self.left_info = None
            self.right_info = None
            self.frames = 0
            self.tracked = 0
            self.last_report = time.time()
            self.rate = 0.0
            self.last_frames = 0
            self.landmarks = 0
            self.frozen = 0          # consecutive frames with a bit-identical pose
            self.last_pose = None

            self.pub_odom = self.create_publisher(Odometry, '/vo/odom', 10)
            self.pub_status = self.create_publisher(String, '/vo/status', 1)
            self.tf = TransformBroadcaster(self) if self.publish_tf else None
            # map -> odom is published whenever SLAM is on, independently of
            # publish_tf: that flag is about who owns odom -> base_link (the
            # fusion node does), and this is a different edge of the tree.
            self.tf_map = TransformBroadcaster(self) if self.want_slam else None
            self.correction_m = 0.0
            self.loop_closures = 0
            self.correction_rejected = 0
            self._last_lc_len = 0

            # camera_info is a low-rate metadata topic, safe to hold open.
            self.create_subscription(CameraInfo, f'{self.ns}/infra1/camera_info',
                                     self._left_info, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f'{self.ns}/infra2/camera_info',
                                     self._right_info, qos_profile_sensor_data)

            # enable_sync:=false means the driver does not cross-align streams,
            # so the IR pair arrives with near-identical but not bit-identical
            # stamps. cuVSLAM wants them inside 1 ms; 5 ms of slop lets the
            # synchroniser pair them and we reject anything worse below.
            left = message_filters.Subscriber(self, Image, f'{self.ns}/infra1/image_rect_raw',
                                              qos_profile=qos_profile_sensor_data)
            right = message_filters.Subscriber(self, Image, f'{self.ns}/infra2/image_rect_raw',
                                               qos_profile=qos_profile_sensor_data)
            self.sync = message_filters.ApproximateTimeSynchronizer([left, right], 10, 0.005)
            self.sync.registerCallback(self._stereo)

            self.create_timer(1.0, self._report)
            self.get_logger().info(f'vo_node up — waiting for camera_info on {self.ns}/infra{{1,2}}')

        def _left_info(self, m):
            self.left_info = m

        def _right_info(self, m):
            self.right_info = m

        def _build_tracker(self):
            """Build the stereo rig from camera_info. Called once, on first pair."""
            li, ri = self.left_info, self.right_info
            fx, fy = li.k[0], li.k[4]
            cx, cy = li.k[2], li.k[5]
            # REP-104 says a rectified right camera has P[3] = -fx * baseline, so
            # baseline = -P[3]/P[0]. This driver publishes infra2 with the OPPOSITE
            # sign: measured 2026-08-15, -P[3]/P[0] = -0.0949, and 94.9 mm is
            # exactly the D555's stereo baseline. So the magnitude is right and only
            # the convention differs. Take |.| and keep the sanity check on the size,
            # which is what actually catches a broken camera_info.
            signed = -ri.p[3] / ri.p[0]
            baseline = abs(signed)
            if not (0.01 < baseline < 1.0):
                self.get_logger().error(
                    f'implausible baseline {signed:+.4f} m from camera_info — '
                    'infra2/camera_info is wrong, not just sign-flipped')
                return False

            ident = vslam.Pose(rotation=[0.0, 0.0, 0.0, 1.0], translation=[0.0, 0.0, 0.0])
            # The rig frame IS the left optical frame (left camera at identity).
            # The right camera sits +baseline along optical x.
            right_pose = vslam.Pose(rotation=[0.0, 0.0, 0.0, 1.0],
                                    translation=[baseline, 0.0, 0.0])
            pinhole = vslam.Distortion(model=vslam.Distortion.Model.Pinhole, parameters=[])

            cams = []
            for pose in (ident, right_pose):
                cams.append(vslam.Camera(size=[li.width, li.height],
                                         principal=[cx, cy], focal=[fx, fy],
                                         rig_from_camera=pose, distortion=pinhole))
            rig = vslam.Rig(cameras=cams, imus=[])

            odom_cfg = vslam.Tracker.OdometryConfig()
            odom_cfg.odometry_mode = vslam.Tracker.OdometryMode.Multicamera
            odom_cfg.use_gpu = True
            # image_rect_raw is already rectified by the driver.
            odom_cfg.rectified_stereo_camera = True

            slam_cfg = None
            if self.want_slam:
                slam_cfg = vslam.Tracker.SlamConfig()
                # PLANAR CONSTRAINTS. This is a ground rover: it cannot leave the
                # floor, and telling the optimiser so removes a whole degree of
                # freedom it would otherwise get wrong. Measured 2026-08-22
                # WITHOUT this, cuVSLAM settled into a confident pose 21.7 m
                # underground while still reporting 95 healthy landmarks.
                slam_cfg.planar_constraints = True
            self.tracker = vslam.Tracker(rig, odom_config=odom_cfg, slam_config=slam_cfg)
            self.get_logger().info(
                f'tracker up — {li.width}x{li.height} fx={fx:.1f} baseline={baseline * 100:.2f} cm '
                f'slam={"on" if self.want_slam else "OFF (phase 1: measuring raw drift)"}')
            return True

        def _stereo(self, lmsg, rmsg):
            if self.tracker is None:
                if self.left_info is None or self.right_info is None:
                    return
                if not self._build_tracker():
                    return

            skew = abs((lmsg.header.stamp.sec - rmsg.header.stamp.sec) * 1e9 +
                       (lmsg.header.stamp.nanosec - rmsg.header.stamp.nanosec))
            if skew > 1e6:  # 1 ms — cuVSLAM's stated tolerance
                self.get_logger().warn(f'dropping pair, stereo skew {skew / 1e6:.2f} ms', throttle_duration_sec=5.0)
                return

            limg = self.bridge.imgmsg_to_cv2(lmsg, 'mono8')
            rimg = self.bridge.imgmsg_to_cv2(rmsg, 'mono8')
            ts = lmsg.header.stamp.sec * 10**9 + lmsg.header.stamp.nanosec

            try:
                est, slam_pose = self.tracker.track(ts, [limg, rimg])
            except ValueError as e:
                self.get_logger().warn(f'track rejected frame: {e}', throttle_duration_sec=5.0)
                return

            self.frames += 1
            if est is None or est.world_from_rig is None:
                return   # tracking lost this frame; report() will show the gap
            self.tracked += 1

            # world_from_rig is a PoseWithCovariance, NOT a Pose — the pose is one
            # level down. Its covariance is expressed in the optical rig frame, so
            # it would need conjugating by B too; nothing consumes it in Phase 1,
            # so we leave Odometry.pose.covariance zeroed rather than publish a
            # number in the wrong frame.
            wfr = est.world_from_rig.pose

            # How many landmarks the tracker is holding is the real health signal.
            # A pose being returned is not: a dead tracker returns identity forever
            # while the frame rate stays perfect.
            try:
                self.landmarks = len(self.tracker.get_last_landmarks())
            except Exception:
                self.landmarks = -1

            # cuVSLAM keeps the last 10 loop closures. Counting them turns "the
            # map looks better" into a number, and distinguishes a tracker that
            # is recognising places from one that simply has not revisited any.
            if self.want_slam:
                try:
                    lc = self.tracker.get_loop_closure_poses()
                    if lc is not None and len(lc) != self._last_lc_len:
                        if len(lc) > self._last_lc_len:
                            self.loop_closures += len(lc) - self._last_lc_len
                        self._last_lc_len = len(lc)
                except Exception:
                    pass
            cur = (tuple(wfr.translation), tuple(wfr.rotation))
            self.frozen = self.frozen + 1 if cur == self.last_pose else 0
            self.last_pose = cur

            R = np.eye(4)
            R[:3, :3] = Rotation.from_quat(list(wfr.rotation)).as_matrix()
            R[:3, 3] = list(wfr.translation)

            T = self.B @ R @ self.Binv          # <- the whole point of this node
            q = Rotation.from_matrix(T[:3, :3]).as_quat()   # xyzw

            od = Odometry()
            od.header.stamp = lmsg.header.stamp
            od.header.frame_id = 'odom'
            od.child_frame_id = 'base_link'
            od.pose.pose.position.x = float(T[0, 3])
            od.pose.pose.position.y = float(T[1, 3])
            od.pose.pose.position.z = float(T[2, 3])
            od.pose.pose.orientation.x = float(q[0])
            od.pose.pose.orientation.y = float(q[1])
            od.pose.pose.orientation.z = float(q[2])
            od.pose.pose.orientation.w = float(q[3])
            self.pub_odom.publish(od)

            if self.tf:
                t = TransformStamped()
                t.header = od.header
                t.child_frame_id = 'base_link'
                t.transform.translation.x = od.pose.pose.position.x
                t.transform.translation.y = od.pose.pose.position.y
                t.transform.translation.z = od.pose.pose.position.z
                t.transform.rotation = od.pose.pose.orientation
                self.tf.sendTransform(t)

            self._publish_correction(slam_pose, T, lmsg.header.stamp)

        def _publish_correction(self, slam_pose, T_odom, stamp):
            """map -> odom: everything loop closure has corrected, and nothing else.

            THIS IS WHY LOOP CLOSURE DOES NOT BREAK THE TELEPORT GUARD.

            cuVSLAM returns two poses per frame. The ODOMETRY one is continuous:
            it drifts but never jumps, which is exactly what an odom frame must be
            and what the fusion's teleport rejection depends on. The SLAM one is
            globally consistent but JUMPS whenever a loop closes.

            Publishing the SLAM pose as odom -> base_link would make every loop
            closure look identical to the divergence we spent a day learning to
            reject. So the two are kept apart, the way REP-105 intends:

                odom -> base_link   continuous, drifts     (fusion_node)
                map  -> odom        the correction, jumps  (here)
                map  -> base_link   globally consistent    (the two composed)

            nav2 and nvblox keep consuming odom; anything wanting a globally
            consistent frame reads map. Nothing has to choose between smooth and
            correct.

            map_from_odom = map_from_base * odom_from_base^-1, with both poses
            already conjugated into base_link.
            """
            if slam_pose is None or self.tf_map is None:
                return
            try:
                S = np.eye(4)
                S[:3, :3] = Rotation.from_quat(list(slam_pose.rotation)).as_matrix()
                S[:3, 3] = list(slam_pose.translation)
            except (AttributeError, TypeError, ValueError):
                return                      # not a pose this build returns

            T_map = self.B @ S @ self.Binv
            corr = T_map @ np.linalg.inv(T_odom)
            mag = float(np.linalg.norm(corr[:3, 3]))

            # THE SLAM POSE NEEDS THE SAME PLAUSIBILITY CHECK AS THE ODOMETRY ONE.
            # Measured 2026-08-22: after a room loop the correction came out as
            # [-10.5, -22.2, 87.7] m -- 87 metres straight up, a 91 m "correction"
            # for a rover that had driven about four. The SLAM optimiser had
            # diverged exactly as the odometry once did, and setting
            # planar_constraints did not prevent it.
            #
            # A correction is accumulated drift made visible. On a ground rover
            # over a room it is centimetres to a metre or two; it is never tens of
            # metres and it is never vertical. Publishing this would have dragged
            # the whole map frame with it.
            #
            # The odom frame was untouched throughout -- z exactly 0.000 -- which
            # is the separation earning its keep: the garbage stayed in the frame
            # nothing critical consumes.
            if abs(corr[2, 3]) > MAX_CORRECTION_Z or mag > MAX_CORRECTION_M:
                self.correction_rejected += 1
                self.get_logger().warn(
                    f'SLAM correction implausible ({mag:.1f} m, z={corr[2, 3]:+.1f} m) '
                    f'— not publishing map->odom. Odometry is unaffected.',
                    throttle_duration_sec=10.0)
                return

            # How far loop closure has moved the world. Worth reporting: it IS the
            # accumulated drift, measured rather than estimated.
            self.correction_m = mag

            q = Rotation.from_matrix(corr[:3, :3]).as_quat()
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = float(corr[0, 3])
            t.transform.translation.y = float(corr[1, 3])
            t.transform.translation.z = float(corr[2, 3])
            t.transform.rotation.x = float(q[0])
            t.transform.rotation.y = float(q[1])
            t.transform.rotation.z = float(q[2])
            t.transform.rotation.w = float(q[3])
            self.tf_map.sendTransform(t)

        def _report(self):
            now = time.time()
            dt = now - self.last_report
            self.rate = (self.frames - self.last_frames) / dt if dt > 0 else 0.0
            self.last_report, self.last_frames = now, self.frames
            self.pub_status.publish(String(data=json.dumps({
                'rate_hz': round(self.rate, 2),
                'frames': self.frames,
                'tracked': self.tracked,
                'tracker_up': self.tracker is not None,
                'landmarks': self.landmarks,
                'slam': self.want_slam,
                # How far loop closure has moved the world. This IS the
                # accumulated drift, measured rather than estimated.
                'correction_m': round(self.correction_m, 3),
                'loop_closures': self.loop_closures,
                'correction_rejected': self.correction_rejected,
                'frozen_frames': self.frozen,
                # Rate alone is NOT enough: a dead tracker returns identity at a
                # perfect 30 Hz. Landmarks are what say it is really tracking.
                'healthy': (self.tracker is not None and self.rate >= 10.0
                            and self.landmarks > 20),
            })))

    rclpy.init()
    node = VoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
