#!/usr/bin/env python3
"""pixel_to_goal — turn a VLM-picked color pixel into an odom-frame Nav2 goal.

This is the Jetson half of "go to the red bottle": the Pi 5 brain's VLM
(langrobo_core/tools/approach.py: approach_described_object) already looks
at the color frame and picks a pixel; this node turns that pixel into
somewhere Nav2 can actually drive.

CONTRACT (topics, matching what ROS2Bridge.ground_pixel() on the Pi 5
already sends/expects — this side did not exist before; that method has
been publishing into the void since it was written):

    in   /vision/pixel_query   geometry_msgs/PointStamped
         header.frame_id = an opaque request id, echoed back unchanged
         point.x, point.y = pixel (u, v) in the CURRENT color image's own
                            resolution — read /camera/camera0/color/camera_info
                            live for width/height, never assume a fixed size.
                            (rgb_camera.color_profile is silently not honoured
                            by this D555 — see image_bridge.py's docstring.)

    out  /vision/pixel_result  std_msgs/String, JSON:
         {"id": ..., "ok": true,
          "goal": {"x": <odom metres>, "y": <odom metres>, "yaw": <radians>},
          "depth_m": ...,
          "object": {"x": <odom metres>, "y": <odom metres>},
          "relative": {"forward_m": ..., "left_m": ..., "bearing_deg": ...}}

         "goal" is NOT where the object is: it is _STANDOFF_M (0.45 m) SHORT
         of it, on the robot->object line, so nav2 parks in front rather than
         inside it. Anything answering "where is the chair" must read
         "object"/"relative" — using "goal" understates every distance by the
         standoff and was the reason those two fields were added (2026-09-10).

         "object" is the object itself in odom. "relative" is the same point in
         base_link terms for a human answer: +forward_m ahead, +left_m to the
         robot's left, bearing_deg = atan2(left, forward), 0 straight ahead and
         positive counter-clockwise.

         depth_m and forward_m are NOT the same number and neither is wrong.
         depth_m is the raw sensor value: the Z-forward component in the camera
         optical frame (perpendicular distance to the image plane, which is what
         a RealSense depth pixel stores — not a straight-line range). forward_m
         is measured from base_link, which sits 0.17 m BEHIND the camera (the
         static TF in ./rover pose), so for an object near the optical axis
         forward_m ~= depth_m + 0.17. Verified live 2026-09-10: depth 1.19 ->
         forward 1.36. Report forward_m/left_m to a human ("how far from the
         robot"); depth_m is the sensor reading behind it.
         or
         {"id": ..., "ok": false, "reason": "..."}
         ground_pixel() has its own 4 s timeout and treats a silent query as
         "no_reply_from_jetson" — every OTHER failure here must still reply,
         with a reason, or the Pi 5 waits the full timeout for nothing.

FRAME: odom, not map. approach.py's docstring and ros2_bridge.py's
get_current_pose()/_nav_worker() were written assuming a `map` frame from a
different, fuller perception stack (Isaac ROS detections_3d, a pan/tilt
head) this rover does not have. This rover's Nav2 runs in `odom` only —
single session, no relocalisation, documented in rover's nav) case and
README/TODO. The goal published here is in odom to match what Nav2 here
actually understands.

The Pi 5 side is fixed and no longer needs watching in three places: goals,
TF pose reads and detection frames all now come from one constant,
ROS2Bridge.NAV_FRAME (default "odom", override with LANGROBO_NAV_FRAME).
Two of those three had been corrected by hand on 2026-09-06 and the third —
on_detections, which demanded "map" — was missed and silently dropped every
detection it was given. If this node's output frame ever changes, change
NAV_FRAME to match and nothing else.

DEPTH: aligned_depth_to_color, not the raw depth stream. Color and depth are
different sensors with different intrinsics/FOV (and, since the color
profile isn't honoured, potentially different resolutions too) — a color
pixel does not correspond 1:1 to the same depth pixel index. align_depth
(rover's camera) case, added 2026-09-06) makes the driver do that
reprojection itself; verified it does NOT starve cuVSLAM's IR pair the way
enabling color+sync once did (checked IR/VO rate immediately after enabling,
both stayed at 30 Hz).

NO tf2_geometry_msgs — not in the frozen image (checked; only tf2_ros is
available). The camera-frame -> odom point transform below is done by hand
with the same manual-quaternion style already used elsewhere in this repo
(e.g. fusion_node's yaw extraction) rather than pulling in a dependency the
container can't have added to it.

Run:  python3 pixel_to_goal.py
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

# Mirrors langrobo_core/tools/approach.py's _STANDOFF_M — different repos, on
# different machines, so it cannot be imported. It reads the SAME environment
# variable that side does, so exporting LANGROBO_STANDOFF_M once (here and on
# the Pi 5) keeps described-object goals and YOLO-class goals agreeing without
# editing two files in two repos and hoping.
_STANDOFF_M = float(os.environ.get('LANGROBO_STANDOFF_M', '0.45'))
_MAX_SENSOR_AGE_S = 1.5   # camera_info / depth older than this = stale, refuse

# aligned_depth_to_color is only ~55% valid overall (measured 2026-09-06,
# stereo-to-color reprojection punches real holes) and a VLM points at an
# object's CENTRE, which for anything with real geometry is often near an
# edge — exactly where reprojection holes cluster. A fixed small window
# (originally 2, i.e. 5x5) failed on the very first live test
# ("no_depth_at_pixel" on a wooden wedge whose edge sat in a hole) even
# though the surrounding region was 82% valid. Expand outward instead of
# giving up at one size: try each window, first one with a valid median
# wins. Any answer within this radius is still "the object", so precision
# isn't traded away, just robustness to one unlucky pixel.
_DEPTH_WINDOWS = (2, 5, 10, 18)   # half-widths tried in order: 5x5, 11x11, 21x21, 37x37
_MIN_DEPTH_M = 0.3        # D555 depth is blind closer than this
_MAX_DEPTH_M = 6.0        # beyond this the reading is usually noise


def compute_standoff_goal(rx, ry, ox, oy, standoff):
    """Nav2 goal (gx, gy, yaw_rad) that parks `standoff` metres short of the
    object (ox, oy) on the robot->object line, facing the object. Same
    geometry as approach.py's compute_standoff_goal (that one returns yaw in
    degrees; this one keeps radians since that's what the JSON contract
    above uses)."""
    dx, dy = ox - rx, oy - ry
    dist = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    if dist <= standoff or dist < 1e-6:
        return rx, ry, yaw
    scale = (dist - standoff) / dist
    return rx + dx * scale, ry + dy * scale, yaw


def _yaw_from_quat(q) -> float:
    """Yaw (radians) about Z from quaternion q. Same manual extraction as
    fusion_node's — no tf2_geometry_msgs in the frozen image, see the module
    docstring."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _rotate_by_quat(x: float, y: float, z: float, q) -> tuple:
    """Rotate vector (x,y,z) by quaternion q (x,y,z,w). v' = q * v * q^-1,
    expanded (Rodrigues form) to avoid a quaternion-multiply library."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    cx, cy, cz = qy * z - qz * y, qz * x - qx * z, qx * y - qy * x
    ccx = qy * cz - qz * cy
    ccy = qz * cx - qx * cz
    ccz = qx * cy - qy * cx
    return (x + 2 * qw * cx + 2 * ccx,
            y + 2 * qw * cy + 2 * ccy,
            z + 2 * qw * cz + 2 * ccz)


class PixelToGoal(Node):

    def __init__(self):
        super().__init__('pixel_to_goal')
        self._cv_bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._camera_info = None
        self._camera_info_at = 0.0
        self._depth = None
        self._depth_at = 0.0

        self.create_subscription(
            CameraInfo, '/camera/camera0/color/camera_info', self._on_camera_info, 5)
        self.create_subscription(
            Image, '/camera/camera0/aligned_depth_to_color/image_raw', self._on_depth, 1)
        self.create_subscription(
            PointStamped, '/vision/pixel_query', self._on_query, 5)
        self._result_pub = self.create_publisher(String, '/vision/pixel_result', 5)
        self.get_logger().info('pixel_to_goal up — waiting for /vision/pixel_query')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg
        self._camera_info_at = time.monotonic()

    def _on_depth(self, msg: Image) -> None:
        try:
            self._depth = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warning(f'depth imgmsg_to_cv2 failed: {e}', throttle_duration_sec=5.0)
            return
        self._depth_at = time.monotonic()

    def _reply(self, req_id: str, ok: bool, **fields) -> None:
        self._result_pub.publish(String(data=json.dumps({"id": req_id, "ok": ok, **fields})))

    def _sample_depth(self, depth: np.ndarray, ui: int, vi: int) -> tuple:
        """Median depth (mm) near (ui, vi), trying each window in
        _DEPTH_WINDOWS until one has valid data. Returns (depth_mm,
        window_half_width) or (None, last_half_width) if all fail."""
        h, w = depth.shape[:2]
        half = _DEPTH_WINDOWS[-1]
        for half in _DEPTH_WINDOWS:
            y0, y1 = max(0, vi - half), min(h, vi + half + 1)
            x0, x1 = max(0, ui - half), min(w, ui + half + 1)
            window = depth[y0:y1, x0:x1].astype(np.float64).flatten()
            valid = window[window > 0]
            if valid.size > 0:
                return float(np.median(valid)), half
        return None, half

    def _on_query(self, msg: PointStamped) -> None:
        req_id = msg.header.frame_id
        u, v = msg.point.x, msg.point.y
        now = time.monotonic()

        info = self._camera_info
        if info is None or now - self._camera_info_at > _MAX_SENSOR_AGE_S:
            self.get_logger().warning(f'query {req_id} ({u:.0f},{v:.0f}): no_camera_info')
            self._reply(req_id, False, reason="no_camera_info")
            return
        depth = self._depth
        if depth is None or now - self._depth_at > _MAX_SENSOR_AGE_S:
            self.get_logger().warning(f'query {req_id} ({u:.0f},{v:.0f}): no_depth_frame')
            self._reply(req_id, False, reason="no_depth_frame")
            return

        ui, vi = int(round(u)), int(round(v))
        h, w = depth.shape[:2]
        if not (0 <= ui < w and 0 <= vi < h):
            self.get_logger().warning(
                f'query {req_id} ({u:.0f},{v:.0f}): pixel_out_of_bounds for {w}x{h} frame')
            self._reply(req_id, False, reason="pixel_out_of_bounds")
            return

        depth_mm, half = self._sample_depth(depth, ui, vi)
        if depth_mm is None:
            self.get_logger().warning(
                f'query {req_id} ({ui},{vi}): no valid depth within {2*half+1}x{2*half+1} window')
            self._reply(req_id, False, reason="no_depth_at_pixel")
            return
        depth_m = depth_mm / 1000.0   # D400-series depth is mm, uint16
        if not (_MIN_DEPTH_M <= depth_m <= _MAX_DEPTH_M):
            self.get_logger().warning(
                f'query {req_id} ({ui},{vi}): depth {depth_m:.2f}m out of range')
            self._reply(req_id, False, reason="depth_out_of_range")
            return
        self.get_logger().info(
            f'query {req_id} ({ui},{vi}): depth {depth_m:.2f}m (window ±{half}px)')

        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]
        x_cam = (u - cx) * depth_m / fx
        y_cam = (v - cy) * depth_m / fy
        z_cam = depth_m   # optical-frame convention: Z forward, X right, Y down

        # NO TIMEOUT. This runs in _on_query, i.e. inside the single-threaded
        # executor's callback — the very thread the TransformListener needs in
        # order to consume /tf and fill the buffer. Blocking here cannot
        # succeed: the buffer can only be filled by the thread we are blocking,
        # so a 0.5s timeout was a guaranteed 0.5s stall followed by the same
        # failure. Latest-available (Time()) is right anyway — cuVSLAM
        # publishes odom->base_link at 20 Hz and the depth frame this pixel
        # came from is already gated at _MAX_SENSOR_AGE_S.
        try:
            cam_to_odom = self._tf_buffer.lookup_transform(
                'odom', info.header.frame_id, Time())
        except Exception as e:
            self._reply(req_id, False, reason=f"tf_failed:{type(e).__name__}")
            return

        t = cam_to_odom.transform.translation
        rot_x, rot_y, rot_z = _rotate_by_quat(
            x_cam, y_cam, z_cam, cam_to_odom.transform.rotation)
        ox, oy = t.x + rot_x, t.y + rot_y

        try:
            base_tf = self._tf_buffer.lookup_transform('odom', 'base_link', Time())
        except Exception:
            self._reply(req_id, False, reason="no_robot_pose")
            return
        rx = base_tf.transform.translation.x
        ry = base_tf.transform.translation.y

        gx, gy, yaw = compute_standoff_goal(rx, ry, ox, oy, _STANDOFF_M)

        # Robot-relative view of the SAME object point, for tools that answer
        # "how far / which way is it" instead of driving there. Done here rather
        # than on the Pi 5 because the Pi 5 would have to reconstruct it from
        # `goal`, which is standoff-shortened — it would be wrong by 0.45 m and
        # wrong in a way that looks plausible.
        r_yaw = _yaw_from_quat(base_tf.transform.rotation)
        dx, dy = ox - rx, oy - ry
        forward = dx * math.cos(r_yaw) + dy * math.sin(r_yaw)
        left = -dx * math.sin(r_yaw) + dy * math.cos(r_yaw)

        self._reply(req_id, True,
                   goal={"x": round(gx, 3), "y": round(gy, 3), "yaw": round(yaw, 4)},
                   depth_m=round(depth_m, 2),
                   object={"x": round(ox, 3), "y": round(oy, 3)},
                   relative={"forward_m": round(forward, 2),
                             "left_m": round(left, 2),
                             "bearing_deg": round(math.degrees(math.atan2(left, forward)), 1)})


def main():
    rclpy.init()
    node = PixelToGoal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
