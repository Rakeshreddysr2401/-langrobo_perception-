"""VLM pixel grounding — turn "that pixel is the chair" into a Nav2-ready map goal.

The Pi5 brain's VLM flow for "go near the chair":
  1. Pi5 grabs a frame from /camera/color/image_raw/compressed (the look() feed
     published by detections_3d at 2 Hz).
  2. The VLM finds the object in the image and returns its PIXEL (u, v).
  3. Pi5 publishes that pixel here; this node samples depth, deprojects to 3D,
     transforms to `map`, pulls the point back toward the robot by
     `approach_offset_m` (you want to stop NEAR the chair, not inside it),
     faces the goal at the object, and answers with a PoseStamped.
  4. Pi5 forwards the pose as a NavigateToPose goal.

Contract (all frames map, all units metres):
  in:   /vision/pixel_query   geometry_msgs/PointStamped
          point.x = u pixel column in the COLOR image
          point.y = v pixel row
          header.frame_id is echoed back verbatim (use it as a request id)
  out:  /vision/pixel_goal    geometry_msgs/PoseStamped (frame_id "map")
          header.frame_id of the REQUEST echoed in... pose is in map; the
          request id travels in the pose's header.frame_id suffix — see below.

  On failure (no depth at pixel, TF not ready) nothing is published — the Pi5
  side should time out (~2 s) and retry or re-prompt the VLM.

Depth sampling matches detections_3d_node: the color pixel is mapped into the
unaligned depth image via intrinsics angle-equality (color↔depth baseline of a
few mm is negligible beyond 0.3 m), then a median over a small patch.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def _decode_depth(msg: Image) -> np.ndarray | None:
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding == "16UC1":
        return buf.view(np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
    if msg.encoding == "32FC1":
        return buf.view(np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
    return None


def _quat_rotate(qx, qy, qz, qw, v):
    q = np.array([qx, qy, qz])
    t = 2.0 * np.cross(q, v)
    return v + qw * t + np.cross(q, t)


class PixelToGoalNode(Node):
    def __init__(self):
        super().__init__("pixel_to_goal")

        self.declare_parameter("color_info_topic", "/camera/camera0/color/camera_info")
        self.declare_parameter("depth_topic", "/camera/camera0/depth/image_rect_raw")
        self.declare_parameter("depth_info_topic", "/camera/camera0/depth/camera_info")
        self.declare_parameter("query_topic", "/vision/pixel_query")
        self.declare_parameter("goal_topic", "/vision/pixel_goal")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("approach_offset_m", 0.6)
        self.declare_parameter("min_range_m", 0.25)
        self.declare_parameter("max_range_m", 8.0)
        self.declare_parameter("depth_patch_px", 9)

        p = lambda n: self.get_parameter(n).value
        self._target_frame = p("target_frame")
        self._base_frame = p("base_frame")
        self._offset = float(p("approach_offset_m"))
        self._min_range = float(p("min_range_m"))
        self._max_range = float(p("max_range_m"))
        self._patch = int(p("depth_patch_px")) // 2

        self._depth = None
        self._color_info = None
        self._depth_info = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(Image, p("depth_topic"), self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p("color_info_topic"),
                                 self._on_color_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p("depth_info_topic"),
                                 self._on_depth_info, qos_profile_sensor_data)
        self.create_subscription(PointStamped, p("query_topic"), self._on_query, 10)
        self._pub = self.create_publisher(PoseStamped, p("goal_topic"), 10)

        self.get_logger().info("pixel_to_goal ready — send PointStamped pixel "
                               f"queries on {p('query_topic')}")

    def _on_depth(self, msg): self._depth = msg
    def _on_color_info(self, msg): self._color_info = msg
    def _on_depth_info(self, msg): self._depth_info = msg

    def _on_query(self, msg: PointStamped):
        if None in (self._depth, self._color_info, self._depth_info):
            self.get_logger().warning("query dropped: camera streams not ready")
            return
        depth_msg = self._depth

        try:
            tf_cam = self._tf_buffer.lookup_transform(
                self._target_frame, depth_msg.header.frame_id, rclpy.time.Time())
            tf_base = self._tf_buffer.lookup_transform(
                self._target_frame, self._base_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warning(f"query dropped: TF not ready ({e})")
            return

        dep = _decode_depth(depth_msg)
        if dep is None:
            self.get_logger().warning("query dropped: undecodable depth encoding")
            return

        ci, di = self._color_info, self._depth_info
        fx_c, fy_c, cx_c, cy_c = ci.k[0], ci.k[4], ci.k[2], ci.k[5]
        fx_d, fy_d, cx_d, cy_d = di.k[0], di.k[4], di.k[2], di.k[5]

        u_c, v_c = float(msg.point.x), float(msg.point.y)
        u_d = int((u_c - cx_c) / fx_c * fx_d + cx_d)
        v_d = int((v_c - cy_c) / fy_c * fy_d + cy_d)
        if not (0 <= u_d < dep.shape[1] and 0 <= v_d < dep.shape[0]):
            self.get_logger().warning(f"query dropped: pixel ({u_c:.0f},{v_c:.0f}) "
                                      "maps outside the depth image")
            return

        h = self._patch
        patch = dep[max(0, v_d - h):v_d + h + 1,
                    max(0, u_d - h):u_d + h + 1].astype(np.float32)
        patch = patch[patch > 0]
        if patch.size < 3:
            self.get_logger().warning("query dropped: no valid depth at pixel "
                                      "(object too close / reflective / out of range)")
            return
        z = float(np.median(patch))
        if dep.dtype == np.uint16:
            z /= 1000.0
        if not (self._min_range < z < self._max_range):
            self.get_logger().warning(f"query dropped: depth {z:.2f}m outside "
                                      f"[{self._min_range}, {self._max_range}]")
            return

        # Object point: deproject in depth optical frame → map.
        pt = np.array([(u_d - cx_d) / fx_d * z,
                       (v_d - cy_d) / fy_d * z,
                       z])
        t, q = tf_cam.transform.translation, tf_cam.transform.rotation
        obj = _quat_rotate(q.x, q.y, q.z, q.w, pt) + np.array([t.x, t.y, t.z])

        # Goal: on the floor, pulled back from the object toward the robot so
        # the rover stops NEAR it, facing it.
        rb = tf_base.transform.translation
        robot = np.array([rb.x, rb.y])
        to_obj = obj[:2] - robot
        dist = float(np.linalg.norm(to_obj))
        if dist < 1e-3:
            self.get_logger().warning("query dropped: object at robot position")
            return
        heading = to_obj / dist
        goal_xy = obj[:2] - heading * min(self._offset, dist * 0.5)
        yaw = float(np.arctan2(heading[1], heading[0]))

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._target_frame
        out.pose.position.x = float(goal_xy[0])
        out.pose.position.y = float(goal_xy[1])
        out.pose.orientation.z = float(np.sin(yaw / 2.0))
        out.pose.orientation.w = float(np.cos(yaw / 2.0))
        self._pub.publish(out)
        self.get_logger().info(
            f"pixel ({u_c:.0f},{v_c:.0f}) depth {z:.2f}m → object map "
            f"({obj[0]:.2f},{obj[1]:.2f},{obj[2]:.2f}) → goal "
            f"({goal_xy[0]:.2f},{goal_xy[1]:.2f}) yaw {np.degrees(yaw):.0f}°"
            + (f" [req {msg.header.frame_id}]" if msg.header.frame_id else ""))


def main(args=None):
    rclpy.init(args=args)
    node = PixelToGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
