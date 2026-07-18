"""3D object detections for the brain — YOLO + D555 depth → map-frame JSON.

The Pi5 brain's approach_object tool ("come here", "go near the chair") needs
metric, map-frame object positions. This node runs YOLOv8n on the D555 color
stream, samples the (unaligned) depth image at each detection via
intrinsics-angle mapping, deprojects to a 3D point in the depth optical frame,
transforms it to `map` with TF (cuVSLAM), and publishes the Pi5 contract:

  /vision/detections_3d   std_msgs/String   JSON:
    {"frame": "map", "stamp": 1752300000.0,
     "objects": [{"label": "chair", "x": 2.31, "y": -0.42, "z": 0.35,
                  "conf": 0.81}, ...]}

Contract rules (ros2_bridge.on_detections on the Pi5):
  - frame MUST be "map": we publish NOTHING until TF map→optical resolves.
  - labels are lowercase COCO class names.
  - freshness is judged by receive time on the Pi5 — publish steadily.

YOLO runs ON DEMAND (2026-07-18): it is the single biggest CPU cost on the
6-core Orin, so it only runs at full detect_rate when something needs it (a
target hunt is set, or a subscriber is on detections_3d/target_result);
otherwise it trickles at idle_detect_rate. The look() feed is independent and
always on. Tune with the detect_rate / idle_detect_rate params.

It ALSO republishes the color stream as JPEG on
/camera/color/image_raw/compressed (low rate): with the ai_stack voice
container parked (compute freed for perception), this is the only camera feed
the brain's look() tool and Telegram watch alerts have.

And it IS the brain's YOLO target finder (visual servoing,
navigate_to_visible_object). ARCHITECTURE.md contract:

  /vision/target          String   Pi5 → here   COCO class to hunt ("" = stop)
  /vision/target_result   String   here → Pi5   JSON, published EVERY detect
      tick while a target is set (found or not — the brain treats silence as
      a dead camera and halts):
    {"target": "chair", "found": true, "bearing_x": 0.31, "rel_size": 0.18,
     "conf": 0.72, "stamp": 1752300000.0}
      bearing_x: horizontal offset of the box centre, -1 (left) .. +1 (right)
      rel_size:  max(box_w/W, box_h/H) — the brain stops approaching at 0.45

The target hunt deliberately does NOT require TF/SLAM — it is the no-map
fallback path — so it runs even while cuVSLAM is still starting.

Design notes:
  - No cv_bridge / tf2_geometry_msgs imports: images decode via numpy, the
    point transform is a quaternion rotation done by hand.
  - Depth is NOT the aligned_depth stream: a color pixel is mapped into the
    depth image through both cameras' intrinsics (angle equality). The few-mm
    color↔depth baseline is negligible at approach ranges (>0.3 m).
  - Runs in the isaac_ros container (torch/ultralytics installed 2026-07-13,
    image isaac_ros:langrobo-nav-stack-1.2).
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

import torch
torch.backends.cudnn.enabled = False  # cuDNN version mismatch on Jetson (same as target_node)
from ultralytics import YOLO

from tf2_ros import Buffer, TransformListener


def _decode_image(msg: Image) -> np.ndarray | None:
    """sensor_msgs/Image → numpy array without cv_bridge."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return img[:, :, ::-1] if msg.encoding == "rgb8" else img  # YOLO wants BGR
    if msg.encoding == "16UC1":
        return buf.view(np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
    if msg.encoding == "32FC1":
        return buf.view(np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
    if msg.encoding == "mono8":
        return buf.reshape(msg.height, msg.step)[:, :msg.width]
    return None


def _quat_rotate(qx, qy, qz, qw, v):
    """Rotate vector v by quaternion q (numpy, no tf2_geometry_msgs)."""
    q = np.array([qx, qy, qz])
    t = 2.0 * np.cross(q, v)
    return v + qw * t + np.cross(q, t)


class Detections3DNode(Node):
    def __init__(self):
        super().__init__("detections_3d")

        self.declare_parameter("model", "/models/yolov8n.pt")
        self.declare_parameter("confidence", 0.45)
        self.declare_parameter("device", "cuda")
        self.declare_parameter("detect_rate", 5.0)   # Hz — rate WHEN IN DEMAND
        # On-demand gating: YOLO is the biggest CPU draw on the 6-core Orin
        # (~half a core at 5Hz), but it is only useful when the brain is
        # actually looking for something. "In demand" = a target hunt is set
        # (/vision/target) OR someone subscribes to a detection output. When
        # idle we drop YOLO to idle_detect_rate so /vision/detections_3d stays
        # fresh for the Pi5 without burning the core; set 0.0 to stop YOLO
        # entirely while idle (max saving — safe once the brain is confirmed to
        # subscribe on demand). The look() feed is separate and always on.
        self.declare_parameter("idle_detect_rate", 1.0)  # Hz — rate WHEN IDLE
        self.declare_parameter("max_range_m", 6.0)
        self.declare_parameter("min_range_m", 0.25)
        self.declare_parameter("depth_patch_px", 7)
        self.declare_parameter("color_topic", "/camera/camera0/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera0/depth/image_rect_raw")
        self.declare_parameter("color_info_topic", "/camera/camera0/color/camera_info")
        self.declare_parameter("depth_info_topic", "/camera/camera0/depth/camera_info")
        self.declare_parameter("output_topic", "/vision/detections_3d")
        self.declare_parameter("target_frame", "map")
        # look() feed for the brain (0 = disabled). JPEG on
        # /camera/color/image_raw/compressed — the voice container's
        # camera_node used to provide this; with ai_stack parked we do.
        self.declare_parameter("look_feed_rate", 2.0)

        p = lambda n: self.get_parameter(n).value
        self._conf_min = float(p("confidence"))
        self._max_range = float(p("max_range_m"))
        self._min_range = float(p("min_range_m"))
        self._patch = int(p("depth_patch_px")) // 2
        self._target_frame = p("target_frame")

        self.get_logger().info(f"Loading YOLO {p('model')} on {p('device')}...")
        self._model = YOLO(p("model"))
        self._device = p("device")

        self._color = self._depth = None
        self._color_info = self._depth_info = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._no_tf_logged = False

        self.create_subscription(Image, p("color_topic"), self._on_color,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, p("depth_topic"), self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p("color_info_topic"), self._on_color_info,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p("depth_info_topic"), self._on_depth_info,
                                 qos_profile_sensor_data)
        self._pub = self.create_publisher(String, p("output_topic"), 10)

        # Target finder (brain visual servoing) — see module docstring.
        self._target = ""
        self.create_subscription(String, "/vision/target", self._on_target, 10)
        self._target_pub = self.create_publisher(String, "/vision/target_result", 10)

        look_rate = float(p("look_feed_rate"))
        if look_rate > 0:
            self._look_pub = self.create_publisher(
                CompressedImage, "/camera/color/image_raw/compressed", 1)
            self.create_timer(1.0 / look_rate, self._publish_look_feed)

        # Timer ticks at the ACTIVE rate; when idle, _tick self-throttles YOLO
        # to idle_detect_rate (or skips it entirely if that is 0).
        idle_rate = float(p("idle_detect_rate"))
        self._idle_period = (1.0 / idle_rate) if idle_rate > 0 else None
        self._last_infer_mono = 0.0
        self._in_demand = None  # tri-state so the first transition logs
        self.create_timer(1.0 / float(p("detect_rate")), self._tick)
        self.get_logger().info("detections_3d ready — waiting for camera + TF")

    # ── Subscriptions (cache latest) ──────────────────────────────────────

    def _on_color(self, msg): self._color = msg
    def _on_depth(self, msg): self._depth = msg
    def _on_color_info(self, msg): self._color_info = msg
    def _on_depth_info(self, msg): self._depth_info = msg

    def _on_target(self, msg):
        label = msg.data.lower().strip()
        if label != self._target:
            self.get_logger().info(f"target hunt: {label!r}" if label
                                   else "target hunt: idle")
        self._target = label

    # ── look() feed (brain camera snapshot + watch alerts) ────────────────

    def _publish_look_feed(self):
        if self._color is None:
            return
        img = _decode_image(self._color)
        if img is None:
            return
        if img.ndim == 2:
            img = np.stack([img] * 3, axis=-1)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        out = CompressedImage()
        out.header = self._color.header
        out.format = "jpeg"
        out.data = jpg.tobytes()
        self._look_pub.publish(out)

    # ── Detection tick ────────────────────────────────────────────────────

    def _publish_target_result(self, boxes, width, height):
        """Visual-servoing answer for the label on /vision/target — published
        every tick while hunting (found or not), no TF/SLAM required."""
        best = None
        for label, conf, xyxy in boxes:
            if label == self._target and (best is None or conf > best[1]):
                best = (label, conf, xyxy)
        if best is None:
            out = {"target": self._target, "found": False, "stamp": time.time()}
        else:
            _, conf, (u1, v1, u2, v2) = best
            out = {
                "target": self._target, "found": True,
                "bearing_x": round(((u1 + u2) / 2 - width / 2) / (width / 2), 3),
                "rel_size": round(max((u2 - u1) / width, (v2 - v1) / height), 3),
                "conf": round(conf, 2), "stamp": time.time(),
            }
        self._target_pub.publish(String(data=json.dumps(out)))

    def _demand(self) -> bool:
        """True when YOLO output is actually wanted: an active target hunt, or
        a live subscriber on either detection output. Cheap to poll."""
        return (bool(self._target)
                or self._pub.get_subscription_count() > 0
                or self._target_pub.get_subscription_count() > 0)

    def _tick(self):
        if self._color is None or self._color_info is None:
            return

        # On-demand gate: only spend YOLO when something needs it. Idle ->
        # trickle at idle_detect_rate (or skip if disabled). Log transitions.
        demand = self._demand()
        if demand != self._in_demand:
            self.get_logger().info(
                "YOLO active (demand)" if demand else "YOLO idle (no demand)")
            self._in_demand = demand
        if not demand:
            if self._idle_period is None:
                return
            if time.monotonic() - self._last_infer_mono < self._idle_period:
                return
        self._last_infer_mono = time.monotonic()

        color_msg, depth_msg = self._color, self._depth

        img = _decode_image(color_msg)
        if img is None:
            return
        if img.ndim == 2:  # mono → 3ch for YOLO
            img = np.stack([img] * 3, axis=-1)

        # YOLO once per tick; everything downstream shares the boxes.
        boxes = []
        for r in self._model(img, device=self._device, verbose=False):
            for b in r.boxes:
                conf = float(b.conf[0])
                if conf < self._conf_min:
                    continue
                boxes.append((self._model.names[int(b.cls[0])].lower(), conf,
                              tuple(map(int, b.xyxy[0]))))

        # Path 1 — target finder (no TF needed; the no-map fallback).
        if self._target:
            self._publish_target_result(boxes, img.shape[1], img.shape[0])

        # Path 2 — map-frame detections. Contract: map frame or silence; the
        # lookup also fails while cuVSLAM is still starting — that silence is
        # correct.
        if depth_msg is None or self._depth_info is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target_frame, depth_msg.header.frame_id,
                rclpy.time.Time())
        except Exception:
            if not self._no_tf_logged:
                self.get_logger().warning(
                    f"TF {self._target_frame} → {depth_msg.header.frame_id} "
                    f"not available yet — publishing nothing (correct while "
                    f"vSLAM starts)")
                self._no_tf_logged = True
            return
        self._no_tf_logged = False

        dep = _decode_image(depth_msg)
        if dep is None:
            return

        ci, di = self._color_info, self._depth_info
        fx_c, fy_c, cx_c, cy_c = ci.k[0], ci.k[4], ci.k[2], ci.k[5]
        fx_d, fy_d, cx_d, cy_d = di.k[0], di.k[4], di.k[2], di.k[5]

        t = tf.transform.translation
        q = tf.transform.rotation

        objects = []
        for label, conf, (u1, v1, u2, v2) in boxes:
            u_c, v_c = (u1 + u2) // 2, (v1 + v2) // 2
            # Map the color pixel into the depth image via angle equality
            # (intrinsics-normalised coordinates; baseline ≈ 0).
            u_d = int((u_c - cx_c) / fx_c * fx_d + cx_d)
            v_d = int((v_c - cy_c) / fy_c * fy_d + cy_d)
            if not (0 <= u_d < dep.shape[1] and 0 <= v_d < dep.shape[0]):
                continue
            h = self._patch
            patch = dep[max(0, v_d - h):v_d + h + 1,
                        max(0, u_d - h):u_d + h + 1].astype(np.float32)
            patch = patch[patch > 0]
            if patch.size < 3:
                continue
            z = float(np.median(patch))
            if dep.dtype == np.uint16:
                z /= 1000.0                      # mm → m
            if not (self._min_range < z < self._max_range):
                continue
            # Deproject in the depth optical frame, then → map.
            pt = np.array([(u_d - cx_d) / fx_d * z,
                           (v_d - cy_d) / fy_d * z,
                           z])
            mp = _quat_rotate(q.x, q.y, q.z, q.w, pt) + np.array([t.x, t.y, t.z])
            objects.append({
                "label": label,
                "x": round(float(mp[0]), 3),
                "y": round(float(mp[1]), 3),
                "z": round(float(mp[2]), 3),
                "conf": round(conf, 2),
            })

        if objects:
            self._pub.publish(String(data=json.dumps(
                {"frame": self._target_frame, "stamp": time.time(),
                 "objects": objects})))


def main(args=None):
    rclpy.init(args=args)
    node = Detections3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
