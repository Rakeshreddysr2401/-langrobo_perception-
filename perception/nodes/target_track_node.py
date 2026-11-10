#!/usr/bin/env python3
"""target_track_node.py — keep seeing a locked thing, every frame (CLOSED_LOOP_PLAN.md stage 2).

    in   /target/lock    std_msgs/String JSON  -- find it once, then follow it:
           {"id": "r7", "stamp": <the photo's camera stamp, s>, "box": [x0, y0, x1, y1],
            "object": {"x", "y", "z"}, "camera": {"t": [..], "q": [..]}, "label": "dot"}
           i.e. the photo's stamp, the box in it, and pixel_to_goal's reply
           ("object" + "camera") for that photo, passed through. "stamp" 0 or
           absent: the newest frame, with the camera pose looked up now.
         /target/cancel  std_msgs/String (an id, or "" for any)
    out  /target/state   std_msgs/String JSON, every tracked frame (~15 Hz) and on
           every change: {"id", "stamp", "state", "object": [x, y, z], "sigma_m",
           "score", "pixel", "depth", "unseen_s", "sightings", "from_lock_m",
           "rejected", "floor", "label"} -- target_lock.Target.status(); plus
           {"state": "locked" | "refused" (+ "reason") | "idle"} on a lock / cancel.

This node only SEES. It never drives and never sends goals: what to do with a
refined point (re-send /reach's goal, finish on it, ask the VLM after "lost") is
the caller's -- target_lock.goal_update_due() is the rule for re-sending, since
every new /reach goal restarts its attempts.

COST: asleep (nothing locked) it holds only the JPEG ring -- image_bridge's
/camera/color/image_raw/compressed at 5 Hz, bytes kept, never decoded, ~2 s
-- and camera_info. Colour, aligned depth and TF are subscribed only while a
lock is held (the detections_3d lesson: an idle subscriber to images and /tf
cost 29 % CPU). Awake: one decode + ~1 ms of matching per frame.

WHY THE PHOTO AND ITS POSE COME WITH THE LOCK. The VLM answers 5-40 s after the
photo; image_bridge's frames are kept here for LOCK_RING_S only and the TF
buffer for 10 s. So the Pi 5 announces the photo on /vision/pixel_snapshot
(already, for pixel_to_goal), this node then keeps THAT JPEG, and the camera
pose comes back in pixel_to_goal's reply -- one owner of the photo's pose.
"""
import json
import os
import threading
import time
from collections import OrderedDict, deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from target_lock import Target

LOCK_RING_S = 2.5        # image_bridge JPEGs kept (5 Hz -> ~13 frames, ~30 KB each)
SNAPSHOTS = 24           # announced photos kept (= pixel_to_goal's SNAPSHOTS)
SNAP_MAX_DT_S = 0.10     # the kept JPEG must be this close to the announced stamp
DEPTH_MAX_DT_S = 0.02    # aligned depth pairs with the colour frame of the same stamp (they match exactly)
DEPTH_RING = 6           # depth frames kept for pairing (~0.4 s at 15 Hz)
PAIR_WAIT_S = 0.15       # a colour frame waits this long for its depth, then runs without it
TF_RETRY_S = 0.15        # a frame whose TF is not in yet is skipped after this


def _ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _decode(msg):
    """sensor_msgs/Image -> BGR uint8, or depth in METRES float32 (no cv_bridge)."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return img[:, :, ::-1].copy() if msg.encoding == "rgb8" else img
    if msg.encoding == "16UC1":     # D400 depth: millimetres
        return buf.view(np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width].astype(np.float32) / 1000.0
    if msg.encoding == "32FC1":
        return buf.view(np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
    return None


class TargetTrackNode(Node):
    def __init__(self):
        super().__init__("target_track")
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self._color_topic = p("color_topic", "/camera/camera0/color/image_raw")
        self._depth_topic = p("depth_topic", "/camera/camera0/aligned_depth_to_color/image_raw")
        self._max_hz = float(p("max_hz", 15.0))
        self._frame = p("odom_frame", "odom")

        self._jpegs = deque()                 # (stamp_ns, bytes)
        self._snaps = OrderedDict()           # stamp_ns -> bytes (announced photos)
        self._info = None                     # latest colour CameraInfo
        self._target = None                   # target_lock.Target while locked
        self._lock_id = ""
        self._depths = deque(maxlen=DEPTH_RING)   # recent aligned depth frames
        self._pending = None                       # (colour msg, arrival) waiting for its depth
        self._last_t = 0.0
        self._paired = self._unpaired = 0
        self._last_state = None
        self._subs = []
        self._tf_listener = None
        # TF on its own node and thread (detections_3d's lesson: one node, one executor)
        self._tf_buffer = Buffer()
        self._tf_node = rclpy.create_node("target_track_tf")
        self._tf_exec = SingleThreadedExecutor()
        self._tf_exec.add_node(self._tf_node)
        threading.Thread(target=self._tf_exec.spin, daemon=True).start()

        self.create_subscription(CompressedImage, "/camera/color/image_raw/compressed", self._on_jpeg, 5)
        self.create_subscription(CameraInfo, "/camera/camera0/color/camera_info", self._on_info, qos_profile_sensor_data)
        self.create_subscription(PointStamped, "/vision/pixel_snapshot", self._on_snapshot, 5)
        self.create_subscription(String, "/target/lock", self._on_lock, 5)
        self.create_subscription(String, "/target/cancel", self._on_cancel, 5)
        self._pub = self.create_publisher(String, "/target/state", 10)
        self.get_logger().info("target_track up: asleep until /target/lock")

    # ── always on (cheap) ────────────────────────────────────────────────────
    def _on_jpeg(self, msg):
        ns = _ns(msg.header.stamp)
        self._jpegs.append((ns, bytes(msg.data)))
        while self._jpegs and ns - self._jpegs[0][0] > LOCK_RING_S * 1e9:
            self._jpegs.popleft()

    def _on_info(self, msg):
        self._info = msg

    def _on_snapshot(self, msg):
        ns = _ns(msg.header.stamp)
        jpg = self._jpeg_near(ns)
        if jpg is None:
            self.get_logger().warning(f"photo {ns / 1e9:.3f}: no JPEG within {SNAP_MAX_DT_S * 1000:.0f} ms to keep")
            return
        self._snaps[ns] = jpg
        while len(self._snaps) > SNAPSHOTS:
            self._snaps.popitem(last=False)

    def _jpeg_near(self, ns):
        if not self._jpegs:
            return None
        s, data = min(self._jpegs, key=lambda f: abs(f[0] - ns))
        return data if abs(s - ns) <= SNAP_MAX_DT_S * 1e9 else None

    # ── lock / cancel ────────────────────────────────────────────────────────
    def _say(self, **fields):
        self._pub.publish(String(data=json.dumps({"id": self._lock_id, **fields})))

    def _refuse(self, req_id, reason):
        self.get_logger().warning(f"lock {req_id}: refused ({reason})")
        self._pub.publish(String(data=json.dumps({"id": req_id, "state": "refused", "reason": reason})))

    def _on_lock(self, msg):
        try:
            req = json.loads(msg.data)
            req_id = str(req.get("id", ""))
            box = [float(v) for v in req["box"]]
            obj = req["object"]
            point = np.array([float(obj["x"]), float(obj["y"]), float(obj["z"])])
        except (ValueError, KeyError, TypeError) as e:
            self._refuse(str(locals().get("req_id", "")), f"bad_request:{type(e).__name__}:{e}")
            return
        if self._info is None:
            return self._refuse(req_id, "no_camera_info")
        K = np.array(self._info.k, float).reshape(3, 3)
        stamp_ns = int(float(req.get("stamp") or 0) * 1e9)
        if stamp_ns:
            # the stamp arrives as float seconds: match the kept photo to within 1 ms,
            # never by exact nanoseconds (float rounding), then the live ring
            near = min(self._snaps, key=lambda s: abs(s - stamp_ns), default=None)
            jpg = self._snaps[near] if near is not None and abs(near - stamp_ns) < 1_000_000 else None
            jpg = jpg or self._jpeg_near(stamp_ns)
            if jpg is None:
                return self._refuse(req_id, "photo_not_kept")
        elif self._jpegs:
            stamp_ns, jpg = self._jpegs[-1]
        else:
            return self._refuse(req_id, "no_frames")
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return self._refuse(req_id, "jpeg_decode")
        cam = req.get("camera")
        if cam:
            R, t = _quat_to_R(*cam["q"]), np.array(cam["t"], float)
        else:
            cam_now = self._cam_pose(stamp_ns, self._info.header.frame_id, fresh_listener=True)
            if cam_now is None:
                return self._refuse(req_id, "no_camera_pose (send pixel_to_goal's 'camera')")
            R, t = cam_now
        try:
            target = Target(img, box, point, (R, t), K, time.monotonic(), str(req.get("label", "")))
        except ValueError as e:
            return self._refuse(req_id, str(e))
        self._target, self._lock_id = target, req_id
        self._last_state = None
        self._wake()
        self.get_logger().info(f"lock {req_id}: {target.label or 'target'} at "
                               f"({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}), box {[round(b) for b in box]}"
                               f"{', on the floor' if target.floor else ''}")
        self._say(**{**target.status(time.monotonic()), "state": "locked"})

    def _on_cancel(self, msg):
        if self._target is not None and msg.data in ("", self._lock_id):
            self.get_logger().info(f"lock {self._lock_id}: cancelled (frames with depth "
                                   f"{self._paired}, without {self._unpaired})")
            self._paired = self._unpaired = 0
            self._target = None
            self._sleep()
            self._say(state="idle")

    # ── awake: colour + depth + TF ───────────────────────────────────────────
    def _wake(self):
        if self._subs:
            return
        self._subs = [self.create_subscription(Image, self._color_topic, self._on_color, qos_profile_sensor_data),
                      self.create_subscription(Image, self._depth_topic, self._on_depth, qos_profile_sensor_data)]
        if self._tf_listener is None:
            self._tf_listener = TransformListener(self._tf_buffer, self._tf_node)

    def _sleep(self):
        for s in self._subs:
            self.destroy_subscription(s)
        self._subs = []
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None
            self._tf_buffer = Buffer()      # never place with a pose from before the sleep
        self._depths.clear()
        self._pending = None

    def _cam_pose(self, stamp_ns, frame, fresh_listener=False):
        if fresh_listener and self._tf_listener is None:
            self._tf_listener = TransformListener(self._tf_buffer, self._tf_node)
        deadline = time.monotonic() + TF_RETRY_S
        while True:
            try:
                tf = self._tf_buffer.lookup_transform(self._frame, frame, Time(nanoseconds=stamp_ns))
                q, tr = tf.transform.rotation, tf.transform.translation
                return _quat_to_R(q.x, q.y, q.z, q.w), np.array([tr.x, tr.y, tr.z])
            except Exception:  # noqa: BLE001 -- not in yet / extrapolation: retry briefly, then skip
                if time.monotonic() > deadline:
                    return None
                time.sleep(0.01)

    # Colour and aligned depth carry the SAME stamp but arrive as two messages in
    # either order. Pairing a colour frame with "the latest depth" used the previous
    # depth (66 ms older) whenever colour came first, failed the time check, and the
    # tracker ran on bearing alone -- a 25 cm range error was never corrected (twin,
    # 2026-10-21). So: keep a few depths, pair by stamp, and let a colour frame wait
    # briefly for its own depth.
    def _depth_for(self, ns):
        best = min(self._depths, key=lambda m: abs(_ns(m.header.stamp) - ns), default=None)
        return best if best is not None and abs(_ns(best.header.stamp) - ns) <= DEPTH_MAX_DT_S * 1e9 else None

    def _on_depth(self, msg):
        self._depths.append(msg)
        if self._pending is not None:
            colour, _ = self._pending
            if abs(_ns(colour.header.stamp) - _ns(msg.header.stamp)) <= DEPTH_MAX_DT_S * 1e9:
                self._pending = None
                self._process(colour, msg)

    def _on_color(self, msg):
        if self._target is None or self._info is None:
            return
        now = time.monotonic()
        if self._pending is not None and now - self._pending[1] > PAIR_WAIT_S:
            colour, self._pending = self._pending[0], None     # its depth never came: go without
            self._process(colour, None)
        if now - self._last_t < 1.0 / self._max_hz:
            return
        self._last_t = now
        d = self._depth_for(_ns(msg.header.stamp))
        if d is None:
            self._pending = (msg, now)          # wait for its depth (_on_depth runs it)
            return
        self._process(msg, d)

    def _process(self, msg, depth_msg):
        if self._target is None:
            return
        now = time.monotonic()
        ns = _ns(msg.header.stamp)
        cam = self._cam_pose(ns, msg.header.frame_id or self._info.header.frame_id)
        if cam is None:
            return
        img = _decode(msg)
        if img is None:
            return
        depth = _decode(depth_msg) if depth_msg is not None else None
        if depth is not None and depth.shape[:2] != img.shape[:2]:
            depth = None            # not aligned to colour: never sample it at colour pixels
        if depth is None:
            self._unpaired += 1
        else:
            self._paired += 1
        K = np.array(self._info.k, float).reshape(3, 3)
        st = self._target.update(img, depth, cam, K, now)
        if st["state"] != self._last_state:
            self.get_logger().info(f"lock {self._lock_id}: {st['state']} "
                                   f"(object {st['object']}, sigma {st['sigma_m']} m, from lock {st['from_lock_m']} m)")
            self._last_state = st["state"]
        self._say(stamp=ns / 1e9, with_depth=depth is not None, **st)


def _spin(node):
    """EventsExecutor, as image_bridge / pixel_to_goal (ROVER_EXECUTOR=single: rclpy.spin)."""
    if os.environ.get("ROVER_EXECUTOR", "events") != "events":
        rclpy.spin(node)
        return
    from rclpy.experimental import EventsExecutor
    ex = EventsExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        ex.shutdown()


def main():
    rclpy.init()
    node = TargetTrackNode()
    try:
        _spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
