#!/usr/bin/env python3
"""frame_capture -- keep every keyframe with its depth and the camera's pose at the photo's stamp.

Frame memory step 1 (Pi 5 ~/ros2_ws docs/plans/FRAME_MEMORY.md, 2026-11-21). WHEN a frame is kept:
frame_store.py. Runs no model: decode a thumbnail, look up two TFs, write three files.

    in   /camera/color/image_raw/compressed          image_bridge's JPEG; header.stamp = the camera's stamp
    in   /camera/camera0/aligned_depth_to_color/image_raw   16UC1, kept ~2 s (like pixel_to_goal)
    in   /camera/camera0/color/camera_info           K and the optical frame
    in   /fusion/status                              origin_epoch: which odom the poses are in
    in   /home/room                                  the saved house map's id ({"id", "frame": "map"})
    TF   odom <- base_link and odom <- camera optical AT THE STAMP; map <- odom at the stamp (if any)
    out  /logs/frames/<origin>/<stamp_ns>.jpg|.depth.png|.json   (host ~/rover/logs/frames)
    out  /frames/new      std_msgs/String JSON: {"id", "json", "trigger", "base": {x, y, yaw_deg}, "room"}
    out  /frames/status   std_msgs/String JSON, 1 Hz: kept / skipped (by reason) / disk MB

The .json (written last = the frame is complete):
    {"id": stamp_ns, "stamp_ns", "origin", "trigger", "room",
     "K": {"fx","fy","cx","cy","w","h"}, "camera_frame",
     "cam_odom": {"t": [x,y,z], "q": [x,y,z,w]},     the colour camera's optical pose in odom
     "base_odom": {"x","y","yaw_deg"},
     "map_odom": {"t","q"} | null,                    map <- odom at the stamp (house map); null before SLAM
     "depth": "<id>.depth.png" | null, "depth_dt_ms", "depth_units_m": 0.001}

A photo whose pose is not in TF yet waits (up to PENDING_S) -- the pose AT the stamp, never the newest
one: turning at 30 deg/s, 0.25 s of lag puts a thing 2 m away 25 cm sideways (2026-11-20 review).
Disk: FRAMES_MAX_GB (default 5), oldest frames go first.
"""
import json
import os
import struct
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

import frame_store as fs

ROOT = os.environ.get('FRAMES_DIR', '/logs/frames')
MAX_BYTES = int(float(os.environ.get('FRAMES_MAX_GB', '5')) * 1e9)
DEPTH_RING_S, DEPTH_MAX_DT_S = 2.0, 0.10
PENDING_S = 1.0                      # a photo waits this long for its TF


def _q_yaw_deg(q) -> float:
    return float(np.degrees(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))


def _tf_dict(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return {'t': [round(t.x, 4), round(t.y, 4), round(t.z, 4)],
            'q': [round(q.x, 6), round(q.y, 6), round(q.z, 6), round(q.w, 6)]}


class FrameCapture(Node):
    def __init__(self):
        super().__init__('frame_capture')
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.trigger = fs.Trigger()
        self.depth = deque()                 # (stamp_ns, Image msg) -- decoded only when kept
        self.pending = deque()               # (arrived monotonic, CompressedImage)
        self.info = None
        self.origin = None
        self.room = None
        self.stats = {'kept': 0, 'by': {}, 'skipped': {}, 'images': 0, 'pruned': 0}
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self._on_img, 5)
        # RAW: 30 Hz x 225 KB; unpacking each one in Python cost 27 % of a core (measured 2026-11-21).
        # Only the stamp is read here; the kept frame is unpacked in _consider.
        self.create_subscription(Image, '/camera/camera0/aligned_depth_to_color/image_raw', self._on_depth, 2,
                                 raw=True)
        self.create_subscription(CameraInfo, '/camera/camera0/color/camera_info', self._on_info, 2)
        self.create_subscription(String, '/fusion/status', self._on_status, 2)
        self.create_subscription(String, '/home/room', self._on_room, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.pub_new = self.create_publisher(String, '/frames/new', 10)
        self.pub_status = self.create_publisher(String, '/frames/status', 2)
        self.create_timer(0.1, self._drain)
        self.create_timer(1.0, self._status)
        self.create_timer(60.0, self._prune)
        self.get_logger().info(f'frame_capture up: {ROOT}, keyframe per {fs.MOVE_M} m / {fs.TURN_DEG} deg / '
                               f'scene diff {fs.SCENE_DIFF} / {fs.STILL_S} s still, max {MAX_BYTES / 1e9:.0f} GB')

    # ── inputs ───────────────────────────────────────────────────────────────
    def _on_info(self, m):
        self.info = m

    def _on_status(self, m):
        try:
            self.origin = json.loads(m.data).get('origin_epoch')
        except Exception:
            pass

    def _on_room(self, m):
        try:
            self.room = json.loads(m.data).get('id')
        except Exception:
            pass

    def _on_depth(self, m: bytes):
        sec, nsec = struct.unpack_from('<iI', m, 4)      # CDR: 4-byte encapsulation, then header.stamp
        st = sec * 10**9 + nsec
        if self.depth and st - self.depth[-1][0] < 0.05e9:
            return
        self.depth.append((st, m))
        while self.depth and st - self.depth[0][0] > DEPTH_RING_S * 1e9:
            self.depth.popleft()

    def _on_img(self, m):
        self.stats['images'] += 1
        self.pending.append((time.monotonic(), m))

    def _skip(self, why):
        self.stats['skipped'][why] = self.stats['skipped'].get(why, 0) + 1

    # ── the work ─────────────────────────────────────────────────────────────
    def _drain(self):
        while self.pending:
            arrived, m = self.pending[0]
            st = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            stamp = Time(nanoseconds=st)
            if self.info is None or self.origin is None:
                self.pending.popleft()
                self._skip('no_camera_info_or_origin')
                continue
            try:
                base = self.tf.lookup_transform('odom', 'base_link', stamp)
                cam = self.tf.lookup_transform('odom', self.info.header.frame_id, stamp)
            except Exception:
                if time.monotonic() - arrived < PENDING_S:
                    return                   # its pose is not in TF yet: wait, keep order
                self.pending.popleft()
                self._skip('no_tf_at_stamp')
                continue
            self.pending.popleft()
            self._consider(m, st, stamp, base, cam)

    def _consider(self, m, st, stamp, base, cam):
        b = base.transform
        pose = (b.translation.x, b.translation.y, _q_yaw_deg(b.rotation))
        gray = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_REDUCED_GRAYSCALE_4)
        th = fs.thumb(gray) if gray is not None else None
        why = self.trigger.check(pose, st / 1e9, th)
        if why is None:
            return
        depth_png, depth_dt = None, None
        if self.depth:
            dst, dm = min(self.depth, key=lambda d: abs(d[0] - st))
            depth_dt = (dst - st) / 1e6
            if abs(depth_dt) <= DEPTH_MAX_DT_S * 1e3:
                arr = self.bridge.imgmsg_to_cv2(deserialize_message(dm, Image), desired_encoding='passthrough')
                ok, enc = cv2.imencode('.png', arr, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                depth_png = enc.tobytes() if ok else None
        if depth_png is None:
            self._skip('kept_without_depth')
        try:
            mo = _tf_dict(self.tf.lookup_transform('map', 'odom', stamp))
        except Exception:
            mo = None
        K = self.info.k
        meta = {'id': st, 'stamp_ns': st, 'origin': self.origin, 'trigger': why, 'room': self.room,
                'K': {'fx': K[0], 'fy': K[4], 'cx': K[2], 'cy': K[5], 'w': self.info.width, 'h': self.info.height},
                'camera_frame': self.info.header.frame_id, 'cam_odom': _tf_dict(cam),
                'base_odom': {'x': round(pose[0], 4), 'y': round(pose[1], 4), 'yaw_deg': round(pose[2], 2)},
                'map_odom': mo, 'depth': f'{st}.depth.png' if depth_png else None,
                'depth_dt_ms': None if depth_dt is None else round(depth_dt, 1), 'depth_units_m': 0.001}
        path = fs.write_bundle(ROOT, meta, bytes(m.data), depth_png)
        self.trigger.kept(pose, st / 1e9, th)
        self.stats['kept'] += 1
        self.stats['by'][why] = self.stats['by'].get(why, 0) + 1
        self.pub_new.publish(String(data=json.dumps({'id': st, 'json': path, 'trigger': why, 'room': self.room,
                                                     'base': meta['base_odom']})))

    def _status(self):
        self.pub_status.publish(String(data=json.dumps({**self.stats, 'pending': len(self.pending),
                                                        'depth_ring': len(self.depth), 'origin': self.origin})))

    def _prune(self):
        self.stats['disk_mb'] = round(fs.disk_use(ROOT) / 1e6, 1)
        n = fs.prune(ROOT, MAX_BYTES)
        if n:
            self.stats['pruned'] += n
            self.get_logger().info(f'pruned {n} oldest frames (over {MAX_BYTES / 1e9:.0f} GB)')


def main():
    rclpy.init()
    n = FrameCapture()
    try:
        # EventsExecutor: the default executor's per-message bookkeeping was most of this node's CPU
        # (30 Hz depth + 4 Hz JPEG + TF; profiled 2026-11-21). Fallback: the default one.
        try:
            from rclpy.experimental import EventsExecutor
            ex = EventsExecutor()
        except Exception:
            from rclpy.executors import SingleThreadedExecutor
            ex = SingleThreadedExecutor()
        ex.add_node(n)
        ex.spin()
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
