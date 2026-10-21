#!/usr/bin/env python3
"""bag_frames.py — a recorded drive -> colour frames + the camera's odom pose at each.

    # in the rover container (rosbag2_py, tf2_ros):
    python3 bag_frames.py /logs/bags/<run> OUT_DIR [--every N]

Writes OUT_DIR/frames/<i>.jpg (image_bridge's JPEG, as recorded) and
OUT_DIR/meta.npz:
    stamp_ns (N,)       the colour frame's camera stamp
    R (N, D, 3, 3), t (N, D, 3)   odom <- colour optical frame at stamp + SHIFTS_MS[d]
    ok (N, D)           whether TF had that time
    shifts_ms (D,)      SHIFTS_MS
    K (3, 3), frame     the colour camera_info
    base (N, 3)         odom x, y, yaw of base_link at the stamp
The time shifts measure the camera-vs-pose time offset (LOCALIZATION_GAPS G8):
replay_target_lock.py finds the shift that makes static objects stay put.
"""
import argparse
import json
import math
import os

import numpy as np
import rosbag2_py
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

SHIFTS_MS = np.arange(-200, 201, 20)
COLOUR = "/camera/color/image_raw/compressed"
INFO = "/camera/camera0/color/camera_info"


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--every", type=int, default=1, help="keep every Nth colour frame")
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out, "frames"), exist_ok=True)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/tf", "/tf_static", COLOUR, INFO]))
    buf = Buffer(cache_time=Duration(seconds=100000))
    frames, info, k = [], None, 0
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic in ("/tf", "/tf_static"):
            for tr in deserialize_message(data, TFMessage).transforms:
                if topic == "/tf_static":
                    buf.set_transform_static(tr, "bag")
                else:
                    buf.set_transform(tr, "bag")
        elif topic == INFO:
            if info is None:
                info = deserialize_message(data, CameraInfo)
        else:
            k += 1
            if k % a.every:
                continue
            m = deserialize_message(data, CompressedImage)
            ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            frames.append((ns, bytes(m.data)))
    if info is None or not frames:
        raise SystemExit("no camera_info or no colour frames in the bag")
    cam = info.header.frame_id
    N, D = len(frames), len(SHIFTS_MS)
    R = np.zeros((N, D, 3, 3)); t = np.zeros((N, D, 3)); ok = np.zeros((N, D), bool)
    base = np.full((N, 3), np.nan); stamps = np.zeros(N, np.int64)
    for i, (ns, jpg) in enumerate(frames):
        stamps[i] = ns
        open(os.path.join(a.out, "frames", f"{i:06d}.jpg"), "wb").write(jpg)
        for d, s in enumerate(SHIFTS_MS):
            try:
                tf = buf.lookup_transform("odom", cam, Time(nanoseconds=int(ns + s * 1e6)))
            except Exception:  # noqa: BLE001 -- outside the recorded TF: marked not ok
                continue
            R[i, d] = quat_R(tf.transform.rotation)
            tr = tf.transform.translation
            t[i, d] = (tr.x, tr.y, tr.z)
            ok[i, d] = True
        try:
            b = buf.lookup_transform("odom", "base_link", Time(nanoseconds=int(ns)))
            q = b.transform.rotation
            base[i] = (b.transform.translation.x, b.transform.translation.y,
                       math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        except Exception:  # noqa: BLE001
            pass
    np.savez_compressed(os.path.join(a.out, "meta.npz"), stamp_ns=stamps, R=R, t=t, ok=ok,
                        shifts_ms=SHIFTS_MS, K=np.array(info.k).reshape(3, 3), base=base)
    json.dump({"bag": a.bag, "frame": cam, "frames": N, "width": info.width, "height": info.height},
              open(os.path.join(a.out, "info.json"), "w"))
    z = list(SHIFTS_MS).index(0)
    print(f"{N} frames, pose at the stamp for {ok[:, z].sum()} ({cam}, {info.width}x{info.height})")


if __name__ == "__main__":
    main()
