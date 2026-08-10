#!/usr/bin/env python3
"""
floor_probe.py — where does the FLOOR sit, according to the robot?

WHY
    base_link's origin is on the ground, so open floor must deproject to z ~= 0.00 m.
    On 2026-08-10 it deprojected to z = +0.04 m, which caused two separate problems:

      1. esdf_slice_min_height was 0.05, i.e. 1 cm above the apparent floor -- well
         inside depth noise. nvblox mapped THE FLOOR AS AN OBSTACLE: a solid inflation
         plateau with no lethal cells, MPPI crawled at 0.082 m/s, and nav2 aborted with
         "Failed to make progress". The band was raised to 0.12 as a workaround.
      2. It implies the base_link -> camera0_link static TF in run_stack.sh overstates
         the camera height by ~4 cm (it publishes z 0.20; the data implies ~0.16).

    Fixing the TF lets the obstacle band come back DOWN, which restores the ability to
    see short obstacles, and de-biases every 3D detection's z.

    This tool measures the error instead of guessing it. Run it on open, flat floor.

USAGE
    python3 floor_probe.py                    # measure and report
    python3 floor_probe.py --min 0.5 --max 1.6   # forward window to sample (metres)

READING THE RESULT
    floor z ~= 0.00      -> the camera TF height is right
    floor z = +0.04      -> the TF is 4 cm TOO TALL; subtract it from the static TF
    floor z = -0.03      -> the TF is 3 cm too short; add it
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros

DEPTH = "/camera/camera0/depth/image_rect_raw"
INFO = "/camera/camera0/depth/camera_info"


def quat_to_R(x, y, z, w):
    """Rotation matrix from a quaternion (same convention as tf2)."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


class FloorProbe(Node):
    def __init__(self):
        super().__init__("floor_probe")
        self.br = CvBridge()
        self.depth = None
        self.info = None
        self.frame = None
        self.create_subscription(Image, DEPTH, self._depth_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, INFO, self._info_cb,
                                 qos_profile_sensor_data)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _depth_cb(self, m):
        self.depth = self.br.imgmsg_to_cv2(m, desired_encoding="passthrough")
        self.frame = m.header.frame_id

    def _info_cb(self, m):
        self.info = m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--min", type=float, default=0.5,
                    help="nearest forward distance to sample, metres")
    ap.add_argument("--max", type=float, default=1.6,
                    help="farthest forward distance to sample, metres")
    ap.add_argument("--halfwidth", type=float, default=0.20,
                    help="half-width of the centre strip sampled, metres")
    args = ap.parse_args()

    rclpy.init()
    node = FloorProbe()

    t_end = time.time() + 8.0
    while time.time() < t_end and (node.depth is None or node.info is None):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.depth is None or node.info is None:
        print("\n  ✗ no depth/camera_info. Is the camera up? './run_stack.sh up'\n")
        rclpy.shutdown()
        return 1

    # let TF fill
    t_end = time.time() + 3.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    try:
        tr = node.tf_buffer.lookup_transform(
            "base_link", node.frame, rclpy.time.Time())
    except Exception as e:
        print(f"\n  ✗ no TF base_link -> {node.frame}: {e}\n")
        rclpy.shutdown()
        return 1

    t = tr.transform.translation
    q = tr.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.x, t.y, t.z])

    raw = np.asarray(node.depth)
    # 16UC1 depth is millimetres, 32FC1 is already metres. Decide on the ORIGINAL
    # dtype (the float cast below would erase that), with a magnitude cross-check.
    is_mm = np.issubdtype(raw.dtype, np.integer) or float(np.nanmax(raw)) > 100.0
    z = raw.astype(np.float32) * (0.001 if is_mm else 1.0)

    K = node.info.k
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    h, w = z.shape

    # Deproject every valid pixel in the lower half of the image (where floor lives).
    vs, us = np.nonzero((z > 0.15) & (z < 4.0))
    keep = vs > (h // 2)
    vs, us = vs[keep], us[keep]
    if vs.size == 0:
        print("\n  ✗ no valid depth in the lower image half — point at open floor.\n")
        rclpy.shutdown()
        return 1

    d = z[vs, us]
    # optical frame: x right, y down, z forward
    px = (us - cx) * d / fx
    py = (vs - cy) * d / fy
    pts_opt = np.stack([px, py, d], axis=1)
    pts_base = pts_opt @ R.T + T

    X, Y, Z = pts_base[:, 0], pts_base[:, 1], pts_base[:, 2]
    band = (X > args.min) & (X < args.max) & (np.abs(Y) < args.halfwidth)
    if band.sum() < 50:
        print(f"\n  ✗ only {band.sum()} points in the sample window — "
              f"move to open flat floor or widen --min/--max.\n")
        rclpy.shutdown()
        return 1

    Zb = Z[band]
    med = float(np.median(Zb))
    p16, p84 = float(np.percentile(Zb, 16)), float(np.percentile(Zb, 84))

    print(f"\n  camera frame: {node.frame}")
    print(f"  base_link -> camera translation: "
          f"x {T[0]:.3f}  y {T[1]:.3f}  z {T[2]:.3f} m")
    print(f"  sampled {band.sum()} floor points, {args.min:.1f}-{args.max:.1f} m ahead, "
          f"|y| < {args.halfwidth:.2f} m\n")

    # per-distance breakdown: a tilted camera shows z drifting with range
    print("   forward      floor z    n")
    edges = np.arange(args.min, args.max + 1e-9, 0.25)
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (X > lo) & (X < hi) & (np.abs(Y) < args.halfwidth)
        if m.sum() < 20:
            continue
        print(f"   {lo:.2f}-{hi:.2f} m   {np.median(Z[m]):+.3f} m   {m.sum()}")

    print(f"\n  FLOOR z (median) = {med:+.3f} m   (16-84%: {p16:+.3f} .. {p84:+.3f})")
    print(f"  ideal is 0.000 — base_link's origin is on the ground\n")

    # A pure height error offsets the floor by a CONSTANT. A pitch error makes the
    # apparent floor z drift with range. Separating them matters: shifting the static
    # TF z only fixes the constant part, so report the slope too.
    Xb = X[band]
    if Xb.ptp() > 0.3:
        slope, intercept = np.polyfit(Xb, Zb, 1)
        pitch_deg = math.degrees(math.atan(slope))
        print(f"  plane fit: z = {slope:+.4f} * x {intercept:+.4f}")
        print(f"  => camera pitch error ~ {pitch_deg:+.2f}° "
              f"({'nose-up' if slope > 0 else 'nose-down'}), "
              f"height error at x=0: {intercept:+.3f} m")
        if abs(pitch_deg) > 1.5:
            print(f"  ⚠ pitch >1.5° — shim the camera mount; a z-only TF fix cannot")
            print(f"    correct a tilt, and the floor will still rise with distance.")
        print()

    err = med
    if abs(err) < 0.015:
        print(f"  ✓ within 1.5 cm. The camera height TF is good.")
    else:
        suggested = T[2] - err
        direction = "TOO TALL" if err > 0 else "TOO SHORT"
        print(f"  ✗ the base_link -> camera TF is {direction} by {abs(err) * 100:.1f} cm.")
        print(f"    Current static TF z = {T[2]:.3f}  ->  suggested z = {suggested:.3f}")
        print(f"    Edit run_stack.sh, the 'base_link -> camera0_link static TF' line:")
        print(f"        --z {suggested:.2f}")
        print(f"    Then './run_stack.sh up' and re-run this probe to confirm ~0.000.")
        if err > 0:
            print(f"\n    NOTE: a positive error is what made nvblox map the floor as an")
            print(f"    obstacle. Once this reads ~0, esdf_slice_min_height in")
            print(f"    config/nvblox.yaml can come back down from 0.12 toward ~0.06,")
            print(f"    which restores seeing obstacles shorter than 12 cm.")
    print()

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
