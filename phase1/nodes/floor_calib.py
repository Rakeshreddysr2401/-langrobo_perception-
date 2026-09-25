#!/usr/bin/env python3
"""floor_calib.py — measure the D555's tilt and height from the floor itself.

    ./rover camera --floor          rover parked on open, flat floor; moves nothing

Averages ~20 depth frames, back-projects the lower part of the image (the
floor in front of the nose), fits a plane with RANSAC, and reads off where
camera0_link sits relative to that plane:

    pitch   nose-down positive (ROS/URDF convention, rotation about y)
    roll    left-side-up positive (rotation about x)
    height  camera0_link above the floor

WHY IT MATTERS. nvblox turns everything between two heights into obstacles.
With the camera tilted but the TF saying level, the floor itself rises with
range (1.3 deg is 6.8 cm at 3 m), so the obstacle band had to start at 10 cm
and every object under 10 cm -- a remote, a shoe, a 6-8 cm box -- was
invisible to navigation. With the measured tilt in the TF the floor lands at
z ~ 0 and the band can start at 5 cm (one voxel).

Prints the lines to put in description/params.yaml. Writes nothing.
"""
import math
import sys
import time
import warnings

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

NS = '/camera/camera0/depth'
FRAMES = 20
ZMIN, ZMAX = 0.35, 2.0          # m along the optical axis: past min-Z, before noise grows
RANSAC_TOL = 0.01               # m
ITERS = 400


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Floor(Node):
    def __init__(self):
        super().__init__('floor_calib')
        self.info, self.frames = None, []
        self.create_subscription(CameraInfo, f'{NS}/camera_info', self._info, qos_profile_sensor_data)
        # reliable: at 900 kB a frame, best-effort drops about half of them
        self.create_subscription(Image, f'{NS}/image_rect_raw', self._img,
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)

    def _info(self, m):
        self.info = m

    def _img(self, m):
        if len(self.frames) >= FRAMES or m.encoding not in ('16UC1', 'mono16'):
            return
        d = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
        self.frames.append((m.header.frame_id, d.astype(np.float32) * 0.001))


def fit_plane(P, rng):
    best, best_n = None, 0
    for _ in range(ITERS):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((P - a) @ n) < RANSAC_TOL
        if inl.sum() > best_n:
            best, best_n = inl, inl.sum()
    Q = P[best]
    c = Q.mean(axis=0)
    # the normal is the covariance's smallest eigenvector. (A plain SVD of the
    # points builds an N x N matrix it never uses: 4.3 GB at 24k points, OOM.)
    _, v = np.linalg.eigh((Q - c).T @ (Q - c))
    n = v[:, 0]
    return n, c, best


def main():
    rclpy.init()
    node = Floor()
    end = time.time() + 20
    while (node.info is None or len(node.frames) < FRAMES) and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.info is None or len(node.frames) < 5:
        sys.exit(f'  no depth on {NS} ({len(node.frames)} frames) -- ./rover camera first')
    frame = node.frames[0][0]
    end = time.time() + 5
    while not node.buf.can_transform('camera0_link', frame, Time()) and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    tf = node.buf.lookup_transform('camera0_link', frame, Time()).transform
    R, t = quat_to_R(tf.rotation), np.array([tf.translation.x, tf.translation.y, tf.translation.z])

    D = np.stack([f[1] for f in node.frames])
    D[D <= 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)   # pixels with no depth at all
        z = np.nanmedian(D, axis=0)          # per-pixel median: stereo noise averages out
    h, w = z.shape
    K = np.array(node.info.k).reshape(3, 3)
    v, u = np.mgrid[0:h, 0:w]
    rows = v > int(h * 0.55)                 # the lower part of the image: floor ahead
    ok = rows & np.isfinite(z) & (z > ZMIN) & (z < ZMAX)
    if ok.sum() < 2000:
        sys.exit(f'  only {int(ok.sum())} usable floor pixels -- clear the floor in front of the nose')
    zz = z[ok]
    x = (u[ok] - K[0, 2]) * zz / K[0, 0]
    y = (v[ok] - K[1, 2]) * zz / K[1, 1]
    P = np.stack([x, y, zz], axis=1) @ R.T + t          # into camera0_link
    rng = np.random.default_rng(0)
    if len(P) > 40000:
        P = P[rng.choice(len(P), 40000, replace=False)]
    n, c, inl = fit_plane(P, rng)
    if n[2] < 0:
        n = -n                                           # the floor's normal points up
    height = float(-(c @ n))                             # camera0_link origin above the plane
    pitch = -math.asin(max(-1.0, min(1.0, n[0])))        # n = (-sin p, sin r cos p, cos r cos p)
    roll = math.atan2(n[1], n[2])
    rms = float(np.sqrt(np.mean(((P[inl] - c) @ n) ** 2)))
    frac = inl.mean()

    print(f'  {len(node.frames)} depth frames, {len(P)} floor points, {frac * 100:.0f}% on the plane, '
          f'rms {rms * 1000:.1f} mm')
    print(f'  camera0_link above the floor : {height:.4f} m')
    print(f'  pitch (nose down +)          : {math.degrees(pitch):+.2f} deg   ({pitch:+.4f} rad)')
    print(f'  roll  (left side up +)       : {math.degrees(roll):+.2f} deg   ({roll:+.4f} rad)')
    if frac < 0.6 or rms > 0.01:
        print('  ! weak fit: is the floor in front clear and flat? Numbers not trustworthy.')
        sys.exit(1)
    print('\n  description/params.yaml, camera:')
    print(f'    pitch: {{value: {pitch:.4f}, status: calibrated, source: "floor plane from depth, '
          f'./rover camera --floor", date: {time.strftime("%Y-%m-%d")}}}')
    print(f'    roll:  {{value: {roll:.4f}, status: calibrated, source: "floor plane from depth, '
          f'./rover camera --floor", date: {time.strftime("%Y-%m-%d")}}}')
    print(f'  (lens_z is {height:.3f} by this fit; the tape said 0.175)')


if __name__ == '__main__':
    main()
