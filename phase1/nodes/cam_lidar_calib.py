#!/usr/bin/env python3
"""cam_lidar_calib.py — make the depth camera agree with the LiDAR.

    ./rover camera --lidar            capture ONE pose (rover still; moves nothing)
    ./rover camera --lidar --solve    combine every pose captured so far
    ./rover camera --lidar --reset    forget them and start again

WHY. Everything the camera sees is placed in the map through the camera's
extrinsic. On 2026-09-25 the depth sat 1.1 deg rotated from the LiDAR (and
~1.5 cm off), so an object 3 m away landed ~6 cm from where the LiDAR put the
same wall; VO was using a third value (2.06 deg). This measures the camera
against the LiDAR -- the one sensor whose mount is calibrated by driving.

HOW. At the LiDAR's height (20-30 cm) the depth camera and the laser see the
same walls. Each capture takes the per-pixel median of ~15 depth frames and
~20 scans, cuts the depth to that band, and fits it to the scan point-to-line
(phase1/harness/common.icp). The fit is the correction to the camera's pose
in base_link. One view can be degenerate (a single flat wall pins nothing
along it), so capture 4-6 poses facing corners or furniture 0.6-2.5 m away,
then --solve.

Results go to /logs/calib/cam_lidar.jsonl. --solve prints the params.yaml
lines. Nothing is written to the config by this tool.
"""
import json
import math
import sys
import time
import warnings
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'harness'))
from common import icp  # noqa: E402

OUT = Path('/logs/calib/cam_lidar.jsonl')
NS = '/camera/camera0/depth'
BAND = (0.20, 0.30)          # m above the floor: straddles the 25 cm laser plane
ZMAX = 3.0                   # m along the optical axis: past this depth is too noisy


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Grab(Node):
    def __init__(self):
        super().__init__('cam_lidar_calib')
        self.info, self.depth, self.scans, self.frame = None, [], [], None
        self.create_subscription(CameraInfo, f'{NS}/camera_info', lambda m: setattr(self, 'info', m), 5)
        self.create_subscription(Image, f'{NS}/image_rect_raw', self._d,
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(LaserScan, '/scan', self._s, qos_profile_sensor_data)
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)

    def _d(self, m):
        if len(self.depth) < 15:
            self.frame = m.header.frame_id
            self.depth.append(np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
                              .astype(np.float32) * 0.001)

    def _s(self, m):
        if len(self.scans) < 20:
            self.scans.append(m)


def capture():
    rclpy.init()
    n = Grab()
    end = time.time() + 25
    while time.time() < end and not (len(n.depth) >= 15 and len(n.scans) >= 20 and n.info is not None
                                     and n.buf.can_transform('base_link', n.frame or 'x', Time())
                                     and n.buf.can_transform('base_link', 'laser', Time())):
        rclpy.spin_once(n, timeout_sec=0.05)
    if len(n.depth) < 15 or len(n.scans) < 20:
        sys.exit(f'  not enough data ({len(n.depth)} depth, {len(n.scans)} scans): camera and lidar layers up?')
    tc = n.buf.lookup_transform('base_link', n.frame, Time()).transform
    tl = n.buf.lookup_transform('base_link', 'laser', Time()).transform
    cam_link = n.buf.lookup_transform('base_link', 'camera0_link', Time()).transform

    D = np.stack(n.depth)
    D[D <= 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        Z = np.nanmedian(D, axis=0)
    K = np.array(n.info.k).reshape(3, 3)
    v, u = np.mgrid[0:Z.shape[0], 0:Z.shape[1]]
    ok = np.isfinite(Z) & (Z > 0.4) & (Z < ZMAX)
    P = np.stack([(u[ok] - K[0, 2]) * Z[ok] / K[0, 0], (v[ok] - K[1, 2]) * Z[ok] / K[1, 1], Z[ok]], 1)
    P = P @ quat_R(tc.rotation).T + [tc.translation.x, tc.translation.y, tc.translation.z]
    band = P[(P[:, 2] > BAND[0]) & (P[:, 2] < BAND[1])][:, :2]
    band = band[::max(1, len(band) // 4000)]

    R = np.array([np.asarray(s.ranges) for s in n.scans if len(s.ranges) == len(n.scans[0].ranges)])
    R[~np.isfinite(R) | (R < 0.1) | (R > 8.0)] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        r = np.nanmedian(R, axis=0)
    a = n.scans[0].angle_min + np.arange(r.size) * n.scans[0].angle_increment
    okl = np.isfinite(r)
    ly = yaw_of(tl.rotation)
    L = np.stack([r[okl] * np.cos(a[okl] + ly) + tl.translation.x,
                  r[okl] * np.sin(a[okl] + ly) + tl.translation.y], 1)
    L = L[~((L[:, 0] < 0.23) & (L[:, 0] > -0.2) & (np.abs(L[:, 1]) < 0.21))]     # the rover itself

    f = icp(band, L, (0.0, 0.0, 0.0))
    rec = {'t': time.strftime('%Y-%m-%d %H:%M:%S'), 'dx': f.x, 'dy': f.y, 'dth': f.th,
           'residual': f.residual, 'inliers': f.inliers, 'eig': [float(e) for e in f.eig],
           'ok': bool(f.ok), 'why': f.why, 'depth_pts': int(len(band)),
           'cam_tf': [cam_link.translation.x, cam_link.translation.y, yaw_of(cam_link.rotation)]}
    print(f'  depth at {BAND[0] * 100:.0f}-{BAND[1] * 100:.0f} cm: {len(band)} points; lidar: {len(L)} points')
    print(f'  correction: dx {f.x * 100:+.1f} cm  dy {f.y * 100:+.1f} cm  dyaw {math.degrees(f.th):+.2f} deg   '
          f'(residual {f.residual * 100:.1f} cm, {f.inliers} pts, min eig {f.eig[0]:.0f})')
    if not f.ok:
        print(f'  ✗ not usable: {f.why}. Face a corner or furniture 0.6-2.5 m away and capture again.')
        return
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open('a') as fh:
        fh.write(json.dumps(rec) + '\n')
    k = sum(1 for _ in OUT.open())
    print(f'  ✓ pose {k} saved. Move the rover BY HAND to a different spot and heading, then capture again.'
          f'{"  (4-6 poses, then --solve)" if k < 4 else "  Enough for --solve."}')


def solve():
    if not OUT.exists():
        sys.exit('  no poses yet: ./rover camera --lidar')
    recs = [json.loads(line) for line in OUT.open() if line.strip()]
    recs = [r for r in recs if r['ok']]
    if len(recs) < 3:
        sys.exit(f'  {len(recs)} usable pose(s); capture at least 3 (better 4-6)')
    # the camera pose in base_link each fit implies: correction applied on the left
    yaws, xs, ys, w = [], [], [], []
    for r in recs:
        cx, cy, cyaw = r['cam_tf']
        c, s = math.cos(r['dth']), math.sin(r['dth'])
        xs.append(c * cx - s * cy + r['dx'])
        ys.append(s * cx + c * cy + r['dy'])
        yaws.append(cyaw + r['dth'])
        w.append(1.0 / max(r['residual'], 0.003) ** 2)
    w = np.array(w) / sum(w)
    yaw, x, y = (float(np.dot(w, v)) for v in (yaws, xs, ys))
    print(f'  {len(recs)} poses:')
    for r, yy, xx, y2 in zip(recs, yaws, xs, ys):
        print(f'    {r["t"]}  yaw {math.degrees(yy):+.2f} deg  x {xx:.4f}  y {y2:+.4f}  (res {r["residual"] * 100:.1f} cm)')
    print(f'  camera0_link, from the LiDAR:  yaw {math.degrees(yaw):+.2f} deg (spread {math.degrees(np.std(yaws)):.2f}), '
          f'x {x:.4f} (spread {np.std(xs) * 100:.1f} cm), y {y:+.4f} (spread {np.std(ys) * 100:.1f} cm)')
    print('\n  description/params.yaml, camera:')
    print(f'    yaw: {{value: {yaw:.4f}, status: calibrated, source: "camera vs LiDAR, {len(recs)} poses, '
          f'./rover camera --lidar", date: {time.strftime("%Y-%m-%d")}}}')
    print('  x and y are a cross-check against the VO lever-arm fit (vo_lever_x/y); depth band fits'
          ' pin yaw far better than position.')


if __name__ == '__main__':
    if '--reset' in sys.argv:
        OUT.unlink(missing_ok=True)
        print('  poses cleared')
    elif '--solve' in sys.argv:
        solve()
    else:
        capture()
