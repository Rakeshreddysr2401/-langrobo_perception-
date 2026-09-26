#!/usr/bin/env python3
"""ghost_check.py -- which layer is holding an obstacle nobody can see?

    python3 phase3/tools/ghost_check.py          (in the rover container, stack up)

Owner, 2026-09-26: "if someone goes in front of the rover it gets the light
blue circle with pink, but it does not disappear when he goes off" -- only in
front, only sometimes, only while the rover stands still.

The local costmap is the per-cell MAXIMUM of two layers (nav2.yaml):
    nvblox_layer  the camera, 5-27 cm, ahead only, REMEMBERS (decay off by design)
    lidar_layer   one plane at 25 cm, all round, cleared by raytracing
so a cell that stays lethal is held by one of them. This takes every lethal
cell ahead of the rover (0.35-2.5 m, +-1 m) and asks what backs it NOW:

    live           a LiDAR point within 6 cm, in the last 10 scans
    camera memory  nvblox's planning slice (the exact thing the costmap layer
                   reads) marks it, and the LiDAR sees nothing there: a low
                   object under the LiDAR plane -- or a person nvblox has not
                   yet seen leave (it can only clear a spot by seeing PAST it,
                   within its 3 m integration range)
    lidar layer    neither: the LiDAR layer marked it once and no beam has
                   crossed it since (a nearer thing in the way, or no return)

First run, 2026-09-26: a walk-in cleared from both maps within ~2 s; a
~400-cell blob that predated the test cleared when the owner walked past it
again, with the camera grid unchanged -- so run this WHILE a stuck blob is on
screen to see which layer holds it.
"""
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from nvblox_msgs.msg import DistanceMapSlice
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

AHEAD = (0.35, 2.5)     # m forward
SIDE = 1.0              # m either side
NEAR = 0.06             # m: "backed by" radius
SLICE_SOLID = 0.0       # m: slice distance at or under this is inside an obstacle (binary layer rule)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def near(A, B, d):
    """For each point of A, is any point of B within d? (chunked, no scipy)"""
    out = np.zeros(len(A), bool)
    if len(A) == 0 or len(B) == 0:
        return out
    for i in range(0, len(A), 400):
        D = np.hypot(A[i:i + 400, None, 0] - B[None, :, 0], A[i:i + 400, None, 1] - B[None, :, 1])
        out[i:i + 400] = D.min(1) < d
    return out


def main():
    rclpy.init()
    n = Node('ghost_check')
    got, pose, scans = {}, [None], []
    vol = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)
    lat = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: got.__setitem__('cm', m), lat)
    n.create_subscription(DistanceMapSlice, '/nvblox_node/static_map_slice', lambda m: got.__setitem__('sl', m), vol)
    n.create_subscription(Odometry, '/odom', lambda m: pose.__setitem__(0, m.pose.pose), 10)
    n.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    buf = Buffer()
    TransformListener(buf, n)
    end = time.time() + 15
    while time.time() < end and not ({'cm', 'sl'} <= got.keys() and pose[0] and len(scans) >= 10):
        rclpy.spin_once(n, timeout_sec=0.05)
    missing = [k for k, ok in (('costmap', 'cm' in got), ('nvblox slice', 'sl' in got),
                               ('/odom', pose[0] is not None), ('/scan', len(scans) >= 10)) if not ok]
    if missing:
        print('no data from: ' + ', '.join(missing))
        return 1

    p = pose[0]
    th, px, py = yaw_of(p.orientation), p.position.x, p.position.y

    m = got['cm']
    a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    ys, xs = np.nonzero(a == 100)                      # nav2 publishes 0-100: 100 = lethal
    r = m.info.resolution
    C = np.c_[m.info.origin.position.x + (xs + .5) * r, m.info.origin.position.y + (ys + .5) * r]
    dx, dy = C[:, 0] - px, C[:, 1] - py
    f, l = dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th)
    box = (f > AHEAD[0]) & (f < AHEAD[1]) & (np.abs(l) < SIDE)
    C, F, L = C[box], f[box], l[box]

    s = got['sl']
    d = np.array(s.data, dtype=np.float32).reshape(s.height, s.width)
    sy, sx = np.nonzero((d <= SLICE_SOLID) & (d != s.unknown_value))
    S = np.c_[s.origin.x + (sx + .5) * s.resolution, s.origin.y + (sy + .5) * s.resolution]

    tr = buf.lookup_transform('odom', scans[-1].header.frame_id, Time())
    ly, lx0, ly0 = yaw_of(tr.transform.rotation), tr.transform.translation.x, tr.transform.translation.y
    pts = []
    for sc in scans[-10:]:
        rr = np.asarray(sc.ranges)
        aa = sc.angle_min + np.arange(rr.size) * sc.angle_increment
        ok = np.isfinite(rr) & (rr > 0.05)
        pts.append(np.c_[lx0 + rr[ok] * np.cos(aa[ok] + ly), ly0 + rr[ok] * np.sin(aa[ok] + ly)])
    P = np.vstack(pts)

    live = near(C, P, NEAR)
    cam = ~live & near(C, S, NEAR)
    stuck = ~live & ~cam
    print(f'lethal cells ahead ({AHEAD[0]}-{AHEAD[1]} m, +-{SIDE} m, {r * 100:.1f} cm cells): {len(C)}')
    print(f'  live (LiDAR sees it now)       {live.sum():5d}')
    print(f'  camera memory (nvblox only)    {cam.sum():5d}   low object, or a person not yet seen to leave')
    print(f'  stuck in the LiDAR layer       {stuck.sum():5d}   no beam has crossed it since it was marked')
    for name, sel in (('camera memory', cam), ('stuck in the LiDAR layer', stuck)):
        if sel.sum() < 5:
            continue
        H, xe, ye = np.histogram2d(F[sel], L[sel], bins=[np.arange(AHEAD[0], AHEAD[1] + 0.01, 0.25),
                                                         np.arange(-SIDE, SIDE + 0.01, 0.25)])
        print(f'\n  where ({name}): rows metres ahead, columns metres LEFT(+) to RIGHT(-)')
        print('         ' + ' '.join(f'{c:+5.2f}' for c in ye[:-1][::-1]))
        for i, x0 in enumerate(xe[:-1]):
            print(f'   {x0:4.2f}  ' + ' '.join(f'{int(v):5d}' for v in H[i][::-1]))
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
