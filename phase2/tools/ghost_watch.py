#!/usr/bin/env python3
"""ghost_watch.py — what the camera map holds in front of the rover, live.

    python3 phase2/tools/ghost_watch.py [SECONDS] [back]   (in the rover container, map layer up)

Every 2 s, for the box 0.4-2.5 m ahead (or, with "back", behind) and 1 m either
side of the rover:
    nvblox   cells the camera marked (nvblox's own grid, never forgets)
    erased   of those, erased by ghost_clear (moved, or under the rover)
    kept     what nav2 / goal_exec see (/ghost_clear/grid)
The floor tests (2026-10-04): stand in front, the rover turns away, walk off
-> nvblox keeps the legs (the camera no longer sees there), 'erased' rises and
'kept' falls back; put a shoe down -> 'kept' rises and stays.
"""
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

AHEAD = (0.4, 2.5)
SIDE = 1.0


def count(m, pose, sign):
    if m is None or pose is None:
        return None
    i = m.info
    a = np.asarray(m.data, np.int8).reshape(i.height, i.width)
    jj, ii = np.nonzero(a >= 50)
    x = i.origin.position.x + (ii + 0.5) * i.resolution - pose[0]
    y = i.origin.position.y + (jj + 0.5) * i.resolution - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    fwd, side = sign * (c * x + s * y), -s * x + c * y
    return int(((fwd >= AHEAD[0]) & (fwd <= AHEAD[1]) & (np.abs(side) <= SIDE)).sum())


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    sign = -1.0 if 'back' in sys.argv[2:] else 1.0
    rclpy.init()
    n = rclpy.create_node('ghost_watch')
    got = {}
    rel = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    for topic, key in (('/nvblox_node/static_occupancy_grid', 'raw'),
                       ('/ghost_clear/grid', 'kept'), ('/ghost_clear/erased', 'erased')):
        n.create_subscription(OccupancyGrid, topic, lambda m, k=key: got.__setitem__(k, m), rel)

    def odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        got['pose'] = (p.x, p.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))
    n.create_subscription(Odometry, '/odom', odom, 10)
    where = 'behind' if sign < 0 else 'ahead'
    print(f'  {where} {AHEAD[0]}-{AHEAD[1]} m, +-{SIDE} m:   nvblox   erased   kept', flush=True)
    t0, nxt = time.time(), 0.0
    while time.time() - t0 < secs:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - t0 >= nxt:
            nxt += 2.0
            p = got.get('pose')
            r, e, k = (count(got.get(x), p, sign) for x in ('raw', 'erased', 'kept'))
            print(f'  {time.time() - t0:5.0f} s                         {r!s:>6}   {e!s:>6}   {k!s:>6}', flush=True)


if __name__ == '__main__':
    main()
