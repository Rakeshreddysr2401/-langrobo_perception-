#!/usr/bin/env python3
"""memory_where -- "where is X?" through the frame memory, checked against the LiDAR (robot standing still).

Run INSIDE the rover container (reads only; never drives):
  python3 /opt/rover/perception/tools/memory_where.py "a helmet" "a dustbin" ...

For each thing: the Mac's /find (newest confirmed sighting) -> pixel_to_goal places that frame's box on
its own depth and pose -> the object point in the LiDAR's frame -> the current /scan's nearest return
within +-3 deg of that bearing. The LiDAR is an independent measurement, but only of things that reach its
plane (25 cm up): for lower things it reports what is BEHIND them (shown, not counted).

Frame memory status item 3 (Pi docs/plans/FRAME_MEMORY.md), 2026-11-21.
"""
import json
import math
import sys
import time
import urllib.request

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

MAC = 'http://192.168.1.10:8095'
LIDAR_FRAME_GUESS = 'laser'


def qmat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class N(Node):
    def __init__(self):
        super().__init__('memory_where')
        self.scan, self.results = None, {}
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 5)
        self.create_subscription(String, '/vision/pixel_result', self._res, 10)
        self.q = self.create_publisher(PointStamped, '/vision/pixel_query', 5)

    def _res(self, m):
        j = json.loads(m.data)
        self.results[j.get('id')] = j


def spin_until(n, cond, t):
    end = time.time() + t
    while time.time() < end and not cond():
        rclpy.spin_once(n, timeout_sec=0.1)


def find(words):
    r = urllib.request.Request(MAC + '/find', json.dumps({'words': words, 'frames': 6, 'enough': 1}).encode(),
                               {'Content-Type': 'application/json'})
    return json.load(urllib.request.urlopen(r, timeout=120))


rclpy.init()
n = N()
spin_until(n, lambda: n.scan is not None and n.q.get_subscription_count() > 0, 10)
spin_until(n, lambda: False, 1.0)
frame = n.scan.header.frame_id or LIDAR_FRAME_GUESS
print(f"{'thing':16s} {'seen':>8s}  {'ahead':>6s} {'side':>10s}   camera  LiDAR  diff")
for words in sys.argv[1:]:
    t0 = time.time()
    r = find(words)
    f = (r.get('found') or [None])[0]
    if not f:
        m = r.get('maybe')
        print(f"{words:16s} not seen" + (f"  (maybe: {m['looks_like']})" if m else ''))
        continue
    st = int(f['stamp_ns'])
    x0, y0, x1, y1 = f['box']
    q = PointStamped()
    q.header.stamp.sec, q.header.stamp.nanosec = st // 10**9, st % 10**9
    rid = f'mw{len(n.results)}'
    q.header.frame_id = f'{rid};box={x0},{y0},{x1},{y1};what={words}'
    q.point.x, q.point.y = (x0 + x1) / 2, (y0 + y1) / 2
    n.q.publish(q)
    spin_until(n, lambda: rid in n.results, 8)
    g = n.results.get(rid)
    if not g or not g.get('ok'):
        print(f"{words:16s} found but not placed: {(g or {}).get('reason')}")
        continue
    # the object in the LiDAR's own frame, now
    o = g['object']
    tf = n.tf.lookup_transform(frame, 'odom', Time())
    t = tf.transform.translation
    p = qmat(tf.transform.rotation) @ np.array([o['x'], o['y'], o['z']]) + np.array([t.x, t.y, t.z])
    rng_cam = math.hypot(p[0], p[1])
    bear = math.atan2(p[1], p[0])
    s = n.scan
    angs = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
    d = np.abs((angs - bear + math.pi) % (2 * math.pi) - math.pi)
    rr = np.array(s.ranges)
    ok = (d < math.radians(3)) & np.isfinite(rr) & (rr > s.range_min)
    rng_lidar = float(rr[ok].min()) if ok.any() else None
    # ahead / side from the LiDAR centre, in the ROBOT's directions (base_link axes)
    tb = n.tf.lookup_transform('base_link', 'odom', Time())
    tb_t = tb.transform.translation
    pb = qmat(tb.transform.rotation) @ np.array([o['x'], o['y'], o['z']]) + np.array([tb_t.x, tb_t.y, tb_t.z])
    tl = n.tf.lookup_transform('base_link', frame, Time()).transform.translation
    ahead, left = pb[0] - tl.x, pb[1] - tl.y
    seen = time.strftime('%H:%M:%S', time.localtime(st / 1e9))
    diff = '' if rng_lidar is None else f'{(rng_cam - rng_lidar) * 100:+5.0f} cm'
    print(f"{words:16s} {seen:>8s}  {ahead * 100:4.0f}cm {abs(left) * 100:4.0f}cm {'left ' if left >= 0 else 'right'}"
          f"   {rng_cam:5.2f}m  {'' if rng_lidar is None else f'{rng_lidar:5.2f}m'}  {diff}   "
          f"({f['decided_by']}, {time.time() - t0:.0f} s)")
rclpy.shutdown()
