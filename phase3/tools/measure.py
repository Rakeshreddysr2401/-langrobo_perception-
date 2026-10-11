#!/usr/bin/env python3
"""measure.py — tape-measure check: where does each source put the thing in front?

    python3 measure.py [--bearing 0] [--half 0.10] [--once]

MOVES NOTHING. Once a second, looking along --bearing (degrees, 0 = straight
ahead, + = left) in a corridor --half metres either side of that line, it prints
the nearest obstacle as each source sees it:

    scan        raw /scan, the LiDAR plane (25 cm)
    L-lidar     local costmap, lidar_layer         (lethal cells)
    L-nvblox    local costmap, nvblox_layer        (camera depth)
    G-lidar     global costmap, lidar_layer
    G-nvblox    global costmap, nvblox_layer
    L-master    local costmap after the merge (what MPPI checks against)

Each column is   front=<cm from the FRONT BUMPER (x = +0.182)>  w=<cm wide>  age=<s>
so lay the tape from the front edge of the body, straight out. For
|--bearing| >= 60 deg "front" is from the rover's CENTRE instead (the side
edge is 19 cm out, the back edge 17.8 cm). "w" is the
lateral extent of the lethal cells inside the corridor (an object narrower than
2 * --half shows its own width). "age" is how old that source's last message is:
a source that stops updating shows here first.

Use it to check (OPEN_ISSUES / NAV_GUIDE §2):
  1. distance  — box at 30 / 50 / 100 / 150 cm on the tape: each row within ~3 cm?
  2. low thing — a shoe or a book (< 25 cm): scan + L-lidar see nothing,
                 nvblox must.
  3. clearing  — take the object away: lidar columns go "none" within ~1 s,
                 nvblox stays (it remembers; ghost_clear only erases what the
                 LiDAR also saw).
"""
import argparse
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

FRONT = 0.182                                    # base_link -> front bumper (nav2.yaml footprint)
LETHAL = 100                                     # nav2 publishes 254 as 100
GRIDS = [
    ('L-lidar', '/local_costmap/lidar_layer'),
    ('L-nvblox', '/local_costmap/nvblox_layer'),
    ('G-lidar', '/global_costmap/lidar_layer'),
    ('G-nvblox', '/global_costmap/nvblox_layer'),
    ('L-master', '/local_costmap/costmap'),
]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Measure(Node):
    def __init__(self, bearing_deg, half):
        super().__init__('measure')
        self.bearing = math.radians(bearing_deg)
        self.half = half
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.grids = {}                          # name -> (msg, rx time)
        self.scan = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name, topic in GRIDS:
            self.create_subscription(OccupancyGrid, topic,
                                     lambda m, n=name: self._grid(n, m), latched)
        self.create_subscription(LaserScan, '/scan', self._scan, 10)

    def _grid(self, name, m):
        self.grids[name] = m

    def _scan(self, m):
        self.scan = m

    def age(self, stamp):
        return (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9

    def from_points(self, xs, ys):
        """base_link points -> (front cm, width cm) of the nearest in the corridor."""
        c, s = math.cos(self.bearing), math.sin(self.bearing)
        along = xs * c + ys * s                  # rotate into the bearing's frame
        across = -xs * s + ys * c
        keep = (along > 0) & (np.abs(across) <= self.half)
        if not keep.any():
            return None
        a, w = along[keep], across[keep]
        near = a.min()
        blob = a <= near + 0.10                  # the nearest surface, 10 cm deep
        front = near - FRONT / max(c, 1e-3) if abs(self.bearing) < math.radians(60) else near
        return front * 100, (w[blob].max() - w[blob].min()) * 100

    def grid_hit(self, m):
        try:
            t = self.tf.lookup_transform('base_link', m.header.frame_id, Time())
        except Exception as e:                   # noqa: BLE001 -- show, don't die
            return f'no tf ({type(e).__name__})'
        data = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        rows, cols = np.nonzero(data >= LETHAL)
        if rows.size == 0:
            return 'none'
        r = m.info.resolution
        gx = m.info.origin.position.x + (cols + 0.5) * r
        gy = m.info.origin.position.y + (rows + 0.5) * r
        tr, yw = t.transform.translation, yaw_of(t.transform.rotation)
        bx = tr.x + gx * math.cos(yw) - gy * math.sin(yw)
        by = tr.y + gx * math.sin(yw) + gy * math.cos(yw)
        hit = self.from_points(bx, by)
        return 'none' if hit is None else f'{hit[0]:6.1f}  w={hit[1]:4.0f}'

    def scan_hit(self, m):
        try:
            t = self.tf.lookup_transform('base_link', m.header.frame_id, Time())
        except Exception as e:                   # noqa: BLE001
            return f'no tf ({type(e).__name__})'
        rng = np.asarray(m.ranges)
        ang = m.angle_min + np.arange(rng.size) * m.angle_increment
        ok = np.isfinite(rng) & (rng >= m.range_min) & (rng <= m.range_max)
        rng, ang = rng[ok], ang[ok]
        tr, yw = t.transform.translation, yaw_of(t.transform.rotation)
        lx, ly = rng * np.cos(ang), rng * np.sin(ang)
        bx = tr.x + lx * math.cos(yw) - ly * math.sin(yw)
        by = tr.y + lx * math.sin(yw) + ly * math.cos(yw)
        hit = self.from_points(bx, by)
        return 'none' if hit is None else f'{hit[0]:6.1f}  w={hit[1]:4.0f}'

    def report(self):
        lines = [time.strftime('%H:%M:%S') +
                 f'  bearing {math.degrees(self.bearing):+.0f} deg, ±{self.half * 100:.0f} cm'
                 '   front(cm)  width(cm)  age(s)']
        if self.scan is None:
            lines.append(f'  {"scan":9s} no message')
        else:
            lines.append(f'  {"scan":9s} {self.scan_hit(self.scan):22s} '
                         f'age={self.age(self.scan.header.stamp):5.2f}')
        for name, _ in GRIDS:
            m = self.grids.get(name)
            if m is None:
                lines.append(f'  {name:9s} no message')
                continue
            lines.append(f'  {name:9s} {self.grid_hit(m):22s} age={self.age(m.header.stamp):5.2f}')
        print('\n'.join(lines), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bearing', type=float, default=0.0, help='degrees, 0 = ahead, + = left')
    ap.add_argument('--half', type=float, default=0.10, help='corridor half-width, m')
    ap.add_argument('--once', action='store_true', help='one report after 3 s, then exit')
    a = ap.parse_args()
    rclpy.init()
    n = Measure(a.bearing, a.half)
    end = time.time() + 3.0
    while rclpy.ok() and time.time() < end:      # let TF and the latched grids arrive
        rclpy.spin_once(n, timeout_sec=0.1)
    try:
        while rclpy.ok():
            n.report()
            if a.once:
                break
            end = time.time() + 1.0
            while time.time() < end:
                rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
