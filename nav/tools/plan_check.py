#!/usr/bin/env python3
"""plan_check.py — does the planner give paths the BODY fits?  MOVES NOTHING.

    python3 plan_check.py [--r 1.0,2.0] [--n 8] [--csv /logs/plan_check.csv]

Asks planner_server (ComputePathToPose, the planner only -- no controller, no
wheels) for a ring of goals around where the rover stands: n directions at each
radius, each goal facing outward. For every returned path it sweeps the rover's
REAL outline (description/params.yaml, no padding) along every pose against
the current global costmap and reports:

    ok         a path came back
    ms         planning time
    len        path length (m)
    clear_cm   the closest any LETHAL cell comes to the outline along the path
               (negative = the body overlaps it)
    first_deg  how far the path's first 20 cm turns away from the rover's
               heading -- "forward, then turn" shows here as a small number

WHY
    NAV_PLAN.md N0/N5: the planner is the half of navigation that can be
    tested with the rover parked. A path whose outline touches a wall is a
    failure before a wheel turns; this finds it without driving.
"""
import argparse
import csv
import math
import sys
import time

import numpy as np
import rclpy
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros

# the tape-measured outer box (nav2.yaml footprint; description/params.yaml)
FRONT, REAR, SIDE = 0.182, -0.178, 0.190
FRAME = 'odom'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def outline_pts(step=0.01):
    """Points along the outline, in base_link."""
    xs = np.arange(REAR, FRONT + 1e-9, step)
    ys = np.arange(-SIDE, SIDE + 1e-9, step)
    return np.vstack([np.c_[xs, np.full_like(xs, SIDE)], np.c_[xs, np.full_like(xs, -SIDE)],
                      np.c_[np.full_like(ys, FRONT), ys], np.c_[np.full_like(ys, REAR), ys]])


class Check(Node):
    def __init__(self):
        super().__init__('plan_check')
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        self.grid = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._grid, q)
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.ac = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

    def _grid(self, m):
        self.grid = m

    def wait(self, pred, t=15.0):
        t0 = time.time()
        while not pred() and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        return pred()

    def pose(self):
        tr = self.tf.lookup_transform(FRAME, 'base_link', rclpy.time.Time()).transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def lethal_xy(self):
        g = self.grid
        a = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        ys, xs = np.nonzero(a >= 100)            # 100 = LETHAL (254) in the OccupancyGrid
        r = g.info.resolution
        return np.c_[g.info.origin.position.x + (xs + 0.5) * r,
                     g.info.origin.position.y + (ys + 0.5) * r], r

    def snap(self, x, y, within=0.35, max_cost=50):
        """The free cell (cost < max_cost, known) nearest (x, y), within `within`
        m -- a ring point inside a sofa is not a planner test. None if none."""
        g = self.grid
        a = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        r = g.info.resolution
        i0 = int((x - g.info.origin.position.x) / r)
        j0 = int((y - g.info.origin.position.y) / r)
        k = int(within / r)
        best = None
        for j in range(max(0, j0 - k), min(g.info.height, j0 + k + 1)):
            for i in range(max(0, i0 - k), min(g.info.width, i0 + k + 1)):
                v = a[j, i]
                if 0 <= v < max_cost:
                    d = math.hypot(i - i0, j - j0) * r
                    if d <= within and (best is None or d < best[0]):
                        best = (d, g.info.origin.position.x + (i + 0.5) * r,
                                g.info.origin.position.y + (j + 0.5) * r)
        return None if best is None else best[1:]

    def plan(self, x, y, th):
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = FRAME
        goal.goal.pose.position.x, goal.goal.pose.position.y = x, y
        goal.goal.pose.orientation.z, goal.goal.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        goal.use_start = False
        t0 = time.time()
        f = self.ac.send_goal_async(goal)
        if not self.wait(lambda: f.done(), 10) or not f.result().accepted:
            return None, (time.time() - t0) * 1000, 'rejected'
        rf = f.result().get_result_async()
        if not self.wait(lambda: rf.done(), 15):
            return None, (time.time() - t0) * 1000, 'timeout'
        res = rf.result().result
        ms = (time.time() - t0) * 1000
        ec = getattr(res, 'error_code', 0)
        if not res.path.poses:
            return None, ms, f'error {ec}'
        return [(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation))
                for p in res.path.poses], ms, ''


def clearance(path, lethal, res):
    """min over path poses of (distance from the outline to the nearest lethal
    cell centre, minus half a cell); negative if a lethal cell is inside."""
    if len(lethal) == 0:
        return float('inf')
    ol = outline_pts()
    best = float('inf')
    for x, y, th in path[::2]:
        c, s = math.cos(th), math.sin(th)
        near = lethal[np.hypot(lethal[:, 0] - x, lethal[:, 1] - y) < 0.8]
        if len(near) == 0:
            continue
        # into the body frame
        dx, dy = near[:, 0] - x, near[:, 1] - y
        bx, by = c * dx + s * dy, -s * dx + c * dy
        inside = (bx > REAR) & (bx < FRONT) & (np.abs(by) < SIDE)
        if inside.any():
            return -res
        d = np.min(np.hypot(bx[:, None] - ol[None, :, 0], by[:, None] - ol[None, :, 1]))
        best = min(best, d - res / 2)
    return best


def first_turn(path, th0):
    """Heading change over the path's first 20 cm of travel, degrees."""
    d = 0.0
    for (x0, y0, _), (x1, y1, t1) in zip(path, path[1:]):
        d += math.hypot(x1 - x0, y1 - y0)
        if d >= 0.20:
            return math.degrees(abs(math.atan2(math.sin(t1 - th0), math.cos(t1 - th0))))
    return math.degrees(abs(math.atan2(math.sin(path[-1][2] - th0), math.cos(path[-1][2] - th0))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--r', default='1.0,2.0', help='radii, m')
    ap.add_argument('--n', type=int, default=8, help='directions per radius')
    ap.add_argument('--csv', default='')
    a = ap.parse_args()
    rclpy.init()
    n = Check()
    if not n.ac.wait_for_server(timeout_sec=10):
        sys.exit('no /compute_path_to_pose -- is nav2 up? (./rover nav)')
    if not n.wait(lambda: n.grid is not None) or not n.wait(lambda: n.tf.can_transform(FRAME, 'base_link', rclpy.time.Time())):
        sys.exit('no global costmap or no odom->base_link')
    x0, y0, th0 = n.pose()
    lethal, res = n.lethal_xy()
    print(f'rover at ({x0:.2f}, {y0:.2f}) {math.degrees(th0):.0f} deg; '
          f'{len(lethal)} lethal cells in the global costmap')
    print(f"{'goal':>14} {'ok':>3} {'ms':>6} {'len':>5} {'clear_cm':>8} {'first_deg':>9}  note")
    rows = []
    for r in [float(v) for v in a.r.split(',')]:
        for k in range(a.n):
            rel = 2 * math.pi * k / a.n
            th = th0 + rel
            label = f'{r:.1f}m @{math.degrees(rel):+4.0f}'
            snapped = n.snap(x0 + r * math.cos(th), y0 + r * math.sin(th))
            if snapped is None:
                print(f'{label:>14} {"-":>3} {"":>6} {"":>5} {"":>8} {"":>9}  goal in obstacle (no free cell within 35 cm) -- skipped')
                rows.append(dict(goal=label, ok=-1, ms='', len='', clear_cm='', first_deg='', note='goal in obstacle'))
                continue
            gx, gy = snapped
            path, ms, why = n.plan(gx, gy, th)
            if path is None:
                print(f'{label:>14} {"no":>3} {ms:6.0f} {"":>5} {"":>8} {"":>9}  {why}')
                rows.append(dict(goal=label, ok=0, ms=round(ms), len='', clear_cm='', first_deg='', note=why))
                continue
            L = sum(math.hypot(b[0] - a_[0], b[1] - a_[1]) for a_, b in zip(path, path[1:]))
            cl = clearance(path, lethal, res)
            ft = first_turn(path, th0)
            # cl < 0 only when a lethal cell CENTRE is inside the body; a
            # cell whose edge meets the outline reads ~0 and is only touching
            note = 'BODY OVERLAPS LETHAL' if cl < -0.005 else ('touching' if cl < 0.005 else '')
            cl = max(cl, 0.0) if cl >= -0.005 else cl
            print(f'{label:>14} {"yes":>3} {ms:6.0f} {L:5.2f} {cl * 100:8.1f} {ft:9.0f}  {note}')
            rows.append(dict(goal=label, ok=1, ms=round(ms), len=round(L, 2),
                             clear_cm=round(cl * 100, 1), first_deg=round(ft), note=note))
    tested = [r for r in rows if r['ok'] >= 0]
    ok = [r for r in tested if r['ok'] == 1]
    bad = [r for r in ok if r['clear_cm'] < 0]
    ms = [r['ms'] for r in tested] or [0]
    print(f'\n{len(ok)}/{len(tested)} reachable-looking goals planned ({len(rows) - len(tested)} skipped: in obstacle); '
          f'{len(bad)} with the body overlapping lethal; '
          f'median {np.median(ms):.0f} ms, worst {max(ms):.0f} ms')
    if a.csv:
        with open(a.csv, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['t', 'x0', 'y0', 'th0'] + list(rows[0]))
            if f.tell() == 0:
                w.writeheader()
            for r in rows:
                w.writerow(dict(t=round(time.time()), x0=round(x0, 2), y0=round(y0, 2),
                                th0=round(math.degrees(th0)), **r))
    rclpy.shutdown()
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
