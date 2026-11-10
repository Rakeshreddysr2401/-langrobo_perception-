#!/usr/bin/env python3
"""nav_course.py — a fixed course of goals, driven and graded (NAV_GUIDE.md F1).

    python3 nav_course.py record [--course /logs/nav_course.json]
        MOVES NOTHING. Click points in RViz with "Publish Point" (/clicked_point),
        in driving order. Each is saved relative to where the rover stands now,
        so a later run from the SAME tape mark, facing the same way, drives the
        same course. Ctrl-C saves.

    python3 nav_course.py run --label k2 [--course ...] [--only 3]
        DRIVES. Each point through /reach/goal (what RViz's 2D Goal Pose and
        the brain use), facing the way it travels, then back to the start.
        One CSV row per goal: /logs/f1_course_<label>_<time>.csv

WHAT A ROW SAYS
    result, secs           reach's own verdict and how long it took
    err_cm, yaw_err_deg    where it stopped vs the goal (fused pose)
    attempts, phases       reach attempts and every recovery phase it ran
                           (tight, way_back, pass, wait, escape, ...)
    nav2_fails             reach "recover" lines: nav2 or goal_exec failed
    clear_min_cm           closest LiDAR point to the REAL outline (no padding)
                           while driving; negative = inside it
    v_mean, v_max          m/s over the goal, from /odom
    path_m                 distance actually driven
    cm_slow_s              seconds the collision monitor was acting
    load1                  1-minute load average at the end (the Jetson CPU)

Ctrl-C cancels the goal (reach stops the wheels) and keeps the rows so far.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
import tf2_ros

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:          # older nav2: no state message, column stays 0
    CollisionMonitorState = None

FRONT, REAR, SIDE = 0.182, -0.178, 0.190     # nav2.yaml footprint, unpadded
FRAME = 'odom'
GOAL_TIMEOUT = 330.0                        # reach's MAX_S is 300


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rect_dist(x, y):
    """Signed distance from points (base_link) to the body rectangle."""
    dx = np.maximum(np.maximum(REAR - x, x - FRONT), 0.0)
    dy = np.maximum(np.abs(y) - SIDE, 0.0)
    out = np.hypot(dx, dy)
    inside = (x >= REAR) & (x <= FRONT) & (np.abs(y) <= SIDE)
    depth = np.minimum.reduce([x - REAR, FRONT - x, SIDE - np.abs(y)])
    return np.where(inside, -depth, out)


class Course(Node):
    def __init__(self):
        super().__init__('nav_course')
        self.odom = None
        self.status = []
        self.laser = None            # (x, y, yaw) of the laser in base_link
        self.clear_min = math.inf
        self.cm_active = False
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(String, '/reach/status', self._status, 50)
        self.create_subscription(LaserScan, '/scan', self._scan, 10)
        if CollisionMonitorState is not None:
            self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                     self._cm, 10)
        self.pub = self.create_publisher(PoseStamped, '/reach/goal', 10)
        self.cancel = self.create_publisher(Empty, '/reach/cancel', 10)

    def _odom(self, m):
        self.odom = m

    def _status(self, m):
        try:
            self.status.append(json.loads(m.data))
        except ValueError:
            pass

    def _cm(self, m):
        self.cm_active = m.action_type != 0      # 0 = DO_NOTHING

    def _scan(self, m):
        if self.laser is None:
            try:
                t = self.tf.lookup_transform('base_link', m.header.frame_id, Time())
            except Exception:
                return
            tr = t.transform
            self.laser = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(r.size) * m.angle_increment
        ok = np.isfinite(r) & (r > max(m.range_min, 0.05))
        if not ok.any():
            return
        lx, ly, lth = self.laser
        x = lx + r[ok] * np.cos(a[ok] + lth)
        y = ly + r[ok] * np.sin(a[ok] + lth)
        d = rect_dist(x, y)
        # one stray return is not a wall: the 3rd smallest, like collision_monitor's min_points
        self.clear_min = min(self.clear_min, float(np.sort(d)[min(2, d.size - 1)]))

    def pose(self):
        p = self.odom.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def wait_for(self, what, secs=10.0):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.1)
            if what():
                return True
        return False


def to_start(start, x, y):
    """odom point -> the start pose's frame"""
    sx, sy, sth = start
    dx, dy = x - sx, y - sy
    return (math.cos(-sth) * dx - math.sin(-sth) * dy,
            math.sin(-sth) * dx + math.cos(-sth) * dy)


def from_start(start, x, y):
    sx, sy, sth = start
    return (sx + math.cos(sth) * x - math.sin(sth) * y,
            sy + math.sin(sth) * x + math.cos(sth) * y)


def record(n, path):
    if not n.wait_for(lambda: n.odom is not None):
        sys.exit('no /odom -- is ./rover fused up?')
    start = n.pose()
    pts = []

    def clicked(m):
        try:
            t = n.tf.lookup_transform(FRAME, m.header.frame_id, Time())
        except Exception as e:
            print(f'  ignored: no transform {m.header.frame_id} -> {FRAME} ({e})')
            return
        q, tr = t.transform.rotation, t.transform.translation
        th = yaw_of(q)
        x = tr.x + math.cos(th) * m.point.x - math.sin(th) * m.point.y
        y = tr.y + math.sin(th) * m.point.x + math.cos(th) * m.point.y
        rx, ry = to_start(start, x, y)
        pts.append([round(rx, 3), round(ry, 3)])
        print(f'  point {len(pts)}: {rx:+.2f} ahead, {ry:+.2f} left of the start')

    n.create_subscription(PointStamped, '/clicked_point', clicked, 10)
    print('RViz: "Publish Point" tool, click the course in driving order. Ctrl-C saves.')
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    if not pts:
        sys.exit('no points -- nothing saved')
    with open(path, 'w') as f:
        json.dump({'recorded': time.strftime('%Y-%m-%d %H:%M'), 'points': pts}, f, indent=1)
    print(f'saved {len(pts)} points to {path}. Mark the start on the floor (tape at the '
          f'wheels), and start every run there, facing the same way.')


def run(n, path, label, only):
    with open(path) as f:
        pts = json.load(f)['points']
    if not n.wait_for(lambda: n.odom is not None):
        sys.exit('no /odom -- is ./rover fused up?')
    start = n.pose()
    goals, prev = [], (0.0, 0.0)
    for x, y in pts:
        goals.append((x, y, math.atan2(y - prev[1], x - prev[0])))
        prev = (x, y)
    goals.append((0.0, 0.0, 0.0))                      # home, facing as it started
    if only:
        goals = [goals[only - 1]]
    out = f'/logs/f1_course_{label}_{time.strftime("%Y%m%d-%H%M")}.csv'
    cols = ['label', 'goal', 'gx', 'gy', 'gdeg', 'result', 'secs', 'err_cm', 'yaw_err_deg',
            'attempts', 'phases', 'nav2_fails', 'clear_min_cm', 'v_mean', 'v_max', 'path_m',
            'cm_slow_s', 'load1', 'why']
    rows = []
    try:
        for i, (gx, gy, gth) in enumerate(goals, 1):
            ox, oy = from_start(start, gx, gy)
            oth = wrap(start[2] + gth)
            p = PoseStamped()
            p.header.frame_id = FRAME
            p.header.stamp = n.get_clock().now().to_msg()
            stamp = f'{p.header.stamp.sec}.{p.header.stamp.nanosec}'
            p.pose.position.x, p.pose.position.y = ox, oy
            p.pose.orientation.z, p.pose.orientation.w = math.sin(oth / 2), math.cos(oth / 2)
            n.status.clear()
            n.clear_min, cm_s, vs, path_m = math.inf, 0.0, [], 0.0
            last = n.pose()
            name = 'home' if i == len(goals) and not only else str(i)
            print(f'goal {name}: {gx:+.2f} ahead, {gy:+.2f} left, {math.degrees(gth):+.0f} deg')
            n.pub.publish(p)
            t0 = tick = time.time()
            final = None
            while time.time() - t0 < GOAL_TIMEOUT:
                rclpy.spin_once(n, timeout_sec=0.05)
                now = time.time()
                if now - tick >= 0.1:
                    v = n.odom.twist.twist.linear
                    vs.append(math.hypot(v.x, v.y))
                    if n.cm_active:
                        cm_s += now - tick
                    cur = n.pose()
                    path_m += math.hypot(cur[0] - last[0], cur[1] - last[1])
                    last, tick = cur, now
                mine = [s for s in n.status if s.get('goal_stamp') == stamp]
                final = next((s for s in mine if 'result' in s), None)
                if final:
                    break
            secs = time.time() - t0
            if not final:
                n.cancel.publish(Empty())
                final = {'result': 'timeout'}
            mine = [s for s in n.status if s.get('goal_stamp') == stamp]
            x, y, th = n.pose()
            phases = [s['phase'] for s in mine if 'phase' in s and s['phase'] not in ('nav2', 'finish', 'goal_exec')]
            row = dict(label=label, goal=name, gx=gx, gy=gy, gdeg=round(math.degrees(gth)),
                       result=final.get('result'), secs=round(secs, 1),
                       err_cm=round(math.hypot(x - ox, y - oy) * 100, 1),
                       yaw_err_deg=round(math.degrees(wrap(th - oth)), 1),
                       attempts=max([s.get('attempt', 0) for s in mine] or [0]),
                       phases='+'.join(phases),
                       nav2_fails=sum(1 for s in mine if s.get('phase') == 'recover'),
                       clear_min_cm=round(n.clear_min * 100, 1) if n.clear_min < math.inf else '',
                       v_mean=round(float(np.mean(vs)), 3) if vs else 0,
                       v_max=round(float(np.max(vs)), 3) if vs else 0,
                       path_m=round(path_m, 2), cm_slow_s=round(cm_s, 1),
                       load1=round(os.getloadavg()[0], 1), why=final.get('why', ''))
            rows.append(row)
            print('   ' + ', '.join(f'{k} {row[k]}' for k in
                                    ('result', 'secs', 'err_cm', 'attempts', 'phases',
                                     'clear_min_cm', 'v_mean', 'v_max', 'load1')))
            time.sleep(1.0)
    except KeyboardInterrupt:
        n.cancel.publish(Empty())
        print('\ncancelled -- rows so far kept')
    if rows:
        with open(out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)
        ok = [r for r in rows if r['result'] == 'reached']
        print(f'\n{len(ok)}/{len(rows)} reached; total {sum(r["secs"] for r in rows):.0f} s; '
              f'recovery phases {sum(len(r["phases"].split("+")) for r in rows if r["phases"])}; '
              f'min clearance {min((r["clear_min_cm"] for r in rows if r["clear_min_cm"] != ""), default="-")} cm'
              f'\n-> {out}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['record', 'run'])
    ap.add_argument('--course', default='/logs/nav_course.json')
    ap.add_argument('--label', default='baseline')
    ap.add_argument('--only', type=int, default=0, help='drive just goal N (1-based)')
    a = ap.parse_args()
    rclpy.init()
    n = Course()
    try:
        record(n, a.course) if a.mode == 'record' else run(n, a.course, a.label, a.only)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
