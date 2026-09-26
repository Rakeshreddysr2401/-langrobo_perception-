#!/usr/bin/env python3
"""gap_pass.py — find the gap ahead and crawl straight through it (./rover pass).

    ./rover pass [D] [--margin 0.03] [--dry]
        D        how far ahead (m, along the gap's line) to finish; default 1.3
        --dry    measure and print only; nothing moves

nav2 cannot thread a gap within a few cm of the rover's width: its map is in
cells, and a cell an edge touches is solid (LOCALIZATION.md §13). This uses
the raw points instead -- the LiDAR plus the depth camera at the rover's
height, 1 cm (depth_obstacles.py) -- and goal_exec's PASS MODE to drive it.

HOW THE GAP IS FOUND
    Every straight line ahead is tried: heading within ±35 deg of the rover's,
    in 1 deg steps. Along each, the corridor from 0.2 m behind the rover to D
    ahead (plus the body) is searched for the widest strip no point falls in;
    its middle is the line's offset. The line with the widest strip wins
    (within 1 cm of the widest, the one needing the least turn). That strip's
    width is the gap: its narrowest point along the whole pass.

    It passes if  width >= 2 x (half-width 0.19 + margin): 44 cm at the
    owner's 3 cm. Otherwise it refuses and says the measured width.

THEN
    goal_exec gets the end point (D ahead on the line, facing along it) with
    the pre-goal 0.2 m behind the rover's position on the line: it lines up
    (turn, straight, turn, 5 cm margin), then crawls the line at 5 cm/s,
    checking the corridor at the pass margin and stopping on any contact.
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
from depth_obstacles import DepthObstacles  # noqa: E402
from goal_exec import FRONT, REAR, SIDE  # noqa: E402

BACK = 0.20            # m, the pre-goal: this far behind the rover's spot on the line
VX = 0.05              # m/s through the gap
HALF_VIEW = 0.60       # m, a strip must lie within this of the line's origin
NEAR = 0.25            # m, the gap's line must pass this close to the rover


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def find_gap(pts, D, margin):
    """pts: N x 2 base_link. The best line through a GAP -- a strip with
    obstacles on BOTH sides (open floor is not a gap and would always be
    widest). Returns (best, candidates), candidates sorted widest first."""
    cands = []
    for deg in range(-35, 36):
        phi = math.radians(deg)
        u = np.array([math.cos(phi), math.sin(phi)])
        al = pts @ u
        cr = pts[:, 1] * u[0] - pts[:, 0] * u[1]
        sel = (al > -BACK - REAR - margin) & (al < D + FRONT + margin) & (np.abs(cr) < HALF_VIEW + 0.4)
        ys = np.sort(cr[sel])
        if len(ys) < 2:
            continue
        w = np.diff(ys)
        mid = (ys[:-1] + ys[1:]) / 2
        ok = np.abs(mid) <= HALF_VIEW
        if not ok.any():
            continue
        k = int(np.argmax(np.where(ok, w, -1)))
        cands.append(dict(phi=phi, deg=deg, c=float(mid[k]), width=float(w[k]),
                          lo=float(ys[k]), hi=float(ys[k + 1])))
    if not cands:
        return None, []
    # the gap the rover is FACING: the line must pass within NEAR of the rover
    # (it lines up with a short turn-straight-turn, not a trip across the
    # room). Among those, the widest; within 1 cm of it, the least turn.
    near = [c for c in cands if abs(c['c']) <= NEAR]
    if not near:
        return None, sorted(cands, key=lambda c: -c['width'])
    top = max(c['width'] for c in near)
    best = min((c for c in near if c['width'] >= top - 0.01), key=lambda c: abs(c['deg']))
    return best, sorted(near, key=lambda c: -c['width'])


class Pass(Node):
    def __init__(self):
        super().__init__('gap_pass')
        self.pose, self.scan_pts, self.status = None, None, []
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(String, '/goal_exec/status', lambda m: self.status.append(json.loads(m.data)), 10)
        self.pub = self.create_publisher(String, '/goal_exec/pass', 10)
        self.cancel = self.create_publisher(Empty, '/goal_exec/cancel', 10)
        self.dobs = DepthObstacles(self, self.buf, lambda: self.pose)
        self.mount = None
        self.scans = []

    def _odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _scan(self, m):
        if self.mount is None:
            if not self.buf.can_transform('base_link', m.header.frame_id, Time()):
                return
            tr = self.buf.lookup_transform('base_link', m.header.frame_id, Time()).transform
            self.mount = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
        r = np.asarray(m.ranges)
        a = m.angle_min + np.arange(r.size) * m.angle_increment
        ok = np.isfinite(r) & (r > 0.05) & (r < 8.0)
        x, y, yaw = self.mount
        p = np.stack([r[ok] * np.cos(a[ok] + yaw) + x, r[ok] * np.sin(a[ok] + yaw) + y], 1)
        own = (p[:, 0] < 0.202) & (p[:, 0] > -0.198) & (np.abs(p[:, 1]) < 0.21)
        self.scans.append(p[~own])
        self.scans = self.scans[-10:]

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    margin = float(sys.argv[sys.argv.index('--margin') + 1]) if '--margin' in sys.argv else 0.03
    if '--margin' in sys.argv:
        args = [a for a in args if a != sys.argv[sys.argv.index('--margin') + 1]]
    D = float(args[0]) if args else 1.3
    dry = '--dry' in sys.argv
    rclpy.init()
    n = Pass()
    # the camera's mount transform takes ~2 s to arrive; then 8 depth updates
    end = time.time() + 12
    while time.time() < end and (n.dobs.frames < 8 or len(n.scans) < 10):
        n.spin_for(0.2)
    if n.pose is None or not n.scans:
        sys.exit('  need /odom and /scan: ./rover fused, ./rover lidar')
    if n.dobs.frames < 8:
        sys.exit(f'  depth camera: only {n.dobs.frames} frames in 12 s -- is it streaming?')
    pts = np.vstack(n.scans + [n.dobs.points_base(n.pose)])
    g, cands = find_gap(pts, D, margin)
    need = 2 * (SIDE + margin)
    print(f'  points: LiDAR {sum(len(s) for s in n.scans)} over {len(n.scans)} scans, depth {len(n.dobs.points_base(n.pose))} (1 cm, rover height)')
    if g is None:
        sys.exit(f'  no gap ahead: no line bounded on both sides passes within {NEAR * 100:.0f} cm of the rover')
    if dry:
        # top view, base_link: nose up, 4 cm cells; L = LiDAR, d = depth, R = rover
        dp = n.dobs.points_base(n.pose)
        lp = np.vstack(n.scans)
        cell = lambda q: set(map(tuple, np.round(q / 0.04).astype(int)))
        Lc, Dc = cell(lp), cell(dp)
        print('  top view (nose up, 4 cm cells): L LiDAR, d depth, B both, R rover, * best line')
        u = np.array([1.0, 0.0])
        for ix in range(int((D + 0.3) / 0.04), -int(0.3 / 0.04) - 1, -1):
            row = ''
            for iy in range(20, -21, -1):
                c = (ix, iy)
                if abs(ix * .04) <= FRONT and abs(iy * .04) <= SIDE:
                    row += 'R'
                elif c in Lc and c in Dc:
                    row += 'B'
                elif c in Lc:
                    row += 'L'
                elif c in Dc:
                    row += 'd'
                else:
                    row += '.'
            print('   ' + row)
    shown = []
    for cd in cands:
        if all(abs(cd['deg'] - s['deg']) > 4 for s in shown):
            shown.append(cd)
    print('  candidate lines (widest strip bounded on both sides):')
    for cd in shown[:4]:
        print(f'    {cd["deg"]:+3d} deg  offset {cd["c"] * 100:+5.1f} cm  width {cd["width"] * 100:5.1f} cm')
    print(f'  best line: {g["deg"]:+d} deg from the rover\'s heading, offset {g["c"] * 100:+.1f} cm')
    print(f'  narrowest along it (from {BACK:.1f} m behind to {D:.2f} m ahead): '
          f'{g["width"] * 100:.1f} cm   [{g["lo"] * 100:+.1f} .. {g["hi"] * 100:+.1f} cm]'
          )
    print(f'  needs {need * 100:.0f} cm (38 cm rover + 2 x {margin * 100:.0f} cm); lane for the centre '
          f'{(g["width"] - 2 * SIDE - 2 * margin) * 100:+.1f} cm')
    if g['width'] < need:
        print('  => REFUSED: too tight')
        return 1
    # the line in odom
    x, y, th = n.pose
    phi = th + g['phi']
    u = np.array([math.cos(phi), math.sin(phi)])
    nrm = np.array([-u[1], u[0]])
    base_u = np.array([math.cos(g['phi']), math.sin(g['phi'])])
    base_n = np.array([-base_u[1], base_u[0]])
    origin_b = g['c'] * base_n                    # line point nearest the rover, base_link
    c, s = math.cos(th), math.sin(th)
    origin = np.array([x + c * origin_b[0] - s * origin_b[1], y + s * origin_b[0] + c * origin_b[1]])
    end = origin + D * u
    L = D + BACK
    print(f'  end (odom): ({end[0]:+.3f}, {end[1]:+.3f}, {math.degrees(phi):+.1f} deg); '
          f'straight crawl {L:.2f} m at {VX * 100:.0f} cm/s')
    if dry:
        print('  --dry: nothing moves')
        return 0
    print('  THIS DRIVES THE ROVER. Ctrl-C cancels.')
    req = dict(x=float(end[0]), y=float(end[1]), th=float(phi), frame='odom', L=L, margin=margin, vx=VX)
    t0 = time.time()
    n.pub.publish(String(data=json.dumps(req)))
    seen = 0
    try:
        while time.time() - t0 < 240:
            n.spin_for(0.1)
            for st in n.status[seen:]:
                line = f'  {time.time() - t0:5.1f}s  {st["state"]:6s} try {st.get("tries")}  err {st.get("err_cm")} cm'
                if st.get('contacts'):
                    line += f'  contacts {st["contacts"]}'
                print(line)
                if st['state'] == 'done':
                    print(f'  => {st["result"]}: {st["why"]}')
                    return 0 if st['result'] == 'reached' else 1
            seen = len(n.status)
        n.cancel.publish(Empty())
        print('  => gave up after 240 s; cancelled')
        return 1
    except KeyboardInterrupt:
        n.cancel.publish(Empty())
        n.spin_for(0.3)
        print('  => cancelled')
        return 1


if __name__ == '__main__':
    sys.exit(main())
