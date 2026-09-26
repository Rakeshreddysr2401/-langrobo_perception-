#!/usr/bin/env python3
"""gap_pass.py — find the gap ahead and crawl straight through it (./rover pass).

    ./rover pass [D] [--margin 0.03] [--dry] [--look DEG | --no-look]
        D        how far ahead (m, along the gap's line) to finish; default 1.3
        --dry    measure and print only; nothing moves (no look either)
        --look   before measuring, turn DEG left, back, DEG right, back
                 (default 25): the camera sees 87 deg and nothing near its
                 nose, so a look fills in the gap's sides and low things

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
from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA, Empty, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
from depth_obstacles import DepthObstacles  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from goal_exec import FRONT, REAR, SIDE, GoalExec, wrap  # noqa: E402

BACK = 0.20            # m, the pre-goal: this far behind the rover's spot on the line
VX = 0.05              # m/s through the gap
RETRIES = 2            # re-measure after 'blocked ahead' at most this often
LOOK_DEG = 25.0
PIVOT_FILE = Path('/logs/goal_exec_pivot.json')
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


def corridor_markers(stamp, pose, g, D, margin, ok):
    """RViz: the strip found (its edges), the rover's swept outline + margin
    along the line, the line, and the width -- green if it fits, red if not."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    to_odom = lambda bx, by: Point(x=x + c * bx - s * by, y=y + s * bx + c * by, z=0.02)
    u = np.array([math.cos(g['phi']), math.sin(g['phi'])])
    nv = np.array([-u[1], u[0]])
    a0, a1 = -BACK - REAR, D + FRONT
    def seg(al0, al1, cr):
        return [to_odom(*(al0 * u + cr * nv)), to_odom(*(al1 * u + cr * nv))]
    col = ColorRGBA(r=0.1, g=0.85, b=0.2, a=1.0) if ok else ColorRGBA(r=0.95, g=0.15, b=0.1, a=1.0)
    out = []
    def mk(i, typ, scale, color, pts=None, text=None, at=None):
        m = Marker()
        m.header.frame_id, m.header.stamp = 'odom', stamp
        m.ns, m.id, m.type, m.action = 'gap_pass', i, typ, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = color
        m.pose.orientation.w = 1.0
        if pts:
            m.points = pts
        if text:
            m.text = text
            m.pose.position = at
        out.append(m)
    edges = seg(a0, a1, g['lo']) + seg(a0, a1, g['hi'])
    mk(0, Marker.LINE_LIST, 0.012, col, edges)                            # the strip's edges
    half = SIDE + margin
    sweep = seg(a0, a1, g['c'] - half) + seg(a0, a1, g['c'] + half)
    mk(1, Marker.LINE_LIST, 0.006, ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8), sweep)   # rover + margin
    mk(2, Marker.LINE_LIST, 0.008, ColorRGBA(r=1.0, g=0.85, b=0.0, a=1.0), seg(-BACK, D, g['c']))  # the line
    mk(3, Marker.TEXT_VIEW_FACING, 0.08, col,
       text=f"{g['width'] * 100:.0f} cm {'PASS' if ok else 'TOO TIGHT'} (needs {2 * half * 100:.0f})",
       at=to_odom(*((D * 0.5) * u + (g['c'] + 0.35) * nv)))
    ma = MarkerArray()
    ma.markers = out
    return ma


class Pass(Node):
    def __init__(self, name='gap_pass', depth_active=True):
        super().__init__(name)
        self.pose, self.scan_pts, self.status = None, None, []
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(String, '/goal_exec/status', lambda m: self.status.append(json.loads(m.data)), 10)
        self.pub = self.create_publisher(String, '/goal_exec/pass', 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cancel = self.create_publisher(Empty, '/goal_exec/cancel', 10)
        # latched, so RViz shows the last measurement whenever it looks
        self.pub_mk = self.create_publisher(MarkerArray, '/gap_pass/markers', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.dobs = DepthObstacles(self, self.buf, lambda: self.pose, active=depth_active)
        self.mount = None
        self.scans = []
        self.pose_t = self.scan_t = 0.0

    def _odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.pose_t = time.time()

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
        self.scan_t = time.time()
        self.scans = self.scans[-10:]

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def _ex():
    try:
        prior = json.loads(PIVOT_FILE.read_text())
    except (OSError, ValueError):
        prior = None
    return GoalExec(prior)


def turn_to(n, heading, ex=None, tol_deg=3.0):
    """Turn in place to `heading` (odom) on the fused heading; the swing is
    checked first against the outline swept about the learned pivot (5 cm).
    Returns '' or why it did not turn."""
    ex = ex or _ex()
    e = wrap(heading - n.pose[2])
    pts = np.vstack(n.scans[-3:] + [n.dobs.points_base(n.pose)])
    ok, why = ex.turn_clear(pts, e)
    if not ok:
        return why
    t0, last, blocked, why = time.time(), None, 0, ''
    while time.time() - t0 < 8.0:
        n.spin_for(0.05)
        # never turn on a frozen view: the pose drives the stop, the scan the check
        if time.time() - n.pose_t > 0.3 or time.time() - n.scan_t > 0.5:
            why = 'pose or scan stale: stopped'
            break
        e = wrap(heading - n.pose[2])
        ok, w_ = ex.turn_clear(np.vstack(n.scans[-1:] + [n.dobs.points_base(n.pose)]), e)
        blocked = 0 if ok else blocked + 1
        if blocked >= ex.BLOCK_STEPS:
            why = 'blocked mid-turn: ' + w_
            break
        w_meas = 0.0 if last is None else wrap(n.pose[2] - last[1]) / max(time.time() - last[0], 1e-3)
        last = (time.time(), n.pose[2])
        lead = abs(w_meas) * ex.STOP_LEAD if w_meas * e > 0 else 0.0
        if abs(e) <= max(math.radians(tol_deg), lead):
            break
        tw = Twist()
        tw.angular.z = math.copysign(min(ex.WZ_MAX, max(ex.WZ_MIN, ex.K_TH * abs(e))), e)
        n.cmd.publish(tw)
    for _ in range(3):
        n.cmd.publish(Twist())
    n.spin_for(0.6)                               # let it settle; the camera catches up
    return why


def look_to(n, heading):
    why = turn_to(n, heading)
    if why:
        print(f'  face: skipped ({why})')


def look(n, deg):
    """Turn deg left, back, deg right, back. The depth memory fills in as it
    turns (it is in odom)."""
    ex = _ex()
    th0 = n.pose[2]
    for target in (deg, 0.0, -deg, 0.0):
        why = turn_to(n, wrap(th0 + math.radians(target)), ex)
        if why:
            print(f'  look: skipped the turn to {target:+.0f} deg ({why})')
    print(f'  look: ±{deg:.0f} deg done, heading {math.degrees(wrap(n.pose[2] - th0)):+.1f} deg from the start, '
          f'depth memory {len(n.dobs.points_base(n.pose))} points')


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    margin = float(sys.argv[sys.argv.index('--margin') + 1]) if '--margin' in sys.argv else 0.03
    if '--margin' in sys.argv:
        args = [a for a in args if a != sys.argv[sys.argv.index('--margin') + 1]]
    D = float(args[0]) if args else 1.3
    dry = '--dry' in sys.argv
    look_deg = float(sys.argv[sys.argv.index('--look') + 1]) if '--look' in sys.argv else LOOK_DEG
    if '--look' in sys.argv:
        args = [a for a in args if a != sys.argv[sys.argv.index('--look') + 1]]
        D = float(args[0]) if args else 1.3
    do_look = not dry and '--no-look' not in sys.argv
    rclpy.init()
    n = Pass()
    # the camera's mount transform takes ~2 s to arrive; then 8 depth updates
    end = time.time() + 20
    while time.time() < end and (n.dobs.frames < 8 or len(n.scans) < 10):
        n.spin_for(0.2)
    if n.pose is None or not n.scans:
        sys.exit('  need /odom and /scan: ./rover fused, ./rover lidar')
    if n.dobs.frames < 8:
        sys.exit(f'  depth camera: only {n.dobs.frames} frames in 20 s -- is it streaming?')
    if do_look:
        look(n, look_deg)
    # a pass stopped because the camera saw something new close up (a stool's
    # feet, under the band it could see from further back) is measured again
    # from where it stopped, toward the same end point: at most RETRIES times
    out, end = attempt(n, D, margin, dry)
    for k in range(RETRIES):
        if out != 'blocked':
            break
        x, y, th = n.pose
        D = max(0.4, float((end - np.array([x, y])) @ np.array([math.cos(th), math.sin(th)])))
        print(f'\n  re-measuring from here (retry {k + 1}/{RETRIES}), {D:.2f} m still to go')
        n.spin_for(1.0)
        if do_look:
            look(n, look_deg)
        out, end = attempt(n, D, margin, dry)
    return 0 if out in ('reached', 'dry') else 1


def attempt(n, D, margin, dry):
    """Measure, and unless dry or refused, send one pass and follow it.
    Returns (outcome, end_point_odom)."""
    pts = np.vstack(n.scans + [n.dobs.points_base(n.pose)])
    g, cands = find_gap(pts, D, margin)
    need = 2 * (SIDE + margin)
    print(f'  points: LiDAR {sum(len(s) for s in n.scans)} over {len(n.scans)} scans, depth {len(n.dobs.points_base(n.pose))} (1 cm, rover height)')
    if g is None:
        print(f'  no gap ahead: no line bounded on both sides passes within {NEAR * 100:.0f} cm of the rover')
        return 'none', None
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
    n.pub_mk.publish(corridor_markers(n.get_clock().now().to_msg(), n.pose, g, D, margin, g['width'] >= need))
    if g['width'] < need:
        print('  => REFUSED: too tight')
        return 'tight', None
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
        return 'dry', end
    print('  THIS DRIVES THE ROVER. Ctrl-C cancels.')
    req = dict(x=float(end[0]), y=float(end[1]), th=float(phi), frame='odom', L=L, margin=margin, vx=VX)
    t0 = time.time()
    n.status.clear()                              # not a previous attempt's 'done'
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
                    if st['result'] == 'reached':
                        return 'reached', end
                    if st['result'] == 'refused' and st['why'].startswith('blocked ahead'):
                        return 'blocked', end
                    return 'failed', end
            seen = len(n.status)
        n.cancel.publish(Empty())
        print('  => gave up after 240 s; cancelled')
        return 'failed', end
    except KeyboardInterrupt:
        n.cancel.publish(Empty())
        n.spin_for(0.3)
        print('  => cancelled')
        return 'cancelled', end


if __name__ == '__main__':
    sys.exit(main())
