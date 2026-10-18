#!/usr/bin/env python3
"""relocalize.py — where am I in the saved home? One lidar scan against the saved map.

    python3 relocalize.py <map.yaml> [--hint x,y,theta] [--json]     (in the container: reads one /scan)

slam_toolbox resumes a saved room only from a GIVEN pose (map_start_pose):
`./rover slam` used to give it where the rover stood at the last autosave. On
2026-10-18 the rover had been carried elsewhere and power-cycled several times
in between: slam_toolbox locked onto the wrong place, and the 2-minute autosave
then wrote that wrong lock over the good room (rover OPEN_ISSUES, kino_sim
docs/PLAN_REAL_TO_TWIN.md R3).

This finds the pose instead of assuming it:

  1. the saved occupancy grid -> a likelihood field: for every cell, how far
     the nearest wall is (cv2.distanceTransform), turned into a 0..1 score
  2. the scan -> ~180 endpoints in the rover's own frame (base_link)
  3. a coarse search over every FREE cell (10 cm) x 72 headings (5 deg): the
     score of a pose is the mean field value under the scan's endpoints
  4. the best few refined locally (2.5 cm, 1 deg)
  5. accepted only if the score is high AND clearly better than the best pose
     somewhere else (a symmetric room has two good answers: say so, do not guess)

`verdict` is one of: 'locked' (use it), 'ambiguous' (two places fit), 'lost'
(nothing fits: a new room, or the furniture moved a lot). Pure numpy + cv2:
~1-3 s on the Orin for a home-sized map.
"""
import math

import numpy as np

SIGMA = 0.05            # m: how close an endpoint must be to a wall to count fully
HIT = 0.10              # m: an endpoint within this of a wall is a "hit" (for the report)
LOCK_SCORE = 0.55       # mean field value needed to trust a pose
LOCK_HITS = 0.60        # and this fraction of endpoints on walls
MARGIN = 0.10           # the runner-up somewhere else must score this much lower
ELSEWHERE_M = 0.5       # "somewhere else": farther than this, or ...
ELSEWHERE_DEG = 25.0    # ... turned more than this


def load_map(yaml_path):
    """A map_saver map (.yaml + .pgm) -> (occupied bool grid [row 0 = lowest y], resolution, origin x, y)."""
    import cv2
    import yaml
    from pathlib import Path
    meta = yaml.safe_load(Path(yaml_path).read_text())
    img = cv2.imread(str(Path(yaml_path).parent / meta['image']), cv2.IMREAD_GRAYSCALE)
    img = np.flipud(img)                                  # map_saver writes the top row first
    occ_th = meta.get('occupied_thresh', 0.65)
    free_th = meta.get('free_thresh', 0.25)
    p = (255 - img.astype(float)) / 255.0 if not meta.get('negate', 0) else img.astype(float) / 255.0
    occupied = p > occ_th
    free = p < free_th
    return occupied, free, float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])


def grid_from_msg(msg):
    """An OccupancyGrid message -> (occupied, free, resolution, ox, oy)."""
    g = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    return g >= 65, (g >= 0) & (g < 25), msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y


def field(occupied, res):
    """Likelihood field: 1 on a wall, falling off with distance (Gaussian, SIGMA)."""
    import cv2
    d = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5) * res
    return np.exp(-0.5 * (d / SIGMA) ** 2).astype(np.float32), d


def scan_points(ranges, angle_min, angle_inc, laser_pose, rmin=0.15, rmax=8.0, keep=180):
    """LaserScan ranges -> endpoints in base_link (N x 2), at most `keep`, evenly spread.
    laser_pose: (x, y, yaw) of the laser in base_link."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    r, a = r[ok], a[ok]
    if len(r) > keep:
        idx = np.linspace(0, len(r) - 1, keep).round().astype(int)
        r, a = r[idx], a[idx]
    lx, ly, lth = laser_pose
    px, py = r * np.cos(a + lth) + lx, r * np.sin(a + lth) + ly
    return np.stack([px, py], 1)


def _score(F, res, ox, oy, pts, xs, ys, ths):
    """Mean field value of the endpoints for every (x, y) in xs/ys (same length) x every theta."""
    H, W = F.shape
    out = np.empty((len(xs), len(ths)), np.float32)
    for k, th in enumerate(ths):
        c, s = math.cos(th), math.sin(th)
        rx = pts[:, 0] * c - pts[:, 1] * s           # endpoints rotated, then shifted per candidate
        ry = pts[:, 0] * s + pts[:, 1] * c
        u = ((xs[:, None] + rx[None, :] - ox) / res).astype(np.int32)
        v = ((ys[:, None] + ry[None, :] - oy) / res).astype(np.int32)
        inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        vals = np.zeros(u.shape, np.float32)
        vals[inside] = F[v[inside], u[inside]]
        out[:, k] = vals.mean(1)
    return out


def relocalize(occupied, free, res, ox, oy, pts, hint=None, coarse=0.10, coarse_deg=5.0):
    """-> dict(verdict, pose (x, y, theta), score, hits, runner_up, why)."""
    F, dist = field(occupied, res)
    # candidates: free cells (the rover stands on free floor), every `coarse` m
    step = max(1, int(round(coarse / res)))
    vv, uu = np.nonzero(free[::step, ::step])
    xs = ox + (uu * step + 0.5) * res
    ys = oy + (vv * step + 0.5) * res
    if hint is not None:                              # the hint is just one more candidate, never trusted blindly
        xs, ys = np.append(xs, hint[0]), np.append(ys, hint[1])
    if len(xs) == 0 or len(pts) < 20:
        return dict(verdict='lost', pose=None, score=0.0, hits=0.0, runner_up=0.0, why='no free floor or too few beams')
    ths = np.radians(np.arange(0.0, 360.0, coarse_deg))
    S = _score(F, res, ox, oy, pts, xs, ys, ths)
    # the best 12 distinct coarse poses, then refine each
    flat = np.argsort(S, axis=None)[::-1]
    seeds = []
    for f in flat:
        i, k = divmod(int(f), S.shape[1])
        p = (xs[i], ys[i], ths[k])
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 0.15 or abs(_wrap(p[2] - q[2])) > math.radians(10) for q in seeds):
            seeds.append(p)
        if len(seeds) == 12:
            break
    refined = []
    for (x, y, th) in seeds:
        best = (x, y, th, -1.0)
        for span, dstep, astep in ((0.10, 0.025, math.radians(2)), (0.03, 0.01, math.radians(0.5))):
            gx = np.arange(best[0] - span, best[0] + span + 1e-9, dstep)
            gy = np.arange(best[1] - span, best[1] + span + 1e-9, dstep)
            GX, GY = np.meshgrid(gx, gy)
            gt = np.arange(best[2] - 3 * astep, best[2] + 3 * astep + 1e-9, astep)
            s = _score(F, res, ox, oy, pts, GX.ravel(), GY.ravel(), gt)
            i, k = np.unravel_index(int(np.argmax(s)), s.shape)
            if s[i, k] > best[3]:
                best = (float(GX.ravel()[i]), float(GY.ravel()[i]), float(gt[k]), float(s[i, k]))
        refined.append(best)
    refined.sort(key=lambda p: -p[3])
    top = refined[0]
    runner = max((p[3] for p in refined[1:]
                  if math.hypot(p[0] - top[0], p[1] - top[1]) > ELSEWHERE_M
                  or abs(math.degrees(_wrap(p[2] - top[2]))) > ELSEWHERE_DEG), default=0.0)
    hits = _hits(dist, res, ox, oy, pts, top)
    pose = (top[0], top[1], _wrap(top[2]))
    if top[3] < LOCK_SCORE or hits < LOCK_HITS:
        verdict, why = 'lost', f'best fit only {top[3]:.2f} ({100 * hits:.0f} % of beams on walls)'
    elif top[3] - runner < MARGIN:
        verdict, why = 'ambiguous', f'two places fit: {top[3]:.2f} vs {runner:.2f}'
    else:
        verdict, why = 'locked', f'fit {top[3]:.2f}, {100 * hits:.0f} % of beams on walls; next best elsewhere {runner:.2f}'
    return dict(verdict=verdict, pose=pose, score=round(top[3], 3), hits=round(hits, 3),
                runner_up=round(runner, 3), why=why)


def agreement(occupied, res, ox, oy, pts, pose):
    """Fraction of endpoints within HIT of a wall at a GIVEN pose: does the current scan
    still agree with the map where SLAM says the rover is? (The autosave's gate.)"""
    _, dist = field(occupied, res)
    return _hits(dist, res, ox, oy, pts, pose)


def _hits(dist, res, ox, oy, pts, pose):
    x, y, th = pose[:3]
    c, s = math.cos(th), math.sin(th)
    u = ((x + pts[:, 0] * c - pts[:, 1] * s - ox) / res).astype(int)
    v = ((y + pts[:, 0] * s + pts[:, 1] * c - oy) / res).astype(int)
    H, W = dist.shape
    inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    if not inside.any():
        return 0.0
    return float((dist[v[inside], u[inside]] < HIT).sum() / len(pts))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def conflict(occupied, free, res, ox, oy, pts, pose, clear=0.15):
    """Fraction of endpoints that land on floor the map knows is FREE and open
    (> `clear` from any wall). A wrong lock puts walls where the map saw floor;
    a new, unmapped room puts them in UNKNOWN cells, which is not a conflict."""
    _, dist = field(occupied, res)
    x, y, th = pose[:3]
    c, s = math.cos(th), math.sin(th)
    u = ((x + pts[:, 0] * c - pts[:, 1] * s - ox) / res).astype(int)
    v = ((y + pts[:, 0] * s + pts[:, 1] * c - oy) / res).astype(int)
    H, W = dist.shape
    inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    if not inside.any():
        return 0.0
    bad = free[v[inside], u[inside]] & (dist[v[inside], u[inside]] > clear)
    return float(bad.sum() / len(pts))


def read_scan(timeout=10.0):
    """One /scan and the laser's pose in base_link, from the live robot (or twin)."""
    import rclpy
    import tf2_ros
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    rclpy.init()
    n = rclpy.create_node('relocalize')
    box = {}
    n.create_subscription(LaserScan, '/scan', lambda m: box.__setitem__('s', m),
                          QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    import time
    end, lp = time.time() + timeout, None
    while time.time() < end and ('s' not in box or lp is None):
        rclpy.spin_once(n, timeout_sec=0.1)
        if 's' in box and lp is None:
            try:
                t = buf.lookup_transform('base_link', box['s'].header.frame_id, rclpy.time.Time()).transform
                q = t.rotation
                lp = (t.translation.x, t.translation.y,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            except Exception:  # noqa: BLE001 -- not yet
                pass
    n.destroy_node()
    rclpy.shutdown()
    if 's' not in box or lp is None:
        return None
    s = box['s']
    return scan_points(s.ranges, s.angle_min, s.angle_increment, lp)


def main():
    import argparse
    import json
    ap = argparse.ArgumentParser()
    ap.add_argument('map_yaml')
    ap.add_argument('--hint', help='x,y,theta: where the rover probably is (one more candidate, not trusted)')
    a = ap.parse_args()
    pts = read_scan()
    if pts is None:
        print(json.dumps(dict(verdict='lost', pose=None, why='no /scan or no base_link -> laser transform')))
        return 2
    occ, free, res, ox, oy = load_map(a.map_yaml)
    hint = tuple(float(v) for v in a.hint.split(',')) if a.hint else None
    out = relocalize(occ, free, res, ox, oy, pts, hint=hint)
    print(json.dumps(out))
    return 0 if out['verdict'] == 'locked' else 1


if __name__ == '__main__':
    import sys
    sys.exit(main())
