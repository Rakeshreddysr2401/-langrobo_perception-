"""sonar_witness.py -- the front sonars as a WITNESS for what the map does not have. No ROS.

The two sonars (front corners, 6.5 cm up, ~30 deg cones; description/params.yaml
`sonar`) hear what neither the LiDAR (one plane at 25 cm) nor the camera (blind
under ~40 cm) sees close in front: a slipper, a chair's foot, a toy. But a cone
says HOW FAR, not WHERE: painted into the map as an arc, a doorframe beside the
rover would look like a wall across its path and every tight gap would close.

So an echo is used only when the map cannot explain it:
  explained   a known obstacle point (LiDAR or camera map) lies inside that
              sensor's cone no farther than the echo + EXPLAIN_M -- the rover
              already knows it (the doorframe): nothing is added, nothing changes
  unexplained nothing known there: something real the map lacks. One obstacle
              point straight ahead of that sensor at the echo's range, and the
              usual checks (goal_exec's drive / turn clearance, the tight route)
              stop before it, exactly as for a LiDAR point.
Only echoes within USE_M count (beyond it the camera sees the floor ahead).
"""
import math

import numpy as np

USE_M = 0.60            # m: only close echoes -- where the camera is blind
EXPLAIN_M = 0.05        # m: a known point this much beyond the echo still explains it
CONE_SLACK = 0.05       # rad added to the half-angle when looking for an explanation


def witness_points(readings, known, fov, use_m=USE_M, explain_m=EXPLAIN_M):
    """readings: [(range m, (sx, sy, syaw)) ...] in base_link (inf / nan = no echo).
    known: N x 2 obstacle points in base_link (or None). -> M x 2 points to add."""
    out = []
    K = np.zeros((0, 2)) if known is None or not len(known) else np.asarray(known, float)[:, :2]
    half = fov / 2 + CONE_SLACK
    for r, (sx, sy, syaw) in readings:
        if r is None or not math.isfinite(r) or r > use_m:
            continue
        if len(K):
            dx, dy = K[:, 0] - sx, K[:, 1] - sy
            ang = np.abs((np.arctan2(dy, dx) - syaw + math.pi) % (2 * math.pi) - math.pi)
            if np.any((ang <= half) & (np.hypot(dx, dy) <= r + explain_m)):
                continue
        out.append((sx + r * math.cos(syaw), sy + r * math.sin(syaw)))
    return np.array(out, float).reshape(-1, 2)
