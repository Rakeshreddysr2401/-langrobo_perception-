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


# ── the other way round: a camera speck the sonar VOUCHES is not there ─────────
# Real robot 2026-10-19 (bag ..._rviz_real3, recorded): leaving the bedroom door,
# goal_exec refused a 90 cm drive twice -- "obstacle 0.32 m into the leg" -- for ONE
# 2.5 cm cell of the fused camera map, 52 cm ahead, 17 cm from any LiDAR point, on
# bare polished marble in the image: a reflection. ~45 s lost. A sonar pointed
# straight at it would have heard nothing there.
# The camera map is flat (no heights), so silence cannot tell a floor reflection
# from a tabletop edge at 28 cm (above the cone) -- the rover drove under one this
# morning. But a furniture edge is a LINE of cells, a slipper a patch; a reflection
# is a lone speck. So a camera point is overruled only when ALL hold:
#   camera-only   no LiDAR point within ALONE_M
#   a lone speck  everything of the camera map within NEIGHBOUR_M of it spans <= SPECK_M
#   in plain view within CENTRE_RAD of a sonar's axis, VOUCH_NEAR..USE_M from it
#   heard nothing VOUCH_N readings in a row (fresh) all put the nearest echo
#                 >= VOUCH_BEYOND_M farther than the speck, or no echo at all
SPECK_M = 0.08
NEIGHBOUR_M = 0.12
ALONE_M = 0.10
CENTRE_RAD = math.radians(10)
VOUCH_NEAR = 0.15
VOUCH_N = 3
VOUCH_BEYOND_M = 0.10


def vouched_specks(camera, lidar, sensors):
    """camera: N x 2 camera-map points (base_link); lidar: M x 2; sensors: [((sx, sy, syaw),
    [recent ranges, newest last]) ...]. -> bool mask over camera: True = overruled (drop)."""
    cam = np.zeros((0, 2)) if camera is None or not len(camera) else np.asarray(camera, float)[:, :2]
    drop = np.zeros(len(cam), bool)
    if not len(cam):
        return drop
    lid = np.zeros((0, 2)) if lidar is None or not len(lidar) else np.asarray(lidar, float)[:, :2]
    for (sx, sy, syaw), recent in sensors:
        if len(recent) < VOUCH_N:
            continue
        last = [r for r in recent[-VOUCH_N:]]
        dx, dy = cam[:, 0] - sx, cam[:, 1] - sy
        dist = np.hypot(dx, dy)
        ang = np.abs((np.arctan2(dy, dx) - syaw + math.pi) % (2 * math.pi) - math.pi)
        cand = np.nonzero((ang <= CENTRE_RAD) & (dist >= VOUCH_NEAR) & (dist <= USE_M) & ~drop)[0]
        for k in cand:
            if drop[k]:
                continue
            if not all((not math.isfinite(r)) or r >= dist[k] + VOUCH_BEYOND_M for r in last):
                continue                                # the sonar hears something there (or nearer)
            if len(lid) and np.min(np.hypot(lid[:, 0] - cam[k, 0], lid[:, 1] - cam[k, 1])) <= ALONE_M:
                continue                                # the LiDAR has something there too
            nb = np.hypot(cam[:, 0] - cam[k, 0], cam[:, 1] - cam[k, 1]) <= NEIGHBOUR_M
            span = np.hypot(*(cam[nb].max(0) - cam[nb].min(0)))
            if span <= SPECK_M:
                drop |= nb                              # the whole lone speck
    return drop
