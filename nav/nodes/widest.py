"""widest.py — the widest way from the rover to a goal, on the LiDAR's points.

No ROS. reach_node.py asks it when nav2 finds no path: is there ANY way wide
enough for the body, or is the goal simply out of reach through gaps too tight?

WHY (twin 2026-10-20, world gap36: the only way on is a 36 cm gap, the rover is
38). The old answer took 269 s and 3.8 m of driving -- tight search, pass_far x4,
an "approach" through nav2 for 125 s, waits -- before "no way from here". The
real robot did the same in its tight spots (way_back 24-32 hops). A person looks
and says "too narrow". This is that look, with a number.

HOW: the points go onto a grid (RES); each cell's clearance is its distance to the
nearest point (capped at CAP); the widest way is the path that keeps the largest
smallest clearance (a bottleneck / max-min path, Dijkstra on -clearance). Its
bottleneck is the narrowest place the rover would have to pass.

ONE-SIDED, ON PURPOSE: it may only ever say "too tight" when that is certain.
  - LiDAR points only: the camera map can hold glossy-floor specks that narrow
    a gap that fits (the door, 2026-10-10); the LiDAR's points are real things.
  - Unseen space is free, and every bottleneck gets a cell of the benefit of the
    doubt on each side (+2 RES). So a gap is called too tight only when even the
    most generous reading of what the LiDAR saw cannot take the body.
  - Things the LiDAR cannot see (low, or above its plane) only make it narrower:
    they never turn a "too tight" into a fit.
"""
import heapq
import math

import numpy as np

RES = 0.025                 # m per cell (the global costmap's)
CAP = 0.40                  # m: clearance beyond this does not matter
MARGIN_BOX = 1.0            # m of grid around the rover and the goal


def widest(pts, goal, start=(0.0, 0.0)):
    """pts: N x 2 points (any frame), goal / start (x, y) in it. Returns
    (width_m, info): width = 2 x the bottleneck clearance + 2 RES (the generous
    reading), or None if there is nothing in the way at all (CAP everywhere).
    info: {'at': (x, y) of the bottleneck, 'cells': grid size, 'ms'}."""
    import time
    t0 = time.time()
    pts = np.asarray(pts, float).reshape(-1, 2)
    x0 = min(start[0], goal[0]) - MARGIN_BOX
    y0 = min(start[1], goal[1]) - MARGIN_BOX
    x1 = max(start[0], goal[0]) + MARGIN_BOX
    y1 = max(start[1], goal[1]) + MARGIN_BOX
    W, H = int(math.ceil((x1 - x0) / RES)), int(math.ceil((y1 - y0) / RES))
    inb = (pts[:, 0] >= x0) & (pts[:, 0] < x1) & (pts[:, 1] >= y0) & (pts[:, 1] < y1)
    occ = np.zeros((H, W), bool)
    ij = ((pts[inb] - (x0, y0)) / RES).astype(int)
    occ[ij[:, 1], ij[:, 0]] = True
    # clearance: distance (m) to the nearest occupied cell, capped -- shifted minima
    clear = np.full((H, W), CAP)
    k = int(CAP / RES)
    for dj in range(-k, k + 1):
        for di in range(-k, k + 1):
            d = math.hypot(di, dj) * RES
            if d >= CAP:
                continue
            src = occ[max(0, -dj):H - max(0, dj), max(0, -di):W - max(0, di)]
            dst = clear[max(0, dj):H - max(0, -dj), max(0, di):W - max(0, -di)]
            np.minimum(dst, np.where(src, d, CAP), out=dst)
    cell = lambda p: (min(H - 1, max(0, int((p[1] - y0) / RES))), min(W - 1, max(0, int((p[0] - x0) / RES))))
    s, g = cell(start), cell(goal)
    # max-min path: best[c] = the largest bottleneck with which c can be reached
    best = np.full((H, W), -1.0)
    best[s] = clear[s]
    pq = [(-clear[s], s)]
    while pq:
        nb, c = heapq.heappop(pq)
        b = -nb
        if b < best[c]:
            continue
        if c == g:
            break
        j, i = c
        for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            jj, ii = j + dj, i + di
            if 0 <= jj < H and 0 <= ii < W:
                v = min(b, clear[jj, ii])
                if v > best[jj, ii]:
                    best[jj, ii] = v
                    heapq.heappush(pq, (-v, (jj, ii)))
    bott = float(best[g])
    info = {'cells': W * H, 'ms': round((time.time() - t0) * 1000)}
    if bott >= CAP - 1e-9:
        return None, info
    return 2 * bott + 2 * RES, info
