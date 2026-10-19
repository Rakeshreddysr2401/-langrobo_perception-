"""house_route.py -- a coarse route through the house map, for reach's legs.

Pure numpy + cv2, no ROS: reach_node calls route() with the slam_toolbox /map.

WHY (twin, apt_s1, 2026-10-19, recorded). nav2 plans only inside its 8 x 8 m
window, so reach drives far goals in LEGS -- and took each leg's end on the
STRAIGHT LINE to the goal. In a home the straight line runs through furniture
and walls: a leg end landed on the end of the wall beside the corridor door
("no path"), another between a ball, a bin and chair legs (76 s of turning to
reach it), and the bedroom behind a wall was "no path" for 100 s while its door
was 3 m away. A person walks the ROUTE: out of this room, down the corridor,
in at the door. The house map (slam_toolbox, the LiDAR's walls and tall
furniture, kept across power cycles) has the doors; this finds that route, and
reach takes its legs along it.

The grid is coarse (CELL m) on purpose: room-level routing, not driving. Walls
are grown by the body's half-width (a 75 cm door stays open), cells near
anything cost more (the route keeps to the middle), and space the LiDAR has not
seen yet is allowed at a price (a door seen from the corridor leads into an
unknown room -- the route must be able to go in).
"""
import heapq
import math

import numpy as np

CELL = 0.10            # m: the routing grid
BODY_R = 0.20          # m: blocked within this of an occupied cell (half the 38 cm body + 1 cm)
KEEP_OFF = 0.60        # m: closer than this to anything costs extra (prefer the middle)
WALL_W = 3.0           # extra cost per cell at BODY_R, falling to 0 at KEEP_OFF
UNKNOWN_W = 2.0        # extra cost per cell of unseen floor
OCC = 65               # occupancy >= this is a wall (nav2 map_saver's threshold)
SNAP_M = 0.6           # a start/goal inside the grown walls moves to the nearest open cell within this


def grid_costs(data, res):
    """OccupancyGrid data (H x W, -1/0..100) at res -> (blocked, step cost, cell) at CELL.
    Distances are measured on the FINE map (a 36 cm door must stay shut and a 75 cm
    one open -- a coarse grid rounds door edges by a cell), then a coarse cell takes
    the best point inside it; unknown if all of it is unseen."""
    import cv2
    occ = data >= OCC
    # to the occupied cell's EDGE, not its centre: else every gap reads one cell wider
    dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res - res / 2
    k = max(1, int(round(CELL / res)))
    H, W = data.shape
    h, w = H // k, W // k
    dk = dist[:h * k, :w * k].reshape(h, k, w, k).max(axis=(1, 3))
    unknown = (data[:h * k, :w * k] < 0).reshape(h, k, w, k).all(axis=(1, 3))
    blocked = dk < BODY_R
    near = np.clip((KEEP_OFF - dk) / (KEEP_OFF - BODY_R), 0.0, 1.0)
    return blocked, 1.0 + WALL_W * near + UNKNOWN_W * unknown, k * res


def _snap(blocked, ij, cell):
    i, j = ij
    if 0 <= i < blocked.shape[0] and 0 <= j < blocked.shape[1] and not blocked[i, j]:
        return ij
    r = int(SNAP_M / cell)
    best = None
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            a, b = i + di, j + dj
            if 0 <= a < blocked.shape[0] and 0 <= b < blocked.shape[1] and not blocked[a, b]:
                d = di * di + dj * dj
                if best is None or d < best[0]:
                    best = (d, (a, b))
    return best[1] if best else None


def astar(blocked, c, s, g):
    """8-connected A* on cell indices; -> [(i, j), ...] from s to g, or None."""
    H, W = blocked.shape
    cmin = float(c.min())
    h = lambda a: cmin * math.hypot(a[0] - g[0], a[1] - g[1])
    openq = [(h(s), 0.0, s)]
    came = {s: None}
    gs = {s: 0.0}
    nb = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
          (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
    while openq:
        _, gc, u = heapq.heappop(openq)
        if u == g:
            out = []
            while u is not None:
                out.append(u)
                u = came[u]
            return out[::-1]
        if gc > gs.get(u, math.inf):
            continue
        for di, dj, step in nb:
            v = (u[0] + di, u[1] + dj)
            if not (0 <= v[0] < H and 0 <= v[1] < W) or blocked[v]:
                continue
            ng = gc + step * 0.5 * (c[u] + c[v])
            if ng < gs.get(v, math.inf):
                gs[v] = ng
                came[v] = u
                heapq.heappush(openq, (ng + h(v), ng, v))
    return None


def route(data, res, ox, oy, start, goal):
    """The house map (H x W int array, row 0 at oy) and two points in its frame ->
    (route [(x, y), ...] start to goal in that frame, length m) or (None, why)."""
    blocked, c, cell = grid_costs(np.asarray(data), res)
    to_ij = lambda p: (int((p[1] - oy) / cell), int((p[0] - ox) / cell))
    s, g = _snap(blocked, to_ij(start), cell), _snap(blocked, to_ij(goal), cell)
    if s is None:
        return None, 'the rover is not on open floor of the house map'
    if g is None:
        return None, 'the goal is not on open floor of the house map'
    cells = astar(blocked, c, s, g)
    if cells is None:
        return None, 'no route on the house map'
    pts = [(ox + (j + 0.5) * cell, oy + (i + 0.5) * cell) for i, j in cells]
    pts[0], pts[-1] = tuple(start[:2]), tuple(goal[:2])
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    return pts, length


def along(pts, d):
    """The point d metres along the route (clamped) and the route's heading there."""
    acc = 0.0
    for a, b in zip(pts, pts[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if acc + L >= d and L > 0:
            f = (d - acc) / L
            return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])), math.atan2(b[1] - a[1], b[0] - a[0])
        acc += L
    a, b = pts[-2] if len(pts) > 1 else pts[-1], pts[-1]
    return b, math.atan2(b[1] - a[1], b[0] - a[0]) if a != b else 0.0
