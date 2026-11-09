"""rl2_drive.py -- the route-following driver on the robot: observation and policy. No ROS.

kino_0.1.0. Trained in ~/kino_sim/rl2 (env.py); this file must build EXACTLY what env._observe
builds -- kino_sim rl2/tests/test_parity_rl2.py checks it on the training env's own points.

Inputs reach already has (base_link at the rover now):
  scan    the latest LiDAR scan's points            -> patch ch0 (as the env's ray-cast hits)
  cam     the camera obstacle memory (dobs) points    -> patch ch1 (the env's seen camera-band edges)
  known   known_floor.KnownFloor.known_base(pose)       -> patch ch2 (floor not known)
  guide   tight_route._guide on the LiDAR (h(x, y) -> walking distance, base frame at its own pose)
  target  the leg's end or the goal (base frame), vel (v, w), last command (m/s, rad/s),
          pack (pivot left, pivot right, k), t in this drive (s), s since the last 10 cm of progress
The shield is rl_drive.shield (all points, 2 cm), as the training env's.
"""
import math

import numpy as np

P, PRES, P_BACK = 64, 0.04, 24
WAYPTS, WSTEP = 8, 0.25
MAX_S, STALL_S = 90.0, 25.0
VX_MAX, VX_MIN, WZ_MAX = 0.20, -0.10, 1.5
STATE_DIM = 31
DIRS = [k * 2 * math.pi / 16 for k in range(16)]
_u = (np.arange(P) - P_BACK + 0.5) * PRES
_v = (np.arange(P) - P / 2 + 0.5) * PRES
PU, PV = np.meshgrid(_u, _v, indexing='ij')         # patch cell centres, base_link (as env pu, pv)


def raster(pts):
    """N x 2 base_link points -> P x P patch layer (rows = forward u, cols = left v)."""
    img = np.zeros((P, P), np.float32)
    if pts is None or not len(pts):
        return img
    pts = np.asarray(pts, np.float32)                    # float32 as the env, for the same cells
    i = (pts[:, 0] / np.float32(PRES) + P_BACK).astype(int)        # truncation, as torch .long() in env
    j = (pts[:, 1] / np.float32(PRES) + P / 2).astype(int)
    ok = (i >= 0) & (i < P) & (j >= 0) & (j < P)
    img[i[ok], j[ok]] = 1.0
    return img


def route_points(h, target, th=0.0):
    """WAYPTS points 25 cm apart walking the guide downhill from the rover (base frame), as
    env._observe; with no guide, straight toward the target. The env tries 16 directions fixed in
    the WORLD (map/odom) frame, so th (the rover's heading there) turns them into base frame."""
    pts = []
    x = y = 0.0
    for _ in range(WAYPTS):
        if h is None:
            d = math.hypot(target[0] - x, target[1] - y)
            s = min(WSTEP, d) / d if d > 1e-6 else 0.0
            x, y = x + (target[0] - x) * s, y + (target[1] - y) * s
        else:
            best = min([a - th for a in DIRS], key=lambda a: h(x + WSTEP * math.cos(a), y + WSTEP * math.sin(a)))
            x, y = x + WSTEP * math.cos(best), y + WSTEP * math.sin(best)
        pts.append((x, y))
    return pts


def observe(scan, cam, h, target, vel, last, pack, t, stall_s, th=0.0, known=None):
    """(state (31,), patch (3, P, P)) exactly as rl2/env.py builds them (a 2-channel policy: patch[:2])."""
    gx, gy = target
    gd = math.hypot(gx, gy)
    ga = math.atan2(gy, gx)
    F0 = h(0.0, 0.0) if h is not None else gd
    if not math.isfinite(F0) or F0 > 500:
        F0 = gd
    way = []
    for wx, wy in route_points(h, target, th):
        way += [max(-1.5, min(1.5, wx / 2.0)), max(-1.5, min(1.5, wy / 2.0))]
    pl, pr, k = pack
    state = np.array([min(gd, 6.0) / 6.0, math.sin(ga), math.cos(ga)] + way
                     + [min(F0, 12.0) / 12.0, vel[0] / 0.2, vel[1] / 1.0,
                        last[0] / VX_MAX, last[1] / WZ_MAX,
                        pl[0] / 0.1, pl[1] / 0.1, pr[0] / 0.1, pr[1] / 0.1, k,
                        min(t, MAX_S) / MAX_S, stall_s / STALL_S], np.float32)
    # ch2 (policies from r005): floor NOT known -- not seen, not driven over (known_floor.py), at
    # each cell's centre; where the shield will not let it back up. known None: all unknown.
    unk = np.ones((P, P), np.float32) if known is None else (~known(PU, PV)).astype(np.float32)
    patch = np.stack([raster(scan), raster(cam), unk])
    return state, patch


def to_cmd(a):
    a = np.clip(np.asarray(a, float), -1, 1)
    vx = a[0] * VX_MAX if a[0] >= 0 else -a[0] * VX_MIN
    return float(vx), float(a[1] * WZ_MAX)
