"""rl_drive.py -- the learned tight-space driver: observation, policy, shield. No ROS.

RL_PLAN.md. reach_node.rl_round() calls step() at 10 Hz with what it already
has (obstacle points in base_link from near_now(): LiDAR + fused camera +
sonar; the target; the walking-distance guide; odometry) and publishes the
command it returns on /cmd_vel. Trained in ~/kino_sim/rl (env.py): the
observation, the action scaling and the SHIELD here must stay identical to
env.py -- kino_sim rl/tests check it on random cases.

THE SHIELD decides, not the policy: every command is rolled 0.8 s ahead with
the measured motion model (lag, deadband, the learned pivot slide) and sent
only if the outline + 2 cm (SHIELD_MARGIN) meets no point on the way. Else
the same turn at half speed, else the turn alone, else stop. The policy can be
clumsy; it cannot drive into what the rover sees.
"""
import json
import math

import numpy as np

FRONT, REAR, SIDE = 0.182, 0.178, 0.19          # = goal_exec
SECTORS, CLIP = 72, 3.0
VX_MAX, VX_MIN, WZ_MAX = 0.20, -0.10, 1.5
OBS_DIM = SECTORS + 3 + 3 + 2 + 2 + 5 + 1
MAX_S = 60.0
SHIELD_T, SHIELD_DT, SHIELD_MARGIN = 16, 0.05, 0.02    # 0.8 s in 0.05 s steps (0.1: a hard turn moved the rear corner ~4 cm between checks -- kino_sim rl2 r008, 2026-11-10)
HALF_TRACK, DEADBAND, TURN_GAIN, TURN_OFF, ARC = 0.17, 0.01, 0.52, 0.21, 0.22
LAG_S = 0.2
SHIELD_NEAR_M = 1.6                              # points farther cannot be met in 0.8 s


class Policy:
    """The exported actor (kino_sim rl/export.py): normaliser + MLP with ELU, numpy."""

    def __init__(self, path):
        d = np.load(path)
        self.mean = d['mean'] if 'mean' in d else None
        self.std = d['std'] if 'std' in d else None
        self.eps = float(d['eps']) if 'eps' in d else 0.0
        self.n = int(d['n_layers'])
        self.W = [d[f'W{i}'] for i in range(self.n)]
        self.b = [d[f'b{i}'] for i in range(self.n)]
        self.meta = json.loads(str(d['meta'])) if 'meta' in d else {}
        assert int(d['obs_dim']) == OBS_DIM, f"policy expects {int(d['obs_dim'])} inputs, rl_drive builds {OBS_DIM}"

    def __call__(self, obs):
        x = np.asarray(obs, np.float32)
        if self.mean is not None:
            x = (x - self.mean) / (self.std + self.eps)
        for i in range(self.n):
            x = x @ self.W[i].T + self.b[i]
            if i < self.n - 1:
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
        return x


def to_cmd(a):
    a = np.clip(np.asarray(a, float), -1, 1)
    vx = a[0] * VX_MAX if a[0] >= 0 else -a[0] * VX_MIN
    return float(vx), float(a[1] * WZ_MAX)


def outline_dist(bx, by, margin=0.0):
    dx = np.maximum(bx - (FRONT + margin), -(REAR + margin) - bx)
    dy = np.abs(by) - (SIDE + margin)
    out = np.hypot(np.maximum(dx, 0), np.maximum(dy, 0))
    return np.where((dx < 0) & (dy < 0), np.minimum(np.maximum(dx, dy), 0), out)


def sectors(pts):
    """72 x clearance from the outline per 5 deg sector, base_link points (m, clipped)."""
    S = np.full(SECTORS, CLIP)
    if pts is None or not len(pts):
        return S
    bx, by = pts[:, 0], pts[:, 1]
    d = np.minimum(outline_dist(bx, by), CLIP)
    sec = np.clip(((np.arctan2(by, bx) + math.pi) / (2 * math.pi) * SECTORS).astype(int), 0, SECTORS - 1)
    np.minimum.at(S, sec, d)
    return S


def guide_obs(h, x, y, th, goal_b):
    """(walking distance, sin, cos of its downhill in the body frame) at (x, y, th) in the
    guide's frame; h(x, y) -> m or inf (tight_route._guide). Straight line when no guide."""
    gd = math.hypot(*goal_b)
    ga = math.atan2(goal_b[1], goal_b[0])
    if h is None:
        return min(gd, CLIP), math.sin(ga), math.cos(ga)
    F0 = h(x, y)
    best, bestF = None, F0
    for k in range(8):
        a = k * math.pi / 4
        f = h(x + 0.15 * math.cos(a), y + 0.15 * math.sin(a))
        if f < bestF:
            best, bestF = a, f
    if best is None or not math.isfinite(F0) or F0 > 500:
        return min(gd, CLIP), math.sin(ga), math.cos(ga)
    da = best - th
    return min(F0, CLIP), math.sin(da), math.cos(da)


def observe(pts, goal_b, guide, vel, last, pack, t):
    """The OBS_DIM vector, exactly as env.py builds it.
    pts: N x 2 base_link; goal_b: (x, y) base_link; guide: (F, sin, cos) from guide_obs;
    vel: measured (vx, wz); last: last SENT (vx, wz) in m/s, rad/s;
    pack: (pl (x, y), pr (x, y), k); t: s in RL mode."""
    S = sectors(pts)
    gd = math.hypot(*goal_b)
    ga = math.atan2(goal_b[1], goal_b[0])
    pl, pr, k = pack
    return np.concatenate([S / CLIP,
                           [min(gd, CLIP) / CLIP, math.sin(ga), math.cos(ga)],
                           [guide[0] / CLIP, guide[1], guide[2]],
                           [vel[0] / 0.2, vel[1] / 1.0],
                           [last[0] / VX_MAX, last[1] / WZ_MAX],
                           [pl[0] / 0.1, pl[1] / 0.1, pr[0] / 0.1, pr[1] / 0.1, k],
                           [min(t, MAX_S) / MAX_S]]).astype(np.float32)


def _targets(vx, wz, k):
    vl, vr = vx - wz * HALF_TRACK, vx + wz * HALF_TRACK
    if abs(vl) < DEADBAND and abs(vr) < DEADBAND:
        return 0.0, 0.0
    if abs(vx) > 0.02:
        return vx, ARC * wz
    return vx, math.copysign(max(0.0, k * (TURN_GAIN * abs(wz) - TURN_OFF)), wz)


_m = SHIELD_MARGIN
_ru = np.arange(-REAR - _m, FRONT + _m + 1e-6, 0.02)
_rv = np.arange(-SIDE - _m, SIDE + _m + 1e-6, 0.02)
RING_U = np.concatenate([_ru, _ru, np.full_like(_rv, -REAR - _m), np.full_like(_rv, FRONT + _m)])
RING_V = np.concatenate([np.full_like(_ru, -SIDE - _m), np.full_like(_ru, SIDE + _m), _rv, _rv])
CREEP_TOL = 0.001             # m: how much nearer an already-close point may come (rounding)


def sweep_hits(pts, vx, wz, vel, pack, lag=LAG_S, known=None):
    """Does command (vx, wz), held 0.8 s from the measured (v, w), bring the outline + 2 cm
    onto any base_link point?"""
    back = known is not None and vx < 0
    if (pts is None or not len(pts)) and not back:
        return False
    if pts is None:
        pts = np.zeros((0, 2))
    pl, pr, k = pack
    vt, wt = _targets(vx, wz, k)
    v, w = vel
    x = y = th = 0.0
    a = SHIELD_DT / (lag + SHIELD_DT)
    m = SHIELD_MARGIN
    # a point ALREADY within the margin (crept in: raster cells, noise) blocks only motion that
    # brings the outline nearer to it; else every command, backing away too, is refused and the
    # rover is pinned (rl2 r002, 2026-11-09: 1 in 3 stalls ended 0-0.2 cm from a thing, frozen)
    d0 = outline_dist(pts[:, 0], pts[:, 1])
    near0 = ((pts[:, 0] < FRONT + m) & (pts[:, 0] > -REAR - m) & (np.abs(pts[:, 1]) < SIDE + m))
    for _ in range(SHIELD_T):
        v += a * (vt - v)
        w += a * (wt - w)
        P = pl if w >= 0 else pr
        vb, vyb = v + w * P[1], -w * P[0]
        c, s = math.cos(th), math.sin(th)
        x, y, th = x + (vb * c - vyb * s) * SHIELD_DT, y + (vb * s + vyb * c) * SHIELD_DT, th + w * SHIELD_DT
        c, s = math.cos(th), math.sin(th)
        qx, qy = pts[:, 0] - x, pts[:, 1] - y
        bx, by = c * qx + s * qy, -s * qx + c * qy
        inb = (bx < FRONT + m) & (bx > -REAR - m) & (np.abs(by) < SIDE + m)
        if inb.any():
            inb &= ~(near0 & (outline_dist(bx, by) >= d0 - CREEP_TOL))
            if inb.any():
                return True
        if back:
            # backing up: the outline + margin must stay over KNOWN floor (known_floor.py: seen by
            # the camera or driven over) -- nothing on the rover looks back (rl2 r003, 2026-11-09)
            ru, rv = RING_U, RING_V
            if not known(x + c * ru - s * rv, y + s * ru + c * rv).all():
                return True
    return False


def shield(pts, vx, wz, vel, pack, known=None):
    """-> (vx, wz) to send, changed, stopped -- the order env.py uses. known: f(bx, by) -> bool,
    is the floor at these base_link points known (known_floor.KnownFloor.known_base); with it a
    reverse may move the outline only over known floor (as kino_sim rl2/env.py REVERSE_KNOWN)."""
    if pts is not None and len(pts):
        pts = pts[np.hypot(pts[:, 0], pts[:, 1]) < SHIELD_NEAR_M]
    for i, (cv, cw) in enumerate(((vx, wz), (0.5 * vx, wz), (0.0, wz))):
        if not sweep_hits(pts, cv, cw, vel, pack, known=known):
            return cv, cw, i > 0, False
    return 0.0, 0.0, True, True
