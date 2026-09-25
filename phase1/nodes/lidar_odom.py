#!/usr/bin/env python3
"""lidar_odom.py — LiDAR odometry: where the rover is, from the walls, every scan.

SENSOR_FUSION_PLAN.md stage M2 (LOCALIZATION_GAPS.md G1, G2, G4). No ROS in
this file: the same class runs offline on recorded bags (phase1/harness) and,
later, live in a node.

    lo = LidarOdom(mount=(x, y, yaw))      # base_link -> laser, from TF
    lo.on_gyro(t, wz)                      # 200 Hz, IMU stamps
    lo.on_scan(t, ranges, angle_min, angle_inc, scan_time)
    lo.x, lo.y, lo.th, lo.last             # pose in its own odom frame, and the fit

WHAT EACH STEP IS FOR

  de-skew   A C1 scan takes ~0.1 s. Turning at w rad/s, the rover turns w*0.1
            rad DURING one scan, so the walls in it are bent. Every beam is
            moved to where it would have been seen from the pose at the scan's
            stamp, using the gyro's rotation (and the last velocity) between
            the beam's time and the stamp. Which beam the stamp belongs to and
            which way the beams run in time are settings (DESKEW_DIR, DESKEW_REF),
            chosen by grading against LiDAR truth, not assumed.

  predict   gyro rotation since the last scan + the last velocity: the
            starting guess for the match.

  match     the scan against a SUBMAP (the last few keyframes, voxel-thinned),
            point-to-line (Censi 2008), Huber-weighted, so walls pull and
            people do not. Matching against a submap rather than the previous
            scan alone keeps errors from compounding scan by scan.

  confidence  the fit's residual, inlier count and the smallest eigenvalue of
            its translation information: in a plain corridor that eigenvalue
            collapses (Zhang, Kaess & Singh 2016) and the fit says so instead
            of returning a confident number. A failed fit falls back to the
            prediction and is flagged.
"""
import math
from collections import deque

import numpy as np
from scipy.spatial import cKDTree

RANGE_LO, RANGE_HI = 0.12, 8.0
# the rover's own outline + 2 cm (description/params.yaml envelope)
OWN = (0.182 + 0.02, -0.178 - 0.02, 0.19 + 0.02)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


class Submap:
    """Recent keyframe points in the odom frame, thinned, with line normals."""

    def __init__(self, voxel):
        self.voxel = voxel
        self.pts = None
        self.tree = None
        self.nrm = None
        self.flat = None

    def build(self, clouds):
        P = np.vstack(clouds)
        key = np.floor(P / self.voxel).astype(np.int64)
        _, idx = np.unique(key[:, 0] * 1_000_003 + key[:, 1], return_index=True)
        self.pts = P[idx]
        self.tree = cKDTree(self.pts)
        k = min(8, len(self.pts))
        _, nb = self.tree.query(self.pts, k=k)
        q = self.pts[nb] - self.pts[nb].mean(axis=1, keepdims=True)
        w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', q, q))
        self.nrm = v[:, :, 0]
        self.flat = 1.0 - w[:, 0] / np.maximum(w[:, 1], 1e-12)


class Fit:
    __slots__ = ('x', 'y', 'th', 'ok', 'residual', 'inliers', 'eig', 'why', 'iters')

    def __init__(self, x, y, th, ok, residual=1.0, inliers=0, eig=0.0, why='', iters=0):
        self.x, self.y, self.th, self.ok = x, y, th, ok
        self.residual, self.inliers, self.eig, self.why, self.iters = residual, inliers, eig, why, iters


def match(src, sm, guess, iters=30):
    """Pose of src (base_link points) in the submap's frame, point-to-line."""
    x, y, th = guess
    gate = 0.25
    for it in range(iters):
        cur = src @ rot(th).T + np.array([x, y])
        d, idx = sm.tree.query(cur)
        keep = (d < gate) & (sm.flat[idx] > 0.8)
        if keep.sum() < 40:
            return Fit(x, y, th, False, why='no overlap', iters=it)
        n, q, p = sm.nrm[idx[keep]], sm.pts[idx[keep]], cur[keep]
        r = np.einsum('ij,ij->i', n, p - q)
        rel = p - np.array([x, y])
        J = np.stack([n[:, 0], n[:, 1], -n[:, 0] * rel[:, 1] + n[:, 1] * rel[:, 0]], axis=1)
        w = np.where(np.abs(r) < 0.02, 1.0, 0.02 / np.maximum(np.abs(r), 1e-9))
        H = J.T @ (J * w[:, None])
        dx = -np.linalg.solve(H + 1e-9 * np.eye(3), J.T @ (w * r))
        x, y, th = x + dx[0], y + dx[1], wrap(th + dx[2])
        gate = max(0.05, gate * 0.7)
        if abs(dx[0]) + abs(dx[1]) < 1e-4 and abs(dx[2]) < 1e-4:
            break
    res = float(np.median(np.abs(r)))
    eig = float(np.linalg.eigvalsh(H[:2, :2])[0])
    ok, why = True, ''
    if res > 0.03:
        ok, why = False, f'residual {res * 100:.1f} cm'
    elif keep.sum() < 100:
        ok, why = False, f'{int(keep.sum())} inliers'
    elif eig < 20.0:
        ok, why = False, 'degenerate'
    return Fit(x, y, th, ok, res, int(keep.sum()), eig, why, it + 1)


class LidarOdom:
    # which way the beams run in time along the array (+1: index order,
    # -1: reverse, 0: no de-skew) and which fraction of the sweep the stamp
    # belongs to. GRADED 2026-09-26 over 13 turning runs, 43 LiDAR-truth
    # checkpoints (position mean / max, heading mean / max):
    #   off           0.61 / 1.57 cm   1.32 / 5.31 deg
    #   +1 (any ref)  0.87-1.04 / 2.3-2.8 cm   2.7-3.2 / 12-14 deg  (worse than off)
    #   -1 (any ref)  0.27-0.28 / 0.88-0.98 cm 0.15-0.17 / 0.37-0.39 deg
    # The beams run backwards in time along the array (the C1 spins clockwise);
    # the reference point barely matters.
    DESKEW_DIR = -1
    DESKEW_REF = 0.5

    KF_DIST, KF_ROT = 0.10, math.radians(10.0)
    SUBMAP_KF = 10
    VOXEL = 0.03

    def __init__(self, mount, deskew_dir=None, deskew_ref=None):
        self.mount = mount
        self.dir = self.DESKEW_DIR if deskew_dir is None else deskew_dir
        self.ref = self.DESKEW_REF if deskew_ref is None else deskew_ref
        self.x = self.y = self.th = 0.0
        self.vx = self.vy = 0.0                # odom frame, m/s
        self.t = None
        self.g_t, self.g_th = [], []           # gyro heading, integrated on IMU stamps
        self.g_bias = None
        self._g_boot = []
        self.kfs = deque(maxlen=self.SUBMAP_KF)
        self.kf_pose = None
        self.sm = Submap(self.VOXEL)
        self.last = None
        self.n_fail = 0

    # ── gyro ────────────────────────────────────────────────────────────────
    def on_gyro(self, t, wz):
        if self.g_bias is None:
            # bias from the first second: every recorded run starts still
            self._g_boot.append(wz)
            if len(self._g_boot) >= 150:
                self.g_bias = float(np.median(self._g_boot))
            return
        if self.g_t:
            dt = t - self.g_t[-1]
            if not 0.0 < dt < 0.1:
                self.g_t[-1] = t
                return
            self.g_th.append(self.g_th[-1] + (wz - self.g_bias) * dt)
        else:
            self.g_th.append(0.0)
        self.g_t.append(t)
        if len(self.g_t) > 4000:                 # keep ~20 s: only recent heading is ever read
            del self.g_t[:1000], self.g_th[:1000]

    def gyro_at(self, t):
        if len(self.g_t) < 2:
            return None
        return np.interp(t, self.g_t, self.g_th)

    # ── scan ────────────────────────────────────────────────────────────────
    def points(self, t, ranges, amin, ainc, scan_time):
        r = np.asarray(ranges, dtype=np.float64)
        n = r.size
        a = amin + np.arange(n) * ainc
        ok = np.isfinite(r) & (r > RANGE_LO) & (r < RANGE_HI)
        p = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
        mx, my, myaw = self.mount
        p = p @ rot(myaw).T + np.array([mx, my])           # base_link at each beam's own time
        if self.dir != 0 and scan_time > 0 and len(self.g_t) >= 2:
            # beam i was measured at t + dir * (i/n - ref) * scan_time; bring it
            # to the base_link pose at t: rotate by the heading change, shift
            # by the translation (last velocity, in base axes)
            frac = np.arange(n) / n
            tb = t + self.dir * (frac - (self.ref if self.dir > 0 else 1.0 - self.ref)) * scan_time
            dth = np.interp(tb, self.g_t, self.g_th) - np.interp(t, self.g_t, self.g_th)
            c, s = np.cos(dth), np.sin(dth)
            p = np.stack([c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]], axis=1)
            vb = rot(-self.th) @ np.array([self.vx, self.vy])
            p += np.outer(tb - t, vb)
        own = (p[:, 0] < OWN[0]) & (p[:, 0] > OWN[1]) & (np.abs(p[:, 1]) < OWN[2])
        return p[ok & ~own]

    def on_scan(self, t, ranges, amin, ainc, scan_time):
        p = self.points(t, ranges, amin, ainc, scan_time)
        if len(p) < 60:
            return None
        if self.t is None:
            self.t = t
            self._keyframe(p)
            self.last = Fit(0.0, 0.0, 0.0, True, 0.0, len(p), 0.0)
            return self.last
        dt = t - self.t
        g0, g1 = self.gyro_at(self.t), self.gyro_at(t)
        dth = (g1 - g0) if (g0 is not None and g1 is not None) else 0.0
        guess = (self.x + self.vx * dt, self.y + self.vy * dt, wrap(self.th + dth))
        f = match(p, self.sm, guess)
        if not f.ok:
            self.n_fail += 1
            f.x, f.y, f.th = guess
        if dt > 0:
            a = 0.5                               # light smoothing of the velocity
            self.vx = a * (f.x - self.x) / dt + (1 - a) * self.vx
            self.vy = a * (f.y - self.y) / dt + (1 - a) * self.vy
        self.x, self.y, self.th, self.t = f.x, f.y, f.th, t
        self.last = f
        kx, ky, kth = self.kf_pose
        if f.ok and (math.hypot(f.x - kx, f.y - ky) > self.KF_DIST or abs(wrap(f.th - kth)) > self.KF_ROT):
            self._keyframe(p)
        return f

    def _keyframe(self, p):
        self.kfs.append(p @ rot(self.th).T + np.array([self.x, self.y]))
        self.kf_pose = (self.x, self.y, self.th)
        self.sm.build(list(self.kfs))
