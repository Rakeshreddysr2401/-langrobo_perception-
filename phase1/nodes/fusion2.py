#!/usr/bin/env python3
"""fusion2.py — one estimator, every sensor weighted by its own reliability.

SENSOR_FUSION_PLAN.md M3. No ROS inside: the harness grades it offline on
recorded runs, and a node will run the same class live.

STATE   X = [x, y, th, vx, vy, bg]
        pose in the odom frame, body-frame velocity (vy is real: a skid-steer
        pivot slides sideways), and the gyro's bias.

PREDICT on every gyro sample (200 Hz): th += (wz - bg) dt, and the pose moves
        with the body velocity. The gyro is the one sensor fast enough to carry
        the pose between everything else.

CORRECT, each by its own evidence, each checked against the prediction
(Mahalanobis gate) before it is allowed in:

  lidar   the LiDAR odometry pose, ABSOLUTE, in its own frame mapped onto ours
          by an offset. Healthy fits count with the fit's own variance; a
          failed fit is skipped, and when the fits come back the offset is
          re-anchored to where we are, so a LiDAR outage never yanks the pose.
  vo      cuVSLAM, as body velocity and turn rate over ~0.2 s windows, with
          noise that grows as landmarks fall. A heading jump (the 7.6 deg
          glitch of 2026-09-26) fails the gate and is dropped.
  wheels  forward speed and "no sideways speed", from the four encoders, with
          noise that grows with the turn rate (scrub) and with slip evidence:
          front vs rear on a side, and the wheels' turn rate vs the gyro's.
  still   all four encoders unchanged and nothing commanded for 0.3 s: the
          rover is certainly still. Velocity is pinned to zero and the gyro's
          bias is re-measured. Only the wheels can know this when people walk
          past.

FLAGS   lidar_ok, slip, stuck (wheels turning while everything else says the
        rover is not moving), plus how many measurements each gate rejected.
"""
import math
from collections import deque

import numpy as np

# The Phase 1 wheel constants (description/build keeps them honest)
METRES_PER_COUNT = math.pi * 0.085 / 1560.0
WHEEL_BASE_ROT_M = 0.5216

GATE = {'lidar': 16.3, 'vo': 16.3, 'wheels': 13.8}   # chi^2, 99.9% for 3 / 3 / 2 dof


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    """SE(2) a (+) b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class Fusion2:
    # process noise, continuous-time densities (tuned on the recorded runs)
    Q_TH = 0.002 ** 2        # rad^2/s   the gyro's white noise on heading
    Q_V = 0.8 ** 2           # (m/s)^2/s body velocity random walk
    Q_BG = 0.0005 ** 2       # (rad/s)^2/s bias walk
    VO_WINDOW = 0.2          # s
    # How cuVSLAM enters, GRADED 2026-09-26 over 19 runs (mean of each run's
    # worst checkpoint, with / without the LiDAR):
    #   velocity  0.3 cm 0.19 deg / 7.1 cm 1.12 deg
    #   absolute  0.6 cm 0.27 deg / 6.2 cm 1.43 deg
    # Absolute tracks position better on its own but fights a healthy LiDAR.
    # 'adaptive' takes each where it wins: velocity while the LiDAR is healthy,
    # absolute (re-anchored at the switch) when it is not.
    VO_MODE = 'adaptive'

    def __init__(self, use=('lidar', 'vo', 'wheels', 'still')):
        self.use = set(use)
        self.X = np.zeros(6)
        self.P = np.diag([1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-4])
        self.t = None
        self.bias_init = []
        self.gz_last = 0.0
        self.l_off = None        # lidar frame -> our odom frame
        self.l_gap = True
        self.l_rejects = 0
        self.vo_ref = None       # (t, pose) at the start of the current VO window
        self.v_off, self.v_gap, self.v_rejects = None, True, 0
        self.ticks_ref = None    # (t, ticks)
        self.still_since = None
        self.last_ticks = None
        self.cmd_active = False
        self.ticks_hist = []     # (t, ticks) for ~0.25 s of smoothing
        self.flags = {'lidar_ok': False, 'slip': 0.0, 'stuck': False}
        self.rejected = {'lidar': 0, 'vo': 0, 'wheels': 0}
        self.accepted = {'lidar': 0, 'vo': 0, 'wheels': 0, 'still': 0}
        self._moving_wheels_since = None
        # MOTION-ONLY track, never corrected, to carry late measurements forward.
        # It must not be the corrected state: its history then contains our own
        # corrections, each one gets added into the next late measurement, and
        # the filter chases itself (2026-09-26: parked, it wandered 10-20 cm).
        self.dr = (0.0, 0.0, 0.0)
        self.hist = deque(maxlen=200)      # (t, dead-reckoned pose): ~1 s

    # ── predict ─────────────────────────────────────────────────────────────
    def on_gyro(self, t, wz):
        if self.t is None:
            self.bias_init.append(wz)
            if len(self.bias_init) >= 150:           # the first ~0.75 s: runs start still
                self.X[5] = float(np.median(self.bias_init))
                self.t = t
            return
        dt = t - self.t
        if not 0.0 < dt < 0.1:
            self.t = t
            return
        self.t = t
        self.gz_last = wz
        x, y, th, vx, vy, bg = self.X
        w = wz - bg
        c, s = math.cos(th), math.sin(th)
        self.X = np.array([x + (vx * c - vy * s) * dt, y + (vx * s + vy * c) * dt,
                           wrap(th + w * dt), vx, vy, bg])
        dx, dy = (vx * c - vy * s) * dt, (vx * s + vy * c) * dt
        rc, rs = math.cos(self.dr[2] - th), math.sin(self.dr[2] - th)   # same body motion, dr's heading
        self.dr = (self.dr[0] + rc * dx - rs * dy, self.dr[1] + rs * dx + rc * dy, wrap(self.dr[2] + w * dt))
        self.hist.append((t, self.dr))
        F = np.eye(6)
        F[0, 2] = (-vx * s - vy * c) * dt
        F[0, 3], F[0, 4] = c * dt, -s * dt
        F[1, 2] = (vx * c - vy * s) * dt
        F[1, 3], F[1, 4] = s * dt, c * dt
        F[2, 5] = -dt
        Q = np.diag([1e-8 * dt, 1e-8 * dt, self.Q_TH * dt, self.Q_V * dt, self.Q_V * dt, self.Q_BG * dt])
        self.P = F @ self.P @ F.T + Q

    # ── the generic gated update ────────────────────────────────────────────
    def _update(self, name, z, h, H, R, angle_rows=()):
        y = z - h
        for i in angle_rows:
            y[i] = wrap(y[i])
        S = H @ self.P @ H.T + R
        d2 = float(y @ np.linalg.solve(S, y))
        if d2 > GATE.get(name, 16.3):
            self.rejected[name] = self.rejected.get(name, 0) + 1
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.X = self.X + K @ y
        self.X[2] = wrap(self.X[2])
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T      # Joseph form: stays symmetric
        self.accepted[name] = self.accepted.get(name, 0) + 1
        return True

    # ── lidar ───────────────────────────────────────────────────────────────
    def motion_since(self, t):
        """Motion from time t to now in the body frame at t, from the
        dead-reckoned track only -- no corrections in it."""
        if not self.hist or t >= self.hist[-1][0]:
            return (0.0, 0.0, 0.0)
        then = self.hist[0][1]
        for tt, pp in reversed(self.hist):
            if tt <= t:
                then = pp
                break
        return compose(inverse(then), self.dr)

    def on_lidar(self, t, pose, ok, var_xy, var_th):
        """t is when the scan was MEASURED; it may arrive later (the node holds
        each scan ~0.12 s for the de-skew). The measurement is carried forward
        by our own motion since t, so a late fix is not applied as if current."""
        if self.t is None or 'lidar' not in self.use:
            return
        pose = compose(pose, self.motion_since(t))
        self.flags['lidar_ok'] = bool(ok)
        if not ok:
            self.l_gap = True
            return
        cur = (self.X[0], self.X[1], self.X[2])
        if self.l_gap or self.l_off is None:
            # (re-)anchor: whatever the LiDAR frame did during the gap, from
            # here on it is measured relative to where we are now
            self.l_off = compose(cur, inverse(pose))
            self.l_gap = False
            return
        z = np.array(compose(self.l_off, pose))
        H = np.zeros((3, 6))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        R = np.diag([var_xy, var_xy, var_th])
        y = z - np.array(cur)
        y[2] = wrap(y[2])
        if math.hypot(y[0], y[1]) > 0.30 or abs(y[2]) > math.radians(10.0):
            # physically impossible in one scan: a LiDAR fault, not a
            # disagreement. Re-anchor when the fits settle.
            self.rejected['lidar'] += 1
            self.l_gap = True
            return
        if not self._update('lidar', z, np.array(cur), H, R, angle_rows=(2,)):
            # A HEALTHY fit that fails the gate means WE are overconfident: the
            # LiDAR is the most accurate sensor here (LOCALIZATION.md §10), and
            # a skid-steer pivot slides faster than the velocity states follow.
            # Widen our own uncertainty by the disagreement and take the fix.
            # (The first version rejected these and re-anchored after five in a
            # row, which locked drift in: 29.5 cm on pivot90 vs lidar's 0.3.)
            self.P[:3, :3] += np.diag(y ** 2)
            self.P[3:5, 3:5] += np.eye(2) * 0.05 ** 2
            self._update('lidar', z, np.array(cur), H, R, angle_rows=(2,))
            self.inflated = getattr(self, 'inflated', 0) + 1

    # ── vo ──────────────────────────────────────────────────────────────────
    def on_vo(self, t, pose, landmarks=100.0, healthy=True):
        if self.t is None or 'vo' not in self.use:
            return
        if not healthy or landmarks < 20:
            self.vo_ref = None
            self.v_gap = True
            return
        lidar_healthy = 'lidar' in self.use and self.flags['lidar_ok']
        if self.VO_MODE == 'absolute' or (self.VO_MODE == 'adaptive' and not lidar_healthy):
            return self._vo_absolute(t, pose, landmarks)
        self.v_gap = True        # re-anchor the absolute offset if the LiDAR drops out later
        if self.vo_ref is None:
            self.vo_ref = (t, pose)
            return
        t0, p0 = self.vo_ref
        dt = t - t0
        if dt < self.VO_WINDOW:
            return
        self.vo_ref = (t, pose)
        d = compose(inverse(p0), pose)             # motion in the body frame at the window start
        z = np.array([d[0] / dt, d[1] / dt, d[2] / dt])
        # h: body velocity, and the turn rate the gyro implies (wz - bg)
        h = np.array([self.X[3], self.X[4], self.gz_last - self.X[5]])
        H = np.zeros((3, 6))
        H[0, 3] = H[1, 4] = 1.0
        H[2, 5] = -1.0
        k = 100.0 / max(landmarks, 20.0)           # fewer landmarks, less trust
        R = np.diag([(0.03 * k) ** 2, (0.03 * k) ** 2, (math.radians(3.0) * k) ** 2])
        self._update('vo', z, h, H, R)

    def _vo_absolute(self, t, pose, landmarks):
        """cuVSLAM's pose, like the LiDAR's: an offset maps its frame onto ours,
        re-anchored after unhealthy stretches or a run of rejected fixes (its
        teleports), and each fix gated. Its noise grows as landmarks fall, and
        per fix it is far looser than the LiDAR, so with the LiDAR healthy it
        barely moves the estimate; without it, it carries position."""
        cur = (self.X[0], self.X[1], self.X[2])
        pose = compose(pose, self.motion_since(t))
        if self.v_gap or self.v_off is None:
            self.v_off = compose(cur, inverse(pose))
            self.v_gap = False
            return
        z = np.array(compose(self.v_off, pose))
        k = 100.0 / max(landmarks, 20.0)
        R = np.diag([(0.02 * k) ** 2, (0.02 * k) ** 2, (math.radians(1.0) * k) ** 2])
        H = np.zeros((3, 6))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        if self._update('vo', z, np.array(cur), H, R, angle_rows=(2,)):
            self.v_rejects = 0
        else:
            self.v_rejects += 1
            if self.v_rejects >= 10:
                self.v_gap, self.v_rejects = True, 0

    # ── wheels ──────────────────────────────────────────────────────────────
    def on_cmd(self, t, vx, wz):
        self.cmd_active = abs(vx) > 1e-3 or abs(wz) > 1e-3

    def on_ticks(self, t, ticks):
        if self.t is None:
            return
        ticks = np.asarray(ticks, dtype=float)
        self.ticks_hist.append((t, ticks))
        while self.ticks_hist and t - self.ticks_hist[0][0] > 0.25:
            self.ticks_hist.pop(0)
        moved = self.last_ticks is not None and np.any(ticks != self.last_ticks)
        self.last_ticks = ticks

        # certainly still?
        if 'still' in self.use:
            if moved or self.cmd_active:
                self.still_since = None
            elif self.still_since is None:
                self.still_since = t
            elif t - self.still_since > 0.3:
                H = np.zeros((3, 6))
                H[0, 3] = H[1, 4] = 1.0
                H[2, 5] = -1.0
                z = np.array([0.0, 0.0, 0.0])
                h = np.array([self.X[3], self.X[4], self.gz_last - self.X[5]])
                R = np.diag([0.002 ** 2, 0.002 ** 2, 0.003 ** 2])
                self._update('still', z, h, H, R)

        if 'wheels' not in self.use or len(self.ticks_hist) < 3:
            return
        (t0, k0), (t1, k1) = self.ticks_hist[0], self.ticks_hist[-1]
        dt = t1 - t0
        if dt < 0.1:
            return
        d = (k1 - k0) * METRES_PER_COUNT                     # lf, lr, rf, rr
        vl, vr = 0.5 * (d[0] + d[1]) / dt, 0.5 * (d[2] + d[3]) / dt
        v = 0.5 * (vl + vr)
        w_wheels = (vr - vl) / WHEEL_BASE_ROT_M
        w_gyro = self.gz_last - self.X[5]
        # slip evidence: the two wheels on a side must agree; the wheels' turn
        # rate must agree with the gyro's
        side = max(abs(d[0] - d[1]) / max(abs(d[0]), abs(d[1]), 1e-3),
                   abs(d[2] - d[3]) / max(abs(d[2]), abs(d[3]), 1e-3))
        yaw_dis = abs(w_wheels - w_gyro) / (abs(w_gyro) + 0.2)
        slip = min(1.0, 0.5 * side + 0.5 * yaw_dis) if max(abs(vl), abs(vr)) > 0.01 else 0.0
        self.flags['slip'] = slip
        sv = 0.02 + 0.15 * abs(w_gyro) + 0.3 * slip
        svy = 0.01 + 0.20 * abs(w_gyro) + 0.3 * slip
        H = np.zeros((2, 6))
        H[0, 3] = H[1, 4] = 1.0
        ok = self._update('wheels', np.array([v, 0.0]), np.array([self.X[3], self.X[4]]), H,
                          np.diag([sv ** 2, svy ** 2]))
        # stuck: the wheels insist on motion that nothing else sees
        fused_speed = math.hypot(self.X[3], self.X[4])
        if abs(v) > 0.05 and fused_speed < 0.01 and not ok:
            if self._moving_wheels_since is None:
                self._moving_wheels_since = t
            self.flags['stuck'] = t - self._moving_wheels_since > 1.0
        else:
            self._moving_wheels_since = None
            self.flags['stuck'] = False

    # ── out ─────────────────────────────────────────────────────────────────
    @property
    def pose(self):
        return float(self.X[0]), float(self.X[1]), float(self.X[2])
