"""Shared pieces of the localization test harness (SENSOR_FUSION_PLAN.md §5).

    read_bag(path)            every message we grade, as plain numpy arrays
    still_windows(b)          the stretches where the rover was certainly still
    window_scan(b, w)         one clean scan per still window, in base_link
    icp(src, dst, guess)      point-to-line ICP that also says how sure it is

The TRUTH for a run is where the LiDAR says the rover was, measured scan
against scan in a static room, at the moments the rover stood still. It uses
no odometry at all -- the wheels, gyro, camera and fusion are what it grades.
Moving scans are never used for truth: a C1 scan takes 0.1 s, so a turning
rover smears it (SENSOR_FUSION_PLAN.md §1).
"""
import json
import math
import warnings
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

# the C1's usable range for truth: its blind zone is 5 cm, and past ~8 m a
# 0.72 deg beam spacing is 10 cm between points -- too sparse to fit lines to
RANGE_LO, RANGE_HI = 0.10, 8.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def se2(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def se2_parts(T):
    return float(T[0, 2]), float(T[1, 2]), math.atan2(T[1, 0], T[0, 0])


def relative(T0, T1):
    """Pose 1 expressed in pose 0's frame."""
    return np.linalg.inv(T0) @ T1


# ── the bag ─────────────────────────────────────────────────────────────────
@dataclass
class Bag:
    path: str
    scans: list = field(default_factory=list)        # (t, ranges ndarray, angle_min, inc, scan_time)
    gyro: np.ndarray = None                           # t, wz
    ticks: np.ndarray = None                          # t, lf, lr, rf, rr
    odom: dict = field(default_factory=dict)          # topic -> ndarray t, x, y, yaw
    cmd: np.ndarray = None                            # t, vx, wz
    laser: tuple = None                               # base_link -> laser: x, y, yaw
    vo_status: np.ndarray = None                      # t, landmarks, healthy (0/1)


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage = 'mcap' if any(p.suffix == '.mcap' for p in __import__('pathlib').Path(path).glob('*')) else 'sqlite3'
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {n: get_message(t) for n, t in types.items()}
    b = Bag(path=str(path))
    gyro, ticks, cmd, vos = [], [], [], []
    odom = {}

    def stamp(m, t_ns):
        # header stamps where there is one (the /scan stamp carries the
        # measured +82 ms correction); receive time for headerless messages
        h = getattr(m, 'header', None)
        if h is not None and (h.stamp.sec or h.stamp.nanosec):
            return h.stamp.sec + h.stamp.nanosec * 1e-9
        return t_ns * 1e-9

    while r.has_next():
        topic, data, t_ns = r.read_next()
        if topic not in cls:
            continue
        m = deserialize_message(data, cls[topic])
        if topic == '/scan':
            b.scans.append((stamp(m, t_ns), np.asarray(m.ranges, dtype=np.float64),
                            m.angle_min, m.angle_increment, m.scan_time))
        elif topic == '/gyro/base':
            gyro.append((stamp(m, t_ns), m.angular_velocity.z))
        elif topic == '/wheel_ticks':
            ticks.append((t_ns * 1e-9, m.x, m.y, m.z, m.w))
        elif topic == '/vo/status':
            try:
                j = json.loads(m.data)
                vos.append((t_ns * 1e-9, float(j.get('landmarks', 0)), 1.0 if j.get('healthy', True) else 0.0))
            except ValueError:
                pass
        elif topic == '/cmd_vel':
            cmd.append((t_ns * 1e-9, m.linear.x, m.angular.z))
        elif topic in ('/odom', '/vo/odom', '/lidar/odom'):
            p = m.pose.pose
            odom.setdefault(topic, []).append((stamp(m, t_ns), p.position.x, p.position.y,
                                               yaw_of(p.orientation)))
        elif topic == '/tf_static' and b.laser is None:
            for tf in m.transforms:
                if tf.header.frame_id == 'base_link' and tf.child_frame_id == 'laser':
                    tr = tf.transform
                    b.laser = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
    b.gyro = np.array(sorted(gyro)) if gyro else None
    b.ticks = np.array(sorted(ticks)) if ticks else None
    b.cmd = np.array(sorted(cmd)) if cmd else None
    b.vo_status = np.array(sorted(vos)) if vos else None
    b.odom = {k: np.array(sorted(v)) for k, v in odom.items()}
    b.scans.sort(key=lambda s: s[0])
    return b


# ── when was it still? ──────────────────────────────────────────────────────
STILL_MIN_S = 1.2        # a window shorter than this is a pause, not a stop
GYRO_STILL = 0.02        # rad/s after bias; the turns are > 0.2 rad/s actual
CMD_STILL = 1e-3


def still_windows(b):
    """[(t0, t1)] where the wheels did not move, nothing was commanded, and the
    gyro (if recorded) was quiet. The wheels are the certain signal here: they
    cannot see people walking past (SENSOR_FUSION_PLAN.md §3.1)."""
    if b.ticks is None or len(b.ticks) < 2:
        raise SystemExit('no /wheel_ticks in the bag: stillness cannot be judged')
    t = b.ticks[:, 0]
    moving = np.zeros(len(t), dtype=bool)
    moving[1:] = np.any(np.diff(b.ticks[:, 1:], axis=0) != 0, axis=1)
    if b.cmd is not None and len(b.cmd):
        i = np.searchsorted(b.cmd[:, 0], t, side='right') - 1
        ok = i >= 0
        c = np.zeros(len(t), dtype=bool)
        c[ok] = (np.abs(b.cmd[i[ok], 1]) > CMD_STILL) | (np.abs(b.cmd[i[ok], 2]) > CMD_STILL)
        moving |= c
    if b.gyro is not None and len(b.gyro) > 50:
        bias = float(np.median(b.gyro[:50, 1]))
        gz = np.interp(t, b.gyro[:, 0], np.abs(b.gyro[:, 1] - bias))
        moving |= gz > GYRO_STILL
    out, start = [], None
    for k in range(len(t)):
        if not moving[k] and start is None:
            start = t[k]
        if (moving[k] or k == len(t) - 1) and start is not None:
            end = t[k - 1] if moving[k] else t[k]
            # trim 0.3 s each end: the chassis rocks as it stops
            if end - start >= STILL_MIN_S + 0.6:
                out.append((start + 0.3, end - 0.3))
            start = None
    return out


def window_scan(b, w):
    """One clean scan for a still window, as base_link points.

    Every scan in the window saw the same room from the same place, so the
    per-beam median of their ranges cancels the C1's +-30 mm noise."""
    if b.laser is None:
        raise SystemExit('no base_link -> laser in /tf_static: record with the lidar layer up')
    lx, ly, lyaw = b.laser
    s = [sc for sc in b.scans if w[0] <= sc[0] <= w[1]]
    if not s:
        return None, 0
    n = len(s[0][1])
    same = [sc for sc in s if len(sc[1]) == n]
    R = np.array([sc[1] for sc in same])
    R[~np.isfinite(R) | (R < RANGE_LO) | (R > RANGE_HI)] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)   # beams with no return at all
        r = np.nanmedian(R, axis=0)
    a = same[0][2] + np.arange(n) * same[0][3]
    ok = np.isfinite(r)
    p = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
    c, si = math.cos(lyaw), math.sin(lyaw)
    return p @ np.array([[c, -si], [si, c]]).T + np.array([lx, ly]), len(same)


# ── scan matching ───────────────────────────────────────────────────────────
@dataclass
class Fit:
    x: float
    y: float
    th: float
    residual: float           # median |point-to-line distance|, m
    inliers: int
    eig: np.ndarray           # eigenvalues of the TRANSLATION information, ascending
    cov: np.ndarray           # 3x3 covariance of (x, y, th)
    ok: bool
    why: str = ''


def normals(dst, tree, k=8):
    """Line normals from the k nearest neighbours; flatness = how line-like."""
    _, idx = tree.query(dst, k=min(k, len(dst)))
    nb = dst[idx] - dst[idx].mean(axis=1, keepdims=True)
    C = np.einsum('nki,nkj->nij', nb, nb)
    w, v = np.linalg.eigh(C)
    flat = 1.0 - w[:, 0] / np.maximum(w[:, 1], 1e-12)
    return v[:, :, 0], flat


def icp(src, dst, guess=(0.0, 0.0, 0.0), iters=60):
    """Pose of src's frame in dst's frame: dst ~ R(th) src + (x, y).

    Point-to-line (Censi 2008) with a Huber kernel, so walls pull and a moving
    person does not. The information matrix J'WJ says which directions the
    scene actually pins down: in a plain corridor its smallest eigenvalue
    collapses along the corridor (Zhang, Kaess & Singh 2016), and the fit
    reports itself as degenerate instead of returning a confident number."""
    if src is None or dst is None or len(src) < 60 or len(dst) < 60:
        return Fit(0, 0, 0, 1.0, 0, np.zeros(2), np.eye(3), False, 'too few points')
    tree = cKDTree(dst)
    nrm, flat = normals(dst, tree)
    x, y, th = guess
    gate = 0.30
    H = np.eye(3)
    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        cur = src @ np.array([[c, -s], [s, c]]).T + np.array([x, y])
        d, idx = tree.query(cur)
        keep = (d < gate) & (flat[idx] > 0.8)
        if keep.sum() < 40:
            return Fit(x, y, th, 1.0, int(keep.sum()), np.zeros(2), np.eye(3), False, 'no overlap')
        n = nrm[idx[keep]]
        q = dst[idx[keep]]
        p = cur[keep]
        r = np.einsum('ij,ij->i', n, p - q)
        rel = p - np.array([x, y])
        J = np.stack([n[:, 0], n[:, 1], n[:, 0] * -rel[:, 1] + n[:, 1] * rel[:, 0]], axis=1)
        k = 0.02  # Huber knee, m: the C1's noise
        w = np.where(np.abs(r) < k, 1.0, k / np.maximum(np.abs(r), 1e-9))
        H = J.T @ (J * w[:, None])
        g = J.T @ (w * r)
        try:
            dx = -np.linalg.solve(H + 1e-9 * np.eye(3), g)
        except np.linalg.LinAlgError:
            return Fit(x, y, th, 1.0, int(keep.sum()), np.zeros(2), np.eye(3), False, 'singular')
        x, y, th = x + dx[0], y + dx[1], wrap(th + dx[2])
        gate = max(0.05, gate * 0.8)
        if np.linalg.norm(dx[:2]) < 1e-5 and abs(dx[2]) < 1e-6:
            break
    res = float(np.median(np.abs(r)))
    sigma2 = float(np.sum(w * r * r) / max(1, w.sum() - 3))
    # translation block only: rotation's information grows with range squared,
    # so mixing the two in one eigen-decomposition compares metres with radians
    eig = np.linalg.eigvalsh(H[:2, :2])
    cov = sigma2 * np.linalg.inv(H + 1e-9 * np.eye(3))
    ok, why = True, ''
    if res > 0.02:
        ok, why = False, f'residual {res * 100:.1f} cm'
    elif keep.sum() < 150:
        ok, why = False, f'{int(keep.sum())} inliers'
    elif eig[0] < 20.0:
        ok, why = False, 'degenerate (corridor-like: one direction unconstrained)'
    return Fit(x, y, th, res, int(keep.sum()), eig, cov, ok, why)


# ── self-test: does the truth recover a motion we know? ─────────────────────
def self_test(bag_dir):
    """Two real scans from the same spot (different still moments of a still
    run), one moved by a KNOWN transform; ICP must recover it from a guess
    that is deliberately off. Real sensor noise, real room, known answer."""
    b = read_bag(bag_dir)
    ws = still_windows(b)
    if not ws or ws[0][1] - ws[0][0] < 6:
        raise SystemExit('needs a still run of >= 6 s: ./rover record still --secs 20')
    mid = 0.5 * (ws[0][0] + ws[0][1])
    ref, _ = window_scan(b, (ws[0][0], mid))
    other, _ = window_scan(b, (mid, ws[0][1]))
    cases = [(0.30, 0.00, 0.0), (1.00, 0.20, 90.0), (0.05, -0.03, 5.0), (0.00, 0.00, 180.0),
             (0.50, 0.50, -45.0)]
    bad = 0
    for x, y, deg in cases:
        th = math.radians(deg)
        # the rover moved to (x, y, th): a wall at p (start frame) is seen at
        # R(-th)(p - t) from there -- so `other`, taken at the start, is what
        # the moved rover would see after this change of frame
        c, s = math.cos(th), math.sin(th)
        moved = (other - np.array([x, y])) @ np.array([[c, -s], [s, c]])
        # keep only what a real sensor could see from there
        moved = moved[np.hypot(moved[:, 0], moved[:, 1]) < RANGE_HI]
        guess = (x + 0.05, y - 0.04, th + math.radians(4))     # a deliberately poor start
        f = icp(moved, ref, guess)
        ep = math.hypot(f.x - x, f.y - y) * 100
        eh = math.degrees(wrap(f.th - th))
        good = f.ok and ep < 1.0 and abs(eh) < 0.3
        bad += not good
        print(f"  [{'ok' if good else 'FAIL'}] moved {x:+.2f} {y:+.2f} m {deg:+6.1f} deg -> "
              f"found {f.x:+.3f} {f.y:+.3f} {math.degrees(f.th):+7.2f}   "
              f"err {ep:.2f} cm {eh:+.3f} deg  (res {f.residual * 100:.2f} cm, {f.inliers} pts"
              f"{', ' + f.why if f.why else ''})")
    print(f"  truth self-test: {'PASS' if not bad else f'{bad} FAILED'}")
    return bad == 0
