#!/usr/bin/env python3
"""scan_tools.py — scan helpers shared by the LiDAR tools that still run.

Kept from lidar/pivot_test.py when that test was retired (2026-09-26; the
harness grades turns now): ./rover drive (phase3/nodes/pivot_goto.py) and
./rover lidar --lag (scan_lag.py) use these.

    turn_twist(wz)                 the turn command in the PIVOT_ANCHOR mode
    scan_in_base(scan, lx, yaw)    a scan as base_link points, the rover itself removed
    icp(src, dst, theta0)          rigid 2D point-to-point fit
    CORNERS                        the measured outline (nav2.yaml's footprint;
                                   description/build keeps it honest)
"""
import math
import os

import numpy as np
from geometry_msgs.msg import Twist
from scipy.spatial import cKDTree

# 1.5, not 0.5: TODO 43 measured pivots reaching ~7% of ANY commanded rate --
# the firmware's PI saturates to full duty and the motors stall against scrub.
# A low command does not give a gentler pivot, only a slower one.
WZ = float(os.environ.get("PIVOT_WZ", "1.5"))
# Deadlines assume the rate the rover ACTUALLY reaches, not the commanded one.
# The first version budgeted |angle|/WZ and every turn "stalled" at 18 s while
# still turning. Abort instead on NO PROGRESS for STALL_S.
MIN_RATE = 0.03          # rad/s, the slowest turn still worth waiting for
STALL_S = 8.0
# The spin sweeps a circle of this radius: the furthest corner of the measured
# 36 x 38 envelope (nav2.yaml's footprint) plus 5 cm. Anything the lidar sees
# inside it gets hit.
CORNERS = np.array([[0.182, 0.19], [0.182, -0.19], [-0.178, -0.19], [-0.178, 0.19]])
SWEEP_R = float(np.max(np.linalg.norm(CORNERS, axis=1))) + 0.05

# LEFT-ANCHORED TURNS (PIVOT_ANCHOR=left). Commanded as a pure wz, this rover
# picks between two behaviours unpredictably: usually the left wheels stay
# planted and it rotates about the left tyres, sometimes the left side joins in
# and it rotates about the centre (2026-09-23: 7 of 8, then a -90 that pivoted
# near the centre and landed a planned turn 35 cm off). A planner cannot plan
# around a coin flip. The firmware mixes wL = vx - wz*B/2 and switches a side
# OFF (duty 0, braked) when its target is under 0.01 m/s, so commanding
# vx = wz*B/2 makes the left target exactly zero and removes the choice: the
# right side swings the rover about the planted left tyres every time. B is the
# FIRMWARE's WHEEL_BASE_M, not the physical track -- it has to cancel the
# firmware's own arithmetic.
FW_HALF_B = 0.17
ANCHOR = os.environ.get("PIVOT_ANCHOR", "none").strip().lower()


# "left" -- left side OFF -- was tried first and is WRONG: an unpowered left
# side does not stay planted, it free-rolls behind the right. Measured
# 2026-09-23: the rover swung about a point 75 cm to its left, ~60 cm of slide
# per 45 deg. Kept only so that result can be reproduced.
#
# "hold" is the fix: give the left side a small target, HOLD_V against the turn
# direction, so the firmware's PI actively holds it near zero. It cannot then
# free-roll forward (the "left" failure), and it cannot reverse at full speed
# either (the occasional centre-pivot that made pure wz a coin flip). It also
# stops the left motor sitting at full duty against a 0.255 m/s target it
# cannot reach, which is what a pure-wz turn asks of it for seconds at a time.
HOLD_V = 0.02


def turn_twist(wz):
    """Twist for a turn at wz, in the mode ANCHOR selects."""
    t = Twist()
    t.angular.z = wz
    if ANCHOR == "left":
        t.linear.x = wz * FW_HALF_B
    elif ANCHOR == "hold":
        # wL = vx - wz*B/2 = -sign(wz) * HOLD_V
        t.linear.x = wz * FW_HALF_B - math.copysign(HOLD_V, wz)
    return t

RANGE_LO, RANGE_HI = 0.20, 6.0
SETTLE = 1.5


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def scan_in_base(scan, lx, lz_yaw):
    """Scan points expressed in base_link, using the calibrated mount."""
    r = np.asarray(scan.ranges, dtype=np.float64)
    a = scan.angle_min + np.arange(r.size) * scan.angle_increment
    ok = np.isfinite(r) & (r > RANGE_LO) & (r < RANGE_HI)
    q = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
    c, s = math.cos(lz_yaw), math.sin(lz_yaw)
    R = np.array([[c, -s], [s, c]])
    q = q @ R.T + np.array([lx, 0.0])
    # Self-filter: returns inside the rover's own outline (+2 cm) are the rover
    # (since 2026-09-25, something at the rear-right corner of the frame).
    own = (q[:, 0] < CORNERS[:, 0].max() + 0.02) & (q[:, 0] > CORNERS[:, 0].min() - 0.02) \
        & (np.abs(q[:, 1]) < CORNERS[:, 1].max() + 0.02)
    return q[~own]


def icp(src, dst, theta0, iters=60):
    """Rigid 2D fit: dst ~ R(theta) src + t, started at theta0."""
    tree = cKDTree(dst)
    th, t = theta0, np.zeros(2)
    used = 0
    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s], [s, c]])
        cur = src @ R.T + t
        d, idx = tree.query(cur)
        keep = d < max(0.20, np.percentile(d, 70))
        used = int(keep.sum())
        if used < 40:
            return None
        P, Q = src[keep], dst[idx[keep]]
        pc, qc = P.mean(axis=0), Q.mean(axis=0)
        H = (P - pc).T @ (Q - qc)
        U, _, Vt = np.linalg.svd(H)
        D = np.diag([1.0, np.linalg.det(Vt.T @ U.T)])
        Rn = Vt.T @ D @ U.T
        th_n = math.atan2(Rn[1, 0], Rn[0, 0])
        t_n = qc - Rn @ pc
        if abs(wrap(th_n - th)) < 1e-6 and np.linalg.norm(t_n - t) < 1e-6:
            th, t = th_n, t_n
            break
        th, t = th_n, t_n
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    d, _ = tree.query(src @ R.T + t)
    return th, t, float(np.median(d)), used
