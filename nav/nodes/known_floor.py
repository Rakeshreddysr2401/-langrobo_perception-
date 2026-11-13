"""known_floor.py -- the floor the rover KNOWS: what its camera has looked at, and where its body has
stood. No ROS. kino_0.1.0.

WHY (kino_sim rl2 r003, 2026-11-09): nothing on the rover looks back -- the LiDAR's plane is above
low things, the camera and the sonars face forward. Once the camera-channel bugs in training were
fixed, every contact left was the rover BACKING into a low object it had never seen. The shield
(rl_drive.sweep_hits, known=...) now lets a reverse move the outline only over known floor; the
training env (kino_sim rl2/env.py, REVERSE_KNOWN) does the same with the same geometry.

A 4 cm grid in odom, 40 x 40 m about where reach started. Marked from each pose (mark(), ~10 Hz):
the camera's floor fan (+-43.5 deg, 0.25-3.0 m, as rl2/env.py CAM_*) and the body's cells.
"""
import math

import numpy as np

RES = 0.04
HALF = 20.0
N = int(2 * HALF / RES)
CAM_HALF, CAM_NEAR, CAM_FAR = math.radians(43.5), 0.25, 3.0
FRONT, REAR, SIDE = 0.182, 0.178, 0.19          # rl2/env.py (goal_exec's outline)
# 48 rays x 3 cm (kino_sim rl2/env.py FAN_N, FAN_DR): 24 x 6 cm left holes in the 4 cm grid, beside the
# rover's own way in too -- the reverse rule then refused backing out of dead ends (r008, 2026-11-10)
_fa, _fr = np.meshgrid(np.linspace(-CAM_HALF, CAM_HALF, 48), np.arange(CAM_NEAR, CAM_FAR, 0.03), indexing='ij')
FAN_A, FAN_R = _fa.ravel(), _fr.ravel()
_bu, _bv = np.meshgrid(np.arange(-REAR + 0.01, FRONT, 0.02), np.arange(-SIDE + 0.01, SIDE, 0.02), indexing='ij')
BODY_U, BODY_V = _bu.ravel(), _bv.ravel()


class KnownFloor:
    def __init__(self):
        self.grid = np.zeros((N, N), bool)
        self.origin = None                           # odom (x, y) of the grid's centre

    def _ij(self, x, y):
        i = np.floor((np.asarray(x) - self.origin[0] + HALF) / RES).astype(int)
        j = np.floor((np.asarray(y) - self.origin[1] + HALF) / RES).astype(int)
        return i, j

    def mark(self, pose):
        """The camera fan and the body's cells at pose (x, y, th) in odom."""
        x, y, th = pose[:3]
        if self.origin is None:
            self.origin = (x, y)
        a = th + FAN_A
        px = np.concatenate([x + np.cos(a) * FAN_R, x + math.cos(th) * BODY_U - math.sin(th) * BODY_V])
        py = np.concatenate([y + np.sin(a) * FAN_R, y + math.sin(th) * BODY_U + math.cos(th) * BODY_V])
        i, j = self._ij(px, py)
        ok = (i >= 0) & (i < N) & (j >= 0) & (j < N)
        self.grid[i[ok], j[ok]] = True

    def mark_free_grid(self, grid, res, origin, pose, r_max=3.0):
        """The cells a camera map (nvblox, via ghost_clear: 0 = observed free, 100 = occupied,
        -1 = never seen) has OBSERVED free, within r_max of pose, eroded by one cell so no cell
        touching something unseen or occupied counts.

        WHY (real robot 2026-11-10, rl_dumps 161517-162059): reach was restarted at 16:13 UTC to
        deploy a fix, and this memory restarted empty. 2-8 minutes later four rl2 rounds ended
        pinned 0.4-1.4 cm from things: the policy asked to back out in 80-100 % of its steps, and
        the reverse rule refused every reverse -- the floor behind, driven over minutes before,
        was "unknown" to the new process. The camera map lives in the mapping layer and survives
        a reach restart; rebuilt from the bag, it knew the floor behind at all three moments tried."""
        from scipy import ndimage
        g = np.asarray(grid)
        free = ndimage.binary_erosion(g == 0, structure=np.ones((3, 3), bool))
        ii, jj = np.nonzero(free)
        if not len(ii):
            return 0
        x = origin[0] + (jj + 0.5) * res
        y = origin[1] + (ii + 0.5) * res
        near = np.hypot(x - pose[0], y - pose[1]) <= r_max
        x, y = x[near], y[near]
        if self.origin is None:
            self.origin = (pose[0], pose[1])
        i, j = self._ij(x, y)
        ok = (i >= 0) & (i < N) & (j >= 0) & (j < N)
        self.grid[i[ok], j[ok]] = True
        return int(ok.sum())

    def known_base(self, pose):
        """f(bx, by) -> bool array: are these base_link points (at pose) over known floor?"""
        x, y, th = pose[:3]
        c, s = math.cos(th), math.sin(th)

        def f(bx, by):
            if self.origin is None:
                return np.zeros(np.shape(bx), bool)
            i, j = self._ij(x + c * bx - s * by, y + s * bx + c * by)
            ok = (i >= 0) & (i < N) & (j >= 0) & (j < N)
            out = np.zeros(np.shape(bx), bool)
            out[ok] = self.grid[i[ok], j[ok]]
            return out
        return f
