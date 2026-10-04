"""ghost_clear.py — erase the camera's marks of things that have moved away.

No ROS: ghost_clear_node.py feeds it nvblox's grid, LiDAR scans and the pose;
phase2/tools/test_ghost_clear.py runs it offline.

WHY (2026-10-04, OPEN_ISSUES S10). nvblox never forgets (nvblox.launch.py,
"DO NOT LET THE MAP FORGET": a forgetting map loses the shoe the camera saw
before the rover turned away). So everyone who walks past leaves their legs
in the map. That day, people in a doorway closed the corridor the rover had
driven in by, and later the rover sat boxed in by marks under its own body
while the LiDAR's nearest return was 31 cm away. Only `./rover map` (a new
map) freed it.

THE RULE (owner's choice, 2026-10-04). The LiDAR is one plane at ~25 cm, all
round, fresh every sweep; the camera sees 3-27 cm, ahead only, and remembers.

  tall     a camera mark the LiDAR has also hit (within HIT_R, in the last
           HIT_KEEP_S): something that reaches the LiDAR plane -- a person,
           a chair. A shoe never gets here: the LiDAR passes over it.
  cleared  a tall mark the LiDAR now sees THROUGH (a beam passes the cell
           and returns from beyond it): it has moved. Erased from the grid
           nav2 and goal_exec read.
  body     a mark under the rover's own outline: the rover is standing there.
           Erased too.

An erased mark comes back when the LiDAR hits it again, or when the camera
has had it in view for VIEW_S while still marked -- nvblox clears a cell it
sees past, so a mark that stays in plain view is real. The one case the rule
gets wrong, until the camera sees it: a low thing put down exactly where a
person stood, or where the rover itself stood.
Beams stop FREE_MARGIN short of what they hit, more than HIT_R, so a low
thing against a wall or a chair leg is never seen "through".

Cells are keyed by their global index (x / res, y / res), so the state
survives nvblox growing its grid.
"""
import math

import numpy as np

HIT_R_CELLS = 2        # 5 cm at 2.5 cm: a LiDAR hit tags camera marks this near as tall
HIT_KEEP_S = 8.0       # s a LiDAR hit counts for tagging (nvblox marks lag the scan)
BACK_S = 1.0           # s: a cell hit this recently is not seen through
FREE_FROM = 0.25       # m from the laser: the first cell a beam vouches for
FREE_MARGIN = 0.12     # m short of the hit: the last cell a beam vouches for
FREE_MAX = 4.0         # m: beyond this a beam vouches for nothing
VIEW_S = 3.0           # s in the camera's view, still marked = real
VIEW_HALF = math.radians(40.0)   # the D555's ~87 deg, a little inside
VIEW_NEAR, VIEW_FAR = 0.5, 2.5   # m from the camera: where nvblox clears what it sees past
CAM_FWD = 0.15         # m: the camera ahead of base_link
# the rover's outline (fused_obstacles OWN_*), minus a cell: under it = stale
BODY_FRONT, BODY_REAR, BODY_SIDE = 0.18, 0.17, 0.18

_SHIFT = 1 << 20       # key = ix * _SHIFT + iy, ix/iy offset to stay positive
_OFF = 1 << 19


def keys_of(ix, iy):
    return (np.asarray(ix, np.int64) + _OFF) * _SHIFT + (np.asarray(iy, np.int64) + _OFF)


class GhostMap:
    def __init__(self, res=0.025):
        self.res = res
        self.hits = []                      # [(t, keys)] recent LiDAR hit cells
        self.tall = set()
        self.cleared = set()
        self.view_s = {}                    # cleared key -> seconds in view, still marked
        self.occ = np.zeros(0, np.int64)    # keys nvblox marks now
        self._dil = [(dx, dy) for dx in range(-HIT_R_CELLS, HIT_R_CELLS + 1)
                     for dy in range(-HIT_R_CELLS, HIT_R_CELLS + 1)
                     if dx * dx + dy * dy <= HIT_R_CELLS * HIT_R_CELLS]

    # ── inputs ───────────────────────────────────────────────────────────────
    def on_scan(self, laser_xy, hits_xy, now):
        """laser_xy (2,), hits_xy (N, 2): the laser and its returns, in odom."""
        r = self.res
        hits_xy = np.asarray(hits_xy, float).reshape(-1, 2)
        hk = np.unique(keys_of(np.floor(hits_xy[:, 0] / r), np.floor(hits_xy[:, 1] / r)))
        self.hits = [(t, k) for t, k in self.hits if now - t <= HIT_KEEP_S] + [(now, hk)]
        # something is back where a mark was erased: it is real again
        if self.cleared and hk.size:
            back = set(self._dilate(hk).tolist()) & self.cleared
            self.cleared -= back
            for k in back:
                self.view_s.pop(k, None)
        # seen through: every cell a beam crossed, short of what it hit
        if not self.tall or not hits_xy.size:
            return
        # Per tall cell, not per beam: the beam pointing at the cell, and
        # whether it came back from beyond it. A few hundred tall cells, not
        # 720 beams x 300 steps (that cost 37% of a core, 2026-10-04).
        lx, ly = float(laser_xy[0]), float(laser_xy[1])
        d = hits_xy - (lx, ly)
        b_ang = np.arctan2(d[:, 1], d[:, 0])
        b_rng = np.hypot(d[:, 0], d[:, 1])
        order = np.argsort(b_ang)
        b_ang, b_rng = b_ang[order], b_rng[order]
        spacing = np.median(np.diff(b_ang)) if b_ang.size > 2 else 0.0
        tall = np.fromiter(self.tall, np.int64, len(self.tall))
        ix, iy = self._unkey(tall)
        cx, cy = (ix + 0.5) * r - lx, (iy + 0.5) * r - ly
        c_ang, c_rng = np.arctan2(cy, cx), np.hypot(cx, cy)
        j = np.clip(np.searchsorted(b_ang, c_ang), 1, b_ang.size - 1)
        j = np.where(np.abs(b_ang[j - 1] - c_ang) < np.abs(b_ang[j] - c_ang), j - 1, j)
        aimed = np.abs(b_ang[j] - c_ang) <= max(spacing, 1e-6)            # a beam really points at it
        beyond = b_rng[j] - FREE_MARGIN >= c_rng                        # ...and returned from past it
        near = (c_rng >= FREE_FROM) & (c_rng <= FREE_MAX)               # where a beam vouches at all
        recent = self._recent_hits(BACK_S, now)
        through = tall[aimed & beyond & near & ~np.isin(tall, recent)]
        self.cleared.update(through.tolist())

    def on_grid(self, occ_xy_idx, now):
        """occ_xy_idx (N, 2) int: the global cell indices nvblox marks now.
        Tags the tall ones; forgets state for cells nvblox no longer marks."""
        occ = keys_of(occ_xy_idx[:, 0], occ_xy_idx[:, 1]) if len(occ_xy_idx) else np.zeros(0, np.int64)
        self.occ = occ
        hits = self._recent_hits(HIT_KEEP_S, now)
        if occ.size and hits.size:
            self.tall.update(occ[np.isin(occ, self._dilate(hits))].tolist())
        # nvblox let go of a cell itself: nothing left to keep about it
        live = set(occ.tolist())
        self.tall &= live
        gone = self.cleared - live
        self.cleared -= gone
        for k in gone:
            self.view_s.pop(k, None)

    def on_pose(self, pose, camera_live, dt):
        """pose (x, y, yaw) in odom. camera_live: nvblox is being fed depth
        right now (the turn gate passes frames only while not turning)."""
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        r = self.res
        # under the body
        if self.occ.size:
            ix, iy = self._unkey(self.occ)
            cx, cy = (ix + 0.5) * r - x, (iy + 0.5) * r - y
            fwd, side = c * cx + s * cy, -s * cx + c * cy
            under = (fwd < BODY_FRONT) & (fwd > -BODY_REAR) & (np.abs(side) < BODY_SIDE)
            self.cleared.update(self.occ[under].tolist())
        # in plain view and still marked: real
        if not camera_live or not self.cleared:
            return
        ck = np.fromiter(self.cleared, np.int64, len(self.cleared))
        ix, iy = self._unkey(ck)
        camx, camy = x + CAM_FWD * c, y + CAM_FWD * s
        dx, dy = (ix + 0.5) * r - camx, (iy + 0.5) * r - camy
        dist = np.hypot(dx, dy)
        bearing = np.abs(np.arctan2(np.sin(np.arctan2(dy, dx) - th), np.cos(np.arctan2(dy, dx) - th)))
        seen = ck[(dist >= VIEW_NEAR) & (dist <= VIEW_FAR) & (bearing <= VIEW_HALF)]
        for k in seen.tolist():
            self.view_s[k] = self.view_s.get(k, 0.0) + dt
            if self.view_s[k] >= VIEW_S:
                self.cleared.discard(k)
                self.tall.discard(k)       # the LiDAR passes over it: low, keep it
                del self.view_s[k]

    # ── output ───────────────────────────────────────────────────────────────
    def erase_mask(self, keys):
        """Bool per key: True = erase this mark from the published grid."""
        if not self.cleared:
            return np.zeros(len(keys), bool)
        return np.isin(keys, np.fromiter(self.cleared, np.int64, len(self.cleared)))

    # ── helpers ──────────────────────────────────────────────────────────────
    def _recent_hits(self, within, now):
        ks = [k for t, k in self.hits if now - t <= within]
        return np.unique(np.concatenate(ks)) if ks else np.zeros(0, np.int64)

    def _dilate(self, keys):
        ix, iy = self._unkey(keys)
        return np.unique(np.concatenate([keys_of(ix + dx, iy + dy) for dx, dy in self._dil]))

    @staticmethod
    def _unkey(keys):
        keys = np.asarray(keys, np.int64)
        return keys // _SHIFT - _OFF, keys % _SHIFT - _OFF
