"""person_tracker.py — the same person keeps the same id from frame to frame.

No ROS: detections_3d feeds it the people it found in one frame (odom-frame
floor positions) and publishes what it returns; test_person_tracker.py runs it
offline.

WHY (2026-10-10). "Mitra, follow me" needs to know which of the people in view
is ME, frame after frame -- not just "a person". detections_3d smoothed object
positions per LABEL, so two people in frame shared one track and fought, and a
follower would swap to whoever walked past. Ultralytics' ByteTrack needs the
`lap` package, which the orin-nav image does not carry (and the image is never
modified), so this is a small tracker of our own.

How a frame is matched:
  - every track is moved to where its velocity says it should be now,
  - each detection is matched to the nearest predicted track on the FLOOR
    (odom x, y) -- not by box overlap, because boxes jump sideways every time
    the rover turns, while a person's floor position does not,
  - one detection per track and one track per detection (scipy's assignment,
    or a greedy pass where scipy is missing), and nothing further than GATE_M,
  - ByteTrack's one good idea is kept: a weak detection (conf below
    NEW_TRACK_CONF) can continue a track but never starts one, so a person
    half hidden behind a chair keeps their id, and a coat on a hook does not
    become a new person.

A track lost from view is kept for MAX_LOST_S with its last position, so a
person stepping behind something for a moment comes back with the same id.
A track is reported once it has been seen MIN_HITS times, so one stray box is
never somebody to follow.
"""
import math

try:
    from scipy.optimize import linear_sum_assignment
except ImportError:            # dev box without scipy: greedy is fine for a few people
    linear_sum_assignment = None

GATE_M = 0.8           # a detection further than this from a track's prediction is someone else
MAX_LOST_S = 1.5       # keep a track this long after it was last seen
MIN_HITS = 2           # frames before a track is reported
NEW_TRACK_CONF = 0.35  # weaker detections only continue existing tracks
POS_ALPHA = 0.6        # weight of a new sample in the position average
VEL_ALPHA = 0.4        # weight of a new sample in the velocity average
MAX_SPEED = 2.5        # m/s; a faster jump is a depth glitch, not a person
MAX_PREDICT_S = 0.5    # never coast a lost track further than this on velocity


class Track:
    __slots__ = ("id", "x", "y", "vx", "vy", "t", "hits", "conf", "extra")

    def __init__(self, tid, det, t):
        self.id = tid
        self.x, self.y = det["x"], det["y"]
        self.vx = self.vy = 0.0
        self.t = t
        self.hits = 1
        self.conf = det["conf"]
        self.extra = det

    def predict(self, t):
        dt = min(max(t - self.t, 0.0), MAX_PREDICT_S)
        return self.x + self.vx * dt, self.y + self.vy * dt

    def update(self, det, t):
        dt = t - self.t
        nx = POS_ALPHA * det["x"] + (1 - POS_ALPHA) * self.x
        ny = POS_ALPHA * det["y"] + (1 - POS_ALPHA) * self.y
        if dt > 1e-3:
            vx, vy = (nx - self.x) / dt, (ny - self.y) / dt
            speed = math.hypot(vx, vy)
            if speed > MAX_SPEED:
                vx, vy = vx * MAX_SPEED / speed, vy * MAX_SPEED / speed
            self.vx = VEL_ALPHA * vx + (1 - VEL_ALPHA) * self.vx
            self.vy = VEL_ALPHA * vy + (1 - VEL_ALPHA) * self.vy
        self.x, self.y = nx, ny
        self.t = t
        self.hits += 1
        self.conf = det["conf"]
        self.extra = det


def _assign(cost, gate):
    """(track_index, detection_index) pairs with cost <= gate, each used once."""
    if not cost or not cost[0]:
        return []
    if linear_sum_assignment is not None:
        import numpy as np
        rows, cols = linear_sum_assignment(np.array(cost))
        pairs = zip(rows.tolist(), cols.tolist())
    else:
        flat = sorted((c, i, j) for i, row in enumerate(cost) for j, c in enumerate(row))
        used_i, used_j, pairs = set(), set(), []
        for c, i, j in flat:
            if i not in used_i and j not in used_j:
                used_i.add(i)
                used_j.add(j)
                pairs.append((i, j))
    return [(i, j) for i, j in pairs if cost[i][j] <= gate]


class PersonTracker:
    def __init__(self):
        self._tracks = []
        self._next_id = 1

    def update(self, dets, t):
        """dets: [{"x", "y", "conf", ...}] in odom, from ONE frame taken at t (s).

        Returns the reported people, each the latest detection dict plus
        "id", smoothed "x"/"y", "vx"/"vy" and "lost_s" (0 = seen this frame).
        """
        preds = [tr.predict(t) for tr in self._tracks]
        cost = [[math.hypot(px - d["x"], py - d["y"]) for d in dets] for px, py in preds]
        matched_t, matched_d = set(), set()
        for i, j in _assign(cost, GATE_M):
            self._tracks[i].update(dets[j], t)
            matched_t.add(i)
            matched_d.add(j)

        self._tracks = [tr for i, tr in enumerate(self._tracks)
                        if i in matched_t or t - tr.t <= MAX_LOST_S]
        for j, d in enumerate(dets):
            if j not in matched_d and d["conf"] >= NEW_TRACK_CONF:
                self._tracks.append(Track(self._next_id, d, t))
                self._next_id += 1

        out = []
        for tr in self._tracks:
            if tr.hits < MIN_HITS:
                continue
            p = dict(tr.extra)
            p.update(id=tr.id, x=round(tr.x, 3), y=round(tr.y, 3),
                     vx=round(tr.vx, 2), vy=round(tr.vy, 2),
                     conf=round(tr.conf, 2), lost_s=round(max(t - tr.t, 0.0), 2))
            out.append(p)
        return out
