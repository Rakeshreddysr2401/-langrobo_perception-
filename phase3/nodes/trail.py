"""trail.py — the way the rover came: driven poses, and the way back along them.

No ROS: reach_node feeds it poses and asks it for waypoints; test_trail.py
runs it offline.

WHY (2026-10-04, OPEN_ISSUES S10). Asked to go to someone back through the
doorway it had just driven in by, the rover found the corridor closed in its
map -- stale depth marks of the people who had walked through it (nvblox
never forgets, by design) -- and nav2 planned a detour through unmapped floor
into a dead end behind a cot. It retried from there four times. The corridor
it had driven minutes earlier was the one route known to be passable, and
nothing used it. Now, on "no path", reach looks down the trail (the camera
clears a stale mark only by seeing past it) and drives back along it in short
hops, every hop through nav2's normal checks -- the trail picks the ROUTE, it
never switches a collision check off.

A trail point is a place the rover's centre has been. It says nothing about
what has moved in since, which is why each hop is still planned and checked.
"""
import math

SPACING = 0.15        # m between kept points
MAX_POINTS = 2000     # ~300 m of driving
JUMP_M = 0.5          # a sample this far from the last trail point was not
                      # driven to (a pose reset, or the rover carried while reach
                      # was idle) -- start a new trail
HOP_M = 1.0           # m between way-back waypoints
MIN_AHEAD_M = 0.6     # a target this close to the rover is no way back
MIN_GAIN_M = 0.5      # the trail must lead at least this much nearer the goal


class Trail:
    def __init__(self):
        self.pts = []                      # [(x, y)], oldest first

    def add(self, x, y):
        """Feed pose samples while reach drives (10+ Hz, so consecutive
        samples are cm apart and only a carry or a reset can jump)."""
        if self.pts:
            d = math.hypot(x - self.pts[-1][0], y - self.pts[-1][1])
            if d > JUMP_M:
                self.pts = []
            elif d < SPACING:
                return
        self.pts.append((x, y))
        del self.pts[:-MAX_POINTS]

    def way_back(self, pose, goal):
        """Waypoints from near the rover, along the trail, to the trail point
        nearest the goal -- or [] when the trail leads nowhere nearer.

        pose, goal: (x, y, ...). Waypoints are (x, y, heading along the trail)."""
        if len(self.pts) < 2:
            return []
        px, py = pose[0], pose[1]
        here = min(range(len(self.pts)), key=lambda i: _d(self.pts[i], (px, py)))
        if _d(self.pts[here], (px, py)) > JUMP_M:
            return []                      # the rover is not on its trail
        best = min(range(len(self.pts)), key=lambda i: _d(self.pts[i], goal))
        if _d(self.pts[best], (px, py)) < MIN_AHEAD_M:
            return []
        if _d((px, py), goal) - _d(self.pts[best], goal) < MIN_GAIN_M:
            return []
        step = 1 if best > here else -1
        path = [self.pts[i] for i in range(here, best + step, step)]
        hops, run = [], 0.0
        for a, b in zip(path, path[1:]):
            run += _d(a, b)
            if run >= HOP_M:
                hops.append(b)
                run = 0.0
        if not hops or hops[-1] != path[-1]:
            hops.append(path[-1])
        out, prev = [], (px, py)
        for h in hops:
            out.append((h[0], h[1], math.atan2(h[1] - prev[1], h[0] - prev[0])))
            prev = h
        return out


def _d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def same_failure(prev, why, pose, near_m=0.3):
    """prev: (why, (x, y)) of the last failed attempt, or None. True when this
    one failed the same way from (nearly) the same spot -- retrying it again
    is the same move again (the 2026-10-04 drive: four 'no path' from one
    spot, 316 s)."""
    if prev is None:
        return False
    return _kind(prev[0]) == _kind(why) and _d(prev[1], pose) <= near_m


def _kind(why):
    return (why or '').split(':')[0].strip()


def way_back_note(why):
    """For the brain's report: why the way the rover came did not work."""
    if 'no path' in why:
        # nothing the LiDAR or a fresh look confirmed: depth-map leftovers
        return ('the way I came is marked blocked on my map, and looking at it did not '
                'clear it -- likely marks left by people walking through (my depth map '
                'keeps what it saw); a person may need to check')
    if why.startswith('narrow') or 'stuck' in why:
        return f'the way I came is too tight to drive back right now ({why})'
    return f'I could not drive back the way I came ({why})'
