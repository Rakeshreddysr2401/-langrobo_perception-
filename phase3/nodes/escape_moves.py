"""escape_moves.py -- which short move takes the rover OFF the thing it is parked against.

Pure geometry, no ROS: reach_node's escape() asks best_move() and drives the answer.

Why (twin, apt_s1, 2026-10-19): the rover stopped at an angle with the END of a
27 cm shoe rack (under the LiDAR plane: only the camera's map has it) 1 cm off
its front-left SIDE. The escape then knew only straight moves (+5, +10, -5 cm);
each one slides the side along the rack, the gap never opens, so it said "no
straight move opens space" -- and five goals in a row failed from that spot with
0.0 m driven. A 10-20 deg turn away swings the front-left off the rack at once.

A turn on this skid-steer is NOT about base_link: it pivots about a point P the
rover learns (goal_exec, logs/goal_exec_pivot.json; ~5 cm ahead, 4 cm left for
a left turn), and the real slide varies (2.6-13.6 cm per 60 deg, real robot
2026-10-18). So a turn is checked about the centre, about P and about 2P, and
its WORST sweep counts.
"""
import math

import numpy as np

FRONT, REAR, SIDE = 0.182, 0.178, 0.19          # = goal_exec outline (base_link)
STRAIGHTS = (0.05, 0.10, -0.05)                 # m along the heading (back only a little: blind behind)
TURNS = tuple(math.radians(d) * s for d in (10, 20, 30) for s in (1, -1))
# forward a little, THEN turn: beside a shelf with a wall behind, no single move opens
# space (every turn swings a rear corner back into the wall); a person rolls forward, then turns
COMBOS = tuple((dx, a) for dx in (0.05, 0.10) for a in TURNS)
# STEERING SLIDE, when nothing short opens space (2026-10-19, twin apt_s1): parked
# 1-2 cm along the coffee table's WHOLE side, every turn swings a corner in, short
# moves only slide, and nav2 will not move (its 2 cm hard margin already overlaps;
# at 1 cm its 2.5 cm cells let it graze the sofa by 1.4 cm instead). A person walks
# on along the table, keeping off it, until past its end. A straight slide is no good:
# the rover is never exactly parallel, so the gap closes (measured: refused at 3-5 deg).
# So: forward at a crawl, steering off the nearest point every tick (reach_node
# slide_along), only when nothing is IN the body's way ahead -- a thing beside it
# may converge by up to SLIDE_ROOM, which the steering takes up.
SLIDE_L = 0.40          # m per slide (an escape may repeat it)
SLIDE_ROOM = 0.04       # m: how far a side thing may converge into the path ahead
SLIDE_OPEN = 0.03       # m: done when this much is open (nav2's 2 cm hard margin + 1 cm)


def slide_clear(pts, L=SLIDE_L, room=SLIDE_ROOM):
    """Is the body's way ahead clear for L (nothing within SIDE - room of the centre
    line between the nose and L past it), and is there room to steer on BOTH sides
    of it (a side thing at least `room` off the outline at the start)?"""
    if pts is None or not len(pts):
        return True
    x, y = pts[:, 0], pts[:, 1]
    ahead = (x > FRONT) & (x < FRONT + L) & (np.abs(y) < SIDE - room)
    return not ahead.any()


def slide_steer(pts, tie=0.003):
    """-1, 0 or +1: which way to turn (+ = left) while crawling forward. On the side of
    the nearest point, compare what is nearest the FRONT half (x >= 0, or ahead) with
    what is nearest the REAR half: front closer -> turn away from that side; rear
    closer -> turn toward it (turning swings the rear the other way); within `tie`
    (parallel) -> 0, straight on: either turn would swing one end in."""
    if pts is None or not len(pts):
        return 0
    x, y = pts[:, 0], pts[:, 1]
    ox = np.maximum(np.maximum(-REAR - x, x - FRONT), 0.0)
    d = np.hypot(ox, np.maximum(np.abs(y) - SIDE, 0.0))
    side = 1.0 if y[int(np.argmin(d))] > 0 else -1.0
    on = (y * side) > 0
    front = d[on & (x >= 0)].min() if (on & (x >= 0)).any() else np.inf
    rear = d[on & (x < 0)].min() if (on & (x < 0)).any() else np.inf
    if abs(front - rear) < tie:
        return 0
    return int(-side if front < rear else side)


SAMPLES = 6                                     # poses checked along each move
OPENS = 0.01                                    # a move must open at least this much
SWEEP_SLACK = 0.002                             # ... and never close the gap by more than this on the way


def body_gap(pts, front=FRONT, rear=REAR, side=SIDE):
    """Signed distance from the outline to the nearest point (base_link N x 2); negative = inside."""
    if pts is None or not len(pts):
        return float('inf')
    x, y = pts[:, 0], np.abs(pts[:, 1])
    ox = np.maximum(np.maximum(-rear - x, x - front), 0.0)
    oy = np.maximum(y - side, 0.0)
    out = np.hypot(ox, oy)
    inside = (ox == 0) & (oy == 0)
    depth = np.minimum(np.minimum(front - x, x + rear), side - y)
    return float(np.min(np.where(inside, -depth, out)))


def moved(pts, dx=0.0, a=0.0, P=(0.0, 0.0)):
    """The points as seen from the body after driving dx straight, THEN turning a about P."""
    if pts is None or not len(pts):
        return pts
    q = pts - np.array([dx, 0.0])
    if abs(a) < 1e-12:
        return q
    c, s = math.cos(a), math.sin(a)
    P = np.asarray(P, dtype=float)
    t = P - np.array([c * P[0] - s * P[1], s * P[0] + c * P[1]])     # base moves by (I - R(a)) P
    q = q - t
    return np.stack([c * q[:, 0] + s * q[:, 1], -s * q[:, 0] + c * q[:, 1]], 1)


def path_gaps(gap_of, kind, v, pivots):
    """gap_of(dx=, a=, P=) at SAMPLES poses along the move; for a turn the worst pivot at each."""
    fr = np.linspace(1.0 / SAMPLES, 1.0, SAMPLES)
    if kind == 'drive':
        return [gap_of(dx=f * v) for f in fr]
    dx, a = (0.0, v) if kind == 'turn' else v
    out = [gap_of(dx=f * dx) for f in fr] if dx else []
    P = pivots[1 if a > 0 else -1]
    variants = (np.zeros(2), np.asarray(P, float), 2 * np.asarray(P, float))
    return out + [min(gap_of(dx=dx, a=f * a, P=Pv) for Pv in variants) for f in fr]


def best_move(gap_of, g0, pivots):
    """-> (kind 'drive'|'turn'|'drive+turn', value m|rad|(m, rad), end gap) or None.
    gap_of(dx=0, a=0, P=(0,0)) -> the gap with the body moved so. A move counts if its
    whole sweep keeps today's gap (less SWEEP_SLACK) and it ends OPENS wider; the
    widest end wins, the simpler move on a tie (straight, then a turn, then both)."""
    best = None
    for kind, vals in (('drive', STRAIGHTS), ('turn', TURNS), ('drive+turn', COMBOS)):
        for v in vals:
            path = path_gaps(gap_of, kind, v, pivots)
            sweep, end = min(path), path[-1]
            if sweep >= g0 - SWEEP_SLACK and end >= g0 + OPENS and (best is None or end > best[2] + 0.005):
                best = (kind, v, end)
    return best
