"""tight_route.py — no route on the map: find turns and short drives that fit.

No ROS. reach_node.py calls plan() when nav2 says "no path"; goal_exec drives
each step and checks it again on fresh points; reach re-plans after every
step from where the rover really is. phase3/tools/test_tight_route.py runs
it offline on the points measured at the door.

WHY (2026-10-10, the bedroom door, owner watching). "Go outside": the door
leaf stood open toward the rover, 30 cm ahead; the gap past its free edge was
77 cm (rover 38). nav2 said NO_VALID_PATH (208) for every goal through it,
and at footprint_padding 0.01 still only offered a 5.3 m loop: its lattice
turns on a 0.4 m radius and pays 5x for turning in place, and the rover's
left side was 5 cm from a wall corner. reach then did what it knows -- the
way back, nav2 again -- and gave up: "no path ... 3 times from the same
spot". The owner: "it can take left rotation and move". It could: this
search, on goal_exec's own checks, found forward 10 cm, left 45, forward
1.6 m in 0.4 s; driven through goal_exec step by step it went out of the
door, every step within 1.1 cm. Later, "come to me" from the far side failed
the same way after 188 s without one move toward the owner.

HOW
    A* over the rover's pose. Moves are what goal_exec does exactly: a turn
    in place (about the LEARNED pivot, so the slide is in the plan) or a
    straight drive. Each is checked with goal_exec's own turn_clear /
    drive_clear on the points -- the same test goal_exec will run before
    moving -- but at PLAN_MARGIN (2 cm), one more than goal_exec's 1 cm, so a
    plan still fits after a centimetre of slide. Points are those reach and
    goal_exec decide on (LiDAR + the fused camera map), not nav2's costmap.
    Cost: metres driven, reverse x3 (blind for low things behind), and each
    turn a fixed price plus its angle, so few, small turns win.

    Done = within GOAL_TOL of the target. The target is the goal when it is
    within REACH_M, else the farthest point toward it that fits (reach_node).

WHAT IT WILL NOT DO
    Plan through what it has not seen: points only exist where something
    was seen, so the search stays within SEARCH_R of the start, where the
    LiDAR (all round) and the camera (ahead) have looked. Back up more than
    BACK_MAX in one step. Take longer than BUDGET_S: then it answers None,
    and reach goes on with its other recoveries.
"""
import heapq
import math
import time

import numpy as np

from goal_exec import FRONT, MARGIN, REAR, SIDE, R, wrap

PLAN_MARGIN = 0.02            # m: goal_exec drives at 1 cm; the plan keeps 2
DRIVES = (0.05, 0.10, 0.20, 0.35, 0.50, 1.00, -0.05, -0.10)   # m, + is forward
BACK_MAX = 0.10               # m: the largest reverse (blind below the LiDAR behind)
TURNS = tuple(math.radians(a) for a in (15, 30, 45, 90, -15, -30, -45, -90))
TURN_COST = 0.30              # per turn, in metres of driving: a turn is ~6 s with the settle, and slides
TURN_COST_RAD = 0.15          # per radian turned
REVERSE_X = 3.0
GOAL_TOL = 0.12               # m
SEARCH_R = 2.6                # m from the start: beyond, nothing has been looked at
BUDGET_S = 3.0                # s of search, then give up (None)
# Search size, the SAME on every computer. It was 40000, so the time budget
# decided instead -- and the laptop (~8x the Orin) found plans the robot never
# does: a 13-move detour round a 7 cm floor speck (B20 L45 B5 L90 ...) where the
# Orin times out and takes the LiDAR's plan, so twin unit tests did not show the
# robot's behaviour (test_reach_tight failed on the laptop, 2026-10-20). The
# largest plan the real robot found that day: 3523 poses (5.6 s on the Orin).
# A dead end now also answers sooner ("wall": 965 ms, not the whole budget).
MAX_EXPAND = 6000
KEY_XY, KEY_TH = 0.04, math.radians(15)
H_WEIGHT = 1.3              # >1: greedier toward the target -- any fitting way, found fast
CROP = 1.45                   # m: points beyond this from a pose cannot touch any move (1 m drive + body + margin)
MERGE_MAX = 1.5               # m: goal_exec's MAX_LEG


def _local(pts, st):
    """Points (start frame) in the frame of pose st, cropped to CROP."""
    c, s = math.cos(st[2]), math.sin(st[2])
    q = pts - (st[0], st[1])
    b = np.stack([c * q[:, 0] + s * q[:, 1], -s * q[:, 0] + c * q[:, 1]], 1)
    return b[(np.abs(b[:, 0]) < CROP) & (np.abs(b[:, 1]) < CROP)]


def _drive_ok(ge, p, d):
    """goal_exec.drive_clear at PLAN_MARGIN."""
    if not len(p):
        return True
    sgn = 1.0 if d >= 0 else -1.0
    x = p[:, 0] * sgn
    edge = FRONT if d >= 0 else REAR
    hit = (x > edge) & (x < edge + abs(d) + PLAN_MARGIN) & (np.abs(p[:, 1]) < SIDE + PLAN_MARGIN)
    return not hit.any()


def _turn_ok(ge, p, a):
    """goal_exec.turn_clear (learned pivot, every 3 deg) at PLAN_MARGIN."""
    return abs(a) <= _max_turn(ge, p, 1 if a > 0 else -1, abs(a)) + 1e-9


_SWEEPS = {}


def _sweep(P, k, upto):
    """Rotations and slides of the body every 3 deg from 0 to upto, about P."""
    key = (round(float(P[0]), 4), round(float(P[1]), 4), k, round(upto, 4))
    if key not in _SWEEPS:
        step = math.radians(3)
        ang = np.minimum(np.arange(0.0, upto + step, step), upto) * k
        c, s = np.cos(ang), np.sin(ang)
        Rs = np.stack([np.stack([c, -s], -1), np.stack([s, c], -1)], -2)       # A x 2 x 2
        ts = np.einsum('aij,j->ai', np.eye(2)[None] - Rs, np.asarray(P, float))  # A x 2
        _SWEEPS[key] = (np.abs(ang), Rs, ts)
    return _SWEEPS[key]


def _max_turn(ge, p, k, upto):
    """The largest turn (rad, <= upto) direction k (+1 left) the outline can
    sweep about the learned pivot, checked every 3 deg like goal_exec. One
    sweep answers every turn that way: a turn of a is clear exactly when
    every angle up to a is. All the angles in one array operation."""
    P = ge.P[k]
    m = PLAN_MARGIN + (ge.UNLEARNED_EXTRA if ge.P_n[k] < 2 else 0.0)
    # every point the outline can sweep is within its corner radius of the
    # pivot: the rest cannot touch, whatever the angle
    reach_r = math.hypot(max(FRONT, REAR) + m, SIDE + m) + 0.005
    p = p[np.hypot(p[:, 0] - P[0], p[:, 1] - P[1]) < reach_r + float(np.hypot(*P))]
    if not len(p):
        return upto
    ang, Rs, ts = _sweep(P, k, upto)
    q = np.einsum('anj,aji->ani', p[None, :, :] - ts[:, None, :], Rs)       # (p - t) @ R, per angle
    hit = ((q[..., 0] < FRONT + m) & (q[..., 0] > -REAR - m) & (np.abs(q[..., 1]) < SIDE + m)).any(1)
    if not hit.any():
        return upto
    first = int(np.argmax(hit))
    return float(ang[first - 1]) if first > 0 else 0.0


def _inside_now(p):
    """Points within goal_exec's own MARGIN of the outline where the rover
    stands: goal_exec would call it touching. (Within PLAN_MARGIN is fine at
    the start: each move is checked, and a side point blocks only turns.)"""
    return ((p[:, 0] < FRONT + MARGIN) & (p[:, 0] > -REAR - MARGIN)
            & (np.abs(p[:, 1]) < SIDE + MARGIN))


def plan(pts, target, ge, budget_s=BUDGET_S):
    """pts: N x 2 obstacle points in the rover's frame (base_link, now).
    target: (x, y) in the same frame. ge: a GoalExec (for its learned pivot).
    Returns (steps, info): steps a list of ('drive', m) / ('turn', rad), merged
    and ready for goal_exec, or None; info a dict for the log."""
    t0 = time.time()
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    pts = pts[np.hypot(pts[:, 0], pts[:, 1]) < SEARCH_R + CROP]
    # a fused-map cell arrives as its four corners and neighbours share them:
    # one point per 5 mm is the same obstacle at a quarter of the work
    pts = np.unique(np.round(pts / 0.005), axis=0) * 0.005
    tx, ty = float(target[0]), float(target[1])
    start = (0.0, 0.0, 0.0)
    # A start goal_exec already calls touching (within its 1 cm) is reported,
    # not planned from: reach's escape steps off first.
    if _inside_now(_local(pts, start)).any():
        return None, {'why': 'start within 1 cm of something', 'ms': 0, 'expanded': 0}

    def h(st):
        return math.hypot(tx - st[0], ty - st[1])

    def key(st):
        return (round(st[0] / KEY_XY), round(st[1] / KEY_XY), round(wrap(st[2]) / KEY_TH))

    pq = [(h(start), 0.0, 0, start, None)]
    parent = {}
    seen = set()
    n = 0
    tie = 0
    while pq:
        if time.time() - t0 > budget_s or n > MAX_EXPAND:
            return None, {'why': 'no fit found in the time', 'ms': round((time.time() - t0) * 1000), 'expanded': n}
        _, cost, _, st, via = heapq.heappop(pq)
        k = key(st)
        if k in seen:
            continue
        seen.add(k)
        parent[k] = via
        n += 1
        if h(st) <= GOAL_TOL:
            steps = []
            while parent[k] is not None:
                pk, mv = parent[k]
                steps.append(mv)
                k = pk
            steps.reverse()
            return merge(steps), {'ms': round((time.time() - t0) * 1000), 'expanded': n,
                                  'end': [round(st[0], 3), round(st[1], 3), round(math.degrees(st[2]), 1)]}
        p = _local(pts, st)
        c, s = math.cos(st[2]), math.sin(st[2])
        for d in DRIVES:
            if d < -BACK_MAX:
                continue
            nst = (st[0] + d * c, st[1] + d * s, st[2])
            if math.hypot(nst[0], nst[1]) > SEARCH_R or key(nst) in seen:
                continue
            if _drive_ok(ge, p, d):
                g = cost + (abs(d) if d > 0 else REVERSE_X * abs(d))
                tie += 1
                heapq.heappush(pq, (g + H_WEIGHT * h(nst), g, tie, nst, (k, ('drive', d))))
        free = {k_: _max_turn(ge, p, k_, max(abs(a) for a in TURNS)) for k_ in (1, -1)}
        for a in TURNS:
            sl = R(st[2]) @ ((np.eye(2) - R(a)) @ ge.P[1 if a > 0 else -1])
            nst = (st[0] + sl[0], st[1] + sl[1], wrap(st[2] + a))
            if key(nst) in seen:
                continue
            if abs(a) <= free[1 if a > 0 else -1] + 1e-9:
                g = cost + TURN_COST + TURN_COST_RAD * abs(a)
                tie += 1
                heapq.heappush(pq, (g + H_WEIGHT * h(nst), g, tie, nst, (k, ('turn', a))))
    return None, {'why': 'no fit: every way is blocked', 'ms': round((time.time() - t0) * 1000), 'expanded': n}


def merge(steps):
    """Join neighbours of a kind: drives one way up to MERGE_MAX, turns one way."""
    out = []
    for kind, v in steps:
        if out and out[-1][0] == kind and (out[-1][1] > 0) == (v > 0):
            if kind == 'turn' or abs(out[-1][1] + v) <= MERGE_MAX:
                out[-1] = (kind, out[-1][1] + v)
                continue
        out.append((kind, v))
    return out


def poses_of(steps, start, ge):
    """The pose after each step, from start (x, y, th): what the plan expects,
    turns sliding about the learned pivot as goal_exec's do. For drawing."""
    x, y, th = start
    out = [(x, y, th)]
    for kind, v in steps or []:
        if kind == 'drive':
            x, y = x + v * math.cos(th), y + v * math.sin(th)
        else:
            sl = R(th) @ ((np.eye(2) - R(v)) @ ge.P[1 if v > 0 else -1])
            x, y, th = x + float(sl[0]), y + float(sl[1]), wrap(th + v)
        out.append((x, y, th))
    return out


def describe(steps):
    """[('drive', 0.1), ('turn', 0.785)] -> 'F10 L45' (for the log and the brain)."""
    words = []
    for kind, v in steps or []:
        if kind == 'drive':
            words.append(f"{'F' if v > 0 else 'B'}{abs(v) * 100:.0f}")
        else:
            words.append(f"{'L' if v > 0 else 'R'}{abs(math.degrees(v)):.0f}")
    return ' '.join(words)
