"""follow.py — "follow me": keep a person ~1 m ahead, facing them.

No ROS: follow_node feeds it the pose, the people (/vision/people) and the
obstacle points; test_follow.py runs it offline.

WHY (2026-10-10). "Mitra, follow me" / "come to me". nav2 plans to a FIXED
pose; a person walks, and a goal re-sent every second is a robot that stops,
re-plans and lurches. This is the direct way: face the person, close to GAP,
check the corridor every step. If the straight line is blocked it stops and
keeps facing them; it never plans around (a later step can hand that to reach).

WHO. At start: the person nearest the middle of the view (smallest |bearing|)
within START_RANGE, seen in this frame. Then the tracker id. The tracker
drops a track 1.5 s after losing it and a returning person gets a NEW id
(floor check 2026-10-10: out of view 3.6 s -> id 2), so when our id is gone
the follower RE-ACQUIRES: someone seen now within REACQ_M of where the target
should be by now (last position + velocity, coasting at most COAST_S). Nobody
for LOST_S -> stop, 'lost'.

SEARCH (2026-10-12). Every real follow of 2026-10-10/11 ended "lost sight for
3 s": a person who steps out of the side of the view is gone before the rover
has turned after them. Now, once the coast is over (unseen > COAST_S), it turns
in place toward the side they left by -- the side of the last bearing, else
the side they were walking to -- at SEARCH_W, at most SEARCH_TURN, and anyone
seen within REACQ_SEARCH_M of where they should be is them again. A person
lost straight ahead gets no turn: it keeps closing on their last position (a
doorway). Nobody for LOST_S (now 8 s) -> 'lost', as before.

FIND, REMEMBER, SEARCH FURTHER (2026-10-13, owner: "when someone asks to
follow me it asks them to be in front -- it needs to find them by rotating;
save his pic; when he goes out of view it needs to search here and there and
find and follow him").
  - FIND: "follow me" with nobody in view no longer ends 'nobody'. It looks
    ahead FIND_WAIT_S (YOLO waking), then turns in place, a full circle at
    most; the first person seen within START_RANGE is the one (status
    found=true -- the brain says so, and "stop" from the wrong person ends it).
    A full circle with no one, or no room to turn: 'nobody'.
  - REMEMBER: every person carries a `look` from detections_3d (their
    clothes' colours, phase4/nodes/person_look.py); the follower keeps a
    running average of its target's. Someone standing where the target should
    be but looking clearly different (LOOK_OTHER) is NOT re-acquired, and while
    searching anyone in view who looks like them (LOOK_SAME) is, wherever they
    are. Thresholds are first guesses: every re-acquire logs its similarity.
  - SEARCH in three stages, LOST_S (now 40 s) the cap over all of them:
    turn toward the side they left by (as above), then DRIVE to where they
    were last seen (at SEARCH_V, the corridor governor still in charge; a
    block there skips ahead), then turn a full circle there. Nobody -> 'lost'.

MOTION (vx, wz; no reversing — the camera looks forward):
  - wz = K_W * bearing error, capped W_MAX; inside TURN_DEADBAND none;
  - vx = K_V * (distance - GAP), capped V_MAX, none inside DIST_DEADBAND;
    scaled by cos(error)^2 and zero beyond DRIVE_MAX_ERR, so it turns to
    face them before it drives;
  - both rate-limited (A_MAX, AW_MAX): no lurches.

SAFETY (every step, before anything is commanded):
  - a speed governor on the corridor ahead (outline width + SIDE_MARGIN,
    LiDAR + fused-map points in base_link): the free distance from the front
    edge to the nearest point in it allows (free - FRONT_MARGIN) / STOP_T m/s,
    so it slows smoothly and holds FRONT_MARGIN off. Under V_MIN allowed ->
    blocked: vx 0 (turning still allowed if the turn is clear); blocked for
    BLOCKED_S -> stop, 'blocked'. Points within PERSON_R of the target are the
    target, not an obstacle. Blocked within NEAR_OK_M of the gap is HOLDING,
    not a failure: it is already about where it should be (2026-10-12: a
    follow quit 'blocked: something 0.25 m ahead' with the person ~1.15 m away,
    i.e. 3 cm past the dead band); "come to me" counts it as reached. (The first version stopped hard from full speed
    when something entered a corridor sized by the WANTED speed -- 0.39 m off
    a wall in test_follow. The governor slows instead: a hard stop from
    0.3 m/s slides these wheels, and slides corrupt the pose.)
  - turning: nothing within the outline's turning circle + TURN_MARGIN;
  - a single jump of the target's position by more than JUMP_M from where it
    should be is ignored once (a stale colour frame met fresh depth, floor
    check 2026-10-10: 1.52 -> 0.96 -> 1.76 m);
  - the node adds: pose not trusted, people messages stale, teleop/other
    goals, and the overall time cap.
"""
import math

import numpy as np

from goal_exec import FRONT, REAR, SIDE, wrap

GAP = 1.0             # m kept between base_link and the person
START_RANGE = 4.0     # m: who can be picked at start
V_MAX = 0.30          # m/s
W_MAX = 0.8           # rad/s
K_V = 0.6             # 1/s
K_W = 1.5             # 1/s
A_MAX = 0.4           # m/s^2
AW_MAX = 2.0          # rad/s^2
DIST_DEADBAND = 0.12  # m around GAP: no drive
TURN_DEADBAND = math.radians(6)
DRIVE_MAX_ERR = math.radians(40)
STOP_T = 1.0          # s: drive no faster than the free distance ahead allows in this time
FRONT_MARGIN = 0.25   # m always kept clear in front
V_MIN = 0.03          # m/s: a governor below this is "blocked"
SIDE_MARGIN = 0.05    # m each side of the outline
TURN_MARGIN = 0.03    # m beyond the outline's turning circle
PERSON_R = 0.40       # m around the target: the person, not an obstacle
REACQ_M = 1.0         # m from the target's predicted position
COAST_S = 1.0         # s of the target's velocity used to predict it
LOST_S = 40.0         # s without the target -> lost: the cap over the whole search
BLOCKED_S = 10.0      # s blocked -> give up
NEAR_OK_M = 0.5       # m past GAP: blocked this close is holding, not a failure
SEARCH_W = 0.5        # rad/s: the search turn
SEARCH_TURN = math.radians(120)   # rad: the most the search turns
SEARCH_VY = 0.15      # m/s: sideways walk that picks the search side when facing their spot
REACQ_SEARCH_M = 2.0  # m from the predicted position: re-acquire radius while searching
JUMP_M = 0.7          # m: a one-frame jump bigger than this is ignored once
SEARCH_V = 0.2        # m/s: the drive to where they were last seen
GO_DONE_M = 0.5       # m from that spot: there, look around
FIND_WAIT_S = 1.0     # s looking straight ahead before the find turn (YOLO waking up)
FIND_MAX_S = 25.0     # s: a find that has not come round by now (wheels slipping, stuck) ends
LOOK_SAME = 0.70      # look_sim this high, anywhere in view while searching: them again
LOOK_OTHER = 0.40     # look_sim this low: someone else, even where they should be
LOOK_ALPHA = 0.2      # weight of a new look in the target's running average
LOOK_BINS = 11        # = phase4/nodes/person_look.BINS (one band's histogram)

TURN_R = math.hypot(max(FRONT, REAR), SIDE)
FULL_TURN = 2 * math.pi - math.radians(10)   # a "full circle", allowing for the stop


def look_sim(a, b):
    """0..1, 1 = the same clothes' colours (histogram intersection per band,
    averaged). None when either look is missing."""
    if not a or not b or len(a) != len(b) or len(a) % LOOK_BINS:
        return None
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = len(a) // LOOK_BINS
    return float(np.minimum(a, b).sum() / n)


def blend_look(old, new):
    if not new:
        return old
    if not old or len(old) != len(new):
        return list(new)
    return np.round((1 - LOOK_ALPHA) * np.asarray(old) + LOOK_ALPHA * np.asarray(new), 3).tolist()


class Follower:
    def __init__(self):
        self.state = 'idle'          # idle | finding | following | done
        self.result = ''             # done: lost | blocked | nobody | cancelled | ...
        self.why = ''
        self.target_id = None
        self.tx = self.ty = None     # target's odom position, last accepted
        self.tvx = self.tvy = 0.0
        self.t_seen = None           # time the target was last seen
        self.blocked_since = None
        self.blocked_why = ''
        self.holding = ''            # blocked, but already near the gap
        self.search_dir = None       # +1 left / -1 right / 0 no turn; None = not searching
        self.search_phase = None     # turn | go | spin; None = not searching
        self.search_turned = 0.0
        self.search_yaw = None
        self.look = None             # the target's clothes' colours, running average
        self.last_sim = None         # look_sim of the last re-acquire, for the log
        self.found = False           # the target was found by turning (FIND)
        self.find_t0 = None
        self.find_yaw = None
        self.find_turned = 0.0
        self.jumped = False
        self.vx = self.wz = 0.0
        self.t = None

    # ── who ────────────────────────────────────────────────────────────────
    def start(self, t, people, target_id=None, find=False):
        """Pick the target. With nobody in view: find=True turns to look for
        someone (state finding); otherwise False (state done, result nobody)."""
        self.__init__()
        seen = self._candidates(people)
        if target_id is not None:
            seen = [p for p in seen if p['id'] == target_id]
        if not seen:
            if find and target_id is None:
                self.state, self.t, self.find_t0 = 'finding', t, t
                return True
            self._finish('nobody', 'no one in view to follow')
            return False
        p = min(seen, key=lambda p: abs(p['bearing']))
        self.state = 'following'
        self.t = t
        self._accept(p, t)
        return True

    @staticmethod
    def _candidates(people):
        return [p for p in people if p.get('lost_s', 0) == 0 and 'bearing' in p
                and p.get('dist', 99) <= START_RANGE]

    def _looks_other(self, p):
        """Clearly not the target by their clothes (unknown looks never are)."""
        sim = look_sim(self.look, p.get('look'))
        return sim is not None and sim < LOOK_OTHER

    def cancel(self, why='cancelled'):
        if self.state in ('following', 'finding'):
            self._finish('cancelled', why)

    def _finish(self, result, why):
        self.state, self.result, self.why = 'done', result, why
        self.vx = self.wz = 0.0

    def _accept(self, p, t):
        self.target_id = p['id']
        self.tx, self.ty = p['x'], p['y']
        self.tvx, self.tvy = p.get('vx', 0.0), p.get('vy', 0.0)
        self.t_seen = t - p.get('lost_s', 0.0)
        self.look = blend_look(self.look, p.get('look'))
        self.search_dir, self.search_phase = None, None
        self.search_turned, self.search_yaw = 0.0, None

    def predicted(self, t):
        dt = min(max(t - self.t_seen, 0.0), COAST_S)
        return self.tx + self.tvx * dt, self.ty + self.tvy * dt

    def update_people(self, t, people):
        """A /vision/people message (taken at about t). Keeps or re-acquires the target."""
        if self.state == 'finding':
            seen = self._candidates(people)
            if seen:                        # the first person found is the one
                self.state, self.found = 'following', True
                self._accept(min(seen, key=lambda p: abs(p['bearing'])), t)
            return
        if self.state != 'following':
            return
        mine = [p for p in people if p['id'] == self.target_id and p.get('lost_s', 0) == 0]
        if mine:
            p = mine[0]
            px, py = self.predicted(t)
            if math.hypot(p['x'] - px, p['y'] - py) > JUMP_M and not self.jumped:
                self.jumped = True          # ignore one wild sample
                return
            self.jumped = False
            self._accept(p, t)
            return
        # our id is gone (or only coasting): someone where the target should be?
        px, py = self.predicted(t)
        searching = self.search_phase is not None
        r = REACQ_SEARCH_M if searching else REACQ_M
        seen = [p for p in people if p.get('lost_s', 0) == 0]
        near = [p for p in seen if math.hypot(p['x'] - px, p['y'] - py) <= r
                and not self._looks_other(p)]
        if near:
            p = min(near, key=lambda p: math.hypot(p['x'] - px, p['y'] - py))
            self.last_sim = look_sim(self.look, p.get('look'))
            self._accept(p, t)
            return
        if searching and self.look:         # anyone in view who looks like them
            alike = [(look_sim(self.look, p.get('look')) or 0.0, p) for p in seen
                     if p.get('dist', 99) <= START_RANGE]
            alike = [(s, p) for s, p in alike if s >= LOOK_SAME]
            if alike:
                self.last_sim, p = max(alike, key=lambda a: a[0])
                self._accept(p, t)

    # ── motion ─────────────────────────────────────────────────────────────
    def step(self, t, pose, pts=None):
        """pose (x, y, yaw) odom; pts N x 2 obstacle points in base_link.
        Returns (vx, wz)."""
        if self.state == 'finding':
            return self._find_step(t, pose, pts)
        if self.state != 'following':
            return 0.0, 0.0
        dt = 0.05 if self.t is None else min(max(t - self.t, 0.0), 0.2)
        self.t = t
        if t - self.t_seen > LOST_S:
            self._finish('lost', f'lost sight of them for {LOST_S:.0f} s')
            return 0.0, 0.0

        px, py = self.predicted(t)
        dx, dy = px - pose[0], py - pose[1]
        dist = math.hypot(dx, dy)
        err = wrap(math.atan2(dy, dx) - pose[2])

        wz = 0.0 if abs(err) < TURN_DEADBAND else max(-W_MAX, min(W_MAX, K_W * err))
        vx = 0.0
        if abs(dist - GAP) > DIST_DEADBAND and dist > GAP and abs(err) < DRIVE_MAX_ERR:
            vx = min(V_MAX, K_V * (dist - GAP)) * math.cos(err) ** 2

        # the coast is over and they are still unseen: search (see SEARCH)
        searching = t - self.t_seen > COAST_S
        if searching:
            vx, wz = self._search(pose, px, py, err)
            if self.state != 'following':
                return 0.0, 0.0

        # target in base_link: its points are the person, not an obstacle
        c, s = math.cos(pose[2]), math.sin(pose[2])
        tb = np.array([dx * c + dy * s, -dx * s + dy * c])
        if pts is not None and len(pts):
            pts = pts[np.hypot(pts[:, 0] - tb[0], pts[:, 1] - tb[1]) > PERSON_R]

        free = self.free_ahead(pts)
        v_cap = max(0.0, free - FRONT_MARGIN) / STOP_T
        ok = v_cap >= V_MIN or vx == 0.0          # nothing wanted = nothing blocked
        why = f'something {free:.2f} m ahead' if not ok else ''
        vx = min(vx, v_cap)
        self.holding = ''
        if searching:
            self.blocked_since = None
            if not ok:
                vx = 0.0
                if self.search_phase == 'go':     # can't get to their spot: look around from here
                    self._begin_spin(pose)
            else:
                vx = max(self.vx - A_MAX * dt, min(self.vx + A_MAX * dt, vx))
                vx = min(vx, v_cap)
        elif not ok and dist <= GAP + NEAR_OK_M:
            vx = 0.0                     # near enough: hold here, it is not a failure
            self.blocked_since = None
            self.holding = why
        elif not ok:
            vx = 0.0
            self.blocked_since = self.blocked_since or t
            self.blocked_why = why
            if t - self.blocked_since > BLOCKED_S:
                self._finish('blocked', why)
                return 0.0, 0.0
        elif dist <= GAP:
            self.blocked_since = None
            vx = 0.0          # already inside the gap: stop now, not gently (floor dry run
                              # 2026-10-10: 0.17 m/s with them at 0.95 m, decelerating)
        else:
            self.blocked_since = None
            vx = max(self.vx - A_MAX * dt, min(self.vx + A_MAX * dt, vx))   # gentle...
            vx = min(vx, v_cap)          # ...but never past what the free distance allows
        # a stop for safety is immediate; everything else is rate limited
        wz = max(self.wz - AW_MAX * dt, min(self.wz + AW_MAX * dt, wz))
        if not self.turn_clear(pts):
            wz = 0.0
            if self.search_phase == 'turn':       # no room to turn: go to their spot instead
                self.search_phase = 'go'
            elif self.search_phase == 'spin':
                self._finish('lost', "lost sight of them, and something too close beside me "
                                     "to turn and look around")
                return 0.0, 0.0
        self.vx, self.wz = vx, wz
        self.dist, self.err = dist, err
        return vx, wz

    def _turned(self, pose):
        """Radians turned since the last call (and remember this heading)."""
        d = 0.0 if self.search_yaw is None else abs(wrap(pose[2] - self.search_yaw))
        self.search_yaw = pose[2]
        return d

    def _begin_spin(self, pose):
        self.search_phase, self.search_turned, self.search_yaw = 'spin', 0.0, pose[2]

    def _search(self, pose, px, py, err):
        """(vx, wz) wanted while the target is unseen: turn, go, spin (SEARCH)."""
        if self.search_phase is None:
            c0, s0 = math.cos(pose[2]), math.sin(pose[2])
            vy_b = -self.tvx * s0 + self.tvy * c0          # their sideways walk, base_link
            if abs(err) > TURN_DEADBAND:
                self.search_dir = 1 if err > 0 else -1
            elif abs(vy_b) > SEARCH_VY:
                self.search_dir = 1 if vy_b > 0 else -1
            else:
                self.search_dir = 0
            self.search_phase = 'turn' if self.search_dir else 'go'
            self.search_turned, self.search_yaw = 0.0, pose[2]
        if self.search_phase == 'turn':
            self.search_turned += self._turned(pose)
            if self.search_turned < SEARCH_TURN:
                return 0.0, self.search_dir * SEARCH_W
            self.search_phase = 'go'
        if self.search_phase == 'go':
            dx, dy = px - pose[0], py - pose[1]
            d = math.hypot(dx, dy)
            if d > GO_DONE_M:
                e = wrap(math.atan2(dy, dx) - pose[2])
                wz = 0.0 if abs(e) < TURN_DEADBAND else max(-W_MAX, min(W_MAX, K_W * e))
                vx = min(SEARCH_V, K_V * d) * math.cos(e) ** 2 if abs(e) < DRIVE_MAX_ERR else 0.0
                return vx, wz
            self._begin_spin(pose)
        self.search_turned += self._turned(pose)
        if self.search_turned >= FULL_TURN:
            self._finish('lost', 'looked where they went and all around there, '
                                 'and could not see them')
            return 0.0, 0.0
        return 0.0, (self.search_dir or 1) * SEARCH_W

    def _find_step(self, t, pose, pts):
        """FIND: nobody in view at the start -- look ahead, then turn a full circle."""
        dt = 0.05 if self.t is None else min(max(t - self.t, 0.0), 0.2)
        self.t = t
        wz = 0.0
        if t - self.find_t0 >= FIND_WAIT_S:
            if self.find_yaw is not None:
                self.find_turned += abs(wrap(pose[2] - self.find_yaw))
            self.find_yaw = pose[2]
            if self.find_turned >= FULL_TURN:
                self._finish('nobody', 'turned a full circle and saw no one')
                return 0.0, 0.0
            if t - self.find_t0 > FIND_MAX_S:
                self._finish('nobody', f'looked for {FIND_MAX_S:.0f} s, turned only '
                                       f'{math.degrees(self.find_turned):.0f} deg, and saw no one')
                return 0.0, 0.0
            if not self.turn_clear(pts):
                self._finish('nobody', 'no one in view, and something too close beside me '
                                       'to turn and look')
                return 0.0, 0.0
            wz = SEARCH_W
        wz = max(self.wz - AW_MAX * dt, min(self.wz + AW_MAX * dt, wz))
        self.vx, self.wz = 0.0, wz
        return 0.0, wz

    @staticmethod
    def free_ahead(pts, horizon=3.0):
        """Free distance (m) from the front edge to the nearest point in the
        outline's width + SIDE_MARGIN; horizon when nothing is there."""
        if pts is None or not len(pts):
            return horizon
        hit = (pts[:, 0] > FRONT - 0.02) & (np.abs(pts[:, 1]) < SIDE + SIDE_MARGIN)
        if not hit.any():
            return horizon
        return max(0.0, min(horizon, float(np.min(pts[hit, 0])) - FRONT))

    @staticmethod
    def turn_clear(pts):
        if pts is None or not len(pts):
            return True
        return not (np.hypot(pts[:, 0], pts[:, 1]) < TURN_R + TURN_MARGIN).any()

    def status(self):
        d = {'state': self.state, 'target': self.target_id}
        if self.state == 'done':
            d.update(result=self.result, why=self.why)
        elif self.state == 'finding':
            d.update(turned=round(math.degrees(self.find_turned)), wz=round(self.wz, 2))
        elif self.state == 'following':
            d.update(dist=round(getattr(self, 'dist', 0.0), 2),
                     bearing=round(getattr(self, 'err', 0.0), 2),
                     vx=round(self.vx, 2), wz=round(self.wz, 2),
                     unseen_s=round(max((self.t or 0) - self.t_seen, 0.0), 1))
            if self.blocked_since is not None:
                d['blocked'] = self.blocked_why
            if self.holding:
                d['holding'] = self.holding
            if self.search_phase:
                d['searching'] = self.search_phase
                d['turned'] = round(math.degrees(self.search_turned))
            if self.found:
                d['found'] = True
        return d
