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
LOST_S = 8.0          # s without the target -> lost (search included)
BLOCKED_S = 10.0      # s blocked -> give up
NEAR_OK_M = 0.5       # m past GAP: blocked this close is holding, not a failure
SEARCH_W = 0.5        # rad/s: the search turn
SEARCH_TURN = math.radians(120)   # rad: the most the search turns
SEARCH_VY = 0.15      # m/s: sideways walk that picks the search side when facing their spot
REACQ_SEARCH_M = 2.0  # m from the predicted position: re-acquire radius while searching
JUMP_M = 0.7          # m: a one-frame jump bigger than this is ignored once

TURN_R = math.hypot(max(FRONT, REAR), SIDE)


class Follower:
    def __init__(self):
        self.state = 'idle'          # idle | following | done
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
        self.search_turned = 0.0
        self.search_yaw = None
        self.jumped = False
        self.vx = self.wz = 0.0
        self.t = None

    # ── who ────────────────────────────────────────────────────────────────
    def start(self, t, people, target_id=None):
        """Pick the target. False (and state done, result nobody) if no one fits."""
        self.__init__()
        seen = [p for p in people if p.get('lost_s', 0) == 0 and 'bearing' in p
                and p.get('dist', 99) <= START_RANGE]
        if target_id is not None:
            seen = [p for p in seen if p['id'] == target_id]
        if not seen:
            self._finish('nobody', 'no one in view to follow')
            return False
        p = min(seen, key=lambda p: abs(p['bearing']))
        self.state = 'following'
        self.t = t
        self._accept(p, t)
        return True

    def cancel(self, why='cancelled'):
        if self.state == 'following':
            self._finish('cancelled', why)

    def _finish(self, result, why):
        self.state, self.result, self.why = 'done', result, why
        self.vx = self.wz = 0.0

    def _accept(self, p, t):
        self.target_id = p['id']
        self.tx, self.ty = p['x'], p['y']
        self.tvx, self.tvy = p.get('vx', 0.0), p.get('vy', 0.0)
        self.t_seen = t - p.get('lost_s', 0.0)
        self.search_dir, self.search_turned, self.search_yaw = None, 0.0, None

    def predicted(self, t):
        dt = min(max(t - self.t_seen, 0.0), COAST_S)
        return self.tx + self.tvx * dt, self.ty + self.tvy * dt

    def update_people(self, t, people):
        """A /vision/people message (taken at about t). Keeps or re-acquires the target."""
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
        r = REACQ_SEARCH_M if self.search_dir is not None else REACQ_M
        near = [p for p in people if p.get('lost_s', 0) == 0
                and math.hypot(p['x'] - px, p['y'] - py) <= r]
        if near:
            self._accept(min(near, key=lambda p: math.hypot(p['x'] - px, p['y'] - py)), t)

    # ── motion ─────────────────────────────────────────────────────────────
    def step(self, t, pose, pts=None):
        """pose (x, y, yaw) odom; pts N x 2 obstacle points in base_link.
        Returns (vx, wz)."""
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
        if t - self.t_seen > COAST_S:
            if self.search_dir is None:
                c0, s0 = math.cos(pose[2]), math.sin(pose[2])
                vy_b = -self.tvx * s0 + self.tvy * c0          # their sideways walk, base_link
                if abs(err) > TURN_DEADBAND:
                    self.search_dir = 1 if err > 0 else -1
                elif abs(vy_b) > SEARCH_VY:
                    self.search_dir = 1 if vy_b > 0 else -1
                else:
                    self.search_dir = 0
                self.search_yaw = pose[2]
            if self.search_dir:
                self.search_turned += abs(wrap(pose[2] - self.search_yaw))
                self.search_yaw = pose[2]
                vx = 0.0
                wz = self.search_dir * SEARCH_W if self.search_turned < SEARCH_TURN else 0.0

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
        if not ok and dist <= GAP + NEAR_OK_M:
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
        self.vx, self.wz = vx, wz
        self.dist, self.err = dist, err
        return vx, wz

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
        elif self.state == 'following':
            d.update(dist=round(getattr(self, 'dist', 0.0), 2),
                     bearing=round(getattr(self, 'err', 0.0), 2),
                     vx=round(self.vx, 2), wz=round(self.wz, 2),
                     unseen_s=round(max((self.t or 0) - self.t_seen, 0.0), 1))
            if self.blocked_since is not None:
                d['blocked'] = self.blocked_why
            if self.holding:
                d['holding'] = self.holding
            if self.search_dir:
                d['searching'] = round(math.degrees(self.search_turned))
        return d
