#!/usr/bin/env python3
"""goal_exec.py — reach an exact pose (x, y, θ). SENSOR_FUSION_PLAN.md M4.

No ROS inside: phase3/tools/sim_goal_exec.py drives this class through a
skid-steer simulator with the rover's real quirks, and goal_exec_node.py runs
the same class live on fusion2's /odom.

    ex = GoalExec()
    ex.set_goal((x, y, th))                      # in the pose's frame (odom)
    vx, wz = ex.step(t, pose, scan_pts)          # 20 Hz; scan in base_link
    ex.state, ex.result, ex.why

HOW IT MOVES: approach along the goal line
    The goal (x, y, th) defines a LINE: through the goal point, along th.
    1. PRE-GOAL  if the rover is not on that line, facing along it, it goes to
                 a point L behind the goal on the line (turn -> straight), then
                 turns to th, both turns' slides pre-compensated.
    2. APPROACH  a straight drive ALONG THE LINE to the goal, holding th and
                 steering out cross-track error as it goes. The last motion is
                 a straight drive, so no final turn can slide it off.
    3. RETRY     still off the line? Back up ALONG the line to the pre-goal and
                 approach again: sideways error is corrected by steering,
                 without a single turn in place.
    4. HEADING   only a small heading error left: a small turn (small slide).

    The first design turned to face each correction and back. On this rover
    that is two large turns, each sliding ~6 cm, to fix 5 cm: it never
    converged (simulator, 2026-09-26). Docking along a line is also how the
    arm will approach an object later.

THE SLIDE, LEARNED
    This rover does not turn about its centre: a turn rotates it about a pivot
    point P in the body frame, which moved from near the left tyres (weak pack,
    2026-09-24, ~33 cm per 90) to near the centre (charged, ~6 cm per 90). A
    fixed P is always wrong. So P is MEASURED from every turn:
        base moved d (body frame at the turn's start) through angle a:
        d = (I - R(a)) P   =>   P = (I - R(a))^-1 d
    one estimate per turn direction, smoothed. Before the approach, the aim
    point is shifted by the slide the FINAL turn will cause,
        aim = goal - R(h_arrive) (I - R(th_goal - h_arrive)) P
    so that turn slides the rover INTO place rather than out of it.

SAFETY, before each primitive and while it runs
    turn   : the outline + MARGIN swept about P over the turn, every 3 deg,
             against the scan -- at the start, and every step for what is
             left of the turn
    drive  : the outline + MARGIN swept along the leg, against the scan --
             at the start, and every step for what is left of the leg
             (until 2026-09-26 only at the start: a person stepping in
             mid-leg was not seen). Mid-motion a block must show on
             BLOCK_STEPS consecutive steps, so one noisy point cannot stop it.
    always : no progress for STALL_S -> stop; pose unsure (caller says) -> stop
    A refusal ends the goal with the reason; the caller decides what next.

PASS MODE (./rover pass): a straight crawl through a tight gap
    set_goal(end_pose, pass_=dict(L=..., margin=0.03, vx=0.05)): the goal line
    is the gap's centre line, the pre-goal L before the goal is in front of the
    gap, and the approach is the pass. Differences from a normal goal:
      - the approach is checked along the LINE (where the rover will be once
        it has steered onto it), outline + the pass margin (owner: 3 cm), not
        along the body's current heading;
      - every step the rest of the line is re-checked (low things appear
        only as the camera closes in): blocked -> 'refused: blocked ahead',
        and ./rover pass re-measures from there and tries a new line;
      - every step, anything within CONTACT of the outline, ahead in the
        direction of travel or beside it, stops the leg; it then backs up
        along the line to the pre-goal and tries again (MAX_TRIES);
      - line speed capped at the pass vx; turns and the pre-goal manoeuvre
        keep the normal 5 cm margin;
      - done within 3 cm / 3 deg at the end: past the gap, a final pivot
        next to its edges is the riskiest move left, so it is not made.
"""
import math

import numpy as np

# the measured outline (description/params.yaml; nav2.yaml footprint)
FRONT, REAR, SIDE = 0.182, 0.178, 0.19
MARGIN = 0.05                 # the owner's 5 cm


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def R(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


class GoalExec:
    # limits and gains (tuned in the simulator, then on the rover)
    WZ_MAX = 1.5          # rad/s commanded; the pivot reaches far less (M4 motion data)
    WZ_MIN = 1.0          # measured: 0.6 turns at ~0.1 rad/s and stalled a live goal; 1.0 gives ~0.31
    STOP_LEAD = 0.25      # s   stop a turn this far ahead on the measured rate (it coasts)
    VX_MAX = 0.15         # m/s
    VX_MIN = 0.035        # the firmware drops a side's target under 0.01 m/s
    ACC = 0.10            # m/s^2 planned deceleration: gentle, the drive coasts ~0.2 s after a stop
    K_TH = 2.5            # turn: rad/s per rad of heading error
    # while driving the rover follows only ~22% of a turn command (measured),
    # so the gains are in COMMAND units and ~4x what a responsive base needs
    K_HOLD = 10.0         # drive: heading hold
    K_CT = 15.0           # drive: cross-track pull toward the line
    WZ_DRIVE = 1.0        # drive: turn-command clip (~0.22 rad/s actual)
    POS_TOL = 0.015       # m   final position
    YAW_TOL = math.radians(1.0)
    FACE_TOL = math.radians(2.0)
    ARRIVE = 0.003        # m   along-track: close enough to stop the leg
    REVERSE_WITHIN = 0.6  # m   a point this close and behind is reached backwards
    MAX_TRIES = 8         # a full correction cycle (pre-goal + approach) is two
    L = 0.35              # m   pre-goal distance behind the goal: runway to steer on
    L_MAX = 0.50          # m   the line approach covers at most this
    LINE_OK = 0.08        # m   cross-track that a line approach can still steer out
    LINE_YAW = math.radians(15)
    RUNWAY = 0.20         # m   minimum approach length to steer out cross-track
    MAX_LEG = 1.5         # m   longer moves belong to nav2
    STALL_S = 6.0
    CONTACT = 0.012       # m   pass mode: this close to the outline stops the leg
    BLOCK_STEPS = 2       # consecutive blocked steps that stop a turn or leg mid-motion

    def __init__(self, P_prior=None):
        # the learned pivot, per turn direction. A prior (the node saves what
        # it learned after every goal) means a session starts with a good
        # estimate: unlearned, on a weak pack, the first goals ran out of tries
        # while every turn slid 25-50 cm (simulator, 2026-09-26).
        self.P = {+1: np.zeros(2), -1: np.zeros(2)}
        self.P_n = {+1: 0, -1: 0}
        if P_prior:
            for k in (+1, -1):
                self.P[k] = np.array(P_prior[str(k)], dtype=float)
                self.P_n[k] = 3
        self.state = 'idle'
        self.result = None
        self.why = ''
        self.goal = None
        self.tries = 0
        self.log = []
        self.pass_ = None
        self.backoff = False
        self.backing = False

    # ── the learned slide ───────────────────────────────────────────────────
    def learn_pivot(self, p0, p1):
        """One turn: base from p0 to p1 (x, y, th). Update P for its direction."""
        a = wrap(p1[2] - p0[2])
        if abs(a) < math.radians(20):
            return
        d = R(-p0[2]) @ np.array([p1[0] - p0[0], p1[1] - p0[1]])
        M = np.eye(2) - R(a)
        Pn = np.linalg.solve(M, d)
        if np.linalg.norm(Pn) > 0.6:             # physically impossible: not a turn about a point
            return
        k = 1 if a > 0 else -1
        w = 1.0 / (self.P_n[k] + 1) if self.P_n[k] < 4 else 0.25
        self.P[k] = (1 - w) * self.P[k] + w * Pn
        self.P_n[k] += 1

    def pivot_state(self):
        """What to save between runs."""
        return {'1': self.P[1].round(4).tolist(), '-1': self.P[-1].round(4).tolist(),
                'n': {'1': self.P_n[1], '-1': self.P_n[-1]}}

    def slide(self, heading, a):
        """Base displacement (world) a turn of a from `heading` will cause."""
        if abs(a) < 1e-6:
            return np.zeros(2)
        P = self.P[1 if a > 0 else -1]
        return R(heading) @ ((np.eye(2) - R(a)) @ P)

    # ── safety ──────────────────────────────────────────────────────────────
    @staticmethod
    def _inside(pts, m=MARGIN):
        return (pts[:, 0] < FRONT + m) & (pts[:, 0] > -REAR - m) & (np.abs(pts[:, 1]) < SIDE + m)

    def turn_clear(self, pts, a):
        """Scan points (base_link) clear of the outline swept about P through a?"""
        if pts is None or len(pts) == 0:
            return True, ''
        P = self.P[1 if a > 0 else -1]
        for s in np.linspace(0.0, a, max(2, int(abs(math.degrees(a)) / 3) + 1)):
            # pose after turning s about P, as a transform of the body frame
            t = (np.eye(2) - R(s)) @ P
            q = (pts - t) @ R(s)                 # points in the turned body frame
            hit = self._inside(q)
            if hit.any():
                d = float(np.min(np.hypot(pts[hit, 0], pts[hit, 1])))
                return False, f'something {d:.2f} m away is in the {math.degrees(a):+.0f} deg swing'
        return True, ''

    def drive_clear(self, pts, dist):
        if pts is None or len(pts) == 0:
            return True, ''
        sgn = 1.0 if dist >= 0 else -1.0
        x = pts[:, 0] * sgn
        edge = FRONT if dist >= 0 else REAR
        hit = (x > edge) & (x < edge + abs(dist) + MARGIN) & (np.abs(pts[:, 1]) < SIDE + MARGIN)
        if hit.any():
            return False, f'obstacle {float(np.min(x[hit]) - edge):.2f} m into the {abs(dist):.2f} m leg'
        return True, ''

    def line_clear(self, pts, pose, a0, a1, m):
        """Pass mode: the outline + m swept ALONG THE GOAL LINE from along a0 to
        a1, against points (base_link) -> world -> line coordinates."""
        if pts is None or len(pts) == 0:
            return True, ''
        c, s = math.cos(pose[2]), math.sin(pose[2])
        w = np.stack([pose[0] + c * pts[:, 0] - s * pts[:, 1], pose[1] + s * pts[:, 0] + c * pts[:, 1]], 1)
        d = w - self.goal[:2]
        al = d @ self.u
        cr = self.u[0] * d[:, 1] - self.u[1] * d[:, 0]
        lo, hi = min(a0, a1) - REAR, max(a0, a1) + FRONT
        hit = (al > lo - m) & (al < hi + m) & (np.abs(cr) < SIDE + m)
        if hit.any():
            k = int(np.argmin(np.abs(cr) + 10 * ~hit))
            return False, (f'the gap is too tight: something {abs(cr[k]) - SIDE:+.3f} m from the '
                           f'side of the line at {al[k] - a0:.2f} m (margin {m:.2f})')
        return True, ''

    def contact(self, pts, sgn, beside=True):
        """Pass mode, every step: anything within CONTACT of the outline, ahead
        in the direction of travel or beside the body."""
        if pts is None or len(pts) == 0:
            return ''
        x, y = pts[:, 0], np.abs(pts[:, 1])
        c = self.CONTACT
        ahead = (x * sgn > (FRONT if sgn > 0 else REAR)) & (x * sgn < (FRONT if sgn > 0 else REAR) + c + 0.02) & (y < SIDE + c)
        side = (x < FRONT) & (x > -REAR) & (y > SIDE - 0.01) & (y < SIDE + c)
        if ahead.any():
            return 'contact ahead'
        if beside and side.any():
            return 'contact beside'
        return ''

    # ── goals ───────────────────────────────────────────────────────────────
    def set_goal(self, goal, pass_=None):
        """pass_: None, or dict(L, margin, vx) for a pass (see PASS MODE)."""
        self.pass_ = pass_
        self.backoff = False
        self.log = []
        cls = type(self)
        if pass_:
            self.L = float(pass_['L'])
            self.L_MAX = self.L + 0.10
            self.POS_TOL, self.YAW_TOL = 0.03, math.radians(3.0)
            self.LINE_OK = 0.05
        else:
            self.L, self.L_MAX, self.POS_TOL, self.YAW_TOL, self.LINE_OK = (
                cls.L, cls.L_MAX, cls.POS_TOL, cls.YAW_TOL, cls.LINE_OK)
        self.goal = np.array(goal, dtype=float)
        self.u = np.array([math.cos(self.goal[2]), math.sin(self.goal[2])])   # the goal line
        self.tries = 0
        self.result, self.why = None, ''
        self.state = 'plan'

    def cancel(self, why='cancelled'):
        self.state, self.result, self.why = 'done', 'cancelled', why

    def _finish(self, result, why=''):
        self.state, self.result, self.why = 'done', result, why

    def line_coords(self, pose):
        """Pose relative to the goal line: along (negative = behind the goal), cross (+ left)."""
        d = pose[:2] - self.goal[:2]
        return float(d @ self.u), float(self.u[0] * d[1] - self.u[1] * d[0])

    def _pre_goal_plan(self, pose, L):
        """Turn -> straight -> turn to land ON the line, L behind the goal, facing
        along it; both turns' slides included (iterated, heading on both sides)."""
        g = self.goal
        target = g[:2] - L * self.u
        v0 = target - pose[:2]
        h0 = math.atan2(v0[1], v0[0])
        # forward or reverse, decided once: whichever needs less turning in total
        costs = []
        for rev in (False, True):
            h = wrap(h0 + math.pi) if rev else h0
            costs.append((abs(wrap(h - pose[2])) + abs(wrap(g[2] - h)), rev, h))
        _, rev, h = min(costs)
        aim = target
        for _ in range(8):
            p1 = pose[:2] + self.slide(pose[2], wrap(h - pose[2]))
            aim = target - self.slide(h, wrap(g[2] - h))
            v = aim - p1
            if float(np.hypot(*v)) < 1e-3:
                break
            hn = math.atan2(v[1], v[0])
            hn = wrap(hn + math.pi) if rev else hn
            if abs(wrap(hn - h)) < 1e-4:
                break
            h = hn
        return aim, h, rev

    # ── the controller ──────────────────────────────────────────────────────
    def step(self, t, pose, pts=None):
        """pose = (x, y, th) in the goal's frame; pts = scan in base_link (N x 2)."""
        pose = np.asarray(pose, dtype=float)
        if self.state in ('idle', 'done'):
            return 0.0, 0.0
        if self.state == 'plan':
            self._plan(t, pose)
            if self.state in ('done', 'plan'):
                return 0.0, 0.0
        if self.state == 'turn':
            return self._turn(t, pose, pts)
        if self.state == 'drive':
            return self._drive(t, pose, pts)
        return 0.0, 0.0

    def _plan(self, t, pose):
        g = self.goal
        err = float(np.hypot(*(g[:2] - pose[:2])))
        yerr = wrap(g[2] - pose[2])
        along, cross = self.line_coords(pose)
        if err <= self.POS_TOL and abs(yerr) <= self.YAW_TOL:
            self._finish('reached', f'{err * 100:.1f} cm, {math.degrees(yerr):+.1f} deg')
            return
        if self.tries >= self.MAX_TRIES:
            self._finish('failed', f'{err * 100:.1f} cm, {math.degrees(yerr):+.1f} deg after {self.tries} tries')
            return
        self.tries += 1
        near_line = (abs(yerr) <= self.LINE_YAW and abs(cross) <= self.LINE_OK
                     and -self.L_MAX <= along <= 0.10)
        if err <= self.POS_TOL:
            self._start_turn(t, pose, g[2], 'final')                 # 4. heading only
        elif self.backoff and near_line:
            # pass mode, after a contact stop: out the way it came, then again
            self.backoff = False
            self._backing_next = True
            self._start_line(t, pose, self.goal[:2] - self.L * self.u)
        elif near_line and (along <= -self.RUNWAY or abs(cross) <= self.POS_TOL):
            # 2. approach along the line; the runway is where cross-track error
            #    gets steered out, so it needs some length unless already on it
            self._start_line(t, pose, self.goal[:2])
        elif near_line:
            # 3. too close to steer out the offset: back up along the line to
            #    the pre-goal first, then approach again from there
            self._start_line(t, pose, self.goal[:2] - self.L * self.u)
        else:
            dist = float(np.hypot(*(self.goal[:2] - self.L * self.u - pose[:2])))
            if dist > self.MAX_LEG:
                self._finish('refused', f'{dist:.2f} m is a job for nav2 (max {self.MAX_LEG} m)')
                return
            self.aim, h, self.rev = self._pre_goal_plan(pose, self.L)    # 1. pre-goal
            self._start_turn(t, pose, h, 'face')

    def _start_turn(self, t, pose, heading, purpose):
        a = wrap(heading - pose[2])
        self.turn_target, self.turn_purpose = heading, purpose
        self.turn_start = pose.copy()
        self.best, self.best_t = abs(a), t
        self.state = 'turn'
        self._checked = False
        self._w_prev = None                      # (t, th) for the measured turn rate

    def _turn(self, t, pose, pts):
        e = wrap(self.turn_target - pose[2])
        if not self._checked:
            ok, why = self.turn_clear(pts, e)
            if not ok:
                self._finish('refused', why)
                return 0.0, 0.0
            self._checked = True
            self._blocked = 0
        else:
            ok, why = self.turn_clear(pts, e)            # what is left of the swing
            self._blocked = 0 if ok else self._blocked + 1
            if self._blocked >= self.BLOCK_STEPS:
                self._finish('refused', 'blocked mid-turn: ' + why)
                return 0.0, 0.0
        tol = self.YAW_TOL if self.turn_purpose in ('final', 'align') else self.FACE_TOL
        # the measured turn rate, to stop early by what it will coast
        w_meas = 0.0
        if self._w_prev is not None and t > self._w_prev[0]:
            w_meas = wrap(pose[2] - self._w_prev[1]) / (t - self._w_prev[0])
        self._w_prev = (t, pose[2])
        lead = abs(w_meas) * self.STOP_LEAD if w_meas * e > 0 else 0.0
        if abs(e) <= max(tol, lead):
            self.learn_pivot(self.turn_start, pose)
            if self.turn_purpose == 'face':
                self._start_leg(t, pose, self.aim, self.rev, hold=None)
            elif self.turn_purpose == 'toline':
                self._start_turn(t, pose, self.goal[2], 'align')
            else:
                self.state = 'plan'
            return 0.0, 0.0
        if abs(e) < self.best - math.radians(0.5):
            self.best, self.best_t = abs(e), t
        if t - self.best_t > self.STALL_S:
            self._finish('stalled', f'turn stuck {math.degrees(e):+.1f} deg from target')
            return 0.0, 0.0
        # never below WZ_MIN: under it the scrub wins and the turn stalls
        w = min(self.WZ_MAX, max(self.WZ_MIN, self.K_TH * abs(e)))
        return 0.0, math.copysign(w, e)

    def _start_line(self, t, pose, point):
        """Drive along the GOAL LINE (heading = goal th) to `point` on it."""
        self.backing, self._backing_next = getattr(self, '_backing_next', False), False
        along_now, _ = self.line_coords(pose)
        along_to = float((point - self.goal[:2]) @ self.u)
        self.line_mode = True
        self._start_leg(t, pose, point, rev=along_to < along_now, hold=self.goal[2])

    def _start_leg(self, t, pose, aim, rev, hold):
        """A straight leg to `aim`. hold=None: the line from here to aim (after a
        face turn); hold=heading: the goal line, heading held at the goal's."""
        self.aim, self.rev = np.asarray(aim, dtype=float), rev
        if hold is None:
            self.line_mode = False
            self.leg_origin = pose[:2].copy()
            v = self.aim - self.leg_origin
            self.leg_dir = v / max(float(np.hypot(*v)), 1e-9)
            self.leg_hold = math.atan2(self.leg_dir[1], self.leg_dir[0])
            if rev:
                self.leg_hold = wrap(self.leg_hold + math.pi)
        else:
            self.leg_origin = self.goal[:2].copy()
            self.leg_dir = self.u * (-1.0 if rev else 1.0)
            self.leg_hold = hold
        self.after_leg = 'toline' if hold is None else 'plan'
        self.best, self.best_t = 1e9, t
        self.state = 'drive'
        self._checked = False

    def _drive(self, t, pose, pts):
        # progress: signed distance still to go along the leg direction
        left = float((self.aim - pose[:2]) @ self.leg_dir)
        rel = pose[:2] - self.leg_origin
        cross = float(self.leg_dir[0] * rel[1] - self.leg_dir[1] * rel[0])   # + left of the line
        sgn = -1.0 if self.rev else 1.0
        passing = self.pass_ and self.line_mode
        if not self._checked:
            if passing:
                a_now, _ = self.line_coords(pose)
                a_to = float((self.aim - self.goal[:2]) @ self.u)
                ok, why = self.line_clear(pts, pose, a_now, a_to, self.pass_['margin'])
            else:
                ok, why = self.drive_clear(pts, sgn * max(left, 0.0))
            if not ok:
                self._finish('refused', why)
                return 0.0, 0.0
            self._checked = True
            self._blocked = 0
        elif not passing:
            ok, why = self.drive_clear(pts, sgn * max(left, 0.0))   # what is left of the leg
            self._blocked = 0 if ok else self._blocked + 1
            if self._blocked >= self.BLOCK_STEPS:
                self._finish('refused', 'blocked mid-leg: ' + why)
                return 0.0, 0.0
        if passing and not self.backing:
            # the rest of the line, every step: things low down are seen only
            # once the camera is close (depth_obstacles.py, band floor by
            # range), so the corridor can close after the crawl has begun
            a_now, _ = self.line_coords(pose)
            a_to = float((self.aim - self.goal[:2]) @ self.u)
            if a_to - a_now > 0.02:
                ok, why = self.line_clear(pts, pose, a_now + 0.02, a_to, self.pass_['margin'])
                if not ok:
                    self._finish('refused', 'blocked ahead: ' + why)
                    return 0.0, 0.0
        if passing:
            hit = self.contact(pts, sgn, beside=not self.backing)
            if hit:
                self.log.append((t, hit, round(cross, 3)))
                self.backoff = not self.backing
                self.state = 'plan'              # stop; _plan backs out along the line and retries
                return 0.0, 0.0
        if left <= self.ARRIVE:
            if self.after_leg == 'toline':
                self._start_turn(t, pose, self.goal[2], 'align')
            else:
                self.state = 'plan'
            return 0.0, 0.0
        if left < self.best - 0.003:
            self.best, self.best_t = left, t
        if t - self.best_t > self.STALL_S:
            self._finish('stalled', f'drive stuck {left * 100:.1f} cm short')
            return 0.0, 0.0
        vmax = self.pass_['vx'] if passing else self.VX_MAX
        v = max(self.VX_MIN, min(vmax, math.sqrt(2 * self.ACC * left)))
        # hold the heading and steer onto the line. The SAME sign forward and
        # reversing: turning the body clockwise turns the direction of TRAVEL
        # clockwise either way. (Two earlier versions flipped it for reverse
        # and drove off the line backwards, simulator 2026-09-26.)
        he = wrap(self.leg_hold - pose[2])
        wz = self.K_HOLD * he - self.K_CT * cross
        wz = max(-self.WZ_DRIVE, min(self.WZ_DRIVE, wz))
        return sgn * v, wz
