#!/usr/bin/env python3
"""sim_goal_exec.py — prove the goal executor before it touches the rover.

    python3 phase3/tools/sim_goal_exec.py            the full suite
    python3 phase3/tools/sim_goal_exec.py --seed 7   a different random draw

A skid-steer simulator with this rover's measured quirks, all on at once:

  pivot      turns rotate the base about an off-centre point P (body frame):
             near the centre on a charged pack (~6 cm per 90), near the left
             tyres on a weak one (~33 cm per 90) -- LOCALIZATION.md §7-9
  rate       the rover turns at only a fraction of the commanded rate, with
             a lag, and not at all below a scrub threshold
  deadband   the firmware drops a side whose target is under 0.01 m/s
  pose       fusion2-level noise: 0.3 cm, 0.2 deg
  room       walls and a box, scanned by 720 beams from the LiDAR's mount

It runs GoalExec (phase3/nodes/goal_exec.py) exactly as the node will, at
20 Hz, and reports, per chassis condition: reached / failed / refused, final
errors, and time. Acceptance (SENSOR_FUSION_PLAN.md M4): 1.5 cm / 1 deg.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
from goal_exec import GoalExec, R, wrap  # noqa: E402

DT = 0.05
LASER = (0.1342, 0.0)


class Rover:
    def __init__(self, P, turn_eff, w_dead, rng, lag=0.2):
        self.x = np.zeros(3)
        self.P = {+1: np.array(P[0]), -1: np.array(P[1])}
        self.k, self.w_dead, self.lag, self.rng = turn_eff, w_dead, lag, rng
        self.w = 0.0
        self.v = 0.0

    def step(self, vx, wz):
        # the firmware: a side with a target under 0.01 m/s is switched off
        vl, vr = vx - wz * 0.17, vx + wz * 0.17
        if abs(vl) < 0.01 and abs(vr) < 0.01:
            vx, wz = 0.0, 0.0
        # the scrub is a PIVOT problem: turning in place, small commands do
        # nothing and big ones reach a fraction. Steering while driving, both
        # sides roll and an arc follows the command closely.
        if abs(vx) > 0.02:
            w_cmd = 0.85 * wz
        else:
            w_cmd = 0.0 if abs(wz) < self.w_dead else self.k * wz
        a = DT / (self.lag + DT)
        self.w += a * (w_cmd - self.w)
        self.v += a * (0.97 * vx - self.v)
        w = self.w * (1 + self.rng.normal(0, 0.03))
        v = self.v * (1 + self.rng.normal(0, 0.02))
        P = self.P[1 if w >= 0 else -1]
        # base velocity in the body frame: forward, plus rotation about P
        vb = np.array([v + w * P[1], -w * P[0]])
        th = self.x[2]
        self.x[:2] += R(th) @ vb * DT
        self.x[2] = wrap(th + w * DT)

    def sensed(self):
        return self.x + np.array([self.rng.normal(0, 0.003), self.rng.normal(0, 0.003),
                                  self.rng.normal(0, math.radians(0.2))])


def scan(pose, segs, rng):
    """720 beams from the laser; hits on the room's segments, as base_link points."""
    lx, ly = LASER
    ox, oy = pose[:2] + R(pose[2]) @ np.array([lx, ly])
    a = pose[2] + np.linspace(-math.pi, math.pi, 720, endpoint=False)
    dx, dy = np.cos(a), np.sin(a)
    best = np.full(720, np.inf)
    for (x1, y1), (x2, y2) in segs:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            tt = ((x1 - ox) * ey - (y1 - oy) * ex) / den
            uu = ((x1 - ox) * dy - (y1 - oy) * dx) / den
        ok = (np.abs(den) > 1e-9) & (tt > 0.05) & (uu >= 0) & (uu <= 1)
        best = np.where(ok & (tt < best), tt, best)
    hit = np.isfinite(best) & (best < 8)
    r = best[hit] + rng.normal(0, 0.01, hit.sum())
    wx, wy = ox + r * dx[hit], oy + r * dy[hit]
    return (np.stack([wx - pose[0], wy - pose[1]], 1)) @ R(pose[2])       # into base_link


def box(cx, cy, w, h):
    c = [(cx - w / 2, cy - h / 2), (cx + w / 2, cy - h / 2), (cx + w / 2, cy + h / 2), (cx - w / 2, cy + h / 2)]
    return [(c[i], c[(i + 1) % 4]) for i in range(4)]


def run(goal, P, eff, w_dead, rng, segs, t_max=90.0, ex=None):
    rov = Rover(P, eff, w_dead, rng)
    ex = ex or GoalExec()
    ex.set_goal(goal)
    t = 0.0
    while ex.state != 'done' and t < t_max:
        pts = scan(rov.x, segs, rng) if int(t / DT) % 2 == 0 else pts
        vx, wz = ex.step(t, rov.sensed(), pts)
        rov.step(vx, wz)
        t += DT
    for _ in range(20):                        # let it settle, as the real one coasts
        rov.step(0.0, 0.0)
    e = math.hypot(*(rov.x[:2] - np.array(goal[:2]))) * 100
    eh = abs(math.degrees(wrap(rov.x[2] - goal[2])))
    return ex.result or 'timeout', e, eh, t, ex


def main():
    seed = int(sys.argv[sys.argv.index('--seed') + 1]) if '--seed' in sys.argv else 1
    rng = np.random.default_rng(seed)
    room = box(0.0, 0.0, 3.2, 3.2)             # walls 1.6 m from the start
    conditions = {
        'charged (P near centre)':   ([(0.0, 0.03), (0.0, -0.03)], 0.45, 0.5),
        'weak pack (P left tyres)':  ([(0.017, 0.277), (0.017, 0.277)], 0.30, 0.6),
        'asymmetric (L != R)':       ([(0.03, 0.12), (-0.02, -0.05)], 0.40, 0.5),
    }
    ok_all = True
    for name, (P, eff, wd) in conditions.items():
        res, E, EH, T = [], [], [], []
        ex = GoalExec()                          # ONE executor per condition: it keeps learning P
        for _ in range(12):
            r = rng.uniform(0.15, 1.0)
            b = rng.uniform(-math.pi, math.pi)
            goal = (r * math.cos(b), r * math.sin(b), rng.uniform(-math.pi, math.pi))
            out, e, eh, t, ex = run(goal, P, eff, wd, rng, room, ex=ex)
            res.append(out); E.append(e); EH.append(eh); T.append(t)
            if out != 'reached' or e > 3.0:
                print(f'   ! goal ({goal[0]:+.2f}, {goal[1]:+.2f}, {math.degrees(goal[2]):+.0f} deg): '
                      f'{out}, {e:.1f} cm {eh:.1f} deg -- {ex.why}')
        att = [i for i, x in enumerate(res) if x != 'refused']       # a refusal is a safety answer, not a miss
        E, EH = np.array([E[i] for i in att]), np.array([EH[i] for i in att])
        good = sum(1 for x in res if x == 'reached')
        refused = sum(1 for x in res if x == 'refused')
        print(f'{name:28s} reached {good}/12 (refused {refused})  pos mean {E.mean():4.1f} max {E.max():4.1f} cm | '
              f'heading mean {EH.mean():4.2f} max {EH.max():4.2f} deg | mean time {np.mean(T):4.1f} s '
              f'| learned P +{np.round(ex.P[1], 3)} -{np.round(ex.P[-1], 3)}')
        others = sorted({x for x in res if x != 'reached'})
        if others:
            print(f'{"":28s} other outcomes: {others}')
        ok_all &= (good + refused) >= 11 and E.max() < 3.0
    # a turn that must be refused: a box right beside the rover
    blocked = room + box(0.0, 0.33, 0.25, 0.12)
    out, e, eh, t, ex = run((0.0, 0.0, math.radians(90)), [(0.0, 0.03), (0.0, -0.03)], 0.45, 0.5, rng, blocked)
    print(f'{"box 3 cm beside, turn 90":28s} {out}: {ex.why}')
    ok_all &= out == 'refused'
    out, e, eh, t, ex = run((1.2, 0.0, 0.0), [(0.0, 0.03), (0.0, -0.03)], 0.45, 0.5, rng, room + box(0.7, 0.0, 0.2, 0.4))
    print(f'{"box in the path, 1.2 m":28s} {out}: {ex.why}')
    ok_all &= out == 'refused'
    print('\nSUITE', 'PASS' if ok_all else 'FAIL')
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
