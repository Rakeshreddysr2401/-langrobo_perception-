#!/usr/bin/env python3
"""sim_gap_pass.py — goal_exec's PASS MODE through tight gaps, simulated.

    python3 phase3/tools/sim_gap_pass.py [--seed N]

Gaps of 50 / 48 / 46 / 43 cm between two 30 cm-deep blocks, the gap's centre
line slanted, the rover starting off it and turned ~20 deg (as it stood at the
real gap, LOCALIZATION.md §13). The rover model is sim_goal_exec's measured
one, in all three pack conditions. Checked against the TRUE geometry every
step: did any part of the outline touch a block? With the owner's 3 cm pass
margin the rover needs 38 + 2 x 3 = 44 cm in theory; with ~1 cm of pose and
point error per side, 50 and 48 must pass without a touch, 46 is the boundary
(either answer, never a touch), 43 must be refused. --noise sets the LiDAR's
range sd (default 3 mm, measured).
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from goal_exec import FRONT, REAR, SIDE, GoalExec, R, wrap  # noqa: E402
from sim_goal_exec import DT, Rover, box, scan  # noqa: E402

MARGIN, VX = 0.03, 0.05
NOISE = float(sys.argv[sys.argv.index('--noise') + 1]) if '--noise' in sys.argv else 0.003   # m, measured C1


def blocks(gap, phi, c, xg, depth=0.30, width=0.6):
    """Two blocks leaving `gap` between them, centre line at angle phi through
    (xg along, c across) in the start frame."""
    segs, solid = [], []
    for side in (+1, -1):
        cy = side * (gap / 2 + width / 2)
        # block in the gap frame, then rotated/translated
        loc = box(0.0, cy, depth, width)
        for (a, b) in loc:
            A = R(phi) @ np.array(a) + R(phi) @ np.array([xg, c])
            B = R(phi) @ np.array(b) + R(phi) @ np.array([xg, c])
            segs.append((tuple(A), tuple(B)))
            n = int(np.hypot(*(B - A)) / 0.004) + 2
            solid.append(np.linspace(A, B, n))
    return segs, np.vstack(solid)


def touches(pose, solid):
    q = (solid - pose[:2]) @ R(pose[2])
    inside = (q[:, 0] < FRONT) & (q[:, 0] > -REAR) & (np.abs(q[:, 1]) < SIDE)
    if inside.any():
        return 0.0
    # clearance: distance from outline to nearest block point
    dx = np.maximum(np.maximum(q[:, 0] - FRONT, -REAR - q[:, 0]), 0)
    dy = np.maximum(np.abs(q[:, 1]) - SIDE, 0)
    return float(np.min(np.hypot(dx, dy)))


def run(gap, cond, rng, noise, t_max=180.0):
    P, eff, arc = cond
    phi = math.radians(rng.uniform(-10, 10))
    c = rng.uniform(-0.04, 0.04)
    xg = 1.0                                    # constriction along the line
    segs, solid = blocks(gap, phi, c, xg)
    room = box(0.3, 0.0, 4.0, 3.0)
    rov = Rover(P, eff, arc, rng)
    rov.x = np.array([0.0, rng.uniform(-0.05, 0.05), phi + math.radians(rng.choice([-20, 20]))])
    u = np.array([math.cos(phi), math.sin(phi)])
    n = np.array([-u[1], u[0]])
    p0 = R(phi) @ np.array([xg, c])             # constriction centre, world
    end = p0 + (REAR + 0.15 + 0.15) * u         # rear 15 cm past the blocks' far face
    pre = p0 - (0.15 + FRONT + 0.30) * u        # front 30 cm before the near face
    L = float((end - pre) @ u)
    ex = GoalExec({'1': list(P[0]), '-1': list(P[1])})     # the pivot as the node would have learned it
    ex.set_goal((end[0], end[1], phi), pass_=dict(L=L, margin=MARGIN, vx=VX))
    t, worst = 0.0, 9.0
    pts = None
    while ex.state != 'done' and t < t_max:
        if int(t / DT) % 2 == 0:
            pts = scan(rov.x, segs + room, rng, noise)
        vx, wz = ex.step(t, rov.sensed(), pts)
        rov.step(vx, wz)
        worst = min(worst, touches(rov.x, solid))
        t += DT
    for _ in range(20):
        rov.step(0.0, 0.0)
        worst = min(worst, touches(rov.x, solid))
    e = math.hypot(*(rov.x[:2] - end)) * 100
    return ex.result or 'timeout', ex.why, e, worst, t, ex.tries


def main():
    seed = int(sys.argv[sys.argv.index('--seed') + 1]) if '--seed' in sys.argv else 1
    rng = np.random.default_rng(seed)
    conds = {'charged': ([(0.0, 0.03), (0.0, -0.03)], 1.0, 0.22),
             'weak pack': ([(0.017, 0.277), (0.017, 0.277)], 0.6, 0.15),
             'asymmetric': ([(0.03, 0.12), (-0.02, -0.05)], 0.9, 0.20)}
    ok = True
    # 46 cm is the boundary: 1 cm of slack per side beyond the 3 cm margin is
    # about the pose + point error, so either answer is right -- if it never touches
    for gap, want in ((0.50, 'reached'), (0.48, 'reached'), (0.46, None), (0.43, 'refused')):
        for name, cond in conds.items():
            for k in range(3):
                res, why, e, worst, t, tries = run(gap, cond, rng, NOISE)
                good = (res == want if want else res in ('reached', 'refused')) and worst > 0.0
                ok &= good
                print(f'  {"ok " if good else "BAD"} gap {gap*100:.0f} cm  {name:10s}  {res:8s} end {e:4.1f} cm  '
                      f'closest {worst*100:4.1f} cm  {t:5.1f} s  tries {tries}' + ('' if res == 'reached' else f'  -- {why}'))
    print('\nSUITE', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
