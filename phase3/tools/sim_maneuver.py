#!/usr/bin/env python3
"""sim_maneuver.py — goal_exec's TURN-AND-SHUFFLE, simulated against the true geometry.

    python3 phase3/tools/sim_maneuver.py [--n 60] [--seed N]

A turn the outline cannot sweep may be made by a short straight move first,
or by the largest safe part of it, a short move, then the rest (goal_exec.py,
TURN-AND-SHUFFLE). Here: random boxes beside and around the rover, turns of
45-135 deg either way, the measured rover model of sim_goal_exec (slide about
the pivot, turn response, noise), in all three pack conditions.

Checked against the TRUE boxes every step: did any part of the outline touch
one? Pass = no touch, ever, and every turn it finishes ends within 2.5 deg
(sim_goal_exec's turn-only bar, measured after the coast). A refusal is
always allowed -- near the edge a fresh, noisy scan can disagree with the
first; those are counted ("planned offline, refused live"), not failed.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from goal_exec import FRONT, REAR, SIDE, GoalExec, R, wrap  # noqa: E402
from sim_goal_exec import DT, Rover, box, scan  # noqa: E402

N = int(sys.argv[sys.argv.index('--n') + 1]) if '--n' in sys.argv else 60
SEED = int(sys.argv[sys.argv.index('--seed') + 1]) if '--seed' in sys.argv else 7
PACKS = [  # (pivot +, pivot -), turn efficiency, arc share -- as sim_goal_exec
    ([(0.0, 0.03), (0.0, -0.03)], 1.0, 0.22),
    ([(0.04, 0.08), (0.02, -0.02)], 0.85, 0.22),
    ([(0.06, 0.12), (0.05, -0.06)], 0.7, 0.20),
]


def seg_points(segs, step=0.004):
    out = []
    for (x1, y1), (x2, y2) in segs:
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / step))
        out.append(np.stack([np.linspace(x1, x2, n), np.linspace(y1, y2, n)], 1))
    return np.vstack(out)


def touches(pose, solid):
    q = (solid - pose[:2]) @ R(pose[2])
    return bool(((q[:, 0] < FRONT) & (q[:, 0] > -REAR) & (np.abs(q[:, 1]) < SIDE)).any())


def one(rng, a, boxes, pack):
    P, eff, arc = pack
    room = box(0.0, 0.0, 3.2, 3.2)
    segs = room + [s for b in boxes for s in b]
    solid = seg_points([s for b in boxes for s in b])
    rov = Rover(P, eff, arc, rng)
    if touches(rov.x, solid):
        return None
    ex = GoalExec({'1': list(P[0]), '-1': list(P[1])})
    pts0 = scan(rov.x, segs, rng)
    direct = ex.turn_clear(pts0, a)[0]
    plan = None if direct else ex.maneuver(pts0, a)
    ex.set_goal((0.0, 0.0, wrap(a)), turn_only=True)
    t, touched, pts = 0.0, False, pts0
    while ex.state != 'done' and t < 90.0:
        pts = scan(rov.x, segs, rng) if int(t / DT) % 2 == 0 else pts
        vx, wz = ex.step(t, rov.sensed(), pts)
        rov.step(vx, wz)
        touched |= touches(rov.x, solid)
        t += DT
    for _ in range(20):
        rov.step(0.0, 0.0)
        touched |= touches(rov.x, solid)
    eh = abs(math.degrees(wrap(a - rov.x[2])))
    return dict(direct=direct, plan=plan, out=ex.result, why=ex.why, eh=eh, touched=touched,
                shuffles=ex.maneuvers, t=t)


def main():
    rng = np.random.default_rng(SEED)
    bad, stats = [], {'direct': 0, 'shuffled': 0, 'refused': 0, 'skipped': 0, 'offline_fit': 0}
    k = 0
    while k < N:
        a = math.radians(rng.choice([45, 60, 90, 120, 135])) * rng.choice([-1, 1])
        boxes = []
        for _ in range(rng.integers(1, 3)):
            ang = rng.uniform(-math.pi, math.pi)
            r = rng.uniform(0.27, 0.42)
            w, h = rng.uniform(0.06, 0.3), rng.uniform(0.06, 0.3)
            boxes.append(box(r * math.cos(ang), r * math.sin(ang), w, h))
        res = one(rng, a, boxes, PACKS[k % 3])
        if res is None:
            stats['skipped'] += 1
            continue
        k += 1
        tag = f'#{k:02d} turn {math.degrees(a):+4.0f} pack {k % 3}'
        if res['touched']:
            bad.append(f'{tag}: TOUCHED ({res["out"]}: {res["why"]})')
        if res['out'] == 'reached':
            stats['shuffled' if res['shuffles'] else 'direct'] += 1
            if res['eh'] > 2.5:
                bad.append(f'{tag}: reached but {res["eh"]:.1f} deg off')
        elif res['out'] == 'refused':
            stats['refused'] += 1
            if res['plan']:
                stats['offline_fit'] += 1
        else:
            bad.append(f'{tag}: {res["out"]}: {res["why"]}')
        if res['shuffles'] or res['touched']:
            print(f'  {tag}: {res["out"]:8s} shuffles {res["shuffles"]}  {res["eh"]:.1f} deg  '
                  f'{res["t"]:.0f} s  plan {res["plan"]}  {res["why"]}')
    print(f'\n  {N} turns: direct {stats["direct"]}, with a shuffle {stats["shuffled"]}, '
          f'refused {stats["refused"]} (of them planned offline, refused live: {stats["offline_fit"]}; '
          f'layouts already touching, skipped: {stats["skipped"]})')
    for b in bad:
        print(f'   ! {b}')
    print('\nSUITE', 'PASS' if not bad else 'FAIL')
    return 0 if not bad else 1


if __name__ == '__main__':
    sys.exit(main())
