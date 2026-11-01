#!/usr/bin/env python3
"""replay_tight.py -- run tight_route.plan again on what reach saved.

    mkdir -p ~/rover/logs/tight_dumps          # reach saves every failed search there
    python3 phase3/tools/replay_tight.py ~/rover/logs/tight_dumps/*.npz [--budget 8]

Each dump holds the exact points (LiDAR, camera, sonar, confirmed), the target
and the learned pivot reach planned with, so a failure on the robot or the twin
can be searched again offline with changed code, as long as it takes.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import goal_exec as G  # noqa: E402
import tight_route as T  # noqa: E402


def load(f):
    d = np.load(f, allow_pickle=False)
    piv = json.loads(str(d['pivot']))
    ge = G.GoalExec(P_prior=piv)
    for k in (1, -1):
        ge.P_n[k] = int(piv['n'][str(k)])
    return d, ge


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dumps', nargs='+')
    ap.add_argument('--budget', type=float, default=None, help='s (default: what reach had)')
    a = ap.parse_args()
    for f in a.dumps:
        d, ge = load(f)
        mode = Path(f).stem.rsplit('_', 1)[-1]
        pts = d['every'] if mode == 'all' else d['lidar']
        was = json.loads(str(d['info']))
        budget = a.budget if a.budget is not None else float(d['budget_s'])
        steps, info = T.plan(pts, tuple(d['target']), ge, budget_s=budget)
        got = f"FOUND {T.describe(steps)}" if steps else f"none: {info.get('why')}"
        print(f"{Path(f).name}: {len(pts)} points; then: {was.get('why')} ({was.get('expanded')} poses); "
              f"now: {got} ({info.get('expanded')} poses, {info.get('ms')} ms)")


if __name__ == '__main__':
    main()
