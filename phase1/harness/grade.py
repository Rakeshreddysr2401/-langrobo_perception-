#!/usr/bin/env python3
"""grade.py — score every pose estimate in a recorded run against LiDAR truth.

    python3 grade.py BAG_DIR            one run: table + BAG_DIR/grade.json
    python3 grade.py --summary DIR      every graded run under DIR -> DIR/SUMMARY.md

TRUTH (common.py): at every moment the rover stood still, the LiDAR scan is
matched against the scan from the first still moment. That gives where the
rover really was, relative to its start, from the room's walls alone. A
checkpoint only counts if the match is tight (residual <= 2 cm, >= 150 line
points, not corridor-degenerate) AND two independent starting guesses land
on the same answer (within 1 cm and 0.3 deg), so ICP cannot quietly pick the
wrong wall.

GRADED, each relative to its own pose at the first still moment:
    fused         /odom            what the rover uses today (fusion.py)
    vo            /vo/odom         cuVSLAM alone
    wheels        /wheel_ticks     ticks + the symmetric effective track
    wheels+gyro   ticks + gyro     the classic pairing
New estimators (SENSOR_FUSION_PLAN.md stages B-D) are added as topics in
the bag and graded the same way.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (icp, read_bag, relative, se2, se2_parts, still_windows,  # noqa: E402
                    window_scan, wrap)

# the Phase 1 constants the live fusion uses, so "wheels" here is the same
# wheel model the rover runs (and description/build keeps them honest)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
try:
    from fusion import METRES_PER_COUNT, WHEEL_BASE_ROT_M  # noqa: E402
except Exception:  # noqa: BLE001 -- harness still runs off-rover
    METRES_PER_COUNT, WHEEL_BASE_ROT_M = math.pi * 0.085 / 1560.0, 0.5216

AGREE_M, AGREE_RAD = 0.01, math.radians(0.3)
SPLIT_S = 5.0          # long stills are cut into checkpoints: drift while parked


# ── estimators: each is t -> (x, y, yaw) ────────────────────────────────────
def from_odom(a):
    t = a[:, 0]
    yaw = np.unwrap(a[:, 3])
    return lambda q: (float(np.interp(q, t, a[:, 1])), float(np.interp(q, t, a[:, 2])),
                      float(np.interp(q, t, yaw)))


def integrate(t, ds, dth):
    """Midpoint dead reckoning from per-step distance and heading change."""
    x = np.zeros(len(t)); y = np.zeros(len(t)); th = np.zeros(len(t))
    for k in range(1, len(t)):
        h = th[k - 1] + 0.5 * dth[k]
        x[k] = x[k - 1] + ds[k] * math.cos(h)
        y[k] = y[k - 1] + ds[k] * math.sin(h)
        th[k] = th[k - 1] + dth[k]
    return from_odom(np.stack([t, x, y, th], axis=1))


def wheel_steps(b):
    d = np.zeros((len(b.ticks), 4))
    d[1:] = np.diff(b.ticks[:, 1:], axis=0)
    left = 0.5 * (d[:, 0] + d[:, 1]) * METRES_PER_COUNT
    right = 0.5 * (d[:, 2] + d[:, 3]) * METRES_PER_COUNT
    return 0.5 * (left + right), (right - left) / WHEEL_BASE_ROT_M


def gyro_heading(b):
    g = b.gyro
    bias = float(np.median(g[:50, 1]))
    th = np.concatenate([[0.0], np.cumsum(0.5 * (g[1:, 1] + g[:-1, 1] - 2 * bias) * np.diff(g[:, 0]))])
    return lambda q: float(np.interp(q, g[:, 0], th))


def estimators(b):
    e = {}
    if '/odom' in b.odom:
        e['fused'] = from_odom(b.odom['/odom'])
    if '/vo/odom' in b.odom:
        e['vo'] = from_odom(b.odom['/vo/odom'])
    ds, dth = wheel_steps(b)
    e['wheels'] = integrate(b.ticks[:, 0], ds, dth)
    if b.gyro is not None and len(b.gyro) > 50:
        gh = gyro_heading(b)
        t = b.ticks[:, 0]
        head = np.array([gh(q) for q in t])
        e['wheels+gyro'] = integrate(t, ds, np.concatenate([[0.0], np.diff(head)]))
    return e


# ── truth ───────────────────────────────────────────────────────────────────
def checkpoints(b):
    out = []
    for w in still_windows(b):
        n = max(1, int((w[1] - w[0]) // SPLIT_S))
        edges = np.linspace(w[0], w[1], n + 1)
        out += [(edges[k], edges[k + 1]) for k in range(n)]
    return out


def truth(b, cps, est):
    """[(checkpoint, Fit or None, note)] -- pose of each checkpoint in the first's frame."""
    ref, nref = window_scan(b, cps[0])
    rows = [(cps[0], None, f'reference ({nref} scans)')]
    guesses = [k for k in ('fused', 'wheels+gyro', 'wheels') if k in est][:2]
    mid0 = 0.5 * sum(cps[0])
    for w in cps[1:]:
        src, n = window_scan(b, w)
        mid = 0.5 * sum(w)
        fits = []
        for k in guesses:
            g = se2_parts(relative(se2(*est[k](mid0)), se2(*est[k](mid))))
            fits.append(icp(src, ref, g))
        f = fits[0]
        if not f.ok:
            rows.append((w, None, f.why))
            continue
        if len(fits) > 1:
            o = fits[1]
            if not o.ok or math.hypot(f.x - o.x, f.y - o.y) > AGREE_M or abs(wrap(f.th - o.th)) > AGREE_RAD:
                rows.append((w, None, 'two starting guesses disagree: ambiguous match'))
                continue
        rows.append((w, f, f'{n} scans, residual {f.residual * 100:.1f} cm, {f.inliers} pts'))
    return rows


def jumps(a, limit=0.05):
    """Steps in a pose stream bigger than the rover can move between messages."""
    if a is None or len(a) < 2:
        return 0
    return int(np.sum(np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2])) > limit))


# ── grading ─────────────────────────────────────────────────────────────────
def grade(bag_dir):
    bag_dir = Path(bag_dir)
    meta = {}
    if (bag_dir / 'scenario.json').exists():
        meta = json.loads((bag_dir / 'scenario.json').read_text())
    b = read_bag(bag_dir)
    cps = checkpoints(b)
    if len(cps) < 2:
        raise SystemExit(f'{bag_dir.name}: {len(cps)} still moment(s) -- a run needs a still start '
                         'and at least one more still stop to grade')
    est = estimators(b)
    rows = truth(b, cps, est)
    t0 = 0.5 * sum(cps[0])
    mid0 = {k: se2(*f(t0)) for k, f in est.items()}

    print(f'\n  {bag_dir.name}   scenario: {meta.get("scenario", "?")}   '
          f'{len(b.scans)} scans, {len(cps)} still moments, laser {b.laser}')
    names = list(est)
    print('  ' + f'{"t (s)":>7} {"truth x,y cm / deg":>24}  ' +
          ''.join(f'{n + " cm/deg":>18}' for n in names))
    per = {n: [] for n in names}
    cp_out = []
    for w, f, note in rows:
        mid = 0.5 * sum(w)
        if f is None:
            print(f'  {mid - t0:7.1f} {"-":>24}  {note}')
            cp_out.append({'t': mid - t0, 'truth': None, 'note': note})
            continue
        line = f'  {mid - t0:7.1f} {f"{f.x * 100:+6.1f} {f.y * 100:+6.1f} / {math.degrees(f.th):+7.2f}":>24}  '
        errs = {}
        for n in names:
            x, y, th = se2_parts(relative(mid0[n], se2(*est[n](mid))))
            ep, eh = math.hypot(x - f.x, y - f.y), wrap(th - f.th)
            per[n].append((ep, eh))
            errs[n] = {'pos_cm': ep * 100, 'head_deg': math.degrees(eh)}
            line += f'{f"{ep * 100:5.1f} / {math.degrees(eh):+5.2f}":>18}'
        print(line)
        cp_out.append({'t': mid - t0, 'truth': {'x': f.x, 'y': f.y, 'th': f.th,
                                                'residual': f.residual, 'inliers': f.inliers},
                       'errors': errs})
    summary = {}
    for n, v in per.items():
        if not v:
            continue
        p = np.array([e[0] for e in v]) * 100
        h = np.degrees(np.abs([e[1] for e in v]))
        summary[n] = {'final_pos_cm': float(p[-1]), 'max_pos_cm': float(p.max()),
                      'final_head_deg': float(h[-1]), 'max_head_deg': float(h.max()),
                      'checkpoints': len(v)}
    j = {n: jumps(b.odom.get(t)) for n, t in (('fused', '/odom'), ('vo', '/vo/odom')) if t in b.odom}
    print('\n  ' + f'{"":14}{"final cm":>9}{"max cm":>8}{"final deg":>10}{"max deg":>9}{"jumps":>7}')
    for n, s in summary.items():
        print(f'  {n:14}{s["final_pos_cm"]:9.1f}{s["max_pos_cm"]:8.1f}'
              f'{s["final_head_deg"]:10.2f}{s["max_head_deg"]:9.2f}{str(j.get(n, "")):>7}')
    good = sum(1 for r in rows[1:] if r[1] is not None)
    print(f'\n  truth: {good}/{len(rows) - 1} checkpoints usable')
    out = {'bag': bag_dir.name, 'scenario': meta.get('scenario'), 'meta': meta,
           'checkpoints': cp_out, 'summary': summary, 'jumps': j,
           'truth_usable': [good, len(rows) - 1]}
    (bag_dir / 'grade.json').write_text(json.dumps(out, indent=1))
    return out


def summarize(root):
    root = Path(root)
    runs = [json.loads(p.read_text()) for p in sorted(root.glob('*/grade.json'))]
    lines = ['# Localization runs — graded against LiDAR truth', '',
             'Generated by `phase1/harness/grade.py --summary`. Position error in cm, '
             'heading error in degrees, at the LAST usable checkpoint (max in brackets).', '',
             '| run | scenario | truth | ' + ' | '.join(['fused', 'vo', 'wheels', 'wheels+gyro']) + ' |',
             '|---|---|---|' + '---|' * 4]
    for r in runs:
        cells = []
        for n in ('fused', 'vo', 'wheels', 'wheels+gyro'):
            s = r['summary'].get(n)
            cells.append('—' if not s else
                         f'{s["final_pos_cm"]:.1f} ({s["max_pos_cm"]:.1f}) cm / '
                         f'{s["final_head_deg"]:.2f} ({s["max_head_deg"]:.2f})°')
        u = r.get('truth_usable', [0, 0])
        lines.append(f'| {r["bag"]} | {r.get("scenario") or "?"} | {u[0]}/{u[1]} | ' + ' | '.join(cells) + ' |')
    (root / 'SUMMARY.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    a = sys.argv[1:]
    if not a:
        sys.exit(__doc__)
    if a[0] == '--self-test':
        from common import self_test
        sys.exit(0 if self_test(a[1]) else 1)
    if a[0] == '--summary':
        summarize(a[1] if len(a) > 1 else '/logs/bags')
    else:
        for p in a:
            grade(p)
