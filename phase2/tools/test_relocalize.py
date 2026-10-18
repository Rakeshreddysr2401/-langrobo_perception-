"""test_relocalize.py -- relocalize.py on synthetic rooms: a ray-cast scan from a known pose must
lock within 5 cm / 2 deg; a symmetric room and a foreign room must NOT lock (never guess).

    python3 phase2/tools/test_relocalize.py      (numpy + cv2; laptop or container)
"""
import math, time, numpy as np
import sys, pathlib; sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent)); import relocalize as R

def room(res=0.05, symmetric=False):
    W, H = int(8 / res), int(6 / res)
    occ = np.zeros((H, W), bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    if not symmetric:                                 # an L: a wall stub, a sofa, a table leg
        occ[:int(3.5 / res), int(5 / res)] = True
        occ[int(4 / res):int(4.8 / res), int(1 / res):int(3 / res)] = True
        occ[int(2 / res):int(2.1 / res), int(2.5 / res):int(2.6 / res)] = True
    free = ~occ
    return occ, free, res, 0.0, 0.0

def raycast(occ, res, pose, n=720, noise=0.01, rng=np.random.default_rng(0)):
    x, y, th = pose; out = []
    for a in np.linspace(-math.pi, math.pi, n, endpoint=False):
        for r in np.arange(0.1, 8, res / 2):
            u, v = int((x + r * math.cos(th + a)) / res), int((y + r * math.sin(th + a)) / res)
            if not (0 <= u < occ.shape[1] and 0 <= v < occ.shape[0]) or occ[v, u]:
                out.append(r + rng.normal(0, noise)); break
        else: out.append(float('inf'))
    return out

occ, free, res, ox, oy = room()
for truth in [(2.0, 2.5, 0.3), (6.5, 1.2, -2.0), (1.2, 1.0, 2.8)]:
    ranges = raycast(occ, res, truth)
    pts = R.scan_points(ranges, -math.pi, 2 * math.pi / 720, (0.134, 0.0, math.radians(88.6)))
    # the scan is in the LASER frame: undo the mount, as the real one is
    lx, ly, lth = 0.134, 0.0, math.radians(88.6)
    # raycast was from the base pose with base-frame angles -> rebuild pts in base_link directly
    a = np.linspace(-math.pi, math.pi, 720, endpoint=False); r = np.array(ranges); ok = np.isfinite(r)
    pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1)[::4]
    t = time.time(); out = R.relocalize(occ, free, res, ox, oy, pts); dt = time.time() - t
    p = out['pose']; err = math.hypot(p[0] - truth[0], p[1] - truth[1]); aerr = math.degrees(R._wrap(p[2] - truth[2]))
    print(f"truth {truth} -> {out['verdict']} pose ({p[0]:.3f},{p[1]:.3f},{math.degrees(p[2]):.1f}) err {100*err:.1f} cm {aerr:.1f} deg  {out['why']}  {dt:.1f}s")
    assert out['verdict'] == 'locked' and err < 0.05 and abs(aerr) < 2
# symmetric box room: a pose and its 180-degree twin both fit
occ2, free2, *_ = room(symmetric=True)
r2 = raycast(occ2, res, (2.0, 2.0, 0.0)); a = np.linspace(-math.pi, math.pi, 720, endpoint=False); r = np.array(r2)
pts = np.stack([r * np.cos(a), r * np.sin(a)], 1)[::4]
out = R.relocalize(occ2, free2, res, 0, 0, pts); print('symmetric room ->', out['verdict'], out['why'])
assert out['verdict'] == 'ambiguous'
# a scan from somewhere else entirely (a different room) -> lost
other = np.zeros_like(occ); other[0, :] = other[-1, :] = other[:, 0] = other[:, -1] = True; other[:, int(2/res)] = True
r3 = raycast(other, res, (1.0, 3.0, 0.0)); r = np.array(r3); pts = np.stack([r * np.cos(a), r * np.sin(a)], 1)[::4]
pts = pts[np.isfinite(pts).all(1)]
out = R.relocalize(occ, free, res, 0, 0, pts); print('foreign room ->', out['verdict'], out['why'])
assert out['verdict'] in ('lost', 'ambiguous')
print('ALL OK')
