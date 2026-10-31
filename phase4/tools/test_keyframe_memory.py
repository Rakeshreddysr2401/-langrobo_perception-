#!/usr/bin/env python3
"""test_keyframe_memory.py -- offline checks of the 10-minute frame memory.

    python3 phase4/tools/test_keyframe_memory.py    (no robot, no ROS graph, no model)
    in the container: docker exec rover python3 /opt/rover4/tools/test_keyframe_memory.py

pixel_to_goal: a keyframe's depth goes to its OWN store and never pushes the
brain's 24 photos out; it comes back bit-exact; the store is bounded; a query
by a keyframe's stamp finds it. keyframe_memory: hits are the best frames,
spread over moments and places; confidence is judged against the reference
photos; frames older than the window are forgotten; an odom restart empties it.
"""
import importlib.util
import json
import logging
import sys
import time
import types
from collections import OrderedDict, deque
from pathlib import Path

import numpy as np

here = Path(__file__).resolve().parent
for name in ("rclpy", "rclpy.node", "rclpy.time", "rclpy.duration", "rclpy.qos",
             "tf2_ros", "geometry_msgs", "geometry_msgs.msg", "sensor_msgs",
             "sensor_msgs.msg", "std_msgs", "std_msgs.msg", "cv_bridge", "nav_msgs", "nav_msgs.msg"):
    if name not in sys.modules:
        try:
            importlib.import_module(name)
        except Exception:
            m = types.ModuleType(name)
            m.__getattr__ = lambda attr: type(attr, (), {})
            sys.modules[name] = m


def load(mod, file):
    spec = importlib.util.spec_from_file_location(mod, here.parent / "nodes" / file)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


p2g = load("p2g", "pixel_to_goal.py")
kfm = load("kfm", "keyframe_memory.py")
log = logging.getLogger("test")
failures = []


def check(name, ok, detail=""):
    print(("PASS  " if ok else "FAIL  ") + name + (f"   ({detail})" if detail and not ok else ""))
    if not ok:
        failures.append(name)


class Stamp:
    def __init__(self, ns):
        self.sec, self.nanosec = divmod(ns, 1_000_000_000)


class Msg:
    def __init__(self, ns):
        self.header = types.SimpleNamespace(stamp=Stamp(ns))


# ── pixel_to_goal: the keyframe store ───────────────────────────────────────

def bare_p2g():
    n = p2g.PixelToGoal.__new__(p2g.PixelToGoal)
    n.get_logger = lambda: log
    n._snaps, n._keyframes, n._kf_wanted, n._snap_pending = OrderedDict(), OrderedDict(), OrderedDict(), []
    n._scans = deque()
    depth = (np.arange(252 * 448, dtype=np.uint32) % 4000).astype(np.uint16).reshape(252, 448)
    n._make_snapshot = lambda ns, gap_tf=None: (n._store_snapshot(ns, depth, f"tf@{ns}", 0.01), "")
    n._scan_near = lambda ns: None
    return n, depth


n, depth = bare_p2g()
for i in range(24):                                   # the brain's 24 photos
    n._on_snapshot(Msg(1_000 + i))
for i in range(100):                                  # then 100 keyframes while driving
    n._on_keyframe_snapshot(Msg(10_000 + i))
check("keyframes never push the brain's photos out", len(n._snaps) == 24 and 1_000 in n._snaps,
      f"{len(n._snaps)} photos")
check("keyframes go to their own store", len(n._keyframes) == 100 and 10_000 not in n._snaps)
snap = n._keyframe_snap(10_042)
check("keyframe depth comes back bit-exact", snap is not None and np.array_equal(snap["depth"], depth))
check("keyframe keeps its camera pose", snap is not None and snap["cam_tf"] == "tf@10042")
check("PNG is smaller than raw depth", len(n._keyframes[10_042]["png"]) < depth.nbytes,
      f'{len(n._keyframes[10_042]["png"])} vs {depth.nbytes}')

n2, _ = bare_p2g()
for i in range(p2g.KEYFRAMES + 50):
    n2._on_keyframe_snapshot(Msg(50_000 + i))
check("keyframe store is bounded", len(n2._keyframes) == p2g.KEYFRAMES and 50_000 not in n2._keyframes)
n2._keyframes[next(iter(n2._keyframes))]["kept"] -= p2g.KEYFRAME_MAX_S + 1
check("an expired keyframe is not served", n2._keyframe_snap(next(iter(n2._keyframes))) is None)

n3, _ = bare_p2g()
n3._on_snapshot(Msg(7))                               # the brain held a frame ...
n3._on_keyframe_snapshot(Msg(7))                      # ... that is also a keyframe
check("a frame held by both is in both stores", 7 in n3._snaps and 7 in n3._keyframes)
n3._kf_wanted[8] = True                               # a keyframe still pending ...
n3._on_snapshot(Msg(8))                               # ... the brain asks for the same frame
check("the brain's photo wins a pending keyframe stamp", 8 in n3._snaps)

# ── keyframe_memory: search, spread, confidence, forgetting ─────────────────

def unit(v):
    v = np.asarray(v, np.float32)
    return v / np.linalg.norm(v)


def vecs(direction, strength=1.0):
    v = np.full((5, 768), 0.01, np.float32)
    v[:, direction] += strength
    return np.stack([unit(r) for r in v])


def bare_kfm():
    m = kfm.KeyframeMemory.__new__(kfm.KeyframeMemory)
    m.get_logger = lambda: log
    m.frames, m.lock, m.ref, m.last_kf, m.origin = deque(), __import__("threading").Lock(), None, None, None
    m.stats = {"cleared": 0}
    return m


def kf(t_ago, x, y, yaw, v, now):
    return kfm.Keyframe(int((now - t_ago) * 1e9), 0, 0, now - t_ago, x, y, yaw, b"jpg", 448, 252, v)


now = time.time()
m = bare_kfm()
m.frames.extend([kf(500, 0, 0, 0, vecs(1), now), kf(400, 1, 0, 0, vecs(1), now),
                 kf(300, 2, 0, 0, vecs(5), now),                         # the fridge
                 kf(299, 2.05, 0, 0, vecs(5, 0.9), now),                 # same moment, same place
                 kf(100, 4, 0, 0, vecs(5, 0.8), now),                    # a second fridge view elsewhere
                 kf(10, 5, 0, 0, vecs(2), now)])
q = unit(np.eye(768)[5])
r = m.search(q, k=3, with_jpeg=False)
check("best frame first", r["hits"][0]["robot"]["x"] == 2.0, r["hits"][0]["robot"])
check("one hit per moment and place", [h["robot"]["x"] for h in r["hits"][:2]] == [2.0, 4.0],
      [h["robot"]["x"] for h in r["hits"]])
check("a hit says how long ago", abs(r["hits"][0]["age_s"] - 300) < 2)
check("without reference photos, judged against the other frames", r["baseline_from"] == "other frames")

m.ref = np.stack([vecs(9) for _ in range(10)])       # unrelated photos
r = m.search(q, k=1, with_jpeg=False)
check("with reference photos, a clear match is strong", r["hits"][0]["confidence"] == "strong", r["hits"][0])
r = m.search(unit(np.eye(768)[9]), k=1, with_jpeg=False)
check("words that match only the reference are weak", r["hits"][0]["confidence"] == "weak", r["hits"][0])

old = bare_kfm()
old.frames.extend([kf(kfm.WINDOW_S + 5, 0, 0, 0, vecs(1), now), kf(5, 1, 0, 0, vecs(1), now)])
r = old.search(q, k=3, with_jpeg=False)
check("frames older than the window are forgotten", r["frames"] == 1, r["frames"])

fz = bare_kfm()
fz.frames.extend([kf(5, 0, 0, 0, vecs(1), now)])
fz._on_fusion(types.SimpleNamespace(data=json.dumps({"origin_epoch": 1.0})))
fz._on_fusion(types.SimpleNamespace(data=json.dumps({"origin_epoch": 1.0})))
check("same odom origin keeps the memory", len(fz.frames) == 1)
fz._on_fusion(types.SimpleNamespace(data=json.dumps({"origin_epoch": 2.0})))
check("an odom restart empties the memory", len(fz.frames) == 0 and fz.stats["cleared"] == 1)

pk = bare_kfm()
pk.stats.update(kept=0, same_view=0)
pk._store(kf(590, 0, 0, 0, vecs(1), now), "first")
how = pk._store(kf(0, 0, 0, 0, vecs(1), now), "refresh")          # parked, same view, 10 min later
r = pk.search(q, k=1, with_jpeg=False)
check("parked: a same-view refresh replaces the old frame", how == "replaced" and len(pk.frames) == 1)
check("parked: the room it is looking at is never forgotten", r["frames"] == 1 and r["hits"][0]["age_s"] < 2,
      r["hits"][0]["age_s"] if r["hits"] else "empty")
check("a refresh of a CHANGED view is kept as new", pk._store(kf(0, 0, 0, 0, vecs(7), now), "refresh") == "kept"
      and len(pk.frames) == 2)
check("a move is always kept, even if it looks the same",
      pk._store(kf(0, 1, 0, 0, vecs(7), now), "moved") == "kept" and len(pk.frames) == 3)

boxes = kfm.tile_boxes(448, 252)
check("tile boxes cover the frame", boxes[0] == [0, 0, 448, 252] and boxes[4] == [224, 126, 448, 252])

print(f"\n{'ALL PASS' if not failures else str(len(failures)) + ' FAILED'}")
sys.exit(1 if failures else 0)
