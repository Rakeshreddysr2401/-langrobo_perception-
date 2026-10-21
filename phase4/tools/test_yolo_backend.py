#!/usr/bin/env python3
"""test_yolo_backend.py — is the TensorRT detector the same detector, and faster?

    # in the rover container, the way ./rover detect runs the node:
    LD_LIBRARY_PATH=/hosttrt/lib:$LD_LIBRARY_PATH PYTHONPATH=/hosttrt/py:$PYTHONPATH \
      python3 phase4/tools/test_yolo_backend.py [frames_dir]

1. decode(): boxes, letterbox undo, class-aware NMS -- synthetic, no GPU.
2. With a frames dir (JPEGs, e.g. extracted from a bag) and both models in
   /models: engine vs .pt on every frame. PASS when >= 95 % of the .pt's boxes
   are found (same label, IoU >= 0.5) and every difference sits within 0.03 of
   the 0.25 floor (FP16 rounding), and prints ms / CPU ms per frame.
3. Fallback: an engine whose header names another TensorRT is refused and the
   .pt is used -- the detector always comes up.
"""
import glob
import json
import os
import struct
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "nodes"))
import yolo_backend as yb  # noqa: E402

ENGINE = os.environ.get("ENGINE") or next(iter(sorted(glob.glob("/models/yolov8n_640x384_trt*.engine"))), "")
PT = os.environ.get("PT", "/models/yolov8n.pt")
fails = []


def check(ok, what):
    print(("  PASS " if ok else "  FAIL ") + what)
    if not ok:
        fails.append(what)


def iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    i = max(0, x2 - x1) * max(0, y2 - y1)
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
    return i / u if u > 0 else 0.0


def test_decode():
    print("decode (synthetic)")
    names = {0: "person", 1: "bottle"}
    # 5 candidates, (4 + 2 classes, N): cx, cy, w, h in the 640x384 letterboxed frame
    cand = [  # cx, cy, w, h, p_person, p_bottle
        (320, 192, 100, 200, 0.90, 0.10),   # person A
        (322, 194, 100, 200, 0.80, 0.05),   # same person, lower: NMS drops it
        (322, 194, 100, 200, 0.05, 0.60),   # a bottle in the same place: other class, kept
        (100, 100, 20, 40, 0.10, 0.20),     # under the 0.25 floor: dropped
        (600, 300, 60, 60, 0.30, 0.00),     # weak person, kept
    ]
    out = np.array(cand, np.float32).T
    # 448x252 image letterboxed into 640x384: r = 640/448, pad top (384-360)/2 = 12
    r, left, top = 640 / 448, 0, 12
    got = yb.decode(out, r, left, top, 448, 252, names)
    check([g[0] for g in got] == ["person", "bottle", "person"], f"kept person, bottle, weak person: {[(g[0], round(g[1], 2)) for g in got]}")
    x1, y1, x2, y2 = got[0][2]
    check(abs(x1 - (270 / r)) < 0.01 and abs(y1 - (92 - top) / r) < 0.01, "box mapped back through the letterbox")
    check(all(0 <= v <= 448 for v in (x1, x2)) and all(0 <= v <= 252 for v in (y1, y2)), "box inside the image")
    check(yb.decode(out[:, 3:4], r, left, top, 448, 252, names) == [], "nothing over the floor -> []")


def test_parity(frames_dir):
    import cv2
    print(f"engine vs .pt on {frames_dir}")
    if not (ENGINE and os.path.exists(ENGINE) and os.path.exists(PT)):
        print(f"  SKIP: need {ENGINE or '/models/yolov8n_640x384_trt*.engine'} and {PT}")
        return
    frames = [cv2.imread(f) for f in sorted(glob.glob(os.path.join(frames_dir, "*.jpg")))]
    if not frames:
        print("  SKIP: no frames")
        return
    res = {}
    for name, det in (("engine", yb.load_detector(ENGINE)), ("pt", yb.load_detector(PT))):
        print(f"  {name}: {det.backend}")
        check(not (name == "engine" and not det.backend.startswith("tensorrt")), f"{name} runs on {det.backend.split()[0]}")
        for f in frames[:10]:
            det.detect(f)
        wall, cpu, dets = [], [], []
        for f in frames:
            t0, c0 = time.perf_counter(), time.process_time()
            dets.append(det.detect(f))
            wall.append((time.perf_counter() - t0) * 1e3)
            cpu.append((time.process_time() - c0) * 1e3)
        res[name] = dets
        print(f"    {np.median(wall):.1f} ms/frame (p90 {np.percentile(wall, 90):.1f}), CPU {np.median(cpu):.1f} ms/frame")
    found = total = 0
    odd = []
    for de, dp in zip(res["engine"], res["pt"]):
        used = set()
        for lp, cp, bp in dp:
            total += 1
            m = [j for j, (le, ce, be) in enumerate(de) if j not in used and le == lp and iou(bp, be) >= 0.5]
            if m:
                found += 1
                used.add(m[0])
            else:
                odd.append(cp)
        odd += [ce for j, (le, ce, be) in enumerate(de) if j not in used]
    print(f"    .pt boxes found by the engine: {found}/{total}; differences at conf {sorted(round(c, 3) for c in odd)}")
    check(total == 0 or found / total >= 0.95, ">= 95 % of the .pt's boxes")
    check(all(c < yb.CONF_FLOOR + 0.03 for c in odd), "every difference is at the confidence floor")


def test_fallback():
    print("fallback")
    if not (ENGINE and os.path.exists(ENGINE) and os.path.exists(PT)):
        print("  SKIP: needs both models")
        return
    meta, raw = yb.read_engine_file(ENGINE)
    meta["trt"] = "0.0.0"
    with tempfile.NamedTemporaryFile(suffix=".engine", delete=False) as f:
        head = json.dumps(meta).encode()
        f.write(struct.pack("<i", len(head)) + head + raw)
    said = []

    class Log:
        def error(self, m):
            said.append(m)

    det = yb.load_detector(f.name, fallback=PT, log=Log())
    os.unlink(f.name)
    check(det.backend.startswith("pytorch"), f"a mismatched engine falls back: {det.backend.split(' (')[0]}")
    check(bool(said) and "./rover engine" in said[0], f"and says how to fix it: {said[0][:90] if said else ''}...")


if __name__ == "__main__":
    test_decode()
    if len(sys.argv) > 1:
        test_parity(sys.argv[1])
    test_fallback()
    print("\nALL PASS" if not fails else f"\n{len(fails)} FAILED: {fails}")
    sys.exit(1 if fails else 0)
