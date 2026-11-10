#!/usr/bin/env python3
"""replay_target_lock.py — the tracker on REAL recorded drives (CLOSED_LOOP_PLAN.md stage 3).

    # in the rover container, the host TensorRT on the path (as ./rover detect):
    python3 replay_target_lock.py OUT_DIR [OUT_DIR ...]      (OUT_DIR from bag_frames.py)

The recordings have colour, TF and the fused pose but NO depth, so the truth
comes from the drive itself: a static object YOLO sees from many places has
one 3D point where all its sight lines meet (least-squares triangulation).

A. POSE PREDICTION on the real robot: that point, projected through each
   frame's recorded camera pose, against where YOLO saw the object (px). The
   error is reported for straight driving and for turns, and for camera-vs-pose
   time shifts of -200..+200 ms: the shift with the least error is the
   camera's time offset (LOCALIZATION_GAPS G8, never measured until now).
B. THE TRACKER on real images (JPEG, blur, exposure, ~4 frames a second --
   coarser than the live 15): locked at the object's first sighting -- once on
   the triangulated point, once 25 cm off it sideways -- and run with no depth
   (bearing only), at the stamp's pose as the live node does. Scored on every
   frame where YOLO also sees the object: tracking?  on the object (inside
   YOLO's box)?  pixel error to the truth's projection.

Writes OUT_DIR/stage3.json and OUT_DIR/strip_<object>.jpg (tracker box green,
YOLO's box blue, the truth's projection red) for checking by eye.
"""
import glob
import json
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "nodes"))
import target_lock as tl  # noqa: E402

STATIC = {"chair", "couch", "bed", "dining table", "tv", "laptop", "bottle", "cup", "potted plant",
          "refrigerator", "oven", "microwave", "sink", "toilet", "clock", "vase", "suitcase",
          "backpack", "handbag", "bowl", "keyboard", "bench", "book", "teddy bear", "toaster"}
CONF = 0.40
MIN_SIGHTINGS = 6       # DISTINCT views (see MIN_MOVE): a rover standing still repeats one
MIN_BASELINE_M = 0.3
RELIABLE_PX = 6.0        # a triangulation whose views agree this well is one thing (truth)
MIN_MOVE_M, MIN_TURN_RAD = 0.05, math.radians(3)   # a sighting counts after this much camera motion
TRACK_S = 60.0


def detections(out, frames, engine, fallback):
    path = os.path.join(out, "dets.json")
    if os.path.exists(path):
        return json.load(open(path))
    import yolo_backend as yb
    det = yb.load_detector(engine, fallback=fallback)
    print(f"  detecting with {det.backend}")
    res = []
    for i, f in enumerate(frames):
        img = cv2.imread(f)
        res.append([[l, round(c, 3), [round(v, 1) for v in b]] for l, c, b in det.detect(img)
                    if l in STATIC and c >= CONF])
        if i % 2000 == 0:
            print(f"    {i}/{len(frames)}")
    json.dump(res, open(path, "w"))
    return res


def rays_for(K, R, t, box):
    u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    return tl.ray(K, R, t, u, v)


def rough_point(K, R, t, box, H):
    """Floor contact of the box's bottom centre, lifted onto the centre ray: a first
    guess good enough to group sightings (objects ON the floor; others skipped)."""
    if box[3] > H - 3:
        return None
    o, d = tl.ray(K, R, t, (box[0] + box[2]) / 2, box[3])
    hit = tl.floor_hit(o, d)
    if hit is None:
        return None
    r = math.hypot(*(hit - o)[:2])
    if not 0.4 < r < 6.0:
        return None
    o, d = rays_for(K, R, t, box)
    s = r / max(1e-6, math.hypot(d[0], d[1]))
    return o + d * s


def triangulate(origins, dirs):
    A = np.zeros((3, 3)); b = np.zeros(3)
    for o, d in zip(origins, dirs):
        P = np.eye(3) - np.outer(d, d)
        A += P; b += P @ o
    return np.linalg.lstsq(A, b, rcond=None)[0]


def residuals(K, R, t, idx, boxes, p):
    out = []
    for i, box in zip(idx, boxes):
        u, v, z = tl.project(K, R[i], t[i], p)
        out.append(math.hypot(u - (box[0] + box[2]) / 2, v - (box[1] + box[3]) / 2) if u is not None else 1e9)
    return np.array(out)


def find_objects(K, R0, t0, ok0, dets, H):
    clusters = []        # label, points, (frame, box)
    for i, ds in enumerate(dets):
        if not ok0[i]:
            continue
        for label, conf, box in ds:
            p = rough_point(K, R0[i], t0[i], box, H)
            if p is None:
                continue
            for c in clusters:
                if c["label"] == label and np.linalg.norm(np.mean(c["pts"], 0) - p) < 0.6:
                    j = c["obs"][-1][0]           # only a NEW view counts (a still rover repeats one)
                    moved = np.linalg.norm(t0[i] - t0[j]) >= MIN_MOVE_M
                    turned = math.acos(np.clip((np.trace(R0[j].T @ R0[i]) - 1) / 2, -1, 1)) >= MIN_TURN_RAD
                    if moved or turned:
                        c["pts"].append(p); c["obs"].append((i, box))
                    break
            else:
                clusters.append({"label": label, "pts": [p], "obs": [(i, box)]})
    objects = []
    for c in clusters:
        if len(c["obs"]) < MIN_SIGHTINGS:
            continue
        idx = np.array([i for i, _ in c["obs"]]); boxes = [b for _, b in c["obs"]]
        cams = t0[idx]
        if np.ptp(cams[:, 0]) ** 2 + np.ptp(cams[:, 1]) ** 2 < MIN_BASELINE_M ** 2:
            continue           # all seen from one spot: no triangulation
        keep = np.ones(len(idx), bool)
        for _ in range(4):
            o_d = [rays_for(K, R0[i], t0[i], b) for i, b in zip(idx[keep], np.array(boxes)[keep])]
            p = triangulate([o for o, _ in o_d], [d for _, d in o_d])
            res = residuals(K, R0, t0, idx, boxes, p)
            new = res < max(15.0, 3 * np.median(res[keep]))
            if (new == keep).all():
                break
            keep = new
        if keep.sum() < MIN_SIGHTINGS:
            continue
        res = residuals(K, R0, t0, idx[keep], np.array(boxes)[keep], p)
        objects.append({"label": c["label"], "p": p, "idx": idx[keep], "boxes": np.array(boxes)[keep],
                        "residual_px": float(np.median(res)),
                        # views that agree = one physical thing; a set of identical chairs
                        # merges into one 'object' whose point is between them
                        "reliable": bool(np.median(res) <= RELIABLE_PX)})
    return objects


def time_offset(K, R, t, ok, shifts, objects):
    best = None
    table = []
    for d, s in enumerate(shifts):
        res = []
        for o in objects:
            sel = ok[o["idx"], d]
            if sel.sum() < MIN_SIGHTINGS // 2:
                continue
            i = o["idx"][sel]
            o_d = [rays_for(K, R[k, d], t[k, d], b) for k, b in zip(i, o["boxes"][sel])]
            p = triangulate([a for a, _ in o_d], [b for _, b in o_d])
            res.extend(residuals(K, R[:, d], t[:, d], i, o["boxes"][sel], p))
        if res:
            m = float(np.median(res))
            table.append((int(s), round(m, 2)))
            if best is None or m < best[1]:
                best = (int(s), m)
    return best, table


def run_tracker(K, R0, t0, ok0, stamps, frames, dets, o, offset_m, W, H):
    """Lock at the first sighting, run to TRACK_S, score against YOLO."""
    i0, box0 = int(o["idx"][0]), o["boxes"][0]
    p_truth = o["p"]
    guess = p_truth.copy()
    if offset_m:
        view = p_truth - t0[i0]
        side = np.cross(view, [0, 0, 1.0]); side /= np.linalg.norm(side)
        guess = p_truth + offset_m * side
    tgt = tl.Target(cv2.imread(frames[i0]), box0, guess, (R0[i0], t0[i0]), K, stamps[i0] / 1e9, o["label"])
    rows = []
    for i in range(i0 + 1, len(frames)):
        tsec = stamps[i] / 1e9
        if tsec - stamps[i0] / 1e9 > TRACK_S:
            break
        if not ok0[i]:
            continue
        st = tgt.update(cv2.imread(frames[i]), None, (R0[i], t0[i]), K, tsec)
        u, v, z = tl.project(K, R0[i], t0[i], p_truth)
        yolo = None
        if u is not None:
            for label, conf, b in dets[i]:
                if label == o["label"] and b[0] - 5 <= u <= b[2] + 5 and b[1] - 5 <= v <= b[3] + 5:
                    yolo = b; break
        row = {"i": i, "state": st["state"], "yolo": yolo, "truth": None if u is None else (u, v)}
        if st["state"] == "tracking":
            row["px"] = st["pixel"]
            row["err_px"] = math.hypot(st["pixel"][0] - u, st["pixel"][1] - v) if u is not None else None
            row["on_object"] = yolo is not None and yolo[0] <= st["pixel"][0] <= yolo[2] and yolo[1] <= st["pixel"][1] <= yolo[3]
        row["err3d_m"] = float(np.linalg.norm(np.array(st["object"]) - p_truth))
        rows.append(row)
    seen = [r for r in rows if r["yolo"] is not None]
    trk = [r for r in seen if r["state"] == "tracking"]
    wrong = [r for r in trk if not r["on_object"]]
    errs = [r["err_px"] for r in trk if r.get("err_px") is not None]
    states = {}
    for r in rows:
        states[r["state"]] = states.get(r["state"], 0) + 1
    return {"lock_frame": i0, "frames": len(rows), "yolo_frames": len(seen),
            "tracking_when_yolo_sees": round(len(trk) / max(1, len(seen)), 3),
            "off_object_when_tracking": round(len(wrong) / max(1, len(trk)), 3),
            "px_err_median": round(float(np.median(errs)), 1) if errs else None,
            "px_err_p90": round(float(np.percentile(errs, 90)), 1) if errs else None,
            "err3d_start_m": round(float(np.linalg.norm(guess - p_truth)), 3),
            "err3d_end_m": round(rows[-1]["err3d_m"], 3) if rows else None,
            "states": states}, rows


def strip(out, frames, o, rows, n=8):
    sel = [r for r in rows if r["yolo"] is not None][:: max(1, len(rows) // (n * 2))][:n]
    tiles = []
    for r in sel:
        im = cv2.imread(frames[r["i"]])
        if r["yolo"]:
            b = [int(v) for v in r["yolo"]]; cv2.rectangle(im, b[:2], b[2:], (255, 120, 0), 1)
        if r["truth"]:
            cv2.circle(im, tuple(int(v) for v in r["truth"]), 4, (0, 0, 255), -1)
        if r.get("px"):
            cv2.drawMarker(im, tuple(int(v) for v in r["px"]), (0, 255, 0), cv2.MARKER_CROSS, 14, 2)
        cv2.putText(im, f"{r['i']} {r['state']}", (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        tiles.append(im)
    if tiles:
        while len(tiles) % 4:
            tiles.append(np.zeros_like(tiles[0]))
        grid = np.vstack([np.hstack(tiles[k:k + 4]) for k in range(0, len(tiles), 4)])
        name = f"strip_{o['label'].replace(' ', '_')}_{o['idx'][0]}.jpg"
        cv2.imwrite(os.path.join(out, name), grid)
        return name


def main():
    engine = next(iter(sorted(glob.glob("/models/yolov8n_640x384_trt*.engine"))), "")
    for out in sys.argv[1:]:
        meta = np.load(os.path.join(out, "meta.npz"))
        info = json.load(open(os.path.join(out, "info.json")))
        W, H = info["width"], info["height"]
        frames = sorted(glob.glob(os.path.join(out, "frames", "*.jpg")))
        K, shifts = meta["K"], meta["shifts_ms"]
        z = list(shifts).index(0)
        R, t, ok, stamps = meta["R"], meta["t"], meta["ok"], meta["stamp_ns"]
        print(f"\n=== {info['bag']}: {len(frames)} frames")
        dets = detections(out, frames, engine, "/models/yolov8n.pt")
        objects = find_objects(K, R[:, z], t[:, z], ok[:, z], dets, H)
        print(f"  A. {len(objects)} static objects triangulated (>= {MIN_SIGHTINGS} sightings, >= {MIN_BASELINE_M} m apart)")
        print("     per object (median px between the views): " +
              ", ".join(f"{o['label']}@{o['idx'][0]} {o['residual_px']:.1f}{'' if o['reliable'] else ' (merged: not truth)'}" for o in objects))
        objects_all = objects
        objects = [o for o in objects if o["reliable"]]
        print(f"     {len(objects)} reliable as truth (<= {RELIABLE_PX:.0f} px); the rest are look-alikes merged or moved things")
        if not objects:
            continue
        best, table = time_offset(K, R, t, ok, shifts, objects)
        print(f"     median reprojection error by camera time shift (ms, px): {table}")
        print(f"     least error at {best[0]:+d} ms ({best[1]:.1f} px); at 0 ms {dict(table)[0]:.1f} px")
        # error at the stamp, turning vs straight (yaw rate from the base pose)
        base = meta["base"]
        yaw_rate = np.zeros(len(frames))
        dtt = np.diff(stamps) / 1e9
        dyaw = np.diff(np.unwrap(np.nan_to_num(base[:, 2])))
        yaw_rate[1:] = np.abs(dyaw / np.maximum(dtt, 1e-3))
        speed = np.zeros(len(frames))
        speed[1:] = np.hypot(np.diff(np.nan_to_num(base[:, 0])), np.diff(np.nan_to_num(base[:, 1]))) / np.maximum(dtt, 1e-3)
        groups = {"still": [], "driving": [], "turning": []}
        for o in objects:
            res = residuals(K, R[:, z], t[:, z], o["idx"], o["boxes"], o["p"])
            for i, r in zip(o["idx"], res):
                g = "turning" if yaw_rate[i] > 0.3 else "driving" if speed[i] > 0.05 else "still"
                groups[g].append(r)
        med = lambda v: f"{np.median(v):.1f} px (p90 {np.percentile(v, 90):.1f}, n={len(v)})" if v else "-"  # noqa: E731
        print(f"     at 0 ms: still {med(groups['still'])}; driving {med(groups['driving'])}; turning > 0.3 rad/s {med(groups['turning'])}")
        straight, turn = groups["still"] + groups["driving"], groups["turning"]
        report = {"bag": info["bag"], "objects": [], "time_offset_ms": best[0], "offset_table": table,
                  "px_straight": float(np.median(straight)), "px_turning": float(np.median(turn)) if turn else None}
        print("  B. tracker, locked at each object's first sighting, no depth (bearing only):")
        print(f"     {'object':22s} {'seen':>5s} {'track%':>7s} {'off-obj%':>8s} {'px med':>6s} {'p90':>5s}   "
              f"{'25 cm off: track%':>17s} {'3D start->end':>14s}")
        for o in objects:
            r0, rows0 = run_tracker(K, R[:, z], t[:, z], ok[:, z], stamps, frames, dets, o, 0.0, W, H)
            r1, _ = run_tracker(K, R[:, z], t[:, z], ok[:, z], stamps, frames, dets, o, 0.25, W, H)
            if r0["yolo_frames"] < 5:
                continue
            name = f"{o['label']}@{o['idx'][0]}"
            print(f"     {name:22s} {r0['yolo_frames']:5d} {r0['tracking_when_yolo_sees'] * 100:6.0f}% "
                  f"{r0['off_object_when_tracking'] * 100:7.0f}% {r0['px_err_median'] or 0:6.1f} {r0['px_err_p90'] or 0:5.1f}   "
                  f"{r1['tracking_when_yolo_sees'] * 100:16.0f}% {r1['err3d_start_m']:6.2f}->{r1['err3d_end_m']:.2f} m")
            report["objects"].append({"name": name, "point": o["p"].round(3).tolist(), "sightings": int(len(o["idx"])),
                                      "locked_on_truth": r0, "locked_25cm_off": r1,
                                      "strip": strip(out, frames, o, rows0)})
        # B2. Look-alike sets (identical chairs merged above): no truth, but a lock that
        # switches chairs shows as a sudden jump of its point (dining chairs ~0.45 m apart)
        merged = [o for o in objects_all if not o["reliable"]]
        if merged:
            print("  B2. look-alike sets (no truth): does the lock jump between them? "
                  "(a jump = the point moves > 0.3 m between two tracked frames)")
            for o in merged:
                i0, box0 = int(o["idx"][0]), o["boxes"][0]
                o_, d_ = rays_for(K, R[i0, z], t[i0, z], box0)
                guess = rough_point(K, R[i0, z], t[i0, z], box0, H)
                if guess is None:
                    continue
                tgt = tl.Target(cv2.imread(frames[i0]), box0, guess, (R[i0, z], t[i0, z]), K, stamps[i0] / 1e9, o["label"])
                last, jumps, tracked, n = None, 0, 0, 0
                for i in range(i0 + 1, len(frames)):
                    if stamps[i] - stamps[i0] > TRACK_S * 1e9:
                        break
                    if not ok[i, z]:
                        continue
                    st = tgt.update(cv2.imread(frames[i]), None, (R[i, z], t[i, z]), K, stamps[i] / 1e9)
                    n += 1
                    if st["state"] == "tracking":
                        tracked += 1
                        p_ = np.array(st["object"])
                        if last is not None and np.linalg.norm(p_ - last) > 0.3:
                            jumps += 1
                        last = p_
                print(f"     {o['label']}@{i0}: tracked {tracked}/{n} frames, jumps {jumps}")
                report.setdefault("lookalike_sets", []).append({"name": f"{o['label']}@{i0}", "tracked": tracked,
                                                               "frames": n, "jumps": jumps})
        json.dump(report, open(os.path.join(out, "stage3.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
