#!/usr/bin/env python3
"""test_target_lock.py — does the lock keep the right thing, against exact truth?

    python3 phase4/tools/test_target_lock.py          (no ROS, no GPU; ~20 s)

A tiny ray-cast world (textured rectangles: wall, floor, the object, a
look-alike, an occluder) is rendered as the D555 colour camera at 448x252 with
an aligned depth image, from a rover pose that drives and turns. Each case
locks the object the way the robot will (one frame, a box, a first 3D guess)
and checks what target_lock.Target reports, frame by frame, against the truth.

Cases (each is a claim in target_lock.py / CLOSED_LOOP_PLAN.md):
  still          locked and still: the point stays within 1 cm
  approach       driving 2 m toward it while turning: within 3 cm, tracking throughout
  wrong_first    the first 3D guess 25 cm off (VLM y error, background depth): corrected
  moved          the object slides 30 cm mid-drive: the lock follows it
  occluded       a box passes in front for ~1 s: "occluded", then tracking again, no jump
  look_alike     an identical object 45 cm beside it: never jumps to it
  out_of_view    the rover turns away and back: "out_of_view", then reacquired
  floor_dot      a flat dot on the floor, no depth on it at all: floor plane, within 3 cm
  gone           the object is removed: "lost" after LOST_S, point not dragged anywhere
  too_close      at 30 cm: "too_close", point held
  pose_noise     1 cm / 0.5 deg pose noise each frame: still within 3 cm
  camera_real    auto-exposure swings (x0.6-1.4) and motion blur in turns: still tracking
  goal_policy    goal_update_due / standoff_goal rules
"""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "nodes"))
import target_lock as tl  # noqa: E402

W, H = 448, 252
FX = FY = 232.0                      # ~88 deg horizontal, the D555 colour stream
K = np.array([[FX, 0, W / 2], [0, FY, H / 2], [0, 0, 1]])
CAM_FWD, CAM_UP = 0.17, 0.30         # camera on base_link: 17 cm forward (static TF), ~30 cm up
DT = 1 / 15                          # colour at 15 Hz
rng_tex = np.random.default_rng(7)


def texture(n, block, seed):
    r = np.random.default_rng(seed)
    t = r.integers(30, 225, (n // block + 1, n // block + 1)).astype(np.uint8)
    return np.kron(t, np.ones((block, block), np.uint8))[:n, :n]


TEX = {"wall": texture(512, 32, 1), "floor": texture(512, 24, 2), "obj": texture(64, 6, 3),
       "occ": texture(64, 16, 4), "dot": np.full((64, 64), 20, np.uint8)}


class Rect:
    """A textured rectangle: corner c, edge vectors a, b (metres); texture scale in m."""

    def __init__(self, c, a, b, tex, tex_m, depth_ok=True):
        self.c, self.a, self.b = (np.asarray(v, float) for v in (c, a, b))
        self.n = np.cross(self.a, self.b)
        self.tex, self.tex_m, self.depth_ok = TEX[tex], tex_m, depth_ok


def obj_face(x, y, w=0.12, h=0.25, z=0.0, tex="obj"):
    """A box's face toward -x (toward the rover start): centred at y, width w, height h."""
    return Rect((x, y - w / 2, z), (0, w, 0), (0, 0, h), tex, max(w, h))


def scene(obj=(2.5, 0.2), extra=()):
    return [Rect((4.0, -4, 0), (0, 8, 0), (0, 0, 2.5), "wall", 4.0),
            Rect((-2, -4, 0), (8, 0, 0), (0, 8, 0), "floor", 3.0)] + \
           ([obj_face(*obj)] if obj else []) + list(extra)


def cam_pose(x, y, yaw):
    """Colour optical frame in odom: z forward, x right, y down."""
    M = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return Rz @ M, np.array([x + CAM_FWD * c, y + CAM_FWD * s, CAM_UP])


def render(rects, cam):
    R, t = cam
    us, vs = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    d = np.stack([(us - K[0, 2]) / FX, (vs - K[1, 2]) / FY, np.ones_like(us)], -1) @ R.T  # odom dirs
    best = np.full((H, W), np.inf)
    img = np.full((H, W), 128, np.uint8)
    depth_ok = np.zeros((H, W), bool)
    for r in rects:
        den = d @ r.n
        with np.errstate(divide="ignore", invalid="ignore"):
            s = ((r.c - t) @ r.n) / den
        P = t + d * s[..., None]
        q = P - r.c
        la, lb = (q @ r.a) / (r.a @ r.a), (q @ r.b) / (r.b @ r.b)
        hit = (s > 0.01) & (s < best) & (la >= 0) & (la <= 1) & (lb >= 0) & (lb <= 1)
        n = r.tex.shape[0]
        ia = np.clip((la * np.linalg.norm(r.a) / r.tex_m * n).astype(int), 0, n - 1)
        ib = np.clip((lb * np.linalg.norm(r.b) / r.tex_m * n).astype(int), 0, n - 1)
        img[hit] = r.tex[ib[hit], ia[hit]]
        depth_ok[hit] = r.depth_ok
        best[hit] = s[hit]
    zcam = best * (d @ R[:, 2])            # optical-axis depth, as a RealSense pixel stores it
    depth = np.where(np.isfinite(best) & depth_ok, zcam, 0.0).astype(np.float32)
    return np.stack([img] * 3, -1), depth


def noisy_depth(depth, rng, holes=0.3, sigma_rel=0.01):
    d = depth * (1 + sigma_rel * rng.standard_normal(depth.shape))
    d[rng.random(depth.shape) < holes] = 0.0
    return d.astype(np.float32)


def box_of(rect, cam):
    """The object's screen box (what a detector / the VLM would give)."""
    R, t = cam
    pts = [rect.c, rect.c + rect.a, rect.c + rect.b, rect.c + rect.a + rect.b]
    uv = [tl.project(K, R, t, p)[:2] for p in pts]
    us, vs = [p[0] for p in uv], [p[1] for p in uv]
    return min(us), min(vs), max(us), max(vs)


def centre(rect):
    return rect.c + rect.a / 2 + rect.b / 2


def lock(rects, obj_rect, pose, guess_offset=(0, 0, 0), rng=None):
    cam = cam_pose(*pose)
    img, depth = render(rects, cam)
    truth = centre(obj_rect)
    tgt = tl.Target(img, box_of(obj_rect, cam), truth + np.array(guess_offset), cam, K, 0.0, "obj")
    return tgt, truth


fails = []


def check(ok, what):
    print(("    PASS " if ok else "    FAIL ") + what)
    if not ok:
        fails.append(what)


def camera_effects(img, k, yaw_rate, rng):
    """What the D555's colour stream does that a renderer does not: exposure, blur."""
    import cv2
    out = np.clip(img.astype(np.float32) * rng.uniform(0.6, 1.4) + rng.uniform(-20, 20), 0, 255).astype(np.uint8)
    n = int(min(9, abs(yaw_rate) * 40))          # px of horizontal smear while turning
    if n >= 2:
        out = cv2.blur(out, (n, 1))
    return out


def run(rects_at, tgt, poses, rng, t0=0.0, noise_pose=0.0, holes=0.3, log_states=False, effects=False):
    """rects_at(k) -> the world at frame k. Returns statuses."""
    out = []
    for k, (x, y, yaw) in enumerate(poses):
        cam_true = cam_pose(x, y, yaw)
        img, depth = render(rects_at(k), cam_true)
        if effects:
            prev = poses[k - 1][2] if k else yaw
            img = camera_effects(img, k, (yaw - prev) / DT, rng)
        if noise_pose:
            cam_used = cam_pose(x + noise_pose * rng.standard_normal(), y + noise_pose * rng.standard_normal(),
                                yaw + math.radians(0.5) * rng.standard_normal())
        else:
            cam_used = cam_true
        out.append(tgt.update(img, noisy_depth(depth, rng, holes), cam_used, K, t0 + (k + 1) * DT))
    return out


def err(s, truth):
    return float(np.linalg.norm(np.array(s["object"]) - truth))


def case(name):
    print(f"  {name}")


def main():
    rng = np.random.default_rng(0)
    t_start = time.time()

    case("still")
    o = obj_face(2.5, 0.2)
    tgt, truth = lock(scene(extra=()), o, (0, 0, 0))
    W_ = scene()
    ss = run(lambda k: W_, tgt, [(0, 0, 0)] * 30, rng)
    check(all(s["state"] == "tracking" for s in ss), f"tracking every frame ({sum(s['state'] == 'tracking' for s in ss)}/30)")
    check(err(ss[-1], truth) < 0.01, f"point within 1 cm ({err(ss[-1], truth) * 100:.1f} cm)")

    case("approach")
    tgt, truth = lock(W_, o, (0, 0, 0))
    poses = [(0.07 * k, 0.0, math.radians(8) * math.sin(k / 6)) for k in range(29)]   # 2 m, weaving +-8 deg
    ss = run(lambda k: W_, tgt, poses, rng)
    inview = [s for s in ss if s["state"] not in ("out_of_view", "too_close")]
    tr = sum(s["state"] == "tracking" for s in inview)
    check(tr == len(inview), f"tracking {tr}/{len(inview)} frames while it is fully in view, driving and turning")
    check(ss[-1]["state"] in ("out_of_view", "too_close"),
          f"at the end (0.4 m, cut off by the frame) an honest '{ss[-1]['state']}', never 'lost'")
    worst = max(err(s, truth) for s in ss[5:])
    check(worst < 0.03, f"point within 3 cm all the way ({worst * 100:.1f} cm worst)")

    case("wrong_first")
    tgt, truth = lock(W_, o, (0, 0, 0), guess_offset=(0.20, -0.15, 0.0))   # 25 cm off: depth behind + y error
    ss = run(lambda k: W_, tgt, [(0.03 * k, 0, 0) for k in range(30)], rng)
    first = next((k for k, s in enumerate(ss) if err(s, truth) < 0.05), None)
    check(first is not None, f"25 cm first guess corrected to < 5 cm by frame {first}")
    check(err(ss[-1], truth) < 0.03, f"and stays there ({err(ss[-1], truth) * 100:.1f} cm)")

    case("moved")
    tgt, truth = lock(W_, o, (0, 0, 0))
    o2 = obj_face(2.5, -0.1)                                   # slides 30 cm to the right
    ss = run(lambda k: scene() if k < 10 else scene(obj=(2.5, -0.1)), tgt,
             [(0.04 * k, 0, 0) for k in range(40)], rng)
    back = next((k for k, s in enumerate(ss) if k >= 10 and err(s, centre(o2)) < 0.05), None)
    check(err(ss[-1], centre(o2)) < 0.04, f"followed the 30 cm move ({err(ss[-1], centre(o2)) * 100:.1f} cm from the new spot, "
                                         f"from_lock_m {ss[-1]['from_lock_m']})")
    check(back is not None and (back - 10) * DT < 2.5, f"found again {((back or 99) - 10) * DT:.1f} s after the move")

    case("occluded")
    tgt, truth = lock(W_, o, (0, 0, 0))
    occ = Rect((1.6, -0.1, 0), (0, 0.6, 0), (0, 0, 0.6), "occ", 0.6)
    ss = run(lambda k: scene(extra=[occ]) if 10 <= k < 25 else W_, tgt, [(0, 0, 0)] * 40, rng)
    mid = [s["state"] for s in ss[11:24]]
    check(mid.count("occluded") >= 10, f"'occluded' while hidden ({mid.count('occluded')}/13)")
    check(all(err(s, truth) < 0.02 for s in ss), "the point never jumps (to the occluder)")
    check(ss[-1]["state"] == "tracking", "tracking again after")

    case("look_alike")
    tgt, truth = lock(W_, o, (0, 0, 0))
    twin = obj_face(2.5, 0.65)                                 # the same look, 45 cm to the left
    poses = [(0.05 * k, 0.0, math.radians(10) * math.sin(k / 5)) for k in range(30)]
    ss = run(lambda k: scene(extra=[twin]), tgt, poses, rng)
    worst = max(err(s, truth) for s in ss)
    check(worst < 0.05, f"never jumps to the look-alike ({worst * 100:.1f} cm worst)")

    case("out_of_view")
    tgt, truth = lock(W_, o, (0, 0, 0))
    yaws = [math.radians(a) for a in list(range(0, 91, 10)) + list(range(90, -1, -10))]
    ss = run(lambda k: W_, tgt, [(0, 0, y) for y in yaws], rng)
    states = [s["state"] for s in ss]
    check("out_of_view" in states, "out_of_view while turned away")
    check(states[-1] == "tracking" and err(ss[-1], truth) < 0.02, f"reacquired after ({states[-1]}, {err(ss[-1], truth) * 100:.1f} cm)")

    case("floor_dot")
    dot = Rect((1.5, 0.25, 0.001), (0, -0.1, 0), (0.1, 0, 0), "dot", 0.1, depth_ok=False)
    # a dark dot with a lighter ring around it: something to match
    ring = Rect((1.45, 0.3, 0.0005), (0, -0.2, 0), (0.2, 0, 0), "obj", 0.2, depth_ok=False)
    Wd = scene(obj=None, extra=[dot, ring])
    cam = cam_pose(0, 0, 0)
    img, depth = render(Wd, cam)
    truth = centre(dot)
    tgt = tl.Target(img, box_of(ring, cam), truth + np.array([0.12, -0.05, 0.0]), cam, K, 0.0, "dot")
    ss = run(lambda k: Wd, tgt, [(0.025 * k, 0, 0) for k in range(24)], rng)
    srcs = {s["depth"] for s in ss if s["depth"]}
    check(tgt.floor and "floor" in srcs, f"uses the floor plane (sources {srcs})")
    e = err(ss[-1], truth)
    check(e < 0.03, f"dot within 3 cm with no depth on it ({e * 100:.1f} cm)")

    case("gone")
    tgt, truth = lock(W_, o, (0, 0, 0))
    ss = run(lambda k: W_ if k < 5 else scene(obj=None), tgt, [(0, 0, 0)] * 60, rng)
    lost_at = next((k for k, s in enumerate(ss) if s["state"] == "lost"), None)
    check(lost_at is not None and (lost_at - 5) * DT >= tl.LOST_S - 0.2,
          f"'lost' after ~{tl.LOST_S} s unseen (frame {lost_at})")
    check(err(ss[-1], truth) < 0.02, "the point stays where it was (not dragged to the wall)")

    case("too_close")
    tgt, truth = lock(W_, o, (1.6, 0.2, 0))
    ss = run(lambda k: W_, tgt, [(1.6 + 0.05 * k, 0.2, 0) for k in range(12)], rng)
    z_end = 2.5 - (1.6 + 0.05 * 11 + CAM_FWD)
    check(ss[-1]["state"] == "too_close", f"'too_close' with the camera {z_end * 100:.0f} cm from it ({ss[-1]['state']})")
    check(err(ss[-1], truth) < 0.03, "point held for the final approach")

    case("pose_noise")
    tgt, truth = lock(W_, o, (0, 0, 0))
    ss = run(lambda k: W_, tgt, [(0.05 * k, 0, 0) for k in range(30)], rng, noise_pose=0.01)
    worst = max(err(s, truth) for s in ss[5:])
    check(worst < 0.03, f"within 3 cm under 1 cm / 0.5 deg pose noise ({worst * 100:.1f} cm worst)")

    case("camera_real")
    tgt, truth = lock(W_, o, (0, 0, 0))
    poses = [(0.05 * k, 0.0, math.radians(12) * math.sin(k / 4)) for k in range(30)]
    ss = run(lambda k: W_, tgt, poses, rng, effects=True)
    tr = sum(s["state"] == "tracking" for s in ss)
    worst = max(err(s, truth) for s in ss)
    check(tr >= 0.8 * len(ss) and worst < 0.04,
          f"tracking {tr}/{len(ss)} with exposure swings + turn blur, {worst * 100:.1f} cm worst")

    case("goal_policy")
    check(not tl.goal_update_due((2, 0), (2.05, 0), (0, 0), 5.0), "5 cm shift: no re-send")
    check(tl.goal_update_due((2, 0), (2.2, 0), (0, 0), 5.0), "20 cm shift: re-send")
    check(not tl.goal_update_due((2, 0), (2.2, 0), (0, 0), 1.0), "not within 2 s of the last")
    check(not tl.goal_update_due((2, 0), (2.2, 0), (1.6, 0), 5.0), "not in the last metre")
    gx, gy, gyaw = tl.standoff_goal((2.0, 0.0), (0.0, 0.0), 0.45)
    check(abs(gx - 1.55) < 1e-9 and abs(gy) < 1e-9 and abs(gyaw) < 1e-9, "standoff 0.45 m short, facing it")

    # cost per frame (the update itself, rendering excluded)
    tgt, truth = lock(W_, o, (0, 0, 0))
    img, depth = render(W_, cam_pose(0.5, 0, 0))
    t0 = time.perf_counter()
    for k in range(100):
        tgt.update(img, depth, cam_pose(0.5, 0, 0), K, 1 + k * DT)
    ms = (time.perf_counter() - t0) * 10
    print(f"\n  update(): {ms:.2f} ms per frame on this CPU   (whole run {time.time() - t_start:.0f} s)")
    print("\nALL PASS" if not fails else f"\n{len(fails)} FAILED:\n  " + "\n  ".join(fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
