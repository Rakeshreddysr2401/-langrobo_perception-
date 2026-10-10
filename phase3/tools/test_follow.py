#!/usr/bin/env python3
"""test_follow.py — offline checks of nodes/follow.py, closed loop.

    python3 phase3/tools/test_follow.py      (no robot, no ROS graph)

A unicycle rover at 20 Hz and a scripted person published at 5 Hz the way
/vision/people does (id, odom x/y, velocity, dist, bearing). walk: a person
walking away is followed at ~GAP. stop: when they stop it stops at ~GAP.
side: a person 70 deg off is faced before any driving. close: a person
walking up to the rover is never reversed from. box: a box between them
stops it, and it gives up after BLOCKED_S. gone: vanishing for LOST_S ends
it. new_id: the tracker's new id after a short gap is re-acquired. glitch:
one wild sample is ignored. pick: the start picks the one nearest the middle.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "nodes"))
import follow as fl  # noqa: E402

DT = 0.05


def person_msg(pid, px, py, vx, vy, pose, lost_s=0.0):
    dx, dy = px - pose[0], py - pose[1]
    return {"id": pid, "x": px, "y": py, "vx": vx, "vy": vy, "conf": 0.9,
            "lost_s": lost_s, "dist": math.hypot(dx, dy),
            "bearing": fl.wrap(math.atan2(dy, dx) - pose[2])}


def run(path, T, pose=(0.0, 0.0, 0.0), obstacles=None, visible=lambda t: True,
        pid=lambda t: 1, glitch_at=None, f=None):
    """path(t) -> (x, y) of the person. Returns (follower, log of t, pose, vx, wz, dist)."""
    f = f or fl.Follower()
    pose = list(pose)
    x0, y0 = path(0.0)
    assert f.start(0.0, [person_msg(1, x0, y0, 0, 0, pose)])
    log = []
    t = 0.0
    while t < T and f.state == "following":
        t += DT
        if round(t / DT) % 4 == 0 and visible(t):          # 5 Hz people
            x, y = path(t)
            xb, yb = path(t - 0.2)
            if glitch_at is not None and abs(t - glitch_at) < DT / 2:
                x, y = x - 1.0 * math.sin(pose[2]), y + 1.0 * math.cos(pose[2])   # 1 m to the side
            f.update_people(t, [person_msg(pid(t), x, y, (x - xb) / 0.2, (y - yb) / 0.2, pose)])
        pts = None
        if obstacles is not None:
            c, s = math.cos(pose[2]), math.sin(pose[2])
            d = obstacles - np.array(pose[:2])
            pts = np.stack([d[:, 0] * c + d[:, 1] * s, -d[:, 0] * s + d[:, 1] * c], 1)
        vx, wz = f.step(t, tuple(pose), pts)
        assert 0.0 <= vx <= fl.V_MAX + 1e-9, vx
        assert abs(wz) <= fl.W_MAX + 1e-9, wz
        pose[0] += vx * math.cos(pose[2]) * DT
        pose[1] += vx * math.sin(pose[2]) * DT
        pose[2] = fl.wrap(pose[2] + wz * DT)
        x, y = path(t)
        log.append((t, tuple(pose), vx, wz, math.hypot(x - pose[0], y - pose[1])))
    return f, log


def test_walk():
    f, log = run(lambda t: (1.5 + 0.25 * t, 0.0), 20.0)       # 0.25 m/s away
    tail = [d for t, _, _, _, d in log if t > 12]
    assert f.state == "following", f.status()
    assert abs(np.mean(tail) - fl.GAP) < 0.5, np.mean(tail)
    assert min(d for *_, d in log) > 0.6


def test_stop():
    f, log = run(lambda t: (1.5 + 0.25 * min(t, 6.0), 0.0), 20.0)
    *_, vx, wz, d = log[-1]
    assert vx < 0.01 and abs(d - fl.GAP) < fl.DIST_DEADBAND + 0.1, (vx, d)


def test_side():
    a = math.radians(70)
    f, log = run(lambda t: (2.0 * math.cos(a), 2.0 * math.sin(a)), 6.0)
    first_drive = next(t for t, _, vx, _, _ in log if vx > 0.01)
    yaw_then = next(p[2] for t, p, *_ in log if t >= first_drive)
    assert abs(fl.wrap(a - yaw_then)) < fl.DRIVE_MAX_ERR + 0.05, math.degrees(yaw_then)
    assert abs(log[-1][4] - fl.GAP) < 0.3, log[-1][4]


def test_close():
    f, log = run(lambda t: (2.0 - 0.3 * min(t, 4.0), 0.0), 8.0)   # walks up to 0.8 m
    assert all(vx < 0.01 for t, _, vx, _, _ in log if t > 3.0)
    # never forward while they are inside the gap (the robot was moving: 2 m start)
    f, log = run(lambda t: (3.0 - 0.8 * min(t, 2.6), 0.0), 6.0)   # walks up fast
    bad = [(round(t, 2), round(vx, 2), round(d, 2)) for t, _, vx, _, d in log if d < fl.GAP and vx > 0]
    assert not bad, f"drove forward inside the gap: {bad[:3]}"


def test_box():
    box = np.array([[1.0 + i * 0.02, j * 0.02] for i in range(5) for j in range(-5, 6)])
    f, log = run(lambda t: (3.0, 0.0), 20.0, obstacles=box)
    assert f.state == "done" and f.result == "blocked", f.status()
    front = max(p[0] for _, p, *_ in log) + fl.FRONT
    assert front < 1.0, f"drove into the box: front edge at {front:.2f}"


def test_gone():
    f, log = run(lambda t: (2.0, 0.0), 10.0, visible=lambda t: t < 2.0)
    assert f.state == "done" and f.result == "lost", f.status()
    assert log[-1][0] < 2.0 + fl.LOST_S + 0.3


def test_new_id():
    f, log = run(lambda t: (2.0 + 0.2 * t, 0.0), 12.0,
                 visible=lambda t: not (3.0 < t < 5.0), pid=lambda t: 1 if t < 4 else 2)
    assert f.state == "following" and f.target_id == 2, f.status()


def test_glitch():
    f = fl.Follower()
    f, log = run(lambda t: (2.5, 0.0), 4.0, glitch_at=2.0, f=f)
    assert f.state == "following", f.status()
    wz = max(abs(w) for t, _, _, w, _ in log if 2.0 < t < 3.0)
    assert wz < 0.05, f"turned at {wz:.2f} rad/s toward a one-frame glitch"


def test_pick():
    f = fl.Follower()
    pose = (0.0, 0.0, 0.0)
    ok = f.start(0.0, [person_msg(1, 1.0, 1.0, 0, 0, pose),     # 45 deg
                       person_msg(2, 2.0, 0.2, 0, 0, pose),     # ~6 deg  <- this one
                       person_msg(3, 1.5, -1.2, 0, 0, pose)])   # -39 deg
    assert ok and f.target_id == 2
    f2 = fl.Follower()
    assert not f2.start(0.0, [person_msg(1, 6.0, 0.0, 0, 0, pose)])
    assert f2.result == "nobody"


if __name__ == "__main__":
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as e:
                fails += 1
                print(f"FAIL {name}: {e}")
    sys.exit(1 if fails else 0)
