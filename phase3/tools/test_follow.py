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
near_hold: something between them, the person just past the gap: it holds and
keeps following, no 'blocked'. search: a person who walks out to the left is
turned after, and re-acquired (new id) farther than REACQ_M from the coast.
find / find_nobody / find_no_room: "follow me" with nobody in view turns to
find someone (2026-10-13), gives up after a full circle, or at once with no
room to turn. gone: lost straight ahead, it drives to their last spot and
looks all around there before 'lost'. look_*: someone who looks like the
target is re-acquired anywhere in view while searching; someone who looks
clearly different is not, even standing where the target should be.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "nodes"))
import follow as fl  # noqa: E402

DT = 0.05


RED = [0.9] + [0.0] * 7 + [0.1, 0.0, 0.0] + [0.0] * 8 + [0.8, 0.2, 0.0]    # red top, dark trousers
BLUE = [0.0] * 5 + [0.9, 0.0, 0.0] + [0.0, 0.1, 0.0] + [0.0] * 8 + [0.1, 0.1, 0.8]  # blue top, white trousers
FOV = math.radians(45)     # half the camera's view


def person_msg(pid, px, py, vx, vy, pose, lost_s=0.0, look=None):
    dx, dy = px - pose[0], py - pose[1]
    d = {"id": pid, "x": px, "y": py, "vx": vx, "vy": vy, "conf": 0.9,
         "lost_s": lost_s, "dist": math.hypot(dx, dy),
         "bearing": fl.wrap(math.atan2(dy, dx) - pose[2])}
    if look is not None:
        d["look"] = look
    return d


def in_view(pose, x, y):
    return abs(fl.wrap(math.atan2(y - pose[1], x - pose[0]) - pose[2])) < FOV


def drive(f, pose, vx, wz):
    pose[0] += vx * math.cos(pose[2]) * DT
    pose[1] += vx * math.sin(pose[2]) * DT
    pose[2] = fl.wrap(pose[2] + wz * DT)


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


def test_wall_slows_and_holds_off():
    # a wall across the way at x = 1.5, the person behind it at 3.0
    wall = np.array([[1.5, j * 0.02] for j in range(-30, 31)])
    f, log = run(lambda t: (3.0, 0.0), 20.0, obstacles=wall)
    front = max(p[0] for _, p, *_ in log) + fl.FRONT
    gap = 1.5 - front
    assert fl.FRONT_MARGIN - 0.05 <= gap <= fl.FRONT_MARGIN + 0.1, f"stopped {gap:.2f} m from the wall"
    # no stop-go creeping: once it has slowed below 0.05 m/s it never speeds up again
    slow = next(i for i, (t, _, vx, _, _) in enumerate(log) if t > 1 and vx < 0.05)
    vs = [vx for _, _, vx, _, _ in log[slow:]]
    assert all(b <= a + 1e-9 for a, b in zip(vs, vs[1:])), "stop-go near the wall"
    assert f.result == "blocked", f.status()


def test_sudden_obstacle_caps_at_once():
    f = fl.Follower()
    f.start(0.0, [person_msg(1, 3.0, 0.0, 0, 0, (0, 0, 0))])
    f.vx = 0.3                                   # cruising
    box = np.array([[0.55, 0.0]])                # 0.37 m past the front edge
    vx, _ = f.step(0.05, (0.0, 0.0, 0.0), box)
    # smoothing alone would allow 0.3 - A_MAX * dt = 0.28 here
    assert vx <= (0.55 - fl.FRONT - fl.FRONT_MARGIN) / fl.STOP_T + 1e-9, vx


def test_near_hold():
    # 2026-10-12: blocked with them 1.15 m away ended the follow; it must hold
    # the box 0.25 m past the front edge: blocked from the start, 0.85 m short of them
    box = np.array([[0.45 + i * 0.02, j * 0.02] for i in range(3) for j in range(-5, 6)])
    f, log = run(lambda t: (1.3, 0.0), 20.0, obstacles=box)
    assert f.state == "following", f.status()
    assert f.holding and "blocked" not in f.status(), f.status()
    front = max(p[0] for _, p, *_ in log) + fl.FRONT
    assert front < 0.45, f"drove into the box: front edge at {front:.2f}"


def test_search():
    # walks off to the left at 0.6 m/s, unseen from 2 s, stands at y=3.0; back in view at 6 s as id 2
    path = lambda t: (2.0, 0.6 * min(t, 5.0))
    f, log = run(path, 12.0, visible=lambda t: not (2.0 < t < 6.0), pid=lambda t: 1 if t < 2 else 2)
    assert f.state == "following" and f.target_id == 2, f.status()
    turning = [wz for t, _, vx, wz, _ in log if 3.2 < t < 6.0]
    assert turning and min(turning) > 0.2, f"search did not turn left: {turning[:5]}"
    # it slows at A_MAX like any stop that is not for safety, then turns in place
    assert all(vx == 0.0 for t, _, vx, _, _ in log if 4.0 < t < 6.0), "drove while searching"


def test_gone():
    # lost straight ahead (a doorway): no turn first -- it goes to their last
    # spot, looks all around there, and only then is it 'lost'
    f, log = run(lambda t: (2.5, 0.0), 45.0, visible=lambda t: t < 2.0)
    assert f.state == "done" and f.result == "lost", f.status()
    assert "all around" in f.why, f.why
    assert log[-1][0] < 2.0 + fl.LOST_S + 0.3
    closest = min(math.hypot(2.5 - p[0], p[1]) for _, p, *_ in log)
    assert closest <= fl.GO_DONE_M + 0.1, f"did not go to their spot: {closest:.2f} m"
    yaws = [p[2] for t, p, *_ in log if t > 5.0]
    turned = sum(abs(fl.wrap(b - a)) for a, b in zip(yaws, yaws[1:]))
    assert turned > math.radians(330), f"looked around only {math.degrees(turned):.0f} deg"


def _find(person=None, T=20.0, obstacles=None):
    """Start with nobody in view; person (x, y) is seen only inside the view."""
    f = fl.Follower()
    pose = [0.0, 0.0, 0.0]
    assert f.start(0.0, [], find=True) and f.state == "finding"
    t, log = 0.0, []
    while t < T and f.state in ("finding", "following"):
        t += DT
        if round(t / DT) % 4 == 0:
            ppl = [person_msg(7, *person, 0, 0, pose)] if person and in_view(pose, *person) else []
            f.update_people(t, ppl)
        pts = None
        if obstacles is not None:
            c, s_ = math.cos(pose[2]), math.sin(pose[2])
            d = obstacles - np.array(pose[:2])
            pts = np.stack([d[:, 0] * c + d[:, 1] * s_, -d[:, 0] * s_ + d[:, 1] * c], 1)
        vx, wz = f.step(t, tuple(pose), pts)
        drive(f, pose, vx, wz)
        log.append((t, f.state, tuple(pose), vx, wz))
    return f, log


def test_find():
    f, log = _find(person=(-2.0, -0.5))        # behind it
    assert f.state == "following" and f.target_id == 7 and f.found, f.status()
    assert f.status().get("found") is True
    first_turn = next(t for t, st, p, vx, wz in log if abs(wz) > 0.05)
    assert first_turn >= fl.FIND_WAIT_S, f"turned before looking ahead: {first_turn:.2f} s"
    assert all(vx == 0.0 for t, st, p, vx, wz in log if st == "finding"), "drove while finding"
    assert abs(fl.wrap(math.atan2(-0.5 - log[-1][2][1], -2.0 - log[-1][2][0]) - log[-1][2][2])) < 0.3


def test_find_nobody():
    f, log = _find(T=25.0)
    assert f.state == "done" and f.result == "nobody" and "full circle" in f.why, f.status()
    assert log[-1][0] < fl.FIND_WAIT_S + 2 * math.pi / fl.SEARCH_W + 2.0


def test_find_no_room():
    post = np.array([[0.0, 0.25]])             # right beside it: inside the turning circle
    f, log = _find(obstacles=post)
    assert f.result == "nobody" and "too close" in f.why, f.status()
    assert all(abs(wz) < 1e-9 for *_, wz in log), "turned into the post"


def _lose_then(people_after, T=20.0):
    """Follow a RED person at (2, 0) for 2 s; then they are gone and
    people_after(t, pose) is all the camera sees."""
    f = fl.Follower()
    pose = [0.0, 0.0, 0.0]
    assert f.start(0.0, [person_msg(1, 2.0, 0.0, 0, 0, pose, look=RED)])
    t = 0.0
    while t < T and f.state == "following":
        t += DT
        if round(t / DT) % 4 == 0:
            ppl = ([person_msg(1, 2.0, 0.0, 0, 0, pose, look=RED)] if t < 2.0
                   else people_after(t, pose))
            f.update_people(t, ppl)
            if t >= 2.0 and f.target_id != 1:
                return f, t
        drive(f, pose, *f.step(t, tuple(pose), None))
    return f, t


def test_look_reacquires_them_anywhere():
    # 4 s after losing them, the same clothes 3 m to the side of where they were
    seen = lambda t, pose: [person_msg(9, 2.0, 3.0, 0, 0, pose, look=RED)] if t > 6.0 else []
    f, t = _lose_then(seen)
    assert f.state == "following" and f.target_id == 9, f.status()
    assert f.last_sim is not None and f.last_sim >= fl.LOOK_SAME


def test_look_rejects_someone_else_where_they_were():
    # someone in BLUE steps into their spot: not them -- and not followed
    seen = lambda t, pose: [person_msg(5, 2.1, 0.1, 0, 0, pose, look=BLUE)] if t > 3.0 else []
    f, t = _lose_then(seen, T=10.0)
    assert f.target_id == 1, f"followed someone else: {f.status()}"
    assert fl.look_sim(RED, BLUE) < fl.LOOK_OTHER and fl.look_sim(RED, RED) > 0.99


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
    # find=True turns to look instead -- but not for a named id
    assert fl.Follower().start(0.0, [], find=True)
    f3 = fl.Follower()
    assert not f3.start(0.0, [], target_id=4, find=True) and f3.result == "nobody"


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
