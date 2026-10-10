#!/usr/bin/env python3
"""test_person_tracker.py — offline checks of nodes/person_tracker.py.

    python3 phase4/tools/test_person_tracker.py      (no robot, no ROS graph)

walking: one person crossing the room keeps one id. two: two people walking
past each other keep their own ids. occluded: a person hidden for 1 s comes
back with the same id; hidden for 3 s is someone new. weak: a low-confidence
box continues a track but never starts one. stray: a single box is never
reported.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "nodes"))
import person_tracker as pt  # noqa: E402

DT = 0.2  # 5 Hz, detections_3d's default detect_rate


def det(x, y, conf=0.8):
    return {"x": x, "y": y, "conf": conf}


def ids(people):
    return sorted(p["id"] for p in people if p["lost_s"] == 0)


def test_walking():
    tr = pt.PersonTracker()
    seen = set()
    for k in range(30):                       # 1.2 m/s for 6 s
        out = tr.update([det(0.5 + 0.24 * k, 1.0)], k * DT)
        seen.update(ids(out))
    assert seen == {1}, seen
    assert abs(out[0]["vx"] - 1.2) < 0.2, out[0]


def test_two_cross():
    tr = pt.PersonTracker()
    for k in range(25):                       # A walks +x, B walks -x, 0.6 m apart in y
        a, b = det(0.0 + 0.2 * k, 0.0), det(5.0 - 0.2 * k, 0.6)
        out = tr.update([a, b], k * DT)
    by_id = {p["id"]: p for p in out}
    assert set(by_id) == {1, 2}, by_id
    assert by_id[1]["x"] > by_id[2]["x"], "ids swapped when they passed"


def test_occluded():
    tr = pt.PersonTracker()
    t = 0.0
    for _ in range(5):
        tr.update([det(2.0, 0.0)], t); t += DT
    for _ in range(5):                        # 1 s behind a chair
        out = tr.update([], t); t += DT
    assert out and out[0]["lost_s"] > 0
    out = tr.update([det(2.1, 0.05)], t)
    assert ids(out) == [1], out

    tr = pt.PersonTracker()
    t = 0.0
    for _ in range(5):
        tr.update([det(2.0, 0.0)], t); t += DT
    for _ in range(15):                       # 3 s gone: forgotten
        tr.update([], t); t += DT
    tr.update([det(2.0, 0.0)], t); t += DT
    out = tr.update([det(2.0, 0.0)], t)
    assert ids(out) == [2], out


def test_weak():
    tr = pt.PersonTracker()
    t = 0.0
    for _ in range(3):
        tr.update([det(1.0, 0.0)], t); t += DT
    out = tr.update([det(1.05, 0.0, conf=0.22)], t); t += DT
    assert ids(out) == [1], "weak box should continue the track"
    for _ in range(5):
        out = tr.update([det(3.0, 2.0, conf=0.22)], t); t += DT
    assert all(p["id"] == 1 for p in out), "weak box must not start a track"


def test_stray():
    tr = pt.PersonTracker()
    assert tr.update([det(1.0, 1.0)], 0.0) == []
    assert tr.update([], DT) == []


def test_jump_is_someone_else():
    tr = pt.PersonTracker()
    t = 0.0
    for _ in range(3):
        tr.update([det(1.0, 0.0)], t); t += DT
    tr.update([det(3.0, 0.0)], t); t += DT    # 2 m in 0.2 s: not the same person
    out = tr.update([det(3.0, 0.0)], t)
    assert 2 in ids(out), out


def test_one_person_two_boxes():
    tr = pt.PersonTracker()
    t = 0.0
    for _ in range(5):        # whole body + upper body, 0.4 m apart in depth
        whole = dict(det(1.0, 0.0, 0.85), box=[200, 0, 440, 480])
        upper = dict(det(1.3, 0.3, 0.6), box=[220, 0, 430, 250])
        out = tr.update([whole, upper], t); t += DT
    assert ids(out) == [1], out


def test_two_people_side_by_side():
    tr = pt.PersonTracker()
    t = 0.0
    for _ in range(5):
        a = dict(det(2.0, 0.3), box=[100, 50, 250, 470])
        b = dict(det(2.0, -0.3), box=[330, 50, 480, 470])
        out = tr.update([a, b], t); t += DT
    assert ids(out) == [1, 2], out


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
    if pt.linear_sum_assignment is None:
        print("(scipy missing: ran the greedy matcher)")
    sys.exit(1 if fails else 0)
