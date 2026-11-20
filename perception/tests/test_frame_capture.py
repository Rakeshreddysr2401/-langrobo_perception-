#!/usr/bin/env python3
"""test_frame_capture.py -- offline checks of frame_store (when a keyframe is kept, how it is stored).

    python3 perception/tests/test_frame_capture.py      (no robot, no ROS)
"""
import json
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import frame_store as fs  # noqa: E402

fails = 0


def check(name, cond):
    global fails
    print(('PASS ' if cond else 'FAIL ') + name)
    fails += not cond


rng = np.random.default_rng(0)
room = rng.integers(40, 200, (72, 112)).astype(np.uint8)
th = fs.thumb(room)
check('thumb is 18 x 32', th.shape == (fs.THUMB_H, fs.THUMB_W))

# brightness change only (auto-exposure) is not a scene change
check('exposure shift is not a change', fs.scene_diff(fs.thumb(np.clip(room.astype(int) + 30, 0, 255)), th) < 3)
# a bottle (a bright block, ~6 % of the picture) appears
bottle = room.copy()
bottle[30:55, 50:65] = 255
check('a new object is a change', fs.scene_diff(fs.thumb(bottle), th) >= fs.SCENE_DIFF)

t = fs.Trigger()
check('first frame kept', t.check((0, 0, 0), 0.0, th) == 'first')
t.kept((0, 0, 0), 0.0, th)
check('too soon -> no', t.check((1, 0, 0), 0.5, th) is None)
check('moved 0.4 m -> moved', t.check((0.41, 0, 0), 2.0, th) == 'moved')
check('turned 21 deg -> turned', t.check((0, 0, 21), 2.0, th) == 'turned')
check('turn wraps at 180', t.check((0, 0, 0), 2.0, th) is None and fs.ang_deg(179, -179) == 2)
check('still, same scene -> no', t.check((0.01, 0, 0.5), 5.0, th) is None)
check('still, bottle appears -> scene', t.check((0.01, 0, 0.5), 5.0, fs.thumb(bottle)) == 'scene')
check('moving 10 cm, picture differs -> not "scene"', t.check((0.10, 0, 0), 5.0, fs.thumb(bottle)) is None)
check('30 s still -> still', t.check((0, 0, 0), 31.0, th) == 'still')

with tempfile.TemporaryDirectory() as d:
    for i in range(5):
        fs.write_bundle(d, {'id': 1000 + i, 'origin': 7}, b'J' * 1000, b'D' * 2000)
    files = sorted(os.listdir(os.path.join(d, '7')))
    check('bundle = jpg + depth + json', '1000.jpg' in files and '1000.depth.png' in files and '1000.json' in files)
    check('json is complete', json.load(open(os.path.join(d, '7', '1000.json')))['id'] == 1000)
    check('no temp left', not any(f.endswith('.tmp') for f in files))
    use = fs.disk_use(d)
    gone = fs.prune(d, use - 2500)            # must drop the 1-2 oldest
    left = sorted(f for f in os.listdir(os.path.join(d, '7')) if f.endswith('.json'))
    check('prune drops the oldest first', gone >= 1 and left[0] != '1000.json' and '1004.json' in left)
    check('prune gets under the limit', fs.disk_use(d) <= use - 2500)

# ── remembered frames: poses back in today's odom ───────────────────────────
import math  # noqa: E402


def yawq(deg):
    return (0.0, 0.0, math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2))


def close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


p = ((1.0, 2.0, 0.3), yawq(30))
check('invert . compose = identity', close(fs.compose(fs.invert(p), p)[0], (0, 0, 0)))
check('compose rotates then shifts', close(fs.compose(((1, 0, 0), yawq(90)), ((1, 0, 0), yawq(0)))[0], (1, 1, 0)))

cam = {'t': [0.5, 0.0, 0.17], 'q': list(yawq(0))}
meta = {'origin': 111.0, 'cam_odom': cam, 'map_odom': {'t': [2.0, 1.0, 0.0], 'q': list(yawq(90))}}
pose, why = fs.cam_pose_now(meta, 111.0, None)
check('same power-on -> pose as stored', why == 'same_odom' and close(pose[0], (0.5, 0.0, 0.17)))
# after a power cycle the new odom sits elsewhere in the house map: map <- odom_now = (3, 1), yaw 90
pose, why = fs.cam_pose_now(meta, 222.0, ((3.0, 1.0, 0.0), yawq(90)))
# camera in map = (2,1) + rot90(0.5,0) = (2, 1.5); in odom_now: rot(-90)((2,1.5) - (3,1)) = rot(-90)(-1, 0.5) = (0.5, 1)
check('other power-on -> through the house map', why == 'via_house_map' and close(pose[0], (0.5, 1.0, 0.17), 1e-6))
pose, why = fs.cam_pose_now({**meta, 'map_odom': None}, 222.0, ((3.0, 1.0, 0.0), yawq(90)))
check('other power-on, no map then -> refuse', pose is None and why == 'other_power_on_without_map')

with tempfile.TemporaryDirectory() as d:
    import cv2
    dep = (np.arange(252 * 448) % 4000).astype(np.uint16).reshape(252, 448)
    ok, enc = cv2.imencode('.png', dep)
    fs.write_bundle(d, {'id': 555, 'origin': 9, 'depth': '555.depth.png'}, b'J', enc.tobytes())
    j = fs.find_frame(d, 555)
    m, back = fs.load_frame(j)
    check('find_frame by stamp', j is not None and fs.find_frame(d, 556) is None)
    check('depth comes back exactly (16-bit)', back is not None and back.dtype == np.uint16 and (back == dep).all())

print(f"\n{'ALL PASS' if not fails else f'{fails} FAILED'}")
sys.exit(1 if fails else 0)
