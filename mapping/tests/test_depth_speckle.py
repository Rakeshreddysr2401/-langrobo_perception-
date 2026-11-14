#!/usr/bin/env python3
"""test_depth_speckle.py — the speckle filter in front of the map (mapping/nodes/depth_speckle.py).

    python3 mapping/tests/test_depth_speckle.py      (no robot, no ROS)

Two real D555 frames from 2026-11-14 (fixtures/depth_speckle_20261114.npz): `door` = pinned against
the dark door, 3-5 % scattered blobs at random ranges (it painted the false cells that closed the
doorway); `good` = the marble floor and the door from a metre back.
"""
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import depth_speckle as DS  # noqa: E402

FX = np.load(Path(__file__).parent / 'fixtures' / 'depth_speckle_20261114.npz')
fails = []


def check(name, ok, detail=''):
    print(('  ok   ' if ok else '  FAIL ') + name + (f'  ({detail})' if detail else ''))
    if not ok:
        fails.append(name)


def cdr_image(depth, frame='camera0_depth_optical_frame'):
    """sensor_msgs/Image as the wire carries it (little-endian CDR), built by hand: no ROS in this test."""
    h, w = depth.shape
    b = bytearray(b'\x00\x01\x00\x00') + struct.pack('<iI', 1794668315, 663000000)

    def pad(n):
        while (len(b) - 4) % n:
            b.append(0)

    def string(s):
        pad(4); b.extend(struct.pack('<I', len(s) + 1)); b.extend(s.encode() + b'\x00')
    string(frame); pad(4); b.extend(struct.pack('<II', h, w)); string('16UC1')
    b.append(0); pad(4); b.extend(struct.pack('<I', w * 2)); pad(4); b.extend(struct.pack('<I', h * w * 2))
    b.extend(depth.astype('<u2').tobytes())
    return bytes(b)


fx = float(FX['fx'])
# 1. the door frame: nothing in it is a surface
d = FX['door'].copy(); before = int((d > 0).sum()); DS.clean(d, fx)
check('door frame: the scattered blobs are gone', (d > 0).sum() <= 0.02 * before, f'{before} -> {int((d > 0).sum())} valid px')
# 2. the good frame: the floor stays
g0 = FX['good']; g = g0.copy(); DS.clean(g, fx)
floor0, floor1 = int((g0[336:] > 0).sum()), int((g[336:] > 0).sum())
check('good frame: the floor (bottom third) is kept', floor1 >= 0.85 * floor0, f'{floor1 / floor0 * 100:.0f} %')
check('good frame: nothing is added or moved', bool(np.all((g == g0) | (g == 0))))
# 3. a made floor with a small box standing on it (joined to the floor where it stands) and a blob in the air
h, w = 504, 896
v = np.arange(h, dtype=np.float32)[:, None] + np.zeros((1, w), np.float32)
z = np.zeros((h, w), np.float32); rows = v > 262
z[rows] = 0.175 * fx / (v[rows] - 256)                          # the floor seen from 17.5 cm up
s = (z * 1000).astype(np.uint16)
s[300:340, 400:420] = s[340, 400:420]                             # a 40 x 20 px box whose foot is on floor row 340
s[60:90, 100:130] = 700                                           # 900 px of nothing, in the air
t = s.copy(); DS.clean(t, fx)
check('a thing standing on the floor is kept', bool((t[300:340, 400:420] > 0).all()))
check('a blob with no surface around it is removed', bool((t[60:90, 100:130] == 0).all()))
check('the floor is untouched', bool(np.array_equal(t[rows[:, 0]][:, :90], s[rows[:, 0]][:, :90])))
# 4. off = untouched
u = FX['door'].copy(); n = DS.clean(u, fx, 0)
check('min_patch_px 0 turns it off', n == 0 and np.array_equal(u, FX['door']))
# 5. the wire message: same header, cleaned pixels; a strange message passes untouched
msg = cdr_image(FX['door']); out, removed = DS.clean_cdr(msg, fx)
head = len(msg) - h * w * 2
check('message: header bytes unchanged, length unchanged', out[:head] == msg[:head] and len(out) == len(msg))
check('message: the pixels are the cleaned frame', np.array_equal(np.frombuffer(out, '<u2', h * w, head).reshape(h, w), d) and removed == before - int((d > 0).sum()))
odd = msg[:-10]; o2, r2 = DS.clean_cdr(odd, fx)
check('message: an unexpected layout passes untouched', o2 == odd and r2 == -1)

print('PASS' if not fails else f'FAIL: {fails}')
sys.exit(1 if fails else 0)
