"""frame_store -- WHEN to keep a frame and HOW it is kept on disk. No ROS (frame_capture.py is the node).

Frame memory (Pi 5 ~/ros2_ws docs/plans/FRAME_MEMORY.md, 2026-11-21): every keyframe the robot sees is
kept with its colour JPEG, its aligned depth and the camera's pose AT THE PHOTO'S STAMP, so a thing found
in it later -- minutes or days later -- is placed from that frame's own depth and pose (x1 + dx, y1 + dy),
wherever the robot is by then. The Mac's memory service embeds the JPEG (EmbeddingGemma 2); the depth
stays here and pixel_to_goal grounds a box on it.

A keyframe when, since the last kept one, the robot
  moved >= MOVE_M, or turned >= TURN_DEG          a new view
  OR the scene changed while it stood still        someone put a bottle in front of a parked robot:
                                                   the thumbnail differs by >= SCENE_DIFF (mean |dI|, 0-255)
  OR STILL_S passed                                a slow change
and never more often than MIN_GAP_S.
"""
import json
import os
import time

import numpy as np

MOVE_M = float(os.environ.get('FRAMES_MOVE_M', '0.40'))
TURN_DEG = float(os.environ.get('FRAMES_TURN_DEG', '20'))
STILL_S = float(os.environ.get('FRAMES_STILL_S', '30'))
SCENE_DIFF = float(os.environ.get('FRAMES_SCENE_DIFF', '10'))
MIN_GAP_S = float(os.environ.get('FRAMES_MIN_GAP_S', '1.0'))
# "standing still" for the scene test: a moving camera always changes the picture
STILL_MOVE_M, STILL_TURN_DEG = 0.03, 2.0
THUMB_W, THUMB_H = 32, 18


def ang_deg(a: float, b: float) -> float:
    """|a - b| in degrees, wrapped to [0, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return abs(d)


def thumb(gray: np.ndarray) -> np.ndarray:
    """THUMB_W x THUMB_H block-mean of a grayscale image (float32). cv2 INTER_AREA when there
    (the numpy loop cost ~25 ms a frame on the Jetson, 2026-11-21), the same means without it."""
    try:
        import cv2
        return cv2.resize(gray.astype(np.float32), (THUMB_W, THUMB_H), interpolation=cv2.INTER_AREA)
    except ImportError:
        pass
    h, w = gray.shape[:2]
    ys = np.linspace(0, h, THUMB_H + 1).astype(int)
    xs = np.linspace(0, w, THUMB_W + 1).astype(int)
    g = gray.astype(np.float32)
    return np.array([[g[ys[i]:ys[i + 1], xs[j]:xs[j + 1]].mean() for j in range(THUMB_W)]
                     for i in range(THUMB_H)], dtype=np.float32)


def scene_diff(a: np.ndarray, b: np.ndarray) -> float:
    """Mean absolute difference of two thumbnails, with the overall brightness change removed
    (auto-exposure shifts the whole picture; that is not a change in the room)."""
    return float(np.abs((a - a.mean()) - (b - b.mean())).mean())


class Trigger:
    """Decides, frame by frame, whether to keep it. pose = (x, y, yaw_deg) in odom at the photo's stamp."""

    def __init__(self):
        self.last = None            # (x, y, yaw_deg, t, thumb)

    def check(self, pose, t: float, th: np.ndarray):
        """The reason to keep this frame ('first', 'moved', 'turned', 'scene', 'still'), or None."""
        if self.last is None:
            return 'first'
        lx, ly, lyaw, lt, lth = self.last
        if t - lt < MIN_GAP_S:
            return None
        moved = float(np.hypot(pose[0] - lx, pose[1] - ly))
        turned = ang_deg(pose[2], lyaw)
        if moved >= MOVE_M:
            return 'moved'
        if turned >= TURN_DEG:
            return 'turned'
        if moved < STILL_MOVE_M and turned < STILL_TURN_DEG and lth is not None and th is not None \
                and scene_diff(th, lth) >= SCENE_DIFF:
            return 'scene'
        if t - lt >= STILL_S:
            return 'still'
        return None

    def kept(self, pose, t: float, th: np.ndarray):
        self.last = (pose[0], pose[1], pose[2], t, th)


def write_bundle(root: str, meta: dict, jpeg: bytes, depth_png: bytes | None) -> str:
    """<root>/<origin>/<id>.jpg|.depth.png|.json, written so a reader never sees half a frame:
    the .json goes last and is the 'this frame is complete' marker. Returns the json path."""
    d = os.path.join(root, str(meta['origin']))
    os.makedirs(d, exist_ok=True)
    base = os.path.join(d, str(meta['id']))
    with open(base + '.jpg', 'wb') as f:
        f.write(jpeg)
    if depth_png is not None:
        with open(base + '.depth.png', 'wb') as f:
            f.write(depth_png)
    tmp = base + '.json.tmp'
    with open(tmp, 'w') as f:
        json.dump(meta, f)
    os.replace(tmp, base + '.json')
    return base + '.json'


def disk_use(root: str) -> int:
    n = 0
    for dp, _, fs in os.walk(root):
        for f in fs:
            try:
                n += os.path.getsize(os.path.join(dp, f))
            except OSError:
                pass
    return n


def prune(root: str, max_bytes: int) -> int:
    """Delete the OLDEST frames (by id = stamp) until under max_bytes. Returns frames deleted."""
    use = disk_use(root)
    if use <= max_bytes:
        return 0
    frames = []
    for dp, _, fs in os.walk(root):
        for f in fs:
            if f.endswith('.json'):
                frames.append((f[:-5], dp))
    frames.sort(key=lambda x: int(x[0]) if x[0].isdigit() else 0)
    gone = 0
    for fid, dp in frames:
        if use <= max_bytes:
            break
        for ext in ('.json', '.jpg', '.depth.png', '.sent'):
            p = os.path.join(dp, fid + ext)
            try:
                use -= os.path.getsize(p)
                os.remove(p)
            except OSError:
                pass
        gone += 1
    return gone


def now() -> float:
    return time.monotonic()


# ── reading a remembered frame back (pixel_to_goal grounds a box on it) ──────
def find_frame(root: str, stamp_ns: int):
    """The .json of the kept frame with this stamp (any origin), or None."""
    import glob
    hits = glob.glob(os.path.join(root, '*', f'{int(stamp_ns)}.json'))
    return hits[0] if hits else None


def load_frame(json_path: str):
    """(meta, depth uint16 array or None) of a kept frame."""
    meta = json.load(open(json_path))
    depth = None
    if meta.get('depth'):
        import cv2
        depth = cv2.imread(os.path.join(os.path.dirname(json_path), meta['depth']), cv2.IMREAD_UNCHANGED)
    return meta, depth


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _qrot(q, v):
    x, y, z, _ = _qmul(_qmul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def compose(a, b):
    """a * b for poses (t, q): b expressed in a's frame -> the outer frame."""
    ta, qa = a
    tb, qb = b
    r = _qrot(qa, tb)
    return ((ta[0] + r[0], ta[1] + r[1], ta[2] + r[2]), _qmul(qa, qb))


def invert(a):
    t, q = a
    qi = (-q[0], -q[1], -q[2], q[3])
    r = _qrot(qi, t)
    return ((-r[0], -r[1], -r[2]), qi)


def cam_pose_now(meta: dict, origin_now, map_odom_now):
    """The remembered frame's camera pose in TODAY's odom: (t, q), why.
    Same odom (origin) -> as stored. Another power-on -> through the house map:
    odom_now <- map <- odom_then <- camera. None when that is not possible."""
    cam = (tuple(meta['cam_odom']['t']), tuple(meta['cam_odom']['q']))
    if origin_now is not None and meta.get('origin') == origin_now:
        return cam, 'same_odom'
    mo_then = meta.get('map_odom')
    if mo_then and map_odom_now:
        then = (tuple(mo_then['t']), tuple(mo_then['q']))
        return compose(invert(map_odom_now), compose(then, cam)), 'via_house_map'
    return None, 'other_power_on_without_map'
