#!/usr/bin/env python3
"""depth_obstacles.py — what the depth camera sees at the rover's height, at 1 cm,
remembered in odom. Used by goal_exec (every goal) and ./rover pass.

WHY NOT THE NVBLOX COSTMAP
    nvblox's slice is 2.5-5 cm cells, and a cell an edge touches is solid: a
    ~50 cm gap read as 40-45 cm at 5 cm (LOCALIZATION.md §13). The raw depth
    points place an edge to ~1 cm. The LiDAR (one plane at 25 cm) cannot stand
    in: it passes between a stool's legs under the seat and sees nothing.

WHY A MEMORY
    The camera looks ahead (87 deg) and cannot see the floor within ~30 cm of
    the nose. Driving through a gap, its edges leave the view before the rover
    reaches them. So points are kept in odom (fusion2, ~1 cm) and read back in
    base_link.

WHY IT FORGETS
    A foot seen while someone stands by is not an obstacle a minute later, and
    a mark that never clears closes gaps (the §13 carry marks). A remembered
    voxel is dropped when the camera, looking at where it was, measures depth
    clearly BEYOND it -- it has seen through it. Voxels out of view keep until
    MEM_S; nothing beyond RANGE is kept.

FILTERS
    band      z up to 0.27 m in base_link (objects over 27 cm are above the
              rover); the FLOOR of the band depends on range, because the
              floor's own noise does. Measured 2026-09-26 on free floor (no
              object within 8 cm): within 0.9 m no floor pixel above 2.0 cm
              (sd 4-7 mm); 0.9-1.7 m, 0.2-0.7% above 2 cm, rare above 2.5.
                  range < 1.0 m    2.0 cm
                  1.0 - 1.6 m      3.5 cm
                  beyond           5.0 cm
              A stool's feet stuck out under the old flat 5 cm and the wheels
              reach the floor: the feet are seen now as the rover closes in,
              and remembered. Flatter than ~2 cm stays invisible.
    edges     pixels whose 3x3 neighbourhood spans > 5 cm of depth are
              dropped: the depth camera's flying pixels at object edges.
    repeats   a voxel counts once seen in MIN_HITS updates.
"""
import json
import math
import struct
import time
from pathlib import Path

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

# the turn gate is shared with nvblox's depth_gate (phase2/nodes/turn_gate.py):
# /opt/rover2/nodes in the container, ../../phase2/nodes on the host
for _d in (Path('/opt/rover2/nodes'), Path(__file__).resolve().parent.parent.parent / 'phase2' / 'nodes'):
    if (_d / 'turn_gate.py').exists():
        import sys as _sys
        _sys.path.insert(0, str(_d))
        break
from turn_gate import TurnGate  # noqa: E402

DEPTH = '/camera/camera0/depth/image_rect_raw'
INFO = '/camera/camera0/depth/camera_info'
Z_MAX = 0.27
Z_FLOOR = ((1.0, 0.020), (1.6, 0.035), (9e9, 0.050))   # (range below, min height), m
VOX = (0.01, 0.01, 0.02)        # m, x y z
STRIDE = 3                      # px
RATE_HZ = 5.0
RANGE = 2.5                     # m, depth used and memory kept
MEM_S = 120.0
# The rover's own outline in base_link (= goal_exec_node._scan's LiDAR self-filter),
# and how far inside it a remembered point must be to be called stale (points_base).
OWN_FRONT, OWN_REAR, OWN_SIDE = 0.202, 0.198, 0.21
OWN_INSET = 0.05
MIN_HITS = 2
SEE_THROUGH = 0.04              # m beyond a voxel that proves it gone
EDGE = 0.05                     # m of depth spread that marks a flying pixel
# The camera mount (base_link -> depth optical) is static and calibrated, but
# a node flooded by 30 Hz images and /tf could go 15 s+ without receiving it
# from /tf_static -- and without it the camera is blind (./rover pass refused
# on "0 frames", 2026-09-26). Saved on every live lookup, used after TF_WAIT.
CAM_CACHE = Path('/logs/calib/depth_cam_tf.json')
TF_WAIT = 3.0                   # s


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class DepthObstacles:
    """Attach to a node that also keeps `pose` (x, y, th in odom) current:
        dobs = DepthObstacles(node, tf_buffer, lambda: node.pose)
        dobs.points_base()  -> N x 2, base_link"""

    def __init__(self, node, tf_buffer, get_pose, active=True):
        self.node, self.buf, self.get_pose = node, tf_buffer, get_pose
        self.K = None
        self.cam = None                  # (R, t) camera optical -> base_link
        self.mem = np.zeros((0, 5))      # x, y, z (odom), t_last, hits
        self.last_t = 0.0
        self.frames = 0
        self.updated = 0.0               # time of the last completed update
        self.first_img = 0.0
        self.cam_src = None
        # a frame taken mid-turn lands degrees off (turn_gate.py); a frame
        # placed with the pose at ARRIVAL, not at its stamp, lands where the
        # rover had got to 0.1-0.2 s later: 2-3 cm at 0.2 m/s straight, and
        # degrees in a turn (2026-10-03). Both are dropped / fixed here.
        self.gate = TurnGate(node, active=False)   # listens only while depth does (set_active)
        self.no_pose_at_stamp = 0
        # ONE message is all it needs (the intrinsics never change), then it
        # unsubscribes: at 30 Hz this was ~30 Python wake-ups a second in each
        # of reach and goal_exec, forever (2026-09-28 CPU diet, OPEN_ISSUES #1).
        self.info_sub = node.create_subscription(CameraInfo, INFO, self._info, qos_profile_sensor_data)
        # RAW: the camera sends 30 Hz; decoding every frame in Python starved
        # the node under load (0 frames in 12 s while the recorder ran,
        # 2026-09-26). Only the RATE_HZ frames used are deserialized.
        # Depth 5, not 1: a frame is ~800 KB in fragments, and with a history
        # of 1 each new frame's fragments evicted the one being reassembled --
        # no image ever arrived (2026-09-26).
        self.sub = None
        self.set_active(active)

    def set_active(self, on: bool) -> None:
        """Subscribe to depth only while someone needs it.

        Receiving is the cost, not decoding: every ~800 KB frame, ~25 a second,
        crosses into Python even though only RATE_HZ are decoded. Idle,
        goal_exec and reach did that all day -- ~40% CPU each, measured
        2026-09-26 -- on a Jetson at 76-88% on every core, where the camera
        driver ("callback took too long") then dropped depth for up to 9 s and
        the brain could not measure an object it had just found. The memory
        is kept while inactive; it just is not updated."""
        if on and self.sub is None:
            self.gate.set_active(True)
            self.sub = self.node.create_subscription(Image, DEPTH, self._raw, qos_profile_sensor_data,
                                                     raw=True)
        elif not on and self.sub is not None:
            self.node.destroy_subscription(self.sub)
            self.sub = None
            self.gate.set_active(False)

    def _info(self, m):
        if self.K is None:
            self.K = np.array(m.k).reshape(3, 3)
        if self.info_sub is not None:
            self.node.destroy_subscription(self.info_sub)
            self.info_sub = None

    def _raw(self, data):
        if time.time() - self.last_t < 1.0 / RATE_HZ or self.K is None:
            return
        # the stamp straight out of the CDR (encapsulation, int32 sec, uint32
        # nanosec): a frame taken mid-turn is not even decoded
        sec, nsec = struct.unpack_from('<iI', data, 4)
        if not self.gate.still(sec + nsec * 1e-9):
            return
        self._depth(deserialize_message(data, Image))

    def _pose_at(self, stamp):
        """odom -> base_link at the frame's own time; None if the TF history
        does not reach it (the frame is skipped, the next one is used)."""
        try:
            tr = self.buf.lookup_transform('odom', 'base_link', Time.from_msg(stamp)).transform
        except Exception:
            self.no_pose_at_stamp += 1
            return None
        q = tr.rotation
        return (tr.translation.x, tr.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def age(self):
        """Seconds since the view was last updated (inf: never)."""
        return time.time() - self.updated if self.updated else float('inf')

    def _depth(self, m):
        now = time.time()
        pose = self._pose_at(m.header.stamp)
        if pose is None:
            return
        if self.cam is None or self.cam_src == 'cache':
            self._mount(m.header.frame_id, now)
            if self.cam is None:
                return
        self.last_t = now
        if m.encoding not in ('16UC1', 'mono16'):
            return
        d = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)[::STRIDE, ::STRIDE]
        d = d.astype(np.float32) * 0.001
        self.update(d, pose, now)

    def _mount(self, frame, now):
        self.first_img = self.first_img or now
        if self.buf.can_transform('base_link', frame, Time()):
            tr = self.buf.lookup_transform('base_link', frame, Time()).transform
            q = [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]
            t = [tr.translation.x, tr.translation.y, tr.translation.z]
            self.cam = (quat_R(tr.rotation), np.array(t))
            self.cam_src = 'tf'
            try:
                CAM_CACHE.parent.mkdir(parents=True, exist_ok=True)
                CAM_CACHE.write_text(json.dumps({'frame': frame, 'q': q, 't': t, 'saved': time.time()}))
            except OSError:
                pass
        elif self.cam is None and now - self.first_img > TF_WAIT:
            try:
                d = json.loads(CAM_CACHE.read_text())
            except (OSError, ValueError):
                return
            if d.get('frame') != frame:
                return
            class _Q:
                pass
            q = _Q()
            q.x, q.y, q.z, q.w = d['q']
            self.cam = (quat_R(q), np.array(d['t']))
            self.cam_src = 'cache'
            self.node.get_logger().warn(f'depth camera mount from {CAM_CACHE} (no /tf_static after {TF_WAIT:.0f} s)')

    # ── the model, no ROS (testable) ────────────────────────────────────────
    def update(self, d, pose, now):
        Rc, tc = self.cam
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        h, w = d.shape
        # flying pixels: depth spread over the 3x3 neighbourhood
        pad = np.pad(d, 1, mode='edge')
        nb = np.stack([pad[i:i + h, j:j + w] for i in range(3) for j in range(3)])
        spread = nb.max(0) - nb.min(0)
        valid = (d > 0.15) & (d < RANGE)
        good = valid & (spread < EDGE)
        v, u = np.nonzero(good)
        z = d[v, u]
        uu, vv = u * STRIDE, v * STRIDE
        P = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], 1) @ Rc.T + tc   # base_link
        rng = np.hypot(P[:, 0], P[:, 1])
        zmin = np.select([rng < r for r, _ in Z_FLOOR], [z for _, z in Z_FLOOR])
        P = P[(P[:, 2] > zmin) & (P[:, 2] < Z_MAX)]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        Wd = np.stack([pose[0] + c * P[:, 0] - s * P[:, 1], pose[1] + s * P[:, 0] + c * P[:, 1], P[:, 2]], 1)

        # forget what the camera now sees through
        mem = self.mem
        if len(mem):
            q = mem[:, :3] - np.array([pose[0], pose[1], 0.0])
            B = np.stack([c * q[:, 0] + s * q[:, 1], -s * q[:, 0] + c * q[:, 1], q[:, 2]], 1)  # base
            C = (B - tc) @ Rc                                                              # optical
            zc = C[:, 2]
            inview = zc > 0.15
            pu = np.full(len(C), -1, int)
            pv = np.full(len(C), -1, int)
            pu[inview] = np.round((C[inview, 0] * fx / zc[inview] + cx) / STRIDE).astype(int)
            pv[inview] = np.round((C[inview, 1] * fy / zc[inview] + cy) / STRIDE).astype(int)
            inview &= (pu >= 0) & (pu < w) & (pv >= 0) & (pv < h)
            gone = np.zeros(len(C), bool)
            meas = d[pv[inview], pu[inview]]
            gone[inview] = valid[pv[inview], pu[inview]] & (meas > zc[inview] + SEE_THROUGH)
            far = np.hypot(q[:, 0], q[:, 1]) > RANGE
            old = now - mem[:, 3] > MEM_S
            mem = mem[~(gone | far | old)]

        # merge the new observation (one hit per voxel per update)
        if len(Wd):
            key = np.floor(Wd / np.array(VOX)).astype(np.int64)
            key, idx = np.unique(key, axis=0, return_index=True)
            new = np.column_stack([(key + 0.5) * np.array(VOX), np.full(len(key), now), np.ones(len(key))])
            if len(mem):
                allk = np.vstack([np.floor(mem[:, :3] / np.array(VOX)).astype(np.int64), key])
                allv = np.vstack([mem, new])
                uk, inv = np.unique(allk, axis=0, return_inverse=True)
                inv = inv.ravel()
                out = np.zeros((len(uk), 5))
                out[:, :3] = (uk + 0.5) * np.array(VOX)
                np.maximum.at(out[:, 3], inv, allv[:, 3])
                np.add.at(out[:, 4], inv, allv[:, 4])
                out[:, 4] = np.minimum(out[:, 4], 50)
                mem = out
            else:
                mem = new
        self.mem = mem
        self.frames += 1
        self.updated = time.time()

    def points_base(self, pose=None):
        """Remembered obstacle points (seen MIN_HITS times) in base_link, N x 2.

        A remembered point INSIDE the rover's own outline is forgotten here: the
        rover is standing on that spot, so nothing is there now. Nothing else
        could clear it -- the camera cannot see within ~30 cm of the nose, let
        alone under the rover -- so it boxed the rover in until MEM_S ran out:
        2026-10-02 at a door, every turn and drive refused for ~85 s with
        "something 0.01-0.02 m away" (measured from base_link's centre: inside
        the body), though the owner could see free space. The outline is the one
        goal_exec_node._scan drops from the LiDAR as "the rover itself".

        Only DEEP inside: OWN_INSET (5 cm) in from every edge. The first version
        forgot every point inside the outline, and a real obstacle a turn slides
        the outline 1-2 cm onto (a low chair foot beside a 45 cm gap, under the
        LiDAR and out of the camera's view -- only this memory knew it) was
        forgotten at the moment it mattered, and the rover turned into it
        (2026-10-03). The door phantoms were 1-2 cm from the CENTRE; real contact
        shows at the edge, and must keep blocking."""
        pose = pose if pose is not None else self.get_pose()
        full = self.mem
        if pose is None or not len(full):
            return np.zeros((0, 2))
        c, s = math.cos(pose[2]), math.sin(pose[2])
        q = full[:, :2] - np.array(pose[:2])
        b = np.stack([c * q[:, 0] + s * q[:, 1], -s * q[:, 0] + c * q[:, 1]], 1)
        own = ((b[:, 0] < OWN_FRONT - OWN_INSET) & (b[:, 0] > -OWN_REAR + OWN_INSET)
               & (np.abs(b[:, 1]) < OWN_SIDE - OWN_INSET))
        if own.any():
            self.mem = full[~own]
        return b[(~own) & (full[:, 4] >= MIN_HITS)]
