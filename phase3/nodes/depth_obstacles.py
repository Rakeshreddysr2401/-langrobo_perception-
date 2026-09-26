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
import math
import time

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

DEPTH = '/camera/camera0/depth/image_rect_raw'
INFO = '/camera/camera0/depth/camera_info'
Z_MAX = 0.27
Z_FLOOR = ((1.0, 0.020), (1.6, 0.035), (9e9, 0.050))   # (range below, min height), m
VOX = (0.01, 0.01, 0.02)        # m, x y z
STRIDE = 3                      # px
RATE_HZ = 5.0
RANGE = 2.5                     # m, depth used and memory kept
MEM_S = 120.0
MIN_HITS = 2
SEE_THROUGH = 0.04              # m beyond a voxel that proves it gone
EDGE = 0.05                     # m of depth spread that marks a flying pixel


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class DepthObstacles:
    """Attach to a node that also keeps `pose` (x, y, th in odom) current:
        dobs = DepthObstacles(node, tf_buffer, lambda: node.pose)
        dobs.points_base()  -> N x 2, base_link"""

    def __init__(self, node, tf_buffer, get_pose):
        self.node, self.buf, self.get_pose = node, tf_buffer, get_pose
        self.K = None
        self.cam = None                  # (R, t) camera optical -> base_link
        self.mem = np.zeros((0, 5))      # x, y, z (odom), t_last, hits
        self.last_t = 0.0
        self.frames = 0
        self.updated = 0.0               # time of the last completed update
        node.create_subscription(CameraInfo, INFO, self._info, qos_profile_sensor_data)
        # RAW: the camera sends 30 Hz; decoding every frame in Python starved
        # the node under load (0 frames in 12 s while the recorder ran,
        # 2026-09-26). Only the RATE_HZ frames used are deserialized.
        node.create_subscription(Image, DEPTH, self._raw, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True)

    def _info(self, m):
        if self.K is None:
            self.K = np.array(m.k).reshape(3, 3)

    def _raw(self, data):
        if time.time() - self.last_t < 1.0 / RATE_HZ or self.K is None:
            return
        self._depth(deserialize_message(data, Image))

    def age(self):
        """Seconds since the view was last updated (inf: never)."""
        return time.time() - self.updated if self.updated else float('inf')

    def _depth(self, m):
        now = time.time()
        pose = self.get_pose()
        if pose is None:
            return
        if self.cam is None:
            if not self.buf.can_transform('base_link', m.header.frame_id, Time()):
                return
            tr = self.buf.lookup_transform('base_link', m.header.frame_id, Time()).transform
            self.cam = (quat_R(tr.rotation), np.array([tr.translation.x, tr.translation.y, tr.translation.z]))
        self.last_t = now
        if m.encoding not in ('16UC1', 'mono16'):
            return
        d = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)[::STRIDE, ::STRIDE]
        d = d.astype(np.float32) * 0.001
        self.update(d, pose, now)

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
        """Remembered obstacle points (seen MIN_HITS times) in base_link, N x 2."""
        pose = pose if pose is not None else self.get_pose()
        mem = self.mem
        if pose is None or not len(mem):
            return np.zeros((0, 2))
        mem = mem[mem[:, 4] >= MIN_HITS]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        q = mem[:, :2] - np.array(pose[:2])
        return np.stack([c * q[:, 0] + s * q[:, 1], -s * q[:, 0] + c * q[:, 1]], 1)
