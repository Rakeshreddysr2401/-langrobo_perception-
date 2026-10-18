#!/usr/bin/env python3
"""Keep the slam_toolbox room on disk so a power cycle does not lose it -- and never
let a wrong lock overwrite a good room.

    python3 map_autosave.py [period_s]        (run by ./rover slam, in the container)

Every period_s it asks slam_toolbox to serialize its graph into /maps/.autosave
and, only if that worked AND the gate below passes, moves it over
/maps/last.{posegraph,data}, writes the occupancy picture /maps/last.{pgm,yaml}
(what relocalize.py matches a scan against at the next start) and the pose the
rover stood at (/maps/last.pose, map frame). The previous last.* is kept as
last.prev.* first. Writes go to a temp name and are renamed, so a power cut
mid-save leaves the old good files.

THE GATE (2026-10-18). The room was resumed from where the rover stood at the
last save; it had been carried elsewhere, slam_toolbox locked onto the wrong
place, and this autosave wrote that over the good room within 2 minutes. Now:
the room as it was when SLAM started (/maps/.ref.*, copied by ./rover slam) is
the reference; at every save the current scan is laid on it at the pose SLAM
claims. Scan endpoints on floor the reference knows is open (relocalize.conflict)
mean SLAM is wrong about where the rover is: the save goes to suspect.* and
last.* is kept. Endpoints in UNKNOWN space are a newly explored room -- fine.
"""
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import relocalize as RL  # noqa: E402

MAPS = '/maps'
EXTS = ('posegraph', 'data')
CONFLICT_MAX = 0.20      # > this fraction of scan endpoints on known-open floor: SLAM's pose is wrong
CONFLICT_SAVES = 2       # ... at this many saves in a row (one bad moment -- a person -- is not enough)


class Watch:
    """One node for the whole run: the newest /map, /scan and map -> base_link."""

    def __init__(self):
        import rclpy
        import tf2_ros
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import LaserScan
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node('map_autosave')
        self.map = self.scan = None
        self.node.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m),
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m),
                                      QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self.node)
        # /home/room (latched): which saved room this map frame IS, so the Pi 5 brain can keep
        # saved places across power cycles (it stores them in this frame, per room id).
        from std_msgs.msg import String
        self.room_pub = self.node.create_publisher(String, '/home/room',
                                                   QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.String = String

    def announce_room(self):
        import json
        try:
            rid = Path(f'{MAPS}/last.id').read_text().strip()
            state = Path(f'{MAPS}/last.state').read_text().strip() if os.path.exists(f'{MAPS}/last.state') else 'new'
        except OSError:
            return None
        self.room_pub.publish(self.String(data=json.dumps({'id': rid, 'frame': 'map', 'state': state})))
        return rid

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def pose(self, parent='map', child='base_link'):
        from rclpy.time import Time
        try:
            t = self.buf.lookup_transform(parent, child, Time()).transform
        except Exception:  # noqa: BLE001 - not there yet
            return None
        q = t.rotation
        return t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def scan_points(self):
        s = self.scan
        if s is None:
            return None
        lp = self.pose('base_link', s.header.frame_id)
        if lp is None:
            return None
        return RL.scan_points(s.ranges, s.angle_min, s.angle_increment, lp)


def write_map(msg, stem):
    """An OccupancyGrid -> stem.pgm + stem.yaml, as nav2's map_saver writes them (trinary)."""
    g = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    img = np.full(g.shape, 205, np.uint8)           # unknown
    img[(g >= 0) & (g < 25)] = 254                  # free
    img[g >= 65] = 0                                # occupied
    img = np.flipud(img)                            # the top row first
    h, w = img.shape
    with open(stem + '.pgm.tmp', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(img.tobytes())
    o = msg.info.origin.position
    with open(stem + '.yaml.tmp', 'w') as f:
        f.write(f'image: {os.path.basename(stem)}.pgm\nmode: trinary\nresolution: {msg.info.resolution}\n'
                f'origin: [{o.x}, {o.y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    os.replace(stem + '.pgm.tmp', stem + '.pgm')
    os.replace(stem + '.yaml.tmp', stem + '.yaml')


def load_ref():
    """The room SLAM started from (./rover slam copies last.* to .ref.* before resuming), or None."""
    y = f'{MAPS}/.ref.yaml'
    if not os.path.exists(y):
        return None
    try:
        return RL.load_map(y)
    except Exception as e:  # noqa: BLE001 - a bad reference disables the gate, never the saves
        print('reference room unreadable, gate off:', e, flush=True)
        return None


def save_once(w, ref, strikes):
    """-> (ok, why, strikes)."""
    w.spin(1.0)
    pose = w.pose()
    if pose is None:
        return False, 'no map -> base_link transform', strikes
    target = 'last'
    if ref is not None:
        pts = w.scan_points()
        if pts is not None and len(pts) >= 20:
            c = RL.conflict(*ref, pts, pose)
            strikes = strikes + 1 if c > CONFLICT_MAX else 0
            if strikes >= CONFLICT_SAVES:
                target = 'suspect'
                print(f'{time.strftime("%H:%M:%S")} GATE: {100 * c:.0f} % of the scan lands on floor the saved '
                      f'room knows is open, at the pose SLAM gives ({pose[0]:.2f}, {pose[1]:.2f}) -- SLAM is '
                      f'probably wrong about where the rover is. Saving to suspect.*, keeping last.*', flush=True)
    tmp = f'{MAPS}/.autosave'
    cmd = ['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
           'slam_toolbox/srv/SerializePoseGraph', f"{{filename: '{tmp}'}}"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    if 'result=0' not in r.stdout.replace(' ', '').replace('\n', ''):
        # ResultSUCCESS is 0 for SerializePoseGraph; anything else, keep the old files
        return False, r.stdout.strip()[-200:] or r.stderr.strip()[-200:], strikes
    if not all(os.path.getsize(f'{tmp}.{e}') > 0 for e in EXTS):
        return False, 'empty output', strikes
    if target == 'last':
        for e in EXTS + ('pgm', 'yaml', 'pose'):
            if os.path.exists(f'{MAPS}/last.{e}'):
                shutil.copy2(f'{MAPS}/last.{e}', f'{MAPS}/last.prev.{e}')
    for e in EXTS:
        os.replace(f'{tmp}.{e}', f'{MAPS}/{target}.{e}')
    if w.map is not None:
        write_map(w.map, f'{MAPS}/{target}')
    with open(f'{MAPS}/.pose.tmp', 'w') as f:
        f.write('%.3f,%.3f,%.3f\n' % pose)
    os.replace(f'{MAPS}/.pose.tmp', f'{MAPS}/{target}.pose')
    return True, '' if target == 'last' else 'to suspect.* (gate)', strikes


def main():
    period = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    w = Watch()
    rid = w.announce_room()
    print('room id:', rid or 'none (no maps/last.id: /home/room not published)', flush=True)
    ref = load_ref()
    print('reference room:', 'maps/.ref (the gate is on)' if ref is not None else 'none (a fresh room: no gate)', flush=True)
    strikes = 0
    while True:
        w.spin(period)
        try:
            ok, why, strikes = save_once(w, ref, strikes)
        except Exception as e:  # noqa: BLE001 - keep trying; a missed save is not fatal
            ok, why = False, repr(e)
        print(time.strftime('%H:%M:%S'), ('saved ' + (why or 'last')) if ok else f'save failed: {why}', flush=True)


if __name__ == '__main__':
    main()
