"""sonar_feed.py -- the front sonars for a node that decides on obstacle points.

    feed = SonarFeed(node, tf_buffer)
    extra = feed.points(known)      # N x 2 in base_link: the UNEXPLAINED close echoes

Subscribes /sonar/left and /sonar/right (sensor_msgs/Range: the Pi 5's
sonar_node on the robot, isaac/ros_io.py on the twin). Each sensor's mount
(base_link <- its frame) comes from TF once -- the rover URDF's sonar_left /
sonar_right -- and is then kept. Readings older than MAX_AGE_S are not used,
and without the frames (an old URDF still being served) the sonars are simply
not used: nothing here can stop the rover by being absent. sonar_witness.py
has the rule (only echoes the map cannot explain become points).
"""
import math
import os
import time
from collections import deque

import numpy as np

from rclpy.time import Time
from sensor_msgs.msg import Range

from sonar_witness import vouched_specks, witness_points

MAX_AGE_S = 0.3
VOUCH_WINDOW_S = 0.5           # the readings a vouch may use
# SONAR_CLEAR=off: never overrule a camera speck (sonar_witness.vouched_specks)
CLEAR = os.environ.get('SONAR_CLEAR', 'on') != 'off'
TOPICS = ('/sonar/left', '/sonar/right')


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SonarFeed:
    def __init__(self, node, buf):
        self.node, self.buf = node, buf
        self.last = {}                 # frame -> (time, range, fov)
        self.hist = {}                 # frame -> recent (time, range)
        self.mount = {}                # frame -> (x, y, yaw) in base_link
        self.warned = set()
        self.used = 0                  # how many points the last call added (for status lines)
        self.cleared = 0               # how many camera points the last vouch overruled
        for t in TOPICS:
            node.create_subscription(Range, t, self._range, 10)

    def _range(self, m):
        now = time.time()
        self.last[m.header.frame_id] = (now, float(m.range), float(m.field_of_view))
        self.hist.setdefault(m.header.frame_id, deque(maxlen=8)).append((now, float(m.range)))

    def _mount(self, frame):
        if frame not in self.mount:
            try:
                tr = self.buf.lookup_transform('base_link', frame, Time()).transform
                self.mount[frame] = (tr.translation.x, tr.translation.y, _yaw(tr.rotation))
            except Exception:  # noqa: BLE001 - no frame (old URDF): the sonar is not used
                if frame not in self.warned:
                    self.warned.add(frame)
                    self.node.get_logger().warning(f'sonar frame {frame} not in TF: that sonar is not used')
                return None
        return self.mount[frame]

    def points(self, known):
        now, readings, fov = time.time(), [], 0.52
        for frame, (t, r, f) in self.last.items():
            if now - t > MAX_AGE_S:
                continue
            m = self._mount(frame)
            if m is not None:
                readings.append((r, m))
                fov = f
        p = witness_points(readings, known, fov)
        self.used = len(p)
        return p

    def vouch(self, camera, lidar):
        """Bool mask over camera (N x 2, base_link): the lone specks the sonars vouch
        are not there (sonar_witness.vouched_specks). All False when switched off,
        without fresh readings, or without the frames."""
        self.cleared = 0
        if not CLEAR or camera is None or not len(camera):
            return np.zeros(0 if camera is None else len(camera), bool)
        now, sensors = time.time(), []
        for frame, h in self.hist.items():
            recent = [r for t, r in h if now - t <= VOUCH_WINDOW_S]
            m = self._mount(frame)
            if m is not None and recent:
                sensors.append((m, recent))
        mask = vouched_specks(camera, lidar, sensors)
        self.cleared = int(mask.sum())
        return mask
