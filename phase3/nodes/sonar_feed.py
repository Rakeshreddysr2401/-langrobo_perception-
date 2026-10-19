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
import time

from rclpy.time import Time
from sensor_msgs.msg import Range

from sonar_witness import witness_points

MAX_AGE_S = 0.3
TOPICS = ('/sonar/left', '/sonar/right')


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SonarFeed:
    def __init__(self, node, buf):
        self.node, self.buf = node, buf
        self.last = {}                 # frame -> (time, range, fov)
        self.mount = {}                # frame -> (x, y, yaw) in base_link
        self.warned = set()
        self.used = 0                  # how many points the last call added (for status lines)
        for t in TOPICS:
            node.create_subscription(Range, t, self._range, 10)

    def _range(self, m):
        self.last[m.header.frame_id] = (time.time(), float(m.range), float(m.field_of_view))

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
