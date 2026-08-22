#!/usr/bin/env python3
"""Draw the rover as a little car in RViz, instead of an arrow.

An arrow tells you position and heading. It does not tell you how BIG the rover
is, and on a map where the question is usually "does it fit through there" the
size is the thing you want to see. This publishes the real measured footprint --
the same numbers nav2's costmap uses -- as a body with four wheels, so what you
see on screen is the space the rover actually occupies.

Everything is in `base_link`, so TF places it and the markers never need
updating as the rover drives. They are published once a second with
TRANSIENT_LOCAL durability, which means RViz gets them the moment it connects
rather than sitting empty until the next publish.

The nose is a separate colour on purpose: on a 2D map from above, a symmetric
body is genuinely ambiguous about which way it faces, and "which way is forward"
is exactly what you are trying to read when the map looks wrong.

Run:  python3 rover_marker.py
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# MEASURED, and deliberately the same numbers as phase3/config/nav2.yaml's
# footprint. If these two ever disagree, the picture is lying about the thing
# the planner believes.
FRONT = 0.180        # m, camera front face
REAR = -0.170        # m, rear wheel centre + wheel radius
SIDE = 0.190         # m, wheel centre + tyre half-width
BODY_H = 0.10        # m, visual only
BODY_Z = 0.105       # m, sits above the wheels

WHEEL_R = 0.0425     # m, 85 mm tyre OD confirmed with a tape
WHEEL_W = 0.035      # m, visual only
WHEEL_X = 0.1275     # m, from centre to each axle
WHEEL_Y = 0.170      # m, wheel centre

# Orange body against a blue-grey costmap and a magenta obstacle layer: nothing
# else on screen is this colour, which is the whole point of picking it.
BODY_RGBA = (1.00, 0.48, 0.09, 0.95)
NOSE_RGBA = (0.20, 0.90, 1.00, 1.00)   # cyan: this end is the front
WHEEL_RGBA = (0.13, 0.13, 0.16, 1.00)
CAM_RGBA = (0.85, 0.85, 0.88, 1.00)


class RoverMarker(Node):
    def __init__(self):
        super().__init__('rover_marker')
        self.declare_parameter('frame', 'base_link')
        self.frame = self.get_parameter('frame').value

        # TRANSIENT_LOCAL: RViz is almost always started AFTER the robot, and
        # with VOLATILE it would show nothing until the next tick. A late
        # subscriber getting an empty screen is the failure this avoids.
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(MarkerArray, '/rover/model', qos)
        self.create_timer(1.0, self._publish)
        self._publish()
        self.get_logger().info(
            f'rover model on /rover/model in {self.frame} — '
            f'{(FRONT - REAR) * 100:.0f} x {SIDE * 200:.0f} cm')

    def _mk(self, mid, kind, sx, sy, sz, x, y, z, rgba):
        m = Marker()
        m.header.frame_id = self.frame
        m.ns = 'rover'
        m.id = mid
        m.type = kind
        m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _wheel(self, mid, x, y):
        m = self._mk(mid, Marker.CYLINDER,
                     WHEEL_R * 2, WHEEL_R * 2, WHEEL_W,
                     x, y, WHEEL_R, WHEEL_RGBA)
        # A cylinder's axis is +z; a wheel's axis is +y. Rotate 90 deg about x.
        h = math.pi / 4
        m.pose.orientation.x = math.sin(h)
        m.pose.orientation.w = math.cos(h)
        return m

    def _publish(self):
        a = MarkerArray()
        cx = (FRONT + REAR) / 2.0

        # body
        a.markers.append(self._mk(
            0, Marker.CUBE, FRONT - REAR - 0.04, SIDE * 2 - 0.06, BODY_H,
            cx, 0.0, BODY_Z, BODY_RGBA))

        # nose wedge — which way is forward, readable from directly above
        a.markers.append(self._mk(
            1, Marker.CUBE, 0.05, SIDE * 2 - 0.10, BODY_H + 0.01,
            FRONT - 0.045, 0.0, BODY_Z, NOSE_RGBA))

        # the D555, sitting on the front face
        a.markers.append(self._mk(
            2, Marker.CUBE, 0.03, 0.13, 0.04,
            FRONT - 0.015, 0.0, BODY_Z + BODY_H / 2 + 0.02, CAM_RGBA))

        # four wheels
        for i, (x, y) in enumerate([(WHEEL_X, WHEEL_Y), (WHEEL_X, -WHEEL_Y),
                                    (-WHEEL_X, WHEEL_Y), (-WHEEL_X, -WHEEL_Y)]):
            a.markers.append(self._wheel(10 + i, x, y))

        # heading ray: shows where the rover is pointed, well past its own nose,
        # so you can see what it is aimed at rather than only which way it sits.
        ray = self._mk(20, Marker.ARROW, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, NOSE_RGBA)
        ray.scale.x, ray.scale.y, ray.scale.z = 0.022, 0.045, 0.05
        ray.points = [Point(x=FRONT, y=0.0, z=BODY_Z),
                      Point(x=FRONT + 0.30, y=0.0, z=BODY_Z)]
        ray.color.a = 0.9
        a.markers.append(ray)

        self.pub.publish(a)


def main():
    rclpy.init()
    n = RoverMarker()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
