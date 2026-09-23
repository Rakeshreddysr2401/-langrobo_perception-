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

WHAT IS DRAWN, and why each piece is separate
    chassis    36 x 28 x 16 cm frame, 6 cm off the floor
    wheels     at their real axles, so they stick out past the chassis and the
               silhouette BECOMES the envelope instead of a cube asserting it
    footprint  a flat outline of the 36 x 38 cm polygon nav2 keeps clear. This
               is the one to read for "does it fit through there" -- the body
               is smaller and the wheels only touch the envelope at four points
    lidar      the C1 puck on top. Every beam on screen originates here, so if
               the ring looks offset from the walls, start by asking whether
               this puck is where the TF claims
    nose + ray cyan, pointing forward, visible from directly above

Run:  python3 rover_marker.py
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# EVERY NUMBER BELOW is derived by description/build from description/params.yaml
# (tape-measured 2026-09-24) and checked against it on every build -- change
# them there. This node retires once RViz shows the URDF (build plan step 4).
#
# THE OUTER ENVELOPE -- the same numbers as phase3/config/nav2.yaml's footprint.
# If these two ever disagree the picture is lying about the thing the planner
# believes, which is the one job this node has. Not centred: base_link is
# midway between the axles, 2 mm ahead of the frame's centre.
FRONT = 0.182        # m, the frame's nose
REAR = -0.178        # m, the frame's tail (the tyres stop inside it)
SIDE = 0.190         # m, outer face of the tyres

# THE FRAME ITSELF, narrower than the envelope: the tyres stick out 5 cm past
# each side, and drawing them at their real positions makes the silhouette.
BODY_L = 0.36        # m, nose to tail
BODY_W = 0.28        # m, across the frame, tyres excluded
BODY_H = 0.16        # m, base frame to top
BODY_X = (FRONT + REAR) / 2   # m, the frame's centre
BODY_Z = 0.06 + BODY_H / 2    # m, the frame sits 6 cm off the floor

WHEEL_R = 0.0415     # m, 83 mm across the grips
WHEEL_W = 0.040      # m
WHEEL_X = 0.119      # m, axles 6.3 and 30.1 cm behind the nose
WHEEL_Y = 0.170      # m, half the 34 cm track
WHEEL_Z = 0.041      # m, the axle: the motors hang below the frame

# The D555 case (167 x 48 x 42 mm), front glass 0.9 cm inside the nose.
CAM_X = 0.173 - 0.024
CAM_Z = 0.175

# The RPLidar C1, on top, its front edge 2 cm behind the nose. Drawn because
# every beam on screen originates here: if the ring ever looks offset from the
# walls, the first question is whether this puck is where the TF says it is.
LIDAR_X = 0.1342     # m, the rotor centre
LIDAR_Z = 0.2498     # m, the laser plane: frame top 0.22 + 29.8 mm
LIDAR_R = 0.0278     # m, half the 55.6 mm case
LIDAR_H = 0.0413     # m, the case
LIDAR_BASE = 0.22    # m, it sits on the frame top

# Orange body against a blue-grey costmap and a magenta obstacle layer: nothing
# else on screen is this colour, which is the whole point of picking it.
BODY_RGBA = (1.00, 0.48, 0.09, 0.95)
NOSE_RGBA = (0.20, 0.90, 1.00, 1.00)   # cyan: this end is the front
WHEEL_RGBA = (0.13, 0.13, 0.16, 1.00)
CAM_RGBA = (0.85, 0.85, 0.88, 1.00)
LIDAR_RGBA = (0.95, 0.30, 0.05, 1.00)
FOOT_RGBA = (1.00, 0.48, 0.09, 0.55)   # the envelope outline, on the floor


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
                     x, y, WHEEL_Z, WHEEL_RGBA)
        # A cylinder's axis is +z; a wheel's axis is +y. Rotate 90 deg about x.
        h = math.pi / 4
        m.pose.orientation.x = math.sin(h)
        m.pose.orientation.w = math.cos(h)
        return m

    def _publish(self):
        a = MarkerArray()

        # the frame, at its OWN size. The wheels below stick out past it, so
        # the silhouette becomes the 36 x 38 envelope without any cube claiming
        # to be it.
        a.markers.append(self._mk(
            0, Marker.CUBE, BODY_L, BODY_W, BODY_H,
            BODY_X, 0.0, BODY_Z, BODY_RGBA))

        # nose wedge — which way is forward, readable from directly above
        a.markers.append(self._mk(
            1, Marker.CUBE, 0.05, BODY_W - 0.06, BODY_H + 0.01,
            FRONT - 0.025, 0.0, BODY_Z, NOSE_RGBA))

        # the D555, just inside the nose
        a.markers.append(self._mk(
            2, Marker.CUBE, 0.048, 0.167, 0.042,
            CAM_X, 0.0, CAM_Z, CAM_RGBA))

        # the RPLidar C1, on top. Every beam on screen starts at this puck.
        a.markers.append(self._mk(
            3, Marker.CYLINDER, LIDAR_R * 2, LIDAR_R * 2, LIDAR_H,
            LIDAR_X, 0.0, LIDAR_BASE + LIDAR_H / 2, LIDAR_RGBA))

        # four wheels
        for i, (x, y) in enumerate([(WHEEL_X, WHEEL_Y), (WHEEL_X, -WHEEL_Y),
                                    (-WHEEL_X, WHEEL_Y), (-WHEEL_X, -WHEEL_Y)]):
            a.markers.append(self._wheel(10 + i, x, y))

        # THE FOOTPRINT nav2 keeps clear, drawn flat on the floor. This is the
        # answer to "does it fit through there" -- not the body, which is
        # smaller, and not the wheels, which only touch the envelope at four
        # points. Same polygon as phase3/config/nav2.yaml.
        foot = self._mk(21, Marker.LINE_STRIP, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, FOOT_RGBA)
        foot.scale.x = 0.012
        corners = [(FRONT, SIDE), (FRONT, -SIDE), (REAR, -SIDE), (REAR, SIDE)]
        foot.points = [Point(x=x, y=y, z=0.005) for x, y in corners + corners[:1]]
        a.markers.append(foot)

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
