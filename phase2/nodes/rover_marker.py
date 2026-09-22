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
    chassis    36 x 28 cm, the real body
    wheels     at their real axles, so they stick out past the chassis and the
               silhouette BECOMES the envelope instead of a cube asserting it
    footprint  a flat outline of the 46 x 42 cm polygon nav2 keeps clear. This
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

# THE OUTER ENVELOPE — deliberately the same numbers as phase3/config/nav2.yaml's
# footprint. If these two ever disagree the picture is lying about the thing the
# planner believes, which is the one job this node has.
#
# As described by the owner 2026-09-22: body 36 x 28 cm, tyres 5 cm past each
# end and 7 cm past each side => 46 x 42 cm overall. NOT TAPE-MEASURED YET.
# The previous values here (35 x 38) put part of the tyres OUTSIDE the drawn
# body, so the picture was smaller than the real rover in the exact direction
# that matters for "does it fit". If a doorway that used to pass starts
# failing, measure tyre-to-tyre and nose-to-tail and fix BOTH files.
FRONT = 0.230        # m, outer edge of the front tyres
REAR = -0.230        # m, outer edge of the rear tyres
SIDE = 0.210         # m, outer face of the side tyres

# THE BODY ITSELF, which is smaller than the envelope. Drawing the body at its
# real size and the wheels at their real positions means the silhouette comes
# out as 46 x 42 on its own, instead of one cube asserting it.
BODY_L = 0.36        # m, nose to tail, chassis only
BODY_W = 0.28        # m, across the chassis, tyres excluded
BODY_H = 0.10        # m, visual only
BODY_Z = 0.105       # m, sits above the wheels

WHEEL_R = 0.0425     # m, 85 mm tyre OD confirmed with a tape
WHEEL_W = 0.035      # m, visual only
WHEEL_X = FRONT - WHEEL_R      # m, axle, so the tyre's outer edge lands on FRONT
WHEEL_Y = SIDE - WHEEL_W / 2   # m, so the tyre's outer face lands on SIDE

# The RPLidar C1, on top, 3.5 cm behind the camera lens. Drawn because every
# beam on screen originates here: if the ring ever looks offset from the walls,
# the first question is whether this puck is where the TF says it is.
LIDAR_X = 0.135
LIDAR_Z = 0.21
LIDAR_R = 0.038
LIDAR_H = 0.04

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
                     x, y, WHEEL_R, WHEEL_RGBA)
        # A cylinder's axis is +z; a wheel's axis is +y. Rotate 90 deg about x.
        h = math.pi / 4
        m.pose.orientation.x = math.sin(h)
        m.pose.orientation.w = math.cos(h)
        return m

    def _publish(self):
        a = MarkerArray()

        # the chassis, at its OWN size. The wheels below stick out past it, so
        # the silhouette becomes the 46 x 42 envelope without any cube claiming
        # to be it.
        a.markers.append(self._mk(
            0, Marker.CUBE, BODY_L, BODY_W, BODY_H,
            0.0, 0.0, BODY_Z, BODY_RGBA))

        # nose wedge — which way is forward, readable from directly above
        a.markers.append(self._mk(
            1, Marker.CUBE, 0.05, BODY_W - 0.06, BODY_H + 0.01,
            BODY_L / 2 - 0.025, 0.0, BODY_Z, NOSE_RGBA))

        # the D555, sitting on the front face
        a.markers.append(self._mk(
            2, Marker.CUBE, 0.03, 0.13, 0.04,
            BODY_L / 2 - 0.015, 0.0, BODY_Z + BODY_H / 2 + 0.02, CAM_RGBA))

        # the RPLidar C1, on top. Every beam on screen starts at this puck.
        a.markers.append(self._mk(
            3, Marker.CYLINDER, LIDAR_R * 2, LIDAR_R * 2, LIDAR_H,
            LIDAR_X, 0.0, LIDAR_Z, LIDAR_RGBA))

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
