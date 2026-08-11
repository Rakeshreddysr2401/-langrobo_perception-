#!/usr/bin/env python3
"""
rover_marker.py — draw the rover's actual body in RViz.

WHY
    This repo has NO URDF and no robot_description, so RViz's RobotModel display
    has nothing to render. Until now the rover appeared only as a set of TF
    coordinate axes, which is why "I can't see the rover" is the first thing
    everyone says when they open the view (learn/00-setup.md).

    A URDF would be the "proper" answer, but it is a lot of XML for a two-wheeled
    box, and it needs joint states to stay honest. A marker attached to base_link
    gives the same thing you actually want — a body you can watch drive around —
    with no moving parts to drift out of sync.

WHAT IT DRAWS  (all in base_link, so they follow the pose automatically)
    body      grey box, real measured size (README §2: L 30 cm x W 17 cm)
    nose      green arrow, +x — which way is FORWARD (the #1 confusion in RViz)
    camera    blue box at the measured mount point (x 0.10, z 0.163)
    fov       translucent wedge showing the ~87 deg the camera can actually see

    The FOV wedge is deliberate: the rover is BLIND to its sides and back (no pan
    /tilt head, SYSTEM_INTEGRATION §6 G2), and that single fact explains most
    "why is the map wrong" confusion. Better to see the blindness than infer it.

USAGE
    python3 rover_marker.py                 # latched-ish, 2 Hz, /rover/model
    Add a MarkerArray display on /rover/model in RViz.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# Measured, not guessed. Keep these in step with the static TF in run_stack.sh
# and the chassis figures in README §2 — if the rover is rebuilt, fix them here.
BODY_L, BODY_W, BODY_H = 0.30, 0.17, 0.16   # chassis, metres
CAM_X, CAM_Z = 0.10, 0.163                  # base_link -> camera0_link (floor_probe)
FOV_DEG, FOV_RANGE = 87.0, 3.0              # D555 horizontal FOV, drawn 3 m deep


def _base(mid, kind, r, g, b, a):
    m = Marker()
    m.header.frame_id = "base_link"
    m.ns, m.id, m.type, m.action = "rover", mid, kind, Marker.ADD
    m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
    m.pose.orientation.w = 1.0
    return m


def build() -> MarkerArray:
    arr = MarkerArray()

    # --- chassis ------------------------------------------------------------
    # base_link's origin is ON THE FLOOR between the wheels, so the box centre
    # sits half its height up. Getting this wrong makes the rover look sunk.
    body = _base(0, Marker.CUBE, 0.35, 0.38, 0.42, 0.85)
    body.scale.x, body.scale.y, body.scale.z = BODY_L, BODY_W, BODY_H
    body.pose.position.z = BODY_H / 2.0
    arr.markers.append(body)

    # --- forward arrow ------------------------------------------------------
    nose = _base(1, Marker.ARROW, 0.20, 0.90, 0.30, 0.95)
    nose.scale.x, nose.scale.y, nose.scale.z = 0.02, 0.04, 0.05   # shaft/head
    nose.points = [Point(x=0.0, y=0.0, z=BODY_H + 0.02),
                   Point(x=BODY_L / 2 + 0.10, y=0.0, z=BODY_H + 0.02)]
    arr.markers.append(nose)

    # --- camera -------------------------------------------------------------
    cam = _base(2, Marker.CUBE, 0.20, 0.55, 0.95, 0.95)
    cam.scale.x, cam.scale.y, cam.scale.z = 0.03, 0.09, 0.03
    cam.pose.position.x, cam.pose.position.z = CAM_X, CAM_Z
    arr.markers.append(cam)

    # --- field of view ------------------------------------------------------
    # A flat triangle fan at camera height. This is the honest picture of what
    # the robot can see: everything outside this wedge is unknown to it.
    fov = _base(3, Marker.TRIANGLE_LIST, 0.20, 0.55, 0.95, 0.13)
    fov.scale.x = fov.scale.y = fov.scale.z = 1.0
    half = math.radians(FOV_DEG / 2.0)
    apex = Point(x=CAM_X, y=0.0, z=CAM_Z)
    steps = 12
    for i in range(steps):
        a0 = -half + (2 * half) * (i / steps)
        a1 = -half + (2 * half) * ((i + 1) / steps)
        fov.points.append(apex)
        fov.points.append(Point(x=CAM_X + FOV_RANGE * math.cos(a0),
                                y=FOV_RANGE * math.sin(a0), z=CAM_Z))
        fov.points.append(Point(x=CAM_X + FOV_RANGE * math.cos(a1),
                                y=FOV_RANGE * math.sin(a1), z=CAM_Z))
    arr.markers.append(fov)

    return arr


class RoverMarker(Node):
    def __init__(self):
        super().__init__("rover_marker")
        # TRANSIENT_LOCAL so an RViz started later still gets the geometry
        # without waiting for the next tick.
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(MarkerArray, "/rover/model", qos)
        self.arr = build()
        self.tick()                       # publish immediately, don't wait 0.5 s
        self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f"rover body on /rover/model — {BODY_L*100:.0f}x{BODY_W*100:.0f} cm, "
            f"camera at x={CAM_X} z={CAM_Z}, FOV {FOV_DEG:.0f} deg")

    def tick(self):
        now = self.get_clock().now().to_msg()
        for m in self.arr.markers:
            m.header.stamp = now
        self.pub.publish(self.arr)


def main():
    rclpy.init()
    node = RoverMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
