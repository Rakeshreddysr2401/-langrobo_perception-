#!/usr/bin/env python3
"""blocker_view.py — WHO says a cell is blocked, and WHAT the rover is aiming at.

    /local_costmap/lidar_layer    nav_msgs/OccupancyGrid  (nav2's LiDAR layer, near the rover)
    /local_costmap/nvblox_layer   nav_msgs/OccupancyGrid  (nav2's camera layer, near the rover)
    /global_costmap/{lidar,nvblox}_layer                  (the same for the 8 x 8 m room view)
    /vision/pixel_result          std_msgs/String JSON    (pixel_to_goal's answer)
        -> /rover_view/blockers       visualization_msgs/MarkerArray  (near, ~1.7 Hz)
        -> /rover_view/blockers_room  visualization_msgs/MarkerArray  (room, ~0.8 Hz)
        -> /rover_view/target         visualization_msgs/MarkerArray  (latched)

    RED     only the LiDAR says blocked (a plane at 25 cm: walls, legs, people)
    ORANGE  only the camera says blocked (low things under the LiDAR -- and
            phantoms: reflections on the glossy floor, a tilted camera)
    PURPLE  both agree
    GREEN   the object the brain picked (sphere) and the goal pose (arrow),
            labelled "<what>: <source> <depth> m"

WHY (owner, 2026-10-03): "a good RViz that helps us see easily which objects it
considers a blocker, and their boundaries". nav2's costmap shows the per-cell
MAXIMUM of its layers, so red on empty floor could be the camera, the LiDAR or
both -- that night it was the camera alone (a mount tipped 2.6 deg; ghost_check:
1913 camera-only cells, 0 LiDAR). The colours answer that at a glance.

DISPLAY ONLY. Nothing plans on these topics. Run: python3 blocker_view.py
"""
import json

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

LETHAL = 100                                      # nav2 publishes lethal (254) as 100
RED = ColorRGBA(r=0.95, g=0.15, b=0.15, a=0.9)
ORANGE = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9)
PURPLE = ColorRGBA(r=0.65, g=0.2, b=0.9, a=0.9)
GREEN = ColorRGBA(r=0.1, g=0.9, b=0.3, a=0.95)
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def classify(lidar: np.ndarray, camera: np.ndarray) -> dict:
    """Lethal cells by source: {"lidar", "camera", "both"} -> flat indices. Pure."""
    li, ca = lidar >= LETHAL, camera >= LETHAL
    return {"lidar": np.flatnonzero(li & ~ca), "camera": np.flatnonzero(ca & ~li),
            "both": np.flatnonzero(li & ca)}


def crop_to(grid: np.ndarray, grid_origin: tuple, origin: tuple, size: tuple, res: float) -> np.ndarray:
    """The (h, w) window of grid (rows = y) whose origin is `origin`, by position;
    cells outside grid are 0. Same resolution. Pure."""
    w, h = size
    dx = int(round((origin[0] - grid_origin[0]) / res))
    dy = int(round((origin[1] - grid_origin[1]) / res))
    out = np.zeros((h, w), dtype=grid.dtype)
    gy0, gx0 = max(0, dy), max(0, dx)
    gy1, gx1 = min(grid.shape[0], dy + h), min(grid.shape[1], dx + w)
    if gy1 > gy0 and gx1 > gx0:
        out[gy0 - dy:gy1 - dy, gx0 - dx:gx1 - dx] = grid[gy0:gy1, gx0:gx1]
    return out


class BlockerView(Node):
    def __init__(self):
        super().__init__("blocker_view")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._grids = {}
        self._pubs = {
            "local": self.create_publisher(MarkerArray, "/rover_view/blockers", latched),
            "global": self.create_publisher(MarkerArray, "/rover_view/blockers_room", latched),
        }
        for scope in ("local", "global"):
            for layer in ("lidar", "nvblox"):
                self.create_subscription(
                    OccupancyGrid, f"/{scope}_costmap/{layer}_layer",
                    lambda m, s=scope, l=layer: self._on_grid(s, l, m), latched)
        self._target_pub = self.create_publisher(MarkerArray, "/rover_view/target", latched)
        self.create_subscription(String, "/vision/pixel_result", self._on_result, 10)
        self.get_logger().info("blocker_view up: RED LiDAR, ORANGE camera only, PURPLE both, GREEN target")

    def _on_grid(self, scope: str, layer: str, msg: OccupancyGrid) -> None:
        self._grids[(scope, layer)] = msg
        other = self._grids.get((scope, "nvblox" if layer == "lidar" else "lidar"))
        if other is None:
            return
        li, ca = self._grids[(scope, "lidar")], self._grids[(scope, "nvblox")]
        if abs(li.info.resolution - ca.info.resolution) > 1e-6:
            return
        w, h, res = li.info.width, li.info.height, li.info.resolution
        ox, oy = li.info.origin.position.x, li.info.origin.position.y
        # the camera layer covers a larger window (measured: 424 x 424 vs the
        # costmap's 160 x 160): cut out the costmap's window, by position
        cam = crop_to(np.asarray(ca.data, dtype=np.int16).reshape(ca.info.height, ca.info.width),
                      (ca.info.origin.position.x, ca.info.origin.position.y), (ox, oy), (w, h), res)
        groups = classify(np.asarray(li.data, dtype=np.int16), cam.ravel())
        out = MarkerArray()
        for i, (name, color) in enumerate((("lidar", RED), ("camera", ORANGE), ("both", PURPLE))):
            m = Marker()
            m.header = li.header
            m.ns, m.id, m.type = f"blocked_{name}", i, Marker.CUBE_LIST
            m.action = Marker.ADD if len(groups[name]) else Marker.DELETE
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = res
            m.scale.z = 0.02
            m.color = color
            idx = groups[name]
            xs = ox + (idx % w + 0.5) * res
            ys = oy + (idx // w + 0.5) * res
            m.points = [Point(x=float(x), y=float(y), z=0.01) for x, y in zip(xs, ys)]
            out.markers.append(m)
        legend = Marker()
        legend.header.frame_id = "base_link"
        legend.header.stamp = li.header.stamp
        legend.ns, legend.id, legend.type = "legend", 9, Marker.TEXT_VIEW_FACING
        legend.pose.position.x, legend.pose.position.z = -0.45, 0.35
        legend.pose.orientation.w = 1.0
        legend.scale.z = 0.06
        legend.color = WHITE
        legend.text = (f"blocked: RED lidar {len(groups['lidar'])} | ORANGE camera {len(groups['camera'])}"
                       f" | PURPLE both {len(groups['both'])}")
        if scope == "local":
            out.markers.append(legend)
        self._pubs[scope].publish(out)

    def _on_result(self, msg: String) -> None:
        try:
            r = json.loads(msg.data)
        except ValueError:
            return
        if not r.get("ok") or "object" not in r:
            return
        out = MarkerArray()
        obj = Marker()
        obj.header.frame_id = "odom"
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.ns, obj.id, obj.type = "target", 0, Marker.SPHERE
        obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = r["object"]["x"], r["object"]["y"], 0.15
        obj.pose.orientation.w = 1.0
        obj.scale.x = obj.scale.y = obj.scale.z = 0.12
        obj.color = GREEN
        goal = Marker()
        goal.header = obj.header
        goal.ns, goal.id, goal.type = "target", 1, Marker.ARROW
        g = r["goal"]
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = g["x"], g["y"], 0.05
        goal.pose.orientation.z, goal.pose.orientation.w = float(np.sin(g["yaw"] / 2)), float(np.cos(g["yaw"] / 2))
        goal.scale.x, goal.scale.y, goal.scale.z = 0.3, 0.05, 0.05
        goal.color = GREEN
        label = Marker()
        label.header = obj.header
        label.ns, label.id, label.type = "target", 2, Marker.TEXT_VIEW_FACING
        label.pose.position.x, label.pose.position.y, label.pose.position.z = r["object"]["x"], r["object"]["y"], 0.4
        label.pose.orientation.w = 1.0
        label.scale.z = 0.08
        label.color = GREEN
        label.text = f'{r.get("what") or "target"}: {r.get("source", "camera")} {r.get("depth_m", "?")} m'
        out.markers = [obj, goal, label]
        self._target_pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(BlockerView())


if __name__ == "__main__":
    main()
