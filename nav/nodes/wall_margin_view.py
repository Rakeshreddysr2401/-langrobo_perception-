#!/usr/bin/env python3
"""wall_margin_view.py — the costmap drawn the way the owner reads a room.

    /global_costmap/costmap  nav_msgs/OccupancyGrid (nav2, what it plans on)
        -> /rover_view/wall_margin  nav_msgs/OccupancyGrid, for RViz only:
               a wall (nav2 lethal)            98  -> RED    in RViz's costmap colours
               within MARGIN_M of a wall       55  -> VIOLET (1 cm)
               everything else                  0  -> clear: the floor shows white

WHY (owner, 2026-09-27). nav2's own costmap is drawn for the rover's CENTRE:
its cyan band says "the centre may not be here", so it is half the rover
(0.178 m) plus footprint_padding (0.05 then) = ~23 cm wide beside every wall. Parked
in a pocket with 15 cm between its wheels and the walls, the rover looked as if
it were sitting in forbidden space when it was not. The rule the planner
actually enforces is the owner's: the BODY keeps its margin (2 cm since the
same day) from a wall. This shows
exactly that -- walls, then the 5 cm margin, then free floor -- in body terms.

DISPLAY ONLY. Nothing plans on this topic, so it can never make the rover
drive closer to anything. MARGIN_M should equal nav2.yaml footprint_padding;
change both together.
"""
import math
import os

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

MARGIN_M = float(os.environ.get("ROVER_WALL_MARGIN_M", "0.01"))   # = footprint_padding (1 cm, NAV_PLAN.md N1)
WALL, BAND, FREE = 98, 55, 0
LETHAL = 100          # nav2 publishes lethal (254) as 100 in an OccupancyGrid


class WallMarginView(Node):
    def __init__(self):
        super().__init__("wall_margin_view")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(OccupancyGrid, "/rover_view/wall_margin", latched)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_costmap, latched)
        self._offsets, self._res = [], None
        self.get_logger().info(f"wall_margin_view up — walls red, {MARGIN_M * 100:.0f} cm violet margin")

    def _disk(self, res: float) -> list:
        """Cell offsets within MARGIN_M (plus half a cell, so a 5 cm margin on
        a 2.5 cm grid is two full cells, not one and a sliver)."""
        r = MARGIN_M / res + 0.5
        n = int(math.ceil(r))
        return [(dy, dx) for dy in range(-n, n + 1) for dx in range(-n, n + 1)
                if (dx or dy) and dx * dx + dy * dy <= r * r]

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        h, w = msg.info.height, msg.info.width
        if not h or not w:
            return
        if self._res != msg.info.resolution:
            self._res, self._offsets = msg.info.resolution, self._disk(msg.info.resolution)
        wall = np.asarray(msg.data, dtype=np.int8).reshape(h, w) == LETHAL
        near = np.zeros_like(wall)
        n = max((max(abs(dy), abs(dx)) for dy, dx in self._offsets), default=0)
        padded = np.pad(wall, n)
        for dy, dx in self._offsets:
            near |= padded[n + dy:n + dy + h, n + dx:n + dx + w]
        out = np.full((h, w), FREE, dtype=np.int8)
        out[near] = BAND
        out[wall] = WALL
        view = OccupancyGrid()
        view.header, view.info = msg.header, msg.info
        view.data = out.ravel().tolist()
        self._pub.publish(view)


def main():
    rclpy.init()
    node = WallMarginView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
