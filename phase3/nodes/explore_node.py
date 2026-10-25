#!/usr/bin/env python3
"""explore_node.py — map the home by itself: drive to the edge of what is known, repeat.

    ./rover explore [max_minutes]          THIS DRIVES THE ROVER (a person watches: no drop-off sensing)

FRONTIER EXPLORATION on slam_toolbox's /map (the LiDAR map, map frame):
  1. frontier cells = FREE cells next to UNKNOWN ones; grouped into connected clusters,
     clusters under MIN_FRONTIER_M of edge are noise and skipped
  2. a goal per cluster = the cluster's free cell nearest its centre, pulled back
     STANDOFF_M into known free space (the goal is never inside the unknown), facing the unknown
  3. the nearest goal not yet failed or visited is sent to reach_node (/reach/goal), the
     rover's own goal driver: nav2 for the route, goal_exec for the finish, "too tight"
     refusals and recoveries all as for any RViz or Telegram goal
  4. on reach's result: reached -> visited; failed -> blacklisted (that spot, BLACKLIST_M round)
  5. done when no frontier is left, or after max_minutes, or MAX_FAILS failures in a row

It never commands the wheels itself, so stopping it (Ctrl-C / ./rover stop / MANUAL on
the phone) leaves reach's own stop behaviour. /explore/status (JSON) says what it does.
Plan: ISAAC_ROS_5_UPGRADE.md "explore".
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros

MIN_FRONTIER_M = 0.30     # a cluster with less edge than this is noise (a chair leg's shadow)
STANDOFF_M = 0.25         # goal this far back from the frontier, inside known free space
CLEAR_M = 0.30            # goal cell must have no wall/unknown within this (the body is 38 x 36 cm)
MIN_GOAL_M = 0.50         # frontiers nearer than this are "here": they clear as the rover looks
BLACKLIST_M = 0.50        # a failed goal blocks this radius
GOAL_TIMEOUT_S = 180.0    # reach gets this long per goal
MAX_FAILS = 4             # this many failed goals in a row -> stop and say so


def frontier_goals(grid, robot_xy, blocked):
    """OccupancyGrid -> [(dist, x, y, yaw, size_m)] goal candidates, nearest first."""
    from scipy import ndimage
    w, h, res = grid.info.width, grid.info.height, grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    g = np.array(grid.data, dtype=np.int16).reshape(h, w)
    free, unknown, occ = (g >= 0) & (g < 25), g < 0, g >= 65
    nb_unknown = ndimage.binary_dilation(unknown, structure=np.ones((3, 3)))
    frontier = free & nb_unknown
    # a goal cell: free, with neither wall nor unknown within CLEAR_M
    r = max(1, int(round(CLEAR_M / res)))
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    disk = xx * xx + yy * yy <= r * r
    safe = free & ~ndimage.binary_dilation(occ | unknown, structure=disk)
    labels, n = ndimage.label(frontier, structure=np.ones((3, 3)))
    out = []
    for k in range(1, n + 1):
        vs, us = np.nonzero(labels == k)
        size = len(vs) * res
        if size < MIN_FRONTIER_M:
            continue
        cu, cv = us.mean(), vs.mean()
        # pull back into known free space: the safe cell nearest the cluster centre, at least
        # STANDOFF_M from it, so the camera and LiDAR see past the frontier from there
        sv, su = np.nonzero(safe)
        if not len(sv):
            break
        d = np.hypot(su - cu, sv - cv) * res
        ok = d >= STANDOFF_M
        if not ok.any():
            continue
        i = np.argmin(np.where(ok, d, np.inf))
        gx, gy = ox + (su[i] + 0.5) * res, oy + (sv[i] + 0.5) * res
        if any(math.hypot(gx - bx, gy - by) < BLACKLIST_M for bx, by in blocked):
            continue
        dist = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
        if dist < MIN_GOAL_M:
            continue
        fx, fy = ox + (cu + 0.5) * res, oy + (cv + 0.5) * res
        out.append((dist, gx, gy, math.atan2(fy - gy, fx - gx), size))
    return sorted(out)


class Explore(Node):
    def __init__(self, max_min):
        super().__init__('explore')
        self.max_s = max_min * 60.0
        self.grid = None
        self.result = None
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'grid', m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(String, '/reach/status', self._reach, 20)
        self.goal_pub = self.create_publisher(PoseStamped, '/reach/goal', 10)
        self.status_pub = self.create_publisher(String, '/explore/status', 10)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.goal_stamp = None

    def _reach(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        if 'result' in d and self.goal_stamp and d.get('goal_stamp') == self.goal_stamp:
            self.result = d

    def say(self, **kw):
        kw['t'] = round(time.time(), 1)
        self.status_pub.publish(String(data=json.dumps(kw)))
        self.get_logger().info(json.dumps(kw))

    def pose(self):
        try:
            t = self.buf.lookup_transform('map', 'base_link', Time()).transform
        except Exception:  # noqa: BLE001 -- not yet
            return None
        return t.translation.x, t.translation.y

    def wait(self, secs, until=lambda: False):
        end = time.time() + secs
        while time.time() < end and not until():
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        t0 = time.time()
        self.wait(20, lambda: self.grid is not None and self.pose() is not None)
        if self.grid is None or self.pose() is None:
            self.say(phase='done', why='no /map or no map -> base_link: is ./rover slam up?')
            return 1
        blocked, fails, n_goals = [], 0, 0
        while time.time() - t0 < self.max_s:
            self.wait(1.0)                                   # a fresh /map after the last move
            here = self.pose()
            cands = frontier_goals(self.grid, here, blocked)
            if not cands:
                self.say(phase='done', why='no frontier left: the reachable home is mapped',
                         goals=n_goals, minutes=round((time.time() - t0) / 60, 1))
                return 0
            dist, gx, gy, yaw, size = cands[0]
            g = PoseStamped()
            g.header.frame_id = 'map'
            g.header.stamp = self.get_clock().now().to_msg()
            g.pose.position.x, g.pose.position.y = gx, gy
            g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.goal_stamp = f'{g.header.stamp.sec}.{g.header.stamp.nanosec:09d}'
            self.result = None
            n_goals += 1
            self.say(phase='goal', n=n_goals, x=round(gx, 2), y=round(gy, 2), dist_m=round(dist, 2),
                     frontier_m=round(size, 2), frontiers=len(cands))
            self.goal_pub.publish(g)
            self.wait(GOAL_TIMEOUT_S, lambda: self.result is not None)
            r = (self.result or {}).get('result', 'timeout')
            if r == 'reached':
                fails = 0
                blocked.append((gx, gy))                     # been there: its frontier is seen
            else:
                fails += 1
                blocked.append((gx, gy))
                self.say(phase='failed', n=n_goals, result=r, why=(self.result or {}).get('why', ''))
                if fails >= MAX_FAILS:
                    self.say(phase='done', why=f'{fails} goals in a row failed: the rest is out of reach',
                             goals=n_goals)
                    return 2
        self.say(phase='done', why='time limit', goals=n_goals)
        return 0


def main():
    rclpy.init()
    n = Explore(float(sys.argv[1]) if len(sys.argv) > 1 else 20.0)
    try:
        rc = n.run()
    except KeyboardInterrupt:
        rc = 130
    n.destroy_node()
    rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
