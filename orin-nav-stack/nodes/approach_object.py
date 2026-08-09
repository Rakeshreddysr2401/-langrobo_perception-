#!/usr/bin/env python3
"""Go near a named object: YOLO 3D detection -> nav2 goal (obstacle-aware, self-correcting).

The upgrade over visual_approach.py's blind 2D mono servo: we take the object's
METRIC MAP coordinate from /vision/detections_3d and send it to nav2, which plans
a path on the nvblox costmap, follows it with MPPI (correcting drift live off the
cuVSLAM-fused pose), and stops short so it does not bump the object.

    approach_object.py <label> [stop_dist_m=0.35] [timeout_s=40]

Flow: set /vision/target -> read freshest high-conf <label> in map frame -> compute
a goal `stop_dist` short of it along (robot->object), facing the object -> send
/navigate_to_pose -> report. Robot pose comes from TF map->base_link.
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

DET_TOPIC = "/vision/detections_3d"
TARGET_TOPIC = "/vision/target"
DET_MAX_AGE = 2.0        # s: ignore stale detections
MIN_CONF = 0.4


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Approacher(Node):
    def __init__(self, label, stop_dist):
        super().__init__("approach_object")
        self.label = label.lower()
        self.stop_dist = stop_dist
        self.dets = {}          # label -> (x, y, conf, stamp)
        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self)
        self.create_subscription(String, DET_TOPIC, self._on_det, qos_profile_sensor_data)
        self.target_pub = self.create_publisher(String, TARGET_TOPIC, 10)
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _on_det(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        stamp = d.get("stamp", time.time())
        for o in d.get("objects", []):
            lbl = str(o.get("label", "")).lower()
            best = self.dets.get(lbl)
            if best is None or o.get("conf", 0) >= best[2]:
                self.dets[lbl] = (o["x"], o["y"], o.get("conf", 0.0), stamp)

    def robot_xy(self):
        try:
            t = self.tf_buf.lookup_transform("map", "base_link", rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def find(self, timeout=10.0):
        self.target_pub.publish(String(data=self.label))   # wake the detector
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.target_pub.publish(String(data=self.label))
            d = self.dets.get(self.label)
            if d and (time.time() - d[3]) < DET_MAX_AGE and d[2] >= MIN_CONF and self.robot_xy():
                return d
        return None

    def compute_goal(self, obj_x, obj_y):
        rx, ry = self.robot_xy()
        dx, dy = obj_x - rx, obj_y - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return None
        ux, uy = dx / dist, dy / dist
        reach = max(0.0, dist - self.stop_dist)     # stop `stop_dist` short
        gx, gy = rx + ux * reach, ry + uy * reach
        gyaw = math.atan2(dy, dx)                    # face the object
        return gx, gy, gyaw, dist

    def send(self, gx, gy, gyaw):
        if not self.nav.wait_for_server(timeout_sec=5.0):
            print("nav2 action server not available")
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        qx, qy, qz, qw = yaw_to_quat(gyaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        fut = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh or not gh.accepted:
            print("goal rejected")
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        status = res_fut.result().status
        return status == 4   # STATUS_SUCCEEDED


# nav2 refuses to plan into unmapped (NO_INFORMATION) space (track_unknown_space:
# True), so a far object fails a single big goal. Approach in short STEPS instead:
# each goal stays inside the known map, nvblox maps further ahead as we drive, and
# re-detecting every step keeps us locked on (continuous self-correction).
STEP_MAX = 0.8       # m: max nav2 goal distance per step
STOP_TOL = 0.12      # m: "arrived" band around stop_dist
MAX_STEPS = 8


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    label = sys.argv[1]
    stop_dist = float(sys.argv[2]) if len(sys.argv) > 2 else 0.35

    rclpy.init()
    ap = Approacher(label, stop_dist)
    print(f"looking for '{label}' ...")

    for step in range(1, MAX_STEPS + 1):
        det = ap.find(timeout=8.0)
        if not det:
            print(f"  step {step}: lost sight of '{label}' — stopping.")
            break
        ox, oy, conf, _ = det
        rx, ry = ap.robot_xy()
        dist = math.hypot(ox - rx, oy - ry)
        print(f"  step {step}: {label} at ({ox:.2f},{oy:.2f}) conf {conf:.2f}, {dist:.2f} m away")
        if dist <= stop_dist + STOP_TOL:
            print(f"ARRIVED near the {label} ({dist:.2f} m).")
            break
        # short goal toward the object, capped to stay in known space
        reach = min(STEP_MAX, dist - stop_dist)
        ux, uy = (ox - rx) / dist, (oy - ry) / dist
        gx, gy = rx + ux * reach, ry + uy * reach
        gyaw = math.atan2(oy - ry, ox - rx)     # keep facing the object
        print(f"    -> step goal ({gx:.2f},{gy:.2f}) yaw {math.degrees(gyaw):.0f} (+{reach:.2f} m)")
        if not ap.send(gx, gy, gyaw):
            print("    step goal failed (planner/costmap) — retrying shorter next loop.")
        time.sleep(1.0)   # let nvblox + pose settle before re-detecting
    else:
        print(f"reached step limit approaching '{label}'.")

    ap.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
