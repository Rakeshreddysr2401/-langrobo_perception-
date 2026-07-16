#!/usr/bin/env python3
"""LangRobo Pi5 client — the single integration point between the VLM brain
and the Jetson perception/navigation stack.

Runs natively on the Pi5 (ROS 2 Jazzy, same WiFi as the Jetson, plain
multicast discovery — do NOT set ROS_DISCOVERY_SERVER).

The "go near the chair" flow this implements:
  1. look()           grab the newest camera frame (2 Hz JPEG feed from Jetson)
  2.   → VLM finds the chair in the image, returns a pixel (u, v)
  3. ground_pixel()   Jetson deprojects the pixel with real depth → map goal
  4. go_to()          send the goal to Nav2 on the Jetson (action, with result)
Steps 3+4 combined: go_near_pixel(u, v).

Known-object shortcut (YOLO already runs on the Jetson):
  objects()           latest map-frame detections
  go_near_object("chair")   navigate to a YOLO detection by label

Library use from your VLM script:
    from langrobo_client import LangRoboClient
    bot = LangRoboClient()
    path = bot.look("/tmp/frame.jpg")        # → give to VLM
    res = bot.ground_pixel(448, 252)         # → {"ok": True, "goal": {...}}
    bot.go_to(**res["goal"])                 # blocks until arrival

CLI:
    ./langrobo_client.py look out.jpg
    ./langrobo_client.py objects
    ./langrobo_client.py ground 448 252
    ./langrobo_client.py go 1.2 0.3 [yaw]
    ./langrobo_client.py go-pixel 448 252
    ./langrobo_client.py go-object chair
    ./langrobo_client.py cancel
    ./langrobo_client.py status              # stack health seen from the Pi5
"""

import json
import math
import sys
import time
import uuid

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

JPEG_TOPIC = "/camera/color/image_raw/compressed"
DETECTIONS_TOPIC = "/vision/detections_3d"
QUERY_TOPIC = "/vision/pixel_query"
RESULT_TOPIC = "/vision/pixel_result"


class LangRoboClient(Node):
    def __init__(self, node_name: str = "langrobo_pi5_client"):
        rclpy.init(args=None) if not rclpy.ok() else None
        super().__init__(node_name)
        self._jpeg = None
        self._detections = None
        self._results = {}  # request id → result dict

        self.create_subscription(CompressedImage, JPEG_TOPIC,
                                 self._on_jpeg, qos_profile_sensor_data)
        self.create_subscription(String, DETECTIONS_TOPIC,
                                 self._on_detections, 10)
        self.create_subscription(String, RESULT_TOPIC, self._on_result, 10)
        self._query_pub = self.create_publisher(PointStamped, QUERY_TOPIC, 10)
        self._nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._goal_handle = None

    # -- callbacks -----------------------------------------------------------
    def _on_jpeg(self, msg): self._jpeg = msg
    def _on_detections(self, msg): self._detections = msg.data

    def _on_result(self, msg):
        try:
            r = json.loads(msg.data)
            self._results[r.get("id", "")] = r
        except json.JSONDecodeError:
            pass

    def _spin_until(self, cond, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if cond():
                return True
        return False

    # -- API -----------------------------------------------------------------
    def look(self, path: str = "/tmp/langrobo_look.jpg",
             timeout: float = 3.0) -> str:
        """Save the newest camera frame as JPEG; returns the path."""
        self._jpeg = None  # force a FRESH frame, not a stale one
        if not self._spin_until(lambda: self._jpeg is not None, timeout):
            raise TimeoutError(f"no frame on {JPEG_TOPIC} within {timeout}s — "
                               "is run_vision_ai.sh up on the Jetson?")
        with open(path, "wb") as f:
            f.write(bytes(self._jpeg.data))
        return path

    def objects(self, timeout: float = 3.0) -> list:
        """Latest YOLO detections in the map frame:
        [{"label": "chair", "x":.., "y":.., "z":.., "conf":..}, ...]"""
        if not self._spin_until(lambda: self._detections is not None, timeout):
            raise TimeoutError(f"no detections on {DETECTIONS_TOPIC}")
        return json.loads(self._detections).get("objects", [])

    def ground_pixel(self, u: float, v: float, timeout: float = 3.0) -> dict:
        """Pixel (u, v) in the color image → map-frame goal, using the
        Jetson's real depth. Returns the full result dict; check ["ok"]."""
        # A fresh node must wait for DDS to match the Jetson's subscriber,
        # or the query publish is silently lost (fire into the void).
        if not self._spin_until(
                lambda: self._query_pub.get_subscription_count() > 0, 5.0):
            return {"ok": False, "reason": "jetson_pixel_to_goal_not_running"}
        req_id = uuid.uuid4().hex[:8]
        msg = PointStamped()
        msg.header.frame_id = req_id
        msg.point.x, msg.point.y = float(u), float(v)
        self._query_pub.publish(msg)
        if not self._spin_until(lambda: req_id in self._results, timeout):
            return {"id": req_id, "ok": False, "reason": "no_reply_from_jetson"}
        return self._results.pop(req_id)

    def go_to(self, x: float, y: float, yaw: float = 0.0,
              wait: bool = True, timeout: float = 180.0) -> dict:
        """Send a map-frame goal to Nav2 on the Jetson.
        wait=True blocks until the rover arrives (or Nav2 gives up)."""
        if not self._nav.wait_for_server(timeout_sec=5.0):
            return {"ok": False, "reason": "nav2_action_server_unreachable"}
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send = self._nav.send_goal_async(goal)
        if not self._spin_until(lambda: send.done(), 10.0):
            return {"ok": False, "reason": "goal_send_timeout"}
        self._goal_handle = send.result()
        if not self._goal_handle.accepted:
            return {"ok": False, "reason": "goal_rejected_by_nav2"}
        if not wait:
            return {"ok": True, "state": "accepted"}

        result_f = self._goal_handle.get_result_async()
        if not self._spin_until(lambda: result_f.done(), timeout):
            return {"ok": False, "reason": "navigation_timeout"}
        status = result_f.result().status  # 4 = SUCCEEDED
        return {"ok": status == 4, "state": "arrived" if status == 4
                else f"nav2_status_{status}"}

    def go_near_pixel(self, u: float, v: float, wait: bool = True) -> dict:
        """The full VLM flow: ground the pixel, then navigate to it."""
        res = self.ground_pixel(u, v)
        if not res.get("ok"):
            return res
        g = res["goal"]
        nav = self.go_to(g["x"], g["y"], g["yaw"], wait=wait)
        return {**res, **nav}

    def go_near_object(self, label: str, wait: bool = True) -> dict:
        """Navigate near the highest-confidence YOLO detection of `label`.
        The object's map point is sent as the goal position; Nav2's own
        footprint/costmap keeps the rover from driving into it."""
        matches = [o for o in self.objects() if o["label"] == label]
        if not matches:
            return {"ok": False, "reason": f"no_{label}_detected"}
        best = max(matches, key=lambda o: o["conf"])
        return {"object": best,
                **self.go_to(best["x"], best["y"], wait=wait)}

    def cancel(self) -> dict:
        """Cancel the current navigation goal."""
        if self._goal_handle is None:
            return {"ok": False, "reason": "no_active_goal"}
        f = self._goal_handle.cancel_goal_async()
        self._spin_until(lambda: f.done(), 5.0)
        self._goal_handle = None
        return {"ok": True, "state": "cancelled"}

    def status(self) -> dict:
        """What the Pi5 can see of the Jetson stack right now."""
        topics = dict(self.get_topic_names_and_types())
        return {
            "camera_feed": JPEG_TOPIC in topics,
            "detections": DETECTIONS_TOPIC in topics,
            "pixel_grounding": RESULT_TOPIC in topics,
            "nav2": "/navigate_to_pose/_action/status" in topics
                    or any(t.startswith("/navigate_to_pose") for t in topics),
            "slam_odometry": "/visual_slam/tracking/odometry" in topics,
        }


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        sys.exit(1)
    cmd, rest = args[0], args[1:]
    bot = LangRoboClient()
    try:
        if cmd == "look":
            print(bot.look(rest[0] if rest else "/tmp/langrobo_look.jpg"))
        elif cmd == "objects":
            print(json.dumps(bot.objects(), indent=2))
        elif cmd == "ground":
            print(json.dumps(bot.ground_pixel(float(rest[0]), float(rest[1]))))
        elif cmd == "go":
            yaw = float(rest[2]) if len(rest) > 2 else 0.0
            print(json.dumps(bot.go_to(float(rest[0]), float(rest[1]), yaw)))
        elif cmd == "go-pixel":
            print(json.dumps(bot.go_near_pixel(float(rest[0]), float(rest[1]))))
        elif cmd == "go-object":
            print(json.dumps(bot.go_near_object(rest[0])))
        elif cmd == "cancel":
            print(json.dumps(bot.cancel()))
        elif cmd == "status":
            # give discovery a moment to see the Jetson's publishers
            time.sleep(2.0)
            rclpy.spin_once(bot, timeout_sec=0.5)
            print(json.dumps(bot.status(), indent=2))
        else:
            print(f"unknown command: {cmd}")
            sys.exit(1)
    finally:
        bot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
