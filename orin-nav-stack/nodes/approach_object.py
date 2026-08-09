#!/usr/bin/env python3
"""Go near a named object: YOLO 3D detection -> ONE continuously-updated nav2 goal.

    approach_object.py <label> [stop_dist_m=0.35] [timeout_s=90]

We take the object's metric MAP coordinate from /vision/detections_3d and send a
goal `stop_dist` short of it, facing it. nav2 plans on the nvblox costmap and
follows with MPPI, correcting drift live off the cuVSLAM+EKF fused pose.

WHY ONE GOAL, NOT STEPS (rewritten 2026-08-10)
----------------------------------------------
The previous version issued up to 8 short (<=0.8 m) goals with a 1 s sleep between
them. That made the rover stop, rotate to satisfy the per-step yaw tolerance,
pause, re-detect, and rotate again — jerky by construction, and every fresh noisy
detection changed the commanded heading, so it visibly hunted left/right instead
of driving to the bottle.

Its stated justification was that nav2 "refuses to plan into unmapped space
because track_unknown_space: True". That was only half right: planner_server has
allow_unknown: true (config/nav2.yaml), so NavFn does plan through unknown cells.
Stepping is therefore kept only as a FALLBACK for when a full-distance goal is
genuinely rejected or aborted — not as the normal path.

Now: send one goal to the standoff pose and let MPPI drive it continuously. Keep
detecting while it runs, and re-issue the goal ONLY when the object's smoothed
position has actually moved more than GOAL_UPDATE_M. Detection smoothing lives in
detections_3d.py (EMA + jump reset), so small per-frame jitter no longer moves the
goal at all.

Also reports /safety/state, because the single most confusing failure mode is
safety_guard zeroing vx while letting wz through: nav2 keeps commanding
forward+turn, only the turn reaches the wheels, and the rover appears to spin
aimlessly with no error anywhere. If you see FWD_BLOCKED in this output, that is
what is happening.
"""
import json
import math
import sys
import threading
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
SAFETY_TOPIC = "/safety/state"
DET_MAX_AGE = 3.0        # s: ignore stale detections
MIN_CONF = 0.30

STOP_TOL = 0.12          # m: "arrived" band around stop_dist
GOAL_UPDATE_M = 0.30     # m: re-issue the goal only if the target moved this far
FALLBACK_STEP = 0.8      # m: shortened goal distance after a rejection/abort

# YOLOv8n label confusions, measured on THIS rig (2026-08-10).
#
# The green plastic bottle scores 'bottle' 0.70 at 2.3 m, but from ~1.4 m the very
# same box comes back 'vase' 0.40 / 'bottle' 0.15 — so a bottle approach lost its
# target precisely as it closed in, which read as the robot giving up near the
# goal. yolov8n is a 3.2M-param model; this class confusion is inherent to it, not
# a bug to tune away. Accept the confusable set as one target instead.
# (Upgrading to yolov8s would help, but the Orin is already perception-bound.)
ALIASES = {
    "bottle": ("bottle", "vase"),
    "vase": ("vase", "bottle"),
    "cup": ("cup", "bowl", "wine glass"),
    "couch": ("couch", "bed", "bench", "chair"),
    "chair": ("chair", "bench", "couch"),
}


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Approacher(Node):
    def __init__(self, label, stop_dist):
        super().__init__("approach_object")
        self.label = label.lower()
        self.stop_dist = stop_dist
        self.dets = {}            # label -> (x, y, conf, stamp)
        self.safety = "unknown"
        self.tf_buf = Buffer()
        # No spin_thread: main() already spins this node on its own executor, so a
        # second executor is redundant and threw ExternalShutdownException tracebacks
        # on exit.
        TransformListener(self.tf_buf, self)
        self.create_subscription(String, DET_TOPIC, self._on_det, qos_profile_sensor_data)
        self.create_subscription(String, SAFETY_TOPIC, self._on_safety, 10)
        self.health = None
        self.create_subscription(String, "/odom/health", self._on_health, 10)
        self.target_pub = self.create_publisher(String, TARGET_TOPIC, 10)
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        # Keep the detector awake: it gates YOLO on demand (detections_3d._demand).
        self.create_timer(0.5, lambda: self.target_pub.publish(String(data=self.label)))

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

    def _on_safety(self, msg):
        self.safety = msg.data

    def _on_health(self, msg):
        try:
            self.health = json.loads(msg.data)
        except Exception:
            pass

    def pose_warning(self):
        """Non-empty when the pose backing every distance below is untrustworthy."""
        h = self.health
        if h is None or h.get("trust", False):
            return ""
        return f"{h.get('status')}: {h.get('detail')}"

    def robot_xy(self):
        try:
            t = self.tf_buf.lookup_transform("map", "base_link", rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def target(self):
        """Freshest confident detection of our label (or a known alias), or None."""
        best = None
        for lbl in ALIASES.get(self.label, (self.label,)):
            d = self.dets.get(lbl)
            if d and (time.time() - d[3]) < DET_MAX_AGE and d[2] >= MIN_CONF:
                if best is None or d[2] > best[2]:
                    best = d
        return best

    def wait_for_target(self, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.target() and self.robot_xy():
                return self.target()
            time.sleep(0.1)
        return None

    def goal_for(self, ox, oy, cap=None):
        """Standoff pose `stop_dist` short of (ox,oy), facing it. cap limits reach."""
        rxy = self.robot_xy()
        if rxy is None:
            return None
        rx, ry = rxy
        dx, dy = ox - rx, oy - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return None
        reach = max(0.0, dist - self.stop_dist)
        if cap is not None:
            reach = min(reach, cap)
        ux, uy = dx / dist, dy / dist
        return (rx + ux * reach, ry + uy * reach, math.atan2(dy, dx), dist)

    # ---- async goal handling (the executor spins in a background thread) ----
    def send_goal(self, gx, gy, gyaw, timeout=5.0):
        if not self.nav.wait_for_server(timeout_sec=timeout):
            print("  nav2 action server not available")
            return None
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        # Stamp 0 = "use the latest available transform" (2026-08-10). With
        # now(), planner_server failed EVERY plan with
        #   'GridBased plugin failed to plan ...: "Unable to transform poses to
        #    global frame"'
        # and nav2 aborted the goal ~10 s in. The goal is in `map` while the
        # costmaps are in `odom`, so the planner must apply map->odom AT THE GOAL
        # STAMP — but /tf arrives with gaps up to 0.37 s, so a goal stamped "now"
        # is repeatedly in TF's future. Zero sidesteps the race entirely; the goal
        # stays anchored in map either way.
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        qx, qy, qz, qw = yaw_to_quat(gyaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        fut = self.nav.send_goal_async(goal)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        if not fut.done():
            print("  goal send timed out")
            return None
        gh = fut.result()
        if not gh or not gh.accepted:
            print("  goal REJECTED by nav2 (planner could not accept it)")
            return None
        return gh


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    label = sys.argv[1]
    stop_dist = float(sys.argv[2]) if len(sys.argv) > 2 else 0.35
    timeout_s = float(sys.argv[3]) if len(sys.argv) > 3 else 90.0

    rclpy.init()
    ap = Approacher(label, stop_dist)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(ap)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    rc = 1
    try:
        print(f"looking for '{label}' ...")
        det = ap.wait_for_target(timeout=15.0)
        if not det:
            print(f"never saw '{label}' (or no map->base_link TF) — nothing to approach.")
            return
        ox, oy = det[0], det[1]
        print(f"found {label} at ({ox:.2f},{oy:.2f}) conf {det[2]:.2f}")

        def report_arrival(d):
            """Print arrival; return exit code (0 = verified, 2 = pose untrusted).

            Every distance in this behaviour is derived from the fused pose, so if
            odom_health says that pose is bad, "arrived" is a claim we cannot back
            up. Say so instead of reporting a clean success — a silent false
            success is what makes a robot look like it is lying.
            """
            warn = ap.pose_warning()
            print(f"ARRIVED near the {label} ({d:.2f} m)"
                  + ("." if seen_recently else " — at its last known position, "
                     "not currently in view."))
            if not warn:
                return 0
            print(f"  BUT THE POSE IS NOT TRUSTWORTHY — {warn}")
            print("  Treat this arrival as UNVERIFIED: the measured distance may "
                  "not match reality.")
            return 2

        goal_basis = None      # object position the live goal was built from
        gh = None              # active goal handle
        res_fut = None
        cap = None             # fallback reach cap after a rejection/abort
        last_seen = time.time()
        seen_recently = True
        t0 = time.time()
        last_report = 0.0

        while time.time() - t0 < timeout_s:
            # --- refresh target ------------------------------------------------
            # Losing sight does NOT abort. The whole point of grounding the object
            # in the MAP frame is that the goal survives occlusion, a label flip
            # (bottle->vase), the object leaving the camera FOV during a turn, or
            # YOLO simply missing a few frames. The robot is localized, so it can
            # keep driving to the last known coordinate — that is what makes this
            # behave like a robot instead of something that only moves while it is
            # staring at the target. We only stop on arrival or timeout.
            d = ap.target()
            if d:
                ox, oy = d[0], d[1]
                if not seen_recently:
                    print(f"  re-acquired '{label}'")
                seen_recently = True
                last_seen = time.time()
            elif seen_recently and time.time() - last_seen > 3.0:
                seen_recently = False
                print(f"  lost sight of '{label}' — continuing to its last known "
                      f"map position ({ox:.2f},{oy:.2f})")

            rxy = ap.robot_xy()
            if rxy is None:
                time.sleep(0.2)
                continue
            dist = math.hypot(ox - rxy[0], oy - rxy[1])

            # --- arrival -------------------------------------------------------
            if dist <= stop_dist + STOP_TOL:
                if gh is not None:
                    gh.cancel_goal_async()
                rc = report_arrival(dist)
                break

            # --- (re)issue the goal only when it actually matters ---------------
            moved = (goal_basis is None
                     or math.hypot(ox - goal_basis[0], oy - goal_basis[1]) > GOAL_UPDATE_M)
            if gh is None or moved:
                if gh is not None and moved:
                    print(f"  target moved >{GOAL_UPDATE_M} m — re-issuing goal")
                    gh.cancel_goal_async()
                    time.sleep(0.3)
                g = ap.goal_for(ox, oy, cap=cap)
                if g is None:
                    time.sleep(0.2)
                    continue
                gx, gy, gyaw, dist = g
                print(f"  goal ({gx:.2f},{gy:.2f}) yaw {math.degrees(gyaw):.0f}deg "
                      f"| {label} {dist:.2f} m away"
                      + (f" [capped to {cap:.1f} m]" if cap else ""))
                gh = ap.send_goal(gx, gy, gyaw)
                if gh is None:
                    # Rejected: shorten the goal so it lands inside well-mapped
                    # space, and try again next loop.
                    cap = FALLBACK_STEP
                    time.sleep(1.0)
                    continue
                res_fut = gh.get_result_async()
                goal_basis = (ox, oy)

            # --- watch the running goal ----------------------------------------
            if res_fut is not None and res_fut.done():
                status = res_fut.result().status
                if status == 4:          # SUCCEEDED
                    if dist <= stop_dist + STOP_TOL:
                        rc = report_arrival(dist)
                        break
                    # Succeeded short of the object (capped/fallback goal) — keep going.
                    print(f"  leg done, {dist:.2f} m to go — continuing")
                    cap = None
                else:
                    print(f"  nav2 goal ended status={status} (aborted/cancelled) "
                          f"— retrying shorter")
                    cap = FALLBACK_STEP
                    time.sleep(1.0)
                gh, res_fut = None, None
                continue

            # --- periodic status, incl. the silent-spin failure mode -------------
            if time.time() - last_report > 3.0:
                last_report = time.time()
                note = ""
                if ap.safety.startswith("FWD_BLOCKED"):
                    note = ("  <-- forward is being ZEROED by safety_guard; nav2 is "
                            "unaware and only its turn commands reach the wheels")
                elif ap.safety.startswith("TRIPPED"):
                    note = "  <-- safety_guard LATCHED; wheels held at zero"
                print(f"    {dist:.2f} m to go | safety: {ap.safety}{note}")

            time.sleep(0.2)
        else:
            print(f"timed out after {timeout_s:.0f}s approaching '{label}'.")

        if gh is not None and rc != 0:
            gh.cancel_goal_async()
            time.sleep(0.3)
    finally:
        ex.shutdown()
        th.join(timeout=2.0)
        ap.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(rc)


if __name__ == "__main__":
    main()
