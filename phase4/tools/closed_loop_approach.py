#!/usr/bin/env python3
"""closed_loop_approach.py — "go near X" on the twin: one-shot vs closed loop, judged on the truth.

    # in the rover_twin container (domain 42), the twin's stack up (TWIN=1 ./rover up):
    python3 closed_loop_approach.py --label bottle --truth 0.90 0.15 0.125 --mode closed
    python3 closed_loop_approach.py --truth 2.20 -0.35 0.005 --size 0.25 0.18 --vlm-dy 40 --mode oneshot

Plays the Pi 5 brain's part of CLOSED_LOOP_PLAN.md (stage 4; stage 5 moves it to
the Pi 5):
  1. FIND   --label: YOLO (yolo_backend, the TensorRT engine) on image_bridge's
            newest JPEG picks the box; otherwise a VLM stand-in points at the
            TRUE spot (--truth, projected through the camera pose) with a box of
            --size, shifted --vlm-dy px down (Gemma's measured y error is up to
            ~45 px: pixel_to_goal.py). The world frame IS odom on a fresh twin.
  2. GROUND /vision/pixel_snapshot + /vision/pixel_query with the box -> the object
            point, the standoff goal and the photo's camera pose (pixel_to_goal).
  3. LOCK   (closed) /target/lock with that photo, box, object and camera.
  4. DRIVE  /reach/goal. Closed: on /target/state, re-send the goal when
            target_lock.goal_update_due() says the point really moved (>= 12 cm,
            >= 2 s apart, never in the last metre). One-shot: never.
  5. JUDGE  when /reach/status ends our latest goal: the TRUE pose (/sim/truth/pose)
            against the TRUE object: distance base_link -> object centre (the target
            is standoff + the object's radius), and how far off-centre the rover
            faces it (lateral miss at the object, cm).
Prints one JSON line (and --out appends it to a file) for comparing runs.
"""
import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "nodes"))
import target_lock as tl  # noqa: E402


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Brain(Node):
    def __init__(self):
        super().__init__("closed_loop_approach")
        self.jpeg = None
        self.info = None
        self.truth = None
        self.results, self.states, self.reach = {}, [], []
        self.create_subscription(CompressedImage, "/camera/color/image_raw/compressed", self._jpeg, 2)
        self.create_subscription(CameraInfo, "/camera/camera0/color/camera_info", lambda m: setattr(self, "info", m), 2)
        self.create_subscription(PoseStamped, "/sim/truth/pose", lambda m: setattr(self, "truth", m.pose), 5)
        self.create_subscription(String, "/vision/pixel_result", self._result, 10)
        self.create_subscription(String, "/target/state", lambda m: self.states.append(json.loads(m.data)), 50)
        self.create_subscription(String, "/reach/status", lambda m: self.reach.append(json.loads(m.data)), 50)
        self.p_snap = self.create_publisher(PointStamped, "/vision/pixel_snapshot", 5)
        self.p_query = self.create_publisher(PointStamped, "/vision/pixel_query", 5)
        self.p_lock = self.create_publisher(String, "/target/lock", 5)
        self.p_cancel = self.create_publisher(String, "/target/cancel", 5)
        self.p_goal = self.create_publisher(PoseStamped, "/reach/goal", 5)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)

    def _jpeg(self, m):
        self.jpeg = m

    def _result(self, m):
        d = json.loads(m.data)
        self.results[d.get("id")] = d

    def wait_subscribers(self, s=5.0):
        """A new process's first messages are dropped until discovery has matched its
        publishers (a lock went missing, 2026-10-21): wait for every reader."""
        pubs = (self.p_snap, self.p_query, self.p_lock, self.p_goal)
        return self.spin_for(s, until=lambda: all(p.get_subscription_count() > 0 for p in pubs))

    def spin_for(self, s, until=None):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until and until():
                return True
        return False

    def robot(self):
        tf = self.tf.lookup_transform("odom", "base_link", Time())
        q = tf.transform.rotation
        return (tf.transform.translation.x, tf.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def goal(self, x, y, yaw):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.p_goal.publish(m)
        return f"{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="", help="find it with YOLO (a COCO label)")
    ap.add_argument("--truth", nargs=3, type=float, required=True, help="the object's TRUE centre x y z (world = odom)")
    ap.add_argument("--radius", type=float, default=0.033, help="the object's radius (m), for the judge")
    ap.add_argument("--size", nargs=2, type=float, default=(0.07, 0.25), help="VLM stand-in box size w h (m)")
    ap.add_argument("--vlm-dy", type=float, default=0.0, help="VLM stand-in: shift the box this many px down")
    ap.add_argument("--vlm-dx", type=float, default=0.0)
    ap.add_argument("--guess-off", nargs=2, type=float, default=(0.0, 0.0), metavar=("FWD", "LEFT"),
                    help="move the grounded first guess this far (m, from the rover's view): a bad "
                         "first estimate -- background depth behind a thin thing, clutter in the box")
    ap.add_argument("--mode", choices=("closed", "oneshot"), default="closed")
    ap.add_argument("--standoff", type=float, default=0.45)
    ap.add_argument("--timeout", type=float, default=150.0)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    rclpy.init()
    n = Brain()
    truth = np.array(a.truth)
    out = {"mode": a.mode, "label": a.label or "vlm-standin", "truth": a.truth, "vlm_dy": a.vlm_dy, "vlm_dx": a.vlm_dx}

    def done(**kw):
        out.update(kw)
        print(json.dumps(out))
        if a.out:
            open(a.out, "a").write(json.dumps(out) + "\n")
        n.p_cancel.publish(String(data=""))
        n.spin_for(0.3)
        rclpy.shutdown()
        sys.exit(0 if "error" not in out else 1)

    if not n.wait_subscribers():
        done(error="pixel_to_goal / target_track / reach are not all listening")
    if not n.spin_for(15, until=lambda: n.jpeg is not None and n.info is not None and n.truth is not None):
        done(error="no JPEG / camera_info / truth pose (is the twin and TWIN=1 ./rover up running?)")
    # world (the truth) -> odom (what the rover believes): equal on a fresh twin, but odom
    # drifts and runs are chained (the rover drives back to its start between runs)
    n.spin_for(5, until=lambda: n.tf.can_transform("odom", "base_link", Time()))
    try:
        rx, ry, ryaw = n.robot()
    except Exception as e:  # noqa: BLE001
        done(error=f"no odom -> base_link: {e}")
    tp = n.truth
    tyaw = math.atan2(2 * (tp.orientation.w * tp.orientation.z), 1 - 2 * tp.orientation.z ** 2)
    dyaw = ryaw - tyaw
    c, s_ = math.cos(dyaw), math.sin(dyaw)

    def world_to_odom(p):
        x, y = p[0] - tp.position.x, p[1] - tp.position.y
        return np.array([rx + c * x - s_ * y, ry + s_ * x + c * y, p[2]])
    truth_odom = world_to_odom(truth)
    out["odom_vs_world_cm"] = round(math.hypot(truth_odom[0] - truth[0], truth_odom[1] - truth[1]) * 100, 1)
    # a FRESH photo whose camera pose the TF buffer already holds (the listener started
    # with this script: an older photo is before its first transform)
    cam_frame = n.info.header.frame_id
    n.spin_for(5, until=lambda: n.tf.can_transform("odom", cam_frame, Time()))
    n.jpeg = None
    n.spin_for(3, until=lambda: n.jpeg is not None)
    n.spin_for(0.3)
    photo = n.jpeg
    img = cv2.imdecode(np.frombuffer(bytes(photo.data), np.uint8), cv2.IMREAD_COLOR)
    K = np.array(n.info.k).reshape(3, 3)
    stamp = photo.header.stamp

    # 1. FIND
    if a.label:
        import yolo_backend as yb
        import glob
        eng = next(iter(sorted(glob.glob("/models/yolov8n_640x384_trt*.engine"))), "/models/yolov8n.pt")
        det = yb.load_detector(eng, fallback="/models/yolov8n.pt")
        hits = [d for d in det.detect(img) if d[0] == a.label]
        if not hits:
            done(error=f"YOLO sees no {a.label} in the photo")
        box = list(max(hits, key=lambda d: d[1])[2])
    else:
        n.spin_for(0.3)
        try:
            tf = n.tf.lookup_transform("odom", photo.header.frame_id or n.info.header.frame_id, Time.from_msg(stamp))
        except Exception as e:  # noqa: BLE001
            done(error=f"no camera pose for the photo: {e}")
        R, t = quat_R(tf.transform.rotation), np.array([tf.transform.translation.x, tf.transform.translation.y,
                                                        tf.transform.translation.z])
        u, v, z = tl.project(K, R, t, truth_odom)
        if u is None:
            done(error="the truth is behind the camera")
        w = K[0, 0] * a.size[0] / z
        h = K[1, 1] * a.size[1] / z
        u, v = u + a.vlm_dx, v + a.vlm_dy
        box = [u - w / 2, v - h / 2, u + w / 2, v + h / 2]
    out["box"] = [round(b, 1) for b in box]

    # 2. GROUND (the photo's depth + pose, as the brain does)
    snap = PointStamped()
    snap.header.stamp = stamp
    n.p_snap.publish(snap)
    n.spin_for(0.5)
    q = PointStamped()
    q.header.stamp = stamp
    q.header.frame_id = "cla1;box=" + ",".join(f"{b:.0f}" for b in box)
    q.point.x, q.point.y = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    n.p_query.publish(q)
    if not n.spin_for(6, until=lambda: "cla1" in n.results):
        done(error="no reply from pixel_to_goal")
    r = n.results["cla1"]
    if not r.get("ok"):
        done(error=f"pixel_to_goal: {r.get('reason')}")
    obj = np.array([r["object"]["x"], r["object"]["y"], r["object"].get("z", 0.0)])
    if any(a.guess_off):
        f, l = a.guess_off                    # forward / left of the rover's heading now
        obj = obj + np.array([f * math.cos(ryaw) - l * math.sin(ryaw), f * math.sin(ryaw) + l * math.cos(ryaw), 0.0])
        r["object"] = {"x": float(obj[0]), "y": float(obj[1]), "z": float(obj[2])}
        gx, gy, gyaw = tl.standoff_goal(obj[:2], (rx, ry), a.standoff)
        r["goal"] = {"x": gx, "y": gy, "yaw": gyaw}
        out["guess_off"] = list(a.guess_off)
    out["first_guess_err_m"] = round(float(np.linalg.norm(obj[:2] - truth_odom[:2])), 3)

    # 3. LOCK
    if a.mode == "closed":
        lock = {"id": "cla1", "stamp": stamp.sec + stamp.nanosec * 1e-9, "box": box, "label": a.label or "thing",
                "object": r["object"], "camera": r["camera"]}
        n.p_lock.publish(String(data=json.dumps(lock)))
        if not n.spin_for(3, until=lambda: any(s.get("state") in ("locked", "refused") for s in n.states)):
            done(error="no answer from target_track")
        if any(s.get("state") == "refused" for s in n.states):
            done(error=f"lock refused: {[s for s in n.states if s.get('state') == 'refused'][0].get('reason')}")

    # 4. DRIVE
    g = r["goal"]
    my_stamp = n.goal(g["x"], g["y"], g["yaw"])
    sent_obj, sent_t, updates, t0 = obj[:2].copy(), time.time(), 0, time.time()
    seen = 0
    while time.time() - t0 < a.timeout:
        n.spin_for(0.2)
        fin = [s for s in n.reach if s.get("goal_stamp") == my_stamp and "result" in s]
        if fin:
            out["reach"] = fin[-1]["result"]
            # closed: the last metre on the refined point -- one exact correction
            last = next((s for s in reversed(n.states) if s.get("state") == "tracking"), None)
            if (a.mode == "closed" and fin[-1]["result"] == "reached" and last is not None
                    and not out.get("final_correction") and tl.final_correction_due(sent_obj, last["object"][:2])):
                rx, ry, _ = n.robot()
                p = np.array(last["object"])
                gx, gy, gyaw = tl.standoff_goal(p[:2], (rx, ry), a.standoff)
                my_stamp = n.goal(gx, gy, gyaw)
                out["final_correction"] = round(float(np.linalg.norm(p[:2] - sent_obj)) * 100, 1)
                sent_obj = p[:2].copy()
                print(f"  final correction: {out['final_correction']} cm", flush=True)
                continue
            break
        if a.mode == "closed":
            new = n.states[seen:]
            seen = len(n.states)
            tracked = [s for s in new if s.get("state") == "tracking"]
            if tracked:
                p = np.array(tracked[-1]["object"])
                try:
                    rx, ry, _ = n.robot()
                except Exception:  # noqa: BLE001
                    continue
                if tl.goal_update_due(sent_obj, p[:2], (rx, ry), time.time() - sent_t):
                    gx, gy, gyaw = tl.standoff_goal(p[:2], (rx, ry), a.standoff)
                    my_stamp = n.goal(gx, gy, gyaw)
                    sent_obj, sent_t = p[:2].copy(), time.time()
                    updates += 1
                    print(f"  goal update {updates}: object -> ({p[0]:.2f}, {p[1]:.2f}), "
                          f"{np.linalg.norm(p[:2] - truth_odom[:2]) * 100:.0f} cm from the truth", flush=True)
    else:
        out["reach"] = "timeout"
    n.spin_for(1.0)

    # 5. JUDGE on the truth
    tp = n.truth
    yaw = math.atan2(2 * (tp.orientation.w * tp.orientation.z), 1 - 2 * tp.orientation.z ** 2)
    dx, dy = truth[0] - tp.position.x, truth[1] - tp.position.y
    dist = math.hypot(dx, dy)
    lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)      # + = object to the rover's left
    st = {}
    for s in n.states:
        st[s.get("state")] = st.get(s.get("state"), 0) + 1
    final_obj = next((s["object"] for s in reversed(n.states) if s.get("state") == "tracking"), None)
    done(time_s=round(time.time() - t0, 1), goal_updates=updates,
         dist_to_centre_m=round(dist, 3), dist_err_cm=round((dist - a.standoff - a.radius) * 100, 1),
         lateral_miss_cm=round(lateral * 100, 1), facing_err_deg=round(math.degrees(math.atan2(lateral, dist)), 1),
         tracker_states=st,
         final_track_err_m=None if final_obj is None else round(float(np.linalg.norm(np.array(final_obj[:2]) - truth_odom[:2])), 3))


if __name__ == "__main__":
    main()
