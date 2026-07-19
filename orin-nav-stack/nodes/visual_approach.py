#!/usr/bin/env python3
"""visual_approach.py <label> — drive to a VISIBLE object by camera bearing.

No map, no SLAM, no saved coordinates: pure visual servoing. This is the
"go to the thing I can see" behaviour — robust to the map/costmap problems
that make the Nav2 path abort (reflective-floor depth noise, allow_unknown,
sparse costmap). It closes a loop on what YOLO sees in the image RIGHT NOW.

Flow:
  1. Set the YOLO target hunt: publish <label> on /vision/target. The Jetson
     detector (detections_3d) then answers every tick on /vision/target_result
     with {found, bearing_x (-1 left..+1 right), rel_size (0..1 apparent size)}.
  2. Close the loop at 10 Hz, publishing motion on /cmd_vel until the object
     fills rel_size >= --stop of the frame, then stop.

Control law (tuned to THIS rover's hardware):
  - Forward motion works on smooth marble; IN-PLACE TURNS STARVE FOR TORQUE
    (they stall + hum). So we STEER BY ARC: always creep forward while turning,
    which drives both wheels forward at different speeds (good traction) instead
    of counter-rotating them (stalls). wz is proportional to the bearing error.
  - Publishes /cmd_vel directly at low speed, like drive_test.py (the proven
    path). NOTE: this bypasses the Nav2 collision monitor, so speeds are kept
    low and we stop on apparent size; supervise the first runs.
  - Tracking drop-out (object leaves the narrow, down-tilted FOV) is expected:
    within a short grace we keep the last steer to swing back toward it; past
    that we STOP and wait to re-acquire rather than spin blindly.

Run in the isaac_ros container, ROS sourced, ROS_DOMAIN_ID=0:
  python3 visual_approach.py "potted plant"
  python3 visual_approach.py bottle --stop 0.4 --timeout 45
"""
import argparse
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

LOOP_HZ = 10.0
FRESH_S = 0.6         # a target_result older than this is treated as no data
PERSIST_S = 1.5       # keep driving on the last sighting through this much dropout
REPORT_S = 1.0        # telemetry print period


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("label", help="COCO class to approach, e.g. 'potted plant'")
    ap.add_argument("--stop", type=float, default=0.45,
                    help="rel_size at which we've arrived (0..1)")
    ap.add_argument("--vx", type=float, default=0.15, help="forward cruise m/s")
    ap.add_argument("--wz-max", type=float, default=0.9, help="max steer rad/s")
    ap.add_argument("--kang", type=float, default=1.3, help="steer gain on bearing")
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    n = Node("visual_approach")
    tpub = n.create_publisher(String, "/vision/target", 10)
    cpub = n.create_publisher(Twist, "/cmd_vel", 10)

    state = {"res": None, "t": 0.0}

    def on_res(m):
        try:
            state["res"] = json.loads(m.data)
            state["t"] = time.time()
        except (json.JSONDecodeError, TypeError):
            pass

    n.create_subscription(String, "/vision/target_result", on_res, 10)

    def stop():
        z = Twist()
        for _ in range(5):
            cpub.publish(z)
            time.sleep(0.02)

    # Let DDS match the detector's subscriber before the first target publish,
    # or it fires into the void (same gotcha as langrobo_client.ground_pixel).
    deadline = time.time() + 5.0
    while time.time() < deadline and tpub.get_subscription_count() == 0:
        rclpy.spin_once(n, timeout_sec=0.1)
    tpub.publish(String(data=args.label))
    print(f"hunting {args.label!r} -> arc-approach until rel_size>={args.stop} "
          f"(vx={args.vx} wz_max={args.wz_max} timeout={args.timeout}s)")
    if tpub.get_subscription_count() == 0:
        print("WARNING: nobody subscribed to /vision/target — is detections_3d up?")

    t0 = time.time()
    last_seen = 0.0
    last_bearing = 0.0
    last_pub = 0.0
    last_report = 0.0
    period = 1.0 / LOOP_HZ
    outcome = "timeout"
    best_size = last_size = 0.0
    ticks = hits = 0        # for the found-rate telemetry
    try:
        while time.time() - t0 < args.timeout:
            rclpy.spin_once(n, timeout_sec=0.02)
            now = time.time()
            # Re-assert the target ~every 2 s so a detector restart re-arms.
            if now - last_pub >= period and int((now - t0) * 2) % 4 == 0:
                tpub.publish(String(data=args.label))

            r = state["res"]
            found = bool(r and r.get("found")) and (now - state["t"]) < FRESH_S
            cmd = Twist()
            mode = "hold"

            if found:
                b = float(r.get("bearing_x", 0.0))
                last_bearing, last_seen = b, now
                last_size = float(r.get("rel_size", 0.0))
                best_size = max(best_size, last_size)
                if last_size >= args.stop:
                    outcome = "arrived"
                    break
                # Arc toward the object: creep forward (traction) + steer
                # proportional to how far off-centre it is.
                cmd.linear.x = args.vx
                cmd.angular.z = clamp(-args.kang * b, -args.wz_max, args.wz_max)
                mode = "track"
            elif last_seen > 0 and (now - last_seen) < PERSIST_S:
                # Brief dropout (object flickered out of the narrow FOV): KEEP
                # driving on the last bearing so a distant, blinking target
                # still closes distance instead of stalling. Decay forward as
                # the gap grows so we don't barrel on blindly.
                decay = 1.0 - (now - last_seen) / PERSIST_S
                cmd.linear.x = args.vx * decay
                cmd.angular.z = clamp(-args.kang * last_bearing,
                                      -args.wz_max, args.wz_max)
                mode = "coast"
            else:
                cmd = Twist()  # lost too long -> hold, wait to re-acquire

            ticks += 1
            hits += 1 if found else 0
            if now - last_pub >= period:  # 10 Hz feeds the ~500 ms ESP32 watchdog
                cpub.publish(cmd)
                last_pub = now
            if now - last_report >= REPORT_S:
                print(f"  [{now-t0:4.1f}s] {mode:5s} track={100*hits//max(ticks,1):3d}% "
                      f"bearing={last_bearing:+.2f} size={last_size:.2f} "
                      f"vx={cmd.linear.x:.2f} wz={cmd.angular.z:+.2f}", flush=True)
                last_report = now
    finally:
        stop()
        tpub.publish(String(data=""))  # end the target hunt
        n.destroy_node()
        rclpy.shutdown()

    print(f"RESULT: {outcome} (best rel_size seen {best_size:.2f}, "
          f"elapsed {time.time()-t0:.1f}s)")


if __name__ == "__main__":
    main()
