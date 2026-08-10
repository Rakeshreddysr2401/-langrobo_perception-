#!/usr/bin/env python3
"""
stack_status.py — the single source of truth for "is the stack actually healthy?"

WHY THIS EXISTS
    The old `run_stack.sh status` printed PUBLISHER COUNTS. Every failure this rig has
    ever had was a RATE COLLAPSE, not an absence:

      * D555 stereo IR falling 22 Hz -> 0.97 Hz under CPU load (publisher count: still 1)
      * cuVSLAM freezing: process alive, slam=True, frame counter rising, but /odom
        stops (publisher count: still 1)
      * nvblox back_projected_depth at 0.47 Hz with 7.3 s gaps (publisher count: still 1)
      * ESP32 /wheel_state at 1 Hz instead of 20 Hz (publisher count: still 1)
        — since reflashed; it now runs at 10 Hz, still half the firmware's 20 (learn/03-imu.md)

    A publisher count of 1 was printed in every one of those cases. So it measures rates,
    TF freshness, and pose trust instead.

    `ros2 topic hz` is unreliable here (README S9: "the CLI probe is flaky under load"),
    so this subscribes to everything at once in one process and counts for a fixed window.

USAGE
    python3 stack_status.py [--window 4.0] [--json]

QoS NOTE
    Everything is subscribed BEST_EFFORT + VOLATILE, which is compatible with every
    publisher on this stack (a BEST_EFFORT sub accepts a RELIABLE pub; a VOLATILE sub
    accepts a TRANSIENT_LOCAL pub). That keeps one QoS profile for all topics.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message

import tf2_ros

# topic -> (label, expected_min_hz, owning run_stack.sh layer, why a drop matters)
#
# The owner matters: "no publisher" means two very different things depending on it.
# If you have not run './run_stack.sh fuse' yet, a missing EKF is EXPECTED, not a
# fault -- so absence is reported as "not started (run: fuse)" rather than as a
# problem. A topic whose owner IS running but whose rate has collapsed is the real
# failure mode this tool exists to catch.
WATCH = [
    ("/camera/camera0/infra1/image_rect_raw", "camera IR left",  15.0, "up",
     "cuVSLAM input. <10 Hz => cuVSLAM freezes SILENTLY and never recovers"),
    ("/camera/camera0/depth/image_rect_raw",  "camera depth",    10.0, "up",
     "nvblox + safety_guard bumper input"),
    ("/odom",                                 "cuVSLAM odom",    10.0, "up",
     "visual odometry. 0 Hz with the camera alive = cuVSLAM FROZEN, restart it"),
    ("/nvblox_node/static_map_slice",         "nvblox slice",     5.0, "up",
     "the costmap's obstacle layer"),
    ("/odometry/filtered",                    "EKF odom",        10.0, "fuse",
     "fused pose"),
    ("/imu/base",                             "IMU (gyro)",      50.0, "fuse",
     "gyro yaw into the EKF"),
    ("/perception/depth_points",              "collision src",    5.0, "vision",
     "nav2 collision_monitor source. Stale => monitor HOLDS THE ROBOT AT ZERO"),
    ("/wheel_state",                          "ESP32 encoders",  15.0, "esp32",
     "MUST be ~20 Hz. Measured 10 Hz 2026-08-11 = half rate, cause unknown (learn/03-imu.md). "
     "1 Hz = ESP32 on pre-f4be55a firmware, reflash it"),
    ("/cmd_vel",                              "cmd_vel out",      0.0, "-",
     "final motor command (0 Hz when parked is correct)"),
]

TFS = [("map", "odom", "SLAM correction"), ("odom", "base_link", "pose")]


class StackStatus(Node):
    def __init__(self, window):
        super().__init__("stack_status")
        self.window = window
        self.counts = {t: 0 for t, _, _, _, _ in WATCH}
        self.last = {t: None for t, _, _, _, _ in WATCH}
        self.health = None

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.present = {t: False for t, _, _, _, _ in WATCH}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def discover(self, timeout=5.0):
        """Wait for DDS discovery, THEN subscribe.

        Do not merge this back into __init__: get_topic_names_and_types() returns an
        EMPTY graph for the first second or two of a node's life, so subscribing
        immediately reported "no publisher" for every topic on a perfectly healthy
        stack (observed 2026-08-10 — TF and /slam/status were live at the time).
        Poll until the watched topics show up rather than guessing a sleep.
        """
        deadline = time.time() + timeout
        graph = {}
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            graph = dict(self.get_topic_names_and_types())
            # stop early once every watched topic that will ever appear has
            if all(t in graph for t, _, _, _, _ in WATCH):
                break

        for topic, _, _, _, _ in WATCH:
            types = graph.get(topic)
            if not types:
                continue
            try:
                msg_cls = get_message(types[0])
            except Exception:
                continue
            self.present[topic] = True
            self.create_subscription(
                msg_cls, topic, self._make_cb(topic), self.qos)

        # /odom/health is low rate; grab whatever it last said.
        if "/odom/health" in graph:
            from std_msgs.msg import String
            self.create_subscription(String, "/odom/health", self._health_cb, self.qos)

        # Subscriptions need a moment to match their publishers before counting starts,
        # otherwise the first fraction of the window is silent and every rate reads low.
        settle = time.time() + 1.0
        while time.time() < settle:
            rclpy.spin_once(self, timeout_sec=0.05)
        for t in self.counts:
            self.counts[t] = 0

    def _make_cb(self, topic):
        def cb(_msg):
            self.counts[topic] += 1
            self.last[topic] = time.time()
        return cb

    def _health_cb(self, msg):
        try:
            self.health = json.loads(msg.data)
        except Exception:
            self.health = {"raw": msg.data}

    def tf_ages(self):
        """Age in seconds of each TF hop, or None if the transform is absent."""
        out = {}
        for parent, child, _ in TFS:
            try:
                tr = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time())
                stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                # stamp 0 means "latest available" -- treat as fresh, not 55 years old
                out[(parent, child)] = 0.0 if stamp == 0.0 else max(
                    0.0, time.time() - stamp)
            except Exception:
                out[(parent, child)] = None
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--window", type=float, default=4.0,
                    help="seconds to count messages over (default 4)")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args()

    rclpy.init()
    node = StackStatus(args.window)
    node.discover()

    deadline = time.time() + args.window
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    rates = {t: node.counts[t] / args.window for t, _, _, _, _ in WATCH}
    ages = node.tf_ages()

    try:
        with open("/proc/loadavg") as f:
            load1 = float(f.read().split()[0])
    except Exception:
        load1 = float("nan")

    if args.json:
        print(json.dumps({
            "rates_hz": rates,
            "present": node.present,
            "tf_age_s": {f"{p}->{c}": ages[(p, c)] for p, c, _ in TFS},
            "odom_health": node.health,
            "load1": load1,
        }, indent=2))
        rclpy.shutdown()
        return 0

    bad = []       # real failures: something is running but wrong
    missing = []   # a layer simply has not been started yet
    print(f"\n  stack status  (counted over {args.window:.0f}s)\n")
    print(f"  {'':<18} {'rate':>9}   {'want':>7}")
    for topic, label, want, owner, why in WATCH:
        hz = rates[topic]
        if not node.present[topic]:
            mark, note = "  --", f"not started (run: {owner})"
            if want > 0:
                missing.append(f"{label}: not started — ./run_stack.sh {owner}"
                               if owner not in ("-", "esp32")
                               else f"{label}: not publishing — {why}")
        elif want > 0 and hz < want:
            mark, note = "  !!", f"LOW — {why}"
            bad.append(f"{label}: {hz:.1f} Hz (want >={want:.0f}) — {why}")
        else:
            mark, note = "  ok", ""
        want_s = f">={want:.0f} Hz" if want > 0 else "-"
        print(f"{mark} {label:<18} {hz:>6.1f} Hz   {want_s:>7}   {note}")

    print()
    for parent, child, meaning in TFS:
        age = ages[(parent, child)]
        if age is None:
            print(f"  !! TF {parent}->{child:<10} MISSING   ({meaning})")
            bad.append(f"TF {parent}->{child} missing — {meaning}")
        elif age > 1.0:
            print(f"  !! TF {parent}->{child:<10} {age:5.1f}s old  ({meaning}) STALE")
            bad.append(f"TF {parent}->{child} is {age:.1f}s stale")
        else:
            print(f"  ok TF {parent}->{child:<10} {age:5.2f}s old  ({meaning})")

    print()
    if node.health is None:
        print("  -- /odom/health   not published (run './run_stack.sh fuse')")
    else:
        trust = node.health.get("trust")
        status = node.health.get("status", "?")
        mark = "ok" if trust else "!!"
        print(f"  {mark} pose trust      trust={trust}  status={status}")
        if not trust:
            bad.append(f"POSE NOT TRUSTED ({status}) — do not drive, "
                       f"do not believe any 'arrived'")

    # CPU is a safety property on this box: load >8 on 6 cores starves the D555
    # reader callbacks, which is what freezes cuVSLAM.
    mark = "!!" if load1 > 8.0 else "ok"
    print(f"  {mark} orin load1      {load1:.1f}   (>8 on 6 cores starves the camera)")
    if load1 > 8.0:
        bad.append(f"load {load1:.1f} — expect IR to drop and cuVSLAM to freeze")

    print()
    if bad:
        print(f"  {len(bad)} PROBLEM(S) — something IS running but is wrong:")
        for b in bad:
            print(f"    - {b}")
        print()
    if missing:
        print(f"  {len(missing)} layer(s) not started (fine if you haven't got there yet):")
        for m in missing:
            print(f"    - {m}")
        print()
    if not bad and not missing:
        print("  all checks passed\n")

    rclpy.shutdown()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
