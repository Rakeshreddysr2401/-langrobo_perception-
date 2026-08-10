#!/usr/bin/env python3
"""Is the pose trustworthy right now? Publishes /odom/health for motion behaviours.

WHY (2026-08-10)
----------------
cuVSLAM fails SILENTLY. During a live "go near the bottle" run on polished marble
with the IR emitter off (it must stay off, or scale breaks worse) it reported
`slam=True` with a steadily rising tracked-frame count while:

  * under-reporting translation ~8x — it claimed 0.13 m of travel for a drive that
    physically covered ~1.1 m and ended beside the target, and
  * drifting yaw ~40 deg while the rover was PARKED with motors dead, nose to a
    flat door with nothing to track.

Nothing in the stack noticed. Every closed-loop behaviour (approach_object's
distance-to-target, odom_move's calibrated moves, "go back to the start pose")
kept acting on that pose as if it were fact — which is exactly how a robot drives
confidently into a wall.

This node is the missing cross-check. It compares three independent stories about
how far the robot has moved:

    commanded   integral of /cmd_vel linear.x        "what we asked for"
    visual      path length of cuVSLAM /odom         "what the camera believes"
    wheels      integral of /wheel_odom twist vx     "what the ground says"

Wheels are the honest witness: encoders do not care about texture or reflections.
/wheel_state currently arrives at 10 Hz, not the 20 Hz the firmware asks for, so the
wheel comparison is coarse and is reported as LOW_RATE rather than trusted blindly.
(It was 1 Hz before the reflash; the remaining half-rate is tracked in learn/03-imu.md.)
The commanded-vs-visual and stationary-drift checks work regardless.

    /odom/health  std_msgs/String  JSON:
      {"status": "OK" | "VO_UNDER_REPORTING" | "VO_DRIFT_STATIONARY"
                 | "VO_SCALE_OFF" | "NO_DATA",
       "trust": true|false, "vo_m":, "wheel_m":, "cmd_m":, "scale":,
       "wheel_rate_hz":, "detail": "..."}

`trust: false` means: do not start a new autonomous move, and do not believe any
"I have arrived" claim. Consumers: approach_object.py, odom_move.py (home).
"""
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

WINDOW_S = 4.0           # rolling comparison window
MIN_CMD_M = 0.20         # need this much commanded travel before judging scale
UNDER_RATIO = 0.35       # visual < this * commanded => under-reporting
SCALE_LO, SCALE_HI = 0.65, 1.45     # acceptable visual/wheel ratio
DRIFT_YAW_DEG = 6.0      # yaw change while commanded-stationary => drift
WHEEL_STALE_S = 2.0
WHEEL_MIN_RATE = 5.0     # Hz below which wheel odom is too coarse to trust
VO_STALE_S = 1.5         # no cuVSLAM /odom for this long => it has stalled
IR_MIN_RATE = 10.0       # Hz: below this the stereo pair cannot feed cuVSLAM
IR_TOPIC = "/camera/camera0/infra1/image_rect_raw"


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomHealth(Node):
    def __init__(self):
        super().__init__("odom_health")
        rel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.vo = deque()        # (t, x, y, yaw) from cuVSLAM /odom
        self.cmd = deque()       # (t, |vx|)
        self.wheel = deque()     # (t, |vx|)
        self.wheel_times = deque(maxlen=40)
        self.create_subscription(Odometry, "/odom", self._vo_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/wheel_odom", self._wheel_cb, rel)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        # Stereo IR liveness. On a saturated Orin the D555's DDS reader callbacks
        # fall behind and this pair collapses from ~22 Hz to ~1 Hz; cuVSLAM then
        # freezes and NEVER recovers on its own, while everything downstream keeps
        # navigating on its last pose. Measured 2026-08-10 — this check exists so
        # that failure announces itself instead of being mistaken for bad odometry.
        self.ir_times = deque(maxlen=40)
        self.create_subscription(Image, IR_TOPIC,
                                 lambda _m: self.ir_times.append(time.time()),
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(String, "/odom/health", 10)
        self.create_timer(0.5, self._tick)
        self._last_status = None
        self.get_logger().info(
            "odom_health up: cross-checking cuVSLAM /odom against /cmd_vel and "
            "/wheel_odom -> /odom/health")

    # ---- inputs ----------------------------------------------------------
    def _vo_cb(self, m):
        p = m.pose.pose.position
        self.vo.append((time.time(), p.x, p.y, yaw_of(m.pose.pose.orientation)))

    def _wheel_cb(self, m):
        t = time.time()
        self.wheel.append((t, abs(m.twist.twist.linear.x)))
        self.wheel_times.append(t)

    def _cmd_cb(self, m):
        self.cmd.append((time.time(), abs(m.linear.x)))

    @staticmethod
    def _trim(dq, cutoff):
        while dq and dq[0][0] < cutoff:
            dq.popleft()

    @staticmethod
    def _integrate(dq):
        """Trapezoid-free: each sample holds until the next (commands are ZOH)."""
        total = 0.0
        for i in range(1, len(dq)):
            dt = dq[i][0] - dq[i - 1][0]
            if 0.0 < dt < 1.0:
                total += dq[i - 1][1] * dt
        return total

    @staticmethod
    def _rate(times):
        if len(times) < 2:
            return 0.0
        span = times[-1] - times[0]
        return (len(times) - 1) / span if span > 0 else 0.0

    def _wheel_rate(self):
        return self._rate(self.wheel_times)

    # ---- verdict ---------------------------------------------------------
    def _tick(self):
        now = time.time()
        cutoff = now - WINDOW_S
        for dq in (self.vo, self.cmd, self.wheel):
            self._trim(dq, cutoff)

        # Liveness first: a stalled camera or a frozen cuVSLAM makes every
        # comparison below meaningless, and is the failure that actually bit us.
        while self.ir_times and self.ir_times[0] < now - WINDOW_S:
            self.ir_times.popleft()
        ir_rate = self._rate(self.ir_times)
        vo_age = (now - self.vo[-1][0]) if self.vo else 1e9

        if vo_age > VO_STALE_S:
            return self._emit(
                "VO_STALLED", False, 0, 0, 0, None, self._wheel_rate(),
                f"no cuVSLAM /odom for {vo_age:.1f}s (stereo IR {ir_rate:.1f} Hz). "
                f"cuVSLAM does NOT self-recover — restart it (./run_stack.sh fuse). "
                f"Until then the pose is FROZEN at its last value.",
                ir_rate)
        if ir_rate and ir_rate < IR_MIN_RATE:
            return self._emit(
                "CAMERA_STARVED", False, 0, 0, 0, None, self._wheel_rate(),
                f"stereo IR only {ir_rate:.1f} Hz (needs >{IR_MIN_RATE:.0f}). The "
                f"Orin is CPU-saturated and the D555 DDS callbacks are falling "
                f"behind — cuVSLAM is about to freeze. Shed load (pause YOLO).",
                ir_rate)

        if len(self.vo) < 3:
            self._emit("NO_DATA", False, 0, 0, 0, None,
                       detail="no cuVSLAM /odom in window")
            return

        vo_m = sum(math.dist(self.vo[i][1:3], self.vo[i - 1][1:3])
                   for i in range(1, len(self.vo)))
        dyaw = abs(math.degrees(math.atan2(
            math.sin(self.vo[-1][3] - self.vo[0][3]),
            math.cos(self.vo[-1][3] - self.vo[0][3]))))
        cmd_m = self._integrate(self.cmd)
        wheel_m = self._integrate(self.wheel)
        w_rate = self._wheel_rate()
        w_fresh = bool(self.wheel) and (now - self.wheel[-1][0]) < WHEEL_STALE_S
        scale = (vo_m / wheel_m) if (w_fresh and wheel_m > 0.15) else None

        # 1) commanded a real drive but the camera barely moved
        if cmd_m >= MIN_CMD_M and vo_m < UNDER_RATIO * cmd_m:
            return self._emit(
                "VO_UNDER_REPORTING", False, vo_m, wheel_m, cmd_m, scale, w_rate,
                f"commanded {cmd_m:.2f} m but visual odom moved {vo_m:.2f} m "
                f"({vo_m / cmd_m:.0%}) — low-texture surface, pose is NOT reliable")

        # 2) told to hold still but the pose is wandering
        if cmd_m < 0.02 and dyaw > DRIFT_YAW_DEG:
            return self._emit(
                "VO_DRIFT_STATIONARY", False, vo_m, wheel_m, cmd_m, scale, w_rate,
                f"stationary but visual yaw drifted {dyaw:.1f} deg in {WINDOW_S:.0f}s "
                f"— camera has nothing to track")

        # 3) encoders disagree with the camera about distance
        if scale is not None and w_rate >= WHEEL_MIN_RATE and not (SCALE_LO <= scale <= SCALE_HI):
            return self._emit(
                "VO_SCALE_OFF", False, vo_m, wheel_m, cmd_m, scale, w_rate,
                f"visual {vo_m:.2f} m vs wheels {wheel_m:.2f} m (scale {scale:.2f}) "
                f"— they disagree about how far the robot went")

        detail = "pose consistent"
        if w_rate and w_rate < WHEEL_MIN_RATE:
            detail = (f"pose plausible, but wheel odom is only {w_rate:.1f} Hz "
                      f"(reflash ESP32 for 20 Hz) so the ground truth check is coarse")
        elif not w_fresh:
            detail = "pose plausible, but NO wheel odom — camera is unchecked"
        self._emit("OK", True, vo_m, wheel_m, cmd_m, scale, w_rate, detail)

    def _emit(self, status, trust, vo_m, wheel_m, cmd_m, scale, w_rate=0.0,
              detail="", ir_rate=None):
        self.pub.publish(String(data=json.dumps({
            "status": status, "trust": trust,
            "vo_m": round(vo_m, 3), "wheel_m": round(wheel_m, 3),
            "cmd_m": round(cmd_m, 3),
            "scale": (round(scale, 3) if scale is not None else None),
            "wheel_rate_hz": round(w_rate, 1),
            "ir_rate_hz": (round(ir_rate, 1) if ir_rate is not None
                           else round(self._rate(self.ir_times), 1)),
            "detail": detail,
        })))
        if status != self._last_status:
            # Two distinct call sites on purpose: rclpy caches severity PER CALL
            # SITE and raises "Logger severity cannot be changed between calls."
            # if one line alternates between info() and warning().
            if trust:
                self.get_logger().info(f"{status}: {detail}")
            else:
                self.get_logger().warning(f"{status}: {detail}")
            self._last_status = status


def main():
    rclpy.init()
    rclpy.spin(OdomHealth())


if __name__ == "__main__":
    main()
