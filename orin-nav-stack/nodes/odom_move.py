#!/usr/bin/env python3
"""Closed-loop precise move on the FUSED EKF odometry (/odometry/filtered).

Unlike drive_test.py (which watches raw /odom + a separately-integrated gyro,
each with a hand-tuned fudge factor), this closes the loop on the EKF output
that already fuses cuVSLAM (absolute x,y,yaw) + D555 gyro (yaw-rate) + wheel
encoders (vx). Distance AND heading both come from one consistent source, so
there is no CAL/ROT_CAL guesswork — measure once, tune only if the floor lies.

Run INSIDE the container (ROS 2 sourced, ROS_DOMAIN_ID=0):
    straight <cm> [speed]      forward until fused odom moves <cm>
    back <cm> [speed]          backward
    rotate <deg> [r|l] [wz]    turn in place until fused yaw sweeps <deg> (default left)
    home [speed]               drive back to (0,0) then face original heading

Feeds the ESP32 500 ms watchdog at 20 Hz and always sends an explicit stop.
Publishes straight to /cmd_vel (bypasses safety_guard, like drive_test/teleop) —
drive in clear space and keep tether slack. Rotation/reverse are always allowed
by safety_guard anyway; forward is the only gated direction.
"""
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

ODOM_TOPIC = "/odometry/filtered"

# Tunables (calibrated live 2026-08-09, emitter OFF).
#   straight: cmd 60cm -> odom-stop 58 -> real 65 (odom ~0.97x true + ~5cm coast)
DIST_CAL = 1.03          # real_cm = DIST_CAL * odom_cm (cuVSLAM ~3% under)
COAST_M = 0.03           # stop this far early so momentum coasts onto target
#   cmd60: COAST 0.05 -> real 57 (under) ; 0.00 -> real 65 (over) ; 0.03 ~= 60
ROT_CAL = 1.00           # real_deg = ROT_CAL * odom_deg (measure on first rotate)
V_CRUISE = 0.15          # m/s default forward cruise
V_MIN = 0.09             # m/s floor (below this the motors stall)
W_CRUISE = 1.2           # rad/s default rotate
W_MIN = 0.55             # rad/s floor
SLOW_D = 0.15            # m: start ramping down inside this remaining distance
SLOW_A = math.radians(25)  # rad: start ramping down inside this remaining angle
TOL_D = 0.02            # m stop tolerance
TOL_A = math.radians(2.5)  # rad stop tolerance
TIMEOUT = 30.0
PUB_DT = 0.05           # 20 Hz -> feeds the 500 ms watchdog


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Mover:
    def __init__(self, node):
        self.node = node
        self.x = self.y = self.yaw = None
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        node.create_subscription(Odometry, ODOM_TOPIC, self._cb, qos)
        self.pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def _cb(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = yaw_of(m.pose.pose.orientation)

    def wait_ready(self, t=6.0):
        t0 = time.time()
        while time.time() - t0 < t and self.x is None:
            time.sleep(0.05)
        return self.x is not None

    def _drive(self, target, is_rot, sign, cruise, vmin, slow, tol):
        """Publish sign*speed (ramped) until `achieved` reaches target. Returns achieved."""
        x0, y0, yaw0 = self.x, self.y, self.yaw
        acc_yaw = 0.0
        prev_yaw = yaw0
        t0 = time.time()
        last = 0.0
        achieved = 0.0
        while time.time() - t0 < TIMEOUT:
            if is_rot:
                d = self.yaw - prev_yaw
                if d > math.pi:
                    d -= 2 * math.pi
                elif d < -math.pi:
                    d += 2 * math.pi
                acc_yaw += d
                prev_yaw = self.yaw
                achieved = abs(acc_yaw)
            else:
                achieved = math.hypot(self.x - x0, self.y - y0)
            remaining = target - achieved
            if remaining <= tol:
                break
            spd = cruise if remaining > slow else max(vmin, cruise * remaining / slow)
            now = time.time()
            if now - last >= PUB_DT:
                cmd = Twist()
                if is_rot:
                    cmd.angular.z = sign * spd
                else:
                    cmd.linear.x = sign * spd
                self.pub.publish(cmd)
                last = now
            time.sleep(0.005)
        self.stop()
        return achieved

    def stop(self):
        for _ in range(6):
            self.pub.publish(Twist())
            time.sleep(0.03)

    def straight(self, cm, speed, back=False):
        target = (cm / 100.0) / DIST_CAL
        stop_target = max(0.0, target - COAST_M)   # break early; coast onto target
        sign = -1.0 if back else 1.0
        print(f"{'back' if back else 'straight'} {cm}cm -> fused-odom target {target*100:.1f}cm "
              f"(stop@{stop_target*100:.0f}cm + coast)")
        got = self._drive(stop_target, False, sign, speed, V_MIN, SLOW_D, TOL_D)
        print(f"  DONE: fused odom moved {got*100:.1f}cm at stop "
              f"-> ~{(got+COAST_M)*100*DIST_CAL:.0f}cm real expected after coast")

    def rotate(self, deg, right, wz):
        target = math.radians(deg) / ROT_CAL
        sign = -1.0 if right else 1.0
        print(f"rotate {deg}deg {'RIGHT' if right else 'LEFT'} -> fused-yaw target {math.degrees(target):.0f}deg")
        got = self._drive(target, True, sign, wz, W_MIN, SLOW_A, TOL_A)
        print(f"  DONE: fused yaw swept {math.degrees(got):.1f}deg (target {deg}) "
              f"-> ~{math.degrees(got)*ROT_CAL:.0f}deg real expected")

    def home(self, speed):
        """Return to (0,0) then restore heading 0 — using fused pose directly."""
        print(f"home: from ({self.x:.2f},{self.y:.2f}, yaw {math.degrees(self.yaw):.0f}) -> (0,0, yaw 0)")
        dist = math.hypot(self.x, self.y)
        if dist > TOL_D:
            # 1) rotate to face the origin
            bearing = math.atan2(-self.y, -self.x)
            self._turn_to(bearing, speed_w=W_CRUISE)
            # 2) drive the straight-line distance to origin
            self.straight(dist * 100.0, speed)
        # 3) restore original heading (yaw 0)
        self._turn_to(0.0, speed_w=W_CRUISE)
        print(f"  HOME DONE: now at ({self.x:.2f},{self.y:.2f}, yaw {math.degrees(self.yaw):.0f})")

    def _turn_to(self, target_yaw, speed_w):
        err = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
        self.rotate(abs(math.degrees(err)), right=(err < 0), wz=speed_w)


def main():
    a = sys.argv
    mode = a[1] if len(a) > 1 else ""
    if mode not in ("straight", "back", "rotate", "home"):
        print(__doc__)
        sys.exit(1)

    rclpy.init()
    node = rclpy.create_node("odom_move")
    mv = Mover(node)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    if not mv.wait_ready():
        print(f"NO {ODOM_TOPIC} — is fusion up? (./run_stack.sh fuse)")
        rclpy.shutdown()
        return
    if mv.pub.get_subscription_count() == 0:
        print("WARNING: no /cmd_vel subscriber (rover/ESP32 not linked?)")

    try:
        if mode == "straight":
            mv.straight(float(a[2]), float(a[3]) if len(a) > 3 else V_CRUISE)
        elif mode == "back":
            mv.straight(float(a[2]), float(a[3]) if len(a) > 3 else V_CRUISE, back=True)
        elif mode == "rotate":
            right = len(a) > 3 and a[3].lower().startswith("r")
            wz = float(a[4]) if len(a) > 4 else W_CRUISE
            mv.rotate(float(a[2]), right, wz)
        elif mode == "home":
            mv.home(float(a[2]) if len(a) > 2 else V_CRUISE)
    finally:
        mv.stop()
        ex.shutdown()
        th.join(timeout=2.0)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
