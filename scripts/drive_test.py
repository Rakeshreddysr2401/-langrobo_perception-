#!/usr/bin/env python3
"""Closed-loop drivetrain calibration helper.

Drives the rover by watching FEEDBACK (not a timer) and stops at the target:
  straight/back <cm>  -> uses visual ODOMETRY (/odom) for distance
  rotate <deg> [r|l]  -> uses the D555 GYRO for angle (visual odom is blind to
                         in-place rotation on this floor; the gyro is not)

The node is spun on a DEDICATED THREAD so the 200Hz IMU is never dropped (a
throttled main loop under-integrates the gyro and under-reads rotation).
Feeds the ESP32 500ms watchdog at 10Hz and always sends an explicit stop.

Run INSIDE the isaac_ros container (ROS 2 sourced, ROS_DOMAIN_ID=0):
    straight <cm> [speed]     forward until odom moves <cm>
    back <cm> [speed]         backward
    rotate <deg> [right|left] turn in place until the GYRO reads <deg> (default left)

WARNING: the rover is tethered — keep rotate angles small or the wires tangle.
"""
import math, sys, threading, time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

VX = 0.15      # m/s DEFAULT straight cruise (slow/careful; override as 3rd arg)
WZ = 1.5       # rad/s rotate command (moderate; keep angles small — wire tangle)
TIMEOUT = 25.0 # s hard cap
# Straight-line odom->real calibration (reflective-floor odom under-reads+coast):
#   CAL 1.20 -> asked 80 got 85 (over) / asked 120 got 115; asked 60 got ~57.
CAL = 1.20
# Gyro under-reads real rotation (~28%): consistent scale from the slightly
# tilted camera/IMU mount (front camera weight) + a little coast. Measured
# 2026-07-18: gyro 86 -> real 120 (right) / 110 (left). real ~= ROT_CAL * gyro.
ROT_CAL = 1.30
IMU_TOPIC = "/camera/camera0/motion/sample"
# base_link <- camera0_motion_optical_frame rotation (from live TF, xyzw).
Q_BASE_OPT = (-0.5, 0.5, -0.5, 0.5)


def quat_rotate(q, v):
    x, y, z, w = q
    qv = np.array([x, y, z]); vv = np.array(v, dtype=float)
    t = 2.0 * np.cross(qv, vv)
    return vv + w * t + np.cross(qv, t)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    amt = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    arg3 = sys.argv[3] if len(sys.argv) > 3 else None
    if mode not in ("straight", "back", "rotate") or amt <= 0:
        print(__doc__); sys.exit(1)
    is_rot = (mode == "rotate")
    speed = float(arg3) if (arg3 and not is_rot) else VX
    rot_right = bool(is_rot and arg3 and arg3.lower().startswith("r"))

    rclpy.init()
    n = rclpy.create_node("drive_test")

    pose = {}
    n.create_subscription(Odometry, "/odom",
                          lambda m: pose.update(x=m.pose.pose.position.x,
                                                y=m.pose.pose.position.y),
                          qos_profile_sensor_data)

    # gyro-integrated yaw (rad) at the IMU's full 200Hz, self-consistent stamps.
    gyro = {"yaw": 0.0, "t": None}
    def imu_cb(m):
        w = quat_rotate(Q_BASE_OPT, [m.angular_velocity.x,
                                     m.angular_velocity.y, m.angular_velocity.z])
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if gyro["t"] is not None:
            dt = ts - gyro["t"]
            if 0.0 < dt < 0.2:
                gyro["yaw"] += float(w[2]) * dt
        gyro["t"] = ts
    imu_qos = QoSProfile(depth=200); imu_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    n.create_subscription(Imu, IMU_TOPIC, imu_cb, imu_qos)

    pub = n.create_publisher(Twist, "/cmd_vel", 10)

    # spin on a dedicated thread: IMU callbacks run at full rate, never dropped.
    ex = rclpy.executors.SingleThreadedExecutor(); ex.add_node(n)
    spin_thread = threading.Thread(target=ex.spin, daemon=True); spin_thread.start()

    ready = (lambda: gyro["t"] is not None) if is_rot else (lambda: "x" in pose)
    t = time.time()
    while time.time() - t < 6 and (not ready() or pub.get_subscription_count() == 0):
        time.sleep(0.05)
    if not ready():
        print("NO feedback (gyro/odom) — cannot close the loop"); rclpy.shutdown(); return
    if pub.get_subscription_count() == 0:
        print("WARNING: no /cmd_vel subscriber (rover not connected?)")

    if is_rot:
        target = math.radians(amt) / ROT_CAL; unit = "rad"
        cruise, vmin, slow = WZ, 0.7, math.radians(25)
        y_start = gyro["yaw"]
        print(f"rotate {amt}deg {'RIGHT' if rot_right else 'LEFT'} -> gyro feedback, "
              f"target {math.degrees(target):.0f}deg (ROT_CAL={ROT_CAL}, {TIMEOUT}s cap)")
    else:
        target = amt / 100.0 / CAL; unit = "m"
        cruise, vmin, slow = speed, 0.10, 0.15
        x0, y0 = pose["x"], pose["y"]
        print(f"{mode} {amt}cm -> odom feedback, target {target*100:.0f}cm odom (CAL={CAL})")
    sign = (-1.0 if rot_right else 1.0) if is_rot else (-1.0 if mode == "back" else 1.0)
    stop_at = target - (math.radians(4) if is_rot else 0.03)

    achieved = 0.0
    cmd = Twist(); t0 = time.time(); last_pub = 0.0
    while time.time() - t0 < TIMEOUT:
        achieved = abs(gyro["yaw"] - y_start) if is_rot \
            else math.hypot(pose["x"] - x0, pose["y"] - y0)
        if achieved >= stop_at:
            break
        remaining = target - achieved
        spd = cruise if remaining > slow else max(vmin, cruise * remaining / slow)
        now = time.time()
        if now - last_pub >= 0.1:          # 10Hz -> feeds the 500ms watchdog
            if is_rot: cmd.angular.z = sign * spd
            else:      cmd.linear.x = sign * spd
            pub.publish(cmd); last_pub = now
        time.sleep(0.01)

    stop = Twist()
    for _ in range(6):
        pub.publish(stop); time.sleep(0.03)

    dt = time.time() - t0
    if is_rot:
        print(f"  DONE: gyro turned {math.degrees(achieved):.1f} deg (target {amt}) in {dt:.1f}s")
    else:
        print(f"  DONE: odom moved {achieved*100:.1f} cm (odom target {target*100:.0f}) "
              f"in {dt:.1f}s -> ~{achieved*100*CAL:.0f}cm real expected")
    if achieved < target * 0.1:
        print("  >>> barely moved: torque stall, or feedback (odom/gyro) dead")
    ex.shutdown()                    # unblock spin()
    spin_thread.join(timeout=2.0)    # let the thread exit before context teardown
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
