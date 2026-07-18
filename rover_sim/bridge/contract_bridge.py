#!/usr/bin/env python3
"""rover_sim contract bridge — the python half of the D555/ESP32
impersonation (SIM_REAL_PARITY.md §2, §3).

Images flow C++ end-to-end (ros_gz parameter_bridge remaps gz topics straight
onto the contract names; sensors carry gz_frame_id = contract optical frames;
gz runs with --initial-sim-time so stamps track the wall clock). A python hop
for 896x504 images capped at ~5Hz on the laptop — so python only handles:

  role "depth"   /rover_sim/depth32 (32FC1 m, bridged)
                 -> /camera/camera0/depth/image_rect_raw (16UC1 mm,
                    header preserved so it pairs with infra1/camera_info)
  role "motion"  /rover_sim/imu -> /camera/camera0/motion/sample
                    (gz body axes -> RealSense optical axes, wall stamp)
                 /cmd_vel -> rover_firmware.ino emulation (PWM deadband,
                    500ms watchdog) -> /rover_sim/drive_cmd (gz DiffDrive)
                 /tf_static  (the captured REAL camera frame tree)
                 /rover_sim/status  (sim-mode marker, 1Hz JSON)

Intrinsics divergence (documented): sim camera_info is gz-derived (centered
principal point, fx==fy) — real values live in d555_contract/ for reference.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

# rover_firmware.ino constants — keep byte-identical to the Pi5 copy
PWM_MIN = 130.0 / 255.0     # ~51% duty floor (L298N static friction)
MAX_LINEAR_VEL = 0.30       # m/s at full PWM
MAX_ANGULAR_VEL = 2.0       # firmware's (uncalibrated) wz normalizer
DEAD_STICK = 0.02           # below this, wheel is parked
CMD_TIMEOUT_S = 0.5         # firmware watchdog
TRACK = 0.24                # sim model wheel_separation (model.sdf)


def firmware_twist(vx: float, wz: float) -> Twist:
    """Port of driveFromTwist() + setMotor(): Twist -> what the wheels DO."""
    linear_pct = max(-1.0, min(1.0, vx / MAX_LINEAR_VEL))
    angular_pct = max(-1.0, min(1.0, wz / MAX_ANGULAR_VEL))
    left = max(-1.0, min(1.0, linear_pct - angular_pct))
    right = max(-1.0, min(1.0, linear_pct + angular_pct))

    def duty(v):
        if abs(v) <= DEAD_STICK:
            return 0.0
        d = PWM_MIN + abs(v) * (1.0 - PWM_MIN)
        return d if v > 0 else -d

    v_l = duty(left) * MAX_LINEAR_VEL
    v_r = duty(right) * MAX_LINEAR_VEL
    out = Twist()
    out.linear.x = (v_l + v_r) / 2.0
    out.angular.z = (v_r - v_l) / TRACK
    return out


class ContractBridge(Node):
    def __init__(self, contract_dir: Path, role: str):
        super().__init__(f"rover_sim_bridge_{role}")
        q = qos_profile_sensor_data
        self._counts = {}

        if role in ("depth", "all"):
            self._pub_depth = self.create_publisher(
                Image, "/camera/camera0/depth/image_rect_raw", q)
            self.create_subscription(Image, "/rover_sim/depth32", self._on_depth, q)
            self._counts["depth"] = 0

        if role in ("motion", "all"):
            self._pub_imu = self.create_publisher(
                Imu, "/camera/camera0/motion/sample", q)
            self._pub_status = self.create_publisher(String, "/rover_sim/status", 10)
            self._pub_drive = self.create_publisher(Twist, "/rover_sim/drive_cmd", 10)
            self.create_subscription(Imu, "/rover_sim/imu", self._on_imu, q)
            self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
            self._counts.update(imu=0, cmd=0)

            # the captured REAL static frame tree, re-stamped
            self._static_bc = StaticTransformBroadcaster(self)
            tfs = []
            doc = next(yaml.safe_load_all(
                (contract_dir / "tf_static_camera0.yaml").read_text()))
            for t in doc["transforms"]:
                m = TransformStamped()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = t["header"]["frame_id"]
                m.child_frame_id = t["child_frame_id"]
                tr, rot = t["transform"]["translation"], t["transform"]["rotation"]
                m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = \
                    float(tr["x"]), float(tr["y"]), float(tr["z"])
                m.transform.rotation.x, m.transform.rotation.y = float(rot["x"]), float(rot["y"])
                m.transform.rotation.z, m.transform.rotation.w = float(rot["z"]), float(rot["w"])
                tfs.append(m)
            self._static_bc.sendTransform(tfs)

            self._last_cmd_t = 0.0
            self._stopped = True
            self.create_timer(0.05, self._watchdog)  # 20Hz, like the fw loop
            self.create_timer(1.0, self._status)

        self.get_logger().info(f"contract bridge up (role: {role})")

    def _on_depth(self, m):
        # 32FC1 metres -> 16UC1 millimetres, 0 = invalid (RealSense
        # convention). Header preserved: same stamp/frame as the gz render,
        # so it pairs with the bridged infra1 image + camera_infos.
        f = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
        mm = np.where(np.isfinite(f) & (f > 0.0),
                      np.clip(f * 1000.0, 0, 65535), 0).astype(np.uint16)
        out = Image(header=m.header, height=m.height, width=m.width,
                    encoding="16UC1", is_bigendian=0, step=m.width * 2,
                    data=mm.tobytes())
        self._pub_depth.publish(out)
        self._counts["depth"] += 1

    def _on_imu(self, m):
        # gz body frame (x fwd, y left, z up) -> RealSense optical frame
        # (x right, y down, z fwd): (x,y,z)_opt = (-y, -z, x)_body
        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "camera0_motion_optical_frame"
        out.angular_velocity.x = -m.angular_velocity.y
        out.angular_velocity.y = -m.angular_velocity.z
        out.angular_velocity.z = m.angular_velocity.x
        out.linear_acceleration.x = -m.linear_acceleration.y
        out.linear_acceleration.y = -m.linear_acceleration.z
        out.linear_acceleration.z = m.linear_acceleration.x
        out.orientation_covariance[0] = -1.0  # no orientation, like the D555
        self._pub_imu.publish(out)
        self._counts["imu"] += 1

    def _on_cmd_vel(self, m):
        self._last_cmd_t = time.monotonic()
        self._counts["cmd"] += 1
        self._stopped = False
        self._pub_drive.publish(firmware_twist(m.linear.x, m.angular.z))

    def _watchdog(self):
        if not self._stopped and time.monotonic() - self._last_cmd_t > CMD_TIMEOUT_S:
            self._pub_drive.publish(Twist())  # fw watchdog: silence = stop
            self._stopped = True

    def _status(self):
        self._pub_status.publish(String(data=json.dumps(
            {"mode": "sim", "counts": self._counts})))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--contract", required=True,
                    help="dir with the captured d555_contract yamls")
    ap.add_argument("--role", default="all", choices=["all", "depth", "motion"])
    args = ap.parse_args()
    rclpy.init()
    rclpy.spin(ContractBridge(Path(args.contract), args.role))


if __name__ == "__main__":
    main()
