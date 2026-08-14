import json, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


def yaw_of(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


rclpy.init()
n = Node("showall")
d = {}
cnt = {}


def cb(k):
    def f(m):
        d[k] = m
        cnt[k] = cnt.get(k, 0) + 1
    return f


n.create_subscription(Odometry, "/vo/odom", cb("odom"), qos_profile_sensor_data)
n.create_subscription(String, "/vo/status", cb("status"), 10)
n.create_subscription(Imu, "/gyro/base", cb("imu_base"), qos_profile_sensor_data)
n.create_subscription(Imu, "/camera/camera0/imu", cb("imu_raw"), qos_profile_sensor_data)
n.create_subscription(Vector3, "/wheel_state", cb("wheel"), qos_profile_sensor_data)

t0 = time.time()
while time.time() - t0 < 6.0:
    rclpy.spin_once(n, timeout_sec=0.1)
el = time.time() - t0

W = 74
print("=" * W)
print("  EVERY SENSOR, RIGHT NOW".center(W))
print("=" * W)

print("\n── cuVSLAM  /vo/odom ─────────────────────────────────────────────")
if "odom" in d:
    m = d["odom"]
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    print(f"   position     x {p.x*100:+9.2f} cm   y {p.y*100:+9.2f} cm   z {p.z*100:+9.2f} cm")
    print(f"   orientation  yaw {yaw_of(q):+8.3f} deg")
    print(f"   quaternion   x {q.x:+.5f}  y {q.y:+.5f}  z {q.z:+.5f}  w {q.w:+.5f}")
    print(f"   frame        {m.header.frame_id} -> {m.child_frame_id}")
    print(f"   rate         {cnt.get('odom',0)/el:.1f} Hz")
else:
    print("   NOT PUBLISHING")

print("\n── cuVSLAM health  /vo/status ────────────────────────────────────")
if "status" in d:
    s = json.loads(d["status"].data)
    for k in ("rate_hz", "frames", "tracked", "landmarks", "frozen_frames", "healthy"):
        print(f"   {k:<16} {s.get(k)}")
    print("   (landmarks is the real health signal — a dead tracker still")
    print("    returns a pose at a perfect 30 Hz)")
else:
    print("   NOT PUBLISHING")

print("\n── IMU, re-framed into base_link  /gyro/base ─────────────────────")
if "imu_base" in d:
    m = d["imu_base"]
    w, a = m.angular_velocity, m.linear_acceleration
    print(f"   gyro   x {math.degrees(w.x):+8.3f}  y {math.degrees(w.y):+8.3f}  z {math.degrees(w.z):+8.3f}  deg/s")
    print(f"          (z is yaw rate — the only axis used)")
    print(f"   accel  x {a.x:+8.3f}  y {a.y:+8.3f}  z {a.z:+8.3f}  m/s^2")
    print(f"          (z should read about +9.8 = gravity, proving level)")
    print(f"   rate   {cnt.get('imu_base',0)/el:.1f} Hz")
else:
    print("   NOT PUBLISHING")

print("\n── IMU, raw from the camera ──────────────────────────────────────")
if "imu_raw" in d:
    m = d["imu_raw"]
    w, a = m.angular_velocity, m.linear_acceleration
    print(f"   gyro   x {math.degrees(w.x):+8.3f}  y {math.degrees(w.y):+8.3f}  z {math.degrees(w.z):+8.3f}  deg/s")
    print(f"   accel  x {a.x:+8.3f}  y {a.y:+8.3f}  z {a.z:+8.3f}  m/s^2")
    print(f"   frame  {m.header.frame_id}   (optical axes: x right, y DOWN, z fwd)")
    print(f"   rate   {cnt.get('imu_raw',0)/el:.1f} Hz")
else:
    print("   not subscribed / no publisher")

print("\n── wheels  /wheel_state ──────────────────────────────────────────")
if "wheel" in d:
    m = d["wheel"]
    print(f"   velL {m.x:+.4f} m/s   velR {m.y:+.4f} m/s   cmd_vx {m.z:+.4f} m/s")
    print(f"   implied vx {(m.x+m.y)/2:+.4f} m/s   yaw rate "
          f"{math.degrees((m.y-m.x)/0.34):+.3f} deg/s")
    print(f"   rate {cnt.get('wheel',0)/el:.1f} Hz   (want ~20)")
else:
    print("   NOT PUBLISHING")

print("\n" + "=" * W)
print("  FUSED is computed in compare.py, not published as a topic:")
print("     distance from cuVSLAM, heading from the gyro")
print("     see it live with:  ./rover compare --plain")
print("=" * W)
