"""How much z does cuVSLAM invent per metre driven?

Stationary it is exactly 0.000 (measured 90 s), so the error is created by
MOTION. If z/distance is a constant, the cause is a fixed geometric angle --
camera pitch, or an extrinsic. If it grows with turning rather than with
distance, the cause is rotational.

Drive with the teleop while this runs. Straight lines first, then turns.
"""
import math, time
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
n = rclpy.create_node("zdrift")
s = {"path": 0.0, "yawsum": 0.0, "prev": None, "z": 0.0, "x": 0.0, "y": 0.0}

def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def cb(m):
    p = m.pose.pose.position
    th = yaw_of(m.pose.pose.orientation)
    if s["prev"] is not None:
        px, py, pth = s["prev"]
        s["path"] += math.hypot(p.x-px, p.y-py)
        s["yawsum"] += abs((th-pth+math.pi) % (2*math.pi) - math.pi)
    s["prev"] = (p.x, p.y, th)
    s["x"], s["y"], s["z"] = p.x, p.y, p.z

n.create_subscription(Odometry, "/vo/odom", cb, qos_profile_sensor_data)

DUR = 120.0
print("  DRIVE THE ROVER with the teleop for the next %ds." % DUR)
print("  Straight lines first if you can, then some turns.")
print()
print("      t     driven   turned      z       z per m driven   z per rad turned")
print("   " + "-"*72)
t0 = time.time(); last = 0.0
while rclpy.ok() and time.time()-t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.time()-last < 6.0:
        continue
    last = time.time()
    pm = s["z"]/s["path"] if s["path"] > 0.05 else 0.0
    pr = s["z"]/s["yawsum"] if s["yawsum"] > 0.05 else 0.0
    print("   %5.0fs  %6.2f m  %6.2f rad  %+7.3f m   %+9.4f       %+9.4f"
          % (time.time()-t0, s["path"], s["yawsum"], s["z"], pm, pr))

print()
print("  driven %.2f m, turned %.2f rad, z ended %+.3f m" % (s["path"], s["yawsum"], s["z"]))
if s["path"] > 0.2:
    print("  z per metre driven : %+.4f  (= sin of a %.1f deg pitch error)"
          % (s["z"]/s["path"], math.degrees(math.asin(max(-1, min(1, s["z"]/s["path"]))))))
if s["yawsum"] > 0.2:
    print("  z per radian turned: %+.4f" % (s["z"]/s["yawsum"]))
rclpy.shutdown()
