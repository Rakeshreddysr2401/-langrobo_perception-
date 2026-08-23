"""What commanded wz actually rotates this rover?

RPP's rotate-to-heading clamps its output to curr_speed +/- max_angular_accel*dt.
From a standstill that is 0.15 rad/s, and if 0.15 does not break skid-steer
scrub the rover never moves, so curr_speed stays 0 and the clamp never rises.
A deadlock. This measures where the floor actually is.

Spins in place, briefly, at increasing commanded wz. Stops between each step.
"""
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
n = Node("wzsweep")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
g = {}
n.create_subscription(Odometry, "/odom", lambda m: g.__setitem__("o", m), qos_profile_sensor_data)
e = time.time() + 15
while time.time() < e and "o" not in g:
    rclpy.spin_once(n, timeout_sec=0.1)

def yaw():
    q = g["o"].pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def stop():
    t = Twist()
    for _ in range(12):
        pub.publish(t); rclpy.spin_once(n, timeout_sec=0.05)

print("  commanded    measured      moved?")
print("  " + "-"*42)
results = []
for wz in [0.15, 0.3, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0]:
    stop(); time.sleep(0.4)
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.02)
    y0 = yaw(); t0 = time.time()
    t = Twist(); t.angular.z = float(wz)
    while time.time() - t0 < 1.6:
        pub.publish(t)
        rclpy.spin_once(n, timeout_sec=0.02)
    dt = time.time() - t0
    y1 = yaw()
    stop()
    d = (y1 - y0 + math.pi) % (2*math.pi) - math.pi
    rate = abs(d)/dt
    moved = rate > 0.05
    results.append((wz, rate, moved))
    print("  %5.2f rad/s   %5.2f rad/s   %s" % (wz, rate, "YES" if moved else "no -- did not break scrub"))
    time.sleep(0.3)
stop()
print()
first = next((w for w, r, m in results if m), None)
if first:
    print("  breakaway: the lowest commanded wz that actually rotated is %.2f rad/s" % first)
    print("  RPP's first step from standstill is max_angular_accel * 0.1 s,")
    print("  so max_angular_accel must be at least %.1f rad/s^2 to clear it." % (first/0.1))
else:
    print("  nothing rotated at any tested wz -- something else is wrong")
rclpy.shutdown()
