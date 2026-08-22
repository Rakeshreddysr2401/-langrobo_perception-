import rclpy, math, time
from rclpy.qos import qos_profile_sensor_data
from nvblox_msgs.msg import DistanceMapSlice
from nav_msgs.msg import Odometry
rclpy.init(); n = rclpy.create_node("em"); g = {}
n.create_subscription(DistanceMapSlice, "/nvblox_node/static_map_slice", lambda m: g.__setitem__("s", m), qos_profile_sensor_data)
n.create_subscription(Odometry, "/odom", lambda m: g.__setitem__("o", m), qos_profile_sensor_data)
e = time.time() + 20
while time.time() < e and len(g) < 2:
    rclpy.spin_once(n, timeout_sec=0.1)
s = g["s"]; o = g["o"].pose.pose
px, py = o.position.x, o.position.y
res = s.resolution; ox = s.origin.x; oy = s.origin.y; uv = s.unknown_value
print("  slice %dx%d @ %.3f  origin (%.2f, %.2f)  unknown_value %.0f"
      % (s.width, s.height, res, ox, oy, uv))
print("  rover (%+.2f, %+.2f)" % (px, py))
print()
def dist(x, y):
    i = int((x-ox)/res); j = int((y-oy)/res)
    if not (0 <= i < s.width and 0 <= j < s.height): return None
    v = s.data[j*s.width+i]
    return v
print("  ESDF distance-to-obstacle, metres. U = unknown, X = off-slice")
print("  (cost = 254*(1 - d/1.0), so d>=1.0 is FREE, d~0 is LETHAL)")
print()
hdr = "        "
for i in range(-8, 9, 2):
    hdr += "%6.2f" % (i*0.05)
print(hdr + "   <- x offset (m)")
for j in range(8, -9, -2):
    row = "  %+5.2f " % (j*0.05)
    for i in range(-8, 9, 2):
        d = dist(px+i*0.05, py+j*0.05)
        if d is None: row += "     X"
        elif d == uv: row += "     U"
        else: row += "%6.2f" % d
    print(row)
print()
for r, lbl in [(0.0, "at the rover"), (0.5, "0.5 m ahead"), (1.0, "1.0 m ahead"), (1.5, "1.5 m ahead")]:
    d = dist(px+r, py)
    if d is None: print("  %-14s off-slice" % lbl)
    elif d == uv: print("  %-14s UNKNOWN" % lbl)
    else:
        cost = 0 if d >= 1.0 else int(254*(1-d/1.0))
        print("  %-14s distance %.2f m  -> nvblox layer cost %d" % (lbl, d, cost))
rclpy.shutdown()
