import rclpy, time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nvblox_msgs.msg import DistanceMapSlice
from nav_msgs.msg import OccupancyGrid, Odometry
QT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
rclpy.init(); n = rclpy.create_node("nums"); g = {}
n.create_subscription(DistanceMapSlice, "/nvblox_node/static_map_slice", lambda m: g.__setitem__("s", m), qos_profile_sensor_data)
n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: g.__setitem__("c", m), QT)
n.create_subscription(Odometry, "/odom", lambda m: g.__setitem__("o", m), qos_profile_sensor_data)
e = time.time() + 25
while time.time() < e and len(g) < 3:
    rclpy.spin_once(n, timeout_sec=0.1)
s = g["s"]; c = g["c"]; o = g["o"].pose.pose
px, py = o.position.x, o.position.y
r = c.info.resolution
ci = int((px-c.info.origin.position.x)/r); cj = int((py-c.info.origin.position.y)/r)

print("  COSTMAP values, 5 cm cells, rover marked [ ]")
print("  (-1 unknown, 0 free, 99 INSCRIBED, 100 LETHAL)")
print()
hdr = "         "
for i in range(-7, 8): hdr += "%5.2f" % (i*0.05)
print(hdr)
for j in range(7, -8, -1):
    row = "  %+5.2f " % (j*0.05)
    for i in range(-7, 8):
        ii, jj = ci+i, cj+j
        v = c.data[jj*c.info.width+ii] if (0 <= ii < c.info.width and 0 <= jj < c.info.height) else None
        cell = "  off" if v is None else ("%5d" % v)
        if i == 0 and j == 0: cell = "[%3s]" % (v if v is not None else "?")
        row += cell
    print(row)
print()
print("  ESDF distance at the same cells")
print()
print(hdr)
for j in range(7, -8, -1):
    row = "  %+5.2f " % (j*0.05)
    for i in range(-7, 8):
        x = px+i*0.05; y = py+j*0.05
        si = int((x-s.origin.x)/s.resolution); sj = int((y-s.origin.y)/s.resolution)
        if not (0 <= si < s.width and 0 <= sj < s.height): row += "  off"; continue
        v = s.data[sj*s.width+si]
        row += "  unk" if v == s.unknown_value else "%5.2f" % v
    print(row)
rclpy.shutdown()
