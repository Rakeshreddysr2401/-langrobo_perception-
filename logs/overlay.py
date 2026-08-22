import rclpy, math, time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
QT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
QV = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
rclpy.init(); n = rclpy.create_node("ov"); g = {}
n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: g.__setitem__("c", m), QT)
n.create_subscription(OccupancyGrid, "/nvblox_node/static_occupancy_grid", lambda m: g.__setitem__("m", m), QV)
n.create_subscription(Odometry, "/odom", lambda m: g.__setitem__("o", m), qos_profile_sensor_data)
e = time.time() + 25
while time.time() < e and len(g) < 3:
    rclpy.spin_once(n, timeout_sec=0.1)
o = g["o"].pose.pose; px, py = o.position.x, o.position.y

def at(m, x, y):
    res = m.info.resolution
    i = int((x-m.info.origin.position.x)/res); j = int((y-m.info.origin.position.y)/res)
    if not (0 <= i < m.info.width and 0 <= j < m.info.height): return None
    return m.data[j*m.info.width+i]

print("  RAW nvblox map (left)      vs      nav2 costmap (right)")
print("  . free  # occupied  ? unknown      . free  : trav  # blocked  ? unknown")
print()
for j in range(11, -12, -1):
    L = ""; R = ""
    for i in range(-22, 23):
        x = px + i*0.05; y = py + j*0.05
        if i == 0 and j == 0: L += "S"; R += "S"; continue
        a = at(g["m"], x, y)
        L += "?" if a is None or a < 0 else ("." if a < 50 else "#")
        b = at(g["c"], x, y)
        R += "?" if b is None or b < 0 else ("." if b == 0 else (":" if b < 99 else "#"))
    print("   " + L + "   " + R)
print()
print("  each window 2.25 m wide")
# count in the raw map above the rover
occ = 0; fre = 0; unk = 0
for j in range(1, 25):
    for i in range(-22, 23):
        a = at(g["m"], px+i*0.05, py+j*0.05)
        if a is None or a < 0: unk += 1
        elif a < 50: fre += 1
        else: occ += 1
print("  in the raw map, the 2.25 x 1.2 m patch AHEAD of the rover:")
print("    occupied %d   free %d   unknown %d" % (occ, fre, unk))
rclpy.shutdown()
