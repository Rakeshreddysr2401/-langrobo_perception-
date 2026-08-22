import rclpy, time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nvblox_msgs.msg import DistanceMapSlice
from nav_msgs.msg import OccupancyGrid, Odometry
QT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
rclpy.init(); n = rclpy.create_node("sbs"); g = {}
n.create_subscription(DistanceMapSlice, "/nvblox_node/static_map_slice", lambda m: g.__setitem__("s", m), qos_profile_sensor_data)
n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: g.__setitem__("c", m), QT)
n.create_subscription(Odometry, "/odom", lambda m: g.__setitem__("o", m), qos_profile_sensor_data)
e = time.time() + 25
while time.time() < e and len(g) < 3:
    rclpy.spin_once(n, timeout_sec=0.1)
s = g["s"]; c = g["c"]; o = g["o"].pose.pose
px, py = o.position.x, o.position.y

def esdf(x, y):
    i = int((x-s.origin.x)/s.resolution); j = int((y-s.origin.y)/s.resolution)
    if not (0 <= i < s.width and 0 <= j < s.height): return "off"
    v = s.data[j*s.width+i]
    return "unk" if v == s.unknown_value else round(v, 2)

def cm(x, y):
    r = c.info.resolution
    i = int((x-c.info.origin.position.x)/r); j = int((y-c.info.origin.position.y)/r)
    if not (0 <= i < c.info.width and 0 <= j < c.info.height): return "off"
    return c.data[j*c.info.width+i]

print("  same world point, both sources.  expected cost = 254*(1-d) then /2.55 for the topic")
print()
print("   world point        ESDF d    costmap    expected if the layer worked")
print("   " + "-"*66)
for dx, dy in [(0,0), (0.3,0), (0.6,0), (1.0,0), (1.5,0), (2.0,0),
               (0,0.5), (0,-0.5), (-1.0,0), (0,1.0)]:
    x, y = px+dx, py+dy
    d = esdf(x, y); v = cm(x, y)
    if isinstance(d, float):
        raw = 0 if d >= 1.0 else int(254*(1-d/1.0))
        exp = 0 if raw == 0 else (99 if raw == 253 else (100 if raw == 254 else 1+(97*(raw-1))//251))
        exps = "%d  (raw %d)" % (exp, raw)
    else:
        exps = "-"
    lbl = "(%+.1f,%+.1f)" % (dx, dy)
    print("   %-16s %8s %8s    %s" % (lbl, d, v, exps))
print()
print("  costmap legend: -1 unknown, 0 free, 1-98 traversable, 99 INSCRIBED, 100 LETHAL")
rclpy.shutdown()
