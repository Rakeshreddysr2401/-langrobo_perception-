import rclpy, math, time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
Q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
rclpy.init(); n = rclpy.create_node("around"); g = {}
n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: g.__setitem__("c", m), Q)
n.create_subscription(Odometry, "/odom", lambda m: g.__setitem__("o", m), qos_profile_sensor_data)
e = time.time() + 20
while time.time() < e and len(g) < 2:
    rclpy.spin_once(n, timeout_sec=0.1)
m = g["c"]; o = g["o"].pose.pose
res = m.info.resolution; ox = m.info.origin.position.x; oy = m.info.origin.position.y
W = m.info.width; H = m.info.height
px, py = o.position.x, o.position.y
si = int((px-ox)/res); sj = int((py-oy)/res)
print("  rover at (%+.2f,%+.2f)  cell (%d,%d)" % (px, py, si, sj))
v = m.data[sj*W+si]
name = {-1: "UNKNOWN", 99: "INSCRIBED", 100: "LETHAL"}.get(v, "cost %d" % v)
print("  the cell the rover is standing in:  %s" % name)
if v == 99 or v == 100 or v < 0:
    print("  ^^ NavFn cannot expand from a blocked start. This alone fails every goal.")
print()
print("  S rover   . free   : traversable   # INSCRIBED/LETHAL   ? unknown")
print()
for j in range(sj+11, sj-12, -1):
    row = ""
    for i in range(si-30, si+31):
        if (i, j) == (si, sj): row += "S"; continue
        if not (0 <= i < W and 0 <= j < H): row += " "; continue
        vv = m.data[j*W+i]
        row += "?" if vv < 0 else ("." if vv == 0 else (":" if vv < 99 else "#"))
    print("   " + row)
print()
print("  window is 3.0 m wide, 5 cm per character")
# nearest free cell
best = None
for j in range(max(0,sj-60), min(H,sj+60)):
    for i in range(max(0,si-60), min(W,si+60)):
        vv = m.data[j*W+i]
        if 0 <= vv < 99:
            d = math.hypot((i-si)*res, (j-sj)*res)
            if best is None or d < best[0]: best = (d, i, j, vv)
if best:
    print("  nearest traversable cell: %.2f m away (cost %d)" % (best[0], best[3]))
rclpy.shutdown()
