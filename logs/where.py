"""Where CAN it go? Reachable points around the rover, and the turn each needs."""
import math, time
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
QT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
rclpy.init(); n = rclpy.create_node("where"); g = {}
n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: g.__setitem__("c", m), QT)
n.create_subscription(Odometry, "/odom", lambda m: g.__setitem__("o", m), qos_profile_sensor_data)
e = time.time() + 20
while time.time() < e and len(g) < 2:
    rclpy.spin_once(n, timeout_sec=0.1)
c = g["c"]; o = g["o"].pose.pose
px, py = o.position.x, o.position.y
q = o.orientation
th = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
r = c.info.resolution
def cost(x, y):
    i = int((x-c.info.origin.position.x)/r); j = int((y-c.info.origin.position.y)/r)
    if not (0 <= i < c.info.width and 0 <= j < c.info.height): return None
    return c.data[j*c.info.width+i]
print("  rover at (%+.2f, %+.2f) facing %+.0f deg" % (px, py, math.degrees(th)))
print()
print("   direction     dist   goal            turn needed   status")
print("   " + "-"*62)
for a in range(0, 360, 30):
    ar = math.radians(a)
    best = None
    for d in [x*0.1 for x in range(20, 5, -1)]:
        x, y = px + d*math.cos(ar), py + d*math.sin(ar)
        v = cost(x, y)
        if v is not None and 0 <= v < 99:
            # the whole ray must be clear too, not just the end point
            if all((lambda w: w is not None and 0 <= w < 99)(cost(px+s*math.cos(ar), py+s*math.sin(ar)))
                   for s in [k*0.1 for k in range(1, int(d*10)+1)]):
                best = (d, x, y); break
    turn = math.degrees((ar - th + math.pi) % (2*math.pi) - math.pi)
    if best:
        print("   %4d deg     %.2f m  (%+.2f,%+.2f)   %+4.0f deg      OK"
              % (a, best[0], best[1], best[2], turn))
    else:
        print("   %4d deg       --                     %+4.0f deg      blocked" % (a, turn))
rclpy.shutdown()
