import rclpy, math, time
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from collections import Counter

Q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
rclpy.init()
n = Node("plantest")
g = {}
n.create_subscription(Odometry, "/odom", lambda m: g.__setitem__("o", m), qos_profile_sensor_data)
n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: g.__setitem__("c", m), Q)
e = time.time() + 20
while time.time() < e and len(g) < 2:
    rclpy.spin_once(n, timeout_sec=0.1)

c = Counter(g["c"].data); tot = len(g["c"].data)
trav = sum(v for k, v in c.items() if 1 <= k <= 98)
print("  costmap now:  unknown %.1f%%  free %.1f%%  traversable %.1f%%"
      % (c[-1]*100/tot, c[0]*100/tot, trav*100/tot))
print("                INSCRIBED %.1f%%  LETHAL %.1f%%   <- these block NavFn"
      % (c[99]*100/tot, c[100]*100/tot))
print("  costmap was:  unknown 57.7%  free 5.8%  traversable 9.4%")
print("                INSCRIBED 21.8%  LETHAL 5.2%")
print()

o = g["o"].pose.pose
px, py = o.position.x, o.position.y
q = o.orientation
th = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
ac = ActionClient(n, ComputePathToPose, "compute_path_to_pose")
ac.wait_for_server(timeout_sec=15)

def tryd(dx, dy, label):
    gp = PoseStamped()
    gp.header.frame_id = "odom"
    gp.pose.position.x = px + dx
    gp.pose.position.y = py + dy
    gp.pose.orientation.w = 1.0
    m = ComputePathToPose.Goal(); m.goal = gp; m.use_start = False
    f = ac.send_goal_async(m)
    rclpy.spin_until_future_complete(n, f, timeout_sec=15)
    h = f.result()
    if h is None or not h.accepted:
        print("  %-26s not accepted" % label); return
    rf = h.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=25)
    r = rf.result()
    if r is None:
        print("  %-26s no result" % label); return
    ec = r.result.error_code
    verdict = "OK" if ec == 0 else ("FAIL(%d)" % ec)
    print("  %-26s %-10s poses=%d" % (label, verdict, len(r.result.path.poses)))

print("  rover (%+.2f,%+.2f) heading %+.0f deg" % (px, py, math.degrees(th)))
print()
for d in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]:
    tryd(d*math.cos(th), d*math.sin(th), "%.1f m straight ahead" % d)
print()
for lbl, dx, dy in [("1.5 m +x", 1.5, 0), ("1.5 m -x", -1.5, 0),
                    ("1.5 m +y", 0, 1.5), ("1.5 m -y", 0, -1.5)]:
    tryd(dx, dy, lbl)
rclpy.shutdown()
