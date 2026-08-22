import rclpy, math, time, sys
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

QT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
rclpy.init()
n = Node("drive1m")
g = {}
n.create_subscription(Odometry, "/odom", lambda m: g.__setitem__("o", m), qos_profile_sensor_data)
n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: g.__setitem__("c", m), QT)
cmds = []
n.create_subscription(Twist, "/cmd_vel", lambda m: cmds.append((time.time(), m.linear.x, m.angular.z)), 10)
e = time.time() + 20
while time.time() < e and len(g) < 2:
    rclpy.spin_once(n, timeout_sec=0.1)

o = g["o"].pose.pose
sx, sy = o.position.x, o.position.y
q = o.orientation
th = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
gx, gy = sx + 1.0*math.cos(th), sy + 1.0*math.sin(th)
print("  start (%+.3f, %+.3f)  heading %+.1f deg" % (sx, sy, math.degrees(th)))
print("  goal  (%+.3f, %+.3f)  1.00 m straight ahead" % (gx, gy))
print()

# ---- plan first, and refuse to drive a path that touches blocked cells ----
pc = ActionClient(n, ComputePathToPose, "compute_path_to_pose")
pc.wait_for_server(timeout_sec=15)
gp = PoseStamped(); gp.header.frame_id = "odom"
gp.pose.position.x = gx; gp.pose.position.y = gy
gp.pose.orientation.z = q.z; gp.pose.orientation.w = q.w
m = ComputePathToPose.Goal(); m.goal = gp; m.use_start = False
f = pc.send_goal_async(m); rclpy.spin_until_future_complete(n, f, timeout_sec=15)
h = f.result()
rf = h.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=25)
res = rf.result().result
if res.error_code != 0 or not res.path.poses:
    print("  PLAN FAILED (error %d) — not driving." % res.error_code)
    rclpy.shutdown(); sys.exit(1)

c = g["c"]; r = c.info.resolution
worst = 0
for p in res.path.poses:
    x, y = p.pose.position.x, p.pose.position.y
    i = int((x-c.info.origin.position.x)/r); j = int((y-c.info.origin.position.y)/r)
    if 0 <= i < c.info.width and 0 <= j < c.info.height:
        v = c.data[j*c.info.width+i]
        if v > worst: worst = v
print("  planned path: %d poses, worst cell cost on it = %d" % (len(res.path.poses), worst))
if worst >= 99:
    print("  path touches a BLOCKED cell — refusing to drive.")
    rclpy.shutdown(); sys.exit(1)
print("  path is clear. driving.")
print()

# ---- drive ----
ac = ActionClient(n, NavigateToPose, "navigate_to_pose")
ac.wait_for_server(timeout_sec=15)
ng = NavigateToPose.Goal(); ng.pose = gp
fut = ac.send_goal_async(ng)
rclpy.spin_until_future_complete(n, fut, timeout_sec=15)
gh = fut.result()
if gh is None or not gh.accepted:
    print("  goal REJECTED"); rclpy.shutdown(); sys.exit(1)
print("  goal accepted — moving")
print()
print("     t      travelled   remaining   cmd_vx   cmd_wz")
print("   " + "-"*54)
resf = gh.get_result_async()
t0 = time.time()
last = 0
while rclpy.ok() and not resf.done() and time.time()-t0 < 60:
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.time()-last < 0.7:
        continue
    last = time.time()
    o = g["o"].pose.pose
    px, py = o.position.x, o.position.y
    trav = math.hypot(px-sx, py-sy)
    rem = math.hypot(gx-px, gy-py)
    vx = wz = 0.0
    if cmds:
        vx, wz = cmds[-1][1], cmds[-1][2]
    print("   %5.1fs   %6.1f cm   %6.1f cm   %+6.2f   %+6.2f"
          % (time.time()-t0, trav*100, rem*100, vx, wz))
rclpy.spin_until_future_complete(n, resf, timeout_sec=10)
o = g["o"].pose.pose
px, py = o.position.x, o.position.y
q2 = o.orientation
th2 = math.atan2(2*(q2.w*q2.z + q2.x*q2.y), 1 - 2*(q2.y*q2.y + q2.z*q2.z))
print()
print("  ── result " + "-"*46)
print("   travelled       %.1f cm   (asked for 100)" % (math.hypot(px-sx, py-sy)*100))
print("   stopped         %.1f cm from the goal   (tolerance 15)" % (math.hypot(gx-px, gy-py)*100))
print("   heading change  %+.1f deg" % (math.degrees((th2-th+math.pi) % (2*math.pi) - math.pi)))
print("   cmd_vel msgs    %d" % len(cmds))
if cmds:
    print("   peak commanded  vx %+.2f m/s   wz %+.2f rad/s"
          % (max(abs(x[1]) for x in cmds), max(abs(x[2]) for x in cmds)))
    still = [x for x in cmds[-5:]]
    print("   last command    vx %+.2f  wz %+.2f  (should be 0, 0)" % (still[-1][1], still[-1][2]))
rclpy.shutdown()
