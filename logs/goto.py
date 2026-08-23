"""Send the rover to an x,y in the odom frame, and watch it get there.

  goto.py 2.0 1.5          go to (2.0, 1.5)
  goto.py 2.0 1.5 --rel    2.0 m ahead, 1.5 m to the left OF WHERE IT IS NOW

Unmapped floor is allowed -- the planner routes through it and the map fills in
as the rover drives. That is deliberate, and it is also the risk: unknown is not
the same as empty. This rover sees nothing below 10 cm, nothing overhanging
above 24 cm, nothing outside 87 deg and nothing downward at all, so an
autonomous run still needs a human watching it.
"""
import math, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, DurabilityPolicy,
                       ReliabilityPolicy, HistoryPolicy)

if len(sys.argv) < 3:
    print(__doc__); sys.exit(1)
GX_IN, GY_IN = float(sys.argv[1]), float(sys.argv[2])
REL = "--rel" in sys.argv
TIMEOUT = 180.0

QT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
rclpy.init()
n = Node("goto")
g = {}
cmds = []
n.create_subscription(Odometry, "/odom", lambda m: g.__setitem__("o", m), qos_profile_sensor_data)
n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: g.__setitem__("c", m), QT)
n.create_subscription(Twist, "/cmd_vel", lambda m: cmds.append((m.linear.x, m.angular.z)), 10)
e = time.time() + 20
while time.time() < e and len(g) < 2:
    rclpy.spin_once(n, timeout_sec=0.1)
if len(g) < 2:
    print("  no /odom or no costmap -- is ./rover nav running?"); rclpy.shutdown(); sys.exit(1)

# Nothing else may own /cmd_vel. The teleop streams zeros at 10 Hz in MANUAL and
# they cancel nav2's commands; a run under contention stalls and looks like a
# controller fault. Abort rather than warn -- a warning you drive through is not
# a check.
print("  checking nothing else owns /cmd_vel ...")
cmds.clear()
t0 = time.time()
while time.time() - t0 < 3.0:
    rclpy.spin_once(n, timeout_sec=0.1)
if cmds:
    print("  ABORT: something else is publishing /cmd_vel (%d msgs in 3 s)." % len(cmds))
    print("         The teleop is in MANUAL. Switch it to AUTO and re-run.")
    rclpy.shutdown(); sys.exit(1)
print("  /cmd_vel is free.")

o = g["o"].pose.pose
sx, sy = o.position.x, o.position.y
q = o.orientation
th = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
if REL:
    gx = sx + GX_IN*math.cos(th) - GY_IN*math.sin(th)
    gy = sy + GX_IN*math.sin(th) + GY_IN*math.cos(th)
else:
    gx, gy = GX_IN, GY_IN
dist0 = math.hypot(gx-sx, gy-sy)
print()
print("  from (%+.2f, %+.2f) heading %+.0f deg" % (sx, sy, math.degrees(th)))
print("  to   (%+.2f, %+.2f)   %.2f m away" % (gx, gy, dist0))
print()

# IS THE GOAL ITSELF REACHABLE?
#
# NavFn plans to within a tolerance of the goal, so it returns a perfectly good
# path even when the goal is inside an obstacle -- the path simply stops short.
# The controller then drives to the end of it, finds it is not within
# xy_goal_tolerance, and tries again: approach, stall, back up, approach. That
# is nav2's recovery behaviour doing exactly what it should on a goal that can
# never be reached, and it looks like a controller fault.
#
# Measured 2026-08-23: goal (1.20, 0.00) was INSCRIBED, every cell within 20 cm
# of it was blocked, and the rover thrashed for 52 s reaching 42 cm away before
# giving up. Checking the ROUTE is not enough; check the destination.
c0 = g["c"]; r0 = c0.info.resolution
def cost_at(x, y):
    i = int((x-c0.info.origin.position.x)/r0); j = int((y-c0.info.origin.position.y)/r0)
    if not (0 <= i < c0.info.width and 0 <= j < c0.info.height):
        return None
    return c0.data[j*c0.info.width+i]

gv = cost_at(gx, gy)
if gv is None:
    print("  ABORT: the goal is outside the costmap (it covers 12 x 12 m around the rover).")
    rclpy.shutdown(); sys.exit(1)
if gv >= 99:
    what = "inside an obstacle" if gv == 100 else "too close to an obstacle for the rover to fit"
    print("  ABORT: the goal is %s." % what)
    # Offer the nearest place it COULD stand, so the answer is actionable.
    best = None
    for ring in range(1, 41):
        rad = ring*r0
        for a in range(0, 360, 10):
            x = gx + rad*math.cos(math.radians(a)); y = gy + rad*math.sin(math.radians(a))
            v = cost_at(x, y)
            if v is not None and 0 <= v < 99:
                best = (rad, x, y); break
        if best: break
    if best:
        print("         nearest reachable point is %.0f cm away, at (%+.2f, %+.2f)."
              % (best[0]*100, best[1], best[2]))
        print("         try:  goto.py %.2f %.2f" % (best[1], best[2]))
    else:
        print("         no reachable point found within 2 m of it.")
    rclpy.shutdown(); sys.exit(1)

gp = PoseStamped(); gp.header.frame_id = "odom"
gp.pose.position.x = gx; gp.pose.position.y = gy
gp.pose.orientation.z = q.z; gp.pose.orientation.w = q.w

pc = ActionClient(n, ComputePathToPose, "compute_path_to_pose")
if not pc.wait_for_server(timeout_sec=15):
    print("  planner not available -- is ./rover nav running?"); rclpy.shutdown(); sys.exit(1)
m = ComputePathToPose.Goal(); m.goal = gp; m.use_start = False
f = pc.send_goal_async(m); rclpy.spin_until_future_complete(n, f, timeout_sec=15)
h = f.result()
rf = h.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=30)
res = rf.result().result
if res.error_code != 0 or not res.path.poses:
    print("  NO PATH (error %d) -- not driving." % res.error_code)
    print("  208 = NO_VALID_PATH: the goal is unreachable or inside an obstacle.")
    rclpy.shutdown(); sys.exit(1)

c = g["c"]; r = c.info.resolution
worst = 0; unknown = 0
for p in res.path.poses:
    x, y = p.pose.position.x, p.pose.position.y
    i = int((x-c.info.origin.position.x)/r); j = int((y-c.info.origin.position.y)/r)
    if 0 <= i < c.info.width and 0 <= j < c.info.height:
        v = c.data[j*c.info.width+i]
        if v < 0: unknown += 1
        elif v > worst: worst = v
pct = unknown*100//max(len(res.path.poses), 1)
print("  route: %d poses, worst cost %d, %d%% crosses UNMAPPED floor" % (len(res.path.poses), worst, pct))
if worst >= 99:
    print("  route touches a BLOCKED cell -- refusing.")
    rclpy.shutdown(); sys.exit(1)
print("  driving. tap MANUAL on the phone to stop it at any time.")
print()

ac = ActionClient(n, NavigateToPose, "navigate_to_pose")
ac.wait_for_server(timeout_sec=15)
ng = NavigateToPose.Goal(); ng.pose = gp
fut = ac.send_goal_async(ng)
rclpy.spin_until_future_complete(n, fut, timeout_sec=15)
gh = fut.result()
if gh is None or not gh.accepted:
    print("  goal REJECTED"); rclpy.shutdown(); sys.exit(1)

print("      t       here            to go    vx     wz")
print("   " + "-"*52)
resf = gh.get_result_async()
t0 = time.time(); last = 0.0; stuck_since = None; prev = None
while rclpy.ok() and not resf.done() and time.time()-t0 < TIMEOUT:
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.time()-last < 1.0:
        continue
    last = time.time()
    o = g["o"].pose.pose
    px, py = o.position.x, o.position.y
    rem = math.hypot(gx-px, gy-py)
    vx, wz = (cmds[-1] if cmds else (0.0, 0.0))
    print("   %5.1fs   (%+.2f,%+.2f)   %6.1f cm  %+5.2f %+5.2f"
          % (time.time()-t0, px, py, rem*100, vx, wz))
    if prev and math.hypot(px-prev[0], py-prev[1]) < 0.01:
        stuck_since = stuck_since or time.time()
        if time.time()-stuck_since > 25:
            print("   -- not moving for 25 s; nav2 still thinks it is driving.")
            stuck_since = time.time()
    else:
        stuck_since = None
    prev = (px, py)

rclpy.spin_until_future_complete(n, resf, timeout_sec=10)
o = g["o"].pose.pose
px, py = o.position.x, o.position.y
print()
print("  -- result " + "-"*44)
print("   ended at (%+.3f, %+.3f)" % (px, py))
print("   %.1f cm from the goal   (tolerance 5 cm)" % (math.hypot(gx-px, gy-py)*100))
print("   straight-line distance covered: %.1f cm of %.1f cm"
      % (math.hypot(px-sx, py-sy)*100, dist0*100))
rclpy.shutdown()
