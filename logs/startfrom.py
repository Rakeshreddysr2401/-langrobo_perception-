import rclpy, math, time
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = Node("sf"); g = {}
n.create_subscription(Odometry, "/odom", lambda m: g.__setitem__("o", m), qos_profile_sensor_data)
e = time.time() + 15
while time.time() < e and "o" not in g:
    rclpy.spin_once(n, timeout_sec=0.1)
o = g["o"].pose.pose
px, py = o.position.x, o.position.y
ac = ActionClient(n, ComputePathToPose, "compute_path_to_pose")
ac.wait_for_server(timeout_sec=15)

def plan(sx, sy, gx, gy, label):
    st = PoseStamped(); st.header.frame_id = "odom"
    st.pose.position.x = sx; st.pose.position.y = sy; st.pose.orientation.w = 1.0
    gp = PoseStamped(); gp.header.frame_id = "odom"
    gp.pose.position.x = gx; gp.pose.position.y = gy; gp.pose.orientation.w = 1.0
    m = ComputePathToPose.Goal(); m.goal = gp; m.start = st; m.use_start = True
    f = ac.send_goal_async(m)
    rclpy.spin_until_future_complete(n, f, timeout_sec=15)
    h = f.result()
    if h is None or not h.accepted:
        print("  %-46s not accepted" % label); return
    rf = h.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=25)
    r = rf.result()
    if r is None:
        print("  %-46s no result" % label); return
    ec = r.result.error_code
    v = "OK" if ec == 0 else ("FAIL(%d)" % ec)
    print("  %-46s %-10s poses=%d" % (label, v, len(r.result.path.poses)))

print("  rover is at (%+.2f, %+.2f) -- an INSCRIBED cell" % (px, py))
print()
print("  A: start AT the rover (what nav2 does normally)")
plan(px, py, px+1.0, py, "     from rover  -> 1.0 m ahead")
print()
print("  B: start 20 cm into the FREE patch just behind it")
fy = py - 0.20
plan(px, fy, px+1.0, fy, "     from free cell -> 1.0 m ahead")
plan(px, fy, px+2.0, fy, "     from free cell -> 2.0 m ahead")
plan(px, fy, px+3.0, fy, "     from free cell -> 3.0 m ahead")
rclpy.shutdown()
