import rclpy, time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
QT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
rclpy.init(); n = rclpy.create_node("stale"); got = []
n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: got.append(m), QT)
e = time.time() + 25
while time.time() < e and len(got) < 6:
    rclpy.spin_once(n, timeout_sec=0.1)
print("  received %d costmap messages" % len(got))
if len(got) < 2:
    print("  not enough to compare"); rclpy.shutdown(); raise SystemExit
a, b = got[0], got[-1]
ta = a.header.stamp.sec + a.header.stamp.nanosec*1e-9
tb = b.header.stamp.sec + b.header.stamp.nanosec*1e-9
print("  first stamp %.2f   last stamp %.2f   gap %.2f s" % (ta, tb, tb-ta))
print("  origin first (%.2f, %.2f)  last (%.2f, %.2f)"
      % (a.info.origin.position.x, a.info.origin.position.y,
         b.info.origin.position.x, b.info.origin.position.y))
if a.info.width == b.info.width and a.info.height == b.info.height:
    diff = sum(1 for x, y in zip(a.data, b.data) if x != y)
    print("  cells that changed between first and last: %d of %d (%.1f%%)"
          % (diff, len(a.data), diff*100.0/len(a.data)))
    if diff == 0:
        print("  -> the costmap is NOT changing. It is stale, not merely blocked.")
rclpy.shutdown()
