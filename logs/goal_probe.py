# Safe probe: send /reach/goal = the rover's CURRENT pose (nothing to drive), stamped now, up to 3 times; print reach's answers.
import time, json, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
rclpy.init(); n=Node('goal_probe'); st={}; got=[]
n.create_subscription(Odometry,'/odom',lambda m:st.update(o=m),10)
n.create_subscription(String,'/reach/status',lambda m:got.append(m.data),10)
pub=n.create_publisher(PoseStamped,'/reach/goal',10)
t=time.time()
while time.time()-t<10 and ('o' not in st or pub.get_subscription_count()==0): rclpy.spin_once(n,timeout_sec=0.1)
print('subs on /reach/goal:',pub.get_subscription_count(),'odom:', 'o' in st)
for k in range(3):
    p=PoseStamped(); p.header.frame_id='odom'; p.header.stamp=n.get_clock().now().to_msg(); p.pose=st['o'].pose.pose
    pub.publish(p); print(f'sent #{k+1} stamp {p.header.stamp.sec}.{p.header.stamp.nanosec:09d}')
    t=time.time()
    while time.time()-t<4: rclpy.spin_once(n,timeout_sec=0.1)
    if got: print('reach answered:', got[-1][:200]); break
else: print('NO answer from reach in 12 s')
