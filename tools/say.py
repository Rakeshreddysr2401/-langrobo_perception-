# say.py "<sentence>" [seconds]: publish it as a spoken sentence (/voice/user_input) and print
# what the robot says, reach/goal_exec status changes and the brain's thinking flag, with times.
import sys, time, json, rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
text=sys.argv[1]; dur=float(sys.argv[2]) if len(sys.argv)>2 else 90
rclpy.init(); n=Node('say_probe'); t0=time.time(); last={}
def log(tag,msg): print(f'{time.time()-t0:6.1f}s {tag}: {msg}', flush=True)
def onstr(tag):
    def f(m):
        d=m.data
        if tag in ('reach','goal_exec','follow'):
            try: j=json.loads(d); d=' '.join(f'{k}={j[k]}' for k in ('phase','state','result','why','attempt','dist','req') if k in j) or d[:160]
            except Exception: d=d[:160]
        if last.get(tag)!=d: last[tag]=d; log(tag,d)
    return f
n.create_subscription(String,'/voice/robot_speech',onstr('SAYS'),10)
n.create_subscription(String,'/reach/status',onstr('reach'),10)
n.create_subscription(String,'/goal_exec/status',onstr('goal_exec'),10)
n.create_subscription(String,'/follow/status',onstr('follow'),10)
n.create_subscription(Bool,'/brain/thinking',lambda m:log('thinking',m.data),10)
pub=n.create_publisher(String,'/voice/user_input',10)
t=time.time()
while pub.get_subscription_count()==0 and time.time()-t<10: rclpy.spin_once(n,timeout_sec=0.2)
pub.publish(String(data=text)); log('HEARD',text)
while time.time()-t0<dur: rclpy.spin_once(n,timeout_sec=0.2)
