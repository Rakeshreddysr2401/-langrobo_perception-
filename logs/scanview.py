# Robot-centred lidar picture (forward = up) + clearance per direction, to /logs/peek/scan.png
import math, time, numpy as np, cv2, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
rclpy.init(); n=Node('scanview'); box={}
n.create_subscription(LaserScan,'/scan',lambda m:box.__setitem__('s',m),QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT))
tb=tf2_ros.Buffer(); tf2_ros.TransformListener(tb,n); t=time.time(); off=None
while ('s' not in box or off is None) and time.time()-t<10:
    rclpy.spin_once(n,timeout_sec=0.2)
    try:
        q=tb.lookup_transform('base_link',box['s'].header.frame_id,rclpy.time.Time()).transform.rotation; off=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    except Exception: pass
print(f'laser yaw on the body: {math.degrees(off):.1f} deg')
s=box['s']; S=600; k=S/8.0; im=np.full((S,S,3),255,np.uint8); c=S//2
for r in (0.5,1,2,3): cv2.circle(im,(c,c),int(r*k),(220,220,220),1)
sec={}
for i,d in enumerate(s.ranges):
    if not (s.range_min<d<s.range_max): continue
    a=off+s.angle_min+i*s.angle_increment  # base_link: x fwd, y left
    u=int(c-d*math.sin(a)*k); v=int(c-d*math.cos(a)*k)
    if 0<=u<S and 0<=v<S: im[v,u]=(0,0,255)
    deg=(math.degrees(a)+360)%360; key=int(((deg+22.5)%360)//45)
    sec[key]=min(sec.get(key,9),d)
cv2.rectangle(im,(int(c-0.19*k),int(c-0.182*k)),(int(c+0.19*k),int(c+0.178*k)),(0,150,0),1)
cv2.arrowedLine(im,(c,c),(c,int(c-0.4*k)),(0,150,0),1)
cv2.imwrite('/logs/peek/scan.png',im)
names=['front','front-left','left','back-left','back','back-right','right','front-right']
print('nearest (m):',', '.join(f'{names[k]} {sec.get(k,9):.2f}' for k in range(8)))
