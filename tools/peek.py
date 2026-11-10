# Dump one colour JPEG, and a PNG of /map + /scan + the robot pose, into /logs/peek/
import os, math, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import tf2_ros
import cv2
out = '/logs/peek'; os.makedirs(out, exist_ok=True)
class P(Node):
    def __init__(s):
        super().__init__('peek'); s.img=s.scan=s.map=s.odom=None
        s.create_subscription(CompressedImage,'/camera/color/image_raw/compressed',lambda m:setattr(s,'img',m),2)
        s.create_subscription(LaserScan,'/scan',lambda m:setattr(s,'scan',m),QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT))
        s.create_subscription(OccupancyGrid,'/map',lambda m:setattr(s,'map',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        s.create_subscription(Odometry,'/odom',lambda m:setattr(s,'odom',m),5)
rclpy.init(); n=P(); tb=tf2_ros.Buffer(); tl=tf2_ros.TransformListener(tb,n); t=time.time()
while time.time()-t<12 and None in (n.img,n.scan,n.map,n.odom): rclpy.spin_once(n,timeout_sec=0.2)
if n.img: open(f'{out}/color.jpg','wb').write(bytes(n.img.data))
if n.map:
    m=n.map; g=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
    im=np.full(g.shape+(3,),128,np.uint8); im[g==0]=255; im[g>50]=0
    r=m.info.resolution; ox,oy=m.info.origin.position.x,m.info.origin.position.y
    def px(x,y): return int((x-ox)/r), int((y-oy)/r)
    if n.odom:
        while True:
            try: tr=tb.lookup_transform('map','base_link',rclpy.time.Time()); break
            except Exception: rclpy.spin_once(n,timeout_sec=0.2)
        x,y=tr.transform.translation.x,tr.transform.translation.y; q=tr.transform.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if n.scan:
            for i,d in enumerate(n.scan.ranges):
                if n.scan.range_min<d<n.scan.range_max:
                    a=yaw+n.scan.angle_min+i*n.scan.angle_increment; u,v=px(x+d*math.cos(a),y+d*math.sin(a))
                    if 0<=u<g.shape[1] and 0<=v<g.shape[0]: im[v,u]=(0,0,255)
        u,v=px(x,y); cv2.circle(im,(u,v),int(0.19/r),(0,160,0),1); cv2.line(im,(u,v),(int(u+0.3/r*math.cos(yaw)),int(v+0.3/r*math.sin(yaw))),(0,160,0),1)
        print(f'map pose x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}')
    im=cv2.flip(im,0); s=max(1,int(600/max(im.shape[:2]))); im=cv2.resize(im,None,fx=s,fy=s,interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(f'{out}/map.png',im); print('map',m.info.width,'x',m.info.height,'res',r,'origin',round(ox,2),round(oy,2))
print('got', {k:getattr(n,k) is not None for k in ('img','scan','map','odom')})
