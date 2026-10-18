# Render global costmap + its nvblox and lidar layers around the rover, with a goal (odom x y), to /logs/peek/cost.png
import sys, math, time, numpy as np, cv2, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
gx, gy = float(sys.argv[1]), float(sys.argv[2])
rclpy.init(); n=Node('costpeek'); box={}
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
for t in ('/global_costmap/costmap','/global_costmap/nvblox_layer','/global_costmap/lidar_layer'):
    n.create_subscription(OccupancyGrid,t,lambda m,t=t:box.__setitem__(t,m),q)
tb=tf2_ros.Buffer(); tf2_ros.TransformListener(tb,n); t0=time.time(); tr=None
while time.time()-t0<15 and (len(box)<3 or tr is None):
    rclpy.spin_once(n,timeout_sec=0.2)
    try: tr=tb.lookup_transform('odom','base_link',rclpy.time.Time())
    except Exception: pass
rx,ry=tr.transform.translation.x,tr.transform.translation.y; qq=tr.transform.rotation
ryaw=math.atan2(2*(qq.w*qq.z+qq.x*qq.y),1-2*(qq.y*qq.y+qq.z*qq.z))
tiles=[]
for t in ('/global_costmap/costmap','/global_costmap/nvblox_layer','/global_costmap/lidar_layer'):
    m=box.get(t)
    if m is None: print('missing',t); continue
    r=m.info.resolution; ox,oy=m.info.origin.position.x,m.info.origin.position.y
    g=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
    def px(x,y): return int((x-ox)/r), int((y-oy)/r)
    cx,cy=px((rx+gx)/2,(ry+gy)/2); h=int(1.6/r)
    sub=g[max(cy-h,0):cy+h, max(cx-h,0):cx+h]; y0,x0=max(cy-h,0),max(cx-h,0)
    im=np.zeros(sub.shape+(3,),np.uint8)
    im[sub==-1]=(90,90,90); im[(sub>=0)&(sub<1)]=(255,255,255)
    mid=(sub>=1)&(sub<99); v=(sub[mid]/99.0)
    im[mid]=np.stack([255*(1-v),255*(1-v),np.full_like(v,255)],-1).astype(np.uint8)
    im[sub>=99]=(0,0,0); im[sub==99]=(140,0,140)
    u,vv=px(rx,ry); cv2.circle(im,(u-x0,vv-y0),int(0.19/r),(0,170,0),1)
    cv2.line(im,(u-x0,vv-y0),(int(u-x0+0.3/r*math.cos(ryaw)),int(vv-y0+0.3/r*math.sin(ryaw))),(0,170,0),1)
    u,vv=px(gx,gy); cv2.drawMarker(im,(u-x0,vv-y0),(255,0,0),cv2.MARKER_CROSS,12,2)
    im=cv2.flip(im,0); im=cv2.resize(im,(300,300),interpolation=cv2.INTER_NEAREST)
    cv2.putText(im,t.split('/')[-1],(5,15),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,120,0),1)
    tiles.append(im)
    lethal=(g>=99).sum(); print(f'{t}: {m.info.width}x{m.info.height} res {r} lethal(99+) cells {lethal}')
cv2.imwrite('/logs/peek/cost.png',np.hstack(tiles))
print(f'rover odom ({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}) goal ({gx},{gy})')
