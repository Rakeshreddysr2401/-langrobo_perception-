import rclpy, time, math, sys
from rclpy.qos import qos_profile_sensor_data
from nvblox_msgs.msg import DistanceMapSlice
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
Q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE,history=HistoryPolicy.KEEP_LAST)
rclpy.init(); n=rclpy.create_node("esdf"); g={}
n.create_subscription(DistanceMapSlice,"/nvblox_node/static_map_slice",lambda m:g.__setitem__("s",m),qos_profile_sensor_data)
n.create_subscription(OccupancyGrid,"/nvblox_node/static_occupancy_grid",lambda m:g.__setitem__("o",m),Q)
e=time.time()+20
while time.time()<e and len(g)<2: rclpy.spin_once(n,timeout_sec=0.1)
tag = sys.argv[1] if len(sys.argv)>1 else ""
print(f"  ── ESDF slice {tag} " + "─"*(40-len(tag)))
if "s" not in g:
    print("  no slice"); rclpy.shutdown(); raise SystemExit
m=g["s"]; uv=m.unknown_value
k=sorted(v for v in m.data if v!=uv and not math.isnan(v))
if not k:
    print("  slice is entirely unobserved"); rclpy.shutdown(); raise SystemExit
med=k[len(k)//2]
over=sum(1 for v in k if v>1.0)
print(f"   known cells            {len(k)}")
print(f"   median clearance       {med:.2f} m      (want well above 0.35)")
print(f"   further than 1.0 m     {over*100//len(k)}%           (want much more than 15%)")
if "o" in g:
    o=g["o"]; res=o.info.resolution; a=res*res
    occ=sum(1 for v in o.data if v>=50); fr=sum(1 for v in o.data if 0<=v<50)
    print(f"   floor  {fr*a:6.1f} m2")
    print(f"   wall   {occ*a:6.1f} m2      (a room perimeter should be under ~1)")
    if occ: print(f"   ratio  wall is {occ*100/max(fr,1):.0f}% of floor   (want under ~10%)")
rclpy.shutdown()
