#!/usr/bin/env python3
"""Command a range of angular.z and see what the wheels and odom actually do."""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a):
    return (a+math.pi)%(2*math.pi)-math.pi

class P(Node):
    def __init__(s):
        super().__init__("wz_probe")
        s.odom=None; s.wheel=None
        s.create_subscription(Odometry,"/odom",lambda m:setattr(s,"odom",m),10)
        s.create_subscription(Vector3,"/wheel_state",lambda m:setattr(s,"wheel",m),10)
        s.cmd=s.create_publisher(Twist,"/cmd_vel",10)
    def spin(s,t):
        e=time.time()+t
        while time.time()<e and rclpy.ok(): rclpy.spin_once(s,timeout_sec=0.05)

def main():
    rclpy.init(); n=P(); n.spin(2.0)
    if n.odom is None: print("no /odom"); return 1
    print(f"      wheel_state sample: {n.wheel}")
    print(f"      {'cmd wz':>8s} {'measured wz':>12s} {'wheel x':>9s} {'wheel y':>9s} {'wheel z':>9s}")
    for wz in (0.3, 0.5, 0.8, 1.2, -0.8):
        t=Twist(); t.angular.z=wz
        _,y0 = None, yaw_of(n.odom.pose.pose.orientation)
        t0=time.time(); acc=0.0; prev=y0
        w=[]
        while time.time()-t0 < 2.0 and rclpy.ok():
            n.cmd.publish(t); n.spin(0.05)
            y=yaw_of(n.odom.pose.pose.orientation); acc+=wrap(y-prev); prev=y
            if n.wheel: w.append((n.wheel.x,n.wheel.y,n.wheel.z))
        for _ in range(5): n.cmd.publish(Twist()); n.spin(0.05)
        n.spin(0.8)
        dt=time.time()-t0
        wx=sum(a[0] for a in w)/max(len(w),1); wy=sum(a[1] for a in w)/max(len(w),1)
        wzs=sum(a[2] for a in w)/max(len(w),1)
        print(f"      {wz:+8.2f} {acc/dt:+12.3f} {wx:+9.3f} {wy:+9.3f} {wzs:+9.3f}")
    # and a linear one, as the control
    t=Twist(); t.linear.x=0.07
    t0=time.time(); w=[]
    while time.time()-t0<1.5 and rclpy.ok():
        n.cmd.publish(t); n.spin(0.05)
        if n.wheel: w.append((n.wheel.x,n.wheel.y,n.wheel.z))
    for _ in range(5): n.cmd.publish(Twist()); n.spin(0.05)
    wx=sum(a[0] for a in w)/max(len(w),1); wy=sum(a[1] for a in w)/max(len(w),1)
    wzs=sum(a[2] for a in w)/max(len(w),1)
    print(f"      linear 0.07 m/s          wheel {wx:+.3f} {wy:+.3f} {wzs:+.3f}  (control)")
    n.destroy_node(); rclpy.shutdown(); return 0
raise SystemExit(main())
