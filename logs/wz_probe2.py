#!/usr/bin/env python3
"""Higher wz, and an arc, to separate friction from a broken angular mapping."""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi

class P(Node):
    def __init__(s):
        super().__init__("wz_probe2"); s.odom=None; s.wheel=None
        s.create_subscription(Odometry,"/odom",lambda m:setattr(s,"odom",m),10)
        s.create_subscription(Vector3,"/wheel_state",lambda m:setattr(s,"wheel",m),10)
        s.cmd=s.create_publisher(Twist,"/cmd_vel",10)
    def spin(s,t):
        e=time.time()+t
        while time.time()<e and rclpy.ok(): rclpy.spin_once(s,timeout_sec=0.05)

def trial(n,label,lin,ang,dur=2.5):
    y0=yaw_of(n.odom.pose.pose.orientation); acc=0.0; prev=y0
    t=Twist(); t.linear.x=lin; t.angular.z=ang
    t0=time.time(); w=[]
    while time.time()-t0<dur and rclpy.ok():
        n.cmd.publish(t); n.spin(0.05)
        y=yaw_of(n.odom.pose.pose.orientation); acc+=wrap(y-prev); prev=y
        if n.wheel: w.append((n.wheel.x,n.wheel.y))
    for _ in range(6): n.cmd.publish(Twist()); n.spin(0.05)
    n.spin(1.0)
    dt=time.time()-t0
    wx=max((abs(a[0]) for a in w), default=0); wy=max((abs(a[1]) for a in w), default=0)
    print(f"      {label:22s} turned {math.degrees(acc):+7.1f} deg  "
          f"({acc/dt:+.3f} rad/s)   peak wheel |L| {wx:.3f} |R| {wy:.3f}")
    return acc

def main():
    rclpy.init(); n=P(); n.spin(2.0)
    if n.odom is None: print("no /odom"); return 1
    print("      PIVOT, increasing command:")
    for wz in (1.5, 2.0, 3.0):
        trial(n,f"pivot wz={wz}",0.0,wz)
    print("      ARC (linear + angular) — does it turn when already rolling?")
    for wz in (0.5, 1.0):
        trial(n,f"arc 0.08 m/s wz={wz}",0.08,wz)
    print("      reverse arc, to unwind:")
    trial(n,"arc -0.08 wz=-1.0",-0.08,-1.0)
    n.destroy_node(); rclpy.shutdown(); return 0
raise SystemExit(main())
