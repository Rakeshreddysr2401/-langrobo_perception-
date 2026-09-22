#!/usr/bin/env python3
"""Signed time series of both wheel sides during a pivot command.

Magnitudes hide the two failures that matter: both sides turning the SAME way
(which drives instead of turning) and one side doing all the work (which arcs
and scrubs). This prints the signed values, ten times a second.
"""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi

class T(Node):
    def __init__(s):
        super().__init__("wheel_trace"); s.odom=None; s.wheel=None
        s.create_subscription(Odometry,"/odom",lambda m:setattr(s,"odom",m),10)
        s.create_subscription(Vector3,"/wheel_state",lambda m:setattr(s,"wheel",m),10)
        s.cmd=s.create_publisher(Twist,"/cmd_vel",10)
    def spin(s,t):
        e=time.time()+t
        while time.time()<e and rclpy.ok(): rclpy.spin_once(s,timeout_sec=0.02)

def trace(n,label,lin,ang,dur=4.0):
    print(f"      --- {label}: linear {lin:+.2f}  angular {ang:+.2f} ---")
    print(f"      {'t':>5s} {'L (x)':>8s} {'R (y)':>8s} {'z':>8s} {'yaw deg':>9s}")
    y0=yaw_of(n.odom.pose.pose.orientation); acc=0.0; prev=y0
    t=Twist(); t.linear.x=lin; t.angular.z=ang
    t0=time.time(); nxt=0.0
    while time.time()-t0<dur and rclpy.ok():
        n.cmd.publish(t); n.spin(0.02)
        y=yaw_of(n.odom.pose.pose.orientation); acc+=wrap(y-prev); prev=y
        el=time.time()-t0
        if el>=nxt and n.wheel:
            print(f"      {el:5.1f} {n.wheel.x:+8.3f} {n.wheel.y:+8.3f} "
                  f"{n.wheel.z:+8.3f} {math.degrees(acc):+9.1f}")
            nxt+=0.5
    for _ in range(8): n.cmd.publish(Twist()); n.spin(0.02)
    n.spin(1.0)
    print(f"      net {math.degrees(acc):+.1f} deg")

def main():
    rclpy.init(); n=T()
    for _ in range(60):
        n.spin(0.25)
        if n.odom is not None and n.wheel is not None: break
    if n.odom is None: print("      no /odom"); return 1
    trace(n,"PIVOT left", 0.0, 1.5)
    trace(n,"PIVOT right",0.0,-1.5)
    trace(n,"STRAIGHT (control)",0.08,0.0,3.0)
    n.destroy_node(); rclpy.shutdown(); return 0
raise SystemExit(main())
