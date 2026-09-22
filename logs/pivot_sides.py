#!/usr/bin/env python3
"""Is the left side really dead on pivots? Repeat it, both directions.

One trace can be a stall, a snag or a flat spot on a tyre. This alternates
left and right pivots several times and reports, per attempt, the mean SIGNED
speed of each side while the command was held -- so "that side never moves"
and "that side moves the wrong way" look different.
"""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi
WZ=1.5; TRACK_HALF=0.1925

class P(Node):
    def __init__(s):
        super().__init__("pivot_sides"); s.odom=None; s.wheel=None
        s.create_subscription(Odometry,"/odom",lambda m:setattr(s,"odom",m),10)
        s.create_subscription(Vector3,"/wheel_state",lambda m:setattr(s,"wheel",m),10)
        s.cmd=s.create_publisher(Twist,"/cmd_vel",10)
    def spin(s,t):
        e=time.time()+t
        while time.time()<e and rclpy.ok(): rclpy.spin_once(s,timeout_sec=0.02)

def attempt(n,wz,dur=3.0):
    y0=yaw_of(n.odom.pose.pose.orientation); acc=0.0; prev=y0
    t=Twist(); t.angular.z=wz
    t0=time.time(); L=[]; R=[]
    while time.time()-t0<dur and rclpy.ok():
        n.cmd.publish(t); n.spin(0.02)
        y=yaw_of(n.odom.pose.pose.orientation); acc+=wrap(y-prev); prev=y
        if n.wheel: L.append(n.wheel.x); R.append(n.wheel.y)
    for _ in range(8): n.cmd.publish(Twist()); n.spin(0.02)
    n.spin(1.0)
    ml=sum(L)/max(len(L),1); mr=sum(R)/max(len(R),1)
    want=wz*TRACK_HALF
    return ml,mr,math.degrees(acc),(-want),(want)

def main():
    rclpy.init(); n=P()
    for _ in range(60):
        n.spin(0.25)
        if n.odom is not None and n.wheel is not None: break
    if n.odom is None: print("      no /odom"); return 1
    print(f"      pivot at wz=+/-{WZ}. Each side should run +/-{WZ*TRACK_HALF:.3f} m/s.")
    print(f"      {'dir':>6s} {'mean L':>9s} {'want L':>9s} {'mean R':>9s} {'want R':>9s} {'turned':>9s}")
    for i in range(3):
        for wz in (WZ,-WZ):
            ml,mr,deg,wl,wr = attempt(n,wz)
            tag = "LEFT " if wz>0 else "RIGHT"
            print(f"      {tag:>6s} {ml:+9.3f} {wl:+9.3f} {mr:+9.3f} {wr:+9.3f} {deg:+9.1f}")
    n.destroy_node(); rclpy.shutdown(); return 0
raise SystemExit(main())
