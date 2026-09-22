#!/usr/bin/env python3
"""What yaw rate does the rover ACTUALLY reach for each commanded wz?

Runs each command for a fixed window and reports achieved rad/s, the ratio,
and the peak wheel speeds. Alternates direction so it does not wind up in one
direction and hit something.
"""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi
TRACK_HALF = 0.1925   # m, from the measured footprint

class S(Node):
    def __init__(s):
        super().__init__("wz_sweep"); s.odom=None; s.wheel=None
        s.create_subscription(Odometry,"/odom",lambda m:setattr(s,"odom",m),10)
        s.create_subscription(Vector3,"/wheel_state",lambda m:setattr(s,"wheel",m),10)
        s.cmd=s.create_publisher(Twist,"/cmd_vel",10)
    def spin(s,t):
        e=time.time()+t
        while time.time()<e and rclpy.ok(): rclpy.spin_once(s,timeout_sec=0.04)

def run(n,wz,dur=3.0):
    y0=yaw_of(n.odom.pose.pose.orientation); acc=0.0; prev=y0
    t=Twist(); t.angular.z=wz
    t0=time.time(); pl=pr=0.0
    while time.time()-t0<dur and rclpy.ok():
        n.cmd.publish(t); n.spin(0.04)
        y=yaw_of(n.odom.pose.pose.orientation); acc+=wrap(y-prev); prev=y
        if n.wheel:
            pl=max(pl,abs(n.wheel.x)); pr=max(pr,abs(n.wheel.y))
    for _ in range(8): n.cmd.publish(Twist()); n.spin(0.04)
    n.spin(1.2)
    dt=time.time()-t0
    got=acc/dt
    need=abs(wz)*TRACK_HALF
    print(f"      {wz:+6.2f} {got:+11.3f} {abs(got/wz)*100:7.0f}%  "
          f"{math.degrees(acc):+9.1f}   {need:8.3f} {pl:8.3f} {pr:8.3f}")
    return got

def main():
    rclpy.init(); n=S(); n.spin(2.0)
    if n.odom is None: print("      no /odom"); return 1
    print(f"      {'cmd wz':>6s} {'got rad/s':>11s} {'ratio':>8s} {'turned':>9s}   "
          f"{'need m/s':>8s} {'peak L':>8s} {'peak R':>8s}")
    for wz in (0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.8, -1.8, 2.5, -2.5):
        run(n, wz)
    n.destroy_node(); rclpy.shutdown(); return 0
raise SystemExit(main())
