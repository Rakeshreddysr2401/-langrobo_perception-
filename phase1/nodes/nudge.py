#!/usr/bin/env python3
"""Does a command reach the ESP32, and do the motors answer it?

/wheel_state.z is targetVx as the FIRMWARE last received it, so z tells us the
command arrived; x,y tell us the wheels moved. 0.05 m/s for 1.5 s = ~7 cm.
"""
import time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

class N(Node):
    def __init__(s):
        super().__init__("nudge"); s.w=None
        s.create_subscription(Vector3,"/wheel_state",lambda m:setattr(s,"w",m),10)
        s.cmd=s.create_publisher(Twist,"/cmd_vel",10)
    def spin(s,t):
        e=time.time()+t
        while time.time()<e and rclpy.ok(): rclpy.spin_once(s,timeout_sec=0.02)

rclpy.init(); n=N()
for _ in range(60):
    n.spin(0.25)
    if n.w is not None: break
t=Twist(); t.linear.x=0.05
t0=time.time(); zs=[]; L=[]; R=[]
while time.time()-t0<1.5:
    n.cmd.publish(t); n.spin(0.03)
    if n.w: zs.append(n.w.z); L.append(n.w.x); R.append(n.w.y)
for _ in range(8): n.cmd.publish(Twist()); n.spin(0.03)
got = max(zs) if zs else 0.0
pl, pr = (max(L, key=abs), max(R, key=abs)) if L else (0.0, 0.0)
print(f"      firmware received vx : max {got:+.3f}  (sent +0.050)")
print(f"      wheels               : peak L {pl:+.3f}  peak R {pr:+.3f}")
n.destroy_node(); rclpy.shutdown()
if got < 0.04:
    print("      ✗ the command never reached the firmware — /cmd_vel link, not motors")
    raise SystemExit(1)
if abs(pl) < 0.01 and abs(pr) < 0.01:
    print("      ✗ command ARRIVED and neither side moved: motor power is off")
    print("        (battery feed to the BTS7960 drivers, a switch, or a connector)")
    raise SystemExit(1)
if abs(pl) < 0.01 or abs(pr) < 0.01:
    print("      ✗ only one side moved — check that side's driver and motor wiring")
    raise SystemExit(1)
print("      ✓ command arrived and both sides drove")
