#!/usr/bin/env python3
"""What yaw rate does the rover ACTUALLY reach at a commanded wz?

This measures the one number langrobo_core's timed turns depend on and have
never had: LANGROBO_STEADY_ANGULAR_VEL, the achieved rad/s at the commanded
rate. movement.py computes every turn as

    duration = radians(angle) / _STEADY_STATE_ANGULAR_VEL

and _STEADY_STATE_ANGULAR_VEL DEFAULTS TO THE COMMANDED RATE (5.0 rad/s). On a
skid-steer that is never true -- TODO 13 measured the body turning at ~65 deg/s
against a commanded ~290 deg/s, because scrub absorbs roughly two thirds of the
torque. So every turn runs ~4x too short: ask for 360 and get ~90.

WHY NOT wzsweep.py: it sweeps 0.15..3.0 rad/s looking for where motion STARTS
(the RPP rotate-to-heading deadlock). It never tests 5.0, which is what the
brain actually commands, and it only turns one way.

WHY NOT calibrate_rotation.py: that measures the effective TRACK WIDTH from a
hand-set known angle -- WHEEL_BASE_ROT_M, a different constant for a different
purpose (wheel odometry). This one commands the turn itself and times it.

BOTH DIRECTIONS. Scrub is not guaranteed symmetric; unequal tyre wear or weight
shows up as a left/right difference, and a single scalar has to live with both.

TWO SOURCES. Yaw is integrated from /odom (fused, whose heading is the gyro
override) and independently from /gyro/base. They should agree closely; if they
do not, something is wrong with the estimate and the number is not trustworthy
-- print both rather than average them into one confident-looking answer.

Usage:  python3 /logs/measure_yaw_rate.py [commanded_wz] [seconds] [repeats]
        defaults: 5.0 rad/s, 5.0 s, 2 repeats each direction

READ THE RAMP LINE, NOT JUST THE MEAN. A skid-steer takes real time to break
static scrub. Averaging one short window conflates the spin-up with the steady
state, and those are two SEPARATE constants -- _STEADY_STATE_ANGULAR_VEL and
_TURN_STARTUP_DELAY. If the per-bucket rate is still climbing at the end of the
run, there is no steady state in that window and the mean understates it.

THE ROVER PIVOTS IN PLACE AT FULL DUTY. Clear floor, human watching.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

WZ = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
REPEATS = int(sys.argv[3]) if len(sys.argv) > 3 else 2


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Meas(Node):
    def __init__(self):
        super().__init__('measure_yaw_rate')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_yaw = None
        self.gyro_z = 0.0
        self.create_subscription(Odometry, '/odom', self._odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/gyro/base', self._gyro, qos_profile_sensor_data)
        # integrals, unwrapped: a 2 s pivot can exceed pi and must not fold back
        self.odom_acc = 0.0
        self.gyro_acc = 0.0
        self.gyro_t = None

    def _odom(self, m):
        q = m.pose.pose.orientation
        y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.odom_yaw is not None:
            self.odom_acc += wrap(y - self.odom_yaw)
        self.odom_yaw = y

    def _gyro(self, m):
        now = time.time()
        if self.gyro_t is not None:
            self.gyro_acc += m.angular_velocity.z * (now - self.gyro_t)
        self.gyro_t = now
        self.gyro_z = m.angular_velocity.z

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def stop(self):
        t = Twist()
        for _ in range(15):
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_once(self, wz):
        self.stop()
        self.spin(0.6)
        self.odom_acc = 0.0
        self.gyro_acc = 0.0
        self.gyro_t = None      # else the first sample integrates the idle gap
        self.profile = []       # (t_since_start, |gyro z|) — the ramp is visible here
        t0 = time.time()
        t = Twist()
        t.angular.z = float(wz)
        while time.time() - t0 < DUR:
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.01)
            self.profile.append((time.time() - t0, abs(self.gyro_z)))
        dt = time.time() - t0
        self.stop()
        self.spin(0.5)          # let the integrals catch the tail of the motion
        return abs(self.odom_acc) / dt, abs(self.gyro_acc) / dt, dt

    def buckets(self, width=0.5):
        """Mean |yaw rate| per time bucket. A rover that is still accelerating
        at the end of the window has no steady state to calibrate against."""
        if not self.profile:
            return []
        out = []
        n = int(math.ceil(DUR / width))
        for i in range(n):
            lo, hi = i * width, (i + 1) * width
            vals = [r for (t, r) in self.profile if lo <= t < hi]
            if vals:
                out.append((lo, sum(vals) / len(vals)))
        return out


def main():
    rclpy.init()
    n = Meas()
    print(f"waiting for /odom and /gyro/base ...")
    end = time.time() + 15
    while time.time() < end and (n.odom_yaw is None or n.gyro_t is None):
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.odom_yaw is None or n.gyro_t is None:
        print("ERROR: no /odom or /gyro/base. Is ./rover fused up?")
        return

    print(f"\ncommanded wz = {WZ} rad/s ({math.degrees(WZ):.0f} deg/s), "
          f"{DUR} s per run, {REPEATS} runs each way")
    print("THE ROVER IS ABOUT TO PIVOT.\n")
    print("  dir   run    odom rate        gyro rate       swept")
    print("  " + "-" * 52)
    got = {'L': [], 'R': []}
    try:
        for rep in range(REPEATS):
            for name, sign in (('L', +1.0), ('R', -1.0)):
                o, g, dt = n.run_once(sign * WZ)
                got[name].append(o)
                print(f"  {name}     {rep+1}    {o:5.2f} rad/s     {g:5.2f} rad/s    "
                      f"{math.degrees(o)*dt:6.1f} deg")
                prof = "  ".join(f"{t:.1f}s:{r:4.2f}" for t, r in n.buckets())
                print(f"        ramp  {prof}")
                time.sleep(0.4)
    finally:
        n.stop()

    allr = got['L'] + got['R']
    if not allr:
        return
    mean = sum(allr) / len(allr)
    ml = sum(got['L']) / len(got['L']) if got['L'] else 0
    mr = sum(got['R']) / len(got['R']) if got['R'] else 0
    print("\n  " + "-" * 52)
    print(f"  achieved   left {ml:.2f}   right {mr:.2f}   mean {mean:.2f} rad/s "
          f"({math.degrees(mean):.0f} deg/s)")
    print(f"  commanded  {WZ:.2f} rad/s  ->  ratio {mean/WZ:.2f}")
    if ml > 0 and mr > 0:
        asym = abs(ml - mr) / max(ml, mr) * 100
        print(f"  asymmetry  {asym:.0f}%  " +
              ("(fine)" if asym < 15 else "(large -- one scalar will not serve both)"))
    print(f"\n  With the current default, a 360 deg request turns "
          f"{math.degrees(mean) * (math.radians(360)/WZ):.0f} deg.")
    print(f"\n  SET THIS:  LANGROBO_STEADY_ANGULAR_VEL={mean:.2f}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
