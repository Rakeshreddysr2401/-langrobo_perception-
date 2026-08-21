#!/usr/bin/env python3
"""Watch a teleop button press travel all the way to the wheels.

Four things have to happen for a button to move the rover, and when nothing
moves they are indistinguishable from the outside:

    phone -> Pi5 web -> /cmd_vel -> WiFi -> ESP32 -> PID -> BTS7960 -> motors

This prints the two ends of that chain side by side, so a failure names its own
stage: /cmd_vel silent means the web app or MANUAL mode; /cmd_vel moving with
velL/velR flat means the board heard it and the wheels did not turn (PID gate,
direction constants, driver, or wiring).

velL and velR are MEASURED from the encoders, not echoed back from the command,
so they are real evidence that the wheels turned.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, Vector3


class Watch(Node):
    def __init__(self):
        super().__init__('watch_teleop')
        self.create_subscription(Twist, '/cmd_vel', self._cmd, 10)
        self.create_subscription(Vector3, '/wheel_state', self._wheel,
                                 qos_profile_sensor_data)
        self.vx = self.wz = 0.0
        self.velL = self.velR = 0.0
        self.ncmd = 0
        self.peakL = self.peakR = 0.0
        self.peak_vx = 0.0
        self.saw_cmd = False
        self.saw_wheel = False

    def _cmd(self, m):
        self.vx, self.wz = m.linear.x, m.angular.z
        self.ncmd += 1
        if abs(m.linear.x) > 0.01 or abs(m.angular.z) > 0.01:
            self.saw_cmd = True
            self.peak_vx = max(self.peak_vx, abs(m.linear.x))

    def _wheel(self, m):
        self.velL, self.velR = m.x, m.y
        if abs(m.x) > 0.02 or abs(m.y) > 0.02:
            self.saw_wheel = True
        self.peakL = max(self.peakL, abs(m.x))
        self.peakR = max(self.peakR, abs(m.y))


def main():
    window = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
    rclpy.init()
    n = Watch()
    for _ in range(30):
        rclpy.spin_once(n, timeout_sec=0.05)

    print(f'\n  WATCHING for {window:.0f} s — press FORWARD briefly, then release\n')
    print('     cmd_vel vx     wz        velL      velR')
    print('  ' + '-' * 46)
    end = time.time() + window
    last = 0.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
        now = time.time()
        if now - last >= 0.25:
            last = now
            mark = ''
            if abs(n.vx) > 0.01 or abs(n.wz) > 0.01:
                mark = '  <- commanding'
            if abs(n.velL) > 0.02 or abs(n.velR) > 0.02:
                mark += '  WHEELS TURNING'
            print(f'     {n.vx:+7.3f}  {n.wz:+7.3f}   {n.velL:+8.3f}  '
                  f'{n.velR:+8.3f}{mark}')

    print(f'\n  {n.ncmd} /cmd_vel messages seen')
    print(f'  peak commanded vx {n.peak_vx:.3f} m/s')
    print(f'  peak measured velL {n.peakL:.3f}   velR {n.peakR:.3f} m/s\n')

    if not n.saw_cmd:
        print('  NOTHING WAS COMMANDED.')
        print('  /cmd_vel never carried a non-zero velocity, so the button press')
        print('  did not reach ROS at all. The rover is not the problem: check')
        print('  the page is set to MANUAL, and that the phone is on the same')
        print('  network as the Pi 5.')
    elif not n.saw_wheel:
        print('  COMMANDED, BUT THE WHEELS DID NOT TURN.')
        print('  The ESP32 received a real velocity and the encoders measured')
        print('  nothing. The network and the web app are fine. Suspect, in order:')
        print('    - controlEnabled false (agent not connected when it was sent)')
        print('    - gMinDuty 0.08 too low to break static friction')
        print('    - BTS7960 enable pins, or motor power not switched on')
    else:
        same = (n.peakL > 0.02 and n.peakR > 0.02)
        print('  COMMAND REACHED THE WHEELS AND THEY TURNED.')
        if not same:
            side = 'RIGHT' if n.peakL > n.peakR else 'LEFT'
            print(f'  But only ONE side moved — {side} measured nothing. Driving')
            print('  forward will curve instead of going straight.')
        else:
            ratio = min(n.peakL, n.peakR) / max(n.peakL, n.peakR)
            print(f'  Both sides moved, balance {ratio:.2f} '
                  f'(1.00 = perfectly matched).')

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
