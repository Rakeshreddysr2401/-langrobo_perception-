#!/usr/bin/env python3
"""run.py — drive one test scenario and record everything it needs.

    ./rover record SCENARIO [--dist M] [--wz RAD_S] [--v M_S] [--secs S] [--images] [--note TEXT]

    still      no motion for --secs (default 60). Walk past it: drift while parked
    manual     record --secs while YOU drive or push it; stop >= 2 s between moves
    straight   forward --dist (default 1.0 m), stop, back --dist, stop
    pivot90    +90 four times, then -90 four times, stopping after each
    pivot360   +360, stop, -360
    square     four times: forward --dist (default 0.5 m), stop, +90, stop
    small      the edge moves: +10, -10, +5, -5 deg, forward and back 5 cm

Every scenario starts and ends with the rover still for 4 s, and stops for
3 s between moves: the grader takes its truth only while the rover is still
(common.py). The bag lands in /logs/bags/<time>_<scenario>/ with a
scenario.json beside it; grade it with ./rover grade.

The motion is controlled by RAW sensors only -- turns stop on the integrated
gyro, straights on the average of the four wheel encoders -- never by the
fusion under test. How far it really went is the truth's job, not this
script's.

Ctrl-C stops the motors first, then closes the bag; the data is kept.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
from fusion import METRES_PER_COUNT  # noqa: E402

BAGS = Path(os.environ.get('HARNESS_BAGS', '/logs/bags'))
TOPICS = ['/scan', '/gyro/base', '/wheel_ticks', '/wheel_state', '/wheel_odom',
          '/vo/odom', '/vo/status', '/odom', '/fusion/status', '/cmd_vel',
          '/tf', '/tf_static', '/rover_diag']
IMAGES = ['/camera/camera0/infra1/image_rect_raw', '/camera/camera0/infra2/image_rect_raw',
          '/camera/camera0/infra1/camera_info', '/camera/camera0/infra2/camera_info']

HOLD_S, PAUSE_S = 4.0, 3.0
STALL_S = 8.0                    # no progress this long -> stop, abort
# the measured envelope's furthest corner + 5 cm (nav2.yaml footprint)
SWEEP_R = math.hypot(0.182, 0.19) + 0.05
HALF_W = 0.19 + 0.05             # corridor half-width kept clear going straight


def plan(a):
    d = a.dist
    return {
        'still': [('hold', a.secs)],
        'manual': [('manual', a.secs)],
        'straight': [('move', d), ('move', -d)],
        'pivot90': [('turn', 90)] * 4 + [('turn', -90)] * 4,
        'pivot360': [('turn', 360), ('turn', -360)],
        'square': [('move', d), ('turn', 90)] * 4,
        'small': [('turn', 10), ('turn', -10), ('turn', 5), ('turn', -5),
                  ('move', 0.05), ('move', -0.05)],
    }[a.scenario]


class Driver(Node):
    def __init__(self):
        super().__init__('harness_run')
        self.scan = self.ticks = self.odom = None
        self.gz = []                                   # (t, wz)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/gyro/base', self._gyro, qos_profile_sensor_data)
        self.create_subscription(Quaternion, '/wheel_ticks', self._ticks, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bias = 0.0

    def _scan(self, m): self.scan = m
    def _ticks(self, m): self.ticks = (m.x, m.y, m.z, m.w)
    def _odom(self, m): self.odom = m

    def _gyro(self, m):
        self.gz.append((time.time(), m.angular_velocity.z))
        if len(self.gz) > 4000:
            del self.gz[:2000]

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self):
        for _ in range(8):
            self.cmd.publish(Twist())
            self.spin(0.03)

    def hold(self, sec):
        """Still, publishing zero so the teleop and the watchdog agree."""
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            self.cmd.publish(Twist())
            self.spin(0.05)

    def measure_bias(self):
        t0 = time.time()
        self.hold(HOLD_S)
        g = [w for t, w in self.gz if t >= t0 + 0.5]
        self.bias = float(np.median(g)) if g else 0.0
        return len(g)

    def points(self, scan=None):
        """Scan in base_link, from the TF the mount lives in (description/)."""
        s = scan or self.scan
        r = np.asarray(s.ranges)
        a = s.angle_min + np.arange(r.size) * s.angle_increment
        ok = np.isfinite(r) & (r > 0.05) & (r < 8.0)
        x, y, yaw = self.mount
        p = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
        c, si = math.cos(yaw), math.sin(yaw)
        return p @ np.array([[c, -si], [si, c]]).T + np.array([x, y])

    def turn(self, deg, wz):
        p = self.points()
        near = float(np.min(np.hypot(p[:, 0], p[:, 1]))) if len(p) else 9.0
        if near < SWEEP_R:
            raise RuntimeError(f'something {near:.2f} m away, inside the {SWEEP_R:.2f} m swing -- clear it')
        target = math.radians(deg)
        tw = Twist(); tw.angular.z = math.copysign(wz, target)
        turned, last_t = 0.0, time.time()
        best, best_t = 0.0, time.time()
        k = len(self.gz)
        while rclpy.ok():
            self.cmd.publish(tw)
            self.spin(0.02)
            for t, w in self.gz[k:]:
                turned += (w - self.bias) * (t - last_t)
                last_t = t
            k = len(self.gz)
            if abs(turned) >= abs(target):
                break
            if abs(turned) > best + math.radians(1):
                best, best_t = abs(turned), time.time()
            if time.time() - best_t > STALL_S:
                self.stop()
                raise RuntimeError(f'stalled at {math.degrees(turned):+.1f} of {deg:+.0f} deg')
        self.stop()
        return math.degrees(turned)

    def move(self, dist, v):
        tw = Twist(); tw.linear.x = math.copysign(v, dist)
        t0 = np.array(self.ticks, dtype=float)
        done, best_t = 0.0, time.time()
        while rclpy.ok():
            left = abs(dist) - done
            p = self.points()
            ahead = p[:, 0] * math.copysign(1, dist)
            front = 0.182 if dist > 0 else 0.178
            block = (ahead > front) & (ahead < front + left + 0.10) & (np.abs(p[:, 1]) < HALF_W)
            if np.any(block):
                self.stop()
                raise RuntimeError('obstacle in the path -- stopped')
            self.cmd.publish(tw)
            self.spin(0.03)
            d = (np.array(self.ticks, dtype=float) - t0).mean() * METRES_PER_COUNT
            if abs(d) > done + 0.005:
                best_t = time.time()
            done = abs(d)
            if done >= abs(dist):
                break
            if time.time() - best_t > STALL_S:
                self.stop()
                raise RuntimeError(f'stalled after {done:.3f} of {abs(dist):.3f} m')
        self.stop()
        return math.copysign(done, dist)


def teleop_auto():
    """The Pi 5 teleop streams zeros over every command in MANUAL."""
    import urllib.request
    try:
        with urllib.request.urlopen('http://192.168.1.16:8091/mode', timeout=4) as r:
            return '"manual": true' not in r.read().decode()
    except Exception:  # noqa: BLE001 -- unreachable teleop is reported, not fatal
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenario', choices=['still', 'manual', 'straight', 'pivot90', 'pivot360', 'square', 'small'])
    ap.add_argument('--dist', type=float, default=None)
    ap.add_argument('--wz', type=float, default=float(os.environ.get('PIVOT_WZ', '1.5')))
    ap.add_argument('--v', type=float, default=0.15)
    ap.add_argument('--secs', type=float, default=60.0)
    ap.add_argument('--images', action='store_true', help='also record stereo IR (~25 MB/s)')
    ap.add_argument('--note', default='')
    a = ap.parse_args()
    if a.dist is None:
        a.dist = 0.5 if a.scenario == 'square' else 1.0
    steps = plan(a)
    moves = any(s[0] in ('move', 'turn') for s in steps)

    rclpy.init()
    n = Driver()
    end = time.time() + 8.0   # discovery over the network can take a few seconds
    while (n.scan is None or n.ticks is None) and time.time() < end:
        n.spin(0.2)
    n.spin(1.0)               # and a moment for the slower ones (gyro, odom)
    missing = [t for t, v in (('/scan', n.scan), ('/wheel_ticks', n.ticks)) if v is None]
    if missing:
        sys.exit(f'  no {", ".join(missing)} -- the grader needs both. ./rover lidar, ./rover wheels')
    if not n.gz:
        print('  ! no /gyro/base: turns cannot be timed on the gyro and wheels+gyro will not be graded.'
              ' ./rover pose starts it.')
        if any(s[0] == 'turn' for s in steps):
            sys.exit('  this scenario turns -- start the gyro first')
    if n.odom is None:
        print('  ! no /odom: the live fusion will not be graded (./rover fused)')
    # Spun on THIS thread: a spin_thread listener's thread never wakes at
    # shutdown and keeps the interpreter alive after the run (seen 2026-09-24).
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    from common import yaw_of
    buf = Buffer(); tfl = TransformListener(buf, n)
    end = time.time() + 5.0
    while not buf.can_transform('base_link', 'laser', Time()) and time.time() < end:
        n.spin(0.1)
    if not buf.can_transform('base_link', 'laser', Time()):
        sys.exit('  no base_link -> laser: ./rover lidar starts it')
    tr = buf.lookup_transform('base_link', 'laser', Time()).transform
    n.mount = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
    tfl.unregister()
    if moves:
        auto = teleop_auto()
        if auto is False:
            sys.exit('  teleop is in MANUAL -- it streams zeros over every command. Tap AUTO.')
        if auto is None:
            print('  ! teleop mode unknown (Pi 5 unreachable) -- if nothing moves, check AUTO')

    # the container runs on UTC; ./rover passes the host's local time
    name = f"{os.environ.get('RUN_STAMP') or datetime.now().strftime('%Y%m%d-%H%M%S')}_{a.scenario}"
    out = BAGS / name
    BAGS.mkdir(parents=True, exist_ok=True)
    topics = TOPICS + (IMAGES if a.images else [])
    rec = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(out), '--topics', *topics],
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, start_new_session=True)
    n.spin(2.5)  # let the recorder discover and subscribe before anything moves
    if rec.poll() is not None:
        sys.exit('  ros2 bag record exited: ' + rec.stderr.read().decode()[-400:])

    log, err = [], None
    started = time.time()
    try:
        print(f'  recording -> {out}')
        k = n.measure_bias()
        print(f'  still {HOLD_S:.0f} s: gyro bias {n.bias:+.4f} rad/s from {k} samples')
        if moves:
            print('  THIS MOVES THE ROVER. Watch it; Ctrl-C stops the motors.')
        for i, (kind, val) in enumerate(steps, 1):
            t = time.time() - started
            if kind == 'hold':
                print(f'  [{i}/{len(steps)}] still for {val:.0f} s -- walk past it, do not touch it')
                n.hold(val); got = None
            elif kind == 'manual':
                print(f'  [{i}/{len(steps)}] {val:.0f} s: drive or push it now; stop >= 2 s between moves')
                n.spin(val); got = None
            elif kind == 'turn':
                got = n.turn(val, a.wz)
                print(f'  [{i}/{len(steps)}] turn {val:+.0f} deg: gyro says {got:+.1f}')
            else:
                got = n.move(val, a.v)
                print(f'  [{i}/{len(steps)}] move {val:+.2f} m: wheels say {got:+.3f}')
            log.append({'step': i, 'kind': kind, 'asked': val, 'raw': got, 't': t})
            n.hold(PAUSE_S if i < len(steps) else HOLD_S)
    except KeyboardInterrupt:
        err = 'interrupted'
    except RuntimeError as e:
        err = str(e)
    finally:
        n.stop()
        os.killpg(rec.pid, signal.SIGINT)
        try:
            rec.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(rec.pid, signal.SIGKILL)
        meta = {'scenario': a.scenario, 'args': vars(a), 'steps': log, 'error': err,
                'git': os.environ.get('GIT_REV', ''), 'gyro_bias': n.bias,
                'laser_mount': n.mount, 'duration_s': time.time() - started}
        if out.exists():
            (out / 'scenario.json').write_text(json.dumps(meta, indent=1))
        n.destroy_node()
        rclpy.try_shutdown()
    if err:
        print(f'  ✗ {err} -- the bag is kept and can still be graded up to that point')
    print(f'  saved {out}\n  grade it:  ./rover grade {out.name}')


if __name__ == '__main__':
    main()
