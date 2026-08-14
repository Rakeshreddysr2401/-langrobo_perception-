#!/usr/bin/env python3
"""
compare.py — every source's answer to "how far did I move?", side by side.

WHY THIS SHAPE
    A single fused pose cannot tell you which sensor is lying. When cuVSLAM
    under-read a 100 cm push as 26 cm, a fused number would have looked
    perfectly plausible; two rows disagreeing would not have. So Phase 1 shows
    the sources SEPARATELY and lets you be the referee with a tape measure.

SOURCES  (a row appears when its topic does; nothing here blocks on a dead one)
    cuvslam   /vo/odom       pose straight from stereo visual odometry
    wheels    /wheel_state   Vector3(velL, velR, cmd_vx), dead-reckoned here
    gyro      /gyro/base     yaw rate only, integrated to a heading

WHAT THE COLUMNS MEAN
    x, y     displacement from where you started, in centimetres
    th       heading change since you started, in degrees
    straight straight-line distance from the start  <- what a tape measures
    path     total distance travelled (bigger if it wandered or reversed)
    Hz       publish rate. A rate collapse is how every failure here has begun.

THE THREE PHASE 1 CHECKS
    ./rover compare --expect 2.00     push 2.00 m straight   -> want 1.90-2.10 m
    ./rover compare --return          out 2.00 m and back    -> want straight <= 0.10 m
    ./rover compare --spin 360        rotate 360 deg by hand -> want |err| <= 10 deg

Ctrl-C ends a run and prints the verdict. Every run is also written to CSV.
"""
import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

WHEEL_BASE_M = 0.34    # rover_firmware_v2.ino:100 — 34 cm between L/R wheel centres
STALE_S = 1.0          # a source with no message for this long is shown as stale


class Source:
    """One independent story about how the robot moved."""

    def __init__(self, name, gives):
        self.name = name
        self.gives = gives          # 'xyth' or 'th'
        self.x = self.y = self.th = 0.0
        self.path = 0.0
        self.n = 0
        self.first = None
        self.last_msg = 0.0
        self.last_rate_t = 0.0
        self.last_rate_n = 0
        self.hz = 0.0
        self._px = self._py = None

    def seen(self):
        self.n += 1
        self.last_msg = time.time()

    def advance(self, x, y, th):
        """Absolute pose in this source's own frame; we zero it at the first sample."""
        if self.first is None:
            self.first = (x, y, th)
        fx, fy, fth = self.first
        dx, dy = x - fx, y - fy
        c, s = math.cos(-fth), math.sin(-fth)
        self.x, self.y = c * dx - s * dy, s * dx + c * dy
        self.th = wrap(th - fth)
        if self._px is not None:
            self.path += math.hypot(self.x - self._px, self.y - self._py)
        self._px, self._py = self.x, self.y

    def integrate(self, vx, wz, dt):
        """Dead reckoning for sources that give velocity, not pose."""
        self.th = wrap(self.th + wz * dt)
        d = vx * dt
        self.x += d * math.cos(self.th)
        self.y += d * math.sin(self.th)
        self.path += abs(d)

    def tick_rate(self, now):
        dt = now - self.last_rate_t
        if dt >= 1.0:
            self.hz = (self.n - self.last_rate_n) / dt
            self.last_rate_t, self.last_rate_n = now, self.n

    @property
    def straight(self):
        return math.hypot(self.x, self.y)

    @property
    def stale(self):
        return self.n == 0 or (time.time() - self.last_msg) > STALE_S


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    """Yaw from a geometry_msgs quaternion. tf_transformations is not in this image."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Compare(Node):
    def __init__(self, args):
        super().__init__('compare')
        self.args = args
        self.src = {
            'cuvslam': Source('cuvslam', 'xyth'),
            'wheels': Source('wheels', 'xyth'),
            'gyro': Source('gyro', 'th'),
        }
        self.t0 = time.time()
        self.wheel_last_t = None
        self.gyro_last_t = None
        self.gyro_yaw = 0.0
        self.rows = []

        self.create_subscription(Odometry, '/vo/odom', self._vo, qos_profile_sensor_data)
        self.create_subscription(Vector3, '/wheel_state', self._wheels, qos_profile_sensor_data)
        self.create_subscription(Imu, '/gyro/base', self._gyro, qos_profile_sensor_data)
        self.create_timer(0.25, self._draw)

    def _vo(self, m):
        s = self.src['cuvslam']
        s.seen()
        s.advance(m.pose.pose.position.x, m.pose.pose.position.y,
                  yaw_of(m.pose.pose.orientation))

    def _wheels(self, m):
        s = self.src['wheels']
        s.seen()
        now = time.time()
        if self.wheel_last_t is not None:
            dt = now - self.wheel_last_t
            # Guard against the integrator eating one huge step after a stall.
            if 0 < dt < 0.5:
                vx = (m.x + m.y) / 2.0
                wz = (m.y - m.x) / WHEEL_BASE_M
                s.integrate(vx, wz, dt)
        self.wheel_last_t = now

    def _gyro(self, m):
        s = self.src['gyro']
        s.seen()
        now = time.time()
        if self.gyro_last_t is not None:
            dt = now - self.gyro_last_t
            if 0 < dt < 0.5:
                self.gyro_yaw = wrap(self.gyro_yaw + m.angular_velocity.z * dt)
                s.th = self.gyro_yaw
        self.gyro_last_t = now

    def _draw(self):
        now = time.time()
        for s in self.src.values():
            s.tick_rate(now)
        el = now - self.t0

        out = ['\033[H\033[J']
        out.append(f'  PHASE 1 — where does each sensor think it is?     t+{el:6.1f}s')
        out.append('')
        out.append('  source        x cm     y cm    th deg   straight cm   path cm      Hz')
        out.append('  ' + '-' * 68)
        for key in ('cuvslam', 'wheels', 'gyro'):
            s = self.src[key]
            if s.n == 0:
                out.append(f'  {key:<10}  {"— no publisher —":^46}')
                continue
            flag = '  STALE' if s.stale else ''
            if s.gives == 'th':
                out.append(f'  {key:<10} {"—":>7}  {"—":>7}  {math.degrees(s.th):8.2f}  '
                           f'{"—":>11}  {"—":>8}  {s.hz:6.1f}{flag}')
            else:
                out.append(f'  {key:<10} {s.x * 100:7.1f}  {s.y * 100:7.1f}  {math.degrees(s.th):8.2f}  '
                           f'{s.straight * 100:11.1f}  {s.path * 100:8.1f}  {s.hz:6.1f}{flag}')
        out.append('')

        a = self.args
        if a.expect:
            out.append(f'  target: push {a.expect * 100:.0f} cm straight   '
                       f'pass band {a.expect * 0.95 * 100:.0f}–{a.expect * 1.05 * 100:.0f} cm')
        elif a.ret:
            out.append('  target: out and back to the SAME mark   pass: straight <= 10.0 cm')
        elif a.spin:
            out.append(f'  target: rotate {a.spin:.0f} deg by hand   pass: |error| <= 10 deg')
        out.append('')
        out.append('  Ctrl-C to finish and grade.')
        print('\n'.join(out), flush=True)

        self.rows.append([round(el, 3)] + [
            v for key in ('cuvslam', 'wheels', 'gyro')
            for v in (round(self.src[key].x, 5), round(self.src[key].y, 5),
                      round(math.degrees(self.src[key].th), 3),
                      round(self.src[key].straight, 5), round(self.src[key].path, 5),
                      round(self.src[key].hz, 2))])

    # ── verdict ────────────────────────────────────────────────────────────
    def verdict(self):
        a = self.args
        print('\n\n  ── result ' + '─' * 58)
        for key in ('cuvslam', 'wheels', 'gyro'):
            s = self.src[key]
            if s.n == 0:
                print(f'  {key:<10} never published — no opinion')
                continue
            if s.gives == 'th':
                print(f'  {key:<10} heading {math.degrees(s.th):+8.2f} deg   ({s.n} msgs, {s.hz:.1f} Hz)')
            else:
                print(f'  {key:<10} straight {s.straight * 100:7.1f} cm   path {s.path * 100:7.1f} cm   '
                      f'heading {math.degrees(s.th):+7.2f} deg   ({s.n} msgs, {s.hz:.1f} Hz)')

        print()
        vo = self.src['cuvslam']
        if vo.n == 0:
            print('  GATE: cannot grade — cuvslam never published.')
        elif a.expect:
            lo, hi = a.expect * 0.95, a.expect * 1.05
            got, err = vo.straight, (vo.straight - a.expect) / a.expect * 100
            ok = lo <= got <= hi
            print(f'  GATE scale: pushed {a.expect * 100:.0f} cm, cuvslam read {got * 100:.1f} cm '
                  f'({err:+.1f}%)  ->  {"PASS" if ok else "FAIL"}')
            if not ok:
                print('    short  -> low-texture under-read; check the IR emitter is OFF,')
                print('              and try a floor with more visual texture.')
                print('    long   -> scale over-estimate, same class of problem.')
            if vo.path > got * 1.3:
                print(f'    NOTE: path ({vo.path * 100:.0f} cm) >> straight ({got * 100:.0f} cm) — '
                      'the pose is jittering, not tracking cleanly.')
        elif a.ret:
            ok = vo.straight <= 0.10
            print(f'  GATE drift: back at the start, cuvslam is {vo.straight * 100:.1f} cm away '
                  f'-> {"PASS" if ok else "FAIL"} (want <= 10.0 cm)')
            print(f'    path travelled {vo.path * 100:.0f} cm — should be about twice your leg length.')
            if not ok:
                print('    This is accumulated drift, and it is what smears a map: nvblox writes')
                print('    depth wherever the pose claims the robot is.')
        elif a.spin:
            err = abs(wrap(math.radians(a.spin) - self.src['cuvslam'].th))
            err_d = math.degrees(err)
            ok = err_d <= 10.0
            print(f'  GATE heading: rotated {a.spin:.0f} deg, cuvslam heading error '
                  f'{err_d:.2f} deg -> {"PASS" if ok else "FAIL"} (want <= 10 deg)')
            g = self.src['gyro']
            if g.n:
                print(f'    gyro independently read {math.degrees(g.th):+.2f} deg — '
                      'if these disagree, believe the gyro.')

        path = self._write_csv()
        print(f'\n  log: {path}\n')

    def _write_csv(self):
        d = os.path.expanduser('~/rover/logs')
        os.makedirs(d, exist_ok=True)
        p = os.path.join(d, f'compare-{datetime.now():%Y%m%d-%H%M%S}.csv')
        head = ['t']
        for k in ('cuvslam', 'wheels', 'gyro'):
            head += [f'{k}_{c}' for c in ('x', 'y', 'th_deg', 'straight', 'path', 'hz')]
        with open(p, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(head)
            w.writerows(self.rows)
        return p


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--expect', type=float, metavar='M', help='grade a straight push of M metres')
    ap.add_argument('--return', dest='ret', action='store_true', help='grade an out-and-back: expect to end where you started')
    ap.add_argument('--spin', type=float, metavar='DEG', help='grade a rotation of DEG degrees')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Compare(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.verdict()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
