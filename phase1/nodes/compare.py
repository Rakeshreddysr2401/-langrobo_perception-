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
    FUSED                    distance from cuVSLAM, heading from the gyro

WHY THERE IS A FUSED ROW AS WELL
    Showing the sources separately is what finds the faults; a fused estimate is
    what you actually navigate on. Measured 2026-08-15 on a 2 m out-and-back,
    fusing this way took the endpoint error from 28.4 cm to 12.2 cm.

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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

WHEEL_BASE_M = 0.34    # rover_firmware_v2.ino:100 — 34 cm between L/R wheel centres
STALE_S = 1.0          # a source with no message for this long is shown as stale

# Path is accumulated in CHORDS of at least this length, not per frame.
# Measured 2026-08-15: parked 125 s, per-frame position noise is ~26 um. Summing
# |delta| every frame adds a magnitude that can never cancel, so `path` ratcheted
# up 9.0 cm while the rover sat perfectly still (`straight` stayed at 0.3 cm, so
# the pose itself was fine — only the accumulator was lying). Waiting until the
# pose has moved 5 mm from the last anchor point puts real travel far above the
# noise floor. Cost: travel is quantised to 5 mm, and a crawl slower than about
# 0.15 cm/s reads low.
PATH_CHORD_M = 0.005

# Seconds of stillness at startup used to measure gyro bias before integrating.
GYRO_BIAS_S = 5.0

# A step larger than this between consecutive /vo/odom messages is not motion.
# At ~28 Hz this is over 4 m/s, which no hand push reaches. Measured 2026-08-15:
# cuVSLAM teleported 202 cm in a single frame, at a healthy 28 Hz, with nothing
# logged — it had lost tracking during a fast shove (peak 76 cm/s, against
# 19 cm/s in the run that worked) and silently re-initialised. Every number after
# such a jump is measured from a corrupted origin, so a run containing one must
# be thrown away, not graded.
JUMP_M = 0.15

# Display / logging order.
ORDER = ('cuvslam', 'wheels', 'gyro', 'FUSED')


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
        self._ax = self._ay = None      # last path anchor

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
        self._accumulate_path()

    def integrate(self, vx, wz, dt):
        """Dead reckoning for sources that give velocity, not pose."""
        self.th = wrap(self.th + wz * dt)
        d = vx * dt
        self.x += d * math.cos(self.th)
        self.y += d * math.sin(self.th)
        self._accumulate_path()

    def step_body(self, bx, by, th):
        """Add a step already expressed in the body frame, with heading supplied."""
        self.th = wrap(th)
        c, s = math.cos(self.th), math.sin(self.th)
        self.x += c * bx - s * by
        self.y += s * bx + c * by
        self._accumulate_path()

    def _accumulate_path(self):
        """Add travel in chords of >= PATH_CHORD_M, so noise cannot ratchet it up."""
        if self._ax is None:
            self._ax, self._ay = self.x, self.y
            return
        d = math.hypot(self.x - self._ax, self.y - self._ay)
        if d >= PATH_CHORD_M:
            self.path += d
            self._ax, self._ay = self.x, self.y

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
            'FUSED': Source('FUSED', 'xyth'),
        }
        self.t0 = time.time()
        self.wheel_last_t = None
        self.gyro_last_t = None
        self.gyro_yaw = 0.0
        self.gyro_bias = None        # rad/s, measured while still at startup
        self.gyro_cal = []           # samples collected during calibration
        self.rows = []
        self.jumps = []              # (t, size_m) teleports seen on /vo/odom
        self.vo_prev = None
        self.vo_peak = 0.0           # fastest real motion seen, m/s

        self.create_subscription(Odometry, '/vo/odom', self._vo, qos_profile_sensor_data)
        self.create_subscription(Vector3, '/wheel_state', self._wheels, qos_profile_sensor_data)
        self.create_subscription(Imu, '/gyro/base', self._gyro, qos_profile_sensor_data)
        self.create_timer(0.25, self._draw)

    def _vo(self, m):
        s = self.src['cuvslam']
        s.seen()
        px, py = m.pose.pose.position.x, m.pose.pose.position.y
        pth = yaw_of(m.pose.pose.orientation)

        # Watch the RAW stream for teleports before it is folded into totals.
        now = time.time()
        if self.vo_prev is not None:
            dt = now - self.vo_prev[0]
            step = math.hypot(px - self.vo_prev[1], py - self.vo_prev[2])
            if step > JUMP_M:
                self.jumps.append((now - self.t0, step))
            else:
                if dt > 0:
                    self.vo_peak = max(self.vo_peak, step / dt)
                self._fuse(px, py, pth)
        self.vo_prev = (now, px, py, pth)

        s.advance(px, py, pth)

    def _fuse(self, px, py, pth):
        """Distance from cuVSLAM, heading from the gyro.

        Measured 2026-08-15 on a 2 m out-and-back. cuVSLAM's heading tracked the
        gyro to within 0.13 deg going FORWARD, then drifted +7.68 deg on the way
        BACK — reversing is its weak case, because features shrink toward the
        image centre and new ones must enter at the edges where they are worst
        observed. A gyro does not care which way the robot is moving.

        So each step is de-rotated out of cuVSLAM's heading and re-applied using
        the gyro's. Replaying the run this way took the endpoint error from
        28.4 cm to 12.2 cm.

        The residual is distance, not heading: that same return leg registered
        186.5 cm against the outbound 198.0 cm, so reversing under-reads by ~6%.
        cuVSLAM is currently the only translation source, so nothing can correct
        it. WHEEL ODOMETRY IS THAT CORRECTION — encoders measure distance without
        caring about visual texture or direction. Blend it in here once the ESP32
        is publishing at 20 Hz again.
        """
        if self.gyro_bias is None:
            return                      # gyro not calibrated yet; nothing to fuse
        _, ox, oy, oth = self.vo_prev
        dx, dy = px - ox, py - oy
        c, s = math.cos(-oth), math.sin(-oth)          # into the body frame
        f = self.src['FUSED']
        f.seen()
        f.step_body(c * dx - s * dy, s * dx + c * dy, self.gyro_yaw)

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

        # A MEMS gyro has a constant offset; integrate it and the heading walks
        # away at a steady rate. Measured 2026-08-15: 9.75 deg over 125 s parked,
        # i.e. 0.078 deg/s — nearly 5 deg per minute, against a 10 deg gate.
        # So: hold still at startup, average the offset, subtract it forever after.
        if self.gyro_bias is None:
            self.gyro_cal.append(m.angular_velocity.z)
            if now - self.t0 >= GYRO_BIAS_S and len(self.gyro_cal) > 50:
                self.gyro_bias = sum(self.gyro_cal) / len(self.gyro_cal)
                self.gyro_last_t = now
            return

        if self.gyro_last_t is not None:
            dt = now - self.gyro_last_t
            if 0 < dt < 0.5:
                self.gyro_yaw = wrap(self.gyro_yaw + (m.angular_velocity.z - self.gyro_bias) * dt)
                s.th = self.gyro_yaw
        self.gyro_last_t = now

    def _draw(self):
        now = time.time()
        for s in self.src.values():
            s.tick_rate(now)
        el = now - self.t0

        out = ['' if self.args.plain else '\033[H\033[J']
        out.append(f'  PHASE 1 — where does each sensor think it is?     t+{el:6.1f}s')
        out.append('')
        out.append('  source        x cm     y cm    th deg   straight cm   path cm      Hz')
        out.append('  ' + '-' * 68)
        for key in ORDER:
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

        if self.jumps:
            out.append(f'  ⚠ {len(self.jumps)} POSE JUMP(S) — tracking was lost. '
                       f'THIS RUN IS INVALID, restart it.')
        if self.vo_peak > 0.25:
            out.append(f'  ⚠ peak speed {self.vo_peak * 100:.0f} cm/s — too fast, '
                       f'keep it under 25 cm/s or the tracker loses features')
        elif self.vo_peak > 0:
            out.append(f'  push speed peak {self.vo_peak * 100:.0f} cm/s — good')

        if self.src['gyro'].n:
            if self.gyro_bias is None:
                out.append(f'  ⏳ CALIBRATING GYRO — KEEP THE ROVER STILL '
                           f'({max(0.0, GYRO_BIAS_S - el):.1f}s left)')
            else:
                out.append(f'  gyro bias removed: {math.degrees(self.gyro_bias):+.4f} deg/s '
                           f'({math.degrees(self.gyro_bias) * 60:+.2f} deg/min)')
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
            v for key in ORDER
            for v in (round(self.src[key].x, 5), round(self.src[key].y, 5),
                      round(math.degrees(self.src[key].th), 3),
                      round(self.src[key].straight, 5), round(self.src[key].path, 5),
                      round(self.src[key].hz, 2))])

    # ── verdict ────────────────────────────────────────────────────────────
    def verdict(self):
        a = self.args
        print('\n\n  ── result ' + '─' * 58)
        for key in ORDER:
            s = self.src[key]
            if s.n == 0:
                print(f'  {key:<10} never published — no opinion')
                continue
            if s.gives == 'th':
                bias = ('' if self.gyro_bias is None
                        else f'   [bias {math.degrees(self.gyro_bias):+.4f} deg/s removed]')
                print(f'  {key:<10} heading {math.degrees(s.th):+8.2f} deg   '
                      f'({s.n} msgs, {s.hz:.1f} Hz){bias}')
            else:
                print(f'  {key:<10} straight {s.straight * 100:7.1f} cm   path {s.path * 100:7.1f} cm   '
                      f'heading {math.degrees(s.th):+7.2f} deg   ({s.n} msgs, {s.hz:.1f} Hz)')

        print()
        vo = self.src['cuvslam']
        if self.jumps:
            print(f'  ✗ RUN INVALID — {len(self.jumps)} pose jump(s), tracking was lost:')
            for t, d in self.jumps[:5]:
                print(f'      t={t:.1f}s   the pose teleported {d * 100:.0f} cm in one frame')
            print(f'    Peak speed was {self.vo_peak * 100:.0f} cm/s. cuVSLAM matches features')
            print('    between frames; move too fast and there is no overlap to match, so it')
            print('    re-initialises and every later number is measured from a wrong origin.')
            print('    NOT GRADED. Push slower (under ~25 cm/s) and run it again.')
            self._write_csv()
            return
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

        # The fused answer, graded on the same bar as cuvslam alone.
        fu = self.src['FUSED']
        if fu.n or fu.path > 0:
            print()
            if a.expect:
                lo, hi = a.expect * 0.95, a.expect * 1.05
                print(f'  FUSED scale: {fu.straight * 100:.1f} cm '
                      f'({(fu.straight - a.expect) / a.expect * 100:+.1f}%)  ->  '
                      f'{"PASS" if lo <= fu.straight <= hi else "FAIL"}')
            elif a.ret:
                print(f'  FUSED drift: {fu.straight * 100:.1f} cm from the start  ->  '
                      f'{"PASS" if fu.straight <= 0.10 else "FAIL"} (want <= 10.0 cm)')
            elif a.spin:
                e = math.degrees(abs(wrap(math.radians(a.spin) - fu.th)))
                print(f'  FUSED heading: error {e:.2f} deg  ->  '
                      f'{"PASS" if e <= 10.0 else "FAIL"}')
            if vo.straight > 1e-6:
                better = vo.straight / fu.straight if fu.straight > 1e-6 else float('inf')
                print(f'    distance from cuVSLAM, heading from the gyro — '
                      f'{better:.1f}x better than cuvslam alone'
                      if better > 1.05 else
                      f'    distance from cuVSLAM, heading from the gyro')

        path = self._write_csv()
        print(f'\n  log: {path}\n')

    def _write_csv(self):
        # /logs is bind-mounted to the host's rover/logs, so a run survives the
        # container being torn down. Fall back only if it is not mounted.
        d = '/logs' if os.path.isdir('/logs') else os.path.expanduser('~/rover-logs')
        os.makedirs(d, exist_ok=True)
        p = os.path.join(d, f'compare-{datetime.now():%Y%m%d-%H%M%S}.csv')
        head = ['t']
        for k in ORDER:
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
    ap.add_argument('--plain', action='store_true', help='do not clear the screen (for logging)')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Compare(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C gives KeyboardInterrupt, SIGTERM (timeout, systemd) gives
        # ExternalShutdownException. Both must still print the verdict — a run
        # that ends without one has wasted a tape measurement.
        pass
    finally:
        try:
            node.verdict()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
