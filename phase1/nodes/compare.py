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
    gyro      /gyro/base     all six IMU axes: 3 rates + 3 accelerations
    wheels    /wheel_ticks   cumulative per-wheel counts (LF, LR, RF, RR)
    FUSED                    every source used for what it is actually good at

WHAT FUSED USES, AND WHY EACH
    No sensor here is good at everything, and one that is bad at a job does not
    get that job.

    heading   gyro z, integrated, bias measured at startup. Beat cuVSLAM by
              7.68 deg on a return leg. Pulled onto cuVSLAM's heading very
              slowly, because gyro bias walks and cuVSLAM's heading does not.
    distance  encoders when both sides are alive -- they do not care about
              texture or direction of travel, which is exactly where cuVSLAM
              fails. cuVSLAM supplies the DIRECTION, the wheels the MAGNITUDE.
              Falls back to cuVSLAM alone when the encoders cannot be trusted.
    tilt      roll and pitch from the ACCELEROMETER, which measures gravity and
              therefore never drifts, smoothed with gyro x and y. Used to keep a
              slope from being counted as floor distance.

    NOT used for position: the accelerometer. Integrating it twice grows error
    with t^2 and diverges by hundreds of metres. Its value is the gravity vector,
    not motion.
    NOT used at all: depth. It is a PRODUCT of the same stereo pair cuVSLAM
    already consumes, so it adds no independent information about where we are.
    It is Phase 2's input for mapping.

    Measured 2026-08-15 on a 2 m out-and-back, gyro heading alone took the
    endpoint error from 28.4 cm to 12.2 cm.

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
import json
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
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String

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
# Fastest a person plausibly pushes this rover by hand. Anything above it is the
# tracker re-initialising, not motion, however small the step.
MAX_PUSH_MS = 1.0
# Below this many tracked landmarks cuVSLAM has too little to match against. The
# operator reported jumps "near walls" and "coming back" — a blank wall with the
# IR emitter off carries almost no texture, so this is the number that should
# collapse just before a teleport. Recorded at every jump to test that.
LOW_LANDMARKS = 30

# Complementary-filter time constant for roll/pitch, seconds. Above it the
# accelerometer wins (absolute, no drift); below it the gyro wins (smooth, and
# immune to the fake tilt that acceleration puts on the accelerometer). A quarter
# second is long enough to ride out a shove and short enough to track a real ramp.
ATTITUDE_TAU = 0.25

# How hard the fused heading is pulled onto cuVSLAM's, per /vo/odom message.
# The gyro measured BETTER than cuVSLAM over a two-minute run (+7.68 deg of
# cuVSLAM error on a return leg), so the gyro must stay dominant -- but its
# residual bias still walks, and cuVSLAM's heading does not drift because it is
# re-measured against the world every frame. At 30 Hz this is a ~30 s time
# constant: long enough that a teleport cannot yank the heading, short enough to
# bound gyro drift over a long run.
YAW_TRUST_VO = 0.001

# How many samples of one-sided motion before we call the other side dead. At
# 20 Hz this is ~5 s of one wheel turning while the other reports nothing, which
# no ordinary manoeuvre produces.
DEAD_SIDE_SAMPLES = 100

# Encoder/cuVSLAM distance ratio is only meaningful once there is real travel to
# divide; below this both totals are dominated by noise.
SCALE_MIN_TRAVEL = 0.20
# cuVSLAM reads ~2% under against a tape, so the honest ratio sits near 1.02.
# Outside this band it is wheel slip or lost tracking, not calibration, and the
# last good scale is kept instead.
SCALE_LO, SCALE_HI = 0.85, 1.25

# Continuous gyro-bias tracking. Measured on 2026-08-15 across three runs, the
# offset was +0.0838, +0.1050 and +0.1808 deg/s -- it moves as the board warms,
# so a five-second average taken at startup goes stale and starts adding error
# instead of removing it. Per sample at ~200 Hz this is a ~25 s time constant:
# slow enough that noise cannot move it, fast enough to follow a warm-up.
BIAS_ADAPT = 2.0e-4
# Below these, both witnesses agree the rover is parked and the gyro is reading
# pure bias. Wheels in m/s, cuVSLAM in m/s.
STILL_WHEEL_MS = 0.010
STILL_VO_MS = 0.010

# Longest gap between /wheel_state messages we will still integrate across.
# The ESP32 should publish at 20 Hz but currently manages 1.000 Hz, so anything
# tighter than this throws away every sample and the wheels row reads a
# permanent 0.0 while messages are visibly arriving.
WHEEL_MAX_DT = 1.5

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
        self.wheel_prev = (0.0, 0.0)
        self.wheel_moved_l = 0       # samples this side reported real motion
        self.wheel_moved_r = 0
        self.wheel_dead_side = None
        self.gyro_last_t = None
        self.gyro_yaw = 0.0
        self.gyro_bias = None        # rad/s, measured while still at startup
        self.gyro_cal = []           # samples collected during calibration
        self.rows = []
        self.jumps = []              # (t, size_m, landmarks) teleports on /vo/odom
        self.vo_prev = None
        self.vo_peak = 0.0           # fastest real motion seen, m/s
        self.landmarks = -1          # latest from /vo/status, -1 = never seen
        self.landmarks_min = None
        self.roll = None             # rad, gravity + gyro complementary filter
        self.pitch = None
        self.pitch_still = []        # accel-only pitch while stationary = mount pitch
        self.fused_yaw = 0.0
        self.gyro_prev_yaw = 0.0
        self.bias_updates = 0
        self.vo_speed = 0.0
        self.enc_path_prev = 0.0
        self.vo_path_raw = 0.0       # cuVSLAM path before any encoder rescale
        self.enc_scale = 1.0         # encoder/cuVSLAM distance ratio, from totals
        self.enc_corrections = 0     # how many steps the encoders actually rescaled
        self.accel = (0.0, 0.0, 0.0)      # raw, base_link frame
        self.gyro_xyz = (0.0, 0.0, 0.0)   # raw rates, all three axes
        self.ticks = (0, 0, 0, 0)         # LF, LR, RF, RR cumulative counts

        self.create_subscription(Odometry, '/vo/odom', self._vo, qos_profile_sensor_data)
        self.create_subscription(Vector3, '/wheel_state', self._wheels, qos_profile_sensor_data)
        self.create_subscription(Imu, '/gyro/base', self._gyro, qos_profile_sensor_data)
        self.create_subscription(String, '/vo/status', self._status, 10)
        # Per-wheel cumulative counts. Velocity has to be integrated and loses
        # whatever a dropped message carried; a total never does. It is also the
        # only view that shows one wheel slipping, since velL averages the two
        # left encoders together and hides it.
        self.create_subscription(Quaternion, '/wheel_ticks', self._ticks,
                                 qos_profile_sensor_data)
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
            speed = step / dt if dt > 0 else 0.0

            # Two tests, not one. A fixed 15 cm threshold lets a 14.7 cm hop in a
            # single 33 ms frame through as "real motion" -- which reads as
            # 442 cm/s and then gets reported as the operator pushing too fast.
            # Nobody hand-pushes a rover at 4.4 m/s, so any step implying more
            # than MAX_PUSH_MS is a teleport whatever its size.
            if step > JUMP_M or speed > MAX_PUSH_MS:
                self.jumps.append((now - self.t0, step, self.landmarks))
            else:
                self.vo_speed = speed
                self.vo_peak = max(self.vo_peak, speed)
                self._fuse(px, py, pth)
        self.vo_prev = (now, px, py, pth)

        s.advance(px, py, pth)

    def _status(self, m):
        """Track cuVSLAM's landmark count — the number that should explain jumps.

        A teleport is the tracker failing to match features between frames. The
        landmark count is how many it currently holds, so if the operator's
        observation is right (jumps near walls, jumps reversing) this collapses
        first. Recorded per jump so the correlation is visible rather than
        argued about.
        """
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        n = d.get('landmarks')
        if isinstance(n, (int, float)):
            self.landmarks = int(n)
            if self.landmarks_min is None or self.landmarks < self.landmarks_min:
                self.landmarks_min = self.landmarks

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

        DISTANCE comes from the encoders when they are trustworthy. That same
        return leg registered 186.5 cm against the outbound 198.0 cm, so cuVSLAM
        under-reads ~6% in reverse, and being the only translation source nothing
        could contradict it. Encoders measure distance without caring about
        visual texture or direction of travel, so where both exist we take
        cuVSLAM's DIRECTION and the encoders' MAGNITUDE.

        HEADING is corrected slowly toward cuVSLAM. The gyro is better over a
        two-minute run but its residual bias still walks; cuVSLAM's heading does
        not drift because it is measured against the world afresh each frame. A
        long time constant takes the drift-free part without importing its noise.

        TILT tips the step out of the horizontal plane, using roll/pitch from
        gravity, so driving up a ramp is not counted as floor distance.
        """
        if self.gyro_bias is None:
            return                      # gyro not calibrated yet; nothing to fuse
        _, ox, oy, oth = self.vo_prev
        dx, dy = px - ox, py - oy
        c, s = math.cos(-oth), math.sin(-oth)          # into the body frame
        bx, by = c * dx - s * dy, s * dx + c * dy

        # ── heading: gyro, pulled slowly onto cuVSLAM so bias cannot run away ──
        vo_th = self.src['cuvslam'].th
        # Recursive, on its OWN previous value. Written as
        #     fused = gyro_yaw + err * k
        # the correction never accumulated -- every message recomputed it from
        # gyro_yaw, so the result stayed 99.9% gyro no matter how long it ran.
        # Measured 2026-08-15: parked 107 s, cuVSLAM held +0.01 deg, the gyro
        # walked to -9.52 deg, and FUSED reported -9.51. The filter was doing
        # nothing at all. Integrating the gyro's STEP onto the fused state and
        # then pulling that toward cuVSLAM is what actually bounds the drift.
        self.fused_yaw = wrap(self.fused_yaw + wrap(self.gyro_yaw - self.gyro_prev_yaw))
        self.fused_yaw = wrap(self.fused_yaw
                              + wrap(vo_th - self.fused_yaw) * YAW_TRUST_VO)
        self.gyro_prev_yaw = self.gyro_yaw

        # ── distance: one scale factor from TOTALS, not a per-step ratio ──────
        # Per-step was systematically wrong, and the failure is worth keeping in
        # mind. w.path accumulates in 5 mm chords while vo_step is smooth
        # per-frame, so the per-step ratio was lumpy -- often exactly 0, sometimes
        # ~2x. The sanity band then did something perverse: a 0 ratio fell below
        # it and was REJECTED (keeping the full cuVSLAM step), while a large one
        # passed and was APPLIED. It could only ever inflate. Measured 2026-08-21
        # on a driven out-and-back: FUSED path 313 cm against cuVSLAM's 244 and
        # the wheels' own 254, and FUSED endpoint error 6.9 cm against cuVSLAM's
        # 2.2 -- fusion made the answer worse, which fusion must never do.
        #
        # Totals do not have that problem. Both sides are large numbers by the
        # time they matter, chord quantisation averages out, and the ratio is the
        # honest calibration between the two sensors.
        vo_step = math.hypot(bx, by)
        self.vo_path_raw += vo_step
        w = self.src['wheels']
        enc_ok = (self.wheel_dead_side is None and self.wheel_moved_l
                  and self.wheel_moved_r)
        if enc_ok and self.vo_path_raw > SCALE_MIN_TRAVEL:
            ratio = w.path / self.vo_path_raw
            # cuVSLAM reads ~2% under, so ~1.02 is expected. Anything outside
            # this band is wheel slip or a tracking failure, not calibration.
            if SCALE_LO < ratio < SCALE_HI:
                self.enc_scale = ratio
                self.enc_corrections += 1
        bx, by = bx * self.enc_scale, by * self.enc_scale

        # ── tilt: only the horizontal component is floor travel ───────────────
        if self.pitch is not None:
            bx *= math.cos(self.pitch)
            by *= math.cos(self.roll)

        f = self.src['FUSED']
        f.seen()
        f.step_body(bx, by, self.fused_yaw)

    def _wheels(self, m):
        s = self.src['wheels']
        s.seen()
        now = time.time()
        vx = (m.x + m.y) / 2.0
        wz = (m.y - m.x) / WHEEL_BASE_M

        # Watch for one side reading a hard zero while the other moves. Both LEFT
        # encoders died on 2026-08-15 (they worked three hours earlier), and the
        # arithmetic above turns that into a rover that appears to spin: velL is
        # pinned at 0 so wz = velR / 0.34 forever. Without this flag the wheels
        # row just prints a confident wrong heading, which is worse than printing
        # nothing. Detection is asymmetric on purpose -- both sides at zero is a
        # parked rover, not a fault.
        if abs(m.x) > 0.01:
            self.wheel_moved_l += 1
        if abs(m.y) > 0.01:
            self.wheel_moved_r += 1

        # Count samples, do not latch a boolean. The first version set the flag
        # the moment either side registered first and never cleared it, so a
        # right wheel that started a fraction of a second after the left was
        # reported dead for the rest of the run -- it wrongly accused a working
        # encoder on 2026-08-15, and the tick log disproved it. Now a side has to
        # stay silent through a meaningful amount of the OTHER side's motion, and
        # the accusation is withdrawn as soon as it does move.
        lo, hi = sorted((self.wheel_moved_l, self.wheel_moved_r))
        if lo == 0 and hi >= DEAD_SIDE_SAMPLES:
            self.wheel_dead_side = 'LEFT' if self.wheel_moved_r > 0 else 'RIGHT'
        else:
            self.wheel_dead_side = None
        if self.wheel_last_t is not None:
            dt = now - self.wheel_last_t
            if 0 < dt < WHEEL_MAX_DT:
                # Trapezoidal, not rectangular. The firmware reports the
                # INSTANTANEOUS velocity it measured over one 20 ms control
                # period, so at 1 Hz we are point-sampling a signal that updates
                # 50x faster. Assuming the last sample held for the whole second
                # is the crudest possible estimate; averaging consecutive samples
                # roughly halves the error when speed varies smoothly.
                pv, pw = self.wheel_prev
                s.integrate((vx + pv) / 2.0, (wz + pw) / 2.0, dt)
        self.wheel_last_t = now
        self.wheel_prev = (vx, wz)

    def _ticks(self, m):
        self.ticks = (int(m.x), int(m.y), int(m.z), int(m.w))

    def _gyro(self, m):
        s = self.src['gyro']
        s.seen()
        now = time.time()
        self.accel = (m.linear_acceleration.x, m.linear_acceleration.y,
                      m.linear_acceleration.z)
        self.gyro_xyz = (m.angular_velocity.x, m.angular_velocity.y,
                         m.angular_velocity.z)

        # A MEMS gyro has an offset; integrate it and the heading walks away at a
        # steady rate. Measure it while still at startup and subtract it after.
        #
        # BUT THE OFFSET IS NOT CONSTANT. Three runs on 2026-08-15 measured it at
        # +0.0838, +0.1050 and +0.1808 deg/s — better than a factor of two apart,
        # and it moves with temperature as the board warms. Subtracting a stale
        # five-second average is then worse than useless: on the third run it
        # OVER-corrected and the heading walked -9.52 deg in 107 s with the rover
        # standing still. Against a 10 deg gate that is nearly a failure produced
        # entirely by a parked robot.
        #
        # So the bias is re-measured continuously. Any moment the rover is known
        # to be still, whatever the gyro reads IS the bias by definition, and it
        # is eased toward that. See _still().
        if self.gyro_bias is None:
            self.gyro_cal.append(m.angular_velocity.z)
            if now - self.t0 >= GYRO_BIAS_S and len(self.gyro_cal) > 50:
                self.gyro_bias = sum(self.gyro_cal) / len(self.gyro_cal)
                self.gyro_last_t = now
            return

        if self._still():
            # Standing still, so every rad/s the gyro reports is offset. Ease
            # toward it rather than jumping: a single noisy sample must not
            # become the bias, but a slow warm-up drift must be followed.
            self.gyro_bias += (m.angular_velocity.z - self.gyro_bias) * BIAS_ADAPT
            self.bias_updates += 1

        if self.gyro_last_t is not None:
            dt = now - self.gyro_last_t
            if 0 < dt < 0.5:
                self.gyro_yaw = wrap(self.gyro_yaw + (m.angular_velocity.z - self.gyro_bias) * dt)
                s.th = self.gyro_yaw
                self._attitude(m, dt)
        self.gyro_last_t = now

    def _still(self):
        """Is the rover definitely not moving?

        Deliberately conservative — a false 'still' while turning would absorb
        real rotation into the bias and permanently corrupt the heading. So it
        needs BOTH independent witnesses to agree there is no motion: the wheels
        (which cannot be fooled by a textureless scene) and cuVSLAM (which cannot
        be fooled by wheels spinning on a slippery floor).
        """
        if self.src['wheels'].n == 0 or self.src['cuvslam'].n == 0:
            return False
        if time.time() - self.src['wheels'].last_msg > 1.0:
            return False                      # no fresh wheel data; do not guess
        velL, velR = self.wheel_prev
        if abs(velL) > STILL_WHEEL_MS or abs(velR) > STILL_WHEEL_MS:
            return False
        return self.vo_speed < STILL_VO_MS

    def _attitude(self, m, dt):
        """Roll and pitch, from the accelerometer and the two unused gyro axes.

        The accelerometer cannot give position -- integrating it twice makes the
        error grow with t^2 and it diverges by hundreds of metres. That is why
        the plan calls it unusable, and for position that is correct.

        But it measures the GRAVITY VECTOR, and gravity does not drift. Whichever
        way the rover tips, gravity still points down, so it gives roll and pitch
        as absolute angles with no accumulating error. That is something no other
        sensor on this rover provides: the gyro's roll/pitch rates drift, and
        cuVSLAM's tilt comes from the same visual tracking that teleports.

        The two are complementary in the literal sense. The gyro is smooth and
        instant but drifts; the accelerometer is absolute but noisy and confused
        by acceleration (push the rover forward and it reads a fake backwards
        tilt). So integrate the gyro for the short term and let the accelerometer
        pull it back slowly. TAU sets how slowly.

        Used for two things:
          * tilt-compensating the travel direction, so a slope or a threshold
            does not get counted as horizontal distance
          * measuring the camera's mount pitch, which was never put on a tape
            (TODO 6) and shifts the ground plane in Phase 2's map
        """
        ax, ay, az = (m.linear_acceleration.x, m.linear_acceleration.y,
                      m.linear_acceleration.z)
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        if not (5.0 < mag < 15.0):      # being shaken or dropped; gravity unreadable
            return

        # Gravity in base_link: level and still means (0, 0, +9.81).
        pitch_acc = math.atan2(-ax, math.hypot(ay, az))
        roll_acc = math.atan2(ay, az)

        if self.pitch is None:                 # first good sample seeds it
            self.pitch, self.roll = pitch_acc, roll_acc
            return

        k = ATTITUDE_TAU / (ATTITUDE_TAU + dt)      # gyro weight
        self.pitch = wrap(k * (self.pitch + m.angular_velocity.y * dt)
                          + (1.0 - k) * pitch_acc)
        self.roll = wrap(k * (self.roll + m.angular_velocity.x * dt)
                         + (1.0 - k) * roll_acc)

        # While the rover is still, the accelerometer alone IS the mount pitch:
        # the rig is level, so anything left over is the camera sitting nose-up
        # or nose-down relative to base_link.
        if self.gyro_bias is not None and abs(m.angular_velocity.z - self.gyro_bias) < 0.01:
            self.pitch_still.append(pitch_acc)
            if len(self.pitch_still) > 2000:
                self.pitch_still.pop(0)

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
        if self.landmarks >= 0:
            tag = '  ← TOO FEW, tracking is fragile here' \
                  if self.landmarks < LOW_LANDMARKS else ''
            out.append(f'  landmarks {self.landmarks:4d}  (lowest seen '
                       f'{self.landmarks_min if self.landmarks_min is not None else 0}){tag}')

        if self.pitch is not None:
            mp = ''
            if len(self.pitch_still) > 200:
                mp = (f'   camera mount pitch '
                      f'{math.degrees(sum(self.pitch_still) / len(self.pitch_still)):+.2f} deg')
            out.append(f'  tilt: roll {math.degrees(self.roll):+6.2f} deg   '
                       f'pitch {math.degrees(self.pitch):+6.2f} deg{mp}')
        if self.enc_corrections:
            out.append(f'  encoder scale {self.enc_scale:.4f}  '
                       f'(wheels {self.src["wheels"].path * 100:.1f} cm / cuvslam '
                       f'{self.vo_path_raw * 100:.1f} cm) — distance from the '
                       f'wheels, direction from cuVSLAM')

        w = self.src['wheels']
        if w.n and w.hz > 0 and w.hz < 15:
            out.append(f'  ⚠ wheels at {w.hz:.1f} Hz, not 20 — distance is a coarse '
                       f'estimate, treat it as a sanity check not a reference')

        if self.src['gyro'].n:
            if self.gyro_bias is None:
                out.append(f'  ⏳ CALIBRATING GYRO — KEEP THE ROVER STILL '
                           f'({max(0.0, GYRO_BIAS_S - el):.1f}s left)')
            else:
                still = '  STILL — retuning bias' if self._still() else ''
                out.append(f'  gyro bias removed: {math.degrees(self.gyro_bias):+.4f} deg/s '
                           f'({math.degrees(self.gyro_bias) * 60:+.2f} deg/min)   '
                           f'{self.bias_updates} updates{still}')
        out.append('')

        a = self.args
        if a.expect:
            out.append(f'  target: push {a.expect * 100:.0f} cm straight   '
                       f'pass band {a.expect * 0.95 * 100:.0f}–{a.expect * 1.05 * 100:.0f} cm')
        elif a.ret:
            out.append('  target: out and back to the SAME mark   pass: straight <= 10.0 cm')
        elif a.spin:
            out.append(f'  target: rotate {a.spin:.0f} deg by hand   pass: |error| <= 10 deg')
        if self.wheel_dead_side:
            out.append('')
            out.append(f'  !! {self.wheel_dead_side} encoders reading zero while the other side moves.')
            out.append('     IGNORE the wheels row — it will report a turn that is not happening.')
            out.append('     cuvslam, gyro and FUSED are unaffected.')
        out.append('')
        out.append('  Ctrl-C to finish and grade.')
        print('\n'.join(out), flush=True)

        self.rows.append(
            [round(el, 3)]
            + [v for key in ORDER
               for v in (round(self.src[key].x, 5), round(self.src[key].y, 5),
                         round(math.degrees(self.src[key].th), 3),
                         round(self.src[key].straight, 5),
                         round(self.src[key].path, 5),
                         round(self.src[key].hz, 2))]
            + [round(math.degrees(self.roll), 3) if self.roll is not None else '',
               round(math.degrees(self.pitch), 3) if self.pitch is not None else '',
               self.landmarks]
            + [round(v, 4) for v in self.accel]
            + [round(v, 5) for v in self.gyro_xyz]
            + [round(self.wheel_prev[0], 4), round(self.wheel_prev[1], 4)]
            + list(self.ticks))

    # ── verdict ────────────────────────────────────────────────────────────
    def verdict(self):
        a = self.args
        print('\n\n  ── result ' + '─' * 58)
        if self.wheel_dead_side:
            print(f'  NOTE: {self.wheel_dead_side} encoders read zero throughout while the other')
            print('  side moved. The wheels row below is arithmetic on a dead input —')
            print('  disregard it. The gates are graded on cuvslam / gyro / FUSED,')
            print('  none of which use the wheels.')
            print()
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
            for t, d, lm in self.jumps[:5]:
                lmtxt = '' if lm < 0 else f'   landmarks {lm}'
                print(f'      t={t:.1f}s   the pose teleported {d * 100:.0f} cm '
                      f'in one frame{lmtxt}')

            # Was it speed, or was it a featureless scene? These need different
            # answers -- push slower vs move somewhere with texture -- and
            # blaming speed by default sent the operator to fix the wrong thing.
            lms = [lm for _, _, lm in self.jumps if lm >= 0]
            starved = [lm for lm in lms if lm < LOW_LANDMARKS]
            print()
            print(f'    real peak push speed {self.vo_peak * 100:.0f} cm/s '
                  f'(teleports excluded)')
            if lms:
                print(f'    landmarks at the jumps: min {min(lms)}, median '
                      f'{sorted(lms)[len(lms) // 2]}, max {max(lms)}')
            if lms and len(starved) >= max(1, len(lms) // 2):
                print('    => MOSTLY A FEATURELESS SCENE, not speed. cuVSLAM had too')
                print('       few landmarks to match against. A blank wall with the IR')
                print('       emitter off has almost no texture. Run where the camera')
                print('       can see furniture, edges, clutter — not a bare wall.')
            elif self.vo_peak > 0.25:
                print('    => TOO FAST. Keep it under ~25 cm/s: cuVSLAM matches features')
                print('       between frames, and past that there is no overlap to match.')
            else:
                print('    => Neither speed nor landmark starvation stands out. Suspect')
                print('       reversing (measured much worse than forward) or motion blur.')
            print('    NOT GRADED. Fix the above and run it again.')
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
        # Everything the sensors actually said, not just the derived poses. A run
        # that produces a surprising number is worth re-reading afterwards, and
        # you cannot re-read what was never written down.
        head += ['roll_deg', 'pitch_deg', 'landmarks',
                 'accel_x', 'accel_y', 'accel_z',
                 'gyro_x', 'gyro_y', 'gyro_z',
                 'velL', 'velR', 'tick_LF', 'tick_LR', 'tick_RF', 'tick_RR']
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
