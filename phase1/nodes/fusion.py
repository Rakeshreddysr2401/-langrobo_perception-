#!/usr/bin/env python3
"""fusion.py — the pose estimator, with no ROS in it.

WHY IT LIVES HERE AND NOT IN A NODE
    This algorithm was validated over two days of tape-measured runs. If the
    node that ships re-implemented it, that validation would apply to nothing.
    So it lives in one place: compare.py measures and grades it, fusion_node.py
    publishes it, and they cannot drift apart.

WHAT IT USES, AND WHY EACH
    No sensor here is good at everything, and one that is bad at a job does not
    get that job.

    heading   gyro z, bias re-measured continuously while the rover is still.
              Measured -0.2% to -0.9% across five turns, against cuVSLAM's +11%
              on a 90 deg turn. Corrected toward cuVSLAM only in a straight
              line, because rotation is visual odometry's weakest case.
    distance  encoder ticks, which do not care about visual texture or direction
              of travel. cuVSLAM supplies the DIRECTION, the wheels the
              MAGNITUDE. The calibration freezes while turning, because the
              wheels scrub 1.60x front-to-rear in a pivot and 1.00x straight.
    tilt      roll and pitch from the ACCELEROMETER, which measures gravity and
              therefore never drifts, smoothed with gyro x and y.

    NOT used for position: the accelerometer. Integrating it twice grows error
    as t^2 -- 180 m in a minute. Its value is the gravity vector, not motion.

FAILURE IS THE POINT
    Every sensor here is covered by the others:

      cuVSLAM blind (<30 landmarks)  -> wheels + gyro
      cuVSLAM teleports              -> wheels + gyro
      cuVSLAM silent                 -> wheels + gyro, via pulse()
      wheels slip in a turn          -> gyro heading
      wheels stale                   -> cuVSLAM distance, last good scale
      gyro stale                     -> cuVSLAM heading

    Measured driving hard through 12 teleports with landmarks down to 17:
    cuVSLAM ended 79.6 cm from the start, the wheels 55.7 cm, and this 4.9 cm.
    Beating BOTH inputs is the whole point; an earlier version that kept
    listening to a blind cuVSLAM managed 42.4 cm, worse than the wheels alone.

USAGE
    f = PoseFusion()
    f.on_gyro(t, gx, gy, gz, ax, ay, az)     # 200 Hz
    f.on_ticks(t, lf, lr, rf, rr)            # 20 Hz
    f.on_wheels(t, velL, velR)               # 20 Hz
    f.on_landmarks(n)                        # 1 Hz
    f.on_vo(t, x, y, yaw)                    # 30 Hz
    f.pulse(t)                               # 20 Hz, independent heartbeat
    f.x, f.y, f.yaw, f.roll, f.pitch, f.health()
"""
import math
import time

# Verified against a 200 cm tape push 2026-08-15: all four wheels within 3% of
# this, so ENCODER_CPR 1560 is correct and no per-wheel constant is needed.
METRES_PER_COUNT = math.pi * 0.085 / 1560.0

WHEEL_BASE_M = 0.34    # rover_firmware_v2.ino:100 — 34 cm between L/R wheel centres

# Rotation needs a DIFFERENT width, and this is not a fudge. This is a four-wheel
# SKID-STEER rover: it has no steering, so turning drags all four tyres sideways
# across the floor. The geometry that converts a left/right speed difference into
# a yaw rate is therefore not the physical 34 cm track -- it is an effective
# width that includes the scrub, and it is always larger.
#
# Measured 2026-08-21, logs/calibrate_rotation.py:
#
#     turn       wheels read   truth   ratio   implied width   peak rate
#     90 left       146.49       90     1.628      0.5534 m        -
#     360 left      556.49      360     1.546      0.5256 m     21.4 deg/s
#     360 left      548.34      360     1.523      0.5179 m     76.2 deg/s
#     360 left      553.08      360     1.536      0.5224 m     74.3 deg/s
#     360 left      551.49      360     1.532      0.5209 m     75.1 deg/s
#
# The four 360s span 1.5% and their mean is what is used -- four times the
# signal of a 90, and returning to the same floor line is far easier to judge
# than a right angle.
#
# Scrub is not a fixed property of the chassis. It varies with turn rate, tyre
# loading and floor, so this is a calibrated average, not a dimension.
# Re-measure on carpet.
#
# Using the physical 0.34 m made the wheels 63% wrong on every turn.
WHEEL_BASE_ROT_M = 0.5216

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

# A ground rover on a floor cannot be this far above or below where it started.
# Landmarks are NOT sufficient to catch a diverged tracker: measured 2026-08-22,
# cuVSLAM reported 95 landmarks -- healthy by every signal we had -- while
# claiming the rover was 17 m away and 21.7 m UNDERGROUND, frozen there. It had
# jumped once and settled into a confident wrong pose.
#
# The damage was not obvious either. FUSED takes DISTANCE from the encoders and
# DIRECTION from cuVSLAM, so the magnitudes stayed right while the directions
# went random: 37.4 m driven inside a 2.1 x 1.6 m box, a random walk that
# largely cancelled itself out.
#
# Gravity says which way is down and never drifts, so this is cheap and certain.
# 0.30 m allows a threshold or a shallow ramp and nothing more.
VO_MAX_Z = 0.30

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
YAW_TRUST_VO = 2.0e-5

# Do not accept a heading correction from cuVSLAM WHILE TURNING. Rotation is
# visual odometry's weakest case -- the scene sweeps, features leave the frame,
# and on this rig a pivot swings the camera at ~34 cm/s, past its ~25 cm/s
# tracking limit. Measured 2026-08-21 on a hand-set 90 deg turn: the gyro read
# 89.80 deg and cuVSLAM read 99.85. Correcting toward cuVSLAM there does not
# bound drift, it imports an 11% error into a reading that was already right.
# Below this rate the rover is driving straight or standing still, which is
# where cuVSLAM's heading is genuinely trustworthy.
YAW_CORRECT_MAX_RATE = 0.05      # rad/s, about 3 deg/s

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
# A source silent this long is not contributing, and somebody else must cover
# for it. Deliberately tighter than STALE_S (which is only a display hint):
# by the time a sensor has been quiet half a second the pose has already stopped
# being updated by it, and waiting longer just loses more travel.
COVER_STALE_S = 0.5

STILL_WHEEL_MS = 0.010

STILL_VO_MS = 0.010

# Longest gap between /wheel_state messages we will still integrate across.
# The ESP32 should publish at 20 Hz but currently manages 1.000 Hz, so anything
# tighter than this throws away every sample and the wheels row reads a
# permanent 0.0 while messages are visibly arriving.
WHEEL_MAX_DT = 1.5


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Source:
    """One independent story about how the robot moved."""

    def __init__(self, name, gives):
        self.name = name
        self.gives = gives          # 'xyth' or 'th'
        self.x = self.y = self.th = 0.0
        # th wraps to +/-180, which makes a 360 deg test unreadable: every source
        # correctly reports "about 0" after a full turn and you cannot tell that
        # from having barely moved. th_total accumulates the wrapped DELTAS, so a
        # full turn reads 360 and two turns read 720.
        self.th_total = 0.0
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
        self._set_th(wrap(th - fth))
        self._accumulate_path()

    def _set_th(self, new_th):
        self.th_total += wrap(new_th - self.th)
        self.th = new_th

    def integrate(self, vx, wz, dt):
        """Dead reckoning for sources that give velocity, not pose."""
        self._set_th(wrap(self.th + wz * dt))
        d = vx * dt
        self.x += d * math.cos(self.th)
        self.y += d * math.sin(self.th)
        self._accumulate_path()

    def step_body(self, bx, by, th):
        """Add a step already expressed in the body frame, with heading supplied."""
        self._set_th(wrap(th))
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


class PoseFusion:
    """One fused pose from cuVSLAM, the IMU and four wheel encoders."""

    def __init__(self):
        self.out = Source('FUSED', 'xyth')   # the fused estimate itself

        # gyro
        self.t0 = time.time()
        self.gyro_bias = None        # rad/s, None until calibrated
        self.gyro_cal = []
        self.gyro_yaw = 0.0
        self.gyro_prev_yaw = 0.0
        self.gyro_last_t = None
        self.gyro_xyz = (0.0, 0.0, 0.0)
        self.accel = (0.0, 0.0, 0.0)
        self.gyro_n = 0
        self.gyro_last_msg = 0.0
        self.bias_updates = 0

        # attitude, from gravity
        self.roll = None
        self.pitch = None
        self.pitch_still = []

        # cuVSLAM
        self.vo_prev = None
        self.vo_th = 0.0             # cuVSLAM heading, zeroed at the first sample
        self.vo_first = None
        self.vo_n = 0
        self.vo_last_msg = 0.0
        self.vo_speed = 0.0
        self.vo_speed_t = 0.0        # when vo_speed was last ACTUALLY updated
        self.vo_peak = 0.0
        self.vo_path_raw = 0.0
        self.landmarks = -1
        self.landmarks_min = None
        self.jumps = []

        # wheels
        self.wheel_prev = (0.0, 0.0)
        self.wheel_n = 0
        self.wheel_last_msg = 0.0
        self.wheel_moved_l = 0
        self.wheel_moved_r = 0
        self.wheel_dead_side = None
        self.ticks = None
        self.tick_path = 0.0
        self.tick_travel = 0.0
        self.tick_at_fuse = 0.0

        # fused state
        self.fused_yaw = 0.0
        self.enc_scale = 1.0
        self.enc_path_dr = 0.0
        self.enc_path_at_fuse = 0.0
        self.enc_corrections = 0

        # health counters
        self.dr_steps = 0
        self.cover_vo = 0
        self.cover_gyro = 0
        self.cover_enc = 0
        self.vo_unhealthy = 0        # frames cuVSLAM was dropped, low landmarks
        self.vo_implausible = 0      # frames cuVSLAM claimed an impossible z
        self.vo_z = 0.0              # latest cuVSLAM z, for the health report
        self.yaw_held = 0
        self.scale_held = 0

    # ── outputs ────────────────────────────────────────────────────────────
    @property
    def x(self):
        return self.out.x

    @property
    def y(self):
        return self.out.y

    @property
    def yaw(self):
        return self.out.th

    @property
    def ready(self):
        """True once the gyro bias is measured; before that there is no pose."""
        return self.gyro_bias is not None

    def health(self):
        now = time.time()
        return {
            'ready': self.ready,
            'vo_alive': self.vo_n > 0 and (now - self.vo_last_msg) <= COVER_STALE_S,
            'wheels_alive': self.wheel_n > 0 and (now - self.wheel_last_msg) <= COVER_STALE_S,
            'gyro_alive': self.gyro_n > 0 and (now - self.gyro_last_msg) <= COVER_STALE_S,
            'landmarks': self.landmarks,
            'vo_dropped': self.vo_unhealthy,
            'vo_implausible': self.vo_implausible,
            'vo_z': round(self.vo_z, 3),
            'dead_reckoned': self.dr_steps,
            'dr_metres': self.enc_path_dr,
            'jumps': len(self.jumps),
            'enc_scale': self.enc_scale,
            'wheel_dead_side': self.wheel_dead_side,
            'bias_deg_s': None if self.gyro_bias is None else math.degrees(self.gyro_bias),
            'roll': self.roll,
            'pitch': self.pitch,
        }

    # ── inputs ─────────────────────────────────────────────────────────────
    def on_landmarks(self, n):
        self.landmarks = int(n)
        if self.landmarks_min is None or self.landmarks < self.landmarks_min:
            self.landmarks_min = self.landmarks

    def on_wheels(self, t, velL, velR):
        self.wheel_n += 1
        self.wheel_last_msg = t
        if abs(velL) > 0.01:
            self.wheel_moved_l += 1
        if abs(velR) > 0.01:
            self.wheel_moved_r += 1
        lo, hi = sorted((self.wheel_moved_l, self.wheel_moved_r))
        if lo == 0 and hi >= DEAD_SIDE_SAMPLES:
            self.wheel_dead_side = 'LEFT' if self.wheel_moved_r > 0 else 'RIGHT'
        else:
            self.wheel_dead_side = None
        self.wheel_prev = (velL, velR)

    def on_ticks(self, t, lf, lr, rf, rr):
        """Signed body distance from raw cumulative counts.

        NOT from Source.path, which accumulates in 5 mm chords. At 30 Hz that
        increment is usually exactly zero, so dead reckoning asked "how far since
        the last frame?", got 0, and moved nothing -- it reported carrying 0
        frames through 32 teleports while the rover was really moving.
        """
        v = (int(lf), int(lr), int(rf), int(rr))
        if self.ticks is not None:
            d = [c - p for c, p in zip(v, self.ticks)]
            left = 0.5 * (d[0] + d[1])
            right = 0.5 * (d[2] + d[3])
            step = 0.5 * (left + right) * METRES_PER_COUNT
            self.tick_path += step
            self.tick_travel += abs(step)
        self.ticks = v

    def on_gyro(self, t, gx, gy, gz, ax, ay, az):
        self.gyro_n += 1
        self.gyro_last_msg = t
        self.accel = (ax, ay, az)
        self.gyro_xyz = (gx, gy, gz)

        if self.gyro_bias is None:
            self.gyro_cal.append(gz)
            if t - self.t0 >= GYRO_BIAS_S and len(self.gyro_cal) > 50:
                self.gyro_bias = sum(self.gyro_cal) / len(self.gyro_cal)
                self.gyro_last_t = t
            return

        if self._still():
            self.gyro_bias += (gz - self.gyro_bias) * BIAS_ADAPT
            self.bias_updates += 1

        if self.gyro_last_t is not None:
            dt = t - self.gyro_last_t
            if 0 < dt < 0.5:
                self.gyro_yaw = wrap(self.gyro_yaw + (gz - self.gyro_bias) * dt)
                self._attitude_step(gx, gy, dt)
        self.gyro_last_t = t

    def on_vo(self, t, px, py, pth, pz=0.0):
        """A cuVSLAM pose. Teleports and divergence are rejected here."""
        self.vo_n += 1
        self.vo_last_msg = t
        self.vo_z = pz

        # Is it even claiming something physically possible? See VO_MAX_Z.
        # Checked before anything else, because a diverged tracker still reports
        # healthy landmarks and a perfectly steady rate.
        if abs(pz) > VO_MAX_Z:
            self.vo_implausible += 1
            self._deadreckon()
            self.vo_prev = (t, px, py, pth)
            return
        if self.vo_first is None:
            self.vo_first = (px, py, pth)
        fx, fy, fth = self.vo_first
        dx, dy = px - fx, py - fy
        c, s = math.cos(-fth), math.sin(-fth)
        self.vo_th = wrap(pth - fth)
        _ = (c * dx - s * dy, s * dx + c * dy)   # zeroed pose, kept for symmetry

        if self.vo_prev is not None:
            dt = t - self.vo_prev[0]
            step = math.hypot(px - self.vo_prev[1], py - self.vo_prev[2])
            speed = step / dt if dt > 0 else 0.0
            if step > JUMP_M or speed > MAX_PUSH_MS:
                self.jumps.append((t - self.t0, step, self.landmarks))
                self._deadreckon()
            else:
                self.vo_speed = speed
                self.vo_speed_t = time.time()
                self.vo_peak = max(self.vo_peak, speed)
                self._fuse(px, py, pth)
        self.vo_prev = (t, px, py, pth)

    def pulse(self, t):
        """Independent heartbeat: keep the pose alive when cuVSLAM goes quiet.

        A teleport is handled in on_vo, because a message still arrives. This is
        the other failure: no message at all. Nothing else would notice, because
        every other path is triggered by an incoming cuVSLAM pose.
        """
        if self.gyro_bias is None:
            return
        vo_gone = self.vo_n == 0 or (t - self.vo_last_msg) > COVER_STALE_S
        w_alive = self.wheel_n > 0 and (t - self.wheel_last_msg) <= COVER_STALE_S
        if vo_gone and w_alive:
            self._deadreckon()
            self.cover_vo += 1

    # ── the algorithm ──────────────────────────────────────────────────────

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

        # Is cuVSLAM worth listening to at all? Refusing individual teleports is
        # not enough: between them the messages keep arriving and the DIRECTION
        # in them is just as corrupted. Measured 2026-08-21 with landmarks down
        # to 4, FUSED took its heading from cuVSLAM throughout and landed 42.4 cm
        # from the start, while the wheels ALONE managed 17.4 cm. Fusion did
        # worse than its own worst input because it was still listening to the
        # broken one.
        #
        # Landmarks are the honest health signal -- a dead tracker still emits a
        # confident pose at a perfect 30 Hz. Below LOW_LANDMARKS there is not
        # enough of the world in view to solve for motion, so cuVSLAM is dropped
        # entirely and the pose runs on wheels and gyro until it recovers.
        if 0 <= self.landmarks < LOW_LANDMARKS:
            self.vo_unhealthy += 1
            self._deadreckon()
            return

        _, ox, oy, oth = self.vo_prev
        dx, dy = px - ox, py - oy
        c, s = math.cos(-oth), math.sin(-oth)          # into the body frame
        bx, by = c * dx - s * dy, s * dx + c * dy

        # ── heading: gyro, pulled slowly onto cuVSLAM so bias cannot run away ──
        vo_th = self.vo_th
        # Recursive, on its OWN previous value. Written as
        #     fused = gyro_yaw + err * k
        # the correction never accumulated -- every message recomputed it from
        # gyro_yaw, so the result stayed 99.9% gyro no matter how long it ran.
        # Measured 2026-08-15: parked 107 s, cuVSLAM held +0.01 deg, the gyro
        # walked to -9.52 deg, and FUSED reported -9.51. The filter was doing
        # nothing at all. Integrating the gyro's STEP onto the fused state and
        # then pulling that toward cuVSLAM is what actually bounds the drift.
        if self.gyro_n and (time.time() - self.gyro_last_msg) <= COVER_STALE_S:
            self.fused_yaw = wrap(self.fused_yaw
                                  + wrap(self.gyro_yaw - self.gyro_prev_yaw))
            turning = abs(self.gyro_xyz[2] - self.gyro_bias) > YAW_CORRECT_MAX_RATE
            if not turning:
                self.fused_yaw = wrap(self.fused_yaw
                                      + wrap(vo_th - self.fused_yaw) * YAW_TRUST_VO)
            else:
                self.yaw_held += 1
        else:
            # The gyro has gone quiet. Its step is the thing being integrated, so
            # a stale one would hold the heading frozen through a real turn --
            # worse than having no gyro at all. cuVSLAM's heading does not drift,
            # so hand the job straight to it rather than coasting on a dead input.
            self.fused_yaw = vo_th
            self.cover_gyro += 1
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
        enc_ok = (self.wheel_dead_side is None and self.wheel_moved_l
                  and self.wheel_moved_r)
        self.tick_at_fuse = self.tick_path
        w_fresh = self.wheel_n > 0 and (time.time() - self.wheel_last_msg) <= COVER_STALE_S
        if not (enc_ok and w_fresh):
            self.cover_enc += 1        # cuVSLAM carrying distance on its own

        # Do not re-calibrate the encoder scale WHILE TURNING. Measured
        # 2026-08-21 through a 360 deg pivot, the two wheels on one side --
        # bolted to the same chassis and driven by the same BTS7960, so
        # mechanically obliged to sweep the same arc -- disagreed by 1.60x on
        # the left and 1.29x on the right. Going straight the same wheels agree
        # to 1.00x and 1.03x. So the encoders are an excellent distance
        # reference in a straight line and a poor one mid-turn, and folding
        # turn samples into the ratio would drag a good calibration off with
        # scrub that is not travel at all.
        turning_now = (self.gyro_bias is not None
                       and abs(self.gyro_xyz[2] - self.gyro_bias) > YAW_CORRECT_MAX_RATE)
        if turning_now:
            self.scale_held += 1
        if (enc_ok and w_fresh and not turning_now
                and self.vo_path_raw > SCALE_MIN_TRAVEL):
            ratio = (self.tick_travel - self.enc_path_dr) / self.vo_path_raw
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

        f = self.out
        f.seen()
        f.step_body(bx, by, self.fused_yaw)

    def _deadreckon(self):
        """Advance FUSED on wheels and gyro alone — no cuVSLAM at all.

        Used for two different failures. A teleport, where a message arrives and
        is nonsense; and cuVSLAM being unhealthy for a stretch, where the
        messages keep coming and the DIRECTION in them is untrustworthy. The
        second is the one that hurt: on 2026-08-21, with landmarks down to 4,
        FUSED kept taking its heading from cuVSLAM and finished 42.4 cm from the
        start while the wheels alone managed 17.4 cm. Fusion did worse than its
        own worst input, because it was still listening to the broken one.

        Distance comes from raw cumulative ticks, which are signed, so reversing
        subtracts instead of adding. Heading comes from the gyro, integrated onto
        the filter's own value with no cuVSLAM correction.
        """
        if self.gyro_bias is None:
            return
        step = self.tick_path - self.tick_at_fuse
        self.tick_at_fuse = self.tick_path

        f = self.out
        f.seen()          # alive even when standing still; see the note below

        self.fused_yaw = wrap(self.fused_yaw
                              + wrap(self.gyro_yaw - self.gyro_prev_yaw))
        self.gyro_prev_yaw = self.gyro_yaw

        if abs(step) < 1e-9:
            return
        self.enc_path_dr += abs(step)
        f.step_body(step, 0.0, self.fused_yaw)
        self.dr_steps += 1

    def _still(self):
        """Is the rover definitely not moving?

        Deliberately conservative — a false 'still' while turning would absorb
        real rotation into the bias and permanently corrupt the heading. So it
        needs BOTH independent witnesses to agree there is no motion: the wheels
        (which cannot be fooled by a textureless scene) and cuVSLAM (which cannot
        be fooled by wheels spinning on a slippery floor).

        FOUND ON THE FLOOR 2026-09-02: it could still be fooled, because
        `vo_speed` is a WITNESS THAT GOES SILENT, not one that reports "unknown".
        It is only ever written inside on_vo's accepted branch -- never during a
        low-landmark dropout or a rejected jump -- so a landmark dip mid-turn
        freezes it at whatever it read a moment before, often near zero right as
        a rotation begins. A hand-rotated ~90 deg turn, landmarks dipping
        repeatedly throughout it (vo_dropped +28 that run), was read by cuVSLAM
        as ~88 deg and by FUSED as ~2 deg: `_still()` was reading a stale "not
        moving" off a witness that had stopped testifying, adapted gyro_bias
        toward the real, large turn rate the whole time, and ate the rotation.
        Same failure the docstring above already worried about, from an angle it
        didn't cover: not "cuVSLAM is fooled", but "cuVSLAM went quiet and we
        kept using its last answer".

        Fix: a witness that has not spoken recently does not get to vote "still".
        """
        if self.wheel_n == 0 or self.vo_n == 0:
            return False
        if time.time() - self.wheel_last_msg > 1.0:
            return False                      # no fresh wheel data; do not guess
        if time.time() - self.vo_speed_t > COVER_STALE_S:
            return False                      # vo_speed is stale; do not guess
        velL, velR = self.wheel_prev
        if abs(velL) > STILL_WHEEL_MS or abs(velR) > STILL_WHEEL_MS:
            return False
        return self.vo_speed < STILL_VO_MS

    def _attitude_step(self, gx, gy, dt):
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
        ax, ay, az = self.accel
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
        self.pitch = wrap(k * (self.pitch + gy * dt)
                          + (1.0 - k) * pitch_acc)
        self.roll = wrap(k * (self.roll + gx * dt)
                         + (1.0 - k) * roll_acc)

        # While the rover is still, the accelerometer alone IS the mount pitch:
        # the rig is level, so anything left over is the camera sitting nose-up
        # or nose-down relative to base_link.
        if self.gyro_bias is not None and abs(self.gyro_xyz[2] - self.gyro_bias) < 0.01:
            self.pitch_still.append(pitch_acc)
            if len(self.pitch_still) > 2000:
                self.pitch_still.pop(0)
