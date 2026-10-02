"""turn_gate.py — is the rover turning at this moment? (2026-10-03)

A depth frame taken while the rover turns is placed in odom with a pose that
is a few hundredths of a second off, and at 1 rad/s that is degrees: the same
chair leg lands 3-10 cm beside itself. Measured on the floor, 2026-10-03,
nvblox at a 3 cm slice, four goal_exec turns of 25 deg in place:

    cells 3-10 cm from a real object   painted while turning 417, still 88
    the pose itself after the turns    off by one 2.5 cm cell

So the smear is the frames painted DURING the turn, not the pose after it.
Both depth maps (nvblox via depth_gate.py, the 1 cm memory in
phase3/nodes/depth_obstacles.py) keep a frame only if the gyro showed no
turn faster than WZ_MAX from BEFORE_S before the frame to the latest sample.
Straight driving and standing still map as before; a turn in place maps
nothing until it stops (one frame later).

No gyro (dead, or not started): every frame passes, as before this gate, and
it says so once -- the maps must not go blind because the gyro did.

The gyro is 200 Hz: a user that needs depth only now and then (goal_exec,
reach) calls set_active() with its depth subscription, so it does not wake
200 times a second all day. Frames from before a fresh start + BEFORE_S are
dropped: there is no gyro history yet to clear them.
"""
import collections

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

GYRO = '/gyro/base'
WZ_MAX = 0.15          # rad/s (8.6 deg/s): MPPI's gentle steering passes, a turn in place does not
BEFORE_S = 0.20        # s before the frame's stamp: the turn that just ended still counts
STALE_S = 0.5          # s without a gyro sample = no gyro
STARTUP_S = 3.0        # s after a start in which an empty history is discovery, not a dead gyro


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class TurnGate:
    def __init__(self, node, wz_max=WZ_MAX, active=True):
        self.node, self.wz_max = node, wz_max
        self.hist = collections.deque(maxlen=400)      # (stamp, |wz|), ~2 s at 200 Hz
        self.passed = self.dropped = self.blind = 0
        self.warned = False
        self.sub, self.since = None, 0.0
        self.set_active(active)

    def set_active(self, on: bool) -> None:
        if on and self.sub is None:
            self.hist.clear()
            self.since = self.node.get_clock().now().nanoseconds * 1e-9
            self.sub = self.node.create_subscription(Imu, GYRO, self._gyro, qos_profile_sensor_data)
        elif not on and self.sub is not None:
            self.node.destroy_subscription(self.sub)
            self.sub = None

    def _gyro(self, m):
        self.hist.append((stamp_s(m.header), abs(m.angular_velocity.z)))

    def still(self, t):
        """True: keep the frame stamped t (s). False: it was taken in a turn."""
        if t < self.since + BEFORE_S:
            self.dropped += 1                           # just started: no history to clear it
            return False
        if not self.hist and t < self.since + STARTUP_S:
            self.dropped += 1                           # the gyro subscription is still connecting
            return False
        if not self.hist or self.hist[-1][0] < t - STALE_S:
            if not self.warned:
                self.node.get_logger().warn(f'turn gate: no {GYRO} -- depth frames pass ungated')
                self.warned = True
            self.blind += 1
            return True
        self.warned = False
        lo = t - BEFORE_S
        turning = any(w > self.wz_max for s, w in self.hist if s >= lo)
        if turning:
            self.dropped += 1
        else:
            self.passed += 1
        return not turning

    def counts(self):
        return dict(passed=self.passed, dropped=self.dropped, ungated=self.blind)
