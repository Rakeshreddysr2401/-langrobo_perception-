#!/usr/bin/env python3
"""turn_shaper.py — make nav2's turns happen at the rate nav2 asked for.

    /cmd_vel_turn  (velocity_smoother output)  ->  /cmd_vel  (the ESP32)

WHY. The firmware turns wz into wheel speeds with the physical 0.34 m track,
but this four-wheel skid-steer turns as if the track were ~0.52 m (tyre
scrub, description/params.yaml effective_track), and in place it does not
turn at all below ~0.8 rad/s commanded. Measured (nav2.yaml, 2026-08-23):
1.0 -> 0.21, 2.0 -> 0.59, 3.0 -> 0.86 rad/s. nav2's RPP and Spin predict
their motion from the command, so they overshot, hunted, timed out and saw
collisions that were not there -- the owner's "it struggles in rotation".
goal_exec already compensates for itself; this does the same for nav2.

MAPPING, from phase3/config/turn_calibration.json (written by the spin
calibration; falls back to the 2026-08-23 table):
  turning in place (|vx| < VX_STILL): the command that GIVES the desired
      rate, interpolated on the measured table; a small nonzero turn is
      lifted to the smallest command that turns at all (the deadband)
  driving: wz * effective_track / physical_track (the kinematic part only:
      a moving wheel has no static deadband)
Clamped to +-WZ_MAX. linear.x passes through untouched. Nothing received for
STALE_S: nothing sent (the firmware stops on its own 0.5 s timeout).
"""
import json
import math
import time
from pathlib import Path

import numpy as np

CAL = Path(__file__).resolve().parent.parent / 'config' / 'turn_calibration.json'
DEFAULT_SPIN = [[0.15, 0.0], [0.3, 0.01], [0.5, 0.03], [0.8, 0.11], [1.0, 0.21], [2.0, 0.59], [3.0, 0.86]]
PHYSICAL_TRACK, EFFECTIVE_TRACK = 0.34, 0.5216
VX_STILL = 0.02
WZ_MAX = 2.5            # command cap: teleop pivots cleanly at 5.0; kept gentler for cuVSLAM
MIN_TURN = 0.02          # desired rad/s below this is "no turn"
MIN_REAL_TURN = 0.05     # a command giving less than this does not really turn it (0.3 -> 0.01)
STALE_S = 0.3


def load(path=CAL):
    try:
        d = json.loads(Path(path).read_text())
        spin = d.get('spin') or DEFAULT_SPIN
    except (OSError, ValueError):
        spin = DEFAULT_SPIN
    # one curve from both directions: |cmd| -> mean |actual|, kept monotonic
    by = {}
    for c, a in spin:
        by.setdefault(round(abs(c), 3), []).append(abs(a))
    cmds = sorted(by)
    acts = np.maximum.accumulate([float(np.mean(by[c])) for c in cmds])
    return np.array(cmds), np.array(acts)


def shape(vx, wz, cmds, acts):
    """(vx, wz) nav2 wants -> (vx, wz) to send."""
    if abs(wz) < MIN_TURN:
        return vx, 0.0
    if abs(vx) >= VX_STILL:
        return vx, max(-WZ_MAX, min(WZ_MAX, wz * EFFECTIVE_TRACK / PHYSICAL_TRACK))
    turning = acts >= MIN_REAL_TURN             # commands that really turn it
    lo = cmds[turning][0] if turning.any() else cmds[-1]
    want = abs(wz)
    c = float(np.interp(want, acts[turning], cmds[turning])) if turning.any() else cmds[-1]
    c = max(lo, c)                              # lift over the deadband
    return vx, math.copysign(min(WZ_MAX, c), wz)


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    class TurnShaper(Node):
        def __init__(self):
            super().__init__('turn_shaper')
            self.cmds, self.acts = load()
            self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.create_subscription(Twist, '/cmd_vel_turn', self._in, 10)
            src = CAL if CAL.exists() else 'the 2026-08-23 default table'
            self.get_logger().info(f'turn_shaper up: /cmd_vel_turn -> /cmd_vel, calibration from {src}')

        def _in(self, m):
            vx, wz = shape(m.linear.x, m.angular.z, self.cmds, self.acts)
            out = Twist()
            out.linear.x, out.angular.z = vx, wz
            self.pub.publish(out)

    rclpy.init()
    n = TurnShaper()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
