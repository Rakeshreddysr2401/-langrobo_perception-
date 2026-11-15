"""wheel_watch.py -- are the wheels doing what /cmd_vel asks? (reach, while it pursues a goal)

Real robot 2026-11-15 22:18: a goal given ~2 min after a Pi 5 power cycle. /wheel_state came at
20 Hz, but the wheels did not turn: goal_exec reported "turn stuck" twice, rl2 drove "-0.10 m/s,
+0.4-0.7 rad/s" for 25 s with the shield allowing every step and the rover moved 0.00 m, then tight
moves -- 96 s spent blaming the route for dead wheels. Two minutes later they answered.

The rule: for WINDOW_S the commanded wheel speeds (vx -/+ wz * HALF_TRACK, rl_drive's) asked for
at least MIN_CMD on some wheel on >= BUSY of the samples, and NO wheel's measured speed
(/wheel_state x = left, y = right, m/s; firmware/HARDWARE.md) went over MOVING. Then it says why:
  - no /wheel_state at all in the window          -> the ESP32's data is not arriving
  - the firmware's echo of vx (/wheel_state z) is 0 while vx was asked -> the commands do not arrive
  - else                                          -> the wheels do not turn (motor power, stall)
Small commands near a contact (the shield halving / stopping, 2026-11-15 08:38: ~0.05 m/s average)
stay below MIN_CMD / BUSY: that is a different problem and is not called dead wheels here.
"""
from collections import deque

HALF_TRACK = 0.17     # m, rl_drive.HALF_TRACK
WINDOW_S = 3.0        # the motor lag is 0.4-0.9 s (measured 2026-11-15): 3 s is well past it
MIN_CMD = 0.06        # m/s on a wheel: a command that turns it (above the deadband)
BUSY = 0.8            # share of the window's commands that must be that large
MOVING = 0.01         # m/s: a wheel above this is turning
ECHO_ZERO = 0.005     # m/s: the firmware's echoed vx counts as zero below this


class WheelWatch:
    def __init__(self):
        self.reset()

    def reset(self):
        self.cmd = deque()     # (t, vx, wz)
        self.state = deque()   # (t, left, right, echo_vx)

    def on_cmd(self, t, vx, wz):
        self.cmd.append((t, vx, wz))
        self._trim(t)

    def on_state(self, t, left, right, echo_vx):
        self.state.append((t, left, right, echo_vx))
        self._trim(t)

    def _trim(self, t):
        for q in (self.cmd, self.state):
            while q and q[0][0] < t - WINDOW_S:
                q.popleft()

    def verdict(self, t):
        """None while all is well; else one sentence saying what is wrong."""
        self._trim(t)
        if len(self.cmd) < 5 or self.cmd[0][0] > t - WINDOW_S + 0.5:
            return None                       # not commanding for a whole window yet
        big = [max(abs(vx - wz * HALF_TRACK), abs(vx + wz * HALF_TRACK)) >= MIN_CMD for _, vx, wz in self.cmd]
        if sum(big) < BUSY * len(big):
            return None
        if not self.state:
            return f'no /wheel_state for {WINDOW_S:.0f} s while driving: the ESP32 data does not arrive (Pi 5 / micro-ROS)'
        if any(abs(l) > MOVING or abs(r) > MOVING for _, l, r, _ in self.state):
            return None
        asked_vx = [abs(vx) >= MIN_CMD for _, vx, _ in self.cmd]
        echo = max(abs(e) for *_, e in self.state)
        if sum(asked_vx) >= BUSY * len(asked_vx) and echo < ECHO_ZERO:
            return (f'wheels not responding: drive commands do not reach the ESP32 for {WINDOW_S:.0f} s '
                    f'(its echo of vx is 0) -- after a Pi 5 restart wait ~2 min, check ./rover wheels')
        return (f'wheels not turning for {WINDOW_S:.0f} s although commanded (encoders read 0) '
                f'-- motor power off, or the rover is stuck')
