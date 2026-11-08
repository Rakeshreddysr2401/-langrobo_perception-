"""Bounded RL re-entry only after recovery has provided a new physical context.

Does not change collision checks or authorize motion. The caller still enforces
sensor freshness, the original controller deadline and all existing shields.
"""
import math

MIN_MOVE_M = .25
MIN_WAIT_S = 5.0


def allowed(rounds, stops, pose, now, max_rounds=3):
    """stops: prior round-end (x, y, monotonic_time), not goal distance."""
    if rounds >= max_rounds or pose is None:
        return False
    if not all(math.isfinite(float(v)) for v in (*pose[:2], now)):
        return False
    if rounds == 0:
        return True
    if not stops or now - stops[-1][2] < MIN_WAIT_S:
        return False
    # A retreat is useful too; direct distance-to-goal need not decrease.
    # Never return to any previous stalled location within this goal.
    return all(math.hypot(pose[0]-x, pose[1]-y) >= MIN_MOVE_M
               for x, y, _ in stops)
