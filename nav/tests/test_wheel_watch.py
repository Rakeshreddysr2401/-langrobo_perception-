#!/usr/bin/env python3
"""wheel_watch: dead wheels are named, normal driving and small near-contact commands are not."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import wheel_watch as WW  # noqa: E402


def run(cmd, wheels, secs=4.0):
    w = WW.WheelWatch()
    t = 0.0
    while t < secs:
        w.on_cmd(t, *cmd)
        if wheels is not None:
            w.on_state(t, *wheels(t))
        t += 0.05
    return w.verdict(t)


def test_normal_driving_is_fine():
    assert run((0.15, 0.3), lambda t: (0.10, 0.20, 0.15)) is None


def test_the_22_18_case_commands_lost():
    """rl2 asked -0.10 m/s +0.5 rad/s; wheels 0; the firmware echo 0 -> commands do not arrive"""
    why = run((-0.10, 0.5), lambda t: (0.0, 0.0, 0.0))
    assert why and 'do not reach the ESP32' in why, why


def test_wheels_not_turning_with_commands_arriving():
    why = run((0.15, 0.0), lambda t: (0.0, 0.0, 0.15))
    assert why and 'not turning' in why, why


def test_no_wheel_data_at_all():
    why = run((0.15, 0.0), None)
    assert why and 'no /wheel_state' in why, why


def test_a_spin_in_place_with_dead_wheels():
    """vx 0, wz 0.8: no vx to echo, still dead wheels"""
    why = run((0.0, 0.8), lambda t: (0.0, 0.0, 0.0))
    assert why and 'not turning' in why, why


def test_small_near_contact_commands_are_not_dead_wheels():
    """2026-11-15 08:38: the shield's chatter gave ~0.04 m/s on a wheel -- below MIN_CMD"""
    assert run((-0.03, 0.05), lambda t: (0.0, 0.0, 0.0)) is None


def test_wheels_that_start_late_are_fine():
    """the motor lag (0.4-0.9 s): turning after 1 s is not dead"""
    assert run((0.15, 0.0), lambda t: ((0.12, 0.12, 0.15) if t > 1.0 else (0.0, 0.0, 0.15))) is None


def test_a_short_command_is_not_judged():
    assert run((0.15, 0.0), lambda t: (0.0, 0.0, 0.0), secs=2.0) is None


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok  ', name)
    print('ALL PASS')
