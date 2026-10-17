#!/usr/bin/env python3
"""Watchdog decisions without a robot:  python3 ops/test_watchdog.py"""
import importlib.util
import pathlib

spec = importlib.util.spec_from_file_location('wd', pathlib.Path(__file__).parent / 'rover_watchdog.py')
wd = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wd)
wd.LOG = pathlib.Path('/tmp/wd_test.jsonl')

OK = {t: 99.0 for _, t, _ in wd.CHECKS}


def test_judge():
    hz = dict(OK, **{'/scan': 0.0, '/odom': 3.0})
    L = wd.judge(hz)
    assert L['lidar']['status'] == 'down' and L['fused']['status'] == 'low' and L['camera']['status'] == 'ok'


def test_restart_is_limited_and_pose_gets_slam_false(monkeypatch=None):
    calls = []
    wd.subprocess.run = lambda cmd, **kw: calls.append((cmd, kw.get('env', {}).get('SLAM'))) or type('R', (), {'returncode': 0, 'stdout': 'x'})()
    wd.history.clear()
    assert wd.restart('pose') and wd.restart('pose')
    assert not wd.restart('pose'), 'third restart in the window must be refused'
    assert len(calls) == 2 and calls[0][1] == 'false' and calls[0][0][1] == 'pose'


if __name__ == '__main__':
    test_judge()
    test_restart_is_limited_and_pose_gets_slam_false()
    print('PASS')
