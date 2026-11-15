#!/usr/bin/env python3
"""Watchdog decisions without a robot:  python3 ops/tests/test_watchdog.py"""
import importlib.util
import pathlib

spec = importlib.util.spec_from_file_location('wd', pathlib.Path(__file__).resolve().parent.parent / 'rover_watchdog.py')
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


def test_a_probe_line_is_judged_once_and_a_silent_probe_is_replaced():
    import time
    started, stopped = [], []
    fake = type('P', (), {'poll': lambda self: None, 'stdout': [], 'terminate': lambda self: stopped.append(1)})
    wd.subprocess.Popen = lambda *a, **kw: started.append(1) or fake()
    wd.subprocess.run = lambda *a, **kw: None
    wd.probe.update(proc=None, started=0.0, line_t=0.0, data=None, judged_t=0.0, refreshed=0.0)
    assert wd.probe_latest() is None and len(started) == 1, 'first call starts the probe, nothing to judge yet'
    wd.probe.update(data={'hz': {}, 'moving': False}, line_t=time.time())
    assert wd.probe_latest() == {'hz': {}, 'moving': False}
    assert wd.probe_latest() is None and len(started) == 1, 'the same line is not judged twice'
    wd.probe.update(started=time.time() - wd.PROBE_SILENT_S - 1, line_t=0.0)
    try:
        wd.probe_latest()
        raise AssertionError('a silent probe must be reported')
    except RuntimeError:
        pass
    assert len(started) == 2 and stopped, 'and replaced by a new one'


def test_a_silent_topic_gets_one_fresh_probe_then_is_believed():
    import time
    stops = []
    wd.container_up = lambda: True
    wd.pi5_status = lambda: {'reachable': True, 'manual': False}
    wd.probe_stop = lambda: stops.append(1)
    line = {'hz': dict(OK, **{'/scan': 0.0}), 'moving': False, 'silent': ['/scan']}
    wd.probe_latest = lambda: line
    wd.probe['refreshed'] = 0.0
    wd.state['layers'] = {}
    wd.step()
    assert stops == [1] and wd.state['layers'] == {}, 'first time: a fresh probe, the line is not judged'
    wd.step()
    assert stops == [1] and wd.state['layers']['lidar']['status'] == 'down', 'the fresh probe says the same: believed'


if __name__ == '__main__':
    test_judge()
    test_restart_is_limited_and_pose_gets_slam_false()
    test_a_probe_line_is_judged_once_and_a_silent_probe_is_replaced()
    test_a_silent_topic_gets_one_fresh_probe_then_is_believed()      # last: it replaces wd.probe_latest
    print('PASS')
