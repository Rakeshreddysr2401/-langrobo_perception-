#!/usr/bin/env python3
"""clock_check's verdict without a robot:  python3 tools/tests/test_clock_check.py"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import clock_check as c  # noqa: E402


def test_verdict():
    assert c.verdict([])[0] == 2, 'no sonar message is "not checked", not a pass'
    assert c.verdict([0.012, 0.080, 0.009])[0] == 0, 'a few ms of WiFi delay is agreement'
    code, text = c.verdict([0.9, 0.85, 1.4])
    assert code == 1 and 'behind' in text, 'Pi 5 stamps older than our clock: it is behind'
    code, text = c.verdict([-3.0, -2.9])
    assert code == 1 and 'ahead' in text
    assert c.verdict([1.79e9])[0] == 1, 'one machine still in 1970'


if __name__ == '__main__':
    test_verdict()
    print('PASS')
