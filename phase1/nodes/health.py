#!/usr/bin/env python3
"""One line of truth about whether the pose is real.

`ros2 topic echo /fusion/status --once` TRUNCATES the JSON payload -- it prints
`"vo_implausible":...` and stops -- so anything that greps its output silently
reads nothing. This subscribes properly instead.

The question it answers cannot be answered by rates. cuVSLAM has diverged three
times (TODO 21) and the fallback is silent: the rover drops to gyro-plus-wheel
dead reckoning while /odom stays at 20 Hz, `ready` stays true, `vo_alive` stays
true and every gate stays green. One session ran 21 m that way unnoticed.
"""
import json
import sys
import time

import rclpy
from std_msgs.msg import String

VO_Z_LIMIT = 0.30       # a ground rover is not this far above or below the floor


def main():
    rclpy.init()
    n = rclpy.create_node('health')
    got = []
    n.create_subscription(String, '/fusion/status',
                          lambda m: got.append(json.loads(m.data)), 10)
    end = time.time() + 12.0
    while time.time() < end and len(got) < 4:
        rclpy.spin_once(n, timeout_sec=0.1)

    if not got:
        print('  cuvslam   no /fusion/status — run ./rover fused')
        if rclpy.ok():
            rclpy.shutdown()
        return 1

    a, b = got[0], got[-1]
    z = float(b.get('vo_z', 0.0))
    imp = int(b.get('vo_implausible', 0))
    dr = float(b.get('dr_metres', 0.0))
    lm = int(b.get('landmarks', -1))
    climbing = imp > int(a.get('vo_implausible', 0))

    print('  cuvslam   vo_z %+.3f m   implausible %d   dead-reckoned %.2f m   '
          'landmarks %d' % (z, imp, dr, lm))

    if abs(z) > VO_Z_LIMIT or climbing:
        print('            ✗ DIVERGED — the pose is dead reckoning, and every rate '
              'above still reads OK.')
        print('              Fix: ./rover pose  THEN  ./rover fused.')
        print('              ./rover fused alone restarts fusion and NOT cuvslam, '
              'so the divergence survives.')
        print('              Resetting the pose invalidates the map — expect to '
              'redrive a lap.')
        rc = 1
    elif imp > 0:
        print('            ! %d poses were rejected earlier but it is steady now.' % imp)
        rc = 0
    else:
        print('            ok — tracking, not dead reckoning')
        rc = 0

    if 0 <= lm < 30:
        print('            ! only %d landmarks — tracking is fragile here. '
              'Drive where there is texture.' % lm)
    if rclpy.ok():
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
