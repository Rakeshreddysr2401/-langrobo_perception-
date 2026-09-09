#!/usr/bin/env python3
"""One line of truth about whether the pose is real.

`ros2 topic echo /fusion/status --once` TRUNCATES the JSON payload -- it prints
`"vo_implausible":...` and stops -- so anything that greps its output silently
reads nothing. This subscribes properly instead.

The question it answers cannot be answered by rates. cuVSLAM has diverged three
times (TODO 21) and the fallback is silent: the rover drops to gyro-plus-wheel
dead reckoning while /odom stays at 20 Hz, `ready` stays true, `vo_alive` stays
true and every gate stays green. One session ran 21 m that way unnoticed.

There are TWO silent failures here, and they need different fields (TODO 30):

  DIVERGED     cuVSLAM keeps publishing a confidently wrong pose. `vo_alive`
               stays true -- freshness is fine, the number is a lie. Caught by
               vo_z and vo_implausible.
  NOT RUNNING  cuVSLAM stops publishing entirely. Now vo_z, vo_implausible,
               landmarks and dr_metres are all LAST-KNOWN values, frozen at the
               moment it died, and dr_metres -- the one that would move -- reads
               0.00 until the rover drives. At a standstill a dead tracker is
               indistinguishable from a healthy one on those four. Only
               `vo_alive` catches it.

This checked only the first for a while, and passed a pose with no cuVSLAM node
running at all on 2026-09-09.
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

    # Freshness, sampled across the whole window rather than once: vo drops out
    # for ~1 s at a time under full load (frame gaps past cuVSLAM's
    # max_frame_delta_s -- TODO 31), and a single false reading there is a
    # flicker, not a death. All-false across ~4 s is a death.
    alive = [s['vo_alive'] for s in got if 'vo_alive' in s]

    if alive and not any(alive):
        print('            ✗ NOT RUNNING — no visual odometry reached fusion in '
              'any of %d samples.' % len(alive))
        print('              The numbers above are cuVSLAM\'s LAST values, frozen '
              'when it stopped. They are not evidence of anything.')
        print('              Confirm: ros2 topic hz /vo/odom   (expect no publisher)')
        print('              Fix: ./rover pose  THEN  ./rover fused.')
        print('              Resetting the pose invalidates the map — expect to '
              'redrive a lap.')
        rc = 1
    elif abs(z) > VO_Z_LIMIT or climbing:
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

    if not alive:
        print('            ! this fusion_node does not publish vo_alive — a dead '
              'tracker cannot be detected here. See TODO 30.')
    elif any(alive) and not all(alive):
        print('            ! vo dropped out in %d of %d samples — frame gaps past '
              'max_frame_delta_s (1.0 s) reset tracking. See TODO 31.'
              % (alive.count(False), len(alive)))

    # Only meaningful while it is actually tracking; frozen otherwise.
    if alive and any(alive) and 0 <= lm < 30:
        print('            ! only %d landmarks — tracking is fragile here. '
              'Drive where there is texture.' % lm)
    if rclpy.ok():
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
