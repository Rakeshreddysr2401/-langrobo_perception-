#!/usr/bin/env python3
"""Keep the slam_toolbox pose graph on disk so a power cycle does not lose the room.

    python3 map_autosave.py [period_s]        (run by ./rover slam, in the container)

Every period_s it asks slam_toolbox to serialize its graph into /maps/.autosave
and, only if that worked, moves it over /maps/last.{posegraph,data}. The
previous last.* is kept as last.prev.* first, so one bad save (or a map
corrupted by resuming from the wrong place) can be undone by hand:

    cd maps && for e in posegraph data; do cp last.prev.$e last.$e; done

`./rover slam` loads last.* on its next start (SLAM_FRESH=1 ignores it) and
starts from last.pose: where the rover stood when it was saved (map frame).
Writes go to a temp name and are renamed, so a power cut mid-save leaves the
old good files, not half of a new one.
"""
import math
import os
import shutil
import subprocess
import sys
import time

MAPS = '/maps'
EXTS = ('posegraph', 'data')


def robot_pose(timeout=5.0):
    """x,y,theta of base_link in the map frame, or None."""
    import rclpy
    import tf2_ros
    from rclpy.time import Time
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('map_autosave_pose')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    t0 = time.time()
    try:
        while time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                t = buf.lookup_transform('map', 'base_link', Time()).transform
            except Exception:  # noqa: BLE001 - not there yet
                continue
            q = t.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return t.translation.x, t.translation.y, yaw
        return None
    finally:
        node.destroy_node()


def save_once():
    # Where the rover stands WHEN SAVED is where it will most likely stand when
    # it wakes (power-off in place): the next start resumes from this pose.
    pose = robot_pose()
    if pose is None:
        return False, 'no map -> base_link transform'
    tmp = f'{MAPS}/.autosave'
    cmd = ['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
           'slam_toolbox/srv/SerializePoseGraph', f"{{filename: '{tmp}'}}"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    if 'result=0' not in r.stdout.replace(' ', '').replace('\n', ''):
        # ResultSUCCESS is 0 for SerializePoseGraph; anything else, keep the old files
        return False, r.stdout.strip()[-200:] or r.stderr.strip()[-200:]
    if not all(os.path.getsize(f'{tmp}.{e}') > 0 for e in EXTS):
        return False, 'empty output'
    for e in EXTS:
        if os.path.exists(f'{MAPS}/last.{e}'):
            shutil.copy2(f'{MAPS}/last.{e}', f'{MAPS}/last.prev.{e}')
    for e in EXTS:
        os.replace(f'{tmp}.{e}', f'{MAPS}/last.{e}')
    with open(f'{MAPS}/.pose.tmp', 'w') as f:
        f.write('%.3f,%.3f,%.3f\n' % pose)
    os.replace(f'{MAPS}/.pose.tmp', f'{MAPS}/last.pose')
    return True, ''


def main():
    period = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    while True:
        time.sleep(period)
        try:
            ok, why = save_once()
        except Exception as e:  # noqa: BLE001 - keep trying; a missed save is not fatal
            ok, why = False, repr(e)
        print(time.strftime('%H:%M:%S'), 'saved last' if ok else f'save failed: {why}', flush=True)


if __name__ == '__main__':
    main()
