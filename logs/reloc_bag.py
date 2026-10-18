# reloc_bag.py <bag dir>: relocalize real scans (every ~20 s after the last SLAM restart) against the
# bag's final /map, and compare with SLAM's own map -> base_link at that moment.
import sys, math, time
sys.path.insert(0, '/r/phase2/tools')
import numpy as np
import relocalize as RL
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import tf2_py
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.duration import Duration

bag = sys.argv[1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=['/map', '/scan', '/tf', '/tf_static']))
buf = tf2_py.BufferCore(Duration(seconds=7200).to_msg())   # the whole bag: the default 10 s would evict it
maps, scans = [], []
cls = {k: get_message(v) for k, v in types.items() if k in ('/map', '/scan', '/tf', '/tf_static')}
n = 0
while r.has_next():
    topic, data, t = r.read_next()
    m = deserialize_message(data, cls[topic])
    if topic in ('/tf', '/tf_static'):
        for tr in m.transforms:
            (buf.set_transform_static if topic == '/tf_static' else buf.set_transform)(tr, 'bag')
    elif topic == '/map':
        maps.append((t, m))
    elif topic == '/scan':
        n += 1
        if n % 200 == 0:                       # 10 Hz -> one every 20 s
            # keep the scan + SLAM's pose for it once TF has caught up (look up later)
            scans.append((t, m))
print('maps', len(maps), 'scans kept', len(scans))
# the final map; scans after the last time the map got SMALLER (a SLAM restart)
sizes = [(t, mm.info.width * mm.info.height) for t, mm in maps]
restart = max([t for (t, s), (t0, s0) in zip(sizes[1:], sizes[:-1]) if s < s0] or [sizes[0][0]])
tmap, mmsg = maps[-1]
occ, free, res, ox, oy = RL.grid_from_msg(mmsg)
print(f'final map {mmsg.info.width}x{mmsg.info.height} @ {res} m; using scans after the SLAM restart')
ok = tot = 0
for t, s in scans:
    if t < restart + 20e9:
        continue
    stamp = s.header.stamp
    try:
        lp = buf.lookup_transform_core('base_link', s.header.frame_id, stamp)
        tb = buf.lookup_transform_core('map', 'base_link', stamp)
    except Exception as e:
        continue
    q = lp.transform.rotation; lyaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    pts = RL.scan_points(s.ranges, s.angle_min, s.angle_increment, (lp.transform.translation.x, lp.transform.translation.y, lyaw))
    q = tb.transform.rotation; truth = (tb.transform.translation.x, tb.transform.translation.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    t0 = time.time(); out = RL.relocalize(occ, free, res, ox, oy, pts); dt = time.time() - t0
    tot += 1
    if out['pose']:
        e = math.hypot(out['pose'][0] - truth[0], out['pose'][1] - truth[1]); a = math.degrees(RL._wrap(out['pose'][2] - truth[2]))
    else:
        e, a = float('nan'), float('nan')
    good = out['verdict'] == 'locked' and e < 0.10 and abs(a) < 5
    wrong = out['verdict'] == 'locked' and not good
    ok += good
    print(f"  slam ({truth[0]:+.2f},{truth[1]:+.2f},{math.degrees(truth[2]):+5.0f}) -> {out['verdict']:9s} err {100*e:5.1f} cm {a:+5.1f} deg  {out['why']}  {dt:.1f}s" + ('   <<< WRONG LOCK' if wrong else ''))
print(f'locked correctly {ok}/{tot}')
