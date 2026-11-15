import time, rclpy
from tf2_msgs.msg import TFMessage
rclpy.init(); n = rclpy.create_node('disc_probe'); t0 = time.time()
got = {}
def cb(m):
    for t in m.transforms:
        got.setdefault(f'{t.header.frame_id}->{t.child_frame_id}', round(time.time() - t0, 1))
n.create_subscription(TFMessage, '/tf', cb, 100)
last = None
while time.time() - t0 < 75:
    rclpy.spin_once(n, timeout_sec=0.2)
    pubs = sorted(i.node_name for i in n.get_publishers_info_by_topic('/tf'))
    row = (len(n.get_node_names()), tuple(pubs), n.count_publishers('/camera/camera0/infra1/camera_info'), n.count_publishers('/scan'), n.count_publishers('/map'))
    if row != last:
        print(f'{time.time()-t0:5.1f}s nodes={row[0]:3d} tf_pubs={list(row[1])} camera_info={row[2]} scan={row[3]} map={row[4]}', flush=True); last = row
print('first /tf message per transform (s):', got)
