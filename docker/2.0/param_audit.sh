#!/usr/bin/env bash
# param_audit.sh -- which of OUR nav2.yaml parameters does the running nav2 silently ignore?
# A renamed or moved parameter (nav2 1.5: MPPI path settings -> PathHandler) is not an error in
# ROS 2: the node just runs on its default. This lists, per node, every key we set that the live
# node does not have. Run with nav2 up:
#   ~/rover/docker/2.0/param_audit.sh          (real robot, domain 0, container rover)
#   TWIN=1 ~/rover/docker/2.0/param_audit.sh   (twin, domain 42, container rover_twin)
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [ "${TWIN:-0}" = 1 ]; then D=42; CT=rover_twin; else D=0; CT=rover; fi
E="unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; export ROS_DOMAIN_ID=$D; for f in /opt/ros/*/setup.bash; do source \$f; done;"
docker exec -i "$CT" bash -lc "$E python3 - /opt/rover3/config/nav2.yaml" <<'PY' 2>&1 | grep -v XMLPARSER
import sys, yaml, rclpy
from rcl_interfaces.srv import ListParameters
rclpy.init(); n = rclpy.create_node('param_audit')
ours = yaml.safe_load(open(sys.argv[1]))
def flat(d, p=''):
    o = []
    for k, v in (d or {}).items():
        o += flat(v, p + k + '.') if isinstance(v, dict) else [p + k]
    return o
targets = []
for top, v in ours.items():
    if 'ros__parameters' in v: targets.append(('/' + top, v['ros__parameters']))
    else:
        for sub, w in v.items():
            if isinstance(w, dict) and 'ros__parameters' in w: targets.append((f'/{top}/{sub}', w['ros__parameters']))
bad = 0
for node, params in targets:
    c = n.create_client(ListParameters, node + '/list_parameters')
    if not c.wait_for_service(timeout_sec=3):
        print(f'  {node}: not running'); continue
    f = c.call_async(ListParameters.Request(depth=0)); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    live = set(f.result().result.names) if f.result() else set()
    miss = sorted(k for k in flat(params) if k not in live and k != 'use_sim_time')
    bad += len(miss)
    print(f'  {node}: {len(flat(params))} set, {len(miss)} ignored' + (': ' + ', '.join(miss) if miss else ''))
print(f'== {bad} ignored in total')
PY
