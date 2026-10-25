#!/usr/bin/env bash
# topic_audit.sh -- topics whose publishers and subscribers disagree on the TYPE. In ROS 2 a
# TwistStamped publisher and a Twist subscriber on one topic never connect and nothing errors:
# nav2 1.5 switched to TwistStamped by default and every nav2 command died at /cmd_vel_smoothed
# (twin, 2026-10-25). Run with the stack up:
#   ~/rover/docker/2.0/topic_audit.sh          (real robot, domain 0)
#   TWIN=1 ~/rover/docker/2.0/topic_audit.sh   (twin, domain 42)
# Also lists topics with publishers but no subscribers among the command/sensor chain.
set -uo pipefail
if [ "${TWIN:-0}" = 1 ]; then D=42; CT=rover_twin; else D=0; CT=rover; fi
E="unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; export ROS_DOMAIN_ID=$D; for f in /opt/ros/*/setup.bash; do source \$f; done;"
docker exec -i "$CT" bash -lc "$E python3 -" <<'PY' 2>&1 | grep -v XMLPARSER
import time, rclpy
rclpy.init(); n = rclpy.create_node('topic_audit'); time.sleep(3)
bad = 0
CHAIN = ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel_turn', '/cmd_vel', '/scan', '/odom')
for name, _ in sorted(n.get_topic_names_and_types()):
    if '/_buf/' in name:   # Lyrical's internal rosidl::Buffer side topics
        continue
    pubs = n.get_publishers_info_by_topic(name); subs = n.get_subscriptions_info_by_topic(name)
    tp = {p.topic_type for p in pubs}; ts = {s.topic_type for s in subs}
    if len(tp | ts) > 1:
        bad += 1
        print(f'  MISMATCH {name}: publishers {sorted(tp)} ({", ".join(p.node_name for p in pubs)}) '
              f'vs subscribers {sorted(ts)} ({", ".join(s.node_name for s in subs)})')
    if name in CHAIN and pubs and not subs:
        print(f'  no subscriber: {name} ({", ".join(p.node_name for p in pubs)})')
print(f'== {bad} type mismatches')
PY
