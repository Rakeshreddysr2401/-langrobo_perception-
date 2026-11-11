#!/usr/bin/env bash
# replay_nvblox.sh — build nvblox's map again from a RECORDED real drive, with other settings.
#
#   mapping/tools/replay_nvblox.sh BAG_DIR OUT_DIR "94 159 327" ['{"static_mapper.projective_integrator_max_integration_distance_m": 2.0}']
#
# Why: on the real robot nvblox's obstacle cells were 20-40 cm thick around every wall
# (2026-11-10, kino_sim docs/ROBUSTNESS.md R2); the twin's camera has no stereo noise and never
# shows it. This runs the robot's own depth gate, nvblox launch and ghost clearing in a throwaway
# container (no network, ROS domain 99: the robot is not touched) on a bag's depth + pose, and
# saves the grids at the asked seconds into the bag. Judge them with
# kino_sim tools/nvblox_thickness.py. The bag needs: depth image + camera_info, /tf, /tf_static,
# /odom, /scan. /gyro/base (the turn gate's input) is rebuilt from /odom's turn rate.
set -eo pipefail
BAG=$(cd "$1" && pwd); OUT=$2; TIMES=$3; PARAMS=${4:-\{\}}
IMAGE=${IMAGE:-orin-nav:2.0}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
LAST=$(for t in $TIMES; do echo "$t"; done | sort -n | tail -1)
# every node on the BAG's clock (ros2 bag play --clock): the turn gate compares frame stamps with
# its own start time, and on the wall clock a recorded frame is always "before the start"
PARAMS=$(python3 -c 'import json,sys; d=json.loads(sys.argv[1]); d["use_sim_time"]=True; print(json.dumps(d))' "$PARAMS")
echo "== replay_nvblox  bag $(basename "$BAG")  until $LAST s  params $PARAMS"
docker run --rm --name nvblox_replay --runtime nvidia --network none --entrypoint bash \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_DISABLE_REQUIRE=1 \
  -e ROS_DOMAIN_ID=99 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e NVBLOX_PARAMS="$PARAMS" -e TIMES="$TIMES" -e LAST="$LAST" \
  -v "$REPO/mapping":/opt/rover/mapping:ro -v "$BAG":/bag:ro -v "$OUT":/out "$IMAGE" -c '
for f in /opt/ros/*/setup.bash; do source $f; done
python3 /opt/rover/mapping/tools/replay_nvblox_helper.py gyro > /out/gyro.log 2>&1 &
python3 /opt/rover/mapping/nodes/depth_gate.py --ros-args -p use_sim_time:=true > /out/depth_gate.log 2>&1 &
ros2 launch /opt/rover/mapping/launch/nvblox.launch.py > /out/nvblox.log 2>&1 &
python3 /opt/rover/mapping/nodes/ghost_clear_node.py --ros-args -p use_sim_time:=true > /out/ghost_clear.log 2>&1 &
python3 /opt/rover/mapping/tools/replay_nvblox_helper.py snap > /out/snap.log 2>&1 &
SNAP=$!
sleep 12      # nvblox allocates its GPU tables before it takes a frame
ros2 bag play /bag --clock 100 --playback-duration $((LAST + 4)) --topics /tf /tf_static /odom /scan \
  /camera/camera0/depth/image_rect_raw /camera/camera0/depth/camera_info > /out/play.log 2>&1
sleep 3; kill -INT $SNAP 2>/dev/null; sleep 1
grep -c . /out/snap.log >/dev/null && tail -6 /out/snap.log
'
ls -la "$OUT"/*.npz 2>/dev/null | awk '{print "  " $5, $9}'
