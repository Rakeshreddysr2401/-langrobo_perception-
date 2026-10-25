#!/usr/bin/env bash
# mps.sh -- the host's CUDA MPS daemon (ISAAC_ROS_5_UPGRADE.md B1): lets the rover container's GPU
# users (cuVSLAM, nvblox, YOLO) share the Orin's GPU without time-slicing contexts.
#   ops/mps.sh start | stop | status       (needs sudo; the rover uses it with MPS=1 ./rover up)
# Same as NVIDIA's isaac_ros_gpu_partitioning/start_cuda_mps.sh, on the host. Pipe: /tmp/nvidia-mps.
set -uo pipefail
export CUDA_MPS_PIPE_DIRECTORY=/tmp/nvidia-mps CUDA_MPS_LOG_DIRECTORY=/tmp/nvidia-mps-log
case "${1:-status}" in
  start) pgrep -f "^nvidia-cuda-mps-control" >/dev/null && { echo "MPS already running"; exit 0; }
         sudo mkdir -p "$CUDA_MPS_PIPE_DIRECTORY" "$CUDA_MPS_LOG_DIRECTORY"
         sudo -E nvidia-cuda-mps-control -d && echo "MPS started (pipe $CUDA_MPS_PIPE_DIRECTORY)" ;;
  stop)  echo quit | sudo -E nvidia-cuda-mps-control && echo "MPS stopped" ;;
  status) if pgrep -f "^nvidia-cuda-mps-control" >/dev/null; then
            echo "MPS running; servers: $(pgrep -c -f nvidia-cuda-mps-server)"; sudo tail -3 "$CUDA_MPS_LOG_DIRECTORY/control.log"
          else echo "MPS not running"; fi ;;
  *) echo "usage: $0 start|stop|status"; exit 2 ;;
esac
