#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
# CUDA 12 pip libs must stay ahead of anything setup.bash prepends.
CU12=/usr/local/lib/python3.10/dist-packages/nvidia
GXF_LIB=/opt/ros/humble/share/isaac_ros_gxf/gxf/lib
GXF_PATH=$(ls -d ${GXF_LIB}/*/ 2>/dev/null | tr '\n' ':')
export LD_LIBRARY_PATH=${CU12}/cuda_runtime/lib:${CU12}/cublas/lib:${CU12}/cusolver/lib:${CU12}/cusparse/lib:${CU12}/nvjitlink/lib:${CU12}/nvtx/lib:${GXF_PATH}${LD_LIBRARY_PATH}
exec "$@"
