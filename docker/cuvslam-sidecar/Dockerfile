# cuVSLAM sidecar for Jetson Orin Nano on JetPack 7.2.
# Isaac ROS 4.x ships Thor-only (ARMv9/SVE) binaries, so cuVSLAM comes from
# Isaac ROS 3.2.6 (Humble/Ubuntu 22.04) — the last release with an Orin build.
# CUDA 12 user-space libs come from pip; the JP7.2 host driver runs them fine.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg2 locales python3-pip tzdata \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble apt repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list

# NVIDIA Isaac ROS release-3 repo (component release-3.0 carries 3.2.6)
COPY nvidia-isaac-ros.gpg /usr/share/keyrings/nvidia-isaac-ros.gpg
RUN echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0" \
        > /etc/apt/sources.list.d/nvidia-isaac-ros.list

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-isaac-ros-visual-slam \
        libegl1 libgles2 \
    && rm -rf /var/lib/apt/lists/*

# VPI 3 from the JetPack 6 (L4T r36.4) repo — needed by the NITROS image chain.
RUN curl -sSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
        -o /usr/share/keyrings/jetson-ota.asc \
    && echo "deb [signed-by=/usr/share/keyrings/jetson-ota.asc] https://repo.download.nvidia.com/jetson/common r36.4 main" \
        > /etc/apt/sources.list.d/jetson-r36.list \
    && apt-get update && apt-get install -y --no-install-recommends libnvvpi3 cuda-nvtx-12-6 \
    && ln -s /usr/local/cuda-12.6/targets/aarch64-linux/lib/libnvToolsExt.so /usr/lib/aarch64-linux-gnu/libnvToolsExt.so.1 \
    && rm -rf /var/lib/apt/lists/*

# GPU mount hook trigger for the NVIDIA container runtime
ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

# CUDA 12 user-space (cudart/cublas/cusolver + their deps) — validated on the
# JP7.2 (CUDA 13 era) driver already.
RUN pip3 install --no-cache-dir \
        nvidia-cuda-runtime-cu12 nvidia-cublas-cu12 nvidia-cusolver-cu12 \
        nvidia-cusparse-cu12 nvidia-nvjitlink-cu12 nvidia-nvtx-cu12

ENV CU12=/usr/local/lib/python3.10/dist-packages/nvidia
ENV LD_LIBRARY_PATH=${CU12}/cuda_runtime/lib:${CU12}/cublas/lib:${CU12}/cusolver/lib:${CU12}/cusparse/lib:${CU12}/nvjitlink/lib

# UDP-only DDS transport: the jazzy perception container has private IPC, so
# FastDDS shared memory cannot cross the container boundary.
COPY udp_only.xml /udp_only.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/udp_only.xml

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
