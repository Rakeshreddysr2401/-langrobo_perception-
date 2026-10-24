# syntax=docker/dockerfile:1
# orin-nav:2.0 -- the rover's image on Isaac ROS 5.0 / ROS 2 Lyrical, Jetson Orin (JetPack 7.2).
#
# A real recipe (orin-nav:1.x were `docker commit` layers). Build on the Jetson:
#   cd ~/rover/docker/2.0 && docker build -t orin-nav:2.0 .
# Only what the rover runs goes in. Built from source on top: librealsense with DDS (the
# D555 is a PoE/DDS camera; apt's librealsense2 is not built with DDS) and realsense-ros.
# Our own C++ bits (sllidar_ros2, phase3/plugins) stay host-mounted workspaces built by
# lidar/build.sh and phase3/plugins/build.sh, as before.
# Repos and base image follow isaac-ros-cli 2.6.0's Dockerfile.isaac_ros (release-5.0).
FROM nvcr.io/nvidia/base/ubuntu:noble-20251013

ENV DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash \
    ROS_DISTRO=lyrical \
    ROS_ROOT=/opt/ros/lyrical \
    PATH=/usr/local/cuda/bin:/usr/src/tensorrt/bin:${PATH}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# 1. Apt sources: Isaac ROS 5.0 (Jetson), NVIDIA's ROS Lyrical buildfarm for noble,
#    CUDA, Jetson r39.2, packages.ros.org (colcon, rosdep only). isaac-ros-cli with ISAAC_ROS_PLATFORM=arm64-jetpack writes the
#    official apt pins (CUDA 13.2, TensorRT 10.16) -- that is its only job here.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg lsb-release wget \
    && k=/usr/share/keyrings/nvidia-isaac-ros.gpg \
    && curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor -o $k \
    && echo "deb [signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/release-5.0 noble-jetpack main" \
         > /etc/apt/sources.list.d/nvidia-isaac-ros.list \
    && echo "deb [arch=arm64 signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/ubuntu/main noble main" \
         > /etc/apt/sources.list.d/isaac-ros-buildfarm.list \
    && wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/sbsa/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb && rm cuda-keyring_1.1-1_all.deb \
    && j=/usr/share/keyrings/nvidia-jetson.gpg \
    && curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc | gpg --dearmor -o $j \
    && echo "deb [signed-by=$j] http://repo.download.nvidia.com/jetson/common r39.2 main" \
         > /etc/apt/sources.list.d/nvidia-jetson.list \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
         https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.noble_all.deb \
    && dpkg -i /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb \
    && apt-get update \
    && ISAAC_ROS_PLATFORM=arm64-jetpack apt-get install -y --no-install-recommends isaac-ros-cli \
    && rm -rf /var/lib/apt/lists/*

# 2. ROS Lyrical + the Isaac ROS 5.0 packages the rover runs + python deps.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git pkg-config \
        python3-colcon-common-extensions python3-rosdep python3-pip \
        ros-lyrical-ros-base \
        ros-lyrical-isaac-ros-cuvslam ros-lyrical-isaac-ros-visual-slam-interfaces \
        ros-lyrical-nvblox-ros ros-lyrical-nvblox-msgs \
        ros-lyrical-navigation2 ros-lyrical-slam-toolbox \
        ros-lyrical-robot-state-publisher ros-lyrical-tf2-ros \
        ros-lyrical-cv-bridge ros-lyrical-image-transport ros-lyrical-rosbag2-py \
        ros-lyrical-diagnostic-updater ros-lyrical-xacro \
        tensorrt python3-libnvinfer \
        python3-numpy python3-scipy python3-opencv python3-lz4 python3-yaml \
        libssl-dev libusb-1.0-0-dev libudev-dev \
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /opt/ros/lyrical/setup.bash" >> /etc/bash.bashrc

# 3. librealsense with DDS, the flags orin-nav:1.2 was built with (its CMakeCache):
#    DDS on, RSUSB backend, Release, no CUDA, no python, tools on, no examples.
ARG LIBREALSENSE_VERSION=v2.58.2
RUN git clone --depth 1 -b ${LIBREALSENSE_VERSION} https://github.com/IntelRealSense/librealsense /tmp/librealsense \
    && cmake -S /tmp/librealsense -B /tmp/librealsense/build \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
        -DBUILD_WITH_DDS=ON -DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_CUDA=OFF \
        -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_TOOLS=ON -DBUILD_UNIT_TESTS=OFF \
    && cmake --build /tmp/librealsense/build -j4 \
    && cmake --install /tmp/librealsense/build \
    && rm -rf /tmp/librealsense
ENV LD_LIBRARY_PATH=/opt/librealsense/lib

# 4. realsense-ros 4.58.2 against that librealsense. 4.58.2 already has the Kilted-era fixes
#    NVIDIA patches 4.56.3 for (target_link_libraries, tf2 .hpp); it only lacks "lyrical" in its
#    ROS_DISTRO switch -- patches/realsense-ros-4.58.2-lyrical.patch adds it.
ARG REALSENSE_ROS_VERSION=4.58.2
COPY patches/realsense-ros-4.58.2-lyrical.patch /tmp/realsense-ros-lyrical.patch
RUN mkdir -p /opt/realsense_ws/src && cd /opt/realsense_ws/src \
    && git clone --depth 1 -b ${REALSENSE_ROS_VERSION} https://github.com/IntelRealSense/realsense-ros \
    && git -C realsense-ros apply /tmp/realsense-ros-lyrical.patch \
    && source /opt/ros/lyrical/setup.bash \
    && cd /opt/realsense_ws \
    && CMAKE_PREFIX_PATH=/opt/librealsense:$CMAKE_PREFIX_PATH \
       colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log src /tmp/realsense-ros-lyrical.patch \
    && echo "source /opt/realsense_ws/install/setup.bash" >> /etc/bash.bashrc

# 5. GPU sharing (CUDA MPS): lets YOLO run next to cuVSLAM without starving it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-lyrical-isaac-ros-gpu-partitioning \
    && rm -rf /var/lib/apt/lists/*

LABEL org.orin-nav.version="2.0" \
      org.orin-nav.stack="Isaac ROS 5.0 / ROS 2 Lyrical / JetPack 7.2 (Orin)" \
      org.orin-nav.recipe="rover/docker/2.0/Dockerfile"
