ARG SDK_IMAGE=go2-ros2-sdk:edu
FROM ${SDK_IMAGE}
USER root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -o APT::Update::Error-Mode=any && apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions \
    python3-open3d python3-yaml python3-numpy python3-opencv \
    ros-humble-sensor-msgs-py ros-humble-tf2-ros \
    ros-humble-nav2-bringup ros-humble-nav2-costmap-2d ros-humble-nav2-controller \
    ros-humble-nav2-rotation-shim-controller ros-humble-pointcloud-to-laserscan \
    ros-humble-robot-state-publisher ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*
COPY navigation/go2_navigation /opt/go2_navigation_src/go2_navigation
RUN . /opt/ros/humble/setup.sh && \
    CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon --log-base /tmp/go2_nav_logs build \
      --base-paths /opt/go2_navigation_src --build-base /tmp/go2_nav_build \
      --install-base /opt/go2_navigation --merge-install --parallel-workers 1 \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /tmp/go2_nav_build /tmp/go2_nav_logs
COPY go2_3d/check_nav2_plugins.py /opt/go2/check_nav2_plugins.py
RUN . /opt/ros/humble/setup.sh && . /opt/go2_navigation/setup.sh && python3 /opt/go2/check_nav2_plugins.py
ENV OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
WORKDIR /opt/go2_project
