ARG SDK_IMAGE=go2-ros2-sdk:edu
FROM ${SDK_IMAGE}
USER root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -o APT::Update::Error-Mode=any && apt-get install -y --no-install-recommends \
    python3-open3d python3-yaml python3-numpy \
    ros-humble-sensor-msgs-py ros-humble-tf2-ros \
    ros-humble-nav2-bringup ros-humble-nav2-costmap-2d ros-humble-pointcloud-to-laserscan \
    ros-humble-robot-state-publisher ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*
COPY go2_3d/check_nav2_plugins.py /opt/go2/check_nav2_plugins.py
RUN . /opt/ros/humble/setup.sh && python3 /opt/go2/check_nav2_plugins.py
ENV OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
WORKDIR /opt/go2_project
